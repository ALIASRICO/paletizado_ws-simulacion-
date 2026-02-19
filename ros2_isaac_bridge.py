#!/usr/bin/env python3
"""
================================================================================
ROS2-Isaac Sim Bridge para Robot CR20
================================================================================

Este script conecta Isaac Sim con ROS2 Jazzy publicando estados de joints
y escuchando comandos para controlar el robot CR20.

Uso:
    1. Iniciar Isaac Sim con el robot CR20 cargado
    2. En otra terminal:
        source /opt/ros/jazzy/setup.bash
        source ~/dobot_ws/install/setup.bash
        python3 ~/ros2_isaac_bridge.py

Tópics ROS2:
    - /cr20/joint_states (sensor_msgs/JointState) - Estado actual de joints
    - /cr20/joint_commands (std_msgs/Float64MultiArray) - Comandos de posición

================================================================================
"""

import sys
import os
import time
import threading
import numpy as np

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64

# Isaac Sim path
ISAAC_SIM_PATH = "/home/iudc/isaacsim"

# Add Isaac Sim Python packages to path
isaac_python_path = os.path.join(ISAAC_SIM_PATH, "python_packages", "isaacsim")
if os.path.exists(isaac_python_path):
    sys.path.insert(0, isaac_python_path)
    print(f"[Isaac] Agregado: {isaac_python_path}")

# Also try the kit/python path
kit_python = os.path.join(ISAAC_SIM_PATH, "kit", "python")
if os.path.exists(kit_python):
    for item in os.listdir(kit_python):
        item_path = os.path.join(kit_python, item)
        if os.path.isdir(item_path):
            sys.path.insert(0, item_path)
    print(f"[Isaac] Kit Python agregado")

# Global flag for Isaac Sim connection
ISAAC_AVAILABLE = False

try:
    import omni
    import omni.usd
    import omni.isaac.core
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.types import ArticulationAction
    from pxr import Usd, UsdGeom, Gf, Sdf
    ISAAC_AVAILABLE = True
    print("[Isaac] ✓ Módulo Isaac Core cargado exitosamente")
except ImportError as e:
    print(f"[Isaac] Advertencia: No se pudo importar Isaac Sim: {e}")
    print("[Isaac] Ejecutando en modo simulación (sin conexión real)")


class CR20IsaacBridge(Node):
    """
    Nodo ROS2 que conecta Isaac Sim con el robot CR20.
    
    Publica estados de joints desde Isaac Sim y recibe comandos
    para mover el robot.
    """
    
    def __init__(self):
        super().__init__('cr20_isaac_bridge')
        
        self.get_logger().info("Iniciando bridge ROS2-Isaac Sim para CR20")
        
        # Joint configuration for CR20
        self.joint_names = [
            'joint1',   # Base rotation
            'joint2',   # Shoulder
            'joint3',   # Elbow
            'joint4',   # Wrist 1
            'joint5',   # Wrist 2
            'joint6',   # Wrist 3 (flange)
        ]
        
        # Current joint positions and velocities
        self.current_positions = [0.0] * 6
        self.current_velocities = [0.0] * 6
        
        # Isaac Sim connection
        self.isaac_robot = None
        self.isaac_stage = None
        
        # Initialize Isaac Sim connection
        self._init_isaac_sim()
        
        # ROS2 Publishers
        self.joint_states_pub = self.create_publisher(
            JointState,
            '/cr20/joint_states',
            10
        )
        
        # ROS2 Subscribers
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/cr20/joint_commands',
            self.joint_command_callback,
            10
        )
        
        # Individual joint command subscribers (optional)
        for i, joint in enumerate(self.joint_names):
            self.create_subscription(
                Float64,
                f'/cr20/{joint}_position/command',
                lambda msg, idx=i: self.single_joint_callback(msg, idx),
                10
            )
        
        # Timer for publishing joint states (100Hz)
        self.timer = self.create_timer(0.01, self.publish_joint_states)
        
        self.get_logger().info("Bridge inicializado exitosamente")
        self.get_logger().info(f"Tópicos disponibles:")
        self.get_logger().info(f"  - /cr20/joint_states (publica estado)")
        self.get_logger().info(f"  - /cr20/joint_commands (recibe comandos)")
        
        # Show Isaac status
        if self.isaac_robot is not None:
            self.get_logger().info("✓ Conectado a Isaac Sim")
        else:
            self.get_logger().warn("⚠ Modo simulación (sin Isaac Sim)")
    
    def _init_isaac_sim(self):
        """Inicializa conexión con Isaac Sim."""
        if not ISAAC_AVAILABLE:
            self.get_logger().warn("Isaac Sim no disponible, usando modo simulación")
            return
        
        try:
            # Get the stage
            self.isaac_stage = omni.usd.get_context().get_stage()
            
            if self.isaac_stage is None:
                self.get_logger().warn("No se pudo obtener el stage de USD")
                return
            
            # Look for cr20_robot
            robot_path = "/cr20_robot"
            
            # Check if robot exists
            if self.isaac_stage.GetPrimAtPath(robot_path):
                self.get_logger().info(f"Robot encontrado en: {robot_path}")
                
                # Try to get the robot using Isaac Core
                try:
                    world = World.instance()
                    if world is not None:
                        self.isaac_robot = world.scene.get_object("cr20_robot")
                        if self.isaac_robot:
                            self.get_logger().info("Robot CR20 conectado exitosamente")
                            # Get joint names from the robot
                            if hasattr(self.isaac_robot, 'joint_names'):
                                self.joint_names = self.isaac_robot.joint_names
                                self.get_logger().info(f"Joints: {self.joint_names}")
                except Exception as e:
                    self.get_logger().warn(f"No se pudo obtener robot de World: {e}")
                    
                # Alternative: directly read from USD
                self._read_joints_from_usd(robot_path)
            else:
                self.get_logger().warn(f"Robot no encontrado en {robot_path}")
                self.get_logger().info("Buscando robot en el stage...")
                self._search_robot_in_stage()
                
        except Exception as e:
            self.get_logger().error(f"Error inicializando Isaac Sim: {e}")
    
    def _search_robot_in_stage(self):
        """Busca el robot en el stage de USD."""
        if self.isaac_stage is None:
            return
        
        # Iterate through all prims looking for joints
        for prim in self.isaac_stage.Traverse():
            prim_path = prim.GetPath().pathString
            if 'robot' in prim_path.lower() or 'cr20' in prim_path.lower():
                self.get_logger().info(f"Posible robot encontrado: {prim_path}")
    
    def _read_joints_from_usd(self, robot_path):
        """Lee los joints directamente del USD stage."""
        if self.isaac_stage is None:
            return
        
        prim = self.isaac_stage.GetPrimAtPath(robot_path)
        if not prim:
            return
        
        # Look for joint prims
        joints_found = []
        for child_prim in prim.GetChildren():
            if child_prim.IsA(UsdGeom.Joint):
                joint_name = child_prim.GetPath().name
                joints_found.append(joint_name)
        
        if joints_found:
            self.get_logger().info(f"Joints encontrados en USD: {joints_found}")
    
    def publish_joint_states(self):
        """Publica el estado actual de los joints."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Update positions from Isaac Sim if available
        if self.isaac_robot is not None:
            try:
                # Get current joint positions
                joint_positions = self.isaac_robot.get_joint_positions()
                joint_velocities = self.isaac_robot.get_joint_velocities()
                
                if joint_positions is not None:
                    self.current_positions = joint_positions.tolist()
                if joint_velocities is not None:
                    self.current_velocities = joint_velocities.tolist()
                    
            except Exception as e:
                pass  # Keep previous values
        
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = self.current_velocities
        msg.effort = [0.0] * len(self.joint_names)  # Effort not available
        
        self.joint_states_pub.publish(msg)
    
    def joint_command_callback(self, msg: Float64MultiArray):
        """Recibe comandos de posición para todos los joints."""
        if len(msg.data) == 0:
            return
        
        positions = list(msg.data)
        
        # Pad or trim to match joint count
        while len(positions) < len(self.joint_names):
            positions.append(0.0)
        positions = positions[:len(self.joint_names)]
        
        self.get_logger().debug(f"Comando recibido: {positions}")
        
        # Send to Isaac Sim
        self._send_joint_command(positions)
    
    def single_joint_callback(self, msg: Float64, joint_idx: int):
        """Recibe comando para un solo joint."""
        if joint_idx < len(self.current_positions):
            self.current_positions[joint_idx] = msg.data
            self._send_joint_command(self.current_positions)
    
    def _send_joint_command(self, positions):
        """Envia comandos de posición a Isaac Sim."""
        if self.isaac_robot is not None:
            try:
                # Convert to numpy array
                pos_array = np.array(positions, dtype=np.float32)
                
                # Create articulation action
                action = ArticulationAction(joint_positions=pos_array)
                
                # Apply action
                self.isaac_robot.apply_action(action)
                
            except Exception as e:
                self.get_logger().error(f"Error enviando comando a Isaac: {e}")


def main(args=None):
    """Función principal."""
    print("=" * 60)
    print("  ROS2-Isaac Sim Bridge para CR20")
    print("=" * 60)
    print()
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create and spin the node
        bridge = CR20IsaacBridge()
        
        print()
        print("-" * 60)
        print("  Bridge activo. Presiona Ctrl+C para terminar.")
        print("-" * 60)
        print()
        
        # Spin the node
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        print()
        print("Bridge detenido por el usuario")
        
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        # Cleanup
        try:
            rclpy.shutdown()
        except:
            pass
        print("Bridge terminado")


if __name__ == '__main__':
    main()
