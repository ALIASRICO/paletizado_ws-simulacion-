#!/usr/bin/env python3
"""
Nodo ROS2 para simular movimiento de cinta transportadora en Gazebo.
Mueve las cajas linealmente en dirección Y+ y las recicla al final.

Uso:
    ros2 run pruebas_de_vision conveyor_controller --ros-args -p belt_speed:=0.1
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import math
import random
import time
import subprocess
from typing import List, Dict, Tuple, Optional

from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from std_msgs.msg import Header


class ConveyorController(Node):
    """
    Nodo que simula una cinta transportadora moviendo cajas linealmente.
    
    Las cajas se mueven en dirección Y+ y al llegar al final,
    se reposicionan al inicio con nuevas propiedades aleatorias.
    """

    def __init__(self) -> None:
        super().__init__('conveyor_controller')

        # Declarar parámetros
        self.declare_parameter('num_boxes', 5)
        self.declare_parameter('update_rate', 30.0)  # Hz - más alto para movimiento suave
        self.declare_parameter('belt_speed', 0.1)  # m/s
        self.declare_parameter('belt_y_min', -1.3)  # Inicio de la cinta
        self.declare_parameter('belt_y_max', 1.3)   # Final de la cinta
        self.declare_parameter('belt_x_min', -0.25)  # Borde izquierdo
        self.declare_parameter('belt_x_max', 0.25)   # Borde derecho
        self.declare_parameter('belt_height', 0.12)  # Altura de la superficie
        self.declare_parameter('box_prefix', 'cardboard_box_')
        self.declare_parameter('world_name', 'conveyor_belt')

        # Obtener parámetros
        self.num_boxes = self.get_parameter('num_boxes').value
        self.update_rate = self.get_parameter('update_rate').value
        self.belt_speed = self.get_parameter('belt_speed').value
        self.belt_y_min = self.get_parameter('belt_y_min').value
        self.belt_y_max = self.get_parameter('belt_y_max').value
        self.belt_x_min = self.get_parameter('belt_x_min').value
        self.belt_x_max = self.get_parameter('belt_x_max').value
        self.belt_height = self.get_parameter('belt_height').value
        self.box_prefix = self.get_parameter('box_prefix').value
        self.world_name = self.get_parameter('world_name').value

        # Tamaños de cajas disponibles (ancho, largo, alto)
        self.box_sizes = [
            [0.24, 0.18, 0.24],  # Grande
            [0.20, 0.15, 0.20],  # Mediana
            [0.16, 0.12, 0.16],  # Pequeña
            [0.18, 0.18, 0.28],  # Mediana-alta
            [0.30, 0.20, 0.20],  # Grande rectangular
        ]
        
        # Colores de cartón
        self.box_colors = [
            (0.55, 0.35, 0.15),  # Marrón oscuro
            (0.60, 0.40, 0.18),  # Marrón medio
            (0.50, 0.32, 0.12),  # Marrón chocolate
            (0.58, 0.38, 0.16),  # Marrón
            (0.52, 0.33, 0.14),  # Marrón kraft
        ]

        # Estado de cada caja
        self.box_states: Dict[str, Dict] = {}
        self._initialize_box_states()

        # Tiempo inicial
        self.start_time = time.time()
        
        # Contador de movimientos exitosos
        self.success_count = 0
        self.error_count = 0

        # Timer para actualizar movimiento
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.update_movement)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Conveyor Controller iniciado')
        self.get_logger().info(f'  Velocidad de cinta: {self.belt_speed} m/s')
        self.get_logger().info(f'  Número de cajas: {self.num_boxes}')
        self.get_logger().info(f'  Rango Y: [{self.belt_y_min}, {self.belt_y_max}]')
        self.get_logger().info(f'  Update rate: {self.update_rate} Hz')
        self.get_logger().info('=' * 50)

    def _initialize_box_states(self) -> None:
        """Inicializa el estado de cada caja con posición distribuida."""
        # Distribuir cajas a lo largo de la cinta
        y_spacing = (self.belt_y_max - self.belt_y_min) / (self.num_boxes + 1)
        
        for i in range(1, self.num_boxes + 1):
            box_name = f'{self.box_prefix}{i}'
            
            # Posición inicial distribuida
            y_pos = self.belt_y_min + y_spacing * i
            
            # Posición X aleatoria dentro de la cinta
            x_pos = random.uniform(self.belt_x_min + 0.05, self.belt_x_max - 0.05)
            
            # Tamaño aleatorio
            size = random.choice(self.box_sizes)
            
            # Altura = superficie de cinta + mitad de la altura de la caja
            z_pos = self.belt_height + size[2] / 2
            
            # Rotación aleatoria
            yaw = random.uniform(-math.pi, math.pi)
            
            self.box_states[box_name] = {
                'current_x': x_pos,
                'current_y': y_pos,
                'current_z': z_pos,
                'current_yaw': yaw,
                'size': size,
                'color': random.choice(self.box_colors),
            }
            
            self.get_logger().debug(
                f'Caja {box_name} inicializada en ({x_pos:.2f}, {y_pos:.2f}, {z_pos:.2f})'
            )

    def update_movement(self) -> None:
        """Actualiza el movimiento de todas las cajas."""
        dt = 1.0 / self.update_rate

        for box_name, state in self.box_states.items():
            try:
                self._move_box(box_name, state, dt)
            except Exception as e:
                self.get_logger().warn(f'Error moviendo {box_name}: {e}')

    def _move_box(self, box_name: str, state: Dict, dt: float) -> None:
        """
        Mueve una caja en dirección Y+ y la recicla si llega al final.
        """
        # Mover en dirección Y+
        state['current_y'] += self.belt_speed * dt
        
        # Verificar si llegó al final de la cinta
        if state['current_y'] > self.belt_y_max:
            # Reciclar: reposicionar al inicio con nuevas propiedades
            self._recycle_box(box_name, state)
        
        # Enviar comando a Gazebo
        success = self._set_model_pose(box_name, state)
        
        if success:
            self.success_count += 1
        else:
            self.error_count += 1
            
        # Log periódico de estado
        if (self.success_count + self.error_count) % 300 == 0:
            self.get_logger().info(
                f'Movimientos: {self.success_count} exitosos, {self.error_count} errores'
            )

    def _recycle_box(self, box_name: str, state: Dict) -> None:
        """Reposiciona una caja al inicio de la cinta con nuevas propiedades."""
        # Nueva posición X aleatoria
        state['current_x'] = random.uniform(self.belt_x_min + 0.05, self.belt_x_max - 0.05)
        
        # Posición Y al inicio
        state['current_y'] = self.belt_y_min + random.uniform(0, 0.2)
        
        # Nuevo tamaño aleatorio
        new_size = random.choice(self.box_sizes)
        state['size'] = new_size
        
        # Nueva altura basada en el tamaño
        state['current_z'] = self.belt_height + new_size[2] / 2
        
        # Nueva rotación aleatoria
        state['current_yaw'] = random.uniform(-math.pi, math.pi)
        
        # Nuevo color
        state['color'] = random.choice(self.box_colors)
        
        self.get_logger().info(
            f'Caja {box_name} reciclada - Nueva posición: ({state["current_x"]:.2f}, {state["current_y"]:.2f})'
        )

    def _set_model_pose(self, box_name: str, state: Dict) -> bool:
        """
        Establece la pose de un modelo en Gazebo usando el servicio set_pose.
        
        FORMATO CORRECTO para gz.msgs.Pose:
        name: "model_name"
        position { x: ... y: ... z: ... }
        orientation { x: ... y: ... z: ... w: ... }
        """
        x = state['current_x']
        y = state['current_y']
        z = state['current_z']
        yaw = state['current_yaw']
        
        # Convertir yaw a cuaternión (solo rotación en Z)
        qw = math.cos(yaw / 2)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2)
        
        # FORMATO CORRECTO del mensaje Pose para Gazebo Sim
        pose_msg = f'''name: "{box_name}"
position {{
  x: {x:.6f}
  y: {y:.6f}
  z: {z:.6f}
}}
orientation {{
  x: {qx:.6f}
  y: {qy:.6f}
  z: {qz:.6f}
  w: {qw:.6f}
}}
'''
        
        try:
            # Llamar al servicio de Gazebo
            service_name = f'/world/{self.world_name}/set_pose'
            result = subprocess.run(
                ['gz', 'service', '-s', service_name, 
                 '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean', 
                 '--timeout', '200',
                 '--req', pose_msg],
                capture_output=True,
                text=True,
                timeout=0.2  # Timeout más corto
            )
            
            # Verificar respuesta
            if 'true' in result.stdout.lower():
                return True
            return result.returncode == 0
            
        except subprocess.TimeoutExpired:
            # No loguear cada timeout para no saturar
            return False
        except Exception as e:
            self.get_logger().debug(f'Error en servicio: {e}')
            return False

    def set_speed(self, speed: float) -> None:
        """Cambia la velocidad de la cinta."""
        self.belt_speed = speed
        self.get_logger().info(f'Velocidad de cinta cambiada a {speed} m/s')


def main(args: List = None) -> None:
    rclpy.init(args=args)
    
    controller = ConveyorController()
    
    # Usar MultiThreadedExecutor para mejor rendimiento
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
