#!/usr/bin/env python3
"""
Nodo ROS2 para aplicar movimiento dinámico a cajas en Gazebo.
Usa el servicio set_pose de Gazebo para mover modelos directamente.

Uso:
    ros2 run pruebas_de_vision box_mover --ros-args -p num_boxes:=8
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import math
import random
import time
import subprocess
import json
from typing import List, Dict, Tuple, Optional

from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from std_msgs.msg import Header


class BoxMover(Node):
    """
    Nodo que aplica movimiento a cajas en Gazebo usando el servicio set_pose.
    
    Gazebo Sim no procesa cmd_vel para modelos estáticos sin plugin,
    así que usamos el servicio /world/<world>/set_pose directamente.
    """

    def __init__(self) -> None:
        super().__init__('box_mover')

        # Declarar parámetros
        self.declare_parameter('num_boxes', 8)
        self.declare_parameter('update_rate', 10.0)  # Hz (más bajo para no sobrecargar)
        self.declare_parameter('vertical_amplitude', 0.02)  # metros
        self.declare_parameter('vertical_frequency', 0.5)  # Hz
        self.declare_parameter('rotation_speed', 0.3)  # rad/s max
        self.declare_parameter('horizontal_drift', 0.01)  # m/s max
        self.declare_parameter('box_prefix', 'cardboard_box_')
        self.declare_parameter('world_name', 'dynamic_boxes')

        # Obtener parámetros
        self.num_boxes = self.get_parameter('num_boxes').value
        self.update_rate = self.get_parameter('update_rate').value
        self.vertical_amplitude = self.get_parameter('vertical_amplitude').value
        self.vertical_frequency = self.get_parameter('vertical_frequency').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.horizontal_drift = self.get_parameter('horizontal_drift').value
        self.box_prefix = self.get_parameter('box_prefix').value
        self.world_name = self.get_parameter('world_name').value

        # Posiciones iniciales de las cajas (x, y, z, yaw)
        # Estas deben coincidir con el archivo SDF
        self.initial_poses: Dict[str, Dict] = {
            'cardboard_box_1': {'x': 0.0, 'y': 0.0, 'z': 0.12, 'yaw': 0.3},
            'cardboard_box_2': {'x': -0.35, 'y': 0.25, 'z': 0.10, 'yaw': -0.5},
            'cardboard_box_3': {'x': 0.40, 'y': -0.20, 'z': 0.08, 'yaw': 0.8},
            'cardboard_box_4': {'x': -0.25, 'y': -0.35, 'z': 0.14, 'yaw': 1.2},
            'cardboard_box_5': {'x': 0.30, 'y': 0.30, 'z': 0.10, 'yaw': -0.2},
            'cardboard_box_6': {'x': -0.15, 'y': -0.10, 'z': 0.06, 'yaw': 0.6},
            'cardboard_box_7': {'x': 0.20, 'y': -0.40, 'z': 0.12, 'yaw': -1.0},
            'cardboard_box_8': {'x': -0.40, 'y': 0.10, 'z': 0.08, 'yaw': 1.5},
        }
        
        # Estado de cada caja
        self.box_states: Dict[str, Dict] = {}
        self._initialize_box_states()

        # Tiempo inicial para oscilaciones
        self.start_time = time.time()

        # Timer para actualizar movimiento
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.update_movement)

        self.get_logger().info(f'BoxMover iniciado con {self.num_boxes} cajas')
        self.get_logger().info(f'Frecuencia de actualización: {self.update_rate} Hz')
        self.get_logger().info(f'Método: Servicio set_pose de Gazebo')

    def _initialize_box_states(self) -> None:
        """Inicializa el estado de cada caja con parámetros aleatorios."""
        for i in range(1, self.num_boxes + 1):
            box_name = f'{self.box_prefix}{i}'
            
            # Obtener pose inicial del diccionario o usar valores por defecto
            initial = self.initial_poses.get(box_name, {'x': 0, 'y': 0, 'z': 0.1, 'yaw': 0})
            
            self.box_states[box_name] = {
                'phase_offset': random.uniform(0, 2 * math.pi),
                'rotation_direction': random.choice([-1, 1]),
                'rotation_speed': random.uniform(0.1, self.rotation_speed),
                'drift_x': random.uniform(-self.horizontal_drift, self.horizontal_drift),
                'drift_y': random.uniform(-self.horizontal_drift, self.horizontal_drift),
                'vertical_offset': random.uniform(0.5, 1.5),
                # Posición actual (se actualiza con el tiempo)
                'current_x': initial['x'],
                'current_y': initial['y'],
                'current_z': initial['z'],
                'current_yaw': initial['yaw'],
                # Velocidades acumuladas
                'vx': 0.0,
                'vy': 0.0,
                'vz': 0.0,
            }
            self.get_logger().debug(f'Caja {box_name} inicializada en ({initial["x"]}, {initial["y"]}, {initial["z"]})')

    def update_movement(self) -> None:
        """Actualiza el movimiento de todas las cajas."""
        current_time = time.time() - self.start_time
        dt = 1.0 / self.update_rate

        for box_name, state in self.box_states.items():
            try:
                self._apply_box_movement(box_name, state, current_time, dt)
            except Exception as e:
                self.get_logger().warn(f'Error moviendo {box_name}: {e}')

    def _apply_box_movement(self, box_name: str, state: Dict, current_time: float, dt: float) -> None:
        """
        Aplica movimiento a una caja específica usando el servicio set_pose.
        """
        # Calcular velocidad vertical (oscilación sinusoidal)
        omega = 2 * math.pi * self.vertical_frequency
        phase = omega * current_time + state['phase_offset']
        
        # Velocidad vertical
        vz = self.vertical_amplitude * omega * math.cos(phase) * state['vertical_offset']

        # Velocidad angular (rotación)
        angular_z = state['rotation_speed'] * state['rotation_direction']

        # Añadir variación aleatoria gradual a la deriva
        if random.random() < 0.05:  # 5% de probabilidad cada update
            state['drift_x'] += random.uniform(-0.002, 0.002)
            state['drift_y'] += random.uniform(-0.002, 0.002)
            # Limitar deriva máxima
            state['drift_x'] = max(-self.horizontal_drift, 
                                   min(self.horizontal_drift, state['drift_x']))
            state['drift_y'] = max(-self.horizontal_drift, 
                                   min(self.horizontal_drift, state['drift_y']))

        # Integrar velocidades para obtener nueva posición
        state['current_x'] += state['drift_x'] * dt
        state['current_y'] += state['drift_y'] * dt
        state['current_z'] += vz * dt
        state['current_yaw'] += angular_z * dt

        # Limitar z para que no se vaya demasiado alto o bajo
        initial_z = self.initial_poses.get(box_name, {'z': 0.1})['z']
        state['current_z'] = max(initial_z - 0.05, min(initial_z + 0.05, state['current_z']))

        # Limitar posición horizontal para que no se salga del área
        state['current_x'] = max(-0.8, min(0.8, state['current_x']))
        state['current_y'] = max(-0.8, min(0.8, state['current_y']))

        # Enviar comando a Gazebo usando gz service
        self._set_model_pose(box_name, state)

    def _set_model_pose(self, box_name: str, state: Dict) -> bool:
        """
        Establece la pose de un modelo en Gazebo usando el servicio set_pose.
        
        Usa el comando 'gz service' para llamar al servicio de Gazebo.
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
        
        # Crear el mensaje Protobuf para el servicio set_pose
        # Formato requerido por Gazebo Sim
        pose_msg = f"""
entity_name: "{box_name}"
pose {{
  position {{
    x: {x}
    y: {y}
    z: {z}
  }}
  orientation {{
    x: {qx}
    y: {qy}
    z: {qz}
    w: {qw}
  }}
}}
"""
        
        try:
            # Llamar al servicio de Gazebo
            service_name = f'/world/{self.world_name}/set_pose'
            result = subprocess.run(
                ['gz', 'service', '-s', service_name, '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean', '--timeout', '1000',
                 '--req', pose_msg],
                capture_output=True,
                text=True,
                timeout=0.5
            )
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'Timeout moviendo {box_name}')
            return False
        except Exception as e:
            self.get_logger().debug(f'Error en servicio: {e}')
            return False

    def apply_random_impulse(self, box_name: str) -> bool:
        """
        Aplica un cambio aleatorio a la posición de una caja.
        """
        if box_name not in self.box_states:
            self.get_logger().warn(f'Caja {box_name} no encontrada')
            return False

        state = self.box_states[box_name]
        state['current_x'] += random.uniform(-0.1, 0.1)
        state['current_y'] += random.uniform(-0.1, 0.1)
        state['current_yaw'] += random.uniform(-0.5, 0.5)

        self.get_logger().info(f'Impulso aplicado a {box_name}')
        return True

    def random_impulse_all(self) -> None:
        """Aplica impulsos aleatorios a todas las cajas."""
        for box_name in self.box_states:
            self.apply_random_impulse(box_name)

        self.get_logger().info('Impulsos aleatorios aplicados a todas las cajas')


def main(args: List = None) -> None:
    rclpy.init(args=args)
    
    box_mover = BoxMover()
    
    # Usar MultiThreadedExecutor para mejor rendimiento
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(box_mover)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        box_mover.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
