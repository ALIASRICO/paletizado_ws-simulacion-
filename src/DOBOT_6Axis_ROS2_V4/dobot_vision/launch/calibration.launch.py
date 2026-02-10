"""
calibration.launch.py — Lanzar calibración cámara→robot

Uso:
  1. Terminal 1: Lanzar driver del robot
     export IP_address=192.168.5.1 && export DOBOT_TYPE=cr20
     ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py

  2. Terminal 2: Lanzar visión real
     ros2 launch dobot_vision vision_real.launch.py

  3. Terminal 3: Lanzar calibración
     ros2 launch dobot_vision calibration.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('dobot_vision')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        # Argumentos configurables
        DeclareLaunchArgument(
            'output_file',
            default_value='',
            description='Ruta al archivo de salida de calibración'
        ),

        # Nodo de calibración
        Node(
            package='dobot_vision',
            executable='calibration_node',
            name='calibration_node',
            output='screen',
            prefix='xterm -e' if False else '',  # Cambiar a True si se necesita terminal dedicada
            parameters=[{
                'conveyor_tag_ids': [10, 11, 12, 13],
                'palet_tag_ids': [20, 21, 22, 23],
                'tag_size': 0.10,
                'detection_topic': '/apriltag/detections',
                'min_decision_margin': 30.0,
                'samples_per_tag': 20,
                'output_file': LaunchConfiguration('output_file'),
            }],
        ),
    ])
