#!/usr/bin/env python3
"""
Launch file para simulación de cinta transportadora con detección YOLO.

Incluye:
- Gazebo con mundo de cinta transportadora
- Bridge ROS2-Gazebo para la cámara
- Controlador de cinta transportadora
- Detector YOLO
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Obtener directorio del paquete
    pkg_dir = get_package_share_directory('pruebas_de_vision')
    
    # Ruta al mundo SDF
    world_file = os.path.join(pkg_dir, 'worlds', 'conveyor_belt.sdf')
    
    # Argumentos de launch
    declared_arguments = [
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Ejecutar Gazebo sin GUI'
        ),
        DeclareLaunchArgument(
            'belt_speed',
            default_value='0.1',
            description='Velocidad de la cinta transportadora (m/s)'
        ),
        DeclareLaunchArgument(
            'spawn_interval',
            default_value='2.0',
            description='Intervalo entre generación de cajas (segundos)'
        ),
        DeclareLaunchArgument(
            'max_boxes',
            default_value='5',
            description='Máximo de cajas simultáneas'
        ),
    ]
    
    # Lanzar Gazebo con el mundo de la cinta transportadora
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r',
            world_file,
        ] + (['-g'] if False else []),  # headless
        output='screen',
    )
    
    # Bridge para la cámara
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )
    
    # Controlador de la cinta transportadora
    conveyor_controller = Node(
        package='pruebas_de_vision',
        executable='conveyor_controller',
        name='conveyor_controller',
        parameters=[{
            'world_name': 'conveyor_belt',
            'belt_speed': LaunchConfiguration('belt_speed'),
            'spawn_interval': LaunchConfiguration('spawn_interval'),
            'max_boxes': LaunchConfiguration('max_boxes'),
        }],
        output='screen',
    )
    
    return LaunchDescription(
        declared_arguments + [
            gz_sim,
            camera_bridge,
            conveyor_controller,
        ]
    )
