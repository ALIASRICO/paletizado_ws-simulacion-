#!/usr/bin/env python3
"""
Launch file para simulación de cinta transportadora con Gazebo Sim 8
Incluye:
- Gazebo Sim con mundo de conveyor
- ROS2-Gazebo bridge para cámara
- Nodo controlador de conveyor
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Directorio del paquete
    pkg_share = FindPackageShare('gz_conveyorbelt')
    
    # Ruta al mundo SDF
    world_file = PathJoinSubstitution([
        pkg_share, 'worlds', 'conveyor_demo.sdf'
    ])
    
    # Argumentos de launch
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Ruta al archivo de mundo SDF'
    )
    
    # Gazebo Sim
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r',
            LaunchConfiguration('world')
        ],
        output='screen',
        additional_env={
            'GZ_PLUGIN_PATH': os.path.join(
                os.path.expanduser('~/dobot_ws'),
                'install/gz_conveyorbelt/lib'
            )
        }
    )
    
    # ROS2-Gazebo bridge para cámara
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_world_arg,
        gz_sim,
        bridge_node,
    ])
