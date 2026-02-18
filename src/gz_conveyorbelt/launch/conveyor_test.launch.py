#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
╔═══════════════════════════════════════════════════════════════════════════╗
║           CONVEYOR TEST LAUNCH FILE                                       ║
╠═══════════════════════════════════════════════════════════════════════════╣
║  Launch file para el sistema de banda transportadora con                  ║
║  conveyor_test.sdf y conveyor_logic_controller.                           ║
║                                                                           ║
║  Incluye:                                                                 ║
║  - Gazebo Harmonic con mundo conveyor_test.sdf                            ║
║  - ROS2-Gazebo Bridge                                                     ║
║  - Conveyor Logic Controller Node                                         ║
╚═══════════════════════════════════════════════════════════════════════════╝
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ═══════════════════════════════════════════════════════════════════════
    # CONSTANTES Y RUTAS
    # ═══════════════════════════════════════════════════════════════════════
    pkg_name = 'gz_conveyorbelt'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Rutas de archivos
    world_file = os.path.join(pkg_share, 'worlds', 'conveyor_test.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_scripted.yaml')
    
    # ═══════════════════════════════════════════════════════════════════════
    # ARGUMENTOS DE LANZAMIENTO
    # ═══════════════════════════════════════════════════════════════════════
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Ruta al archivo SDF del mundo'
    )
    
    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Ejecutar Gazebo sin GUI'
    )
    
    declare_speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.1',
        description='Velocidad de la banda (m/s)'
    )
    
    declare_spawn_interval_arg = DeclareLaunchArgument(
        'spawn_interval',
        default_value='8.0',
        description='Intervalo de spawn automático (segundos)'
    )
    
    declare_update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='20.0',
        description='Frecuencia de actualización (Hz)'
    )
    
    declare_auto_spawn_arg = DeclareLaunchArgument(
        'auto_spawn',
        default_value='true',
        description='Habilitar spawn automático de cajas'
    )
    
    # ═══════════════════════════════════════════════════════════════════════
    # GAZEBO SIM (HARMONIC)
    # ═══════════════════════════════════════════════════════════════════════
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            LaunchConfiguration('world'),
            '-r',  # Run simulation immediately
        ],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': pkg_share}
    )
    
    # ═══════════════════════════════════════════════════════════════════════
    # ROS2 - GAZEBO BRIDGE
    # ═══════════════════════════════════════════════════════════════════════
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}',
        ],
        parameters=[{
            'config_file': bridge_config,
        }]
    )
    
    # ═══════════════════════════════════════════════════════════════════════
    # CONVEYOR LOGIC CONTROLLER NODE
    # ═══════════════════════════════════════════════════════════════════════
    # Delay para esperar a que Gazebo esté listo
    conveyor_controller = TimerAction(
        period=5.0,  # Esperar 5 segundos
        actions=[
            Node(
                package=pkg_name,
                executable='conveyor_logic_controller',
                name='conveyor_logic_controller',
                output='screen',
                parameters=[{
                    'conveyor_speed': LaunchConfiguration('speed'),
                    'spawn_interval': LaunchConfiguration('spawn_interval'),
                    'update_rate': LaunchConfiguration('update_rate'),
                    'auto_spawn': LaunchConfiguration('auto_spawn'),
                    'world_name': 'conveyor_world',
                }]
            )
        ]
    )
    
    # ═══════════════════════════════════════════════════════════════════════
    # LAUNCH DESCRIPTION
    # ═══════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        # Argumentos
        declare_world_arg,
        declare_headless_arg,
        declare_speed_arg,
        declare_spawn_interval_arg,
        declare_update_rate_arg,
        declare_auto_spawn_arg,
        
        # Nodos
        gz_sim,
        bridge_node,
        conveyor_controller,
    ])
