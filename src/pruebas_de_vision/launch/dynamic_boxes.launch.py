#!/usr/bin/env python3
"""
Launch file para simulación con cajas dinámicas.
Incluye Gazebo + cámara + nodo de movimiento de cajas.

Uso:
    ros2 launch pruebas_de_vision dynamic_boxes.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:
    # Obtener directorio del paquete
    pkg_dir = get_package_share_directory('pruebas_de_vision')
    
    # Ruta al mundo SDF
    world_file = os.path.join(pkg_dir, 'worlds', 'dynamic_boxes.sdf')
    
    # Argumentos de launch
    num_boxes_arg = DeclareLaunchArgument(
        'num_boxes',
        default_value='8',
        description='Número de cajas a mover'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='30.0',
        description='Frecuencia de actualización del movimiento (Hz)'
    )
    
    # Lanzar Gazebo con el mundo de cajas dinámicas
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )
    
    # Bridge para la cámara (ROS2 <-> Gazebo)
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )
    
    # Bridges para cmd_vel de cada caja (ROS2 -> Gazebo)
    # El formato es: /model/<model_name>/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
    cmd_vel_bridges = []
    for i in range(1, 9):  # 8 cajas
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/cardboard_box_{i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            output='screen',
        )
        cmd_vel_bridges.append(bridge)
    
    # Nodo para mover las cajas (iniciado después de que Gazebo esté listo)
    box_mover_node = TimerAction(
        period=5.0,  # Esperar 5 segundos a que Gazebo inicie
        actions=[
            Node(
                package='pruebas_de_vision',
                executable='box_mover',
                name='box_mover',
                parameters=[{
                    'num_boxes': LaunchConfiguration('num_boxes'),
                    'update_rate': LaunchConfiguration('update_rate'),
                }],
                output='screen',
            ),
        ],
    )
    
    # Lista de todos los nodos
    nodes = [
        num_boxes_arg,
        update_rate_arg,
        gz_sim,
        bridge_camera,
    ]
    
    # Agregar bridges de cmd_vel
    nodes.extend(cmd_vel_bridges)
    
    # Agregar nodo de movimiento
    nodes.append(box_mover_node)
    
    return LaunchDescription(nodes)
