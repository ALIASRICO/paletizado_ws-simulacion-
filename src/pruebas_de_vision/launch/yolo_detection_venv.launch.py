#!/usr/bin/env python3
"""
Launch file para detección YOLO con entorno virtual aislado.
Lanza Gazebo + Cámara + YOLO Detector usando el venv de YOLO.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Rutas
    pkg_share = FindPackageShare('pruebas_de_vision').find('pruebas_de_vision')
    workspace = os.path.expanduser('~/dobot_ws')
    venv_python = os.path.join(workspace, 'yolo_venv', 'bin', 'python3')
    
    # Parámetros configurables
    model_path = LaunchConfiguration('model_path', default='yolov8n-obb.pt')
    confidence = LaunchConfiguration('confidence', default='0.5')
    show_window = LaunchConfiguration('show_window', default='true')
    
    # 1. Lanzar Gazebo con la cámara
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('pruebas_de_vision'),
            '/launch/camera_test.launch.py'
        ])
    )
    
    # 2. Bridge ROS2-Gazebo para la cámara
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen'
    )
    
    # 3. Detector YOLO (ejecutado con el Python del venv)
    yolo_detector_script = os.path.join(
        workspace, 'src', 'pruebas_de_vision', 'pruebas_de_vision', 'yolo_detector.py'
    )
    
    yolo_detector = ExecuteProcess(
        cmd=[
            venv_python,
            yolo_detector_script,
            '--model', model_path,
            '--confidence', confidence,
            '--show-window', show_window,
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # Delay para esperar a que Gazebo esté listo
    delayed_yolo = TimerAction(
        period=8.0,  # Esperar 8 segundos a que Gazebo inicie
        actions=[yolo_detector]
    )
    
    return LaunchDescription([
        gazebo_launch,
        bridge_node,
        delayed_yolo,
    ])
