#!/usr/bin/env python3
"""
vision_real.launch.py — Visión con Cámara Física (RealSense D435i)

Lanza la cadena completa de percepción con hardware real:
  RealSense D435i → apriltag_node → pose_publisher

NO requiere Gazebo. Funciona con la cámara conectada por USB.

TOPICS DE SALIDA:
  /camera/color/image_raw              — imagen RGB (de realsense2_camera)
  /camera/depth/image_rect_raw         — imagen depth
  /camera/color/camera_info            — calibración intrínseca real
  /apriltag/detections                 — detecciones 2D+homography
  /vision/tag_poses                    — poses 3D validadas (para Coordinador)
  /vision/all_tag_poses                — PoseArray de todas las detecciones válidas

USO:
  ros2 launch dobot_vision vision_real.launch.py
  ros2 launch dobot_vision vision_real.launch.py camera_fps:=15
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # Paths
    vision_pkg = get_package_share_directory('dobot_vision')
    tags_config = os.path.join(vision_pkg, 'config', 'tags_36h11.yaml')

    # ─── Resolución / FPS se configuran directamente en el nodo ───
    # Para cambiar: editar los valores de color_profile y depth_profile abajo
    # Formatos válidos: '640x480x15', '640x480x30', '1280x720x30', etc.


    # ─── Intel RealSense D435i Node ───
    # Driver oficial: publica RGB, Depth, IMU, PointCloud, camera_info
    # Nota: color_profile y depth_profile se configuran individualmente
    # porque LaunchConfiguration no soporta concatenación con +
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            'rgb_camera.color_profile': '640x480x15',
            'depth_module.depth_profile': '640x480x15',
            'align_depth.enable': True,
            'pointcloud.enable': False,
        }],
        output='screen',
    )

    # ─── AprilTag Detection Node ───
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='apriltag',
        remappings=[
            # RealSense publica en /camera/camera/... (namespace=camera, name=camera)
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        parameters=[
            tags_config,
            {'use_sim_time': False},
        ],
        output='screen',
    )

    # ─── Vision Display (visualización OpenCV + poses 3D) ───
    vision_display_node = Node(
        package='dobot_vision',
        executable='vision_display',
        name='vision_display',
        parameters=[{
            'reference_frame': 'world',
            'camera_frame': 'camera_color_optical_frame',
            'tag_size_m': 0.10,
            'min_decision_margin': 20.0,
            'pose_smoothing_window': 5,
            'target_tag_ids': [10, 11, 12, 13, 20, 21, 22, 23],
            'show_window': True,
            'window_name': 'Dobot Vision',
            'workspace_bounds.x_min': -2.0,
            'workspace_bounds.x_max': 2.0,
            'workspace_bounds.y_min': -2.0,
            'workspace_bounds.y_max': 2.0,
            'workspace_bounds.z_min': 0.0,
            'workspace_bounds.z_max': 4.0,
        }],
        output='screen',
    )

    # ─── Static TF: mount de la cámara ───
    # REGLA #3: Posición configurable — AJUSTAR tras medición física
    # Ejemplo: cámara en techo a 2m, mirando hacia abajo
    # Los valores DEBEN actualizarse con la posición real medida
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '3.5',          # Cámara a 3.5m del suelo
            '--roll', '0.0',
            '--pitch', '1.5708',   # pi/2 = mirando abajo
            '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    return LaunchDescription([
        # Nodes
        realsense_node,
        camera_tf,
        apriltag_node,
        vision_display_node,
    ])
