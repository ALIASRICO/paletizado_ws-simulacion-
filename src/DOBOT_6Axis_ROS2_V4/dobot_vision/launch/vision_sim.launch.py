#!/usr/bin/env python3
"""
vision_sim.launch.py — Visión en Simulación (Gazebo)

Lanza la cadena de percepción sobre los topics de cámara simulada en Gazebo:
  camera (Gazebo) → apriltag_node → pose_publisher

PRERREQUISITO: Gazebo con cámara debe estar corriendo:
  ros2 launch dobot_gazebo gazebo_with_camera.launch.py

TOPICS DE ENTRADA (de Gazebo):
  /camera/camera/color/image_raw       — imagen RGB
  /camera/camera/color/camera_info     — calibración (si bridge activo)

TOPICS DE SALIDA:
  /apriltag/detections                 — detecciones 2D+homography
  /vision/tag_poses                    — poses 3D validadas (para Coordinador)
  /vision/all_tag_poses                — PoseArray de todas las detecciones válidas
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # Paths
    vision_pkg = get_package_share_directory('dobot_vision')
    tags_config = os.path.join(vision_pkg, 'config', 'tags_36h11.yaml')
    vision_params = os.path.join(vision_pkg, 'config', 'vision_params.yaml')

    # ─── AprilTag Detection Node ───
    # Suscribe a imagen rectificada y camera_info, publica detecciones + TF
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='apriltag',
        remappings=[
            # En Gazebo, la imagen viene directamente del bridge
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        parameters=[
            tags_config,
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # ─── Pose Publisher (nuestro nodo) ───
    # Recibe detecciones, valida, suaviza, publica poses en frame world
    pose_publisher_node = Node(
        package='dobot_vision',
        executable='pose_publisher',
        name='pose_publisher',
        parameters=[
            vision_params,
            {'use_sim_time': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        apriltag_node,
        pose_publisher_node,
    ])
