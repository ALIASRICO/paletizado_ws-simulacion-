"""
=================================================================
Launch para Detección YOLOv8-OBB con Cámara en Gazebo
=================================================================
Lanza Gazebo + Bridge + Detector YOLO en un solo comando.

Uso:
    # Con modelo entrenado
    ros2 launch pruebas_de_vision yolo_detection.launch.py
    
    # Especificando modelo
    ros2 launch pruebas_de_vision yolo_detection.launch.py model_path:=/path/to/best.pt
    
    # Sin mostrar ventana de OpenCV
    ros2 launch pruebas_de_vision yolo_detection.launch.py show_window:=false

Topics publicados:
    /vision/detections/image   - Imagen con detecciones anotadas
    /vision/detections/poses   - PoseArray con posiciones detectadas
    /vision/detections/bboxes  - Detection2DArray con bounding boxes
=================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    pkg_path = get_package_share_directory('pruebas_de_vision')
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')
    
    # World file
    world_file = os.path.join(pkg_path, 'worlds', 'camera_test.sdf')
    
    # Argumentos de launch
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Ruta al modelo YOLO entrenado (.pt o .onnx)'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.25',
        description='Umbral de confianza para detecciones'
    )
    
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='Mostrar ventana de OpenCV con detecciones'
    )
    
    # Lanzar Gazebo con el mundo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
        }.items(),
    )
    
    # Bridge: Gazebo → ROS2 para la imagen
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )
    
    # Nodo detector YOLO
    yolo_detector = Node(
        package='pruebas_de_vision',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'show_window': LaunchConfiguration('show_window'),
            'image_topic': '/camera/color/image_raw',
            'publish_annotated': True,
            'publish_poses': True,
        }],
        output='screen',
    )
    
    return LaunchDescription([
        # Argumentos
        model_path_arg,
        confidence_arg,
        show_window_arg,
        
        # Nodos
        gz_sim,
        camera_bridge,
        yolo_detector,
    ])
