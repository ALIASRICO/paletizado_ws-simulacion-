"""
=================================================================
Launch para pruebas de visión: Cámara aislada en Gazebo
=================================================================
Uso:
    ros2 launch pruebas_de_vision camera_test.launch.py
=================================================================
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    pkg_path = get_package_share_directory('pruebas_de_vision')
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

    # World file
    world_file = os.path.join(pkg_path, 'worlds', 'camera_test.sdf')

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
    # Formato: topic@ros2_msg_type[gz_msg_type
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        camera_bridge,
    ])
