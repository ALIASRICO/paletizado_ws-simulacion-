import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Robot type
    robot_type = os.getenv("DOBOT_TYPE", "cr20")
    robot_name_in_model = f'{robot_type}_robot'
    package_name = 'cra_description'
    urdf_name = f"{robot_type}_robot.xacro"

    # Paths
    cra_description_path = get_package_share_directory(package_name)
    dobot_gazebo_path = get_package_share_directory('dobot_gazebo')
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

    # Process XACRO
    xacro_file = os.path.join(cra_description_path, 'urdf', urdf_name)
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_content = doc.toxml()

    # World file
    world_file = os.path.join(dobot_gazebo_path, 'worlds', 'cr.sdf')

    # Launch Gz Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    # Spawn robot in Gz
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name_in_model,
            '-allow_renaming', 'true',
            '-z', '1.0',  # Altura de 1 metro
        ],
        output='screen',
    )

    # Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # Load joint state broadcaster after spawn
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    # Load joint trajectory controller after joint state broadcaster
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             f'{robot_type}_group_controller'],
        output='screen',
    )

    # Event handlers: load controllers sequentially after spawn
    event_spawn_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    event_broadcaster_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_trajectory_controller],
        )
    )

    return LaunchDescription([
        event_spawn_done,
        event_broadcaster_done,
        gz_sim,
        robot_state_publisher,
        gz_spawn_entity,
        gz_bridge,
    ])