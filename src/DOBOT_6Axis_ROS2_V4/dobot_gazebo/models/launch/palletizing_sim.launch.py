"""
=================================================================
Launch Unificado: Simulación Gazebo + Robot CR20
=================================================================
Uso:
    ros2 launch dobot_gazebo palletizing_sim.launch.py
    ros2 launch dobot_gazebo palletizing_sim.launch.py world:=minimal_calibration.sdf
=================================================================
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # ======================== ARGS ========================
    robot_type = os.getenv("DOBOT_TYPE", "cr20")
    robot_name = f'{robot_type}_robot'
    package_name = 'cra_description'

    # World file argument (default: minimal_calibration.sdf)
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='minimal_calibration.sdf',
        description='Nombre del archivo SDF del mundo en dobot_gazebo/worlds'
    )

    # ======================== PATHS ========================
    cra_description_path = get_package_share_directory(package_name)
    dobot_gazebo_path = get_package_share_directory('dobot_gazebo')
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

    # ======================== XACRO ========================
    xacro_file = os.path.join(cra_description_path, 'urdf', f'{robot_type}_robot.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    # ======================== WORLD ========================
    worlds_dir = os.path.join(dobot_gazebo_path, 'worlds')
    world_file = PathJoinSubstitution([
        worlds_dir,
        LaunchConfiguration('world'),
    ])

    # Set GZ model path for apriltag textures
    models_path = os.path.join(dobot_gazebo_path, 'models')
    gz_model_path = os.environ.get('GZ_SIM_RESOURCE_URI', '')
    os.environ['GZ_SIM_RESOURCE_URI'] = f'{models_path}:{gz_model_path}'

    # ======================== GAZEBO SIM ========================
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
        }.items(),
    )

    # ======================== STATE PUBLISHER ========================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ======================== SPAWN ROBOT ========================
    # Robot en el origen (z=0, el suelo está en z=0)
    gz_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-z', '0.72',  # Altura de 1 metro
        ],
        output='screen',
    )

    # ======================== BRIDGES ========================
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # ======================== CONTROLLERS ========================
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    load_jtc = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             f'{robot_type}_group_controller'],
        output='screen',
    )

    # Secuencia: spawn → jsb → jtc
    event_spawn_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_robot,
            on_exit=[load_jsb],
        )
    )

    event_jsb_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_jsb,
            on_exit=[load_jtc],
        )
    )

    # ======================== LAUNCH ========================
    ld = LaunchDescription()

    # Arguments
    ld.add_action(world_arg)

    # Events
    ld.add_action(event_spawn_done)
    ld.add_action(event_jsb_done)

    # Core
    ld.add_action(gz_sim)
    ld.add_action(robot_state_publisher)

    # Spawn robot only (AprilTags are in the SDF world file)
    ld.add_action(gz_spawn_robot)

    # Bridges
    ld.add_action(clock_bridge)

    return ld
