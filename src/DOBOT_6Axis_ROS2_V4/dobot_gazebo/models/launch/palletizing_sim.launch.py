"""
=================================================================
Launch Unificado: Simulación Gazebo + MoveIt/RViz
Replica el entorno físico: CR20 + mesa + cinta + cámara + AprilTags
=================================================================
Uso:
    ros2 launch dobot_gazebo palletizing_sim.launch.py
    ros2 launch dobot_gazebo palletizing_sim.launch.py robot_type:=cr20
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # ======================== ARGS ========================
    robot_type = os.getenv("DOBOT_TYPE", "cr20")
    robot_name = f'{robot_type}_robot'
    package_name = 'cra_description'

    # ======================== PATHS ========================
    cra_description_path = get_package_share_directory(package_name)
    dobot_gazebo_path = get_package_share_directory('dobot_gazebo')
    dobot_vision_path = get_package_share_directory('dobot_vision')
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

    # ======================== XACRO ========================
    # Robot URDF
    xacro_file = os.path.join(cra_description_path, 'urdf', f'{robot_type}_robot.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    # Camera URDF
    camera_xacro = os.path.join(cra_description_path, 'urdf', 'camera_ceiling.urdf.xacro')
    camera_doc = xacro.parse(open(camera_xacro))
    xacro.process_doc(camera_doc)
    camera_description = camera_doc.toxml()

    # ======================== WORLD ========================
    world_file = os.path.join(dobot_gazebo_path, 'worlds', 'palletizing.sdf')

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
            'gz_args': f'-r {world_file}',
        }.items(),
    )

    # ======================== STATE PUBLISHERS ========================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    camera_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='camera_state_publisher',
        namespace='camera',
        output='screen',
        parameters=[{
            'robot_description': camera_description,
            'use_sim_time': True,
        }],
    )

    # ======================== SPAWN ENTITIES ========================
    # Robot en el centro de la mesa (z=0.72 = altura de la mesa)
    gz_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-z', '0.72',
        ],
        output='screen',
    )

    # Cámara — SDF standalone para renderizado (URDF solo para TF)
    # El modelo SDF tiene la resolución/FOV exactas de la D435i real
    gz_spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(models_path, 'realsense_d435i', 'model.sdf'),
            '-name', 'realsense_camera',
            '-allow_renaming', 'true',
        ],
        output='screen',
    )

    # AprilTags — Spawn cada tag individualmente
    apriltag_spawners = []
    for tag_id in [10, 11, 12, 13]:
        apriltag_spawners.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', os.path.join(models_path, 'apriltags', f'tag_{tag_id}.sdf'),
                    '-name', f'apriltag_{tag_id}',
                    '-allow_renaming', 'true',
                ],
                output='screen',
            )
        )

    # ======================== BRIDGES ========================
    # Bridge Gazebo → ROS2 (mismos topics que la cámara real)
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/camera/camera/depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
        ],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # ======================== APRILTAG DETECTOR ========================
    # Mismo apriltag_ros que se usa con la cámara real
    tags_config = os.path.join(dobot_vision_path, 'config', 'tags_36h11.yaml')

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='apriltag',
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        parameters=[
            tags_config,
            {'use_sim_time': True},
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

    # Events
    ld.add_action(event_spawn_done)
    ld.add_action(event_jsb_done)

    # Core
    ld.add_action(gz_sim)
    ld.add_action(robot_state_publisher)
    ld.add_action(camera_state_publisher)

    # Spawn
    ld.add_action(gz_spawn_robot)
    ld.add_action(gz_spawn_camera)
    for spawner in apriltag_spawners:
        ld.add_action(spawner)

    # Bridges
    ld.add_action(clock_bridge)
    ld.add_action(camera_bridge)

    # Vision
    ld.add_action(apriltag_node)

    return ld
