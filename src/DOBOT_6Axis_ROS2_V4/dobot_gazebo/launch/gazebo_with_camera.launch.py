import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
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

    # Process Robot XACRO
    xacro_file = os.path.join(cra_description_path, 'urdf', urdf_name)
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_content = doc.toxml()

    # Process Camera XACRO
    camera_xacro_file = os.path.join(cra_description_path, 'urdf', 'camera_ceiling.urdf.xacro')
    camera_doc = xacro.parse(open(camera_xacro_file))
    xacro.process_doc(camera_doc)
    camera_description_content = camera_doc.toxml()

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

    # Camera State Publisher
    camera_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='camera_state_publisher',
        namespace='camera',
        output='screen',
        parameters=[{
            'robot_description': camera_description_content,
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
            '-z', '1.0',  # Robot a 1 metro
        ],
        output='screen',
    )

    # Spawn camera in Gz
    gz_spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(dobot_gazebo_path, 'models', 'realsense_d435i', 'model.sdf'),
            '-name', 'realsense_camera',
            '-allow_renaming', 'true',
        ],
        output='screen',
    )

    # Bridge for camera topics
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # RGB Camera
            '/camera/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # Camera Info (necesario para AprilTag pose estimation)
            '/camera/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Depth Camera  
            '/camera/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/camera/camera/depth/image', '/camera/camera/depth/image_rect_raw'),
        ],
        output='screen',
    )

    # Bridge for robot
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

    # Load joint trajectory controller
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             f'{robot_type}_group_controller'],
        output='screen',
    )

    # Event handlers
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
        camera_state_publisher,
        gz_spawn_entity,
        gz_spawn_camera,
        gz_bridge,
        camera_bridge,
    ])