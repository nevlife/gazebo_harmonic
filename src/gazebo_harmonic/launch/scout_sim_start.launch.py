import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = get_package_share_directory('gazebo_harmonic')
    scout_share = get_package_share_directory('scout_base')
    world_path = os.path.join(gazebo_share, 'world', 'empty_mobile.sdf')
    xacro_path = os.path.join(gazebo_share, 'urdf', 'scout_gazebo.xacro')

    resource_paths = [
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        scout_share,
        os.path.dirname(scout_share),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str,
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': [
                IfElseSubstitution(
                    LaunchConfiguration('headless'),
                    if_value='-s ',
                ),
                f'-r {world_path} -v 3',
            ],
        }.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'scout',
            '-string', Command(['xacro ', xacro_path]),
            '-x', LaunchConfiguration('start_x'),
            '-y', LaunchConfiguration('start_y'),
            '-z', LaunchConfiguration('start_z'),
            '-Y', LaunchConfiguration('start_yaw'),
        ],
        output='screen',
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scout/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scout/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scout/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scout/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/scout/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/scout/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/scout/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scout_sensor_tf_laser',
        arguments=[
            '--frame-id', 'laser_link',
            '--child-frame-id', 'scout/base_footprint/laser_sensor',
        ],
        output='screen',
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scout_sensor_tf_camera',
        arguments=[
            '--frame-id', 'camera_link',
            '--child-frame-id', 'scout/base_footprint/camera_sensor',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_z', default_value='0.3'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('headless', default_value='false'),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=':'.join(path for path in resource_paths if path),
        ),
        simulator,
        spawn,
        state_publisher,
        bridge,
        static_tf_laser,
        static_tf_camera,
    ])
