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
    limo_share = get_package_share_directory('limo_base')
    world_path = os.path.join(gazebo_share, 'world', 'empty_mobile.sdf')
    xacro_path = os.path.join(gazebo_share, 'urdf', 'limo_gazebo.xacro')

    resource_paths = [
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        limo_share,
        os.path.dirname(limo_share),
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
            '-name', 'limo',
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
            '/limo/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/limo/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/limo/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/limo/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/limo/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/limo/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/limo/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/limo/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='limo_sensor_tf_laser',
        arguments=[
            '--frame-id', 'laser_link',
            '--child-frame-id', 'limo/base_footprint/laser_sensor',
        ],
        output='screen',
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='limo_sensor_tf_camera',
        arguments=[
            '--frame-id', 'camera_link',
            '--child-frame-id', 'limo/base_footprint/camera_sensor',
        ],
        output='screen',
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='limo_sensor_tf_imu',
        arguments=[
            '--frame-id', 'imu_link',
            '--child-frame-id', 'limo/base_footprint/imu_sensor',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_z', default_value='0.0'),
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
        static_tf_imu,
    ])
