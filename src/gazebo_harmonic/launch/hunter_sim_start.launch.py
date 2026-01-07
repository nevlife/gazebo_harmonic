import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Set Gazebo model path
    gazebo_model_path = '/home/pgw/dev/gazebo_models_worlds_collection'
    lidar_model_path = os.path.join(get_package_share_directory('hunter_base'), 'urdf')
    combined_path = f'{gazebo_model_path}:{lidar_model_path}'
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined_path
    )

    # Use empty world with GPS support
    pkg_share = get_package_share_directory('gazebo_harmonic')
    gazebo_world_path = os.path.join(pkg_share, 'world', 'empty_with_gps.sdf')

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {gazebo_world_path} -v 4'}.items()
    )

    car_sim_options = {
        'start_x': '0',
        'start_y': '0',
        'start_z': '0.4',
        'start_yaw': '0',
        'pub_tf': 'true',
        'tf_freq': '100.0',
        'blue': 'false'
    }

    spawn_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                         get_package_share_directory('gazebo_harmonic'),
                         'launch', 'hunter_spawn.launch.py')
        ]),
        launch_arguments=car_sim_options.items()
    )

    # Bridge between Gazebo and ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        remappings=[('/velodyne_points/points', '/velodyne_points')],
        arguments=[
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/gps@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            # '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # Static TF to bridge URDF link and Gazebo sensor frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'velodyne_sensor', 'hunter2/base_link/velodyne_sensor'],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo_simulator,
        spawn_car,
        bridge,
        static_tf,
    ])
