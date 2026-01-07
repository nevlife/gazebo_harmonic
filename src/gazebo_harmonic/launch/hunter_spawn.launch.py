import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
import launch_ros
import launch


def generate_launch_description():
    urdf_pkg_share = FindPackageShare(package='hunter_base').find('hunter_base')
    pkg_share = FindPackageShare(package='gazebo_harmonic').find('gazebo_harmonic')
    urdf_file = os.path.join(pkg_share, 'urdf/hunter_gazebo.xacro')
    #urdf_file = os.path.join(urdf_pkg_share, 'urdf/hunter_base_gazebo.xacro')

    SetEnvironmentVariable('GAZEBO_MODEL_PATH', pkg_share)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': launch_ros.descriptions.ParameterValue( launch.substitutions.Command([
                    'xacro ', urdf_file]), value_type=str)
        }]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('start_x'),
            '-y', LaunchConfiguration('start_y'),
            '-z', LaunchConfiguration('start_z'),
            '-Y', LaunchConfiguration('start_yaw')
        ],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    load_ackermann_steering_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_x', default_value='0.0',
                              description='X coordinate of starting position'),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description='Y coordinate of starting position'),
        DeclareLaunchArgument('start_z', default_value='0.0',
                              description='Z coordinate of starting position'),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='Yaw angle of starting orientation'),
        DeclareLaunchArgument('robot_name', default_value='',
                              description='Name and prefix for this robot'),
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_ackermann_steering_controller],
            )
        ),
    ])
