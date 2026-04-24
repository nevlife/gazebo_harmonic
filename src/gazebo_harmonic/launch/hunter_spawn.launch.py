import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
import launch_ros
import launch


def generate_launch_description():
    pkg_share = FindPackageShare(package='gazebo_harmonic').find('gazebo_harmonic')
    urdf_file = os.path.join(pkg_share, 'urdf/hunter_gazebo.xacro')

    # Lightweight URDF for robot_state_publisher (heavy meshes excluded)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': launch_ros.descriptions.ParameterValue(
                launch.substitutions.Command(['xacro ', urdf_file, ' for_rviz:=true']),
                value_type=str)
        }]
    )

    # Full URDF for Gazebo spawn (all meshes included)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-string', launch.substitutions.Command(['xacro ', urdf_file, ' for_rviz:=false']),
            '-x', LaunchConfiguration('start_x'),
            '-y', LaunchConfiguration('start_y'),
            '-z', LaunchConfiguration('start_z'),
            '-Y', LaunchConfiguration('start_yaw')
        ],
        output='screen'
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
    ])
