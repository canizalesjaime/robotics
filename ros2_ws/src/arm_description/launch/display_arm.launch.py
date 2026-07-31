from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_name = 'arm_description'

    urdf_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'urdf',
        'arm.urdf.xacro'
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz',
        'arm.rviz'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_description': Command(['xacro ', urdf_path])
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
