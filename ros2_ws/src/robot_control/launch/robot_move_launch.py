from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='lunar',
            output='screen',
            #name='l',
        ),
        Node(
            package='robot_control',
            executable='imu',
        ),
        Node(
            package='robot_control',
            executable='motor',
        ),
        Node(
            package='robot_control',
            executable='arm',
        ),
        Node(
            package='robot_control',
            executable='robot_interface',
            parameters=[
              {
               "host": "0.0.0.0",
               "port": 8000
              }
            ],
            output='screen'
        ),
        ])
