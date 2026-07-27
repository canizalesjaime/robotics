from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='lunar',
            name='l',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='mpu6050',
            name='i'
        ),
        Node(
            package='robot_control',
            executable='motor',
            name='m'
        ),
        Node(
            package='robot_control',
            executable='arm',
            name='a'
        ),
        Node(
            package='robot_control',
            executable='robot_interface',
            name='r',
            parameters=[
              {
               "host": "0.0.0.0",
               "port": 8000
              }
            ],
            output='screen'
        ),
        ])
