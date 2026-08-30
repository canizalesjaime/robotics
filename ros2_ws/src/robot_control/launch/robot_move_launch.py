from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='imu_bno055',
        ),
        Node(
            package='robot_control',
            executable='ttmotors',
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
            #name='l',
        ),
        ])
