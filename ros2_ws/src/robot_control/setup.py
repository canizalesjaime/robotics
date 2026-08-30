import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='canizales.jaime16@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = robot_control.teleop_node:main',
            'ttmotors = robot_control.tt_motors_node:main',
            'ultrasonic = robot_control.ultrasonic_node:main',
            'mpu6050 = robot_control.mpu6050_node:main',
            'picam = robot_control.picam_node:main',
            'arm = robot_control.arm_node:main',
            'lunar = robot_control.lunar_node:main',
            'robot_interface = robot_control.robot_interface:main',
            'imu_bno055 = robot_control.imu_bno055_node:main',
        ],
    },
)
