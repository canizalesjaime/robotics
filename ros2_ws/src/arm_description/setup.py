from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_description'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        # Register package
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # Install package.xml
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # Install launch files
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),

        # Install URDF/Xacro files
        (
            'share/' + package_name + '/urdf',
            glob('urdf/*')
        ),

        # Install STL meshes
        (
            'share/' + package_name + '/meshes',
            glob('meshes/*')
        ),

        # Install RViz configs (if the folder exists)
        (
            'share/' + package_name + '/rviz',
            glob('rviz/*')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='root',
    maintainer_email='canizales.jaime16@gmail.com',

    description='Arm description package',

    license='TODO: License declaration',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
        ],
    },
)