from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'lerobot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.[sw][dd][ff]*')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oberton31',
    maintainer_email='bertonoliver50@gmail.com',
    description='ROS 2 package providing Gazebo simulation support for the LeRobot SO101 robotic arm.',
    license='Apache-2.0',
    tests_require=[],
    entry_points={
        'console_scripts': [
            # executable_name = package_folder.filename:main_function
            'lerobot_ros2_bridge = lerobot_gazebo.lerobot_ros2_bridge:main',
        ],
    },
)