from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tof_sensor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucy',
    maintainer_email='lucy@lucy.local',
    description='VL53L5CX 8x8 ToF sensor ROS2 driver',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tof_node = tof_sensor.tof_node:main',
        ],
    },
)
