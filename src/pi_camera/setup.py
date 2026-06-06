from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pi_camera'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucy',
    maintainer_email='lucy@lucy.local',
    description='Raspberry Pi Camera Module 3 Wide driver for ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_node = pi_camera.camera_node:main',
        ],
    },
)
