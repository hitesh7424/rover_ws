"""
Launch only sensor subsystems (LiDAR, ToF, Camera) without micro-ROS agent.
Useful for testing individual sensors or when the motor controller is not available.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the sensors-only launch description."""

    # Declare launch arguments
    lidar_enabled = DeclareLaunchArgument(
        'lidar',
        default_value='true',
        description='Enable YDLiDAR'
    )
    tof_enabled = DeclareLaunchArgument(
        'tof',
        default_value='true',
        description='Enable Time-of-Flight sensor'
    )
    camera_enabled = DeclareLaunchArgument(
        'camera',
        default_value='false',
        description='Enable Raspberry Pi camera'
    )

    # Get package paths
    ydlidar_dir = FindPackageShare('ydlidar_ros2_driver')
    tof_sensor_dir = FindPackageShare('tof_sensor')
    pi_camera_dir = FindPackageShare('pi_camera')

    # Launch files
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ydlidar_dir, 'launch',
                'ydlidar.py'
            ])
        ),
        condition=LaunchConfiguration('lidar')
    )

    tof_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                tof_sensor_dir, 'launch',
                'tof.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('tof'))
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pi_camera_dir, 'launch',
                'camera.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('camera'))
    )

    return LaunchDescription([
        lidar_enabled,
        tof_enabled,
        camera_enabled,
        lidar_launch,
        tof_launch,
        camera_launch,
    ])
