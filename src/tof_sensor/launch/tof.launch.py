"""
tof_sensor launch file
----------------------
Usage:
  ros2 launch tof_sensor tof.launch.py
  ros2 launch tof_sensor tof.launch.py frequency_hz:=10 lpn_pin:=23
  ros2 launch tof_sensor tof.launch.py resolution:=16   # 4x4 mode
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('i2c_bus',        default_value='1'),
        DeclareLaunchArgument('lpn_pin',        default_value='-1',
                              description='GPIO pin for LPN (-1 = not used)'),
        DeclareLaunchArgument('resolution',     default_value='64',
                              description='64 = 8x8, 16 = 4x4'),
        DeclareLaunchArgument('frequency_hz',   default_value='15'),
        DeclareLaunchArgument('frame_id',       default_value='tof_frame'),
        DeclareLaunchArgument('h_fov_deg',      default_value='63.0'),
        DeclareLaunchArgument('v_fov_deg',      default_value='63.0'),
        DeclareLaunchArgument('max_range_m',    default_value='4.0'),
        DeclareLaunchArgument('min_range_m',    default_value='0.02'),
        DeclareLaunchArgument('mount_height_m', default_value='0.15'),

        Node(
            package='tof_sensor',
            executable='tof_node',
            name='tof_node',
            output='screen',
            parameters=[{
                'i2c_bus':        LaunchConfiguration('i2c_bus'),
                'lpn_pin':        LaunchConfiguration('lpn_pin'),
                'resolution':     LaunchConfiguration('resolution'),
                'frequency_hz':   LaunchConfiguration('frequency_hz'),
                'frame_id':       LaunchConfiguration('frame_id'),
                'h_fov_deg':      LaunchConfiguration('h_fov_deg'),
                'v_fov_deg':      LaunchConfiguration('v_fov_deg'),
                'max_range_m':    LaunchConfiguration('max_range_m'),
                'min_range_m':    LaunchConfiguration('min_range_m'),
                'mount_height_m': LaunchConfiguration('mount_height_m'),
            }],
        ),

        # Static TF: base_link -> tof_frame
        # Adjust xyz (forward, left, up from base) and rpy to match your mount
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tof_tf',
            arguments=[
                '0.1', '0.0', '0.15',   # x y z  (10cm forward, 15cm up)
                '0',   '0',   '0',       # roll pitch yaw
                'base_link', 'tof_frame'
            ],
        ),
    ])
