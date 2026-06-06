"""
Launch only the motor controller (micro-ROS agent).
Useful for testing motor control without sensors.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate the motors-only launch description."""

    # Declare launch arguments
    micro_ros_port = DeclareLaunchArgument(
        'micro_ros_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for micro-ROS agent (e.g., /dev/ttyUSB0, /dev/ttyACM0, or esp32_serial)'
    )
    micro_ros_baudrate = DeclareLaunchArgument(
        'micro_ros_baudrate',
        default_value='115200',
        description='Baud rate for micro-ROS serial connection'
    )

    # Micro-ROS Agent
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', LaunchConfiguration('micro_ros_port'),
            '-b', LaunchConfiguration('micro_ros_baudrate')
        ],
        output='screen'
    )

    return LaunchDescription([
        micro_ros_port,
        micro_ros_baudrate,
        micro_ros_agent,
    ])
