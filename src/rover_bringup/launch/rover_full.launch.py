"""
Complete rover bringup launch file.
Launches all robot subsystems in the correct order:
1. Micro-ROS agent (communicates with ESP32 motor controller)
2. Sensors (LiDAR, ToF, Camera)
3. Robot state publisher
4. Navigation stack (optional)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os
import subprocess  

def generate_launch_description():
    """Generate the rover complete bringup launch description."""

    # Declare launch arguments
    start_micro_ros = DeclareLaunchArgument(
        'start_micro_ros',
        default_value='true',
        description='Start the micro-ROS agent'
    )
    micro_ros_port = DeclareLaunchArgument(
        'micro_ros_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for micro-ROS agent (e.g., /dev/ttyUSB0, /dev/ttyACM0, or esp32_serial)'
    )
    micro_ros_baudrate = DeclareLaunchArgument(
        'micro_ros_baudrate',
        default_value='115200',
        description='Baud rate for micro-ROS serial connection'
    )
    start_lidar = DeclareLaunchArgument(
        'start_lidar',
        default_value='true',
        description='Start the YDLiDAR sensor'
    )
    start_tof = DeclareLaunchArgument(
        'start_tof',
        default_value='false',
        description='Start the Time-of-Flight sensor'
    )
    start_camera = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description='Start the Raspberry Pi camera'
    )
    start_webbridge = DeclareLaunchArgument(
        'start_webbridge',
        default_value='true',
        description='Start the ROS 2 web bridge for websocket communication'
    )
    webbridge_port = DeclareLaunchArgument(
        'webbridge_port',
        default_value='9090',
        description='Port for the web bridge websocket server'
    )
    start_webserver = DeclareLaunchArgument(
        'start_webserver',
        default_value='false',
        description='Start the web server for the UI'
    )
    webserver_port = DeclareLaunchArgument(
        'webserver_port',
        default_value='8000',
        description='Port for the web server'
    )
    webserver_dir = DeclareLaunchArgument(
        'webserver_dir',
        default_value=os.path.join(os.path.dirname(__file__), '..', '..', '..', 'webpage'),
        description='Directory containing the web UI files'
    )

    # Get package paths
    ydlidar_dir = FindPackageShare('ydlidar_ros2_driver')
    tof_sensor_dir = FindPackageShare('tof_sensor')
    pi_camera_dir = FindPackageShare('pi_camera')
    micro_ros_agent_dir = FindPackageShare('micro_ros_agent')
    rosbridge_dir = FindPackageShare('rosbridge_server')

    # Micro-ROS Agent (as executable process)
    micro_ros_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                micro_ros_agent_dir, 'launch',
                'micro_ros_agent_launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_micro_ros'))
    )

    # YDLiDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ydlidar_dir, 'launch',
                'ydlidar.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_lidar'))
    )

    # Time-of-Flight Sensor
    tof_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                tof_sensor_dir, 'launch',
                'tof.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_tof'))
    )

    # Pi Camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pi_camera_dir, 'launch',
                'camera.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_camera'))
    )
    
    # ROS 2 Web Bridge (WebSocket server for web UI)
    webbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                rosbridge_dir,
                'launch',
                'rosbridge_websocket_launch.xml'
            ])
        ),
        condition=IfCondition(
            LaunchConfiguration('start_webbridge')
        )
    )

    return LaunchDescription([
        # Arguments
        start_micro_ros,
        micro_ros_port,
        micro_ros_baudrate,
        start_lidar,
        start_tof,
        start_camera,
        # Executables and launch files
        micro_ros_agent,
        lidar_launch,
        tof_launch,
        camera_launch,
        
        # Arguments
        start_webbridge,
        webbridge_port,
        start_webserver,
        webserver_port,
        webserver_dir,
        # Executables and launch files
        webbridge,
        # webserver,
    ])
