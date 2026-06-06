import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Define package name
    package_name = 'rover_description'
    
    # Get package share directory
    pkg_share = get_package_share_directory(package_name)
    
    # Path to the Xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    
    # Path to the RViz config
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf.rviz')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    # Define LaunchDescription
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_use_rviz)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    
    return ld
