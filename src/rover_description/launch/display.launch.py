import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare("rover_description")

    # ── Launch arguments ────────────────────────────────────────────
    use_gui_arg = DeclareLaunchArgument(
        name="use_gui",
        default_value="false",
        choices=["true", "false"],
        description="Use joint_state_publisher_gui instead of joint_state_publisher",
    )

    use_rviz_arg = DeclareLaunchArgument(
        name="use_rviz",
        default_value="false",
        choices=["true", "false"],
        description="Launch RViz2",
    )

    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=PathJoinSubstitution([pkg_share, "rviz", "display.rviz"]),
        description="Path to RViz2 config file",
    )

    # ── Robot description (xacro → URDF string) ─────────────────────
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "rover.urdf.xacro"])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", urdf_file]),
            value_type=str,
        )
    }

    # ── Nodes ────────────────────────────────────────────────────────

    # Publishes the URDF to /robot_description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Publishes joint states — GUI version (slider panel)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    # Publishes joint states — headless version (all joints at zero)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_gui")),
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            use_gui_arg,
            use_rviz_arg,
            rviz_config_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
