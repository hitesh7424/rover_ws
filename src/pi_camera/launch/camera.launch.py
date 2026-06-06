"""
pi_camera launch file
---------------------
Usage examples:

  # Default (high_speed 1536x864)
  ros2 launch pi_camera camera.launch.py

  # SLAM-friendly low resolution
  ros2 launch pi_camera camera.launch.py profile:=slam

  # Full resolution
  ros2 launch pi_camera camera.launch.py profile:=full_res

  # Custom resolution
  ros2 launch pi_camera camera.launch.py width:=1920 height:=1080

  # Object detection mode
  ros2 launch pi_camera camera.launch.py profile:=detection
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LOCAL_LIB = '/usr/local/lib/aarch64-linux-gnu'

# Patch environment so camera_ros uses the correct libcamera build
CAMERA_ENV = {
    **os.environ,
    'LD_PRELOAD':     f'{LOCAL_LIB}/libcamera.so.0.7.1',
    'LD_LIBRARY_PATH': f'{LOCAL_LIB}:' + os.environ.get('LD_LIBRARY_PATH', ''),
}


def generate_launch_description():

    # ---- launch arguments ----
    profile_arg = DeclareLaunchArgument(
        'profile', default_value='detection',
        description='Resolution profile: high_speed | balanced | full_res | slam | detection'
    )
    width_arg = DeclareLaunchArgument(
        'width', default_value='0',
        description='Override width (0 = use profile)'
    )
    height_arg = DeclareLaunchArgument(
        'height', default_value='0',
        description='Override height (0 = use profile)'
    )
    format_arg = DeclareLaunchArgument(
        'format', default_value='RGB888',
        description='Pixel format: RGB888 | BGR888 | XRGB8888 | YUYV'
    )
    camera_id_arg = DeclareLaunchArgument(
        'camera_id', default_value='0',
        description='Camera index (0 for CAM0 port)'
    )

    # ---- profile -> resolution lookup ----
    PROFILES = {
        'high_speed':  (1536,  864),
        'balanced':    (2304, 1296),
        'full_res':    (4608, 2592),
        'slam':        (1280,  720),
        'detection':   (640,   480),
    }

    def launch_camera(context):
        profile  = LaunchConfiguration('profile').perform(context)
        width    = int(LaunchConfiguration('width').perform(context))
        height   = int(LaunchConfiguration('height').perform(context))
        fmt      = LaunchConfiguration('format').perform(context)
        cam_id   = LaunchConfiguration('camera_id').perform(context)

        if width == 0 or height == 0:
            width, height = PROFILES.get(profile, PROFILES['detection'])

        return [
            LogInfo(msg=f'[pi_camera] Profile: {profile} | Resolution: {width}x{height} | Format: {fmt}'),

            # ---- camera_ros node (the actual driver) ----
            Node(
                package='camera_ros',
                executable='camera_node',
                name='camera',
                namespace='camera',
                parameters=[{
                    'width':    width,
                    'height':   height,
                    'format':   fmt,
                    'camera':   int(cam_id),
                }],
                remappings=[
                    ('~/image_raw',            '/camera/image_raw'),
                    ('~/camera_info',          '/camera/camera_info'),
                    ('~/image_raw/compressed', '/camera/image_raw/compressed'),
                ],
                additional_env=CAMERA_ENV,
                output='screen',
            ),
        ]

    return LaunchDescription([
        profile_arg,
        width_arg,
        height_arg,
        format_arg,
        camera_id_arg,
        OpaqueFunction(function=launch_camera),
    ])
