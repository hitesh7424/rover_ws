#!/usr/bin/env python3
"""
pi_camera - Raspberry Pi Camera Module 3 Wide ROS2 node
Wraps camera_ros with selectable resolution/fps profiles
and adds useful diagnostics for robotics use.
"""

import subprocess
import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult


# ---------------------------------------------------------------------------
# Resolution / FPS profiles
# ---------------------------------------------------------------------------
PROFILES = {
    'high_speed':   {'width': 1536,  'height': 864,  'label': '1536x864  @ ~30fps  (high speed, low load)'},
    'balanced':     {'width': 2304,  'height': 1296, 'label': '2304x1296 @ ~56fps  (balanced)'},
    'full_res':     {'width': 4608,  'height': 2592, 'label': '4608x2592 @ ~14fps  (full resolution)'},
    'slam':         {'width': 1280,  'height': 720,  'label': '1280x720  @ ~30fps  (lightweight for SLAM)'},
    'detection':    {'width': 640,   'height': 480,  'label': '640x480   @ ~30fps  (fast object detection)'},
}


class PiCameraNode(Node):
    """
    Thin ROS2 wrapper that launches camera_ros/camera_node as a subprocess
    using the correct libcamera libraries, then monitors it.

    Topics published (by the underlying camera_ros node):
      /camera/image_raw              - sensor_msgs/Image
      /camera/image_raw/compressed   - sensor_msgs/CompressedImage
      /camera/camera_info            - sensor_msgs/CameraInfo

    Parameters (set at launch via -p or launch file):
      profile       (string)  : one of high_speed | balanced | full_res | slam | detection
      width         (int)     : override width  (ignored if profile is set)
      height        (int)     : override height (ignored if profile is set)
      camera_id     (int)     : camera index (default 0)
      format        (string)  : pixel format  (default RGB888)
    """

    def __init__(self):
        super().__init__('pi_camera')

        # ---- declare parameters ----
        self.declare_parameter(
            'profile', 'detection',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Resolution profile: ' + ' | '.join(PROFILES.keys())
            )
        )
        self.declare_parameter('width',     0,       ParameterDescriptor(description='Override width  (0 = use profile)'))
        self.declare_parameter('height',    0,       ParameterDescriptor(description='Override height (0 = use profile)'))
        self.declare_parameter('camera_id', 0,       ParameterDescriptor(description='Camera index'))
        self.declare_parameter('format',    'RGB888', ParameterDescriptor(description='Pixel format'))

        self.add_on_set_parameters_callback(self._on_params_changed)

        # ---- resolve resolution ----
        profile_name = self.get_parameter('profile').value
        w_override   = self.get_parameter('width').value
        h_override   = self.get_parameter('height').value
        self._camera_id = self.get_parameter('camera_id').value
        self._format    = self.get_parameter('format').value

        if w_override > 0 and h_override > 0:
            self._width  = w_override
            self._height = h_override
            self.get_logger().info(f'Using custom resolution: {self._width}x{self._height}')
        else:
            profile = PROFILES.get(profile_name, PROFILES['detection'])
            self._width  = profile['width']
            self._height = profile['height']
            self.get_logger().info(f'Profile [{profile_name}]: {profile["label"]}')

        self._print_available_profiles()

        # ---- launch camera_ros subprocess ----
        self._proc = None
        self._launch_camera_ros()

        # ---- watchdog timer (restarts subprocess if it dies) ----
        self._watchdog = self.create_timer(5.0, self._watchdog_cb)

    # ------------------------------------------------------------------
    def _print_available_profiles(self):
        self.get_logger().info('Available profiles (use -p profile:=<name> to change):')
        for name, p in PROFILES.items():
            self.get_logger().info(f'  {name:12s} -> {p["label"]}')

    # ------------------------------------------------------------------
    def _build_cmd(self):
        """Build the camera_ros command with correct library paths."""
        local_lib = '/usr/local/lib/aarch64-linux-gnu'
        env_patch = {
            **os.environ,
            'LD_PRELOAD':    f'{local_lib}/libcamera.so.0.7.1',
            'LD_LIBRARY_PATH': f'{local_lib}:' + os.environ.get('LD_LIBRARY_PATH', ''),
        }

        cmd = [
            sys.executable, '-m', 'ros2', 'run', 'camera_ros', 'camera_node',
            '--ros-args',
            '-p', f'width:={self._width}',
            '-p', f'height:={self._height}',
            '-p', f'camera:={self._camera_id}',
            '-p', f'format:={self._format}',
        ]
        return cmd, env_patch

    def _launch_camera_ros(self):
        local_lib = '/usr/local/lib/aarch64-linux-gnu'
        env = {
            **os.environ,
            'LD_PRELOAD':     f'{local_lib}/libcamera.so.0.7.1',
            'LD_LIBRARY_PATH': f'{local_lib}:' + os.environ.get('LD_LIBRARY_PATH', ''),
        }

        cmd = [
            'ros2', 'run', 'camera_ros', 'camera_node',
            '--ros-args',
            '-p', f'width:={self._width}',
            '-p', f'height:={self._height}',
            '-p', f'camera:={self._camera_id}',
            '-p', f'format:={self._format}',
        ]

        self.get_logger().info(f'Launching camera_ros at {self._width}x{self._height} format={self._format}')
        self._proc = subprocess.Popen(cmd, env=env)
        self.get_logger().info(f'camera_ros PID: {self._proc.pid}')

    # ------------------------------------------------------------------
    def _watchdog_cb(self):
        if self._proc is None:
            return
        ret = self._proc.poll()
        if ret is not None:
            self.get_logger().warn(f'camera_ros exited with code {ret}, restarting...')
            self._launch_camera_ros()

    # ------------------------------------------------------------------
    def _on_params_changed(self, params):
        """Allow hot-swapping profile at runtime via ros2 param set."""
        restart_needed = False
        for p in params:
            if p.name == 'profile' and p.value in PROFILES:
                profile = PROFILES[p.value]
                self._width  = profile['width']
                self._height = profile['height']
                self.get_logger().info(f'Switching to profile [{p.value}]: {profile["label"]}')
                restart_needed = True
            elif p.name == 'width' and p.value > 0:
                self._width = p.value
                restart_needed = True
            elif p.name == 'height' and p.value > 0:
                self._height = p.value
                restart_needed = True
            elif p.name == 'format':
                self._format = p.value
                restart_needed = True

        if restart_needed:
            self._restart_camera()

        return SetParametersResult(successful=True)

    def _restart_camera(self):
        if self._proc and self._proc.poll() is None:
            self.get_logger().info('Stopping camera_ros for restart...')
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._launch_camera_ros()

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self._proc and self._proc.poll() is None:
            self.get_logger().info('Shutting down camera_ros...')
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        super().destroy_node()


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
