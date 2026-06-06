#!/usr/bin/env python3
"""
tof_node — VL53L5CX 8x8 ToF sensor ROS2 driver
================================================
Publishes:
  /tof/pointcloud   sensor_msgs/PointCloud2   — 3D point cloud for RViz / SLAM
  /tof/scan         sensor_msgs/LaserScan     — middle row as 1D scan (SLAM-ready)
  /tof/raw          std_msgs/Float32MultiArray — raw 8x8 distance grid in mm

Parameters:
  i2c_bus       (int)   : I2C bus number        (default: 1)
  i2c_addr      (int)   : sensor I2C address     (default: 0x29)
  lpn_pin       (int)   : LPN GPIO pin (-1 = not used) (default: -1)
  resolution    (int)   : 16 (4x4) or 64 (8x8)  (default: 64)
  frequency_hz  (int)   : ranging frequency      (default: 15)
  frame_id      (str)   : TF frame name          (default: tof_frame)
  h_fov_deg     (float) : horizontal FoV degrees (default: 63.0)
  v_fov_deg     (float) : vertical FoV degrees   (default: 63.0)
  max_range_m   (float) : clip distances beyond  (default: 4.0)
  min_range_m   (float) : clip distances below   (default: 0.02)
  mount_height_m(float) : sensor height from ground (default: 0.15)
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import struct


class ToFNode(Node):

    def __init__(self):
        super().__init__('tof_node')

        # ---- parameters ----
        self.declare_parameter('i2c_bus',        1)
        self.declare_parameter('i2c_addr',       0x29)
        self.declare_parameter('lpn_pin',        -1)
        self.declare_parameter('resolution',     64)
        self.declare_parameter('frequency_hz',   15)
        self.declare_parameter('frame_id',       'tof_frame')
        self.declare_parameter('h_fov_deg',      63.0)
        self.declare_parameter('v_fov_deg',      63.0)
        self.declare_parameter('max_range_m',    4.0)
        self.declare_parameter('min_range_m',    0.02)
        self.declare_parameter('mount_height_m', 0.15)

        self._resolution     = self.get_parameter('resolution').value
        self._freq           = self.get_parameter('frequency_hz').value
        self._frame_id       = self.get_parameter('frame_id').value
        self._h_fov          = math.radians(self.get_parameter('h_fov_deg').value)
        self._v_fov          = math.radians(self.get_parameter('v_fov_deg').value)
        self._max_range      = self.get_parameter('max_range_m').value
        self._min_range      = self.get_parameter('min_range_m').value
        self._mount_height   = self.get_parameter('mount_height_m').value
        self._lpn_pin        = self.get_parameter('lpn_pin').value

        # grid size (4x4 or 8x8)
        self._grid = 8 if self._resolution == 64 else 4

        # ---- precompute ray angles for each zone ----
        self._angles = self._compute_angles()

        # ---- publishers ----
        self._pub_pc   = self.create_publisher(PointCloud2,      '/tof/pointcloud', 10)
        self._pub_scan = self.create_publisher(LaserScan,        '/tof/scan',       10)
        self._pub_raw  = self.create_publisher(Float32MultiArray, '/tof/raw',       10)

        # ---- init sensor ----
        self._sensor = self._init_sensor()

        # ---- polling timer ----
        self._timer = self.create_timer(1.0 / self._freq, self._poll)
        self.get_logger().info(
            f'ToF node started | grid={self._grid}x{self._grid} '
            f'freq={self._freq}Hz frame={self._frame_id}'
        )

    # ------------------------------------------------------------------
    def _compute_angles(self):
        """
        Precompute (az, el) angles for each of the grid zones.
        Returns array of shape (grid, grid, 2) where [..., 0]=azimuth, [..., 1]=elevation.
        The VL53L5CX zones are ordered row-major, top-left to bottom-right
        when sensor faces forward.
        """
        g = self._grid
        angles = np.zeros((g, g, 2))
        for row in range(g):
            for col in range(g):
                # azimuth: left to right  (-h_fov/2 to +h_fov/2)
                az = -self._h_fov / 2 + (col + 0.5) * self._h_fov / g
                # elevation: top to bottom (+v_fov/2 to -v_fov/2)
                el = self._v_fov / 2 - (row + 0.5) * self._v_fov / g
                angles[row, col] = [az, el]
        return angles

    # ------------------------------------------------------------------
    def _init_sensor(self):
        try:
            import vl53l5cx_ctypes as vl53
        except ImportError:
            self.get_logger().fatal(
                'vl53l5cx_ctypes not found! '
                'Install with: pip3 install vl53l5cx-ctypes --break-system-packages'
            )
            raise

        # optional LPN pin control
        if self._lpn_pin >= 0:
            try:
                import lgpio
                h = lgpio.gpiochip_open(0)
                lgpio.gpio_claim_output(h, self._lpn_pin)
                lgpio.gpio_write(h, self._lpn_pin, 1)
                time.sleep(0.1)
                self.get_logger().info(f'LPN pin {self._lpn_pin} set HIGH')
            except Exception as e:
                self.get_logger().warn(f'Could not set LPN pin: {e}')

        self.get_logger().info('Initializing VL53L5CX sensor...')
        sensor = vl53.VL53L5CX()
        sensor.set_resolution(self._resolution)
        sensor.set_ranging_frequency_hz(self._freq)
        sensor.start_ranging()
        self.get_logger().info('Sensor ranging started.')
        return sensor

    # ------------------------------------------------------------------
    def _poll(self):
        if not self._sensor.data_ready():
            return

        data = self._sensor.get_data()
        now  = self.get_clock().now().to_msg()

        # reshape to (grid, grid), convert mm -> m, clip
        raw = np.array(data.distance_mm[:self._grid * self._grid], dtype=np.float32)
        raw = raw.reshape((self._grid, self._grid)) / 1000.0
        raw = np.fliplr(raw)  # flip horizontally to match physical orientation
        raw = np.clip(raw, self._min_range, self._max_range)

        self._publish_raw(raw, now)
        self._publish_pointcloud(raw, now)
        self._publish_laserscan(raw, now)

    # ------------------------------------------------------------------
    def _publish_raw(self, grid_m, stamp):
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='row',    size=self._grid, stride=self._grid * self._grid),
            MultiArrayDimension(label='col',    size=self._grid, stride=self._grid),
        ]
        msg.data = (grid_m * 1000.0).flatten().tolist()  # back to mm for raw
        self._pub_raw.publish(msg)

    # ------------------------------------------------------------------
    def _publish_pointcloud(self, grid_m, stamp):
        """
        Convert each zone distance into an (x, y, z) 3D point.
        Convention: x=forward, y=left, z=up
        """
        points = []
        g = self._grid

        for row in range(g):
            for col in range(g):
                d = float(grid_m[row, col])
                if d <= self._min_range or d >= self._max_range:
                    continue

                az = self._angles[row, col, 0]
                el = self._angles[row, col, 1]

                x = d * math.cos(el) * math.cos(az)
                y = d * math.cos(el) * math.sin(az)
                z = d * math.sin(el)

                points.append((x, y, z))

        if not points:
            return

        # pack into PointCloud2
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
        data = bytearray()
        for p in points:
            data += struct.pack('fff', *p)

        msg = PointCloud2()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._frame_id
        msg.height          = 1
        msg.width           = len(points)
        msg.fields          = fields
        msg.is_bigendian    = False
        msg.point_step      = point_step
        msg.row_step        = point_step * len(points)
        msg.data            = bytes(data)
        msg.is_dense        = True

        self._pub_pc.publish(msg)

    # ------------------------------------------------------------------
    def _publish_laserscan(self, grid_m, stamp):
        """
        Use the middle row of the ToF grid as a 1D LaserScan.
        This makes it usable directly with SLAM toolbox / Nav2.
        """
        g       = self._grid
        mid_row = g // 2
        ranges  = [float(grid_m[mid_row, col]) for col in range(g)]

        msg = LaserScan()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._frame_id
        msg.angle_min       = -self._h_fov / 2
        msg.angle_max       =  self._h_fov / 2
        msg.angle_increment = self._h_fov / g
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / self._freq
        msg.range_min       = self._min_range
        msg.range_max       = self._max_range
        msg.ranges          = ranges

        self._pub_scan.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            self._sensor.stop_ranging()
            self.get_logger().info('Sensor stopped.')
        except Exception:
            pass
        super().destroy_node()


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ToFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
