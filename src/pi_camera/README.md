# pi_camera — ROS2 Package for Raspberry Pi Camera Module 3 Wide

ROS2 Jazzy package for the IMX708 Wide camera on Raspberry Pi 5 running Ubuntu 24.04.

## Topics Published

| Topic | Type | Description |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Raw frames (use for CV, SLAM) |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Compressed (use for streaming to remote PC) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Calibration metadata |

---

## Install

```bash
# 1. Copy package into your ROS2 workspace
cp -r pi_camera ~/ros2_ws/src/

# 2. Build
cd ~/ros2_ws
colcon build --packages-select pi_camera
source install/setup.bash
```

---

## Launch

```bash
# Default — 1536x864 high speed
ros2 launch pi_camera camera.launch.py

# SLAM (low load, 1280x720)
ros2 launch pi_camera camera.launch.py profile:=slam

# Object detection (fastest, 640x480)
ros2 launch pi_camera camera.launch.py profile:=detection

# Full resolution (1536x864)
ros2 launch pi_camera camera.launch.py profile:=balanced

# Custom resolution
ros2 launch pi_camera camera.launch.py width:=1920 height:=1080
```

---

## Resolution Profiles

| Profile | Resolution | FPS | Use case |
|---|---|---|---|
| `high_speed` | 1536x864 | ~30 | General purpose (default) |
| `balanced` | 2304x1296 | ~56 | High frame rate |
| `full_res` | 4608x2592 | ~14 | Max quality photos |
| `slam` | 1280x720 | ~30 | SLAM / mapping (low load) |
| `detection` | 640x480 | ~30 | Object detection (lightest) |

---

## Switch Profile at Runtime (no restart needed)

```bash
ros2 param set /camera profile slam
```

---

## Verify it's working

```bash
ros2 topic list
ros2 topic hz /camera/image_raw

# View live stream
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw
```

---

## Record rosbag

```bash
ros2 bag record /camera/image_raw /camera/camera_info
```

---

## Autostart on boot (systemd)

Create `/etc/systemd/system/pi_camera.service`:

```ini
[Unit]
Description=Pi Camera ROS2 Node
After=network.target

[Service]
User=lucy
Environment="LD_PRELOAD=/usr/local/lib/aarch64-linux-gnu/libcamera.so.0.7.1"
Environment="LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu"
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source /home/lucy/ros2_ws/install/setup.bash && ros2 launch pi_camera camera.launch.py'
Restart=always

[Install]
WantedBy=multi-user.target
```

Then:
```bash
sudo systemctl enable pi_camera
sudo systemctl start pi_camera
```

---

## Camera Calibration (recommended for SLAM/CV)

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image_raw
```

Save the resulting YAML to:
`~/.ros/camera_info/imx708_wide_1536x864.yaml`
