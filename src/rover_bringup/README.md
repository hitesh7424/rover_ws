# Rover Bringup Package

This package provides launch files to start all rover robot subsystems in the correct order.

## Overview

The rover consists of:
- **Motor Controller**: ESP32 micro-ROS agent for motor control and odometry
- **LiDAR**: YDLiDAR sensor for obstacle detection and mapping
- **Time-of-Flight (ToF)**: Close-range proximity sensor
- **Camera**: Raspberry Pi camera for vision processing

## Launch Files

### `rover_full.launch.py` (Complete Bringup)
Launches all robot subsystems: micro-ROS agent, LiDAR, ToF sensor, and optionally the camera.

```bash
# Launch everything
ros2 launch rover_bringup rover_full.launch.py

# Launch without camera (default)
ros2 launch rover_bringup rover_full.launch.py start_camera:=false

# Launch only micro-ROS and LiDAR
ros2 launch rover_bringup rover_full.launch.py start_tof:=false start_camera:=false
```

**Launch Arguments:**
- `start_micro_ros` (default: `true`) - Start the micro-ROS agent for motor control
- `start_sensors` (default: `true`) - Enable sensor group
- `start_lidar` (default: `true`) - Enable YDLiDAR
- `start_tof` (default: `true`) - Enable Time-of-Flight sensor
- `start_camera` (default: `false`) - Enable Raspberry Pi camera

### `sensors_only.launch.py` (Sensors Only)
Launches only the sensor subsystems without the motor controller. Useful for testing sensors independently.

```bash
# Launch all sensors
ros2 launch rover_bringup sensors_only.launch.py

# Launch only LiDAR and ToF (no camera)
ros2 launch rover_bringup sensors_only.launch.py camera:=false

# Launch only LiDAR
ros2 launch rover_bringup sensors_only.launch.py tof:=false camera:=false
```

**Launch Arguments:**
- `lidar` (default: `true`) - Enable YDLiDAR
- `tof` (default: `true`) - Enable Time-of-Flight sensor
- `camera` (default: `false`) - Enable Raspberry Pi camera

### `motors_only.launch.py` (Motors Only)
Launches only the motor controller without sensors. Useful for testing motor control independently.

```bash
ros2 launch rover_bringup motors_only.launch.py
```

## Topics Published by Rover

After launching, the following topics are available:

### From Motor Controller (Micro-ROS)
- `/odom` - Odometry data (position and velocity)
- `/battery_state` - Battery voltage state
- `/rover/encoder_ticks` - Raw encoder tick counts
- `/rover/motor_pwm` - Motor PWM output values

### From LiDAR
- `/scan` - Laser scan data for obstacle detection

### From ToF Sensor
- `/tof/distance` - Distance measurement in meters

### From Camera
- `/image_raw` - Raw camera image stream

## Topics Subscribed by Rover

- `/cmd_vel` - Velocity commands (linear.x and angular.z)

## ROS 2 Nodes Architecture

```
rover_full
├── micro_ros_agent (communicates with ESP32)
│   ├── Publishes: /odom, /battery_state, /rover/encoder_ticks, /rover/motor_pwm
│   └── Subscribes: /cmd_vel
├── ydlidar_ros2_driver
│   └── Publishes: /scan
├── tof_sensor
│   └── Publishes: /tof/distance
└── camera (optional)
    └── Publishes: /image_raw
```

## Setup Requirements

### Hardware
- ESP32 microcontroller with rover_microros firmware loaded
- YDLiDAR X2 or X4 sensor
- Time-of-Flight sensor (e.g., VL53L0X)
- Raspberry Pi camera (optional)

### Prerequisites
Before launching, ensure:

1. **Micro-ROS Agent is running** (on the host PC or embedded system):
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```
   Or use the dedicated micro_ros_agent launch file.

2. **Serial ports are configured** correctly:
   - Motor controller: typically `/dev/ttyUSB0` or `/dev/ttyACM0`
   - LiDAR: check with `ls /dev/tty*`

3. **udev rules are set** (if needed for device access):
   ```bash
   sudo cat /etc/udev/rules.d/99-usb-ports.rules
   ```

## Usage Examples

### Complete Rover Startup
```bash
# Terminal 1: Source ROS setup
source install/setup.bash

# Terminal 2: Launch complete robot
ros2 launch rover_bringup rover_full.launch.py

# Terminal 3: Send motor commands
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"
```

### Test Sensors Only
```bash
ros2 launch rover_bringup sensors_only.launch.py

# In another terminal, view sensor data
ros2 topic echo /scan          # View LiDAR scan
ros2 topic echo /tof/distance  # View ToF reading
```

### Test Motor Control
```bash
ros2 launch rover_bringup motors_only.launch.py

# In another terminal, send velocity command
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"
```

## Troubleshooting

### Micro-ROS agent fails to connect
- Check USB connection: `ls -l /dev/tty*`
- Verify ESP32 firmware is loaded correctly
- Check baud rate settings (typically 115200)

### LiDAR not publishing scan data
- Verify USB connection to LiDAR
- Check LiDAR port configuration in `ydlidar_ros2_driver`
- View debug info: `ros2 topic echo /scan`

### No odometry data
- Ensure motor controller is connected and running
- Check micro-ROS agent is receiving data from ESP32
- Verify encoders are properly wired

## Configuration

Each sensor package may have configuration files in their respective directories:
- `pi_camera/config/` - Camera parameters
- `tof_sensor/config/` - ToF sensor calibration
- `ydlidar_ros2_driver/config/` - LiDAR parameters

Modify these files before launching for custom configurations.

## Development

To add new components or modify the bringup sequence:

1. Edit the relevant launch file in `launch/`
2. Add new launch arguments as needed
3. Rebuild the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select rover_bringup
   source install/setup.bash
   ```

## References

- [ROS 2 Launch Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [Micro-ROS Documentation](https://micro.ros.org/)
- [YDLiDAR ROS 2 Driver](https://github.com/YDLIDAR/ydlidar_ros2_driver)
