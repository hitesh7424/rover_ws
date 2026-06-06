
# ============================================================================
# Micro-ROS Setup Commands
# ============================================================================

# 1. INITIAL SETUP
# ============================================================================
cd ~/ros2_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# 2. INSTALL DEPENDENCIES
# ============================================================================
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y

# 3. BUILD THE WORKSPACE
# ============================================================================
colcon build
source install/local_setup.bash

# 4. BUILD THE AGENT
# ============================================================================
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh

# 5. RUN THE AGENT
# ============================================================================
# Option A: WiFi connection
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888

# Option B: Serial connection
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

ros2 service call /rover/system/reboot std_srvs/srv/Trigger