# Rover Micro-ROS Architecture

## File Separation

The codebase has been split into two modules for maintainability:

### 1. **motor_controller.h** — Motor Control & PID Loop
**Status: Ready for rebuild**

Contains all low-level motor control logic:
- **Motor Driver Functions:**
  - `applyDeadband()` — handles PWM deadband compensation
  - `setMotorA() / setMotorB()` — direct motor PWM control
  - `stopMotors()` — emergency stop

- **PID Controller:**
  - `runController()` — main velocity control loop (runs every 10ms)
  - Encoder reading & speed calculation
  - Odometry computation (differential drive kinematics)
  - PI feedback with filtered D-term
  - Anti-windup integrator protection

- **Configuration (tunable constants):**
  - PWM settings (frequency, resolution, deadband)
  - Robot geometry (wheel diameter, wheelbase, ticks per revolution)
  - PID gains (KP, KI, KD)
  - Control intervals

- **Interface Functions:**
  - `setTargetVelocity(linear_x, angular_z)` — differential drive kinematics conversion
  - `readVoltage()` — battery voltage reading
  - `initMotorController()` — hardware initialization

- **Public State (shared with main):**
  - Encoder counts: `encA_count`, `encB_count`
  - Odometry: `pose_x`, `pose_y`, `pose_yaw`, `vel_lin`, `vel_ang`
  - Motor outputs: `pwmA_out`, `pwmB_out`
  - Actual speeds: `actualA_ms`, `actualB_ms`

### 2. **rover_microros.ino** — ROS 2 Communication
**Status: Stable**

Handles all ROS2 and agent communication:
- ROS2 node, publishers, subscribers setup
- Topic management:
  - **Publishers:** `/odom`, `/battery_state`, `/rover/encoder_ticks`, `/rover/motor_pwm`
  - **Subscribers:** `/cmd_vel`
- Connection state machine (WAITING → AVAILABLE → CONNECTED)
- Agent ping/reconnect logic
- LED status indication
- Battery monitoring
- Main event loop

**Key Functions:**
- `cmdvel_callback()` — calls `setTargetVelocity()` from motor_controller.h
- `publishOdom() / publishBattery() / publishEncoders() / publishPwm()` — publish measurements
- `createEntities() / destroyEntities()` — ROS lifecycle management
- `updateLed()` — connection status indicator

## Data Flow

```
cmd_vel ROS topic
      ↓
cmdvel_callback()
      ↓
setTargetVelocity() [motor_controller.h]
  ├→ converts (linear.x, angular.z) → (vLeft, vRight)
  └→ sets targetA_ms, targetB_ms
      ↓
runController() [motor_controller.h] — runs every 10ms
  ├→ reads encA_count, encB_count
  ├→ calculates speeds & odometry
  ├→ PI control
  ├→ updates pwmA_out, pwmB_out
  ├→ updates pose_x, pose_y, pose_yaw
  └→ updates vel_lin, vel_ang
      ↓
setMotorA/B()
  ├→ applies deadband
  └→ writes PWM to motor driver
      ↓
Publishers (in main loop)
  ├→ publishOdom() [uses pose_*, vel_*]
  ├→ publishBattery()
  ├→ publishEncoders() [uses encA_count, encB_count]
  └→ publishPwm() [uses pwmA_out, pwmB_out]
```

## Compilation

The Arduino IDE will automatically include `motor_controller.h` when compiling `rover_microros.ino` if they're in the same directory.

To compile:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32 rover_microros/
```

## Rebuild Instructions

If modifying the PID controller, edit only `motor_controller.h`:

1. Adjust tuning constants (KP, KI, KD, etc.)
2. Modify `runController()` logic if needed
3. Recompile and upload

The ROS communication layer in `rover_microros.ino` remains independent and doesn't need changes.
