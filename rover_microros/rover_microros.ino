// ════════════════════════════════════════════════════════════════
//  ROVER — micro-ROS  |  Velocity-controlled differential drive
//  Board  : ESP32 (Arduino core v3.x)
//  Agent  : ros2 run micro_ros_agent micro_ros_agent serial
//             --dev /dev/ttyUSB0 -b 115200
//
//  Architecture:
//    • motor_controller.h — separate PID/motor control module
//    • this file         — ROS 2 communication & state management
//
//  Topics published:
//    /odom                  nav_msgs/msg/Odometry         50 Hz
//    /battery_state         sensor_msgs/msg/BatteryState   1 Hz
//    /rover/encoder_ticks   std_msgs/msg/Int32MultiArray  50 Hz
//    /rover/motor_pwm       std_msgs/msg/Int32MultiArray  10 Hz
//
//  Topics subscribed:
//    /cmd_vel               geometry_msgs/msg/Twist
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/int32_multi_array.h>

// Separate motor controller module
#include "motor_controller.h"

// ════════════════════════════════════════════════════════════════
// BOARD & LED
// ════════════════════════════════════════════════════════════════
#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif

// Publishing intervals (Hz)
#define ODOM_INTERVAL_MS  20   // 50 Hz
#define BATT_INTERVAL_MS  1000 // 1 Hz
#define PWM_INTERVAL_MS   100  // 10 Hz

// Battery monitoring
#define BATT_MIN_V  10.5f
#define BATT_MAX_V  12.6f

// Connection management
#define PING_INTERVAL_MS   500
#define PING_TIMEOUT_MS    100
#define PING_ATTEMPTS        1
#define RECONNECT_DELAY_MS 2000

// ════════════════════════════════════════════════════════════════
// ROS 2 ENTITIES
// ════════════════════════════════════════════════════════════════
rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rclc_executor_t executor;

rcl_publisher_t    pub_odom, pub_batt, pub_enc, pub_pwm;
rcl_subscription_t sub_cmdvel;

nav_msgs__msg__Odometry          msg_odom;
sensor_msgs__msg__BatteryState   msg_batt;
std_msgs__msg__Int32MultiArray   msg_enc, msg_pwm;
geometry_msgs__msg__Twist        msg_cmdvel;

int32_t enc_data[2], pwm_data[2];

unsigned long lastOdomTime = 0, lastBattTime = 0;
unsigned long lastPwmTime  = 0, lastPingTime = 0;

// ════════════════════════════════════════════════════════════════
// CONNECTION STATE MACHINE
// ════════════════════════════════════════════════════════════════
enum class AgentState : uint8_t {
  WAITING, AVAILABLE, CONNECTED, DISCONNECTED
};
AgentState agentState = AgentState::WAITING;
unsigned long lastReconnectAttempt = 0;

#define RCCHECK(fn)     { if ((fn) != RCL_RET_OK) return false; }
#define RCSOFTCHECK(fn) { rcl_ret_t _r=(fn); (void)_r; }

// ════════════════════════════════════════════════════════════════
// LED STATE MACHINE
// ════════════════════════════════════════════════════════════════
unsigned long lastBlinkTime = 0;

void printState(const char* message) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] ");
  Serial.println(message);
}

void updateLed() {
  switch (agentState) {
    case AgentState::WAITING:
    case AgentState::AVAILABLE:
      if (millis() - lastBlinkTime >= 500) { lastBlinkTime = millis(); digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); }
      break;
    case AgentState::DISCONNECTED:
      if (millis() - lastBlinkTime >= 100) { lastBlinkTime = millis(); digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); }
      break;
    case AgentState::CONNECTED:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
  }
}

// ════════════════════════════════════════════════════════════════
// ROS CMD_VEL CALLBACK
// ════════════════════════════════════════════════════════════════
void cmdvel_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  float vx = (float)msg->linear.x;
  float wz = (float)msg->angular.z;
  setTargetVelocity(vx, wz);
}

// ════════════════════════════════════════════════════════════════
// PUBLISHERS
// ════════════════════════════════════════════════════════════════
void publishOdom() {
  int64_t ns = (int64_t)millis() * 1000000LL;
  msg_odom.header.stamp.sec     = (int32_t)(ns / 1000000000LL);
  msg_odom.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  msg_odom.pose.pose.position.x    = pose_x;
  msg_odom.pose.pose.position.y    = pose_y;
  msg_odom.pose.pose.position.z    = 0.0;
  msg_odom.pose.pose.orientation.x = 0.0;
  msg_odom.pose.pose.orientation.y = 0.0;
  msg_odom.pose.pose.orientation.z = sin(pose_yaw / 2.0);
  msg_odom.pose.pose.orientation.w = cos(pose_yaw / 2.0);
  msg_odom.twist.twist.linear.x    = vel_lin;
  msg_odom.twist.twist.angular.z   = vel_ang;
  RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
}

void publishBattery() {
  float v   = readVoltage();
  float pct = constrain((v - BATT_MIN_V) / (BATT_MAX_V - BATT_MIN_V), 0.0f, 1.0f);
  msg_batt.voltage               = v;
  msg_batt.percentage            = pct;
  msg_batt.present               = true;
  msg_batt.power_supply_status   = 2;
  msg_batt.power_supply_health   = (pct > 0.2f) ? 1 : 4;
  msg_batt.power_supply_technology = 3;
  RCSOFTCHECK(rcl_publish(&pub_batt, &msg_batt, NULL));
}

void publishEncoders() {
  enc_data[0] = (int32_t)encA_count;
  enc_data[1] = (int32_t)encB_count;
  RCSOFTCHECK(rcl_publish(&pub_enc, &msg_enc, NULL));
}

void publishPwm() {
  pwm_data[0] = pwmA_out;
  pwm_data[1] = pwmB_out;
  RCSOFTCHECK(rcl_publish(&pub_pwm, &msg_pwm, NULL));
}

// ════════════════════════════════════════════════════════════════
// CREATE / DESTROY ROS ENTITIES
// ════════════════════════════════════════════════════════════════
bool createEntities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "rover_node", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_odom, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"));
  RCCHECK(rclc_publisher_init_default(&pub_batt, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "/battery_state"));
  RCCHECK(rclc_publisher_init_default(&pub_enc, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/rover/encoder_ticks"));
  RCCHECK(rclc_publisher_init_default(&pub_pwm, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/rover/motor_pwm"));
  RCCHECK(rclc_subscription_init_default(&sub_cmdvel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmdvel,
    &msg_cmdvel, &cmdvel_callback, ON_NEW_DATA));

  // Odometry static fields
  msg_odom.header.frame_id.data     = (char*)"odom";
  msg_odom.header.frame_id.size     = 4;
  msg_odom.header.frame_id.capacity = 5;
  msg_odom.child_frame_id.data      = (char*)"base_link";
  msg_odom.child_frame_id.size      = 9;
  msg_odom.child_frame_id.capacity  = 10;
  msg_odom.pose.covariance[0]  = 0.01;
  msg_odom.pose.covariance[7]  = 0.01;
  msg_odom.pose.covariance[35] = 0.05;
  msg_odom.twist.covariance[0] = 0.01;
  msg_odom.twist.covariance[35]= 0.05;

  // Array message static fields
  msg_enc.data.data = enc_data; msg_enc.data.size = 2; msg_enc.data.capacity = 2;
  msg_pwm.data.data = pwm_data; msg_pwm.data.size = 2; msg_pwm.data.capacity = 2;

  lastOdomTime = lastBattTime = lastPwmTime = lastPingTime = millis();
  return true;
}

void destroyEntities() {
  rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
  rcl_publisher_fini(&pub_odom, &node);
  rcl_publisher_fini(&pub_batt, &node);
  rcl_publisher_fini(&pub_enc,  &node);
  rcl_publisher_fini(&pub_pwm,  &node);
  rcl_subscription_fini(&sub_cmdvel, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ════════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  delay(500);  // Give serial time to initialize
  printState("=== ROVER STARTING ===");
  printState("Initializing motor controller...");
  
  // Initialize motor controller (pins, encoders, PWM)
  initMotorController();
  
  printState("Setting up micro-ROS transports...");
  set_microros_transports();
  
  printState("Waiting for agent to become available...");
}

// ════════════════════════════════════════════════════════════════
// LOOP — state machine
// ════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // Velocity controller runs unconditionally at CTRL_INTERVAL_MS
  if (now - lastCtrlTime >= CTRL_INTERVAL_MS) {
    runController();
  }

  updateLed();

  switch (agentState) {

    case AgentState::WAITING: {
      unsigned long now = millis();
      if (now - lastReconnectAttempt >= RECONNECT_DELAY_MS) {
        lastReconnectAttempt = now;
        printState("WAITING: Pinging agent...");
        if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) == RMW_RET_OK) {
          printState("WAITING: Agent found! Transitioning to AVAILABLE.");
          agentState = AgentState::AVAILABLE;
        } else {
          printState("WAITING: Agent not responding, will retry...");
        }
      }
      break;
    }

    case AgentState::AVAILABLE: {
      printState("AVAILABLE: Creating ROS entities...");
      if (createEntities()) {
        printState("AVAILABLE: Entities created successfully! Transitioning to CONNECTED.");
        agentState = AgentState::CONNECTED;
      } else {
        printState("AVAILABLE: Failed to create entities, destroying and returning to WAITING.");
        destroyEntities();
        agentState = AgentState::WAITING;
        lastReconnectAttempt = millis();
      }
      break;
    }

    case AgentState::CONNECTED:
      RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

      if (now - lastOdomTime >= ODOM_INTERVAL_MS) {
        lastOdomTime = now;
        publishOdom();
        publishEncoders();
      }
      if (now - lastBattTime >= BATT_INTERVAL_MS) {
        lastBattTime = now;
        publishBattery();
      }
      if (now - lastPwmTime >= PWM_INTERVAL_MS) {
        lastPwmTime = now;
        publishPwm();
      }
      if (now - lastPingTime >= PING_INTERVAL_MS) {
        lastPingTime = now;
        if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) != RMW_RET_OK) {
          printState("CONNECTED: Agent lost! Stopping motors and transitioning to DISCONNECTED.");
          stopMotors();
          destroyEntities();
          agentState = AgentState::DISCONNECTED;
          lastReconnectAttempt = millis();
        }
      }
      break;

    case AgentState::DISCONNECTED: {
      unsigned long now = millis();
      if (now - lastReconnectAttempt >= RECONNECT_DELAY_MS) {
        printState("DISCONNECTED: Returning to WAITING state.");
        agentState = AgentState::WAITING;
        lastReconnectAttempt = now;
      }
      break;
    }
  }
}