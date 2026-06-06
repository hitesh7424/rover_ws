// ════════════════════════════════════════════════════════════════
//  MOTOR CONTROLLER & PID LOOP
//  Separated module for velocity-controlled differential drive
//  Status: READY FOR REBUILD
// ════════════════════════════════════════════════════════════════

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

// ════════════════════════════════════════════════════════════════
// PINS
// ════════════════════════════════════════════════════════════════
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19
#define ENA 25
#define ENB 26

#define ENC_A_CHA 32
#define ENC_A_CHB 33
#define ENC_B_CHA 34
#define ENC_B_CHB 35

#define VOLTAGE_PIN 36
#define R1 30000.0f
#define R2  7500.0f

// ════════════════════════════════════════════════════════════════
// PWM CONFIGURATION
// ════════════════════════════════════════════════════════════════
#define PWM_FREQ      1000
#define PWM_RES       8           // 8-bit → 0–255
#define PWM_MAX       255
#define PWM_MIN_MOVE  205         // Deadband threshold

// ════════════════════════════════════════════════════════════════
// ROBOT GEOMETRY
// ════════════════════════════════════════════════════════════════
#define TICKS_PER_REV_A  116.0f
#define TICKS_PER_REV_B  116.0f
#define WHEEL_DIAMETER_M  0.065f
#define WHEEL_CIRCUM_M   (WHEEL_DIAMETER_M * (float)M_PI)
#define WHEEL_BASE_M      0.23f

#define M_PER_TICK_A  (WHEEL_CIRCUM_M / TICKS_PER_REV_A)
#define M_PER_TICK_B  (WHEEL_CIRCUM_M / TICKS_PER_REV_B)

// ════════════════════════════════════════════════════════════════
// VELOCITY CONTROLLER TUNING
// ════════════════════════════════════════════════════════════════
#define MAX_WHEEL_SPEED   1.26f
#define FF_GAIN           0.0f
#define KP                1000.0f
#define KI                100.0f
#define KD                0.5f
#define D_FILTER_ALPHA    0.15f
#define I_MAX             250.0f
#define ACCEL_RATE        9999.0f
#define CTRL_INTERVAL_MS  10

// ════════════════════════════════════════════════════════════════
// ENCODER STATE (volatile)
// ════════════════════════════════════════════════════════════════
volatile long encA_count = 0;
volatile long encB_count = 0;

void IRAM_ATTR ISR_ENC_A() { if (digitalRead(ENC_A_CHB)) encA_count--; else encA_count++; }
void IRAM_ATTR ISR_ENC_B() { if (digitalRead(ENC_B_CHB)) encB_count--; else encB_count++; }

// ════════════════════════════════════════════════════════════════
// CONTROLLER STATE
// ════════════════════════════════════════════════════════════════
float targetA_ms = 0.0f, targetB_ms = 0.0f;
float rampA_ms   = 0.0f, rampB_ms   = 0.0f;
float integA     = 0.0f, integB     = 0.0f;
float prevErrA   = 0.0f, prevErrB   = 0.0f;
float dFiltA     = 0.0f, dFiltB     = 0.0f;
long  lastEncA   = 0,    lastEncB   = 0;
float actualA_ms = 0.0f, actualB_ms = 0.0f;
int   pwmA_out   = 0,    pwmB_out   = 0;

unsigned long lastCtrlTime = 0;

// ════════════════════════════════════════════════════════════════
// ODOMETRY OUTPUT
// ════════════════════════════════════════════════════════════════
float pose_x   = 0.0f, pose_y   = 0.0f;
float pose_yaw = 0.0f;
float vel_lin  = 0.0f, vel_ang  = 0.0f;

// ════════════════════════════════════════════════════════════════
// MOTOR DRIVER FUNCTIONS
// ════════════════════════════════════════════════════════════════
static inline int applyDeadband(int pwm) {
  if (pwm == 0)                        return 0;
  if (pwm >  0 && pwm <  PWM_MIN_MOVE) return PWM_MIN_MOVE;
  if (pwm < 0  && pwm > -PWM_MIN_MOVE) return -PWM_MIN_MOVE;
  return pwm;
}

void setMotorA(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  pwm = applyDeadband(pwm);
  pwmA_out = pwm;
  if      (pwm > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else if (pwm < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else              { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }
  ledcWrite(ENA, abs(pwm));
}

void setMotorB(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  pwm = applyDeadband(pwm);
  pwmB_out = pwm;
  if      (pwm > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else if (pwm < 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else              { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
  ledcWrite(ENB, abs(pwm));
}

void stopMotors() {
  targetA_ms = 0; targetB_ms = 0;
  rampA_ms   = 0; rampB_ms   = 0;
  integA     = 0; integB     = 0;
  prevErrA   = 0; prevErrB   = 0;
  dFiltA     = 0; dFiltB     = 0;
  setMotorA(0);   setMotorB(0);
}

// ════════════════════════════════════════════════════════════════
// PID CONTROLLER LOOP
// ════════════════════════════════════════════════════════════════
void runController() {
  unsigned long now = millis();
  float dt = (now - lastCtrlTime) / 1000.0f;
  if (dt <= 0.0f) return;
  lastCtrlTime = now;

  // Measure actual wheel speeds
  long curA = encA_count, curB = encB_count;
  long dA = curA - lastEncA, dB = curB - lastEncB;
  lastEncA = curA; lastEncB = curB;

  actualA_ms = (dA * M_PER_TICK_A) / dt;
  actualB_ms = (dB * M_PER_TICK_B) / dt;

  // Odometry (differential drive kinematics)
  float distA = dA * M_PER_TICK_A;
  float distB = dB * M_PER_TICK_B;
  float d_c   = (distA + distB) * 0.5f;
  float d_th  = (distB - distA) / WHEEL_BASE_M;

  pose_x   += d_c * cos(pose_yaw + d_th * 0.5f);
  pose_y   += d_c * sin(pose_yaw + d_th * 0.5f);
  pose_yaw += d_th;
  while (pose_yaw >  M_PI) pose_yaw -= 2.0 * M_PI;
  while (pose_yaw < -M_PI) pose_yaw += 2.0 * M_PI;

  vel_lin = d_c / dt;
  vel_ang = d_th / dt;

  // Hard stop
  if (targetA_ms == 0.0f && targetB_ms == 0.0f) {
    rampA_ms = 0; rampB_ms = 0;
    integA   = 0; integB   = 0;
    prevErrA = 0; prevErrB = 0;
    dFiltA   = 0; dFiltB   = 0;
    setMotorA(0); setMotorB(0);
    return;
  }

  // Update setpoint (no ramping)
  rampA_ms = targetA_ms;
  rampB_ms = targetB_ms;

  // Motor A PID
  {
    float err  = rampA_ms - actualA_ms;
    integA    += err * dt;
    integA     = constrain(integA, -I_MAX, I_MAX);
    float rawD = (err - prevErrA) / dt;
    dFiltA     = D_FILTER_ALPHA * rawD + (1.0f - D_FILTER_ALPHA) * dFiltA;
    prevErrA   = err;
    setMotorA((int)(KP*err + KI*integA + KD*dFiltA));
  }

  // Motor B PID
  {
    float err  = rampB_ms - actualB_ms;
    integB    += err * dt;
    integB     = constrain(integB, -I_MAX, I_MAX);
    float rawD = (err - prevErrB) / dt;
    dFiltB     = D_FILTER_ALPHA * rawD + (1.0f - D_FILTER_ALPHA) * dFiltB;
    prevErrB   = err;
    setMotorB((int)(KP*err + KI*integB + KD*dFiltB));
  }
}

// ════════════════════════════════════════════════════════════════
// CMD_VEL CALLBACK — differential drive kinematics
// ════════════════════════════════════════════════════════════════
inline void setTargetVelocity(float linear_x, float angular_z) {
  float vLeft  = linear_x - angular_z * (WHEEL_BASE_M / 2.0f);
  float vRight = linear_x + angular_z * (WHEEL_BASE_M / 2.0f);

  targetA_ms = constrain(vLeft,  -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
  targetB_ms = constrain(vRight, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
}

// ════════════════════════════════════════════════════════════════
// VOLTAGE MONITORING
// ════════════════════════════════════════════════════════════════
float readVoltage() {
  long sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogRead(VOLTAGE_PIN); delayMicroseconds(100); }
  float v_adc = (sum / 16.0f / 4095.0f) * 3.3f;
  return v_adc * ((R1 + R2) / R2) * 1.0661f;
}

// ════════════════════════════════════════════════════════════════
// INITIALIZATION
// ════════════════════════════════════════════════════════════════
void initMotorController() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);
  stopMotors();

  pinMode(ENC_A_CHA, INPUT); pinMode(ENC_A_CHB, INPUT);
  pinMode(ENC_B_CHA, INPUT); pinMode(ENC_B_CHB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_CHA), ISR_ENC_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_CHA), ISR_ENC_B, RISING);

  pinMode(VOLTAGE_PIN, INPUT);
  lastCtrlTime = millis();
}

#endif // MOTOR_CONTROLLER_H
