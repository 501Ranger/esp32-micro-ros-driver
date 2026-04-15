#include <Arduino.h>
#include <Wire.h>

#include <cmath>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <builtin_interfaces/msg/time.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include "robot_config.h"

namespace {

using namespace robot_config;

constexpr float PI_F = 3.14159265358979323846f;
constexpr float RAD_PER_DEG = PI_F / 180.0f;
constexpr float STANDARD_GRAVITY = 9.80665f;
constexpr uint8_t QMI8658_ADDRESS = 0x6A;
constexpr uint8_t QMI8658_WHO_AM_I = 0x00;
constexpr uint8_t QMI8658_CTRL1 = 0x02;
constexpr uint8_t QMI8658_CTRL2 = 0x03;
constexpr uint8_t QMI8658_CTRL3 = 0x04;
constexpr uint8_t QMI8658_CTRL5 = 0x06;
constexpr uint8_t QMI8658_CTRL7 = 0x08;
constexpr uint8_t QMI8658_STATUS0 = 0x2E;
constexpr uint8_t QMI8658_TEMP_L = 0x33;
constexpr uint8_t QMI8658_ACCEL_X_L = 0x35;

enum class AgentState {
  WaitingAgent,
  AgentAvailable,
  AgentConnected,
  AgentDisconnected,
};

struct WheelMeasurement {
  int32_t ticks = 0;
  float velocity_mps = 0.0f;
};

struct ImuSample {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  float temperature_c = 0.0f;
  bool valid = false;
};

class MotorDriver {
 public:
  MotorDriver(uint8_t forward_pin, uint8_t reverse_pin, uint8_t forward_channel,
              uint8_t reverse_channel, bool inverted)
      : forward_pin_(forward_pin),
        reverse_pin_(reverse_pin),
        forward_channel_(forward_channel),
        reverse_channel_(reverse_channel),
        inverted_(inverted) {}

  void begin() const {
    pinMode(forward_pin_, OUTPUT);
    pinMode(reverse_pin_, OUTPUT);
    ledcSetup(forward_channel_, PWM_FREQUENCY, PWM_BITS);
    ledcSetup(reverse_channel_, PWM_FREQUENCY, PWM_BITS);
    ledcAttachPin(forward_pin_, forward_channel_);
    ledcAttachPin(reverse_pin_, reverse_channel_);
    stop();
  }

  void setDuty(float duty) const {
    duty = constrain(duty, -1.0f, 1.0f);
    if (inverted_) {
      duty = -duty;
    }

    if (duty != 0.0f) {
      const float magnitude = fabsf(duty);
      if (magnitude < MOTOR_MIN_EFFECTIVE_DUTY) {
        duty = copysignf(MOTOR_MIN_EFFECTIVE_DUTY, duty);
      }
    }

    const int16_t pwm = static_cast<int16_t>(roundf(fabsf(duty) * PWM_MAX));
    if (pwm == 0) {
      stop();
      return;
    }

    if (duty > 0.0f) {
      ledcWrite(forward_channel_, pwm);
      ledcWrite(reverse_channel_, 0);
      digitalWrite(reverse_pin_, LOW);
    } else {
      ledcWrite(forward_channel_, 0);
      ledcWrite(reverse_channel_, pwm);
      digitalWrite(forward_pin_, LOW);
    }
  }

  void stop() const {
    ledcWrite(forward_channel_, 0);
    ledcWrite(reverse_channel_, 0);
    digitalWrite(forward_pin_, LOW);
    digitalWrite(reverse_pin_, LOW);
  }

 private:
  uint8_t forward_pin_;
  uint8_t reverse_pin_;
  uint8_t forward_channel_;
  uint8_t reverse_channel_;
  bool inverted_;
};

class QMI8658 {
 public:
  bool begin() {
    Wire.begin(I2C_SDA, I2C_SCL, IMU_I2C_FREQUENCY);
    delay(20);

    uint8_t who_am_i = 0;
    if (!readRegister(QMI8658_WHO_AM_I, who_am_i)) {
      return false;
    }

    if (who_am_i != 0x05) {
      return false;
    }

    if (!writeRegister(QMI8658_CTRL1, 0x40)) {
      return false;
    }

    delay(10);

    const uint8_t accel_cfg = (0b001 << 4) | 0b0101;
    const uint8_t gyro_cfg = (0b100 << 4) | 0b0101;
    const uint8_t filter_cfg = (1 << 4) | (0b01 << 5) | (1 << 0) | (0b01 << 1);
    const uint8_t enable_cfg = 0b00000011;

    return writeRegister(QMI8658_CTRL2, accel_cfg) &&
           writeRegister(QMI8658_CTRL3, gyro_cfg) &&
           writeRegister(QMI8658_CTRL5, filter_cfg) &&
           writeRegister(QMI8658_CTRL7, enable_cfg);
  }

  bool calibrateGyroBias(uint16_t samples = 400) {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    uint16_t collected = 0;

    while (collected < samples) {
      ImuSample sample;
      if (read(sample)) {
        sum_x += sample.gx;
        sum_y += sample.gy;
        sum_z += sample.gz;
        ++collected;
      }
      delay(5);
    }

    gyro_bias_x_ = sum_x / samples;
    gyro_bias_y_ = sum_y / samples;
    gyro_bias_z_ = sum_z / samples;
    return true;
  }

  bool read(ImuSample &sample) {
    uint8_t status = 0;
    if (!readRegister(QMI8658_STATUS0, status)) {
      sample.valid = false;
      return false;
    }

    if ((status & 0x03) == 0) {
      sample.valid = false;
      return false;
    }

    uint8_t raw[14] = {0};
    if (!readRegisters(QMI8658_TEMP_L, raw, sizeof(raw))) {
      sample.valid = false;
      return false;
    }

    const int16_t raw_temp = static_cast<int16_t>((raw[1] << 8) | raw[0]);
    const int16_t raw_ax = static_cast<int16_t>((raw[3] << 8) | raw[2]);
    const int16_t raw_ay = static_cast<int16_t>((raw[5] << 8) | raw[4]);
    const int16_t raw_az = static_cast<int16_t>((raw[7] << 8) | raw[6]);
    const int16_t raw_gx = static_cast<int16_t>((raw[9] << 8) | raw[8]);
    const int16_t raw_gy = static_cast<int16_t>((raw[11] << 8) | raw[10]);
    const int16_t raw_gz = static_cast<int16_t>((raw[13] << 8) | raw[12]);

    constexpr float accel_lsb_per_g = 8192.0f;
    constexpr float gyro_lsb_per_dps = 128.0f;

    sample.temperature_c = static_cast<float>(raw_temp) / 256.0f;
    sample.ax = (static_cast<float>(raw_ax) / accel_lsb_per_g) * STANDARD_GRAVITY;
    sample.ay = (static_cast<float>(raw_ay) / accel_lsb_per_g) * STANDARD_GRAVITY;
    sample.az = (static_cast<float>(raw_az) / accel_lsb_per_g) * STANDARD_GRAVITY;
    sample.gx = (static_cast<float>(raw_gx) / gyro_lsb_per_dps) * RAD_PER_DEG - gyro_bias_x_;
    sample.gy = (static_cast<float>(raw_gy) / gyro_lsb_per_dps) * RAD_PER_DEG - gyro_bias_y_;
    sample.gz = (static_cast<float>(raw_gz) / gyro_lsb_per_dps) * RAD_PER_DEG - gyro_bias_z_;
    sample.valid = true;
    return true;
  }

 private:
  bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(QMI8658_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }

  bool readRegister(uint8_t reg, uint8_t &value) {
    return readRegisters(reg, &value, 1);
  }

  bool readRegisters(uint8_t reg, uint8_t *buffer, size_t length) {
    Wire.beginTransmission(QMI8658_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
      return false;
    }

    const size_t received = Wire.requestFrom(QMI8658_ADDRESS, static_cast<uint8_t>(length));
    if (received != length) {
      return false;
    }

    for (size_t i = 0; i < length; ++i) {
      buffer[i] = Wire.read();
    }
    return true;
  }

  float gyro_bias_x_ = 0.0f;
  float gyro_bias_y_ = 0.0f;
  float gyro_bias_z_ = 0.0f;
};

MotorDriver left_motor(M1_IN1, M1_IN2, LEFT_FORWARD_CHANNEL, LEFT_REVERSE_CHANNEL, LEFT_MOTOR_INVERTED);
MotorDriver right_motor(M2_IN1, M2_IN2, RIGHT_FORWARD_CHANNEL, RIGHT_REVERSE_CHANNEL, RIGHT_MOTOR_INVERTED);
QMI8658 imu_sensor;
HardwareSerial ros_serial(0);

volatile int32_t left_encoder_ticks = 0;
volatile int32_t right_encoder_ticks = 0;
volatile uint8_t left_encoder_state = 0;
volatile uint8_t right_encoder_state = 0;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t control_timer;
rclc_executor_t executor;
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t left_ticks_publisher;
rcl_publisher_t right_ticks_publisher;
rcl_publisher_t cmd_vel_echo_publisher;
rcl_publisher_t left_pwm_publisher;
rcl_publisher_t right_pwm_publisher;
rcl_subscription_t cmd_vel_subscription;

geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Int32 left_ticks_msg;
std_msgs__msg__Int32 right_ticks_msg;
std_msgs__msg__Float32 left_pwm_msg;
std_msgs__msg__Float32 right_pwm_msg;
geometry_msgs__msg__Twist cmd_vel_echo_msg;

geometry_msgs__msg__Twist target_cmd;

AgentState agent_state = AgentState::WaitingAgent;
bool ros_entities_created = false;
bool ros_messages_ready = false;
bool time_synced = false;
bool imu_ready = false;

WheelMeasurement left_wheel;
WheelMeasurement right_wheel;
ImuSample latest_imu;

unsigned long last_agent_check_ms = 0;
unsigned long last_cmd_vel_ms = 0;
unsigned long last_control_update_ms = 0;

float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_yaw = 0.0f;
float current_left_duty = 0.0f;
float current_right_duty = 0.0f;

int8_t quadratureDelta(uint8_t previous_state, uint8_t current_state) {
  static constexpr int8_t lookup_table[16] = {0, -1, 1, 0, 1, 0, 0, -1,
                                              -1, 0, 0, 1, 0, 1, -1, 0};
  return lookup_table[(previous_state << 2) | current_state];
}

uint8_t readEncoderState(uint8_t pin_a, uint8_t pin_b) {
  return static_cast<uint8_t>((digitalRead(pin_a) << 1) | digitalRead(pin_b));
}

void IRAM_ATTR onLeftEncoderChange() {
  const uint8_t current_state = readEncoderState(M1_ENC_A, M1_ENC_B);
  left_encoder_ticks += quadratureDelta(left_encoder_state, current_state);
  left_encoder_state = current_state;
}

void IRAM_ATTR onRightEncoderChange() {
  const uint8_t current_state = readEncoderState(M2_ENC_A, M2_ENC_B);
  right_encoder_ticks += quadratureDelta(right_encoder_state, current_state);
  right_encoder_state = current_state;
}

int32_t atomicReadTicks(volatile int32_t &ticks) {
  noInterrupts();
  const int32_t value = ticks;
  interrupts();
  return value;
}

builtin_interfaces__msg__Time nowRosTime() {
  builtin_interfaces__msg__Time stamp;
  stamp.sec = 0;
  stamp.nanosec = 0;

  if (time_synced) {
    const int64_t now_ns = rmw_uros_epoch_nanos();
    if (now_ns > 0) {
      stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
      stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
      return stamp;
    }
  }

  const uint64_t fallback_ns = static_cast<uint64_t>(millis()) * 1000000ULL;
  stamp.sec = static_cast<int32_t>(fallback_ns / 1000000000ULL);
  stamp.nanosec = static_cast<uint32_t>(fallback_ns % 1000000000ULL);
  return stamp;
}

void setQuaternionFromYaw(double yaw, geometry_msgs__msg__Quaternion &quat) {
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = sin(yaw * 0.5);
  quat.w = cos(yaw * 0.5);
}

void stopMotors() {
  left_motor.stop();
  right_motor.stop();
}

void applyMotorCommand(float left_velocity_mps, float right_velocity_mps) {
  float left_duty = constrain(left_velocity_mps / MAX_WHEEL_LINEAR_SPEED_MPS, -1.0f, 1.0f);
  float right_duty = constrain(right_velocity_mps / MAX_WHEEL_LINEAR_SPEED_MPS, -1.0f, 1.0f);

  if (left_duty != 0.0f && fabsf(left_duty) < MOTOR_MIN_EFFECTIVE_DUTY) {
    left_duty = copysignf(MOTOR_MIN_EFFECTIVE_DUTY, left_duty);
  }
  if (right_duty != 0.0f && fabsf(right_duty) < MOTOR_MIN_EFFECTIVE_DUTY) {
    right_duty = copysignf(MOTOR_MIN_EFFECTIVE_DUTY, right_duty);
  }

  current_left_duty = left_duty;
  current_right_duty = right_duty;
  left_motor.setDuty(left_duty);
  right_motor.setDuty(right_duty);
}

void updateWheelMeasurements(float dt) {
  static int32_t last_left_ticks = 0;
  static int32_t last_right_ticks = 0;

  const int32_t raw_left_ticks = atomicReadTicks(left_encoder_ticks);
  const int32_t raw_right_ticks = atomicReadTicks(right_encoder_ticks);

  left_wheel.ticks = LEFT_ENCODER_INVERTED ? -raw_left_ticks : raw_left_ticks;
  right_wheel.ticks = RIGHT_ENCODER_INVERTED ? -raw_right_ticks : raw_right_ticks;

  const int32_t delta_left_ticks = left_wheel.ticks - last_left_ticks;
  const int32_t delta_right_ticks = right_wheel.ticks - last_right_ticks;

  const float meters_per_tick = (PI_F * WHEEL_DIAMETER_M) / ENCODER_TICKS_PER_WHEEL_REV;
  const float left_delta_m = delta_left_ticks * meters_per_tick;
  const float right_delta_m = delta_right_ticks * meters_per_tick;

  left_wheel.velocity_mps = left_delta_m / dt;
  right_wheel.velocity_mps = right_delta_m / dt;

  const float linear_velocity = 0.5f * (left_wheel.velocity_mps + right_wheel.velocity_mps);
  const float angular_velocity = (right_wheel.velocity_mps - left_wheel.velocity_mps) / TRACK_WIDTH_M;
  const float delta_yaw = angular_velocity * dt;
  const float heading_mid = odom_yaw + 0.5f * delta_yaw;

  odom_x += linear_velocity * cosf(heading_mid) * dt;
  odom_y += linear_velocity * sinf(heading_mid) * dt;
  odom_yaw += delta_yaw;

  last_left_ticks = left_wheel.ticks;
  last_right_ticks = right_wheel.ticks;
}

void fillOdomMessage(const builtin_interfaces__msg__Time &stamp) {
  odom_msg.header.stamp = stamp;
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  setQuaternionFromYaw(odom_yaw, odom_msg.pose.pose.orientation);

  odom_msg.twist.twist.linear.x = 0.5 * (left_wheel.velocity_mps + right_wheel.velocity_mps);
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z =
      (right_wheel.velocity_mps - left_wheel.velocity_mps) / TRACK_WIDTH_M;
}

void fillImuMessage(const builtin_interfaces__msg__Time &stamp) {
  imu_msg.header.stamp = stamp;
  imu_msg.angular_velocity.x = latest_imu.gx;
  imu_msg.angular_velocity.y = latest_imu.gy;
  imu_msg.angular_velocity.z = latest_imu.gz;
  imu_msg.linear_acceleration.x = latest_imu.ax;
  imu_msg.linear_acceleration.y = latest_imu.ay;
  imu_msg.linear_acceleration.z = latest_imu.az;
}

void initializeCovariances() {
  for (size_t i = 0; i < 9; ++i) {
    imu_msg.orientation_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }

  for (size_t i = 0; i < 36; ++i) {
    odom_msg.pose.covariance[i] = 0.0;
    odom_msg.twist.covariance[i] = 0.0;
  }

  imu_msg.orientation_covariance[0] = -1.0;
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[8] = 0.02;
  imu_msg.linear_acceleration_covariance[0] = 0.1;
  imu_msg.linear_acceleration_covariance[4] = 0.1;
  imu_msg.linear_acceleration_covariance[8] = 0.1;

  odom_msg.pose.covariance[0] = 0.02;
  odom_msg.pose.covariance[7] = 0.02;
  odom_msg.pose.covariance[35] = 0.05;
  odom_msg.twist.covariance[0] = 0.05;
  odom_msg.twist.covariance[7] = 0.05;
  odom_msg.twist.covariance[35] = 0.1;
}

bool initializeRosMessages() {
  if (ros_messages_ready) {
    return true;
  }

  if (!nav_msgs__msg__Odometry__init(&odom_msg)) {
    return false;
  }
  if (!sensor_msgs__msg__Imu__init(&imu_msg)) {
    return false;
  }
  if (!std_msgs__msg__Int32__init(&left_ticks_msg)) {
    return false;
  }
  if (!std_msgs__msg__Int32__init(&right_ticks_msg)) {
    return false;
  }
  if (!std_msgs__msg__Float32__init(&left_pwm_msg)) {
    return false;
  }
  if (!std_msgs__msg__Float32__init(&right_pwm_msg)) {
    return false;
  }
  if (!geometry_msgs__msg__Twist__init(&cmd_vel_msg)) {
    return false;
  }
  if (!geometry_msgs__msg__Twist__init(&cmd_vel_echo_msg)) {
    return false;
  }
  if (!geometry_msgs__msg__Twist__init(&target_cmd)) {
    return false;
  }

  if (!rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, ODOM_FRAME)) {
    return false;
  }
  if (!rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, BASE_FRAME)) {
    return false;
  }
  if (!rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, IMU_FRAME)) {
    return false;
  }

  initializeCovariances();
  ros_messages_ready = true;
  return true;
}

void cmdVelCallback(const void *msg_in) {
  const auto *msg = static_cast<const geometry_msgs__msg__Twist *>(msg_in);
  target_cmd = *msg;
  last_cmd_vel_ms = millis();
}

void controlTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == nullptr) {
    return;
  }

  const unsigned long now_ms = millis();
  float dt = static_cast<float>(now_ms - last_control_update_ms) / 1000.0f;
  if (dt <= 0.0f) {
    dt = CONTROL_PERIOD_MS / 1000.0f;
  }
  last_control_update_ms = now_ms;

  updateWheelMeasurements(dt);
  if (imu_ready) {
    imu_sensor.read(latest_imu);
  } else {
    latest_imu.valid = false;
  }

  float linear_command = target_cmd.linear.x;
  float angular_command = target_cmd.angular.z;
  if (now_ms - last_cmd_vel_ms > COMMAND_TIMEOUT_MS) {
    linear_command = 0.0f;
    angular_command = 0.0f;
  }

  const float half_track = TRACK_WIDTH_M * 0.5f;
  const float left_target = linear_command - angular_command * half_track;
  const float right_target = linear_command + angular_command * half_track;
  applyMotorCommand(left_target, right_target);

  const builtin_interfaces__msg__Time stamp = nowRosTime();
  fillOdomMessage(stamp);

  left_ticks_msg.data = left_wheel.ticks;
  right_ticks_msg.data = right_wheel.ticks;
  left_pwm_msg.data = current_left_duty;
  right_pwm_msg.data = current_right_duty;
  cmd_vel_echo_msg = target_cmd;

  (void) rcl_publish(&odom_publisher, &odom_msg, nullptr);
  (void) rcl_publish(&left_ticks_publisher, &left_ticks_msg, nullptr);
  (void) rcl_publish(&right_ticks_publisher, &right_ticks_msg, nullptr);
  (void) rcl_publish(&cmd_vel_echo_publisher, &cmd_vel_echo_msg, nullptr);
  (void) rcl_publish(&left_pwm_publisher, &left_pwm_msg, nullptr);
  (void) rcl_publish(&right_pwm_publisher, &right_pwm_msg, nullptr);

  if (latest_imu.valid) {
    fillImuMessage(stamp);
    (void) rcl_publish(&imu_publisher, &imu_msg, nullptr);
  }
}

bool createRosEntities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) {
    return false;
  }

  if (rclc_node_init_default(&node, NODE_NAME, "", &support) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &odom_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ODOM_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &imu_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), IMU_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &left_ticks_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), LEFT_TICKS_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &right_ticks_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), RIGHT_TICKS_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &cmd_vel_echo_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), CMD_VEL_ECHO_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &left_pwm_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), LEFT_PWM_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &right_pwm_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), RIGHT_PWM_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_subscription_init_default(
          &cmd_vel_subscription, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), CMD_VEL_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(CONTROL_PERIOD_MS),
                              controlTimerCallback) != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_subscription(&executor, &cmd_vel_subscription, &cmd_vel_msg,
                                     &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_timer(&executor, &control_timer) != RCL_RET_OK) {
    return false;
  }

  time_synced = rmw_uros_sync_session(1000) == RMW_RET_OK;
  ros_entities_created = true;
  last_control_update_ms = millis();
  return true;
}

void destroyRosEntities() {
  if (!ros_entities_created) {
    return;
  }

  rcl_publisher_fini(&odom_publisher, &node);
  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&left_ticks_publisher, &node);
  rcl_publisher_fini(&right_ticks_publisher, &node);
  rcl_publisher_fini(&cmd_vel_echo_publisher, &node);
  rcl_publisher_fini(&left_pwm_publisher, &node);
  rcl_publisher_fini(&right_pwm_publisher, &node);
  rcl_subscription_fini(&cmd_vel_subscription, &node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  ros_entities_created = false;
  time_synced = false;
}

void setupEncoders() {
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M1_ENC_B, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_B, INPUT_PULLUP);

  left_encoder_state = readEncoderState(M1_ENC_A, M1_ENC_B);
  right_encoder_state = readEncoderState(M2_ENC_A, M2_ENC_B);

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), onLeftEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B), onLeftEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), onRightEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_B), onRightEncoderChange, CHANGE);
}

void setupTransport() {
  ros_serial.begin(UART_BAUDRATE, SERIAL_8N1, UART0_RX, UART0_TX);
  set_microros_serial_transports(ros_serial);
}

void updateAgentStateMachine() {
  const unsigned long now_ms = millis();

  switch (agent_state) {
    case AgentState::WaitingAgent:
      stopMotors();
      if (now_ms - last_agent_check_ms > 500) {
        last_agent_check_ms = now_ms;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          agent_state = AgentState::AgentAvailable;
        }
      }
      break;

    case AgentState::AgentAvailable:
      if (createRosEntities()) {
        agent_state = AgentState::AgentConnected;
      } else {
        destroyRosEntities();
        agent_state = AgentState::WaitingAgent;
      }
      break;

    case AgentState::AgentConnected:
      if (now_ms - last_agent_check_ms > 1000) {
        last_agent_check_ms = now_ms;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          agent_state = AgentState::AgentDisconnected;
          break;
        }
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      break;

    case AgentState::AgentDisconnected:
      destroyRosEntities();
      stopMotors();
      agent_state = AgentState::WaitingAgent;
      break;
  }
}

}  // namespace

void setup() {
  left_motor.begin();
  right_motor.begin();
  setupEncoders();
  setupTransport();

  (void) initializeRosMessages();
  imu_ready = imu_sensor.begin();
  if (imu_ready) {
    imu_ready = imu_sensor.calibrateGyroBias();
  }

  last_cmd_vel_ms = millis();
  last_control_update_ms = millis();
}

void loop() {
  updateAgentStateMachine();
}

