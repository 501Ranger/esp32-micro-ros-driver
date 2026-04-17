#include "robot_app.h"

#include <cmath>
#include <ArduinoOTA.h>

#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

#include "robot_config.h"

namespace robot {

using namespace robot_config;

namespace {

constexpr float PI_F = 3.14159265358979323846f;

}  // namespace

RobotApp *RobotApp::instance_ = nullptr;

RobotApp::RobotApp()
    : left_motor_(M1_IN1, M1_IN2, LEFT_FORWARD_CHANNEL, LEFT_REVERSE_CHANNEL, LEFT_MOTOR_INVERTED),
      right_motor_(M2_IN1, M2_IN2, RIGHT_FORWARD_CHANNEL, RIGHT_REVERSE_CHANNEL, RIGHT_MOTOR_INVERTED),
      left_pid_(MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_INTEGRAL_LIMIT, MOTOR_PID_OUTPUT_LIMIT),
      right_pid_(MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_INTEGRAL_LIMIT, MOTOR_PID_OUTPUT_LIMIT),
      ros_serial_(0) {
  instance_ = this;
}

void RobotApp::setup() {
  left_motor_.begin();
  right_motor_.begin();
  setupSensors();
  setupTransport();

  (void) initializeRosMessages();
  imu_ready_ = imu_sensor_.begin();
  if (imu_ready_) {
    imu_ready_ = imu_sensor_.calibrateGyroBias();
  }

  Serial1.begin(UART_BAUDRATE, SERIAL_8N1, 15, 16);

  last_cmd_vel_ms_ = millis();
  last_control_update_ms_ = millis();

#ifdef USE_WIFI_TRANSPORT
  ArduinoOTA.begin();
#endif
}

void RobotApp::loop() {
#ifdef USE_WIFI_TRANSPORT
  ArduinoOTA.handle();
#endif
  handleSerialCommands();
  updateAgentStateMachine();
}

void RobotApp::handleSerialCommands() {
#ifdef USE_WIFI_TRANSPORT
  static String input_buffer = "";
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      if (input_buffer.length() > 0) {
        int separator_idx = input_buffer.indexOf(':');
        if (separator_idx != -1) {
          String cmd = input_buffer.substring(0, separator_idx);
          float val = input_buffer.substring(separator_idx + 1).toFloat();

          bool updated = true;
          if (cmd == "p") kp_ = val;
          else if (cmd == "i") ki_ = val;
          else if (cmd == "d") kd_ = val;
          else if (cmd == "f") kf_ = val;
          else if (cmd == "o") output_limit_ = val;
          else updated = false;

          if (updated) {
            left_pid_.setGains(kp_, ki_, kd_);
            right_pid_.setGains(kp_, ki_, kd_);
            left_pid_.setOutputLimit(output_limit_);
            right_pid_.setOutputLimit(output_limit_);
            Serial1.printf("Updated -> P:%.2f I:%.2f D:%.2f F:%.2f O:%.2f\n", kp_, ki_, kd_, kf_, output_limit_);
          }
        }
        input_buffer = "";
      }
    } else {
      input_buffer += c;
    }
  }
#endif
}

void RobotApp::cmdVelCallback(const void *msg_in) {
  if (instance_ == nullptr || msg_in == nullptr) {
    return;
  }

  instance_->cmdVelCallbackImpl(*static_cast<const geometry_msgs__msg__Twist *>(msg_in));
}

void RobotApp::controlTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == nullptr || instance_ == nullptr) {
    return;
  }

  instance_->controlTimerCallbackImpl();
}

void RobotApp::cmdVelCallbackImpl(const geometry_msgs__msg__Twist &msg) {
  target_cmd_ = msg;
  last_cmd_vel_ms_ = millis();
}

void RobotApp::controlTimerCallbackImpl() {
  const unsigned long now_ms = millis();
  float dt = static_cast<float>(now_ms - last_control_update_ms_) / 1000.0f;
  if (dt <= 0.0f) {
    dt = CONTROL_PERIOD_MS / 1000.0f;
  }
  last_control_update_ms_ = now_ms;

  if (imu_ready_) {
    imu_sensor_.read(latest_imu_);
  } else {
    latest_imu_.valid = false;
  }
  updateWheelMeasurements(dt);

  float linear_command = target_cmd_.linear.x;
  float angular_command = target_cmd_.angular.z;
  if (now_ms - last_cmd_vel_ms_ > COMMAND_TIMEOUT_MS) {
    linear_command = 0.0f;
    angular_command = 0.0f;
  }

  const float half_track = TRACK_WIDTH_M * 0.5f;
  const float left_target = linear_command - angular_command * half_track;
  const float right_target = linear_command + angular_command * half_track;
  target_left_velocity_mps_ = left_target;
  target_right_velocity_mps_ = right_target;
  applyMotorCommand(left_target, right_target);

  // VOFA+ JustFloat protocol
  float debug_data[] = {
      target_left_velocity_mps_,
      left_wheel_.velocity_mps,
      target_right_velocity_mps_,
      right_wheel_.velocity_mps
  };
  Serial1.write(reinterpret_cast<uint8_t*>(debug_data), sizeof(debug_data));
  const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
  Serial1.write(tail, 4);

  const builtin_interfaces__msg__Time stamp = nowRosTime();
  fillOdomMessage(stamp);

  left_ticks_msg_.data = left_wheel_.ticks;
  right_ticks_msg_.data = right_wheel_.ticks;
  left_pwm_msg_.data = current_left_duty_;
  right_pwm_msg_.data = current_right_duty_;
  cmd_vel_echo_msg_ = target_cmd_;

  (void) rcl_publish(&odom_publisher_, &odom_msg_, nullptr);
  (void) rcl_publish(&left_ticks_publisher_, &left_ticks_msg_, nullptr);
  (void) rcl_publish(&right_ticks_publisher_, &right_ticks_msg_, nullptr);
  (void) rcl_publish(&cmd_vel_echo_publisher_, &cmd_vel_echo_msg_, nullptr);
  (void) rcl_publish(&left_pwm_publisher_, &left_pwm_msg_, nullptr);
  (void) rcl_publish(&right_pwm_publisher_, &right_pwm_msg_, nullptr);

  if (latest_imu_.valid) {
    fillImuMessage(stamp);
    (void) rcl_publish(&imu_publisher_, &imu_msg_, nullptr);
  }
}

void RobotApp::updateAgentStateMachine() {
  const unsigned long now_ms = millis();

  switch (agent_state_) {
    case AgentState::WaitingAgent:
      stopMotors();
      if (now_ms - last_agent_check_ms_ > 500) {
        last_agent_check_ms_ = now_ms;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          agent_state_ = AgentState::AgentAvailable;
        }
      }
      break;

    case AgentState::AgentAvailable:
      if (createRosEntities()) {
        agent_state_ = AgentState::AgentConnected;
      } else {
        destroyRosEntities();
        agent_state_ = AgentState::WaitingAgent;
      }
      break;

    case AgentState::AgentConnected:
      if (now_ms - last_agent_check_ms_ > 1000) {
        last_agent_check_ms_ = now_ms;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          agent_state_ = AgentState::AgentDisconnected;
          break;
        }
      }
      rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(5));
      break;

    case AgentState::AgentDisconnected:
      destroyRosEntities();
      stopMotors();
      agent_state_ = AgentState::WaitingAgent;
      break;
  }
}

void RobotApp::stopMotors() {
  left_pid_.reset();
  right_pid_.reset();
  current_left_duty_ = 0.0f;
  current_right_duty_ = 0.0f;
  target_left_velocity_mps_ = 0.0f;
  target_right_velocity_mps_ = 0.0f;
  left_motor_.stop();
  right_motor_.stop();
}

void RobotApp::setupTransport() {
#ifdef USE_WIFI_TRANSPORT
  Serial.begin(UART_BAUDRATE, SERIAL_8N1, UART0_RX, UART0_TX);
  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  set_microros_wifi_transports(const_cast<char*>(WIFI_SSID), const_cast<char*>(WIFI_PASSWORD), agent_ip, AGENT_PORT);
#else
  ros_serial_.begin(UART_BAUDRATE, SERIAL_8N1, UART0_RX, UART0_TX);
  set_microros_serial_transports(ros_serial_);
#endif
}

void RobotApp::setupSensors() {
  left_encoder_.begin(M1_ENC_A, M1_ENC_B, 0);
  right_encoder_.begin(M2_ENC_A, M2_ENC_B, 1);
}

void RobotApp::applyMotorCommand(float left_velocity_mps, float right_velocity_mps) {
  float left_duty = 0.0f;
  float right_duty = 0.0f;

  if (fabsf(left_velocity_mps) <= VELOCITY_DEADBAND_MPS) {
    left_pid_.reset();
  } else {
    const float left_feedforward =
        kf_ * (left_velocity_mps / MAX_WHEEL_LINEAR_SPEED_MPS);
    const float left_pid_correction = left_pid_.update(left_velocity_mps, left_wheel_.velocity_mps, CONTROL_PERIOD_MS / 1000.0f);
    left_duty = left_feedforward + left_pid_correction;
  }

  if (fabsf(right_velocity_mps) <= VELOCITY_DEADBAND_MPS) {
    right_pid_.reset();
  } else {
    const float right_feedforward =
        kf_ * (right_velocity_mps / MAX_WHEEL_LINEAR_SPEED_MPS);
    const float right_pid_correction =
        right_pid_.update(right_velocity_mps, right_wheel_.velocity_mps, CONTROL_PERIOD_MS / 1000.0f);
    right_duty = right_feedforward + right_pid_correction;
  }

  left_duty = constrain(left_duty, -1.0f, 1.0f);
  right_duty = constrain(right_duty, -1.0f, 1.0f);

  if (left_duty != 0.0f && fabsf(left_duty) < MOTOR_MIN_EFFECTIVE_DUTY) {
    left_duty = copysignf(MOTOR_MIN_EFFECTIVE_DUTY, left_duty);
  }
  if (right_duty != 0.0f && fabsf(right_duty) < MOTOR_MIN_EFFECTIVE_DUTY) {
    right_duty = copysignf(MOTOR_MIN_EFFECTIVE_DUTY, right_duty);
  }

  current_left_duty_ = left_duty;
  current_right_duty_ = right_duty;
  left_motor_.setDuty(left_duty);
  right_motor_.setDuty(right_duty);
}

void RobotApp::updateWheelMeasurements(float dt) {
  static int32_t last_left_ticks = 0;
  static int32_t last_right_ticks = 0;

  const int32_t raw_left_ticks = left_encoder_.readTicks();
  const int32_t raw_right_ticks = right_encoder_.readTicks();

  left_wheel_.ticks = LEFT_ENCODER_INVERTED ? -raw_left_ticks : raw_left_ticks;
  right_wheel_.ticks = RIGHT_ENCODER_INVERTED ? -raw_right_ticks : raw_right_ticks;

  const int32_t delta_left_ticks = left_wheel_.ticks - last_left_ticks;
  const int32_t delta_right_ticks = right_wheel_.ticks - last_right_ticks;

  const float meters_per_tick = (PI_F * WHEEL_DIAMETER_M) / ENCODER_TICKS_PER_WHEEL_REV;
  const float left_delta_m = delta_left_ticks * meters_per_tick;
  const float right_delta_m = delta_right_ticks * meters_per_tick;

  left_wheel_.velocity_mps = (left_delta_m / dt);
  right_wheel_.velocity_mps = (right_delta_m / dt);

  const float linear_velocity = 0.5f * (left_wheel_.velocity_mps + right_wheel_.velocity_mps);
  
  float angular_velocity;
  if (latest_imu_.valid) {
    angular_velocity = latest_imu_.gz;
  } else {
    angular_velocity = (right_wheel_.velocity_mps - left_wheel_.velocity_mps) / TRACK_WIDTH_M;
  }
  
  const float delta_yaw = angular_velocity * dt;
  const float heading_mid = odom_yaw_ + 0.5f * delta_yaw;

  odom_x_ += linear_velocity * cosf(heading_mid) * dt;
  odom_y_ += linear_velocity * sinf(heading_mid) * dt;
  odom_yaw_ += delta_yaw;

  last_left_ticks = left_wheel_.ticks;
  last_right_ticks = right_wheel_.ticks;
}

void RobotApp::fillOdomMessage(const builtin_interfaces__msg__Time &stamp) {
  odom_msg_.header.stamp = stamp;
  odom_msg_.pose.pose.position.x = odom_x_;
  odom_msg_.pose.pose.position.y = odom_y_;
  odom_msg_.pose.pose.position.z = 0.0;
  setQuaternionFromYaw(odom_yaw_, odom_msg_.pose.pose.orientation);

  odom_msg_.twist.twist.linear.x = 0.5 * (left_wheel_.velocity_mps + right_wheel_.velocity_mps);
  odom_msg_.twist.twist.linear.y = 0.0;
  odom_msg_.twist.twist.linear.z = 0.0;
  odom_msg_.twist.twist.angular.x = 0.0;
  odom_msg_.twist.twist.angular.y = 0.0;
  odom_msg_.twist.twist.angular.z =
      (right_wheel_.velocity_mps - left_wheel_.velocity_mps) / TRACK_WIDTH_M;
}

void RobotApp::fillImuMessage(const builtin_interfaces__msg__Time &stamp) {
  imu_msg_.header.stamp = stamp;
  imu_msg_.angular_velocity.x = latest_imu_.gx;
  imu_msg_.angular_velocity.y = latest_imu_.gy;
  imu_msg_.angular_velocity.z = latest_imu_.gz;
  imu_msg_.linear_acceleration.x = latest_imu_.ax;
  imu_msg_.linear_acceleration.y = latest_imu_.ay;
  imu_msg_.linear_acceleration.z = latest_imu_.az;
}

void RobotApp::initializeCovariances() {
  for (size_t i = 0; i < 9; ++i) {
    imu_msg_.orientation_covariance[i] = 0.0;
    imu_msg_.angular_velocity_covariance[i] = 0.0;
    imu_msg_.linear_acceleration_covariance[i] = 0.0;
  }

  for (size_t i = 0; i < 36; ++i) {
    odom_msg_.pose.covariance[i] = 0.0;
    odom_msg_.twist.covariance[i] = 0.0;
  }

  imu_msg_.orientation_covariance[0] = -1.0;
  imu_msg_.angular_velocity_covariance[0] = 0.02;
  imu_msg_.angular_velocity_covariance[4] = 0.02;
  imu_msg_.angular_velocity_covariance[8] = 0.02;
  imu_msg_.linear_acceleration_covariance[0] = 0.1;
  imu_msg_.linear_acceleration_covariance[4] = 0.1;
  imu_msg_.linear_acceleration_covariance[8] = 0.1;

  odom_msg_.pose.covariance[0] = 0.02;
  odom_msg_.pose.covariance[7] = 0.02;
  odom_msg_.pose.covariance[35] = 0.05;
  odom_msg_.twist.covariance[0] = 0.05;
  odom_msg_.twist.covariance[7] = 0.05;
  odom_msg_.twist.covariance[35] = 0.1;
}

bool RobotApp::initializeRosMessages() {
  if (ros_messages_ready_) {
    return true;
  }

  if (!nav_msgs__msg__Odometry__init(&odom_msg_)) {
    return false;
  }
  if (!sensor_msgs__msg__Imu__init(&imu_msg_)) {
    return false;
  }
  if (!std_msgs__msg__Int32__init(&left_ticks_msg_)) {
    return false;
  }
  if (!std_msgs__msg__Int32__init(&right_ticks_msg_)) {
    return false;
  }
  if (!std_msgs__msg__Float32__init(&left_pwm_msg_)) {
    return false;
  }
  if (!std_msgs__msg__Float32__init(&right_pwm_msg_)) {
    return false;
  }
  if (!geometry_msgs__msg__Twist__init(&cmd_vel_msg_)) {
    return false;
  }
  if (!geometry_msgs__msg__Twist__init(&cmd_vel_echo_msg_)) {
    return false;
  }
  if (!geometry_msgs__msg__Twist__init(&target_cmd_)) {
    return false;
  }

  if (!rosidl_runtime_c__String__assign(&odom_msg_.header.frame_id, ODOM_FRAME)) {
    return false;
  }
  if (!rosidl_runtime_c__String__assign(&odom_msg_.child_frame_id, BASE_FRAME)) {
    return false;
  }
  if (!rosidl_runtime_c__String__assign(&imu_msg_.header.frame_id, IMU_FRAME)) {
    return false;
  }

  initializeCovariances();
  ros_messages_ready_ = true;
  return true;
}

bool RobotApp::createRosEntities() {
  allocator_ = rcl_get_default_allocator();

  if (rclc_support_init(&support_, 0, nullptr, &allocator_) != RCL_RET_OK) {
    return false;
  }

  if (rclc_node_init_default(&node_, NODE_NAME, "", &support_) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &odom_publisher_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ODOM_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &imu_publisher_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), IMU_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &left_ticks_publisher_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), LEFT_TICKS_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &right_ticks_publisher_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), RIGHT_TICKS_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &cmd_vel_echo_publisher_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), CMD_VEL_ECHO_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &left_pwm_publisher_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), LEFT_PWM_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &right_pwm_publisher_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), RIGHT_PWM_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_subscription_init_default(
          &cmd_vel_subscription_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), CMD_VEL_TOPIC) != RCL_RET_OK) {
    return false;
  }

  if (rclc_timer_init_default(&control_timer_, &support_, RCL_MS_TO_NS(CONTROL_PERIOD_MS),
                              controlTimerCallback) != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_init(&executor_, &support_.context, 2, &allocator_) != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_subscription(&executor_, &cmd_vel_subscription_, &cmd_vel_msg_,
                                     &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_timer(&executor_, &control_timer_) != RCL_RET_OK) {
    return false;
  }

  time_synced_ = rmw_uros_sync_session(1000) == RMW_RET_OK;
  ros_entities_created_ = true;
  last_control_update_ms_ = millis();
  return true;
}

void RobotApp::destroyRosEntities() {
  if (!ros_entities_created_) {
    return;
  }

  rcl_publisher_fini(&odom_publisher_, &node_);
  rcl_publisher_fini(&imu_publisher_, &node_);
  rcl_publisher_fini(&left_ticks_publisher_, &node_);
  rcl_publisher_fini(&right_ticks_publisher_, &node_);
  rcl_publisher_fini(&cmd_vel_echo_publisher_, &node_);
  rcl_publisher_fini(&left_pwm_publisher_, &node_);
  rcl_publisher_fini(&right_pwm_publisher_, &node_);
  rcl_subscription_fini(&cmd_vel_subscription_, &node_);
  rcl_timer_fini(&control_timer_);
  rclc_executor_fini(&executor_);
  rcl_node_fini(&node_);
  rclc_support_fini(&support_);

  ros_entities_created_ = false;
  time_synced_ = false;
}

builtin_interfaces__msg__Time RobotApp::nowRosTime() const {
  builtin_interfaces__msg__Time stamp;
  stamp.sec = 0;
  stamp.nanosec = 0;

  if (time_synced_) {
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

void RobotApp::setQuaternionFromYaw(double yaw, geometry_msgs__msg__Quaternion &quat) {
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = sin(yaw * 0.5);
  quat.w = cos(yaw * 0.5);
}

}  // namespace robot
