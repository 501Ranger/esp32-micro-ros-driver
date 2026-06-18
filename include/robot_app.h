#pragma once

#include <Arduino.h>

#include <builtin_interfaces/msg/time.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/battery_state.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "encoder_reader.h"
#include "motor_driver.h"
#include "pid_controller.h"
#include "qmi8658_sensor.h"
#include "robot_config.h"
#include "robot_types.h"
#include "web_manager.h"
#include "wifi_config_manager.h"

namespace robot {

class RobotApp {
 public:
  RobotApp();

  void setup();
  void loop();

 private:
  static void cmdVelCallback(const void *msg_in);
  static void controlTimerCallback(rcl_timer_t *timer, int64_t last_call_time);

  void cmdVelCallbackImpl(const geometry_msgs__msg__Twist &msg);
  void controlTimerCallbackImpl();
  void updateAgentStateMachine();
  void stopMotors();
  void setupTransport();
  bool connectConfiguredWifi();
  void startConfigPortal();
  void startOta();
  void setupSensors();
  void setupCAN();
  void updateBattery();
  void applyMotorCommand(float left_velocity_mps, float right_velocity_mps);
  void handleSerialCommands();
  void updateWheelMeasurements(float dt);
  void fillOdomMessage(const builtin_interfaces__msg__Time &stamp);
  void initializeCovariances();
  bool initializeRosMessages();
  bool createRosEntities();
  void destroyRosEntities();
  builtin_interfaces__msg__Time nowRosTime() const;
  static void setQuaternionFromYaw(double yaw, geometry_msgs__msg__Quaternion &quat);

  static RobotApp *instance_;

  MotorDriver left_motor_;
  MotorDriver right_motor_;
  EncoderReader left_encoder_;
  EncoderReader right_encoder_;
  QMI8658Sensor imu_sensor_;
  PidController left_pid_;
  PidController right_pid_;
  WebManager web_manager_;
  WifiConfigManager wifi_config_manager_;
  HardwareSerial ros_serial_;
  NetworkConfig network_config_;

  rcl_allocator_t allocator_;
  rclc_support_t support_;
  rcl_node_t node_;
  rcl_timer_t control_timer_;
  rclc_executor_t executor_;
  rcl_publisher_t odom_publisher_;
  rcl_publisher_t battery_publisher_;
  rcl_subscription_t cmd_vel_subscription_;

  geometry_msgs__msg__Twist cmd_vel_msg_;
  nav_msgs__msg__Odometry odom_msg_;
  sensor_msgs__msg__BatteryState battery_msg_;
  geometry_msgs__msg__Twist target_cmd_;

  AgentState agent_state_ = AgentState::WaitingAgent;
  bool ros_entities_created_ = false;
  bool ros_messages_ready_ = false;
  bool time_synced_ = false;
  bool imu_ready_ = false;
  bool wifi_ready_ = false;
  bool transport_ready_ = false;
  bool ota_started_ = false;

  WheelMeasurement left_wheel_;
  WheelMeasurement right_wheel_;
  ImuSample latest_imu_;

  int32_t last_left_ticks_ = 0;
  int32_t last_right_ticks_ = 0;

  unsigned long last_agent_check_ms_ = 0;
  unsigned long last_cmd_vel_ms_ = 0;
  unsigned long last_control_update_ms_ = 0;

  float odom_x_ = 0.0f;
  float odom_y_ = 0.0f;
  float odom_yaw_ = 0.0f;
  float current_left_duty_ = 0.0f;
  float current_right_duty_ = 0.0f;
  float target_left_velocity_mps_ = 0.0f;
  float target_right_velocity_mps_ = 0.0f;

  // Battery monitoring
  float battery_voltage_ = 12.0f;
  int battery_percentage_ = 100;
  unsigned long last_battery_update_ms_ = 0;

  // Dynamic tuning parameters
  float kp_ = config::MOTOR_PID_KP;
  float ki_ = config::MOTOR_PID_KI;
  float kd_ = config::MOTOR_PID_KD;
  float kf_ = config::MOTOR_FEEDFORWARD_GAIN;
  float output_limit_ = config::MOTOR_PID_OUTPUT_LIMIT;
  float motor_min_duty_ = config::MOTOR_MIN_EFFECTIVE_DUTY;

  int ros_init_stage_ = 0;
};

}  // namespace robot
