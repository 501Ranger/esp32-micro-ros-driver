#pragma once

#include <Arduino.h>

namespace robot_config {

constexpr uint8_t M1_IN1 = 14;
constexpr uint8_t M1_IN2 = 21;
constexpr uint8_t M1_ENC_A = 11;
constexpr uint8_t M1_ENC_B = 10;

constexpr uint8_t M2_IN1 = 48;
constexpr uint8_t M2_IN2 = 47;
constexpr uint8_t M2_ENC_A = 12;
constexpr uint8_t M2_ENC_B = 13;

constexpr uint8_t I2C_SDA = 35;
constexpr uint8_t I2C_SCL = 36;

constexpr uint8_t UART0_TX = 43;
constexpr uint8_t UART0_RX = 44;
constexpr uint32_t UART_BAUDRATE = 115200;

constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = false;
constexpr bool LEFT_ENCODER_INVERTED = false;
constexpr bool RIGHT_ENCODER_INVERTED = false;

constexpr uint8_t PWM_BITS = 8;
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t LEFT_FORWARD_CHANNEL = 0;
constexpr uint8_t LEFT_REVERSE_CHANNEL = 1;
constexpr uint8_t RIGHT_FORWARD_CHANNEL = 2;
constexpr uint8_t RIGHT_REVERSE_CHANNEL = 3;
constexpr int16_t PWM_MAX = (1 << PWM_BITS) - 1;
constexpr float MOTOR_MIN_EFFECTIVE_DUTY = 0.60f;

constexpr float WHEEL_DIAMETER_M = 0.048f;
constexpr float TRACK_WIDTH_M = 0.130f;
constexpr float ENCODER_TICKS_PER_WHEEL_REV = 1040.0f;
constexpr float MAX_WHEEL_LINEAR_SPEED_MPS = 0.30f;
constexpr float VELOCITY_DEADBAND_MPS = 0.01f;

constexpr float MOTOR_PID_KP = 1.00f;
constexpr float MOTOR_PID_KI = 0.00f;
constexpr float MOTOR_PID_KD = 0.00f;
constexpr float MOTOR_PID_INTEGRAL_LIMIT = 0.30f;
constexpr float MOTOR_PID_OUTPUT_LIMIT = 0.60f;
constexpr float MOTOR_FEEDFORWARD_GAIN = 1.00f;

constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint32_t COMMAND_TIMEOUT_MS = 100;
constexpr uint32_t IMU_I2C_FREQUENCY = 400000;

constexpr char NODE_NAME[] = "esp32s3_base";
constexpr char CMD_VEL_TOPIC[] = "cmd_vel";
constexpr char ODOM_TOPIC[] = "odom";
constexpr char IMU_TOPIC[] = "imu/data_raw";
constexpr char LEFT_TICKS_TOPIC[] = "left_wheel/ticks";
constexpr char RIGHT_TICKS_TOPIC[] = "right_wheel/ticks";
constexpr char CMD_VEL_ECHO_TOPIC[] = "debug/cmd_vel_echo";
constexpr char LEFT_PWM_TOPIC[] = "debug/left_pwm";
constexpr char RIGHT_PWM_TOPIC[] = "debug/right_pwm";
constexpr char ODOM_FRAME[] = "odom";
constexpr char BASE_FRAME[] = "base_link";
constexpr char IMU_FRAME[] = "imu_link";

}  // namespace robot_config
