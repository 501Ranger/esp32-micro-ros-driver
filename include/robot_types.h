#pragma once

#include <stdint.h>

namespace robot {

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
  float pitch = 0.0f;
  float roll = 0.0f;
  float yaw = 0.0f;
  float temperature_c = 0.0f;
  bool valid = false;
};

struct SystemStatus {
  float battery_voltage = 12.0f;
  int battery_percentage = 100;
  AgentState agent_state = AgentState::WaitingAgent;
  int wifi_rssi = 0;
  float left_speed = 0.0f;
  float left_target = 0.0f;
  float right_speed = 0.0f;
  float right_target = 0.0f;
  float yaw = 0.0f;
  uint32_t uptime_sec = 0;
  bool vofa_debug = false;
};

}  // namespace robot
