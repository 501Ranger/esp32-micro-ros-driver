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

}  // namespace robot
