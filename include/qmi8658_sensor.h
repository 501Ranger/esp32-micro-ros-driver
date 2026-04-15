#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "robot_types.h"

namespace robot {

class QMI8658Sensor {
 public:
  bool begin();
  bool calibrateGyroBias(uint16_t samples = 400);
  bool read(ImuSample &sample);

 private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegister(uint8_t reg, uint8_t &value);
  bool readRegisters(uint8_t reg, uint8_t *buffer, size_t length);

  float gyro_bias_x_ = 0.0f;
  float gyro_bias_y_ = 0.0f;
  float gyro_bias_z_ = 0.0f;
};

}  // namespace robot
