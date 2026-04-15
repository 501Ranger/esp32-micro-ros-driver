#include "qmi8658_sensor.h"

#include "robot_config.h"

namespace robot {

using namespace robot_config;

namespace {

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

}  // namespace

bool QMI8658Sensor::begin() {
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

bool QMI8658Sensor::calibrateGyroBias(uint16_t samples) {
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

bool QMI8658Sensor::read(ImuSample &sample) {
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

bool QMI8658Sensor::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMI8658_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool QMI8658Sensor::readRegister(uint8_t reg, uint8_t &value) {
  return readRegisters(reg, &value, 1);
}

bool QMI8658Sensor::readRegisters(uint8_t reg, uint8_t *buffer, size_t length) {
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

}  // namespace robot
