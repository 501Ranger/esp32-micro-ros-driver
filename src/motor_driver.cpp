#include "motor_driver.h"

#include <cmath>

#include "robot_config.h"

namespace robot {

using namespace robot_config;

MotorDriver::MotorDriver(uint8_t forward_pin, uint8_t reverse_pin, uint8_t forward_channel,
                         uint8_t reverse_channel, bool inverted)
    : forward_pin_(forward_pin),
      reverse_pin_(reverse_pin),
      forward_channel_(forward_channel),
      reverse_channel_(reverse_channel),
      inverted_(inverted) {}

void MotorDriver::begin() const {
  pinMode(forward_pin_, OUTPUT);
  pinMode(reverse_pin_, OUTPUT);
  ledcSetup(forward_channel_, PWM_FREQUENCY, PWM_BITS);
  ledcSetup(reverse_channel_, PWM_FREQUENCY, PWM_BITS);
  ledcAttachPin(forward_pin_, forward_channel_);
  ledcAttachPin(reverse_pin_, reverse_channel_);
  stop();
}

void MotorDriver::setDuty(float duty) const {
  duty = constrain(duty, -1.0f, 1.0f);
  if (inverted_) {
    duty = -duty;
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

void MotorDriver::stop() const {
  ledcWrite(forward_channel_, 0);
  ledcWrite(reverse_channel_, 0);
  digitalWrite(forward_pin_, LOW);
  digitalWrite(reverse_pin_, LOW);
}

}  // namespace robot
