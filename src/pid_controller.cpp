#include "pid_controller.h"

#include <Arduino.h>

namespace robot {

PidController::PidController(float kp, float ki, float kd, float integral_limit, float output_limit)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      integral_limit_(integral_limit),
      output_limit_(output_limit) {}

float PidController::update(float target, float measurement, float dt) {
  const float error = target - measurement;
  integral_ += error * dt;
  integral_ = constrain(integral_, -integral_limit_, integral_limit_);

  float derivative = 0.0f;
  if (initialized_ && dt > 0.0f) {
    derivative = (error - previous_error_) / dt;
  }

  previous_error_ = error;
  initialized_ = true;

  const float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  return constrain(output, -output_limit_, output_limit_);
}

void PidController::reset() {
  integral_ = 0.0f;
  previous_error_ = 0.0f;
  initialized_ = false;
}

}  // namespace robot
