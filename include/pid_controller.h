#pragma once

namespace robot {

class PidController {
 public:
  PidController(float kp, float ki, float kd, float integral_limit, float output_limit);

  float update(float target, float measurement, float dt);
  void reset();

  void setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void setIntegralLimit(float limit) { integral_limit_ = limit; }
  void setOutputLimit(float limit) { output_limit_ = limit; }

 private:
  float kp_;
  float ki_;
  float kd_;
  float integral_limit_;
  float output_limit_;
  float integral_ = 0.0f;
  float previous_error_ = 0.0f;
  bool initialized_ = false;
};

}  // namespace robot
