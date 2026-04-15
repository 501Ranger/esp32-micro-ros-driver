#pragma once

#include <Arduino.h>

namespace robot {

class MotorDriver {
 public:
  MotorDriver(uint8_t forward_pin, uint8_t reverse_pin, uint8_t forward_channel,
              uint8_t reverse_channel, bool inverted);

  void begin() const;
  void setDuty(float duty) const;
  void stop() const;

 private:
  uint8_t forward_pin_;
  uint8_t reverse_pin_;
  uint8_t forward_channel_;
  uint8_t reverse_channel_;
  bool inverted_;
};

}  // namespace robot
