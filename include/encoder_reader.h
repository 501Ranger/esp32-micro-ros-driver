#pragma once

#include <Arduino.h>

namespace robot {

class EncoderReader {
 public:
  EncoderReader() = default;

  void begin(uint8_t pin_a, uint8_t pin_b, uint8_t index);
  int32_t readTicks() const;

 private:
  static void IRAM_ATTR handleInterrupt0();
  static void IRAM_ATTR handleInterrupt1();
  void IRAM_ATTR handleInterrupt();
  static int8_t quadratureDelta(uint8_t previous_state, uint8_t current_state);
  uint8_t readState() const;

  uint8_t pin_a_ = 0;
  uint8_t pin_b_ = 0;
  volatile int32_t ticks_ = 0;
  volatile uint8_t state_ = 0;

  static EncoderReader *instances_[2];
};

}  // namespace robot
