#include "encoder_reader.h"

namespace robot {

EncoderReader *EncoderReader::instances_[2] = {nullptr, nullptr};

void EncoderReader::begin(uint8_t pin_a, uint8_t pin_b, uint8_t index) {
  pin_a_ = pin_a;
  pin_b_ = pin_b;

  pinMode(pin_a_, INPUT_PULLUP);
  pinMode(pin_b_, INPUT_PULLUP);
  state_ = readState();

  if (index < 2) {
    instances_[index] = this;
  }

  const auto handler = (index == 0) ? handleInterrupt0 : handleInterrupt1;
  attachInterrupt(digitalPinToInterrupt(pin_a_), handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_b_), handler, CHANGE);
}

int32_t EncoderReader::readTicks() const {
  noInterrupts();
  const int32_t value = ticks_;
  interrupts();
  return value;
}

void IRAM_ATTR EncoderReader::handleInterrupt0() {
  if (instances_[0] != nullptr) {
    instances_[0]->handleInterrupt();
  }
}

void IRAM_ATTR EncoderReader::handleInterrupt1() {
  if (instances_[1] != nullptr) {
    instances_[1]->handleInterrupt();
  }
}

void IRAM_ATTR EncoderReader::handleInterrupt() {
  const uint8_t current_state = readState();
  ticks_ += quadratureDelta(state_, current_state);
  state_ = current_state;
}

int8_t EncoderReader::quadratureDelta(uint8_t previous_state, uint8_t current_state) {
  static constexpr int8_t lookup_table[16] = {
      0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  return lookup_table[(previous_state << 2) | current_state];
}

uint8_t EncoderReader::readState() const {
  return static_cast<uint8_t>((digitalRead(pin_a_) << 1) | digitalRead(pin_b_));
}

}  // namespace robot
