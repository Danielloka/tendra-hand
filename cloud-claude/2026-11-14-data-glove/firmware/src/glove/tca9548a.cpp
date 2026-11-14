#include "tca9548a.h"

void Tca9548a::begin() {
  pinMode(reset_pin_, OUTPUT);
  hardwareReset();
}

void Tca9548a::hardwareReset() {
  digitalWrite(reset_pin_, LOW);
  delayMicroseconds(2);  // datasheet minimum is 6 ns; 2 µs is generous and covers the RC on the line
  digitalWrite(reset_pin_, HIGH);
  delayMicroseconds(2);
  current_ = kNone;      // reset state: all channels deselected
}

bool Tca9548a::select(int8_t channel) {
  if (channel == current_) return true;
  const uint8_t mask = (channel >= 0 && channel < 8) ? static_cast<uint8_t>(1u << channel) : 0;
  if (!bus_.write(addr_, &mask, 1)) {
    current_ = kUnknown;  // state unknown after a failed write: force the next select to write
    return false;
  }
  current_ = (mask == 0) ? kNone : channel;
  ++switches_;
  return true;
}

bool Tca9548a::readControl(uint8_t& value) { return bus_.read(addr_, &value, 1); }
