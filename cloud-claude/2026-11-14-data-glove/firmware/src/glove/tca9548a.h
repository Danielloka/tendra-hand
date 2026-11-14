// TCA9548A 8-channel I2C switch (TI SCPS207). Replaced the PCA9848 in the BOM (stock); the
// control-register protocol is the same: one data byte, bit n = channel n enabled.
//  - Address 0x70 with A2..A0 = GND.
//  - RESET active low; tW(L) >= 6 ns, tREC(STA) = 0 ns (datasheet §5.7). A reset deselects all
//    channels, so the cached channel is invalidated.
//  - fSCL max 400 kHz.
// The driver enables exactly one channel at a time (all branches carry 0x35/0x22, so two open
// channels would collide) and caches the selection to skip redundant writes.
#pragma once

#include "i2c_bus.h"

class Tca9548a {
 public:
  static constexpr int8_t kNone = -1;

  Tca9548a(I2cBus& bus, uint8_t addr, int reset_pin) : bus_(bus), addr_(addr), reset_pin_(reset_pin) {}

  void begin();                 // configures the reset pin, pulses reset, deselects everything
  bool select(int8_t channel);  // 0..7, or kNone to disconnect all; no-op if already selected
  bool deselectAll() { return select(kNone); }
  void hardwareReset();
  bool readControl(uint8_t& value);
  int8_t current() const { return current_; }
  void invalidate() { current_ = kUnknown; }
  uint32_t switchCount() const { return switches_; }

 private:
  static constexpr int8_t kUnknown = -2;
  I2cBus& bus_;
  uint8_t addr_;
  int reset_pin_;
  int8_t current_ = kUnknown;
  uint32_t switches_ = 0;
};
