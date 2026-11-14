// Button: debounced active-low push button with short-press and long-press events.
// A long press fires once when the hold time is reached (not on release), so the user gets
// feedback while still holding; the following release then produces no short press.
#pragma once

#include <Arduino.h>

class Button {
 public:
  enum class Event : uint8_t { None, ShortPress, LongPress };

  Button(int pin, uint32_t debounce_ms, uint32_t long_ms)
      : pin_(pin), debounce_ms_(debounce_ms), long_ms_(long_ms) {}

  void begin() { pinMode(pin_, INPUT); }   // external pull-up on the PCB
  Event poll(uint32_t now_ms);             // call every <= 10 ms
  bool pressed() const { return stable_pressed_; }

 private:
  int pin_;
  uint32_t debounce_ms_, long_ms_;
  bool raw_last_ = false;
  bool stable_pressed_ = false;
  bool long_fired_ = false;
  uint32_t raw_change_ms_ = 0;
  uint32_t press_start_ms_ = 0;
};
