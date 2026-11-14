#include "button.h"

Button::Event Button::poll(uint32_t now_ms) {
  const bool raw = digitalRead(pin_) == LOW;
  if (raw != raw_last_) {
    raw_last_ = raw;
    raw_change_ms_ = now_ms;
  }
  Event ev = Event::None;
  if (raw != stable_pressed_ && (now_ms - raw_change_ms_) >= debounce_ms_) {
    stable_pressed_ = raw;
    if (stable_pressed_) {
      press_start_ms_ = now_ms;
      long_fired_ = false;
    } else if (!long_fired_) {
      ev = Event::ShortPress;
    }
  }
  if (stable_pressed_ && !long_fired_ && (now_ms - press_start_ms_) >= long_ms_) {
    long_fired_ = true;
    ev = Event::LongPress;
  }
  return ev;
}
