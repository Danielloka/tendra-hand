// Leds: the two indicator LEDs.
//  - LED_SYNC (red, GPIO12) is reserved for camera-sync codes so a video can find it reliably;
//    the only other use is a solid "shutting down" before the critical-battery deep sleep.
//    The SYNC blink runs from a periodic esp_timer (50 ms slots), independent of task load, and
//    records the exact esp_timer time of its first (rising) edge.
//  - LED_STATUS (green, GPIO13) shows the state with patterns, updated by the health task.
#pragma once

#include <Arduino.h>
#include <atomic>
#include <esp_timer.h>

class Leds {
 public:
  enum class Pattern : uint8_t {
    Off,
    Unpaired,     // two short blinks every 1 s
    Idle,         // paired, not streaming: 50 ms blip every 2 s
    Streaming,    // on, with a short off-blink every 1 s (alive)
    LowBattery,   // 4 Hz blink
    Fault,        // 1 Hz, 50 % duty (sensor faults or bus recovery loop)
  };

  using SyncEdgeCallback = void (*)(uint8_t code, int64_t t_us, void* ctx);

  Leds(int sync_pin, int status_pin) : sync_pin_(sync_pin), status_pin_(status_pin) {}

  void begin();
  // Status LED pattern, evaluated in tick() (call every 10 ms from the health task).
  void setPattern(Pattern p) { pattern_ = p; }
  Pattern pattern() const { return pattern_; }
  void tick(uint32_t now_ms);

  // Starts a SYNC code (low 4 bits of `code`). Returns false if a blink is already running.
  bool startSync(uint8_t code, uint32_t bit_ms, SyncEdgeCallback cb, void* ctx);
  bool syncBusy() const { return sync_slot_.load() >= 0; }
  void syncSolid(bool on);   // only when no SYNC is running (shutdown indication)
  void allOff();

 private:
  static void syncTimerCb(void* arg);
  int sync_pin_, status_pin_;
  volatile Pattern pattern_ = Pattern::Off;
  esp_timer_handle_t sync_timer_ = nullptr;
  std::atomic<int> sync_slot_{-1};
  uint8_t sync_code_ = 0;
  SyncEdgeCallback sync_cb_ = nullptr;
  void* sync_ctx_ = nullptr;
};
