// GloveState: the small set of values shared between tasks. Scalars are std::atomic (32-bit or
// smaller: lock-free on the ESP32-S3); the latest frame snapshot is guarded by a spinlock and
// copied in/out (155 bytes, < 1 µs). No task ever waits on another task through this struct.
#pragma once

#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "glove_config.h"
#include "protocol.h"
#include "spsc_ring.h"

struct GloveState {
  tg::Side side = tg::Side::Unknown;

  // Control (written by radio command handler, button, console; read by the sampler).
  std::atomic<bool> streaming{false};
  std::atomic<bool> usb_raw{false};
  std::atomic<uint32_t> rate_hz{kDefaultRateHz};
  std::atomic<bool> rate_changed{false};
  std::atomic<bool> shutdown_request{false};
  std::atomic<bool> shutdown_done{false};
  std::atomic<bool> scan_request{false};
  std::atomic<bool> scan_done{false};

  // Health -> sampler (ORed into DataPacket::status) and battery numbers.
  std::atomic<uint16_t> health_flags{0};
  std::atomic<uint8_t> batt_soc{255};
  std::atomic<uint16_t> batt_mv{0};

  // Sampler -> everyone.
  std::atomic<uint32_t> seq{0};
  std::atomic<uint32_t> sensor_ok{0};
  std::atomic<uint32_t> faulty_mask{0};
  std::atomic<int16_t> imu_temp_raw[tg::kNumImu] = {};
  std::atomic<uint32_t> sweeps{0};
  std::atomic<uint32_t> overruns{0};
  std::atomic<uint32_t> last_sweep_us{0};
  std::atomic<uint32_t> max_sweep_us{0};
  std::atomic<uint32_t> bus_recoveries{0};

  // Radio -> everyone.
  std::atomic<bool> linked{false};      // dongle MAC known and recent sends ACKed
  std::atomic<uint32_t> tx_ok{0};
  std::atomic<uint32_t> tx_fail{0};

  // Frames: sampler -> radio, and sampler -> USB console (raw mode). Separate rings so a slow
  // USB host can never cost radio frames and vice versa.
  tg::SpscRing<tg::DataPacket, kRadioRingSlots> radio_ring;
  tg::SpscRing<tg::DataPacket, 16> usb_ring;

  // Events: any task -> radio (queue, multi-producer, never waited on by producers).
  QueueHandle_t radio_events = nullptr;
  QueueHandle_t usb_events = nullptr;
  TaskHandle_t radio_task = nullptr;

  // Scan results (sampler writes while scanning, console reads after scan_done):
  // index 0 = upstream (no channel selected), 1..8 = channel 0..7; bit a = address a ACKed.
  uint32_t scan_bits[kMuxChannels + 1][4] = {};

  // Latest frame snapshot (console `d`).
  void putSnapshot(const tg::DataPacket& p) {
    portENTER_CRITICAL(&snap_lock_);
    snapshot_ = p;
    portEXIT_CRITICAL(&snap_lock_);
  }
  void getSnapshot(tg::DataPacket& p) {
    portENTER_CRITICAL(&snap_lock_);
    p = snapshot_;
    portEXIT_CRITICAL(&snap_lock_);
  }

 private:
  portMUX_TYPE snap_lock_ = portMUX_INITIALIZER_UNLOCKED;
  tg::DataPacket snapshot_{};
};

extern GloveState g_state;

// Posts an event to the radio (and to USB when raw mode is on). Never blocks; callable from any
// task or esp_timer callback (not from an ISR).
void post_event(tg::EventType type, uint8_t arg = 0, uint16_t arg2 = 0, int64_t t_us = -1);
