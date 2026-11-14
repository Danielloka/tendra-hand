// Sampler: owns the sensor bus (I2C0 + TCA9548A + 15 TMAG5273 + 2 ICM-42688-P) and produces one
// DataPacket per period.
//
// Timing: a periodic esp_timer (default 5 ms) notifies the sampler task (core 1, priority 20).
// Each sweep:
//   t_us = esp_timer_get_time()           (start of sweep = packet timestamp)
//   read every healthy sensor, grouped by mux channel; the channel order alternates
//   (0..6, then 6..0) so the channel left selected by one sweep is the first of the next:
//   6 switches per sweep instead of 7
//   at most one re-init of a faulty sensor (retry every 1 s)
//   sweep_us = duration
// Overrun = the sweep took longer than the period, or the timer notified more than once
// before the task took the notification (a tick was missed).
//
// Fault handling: kSensorFaultThreshold consecutive errors mark a sensor faulty (skipped in
// sweeps, retried every kSensorRetryMs). If every sensor fails for kAllFailSweepsBeforeRecovery
// sweeps in a row: I2C bus recovery (9 clocks + STOP), mux hardware reset, per-channel SDA check,
// then re-init of everything. Nothing in the sweep can block for more than the I2C timeout.
#pragma once

#include <Arduino.h>
#include <esp_timer.h>

#include "glove_config.h"
#include "glove_state.h"
#include "i2c_bus.h"
#include "icm42688.h"
#include "tca9548a.h"
#include "tmag5273.h"

class Sampler {
 public:
  Sampler();
  bool begin();             // bus + sensors init (from setup, before the task starts)
  void start();             // creates the task and the timer
  int sensorsOk() const;    // number of sensors that initialised at boot

 private:
  struct SensorState {
    uint8_t consecutive_errors = 0;
    bool faulty = false;
    bool needs_init = true;
    uint32_t next_retry_ms = 0;
    uint32_t total_errors = 0;
  };

  static void taskEntry(void* arg);
  static void timerCb(void* arg);
  void run();
  void sweep(uint32_t missed_ticks);
  bool readSensor(int i, tg::DataPacket& p, bool& saturated, bool& stale);
  bool initSensor(int i);
  void noteResult(int i, bool ok, uint32_t now_ms);
  void recoverBus();
  void scanSlice();
  void shutdownSensors();
  void applyRate();

  I2cBus bus_;
  Tca9548a mux_;
  Tmag5273* tmag_[tg::kNumMag];
  Icm42688* imu_[tg::kNumImu];
  SensorState st_[kSensorCount];
  int boot_ok_ = 0;

  TaskHandle_t task_ = nullptr;
  esp_timer_handle_t timer_ = nullptr;
  uint32_t period_us_ = 1000000 / kDefaultRateHz;
  bool forward_ = true;           // serpentine direction
  uint8_t all_fail_sweeps_ = 0;
  uint32_t last_recovery_ms_ = 0;
  bool recovered_flag_ = false;
  int scan_index_ = -1;           // -1 = idle; else (slot * 128 + addr)
};
