#include "health.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>

#include "glove_config.h"
#include "glove_state.h"
#include "icm42688.h"

Health g_health;

Health::Health()
    : fg_bus_(Wire1, kPinFgSda, kPinFgScl, kFuelGaugeI2cHz, 10),
      fg_(fg_bus_, kFuelGaugeAddr),
      leds_(kPinLedSync, kPinLedStatus),
      button_(kPinButton, kButtonDebounceMs, kButtonLongPressMs) {}

bool Health::begin() {
  leds_.begin();
  button_.begin();
  pinMode(kPinPgoodN, INPUT);     // BQ24074 PGOOD is open drain; pull-up on the PCB
  pinMode(kPinFgAlertN, INPUT);   // MAX17048 ALRT is open drain; pull-up on the PCB
  // VERIFY: PCB pull-ups exist on PGOOD_N and FG_ALRT_N; if not, use INPUT_PULLUP here.
  fg_ok_ = fg_bus_.begin() &&
           fg_.begin(static_cast<uint8_t>(kSocWarnPct), kVcellWarnMv, 5100 /* VALRT.MAX = 0xFF */);
  if (fg_ok_) readBattery();
  return fg_ok_;
}

Health::Battery Health::battery() const {
  portENTER_CRITICAL(&lock_);
  Battery b = batt_;
  portEXIT_CRITICAL(&lock_);
  return b;
}

void Health::start() {
  xTaskCreatePinnedToCore(&Health::taskEntry, "health", 4096, this, kHealthPriority, nullptr, kHealthCore);
}

void Health::taskEntry(void* arg) { static_cast<Health*>(arg)->run(); }

void Health::readBattery() {
  Max17048::Reading r;
  const bool charging = digitalRead(kPinPgoodN) == LOW;
  Battery b = battery();
  b.charging = charging;
  if (fg_ok_ && fg_.read(r)) {
    b.valid = true;
    b.mv = r.vcell_mv;
    b.soc = r.soc_pct;
    b.crate = r.crate_pct_per_hr;
    b.status_hi = r.status_hi;
    if (r.alert) fg_.clearAlert();   // polled anyway; keeps the ALRT pin released
  } else {
    b.valid = false;
    fg_ok_ = fg_bus_.begin() && fg_.begin(static_cast<uint8_t>(kSocWarnPct), kVcellWarnMv, 5100);
  }
  portENTER_CRITICAL(&lock_);
  batt_ = b;
  portEXIT_CRITICAL(&lock_);
  if (b.valid) {
    g_state.batt_mv.store(b.mv);
    const float soc = b.soc > 100.0f ? 100.0f : b.soc;
    g_state.batt_soc.store(static_cast<uint8_t>(soc + 0.5f));
  }
}

void Health::updatePattern(uint32_t now_ms) {
  (void)now_ms;
  const uint16_t f = g_state.health_flags.load();
  Leds::Pattern p;
  if (f & (tg::kStatusLowBattery | tg::kStatusCriticalBattery)) {
    p = Leds::Pattern::LowBattery;
  } else if (g_state.faulty_mask.load() != 0 || (f & tg::kStatusFuelGaugeFault)) {
    p = Leds::Pattern::Fault;
  } else if (!g_state.linked.load()) {
    p = Leds::Pattern::Unpaired;
  } else if (g_state.streaming.load()) {
    p = Leds::Pattern::Streaming;
  } else {
    p = Leds::Pattern::Idle;
  }
  leds_.setPattern(p);
}

void Health::run() {
  esp_task_wdt_add(nullptr);
  TickType_t wake = xTaskGetTickCount();
  uint32_t tick = 0;
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    esp_task_wdt_reset();
    const uint32_t now = millis();
    ++tick;

    // --- 100 Hz: button + LEDs ---
    switch (button_.poll(now)) {
      case Button::Event::ShortPress:
        post_event(tg::EventType::Mark);
        break;
      case Button::Event::LongPress: {
        const bool on = !g_state.streaming.load();
        g_state.streaming.store(on);
        post_event(on ? tg::EventType::StreamStart : tg::EventType::StreamStop);
        break;
      }
      case Button::Event::None:
        break;
    }
    leds_.tick(now);

    if (tick % 10 != 0) continue;
    // --- 10 Hz: flags, temperature, pattern ---
    uint16_t flags = g_state.health_flags.load();
    const bool charging = digitalRead(kPinPgoodN) == LOW;
    flags = charging ? (flags | tg::kStatusCharging) : (flags & ~tg::kStatusCharging);

    float tmax = -100.0f;
    const uint32_t ok = g_state.sensor_ok.load();
    for (int i = 0; i < kSensorCount; ++i) {
      if (kSensorMap[i].type != SensorType::Icm42688 || !(ok & (1u << i))) continue;
      const int16_t raw = g_state.imu_temp_raw[kSensorMap[i].slot].load();
      if (raw == Icm42688::kInvalid) continue;
      const float t = Icm42688::tempC(raw);
      if (t > tmax) tmax = t;
    }
    if (!hot_ && tmax > kImuOverTempC) {
      hot_ = true;
      post_event(tg::EventType::OverTemp, 1, static_cast<uint16_t>(tmax * 100));
    } else if (hot_ && tmax < kImuOverTempClearC) {
      hot_ = false;
      post_event(tg::EventType::OverTemp, 0, static_cast<uint16_t>(tmax * 100));
    }
    flags = hot_ ? (flags | tg::kStatusOverTemp) : (flags & ~tg::kStatusOverTemp);

    const bool alert_pin = digitalRead(kPinFgAlertN) == LOW;
    if (tick % 100 == 0 || alert_pin) {
      // --- 1 Hz (or on ALRT): battery ---
      readBattery();
      const Battery b = battery();
      flags = b.valid ? (flags & ~tg::kStatusFuelGaugeFault) : (flags | tg::kStatusFuelGaugeFault);
      if (b.valid) {
        const bool low = b.soc < kSocWarnPct || b.mv < kVcellWarnMv;
        flags = low ? (flags | tg::kStatusLowBattery) : (flags & ~tg::kStatusLowBattery);
        if (low && !low_reported_) post_event(tg::EventType::BatteryLow, static_cast<uint8_t>(b.soc), b.mv);
        low_reported_ = low;
        if (b.mv < kVcellCriticalMv && !b.charging) {
          if (++critical_count_ >= kCriticalConfirmCount) {
            flags |= tg::kStatusCriticalBattery;
            g_state.health_flags.store(flags);
            criticalShutdown();   // does not return
          }
        } else {
          critical_count_ = 0;
        }
      }
    }
    g_state.health_flags.store(flags);
    updatePattern(now);
  }
}

void Health::criticalShutdown() {
  const Battery b = battery();
  post_event(tg::EventType::BatteryCritical, 0, b.mv);
  g_state.streaming.store(false);
  post_event(tg::EventType::StreamStop);
  // Give the radio ~200 ms to deliver the events.
  for (int i = 0; i < 20; ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_task_wdt_reset();
  }
  g_state.shutdown_request.store(true);
  for (int i = 0; i < 50 && !g_state.shutdown_done.load(); ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_task_wdt_reset();
  }
  leds_.setPattern(Leds::Pattern::Off);
  leds_.allOff();
  leds_.syncSolid(true);
  for (int i = 0; i < 100; ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_task_wdt_reset();
  }
  leds_.allOff();
  enterProtectiveSleep();
}

bool Health::cellTooLowToRun() {
  // Minimal standalone read (used before the tasks exist).
  I2cBus bus(Wire1, kPinFgSda, kPinFgScl, kFuelGaugeI2cHz, 10);
  if (!bus.begin()) return false;
  pinMode(kPinPgoodN, INPUT);
  const bool usb = digitalRead(kPinPgoodN) == LOW;
  uint16_t raw = 0;
  const bool ok = bus.readReg16(kFuelGaugeAddr, 0x02, raw);
  bus.end();
  if (!ok || usb) return false;   // unknown -> run (the 1 s check takes over); USB -> run
  const uint32_t mv = (static_cast<uint32_t>(raw) * 78125u) / 1000000u;
  return mv < kVcellWakeMinMv;
}

void Health::enterProtectiveSleep() {
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  // Wait for the button to be released, otherwise ext0 would wake us at once.
  pinMode(kPinButton, INPUT);
  for (int i = 0; i < 500 && digitalRead(kPinButton) == LOW; ++i) delay(10);
  // VERIFY: GPIO0 is a strapping pin; strapping is latched only on chip reset, not on deep-sleep
  // wake (ESP32-S3 TRM "Reset and Clock"), so a button wake must not enter download mode.
  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(kPinButton), 0);
  // The MAX17048 keeps running (hibernate ~3 µA) and is re-read on wake.
  esp_deep_sleep_start();
}
