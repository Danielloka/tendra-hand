// Health: safety + UI task on core 0, 10 ms tick.
//   every 10 ms : button (short press = MARK event, 2 s long press = start/stop streaming), LEDs
//   every 100 ms: PGOOD (charging flag), IMU temperature (> 45 °C flag + event), LED pattern,
//                 fuel-gauge ALRT pin (triggers an immediate battery read)
//   every 1 s   : MAX17048 (SOC, VCELL, CRATE) on I2C1 (owned by this task only)
// Battery policy:
//   warn     SOC < 15 % or VCELL < 3.50 V  -> kStatusLowBattery, BatteryLow event, LED 4 Hz blink
//   critical VCELL < 3.30 V for 3 readings in a row, and no USB power
//            -> BatteryCritical event, stop streaming, sensors to sleep/off, red LED 1 s,
//               deep sleep with wake on the button (GPIO0 low). On wake the firmware checks VCELL
//               first and goes straight back to sleep below 3.40 V without USB power.
// The brown-out detector is left at its default (enabled); nothing here changes it.
#pragma once

#include <Arduino.h>

#include "button.h"
#include "i2c_bus.h"
#include "leds.h"
#include "max17048.h"

class Health {
 public:
  Health();
  bool begin();   // fuel gauge on I2C1 + button
  void start();
  Leds& leds() { return leds_; }

  struct Battery {
    bool valid;
    uint16_t mv;
    float soc;
    float crate;
    uint8_t status_hi;
    bool charging;
  };
  Battery battery() const;

  // Quick check used at boot after a deep-sleep wake: true if the cell is too low to run.
  static bool cellTooLowToRun();
  [[noreturn]] static void enterProtectiveSleep();

 private:
  static void taskEntry(void* arg);
  void run();
  void readBattery();
  void criticalShutdown();
  void updatePattern(uint32_t now_ms);

  I2cBus fg_bus_;
  Max17048 fg_;
  Leds leds_;
  Button button_;
  bool fg_ok_ = false;
  uint8_t critical_count_ = 0;
  bool low_reported_ = false;
  bool hot_ = false;
  Battery batt_{};
  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
};

extern Health g_health;
