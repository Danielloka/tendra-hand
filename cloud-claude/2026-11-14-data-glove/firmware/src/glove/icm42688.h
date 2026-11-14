// ICM-42688-P 6-axis IMU over I2C (TDK DS-000347, checked against rev 1.2 of the datasheet).
// VERIFY: re-check these values against the current revision (1.9 on tdk.com) before release.
//   I2C address b110100X: 0x68 with AP_AD0 low (datasheet I2C section).
//   Bank 0 registers used:
//     0x11 DEVICE_CONFIG   [0] SOFT_RESET_CONFIG (wait 1 ms after writing 1)
//     0x1D..0x2A TEMP_DATA1/0, ACCEL_DATA_X1..Z0, GYRO_DATA_X1..Z0 (big endian by default,
//                INTF_CONFIG0.SENSOR_DATA_ENDIAN = 1)
//     0x4E PWR_MGMT0       [5] TEMP_DIS, [3:2] GYRO_MODE (11 = LN), [1:0] ACCEL_MODE (11 = LN);
//                          no register writes for 200 µs after leaving OFF; gyro >= 45 ms on
//     0x4F GYRO_CONFIG0    [7:5] GYRO_FS_SEL (001 = ±1000 dps), [3:0] GYRO_ODR (0110 = 1 kHz)
//     0x50 ACCEL_CONFIG0   [7:5] ACCEL_FS_SEL (010 = ±4 g), [3:0] ACCEL_ODR (0110 = 1 kHz, LN)
//     0x75 WHO_AM_I        = 0x47
//     0x76 REG_BANK_SEL
//   Scale: ±4 g -> 8192 LSB/g, ±1000 dps -> 32.8 LSB/dps, temp °C = raw / 132.48 + 25.
// The sampler reads at 200 Hz from 1 kHz ODR data (the UI path's default anti-alias filter
// applies). INT1 of the hand IMU goes to GPIO7 for a later data-ready / tap interrupt.
#pragma once

#include "i2c_bus.h"

class Icm42688 {
 public:
  struct Sample {
    int16_t temp_raw;
    int16_t accel[3];
    int16_t gyro[3];
  };
  enum class InitResult : uint8_t { Ok, NoAck, BadWhoAmI, WriteFailed };

  Icm42688(I2cBus& bus, uint8_t addr) : bus_(bus), addr_(addr) {}

  // Soft reset + configuration. Blocks ~1.3 ms (reset wait + 200 µs power-up guard); called from
  // the sampler only at boot and as a rate-limited re-init of a faulty sensor.
  InitResult init();
  bool read(Sample& s);   // one 14-byte burst: temp, accel xyz, gyro xyz
  bool powerOff();        // PWR_MGMT0 = 0: accel, gyro and temperature off
  uint8_t whoAmI() const { return who_; }

  static float tempC(int16_t raw) { return raw / 132.48f + 25.0f; }
  // A just-reset or powered-down IMU reports 0x8000 ("invalid data", INTF_CONFIG0 default).
  static constexpr int16_t kInvalid = INT16_MIN;

 private:
  I2cBus& bus_;
  uint8_t addr_;
  uint8_t who_ = 0;
};
