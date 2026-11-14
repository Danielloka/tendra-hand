// TMAG5273x1 3-axis linear Hall sensor (TI SLYS045C, rev. April 2026). Register facts used here
// were checked against that datasheet (Table 8-1 and §8.1.x):
//   0x00 DEVICE_CONFIG_1  [7] CRC_EN, [6:5] MAG_TEMPCO, [4:2] CONV_AVG, [1:0] I2C_RD
//   0x01 DEVICE_CONFIG_2  [7:5] THR_HYST, [4] LP_LN (1 = low noise), [3] I2C_GLITCH_FILTER (0 = on),
//                         [2] TRIGGER_MODE, [1:0] OPERATING_MODE (0 standby, 1 sleep, 2 continuous, 3 W&S)
//   0x02 SENSOR_CONFIG_1  [7:4] MAG_CH_EN (7 = X,Y,Z), [3:0] SLEEPTIME
//   0x03 SENSOR_CONFIG_2  [1] X_Y_RANGE, [0] Z_RANGE (0 = ±40 mT on x1 parts)
//   0x07 T_CONFIG         [0] T_CH_EN
//   0x08 INT_CONFIG_1     (0 = no interrupt)
//   0x0D DEVICE_ID        [1:0] VER (1 = ±40/±80 mT parts)
//   0x0E/0x0F MANUFACTURER_ID LSB/MSB = 0x49 / 0x54 ("TI")
//   0x12..0x17 X/Y/Z MSB/LSB results (two's complement), 0x18 CONV_STATUS
//   CONV_STATUS: [7:5] SET_COUNT, [4] POR (write 1 to clear), [1] DIAG_STATUS, [0] RESULT_STATUS
//
// Conversion: CONV_AVG = 4 (16x) -> 0.8 kSPS for 3 axes (Table 6-4), i.e. one XYZ set every
// 1.25 ms: 4 fresh sets per 5 ms sweep, ~4x less noise than 1x. (8x = 625 µs would also fit, but
// then exactly 8 conversions fall in a 5 ms period and the 3-bit SET_COUNT could not detect a
// stuck sensor.)
// Read path: I2C_RD = 1 ("1-byte read command for 16-bit data"): a bare address+R returns
// X_MSB X_LSB Y_MSB Y_LSB Z_MSB Z_LSB CONV_STATUS (7 bytes, T channel disabled, CRC off), so a
// sample costs 8 bytes on the bus instead of 10 for a register read.
// NOTE: in I2C_RD = 1 mode a normal register read is not available, so init() first writes
// DEVICE_CONFIG_1 = 0 (writes always work) before reading the ID registers.
#pragma once

#include "i2c_bus.h"

class Tmag5273 {
 public:
  struct Sample {
    int16_t xyz[3];
    uint8_t conv_status;
    bool saturated;   // any axis at (or within kSatMargin of) full scale
    bool stale;       // SET_COUNT unchanged since the previous read: no new conversion
    bool por;         // device reset itself (configuration lost): re-init needed
    bool diag_fail;   // DIAG_STATUS set
  };

  enum class InitResult : uint8_t { Ok, NoAck, BadManufacturer, BadDeviceId, WriteFailed };

  Tmag5273(I2cBus& bus, uint8_t addr) : bus_(bus), addr_(addr) {}

  InitResult init();       // verify IDs, configure, enter continuous mode
  bool read(Sample& s);    // 1-byte read command; false on bus error
  bool sleep();            // OPERATING_MODE = sleep (5 nA typ); needs standard mode first
  static constexpr uint32_t kConversionUs = 1250;   // 16x averaging, 3 axes
  uint8_t deviceId() const { return device_id_; }
  uint8_t addr() const { return addr_; }

  // Full-scale is int16 range at ±40 mT (820 LSB/mT * 40 mT = 32800 > 32767).
  // VERIFY: whether the device clips exactly at ±32767/-32768 or a little below; the margin
  // treats the top ~0.1 % as saturated.
  static constexpr int16_t kSatMargin = 32;

 private:
  bool writeChecked(uint8_t reg, uint8_t v);
  I2cBus& bus_;
  uint8_t addr_;
  uint8_t device_id_ = 0;
  int last_set_count_ = -1;
};
