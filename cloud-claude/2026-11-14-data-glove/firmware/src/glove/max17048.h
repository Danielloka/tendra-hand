// MAX17048 1-cell ModelGauge fuel gauge (Maxim 19-6171, register summary Table 2), on I2C1.
// All registers are 16-bit, MSB first. Address 0x36 (0x6C/0x6D 8-bit).
//   0x02 VCELL   78.125 µV/LSB
//   0x04 SOC     1/256 %/LSB (upper byte = whole %)
//   0x08 VERSION 0x001x
//   0x0C CONFIG  [15:8] RCOMP (default 0x97), [7] SLEEP, [6] ALSC, [5] ALRT, [4:0] ATHD
//                (empty alert at (32 - ATHD) %)
//   0x14 VALRT   [15:8] MIN, [7:0] MAX, 20 mV/LSB
//   0x16 CRATE   0.208 %/hr/LSB, signed
//   0x1A STATUS  [14] EnVR, [13] SC, [12] HD, [11] VR, [10] VL, [9] VH, [8] RI
//   0xFE CMD     write 0x5400 = POR (no ACK)
// VERIFY: the default ModelGauge model suits the chosen LiPo cell (else load a custom model).
// The default ModelGauge model is used (no custom TABLE): fine for generic LiPo cells; SOC
// accuracy improves with a characterised model. RCOMP temperature compensation is not applied
// (the cell is near skin temperature).
#pragma once

#include "i2c_bus.h"

class Max17048 {
 public:
  struct Reading {
    uint16_t vcell_mv;
    float soc_pct;
    float crate_pct_per_hr;
    uint8_t status_hi;   // STATUS[15:8]
    bool alert;          // CONFIG.ALRT
  };

  explicit Max17048(I2cBus& bus, uint8_t addr = 0x36) : bus_(bus), addr_(addr) {}

  // Checks VERSION, programs the empty-alert threshold and the voltage alert window, clears
  // STATUS.RI. Leaves RCOMP at its POR value.
  bool begin(uint8_t empty_alert_pct, uint16_t valrt_min_mv, uint16_t valrt_max_mv);
  bool read(Reading& r);
  bool clearAlert();   // clears CONFIG.ALRT and the STATUS alert bits
  uint16_t version() const { return version_; }

 private:
  I2cBus& bus_;
  uint8_t addr_;
  uint16_t version_ = 0;
};
