// Tendra glove configuration: pins, buses, timing, safety thresholds and the SensorMap (the single
// source of truth for every sensor on the glove). Used only by the `glove` build.
//
// Board: custom pod with ESP32-S3-MINI-1-N8 (8 MB quad flash, no PSRAM).
#pragma once

#include <stdint.h>

#include "protocol.h"

#define GLOVE_FW_VERSION "0.1.0"
constexpr uint16_t kGloveFwVersion = (0 << 8) | 1;  // major << 8 | minor (HELLO packet)

// ----- Pins (fixed by the PCB) -----
// GPIO0 is a strapping pin (LOW at reset = download mode): the button must not be held while
// resetting. GPIO19/20 = native USB, GPIO43/44 = UART0 debug pads: nothing is logged there.
constexpr int kPinButton = 0;        // BOOT/USER, active low, external pull-up
constexpr int kPinFgSda = 1;         // I2C1: MAX17048 only
constexpr int kPinFgScl = 2;
constexpr int kPinSensSda = 4;       // I2C0: sensor tree behind the TCA9548A
constexpr int kPinSensScl = 5;
constexpr int kPinMuxResetN = 6;     // TCA9548A RESET, active low
constexpr int kPinHandImuInt1 = 7;   // hand-plate ICM-42688-P INT1 (unused for now; data-ready later)
constexpr int kPinFgAlertN = 10;     // MAX17048 ALRT, active low (open drain)
constexpr int kPinPgoodN = 11;       // BQ24074 PGOOD, low = USB/input power present
constexpr int kPinLedSync = 12;      // red, active high, ~10 mA: camera sync blinks
constexpr int kPinLedStatus = 13;    // green, active high
constexpr int kPinSideSel = 14;      // input pull-up: jumper to GND = LEFT, open = RIGHT

// ----- I2C -----
// Sensor bus fixed at 400 kHz: the TCA9548A is specified to 400 kHz (TI SCPS207, fSCL max) and
// the TMAG5273 SDA sink is weak (2 mA), with 3.3 k pull-ups upstream and per channel.
constexpr uint32_t kSensorI2cHz = 400000;
constexpr uint32_t kFuelGaugeI2cHz = 100000;
constexpr uint16_t kI2cTimeoutMs = 2;          // per transaction; NACKs return at once
constexpr uint8_t kMuxAddr = 0x70;             // TCA9548A, A2..A0 = GND
constexpr uint8_t kFuelGaugeAddr = 0x36;       // MAX17048 (fixed, 0x6C/0x6D 8-bit)

// ----- Sampling -----
// Bus budget per 200 Hz sweep at 400 kHz (2.5 µs/bit, 9 bits/byte incl. ACK, ~2 bits START/STOP):
//   mux select      2 bytes  -> 20 bit -> ~50 µs  x 6 (serpentine order)   =  300 µs
//   TMAG 1-byte rd  8 bytes  -> 74 bit -> ~185 µs x 15                     = 2775 µs
//   ICM burst       2 + Sr + 15 bytes -> ~156 bit -> ~390 µs x 2           =  780 µs
//   total wire time ~3.86 ms of the 5.0 ms period (77 %)
// VERIFY: per-transaction software overhead of the arduino-esp32 3.3 I2C driver on the S3
// (23 transactions per sweep; at ~40 µs each that is ~0.9 ms more, i.e. ~4.8 ms). Watch
// sweep_us / kStatusOverrun on hardware; if 200 Hz overruns, use SET_RATE 100.

constexpr uint32_t kDefaultRateHz = 200;
constexpr uint32_t kAllowedRatesHz[] = {50, 100, 200};
// IMU temperature stays in every burst read: only 2 bytes per IMU (45 µs at 400 kHz), and it is
// the skin-temperature input of the health task.

// ----- Fault handling -----
constexpr uint8_t kSensorFaultThreshold = 5;    // consecutive errors before a sensor is "faulty"
constexpr uint32_t kSensorRetryMs = 1000;       // re-init period of a faulty sensor
constexpr uint8_t kMaxReinitsPerSweep = 1;      // bounds the extra time a re-init adds to a sweep
constexpr uint8_t kAllFailSweepsBeforeRecovery = 3;  // whole-bus failure -> recovery + mux reset
constexpr uint32_t kMinRecoveryIntervalMs = 500;

// ----- Battery / safety (health task) -----
constexpr float kSocWarnPct = 15.0f;
constexpr uint16_t kVcellWarnMv = 3500;
constexpr uint16_t kVcellCriticalMv = 3300;
constexpr uint16_t kVcellWakeMinMv = 3400;      // after a critical shutdown, stay asleep below this
constexpr uint8_t kCriticalConfirmCount = 3;    // consecutive 1 s readings below critical (load dips)
constexpr float kImuOverTempC = 45.0f;          // skin safety
constexpr float kImuOverTempClearC = 43.0f;     // hysteresis
constexpr uint32_t kTaskWdtTimeoutMs = 2000;

// ----- Button -----
constexpr uint32_t kButtonDebounceMs = 30;
constexpr uint32_t kButtonLongPressMs = 2000;

// ----- Radio -----
constexpr uint8_t kWifiChannel = 1;             // must match the dongle (kDongleChannel)
constexpr uint32_t kHelloPeriodMs = 500;        // pairing broadcast period while unpaired
constexpr uint16_t kTxFailsBeforeRepair = 200;  // consecutive unacked sends (~1 s at 200 Hz)
constexpr int kRadioRingSlots = 32;             // 160 ms of frames at 200 Hz
constexpr int kMaxInFlight = 4;                 // ESP-NOW sends awaiting their send callback

// ----- SYNC blink -----
constexpr uint32_t kSyncBitMs = 50;             // start bit + 4 data bits, MSB first

// ----- Task layout -----
// core 1: sampler (only task there besides Arduino's idle loop) -> I2C0 owned exclusively by it.
// core 0: radio (Wi-Fi/ESP-NOW live here too), health/UI, USB console.
constexpr int kSamplerCore = 1;
constexpr int kRadioCore = 0;
constexpr int kHealthCore = 0;
constexpr int kConsoleCore = 0;
constexpr int kSamplerPriority = 20;            // above everything app-level on core 1
constexpr int kRadioPriority = 10;
constexpr int kHealthPriority = 5;
constexpr int kConsolePriority = 2;

// ----- SensorMap -----
enum class SensorType : uint8_t { Tmag5273 = 0, Icm42688 = 1 };

struct SensorDesc {
  const char* name;
  uint8_t channel;   // TCA9548A channel 0..7
  uint8_t addr;      // 7-bit I2C address
  SensorType type;
  uint8_t slot;      // index into DataPacket::mag (TMAG) or ::imu (ICM)
};

// Order = packet bit order (sensor_ok bit i). Rows MUST be grouped by channel (the sampler reads
// a channel's sensors back to back and test_protocol checks the grouping). TMAG part variants:
// A1 = 0x35, B1 = 0x22, C1 = 0x78, D1 = 0x44 (factory addresses, TMAG5273 datasheet Table 6-2).
constexpr SensorDesc kSensorMap[] = {
    // CH0 index finger
    {"index_pip", 0, 0x35, SensorType::Tmag5273, 0},
    {"index_dip", 0, 0x22, SensorType::Tmag5273, 1},
    // CH1 middle finger
    {"middle_pip", 1, 0x35, SensorType::Tmag5273, 2},
    {"middle_dip", 1, 0x22, SensorType::Tmag5273, 3},
    // CH2 ring finger
    {"ring_pip", 2, 0x35, SensorType::Tmag5273, 4},
    {"ring_dip", 2, 0x22, SensorType::Tmag5273, 5},
    // CH3 little finger
    {"little_pip", 3, 0x35, SensorType::Tmag5273, 6},
    {"little_dip", 3, 0x22, SensorType::Tmag5273, 7},
    // CH4 thumb
    {"thumb_mcp", 4, 0x35, SensorType::Tmag5273, 8},
    {"thumb_ip", 4, 0x22, SensorType::Tmag5273, 9},
    {"thumb_imu", 4, 0x68, SensorType::Icm42688, 0},
    // CH5 back-of-hand plate: MCP sensors
    {"index_mcp", 5, 0x35, SensorType::Tmag5273, 10},
    {"middle_mcp", 5, 0x22, SensorType::Tmag5273, 11},
    {"ring_mcp", 5, 0x78, SensorType::Tmag5273, 12},
    {"little_mcp", 5, 0x44, SensorType::Tmag5273, 13},
    // CH6 plate: hand IMU (INT1 -> GPIO7) + reference magnetometer (no magnet: ambient field)
    {"hand_imu", 6, 0x68, SensorType::Icm42688, 1},
    {"ref_mag", 6, 0x35, SensorType::Tmag5273, 14},
    // CH7 spare: scanned only
};
constexpr int kSensorCount = sizeof(kSensorMap) / sizeof(kSensorMap[0]);
constexpr int kMuxChannels = 8;
constexpr int kScanOnlyChannel = 7;

static_assert(kSensorCount == tg::kNumSensors, "SensorMap must list 15 TMAG + 2 IMU");
static_assert(kSensorCount <= 32, "sensor_ok is a 32-bit mask");

constexpr int count_type(SensorType t) {
  int n = 0;
  for (const auto& s : kSensorMap) n += (s.type == t) ? 1 : 0;
  return n;
}
static_assert(count_type(SensorType::Tmag5273) == tg::kNumMag, "15 TMAG5273");
static_assert(count_type(SensorType::Icm42688) == tg::kNumImu, "2 ICM-42688-P");

constexpr bool map_grouped_by_channel() {
  for (int i = 1; i < kSensorCount; ++i)
    if (kSensorMap[i].channel < kSensorMap[i - 1].channel) return false;
  return true;
}
static_assert(map_grouped_by_channel(), "SensorMap rows must be sorted by mux channel");

constexpr bool map_slots_unique() {
  for (int i = 0; i < kSensorCount; ++i)
    for (int j = i + 1; j < kSensorCount; ++j)
      if (kSensorMap[i].type == kSensorMap[j].type && kSensorMap[i].slot == kSensorMap[j].slot)
        return false;
  return true;
}
static_assert(map_slots_unique(), "duplicate packet slot in SensorMap");

constexpr bool map_addrs_unique_per_channel() {
  for (int i = 0; i < kSensorCount; ++i)
    for (int j = i + 1; j < kSensorCount; ++j)
      if (kSensorMap[i].channel == kSensorMap[j].channel && kSensorMap[i].addr == kSensorMap[j].addr)
        return false;
  return true;
}
static_assert(map_addrs_unique_per_channel(), "two sensors share an address on one channel");

constexpr bool map_avoids_mux_addr() {
  for (const auto& s : kSensorMap)
    if (s.addr == kMuxAddr) return false;
  return true;
}
static_assert(map_avoids_mux_addr(), "a sensor address collides with the TCA9548A");
