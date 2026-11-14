#include "tmag5273.h"

namespace {
constexpr uint8_t kRegDeviceConfig1 = 0x00;
constexpr uint8_t kRegDeviceConfig2 = 0x01;
constexpr uint8_t kRegSensorConfig1 = 0x02;
constexpr uint8_t kRegSensorConfig2 = 0x03;
constexpr uint8_t kRegTConfig = 0x07;
constexpr uint8_t kRegIntConfig1 = 0x08;
constexpr uint8_t kRegDeviceId = 0x0D;
constexpr uint8_t kRegManufacturerLsb = 0x0E;   // 0x49, then 0x0F = 0x54
constexpr uint8_t kRegConvStatus = 0x18;

constexpr uint8_t kManufacturerLsb = 0x49;
constexpr uint8_t kManufacturerMsb = 0x54;
constexpr uint8_t kDeviceVer40mT = 0x01;

constexpr uint8_t kConvAvg16x = 4;              // 0.8 kSPS, 3 axes
constexpr uint8_t kI2cRd16Bit1Byte = 1;
constexpr uint8_t kDevCfg1Run = (kConvAvg16x << 2) | kI2cRd16Bit1Byte;  // 0x11, CRC off, no tempco
constexpr uint8_t kLowNoise = 1u << 4;
constexpr uint8_t kOpContinuous = 2;
constexpr uint8_t kOpSleep = 1;
constexpr uint8_t kMagChXYZ = 0x7 << 4;         // SLEEPTIME = 0
constexpr uint8_t kRange40mT = 0x00;            // X_Y_RANGE = 0, Z_RANGE = 0
constexpr uint8_t kStatusPor = 1u << 4;
constexpr uint8_t kStatusDiag = 1u << 1;
}  // namespace

bool Tmag5273::writeChecked(uint8_t reg, uint8_t v) { return bus_.writeReg(addr_, reg, v); }

Tmag5273::InitResult Tmag5273::init() {
  last_set_count_ = -1;
  // Back to standard 3-byte read mode so the ID registers can be read. This write also wakes the
  // device if it was in sleep mode (a valid address wakes it; the first write may not be ACKed,
  // datasheet §5.11 note 1 - tstart_sleep = 50 µs), hence one retry.
  if (!writeChecked(kRegDeviceConfig1, 0x00)) {
    delayMicroseconds(60);
    if (!writeChecked(kRegDeviceConfig1, 0x00)) return InitResult::NoAck;
  }
  uint8_t id[3];
  if (!bus_.readRegs(addr_, kRegDeviceId, id, 3)) return InitResult::NoAck;  // 0x0D, 0x0E, 0x0F
  device_id_ = id[0];
  if (id[1] != kManufacturerLsb || id[2] != kManufacturerMsb) return InitResult::BadManufacturer;
  if ((id[0] & 0x03) != kDeviceVer40mT) return InitResult::BadDeviceId;  // x2 (±133 mT) part fitted?

  // Standby while configuring.
  if (!writeChecked(kRegDeviceConfig2, kLowNoise)) return InitResult::WriteFailed;
  if (!writeChecked(kRegSensorConfig2, kRange40mT)) return InitResult::WriteFailed;
  if (!writeChecked(kRegSensorConfig1, kMagChXYZ)) return InitResult::WriteFailed;
  if (!writeChecked(kRegTConfig, 0x00)) return InitResult::WriteFailed;   // temp channel off
  if (!writeChecked(kRegIntConfig1, 0x00)) return InitResult::WriteFailed;
  if (!writeChecked(kRegConvStatus, kStatusPor)) return InitResult::WriteFailed;  // clear POR (W1C)
  // Read mode last: from here on, plain reads return XYZ + CONV_STATUS.
  if (!writeChecked(kRegDeviceConfig1, kDevCfg1Run)) return InitResult::WriteFailed;
  if (!writeChecked(kRegDeviceConfig2, kLowNoise | kOpContinuous)) return InitResult::WriteFailed;
  return InitResult::Ok;
}

bool Tmag5273::read(Sample& s) {
  uint8_t b[7];
  if (!bus_.read(addr_, b, sizeof(b))) return false;
  bool sat = false;
  for (int i = 0; i < 3; ++i) {
    const int16_t v = static_cast<int16_t>(static_cast<uint16_t>(b[2 * i]) << 8 | b[2 * i + 1]);
    s.xyz[i] = v;
    if (v >= INT16_MAX - kSatMargin || v <= INT16_MIN + kSatMargin) sat = true;
  }
  s.conv_status = b[6];
  s.saturated = sat;
  s.por = (b[6] & kStatusPor) != 0;
  s.diag_fail = (b[6] & kStatusDiag) != 0;
  const int set_count = b[6] >> 5;
  // SET_COUNT is a 3-bit rolling count, so "unchanged" only means "stuck" when the number of
  // conversions per sample period is not a multiple of 8 (200 Hz: 4 -> fine; 100 Hz: 8 -> not).
  // The sampler decides whether to trust it; it is a hint, never an error by itself.
  // VERIFY: SET_COUNT semantics in continuous mode (Table 6-1) on real parts.
  s.stale = (set_count == last_set_count_);
  last_set_count_ = set_count;
  return true;
}

bool Tmag5273::sleep() {
  // DEVICE_CONFIG_2 is written in place; the read mode does not matter for writes.
  return writeChecked(kRegDeviceConfig2, kLowNoise | kOpSleep);
}
