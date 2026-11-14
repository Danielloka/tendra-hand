#include "icm42688.h"

namespace {
constexpr uint8_t kRegDeviceConfig = 0x11;
constexpr uint8_t kRegTempData1 = 0x1D;
constexpr uint8_t kRegPwrMgmt0 = 0x4E;
constexpr uint8_t kRegGyroConfig0 = 0x4F;
constexpr uint8_t kRegAccelConfig0 = 0x50;
constexpr uint8_t kRegWhoAmI = 0x75;
constexpr uint8_t kRegBankSel = 0x76;

constexpr uint8_t kWhoAmI = 0x47;
constexpr uint8_t kSoftReset = 0x01;
constexpr uint8_t kGyro1000dps1kHz = (0b001 << 5) | 0b0110;   // 0x26
constexpr uint8_t kAccel4g1kHz = (0b010 << 5) | 0b0110;       // 0x46
constexpr uint8_t kPwrGyroLnAccelLn = (0b11 << 2) | 0b11;     // 0x0F, temperature on
}  // namespace

Icm42688::InitResult Icm42688::init() {
  if (!bus_.writeReg(addr_, kRegBankSel, 0)) return InitResult::NoAck;
  if (!bus_.readRegs(addr_, kRegWhoAmI, &who_, 1)) return InitResult::NoAck;
  if (who_ != kWhoAmI) return InitResult::BadWhoAmI;
  if (!bus_.writeReg(addr_, kRegDeviceConfig, kSoftReset)) return InitResult::WriteFailed;
  delayMicroseconds(1100);   // datasheet: wait 1 ms after soft reset before any access
  // Soft reset leaves bank 0 selected and I2C enabled; configure full scale + ODR while the
  // sensors are still off, then switch them on.
  if (!bus_.writeReg(addr_, kRegGyroConfig0, kGyro1000dps1kHz)) return InitResult::WriteFailed;
  if (!bus_.writeReg(addr_, kRegAccelConfig0, kAccel4g1kHz)) return InitResult::WriteFailed;
  if (!bus_.writeReg(addr_, kRegPwrMgmt0, kPwrGyroLnAccelLn)) return InitResult::WriteFailed;
  delayMicroseconds(250);    // no register writes for 200 µs after leaving OFF
  // Readback check of the configuration (cheap, catches a half-applied init).
  uint8_t cfg[3];
  if (!bus_.readRegs(addr_, kRegPwrMgmt0, cfg, 3)) return InitResult::NoAck;
  if (cfg[0] != kPwrGyroLnAccelLn || cfg[1] != kGyro1000dps1kHz || cfg[2] != kAccel4g1kHz)
    return InitResult::WriteFailed;
  // Data is invalid (0x8000) until the first samples are ready (gyro start-up ~30 ms): the
  // sampler passes it through; the host ignores 0x8000.
  return InitResult::Ok;
}

bool Icm42688::read(Sample& s) {
  uint8_t b[14];
  if (!bus_.readRegs(addr_, kRegTempData1, b, sizeof(b))) return false;
  auto be16 = [&](int i) { return static_cast<int16_t>(static_cast<uint16_t>(b[i]) << 8 | b[i + 1]); };
  s.temp_raw = be16(0);
  for (int i = 0; i < 3; ++i) {
    s.accel[i] = be16(2 + 2 * i);
    s.gyro[i] = be16(8 + 2 * i);
  }
  return true;
}

bool Icm42688::powerOff() { return bus_.writeReg(addr_, kRegPwrMgmt0, 0x00); }
