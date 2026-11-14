#include "max17048.h"

namespace {
constexpr uint8_t kRegVcell = 0x02;
constexpr uint8_t kRegSoc = 0x04;
constexpr uint8_t kRegVersion = 0x08;
constexpr uint8_t kRegConfig = 0x0C;
constexpr uint8_t kRegValrt = 0x14;
constexpr uint8_t kRegCrate = 0x16;
constexpr uint8_t kRegStatus = 0x1A;

constexpr uint16_t kConfigAlrt = 1u << 5;
constexpr uint16_t kStatusRi = 1u << 8;
constexpr uint16_t kStatusAlertBits = 0x3E00;   // SC, HD, VR, VL, VH (bits 13..9)
}  // namespace

bool Max17048::begin(uint8_t empty_alert_pct, uint16_t vmin_mv, uint16_t vmax_mv) {
  if (!bus_.readReg16(addr_, kRegVersion, version_)) return false;
  if ((version_ & 0xFFF0) != 0x0010) return false;   // datasheet: VERSION = 0x001_
  uint16_t cfg;
  if (!bus_.readReg16(addr_, kRegConfig, cfg)) return false;
  if (empty_alert_pct < 1) empty_alert_pct = 1;
  if (empty_alert_pct > 32) empty_alert_pct = 32;
  const uint16_t athd = static_cast<uint16_t>(32 - empty_alert_pct) & 0x1F;
  cfg = static_cast<uint16_t>((cfg & 0xFF00) | athd);   // SLEEP = 0, ALSC = 0, ALRT = 0
  if (!bus_.writeReg16(addr_, kRegConfig, cfg)) return false;
  const uint16_t vmin = static_cast<uint16_t>(vmin_mv / 20), vmax = static_cast<uint16_t>(vmax_mv / 20);
  if (!bus_.writeReg16(addr_, kRegValrt, static_cast<uint16_t>((vmin & 0xFF) << 8 | (vmax & 0xFF)))) return false;
  uint16_t st;
  if (!bus_.readReg16(addr_, kRegStatus, st)) return false;
  return bus_.writeReg16(addr_, kRegStatus, static_cast<uint16_t>(st & ~(kStatusRi | kStatusAlertBits)));
}

bool Max17048::read(Reading& r) {
  uint16_t v, soc, crate, st, cfg;
  if (!bus_.readReg16(addr_, kRegVcell, v)) return false;
  if (!bus_.readReg16(addr_, kRegSoc, soc)) return false;
  if (!bus_.readReg16(addr_, kRegCrate, crate)) return false;
  if (!bus_.readReg16(addr_, kRegStatus, st)) return false;
  if (!bus_.readReg16(addr_, kRegConfig, cfg)) return false;
  r.vcell_mv = static_cast<uint16_t>((static_cast<uint32_t>(v) * 78125u) / 1000000u);  // 78.125 µV/LSB
  r.soc_pct = soc / 256.0f;
  r.crate_pct_per_hr = static_cast<int16_t>(crate) * 0.208f;
  r.status_hi = static_cast<uint8_t>(st >> 8);
  r.alert = (cfg & kConfigAlrt) != 0;
  return true;
}

bool Max17048::clearAlert() {
  uint16_t st, cfg;
  if (!bus_.readReg16(addr_, kRegStatus, st)) return false;
  if (!bus_.writeReg16(addr_, kRegStatus, static_cast<uint16_t>(st & ~kStatusAlertBits))) return false;
  if (!bus_.readReg16(addr_, kRegConfig, cfg)) return false;
  return bus_.writeReg16(addr_, kRegConfig, static_cast<uint16_t>(cfg & ~kConfigAlrt));
}
