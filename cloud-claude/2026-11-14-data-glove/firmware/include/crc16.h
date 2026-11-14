// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0x0000.
// Check value: crc16_ccitt("123456789") == 0x29B1. Header-only so the glove, the dongle and the
// host tests share exactly one implementation (the Python reader has a copy, tested against it).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace tg {

constexpr uint16_t kCrc16Init = 0xFFFF;

// Bitwise, no table: ~150 B packets at 200 Hz cost a few µs on the ESP32-S3, not worth 512 B of
// table in flash cache.
inline uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline uint16_t crc16_ccitt(const void* data, size_t len) {
  return crc16_ccitt_update(kCrc16Init, static_cast<const uint8_t*>(data), len);
}

}  // namespace tg
