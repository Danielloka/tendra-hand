// Feetech SCS serial-bus protocol (SCS0009 and other "SCSCL" servos): packets, checksum, parser.
//
// Written from the protocol description (Feetech memory table + the SCSCL/SCS classes of the
// official FTServo_Arduino library, read for reference only; no code copied).
//
// Instruction packet:  FF FF  ID  LEN  INSTR  P1 .. Pn  CHK      LEN = n + 2
// Status (reply):      FF FF  ID  LEN  ERROR  P1 .. Pn  CHK      LEN = n + 2
// CHK = ~(ID + LEN + INSTR/ERROR + P1 + .. + Pn) & 0xFF
//
// SCS servos are BIG-endian: a 16-bit register at address a holds the HIGH byte at a and the LOW
// byte at a+1 (the Feetech header still calls a "_L"). STS/SMS servos are little-endian; this file
// is for SCS only.
//
// Pure C++ with no Arduino dependencies, so it is unit-tested on the PC (test/host/).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace scs {

constexpr uint8_t kBroadcastId = 0xFE;
constexpr uint8_t kMaxId = 0xFD;

// Instructions.
constexpr uint8_t kInstPing = 0x01;
constexpr uint8_t kInstRead = 0x02;
constexpr uint8_t kInstWrite = 0x03;
constexpr uint8_t kInstSyncWrite = 0x83;

// SCSCL memory table (SCS0009). EPROM registers are not written by this firmware.
namespace reg {
constexpr uint8_t kVersion = 3;            // 2 bytes, read-only
constexpr uint8_t kId = 5;                 // EPROM
constexpr uint8_t kBaudRate = 6;           // EPROM
constexpr uint8_t kMinAngleLimit = 9;      // EPROM, 2 bytes
constexpr uint8_t kMaxAngleLimit = 11;     // EPROM, 2 bytes
constexpr uint8_t kTorqueEnable = 40;      // 1 byte: 0 = off (limp), 1 = on
constexpr uint8_t kGoalPosition = 42;      // 2 bytes, ticks
constexpr uint8_t kGoalTime = 44;          // 2 bytes, ms (0 = use goal speed)
constexpr uint8_t kGoalSpeed = 46;         // 2 bytes (0 = maximum speed! never send 0)
constexpr uint8_t kLock = 48;              // EPROM write lock
constexpr uint8_t kPresentPosition = 56;   // 2 bytes, ticks
constexpr uint8_t kPresentSpeed = 58;      // 2 bytes, bit 15 = sign
constexpr uint8_t kPresentLoad = 60;       // 2 bytes, bit 10 = sign, 0..1000 (0.1 % of max)
constexpr uint8_t kPresentVoltage = 62;    // 1 byte, 0.1 V
constexpr uint8_t kPresentTemperature = 63;// 1 byte, deg C
constexpr uint8_t kMoving = 66;            // 1 byte
}  // namespace reg

// Feedback block read in one go: registers 56..63 (position, speed, load, voltage, temperature).
constexpr uint8_t kFeedbackStart = reg::kPresentPosition;
constexpr uint8_t kFeedbackLen = 8;

// SCS0009: 1024 positions over 300 degrees (0.293 deg per tick), centre 512.
constexpr int kTicksMin = 0;
constexpr int kTicksMax = 1023;
constexpr int kTicksCenter = 512;
constexpr float kTicksPerRad = 1024.0f / (300.0f * 0.017453292519943295f);  // ~195.57

// Status error bits (from the Feetech docs; the exact meaning varies slightly per model).
constexpr uint8_t kErrVoltage = 0x01;
constexpr uint8_t kErrAngle = 0x02;
constexpr uint8_t kErrOverheat = 0x04;
constexpr uint8_t kErrOverload = 0x20;

// Largest packet we build or accept (sync write of 20 servos x 7 bytes + overhead fits).
constexpr size_t kMaxPacket = 200;

inline uint8_t checksum(const uint8_t* body, size_t n) {
  // body = ID, LEN, INSTR/ERROR, params (everything between the FF FF header and the checksum).
  uint32_t sum = 0;
  for (size_t i = 0; i < n; ++i) sum += body[i];
  return static_cast<uint8_t>(~sum);
}

inline void put16(uint8_t* out, uint16_t v) {  // big-endian (SCS)
  out[0] = static_cast<uint8_t>(v >> 8);
  out[1] = static_cast<uint8_t>(v & 0xFF);
}
inline uint16_t get16(const uint8_t* in) { return static_cast<uint16_t>((in[0] << 8) | in[1]); }

// Sign-magnitude decoding used by SCS feedback registers.
inline int16_t signMag(uint16_t raw, int sign_bit) {
  const uint16_t mag = raw & static_cast<uint16_t>((1u << sign_bit) - 1);
  return (raw & (1u << sign_bit)) ? -static_cast<int16_t>(mag) : static_cast<int16_t>(mag);
}

// Builds one instruction packet into `out` (capacity kMaxPacket). Returns its length, 0 on error.
inline size_t buildPacket(uint8_t* out, uint8_t id, uint8_t instr, const uint8_t* params,
                          size_t n_params) {
  if (n_params + 6 > kMaxPacket) return 0;
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = id;
  out[3] = static_cast<uint8_t>(n_params + 2);
  out[4] = instr;
  for (size_t i = 0; i < n_params; ++i) out[5 + i] = params[i];
  out[5 + n_params] = checksum(out + 2, n_params + 3);
  return n_params + 6;
}

inline size_t buildPing(uint8_t* out, uint8_t id) { return buildPacket(out, id, kInstPing, nullptr, 0); }

inline size_t buildRead(uint8_t* out, uint8_t id, uint8_t addr, uint8_t len) {
  const uint8_t p[2] = {addr, len};
  return buildPacket(out, id, kInstRead, p, 2);
}

inline size_t buildWrite(uint8_t* out, uint8_t id, uint8_t addr, const uint8_t* data, size_t len) {
  uint8_t p[kMaxPacket];
  if (len + 1 > sizeof(p)) return 0;
  p[0] = addr;
  for (size_t i = 0; i < len; ++i) p[1 + i] = data[i];
  return buildPacket(out, id, kInstWrite, p, len + 1);
}

// Goal position + goal time + goal speed, the 6-byte block at register 42.
inline void encodeGoal(uint8_t* out6, uint16_t ticks, uint16_t time_ms, uint16_t speed) {
  put16(out6, ticks);
  put16(out6 + 2, time_ms);
  put16(out6 + 4, speed);
}

// SYNC WRITE to broadcast: FF FF FE LEN 83 addr len [id d1..dlen] x n CHK,  LEN = (len+1)*n + 4.
// `data` holds n blocks of `len` bytes, in the same order as `ids`.
inline size_t buildSyncWrite(uint8_t* out, uint8_t addr, uint8_t len, const uint8_t* ids,
                             const uint8_t* data, size_t n) {
  uint8_t p[kMaxPacket];
  const size_t n_params = 2 + n * (len + 1u);
  if (n_params + 6 > kMaxPacket) return 0;
  p[0] = addr;
  p[1] = len;
  size_t k = 2;
  for (size_t s = 0; s < n; ++s) {
    p[k++] = ids[s];
    for (size_t i = 0; i < len; ++i) p[k++] = data[s * len + i];
  }
  return buildPacket(out, kBroadcastId, kInstSyncWrite, p, n_params);
}

// Incremental status-packet parser: feed bytes one at a time; returns true when a complete frame
// with a valid checksum is available in id()/error()/params(). Resynchronises on garbage.
class Parser {
 public:
  void reset() { state_ = 0; }

  bool feed(uint8_t b) {
    switch (state_) {
      case 0:  // want first FF
        if (b == 0xFF) state_ = 1;
        return false;
      case 1:  // want second FF
        state_ = (b == 0xFF) ? 2 : 0;
        return false;
      case 2:  // ID (a third FF is tolerated as part of the header)
        if (b == 0xFF) return false;
        frame_[0] = b;
        state_ = 3;
        return false;
      case 3:  // LEN
        if (b < 2 || b + 4u > kMaxPacket) {
          state_ = (b == 0xFF) ? 1 : 0;
          ++bad_frames_;
          return false;
        }
        frame_[1] = b;
        got_ = 0;
        state_ = 4;
        return false;
      default: {  // ERROR/INSTR, params, checksum: LEN bytes in total
        frame_[2 + got_++] = b;
        if (got_ < frame_[1]) return false;
        state_ = 0;
        const size_t body = 1u + frame_[1];  // ID, LEN, ERROR, params (without checksum)
        if (checksum(frame_, body) != frame_[body]) {
          ++bad_frames_;
          return false;
        }
        return true;
      }
    }
  }

  uint8_t id() const { return frame_[0]; }
  uint8_t error() const { return frame_[2]; }  // INSTR when the frame is an echoed request
  const uint8_t* params() const { return frame_ + 3; }
  size_t paramCount() const { return frame_[1] - 2u; }
  uint32_t badFrames() const { return bad_frames_; }
  // Frame bytes without the FF FF header (ID, LEN, ERROR, params, CHK).
  const uint8_t* raw() const { return frame_; }
  size_t rawLen() const { return frame_[1] + 2u; }

 private:
  uint8_t frame_[kMaxPacket];
  uint8_t state_ = 0;
  size_t got_ = 0;
  uint32_t bad_frames_ = 0;
};

}  // namespace scs
