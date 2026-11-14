#include "i2c_bus.h"

namespace {
// TwoWire::endTransmission() result codes (Arduino convention, also used by arduino-esp32).
constexpr uint8_t kOk = 0;
constexpr uint8_t kNackAddr = 2;
constexpr uint8_t kNackData = 3;
// Half period of the recovery clock: 5 µs -> 100 kHz, slow enough for any slave.
constexpr uint32_t kRecoveryHalfPeriodUs = 5;
}  // namespace

bool I2cBus::begin() {
  if (!wire_.begin(sda_, scl_, hz_)) return false;
  wire_.setTimeOut(timeout_ms_);
  return true;
}

void I2cBus::end() { wire_.end(); }

bool I2cBus::finish(uint8_t r) {
  if (r == kOk) return true;
  if (r == kNackAddr || r == kNackData) {
    ++nacks_;
  } else {
    ++errors_;  // 1 = too long, 4 = other, 5 = timeout
  }
  return false;
}

bool I2cBus::writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  wire_.beginTransmission(addr);
  wire_.write(reg);
  wire_.write(value);
  return finish(wire_.endTransmission(true));
}

bool I2cBus::writeReg16(uint8_t addr, uint8_t reg, uint16_t v) {
  wire_.beginTransmission(addr);
  wire_.write(reg);
  wire_.write(static_cast<uint8_t>(v >> 8));
  wire_.write(static_cast<uint8_t>(v & 0xFF));
  return finish(wire_.endTransmission(true));
}

bool I2cBus::readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
  wire_.beginTransmission(addr);
  wire_.write(reg);
  if (!finish(wire_.endTransmission(false))) return false;  // repeated START follows
  return read(addr, buf, len);
}

bool I2cBus::readReg16(uint8_t addr, uint8_t reg, uint16_t& v) {
  uint8_t b[2];
  if (!readRegs(addr, reg, b, 2)) return false;
  v = static_cast<uint16_t>(b[0] << 8 | b[1]);
  return true;
}

bool I2cBus::write(uint8_t addr, const uint8_t* buf, size_t len) {
  wire_.beginTransmission(addr);
  if (len) wire_.write(buf, len);
  return finish(wire_.endTransmission(true));
}

bool I2cBus::read(uint8_t addr, uint8_t* buf, size_t len) {
  const size_t got = wire_.requestFrom(addr, len, true);
  if (got != len) {
    ++errors_;
    while (wire_.available()) wire_.read();
    return false;
  }
  for (size_t i = 0; i < len; ++i) buf[i] = static_cast<uint8_t>(wire_.read());
  return true;
}

bool I2cBus::probe(uint8_t addr) {
  wire_.beginTransmission(addr);
  return wire_.endTransmission(true) == kOk;  // a probe NACK is not an error
}

bool I2cBus::sdaHeldLow() { return digitalRead(sda_) == LOW; }

bool I2cBus::recover() {
  ++recoveries_;
  wire_.end();
  // Drive SCL as open drain; SDA as input (pull-ups are external, 3.3 k).
  pinMode(sda_, INPUT);
  pinMode(scl_, OUTPUT_OPEN_DRAIN);
  digitalWrite(scl_, HIGH);
  delayMicroseconds(kRecoveryHalfPeriodUs);
  // Up to 9 clocks: a slave in the middle of sending a byte releases SDA after at most 8 data
  // bits + the ACK slot.
  for (int i = 0; i < 9 && digitalRead(sda_) == LOW; ++i) {
    digitalWrite(scl_, LOW);
    delayMicroseconds(kRecoveryHalfPeriodUs);
    digitalWrite(scl_, HIGH);
    delayMicroseconds(kRecoveryHalfPeriodUs);
  }
  // STOP: SDA low -> high while SCL is high.
  pinMode(sda_, OUTPUT_OPEN_DRAIN);
  digitalWrite(sda_, LOW);
  delayMicroseconds(kRecoveryHalfPeriodUs);
  digitalWrite(scl_, HIGH);
  delayMicroseconds(kRecoveryHalfPeriodUs);
  digitalWrite(sda_, HIGH);
  delayMicroseconds(kRecoveryHalfPeriodUs);
  pinMode(sda_, INPUT);
  const bool free_bus = digitalRead(sda_) == HIGH;
  begin();
  return free_bus;
}
