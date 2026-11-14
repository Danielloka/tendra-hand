// I2cBus: thin, non-allocating wrapper around one TwoWire controller.
//  - every transaction has a timeout (TwoWire::setTimeOut) and returns a bool, never blocks forever
//  - error counters (NACK, timeout/other) for diagnostics
//  - bus recovery: if SDA is held low by a slave stuck mid-byte, clock SCL up to 9 times until
//    SDA is released, then generate a STOP, and re-initialise the controller.
//
// Not thread-safe: each bus has exactly one owner task (I2C0 = sampler, I2C1 = health task).
#pragma once

#include <Arduino.h>
#include <Wire.h>

class I2cBus {
 public:
  I2cBus(TwoWire& wire, int sda, int scl, uint32_t hz, uint16_t timeout_ms)
      : wire_(wire), sda_(sda), scl_(scl), hz_(hz), timeout_ms_(timeout_ms) {}

  bool begin();
  void end();

  // Register access (8-bit register address, repeated START between write and read).
  bool writeReg(uint8_t addr, uint8_t reg, uint8_t value);
  bool writeReg16(uint8_t addr, uint8_t reg, uint16_t value_msb_first);
  bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);
  bool readReg16(uint8_t addr, uint8_t reg, uint16_t& value_msb_first);

  // Raw transfers.
  bool write(uint8_t addr, const uint8_t* buf, size_t len);
  bool read(uint8_t addr, uint8_t* buf, size_t len);   // START addr+R ... STOP (no register)
  bool probe(uint8_t addr);                             // zero-length write, true on ACK

  // Bus recovery. Returns true if SDA is high (bus free) afterwards. Leaves the controller
  // re-initialised in either case.
  bool recover();
  bool sdaHeldLow();   // only meaningful while the controller is detached (used by recover())

  uint32_t nackCount() const { return nacks_; }
  uint32_t errorCount() const { return errors_; }
  uint32_t recoveryCount() const { return recoveries_; }
  uint32_t hz() const { return hz_; }

 private:
  bool finish(uint8_t end_result);
  TwoWire& wire_;
  int sda_, scl_;
  uint32_t hz_;
  uint16_t timeout_ms_;
  uint32_t nacks_ = 0;
  uint32_t errors_ = 0;
  uint32_t recoveries_ = 0;
};
