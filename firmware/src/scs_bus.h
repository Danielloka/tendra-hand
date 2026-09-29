// ScsBus: talks to Feetech SCS servos on one half-duplex TTL bus (ESP32 UART -> FE-URT-1 -> servos).
//
// Request/response: send an instruction packet, then wait (with a timeout) for the status packet
// of that servo. The FE-URT-1 switches the bus direction by itself, but on such adapters the MCU
// can hear its own transmission ("echo"); an exact copy of the request is skipped automatically.
//
// The byte stream is abstract (ScsPort) so the bus logic can be unit-tested on the PC with a fake
// servo. On the ESP32, ScsUartPort (scs_uart_port.h) wraps a HardwareSerial.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "scs_protocol.h"

class ScsPort {
 public:
  virtual ~ScsPort() = default;
  virtual void write(const uint8_t* data, size_t n) = 0;  // returns when the bytes are sent
  virtual int read() = 0;                                 // next received byte, or -1 if none
  virtual void clearInput() = 0;                          // drop any stale received bytes
  virtual uint32_t micros() = 0;
};

struct ScsFeedback {
  uint16_t position = 0;     // ticks
  int16_t speed = 0;         // raw, signed
  int16_t load = 0;          // signed, 0..1000 = 0..100 % of max torque
  uint8_t voltage_dv = 0;    // 0.1 V
  uint8_t temperature_c = 0;
  uint8_t error = 0;         // status error bits of the reply
};

class ScsBus {
 public:
  explicit ScsBus(ScsPort& port, uint32_t timeout_us = 3000) : port_(port), timeout_us_(timeout_us) {}

  void setTimeout(uint32_t us) { timeout_us_ = us; }

  bool ping(uint8_t id);
  // Reads `len` bytes starting at `addr` into `out`. False on timeout or bad reply.
  bool read(uint8_t id, uint8_t addr, uint8_t* out, uint8_t len);
  // Writes and waits for the servo's acknowledgement (none for the broadcast ID).
  bool write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len);
  bool writeByte(uint8_t id, uint8_t addr, uint8_t v) { return write(id, addr, &v, 1); }

  bool setTorque(uint8_t id, bool on) { return writeByte(id, scs::reg::kTorqueEnable, on ? 1 : 0); }
  bool writeGoal(uint8_t id, uint16_t ticks, uint16_t time_ms, uint16_t speed);
  bool readFeedback(uint8_t id, ScsFeedback& fb);

  // One broadcast packet sets goal position/time/speed of n servos (no reply).
  bool syncWriteGoals(const uint8_t* ids, const uint16_t* ticks, size_t n, uint16_t time_ms,
                      uint16_t speed);

  uint8_t lastError() const { return last_error_; }  // status error bits of the last reply
  uint32_t timeouts() const { return timeouts_; }
  uint32_t badFrames() const { return parser_.badFrames(); }

 private:
  // Sends `pkt`, then (if `id` is not broadcast) waits for that servo's status packet.
  bool transact(const uint8_t* pkt, size_t n, uint8_t id);

  ScsPort& port_;
  uint32_t timeout_us_;
  scs::Parser parser_;
  uint8_t tx_[scs::kMaxPacket];
  uint8_t last_error_ = 0;
  uint32_t timeouts_ = 0;
};
