#include "scs_bus.h"

#include <string.h>

bool ScsBus::transact(const uint8_t* pkt, size_t n, uint8_t id) {
  if (n == 0) return false;
  port_.clearInput();
  port_.write(pkt, n);
  if (id == scs::kBroadcastId) return true;  // broadcast: servos never reply

  parser_.reset();
  bool first = true;
  const uint32_t start = port_.micros();
  while (port_.micros() - start < timeout_us_) {
    const int c = port_.read();
    if (c < 0) continue;
    if (!parser_.feed(static_cast<uint8_t>(c))) continue;
    // Skip our own echo: the first frame, byte-for-byte equal to what we sent.
    const bool echo = first && parser_.rawLen() == n - 2 && memcmp(parser_.raw(), pkt + 2, n - 2) == 0;
    first = false;
    if (echo || parser_.id() != id) continue;
    last_error_ = parser_.error();
    return true;
  }
  ++timeouts_;
  return false;
}

bool ScsBus::ping(uint8_t id) { return transact(tx_, scs::buildPing(tx_, id), id); }

bool ScsBus::read(uint8_t id, uint8_t addr, uint8_t* out, uint8_t len) {
  if (id == scs::kBroadcastId) return false;
  if (!transact(tx_, scs::buildRead(tx_, id, addr, len), id)) return false;
  if (parser_.paramCount() != len) return false;
  memcpy(out, parser_.params(), len);
  return true;
}

bool ScsBus::write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len) {
  return transact(tx_, scs::buildWrite(tx_, id, addr, data, len), id);
}

bool ScsBus::writeGoal(uint8_t id, uint16_t ticks, uint16_t time_ms, uint16_t speed) {
  uint8_t d[6];
  scs::encodeGoal(d, ticks, time_ms, speed);
  return write(id, scs::reg::kGoalPosition, d, 6);
}

bool ScsBus::readFeedback(uint8_t id, ScsFeedback& fb) {
  uint8_t d[scs::kFeedbackLen];
  if (!read(id, scs::kFeedbackStart, d, scs::kFeedbackLen)) return false;
  fb.position = scs::get16(d);
  fb.speed = scs::signMag(scs::get16(d + 2), 15);
  fb.load = scs::signMag(scs::get16(d + 4), 10);
  fb.voltage_dv = d[6];
  fb.temperature_c = d[7];
  fb.error = last_error_;
  return true;
}

bool ScsBus::syncWriteGoals(const uint8_t* ids, const uint16_t* ticks, size_t n, uint16_t time_ms,
                            uint16_t speed) {
  uint8_t data[scs::kMaxPacket];
  if (n == 0) return true;
  if (n * 6 > sizeof(data)) return false;
  for (size_t i = 0; i < n; ++i) scs::encodeGoal(data + 6 * i, ticks[i], time_ms, speed);
  return transact(tx_, scs::buildSyncWrite(tx_, scs::reg::kGoalPosition, 6, ids, data, n),
                  scs::kBroadcastId);
}
