#include "uln2003_stepper.h"

#include <Arduino.h>

namespace {
// Half-step sequence, bits = IN1..IN4 (MSB first).
constexpr uint8_t kHalfStep[8] = {0b1000, 0b1100, 0b0100, 0b0110,
                                  0b0010, 0b0011, 0b0001, 0b1001};
}  // namespace

Uln2003Stepper::Uln2003Stepper(const JointConfig& cfg)
    : cfg_(cfg),
      max_vel_rad_s_(kDefaultMaxSpeed / kDefaultStepsPerRad),
      max_acc_rad_s2_(kDefaultAccel / kDefaultStepsPerRad) {}

void Uln2003Stepper::begin() {
  for (uint8_t pin : cfg_.pins) pinMode(pin, OUTPUT);
  release();
  applyLimits();
}

void Uln2003Stepper::update(uint32_t now_us) {
  const uint32_t now_ms = now_us / 1000;
  if (profile_.update(now_us) != 0) {
    writePhase();
    last_active_ms_ = now_ms;
  } else if (energized_ && !profile_.isMoving() && now_ms - last_active_ms_ > kIdleReleaseMs) {
    release();
  }
}

void Uln2003Stepper::setTarget(float rad) {
  if (rad < cfg_.min_rad) rad = cfg_.min_rad;
  if (rad > cfg_.max_rad) rad = cfg_.max_rad;
  profile_.setTarget(toSteps(rad));
}

float Uln2003Stepper::target() const {
  return (cfg_.invert ? -1.0f : 1.0f) * profile_.target() / steps_per_rad_;
}

float Uln2003Stepper::position() const {
  return (cfg_.invert ? -1.0f : 1.0f) * profile_.position() / steps_per_rad_;
}

void Uln2003Stepper::setZero() {
  phase_offset_ += profile_.position();
  profile_.setPosition(0);
}

void Uln2003Stepper::release() {
  for (uint8_t pin : cfg_.pins) digitalWrite(pin, LOW);
  energized_ = false;
}

void Uln2003Stepper::setLimits(float max_vel_rad_s, float max_acc_rad_s2) {
  max_vel_rad_s_ = max_vel_rad_s;
  max_acc_rad_s2_ = max_acc_rad_s2;
  applyLimits();
}

void Uln2003Stepper::setScale(float steps_per_rad) {
  // The motor does not move; only the steps <-> radians conversion changes.
  steps_per_rad_ = steps_per_rad;
  applyLimits();
}

void Uln2003Stepper::moveRaw(long steps) {
  if (steps > kMaxRawMove) steps = kMaxRawMove;
  if (steps < -kMaxRawMove) steps = -kMaxRawMove;
  profile_.setTarget(profile_.target() + (cfg_.invert ? -steps : steps));  // positive = closing
}

void Uln2003Stepper::applyLimits() {
  const float speed = max_vel_rad_s_ * steps_per_rad_;
  profile_.configure(speed < kMotorMaxSpeed ? speed : kMotorMaxSpeed,
                     max_acc_rad_s2_ * steps_per_rad_);
}

void Uln2003Stepper::writePhase() {
  const uint8_t bits = kHalfStep[(profile_.position() + phase_offset_) & 7];
  for (int i = 0; i < 4; ++i) digitalWrite(cfg_.pins[i], (bits >> (3 - i)) & 1);
  energized_ = true;
}

long Uln2003Stepper::toSteps(float rad) const {
  return lroundf((cfg_.invert ? -rad : rad) * steps_per_rad_);
}
