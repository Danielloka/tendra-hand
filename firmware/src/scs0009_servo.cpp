#include "scs0009_servo.h"

Scs0009Servo::Scs0009Servo(const ServoJointConfig& cfg, ScsBus& bus)
    : cfg_(cfg),
      bus_(bus),
      cal_{cfg.servo_per_joint * scs::kTicksPerRad, cfg.zero_ticks, cfg.invert} {
  profile_.setPosition(cfg.zero_ticks);
}

void Scs0009Servo::begin() {
  // Never move on boot: torque explicitly off, then only read where the servo is.
  bus_.setTorque(cfg_.servo_id, false);
  torque_on_ = false;
  goal_pending_ = false;
  applyLimits();
  poll(0);
}

void Scs0009Servo::update(uint32_t now_us) {
  if (!torque_on_) return;
  if (profile_.update(now_us) != 0) goal_pending_ = true;
}

void Scs0009Servo::poll(uint32_t now_ms) {
  last_poll_ms_ = now_ms;
  ScsFeedback fb;
  if (!bus_.readFeedback(cfg_.servo_id, fb)) {
    if (missed_ < 255) ++missed_;
    return;
  }
  fb_ = fb;
  fb_valid_ = true;
  missed_ = 0;
  // Limp joint (moved by hand, by a tendon, ...): the ramp starts from where it really is.
  if (!torque_on_) profile_.setPosition(fb_.position);
}

bool Scs0009Servo::pollDue(uint32_t now_ms) const {
  return online() || now_ms - last_poll_ms_ >= kOfflineRetryMs;
}

float Scs0009Servo::position() const {
  return cal_.ticksToJoint(online() ? fb_.position : profile_.position());
}

bool Scs0009Servo::feedback(MotorFeedback& out) const {
  if (!online()) return false;
  out.position_rad = cal_.ticksToJoint(fb_.position);
  out.load_pct = cal_.sign() * fb_.load * 0.1f;  // positive = pulling in the closing direction
  out.temperature_c = fb_.temperature_c;
  out.voltage_v = fb_.voltage_dv * 0.1f;
  out.error = fb_.error;
  return true;
}

bool Scs0009Servo::ensureTorque() {
  if (torque_on_) return true;
  if (!online()) return false;  // position unknown: switching on could jump
  // Hold exactly where the servo is, then switch torque on.
  const uint16_t here = fb_.position;
  profile_.setPosition(here);
  bus_.writeGoal(cfg_.servo_id, here, kServoGoalTimeMs, kServoGoalSpeed);
  if (!bus_.setTorque(cfg_.servo_id, true)) return false;
  torque_on_ = true;
  goal_pending_ = false;
  return true;
}

void Scs0009Servo::setTarget(float rad) {
  if (rad < cfg_.min_rad) rad = cfg_.min_rad;
  if (rad > cfg_.max_rad) rad = cfg_.max_rad;
  if (!ensureTorque()) return;
  profile_.setTarget(clampTicks(cal_.jointToTicks(rad), scs::kTicksMin, scs::kTicksMax));
}

void Scs0009Servo::moveRaw(long ticks) {
  if (ticks > kMaxRawMove) ticks = kMaxRawMove;
  if (ticks < -kMaxRawMove) ticks = -kMaxRawMove;
  if (!ensureTorque()) return;
  const long t = profile_.target() + (cfg_.invert ? -ticks : ticks);  // positive = closing
  profile_.setTarget(clampTicks(t, scs::kTicksMin, scs::kTicksMax));
}

void Scs0009Servo::setZero() {
  cal_.zero_ticks = online() ? fb_.position : profile_.position();
}

void Scs0009Servo::release() {
  bus_.setTorque(cfg_.servo_id, false);
  torque_on_ = false;
  goal_pending_ = false;
  profile_.setPosition(online() ? fb_.position : profile_.position());
}

void Scs0009Servo::setLimits(float max_vel_rad_s, float max_acc_rad_s2) {
  max_vel_rad_s_ = max_vel_rad_s;
  max_acc_rad_s2_ = max_acc_rad_s2;
  applyLimits();
}

void Scs0009Servo::setScale(float ticks_per_rad) {
  // Nothing moves: the goal stays at the same servo ticks, only the conversion changes.
  cal_.native_per_rad = ticks_per_rad;
  applyLimits();
}

void Scs0009Servo::applyLimits() {
  const float speed = max_vel_rad_s_ * cal_.native_per_rad;
  profile_.configure(speed < kServoMaxTicksPerS ? speed : kServoMaxTicksPerS,
                     max_acc_rad_s2_ * cal_.native_per_rad);
}
