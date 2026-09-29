// Feetech SCS0009 smart servo on the SCS bus (v1 hand), implements MotorDriver.
//
// The servo has its own position controller and an absolute position sensor (0..1023 ticks over
// 300 deg), so there is no homing: it reports where it is. The firmware still ramps the goal with
// the same trapezoidal MotionProfile as the steppers (1 tick at a time) and streams it to the bus,
// so motion is smooth and speed-limited no matter how far a target jumps.
//
// Safety: begin() switches torque OFF and only reads. Torque comes on with the first target, and
// the goal is first set to the measured position, so the joint never jumps. A servo that has never
// answered cannot be switched on (its position is unknown).
//
// Bus traffic is not done in update(): the owner calls poll() (feedback, round robin) and sends
// pending goals for all servos in one SYNC WRITE (see main.cpp).
#pragma once

#include "config_v1.h"
#include "motion_profile.h"
#include "motor_driver.h"
#include "scs_bus.h"
#include "servo_calibration.h"

class Scs0009Servo : public MotorDriver {
 public:
  Scs0009Servo(const ServoJointConfig& cfg, ScsBus& bus);

  void begin() override;
  void update(uint32_t now_us) override;

  void setTarget(float rad) override;
  float target() const override { return cal_.ticksToJoint(profile_.target()); }
  float position() const override;
  bool isMoving() const override { return profile_.isMoving(); }

  void setZero() override;
  void stop() override { profile_.stop(); }
  void release() override;  // torque off: the joint goes limp

  void setLimits(float max_vel_rad_s, float max_acc_rad_s2) override;
  void setScale(float ticks_per_rad) override;
  float scale() const override { return cal_.native_per_rad; }
  void moveRaw(long ticks) override;

  bool online() const override { return fb_valid_ && missed_ < kMaxMissedReplies; }
  bool feedback(MotorFeedback& out) const override;

  // ----- bus side, used by main.cpp -----
  uint8_t id() const { return cfg_.servo_id; }
  bool torqueOn() const { return torque_on_; }
  long zeroTicks() const { return cal_.zero_ticks; }
  // Reads position/load/voltage/temperature (blocking, well under 1 ms when the servo answers).
  void poll(uint32_t now_ms);
  // True if it is worth polling now (offline servos are retried slowly).
  bool pollDue(uint32_t now_ms) const;
  bool goalPending() const { return goal_pending_; }
  uint16_t goalTicks() const { return static_cast<uint16_t>(profile_.position()); }
  void goalSent() { goal_pending_ = false; }

 private:
  bool ensureTorque();
  void applyLimits();

  const ServoJointConfig& cfg_;
  ScsBus& bus_;
  ServoCalibration cal_;
  MotionProfile profile_;  // in absolute servo ticks; position() = goal being streamed
  float max_vel_rad_s_ = kDefaultMaxVelRad;
  float max_acc_rad_s2_ = kDefaultAccRad;
  ScsFeedback fb_;
  bool fb_valid_ = false;
  uint8_t missed_ = 0;
  uint32_t last_poll_ms_ = 0;
  bool torque_on_ = false;
  bool goal_pending_ = false;
};
