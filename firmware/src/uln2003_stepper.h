// 28BYJ-48 unipolar stepper on a ULN2003 board, driven in half-step mode.
//
// Half-stepping alternates one and two energized coils (8 phases per cycle): twice the resolution
// of full-stepping and noticeably smoother/quieter. Position is counted, not measured (open loop),
// so it is only correct if no steps are missed and the joint was straight at power-up.
#pragma once

#include "config.h"
#include "motion_profile.h"
#include "motor_driver.h"

class Uln2003Stepper : public MotorDriver {
 public:
  explicit Uln2003Stepper(const JointConfig& cfg);

  void begin() override;
  void update(uint32_t now_us) override;

  void setTarget(float rad) override;
  float target() const override;
  float position() const override;
  bool isMoving() const override { return profile_.isMoving(); }

  void setZero() override;
  void stop() override { profile_.stop(); }
  void release() override;

  void setLimits(float max_vel_rad_s, float max_acc_rad_s2) override;
  void setScale(float steps_per_rad) override;
  float scale() const override { return steps_per_rad_; }
  void moveRaw(long steps) override;

 private:
  void writePhase();
  void applyLimits();
  long toSteps(float rad) const;

  const JointConfig& cfg_;
  MotionProfile profile_;
  float steps_per_rad_ = kDefaultStepsPerRad;
  float max_vel_rad_s_;   // joint-space limits; converted to steps in applyLimits()
  float max_acc_rad_s2_;
  long phase_offset_ = 0;  // keeps the coil phase continuous when the step counter is re-zeroed
  bool energized_ = false;
  uint32_t last_active_ms_ = 0;
};
