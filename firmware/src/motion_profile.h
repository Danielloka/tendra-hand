// Trapezoidal motion profile for a stepper: accelerate, cruise, decelerate, one step at a time.
//
// Smoothness comes from never changing speed abruptly: after every step the speed changes by at
// most what the acceleration limit allows (v_next^2 = v^2 +/- 2*a*1 step). Braking starts when the
// remaining distance equals the stopping distance v^2 / (2a). A new target can arrive at any time,
// even mid-move in the opposite direction: the motor brakes, reverses and continues.
//
// Pure C++ with no Arduino dependencies, so it can be unit-tested on a PC.
#pragma once

#include <math.h>
#include <stdint.h>

class MotionProfile {
 public:
  void configure(float max_speed, float accel) {
    max_speed_ = max_speed;
    accel_ = accel;
  }
  void setTarget(long target) { target_ = target; }
  void setPosition(long position) {
    position_ = target_ = position;
    speed_ = 0.0f;
  }
  // Brake to a halt with the configured deceleration.
  void stop() {
    long stop_steps = static_cast<long>(speed_ * speed_ / (2.0f * accel_));
    target_ = position_ + (speed_ > 0 ? stop_steps : -stop_steps);
  }

  long position() const { return position_; }
  long target() const { return target_; }
  float speed() const { return speed_; }
  bool isMoving() const { return speed_ != 0.0f || position_ != target_; }

  // Returns +1 or -1 when a step must be taken now (position is already updated), else 0.
  int update(uint32_t now_us) {
    if (!isMoving()) return 0;
    if (speed_ != 0.0f && now_us - last_step_us_ < static_cast<uint32_t>(1e6f / fabsf(speed_))) {
      return 0;
    }
    speed_ = nextSpeed();
    if (speed_ == 0.0f) return 0;  // arrived
    const int dir = speed_ > 0.0f ? 1 : -1;
    position_ += dir;
    last_step_us_ = now_us;
    return dir;
  }

 private:
  float nextSpeed() const {
    const long remaining = target_ - position_;
    const int wanted = (remaining > 0) - (remaining < 0);
    const float v_min = sqrtf(2.0f * accel_);  // speed after one step from standstill
    if (speed_ == 0.0f) return wanted * v_min;

    const int dir = speed_ > 0.0f ? 1 : -1;
    const float stopping = speed_ * speed_ / (2.0f * accel_);
    const bool brake = dir != wanted || fabsf(static_cast<float>(remaining)) <= stopping;
    if (brake) {
      if (fabsf(speed_) <= v_min) return wanted * v_min;  // slow enough to stop or reverse
      const float v2 = speed_ * speed_ - 2.0f * accel_;
      return dir * sqrtf(v2 > v_min * v_min ? v2 : v_min * v_min);
    }
    const float v = sqrtf(speed_ * speed_ + 2.0f * accel_);
    return dir * (v < max_speed_ ? v : max_speed_);
  }

  long position_ = 0;
  long target_ = 0;
  float speed_ = 0.0f;  // signed, steps/s
  float max_speed_ = 800.0f;
  float accel_ = 1600.0f;
  uint32_t last_step_us_ = 0;
};
