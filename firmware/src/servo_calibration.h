// Joint angle <-> servo ticks, a per-joint linear calibration.
//
//   ticks = zero_ticks + sign * native_per_rad * q        sign = invert ? -1 : +1
//
// native_per_rad = ticks per JOINT radian = servo_per_joint * scs::kTicksPerRad, where
// servo_per_joint is how many servo radians one joint radian needs. For a tendon from a spool of
// radius r_spool to a joint drum of radius r_joint (same tendon length on both sides):
//   r_spool * d(theta_servo) = r_joint * d(q)   =>   servo_per_joint = r_joint / r_spool.
// Until measured, servo_per_joint = 1.0. zero_ticks is the servo reading when the joint is straight.
//
// Pure C++, unit-tested on the PC.
#pragma once

#include <math.h>
#include <stdint.h>

struct ServoCalibration {
  float native_per_rad;  // ticks per joint radian (> 0)
  long zero_ticks;       // servo position at q = 0
  bool invert;           // true if increasing ticks OPENS the joint

  float sign() const { return invert ? -1.0f : 1.0f; }

  // Unclamped; the caller clamps to the servo's tick range.
  long jointToTicks(float q_rad) const {
    return zero_ticks + lroundf(sign() * native_per_rad * q_rad);
  }
  float ticksToJoint(long ticks) const {
    return sign() * static_cast<float>(ticks - zero_ticks) / native_per_rad;
  }
};

inline float servoPerJointFromRadii(float joint_drum_radius_m, float spool_radius_m) {
  return joint_drum_radius_m / spool_radius_m;
}

inline long clampTicks(long t, long lo, long hi) { return t < lo ? lo : (t > hi ? hi : t); }
