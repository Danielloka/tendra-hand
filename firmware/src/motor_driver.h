// Hardware abstraction layer (HAL) for one joint actuator.
//
// Everything above this interface works in joint radians and never touches pins. Each motor
// type implements it: Uln2003Stepper today, an SCS0009 servo driver later. Swapping motors then
// means swapping the driver, nothing else.
#pragma once

#include <stdint.h>

class MotorDriver {
 public:
  virtual ~MotorDriver() = default;

  virtual void begin() = 0;
  // Called as often as possible from loop(); drives the motion.
  virtual void update(uint32_t now_us) = 0;

  virtual void setTarget(float rad) = 0;
  virtual float target() const = 0;
  // Estimated (stepper: counted steps) or measured (servo) joint position.
  virtual float position() const = 0;
  virtual bool isMoving() const = 0;

  // Declare the current pose to be 0 rad (after the joint was straightened by hand).
  virtual void setZero() = 0;
  // Stop as quickly as the acceleration limit allows.
  virtual void stop() = 0;
  // Remove power/torque. The motor re-energizes on the next move.
  virtual void release() = 0;

  virtual void setLimits(float max_vel_rad_s, float max_acc_rad_s2) = 0;
  // Native units (steps, servo ticks) per joint radian; set during calibration.
  virtual void setScale(float native_per_rad) = 0;
  virtual float scale() const = 0;
  // Move by a raw amount of native units, bypassing joint limits (calibration only).
  virtual void moveRaw(long native_units) = 0;
};
