// Hardware abstraction layer (HAL) for one joint actuator.
//
// Everything above this interface works in joint radians and never touches pins. Each motor
// type implements it: Uln2003Stepper (v0 hand) and Scs0009Servo (v1 hand). Swapping motors then
// means swapping the driver, nothing else.
#pragma once

#include <stdint.h>

// Measured state, for motors that can report it (servos). Steppers have none.
struct MotorFeedback {
  float position_rad = 0.0f;   // measured joint position
  float load_pct = 0.0f;       // signed, % of maximum torque
  float temperature_c = 0.0f;
  float voltage_v = 0.0f;
  uint8_t error = 0;           // driver-specific error bits (0 = fine)
};

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

  // Optional feedback. online(): the motor answers and can accept targets (steppers: always).
  virtual bool online() const { return true; }
  // Fills `out` and returns true if measured data is available.
  virtual bool feedback(MotorFeedback& out) const {
    (void)out;
    return false;
  }
};
