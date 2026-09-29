// Tendra Hand firmware configuration: pins, joints and motion defaults.
// Keep in sync with CLAUDE.md (pin map) and sim/convert.py (joint names, order, limits).
#pragma once

#include <stdint.h>

#define FIRMWARE_VERSION "0.1.0"

constexpr int kNumJoints = 8;
constexpr float kDegToRad = 0.017453292519943295f;

// 28BYJ-48 in half-step mode: 2 * 2037.886 steps per output revolution (gear ratio ~63.68:1).
constexpr float kHalfStepsPerRev = 4075.772f;
constexpr float kHalfStepsPerRad = kHalfStepsPerRev / 6.283185307f;  // ~648.7

// Joint radians -> motor steps. The motor spool radius is 10 mm; the joint-side moment arm is not
// measured yet, so every joint starts at 1:1 (one motor radian = one joint radian) and is
// calibrated with the M/K serial commands. See docs (calibration) and research/log.md.
constexpr float kDefaultStepsPerRad = kHalfStepsPerRad;

// Default motion limits, in motor steps. 28BYJ-48 at 5 V misses steps above ~1000 half-steps/s.
constexpr float kDefaultMaxSpeed = 800.0f;   // half-steps/s   (~1.2 rad/s at 1:1)
constexpr float kDefaultAccel = 1600.0f;     // half-steps/s^2
// Hard cap regardless of calibration or requested joint speed.
constexpr float kMotorMaxSpeed = 1000.0f;    // half-steps/s

// Coils are switched off after this long without motion, to save power and heat (5 V / 2 A
// supply). The 64:1 gearbox mostly holds position unpowered.
constexpr uint32_t kIdleReleaseMs = 1000;

// Largest raw (uncalibrated, unclamped) move accepted by the M command, in steps.
constexpr long kMaxRawMove = 4096;

struct JointConfig {
  const char* name;
  uint8_t pins[4];  // ULN2003 IN1..IN4
  float min_rad;    // joint limits, sign convention: positive = closing the hand
  float max_rad;
  bool invert;      // flip if a motor turns the wrong way (positive must close the hand)
};

// Motor order M1..M8 = protocol order = sim actuator order.
// Pin warnings (ESP32-S3-N16R8), see CLAUDE.md:
//   GPIO 35-37 are octal-PSRAM pins: usable only because PSRAM is disabled.
//   GPIO 0 is a boot strapping pin (M7 IN3). GPIO 43 is UART0 TX (M8 IN4).
constexpr JointConfig kJoints[kNumJoints] = {
    {"index_dip",      {4, 5, 6, 7},     -5 * kDegToRad,   95 * kDegToRad, false},  // M1
    {"index_pip",      {15, 16, 17, 18}, -5 * kDegToRad,   95 * kDegToRad, false},  // M2
    {"index_mcp_flex", {8, 14, 46, 9},   -5 * kDegToRad,   95 * kDegToRad, false},  // M3
    {"index_mcp_abd",  {10, 11, 12, 13}, -15 * kDegToRad,  15 * kDegToRad, false},  // M4
    {"thumb_ip",       {1, 2, 42, 41},   -5 * kDegToRad,   95 * kDegToRad, false},  // M5
    {"thumb_mcp",      {40, 39, 38, 37}, -5 * kDegToRad,   95 * kDegToRad, false},  // M6
    {"thumb_cmc_flex", {36, 35, 0, 45},  -13 * kDegToRad,  80 * kDegToRad, false},  // M7
    {"thumb_cmc_rot",  {48, 47, 21, 43}, -100 * kDegToRad, 40 * kDegToRad, false},  // M8
};
