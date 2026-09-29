// Tendra Hand V1 configuration: 21 joints on Feetech SCS0009 servos (half-duplex TTL bus via the
// FE-URT-1). Used only by the `hand_v1_servo` build (TENDRA_HAND_V1). The v0 stepper hand keeps
// using config.h, unchanged.
//
// Keep in sync with the PC side (v1 joint table in software/tendra) and the v1 sim model.
#pragma once

#include <stdint.h>

#define FIRMWARE_VERSION "0.2.0"
#define HAND_VARIANT "v1-servo"

constexpr int kNumJoints = 21;
constexpr float kDegToRad = 0.017453292519943295f;

// ----- Servo bus (ESP32-S3 UART1 -> FE-URT-1 -> servos) -----
// GPIO 17/18 are plain GPIOs on the ESP32-S3-N16R8: not strapping pins (0, 3, 45, 46), not
// USB (19, 20), not UART0 (43, 44), not flash/PSRAM (26-37), not the RGB LED (48).
// Wiring: ESP32 TX (GPIO 17) -> FE-URT-1 "TX" pin, ESP32 RX (GPIO 18) -> FE-URT-1 "RX" pin, GND to
// GND. Some FE-URT-1 boards have RX/TX silk-screened the wrong way round: if nothing answers
// (send B), swap the two wires.
constexpr int kServoUartTxPin = 17;
constexpr int kServoUartRxPin = 18;
constexpr uint32_t kServoBaud = 1000000;   // SCS0009 factory default
constexpr uint32_t kServoReplyTimeoutUs = 3000;

// ----- Motion -----
// Default joint-space limits (same as the v0 defaults: ~1.2 rad/s, ~2.5 rad/s^2).
constexpr float kDefaultMaxVelRad = 1.2f;   // rad/s
constexpr float kDefaultAccRad = 2.4f;      // rad/s^2
// Hard cap of the firmware's own ramp, in servo ticks/s (~117 deg/s at the servo horn).
constexpr float kServoMaxTicksPerS = 400.0f;
// Goal speed register value sent with every goal, a second (servo-side) speed limit. The firmware
// streams small goal steps, so this only matters if something goes wrong. Unit assumed to be
// ticks/s (Feetech SCSCL examples); 0 would mean "no limit", so it is never sent.
constexpr uint16_t kServoGoalSpeed = 600;
constexpr uint16_t kServoGoalTimeMs = 0;    // 0 = move at goal speed
// Goals are streamed to all servos with one SYNC WRITE this often.
constexpr uint32_t kGoalPeriodMs = 20;
// One servo's feedback (position, load, voltage, temperature) is read every kPollPeriodMs, round
// robin; offline servos are retried only every kOfflineRetryMs so they don't stall the loop.
constexpr uint32_t kPollPeriodMs = 2;
constexpr uint32_t kOfflineRetryMs = 1000;
constexpr uint8_t kMaxMissedReplies = 3;   // consecutive timeouts before a servo counts as offline

// Largest raw (uncalibrated, unclamped) move accepted by the M command, in servo ticks (~88 deg).
constexpr long kMaxRawMove = 300;

// Default spool radius on the servo horn (m). Informational until the joint drum radii are
// measured: servo_per_joint = r_joint_drum / r_spool (see servo_calibration.h).
constexpr float kDefaultSpoolRadius = 0.006f;  // 6 mm = joint drum radius (research.md), so 1:1

struct ServoJointConfig {
  const char* name;
  uint8_t servo_id;       // bus ID = motor number
  float min_rad;          // joint limits, positive = closing the hand
  float max_rad;
  bool invert;            // true if increasing servo ticks opens the joint
  float spool_radius_m;
  float servo_per_joint;  // servo radians per joint radian (calibration scale, default 1.0)
  int16_t zero_ticks;     // servo reading with the joint straight (q = 0); calibrate, then set here
};

// Motor order M1..M21 = protocol order = servo ID.
constexpr ServoJointConfig kJoints[kNumJoints] = {
    {"index_dip",       1, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"index_pip",       2, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"index_mcp_flex",  3, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"index_mcp_abd",   4, -15 * kDegToRad,  15 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"thumb_ip",        5, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"thumb_mcp_flex",  6, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"thumb_cmc_flex",  7, -13 * kDegToRad,  80 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"thumb_cmc_rot",   8, -100 * kDegToRad, 40 * kDegToRad, false, kDefaultSpoolRadius, 1.25f, 512},  // 7.5 mm drum
    {"thumb_mcp_abd",   9, -20 * kDegToRad,  20 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"middle_dip",     10, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"middle_pip",     11, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"middle_mcp_flex",12, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"middle_mcp_abd", 13, -15 * kDegToRad,  15 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"ring_dip",       14, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"ring_pip",       15, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"ring_mcp_flex",  16, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"ring_mcp_abd",   17, -15 * kDegToRad,  15 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"little_dip",     18, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"little_pip",     19, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"little_mcp_flex",20, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
    {"little_mcp_abd", 21, -20 * kDegToRad,  20 * kDegToRad, false, kDefaultSpoolRadius, 1.0f, 512},
};
// mcp_abd: positive = toward the thumb, for every finger.
