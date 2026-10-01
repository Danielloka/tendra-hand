// Tendra Hand V1 configuration: 16 Feetech SCS0009 servos (half-duplex TTL bus via the FE-URT-1)
// for 20 joints: the four finger DIPs are coupled passively to their PIPs, so the firmware never
// sees them. Used only by the `hand_v1_servo` build (TENDRA_HAND_V1). The v0 stepper hand keeps
// using config.h, unchanged.
//
// Keep in sync with the PC side (v1 joint table in software/tendra) and the v1 sim model.
#pragma once

#include <stdint.h>

#define FIRMWARE_VERSION "0.4.0"  // 0.4.0: 16 servos (DIP coupled to PIP), 5 mm spools, IDs renumbered
#define HAND_VARIANT "v1-servo"

constexpr int kNumJoints = 16;
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

// Spool radius on the servo horn (m), the same on every servo. Informational: the scale that
// matters is servo_per_joint = r_joint_drum / r_spool (see servo_calibration.h).
constexpr float kDefaultSpoolRadius = 0.005f;  // 5 mm (2026-10-01, was 6 mm)

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

// Motor order M1..M16 = protocol order = servo ID (since 0.4.0, 2026-10-01). The four finger DIPs
// have no servo: a passive coupling tendon bends each DIP by 0.75 x its PIP (see tendon_router.py).
// servo_per_joint = drum radius / 5 mm spool: finger mcp_flex 7 mm -> 1.4, thumb_cmc_rot 7.5 mm ->
// 1.5, every other drum 6 mm -> 1.2.
// zero_ticks: thumb_cmc_rot needs 140 deg x 1.5 = 210 servo degrees, so its zero sits off-centre
// (665: -100..40 deg -> ticks 153..870). If its invert flag turns out true, use ~359 instead.
constexpr ServoJointConfig kJoints[kNumJoints] = {
    {"index_pip",       1, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.2f, 512},
    {"index_mcp_flex",  2, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.4f, 512},
    {"index_mcp_abd",   3, -15 * kDegToRad,  15 * kDegToRad, false, kDefaultSpoolRadius, 1.2f, 512},
    {"thumb_ip",        4, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.2f, 512},
    {"thumb_mcp_flex",  5, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.2f, 512},
    {"thumb_cmc_flex",  6, -13 * kDegToRad,  45 * kDegToRad, false, kDefaultSpoolRadius, 1.2f, 512},
    {"thumb_cmc_rot",   7, -100 * kDegToRad, 40 * kDegToRad, false, kDefaultSpoolRadius, 1.5f, 665},  // 7.5 mm drum
    {"middle_pip",      8, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.2f, 512},
    {"middle_mcp_flex", 9, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.4f, 512},
    {"middle_mcp_abd", 10, -15 * kDegToRad,  15 * kDegToRad, false, kDefaultSpoolRadius, 1.2f, 512},
    {"ring_pip",       11, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.2f, 512},
    {"ring_mcp_flex",  12, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.4f, 512},
    {"ring_mcp_abd",   13, -15 * kDegToRad,  15 * kDegToRad, false, kDefaultSpoolRadius, 1.2f, 512},
    {"little_pip",     14, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.2f, 512},
    {"little_mcp_flex",15, -5 * kDegToRad,   95 * kDegToRad, false, kDefaultSpoolRadius, 1.4f, 512},
    {"little_mcp_abd", 16, -20 * kDegToRad,  20 * kDegToRad, false, kDefaultSpoolRadius, 1.2f, 512},
};
// mcp_abd: positive = toward the thumb, for every finger.
