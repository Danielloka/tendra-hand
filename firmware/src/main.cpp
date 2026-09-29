// Tendra Hand firmware: low-level joint control over USB serial.
//
// Two hands, one protocol, chosen at build time (platformio.ini):
//   v0 (env tendra_s3):       8 joints, 28BYJ-48 steppers on ULN2003 boards  (config.h)
//   v1 (env hand_v1_servo):  21 joints, Feetech SCS0009 servos on one bus    (config_v1.h)
//
// The PC sends joint targets in radians; this firmware moves the motors there smoothly.
// Text protocol, one command per line (easy to test by hand in a serial monitor), N = joints:
//
//   P q1 q2 ... qN   set all joint targets [rad], motor order M1..MN
//   J i q            set target of joint i (1..N) [rad]
//   S                status -> "S q1 ... qN moving_mask"  (v1: measured positions)
//   F                feedback -> "F q,load,temp,volt ..." per joint (v1; "nan,0,0,0" if none)
//   X                stop all joints (smooth deceleration)
//   R                release all motors: coils off / servo torque off (joints go limp)
//   Z                current pose = zero (straighten all joints by hand first)
//   M i n            raw move joint i by n native units (steps/ticks), ignores limits (calibration)
//   K i s            set scale of joint i [native units per rad] (calibration)
//   V i v a          set max velocity [rad/s] and acceleration [rad/s^2] of joint i (0 = all)
//   B                (v1) scan the servo bus -> "B id id ..." (servos that answer a ping)
//   I                firmware info
//
// Replies: "OK", "ERR <reason>", or the requested data. Nothing moves until a command arrives.
#include <Arduino.h>

#include "motor_driver.h"

#if defined(TENDRA_HAND_V1)
#include "config_v1.h"
#include "scs0009_servo.h"
#include "scs_bus.h"
#include "scs_uart_port.h"
#else
#include "config.h"
#include "uln2003_stepper.h"
#endif

namespace {

#if defined(TENDRA_HAND_V1)
ScsUartPort g_port(Serial1);
ScsBus g_bus(g_port, kServoReplyTimeoutUs);
Scs0009Servo* g_servos[kNumJoints];  // created in setup()
uint32_t g_last_goal_ms = 0;
uint32_t g_last_poll_ms = 0;
int g_poll_next = 0;
#else
Uln2003Stepper g_steppers[kNumJoints] = {
    Uln2003Stepper(kJoints[0]), Uln2003Stepper(kJoints[1]), Uln2003Stepper(kJoints[2]),
    Uln2003Stepper(kJoints[3]), Uln2003Stepper(kJoints[4]), Uln2003Stepper(kJoints[5]),
    Uln2003Stepper(kJoints[6]), Uln2003Stepper(kJoints[7]),
};
#endif
// Code below only sees the HAL interface.
MotorDriver* g_joints[kNumJoints];

constexpr size_t kLineMax = 512;  // "P" with 21 values needs ~200 characters
char g_line[kLineMax];
size_t g_line_len = 0;

// Parse a 1-based joint index; returns 0-based index or -1.
int parseJoint(const char* token) {
  if (token == nullptr) return -1;
  const int i = atoi(token);
  return (i >= 1 && i <= kNumJoints) ? i - 1 : -1;
}

uint32_t offlineMask() {
  uint32_t mask = 0;
  for (int i = 0; i < kNumJoints; ++i) {
    if (!g_joints[i]->online()) mask |= 1ul << i;
  }
  return mask;
}

void printStatus() {
  Serial.print('S');
  uint32_t moving = 0;
  for (int i = 0; i < kNumJoints; ++i) {
    Serial.print(' ');
    Serial.print(g_joints[i]->position(), 4);
    if (g_joints[i]->isMoving()) moving |= 1ul << i;
  }
  Serial.print(' ');
  Serial.println(static_cast<unsigned long>(moving));
}

void printFeedback() {
  Serial.print('F');
  for (int i = 0; i < kNumJoints; ++i) {
    MotorFeedback fb;
    if (g_joints[i]->feedback(fb)) {
      Serial.printf(" %.4f,%.1f,%.0f,%.1f", fb.position_rad, fb.load_pct, fb.temperature_c,
                    fb.voltage_v);
    } else {
      Serial.print(" nan,0,0,0");
    }
  }
  Serial.println();
}

void printInfo() {
#if defined(TENDRA_HAND_V1)
  Serial.print("I tendra-hand fw " FIRMWARE_VERSION " hand=" HAND_VARIANT " joints=");
#else
  Serial.print("I tendra-hand fw " FIRMWARE_VERSION " joints=");
#endif
  Serial.print(kNumJoints);
  for (int i = 0; i < kNumJoints; ++i) {
    Serial.print(' ');
    Serial.print(kJoints[i].name);
    Serial.print(':');
    Serial.print(g_joints[i]->scale(), 2);
#if defined(TENDRA_HAND_V1)
    // name:ticks_per_rad:zero_ticks:online (copy calibrated zeros into config_v1.h)
    Serial.print(':');
    Serial.print(g_servos[i]->zeroTicks());
    Serial.print(':');
    Serial.print(g_joints[i]->online() ? 1 : 0);
#endif
  }
  Serial.println();
}

void releaseAll() {
#if defined(TENDRA_HAND_V1)
  g_bus.setTorque(scs::kBroadcastId, false);  // reaches every servo at once, even unknown ones
#endif
  for (MotorDriver* j : g_joints) j->release();
}

#if defined(TENDRA_HAND_V1)
void scanBus() {
  Serial.print('B');
  g_bus.setTimeout(1500);
  for (int id = 0; id <= scs::kMaxId; ++id) {
    if (g_bus.ping(static_cast<uint8_t>(id))) {
      Serial.print(' ');
      Serial.print(id);
    }
  }
  g_bus.setTimeout(kServoReplyTimeoutUs);
  Serial.println();
}

// Stream every changed goal to the bus in one SYNC WRITE.
void sendGoals() {
  uint8_t ids[kNumJoints];
  uint16_t ticks[kNumJoints];
  size_t n = 0;
  for (Scs0009Servo* s : g_servos) {
    if (s->torqueOn() && s->goalPending()) {
      ids[n] = s->id();
      ticks[n] = s->goalTicks();
      ++n;
      s->goalSent();
    }
  }
  if (n > 0) g_bus.syncWriteGoals(ids, ticks, n, kServoGoalTimeMs, kServoGoalSpeed);
}

// Read feedback of the next servo that is due (round robin).
void pollNext(uint32_t now_ms) {
  for (int k = 0; k < kNumJoints; ++k) {
    Scs0009Servo* s = g_servos[g_poll_next];
    g_poll_next = (g_poll_next + 1) % kNumJoints;
    if (s->pollDue(now_ms)) {
      s->poll(now_ms);
      return;
    }
  }
}
#endif

void handleLine(char* line) {
  char* save = nullptr;
  const char* cmd = strtok_r(line, " \t", &save);
  if (cmd == nullptr) return;
  auto next = [&]() { return strtok_r(nullptr, " \t", &save); };

  switch (toupper(cmd[0])) {
    case 'P': {
      float q[kNumJoints];
      for (int i = 0; i < kNumJoints; ++i) {
        const char* t = next();
        if (t == nullptr) {
          Serial.printf("ERR P needs %d values\n", kNumJoints);
          return;
        }
        q[i] = atof(t);
      }
      for (int i = 0; i < kNumJoints; ++i) g_joints[i]->setTarget(q[i]);  // offline: ignored
      const uint32_t offline = offlineMask();
      if (offline == 0) {
        Serial.println("OK");
      } else {
        Serial.printf("OK offline %lu\n", static_cast<unsigned long>(offline));
      }
      return;
    }
    case 'J': {
      const int i = parseJoint(next());
      const char* t = next();
      if (i < 0 || t == nullptr) break;
      if (!g_joints[i]->online()) {
        Serial.println("ERR joint offline");
        return;
      }
      g_joints[i]->setTarget(atof(t));
      Serial.println("OK");
      return;
    }
    case 'S':
      printStatus();
      return;
    case 'F':
      printFeedback();
      return;
    case 'X':
      for (MotorDriver* j : g_joints) j->stop();
      Serial.println("OK");
      return;
    case 'R':
      releaseAll();
      Serial.println("OK");
      return;
    case 'Z':
      for (MotorDriver* j : g_joints) {
        if (j->isMoving()) {
          Serial.println("ERR joints are moving");
          return;
        }
      }
      for (MotorDriver* j : g_joints) j->setZero();
      Serial.println("OK");
      return;
    case 'M': {
      const int i = parseJoint(next());
      const char* t = next();
      if (i < 0 || t == nullptr) break;
      if (!g_joints[i]->online()) {
        Serial.println("ERR joint offline");
        return;
      }
      g_joints[i]->moveRaw(atol(t));
      Serial.println("OK");
      return;
    }
    case 'K': {
      const int i = parseJoint(next());
      const char* t = next();
      if (i < 0 || t == nullptr || atof(t) <= 0) break;
      g_joints[i]->setScale(atof(t));
      Serial.println("OK");
      return;
    }
    case 'V': {
      const char* ti = next();
      const char* tv = next();
      const char* ta = next();
      if (ti == nullptr || tv == nullptr || ta == nullptr || atof(tv) <= 0 || atof(ta) <= 0) break;
      const int i = atoi(ti);
      if (i < 0 || i > kNumJoints) break;
      for (int k = 0; k < kNumJoints; ++k) {
        if (i == 0 || k == i - 1) g_joints[k]->setLimits(atof(tv), atof(ta));
      }
      Serial.println("OK");
      return;
    }
#if defined(TENDRA_HAND_V1)
    case 'B':
      scanBus();
      return;
#endif
    case 'I':
      printInfo();
      return;
  }
  Serial.println("ERR bad command (send I for info)");
}

void readSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (g_line_len > 0) {
        g_line[g_line_len] = '\0';
        handleLine(g_line);
        g_line_len = 0;
      }
    } else if (g_line_len < kLineMax - 1) {
      g_line[g_line_len++] = c;
    } else {
      g_line_len = 0;  // line too long: drop it
      Serial.println("ERR line too long");
    }
  }
}

}  // namespace

void setup() {
#if defined(TENDRA_HAND_V1)
  // Safety first: every servo torque OFF before anything else (broadcast, no reply), then each
  // servo is switched off again individually and its position read. Nothing moves.
  g_port.begin(kServoBaud, kServoUartRxPin, kServoUartTxPin);
  g_bus.setTorque(scs::kBroadcastId, false);
  for (int i = 0; i < kNumJoints; ++i) {
    g_servos[i] = new Scs0009Servo(kJoints[i], g_bus);
    g_joints[i] = g_servos[i];
    g_joints[i]->begin();
  }
#else
  // Safety first: every motor pin LOW (coils off) before anything else runs.
  for (const JointConfig& j : kJoints) {
    for (uint8_t pin : j.pins) {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    }
  }
  for (int i = 0; i < kNumJoints; ++i) {
    g_joints[i] = &g_steppers[i];
    g_joints[i]->begin();
  }
#endif
  Serial.begin(921600);  // native USB ignores the baud rate
}

void loop() {
  readSerial();
  const uint32_t now = micros();
  for (MotorDriver* j : g_joints) j->update(now);
#if defined(TENDRA_HAND_V1)
  const uint32_t now_ms = millis();
  if (now_ms - g_last_goal_ms >= kGoalPeriodMs) {
    g_last_goal_ms = now_ms;
    sendGoals();
  }
  if (now_ms - g_last_poll_ms >= kPollPeriodMs) {
    g_last_poll_ms = now_ms;
    pollNext(now_ms);
  }
#endif
}
