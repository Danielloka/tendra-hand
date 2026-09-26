// Tendra Hand firmware: low-level joint control over USB serial.
//
// The PC sends joint targets in radians; this firmware moves the motors there smoothly.
// Text protocol, one command per line (easy to test by hand in a serial monitor):
//
//   P q1 q2 ... q8   set all joint targets [rad], motor order M1..M8
//   J i q            set target of joint i (1..8) [rad]
//   S                status -> "S q1 ... q8 moving_mask"
//   X                stop all joints (smooth deceleration)
//   R                release all motor coils (joints go limp)
//   Z                current pose = zero (straighten all joints by hand first)
//   M i n            raw move joint i by n native units (steps), ignores limits (calibration)
//   K i s            set scale of joint i [native units per rad] (calibration)
//   V i v a          set max velocity [rad/s] and acceleration [rad/s^2] of joint i (0 = all)
//   I                firmware info
//
// Replies: "OK", "ERR <reason>", or the requested data. Nothing moves until a command arrives.
#include <Arduino.h>

#include "config.h"
#include "motor_driver.h"
#include "uln2003_stepper.h"

namespace {

Uln2003Stepper g_steppers[kNumJoints] = {
    Uln2003Stepper(kJoints[0]), Uln2003Stepper(kJoints[1]), Uln2003Stepper(kJoints[2]),
    Uln2003Stepper(kJoints[3]), Uln2003Stepper(kJoints[4]), Uln2003Stepper(kJoints[5]),
    Uln2003Stepper(kJoints[6]), Uln2003Stepper(kJoints[7]),
};
// Code below only sees the HAL interface: swap the array above to change motor type.
MotorDriver* g_joints[kNumJoints];

constexpr size_t kLineMax = 160;
char g_line[kLineMax];
size_t g_line_len = 0;

// Parse a 1-based joint index; returns 0-based index or -1.
int parseJoint(const char* token) {
  if (token == nullptr) return -1;
  const int i = atoi(token);
  return (i >= 1 && i <= kNumJoints) ? i - 1 : -1;
}

void printStatus() {
  Serial.print('S');
  uint16_t moving = 0;
  for (int i = 0; i < kNumJoints; ++i) {
    Serial.print(' ');
    Serial.print(g_joints[i]->position(), 4);
    if (g_joints[i]->isMoving()) moving |= 1 << i;
  }
  Serial.print(' ');
  Serial.println(moving);
}

void printInfo() {
  Serial.print("I tendra-hand fw " FIRMWARE_VERSION " joints=");
  Serial.print(kNumJoints);
  for (int i = 0; i < kNumJoints; ++i) {
    Serial.print(' ');
    Serial.print(kJoints[i].name);
    Serial.print(':');
    Serial.print(g_joints[i]->scale(), 2);
  }
  Serial.println();
}

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
          Serial.println("ERR P needs 8 values");
          return;
        }
        q[i] = atof(t);
      }
      for (int i = 0; i < kNumJoints; ++i) g_joints[i]->setTarget(q[i]);
      Serial.println("OK");
      return;
    }
    case 'J': {
      const int i = parseJoint(next());
      const char* t = next();
      if (i < 0 || t == nullptr) break;
      g_joints[i]->setTarget(atof(t));
      Serial.println("OK");
      return;
    }
    case 'S':
      printStatus();
      return;
    case 'X':
      for (MotorDriver* j : g_joints) j->stop();
      Serial.println("OK");
      return;
    case 'R':
      for (MotorDriver* j : g_joints) j->release();
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
  Serial.begin(921600);  // native USB ignores the baud rate
}

void loop() {
  readSerial();
  const uint32_t now = micros();
  for (MotorDriver* j : g_joints) j->update(now);
}
