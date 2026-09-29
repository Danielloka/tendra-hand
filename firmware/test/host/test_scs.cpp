// PC-side unit tests for the SCS servo stack (no ESP32, no servos needed):
//   scs_protocol.h (packets, checksum, parser), ScsBus (against a fake servo bus, with and without
//   echo), servo_calibration.h (joint angle <-> ticks) and Scs0009Servo (boot safety, ramp, limits).
//
// Run from firmware/ (uses a throwaway C++ compiler from the ziglang Python package):
//   uvx --from ziglang python -m ziglang c++ -std=c++17 -O2 -w -Isrc -Iinclude test/host/test_scs.cpp src/scs_bus.cpp src/scs0009_servo.cpp -o .pio/test_scs.exe && .pio/test_scs.exe
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <vector>

#include "config_v1.h"
#include "scs0009_servo.h"
#include "scs_bus.h"
#include "scs_protocol.h"
#include "servo_calibration.h"

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                   \
  do {                                                                \
    ++g_checks;                                                       \
    if (!(cond)) {                                                    \
      ++g_failures;                                                   \
      std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);   \
    }                                                                 \
  } while (0)

bool bytesEq(const uint8_t* got, size_t n, std::vector<uint8_t> want) {
  if (n != want.size()) {
    std::printf("  length %zu, want %zu\n", n, want.size());
    return false;
  }
  if (std::memcmp(got, want.data(), n) != 0) {
    std::printf("  got :");
    for (size_t i = 0; i < n; ++i) std::printf(" %02X", got[i]);
    std::printf("\n  want:");
    for (uint8_t b : want) std::printf(" %02X", b);
    std::printf("\n");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------------------------
// A fake SCS bus: servos with a register file (big-endian, like the SCS0009) that answer PING,
// READ, WRITE and SYNC WRITE. Optionally echoes every transmitted byte back, like a half-duplex
// adapter without echo suppression.

struct FakeServo {
  uint8_t mem[80] = {};
  int writes_to_goal = 0;
  int torque_writes = 0;
};

class FakeBus : public ScsPort {
 public:
  bool echo = false;
  std::map<uint8_t, FakeServo> servos;
  std::vector<std::vector<uint8_t>> sent;  // every packet the master wrote

  void write(const uint8_t* data, size_t n) override {
    sent.emplace_back(data, data + n);
    if (echo) rx.insert(rx.end(), data, data + n);
    scs::Parser p;  // instruction packets have the same framing as status packets
    for (size_t i = 0; i < n; ++i) {
      if (p.feed(data[i])) handle(p);
    }
  }
  int read() override {
    if (rx.empty()) return -1;
    const int c = rx.front();
    rx.pop_front();
    return c;
  }
  void clearInput() override { rx.clear(); }
  uint32_t micros() override { return t_us += 5; }  // time passes as the master polls

  std::deque<uint8_t> rx;
  uint32_t t_us = 0;

 private:
  void reply(uint8_t id, const uint8_t* data, size_t n) {
    uint8_t pkt[scs::kMaxPacket];
    const size_t len = scs::buildPacket(pkt, id, 0 /* error */, data, n);
    rx.insert(rx.end(), pkt, pkt + len);
  }
  void writeMem(FakeServo& s, uint8_t addr, const uint8_t* d, size_t n) {
    for (size_t i = 0; i < n; ++i) s.mem[addr + i] = d[i];
    if (addr <= scs::reg::kGoalPosition && addr + n > scs::reg::kGoalPosition) ++s.writes_to_goal;
    if (addr == scs::reg::kTorqueEnable) ++s.torque_writes;
  }
  void handle(const scs::Parser& p) {
    const uint8_t id = p.id();
    const uint8_t instr = p.error();
    const uint8_t* prm = p.params();
    if (instr == scs::kInstSyncWrite && id == scs::kBroadcastId) {
      const uint8_t addr = prm[0], len = prm[1];
      for (size_t k = 2; k + len + 1u <= p.paramCount(); k += len + 1u) {
        auto it = servos.find(prm[k]);
        if (it != servos.end()) writeMem(it->second, addr, prm + k + 1, len);
      }
      return;
    }
    if (id == scs::kBroadcastId) {
      if (instr == scs::kInstWrite) {
        for (auto& kv : servos) writeMem(kv.second, prm[0], prm + 1, p.paramCount() - 1);
      }
      return;  // no reply to broadcast
    }
    auto it = servos.find(id);
    if (it == servos.end()) return;  // nobody home: master times out
    FakeServo& s = it->second;
    switch (instr) {
      case scs::kInstPing:
        reply(id, nullptr, 0);
        break;
      case scs::kInstRead:
        reply(id, s.mem + prm[0], prm[1]);
        break;
      case scs::kInstWrite:
        writeMem(s, prm[0], prm + 1, p.paramCount() - 1);
        reply(id, nullptr, 0);
        break;
    }
  }
};

void setPresent(FakeServo& s, uint16_t ticks) { scs::put16(s.mem + scs::reg::kPresentPosition, ticks); }
uint16_t goalOf(const FakeServo& s) { return scs::get16(s.mem + scs::reg::kGoalPosition); }

// ---------------------------------------------------------------------------------------------

void testPackets() {
  std::printf("packets\n");
  uint8_t b[scs::kMaxPacket];

  // Classic ping of ID 1: checksum = ~(1 + 2 + 1) = 0xFB.
  CHECK(bytesEq(b, scs::buildPing(b, 1), {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));

  // Read 8 bytes from register 56 (feedback block).
  CHECK(bytesEq(b, scs::buildRead(b, 1, 56, 8), {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x08, 0xB8}));

  // Goal 1000 ticks, time 0, speed 1500 -- big-endian (0x03E8 -> 03 E8).
  uint8_t goal[6];
  scs::encodeGoal(goal, 1000, 0, 1500);
  CHECK(bytesEq(b, scs::buildWrite(b, 1, scs::reg::kGoalPosition, goal, 6),
                {0xFF, 0xFF, 0x01, 0x09, 0x03, 0x2A, 0x03, 0xE8, 0x00, 0x00, 0x05, 0xDC, 0xFC}));

  // Broadcast torque off.
  const uint8_t off = 0;
  CHECK(bytesEq(b, scs::buildWrite(b, scs::kBroadcastId, scs::reg::kTorqueEnable, &off, 1),
                {0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x28, 0x00, 0xD2}));

  // Sync write of two goals: LEN = (6 + 1) * 2 + 4 = 18.
  const uint8_t ids[2] = {1, 2};
  uint8_t data[12];
  scs::encodeGoal(data, 0x200, 0, 600);
  scs::encodeGoal(data + 6, 0x300, 0, 600);
  CHECK(bytesEq(b, scs::buildSyncWrite(b, scs::reg::kGoalPosition, 6, ids, data, 2),
                {0xFF, 0xFF, 0xFE, 0x12, 0x83, 0x2A, 0x06, 0x01, 0x02, 0x00, 0x00, 0x00, 0x02, 0x58,
                 0x02, 0x03, 0x00, 0x00, 0x00, 0x02, 0x58, 0x80}));

  // Sync write of all 20 servos still fits in one packet.
  uint8_t ids20[20], data20[20 * 6] = {};
  for (int i = 0; i < 20; ++i) ids20[i] = static_cast<uint8_t>(i + 1);
  CHECK(scs::buildSyncWrite(b, scs::reg::kGoalPosition, 6, ids20, data20, 20) == 20 * 7 + 8);

  // Big-endian helpers and sign-magnitude feedback.
  uint8_t two[2];
  scs::put16(two, 0x1234);
  CHECK(two[0] == 0x12 && two[1] == 0x34 && scs::get16(two) == 0x1234);
  CHECK(scs::signMag(0x0405, 10) == -5);
  CHECK(scs::signMag(0x0005, 10) == 5);
  CHECK(scs::signMag(0x8010, 15) == -16);
  CHECK(std::fabs(scs::kTicksPerRad - 195.57f) < 0.01f);
}

void testParser() {
  std::printf("parser\n");
  scs::Parser p;
  auto feedAll = [&](std::vector<uint8_t> bytes) {
    int frames = 0;
    for (uint8_t c : bytes) frames += p.feed(c) ? 1 : 0;
    return frames;
  };

  // Status of ID 3 with error 0 and two data bytes, after line noise.
  uint8_t pkt[16];
  const uint8_t d[2] = {0x01, 0xF4};
  const size_t n = scs::buildPacket(pkt, 3, 0x00, d, 2);
  std::vector<uint8_t> bytes = {0x00, 0x13, 0xFF, 0x42};
  bytes.insert(bytes.end(), pkt, pkt + n);
  CHECK(feedAll(bytes) == 1);
  CHECK(p.id() == 3 && p.error() == 0 && p.paramCount() == 2);
  CHECK(scs::get16(p.params()) == 500);

  // Corrupted checksum: rejected and counted; the next good frame still parses.
  std::vector<uint8_t> bad(pkt, pkt + n);
  bad.back() ^= 0x55;
  const uint32_t bad_before = p.badFrames();
  CHECK(feedAll(bad) == 0);
  CHECK(p.badFrames() == bad_before + 1);
  CHECK(feedAll(std::vector<uint8_t>(pkt, pkt + n)) == 1);

  // Extra 0xFF in the header is tolerated.
  std::vector<uint8_t> hdr = {0xFF};
  hdr.insert(hdr.end(), pkt, pkt + n);
  CHECK(feedAll(hdr) == 1 && p.id() == 3);
}

void testBus(bool echo) {
  std::printf("bus (echo %s)\n", echo ? "on" : "off");
  FakeBus port;
  port.echo = echo;
  port.servos[1] = FakeServo{};
  port.servos[3] = FakeServo{};
  ScsBus bus(port, 2000);

  CHECK(bus.ping(1));
  CHECK(bus.ping(3));
  CHECK(!bus.ping(2));  // absent: times out
  CHECK(bus.timeouts() == 1);

  FakeServo& s1 = port.servos[1];
  setPresent(s1, 700);
  scs::put16(s1.mem + scs::reg::kPresentLoad, 0x0400 | 123);  // -123 = 12.3 % opening
  s1.mem[scs::reg::kPresentVoltage] = 52;
  s1.mem[scs::reg::kPresentTemperature] = 31;
  ScsFeedback fb;
  CHECK(bus.readFeedback(1, fb));
  CHECK(fb.position == 700 && fb.load == -123 && fb.voltage_dv == 52 && fb.temperature_c == 31);
  CHECK(!bus.readFeedback(2, fb));

  CHECK(bus.setTorque(1, true));
  CHECK(s1.mem[scs::reg::kTorqueEnable] == 1);
  CHECK(bus.writeGoal(3, 256, 0, 600));
  CHECK(goalOf(port.servos[3]) == 256);
  CHECK(scs::get16(port.servos[3].mem + scs::reg::kGoalSpeed) == 600);

  const uint8_t ids[2] = {1, 3};
  const uint16_t ticks[2] = {111, 999};
  CHECK(bus.syncWriteGoals(ids, ticks, 2, 0, 600));
  CHECK(goalOf(port.servos[1]) == 111 && goalOf(port.servos[3]) == 999);

  CHECK(bus.setTorque(scs::kBroadcastId, false));  // broadcast: no reply expected
  CHECK(port.servos[1].mem[scs::reg::kTorqueEnable] == 0);
}

void testCalibration() {
  std::printf("calibration\n");
  ServoCalibration c{scs::kTicksPerRad, 512, false};
  CHECK(c.jointToTicks(0.0f) == 512);
  CHECK(c.jointToTicks(1.0f) == 512 + 196);   // 195.57 rounds to 196
  CHECK(c.jointToTicks(-0.5f) == 512 - 98);
  CHECK(std::fabs(c.ticksToJoint(512 + 196) - 196 / scs::kTicksPerRad) < 1e-6f);

  ServoCalibration inv{2.0f * scs::kTicksPerRad, 300, true};  // 2 servo rad per joint rad, inverted
  CHECK(inv.jointToTicks(0.5f) == 300 - 196);
  for (float q : {-1.2f, -0.3f, 0.0f, 0.25f, 1.6f}) {
    CHECK(std::fabs(inv.ticksToJoint(inv.jointToTicks(q)) - q) < 1.0f / inv.native_per_rad);
  }
  CHECK(std::fabs(servoPerJointFromRadii(0.006f, 0.008f) - 0.75f) < 1e-6f);
  CHECK(clampTicks(-40, 0, 1023) == 0 && clampTicks(2000, 0, 1023) == 1023);
}

// Runs the servo's update loop for `ms` of simulated time, streaming goals like main.cpp does.
void run(Scs0009Servo& s, FakeBus& port, ScsBus& bus, uint32_t& t_us, uint32_t ms) {
  for (uint32_t k = 0; k < ms * 10; ++k) {
    t_us += 100;
    s.update(t_us);
    if (k % 200 == 0 && s.goalPending() && s.torqueOn()) {
      const uint8_t id = s.id();
      const uint16_t g = s.goalTicks();
      bus.syncWriteGoals(&id, &g, 1, kServoGoalTimeMs, kServoGoalSpeed);
      s.goalSent();
      setPresent(port.servos[id], g);  // ideal servo: follows the goal
    }
  }
}

void testServo() {
  std::printf("Scs0009Servo\n");
  FakeBus port;
  port.servos[1] = FakeServo{};
  setPresent(port.servos[1], 600);        // joint was left bent at boot
  port.servos[1].mem[scs::reg::kTorqueEnable] = 1;  // and (hypothetically) powered up with torque
  ScsBus bus(port, 2000);
  const ServoJointConfig& cfg = kJoints[0];  // index_dip, ID 1, -5..95 deg
  Scs0009Servo s(cfg, bus);

  // Boot: torque switched off, position read, no goal written.
  s.begin();
  CHECK(port.servos[1].mem[scs::reg::kTorqueEnable] == 0);
  CHECK(port.servos[1].writes_to_goal == 0);
  CHECK(s.online() && !s.torqueOn() && !s.isMoving());
  CHECK(std::fabs(s.position() - (600 - 512) / scs::kTicksPerRad) < 1e-4f);

  // A limp joint moved by hand is tracked.
  setPresent(port.servos[1], 580);
  s.poll(1);
  CHECK(std::fabs(s.position() - (580 - 512) / scs::kTicksPerRad) < 1e-4f);

  // First target: goal = present position BEFORE torque on (no jump), then a smooth ramp.
  s.setTarget(0.5f);
  CHECK(s.torqueOn());
  CHECK(port.servos[1].mem[scs::reg::kTorqueEnable] == 1);
  CHECK(port.servos[1].writes_to_goal == 1 && goalOf(port.servos[1]) == 580);
  CHECK(s.isMoving());
  uint32_t t = 0;
  uint16_t prev = 580;
  int max_jump = 0;
  for (int i = 0; i < 300 && s.isMoving(); ++i) {
    run(s, port, bus, t, 20);
    const int jump = std::abs(goalOf(port.servos[1]) - prev);
    if (jump > max_jump) max_jump = jump;
    prev = goalOf(port.servos[1]);
  }
  CHECK(!s.isMoving());
  CHECK(goalOf(port.servos[1]) == 512 + 98);  // 0.5 rad = 97.8 ticks
  CHECK(max_jump <= 10);  // 400 ticks/s cap -> <= 8 ticks per 20 ms, +1 for rounding
  s.poll(2);
  CHECK(std::fabs(s.position() - 0.5f) < 0.01f);

  // Targets beyond the joint limit are clamped to it (95 deg).
  s.setTarget(3.0f);
  CHECK(std::fabs(s.target() - 95 * kDegToRad) < 0.01f);
  s.stop();

  // Release: torque off.
  s.release();
  CHECK(!s.torqueOn() && port.servos[1].mem[scs::reg::kTorqueEnable] == 0);

  // Zero: the current (bent) pose becomes q = 0.
  s.poll(3);
  s.setZero();
  CHECK(std::fabs(s.position()) < 1e-6f);

  // Feedback is reported in joint units.
  scs::put16(port.servos[1].mem + scs::reg::kPresentLoad, 250);
  port.servos[1].mem[scs::reg::kPresentVoltage] = 50;
  s.poll(4);
  MotorFeedback fb;
  CHECK(s.feedback(fb) && std::fabs(fb.load_pct - 25.0f) < 1e-4f && std::fabs(fb.voltage_v - 5.0f) < 1e-4f);

  // A servo that never answered cannot be switched on.
  Scs0009Servo ghost(kJoints[1], bus);  // ID 2: not on the bus
  ghost.begin();
  CHECK(!ghost.online());
  ghost.setTarget(0.3f);
  CHECK(!ghost.torqueOn() && !ghost.isMoving());
  MotorFeedback none;
  CHECK(!ghost.feedback(none));
}

void testConfig() {
  std::printf("config_v1\n");
  CHECK(kNumJoints == 20);
  for (int i = 0; i < kNumJoints; ++i) {
    CHECK(kJoints[i].servo_id == i + 1);
    CHECK(kJoints[i].min_rad < 0.0f && kJoints[i].max_rad > 0.0f);  // q = 0 (straight) reachable
    CHECK(kJoints[i].servo_per_joint > 0.0f);
    for (int k = 0; k < i; ++k) CHECK(std::strcmp(kJoints[i].name, kJoints[k].name) != 0);
  }
}

}  // namespace

int main() {
  testPackets();
  testParser();
  testBus(false);
  testBus(true);
  testCalibration();
  testServo();
  testConfig();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
