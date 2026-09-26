// PC-side unit tests for src/motion_profile.h (no ESP32 needed).
//
// Run from firmware/ (uses a throwaway C++ compiler from the ziglang Python package):
//   uvx --from ziglang python -m ziglang c++ -std=c++17 -O2 -w -Isrc test/host/test_motion_profile.cpp -o .pio/test_motion_profile.exe && .pio/test_motion_profile.exe
#include <cmath>
#include <cstdio>
#include <vector>

#include "motion_profile.h"

namespace {

struct Sample {
  uint32_t t_us;
  float speed;
};

struct Result {
  long final_pos;
  double t_done;
  double max_speed;
  std::vector<Sample> log;
};

// Simulate at 10 us resolution. Optionally change the target after `retarget_step` steps.
Result run(MotionProfile& p, long target, double t_limit_s, long retarget_step = -1,
           long retarget = 0) {
  p.setTarget(target);
  Result r{0, -1.0, 0.0, {}};
  long steps = 0;
  for (uint32_t t = 0; t < t_limit_s * 1e6; t += 10) {
    if (p.update(t) != 0) {
      ++steps;
      r.max_speed = std::fmax(r.max_speed, std::fabs(p.speed()));
      r.log.push_back({t, p.speed()});
      if (steps == retarget_step) p.setTarget(retarget);
    }
    if (!p.isMoving()) {
      r.t_done = t / 1e6;
      break;
    }
  }
  r.final_pos = p.position();
  return r;
}

int g_fails = 0;
void check(bool ok, const char* what) {
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++g_fails;
}

}  // namespace

int main() {
  constexpr float kMax = 800, kAcc = 1600;  // firmware defaults (config.h)
  {
    MotionProfile p;
    p.configure(kMax, kAcc);
    const Result r = run(p, 4000, 20);
    std::printf("  long move: t %.3f s, vmax %.1f\n", r.t_done, r.max_speed);
    check(r.final_pos == 4000, "long move ends exactly on target");
    check(r.max_speed <= kMax + 0.01, "never exceeds max speed");
    // Ideal trapezoid: accel 0.5 s (200 steps) + cruise 3600/800 = 4.5 s + decel 0.5 s = 5.5 s.
    check(std::fabs(r.t_done - 5.5) < 0.05, "takes the ideal trapezoid time (5.5 s)");
  }
  {
    MotionProfile p;
    p.configure(kMax, kAcc);
    check(run(p, 10, 5).final_pos == 10, "short move exact");
  }
  {
    MotionProfile p;
    p.configure(kMax, kAcc);
    check(run(p, -1500, 10).final_pos == -1500, "negative move exact");
  }
  {
    MotionProfile p;
    p.configure(kMax, kAcc);
    const Result r = run(p, 3000, 20, 800, -500);
    check(r.final_pos == -500, "reverses mid-move and ends on the new target");
    double worst = 0;
    for (size_t i = 1; i < r.log.size(); ++i) {
      const Sample &a = r.log[i - 1], &b = r.log[i];
      if (std::fabs(a.speed) < 60 || std::fabs(b.speed) < 60) continue;  // start/stop steps
      worst = std::fmax(worst, std::fabs(b.speed - a.speed) / ((b.t_us - a.t_us) / 1e6));
    }
    std::printf("  worst acceleration between steps: %.0f steps/s^2\n", worst);
    check(worst < kAcc * 1.3, "acceleration stays within limit (+30% discretisation)");
  }
  {
    MotionProfile p;
    p.configure(kMax, kAcc);
    p.setTarget(3000);
    uint32_t t = 0;
    for (; t < 1500000; t += 10) p.update(t);  // cruising at full speed
    p.stop();
    const long at = p.position();
    for (; t < 5000000 && p.isMoving(); t += 10) p.update(t);
    check(!p.isMoving() && p.position() - at <= 205, "stop() brakes within ~200 steps");
  }
  std::printf(g_fails ? "%d FAILED\n" : "ALL PASSED\n", g_fails);
  return g_fails;
}
