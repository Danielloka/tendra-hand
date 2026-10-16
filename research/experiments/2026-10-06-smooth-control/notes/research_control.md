# Control layer between a learned policy and the motors (Tendra Hand)

Legend: [S] = found in a source I opened/searched. [M] = from my own background knowledge, not verified this session. [?] = uncertain.

## 0. The big picture in plain words
A learned policy outputs a new target 20 times a second. If the servo just receives each target as a step, it jumps toward it, stops, jumps again: stair-step jitter. And if the policy output is noisy, the hand "buzzes" even when nothing should move. Fix it in layers:
1. Train the policy to be smooth (reward/regularisation, action filter).
2. Put a fast, simple controller between policy and motors (interpolates targets, limits speed/jerk).
3. Let the servo itself ignore tiny errors (deadband) and run gentle gains.
4. Add a task-phase state machine that freezes targets when "hold".
5. Safety filters and watchdogs on top.
6. Model the real servo in sim (latency, noise, gains) so sim matches real.
7. Measure it with a repeatable test protocol.

## 1. Hierarchical control + target interpolation
**Explanation.** Common practice: policy at 50 Hz outputs setpoints, a low-level PD controller runs much faster (examples: policy 50 Hz, low-level 200 Hz in sim / 8 kHz on the real robot) [S: search summary of legged RL papers, e.g. https://arxiv.org/pdf/2406.01152 and others; I did not open each]. With a PD policy the policy can be slow (~50 Hz) while torque is produced at ~1 kHz [S, same search].
**Ruckig** (online trajectory generation): you give current position/velocity/acceleration and a new target plus max velocity/acceleration/jerk; each control cycle it returns the next state. A new target mid-motion is handled instantly from the current state, so there are no discontinuities. Works for cycles down to 500 us (2 kHz); mean 19.8 us for 7 DoF; Python via pip (community version) [S: https://github.com/pantor/ruckig, https://arxiv.org/pdf/2105.04830 (search snippet)].
**Alternatives** [M]: linear interpolation (simple, velocity jumps at each new target), cubic/quintic (continuous velocity/accel, but you must re-plan when the target changes), minimum-jerk (quintic with zero end vel/acc), and a critically damped 2nd-order filter (one line of code; no planning; behaves like a smooth chase of the target). The critically damped filter or Ruckig is better than interpolation when the target is replaced every 50 ms, because the "end of segment" never arrives.
**Why it removes stair-steps.** The servo sees a continuous stream of tiny position steps (e.g. every 5-10 ms) instead of a big step every 50 ms; velocity is continuous.
**Numbers.** OpenAI Dactyl smoothed actions with an exponential moving average, coefficient 0.3 per 80 ms [S: search summary of https://arxiv.org/pdf/1808.00177]. Other works: alpha 0.1 or gamma 0.98 [S, same search; contexts unverified]. Rule of thumb [M]: filter time constant about 1-3 policy steps (50-150 ms for 20 Hz).
**Tendra.**
- Sim: policy 20 Hz -> add a "setpoint generator" (critically damped filter or Ruckig, per joint, on the 4 ms physics tick, = 250 Hz) before the position actuators. Train WITH it, so the policy knows about the lag (never filter only at deployment).
- ESP32: run the same generator at 100-200 Hz in firmware and send Goal Position to the SCS0009 bus each tick. Bus budget: 1 Mbps, one Sync Write for 16 servos is roughly 16 x 6-7 bytes ~ 120 bytes ~ 1.2 ms+; so 100-200 Hz is feasible, a full feedback read of 16 servos is the expensive part (SCS firmware does not answer Sync Read, so reads are one by one) [S: https://github.com/pollen-robotics/rustypot/pull/132]. Compute-light: Ruckig is C++ (no dependency issue on ESP32 is verified? [?]); a hand-written filter is safer.

## 2. Impedance / compliance, grip force, slip
**Explanation.** Impedance control = the joint behaves like a spring+damper around the target (stiffness K, damping D). Low K = soft and safe, high K = precise but shaky and strong. A position servo with a P gain already is this spring; the "stiffness" is its P gain / torque limit.
**Variable impedance by RL.** VICES (Martin-Martin et al., IROS 2019) makes the policy output both target and stiffness; improves sample efficiency, low energy, safe, transfers sim-to-real [S: https://arxiv.org/abs/1906.08880]. Many newer papers do the same (stiffness as an action) [S: search list e.g. https://arxiv.org/html/2603.14068].
**Hand precedent.** LEAP Hand (Dynamixel) uses current-based position mode: control law current = Kp*err - Kd*derr; a current cap limits force; fingertip force readings were mapped linearly to Kp in one use; effective Kp ~0.0786 Nm/rad per 100 units, Kd ~0.0014 [S: https://arxiv.org/pdf/2309.06440 (search summary)]. Current limits ~300 mA (Lite) / ~550 mA (Full) [S: search]. Continuous grasping can overheat motors [S].
**Slip detection.** Needs tactile/force sensing; each incipient-slip event = signal to raise grip force [S: https://arxiv.org/pdf/1810.13381, https://arxiv.org/pdf/2202.06140 (cable-driven hand with PVDF slip sensor)]. Tendra has no sensing yet. Proxy [M]: SCS0009 reports present load (reg 60-61) and current (69-70 in SCSCL header) [S: FTServo_Arduino SCSCL.h]; a sudden drop in load with position creeping = possible slip. Weak, noisy; mark as research idea.
**Tendra.**
- Sim: add 1-2 "grip" scalars (global stiffness / torque limit) to the action space, with a penalty on force; or fix a per-phase gain schedule (stiff in reach, softer in grasp). Safer than per-joint stiffness (20 more outputs = more jitter).
- Firmware: SCS0009 has no current-mode like Dynamixel; the closest are the torque-limit and P-coefficient registers (see section 3). Note CLAUDE.md: SCS0009 backs off at >80% load for 2 s, so hold pretension low.

## 3. Servo-level smoothness: Feetech SCS0009
**Registers found** (SCS series; from FTServo SCSCL.h and the SCS0009 datasheet page) [S: https://raw.githubusercontent.com/ftservo/FTServo_Arduino/main/src/SCSCL.h, https://docs.atelierdekhalil.com/zig-animatronic/robot/head_and_neck/servomotor-feetech-scs0009-datasheet/]:
| Addr | Name | Notes |
|---|---|---|
| 5 / 6 | ID / baud | default 1 Mbps |
| 9-12 | min/max angle limits | 0/0 = motor (wheel) mode |
| 21 / 22 / 23 | P / D / I coefficient | defaults found in Feetech-tuna: P 32, D 32, I 0 [S: https://github.com/iotdesignshop/Feetech-tuna] (for which model? [?]) |
| 24-25 | Punch (min start force) | doc says it "reduces chatter at take-off" (translated from French) |
| 26 / 27 | CW / CCW dead zone | 1 unit ~ 0.38 deg; larger = less sensitive, less jitter [S: search result, source not opened; the 0.38 deg number is for a 1024-step ~ 300-360 deg servo [?]] |
| 40 | Torque enable | |
| 42-43 | Goal position | |
| 44-45 | Goal time | SCS moves over a given time (ms); a built-in interpolation |
| 46-47 | Goal speed | |
| 48 | Lock | EEPROM write protect |
| 56-63 | present position, speed, load, voltage, temperature | |
| 66 | Moving flag | |
| 69-70 | Present current | |
Gaps/conflicts: the Atelier page says SCS0009 lacks the STS-type position offset and acceleration parameters, while rustypot PR #132 lists "Address 41: Acceleration" for SCS0009 [S: https://github.com/pollen-robotics/rustypot/pull/132]. Treat acceleration register as UNVERIFIED [?]: test on hardware (write 41, see if motion profile changes). Also "factory block" 78-84 (moving threshold, dead zones, velocity unit, high-temp shutdown, max velocity) is community-reversed, not vendor docs [S: https://github.com/adityakamath/SCServo_Linux]. I could not obtain the official SCS0009 PDF memory table (Feetech PDFs fetched as binary, saved in the tool-results folder of this session but not parsed; manuals.plus returned 403).
**Practical recipe** [M, built on the above]:
1. Set dead zone to 1-3 units (~0.4-1.1 deg) on all servos; this is the cheapest "rest still" fix. Cost: steady-state error up to the dead zone, which is fine for a hand with tendon slack anyway.
2. Lower P (and D) if the servo buzzes (LEAP advice: "if jittery, lower P and D" [S: LEAP Hand V2 docs via search]).
3. Use Goal Time/Goal Speed so the servo does its own ramp between ticks; never send max speed. (Speed units: sign-magnitude on bit 15 [S: rustypot PR].)
4. Punch: raise only if the finger fails to start moving (stiction); too high = chatter.
5. Torque limit (EEPROM/SRAM, address not confirmed [?]) = the grip-force knob.
6. When "holding" for seconds, optionally release torque only if the tendon is self-locking: not true for tendon hands; don't.
**Other hands.** Amazing Hand (Pollen/HF; 2 SCS0009 per finger, rustypot Python or Arduino + Feetech TTL linker; calibration scripts; repo has no documented jitter or gain settings) [S: https://github.com/pollen-robotics/AmazingHand]. A forum user reports noisy position readings on a still SCS0009 [S: search result, https://forum.seeedstudio.com/t/issues-with-bus-servo-driver-board-scs0009/295301, not opened]: filter feedback. ORCA hand (ETH): tendon-driven, 17 DoF, >10,000 continuous cycles (~20 h) without failure, auto-calibration, tensioning, tendon routing through the joint centre of rotation [S: https://arxiv.org/pdf/2504.04259]; relevant to Tendra's on-axis routing choice. InMoov: not researched.

## 4. Deadband / hysteresis and a hold state machine
**Explanation.** Deadband: ignore a new command if it differs from the last sent by less than epsilon (e.g. 0.5-1 deg). Hysteresis: use a bigger threshold to START moving than to STOP (e.g. start >1.0 deg, stop <0.3 deg) so it doesn't chatter at the edge. A phase machine (reach -> grasp -> hold -> release) freezes the finger targets in hold, so policy noise cannot reach the servos.
**Evidence.** I found no paper specific to this; it is standard engineering [M]. The nearest found: policy-smoothness regularisation work [S: CAPS, https://www.alphaxiv.org/abs/2012.06644, http://ai.bu.edu/caps/] which does the same inside the policy.
**Tendra design** [M]:
- Phase detection from observed state (contact count with object, object lifted > x cm, finger speed < v for N ticks).
- In hold: targets = latched value; only allow changes if object moves/slips beyond a threshold, or release phase begins.
- Command gating in firmware: send Goal Position only if |new - last_sent| > 1 tick (~0.3 deg); keep the servo's own dead zone as a second layer.
- "Rest": after N seconds with no command, latch and optionally lower torque limit (not zero, tendons slack).

## 5. Safety filters, rate limits, watchdogs
- Control barrier functions (CBF) = a math filter that minimally modifies the commanded action only when a safety condition is about to be violated [S: https://arxiv.org/pdf/2510.14959 CBF-RL, https://www.emergentmind.com/topics/safety-filtering-reinforcement-learning]. CBF-RL: filter used in training plus safety rewards so the policy internalises the constraints, >90% success without runtime filter [S, same].
- For Tendra a full CBF is overkill; use simple versions [M]: clip to joint limits (config_v1.h), per-tick rate limiter (max deg/tick), jerk limit (Ruckig), load limit (read load; reduce target if > ~70%, since SCS0009 trips at 80% for 2 s [project CLAUDE.md]), temperature/voltage limits from regs 62-63, command watchdog (no valid command for 200-500 ms -> hold position or release torque; firmware already torque-off at boot), bus-error counter, NaN/inf rejection on host.
- Runtime monitoring [M]: log present vs goal position, flag tracking error > threshold, stalls, rising temp; anomaly = z-score of tracking error against sim prediction.

## 6. Sim-to-real for smooth control
- Actuator networks (Hwangbo et al., Science Robotics 2019): learn a neural net of the servo's torque response from real data, use it in sim [S: https://www.science.org/doi/10.1126/scirobotics.aau5872 (abstract only; page fetch 403)]. Details (input = recent position-error/velocity history, small MLP) are from my memory [M].
- For a Feetech servo: log command vs present position at 100-200 Hz for chirps/steps (system identification), fit a 1st/2nd order model + deadband + rate limit + delay; put it in MuJoCo as (a) position actuator kp/kv matched, (b) a delay buffer for goal position, (c) quantisation to 1024 ticks, (d) deadband. Randomise kp, delay, noise, friction around the fitted values.
- Randomisation precedent: Dactyl/DeXtreme extensive randomisation + EMA smoothed actions [S: https://arxiv.org/pdf/1808.00177]. Legged examples simulate ~20 ms action latency and randomise ±20-50% [S: search summary].
- Training-side smoothness: CAPS (temporal + spatial smoothness terms added to policy loss) [S: https://www.alphaxiv.org/abs/2012.06644]; Grad-CAPS [S: https://arxiv.org/pdf/2407.04315]; action-rate and joint-acceleration penalties in legged reward functions [S: search summary]. Cheap to add to the existing PPO: penalty on |a_t - a_{t-1}|^2 and on second difference; plus policy-output Lipschitz regulariser. The project already has a jerk penalty in the reward; the missing parts are the low-level interpolator and the actuator model.
- Note on Tendra: CLAUDE.md says the GPU env (MuJoCo Warp) and CPU env must stay equal; implement the setpoint filter in both (a JAX version is a few lines for a 2nd-order filter; Ruckig has no JAX version, use the filter in training and Ruckig-like limits only as a deployment clamp, or implement a jerk-limited filter manually).

## 7. Reliability engineering
- Sample size: a 90% success rate measured with 70 trials has a 95% Clopper-Pearson interval of 80.5-95.9%; with 1,030 trials, 88.0-91.8%; 10 episodes per task = ~19-point interval; 100 trials ~ 6 points [S: https://dev.to/tidiane_stano_c6b88f8b685/robot-policy-evaluation-why-90-vs-92-proves-little and https://www.therobotreport.com/nvidia-shares-how-evaluate-general-purpose-robot-policies-real-world-deployment/]. Report Wilson or Clopper-Pearson intervals, not just the %.
- "Rule of three" [M]: 0 failures in N trials -> 95% upper bound on failure rate ~3/N (N=100 -> <3%).
- Sim regression suite [M]: fixed seeds (e.g. 200 episodes per object/hand, both sides), run on every control/env change, gate merges on thresholds. Tendra already has `sim/eval_grasp.py` with fixed seeds; extend it with the metrics below and a JSON baseline file.
- Metrics [M, standard definitions]:
  - success rate with CI;
  - jerk RMS (third difference of joint angle / dt^3, per joint, over reach+hold);
  - rest jitter: std dev (deg) and peak-to-peak of each joint over a 10 s still window (target e.g. < 0.2 deg std in sim, < 1 deg on hardware = about dead zone);
  - time to settle (time until |q - q_target| < 2% and stays);
  - action rate (mean |a_t - a_{t-1}|), tracking error, max load/temp, number of trips of safety filters;
  - hold test: grasp then hold 30 s with no motion allowed beyond X; count drops.
- Hardware test protocol [M]: idle test (powered, no command, 1 h, log drift and temp); repeatability (go to pose A/B 100-1000 times, measure return error); endurance cycles (ORCA reports >10,000 cycles as reference) [S: arXiv 2504.04259]; thermal soak at max grip; power-loss/bus-glitch injection; 100+ trials per object with random placement.

## 8. Concrete next-step list for Tendra (priority order)
1. In sim: add per-joint setpoint filter/jerk limiter on the 4 ms tick between policy and actuators (both CPU and GPU envs), retrain.
2. Add actuator model (delay 1-3 physics frames, quantisation, deadband, rate limit) and randomise kp/delay.
3. Add CAPS-style smoothness loss + action-rate penalty (if not already).
4. Add phase state machine (reach/grasp/hold/release) with target latch in hold; log its decisions.
5. Firmware: command gating (deadband + hysteresis), rate limit, load/temp monitoring, watchdog; set SCS0009 dead zone 1-3 units, moderate P; verify acceleration register (41) experimentally.
6. Build the hardware idle/repeatability tests and the sim regression metrics (rest jitter, jerk RMS, settle time).
7. Later: gripper stiffness as one policy output; load-based slip proxy; actuator network from real logs.

## Not found / uncertain (honest list)
- Official SCS0009 memory table PDF not parsed (acceleration reg, torque-limit address, default P/D, exact dead-zone units unconfirmed).
- Hwangbo actuator net specifics and OpenAI EMA numbers come from search summaries, not the full texts.
- InMoov handling of jitter not researched.
- Ruckig on ESP32 feasibility not verified.
