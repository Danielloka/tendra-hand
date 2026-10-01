# Tendra Hand — Roadmap

## The ultimate goal

**A robot that can do what humans can do, and do it just as well:** cook a meal, fold laundry, load a dishwasher, tidy a room, use everyday tools. Not only picking things up, but *understanding* a task (what needs to happen, in what order, and why) and carrying it out with human-level skill.

**The hand is the focus.** Almost every useful thing people do goes through their hands, and dexterous hands are the hardest unsolved part of general-purpose robots. So we start there: a hand as capable as a human one, then the intelligence to use it, then the body to take it into real homes.

The path, from the bottom up:

1. **Hand** (Phases 0–3): the V0 prototype, then **Tendra Hand V1**, a human-like hand (20 joints on 16 servos) that moves smoothly and precisely.
2. **Senses** (Phase 4): touch and vision, so the hand knows what it's holding and how hard.
3. **Skills** (Phases 5–6): learned dexterous skills — grasping, in-hand manipulation, tool use.
4. **Understanding** (Phase 7): AI that turns an instruction like "make a sandwich" into a plan and the right skills.
5. **Body** (Phase 8): the hand on an arm and then a mobile robot, doing real chores in real homes.

Each phase has a clear **"done when"** so progress is measurable. Phases can overlap. The open-source track runs the whole time.

---

## Phase 0: Foundation ✅ in progress
Set up the project so everything after it is easy.

- [x] Project structure, CLAUDE.md, READMEs
- [x] Inspect the Fusion URDF export and document its problems
- [x] Git + public GitHub repo, licenses
- [x] Python environment (uv, Python 3.12, MuJoCo)
- [x] Converter: Fusion export → MuJoCo MJCF (PLA masses, valid inertias, joint names, closing-positive)
- [x] MuJoCo viewer with a slider per joint
- [x] Owner checks the sim motions against the real hand (confirmed 2026-09-26)

**Done when:** the hand model loads in MuJoCo with correct joint limits and can be posed with sliders.

## Phase 1: V0 prototype — thumb + index on steppers
Just a start: make the 8-joint prototype move cleanly and control it from the PC, to learn the basics before V1.

- [x] Firmware skeleton in PlatformIO; all motor pins LOW at boot, PSRAM disabled
- [x] Motor **hardware abstraction layer** (`MotorDriver` interface) + `Uln2003Stepper`
- [x] Smooth motion: acceleration/deceleration ramps, non-blocking stepping of all 8 motors at once (tested on PC; hardware test pending)
- [x] Coil release when idle (power budget: 5 V / 2 A)
- [x] PC ↔ ESP32 serial protocol, firmware side (text v0.1)
- [x] PC-side serial client (`tendra.RealHand`, tested against a software ESP32)
- [x] Python `Hand` API with `RealHand` and `SimHand` backends
- [ ] Per-joint calibration: steps per radian, direction, limits
- [x] **Digital twin** software: control the real hand from the MuJoCo sliders (`sim/twin.py`)
- [ ] Hardware bring-up: first power-on checklist, direction check per motor
- [ ] Basic motions: open/close, pinch, point

**Done when:** all 8 joints move smoothly (no jerks, no missed steps) through their full range, and the sim and the real hand match within a few degrees.

## Phase 2: Tendra Hand V1 — the first full hand
Five fingers, 20 joints (4-DOF thumb) on 16 Feetech SCS0009 smart servos (position feedback, no manual homing, one shared bus); each finger's fingertip joint follows its middle joint through a coupling tendon, as in Shadow, ORCA and LEAP. This is the first real Tendra Hand.

- [x] Middle, ring and little fingers (reuse the index design, with sizes scaled to human proportions) — designed in Fusion "Tendra Hand V1" (2026-09-28)
- [x] Thumb base reworked for PTFE-sheathed tendons (hollow pivot); 5th thumb DOF tried and removed again (owner, 2026-09-29)
- [x] Actuator packaging (forearm), tendon routing through the wrist: one channel per strand, tested (`hardware/cad/tendon_router.py`)
- [x] Full-hand MJCF (20 joints, 16 servos, 40 tendons incl. DIP coupling): `sim/models/tendra_hand_v1.xml`
- [x] `Scs0009Servo` driver behind the same HAL (joint code unchanged), firmware env `hand_v1_servo`
- [ ] Full-hand electronics and power budget on the servo bus (plan in `research/experiments/2026-09-28-full-hand/research.md`)
- [ ] Wire the servo bus: ESP32 UART → FE-URT-1 → daisy-chained SCS0009s; assign IDs 1–20
- [ ] Mechanical adapters: servo mounts and tendon spools
- [ ] Test channels and PTFE fit on a printed section first, then print and assemble V1
- [ ] Read back position, load and temperature on the real hand
- [ ] **Digital twin mirror:** sim follows the *measured* joint positions (real → sim), not only the commanded ones
- [ ] Compare steppers (V0) vs. servos (V1): speed, precision, force, noise → research log

**Done when:** all five fingers are actuated and controllable from the PC with the same Python API, and the twin shows measured positions live.

## Phase 3: Mechanics, tendons and system identification
Make V1 robust and the simulation realistic.

- [ ] Tendon friction and pre-tension study on the real hand; tune the MuJoCo tendons
- [ ] Convex decomposition of the palm (and other concave parts) for accurate contact
- [ ] TPU fingertip pads
- [ ] **System identification:** measure real speed, friction and backlash, and tune the sim to match
- [ ] Measure the real mass of printed parts and update the model

**Done when:** simulated and real joint motions match closely under the same commands, and the hand survives long test runs.

## Phase 4: Sensing
- [ ] Fingertip touch/force sensing
- [ ] Camera (hand-eye setup) for vision
- [ ] Sensor data streamed into the Python API and the sim

## Phase 5: AI control
Detailed AI research plan for Phases 5–8 (architecture, stages, maths, resources, ideas): `research/ai/`.

- [ ] **Teleoperation:** control the hand by moving your own hand in front of a webcam (hand tracking). Great for demos and for collecting data. Software done for the sim (`sim/teleop.py`, 2026-09-28); live test and real hand pending.
- [ ] Imitation learning from recorded demonstrations
- [ ] Reinforcement learning in MuJoCo with domain randomization, then **sim-to-real** transfer (on cloud GPUs, since the dev laptop has no NVIDIA GPU)
- [ ] Optional: move to Isaac Sim/Lab for large-scale training

## Phase 6: Vision-based grasping and complex tasks
- [ ] Detect objects with the camera and plan grasps
- [ ] Complex tasks: in-hand rotation, tool use, handling everyday objects
- [ ] Publish benchmarks and results

**Done when:** the hand reliably does a set of benchmark tasks with everyday objects (e.g. open a jar, use scissors, rotate a cube in hand).

## Phase 7: Task understanding
Go from "do this motion" to "do this job".

- [ ] Vision-language-action (VLA) model: camera image + spoken or written instruction → hand actions
- [ ] Task planning: break a long task ("make a sandwich") into steps and pick a learned skill for each
- [ ] Recover from mistakes: notice when a step failed (dropped object, slipped grip) and try again
- [ ] Learn new tasks from a few human demonstrations
- [ ] Open dataset of hand demonstrations for household tasks

**Done when:** given a plain-language instruction, the hand plans and completes multi-step tabletop tasks it hasn't been scripted for.

## Phase 8: Arm, body and real chores
Take the hand out of the lab and into a home.

- [ ] First body: a **bimanual station**: two 7-DOF arms with two Tendra hands on a fixed pole (optional linear lift), head + wrist cameras (`research/ai/vision.md`)
- [ ] Mount the hand on a robot arm (wrist with 2–3 DOF), two hands for bimanual tasks
- [ ] Kitchen tasks: cut, stir, pour, crack an egg, cook a simple meal
- [ ] Household chores: fold laundry, load a dishwasher, tidy up
- [ ] Mobile base / humanoid body to move between rooms
- [ ] Safety around people: force limits, soft contact, stop on unexpected touch

**Done when:** the robot does everyday chores in a real home as well as a person.

---

## Open-source track (all phases)
- [ ] Public GitHub repo, licenses, contribution guide
- [ ] Project website with downloads (CAD, STL, firmware, models), docs and research
- [ ] Build guide + bill of materials (BOM) with costs
- [ ] Tagged releases for every hardware/software milestone
- [ ] Demo videos
