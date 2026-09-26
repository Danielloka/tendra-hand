# Tendra Hand — Roadmap

From a thumb and index finger that move smoothly, to a full human-like hand with AI control.
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

## Phase 1: Thumb + index moving smoothly (steppers)
Make the current 8-joint prototype move cleanly, and control it from the PC.

- [x] Firmware skeleton in PlatformIO; all motor pins LOW at boot, PSRAM disabled
- [x] Motor **hardware abstraction layer** (`MotorDriver` interface) + `Uln2003Stepper`
- [x] Smooth motion: acceleration/deceleration ramps, non-blocking stepping of all 8 motors at once (tested on PC; hardware test pending)
- [x] Coil release when idle (power budget: 5 V / 2 A)
- [x] PC ↔ ESP32 serial protocol, firmware side (text v0.1)
- [ ] PC-side serial client
- [ ] Python `Hand` API with `RealHand` and `SimHand` backends
- [ ] Per-joint calibration: steps per radian, direction, limits
- [ ] **Digital twin v1:** sim mirrors the commanded pose. Control the real hand from the MuJoCo sliders.
- [ ] Basic motions: open/close, pinch, point

**Done when:** all 8 joints move smoothly (no jerks, no missed steps) through their full range, and the sim and the real hand match within a few degrees.

## Phase 2: Stepper → SCS0009 smart servo upgrade
Get position feedback, frees up GPIO pins, and removes manual homing.

- [ ] Wire the servo bus: ESP32 UART → FE-URT-1 → daisy-chained SCS0009s; 5 V power sized for stall current
- [ ] Assign servo IDs; `Scs0009ServoDriver` behind the same HAL (joint code unchanged)
- [ ] Read back position, load and temperature
- [ ] Mechanical adapters: servo mounts and tendon spools
- [ ] **Digital twin v2:** sim follows the *measured* joint positions (real → sim), not only the commanded ones
- [ ] Compare steppers vs. servos (speed, precision, force, noise) → research log

**Done when:** the thumb and index run on servos with the same Python API, and the twin shows measured positions live.

## Phase 3: Mechanics, tendons and system identification
Make the hardware robust and the simulation realistic.

- [ ] Tendon routing, friction and pre-tension study; model tendons in MuJoCo
- [ ] Convex decomposition of the palm (and other concave parts) for accurate contact
- [ ] TPU fingertip pads
- [ ] **System identification:** measure real speed, friction and backlash, and tune the sim to match
- [ ] Redesign the thumb toward 5 DOF (human-like opposition)
- [ ] Measure the real mass of printed parts and update the model

**Done when:** simulated and real joint motions match closely under the same commands, and the hand survives long test runs.

## Phase 4: Full hand
Scale to five fingers.

- [ ] Middle, ring and little fingers (reuse the index design, with sizes scaled to human proportions)
- [ ] Actuator packaging (palm vs. forearm), tendon routing through the wrist
- [ ] Full-hand electronics and power budget on the servo bus
- [ ] Full-hand URDF/MJCF (~20+ DOF)

**Done when:** all five fingers are actuated and controllable from the PC and in the sim.

## Phase 5: Sensing
- [ ] Fingertip touch/force sensing
- [ ] Camera (hand-eye setup) for vision
- [ ] Sensor data streamed into the Python API and the sim

## Phase 6: AI control
- [ ] **Teleoperation:** control the hand by moving your own hand in front of a webcam (hand tracking). Great for demos and for collecting data.
- [ ] Imitation learning from recorded demonstrations
- [ ] Reinforcement learning in MuJoCo with domain randomization, then **sim-to-real** transfer (on cloud GPUs, since the dev laptop has no NVIDIA GPU)
- [ ] Optional: move to Isaac Sim/Lab for large-scale training

## Phase 7: Vision-based grasping and complex tasks
- [ ] Detect objects with the camera and plan grasps
- [ ] Complex tasks: in-hand rotation, tool use, handling everyday objects
- [ ] Publish benchmarks and results

---

## Open-source track (all phases)
- [ ] Public GitHub repo, licenses, contribution guide
- [ ] Project website with downloads (CAD, STL, firmware, models), docs and research
- [ ] Build guide + bill of materials (BOM) with costs
- [ ] Tagged releases for every hardware/software milestone
- [ ] Demo videos
