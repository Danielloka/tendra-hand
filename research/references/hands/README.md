# State of the art in robot hands: overview and plan for Tendra

Research pass of 2026-09-30. Six research agents covered one topic each. A fact-check pass then verified the key numbers against datasheets, papers and repos. The detailed reports, each with sources and a confidence tag on every claim:

| Report | What it covers |
|---|---|
| [1x-neo.md](1x-neo.md) | 1X NEO hand (July 2026): 22 + 3 DOF, forearm motors, tendons at 5–15:1, force control, tactile skin; Halodi's tendon patent |
| [humanoid-company-hands.md](humanoid-company-hands.md) | Tesla Optimus V3, Figure 03, Atlas, Sanctuary, Apptronik, Unitree, Fourier, Xpeng, Xiaomi, Clone, Agibot and more, plus trends |
| [dexterous-and-open-source-hands.md](dexterous-and-open-source-hands.md) | ORCA, LEAP, Amazing Hand, RUKA, Aero Hand Open, DexHand, InMoov; Shadow, Sharpa, Inspire, PSYONIC, Wuji and more |
| [tendon-mechanics.md](tendon-mechanics.md) | Tendon material, capstan friction, pretension and tensioners, force budget, joints, compliance, print materials, bench tests |
| [actuators-and-control.md](actuators-and-control.md) | Servo options, why hobby-servo hands are jerky and how to fix it, force control on position servos, teleop, imitation learning, VLAs, sim-to-real |
| [sensing.md](sensing.md) | Tactile tech (magnetic, barometric, vision), joint-angle sensing, tendon force, a staged sensing plan |

Confidence: numbers in **bold** below were checked against a primary source on 2026-09-30. Everything else is flagged in the detailed report.

---

## 1. The big picture in five sentences

1. **Tendra V1 already follows the industry's layout.** Motors in the forearm pull tendons to the fingers. That's what 1X NEO, Tesla Optimus V3 and Xiaomi's new hand do. 1X's founding patent (Halodi, WO2018149499A1) even uses Tendra's exact scheme: **two opposite-wound cables per joint on one motor**. So does the Shadow Dexterous Hand: **20 forearm motors, 40 Spectra (UHMWPE) tendons, one agonist–antagonist pair per spool**.
2. **The big gap is strength.** Tendra V1 manages about **3.4 N** at the fingertip at stall (about 1.7 N sustained). A human tip pinch is about **80 N** (men) or **50 N** (women). Commercial hands reach **9–20 N** (PSYONIC pinch 9.3, Inspire 10, Wuji 15, Sharpa Wave 20), 1X NEO reaches **45 N**, and open research hands reach 3–14 N.
3. **Smoothness is mostly fixable with firmware and cheap parts**: a higher goal-stream rate, jerk limiting, stiffer tendons, and low-friction contact points.
4. **Softness comes next, done in software.** Torque limits, contact detection and admittance control work now. True force control needs a servo that measures current (the HLS3606M).
5. **Touch and data are where the leaders pull ahead.** Every flagship hand now has fingertip tactile sensing, most have palm cameras, and all of them are trained on teleop data.

## 2. Where Tendra V1 stands

| | Tendra V1 (designed) | Open-source peers | 1X NEO | Human |
|---|---|---|---|---|
| Actuated DOF (hand) | 20 | 16–17 (ORCA, LEAP), 16 joints on 7 motors (Aero Hand Open) | 22 (+3 wrist) | ~21 |
| Actuator | SCS0009, **0.226 N·m** at 6 V, pot sensor (**100 k cycle** life) | XC330 (**0.92 N·m**), XL330 (**0.52 N·m**), HLS3606M (**0.59 N·m**) | Own low-ratio motors, backdrivable | – |
| Knuckle (MCP) torque | ~0.20 N·m | ~0.5–0.8 N·m | **2.6 N·m** | ~2–4 N·m (estimate) |
| Fingertip force | ~3.4 N stall | RUKA **2.74 N** pinch, Aero **~12 N** per finger | **45 N** | **~50–80 N** tip pinch |
| Tendon | 0.4 mm nylon mono | ORCA **0.4 mm braided**, RUKA braid, Aero **Kevlar/Vectran** | Polymer (in-house) | – |
| Force sense | Servo "load" (PWM duty, not current) | Current (Dynamixel/HLS), FSR tips (ORCA) | Motor current at every joint + tactile skin | Skin + muscle |
| DIP joint | Independent | **Coupled to PIP** (ORCA, LEAP v2 Adv, Tesla, Shadow) | Not published | Mostly coupled |

Fingertip force for Tendra, from `tendon-mechanics.md` §4 (MCP-limited, 60 mm pinch lever, 90 % transmission efficiency):

| Drive | Tendon force | MCP torque | Tip force (stall / hold) |
|---|---|---|---|
| SCS0009, spool 6 / drum 6 mm (now) | 38 N | 0.20 N·m | 3.4 / 1.7 N |
| SCS0009, spool 5 / drum 7.5 mm | 45 N | 0.31 N·m | 5.1 / 2.5 N |
| SCS2332 (same protocol), 5 / 7.5 | 88 N | 0.59 N·m | 9.9 / 5.0 N |
| **HLS3606M, 5 / 7.5** | 118 N | 0.80 N·m | **13.3 / 6.6 N** |

⚠ At 118 N the 50 lb braid (break ≈ 222 N) only has a safety factor of ~1.9. Use **80 lb braid (0.43–0.46 mm)** on HLS joints, or cap the torque limit at ~60 %. The printed drums and tie holes must also be tested at that load.

## 3. What the best hands do, and what Tendra takes from each

| Idea | Who does it | Take it for Tendra? |
|---|---|---|
| Motors in the forearm, tendons through the wrist | 1X, Tesla V3, Xiaomi | ✅ already done |
| Tendons cross joints on their axis, so moving one joint doesn't pull another's tendon | Tesla V3 (at the wrist), Tendra (fingers) | ✅ already done; do the same for the future wrist |
| Low gear ratio + current sensing = the motor itself senses force, and fingers can be pushed back | 1X (5–15:1), Unitree Dex5-S | ⏳ partly: HLS3606M current mode now, one BLDC test joint later |
| Braided polymer tendons (UHMWPE, Vectran, Kevlar) in lubricated sheaths | 1X, Tesla, ORCA, Aero, Halodi patent (Vectran) | ✅ do now |
| DIP coupled to PIP | ORCA, LEAP v2 Adv, Tesla, Shadow | ❓ owner's decision (§5) |
| Springs to extend instead of a second strand | Tesla V3 | ❌ keep the antagonistic loops (stiffer, can push). Maybe a *soft extensor strand* as a self-tensioner |
| Rolling-contact joints | Tesla patent (Musk, April 2026: "didn't actually work", replaced) | ❌ keep pin joints with steel pins or bearings |
| Joints that pop out under overload | ORCA | ✅ V1.x: cheap insurance for gears and prints |
| Self-calibration by driving to hard stops | ORCA | ✅ firmware, uses servo load |
| Ratchet spools for tensioning | ORCA | ✅ split spool with friction clamp, or ratchet |
| Fingertip touch (magnetic, barometric, camera-based) | 1X, Figure, Sharpa, Sanctuary, Shadow DEX-EE | ✅ magnetic tips (AnySkin-style), stage 1–2 |
| Palm camera | Figure 03, Boston Dynamics Atlas | ✅ cheap USB camera, stage 2 |
| Durability as a spec (cycles) | 1X (millions), Xiaomi (150 k grasps), ORCA (10 k) | ✅ cycle-test rig on V0 |
| Fast learned low-level control (1 kHz) under a slow planner | Figure Helix 02 | Later; the layers are already there (PC policy → ESP32 → servo loop) |

## 4. The plan: prioritised actions

Ordered by impact per euro and hour. "Free" means firmware or a print change.

### A. Do now (V0 and firmware; nothing to decide)

1. **Firmware smoothness (free).** Details in `actuators-and-control.md` §2.4. Verified in our code: goals go out at 50 Hz with a fixed goal speed of 600 (`config_v1.h`), and each servo's feedback is read only every 40 ms.
   - Stream goals at 100 Hz, and set each servo's goal speed from the ramp's current speed. This removes the stop–go "staircase".
   - Add jerk limiting: a 2nd-order filter after the trapezoid.
   - Add a timestamped streaming command that the ESP32 interpolates, for teleop and policies.
   - Read positions only at ~100 Hz, and load/temperature at a lower rate.
2. **Servo register tool.** Read and log the factory P/D/I, deadband, punch and torque-limit values (registers 16, 21–27). Then tune per joint type: lower P and deadband, set torque limits. The LeRobot community's main fix for jerky Feetech servos is a lower P.
3. **Soft grasp in firmware.** Add a load watchdog and contact flags in the `F` line, and a "close until contact, then squeeze by Δ" grasp mode.
4. **Tendon swap on V0:** braided UHMWPE, **50 lb = 0.36 mm** (PowerPro or Sufix 832), pre-stretched, with 1–1.5 wraps round a post before each knot. It stretches about 1 % at a third of its break load, where nylon mono stretches 2–9 %. That stretch is probably part of why the V0 index "extends hard and grips weakly".
5. **Bench tests, an afternoon each** (`tendon-mechanics.md` §8):
   - capstan friction (μ)
   - line stretch and creep
   - fingertip force vs servo command
   - a cycle test (thousands of flex/extend cycles, logging drift)

### B. Before building V1 (design changes; the owner decides)

6. **The tendon touches only PTFE, polished steel or its own drum.** Put steel dowels or brass eyelets at every bare bend > 20°. This roughly doubles the force reaching the DIP in a curled finger (~30 % → ~65 % efficiency).
7. **Pins:** steel dowels or small bearings at the finger joints. Print the phalanges on their side. Use PETG, not PLA, for structure.
8. **Spool support:** add a bushing or bearing opposite each servo. The two strands pull the servo shaft sideways with up to ~43 N; pretension doesn't use up servo torque, but it does load the shaft.
9. **Tensioners:** a split spool with a friction clamp or ratchet, plus witness marks. Pretension 3–5 N, checked by pluck pitch.
10. **Silicone or soft-TPU fingertip and palm pads.** They cut the pinch force needed to hold an object by ~3×, so they are effectively the cheapest strength upgrade.
11. **More knuckle leverage:** MCP drum radius 7.5–8 mm with a 5 mm spool (G ≈ 1.5), for ~1.5× fingertip force. This means updating `servo_per_joint`, the router, the sim and the tests.
12. **Stronger servos on the high-load joints.** Test **one HLS3606M** first:
    - It gives **0.59 N·m** with **current feedback and a constant-current mode**, in the same 23 × 12 mm outline as the SCS0009, but 2.25 mm taller and with a 25T spline.
    - Aero Hand Open has already proven it in a tendon hand.
    - It needs an `HlsServo` driver (its own memory table, not STS-compatible) and preferably its own bus.
    - Fallback: the **SCS2332**, with the same protocol as the SCS0009 and about 2× its torque.
    - Candidates: 4 × `mcp_flex`, `thumb_cmc_flex`, `thumb_cmc_rot`.
    - Power at **6 V** for +22 % torque.

### C. Next months (sensing and learning)

13. **Sensing stage 1:** calibrate servo load against tendon force with hanging weights. Build one magnetic fingertip (MLX90393 or TMAG5273 under a silicone dome with a magnet, on I²C). Add touch sensors to `GraspEnv`.
14. **Sensing stage 2:**
    - Magnetic tips on all five fingers.
    - Hall-effect joint-angle sensors at the MCPs and the thumb base.
    - A small MCU in the palm with an I²C mux, talking to the ESP32 over one UART through the wrist.
    - A USB palm camera.
    - Once real joint angles are known, spool angle minus joint angle gives tendon stretch, and from that tendon force.
15. **Actuator model in the sim.** Add servo latency, deadband, backlash, the P-gain spring and the torque limit to the V1 MuJoCo model, then randomise them in RL. Every successful sim-to-real hand result relied on this.
16. **Data pipeline:** a LeRobot `Robot` class over `RealHand`. Then train ACT first, Diffusion Policy second and SmolVLA third. Port `GraspEnv` to MJX/MuJoCo Playground for cloud-GPU RL.
17. **Tendra kinematic-twin glove.** DexUMI (2025) and DOGlove (< $600, with force feedback) show the idea works. It's also a possible product.

### D. V2 direction (research)

18. **One BLDC + FOC test joint** (gimbal motor, SimpleFOC, a 5–10:1 capstan) on the index MCP. It lets us feel 1X-style backdrivable force control, and compare it with HLS current mode on force accuracy, heat and cost.
19. Leave room in the forearm for a 20 × 34 mm servo class (XL330/XC330), or for all-HLS.
20. A 2–3 DOF wrist for the bimanual station, with tendons routed through the wrist axes, as in Tesla V3.

## 5. Decisions for the owner

These change the robot design, so they are the owner's call:

1. **Stronger servos on 6 high-load joints (HLS3606M, ~US$30 each), or keep all 20 SCS0009 for V1?** Recommendation: buy **one or two HLS3606M** now and test them on the V0 index MCP rig before committing.
2. **Couple the DIP to the PIP?** Most successful hands do it, and it frees 4 servos (20 → 16) whose budget, space and bus time could go to stronger knuckles. The cost: less independent fingertip control, which humans barely have anyway. Recommendation: try it in the sim first (`test_v1.py` + grasp RL).
3. **MCP gear ratio 1.5 (drum 7.5 mm, spool 5 mm)?** Recommendation: yes. It's cheap strength and keeps enough speed (~400°/s with no load).

## 6. Still open

- HLS3606M mounting-hole positions, and whether the STS3032 reports current.
- Real friction coefficients for our line in our PTFE tubes (bench test 1).
- Figure 03 hand DOF (not officially published), Atlas production hand DOF, Tesla's actuator type.
- 1X hand weight, finger layout and speed are not published.
