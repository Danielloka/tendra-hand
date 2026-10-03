# Actuators and control: how the best hands move smoothly, softly and strongly (and how Tendra gets closer)

Date: 2026-09-30. Scope: actuator choice, low-level control, force/compliance, and learning-based dexterity, applied to **Tendra Hand V1** (16 SCS0009 in the forearm, finger DIPs linked to PIPs, antagonistic tendon loops, ESP32-S3 + FE-URT-1, PC does high-level control).

**How to read the confidence labels**

| Label | Meaning |
|---|---|
| ✅ **confirmed** | From a datasheet, the official memory table/library, or our own code, and checked in this repo (mostly `research/experiments/2026-09-28-full-hand/research.md`, which read the SCS0009 datasheet directly) or in a web search result during this session |
| 🟡 **published, not re-checked** | Widely reported in papers or vendor pages up to mid-2026, recalled from reading, not re-fetched today. Check the number before you buy or cite it |
| 🔴 **speculative** | My inference or an unconfirmed report. Treat as a hypothesis to test |

> Note on this session: the web tools failed for part of the session (a safety-check service timed out), so about 25 searches/fetches were done instead of the planned 30+. Everything that could not be re-checked is labelled 🟡 or 🔴 instead of being presented as fact.

---

## 0. The short version

1. **Your jerkiness risk is mostly in the firmware, not in the servo.** Today the ESP32 ramps each goal one tick (0.29°) at a time but only sends it every **20 ms**, and the servo moves to each small goal at a fixed goal speed of 600 and then stops. That makes a "staircase", and the servo's deadband adds stick-slip at slow speeds. Fix it by sending goals at **100 Hz**, making the servo's **goal speed follow the ramp's current speed** (or use **goal time**), and using a **jerk-limited (S-curve) profile or setpoint interpolation**. This costs nothing in hardware.
2. **The SCS0009 is weak for the MCP joints and the thumb.** Its stall torque is 0.226 N·m at 6 V, and it only runs "safely" at ≤ 80 % of that. Two stronger servos fit the **same 23 × 12 mm footprint** (2–3 mm taller):
   - **Feetech HLS3606M** (the best fit): 0.59 N·m, 1:205 coreless, magnetic sensor, **current feedback and a constant-current (torque) mode**, 20.6 g, ~US$30. It needs a new firmware driver (HLS memory table) and **new spools (25T / 4.95 mm spline instead of 20T / 3.95 mm)**. Already used in a tendon hand (Aero Hand Open, 7 × HLS3606M).
   - **Feetech SCS2332** (the easiest swap): ~0.44 N·m, metal gears, **same SCS protocol**, so no firmware change. Still no current sensing.
   Use one of them on the 4 `mcp_flex` joints and the thumb CMC joints first. Test one HLS3606M on `index_mcp_flex` before buying more.
3. **"Soft" control without a torque sensor:** SCS0009 has **no current sensor**. Its "load" register is the **motor PWM duty**. You can still do good compliant grasping by (a) lowering the servo's **P gain** and **torque limit** (registers 21, 16), (b) running an **admittance loop** on the ESP32 that moves the goal away when the load reading rises, and (c) grasping by "close until load > threshold, then squeeze by a fixed offset". With HLS servos, a real **current limit / torque mode** replaces most of this, as LEAP and ORCA do with current-limited servos.
4. **The state of the art (2025–2026):** the leaders mostly use **tendons with motors in the forearm** (Tesla Optimus Gen 3: 25 actuators per forearm for a 22-DOF hand; the **1X NEO hand, July 2026: 22 + 3 DOF on quasi-direct-drive tendons at 5–15:1, all force-controlled and backdrivable**). They add **force sensing through low gear ratios or current**, plus **tactile skins** (Figure 03, Sharpa Wave, 1X), and run **high-rate low-level control (500 Hz – 1 kHz; Figure's learned "System 0" runs at 1 kHz)** under a learned policy at 7–200 Hz. Tendra's architecture (PC policy → ESP32 → smart servos) is the right shape. The weak points are the actuator's sensing and the loop rate.
5. **Learning path that fits a CPU laptop + rented GPUs:** MuJoCo RL on CPU now → MuJoCo Playground/MJX on a cloud GPU for in-hand RL → **ACT / Diffusion Policy** from teleop demos (hours on one GPU) → **SmolVLA** or **π0/π0.5 LoRA fine-tune** (one 24–80 GB cloud GPU) later. For data: webcam now → **Quest 3 hand tracking** → the **Tendra kinematic-twin glove** (idea #1 in `research/ai/ideas.md`), which is the same concept as DexUMI and DOGlove.

---

## 1. Actuator options

### 1.1 Key words first

- **Stall torque:** the most torque the motor makes when blocked. It can't hold this for long (it overheats). Plan on **~30 % of stall continuously**.
- **Gear ratio:** a small fast motor turns a big slow output. A high ratio (SCS0009: 1:416) gives torque from a tiny motor but adds **friction and backlash** and makes the output **hard to backdrive** (you can't push it by hand). A low ratio (≤ 1:30, "quasi-direct drive") is **backdrivable**: a push on the finger shows up as current in the motor, so the motor "feels" contact without a force sensor. That is called **torque transparency**.
- **Current/torque sensing:** motor torque ≈ k_t × current. A servo that measures current can do **current-based position control** (Dynamixel calls it that): it goes to a position but never pushes harder than a set current. This is the easiest route to a soft, safe hand.
- **Position sensor:** a **potentiometer** (a resistive strip, wears out, noisy) or a **magnetic encoder** (contactless, 12-bit = 4096 counts/turn, no wear).

### 1.2 Comparison table

Torque in N·m (1 kg·cm = 0.098 N·m). Prices are rough single-unit retail in USD, 2025–2026.

| Actuator | Stall torque | No-load speed | Weight | Size (mm) | Price | Position sensor | Protocol | Current/torque sensing | Gear | Backdrivable | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **Feetech SCS0009** (Tendra V1 now) | 0.185 @ 4.8 V, **0.226 @ 6 V** (1.89 / 2.3 kg·cm); rated 0.064 / 0.074; stall current 0.8 / 1.0 A | 0.125 s/60° @ 4.8 V, 0.10 s/60° @ 6 V (100 rpm) | 13.2 g | 23.2 × 12.1 × 25.25 | ~10–15 | 10-bit, 300°, **carbon-film potentiometer, life 100 000 cycles min** (datasheet; some shops say "magnetic", they are wrong) | SCS TTL, 1 Mbps, big-endian, **no sync read** | **No.** "Load" = PWM duty ‰ | 1:416, **copper + steel gears**, PC (plastic) case, iron-core motor, backlash ≤ 0.5°; horn 20T / OD 3.95 mm; input 4–7.4 V | No | ✅ Feetech datasheet A/0 re-read 2026-09-30 |
| **Feetech SCS2332** | ~0.39 @ 4.8 V, **0.44 @ 6 V** (4.5 kg·cm); rated 0.15; stall 1.2 A @ 6 V | 0.09 s/60° @ 6 V | 20 g | 23.2 × 12.1 × 28.5 | ~15–25 | 10-bit **potentiometer**, 300° | SCS TTL, 38.4 k–1 Mbps (same family as SCS0009) | Feedback list = load, position, speed; **no current** (SCS series) | Metal gears, **coreless** motor, 2 ball bearings | No | ✅ shop listing (aifitlab) 2026-09-30. Note: some listings (Kamami/Alibaba: 23 × 11.5 × 24 mm, 8.6 g, PWM) describe a different product under the same name; check before buying |
| **Feetech STS3032** | **0.44 @ 6 V** (4.5 kg·cm); stall 1.2 A @ 6 V | 0.09 s/60° @ 6 V | **20 g** (Feetech page; some shops say ~25 g) | **23 × 12 × 27.5** (shops; Feetech's own page prints "32 × 12 × 27.5", probably a typo) | ~15–25 | **12-bit magnetic**, 360°, multi-turn | **STS** TTL, 38.4 k–1 Mbps (little-endian, has sync read 🟡) | Official datasheet (A/0, 2020-06-08) lists load, position, speed, voltage, temperature, **not current**; shop listings claim current feedback at 6.5 mA/unit 🟡. **Modes: 0 position, 1 closed-loop speed, 2 open-loop speed, 3 step; no constant-current (force) mode** ✅. Overload protection is load-based (> 80 % of stall for 2 s) | **1:205** (datasheet; the shop's 1:345 is wrong), copper + steel gears, **coreless** motor, **aluminium case**, 20.6 g, **horn 25T / OD 4.95 mm**: the same mechanical numbers as the HLS3606M | No | ✅ Feetech product page 2026-09-30; datasheet A/0 read 2026-10-01 |
| **Feetech HLS3606M** ("constant force" servo) | 0.47 @ 4.8 V, **0.59 @ 6 V** (4.8 / 6 kg·cm); rated 0.12 / 0.15; stall current 1.1 / 1.3 A; Kt 4.5 kg·cm/A; terminal resistance 2.8 Ω | 0.11 s/60° @ 4.8 V, 0.09 s/60° (111 rpm) @ 6 V | 20.6 g | **23 × 12 × 27.5** (same plan as SCS0009, 2.25 mm taller; mounting-ear hole positions not checked) | **~30** (Aero Hand Open BOM: 7 servos ≈ US$209) | **12-bit magnetic**, 360°, multi-turn ±7 turns (turn count lost at power-off) | **HLS** TTL, 38.4 k–1 Mbps, own memory table (`HLSCL` class; **not** the STS/`SMS_STS` table) | **Yes: current feedback and modes 0 = position, 1 = constant speed, 2 = constant current ("constant force", target torque in register 44).** Goal current = register 44, 6.5 mA/unit; protection current = register 28 (ORCA hand code) | **1:205**, copper + steel gears, **coreless**, ball bearings, **aluminium case**; **horn 25T / OD 4.95 mm (not the SCS0009's 20T / 3.95 mm, so the spools must change)**; input 4.5–7.4 V; life > 100 000 cycles at 1.5 kg·cm | Partly (lowest ratio of the micro servos) | ✅ **Feetech datasheet HLS3606M A/0 (2024-04-22)**, read 2026-09-30 |
| **Feetech STS3215** (SO-100/SO-101) | ~1.9 @ 7.4 V; **2.9 @ 12 V** (30 kg·cm) | 0.222 s/60° @ 12 V | 55 g | ~45 × 25 × 35 | ~15–20 | 12-bit magnetic, 360° | STS TTL, up to 1 Mbps | Reports load, current, voltage | 1:345 steel (other ratios: 1:147, 1:191) | Barely | ✅ (search) |
| **Feetech HLS, larger models** (**HLS3915** fingers + **HLS3930** wrist in the ORCA hand's Feetech variant) | larger than HLS3606M; HLS3915 stalls at ~1.5 A (orca_core comment) | – | – | standard size | ~30–60 | Magnetic | HLS TTL | Current mode, as above | Metal | No | ✅ model names from `orca_core/constants.py` (2026-09-30); torque not checked. (Earlier text said "HL-2915 class": wrong) |
| **Dynamixel XL330-M288-T** | 0.42 @ 3.7 V, **0.52 @ 5 V**, 0.60 @ 6 V (stall ~1.5 A @ 5 V 🟡) | 103 rpm @ 5 V | 18 g | 20 × 34 × 26 | **US$23.90** (robotis.us) | 12-bit contactless absolute, 360° | Dynamixel 2.0 TTL, up to 4 Mbps, sync + bulk read | **Yes: current-based position mode**, current mode, current feedback | 1:288.4 **plastic** (case too) | Poorly | ✅ ROBOTIS e-Manual, fetched 2026-09-30 |
| **Dynamixel XC330-M288-T** (LEAP Hand) | 0.69 @ 3.7 V, **0.93 @ 5 V (1.8 A)**, 1.10 @ 6 V (2.15 A) | **81 rpm** @ 5 V (not 65; 65 rpm is the XC330-T288 at 11.1 V) | 23 g | 20 × 34 × 26 | **US$89.90** (robotis.us; earlier "~50–60" was wrong) | 12-bit magnetic | Dynamixel 2.0 | **Yes**, same modes | 1:288.35 **metal**, 2 bearings | Poorly | ✅ ROBOTIS e-Manual + store, 2026-09-30 |
| **Dynamixel XC330-T288-T** (ORCA hand v1 fingers) | **0.92 @ 11.1 V** (~1.0 @ 12 V) | 65 rpm @ 11.1 V | 23 g | 20 × 34 × 26 | US$89.90 | 12-bit magnetic | Dynamixel 2.0 | Yes (goal current 1 mA/unit, limit 910 mA per orca_core) | metal | Poorly | ✅ robotis.us, orca_core |
| **Dynamixel XL430-W250-T** | ~1.4 @ 12 V | ~57 rpm | 57 g | 28.5 × 46.5 × 34 | ~50 | 12-bit magnetic | Dynamixel 2.0 | **No current sensing** (PWM only) | 1:258 plastic | No | 🟡 |
| **Waveshare / Hiwonder bus servos** (ST3215 = STS3215 rebadge; SC09 = SCS0009 rebadge; LX-16A) | as the Feetech original; LX-16A ~1.7 | – | – | – | often cheaper | as original; LX-16A pot | as original; LX-16A 115.2 kbps | as original; LX-16A none | – | No | 🟡 |
| **Gimbal BLDC + FOC** (e.g. 2204/2804 gimbal motor, SimpleFOC driver, AS5600/MT6701 encoder) | 0.05–0.15 direct; ~0.5–1.5 with a 1:10 planetary 🔴 | high | 20–40 g + driver | Ø 28 × 15 + driver | 15–40 per joint incl. driver | Magnetic (own) | Your choice (CAN/RS-485/UART) | **Yes** (from current = torque), **true torque control** | 1:1 to 1:10 | **Yes** | 🟡 concept; per-joint numbers depend on the motor |
| **Coreless DC + planetary + magnetic encoder** (Maxon/Faulhaber class, or Chinese 8–12 mm coreless) | depends; fingers use ~0.1–0.5 at output | high | 5–20 g | Ø 8–12 mm × 20–35 | 5 (Chinese) – 200 (Maxon) + driver | Magnetic incremental | Your own driver board | With a current-sense driver: yes | 1:16 – 1:256 | Partly (low ratio) | 🟡 |
| **N20 gear motor + hall encoder** | 0.1–1 (depends on ratio) | – | 10–12 g | 12 × 10 × 25–35 | 3–6 + H-bridge | Hall quadrature (relative, needs homing) | Your own | With a current-sense driver | 1:30–1:1000 **brass/steel spur, lots of backlash** | No (high ratios) | 🟡 |
| **Linear actuators** (Actuonix PQ12, L12; Inspire-style coreless + lead screw) | Actuonix PQ12: up to ~50 N; Inspire: ~10 N per finger fingertip 🟡 | slow (~10–30 mm/s) | 15–40 g | PQ12: 21.5 × 15 × 37 | Actuonix ~70; Inspire whole hand several k | Pot or encoder | PWM / RS-485 / CAN (Inspire) | Inspire: **force sensors per finger** 🟡 | Lead screw: **self-locking, not backdrivable** | No | 🟡 |

**What the table says for Tendra**

- The **SCS family** is the cheapest and smallest, but it is a "hobby-plus" servo: high gear ratio, potentiometer, a closed firmware loop with only P/D/I, and no current sensing.
- The **STS family** is the same idea but better sensing (magnetic, current, sync read). The **STS3032** has the SCS0009's 23 × 12 mm footprint (2.25 mm taller).
- **The most interesting find for Tendra: the Feetech HLS3606M.** It has the **same 23 × 12 mm footprint** as the SCS0009, **2.6× its torque** (0.59 N·m), a **lower gear ratio (1:205 vs 1:416)**, a coreless motor, magnetic sensing, **current feedback and a constant-current (torque) mode**. That is "Dynamixel-style" current control in a micro servo. **The HLS3606M itself is proven in a tendon hand: TetherIA's Aero Hand Open drives 16 joints with 7 × HLS3606M** (~12 N per finger, 374–400 g, US$314 BOM; arXiv 2608.28578, checked 2026-09-30). The ETH **ORCA hand** (2025, tendon-driven, 17 DOF) has a Feetech variant with the bigger **HLS3915** (fingers) and **HLS3930** (wrist) (`orca_core`). Caveats: the HLS3606M has a **25-tooth, 4.95 mm output spline** (SCS0009: 20T, 3.95 mm), so every spool needs a new bore, and its mounting-ear holes were not compared with the SCS0009's.
- **Dynamixel X-series** is what most research hands use (LEAP Hand: 16 × XC330-M288-T) because of **current-based position control**. It costs 2–4× more and is bigger.
- **BLDC + FOC** is the only affordable route to true torque control and backdrivability. It is also a lot of electronics per joint (20 drivers). It is how research "QDD" (quasi-direct-drive) fingers and several 2025 commercial hands get their feel.

### 1.3 What the leading hands use, and why

| Hand | DOF / actuators | Actuation | Why it matters | Confidence |
|---|---|---|---|---|
| **Shadow Dexterous Hand** | 24 joints, 20 actuated | **Tendons from forearm motors** ("Smart Motor" units with **tendon-force (strain-gauge) sensing**); coupled DIP–PIP on the fingers like humans | The classic research hand. Force sensing on each tendon gives torque control through a high-ratio gearbox | 🟡 |
| **Shadow DEX-EE** (with Google DeepMind, 2024) | 3 fingers × 4 DOF | Tendon-driven, lots of sensors, built for **robustness to thousands of hours of RL** | Shows that for learning, **durability and sensing matter more than human likeness** | 🟡 |
| **Tesla Optimus hand (Gen 2 → Gen 3)** | Gen 2: 11 DOF; Gen 3: **22 DOF hand + 3 wrist/forearm**, **25 actuators per forearm** | Actuators moved **into the forearm**; ~3 thin tendons per finger through the wrist ("wrist router" in patents) into channels in the phalanges. Motor/gear type not disclosed | Lower hand inertia → lower peak tendon tension, lighter and more durable hand, better heat removal. Same layout as Tendra V1 | ✅ layout (reports + patent analysis); actuator internals unknown |
| **1X NEO hand** (announced July 2026) | **25 DOF: 22 in fingers/palm + 3 wrist** | **Tendon-driven, motors in the forearm, "quasi-direct-drive" tendons at ~5:1–15:1 gear ratios.** All DOF force-controlled and fully backdrivable; tactile skin (normal force, location, shear) | Peak 3.5 N·m thumb CMC, 2.6 N·m finger MCP, up to 45 N distal force, ±0.2 mm, IP68. 1X's argument: 100:1–200:1 gearboxes make a hand "mechanically numb"; low ratios make **every joint a force sensor** | ✅ (1x.tech page) |
| **Figure 03 + Helix 02** (2025–2026) | 5-finger hands | In-house actuators; **fingertip tactile sensors (~3 g)** and **palm cameras** | Controlled by a 3-layer stack with a **1 kHz learned "System 0"** (see §3.3) | ✅ tactile/palm cams/1 kHz; actuator details unknown |
| **Sharpa Wave** (Sharpa) | 22 active DOF, human size | Actuation not disclosed 🔴; **>1000 taxels per fingertip** ("Dynamic Tactile Array"); **500 Hz control**, 20 N max fingertip force, Gigabit Ethernet, MuJoCo MJCF provided | Part of NVIDIA's GR00T reference humanoid (Computex 2026). Pushes **touch** as the missing sense | ✅ specs from CNX/Sharpa |
| **ORCA hand** (ETH Zürich, 2025) | 17 DOF (16 fingers + wrist), open source | **Tendon-driven** (0.4 mm braided nylon), tendons through each joint's **centre of rotation** (like Tendra's index); servos: **Dynamixel XC330-T288-T (fingers) + XC430-T240BB-T (wrist)**, or in the newer Feetech variant **HLS3915 (fingers) + HLS3930 (wrist)** (the paper names no servo model; models from `orca_core`); DIP rigidly coupled to PIP; pin joints that pop out under overload; ratchet-spool tensioning; auto-calibration by driving to hard stops; FSR fingertip sensors | Material cost < 2000 CHF, ~1.2 kg; the closest open-source relative of Tendra | ✅ paper + orca_core, 2026-09-30 |
| **Pollen AmazingHand** (2025) | 8 DOF, 4 fingers | **8 × SCS0009** in the palm, 2 per finger in a parallel linkage (flex + abduction by differential motion) | Shows what the SCS0009 is good for: light, cheap (< €200), not strong | ✅ |
| **MIDAS Hand** (2026, arXiv 2607.14487) | 16 DOF, 13 active | **Direct drive**, measured low backdrive torque; 283 three-axis taxels; 700 g; BOM < US$3000 | A 2026 open design going the "torque transparent" way | ✅ abstract |
| **MM-Hand** (2026, arXiv 2604.17245) | 21 DOF | **Remote tendon actuation through 1 m Bowden sheaths**, spring-return fingers, joint-angle sensors, tactile, in-palm stereo | **25 N fingertip force through a 1 m sheath**: evidence that remote servos (Tendra `ideas.md` open question) can work | ✅ abstract |
| **Aero Hand Open** (TetherIA, arXiv 2608.28578) | 16 joints, **7 actuators (7 × Feetech HLS3606M)**, 3-DOF thumb | Tendon/cable-driven (braided Kevlar/Vectran), PIP–DIP coupled by a cable, backdrivable; ~12 N per finger, 374–400 g, US$314 BOM; open simulation model **of the cable transmission itself** + an identified actuation map; RL policy runs on the real hand **with no fine-tuning** | Template for modelling Tendra's tendons in the sim | ✅ abstract |
| **Inspire RH56 series** | 6 actuators, 12 joints | **Coreless motor + lead screw linear actuators** in the palm, linkages; per-finger **force sensor** | Cheap and robust, widely used on Chinese humanoids and in research (DexUMI used one). Lead screws are **not backdrivable** | 🟡 |
| **PSYONIC Ability Hand** | 6 actuators | Motor + gearbox in the fingers, fast (full close < 0.2 s claimed) | Prosthetic heritage: speed, touch sensors, and **fingers that survive impact** | 🟡 |
| **LEAP Hand** (CMU, 2023) | 16 DOF | **Direct Dynamixel** (XC330) at the joints, no tendons | Cheap (~US$2k), easy to repair; **current-limited position control** = soft grasps; used in many 2024–2025 papers | 🟡 |
| **Wuji Hand** (2025) | 20 DOF | Micro motors **inside the fingers** 🔴 | Shows the trend to integrated, high-DOF hands | 🔴 |

**The pattern:** (1) tendons + forearm actuators whenever the DOF count is high (Shadow, Tesla, 1X, ORCA, Tendra); (2) **some form of force information** at every joint: tendon-force sensors (Shadow), motor current with low gear ratios (1X at 5–15:1, MIDAS, BLDC designs), current-based servos (LEAP, ORCA), or force sensors (Inspire); (3) tactile skins are the 2025–2026 frontier (Figure, Sharpa, 1X, ORCA Touch, MIDAS).

---

## 2. Smooth motion

### 2.1 Why hobby-servo hands look jerky

| Cause | What happens | Is it in Tendra now? |
|---|---|---|
| **Low setpoint rate** | The goal jumps every 20–33 ms; the servo reaches it and stops → staircase motion, audible "ticking" | **Yes**: goals go out every `kGoalPeriodMs = 20` ms (50 Hz) |
| **Goal speed not matched to the ramp** | The servo moves each small step at its own speed (fixed 600) and then waits: stop–go inside every 20 ms interval | **Yes**: `kServoGoalSpeed = 600` always, while the ramp is capped at 400 ticks/s |
| **Deadband** | The servo ignores errors smaller than the deadband (registers 26/27). At slow speed, error grows to the deadband, then the motor kicks → **stick-slip** | Likely (factory deadband is ≥ 1 tick 🔴) |
| **P-only / low-D controller** | Overshoot and ringing on fast moves; a loaded tendon loop acts like a spring and oscillates | Servo-internal; tunable via registers 21–23 |
| **Quantisation** | 1 tick = 0.293°. At 10 ticks/s the goal changes every 100 ms | Yes (10-bit pot) |
| **Backlash + tendon slack** | Direction reversals have a dead zone (servo gearbox ≤ 0.5° + tendon stretch + PTFE friction hysteresis) | Yes, in a tendon hand it is the biggest error on reversal |
| **Stick-slip in the tendons** | PTFE sheaths have static friction > dynamic friction; the strand sticks, then jumps | Yes (capstan friction grows as e^(μ·total bend angle)) |
| **Bus latency / blocking reads** | A slow reply (up to the 3 ms timeout) delays the goal stream | Possible: `pollNext` is blocking; an offline servo costs 3 ms each retry |
| **Jerk (not only accel) limits missing** | A trapezoid has instant acceleration changes → a small jolt at each ramp corner, visible on light fingers | Yes: `MotionProfile` is trapezoidal |
| **The PC stream re-plans every target** | Teleop sends a new target every 33 ms; each one re-plans the trapezoid → velocity ripple | Yes when streaming from teleop |

### 2.2 What the SCS0009 actually exposes

From the Feetech SCSCL memory table (official FTServo_Arduino `SCSCL.h`, MIT) plus rustypot's SCS0009 definition, as recorded in `research/experiments/2026-09-28-full-hand/research.md` ✅. The SCS protocol descends from the Dynamixel AX-12 protocol, so I map the old Dynamixel names (compliance margin/slope, punch) onto Feetech's registers. That mapping is my inference 🔴.

| Addr | Register | Area | Use for smoothness/compliance | AX-12 equivalent (🔴 inferred) |
|---|---|---|---|---|
| 7 | return delay | EEPROM | Set to 0 (or minimum) → faster replies, higher feedback rate | return delay |
| 16 | max torque limit (0–1000 = 0–100 % of stall) | EEPROM | **Grip-force cap and safety.** EEPROM: set once at setup, don't rewrite per grasp (EEPROM wear) | torque limit |
| 19 | unloading condition (bits: 32 overload, 4 temp, 1 voltage) | EEPROM | Decide whether an overload drops torque | alarm shutdown |
| **21 / 22 / 23** | **P / D / I** | EEPROM | **P = stiffness** (lower = softer finger), D = damping (reduces ringing of the tendon spring), I = removes steady error under load (use very little, it causes windup/hunting) | compliance slope ≈ P |
| 24 | min startup force | EEPROM | Minimum drive to overcome static friction. Too high → kicks; too low → sticks | **punch** |
| 26 / 27 | CW / CCW dead band | EEPROM | Smaller = smoother slow tracking but may cause buzzing at rest | **compliance margin** |
| 37 / 38 / 39 | protective torque / protection time / overload threshold (%) | EEPROM 🟡 | The "80 % for 2 s then back off" rule. Relevant to pretension | – |
| 40 | torque enable | RAM | Already used (off at boot ✅) | torque enable |
| 41 | acceleration | RAM | rustypot lists it; Feetech's own library forces 0 on SCS. **Test whether the SCS0009 honours it** 🔴 | – |
| **42 / 44 / 46** | **goal position / goal time (ms) / goal speed** | RAM | The streaming interface. **Goal time** lets the servo interpolate itself over a known interval | goal / moving speed |
| 56 / 58 / 60 / 62 / 63 | position / speed / **load (PWM ‰, not current)** / voltage / temp | RAM | Feedback. Load is a **torque proxy** (see §3.1) | present load |
| 69 | present current | RAM | Listed in the library but **probably not implemented** on SCS 🔴 | – |

**Not available on the SCS0009:** current sensing, a current (torque) control mode, sync read, a RAM copy of the torque limit (as far as the table shows 🔴), a magnetic encoder.

### 2.3 What the firmware does now (0.3.0, `hand_v1_servo`)

✅ From `firmware/src/scs0009_servo.cpp`, `main.cpp`, `config_v1.h`:

- Torque off at boot; the first command first sets goal = measured position (no jump). Good.
- A **trapezoidal `MotionProfile`** in servo ticks (from the stepper code), speed capped at 400 ticks/s (~117°/s), default 1.2 rad/s, 2.4 rad/s².
- Every **20 ms**, one **SYNC WRITE** with goal position + goal time 0 + goal speed 600 for servos whose goal changed.
- **Feedback: one servo every 2 ms, round robin** → each servo is read every **40 ms (25 Hz)**; reads block for up to 3 ms if a servo is silent (offline servos retried each second).
- No use of P/D/I, deadband, punch or torque-limit registers yet.

### 2.4 Recommended firmware changes (in order)

Each one can be tested on one finger with the `F` feedback command and a log of position vs time.

1. **Stream at 100 Hz, and match the servo's speed to the ramp.** 🔴 (tuning hypothesis, cheap to test)
   - `kGoalPeriodMs = 10`. A sync write of 16 servos × 7 bytes ≈ 120 bytes ≈ **1.2 ms** at 1 Mbps ✅ (estimate in the full-hand research), so 100 Hz uses ~12 % of the bus.
   - Send per servo **goal speed = |profile speed| × 1.2 + small margin** (never 0, since 0 = full speed), instead of a constant 600. The servo then travels each 10 ms step at about the speed the ramp wants, with no stop–go.
   - Alternative to test: **goal time = 1.5 × period** (15 ms) with speed ignored. The servo spreads each step over the interval (its own interpolation). Compare both with a logged step response.
2. **Replace the trapezoid with a jerk-limited profile, or add setpoint smoothing.** 🟡
   - Option A: **S-curve** (7-segment, jerk-limited). The open-source **Ruckig** library (MIT, C++17) does this for many DOF online, but may be heavy for 20 DOF at 100 Hz on an ESP32 🔴. A hand-written per-joint S-curve is ~100 lines.
   - Option B (simplest, very effective): keep the trapezoid and pass its output through a **critically damped 2nd-order filter** (time constant ~30–50 ms). This rounds the corners (limits jerk) and costs a few multiplications per joint.
   - Option C (for teleop/policies): **streaming mode**. The PC sends **timestamped setpoints** at 30–50 Hz; the ESP32 buffers them with one period of delay and interpolates (linear or cubic Hermite) at 100 Hz. This is how ROS `joint_trajectory_controller` and most humanoid stacks do it, and it removes the re-planning ripple. Add a new command, e.g. `T <t_ms> q1..q20`.
3. **Tune the servo registers once, per joint type** (write EEPROM at a calibration step, with unlock/lock, never in the control loop). 🔴 values must be found by experiment
   - The LeRobot community's main fix for jerky Feetech (STS3215) arms is **lowering the P gain** from the factory value ✅ (search result, LeRobot docs/issues). Expect the same on the SCS0009, at the cost of more steady-state error under load.
   - Read and log the factory values of 21–27 first (add them to the `B`/calibration output).
   - Lower the **deadband 26/27 to 1** (or 0 if it doesn't buzz), and set **min startup force 24** just high enough that 1-tick steps move the joint under pretension.
   - Add a little **D** if the loop rings on stops; keep **I = 0** or very small.
   - Set **max torque limit 16** to e.g. 60–80 % for MCP joints, 40–60 % for DIP/PIP (protects the tendons and the 3D-printed drums; see §3).
4. **Faster, non-blocking feedback.** ✅ current behaviour / 🔴 new numbers
   - Set **return delay (7) to 0** if it isn't already.
   - Read **position only** (2 bytes) at high rate and load/voltage/temp at a lower rate: position-only reads of all 16 servos ≈ 16 × ~0.2 ms ≈ 3–4 ms → **~100 Hz full position state** alongside the 100 Hz goal stream.
   - Lower the reply timeout for online servos to ~1 ms (a healthy servo answers in well under 0.5 ms at 1 Mbps 🔴), and skip offline servos for longer.
5. **Backlash / friction compensation (feedforward).** 🟡 standard technique
   - Measure per joint the **hysteresis**: sweep the joint slowly open and closed and record servo angle vs real joint angle (camera + ArUco marker, or the MuJoCo twin + a phone video). The gap on reversal = backlash + tendon stretch + friction.
   - On a direction change, add **±δ/2 ticks** to the goal (a "direction-dependent offset"), ramped in over ~30 ms. This is the position-servo form of friction feedforward.
   - With the tendon model (`tendon_router.py` knows total bend angles), you can predict δ per joint from the **capstan equation** F_out = F_in · e^(−μ·Σθ), μ(PTFE on nylon/fluoro line) ≈ 0.05–0.15 🟡.
6. **Use 6 V, not 5 V, for the servos** (if the supply allows): +22 % torque and +25 % speed (0.226 vs 0.185 N·m) ✅. The SCS0009 range is 4.0–7.4 V ✅.

### 2.5 A note on "smooth" at the policy level

Learned policies are often jerky too. Standard fixes (all 🟡 common practice): **action chunking** (ACT, π0 predict 0.5–1 s of actions at once and blend overlapping chunks with **temporal ensembling**), **low-pass filtering actions**, penalising action changes in RL (your `grasp-rl.md` already does), and running the policy at 10–50 Hz with the ESP32 interpolating in between (item 2C above).

---

## 3. Force and compliance control

### 3.1 Getting a force signal out of the SCS0009

The "load" register (60) is the **PWM duty** the servo applies (‰, signed) ✅, not current. A DC motor's torque follows

```
current  i ≈ (duty · V_bus − k_e · ω) / R       (ω = motor speed)
torque   τ_motor = k_t · i,   τ_joint ≈ N · η · τ_motor · (r_joint / r_spool)
```

So when the joint is **still** (holding or squeezing), load ‰ is roughly proportional to torque 🟡. Calibrate it once per joint type: hang known weights on a string at the fingertip, read load, fit a line (and record the voltage, since duty depends on V_bus). While moving, subtract the speed term (present speed is register 58). Expect a noisy, friction-contaminated signal: good enough for **contact detection and grip-force regulation**, not for fine force control. Your `ideas.md` #6 (loads → joint torques → fingertip force via τ = Jᵀ F) is the right next step.

### 3.2 Compliance with a position servo: four tools, from easy to advanced

| Method | How | Good for | Status |
|---|---|---|---|
| **1. Torque-limited position control** | Set register 16 (max torque) low; command a goal *inside* the object. The servo pushes until it hits the limit | Simplest safe grasp. Coarse, and the limit is EEPROM (set per task type, not per grasp) | ✅ register exists |
| **2. Soft servo gains** | Lower P (21): the servo acts like a **softer spring** K. Contact force ≈ K · (goal − actual) | Real, physical compliance at all frequencies, not only at loop rate. Makes fingers conform to shapes | 🔴 values to find |
| **3. "Close until contact, then squeeze Δ"** | Close at low speed; when load > threshold (contact), set goal = current position + Δ ticks. Force ≈ K · Δ | **Grip-force control** with a single number Δ; this is what many prosthetic and LEAP-style grasps do | 🟡 standard |
| **4. Admittance control on the ESP32** | Every 10 ms: estimate joint torque τ̂ from load; move the goal by `Δq = (τ̂ − τ_desired) / K_virtual` (with damping). The finger **yields when pushed** and **regulates force** when grasping | Safe contact, human pushing the finger, polishing/wiping-type tasks | 🟡 standard theory; performance limited by the noisy load signal and 25–100 Hz rate |

**Impedance vs admittance** in one line: *impedance* control commands a **force** that depends on position error (needs a torque actuator: BLDC, current-controlled Dynamixel); *admittance* control measures force and commands a **position** (fits a stiff position servo like the SCS0009). Tendra should use **admittance** now and could use impedance only with a torque-capable actuator.

**Safe contact rules for the firmware** (🔴 my proposals):
- A per-joint **load watchdog**: if |load| > a limit for > 200 ms while moving, stop and back off by a few ticks (prevents the servo's own 2 s overload shutdown and protects tendons).
- Report **contact flags** (load above a calibrated threshold) in the `F` line so policies can use them as touch.
- Keep **loop pretension low**: every newton of pretension is load the servo carries all the time, uses up the 80 % budget and adds friction (research.md §1.2 ✅).

### 3.3 How 1X, Figure and Tesla control at the low level (public info)

- **Figure Helix** (Feb 2025) 🟡: two systems. **System 2** is a 7B-parameter vision-language model at **7–9 Hz** ("think"); **System 1** is an ~80M-parameter visuomotor transformer at **200 Hz** that outputs the **whole upper body (35 DOF) including individual finger positions** and wrist poses. **Helix 02** (early 2026) ✅ adds **System 0 at 1 kHz**: a learned whole-body controller for balance, **contact forces** and coordination that Figure says replaced ~109,500 lines of hand-written C++. Fingertip touch (~3 g) and palm cameras feed the policy. This is the "fast reflex under slow brain" pattern taken to the lowest layer.
- **1X** ✅: every hand DOF is **natively force-controlled and backdrivable** through **5:1–15:1 tendon drives**. So the low level is a **torque/impedance loop**, and the hand senses contact through its own motors ("push on a finger and it yields, and reports how hard you pushed"). The policy (Redwood AI, 2025 🟡) outputs targets that this layer tracks; 1X also trains a **world model** to evaluate policies 🟡.
- **Tesla Optimus** 🟡: very little published. Custom actuators; 25 per forearm driving tendons (Gen 3) ✅; policies trained from teleop and (reported) human video.
- **Common denominator:** the learned policy never drives motors directly. It outputs **joint position targets** (often with stiffness/damping gains = "impedance targets") at 10–200 Hz, and an embedded **PD / torque loop at ~1 kHz** tracks them. Tendra's layers (PC policy → ESP32 at 100 Hz → servo's internal loop at kHz) already match that pattern.

### 3.4 Tendon space vs joint space, and the coupling matrix

- Each tendon's length depends on the joint angles: **l = l₀ + R q**, where **R** (m × n) is the **moment-arm (coupling) matrix**: R[i][j] = how much tendon i shortens per radian of joint j (= the drum or pass-through radius).
- Forces map back the other way: **τ = Rᵀ f** (joint torques from tendon tensions).
- **Tendra's design choice** (strands cross each joint on its axis, own drum per joint) makes R almost **diagonal**: one servo ↔ one joint. That is why the firmware can stay in joint space with a scalar `servo_per_joint`. ✅ (router tests check coupling; the sim's `test_v1.py` checks moment arms.)
- If residual coupling is measured (e.g. a pass-through strand that isn't exactly on the axis), **compensate in firmware**: servo angle θ = (R q) / r_spool, i.e. replace the scalar scale by a **sparse matrix** (a few off-diagonal terms, per finger). The same R, taken from `tendon_routes.json`/MuJoCo, is used for **force estimation** τ = Rᵀ f.
- **Tendon-space control** (commanding tendon lengths or tensions directly) is what Shadow does with its tendon-force sensors; it is only worth it with tendon-force sensing. For Tendra, **joint space + coupling compensation** is the right choice.
- A human-like mechanical option (owner decision, 🔴 idea): **couple DIP to PIP** (DIP ≈ 2/3 of PIP, as in human fingers and the Shadow hand) → frees 4 servos, whose budget and space could go to stronger MCP/thumb actuators.

---

## 4. Learning-based dexterity, 2024–2026

### 4.1 Teleoperation and data collection

| System | Year | What it is | Cost / access | Fit for Tendra | Conf. |
|---|---|---|---|---|---|
| **Webcam + MediaPipe** (your `sim/teleop.py`) | – | Monocular hand landmarks, retargeting | Free | Now. Occlusion and depth errors limit precision grasps | ✅ |
| **AnyTeleop / dex-retargeting** | 2023 | Vision-based teleop framework and an open retargeting library for many hands | Open source | Use `dex-retargeting` ideas/optimiser as a reference for yours | 🟡 |
| **Apple Vision Pro / Quest 3 hand tracking** (Open-TeleVision, Bunny-VisionPro, VisionProTeleop) | 2024 | Headset hand tracking (~25 keypoints/hand, 60–90 Hz) + stereo view for the operator | Quest 3 ~US$500 | **Next step up from the webcam**; head camera streaming gives immersion | 🟡 |
| **Manus gloves** (Quantum Metaglove, Prime series) | – | Magnetic/flex-sensor gloves, used by many labs and companies | Several k USD | Great data but expensive | 🟡 |
| **Rokoko Smartgloves** | – | IMU + EMF gloves for mocap | ~1–1.5k USD | Cheaper, drift-prone for fingers | 🟡 |
| **DexCap** | 2024 | Portable mocap glove + chest cameras for in-the-wild demos | Open source, ~4k USD hardware | Idea for recording at home | 🟡 |
| **DexUMI** (arXiv 2505.21864) | 2025 | A **wearable exoskeleton shaped like the robot hand**, so the human hand motion is *already* feasible for the robot (and the wearer feels real contact); video inpainting swaps the human hand for the robot hand in the images. 86 % average success on two robot hands | Open source | **Exactly the Tendra kinematic-twin glove idea** (`ideas.md` #1–2). Strong validation of that plan. Follow-ups in 2026: RealDexUMI, SEED-UMI (exoskeleton shared by human and robot) | ✅ |
| **DOGlove** (RSS 2025, arXiv 2502.07730) | 2025 | Open-source glove: **21-DoF motion capture**, **5-DoF cable-driven force feedback**, 5 fingertip LRA haptics; **< US$600**, assembled in hours | Open source (TEA-Lab/DOGlove) | Model for adding force feedback from servo loads to the Tendra glove | ✅ |
| **HOMIE** | 2025 | Humanoid teleop "cockpit": isomorphic exoskeleton arms + motion-sensing gloves + a pedal for locomotion | Open source, low cost | Template for the bimanual pole station (Stage D) | 🟡 |
| **GELLO-style leader arms** | 2023–24 | Scaled, 3D-printed copy of the robot with cheap servos as encoders | ~US$300 | Same principle as SO-101 leader; Tendra glove = "GELLO for the hand" | 🟡 |

### 4.2 Imitation learning and VLAs

| Model | Year | What | Compute to use it | Conf. |
|---|---|---|---|---|
| **ACT** (Action Chunking with Transformers) | 2023 | Small transformer (~80M), predicts action chunks; ~50 demos per task | Trains in hours on one consumer/cloud GPU; **runs on CPU at low Hz**. In LeRobot | 🟡 |
| **Diffusion Policy** | 2023 | Denoising diffusion over action sequences; handles multimodal demos well | One GPU, hours–a day; inference needs a GPU for fast rates (or DDIM few steps) | 🟡 |
| **3D Diffusion Policy (DP3)** | 2024 | Diffusion Policy on point clouds, data-efficient, includes dexterous-hand tasks | One GPU | 🟡 |
| **SmolVLA** (Hugging Face) | 2025 | ~450M-param open VLA, trained on community LeRobot data; async inference | Fine-tune on one GPU; can run on a laptop GPU / fast CPU at low rates | 🟡 |
| **π0 / π0-FAST / π0.5** (Physical Intelligence, open weights via `openpi`) | 2024–2025 | 3B VLM + flow-matching action expert, 50 Hz action chunks; π0.5 adds open-world generalisation | openpi README ✅: **inference > 8 GB GPU, LoRA fine-tune > 22.5 GB (RTX 4090), full fine-tune > 70 GB (A100/H100 80 GB)**. PyTorch port exists (no LoRA there yet) | ✅ |
| **π0.6 / π\*0.6 (RECAP)** | Nov 2025 | Faster and better on dexterous tasks (laundry, box assembly); π\*0.6 improves from the robot's own experience and corrections | Weights not in openpi (as of the README read today) | ✅ model card exists; details 🟡 |
| **NVIDIA GR00T N1 → N1.5 → N1.6 → N1.7** | 2025–2026 | Open humanoid foundation models (VLM + diffusion-transformer action head), cross-embodiment. N1.6 (Sept 2025): 2× larger DiT, **state-relative actions → smoother, less jittery motion**. N1.7 in early access; **GR00T N2** ("world action model") announced for end of 2026 | Fine-tune on one large GPU; open on Hugging Face | ✅ (NVIDIA pages via search) |
| **Figure Helix / Helix 02** | 2025–2026 | See §3.3; closed | – | ✅ |
| **Gemini Robotics / 1.5 / On-Device / Gemini Robotics 2** (Google DeepMind) | 2025–2026 | VLA on Gemini. **On-Device** (June 2025) was the first fine-tunable one (**50–100 demos**), runs on the robot. **Gemini Robotics 2** (30 July 2026): VLA + ER 2 reasoning + On-Device 2, whole-body control and dexterous tasks | Closed (trusted-tester SDK) | ✅ |
| **1X Redwood AI + 1X World Model** | 2025 | NEO's VLA for home tasks and a learned simulator to evaluate policies | Closed | 🟡 |

### 4.3 Sim-to-real RL for in-hand manipulation

| Work | Year | Key idea | Conf. |
|---|---|---|---|
| **OpenAI Dactyl / Rubik's cube** | 2018–2019 | Shadow hand, PPO, massive **domain randomisation** and **automatic domain randomisation (ADR)**; thousands of CPU cores | 🟡 |
| **HORA** (in-hand object rotation) | 2022 | Teacher–student: a privileged RL teacher, then a student using only proprioception history to infer object properties. Allegro hand | 🟡 |
| **DeXtreme** (NVIDIA) | 2022 | Allegro hand cube reorientation in Isaac Gym with vision, GPU-parallel sim | 🟡 |
| **Visual Dexterity** (MIT) | 2023 | Reorientation of novel shapes, including **hand facing down**, with depth camera; low-cost hand | 🟡 |
| **DexPBT** | 2023 | **Population-based training** for hand-arm manipulation, big speedup | 🟡 |
| **DextrAH-G / DextrAH-RGB** (NVIDIA) | 2024–25 | Fabric-guided RL teacher → RGB student for grasping with arm + hand | 🟡 |
| **MuJoCo Playground** (Google DeepMind) | 2025 | MJX/JAX GPU environments including **LEAP hand in-hand cube reorientation, trained in minutes on one GPU** and transferred to the real hand. Warning from its issue tracker: users copying the paper's sim→Dynamixel gain mapping got a **noticeably faster real hand** than in the videos (issue #302, open). Actuator modelling is where sim-to-real breaks | ✅ |
| **Aero Hand Open** | 2026 | Tendon-driven hand with a sim model of the **cable transmission** and an identified motor↔joint map; RL policy transfers with **no fine-tuning** | ✅ abstract |
| **Humanoid dexterous sim-to-real** (e.g. Lin et al. 2025), **DexterityGen** (2025) | 2025 | Vision-based dexterous skills on humanoid hands; a learned low-level "dexterity foundation controller" that teleop/policies steer | 🟡 / 🔴 details |

**Lesson for Tendra:** every successful in-hand RL result used (a) **a hand that matched the sim** (system identification), (b) **domain randomisation** of friction, mass, **actuator gains and latency**, and (c) a **teacher–student** split. Actuator modelling matters most: add **servo latency, deadband, backlash and the P-gain spring** to the V1 MuJoCo actuators, then randomise them (your grasp RL already randomises stiffness ±20 %).

### 4.4 Learning from human video

- **DexMV** (2022): human videos → 3D hand/object pose → retargeted demos → imitation + RL. 🟡
- **EgoDex** (Apple, 2025) ✅: **829 hours** of 30 Hz 1080p egocentric video, 338k episodes, 194 tabletop tasks, with **3D head, upper-body and finger poses** from Vision Pro ARKit, plus language labels; 2 TB. Licence **CC BY-NC-ND**: fine for research and for fitting Tendra synergies, but **not for a commercial product or redistributed derivatives**. Follow-up: EgoScale (2026).
- **EgoMimic** (2024), **DexWild** (2025), **Human-Humanoid co-training** (e.g. HAT, 2025): co-train on human video + a few robot demos. 🟡
- **Retargeting** is the bridge: your Gauss-Newton fingertip/landmark fit is the same idea as `dex-retargeting`. With a kinematic-twin glove it becomes unnecessary.

### 4.5 What is realistic on a CPU laptop + cloud GPUs

| Task | Where | Rough cost | Realistic now? |
|---|---|---|---|
| Grasp RL (current PPO, MuJoCo CPU) | Laptop | free, slow (hours–days) | ✅ yes (running) |
| In-hand reorientation RL, V1 hand | **Cloud GPU with MJX / MuJoCo Playground** (port `GraspEnv` to MJX) | ~US$1–3/h for a single A100/H100-class rental; hours per run 🟡 | ✅ next step |
| ACT / Diffusion Policy on ~50 teleop demos | Free Colab/Kaggle GPU or cheap rental; ACT inference on the laptop CPU | ~free–US$20 | ✅ yes (roadmap Stage A) |
| SmolVLA fine-tune | One cloud GPU | ~US$10–50 | ✅ yes |
| π0 / π0.5 LoRA fine-tune; GR00T fine-tune | One ≥ 24 GB cloud GPU for LoRA (openpi: > 22.5 GB), 80 GB for full fine-tune; inference (> 8 GB GPU) on a rented server over the network | ~US$50–300 per experiment 🔴 | ✅ requirements; Stage C–D |
| Training a VLA from scratch / world models | Clusters | – | ❌ not realistic |

---

## 5. Recommendations for Tendra (prioritised)

### Priority 1: firmware (free, this month)

1. **100 Hz goal stream + per-servo goal speed from the ramp** (or goal time = 1.5 × period); log step responses before and after. (§2.4.1)
2. **Jerk limiting**: 2nd-order filter after the trapezoid (fastest), or an S-curve. (§2.4.2)
3. **Streaming command with timestamps and ESP32-side interpolation** for teleop and policies. (§2.4.2C)
4. **Faster feedback**: position-only reads at ~100 Hz, load/temp slower, shorter timeout, return delay 0. (§2.4.4)
5. **Register calibration tool**: read and log registers 7, 16, 19, 21–27, 37–39 for every servo; a guarded command to write tuned values (EEPROM unlock/lock). (§2.4.3)
6. **Safety and touch**: load watchdog, contact flags in `F`, grasp mode "close until contact, squeeze Δ". (§3.2)

### Priority 2: actuators (before building V1)

| Joints | Recommendation | Why | Conf. |
|---|---|---|---|
| 4 × `mcp_flex`, `thumb_cmc_flex`, `thumb_cmc_rot` (6 servos) | **Option A (preferred): HLS3606M** | 2.6× torque (0.59 N·m), **current feedback + constant-current mode** (true grip-force control), lower gear ratio (1:205, less friction), magnetic sensor, same 23 × 12 plan (27.5 mm tall vs 25.25), 20.6 g, ~US$30, used by Aero Hand Open. Needs an `HlsServo` `MotorDriver` class (Feetech's `HLSCL` table: goal current reg 44 at 6.5 mA/unit, protection current reg 28), **new spools for the 25T / 4.95 mm spline**, and preferably a **separate bus**: Feetech's own wiki says HLS has its own memory table (not STS-compatible) and discourages mixing series on one bus | ✅ datasheet + Feetech wiki; 🔴 ear-hole positions |
| same | **Option B (no firmware work): SCS2332** | ~2× torque (≈0.44 N·m), metal gears, **same SCS protocol**, same footprint (+3.25 mm, +7 g) | ✅ specs from listings; 🔴 confirm the memory table matches SCSCL |
| `pip`, `dip`, `thumb_ip`, `thumb_mcp_flex`, 4 × `mcp_abd` | Keep **SCS0009** | Light loads; small; cheap | ✅ |
| Everything, V1.5 | All **HLS3606M** (or STS3032) | One servo type, current sensing on every joint = force-transparent-ish hand at ~US$25/joint. Compare cost/benefit after the 6-joint test | 🔴 decision later |
| Research option | **One BLDC+FOC test joint** (gimbal motor + small planetary, SimpleFOC) on `index_mcp_flex` | Learn what 1X-style low-ratio torque transparency feels like; compare with HLS current mode | 🔴 experiment |
| Power | **6 V** supply (≥ 15 A for 16 servos) | +22 % torque vs 5 V | ✅ datasheet |

Before buying: measure the **real tendon force needed** per joint on the printed index (spring scale on the strand while the finger holds a known load); compare with **80 % × stall / r_spool** (SCS0009 at 6 V: 0.8 × 0.226 / 0.006 ≈ **30 N** of tendon force; SCS2332 ≈ 58 N; HLS3606M ≈ 78 N, all from the stall torque). For scale, the 1X NEO hand reaches 2.6 N·m at a finger MCP; with a 6 mm drum even the HLS3606M gives only ~0.47 N·m continuous-peak at the MCP. **Torque is Tendra's biggest gap to the leaders; smoothness and softness are fixable in software.** Check that 0.4 mm fishing line and the printed drums survive ~80 N.

### Priority 3: software pipeline

1. **Actuator model in the sim**: servo latency (~10–20 ms), deadband, backlash, P-gain spring, torque limit, and **load-as-PWM** feedback, then **system ID** against the real finger (roadmap Stage C). Randomise these in RL.
2. **Load → torque calibration** per joint type (hanging weights) and **τ = Rᵀ f → fingertip force** estimate (`ideas.md` #6). Feed contact flags to policies.
3. **LeRobot** integration of the real V1 (a `Robot` class over `RealHand`), so teleop data goes straight into LeRobot datasets; then **ACT** first, **Diffusion Policy** second, **SmolVLA** third.
4. Port `GraspEnv` to **MJX/MuJoCo Playground** for cloud-GPU RL (in-hand rotation, rung 5 of the Tendra Ladder).

### Priority 4: data-collection hardware

1. Now: webcam (have it). Cheap upgrade: a **second webcam** for stereo/triangulation, or a depth camera.
2. **Quest 3** hand tracking (~US$500): better tracking, wrist pose, and it doubles as the operator's display.
3. **Tendra kinematic-twin glove** (AS5600/MT6701 magnetic encoders or pots per joint, ESP32, same joint axes as V1). DexUMI and DOGlove (2025) are the closest published designs and validate the idea; add **vibration or cable-brake feedback** from servo loads later (DOGlove-style).
4. Skip Manus/Rokoko unless a sponsor pays; the glove is a better fit and a potential product.

---

## 6. Open questions to test (log results in `research/log.md`)

- What are the SCS0009's **factory P/D/I, deadband and min-startup-force** values? (read registers 21–27)
- Does the SCS0009 honour **register 41 (acceleration)**? Does it honour **goal time** during streaming?
- Is register 16 (torque limit) copied to RAM at boot, or read live from EEPROM? Can a RAM override be written?
- Step response at 50 Hz/fixed speed vs 100 Hz/matched speed vs goal-time mode: overshoot, tracking lag, audible noise.
- Load reading vs hanging weight, at 5 V and 6 V: linear? noise level? hysteresis?
- Tendon hysteresis per joint (servo angle vs joint angle on reversal).

---

## Sources

Confirmed in this repo or in this session's search results:
- Feetech SCS0009 product specification (datasheet), Seeed mirror: https://files.seeedstudio.com/products/Feetech/SCS0009-Specifications.pdf and Switch Science mirror: https://pages.switch-science.com/comparison/files/feetech/serial-scs/SCS0009_datasheet.pdf
- Feetech SCS0009 product page: https://www.feetechrc.com/6v-23kg-serial-bus-steering-gear_65522.html ; Seeed listing: https://www.seeedstudio.com/Feetech-SCS0009-Servo-p-6535.html
- Official Feetech Arduino library (SCSCL memory table): https://github.com/ftservo/FTServo_Arduino/blob/main/src/SCSCL.h
- rustypot SCS0009 definition (no sync read, model number): https://github.com/pollen-robotics/rustypot/blob/develop/src/servo/feetech/scs0009.rs and PR #132/#137: https://github.com/pollen-robotics/rustypot/pull/132 , https://github.com/pollen-robotics/rustypot/pull/137
- Feetech SCS series overview: https://www.feetechrc.com/scs_ttl_Servo.html ; SCS2332/STS3032 6 V 4.5 kg listings: https://www.feetech.cn/en/product-name_55300.html , https://rcdrone.top/products/feetech-scs-sts3032-4-5-kg-servo , https://aifitlab.com/products/feetech-scs2332-servo-motor , https://kamami.pl/en/micro-servos/581851-feetech-ft3325m-micro-digital-servo-120-5906623458820.html
- Feetech STS3215: https://www.feetechrc.com/525603.html , https://www.robotshop.com/products/feetech-12v-30kgcm-magnetic-encoding-servo-sts3215 ; STS3215 backlash/torque test: https://robonine.com/testing-of-feetech-sts3215-servomotor-backlash-repeatability-and-torque/
- FE-URT-1 + ESP32 thread: https://forum.arduino.cc/t/how-to-connect-esp-32-to-feetech-urt-1-to-control-scs-serial-bus-servos/1051765
- Feetech STS3032 (size, 12-bit magnetic, coreless, aluminium case): https://www.feetechrc.com/6v-45kg-magnetic-code-360-degree-serial-bus-steering-gear.html , https://evelta.com/sts3032-6v-4-5kg-360deg-serial-bus-servo-motor/ , https://lxrcmodel.com/product/feetech-sts3032-6v-4-5kg-coreless-motor-360-degree-serial-bus-servo-for-robotic-hands-joints/
- Feetech HLS3606M (constant-current mode, 1:205, 20.6 g, 23 × 12 × 27.5): https://shop.babsco.com/feetech-hls3606m-6kg-constant-force-servo , https://openelab.io/products/feetech-hl3606-ttl-servo-motor , https://servodatabase.com/servo/feetech/hls3606m ; Feetech HLS series page: https://www.feetechrc.com/hl%E6%81%92%E5%8A%9B%E7%B3%BB%E5%88%97%E8%88%B5%E6%9C%BA.html
- ORCA hand Feetech current-limit fix (HLS goal current reg 44 = 6.5 mA/unit, protection current reg 28): https://github.com/orcahand/orca_core/pull/101 ; Feetech series overview (never mix memory tables): https://github.com/ftservo/ftservo-wiki/blob/main/docs/en/products/series.md
- ROBOTIS XL330-M288-T: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/ ; XC330-M288-T: https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/ , https://www.robotis.us/dynamixel-xc330-m288-t/
- LeRobot Feetech notes (P-gain and acceleration): https://github.com/huggingface/lerobot/issues/673 , https://www.mintlify.com/huggingface/lerobot/motors/feetech
- 1X, "NEO's Hands" (25 DOF, 5–15:1 tendon drive, force-controlled, specs): https://www.1x.tech/discover/neos-hands ; https://roboticsandautomationnews.com/2026/07/17/1x-unveils-25-degree-of-freedom-humanoid-robot-hands-for-neo/103405/
- Tesla Optimus Gen 3 hand: https://droids.substack.com/p/the-forearm-is-the-new-hand-inside , https://www.basenor.com/blogs/news/tesla-optimus-gen-3-hands-22-dof-50-actuators-explained
- Figure Helix 02 (System 0 at 1 kHz, fingertip tactile, palm cameras): https://www.figure.ai/news/helix-02
- Sharpa Wave: https://www.sharpa.com/pages/wave , https://www.cnx-software.com/2026/06/02/sharpa-wave-high-end-dexterous-robotic-hand-with-22-dof-high-sensitivity-dynamic-tactile-array/
- ORCA hand (ETH, arXiv 2504.04259): https://arxiv.org/abs/2504.04259 , https://orca.ethz.ch/
- Pollen AmazingHand (8 × SCS0009): https://github.com/pollen-robotics/AmazingHand , https://huggingface.co/blog/pollen-robotics/amazing-hand
- MIDAS Hand (2026): https://arxiv.org/abs/2607.14487 ; MM-Hand (2026): https://arxiv.org/abs/2604.17245 ; Aero Hand Open (2026): https://arxiv.org/abs/2608.28578
- DexUMI: https://arxiv.org/abs/2505.21864 , https://dex-umi.github.io/ ; RealDexUMI: https://arxiv.org/abs/2606.06033 ; SEED-UMI: https://arxiv.org/pdf/2609.11753
- DOGlove: https://arxiv.org/abs/2502.07730 , https://github.com/TEA-Lab/DOGlove/
- openpi README (GPU requirements): https://github.com/Physical-Intelligence/openpi/blob/main/README.md ; π0.6 model card: https://website.pi-asset.com/pi06star/PI06_model_card.pdf
- GR00T N1.6: https://research.nvidia.com/labs/gear/gr00t-n1_6/ ; NVIDIA newsroom (N1.7, N2): https://nvidianews.nvidia.com/news/nvidia-and-global-robotics-leaders-take-physical-ai-to-the-real-world
- Gemini Robotics On-Device: https://deepmind.google/blog/gemini-robotics-on-device-brings-ai-to-local-robotic-devices/ ; Gemini Robotics 2: https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/
- EgoDex: https://arxiv.org/abs/2505.11709 , https://github.com/apple/ml-egodex ; EgoScale: https://arxiv.org/pdf/2602.16710
- MuJoCo Playground: https://playground.mujoco.org/ ; LEAP sim-to-real gain issue: https://github.com/google-deepmind/mujoco_playground/issues/302
- Tendra repo: `firmware/src/scs0009_servo.cpp`, `firmware/src/main.cpp`, `firmware/include/config_v1.h`, `research/experiments/2026-09-28-full-hand/research.md`, `research/ai/*.md`

Published sources for the 🟡 items (not re-fetched in this session; search by title):
- ROBOTIS e-Manual, XL330-M288-T, XC330-M288-T, XL430-W250-T: https://emanual.robotis.com/docs/en/dxl/x/
- LEAP Hand (Shaw, Agarwal, Pathak, RSS 2023): https://leaphand.com , arXiv 2309.06440
- Shadow Robot, Dexterous Hand and DEX-EE: https://www.shadowrobot.com
- Inspire Robots RH56 hands: https://www.inspire-robots.store (vendor site)
- PSYONIC Ability Hand: https://www.psyonic.io
- Figure, "Helix: A Vision-Language-Action Model for Generalist Humanoid Control" (Feb 2025): https://www.figure.ai/news/helix
- 1X (NEO, Redwood AI, world model): https://www.1x.tech
- Ruckig online trajectory generation (MIT licence): https://github.com/pantor/ruckig
- SimpleFOC (open-source FOC for BLDC/gimbal motors): https://simplefoc.com
- Chi et al., Diffusion Policy (2023): arXiv 2303.04137
- Zhao et al., ACT / ALOHA (2023): arXiv 2304.13705
- Black et al., π0 (2024): arXiv 2410.24164 ; π0.5 (2025): arXiv 2504.16054 ; openpi: https://github.com/Physical-Intelligence/openpi
- NVIDIA GR00T N1 (2025): arXiv 2503.14734 ; https://github.com/NVIDIA/Isaac-GR00T
- Gemini Robotics (2025): arXiv 2503.20020
- SmolVLA (2025): arXiv 2506.01844 ; LeRobot: https://github.com/huggingface/lerobot
- OpenAI, Solving Rubik's Cube with a Robot Hand (2019): arXiv 1910.07113
- Handa et al., DeXtreme (2022): arXiv 2210.13702
- Petrenko et al., DexPBT (2023): arXiv 2305.12127
- Qi et al., HORA, In-Hand Object Rotation via Rapid Motor Adaptation (2022): arXiv 2210.04887
- Chen et al., Visual Dexterity (2023, Science Robotics): arXiv 2211.11744
- Qin et al., AnyTeleop (2023): arXiv 2307.04577 ; dex-retargeting: https://github.com/dexsuite/dex-retargeting
- Qin et al., DexMV (2022): arXiv 2108.05877
- Wang et al., DexCap (2024): arXiv 2403.07788
- Cheng et al., Open-TeleVision (2024): arXiv 2407.01512
- MuJoCo Playground (2025): arXiv 2502.08844 ; https://playground.mujoco.org
- HOMIE (2025), DextrAH-G/-RGB (NVIDIA 2024–25), DexterityGen (2025), DexWild (2025), EgoMimic (2024): search arXiv by name (IDs not re-checked)
- Santello et al., Postural hand synergies (1998); Flash & Hogan, minimum-jerk (1985): classic references (already cited in `grasp-rl.md`)

Sources added in the 2026-09-30 fact-check:
- Feetech SCS0009 datasheet A/0 (text extracted from the Seeed mirror PDF): https://files.seeedstudio.com/products/Feetech/SCS0009-Specifications.pdf
- Feetech HLS3606M datasheet A/0, 2024-04-22: https://www.feetechrc.com/Data/feetechrc/upload/file/20240807/6385862508379674057566108.pdf
- Feetech STS3032 product page: https://www.feetechrc.com/6v-45kg-magnetic-code-360-degree-serial-bus-steering-gear.html ; SCS2332 listing: https://aifitlab.com/products/feetech-scs2332-servo-motor
- Feetech series wiki (HLS memory table not STS-compatible; don't mix series on one bus): https://github.com/ftservo/ftservo-wiki/blob/main/docs/en/products/series.md
- ROBOTIS: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/ , https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/ , https://www.robotis.us/dynamixel-xc330-t288-t/ , https://www.robotis.us/dynamixel-xl330-m288-t/
- ORCA servo models: https://github.com/orcahand/orca_core (`orca_core/constants.py`, `hardware/dynamixel_client.py`, `hardware/feetech_client.py`); ORCA v1 (XC330-T288-T + XC430-T240BB-T): https://humanoid.guide/product/orca-v1/
- Aero Hand Open (7 × HLS3606M, US$314): https://arxiv.org/html/2608.28578 , https://github.com/TetherIA/aero-hand-open

Fact-checked 2026-09-30 (partial): SCS0009 and HLS3606M against Feetech datasheets; STS3032, SCS2332 (listings); XL330/XC330 (ROBOTIS e-Manual and store prices); ORCA servo models (orca_core); Aero Hand Open servos; HLS vs STS protocol. Not checked: HLS3606M mounting-hole positions, STS3032 current feedback, LEAP/Shadow/Inspire/PSYONIC rows.
