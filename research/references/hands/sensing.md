# Sensing for dexterous hands: touch, force and proprioception

*Research note, 2026-09-30. Scope: what leading hands use, which sensor technologies exist, how learning uses touch, and a staged, affordable sensing plan for Tendra Hand V1.*

**How to read this note.** Facts tagged **[confirmed]** come from a source fetched during this research (URL in *Sources*). Facts tagged **[background]** come from well-known papers or datasheets that could not be re-fetched this session (the web tools failed partway through). Check those before buying parts. **[speculative]** marks my own estimates or ideas.

---

## 0. Key terms (short)

- **Proprioception**: the robot's sense of its own body, meaning joint angles, speeds and forces. Tendra has this today only through the servo encoders, which see the *spool* and not the *joint*. Tendon stretch and friction sit in between.
- **Tactile sensing (touch)**: sensing contact on the skin: where, how hard (normal force), sideways force (shear), and vibration (slip, texture).
- **Taxel**: a "tactile pixel", one sensing point.
- **Shear**: force along the surface. It is the key signal for **slip**. A sensor that only measures pressure (FSR, Velostat) cannot see shear directly.
- **Force transparency**: a low-friction, low-gear-ratio drive lets outside forces push back through to the motor, so the motor's current or load *is* a force sensor.

---

## 1. What the leading hands use (2025–2026)

| Hand | Actuation | Touch | Other sensing | Status |
|---|---|---|---|---|
| **Figure 03** (Oct 2025) | Motors in hand/forearm (details not public) | In-house "first-generation" fingertip tactile sensor, detects **~3 g**. Built in-house because market sensors weren't durable enough **[confirmed]** | **Palm camera** in each hand, wide FOV, low latency, used by Helix when head cameras are blocked **[confirmed]** | Shipping to pilots |
| **1X NEO hands** (Jul 2026) | **Tendon-driven, 25 DOF**, motors in forearm, tendons through wrist, low gear ratio ≈5:1–15:1 for force transparency **[confirmed]** | Tactile stack moulded into soft polymer skin on fingertips and hand surfaces: normal force, contact location, **shear**, used for slip correction **[confirmed]** | Motor-side force estimation through the transparent drive **[confirmed]** | Product |
| **Tesla Optimus Gen 3** | 22 DOF, ~50 actuators (reported), tendons from forearm | Fingertip force/tactile sensors, "4× more sensitive" than Gen 2 **[confirmed as reported by press/blogs, not by Tesla specs]** | — | Pre-production |
| **Sharpa Wave** | 22 DOF, human-size | Per fingertip: **>1,000 taxels + a miniature camera** (visuo-tactile, "DTA"), 0.005 N precision, 6-D force **[confirmed]** | — | Mass production from Oct 2025 |
| **Shadow DEX-EE** (with Google DeepMind) | 3 fingers, robust | Distal: **stereo camera tactile tip, 640×480 @ 50 FPS**. Middle and proximal phalanges: **36 taxels per finger**, 3-axis **[confirmed]** | Position, force, IMU data **[confirmed]** | Research product |
| **PSYONIC Ability Hand** | Motor + linkage, prosthetic/research | **Fingertip pressure sensors** (6 per finger in their docs **[background]**). Their early paper used a **barometric pressure sensor + IR proximity sensor on a PCB** **[confirmed]** | Motor current | Commercial ($25–50k as prosthesis) |
| **Meta Digit 360** (Oct 2024, with GelSight) | Fingertip only | Camera-based, **~8.3 M "taxels"**, detects **~1 mN**, 18+ modalities (vibration, heat, even smell), on-device AI. Open-sourced design (`facebookresearch/digit360`) **[confirmed]** | — | Research sensor |
| **Wuji Hand 2** | 20 DOF, **direct drive** (motor in each joint), 580 g | **24×32 = 768-point tactile map**, 30 Hz streaming for imitation learning **[confirmed]** | 6-axis IMU **[confirmed]** | Product |
| **Inspire RH56 series** | 6 actuators, linkage | Piezoresistive: RH56DFTP has **1,062 taxels on 17 pads**; RH56H1 260; RH56E2 resistive (17 sensors, 0–30 N) or capacitive (5 sensors, 0–20 N). **Normal force only, no shear** **[confirmed]** | Actuator force | Very common in research (cheap) |
| **Sanctuary Phoenix** | Hydraulic hand, 21 DOF | Tactile sensing on the hand (they have shown touch-driven blind grasping) **[background, not re-verified]** | — | Research/pilot |

**Patterns.**
1. **Everyone at the top has fingertip touch, and most have shear or slip.** Pressure-only arrays (Inspire) are the budget tier.
2. **Two design schools:** (a) *camera-based* tips (Sharpa, DEX-EE, Digit 360) give very high resolution but are bulky, power-hungry and heavy on data. (b) *skin-based* (1X, Figure, Wuji) are thin and durable, with lower resolution.
3. **Tendon hands with transparent drives (1X) lean on motor-side force sensing.** That is the "free" sense Tendra can use first.
4. **Palm cameras (Figure 03)** are a cheap, underrated trick: they give close-range vision while the head camera is blocked.

---

## 2. Tactile technologies compared

Fingertip target for Tendra: ~15–20 mm wide, ~20–25 mm long. Wires must go through the finger and past four joints to the palm.

| Technology | How it works | Examples | Cost per fingertip | Resolution / signals | Wiring | Rate | Durability | ESP32-S3 fit | Fits 15–20 mm tip? |
|---|---|---|---|---|---|---|---|---|---|
| **Vision-based (gel + camera)** | Camera films a soft gel with a reflective skin; depth from shading/markers | GelSight, GelSight Mini, DIGIT, Digit 360, DTact, 9DTact | DIY 9DTact/DIGIT: ~$15–50 in parts **[background]**; GelSight Mini ~$500 **[background]** | Very high: contact shape, shear (markers), force by learning | USB or CSI cable per tip (thick) | 25–60 fps **[background]** | Gel wears and tears; replaceable | **No** (needs USB host/PC; ESP32-S3 can do one small camera at low fps) | Barely: DIGIT is ~20 × 27 mm plus depth **[background]**; too big for Tendra's distal phalanx |
| **Magnetic skin** | Magnetised elastomer over 3-axis magnetometers; deformation changes the field | **ReSkin** (2021), **AnySkin** (2024) | ReSkin: **< $30** per sensor **[confirmed, ReSkin paper]** | 3-axis per chip (normal + shear). ReSkin/AnySkin board: **5 × MLX90393** (4 spaced 7 mm around a central one) under a 20 × 20 mm skin **[confirmed]**; AnySkin detects slip on unseen objects with **92 %** accuracy **[confirmed]** | I²C (4 wires, shared) | ReSkin streams all 5 chips at **~400 Hz**; AnySkin collected data at 100 Hz **[confirmed]** | **Skin is replaceable in seconds, no glue** (AnySkin: 12 ± 5 s; skin 2 mm thick; ReSkin: < 3 mm, rated > 50k contacts); models transfer zero-shot to new skins **[confirmed]** | **Yes** (I²C) | **Yes**: one or several magnetometers (MLX90393 is 3 × 3 mm) under a ~2–5 mm skin |
| **Hall + single magnet (DIY)** | Small magnet in a silicone dome over one 3D Hall sensor | TMAG5273, MLX90393, many papers ("omnidirectional fingertip pressure sensor using Hall effect") | **~$2–6** | 1 "taxel" with 3-D force (normal + shear) | I²C | 100 Hz – 1 kHz | Good; silicone can be recast | **Yes** | **Yes**: easiest option |
| **Piezoresistive film / FSR** | Resistance drops under pressure | Velostat, Interlink FSR 402 (12.7 mm active area **[background]**), eTextile, Inspire, 3D-ViTac | FSR ~$5–8; Velostat sheet ~$5 for many tips | Normal force only; drift, hysteresis, ±10–25 % | Analog: 1 ADC pin per cell, or row/column matrix | kHz possible | Films crease and fatigue | Yes, but ADC channels are scarce (see §5) | Yes (thin), but flat pads on curved tips are awkward |
| **Capacitive** | Gap between electrodes changes | Inspire T2, commercial skins (PPS, Xela uSkin is actually magnetic) | Chips cheap (e.g. MPR121 12-channel touch IC ~$2 **[background]**); a good force sensor is hard to DIY | Normal (with shear in advanced designs) | I²C via capacitance-to-digital IC | 100 Hz+ | Good if sealed | Yes; the ESP32-S3 also has built-in touch pins | Yes, but drift with humidity/temperature |
| **Barometric (MEMS in rubber)** | A tiny air-pressure sensor potted in silicone reads pressure through the rubber | **TakkTile** (Harvard/Right Hand Robotics, used MPL115A2 **[background]**), PSYONIC (barometer on PCB **[confirmed]**), BMP280/BMP390/LPS22 | $1–5 per chip | Normal force, very sensitive, good vibration → slip **[background: "Under Pressure", arXiv 2103.13460]** | I²C/SPI; BMP280 has only 2 addresses → mux | 50–200 Hz (up to ~1 kHz some parts) | Robust once potted | **Yes** | **Yes** (2 × 2 mm packages) |
| **PVDF / piezo film** | Voltage when strain *changes* | PVDF strips, piezo discs | $1–10 | Vibration only (no static force); great for slip onset, textures | Analog + charge amp | kHz | OK | Needs amplifier + fast ADC | Yes (thin) |
| **Accelerometer/IMU in tip** | Detects contact transients | Any MEMS IMU | $2–5 | Contact events, slip vibration | I²C/SPI | kHz | Excellent | Yes | Yes |

### Slip detection in one paragraph
Slip shows up as (1) **shear rising faster than normal force** (ratio near the friction coefficient), or (2) **small high-frequency vibrations** as the surface starts to move. So you want either a shear-capable sensor (magnetic, gel camera) or a fast one (barometer, PVDF, IMU). AnySkin showed learned slip detection with cheap magnetic skins **[confirmed]**, and "Under Pressure" did the same with barometric sensors **[background]**. The classic control loop (Romano et al., 2011, PR2 gripper **[background]**) is: grip lightly, detect slip, then raise grip force by a set factor.

**Takeaway for Tendra:** a **magnetic** fingertip (one Hall/magnetometer chip and a magnet or magnetised skin) is the best value. It is cheap, uses I²C, fits 15–20 mm, measures shear (slip), and the skin is replaceable. Camera tips are for later and only on a PC-connected research finger.

---

## 3. Proprioception: knowing the real joint angle and force

### 3.1 Why Tendra needs more than servo position
The SCS0009 encoder measures the **spool**, not the joint. In between sit tendon stretch (fishing line and PTFE sheath compression), slack, friction and coupling across joints. Under load the joint lags the spool, so the error grows exactly when grasping. Two fixes: **measure at the joint**, or **model the tendon** (stiffness and friction identified per joint) and correct with servo load. Both can be combined.

### 3.2 Joint angle sensors

| Sensor | Principle | Interface | Size | Notes for Tendra |
|---|---|---|---|---|
| **AS5600** | Magnetic 12-bit angle, diametric magnet on axis | I²C (**fixed address 0x36**), analog or PWM out **[background]** | SOIC-8 + 6 × 2.5 mm magnet | Cheap (~$1–3 on boards), but the fixed address means one per bus → needs a mux or analog mode. Board too big for fingers; the bare chip fits the MCP. |
| **AS5048A/B** | Magnetic 14-bit | SPI (A) / I²C (B, few addresses) **[background]** | TSSOP-14 | More accurate, pricier (~$10+). SPI allows daisy-chaining. |
| **MLX90393** | 3-axis magnetometer | I²C: **4 addresses per part set by pins A0/A1** (e.g. 0x0C–0x0F); factory variants use different base addresses (newer breakouts ship at **0x18**), so check the part number; or SPI. Burst mode up to **~717 Hz** (OSR 0, no digital filter), ~500 Hz with light filtering **[confirmed, Melexis datasheet / Adafruit]** | 3 × 3 mm QFN | Can read angle from an off-axis magnet; the same chip is used in ReSkin/AnySkin. |
| **TMAG5273** | 3D linear Hall, built-in angle calculation | I²C: factory-default address set by the variant letter (**A = 0x35, B = 0x22, C = 0x78, D = 0x44**) plus a **user-programmable address register**; up to **10 kSPS for 3 axes** (20 kSPS single axis) without averaging **[confirmed, TI datasheet via search]** | SOT-23-6 (tiny) | **Best fit for small joints:** tiny, cheap (~$1–2), angle in hardware, address set at boot. |
| **Linear analog Hall** (DRV5055, SS49E) + magnet | Field strength vs angle | Analog | SOT-23 / TO-92 | Cheapest, but needs an ADC channel each and calibration; nonlinear. |
| **Potentiometer** | Resistive | Analog | Bulky | Wear, friction, size. Fine for the exoskeleton glove idea, not for fingers. |
| **Flex sensor** (bend resistor) | Resistance vs bend | Analog | Thin strip | Hysteresis and drift; common in data gloves; poor accuracy on a robot joint. |

**How other open hands do it [background, verify]:**
- **LEAP Hand** (CMU, 2023): Dynamixel servos directly at the joints, so servo position ≈ joint position. No tendon, no extra sensors.
- **ORCA Hand** (ETH, 2025): tendon-driven with servos in the base, similar to Tendra. It relies on servo position plus an **automatic calibration** routine that drives joints to their flexion/extension limits under a current limit, detects the stall positions and fits a linear motor-to-joint ratio from the CAD range of motion **[confirmed, arXiv 2504.04259]**. Tactile: **binary FSR fingertip sensors** (RP-C7.6-ST thin-film, 5 fingertips, trigger ≥ 0.05 N); their Ø0.2 mm copper wires snapped after 4 500–7 000 cycles and the silicone skin wore after 2 000–4 000 cycles **[confirmed]**, a warning for Tendra's wiring through joints.
- **Shadow Dexterous Hand** (tendon-driven): **Hall-effect sensors at every joint** plus tendon-tension sensing at the motors. This is the reference design for exactly Tendra's problem.
- **1X NEO**: transparent low-ratio drive, so motor-side sensing reflects outside forces **[confirmed]**.

### 3.3 Getting many sensors through the wrist
20 joint sensors + 5–15 tactile chips are too many for separate wires. Options:

| Option | Wires through wrist | Pros | Cons |
|---|---|---|---|
| **I²C bus + TCA9548A mux** (8 channels, ~$1–5) in the palm | 4 (3V3, GND, SDA, SCL) | Simple, well supported | I²C dislikes long cables (keep < ~30–50 cm, 100–400 kHz); one faulty sensor can hang a branch |
| **Two I²C buses from the ESP32-S3** (it has 2 controllers) | 6 | Doubles bandwidth | Still needs muxing for fixed-address chips |
| **SPI chain** (e.g. AS5048A daisy-chain) | 5–6 | Fast | More wires, chip-select management |
| **Small MCU in the palm** (e.g. RP2040, STM32G0, CH32V003, or a second ESP32-C3) reads everything locally | **4** (power + one serial/CAN/RS-485 pair) | Short sensor wires, robust link, local filtering and timestamps, **scales** | Another firmware to maintain |
| **Share the SCS servo bus** **[speculative]** | 0 extra signal wires | Palm MCU answers as extra "servo IDs" (e.g. 30+) using Feetech's packet format | Adds traffic to a 1 Mbps half-duplex bus already carrying 20 servos; risky |

**Recommendation:** a **palm (or proximal-forearm) sensor MCU** with local I²C/mux branches to each finger, and a separate UART, RS-485 or CAN link to the main ESP32-S3. It is the standard industrial pattern and keeps the servo bus clean.

### 3.4 Tendon tension and force
- **Servo load as a force estimate (free):** SCS0009 reports load and position. With the moment arms already known from `tendon_router.py` and MuJoCo, joint torque ≈ spool radius × tendon tension, and fingertip force follows from τ = Jᵀ F (see `research/ai/ideas.md`). Limits: the load reading is coarse, includes gear and sheath friction (PTFE sheaths with S-bends add hysteresis), and SCS0009 backs off after 2 s above 80 % load. **Calibrate it** on a bench: hang known weights on a strand and record load vs tension, both directions (friction shows up as the gap between loading and unloading).
- **In-line load cells / strain gauges:** accurate but bulky (HX711 + mini load cell, ~$3–5 each, 10–80 Hz **[background]**). Fine for **one test strand on the bench** to calibrate the servo-load model; not for 40 strands.
- **Joint angle + tendon model:** with joint sensors (§3.2), *spool angle − joint angle × radius* = tendon stretch, and stretch × stiffness = tension. This is **a tension sensor with no extra hardware** once joint sensors exist **[speculative but standard series-elastic reasoning]**.

---

## 4. How learning uses touch

- **Imitation / diffusion policies with touch:** e.g. **HATO** (2024, bimanual PSYONIC hands with touch), **3D-ViTac** (2024, low-cost piezoresistive tactile arrays fused with vision into a point cloud), Reactive Diffusion Policy (2025, fast tactile loop). They report clear gains on contact-rich tasks where vision is blocked or ambiguous **[background, verify numbers]**. Tactile imitation papers continue into 2026, e.g. *Touch2Trace* (cable tracing, 2026) and DECO's plug-in tactile adapter for bimanual dexterous diffusion transformers (2026) **[confirmed titles in search results]**.
- **VLA models with touch (2025–2026):** Tactile-VLA, ForceVLA, VTLA and similar add a tactile/force token stream to a vision-language-action model **[background]**. The 2026 survey *Tactile-based Multimodal Fusion in Embodied Intelligence* (arXiv 2605.17336) maps the field **[confirmed title]**. Cross-sensor representation work (TactX, Tac-DINO, 2026) tries to make tactile features transfer between different sensors, and FELT (2026) *generates* tactile signals from vision **[confirmed titles]**. That matters for a hobby hand: you may train on others' data or on simulated touch.
- **Dataset format:** Wuji streams 768 taxels at 30 Hz for imitation data **[confirmed]**. For Tendra, add tactile channels as extra `state` columns in `tendra.dataset` / LeRobot (a few dozen floats per frame is trivial).
- **Simulating touch in MuJoCo:** built-in `touch` sensor (scalar normal force inside a site volume) and the **`touch_grid` sensor plugin** (a taxel grid on a site) **[background]**. Cheap proxies: contact forces per fingertip geom from `mj_contactForce`. For magnetic or gel sensors, sim-to-real needs either calibration to force (then use sim forces) or learned models (Taccel, TacSL in Isaac for gel sensors **[background]**). **Practical rule:** give the RL policy *force-level* tactile features (normal + shear per fingertip), not raw sensor readings, so sim and real can match.
- **Slip and grip-force control** is a low-level reflex, not a learned policy: it runs on the ESP32 at 100–500 Hz and raises grip when shear/normal nears friction or vibration spikes. The high-level policy sets "hold this object", and the reflex keeps it from slipping. This is how humans split the work too.

---

## 5. Staged sensing plan for Tendra

ESP32-S3 facts used below **[background]**: two I²C controllers (any GPIO via the GPIO matrix), two free SPI hosts, three UARTs (UART1 = servo bus on GPIO 17/18 in `config_v1.h`), ADC1 on GPIO 1–10 (ADC2 conflicts with Wi-Fi), native USB on GPIO 19/20 (never touch). Pins below are **suggestions** for the V1 build (no stepper pins in use there); check them against `config_v1.h` before wiring.

### Stage 1: "free" force sense + one fingertip test (≈ $15–40, a weekend or two)
**Goal:** contact detection and rough grip force now, and learn how touch data looks.

1. **Servo telemetry at 50–100 Hz** (position, load, speed) for all 20 servos: firmware `F` command → streaming mode. Log it in `tendra.dataset` as state.
2. **Bench calibration of load → tendon tension:** one strand, a luggage scale or known weights (optionally a 1–5 kg load cell + HX711, ~$5). Fit load = k·tension + friction(direction). Also measure **tendon stretch** vs tension (spool angle vs a protractor on the joint).
3. **Contact detector in software:** compare measured load with what MuJoCo predicts for free motion; a residual above a threshold = contact. Estimate fingertip force via τ = Jᵀ F.
4. **One DIY magnetic fingertip** (index): **TMAG5273** or **MLX90393** breakout (~$5–15) under a cast silicone dome (Smooth-On Ecoflex/Dragon Skin, ~$30 kit shared with later stages) with a 2–3 mm neodymium magnet. 3-axis field → normal + shear after calibration with a kitchen scale. I²C0 on e.g. **GPIO 8 (SDA) / GPIO 9 (SCL)**, 400 kHz, 4 wires down the finger.
5. **Firmware:** read the sensor at 100–200 Hz, send compact binary or text lines (`T <id> bx by bz`); bump the protocol version.

### Stage 2: all five fingertips + joint angles + palm camera (≈ $60–150)
**Goal:** reliable touch on every fingertip, true joint angles, close-range vision.

1. **Five magnetic fingertips** (AnySkin-style skin or the Stage 1 dome design). Chips ~$2–10 each.
2. **Joint angle sensors, starting where error is worst:** MCP flex and MCP abduction of each finger and the thumb CMC joints (largest loads, longest tendon path). **TMAG5273** (SOT-23, programmable address) on a tiny flex PCB or perfboard at each joint, with a small diametric magnet in the child segment's drum axis. Add PIP/DIP later if space allows. Keep joint magnets ≥ ~10 mm away from fingertip sensors, or calibrate out the crosstalk **[speculative, test it]**.
3. **Palm sensor MCU** (RP2040 or ESP32-C3 board, ~$3–6) with a **TCA9548A** (~$2) giving one I²C branch per finger. It links to the main ESP32-S3 over UART2 (e.g. **GPIO 15/16**) or RS-485 (~$1 transceiver) at 1–2 Mbps. Through the wrist: 4 wires (5 V, GND, TX, RX).
4. **Palm camera** (Figure 03 idea): a small USB camera (endoscope-type, ~$10–20) straight to the PC. It needs no ESP32 work and adds a strong input for vision policies.
5. **Software:** joint-angle fusion (joint sensor + spool encoder → tendon stretch → tension), a **slip reflex** on the ESP32 (shear/normal ratio + vibration → raise grip), tactile channels in datasets and in `tendra.joints`/`HandSpec`, and a MuJoCo tactile proxy (fingertip contact force + shear) in `GraspEnv` observations.

### Stage 3: rich skin and research-grade tips (≈ $150–600)
**Goal:** whole-hand touch for in-hand manipulation and VLA training.

1. **Phalanx and palm pads:** piezoresistive matrix (Velostat or eTextile rows/columns, 3D-ViTac-style) or multi-chip magnetic skins (2–5 magnetometers per pad) read by the palm MCU.
2. **Multi-chip magnetic fingertips** (e.g. 3–5 MLX90393 per tip, as in ReSkin/AnySkin) for contact location, not just force.
3. **One camera-based research tip** on a test finger (DIY 9DTact/DIGIT-style, or a GelSight Mini on a gripper) connected by USB to the PC, only if the distal phalanx can be redesigned larger.
4. **Fingertip IMU or PVDF** for fast slip/texture vibration if Stage 2 slip detection is too slow.
5. **Learning:** tactile-conditioned diffusion/VLA policies; pretraining on public tactile datasets; sim tactile via `touch_grid` or learned sim-to-real mappings.

### Summary table

| Stage | Adds | Approx. cost | Bus / pins | Main payoff |
|---|---|---|---|---|
| 1 | Servo load → force model; 1 magnetic fingertip | $15–40 | UART1 (existing); I²C0 GPIO 8/9 | Contact detection, grip force estimate, first touch data |
| 2 | 5 fingertips; ~8–12 joint angle sensors; palm MCU; palm camera | $60–150 | UART2 GPIO 15/16 (or RS-485) to palm MCU; USB camera to PC | True joint angles, slip reflex, better teleop/RL data |
| 3 | Pads on phalanges/palm; multi-taxel tips; optional camera tip | $150–600 | Palm MCU branches; USB for camera tip | In-hand manipulation, tactile VLA research |

Costs are **[speculative]** hobby-market estimates (AliExpress/Adafruit/Mouser-class prices, 2025–2026).

---

## 6. Open questions to test

- How accurate is the SCS0009 load reading as a tension sensor through the PTFE sheaths (hysteresis, resolution)?
- How much does the 0.4 mm line stretch at typical grip tension? That tells whether joint encoders are a must or a nice-to-have.
- Can a TMAG5273 + magnet fit in the MCP and PIP drums without weakening them?
- Magnetic crosstalk between joint magnets and fingertip magnetic sensors.
- Do the sensor wires survive 10⁵ flex cycles through the joints? Use thin silicone-insulated or flex-PCB wiring, routed on the joint axis like the tendons.

---

## Sources

Confirmed during this research (search results / pages):
- Figure 03: https://www.figure.ai/news/introducing-figure-03 · https://www.therobotreport.com/figure-ai-designs-figure-03-humanoid-ai-home-use-scaling/
- 1X NEO hands: https://www.1x.tech/discover/neos-hands · https://interestingengineering.com/ai-robotics/1x-unveils-robot-hands-neo-humanoid · https://embodiedglobal.com/en/article/1x-neo-new-25dof-tendon-driven-hands-ip68-force-transparency-2026
- Tesla Optimus Gen 3 (press/blog level): https://www.basenor.com/blogs/news/tesla-optimus-gen-3-hands-22-dof-50-actuators-explained
- Sharpa Wave: https://www.prnewswire.com/news-releases/ai-robotmaker-sharpa-reaches-key-milestone-with-mass-production-of-worlds-most-advanced-human-sized-robotic-hand-302643434.html · https://interestingengineering.com/ai-robotics/sharpas-advanced-robotic-hand-enters-mass-production
- Shadow DEX-EE: https://shadowrobot.com/dex-ee_series/ · https://www.therobotreport.com/shadow-robot-dex-ee-hand-takes-manipulation-to-next-level/ · https://shadowrobot.com/sensors/
- PSYONIC: https://www.unb.ca/ibme/_assets/documents/past-docs/MEC17-papers/akhtar-the-psyonic-compliant.pdf
- Digit 360: https://ai.meta.com/blog/fair-robotics-open-source/ · https://github.com/facebookresearch/digit360 · https://www.therobotreport.com/gelsight-meta-ai-release-digit-360-tactile-sensor-for-robotic-fingers/
- Wuji Hand 2: https://www.wuji.tech/en/hand2 · https://docs.wuji.tech/docs/en/wuji-hand/latest/overview/ · https://www.roboticscenter.ai/hardware/wuji-hand
- Inspire RH56: https://en.inspire-robots.com/wp-content/uploads/2025/01/INSPIRE-ROBOTS-The-Dexterous-Hand-RH56DFTP-User-Manual-V1.0.0.pdf · https://www.knoxlabs.com/products/inspire-robots-rh56h1-dexterous-hand · https://www.knoxlabs.com/products/inspire-robots-rh56e2-dexterous-hand
- AnySkin: https://arxiv.org/abs/2409.08276 · https://any-skin.github.io/
- Barometric slip detection: https://arxiv.org/pdf/2103.13460
- Hall-effect fingertip: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8587916/
- Tactile SoftHand-A (3D-printed tendon hand with touch): https://arxiv.org/pdf/2406.12731
- 2026 tactile learning: https://arxiv.org/pdf/2605.17336 (survey) · https://arxiv.org/pdf/2606.31236 (TactX) · https://arxiv.org/pdf/2606.12069 (Tac-DINO) · https://arxiv.org/pdf/2607.20683 (FELT) · https://arxiv.org/pdf/2609.15921 (Touch2Trace) · https://arxiv.org/pdf/2602.05513 (DECO)

Background (well known, not re-fetched this session; verify before relying on specs):
- ReSkin: https://arxiv.org/abs/2111.00071
- DIGIT: https://arxiv.org/abs/2005.14679 · GelSight Mini: https://www.gelsight.com/gelsightmini/
- 9DTact: https://arxiv.org/abs/2308.14277 · DTact: https://arxiv.org/abs/2209.13916
- TakkTile: https://www.takktile.com/
- 3D-ViTac: https://arxiv.org/abs/2410.24091 · HATO: https://arxiv.org/abs/2404.16823
- LEAP Hand: https://leaphand.com/ · ORCA Hand: https://www.orcahand.com/ (ORCA now confirmed: https://arxiv.org/html/2504.04259)
- Romano et al., "Human-inspired robotic grasp control with tactile sensing", IEEE T-RO 2011
- MuJoCo sensors: https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor
- Datasheets: AS5600 (ams-OSRAM), MLX90393 (Melexis), TMAG5273 (TI), TCA9548A (TI)

Sources added in the 2026-09-30 fact-check:
- ReSkin full paper (5 × MLX90393, 400 Hz, < $30, < 3 mm): https://arxiv.org/pdf/2111.00071
- AnySkin full text (same 5-magnetometer board, 2 mm skin, 100 Hz, 92 % slip, 12 s swap): https://arxiv.org/html/2409.08276
- MLX90393 datasheet: https://www.melexis.com/-/media/files/documents/datasheets/mlx90393-datasheet-melexis.pdf ; address note: https://github.com/adafruit/Adafruit_CircuitPython_MLX90393/issues/46
- TMAG5273 datasheet: https://www.ti.com/lit/ds/symlink/tmag5273.pdf
- ORCA hand (FSR tips, calibration): https://arxiv.org/html/2504.04259

Fact-checked 2026-09-30 (partial): ReSkin and AnySkin (chip, count, rate, cost, skin), MLX90393 and TMAG5273 (I2C addresses, rates), ORCA sensing and calibration. Still [background]: AS5600/AS5048, GelSight/DIGIT/9DTact numbers, FSR/barometer details, PSYONIC taxel count, Sanctuary, MuJoCo touch_grid, ESP32-S3 peripheral facts.
