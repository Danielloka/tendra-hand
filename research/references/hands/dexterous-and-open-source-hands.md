# Dexterous, research and open-source robot hands: survey for Tendra Hand V1

*Research note, 2026-09-30. English. License: CC BY 4.0 (see `docs/LICENSE`).*

**Why this note exists.** Tendra Hand V1 is a 20-DOF, tendon-driven, 3D-printed hand with 16 Feetech
SCS0009 servos (each finger's DIP is linked to its PIP by a rigid bar) in the forearm, one antagonistic loop (flex + extend strand) per joint on a 6 mm drum,
6 mm servo spools, 0.4 mm line in 1 × 2 mm PTFE tubes. This note surveys other hands to find what we
should copy, what to avoid, and whether the SCS0009 is strong enough.

**How to read the tags.**
- **[C]** = confirmed from a source read for this note (link in Sources).
- **[P]** = taken from the project's earlier research note `research/experiments/2026-09-28-full-hand/research.md`, which cited its own sources.
- **[U]** = from memory / secondary sources, **not verified** for this note. Check before relying on it.
- Numbers without a tag are our own calculations.

> **Limitation of this note:** during this research session the web tools failed repeatedly
> (tool errors, not missing pages), so many specs could not be re-checked against primary sources.
> Everything not verified is marked **[U]**. A follow-up pass should verify the [U] rows, starting with the
> hands in the "closest to Tendra" group (ORCA, RUKA, Amazing Hand, Aero Hand Open, DexHand).

---

## 1. The short version

1. **Nobody else drives 20 DOF with SCS0009-class servos through tendons.** The closest tendon hands
   use Dynamixel XC330/XL330-class servos (ORCA v1: XC330-T288-T; RUKA: XL330-M288-T + XM430 thumb;
   LEAP v2 Adv: XC330-M288-T) or the Feetech **HLS3606M** (Aero Hand Open, 7 servos) [C, 2026-09-30].
   Stall torques: XL330 0.52 N·m @ 5 V, HLS3606M 0.59 N·m @ 6 V, XC330-M288 0.93 N·m @ 5 V, i.e.
   **2.3–4× an SCS0009** (0.226 N·m @ 6 V) [C]. Amazing Hand, the one hand built on SCS0009, uses
   **two SCS0009 per finger working together** and **short linkages, not tendons** [C], and its authors
   still warn that grasping needs "smart software" to protect the servos [C].
2. **Force is our main risk, not travel or DOF count.** With a 6 mm spool, SCS0009 stall
   (≈0.23 N·m) gives ≈38 N of tendon force and ≈2–3 N at the fingertip at stall, ≈1 N continuous,
   before friction [P]. That is enough to show motion, pinch light objects and do teleop/RL demos,
   not enough for tool use.
3. **The best open-source tendon hand to copy is ORCA** (ETH Zurich, 2025): 17 DOF, tendon-driven,
   pop-out joints that dislocate instead of breaking, auto-calibration, a quick tensioning system,
   tendons that only touch metal and PTFE, >10 000 cycles (~20 h) without hardware failure,
   < 2 000 CHF [C].
4. **Commercial hands reduce actuators** (Inspire, PSYONIC: 6 motors for 10–12 joints [U]) or put
   many strong motors in a big forearm (Shadow: 20 motors, ~4 kg [U]). Full-DOF hands with motors
   in the palm (Wuji, Sharpa, Tesollo) use custom micro gearmotors [U]. A hobby project cannot copy
   those, but it can copy the **coupling tricks** (DIP coupled to PIP).
5. **Newest trend (2025–2026):** learning-first hands. RUKA learns a joint→actuator map from
   MoCap-glove data instead of modelling tendons [C]; Aero Hand Open ships as a "simulation-ready"
   tendon hand [C]; papers now model tendon forces and cable routing in MuJoCo for sim-to-real
   (MuJoCable, tendon-force modelling) [C: titles]. Tendra already has the sim side, which is a strength.

---

## 2. Comparison table

DOF = total joints; "act." = actuated (independently driven) joints. Weights and costs as published;
blank or "?" = not found.

| Hand (year) | Open? | DOF / act. | Actuators | Transmission | Joints | Sensing | Force / weight / cost | Tag |
|---|---|---|---|---|---|---|---|---|
| **Tendra V1** (design) | Yes | 20 / 20 | 20× Feetech SCS0009 (forearm) | Antagonistic tendon loop per joint, 0.4 mm line, PTFE tubes | Printed pin joints | Servo position/load/temp only | ≈2–3 N tip at stall (calc.), ? g, low-cost | — |
| **ORCA** (ETH SRL, 2025) | Yes | 17 / 17 (DIP rigidly coupled to PIP) | v1: 16× Dynamixel XC330-T288-T + 1× XC430-T240BB-T (wrist, GT2 belt) in a tower below the hand; newer Feetech variant: HLS3915 fingers + HLS3930 wrist | Flexor + extensor tendon per joint, 0.4 mm braided nylon fishing line, metal pins + PTFE tubes, ratchet spools | Pin joints with bearings in arc-shaped grooves that **pop out** under overload | FSR fingertip sensors (RP-C7.6-ST, binary contact) | < 2 000 CHF, ~1.2 kg, < 8 h assembly, >10 000 cycles (~20 h) | [C] paper + orca_core |
| **Faive / "Getting the ball rolling"** (ETH SRL, 2023) | Partly | ~11 act. [U] | Dynamixel [U] | Tendons | **Rolling-contact** joints | — | — | [C] existence, [U] specs |
| **LEAP Hand v1** (CMU, 2023) | Yes | 16 / 16 | 16× Dynamixel XC330 [U] | **Direct drive** (motor in joint) | Motor = joint; "universal" MCP abduction | Motor current | < $2 000 [U] | [P]/[U] |
| **LEAP Hand v2** (CMU, 2024) | Yes | ~16–17 [U] | Dynamixel [U] | Hybrid: motors + tendons for soft fingers [U] | 3D-printed soft skin over rigid bones | — | — | [C] existence, [U] specs |
| **LEAP Hand v2 Adv** (CMU, 2025) | Yes | 21 / 17 (PIP–DIP coupled by one tendon per finger) | 17× Dynamixel XC330-M288-T | Hybrid rigid-soft; rigid MCP, tendon-coupled PIP/DIP; **two powered palm articulations** (fingers + thumb) | Soft printed exterior (TPU) + printed bones (PLA) | — | ≈ $3 000 | [C] |
| **Amazing Hand** (Pollen / Hugging Face, 2025) | Yes | 8 / 8 (4 fingers × 2) | **8× Feetech SCS0009** (2 per finger, in the hand) | Two servos in **parallel/differential** per finger, ball-joint linkages; proximal+distal phalanx coupled by linkage | Printed + flexible TPU shells | Servo position/load/temp | **400 g, < €200** | [C] |
| **RUKA** (NYU, RSS 2025) | Yes | 15 / 11 (underactuated) | Dynamixel XL330-M288-T (fingers) + XM430-W210-T (thumb) | Tendons of **braided fishing line rated 200 lb** | Rigid hinge joints, soft pads | — | < $1 300, pinch 2.74 N, 6 kg payload, 20 h continuous run | [C] |
| **RUKA-v2** (NYU, 2026) | Yes | 15 (13 hand + 2-DOF parallel wrist), adds finger abduction | Dynamixel [U] | Tendons | — | — | < $1 300 | [C] |
| **Aero Hand Open** (TetherIA, 2025–26) | Yes | 16 / 7 (3-DOF thumb) | **7× Feetech HLS3606M** | Tendons of braided Kevlar/Vectran over pulleys; PIP–DIP coupling cable; thumb joints coupled; backdrivable | Printed | Servo current/position | ~12 N per finger, 374–400 g, **US$314 BOM** (servos ≈ $209) | [C] |
| **DexHand** (The Robot Studio) | Yes | ~15–16 act. [P] | 16 slim micro servos in forearm; Feetech SCS2332/SCS15 on wrist | Tendons, Sufix 832 braided PE (80 lb) | Printed | — | Hobby cost | [P] |
| **InMoov hand** (G. Langevin) | Yes | 5 act. (1 per finger) | Hobby PWM servos in forearm | Pull-pull braided line on the horn, printed channels | Printed pins | — | Very low cost | [U] |
| **Brunel Hand 2.0** (Open Bionics) | Yes (CC) | 4 act. [U] | Linear actuators [U] | Tendons, flexible printed joints [U] | Flexure (printed TPU) [U] | Force sensors (some versions) [U] | ~£1 000+ [U] | [U] |
| **Yale OpenHand** (T42, Model O, Q) | Yes | Underactuated grippers | Dynamixel | Tendons, one motor per finger | **Cast urethane flexures** | — | Low-cost | [U] |
| **MM-Hand** (2026) | Paper | 21 | Remote Feetech motor hub | Tendons in sheaths; compared PTFE vs Bambu feeder tube vs spring tube | — | — | — | [P] |
| **Antagonistic Bowden hand** (2025) | Paper | — | One Dynamixel per joint, both strands on one shaft | PTFE 1/2 mm + spring jacket, 0.75 mm coated steel | — | — | — | [P] |
| **CRAFT** (2026) | Paper | ? | ? | Tendon-driven, hybrid hard-soft compliance | Hard-soft | — | — | [C] title |
| **DexLink** (2026) | Paper | 16 | ? | **Linkage**-driven | — | — | "Compact, affordable" | [C] title |
| **Shadow Dexterous Hand** | No | 24 / 20 (J1–J2 coupled) | 20 small motors in forearm (or pneumatic muscles, older) | **One spool per joint, agonist + antagonist Spectra tendons**, split spool preload, tensioners, 90° over a bar with load cell | Metal pins | Joint Hall sensors, tendon force, optional tactile | 65 N cont. / 110 N max tendon; ~4 kg [U]; ~$100k+ [U] | [P] transmission, [U] rest |
| **Shadow DEX-EE** (with Google DeepMind) | No | 3 fingers, 12 act. [U] | [U] | Tendons [U] | Built for robustness in RL [U] | Heavy tactile [U] | — | [U] |
| **Allegro Hand V4 / V5** | No | 16 / 16 | 16 DC motors in the joints | Direct gear | Metal | Pot./encoders | ~1.1–1.5 kg [U] | [U] |
| **Inspire RH56 (DFX/DFQ)** | No | 12 / 6 | 6 micro linear actuators | **Linkages** | Pins | Force per actuator | ~540 g, very cheap for class [U] | [U] |
| **PSYONIC Ability Hand** | No | 10 / 6 | 6 motors | Linkages | **Fingers pop out and snap back** on impact | Fingertip pressure sensors | ~500 g, closes in ~0.2 s [U] | [U] |
| **Tesollo DG-5F** | No | 20 / 20 | 20 motors in the hand | Direct / gear | Metal | Current, optional F/T | ~1.4 kg [U] | [U] |
| **Wuji Hand** (2025) | No | 20 / 20 | 20 motors in the palm | Micro gear drives | Metal | — | — | [U] |
| **Sharpa Wave** (2025) | No | 22 [U] | In-hand | ? | — | Dense tactile (claimed) | — | [U] |
| **Schunk SVH** | No | 20 / 9 | 9 DC motors | Linkages / coupling | Metal | Encoders | ~1.3 kg [U] | [U] |
| **DLR Awiwi** (Hand Arm System) | No | 19 | ~38 motors (two per joint, variable stiffness) | Dyneema tendons | Rolling/pin | Rich | Research only | [P]/[U] |

---

## 3. Open-source hands in detail

### 3.1 ORCA (ETH Zurich Soft Robotics Lab, 2025) — closest to Tendra

**What it is.** "A reliable and anthropomorphic 17-DoF tendon-driven robotic hand with integrated
tactile sensors, fully assembled in less than eight hours and built for a material cost below
2,000 CHF" [C]. Benchmarked with teleoperation, imitation learning and **zero-shot sim-to-real RL** [C].

**Actuation and routing** [C, paper + `orca_core`, 2026-09-30]:
- v1: 16× Dynamixel XC330-T288-T + 1× XC430-T240BB-T (wrist) in a tower **below** the hand (like our
  forearm). The paper itself names no servo model. The current `orca_core` also supports a Feetech
  build with **HLS3915** fingers and **HLS3930** wrist (current-based position via HLS mode 0 + goal
  current; default finger current limit 900 mA because "HLS3915 fingers stall at 1.5 A"). Some
  third-party pages (e.g. roboticscenter.ai: "17 × STS3215") are wrong.
- **Two tendons per joint** (flexor + extensor): the same antagonistic scheme as Tendra.
- DIP of fingers 2–5 is **rigidly coupled to the PIP**, not separately driven (saves a motor per
  finger). Fingers: ABD, MCP, PIP (3 DOF); thumb: CMC, ABD, MCP, IP (4 DOF); wrist: 1 DOF by GT2 belt.
- "Fishing lines (Nylon fibers braided into a 0.4 mm diameter rope)"; tendons turn around **smooth
  metal pins and rods**, **PTFE (Teflon) tubes** only for non-linear runs; tendons pass through the
  joint centre of rotation "for straightforward control at minimal slack".
- Measured: holds up to 10.5 kg with four fingers, 2 kg with the index alone; ~1.2 kg hand.
- Design rule: **every tendon only touches metal and smooth PTFE**, never printed plastic.
- **Ratchet spool** for quick re-tensioning; wrist driven by a GT2 belt.

**Reliability features** [C]:
- **Popping joints:** joints "designed to pop before breaking", so overload dislocates the joint
  instead of snapping a part or stripping a gear, "while retaining the advantages of bearing pinhole
  joints, such as stability and simple kinematics". The joint uses bearings in **circular-arc
  grooves**, so it "dislocate[s] instead of breaking when excessive radial and axial loads are applied" [C].
- **Auto-calibration** [C]: (1) drive each joint to its flexion and extension limits under a current
  limit, (2) detect the stall positions, (3) compute a linear motor-to-joint ratio from the known
  range of motion in CAD. `orca_core` re-drives a joint at higher current if the measured travel is
  short of a baseline.
- **Tensioning** [C]: a **ratchet spool** on each motor turns one way and locks the other, so a
  tendon can be re-tensioned "in seconds without the need to unscrew the spool or tendon";
  `orca_core` has a `tension` script that winds all tendons taut and holds the motors while you set them.
- **Durability:** "more than 10,000 continuous operation cycles — equivalent to approximately 20 hours —
  without hardware failure" [C]. Weak points found: the tactile sensors' Ø0.2 mm copper wires
  snapped after 4 500–7 000 cycles and the silicone skin degraded after 2 000–4 000 cycles [C].

**Lesson for Tendra:** ORCA proves that a printed, tendon-driven, antagonistic, one-motor-per-joint hand
can be reliable **if** friction points are metal/PTFE and overloads are mechanically released. Its
servos are stronger than ours (see §5).

**Predecessor.** The same lab's "Getting the Ball Rolling" (2023) hand used **rolling-contact joints**
(two curved surfaces that roll on each other, held by tendons/ligaments) and learned an RL policy for
ball rotation [C]. ORCA moved back to **pin joints with bearings**, which suggests pin joints were
easier to make accurate and stable; rolling contact is not required for a good tendon hand
(our inference).

### 3.2 LEAP Hand family (CMU, Pathak lab)

- **v1 (2023):** 16 DOF, 16 Dynamixel servos placed **directly in the joints** (no tendons) [P]; a
  "universal" MCP abduction joint that keeps sideways motion useful when the finger is bent [U].
  Very popular in research because it is cheap, simple and robust. Not comparable to our transmission.
- **v2 (2024):** hybrid rigid-soft; soft 3D-printed exterior over rigid bones [C: title]; tendon or
  hybrid actuation of the fingers [U].
- **v2 Adv (2025):** 17 DOF, ≈ $3 000, "3D printed soft exterior combined with a 3D printed internal bone
  structure", and **two powered articulations in a foldable palm**, one across the four fingers and
  one near the thumb [C].

**Lessons:** (1) a soft skin over hard bones gives both precision and friction/compliance; (2) **palm
cupping** matters for human-like grasps, and LEAP spends a motor on it. Tendra V1 has a rigid palm.

### 3.3 Amazing Hand (Pollen Robotics / Hugging Face, 2025) — the SCS0009 hand

**Facts** [C]:
- 8 DOF, **4 fingers** (thumb, index, middle, ring), 2 phalanges per finger linked together.
- **8× Feetech SCS0009**, **2 per finger**, all inside the hand. The two servos drive the finger
  **in parallel**: moving together gives flexion/extension, moving opposite gives
  abduction/adduction (a differential). Linkage with **2 ball joints**; proximal and distal phalanx
  coupled by a linkage, **no cables**.
- Rigid bones, **flexible TPU shells**, flexible palm.
- 400 g, < €200. 5 V / 2 A supply for 8 servos.
- Control: Waveshare serial bus driver + Python, or Arduino + Feetech TTL Linker. IK via a
  QP solver (Mink library).
- Reads torque (load), position and temperature back from the servos.
- Known issues: flexion angles differ from theory "due to 3D printed parts imperfection, servo horn
  rework, flexibility of plastic parts"; **not yet validated for complex prehensile tasks**, and
  "smart software" is needed before grasping to protect the servos and mechanics.
- Pollen is studying the **Feetech STS3032** (described as stronger, same volume) as an alternative.

**How they get strength from SCS0009:** (a) **two servos share every flexion** (the differential
doubles the flexion torque), (b) **short direct linkages** with no tendon friction, (c) only
**2 DOF per finger** (distal joint coupled), (d) servos sit close to the joints. Even so, they
say it is not ready for serious grasping. Tendra V1 asks one SCS0009 to drive each joint alone,
through ~0.5–1 m of tendon with friction.

### 3.4 RUKA and RUKA-v2 (NYU, Pinto lab)

- **RUKA (RSS 2025)** [C]: tendon-driven humanoid hand, 3D-printed + off-the-shelf parts, 5 fingers,
  **15 underactuated DOF**. Because tendon hands are hard to model, they **learn joint→actuator and
  fingertip→actuator models from motion-capture data** collected with a **MANUS glove**, relying on
  the hand's human-like shape. Actuators: **Dynamixel XL330-M288-T** for the fingers and
  **XM430-W210-T** for the thumb; tendons of **braided fishing line rated 200 lb**; rigid hinge joints
  "for repeatability" plus soft pads; raw materials < $1 300; ~7 h assembly; ran 20 h continuously
  without a significant drop in precision; measured pinch force 2.74 N, 6 kg payload [C, arXiv html].
- **RUKA-v2 (2026)** [C]: fully open-source; 15 DOF (13 in the hand, up from 11 in RUKA, plus a
  **decoupled 2-DOF parallel wrist**: flexion/extension and radial/ulnar deviation) and **finger
  abduction/adduction**; still < $1 300.

**Lesson:** you do not need a perfect analytic tendon model. Record (actuator command, measured
joint/fingertip pose) pairs and fit a small network. Tendra can do the same with its webcam
(MediaPipe) pipeline and the servos' position feedback.

### 3.5 Aero Hand Open (TetherIA)

"Aero Hand Open: A Simulation-Ready Tendon-Driven Hand for Dexterous Manipulation Learning"
(arXiv 2608.28578) [C]. The focus is a tendon hand shipped **with a validated simulation model**
so policies transfer. Specs [C, paper + project pages]: **7 × Feetech HLS3606M** (0.59 N·m stall,
magnetic encoder) drive **16 joints** (3 active DOF in the thumb); tendons of **braided Kevlar/Vectran**
("high tensile strength and low stretch") over pulleys; finger PIP and DIP tied by a coupling cable;
~12 N per finger; 374–400 g; 198 × 95 × 53.5 mm; **US$314 BOM** (servos ≈ $209); backdrivable;
MuJoCo model with 20 spatial tendons. **This is the strongest evidence that the HLS3606M works in a
printed tendon hand**, and it is the drop-in-class servo Tendra could adopt.

### 3.6 DexHand (The Robot Studio) [P]

16 slim micro servos in the forearm, braided PE line (**Sufix 832, 80 lb**), Feetech SCS2332/SCS15
on the wrist. Shows that the hobby community prefers **braided PE (UHMWPE)** over nylon mono.

### 3.7 InMoov hand [U]

One hobby servo per finger in the forearm, braided line in a pull-pull loop on a servo horn, printed
channels. Simple and robust for gestures, but friction in bare printed channels and line wear are
well-known problems; many builders re-tension often.

### 3.8 Brunel Hand / Open Bionics [U]

Prosthetics-derived; few actuators (linear actuators), tendons, **flexible printed (TPU) joints**
that act as extensors and absorb impacts. Lesson: flexure/elastic return can replace the extensor
strand, but you lose active extension force and precise position.

### 3.9 Yale OpenHand [U]

Underactuated grippers/hands (T42, Model O, Model Q) with one Dynamixel per finger pulling a
tendon, and **cast urethane flexure joints** (made by shape-deposition manufacturing). Very robust;
the fingers adapt to object shape. Lesson: **underactuation + compliance buys robustness** but costs
controllable DOF.

### 3.10 Research hands from 2025–2026 papers (titles confirmed, details to read)

- **MM-Hand** (arXiv 2604.17245) [P]: 21 DOF, remote **Feetech** motor hub. Plain PTFE tube had
  "relatively high friction" (extrusion quality); the **Bambu Lab PTFE feeder tube** and a **metal spring
  tube** were better. Software pretension.
- **Antagonistic Bowden hand** (arXiv 2512.24657) [P]: one Dynamixel per joint, **both strands on one
  shaft** (our scheme), PTFE 1/2 mm with spring jacket, octagon-keyed bobbins to set preload. Residual
  length mismatch of the two strands ≈ 0.03 mm when both use the same joint radius.
- **CRAFT** (arXiv 2603.12120) [C: title]: tendon hand with hybrid hard-soft compliance.
- **DexLink** (arXiv 2606.17418) [C: title]: 16-DOF linkage-driven, compact, affordable.
- **ARISTO Hand** (arXiv 2605.30508) [C: title]: sensing-driven distal hyperextension for fine manipulation.
- **Tendon force modelling for sim-to-real RL** (arXiv 2603.04351) [C: title]: models tendon forces so
  RL policies transfer on tendon hands. Directly relevant to `tendra.grasp_env`.
- **MuJoCable** (arXiv 2609.09612) [C: title]: reduced-order surface-routed cable transmission in MuJoCo.
  Relevant to our 42 spatial tendons.

---

## 4. Commercial hands in detail

Partly verified 2026-09-30 (sources at the end of this section). Allegro, Tesollo, Shadow DEX-EE, Linkerbot and Schunk remain [U].

| Hand | Key design choice | What it solves | What Tendra can take |
|---|---|---|---|
| **Shadow Dexterous Hand (E series)** | [C] **24 joints, 20 DC "Smart Motors" in the forearm base**. Each motor drives an **agonist–antagonist pair of Spectra (UHMWPE) tendons via one spool**: 40 tendons, exactly Tendra's scheme. [C] The DIP and PIP of the four fingers form **4 coupled pairs** (16 independent + 8 coupled). [C] A **force sensor on each tendon pair** (~30 mN resolution). [C] Hand + forearm **4.3 kg**. [P] Split spool for preload, a 90° turn over a metal bar at the spool exit. | Full human DOF with remote motors; tendon force control | Validates our scheme 1:1, including the tendon material (Spectra = UHMWPE braid). Copy the split-spool preload and the fixed bar at the spool exit. Note that even Shadow couples the DIP to the PIP. |
| **Shadow DEX-EE** | Built with Google DeepMind for long RL experiments; robustness over anthropomorphism [U] | Hands break under learning-scale use | Design for **thousands of hours**, not demos |
| **Sharpa Wave** | [C] **22 independently actuated DOF**, "motor direct-drive" actuation with a **tendon-driven transmission**, no coupled joints. [C] **>1,000 taxels per fingertip**, 0.005 N sensitivity, plus a **torque sensor per finger**. [C] **20 N** fingertip force, **1.3 kg**, 208 × 90 × 50 mm, 500 Hz control, 24 V, 180 W peak. [C] Durability: **2.5 M press cycles**, fingertip rubbing over 4,000 km. Used on Apptronik Apollo 2 and in NVIDIA designs. | Human-scale 22 DOF plus dense touch | Proof that a tendon hand with fully independent joints reaches 20 N; its durability numbers show what "product grade" means |
| **Wuji Hand** | [C] **20 DOF, motors inside the phalanges** (rotary actuators with reducers, 90° transmissions at the joints), **backdrivable direct drive, FOC at 1 kHz** on all 20 axes. [C] **15 N** fingertip force, 10 kg static grasp, **>300,000 cycles**, **580 g**, ~US$5,000. | Full DOF with no forearm | The opposite design choice to Tendra (motors in the fingers); needs custom micro-motors, out of hobby reach |
| **Inspire RH56DFX** | [C] **6 micro linear servo actuators, 12 joints** (linkage-coupled), built-in position and force sensing. [C] **10 N** per finger, 15 N thumb, 540 g, RS485/CAN, 24 V. The RH56DFTP variant adds tactile arrays. | Cheap, strong, compact; widely used on humanoids | Coupling cuts actuators: 6 motors give usable grasps |
| **PSYONIC Ability Hand** | [C] **6 DOF, 6 brushless motors**, **30 touch sensors**, **470–490 g**, closes in ~0.2 s. [C] Grip **66 N power, 9.3 N pinch, 14 N key**. Fingers dislocate and snap back on impact [U]. | Prosthetic-grade durability and speed | Pinch force (9.3 N) is a realistic near-term target for Tendra; pop-out fingers match ORCA |
| **Allegro V5** | Motors in the joints, 16 DOF, 4 fingers [U] | Simple, rigid, the old research standard | — |
| **Tesollo DG-5F** | 20 motors inside the hand [U] | Full DOF with no forearm | — |
| **Schunk SVH** | 9 motors for 20 joints via mechanical coupling [U] | Industrial robustness | Coupling again |

**Fingertip-force benchmarks** from this table: Inspire 10 N, PSYONIC 9.3 N (pinch), Wuji 15 N, Sharpa 20 N, 1X NEO 45 N. Tendra V1 is at ~3.4 N now, and ~13 N with the HLS3606M at a 1.5 knuckle ratio (see `README.md`).

Sources for this section: [Shadow Hand technical specification (Sept 2025)](https://shadowrobot.com/wp-content/uploads/2025/09/shadow_dexterous_hand_e_technical_specification.pdf), [Shadow Hand (Wikipedia)](https://en.wikipedia.org/wiki/Shadow_Hand), [Sharpa Wave product page](https://www.sharpa.com/pages/wave), [CNX Software on Sharpa Wave](https://www.cnx-software.com/2026/06/02/sharpa-wave-high-end-dexterous-robotic-hand-with-22-dof-high-sensitivity-dynamic-tactile-array/), [Wuji docs](https://docs.wuji.tech/docs/en/wuji-hand/latest/overview/), [Wuji article](https://www.revolutioninai.com/2026/07/wuji-hand-robot-direct-drive-humanoid.html), [Inspire RH56DFX](https://en.inspire-robots.com/product/rh56dfx/), [Knox Labs RH56DFX](https://www.knoxlabs.com/products/inspire-robots-rh56dfx-dexterous-hand), [PSYONIC Ability Hand](https://www.psyonic.io/ability-hand), [humanoid.guide Ability Hand](https://humanoid.guide/product/ability-hand/).

---

## 5. Actuator comparison: is SCS0009 enough?

| Servo | Stall torque (approx.) | Gears | Feedback | Price (approx.) | Used by |
|---|---|---|---|---|---|
| Feetech **SCS0009** | 0.226 N·m @ 6 V [C] | **Copper + steel (metal) gears in a plastic (PC) case**, 1:416 [C datasheet; the earlier "Plastic" was wrong] | Pos (pot), load, temp, voltage | ~$10 [U] | Amazing Hand, Tendra V1 |
| Feetech **STS3032** | 0.44 N·m @ 6 V (4.5 kg·cm) [C] | Metal, aluminium case, coreless [C] | 12-bit magnetic [C] | ~$15–25 [U] | Studied by Pollen ("servo horn is different") |
| Feetech **HLS3606M** | 0.59 N·m @ 6 V [C] | Copper + steel, 1:205, aluminium case, coreless [C] | 12-bit magnetic, **current + constant-current mode** [C] | ~$30 [C, Aero BOM] | **Aero Hand Open** (7×) |
| Dynamixel **XL330-M288** | 0.52 N·m @ 5 V [C] | Plastic [C] | Pos, current | $23.90 [C] | RUKA fingers [C], many low-cost hands |
| Dynamixel **XC330-T288** | 0.92 N·m @ 11.1 V [C] (the M288 5 V version: 0.93 N·m @ 5 V) | Metal [C] | Pos, current | $89.90 [C] | ORCA v1 [C]; LEAP uses XC330-M288 [C] |
| Feetech **STS3215** | ≈1.9–3 N·m (7.4/12 V) [U] | Metal | Pos, load, temp | ~$15 [U] | SO-100/SO-101 arms |

**Force budget for Tendra V1 (6 mm spool, 1:1):**
- SCS0009 stall: 0.23 N·m / 0.006 m ≈ **38 N** of tendon tension, or 0.23 N·m at the joint.
- At an index MCP with an 80 mm lever: ≈ 2.8 N at the fingertip at stall, ≈ 0.9 N continuous [P].
- Capstan losses in PTFE with ≤ 90° total wrap: × 0.85; with 180°: × 0.73 [P].
- **Pretension eats force twice**: it loads both strands all the time, adds friction, and keeps the servo
  loaded. SCS0009 backs off after about 2 s above 80 % load (project note), so a loop that is too tight
  can make the servo give up while simply holding a pose.
- For comparison, adult tip pinch (thumb to index tip) averages **~80 N in men and ~50–55 N in women**
  aged 20–39, key pinch ~115 N / ~80 N (Mathiowetz et al. 1985) [C]; useful kitchen tasks need
  several N per fingertip. For scale: RUKA measured 2.74 N pinch, Aero Hand Open ~12 N per finger [C].

**Conclusion:** SCS0009 can prove the kinematics, tendon routing, control stack and learning pipeline.
For real grasping, plan either (a) higher joint drum radius for MCP joints (e.g. 8 mm drum → 0.30 N·m
at the joint [P]), (b) two servos on the MCP flexion (Amazing Hand style), (c) a DIP–PIP coupling to
free servos, or (d) a drop-in stronger servo (HLS3606M, proven in Aero Hand Open, or STS3032; both
are 23 × 12 × 27.5 mm but have a different output spline, so spools change; or XL330/XC330 for the
MCPs). The SCS0009 has metal (copper + steel) gears [C], but they are tiny and 1:416: shock loads
(a finger hitting the table) go straight into them; a mechanical fuse (pop-out joint, slip clutch or
spring in series) protects them.

---

## 6. Lessons learned across hands

1. **Friction points must be smooth and hard** (ORCA: metal pins + PTFE only; MM-Hand: tube quality
   matters; InMoov: bare printed channels wear line). [C]/[P]
2. **Tendons creep and stretch.** Every tendon hand builds in re-tensioning: split spools (Shadow),
   ratchet spools (ORCA), keyed bobbins (Bowden hand), software pretension (MM-Hand). [P]
   Braided UHMWPE (Dyneema/Spectra, e.g. Sufix 832) stretches far less than nylon monofilament:
   at 1/3 of break load, PowerPro braid stretched 0.7–1 % vs 2–9 % for Berkley mono (FishTalk test) [C];
   creep and water uptake of nylon remain [U: general material knowledge].
3. **Overload must go somewhere safe.** ORCA's joints pop instead of breaking [C]; PSYONIC fingers
   dislocate and snap back [U]; Amazing Hand relies on software current limits [C].
4. **Calibration should be automatic.** ORCA auto-calibrates [C]; Tendra has servo position and load
   feedback, so it can find hard stops without extra sensors.
5. **Couple the DIP.** ORCA (DIP fixed/coupled [P]), Shadow (J1/J2 coupled [U]), Amazing Hand (linkage),
   RUKA (underactuated) all save motors there. Human DIP and PIP are also coupled in most tasks.
6. **Learned models beat hand-written tendon models** for control (RUKA [C]); and **sim-ready release**
   is now expected (Aero Hand Open [C], ORCA zero-shot sim-to-real [C]).
7. **Soft outer skin, rigid bones** (LEAP v2 Adv [C], Amazing Hand TPU shells [C]) improve grasp
   friction and survive bumps.
8. **Palm articulation** (LEAP v2 Adv [C], Shadow little-finger metacarpal [P]) helps human-like grasps.
9. **Durability is measured, not assumed:** ORCA publishes a 10 000-cycle test [C]. A hand that fails
   after a few hours is not useful for learning.
10. **Printed parts are imprecise**; Amazing Hand reports angle errors from part imperfection and
    plastic flex [C]. Per-joint calibration and feedback are essential.

---

## 7. What Tendra can learn / implement (prioritised)

**P1 — do before or while building V1 (cheap, high impact)**

1. **Servo protection in firmware.** Set SCS0009 torque limits per joint, watch the load and temperature
   registers, and back off before the servo's own 2 s / 80 % protection trips. Log load per joint during
   every run. (Amazing Hand warns the same.) Firmware already has `F` (feedback); add limits and a
   safe-release rule.
2. **Auto-calibration routine (ORCA-style).** With torque limited low, drive each joint slowly to its
   flex and extend hard stops, record servo positions, set the zero and the scale. This also measures
   **slack** (dead band between the two stops vs the expected range) and detects creep over time.
3. **Switch the line to braided UHMWPE** (e.g. 0.3–0.4 mm Dyneema/PE braid) and pre-stretch it under
   load before tying. Keep 0.4 mm nylon only for tests. Catalogue diameters (PowerPro and Sufix 832
   agree): 30 lb 0.28 mm, 40 lb 0.33 mm, 50 lb 0.36 mm, 80 lb 0.43–0.46 mm [C], all well inside the
   1.2 mm bores and 1 mm PTFE.
4. **Quick re-tension without disassembly.** A ratchet or screw-clamped split spool on each servo
   (ORCA/Shadow). Our current plan lists this as an option; make it a requirement.
5. **No line on PLA at any turn.** Use steel pins (e.g. 1–2 mm dowel/needle) at every deflection,
   and good-quality PTFE (Bambu-style feeder tube, per MM-Hand) for curved runs.
6. **Single-finger test rig first.** Build one index finger + 4 servos, measure fingertip force,
   friction loss (servo load vs tip force), and run a 10 000-cycle endurance test like ORCA.

**P2 — design changes to consider for V1 / V1.1**

7. **Couple DIP to PIP** mechanically (a small cross-tendon or linkage) on the four fingers. That frees
   4 servos, which can **double up on the MCP flexion** of index and middle (Amazing Hand trick), or be
   removed (fewer parts, less current, smaller forearm).
8. **Bigger drums where force matters** (MCP flex, thumb cmc_flex/mcp_flex): 8 mm drums give +33 %
   joint torque; range is not the limit.
9. **Mechanical fuse per finger:** a pop-out joint (ORCA) or a spring/elastic element in series with
   the tendon, so a crash doesn't strip the SCS0009's small gears.
10. **TPU skin/pads** on phalanges and palm (LEAP v2 Adv, Amazing Hand).
11. **Evaluate a stronger drop-in servo** for the few high-load joints: **HLS3606M** (used by Aero Hand
    Open; current control) or STS3032 (studied by Pollen), or Dynamixel XL330. HLS, STS and SCS each have
    their own memory table; Feetech advises against mixing series on one bus.

**P3 — software / learning**

12. **Learned actuator model (RUKA-style).** Record (servo positions, loads) vs. joint angles from the
    webcam/MediaPipe pipeline or a printed calibration jig; fit a small model per finger. Use it in
    `RealHand` and in the digital twin to correct for tendon stretch and friction.
13. **Tendon-aware sim.** Add tendon compliance and friction/pretension parameters to the MuJoCo model
    and randomise them in `GraspEnv` (see arXiv 2603.04351 and MuJoCable 2609.09612).
14. **Release V1 as "sim-ready"** (Aero Hand Open, ORCA): publish MJCF, calibration data and a known-good
    policy with the CAD, so others can reproduce results.
15. **Consider palm cupping** (LEAP v2 Adv) in a later version.

---

## 8. Open questions for the next research pass

- ORCA paper: exact popping-joint mechanism, auto-calibration algorithm, tactile sensor type, fingertip
  force, how the ratchet spool works. (Paper: arXiv 2504.04259; site orca.ethz.ch.)
- RUKA: exact actuators, tendon material, routing, cost, force; RUKA-v2 changes.
- Aero Hand Open: full specs.
- STS3032 datasheet: torque, gears, protocol compatibility with SCS0009 on one bus.
- SCS0009 gear-stripping reports in the Amazing Hand issue tracker.
- Measured fingertip force of LEAP v1, ORCA, RUKA, Amazing Hand for a fair comparison.

---

## Sources

Confirmed during this session:
- ORCA paper: https://arxiv.org/abs/2504.04259 · project: https://srl.ethz.ch/orcahand.html · https://orca.ethz.ch/
- "Getting the Ball Rolling" (ETH SRL, rolling-contact tendon hand): https://arxiv.org/pdf/2308.02453 · https://srl.ethz.ch/soft-robotic-news/2023/12/learning-a-dexterous-policy-for-a-biomimetic-tendon-driven-hand-with-rolling-contact-joints.html
- Amazing Hand repo: https://github.com/pollen-robotics/AmazingHand · blog: https://huggingface.co/blog/pollen-robotics/amazing-hand
- RUKA: https://arxiv.org/abs/2504.13165 · https://ruka-hand.github.io/
- RUKA-v2: https://arxiv.org/abs/2603.26660 · https://ruka-hand-v2.github.io/
- LEAP Hand v2 Adv: https://v2-adv.leaphand.com/ · https://github.com/leap-hand/LEAP_Hand_V2_Adv_API · IEEE: https://ieeexplore.ieee.org/iel8/11202977/11203009/11203038.pdf
- LEAP Hand v2: https://openreview.net/forum?id=eQomRzRZEP · https://embodied-ai.org/papers/2024/23_LEAP_HAND_V2_Low_cost_Anthr.pdf · https://roboticsconference.org/program/papers/132/
- Aero Hand Open: https://arxiv.org/pdf/2608.28578
- CRAFT: https://arxiv.org/pdf/2603.12120
- DexLink Hand: https://arxiv.org/pdf/2606.17418
- ARISTO Hand: https://arxiv.org/pdf/2605.30508
- Tendon force modelling for sim-to-real: https://arxiv.org/pdf/2603.04351
- MuJoCable: https://arxiv.org/pdf/2609.09612

From the earlier project note (`research/experiments/2026-09-28-full-hand/research.md`):
- Shadow Hand motor unit docs: https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/stable/user_guide/md_motor_unit.html
- Antagonistic Bowden hand: https://arxiv.org/html/2512.24657v1
- MM-Hand: https://arxiv.org/html/2604.17245
- DexHand: https://github.com/TheRobotStudio/V1.0-Dexhand
- MuJoCo Menagerie shadow_hand: https://github.com/google-deepmind/mujoco_menagerie/tree/main/shadow_hand

To verify (all [U] items): LEAP Hand v1 (leaphand.com), Yale OpenHand (eng.yale.edu/grablab/openhand),
InMoov (inmoov.fr), Open Bionics Brunel Hand, Shadow DEX-EE, Allegro, Inspire, PSYONIC, Tesollo, Wuji,
Sharpa, Schunk SVH product pages, Feetech STS3032 and Dynamixel XL330/XC330 datasheets.

Sources added in the 2026-09-30 fact-check:
- ORCA paper (full text): https://arxiv.org/html/2504.04259 ; servo models in code: https://github.com/orcahand/orca_core (`orca_core/constants.py`, `hardware/feetech_client.py`, `hardware/dynamixel_client.py`) ; ORCA v1 listing: https://humanoid.guide/product/orca-v1/
- RUKA full text: https://arxiv.org/html/2504.13165 ; RUKA-v2 abstract: https://arxiv.org/abs/2603.26660
- Aero Hand Open: https://arxiv.org/html/2608.28578 ; https://github.com/TetherIA/aero-hand-open ; https://shop.tetheria.ai/products/aero-hand-open
- LEAP v2 Adv: https://v2-adv.leaphand.com/ ; https://github.com/leap-hand/LEAP_Hand_V2_Adv_API
- Amazing Hand README (8 × SCS0009, no cables, 400 g, < €200, STS3032 to-do "servo horn is different"): https://github.com/pollen-robotics/AmazingHand
- Feetech SCS0009 datasheet: https://files.seeedstudio.com/products/Feetech/SCS0009-Specifications.pdf ; HLS3606M datasheet: https://www.feetechrc.com/Data/feetechrc/upload/file/20240807/6385862508379674057566108.pdf
- Line diameters: https://www.rapala.com/us_en/832-advanced-superline , https://www.tacklewarehouse.com/Power_Pro_Spectra_Braided_Line_Moss_Green/descpage-PPSL.html ; stretch test: https://www.fishtalkmag.com/blog/fishing-line-stretch-test-stretching-truth
- Mathiowetz et al. 1985 (tables): https://klyonsot2013.wordpress.com/wp-content/uploads/2013/11/grip-pinch-strength-norms.pdf

Fact-checked 2026-09-30 (partial): ORCA (servos, tendon, joints, DIP coupling, calibration, tensioning, cycles), RUKA / RUKA-v2, Aero Hand Open, LEAP v2 Adv, Amazing Hand, servo table (SCS0009 gear material corrected to metal), line diameters, human pinch norms. Still [U]: commercial hands (Shadow, Allegro, Inspire, PSYONIC, Tesollo, Wuji, Sharpa, Schunk), InMoov, Brunel, Yale OpenHand, Faive.
