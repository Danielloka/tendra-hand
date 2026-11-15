# 15 · Sensing plan: joint angles, touch, tendon force, wiring, electronics

Agent 15 (sensing architect), 2026-11-15. Study only: everything is in this folder.

| File | What |
|---|---|
| `cad/15_magnetics.py` → `out/15/magnetics.{json,txt}` | Field from each joint magnet at its chip; MR84 steel bearing effects; crosstalk between all 25 magnets of the hand. numpy, ~9 min |
| `cad/15_budget.py` → `out/15/budget.{json,txt}` | Sensor list and address check, I²C and link timing, wire twist fatigue, power, BOM. Plain Python |
| `cad/15_sensor_parts.py` → `out/15/*.step/.stl`, `sensor_spec.json`, `render_*.png` | CadQuery parts (17 solids, all valid) and the machine-readable spec for the finger builder |

Run: `uv run --no-project --with cadquery --with numpy --with trimesh --with matplotlib python cad/15_sensor_parts.py` (~1 min).
Inputs: reports 01, 05, 07, 09, 10, 13 (`out/13/wrist_load.txt`), **14 (sensor parts, verified values)**,
`research/references/hands/sensing.md`, `cad/07_finger.py` (frames and pockets). Tags: **[14]** = verified in report 14,
**[M]** = from memory (verify before buying), **[C]** = my calculation.

Terms:
- **Hall chip**: a chip that measures the magnetic field. With a small magnet that turns with the joint, it reads the joint angle.
- **Diametric magnet**: a disc magnetised across its diameter, so its field turns when it turns.
- **Die gap**: the distance from the magnet face to the sensing element inside the chip.
- **Bus**: shared wires that several chips talk on. **I²C** uses 4 wires (power, ground, data, clock); each chip has an address.
- **Hub**: a small microcontroller in the palm that reads all the sensors.
- **Daisy chain**: the 4 bus wires run from board to board, like an LED strip, with no splices.

---

## 1. Plan summary

**One rule for every joint (layout "R3 slim", agreed with the finger builder on its R2 geometry):**
- The bearing on the sensor side (+x) becomes an **open full-ceramic ZrO2 MR84 (4 × 8 × 2)**. It is non-magnetic and 1 mm
  thinner than the MR84ZZ.
- A diametric magnet sits flush with that bearing's outer face, in the bore of the tube end: Ø2 × 1.5 in the MCP inner
  tube, Ø3 × 1 in the PIP and DIP pins.
- A TMAG5273 faces it across a 0.3 mm air gap, on a Ø6.4 × 0.4 board, all inside the existing Ø6.75 counterbore and
  closed by a printed plug cap.
- Die gap 1.2 mm → **56–66 mT**, inside the ±80 mT range [C].
- **The MCP adds 0 mm of width** (the stack ends at x 8.75, the lug face is at 9.0). The PIP adds +0.55 and the DIP +0.85,
  both where the fingers taper and there is ≥ 3 mm between them.
- The sensor wires cross every joint *inside* the hollow pin, on the axis, so they only twist and never bend.
- **No hardened steel within 3 mm of a magnet.** A steel race next to the magnet can cause 1.3–2.6° of hysteresis
  [C, bound], the one item that could break the 1° target. Hence ZrO2 (or a bronze/iglidur bush) on the sensor side.

**Each finger is one I²C bus with 7 chips.** All five fingers are read in parallel by an RP2040 hub in the palm. Only
**6 thin wires cross the wrist**, through two hollow wrist pins: 5 V, GND, UART TX/RX to the ESP32-S3, and USB D+/D− for
reflashing the hub. Angles stream at **500 Hz**, tips and pads at **200 Hz**.

**Tendon force needs no new hardware.** Spool angle minus joint angle, times the loop stiffness, gives the joint torque.
It resolves ~0.5 N of strand force (SCS0009) or ~0.16 N (HLS3606M) [C]. The servo's load or current reading backs it up.

**Cost:** about **€220** for the whole hand's sensing (40 chips, hub, 12 non-magnetic bearings, wires, plugs, boards), or **~€27 per finger** [C, M prices].

### All sensors

| # | Location | Sensor | Body (part) | Magnet / transducer | Bus / addr | Rate | Cost each |
|---|---|---|---|---|---|---|---|
| 1–4 | index, middle, ring, little **mcp_abd** | TMAG5273**A1** angle | palm knuckle module, dorsal (07: base) | Ø3 × 1 diametric, glued on the end of a brass/316 dorsal abd stub (yoke) | finger bus, 0x35 | 500 Hz | €2.5 |
| 5–8 | 4 × **mcp_flex** | TMAG5273**B1** | proximal, +x MCP lug counterbore (behind an open ZrO2 MR84) | Ø2 × 1.5 diametric in the +x end of the inner axle tube (yoke) | 0x22 | 500 Hz | €2.5 |
| 9–12 | 4 × **pip** | TMAG5273**C1** | proximal, +x PIP lug counterbore (behind an open ZrO2 MR84) | Ø3 × 1 in the +x PIP pin end (middle) | 0x78 | 500 Hz | €2.5 |
| 13–16 | 4 × **dip** (coupled) | TMAG5273**D1** | distal, +x DIP cheek counterbore | Ø3 × 1 in the +x DIP pin end (middle; pin shortened to x 6.0) | 0x44 | 500 Hz | €2.5 |
| 17–20 | thumb **cmc_rot, cmc_flex, mcp_flex, ip** | TMAG5273 A1, B1, C1, D1 | parent side of each axis | same module | thumb bus | 500 Hz | €2.5 |
| 21–25 | 5 **fingertips** (3-axis force) | MLX90393-014 (QFN 3 × 3) | distal tip cavity (05/07), new `tip_carrier` | Ø3 × **1.0** N42 axial magnet in the snap-in skin | each finger bus, 0x18 | 200 Hz | €7 |
| 26–33 | 4 × proximal + 4 × middle **phalanx pads** | BMP581 barometer in silicone | 8 × 10 × 4 pocket in the palm face | the phalanx silicone pad (05 #14) | 0x46 / 0x47 | 200 Hz | €3.5 |
| 34–35 | thumb proximal + metacarpal pads | BMP581 | thumb | same | thumb bus | 200 Hz | €3.5 |
| 36–39 | **palm zones**: thenar, hypothenar, centre, distal palm | BMP581 under the 8–10 mm palm pad (05 #15) | palm | silicone pad | palm_a, palm_b | 200 Hz | €3.5 |
| 40 | palm **range** (optional) | VL53L1X | palm heel (05 #26) | — | palm_a, 0x29 | 50 Hz | €4 |
| — | **tendon force**, 16 joints | none: spool-vs-joint stretch + servo load/current | — | — | servo bus | 50–100 Hz | €0 |

40 chips on 7 I²C buses with no address clash (checked in `15_budget.py`).

---

## 2. Joint angles (deep dive)

### 2.1 Sensor and magnet choice

| Choice | Value | Why |
|---|---|---|
| Chip | **TMAG5273 x1 (±40/±80 mT), SOT-23-6, 2.9 × 2.8 incl. leads** [14] | Smallest part with a built-in angle engine (±0.5° in the X-Y plane) [14]. Four factory addresses, so one variant per joint on each bus with no boot-time reprogramming. 2.3 mA. **3.3 V only (abs. max 4 V)** [14]. |
| Magnet | **MCP Ø2 × 1.5 N45 diametric** (fits 12's R2 inner tube, Ø2 bore); **PIP/DIP Ø3 × 1** (Ø3 pin bore); abd Ø3 × 1 | At a 1.2 mm die gap: Ø2 × 1.5 **55.9 mT**, Ø3 × 1 **66.5 mT** [C]. Ø3 × 1.5 would give 82 mT (over range) and Ø3 × 2 93 mT. 07's drawn MCP (Ø3 × 2, chip 0.4 mm from the axle end) would see **129 mT: saturated**. |
| Board and orientation | **Ø6.4 × 0.4 round FR4 behind the chip; the chip faces the magnet** across a 0.3 mm air gap | Die gap = 0.3 + ~0.9 (die under the package top [M]) = 1.2 mm. The board and chip fit in the Ø6.75 counterbore behind each bearing, so no new pocket. 4 pads 1.0 × 0.9 on the outer face, each taking 2 wires (bus in and bus out). One 100 nF 0402 cap. |
| Sensor-side bearing | **open full-ceramic ZrO2 MR84, 4 × 8 × 2** [M: €4–6, load rating below steel, verify] | Non-magnetic and 1 mm thinner than the MR84ZZ, which is what makes the MCP zero-width. Fallback: a 4 × 8 × 2 flanged iglidur/bronze bush (~2 % friction). |
| Cap | printed PETG plug Ø6.70 × 2.3, pressed into the counterbore | Holds the board, stops the bearing's outer race, 0.2 mm roof, 0.6 × 1.2 wire slot toward +y (dorsal). Files: `joint_cap_{mcp_flex,pip,dip,abd}.step`. |

**Stack along the axis** (finger frame, MCP): bearing x 4.4–6.4 → magnet flush at 4.9–6.4 → 0.3 air gap (magnet and chip
turn relative to each other) → chip 6.7–8.15 → board 8.15–8.55 → roof to **8.75**.

| Joint | Bearing (+x) | Magnet (x) | Chip (x) | Outer face (x) | 12's lug face | Width added |
|---|---|---|---|---|---|---|
| mcp_flex | ZrO2 open MR84, 4.4–6.4 | 4.9–6.4, Ø2 × 1.5 (inner tube cut to end at 6.4) | 6.7–8.15 | **8.75** | 9.0 | **0** |
| pip | ZrO2 open MR84, 4.25–6.25 (−x keeps the Ø5 brass bush for the bar pin web) | 5.25–6.25, Ø3 × 1 | 6.55–8.0 | **8.6** | 8.05 | +0.55 |
| dip | brass bush 4.5–6.0 | 5.0–6.0, Ø3 × 1 (pin shortened from 7.5 to 6.0) | 6.3–7.75 | **8.35** | 7.5 | +0.85 |
| abd | (MR63 → ZrO2 or brass bush) | y 16.7–17.7 on the stub end | chip-out on the board, y 18.2 | y 20.35 | 17.8 | dorsal, palm side (free) |

**Width.**
- 12's pitch analysis needs 18.6–19.6 mm without bosses. The MCP stack stays inside the lug, so nothing is added where the
  pitch is tight.
- At the PIP and DIP the fingers are 16.1 and 15 mm wide on a ≥ 18.5 mm pitch, so the +0.55 / +0.85 leave ≥ 2.4 mm.
- No sensor sits on the −x side of any finger. This holds for **all four fingers**. On the index, +x faces the thumb.

### 2.2 What disturbs the angle, with numbers (`out/15/magnetics.txt`)

**Steel bearing races (review #10 item 7).** A coupled soft-steel dipole model of an MR84ZZ (races, shields, 7 balls,
χ_eff 2.9) around the magnet gives:

| Case | Own field at the die | With MR84 | Shunted | Angle ripple from race asymmetry | Cage/ball part (not calibratable) | Hysteresis bound |
|---|---|---|---|---|---|---|
| A: 07 as drawn (Ø3 × 2 inside the race, 0.85 die gap) | 129 mT (**saturates**) | 65 mT | 50 % | 0.53° | < 0.01° | **2.5°** |
| B: magnet face 1.0 proud of a steel race, 1.5 gap | 61.8 mT | 41.6 mT | 33 % | 0.14° | < 0.01° | **2.9°** |
| C: magnet face 2.0 proud (12's R2 carrier) | 61.8 mT | 54.7 mT | 12 % | 0.01° | < 0.01° | 1.5° |
| D: **R3 slim**, no steel (open ZrO2 / brass bush), die gap 1.2 | 56–66 mT | — | 0 | 0 | 0 | **0** |

Case F (B with the chip 0.15 mm off the axis) adds 0.12° without steel and 0.29° with it; calibration removes it. All values: `out/15/magnetics.json`.

How to read it:
- The **balls do nothing** measurable (< 0.01°).
- **Race shunting** only scales the field; it does not bend the angle, because the race is round and centred on the magnet.
- The real risk is **hysteresis**. Hardened 52100 bearing steel has a coercivity of ~2–4 kA/m [M]. The magnet drives the
  outer race at only 13–34 kA/m, so the race's own magnetisation lags the turning magnet by ~5–9°. Weighted by the race's
  10–50 % share of the field at the chip, that is a **1.3–2.6° bound**.
- Hysteresis depends on direction, so a one-way calibration cannot remove it. A two-way lookup removes most of it, but the
  bound is too close to the 1° target to bet on.

**Decision: no hardened steel within 3 mm of a joint magnet.**

| Joint | Change on the sensor (+x) side | Cost |
|---|---|---|
| PIP | **Open ZrO2 MR84 on +x.** The −x side keeps its Ø5 brass bush. Swapping sides is ruled out: the finger builder showed that an Ø8 seat on −x pushes bar pin A from r 5.1 to ≥ 6.6, and the DIP linkage error then grows from 0.43° to ~3.5°. | ~€5 per joint |
| DIP | Already brass bushes. | — |
| MCP | **Open full-ceramic ZrO2 MR84** on +x [M: ~€4–6; load rating lower than steel, verify]. | Fallback: an iglidur/bronze 4 × 8 × 2 flanged bush, ~2 % friction loss (μ 0.15 at r 2 against a 400 N·mm joint torque). |
| abd | ZrO2 MR63 or a brass bush on the dorsal side. Abd loads are small. | — |
| Others | The dorsal abd stub and the DIP bar pins A/B become **brass or 316** (07 had hardened ISO 8734). Pin A sits 5.1 mm from the PIP axis and stays still relative to the PIP chip, so a steel pin there distorts the reading in an angle-dependent way. 12's R2 MR63 idler (steel) sits 4–5 mm inboard of the MCP magnet and spins with the strands: estimated ≤ 0.2° (a ~2 % field share × ~9° lag). Bench it. | — |

12's R2 alternative (steel MR84ZZ kept, magnet on a printed carrier 2 mm beyond the race) is case C: 12 % shunt and a
1.5° hysteresis bound, and it costs +4 mm of width. R3 slim removes both.
The bench A/B on 08's R9 rig decides: steel MR84ZZ + 2 mm carrier vs open ZrO2 + flush magnet, sweeping both ways. Pass < 1°.

**Crosstalk from the other 24 magnets.** Dipole far field at q = 0:

- **Worst: the ring PIP chip, 0.47 mT from the middle PIP magnet: a 0.64° bound** against a race-shunted 41.6 mT.
  With R3 slim (no steel, 56–66 mT) that is **≤ 0.48° (MCP) and ≤ 0.40° (PIP)**. Abd chips: ≤ 0.45°.
- Tips see ≤ 0.54 mT from joint magnets, which is a false force of **≤ 0.024 N**.
- In a pinch, the thumb tip magnet 10 mm from the index tip magnet adds **0.83 mT ≈ 0.04 N** of false force (0.06 N at
  8 mm). The hub subtracts it with a dipole model from forward kinematics (both tip positions are known).

Most of the crosstalk is static and goes into calibration. The part that changes with a *neighbour's* pose is at most
**±0.3°** (neighbouring fingers move together in most grasps) [C].
Report 14's warning ("3 mT at 10 mm") applies only below 12 mm, and no two joint magnets are that close: same-finger
joints are ≥ 24 mm apart, neighbours 19.5 mm, and the thumb CMC pair (cmc_rot vs cmc_flex) is the closest at
~12 mm (0.32°).

### 2.3 Accuracy budget, per joint after calibration [C]

| Term | Before calibration | After |
|---|---|---|
| Chip angle error (X-Y plane) [14] | ±0.5° | ±0.1° (lookup) |
| Chip 0.15 mm off the axis (placement) | 0.12–0.3° | ~0 (lookup) |
| Race (with a non-magnetic bearing) | 0 | 0 |
| Crosstalk from moving neighbours | ±0.3° | ±0.3° (±0.1° with model subtraction) |
| Noise (32× average) [14] | ~0.05° | 0.05° |
| **Total** | **~1°** | **≤ 0.5°**, meets ≤ 1° |

**Calibration (per joint, ~2 min per finger, automatic):**
1. Tendons at normal pretension, hand in free air.
2. The firmware drives each joint slowly (5°/s) from stop to stop, both ways, while the hub logs the chip angle and the servo
   position.
3. The joint's two hard stops (CAD: −5° and 95°) anchor the scale. The servo path, corrected by the shared coupling table
   (review #15), gives the shape in between.
4. Fit a 4-harmonic Fourier correction for each direction and store 2 × 9 coefficients per joint in the hub's flash.
5. Check the first finger once against 08's protractor jig (R9). Pass: < 1° everywhere.

### 2.4 Does the coupled DIP need a sensor? **Yes (€2.5 per finger).**

- **The bar has play.** Two Ø2 pins with 0.05 mm clearance each, on 3.5–4.5 mm levers, give **±1.4° of DIP play** [C].
  The tip force turns that into a real pose error.
- **It catches faults.** A broken bar, a lost pin or worn holes show up at once as DIP ≠ f(PIP). That is a free health check.
- **It gives the fingertip pose,** needed to turn the tip's 3-axis force into the hand frame (the policy's input).
- **The wiring is already there:** the bus passes the DIP pin anyway. The cost is one chip, one magnet, a cap and a
  0.85 mm boss.

---

## 3. Touch and pressure

| Location | Technology | Why | Range | Resolution | Size | Cost |
|---|---|---|---|---|---|---|
| **Fingertips** (5) | **MLX90393** under 05's snap-in skin, magnet changed to **Ø3 × 1.0 N42 axial** | 3-axis (normal + shear, so slip), same as ReSkin/AnySkin, skin swaps in seconds. The 05 Ø3 × 1.5 N45 gives ~20 mT at rest and passes the chip's **50 mT saturation** [14] at ~1.5 mm compression; Ø3 × 1 starts at ~14 mT and saturates at ~2 mm (~12–15 N) [C, verify on the bench]. | 0.01–12 N normal, ±5 N shear | ~5 mN (noise ~1 µT; 17 mT/mm) [C] | chip 3 × 3 × 1 on a 9.2 × 9.0 board (07 cavity) | €7 |
| **Phalanx pads** (proximal + middle, ×4 fingers; thumb proximal + metacarpal) | **BMP581 barometer** potted under the phalanx silicone pad | Only normal force, but a **2.0 × 2.0 × 0.8** package [14] fits a 4 mm deep pocket. <0.1 Pa noise, up to 480 Hz, so it also feels slip *vibration*. Exactly 2 addresses (0x46/0x47), which matches 2 pads per finger bus. | ~0.02–5 N per pad before the 125 kPa limit [M, TakkTile-class] | ~mN | pocket 8 × 10 × 4 | €3.5 |
| **Palm** (4 zones) | BMP581 in a cup under the 8–10 mm Ecoflex palm pad (05) | Same part and calibration as the pads. The thick pad spreads the load, so the range rises. Fallback: FSR 402 on the RP2040's ADC pins (simpler, drifts ±10–25 % [14]). | ~0.1–20 N per zone [M] | ~10 mN | cup 8 × 10 × 4 (`pad_cup`) | €3.5 |
| **Palm range** (optional) | VL53L1X | Distance to the object before contact (05 #26) | 4–4000 mm | mm | 4.9 × 2.5 × 1.6 | €4 |

Not chosen:
- **Camera tips** (DIGIT-style) are too big for the 14 mm tip (sensing.md).
- **FSRs on the fingers** drift and see no shear.
- **Capacitive** drifts with humidity. Later, it could be a pre-touch layer on the tip board (05 #25).

---

## 4. Tendon force: decided, no strain elements in the hand

| Option | Resolution | Cost | Verdict |
|---|---|---|---|
| **Series-elastic estimate**: strand stretch = r_spool·θ_servo − R_drum·q_joint − couplings (shared table); force = k_loop × stretch | SCS0009: 0.293°/tick × r 5 = 0.026 mm → **0.54 N of strand** at k_loop 21 N/mm (09) → **~0.08 N at the tip**. HLS3606M (0.088°): **0.16 N**. | €0 (needs only the joint sensors) | **Use.** Re-zero the stretch whenever the joint moves free (creep and pretension drift: review #9). |
| Servo load register | SCS: PWM duty in 0.1 % steps, *not* current [14]; HLS: **current at 6.5 mA/LSB** + a current mode [14] | €0 | **Use as a second channel.** With HLS: true torque control. |
| Strain gauge / load cell per strand | mN | 32 channels, too big | **No.** One load cell + HX711 on the **bench** to measure k_loop per joint (08). |
| Photo-reflector flexure at the forearm spool (arXiv 2507.00464, 13 × 7 × 6.5 mm) [14] | 10 mN | ~€3 | **Later**, on 2–4 strands (index flex, thumb) to teach the model. |

---

## 5. Wiring

### 5.1 One finger (the path is waypoint by waypoint in `sensor_spec.json` → `wiring.waypoints_finger`)

```
 tip board (MLX90393) ──static── dip board (+x cheek) ──static, dorsal groove── distal −x wire layer [glue A]
   ══ DIP pin bore: TWIST 76° over 9.0 mm ══ radial Ø1.2 (+y, x 3.2) [glue B, middle]
   ── middle channel ── radial Ø1.2 (+y, x 3.6) [glue C]
   ══ PIP pin bore: TWIST 100° over 9.2 mm ══ −x end [glue D, proximal]
   ── dorsal groove across ── pip board (+x) ── +x face groove ── mcp board (+x) ── knuckle-hood groove to −x [glue E]
   ══ MCP inner-tube bore (Ø2): TWIST 100° over 10.4 mm ══ radial Ø1.2 (+y, x +3.0) [glue F, yoke]
   ── yoke dorsal channel ── exit 4 mm off the abd axis ~ free loop (abd ±15°, path change 1.0 mm, R ≥ 8) ~
   knuckle board (palm): abd chip + JST SH 4-pin socket ── palm harness ── hub
```

- **Wire:** 4 × **38 AWG PTFE stranded (7 × 0.04, OD ~0.30)**. Red 3V3, black GND, yellow SDA, blue SCL.
  Run them **parallel, never pre-twisted**: a pre-twisted bundle strains to first order when it twists, a parallel one only
  to second order.
- **Glue:** a drop of flexible silicone adhesive (Sil-Poxy) at each glue point, not CA, so there is no hard edge.
- **Axle slot (review #6):** 07's slot cut the axle at its most-stressed fibre (−z). Use a **Ø1.2 hole along +y** at the
  neutral axis for the −z strand load. The section modulus drops by < 1 %, and the peak stress at the hole edge is
  ≤ 0.9 × the outer-fibre stress (K_t 3 × 0.3) [C]. Same rule for the PIP and DIP radial holes.
- **Where the MCP hole goes:** 12's R2 already uses +y holes, but its MCP hole is at x −3.0. That leaves only 4.4 mm of
  free twist length to the −x end: 0.35 % strain with 38 AWG, over the limit. **Move it to x +3.0** (through the inner tube,
  the +x sleeve and the cheek) for 10.4 mm (0.06 %). The PIP and DIP slots at x +3.2 are fine.

**Fatigue** (`budget.txt`). Conductor strain in a parallel bundle twisted by θ over a free length L is
ε = √(1 + (rθ/L)²) − 1:

| Crossing | Range | Free length | 38 AWG PTFE | 36 AWG silicone (05/07) |
|---|---|---|---|---|
| DIP pin | 76° | 9.0 | 0.05 % | 0.14 % |
| PIP pin | 100° | 9.2 | **0.08 %** | 0.23 % ✗ |
| MCP axle | 100° | 10.4 | 0.06 % | 0.18 % |
| abd (free loop) | 30° | — | bend R ≥ 8: 0.25 % static, ±0.02 % cycling | — |
| thumb CMC (Ø5 hollow pin) | 141° | 10 | 0.14 % | 0.38 % ✗ |
| wrist flex (6 wires) | 100° | 9 | 0.17 % | 0.47 % ✗ |
| wrist dev | 50° | 9 | 0.04 % | 0.12 % |

Limit: **0.20 % for 10⁶ cycles** (fine-stranded annealed copper; conservative [M]). For comparison, a bend at R 3 strains a
0.05 strand by 0.83 % and a solid 36 AWG wire by 2.1 %; the second is the ORCA failure mode (dead after 4 500–7 000 cycles).
**36 AWG silicone fails at the PIP, thumb and wrist: use 38 AWG.** Bench test W1 (§9) checks this.

**Count per finger:** 4 conductors carry 7 chips. Thumb: 4. Palm: 2 × 4. **Wrist: 6.**

### 5.2 Connectors (swap a finger in about a minute)

- **Finger:** a JST SH 1.0 mm 4-pin socket on the **knuckle board** (palm side, dorsal). The finger keeps a 25 mm pigtail
  with the plug.
- **Why the abd crossing is a free loop, not a hollow stub:** a plug cannot pass a Ø2 bore, and abd is only ±15°. A loop
  4 mm off the axis changes length by just 1.0 mm.
- **To swap:** remove the dorsal palm cover (2 × M2), unplug, pull the two abd pins, lift the finger out.
- **Thumb:** the same, at the CMC mount.
- **Hub:** 7 × JST SH 4-pin (5 fingers, 2 palm buses) + 1 × JST SH 6-pin (wrist).
- **No connector inside a finger:** each one is a failure point and costs 3–4 mm.

### 5.3 Crossing the wrist (the eye plate fills the centre)

| Option | Strain / risk | Verdict |
|---|---|---|
| Through the free eye-plate centre cell (0,0) | The cable must kink up to √(50² + 30²) = **58°** at the 1.2 mm plate. Any slack bows it into the strands. | **No** |
| **Hollow wrist pins:** palm → flex-pin bore (twist) → 2 × 2 groove round the ring (static, `ring_cable_clip`) → dev-pin bore (twist) → forearm | 0.17 % / 0.04 % with 38 AWG. A Ø2.4 bore in 13's Ø5 pin costs 5.6 % of Z (139 → 147 MPa SCS, 315 → 334 MPa HLS) [C]. Sealed, no snag. | **Chosen, confirmed by 13 (W13)**: +x flex pin x 11–21 (cable in and out axially, 10 mm twist), 1.6 slot + 2 × 2 ring groove, +y dev pin (Ø1.6 radial in at y 19.6, 9.4 mm twist, out axially at y 29). Pins are E355+C tube 5 × 1.25 or drilled 17-4PH, not drilled hardened dowels. Z −5.3 %: SCS 145 MPa, HLS 326 MPa |
| Dorsal service loop at x = 0 (on the dev axis), 70 mm free, R ≥ 15 | 0.17 % at the strand. Takes ~15 mm of flex path change. Can snag; needs a spiral wrap. | **Fallback**, and fine for the first build (`dorsal_loop_clip`) |
| Slip ring | contact wear, noise | No |

**Noise.**
- **I²C never leaves the palm,** so it never runs beside the 15 A servo wiring.
- The link is a 2 Mbaud UART with a CRC on every frame.
- The 6-core runs up the dorsal forearm wall, ≥ 10 mm from the servo power branches, and crosses them only at 90°.
- One ground point: the hub's GND joins the ESP32's GND only through the cable.
- If the CRC error rate is > 10⁻⁴, add two RS-485 transceivers (€3) and use the D+/D− pair as the second differential pair.

---

## 6. Electronics architecture

| | A. **Palm RP2040 hub, 7 parallel I²C buses, UART to the ESP32-S3** | B. ESP32-S3 reads all chips through the wrist (TCA9548A mux) | C. One small MCU per finger, shared RS-485 |
|---|---|---|---|
| Wires across the wrist | **6** (4 without USB) | 4 + I²C at 30–50 cm | 4 |
| Bus load | worst finger bus **41 %** at 400 kHz [C] | everything on one master: **239 %** [C]: **does not fit** | ~20 % per finger |
| Rates | angles 500 Hz, tips/pads 200 Hz | ~100 Hz at best | 1 kHz |
| Noise | I²C stays local | I²C across the wrist beside the servo wires | best |
| Firmware | one hub image (Arduino-Pico, PIO I²C) | none | 5 images |
| Cost | €4 board + €6 carrier | €3 mux | 5 × €3 + flex |
| Verdict | **Chosen** | No | Later, with a custom flex PCB |

**Hub:**
- RP2040-Zero (23.5 × 18 mm [14]) on a 32 × 26 mm carrier PCB with a 300 mA 3.3 V LDO and 8 JST SH sockets, in
  `hub_tray`. The tray sits in the dorsal palm shell (hand frame x ±18, y 8.5–13.5, z 28–58) above the strand fan, under the
  removable dorsal cover.
- Buses: 2 hardware I²C + 5 PIO I²C (the RP2040 has 8 PIO state machines).
- Every bus runs at **400 kHz** (the MLX90393's limit [14]). The palm buses could run at 1 MHz.

**Addresses per finger bus:** TMAG5273 A1 abd 0x35, B1 mcp 0x22, C1 pip 0x78, D1 dip 0x44; MLX90393-014 tip 0x18;
BMP581 proximal pad 0x46, middle pad 0x47. The thumb uses the same set. Palm: 2 buses × 2 BMP581 + the VL53L1X (0x29).

**Link frame (hub → ESP32-S3), 110 B at 500 Hz:** sync 2, time 4, 20 angles × int16 (1/16°), 5 tips × 3 × int16,
14 pads × int16, range 2, status 2, CRC16 2.
- 55 kB/s, which is **28 % of a 2 Mbaud UART**. Latency ≤ 2.6 ms.
- ESP32-S3 side: **UART2 on GPIO 15 (RX) / 16 (TX)** (suggested in sensing.md; not used by `config_v1.h`, which has the
  servo bus on 17/18 and USB on 19/20). Check before wiring.

**Power:** 40 chips ≈ 87 mA + RP2040 25 mA = **112 mA at 3.3 V (0.56 W from 5 V)**. The 5 V comes from the logic supply,
never from the servo rail (the TMAG5273's absolute max is 4 V [14]). Drop over the 38 AWG link: ~0.25 V, which the LDO
absorbs. Use two conductors for GND if the cable allows.

**Firmware protocol** (ESP32 0.4.0 → **0.5.0**, kept short):
- `I` line: add `sens=20a5t14p`.
- `A`: one line `A t q1..q20` in rad. Servo-joint order (IDs 1–16), then the DIPs index, middle, ring, little.
- `T`: one line `T t fx fy fz ×5 (N) p1..p14 (N) d_mm`.
- `G <hz>`: stream A and T lines at up to 200 Hz on USB (0 = off).
- The ESP32 uses the 500 Hz data itself (joint loops, slip reflex).
- **Safety:** if hub frames stop for > 50 ms, the firmware marks the data stale, keeps the servos at their last goal, and
  never moves anything.
- PC side: `RealHand` parses `A`/`T`. `tendra.joints.V1` gets the sensor list from `sensor_spec.json`.

**A custom flex PCB would improve:**
- one flex per finger with all 4 chip sites and the tip, folded around the joints, with clock-spring twist sections;
- no hand-soldered 38 AWG wire, ~2 mm less width per joint (no cap);
- a 5th chip site per tip (contact location).

Do it after the first hand works (~€40 + assembly at JLC [M]).

---

## 7. CAD built (`out/15/`, all solids valid; renders `render_*.png` looked at)

| Part | Body / frame | Size | Notes |
|---|---|---|---|
| `joint_cap_mcp_flex`, `_pip`, `_dip`, `joint_cap_abd` | proximal / proximal / distal / palm knuckle, finger frame | Ø6.70 skirt + Ø8.4 × 0.3–1.35 flange | Holds the Ø6.4 board; chip pocket 3.3 × 3.2; 1.2 × 1.0 wire slot to +y |
| `joint_module_<j>`, `abd_module` | assemblies (cap + board + chip + magnet + keeper) | — | References for the finger builder |
| `tip_carrier` | distal, finger frame | 9.5 × 9.3 × 1.2 tray + 2.2 × 2.6 tongue | 0.4 board recess, 1.4 × 0.6 wire channel, the tongue clamps the 4 wires in 07's Ø2.6 bore |
| `pad_cup` (+ `pad_assembly_ref`) | phalanx/palm pockets | 8 × 4 × 10 | BMP581 board 5 × 6 × 0.6 on the floor, silicone fills it |
| `hub_tray` (+ `hub_assembly_ref`) | palm, hand frame | 36 × 5 × 30 | 4 × M2 bosses, 24 × 20 window, 5 finger-plug notches (distal wall), 3 on the proximal wall |
| `wrist_pin_hollow` | wrist cardan | Ø5 × 16, bore 2.4 | Ø1.6 radial exits 3 mm from each end, on opposite sides |
| `ring_cable_clip` | wrist ring | R 11–14 quarter, 3 thick | Covers the 2 × 2 cable groove round the ring |
| `dorsal_loop_clip` | palm back / forearm back | 8 × 3 × 6 | Fallback loop saddle for a Ø1.8 cable, 2 × M2 |

**For the finger builder (12):** read `out/15/sensor_spec.json`. Per sensor it gives the chip, address, body, the magnet
seat (x range, which body), the board x range, the die gap, the pocket (Ø6.75 counterbore + cap), the outer face and the
width change against 07. It also gives the finger wiring waypoints with glue points and hole positions, the connectors,
and the wrist passage.

Changes to 07:
1. MCP (R2 inner tube): Ø2 × 1.5 magnet in its +x end. The tube and sleeve end at x 6.4.
2. +x bearing at the MCP and PIP becomes an **open ZrO2 MR84 4 × 8 × 2** (seat 1 mm shorter). The PIP −x bush stays.
3. Chip faces the magnet across 0.3 mm; the Ø6.4 × 0.4 board and plug cap fill the Ø6.75 counterbore. 12's +4 mm Hall
   bosses are deleted.
4. DIP pin shortened to x 6.0, with a Ø3 × 1 magnet in its +x end.
5. Brass/316 bar pins and abd stub.
6. Radial wire holes along +y: MCP at **x +3.0** (R2 has −3.0: too short a twist), PIP and DIP at x +3.2 (as R2).
7. Dorsal cross grooves 1.2 × 1.0 at the knuckle hood and at z 38–40.
8. Distal internal channel to the +x DIP board.
9. Phalanx pad pockets 8 × 10 × 4: proximal centred at z 22, middle at z 54, on the palm face.
10. Tip skin magnet Ø3 × 1.0 N42 axial.

---

## 8. BOM (whole hand, `out/15/budget.txt`)

| Item | Qty | € |
|---|---|---|
| TMAG5273 A1/B1/C1/D1 bare chips (Mouser/LCSC; hobby boards stock only A1 [14]) | 20 | 32 |
| MLX90393-014 QFN | 5 | 22.5 |
| BMP581 | 14 | 35 |
| VL53L1X module (optional) | 1 | 4 |
| Magnets: Ø2 × 1.5 and Ø3 × 1 diametric N45 (packs of 50); Ø3 × 1 axial N42 (pack 20) | — | 16 |
| JLCPCB panel: Ø6.4 joint boards, 9.2 × 9.0 tip boards, 5 × 6 pad boards | 1 | 18 (+ setup ~25 if assembled) |
| RP2040-Zero + carrier + LDO | 1 | 11 |
| RS-485 transceivers (only if needed) | 2 | 3 |
| 38 AWG PTFE stranded, 4 colours; 6-core link cable | — | 16 |
| JST SH plugs/sockets | 7 | 4 |
| Brass/316 pins and stubs | — | 5 |
| Open full-ceramic ZrO2 MR84 4 × 8 × 2 ×8 (MCP + PIP sensor side) + MR63 ×4 (abd) [M] | 12 | ~55 |
| **Total** | | **~€185–220** (prices [M] unless report 14 gives them) |

---

## 9. Build order and bench tests

1. **Bench finger, one joint first (PIP):**
   - board + cap + magnet on 08's R9 Hall rig;
   - sweep steel MR84 vs ZrO2 vs brass bush, both directions;
   - **pass: < 1° after the two-way lookup, hysteresis < 0.3°.** This decides the MCP bearing.
2. **Wire twist test W1:** 4 × 38 AWG through a Ø3 tube, ±50° twist over 9 mm, 1 Hz motor rig (08's servo cradle).
   Log the I²C error rate and the resistance every 10⁴ cycles. **Pass: 10⁶ cycles, < 5 % ΔR, no I²C errors.**
   Run 36 AWG beside it as a control; it should fail first.
3. **Full bench finger:**
   - 4 angle chips + tip + 2 pads on one bus, read by an RP2040 over USB;
   - calibration routine (§2.3);
   - DIP vs 0.75 × PIP to read the bar play (expect ±1.4°);
   - series-elastic tension vs a load cell on the flex strand (pass: ±1 N up to 40 N).
4. **Tip:**
   - kitchen scale + shear jig, 0–15 N; fit Bxyz → F;
   - check that saturation stays above 12 N with the Ø3 × 1 magnet;
   - slip test: pull a weighted block until it slips (look for the shear/normal ratio and the BMP581 vibration).
5. **Hub + link:** 7 buses at 400 kHz, 500 Hz frames, ESP32 UART2 with the CRC counter, with all 16 servos running stall
   cycles beside the cable. **Pass: < 10⁻⁵ frame errors.**
6. **Hand:** all fingers, knuckle boards, wrist hollow pins (or the dorsal loop first), firmware 0.5.0 `A`/`T` lines.

---

## 10. Risks

| Risk | Effect | Mitigation |
|---|---|---|
| Steel race hysteresis (bound 1.3–2.6°) | Misses the 1° target at the MCP | Non-magnetic sensor-side bearing; R9 A/B test first |
| ZrO2 open MR84 4 × 8 × 2: C0 **not verified** (full ceramic is usually below steel) | MCP +x lug load ~63 N at SCS 80 % (steel s0 2.2), ~150 N at HLS 80 % (steel s0 0.93) (from 12) | **SCS: ZrO2 open MR84. HLS: 4 × 8 × 2 flanged iglidur G / sintered-bronze bush** (projected area 4 × 2 = 8 mm²; 150 N → 19 MPa against ~80 MPa static for iglidur [14/13, verify]; ~2 % friction). Get a real C0 from a ZrO2 datasheet before buying |
| Hand-soldering Ø6.4 boards and 38 AWG | Build difficulty for a learner | JLCPCB-assembled boards; pads 1.0 × 0.9; a wiring jig; a flex PCB later |
| ZrO2 open bearing not available or too weak | MCP stack needs 1 mm more (+1.0 width) | Iglidur/bronze 4 × 8 × 2 bush (2 % friction); or keep MR84ZZ + 12's 2 mm carrier on the index and little outer faces only |
| Tip magnet saturates the MLX90393 above ~12–15 N | Clipped force at hard grasps | Ø3 × 1 magnet; HLS-level forces → TMAG5273 at ±80 mT as the tip chip (needs a 5th address: use the other bus or the address register) |
| BMP581 pad range (~5 N) [M] | Saturates in a power grasp | Thicker pad / larger cup; measure first |
| Wire fatigue (assumed 0.2 % limit) | Dead bus after 10⁵ cycles | W1 test; 38 AWG; glue points soft; every bus reports chip dropouts |
| Wrist rework (13) cannot hollow the pins | No sealed path | Dorsal service loop (fallback built) |
| RP2040 PIO-I²C library effort | Firmware time | Start with 2 HW buses (index + thumb), add PIO buses after |

## Sources

- Report 14 (TMAG5273 package, addresses, range, supply; MLX90393 saturation, I²C 400 kHz, addresses; BMP581 size/noise;
  servo load/current registers; RP2040-Zero; JST SH; photo-reflector tension sensor).
- Reports 05 (tip, skin, wire twist idea), 07 (`07_finger.py` frames and pockets), 09 (eye plate, centre cell free,
  loop stiffness 21 N/mm), 10 (review items 6, 7, 12, 15), 13 (`out/13/wrist_load.txt`: Ø5 pins, plain bushes).
- `research/references/hands/sensing.md` (ReSkin/AnySkin, ORCA wire failures, Shadow Hall sensors, palm MCU pattern).
- From memory, verify: 52100 coercivity 2–4 kA/m; copper fatigue strain 0.2 % at 10⁶; ZrO2 bearing ratings and prices;
  BMP581 force range in rubber; 38 AWG OD; chip and board prices.
