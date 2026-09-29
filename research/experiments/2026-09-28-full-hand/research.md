# Full hand (v1) research: SCS0009 servos, tendons, proportions, power

Date: 2026-09-28. Scope: research input for the v1 CAD, a 21-DOF tendon-driven hand with one Feetech SCS0009 per joint (antagonistic loop on one spool), 42 strands of 0.4 mm line, and servos in the forearm.

Units are mm, g, N·m and A. Where a number comes from my reading of a drawing, from memory, or from a single non-official source, it is marked **(unverified)**.

---

## 1. Feetech SCS0009: exact specs

Main source: the official Feetech product specification, model SCS0009, edition A/0, dated 2020-11-23 ([Seeed mirror](https://files.seeedstudio.com/products/Feetech/SCS0009-Specifications.pdf), [Switch Science mirror](https://pages.switch-science.com/comparison/files/feetech/serial-scs/SCS0009_datasheet.pdf)). I rendered and read the dimension drawing on page 6 myself.

### 1.1 Mechanical (datasheet drawing, page 6)

| Item | Value | Note |
|---|---|---|
| Body L × W × H (no tabs, no spline) | **23.2 × 12.1 × 25.25** | The table says 23.2. The drawing dimensions the length as 23.3. Design for 23.3. |
| Length over the mounting tabs | **32.5** | 2 × 16.25 from the body centre |
| Tab width | **11.0** (unverified) | The drawing's "11" dimension. It is 1.1 narrower than the body. |
| Tab thickness | **1.6** | The tab underside is 16.8 above the case bottom and the top is 18.4 |
| Mounting holes | **2 × Ø2.0**, **28.5 centre to centre** | 2 × 14.25, symmetric about the body centre. Hole centre is 2.0 from the tab tip. |
| Output shaft, along the length | **5.7 from the body centre**, so **≈5.9 from the near end face** (23.3/2 − 5.7 = 5.95) | Shaft is 8.55 from the nearer hole |
| Output shaft, across the width | Centred (6.05 from each side) | |
| Case top (main body shoulder) | 21.45 above the case bottom | A small cap/boss rises to 25.25 |
| Top of cap around the shaft | **25.25** above the bottom = **6.85 above the tab top** | |
| Top of spline | **28.45** above the bottom = **10.05 above the tab top**, 3.2 above the cap | The drawing also shows 2.9, probably the toothed length of the spline (unverified) |
| Spline / horn | **20 teeth, OD 3.95**, horn screw **M2 × 4** into the shaft | Not the common 25T. The datasheet lists **no accessories**, so a horn may not come in the box. |
| Weight | **13.2 ± 1 g** | 21 servos ≈ 277 g |
| Gears / case | Copper + steel gears, PC case | |
| Gear ratio | 1/416 in the datasheet | An older Feetech FAQ says "SCS009: 256:1" (probably an older model). **Conflict: trust the datasheet.** |
| Backlash | ≤ 0.5° | |
| Cable / connector | 15 cm, Molex 5264-3P, **one lead only** (pin 1 GND, 2 Vcc, 3 signal) | There is no second connector for daisy-chaining. You need a hub or bus board. |
| Position sensor | Carbon-film potentiometer, 330° electrical, 100 000 cycles minimum | |

### 1.2 Electrical

| | 4.8 V | 6 V |
|---|---|---|
| Stall torque | 1.89 kg·cm = **0.185 N·m** | 2.3 kg·cm = **0.226 N·m** |
| Rated (continuous) torque | 0.65 kg·cm = 0.064 N·m | 0.75 kg·cm = 0.074 N·m |
| No-load speed | 0.125 s/60° (80 rpm) | 0.10 s/60° (100 rpm) |
| Idle current (stopped) | 5 mA | 6 mA |
| No-load running current | 120 mA | 150 mA |
| Rated current | 300 mA | 400 mA |
| Stall current | **0.8 A** | **1.0 A** |

- Input range **4.0–7.4 V**. Typical is 6 V. Feetech recommends 4.8–6 V for the SCS009 class ([Feetech URT-1 tutorial](https://www.feetechrc.com/Data/feetechrc/upload/file/20201127/start%20%20tutorial201015.pdf)).
- Protections: overload (load > 80 % for 2 s, then torque drops until a new command arrives), voltage (< 4 V or > 9 V), temperature (> 70 °C). All are adjustable.
- **Design consequence:** a pretensioned antagonistic loop loads the servo all the time. If pretension plus holding load stays above about 80 % for 2 s, the servo **backs off by itself**. Keep pretension low (a few N), or tune registers 37–39.

### 1.3 Control

- Position range **0–1023 = 300°**, so **0.293° per step**. Neutral is 511. Rotation is clockwise for 0 → 1023. Absolute position only: **no multi-turn**. Position is lost outside the pot's range.
- Modes: mode 0 is position servo (default, 0–300°). Mode 2 is "open-loop speed motor mode" (wheel/PWM mode), selected by setting the min and max angle limits to 0. Then the goal-time register holds the PWM value, and bit 10 sets the direction.
- Bus: TTL, half-duplex, asynchronous, 8N1. **Default 1 Mbps** (38 400 bps to 1 Mbps). IDs 0–253 (254 = broadcast). **Default ID = 1**, so give each servo its ID one at a time before chaining them.
- Signal levels: high 2–5 V, low 0–0.45 V, so **3.3 V logic is inside spec**. Max position update rate 1 ms.

### 1.4 Protocol (SCS)

Packet: `0xFF 0xFF ID LEN INSTR PARAM… CHECKSUM`, where LEN = number of params + 2 and CHECKSUM = ~(ID + LEN + INSTR + params) & 0xFF. This comes from `SCS::writeBuf` in the official library.

Instructions: PING 0x01, READ 0x02, WRITE 0x03, REG_WRITE 0x04, ACTION 0x05, RESET 0x0A, SYNC_WRITE 0x83. SYNC_READ 0x82 exists in the library, but **the SCS0009 does not support sync read** ([pollen-robotics/rustypot scs0009.rs](https://github.com/pollen-robotics/rustypot/blob/develop/src/servo/feetech/scs0009.rs): `supports_sync_read: false`). Read feedback one servo at a time.

**Byte order: big-endian (high byte first).** The Feetech FAQ says "SCS series high byte first, SMS low byte first". In the library, `SCSCL()` sets `End = 1`, and `Host2SCS` then writes the **high byte to the "_L" address**. This is the opposite of STS/SMS.

Baud codes (register 6), per Feetech: 0 = 1 M, 1 = 500 k, 2 = 250 k, 3 = 128 k, 4 = 115 200, 5 = 76 800, 6 = 57 600, 7 = 38 400. rustypot has a different table from code 5 up; trust Feetech's.

Register map (official library [ftservo/FTServo_Arduino `src/SCSCL.h`](https://github.com/ftservo/FTServo_Arduino/blob/main/src/SCSCL.h), MIT licence, plus rustypot for the extra addresses):

| Addr | Name | Size | Area | Notes |
|---|---|---|---|---|
| 3 | model / version | 2 | EEPROM R | rustypot: SCS0009 model number is 1284 |
| 5 | ID | 1 | EEPROM RW | |
| 6 | baud rate | 1 | EEPROM RW | codes above |
| 7 | return delay | 1 | EEPROM RW | rustypot |
| 8 | response status level | 1 | EEPROM RW | rustypot |
| 9 | min angle limit | 2 | EEPROM RW | 0 and 0 at 9 and 11 selects wheel mode |
| 11 | max angle limit | 2 | EEPROM RW | |
| 13 | max temperature | 1 | EEPROM RW | |
| 14 / 15 | max / min voltage | 1 / 1 | EEPROM RW | units 0.1 V |
| 16 | max torque limit | 2 | EEPROM RW | 0–1000 = 0–100 % of stall (Feetech FAQ). Useful for gentle grasps. |
| 19 | unloading condition | 1 | EEPROM RW | bit flags: 32 overload, 4 temperature, 1 voltage |
| 21 / 22 / 23 | P / D / I | 1 each | EEPROM RW | |
| 24 | min startup force | 2 | EEPROM RW | |
| 26 / 27 | CW / CCW dead band | 1 each | EEPROM RW | |
| 37 / 38 / 39 | protective torque / protection time (×40 ms) / overload threshold (%) | 1 each | RW | see the overload note above |
| **40** | **torque enable** | 1 | SRAM RW | 0 = free, 1 = holding |
| 41 | acceleration | 1 | SRAM RW | rustypot. The library forces ACC = 0 on SCS. |
| **42** | **goal position** | 2 | SRAM RW | 0–1023 |
| **44** | **goal time** | 2 | SRAM RW | ms. In wheel mode this is the PWM (bit 10 = direction). |
| **46** | **goal speed** | 2 | SRAM RW | 0 = max. The unit is unclear: the example comments say 0.059 rpm per unit (written for SCS15), rustypot says steps/s. **Calibrate.** |
| 48 | lock | 1 | SRAM RW | write 0 to unlock the EEPROM before changing the ID, then 1 |
| **56** | **present position** | 2 | SRAM R | |
| 58 | present speed | 2 | SRAM R | sign on bit 15 |
| **60** | **present load** | 2 | SRAM R | 0–1000 = PWM duty to the motor (‰), sign on bit 10. This is **not** a current reading. |
| **62** | **present voltage** | 1 | SRAM R | 0.1 V |
| **63** | **present temperature** | 1 | SRAM R | °C |
| 65 | status | 1 | SRAM R | |
| 66 | moving | 1 | SRAM R | |
| 69 | present current | 2 | SRAM R | listed in the library, sign on bit 15. **May be unimplemented on the SCS0009** (a Feetech FAQ says the SCS series has no current detection). |

**Library.** Use the official **FTServo_Arduino** (Feetech, MIT). The older "SCServo" copies (for example `workloads/scservo`) have identical SCSCL code. Class **`SCSCL`** (`#include <SCServo.h>`). Set `sc.pSerial = &Serial1` after `Serial1.begin(1000000, SERIAL_8N1, RX, TX)`. Functions:
- `WritePos(id, pos, time, speed)`, `RegWritePos` + `RegWriteAction`, `SyncWritePos(ids[], n, pos[], time[], speed[])`
- `EnableTorque(id, 0/1)`, `unLockEprom(id)` / `LockEprom(id)`, `writeByte(id, SCSCL_ID, newId)`
- `FeedBack(id)` (reads 56–70 in one go), `ReadPos`, `ReadSpeed`, `ReadLoad`, `ReadVoltage`, `ReadTemper`, `ReadMove`, `ReadCurrent`
- `PWMMode(id)`, `WritePWM(id, pwm)`, `Ping(id)`

**Bus timing (estimate).** At 1 Mbps a byte takes 10 µs. A SYNC_WRITE of goal position + time + speed to all 21 servos is 8 + 21 × 7 ≈ 155 bytes ≈ **1.6 ms**. With no sync read, one position read is about 8 bytes out + 8 bytes back plus the turnaround, about 0.2–0.3 ms, so all 21 take **≈5–6 ms**. That gives about **100–150 Hz** for full-state feedback.

### 1.5 FE-URT-1 to ESP32-S3

From the Feetech URT-1 tutorial (link above, FAQ items 6–7 and its wiring figure):
- Connect the URT-1 UART header **TX–TX, RX–RX, V–5 V, GND–GND**. The labels are printed from the MCU's side, so **do not cross them**. If nothing responds, try swapping them. Owners on the Arduino forum confirm it works with an ESP32 at 1 Mbps ([thread](https://forum.arduino.cc/t/how-to-connect-esp-32-to-feetech-urt-1-to-control-scs-serial-bus-servos/1051765)).
- Set the board's **level switch to 3.3 V** for the ESP32. The figure note says "Arduino level switch 5V, STM32 level switch 3.3V".
- **"USB port and UART port cannot be used at the same time."** Leave the URT-1's mini-USB unplugged when the ESP32 drives it.
- The URT-1 does **half-duplex switching in hardware** (no direction/enable pin), so a plain `HardwareSerial` works.
- Servo power goes into the blue screw terminal (4.8–8.4 V for SCS). Its current rating is not published (unverified). For 21 servos, feed power through your own distribution board and let the URT-1 carry only signal and GND.
- ESP32-S3: use UART1 on any free GPIO, for example TX = GPIO 17 and RX = GPIO 18. None of the octal-PSRAM or strapping-pin problems from v0 apply, and PSRAM can be turned back on. The ESP32-S3 is **not 5 V tolerant**, so never skip the URT-1 and wire a servo line straight to a GPIO.

---

## 2. Tendon force budget and spool radius

The tendon force at the spool is F = τ / r_spool. With a joint drum of radius r_joint and both strands of the loop on it, the joint torque is τ_joint = τ_servo × r_joint / r_spool. A joint angle Δq needs a servo angle of Δq × r_joint / r_spool.

SCS0009 at 6 V, joint drum r_joint = 6 mm:

| r_spool | Tendon force at stall / rated | Joint torque at stall | Servo degrees per joint degree | Joint resolution |
|---|---|---|---|---|
| 5 mm | 45.1 / 14.7 N | 0.271 N·m | 1.2 | 0.24° |
| **6 mm** | **37.6 / 12.3 N** | **0.226 N·m** | **1.0** | **0.29°** |
| 8 mm | 28.2 / 9.2 N | 0.169 N·m | 0.75 | 0.39° |
| 10 mm | 22.6 / 7.4 N | 0.135 N·m | 0.6 | 0.49° |

Tendon excursion with a 6 mm moment arm:

| Joint | Range | Excursion | Servo angle at r = 6 |
|---|---|---|---|
| MCP flex | −10…+90° (100°) | 10.5 mm | 100° |
| PIP | −5…+100° (105°) | 11.0 mm | 105° |
| DIP | −5…+80° (85°) | 8.9 mm | 85° |
| MCP abd | ±20° (40°) | 4.2 mm | 40° |
| Thumb CMC rot (v0 range) | −100…+40° (140°) | 14.7 mm | 140° |

**Travel is not the constraint.** Even the thumb rotation uses under half of the servo's 300° at r = 6 mm. **Force is the constraint.** At 1:1 the index MCP gives about 0.23 N·m at stall and about 0.07 N·m continuous. Over an 80 mm finger that is about **2.8 N at the fingertip at stall and about 0.9 N continuous**, before friction losses (see section 3).

**Recommendation: r_spool = 6 mm**, measured at the tendon centreline, so the groove bottom sits at 5.8 mm.
- It is the smallest radius that still leaves a solid hub around the 3.95 mm spline and the M2 screw.
- It gives 1:1 with the current 6 mm drums, which makes calibration trivial (steps/rad = 1024/300° × r_j/r_s).
- It is still 15× the line diameter, so there is no bending damage to the line.
- **Do not go to 8–10 mm.** You don't need the travel, and you lose 25–40 % of the force.
- If you need more force, **raise r_joint instead** (for example an 8 mm MCP drum gives 0.30 N·m at the joint and 133° of servo travel). You could also drop to r_spool = 5 mm.
- Keep r_joint the same for the flexor and extensor strands of a loop, so both strands change length equally. That is the condition for a single-spool antagonistic loop to stay tight ([Bowden antagonistic hand, arXiv 2512.24657](https://arxiv.org/html/2512.24657v1): residual length mismatch ≈ 0.03 mm).
- Each strand wraps less than half a turn (14.7 mm on a 37.7 mm circumference). Give each strand its own groove on a two-groove spool, about 1 mm groove width and flanges, total width about 5–6 mm, with an anchor hole per strand.
- Build in **pretension adjustment**. Options: a two-part spool clamped by a screw (Shadow Hand), a ratchet spool (ORCA), or a tension screw per strand.
- Line strength: 0.4 mm nylon mono breaks at roughly 80–100 N and braided PE is stronger (typical catalogue values, unverified). That is above the 37.6 N stall force, so a safety factor of about 2 or more.

---

## 3. Reference designs and routing

| Hand | Actuation and routing | Take-away for us |
|---|---|---|
| **ORCA** (ETH SRL, 2025) [arXiv 2504.04259](https://arxiv.org/abs/2504.04259) | 17 DOF. 16× Dynamixel XC330-T288 + 1× XC430 in a tower below the hand. **Two tendons per joint (flexor + extensor)**, 0.4 mm braided nylon. DIP is fixed/coupled. Tendons are deflected around **smooth metal pins and rods**, with **Teflon tubes for non-linear routing** (for example thumb base to wrist). Tendons are routed **through the joint centre of rotation**. They avoid tendon contact with PLA ("every tendon only touches metal and smooth PTFE"). Ratchet spool for re-tensioning. Wrist uses a GT2 belt. Survived more than 10 000 cycles. | This is closest to ours. Copy it: no line-on-PLA sliding, PTFE for curved runs, pins at sharp deflections, and a quick retension. |
| **Shadow Dexterous Hand** [docs](https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/stable/user_guide/md_motor_unit.html) | 20 motors in the forearm, **each driving a pair of agonist/antagonist Spectra tendons on one spool** (exactly our scheme). Split spool for preload, tensioner units, and a 90° turn around a metal bar at the spool exit (which also carries a load cell). Small motors: 65 N continuous, 110 N maximum tendon force. | This validates one spool per joint. The bar at the spool exit is a good pattern: a fixed deflection point, so the angle into the tube never changes. |
| **Antagonistic Bowden hand** [arXiv 2512.24657](https://arxiv.org/html/2512.24657v1) | One Dynamixel per joint, both strands on the same shaft. PTFE sheath **1 mm ID / 2 mm OD** plus a spring jacket, and 0.75 mm coated steel line. Bobbins keyed with an octagon so preload can be set. Keeps bend radii above the minimum. | Confirms PTFE 1/2 mm for sub-1 mm line. |
| **MM-Hand** [arXiv 2604.17245](https://arxiv.org/html/2604.17245) | 21 DOF with a remote Feetech motor hub. Tested sheaths: a plain PTFE tube had "relatively high friction" (extrusion quality). The Bambu PTFE feeder tube and a metal spring tube were better. Software pretension. | Buy good-quality (smooth-bore) PTFE. |
| **DLR Hand Arm System / Awiwi** | About 38 motors in the forearm, two per joint (antagonistic, variable stiffness), Dyneema tendons (from memory, unverified). | Two motors per joint is the high-end option. We use one plus a loop. |
| **InMoov** | Hobby servos in the forearm, one per finger, pull-pull loop of braided line on the horn, printed channels (from memory, unverified). | Bare printed channels work for hobby loads but wear and have high friction. |
| **DexHand** (The Robot Studio) [repo](https://github.com/TheRobotStudio/V1.0-Dexhand) | 16 slim micro servos in the forearm, Sufix 832 braid (80 lb) for the fingers, Feetech SCS2332/SCS15 on the wrist. | Braided PE is the popular line. |
| **ILDA** (Kim et al., 2021) | Linkages driven by motors in the palm, **not tendons** (unverified). | Not comparable. |
| **LEAP Hand** (CMU) | Dynamixels directly in the joints, **not tendons**. | Not comparable. |

**Friction (capstan).** F_out = F_in · e^(−μθ), where θ is the **sum of all bend angles** along the strand (in rad).
- μ for line in a PTFE sheath: **0.04–0.2** (MM-Hand and Bowden literature).
- μ on bare printed PLA: plan **0.3**. Pin-on-disc PLA gives 0.38–0.57 ([Rapid Prototyping J. 2022](https://www.emerald.com/insight/content/doi/10.1108/rpj-03-2022-0081/full/html)), and layer lines abrade the line.
- Transmitted fraction:

| μ \ total wrap | 45° | 90° | 180° | 360° |
|---|---|---|---|---|
| 0.1 (PTFE) | 0.92 | 0.85 | 0.73 | 0.53 |
| 0.2 | 0.85 | 0.73 | 0.53 | 0.28 |
| 0.3 (PLA) | 0.79 | 0.62 | 0.39 | 0.15 |

- Friction scales with tension, so **pretension costs force twice**: it loads both strands and the servo all the time.

**Recommendations**
- **Channel for PTFE tube 1 mm ID × 2 mm OD: 2.2 mm hole** (a snug slip fit in PLA; FDM holes print about 0.1–0.2 mm undersize, so test-print 2.1 / 2.2 / 2.3). Use tubes from the palm base through the wrist to the servo, where the path curves. Anchor each tube end so it cannot slide; a stepped hole down to 1.2 mm makes a shoulder.
- **Bare channel (no tube), 0.4 mm line: 1.2 mm** modelled, which prints to about 1.0 mm. Only use this for short, **straight** runs inside the fingers, or line it with a smooth pin or tube wherever the strand turns.
- **Channel pitch:** at least 3.0 mm centre to centre for 2 mm tubes, which leaves a 1 mm wall. 42 tubes in a 7 × 6 grid take about **21 × 18 mm**, which fits through a wrist about 30 × 45 mm.
- **Minimum bend radius:** tube runs ≥ 15 mm (PTFE 1/2 mm kinks below about 10 mm, unverified). Bare-line deflection over metal pins or a spool ≥ 3 mm (≥ 7× the line diameter); 5 mm is better. Printed curved channels without a tube: avoid, or keep ≥ 10 mm radius and ≤ 30° total.
- **Maximum total wrap per strand:** aim for **≤ 90°** from spool to joint (about 85 % efficiency in PTFE), and never more than 180°. This does not count the wrap on the joint drum itself, which moves with the joint and does not slide.
- Put **fixed deflection points** (a metal pin or bar) where a strand leaves the spool, so the tube entry angle doesn't change as the spool turns. Shadow uses a 90° turn over a bar here.
- **Wrist:** v1 has no wrist DOF, so run the tubes straight through the wrist. If a wrist DOF comes later, route the whole bundle **through the wrist axis**, as with the finger joints, so wrist motion doesn't change tendon lengths.
- Pass-through strands must cross each joint **on its axis** (current rule), or run inside a sheath fixed on both sides of the joint (Bowden-style), which also makes them independent of that joint.

---

## 4. Human hand proportions and ranges

**Phalanx lengths** (bone lengths from X-rays, 66 adults, right hand, mean in mm) ([Buryanov & Kotiuk 2010, Int. J. Morphol. 28(3)](https://scielo.conicyt.cl/pdf/ijmorphol/v18n3/art15.pdf)):

| Finger | Proximal | Middle | Distal | Metacarpal | Tip soft tissue |
|---|---|---|---|---|---|
| Thumb | 31.57 | — | 21.67 | 46.22 | 5.67 |
| Index | 39.78 | 22.38 | 15.82 | 68.12 | 3.84 |
| Middle | 44.63 | 26.33 | 17.40 | 64.60 | 3.95 |
| Ring | 41.37 | 25.65 | 17.30 | 58.00 | 3.95 |
| Little | 32.74 | 18.11 | 15.96 | 53.69 | 3.73 |

**Ratios relative to the same phalanx of the index**, and scaled to our index proximal = 40 mm (v0 has 40 / 23):

| Finger | Proximal ratio | Middle ratio | Distal ratio | Whole finger ratio | Scaled P / M / D (mm) |
|---|---|---|---|---|---|
| Index | 1.00 | 1.00 | 1.00 | 1.00 | 40.0 / 22.5 / 15.9 |
| Middle | 1.12 | 1.18 | 1.10 | 1.13 | 44.9 / 26.5 / 17.5 |
| Ring | 1.04 | 1.15 | 1.09 | 1.08 | 41.6 / 25.8 / 17.4 |
| Little | 0.82 | 0.81 | 1.01 | 0.86 | 32.9 / 18.2 / 16.0 |
| Thumb | 0.79 (P) | — | 1.37 (D vs index D) | — | 31.7 / — / 21.8, metacarpal 46.5 |

These are joint-to-joint bone lengths. Add about 4 mm of pad at the fingertip (6 mm on the thumb).

**Knuckle layout**
- **MCP pitch ≈ 22 mm.** Shadow Hand, a human-sized design, uses 22.0 mm: knuckles at x = +33, +11, −11, −33 mm ([MuJoCo Menagerie shadow_hand](https://github.com/google-deepmind/mujoco_menagerie/tree/main/shadow_hand)). Human palm breadth over metacarpals II–V is about 80–90 mm on skin (unverified), which also gives about 20–22 mm between joint centres.
- **Knuckle arc (distal–proximal):** middle is the most distal. In Shadow, the index and ring knuckles sit 4 mm behind the middle and the little knuckle about 12.5 mm behind (95 / 99 / 95 / 86.5 mm from the palm origin). Use about **0 / −4 / −4 / −12 mm** for middle / index / ring / little.
- **Transverse arch:** real knuckles lie on a dorsal arch, with the ring and especially the little knuckle dropping toward the palm by a few mm. Shadow adds a little-finger metacarpal joint (0–45°) for cupping. For v1, lower the little and ring knuckles by about 3–6 mm (unverified estimate) or leave them flat for simplicity.
- **Finger splay at rest:** a small fan of about 0 / +3 / −3 / −8° (middle / index / ring / little, positive toward the thumb) looks natural. Each finger then gets its own ±20° abduction (estimate).

**Thumb, 5 DOF (typical robot layout, like Shadow TH1–TH5):**
- CMC has 2 DOF: rotation/opposition, and flexion/abduction (two roughly perpendicular, non-intersecting axes on the metacarpal base).
- MCP has 2 DOF: flexion, plus a small abduction.
- IP has 1 DOF: flexion.
- The thumb base sits low on the palm, about 60–70 mm proximal of the index knuckle and rotated about 45° (Shadow thumb base quaternion ≈ 45° about y).

**Joint ranges for all 21 joints** (positive = closing).
- Human sources: AAOS norms (finger MCP 90, PIP 100, DIP 90, thumb MCP 50, IP 80, CMC abduction 70, CMC flexion 15 / extension 20) via [goniometer.io](https://goniometer.io/range-of-motion).
- Robot references: ORCA Table I, and the Shadow model.

| Joint | Human (approx.) | ORCA / Shadow | **Suggested design range** |
|---|---|---|---|
| index/middle/ring/little `mcp_abd` | about ±20° (unverified) | ±30 / ±20 | **−20…+20°** (middle ±15°) |
| `mcp_flex` | −45 (hyperextension)…+90 | −20…+110 / −15…+90 | **−10…+90°** |
| `pip` | 0…+100 | −20…+130 / 0…+90 | **−5…+100°** |
| `dip` | −10…+90 | coupled / 0…+90 | **−5…+80°** |
| thumb `cmc_rot` (opposition) | about 70° abduction arc | −53…+48 (CMC) / ±60 (TH5) | **−100…+40°** (keep v0) or −60…+60 |
| thumb `cmc_flex` | −20…+15 flexion, 70 abduction | 0…+70 (TH4) | **−15…+70°** |
| thumb `mcp_abd` | about ±10° (unverified) | ±45 / ±12 (TH3) | **−15…+15°** |
| thumb `mcp_flex` | 0…+50 (up to 60) | −20…+115 / ±40 (TH2) | **−10…+60°** |
| thumb `ip` | −20…+80 | −20…+100 / −15…+90 (TH1) | **−10…+80°** |

For comparison, functional daily-use ranges are much smaller: about MCP 61°, PIP 60°, DIP 39°, thumb MCP 21°, IP 18° (Hume et al. 1990, [PubMed 2324451](https://pubmed.ncbi.nlm.nih.gov/2324451/), recalled and not re-read). So a hand that hits the design ranges above covers daily tasks with margin.

---

## 5. Power for 21 × SCS0009

| Case | Estimate |
|---|---|
| All idle, torque on, no load | 21 × 6 mA ≈ **0.13 A** |
| Holding against pretension (always on for loops) | 21 × about 0.1–0.2 A ≈ **2–4 A** (unverified; depends on pretension) |
| Typical motion (a few servos moving plus holding) | **3–6 A** |
| Strong grasp (about 10 flexors near stall, the rest holding) | 10 × 1.0 + 11 × 0.2 ≈ **12 A** |
| Absolute worst case (all 21 stalled at 6 V) | **21 A** (short spikes; the overload protection cuts in after about 2 s) |

- **Voltage:** 6 V gives the full 0.226 N·m. 5 V gives about 0.2 N·m (interpolated). Both are in range.
- **Supply:** **Mean Well LRS-100-5 (5 V 18 A, adjustable 4.5–5.5 V; set it to 5.5 V)** ([spec](https://protosupplies.com/product/mean-well-power-supply5v-18a-90w/)). It covers everything except the theoretical all-stall case. If you want the full 6 V, use a 12 V supply with two or three 6 V / 8–10 A buck modules, one per branch. The current 5 V / 2 A supply is only enough for bench testing a few servos.
- **Bus wiring:** each SCS0009 has **one lead**, so build a **bus board**: a strip of 5264-3P (or 2.54 mm) headers sharing GND / V+ / signal.
  - Split the 21 servos into **three branches** (thumb 5, index + middle 8, ring + little 8).
  - **Inject power into each branch separately** from the supply (a star, not one long chain).
  - Use **18 AWG** per branch (≤ 10 A) and **16 AWG** from the supply to the distribution point.
  - The servo leads themselves are thin (about 26–28 AWG, 15 cm): about 64 mV drop at 1 A, which is fine.
  - Signal is one shared line for all branches: URT-1 channel B signal to all three branch signal pins.
  - Tie GND from the supply, the URT-1 and the ESP32 together at one point.
  - Add a **1000–2200 µF low-ESR capacitor** on each branch and a **fuse** (about 15 A) at the supply.
- **Firmware safety** (same spirit as v0): leave torque disabled at boot (register 40 = 0), and only enable after the PC commands it. Set register 16 (torque limit) as a soft current cap per joint.

---

## 6. Summary: recommendations for the CAD

| Item | Recommendation |
|---|---|
| Servo body (nominal) | 23.3 × 12.1 × 25.25 mm; 32.5 mm over the tabs; tabs 11 wide × 1.6 thick, underside 16.8 above the bottom |
| **Servo pocket (PLA)** | **23.7 × 12.5 mm** (+0.4 / +0.4), depth to the tab seat **16.8 mm**. Tab slots **11.4 wide** out to 32.9 mm total. Leave a cable exit slot at the bottom end (about 4 × 3 mm). |
| Mounting | 2 holes **28.5 mm apart**, symmetric about the body centre. Use **M2 self-tappers into Ø1.6–1.7 mm pilots** (or Ø2.2 clearance + M2 nut). |
| Output shaft | **5.7 mm from the body centre** (≈5.9 from the end face), centred across the width. Spline top 10.05 mm above the tab top; cap 6.85 above the tab top. Keep a Ø14 mm clearance for the spool above the cap. |
| Spline | **20T, OD 3.95 mm**, M2 × 4 screw. Print test spools with spline bores of 3.95 / 4.0 / 4.05 mm, or bond in a metal 20T horn. |
| Servo pitch in the forearm | ≥ 14 mm side by side (12.1 + 1.9 wall), ≥ 34 mm end to end |
| **Spool radius** | **6 mm** at the tendon centreline (1:1 with the 6 mm joint drums). Two grooves, about 1 mm wide, flanged, 5–6 mm total width. Add a preload adjustment (split spool, ratchet or tension screw). |
| Joint drum radius | 6 mm (same for flexor and extensor). Go to 7–8 mm at the MCPs if space allows, for more torque. |
| **Channel: PTFE tube 1 × 2 mm** | **Ø2.2 mm** hole, stepped to Ø1.2 mm at the tube stop; pitch ≥ 3.0 mm. 42 tubes ≈ 21 × 18 mm through the wrist. |
| Channel: bare line (short, straight only) | **Ø1.2 mm** modelled |
| Minimum bend radius | PTFE tube ≥ 15 mm; line over a metal pin or spool ≥ 3 mm (5 mm preferred); printed curved channel: avoid (if needed, ≥ 10 mm and ≤ 30°) |
| Maximum total wrap per strand | ≤ 90° target, ≤ 180° absolute (spool to joint, not counting the joint drum) |
| Deflection points | Metal pins (Ø2–3 mm, for example steel dowels) at every sharp turn and at the spool exit; no line sliding on PLA |
| **Knuckle pitch** | **22 mm** (index / middle / ring / little at +33 / +11 / −11 / −33 mm) |
| Knuckle arc | Middle 0, index −4, ring −4, little −12 mm (proximal offset); optional 3–6 mm palmar drop for ring and little |
| **Finger length ratios vs index** (P / M / D) | Middle 1.12 / 1.18 / 1.10; Ring 1.04 / 1.15 / 1.09; Little 0.82 / 0.81 / 1.01. With index = 40 / 22.5 / 15.9 mm, that gives middle 44.9 / 26.5 / 17.5, ring 41.6 / 25.8 / 17.4, little 32.9 / 18.2 / 16.0. |
| Thumb | Metacarpal 46.5, proximal 31.7, distal 21.8 mm; 5 DOF (CMC rotation + flexion, MCP abduction + flexion, IP) |
| Joint ranges | See the table in section 4 |
| Bus and power | 1 Mbps; IDs set one servo at a time; SYNC_WRITE for commands; ~100–150 Hz read-back. 5–6 V, ≥ 18 A supply; 3 power-injected branches, 18 AWG. |

### Open questions / things to measure on the real parts

1. The tab width (11 mm) and the spline length (2.9 vs 3.2 mm) come from the drawing. Measure a real servo with calipers before finalising the pocket.
2. Does the owner's SCS0009 come with a horn? If not, the spool must key onto the bare 20T spline.
3. The goal-speed unit, and whether present current (address 69) works on the SCS0009: test on the bench.
4. The FE-URT-1 screw-terminal current rating: don't route the full servo current through it until it's known.
5. Measure μ for the chosen line in our PTFE tube with a simple capstan test (hang a weight, measure the pull through a known bend) before fixing the maximum wrap angle.
