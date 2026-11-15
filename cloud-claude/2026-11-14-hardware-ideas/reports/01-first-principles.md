# 01 · First-principles architecture: how to make the hand stronger, stiffer and simpler

Agent 01 (first-principles architect), 2026-11-14. Study only; nothing outside this folder was changed.
Calculations: [`cad/01_force_budget.py`](../cad/01_force_budget.py) (run with
`uv run --no-project --with numpy python cad/01_force_budget.py`), output in
[`out/01/01_force_budget.txt`](../out/01/01_force_budget.txt) and `.json`. Hand calculations, no FEA.

**Read this first: the physics in one paragraph.** A fingertip force F needs a joint torque τ = F × ℓ, where ℓ
is the lever from the joint to the force's line of action. In a tendon hand,
τ_joint = τ_servo × (joint moment arm ÷ spool radius) × η. Here η is the friction efficiency
e^(−μθ) (θ = total sliding turn of the strand, μ = friction coefficient). The joint also behaves like a spring.
Its stiffness is K = (moment arm)² × k_strand, with k_strand = EA ÷ L for the line (EA = axial stiffness,
L = length) in series with the sheath. So there are only **four levers**: servo torque, the ratio
(arm ÷ spool), η, and stiffness. Everything below pulls one of them.

**Main assumptions** (all named constants in the script):
- μ = 0.10 for braid on PTFE or polished steel; 0.99 per ball-bearing idler.
- Index finger 42 / 24 / 23 mm. Tip pinch at MCP 30°, PIP 45°, DIP 34° (= 0.75 × PIP).
- Line EA ≈ 39 × break load (pre-stretched braid at its working load).
- Servos: SCS0009 0.226 N·m stall / 0.074 N·m rated; HLS3606M 0.59 / 0.15 N·m; STS3215 (12 V) 2.94 / ~0.9 N·m (verify).

---

## 1. Top 3 recommendations

**1. Delete the Bowden sheaths. Run bare strands in straight lines on a rigid path, and cross the wrist through
an "eye plate" in the gimbal's middle plane. Firmware cancels the small length changes.**
In V2 the PTFE sheaths carry the whole strand load in compression. That makes the drive soft: the loop is
only ≈ 5 N/mm, so a 10 N tip force would bend the MCP by ~100°, which means it cannot hold 10 N at all. They
also cost friction: the index MCP flexor turns 223° in total, which leaves η = 0.68. All 32 strands have a
median of 185° at a pinch with the wrist straight. With bare braid on a straight path and two idlers, the MCP
flexor turns only ~25° (η = 0.94, **+38 % force**). The loop becomes **60 N/mm (12× stiffer)**, and sheath
creep, sheath end stops, the forearm fan and the take-up problem all disappear.
The wrist needs no sheath either. Each strand passes a small steel eye on a plate in the plane of the two
wrist axes. Its length then changes linearly with the wrist angles (≤ 4.3 mm at ±50°/−30…+20°), and the
servo cancels this exactly, because the wrist servos know their angles. The loop length does not change:
pair each flex/ext strand point-symmetrically on the plate, and the common-mode change is
**0.036 mm (≈ 2 N of pretension)**, against ±6.7 mm per tube in V2.
This deletes about 70 parts: 32 sheaths, 32 barrel adjusters, the comb, the fan and the palm clamp.

**2. Give each joint its own ratio: the MCP gets the smallest metal-hub spool, and the PIP gets a 2:1 "reeved"
drum. With the servos you already own, this doubles the pinch force.**
The pinch maths shows something the earlier notes missed. With the DIP coupled to the PIP, **the PIP runs out
of torque first in a pinch**, not the MCP. Its drive lever is 46 mm (PIP + 0.75 × DIP), against 43 mm for the
MCP. Both must be strengthened together.
- **PIP: a 2:1 reeved drum.** Each strand wraps the drum, turns 180° round a hardened Ø3 pin on the child
  segment, and runs back in a second groove to an anchor on the parent. Both segments then pull at 7 mm, so
  the moment arm is ≈ 12.5 mm (after the 22 % loss at the pin) for the same strand tension. The joint is also
  ≈ 3.2× stiffer.
- **MCP: a plain drum** (the pip strands pass through its middle, so there is no room for 2 grooves), on a
  r 3.0 metal-hub spool.

With stock SCS0009s: **pinch 4.5 N (V2) → 10.3 N at stall** (≈ 8 N inside the servo's 80 %/2 s overload rule,
3.4 N continuous). Joint speed is 236°/s, and servo travel 233–255° of the 300° available.
Hardware: ~€30 (dowels, metal hubs, idlers).

**3. Put strong servos only on the 11 joints that make force, and add joint-angle sensors.**
The strong joints are 4 × MCP flex, 4 × PIP, and the thumb cmc_flex, mcp_flex and ip. The 4 abductions and
cmc_rot stay on SCS0009.
- **HLS3606M with the balanced drive of #2:** **25.7 N pinch at stall, 6.5 N continuous, €353 of servos.**
  This is the cheapest realistic ≥ 10 N hand that stays compact (293 g of servos).
- **45 N needs ~300 N in the MCP strand:** STS3215 (12 V) with 200 lb braid and metal drum sleeves gives
  **45.7 N** for €221 of servos, but +380 g and a 12 V bus. A BLDC test joint is the other route.

Stronger means springier. At 25 N the strand stretch alone (150 lb braid, 230 mm) lets the fingertip sag
≈ 11°, so **a €1–2 magnet + Hall sensor at each MCP/PIP becomes a requirement**. It also turns the tendon into
a force sensor (1° of MCP ≈ 1.2 N at the tip with 80 lb braid).

### Requirements made less dumb (step 1 of the 5-step algorithm)

| Requirement as stated | Problem | Better requirement |
|---|---|---|
| "45 N like 1X NEO" | That is a peak spec from a hand with in-house motors | V2: **≥ 10 N continuous-capable, ≥ 25 N peak** tip pinch. 45 N as a later actuator swap |
| "Same mm per degree at every joint" | It forces one ratio, but the joints have different levers | Each joint gets its own ratio. Firmware already scales per joint |
| "Zero coupling in hardware" | It forced sheaths through the wrist and eyelets at the MCP | Allow **linear, known** coupling and cancel it in firmware (the servos are position-controlled) |
| "Every strand in a sheath" | The sheath is the softest, creepiest and most friction-heavy part | Sheaths only where the path itself moves, and none at all with the eye plate |
| "Sustained force = stall force" | Servos are rated at 25–33 % of stall | Hold with ≥ 2.5 N per finger. With silicone pads (μ ≈ 1) that already holds 0.5 kg in a two-finger pinch |

---

## 2. Idea list (wide)

Tip forces are for the index tip pinch unless noted, from the script's ladder (§4, §4d, §8). "Stall" = peak.

| # | Idea | How it works | Gain (number) | Cost / effort | Risk | Verdict |
|---|---|---|---|---|---|---|
| 1 | **Bare strands, rigid straight path** | No sheaths. Strands run in straight lines between steel eyes and pins fixed to the rigid palm and forearm. Open troughs with lids | η 0.68 → 0.88 (steel pins) / 0.94 (idlers). Loop 5 → 60 N/mm. −~70 parts | M (re-layout palm + forearm) | Strands exposed inside the shells; lines must not touch (skew-line check in CAD) | **Do** |
| 2 | **Wrist eye plate + firmware compensation** | 32 Ø0.84 steel eyes on a 1.6 mm grid, on the cardan's middle ring, in the plane of both axes. ΔL = x·θ_dev + y·θ_flex, cancelled per servo | Common mode 0.036 mm (2 N). Differential ≤ 4.3 mm (≤ 49° of servo at r 5). No sheath take-up | S–M | Needs the wrist angle (servo encoders); SCS0009 travel budget; eye wear when the wrist moves under load | **Do**, with #1 |
| 3 | Short steel-coil housing across the wrist only | Keep a ~60 mm compressionless coil (PTFE-lined) only across the gap | Coil ~1000+ N/mm vs PTFE 5–20 N/mm. Keeps zero-coupling | S | +10–15 % friction at wrist bends; 2.5–3.2 mm OD needs bigger holes | Fallback for #2 |
| 4 | **2:1 reeving at the PIP** | Strand wraps groove A, turns 180° round a Ø3 pin on the child, runs back in groove B, anchors on the parent | Moment arm 7 → 12.5 mm (1.78×). Joint stiffness ×3.2. Strand load unchanged | S (one pin + 2nd groove) | Fraying at the 180° pin (D/d 6.5); fits only if the drum is ≤ 7 mm wide | **Do** (PIP). The MCP does not fit (see §3.2) |
| 5 | **Own spool radius per joint** (metal hub) | MCP strand on a r 3.0 (SCS) / 3.5 (HLS) metal hub, not the r 5 printed spool | MCP ×1.67 (SCS), ×1.43 (HLS) | S (cut-down metal horn or turned sleeve) | SCS0009 300° travel limit; strand load ×1.4–1.7 | **Do** |
| 6 | **HLS3606M on the 11 strong joints** | Same plan as SCS0009, 2.6× torque, current sensing, multi-turn | Pinch 10.3 → **25.7 N** stall, 6.5 N continuous | €200 extra | 25T spline (new hubs); separate HLS driver | **Do next** |
| 7 | STS3215 (12 V) on the strong joints | The LeRobot arm servo: 2.94 N·m for ~€16 | **45.7 N** pinch with 300 N strands; 19.6 N continuous | €221 servos, 12 V supply, forearm +430 cm³ | 55 g each (+380 g); strand 300 N needs 200 lb line, metal drums, 684 bearings | **Path to 45 N** (bench-test one) |
| 8 | FDP idler at the MCP | The PIP flexor passes the MCP on the palm side over a ρ 3 mm idler, so it adds MCP torque. Coupling 0.43 is cancelled in firmware | Pinch +0 % (PIP limits). **Straight push +36 %** (3.7 → 5.0 N) | S | Coupling eats PIP servo travel (+60°) | Maybe. Use an on-axis idler anyway (no fraying eyelet) |
| 9 | Twisted-string actuator (TSA) | A small DC motor twists two strings; the strings shorten with a huge reduction | ~50 N from a €3–15 gearmotor; 30 % stroke | M–L (16 H-bridges + encoders) | String life 10⁴–10⁵ cycles, non-linear, pulls one way only | Research spike, one joint |
| 10 | Spring-return extensors (Tesla-like) | One flexor strand per joint; a spring or elastic extends | Halves strands through the wrist (32 → 16). Lifts the loop's ratio cap (§3.2) | S | −10–20 % flex force; weak, slow extension; stored spring energy | No for fingers; maybe for the abductions |
| 11 | Motors in the palm (LEAP / AmazingHand) | Servos at the finger base, 40 mm strands or direct drive | Loop 347 N/mm (6× stiffer); no wrist routing | M | +200–330 g at the wrist; a 40 mm thick palm; only SCS size fits | No (maybe the 4 abduction servos) |
| 12 | N20 + worm in the fingers | Self-locking gearmotor per joint | Holding costs 0 W | M | 30–40 % efficient, not backdrivable (bad for RL and safety), 16 drivers, mass in the fingers | No |
| 13 | Linkage fingers (Inspire / Ability) | One linear actuator per finger + 4-bars | 10–20 N, robust | L (redesign) | 6 DOF instead of 16: loses the dexterity goal | No (reference only) |
| 14 | Power-grasp synergy tendon | One STS3215 pulls a whiffletree in the palm that tensions all 4 fingers' second groove | +~15 N per finger in power grasp for €16 | M | Fights the precision servos; palm space | Only if #6/#7 are not done |
| 15 | Grip ratchet / pawl | A pawl locks the flex spool after a squeeze | Sustained = stall force (3–4× continuous) at 0 W | M (per spool) | Release needs re-tension; power-loss behaviour must be chosen | Later, if holding tests show the need |
| 16 | **Joint-angle sensors (magnet + Hall)** | Diametric magnet on the MCP/PIP axle, 12-bit sensor on the parent | Cancels a sag of 4–11° at 10–25 N; tendon becomes a force sensor (0.05 N resolution) | €1–2 per joint + wiring | Wiring through the palm | **Do** with #6 |
| 17 | Tension-sensing idler | The strand deflects over an idler on a strain-gauge flexure (Shadow) | ±0.2 N per strand | €3 per channel | Calibration drift | Later (index + thumb) |
| 18 | Magnetic / pop-out abduction | The finger is held to its abduction seat by magnets or a detent | Protects lugs and line from side hits (~20 N breakaway) | S | Stray pop-outs in power grasp | V2.1 |
| 19 | **Metal drum sleeves + capstan-clamp anchors** | Brass/steel groove floor; 2–3 wraps on a steel post + M2 clamp instead of knots | Groove 31–40 MPa (PETG creeps above ~15 MPa); anchor ~90 % of line strength vs 55 % for a knot | S | Small turned/cut parts | **Do** at HLS loads |
| 20 | Hydraulic transmission | Syringe → hose → mini cylinder | Friction no longer grows with bends | L | Leaks, seal friction, mess | No |
| 21 | BLDC + capstan (1X-style) test joint | Gimbal motor + 10:1 cable capstan + FOC | Backdrivable, true torque; 45 N class | €40 per joint, Ø28–40 motor | Electronics per joint | One test joint (index MCP) |
| 22 | Silicone pads | μ 0.3 → 1.0 | 3× less squeeze needed for the same object | S | Wear | **Do** (other agents) |

---

## 3. Deep dives (narrow)

### 3.1 Bare-strand routing with a wrist eye plate (ideas 1 + 2)

**Why the sheath must go (numbers from §1 and §5 of the script).**
- **V2 friction:** wrap per flex strand at a tip pinch, wrist straight, is 64° (middle MCP) to 289° (thumb ip).
  That gives η = 0.60–0.89 at μ 0.1, and 0.47–0.84 at μ 0.15. The forearm fan adds up to 147° by itself
  (index abd).
- **V2 stiffness:** the PTFE sheath (A = 2.36 mm², E ≈ 0.5 GPa, 205 mm) is 5.8 N/mm in series with the braid,
  so the loop is 5.0 N/mm. MCP sag is 51° at 5 N tip and 100° at 10 N. A PEEK tube gets 19.5 N/mm (26° at 10 N).
  **No sheath material makes the 230 mm loop as stiff as the bare line: 60 N/mm with 80 lb braid.**

**The proposed path (index MCP flexor as the example):**

```
 spool (forearm, z -140)  --straight, 15 deg at the forearm guide-->  eye plate (wrist centre, z 0)
   --straight in the palm-->  knuckle-entry idler (MR63, at the abd axis, ~10 deg)  -->  MCP drum (r 7)
 sliding: 25 deg on polished steel + 2 idlers  ->  eta = 0.94   (V2: 223 deg in PTFE -> 0.68)
 pip strands: same, plus the MCP crossing on an on-axis idler -> eta 0.93 (V2 0.65)
 wrist bent 50 deg: +50 deg over the eye -> x 0.92 (V2 sheath: the same x 0.92)
```

**The eye plate (side view of the cardan middle ring):**

```
     palm guide comb (z +30)   . . . . . . .      each strand straight from its comb hole to its eye
                                \  |  /
  flex axis (X) ====[ring]====( eye plate )====[ring]====   plate lies IN the plane of both axes (z 0)
  dev axis (Y)  goes through the same point  \  |  /
     forearm guide comb (z -30)  . . . . . . .
```

- Strand i passes eye (x_i, y_i). To first order its length changes by ΔL_i = x_i·θ_dev + y_i·θ_flex
  (script §7: worst 4.26 mm over flex ±50°, dev −30…+20°).
- **Pairing rule:** the flex and ext strands of one joint use point-symmetric eyes, (x, y) and (−x, −y). Their
  changes are then equal and opposite. That is exactly what one spool turning produces, so the servo cancels
  it by turning ΔL_i ÷ r_spool. The loop length stays constant to 0.036 mm (2.2 N of pretension change with
  80 lb braid).
- **Eye assignment:** put the joints with the least servo travel to spare (SCS0009 MCP/PIP, see §3.2) on the
  inner ring of eyes (offset ≤ 2 mm → ≤ 1.8 mm, ≤ 20° of servo). Put the abductions and HLS joints outside.
- **Firmware:** each tick, spool target = joint target × G + (x_i·θ_dev + y_i·θ_flex) ÷ r_s, with the wrist
  angles read from the wrist servos. The coefficients come from CAD and are checked by a calibration sweep.
  The same table also handles the abduction crossing.

**What fails first, and how this avoids it:**
- **Eye wear when the wrist moves under load.** The strand slides over each eye by up to 4 mm per wrist sweep.
  Fixes: 18G stainless tube eyes with flared, polished mouths (rim r ≥ 0.4), or ceramic fishing-rod guide
  inserts. Test: 10 000 wrist sweeps at 20 N.
- **Strands rubbing each other.** 32 straight lines converge on an 8 × 8 mm grid. The CAD must check that the
  minimum skew-line distance is ≥ 0.5 mm.
- **Loss of the wrist angle (encoder fault).** The fingers then move by up to 35° when the wrist moves. That is
  safe, but the firmware must stop the wrist if its angle is unknown.

**Bench test that proves or kills it (1 day):**
1. Build a printed cardan mock-up with the eye plate, a 230 mm bare path, 2 steel pins and 1 idler.
2. Hang 5 / 20 / 45 N on one end and pull the other with a luggage scale. Pass: η ≥ 0.85 (μ ≤ 0.12).
3. Tie a loop and set 5 N pretension (by pluck pitch). Sweep the wrist ±50° / −30…+20°. Pass: pretension stays
   within 5 ± 3 N, and the differential length matches x·θ to ±0.3 mm.
4. Kill criterion: the pretension swings by more than 10 N, or the strands fray in fewer than 10 k sweeps.

### 3.2 Ratio per joint: small MCP spool + 2:1 reeved PIP (ideas 4 + 5)

**Which joint limits?** (script §4c). Torque per newton of tip force, with the force normal to the pad:

| Pose (MCP / PIP) | MCP lever | PIP drive lever (PIP + 0.75 DIP) | Limits |
|---|---|---|---|
| 0 / 0 (straight push) | 81.0 mm | 50.2 mm | MCP |
| 15 / 20 | 72.6 | 49.4 | MCP |
| **30 / 45 (tip pinch)** | **43.1** | **46.2** | **PIP** |
| 45 / 60 | 21.1 | 43.2 | PIP |
| 60 / 80 (curled) | 5.2 | 38.2 | PIP |

In a pinch both joints are nearly equal, so upgrading only one gains almost nothing. In the script, reeving the
PIP alone takes a pinch from 6.4 to 6.3 N, because the MCP then limits. Both must rise together.

**The loop rule (why there is no free lunch).** In an antagonistic loop, the flex and ext strands must move by
equal and opposite amounts. So r_flex,spool ÷ arm_flex = r_ext,spool ÷ arm_ext = 1/G. The ratio G is therefore
capped by the extensor: G ≤ arm_ext ÷ r_min, where r_min is the smallest spool you can put on the spline.
Reeving only the flexor, with a stepped spool, gives no force gain (G = 1.68 vs 1.89 for the plain small spool
on HLS). **Either reeve both strands, or use a smaller hub.** The reeved PIP below doubles both strands in one
shared 2-groove track.

**Reeved PIP track (section through the drum, looking along the joint axis):**

```
        turnaround pins (radial, on the MIDDLE phalanx, at the top of the drum)
              flex pin  o   o  ext pin            groove A: flex-in (palm half) + ext-in (back half)
           ___________|___|___________            groove B: flex-return + ext-return
          /  groove A  \ | /  groove A \          both segments of a strand wrap r 7 -> arm 7 + 0.78 x 7
   palm  |   groove B   |o|   groove B  |  back
          \____________/   \___________/          flex-return anchors on the PROXIMAL phalanx (palm side),
        to flex anchor         to ext anchor      ext-return on its back side: capstan post + M2 clamp
```

- **Width:** 1.0 (groove A) + 3.0 (Ø3 pin, which sets the groove spacing) + 1.0 (groove B) + 2 × 1.0 (flanges)
  = 7.0 mm. That fits the ~8.8 mm between the PIP bearings.
- **The MCP cannot do the same.** The pip strands cross its middle, which needs ~4 mm more, so 11 mm in 8.8.
  It therefore gets the small metal-hub spool instead (open CAD question: move the MCP bearings inboard and
  cross the pip strands outboard, on the axis line).
- **Travel:** the reeved PIP strand moves 0.218 mm per joint degree (vs 0.122).

**Balanced designs** (script §4d):

| Design | MCP strand | PIP strand | Pinch stall | Pinch cont. | Straight | G MCP/PIP | Joint °/s |
|---|---|---|---|---|---|---|---|
| V2 as built (SCS0009, r 5/5, PTFE) | 45 N | 45 N | 4.5 N | 1.5 N | 2.6 N | 1.4/1.4 | 429 |
| **B0 SCS0009, MCP r 3.0, PIP r 5.5 reeved, bare routing** | 75 | 41 | **10.3** | 3.4 | 6.1 | 2.33/2.55 | 236 |
| **B1 HLS3606M, MCP r 3.5, PIP r 5.0 reeved** | 169 | 118 | **25.7** | 6.5 | 13.7 | 2.00/2.80 | 238 |
| B2 STS3215 12 V, r 7/7 reeved, 250 N cap | 250 | 250 | 38.1 | 19.6 | 20.3 | 1.00/2.00 | 135 |
| B3 as B2, 300 N cap (200 lb line) | 300 | 300 | **45.7** | 19.6 | 24.3 | 1.00/2.00 | 135 |

Notes on the table:
- SCS0009 travel (B0): MCP 100° × 2.33 = 233°, PIP 100° × 2.55 = 255°, plus ≤ 45° of wrist compensation,
  inside 300°.
- B0 at stall is a peak. The SCS0009 drops torque after 2 s above 80 %, so call it 8 N usable peak.
- B1's MCP hub at r 3.5 needs a metal sleeve on the 25T / 4.95 mm spline. On the stock aluminium horn hub
  (r ≈ 4.5, verify) the MCP force drops ~22 %, to ≈ 20 N pinch.

**What the printed parts must carry** (script §6, plus hand calculations):

| Load class | Strand T | Line (SF on knot / on capstan clamp) | Groove pressure T ÷ (r·1.2d) | Joint bearing load | DIP bar force at the class's pinch | Verdict for the part |
|---|---|---|---|---|---|---|
| SCS r 5 (V2) | 45 N | 50 lb (2.7 / 4.4) | 15 MPa | 60 N | 13 N | PETG OK |
| B0 MCP r 3.0 | 75 N | 80 lb (2.8 / 4.5) | 19 MPa | 92 N | 34 N (10 N tip) | PETG at the limit: **brass groove sleeve at the MCP** |
| B1 MCP | 169 N | 150 lb 0.60 mm (2.2 / 3.5) | 34 MPa | ~200 N (2 bearings) | 86 N (25.7 N) | **Metal groove, capstan-clamp anchors, 684ZZ or check MR84 C0** |
| B1 PIP reeved | 118 N per segment, 236 N on the drum | 80 lb (1.7 / 2.7) | 31 MPa | ~260 N | – | Ø3 hardened turnaround pin pressed 4.5 mm (17 MPa in PETG: use **PA-CF** for the middle phalanx or a brass insert) |
| B3 | 300 N | 200 lb 0.75 mm (1.6 / 2.7) | 48 MPa | ~330 N | 150 N (45 N) | Metal drums, 684/694 bearings, steel bar pins Ø2 in brass bushes, PA-CF phalanges |

Other implications:
- **Phalanx bending is not the problem.** An 18 × 16 hollow section with 1.6 mm walls sees 5.6 MPa at 45 N.
  The cut-outs around the hubs (×3–4 local stress) are, so keep ≥ 2.5 mm webs there above 20 N.
- **The spool shaft side load equals the strand load** (both strands leave on one side): 173 N in B1.
  The V2 MR63 spool-rail stubs (C0 ≈ 0.1 kN, verify) are too small, so use **684ZZ outboard bearings**.

**Bench test:** print one PIP joint with the reeved track.
1. Hang 0.5–4 kg on a 40 mm lever and pull the flex strand with a luggage scale. Pass: torque ÷ (T × 7 mm)
   ≥ 1.7.
2. Run 10 000 cycles at 40 N. Pass: no visible fraying at the pin (braid D/d = 6.5).
3. Kill criterion: ratio < 1.5, or fraying in fewer than 10 k cycles. Then try an MR63 bearing as the
   turnaround (needs groove spacing 6 mm, width 10 mm: only fits a wider finger).

### 3.3 Stiffness: once the hand is strong, the tendon becomes the soft part (idea 16)

Joint stiffness K = r_arm² × k_strand. It does not depend on the spool ratio, so a stronger servo does not make
the finger stiffer. Script §5, MCP sag (one strand, the extensor goes slack under load):

| Loop | k (N/mm) | K_MCP (N·m/rad) | Sag at 5 / 10 / 25 N tip |
|---|---|---|---|
| V2 PTFE sheath | 5.0 | 0.24 | 51 / 101 / 253° (cannot hold) |
| V2 PEEK sheath | 19.5 | 0.96 | 13 / 26 / 65° |
| Bare 80 lb, 230 mm | 60 | 2.96 | 4.2 / 8.4 / 21° |
| Bare 150 lb, 230 mm | 113 | 5.54 | 2.2 / 4.5 / 11° |
| Bare 200 lb, 230 mm | 151 | 7.39 | 1.7 / 3.3 / 8.4° |
| Reeved joint, 80 lb | 60 | 9.37 | 1.3 / 2.6 / 6.6° |

What this means:
1. **Bare strands (#1) are not optional.** A sheathed drive cannot hold 10 N at all.
2. Use the thickest line that bends safely over the spool. 150 lb on r 3.5 gives D/d ≈ 12.
3. **Put a joint-angle sensor at each strong joint.** Above ~10 N the 5–10° sag is larger than any useful
   position tolerance. Then close the loop on the joint angle in the ESP32. The sag becomes free force sensing:
   tip force ≈ K × (servo angle ÷ G − joint angle) ÷ lever, i.e. 1.2 N per degree with 80 lb braid. A 12-bit
   sensor (0.09°) resolves about 0.05 N.
4. Hardware: a Ø4 × 2 mm diametric magnet in the MCP stub end, and an AS5600 or TMAG5273 on a 10 × 10 mm board
   on the yoke cheek. €1–2 per joint plus a 4-wire I²C bus through the palm (sensing plan stage 2 in
   `research/references/hands/sensing.md`).

### 3.4 The 45 N question and the actuator choice (ideas 6, 7, 21)

- **Cost per N·m of stall torque:** SCS0009 €40, HLS3606M €47, STS3215 **€5.4**. The cheap torque is in the
  big arm servo, not in the micro servos.
- **For 45 N at the index tip**, the MCP needs 45 × 43 mm ≈ 1.9 N·m in a pinch, or 3.6 N·m pushing straight.
  - At a 7 mm drum and η 0.94, that is **~300 N** of strand.
  - HLS3606M (0.59 N·m) would need r_spool ≈ 2 mm. That is impossible on a 4.95 mm spline, so the HLS tops out
    at ≈ 26 N with a plain MCP.
  - STS3215 (2.94 N·m) reaches 300 N at r ≈ 10 mm, so it has margin. B3 = 45.7 N.
- **Costs of B3 (STS3215):** 11 × 55 g = 605 g of servos (vs 227 g for HLS) and 433 cm³ of forearm. The V2
  forearm envelope is 1 330 cm³, so it can fit. It also needs a 12 V supply branch, 200 lb line, metal
  drums and 684 bearings (§3.2 table). It is fine for the fixed bimanual station (the arm carries it).
  Check the OpenArm J5 payload before choosing it.
- **The 1X-style route** (BLDC + capstan, backdrivable, current = force) is the better 45 N hand. It costs
  ~€40 per joint and its own electronics. Build **one** test joint (index MCP) before deciding.

**Recommendation:** V2 = B1 (HLS3606M on the 11 strong joints), with forearm slots sized for an STS3215 on the
4 MCPs as the later 45 N swap. That swap is only a spool change, because the strand paths stay the same.

---

## 4. What to build as CAD in the next phase

All in the V2 frame (origin = wrist centre, +Z to the fingers, −Y palm side, +X thumb side).

1. **Wrist eye plate** `eye_plate` (on the cardan middle ring, centre at the origin, normal +Z):
   - Disc Ø16 × 2.5 mm, PETG-CF (the strands touch only steel). Held to the ring by 4 spokes, 2 × 3 mm.
   - 32 holes Ø1.30 on a 1.6 mm square grid (6 × 6, minus the 4 corners), each with an 18G stainless tube eye
     (OD 1.27, ID 0.84) 2.5 mm long, glued, both mouths flared and polished to r ≥ 0.4.
   - Assign the eyes by the pairing rule (flex/ext point-symmetric; SCS0009 MCP/PIP pairs within 2 mm of the
     centre). Write a table `eye_xy[strand]` that the firmware compensation reads.
2. **Guide combs** at z = −30 (`forearm_comb`) and z = +30 (`palm_comb`):
   - 2 mm plates with the same 32-hole pattern, scaled so that every strand runs straight from its spool
     tangent point to its eye. Steel tube eyes again.
   - CAD check: the minimum skew distance between strands is ≥ 0.5 mm, in all wrist poses on the 9-point grid.
3. **Forearm spool layout:**
   - Put each spool's tangent point on a straight line to its forearm comb hole. Target ≤ 15° of deflection
     at the comb, which means spools within ±30 mm of the axis at z ≤ −120.
   - Outboard **684ZZ** (4 × 9 × 4) for every spool.
   - Remove the fan, comb bay and barrel adjusters. Tension with the split spool plus the capstan-clamp anchor.
4. **Palm routing:**
   - Open troughs (2 mm wide, lid screwed on), straight from the palm comb to each knuckle.
   - At each knuckle entry, an **idler block** with 2 × MR63ZZ (3 × 6 × 2.5) on a Ø3 dowel at the abduction
     axis: one for the flex strand, one for the ext strand, both crossing the abduction axis on it.
   - Replace the MCP eyelets for the pip strands with a Ø3 polished dowel or a Ø4 roller whose surface is at the
     axis (review #5).
5. **Reeved PIP track** (`pip_track_2to1`), on the middle phalanx:
   - Drum r 7.0. Two grooves 1.0 wide × 0.7 deep, centres at x = ±2.0.
   - A Ø3 × 8 hardened dowel (ISO 8734), radial, pressed 4.5 mm into the hub, at each strand's anchor angle,
     with the flex pin 10° palm side of the top and the ext pin 10° on the back side.
   - Returns anchor on the proximal phalanx: a Ø3 dowel post (2 wraps) + an M2 × 5 clamp screw with washers
     (prior-art #1). One anchor on the palm side, one on the back.
   - Width ≤ 7.0 mm between the bearings. For HLS loads, print the middle phalanx in PA-CF, or use a brass
     insert under the pins.
6. **MCP drive:**
   - Plain r 7 drum with a **brass groove sleeve** (brass tube 14 OD × 0.5 wall, pressed on the hub, grooves cut
     by printed flanges).
   - Servo spool: a metal hub at r 3.0 (SCS0009) or r 3.5 (HLS), made from a cut-down metal horn or a turned
     aluminium sleeve. Capstan anchor on the spool.
7. **Joint-angle sensor pockets** at MCP and PIP:
   - Ø4.1 × 2.2 magnet pocket in the outboard axle end.
   - A 10 × 10 × 1.6 PCB pocket on the parent's cheek, 1.0 mm air gap.
   - A 4-wire channel (Ø2.5) through the palm to the wrist.
8. **Forearm slots** sized for **STS3215** on the 4 MCP servos (45.2 × 24.7 × 35 envelope), so the 45 N
   upgrade is a swap.

---

## 5. Sources

- In-repo: `CLAUDE.md`; `research/references/hands/README.md`, `tendon-mechanics.md` (capstan, EA, μ, pinch
  norms), `actuators-and-control.md` (servo data: SCS0009, HLS3606M, STS3215, STS3032; Aero Hand Open, 1X NEO,
  ORCA, MM-Hand); `cloud-claude/2026-11-09-v2-hand-cad/` README, `reports/01-finger.md`, `02-thumb.md`,
  `03-wrist-forearm.md`, `05-review.md`, `cad/params.py`, `out/assembly/routes.json`, `fan_routes.json`.
- This study: `reports/06-prior-art.md` (clamp-bolt anchor, spring-tube sheaths, GT2 wrist),
  `reports/03-joints-structure.md` (MR84 C0 estimate).
- Standard relations: capstan e^(−μθ), Hooke k = EA/L, joint stiffness K = r²k, virtual work for the coupled DIP
  (τ_PIP,drive = τ_PIP + 0.75 τ_DIP), Lewis-style bearing-pressure checks.
- **Verify before buying:** STS3215 12 V continuous torque and spline size, HLS3606M horn hub size, MR84ZZ /
  MR63ZZ / 684ZZ static ratings, braid diameters and break loads for 150 / 200 lb, OpenArm J5 payload.
