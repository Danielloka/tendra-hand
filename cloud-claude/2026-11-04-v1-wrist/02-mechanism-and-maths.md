# V1 wrist: mechanism and maths

Date: 2026-11-04. Scope: a 2-DOF wrist for Tendra Hand V1: `wrist_flex` ("bend", + toward the palm, −60…+70°) and `wrist_dev` ("wave", + toward the thumb, −30…+20°). All 16 finger servos stay in the forearm, so all **32 strands** (16 antagonistic loops, flex + ext on one 5 mm spool) must cross the wrist.

Numbers come from `wrist_coupling.py` in this folder. It reads `hardware/robot_description/v1_export/tendon_routes.json` and the MuJoCo wrist model. Run it from the repo root:

```
uv run python cloud-claude/2026-11-04-v1-wrist/wrist_coupling.py
```

Assumptions used everywhere: wrist centre C = (x −33.5, y 25.5) mm, the same as `sim/convert_v1_wrist.py`. 50 lb UHMWPE braid with EA ≈ 5 500 N (from `research/references/hands/tendon-mechanics.md`). Estimated strand length from spool to drum is 123…213 mm, so the line's stiffness is k = EA/L ≈ 26…45 N/mm. PTFE tube 1 × 2 mm, E ≈ 0.5 GPa. The SCS0009 stalls at 0.226 N·m, which is **45 N of line pull** on a 5 mm spool. Values marked **(verify)** come from memory or catalogues and should be checked before buying anything.

---

## Short answer

1. **Bare strands through the wrist will not work.** At today's positions (x −69…5, y 15…36), a bend changes strand lengths by up to ±20 mm. Each loop's flex and ext strands sit 10 mm apart in x, so waving the wrist (`wrist_dev`) moves every finger joint by up to **25°**. Bending it (`wrist_flex`) changes both strands of a loop the same way. A single spool can't take that up, so the strand would need **hundreds of newtons** of extra tension, which is impossible: the line goes slack or the servo stalls. Even bundled at the centre, the errors are still 10–20° and ±100 N or more.
2. **Bowden sheaths (the owner's choice) are the right idea.** With both ends of each sheath fixed, the length of line inside it doesn't change, whatever shape the sheath takes, so the coupling is gone. Three things are left over, and all three need design work:
   - **Friction.** At full bend (73°), the wrist alone keeps e^(−μθ) = **0.88** of the force at μ = 0.1, or **0.73** at μ = 0.25.
   - **Slack from bore clearance.** In a bend, the line moves to the inner wall of the tube. With a 1.0 mm bore that is ≈ 0.4 mm, about **12 N** of lost pretension. A 0.6 mm liner cuts it to ≈ 4 N.
   - **Sheath squash and stiffness.** PTFE is soft along its length: 2.3 mm of squash at stall over a 60 mm free span. The 32 tubes also act as a spring of **0.25–0.5 N·m** on the wrist.
3. **Size.** 32 tubes of OD 2 mm need a hole of **≥ 15 mm** if round. An oval of **≈ 22 × 14 mm** (x × y) is better, and it leaves room for a sensor cable.
4. **Recommended concept:**
   - a **hollow cardan** (a ring cross with an oval bore) in a **30 mm wrist gap**;
   - the tubes run as a **"crossed hourglass"**: straight lines through the centre, with a strand from one side of the palm going to a spool on the other side of the forearm;
   - the wrist is moved by **two forearm servos through push rods (2-RSS)**;
   - those servos are **2 × STS3215 (12 V), each with a 2:1 belt or gear**.

   That gives about 7 N·m at stall and about 2 N·m continuous at the wrist. The need is ≈ 0.2 N·m for the hand alone, 1.2 N·m for 0.5 kg at the fingertips, 2.2 N·m for 1 kg at the fingertips, plus ≈ 0.4 N·m for the tube spring. SCS0009 and SCS2332 are too weak.

---

## 1. Coupling maths

### 1.1 What "coupling" means here

A strand works like a fixed-length string between the spool and the joint drum. If the wrist makes the *path* longer, the string has to come from somewhere: it pulls the finger joint, or it stretches. That unwanted link between wrist and finger is the **coupling**.

### 1.2 One bare strand crossing one axis

Model: the strand goes through an eyelet in the forearm at height −h below the axis and an eyelet in the palm at +h. It sits at offset **b** from the axis, measured perpendicular to the axis in the bending plane. The palm turns by θ. From the geometry, the distance between the eyelets is exactly

> **L(θ) = 2·(h·cos(θ/2) + b·sin(θ/2))**, so **ΔL = 2b·sin(θ/2) − 2h·(1 − cos(θ/2)) ≈ b·θ** (small angles)

- The **b·θ** term is the moment-arm effect. A strand on the far side from the bend gets longer; one on the near side gets shorter.
- The **h** term makes even a centred strand a little shorter (−3.6 mm at 70° with h = 10 mm), because the eyelets come closer. A rounded guide right at the axis removes this term. The script calls that case "centre guide".
- Signs in the model: flexion turns the palm about +X toward −Y, so strands on the back (y > 25.5) get longer. Deviation turns it about +Y toward +X, so strands on the thumb side get shorter.

| offset b (mm) | flex −60° | flex +70° | dev −30° | dev +20° |
|---|---|---|---|---|
| 0 | −2.7 | −3.6 | −0.7 | −0.3 |
| 5 | −7.7 | +2.1 | +1.9 | −2.0 |
| 10 | −12.7 | +7.9 | +4.5 | −3.8 |
| 20 | −17.3 | +19.3 | +9.7 | −7.2 |
| 35 | −2.3 | +36.5 | +17.4 | −12.5 |

(ΔL in mm, h = 10 mm. Today the strands sit at x offsets −35.5…+38.5 mm and y offsets −10.5…+10.5 mm from C.)

### 1.3 Antagonistic loop: difference moves the finger, sum changes the tension

One loop has a flex strand and an ext strand on one spool and one drum (radius r_d). Hold the servo still and let the wrist change the path lengths by ΔL_f and ΔL_e.

- If the joint turns by q, the flex strand needs r_d·q less length and the ext strand needs r_d·q more. So the extra stretch is (ΔL_f − r_d q) in the flex strand and (ΔL_e + r_d q) in the ext strand.
- With no outside load, the joint settles where the two tensions are equal (same stiffness k). So:

> **Finger motion: q = (ΔL_f − ΔL_e) / (2·r_d)**  (the *difference*)
> **Tension change: ΔT = k·(ΔL_f + ΔL_e) / 2**  (the *sum*)

That is the key idea:

- **The servo can cancel the difference**: turn it by q·`servo_per_joint`. A feed-forward from the measured wrist angle could do this in software.
- **It can never cancel the sum**, because one spool pays out one strand exactly as fast as it winds in the other. A sum change means slack (the joint gets loose) or an overload (stall, or a broken line).

Results over the full wrist range (worst corner). Drum radii come from the json: 6 mm, 7 mm for `mcp_flex`, 7.5 mm for `thumb_cmc_rot`.

| Bare layout | worst joint error | servo correction | tension change |
|---|---|---|---|
| **Today's spread positions** (flex/ext 10 mm apart in x, same y) | **24.7°** (from dev) | 29.7° | **−772 / +1026 N** (from flex) |
| Round bundle at C (r ≤ 6.1 mm), flex and ext side by side, centre guide | 17° | 24° | −280 / +244 N |
| Round bundle, ext = mirror of flex through C, centre guide | 61° (but can be cancelled) | 74° | ≈ 0 N |

How to read it:

- Today's layout puts each pair side by side in x at the same y. Flexion then changes both strands the same way, which shows up as tension (impossible). Deviation changes them oppositely, which shows up as finger motion (25°).
- Mirroring each pair through the centre makes the sum vanish, so the tension stays constant. The cost is big finger motions that the servo must cancel, using up to 74° of servo travel. Bare strands also put a large moment on the wrist: the grasp tension times the offset. In a fist, 8 flex strands × 45 N at 5–10 mm is about **3 N·m**, more than gravity.

**So bare crossing is out**, unless every strand crosses within about ±0.5 mm of each axis line (section 2, concept b).

### 1.4 Why a Bowden sheath removes the coupling

A Bowden cable (a bicycle brake cable) is a line inside a tube whose two ends are fixed. The length of line *inside* the tube equals the tube's length, whatever shape the tube takes. If the tube can't change length and its ends don't move, the line between the ends can't change length either, so **wrist motion can't reach the finger**.

The tube takes the reaction: it is squeezed with the same force the line is pulled with. That is also why the sim's "all strands through one fixed point `wrist_centre`" is the ideal case.

What is left over:

**(a) Capstan friction.** A line sliding around a total bend θ keeps T_out = T_in·e^(−μθ) (the capstan or Euler–Eytelwein law). Only the *total* turning counts, not the radius. The sheath across the wrist turns by the angle between the forearm and palm axes, **arccos(cos θ_f · cos θ_d)**, which is **72.8°** at flex 70° + dev 20°.

| μ | force kept, wrist only | wrist + palm route, median strand (19°) | wrist + palm route, worst (thumb, 120°) |
|---|---|---|---|
| 0.05 (braid in PTFE, best) | 0.94 | 0.92 | 0.85 |
| 0.10 | 0.88 | 0.85 | 0.71 |
| 0.15 | 0.83 | 0.79 | 0.60 |
| 0.25 (worn or dirty) | 0.73 | 0.67 | 0.43 |

**(b) Bore clearance.** In a straight tube the line sits somewhere in the bore. In a bend, under tension, it moves to the *inner* wall, so its path gets shorter by c·θ, where c = (ID − line)/2.

- 1.0 mm ID with a 0.36 mm line: c = 0.32 mm, so −0.41 mm at full bend.
- Both strands of a loop shorten by the same amount, so this is a **sum** change: about **−12 N** of pretension (braid, k ≈ 29 N/mm). That is more than the planned 3–5 N pretension, so **the loop goes slack at big wrist angles**.
- Fixes, any of these:
  - a tighter liner (0.6 mm ID: −0.15 mm, ≈ 4 N);
  - a soft spring on the ext strand. It takes up the slack, but turns it into ≈ 0.4/6 rad ≈ 4° of finger motion toward extension, which the servo can cancel by feed-forward;
  - or both.

**(c) Squash and stretch of the sheath.** The free part of a tube carries the full line tension as compression: δ = T·L/(E·A). For PTFE 1 × 2, EA ≈ 1 180 N.

| free sheath length | squash at 5 N | at stall (45 N) | = joint angle at a 6 mm drum |
|---|---|---|---|
| 30 mm | 0.13 mm | 1.15 mm | 11° |
| 60 mm | 0.25 mm | 2.3 mm | 22° |
| 100 mm | 0.42 mm | 3.8 mm | 37° |

- This is **compliance**, not coupling: it doesn't depend on the wrist angle. But it is big. Under load the finger gives way, and the servo's encoder can't see it.
- Inside a printed channel in the palm the load goes into the channel walls, so only the **free span across the wrist counts**. Keep that span short, or use a **compressionless sheath** there: a close-wound stainless steel coil with a PTFE liner, as in brake housing, about 2 mm OD **(verify sizes)**. PEEK tube is 7× stiffer along its length, but also 7× stiffer in bending, so the wrist spring gets worse.
- Buckling: an *empty* PTFE tube 60 mm long buckles at only ≈ 1 N. A tube with a **tensioned line inside** doesn't buckle like that. The line pushes on the inside of every curve exactly as hard as the squashed tube wants to bow out (a "follower load"), which is why bicycle cables don't buckle. The ends still need to be fixed.

**(d) Bend radius and spring moment.** If the bend is spread evenly over a length L_b, then R = L_b/θ and each tube pushes back with M = EI/R. For PTFE 1 × 2, EI ≈ 368 N·mm².

| bent length | R at 73° | moment per tube | 32 tubes |
|---|---|---|---|
| 20 mm (today's sim gap) | 16 mm (at the 15 mm limit) | 23 N·mm | **0.75 N·m** |
| 30 mm | 24 mm | 16 N·mm | 0.50 N·m |
| 40 mm | 31 mm | 12 N·mm | 0.37 N·m |
| 60 mm | 47 mm | 8 N·mm | 0.25 N·m |

So the wrist gap should be **≥ 30 mm**: a softer spring, and a radius well above the 15 mm minimum.

**(e) Length mismatch in a bundle: the hidden problem.** A sheath with fixed ends still has to *fit* between its ends. The geometric path through the wrist changes by the same b·θ as a bare strand, where b is the tube's place in the bundle. The tube can't stretch, so it has to change shape (bow out) instead. If a tube is pulled straight and stretched, the stretch *is* coupling.

| layout at the wrist | flex only | dev only | both, worst | half-sine bow needed over 60 mm |
|---|---|---|---|---|
| today's spread (no waist) | 12 mm | 20 mm | 29 mm | 27 mm (impossible) |
| round bundle 32 × 2.1 mm at C | 6.3 | 3.2 | 6.6 | 13 mm |
| oval grid 8 (x) × 4 (y) at C | 3.6 | 3.8 | 6.7 | 13 mm |
| offset axes, 2 rows of tubes on each axis line | 1.2 | 0.5 | 1.7 | 6.5 mm |

The bow comes from the extra length of a half-sine bow, ΔL = π²A²/(4L).

This is a first-order, worst-tube estimate. Loose tubes can also swap places in the waist, which helps. Rules:

- route every tube **through a waist at the wrist centre**, never spread out;
- give the free span **built-in slack** (a gentle pre-bow), so a tube is never pulled straight;
- **check with a printed mock-up** (section 4).

### 1.5 Routing: the "crossed hourglass"

Bringing 32 strands from their palm places to a waist at C and back out to spools in the same places costs a lot of turning with S-curves. The script estimates up to 300° for the thumb strands, which come down very low and far to the side.

The better trick is to run each strand as a **straight line through the centre**:

- in the palm, a straight channel from the finger-entry bore aimed at C;
- below C, a straight line on to the **spool at the point-reflected place**. A strand from the thumb/back side goes to a spool on the little-finger/front side.

The current spool grid is almost point-symmetric about the forearm axis (y 15↔36, 20↔31; x −69…5 around ≈ −32), so this mostly means re-numbering which strand goes to which spool. Straight lines that pass near one point form a narrow **waist**, like the lines of a hyperboloid, so the tubes stay straight in the straight pose.

| wrist gap / forearm made longer by | palm bend at entry, median / worst | kink at C, median / worst | spool fleet angle, worst | force kept, worst strand at max wrist, μ 0.1 / 0.25 |
|---|---|---|---|---|
| 20 mm / 0 | 15° / 66° | 7° / 44° | 14° | 0.73 / 0.45 |
| 30 mm / 0 | 14° / 60° | 7° / 39° | 12° | 0.74 / 0.48 |
| 30 mm / +40 mm | 14° / 60° | 7° / 45° | 7° | 0.73 / 0.46 |

How to read it:

- The palm bend replaces today's palm S-curves, which turn 3…120°, so the median strand gets *better*.
- The worst strands are the thumb's (the 60° palm bend). They come out of the thumb bay at z −12…−24, so they need extra care, as the thumb already does.
- **Fleet angle** is how far the strand arrives out of the spool's plane. 12° is too much for a flanged spool (aim for ≤ 4–5°). Either make the forearm ≈ 40–60 mm longer (more human-like anyway), or add a short guide tube at each spool.

---

## 2. Bundle size and wrist mechanism

### 2.1 How big must the hole be?

- 32 circles of OD 2 mm packed as tightly as possible fit in a circle of **D ≈ 12.8 mm** (best known packing R/r ≈ 6.42 for n = 32, Packomania, **(verify)**).
- Real tubes need room to slide. At about 60 % fill that is **D ≈ 14.6 mm**.
- An 8 × 4 grid at 2.1 mm pitch is 16.8 × 8.4 mm.
- Add ≈ 4 mm for a future palm sensor cable (power plus a UART bus to a palm MCU) and 1–2 mm of wall clearance. **Bore ≈ 22 × 14 mm oval (x × y), or ≈ 18 mm round.**

Comparison:

- a human wrist is ≈ 55 × 40 mm;
- the palm is 91 × 27.5 mm (x −79…12, y 12.5…40);
- the forearm servo box is 91 × 85 mm.

So the wrist can be about 40 mm deep and 50–60 mm wide, like a human's. The palm's small depth (27.5 mm) is why the bore should be oval and long in x.

### 2.2 Concepts

| | Concept | Hollow centre | Coupling / tube fit | Range 130° flex | Build | Verdict |
|---|---|---|---|---|---|---|
| **a** | **Hollow cardan**: an oval ring cross; x-pins to a forearm fork, y-pins to a palm fork; axes cross at C | Yes, the ring bore (22 × 14 mm) | Sheaths: none. Fit: 4–7 mm per tube (slack needed) | Fork arms limit it. 70° flex needs the forearm fork base ≥ 15 mm below C and relief cuts (check in CAD) | Simple: 4 small bearings (e.g. 3 × 7 × 3 mm, **(verify)**) and printed or metal ring | **Recommended** (with d) |
| **b** | **Two offset single axes** (flex lower, dev 10–15 mm higher), strands in rows **exactly on each axis line** (Tesla-style) | Each axis line is free in the middle | Bare: ≤ 0.3 mm (rows ±0.2–0.5 mm). Sheaths: ≤ 1.7 mm | Easier: no crossing pins | The 32 strands must go from a row along x to rows along y between the axes: up to ≈ 40° of extra turning, and 1–2 rows of 16–32 = 16–34 mm along y | Fallback, if the bundle in (a) fights the tubes |
| **c** | **Rolling-contact wrist** (two curved surfaces rolling on each other, held by crossed straps; one stage per axis) | Yes | Same b·θ as a hinge for strands off the centreline (checked: L ≈ 2R − b·θ). Only a line through both centres of curvature keeps its length | Good, compact, human-looking | Straps, preload, two stages; hard to make stiff sideways | Not better for the tendons; more parts |
| **d** | **2-RSS parallel drive** (two forearm servos, cranks, ball-joint push rods to the palm) on a **passive hollow U-joint** (= a) | Yes, from (a) | As (a) | Rods need a ≈ 45–70 mm stroke; crank r ≈ 25–35 mm | Both motors fixed in the forearm, no cable across the wrist, backdrivable | **Recommended actuation** |

**Recommendation: (a) + (d).**

- A hollow oval cardan in a **30 mm gap** (C 15 mm below the palm bottom), with bore ≈ **22 × 14 mm** and outer ring ≈ 28 × 20 mm.
- Flex pins along x on a forearm fork ≈ 46 mm wide. Dev pins along y on a palm fork ≈ 32–36 mm deep, which is a little deeper than the palm but within a human wrist.
- The 32 PTFE tubes run as a crossed hourglass through the ring.
- **Make every wrist part symmetric about x = −33.5** (the forearm axis = the mirror plane), so the left hand uses the same printed parts.
- Keep (b) as the fallback.

### 2.3 Sheath details

- Tube ends: anchor each tube in the palm (where today's 2.2 → 1.2 mm bore step and the wrist plate already are) and in a **forearm anchor plate** ≥ 40 mm below C. Use flared or chamfered metal ferrules so the line doesn't saw on the tube end.
- Inside the wrist span: a 0.6 mm-ID liner, or a coil sheath (section 1.4 b–c).
- Gentle pre-bow: about 5–10 mm of sideways room for each tube in the forearm part of the span.

---

## 3. Wrist torque and actuator

### 3.1 Loads

From `sim/models/tendra_hand_v1_wrist.xml` (solid PLA; a printed hand is lighter, and pads and sensors add a little): everything below the wrist (palm, fingers, thumb) = **289 g**, centre of mass **71 mm** from C, fingertips ≈ 200 mm out.

Static torque, worst case (forearm horizontal), τ = m·g·r:

| case | torque |
|---|---|
| hand alone | **0.20 N·m** |
| hand + 0.5 kg held in a power grasp (≈ 110 mm out) | 0.74 N·m |
| hand + 0.5 kg at the fingertips | 1.19 N·m |
| hand + 1.0 kg in a power grasp | 1.28 N·m |
| hand + 1.0 kg at the fingertips | **2.17 N·m** |
| PTFE tube spring at full bend (40 mm bent length) | +0.37 N·m |
| dynamics (I ≈ 0.002 kg·m², 30 rad/s²) | +0.06 N·m |

**Tendon reaction with Bowden sheaths:** cut through the wrist. At the cut, each sheath carries the line's tension T (pulling away from the hand) and an equal compression T in the tube (pushing on the hand). The two forces lie on the same line (the line is inside the tube), so **they cancel: no net force and no net moment on the wrist**, however hard the fingers squeeze. What is left is only the tube spring (0.25–0.5 N·m) and friction. Bare strands would add Σ T_i·b_i, which is ≈ 3 N·m in a stall fist (section 1.3). That is another reason for sheaths.

**Design target:** ≈ 1.3 N·m continuous (hold 1 kg in a power grasp, or 0.5 kg at the tips, plus the tube spring) and **≈ 2.5 N·m peak**.

### 3.2 How a 2-RSS drive turns servo torque into wrist torque

Rods attach to the palm at x = C_x ± d and at y offset y_r from the flex axis. For long, roughly vertical rods:

- **τ_flex = (F₁ + F₂)·y_r** (both push together);
- **τ_dev = (F₂ − F₁)·d** (one pushes, one pulls).

Stroke per rod ≈ y_r·(sin 70° + sin 60°) + d·(sin 20° + sin 30°) = 1.81 y_r + 0.84 d. A crank of radius r_c turning ±70° gives a stroke of 1.88 r_c.

Example: y_r = 15 mm, d = 20 mm. Stroke 44 mm, so r_c ≈ 24 mm. At mid-stroke, with gear ratio G between servo and crank:

> τ_flex ≈ 2·G·τ_s·y_r/r_c ≈ **1.3·G·τ_s**,  τ_dev ≈ 2·G·τ_s·d/r_c ≈ **1.7·G·τ_s**

Energy check: the work out can't exceed the work in, 2·τ_s·Δφ_s ≥ τ_w·Δθ_w, so a gear ratio is the only real way to get more torque.

| servo (stall) | wrist flex torque at stall, G = 1 / 2 / 3 | continuous (~30 % of stall), G = 2 | fits? |
|---|---|---|---|
| SCS0009 (0.23 N·m) | 0.30 / 0.60 / 0.90 | 0.18 | no |
| SCS2332 (0.44 N·m; DexHand's wrist) | 0.57 / 1.1 / 1.7 | 0.34 | hand alone only |
| STS3215, 7.4 V (~1.9 N·m, **(verify)**) | 2.5 / 4.9 / 7.4 | 1.5 | yes |
| **STS3215, 12 V (~2.9 N·m, (verify))** | 3.8 / **7.5** / 11 | **2.3** | **yes, with margin** |
| HLS3930M (≈ 3 N·m class, current/torque control, **(verify)**) | ≈ 3.9 / 7.8 / – | ≈ 2.3 | yes. Torque control is nice for compliant contact |

**Recommendation: 2 × STS3215 (12 V), each with a 2:1 belt (GT2) or spur gear to its crank. Push rods with ball links, mounted low in the forearm** (below the finger servos), rods along the forearm sides.

- Speed: at about 300°/s at the servo **(verify)**, 2:1 and the linkage give a wrist speed of ≈ 150–200°/s, enough for everyday tasks.
- Bus: STS servos use the same half-duplex TTL bus (as IDs 17 and 18, STS memory table), but they need their own **12 V** supply branch.
- Its 360° magnetic encoder handles the ±140° servo travel that a 2:1 ratio needs.
- A servo inside the wrist (serial drive) is not recommended: it adds mass where it hurts most and blocks the hollow centre.

---

## 4. Risks and limits

1. **Grip changes with wrist angle.** Friction in the bent sheaths takes 12 % (μ 0.1) to 27 % (μ 0.25) of the force at full bend, on top of the palm and finger routing. Thumb strands are the worst (≈ 0.45 kept at μ 0.25). Position control hides this, but force doesn't. Measure it per wrist angle and add a correction (servo load/current reading, or a pull test).
2. **Hysteresis.** Friction depends on the direction of motion. This gives a few tenths of a millimetre of line stretch, roughly 1° at the joint. It is small with braid.
3. **Slack at big angles** from bore clearance (≈ 0.4 mm, about 12 N with a 1 mm bore). Fix it with a tighter liner, a soft ext spring and feed-forward from the wrist angle.
4. **Tube squash** of 1–2 mm under load in the free wrist span, which is 10–20° of finger "give". Keep the span short and stiff, or use a coil sheath there.
5. **Tubes in the bundle fight for length** (4–7 mm per tube at the extremes, for the cardan). Leave slack and do a mock-up test: print the cardan, thread 32 tubes with braid, and check line length change and friction at the corners of the range. If it's bad, switch to concept (b).
6. **Wear** where the line leaves a tube (palm end, forearm anchor, spool side): use rounded metal ferrules and keep the fleet angle ≤ 4–5°. That means re-aiming or lengthening the forearm. Watch PTFE wear on the inner side of the wrist bend.
7. **Thumb strands** come out of the thumb bay very low (z −12…−24), so their bend toward C is 40–60°. Give them the waist slots nearest the thumb side and do their routing together with the open thumb-routing rework.
8. **Range of the cardan.** 70° of flexion is a lot for a fork-and-ring joint. Check in CAD (fork relief cuts, the gap at 30 mm). Also note: above ≈ 55° flexion with radial deviation, the thumb base hits the forearm in the sim. That needs a slimmer forearm top.
9. **Left/right mirror.** Make the wrist symmetric about x = −33.5, so the same printed parts work for both hands. Only the strand-to-slot and strand-to-spool maps mirror. In the sim, `wrist_dev`'s sign already flips for the left hand (`LEFT_AXIS_SIGN`); the hardware must follow the same "positive = toward the thumb" rule.
10. **Sim follow-up.** The sim wrist is ideal: no tube spring, no friction. Later, add ≈ 0.3–0.5 N·m/rad of wrist stiffness, tendon friction as a function of wrist angle, and the actuator torque limit for the chosen drive (`ARM_JOINTS` in `sim/convert_v1_wrist.py`; today it is 2 N·m), so trained policies don't expect a wrist that isn't there.

## 5. Suggested next steps

1. A mock-up: print the oval cardan (30 mm gap) and run 32 × 1 × 2 mm PTFE tubes through it as a crossed hourglass. Measure line length change (dial gauge or servo encoder), friction (spring scale at the drum) and slack over the wrist range. Compare 1.0 mm and 0.6 mm bores.
2. Re-number the strand→spool map for the crossed hourglass, re-aim the palm channels at C in `tendon_router.py` (with tests), and decide how much longer the forearm gets (+40–60 mm).
3. Lay out the 2-RSS drive (2 × STS3215 + 2:1) in the forearm and check the rod paths against the strand fan.

## Sources and confidence

- All geometry comes from `tendon_routes.json` and the MuJoCo wrist model; the formulas are standard (capstan, Hooke, Euler, beam bending).
- Friction coefficients, braid stiffness and the SCS0009 values are from `research/references/hands/tendon-mechanics.md`.
- Servo specs for STS3215, HLS3930M and SCS2332, the circle-packing constant, coil-sheath sizes and bearing sizes are **(verify)**.
- The tube-mismatch figures are a first-order, worst-tube estimate; the mock-up is the real test.
