# Tendon-driven hand mechanics: smoother, stronger, more reliable

Date: 2026-09-30. Scope: the mechanical side of Tendra Hand V1 (20 DOF, 20 × Feetech SCS0009, one antagonistic loop per joint, 6 mm spool : 6 mm drum, 40 strands in 1 × 2 mm PTFE tubes). Part of `research/references/hands/`.

> **How to read the sources in this note.** Web search and web fetch **failed during this session** because the tool's permission check gave no answer. The report builds on:
> - (a) the repo's own research, which was checked against sources on 2026-09-28 (`research/experiments/2026-09-28-full-hand/research.md`, the SCS0009 datasheet read there, ORCA, Shadow, the Bowden hand, MM-Hand);
> - (b) standard engineering relations (capstan, Hooke, Mersenne);
> - (c) well-known literature and datasheet values quoted from memory.
>
> Everything in class (c) is marked **(verify)**. Check those before buying or fixing a design value. The formulas and the worked numbers don't depend on (c) except where noted.

Units: N, mm, N·m, rad. Degrees only in prose.

---

## 0. Summary for the impatient

1. **The tendon is the biggest problem in the drive train today.** 0.4 mm nylon monofilament is about **15–25× more stretchy** than a braided UHMWPE (Dyneema/Spectra) line of the same strength. On a 200 mm strand at the working load it stretches **several mm**, which is **tens of degrees** at a 6 mm drum. That is probably part of why V0 needed "2× more steps", and it also makes stick-slip jerkiness worse. Switch to **braided UHMWPE, about 40–50 lb / 0.33–0.36 mm** (e.g. PowerPro or Sufix 832; catalogue diameters checked 2026-09-30, see 1.5), and pre-stretch it. (Measured at 1/3 of break load, braid stretches 0.7–1 % vs 2–9 % for mono, i.e. ~3–10× less; the 15–25× figure compares lines of equal strength at equal load and is an estimate.)
2. **Friction, not the servo, decides how much force reaches a distal joint.** A DIP flexor crossing two bent joints over printed PLA edges keeps only about **30 %** of the spool force in a fist. With metal/PTFE contact points and UHMWPE it keeps about **65 %**. Rule: **the line touches only PTFE, polished metal, or its own drum.**
3. **The SCS0009 is weak for a humanoid hand.** At 1:1 the index can press with only about **2.5–3.5 N at the fingertip (stall)** and about **1.5 N sustainably**. A human manages 50–70 N in a tip pinch. Cheap improvements get 2×, e.g. 1.5–2:1 through a smaller spool and a bigger MCP drum. The real fix is a stronger servo in the same slot class. The Dynamixel **XL330-M288** has about 2.3× the torque, current sensing and no pot wear.
4. **A stiff line needs a fine tensioner or a little series compliance.** With braid, 0.1 mm of loop-length error is about 2–3 N of tension change. Use a **split spool with a friction clamp** (continuous adjustment) and consider a **soft spring on the extensor strand only**.
5. **Grip comes from the pads as well as the motors.** Silicone pads (μ ≈ 1) instead of bare PLA (μ ≈ 0.3) cut the squeezing force needed to hold an object by about 3×. With weak servos, that is the cheapest "strength" upgrade there is.

The prioritised list is in [section 9](#9-recommendations-for-tendra-v1-prioritised).

---

## 1. Tendon materials

### 1.1 What matters for a hand tendon

| Property | Why it matters for us |
|---|---|
| **Stiffness** (axial EA, in N per unit strain) | Stretch = position error and "mushy" joints. Stretch plus friction also causes stick-slip. |
| **Creep** (slow lengthening under constant load) | The loop loses pretension over hours or days, so you get slack and backlash. |
| **Strength and knot efficiency** | The line must survive stall force × a safety factor, *at the knot*, which is the weak point. |
| **Friction** against the sheath, pins and drum | Force loss (capstan), hysteresis, and stick-slip. |
| **Bending fatigue and abrasion** | Many cycles over small radii and edges fray the line. |
| **Diameter** | Must fit 1.2 mm bores, 1 mm PTFE bore, slits and the on-axis crossings. |
| **Water uptake and temperature** | Nylon absorbs water and changes length and stiffness. UHMWPE softens above ~70 °C. |

### 1.2 Candidates

Typical values, all **(verify)** against the maker's datasheet. Fibre values are for the raw fibre. A braided line is less stiff and less strong per area, because the braid angle and the gaps between fibres cost some of both.

| Material | Tensile modulus E | Elongation at break | Creep | Friction | Knots | Notes |
|---|---|---|---|---|---|---|
| **Nylon (PA6/66) mono**, our 0.4 mm | 1.5–3 GPa | 15–30 % | High, and grows with humidity | Medium (0.2–0.3 on PLA) | Good (85–95 % with Palomar/clinch) | Absorbs a few % water, which softens it. Cheap. |
| **UHMWPE braid** (Dyneema SK75/SK78, Spectra): PowerPro, Sufix 832, Spiderwire | Fibre ~100–120 GPa; braid in practice ~30–60 GPa | 3–5 % (braid) | Medium. Noticeable at >20–30 % of break load, low at a few % | **Very low** (self-lubricating) | **Poor, slippery**: 50–80 % depending on the knot | The most popular choice: DexHand (Sufix 832 80 lb), RUKA (braided line rated 200 lb, confirmed), DLR (Dyneema), Shadow (Spectra). ORCA uses 0.4 mm braided **nylon** (confirmed). Melts ~145 °C, and long loads above ~70 °C creep fast. |
| **Dyneema DM20** (low-creep grade) | Like SK78 | Like SK78 | **Near zero** at working loads | Low | Poor | Sold mostly in sailing ropes. Hard to find as sub-0.5 mm line (verify). |
| **Vectran** (LCP) | ~65–75 GPa | ~3.5 % | **Very low** | Medium | Medium | Used in kite lines and bow strings. Weaker than UHMWPE against UV and abrasion. A good choice where creep matters most. **Used in hands (confirmed):** Halodi/1X's cable-drive patent WO2018149499A1 specifies Vectran cables; Aero Hand Open uses braided Kevlar/Vectran. |
| **Kevlar / aramid** | ~70–130 GPa | 2–4 % | Low | Medium–high | Poor | **Poor bending fatigue over small radii** and self-abrasion, so avoid it on 6 mm drums and sharp pins. |
| **Steel cable** 7×7 / 7×19, nylon-coated (fishing leader, 0.3–0.5 mm) | Rope ~100 GPa | ~2 % | None | Medium (coated) | Needs crimps | Very stiff, but it **fatigues over small pulleys** (wire-rope rule: pulley Ø ≥ 20–40× rope Ø; our 12 mm spool is only ~25–30× a 0.45 mm rope). Kinks. The antagonistic Bowden hand (arXiv 2512.24657) uses 0.75 mm coated steel with bigger bobbins. |

### 1.3 Worked comparison: how much does our strand stretch?

Axial stiffness of one strand of free length L: **k = EA / L**. A stiff line is what makes a high "EA" number. For a finished line it is easiest to estimate EA from the breaking load and the elongation at break, because fishing lines are sold by break load: **EA ≈ F_break / ε_break** (a secant estimate; real lines are nonlinear).

Assumptions: 0.4 mm nylon mono breaks at about 70–100 N (typical charts put 0.36–0.40 mm at ~20 lb / 9 kg; brand-dependent) at about 25 % strain (nylon's typical 25–35 % elongation), so **EA_nylon ≈ 300–450 N**. 50 lb UHMWPE braid breaks at about 222 N (nominal; real breaks are often higher) at about 4 %, so **EA_braid ≈ 5 500 N**. Strand length from the spool to the DIP drum ≈ **200 mm**: about 80 mm through the palm (z 38.5 → −42), 48–110 mm straight in the forearm, and 70–80 mm through the finger (`tendon_routes.json`). Deeper servo levels are about 260 mm.

| | nylon 0.4 mono | UHMWPE braid 50 lb |
|---|---|---|
| k = EA/L (L = 200 mm) | **1.5–2.3 N/mm** | **≈ 28 N/mm** |
| Stretch at 10 N | **4.4–6.7 mm** | 0.36 mm |
| Joint error at a 6 mm drum (Δq = Δx / r) | **42–64°** | **3.4°** |

Joint stiffness from a pretensioned loop. Both strands are in tension, so they act in parallel on the drum:

> **K_joint = r_d² · (k_flex + k_ext)**   (halves if one strand goes slack)

| | nylon | braid |
|---|---|---|
| K_joint (r_d = 6 mm, both strands taut) | 0.11–0.17 N·m/rad | **≈ 2.0 N·m/rad** |
| Deflection under 0.05 N·m (a light push at the tip) | 17–26° | **1.4°** |

For comparison, the SCS0009's own position loop is probably about 2–3 N·m/rad stiff. That is a guess: stall torque over a few degrees of error. Its gear backlash is ≤ 0.5° (datasheet). **With nylon the tendon is about 20× softer than the servo, so it dominates. With braid the two are about equal.**

> Take-away: nylon mono makes every joint a soft spring. That is not all bad (see compliance, section 6), but the spring constant changes with humidity, creeps, and makes the joint angle unpredictable. We should *choose* our compliance, not get it by accident.

### 1.4 Knots and anchoring (the real weak point)

- A braid's rated strength is not its knot strength. Slippery UHMWPE knots can hold as little as **50–60 %** (verify for the knot you use; the Palomar and the 8–10-turn uni knot are the usual braid knots).
- **Let the capstan protect the knot.** Wrap the line around the drum or anchor boss before the knot. The knot then sees only T · e^(−μθ). Example: 1.5 turns (θ = 9.4 rad) on PLA with μ ≈ 0.2 leaves e^(−1.9) ≈ **15 %** of the load on the knot. Our strands already wrap the drums by < 180°, so add an **extra full wrap around an anchor post** next to each tie hole.
- Seal the knot with a drop of thin CA *after* it's tight. CA barely bonds UHMWPE, but it stops the knot loosening under cycling.
- No sharp exits: tie holes need a **≥ 0.5 mm chamfer or fillet**. Line bent over a radius close to its own diameter loses a large share of its strength (the common rule of thumb for fibre rope is bend Ø ≥ 5–10 × line Ø; verify for braid).

### 1.5 Recommendation: tendon

**Braided UHMWPE, 8-carrier, 50 lb class, ≈ 0.36 mm.** For example **PowerPro Spectra 50 lb (0.014", 0.36 mm)** or **Sufix 832 50 lb (0.014", 0.356 mm)**. *Correction 2026-09-30:* the earlier values (0.33 / 0.30 mm) were wrong; both catalogues give 30 lb = 0.28 mm, 40 lb = 0.33 mm, 50 lb = 0.36 mm, 65 lb = 0.41 mm, 80 lb = 0.43 (PowerPro) / 0.46 mm (Sufix). Sufix 832 = 7 HMPE fibres + 1 GORE fibre, 32 picks per inch. Why:
- Break ≥ 222 N nominal. At a 60 % knot that's 133 N, a **safety factor of 3.5** against the SCS0009's 37.7 N stall tendon force at a 6 mm spool. It is still ≥ 1.5 against an XL330 at the same spool (87 N) if we upgrade. For XC330-class servos use 80 lb (0.43–0.46 mm).
- It fits the 1 mm PTFE bore with room to spare (a 0.36 mm line in a 1.0 mm bore).
- About 15× stiffer than our nylon, low friction, and doesn't absorb water.
- Use a **thin, round, coated** braid (8 carriers, tightly braided), not a flat 4-carrier. Flat braids flatten on the drum and saw into printed edges.

**Before use, pre-stretch it:** hang about 30–50 % of the break load (7–10 kg) on a 1 m length for a few hours. That removes the braid's "construction stretch" (fibres bedding in), which would otherwise show up as slack in the first days.

**Vectran** is the choice if creep over weeks turns out to be a problem (check with the creep test in section 8).

---

## 2. Friction: how much of the servo's pull reaches the joint

### 2.1 The capstan (Euler–Eytelwein) equation

A line sliding around a curve with wrap angle θ (rad) and friction coefficient μ:

> **T_out = T_in · e^(−μ·θ)**   (pulling side T_in, far side T_out)

Two things are not obvious at first:
- **θ is the total turning of the line, wherever it happens.** An S-curve counts twice (turn out, turn back), and small bends add up. In a Bowden sheath θ is the integral of its curvature, so a long gentle bend costs the same as a short sharp one with the same angle. The radius only matters for wear, fatigue and kinking.
- **Loss is proportional to tension.** Pretension costs friction in *both* strands all the time, and a strand under grasp load loses the most.

For a tendon-sheath drive (Kaneko et al. 1991; Palli, Borghesan & Melchiorri, IEEE T-RO 2012 "Modeling, identification and control of tendon-based actuation systems", verify) the result is also **direction-dependent**. Pulling, you lose e^(−μθ). When the servo *lets go*, the joint side must overcome the same friction, so the tension at the joint is higher than at the servo. This gives a **force hysteresis loop**: the same servo command gives a different joint force (and, through the line's stretch, a different position) depending on the direction you came from.

### 2.2 Friction coefficients (typical ranges, verify on our parts)

| Pair | μ (kinetic) | Comment |
|---|---|---|
| UHMWPE braid in PTFE tube | ~0.05–0.10 | Both are low-friction polymers. Measure (section 8). |
| Nylon mono in PTFE tube | ~0.1–0.2 | Repo research.md: 0.04–0.2 in the literature. |
| Line on polished steel pin | ~0.1–0.2 | Lower with UHMWPE. |
| Line on bare FDM PLA/PETG | **0.3–0.5** | Layer lines, plus the edges abrade the line (repo research.md: PLA pin-on-disc 0.38–0.57). |
| Line on a rolling pulley (ball bearing) | ~0.005 equivalent | Only bearing friction. Needs space. |

Static friction is higher than kinetic friction for most of these pairs, especially polymer on printed polymer. **Stick-slip** happens when μ_static > μ_kinetic *and* the drive is compliant: the servo winds up the stretchy line until friction breaks free, then the joint jumps. Stiff line + low μ + low μ_s/μ_k ratio (PTFE, UHMWPE and steel all have small ratios) = smooth motion.

### 2.3 Our routes: wrap angles (estimated)

Pieces of each strand's path from the spool to its own drum (from `tendon_router.py` and `tendon_routes.json`):
1. Spool → wrist plate: **straight, 0°**. Good: the tangent point on a round spool doesn't move as it turns.
2. Palm S-curve in PTFE, wrist plate → finger entry. Example `index_dip_flex`: 9.6 mm sideways over 32.5 mm of height, so the average slope is 16.5° and the peak ≈ 24°. Total turning ≈ **50°** (out and back). The long near-straight routes (min bend radius > 200 mm) have ≈ 10–20°. Index strands squeezed beside the thumb bay: up to **56° at the entry**, plus the S-curve.
3. Finger entry → own drum: bare line, crossing each more proximal joint **on its axis**. When a joint is bent by q, the line turns by q at that crossing, around whatever edge the slit or hole has.
4. The wrap on its own drum doesn't count: the line is tied to the drum and doesn't slide there.

To get exact numbers, run this in the repo. It wasn't run for this note because Bash was blocked.

```python
import json, numpy as np
d = json.load(open("hardware/robot_description/v1_export/tendon_routes.json"))
for s in d["strands"]:
    v = np.diff(np.array(s["points_mm"]), axis=0)
    v = v[np.linalg.norm(v, axis=1) > 1e-9]; v /= np.linalg.norm(v, axis=1)[:, None]
    turn = np.degrees(np.arccos(np.clip((v[:-1] * v[1:]).sum(1), -1, 1))).sum()
    print(f"{s['name']:28s} palm/wrist turning {turn:6.1f} deg")
```

### 2.4 Worked example: index DIP flexor, straight finger vs fist

Assumptions: palm S-curve 50°, entry bend 30°, in a fist the MCP is at 90° and the PIP at 95° (so the DIP flexor turns 185° = 3.23 rad at the two crossings).

| Segment | θ (rad) | **Now**: nylon, bare PLA edges (μ 0.3), PTFE μ 0.1 | **Improved**: UHMWPE, PTFE μ 0.06, polished steel/brass at the edges (μ 0.1) |
|---|---|---|---|
| Palm S-curve (PTFE) | 0.87 | 0.916 | 0.949 |
| Entry bend | 0.52 | 0.855 (PLA edge) | 0.949 (steel pin) |
| MCP + PIP crossings (fist) | 3.23 | 0.379 | 0.724 |
| **Straight finger, total** | | **0.78** | **0.90** |
| **Fist, total** | | **0.30** | **0.65** |

**Result: in a fist only 30 % of the servo's pull reaches the DIP drum through printed edges, and 65 % with metal or PTFE contacts. That's a 2.2× gain in fingertip force from material choices alone.** The same logic explains the V0 knuckle that "extends hard": its strands dragged over a funnel in the knuckle base (log 2026-09-29), with a stretchy nylon line.

The **thumb** is worst. An `thumb_ip` strand goes through the palm (≈ 60°), up the hollow journal (≈ 90°), over the sheath loop (≈ 135°), and across the MCP (up to 95°). That's about 285° in PTFE plus 95° at a crossing. At μ = 0.1 in PTFE and a PLA crossing: 0.61 × 0.61 ≈ **0.37**. At μ = 0.06 and a steel crossing: 0.74 × 0.85 ≈ **0.63**.

### 2.5 Sheaths, pulleys and lubricants

- **PTFE tube quality varies a lot.** MM-Hand (arXiv 2604.17245, in repo research) found a plain PTFE tube "relatively high friction" and did better with a **Bambu-style smooth-bore PTFE feeder tube** or a spring (coil) tube. Buy "Capricorn"-style or feeder-grade 1 × 2 mm tube, and check the bore by pulling a weight through a fixed bend (section 8).
- **Fix both ends of each sheath.** A Bowden sheath works only if it can't slide: the sheath takes the compression reaction. The 2.2 → 1.2 mm step at the finger end and a stop at the wrist plate do this. A loose sheath end "pumps" and adds hysteresis.
- **Chamfer the tube mouths** (cut square, then twist a 2 mm drill by hand) and make the exit **coaxial** with the next straight run. A line leaving a tube at an angle saws into the tube lip.
- **Pulleys vs sliding guides.** A ball-bearing idler (e.g. MR63 3 × 6 mm with a printed groove sleeve) removes the loss at that bend almost completely. It's worth it where θ is large and the location is fixed: the spool exit, the cmc_rot deflection pin, the palm entry of the index strands. Everywhere else, a **polished steel dowel** (ground ISO 8734 dowel pins are cheap and already smooth) is the 80/20 choice. The repo already plans Ø3 steel dowels for the thumb. Use them anywhere a bare line turns more than ~20°.
- **Lubricants.** UHMWPE and PTFE barely need any. A dry PTFE spray (bicycle dry lube) in the tubes helps nylon. Avoid oils and greases: they collect dust, which then acts like sandpaper, and some solvents swell PLA or PETG. Silicone oil is safe for these polymers if you want a wet lube (verify).

---

## 3. The antagonistic loop: tension, slack and tensioners

### 3.1 How one loop on one spool behaves

The flexor strand winds onto the spool while the extensor unwinds by the same length (both have radius r_s). At the drum the joint turns by q = φ · r_s / r_d. The loop stays tight only if **every change in joint angle changes both strand paths by equal and opposite amounts**. Anything that breaks that symmetry makes slack in one strand or over-tension in both:

| Cause | Size | Fix |
|---|---|---|
| Different effective radius for the flex and ext strands (groove depth, one strand wraps on top of itself) | Δ = (r_f − r_e) · Δq. 0.2 mm × 1.75 rad = **0.35 mm** | Same groove depth for both, one layer, flanges. Print the drum and spool grooves in the same orientation. |
| Crossing point off the joint axis (offset e) | Δ ≈ e · Δq on *both* strands of a pass-through loop, in the same direction | On-axis slits (done for the index). Keep the slit width ≈ line Ø + 0.2 mm so the line can't wander. |
| Strand loses contact with its drum at the end of the range (wrap runs out) | The moment arm becomes non-constant | Each strand needs contact arc ≥ joint range + ~20°. Check the tie-hole position (the thumb's cmc_rot keeps 20° spare at both limits, good). |
| Line stretch under load | F/k (section 1.3) | Stiffer line. |
| Creep | Slow slack over hours or days | Low tension, low-creep line, re-tension. |
| Humidity (nylon) | Nylon changes length with moisture | UHMWPE. |

**Why this matters more with a stiff line.** Tension change = k · Δ. With braid (k ≈ 28 N/mm at 200 mm), the 0.35 mm radius mismatch above adds or removes **≈ 10 N** over the range. With nylon it's about 0.6 N. A stiff line is precise but **unforgiving of geometry errors and needs a fine tensioner**, or a deliberate soft element (section 3.4).

**A correction to the repo notes.** In a single-spool loop, **pretension doesn't load the servo's torque**. The two strands pull on opposite sides of the spool, so their torques cancel: τ = r_s(T_f − T_e) = 0 at rest. Pretension shows up as:
- **friction drag**: roughly 2·T0·μθ, and it fights every motion;
- **radial (side) load on the servo shaft**: both strands leave the spool toward the wrist, *in parallel*, so the side load is **T_f + T_e** (up to ≈ 38 + 5 N at stall);
- **creep**.

So the SCS0009's "80 % for 2 s" overload cut-off is triggered by friction and grasp load, not by pretension itself. The side load is a real risk. The SCS0009 has no published radial-load rating (verify), and the output shaft bushing in a micro servo isn't meant for 40 N. **Support the spool on the far side** with a small bearing or bushing in the forearm frame. That's cheap and makes it much more reliable.

### 3.2 How much pretension?

- The extensor stays taut while the net joint load, converted to tendon force, is < 2·T0. A bigger T0 means less backlash, but more friction (and the friction hysteresis scales with it) and more creep.
- With braid and good sheaths, **T0 ≈ 3–5 N** (≈ 10–15 % of the 37.7 N stall tendon force) is a good start.
- Friction drag at T0 = 5 N, μθ = 0.1 × 1.5: about 2 × 5 × 0.15 ≈ 1.5 N-equivalent, i.e. 0.009 N·m at the spool, **4 % of stall**. With nylon on PLA (μθ ≈ 0.3 × 3), the same T0 costs 2 × 5 × (e^0.9 − 1) ≈ **15 N**, 40 % of stall. That's the V0 symptom.

**Measure tension without a gauge (Mersenne's law).** Pluck the free strand between the wrist plate and the spool and read its pitch with a phone spectrum app:

> f = (1 / 2L) · √(T / m′)   →   T = m′ · (2Lf)²

For 0.36 mm (50 lb) UHMWPE braid, m′ ≈ 0.97 g/cm³ × 0.102 mm² × ~0.8 fill ≈ **7.9 × 10⁻⁵ kg/m** (verify by weighing 10 m of line). With a free span L = 50 mm and T = 5 N, f ≈ 2.5 kHz. **A 10 % change in tension is only 5 % in pitch**, so it's good for matching all 40 strands and spotting slack, not for precise readings.

### 3.3 Tensioning mechanisms

| Mechanism | Resolution | Pros | Cons | Used by |
|---|---|---|---|---|
| **Split spool + friction clamp**: two discs, one per strand, locked by the M2 horn screw | Continuous | Tiny, cheap, fits our 6 mm spool, adjusts one strand against the other | Must not slip. Check the clamp torque | Shadow (split spool, verify details) |
| Split spool + ratchet or serration | 360°/n teeth. 60 teeth = 6° = **0.63 mm** at r 6 | Can't slip | Too coarse for braid (0.63 mm × 28 N/mm ≈ 18 N per click) unless there's compliance | ORCA (ratchet spool, with braided nylon) |
| Tension screw per strand (M2, 0.4 mm/turn) | 0.05 mm per 1/8 turn | Very fine | Bulky ×40 | Many research hands |
| Bike-style barrel adjuster at the sheath end | Fine | Standard Bowden solution | M5–M7 thread, too big for a 3 mm tube pitch | Bicycles |
| Spring tensioner (spring-loaded idler or anchor) | Automatic | Takes up creep and geometry error, constant tension | Adds compliance, and costs force if it's on the flexor | Prosthetics, DLR (VSA) |
| Software offset (servo zero) | 0.29° = 0.03 mm at r 6 | Free | Only moves *both* strands together, so it can't change loop tension | – |

**Friction clamp check (split spool).** An M2 screw tightened to about 0.2 N·m clamps with roughly F = T/(0.2·d) ≈ 0.2/(0.2 × 0.002) = **500 N** (the standard screw torque rule, K ≈ 0.2). With PLA-on-PLA μ ≈ 0.3 and a mean friction radius of 4 mm, it holds about 0.3 × 500 × 0.004 ≈ **0.6 N·m**. That's 2.6× the servo's 0.226 N·m stall, which is enough. Knurl or rib the faces, add a witness mark, and test it by hanging 5 kg on one strand.

### 3.4 A good idea to test: a soft extensor, a stiff flexor

The flexor carries the grasp load, so make it **stiff** (braid). Give the extensor strand a **small series spring** (k_s ≈ 1–2 N/mm, preload 3–5 N, ≥ 2 mm of travel). For example a spring-loaded idler arm in the forearm's straight run, or a compression spring behind the extensor's anchor. What this does:
- keeps the loop taut automatically through creep, geometry errors (±1–2 mm) and humidity, so no fine tensioner is needed;
- keeps the flexor stiff where it matters, because grasp reaction forces load the flexor;
- makes the joint soft when *pushed closed* (toward the palm), which is harmless and a little safer;
- costs a little force: the servo must also stretch the extensor spring by the loop travel. That's k_s·Δx ≈ 1.5 × 1 = 1.5 N extra at most, and it's constant, so it can be calibrated out.

Try it on the test finger against a plain split spool before committing 20 servos.

### 3.5 Actuation schemes (for context)

| Scheme | Motors for N joints | Examples | Notes |
|---|---|---|---|
| **N, closed loop on one spool** (ours) | N | Shadow Dexterous Hand | Pull both ways, few motors. Tension set mechanically. |
| **N + 1 tendons** (coupled network) | N + 1 | Stanford/JPL (Salisbury & Craig, IJRR 1982, verify) | Fewer tendons, but every motor affects every joint, so control is coupled. |
| **2N** (independent agonist and antagonist) | 2N | Utah/MIT hand (Jacobsen et al. 1986), DLR Awiwi (variable stiffness) | Tension and stiffness are controllable and slack can't happen. 2× the motors, cost and space. |
| **N + spring return** | N | Many 3D-printed prostheses (elastic-cord extensors) | Simple, always taut. The motor always fights the spring, and there's no active extension. |
| **Underactuated** (one motor, several joints through a differential) | < N | SDM hand (Dollar & Howe, IJRR 2010), most prostheses | Adapts to shapes by itself, and it's robust. Not dexterous. |

Keep **N closed loops** for V1: it's the most dexterous option at our motor count. Section 3.4 borrows the best part of the spring-return idea.

---

## 4. Force and torque budget

### 4.1 Formulas

- Tendon force at the spool: **F_t = τ_servo / r_s**
- Joint torque: **τ_j = F_t · η · r_d = τ_servo · G · η**, with the ratio **G = r_d / r_s** and the transmission efficiency η from section 2
- Joint speed: ω_j = ω_servo / G. Servo travel needed = G × the joint range (must be ≤ 300° on the SCS0009)
- Fingertip force for a lever arm ℓ (the distance from the joint axis to the fingertip force's line of action): **F_tip ≈ τ_j / ℓ**. The weakest joint sets the limit. In a pinch that's usually the MCP, which has the longest lever.

### 4.2 SCS0009 numbers (from the datasheet, repo research.md)

6 V: stall **0.226 N·m**, rated 0.074 N·m, no-load 0.10 s/60° (600°/s). Overload cut-off at > 80 % for 2 s. For "holding a grasp" we use **0.11 N·m (≈ 50 % of stall)**, safely below the cut-off.

Note: **confirmed 2026-09-30 from the Feetech datasheet A/0: copper + steel (metal) gears in a PC plastic case**, 1:416, iron-core motor, carbon-film potentiometer (100 000 cycles min), stall 0.185 / 0.226 N·m at 4.8 / 6 V, stall current 0.8 / 1.0 A, horn 20T / OD 3.95 mm. The "plastic gears" in the task description was wrong.

Index: proximal 40, middle 22.4, distal 15.8, pad about 4 mm. The fingertip is about **82 mm from the MCP** with the finger straight, and about **60 mm** as a lever in a typical pinch pose. MCP flexor η ≈ 0.9 (short route, no crossings).

| Drive | G | Tendon force at stall | MCP τ at stall | F_tip stall, pinch (ℓ 60 mm) | F_tip hold 50 % | F_tip straight finger (ℓ 82 mm), stall | Joint speed, no load |
|---|---|---|---|---|---|---|---|
| **SCS0009, 6 : 6 mm (now)** | 1.0 | 37.7 N | 0.203 N·m | **3.4 N** | **1.7 N** | 2.5 N | 600°/s |
| SCS0009, spool 5, drum 7.5 | 1.5 | 45 N | 0.305 | 5.1 | 2.5 | 3.7 | 400°/s |
| SCS0009, spool 4, drum 8 | 2.0 | 57 N | 0.407 | 6.8 | 3.4 | 5.0 | 300°/s |
| SCS0009 + 2:1 moving pulley (block and tackle) | 2.0 | 38 N at the servo, 75 N in the tendon | 0.407 | 6.8 | 3.4 | 5.0 | 300°/s |
| **XL330-M288** (0.52 N·m at 5 V, confirmed ROBOTIS), 6 : 6 | 1.0 | 87 N | 0.468 | 7.8 | 3.9 | 5.7 | depends on the model |
| XL330-M288, spool 5, drum 7.5 | 1.5 | 104 N | 0.70 | 11.7 | 5.9 | 8.6 | |
| XC330-T288 (0.92 N·m at 11.1 V, ~1.0 at 12 V, confirmed; used by ORCA v1) | 1.0 | 150 N | 0.83 | 13.8 | 6.9 | 10.1 | |

With a lower η (the DIP in a fist, 0.3–0.65) the distal joints are weaker still. For the DIP itself the lever is short (≈ 12–15 mm to the pad), so the DIP is rarely the limit. The MCP is.

### 4.3 Compared with a human hand

Adult norms, right hand, ages 20–39 (Mathiowetz et al. 1985, *Arch Phys Med Rehabil* 66:69–72; **checked 2026-09-30 against the paper's tables 2–4**, lb converted to N):

| | Men | Women |
|---|---|---|
| Tip pinch (thumb to index tip) | ~ 80 N (17.6–18.3 lb) | ~ 50–55 N (11.1–12.6 lb) |
| Key (lateral) pinch | ~ 115 N (26.0–26.7 lb) | ~ 80 N (17.6–18.7 lb) |
| Power grip (Jamar, 2nd handle position) | ~ 535 N (120–122 lb) | ~ 310–350 N (70–79 lb) |

*Correction 2026-09-30:* the earlier table had key pinch ~100–110 / 70 N and grip ~450 / 280 N; the paper's means are higher, as above.

Most daily tasks need only a small fraction of maximum: roughly **< 10–20 N** of fingertip force for precision handling (verify). Commercial prosthetic hands deliver on the order of 70–140 N of grip (verify per product).

**So:** at 1:1, Tendra V1 is about **20–25× weaker than a human tip pinch**. For open hands: RUKA measured 2.74 N pinch, Aero Hand Open ~12 N per finger (both confirmed). It can hold light objects (the grasp sim lifts a light cylinder), but it can't cook or use tools "as well as a human". A 2:1 ratio gets it to about 7 N. An XL330-class servo at 1.5:1 gets about 12 N, which is the "daily tasks" range. Getting close to human strength needs a different class of actuator (section 4.5).

### 4.4 Cheap ways to more force, and what they cost

1. **Better pads** (section 7). They don't add force, but they cut the force *needed*. Two-finger pinch holding weight W needs a normal force N = W / (2μ) per finger. For a 200 g object (1.96 N): **3.3 N per finger on PLA (μ 0.3)**, but **1.0 N on silicone (μ 1.0)**. That's the same effect as 3× stronger servos.
2. **Lower friction** (section 2): up to 2× at distal joints in curled poses.
3. **A bigger MCP drum.** The MCP has room for r_d ≈ 7.5–8 mm (the knuckle is the thickest part). The PIP and DIP are limited by the phalanx height (~14 mm), so they stay at about 6 mm. The **MCP is the joint that limits fingertip force**, so that's where it pays. Servo travel: 100° × 1.33 = 133°, fine.
4. **A smaller spool.** r_s = 5 mm is realistic: the spline OD is 3.95 mm, so a 5 mm tendon radius leaves about 2.5 mm of hub wall. r_s = 4 mm is possible only with a **metal 20T horn or insert** under the spool. With G = 2 the travel is 100° × 2 = 200° (OK). **But not the thumb's cmc_rot**: 140° × 1.25 = 175° now, and 280° at G 2, which is too close to the 300° limit.
5. **A 2:1 moving pulley** near the servo: doubles the force without a smaller spool. But it needs a pulley and anchor per strand ×40. Too bulky for V1.

**What you give up:** speed. G = 2 gives 300°/s with no load, and roughly half that under a heavy load (a DC motor's speed drops about linearly with torque). A fist in about 0.5 s is still fine for grasping, since typical human grasp closing is a few hundred °/s (verify). Resolution gets *better*: 0.29° of servo is 0.15° at the joint.

### 4.5 Stronger servos (to evaluate for V1.1 / V2)

| Servo | Torque (stall) | Size | Feedback | Price class | Notes (verify all) |
|---|---|---|---|---|---|
| Feetech **SCS0009** (now) | 0.23 N·m at 6 V | 23 × 12 × 25 | carbon pot, **100 k cycle life** (datasheet) | ~$ | The pot wears out: 20 joints under RL or teleop use could hit 100 k cycles within months. |
| Feetech **STS3032** | **0.44 N·m at 6 V** (4.5 kg·cm), stall 1.2 A (confirmed, Feetech page) | 23 × 12 × **27.5** (2.25 mm taller than the SCS0009), 20 g, aluminium case, coreless | 12-bit magnetic encoder | ~$15–25 | Same plan, but **not a pure drop-in**: taller, and Pollen notes "servo horn is different", so spools change. 2× torque and no pot wear. STS memory table (not SCS). |
| Feetech **HLS3606M** | **0.59 N·m at 6 V**, stall 1.3 A (Feetech datasheet, confirmed) | 23 × 12 × 27.5, 20.6 g, 1:205, 25T / 4.95 mm spline | 12-bit magnetic, **current feedback + constant-current mode** | ~$30 | Used by Aero Hand Open (7 per hand). Best-value upgrade; HLS memory table. |
| Dynamixel **XL330-M288-T** | 0.52 N·m at 5 V | 20 × 34 × 26 | magnetic encoder, **current sensing** (current-based position mode) | $23.90 | Current control gives cheap force control. Plastic gears. Used by RUKA (fingers). (LEAP uses the XC330-M288, not the XL330.) |
| Dynamixel **XC330-T288-T** | 0.92 N·m at 11.1 V | 20 × 34 × 26 | same, metal gears | $89.90 | ORCA v1's choice (16 of them, confirmed in orca_core). ORCA's Feetech variant uses HLS3915. |
| Feetech **STS3215** | ~1.9 N·m (7.4 V) / ~2.9 N·m (12 V version) | 45 × 25 × 35, 55 g | magnetic encoder | ~$15–20 | Used by the LeRobot SO-100 arm. Too big for 20 in a forearm, and at a 6 mm spool its stall pulls ~300–500 N, more than the line and the print can take. |
| Feetech **SCS2332** | 0.44 N·m at 6 V (shop listing) | 23.2 × 12.1 × 28.5, 20 g | pot, 300° | ~$15–25 | Same SCS protocol as the SCS0009; coreless, metal gears. Used on DexHand's wrist. |
| **BLDC + capstan drive** | high, backdrivable | bulky | encoder | $$$ | Research-grade: zero backlash, force-transparent. Not for 20 DOF on a budget. |

For V1, finish with the SCS0009 (bought, same slot), but **design the forearm slots so a 20 × 34 mm servo class could fit in V2**, Checked 2026-09-30: the STS3032 and HLS3606M share the 23 × 12 plan but are 27.5 mm tall and have a different output spline, so allow +2.5 mm height and plan new spools.

---

## 5. Joint design

| Joint type | Friction | Precision | Robust to overload | Tendon routing | Verdict for V1 |
|---|---|---|---|---|---|
| **Printed pin in a printed hole** | High (PLA/PLA μ 0.3–0.5), stick-slip | Wears loose | Snaps | Easy | Avoid |
| **Steel dowel in printed holes** (child turns on the dowel) | Medium (μ 0.2–0.3) | Good if the holes are reamed | Good | Easy | OK for small joints |
| **Bushing** (brass tube, iglidur, PTFE sleeve) | Low (0.1–0.2), smooth | Good | Good | Easy | Good value |
| **Ball bearings** (e.g. MR84, MR63, 683) | Very low | Very good, no play | Good | Width cost | **V0 already uses 4 × 8 mm bearings**, so keep them where there's room |
| **Rolling contact joint** (two curved surfaces rolling, held by straps or tendons; CORE, Cannon & Howell 2005; ETH's 2023 "Getting the ball rolling" hand. *Correction:* ORCA does **not** use rolling joints; it uses poppable pin joints with bearings in arc-shaped grooves) | Very low (rolling) | The centre of rotation moves, but it's predictable | **Can dislocate and snap back**, which avoids breakage | Tendons through the contact point stay about constant length; needs a redesign | V2 candidate |
| **Flexure / compliant** (TPU or urethane hinge; SDM hand, Yale OpenHand) | None | The axis drifts under load; spring back-torque | Excellent | Easy | Good for underactuated grippers, not for a precise 20-DOF hand |

**V1's fingers use stub pins at the ends of the axis** (the middle must stay open for the on-axis tendon slit). Recommendations:
- **Don't let printed stubs rotate against printed holes.** Either press a short **steel dowel** into each stub side, or run the printed stub in a **bearing or bushing**. Printed PLA stubs: 4 mm Ø gives about 12.6 mm² per stub. Loads are small (the tendon forces are < 2 × 40 N), but a printed stub with its **layers across the stub** breaks at the layer interface under bending or impact.
- **Print orientation:** lay each phalanx **on its side**, so the joint axis is vertical. Then (a) the pin holes print round and accurate, and (b) the flexion bending and the lug loads run *along* the layers, not across them. Stubs that stick out of the side then have layers across them, which is weak. That's another reason for steel dowels there.
- **Tolerances:** print holes 0.1–0.2 mm undersize and drill or ream to the final size (a Ø3.0 drill for a Ø3 m6 dowel as a press fit in the parent, Ø3.1–3.2 for a running fit).
- **On-axis crossings:** the line now turns by q around the slit's edge. Make it **as round and hard as the space allows**: fillet ≥ 0.4–0.5 mm (the index sideways slit has 0.4 mm), or glue in a short **brass or steel tube eyelet** (e.g. 1.5 mm OD / 0.9 mm ID). This is the single biggest friction and wear point in a curled finger (section 2.4).
- The alternative from the 2026-09-27 log, an **idler pulley on the axis** (pass-through strands wrap it, flex on one side and ext on the other): no sliding friction at all, but a **linear coupling** (distal joint moves ρ/r_d × q). Software can undo that with a coupling matrix. Keep it in mind for V2.

---

## 6. Compliance and safety

**Why compliance helps grasping:**
- **Position errors become gentle forces.** With a rigid hand, a 1 mm error in object position means crushing force or a miss. A compliant finger conforms.
- **Impacts** (bumping the table, a dropped object) don't break gears, tendons or prints.
- **Force sensing for free:** if you know the stiffness k and measure the deflection, F = k·Δx. That's the **series elastic actuator** idea (Pratt & Williamson, IROS 1995).
- **Stable contact:** a stiff position controller on a stiff object chatters or builds up force. A compliant one settles.

**Cheap ways to get it (from cheapest to dearest):**
1. **Soft pads and skin** (TPU / silicone): local compliance where contact happens (section 7).
2. **Servo settings:** lower the P gain (register 21) and set the **torque limit** (register 16) per joint. That's "software compliance": the servo stops pushing at a set torque. Combined with the **present load** reading (register 60, PWM duty, not current), it gives crude grasp-force control. It costs nothing.
3. **Choosing the line's stiffness:** nylon *is* a series spring (K_joint ≈ 0.1–0.17 N·m/rad). But it's uncontrolled (humidity, creep).
4. **A defined spring** in one strand (section 3.4) or in both (a true SEA). Needs a joint-angle sensor to turn deflection into force.
5. **Joint-angle sensing** at the finger (hall sensors, or vision from the teleop camera) plus the servo's own position: deflection = r_s·φ − r_d·q, which gives the tendon force. That makes the soft line into a force sensor.

**Recommendation:** stiff braid (predictable) + soft pads + servo torque limits for V1. Then add joint-angle sensing and deliberate springs in V2, when force-controlled grasping (RL) needs it.

**Safety:** the torque limit + the overload cut-off + a stall-to-break safety factor ≥ 3 on the line mean the hand can't hurt itself or a person much at SCS0009 forces. Keep that safety factor when moving to stronger servos. At XC330 forces a snapped line whips.

---

## 7. Printing materials and pads

| Material | Stiffness | Toughness | Heat (softens at) | Creep | Printing | Use in Tendra |
|---|---|---|---|---|---|---|
| **PLA** | High (E ~3.5 GPa) | Brittle | ~55–60 °C | High under sustained load | Easy | Prototypes, test fits |
| **PETG** | Medium (~2 GPa) | Tough, less brittle | ~75–80 °C | Medium | Easy (stringy) | **V1 fingers, palm and forearm** |
| **PA / nylon (PA12, PA6), unfilled** | Medium | Very tough, wears well | > 100 °C | Medium | Must be dry; enclosure helps | Spools, drums, bushings |
| **PA-CF / PETG-CF** | Very high | Good | High | Low | Hardened nozzle | Structure only. **The carbon fibres are abrasive: never let a tendon slide on CF parts.** |
| **ASA / ABS** | Medium | Good | ~95–100 °C | Medium | Warps, needs an enclosure | Alternative to PETG |

Servos in the forearm get warm, and a hand left in a car or in the sun easily exceeds 55 °C. **PLA spools and palms can creep or deform there.** Use PETG as the default for V1.

**Pads and skin:**
- **TPU 95A** printed pads: easy and durable, but μ is only about 0.5–0.7 (verify).
- **TPU 85A** or softer: grips better, harder to print.
- **Cast silicone** (e.g. Smooth-On Dragon Skin 10–20 or Ecoflex, Shore 00-30 to 20A): **μ ≈ 1 or more** and conforms well. Wears and tears faster. Cast it in printed moulds over a textured PETG core so it keys in mechanically.
- **Shape matters too:** a rounded pad a few mm thick widens the contact patch, which raises friction torque (resisting twisting), not just sliding friction. Humans use exactly this.
- Worked example above: silicone instead of PLA contact cuts the needed pinch force about **3×**.

---

## 8. Bench experiments to run (cheap, 1 afternoon each)

1. **Capstan μ test.** Loop the line through a 1 × 2 mm PTFE tube bent to a known angle (90°, 180°, 360° around a Ø30 mm printed form). Hang a mass m, and pull the other end slowly with a luggage scale. μ = ln(F_pull / mg) / θ. Repeat for nylon and braid, PTFE and PLA, and a steel pin. **This fixes every η in section 2.**
2. **Stretch test.** Hang 1, 2, 5 kg on a 500 mm strand and measure the length change. EA = F·L/ΔL.
3. **Creep test.** Hang 1 kg (≈ 25 % of the servo's stall tendon force) for 7 days and measure daily. Compare nylon, braid and Vectran.
4. **Friction clamp test** of the split spool: 5 kg on one strand, 1 hour, check the witness mark.
5. **Test finger:** one index plus its palm section. Measure the force at the tip (kitchen scale) vs the servo command and the torque limit, straight and curled. Compare the plain split spool with the extensor-spring version (section 3.4).
6. **Pluck-tension calibration:** measure pitch vs known hanging weights once. After that the phone app is your tension gauge.

Log the results in `research/log.md` and update the η, EA and μ assumptions in this note.

---

## 9. Recommendations for Tendra V1 (prioritised)

Cheap and high-impact first.

| # | Change | Cost | Impact | Why (section) |
|---|---|---|---|---|
| 1 | **Switch the tendon to braided UHMWPE, 50 lb, 0.36 mm (PowerPro / Sufix 832; 40 lb = 0.33 mm), pre-stretched** | ~€15 per spool | ★★★ | ~15× stiffer than nylon mono, lower μ, no water uptake (1) |
| 2 | **The line touches only PTFE, polished steel or its own drum.** Steel dowels at every bare bend > 20°, brass eyelets or big fillets at the on-axis slits, chamfered tube mouths | ~€10 | ★★★ | 2× force at distal joints in curled poses, less wear and stick-slip (2, 5) |
| 3 | **Silicone (or soft TPU) fingertip and palm pads** | ~€20 | ★★★ | ~3× less force needed to hold things (4.4, 7) |
| 4 | **Anchors: 1–1.5 extra wraps around a post before each knot**, chamfered tie holes, Palomar knot + CA | free | ★★ | The knot sees ~15 % of the load (1.4) |
| 5 | **Split spool with friction clamp** (continuous tension adjustment) + witness marks; pretension 3–5 N, checked by pluck pitch | free (print) | ★★ | Braid needs fine tension; low T0 limits friction and creep (3) |
| 6 | **Outboard support for each spool** (bearing or bushing in the forearm frame opposite the servo) | ~€0.30 per joint | ★★ | Both strands side-load the servo shaft (up to ~43 N) (3.1) |
| 7 | **Servo torque limit and lower P gain** per joint; watch present load | free (firmware) | ★★ | Compliance, safety, crude force control (6) |
| 8 | **PETG** (not PLA) for everything structural; unfilled PA for spools if available; never CF where a tendon slides | ~same | ★★ | Heat, creep, toughness (7) |
| 9 | **Pins:** steel dowels or bearings at the finger joints, phalanges printed on their side | ~€10 | ★★ | Smoothness, no wear-out (5) |
| 10 | **Run the capstan, stretch and creep tests** before finalising route angles and η (section 8), and compute the exact route turning (script in 2.3) | an afternoon | ★★ | Replaces the assumptions here with measurements |
| 11 | **MCP drums r 7.5–8 mm, spool r 5 mm** (G ≈ 1.5) on the fingers; keep the thumb cmc_rot as it is | CAD rework | ★★ | ~1.5× fingertip force; update `servo_per_joint`, the router and the sim (4.4) |
| 12 | **Prototype the stiff-flexor / spring-extensor loop** on the test finger | small | ★ (maybe ★★) | Self-tensioning, tolerant of geometry errors (3.4) |
| 13 | **Test one HLS3606M** (checked: 0.59 N·m, current mode, 27.5 mm tall, 25T spline; used by Aero Hand Open) and plan V2 slots for a 20 × 34 mm servo class (XL330 / XC330) | research | ★★★ long term | The only way to useful force; also removes pot wear (100 k cycles) and adds current sensing (4.5) |
| 14 | V2: rolling contact joints with dislocation, on-axis idler pulleys + a software coupling matrix, joint-angle sensors for force estimation | big | ★★ | (5, 6) |

---

## Sources

From the repo (checked against sources on 2026-09-28; see `research/experiments/2026-09-28-full-hand/research.md` for links):
- Feetech SCS0009 specification, ed. A/0, 2020-11-23 ([Seeed mirror](https://files.seeedstudio.com/products/Feetech/SCS0009-Specifications.pdf)): torque, speed, gears, overload protection, pot life, register map.
- ORCA hand, [arXiv 2504.04259](https://arxiv.org/abs/2504.04259): tendons touch only metal and PTFE, ratchet spool, braided line, XC330 servos, > 10 000 cycles.
- Shadow Dexterous Hand motor unit docs ([link](https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/stable/user_guide/md_motor_unit.html)): one spool per joint pair, split spool, bar at the spool exit.
- Antagonistic Bowden hand, [arXiv 2512.24657](https://arxiv.org/html/2512.24657v1): 1/2 mm PTFE, coated steel, keyed bobbins, flex/ext length matching.
- MM-Hand, [arXiv 2604.17245](https://arxiv.org/html/2604.17245): PTFE tube quality matters; feeder-tube PTFE and spring tubes better.
- DexHand ([repo](https://github.com/TheRobotStudio/V1.0-Dexhand)): Sufix 832 braid.
- PLA friction: [Rapid Prototyping J. 2022](https://www.emerald.com/insight/content/doi/10.1108/rpj-03-2022-0081/full/html).

Literature quoted from memory (**verify** before relying on exact numbers):
- M. Kaneko, T. Yamashita, K. Tanie, "Basic considerations on transmission characteristics for tendon drive robots", ICAR 1991.
- G. Palli, G. Borghesan, C. Melchiorri, "Modeling, identification, and control of tendon-based actuation systems", *IEEE Trans. Robotics* 28(2), 2012.
- J. K. Salisbury, J. J. Craig, "Articulated hands: force control and kinematic issues", *IJRR* 1(1), 1982 (N+1 tendons).
- S. C. Jacobsen et al., "Design of the Utah/MIT dextrous hand", ICRA 1986 (2N tendons).
- M. Grebenstein et al., "The DLR hand arm system", ICRA 2011 (antagonistic, variable stiffness, Dyneema).
- J. R. Cannon, L. L. Howell, "A compliant contact-aided revolute joint", *Mechanism and Machine Theory* 40(11), 2005 (CORE rolling joint).
- A. M. Dollar, R. D. Howe, "The highly adaptive SDM hand", *IJRR* 29(5), 2010.
- G. A. Pratt, M. M. Williamson, "Series elastic actuators", IROS 1995.
- V. Mathiowetz et al., "Grip and pinch strength: normative data for adults", *Arch Phys Med Rehabil* 66(2):69–74, 1985.
- DSM Dyneema fibre datasheets (SK75/SK78/DM20); Vectran (Kuraray) datasheets; ROBOTIS e-manual for XL330-M288 / XC330-T288; Feetech STS3032 / STS3215 datasheets; PowerPro and Sufix line catalogues (diameters and break loads).

Standard relations used: capstan (Euler–Eytelwein), Hooke (k = EA/L), Mersenne (string frequency), bolt torque-preload (T = K·F·d, K ≈ 0.2).

Sources added in the 2026-09-30 fact-check:
- Sufix 832 size table: https://www.rapala.com/us_en/832-advanced-superline ; PowerPro size table: https://www.tacklewarehouse.com/Power_Pro_Spectra_Braided_Line_Moss_Green/descpage-PPSL.html
- Line stretch at 1/3 break load (PowerPro braid 0.7–1 %, Berkley mono 2–9 %): https://www.fishtalkmag.com/blog/fishing-line-stretch-test-stretching-truth
- Feetech SCS0009 datasheet A/0: https://files.seeedstudio.com/products/Feetech/SCS0009-Specifications.pdf ; HLS3606M datasheet: https://www.feetechrc.com/Data/feetechrc/upload/file/20240807/6385862508379674057566108.pdf ; STS3032: https://www.feetechrc.com/6v-45kg-magnetic-code-360-degree-serial-bus-steering-gear.html
- ROBOTIS XL330 / XC330: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/ , https://www.robotis.us/dynamixel-xc330-t288-t/
- Mathiowetz et al. 1985 tables: https://klyonsot2013.wordpress.com/wp-content/uploads/2013/11/grip-pinch-strength-norms.pdf (PubMed: https://pubmed.ncbi.nlm.nih.gov/3970660/)
- ORCA (braided nylon, ratchet spool, poppable pin joints): https://arxiv.org/html/2504.04259 ; RUKA (200 lb braided line): https://arxiv.org/html/2504.13165 ; Aero Hand Open (Kevlar/Vectran): https://arxiv.org/html/2608.28578 ; Halodi patent (Vectran cables): https://patents.google.com/patent/WO2018149499A1/en

Fact-checked 2026-09-30 (partial): braid diameters (corrected), line stretch, SCS0009 gear material and pot life, servo torques/sizes/prices (STS3032, HLS3606M, SCS2332, XL330, XC330), human pinch/grip norms (corrected), ORCA joint type (corrected: not rolling contact), tendon choices of ORCA/RUKA/Aero/Halodi. Not checked: friction coefficients, fibre moduli, knot efficiencies, Kaneko/Palli/Salisbury references.
