# 05 · Contact surfaces and sensor-ready mechanics

Study 2026-11-14, agent 05. Everything that touches the object (fingertip shape, pads, skins) and how to make the printed parts ready for sensors (magnetic fingertip, joint angle sensors, wires, palm camera).

Code and outputs (all mine, nothing outside this folder was changed):
- `cad/05_contact_calc.py` -> `out/05/contact_calc.txt` (friction, squeeze force, Hertz patch, stiffness chain, magnet field, wire strain)
- `cad/05_fingertip.py` -> `out/05/05_*.step/.stl` + `05_fingertip_summary.json` (sensor-ready tip, snap-in skin, 2-part skin mould, reference PCB/magnet/shim). Run: `uv run --no-project --with cadquery --with numpy python cad/05_fingertip.py` (4 s).

Terms: **μ** = friction coefficient (friction force = μ × squeeze force). **Shore A** = rubber hardness (higher = harder). **Taxel** = one touch-sensing point. **Hall sensor** = chip that measures a magnetic field. **Diametric magnet** = disc magnetised across its diameter (north on one side of the disc, south on the other), so it makes a rotating field when it spins. **Service loop** = extra slack in a cable so it bends gently instead of stretching.

Assumptions are marked **[A]**. Values from memory of datasheets or papers (not re-fetched this session) are marked **[bg]**; verify before you buy or print.

---

## 1. Top 3 recommendations

**1. Put a cast silicone skin (Shore A 10-30, e.g. Dragon Skin 20) on every fingertip and on the palm before changing anything else. It is the cheapest strength upgrade in the whole hand.** The hand's fingertip force is only 1.7 N sustained / 3.4 N stall. To hold a vertical weight with a two-finger pinch you need N = 1.5 × W / (2μ) per tip. With bare PETG (μ ≈ 0.3) a 3.4 N tip holds **139 g**. With TPU 95A (μ ≈ 0.6) **277 g**. With silicone A40 (μ ≈ 1.0) **462 g**, and with Dragon Skin 10 (μ ≈ 1.3) **601 g** [A, μ values in section 3]. That is 3.3-4.3× less squeeze for the same object, and it costs about €0.60 of silicone per tip. A 500 g object needs 12.3 N per tip on PETG (impossible for this hand) but 2.8-3.7 N on silicone (just possible). The skin must be **replaceable and mechanically locked** (snap-in plug with a 45-degree undercut, no glue), because published silicone skins on hobby hands wear out after ~2 000-4 000 cycles (ORCA, from `sensing.md`).

**2. Make the distal phalanx "sensor-ready" now, even if the chip comes later: a stepped pocket on the palm face (PCB + 1 magnetic chip), a 2.6 mm cable bore towards the DIP axis, and a skin that carries one 3 × 1.5 mm magnet.** I built it (`05_tip_body`, `05_skin`): the pocket costs no outer size (same envelope as the V2 tip: palm face at y -5.8, skin surface at y -8.8), side walls stay ≥ 1.88 mm, the end wall is 3.5 mm, and the chip sits 3.9 mm under the magnet. A D3 × 1.5 mm N45 magnet gives ~20 mT at 3.9 mm and the field rises by ~17 mT per 1 mm of skin compression (hand calc), which a MLX90393 resolves to ~0.06 mm or ~0.05 N per mT with a Dragon Skin dome [A, to be calibrated]. One chip + magnet is **about €6 per fingertip, 4 wires (I2C) per finger**.

**3. Route the sensor cable through each joint along the joint axis (inside a hollow pin) and use ONE 4-wire I2C bus per finger for the tip chip and the joint angle chips.** A cable on the axis only twists. A 1 mm bundle twisted 95 degrees over 10 mm sees 0.34 % strain per conductor (over 20 mm: 0.09 %), which is safe for millions of cycles. A flex cable looped round a 3 mm radius sees 1.7 % strain, which is what snapped the ORCA hand's 0.2 mm copper wires after 4 500-7 000 cycles. For joint angles use a TMAG5273 (SOT-23, 2.9 × 1.6 mm, €1-2) with a diametric magnet on the pin end. The price is **+2.2 mm width at the joint on one side**, which collides with the ~2 mm gap between V2 fingers, so I also give a no-extra-width variant (magnet in the drum flange, chip on the cheek inner face) that must be bench-tested first.

One finding that affects all of this: **pads cannot act as a force filter at the fingertip** (section 4). The tendon is 30-1000× softer than any pad. Soft pads help with friction, contact patch and wear, not with spikes. A thick soft pad helps only at the **palm** against a stiff arm.

---

## 2. Idea list (wide)

Gain column: numbers from `out/05/contact_calc.txt` unless noted. Verdict: **BUILD** = in this study's plan, **TEST** = bench test first, **LATER**, **NO**.

### 2a. Fingertip shape

| # | Idea | How it works | Gain | Cost / effort | Risk | Verdict |
|---|---|---|---|---|---|---|
| 1 | Human-like flat, slightly crowned pad (R 25 across the width, 3 mm thick) | Flat contact patch pinches boxes, cards, bottle caps; crown lets it roll on round things | Largest patch for a given squeeze (Hertz, below); matches V2 envelope | Free (CAD) | Needs soft skin to work | **BUILD** |
| 2 | Round ball tip | Always one contact point | Pushes buttons well | Free | Poor for flat objects, patch ≈ 0.2-1 mm radius on hard materials | NO as default |
| 3 | Pad offset: pulp 3.0 mm proud of the bone face (V2 had 0.8 mm proud of the outline; human pulp ≈ 4-5 mm [bg]) | Soft layer between bone and object | More deformation = bigger patch, space for sensor gap | 0 | Raises thumb-finger collisions slightly | **BUILD** (3.0, same as V2 envelope) |
| 4 | Fingernail / scoop shim: 0.15 × 9 mm stainless sheet in a slot, 1.5 mm proud of the distal end at the palm-face plane | Hard thin edge slides under a coin or card on a table, then pad pinches against thumb | Makes flat-on-table pick-up possible (otherwise impossible with 4 mm-thick soft tip) | €0.30 (shim stock) | Edge snags; keep corners rounded | **BUILD** (slot is in the CAD) |
| 5 | Tip width 16.5 mm (human index) vs a swappable 11 mm "tweezer" tip on 2 screws | Narrow tip reaches into gaps | Precision tasks | One more part | Two parts to keep | LATER |
| 6 | Pad rolled 10 degrees toward the thumb | Pinch pads meet face to face when the thumb opposes | Better pad overlap in key pinch | CAD only | Needs kinematic check | TEST in sim |
| 7 | Fingerprint ridges, 0.3 mm pitch 0.6 mm, moulded into the skin | Ridges raise grip on oily/wet surfaces and make slip vibration measurable | +10-30 % friction on smooth glass [bg, uncertain] | Free in the mould (needs a textured cup) | Ridges print poorly at 0.4 mm nozzle; use resin print or sanding | LATER |
| 8 | Shorter DIP boss: the DIP has **no drum** (bar-coupled), but V2's distal body still starts 7.85 mm from the axis (inherited drum flange radius) | If the boss radius could drop to ~5 mm (check the bar plates), the tip body gets +2.85 mm length (15.2 -> 18 mm) | Room for a second chip (contact location) or longer nail | CAD rework | Linkage plate clearance unknown | **TEST** (ask joints-structure agent) |

### 2b. Pads and skins

| # | Idea | How it works | Gain | Cost / effort | Risk | Verdict |
|---|---|---|---|---|---|---|
| 9 | Cast silicone snap-in skin, Shore A 10-30 | Plug with 45-degree undercut snaps into a pocket; replaced in seconds | 3.3-4.3× less squeeze (section 3) | ~€0.60 per skin; 2-part printed mould | Needs bench pull-out test | **BUILD** |
| 10 | Printed TPU 95A pad with 0.8 mm grooves | Prints on a normal printer, no mould | 2× vs PETG (μ ≈ 0.6-0.7 [bg]) | €0.20 | Lower μ, no sensing | Baseline / fallback |
| 11 | TPU 85A printed pad | Softer, grips more | μ ≈ 0.8, ~2.7× vs PETG | €0.20 | Hard to print, stringing | TEST |
| 12 | Silicone over-moulded on a thin TPU core | TPU gives the shape and the anchor, 1 mm silicone gives grip | Durable + grippy | Two steps | Silicone does not bond to TPU without primer | NO (use snap-in) |
| 13 | Replaceable snap shell (dovetail slide, side detent) | Skin slides in from the side | Easier than a push-in plug | Tip walls are too thin for a dovetail track (8.28 half-width, 1.88 mm wall) | Wall too thin | NO for tips, OK for the palm |
| 14 | Phalanx pads (proximal 1.5 mm, middle 1.2 mm in V2) with **rivet-lock holes**: 2-3 Ø2 holes through the PETG recess, silicone flows through and forms a mushroom head | Mechanical retention without glue | Pull-out from the head, not from adhesion | Mould-in-place (open cup) | Silicone must be poured onto the part | **BUILD** for phalanges |
| 15 | Palm: soft thick pad, 6-10 mm Ecoflex 00-30, printed TPU lattice as a cage | Cushions against a stiff arm | 25 N -> 9 N for 0.5 mm arm error (section 4) | €5 | Heavy (≈ 20 g) | **BUILD** (palm only) |
| 16 | Tacky ultra-soft gel (Ecoflex 00-10, Slacker-softened) | μ > 1.5, adhesion | Smooth glass and metal | €0 extra | Collects dust and hair | TEST on the thumb pad |
| 17 | Gecko-style micro-wedge pad | Adhesion by van der Waals | Adhesion on flat smooth surfaces, μ > 2 [bg] | Hard to make at home | Dust, research-grade | NO |
| 18 | Whole-hand thin glove | Hides gaps, adds friction everywhere | Protects tendon openings from dirt | €5 | Stretches over joints and resists bending | LATER |

### 2c. Passive compliance

| # | Idea | Gain | Verdict |
|---|---|---|---|
| 19 | Soft fingertip pad as a force filter | **~0 %** at the fingertip, because the tendon is already 30-1000× softer than the pad (section 4) | NO as a design reason |
| 20 | Thick soft palm / heel pad against the arm's position error | 25 N -> 9.1 N (10 mm Ecoflex 00-30, 40 × 60 mm) or -> 3.2 N on a 20 × 30 mm phalanx-sized pad | **BUILD** |
| 21 | Skin as a spring for sensing (it is the sensor's mechanical transducer) | Gives 17 mT per mm of compression, i.e. force reading | Part of #22 |

### 2d. Sensor-ready parts

| # | Idea | How it works | Gain | Cost | Risk | Verdict |
|---|---|---|---|---|---|---|
| 22 | Magnetic tip: 1 × MLX90393 or TMAG5273 + 1 magnet in the skin | Skin compression moves the magnet; chip reads Bx, By, Bz = normal + shear force | Force + slip on every tip, €6/tip | €6 | Crosstalk from joint magnets (keep ≥ 10 mm or calibrate) | **BUILD** |
| 23 | AnySkin-style magnetised-particle skin (NdFeB powder in silicone, magnetised after curing) | No magnet to align; the skin swaps in 12 s [confirmed in `sensing.md`] | Zero-shot transfer between skins | Needs a magnetiser (pulse) | Needs 5-chip board 20 × 20 mm: too big for this tip | LATER, needs bigger tip |
| 24 | Barometer (BMP390) potted in silicone | Pressure through the rubber; sensitive, good for vibration/slip | €2, but only normal force | €2 | Sealing, 2 I2C addresses | LATER (second option) |
| 25 | Capacitive pre-touch: copper pads on the same PCB under the silicone, read with an FDC1004 or MPR121 | Senses an approaching conductive/wet object 5-10 mm away | Gives the controller a "touch about to happen" signal | €1.5 | Only works for conductive/wet objects | TEST |
| 26 | Time-of-flight chip in the palm (VL53L1X, 4.9 × 2.5 × 1.6 mm) pointing at the grasp zone | Distance to the object, 4 mm to 4 m | Cheap depth for reach and grasp; €4 | €4 | Hand closes in front of it | **BUILD** (palm pocket) |
| 27 | Fingertip camera (DIGIT/9DTact) | Gel + camera, 10^5+ taxels | Best tactile data | Too big (DIGIT ~20 × 27 mm) | Tip is 16.5 × 15 mm | NO now |
| 28 | Hall angle sensor on the joint pin (diametric magnet + TMAG5273) | 12-bit angle, 0.088 degrees | Measures the **joint**, not the spool: removes tendon stretch from the loop | €2 + magnet €0.5 | +2.2 mm width, section 5 | **TEST** |
| 29 | Hall arc-sweep: small magnet in the drum flange, 3D Hall on the cheek inner face | No pin end needed, no extra width | Calibrated lookup, ~0.3 degrees [A] | €2 | Needs flange-to-cheek gap | **TEST** |
| 30 | Cable via hollow joint pin, torsion only | 0.09-0.34 % strain | Lasts > 10^6 cycles | Pin is a Ø4 × 3 mm tube instead of solid | Pin slots weaken it | **BUILD** (bench test first) |
| 31 | Flex-PCB service loop | Cable bends around R ≥ 10 mm | 0.5 % strain at 0.1 mm / R 10 | Needs custom flex PCB (~€8) | Needs room for R 10, which does not fit a finger joint | NO |
| 32 | Slip-ring contacts on each pin | No wires at all | Infinite life | Tiny contacts, noisy | Wear, 4 contacts per joint | NO |
| 33 | Palm camera in the heel | Close view of the grasp zone (Figure 03 idea) | See `sensing.md` | €10-15 | Needs a window and a thin cable | **BUILD** (mount) |

---

## 3. Deep dive A: friction, and what the pad material is worth

### μ values used [A, dry, against smooth household objects: glass, painted metal, ABS, cardboard]

| Pad | μ | Source |
|---|---|---|
| PETG / PLA, bare | 0.30 | `tendon-mechanics.md` §7 |
| TPU 95A, smooth | 0.60 | same file says 0.5-0.7 |
| TPU 95A, 0.8 mm grooves | 0.70 | [A] |
| TPU 85A | 0.80 | [A] |
| Silicone A40 | 1.0 | `tendon-mechanics.md` §7 says "silicone μ ≈ 1" |
| Dragon Skin 10 (A10) | 1.3 | [bg/A] |
| Ecoflex 00-30 | 1.6 | [bg/A] |

μ of rubber depends on speed, dust, moisture and surface roughness, and often drops 30-50 % on wet or oily objects. Treat these as a ranking, not as design constants. **Bench test 1 measures them.**

### Squeeze force per fingertip (N) to hold a vertical weight, safety factor 1.5

`N = 1.5 × m × 9.81 / (n × μ)`, n = number of contacts (2 for a pinch, 3 for a tripod, ~6 effective for a power wrap with thumb, four fingers and palm). Full table in `out/05/contact_calc.txt`.

| Grasp, mass | PETG 0.3 | TPU 95A 0.6 | Silicone A40 1.0 | Dragon Skin 10 1.3 | Ecoflex 00-30 1.6 |
|---|---|---|---|---|---|
| 2-finger pinch, 200 g | 4.9 | 2.5 | 1.5 | 1.1 | 0.9 |
| 2-finger pinch, 500 g | 12.3 | 6.1 | 3.7 | 2.8 | 2.3 |
| Tripod, 500 g | 8.2 | 4.1 | 2.5 | 1.9 | 1.5 |
| Power wrap, 500 g | 4.1 | 2.0 | 1.2 | 0.9 | 0.8 |
| Power wrap, 1 kg | 8.2 | 4.1 | 2.5 | 1.9 | 1.5 |

Reading it against the hand: stall tip force 3.4 N, sustained 1.7 N (README), ~2.5 N after friction (V2 review). **Maximum mass a two-finger pinch can hold** (same formula turned around):

| Tip force | PETG | TPU 95A | Silicone A40 | Dragon Skin 10 |
|---|---|---|---|---|
| 1.7 N (sustained) | 69 g | 139 g | 231 g | 300 g |
| 3.4 N (stall) | 139 g | 277 g | 462 g | 601 g |
| 9 N (HLS3606M-class, future) | 367 g | 734 g | 1 223 g | 1 590 g |

So a silicone-skinned SCS0009 hand does the work of a 3-4× stronger PETG hand for the grasp that matters most, a bottle or mug. A power wrap of a 1 kg object needs only ~2.5 N per contact on silicone, which the hand can do.

### Contact patch and twist resistance (Hertz, R 8 mm tip on a flat object) [hand calc, thin-pad limits ignored]

| Pad | Patch radius at 1 N | Twist capacity at 1 N | at 3 N |
|---|---|---|---|
| Ecoflex 00-30 (E ≈ 0.12 MPa) | 4.2 mm | 2.8 N·mm | 12.2 N·mm |
| Dragon Skin 10 (0.26 MPa) | 3.3 mm | 2.2 N·mm | 9.4 N·mm |
| Silicone A40 (1.6 MPa) | 1.8 mm | 1.2 N·mm | 5.1 N·mm |
| TPU 95A (26 MPa) | 0.7 mm | 0.3 N·mm | 1.2 N·mm |
| PETG | 0.2 mm | 0.07 N·mm | 0.3 N·mm |

Twist capacity = how much torque the pinch resists before the object rotates about the pinch axis (e.g. a pen or a key). Dragon-Skin-class pads resist ~8× more twist than TPU at the same squeeze. (Hertz only holds while the patch is smaller than the pad thickness, so the 6 mm radii are an upper bound; a 3 mm pad bottoms out.)

### Material pick

- **Default: Smooth-On Dragon Skin 20 (A20)** [bg: Dragon Skin 10/20/30 and Ecoflex 00-30/00-50 are the usual choices; verify the datasheet and pot life before ordering]. Middle ground between grip (μ ≈ 1.2) and wear. 1 lb kits are ~€35; a skin is 0.74 g, so one kit makes > 500 skins.
- **Thumb pad and testing:** Dragon Skin 10 or Ecoflex 00-30 (stickier, picks up dust).
- **Durable alternative:** Smooth-Sil 940 or 950 (A40-50) for the palm and the thumb's side.
- Dust: dust the skin lightly with talc if it picks up lint.

### Mould and casting (all in the CAD)

`out/05/05_mould_cup.stl` (25 × 25 × 8 mm) + `05_mould_cover.stl` (25 × 25 × 5.8 mm), printed parting face up in PETG at 0.12 mm layers.
- The **cup** holds the pad face (outer surface, crown R 25). The **cover** forms the plug with its undercut and carries a 2.8 mm steel dowel (a drill-bit shank) that makes the magnet hole.
- A Ø4 pour hole at the proximal end, a Ø1.2 vent at the distal end, and four Ø1 vents over the pad corners. Four 1.75 mm filament pins (Ø1.9 holes) align the halves; clamp with 2 rubber bands or M3 bolts.
- Release: Ease Release 200 or a thin PTFE spray [bg]. Platinum-cure silicone (Dragon Skin, Ecoflex) is poisoned by sulphur and some resins; PETG and PLA are fine, **do not use resin-printed or sulphur-vulcanised parts**.
- Pad surface finish: the crown prints as stair-steps (0.12 mm). That is fine for the skin (a small texture even helps grip), or sand the cup with 400 grit.
- Cure: Dragon Skin 20 cures in 4 h at room temperature [bg]; Ecoflex 00-30 in 4 h.
- Silicone volume 0.69 cm³ per skin (0.74 g). Mix 5 g, cast 4 skins per batch.
- Press the magnet in from the plug side after demoulding (Ø2.8 hole, Ø3.0 magnet, stretches the rubber; 2.4 mm deep leaves 0.6 mm silicone over it). Close the hole with a dab of silicone adhesive (Sil-Poxy).

### Snap-in retention: numbers and failure modes

The plug has a 45-degree barb: opening half-width 4.8 mm, widest 6.4 mm, bite 1.6 mm per side, plug 8.9 mm long. To push it in the opening has to stretch/compress the barb from 12.8 mm to 9.6 mm (25 % strain), easy for A10-A30 (elongation > 300 % [bg]) and the tip stays printable (the undercut is a 45-degree overhang when the palm face is printed upward).

What fails first: the skin tears at the barb root if it is pulled out sideways. Mitigations: 0.6 mm fillets on the plug corners (in the CAD), A20 or harder for the first test, or a 0.1 mm nylon mesh in the plug (casting-in). **The pull-out force target is > 10 N** (a pad carrying 3 N of friction at 3 N squeeze must not peel; 3× margin). **Bench test 2** measures it.

---

## 4. Deep dive B: do soft pads filter force spikes?

Short answer: **not at the fingertip, yes at the palm.**

Series stiffness chain at the fingertip, tendon k = EA/L with 0.4 mm line (A = 0.126 mm²), L = 0.4 m, reflected to the tip by (drum 7 mm / lever 60 mm)² = 0.0136 [A]:

| Tendon | Tendon k | At the tip | Pad k (10 × 10 mm slab, E A / t) | Result |
|---|---|---|---|---|
| Nylon, E ≈ 3 GPa | 0.94 N/mm | 13 N/m | PETG 8 × 10^7, TPU 1.3 × 10^6, DS10 3 mm 8.7 × 10^3, Ecoflex 5 mm 2.4 × 10^3 N/m | Pad changes the total by < 0.1 % |
| Dyneema braid, E_eff ≈ 40 GPa | 12.6 N/mm | 172 N/m | same | 168-160 N/m (-2 to -7 %) |
| Ideal stiff drive | 100 N/mm | 1 361 N/m | same | Ecoflex 5 mm lowers it to 869 N/m (-36 %), force from 0.3 mm tip jitter 0.41 -> 0.26 N |

Impact peak `F = v √(k m)` with 50 g effective mass at 0.1 m/s [A]: 0.08-0.82 N across all cases and pads, so even a 0.1 m/s tap is below the tip's own 3.4 N limit. Conclusion: the **tendon is the spring**, a fingertip pad adds nothing until the drive is stiffer than ~50 N/mm (a very stiff, short, steel-cable hand; not this design). This also means a fingertip's static compliance is large: at 12.6 N/mm a 45 N strand stretches 3.6 mm, ≈ 30 degrees of joint error at the 7 mm drum. That is why joint-angle sensors (section 5) matter more than pads for control, and it agrees with the V2 review's "8 mm of sheath compression at 45 N".

**The palm is different.** The arm is stiff in position (assume k_arm ≈ 50 N/mm at the hand [A]; real value unknown, measure it), so a 0.5 mm arm error against a table or an object is 25 N. A soft pad in series reduces it:

| Palm pad | Pad k | Force for 0.5 mm arm error |
|---|---|---|
| PETG or TPU 95A 2 mm | > 30 000 N/mm | 25.0 N |
| Dragon Skin 10, 3 mm, 40 × 60 mm | 208 N/mm | 20.2 N |
| Ecoflex 00-30, 10 mm, 40 × 60 mm | 28.8 N/mm | **9.1 N** |
| Ecoflex 00-30, 10 mm, 20 × 30 mm | 7.2 N/mm | **3.2 N** |

Recommendation: 8-10 mm soft pads on the palm heel and on a phalanx-sized zone if the arm shakes against an object (see `research/ai/smooth-control.md`). Keep the fingertip pad thin (3 mm) for sensing. Cost: 10 mm × 40 × 60 mm of Ecoflex is 24 cm³ = 26 g for the palm pad.

---

## 5. Deep dive C: sensor-ready tip, wiring, joint angle sensors

### The tip I built (`05_tip_body`, `05_skin`, `05_ref_*`)

Frame: finger-local, DIP axis at the origin, +z toward the tip, palm face at y = -5.8 (after V2's 2.2 mm pad pocket), skin outer surface at y = -8.8.

```
 side view, dorsal (nail side) at the top, palm face and skin at the bottom, tip end at the right

 y=+8  ______________________________________
      |                                       \   z=23
      |     cable bore (Ø2.6)    cavity        )   tip body 16.56 wide, 15.15 long, 13.8 tall
 y=-1 |  <=====================[ PCB + chip ]    )  (same envelope as the V2 tip)
      |                         \__undercut__/   |
 y=-5.8|___ palm face ____/ opening 9.6 \_________|__ shim slot (0.15 mm stainless, 1.5 mm proud)
      \  silicone skin: plug snaps into the undercut (widest 12.8), pad face at y=-8.8,
       \ crown R 25, magnet 0.6 mm under the surface, 3.9 mm above the chip
```

| Item | Value |
|---|---|
| Tip body | x ±8.28, y -5.8…+8, z 7.85…23, volume 2 715 mm³ (3.45 g PETG 100 %), dorsal round-over 2.0, plan corner radius 4.5 (V2: 6.0) |
| Cavity | 4.8 mm deep, opening ±4.8 mm, widest ±6.4 mm at 1.6 mm depth (45-degree overhang, printable face-up), z 10.0…19.5; min side wall 1.88 mm, distal end wall 3.5 mm |
| PCB | 9.2 × 9.0 × 0.8 mm (FR4, 1 chip + 0402 cap + 4 pads on the proximal edge); chip 3 × 3 × 1.0 mm QFN, top at 3.0 mm below the palm face |
| Cable bore | Ø2.6 mm along z at the PCB level, from the cavity (z 10) to the proximal face (z 7.85) |
| Skin | 690 mm³ (0.74 g), crown R 25, 3.0 mm at the centre (1.8 mm at the edges), plug bottom 0.7 mm above the chip |
| Magnet | N45 Ø3 × 1.5 mm, axial, bottom 3.9 mm above the chip top, 0.6 mm of silicone over it |
| Nail shim | 0.15 × 9 mm stainless, in a 5 mm deep slot, 1.5 mm proud |
| Checks (`05_fingertip_summary.json`) | tip valid; tip × PCB, chip, magnet, shim overlap 0.00 mm³; skin × tip, PCB, chip overlap 0.00 mm³; mould cup and cover valid |

What the V2 tip does not give: the whole assembly shares the 3 mm pad with the V2 geometry, so there is **no change to kinematics**. The tip body would replace the V2 `distal` tip from z = 7.85 outward; the fork, the DIP bar pin and the cheeks (z < 7.85) are not modelled and need merging.

**Field and sensitivity (hand calc, N45 magnet on axis):** D3 × 1.5: 36 mT at 3 mm, **20 mT at 3.9 mm**, 12 mT at 4.9 mm; D2 × 1: 7.5 mT at 3.9 mm; D4 × 1: 24 mT at 3.9 mm. Compressing 1 mm moves the magnet closer: +17 mT (D3 × 1.5). MLX90393 and TMAG5273 both cover ±40 mT or more in their coarser ranges [bg]; TMAG5273 A1 is ±40 mT, A2 ±80 mT. A 1 mT step is ~0.06 mm of skin motion, or ~0.045 N for a Dragon Skin 10 plug [A: E 0.26 MPa, 12 mm² loaded area, 4 mm tall]. Shear is read from Bx and By as the magnet moves sideways. All of this needs calibration with a scale (bench test 3); the figures here are first guesses.

**Cost per fingertip:** MLX90393 ~€4-5 [bg], PCB ~€1.5, magnet €0.3, silicone €0.6, shim €0.3, wires €0.5. About **€7**. Five tips: €35.

**Mass added to the distal:** PCB + chip 0.3 g, skin 0.74 g, shim 0.1 g.

### Wires across the joints

Five fingers, each with four chips (tip + 3 joint angle sensors) on **one shared I2C bus**: VCC, GND, SDA, SCL = 4 wires per finger. Addresses: MLX90393 gives 4 via two pins; the TMAG5273 has a programmable address register (`sensing.md`). A TCA9548A in the forearm splits the buses (one per finger) so the capacitance stays < 400 pF.

| Option | Strain per conductor | Life | Verdict |
|---|---|---|---|
| 4 × 36 AWG silicone wires (Ø0.8-1.0 mm bundle) through a hollow pin, **on the joint axis**; twisting only | 0.34 % at 10 mm free length, 0.09 % at 20 mm (95 degrees) | > 10^6 cycles [A: copper fatigue limit ~0.3-0.5 % for 10^6] | **Use**, with ≥ 10 mm of free bundle either side of the pin |
| Same wires, bent in a loop of R 3 mm | 0.8 % (7 × 0.05 mm stranded) to 2.1 % (single 0.127 mm) | 10^3-10^4 cycles (ORCA failed at 4 500-7 000) | NO |
| Rolled-annealed copper flex PCB 0.1 mm, loop R 3-10 mm | 1.7 % (R 3), 1.0 % (R 5), 0.5 % (R 10) | R 10 only; no room in a 16 mm-high phalanx | NO |
| Slip ring on the pin | none | wear, noise | NO |

How the hollow pin is built: Ø4 × 3 mm stainless tube instead of the solid Ø4 mm pin (hypodermic or K&S style tube [bg]; a Ø4 × 0.5 wall tube is 3.0 mm bore). The wire bundle (0.8 mm) enters through a Ø1.6 radial slot cut at the centre of the distal part, runs inside the tube along x, and exits through a slot in the middle phalanx. Two 1.6 mm slots in a 4 mm tube cost some bending strength; the pin carries almost only shear, so this should be OK but needs a bench check (bench test 4). The 2.6 mm bore in the tip body ends at the proximal face and has to be extended through the distal fork to the DIP pin; same for the middle phalanx and the PIP and MCP axes. This crosses the strand slots at the MCP and PIP (centre slot x ±1.4 mm holds the crossing strands) so the cable has to share that slot or go through the pin ends. **Check with the finger CAD owner.**

### Joint angle sensors: where and what it costs

Two options, ranked by room needed.

**H1: magnet in the pin end (classic).** Diametric magnet Ø3 × 2 mm (or Ø4 × 2) in a pocket in a brass pin end (0.5 mm wall), TMAG5273 or AS5600/MT6701 on a 5 × 5 mm PCB in a pocket on the parent cheek's outer face, 1 mm air gap. Needs **+2.2 mm width on one side of the joint** (1.2 mm pocket + 1.0 gap, the cheek is already a bearing seat). V2 fingers are 18 mm wide on a ~19.5-20 mm pitch, so the gap between fingers is ~2 mm. It fits only if the sensors face alternate sides, and for the little finger on the free outer side. Resolution: AS5600 12 bit = 0.088 degrees = 0.12 mm at an 80 mm lever; MT6701 14 bit = 0.022 degrees. Size of AS5600 SOIC-8 is 5 × 4 mm (too big on a breakout, fine as a bare chip). TMAG5273 SOT-23-6 is 2.9 × 1.6 mm. Cost €2 + magnet €0.5.

**H2: arc-sweep (no extra width).** A Ø2 × 1 magnet in a pocket on the child drum flange at radius ~6 mm; a TMAG5273 on the inner face of the parent cheek, 1.0-1.5 mm from the flange face. The magnet travels an arc of 10 mm over 95 degrees; the 3D Hall output (Bx, By, Bz) is monotonic along the arc and a 5-point lookup (or a polynomial) gives the angle. Needs a flange-to-cheek gap of ≥ 2.5 mm (magnet 1.0 + clearance 1.5) [A: unknown, V2 geometry must be checked]. Resolution ~0.3 degrees [A, depends on calibration]. No extra width, but each joint is calibrated once (the servo itself can do this: sweep the joint, record Hall vs servo angle).

**Crosstalk:** keep joint magnets ≥ 10 mm from the tip chip, or read with the tip magnet in a known spot and subtract (the README flags this as "test it"). Distance from a joint axis to the tip magnet on the distal is 15-22 mm, OK on paper. D3 × 1.5 at 15 mm gives ~1 mT, comparable to the skin signal of 1 mm compression (17 mT) but well below it. Joint magnets are the main disturbance for the tip, not the reverse; calibrate with all joints at the working angle.

**Priority** (from `sensing.md`): MCP flex and abduction first (largest load, longest tendon path), then PIP. The DIP is coupled by the bar to the PIP, so it needs no sensor. Per finger: 2-3 sensors (mcp_abd, mcp_flex, pip); thumb 4. Total 16 + 5 tip chips = 21 chips on 5 buses; cost ~€40 for joints + €35 tips.

### Palm sensors and camera

- **Palm camera mount** (Figure 03 idea): a 12 × 12 × 6 mm pocket in the heel pad, aimed 20 degrees toward the fingertips, with a 0.8 mm TPU window. Use a USB endoscope module (Ø7-8 mm, 640 × 480 or better, €10-15, straight to the PC; no ESP32 work). A 120-degree CSI module (OV2640/OV5640 at 8 × 8 mm) is the alternative, but needs a flat cable through the wrist. Keep the cable on the wrist axis like the tendon sheaths.
- **Time-of-flight chip** VL53L1X (4.9 × 2.5 × 1.56 mm, I2C) in a 5.5 × 3 × 2 mm pocket in the palm centre, window 0.8 mm. Gives 4 mm-4 m range for grasp approach; ~€4.

---

## 6. Bench tests (each can prove or kill an idea in a day)

| # | Test | Method | Pass if |
|---|---|---|---|
| 1 | **Friction ranking** | Pad sample (20 × 20 mm flat) on a block loaded with 500 g; pull it over glass, ABS, painted metal, cardboard with a spring scale or a load cell (HX711, €5); 5 pulls each, dry and with a drop of hand cream | μ values within ±30 % of the table in section 3; silicone ≥ 3× PETG on glass |
| 2 | **Skin pull-out** | Print the tip; snap in the skin; pull the skin pad edge along z, -y and x with a luggage scale | > 10 N in every direction, no tear in 50 push-in/pull-out cycles |
| 3 | **Magnetic calibration** | Fix the tip on a kitchen scale; press with a rod using a z-stage; record Bz vs force 0-5 N; tilt the rod for shear; 100 loading cycles | Repeatability ±10 %, drift < 5 % over 100 cycles, shear visible above 0.2 N |
| 4 | **Wire life** | Cycle a joint 0-95 degrees at 1 Hz with the cable in the hollow pin and the I2C chip logging every 100 ms (CRC errors); 100 k cycles = 28 h | Zero lost frames after 100 k cycles; no broken strand |
| 5 | **Coin / card pick-up** | 20 trials each with and without the shim on a flat table (coin 2 €, credit card), same arm start | ≥ 18/20 with the shim, ≤ 8/20 without |
| 6 | **Pinch hold** | Hang weights (100-500 g) on a ring in a two-finger pinch, servo at fixed torque | Silicone holds ≥ 3× the mass of bare PETG at the same servo load |
| 7 | **Pad drop/impact** | Drop the finger on a load cell at 0.1 m/s with and without the palm pad | Peak force drop > 50 % for the palm pad; < 10 % for the fingertip pad (confirms section 4) |
| 8 | **Hall angle** | Rotate the joint on a bench jig with a protractor ring (10-degree steps) | ±1 degree over 0-95 degrees, repeatable after power-cycle |

---

## 7. What to build as CAD next

1. **Merge the sensor-ready tip into the V2 `distal` phalanx** (`05_tip_body` replaces the V2 tip from z = 7.85; keep the fork, cheeks, linkage pin B). Confirm the DIP boss radius (7.85 mm) with the joints-structure agent; if the boss can shrink to ~5 mm the tip gains 2.85 mm.
2. **Hollow-pin joints** (Ø4 × 3 mm tube pins at DIP, PIP, MCP) with radial cable slots, and a cable channel through the middle and proximal phalanges: bore Ø2.6, 3 mm from the surface, on the palm side. Joint with the tendon-slot owner so the crossing strands and the cable do not share a slot.
3. **Proximal and middle phalanx pads with rivet-lock holes** (V2 proximal 1.5 mm and middle 1.2 mm): 2-3 Ø2 holes through the recess floor and a 0.4 mm lip around the pad outline.
4. **Palm heel pad and palm cage**: 10 mm Ecoflex pour cup with a printed TPU 85A lattice cage and 6 rivet-lock holes; add a palm pocket for the VL53L1X (5.5 × 3 × 2 mm) and the camera (12 × 12 × 6 mm, aimed 20 degrees toward the fingers).
5. **Joint angle sensor bay**: variant H1 (magnet in a brass pin, 5 × 5 mm PCB pocket in the cheek, +2.2 mm) and H2 (flange magnet pocket Ø2.1 × 1.1 mm at r 6, Hall pocket 3.4 × 2.2 × 1.6 mm in the cheek inner face). Print both for the index and bench-test (test 8).
6. **Skin family**: thumb pad skin (wider, larger crown, no magnet first), phalanx skin, palm skin. The same mould concept scaled (the script is parametric).
7. **Swappable tip variants**: 11 mm "tweezer" tip and a "texture" skin with ridges.

---

## 8. Sources

- `research/references/hands/sensing.md` (ReSkin/AnySkin, MLX90393, TMAG5273, AS5600, ORCA wire and skin wear, Figure 03 palm camera, Shadow Hall sensors).
- `research/references/hands/tendon-mechanics.md` §7 (pad μ values, "silicone ≈ 1, bare PLA ≈ 0.3, TPU 95A ≈ 0.5-0.7", 200 g example), §4 (fingertip force, lever 60 mm).
- `research/references/hands/README.md` (force gap: 3.4 N vs 9-20 N commercial, 45 N 1X NEO, 50-80 N human).
- `cloud-claude/2026-11-09-v2-hand-cad/reports/01-finger.md`, `cad/finger.py` (distal phalanx: W = 18, H = 16, hwt = 0.92 × W/2, r_boss 7.85, zt = 23, PAD_T 2.2, tip pad 3.0 mm, proximal 1.5 mm, middle 1.2 mm), `cad/params.py` (finger sizes), `cad/palm.py` (palm pad names).
- Literature facts marked **[bg]** (Smooth-On Dragon Skin/Ecoflex/Smooth-Sil hardness and cure times, Gent Shore-to-modulus relation, stainless tube sizes, VL53L1X size, copper fatigue strain limits, gecko adhesive) are from memory and were not re-fetched: verify before buying.
- Own calculations: `cad/05_contact_calc.py`; CAD: `cad/05_fingertip.py`.
