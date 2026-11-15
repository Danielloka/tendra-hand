# 13 · Wrist load path for bare strands (review 10 #3, #2, #4)

Agent 09, round 2, 2026-11-15. Study only; nothing outside this folder was changed.

**Scripts and outputs**
- Loads: [`cad/13_wrist_load.py`](../cad/13_wrist_load.py), output `out/13/wrist_load.txt` / `.json`.
- CAD: [`cad/13_wrist_parts.py`](../cad/13_wrist_parts.py) writes:
  - STEP + STL for every part in `out/13/`;
  - `checks.json` and `reorder.json`;
  - renders `w13_*.png`.
- Kinematics re-run with the shared travel table:
  `uv run --no-project --with numpy --with scipy --with matplotlib python cad/09_eyeplate_kin.py --round2`
  writes `out/13/kin/` and is summarised in `out/13/travel_table_wrist.json`, the wrist column for `out/12/travel_table.json`.

---

## 1. Verdict

**The wrist can carry the bare-strand load. The cardan has to become metal, and the 09 eye-plate mounting was wrong.**

| Load case (all at 5 N pretension) | Compression through the wrist | Per wrist bearing | Eye-plate load (worst wrist pose) | Torque on the wrist drive |
|---|---|---|---|---|
| SCS0009, fist at the 80 % back-off, all 16 flex strands | **739 N** | 372–379 N | **700 N** | 0.39 N·m |
| SCS power grasp (12 strands at 50 %) | 431 N | 225 N | 409 N | 0.24 N·m |
| SCS pinch (index + thumb at 80 %) | 377 N | 203 N | 358 N | 0.39 N·m |
| HLS3606M fist at 80 % | **1670 N** | 843–859 N | **1585 N** | 1.01 N·m |

| Part | V2 / 09 as drawn | W13 (this report) |
|---|---|---|
| Bearings | 623 on Ø3 pins: **s0 0.55 (SCS) / 0.24 (HLS)** | Flanged plain bush 5 × 7 × 4.5 (iglidur G or sintered bronze) on **Ø5 hardened pins**: **s0 5.0 / 2.2** (bronze 2.5 / 1.1) |
| Pins | Ø3: **492 / 1113 MPa** in bending | Ø5: **137 / 309 MPa** (hollow Ø5/Ø2.4 cable pins: 145 / 326 MPa) |
| Ring | printed PA-CF: **237 / 537 MPa** against ~15 allowed | **CNC 7075-T6**, bar 2.9 × 7: **123 / 278 MPa, SF 2.7 / 1.2** (6061: 1.5 / 0.66) |
| Eye-plate mount | 4 × M1.6 into printed bosses: 139 / 315 MPa screw shear, 28 / 63 MPa on the print | **Tabs bearing on 7075 ledges**: 51 / 115 MPa (yield 500); M2 screws only retain |
| Eye plate itself | – | 1.16 mm stainless, 19.6 mm wide: in-plane 33 / 76 MPa, out-of-plane 136 / 307 MPa → **304 ½-hard** (yield ~760) |
| Palm plate seat | M1.6 into a printed carrier, with the load pulling the plate off it | Plate sits **in a pocket of the 7075 palm yoke**: 0.2–0.67 kN pushes it into the seat |

**The fix ("W13"): the whole tendon-compression loop runs in metal, from the palm plate to the forearm plate.**
- V2 keeps: its axes, ranges (flex ±50°, dev −30…+20°), the 30 mm gap, the push-rod drive and the outlines.
- What changes:
  - Ring, palm yoke (= V2 palm flange plus the palm-plate seat) and forearm yoke (= V2 wrist base) are made in
    **CNC 7075-T6**.
  - **Ø5 hardened pins**, two of them hollow for the sensor cable.
  - **4 flanged plain bushes**.
  - **4 identical laser-cut stainless plates**: eye, palm, forearm and reorder plates.
- Cost estimate (not quoted, verify):
  - 3 small CNC parts about €60–100 (JLC/PCBWay class);
  - 4 laser-cut plates about €15;
  - pins and bushes about €10.
- No printed part carries a concentrated tendon load any more. What printed parts still see:
  - the palm shell gets the knuckle reactions over large areas;
  - the forearm plate carrier takes ≤ 69 N (SCS).

**Re-run with the shared travel table (`out/12`, MCP back to r 5): all 16 servos fit SCS0009 with the 15° margin.**
- Worst servo uses 280° of the 285° allowed (ring mcp_flex). The pips use 235–277°.
- Pretension change over the 9 × 9 wrist grid: worst 3.0 N, median 1.6 N.
- Strands in the wrist span: gap ≥ 0.87 mm.
- η median 0.90, against V2's 0.74.

The eye assignment now also **balances the wrist torque** (Σ T·y and Σ T·x of the flex eyes). Without that term,
a fist at HLS put **−3.0 N·m** on the flex drive, almost its 3.6 N·m stall. With it, the torque is **−1.0 N·m**
(SCS: 0.39 N·m).

**The other integration fixes**
- **The idler block is deleted** (review #4). The finger knuckle module owns the abduction crossing.
- My palm parts end at the palm plate (z +30), plus per-strand liners up to the module entry (§5). The palm-side keep-out for the module is everything
  above z = MCP z − 16.
- **The palm fan-out:** a reorder plate was designed and **rejected**. It cuts the crossings 47 → 2 but adds a
  median 20° and a worst 71° of bend. The fix is per-strand liners inside the rigid palm (§5).

**Sensor cable** (requests from 15): the passage is confirmed and in the CAD, through two hollow pins on the axes,
so the wires only twist (§4).

---

## 2. Load path, first principles

Bare strands pull the hand toward the forearm. With sheaths, the sheath carried that pull back in compression.
Without them, the structure between the strand ends has to carry it.

Equilibrium of the cardan ring (`13_wrist_load.py`, strand directions from the 09 model, all 81 wrist poses):
- **Palm → ring, through the flex pins:** Σ T, along the palm-span direction (fixed in the palm frame, ≈ −z).
- **Forearm → ring, through the dev pins:** Σ T, along the forearm-span direction (≈ +z in the forearm frame).
- **Eye plate:** Σ T·(û_palm + û_forearm), up to 0.95 Σ T at flex 50° / dev −30°. The ring balances it. The
  vector is mostly in the plane of the plate (−370, −570, −170 N at SCS).
- **Torque on the wrist drive:** Σ T·y about the flex axis and Σ T·x about the dev axis, from the eye offsets.
  These are the same moment arms that set the servo compensation. It vanishes only if the flex eyes are balanced,
  so the assignment now includes that term.
- **Palm plate** (strands turn toward the knuckles and the thumb): **0.2–0.67 kN**, pointing toward the wrist
  (palm frame (171, −24, −646) N in the SCS pinch, mostly from the thumb strands).
- **Forearm plate:** ≤ 69 N, because the spools sit nearly in line with it.

Ruled out:
- **A central compression post or ball:** the strands occupy the centre. Eyes on a ring around a Ø10 post give
  ~190° of compensation travel.
- **A spherical shell around the bundle:** it collides with the lugs and push-rod balls in the 30 mm gap.
- **Sheaths across the wrist** remain the fallback from report 09 §7.

## 3. W13 parts (`out/13/`, STEP + STL)

| Part | Material | What changed from V2 / 09 |
|---|---|---|
| `ring_w13` | **7075-T6 CNC** | V2 ring; opening x ±11.5, cut back beyond \|x\| 14.4 (bar 2.9 × 7; past 14.5 the palm flange sweeps); X-boss bush seats Ø7 × 4.5; Ø5 press seats at the Y ends with an r 5 boss; eye-plate **ledges** (x 9.9–11.6, \|y\| 6.6–9.5, 1.8 thick above and below the tabs) with 4 × M2; cable slot, groove and hole |
| `eye_plate_w13` | **304 ½-hard, 1.16 mm, laser cut** | 19.6 × 19 mm with 4 tabs (x to ±11.4 at \|y\| 6.7–9.4); 32 torus eyes, throat Ø0.65, r 0.6, round-2 pattern; 4 × M2 clearance holes |
| `palm_yoke_w13` | **7075-T6 CNC** | V2 palm flange with Ø5 press seats in the lugs, plus the **palm-plate seat**: a frame at z 26–30.4 (window ±8.4 × ±5.6, pocket for the plate) on two legs at \|x\| 11.6–14.4, \|y\| ≤ 3.4 down to the flange (inside the V2 heel keep-out band) |
| `palm_plate_w13` | stainless, same drilling | at z +30 (was +22: the seat needs z ≥ 22) |
| `forearm_yoke_w13` | **7075-T6 CNC** | V2 wrist base; lug bush seats Ø7 × 4.5 |
| `forearm_plate_w13` + `forearm_plate_carrier_w13` | stainless + PAHT-CF | carrier under the plate; 2 legs to the forearm frame's top ring (M2 inserts at (0, ±19.5, −22)) |
| `reorder_plate` + `reorder_spacer` | stainless (4th plate of the drilled set) + PETG | §5 |
| `pin_flex_R_hollow`, `pin_dev_B_hollow` | precision steel tube 5 × 1.25 (E355+C) or drilled 17-4PH | §4 |
| `pin_flex_L`, `pin_dev_F` | ISO 8734 Ø5 hardened dowel | |
| `bush_flex_R/L`, `bush_dev_B/F` | flanged plain bush 5 × 7 × 4.5 (iglidur G, or sintered bronze for SCS only) | flex bushes in the ring, dev bushes in the forearm lugs |

Notes:
- **Hard stops:**
  - V2's flex stop (an arc wall on the palm lug plus a tooth under the ring boss) is **removed**, because it
    occupies the x 13.5–14.4 space the 2.9 mm 7075 ring bar needs (68–117 mm³ clash in the first check).
  - New flex stops: two M3 set screws in the palm yoke that hit the ring's outer y-face at ±52°, plus soft limits
    on the wrist servos. Not modelled.
  - The dev stop pins in the ring's Y ends are kept.
- Friction of the plain bushes: 0.28 N·m (SCS) / 0.63 N·m (HLS), and only while the hand squeezes at full force.
  Ball bearings would need 6800-class sizes (s0 0.93 at HLS) that do not fit the 30 mm gap. A drawn-cup needle
  bearing HK0509 (s0 ~1.5 at HLS) would fit only with 9 mm bosses.

### Checks (`out/13/checks.json`, 9 × 9 wrist grid)

**Validity:** all 17 parts are valid OCC solids. All are watertight except `ring_w13`: its mesh is not closed
where the groove spheres meet the bar. The B-rep is valid, so this is cosmetic for STEP; check the STL in the slicer.

**Interference:**
- W13 parts plus the V2 drive (servos, gears, cranks, rods), the moving bodies rotated per pose: **0 interferences
  in 81 poses.**
- Two clashes were found and fixed on the way:
  - a 3.5 mm bar against V2's flex stop wall (68–117 mm³) → stop wall removed;
  - the bar beyond |x| 14.5 against the palm flange at |flex| ≥ 37.5° (7.6 mm³) → bar cut back to |x| ≤ 14.4.

**Strand clearance (centreline minus line radius) to the metal parts:**

| Part | Smallest gap |
|---|---|
| **Ring** | **0.19 mm**, at dev −30°: the bottom edge of the flex-boss face (as in 09). **Chamfer it 0.6 × 45°**, leaving ≥ 0.6 mm |
| Palm yoke | 1.06 mm |
| Forearm yoke | 3.32 mm |
| Pins | ≥ 2.31 mm |
| Bushes | ≥ 1.79 mm |

**Kinematics** (round 2, `out/13/kin/eyeplate_kin.txt`):
- worst pretension change 3.0 N (median 1.6);
- linear residual ≤ 0.185 mm (1.5° of joint);
- strand gap in the wrist span ≥ 0.87 mm;
- all 16 servos within 285°.

## 4. Sensor cable across the wrist (requests from 15)

The cable is 6 conductors, 38 AWG PTFE, bundle about Ø1.5. Path: palm → +x flex pin → ring → +y dev pin →
forearm. The wires only twist.

| Pin | How it is held | Span | Cable in | Twist length | Cable out |
|---|---|---|---|---|---|
| +x flex pin | pressed in the palm lug, runs in the ring bush | x 11…21 | axially at x 21 (palm side, no relative motion) | 10 mm of Ø2.4 bore | axially at the ring boss face x 11 |
| +y dev pin | pressed in the ring, runs in the forearm-lug bush | y 18.5…29 | Ø1.6 radial hole through the ring and pin at y 19.6 | 9.4 mm | axially at y 29, into the forearm |

- On the ring, a 1.6 × 1.6 slot in the flex-boss face leads the cable up to a 2 × 2 groove on the ring top face.
  The groove runs along the +x bar to the +y end.
- 15's numbers: wire strain 0.14 % (flex, 100°) and 0.04 % (dev, 50°), against a 0.2 % limit.
- **Deburr** the axial pin mouths and the radial hole to r ≈ 0.3 mm.

**Stress in the hollow pins:** the bore lowers the section modulus by 5.3 %, giving 145 MPa (SCS) and 326 MPa
(HLS) in bending.
- A through-hardened dowel cannot be drilled at home. So the two hollow pins are:
  - **cold-drawn precision steel tube 5 × 1.25** (E355+C, yield about 520–640 MPa; SF ≥ 3.5 SCS, ~1.6–2.0 HLS);
  - or 17-4PH bar drilled on a lathe (yield ~1170).
- The bush runs on the tube's OD; keep it h8 or better.

**Fallback:** a dorsal service loop at x = 0, 70 mm long, bend R ≥ 15 mm. The eye plate stays cable-free (a cable
through it would kink by 58°).

## 5. Palm fan-out: reorder plate

Straight lines from the palm-plate holes to their targets give **47 strand pairs closer than 0.97 mm**
(minimum 0.00 mm).

**Targets:**
- **Finger strands:** module entry at MCP z − 16, vertical, at 07's strand exits clipped to a 19 mm module
  (\|dx\| ≤ 6). These are **provisional**; replace them with 12's module exit table when it exists.
- **Thumb strands:** a **thumb roller row**, 8 × MR63 on a Ø3 dowel along Y at x 17, z 50 (y −10.5…+10.5,
  3 mm pitch). Its rolling turn replaces the 68–96° sliding turn on the palm plate.

**Reorder plate:**
- The 4th stainless plate, same torus holes, at **z +38**, on a printed PETG spacer resting on the palm-plate seat.
- Hole positions are optimised (Powell): minimise the bends, keep every pair ≥ 1.10 mm apart and the holes
  ≥ 2.25 mm apart.

**Result (`out/13/reorder.json`): rejected.**

| | Straight lines | Reorder plate at z 38 |
|---|---|---|
| Pairs closer than 0.97 mm | 47 | 2 |
| Smallest centre distance | 0.00 mm | 0.04 mm (two holes nearly coincide) |
| Extra fixed bend per strand | 0° | median **20°**, max **71°** |

Why it fails:
- A plate only 8 mm above the palm plate must deflect strands sharply to separate them.
- Point-symmetric pairing sends neighbours to opposite sides of the palm.
- With the 16 extra bends, η drops by a median of ×0.97 and a worst of ×0.88.
- Earlier tries went the same way: z 40 with the thumb row at z 50 left 11 pairs; an L-BFGS version was worse.

**Recommended instead: give each palm-fan strand its own short housing inside the palm.**
- A PTFE liner in a rigid printed channel (report 02 H5), or a lined coil, from the palm plate to the module entry.
- Both ends sit on the same rigid palm, so the housing carries no load and sees no wrist motion. The 0.24 mm
  pretension budget is not touched.
- Strands in their own tubes may cross each other at different depths, like the laminated palm in report 04.
- Cost: the liner friction over the existing fan bends, η ≈ e^(−0.1 · bend). That is about the same as the bare
  straight lines, because the bends do not change.
- So the bare-strand rule becomes: **bare across the wrist; lined inside the rigid palm wherever strands cross.**

The reorder plate and spacer stay in `out/13/` as a study, and **must not be built**. Re-run the assignment
with 12's real module exits and a strong palm-crossing weight before deciding how many strands need liners.

## 6. Open items

1. **Palm fan-out:** the reorder plate is rejected (§5). Design per-strand palm liners against 12's module exit
   table: re-run `optimise_reorder` / the 09 assignment with the real `FINGER_EXIT`.
2. **Thumb.**
   - The thumb roller row is only a target point here; the thumb designer owns it.
   - The thumb's palm-plate bend (now up to ~67°) still slides on a torus lip.
3. **Forearm fan.**
   - 10 near-crossings (bends ≤ 8.2°, already in η) need the spacer comb from report 09.
   - The forearm slots changed again (`travel_table_wrist.json → forearm_slots / spool_flips`); the forearm
     designer must use this map.
4. **HLS margins.**
   - Ring 7075 SF 1.2, sintered-bronze bush s0 1.1 and hollow pins SF ~1.6 at the HLS 80 % fist.
   - All fine for SCS. Choose iglidur G bushes and 7075 if HLS is planned.
   - The ring formula (M ≈ F·L/8) is approximate: check it on the bench (below) or by FEA.
5. **Values from memory.**
   - Bush static ratings (igus 80 MPa, bronze ~40 MPa), 623/625/6800 C0, HK0509 C0, E355+C yield, HLS torque.
   - Verify all of them from datasheets.

## 7. Bench test that proves the load path (half a day, after the eye-plate wear test, 08)

1. **Load path:**
   - Assemble the W13 cardan alone: ring, yokes, pins, bushes.
   - Hang it from the palm yoke and load the forearm yoke with **800 N** (SCS fist + 10 %) for 1 h, at flex 0,
     +50° and dev −30°.
   - **Pass:** no visible set; the play at the pins stays ≤ 0.1 mm; the ring shows no permanent deformation
     under a dial gauge.
2. **Eye-plate tabs:**
   - Pull 32 strands through the eye plate at 25 N each, with the wrist at flex 50°.
   - **Pass:** the tabs and ledges are unmarked; the plate deflects ≤ 0.1 mm at its centre.
3. **Wrist torque:**
   - With the hand on the cardan, squeeze a 50 mm cylinder at the SCS 80 % limit.
   - Read the wrist servo load.
   - **Pass:** ≤ 0.5 N·m (predicted 0.39).
