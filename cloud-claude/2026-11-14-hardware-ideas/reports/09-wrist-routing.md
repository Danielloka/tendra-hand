# 09 · Bare-strand wrist and palm routing (narrow phase)

Agent 09, 2026-11-14/15. Study only; nothing outside this folder was changed.

- Kinematics: [`cad/09_eyeplate_kin.py`](../cad/09_eyeplate_kin.py)
  (`uv run --no-project --with numpy --with scipy --with matplotlib python cad/09_eyeplate_kin.py`, ~2.5 min).
  Outputs: `out/09/eyeplate_kin.txt` (full printout), `eyeplate_kin.json` (eye table, firmware coefficients,
  forearm slot map, fan crossings), `eyeplate_kin.png` (plots).
- CAD: [`cad/09_routing_parts.py`](../cad/09_routing_parts.py)
  (`uv run --no-project --with cadquery --with numpy --with trimesh --with scipy --with rtree --with matplotlib --with pillow python cad/09_routing_parts.py`, ~9 min).
  It reads the eye table and writes STL + STEP per part to `out/09/`, plus `checks.json` and the renders
  `eye_plate.png`, `wrist_q0.png`, `wrist_flex+50_dev+20.png`, `wrist_flex-50_dev-30.png` and `index_trough_idler.png`.
- Inputs: the V2 wrist as built (`wrist.py` / `wrist_kin.py`: dev outer, flex inner, `R_palm = Ry(dev)·Rx(flex)`,
  flex −50…+50°, dev −30…+20°). The V2 knuckle entries come from `routes.json`, the V2 spool tangents from
  `forearm_summary.json`, and the line is PE#8 (0.47 mm, EA 11.1 kN, report 02).

Words used below:
- **Common mode:** both strands of a joint get longer (or shorter) together. The servo cannot cancel this; it
  only changes the loop pretension.
- **Differential:** one strand gets longer and the other shorter. That is exactly what one spool turn does, so
  the firmware cancels it.
- **Skew distance:** the closest distance between two straight strands that pass each other in 3-D.

---

## 1. Verdict: bare strands across the wrist work, if three things are done right

**Across the wrist, bare strands work.** With the right eye shape and layout, all 32 strands cross the V2
cardan with:

| Check (9 × 9 wrist grid, exact shortest-path model) | Result | Target |
|---|---|---|
| Pretension change per joint (loop length change × 21 N/mm) | **median 1.8 N, worst +3.9 N** (3 joints); −1.9 N at worst on the slack side | ±3 N (report 01 bench criterion) |
| Differential length change cancelled by the servo | up to ±6.5 mm per strand. After a linear correction, **≤ 0.19 mm left (≤ 1.5° of joint)** | < 2° |
| Servo travel (own motion + wrist compensation), SCS0009 300° | **all 16 fit; worst 279° (pip)** | ≤ 290° |
| Closest strands in the wrist free span | **1.32 mm centre to centre = 0.85 mm gap** | ≥ 0.5 mm gap |
| Strand to the V2 wrist parts | **≥ 0.15 mm** (ring flex-axis boss at dev −30°), palm flange ≥ 0.77 mm | > 0 (ask for a 0.5 mm chamfer, §6) |
| New parts vs the V2 wrist parts | **0 interferences in all 81 poses** | 0 |
| Efficiency η from the knuckle entry to the spool (μ 0.10) | **median 0.74 → 0.92** wrist straight, **0.67 → 0.82** at the worst wrist pose, worst strand 0.63 → 0.79 | better than V2 |

**The three conditions** (each one came out of a failed first version):

1. **The eye must pivot in the plane of the two axes.** A pivot h mm off that plane adds about h·θ²/2 of common
   mode to *both* strands. Report 01's 18G tube eye (2.5 mm long, pivots at the mouths ±1.25 mm) gives
   **+15.7 N** of pretension swing, so it is **killed**. The fix is a **thin plate eye with a fully rounded
   edge** (a "torus" lip, r 0.6) in a 1.16 mm stainless plate. The throat must be **tight: Ø0.65 for a 0.47 line**.
   With a Ø0.84 throat the line cuts corners inside the play, and the swing is 4.7 N; with Ø0.65 it is 2.9 N.
2. **The free span across the wrist must be parallel ("axial").** I tried straight lines first (spool → eye →
   knuckle). Pairing the flex and ext strands point-symmetrically (needed to cancel common mode) then makes
   strands **intersect: skew 0.00 mm. Killed.** The working layout uses **three identical drilled plates**: the
   wrist eye plate (on the cardan ring, z 0), a palm plate (z +22) and a forearm plate (z −30). Across the wrist
   all 32 strands then run parallel, so they cannot cross. The fan-out to the knuckles and to the spools happens
   beyond the outer plates, inside rigid parts.
3. **The firmware cancels the wrist** with one line per servo (§4). The servo must know both wrist angles.

**What is not solved yet (said plainly):**
- **The fan-outs beyond the outer plates still cross.**
  - Palm side: about 30–40 strand pairs pass closer than 0.97 mm, most of them within 8 mm above the palm plate.
    I could not get this below 30 by re-assigning eyes. It is structural: point-symmetric pairs on a compact grid
    send neighbours to swapped sides. The fix is a printed **reorder block** (bores at ≥ 2 mm spacing) right
    above the palm plate. This costs a fixed 5–15° bend on about 20 strands, which is **not yet in the η numbers**
    (about −2…−3 % for the affected strands).
  - Forearm side: 11 near-crossings. These are fixed by a **spacer comb** at z −35…−95 (fixed bends 1–15°,
    already included in η).
- **The eye is a wear point.** The lip pressure is T/(r·d) = **160 MPa at SCS stall and 418 MPa at HLS stall**,
  at D/d 2.6. The wrap is up to 61°, but only while the wrist is bent. This is better than the 18G rim
  (224/584 MPa) but far from a Ø3 dowel (64/167 MPa). It must pass the 10 k-cycle bench test (§8) before
  anyone builds on it. For HLS loads it is marginal.
- **The thumb strands turn 68–96° at the palm plate** (they leave the thumb mount sideways). That gives
  η 0.79–0.85, still better than V2's 0.63–0.76. A Ø4 dowel or an MR63 just above the palm plate should take
  this bend instead (thumb designer).

**If the bench test kills the eye** (fraying in < 10 k sweeps, or more than 10 N of pretension swing), use the
fallback in §7: lined steel coil housings across the wrist only. The palm and forearm stay bare.

---

## 2. What else I tried, with the numbers (all in `eyeplate_kin.txt`)

| Wrist crossing | Pitch | Worst pretension swing | Worst compensation travel | Lip pressure SCS / HLS | Gap in the free span | Verdict |
|---|---|---|---|---|---|---|
| Ideal point eye (reference) | – | 0.4 N (pair, flex 50°) | – | – | – | theory |
| 18G tube eye 2.5 long, flare r 0.4 (report 01) | 1.6 | **15.7 N** | 95° | 240 / 628 | 0.54 mm | ❌ pivots off the axis plane |
| Torus eye r 0.4, throat 0.84 | 1.85 | 5.3 N | 111° | 240 / 628 | 0.70 mm | ❌ pressure + swing |
| Torus eye r 0.6, throat 0.84 | 2.25 | 4.7 N | 135° | 160 / 418 | 0.94 mm | ⚠️ swing |
| **Torus eye r 0.6, throat 0.65, 8 × 4 grid** | 2.15 | **2.9 N** | 129° | 160 / 418 | 0.88 mm | ✅ but touches the ring boss (−0.20 mm at dev −30°) |
| **Torus eye r 0.6, throat 0.65, 7 × 5 grid (chosen)** | 2.15 | 3.9 N (median 1.8) | 137° | 160 / 418 | 0.85 mm | ✅ clears the V2 parts (≥ 0.15 mm) |
| Torus eye r 0.8, throat 0.84 | 2.65 | 4.4 N | 159° | 120 / 314 | 1.19 mm | ⚠️ travel; a throat-0.65 version is worth a bench test |
| Crossed Ø3 dowel fairlead (approx.) | 3.6 | **42 N** | 216° (2 of 16 servos out of travel) | 64 / 167 | – | ❌ pivots ±2 mm off the plane, too wide |
| Straight eye → knuckle routing | any | – | – | – | **0.00 mm (strands intersect)** | ❌ |
| Report 01's r 3 MCP spool + eye plate | – | – | MCP 330–427° | – | – | ❌ on SCS0009: keep r 5, or HLS (multi-turn, verify) |

Two first-principles facts carry all of this:
- **Coupling = moment arm.** A strand changes length by (its distance from the axis) × angle. On the plate,
  y sets the flex coupling and x sets the dev coupling. The servo travel to cancel it is **T ≈ 10·|x| + 20·|y|
  servo degrees** (x, y in mm; r_spool 5, dev range 50°, flex range 100°). So the eye field must be small, and
  wide in x rather than y.
- **Common mode = where the line pivots.** A pivot h off the axis plane gives h·θ²/2 per strand. At flex 50°,
  for a pair: h 0 → 0.4 N, h 0.5 → 8.1 N, h 1.25 → 20 N. The loop stiffness is EA/(L_f + L_e) ≈ 21 N/mm
  (13 N/mm for a soft braid), so ±3 N means **±0.14 mm** of loop length.

**Bend at the knuckle entry (the palm idler block).** A roller whose rim touches the abduction axis gives
**±0.01 mm** per strand over ±15° of abduction if its exit tangent is *at* the axis height (a = 0). At the V2 yoke
exit (a = 4.5 mm) it gives **−0.11 mm per strand, which is −4.5 N for a pair** (`checks.json`). A roller
*centred* on the axis is worse: both strands of a joint wrap the same side, so it gives ±0.8 mm of common mode
(±16 N). That kills report 01's "2 × MR63 on the abduction axis" taken literally. The block is therefore built
with the roller rims tangent to the axis plane, at the axis height.

---

## 3. The eye table

**Plate pattern:** 7 × 5 grid at a **2.15 mm pitch**, centre point empty: x ±2.15, ±4.30, ±6.45 and
y 0, ±2.15, ±4.30, in the ring frame, seen from the palm. 34 eyes on the grid; 32 are used, and the unused pair
(+6.45, +4.30) / (−6.45, −4.30) is not drilled.

**Assignment** (simulated annealing over joint ↔ point-pair, orientation and forearm slot): it minimises travel
overruns, then pretension, then forearm-fan crossings. The joints with the least spare travel (the pips at 200°,
cmc_rot at 197°) sit near the middle. The abductions (42° own travel) take the outside.

**Forearm slots:** in the axial layout the spool position no longer affects the wrist kinematics, only the
forearm fan. So the servos were re-slotted within their level (the HLS room on level 2 is kept). The
"forearm slot" column below is the V2 slot (named by its V2 servo ID) that each joint now uses.

| ID | joint | eye flex (x, y) | eye ext (x, y) | c_flex | c_dev | linear residual mm (joint °) | wrist travel ° | own + wrist ° | loop change mm | pretension N | eye wrap max ° | forearm slot (V2 ID) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | index_pip | (-4.30, -2.15) | (+4.30, +2.15) | +0.390 | -0.835 | 0.103 (0.8) | 79 | 279 | -0.074…+0.068 | -1.5…+1.4 | 59 | 13 |
| 2 | index_mcp_flex | (-2.15, -2.15) | (+2.15, +2.15) | +0.390 | -0.418 | 0.082 (0.7) | 58 | 198 | -0.077…+0.072 | -1.6…+1.5 | 58 | 2 (flipped) |
| 3 | index_mcp_abd | (-4.30, +0.00) | (+4.30, +0.00) | -0.000 | -0.836 | 0.039 (0.3) | 41 | 83 | -0.076…+0.041 | -1.9…+1.0 | 57 | 10 |
| 4 | thumb_ip | (+4.30, +4.30) | (-4.30, -4.30) | -0.778 | +0.836 | 0.166 (1.4) | 116 | 256 | -0.069…+0.156 | -1.4…+3.1 | 61 | 3 |
| 5 | thumb_mcp_flex | (-6.45, -2.15) | (+6.45, +2.15) | +0.390 | -1.253 | 0.122 (1.0) | 100 | 212 | -0.072…+0.070 | -1.4…+1.4 | 59 | 5 |
| 6 | thumb_cmc_flex | (+2.15, -2.15) | (-2.15, +2.15) | +0.390 | +0.418 | 0.082 (0.7) | 58 | 198 | -0.077…+0.069 | -1.7…+1.5 | 58 | 8 |
| 7 | thumb_cmc_rot | (+0.00, -2.15) | (+0.00, +2.15) | +0.390 | +0.000 | 0.065 (0.5) | 37 | 235 | -0.078…+0.071 | -1.6…+1.5 | 58 | 12 (flipped) |
| 8 | middle_pip | (-6.45, +0.00) | (+6.45, +0.00) | -0.000 | -1.253 | 0.059 (0.5) | 62 | 262 | -0.074…+0.042 | -1.4…+0.8 | 57 | 9 |
| 9 | middle_mcp_flex | (+4.30, -4.30) | (-4.30, +4.30) | +0.778 | +0.836 | 0.165 (1.4) | 116 | 256 | -0.067…+0.160 | -1.4…+3.3 | 61 | 15 |
| 10 | middle_mcp_abd | (-6.45, +4.30) | (+6.45, -4.30) | -0.778 | -1.253 | 0.189 (1.5) | 137 | 179 | -0.066…+0.160 | -1.6…+3.9 | 61 | 1 |
| 11 | ring_pip | (+0.00, +4.30) | (+0.00, -4.30) | -0.778 | -0.000 | 0.129 (1.1) | 75 | 275 | -0.074…+0.155 | -1.5…+3.2 | 61 | 16 |
| 12 | ring_mcp_flex | (-2.15, +4.30) | (+2.15, -4.30) | -0.778 | -0.418 | 0.147 (1.2) | 96 | 236 | -0.073…+0.152 | -1.5…+3.1 | 61 | 7 |
| 13 | ring_mcp_abd | (+2.15, +0.00) | (-2.15, +0.00) | +0.000 | +0.418 | 0.020 (0.2) | 21 | 63 | -0.079…+0.042 | -1.9…+1.0 | 57 | 11 (flipped) |
| 14 | little_pip | (+4.30, -2.15) | (-4.30, +2.15) | +0.390 | +0.835 | 0.103 (0.8) | 79 | 279 | -0.073…+0.069 | -1.5…+1.4 | 59 | 14 |
| 15 | little_mcp_flex | (+6.45, -2.15) | (-6.45, +2.15) | +0.390 | +1.253 | 0.122 (1.0) | 100 | 240 | -0.072…+0.070 | -1.5…+1.4 | 59 | 6 |
| 16 | little_mcp_abd | (+2.15, +4.30) | (-2.15, -4.30) | -0.778 | +0.418 | 0.146 (1.2) | 96 | 138 | -0.073…+0.158 | -1.8…+3.8 | 61 | 4 |

Notes on the table:
- The c coefficients (servo degrees per wrist degree) follow from the flex strand's eye (x, y) in mm:
  c_flex ≈ −y/5.5 and c_dev ≈ +x/5.2 (the exact values are in the table).
- The 6 joints with an eye at |y| = 4.3 have pretension swings of 3.1–3.9 N. The other 10 stay under 1.9 N.
- If ±3 N is a hard limit, use the 8 × 4 grid (2.9 N worst) and chamfer the ring bosses 1 mm (§6).
- Per strand: the palm and forearm plate points, the spool tangent, the length change range and the wraps at
  q0 and at the worst pose are in `eyeplate_kin.json → eye_table`.

## 4. Firmware compensation

The firmware already knows both wrist angles: the two wrist servos report them.

```
servo_deg[j] = G_j · q_j  (+ 0.60 · q_mcp for the pips, report 02 idler)
             + c_flex[j] · wrist_flex_deg + c_dev[j] · wrist_dev_deg
```

- G_j = 1.4 servo°/joint° (7 mm drum, 5 mm spool). c_flex and c_dev are the table above, fitted by least squares
  on the 9 × 9 grid.
- Sign: a positive servo angle reels in the flex strand. A positive c means the servo must turn toward
  "closing" when the wrist angle grows, because the flex path got shorter.
- What is left after the linear fit is ≤ 0.19 mm, which is ≤ 1.5° of joint. Adding the three quadratic terms
  (θf², θd², θf·θd) only gets it to ≤ 0.14 mm. **Linear is enough**; the joint-angle sensors (report 01/05)
  close the rest.
- Calibration: sweep the wrist with the fingers held at q = 0. For each servo, find the offset that keeps the
  joint's Hall sensor (or a pointer) still, and fit c_flex and c_dev. Expect the fitted values within ±10 % of
  the CAD values.
- **Safety:** if a wrist angle is unknown, freeze the wrist. If a strand is not compensated, its finger moves by
  up to ~ (c × 50°)/1.4 ≈ 45° of joint. That is not dangerous, but it is not acceptable either.

## 5. Parts built (`out/09/`, STL + STEP each; all valid solids, watertight, no mutual interference)

| Part | Material | Size / key features | Body |
|---|---|---|---|
| `eye_plate` | **stainless 304, 1.16 mm** (laser cut by a sheet-metal service, or drilled through the printed jig) | 21.8 × 12 mm, r 1.5 corners. 32 torus eyes: throat Ø0.65, both edges fully rounded r 0.6 (mouth Ø1.85). 2 × M1.6 countersunk at (±9.2, 0). 2.1 g | ring (moves with dev) |
| `eye_block_R/L` | PAHT-CF | x ±(8.0…11.0), \|y\| ≤ 5, z 0.58…3.6, so r ≤ 6.0 from the flex axis (the V2 palm keeps r ≥ 10 there) and ≤ 11.6 from the dev axis (the forearm keeps ≥ 15). Each block is screwed to the inner face of the ring's flex-axis boss with 2 × M1.6 × 4 socket head at (y ±3.6, z 2.2). Ø4.4 relief for the flex pin end (which turns with the palm). Pilot for the plate screw | ring |
| `palm_plate` | stainless, **same drilling as the eye plate** (drill all three as one stack) | at z +22 (palm frame) | palm |
| `palm_plate_carrier` | PETG | 3 mm frame above the plate. Window = the fan hull. Ears with M2 at x ±17 (outside the wrist heel zone) to palm bosses | palm |
| `forearm_plate` | stainless, same drilling | at z −30 (forearm frame); fit pan-head screws from the top | forearm |
| `forearm_plate_carrier` | PAHT-CF (heat) | 3 mm frame under the plate, 2 legs up to the V2 forearm frame's top ring (needs 2 × M2 heat-set inserts at (0, ±19.5, −22)) | forearm |
| `index_trough` + `index_trough_lid` | PETG | Test section of the palm for the index: 6 straight open grooves (1.3 wide, open on the back face) from the palm carrier (z 26) to the knuckle (z 71). 1.6 mm lid, 4 × M2 | palm |
| `index_idler_block` + 2 × MR63ZZ + 2 × Ø3 × 8 dowel + 4 × M3 washer 0.5 | PETG / bought | Rollers on dowels parallel to the abduction axis. Their rims are tangent to x = 20 (the axis plane), and their centres are at the abd axis height z 80.5 (a = 0). Flex strand at y −9, ext at y +5. The pip strands pass straight through the 6 mm slot between the stations. 2 × M2 into the palm | palm |

Bill of extra parts for the whole hand:
- 3 stainless plates (about €10 for a laser-cut set of 5; or 1.2 mm 304 sheet + a carbide Ø0.65 drill +
  a Ø2 ball burr for the round edges).
- 6 M1.6 screws, 4 M2 screws.
- Per finger: 2 MR63ZZ, 2 dowels, 4 washers (16 MR63 for the hand).

**Deleted from V2:** 32 PTFE sheaths, 32 barrel adjusters, the adjuster block, the sheath clamp, the 60 mm
forearm fan bay and the bundle comb, about 70 parts.

## 6. Checks (`out/09/checks.json`)

- **Validity:** all 18 solids valid and watertight. No interference between my parts at q = 0.
- **The new parts against the V2 wrist parts on the full 9 × 9 grid:** ring-body parts rotate with the ring,
  palm parts with the palm, forearm parts stay fixed. The V2 parts are base, ring, palm flange, pins, servos,
  gears, cranks and rods, with the old sheath clamp removed. **0 interferences in 81 poses.**
- **Strands (as 0.47 mm lines) against the V2 wrist parts, 9 × 9:**

  | V2 part | Smallest gap |
  |---|---|
  | wrist base | 3.32 mm |
  | palm flange | 0.77 mm |
  | flex pins | 2.84 mm |
  | dev pins | 6.75 mm |
  | **ring** | **0.15 mm**, at dev −30°: an outer-column strand passes the inner bottom edge of the flex-axis boss, which sticks 1 mm into the ring opening at x 11 |

  **Request to the wrist designer:** chamfer that edge 0.6 × 45°. This gives ≥ 0.6 mm and stays clear of the
  623 seat (r 5).
- **Strand to strand in the wrist free span: ≥ 1.32 mm centre to centre = 0.85 mm gap** (worst at flex −50°,
  dev −24°).
- **Fan-outs:** palm 40 near-crossings at z 22–59 (most at 22–30). Forearm 11, at z −36…−94: these need the
  spacer comb, with bends 1.0–15.3°.

**V2 vs bare strand, knuckle entry → spool** (finger-internal bends are left out of both). The full
per-strand table is in §6 of `eyeplate_kin.txt`.

| | Wrap, wrist straight | Wrap, worst wrist pose | η median (straight / worst) | Worst strand η |
|---|---|---|---|---|
| V2 (PTFE: palm + fan + 58° wrist) | 61–265° (median ~170°) | +58° | 0.74 / 0.67 | 0.63 (thumb cmc_rot) |
| Bare strand (plates + eye + spacer + rollers) | 5–137° (median ~40°) | +55…+65° on the eye | **0.92 / 0.82** | 0.79 (thumb cmc_rot) |

Example: index mcp_flex flex strand, 223° → 34° (η 0.68 → 0.93).

## 7. Fallback if the eye fails the bench test: coil housings across the wrist only

The palm and the forearm stay bare and use the same plates as stop plates. Only the wrist span gets housings:
- 32 × PTFE-lined closed steel coil (0.3 wire, OD 2.0) of about 80 mm. They end on drilled 2 mm **aluminium stop
  plates** (report 02 H2/H6) at palm z +22 and forearm z −30, in the same 7 × 5 pattern scaled to a 2.6 mm
  pitch.
- The bundle bends through the V2 ring opening, as V2 already showed: minimum radius 24 mm.
- Numbers:
  - Shortening at SCS stall about 0.33 mm (2.7° of joint).
  - No firmware compensation and no pretension swing: the housing carries the path change.
  - Friction ×0.90 at the worst wrist pose (58° in the liner).
  - The coil path length changes by ±6.7 mm (V2), so **one end must slide**. Fix the palm end, and let the
    forearm stop plate float on 2 guide rods with a 3 N spring. The spring keeps the coils seated, and the
    plate takes up the ±6.7 mm.
  - Cost about €10 of coil and liner, plus 2 drilled plates.
- This is V2's wrist bundle done right (metal stops, a defined take-up), without the 220 mm sheaths. Expected
  median η about 0.86 (bare palm and forearm, ×0.90 in the wrist liner).

## 8. Bench test that proves or kills the eye (one weekend, before printing a hand)

1. **Print the V2 cardan** (ring, base, palm flange) in PETG with 623 bearings. Fit the three plates and the
   blocks, and lock the wrist with a printed protractor at the 9 × 9 poses.
2. **Pretension swing:** make one loop of PE#8: forearm plate → eye → palm plate, around a Ø14 "drum" pulley
   on a bearing, and back. Use the eyes of joint 10 (the worst one) and of joint 13 (the best one). Set 5 N with
   a luggage scale on the drum arm, then sweep. **Pass:** the loop tension stays at 5 ± 3 N (± 4 N for the
   |y| = 4.3 rows).
3. **Differential:** hold the drum and measure how far a spool-side mark moves vs the wrist angle, with
   calipers or a dial gauge. **Pass:** it matches c × r_spool within ±0.2 mm.
4. **Wear:** 20 N dead weight on one strand. A servo sweeps the wrist flex ±50° while a second servo
   reciprocates the strand 12 mm (one finger stroke). Run 10 000 cycles, then inspect at ×10 and measure the
   break load against a fresh piece. **Kill:** visible fraying or > 30 % strength loss. Run it on stainless and
   on brass, and with throat Ø0.65 vs Ø0.84.
5. **Friction:** pull through the bent eye (wrist 50°) with a scale on each end, at 5 / 20 / 45 N.
   **Pass:** η ≥ 0.88 per eye at 50° (μ ≤ 0.14).

## 9. Open risks

1. **Eye wear** (the main risk): 160/418 MPa lip pressure at D/d 2.6. The bench test decides. If HLS loads
   matter, test the r 0.8 lip (120/314 MPa) at a Ø0.65 throat.
2. **Palm fan-out crossings** (~30–40 pairs) need a reorder block. It is not designed yet and costs about
   5–15° of fixed bend on ~20 strands.
3. **Servo travel:** pips 262–279° of 300°. That leaves no room for report 01's reeved PIP or r 3 MCP spool on
   SCS0009. Those need HLS3606M (multi-turn: verify) or a bigger spool.
4. **SCS0009 range 300°** and **HLS3606M multi-turn** are from memory: verify both.
5. **The thumb's 68–96° turn** at the palm plate needs its own roller.
6. **The wrist servo angle** must be valid before any finger moves (§4 safety).
7. **The index idler block sits at the abduction axis height** (a = 0), inside the V2 yoke's swept space. The
   finger designer must leave this space free, or accept a = 4.5 mm (−4.5 N at ±15° of abduction).
8. **The exact model is frictionless and uses rigid lips.** Braid flattening in the Ø0.65 throat is not
   modelled; the bench test measures it.

## Sources

V2 study `wrist.py`, `wrist_kin.py`, `forearm.py`, `params.py`, `out/assembly/routes.json`,
`fan_routes.json`, `out/wrist/forearm_summary.json`; reports 01 (§3.1, §4), 02 (D1–D3, H1–H8, G1–G12) and 04
(§3.4, PAHT-CF) of this study; capstan law and line data as in report 02.
