# 03 · Wrist and forearm (V2 concept)

Study 2026-11-09 · files: `cad/wrist.py`, `cad/wrist_kin.py` (numbers, no CAD), `cad/forearm.py` ·
outputs: `out/wrist/` (STEP/STL/GLB per part, PNG renders, `wrist_summary.json`, `forearm_summary.json`,
`wrist_check.log`, `forearm_check.log`).

```
uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy --with pillow python cad/wrist.py    # ~3 min
uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy --with pillow python cad/forearm.py  # ~2 min
uv run --no-project --with numpy --with scipy python cad/wrist_kin.py [--opt Y]   # bundle + drive numbers (optimiser ~4 min)
```

## 1. Summary

| | Result |
|---|---|
| Wrist joint | Hollow cardan with crossing axes at the origin. **Dev is the outer axis** (forearm lugs at ±Y), **flex is the inner axis** (palm lugs at ±X). 4 × 623ZZ, 3 mm steel dowels, PETG/PA‑CF |
| Ranges | dev −30…+20° (as params). **Flex −50…+50°, not −60…+70** (see §3; requested params change). Hard stops at range ±2° |
| Ring opening | 24 × 37 mm (needs 19 × 29.4 for the bundle sweep, wrist_kin) |
| Sheath bundle | 32 × PTFE 1×2 in a round 4‑6‑6‑6‑6‑4 bundle (12.9 mm). Clamped at the palm (clamp face z +14) and guided by a comb at z −60. Worst tube bend **R 24.4 mm** over the range (limit 15, each tube kept in its cell = worst case). Centreline R ≥ 21.7. Length change **±6.7 mm** worst tube, taken by the forearm fan (fan bends **R ≥ 22.2 mm**) |
| Drive | 2 × STS3215 (12 V) + 14T:36T spur stage (2.57:1) + crank + M4 push rod with ball ends (2‑RSS). Optimised by differential evolution: **flex ≥ 3.6 N·m stall** everywhere (median 5.1), **≥ 1.1 N·m continuous** (median 1.56). **Dev ≥ 16 N·m stall**. Transmission angle ≥ 30° everywhere, det J never changes sign (no toggle). Servo travel −144…+150° |
| Forearm | 95 × 66 mm, z −18…−230 (212 mm). 16 SCS0009 in two cassettes, shafts toward a central spool rail. Split‑spool tensioners and outboard bearings. 32 barrel adjusters. Room for HLS3606M on the 6 high‑load joints. ESP32‑S3 + FE‑URT‑1 + power board on a tray. 4 × M4 / PCD 50 arm flange |
| Checks | Wrist 5 × 4 pose grid (min/mid/max of both axes) with the palm proxy: **all clear**. All 4 stops engage just past the range. Forearm static plus the moving wrist drive at 9 extreme poses: **all clear**. All 97 solids valid |

## 2. What I took from the research

- **Earlier wrist study (2026‑11‑04):** sheaths clamped at both ends, a bundle through a hollow cardan, ≥ 30 mm gap, 2 × STS3215 + 2:1 via push rods, barrel adjusters, slack. I kept all of these. Its "22 × 14 mm oval" and "−60…+70 in a 30 mm gap" turned out to be too optimistic once the CAD was checked (§3).
- **Optimus V3 patent:** two linear actuators drive the 2‑DOF wrist through a U‑joint, with all actuators in the forearm. Tesla's cable stacks switch exactly at the wrist axes. I use the same "actuators in the forearm, rods to the hand" idea, with servo + crank as a cheap stand‑in for a linear actuator. Sheaths make the axis‑crossing trick unnecessary.
- **1X NEO:** motors in the forearm, the forearm treated as the actuator pack, a 3‑DOF wrist (17.75 N·m). Our forearm is a similar size (95 × 66 vs a human ~80 × 60 mid‑forearm).
- **RUKA‑v2 / CRAFT pattern:** 2‑RSS rods to a passive U‑joint, as recommended by the earlier study.
- **ORCA and tendon‑mechanics.md:** split spool with friction clamp (continuous tension), outboard spool support (2 strands = ~40 N side load), barrel adjusters for fine tension, braid UHMWPE, "the line only touches PTFE, steel or its own drum".

## 3. The big finding: −60…+70° flex does not fit a crossing‑axis cardan in a 30 mm gap

I tried both cardan orders and checked them in numbers and CAD.

1. **The bundle swings in the ring.** The sheaths are clamped in the palm (z +14…+16). That end swings with the palm on a radius of 14–16 mm, so at 70° the bundle passes the ring plane up to **±17–19 mm** off‑axis in Y. A 22 × 14 opening is far too small.
   - If the ring follows the palm in flex (flex outer, the sim's order), the ring and the palm‑side dev lugs must be about 50–60 mm deep, against a 28 mm palm. At full flex the front palm lug also plunges into the bundle's path below the wrist.
   - If the ring follows the forearm in flex (**dev outer**), the deep ring sits on the forearm side, which is ~60 deep anyway. That is the order I use.
2. **The palm sweeps the forearm‑side dev lugs.** The lugs sit at r 22–28 mm from the flex axis, 73–118° from +Z. Anything fixed to the palm near the centre line (the sheath clamp, the sheaths going up into the palm, the palm base) starts at 15–25° and reaches the lugs after about 50–55° of flexion. Above that you would need a slot through the middle of the palm.
3. With **flex ±50° (stops ±52°)**, everything fits. The palm needs only a rounded heel (§6.1), the tubes bend at R ≥ 24 mm, and the drive gets about 2× stronger. ±50° covers the functional wrist range for daily tasks (Ryu et al. 1991: ~±40–55°). To regain ±70°: a gap of ≥ 45–50 mm, or offset (non‑crossing) axes à la Tesla, or a palm with a central notch about 20 mm tall.

Also: the dev‑outer order means `R_palm = Ry(dev) · Rx(flex)`, but the sim (`convert_v1_wrist.py`) uses flex then dev. Either the sim swaps the order, or a small remap is applied. The two differ only in combined poses.

## 4. Wrist design

- **Ring** (`wrist_ring`, PA‑CF or PETG, 7 mm thick). Rounded‑rectangle frame, inner 24 × 37 with square inner corners, outer 30 × 43. It is cut back beyond |x| 13.5 except for the bearing bosses. X bosses (x 11–15.5, Ø13.2) hold a 623ZZ each for the flex pins. The Y ends carry pressed Ø3 dev pins, a Ø13.2 face boss, and a Ø1.6 stop pin. A flex stop tooth hangs under each X boss.
- **Wrist base** (`wrist_base`, z −18…−15, forearm side). Bundle opening 20 × 24. Plate |x| ≤ 12 beside the opening and ≤ 15 elsewhere. Two dev lugs (y 22–27, Ø13.2 boss) with 623ZZ seats, and an arc slot in each lug's inner face, which is the **dev hard stop**. 4 × M3 to the forearm top at (±12.5, ±18).
- **Palm flange** (`palm_flange`, z 10…15) + **sheath clamp bridge** (`palm_sheath_clamp`, z 14…19, a laminated comb of 6 plates pinched by 2 × M2.5 along Y).
  - Palm lugs at x ±(16–19.5) take the pressed flex pins.
  - An arc wall on each lug's inner face catches the ring tooth: the **flex hard stop** at ±52°.
  - Push‑rod ears with the 8 mm ball at (±24.2, 7.1, 2.0).
  - In |x| < 11.5 the flange stays inside r ≤ 18 mm of the flex axis, and at 11.5–14.5 only |y| ≤ 3.5 is kept, so nothing palm‑side sweeps into the ring.
- **Stops (check at 6° past each stop):** flex overlap 9.6 mm³ each way, dev 1.7 mm³ each way, so all four engage. No contact inside the range.
- **Drive** (`wrist_kin.DRIVE`, from `optimize_drive("Y")`):
  - Crank axles along Y at (±22.0, 26.8, −79.0), crank r 19.9, rod 84.7 mm.
  - STS3215 bodies at x ±(13.7–38.4), y −26…9, z −95…−50, shaft at (±26, 9, −61.6).
  - 14T:36T module 0.7 gears (pinion r 5, crank gear r 12.9).
  - The rods run behind the servos and come out of the forearm top at x ≈ ±22…31. They are exposed across the gap, like human extensor tendons; cover them with a sleeve or bellows.
  - Ball link Ø6 at the crank end.

### Torque check (STS3215 12 V stall 2.94 N·m, continuous taken as 30 % = 0.9 N·m, verify datasheet)

| Need (earlier study) | N·m | Capacity (min over the range / median) |
|---|---|---|
| hand alone | 0.20 | flex 3.63 / 5.08 stall · 1.11 / 1.56 continuous |
| + 0.5 kg power grasp | 0.74 | ✓ continuous everywhere |
| + 1 kg power grasp / 0.5 kg at the tips + tube spring | ~1.3 (target) | ✓ continuous over most of the range; 1.1 at the flex ends (85 %) |
| peak | 2.5 | ✓ (3.6 stall at the worst pose) |
| dev | same loads | ≥ 16 stall, ≥ 4.9 continuous: oversized, because the palm balls must sit outside the palm lugs (x ≥ 24) |

- **Rod force:** at most 187 N per N·m of wrist torque, so 470 N at 2.5 N·m. An M4 steel rod (85 mm) buckles at ~1.4 kN.
- **Pins and bearings:** a 3 mm dowel in double shear is fine, but 2 × 623 (C0 ≈ 0.22 kN each) is marginal at peak (~235 N each). See §10.

## 5. Sheath bundle: path, bends and slack

- **Path:** a free bundle from the palm clamp face (palm frame z +14) to the forearm comb (z −60), modelled as the least‑bent cubic (tangent lengths optimised per pose). The section frame is rotation‑minimising, with the twist spread evenly. Every tube keeps its cell, which is the worst case.
- **Bends:** worst tube R **24.4 mm** at flex −50/dev −30 (limit 15); centreline ≥ 21.7. Free span 74 mm at q = 0.
- **Length change of each tube across the wrist:** −6.7…+6.1 mm worst (table below). The sheath ends are fixed (palm clamp and forearm adjuster), so this is length the sheath must find. It slides through the comb and the 60 mm forearm fan below it takes it up as a change of bow.
  - **Rule: cut each sheath at its q = 0 path length + 7 mm**, giving a slight pre‑bow in the fan.
  - The fan's S‑bends stay ≥ 22 mm radius.
  - The **line** inside a sheath does not change length (Bowden); only bore‑clearance slack is left (~0.1 mm per 10° of bend with a 1.0 bore).
- **Friction:** the wrist bend is at most arccos(cos 52°·cos 32°) = 58°, so the wrist keeps e^(−0.1·1.0) ≈ 0.90 of the tendon force at μ 0.1 (0.78 at μ 0.25). That is better than the 73° of the old range.

| ID | joint | slot (side, level / column) | strand | bundle cell (x,y) | wrist ΔL min..max (mm) | fan min R (mm) |
|---|---|---|---|---|---|---|
| 1 | index_pip | R1/1 | flex / ext | 14 (+3.2,−1.1) / 15 (+5.4,−1.1) | −2.6..+2.5 / −3.4..+3.8 | 71 / 37 |
| 2 | index_mcp_flex | R2/3 | flex / ext | 30 (+1.1,+5.4) / 26 (+3.2,+3.2) | −5.4..+4.8 / −4.4..+4.3 | 23 / 37 |
| 3 | index_mcp_abd | R1/2 | flex / ext | 20 (+3.2,+1.1) / 21 (+5.4,+1.1) | −2.6..+2.5 / −3.4..+3.8 | 46 / 88 |
| 4 | thumb_ip | R1/0 | flex / ext | 9 (+5.4,−3.2) / 3 (+3.2,−5.4) | −5.3..+5.6 / −6.3..+6.1 | 45 / 27 |
| 5 | thumb_mcp_flex | R2/2 | flex / ext | 25 (+1.1,+3.2) / 19 (+1.1,+1.1) | −3.5..+3.0 / −1.7..+1.2 | 62 / 98 |
| 6 | thumb_cmc_flex | R2/1 | flex / ext | 13 (+1.1,−1.1) / 8 (+3.2,−3.2) | −1.7..+1.2 / −4.4..+4.3 | 126 / 50 |
| 7 | thumb_cmc_rot | R2/0 | flex / ext | 7 (+1.1,−3.2) / 2 (+1.1,−5.4) | −3.5..+3.0 / −5.4..+4.8 | 46 / 28 |
| 8 | middle_pip | L2/0 | flex / ext | 6 (−1.1,−3.2) / 1 (−1.1,−5.4) | −3.6..+3.0 / −5.4..+4.8 | 46 / 28 |
| 9 | middle_mcp_flex | L2/1 | flex / ext | 12 (−1.1,−1.1) / 11 (−3.2,−1.1) | −1.9..+1.1 / −3.1..+2.0 | 126 / 41 |
| 10 | middle_mcp_abd | R1/3 | flex / ext | 31 (+3.2,+5.4) / 27 (+5.4,+3.2) | −6.3..+6.1 / −5.3..+5.6 | 22 / 34 |
| 11 | ring_pip | L1/0 | flex / ext | 5 (−3.2,−3.2) / 0 (−3.2,−5.4) | −4.9..+3.9 / −6.7..+5.7 | 42 / 27 |
| 12 | ring_mcp_flex | L2/2 | flex / ext | 24 (−1.1,+3.2) / 18 (−1.1,+1.1) | −3.6..+3.0 / −1.9..+1.1 | 62 / 98 |
| 13 | ring_mcp_abd | L1/1 | flex / ext | 10 (−5.4,−1.1) / 4 (−5.4,−3.2) | −4.4..+2.9 / −6.2..+4.7 | 106 / 43 |
| 14 | little_pip | L1/2 | flex / ext | 17 (−3.2,+1.1) / 16 (−5.4,+1.1) | −3.1..+2.0 / −4.4..+2.9 | 46 / 88 |
| 15 | little_mcp_flex | L2/3 | flex / ext | 29 (−1.1,+5.4) / 23 (−3.2,+3.2) | −5.4..+4.8 / −4.9..+3.9 | 23 / 37 |
| 16 | little_mcp_abd | L1/3 | flex / ext | 28 (−3.2,+5.4) / 22 (−5.4,+3.2) | −6.7..+5.7 / −6.2..+4.7 | 22 / 34 |

The cells are assigned by a Hungarian match from the adjuster field to the bundle, so the fan does not cross. The same cell is used at the palm clamp, so the palm integrator routes from these cells (`interface()["palm_flange"]["sheath_clamp"]["tube_cells_xy"]`). Cell = row‑major index from −y, rows of 4,6,6,6,6,4 at a 2.15 pitch. Thumb strands sit on the +x/−y side.

## 6. Interfaces

### 6.1 Palm flange (for the palm integrator)

- **Palm base face z = +15.** The flange occupies z 10…15. Outline x ±21.5 × y ±12.5, corner r 5. In |x| < 11.5 it is cut to r ≤ 18 from the X axis (so |y| ≤ 9.9 at z 15), and at 11.5 < |x| < 14.5 only |y| ≤ 3.5 is kept.
- **Bolts:** 4 × M3 heat‑set inserts (Ø4.0 × 5 deep) in the flange at **(±17.0, ±9.5)**, screwed from inside the palm.
- **Bundle:** the opening below the clamp is 16 × 16 (r 3). The clamp bridge stands **4 mm proud of the palm face** (z 15…19, x ±11.15, y ±8.65), so the palm needs a pocket ≥ 23 × 18 × 4. Sheaths leave the clamp top at z 19 straight up (+Z), in the 32 cells above. Sensor cable Ø3 at (0, −8.65).
- **Palm keep‑out (rounded heel), required for ±52° flex:**
  - For z < 36 and |x| < 9.5, palm material must be inside r ≤ 18 from the flex axis **or** inside a ±20° wedge about +Z (in front and behind the dev lugs).
  - For z < 22, palm material must be inside r ≤ 18 for |x| < 11.5, and within |y| ≤ 3.5 for 11.5 < |x| < 14.5.
  - Everything else (|x| > 14.5) is free.
  - With a plain 28 mm palm box, the ring, base and dev pins collide at ±50° (`collisions_plain_palm` in the summary). This heel is the price of crossing axes in 30 mm.
- **Push‑rod balls** (part of the flange) at (±24.2, 7.1, 2.0). The palm must leave x 19.9…28.5, y 3.5…10, z −2…15 free.
- **Thumb:** none of the wrist parts reach the THUMB_CMC_REGION box over the ±52/−32…+22 range. The wrist servos at the top of the forearm sit at y ≤ 9 and z ≤ −50.

### 6.2 Forearm (from `forearm.interface()`)

- **Per servo ID:** side, level, column y, spool centre and axis, and for flex/ext the spool tangent point, the adjuster (sheath start) point and the bundle cell. Spool groove planes are |x| 10.8 ± 1.1 (level 1) and 5.8 ± 1.1 (level 2). Level 1 at z −141, level 2 at z −175. Columns y −20.25 / −6.75 / 6.75 / 20.25.
- **Slots:** right = thumb + index + middle_abd; left = middle pip/mcp_flex, ring, little. Level 2 (lower, 5 mm deeper) holds **2, 6, 7, 9, 12, 15** (the 4 mcp_flex, thumb cmc_flex and cmc_rot, with +2.25 mm room for HLS3606M) plus 5 and 8.
- **Wrist top:** 4 × M3 inserts at (±12.5, ±18) on the frame's top ring at z −18.
- **Arm mount:** a Ø72 × 5 flange at z −230…−225, **4 × M4 on PCD 50, Ø24 bore**. This stands in for OpenArm J5's flange (sim site `forearm_flange`); verify the real pattern.
- **Envelope:** x ±47.5, y ±33, z −230…−18.

## 7. Forearm design

- **Cassettes:**
  - Two stepped plates (`cassette_R/L`), each with 8 SCS0009 windows.
  - Level‑2 servos sit 5 mm closer to the rail, so their strands run inside the level‑1 spools. This is the V1 "deeper level closer to centre" trick, turned sideways so the forearm is wider than deep.
  - A servo slides out sideways (along its shaft) after removing a side cover and its 2 tab screws. The spool stays on its rail stub, still strung and tensioned, and **nothing is restrung**.
  - The spool's spline hub is a cut‑down stock horn pressed into the printed spool. It is a slip fit with no horn screw, captive between the rail and the servo.
- **Spool rail** (`spool_rail`, x ±3):
  - MR63 seats for the level‑2 spool stubs on both faces.
  - Ø6.4 posts with a 3 × 4 brass sleeve for level 1. The posts pass 1.8 mm beside the level‑2 strands.
  - So the ~40 N strand side load (tendon‑mechanics §3.1) is shared by the servo shaft and the rail.
- **Split spool = tensioner:** two discs (one groove each, r 5, flanges r 6.5), turned against each other and locked by an M2 screw through serrated faces (≈ 0.6 N·m holding, 2.6× SCS0009 stall). This is the coarse adjustment.
- **Adjuster block** (`adjuster_block`, z −132…−123):
  - 32 M2.5 socket screws drilled Ø1.0 through. The strand passes through the screw, and the sheath end sits in the 2 mm hex socket.
  - 0.45 mm per turn, for fine tension and slack take‑up, set from the top through the open front.
  - Odd columns stand on 6 mm risers, so heads at the same height are ≥ 5.0 mm apart (head 4.5).
  - **Strands run bare, straight and tangent from the spool to the adjuster** (9–43 mm).
- **Fan and comb:** 32 S‑curved sheaths from the adjusters to the comb at z −60 (`bundle_comb`, Ø2.3 slide holes). Minimum bend **22.2 mm**. The fan needs ≥ 50 mm of height, which is why the bay starts at z −118.
- **Neck** (z −18…−118): the sheath fan in the middle, wrist servos left and right (x 13.7–38.4), and crank axles on bosses in the back shell. The cranks and rod balls set the 95 mm width. The shell top opens to 72 × 59 so the rods pass.
- **Electronics** (tray z −214):
  - ESP32‑S3‑DevKitC‑1 (69 × 26).
  - FE‑URT‑1 (≈48 × 23, verify).
  - Power distribution board (30 × 23), with 3 hand‑servo branches + caps and a separate 12 V branch for the STS3215s.
  - The bus runs up the rail's cable slot to the daisy‑chained servos.
- **Shell:**
  - Structural back half (`forearm_shell_back`, carries the crank axles).
  - Removable front cover (access to the adjusters, comb and electronics).
  - Two side covers (servo removal).
  - Lofted from 72 × 59 at the top to 95 × 66 at z −48.

**Forearm check results:** static interference is clear, and the 9 extreme wrist poses are clear (§12, `forearm_check.log`, `forearm_summary.json`).

## 8. Assembly and tensioning order

1. Press the 623s into the ring's X bosses and the base lugs. Press the dev pins into the ring and the stop pins into the ring's Y ends. Fit the ring into the base, insert the dev pins through the lug bearings, and check that the dev stops sit at −32/+22.
2. Press the flex pins through the palm lugs into the ring bearings. Check the flex stops at ±52. The gimbal should move freely empty, with < 0.5° play.
3. Cut 32 PTFE tubes to the q = 0 path + 7 mm, labelled `NNf/NNe` on both ends. Thread them from the palm clamp, through the ring, to the comb and on to the adjuster sockets, with the wrist straight. Clamp the laminated palm clamp (2 × M2.5) **lightly**. Don't tie anything across the gap.
4. Fit the servo cassettes (servos centred, torque off), rail and spools.
5. Pass pre‑stretched braid from each spool groove up through its adjuster and sheath to the finger drum. Tie at the drum (extra wrap, CA). At the spool, wrap and anchor each strand in its own disc.
6. **Coarse tension:** turn spool disc B against disc A until both strands are just taut at q = 0, then lock the M2.
7. **Fine tension:** use the barrel adjusters (0.45 mm/turn) to reach a 3–5 N pretension (pluck pitch, tendon‑mechanics §3.2). Then tighten the palm clamp fully.
8. Fit the wrist servos, gears, cranks and rods with the wrist at q = 0 (servo mid position). Sweep the wrist with the fingers held: no finger joint should move > 1°.
9. Close the front and side covers.

**Re‑tensioning** uses the adjusters only (front cover off). **Swapping a servo:** side cover off, 2 screws, pull it out, push the new one in at the same position, recalibrate the zero.

## 9. Failure modes (what fails first)

1. **Braid sawing at the adjuster exit.** The strand leaves the drilled screw at a bare edge. Chamfer and polish the Ø1.0 bore exit, or press in a 1 mm stainless eyelet. This is the most likely wear point.
2. **623 static rating at peak wrist load** (~235 N per flex bearing). Upgrade to 4 mm pins + 624 (C0 ≈ 0.48 kN) if the wrist is driven to stall often.
3. **Rod‑end ball joints:** Ø6 ball links at the crank carry up to ~470 N at 2.5 N·m. Use metal ball links, not RC‑car plastic ones.
4. **Printed lug layers:** print the base and flange with the lugs standing (bores horizontal in the build plane), so pin loads go along the layers.
5. **Sheath fan pre‑bow lost** (tubes pulled straight): the fingers would then move with the wrist. The 7 mm extra length and the comb slide guide prevent it; acceptance test = step 8.
6. **Split‑spool slip:** witness mark, 5 kg hang test (tendon‑mechanics).
7. **STS3215 overheating** when holding 1 kg at the flex ends (1.1 N·m continuous capacity there). Watch servo temperature, or use a higher‑torque servo of the same size (e.g. Dynamixel XM430‑W350, 4.1 N·m, would give ≥ 5 N·m flex).

## 10. Requested params changes

1. **`WRIST_RANGES["wrist_flex"] = (-50, 50)`** (§3), or `WRIST_GAP ≥ 45–50` if ±70 is required (not designed here).
2. **Joint order:** wrist_dev outer (forearm → dev → ring → flex → palm). The sim (`convert_v1_wrist.py`) should follow, or a remap should be used.
3. **`WRIST_OPENING = (24, 37)`** (the bundle needs 19 × 29.4 at the ring with ±52° flex; 22 × 14 is far too small).
4. **`FOREARM_Z_TOP = -18`** for the plate face (the wrist base plate is z −18…−15).
5. New: `PALM_HEEL` keep‑out (§6.1), `FOREARM_ENVELOPE = (±47.5, ±33, −230…−18)`, `ARM_FLANGE = 4 × M4 / PCD 50`.
6. Optional: `PIN_D_BIG = 4` for the wrist (bearings, §9.2).

## 11. Open issues

- **Dev is 4–5× oversized while flex is the limit.** The palm balls must sit outside the palm lugs (x ≥ 24), so dev takes most of the rod stroke. A separate planar 4‑bar for dev, plus flex on two rods driven by one servo, could double flex torque. Not done.
- The cranks set the forearm width (95). A re‑optimisation with "crank radius + ball ≤ 41" would bring it to ~86.
- Servo dimensions (STS3215 shaft offset 11 mm, HLS extra height below the tabs) and FE‑URT‑1 size are from memory: **verify**.
- OpenArm J5 flange pattern: verify.
- The exposed rods in the gap need a cover (sleeve or bellows) for safety.
- The bundle model is first‑order (fixed cells, least‑bent Hermite curve). The printed mock‑up (earlier study, step 3) is still the real test, now with the ±52° ring.
- Wrist‑servo mass (2 × 55 g) and the gears sit in the upper forearm; there is no mass/inertia table yet.

## 12. Status of the final runs (2026-11-10)

- `wrist.py`: 17 parts, all valid. Stops: flex ±52 overlap 9.6 mm³ and dev −32/+22 overlap 1.7 mm³ at 6° past each stop (they engage). Collision grid flex {−50, −25, 0, 25, 50} × dev {−30, −15, 0, 20} with the heel palm proxy and the sheath bundle: **all 20 poses clear**. With a plain 28 mm palm box, the ring, base and dev pins hit the palm at ±50° flex (that is the heel requirement).
- `forearm.py`: 80 parts, all valid. **Static interference clear.** The moving wrist rods, cranks, flange, ring and bundle at flex {−52, 0, 52} × dev {−32, 0, 22} against the shell, frame, comb, adjuster block and the 32 sheaths: **all 9 clear**. Sheath fan min bend 22.2 mm. Adjuster heads 5.0 mm apart.
- Exports: `out/wrist/wrist.step|glb`, `forearm.step|glb`, `stl/`. Renders: `wrist_q0.png`, `wrist_flex±50.png`, `wrist_dev-30.png`, `wrist_flex+50_dev+20.png`, `forearm_open.png`, `forearm_closed.png`.
