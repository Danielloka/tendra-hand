# 04 · Integration: palm, routing, full assembly (V2 concept)

Study 2026-11-09, integrator. New code: `cad/routing.py` (32 strand paths), `cad/palm.py` (palm), `cad/assembly.py`
(full hand + wrist + forearm at a pose, export, renders), `cad/assembly_checks.py` (cross-module checks).
Outputs: `out/assembly/` (`tendra_v2_open.step` 62 MB, `tendra_v2_open.glb` 8.1 MB, `stl/` 225 parts, `pose_*.png`,
`palm_only.png`, `routes.json`, `checks.json`, `mass.json`, `poses.json`, `opposition_rerun.json`, `palm_cache/`).

```
RUN="uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy --with pillow --with rtree python"
$RUN cad/routing.py            # route all 32 strands from scratch (~11 min); then: --polish (~9 min) -> routes.json
$RUN cad/palm.py               # palm solids (~3 min, then cached as BREP in out/assembly/palm_cache)
$RUN cad/assembly.py           # 7 poses, STEP/GLB/STL, renders: 2.5 min with the palm cache (~5.5 min without)
$RUN cad/assembly_checks.py    # validity + cross-module interference + wrist bundle (~8 min)
```

## 1. Summary (read this first)

- The hand closes: **every part is a valid solid (220/225 watertight; the 5 that are not are the wrist designer's
  gear/rod and one forearm sheath mesh)**, the assembly exports to STEP/GLB, and the open pose and the wrist
  dev −30° pose have **zero interference**. The wrist at ±50/−30…+20 has at most 59 mm³ of palm–forearm-cover contact.
- **The routing is the big finding.** Two module interfaces made the brief's "PTFE everywhere, R ≥ 15" impossible
  as delivered:
  1. **The thumb's 8 strand exits sit at z 22–26, only 3–7 mm above the clamp top (z 19), 21–50 mm to the side,
     pointing away from it.** A PTFE path needs ≥ 28 mm of height for that, so it cannot be done. The 8 thumb
     strands therefore run **bare inside bent stainless guide tubes (18G hypotube, R 5, "bike-brake noodle")**
     from the clamp to the thumb mount. The 2 cmc_rot strands first wrap an **idler pulley** (115–125°, rolling, not
     sliding), because their exits point palmar-radial-down.
  2. **The thumb mount (Ø31 housing + Ø42 flange) fills x ≥ 8.5 at z 30–50 under the index column.** The index
     sheaths get past it only because I let the finger's 22 mm **free span pre-turn** (the palm guide is tilted
     up to ~40° instead of pointing −Z). It is re-checked at abd ±15° with the moved yoke exit.
- **24 finger sheaths in PTFE:** min bend radius **13.8–24.1 mm** (3 strands are 1–8 % under 15), wall clearance
  **0.08–0.56 mm** (16 of 24 under the 0.4 target, none touching), free span at abd ±15 ≥ 14.0 mm (3 under 15).
  These are near-misses of an optimiser, not geometric impossibilities (§3).
- **Cell map:** the wrist/forearm map needed **3 swaps + a re-ordering of the 6 thumb-axial cells**, so that nothing
  crosses in the palm. The forearm fan then has **22 tube pairs closer than one OD** (1 with the old map). The forearm
  fan must be re-routed with this map: open issue #3.
- **Thumb opposition with the final finger geometry:** score **32.09** (designer 32.22; abd offset 11.5). But
  **the palmar knuckle bulge that the abd dowels need (y −18.1, + TPU −19.6) gets in the thumb's way.** With a
  palm as thick as the bulge, the score is 52.3: the little pad gap grows from 0.1 to 5.6 mm. I carved a thenar hollow
  for the ring/little paths (the index dowel boss is kept), but the clean fix is open issue #2.
- **Totals:** 756 g including forearm, wrist and 18 servos; the hand alone (fingers + thumb + palm) is **170 g**.
  Wrist centre to middle fingertip 192 mm. Palm 86 mm wide, 28 mm thick (34 mm at the knuckle row).

## 2. What I built

### 2.1 Palm (`cad/palm.py`), see `out/assembly/palm_only.png`

| Part | Material | What it does |
|---|---|---|
| `palm_shell` | PETG (83 cm³) | 28 mm body (y −14…+14) with a scalloped knuckle edge. **Palmar/dorsal knuckle lips** (y −18.1…−13.5 / +11.1…+15.6) hold the 2 × Ø3 abd dowels per finger (2.9 press-fit bores, through from outside). The swept yoke envelope of each finger is cut between the lips. A **free-span pocket** under each yoke gives the 4 sheaths room to bend with abduction. **Bare-strand slots** run from each abd mouth to the yoke. **24 PTFE channels** (Ø2.2 swept bores) run from the guide/mouth to a common **bundle well** (15.6 × 14.8 × 6 mm) over the clamp. **Thumb seat:** zero-clearance pocket for the mount, flange/head clearance, base-sweep hull + 1 mm, 3 × M3 heat-set insert holes along −a1. **Thenar heel block** (x 8…52, y −31…−10, z 15…27) carries the 2 cmc_rot idlers and the open **trough** for the 8 steel guide tubes. **Wrist seat:** flat on the palm flange (z 15). Clamp pocket. 4 × M3 × 8 from inside: the x +17 screws go in before the thumb, through the thumb bay; x −17 dorsal under the back plate; x −17 palmar under the palm pad. **Heel keep-out** cut exactly as report 03. Rounded heel corners carved by the forearm envelope seen from the palm at flex ±37.5/±50 × dev −30/0/+20. **Thenar hollow** carved by the thumb metacarpal / thenar pad / proximal at the ring and little oppositions, keeping a Ø9.2 boss round the index palmar dowel. |
| `palm_back_plate` | PETG 3.5 mm | Dorsal skin over the central palm (x −42…6, z 24…63), 6 × M2. Opens the service bay (dorsal flange screw; dorsal channels). |
| `palm_pad`, `hypothenar_pad`, `knuckle_pad` | TPU 95A | 2 mm central pad, a hypothenar wedge (4 mm at the ulnar edge), 1.5 mm over the knuckle lip. The edges stand proud of the centre, so the **palm face is cupped**. |
| `palm_sheaths` | PTFE 1 × 2 | The 24 finger sheaths, yoke exit / abd mouth → clamp top (palm part only). |
| `thumb_guides` | 18G stainless | 8 bent guide tubes, clamp (z 15.5) → thumb mount sockets. |
| `rot_idlers` | MR63 + tyre, Ø3 axle | The 2 cmc_rot idlers. |

**Print:** shell lying on its back (dorsal face down; the knuckle lips and channel curves lie in the build plane;
supports only under the palmar lip). Back plate inner face down. Pads outer face up. **Assembly order:** palm onto
the palm flange (4 × M3) → push the 24 PTFE tubes down through the channels into the clamp → lay the 8 pre-bent guide
tubes in the trough, fit the idlers → thumb mount (3 × M3) → fingers (abd dowels from outside) → back plate, pads.

### 2.2 Routing (`cad/routing.py` → `out/assembly/routes.json`)

**PTFE paths** are degree-7 Bézier curves. Their first and last three control points are collinear with the
start/end direction, so each path is **G2** where it joins the yoke socket and the clamp. The free control points are
optimised one strand at a time (Powell, then an L-BFGS-B polish pass with a rising clearance weight). The objective
covers: length, total bend (it is capstan friction), curvature over 1/16.8, clearance (2.4 mm centre to centre; 2.1
in the last 4 mm over the clamp, where the cells are on a 2.15 pitch), the thumb mount + base sweep voxels (+3 mm =
bore + 1.6 wall), the flange bolts, the palm skin (|y| ≤ 11.2), staying above the clamp, and the **free span re-checked
at abd ±15** (least-bent cubic from the moved yoke exit to the fixed guide).

**Steel guide tubes** are filleted polylines (straight + R 5 arcs, which is what a tube bender makes). The corners are
grid-searched for the least bend, and the 8 tubes are placed greedily; the best of 14 placement orders is kept.

### 2.3 Assembly (`cad/assembly.py`)

`build(pose)` places every module part in the hand frame. The hand side (fingers, thumb, palm, palm flange, clamp)
moves with `R_palm = Ry(dev)·Rx(flex)`. Part names are prefixed per module (`<finger>_*`, `T_*`, `P_*`, `W_*`, `F_*`).
Colours: PETG per group (sand fingers/thumb, blue-grey palm/wrist/forearm), TPU dark, steel grey, PTFE white,
servos black. GLB 8.1 MB (mm, Z up, not decimated).

## 3. Param / module changes applied

| Change | Where | Why |
|---|---|---|
| `ABD_OFFSET_DEFAULT` 10 → **11.5**, `KNUCKLE_MIN_W = 17` | params | finger request |
| Finger MCPs stay **y = −4**; palm bulges locally (`PALM_KNUCKLE_BULGE` −18.1 / +15.6) | params, palm | keeps the thumb optimisation valid. The bulge later proved to block the thumb, see issue #2 |
| `ABD_PAIR_LIMIT_DEG = −1` (neighbours may only spread) + ring/little caveat | params | spacing kept at 19.5–20; measured: 18 mm fingers touch after ~1° of relative adduction |
| `THUMB_CMC_REGION.z` → (10, 55); `THUMB_P`, `THUMB_AXES`, `THUMB_RANGES`, `THUMB_LENGTHS_HINT` 46.7/36.0/31.6, `THUMB_MOUNT` (3 × M3, r 18.5), `THUMB_HLS` | params | thumb requests |
| `WRIST_RANGES.flex` (−50, 50), `WRIST_RANGES_ALT` (−60…70 needs a 45–50 mm gap), `WRIST_ORDER` dev-outer, `WRIST_OPENING` (24, 37), `FOREARM_Z_TOP` −18, `PALM_HEEL`, `WRIST_FLANGE_BOLTS`, `SHEATH_CLAMP_Z`, `FOREARM_ENVELOPE`, `ARM_FLANGE` | params | wrist requests |
| `PIN_D_WRIST = 3` (**624 / 4 mm not adopted**) | params | a 624 (13 OD) needs ring bosses r ≥ 8.1 against r 6.6 in a 7 mm ring, so it does not fit without a wrist rework |
| `PALM_CELL_SWAPS` (13↔15, 19↔21, 25↔27) + thumb-axial re-order; `CLAMP_THUMB_STEP_Z = 15.5` | params, routing | otherwise the thumb tubes must cross the index tubes rising from their cells |
| **Clamp stepped down to z 15.5 over the 8 thumb cells** | `assembly.clamp_stepped()` (applied to the wrist's `palm_sheath_clamp`; `wrist.py` untouched) | lets the guide tubes turn down 3.5 mm lower. The thumb PTFE is clamped over z 14–15.5 only (glue) |
| Power-grasp pose re-solved by `thumb_kinematics.task_power` with `PALM_Y[0] = −19.6` (knuckle pad face) | assembly | the cylinder rests on the knuckle pads, 3.8 mm further out than the designer's slab |

Not changed: `finger.py`, `thumb.py`, `thumb_kinematics.py`, `wrist.py`, `wrist_kin.py`, `forearm.py` (all read-only).

## 4. Routing results (palm part; `routes.json` has the full rows)

Cell = clamp cell (wrist numbering). R = min bend radius. Clear = wall gap to the nearest tube above z 24.
Turn = total bend in the palm (μ 0.1 → e^(μ·turn)). Free = free-span min R at abd ±15.

| strand | ID | cell | mat | L mm | R mm | clear | turn° | free | ok |
|---|---|---|---|---|---|---|---|---|---|
| index_mcp_flex flex/ext | 2 | 30/26 | PTFE | 64.3/63.8 | 15.6/15.9 | 0.38/0.34 | 109/112 | 15.7/16.9 | gap |
| index_pip flex/ext | 1 | 14/13 | PTFE | 59.9/65.7 | 24.1/16.1 | 0.54/0.20 | 59/143 | 18.3/**14.8** | ✓/gap,free |
| index_mcp_abd flex/ext | 3 | 20/19 | PTFE | 56.1/52.5 | **13.8**/16.0 | 0.32/0.22 | 73/70 | – | R/gap |
| middle_mcp_flex flex/ext | 9 | 12/11 | PTFE | 62.7/62.8 | 20.3/16.4 | 0.20/0.08 | 52/76 | 19.3/18.7 | gap |
| middle_pip flex/ext | 8 | 6/1 | PTFE | 65.2/63.5 | **14.8/14.4** | 0.28/0.35 | 125/97 | 17.0/21.2 | R |
| middle_mcp_abd flex/ext | 10 | 31/25 | PTFE | 55.5/57.1 | 15.3/16.1 | 0.41/0.27 | 91/125 | – | ✓/gap |
| ring_mcp_flex flex/ext | 12 | 24/18 | PTFE | 68.5/63.3 | 16.6/16.9 | 0.26/0.34 | 126/120 | **14.2**/16.4 | gap |
| ring_pip flex/ext | 11 | 5/0 | PTFE | 66.6/63.3 | 15.6/15.6 | 0.38/0.48 | 154/118 | 16.6/21.8 | gap/✓ |
| ring_mcp_abd flex/ext | 13 | 10/4 | PTFE | 54.3/63.2 | 16.0/16.1 | 0.44/0.56 | 122/168 | – | ✓ |
| little_mcp_flex flex/ext | 15 | 29/23 | PTFE | 78.8/63.8 | 16.4/16.3 | 0.29/0.33 | 202/147 | 16.4/15.1 | gap |
| little_pip flex/ext | 14 | 17/16 | PTFE | 68.4/63.9 | 15.2/16.5 | 0.08/0.43 | 154/141 | **14.0**/16.5 | gap,free/✓ |
| little_mcp_abd flex/ext | 16 | 28/22 | PTFE | 54.5/65.7 | 15.2/16.0 | 0.47/0.49 | 145/194 | – | ✓ |
| thumb_cmc_rot flex/ext | 7 | 7/2 | steel + idler | 47.9/63.5 | 5.0/5.0 | ≥ 0.23* | 160/199 (+125/115 idler) | – | ✓ |
| thumb_cmc_flex flex/ext | 6 | 21/9 | steel | 24.1/27.7 | 5.0 | ≥ 0.23* | 164/195 | – | ✓ |
| thumb_mcp_flex flex/ext | 5 | 8/3 | steel | 26.2/27.7 | 5.0 | ≥ 0.23* | 179/189 | – | ✓ |
| thumb_ip flex/ext | 4 | 27/15 | steel | 25.2/28.5 | 5.0 | ≥ 0.23* | 160/188 | – | ✓ |

\* The steel tubes run below z 24, where the placement enforces a 1.5 mm centre distance (0.23 mm wall gap).
The finite-difference radius in `routes.json` reads 4.7–4.8 because of sampling; the arcs are exactly R 5.

**Totals:** PTFE in the palm 1503 mm (24 tubes). The steel guide tubes bend 160–199° each, i.e. **×1.32–1.42
friction at μ 0.1** on the thumb strands, before the thumb's own ×1.25. The finger sheaths bend 52–202° in the palm
(×1.10–1.42); the little finger is the worst because it sits furthest from the bundle.

**Decoupling still holds:** every palm path is fixed in the palm frame (constant length). The free span is a Bowden
span clamped at both ends (yoke socket, palm guide), so the module tables in reports 01/02 (finger ≤ 0.19 mm,
thumb ≤ 0.123 mm) are unchanged. The steel guide tubes and idlers are palm-fixed too.
**Wrist bundle (wrist_kin model, same set of cells):** min tube bend **24.4 mm** at flex ±50 / dev −30
(28.0 at dev +20, 31.9 at dev 0), so ≥ 15 at every wrist extreme. **Forearm fan with the new cell map:** min R
22.8 mm, but **22 tube pairs closer than one OD** (needs a fan re-route; open issue #3).

## 5. Checks (`out/assembly/checks.json`; threshold 0.5 mm³; only module-crossing pairs, since the designers cleared their own)

| Check | Result |
|---|---|
| Validity (225 parts, open pose) | **220 ok.** Not watertight: `W_wrist_gears_R/L`, `W_wrist_rod_R/L` (2–3 solids each), `F_sheath_05e` (wrist/forearm modules) |
| a open hand | **clear** |
| b fist (finger 90/95, thumb in key position) | 19 hits: little distal/tip into palm + knuckle pad (118 / 97 mm³), proximal pads onto the knuckle lip (21–26 mm³). **Full 90/95 is not reachable: the fingertips meet the palm** (as in a real fist); the lip costs a few degrees at the MCP |
| c tip pinch (optimiser angles) | index tip × thumb tip **195 mm³**: the optimiser's point pads meet at 0 mm, so the real pads overlap ~2–3 mm. In hardware the pinch stops at contact, a few degrees earlier |
| d thumb–little (ring/middle/index follow the little) | pads overlap 241 mm³ (same reason as c); **ring/little yokes 185 mm³** at +14/+15 (staggered pivots); thumb metacarpal × knuckle pad 27 mm³ (after the thenar carve) |
| e power grasp Ø55 (lip-aware thumb solve) | thumb proximal × cylinder 69 mm³ (≈ 1 mm), pad 1.5 mm³; fingers wrap at the computed max (no finger contact) |
| f wrist flex +50 + pinch | same 4 pinch contacts as c; **wrist / forearm clear** |
| g wrist dev −30 | **clear** |
| Thumb × index (6 thumb poses × index open/mid/fist) | open index: clear except thumb `index`/`middle` opposition × knuckle pad 3–14 mm³. Mid/fist index: large overlaps for every opposition pose. These are combinations a planner must exclude (the thumb then occupies the index's space), not design faults |
| Thumb base/metacarpal vs palm (rot −31/+40/+110 × flex −37/+13/+63, MCP/IP 0) | **8/9 clear**; rot −31 / flex +63 corner: metacarpal into the palm 1285 mm³ → firmware limit (block rot < −20 with flex > 55) |
| Neighbour fingers, toward each other ±7/10/15 at MCP 0 and 90 | all collide (≥ 95 mm³), as the geometry predicts → `ABD_PAIR_LIMIT_DEG = −1`; ring/little needs a pair table |
| Palm vs wrist + forearm at flex ±50 × dev −30/+20 | −50/+20 clear; −50/−30 2.2 mm³; +50/−30 4.9 mm³; **+50/+20 59 mm³** palm heel × forearm front cover + idler 2.3 mm³ |
| Sheaths ≥ 15 at the wrist extremes | ✓ (24.4 min) |
| Thumb opposition rerun (`opposition_rerun.json`) | final finger geometry: **32.09**; lip-aware palm (slab at −19.6): **52.3** (little 5.6 mm / 23°, ring 1.2 mm / 23°, others unchanged) |

## 6. Pose renders (`out/assembly/`)

`pose_a_open.png` (with forearm, 4 views), `pose_b_fist.png` (fingers 90/95, thumb in the key/"thumbs-up"
position: a thumb-over-fingers fist was not solved), `pose_c_tip_pinch.png`, `pose_d_thumb_little.png`,
`pose_e_power55.png` (with the cylinder), `pose_f_flex50_pinch.png`, `pose_g_dev-30.png`, `palm_only.png`.
Joint angles: `poses.json`. All renders were looked at: no floating parts. The visible thin walls round the thumb
seat (bay cuts) are listed as issue #5.

## 7. Mass and dimensions (`mass.json`)

PETG at 1.27 × 40 % (as briefed), TPU 1.21, steel 7.85, PTFE 2.2, SCS0009 13 g, STS3215 55 g, electronics 30 g.

| Group | g | Notes |
|---|---|---|
| Fingers (4) | 65.3 | PETG 48 g, pins/bars/eyelets, pads |
| Thumb | 37.1 | incl. 2 × 6704 |
| Palm | 67.9 | shell 42 g, back plate, pads, PTFE in the palm, guide tubes, idlers |
| Wrist | 198.7 | 2 × STS3215 (110 g), gears/cranks/rods (steel), sheath bundle |
| Forearm | 387.3 | 16 × SCS0009 (208 g), shell/cassettes 118 g, spools, adjusters, electronics |
| **Total** | **756** | the strands are modelled as steel rods, so this is an over-estimate of ~20 g; HLS3606M on 6 joints adds ~6 × 7 g |

Dimensions: overall x −51.7…72.7, y −62.4…36.0, z −230…192 mm (422 mm long). Hand: wrist centre → middle
fingertip 192 mm. Palm 86 × 28 mm (34 mm over the knuckle lips). Forearm 95 × 66 mm.

## 8. Servo list (ID → joint → forearm slot; HLS room from report 03)

| ID | joint | slot (side, level, column y) | HLS room | palm cells flex/ext |
|---|---|---|---|---|
| 1 | index_pip | R, 1, −6.75 | | 14/13 |
| 2 | index_mcp_flex | R, 2, +20.25 | ✓ | 30/26 |
| 3 | index_mcp_abd | R, 1, +6.75 | | 20/19 |
| 4 | thumb_ip | R, 1, −20.25 | | 27/15 |
| 5 | thumb_mcp_flex | R, 2, +6.75 | **no** (thumb asks for HLS) | 8/3 |
| 6 | thumb_cmc_flex | R, 2, −6.75 | ✓ | 21/9 |
| 7 | thumb_cmc_rot | R, 2, −20.25 | ✓ | 7/2 |
| 8 | middle_pip | L, 2, −20.25 | | 6/1 |
| 9 | middle_mcp_flex | L, 2, −6.75 | ✓ | 12/11 |
| 10 | middle_mcp_abd | R, 1, +20.25 | | 31/25 |
| 11 | ring_pip | L, 1, −20.25 | | 5/0 |
| 12 | ring_mcp_flex | L, 2, +6.75 | ✓ | 24/18 |
| 13 | ring_mcp_abd | L, 1, −6.75 | | 10/4 |
| 14 | little_pip | L, 1, +6.75 | | 17/16 |
| 15 | little_mcp_flex | L, 2, +20.25 | ✓ | 29/23 |
| 16 | little_mcp_abd | L, 1, +20.25 | | 28/22 |
| W1/W2 | wrist flex/dev (2-RSS) | STS3215 ×2, x ±(13.7…38.4), z −95…−50 | | – |

## 9. BOM (whole arm-end)

| Item | Qty | Notes |
|---|---|---|
| Printed PETG parts | 16 fingers + 5 thumb + 2 palm + 4 wrist + ~10 forearm (cassettes, rail, shell, covers, frame, comb, adjuster block, flange) + 32 spool discs | thumb base cap; clamp = 6 laminated plates |
| TPU 95A pads | 11 finger + 2 thumb + 3 palm | |
| SCS0009 (HLS3606M on IDs 2, 6, 7, 9, 12, 15; thumb asks 5, 6, 7) | 16 | |
| STS3215 12 V | 2 | wrist |
| Bearings | 2 × 6704ZZ, 4 × 623ZZ, MR63 (2 palm idlers + forearm spool stubs) | |
| Steel dowels | Ø3: 8 finger abd + 2 thumb CMC + 4 wrist; Ø2: 16 finger + 4 thumb; Ø1.5: 12 posts + 8 link pins; Ø1: 4 nip pins; Ø1.6: 2 wrist stop pins | |
| Stainless | 4 DIP bars 1.2 × 3.2; 8 × 18G eyelets (fingers); 8 × 18G bent guide tubes (thumb, ~25–65 mm) | |
| PTFE 1 × 2 tube | ~10.5 m (32 × ~0.33 m) | cut each to its q = 0 length + 7 mm (report 03) |
| UHMWPE braid 0.36 mm (80 lb on the HLS joints) | ~12 m | |
| Fasteners | 4 × M3 × 8 palm–flange, 3 × M3 + inserts thumb, 6 × M2 × 6 back plate, 3 × M2 thumb cap, 2 × M2.5 clamp, 32 × M2.5 barrel adjusters (drilled Ø1.0), 16 × M2 spool locks, 4 × M3 wrist base, 4 × M4 arm flange | |
| Drive | 2 × 14T/36T m0.7 gear sets, 2 cranks, 2 × M4 rods + 4 ball ends | |
| Electronics | ESP32-S3-DevKitC-1 N16R8, FE-URT-1, power board, 5–6 V ≥ 15 A + 12 V supplies | |

## 10. Open issues, ranked by risk

1. **Thumb strand interface (high).** All 8 thumb strands need bare-strand steel guide tubes (×1.32–1.42 friction,
   UHMWPE fatigue over R 5 steel, and tubes that must be pre-bent and seated in the right order). The cmc_rot pair
   also needs two idlers. **Fix in the thumb module:** bring the strands out of the mount *upward/ulnar*, ≥ 28 mm above
   the clamp (e.g. a turning sheave block inside the base or a shorter bearing stack), or let the thumb sheaths enter
   the bundle from the side with the wrist designer re-checking the bundle.
2. **Knuckle bulge vs thumb opposition (high).** The palmar abd dowel bosses (y −18.1, + pad) get in the way of the
   thumb's ring/little opposition (lip-aware score 52 vs 32). The thenar carve removes most of it, but the little pad
   gap and the d-pose contacts show it. **Fix:** MCPs to y = −2 (the lead's alternative), local Ø9 bosses instead of a
   continuous lip, then re-run the thumb optimiser with the real palm surface.
3. **Forearm fan with the palm cell map (medium).** 22 close tube pairs. Re-route the fan (or re-order the adjusters)
   with `routes.json["cell_map"]`.
4. **PTFE near-misses (medium).** 3 sheaths at R 13.8–14.8, 3 free spans at 14.0–14.8, 16 wall gaps 0.08–0.38 mm.
   A joint (all-strands) optimiser, or 1–2 mm more palm length, should close them. A printed mock-up should check them.
5. **Thumb seat walls (medium).** The bay cuts leave thin curved walls round the mount at x 18–30, z 30–58. A designed
   thenar seat (mount sleeve + ribs) should replace the subtractive bay.
6. **Neighbour abduction (medium).** Neighbours can only spread (`ABD_PAIR_LIMIT_DEG −1`). Ring/little still collide
   at +14/+15 because their pivots are staggered → generate a per-pair (q_a, q_b, mcp) table for the firmware.
7. **Wrist extremes (low).** 59 mm³ of palm heel × forearm front cover at +50/+20, plus a 2.3 mm³ idler graze →
   chamfer the cover's top palmar lip by ~2 mm (forearm module).
8. **Pinch/opposition contact (low).** The optimiser's point pads overlap the real TPU pads by 2–3 mm (c, d). Close
   in hardware with force/current limits; recompute the pose table with pad surfaces.
9. **Fist (low).** The thumb-over-fingers fist was not solved; 90/95 finger closure is limited by palm contact
   (intended stop, but it is on TPU over the knuckle lip).
10. **Wrist hardware (low).** 624/4 mm pins not adopted (needs a wrist rework); the wrist gear/rod meshes are not
    watertight; the clamp step leaves only 1.5 mm of grip on the thumb PTFE (glue).

## 11. Fix round (2026-11-10/11, integrator; owns all modules now)

Lead's priorities 1–6. Every module change is listed in §11.7. The numbers in §§1–10 above are from round 1;
this section supersedes them where they differ.

### 11.1 Before / after

| Issue | Round 1 | Fix round |
|---|---|---|
| Power grasp Ø55 | fingers nearly straight, cylinder through fingers/palm (wrong pose) | **POWER_RESULT** |
| Fist | 90/95 into the palm; thumb "thumbs-up" | **FIST_RESULT** |
| Tip pinch / thumb–little pads | pads overlap 2–3 mm (195 / 241 mm³) | **PINCH_RESULT** |
| Thumb strands | 8 bare strands in R 5 steel guide tubes (160–199° bend each) + 2 idlers | **6 of 8 in PTFE R ≥ 15** (thumb_mcp_flex_ext R 12.7, thumb_cmc_flex_ext R 14.2), 87–107° bend each (×1.16–1.20 friction at μ 0.1 instead of ×1.32–1.42). cmc_rot pair unchanged (idler + guide tube), justified in §11.3 |
| Thumb decoupling | ≤ 0.109 mm common mode, ≤ 0.011 mm true coupling | **0.186 mm** common mode (< 0.2), **≤ 0.011 mm** true coupling |
| Knuckle bulge vs thumb opposition | opposition score 32.09 (thin slab) / 52.3 (palm as thick as the lip) | MCPs y −2, local Ø9.2 bosses flush with the 2 mm pad: **37.25** with the real palm surface (y −16); little pad 0.96 mm / 15° |
| Palm sheaths (24 finger + 6 thumb PTFE) | R 13.8–24 (3 < 15), 16 gaps < 0.4 mm, 3 free spans < 15 | **28 of 30 at R ≥ 15**; **all wall gaps ≥ 0.4 mm** outside the bundle well over the clamp and the spinneret; **all free spans ≥ 15** at abd ±15 |
| Forearm fan | 22 tube pairs closer than one OD (new map, old S-curves) | **routed**: min R **11.3**, 4 wall gaps < 0.4 (min 0.26 mm). Best after 11 passes, time-boxed (§11.6) |
| Neighbour fingers | "relative ≥ −1°", ring/little contact at +14/+15 | exact allowed-interval table `abd_pair_table.json` (§11.5) |
| Palm heel vs forearm cover at +50/+20 | 59 mm³ | **HEEL_RESULT** |
| Invalid solids | wrist gears ×2, rods ×2, forearm sheath 05e | **VALID_RESULT** |

### 11.2 Poses (`cad/poses.py`, `out/assembly/poses_solved.json`)

A new solver works on the real CAD. Every rigid body (finger yoke/proximal/middle/distal with pads, thumb
base/metacarpal/proximal/distal with pads, palm shell + pads) becomes surface samples plus a 0.4 mm voxel grid in its
own frame. Penetration = points of A inside B's voxels (checked against OCC booleans: same answers). Gap = KD-tree
distance (analytic for the cylinder).
- **Power Ø55:** the axis lies in the x–z plane at γ from +x. It is placed so the palm touches it (0.4 mm), with a
  search over γ and height. Each finger: grid over mcp/pip, keep the pose with the smallest pad gaps (proximal +
  middle + 1.5 × distal) and no penetration of the cylinder or the palm. Thumb: Powell from 4 seeds. It minimises
  the pad gap (target 0.3) with the contact on the far side of the cylinder, and penalises penetration of the
  cylinder, the curled fingers and the palm.
- **Fist:** each finger takes the largest mcp + pip with no point inside the palm. The thumb pad goes onto the
  index/middle middle phalanges (target 0.4 mm), without penetrating.
- **Pinch / opposition:** the finger abd is capped by the pair table (ring/little ≤ 9°). The thumb is re-solved so
  its pad sits 0.3 mm from the finger's tip pad, with no penetration.

POSES_TABLE

### 11.3 Thumb strands: the spinneret

The 6 axial strands still come up the cmc_rot axis through the neck (the twist zone is unchanged in principle). The
sheath ends become a fixed **PTFE spinneret plate** at C −31.5 (hex r 3.4, rotated 18° so that sheaths leaving along
g are ≥ 2.14 mm apart). Each strand turns ~60° in its rounded spinneret hole: a fixed bend, so it cannot couple. Its
sheath then leaves along **g, 5° below horizontal toward the wrist clamp**, through a curved bore in the mount and
then the palm. The neck hole shrank from Ø1.6 to Ø1.2 (strand offsets 0.36), which keeps the cmc_rot common-mode
length change at 0.186 mm with a 4.5 mm twist zone.

Two other things were needed:
- **The palm sheath clamp moved down 6 mm** into the palm flange opening (bridge z 8–13 instead of 14–19, with ears on
  the flange's |y| ≤ 3.5 band and 2 × M2). That gives every strand 6 mm more height in the palm.
- **The clamp top is stepped down to z 10.5 over the 8 thumb cells.**

The 6 thumb tubes take the 6 right-hand rim cells (3, 9, 15, 21, 27, 31) and enter vertically. I tried a tilted
entry into the clamp, but a tube leaving the clamp tilted would cross its neighbours in the 2.15 mm-pitch bundle.

**cmc_rot keeps one idler + a guide tube per strand.** The rot drum's strands leave tangentially in the drum plane
(⊥ a1). Every in-plane direction toward the wrist points upward, and the Ø42 mount flange fills the space under the
thenar. A PTFE path from there needs a 300°+ loop (worse friction than the idler). The real fix is a thumb re-layout
(rot drum under the bearings with a bevel/crossed-strand turn, or a differential CMC): open issue.

### 11.4 Finger MCPs y = −2 (local bosses)

The finger MCPs move from y −4 to −2 (`params.FINGERS`). The palmar abd dowel now ends at y −14.5 and sits in a local
Ø9.2 boss to y −16.1, flush with the TPU palm pad. The continuous palmar lip is gone, and the distal pad (TPU 2 mm)
is cut round the bosses. The dorsal knuckle lip stays (y 16.4, bosses to 17.6). Thumb re-score (`opposition_fixround.json`):

| palm model | score | index | middle | ring | little | tip pinch | key | Ø55 |
|---|---|---|---|---|---|---|---|---|
| designer (y −4, slab −14) | 32.22 | 2.0 / 40° | 0.5 / 20° | 1.2 / 16° | 0.1 / 2° | 0 / 0 | 0.9 / 12° | 0.1 / 1° |
| round 1 knuckle lip (slab −19.6) | 52.3 | 2.0 / 40° | 0.5 / 20° | 1.2 / 23° | 5.6 / 23° | 0 / 0 | 0.9 / 12° | 0 / 0 |
| **fix round (y −2, palm −16)** | **37.25** | 2.0 / 42° | 0.7 / 21° | 0.9 / 15° | 1.0 / 15° | 0 / 0 | 0.9 / 12° | 0.05 / 1° |

The thumb was not re-optimised (the CMC is unchanged); a refine run would recover part of the 5 points.

### 11.5 Neighbour abduction table (`out/assembly/abd_pair_table.json`)

Allowed radial-finger abd (deg, + toward the thumb) for each ulnar-neighbour angle, both fingers at the same flexion.
Voxel test, step 1.5°, agrees with OCC. The firmware uses it as a joint-space constraint.

| pair | flexion | q_ulnar −15 | −7.5 | 0 | +7.5 | +15 |
|---|---|---|---|---|---|---|
| index–middle | 0 | −15…15 | −7.5…15 | 0…15 | 6…15 | 13.5…15 |
| index–middle | 45 | −15…15 | −9…15 | −3…15 | 4.5…15 | 12…15 |
| index–middle | 90 | −15…−12 | −10.5…−1.5 | −4.5…7.5 | 1.5…15 | 9…15 |
| middle–ring | 0 | −15…15 | −9…15 | −1.5…15 | 6…15 | 15 |
| middle–ring | 45 | −15…15 | −10.5…15 | −3…15 | 6…15 | 15 |
| middle–ring | 90 | −15…−9 | −13.5…−1.5 | −6…4.5 | 3…10.5 | 13.5…15 |
| ring–little | 0 | −15…15 | −10.5…15 | −1.5…15 | **7.5…9** | **none** |
| ring–little | 45 | −15…15 | −12…15 | −3…15 | **7.5…9** | **none** |
| ring–little | 90 | −15…−6 | −15…0 | −4.5…4.5 | 7.5…9 | none |

Rules for the firmware:
- **Straight neighbours may only spread** (relative ≥ −1…−1.5°).
- **At 90° flexion the window inverts**: curled tips sit below the abd pivot, so abduction swings them the other way.
- **little_mcp_abd ≤ +9°**, and above +7.5° the ring must follow within 0…1.5°. The ring and little pivots are 8 mm
  apart in z, so their yokes meet. Widening that spacing by ~2 mm would lift the cap; not done (it would re-route
  everything).

### 11.6 Routing and fan

- **Cell map:** the left half is as the wrist/forearm map. On the right half: cmc_rot → 7/2 (stepped), the thumb's 6
  → the rim cells by y order, index + middle_abd → the 8 inner cells (Hungarian).
- **Palm optimiser:** Bézier G2 paths, then a polish (L-BFGS + Powell, N = 140 samples, R target 16.8, curvature
  weight ×3), then a targeted fix of the 8 failing strands with the others frozen.
- **Clearance:** relaxed to the 2.15 mm cell pitch only in the **bundle well** above the clamp (15.6 × 14.8 mm, up to
  z 25) and within 9 mm of the spinneret. All 32 tubes must converge onto the bundle's pitch there by construction;
  it is the same packing as in the wrist bundle.
- **Forearm fan** (`routing.route_fan` → `out/assembly/fan_routes.json`, read by `forearm.make_sheaths`): Bézier paths
  from each barrel adjuster (heading +Z) to its comb cell (z −63). The same packing relaxation applies within 12 mm
  under the comb. 11 passes in all (Powell + L-BFGS, warm start).
  - Best: **min R 11.3** (thumb_cmc_rot_ext), R < 15 on 5 tubes (11.3–14.0), 4 wall gaps 0.26–0.37.
  - Time-boxed. The fan has 55 mm of height for up to 22 mm of lateral shift with 32 crossing assignments.
  - The fix is to re-order the forearm adjuster columns to follow the palm map (open issue).
- Wrist bundle with the lower clamp (wrist_kin): min tube R **18.8** at flex ±50 / dev −30 (was 24.4; ≥ 15 ✓). The
  bundle grazed the wrist-base opening (0.79 mm³), so the opening grew by 1 mm in y (BASE_OPEN y 12 → 13): clear.

### 11.7 Module changes (all logged)

| file | change |
|---|---|
| params.py | FINGERS mcp y −4 → −2; PALM_KNUCKLE_BULGE → local bosses (−16.1 / 17.6, Ø9.2); PALM_PAD_FACE_Y −16; SHEATH_CLAMP_Z (14,19) → (8,13); CLAMP_THUMB_STEP_Z 15.5 → 10.5; ABD_PAIR_LIMIT_DEG + table note |
| thumb.py | "spinneret" layout: FAN_DOWN_DEG 5, FAN_TARGET_XY (4,−1), EXIT_L 4, `fan_dir()`, `_hex_rot()` (18°); sheath exits = spinneret hole + 4·g; SHEATH_END_Z −32 → −31.5; BUNDLE_R 2.3 → 3.4; NECK_R 0.45 → 0.36, neck hole Ø1.6 → Ø1.2; fan cavity r BUNDLE_R + 0.7; `FAN_DOWN_DEG = None` restores the original |
| wrist.py | CLAMP_Z from params (z 8–13); bridge ears + 2 × M2 into the flange band; BASE_OPEN y 12 → 13; pinion fused to the servo shaft, big gear to its axle (single solids); push rod rebuilt as one solid of revolution with flat pole caps (watertight) |
| wrist_kin.py | Z_PALM_CLAMP from params (8) |
| forearm.py | `comb_cells()` reads routes.json's cell map (shared palm/comb map); `fan_paths()` uses fan_routes.json; sheaths as one sweep each (was a chain of unioned cylinders) |
| routing.py | per-strand end direction, palm_cell_map, thumb PTFE strands, real base-sweep obstacle, flange rule, well/spinneret relaxation, route_fan |
| palm.py | local bosses, distal pad round the bosses, conservative stepped forearm envelope carve with a 12 % volume guard, bored thumb mount (`thumb_mount_bored`), bore retries, checkpoint of the bored shell |
| assembly.py | poses from poses.py; clamp step over the thumb/rot cells; bored mount replaces T_thumb_mount |
| poses.py | new: pose solver + neighbour table |
