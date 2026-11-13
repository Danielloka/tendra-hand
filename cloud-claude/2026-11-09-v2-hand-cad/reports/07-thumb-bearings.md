# 07 · Thumb: 2 bearings per joint and one track per joint (owner request 2026-11-13)

Thumb designer, 2026-11-13. Code: `cad/thumb.py` (no change to `params.py`, `finger.py` or `palm.py`). Outputs in
`out/thumb/`: `bearing_section.png`, `bearing_section_flexed.png` (cut through the MCP and IP),
`tracks.png` (the three hinge tracks with the tie hole and knot pocket), `thumb_bearings_on.png` /
`thumb_bearings_off.png`, **`thumb_viewer.html`** (open it in a browser: show/hide bearings, axles and pads,
a section cut, and sliders for the four joints), `thumb_rest.step` / `.glb` (bearings are separate, coloured
parts), `routing.json` (new keys `tracks`, `loops_full_grid`), `main.log`.

Run: `uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy --with pillow
python cad/thumb.py`. `build(q, bearings=False)` leaves every `bearing_*` part out.

## 1. The owner's thumb (`hardware/cad/Hand assebly.step`)

The STEP has two thumb segments (2060 and 5314 mm³). I measured them with OCP and sectioned them:

| feature | measured |
|---|---|
| child axle | printed **4 mm bosses** (Ø3.96), **3 mm** long, one on each side of the child |
| parent seat | **Ø8.1**, **2.9 mm** deep, in **3.0 mm** side lugs (lug Ø15 round the axis), open to the **outside**, a ~0.1 mm lip inside |
| track | one groove, floor **r 6.0**, 1–2 mm wide, flanges r 7.5 |
| tie | a ~1 × 2 mm hole **through the track ring at the top**, opposite the side the strand comes from. The strand runs in the groove, through the hole (knotted), and back out, so one strand pulls both ways |
| pass-through strands | cross the joint in a pinch slot on the axis (two r 0.4 notches) |
| thumb width | 15 mm over the bearings (narrow, but no strand crossing inside) |

## 2. What changed

**Bearings (cmc_flex, mcp_flex, ip).** 2 × MR84ZZ per joint, one in each cheek of the parent. The cmc_rot base
keeps its 2 × 6704.

- **Seat:** Ø7.95 (light press, `P.BEARING_SEAT_D`), 3.0 deep, **open to the outside**, like the owner's. A
  **0.8 mm lip** on the inner side (bore Ø6.75, `BEARING_SHOULDER` 0.6) stops the outer race. Every bearing
  cheek is 3.8 mm thick.
- **Axle: 4 mm hardened steel, not printed bosses.** At cmc_flex and mcp_flex, the mcp and ip strands cross
  the axis exactly on it, through a central slot, so a through-axle is impossible. There are **two stubs**
  instead (`pin_cmc_a/b` 4 × 9, `pin_mcp_a/b` 4 × 8), pressed into the child hub and running in the inner
  races. The **IP has one through-axle** (`pin_ip_axle`, 4 × 20 dowel), since nothing crosses the IP.
- **Why steel and not printed bosses:**
  - A 4 mm PETG boss is a layer-weak cantilever carrying the whole joint load.
  - A steel stub sees ~16 MPa at 40 N. The PETG bore pressure falls from ~7 MPa on the old 2 mm stubs to
    ~2.5 MPa.
  - The child does not need a springing fork to go in.
- **Assembly order:**
  1. Put the hub between the cheeks.
  2. Push the stub in from outside through the lip.
  3. Press the bearing over the stub into the seat.
- **Spacer:** a **Ø5.2 spacer boss** on each hub face touches only the inner race (modelled with a 0.05 mm gap).
- **Part names:** `bearing_cmc_flex_l/_r`, `bearing_mcp_flex_l/_r`, `bearing_ip_l/_r`, where `_l` is the −x side
  of the joint axis. They are separate parts for show/hide.

**One track per joint (the owner's way).** Every joint has one groove at R_DRUM 7 mm (0.1222 mm/°, servo ratio
1.4 unchanged; checked slope ±0.1222 on all 8 strand halves). One strand per joint comes in on the flex side,
wraps over the drum, is knotted through a Ø1.0 **tie hole at the top of the track ring**, and leaves on the
extend side.

- **Tie hole:** parallel to the axis at r 7, from the groove wall into a **Ø2.4 × 1.7 knot pocket that opens on
  the hub's side face**. You tie the strand before the hub goes in, and the cheek then covers the knot.
- **cmc_rot:** the same idea in one groove (z −16). The ext half moved down 1.8 mm. The knot slot opens at the
  drum rim, under the base plate.
- **Knot position:** the tie angle is the middle of the free arc between the two tangent points' extreme
  positions, so both wraps keep the same margin. The knot sits on the dorsal-distal side ("top"), 55–68°
  from dorsal.
- **What was removed:** the double grooves and the 8 crimp-bead anchor pockets.
- **What was kept:** the nip pins. They keep crossing strands on the axis; they were not anchors.

Other changes:

- **Track positions:** cmc track at x −4.8 (wall to the central slot **1.6 mm, was 0.8**: fixes review #14),
  mcp track x +3.9 (wall 1.6, was 1.2), ip x +1.0.
- **Crossing order at cmc_flex:** re-sorted (ip −1.8/−0.6, mcp +0.6/+1.8) so both mcp halves cross on the side of
  their track.
- **IP cheeks** now start at |x| 6.25 (was 7.75), inside the 15 mm pad. Two changes keep them clear:
  - The distal stays 12 mm wide up to z 8.
  - The palmar-lower quadrant of the IP cheeks is cut away outside the seat ring, a knuckle shape.
- **Without these changes,** the pad hit the cheeks at IP > 70° (found by the sweep, fixed, re-checked).

## 3. Numbers and checks

| check | result |
|---|---|
| validity | **26/26 parts** valid, watertight, single solids (21 before) |
| collision sweep (28 poses, min/mid/max) | **0 of 28** with interference. Bearings and stubs against parent and child are checked; only press fits are ignored |
| hard stops | first contact cmc_rot +2/+2°, cmc_flex +4/+10°, mcp +2/+2°, ip +2/+2° past the limits (unchanged) |
| knot inside the wrap, worst over each range (other joints at rest/mid/max) | cmc_rot **19.5°** (2.4 mm), cmc_flex 55.6° (6.8 mm), mcp 58.0° (7.1 mm), ip 51.3° (6.3 mm). Never reaches a tangent point |
| loop flex + ext over the own range | cmc_rot exactly constant (1e-14 mm). Hinges are constant when only the own joint moves. Their 0.16–0.19 mm (own range × others rest/mid) and 0.36–0.73 mm (full 4-joint grid) are the existing common-mode terms from the rot twist and nip wraps, **identical to the committed design** (re-run of HEAD: 0.16/0.172/0.187) |
| real coupling (half flex−ext) | ≤ **0.004 mm = 0.03°**; mcp from cmc_flex improved **0.09° → 0.02°** (both mcp halves now cross on one side) |
| opposition | kinematics unchanged (same axes, lengths, ranges, so the Kapandji table in report 02 holds). CAD check of new vs old thumb parts against the CAD finger (committed `finger.py`, the finger designer is editing it) and the cached palm at index/middle/ring/little pads, tip pinch, key pinch, power 55, flat: **identical contacts**; only the pads/tips touch (the intended contacts; the overlap is the existing kinematic-vs-CAD finger difference). No knuckle, bearing or palm contact (`opposition_cad_check.json`) |
| envelope / base sweep | x −48…139, y −129…26, z −12…159 / base x 18…55, y −33…5, z 25…65 (as before) |

**Fits and walls** (min 1.6 mm unless noted):

| item | value |
|---|---|
| seat | Ø7.95, 3.0 deep (tune after a test print) |
| lip | 0.8 thick, 0.6 radial |
| seat ring wall | CMC 3.0, MCP 2.5, IP 2.5 |
| slot ↔ track | 1.6 |
| tie hole wall (groove to knot pocket) | cmc 2.2, ip 2.8, rot 1.2, **mcp 0.7 (thin)** |
| **spacer boss wall** | **0.6** round the 4 mm bore (use a 4 × 7 × 0.5 steel shim instead if it cracks) |
| stub press length | cmc 4.95, mcp 3.95, IP axle 12 |
| web between nip-pin hole and stub bore | cmc 0.95, **mcp 0.55** (unloaded) |

**Widths:** MCP knuckle **21.7** (was 19.5), IP **20.1** (19.1), CMC base 26.5 (unchanged). A human thumb is
about 22 mm.

**Loads:** about 25–30 N per MR84 at SCS0009 stall, ~65 N with HLS. MR84ZZ catalogue C0 is ≈ 0.15 kN, Cr ≈
0.39 kN (from memory; verify). Mass: about +8 g (6 bearings ≈ 3.7 g, steel axles 5.3 g, minus the old pins).

## 4. Interface changes (routing.py, poses.py, assembly.py, palm.py)

1. **`thumb_cmc_rot_ext` strand exit moved 1.8 mm along −a1**, now (48.78, −18.34, 22.58); direction unchanged.
   The palm's cmc_rot idler for that strand must be re-routed (re-run routing/palm). Everything else in
   `interface()` is unchanged: joints, axes, ranges, mount, the 6 axial exits.
   - The stored `out/thumb/interface.json` and `routing.json` were stale (from before the integrator's
     spinneret fix). Their regenerated values match the committed code.
2. **Parts:**
   - removed `pin_ip_a`, `pin_ip_b`;
   - added `pin_ip_axle` and `bearing_{cmc_flex,mcp_flex,ip}_{l,r}`;
   - `pin_cmc_*` and `pin_mcp_*` are now 4 mm and belong to the child (`metacarpal`, `proximal`; they were
     `base`, `metacarpal`).
   - Name filters in poses.py ("pin", "bearing") still work.
3. `build(q, bearings=True)`: new optional argument (backward compatible).
4. The metacarpal's MCP fork is 2.2 mm wider and the IP fork 1 mm wider. The palm carve and the assembly
   checks were not re-run (palm ≈ 35 min). The CAD check above found no new thumb–palm contact.

## 5. Open issues

- **cmc_rot knot margin 19.5°** is the tightest: the 141° range leaves only 39° of the 180° free arc. If the key
  pinch is dropped (rot range ~85°), the margin becomes ~47°.
- **The knot now carries almost the full strand pull.** At μ ≈ 0.15 on PETG, the ≥ 51° wrap leaves ~87 %.
  UHMWPE knots hold 50–80 % of the line, so with 50 lb braid the knot holds ~110–180 N: fine for SCS (45 N),
  marginal for HLS (118 N). Use 80 lb braid and a figure-8 on a bight plus a drop of CA, or the owner's
  proven V0 knot.
- **Thin features:** the MCP tie wall (0.7 mm) and the spacer boss (0.6 mm) are below the 1.6 mm minimum.
- **Stub removal:** the stubs are press fits. Removing one means pulling the bearing first. Proposal (not
  modelled): tap the outer end M2 for a puller screw, which also makes strand replacement a 5-minute job
  (review #8).
- **Kinematic capsules** `THUMB_RADII` (9.5/9.0) are now ~1.1 mm narrower than the knuckles. Update them before
  the next optimiser run.
- **Not changed:** the 1 mm nip pins (review #5) and the Bowden compliance (review #1).

## 6. Ideas (entrepreneur's view)

1. **Every joint knows its angle, almost for free.** The 4 mm stub/axle turns with the child phalanx, and its
   outer end sits flush in the cheek. Use a non-magnetic stub (316 stainless or titanium), glue a Ø3 × 1
   diametral magnet in its end, and clip a 3 × 3 mm magnetic angle sensor (MA732 / MT6701 class) on a tiny flex
   to the cheek's outer face.
   - **Cost:** about $2 per joint, on the same I²C/SPI to the ESP32.
   - **Why it matters:** this is the cleanest answer to review #1. The sheath compliance makes a joint lag up to
     ~70°; with true joint angles the firmware closes the loop and the hand becomes a precise hand.
   - **Selling point:** "every joint senses itself".
2. **One bearing for the whole hand, and swappable phalanges.** MR84ZZ (4 × 8 × 3) in every hinge of the
   thumb and the fingers means one bearing SKU (50 for a few dollars) and one 4 mm dowel size.
   - With the seats open to the outside, a phalanx is a **cartridge**: pull two bearings, the phalanx comes out
     with its own knotted strand, re-tie it at the side-face pocket, press back in.
   - That is a field-service story (spare fingertips by mail), and it is how the owner already builds V0.

## Summary for the lead

- **Files changed:** `cad/thumb.py` only. Outputs regenerated in `out/thumb/`:
  - new: `bearing_section*.png`, `tracks.png`, `thumb_bearings_on/off.png`, `thumb_viewer.html`,
    `opposition_cad_check.json`;
  - updated: `routing.json`, `interface.json`, STEP/GLB/STL (2 stale STLs removed), poses, `main.log`;
  - new report: `reports/07-thumb-bearings.md`.
- **Interface changes:**
  - (a) the `thumb_cmc_rot_ext` exit moves 1.8 mm along −a1, so its palm idler needs a re-route;
  - (b) part names: −`pin_ip_a/b`, +`pin_ip_axle`, +6 `bearing_<joint>_l/_r`, and the `pin_cmc_*` /
    `pin_mcp_*` bodies changed;
  - (c) `build(q, bearings=True)` added.
  Joints, axes, ranges, mount and axial exits are unchanged.
- **Checks:**
  - 26/26 parts valid;
  - 0/28 poses interfering;
  - hard stops 2–10° past the limits;
  - knots ≥ 19.5° (rot) / ≥ 51° (hinges) inside the wrap;
  - cmc_rot loop exactly constant; hinge loops unchanged from the committed design;
  - coupling ≤ 0.03°;
  - opposition kinematics unchanged, and the CAD opposition contacts are identical to before.
- **Not re-run:** palm and assembly checks.
