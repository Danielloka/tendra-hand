# 08 - Integration of the bearing round (2026-11-14)

Integrator report: 2 bearings per joint, one knotted track per joint (finger report 06, thumb report 07) built into the full assembly.

## What changed in the code
- `palm.py`: per finger two MR63ZZ seats for the abduction axis (Ø5.95 x 2.5, pressed in from outside, 0.85 mm shoulder with Ø4.8 bore, 3 mm stub runs on into the yoke gap); bosses stay Ø9.2.
- `routing.py`: strand exits and the clamp cell map re-routed for the moved exits (routes.json, tie_walls.json).
- `params.py` / `finger.py` / `thumb.py`: `TIE_HOLE_D = 1.7`, `tie_liner_*` parts (16G stainless tube OD 1.65 / ID 1.19 x 1.2 mm, glued into every tie hole): 3 per finger (mcp, pip, abd) = 12, thumb 4 (rot, cmc_flex, mcp_flex, ip) = 16 in all.
- `poses.py`: bearing_* and tie_liner_* stay inside the body that holds their seat (press fits are never tested against their own seat).
- `assembly_checks.py`: abd bearings vs P_shell listed as intended press fit; bearings and liners are included in every check.
- `assembly.py`: the palm's bored thumb mount contained an inverted (negative-volume) sliver solid, which made OCC booleans return the other part whole (false 4362 mm3 "hits" with the wrist flange in every pose). The assembly now drops non-positive solids of `thumb_mount_bored` (palm cache unchanged, palm not rebuilt).

## Pipeline and checks
poses.py (4 grasp poses solved: power55 cost 0, fist 0.16, tip pinch 0.36 mm pad gap, little pinch 0.30 mm) -> assembly.py (266 parts, 783.1 g, STEP + .gz, GLB 10.1 MB, STLs) -> assembly_checks.py (fresh, not resumed).
- All 7 poses (open, fist, tip pinch, thumb-little, power 55, flex 50 pinch, dev -30): **0 interferences** (the old thumb_little 0.92 mm3 hit is gone).
- Thumb vs palm over the CMC range: clear except rot -31 / flex +63 (flex +63 is outside the used range; same as before; now also lists the MCP bearing against the palm shell 109 mm3 there).
- Wrist extremes: clear except flex +50 / dev +20: P_rot_idlers x forearm cover 2.3 mm3, P_shell x forearm cover 1.8 mm3 (known, as before).
- Validity: 264 / 266 parts ok; not ok (valid, not watertight): T_thumb_mount, F_sheath_03e (unchanged).
- Forearm fan: min bend radius 11.3 mm, 5 strands < 15 mm, 4 wall gaps < 0.4 mm (unchanged).
- Rolling wrist grid and bundle: clear, min bend 18.8 mm.

## Bill of bearings (per hand)
| Type | Where | Count | Mass |
|---|---|---|---|
| MR84ZZ (4 x 8 x 3) | finger mcp/pip/dip, 2 per joint x 12 joints = 24; thumb cmc_flex, mcp_flex, ip, 2 each = 6 | 30 | 21.0 g |
| MR63ZZ (3 x 6 x 2.5) | finger mcp_abd, 2 per finger, in the palm's knuckle bosses | 8 | 2.4 g |
| 6704ZZ (20 x 27 x 4) | thumb mount, cmc_rot pair (existing) | 2 | not counted in mass.json bearing totals |
| 16G stainless tie liner | one per tie hole | 16 | ~0 g |

Total 38 joint bearings + 2 thumb-mount bearings. GLB: 40 nodes with "bearing" in the name, one mesh per part (e.g. `index_bearing_pip_l`, `index_bearing_abd_palmar`, `T_bearing_ip_l`, `T_bearing_bottom`); 16 `tie_liner` nodes.

## Open issues
- The inverted sliver in `palm.py`'s thumb-mount bore (bore grazes the mount skin) should be fixed at the source on the next palm rebuild; the assembly filter hides it.
- thumb at rot -31 / flex +63 and wrist +50/+20 contacts as above; forearm fan bends under 15 mm remain.
