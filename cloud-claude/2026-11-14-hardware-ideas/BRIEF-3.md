# Round 3 brief (2026-11-16): build the full V2.1 hand in 3D

Owner (verbatim): "Make me see the whole hand latest update. Build full 3d hand."

Same rules as `BRIEF.md` (write only inside this study folder, numbers not words, plain English).
Goal: one complete right hand + wrist + forearm with every round-1/round-2 fix and the sensors, as CAD,
checked, and exported for the 3D viewer. Base: the V2 concept (`../2026-11-09-v2-hand-cad/`: `cad/params.py`
for finger positions/lengths, thumb kinematics, palm, forearm) + this study's reports 12, 13, 15, 16.

## Work packages

| Package | Agent | Model | Output |
|---|---|---|---|
| A. Four fingers + palm knuckle module | finger builder | Opus | `cad/18_fingers.py` → `out/18/fingers/` |
| B. Thumb upgraded to V2.1 | thumb builder | Opus | `cad/18_thumb.py` → `out/18/thumb/` |
| C. Palm, wrist (W13), forearm, full assembly, checks, viewer export | integrator (wrist agent) | Opus | `cad/18_assembly.py` → `out/18/` |

A: make `cad/16_finger.py` parametric per finger (index, middle, ring, little: V2 lengths and positions from
`params.py`, same sensor layout, taper). Replace the bench base with a **palm knuckle module** (19.5–20 mm
pitch, fits report 13's palm, per-strand liners for the palm crossings). Export each finger's parts in the
**hand frame** (origin = wrist centre, +Z fingers, −Y palm, +X thumb) with a body name per part, plus a
function `pose(q) -> {part: 4×4}` for any finger joint angles, and the tendon + sensor-wire polylines.

B: upgrade the V2 thumb (`thumb.py`, `thumb_kinematics.py`; keep its kinematics) to V2.1 rules: bare strands
over steel guides/rollers (no PTFE), drum rings, fuse cups, an angle sensor on each of its 4 joints (R3-slim
style, non-magnetic bearing beside the magnet), tip sensor + silicone skin, wires through hollow pins; its
strands must reach report 13's palm plate (the 68–96° turn needs a roller). Same export contract as A.

C: palm (holds the knuckle module, thumb base, palm plate, sensor hub board, pad pockets), W13 wrist, forearm
with 16 servos + spool cartridges (report 02) using report 13's slot map. Assemble, run interference at the
poses below, export.

## Export contract for the viewer (C writes it; A and B provide pose functions)

- `out/18/hand_v21.glb`: every part at q = 0, node name = part name, mm, Z up, colours.
- `out/18/poses.json`: `{ "parts": {name: body}, "poses": { pose_name: { "q": {...joint: deg}, "T": {name: [16 floats row-major]},
  "lines": {strand_or_wire_name: [[x,y,z],...]} } } }` for at least: `open`, `relaxed`, `fist`, `tip_pinch`
  (thumb–index), `thumb_to_little`, `power_grasp`, `spread`, `wrist_flex_+40`, `wrist_dev_-20`. Lines in the hand frame.
- `out/18/checks.json`: validity, interference per pose, mass, part count.
