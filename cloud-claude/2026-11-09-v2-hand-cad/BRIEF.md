# Brief for the agent team: Tendra Hand "V2 concept" in code CAD

Date 2026-11-09. Owner's request (translated, verbatim intent):

> Model a good hand with a wrist, and a thumb. The tendons go through tubes through the wrist. Use the
> research and find ways to make the hand as good as possible. Use the existing hand as inspiration,
> with its tendon routing and the "wheel" (drum) that the tendon rotates. Make the tendon the same
> distance on each degree. The tip and the next tip are fixed to each other (DIP coupled to PIP).
> Use 1X's tech, Optimus and standard practice. Be creative and think outside the box, but make sure it
> works as well as possible. Especially the thumb.

This is a **study** in `cloud-claude/2026-11-09-v2-hand-cad/`. Do not edit anything outside this folder
(not `hardware/`, `sim/`, `software/`, `CLAUDE.md`). Read whatever you like.

## Required reading (skim what is relevant to your part)

- `CLAUDE.md` (project context, V1 design, sign conventions)
- `research/references/hands/README.md` + the detailed reports there (`1x-neo.md`,
  `humanoid-company-hands.md` (Optimus V3), `tendon-mechanics.md`, `dexterous-and-open-source-hands.md`)
- `cloud-claude/2026-11-04-v1-wrist/` (wrist study: sheaths, cardan, bundle, servo choice)
- `hardware/cad/tendon_router.py`, `hardware/cad/dip_linkage.py` (V1 routing + DIP bar maths)
- `hardware/robot_description/v1_export/hand_v1.json`, `tendon_routes.json` (V1 geometry)
- `research/experiments/2026-09-29-thumb-routing/` (V1 thumb routing problems)

## Hard requirements (from the owner)

1. **Full right hand:** 4 fingers + thumb + palm + **2-DOF wrist** (flex + deviation) + forearm with the
   servos and spools.
2. **Tendons:** every servo joint = one antagonistic loop (flex strand + extend strand) wrapped on a
   **drum ("wheel") on the child segment, centred on the joint axis**, tied/anchored in the drum, driven
   by a **5 mm radius spool** on its servo in the forearm.
3. **"Same distance on each degree":** we read this as
   - a. every drum is a true circle about its joint axis, so strand travel per degree is constant over
     the whole range (no changing lever arm),
   - b. flex + extend strand lengths sum to a constant at every angle (no slack, no over-tension),
   - c. strands of more distal joints cross each joint either exactly on its axis or inside a sheath
     that is clamped on both sides, so they do not change length (no coupling),
   - d. **one common drum radius `R_DRUM` for every servo joint** where at all possible, so 1° of any joint
     = the same mm of strand and the same servo ratio (`R_DRUM / R_SPOOL`). Deviate only with a
     written reason.
4. **DIP rigidly coupled to PIP** (≈ 0.75 × PIP) by a rigid bar per finger, like V1 (`dip_linkage.py`).
   No DIP servo. The thumb IP stays independent.
5. **Tendons pass the wrist in PTFE tubes (Bowden sheaths)**, both sheath ends clamped, bundled through a
   hollow wrist centre, so wrist motion does not move the fingers.
6. **The thumb is the priority.** It must oppose every fingertip (Kapandji-style reach), pinch the index
   tip-to-tip, and make a strong power grasp. Its strands must route cleanly (V1's problem).

## Shared conventions (everyone uses these)

- Units **mm** in CAD, degrees in docs. Right hand.
- **Frame:** origin = **wrist centre** (where the wrist's two axes cross). **+Z** = toward the fingers,
  **−Y** = palm side (flexion bends toward −Y), **+Y** = back of hand, **+X** = thumb side.
- **Sign:** positive joint angle = closing the hand.
- Shared numbers and the skeleton live in `cad/params.py`. **Only the integrator edits it.** If you need
  a change, write it in your report under "Requested params changes" and work with a local override
  until then.
- Each module is a Python file in `cad/` using **CadQuery 2.x** (`uv run --no-project --with cadquery
  --with trimesh --with numpy --with matplotlib python ...`). Each exposes a `build(...)` function that
  returns named solids **already placed in the hand frame at q = 0**, plus a dict of joint axes
  (point, direction) and interface points (strand entry points, sheath clamps, mount holes).
- Outputs go to `out/` (STEP + STL per part, PNG renders). Your report goes to `reports/NN-name.md`.
- Printing: FDM, PETG (structure), TPU 95A pads. Min wall 1.6 mm, sliding clearance 0.25 mm, press fit
  −0.1 mm. Joints run on **steel dowel pins** (2 mm in fingers, 3 mm in thumb base and wrist) or small
  bearings (MR52/MR63/623). Strands only touch PTFE, polished steel or their own drum.
- Tendon: braided UHMWPE 0.28–0.36 mm. Sheath: PTFE 1 × 2 mm (ID × OD); bore for it 2.2 mm. Min sheath
  bend radius 15 mm. Bare strand bores 1.2 mm.
- Servos: Feetech SCS0009 envelope (23.3 × 12.1 × 25.25 mm, tabs 32.5 mm, 28.5 mm hole spacing, shaft
  5.7 mm off-centre) for the 16 hand servos, but leave room for HLS3606M (same footprint, +2.25 mm tall)
  on the high-load joints. Wrist: 2 × STS3215 class (45.2 × 24.7 × 35 mm) per the wrist study.
- Same 16 servo joints and IDs as V1 unless your design argues for a change (state it):
  fingers × (`mcp_abd`, `mcp_flex`, `pip`) + thumb (`cmc_rot`, `cmc_flex`, `mcp_flex`, `ip`).

## Quality bar

Each part must be a valid closed solid that could be sliced and printed. Joints must sweep their range
without the parts colliding (check at min, mid, max). Show it with renders and numbers, not words.
Think like a hand engineer: what fails first (strand, drum, pin, print layer), and why your design avoids it.
