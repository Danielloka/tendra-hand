# Tendra Hand "V2 concept" in code CAD (2026-11-09 … 11-12)

A full right hand with a thumb, a 2-DOF wrist and a forearm, modelled in **CadQuery** (Python code CAD) by an agent team. The brief is in [`BRIEF.md`](BRIEF.md): every joint uses one 7 mm drum ("same distance on each degree"), the DIP is coupled to the PIP by a bar, the tendons run in PTFE tubes through a hollow wrist, and the thumb comes first.

**This is a study, not a decision.** Nothing in `hardware/`, `sim/` or `CLAUDE.md` was changed.

## Summary and recommendation

**The geometry works. The drivetrain does not yet.**

What works, checked in the files:
- Every servo joint has a true 7 mm drum, so 1° of any joint pulls 0.1222 mm of tendon. The servo ratio is 1.4 everywhere, and the flex + extend loop length is constant (to about 1e-14 mm).
- A joint's tendon changes length by at most 0.19 mm when other joints move. The DIP bar holds 0.75 × PIP within 0.5°.
- 223 of 225 parts are valid solids. The 2 that fail are valid shapes whose meshes are not closed.
- Seven poses solve on the real CAD with no part interference: open hand, fist, tip pinch, thumb to little finger, power grasp of a 55 mm cylinder, pinch with the wrist bent 50°, and wrist deviation −30°.
- The new thumb (roll-pitch CMC, optimised for opposition) reaches every fingertip. It is much better than V1's.

What the review ([`reports/05-review.md`](reports/05-review.md)) found:
1. **Critical: the plain PTFE tubes are too soft and too weak.** In a Bowden drive the tube carries the full tendon pull. At SCS0009 stall (45 N) the tube is stressed above PTFE's yield and shortens by about 8 mm, which is about 70° of joint angle. Creep turns this into lasting slack.
2. **Friction:** each strand turns 160–460° in total, so only 45–75 % of the servo force arrives. Fingertip force is about 2–2.5 N, not 3.4 N.
3. **The wrist gear and crank are 3–5× too small** for the claimed torque.
4. The forearm tube fan does not yet hold the tube slack that wrist motion needs.
5. The strands bend sharply at the knuckle eyelet (radius 0.43 mm) and will fray there.
6. Tube ends have no positive stop, a strand can only be replaced by taking the finger apart, and the HLS upgrade path is blocked.

**Recommendation:** keep the geometry: the finger, the thumb kinematics and the wrist layout. Before printing, change the tendon housing:
- Use a **compressionless housing** instead of plain PTFE: a PEEK 1 × 2 tube, or a PTFE-lined steel coil.
- Give every tube end a **ferrule or step stop**.
- Use a **Ø3 roller** instead of the eyelet.

Then bench-test one tube and one finger. The order is in the review's "pre-print order".

## Bearing round (2026-11-13…14)

**Owner's request:** 2 bearings per joint (4 × 8 × 3 mm), and one knotted track per joint, like the owner's own hand: one tendon, tied at a hole at the top of the track ring, so the motor can pull both ways. Every bearing and part must be easy to remove and add back.

**What was built**
- Finger MCP, PIP and DIP, and thumb cmc_flex, MCP and IP: pairs of MR84ZZ bearings on 4 mm steel axles (stubs or through-pins, because the strands cross the axis at the MCP and cmc joints).
- Finger abduction: pairs of MR63ZZ (3 × 6 × 2.5), because MR84ZZ does not fit next to the MCP sheath bores.
- One track per joint at the 7 mm drum. Each tie hole is lined with a 16G stainless tube. The knot sits in a well that opens on the back of the finger (in a pocket on the side of the thumb hub), so a strand can be retied without taking the hand apart.
- The parts named `bearing_*` can be shown or hidden: `build(..., bearings=False)` in `cad/finger.py` and `cad/thumb.py`, and the thumb viewer (`out/thumb/thumb_viewer.html`). The full-hand web viewer (a private claude.ai page, made from `out/assembly/tendra_v2_open.glb`) has **Bearings** (show/hide) and **X-ray** (see-through plastic) buttons.

**Results**
- Bearings per hand: 30 MR84ZZ and 8 MR63ZZ (the 2 thumb-mount 6704ZZ bearings were already there).
- Full assembly: 266 parts, 783 g.
- All 7 test poses are clear of interference: open, fist, tip pinch, thumb to little finger, power grasp of 55 mm, pinch with the wrist bent 50°, and wrist deviation −30°.
- Every track keeps the 0.1222 mm per degree servo ratio, and the strands stay within their wrap margins.

**Open issues** (details in reports 06–08)
- Some walls are below the 1.6 mm minimum: the thumb MCP tie wall (0.7 mm), the thumb spacer boss (0.6 mm), the finger knot well wall (0.6 mm) and the pin A web to the PIP seat (0.28 mm).
- Knot load: the knot takes about 87–90 % of the strand pull. Use stronger line (80 lb braid suggested), a figure-8 knot and a drop of CA. The thumb cmc_rot knot has the smallest wrap margin (19.5°).
- Nothing is printed. Print a test coupon with 7.90 / 7.95 / 8.00 mm seats and 3.9 / 4.0 mm pin holes before the full parts.
- The thumb mount bore in the palm has an inverted sliver. The assembly drops it, but the palm should be fixed at the source on the next rebuild.

## Owner decisions

| Decision | Options |
|---|---|
| Tendon housing | PEEK tube (stiff, same 2 mm OD) / PTFE-lined steel coil (~2.5 mm OD, needs bigger bores) / keep PTFE plus joint-angle sensors |
| Load class | SCS0009 (about 2 N at the fingertip) / HLS3606M (needs the stronger parts in review #7) / another actuator class for the 45–80 N goal |
| Wrist range | ±50° flex with a 30 mm gap (built) / −60…+70° with a 45–50 mm gap |
| Big file in git history | `b2954a6` holds a 62 MB STEP. It can be removed with a history rewrite (force push), only if you agree. |

`hardware/cad/Hand assebly.step` at HEAD holds only 2 bodies (overwritten in commit 7215915); the full index + thumb export is in 8a004b5.

## Files

| Path | What |
|---|---|
| `BRIEF.md` | The owner's request and the team's rules |
| `cad/params.py` | Shared skeleton: frame, drum, finger positions, thumb axes, wrist, servo IDs |
| `cad/finger.py`, `reports/01-finger.md` | Finger: drums, eyelets, DIP bar, abduction yoke |
| `cad/thumb.py`, `thumb_kinematics.py`, `reports/02-thumb.md` | Thumb: optimiser, base on bearings, strands up the rotation axis |
| `cad/wrist.py`, `wrist_kin.py`, `forearm.py`, `reports/03-wrist-forearm.md` | Hollow cardan wrist, 2 × STS3215, forearm with 16 servos and spools |
| `cad/palm.py`, `routing.py`, `poses.py`, `assembly.py`, `assembly_checks.py`, `reports/04-integration.md` | Palm, routing of all 32 strands, poses, full assembly and checks |
| `reports/05-review.md` | Adversarial review: findings, verified and unverified claims |
| `reports/06-bearings-track.md` | Bearing round, fingers: bearings, one track per joint, fits |
| `reports/07-thumb-bearings.md` | Bearing round, thumb: bearings, tracks, knot margins |
| `reports/08-integration-bearings.md` | Bearing round built into the full assembly: checks, bill of bearings |
| `cad/tools/` | Renderer (three.js in headless Chromium), checks, transforms, export |
| `out/assembly/` | `tendra_v2_open.glb`, `.step.gz` (gunzip for the STEP), pose renders, `checks.json`, `routes.json`, `mass.json` |

Rebuild: `cd cad && bash run_fix_pipeline.sh` (the palm takes about 35 min). It runs with `uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy --with pillow --with rtree`.

Licence: text CC BY 4.0, code Apache-2.0, CAD output CERN-OHL-S-2.0 (like `hardware/`).
