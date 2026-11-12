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

## Owner decisions

| Decision | Options |
|---|---|
| Tendon housing | PEEK tube (stiff, same 2 mm OD) / PTFE-lined steel coil (~2.5 mm OD, needs bigger bores) / keep PTFE plus joint-angle sensors |
| Load class | SCS0009 (about 2 N at the fingertip) / HLS3606M (needs the stronger parts in review #7) / another actuator class for the 45–80 N goal |
| Wrist range | ±50° flex with a 30 mm gap (built) / −60…+70° with a 45–50 mm gap |
| Big file in git history | `b2954a6` holds a 62 MB STEP. It can be removed with a history rewrite (force push), only if you agree. |

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
| `cad/tools/` | Renderer (three.js in headless Chromium), checks, transforms, export |
| `out/assembly/` | `tendra_v2_open.glb`, `.step.gz` (gunzip for the STEP), pose renders, `checks.json`, `routes.json`, `mass.json` |

Rebuild: `cd cad && bash run_fix_pipeline.sh` (the palm takes about 35 min). It runs with `uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy --with pillow --with rtree`.

Licence: text CC BY 4.0, code Apache-2.0, CAD output CERN-OHL-S-2.0 (like `hardware/`).
