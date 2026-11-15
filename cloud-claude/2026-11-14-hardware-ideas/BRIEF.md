# Brief for the agent team: better hardware for the Tendra Hand (2026-11-14)

Owner's request (verbatim): "I want to improve the hand. Make the hardware better. Come up with better
ideas and be creative. Mostly the 3D files and 3D printed parts. Make it better. Find better solutions
that will work better. Use agent team with a suitable model. Try and find lots of solutions and be
creative and think like Elon Musk. Try wide and narrow. Think like an engineer."

This is a **study** in `cloud-claude/2026-11-14-hardware-ideas/`. Do **not** edit anything outside this
folder (not `hardware/`, `sim/`, `software/`, `CLAUDE.md`, not the earlier studies). Read anything.

## Where we are (read before you start)

- `CLAUDE.md`: project context, V1 design (16 SCS0009 servos in the forearm, 20 joints, antagonistic
  tendon loop per joint on a drum, DIP coupled to PIP by a rigid bar, PTFE sheaths).
- `research/references/hands/README.md` and its reports (tendon-mechanics.md is key). The main gap is
  **fingertip force: ~2–3.4 N vs 9–20 N commercial, 45 N 1X NEO, 50–80 N human.**
- `cloud-claude/2026-11-09-v2-hand-cad/`: the latest concept, a full hand in CadQuery. **Read its
  README and `reports/05-review.md` first.** Its geometry works; its drivetrain does not:
  PTFE sheaths creep and compress (~8 mm at 45 N), 160–460° total wrap per strand (45–75 % efficiency),
  knots take ~90 % of the pull, 0.43 mm eyelet rims fray the line, sheath ends have no positive stop,
  strands cannot be replaced without teardown, walls < 1.6 mm, wrist gear 3–5× too small.
  Code: `cad/params.py`, `finger.py`, `thumb.py`, `palm.py`, `wrist.py`, `forearm.py`.
- `cloud-claude/2026-11-04-v1-wrist/` (wrist study), `hardware/cad/tendon_router.py`, `dip_linkage.py`.

## The mindset

Think like a first-principles engineer ("the best part is no part", "question every requirement",
"make the requirement less dumb, then delete, then simplify, then speed up, then automate").
- Go **wide**: many genuinely different ideas, including radical ones (change the architecture,
  not just the fillet).
- Then go **narrow**: for the best ideas, numbers (forces, stresses, friction, stiffness, mass, cost,
  print time) and concrete geometry a 3D printer can make.
- Every claim gets a number or a source. Hand calculations are fine; say what you assumed.
- Say what fails first and why your idea avoids it.

## Constraints that are real

- Owner: a learner building at home. FDM printer (assume Bambu/Prusa class, 0.4 mm nozzle; PETG, PLA,
  TPU 95A; PA-CF / PETG-CF possible). Budget hobby-scale (a full hand < ~€600 in parts is the goal).
- Everything must be open source and buildable by others from off-the-shelf parts.
- Sign convention: positive = closing. Frame of the V2 study (origin = wrist centre, +Z to fingers,
  −Y palm side, +X thumb side).
- Servo bus SCS0009 now (0.226 N·m @ 6 V); HLS3606M (0.59 N·m, current sensing, same footprint
  +2.25 mm tall) is the planned upgrade. You may argue for other actuators with numbers and cost.
- Hardware safety: nothing that is dangerous if a strand snaps (springs storing big energy, etc.).

## Output

Write your report to `reports/NN-<topic>.md` (your number is in your prompt). Structure:
1. **Top 3 recommendations** (one paragraph each, with the key number).
2. **Idea list** (wide): a table with idea, how it works, gain (number), cost/effort, risk, verdict.
3. **Deep dives** (narrow) on the best 2–4: maths, dimensions, sketch in words or ASCII,
   failure modes, a bench test that proves or kills it.
4. **What to build as CAD** in the next phase: precise parts with dimensions.
5. Sources.
Scripts for calculations go in `cad/` or `out/` with your number as prefix (e.g. `cad/02_friction.py`).
Plain English, short sentences, explain terms (the owner is learning). Units: mm, N, N·m, degrees.
