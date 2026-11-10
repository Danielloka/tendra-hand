# 02 — Thumb (Tendra V2 concept, study 2026-11-09)

Thumb designer's report. Code: `cad/thumb_kinematics.py` (kinematics, tasks, optimiser, strength) and
`cad/thumb.py` (CadQuery model, tendon routing, decoupling + collision checks, renders, interface).
Outputs: `out/thumb/` (STEP assembly `thumb_rest.step`, `stl/*.stl`, `thumb_rest.glb`, renders `*.png`,
`thumb_design.json`, `routing.json`, `interface.json`, `collisions.json`, envelope hulls).

Run: `uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy
--with pillow python cad/thumb.py` (add `--quick` for a 9-pose collision sweep, `--no-render`).
`python cad/thumb_kinematics.py --x out/thumb/thumb_opt_x.json` re-finalises the design from an optimiser
vector; `--optimise` re-runs differential evolution (~1–2 h on 4 cores).

__BODY__
