# cad/tools

Run everything from `cad/` with:
`uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy --with pillow python your_script.py`
and `from tools.render import render` (add `cad/` to `sys.path` if your script lives elsewhere). One-off setup (already done): `npm i three@0.170.0` in this folder (`node_modules/` is git-ignored).

- **render.py** `render(parts, out_png, views=("iso","front","side","top"), colors=None, size=(1600,1200), title=None, tol=0.15, ang=0.3, edges=True, axes=True, backend="auto")`. `parts` = dict name -> CadQuery Workplane/Shape/STL/STEP path. Headless Chromium + three.js (z-buffer, lights, edge lines, scale bar, legend), grid of views in one PNG; falls back to a numpy rasteriser if node/chromium fails. Views: iso, iso2, front (from -Y, palm side), back, side (from +X), left, top, bottom, or `(elev, azim)`. CLI: `python tools/render.py a.stl b.step -o out.png --views iso,front --title T`.
- **checks.py** `is_valid(shape)`, `interference(parts, threshold=0.5, ignore=[("pin","hole_part"), "name", "pin_*"])` (bbox prefilter + OCC boolean common, volumes in mm^3), `sweep(build_fn, "pip", [0,45,95], other_q={...})` (build_fn(**q) -> parts dict), `report(sweep_result)`.
- **transform.py** `rotate_about(shape, point, dir, deg)`, `place`, `chain_rotations`, `rotate_point` (move anchor points), `polyline_length`, `arc_on_drum`, `tangent_point`, `strand_length`, `spline_through`, `min_bend_radius`, `check_sheath(pts, min_r=15)`.
- **export.py** `export_all(parts, outdir, name, colors=None)` -> `<name>.step` (named, coloured assembly), `stl/<part>.stl`, `<name>.glb` (mm, Z up).
- **demo.py** self-test: `python -m tools.demo` (from `cad/`) writes `out/tools_demo/`.
