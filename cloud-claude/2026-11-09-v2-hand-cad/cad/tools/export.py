"""Export a dict of parts to STEP assembly, per-part STL and one GLB.

    from tools.export import export_all
    export_all(parts, "out/foo", name="hand", colors={"a": "#d04040"})
Writes <outdir>/<name>.step (one assembly, named + coloured parts), <outdir>/stl/<part>.stl (mm) and
<outdir>/<name>.glb (trimesh scene, per-part colours, units = MILLIMETRES, Z up as in the CAD frame).
Returns dict of written paths. glb_scale=0.001 gives metres instead.
"""
from __future__ import annotations

import re
from pathlib import Path

import numpy as np

from .render import PALETTE, _hex, tessellate


def _rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def _safe(n):
    return re.sub(r"[^A-Za-z0-9_.-]", "_", n)


def export_all(parts, outdir, name="assembly", colors=None, tol=0.05, ang=0.2, glb_scale=1.0, step=True, stl=True, glb=True):
    import cadquery as cq
    import trimesh
    out = Path(outdir)
    out.mkdir(parents=True, exist_ok=True)
    colors = colors or {}
    paths = {}
    col = {n: _hex(colors.get(n, PALETTE[i % len(PALETTE)])) for i, n in enumerate(parts)}
    if step:
        asm = cq.Assembly(name=name)
        for n, o in parts.items():
            if isinstance(o, (str, Path)):
                continue
            r, g, b = _rgb(col[n])
            wp = o if hasattr(o, "val") else cq.Workplane(obj=o)
            asm.add(wp, name=_safe(n), color=cq.Color(r / 255, g / 255, b / 255))
        p = out / f"{name}.step"
        asm.export(str(p), exportType="STEP")
        paths["step"] = str(p)
    if stl:
        (out / "stl").mkdir(exist_ok=True)
    scene = trimesh.Scene()
    for n, o in parts.items():
        v, f = tessellate(o, tol, ang)
        if len(f) == 0:
            continue
        m = trimesh.Trimesh(v * glb_scale, f, process=True)
        if stl:
            m.export(str(out / "stl" / f"{_safe(n)}.stl")) if glb_scale == 1.0 else trimesh.Trimesh(v, f).export(str(out / "stl" / f"{_safe(n)}.stl"))
        m.visual.face_colors = np.tile(list(_rgb(col[n])) + [255], (len(m.faces), 1))
        scene.add_geometry(m, node_name=_safe(n), geom_name=_safe(n))
    if stl:
        paths["stl_dir"] = str(out / "stl")
    if glb:
        p = out / f"{name}.glb"
        scene.export(str(p))
        paths["glb"] = str(p)
    return paths
