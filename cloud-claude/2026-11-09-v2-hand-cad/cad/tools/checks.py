"""Validity, interference and joint-sweep checks for CadQuery parts.

    from tools.checks import is_valid, interference, sweep
    is_valid(shape)                       -> dict(valid, watertight, volume, solids, faces)
    interference(parts, threshold=0.5, ignore=[("pin","hole_part")], bbox_pad=0.0)
                                          -> [(a, b, overlap_mm3), ...] sorted by overlap, largest first
    sweep(build_fn, "pip", [0, 45, 95], other_q={"mcp_flex": 30}, ignore=..., threshold=0.5)
                                          -> {value: [(a, b, vol), ...]}
build_fn(**q) must return a dict name -> shape; q holds joint_name=value (degrees) + other_q.
Ignore pairs may be given in either order; an entry "name" alone ignores every pair containing it,
and a prefix ending in "*" matches names that start with it ("pin_*").
"""
from __future__ import annotations

import itertools
import time

import numpy as np


def _shape(o):
    """Workplane/Shape -> a single cq.Shape (compound if several)."""
    import cadquery as cq
    if hasattr(o, "vals"):
        vs = o.vals()
        if len(vs) == 1:
            return vs[0]
        return cq.Compound.makeCompound([v for v in vs if hasattr(v, "wrapped")])
    return o


def _trimesh(o, tol=0.1, ang=0.3):
    import trimesh
    from .render import tessellate
    v, f = tessellate(o, tol, ang)
    return trimesh.Trimesh(v, f, process=True)


def is_valid(shape, tol=0.1):
    """Return dict(valid, watertight, volume (OCC), solids, faces, ok). `ok` = all three pass."""
    s = _shape(shape)
    valid = bool(s.isValid())
    vol = float(s.Volume())
    try:
        m = _trimesh(shape, tol)
        m.merge_vertices()
        wt = bool(m.is_watertight)
    except Exception:
        wt = False
    n_sol = len(s.Solids())
    return dict(valid=valid, watertight=wt, volume=vol, solids=n_sol, faces=len(s.Faces()),
                ok=valid and wt and vol > 0 and n_sol >= 1)


def _ignored(a, b, ignore):
    def hit(name, pat):
        return name == pat or (pat.endswith("*") and name.startswith(pat[:-1]))
    for ig in ignore or ():
        if isinstance(ig, str):
            if hit(a, ig) or hit(b, ig):
                return True
        elif (hit(a, ig[0]) and hit(b, ig[1])) or (hit(a, ig[1]) and hit(b, ig[0])):
            return True
    return False


def interference(parts, threshold=0.5, ignore=(), bbox_pad=0.0, tol=0.1, time_budget=None, verbose=False):
    """Pairwise overlap volumes (mm^3) above `threshold`. Boolean common via OCC; a bounding-box prefilter
    skips pairs that cannot touch. If a boolean fails the pair is re-checked by point sampling (approx)."""
    names = list(parts)
    shapes = {n: _shape(parts[n]) for n in names}
    bbs = {}
    for n, s in shapes.items():
        b = s.BoundingBox()
        bbs[n] = (b.xmin - bbox_pad, b.xmax + bbox_pad, b.ymin - bbox_pad, b.ymax + bbox_pad, b.zmin - bbox_pad, b.zmax + bbox_pad)
    res, t0 = [], time.time()
    for a, b in itertools.combinations(names, 2):
        if _ignored(a, b, ignore):
            continue
        A, B = bbs[a], bbs[b]
        if A[0] > B[1] or B[0] > A[1] or A[2] > B[3] or B[2] > A[3] or A[4] > B[5] or B[4] > A[5]:
            continue
        if time_budget and time.time() - t0 > time_budget:
            print("interference: time budget hit, remaining pairs skipped")
            break
        try:
            c = shapes[a].intersect(shapes[b])
            v = float(c.Volume())
        except Exception:
            v = _approx_overlap(parts[a], parts[b], tol)
        if verbose:
            print(f"  {a} x {b}: {v:.3f}")
        if v > threshold:
            res.append((a, b, round(v, 3)))
    return sorted(res, key=lambda r: -r[2])


def _approx_overlap(a, b, tol, n=20000):
    ma, mb = _trimesh(a, tol), _trimesh(b, tol)
    pts = ma.sample(n)
    inside = mb.contains(pts)
    return float(inside.mean()) * abs(ma.volume)


def sweep(build_fn, joint_name, values, other_q=None, ignore=(), threshold=0.5, **kw):
    """Rebuild with joint_name=v for each v and run interference. Returns {v: pairs}."""
    out = {}
    for v in values:
        q = dict(other_q or {})
        q[joint_name] = v
        parts = build_fn(**q)
        out[v] = interference(parts, threshold=threshold, ignore=ignore, **kw)
    return out


def report(results):
    """Pretty-print a sweep() result."""
    for v, pairs in results.items():
        print(f"{v}: " + ("clear" if not pairs else "; ".join(f"{a}x{b}={vol}" for a, b, vol in pairs)))
