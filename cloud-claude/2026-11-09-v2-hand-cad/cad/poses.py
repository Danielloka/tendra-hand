"""Tendra V2 concept: grasp-pose solver on the real CAD geometry (integrator fix round, 2026-11-10).

Every rigid body (finger yoke / proximal / middle / distal incl. pads, thumb base / metacarpal / proximal /
distal incl. pads, palm shell + pads) becomes a point cloud (surface samples) and a voxel grid (pitch 0.4 mm)
in its own rest frame. A pose moves the points; penetration = points of A inside B's voxels, gap = KD-tree
distance (or the analytic distance to the grasped cylinder).

    solve_power(55)      cylinder against the palm, every finger wraps it (0..1 mm, no penetration), thumb opposite
    solve_fist()         fingers curled as far as the palm allows, thumb onto the index/middle middle phalanges
    backoff_pinch(...)   thumb-index tip pinch / thumb-little opposition backed off to 0..0.5 mm contact
    neighbour_table()    smallest allowed relative abduction of neighbouring fingers (no contact)
    uv run ... python cad/poses.py  -> out/assembly/poses_solved.json, abd_pair_table.json
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import params as P  # noqa: E402

OUT = HERE.parent / "out" / "assembly"
FINGERS = ("index", "middle", "ring", "little")
PITCH = 0.4
_B: dict = {}


class Body:
    def __init__(self, shapes, n=2500):
        import trimesh
        from tools.render import tessellate
        ms = []
        for s in shapes:
            v, f = tessellate(s, 0.15, 0.25)
            ms.append(trimesh.Trimesh(v, f))
        m = trimesh.util.concatenate(ms)
        self.mesh = m
        self.pts = np.vstack([m.sample(n), m.vertices[:: max(1, len(m.vertices) // 1500)]])
        self.vox = m.voxelized(PITCH).fill()

    def inside(self, pts_local):
        return self.vox.is_filled(pts_local)


def _finger_bodies(f):
    import finger
    key = f"F_{f}"
    if key not in _B:
        lp = finger.local_parts(f)
        groups = {}
        for k, s in lp.items():
            b = finger.BODY_OF[k]
            if any(t in k for t in ("pin", "post", "eyelet", "ptfe")):
                continue
            groups.setdefault(b, []).append(s)
        _B[key] = {b: Body(v) for b, v in groups.items() if b in ("yoke", "proximal", "middle", "distal")}
    return _B[key]


def _thumb_bodies():
    import thumb
    if "T" not in _B:
        rp = thumb.rest_parts()
        groups = {}
        for k, s in rp.items():
            b = thumb.PART_BODY[k]
            if b == "mount" or any(t in k for t in ("pin", "nip", "bearing", "neck", "cap")):
                continue
            groups.setdefault(b, []).append(s)
        _B["T"] = {b: Body(v) for b, v in groups.items()}
        _B["T_pad"] = Body([rp["thumb_pad"]], 1500)
    return _B["T"]


def _palm():
    import cadquery as cq
    if "P" not in _B:
        d = OUT / "palm_cache"
        shapes = [cq.Shape.importBrep(str(d / f"{n}.brep")) for n in ("palm_shell", "palm_pad", "hypothenar_pad", "knuckle_pad")]
        _B["P"] = Body(shapes, 6000)
    return _B["P"]


# --- transforms ----------------------------------------------------------------------------------------
def finger_T(f, q):
    """body -> 4x4 (finger-local -> hand frame)"""
    import finger
    g = finger.geom(f)
    return finger.body_transforms(g, finger.full_q(g, q))


def thumb_T(q):
    """body -> (R, t) (rest world -> posed world)"""
    import thumb
    return thumb.body_transforms(q)


def _xf(M, p):
    return p @ M[:3, :3].T + M[:3, 3]


def _inv(M, p):
    return (p - M[:3, 3]) @ M[:3, :3]


def finger_points(f, q, bodies=("proximal", "middle", "distal")):
    T = finger_T(f, q)
    B = _finger_bodies(f)
    return {b: _xf(T[b], B[b].pts) for b in bodies if b in B}


def thumb_points(q, bodies=("metacarpal", "proximal", "distal")):
    F = thumb_T(q)
    B = _thumb_bodies()
    return {b: B[b].pts @ F[b][0].T + F[b][1] for b in bodies if b in B}


def inside_finger(f, q, pts, bodies=("yoke", "proximal", "middle", "distal")):
    T = finger_T(f, q)
    B = _finger_bodies(f)
    return sum(int(B[b].inside(_inv(T[b], pts)).sum()) for b in bodies if b in B)


def inside_thumb(q, pts, bodies=("base", "metacarpal", "proximal", "distal")):
    F = thumb_T(q)
    B = _thumb_bodies()
    n = 0
    for b in bodies:
        if b in B:
            Rm, t = F[b]
            n += int(B[b].inside((pts - t) @ Rm).sum())
    return n


def inside_palm(pts):
    return int(_palm().inside(pts).sum())


# --- cylinder -----------------------------------------------------------------------------------------
def cyl_dist(pts, o, c):
    v = pts - o
    return np.linalg.norm(v - np.outer(v @ c, c), axis=1)


def place_cylinder(D, gamma, zc, gap=0.4):
    """Axis in the x-z plane at gamma from +x, through (0, y0, zc); y0 so that the palm touches (gap)."""
    c = np.array([math.cos(math.radians(gamma)), 0.0, math.sin(math.radians(gamma))])
    pp = _palm().pts
    pp = pp[pp[:, 1] < -10]  # palm face side
    lo, hi = -80.0, -10.0
    for _ in range(40):  # bisection on y0: min distance palm -> axis = R + gap
        y0 = 0.5 * (lo + hi)
        d = cyl_dist(pp, np.array([0, y0, zc]), c).min()
        if d < D / 2 + gap:
            hi = y0
        else:
            lo = y0
    return np.array([0, lo, zc]), c


def finger_wrap(f, o, c, R, abd=0.0):
    """Grid over mcp/pip: no point inside the cylinder or the palm; minimise the pad gaps (prox + mid + dist)."""
    best = None
    for mcp in np.arange(0, 91, 2.0):
        for pip in np.arange(0, 96, 2.5):
            q = {"mcp_abd": abd, "mcp_flex": mcp, "pip": pip}
            pts = finger_points(f, q)
            gaps = {b: float(cyl_dist(p, o, c).min() - R) for b, p in pts.items()}
            if min(gaps.values()) < 0.0:
                continue
            score = gaps["proximal"] + gaps["middle"] + 1.5 * gaps["distal"]
            if best is None or score < best[0]:
                best = (score, q, gaps)
    if best is None:
        return None
    allp = np.vstack(list(finger_points(f, best[1]).values()))
    best[2]["palm_inside_pts"] = inside_palm(allp)
    return best


def thumb_solve(cost_fn, seeds):
    import thumb
    lo = np.array([thumb.RANGES[j][0] for j in thumb.JOINT_NAMES])
    hi = np.array([thumb.RANGES[j][1] for j in thumb.JOINT_NAMES])
    best = None
    for s0 in seeds:
        r = minimize(lambda x: cost_fn(dict(zip(thumb.JOINT_NAMES, np.clip(x, lo, hi)))) + 10 * np.sum((x - np.clip(x, lo, hi)) ** 2),
                     np.asarray(s0, float), method="Powell", options=dict(maxiter=1500, xtol=0.05, ftol=1e-4))
        if best is None or r.fun < best[0]:
            best = (float(r.fun), dict(zip(thumb.JOINT_NAMES, np.clip(r.x, lo, hi).tolist())))
    return best


def _fingers_all_pts(fq):
    return np.vstack([np.vstack(list(finger_points(f, q, ("yoke", "proximal", "middle", "distal")).values())) for f, q in fq.items()])


def solve_power(D=55.0, gammas=(12.0, 22.0), zcs=(64.0, 70.0, 76.0)):
    R = D / 2
    td = json.loads((HERE.parent / "out" / "thumb" / "thumb_design.json").read_text())["eval"]
    best = None
    for gm in gammas:
        for zc in zcs:
            o, c = place_cylinder(D, gm, zc)
            fq, gaps, sc = {}, {}, 0.0
            for f in FINGERS:
                r = finger_wrap(f, o, c, R)
                if r is None:
                    sc += 99
                    continue
                fq[f], gaps[f] = r[1], r[2]
                sc += r[0] + 5 * (r[2]["palm_inside_pts"] > 0)
            if best is None or sc < best[0]:
                best = (sc, gm, zc, o, c, fq, gaps)
    sc, gm, zc, o, c, fq, gaps = best
    fpts = _fingers_all_pts(fq)
    ftree = cKDTree(fpts)
    side = np.cross(c, [0, 0, 1.0])  # unit-ish vector across; palm side = +y from the axis

    def cost(q):
        tp = thumb_points(q)
        allp = np.vstack(list(tp.values()))
        d = cyl_dist(allp, o, c) - R
        J = 50 * float(np.sum(np.maximum(-d, 0) ** 2))  # no penetration of the cylinder
        pad = _thumb_pad_pts(q)
        dp = cyl_dist(pad, o, c) - R
        J += (max(dp.min(), 0) - 0.3) ** 2 * 4  # pad touching (0..0.6)
        J += 0.3 * max(float((cyl_dist(tp["proximal"], o, c) - R).min()), 0)
        cp = pad[np.argmin(dp)] - o
        J += 0.5 * max(cp[1] + 5.0, 0)  # contact on the far side of the cylinder (y below the axis)
        J += 2.0 * inside_finger_all(fq, allp) + 2.0 * inside_palm(allp)
        dd, _ = ftree.query(allp)
        J += 5 * float(np.sum(np.maximum(0.3 - dd, 0)))
        return J

    seeds = [td["power55"]["q_thumb"], td["power40"]["q_thumb"], td["power70"]["q_thumb"], [60, 50, 40, 40]]
    tj, tq = thumb_solve(cost, seeds)
    pad = _thumb_pad_pts(tq)
    tp = np.vstack(list(thumb_points(tq).values()))
    rep = {"cylinder": {"d": D, "gamma": gm, "zc": zc, "origin": o.round(3).tolist(), "axis": c.round(5).tolist()},
           "fingers": fq, "finger_gaps": gaps, "thumb": tq,
           "thumb_pad_gap": round(float((cyl_dist(pad, o, c) - R).min()), 2),
           "thumb_min_gap_all": round(float((cyl_dist(tp, o, c) - R).min()), 2),
           "thumb_inside_fingers_pts": inside_finger_all(fq, tp), "thumb_inside_palm_pts": inside_palm(tp), "cost": tj}
    return rep


def _thumb_pad_pts(q):
    F = thumb_T(q)
    Rm, t = F["distal"]
    return _thumb_bodies() and (_B["T_pad"].pts @ Rm.T + t)


def inside_finger_all(fq, pts):
    return sum(inside_finger(f, q, pts) for f, q in fq.items())


def solve_fist():
    """Fingers: largest flexion with no point inside the palm (and not inside the neighbour); thumb pad onto the
    dorsal side of the index / middle middle phalanges (0..1 mm), no penetration."""
    fq = {}
    for f in FINGERS:
        best = None
        for mcp in np.arange(90, 49, -2.5):
            for pip in np.arange(95, 49, -2.5):
                q = {"mcp_abd": 0.0, "mcp_flex": float(mcp), "pip": float(pip)}
                pts = np.vstack(list(finger_points(f, q, ("proximal", "middle", "distal")).values()))
                if inside_palm(pts) == 0:
                    sc = mcp + pip
                    if best is None or sc > best[0]:
                        best = (sc, q)
                    break
        fq[f] = best[1] if best else {"mcp_abd": 0.0, "mcp_flex": 70.0, "pip": 80.0}
    fpts = _fingers_all_pts(fq)
    ftree = cKDTree(fpts)
    tgt = np.vstack([finger_points(f, fq[f], ("middle",))["middle"] for f in ("index", "middle")])
    tgt_tree = cKDTree(tgt)
    td = json.loads((HERE.parent / "out" / "thumb" / "thumb_design.json").read_text())["eval"]

    def cost(q):
        tp = np.vstack(list(thumb_points(q).values()))
        pad = _thumb_pad_pts(q)
        d, _ = tgt_tree.query(pad)
        J = 2.0 * (d.min() - 0.4) ** 2
        J += 2.0 * inside_finger_all(fq, tp) + 2.0 * inside_palm(tp) + 2.0 * inside_finger_all(fq, pad)
        dd, _ = ftree.query(tp)
        J += 5 * float(np.sum(np.maximum(0.25 - dd, 0)))
        return J

    seeds = [td["lateral"]["q_thumb"], td["index"]["q_thumb"], td["middle"]["q_thumb"], [60, 30, 50, 40], [90, 0, 60, 40]]
    tj, tq = thumb_solve(cost, seeds)
    pad = _thumb_pad_pts(tq)
    tp = np.vstack(list(thumb_points(tq).values()))
    return {"fingers": fq, "thumb": tq, "thumb_pad_gap_to_index_middle": round(float(tgt_tree.query(pad)[0].min()), 2),
            "thumb_inside_fingers_pts": inside_finger_all(fq, tp) + inside_finger_all(fq, pad),
            "thumb_inside_palm_pts": inside_palm(tp), "cost": tj}


def backoff(fname, fq, tq, step=0.25, maxdeg=15.0):
    """Open the finger's mcp/pip (equal steps) until the thumb's distal/pad no longer penetrates it; report gap."""
    pad = _thumb_pad_pts(tq)
    tp = np.vstack([pad] + list(thumb_points(tq, ("proximal", "distal")).values()))
    d = 0.0
    while d <= maxdeg:
        q = dict(fq)
        q["mcp_flex"] = fq["mcp_flex"] - d * 0.4
        q["pip"] = fq["pip"] - d
        n = inside_finger(fname, q, tp, ("middle", "distal"))
        if n == 0:
            fp = np.vstack(list(finger_points(fname, q, ("middle", "distal")).values()))
            gap = float(cKDTree(fp).query(pad)[0].min())
            return {"finger_q": q, "backoff_deg": d, "pad_gap_mm": round(gap, 2)}
        d += step
    return {"finger_q": fq, "backoff_deg": None, "pad_gap_mm": None}


def finger_pad_pts(f, q):
    """Distal pad (pad_tip) points of a finger at q (hand frame)."""
    import finger
    key = f"pad_{f}"
    if key not in _B:
        _B[key] = Body([finger.local_parts(f)["pad_tip"]], 1500)
    return _xf(finger_T(f, q)["distal"], _B[key].pts)


def solve_opposition(f, fq, tq_seed, target=0.3):
    """Thumb pad to the finger's tip pad: gap `target` (0..0.5 mm), no penetration of thumb into finger/palm."""
    fpad = finger_pad_pts(f, fq)
    ftree = cKDTree(fpad)

    def cost(q):
        pad = _thumb_pad_pts(q)
        d = ftree.query(pad)[0]
        tp = np.vstack([pad] + list(thumb_points(q).values()))
        J = 4.0 * (d.min() - target) ** 2
        J += 3.0 * inside_finger(f, fq, tp) + 2.0 * inside_palm(tp)
        return J

    tj, tq = thumb_solve(cost, [tq_seed, list(np.asarray(tq_seed) + [3, -3, 3, 3]), list(np.asarray(tq_seed) + [-3, 3, -3, -3])])
    pad = _thumb_pad_pts(tq)
    tp = np.vstack([pad] + list(thumb_points(tq).values()))
    return {"finger_q": fq, "thumb": tq, "pad_gap_mm": round(float(ftree.query(pad)[0].min()), 2),
            "thumb_inside_finger_pts": inside_finger(f, fq, tp), "thumb_inside_palm_pts": inside_palm(tp), "cost": tj}


def neighbour_table(qs=(-15.0, -7.5, 0.0, 7.5, 15.0), mcps=(0.0, 45.0, 90.0), step=1.5):
    """For each neighbour pair (radial a, ulnar b), flexion m (both fingers mcp = pip = m) and ulnar abd q_b: the
    set of radial abd q_a in [-15, 15] with no contact (yoke..distal, voxel test, agrees with OCC), as intervals.
    Curled fingertips lie below the abd pivot, so at high flexion abduction swings the tips the other way."""
    out = {}
    grid = np.arange(-15.0, 15.01, step)
    for a, b in (("index", "middle"), ("middle", "ring"), ("ring", "little")):
        rows = []
        for m in mcps:
            for qb in qs:
                qbd = {"mcp_abd": qb, "mcp_flex": m, "pip": m}
                pb = np.vstack(list(finger_points(b, qbd, ("yoke", "proximal", "middle", "distal")).values()))
                free = [float(qa) for qa in grid if inside_finger(a, {"mcp_abd": float(qa), "mcp_flex": m, "pip": m}, pb) == 0]
                iv, cur = [], None
                for qa in grid:
                    if float(qa) in free:
                        cur = [float(qa), float(qa)] if cur is None else [cur[0], float(qa)]
                    elif cur is not None:
                        iv.append(cur)
                        cur = None
                if cur is not None:
                    iv.append(cur)
                rows.append({"flex": m, "q_ulnar": qb, "free_q_radial": iv})
        out[f"{a}-{b}"] = rows
    return out


def main():
    """Each task is written to poses_solved.json as soon as it is solved; --resume skips solved tasks."""
    t0 = time.time()
    fp = OUT / "poses_solved.json"
    res = json.loads(fp.read_text()) if ("--resume" in sys.argv and fp.exists()) else {}

    def save():
        fp.write_text(json.dumps(res, indent=1, default=float))

    if "power55" not in res:
        res["power55"] = solve_power(55.0)
        save()
        print("power", json.dumps(res["power55"], default=str)[:900], f"{time.time() - t0:.0f}s", flush=True)
    if "fist" not in res:
        res["fist"] = solve_fist()
        save()
        print("fist", json.dumps(res["fist"], default=str)[:700], f"{time.time() - t0:.0f}s", flush=True)
    td = json.loads((HERE.parent / "out" / "thumb" / "thumb_design.json").read_text())["eval"]
    F = ("mcp_abd", "mcp_flex", "pip")
    abd_cap = {"index": 15.0, "middle": 15.0, "ring": 9.0, "little": 9.0}  # abd_pair_table.json (ring/little)
    for key, fname in (("tip_pinch", "index"), ("little", "little")):
        if f"pinch_{key}" in res:
            continue
        fq = dict(zip(F, td[key]["q_finger"]))
        fq["mcp_abd"] = min(fq["mcp_abd"], abd_cap[fname])
        res[f"pinch_{key}"] = solve_opposition(fname, fq, td[key]["q_thumb"])
        save()
        print(key, res[f"pinch_{key}"], f"{time.time() - t0:.0f}s", flush=True)


if __name__ == "__main__":
    if "--table" in sys.argv:
        print(json.dumps(neighbour_table(), indent=0))
    else:
        main()
