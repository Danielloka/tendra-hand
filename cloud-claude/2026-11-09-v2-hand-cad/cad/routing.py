"""Tendra V2 concept: palm routing of the 32 tendon sheaths (integrator, study 2026-11-09).

Every strand runs from its source in the hand (finger yoke exit, finger abd mouth, thumb mount exit) to its
cell in the wrist's palm sheath clamp (top face z = 19, tubes leave the clamp along -Z). Each path is a
degree-7 Bezier curve whose first and last three control points are collinear with the start / end
directions, so the path is G2 (zero curvature) where it meets the straight sheath in the yoke and in the
clamp. The free control points are optimised strand by strand (several passes) for:
    min bend radius   >= 15 mm for PTFE sheaths (target 16), >= 5 mm for steel guide tubes (target 6)
    clearance         >= 0.4 mm between tube walls (2.4 mm centre to centre), relaxed to the 2.15 mm
                         clamp pitch where both tubes are within 5 mm of the clamp top
    keep-outs         thumb mount + thumb base sweep (+3 mm: bore wall + bay clearance), wrist flange
                         bolts, mount inserts, palm skin (|y| <= 11.3 inside the 28 mm palm)
Finger mcp_flex/pip sheaths: the first 22 mm below the yoke is the FREE span (in a pocket) that bends with
mcp_abd; the palm channel starts at the end of it (the 'guide'). Its direction is free here (the finger
report assumed -Z), which lets the free span pre-turn up to ~40 deg; the free span is re-checked at abd
+-15 deg with the moved yoke exit (least-bent cubic between the moved exit and the fixed guide).

WHY steel guide tubes for the thumb: the thumb mount's 8 strand exits sit at z 22-26, only 3-7 mm above the
clamp top, 21-50 mm to the side and pointing away (-a1 / the rot drum plane). A PTFE path with R >= 15
needs >= 28 mm of height for that offset, so it is geometrically impossible. The 8 thumb strands therefore
run bare inside bent stainless hypotubes (18G, 1.27 x 0.84, R >= 5, the "bicycle V-brake noodle"), from the
clamp top to the mount sockets. The PTFE sheath from the forearm ends in the clamp. Cost: wrap friction.

    uv run --no-project --with numpy --with scipy --with trimesh --with cadquery python cad/routing.py
writes out/assembly/routes.json (paths, per-strand length / min radius / clearance) and prints a table.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment, minimize
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import params as P  # noqa: E402

OUT = HERE.parent / "out" / "assembly"
Z_CELL = P.SHEATH_CLAMP_Z[1]  # 19: clamp top
R_PTFE, R_STEEL = 15.0, 4.0
TARGET = {"ptfe": 16.0, "steel": 5.0}
Z_CELL_THUMB = 15.5  # the clamp top is stepped down to z 15.5 over the thumb cells (integrator change)
CELL_SWAPS = [(13, 15), (19, 21), (25, 27)]  # wrist map -> palm map: thumb gets the whole outer (+x) column
ROT_IDLER_R = 3.6  # cmc_rot idler pulley (MR63 + printed tyre, groove r 3.4) strand-centre radius


def step_zone(x, y):
    """Clamp-top step (z 15.5 instead of 19) over the thumb cells: outer column and the -y rows of the +x half."""
    return (x > 4.25) | ((x > 0) & (y < -2.15))
OD = {"ptfe": P.SHEATH_OD, "steel": 1.27}
GAP = 0.4
FREE_SPAN = 22.0
Y_SKIN = 14.0 - P.WALL_MIN - P.SHEATH_OD / 2 - 0.2  # 11.2: tube centre inside the palm skin
N_S = 72  # samples per path
WALL_TO_BAY = 3.0  # tube centre to the thumb mount / base sweep surface (1.1 r + 1.6 wall + 0.3)
NAMES_ID = {v: k for k, v in P.SERVO_IDS.items()}


# ----------------------------------------------------------------------------------------------------
# Sources
# ----------------------------------------------------------------------------------------------------
def _finger_exits(name, abd):
    import finger
    b = finger.build(name, {"mcp_abd": abd}, with_parts=False)
    return {e["strand"]: (np.array(e["point"]), np.array(e["direction"])) for e in b["interface"]["strand_exits"]}


def strand_table():
    """32 strands: name, sid, role, kind, start point / direction, and the moved exits at abd -+15 (fingers)."""
    import thumb
    out = []
    for f in ("index", "middle", "ring", "little"):
        ex0, exm, exp = _finger_exits(f, 0.0), _finger_exits(f, -15.0), _finger_exits(f, 15.0)
        for k, (p, d) in ex0.items():
            joint, role = k.rsplit("_", 1)
            kind = "abd" if "mcp_abd" in k else "free"
            out.append(dict(name=k, finger=f, joint=joint, role=role, sid=NAMES_ID[joint], kind=kind, mat="ptfe",
                            start=p, dir=d, moved={-15: exm[k], 15: exp[k]} if kind == "free" else None))
    ti = thumb.interface()
    for k, e in ti["strand_exits"].items():
        joint, role = k.rsplit("_", 1)
        d = dict(name=k, finger="thumb", joint=joint, role=role, sid=NAMES_ID[joint],
                 kind="thumb_rot" if "cmc_rot" in k else "thumb_axial", mat="steel",
                 start=np.array(e["point"]), dir=np.array(e["direction"]), moved=None)
        if d["kind"] == "thumb_rot":
            d["exit"], d["exit_dir"] = d["start"], d["dir"]
            d["idler"] = rot_idler(d["start"], d["dir"])
            d["start"], d["dir"] = d["idler"]["depart"], d["idler"]["depart_dir"]
        out.append(d)
    return out


def rot_idler(e, w, target=(14.0, -24.0, 21.0), lead=0.6, rise=0.12):
    """Idler pulley under a cmc_rot sheath socket: the strand leaves the socket along w, wraps the pulley and
    departs horizontally toward the thenar-heel run. Returns centre, axis, wrap, depart point/direction, arc."""
    e, w = np.asarray(e, float), np.asarray(w, float)
    a = e + lead * w  # first contact
    d = np.asarray(target, float) - a
    d[2] = 0.0
    d /= np.linalg.norm(d)
    d = d + np.array([0, 0, rise])
    d /= np.linalg.norm(d)
    k = np.cross(w, d)
    k /= np.linalg.norm(k)
    n = np.cross(k, w)  # toward the inside of the turn
    c = a + ROT_IDLER_R * n
    ang = math.acos(float(np.clip(w @ d, -1, 1)))
    from tools.transform import rot_matrix
    arc = [c + rot_matrix(k, math.degrees(t)) @ (a - c) for t in np.linspace(0, ang, 16)]
    return dict(centre=c, axis=k, wrap_deg=math.degrees(ang), depart=arc[-1], depart_dir=d, arc=np.array(arc), contact=a)


def cells():
    import wrist_kin as K
    return K.tube_offsets()


def assign_cells(strands):
    """Palm-optimal cell map: right half (thumb, index, middle_abd) and left half (middle pip/mcp, ring,
    little) separately, Hungarian on scaled 'arrival' coordinates (where each strand comes from)."""
    C = cells()
    res = {}
    for side in (+1, -1):
        idx = [i for i, s in enumerate(strands) if (P.SERVO_IDS[s["sid"]].startswith(("thumb", "index"))
                                                    or s["sid"] == 10) == (side > 0)]
        cid = [j for j, c in enumerate(C) if np.sign(c[0]) == side]
        arr = []
        for i in idx:
            s = strands[i]
            x, y = s["start"][0], s["start"][1]
            if s["kind"] == "thumb_rot":
                x, y = 60.0, y - 10  # comes in low from the +x palmar side
            elif s["kind"] == "thumb_axial":
                x = 45.0  # low, from +x
            elif s["finger"] == "index":
                x = 8.0  # passes ulnar of the thumb mount, descends from above at x ~ 5
            arr.append((x, y))
        arr = np.array(arr)
        cc = C[cid]
        a = (arr - arr.mean(0)) / (arr.std(0) + 1e-9)
        b = (cc - cc.mean(0)) / (cc.std(0) + 1e-9)
        w = np.array([1.0, 1.3])
        cost = (((a[:, None, :] - b[None, :, :]) * w) ** 2).sum(-1)
        r, c = linear_sum_assignment(cost)
        for i, j in zip(r, c):
            res[strands[idx[i]]["name"]] = cid[j]
    return res


# ----------------------------------------------------------------------------------------------------
# Obstacles
# ----------------------------------------------------------------------------------------------------
_OBS = {}


def obstacles():
    """Voxel point clouds (pitch 1 mm) of the thumb mount and the thumb base sweep hull + analytic bits."""
    if _OBS:
        return _OBS
    import trimesh
    import thumb
    from tools.render import tessellate
    rp = thumb.rest_parts()
    v, f = tessellate(rp["thumb_mount"], 0.1, 0.2)
    mount = trimesh.Trimesh(v, f)
    hull = trimesh.load(str(HERE.parent / "out" / "thumb" / "thumb_base_sweep_hull.stl"))
    pts = [mount.voxelized(1.0).fill().points, hull.voxelized(1.0).fill().points]
    # wrist flange bolts (screw + head + access bore) and the 3 thumb-mount heat-set inserts
    for bx, by in P.WRIST_FLANGE_BOLTS:
        zz = np.arange(15, 26, 1.0)
        th = np.linspace(0, 2 * np.pi, 12, endpoint=False)
        for r in (0.0, 1.5, 3.0):
            pts.append(np.array([[bx + r * np.cos(t), by + r * np.sin(t), z] for z in zz for t in th]))
    ti = thumb.interface()
    a1 = np.array(ti["mount"]["bore_axis"])
    for b in ti["mount"]["bolts"]:
        p = np.array(b["point"])
        pts.append(np.array([p - a1 * s for s in np.arange(0, 7, 0.7)]))
    _OBS["thumb"] = cKDTree(np.vstack(pts[:2]))
    _OBS["bolts"] = cKDTree(np.vstack(pts[2:]))
    _OBS["a1"] = a1
    return _OBS


# ----------------------------------------------------------------------------------------------------
# Curves
# ----------------------------------------------------------------------------------------------------
DEG_N = 7
_T = np.linspace(0, 1, N_S)
_B = np.array([[math.comb(DEG_N, i) * t ** i * (1 - t) ** (DEG_N - i) for i in range(DEG_N + 1)] for t in _T])
_B1 = np.array([[math.comb(DEG_N - 1, i) * t ** i * (1 - t) ** (DEG_N - 1 - i) for i in range(DEG_N)] for t in _T])
_B2 = np.array([[math.comb(DEG_N - 2, i) * t ** i * (1 - t) ** (DEG_N - 2 - i) for i in range(DEG_N - 1)] for t in _T])


def ctrl(x, S, dS, C):
    """x = (a1, a2, b1, b2, m1(3), m2(3)) -> 8 control points. a2 > a1 > 0 along dS, b2 > b1 > 0 above C."""
    a1, da, b1, db = np.abs(x[:4]) + 0.3
    up = np.array([0, 0, 1.0])
    return np.array([S, S + a1 * dS, S + (a1 + da) * dS, x[4:7], x[7:10], C + (b1 + db) * up, C + b1 * up, C])


def curve(cp):
    p = _B @ cp
    d1 = DEG_N * (_B1 @ np.diff(cp, axis=0))
    d2 = DEG_N * (DEG_N - 1) * (_B2 @ np.diff(cp, 2, axis=0))
    sp = np.linalg.norm(d1, axis=1)
    k = np.linalg.norm(np.cross(d1, d2), axis=1) / np.maximum(sp, 1e-9) ** 3
    return p, k, sp


def dense(cp, n=400):
    t = np.linspace(0, 1, n)
    B = np.array([[math.comb(DEG_N, i) * tt ** i * (1 - tt) ** (DEG_N - i) for i in range(DEG_N + 1)] for tt in t])
    return B @ cp


def radii(pts):
    from tools.transform import curvature_radii
    return curvature_radii(pts)


# ----------------------------------------------------------------------------------------------------
# Objective
# ----------------------------------------------------------------------------------------------------
def cost(x, s, others):
    S, dS, C = s["start"], s["dir"], s["cell_xyz"]
    cp = ctrl(x, S, dS, C)
    p, k, sp = curve(cp)
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    L = float(np.sum(seg))
    kmax = 1.0 / TARGET[s["mat"]]
    turn = float(np.sum(0.5 * (k[1:] + k[:-1]) * seg))  # total bend (rad) = capstan friction exponent / mu
    J = 0.02 * L + (W_TURN_STEEL if s["mat"] == "steel" else W_TURN) * turn
    J += 2e6 * float(np.sum(np.maximum(k - kmax, 0) ** 2))
    if s["kind"] == "free":
        J += free_span_cost(s, p, seg)
    ob = obstacles()
    if s["mat"] == "ptfe":
        d, _ = ob["thumb"].query(p, distance_upper_bound=WALL_TO_BAY + 1)
        J += 50 * float(np.sum(np.maximum(WALL_TO_BAY + 0.5 - np.minimum(d, 99), 0) ** 2))
    else:  # steel tubes start at the mount face: ignore their first 1.5 mm, then keep 1.2 from the voxels
        far = np.cumsum(np.r_[0, np.linalg.norm(np.diff(p, axis=0), axis=1)]) > 1.5
        d, _ = ob["thumb"].query(p[far], distance_upper_bound=3)
        J += 50 * float(np.sum(np.maximum(1.2 - np.minimum(d, 99), 0) ** 2))
    d, _ = ob["bolts"].query(p, distance_upper_bound=5)
    J += 50 * float(np.sum(np.maximum(OD[s["mat"]] / 2 + 1.2 - np.minimum(d, 99), 0) ** 2))
    # palm skin (thumb strands may use the thenar heel: y down to -27, x up to 52)
    if s["mat"] == "ptfe":
        J += 20 * float(np.sum(np.maximum(np.abs(p[:, 1]) - Y_SKIN, 0) ** 2 * (p[:, 2] < 66)))
    else:
        J += 20 * float(np.sum(np.maximum(p[:, 1] - Y_SKIN, 0) ** 2 + np.maximum(-28 - p[:, 1], 0) ** 2))
    if s["mat"] == "ptfe":
        J += 50 * float(np.sum(np.maximum(Z_CELL + 0.2 - p[:-6, 2], 0) ** 2))  # stay above the clamp top
    else:  # steel: above the stepped clamp (16.5) in the step zone, above 19.2 over the rest of the clamp block
        q = p[:-6]
        inblock = (np.abs(q[:, 0]) < 11.7) & (np.abs(q[:, 1]) < 9.2) & ~step_zone(q[:, 0], q[:, 1])
        J += 50 * float(np.sum(np.maximum(Z_CELL + 0.2 - q[:, 2], 0) ** 2 * inblock))
        J += 50 * float(np.sum(np.maximum(Z_CELL_THUMB + 0.2 - q[:, 2], 0) ** 2))
    J += 20 * float(np.sum(np.maximum(15.5 - p[:, 2], 0) ** 2))
    # clearance to the other strands
    if others is not None:
        tree, mats, zs = others
        d, j = tree.query(p, k=1, distance_upper_bound=4)
        ok = np.isfinite(d)
        if ok.any():
            req = (OD[s["mat"]] + np.where(mats[j[ok]] == 1, OD["steel"], OD["ptfe"])) / 2 + GAP + 0.25
            near = (p[ok, 2] < Z_CELL + 4) & (zs[j[ok]] < Z_CELL + 4)
            req = np.where(near, 2.1, req)
            J += W_CLEAR * float(np.sum(np.maximum(req - d[ok], 0) ** 2))
    return J


W_CLEAR = 600.0
W_TURN = 3.0
W_TURN_STEEL = 8.0
_HT = np.linspace(0, 1, 40)[:, None]
_H = [(2 * _HT**3 - 3 * _HT**2 + 1), (_HT**3 - 2 * _HT**2 + _HT), (-2 * _HT**3 + 3 * _HT**2), (_HT**3 - _HT**2)]
_H1 = [(6 * _HT**2 - 6 * _HT), (3 * _HT**2 - 4 * _HT + 1), (-6 * _HT**2 + 6 * _HT), (3 * _HT**2 - 2 * _HT)]
_H2 = [(12 * _HT - 6), (6 * _HT - 4), (-12 * _HT + 6), (6 * _HT - 2)]


def _hermite_kmax(P0, T0, P1, T1):
    d1 = _H1[0] * P0 + _H1[1] * T0 + _H1[2] * P1 + _H1[3] * T1
    d2 = _H2[0] * P0 + _H2[1] * T0 + _H2[2] * P1 + _H2[3] * T1
    sp = np.linalg.norm(d1, axis=1)
    return float((np.linalg.norm(np.cross(d1, d2), axis=1) / sp**3).max())


def free_span_cost(s, p, seg):
    """Penalty if the free span (first 22 mm) bends tighter than 15.5 mm at abd -15/+15."""
    cs = np.r_[0, np.cumsum(seg)]
    i = min(int(np.searchsorted(cs, FREE_SPAN)), len(p) - 2)
    G = p[i]
    TG = p[i + 1] - p[i - 1]
    TG /= np.linalg.norm(TG)
    J = 0.0
    for Sa, da in s["moved"].values():
        km = min(_hermite_kmax(Sa, da * h, G, TG * h2) for h in (14.0, 22.0, 30.0) for h2 in (14.0, 22.0, 30.0))
        J += 2e6 * max(km - 1 / 17.0, 0) ** 2
    return J


def initial(s):
    S, dS, C = s["start"], s["dir"], s["cell_xyz"]
    if s["kind"] in ("free", "abd"):
        m1 = S + np.array([0, 0, -0.35 * (S[2] - C[2])])
        m2 = np.array([C[0], C[1], C[2] + 0.35 * (S[2] - C[2])])
        if s["finger"] == "index":  # pass ulnar of the thumb mount
            m1 = np.array([S[0] - 8, S[1], S[2] - 14])
            m2 = np.array([max(C[0], 3.5), C[1], 30.0])
        if s["finger"] == "little":
            m1 = np.array([S[0] + 10, S[1], S[2] - 18])
            m2 = np.array([C[0] - 12, C[1], C[2] + 12])
        x = np.r_[4.0, 4.0, 4.0, 4.0, m1, m2]
    elif s["kind"] == "thumb_axial":
        h = C - S
        h[2] = 0
        h /= np.linalg.norm(h)
        m1 = S + 1.0 * dS + 4.0 * h + np.array([0, 0, 0.6])
        m2 = C + np.array([0, 0, 6.5]) + 3.5 * (-h)
        x = np.r_[0.6, 0.6, 1.0, 1.5, m1, m2]
    else:  # thumb_rot: from the idler along the thenar heel (palmar of the mount), then in from +x/-y
        m1 = S + 12 * dS + np.array([0, 0, 1.5])
        m2 = np.array([C[0] + 5, C[1] - 7, C[2] + 6.5])
        x = np.r_[2.0, 3.0, 1.0, 1.5, m1, m2]
    return x


def fillet_polyline(corners, R, step=0.4):
    """Polyline corners -> dense line/arc path with every interior corner replaced by a tangent arc of radius R.
    Returns (points, ok) where ok = every segment is long enough for its two tangent lengths."""
    c = [np.asarray(q, float) for q in corners]
    pts, ok = [c[0]], True
    tl = [0.0] * len(c)
    for i in range(1, len(c) - 1):
        u, v = c[i] - c[i - 1], c[i + 1] - c[i]
        u, v = u / np.linalg.norm(u), v / np.linalg.norm(v)
        th = math.acos(float(np.clip(u @ v, -1, 1)))
        tl[i] = R * math.tan(th / 2)
    for i in range(1, len(c)):
        if np.linalg.norm(c[i] - c[i - 1]) < tl[i - 1] + (tl[i] if i < len(c) - 1 else 0) - 1e-6:
            ok = False
    for i in range(1, len(c) - 1):
        u, v = c[i] - c[i - 1], c[i + 1] - c[i]
        u, v = u / np.linalg.norm(u), v / np.linalg.norm(v)
        th = math.acos(float(np.clip(u @ v, -1, 1)))
        a, b = c[i] - u * tl[i], c[i] + v * tl[i]
        seg = np.linspace(pts[-1], a, max(2, int(np.linalg.norm(a - pts[-1]) / step) + 1))[1:]
        pts.extend(seg)
        if th > 1e-4:
            k = np.cross(u, v)
            k /= np.linalg.norm(k)
            n = np.cross(k, u)
            ctr = a + n * R
            from tools.transform import rot_matrix
            for t in np.linspace(0, th, max(3, int(R * th / step) + 2))[1:]:
                pts.append(ctr + rot_matrix(k, math.degrees(t)) @ (a - ctr))
        else:
            pts.append(b)
    end = c[-1]
    pts.extend(np.linspace(pts[-1], end, max(2, int(np.linalg.norm(end - pts[-1]) / step) + 1))[1:])
    return np.array(pts), ok


def steel_route(s, placed, R=5.0):
    """Grid search over the corner positions of a filleted polyline (bent hypotube) for one thumb strand:
    minimise total bend + penalties (clearance to `placed` strands, the thumb voxels, the clamp block)."""
    S, dS, C = s["start"], s["dir"], s["cell_xyz"]
    ob = obstacles()
    tree = cKDTree(np.vstack([p for p in placed.values()])) if placed else None
    best = None
    h = C - S
    h[2] = 0
    h /= np.linalg.norm(h)
    if s["kind"] == "thumb_axial":
        grid = [(t1, zk, off) for t1 in (3.9, 4.6, 5.5, 6.5) for zk in np.arange(20.5, 28.1, 0.5) for off in (-4, -2, 0, 2, 4)]
    else:
        grid = [(t1, zk, (off, zw)) for t1 in (3.0, 6.0, 10.0) for zk in np.arange(20.5, 25.1, 0.5)
                for off in (-28, -26, -24, -22) for zw in (16.8, 17.8, 18.8)]
    for t1, zk, off in grid:
        K1 = S + dS * t1
        if s["kind"] == "thumb_axial":
            side = np.cross(h, [0, 0, 1.0])
            K2 = np.array([C[0], C[1], zk]) + side * off * 0.5
            corners = [S, K1, K2, np.array([C[0], C[1], zk - 0.01 if off else zk]), C] if off else [S, K1, np.array([C[0], C[1], zk]), C]
            if off:
                corners = [S, K1, K2 - h * 4.0, np.array([C[0], C[1], zk]), C]
        else:  # rot: along the thenar heel palmar of the mount, then to the cell
            W = np.array([13.0, off[0], off[1]])
            corners = [S, K1, W, np.array([C[0], C[1], zk]), C]
        pts, ok = fillet_polyline(corners, R)
        if not ok:
            continue
        seg = np.diff(pts, axis=0)
        t = seg / np.maximum(np.linalg.norm(seg, axis=1, keepdims=True), 1e-9)
        turn = float(np.sum(np.arccos(np.clip((t[1:] * t[:-1]).sum(1), -1, 1))))
        J = turn + 0.002 * float(np.linalg.norm(seg, axis=1).sum())
        far = np.cumsum(np.r_[0, np.linalg.norm(seg, axis=1)]) > 1.5
        d, _ = ob["thumb"].query(pts[far], distance_upper_bound=3)
        J += 20 * float(np.sum(np.maximum(1.3 - np.minimum(d, 99), 0) ** 2))
        d, _ = ob["bolts"].query(pts, distance_upper_bound=5)
        J += 20 * float(np.sum(np.maximum(1.9 - np.minimum(d, 99), 0) ** 2))
        q = pts[:-3]
        inblock = (np.abs(q[:, 0]) < 11.7) & (np.abs(q[:, 1]) < 9.2) & ~step_zone(q[:, 0], q[:, 1])
        J += 20 * float(np.sum(np.maximum(Z_CELL + 0.2 - q[:, 2], 0) ** 2 * inblock))
        J += 20 * float(np.sum(np.maximum(Z_CELL_THUMB + 0.3 - q[:, 2], 0) ** 2))
        J += 20 * float(np.sum(np.maximum(pts[:, 1] - Y_SKIN, 0) ** 2))
        if tree is not None:
            d, _ = tree.query(pts)
            near = pts[:, 2] < Z_CELL_THUMB + 4.5
            req = np.where(near, 1.5, OD["steel"] + GAP + 0.1)
            J += 50 * float(np.sum(np.maximum(req - d, 0) ** 2))
        if best is None or J < best[0]:
            best = (J, pts, corners)
    return best


def steel_layout(strands, verbose=True, n_orders=14):
    """Place the 8 thumb guide tubes greedily; try several placement orders, keep the lowest total cost."""
    steel = [s for s in strands if s["mat"] == "steel"]
    rot = [s for s in steel if s["kind"] == "thumb_rot"]
    ax = [s for s in steel if s["kind"] == "thumb_axial"]
    rng = np.random.default_rng(0)
    orders = [sorted(ax, key=lambda s: -s["start"][2]), sorted(ax, key=lambda s: s["start"][2]),
              sorted(ax, key=lambda s: s["cell_xyz"][1]), sorted(ax, key=lambda s: -s["cell_xyz"][1]),
              sorted(ax, key=lambda s: s["cell_xyz"][0]), sorted(ax, key=lambda s: -s["cell_xyz"][0])]
    while len(orders) < n_orders:
        orders.append([ax[i] for i in rng.permutation(len(ax))])
    best = None
    for o in orders:
        placed, res, tot = {}, {}, 0.0
        for s in rot + o:
            J, pts, corners = steel_route(s, placed)
            res[s["name"]] = (J, pts, corners)
            placed[s["name"]] = pts
            tot += J
        if best is None or tot < best[0]:
            best = (tot, res)
    for s in steel:
        J, pts, corners = best[1][s["name"]]
        s["pts"], s["corners"], s["J"] = pts, [np.asarray(c).round(3).tolist() for c in corners], J
    if verbose:
        print(f"steel guide tubes: best total cost {best[0]:.2f} over {len(orders)} orders")


def free_span_check(s, cp):
    """Free span (first 22 mm) between the moved yoke exit and the fixed guide, at abd -15/+15: least-bent
    cubic Hermite (tangent length optimised). Returns (min R over both, guide point, guide tangent)."""
    P = dense(cp, 600)
    sl = np.cumsum(np.r_[0, np.linalg.norm(np.diff(P, axis=0), axis=1)])
    i = int(np.searchsorted(sl, FREE_SPAN))
    G, TG = P[i], (P[i + 1] - P[i - 1]) / np.linalg.norm(P[i + 1] - P[i - 1])
    worst = math.inf
    for a, (Sa, da) in s["moved"].items():
        km = min(_hermite_kmax(Sa, da * h, G, TG * h2) for h in np.linspace(10, 36, 14) for h2 in np.linspace(10, 36, 14))
        worst = min(worst, 1 / km)
    return worst, G, TG


def route(strands, cell_map, passes=4, verbose=True):
    C = cells()
    for s in strands:
        c = C[cell_map[s["name"]]]
        s["cell"] = int(cell_map[s["name"]])
        s["cell_xyz"] = np.array([c[0], c[1], Z_CELL_THUMB if s["mat"] == "steel" else Z_CELL])
        s["x"] = initial(s)
    # steel guide tubes: deterministic bent-tube paths, placed greedily (rot first, then axial by exit height)
    steel_layout(strands, verbose)
    order = [i for i in sorted(range(len(strands)), key=lambda i: {"index": 0, "little": 1, "ring": 2, "middle": 3}.get(strands[i]["finger"], 0))
             if strands[i]["mat"] == "ptfe"]
    global W_CLEAR
    for ps, w in zip(range(passes), (100.0, 600.0, 2500.0, 6000.0)):
        W_CLEAR = w
        t0 = time.time()
        for i in order:
            s = strands[i]
            others = _others(strands, i, ps)
            r = minimize(cost, s["x"], args=(s, others), method="Powell",
                         options=dict(maxiter=5000 if ps == 0 else 2500, xtol=1e-3, ftol=1e-6))
            s["x"] = r.x
            s["J"] = float(r.fun)
        if verbose:
            print(f"pass {ps} (w_clear {w:.0f}): {time.time() - t0:.1f} s, total J {sum(strands[i]['J'] for i in order):.2f}")
    rng = np.random.default_rng(1)
    for i in sorted(order, key=lambda i: -strands[i]["J"])[:8]:
        s = strands[i]
        others = _others(strands, i, 9)
        best = (s["J"], s["x"])
        for k in range(5):
            x0 = best[1] + np.r_[rng.normal(0, 0.8, 4), rng.normal(0, 2.5, 6)]
            r = minimize(cost, x0, args=(s, others), method="Powell", options=dict(maxiter=3000, xtol=1e-3, ftol=1e-6))
            if r.fun < best[0]:
                best = (float(r.fun), r.x)
        s["J"], s["x"] = best
    if verbose:
        print(f"retry: total J {sum(strands[i]['J'] for i in order):.2f}")
    return strands


def path_of(s, n=400):
    if s["mat"] == "steel":
        return s["pts"]
    return dense(ctrl(s["x"], s["start"], s["dir"], s["cell_xyz"]), n)


def _others(strands, i, ps):
    pts, mats, zs = [], [], []
    for j, o in enumerate(strands):
        if j == i or (ps == 0 and "J" not in o):
            continue
        p = o["pts"] if o["mat"] == "steel" else curve(ctrl(o["x"], o["start"], o["dir"], o["cell_xyz"]))[0]
        pts.append(p)
        mats.append(np.full(len(p), 1 if o["mat"] == "steel" else 0))
        zs.append(p[:, 2])
    if not pts:
        return None
    pts = np.vstack(pts)
    return cKDTree(pts), np.concatenate(mats), np.concatenate(zs)


# ----------------------------------------------------------------------------------------------------
# Report
# ----------------------------------------------------------------------------------------------------
def evaluate(strands):
    rows = []
    allp = {s["name"]: path_of(s) for s in strands}
    trees = {k: cKDTree(v) for k, v in allp.items()}
    mat = {s["name"]: s["mat"] for s in strands}
    for s in strands:
        p = allp[s["name"]]
        r = radii(p)
        L = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
        clear, who = math.inf, None
        for o in strands:
            if o is s:
                continue
            d, _ = trees[o["name"]].query(p)
            far = (p[:, 2] > Z_CELL + 5)
            if far.any():
                dm = float((d[far]).min()) - (OD[s["mat"]] + OD[mat[o["name"]]]) / 2
                if dm < clear:
                    clear, who = dm, o["name"]
        row = dict(name=s["name"], sid=s["sid"], role=s["role"], kind=s["kind"], material=s["mat"], cell=s["cell"],
                   length=round(L, 1), min_radius=round(float(r.min()), 2), clearance=round(clear, 2), nearest=who)
        # wrap angle (total turning) -> friction factor at mu 0.1 (steel) / 0.1 (PTFE)
        t = np.diff(p, axis=0)
        t /= np.linalg.norm(t, axis=1, keepdims=True)
        turn = float(np.degrees(np.sum(np.arccos(np.clip((t[1:] * t[:-1]).sum(1), -1, 1)))))
        row["turn_deg"] = round(turn, 1)
        row["friction_factor_mu0.1"] = round(math.exp(0.1 * math.radians(turn)), 3)
        if s["kind"] == "free":
            rf, G, TG = free_span_check(s, ctrl(s["x"], s["start"], s["dir"], s["cell_xyz"]))
            row["free_span_min_r_abd15"] = round(rf, 1)
            row["guide"] = np.round(G, 2).tolist()
            row["guide_dir"] = np.round(TG, 3).tolist()
        row["ok"] = bool(r.min() >= (R_PTFE if s["mat"] == "ptfe" else R_STEEL) and clear >= GAP - 1e-6
                         and row.get("free_span_min_r_abd15", 99) >= R_PTFE)
        rows.append(row)
    return rows, allp


def save(strands, rows, allp, path=OUT / "routes.json"):
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {"note": "palm routing, hand frame at wrist q=0 (palm frame), mm. points from the source (finger yoke "
                    "exit / abd mouth / thumb mount exit) to the clamp-cell top (z 19). material ptfe = PTFE 1x2 "
                    "sheath (min R 15), steel = 18G stainless guide tube (bare strand inside, min R 5).",
            "cell_map": {s["name"]: s["cell"] for s in strands},
            "strands": []}
    for s, row in zip(strands, rows):
        extra = {}
        if "idler" in s:
            extra["idler"] = {k: (np.round(v, 3).tolist() if isinstance(v, np.ndarray) else v) for k, v in s["idler"].items()}
            extra["mount_exit"] = s["exit"].round(3).tolist()
        data["strands"].append({**row, **extra, "start": s["start"].round(3).tolist(), "start_dir": s["dir"].round(4).tolist(),
                                "end": s["cell_xyz"].round(3).tolist(), "x": np.round(s["x"], 4).tolist(),
                                "corners": s.get("corners"),
                                "points": (np.round(allp[s["name"]], 3).tolist() if s["mat"] == "steel" else
                                           np.round(allp[s["name"]][::5], 3).tolist() + [np.round(allp[s["name"]][-1], 3).tolist()])})
    path.write_text(json.dumps(data, indent=1))
    return path


def load(path=OUT / "routes.json"):
    """-> list of dicts with 'name', 'material', 'points' (dense, rebuilt from x), 'cell', ..."""
    d = json.loads(Path(path).read_text())
    out = []
    for s in d["strands"]:
        if s["material"] == "steel":
            out.append({**s, "dense": np.array(s["points"])})
            continue
        cp = ctrl(np.array(s["x"]), np.array(s["start"]), np.array(s["start_dir"]), np.array(s["end"]))
        out.append({**s, "dense": dense(cp, 300)})
    return out, d


def forearm_fan_check(cell_map, strands):
    """The forearm fan (adjuster -> comb) with the palm cell map: min bend radius and close pairs."""
    import forearm
    from tools.transform import min_bend_radius
    worst, paths = math.inf, {}
    for s in strands:
        sid, role = s["sid"], s["role"]
        start = forearm.strand_points(sid)[role]["sheath_start"]
        p = forearm.sheath_path(start, cell_map[s["name"]])
        paths[s["name"]] = np.asarray(p)
        worst = min(worst, min_bend_radius(p)[0])
    close = 0
    names = list(paths)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            d = np.linalg.norm(paths[names[i]][:, None] - paths[names[j]][None], axis=-1).min()
            close += d < P.SHEATH_OD
    return dict(min_radius=round(float(worst), 1), pairs_closer_than_od=int(close))


def wrist_cell_map(strands, swaps=()):
    """The wrist/forearm designer's map (forearm.comb_cells), with optional cell swaps."""
    import forearm
    cm = forearm.comb_cells()
    m = {s["name"]: cm[(s["sid"], s["role"])] for s in strands}
    inv = {v: k for k, v in m.items()}
    for a, b in swaps:
        m[inv[a]], m[inv[b]] = b, a
    if swaps:  # thumb axial strands: same lateral order at the mount exit and in the clamp (no crossing)
        C = cells()
        ax = [s for s in strands if s["kind"] == "thumb_axial"]
        lat = np.array([0.2, -0.98])
        ex = sorted(ax, key=lambda s: -(s["start"][:2] @ lat))
        cs = sorted([m[s["name"]] for s in ax], key=lambda c: -(C[c] @ lat))
        for s, c in zip(ex, cs):
            m[s["name"]] = c
    return m


def main():
    t0 = time.time()
    st = strand_table()
    cmap = assign_cells(st) if "--palm-map" in sys.argv else wrist_cell_map(st, CELL_SWAPS)
    route(st, cmap, passes=3)
    rows, allp = evaluate(st)
    p = save(st, rows, allp)
    print(f"{'strand':24s} {'id':>3s} {'cell':>4s} {'mat':5s} {'L':>6s} {'Rmin':>6s} {'clear':>6s} {'turn':>6s} {'free':>5s} ok")
    for r in rows:
        print(f"{r['name']:24s} {r['sid']:3d} {r['cell']:4d} {r['material']:5s} {r['length']:6.1f} {r['min_radius']:6.1f} "
              f"{r['clearance']:6.2f} {r['turn_deg']:6.0f} {r.get('free_span_min_r_abd15', ''):>5} {r['ok']}")
    print("forearm fan with this cell map:", forearm_fan_check(cmap, st))
    print("wrote", p, f"{time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
