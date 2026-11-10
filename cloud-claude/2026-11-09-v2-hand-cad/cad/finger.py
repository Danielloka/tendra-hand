"""Tendra V2 concept hand: one parametric finger module for index, middle, ring and little (study 2026-11-09).

Frame: the hand frame of params.py (origin = wrist centre, +Z to the fingers, -Y palm side, +X thumb side).
Each finger is first built in a *finger-local* frame = the hand frame shifted so the MCP flexion axis
passes through the origin (params.FINGERS[f]["mcp"]); the finger points along +Z at q = 0.

Joints (positive = closing):
    mcp_abd  axis +Y through (0, 0, -ABD_OFFSET)    + = toward the thumb (+X), every finger
    mcp_flex axis +X through (0, 0, 0)              + = toward the palm (-Y)
    pip      axis +X through (0, 0, L1)
    dip      axis +X through (0, 0, L1 + L2)        no servo: dip = f(pip) through a rigid bar (~0.75 x pip)

Tendon drive (one antagonistic loop per servo joint, drum radius params.R_DRUM = 7 mm on every joint):
    mcp_abd   drum = disc about Y on the dorsal end of the yoke; its 2 strands go to sheaths fixed in the palm.
    mcp_flex  drum = proximal phalanx base; grooves at x = -4.3 (flex, palm side) / +4.3 (ext, back side).
    pip       drum = middle phalanx base; grooves at x = -1.4 (flex) / +1.4 (ext).
The mcp_flex and pip strands arrive in PTFE sheaths that are clamped in the yoke (the abduction joint
is crossed inside the sheaths = Bowden, exact). The pip strands cross the MCP flexion axis bare, through a
stainless eyelet whose rim sits just proximal of the axis (see crossing_length(): < 0.2 mm). Every strand
ends on a 1.5 mm steel post that sits flush with its groove floor (the strand runs from the drum onto the
post without a reverse bend, 1-1.5 wraps, then a knot).

API: build(name, q=None) -> {"parts", "joints", "strands", "interface"}; see the bottom for __main__.
Run: uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy \
         --with pillow --with rtree python cad/finger.py [--quick] [--finger index] [--no-export]
"""

from __future__ import annotations

import argparse
import functools
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STUDY = HERE.parent
REPO = STUDY.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "hardware" / "cad"))

import dip_linkage as DL  # noqa: E402  (V1 maths, reused read-only)
import params as P  # noqa: E402

OUT = STUDY / "out" / "finger"
FINGER_NAMES = ("index", "middle", "ring", "little")

# ------------------------------------------------------------------------------------------------
# Design constants (mm). Values that are requests to params.py are marked [REQ].
# ------------------------------------------------------------------------------------------------
R = P.R_DRUM  # 7.0, tendon centreline radius on every drum
LINE_R = P.LINE_D / 2  # 0.18
GROOVE_FLOOR = R - LINE_R  # 6.82
GROOVE_W = P.GROOVE_W  # 1.0
ABD_OFFSET = 11.5  # [REQ] MCP abduction axis below the flexion axis (params default 10, V1 19)
KNUCKLE_MIN_W = 17.0  # MCP knuckle (yoke + drum) is at least this wide (little finger: 17 > 15.5)
X_MCP_GROOVE = 4.3  # mcp_flex grooves (flex -x, ext +x)
X_PIP = 1.4  # pip strands / grooves (flex -x, ext +x)
SLOT_HALF = 2.2  # open slot in the MCP drum where the pip strands cross the axis
STUB_X0 = 2.65  # MCP stub axles start here (the strand slot + eyelets stay free of steel)
SOCKET_TOP = -8.5  # yoke top face (sheath mouths); clears the MCP drum flange by >= 0.25
TUBE_STOP = -9.5  # PTFE sheath ends here (2.2 bore below, 1.2 bore above)
BASE_BOT = -16.0  # yoke bottom (sheaths leave the finger here, pointing -Z)
YOKE_Y = (-9.0, 9.4)
ABD_DISC_Y = (9.0, 14.6)
ABD_GROOVE_Y = {"flex": 10.7, "ext": 13.3}  # flex = +x side strand (pulls toward the thumb)
ABD_PIN_D = P.PIN_D_BIG  # 3 mm dowels, palmar + dorsal (pins far apart: stiff against finger twist)
ABD_PIN_Y = {"palmar": (-12.5, -6.25), "dorsal": (8.0, 18.0)}  # palm holds y < -9.25 and y > 14.85
PALM_GUIDE_DZ = 22.0  # sheath free span below the yoke before the palm's guide channel
ABD_SOCKET_DZ = 12.0  # abd strands: sheath mouths in the palm this far below the abd axis
PIN_D = P.PIN_D_FINGER  # 2 mm dowels: MCP stubs, PIP, DIP
POST_D = 1.5  # anchor posts (steel), flush with the groove floor
POST_R_C = GROOVE_FLOOR - POST_D / 2  # post centre radius (6.07)
POCKET_R = 1.45  # cavity around a post so the strand can wrap it
ANCHOR = {"mcp_flex": 50.0, "pip": 45.0}  # anchor (material) angle, deg, in the (y, z) plane, 0 = +y
LINK_PIN_D = 1.5  # DIP bar pins
BAR_T, BAR_W = 1.2, 3.2  # DIP bar (stainless strip, or PETG for a first print)
RECESS = 1.6  # the -x side face is recessed this much where the bar runs (bar stays inside W)
EYELET_OD, EYELET_ID, EYELET_RIM, EYELET_L = 1.27, 0.84, 0.25, 5.0  # 18G stainless tube
SHEATH_OD, SHEATH_BORE, BARE_BORE = P.SHEATH_OD, P.SHEATH_BORE, P.BARE_BORE
CLR = P.CLEAR_SLIDE  # 0.25
PAD_T = 2.2  # fingertip pad pocket depth (PETG face sits this far above the outline)
LIMITS = dict(P.FINGER_RANGES)
STOP_MARGIN = 0.5  # deg: hard stops sit 2 x margin beyond the servo range

# ------------------------------------------------------------------------------------------------
# Small 2D/3D maths
# ------------------------------------------------------------------------------------------------


def rot2(deg):
    """Flexion in the (y, z) plane: +deg turns +z toward -y."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def rotm(axis, deg):
    d = np.asarray(axis, float) / np.linalg.norm(axis)
    a = math.radians(deg)
    K = np.array([[0, -d[2], d[1]], [d[2], 0, -d[0]], [-d[1], d[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def phi_pt(phi, r, zc=0.0):
    """Point (y, z) at polar angle phi (deg, 0 = +z, 90 = -y = palm side) and radius r about (0, zc)."""
    a = math.radians(phi)
    return (-r * math.sin(a), zc + r * math.cos(a))


def wrap_path(A, B, C, r, ccw=True, n=24):
    """Taut strand A -> B that keeps disk (C, r) on its left (ccw) / right. Returns (length, points)."""
    A, B, C = (np.asarray(v, float) for v in (A, B, C))
    if not ccw:
        M = np.array([1.0, -1.0])
        L, pts = wrap_path(A * M, B * M, C * M, r, True, n)
        return L, [p * M for p in pts]
    d = B - A
    nrm = np.array([-d[1], d[0]]) / np.linalg.norm(d)
    t = float(np.dot(C - A, d) / np.dot(d, d))
    side = float(np.dot(C - A, nrm))  # > 0: the disk is on the left of the straight line
    if side >= r or (side > 0 and (t < 0 or t > 1) and min(np.linalg.norm(C - A), np.linalg.norm(C - B)) >= r):
        return float(np.linalg.norm(d)), [A, B]
    da, db = np.linalg.norm(A - C), np.linalg.norm(B - C)
    ta = math.atan2(*(A - C)[::-1]) + math.acos(min(1.0, r / da))
    tb = math.atan2(*(B - C)[::-1]) - math.acos(min(1.0, r / db))
    arc = (tb - ta) % (2 * math.pi)
    L = math.sqrt(max(da * da - r * r, 0)) + math.sqrt(max(db * db - r * r, 0)) + r * arc
    pts = [A] + [C + r * np.array([math.cos(ta + arc * k / n), math.sin(ta + arc * k / n)]) for k in range(n + 1)] + [B]
    return L, pts


def tangent_from(Pt, C, r, side):
    """Tangent point on circle (C, r) seen from Pt, on the side y < C_y (side=-1) or y > C_y (+1)."""
    Pt, C = np.asarray(Pt, float), np.asarray(C, float)
    d = np.linalg.norm(Pt - C)
    th = math.atan2(*(Pt - C)[::-1])
    a = math.acos(r / d)
    cands = [C + r * np.array([math.cos(th + s * a), math.sin(th + s * a)]) for s in (1, -1)]
    return min(cands, key=lambda p: side * -(p[0] - C[0]))


def plength(pts):
    p = np.asarray(pts, float)
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


# ------------------------------------------------------------------------------------------------
# Per-finger geometry (pure numbers, no CAD)
# ------------------------------------------------------------------------------------------------


@functools.cache
def geom(name: str) -> dict:
    f = P.FINGERS[name]
    L1, L2, L3 = f["lengths"]
    W, H = f["width"], f["height"]
    hw, hh = W / 2, H / 2
    hk = max(W, KNUCKLE_MIN_W) / 2
    r_fl = max(R + 0.6, min(8.0, hh))  # drum flange radius (>= 0.6 above the line)
    g = dict(name=name, mcp=np.array(f["mcp"], float), L1=L1, L2=L2, L3=L3, W=W, H=H, hw=hw, hh=hh, hk=hk,
             r_fl=r_fl, r_boss=r_fl + CLR, r_band=r_fl + 2 * CLR,
             xd=hk - 2.5,  # MCP drum half width (cheeks 2.25 thick)
             xdp=min(5.0, hw - 4.0),  # PIP drum half width
             xt=hw - 4.25,  # DIP tongue half width
             z_pip=L1, z_dip=L1 + L2, z_tip=L1 + L2 + L3)
    # DIP bar (V1 maths, lengths from params). Bar sits outside the axle heads -> AXIS_CLEAR 2 mm is fine.
    DL.AXIS_CLEAR = 2.0
    g["link"] = DL.design(float(L2))
    # eyelet entrance: optimise its height so the pip strands' length change over the MCP range is smallest
    best = None
    for z0 in np.arange(-1.0, 0.81, 0.025):
        g["z0"] = float(z0)
        e = max(v.max() - v.min() for v in crossing_table(g).values())  # peak-to-peak
        if best is None or e < best[0]:
            best = (e, float(z0))
    g["z0"] = best[1]
    return g


def dip_of(g, pip_deg):
    """DIP angle (deg) for a PIP angle (deg), from the bar (exact circle-circle solution)."""
    lk = g["link"]
    a, ga = lk["pin_a"]["r_mm"], math.radians(lk["pin_a"]["angle_deg"])
    b, gb = lk["pin_b"]["r_mm"], math.radians(lk["pin_b"]["angle_deg"])
    pip = np.radians(np.atleast_1d(np.asarray(pip_deg, float)))
    grid = np.concatenate([[0.0], pip])  # dip_curve anchors the branch at the sample nearest 0
    d = DL.dip_curve(g["L2"], a, ga, b, gb, grid)
    out = np.degrees(d[1:])
    return float(out[0]) if np.ndim(pip_deg) == 0 else out


def link_pins_local(g):
    """Pin A (on the proximal) and pin B (on the distal) in the finger-local (y, z) plane at q = 0."""
    lk = g["link"]
    a, ga = lk["pin_a"]["r_mm"], math.radians(lk["pin_a"]["angle_deg"])
    b, gb = lk["pin_b"]["r_mm"], math.radians(lk["pin_b"]["angle_deg"])
    # dip_linkage plane: x along the middle phalanx (= +z here), y toward the back (= +y here)
    A = np.array([a * math.sin(ga), g["z_pip"] + a * math.cos(ga)])
    B = np.array([b * math.sin(gb), g["z_dip"] + b * math.cos(gb)])
    return A, B


def eyelet_axis(g, side):
    """Eyelet entrance P0, axis u (toward the PIP drum tangent), tangent point T (proximal frame, (y, z))."""
    C = np.array([0.0, g["L1"]])
    P0 = np.array([0.0, g["z0"]])
    T = tangent_from(P0 + np.array([0, EYELET_L]), C, R, side)
    u = (T - P0) / np.linalg.norm(T - P0)
    return P0, u, T


def crossing_length(g, side, q_mcp, with_points=False):
    """Bare pip strand from its sheath mouth in the yoke to the PIP drum tangent T, in the proximal
    frame, through the eyelet. The eyelet rim is modelled as two lips (radius rim + line) at the tube mouth;
    the strand wraps whichever is on the inside of its bend."""
    P0, u, T = eyelet_axis(g, side)
    nrm = np.array([u[1], -u[0]])  # right-hand normal (toward +y for an axis along +z)
    rim = EYELET_RIM
    lipL = P0 - nrm * (EYELET_ID / 2 + rim) + u * rim  # palm side lip (left of travel)
    lipR = P0 + nrm * (EYELET_ID / 2 + rim) + u * rim
    rho = rim + LINE_R
    A = rot2(-q_mcp) @ np.array([0.0, SOCKET_TOP])  # sheath mouth (yoke) seen from the proximal phalanx
    l1, p1 = wrap_path(A, T, lipL, rho, True)
    l2, p2 = wrap_path(A, T, lipR, rho, False)
    L, pts = (l1, p1) if l1 >= l2 else (l2, p2)
    return (L, pts) if with_points else L


def crossing_table(g, qs=None):
    qs = np.arange(LIMITS["mcp_flex"][0], LIMITS["mcp_flex"][1] + 1e-9, 2.5) if qs is None else qs
    out = {}
    for side, key in ((-1, "flex"), (1, "ext")):
        v = np.array([crossing_length(g, side, q) for q in qs])
        out[key] = v - crossing_length(g, side, 0.0)
    return out


# ------------------------------------------------------------------------------------------------
# Kinematics
# ------------------------------------------------------------------------------------------------


def full_q(g, q):
    q = {"mcp_abd": 0.0, "mcp_flex": 0.0, "pip": 0.0, **(q or {})}
    q["dip"] = dip_of(g, q["pip"])
    return q


def body_transforms(g, q):
    """4x4 transforms finger-local(q=0) -> hand frame for each body."""
    def T(Rm, p):  # rotation Rm about point p
        M = np.eye(4)
        M[:3, :3] = Rm
        M[:3, 3] = p - Rm @ p
        return M

    tr = np.eye(4)
    tr[:3, 3] = g["mcp"]
    abd = T(rotm((0, 1, 0), q["mcp_abd"]), np.array([0, 0, -ABD_OFFSET]))
    mcp = T(rotm((1, 0, 0), q["mcp_flex"]), np.zeros(3))
    pip = T(rotm((1, 0, 0), q["pip"]), np.array([0, 0, g["z_pip"]]))
    dip = T(rotm((1, 0, 0), q["dip"]), np.array([0, 0, g["z_dip"]]))
    palm = tr
    yoke = tr @ abd
    prox = yoke @ mcp
    mid = prox @ pip
    dist = mid @ dip
    return {"palm": palm, "yoke": yoke, "proximal": prox, "middle": mid, "distal": dist}


def xf(M, pts):
    p = np.atleast_2d(np.asarray(pts, float))
    return (p @ M[:3, :3].T) + M[:3, 3]


def lift(x, yz):
    """(y, z) points in a plane x = const -> 3D."""
    yz = np.atleast_2d(np.asarray(yz, float))
    return np.column_stack([np.full(len(yz), x), yz[:, 0], yz[:, 1]])


# ------------------------------------------------------------------------------------------------
# Strands (numbers): 3D polylines in the hand frame + exact lengths
# ------------------------------------------------------------------------------------------------


def _arc(C, r, t0, t1, n=40):
    t = np.radians(np.linspace(t0, t1, max(2, int(abs(t1 - t0) / 3) + 2)))
    return np.column_stack([C[0] + r * np.cos(t), C[1] + r * np.sin(t)])


def strands(g, q):
    """Each strand from its drum anchor (on the post) to the finger's proximal exit. Polylines in the
    hand frame; 'length' is the bare+drum length from the anchor to the sheath mouth (the sheath part is
    constant), computed from the body-frame geometry."""
    q = full_q(g, q)
    Tb = body_transforms(g, q)
    out = []
    # --- mcp_flex: (u, v) = (y, z) in the proximal frame; drum centre (0, 0)
    for role, x, th_t in (("flex", -X_MCP_GROOVE, 180.0), ("ext", X_MCP_GROOVE, 0.0)):
        th_a = ANCHOR["mcp_flex"] + q["mcp_flex"]  # anchor angle in space-aligned coordinates
        mouth = np.array([R if role == "ext" else -R, SOCKET_TOP])
        arc_sp = _arc((0, 0), R, th_a, th_t)  # space-aligned (yoke frame) arc from anchor to tangent
        wrap = (th_t - th_a) if role == "flex" else (th_a - th_t)
        L = abs(SOCKET_TOP) + R * math.radians(wrap)
        pts = np.vstack([xf(Tb["yoke"], lift(x, arc_sp)), xf(Tb["yoke"], lift(x, [mouth])),
                         xf(Tb["yoke"], [[x, mouth[0], BASE_BOT]])])
        out.append(dict(joint="mcp_flex", role=role, x_local=x, points=pts.tolist(), length=L, wrap_deg=wrap,
                        sheathed_exit=True))
    # --- pip: crossing (proximal frame) + PIP drum (middle frame, centre (0, L1))
    for role, x, side in (("flex", -X_PIP, -1), ("ext", X_PIP, 1)):
        Lc, cpts = crossing_length(g, side, q["mcp_flex"], with_points=True)
        P0, u, T = eyelet_axis(g, side)
        C = np.array([0.0, g["L1"]])
        th_t = math.degrees(math.atan2(*(T - C)[::-1]))  # tangent angle (proximal frame, fixed)
        th_a = ANCHOR["pip"] + q["pip"]  # anchor angle seen from the proximal frame
        if role == "flex":
            th_t = th_t % 360
            wrap = th_t - th_a
        else:
            th_t = (th_t + 180) % 360 - 180
            wrap = th_a - th_t
        L = Lc + R * math.radians(wrap)
        arc_p = _arc(C, R, th_a, th_t)
        cross_p = np.array(cpts[::-1])  # from T back to the mouth (proximal frame)
        pts = np.vstack([xf(Tb["proximal"], lift(x, arc_p)), xf(Tb["proximal"], lift(x, cross_p[1:])),
                         xf(Tb["yoke"], [[x, 0.0, BASE_BOT]])])
        out.append(dict(joint="pip", role=role, x_local=x, points=pts.tolist(), length=L, wrap_deg=wrap,
                        sheathed_exit=True))
    # --- mcp_abd: (x, z) plane at y = groove, centre (0, -ABD_OFFSET); + abd turns the disc clockwise
    for role, xs in (("flex", 1.0), ("ext", -1.0)):
        y = ABD_GROOVE_Y[role]
        th_a = 90.0 - q["mcp_abd"]  # anchor at the top of the disc (in palm-aligned coordinates)
        th_t = 0.0 if role == "flex" else 180.0
        wrap = (th_a - th_t) if role == "flex" else (th_t - th_a)
        t = np.radians(np.linspace(th_a, th_t, 30))
        arc = np.column_stack([R * np.cos(t), np.full(30, y), -ABD_OFFSET + R * np.sin(t)])
        mouth = [xs * R, y, -ABD_OFFSET - ABD_SOCKET_DZ]
        pts = xf(Tb["palm"], np.vstack([arc, [mouth]]))
        L = ABD_SOCKET_DZ + R * math.radians(wrap)
        out.append(dict(joint="mcp_abd", role=role, x_local=None, points=pts.tolist(), length=L, wrap_deg=wrap,
                        sheathed_exit=True))
    return out


# ------------------------------------------------------------------------------------------------
# CAD helpers (CadQuery)
# ------------------------------------------------------------------------------------------------


def _cq():
    import cadquery as cq
    return cq


def box(x0, x1, y0, y1, z0, z1):
    cq = _cq()
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def fbox(x0, x1, y0, y1, z0, z1, r, sel="|Z"):
    cq = _cq()
    w = cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
    try:
        w = w.edges(sel).fillet(r)
    except Exception:
        pass
    return w.val()


def cyl(r, p0, p1):
    cq = _cq()
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    return cq.Solid.makeCylinder(r, float(np.linalg.norm(d)), cq.Vector(*p0), cq.Vector(*(d / np.linalg.norm(d))))


def cylx(r, x0, x1, y=0.0, z=0.0):
    return cyl(r, (x0, y, z), (x1, y, z))


def sector_x(phi0, phi1, r, x0, x1, zc=0.0, r_in=0.0):
    """Prism along X of the annular sector phi0..phi1 (deg, phi convention of phi_pt) about (y=0, z=zc)."""
    cq = _cq()
    n = max(3, int(abs(phi1 - phi0) / 4) + 2)
    outer = [phi_pt(p, r, zc) for p in np.linspace(phi0, phi1, n)]
    inner = [phi_pt(p, r_in, zc) for p in np.linspace(phi1, phi0, n)] if r_in > 0 else [(0.0, zc)]
    pts = outer + inner
    wp = cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0)
    return wp.val()


def ring_groove_x(xc, r_in, r_out, zc=0.0, w=GROOVE_W):
    return cylx(r_out, xc - w / 2, xc + w / 2, 0, zc).cut(cylx(r_in, xc - w / 2 - 0.1, xc + w / 2 + 0.1, 0, zc))


def U(*shapes):
    s = shapes[0]
    for o in shapes[1:]:
        s = s.fuse(o)
    return s.clean()


def keep_near_joint(shape, zc, phi0, phi1, r_inner, r_far=16.0, x0=-30, x1=30):
    """Keep `shape` only inside the angular window phi0..phi1 around a joint axis (for r_inner < r < r_far);
    the inner disk (r < r_inner) and everything beyond r_far are kept. This sets the hard stops."""
    win = sector_x(phi0, phi1, r_far + 30, x0, x1, zc)
    inner = cylx(r_inner, x0, x1, 0, zc)
    far = box(x0, x1, -60, 60, -80, 200).cut(cylx(r_far, x0 - 1, x1 + 1, 0, zc))
    return shape.intersect(U(win, inner, far)).clean()


def place_shape(shape, Tm):
    """Apply a 4x4 rigid transform to a CadQuery shape (re-orthonormalised, via gp_Trsf)."""
    cq = _cq()
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf
    Rm = np.asarray(Tm[:3, :3], float)
    u, _, vt = np.linalg.svd(Rm)
    Rm = u @ vt
    t = np.asarray(Tm[:3, 3], float)
    tr = gp_Trsf()
    tr.SetValues(*[float(v) for row in np.column_stack([Rm, t]) for v in row])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, tr, True).Shape())


# ------------------------------------------------------------------------------------------------
# Parts (finger-local frame, q = 0)
# ------------------------------------------------------------------------------------------------


def stop_windows(lo, hi):
    """Angular windows (deg) for parent and child near a joint with servo range lo..hi."""
    m = STOP_MARGIN
    parent = (90 + hi / 2 + m, 270 + lo / 2 - m)
    child = (-90 - lo / 2 + m, 90 - hi / 2 - m)
    return parent, child


@functools.cache
def local_parts(name: str) -> dict:
    """All solids of one finger in the finger-local frame at q = 0 (cached; build() places copies)."""
    g = geom(name)
    hw, hh, hk, xd, xdp, xt = g["hw"], g["hh"], g["hk"], g["xd"], g["xdp"], g["xt"]
    L1, zp, zd, zt = g["L1"], g["z_pip"], g["z_dip"], g["z_tip"]
    r_fl, r_boss, r_band = g["r_fl"], g["r_boss"], g["r_band"]
    parts = {}

    # ---------------- yoke (MCP clevis + abduction drum + sheath clamps) ----------------
    mcp_par, mcp_chi = stop_windows(*LIMITS["mcp_flex"])
    base = fbox(-hk, hk, YOKE_Y[0], YOKE_Y[1], BASE_BOT, SOCKET_TOP, 1.5)
    cheeks = []
    for s in (-1, 1):
        x0, x1 = sorted((s * (xd + CLR), s * hk))
        arm = box(x0, x1, -hh, YOKE_Y[1], SOCKET_TOP - 0.01, 0.0)
        lug = sector_x(mcp_par[1] - 42, mcp_par[1], 10.0, x0, x1, 0.0, r_in=r_boss - 0.5)
        cheeks.append(U(cylx(r_boss, x0, x1), arm, lug))
    yoke = U(base, *cheeks)
    yoke = keep_near_joint(yoke, 0.0, mcp_par[0], mcp_par[1], r_boss, r_far=40.0)
    disc = cyl(r_fl, (0, ABD_DISC_Y[0], -ABD_OFFSET), (0, ABD_DISC_Y[1], -ABD_OFFSET))
    yoke = U(yoke, disc)
    cut = []
    for role, y in ABD_GROOVE_Y.items():  # abd grooves (full circle) + post pocket at the top
        ring = cyl(r_fl + 1, (0, y - GROOVE_W / 2, -ABD_OFFSET), (0, y + GROOVE_W / 2, -ABD_OFFSET)).cut(
            cyl(GROOVE_FLOOR, (0, y - 1, -ABD_OFFSET), (0, y + 1, -ABD_OFFSET)))
        cut += [ring, cyl(POCKET_R, (0, y - 0.8, -ABD_OFFSET + POST_R_C), (0, y + 0.8, -ABD_OFFSET + POST_R_C))]
    cut.append(cyl(POST_D / 2, (0, ABD_DISC_Y[0] + 0.4, -ABD_OFFSET + POST_R_C), (0, ABD_DISC_Y[1] + 1, -ABD_OFFSET + POST_R_C)))
    cut.append(cyl(ABD_PIN_D / 2, (0, YOKE_Y[0] - 1, -ABD_OFFSET), (0, -6.0, -ABD_OFFSET)))  # palmar pin, blind
    cut.append(cyl(ABD_PIN_D / 2, (0, ABD_PIN_Y["dorsal"][0], -ABD_OFFSET), (0, ABD_DISC_Y[1] + 1, -ABD_OFFSET)))
    cut.append(cylx(PIN_D / 2, -hk - 1, -xd - 0.01))  # MCP stub axle seats (press)
    cut.append(cylx(PIN_D / 2, xd + 0.01, hk + 1))
    for x, y in ((-X_MCP_GROOVE, -R), (X_MCP_GROOVE, R), (-X_PIP, 0.0), (X_PIP, 0.0)):
        cut.append(cyl(SHEATH_BORE / 2, (x, y, BASE_BOT - 1), (x, y, TUBE_STOP)))
        cut.append(cyl(BARE_BORE / 2, (x, y, TUBE_STOP - 0.01), (x, y, SOCKET_TOP + 1)))
    parts["yoke"] = yoke.cut(U(*cut)).clean()

    # ---------------- proximal phalanx (MCP drum, shaft, PIP clevis) ----------------
    pip_par, pip_chi = stop_windows(*LIMITS["pip"])
    drum = cylx(r_fl, -xd, xd)
    neck = box(-xd, xd, -hh, hh, 0, r_band + 0.5)
    shaft = fbox(-hw, hw, -hh, hh, r_band, zp - r_boss, 3.0)
    heels = []
    for s in (-1, 1):
        x0, x1 = sorted((s * (xd + CLR), s * hk))
        heels.append(sector_x(mcp_chi[0], 0.0, 10.0, x0, x1, 0.0, r_in=r_band))
        heels.append(box(x0, x1, -hh, hh, 0.0, r_band + 0.01).cut(cylx(r_band, x0 - 1, x1 + 1)))
    pcheeks = []
    for s in (-1, 1):
        x0, x1 = sorted((s * (xdp + CLR), s * hw))
        arm = box(x0, x1, -hh, hh, zp - r_band - 3, zp)
        lug = sector_x(pip_par[1] - 42, pip_par[1], 10.0, x0, x1, zp, r_in=r_boss - 0.5)
        ch = U(cylx(r_boss, x0, x1, 0, zp), arm, lug)
        pcheeks.append(keep_near_joint(ch, zp, pip_par[0], pip_par[1], r_boss))
    prox = U(drum, neck, shaft, *heels, *pcheeks)
    prox = keep_near_joint(prox, 0.0, mcp_chi[0], mcp_chi[1], r_fl + 0.01)
    prox = keep_near_joint(prox, zp, pip_par[0], pip_par[1], r_boss + 0.01)  # PIP flexion stop at ~96 deg
    # 1 mm groove slot reaches r 12: at 90 deg the flex strand runs through the root of the shaft
    cut = [ring_groove_x(s * X_MCP_GROOVE, GROOVE_FLOOR, 12.0) for s in (-1, 1)]
    cut += [cylx(PIN_D / 2 + CLR / 2, s * STUB_X0, s * (xd + 1)) for s in (-1, 1)]  # stub axles, running fit
    # pip strands: open slot under the eyelets (strand fan), eyelet bosses kept, eyelet + strand bores
    slot = box(-SLOT_HALF, SLOT_HALF, -12, 12, -12, g["z0"] + 1.6)  # eyelet sticks out 0.6 below the plastic
    bosses, bores = [], []
    for side, x in ((-1, -X_PIP), (1, X_PIP)):
        P0, u, T = eyelet_axis(g, side)
        p0 = np.array([x, *P0])
        uu = np.array([0, *u])
        bosses.append(cyl(1.25, p0 + uu * 0.6, p0 + uu * 2.0))
        bores.append(cyl(EYELET_OD / 2, p0 - uu * 0.5, p0 + uu * (EYELET_L + 0.01)))
        bores.append(cyl(BARE_BORE / 2, p0 + uu * (EYELET_L - 0.2), np.array([x, *T]) + uu * 2))
    cut.append(slot.cut(U(*bosses)))
    cut += bores
    pa = np.array([POST_R_C * math.cos(math.radians(ANCHOR["mcp_flex"])), POST_R_C * math.sin(math.radians(ANCHOR["mcp_flex"]))])
    cut.append(cylx(POST_D / 2, -(X_MCP_GROOVE + 1.5), X_MCP_GROOVE + 1.5, pa[0], pa[1]))
    for s in (-1, 1):
        cut.append(cylx(POCKET_R, s * X_MCP_GROOVE - 0.8, s * X_MCP_GROOVE + 0.8, pa[0], pa[1]))
    cut.append(cylx(PIN_D / 2, -hw - 1, hw + 1, 0, zp))  # PIP axle seat (press in the cheeks)
    cut.append(box(-hw - 1, -hw + RECESS, -20, 20, zp - 12, zp + 12))  # bar recess
    A, B = link_pins_local(g)
    cut.append(cylx(LINK_PIN_D / 2, -hw + RECESS - 0.01, -hw + RECESS + 1.5, A[0], A[1]))  # blind, 1.5 deep
    parts["proximal"] = prox.cut(U(*cut)).clean()

    # ---------------- middle phalanx (PIP drum, body, DIP tongue) ----------------
    dip_lo, dip_hi = float(dip_of(g, LIMITS["pip"][0])), float(dip_of(g, LIMITS["pip"][1]))
    g["dip_range"] = (dip_lo, dip_hi)
    dip_par, dip_chi = stop_windows(dip_lo, dip_hi)
    xn = min(xdp, xt)
    mdrum = cylx(r_fl, -xdp, xdp, 0, zp)
    mneck = box(-xdp, xdp, -hh, hh, zp, zp + r_band + 0.5)
    mnarrow = box(-xn, xn, -hh, hh, zp, zd)
    tongue = cylx(r_fl, -xt, xt, 0, zd)
    mbody = fbox(-hw + RECESS, hw, -hh, hh, zp + r_band, zd - r_band, 2.5)
    mheels = []
    for s in (-1, 1):
        x0, x1 = sorted((s * (xdp + CLR), s * (hw if s > 0 else hw - RECESS)))
        mheels.append(sector_x(pip_chi[0], 0.0, 10.0, x0, x1, zp, r_in=r_band))
    mid = U(mdrum, mneck, mnarrow, tongue, mbody, *mheels)
    mid = keep_near_joint(mid, zp, pip_chi[0], pip_chi[1], r_fl + 0.01)
    mid = keep_near_joint(mid, zd, dip_par[0], dip_par[1], r_fl + 0.01)
    cut = [ring_groove_x(s * X_PIP, GROOVE_FLOOR, 11.0, zp) for s in (-1, 1)]  # slot to r 11 (strand at 95 deg)
    cut.append(cylx(PIN_D / 2 + CLR / 2, -xdp - 1, xdp + 1, 0, zp))  # PIP axle, running fit
    cut.append(cylx(PIN_D / 2 + CLR / 2, -xt - 1, xt + 1, 0, zd))  # DIP axle, running fit
    pm = np.array([POST_R_C * math.cos(math.radians(ANCHOR["pip"])), zp + POST_R_C * math.sin(math.radians(ANCHOR["pip"]))])
    cut.append(cylx(POST_D / 2, -(X_PIP + 1.5), X_PIP + 1.5, pm[0], pm[1]))
    cut.append(cylx(POCKET_R, -X_PIP - 0.8, X_PIP + 0.8, pm[0], pm[1]))
    parts["middle"] = mid.cut(U(*cut)).clean()

    # ---------------- distal phalanx (DIP clevis + fingertip) ----------------
    hwt = 0.92 * hw
    dcheeks = []
    for s in (-1, 1):
        x0, x1 = sorted((s * (xt + CLR), s * (hw if s > 0 else hw - RECESS)))
        arm = box(x0, x1, -hh + PAD_T, hh, zd, zd + r_band + 1)
        dcheeks.append(U(cylx(r_boss, x0, x1, 0, zd), arm))
    r_tip = min(hwt - 0.6, 6.0)  # plan-view corner radius of the fingertip
    tip_end = zt - r_tip
    tbody = fbox(-hwt, hwt, -hh + PAD_T, hh, zd + r_boss, zt, r_tip, "|Y and >Z")
    try:  # round the dorsal edges (nail side)
        tbody = _cq().Workplane().add(tbody).edges(">Y").fillet(2.0).val()
    except Exception:
        pass
    dist = U(*dcheeks, tbody, box(-xt - CLR - 0.01, xt + CLR + 0.01, -hh + PAD_T, hh, zd + r_boss, zd + r_band + 1))
    dist = keep_near_joint(dist, zd, dip_chi[0], dip_chi[1], r_boss + 0.01)
    cut = [cylx(PIN_D / 2, -hw - 1, hw + 1, 0, zd)]
    cut.append(box(-hw - 1, -hw + RECESS, -20, 20, zd - 12, zd + 9))
    cut.append(cylx(LINK_PIN_D / 2, -hw + RECESS - 0.01, -hw + RECESS + 1.8, B[0], B[1]))
    parts["distal"] = dist.cut(U(*cut)).clean()

    # ---------------- TPU pads ----------------
    z_pad0 = zd + 6.0
    pad = fbox(-hwt + 0.6, hwt - 0.6, -hh - 0.8, -hh + PAD_T - 0.05, z_pad0, zt - 0.6, r_tip - 0.6, "|Y and >Z")
    bulge = cylx(45.0, -hw, hw, -hh - 0.8 + 45.0, (z_pad0 + zt) / 2)  # flattened, slightly convex pad
    pad = pad.intersect(bulge).cut(cylx(r_boss + CLR, -hw - 1, hw + 1, 0, zd))  # clear the DIP cheek bosses
    try:
        pad = _cq().Workplane().add(pad).edges("<Y").fillet(1.0).val()
    except Exception:
        pass
    parts["pad_tip"] = pad.clean()
    parts["pad_proximal"] = fbox(-hw + 1.8, hw - 1.8, -hh - 1.5, -hh, r_band + 4, zp - r_boss - 3, 1.0, "|Y")
    mz0, mz1 = zp + r_band + 0.5, zd - r_band - 0.5
    if mz1 - mz0 > 3:
        parts["pad_middle"] = fbox(-hw + RECESS + 1.0, hw - 1.8, -hh - 1.2, -hh, mz0, mz1, 0.8, "|Y")

    # ---------------- steel: pins, posts, eyelets; PTFE sheath ends ----------------
    st = {}
    st["pin_mcp_l"] = cylx(PIN_D / 2, -hk, -STUB_X0 - 0.3)
    st["pin_mcp_r"] = cylx(PIN_D / 2, STUB_X0 + 0.3, hk)
    st["pin_pip"] = cylx(PIN_D / 2, -hw + RECESS, hw, 0, zp)
    st["pin_dip"] = cylx(PIN_D / 2, -hw + RECESS, hw, 0, zd)
    st["post_mcp"] = cylx(POST_D / 2, -(X_MCP_GROOVE + 1.5), X_MCP_GROOVE + 1.5, pa[0], pa[1])
    st["post_pip"] = cylx(POST_D / 2, -(X_PIP + 1.5), X_PIP + 1.5, pm[0], pm[1])
    st["post_abd"] = cyl(POST_D / 2, (0, ABD_DISC_Y[0] + 0.4, -ABD_OFFSET + POST_R_C), (0, ABD_DISC_Y[1], -ABD_OFFSET + POST_R_C))
    st["pin_abd_palmar"] = cyl(ABD_PIN_D / 2, (0, ABD_PIN_Y["palmar"][0], -ABD_OFFSET), (0, ABD_PIN_Y["palmar"][1], -ABD_OFFSET))
    st["pin_abd_dorsal"] = cyl(ABD_PIN_D / 2, (0, ABD_PIN_Y["dorsal"][0], -ABD_OFFSET), (0, ABD_PIN_Y["dorsal"][1], -ABD_OFFSET))
    st["pin_link_a"] = cylx(LINK_PIN_D / 2, -hw, -hw + RECESS + 1.5, A[0], A[1])
    st["pin_link_b"] = cylx(LINK_PIN_D / 2, -hw, -hw + RECESS + 1.8, B[0], B[1])
    for side, x, key in ((-1, -X_PIP, "eyelet_flex"), (1, X_PIP, "eyelet_ext")):
        P0, u, T = eyelet_axis(g, side)
        p0, uu = np.array([x, *P0]), np.array([0, *u])
        st[key] = cyl(EYELET_OD / 2, p0, p0 + uu * EYELET_L).cut(cyl(EYELET_ID / 2, p0 - uu, p0 + uu * (EYELET_L + 1)))
    for x, y, key in ((-X_MCP_GROOVE, -R, "ptfe_mcp_flex"), (X_MCP_GROOVE, R, "ptfe_mcp_ext"),
                      (-X_PIP, 0.0, "ptfe_pip_flex"), (X_PIP, 0.0, "ptfe_pip_ext")):
        st[key] = cyl(SHEATH_OD / 2, (x, y, BASE_BOT), (x, y, TUBE_STOP)).cut(cyl(P.SHEATH_ID / 2, (x, y, BASE_BOT - 1), (x, y, TUBE_STOP + 1)))
    parts.update(st)
    return parts


BODY_OF = {"yoke": "yoke", "proximal": "proximal", "middle": "middle", "distal": "distal", "pad_tip": "distal",
           "pad_proximal": "proximal", "pad_middle": "middle", "pin_mcp_l": "yoke", "pin_mcp_r": "yoke",
           "pin_pip": "proximal", "pin_dip": "distal", "post_mcp": "proximal", "post_pip": "middle",
           "post_abd": "yoke", "pin_abd_palmar": "palm", "pin_abd_dorsal": "palm", "pin_link_a": "proximal",
           "pin_link_b": "distal", "eyelet_flex": "proximal", "eyelet_ext": "proximal",
           "ptfe_mcp_flex": "yoke", "ptfe_mcp_ext": "yoke", "ptfe_pip_flex": "yoke", "ptfe_pip_ext": "yoke"}
MATERIAL = {"yoke": "PETG", "proximal": "PETG", "middle": "PETG", "distal": "PETG", "bar": "stainless 1.2 mm (or PETG)",
            "pad_tip": "TPU 95A", "pad_proximal": "TPU 95A", "pad_middle": "TPU 95A"}


def bar_shape(g, q, Tb):
    """The DIP bar between pin A (proximal) and pin B (distal) at pose q, in the hand frame."""
    cq = _cq()
    A, B = link_pins_local(g)
    # pins in the yoke frame (only flexion rotations, so the bar stays in a plane x = const)
    inv_y = np.linalg.inv(Tb["yoke"])
    a = xf(inv_y @ Tb["proximal"], lift(0.0, [A]))[0, 1:]
    b = xf(inv_y @ Tb["distal"], lift(0.0, [B]))[0, 1:]
    L = float(np.linalg.norm(b - a))
    ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    x0 = -g["hw"] + 0.2
    wp = (cq.Workplane("YZ", origin=(x0, 0, 0)).center(*((a + b) / 2)).slot2D(L + BAR_W, BAR_W, ang)
          .extrude(BAR_T))
    for p in (a, b):
        wp = wp.cut(cylx(LINK_PIN_D / 2 + 0.05, x0 - 1, x0 + BAR_T + 1, p[0], p[1]))
    return place_shape(wp.val(), Tb["yoke"]), L


# ------------------------------------------------------------------------------------------------
# Public API
# ------------------------------------------------------------------------------------------------


def build(name: str, q: dict | None = None, with_parts: bool = True) -> dict:
    """Finger `name` at joint angles q (deg: mcp_abd, mcp_flex, pip; dip follows the bar)."""
    g = geom(name)
    qf = full_q(g, q)
    Tb = body_transforms(g, qf)
    parts = {}
    if with_parts:
        for k, s in local_parts(name).items():
            parts[k] = place_shape(s, Tb[BODY_OF[k]])
        parts["bar"], bar_len = bar_shape(g, qf, Tb)
    dip_rng = (float(dip_of(g, LIMITS["pip"][0])), float(dip_of(g, LIMITS["pip"][1])))
    ax = lambda M, p, d: dict(point=tuple(np.round(xf(M, [p])[0], 4)), axis=tuple(np.round(M[:3, :3] @ np.array(d, float), 6)))  # noqa: E731
    joints = {
        "mcp_abd": {**ax(Tb["palm"], (0, 0, -ABD_OFFSET), (0, 1, 0)), "range": LIMITS["mcp_abd"]},
        "mcp_flex": {**ax(Tb["yoke"], (0, 0, 0), (1, 0, 0)), "range": LIMITS["mcp_flex"]},
        "pip": {**ax(Tb["proximal"], (0, 0, g["z_pip"]), (1, 0, 0)), "range": LIMITS["pip"]},
        "dip": {**ax(Tb["middle"], (0, 0, g["z_dip"]), (1, 0, 0)), "range": dip_rng, "coupled_to": "pip",
                "angle_deg": round(qf["dip"], 4)},
    }
    st = strands(g, qf)
    return {"parts": parts, "joints": joints, "strands": st, "interface": interface(g, qf, Tb, st), "q": qf}


def interface(g, q, Tb, st):
    """What the palm must provide / leave clear (hand frame)."""
    yoke_corners = np.array([[x, y, z] for x in (-g["hk"], g["hk"]) for y in (YOKE_Y[0], ABD_DISC_Y[1])
                             for z in (BASE_BOT, -ABD_OFFSET - g["r_fl"], g["r_boss"])])
    env = []
    for a in np.linspace(*LIMITS["mcp_abd"], 13):
        Ta = body_transforms(g, full_q(g, {"mcp_abd": a}))["yoke"]
        env.append(xf(Ta, yoke_corners))
    env = np.vstack(env)
    exits = []
    for s in st:
        pts = np.asarray(s["points"])
        if s["joint"] == "mcp_abd":
            p, d, body = pts[-1], np.array([0, 0, -1.0]), "palm"
        else:
            p, d, body = pts[-1], Tb["yoke"][:3, :3] @ np.array([0, 0, -1.0]), "yoke (moves with mcp_abd)"
        exits.append(dict(strand=f"{g['name']}_{s['joint']}_{s['role']}", point=np.round(p, 3).tolist(),
                          direction=np.round(d, 4).tolist(), sheathed=True, sheath_clamped_in=body))
    m = g["mcp"]
    return {
        "abd_pivot": {"axis_point": np.round(m + [0, 0, -ABD_OFFSET], 3).tolist(), "axis_dir": [0, 1, 0],
                      "pin_d": ABD_PIN_D,
                      "palm_holds_y": [[m[1] + ABD_PIN_Y["palmar"][0], m[1] + YOKE_Y[0] - CLR],
                                       [m[1] + ABD_DISC_Y[1] + CLR, m[1] + ABD_PIN_Y["dorsal"][1]]],
                      "note": "two coaxial 3 mm dowels; the palm holds the outer ends; yoke runs on them"},
        "yoke_envelope": {"min": np.round(env.min(0) - CLR, 2).tolist(), "max": np.round(env.max(0) + CLR, 2).tolist(),
                          "note": "box swept by the yoke + abd disc over mcp_abd range (+0.25 mm); leave clear"},
        "strand_exits": exits,
        "sheath_free_span_mm": PALM_GUIDE_DZ,
        "palm_sheath_guides": [dict(strand=f"{g['name']}_{s['joint']}_{s['role']}", direction=[0, 0, -1],
                                    point=np.round(xf(Tb["palm"], [[s["x_local"], (-R if s["role"] == "flex" else R) if s["joint"] == "mcp_flex" else 0.0, BASE_BOT - PALM_GUIDE_DZ]])[0], 3).tolist())
                               for s in st if s["joint"] != "mcp_abd"],
    }


# ------------------------------------------------------------------------------------------------
# Checks
# ------------------------------------------------------------------------------------------------


def decoupling(name):
    """Length change of each strand (mm) when each OTHER joint sweeps its range (others at min/0/max),
    plus own-joint linearity: mm/deg and flex+ext sum."""
    g = geom(name)
    grid = {j: np.linspace(*LIMITS[j], 9) for j in ("mcp_abd", "mcp_flex", "pip")}
    base_vals = {j: (LIMITS[j][0], 0.0, LIMITS[j][1]) for j in grid}
    rows = []
    keyed = lambda q: {(s["joint"], s["role"]): s for s in strands(g, q)}  # noqa: E731
    for (jn, role) in [(j, r) for j in ("mcp_abd", "mcp_flex", "pip") for r in ("flex", "ext")]:
        row = {"strand": f"{jn}.{role}"}
        for other in grid:
            if other == jn:
                continue
            worst = 0.0
            rest = [k for k in grid if k != other]
            for b0 in base_vals[rest[0]]:
                for b1 in base_vals[rest[1]]:
                    q = {rest[0]: b0, rest[1]: b1}
                    Ls = [keyed({**q, other: v})[(jn, role)]["length"] for v in grid[other]]
                    worst = max(worst, max(Ls) - min(Ls))
            row[other] = worst
        # own joint
        qs = np.linspace(*LIMITS[jn], 21)
        Lf = np.array([keyed({jn: v})[(jn, "flex")]["length"] for v in qs])
        Le = np.array([keyed({jn: v})[(jn, "ext")]["length"] for v in qs])
        mine = Lf if role == "flex" else Le
        slope = np.polyfit(qs, mine, 1)[0]
        row["own_mm_per_deg"] = slope
        row["own_nonlin_mm"] = float(np.abs(mine - np.polyval(np.polyfit(qs, mine, 1), qs)).max())
        row["loop_sum_var_mm"] = float((Lf + Le).max() - (Lf + Le).min())
        rows.append(row)
    return rows


def polyline_3d_length_check(name):
    """Compare the strand lengths from the analytic model with the 3D polylines (sanity)."""
    g = geom(name)
    errs = []
    for q in ({"mcp_flex": 60, "pip": 70, "mcp_abd": 10}, {}):
        for s in strands(g, q):
            pts = np.asarray(s["points"])
            if s["joint"] == "mcp_abd":
                L3 = plength(pts)
            else:  # drop the sheath piece (mouth -> yoke bottom)
                L3 = plength(pts[:-1])
            errs.append(abs(L3 - s["length"]))
    return max(errs)


def collision_check(name, quick=False):
    from tools.checks import interference
    combos = [(a, m, p) for a in (0.0,) for m in LIMITS["mcp_flex"][:1] + (40.0,) + LIMITS["mcp_flex"][1:]
              for p in LIMITS["pip"][:1] + (45.0,) + LIMITS["pip"][1:]]
    if quick:
        combos = [(0, -10, -5), (0, 90, 95), (0, 40, 45)]
    combos.append((LIMITS["mcp_abd"][1], 90.0, 95.0))
    res = {}
    for a, m, p in combos:
        parts = build(name, {"mcp_abd": a, "mcp_flex": m, "pip": p})["parts"]
        hits = interference(parts, threshold=0.05)
        res[(a, m, p)] = hits
    return res


def strands_in_solids(name, q, step=0.3):
    """Sample each bare strand and report points that fall inside a solid (should be none)."""
    import trimesh

    from tools.render import tessellate  # needs rtree (uv --with rtree)
    b = build(name, q)
    meshes = {}
    for k, s in b["parts"].items():
        if k.startswith("ptfe"):
            continue
        v, f = tessellate(s, 0.05, 0.2)
        meshes[k] = trimesh.Trimesh(v, f, process=True)
    bad = []
    for s in b["strands"]:
        pts = np.asarray(s["points"])
        if s["joint"] != "mcp_abd":
            pts = pts[:-1]
        samp = []
        for p0, p1 in zip(pts[:-1], pts[1:]):
            n = max(1, int(np.linalg.norm(p1 - p0) / step))
            samp += [p0 + (p1 - p0) * t for t in np.linspace(0, 1, n, endpoint=False)]
        samp = np.array(samp[3:])  # skip the first points: they sit on the anchor post
        for k, m in meshes.items():
            inside = m.contains(samp)
            if inside.any():
                bad.append((f"{s['joint']}.{s['role']}", k, int(inside.sum()), len(samp)))
    return bad


def sheath_check(name):
    """Free PTFE span from the palm guide (fixed) to the yoke bottom (moves with mcp_abd): min bend radius."""
    from tools.transform import check_sheath
    g = geom(name)
    worst = math.inf
    for a in LIMITS["mcp_abd"]:
        Tb = body_transforms(g, full_q(g, {"mcp_abd": a}))
        for x, y in ((-X_MCP_GROOVE, -R), (X_MCP_GROOVE, R), (-X_PIP, 0.0), (X_PIP, 0.0)):
            top = xf(Tb["yoke"], [[x, y, BASE_BOT]])[0]
            dtop = Tb["yoke"][:3, :3] @ np.array([0, 0, -1.0])
            bot = xf(Tb["palm"], [[x, y, BASE_BOT - PALM_GUIDE_DZ]])[0]
            h = PALM_GUIDE_DZ / 2.5
            ts = np.linspace(0, 1, 60)[:, None]
            p0, p1, p2, p3 = top, top + dtop * h, bot + np.array([0, 0, h]), bot
            curve = (1 - ts) ** 3 * p0 + 3 * (1 - ts) ** 2 * ts * p1 + 3 * (1 - ts) * ts ** 2 * p2 + ts ** 3 * p3
            worst = min(worst, check_sheath(curve, min_r=P.SHEATH_MIN_BEND_R, smooth=False)["min_radius"])
    return worst


def link_report(name):
    g = geom(name)
    lk = g["link"]
    pips = np.linspace(*LIMITS["pip"], 41)
    d = dip_of(g, pips)
    return dict(bar_mm=lk["bar_mm"], max_ratio_error_deg=float(np.abs(d - 0.75 * pips).max()),
                min_transmission_deg=lk["min_transmission_deg"], axis_clear_mm=lk["axis_clear_mm"],
                dip_range=(float(d[0]), float(d[-1])), pin_a=lk["pin_a"], pin_b=lk["pin_b"])


def force_estimate(name):
    """Static fingertip force from the servo through each drum (SCS0009 / HLS3606M stall, 90 % efficiency)."""
    g = geom(name)
    tip = g["L1"] + g["L2"] + g["L3"] - 4.0  # pad centre lever from MCP, finger straight
    res = {}
    for servo, tau in (("SCS0009", 0.226), ("HLS3606M", 0.59)):
        F = tau / (P.R_SPOOL / 1000)  # strand force
        res[servo] = dict(strand_N=round(F, 1), mcp_Nm=round(F * R / 1000 * 0.9, 3),
                          tip_N_straight=round(F * R / 1000 * 0.9 / (tip / 1000), 1))
    return res


# ------------------------------------------------------------------------------------------------
# Rendering
# ------------------------------------------------------------------------------------------------

COLORS = {"yoke": "#6b8fb3", "proximal": "#e8d3a8", "middle": "#e3c48d", "distal": "#e8d3a8", "pad_tip": "#d9534f",
          "pad_proximal": "#d9534f", "pad_middle": "#d9534f", "bar": "#444444"}


def strand_solids(st, r=0.25):
    cq = _cq()
    out = {}
    for s in st:
        pts = np.asarray(s["points"])
        segs = []
        for p0, p1 in zip(pts[:-1], pts[1:]):
            if np.linalg.norm(p1 - p0) > 0.05:
                segs.append(cyl(r, p0, p1))
        out[f"strand_{s['joint']}_{s['role']}"] = cq.Compound.makeCompound(segs)
    return out


def colours(parts):
    c = {}
    for k in parts:
        if k in COLORS:
            c[k] = COLORS[k]
        elif k.startswith("strand"):
            c[k] = "#1f77b4" if k.endswith("flex") else "#2ca02c"
        elif k.startswith("ptfe"):
            c[k] = "#f2f2f2"
        else:
            c[k] = "#9aa0a6"
    return c


def render_finger(name, q, out_png, views=("iso", "front", "side", "top"), cut_x=None, title=None, strands_on=True):
    from tools.render import render
    b = build(name, q)
    parts = dict(b["parts"])
    if strands_on:
        parts.update(strand_solids(b["strands"]))
    if cut_x is not None:  # cutaway: keep x < cut_x (hand frame)
        g = geom(name)
        half = box(-200, g["mcp"][0] + cut_x, -200, 200, -200, 400)
        parts = {k: (v.intersect(half) if hasattr(v, "intersect") else v) for k, v in parts.items()}
        parts = {k: v for k, v in parts.items() if v.Volume() > 1e-6 or k.startswith("strand")}
    return render(parts, out_png, views=views, colors=colours(parts), title=title or f"{name} {q}")


# ------------------------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------------------------


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--finger", action="append")
    ap.add_argument("--quick", action="store_true", help="3 collision poses instead of 10")
    ap.add_argument("--no-export", action="store_true")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--no-collide", action="store_true")
    a = ap.parse_args(argv)
    names = a.finger or list(FINGER_NAMES)
    OUT.mkdir(parents=True, exist_ok=True)
    summary = {}
    for n in names:
        t0 = time.time()
        g = geom(n)
        s = summary[n] = {}
        print(f"\n=== {n}: L {g['L1']}/{g['L2']}/{g['L3']}  W {g['W']} H {g['H']}  knuckle {2 * g['hk']:.1f}  "
              f"eyelet z0 {g['z0']:+.2f}")
        from tools.checks import is_valid
        lp = local_parts(n)
        s["valid"] = {k: is_valid(v)["ok"] for k, v in lp.items()}
        bad = [k for k, ok in s["valid"].items() if not ok]
        print(f"  solids valid+watertight: {len(lp) - len(bad)}/{len(lp)}" + (f"  BAD: {bad}" if bad else ""))
        s["volumes_cm3"] = {k: round(v.Volume() / 1000, 3) for k, v in lp.items() if k in MATERIAL}
        rows = decoupling(n)
        s["decoupling"] = rows
        print("  strand        d/abd    d/mcp    d/pip   (mm, worst over other joints' min/0/max) | own mm/deg  nonlin  loop-sum var")
        for r in rows:
            cells = "  ".join(f"{r.get(j, float('nan')):7.3f}" if j in r else "    own" for j in ("mcp_abd", "mcp_flex", "pip"))
            print(f"  {r['strand']:13s} {cells}  | {r['own_mm_per_deg']:+.4f}   {r['own_nonlin_mm']:.4f}   {r['loop_sum_var_mm']:.4f}")
        s["poly_check_mm"] = polyline_3d_length_check(n)
        s["link"] = link_report(n)
        s["sheath_min_bend_mm"] = sheath_check(n)
        s["force"] = force_estimate(n)
        print(f"  DIP bar {s['link']['bar_mm']:.2f} mm, max |dip-0.75 pip| {s['link']['max_ratio_error_deg']:.2f} deg, "
              f"transmission >= {s['link']['min_transmission_deg']:.0f} deg, dip range {s['link']['dip_range'][0]:.1f}..{s['link']['dip_range'][1]:.1f}")
        print(f"  sheath free span min bend radius over abd range: {s['sheath_min_bend_mm']:.1f} mm (need >= {P.SHEATH_MIN_BEND_R})")
        print(f"  3D polyline vs analytic strand length: {s['poly_check_mm']:.4f} mm")
        print(f"  force: {s['force']}")
        if not a.no_collide:
            col = collision_check(n, quick=a.quick)
            s["collisions"] = {str(k): v for k, v in col.items()}
            for k, v in col.items():
                print(f"  collide abd/mcp/pip {k}: " + ("clear" if not v else "; ".join(f"{x}x{y}={vol}" for x, y, vol in v)))
            sis = []
            for q in ({"mcp_flex": -10, "pip": -5}, {"mcp_flex": 45, "pip": 50}, {"mcp_flex": 90, "pip": 95}):
                sis += [(str(q), *b) for b in strands_in_solids(n, q)]
            s["strand_in_solid"] = sis
            print("  strands inside solids: " + ("none" if not sis else str(sis)))
        if not a.no_export:
            from tools.export import export_all
            b = build(n)
            export_all(b["parts"], OUT / n, name=f"finger_{n}", colors=colours(b["parts"]), glb=False)
            meta = {k: v for k, v in b.items() if k != "parts"}
            (OUT / n / f"finger_{n}.json").write_text(json.dumps(meta, indent=1, default=float))
        if not a.no_render:
            render_finger(n, {}, OUT / f"{n}_q0.png", title=f"{n} q=0")
            render_finger(n, {"mcp_flex": 60, "pip": 70, "mcp_abd": 10}, OUT / f"{n}_curl.png",
                          title=f"{n} abd 10 mcp 60 pip 70")
            if n == "index":
                render_finger(n, {}, OUT / f"{n}_cut.png", views=("side", (20, 60), "front", (10, 120)),
                              cut_x=1.4, title="index cutaway x<1.4 (pip flex strand plane)")
                render_finger(n, {"mcp_flex": 90, "pip": 95}, OUT / f"{n}_fist_cut.png", views=("side", (20, 60)),
                              cut_x=1.4, title="index fist, cutaway")
        print(f"  ({time.time() - t0:.0f} s)")
    (OUT / "summary.json").write_text(json.dumps(summary, indent=1, default=str))
    if not a.no_render and len(names) == 4:
        render_all({}, OUT / "fingers_q0.png", "4 fingers q=0 (hand frame)")
        render_all({"mcp_flex": 70, "pip": 80}, OUT / "fingers_curl.png", "4 fingers mcp 70 pip 80")
        render_all({"mcp_flex": 90, "pip": 95}, OUT / "fingers_fist.png", "4 fingers fist (mcp 90, pip 95)")


def render_all(q, out_png, title):
    from tools.render import render
    parts = {}
    for n in FINGER_NAMES:
        b = build(n, q)
        for k, v in b["parts"].items():
            if k in MATERIAL:
                parts[f"{n}_{k}"] = v
        for k, v in strand_solids(b["strands"]).items():
            parts[f"{n}_{k}"] = v
    cols = {k: colours({k.split("_", 1)[1]: 0})[k.split("_", 1)[1]] for k in parts}
    return render(parts, out_png, views=("iso", "front", "side", "top"), colors=cols, title=title, legend=False)


if __name__ == "__main__":
    main()
