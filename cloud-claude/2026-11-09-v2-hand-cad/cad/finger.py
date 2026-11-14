"""Tendra V2 concept hand: one parametric finger module for index, middle, ring and little (study 2026-11-09).

Frame: the hand frame of params.py (origin = wrist centre, +Z to the fingers, -Y palm side, +X thumb side).
Each finger is first built in a *finger-local* frame = the hand frame shifted so the MCP flexion axis
passes through the origin (params.FINGERS[f]["mcp"]); the finger points along +Z at q = 0.

Joints (positive = closing):
    mcp_abd  axis +Y through (0, 0, -ABD_OFFSET)    + = toward the thumb (+X), every finger
    mcp_flex axis +X through (0, 0, 0)              + = toward the palm (-Y)
    pip      axis +X through (0, 0, L1)
    dip      axis +X through (0, 0, L1 + L2)        no servo: dip = f(pip) through a rigid bar (~0.75 x pip)

Tendon drive (owner's track, 2026-11-13): ONE groove ("track") per servo joint, radius params.R_DRUM = 7 mm.
One strand comes up from the palm-side sheath, wraps over the drum and goes down to the back-side sheath; at
the "top" of the track ring (ANCHOR angle, the middle of the wrap over the joint range) both strand ends go
into a TIE_HOLE_D tie hole in the groove's side wall and are knotted together in a knot well behind it.
The knot cannot pass the hole, so pulling either end turns the joint either way.
    mcp_abd   disc about Y on the dorsal end of the yoke, track at y = ABD_TRACK_Y; strands to palm sheaths.
    mcp_flex  proximal phalanx base, track at x = X_MCP_TRACK (flex and ext strand in the same groove).
    pip       middle phalanx base, track at x = X_PIP_TRACK = 0.
The mcp_flex and pip strands arrive in PTFE sheaths that are clamped in the yoke (the abduction joint
is crossed inside the sheaths = Bowden, exact). The pip strands cross the MCP flexion axis bare, through two
stainless eyelets at x = -/+X_PIP whose rims sit just proximal of the axis (crossing_length(): < 0.2 mm), then
converge (2.4 deg fleet angle) to the single PIP track at x = 0.

Joint bearings (owner, 2026-11-13): every flexion hinge runs on 2 x MR84ZZ (4x8x3), one in each side lug of
the fork (MCP: yoke, PIP: proximal, DIP: distal). Seats are pressed in from the outer face against an inner
shoulder; the child carries a Ø5 hub that touches only the inner races (zero axial play). Axles: MCP = two
4 x 6 mm steel stubs (the centre stays free for the crossing pip strands), PIP and DIP = one 4 mm steel
through-pin, pressed in the child. mcp_abd: 2 x MR63ZZ (3x6x2.5) in the palm's knuckle bosses, on two 3 mm
steel stubs pressed in the yoke (MR84 does not fit beside the mcp flex sheath; see report 06).
Bearings are separate parts "bearing_<joint>_<side>"; build(..., bearings=False) leaves them out.

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
X_MCP_TRACK = -3.3  # the single mcp_flex track (flex + ext strand); between the crossing slot and the -x lug
X_PIP = 1.4  # pip strands where they cross the MCP (eyelets, yoke sheaths): flex -x, ext +x
X_PIP_TRACK = 0.0  # the single PIP track on the middle phalanx
SLOT_HALF = 2.2  # open slot in the MCP drum where the pip strands cross the axis
# --- bearings (params.BEARING_JOINT = MR84ZZ 4x8x3) ---
BRG = P.BEARING_JOINT
BRG_W = BRG["w"]  # 3.0
LIP_T = 0.8  # axial thickness of the seat's inner shoulder (stops the outer race)
LUG_T = BRG_W + LIP_T  # 3.8: side lug = seat + shoulder
SEAT_D = P.BEARING_SEAT_D  # 7.95 light press
LIP_BORE = SEAT_D - 2 * P.BEARING_SHOULDER  # 6.75: only the outer race rests on the shoulder
HUB_R = 2.5  # child hub (touches the inner race only; MR84 inner ring land ~ dia 5.2)
AXLE_D = P.JOINT_AXLE_D  # 4.0 steel
STUB_L = 6.0  # MCP stubs: 4 x 6 mm dowels, flush with the yoke's outer face
BRG_ABD = {"name": "MR63ZZ", "id": 3.0, "od": 6.0, "w": 2.5}  # abd: in the palm bosses
ABD_SEAT_D = 6.0 - 0.05
ABD_BRG_Y = {"palmar": (-12.6, -10.1), "dorsal": (15.55, 18.05)}  # finger-local y of the abd bearings
ABD_LIP = {"palmar": (-10.1, -9.25), "dorsal": (14.85, 15.55)}  # palm shoulders (bore 4.8) next to them
# --- track anchors / tie holes ---
TIE_D = P.TIE_HOLE_D  # 1.7, lined with a 16G tube (P.TIE_LINER)
TIE_R = R - LINE_R - TIE_D / 2 + 0.1  # 6.07 (1.7 hole): tie hole centre; its top breaks 0.1 through the groove floor
WELL_R = 0.8  # knot well radius (well = radial blind hole from r WELL_R0 out through the surface)
WELL_R0 = 4.5
WELL_WALL = 0.6  # groove side wall between the track and the knot well (the knot bears on it)
SOCKET_TOP = -8.5  # yoke top face (sheath mouths); clears the MCP drum flange by >= 0.25
TUBE_STOP = -9.5  # PTFE sheath ends here (2.2 bore below, 1.2 bore above)
BASE_BOT = -16.0  # yoke bottom (sheaths leave the finger here, pointing -Z)
YOKE_Y = (-9.0, 9.4)
ABD_DISC_Y = (9.0, 14.6)
ABD_TRACK_Y = 11.8  # the single abd track (flex strand on the +x side, ext on -x)
ABD_GROOVE_Y = {"flex": ABD_TRACK_Y, "ext": ABD_TRACK_Y}
ABD_PIN_D = P.PIN_D_BIG  # 3 mm steel stubs, palmar + dorsal, pressed in the yoke, running in MR63s in the palm
ABD_PIN_Y = {"palmar": (-12.5, -6.25), "dorsal": (8.0, 18.0)}  # palm holds y < -9.25 and y > 14.85
PALM_GUIDE_DZ = 22.0  # sheath free span below the yoke before the palm's guide channel
ABD_SOCKET_DZ = 12.0  # abd strands: sheath mouths in the palm this far below the abd axis
# Track anchor (tie hole) angle, deg, in the (y, z) plane from +y toward +z. MCP: 50 = 40 deg dorsal of the
# finger axis at q = 0, i.e. straight up (+z of the yoke) at mid-range q = 40: 40 deg of wrap on each side at the
# range ends. PIP: computed in geom() from its tangent points (same rule: equal wrap margins).
ANCHOR = {"mcp_flex": 50.0, "pip": None}  # pip: g["anchor_pip"] (45.0 on all four fingers, 50.9 deg margins)
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
    # PIP / DIP forks: -x lug outer face = the bar recess face (pl), lugs LUG_T thick, child between them
    pl = -hw + RECESS
    g = dict(name=name, mcp=np.array(f["mcp"], float), L1=L1, L2=L2, L3=L3, W=W, H=H, hw=hw, hh=hh, hk=hk,
             r_fl=r_fl, r_boss=r_fl + CLR, r_band=r_fl + 2 * CLR,
             xd=hk - LUG_T - CLR,  # MCP drum half width (yoke lugs LUG_T thick)
             pl=pl, cl=pl + LUG_T, cr=hw - LUG_T,  # PIP/DIP lug faces: -x outer, -x inner, +x inner
             xl=pl + LUG_T + CLR, xr=hw - LUG_T - CLR,  # PIP drum / DIP tongue from xl to xr (asymmetric)
             z_pip=L1, z_dip=L1 + L2, z_tip=L1 + L2 + L3)
    # DIP bar (V1 maths, lengths from params). Pins A and B must clear the Ø8 bearings in the -x lugs:
    # r >= 5.0 (pin A web to the seat 0.27 mm); bar centreline within +-6.5 of the phalanx axis.
    DL.AXIS_CLEAR, DL.PIN_R_MIN, DL.PIN_R_MAX, DL.HALF_HEIGHT = 2.0, 5.0, 6.5, 6.5
    g["link"] = DL.design(float(L2))
    # eyelet entrance: optimise its height so the pip strands' length change over the MCP range is smallest
    best = None
    for z0 in np.arange(-1.0, 0.81, 0.025):
        g["z0"] = float(z0)
        e = max(v.max() - v.min() for v in crossing_table(g).values())  # peak-to-peak
        if best is None or e < best[0]:
            best = (e, float(z0))
    g["z0"] = best[1]
    # PIP track anchor: equal wrap margins at both range ends (flex strand at pip max, ext at pip min)
    C = np.array([0.0, L1])
    th = {}
    for side, key in ((-1, "flex"), (1, "ext")):
        T = eyelet_axis(g, side)[2]
        th[key] = math.degrees(math.atan2(*(T - C)[::-1]))
    th["flex"] %= 360
    th["ext"] = (th["ext"] + 180) % 360 - 180
    lo, hi = LIMITS["pip"]
    g["anchor_pip"] = (th["flex"] + th["ext"] - hi - lo) / 2
    g["wrap_margin_pip"] = th["flex"] - g["anchor_pip"] - hi
    return g


def anchor_deg(g, joint):
    return ANCHOR["mcp_flex"] if joint == "mcp_flex" else g["anchor_pip"]


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
    for role, x, th_t in (("flex", X_MCP_TRACK, 180.0), ("ext", X_MCP_TRACK, 0.0)):
        th_a = ANCHOR["mcp_flex"] + q["mcp_flex"]  # anchor (tie hole) angle in space-aligned coordinates
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
        th_a = g["anchor_pip"] + q["pip"]  # anchor (tie hole) angle seen from the proximal frame
        if role == "flex":
            th_t = th_t % 360
            wrap = th_t - th_a
        else:
            th_t = (th_t + 180) % 360 - 180
            wrap = th_a - th_t
        # the eyelet keeps the strand in its plane x = +-X_PIP; at the eyelet exit E it turns 2.4 deg toward the
        # track at x = X_PIP_TRACK. E -> T is fixed in the proximal frame, so the x step only adds a constant.
        Q, Tq = np.asarray(cpts[-2], float), np.asarray(cpts[-1], float)
        Ex = P0 + u * EYELET_L
        t = float(np.clip(np.dot(Ex - Q, u) / np.dot(Tq - Q, u), 0.0, 1.0))
        E = Q + (Tq - Q) * t
        dET = float(np.linalg.norm(Tq - E))
        L = Lc + math.hypot(dET, x - X_PIP_TRACK) - dET + R * math.radians(wrap)
        arc_p = _arc(C, R, th_a, th_t)
        cross_p = np.array(cpts[::-1])  # from T back to the mouth (proximal frame)
        pts = np.vstack([xf(Tb["proximal"], lift(X_PIP_TRACK, arc_p)), xf(Tb["proximal"], lift(x, [E])),
                         xf(Tb["proximal"], lift(x, cross_p[1:])), xf(Tb["yoke"], [[x, 0.0, BASE_BOT]])])
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


def bearing_solid(p0, p1, spec=None):
    """Ball bearing at its real size between face centres p0 -> p1: outer race, inner race and a shield
    (recessed 0.2 mm) in one solid with the ID bore, like a ZZ bearing seen from outside."""
    spec = spec or BRG
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    u = (p1 - p0) / np.linalg.norm(p1 - p0)
    ro, ri = spec["od"] / 2, spec["id"] / 2
    t = 0.3 * (ro - ri)  # race wall thickness
    def ann(a, b, r_out, r_in):
        return cyl(r_out, a, b).cut(cyl(r_in, a - u, b + u))
    outer = ann(p0, p1, ro, ro - t)
    inner = ann(p0, p1, ri + t, ri)
    shield = ann(p0 + u * 0.2, p1 - u * 0.2, ro - t + 0.01, ri + t - 0.01)
    return U(outer, inner, shield)


def seat_cut_x(xo, inward, zc):
    """Bearing seat (dia SEAT_D, BRG_W deep, open on the outer face xo) + the shoulder bore behind it."""
    s = inward
    return U(cylx(SEAT_D / 2, xo - s * 1.0, xo + s * BRG_W, 0, zc),
             cylx(LIP_BORE / 2, xo + s * (BRG_W - 0.01), xo + s * (LUG_T + 0.5), 0, zc))


def tie_cuts(x_track, phi, zc, x_dir=1.0, r_out=14.0):
    """Owner's tie point on a track (axis along X through (0, zc)): a TIE_D hole through the groove's side wall at
    angle phi (deg from +y toward +z) and radius TIE_R (its top flush with the groove floor), into a knot well
    (radial blind hole, WELL_R, from WELL_R0 out through the part's surface) on the x_dir side of the groove."""
    c, s = math.cos(math.radians(phi)), math.sin(math.radians(phi))
    xw = x_track + x_dir * (GROOVE_W / 2 + WELL_WALL + WELL_R)
    hole = cylx(TIE_D / 2, x_track, xw, TIE_R * c, zc + TIE_R * s)
    well = cyl(WELL_R, (xw, WELL_R0 * c, zc + WELL_R0 * s), (xw, r_out * c, zc + r_out * s))
    return [hole, well]


def liner_tube(p0, p1):
    """16G stainless tie-hole liner (OD/ID/len from params.TIE_LINER) from p0 along p1 - p0 (its direction)."""
    L = P.TIE_LINER
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    u = (p1 - p0) / np.linalg.norm(p1 - p0)
    return cyl(L["od"] / 2, p0, p0 + u * L["len"]).cut(cyl(L["id"] / 2, p0 - u, p0 + u * (L["len"] + 1)))


def tie_liner_x(x_track, phi, zc, x_dir=1.0):
    """Liner of a tie_cuts() hole: starts at the groove's side wall, 0.5 mm of it ends inside the knot well."""
    c, s = math.cos(math.radians(phi)), math.sin(math.radians(phi))
    x0 = x_track + x_dir * GROOVE_W / 2
    return liner_tube((x0, TIE_R * c, zc + TIE_R * s), (x0 + x_dir, TIE_R * c, zc + TIE_R * s))


@functools.cache
def local_parts(name: str) -> dict:
    """All solids of one finger in the finger-local frame at q = 0 (cached; build() places copies)."""
    g = geom(name)
    hw, hh, hk, xd = g["hw"], g["hh"], g["hk"], g["xd"]
    pl, cl, cr, xl, xr = g["pl"], g["cl"], g["cr"], g["xl"], g["xr"]
    L1, zp, zd, zt = g["L1"], g["z_pip"], g["z_dip"], g["z_tip"]
    r_fl, r_boss, r_band = g["r_fl"], g["r_boss"], g["r_band"]
    parts = {}

    # ---------------- yoke (MCP fork with 2 x MR84 + abduction drum + sheath clamps) ----------------
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
    # abd: ONE track (full circle) + tie hole at the top of the disc into a knot notch on its dorsal end
    y = ABD_TRACK_Y
    cut.append(cyl(r_fl + 1, (0, y - GROOVE_W / 2, -ABD_OFFSET), (0, y + GROOVE_W / 2, -ABD_OFFSET)).cut(
        cyl(GROOVE_FLOOR, (0, y - 1, -ABD_OFFSET), (0, y + 1, -ABD_OFFSET))))
    yw0 = y + GROOVE_W / 2 + WELL_WALL
    cut.append(cyl(TIE_D / 2, (0, y, -ABD_OFFSET + TIE_R), (0, yw0 + 0.8, -ABD_OFFSET + TIE_R)))
    cut.append(box(-WELL_R - 0.1, WELL_R + 0.1, yw0, ABD_DISC_Y[1] + 1, -ABD_OFFSET + WELL_R0, -ABD_OFFSET + r_fl + 1))
    cut.append(cyl(ABD_PIN_D / 2, (0, YOKE_Y[0] - 1, -ABD_OFFSET), (0, -6.0, -ABD_OFFSET)))  # palmar stub (press)
    cut.append(cyl(ABD_PIN_D / 2, (0, ABD_PIN_Y["dorsal"][0], -ABD_OFFSET), (0, ABD_DISC_Y[1] + 1, -ABD_OFFSET)))
    for s in (-1, 1):  # MCP bearing seats, pressed in from the outer faces
        cut.append(seat_cut_x(s * hk, -s, 0.0))
    for x, yy in ((X_MCP_TRACK, -R), (X_MCP_TRACK, R), (-X_PIP, 0.0), (X_PIP, 0.0)):
        cut.append(cyl(SHEATH_BORE / 2, (x, yy, BASE_BOT - 1), (x, yy, TUBE_STOP)))
        cut.append(cyl(BARE_BORE / 2, (x, yy, TUBE_STOP - 0.01), (x, yy, SOCKET_TOP + 1)))
    parts["yoke"] = yoke.cut(U(*cut)).clean()

    # ---------------- proximal phalanx (MCP drum + hubs, shaft, PIP fork with 2 x MR84) ----------------
    pip_par, pip_chi = stop_windows(*LIMITS["pip"])
    drum = cylx(r_fl, -xd, xd)
    hubs = [cylx(HUB_R, s * (xd - 0.01), s * (hk - BRG_W - 0.05)) for s in (-1, 1)]  # touch the inner races
    neck = box(-xd, xd, -hh, hh, 0, r_band + 0.5)
    shaft = fbox(-hw, hw, -hh, hh, r_band, zp - r_boss, 3.0)
    heels = []
    for s in (-1, 1):
        x0, x1 = sorted((s * (xd + CLR), s * hk))
        heels.append(sector_x(mcp_chi[0], 0.0, 10.0, x0, x1, 0.0, r_in=r_band))
        heels.append(box(x0, x1, -hh, hh, 0.0, r_band + 0.01).cut(cylx(r_band, x0 - 1, x1 + 1)))
    pcheeks = []
    for x0, x1 in ((-hw, cl), (cr, hw)):
        arm = box(x0, x1, -hh, hh, zp - r_band - 3, zp)
        lug = sector_x(pip_par[1] - 42, pip_par[1], 10.0, x0, x1, zp, r_in=r_boss - 0.5)
        ch = U(cylx(r_boss, x0, x1, 0, zp), arm, lug)
        pcheeks.append(keep_near_joint(ch, zp, pip_par[0], pip_par[1], r_boss))
    prox = U(drum, *hubs, neck, shaft, *heels, *pcheeks)
    prox = keep_near_joint(prox, 0.0, mcp_chi[0], mcp_chi[1], r_fl + 0.01)
    prox = keep_near_joint(prox, zp, pip_par[0], pip_par[1], r_boss + 0.01)  # PIP flexion stop at ~96 deg
    # the single MCP track; 1 mm groove slot reaches r 12 (at 90 deg the flex strand runs through the shaft root)
    cut = [ring_groove_x(X_MCP_TRACK, GROOVE_FLOOR, 12.0)]
    cut += tie_cuts(X_MCP_TRACK, ANCHOR["mcp_flex"], 0.0, x_dir=+1.0)
    x_stub = max(hk - STUB_L - 0.3, SLOT_HALF + 0.3)  # blind end of the stub hole (keeps 0.3 to the slot)
    cut += [cylx(AXLE_D / 2, s * x_stub, s * (hk + 0.1)) for s in (-1, 1)]  # 4 mm stubs (press)
    # pip strands: open slot under the eyelets (strand fan), eyelet bosses kept, eyelet + strand bores
    slot = box(-SLOT_HALF, SLOT_HALF, -12, 12, -12, g["z0"] + 1.6)  # eyelet sticks out 0.6 below the plastic
    bosses, bores = [], []
    for side, x in ((-1, -X_PIP), (1, X_PIP)):
        P0, u, T = eyelet_axis(g, side)
        p0 = np.array([x, *P0])
        uu = np.array([0, *u])
        bosses.append(cyl(1.25, p0 + uu * 0.6, p0 + uu * 2.0))
        bores.append(cyl(EYELET_OD / 2, p0 - uu * 0.5, p0 + uu * (EYELET_L + 0.01)))
        pe = p0 + uu * (EYELET_L - 0.2)
        T3 = np.array([X_PIP_TRACK, *T])
        dv = (T3 - pe) / np.linalg.norm(T3 - pe)
        bores.append(cyl(BARE_BORE / 2, pe, T3 + dv * 2))  # straight, converging to the PIP track (2.4 deg)
    cut.append(slot.cut(U(*bosses)))
    cut += bores
    cut.append(seat_cut_x(pl, +1, zp))  # PIP bearing seats: -x from the bar recess face, +x from the side face
    cut.append(seat_cut_x(hw, -1, zp))
    cut.append(box(-hw - 1, -hw + RECESS, -20, 20, zp - 12, zp + 12))  # bar recess
    A, B = link_pins_local(g)
    cut.append(cylx(LINK_PIN_D / 2, pl - 0.01, pl + 1.5, A[0], A[1]))  # blind, 1.5 deep (0.27 web to the seat)
    parts["proximal"] = prox.cut(U(*cut)).clean()

    # ---------------- middle phalanx (PIP drum + hubs, body, DIP tongue + hubs) ----------------
    dip_lo, dip_hi = float(dip_of(g, LIMITS["pip"][0])), float(dip_of(g, LIMITS["pip"][1]))
    g["dip_range"] = (dip_lo, dip_hi)
    dip_par, dip_chi = stop_windows(dip_lo, dip_hi)
    mdrum = cylx(r_fl, xl, xr, 0, zp)
    mneck = box(xl, xr, -hh, hh, zp, zp + r_band + 0.5)
    mnarrow = box(xl, xr, -hh, hh, zp, zd)
    tongue = cylx(r_fl, xl, xr, 0, zd)
    mhubs = []
    for zc in (zp, zd):
        mhubs += [cylx(HUB_R, xl + 0.01, pl + BRG_W + 0.05, 0, zc), cylx(HUB_R, xr - 0.01, hw - BRG_W - 0.05, 0, zc)]
    mbody = fbox(-hw + RECESS, hw, -hh, hh, zp + r_band, zd - r_band, 2.5)
    mheels = []
    for x0, x1 in ((pl, cl), (cr, hw)):
        mheels.append(sector_x(pip_chi[0], 0.0, 10.0, x0, x1, zp, r_in=r_band))
    mid = U(mdrum, mneck, mnarrow, tongue, mbody, *mheels, *mhubs)
    mid = keep_near_joint(mid, zp, pip_chi[0], pip_chi[1], r_fl + 0.01)
    mid = keep_near_joint(mid, zd, dip_par[0], dip_par[1], r_fl + 0.01)
    cut = [ring_groove_x(X_PIP_TRACK, GROOVE_FLOOR, 11.0, zp)]  # the single PIP track, slot to r 11 (strand at 95)
    cut += tie_cuts(X_PIP_TRACK, g["anchor_pip"], zp, x_dir=+1.0)
    cut.append(cylx(AXLE_D / 2, pl - 1, hw + 1, 0, zp))  # PIP through-pin (press in the middle)
    cut.append(cylx(AXLE_D / 2, pl - 1, hw + 1, 0, zd))  # DIP through-pin (press in the middle)
    parts["middle"] = mid.cut(U(*cut)).clean()

    # ---------------- distal phalanx (DIP fork with 2 x MR84 + fingertip) ----------------
    hwt = 0.92 * hw
    dcheeks = []
    for x0, x1 in ((pl, cl), (cr, hw)):
        arm = box(x0, x1, -hh + PAD_T, hh, zd, zd + r_band + 1)
        dcheeks.append(U(cylx(r_boss, x0, x1, 0, zd), arm))
    r_tip = min(hwt - 0.6, 6.0)  # plan-view corner radius of the fingertip
    tbody = fbox(-hwt, hwt, -hh + PAD_T, hh, zd + r_boss, zt, r_tip, "|Y and >Z")
    try:  # round the dorsal edges (nail side)
        tbody = _cq().Workplane().add(tbody).edges(">Y").fillet(2.0).val()
    except Exception:
        pass
    dist = U(*dcheeks, tbody, box(cl - 0.01, cr + 0.01, -hh + PAD_T, hh, zd + r_boss, zd + r_band + 1))
    dist = keep_near_joint(dist, zd, dip_chi[0], dip_chi[1], r_boss + 0.01)
    cut = [seat_cut_x(pl, +1, zd), seat_cut_x(hw, -1, zd)]
    cut.append(box(-hw - 1, -hw + RECESS, -20, 20, zd - 12, zd + 9))
    cut.append(cylx(LINK_PIN_D / 2, pl - 0.01, pl + 1.8, B[0], B[1]))
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
    # starts r_band + 6 (was + 4): at MCP 90 deg its proximal corner otherwise grazes the palmar abd bearing / palm boss
    parts["pad_proximal"] = fbox(-hw + 1.8, hw - 1.8, -hh - 1.5, -hh, r_band + 6, zp - r_boss - 3, 1.0, "|Y")
    mz0, mz1 = zp + r_band + 0.5, zd - r_band - 0.5
    if mz1 - mz0 > 3:
        parts["pad_middle"] = fbox(-hw + RECESS + 1.0, hw - 1.8, -hh - 1.2, -hh, mz0, mz1, 0.8, "|Y")

    # ---------------- steel: axles, pins, eyelets; PTFE sheath ends; bearings ----------------
    st = {}
    st["pin_mcp_l"] = cylx(AXLE_D / 2, -hk, -(hk - STUB_L))  # 4 x 6 stub, pressed in the proximal
    st["pin_mcp_r"] = cylx(AXLE_D / 2, hk - STUB_L, hk)
    st["pin_pip"] = cylx(AXLE_D / 2, pl, hw, 0, zp)  # 4 mm through-pin, pressed in the middle
    st["pin_dip"] = cylx(AXLE_D / 2, pl, hw, 0, zd)
    st["pin_abd_palmar"] = cyl(ABD_PIN_D / 2, (0, ABD_PIN_Y["palmar"][0], -ABD_OFFSET), (0, ABD_PIN_Y["palmar"][1], -ABD_OFFSET))
    st["pin_abd_dorsal"] = cyl(ABD_PIN_D / 2, (0, ABD_PIN_Y["dorsal"][0], -ABD_OFFSET), (0, ABD_PIN_Y["dorsal"][1], -ABD_OFFSET))
    st["pin_link_a"] = cylx(LINK_PIN_D / 2, -hw, pl + 1.5, A[0], A[1])
    st["pin_link_b"] = cylx(LINK_PIN_D / 2, -hw, pl + 1.8, B[0], B[1])
    for side, x, key in ((-1, -X_PIP, "eyelet_flex"), (1, X_PIP, "eyelet_ext")):
        P0, u, T = eyelet_axis(g, side)
        p0, uu = np.array([x, *P0]), np.array([0, *u])
        st[key] = cyl(EYELET_OD / 2, p0, p0 + uu * EYELET_L).cut(cyl(EYELET_ID / 2, p0 - uu, p0 + uu * (EYELET_L + 1)))
    for x, y, key in ((X_MCP_TRACK, -R, "ptfe_mcp_flex"), (X_MCP_TRACK, R, "ptfe_mcp_ext"),
                      (-X_PIP, 0.0, "ptfe_pip_flex"), (X_PIP, 0.0, "ptfe_pip_ext")):
        st[key] = cyl(SHEATH_OD / 2, (x, y, BASE_BOT), (x, y, TUBE_STOP)).cut(cyl(P.SHEATH_ID / 2, (x, y, BASE_BOT - 1), (x, y, TUBE_STOP + 1)))
    st["tie_liner_mcp"] = tie_liner_x(X_MCP_TRACK, ANCHOR["mcp_flex"], 0.0)  # in the proximal
    st["tie_liner_pip"] = tie_liner_x(X_PIP_TRACK, g["anchor_pip"], zp)  # in the middle
    ya = ABD_TRACK_Y + GROOVE_W / 2
    st["tie_liner_abd"] = liner_tube((0, ya, -ABD_OFFSET + TIE_R), (0, ya + 1, -ABD_OFFSET + TIE_R))  # in the yoke
    parts.update(st)
    parts.update(bearing_parts(g))
    return parts


def bearing_parts(g):
    """The joint bearings as separate parts (finger-local frame, q = 0). l = -x side, r = +x side."""
    hk, hw, pl, zp, zd = g["hk"], g["hw"], g["pl"], g["z_pip"], g["z_dip"]
    b = {}
    b["bearing_mcp_l"] = bearing_solid((-hk, 0, 0), (-hk + BRG_W, 0, 0))
    b["bearing_mcp_r"] = bearing_solid((hk, 0, 0), (hk - BRG_W, 0, 0))
    for j, zc in (("pip", zp), ("dip", zd)):
        b[f"bearing_{j}_l"] = bearing_solid((pl, 0, zc), (pl + BRG_W, 0, zc))
        b[f"bearing_{j}_r"] = bearing_solid((hw, 0, zc), (hw - BRG_W, 0, zc))
    for side, (y0, y1) in ABD_BRG_Y.items():  # MR63 in the palm's knuckle bosses (body: palm)
        b[f"bearing_abd_{side}"] = bearing_solid((0, y0, -ABD_OFFSET), (0, y1, -ABD_OFFSET), BRG_ABD)
    return b


BEARINGS = {  # part -> (type, body, joint)
    "bearing_mcp_l": ("MR84ZZ", "yoke", "mcp_flex"), "bearing_mcp_r": ("MR84ZZ", "yoke", "mcp_flex"),
    "bearing_pip_l": ("MR84ZZ", "proximal", "pip"), "bearing_pip_r": ("MR84ZZ", "proximal", "pip"),
    "bearing_dip_l": ("MR84ZZ", "distal", "dip"), "bearing_dip_r": ("MR84ZZ", "distal", "dip"),
    "bearing_abd_palmar": ("MR63ZZ", "palm", "mcp_abd"), "bearing_abd_dorsal": ("MR63ZZ", "palm", "mcp_abd"),
}
BODY_OF = {"yoke": "yoke", "proximal": "proximal", "middle": "middle", "distal": "distal", "pad_tip": "distal",
           "pad_proximal": "proximal", "pad_middle": "middle", "pin_mcp_l": "proximal", "pin_mcp_r": "proximal",
           "pin_pip": "middle", "pin_dip": "middle", "tie_liner_mcp": "proximal", "tie_liner_pip": "middle", "tie_liner_abd": "yoke", "pin_abd_palmar": "yoke", "pin_abd_dorsal": "yoke",
           "pin_link_a": "proximal", "pin_link_b": "distal", "eyelet_flex": "proximal", "eyelet_ext": "proximal",
           "ptfe_mcp_flex": "yoke", "ptfe_mcp_ext": "yoke", "ptfe_pip_flex": "yoke", "ptfe_pip_ext": "yoke",
           **{k: v[1] for k, v in BEARINGS.items()}}
# press fits modelled at their print size (bearing seats SEAT_D < 8): ignore these pairs in interference checks
PRESS_FIT_PAIRS = [("tie_liner_mcp", "proximal"), ("tie_liner_pip", "middle"), ("tie_liner_abd", "yoke"),
                   ("bearing_mcp_*", "yoke"), ("bearing_pip_*", "proximal"), ("bearing_dip_*", "distal")]
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


def build(name: str, q: dict | None = None, with_parts: bool = True, bearings: bool = True) -> dict:
    """Finger `name` at joint angles q (deg: mcp_abd, mcp_flex, pip; dip follows the bar).
    bearings=False leaves the 8 bearing_* parts out (to see the seats)."""
    g = geom(name)
    qf = full_q(g, q)
    Tb = body_transforms(g, qf)
    parts = {}
    if with_parts:
        for k, s in local_parts(name).items():
            if not bearings and k in BEARINGS:
                continue
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
                      "note": "two coaxial 3 mm steel stubs pressed in the yoke; their outer ends run in two MR63ZZ "
                              "bearings that the palm holds (bearing_seats)"},
        "abd_bearing_seats": [
            {"side": side, "bearing": BRG_ABD["name"], "id_od_w": [BRG_ABD["id"], BRG_ABD["od"], BRG_ABD["w"]],
             "axis_point": np.round(m + [0, 0, -ABD_OFFSET], 3).tolist(), "axis_dir": [0, 1, 0],
             "seat_d": ABD_SEAT_D, "seat_y": [round(float(m[1] + v), 3) for v in ABD_BRG_Y[side]],
             "shoulder_bore_d": 4.8, "shoulder_y": [round(float(m[1] + v), 3) for v in ABD_LIP[side]],
             "open_to": "palm surface (palmar face / back), press in from outside; seat bore continues to the surface",
             "boss_d_min": BRG_ABD["od"] + 2 * P.WALL_MIN}
            for side in ("palmar", "dorsal")],
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
        hits = interference(parts, threshold=0.05, ignore=PRESS_FIT_PAIRS)
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
        for x, y in ((X_MCP_TRACK, -R), (X_MCP_TRACK, R), (-X_PIP, 0.0), (X_PIP, 0.0)):
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
        elif k.startswith("bearing"):
            c[k] = "#d4a017"
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


def bearing_fits(name):
    """Walls and widths around the bearings (mm), per joint."""
    g = geom(name)
    hk, hw, pl = g["hk"], g["hw"], g["pl"]
    A, B = link_pins_local(g)
    rA = float(np.hypot(A[0], A[1] - g["z_pip"]))
    rB = float(np.hypot(B[0], B[1] - g["z_dip"]))
    return {
        "mcp": {"knuckle_w": 2 * hk, "lug_t": LUG_T, "drum_w": 2 * g["xd"], "lug_radial_wall": round(g["r_boss"] - SEAT_D / 2, 2),
                "track_flanges": [round(-X_MCP_TRACK - GROOVE_W / 2 - SLOT_HALF, 2), round(g["xd"] + X_MCP_TRACK - GROOVE_W / 2, 2)],
                "stub_in_child": round(g["xd"] - max(hk - STUB_L, SLOT_HALF + 0.3), 2)},
        "pip": {"finger_w": 2 * hw, "lug_t": LUG_T, "drum_x": [round(g["xl"], 2), round(g["xr"], 2)],
                "lug_radial_wall": round(g["r_boss"] - SEAT_D / 2, 2), "pin_a_web_to_seat": round(rA - LINK_PIN_D / 2 - SEAT_D / 2, 2)},
        "dip": {"tongue_x": [round(g["xl"], 2), round(g["xr"], 2)], "pin_b_web_to_seat": round(rB - LINK_PIN_D / 2 - SEAT_D / 2, 2)},
    }


def render_bearings(name, out_dir=OUT):
    """Bearing / track views of one finger: axis-plane section, track cuts with the tie holes, exploded view,
    and the same finger with and without its bearings."""
    from tools.render import render
    g = geom(name)
    m = g["mcp"]
    b = build(name, {})
    parts = dict(b["parts"])
    keep = {k: v for k, v in parts.items() if not k.startswith("ptfe")}
    # 1) section through all three flexion axes (plane y = 0 of the finger), seen from the back
    half = box(-200, 200, -200, m[1], -200, 400)
    sec = {k: v.intersect(half) for k, v in keep.items()}
    sec = {k: v for k, v in sec.items() if v.Volume() > 1e-6}
    render(sec, out_dir / f"{name}_bearing_section.png", views=("back", (35, 60)), colors=colours(sec),
           title=f"{name}: section through the MCP/PIP/DIP axes (gold = MR84ZZ, grey = 4 mm steel axles)")
    # 2) cuts through the MCP track and the PIP track (side view): tie hole + knot well at the anchor
    st = strand_solids(b["strands"])
    xw = GROOVE_W / 2 + WELL_WALL + WELL_R  # knot well centre, +x of its track
    for tag, xc in (("mcp", X_MCP_TRACK), ("pip", X_PIP_TRACK), ("mcp_knot", X_MCP_TRACK + xw),
                    ("pip_knot", X_PIP_TRACK + xw)):
        cutb = box(-200, m[0] + xc + 0.05, -200, 200, -200, 400)  # +0.05: off the eyelet / well axis planes
        pp = {k: v.intersect(cutb) for k, v in keep.items() if BODY_OF.get(k) in ("yoke", "proximal", "middle")}
        pp = {k: v for k, v in pp.items() if v.Volume() > 1e-6}
        pp.update({k: v for k, v in st.items() if tag.split("_")[0] in k})
        what = "knot well + tie hole" if "knot" in tag else "track"
        render(pp, out_dir / f"{name}_{tag}_cut.png", views=("side", (15, 50)), colors=colours(pp), legend=False,
               title=f"{name}: cut at the {tag.split('_')[0]} {what} (x = {xc:+.1f})")
    # 3) exploded: bearings 7 mm and axles 14 mm out along their own axes (finger-local x / y)
    ex = {}
    for k, v in keep.items():
        d = np.zeros(3)
        if k in BEARINGS or k.startswith("pin_mcp") or k in ("pin_pip", "pin_dip") or k.startswith("pin_abd"):
            c = np.array(v.Center().toTuple()) - m
            if k.startswith("bearing_abd") or k.startswith("pin_abd"):
                d = np.array([0.0, 7.0 * np.sign(c[1]), 0.0])
            elif k in ("pin_pip", "pin_dip"):
                d = np.array([-16.0, 0.0, 0.0])
            else:
                d = np.array([7.0 * np.sign(c[0]) * (2.0 if k.startswith("pin") else 1.0), 0.0, 0.0])
        M = np.eye(4)
        M[:3, 3] = d
        ex[k] = place_shape(v, M) if d.any() else v
    render(ex, out_dir / f"{name}_exploded.png", views=("iso", "front"), colors=colours(ex), legend=False,
           title=f"{name}: bearings and axles pulled out (MR84ZZ x 6, MR63ZZ x 2 for the palm)")
    # 4) with / without bearings
    nob = {k: v for k, v in keep.items() if k not in BEARINGS}
    render(nob, out_dir / f"{name}_no_bearings.png", views=("iso", "left"), colors=colours(nob), legend=False,
           title=f"{name}: bearings removed (build(..., bearings=False)) - empty seats")
    render(keep, out_dir / f"{name}_with_bearings.png", views=("iso", "left"), colors=colours(keep), legend=False,
           title=f"{name}: with its bearings")


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
        s["tracks"] = {"mcp_flex": {"x_local": X_MCP_TRACK, "anchor_deg_from_+y": ANCHOR["mcp_flex"],
                                    "wrap_margin_deg": ANCHOR["mcp_flex"] + LIMITS["mcp_flex"][0]},
                       "pip": {"x_local": X_PIP_TRACK, "anchor_deg_from_+y": round(g["anchor_pip"], 2),
                               "wrap_margin_deg": round(g["wrap_margin_pip"], 2)},
                       "mcp_abd": {"y_local": ABD_TRACK_Y, "anchor": "top of the disc", "wrap_margin_deg": 75.0},
                       "tie_hole_d": TIE_D, "tie_hole_r": TIE_R, "knot_well": {"d": 2 * WELL_R, "wall": WELL_WALL}}
        s["bearings"] = bearing_fits(n)
        print(f"  tracks: mcp x {X_MCP_TRACK:+.1f} anchor {ANCHOR['mcp_flex']:.0f} deg, pip x {X_PIP_TRACK:+.1f} anchor "
              f"{g['anchor_pip']:.1f} deg (wrap margin {g['wrap_margin_pip']:.1f}); abd y {ABD_TRACK_Y}")
        print(f"  bearings: {s['bearings']}")
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
            render_bearings(n)
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
