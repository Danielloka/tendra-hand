"""07 · Tendra V2.1 bench index finger (study 2026-11-14, agent 07).

One index finger (abd yoke, proximal, middle, distal, DIP bar) on a bench base block that bolts to a 2020
extrusion or a flat plate. The three servo joints' strands (abd, MCP, PIP; 6 strands) leave the base bottom
straight down toward where the servos (or hanging weights) go.

Run (from the study folder):
    uv run --no-project --with cadquery --with numpy --with trimesh --with scipy --with rtree python cad/07_finger.py
    [--quick]  fewer interference poses, no renders
Writes out/07/: <part>.stl + <part>.step per printed part, bench_finger_q0.step / .glb (full assembly at q = 0),
bom.csv, checks.json, strands.json, renders bench_finger_*.png.

Frame (finger-local, mm): MCP flexion axis = X through the origin, finger along +Z, -Y = palm side,
+X = thumb side. abd axis = Y through (0, 0, -13). PIP axis X at z = 42, DIP axis X at z = 66, tip z = 89.
Positive = closing (flexion toward -Y, abduction toward +X). Design choices and numbers: reports/07-bench-finger.md
and out/07/budget.txt (cad/07_budget.py; this script reads out/07/budget.json).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
STUDY = HERE.parent
OUT = STUDY / "out" / "07"
OUT.mkdir(parents=True, exist_ok=True)
V2CAD = STUDY.parent / "2026-11-09-v2-hand-cad" / "cad"
sys.path.insert(0, str(V2CAD))
sys.path.insert(0, str(STUDY.parents[1] / "hardware" / "cad"))
import dip_linkage as DL  # noqa: E402

BUD = json.loads((OUT / "budget.json").read_text())
D2R = math.pi / 180

# ================================================================================================ parameters
L1, L2, L3 = 42.0, 24.0, 23.0
ZP, ZD, ZT = L1, L1 + L2, L1 + L2 + L3
Z_ABD = -13.0  # abd axis 13 mm below the MCP (11.5 in V2: the palmar abd support then hits the proximal at MCP 90)
LINE_R = 0.235  # PE#8, d 0.47
R = BUD["R_DRUM"]  # 7.235 line centre on a 14 x 0.5 brass ring
RING_RO, RING_RI = 7.0, 6.5
FL_R = 7.5  # flange: 0.5 proud of the ring
R_I = BUD["R_IDLER"]  # 4.235 on the MR84 idler
RD = 1.5 + LINE_R  # line centre on a Ø3 dowel
X_MCP = 6.4  # MCP lanes: flex -6.4 (-x lug), ext +6.4 (+x lug)
X_PIP_I = 1.0  # pip lanes on the idler: flex -1.0, ext +1.0
X_RING_P = 0.8  # PIP ring centre; lanes flex -0.2, ext +1.8 (1.2 deg fleet from the idler lanes)
Y_ABD = 11.2  # abd ring centre (y); lanes 10.2 (flex, +x side) / 12.2 (ext, -x side)
RANGE = {"abd": (-15.0, 15.0), "mcp": (-10.0, 90.0), "pip": (-5.0, 95.0)}
SWEEP_PIP_MAX = 100.0  # clearance cuts are swept to 100 deg (checked pose), the servo stops at 95
CLR = 0.3
# widths (taper 18 -> 14): half widths at MCP / PIP / DIP / tip
HW_K, HW_P, HW_D, HW_T = 9.0, 8.05, 7.5, 7.0
HH_P0, HH_P1 = 8.0, 7.5  # proximal half height at MCP / PIP
HH_M1 = 7.0  # middle half height at the DIP
X_BARFACE = -6.85  # -x faces are recessed to here where the DIP bar runs (bar x -7.85..-7.05)
BAR_X0, BAR_X1 = -7.85, -7.05
# bearings
MR84 = dict(id=4.0, od=8.0, w=3.0)
MR63 = dict(id=3.0, od=6.0, w=2.5)
SEAT84, SEAT63 = 7.95, 5.95
# MCP x-stations
MCP_LUG = (4.4, 9.0)  # |x| of each proximal lug
MCP_SEAT = (4.4, 7.4)
MCP_RING = (5.6, 7.2)
TONGUE = 4.2
IDLER_SLOT = 1.7
AXLE = dict(od=4.0, id=3.0, x0=-7.4, x1=7.2)
SLEEVE = dict(od=5.0, id=4.0, x=(1.5, 4.4))  # brass sleeves in the tongue cheeks, also the race spacers
# PIP x-stations
PIP_LUGN = (-6.85, -2.65)  # -x lug: outer wire layer -6.85..-5.65, Ø5 bush -5.65..-2.65
PIP_BUSH = (-5.65, -2.65)
PIP_HUB = (-2.45, 4.05)
PIP_RING = (-0.7, 2.3)
PIP_LUGP = (4.25, 8.05)
PIP_SEAT = (4.25, 7.25)
PIP_PIN = (-5.65, 6.6)
# DIP x-stations
DIP_CHN = (-6.85, -2.85)
DIP_BUSHN = (-5.85, -2.85)
DIP_TONG = (-2.65, 4.3)
DIP_CHP = (4.5, 7.5)
DIP_PIN = (-5.85, 7.5)
# redirect / guide dowels (yoke) and anchors
REDIRECT = (6.3, -(R_I + RD))  # pip ext redirect dowel centre (y, z): crossing under the idler is horizontal
YD_PIPF = (R_I - RD, Z_ABD)  # pip flex dowel on the abd plane: strand on its +y side
YD_PIPE = (REDIRECT[0], Z_ABD)  # pip ext dowel on the abd plane
YD_MCPF = (-(R - RD), Z_ABD)  # MCP flex ear dowel (strand on its -y side)
YD_MCPE = ((R - RD), Z_ABD)  # MCP ext ear dowel
PHI_MCP = 50.0  # MCP anchors (deg from +y toward +z), both lugs
Z_BOT = -54.0  # base bottom = strand exit
BASE_DOWEL_Z = {"mcp_flex": -29.0, "mcp_ext": -33.0, "pip_flex": -37.0, "pip_ext": -41.0}
# base dowels are cantilevers pressed 5 mm into one side wall, ending 1 mm past their own strand (x range)
BASE_DOWEL_X = {"mcp_flex": (-16.0, -5.4), "mcp_ext": (5.4, 16.0), "pip_flex": (-16.0, 0.0), "pip_ext": (-16.0, 2.0)}
SLANT = 10.0  # deg, fixed y-bend below the abd plane so every strand presses on its abd-plane dowel
LINK = BUD["link"]

COLORS = {"base": "#8c8c8c", "yoke": "#4f86c6", "proximal": "#e08a3c", "middle": "#5bab6b", "distal": "#c9ae43",
          "skin": "#d05050", "metal": "#b8bcc4", "brass": "#c8a040", "bearing": "#6e7480", "cup": "#8e6fbf",
          "pcb": "#2e7d32", "magnet": "#555566", "bar": "#a0a8b8", "strand": "#202020"}


# ================================================================================================ helpers
def V(*a):
    return cq.Vector(*[float(x) for x in a])


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane().box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl(r, p0, p1):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    h = float(np.linalg.norm(d))
    return cq.Workplane(obj=cq.Solid.makeCylinder(float(r), h, V(*p0), V(*(d / h))))


def cylx(r, x0, x1, y=0.0, z=0.0):
    return cyl(r, (x0, y, z), (x1, y, z))


def cyly(r, y0, y1, x=0.0, z=0.0):
    return cyl(r, (x, y0, z), (x, y1, z))


def cylz(r, z0, z1, x=0.0, y=0.0):
    return cyl(r, (x, y, z0), (x, y, z1))


def prism_yz(pts, x0, x1):
    """polygon in (y, z) extruded along +X from x0 to x1."""
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline([tuple(map(float, p)) for p in pts]).close().extrude(x1 - x0)


def sector_yz(r0, r1, phi0, phi1, x0, x1, yc=0.0, zc=0.0, n=24):
    """annular sector, phi measured from +y toward +z (deg)."""
    t = np.radians(np.linspace(phi0, phi1, n))
    outer = [(yc + r1 * math.cos(a), zc + r1 * math.sin(a)) for a in t]
    inner = [(yc + r0 * math.cos(a), zc + r0 * math.sin(a)) for a in t[::-1]] if r0 > 0 else [(yc, zc)]
    return prism_yz(outer + inner, x0, x1)


def teardrop(r, p0, p1):
    """horizontal hole for printing on the side (+x up): circle + 45 deg apex toward +x. Axis must be in yz."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    h = float(np.linalg.norm(d))
    pl = cq.Plane(origin=V(*p0), xDir=V(1, 0, 0), normal=V(*(d / h)))
    pts = [(r * math.cos(a), r * math.sin(a)) for a in np.radians(np.linspace(45, 315, 25))] + [(r * math.sqrt(2), 0.0)]
    return cq.Workplane(pl).polyline(pts).close().extrude(h)


def U(*ws):
    out = None
    for w in ws:
        if w is None:
            continue
        out = w if out is None else out.union(w)
    return out


def rot_wp(w, p, d, deg):
    if abs(deg) < 1e-9:
        return w
    return w.rotate(V(*p), V(*(np.add(p, d))), float(deg))


def swept(w, p, d, angles):
    return U(*[rot_wp(w, p, d, a) for a in angles])


def angles(lo, hi, step=5.0):
    a = list(np.arange(lo, hi + 1e-9, step))
    if a[-1] < hi - 1e-9:
        a.append(hi)
    return a


def ring_part(ro, ri, x0, x1, gap_phi=None, yc=0.0, zc=0.0, axis="x"):
    if axis == "x":
        w = cylx(ro, x0, x1, yc, zc).cut(cylx(ri, x0 - 1, x1 + 1, yc, zc))
        if gap_phi:
            w = w.cut(sector_yz(ri - 0.5, ro + 0.5, gap_phi[0], gap_phi[1], x0 - 1, x1 + 1, yc, zc))
        return w
    w = cyly(ro, x0, x1, 0.0, zc).cut(cyly(ri, x0 - 1, x1 + 1, 0.0, zc))
    return w


def bearing(spec, x0, x1, y=0.0, z=0.0, axis="x"):
    """two rings (inner race, outer race) so they can belong to different bodies."""
    ri, ro = spec["id"] / 2, spec["od"] / 2
    land_i, land_o = ri + 0.6, ro - 0.6
    c = cylx if axis == "x" else (lambda r, a, b, yy=0.0, zz=0.0: cyly(r, a, b, y, zz))
    if axis == "x":
        inner = cylx(land_i, x0, x1, y, z).cut(cylx(ri, x0 - 1, x1 + 1, y, z))
        outer = cylx(ro, x0, x1, y, z).cut(cylx(land_o, x0 - 1, x1 + 1, y, z))
    else:
        inner = cyly(land_i, x0, x1, y, z).cut(cyly(ri, x0 - 1, x1 + 1, y, z))
        outer = cyly(ro, x0, x1, y, z).cut(cyly(land_o, x0 - 1, x1 + 1, y, z))
    del c
    return inner, outer


def oval_cup_parts(phi, x_c, lanes, t_lip, axis_center=(0.0, 0.0), r_top=RING_RI, n=2, depth=2.6, w_t=2.5,
                   axis="x"):
    """radial two-lane fuse cup (03 design, 1.5 mm crimp-sleeve chambers) + its pocket, at angle phi."""
    L = 2 * lanes + 1.5 + 0.6
    a = math.radians(phi)
    if axis == "x":
        nrm = (0.0, math.cos(a), math.sin(a))
        org = (x_c, axis_center[0], axis_center[1])
        xdir = (1, 0, 0)
    else:  # disc about Y: phi measured in the xz plane from +x toward +z
        nrm = (math.cos(a), 0.0, math.sin(a))
        org = (0.0, x_c, axis_center[1])
        xdir = (0, 1, 0)
    pl = cq.Plane(origin=V(*org), xDir=V(*xdir), normal=V(*nrm))

    def oval(off, l, w, h):
        return cq.Workplane(pl).workplane(offset=off).slot2D(l, w, 0).extrude(h)

    pocket = oval(r_top - depth, L + 0.2, w_t + 0.2, depth + 0.6)
    cup = oval(r_top - depth, L, w_t, depth)
    for s in (-1, 1):
        ch = cq.Workplane(pl).workplane(offset=r_top - depth - 0.1).center(s * lanes, 0).circle(0.75).extrude(depth - t_lip + 0.1)
        hl = cq.Workplane(pl).workplane(offset=r_top - t_lip - 0.1).center(s * lanes, 0).circle(0.4).extrude(t_lip + 0.2)
        cup = cup.cut(ch).cut(hl)
        pocket = pocket.union(cq.Workplane(pl).workplane(offset=r_top - 0.2).center(s * lanes, 0).circle(0.5).extrude(1.5))
    return cup, pocket


def tangential_cup(phi, x_lane, wrap_sign, t_lip=1.2, length=3.0, d_out=2.7, open_out=3.5):
    """single-lane MCP fuse cup on the tangent that continues the wrap beyond the anchor point.
    wrap_sign +1: phi decreasing beyond the anchor (flex), -1: phi increasing (ext)."""
    a = math.radians(phi)
    A = np.array([x_lane, R * math.cos(a), R * math.sin(a)])
    t = wrap_sign * np.array([0.0, math.sin(a), -math.cos(a)])  # continues the wrap direction
    rad = np.array([0.0, math.cos(a), math.sin(a)])
    p0 = A + 0.3 * t
    p1 = p0 + length * t
    cup = cyl(d_out / 2, p0, p1).cut(cyl(0.75, p0 + t_lip * t, p1 + 0.1 * t)).cut(cyl(0.4, p0 - 0.1 * t, p0 + (t_lip + 0.1) * t))
    pocket = cyl(d_out / 2 + 0.1, p0, p1 + 0.1 * t)
    pocket = pocket.union(cyl(d_out / 2 + 0.1, p0 + open_out * rad, p1 + 0.1 * t + open_out * rad))
    # hull between the two cylinders = a slot open radially outward
    q = [p0, p1 + 0.1 * t, p1 + 0.1 * t + open_out * rad, p0 + open_out * rad]
    yz = [(float(v[1]), float(v[2])) for v in q]
    slab = prism_yz(yz, x_lane - d_out / 2 - 0.1, x_lane + d_out / 2 + 0.1)
    pocket = pocket.union(slab)
    pocket = pocket.union(cyl(0.5, A - 0.2 * t, p0 + 0.5 * t))  # strand slot from the ring into the cup
    return cup, pocket


# ================================================================================================ 2D strand maths
def rot2(deg):
    c, s = math.cos(deg * D2R), math.sin(deg * D2R)
    return np.array([[c, -s], [s, c]])


def rot_about(p, c, deg):
    """rotate (y, z) point p about centre c by +deg (flexion: +deg moves +z points toward -y)."""
    return c + rot2(deg) @ (np.asarray(p, float) - c)


def tangent(c1, r1, c2, r2):
    """belt tangent from circle 1 to circle 2 with signed radii (+ ccw, - cw); points or r = 0 allowed.
    returns p1, p2. Coordinates (u, v) = (y, z) with u right, v up."""
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float)
    d = c2 - c1
    L = float(np.linalg.norm(d))
    k = (r1 - r2) / L
    base = math.atan2(d[1], d[0])
    best = None
    for sgn in (1, -1):
        a = base + sgn * math.acos(max(-1.0, min(1.0, k)))
        n = np.array([math.cos(a), math.sin(a)])
        p1, p2 = c1 + r1 * n, c2 + r2 * n
        t = np.array([-n[1], n[0]])
        if np.dot(p2 - p1, t) > 0:
            best = (p1, p2)
    return best


def arc_len(c, r, p_in, p_out):
    """arc from p_in to p_out on circle c, signed radius r (direction of travel). returns (length, angle, pts)."""
    a0 = math.atan2(p_in[1] - c[1], p_in[0] - c[0])
    a1 = math.atan2(p_out[1] - c[1], p_out[0] - c[0])
    if r > 0:
        da = (a1 - a0) % (2 * math.pi)
    else:
        da = -((a0 - a1) % (2 * math.pi))
    n = max(2, int(abs(da) / (3 * D2R)) + 2)
    ts = np.linspace(a0, a0 + da, n)
    pts = np.column_stack([c[0] + abs(r) * np.cos(ts), c[1] + abs(r) * np.sin(ts)])
    return abs(r) * abs(da), abs(da) / D2R, pts


def chain(start, elems, end):
    """start/end: points; elems: list of (centre, signed r). Belt path; returns length, wraps (deg), pts, segs."""
    nodes = [(start, 0.0)] + list(elems) + [(end, 0.0)]
    tans = [tangent(nodes[i][0], nodes[i][1], nodes[i + 1][0], nodes[i + 1][1]) for i in range(len(nodes) - 1)]
    Ls, wraps, pts, segs = 0.0, [], [np.asarray(start, float)], []
    for i in range(len(nodes) - 1):
        p1, p2 = tans[i]
        if i > 0:
            c, r = nodes[i]
            la, ang, ap = arc_len(c, r, tans[i - 1][1], p1)
            Ls += la
            wraps.append(ang)
            pts.extend(ap)
        Ls += float(np.linalg.norm(p2 - p1))
        segs.append((p1, p2))
        pts.append(p2)
    return Ls, wraps, np.array(pts), segs


def drum_from_anchor(c, r, anchor, nxt_c, nxt_r):
    """strand anchored on a drum at point `anchor`, wrapping with signed r toward the next element."""
    p1, p2 = tangent(c, r, nxt_c, nxt_r)
    la, ang, ap = arc_len(c, r, anchor, p1)
    return la, ang, ap


# ================================================================================================ kinematics
def dip_of(pip):
    a, ga = LINK["pin_a"]["r"], LINK["pin_a"]["deg"] * D2R
    b, gb = LINK["pin_b"]["r"], LINK["pin_b"]["deg"] * D2R
    grid = np.radians(np.array([0.0, float(pip)]))
    d = DL.dip_curve(L2, a, ga, b, gb, grid)
    return float(np.degrees(d[1]))


def link_pins(pip, dip, mcp=0.0):
    """pin A (proximal) and pin B (distal) in (y, z), yoke frame."""
    a, ga = LINK["pin_a"]["r"], LINK["pin_a"]["deg"] * D2R
    b, gb = LINK["pin_b"]["r"], LINK["pin_b"]["deg"] * D2R
    A0 = np.array([a * math.sin(ga), ZP + a * math.cos(ga)])
    B0 = np.array([b * math.sin(gb), ZD + b * math.cos(gb)])
    C_pip, C_dip = np.array([0.0, ZP]), np.array([0.0, ZD])
    B = rot_about(rot_about(B0, C_dip, dip), C_pip, pip)
    A, B = rot_about(A0, (0, 0), mcp), rot_about(B, (0, 0), mcp)
    return A, B, A0, B0


def mat_rot(axis, deg, p):
    a = deg * D2R
    c, s = math.cos(a), math.sin(a)
    ax = np.asarray(axis, float)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    Rm = np.eye(3) + s * K + (1 - c) * K @ K
    M = np.eye(4)
    M[:3, :3] = Rm
    M[:3, 3] = np.asarray(p) - Rm @ np.asarray(p)
    return M


def body_T(q):
    pip = q["pip"]
    dip = dip_of(pip)
    Ta = mat_rot((0, 1, 0), q["abd"], (0, 0, Z_ABD))
    Tm = Ta @ mat_rot((1, 0, 0), q["mcp"], (0, 0, 0))
    Tp = Tm @ mat_rot((1, 0, 0), pip, (0, 0, ZP))
    Td = Tp @ mat_rot((1, 0, 0), dip, (0, 0, ZD))
    return {"base": np.eye(4), "yoke": Ta, "proximal": Tm, "middle": Tp, "distal": Td}, dip


def xf(M, pts):
    pts = np.atleast_2d(pts)
    return pts @ M[:3, :3].T + M[:3, 3]


# ================================================================================================ strands
def base_dowel(top_c, top_r, top_side, z_b):
    """base guide dowel (centre, signed r): the strand leaves the yoke abd-plane dowel slanted SLANT deg toward
    top_side (+1 = +y) and turns back to vertical on this dowel. Tangent maths: point = c + r n, travel = perp(n)."""
    a = SLANT * D2R
    h = np.array([top_side * math.sin(a), -math.cos(a)])
    n = np.array([h[1], -h[0]])
    p_top = np.asarray(top_c, float) + top_r * n
    rb = -top_r
    t = (z_b - p_top[1]) / h[1]
    p_b = p_top + t * h
    return p_b - rb * n, rb


def strand_defs():
    """2D (y, z) circles per strand in the YOKE frame at abd = 0; each element tagged with its body."""
    C0 = np.array([0.0, 0.0])
    Cp = np.array([0.0, ZP])
    d = {}
    # pip flex: anchor on the PIP ring (middle), ccw around the palm side, crossed to the idler DORSAL side (cw),
    # down to the abd-plane dowel (cw), slant toward -y, base dowel (ccw), exit
    d["pip_flex"] = dict(x=(X_RING_P - 1.0, -X_PIP_I), drum=(Cp, +R, "middle"),
                         elems=[(C0, -R_I, "yoke"), (np.array(YD_PIPF), -RD, "yoke")], side=-1, zb=BASE_DOWEL_Z["pip_flex"])
    d["pip_ext"] = dict(x=(X_RING_P + 1.0, X_PIP_I), drum=(Cp, -R, "middle"),
                        elems=[(C0, +R_I, "yoke"), (np.array(REDIRECT), -RD, "yoke"), (np.array(YD_PIPE), -RD, "yoke")],
                        side=-1, zb=BASE_DOWEL_Z["pip_ext"])
    d["mcp_flex"] = dict(x=(-X_MCP, -X_MCP), drum=(C0, +R, "proximal"),
                         elems=[(np.array(YD_MCPF), +RD, "yoke")], side=+1, zb=BASE_DOWEL_Z["mcp_flex"])
    d["mcp_ext"] = dict(x=(X_MCP, X_MCP), drum=(C0, -R, "proximal"),
                        elems=[(np.array(YD_MCPE), -RD, "yoke")], side=-1, zb=BASE_DOWEL_Z["mcp_ext"])
    for k, v in d.items():
        tc, tr, _ = v["elems"][-1]
        cb, rb = base_dowel(tc, tr, v["side"], v["zb"])
        v["base"] = (cb, rb)
    return d


SDEF = strand_defs()


def pip_anchor_phi():
    """PIP anchor angle (middle frame, deg from +y toward +z) with equal wrap margins at the range ends."""
    th = {}
    for k in ("pip_flex", "pip_ext"):
        s = SDEF[k]
        c, r, _ = s["drum"]
        nc, nr, _ = s["elems"][0]
        p1, _ = tangent(c, r, nc, nr)
        th[k] = math.degrees(math.atan2(p1[1] - c[1], p1[0] - c[0]))
    # flex tangent on the palm side (~180+), ext on the back side (~0-)
    tf = th["pip_flex"] % 360
    te = (th["pip_ext"] + 180) % 360 - 180
    lo, hi = RANGE["pip"]
    # the anchor rotates with +pip toward larger phi: flex wrap = tf - (phi + pip) >= m at pip = hi; ext wrap =
    # (phi + pip) - te >= m at pip = lo
    phi = (tf + te - hi - lo) / 2
    return phi, tf - phi - hi


PHI_PIP, PIP_MARGIN = pip_anchor_phi()


def strand(name, q, with_pts=False):
    """exact belt length from the drum anchor to the base exit (z = Z_BOT); abd handled by the 3D free segment."""
    s = SDEF[name]
    c, r, body = s["drum"]
    elems = [(e[0], e[1]) for e in s["elems"]]
    mcp, pip = q["mcp"], q["pip"]
    # move the drum (and its anchor) with the joints; elements on the yoke stay
    if body == "middle":
        cc = rot_about(c, (0, 0), mcp)
        phi = PHI_PIP + pip + mcp
        anchor = cc + abs(r) * np.array([math.cos(phi * D2R), math.sin(phi * D2R)])
    else:
        cc = c
        phi = PHI_MCP + mcp
        anchor = cc + abs(r) * np.array([math.cos(phi * D2R), math.sin(phi * D2R)])
    cb, rb = s["base"]
    # vertical final segment: travel (0, -1) -> n = (-1, 0) -> tangent point cb - rb (1, 0)
    exit_pt = np.array([cb[0] - rb, Z_BOT])
    nodes_c = [cc] + [e[0] for e in elems] + [cb]
    nodes_r = [r] + [e[1] for e in elems] + [rb]
    L = 0.0
    wraps = []
    pts = [anchor]
    tans = [tangent(nodes_c[i], nodes_r[i], nodes_c[i + 1], nodes_r[i + 1]) for i in range(len(nodes_c) - 1)]
    tans.append(tangent(nodes_c[-1], nodes_r[-1], exit_pt, 0.0))
    la, ang, ap = arc_len(cc, r, anchor, tans[0][0])
    L += la
    wraps.append(ang)
    pts.extend(ap)
    seg_E_F = None
    for i, (p1, p2) in enumerate(tans):
        L += float(np.linalg.norm(p2 - p1))
        if i == len(tans) - 2:
            seg_E_F = (p1, p2)  # yoke abd-plane dowel -> base dowel: the free segment across the abd joint
        pts.append(p1)
        pts.append(p2)
        if i + 1 < len(nodes_c):
            la, ang, ap = arc_len(nodes_c[i + 1], nodes_r[i + 1], p2, tans[i + 1][0])
            L += la
            wraps.append(ang)
            pts.extend(ap)
    # 3D correction for abduction
    xs = s["x"][1]
    E = np.array([xs, *seg_E_F[0]])
    F = np.array([xs, *seg_E_F[1]])
    Ta = mat_rot((0, 1, 0), q["abd"], (0, 0, Z_ABD))
    E3 = xf(Ta, E)[0]
    dL = float(np.linalg.norm(E3 - F) - np.linalg.norm(E - F))
    L += dL
    if not with_pts:
        return L, wraps
    pts = np.array(pts)
    # 3D points: everything above the base dowel entry F is on the yoke side -> abd rotation
    n_yoke = len(pts)
    for i in range(len(pts)):
        if np.allclose(pts[i], seg_E_F[1]):
            n_yoke = i
            break
    x_line = np.interp(pts[:, 1], [min(0.0, ZP), ZP], [s["x"][1], s["x"][0]]) if body == "middle" else np.full(len(pts), xs)
    P3 = np.column_stack([x_line, pts[:, 0], pts[:, 1]])
    P3[:n_yoke] = xf(Ta, P3[:n_yoke])
    return L, wraps, P3


def abd_strand(role, q, with_pts=False):
    """abd ring about Y at (x, z) = (0, Z_ABD); flex (+x side, lane y 10.2) / ext (-x side, lane y 12.2)."""
    sgn = 1 if role == "flex" else -1
    y = Y_ABD - 1.0 if role == "flex" else Y_ABD + 1.0
    th_a = 90.0 - q["abd"]  # anchor angle in the xz plane from +x toward +z; +abd turns it toward +x
    th_t = 0.0 if role == "flex" else 180.0
    wrap = (th_a - th_t) if role == "flex" else (th_t - th_a)
    L = R * wrap * D2R + (Z_ABD - Z_BOT)
    if not with_pts:
        return L, [wrap]
    t = np.radians(np.linspace(th_a, th_t, 30))
    arc = np.column_stack([R * np.cos(t), np.full(30, y), Z_ABD + R * np.sin(t)])
    pts = np.vstack([arc, [[sgn * R, y, Z_BOT]]])
    return L, [wrap], pts


def strand_checks():
    res = {}
    grid_abd = [-15, -7.5, 0, 7.5, 15]
    grid_mcp = list(np.arange(-10, 90.1, 10))
    grid_pip = list(np.arange(-5, 95.1, 10))
    rows = []
    for qa in grid_abd:
        for qm in grid_mcp:
            for qp in grid_pip:
                q = dict(abd=qa, mcp=qm, pip=qp)
                r = {k: strand(k, q)[0] for k in SDEF}
                r["abd_flex"] = abd_strand("flex", q)[0]
                r["abd_ext"] = abd_strand("ext", q)[0]
                wr = {k: strand(k, q)[1] for k in SDEF}
                rows.append((qa, qm, qp, r, wr))
    Q = np.array([[a, m, p] for a, m, p, _, _ in rows])
    for loop, (f, e) in {"pip": ("pip_flex", "pip_ext"), "mcp": ("mcp_flex", "mcp_ext"),
                         "abd": ("abd_flex", "abd_ext")}.items():
        Lf = np.array([r[f] for *_, r, _ in rows])
        Le = np.array([r[e] for *_, r, _ in rows])
        S = Lf + Le
        X = np.column_stack([np.ones(len(Q)), Q])
        cf, *_ = np.linalg.lstsq(X, Lf, rcond=None)
        resid = Lf - X @ cf
        res[loop] = dict(loop_sum_variation_mm=float(S.max() - S.min()),
                         flex_mm_per_deg=dict(abd=float(cf[1]), mcp=float(cf[2]), pip=float(cf[3])),
                         flex_linear_fit_residual_mm=float(np.abs(resid).max()))
    # coupling in joint degrees (what the firmware adds): flex slope / own slope
    own = {"pip": res["pip"]["flex_mm_per_deg"]["pip"], "mcp": res["mcp"]["flex_mm_per_deg"]["mcp"]}
    res["coupling_deg_per_deg"] = dict(
        pip_per_mcp=res["pip"]["flex_mm_per_deg"]["mcp"] / own["pip"],
        pip_per_abd=res["pip"]["flex_mm_per_deg"]["abd"] / own["pip"],
        mcp_per_abd=res["mcp"]["flex_mm_per_deg"]["abd"] / own["mcp"])
    # minimum wrap on each element over the grid (must stay > 0: the strand stays on its guide)
    wmin = {}
    for k in SDEF:
        W = np.array([w[k] for *_, w in rows])
        wmin[k] = [float(v) for v in W.min(axis=0)]
    res["min_wrap_deg_per_element"] = wmin
    res["pip_anchor_deg"] = PHI_PIP
    res["pip_wrap_margin_deg"] = PIP_MARGIN
    res["grid"] = dict(abd=grid_abd, mcp=[float(v) for v in grid_mcp], pip=[float(v) for v in grid_pip])
    # DIP coupling
    pg = np.linspace(-5, 95, 41)
    dd = np.array([dip_of(p) for p in pg])
    res["dip"] = dict(max_err_vs_0p75_deg=float(np.abs(dd - 0.75 * pg).max()), dip_at_pip95=float(dd[-1]))
    return res


# ================================================================================================ parts
def hw_prox(z):
    return HW_K + (HW_P - HW_K) * np.clip(z / ZP, 0, 1)


def build_yoke():
    """abd child / MCP parent: tongue with the axle and idler, pip slot, dowels, MCP ears, abd disc, stubs."""
    tongue = U(cylx(6.0, -TONGUE, TONGUE), box(-TONGUE, TONGUE, -6.0, 6.0, -9.5, 0.0),
               box(-TONGUE, TONGUE, 0.0, 9.4, -9.5, -3.6))
    body = U(box(-5.0, 5.0, -8.0, 8.4, Z_ABD - 4.0, -9.0), box(-5.0, 5.0, -12.6, -8.0, Z_ABD - 4.6, Z_ABD + 4.6))
    ears = U(box(-9.0, -4.8, -7.9, -1.5, Z_ABD - 2.5, Z_ABD + 2.5), box(4.8, 9.0, 1.5, 7.9, Z_ABD - 2.5, Z_ABD + 2.5))
    disc = cyly(FL_R, 8.4, 12.7, 0.0, Z_ABD).union(cyly(RING_RI, 12.7, 14.0, 0.0, Z_ABD))
    y = U(tongue, body, ears, disc)
    # abd ring groove (ring slides on from the +y side; flange on the yoke side)
    y = y.cut(cyly(FL_R + 1, 9.7, 12.7, 0.0, Z_ABD).cut(cyly(RING_RI, 9.0, 13.0, 0.0, Z_ABD)))
    # pip slot |x| < 1.7: idler circle, strand band, crossing band, exit to the top
    slot_yz = [
        cylx(R_I + 0.45, -IDLER_SLOT, IDLER_SLOT),
        box(-IDLER_SLOT, IDLER_SLOT, 2.9, 8.6, Z_ABD - 5.0, 0.0),
        box(-IDLER_SLOT, IDLER_SLOT, -0.5, 8.6, -4.9, -3.5),
        box(-IDLER_SLOT, IDLER_SLOT, -R_I - 0.45, R_I + 0.45, 0.0, 6.5),
    ]
    for s in slot_yz:
        y = y.cut(s)
    # below the abd plane the strands swing with abduction (pendulum about the abd-plane dowels): flared slots
    fl = math.tan(17 * D2R)
    dz = 6.0
    vflare = cq.Workplane("XZ").polyline([(-IDLER_SLOT, Z_ABD + 0.5), (IDLER_SLOT, Z_ABD + 0.5),
                                          (IDLER_SLOT + dz * fl, Z_ABD - dz), (-IDLER_SLOT - dz * fl, Z_ABD - dz)]).close()
    y = y.cut(vflare.extrude(-(8.6 - 2.9)).translate((0, 2.9, 0)))
    for sx, y0, y1 in ((-1, -8.4, -6.3), (1, 6.3, 8.4)):
        xc = sx * X_MCP
        ef = cq.Workplane("XZ").polyline([(xc - 0.5, Z_ABD + 0.5), (xc + 0.5, Z_ABD + 0.5), (xc + 0.5 + dz * fl, Z_ABD - dz),
                                          (xc - 0.5 - dz * fl, Z_ABD - dz)]).close()
        y = y.cut(ef.extrude(-(y1 - y0)).translate((0, y0, 0)))
    # the pip strands leave the idler toward the PIP drum; that line turns with the MCP: swept exit band
    for k in ("pip_flex", "pip_ext"):
        sd = SDEF[k]
        p_drum, p_idl = tangent(sd["drum"][0], sd["drum"][1], sd["elems"][0][0], sd["elems"][0][1])
        d = (p_drum - p_idl) / np.linalg.norm(p_drum - p_idl)
        nrm = np.array([-d[1], d[0]])
        q4 = [p_idl - 0.6 * nrm - 0.5 * d, p_idl + 12 * d - 0.6 * nrm, p_idl + 12 * d + 0.6 * nrm, p_idl + 0.6 * nrm - 0.5 * d]
        band = prism_yz(q4, -IDLER_SLOT, IDLER_SLOT)
        y = y.cut(swept(band, (0, 0, 0), (1, 0, 0), angles(*RANGE["mcp"], step=4.0)))
    # axle, sleeves
    y = y.cut(cylx(AXLE["od"] / 2, -TONGUE - 1, TONGUE + 1))
    for s in (-1, 1):
        y = y.cut(cylx(SLEEVE["od"] / 2, s * SLEEVE["x"][0], s * (TONGUE + 0.5)))
    # dowel holes (pressed): redirect + 2 pip abd-plane dowels across the slot, MCP ear dowels
    holes = [cylx(1.5, -4.0, 4.0, *REDIRECT), cylx(1.5, -4.0, 4.0, *YD_PIPF), cylx(1.5, -4.0, 4.0, *YD_PIPE),
             cylx(1.5, -9.0, -4.8, *YD_MCPF), cylx(1.5, 4.8, 9.0, *YD_MCPE)]
    for h in holes:
        y = y.cut(h)
    # MCP strand slots through the ears (lane +-6.4, 1.0 wide)
    y = y.cut(box(-X_MCP - 0.5, -X_MCP + 0.5, -8.4, -6.3, Z_ABD - 3.0, Z_ABD + 3.0))
    y = y.cut(box(X_MCP - 0.5, X_MCP + 0.5, 6.3, 8.4, Z_ABD - 3.0, Z_ABD + 3.0))
    # abd stubs: palmar (hollow Ø3 tube, wires) and dorsal (solid Ø3)
    y = y.cut(cyly(SEAT63 / 2, -12.7, -10.1, 0.0, Z_ABD)).cut(cyly(2.4, -10.2, -8.0, 0.0, Z_ABD))
    y = y.cut(cyly(1.5, 10.2, 14.5, 0.0, Z_ABD))
    # abd fuse cup pocket at the top of the disc
    cup, pocket = oval_cup_parts(90.0, Y_ABD, 1.0, 0.8, axis_center=(0.0, Z_ABD), axis="y")
    y = y.cut(pocket)
    # wire path: axle radial slot at x = -3 -> down -> palmar stub
    y = y.cut(teardrop(0.8, (-3.0, 0.0, -1.5), (-3.0, 0.0, Z_ABD + 1.3)))
    y = y.cut(cyl(0.8, (-3.0, 0.0, Z_ABD + 1.3), (-0.5, -8.6, Z_ABD)))
    return y, cup


def build_proximal(yoke_env):
    lugs = []
    for s in (-1, 1):
        x0, x1 = sorted((s * MCP_LUG[0], s * MCP_LUG[1]))
        core = cylx(RING_RI, x0, x1)
        xf0, xf1 = sorted((s * MCP_RING[1], s * MCP_LUG[1]))
        flange = cylx(FL_R, xf0, xf1)
        lugs.append(U(core, flange))
    # shaft (taper in x and y) from z 3 to the PIP
    zs = [3.0, ZP]
    xz = [(-hw_prox(z), z) for z in zs] + [(hw_prox(z), z) for z in zs[::-1]]
    A = cq.Workplane("XZ").polyline([(x, z) for x, z in xz]).close().extrude(-HH_P0 * 2).translate((0, -HH_P0, 0))
    yz = [(-HH_P0, 3.0), (-HH_P1, ZP), (HH_P1, ZP), (HH_P0, 3.0)]
    B = prism_yz(yz, -10, 10)
    shaft = A.intersect(B)
    # knuckle top (hood) on the -x lug for the flex cup, and the shaft root over the lugs
    hood = sector_yz(RING_RI, 10.4, 15.0, 75.0, -MCP_LUG[1], -MCP_LUG[0])
    root = U(sector_yz(RING_RI, FL_R + 0.0, 60, 120, -MCP_LUG[1], MCP_LUG[1]))
    # PIP fork lugs (discs r 7.7 about the PIP axis)
    lugN = cylx(7.7, PIP_LUGN[0], PIP_LUGN[1], 0, ZP)
    lugP = cylx(7.7, PIP_LUGP[0], PIP_LUGP[1], 0, ZP)
    bossP = cylx(4.0, PIP_LUGP[1], PIP_LUGP[1] + 1.0, 0, ZP)
    bossK = cylx(4.0, MCP_LUG[1], MCP_LUG[1] + 0.6)
    p = U(*lugs, shaft, hood, root, lugN, lugP, bossP, bossK)
    # the MCP ring grooves: the ring slides on from the inner side -> open groove from r 6.5 out to 9 at the lane
    for s in (-1, 1):
        x0, x1 = sorted((s * MCP_LUG[0] - s * 0.3, s * MCP_RING[1]))
        p = p.cut(cylx(9.6, x0, x1).cut(cylx(RING_RI, x0 - 1, x1 + 1)))
    # central region between the lugs around the tongue: swept yoke envelope over the MCP range
    p = p.cut(swept(yoke_env, (0, 0, 0), (1, 0, 0), [-a for a in angles(*RANGE["mcp"])]))
    # MCP strands: the yoke-fixed vertical strand below each drum sweeps the shaft root as the MCP turns
    for sx, yl in ((-1, -R), (1, R)):
        seg = box(sx * X_MCP - 0.6, sx * X_MCP + 0.6, yl - 0.5, yl + 0.5, Z_ABD, 0.2)
        p = p.cut(swept(seg, (0, 0, 0), (1, 0, 0), [-a for a in angles(*RANGE["mcp"], step=4.0)]))
    # bearing seats + lips + Hall pocket (+x) + wire groove (-x)
    for s in (-1, 1):
        a, b = sorted((s * MCP_SEAT[0], s * MCP_SEAT[1]))
        p = p.cut(cylx(SEAT84 / 2, a, b))
        a, b = sorted((s * MCP_SEAT[1], s * (MCP_LUG[1] + 1)))
        p = p.cut(cylx(6.75 / 2, a, b))
    p = p.cut(box(MCP_LUG[1] - 0.4, MCP_LUG[1] + 0.7, -2.3, 2.3, -2.3, 2.3))  # 4.6 sq PCB pocket, 1.0 deep
    # PIP: gap for the middle hub, bush + seat + lip + Hall pocket, pin A
    p = p.cut(cylx(FL_R + 0.4, PIP_LUGN[1], PIP_LUGP[0], 0, ZP))
    p = p.cut(cylx(2.5, PIP_BUSH[0], PIP_BUSH[1], 0, ZP)).cut(cylx(0.8, PIP_LUGN[0] - 1, PIP_BUSH[0] + 0.1, 0, ZP))
    p = p.cut(cylx(SEAT84 / 2, PIP_SEAT[0], PIP_SEAT[1], 0, ZP)).cut(cylx(6.75 / 2, PIP_SEAT[1], PIP_LUGP[1] + 2, 0, ZP))
    p = p.cut(box(PIP_LUGP[1], PIP_LUGP[1] + 1.1, -2.3, 2.3, ZP - 2.3, ZP + 2.3))
    A0 = link_pins(0, 0)[2]
    p = p.cut(cylx(1.0, PIP_LUGN[0] - 1, PIP_LUGN[0] + 4.0, A0[0], A0[1]))
    # bar recess: -x face at X_BARFACE for z > 30
    p = p.cut(box(-12, X_BARFACE, -12, 12, 30.0, ZP + 12))
    # pip strand grooves, open toward +x (lay the strand in from the side)
    for k, xfloor in (("pip_flex", -1.7), ("pip_ext", 0.3)):
        sd = SDEF[k]
        p_drum, p_idl = tangent(sd["drum"][0], sd["drum"][1], sd["elems"][0][0], sd["elems"][0][1])
        if True:
            a, b = np.array([0.0, *p_idl]), np.array([0.0, *p_drum])
            d = (b - a) / np.linalg.norm(b - a)
            nrm = np.array([0, -d[2], d[1]])
            q4 = [a[1:] + 0.7 * nrm[1:] - 2 * d[1:], b[1:] + 0.7 * nrm[1:] + 6 * d[1:],
                  b[1:] - 0.7 * nrm[1:] + 6 * d[1:], a[1:] - 0.7 * nrm[1:] - 2 * d[1:]]
            p = p.cut(prism_yz(q4, xfloor, 12.0))
    # wire groove on the -x face: MCP lug centre -> along the face -> PIP -x lug centre (1.2 wide, 1.0 deep)
    p = p.cut(box(-HW_K - 1, -HW_K + 1.0, -0.6, 0.6, 0.0, 30.0))
    p = p.cut(box(-HW_K - 1, X_BARFACE + 1.0, -0.6, 0.6, 29.4, ZP))
    # MCP fuse cups (tangential): flex on -x lug (wrap continues with phi decreasing), ext on +x lug
    cupF, pocF = tangential_cup(PHI_MCP, -X_MCP, +1)
    cupE, pocE = tangential_cup(PHI_MCP, X_MCP, -1)
    p = p.cut(pocF).cut(pocE)
    # PIP: open the -x lug where the bush sits (done), round-over not modelled
    return p, cupF, cupE


def build_middle(prox_env, distal_env):
    hub = U(cylx(RING_RI, PIP_HUB[0], PIP_HUB[1], 0, ZP), cylx(FL_R, PIP_RING[1], PIP_HUB[1], 0, ZP))
    zs = [ZP + 2.0, ZD]
    hw = [HW_P, HW_D]
    xz = [(X_BARFACE, zs[0]), (X_BARFACE, zs[1]), (hw[1], zs[1]), (hw[0], zs[0])]
    A = cq.Workplane("XZ").polyline(xz).close().extrude(-20).translate((0, -10, 0))
    yz = [(-HH_P1, zs[0]), (-HH_M1, zs[1]), (HH_M1, zs[1]), (HH_P1, zs[0])]
    shaft = A.intersect(prism_yz(yz, -10, 10))
    tongue = cylx(6.5, DIP_TONG[0], DIP_TONG[1], 0, ZD)
    m = U(hub, shaft, tongue)
    ring_slot = cylx(9.6, PIP_RING[0] - 0.3, PIP_RING[1], 0, ZP).cut(cylx(RING_RI, -5, 5, 0, ZP))
    # the shaft between the PIP lugs must stay outside the proximal lugs: swept proximal envelope
    m = m.cut(swept(prox_env, (0, 0, ZP), (1, 0, 0), [-a for a in angles(RANGE["pip"][0], SWEEP_PIP_MAX)]))
    # restore the hub inside the gap (the envelope sweep is conservative there)
    m = m.union(hub).cut(ring_slot)
    # DIP: clear the distal cheeks (incl. the pin-B lobe) over the DIP range
    dlo, dhi = dip_of(RANGE["pip"][0]), dip_of(SWEEP_PIP_MAX)
    m = m.cut(swept(distal_env, (0, 0, ZD), (1, 0, 0), angles(dlo, dhi)))
    m = m.union(tongue)
    # pins
    m = m.cut(cylx(2.0, -10, 10, 0, ZP)).cut(cylx(2.0, -10, 10, 0, ZD))
    # PIP fuse cup (two lanes) at the anchor
    cup, pocket = oval_cup_parts(PHI_PIP, X_RING_P, 1.0, 1.2, axis_center=(0.0, ZP))
    m = m.cut(pocket)
    # wire channel: PIP pin (radial at x 3.2) -> along z -> DIP pin
    m = m.cut(cyly(0.8, 0.0, 3.4, 3.2, ZP)).cut(teardrop(0.8, (3.2, 3.0, ZP), (3.2, 3.0, ZD)))
    m = m.cut(cyly(0.8, 0.0, 3.4, 3.2, ZD))
    return m, cup


def build_distal(middle_env):
    chN = U(cylx(7.0, DIP_CHN[0], DIP_CHN[1], 0, ZD))
    Bp = link_pins(0, 0)[3]
    lobe = U(cylx(2.6, DIP_CHN[0], DIP_CHN[1], Bp[0], Bp[1]),
             box(DIP_CHN[0], DIP_CHN[1], Bp[0], 0.0, Bp[1] - 2.6, Bp[1] + 2.6))
    chP = cylx(7.0, DIP_CHP[0], DIP_CHP[1], 0, ZD)
    # tip body (05 tip, adapted to the 14 mm taper): z_rel 4..23, x +-7, y -5.8..7
    YF, YB = -5.8, 7.0
    z0, z1 = ZD + 4.0, ZT
    xz = [(X_BARFACE, z0), (-HW_T, z1), (HW_T, z1), (HW_D, z0)]
    A = cq.Workplane("XZ").polyline(xz).close().extrude(-20).translate((0, -10, 0))
    body = A.intersect(box(-10, 10, YF, YB, z0, z1))
    body = body.edges("|Y and >Z").fillet(4.0)
    base_blk = box(DIP_CHN[0], DIP_CHP[1], YF, YB, ZD + 4.0, ZD + 8.0)
    d = U(chN, lobe, chP, body, base_blk)
    # clear the middle tongue over the DIP range
    dlo, dhi = dip_of(RANGE["pip"][0]), dip_of(SWEEP_PIP_MAX)
    d = d.cut(swept(middle_env, (0, 0, ZD), (1, 0, 0), [-a for a in angles(dlo, dhi)]))
    d = d.union(chN).union(chP).union(lobe)
    d = d.cut(cylx(6.5 + CLR, DIP_CHN[1] + 0.2 - 0.2, DIP_CHP[0], 0, ZD))
    # bushes, pin B, wire groove on +x face, cable bore to the cavity
    d = d.cut(cylx(2.5, DIP_BUSHN[0], DIP_BUSHN[1], 0, ZD)).cut(cylx(0.8, DIP_CHN[0] - 1, DIP_BUSHN[0] + 0.1, 0, ZD))
    d = d.cut(cylx(2.5, DIP_CHP[0], DIP_CHP[1] + 1, 0, ZD))
    d = d.cut(cylx(1.0, DIP_CHN[0] - 1, DIP_CHN[0] + 4.0, Bp[0], Bp[1]))
    d = d.cut(box(DIP_CHP[1] - 1.2, DIP_CHP[1] + 1, -0.6, 0.6, ZD, ZD + 8.3))
    d = d.cut(cylx(0.8, -0.5, DIP_CHP[1] - 0.6, -0.2, ZD + 7.7))
    # sensor cavity with 45 deg undercut (05, scaled: opening +-3.8, widest +-5.2, 4.8 deep), z_rel 10..19.5
    s = lambda v: YF - v  # noqa: E731
    cav = [(-3.8, s(0.0)), (3.8, s(0.0)), (5.2, s(-1.4)), (5.2, s(-4.8)), (-5.2, s(-4.8)), (-5.2, s(-1.4))]
    cavw = cq.Workplane("XY").workplane(offset=ZD + 10.0).polyline(cav).close().extrude(9.5)
    d = d.cut(cavw)
    d = d.cut(cylz(1.3, ZD + 7.5, ZD + 10.1, 0.0, s(-4.4)))
    d = d.cut(box(-4.5, 4.5, YF, YF + 0.25, ZT - 5.0, ZT + 0.1))  # nail shim slot (0.15 shim)
    # skin (snap-in plug + crowned pad) and refs
    plug = cq.Workplane("XY").workplane(offset=ZD + 10.3).polyline(
        [(-3.8, s(0.0)), (3.8, s(0.0)), (5.2, s(-1.4)), (5.2, s(-2.3)), (-5.2, s(-2.3)), (-5.2, s(-1.4))]).close().extrude(8.9)
    pad = box(-6.4, 6.4, s(3.0), YF, ZD + 9.0, ZT - 0.6).edges("|Y").fillet(3.5)
    crown = cylz(25.0, ZD + 8.0, ZT + 1.0, 0.0, s(3.0) + 25.0)
    skin = pad.intersect(crown).union(plug).cut(cyly(1.4, s(2.4), s(-2.3) + 0.01, 0.0, ZD + 15.0))
    pcb = box(-4.6, 4.6, s(-4.8) - 0.8, s(-4.8), ZD + 10.3, ZD + 19.3)
    chip = box(-1.5, 1.5, s(-4.8) - 1.8, s(-4.8) - 0.8, ZD + 13.5, ZD + 16.5)
    mag = cyly(1.5, s(2.4), s(0.9), 0.0, ZD + 15.0)
    shim = box(-4.5, 4.5, YF, YF + 0.15, ZT - 5.0, ZT + 1.5)
    return d, dict(tip_skin=skin, tip_pcb=pcb, tip_chip=chip, tip_magnet=mag, nail_shim=shim)


def build_base():
    """bench block. Top (under the yoke) at Z_ABD - 6.5; open front bay where the curled finger goes;
    palmar abd support = a low bridge between side pillars holding the fixed Ø3 stub (top <= proximal at MCP 90)."""
    ZTOP = Z_ABD - 6.5
    blk = box(-16.0, 16.0, -9.5, 20.0, Z_BOT, ZTOP)  # rear block (behind the palm plane)
    pillars = U(box(-16.0, -13.0, -18.0, -9.5, Z_BOT, Z_ABD - 3.5), box(13.0, 16.0, -18.0, -9.5, Z_BOT, Z_ABD - 3.5))
    floor = box(-16.0, 16.0, -18.0, -9.5, Z_BOT, -46.0)
    bridge = U(box(-16.0, 16.0, -17.0, -12.9, Z_ABD - 3.5, Z_ABD + 2.4), cyly(3.1, -17.0, -12.9, 0.0, Z_ABD))
    lugD = box(-6.0, 6.0, 14.2, 17.8, ZTOP, Z_ABD + 4.6)
    b = U(blk, pillars, floor, bridge, lugD)
    # strand channel and exits through the bottom; window in the front for threading
    b = b.cut(box(-11.0, 11.0, -9.5, 14.0, Z_BOT - 1, ZTOP + 1))
    # abd: fixed stub (palmar, hollow) + MR63 seat (dorsal)
    b = b.cut(cyly(1.5, -18.0, -12.8, 0.0, Z_ABD))
    b = b.cut(cyly(SEAT63 / 2, 14.1, 16.7, 0.0, Z_ABD)).cut(cyly(2.4, 16.6, 18.0, 0.0, Z_ABD))
    # base guide dowels along X, cantilevered from one side wall
    for k, s in SDEF.items():
        cb, rb = s["base"]
        x0, x1 = BASE_DOWEL_X[k]
        b = b.cut(cylx(1.5, x0 - 0.5, x1 + 0.01, cb[0], cb[1]) if x0 < -15 else cylx(1.5, x0 - 0.01, x1 + 0.5, cb[0], cb[1]))
    # mounting through the dorsal face: 2 x M5 (2020 slot, 20 apart) + 4 x M3 (plate)
    for z in (-28.0, -48.0):
        b = b.cut(cyly(2.75, 13.0, 21.0, 0.0, z))
    for x in (-13.5, 13.5):
        for z in (-26.0, -50.0):
            b = b.cut(cyly(1.6, -19.0, 21.0, x, z))
    return b


def envelopes():
    """simplified clearance envelopes (+CLR) for the swept cuts."""
    yoke_env = U(cylx(6.0 + CLR, -TONGUE - 0.0, TONGUE + 0.0),
                 box(-TONGUE, TONGUE, -6.0 - CLR, 9.4 + CLR, -30.0, 0.0),
                 box(-9.0 - CLR, -4.8, -7.9 - CLR, -1.5 + CLR, -30.0, Z_ABD + 2.5 + CLR),
                 box(4.8, 9.0 + CLR, 1.5 - CLR, 7.9 + CLR, -30.0, Z_ABD + 2.5 + CLR),
                 box(-5.0 - CLR, 5.0 + CLR, -12.6 - CLR, -6.0, -30.0, Z_ABD + 4.6 + CLR))
    prox_env = U(cylx(7.7 + CLR, PIP_LUGN[0] - 2, PIP_LUGN[1] + CLR, 0, ZP),
                 cylx(7.7 + CLR, PIP_LUGP[0] - CLR, PIP_LUGP[1] + 2, 0, ZP),
                 box(-10, 10, -HH_P1 - CLR, HH_P1 + CLR, 20.0, ZP).cut(cylx(FL_R + 0.4, PIP_LUGN[1], PIP_LUGP[0], 0, ZP)))
    Bp = link_pins(0, 0)[3]
    distal_env = U(cylx(7.0 + CLR, DIP_CHN[0] - 2, DIP_CHN[1] + CLR, 0, ZD),
                   cylx(2.6 + CLR, DIP_CHN[0] - 2, DIP_CHN[1] + CLR, Bp[0], Bp[1]),
                   box(DIP_CHN[0] - 2, DIP_CHN[1] + CLR, Bp[0] - CLR, 0.0, Bp[1] - 2.6 - CLR, Bp[1] + 2.6 + CLR),
                   cylx(7.0 + CLR, DIP_CHP[0] - CLR, DIP_CHP[1] + 2, 0, ZD))
    middle_env = U(cylx(6.5 + CLR, DIP_TONG[0] - CLR, DIP_TONG[1] + CLR, 0, ZD),
                   box(-10, 10, -HH_M1 - CLR, HH_M1 + CLR, ZD - 14.0, ZD).cut(cylx(7.0 + CLR, DIP_CHN[0] - 3, DIP_CHN[1] + 0.2, 0, ZD)).cut(
                       cylx(7.0 + CLR, DIP_CHP[0] - 0.2, DIP_CHP[1] + 3, 0, ZD)))
    return yoke_env, prox_env, distal_env, middle_env


def build_metal():
    """bought parts, each tagged with its body."""
    m = {}
    # MCP
    m["mcp_axle"] = ("yoke", cylx(2.0, AXLE["x0"], AXLE["x1"]).cut(cylx(1.5, AXLE["x0"] - 1, AXLE["x1"] - 2.0)).cut(
        box(-3.6, -2.4, -2.1, 0, -2.1, -1.0)), "metal")
    m["mcp_magnet"] = ("yoke", cylx(1.5, AXLE["x1"] - 2.0, AXLE["x1"]), "magnet")
    for s, nm in ((-1, "n"), (1, "p")):
        a, b = sorted((s * SLEEVE["x"][0], s * SLEEVE["x"][1]))
        sl = cylx(2.5, a, b).cut(cylx(2.0, a - 1, b + 1))
        if s < 0:
            sl = sl.cut(cyl(0.8, (-3.0, 0.0, -1.5), (-3.0, 0.0, -3.0)))  # Ø1.6 wire hole, in line with the axle slot
        m[f"mcp_sleeve_{nm}"] = ("yoke", sl, "brass")
        a, b = sorted((s * MCP_SEAT[0], s * MCP_SEAT[1]))
        bi, bo = bearing(MR84, a, b)
        m[f"mcp_brg_{nm}_in"], m[f"mcp_brg_{nm}_out"] = ("yoke", bi, "bearing"), ("proximal", bo, "bearing")
        a, b = sorted((s * MCP_RING[0], s * MCP_RING[1]))
        m[f"mcp_ring_{nm}"] = ("proximal", ring_part(RING_RO, RING_RI, a, b, gap_phi=(35, 62)), "brass")
    bi, bo = bearing(MR84, -1.5, 1.5)
    m["idler_in"], m["idler_out"] = ("yoke", bi, "bearing"), ("yoke", bo, "bearing")
    for nm, (yy, zz), x0, x1 in (("dowel_redirect", REDIRECT, -4.0, 4.0), ("dowel_pipflex", YD_PIPF, -4.0, 4.0),
                                 ("dowel_pipext", YD_PIPE, -4.0, 4.0), ("dowel_mcpflex", YD_MCPF, -9.0, -4.8),
                                 ("dowel_mcpext", YD_MCPE, 4.8, 9.0)):
        m[nm] = ("yoke", cylx(1.5, x0, x1, yy, zz), "metal")
    # abd
    m["abd_stub_palmar"] = ("base", cyly(1.5, -17.0, -10.1, 0, Z_ABD).cut(cyly(1.0, -18, -9, 0, Z_ABD)), "metal")
    m["abd_stub_dorsal"] = ("yoke", cyly(1.5, 10.2, 16.7, 0, Z_ABD), "metal")
    bi, bo = bearing(MR63, -12.6, -10.1, 0.0, Z_ABD, axis="y")
    m["abd_brg_p_in"], m["abd_brg_p_out"] = ("base", bi, "bearing"), ("yoke", bo, "bearing")
    bi, bo = bearing(MR63, 14.2, 16.7, 0.0, Z_ABD, axis="y")
    m["abd_brg_d_in"], m["abd_brg_d_out"] = ("yoke", bi, "bearing"), ("base", bo, "bearing")
    m["abd_ring"] = ("yoke", cyly(RING_RO, 9.7, 12.7, 0, Z_ABD).cut(cyly(RING_RI, 9, 13.5, 0, Z_ABD)).cut(
        box(-1.6, 1.6, 9.0, 13.5, Z_ABD + 5.5, Z_ABD + 8.0)), "brass")
    # PIP
    m["pip_pin"] = ("middle", cylx(2.0, *PIP_PIN, 0, ZP).cut(cylx(1.5, PIP_PIN[0] - 1, PIP_PIN[1] - 2.0, 0, ZP)).cut(
        box(2.6, 3.8, 0, 2.1, ZP - 0.6, ZP + 0.6)), "metal")
    m["pip_magnet"] = ("middle", cylx(1.5, PIP_PIN[1] - 2.0, PIP_PIN[1], 0, ZP), "magnet")
    m["pip_bush"] = ("proximal", cylx(2.5, *PIP_BUSH, 0, ZP).cut(cylx(2.0, -9, 0, 0, ZP)), "brass")
    bi, bo = bearing(MR84, *PIP_SEAT, 0, ZP)
    m["pip_brg_in"], m["pip_brg_out"] = ("middle", bi, "bearing"), ("proximal", bo, "bearing")
    gap = (PHI_PIP - 14, PHI_PIP + 14)
    m["pip_ring"] = ("middle", ring_part(RING_RO, RING_RI, *PIP_RING, gap_phi=gap, zc=ZP), "brass")
    # DIP
    m["dip_pin"] = ("middle", cylx(2.0, *DIP_PIN, 0, ZD).cut(cylx(1.5, -1.0, DIP_PIN[1] + 1, 0, ZD)).cut(
        box(2.6, 3.8, 0, 2.1, ZD - 0.6, ZD + 0.6)), "metal")
    m["dip_bush_n"] = ("distal", cylx(2.5, *DIP_BUSHN, 0, ZD).cut(cylx(2.0, -9, 0, 0, ZD)), "brass")
    m["dip_bush_p"] = ("distal", cylx(2.5, *DIP_CHP, 0, ZD).cut(cylx(2.0, 0, 9, 0, ZD)), "brass")
    # DIP bar pins
    A0, B0 = link_pins(0, 0)[2:]
    m["pin_A"] = ("proximal", cylx(1.0, PIP_LUGN[0] + 4.0 - 6.0 - 0.95, PIP_LUGN[0] + 4.0, A0[0], A0[1]), "metal")
    m["pin_B"] = ("distal", cylx(1.0, DIP_CHN[0] + 4.0 - 6.0 - 0.95, DIP_CHN[0] + 4.0, B0[0], B0[1]), "metal")
    # Hall PCBs (TMAG5273 on 4.6 sq boards, chip toward the magnet)
    m["hall_mcp_pcb"] = ("proximal", box(MCP_LUG[1] - 0.4, MCP_LUG[1] + 0.4, -2.25, 2.25, -2.25, 2.25), "pcb")
    m["hall_mcp_chip"] = ("proximal", box(MCP_LUG[1] - 1.4, MCP_LUG[1] - 0.4, -0.8, 0.8, -1.45, 1.45), "pcb")
    m["hall_pip_pcb"] = ("proximal", box(PIP_LUGP[1], PIP_LUGP[1] + 0.8, -2.25, 2.25, ZP - 2.25, ZP + 2.25), "pcb")
    m["hall_pip_chip"] = ("proximal", box(PIP_LUGP[1] - 1.0, PIP_LUGP[1], -0.8, 0.8, ZP - 1.45, ZP + 1.45), "pcb")
    for k, s in SDEF.items():
        cb, rb = s["base"]
        m[f"base_dowel_{k}"] = ("base", cylx(1.5, *BASE_DOWEL_X[k], cb[0], cb[1]), "metal")
    return m


def bar_shape(q):
    """stainless strip 0.8 x 3.2 between pin A and pin B (hole centres), in the yoke frame for this pose."""
    dip = dip_of(q["pip"])
    A, B, _, _ = link_pins(q["pip"], dip, q["mcp"])
    L = float(np.linalg.norm(B - A))
    ang = math.degrees(math.atan2(B[1] - A[1], B[0] - A[0]))  # from +y toward +z
    s = cq.Workplane("YZ", origin=(BAR_X0, 0, 0)).slot2D(L + 3.2, 3.2, 0).extrude(BAR_X1 - BAR_X0)
    s = s.cut(cq.Workplane("YZ", origin=(BAR_X0, 0, 0)).pushPoints([(-L / 2, 0), (L / 2, 0)]).circle(1.025).extrude(1.0))
    mid = (A + B) / 2
    s = s.rotate(V(0, 0, 0), V(1, 0, 0), ang).translate((0, float(mid[0]), float(mid[1])))
    return s, L


# ================================================================================================ assembly
def build_all():
    t0 = time.time()
    yoke_env, prox_env, distal_env, middle_env = envelopes()
    yoke, cup_abd = build_yoke()
    prox, cupF, cupE = build_proximal(yoke_env)
    middle, cup_pip = build_middle(prox_env, distal_env)
    distal, tipparts = build_distal(middle_env)
    base = build_base()
    print(f"printed parts built in {time.time() - t0:.0f} s")
    parts = {  # name: (body, workplane, colour key, printed?)
        "base": ("base", base, "base", True), "yoke": ("yoke", yoke, "yoke", True),
        "proximal": ("proximal", prox, "proximal", True), "middle": ("middle", middle, "middle", True),
        "distal": ("distal", distal, "distal", True),
        "cup_abd": ("yoke", cup_abd, "cup", True), "cup_mcp_flex": ("proximal", cupF, "cup", True),
        "cup_mcp_ext": ("proximal", cupE, "cup", True), "cup_pip": ("middle", cup_pip, "cup", True),
    }
    for k, w in tipparts.items():
        parts[k] = ("distal", w, {"tip_skin": "skin", "tip_pcb": "pcb", "tip_chip": "pcb", "tip_magnet": "magnet",
                                  "nail_shim": "metal"}[k], k == "tip_skin")
    for k, (body, w, col) in build_metal().items():
        parts[k] = (body, w, col, False)
    return parts


def posed(parts, q):
    Tb, dip = body_T(q)
    out = {}
    for n, (body, w, col, pr) in parts.items():
        s = w
        if body in ("distal",):
            s = rot_wp(s, (0, 0, ZD), (1, 0, 0), dip)
        if body in ("middle", "distal"):
            s = rot_wp(s, (0, 0, ZP), (1, 0, 0), q["pip"])
        if body in ("proximal", "middle", "distal"):
            s = rot_wp(s, (0, 0, 0), (1, 0, 0), q["mcp"])
        if body != "base":
            s = rot_wp(s, (0, 0, Z_ABD), (0, 1, 0), q["abd"])
        out[n] = (body, s, col, pr)
    bar, _ = bar_shape(q)
    out["dip_bar"] = ("bar", rot_wp(bar, (0, 0, Z_ABD), (0, 1, 0), q["abd"]), "bar", False)
    return out


# designed contacts (press fits, coaxial seats, touching faces): not interference
ALLOWED = {frozenset(p) for p in [
    ("proximal", "pin_A"), ("distal", "pin_B"), ("dip_bar", "pin_A"), ("dip_bar", "pin_B"),
]}


def interference(pp, threshold=0.05):
    names = list(pp)
    bb = {n: pp[n][1].val().BoundingBox() for n in names}
    hits = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            ba, bb_ = pp[a][0], pp[b][0]
            if ba == bb_:
                continue
            A, B = bb[a], bb[b]
            if A.xmax < B.xmin or B.xmax < A.xmin or A.ymax < B.ymin or B.ymax < A.ymin or A.zmax < B.zmin or B.zmax < A.zmin:
                continue
            if frozenset((a, b)) in ALLOWED:
                continue
            try:
                v = pp[a][1].intersect(pp[b][1]).val().Volume()
            except Exception:
                v = 0.0
            if v > threshold:
                hits.append((a, b, round(v, 3)))
    return hits


def strand_meshes(q, r=0.25):
    import trimesh
    ms = {}
    for k in SDEF:
        _, _, P = strand(k, q, with_pts=True)
        ms[k] = P
    for role in ("flex", "ext"):
        _, _, P = abd_strand(role, q, with_pts=True)
        Ta = mat_rot((0, 1, 0), q["abd"], (0, 0, Z_ABD))
        P = P.copy()
        P[:-1] = xf(Ta, P[:-1])
        ms[f"abd_{role}"] = P
    meshes = {}
    for k, P in ms.items():
        segs = []
        for a, b in zip(P[:-1], P[1:]):
            if np.linalg.norm(b - a) < 1e-3:
                continue
            c = trimesh.creation.cylinder(radius=r, segment=[a, b], sections=8)
            segs.append(c)
        meshes["strand_" + k] = trimesh.util.concatenate(segs)
    return meshes, ms


def strands_in_solids(pp, q, ms, step=0.4):
    """sample each strand centreline; report points inside any solid (the strand's own guides excluded: none)."""
    import trimesh
    from tools.render import tessellate
    meshes = {}
    for n, (body, w, col, pr) in pp.items():
        if col in ("pcb", "magnet", "skin"):
            continue
        v, f = tessellate(w, 0.05, 0.2)
        if len(f):
            meshes[n] = trimesh.Trimesh(v, f, process=True)
    out = {}
    for k, P in ms.items():
        pts = []
        for a, b in zip(P[:-1], P[1:]):
            n = max(1, int(np.linalg.norm(b - a) / step))
            pts.extend(a + (b - a) * t for t in np.linspace(0, 1, n, endpoint=False))
        pts = np.array(pts)
        bad = {}
        for n, m in meshes.items():
            lo, hi = m.bounds
            sel = np.all((pts > lo) & (pts < hi), axis=1)
            if not sel.any():
                continue
            ins = m.contains(pts[sel])
            if ins.any():
                bad[n] = int(ins.sum())
        out[k] = bad
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args()
    t0 = time.time()
    checks = {}
    st = strand_checks()
    checks["strands"] = st
    print(json.dumps({k: v for k, v in st.items() if k != "grid"}, indent=1))
    parts = build_all()
    # validity + volumes + export of printed parts
    valid = {}
    mass = {}
    dens = {"base": 1.27, "yoke": 1.27, "proximal": 1.27, "middle": 1.27, "distal": 1.27, "cup": 1.27, "skin": 1.07}
    for n, (body, w, col, pr) in parts.items():
        s = w.val()
        nsol = len(w.solids().vals())
        valid[n] = dict(valid=bool(s.isValid()), solids=nsol, volume_mm3=round(s.Volume(), 1))
        if pr:
            cq.exporters.export(w, str(OUT / f"{n}.stl"), tolerance=0.02, angularTolerance=0.1)
            cq.exporters.export(w, str(OUT / f"{n}.step"))
            mass[n] = round(s.Volume() * dens.get(col, 1.27) / 1000, 2)
    # closed-mesh check of the exported STLs
    import trimesh
    for n in mass:
        m = trimesh.load(str(OUT / f"{n}.stl"))
        valid[n]["stl_watertight"] = bool(m.is_watertight)
    checks["parts"] = valid
    checks["printed_mass_g_100pct"] = mass
    bad = [n for n, v in valid.items() if not v["valid"] or v["solids"] != 1 or v.get("stl_watertight") is False]
    print("parts:", len(valid), "invalid / multi-solid / open:", bad)
    # interference
    poses = []
    abds = [-15, 0, 15]
    for qa in abds:
        for qm in (0, 45, 90):
            for qp in (0, 50, 100):
                poses.append(dict(abd=qa, mcp=qm, pip=qp))
        poses.append(dict(abd=qa, mcp=90, pip=95))  # fist at the real PIP servo limit
    if args.quick:
        poses = [dict(abd=0, mcp=0, pip=0), dict(abd=15, mcp=90, pip=100), dict(abd=-15, mcp=45, pip=50)]
    inter = []
    for q in poses:
        hits = interference(posed(parts, q))
        inter.append(dict(q=q, hits=hits))
        print("pose", q, "hits", hits)
    checks["interference"] = dict(threshold_mm3=0.05, poses=inter, n_poses=len(poses),
                                  clear=sum(1 for r in inter if not r["hits"]))
    # strands vs solids at 3 poses
    sis = {}
    for q in (dict(abd=0, mcp=0, pip=0), dict(abd=15, mcp=45, pip=50), dict(abd=-15, mcp=90, pip=95)):
        pp = posed(parts, q)
        _, ms = strand_meshes(q)
        sis[json.dumps(q)] = strands_in_solids(pp, q, ms)
    checks["strand_centreline_inside_solids"] = sis
    print("strands in solids:", sis)
    checks["dip_link"] = LINK
    (OUT / "checks.json").write_text(json.dumps(checks, indent=1, default=float))
    # strands json (polylines at q = 0)
    sj = {}
    for q in (dict(abd=0, mcp=0, pip=0),):
        _, ms = strand_meshes(q)
        sj = {k: np.round(P, 3).tolist() for k, P in ms.items()}
    (OUT / "strands.json").write_text(json.dumps(sj))
    # assembly export at q = 0
    from tools.export import export_all
    pp0 = posed(parts, dict(abd=0, mcp=0, pip=0))
    asm = {n: v[1] for n, v in pp0.items()}
    cols = {n: COLORS[v[2]] for n, v in pp0.items()}
    sm, _ = strand_meshes(dict(abd=0, mcp=0, pip=0))
    export_all(asm, OUT / "assembly", name="bench_finger_q0", colors=cols, stl=False)
    # GLB with strands too
    if not args.no_render and not args.quick:
        from tools.render import render
        for tag, q, views in (("q0", dict(abd=0, mcp=0, pip=0), ("iso", "front", "side", "back")),
                              ("pinch", dict(abd=0, mcp=30, pip=45), ("iso", "side")),
                              ("fist_abd", dict(abd=15, mcp=90, pip=95), ("iso", "side", "front"))):
            pp = posed(parts, q)
            sm, _ = strand_meshes(q)
            d = {n: v[1] for n, v in pp.items()}
            c = {n: COLORS[v[2]] for n, v in pp.items()}
            for k, m in sm.items():
                d[k] = m
                c[k] = COLORS["strand"]
            render(d, str(OUT / f"bench_finger_{tag}.png"), views=views, colors=c, size=(1600, 1200),
                   title=f"V2.1 bench index finger, abd {q['abd']} / MCP {q['mcp']} / PIP {q['pip']}", edges=True)
        # cutaway through the pip lanes (x < 0.4) at q0
        pp = posed(parts, dict(abd=0, mcp=0, pip=0))
        cut = box(0.4, 30, -40, 40, -60, 100)
        d, c = {}, {}
        for n, v in pp.items():
            try:
                w = v[1].cut(cut)
                if w.vals():
                    d[n], c[n] = w, COLORS[v[2]]
            except Exception:
                pass
        sm, _ = strand_meshes(dict(abd=0, mcp=0, pip=0))
        for k, m in sm.items():
            d[k] = m
            c[k] = COLORS["strand"]
        render(d, str(OUT / "bench_finger_cut.png"), views=("side",), colors=c, size=(1400, 1400),
               title="cut at x = 0.4 (pip strands, idler, redirect)", edges=True)
    write_bom()
    print(f"done in {time.time() - t0:.0f} s")


def write_bom():
    rows = [
        ("base (bench block)", 1, "PETG, print on the dorsal face, 30 % gyroid, 4 walls", "any", 0.6),
        ("yoke", 1, "PETG (PA-CF for HLS runs), on its side (x up)", "any", 0.1),
        ("proximal", 1, "PETG (PA-CF for HLS runs), on its -x face", "any", 0.15),
        ("middle", 1, "PETG (PA-CF for HLS runs), on its -x face", "any", 0.1),
        ("distal", 1, "PETG, on its -x face", "any", 0.05),
        ("fuse cups (abd, PIP 2-lane; MCP flex, MCP ext 1-lane)", 8, "PETG; flat = 68 N trip (SCS), on side = 170 N (HLS); print spares", "any", 0.05),
        ("tip skin", 2, "Dragon Skin 20 / Smooth-Sil 940, cast in the 05 mould (scaled)", "Smooth-On kit", 1.2),
        ("MR84ZZ 4x8x3", 4, "MCP lugs 2, MCP idler 1, PIP +x 1", "AliExpress/Amazon 10-pack", 3.0),
        ("MR63ZZ 3x6x2.5", 2, "abd, in the base lugs", "10-pack", 1.5),
        ("stainless tube 4.0 x 0.5 (ID 3)", 3, "MCP axle 14.6, PIP pin 12.3, DIP pin 13.4 mm (cut, deburr, radial wire slot)", "model shop / Amazon 300 mm", 3.0),
        ("brass tube 5.0 x 0.5 (ID 4)", 5, "tongue sleeves 2 x 2.9 mm, PIP -x bush 3.0, DIP bushes 2 x 3.0", "K&S / Albion 300 mm", 2.5),
        ("brass tube 14 x 0.5 (ID 13)", 4, "drum rings: MCP 2 x 1.6 mm (C, gap at the cup), PIP 3.0, abd 3.0", "model shop 300 mm", 4.0),
        ("stainless tube 3.0 x 0.5", 1, "palmar abd stub, hollow (wires), 6.8 mm", "300 mm", 1.5),
        ("dowel pin Ø3 x 8 ISO 8734", 3, "redirect + 2 abd-plane pip dowels (yoke)", "M3 dowel assortment", 1.0),
        ("dowel pin Ø3 x 4 / x 6 ISO 8734", 3, "MCP ear dowels 2 x 4.2 (cut from x 6), dorsal abd stub 6.5", "assortment", 0.5),
        ("dowel pin Ø3 x 33", 4, "base guide dowels (or Ø3 silver steel rod cut to length)", "Ø3 silver steel 330 mm", 2.0),
        ("dowel pin Ø2 x 6 ISO 8734", 2, "DIP bar pins A and B (4 mm deep)", "assortment", 0.3),
        ("push-on star washer Ø2", 2, "keeps the bar on the pins", "assortment", 0.3),
        ("stainless strip 0.8 x 3.2, 25 mm", 1, "DIP bar, 2 x Ø2.05 holes 21.55 apart (laser-cut or drill + file)", "laser service / 0.8 sheet", 1.0),
        ("PE#8 UHMWPE 8-strand braid", 1, "~0.47 mm, ~445 N; 6 strands x ~0.4 m + spares; pre-stretch 24 h at 30 %", "fishing shop, 100 m", 12.0),
        ("aluminium single-barrel crimp sleeves 1.3 mm", 12, "beads in the fuse cups (line doubled through)", "100-pack", 3.0),
        ("diametric magnet Ø3 x 2 (N45)", 2, "in the MCP axle and PIP pin ends", "AliExpress", 1.0),
        ("axial magnet Ø3 x 1.5 (N45)", 1, "tip skin (05)", "AliExpress", 0.5),
        ("TMAG5273 (or AS5600) on 4.6 mm square PCB", 2, "MCP and PIP joint angle", "JLCPCB / breakout", 6.0),
        ("MLX90393 on 9.2 x 9.0 PCB", 1, "tip tactile (05)", "JLCPCB", 5.0),
        ("36 AWG silicone wire", 4, "I2C bus through the hollow pins, 0.4 m each", "1 m packs", 3.0),
        ("stainless shim 0.15 x 9 x 6.5", 1, "nail / scoop", "shim stock", 0.3),
        ("M5 x 12 + T-nut (2020)", 2, "base to 2020 extrusion", "any", 0.8),
        ("M3 x 12 + nut", 4, "alternative plate mount", "any", 0.3),
        ("retaining compound (Loctite 641 or 603)", 1, "rings, sleeves, axle, pins", "hardware store", 0.0),
    ]
    with open(OUT / "bom.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["part", "qty", "material / spec", "where to buy (generic)", "cost EUR (est.)"])
        for r in rows:
            w.writerow(r)
        w.writerow(["TOTAL (one bench finger, without servos)", "", "", "", round(sum(r[4] for r in rows), 1)])


if __name__ == "__main__":
    main()
