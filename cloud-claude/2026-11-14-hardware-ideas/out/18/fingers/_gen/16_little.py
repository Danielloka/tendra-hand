"""16 · Tendra V2.1 bench index finger with sensors (R3 slim layout of reports/15-sensing-plan.md), agent 07,
2026-11-15. Copy of 12_finger.py; sensor changes are marked "# S". Outputs in out/16/ (12 and 07 kept).

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

HERE = Path('/home/user/tendra-hand/cloud-claude/2026-11-14-hardware-ideas/cad')
STUDY = HERE.parent
OUT = STUDY / "out" / "16"
OUT12 = STUDY / "out" / "12"
OUT15 = STUDY / "out" / "15"
OUT07 = STUDY / "out" / "07"
OUT.mkdir(parents=True, exist_ok=True)
V2CAD = STUDY.parent / "2026-11-09-v2-hand-cad" / "cad"
sys.path.insert(0, str(V2CAD))
sys.path.insert(0, str(STUDY.parents[1] / "hardware" / "cad"))
import dip_linkage as DL  # noqa: E402

BUD = json.loads((OUT12 / "budget.json").read_text())  # 12_budget.py
BUD07 = json.loads((OUT07 / "budget.json").read_text())  # DIP link (unchanged)
D2R = math.pi / 180

# ================================================================================================ parameters
L1, L2, L3 = 34.0, 20.0, 21.0
TS = 0.0
ZP, ZD, ZT = L1, L1 + L2, L1 + L2 + L3
Z_ABD = -13.0  # abd axis 13 mm below the MCP (11.5 in V2: the palmar abd support then hits the proximal at MCP 90)
LINE_R = 0.235  # PE#8, d 0.47
R = BUD["R"]  # 7.235 line centre on a 14 x 0.5 brass ring
RING_RO, RING_RI = 7.0, 6.5
FL_R = 7.5  # flange: 0.5 proud of the ring
R_I = BUD["RHO"]  # R2: 3.235 on the MR63 idler (Ø3 inner axle tube)
RD = 1.5 + LINE_R  # line centre on a Ø3 dowel
X_MCP = 6.0  # R2: MCP lanes +-6.0 (room for the Ø1.5 nip pins in a 1.6 mm ear wall)
X_PIP_I = 0.9  # R2: pip lanes on the 2.5 mm MR63 race: flex -0.9, ext +0.9
X_RING_P = 0.8  # PIP ring centre; R2 lanes flex 0.1, ext 1.5 (1.5 deg fleet from the idler lanes)
PIP_LANE = 0.7
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
MCP_SEAT = (4.4, 7.4)  # -x: MR84ZZ (steel, no sensor)
MCP_SEAT_P = (4.4, 6.4)  # S: +x open ZrO2 MR84 4x8x2
MCP_RING = (5.2, 6.8)  # R2
TONGUE = 4.2
IDLER_SLOT = 1.7
AXLE = dict(od=3.0, id=2.0, x0=-7.4, x1=6.4)  # S: ends flush at the +x bearing face  # R2: inner Ø3x0.5 tube (wires); Ø4x0.5 sleeves under the MR84s
AXSL = dict(od=4.0, id=3.0, x=(1.45, 7.4))  # R2: outer sleeves, also the idler/bearing spacers
IDL_W = 2.5
MR63_ = dict(id=3.0, od=6.0, w=2.5)
ZRO2 = dict(id=4.0, od=8.0, w=2.0)
SLEEVE = dict(od=5.0, id=4.0, x=(1.7, 4.4))  # brass sleeves in the tongue cheeks (over the Ø4 sleeves)
# PIP x-stations
PIP_LUGN = (-6.85, -2.65)  # -x lug: outer wire layer -6.85..-5.65, Ø5 bush -5.65..-2.65
PIP_BUSH = (-5.65, -2.65)
PIP_HUB = (-2.45, 4.05)
PIP_RING = (-0.7, 2.3)
PIP_LUGP = (4.25, 8.05)
PIP_SEAT = (4.25, 6.25)  # S: open ZrO2 MR84 4x8x2 (sensor side)
PIP_PIN = (-5.65, 6.25)  # S: flush with the bearing outer face
# DIP x-stations
DIP_CHN = (-6.85, -2.85)
DIP_BUSHN = (-5.85, -2.85)
DIP_TONG = (-2.65, 4.3)
DIP_CHP = (4.5, 7.5)
DIP_PIN = (-5.85, 6.0)  # S: shortened, magnet in its +x end
DIP_BUSHP = (4.5, 6.0)
# redirect / guide dowels (yoke) and anchors
# R2: pip flex redirect = MR63 roller dorsal-below the idler: the flex leaves the idler bottom rising DELTA deg and
# wraps the roller over its top; pre-wrap on the idler 90 + DELTA - 5.5 >= 104 deg, >= 14 deg left at MCP 90
DELTA, L_CROSS = 20.0, 3.4
_h = np.array([math.cos(DELTA * D2R), math.sin(DELTA * D2R)])
_n = np.array([_h[1], -_h[0]])
_p = R_I * _n
ROLLER = tuple(_p + L_CROSS * _h + R_I * _n)  # (y, z) centre of the MR63 roller, x lane -0.9
ROLLER_X = (-X_PIP_I - 1.25, -X_PIP_I + 1.25)
# R2: pip ext (dorsal side of the idler) is pre-wrapped 15 deg (so it still touches at MCP -10) by a Ø3 dowel below,
# cantilevered from the +x cheek (x 0.35..4.0); the ext only carries extension force + pretension
BETA, L_EXT = 20.5, 8.0
_he = np.array([-math.sin(BETA * D2R), -math.cos(BETA * D2R)])
_ne = np.array([_he[1], -_he[0]])
EXT_RD = tuple(-R_I * _ne + L_EXT * _he - RD * _ne)
EXT_RD_X = (0.35, 4.0)
# R2 abd crossing: steel x-nips (pins along Y) exactly on the abd plane; the strands run straight down to the exit
NIP_D = {"pip": 2.0, "mcp": 1.5}
PHI_MCP = 50.0  # MCP anchors (deg from +y toward +z), both lugs
CUP_DPHI = 14.0  # R2: separate single-lane cups, flex at phi + 14, ext at phi - 14
Z_BOT = -54.0
CRIMP = dict(w=2.6, h=1.1, l=2.0)  # 2 x 2 mm crimp tube crushed flat (measure yours)
CUP = dict(lw=3.5, lt=2.7, h=2.9, ch_w=2.9, ch_t=2.1, lip=0.6)  # lip 0.6 flat = SCS (67 N); 0.8 on its side = HLS (148 N)
SPEC = json.loads((OUT15 / "sensor_spec.json").read_text())
SENS = {d["id"].split("_", 1)[1]: d for d in SPEC["sensors"]}
PAD_PROX = dict(x=(1.6, 7.0), z=(round(0.405 * L1, 2), round(0.405 * L1 + 10.0, 2)), floor_y=-3.74)  # S: +x half (the pip flex strand runs 2.5-4.5 mm under the palm face)
PAD_MID = dict(x=(-4.0, 4.0), z=(L1 + 9.6, min(L1 + 15.8, L1 + L2 - 8.2)), floor_y=-3.25)
PAD_MID = PAD_MID if PAD_MID['z'][1] - PAD_MID['z'][0] >= 6.0 else None  # no room on a short middle phalanx  # S: 8 x 6.2 x 4 (10 long would break into the PIP ring slot / DIP cheek sweep)
CB = 6.75  # counterbore for the Ø6.70 joint caps
WIRE_HOLE_X = {"mcp": 3.0, "pip": 3.4, "dip": 3.2}  # S: Ø1.2 radial holes along +y (pip 3.4 instead of 3.6: hub face 4.05)
# (12's Hall stations, no longer used): race outer face, magnet (Ø3 x 1.5 diametric) 2.0 mm proud, 0.3 floor, 0.5 air gap, chip 1.0, PCB 0.8
HALL = {}
for _k, _race in (("mcp", 7.4), ("pip", 7.25)):
    _m0 = _race + 2.0
    HALL[_k] = dict(x_race=_race, x_mag=(_m0, _m0 + 1.5), x_lip=_race, x_floor=_m0 + 1.5 + 0.2,
                    x_chip=(_m0 + 1.5 + 0.5, _m0 + 1.5 + 1.5), x_out=_m0 + 1.5 + 2.3)
LINK = {"pin_a": {"r": 5.1, "deg": 23.968749999999996}, "pin_b": {"r": 6.6375, "deg": -95.75}, "bar": 17.04761005407403, "err_deg": 0.42964314137148196, "L2": 20.0, "clear": {"axis_clear_mm": 3.3297290690565897, "min_transmission_deg": 32.84190260624418}}

COLORS = {"base": "#8c8c8c", "yoke": "#4f86c6", "proximal": "#e08a3c", "middle": "#5bab6b", "distal": "#c9ae43",
          "skin": "#d05050", "metal": "#b8bcc4", "brass": "#c8a040", "bearing": "#6e7480", "cup": "#8e6fbf",
          "pcb": "#2e7d32", "magnet": "#555566", "bar": "#a0a8b8", "strand": "#202020", "ceramic": "#f2f0e6",
          "chip": "#111111", "sensorcap": "#7a8aa0", "connector": "#f5f5f5", "wire": "#d62728"}


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


def single_cup(phi, x_c, lane_off, axis_center=(0.0, 0.0), r_top=RING_RI, axis="x", t_lip=CUP["lip"]):
    """R2: radial single-lane fuse cup for a 2 x 2 mm crimp tube (crushed 2.6 x 1.1): chamber 2.9 x 2.1, lip t_lip,
    strand hole Ø0.8 offset lane_off from the cup centre. Returns (cup, pocket)."""
    a = math.radians(phi)
    if axis == "x":
        nrm, org, xdir = (0.0, math.cos(a), math.sin(a)), (x_c, axis_center[0], axis_center[1]), (1, 0, 0)
    else:
        nrm, org, xdir = (math.cos(a), 0.0, math.sin(a)), (0.0, x_c, axis_center[1]), (0, 1, 0)
    pl = cq.Plane(origin=V(*org), xDir=V(*xdir), normal=V(*nrm))
    h = CUP["h"]
    cup = cq.Workplane(pl).workplane(offset=r_top - h).slot2D(CUP["lw"], CUP["lt"], 0).extrude(h)
    ch = cq.Workplane(pl).workplane(offset=r_top - h - 0.1).rect(CUP["ch_w"], CUP["ch_t"]).extrude(h - t_lip + 0.1)
    hl = cq.Workplane(pl).workplane(offset=r_top - t_lip - 0.1).center(lane_off, 0).circle(0.4).extrude(t_lip + 0.2)
    cup = cup.cut(ch).cut(hl)
    pocket = cq.Workplane(pl).workplane(offset=r_top - h).slot2D(CUP["lw"] + 0.2, CUP["lt"] + 0.2, 0).extrude(h + 0.6)
    pocket = pocket.union(cq.Workplane(pl).workplane(offset=r_top - 0.2).center(lane_off, 0).circle(0.5).extrude(1.5))
    return cup, pocket


def tangential_cup(phi, x_lane, wrap_sign, t_lip=CUP["lip"], open_out=3.5):
    """R2: MCP single-lane cup on the tangent that continues the wrap beyond the anchor; 3.5 (radial) x 2.7 (x)
    outside, chamber 2.9 x 2.1 for the crushed crimp, length 2.9 along the tangent."""
    a = math.radians(phi)
    A = np.array([x_lane, R * math.cos(a), R * math.sin(a)])
    t = wrap_sign * np.array([0.0, math.sin(a), -math.cos(a)])
    rad = np.array([0.0, math.cos(a), math.sin(a)])
    p0 = A + 0.3 * t
    L = CUP["h"]

    def blk(c0, length, w_rad, w_x):
        pl = cq.Plane(origin=V(*c0), xDir=V(1, 0, 0), normal=V(*t))
        return cq.Workplane(pl).rect(w_x, w_rad).extrude(length)
    cup = blk(p0, L, CUP["lw"], CUP["lt"]).cut(blk(p0 + t_lip * t, L, CUP["ch_w"], CUP["ch_t"])).cut(
        cyl(0.4, p0 - 0.1 * t, p0 + (t_lip + 0.1) * t))
    pocket = blk(p0, L + 0.1, CUP["lw"] + 0.2, CUP["lt"] + 0.2)
    pocket = pocket.union(blk(p0 + open_out * rad, L + 0.1, CUP["lw"] + 0.2, CUP["lt"] + 0.2))
    q = [p0, p0 + (L + 0.1) * t, p0 + (L + 0.1) * t + open_out * rad, p0 + open_out * rad]
    pocket = pocket.union(prism_yz([(float(v[1]), float(v[2])) for v in q], x_lane - CUP["lt"] / 2 - 0.1, x_lane + CUP["lt"] / 2 + 0.1))
    pocket = pocket.union(cyl(0.5, A - 0.2 * t, p0 + 0.5 * t))
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
def strand_defs():
    """R2: 2D (y, z) circles per strand in the YOKE frame at abd = 0. pip flex: PIP drum (ccw over the palm side),
    idler PALM side (ccw, open belt), roller (cw), down to its nip. pip ext: PIP drum (cw over the back), idler
    DORSAL side (cw), down. MCP: drum, down. Every strand reaches the abd plane vertically and passes a steel x-nip."""
    C0 = np.array([0.0, 0.0])
    Cp = np.array([0.0, ZP])
    d = {}
    d["pip_flex"] = dict(x=(X_RING_P - PIP_LANE, -X_PIP_I), drum=(Cp, +R, "middle"),
                         elems=[(C0, +R_I, "yoke"), (np.array(ROLLER), -R_I, "yoke")])
    d["pip_ext"] = dict(x=(X_RING_P + PIP_LANE, X_PIP_I), drum=(Cp, -R, "middle"),
                        elems=[(C0, -R_I, "yoke"), (np.array(EXT_RD), +RD, "yoke")])
    d["mcp_flex"] = dict(x=(-X_MCP, -X_MCP), drum=(C0, +R, "proximal"), elems=[])
    d["mcp_ext"] = dict(x=(X_MCP, X_MCP), drum=(C0, -R, "proximal"), elems=[])
    for k, v in d.items():
        lc, lr = (v["elems"][-1][0], v["elems"][-1][1]) if v["elems"] else (v["drum"][0], v["drum"][1])
        v["nip"] = np.array([lc[0] - lr, Z_ABD])  # vertical leaving point -> nip on the abd plane
    return d


SDEF = strand_defs()


def pip_anchor_phi():
    th = {}
    for k in ("pip_flex", "pip_ext"):
        sd = SDEF[k]
        c, r, _ = sd["drum"]
        nc, nr, _ = sd["elems"][0]
        p1, _ = tangent(c, r, nc, nr)
        th[k] = math.degrees(math.atan2(p1[1] - c[1], p1[0] - c[0]))
    tf = th["pip_flex"] % 360
    te = (th["pip_ext"] + 180) % 360 - 180
    lo, hi = RANGE["pip"]
    phi = (tf + te - hi - lo) / 2
    return phi, tf - (phi + CUP_DPHI) - hi


PHI_PIP, PIP_MARGIN = pip_anchor_phi()
SDEF["pip_flex"]["phi"], SDEF["pip_ext"]["phi"] = PHI_PIP + CUP_DPHI, PHI_PIP - CUP_DPHI
SDEF["mcp_flex"]["phi"] = SDEF["mcp_ext"]["phi"] = PHI_MCP


def _path(nodes):
    tans = [tangent(nodes[i][0], nodes[i][1], nodes[i + 1][0], nodes[i + 1][1]) for i in range(len(nodes) - 1)]
    return tans


def strand(name, q, with_pts=False):
    """exact belt length from the drum anchor to the base exit; abd applied in 3D to the nip -> exit segment.
    An element the strand no longer touches (wrap would be negative) is dropped (wrap 0)."""
    s = SDEF[name]
    c, r, body = s["drum"]
    mcp, pip = q["mcp"], q["pip"]
    if body == "middle":
        cc = rot_about(c, (0, 0), mcp)
        phi = s["phi"] + pip + mcp
    else:
        cc = np.asarray(c, float)
        phi = s["phi"] + mcp
    anchor = cc + abs(r) * np.array([math.cos(phi * D2R), math.sin(phi * D2R)])
    N = s["nip"]
    elems = [(np.asarray(e[0], float), e[1]) for e in s["elems"]]
    active = list(range(len(elems)))
    while True:
        nodes = [(cc, r)] + [elems[i] for i in active] + [(N, 0.0)]
        tans = _path(nodes)
        wr = []
        bad = None
        for k in range(1, len(nodes) - 1):
            _, ang, _ = arc_len(nodes[k][0], nodes[k][1], tans[k - 1][1], tans[k][0])
            wr.append(ang)
            if ang > 300:
                bad = active[k - 1]
        if bad is None:
            break
        active.remove(bad)
    L = 0.0
    pts = [anchor]
    la, ang0, ap = arc_len(cc, r, anchor, tans[0][0])
    L += la
    pts.extend(ap)
    wraps = [ang0] + [0.0] * len(elems)
    for k, (p1, p2) in enumerate(tans):
        L += float(np.linalg.norm(p2 - p1))
        pts.extend([p1, p2])
        if k + 1 < len(nodes) - 1:
            la, ang, ap = arc_len(nodes[k + 1][0], nodes[k + 1][1], p2, tans[k + 1][0])
            L += la
            wraps[1 + active[k]] = ang
            pts.extend(ap)
    xs = s["x"][1]
    E = np.array([xs, N[0], N[1]])
    F = np.array([xs, N[0], Z_BOT])
    Ta = mat_rot((0, 1, 0), q["abd"], (0, 0, Z_ABD))
    L += float(np.linalg.norm(xf(Ta, E)[0] - F))
    if not with_pts:
        return L, wraps
    pts = np.array(pts)
    x_line = np.interp(pts[:, 1], [0.0, ZP], [s["x"][1], s["x"][0]]) if body == "middle" else np.full(len(pts), xs)
    P3 = np.column_stack([x_line, pts[:, 0], pts[:, 1]])
    P3 = xf(Ta, P3)
    P3 = np.vstack([P3, F])
    return L, wraps, P3


def abd_strand(role, q, with_pts=False):
    """abd ring about Y at (x, z) = (0, Z_ABD); flex (+x side, lane y 10.2) / ext (-x side, lane y 12.2)."""
    sgn = 1 if role == "flex" else -1
    y = Y_ABD - 1.0 if role == "flex" else Y_ABD + 1.0
    th_a = (90.0 - CUP_DPHI if role == "flex" else 90.0 + CUP_DPHI) - q["abd"]  # R2: one cup per strand
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
    res["pip_anchor_deg"] = dict(flex=PHI_PIP + CUP_DPHI, ext=PHI_PIP - CUP_DPHI)
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


YW = [(3.0, 0.0, 0.0), (3.0, 2.6, 0.0), (3.0, 8.0, -4.0), (3.0, 8.0, -3.0)]  # S: yoke wire channel (over the roller dowel, out of the column top)


def nip_pins():
    """R2: steel x-guides on the abd plane. name: (x, y0, y1, d). pip pins are cantilevers pressed 2 mm into the slot
    end walls; MCP pins run through the ears."""
    yfl = SDEF["pip_flex"]["nip"][0]
    yex = SDEF["pip_ext"]["nip"][0]
    gp = NIP_D["pip"] / 2 + LINE_R + 0.05
    gm = NIP_D["mcp"] / 2 + LINE_R + 0.05
    n = {}
    for i, sx in enumerate((-1, 1)):
        n[f"nip_pipflex_{i}"] = (-X_PIP_I + sx * gp, yfl - 2.0, yfl + 0.5 + 2.0, NIP_D["pip"])
        n[f"nip_pipext_{i}"] = (X_PIP_I + sx * gp, yex - 0.7 - 2.0, yex + 1.8, NIP_D["pip"])
        n[f"nip_mcpflex_{i}"] = (-X_MCP + sx * gm, -7.9, -1.5, NIP_D["mcp"])
        n[f"nip_mcpext_{i}"] = (X_MCP + sx * gm, 1.5, 7.9, NIP_D["mcp"])
    return n


def build_yoke():
    """abd child / MCP parent: tongue with the axle and idler, pip slot, dowels, MCP ears, abd disc, stubs."""
    tongue = U(cylx(6.0, -TONGUE, TONGUE), box(-TONGUE, TONGUE, -6.0, 6.0, -9.5, 0.0),
               box(-TONGUE, TONGUE, 0.0, 9.4, -9.5, -3.6))
    body = U(box(-5.0, 5.0, -8.0, 8.4, Z_ABD - 4.0, -9.0), box(-5.0, 5.0, -12.6, -8.0, Z_ABD - 4.6, Z_ABD + 4.6))
    ears = U(box(-9.4, -4.8, -7.9, -1.5, Z_ABD - 2.5, Z_ABD + 2.5), box(4.8, 9.4, 1.5, 7.9, Z_ABD - 2.5, Z_ABD + 2.5))
    disc = cyly(FL_R, 8.4, 12.7, 0.0, Z_ABD).union(cyly(RING_RI, 12.7, 14.0, 0.0, Z_ABD))
    y = U(tongue, body, ears, disc)
    # abd ring groove (ring slides on from the +y side; flange on the yoke side)
    y = y.cut(cyly(FL_R + 1, 9.7, 12.7, 0.0, Z_ABD).cut(cyly(RING_RI, 9.0, 13.0, 0.0, Z_ABD)))
    # R2 pip slot |x| < 1.7: idler, flex crossing under the idler to the roller, both verticals, exit to the top
    yfl = SDEF["pip_flex"]["nip"][0]  # flex vertical (dorsal side of the roller)
    yex = SDEF["pip_ext"]["nip"][0]  # ext vertical (dorsal side of the idler)
    slot_yz = [
        cylx(R_I + 0.65, -IDLER_SLOT, IDLER_SLOT),
        cylx(R_I + 0.45, ROLLER_X[0] - 0.3, ROLLER_X[1] + 0.3, *ROLLER),  # roller pocket (-x side widened)
        box(-IDLER_SLOT, IDLER_SLOT, yex - 0.7, yfl + 0.5, Z_ABD - 5.0, 0.0),
        box(-IDLER_SLOT, IDLER_SLOT, -R_I - 0.65, R_I + 0.65, 0.0, 6.5),
    ]
    for c in slot_yz:
        y = y.cut(c)
    # strands below the nips swing with abduction (pendulum about the nip, which is ON the abd plane): flared slots
    fl = math.tan(17 * D2R)
    dz = 8.6  # R2: to below the abd disc (its rim reaches Z_ABD - 7.5)
    vflare = cq.Workplane("XZ").polyline([(-IDLER_SLOT, Z_ABD), (IDLER_SLOT, Z_ABD),
                                          (IDLER_SLOT + dz * fl, Z_ABD - dz), (-IDLER_SLOT - dz * fl, Z_ABD - dz)]).close()
    y = y.cut(vflare.extrude(-(yfl + 0.5 - (yex - 0.7))).translate((0, yex - 0.7, 0)))
    for sx in (-1, 1):
        xc = sx * X_MCP
        y0, y1 = sorted((sx * (R - 0.7), sx * 8.4))
        ef = cq.Workplane("XZ").polyline([(xc - 0.3, Z_ABD), (xc + 0.3, Z_ABD), (xc + 0.3 + dz * fl, Z_ABD - dz),
                                          (xc - 0.3 - dz * fl, Z_ABD - dz)]).close()
        y = y.cut(ef.extrude(-(y1 - y0)).translate((0, y0, 0)))
        # ear slot above the nip (strand enters from the drum)
        y = y.cut(box(xc - 0.3, xc + 0.3, y0, y1, Z_ABD, Z_ABD + 3.0))
    # the pip strands leave the idler toward the PIP drum; that line turns with the MCP: swept exit band
    for k in ("pip_flex", "pip_ext"):
        sd = SDEF[k]
        p_drum, p_idl = tangent(sd["drum"][0], sd["drum"][1], sd["elems"][0][0], sd["elems"][0][1])
        d = (p_drum - p_idl) / np.linalg.norm(p_drum - p_idl)
        nrm = np.array([-d[1], d[0]])
        q4 = [p_idl - 0.6 * nrm - 0.5 * d, p_idl + 12 * d - 0.6 * nrm, p_idl + 12 * d + 0.6 * nrm, p_idl + 0.6 * nrm - 0.5 * d]
        band = prism_yz(q4, -IDLER_SLOT, IDLER_SLOT)
        y = y.cut(swept(band, (0, 0, 0), (1, 0, 0), angles(*RANGE["mcp"], step=4.0)))
    # nested axle: Ø4 sleeves through the cheeks inside the brass sleeves
    y = y.cut(cylx(AXSL["od"] / 2, -TONGUE - 1, TONGUE + 1))
    for sx in (-1, 1):
        y = y.cut(cylx(SLEEVE["od"] / 2, sx * SLEEVE["x"][0], sx * (TONGUE + 0.5)))
    # roller dowel (Ø3, through both cheeks) + ext redirect dowel (cantilever from +x)
    y = y.cut(cylx(1.5, -4.0, 4.0, *ROLLER)).cut(cylx(1.5, EXT_RD_X[0], EXT_RD_X[1], *EXT_RD))
    # R2 x-nips on the abd plane: pins along Y on both sides of each strand
    NIPS = nip_pins()
    for nm, (x, y0, y1, dpin) in NIPS.items():
        y = y.cut(cyly(dpin / 2, y0, y1, x, Z_ABD))
    # abd stubs: palmar MR63 seat; dorsal stub starts at y 11.5 (clear of the pip flex nip pins)
    y = y.cut(cyly(SEAT63 / 2, -12.7, -10.1, 0.0, Z_ABD)).cut(cyly(2.4, -10.2, -8.0, 0.0, Z_ABD))
    y = y.cut(cyly(1.5, 11.5, 14.5, 0.0, Z_ABD))
    # abd fuse cups: one per strand at 90 -+ 14 (flex lane y 10.2, ext lane 12.2)
    cupF, pocF = single_cup(90.0 - CUP_DPHI, Y_ABD, -1.0, axis_center=(0.0, Z_ABD), axis="y")
    cupE, pocE = single_cup(90.0 + CUP_DPHI, Y_ABD, +1.0, axis_center=(0.0, Z_ABD), axis="y")
    y = y.cut(pocF).cut(pocE)
    # S wire path: Ø1.2 radial hole at x +3.0 along +y (neutral axis) -> Ø1.4 channel in the +x cheek, between the
    # roller dowel and the ext dowel, to the dorsal face of the tongue column -> free loop to the knuckle board
    for a, b in ((YW[0], YW[1]), (YW[1], YW[2]), (YW[2], YW[3])):
        a, b = np.asarray(a, float), np.asarray(b, float)
        dd = (b - a) / np.linalg.norm(b - a)
        y = y.cut(cyl(0.7, a - 0.5 * dd, b + 0.5 * dd))  # overlapping ends instead of spheres (clean mesh)
    cup = (cupF, cupE)
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
    # S: no Hall bosses (R3 slim: the sensor stack sits in the Ø6.75 counterbore)
    p = U(*lugs, shaft, hood, root, lugN, lugP)
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
    p = p.cut(cylx(SEAT84 / 2, -MCP_SEAT[1], -MCP_SEAT[0])).cut(cylx(6.75 / 2, -MCP_LUG[1] - 1, -MCP_SEAT[1]))
    p = p.cut(cylx(SEAT84 / 2, *MCP_SEAT_P)).cut(cylx(CB / 2, MCP_SEAT_P[1], MCP_LUG[1] + 1))  # S: ZrO2 + counterbore
    # PIP: gap for the middle hub, bush + seat + lip + Hall pocket, pin A
    p = p.cut(cylx(FL_R + 0.4, PIP_LUGN[1], PIP_LUGP[0], 0, ZP))
    p = p.cut(cylx(2.5, PIP_BUSH[0], PIP_BUSH[1], 0, ZP)).cut(cylx(0.8, PIP_LUGN[0] - 1, PIP_BUSH[0] + 0.1, 0, ZP))
    p = p.cut(cylx(SEAT84 / 2, PIP_SEAT[0], PIP_SEAT[1], 0, ZP)).cut(cylx(CB / 2, PIP_SEAT[1], PIP_LUGP[1] + 2, 0, ZP))
    A0 = link_pins(0, 0)[2]
    p = p.cut(cylx(1.0, PIP_LUGN[0] - 1, PIP_LUGN[0] + 4.0, A0[0], A0[1]))
    # bar recess: -x face at X_BARFACE for z > 30
    p = p.cut(box(-12, X_BARFACE, -12, 12, ZP - 12.0, ZP + 12))
    # pip strand grooves, open toward +x (lay the strand in from the side)
    # pip strand grooves (lay-in). S: the flex groove opens toward -x for z < 30 so the +x half under the palm face
    # stays solid for the pad pocket; beyond z 30 it opens toward +x as before
    for k in ("pip_flex", "pip_ext"):
        sd = SDEF[k]
        p_drum, p_idl = tangent(sd["drum"][0], sd["drum"][1], sd["elems"][0][0], sd["elems"][0][1])
        a, b = np.asarray(p_idl), np.asarray(p_drum)
        d = (b - a) / np.linalg.norm(b - a)
        nrm = np.array([-d[1], d[0]])
        def band(pa, pb, e0, e1):
            return [pa + 0.7 * nrm - e0 * d, pb + 0.7 * nrm + e1 * d, pb - 0.7 * nrm + e1 * d, pa - 0.7 * nrm - e0 * d]
        if k == "pip_ext":
            p = p.cut(prism_yz(band(a, b, 2, 6), 0.2, 12.0))
        else:
            t30 = (PAD_PROX['z'][1] + 4.0 - a[1]) / d[1]
            t29 = (PAD_PROX['z'][1] + 2.0 - a[1]) / d[1]
            p = p.cut(prism_yz(band(a, a + t30 * d, 2, 0), -12.0, 0.8))
            p = p.cut(prism_yz(band(a + t29 * d, b, 0, 6), -1.6, 12.0))
    # S pad pocket (BMP581) on the proximal palm face, +x half: 5.4 (x) x 10 (z) x 4 (y)
    p = p.cut(box(*PAD_PROX["x"], -12.0, PAD_PROX["floor_y"], *PAD_PROX["z"]))
    # S wire grooves 1.2 x 1.0 (route: PIP -x lug -> dorsal z (ZP - 3.6)-(ZP - 2.4) -> +x -> pip cap -> +x face y 2.4-3.6 -> mcp cap
    # -> +x lug face up -> dorsal hood groove z 10.4-11.6 -> -x lug face down -> -x lug counterbore -> MCP axle -x end)
    HHz = lambda z: HH_P0 + (HH_P1 - HH_P0) * z / ZP  # noqa: E731
    for g in (box(-6.95, -5.85, 0.0, 8.2, ZP - 0.6, ZP + 0.6), box(-6.95, -5.85, HHz(ZP - 3.0) - 1.0, 8.2, (ZP - 3.6), ZP + 0.6),
              box(-6.95, 8.4, HHz(ZP - 3.0) - 1.0, 8.2, (ZP - 3.6), (ZP - 2.4)), box(7.05, 8.4, 2.4, 8.2, (ZP - 3.6), (ZP - 2.4)),
              box(8.0, 9.8, 0.0, 8.8, -0.6, 0.6), box(8.0, 9.8, HHz(11) - 1.0, 8.8, -0.6, 11.6),
              box(-9.8, 9.8, HHz(11) - 1.0, 9.0, 10.4, 11.6), box(-9.8, -8.0, HHz(11) - 1.0, 8.8, -0.6, 11.6),
              box(-9.8, -8.0, 0.0, 8.8, -0.6, 0.6)):
        p = p.cut(g)
    xz = [(hw_prox(z) - 1.0, z) for z in (0.0, ZP)] + [(12.0, ZP), (12.0, 0.0)]
    p = p.cut(cq.Workplane("XZ").polyline(xz).close().extrude(-1.2).translate((0, 2.4, 0)))
    # MCP fuse cups (tangential): flex on -x lug (wrap continues with phi decreasing), ext on +x lug
    cupF, pocF = tangential_cup(PHI_MCP, -X_MCP, +1)
    cupE, pocE = tangential_cup(PHI_MCP, X_MCP, -1)  # R2: resized for the crushed 2 x 2 crimp
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
    cupF, pocF = single_cup(PHI_PIP + CUP_DPHI, X_RING_P, -PIP_LANE, axis_center=(0.0, ZP))
    cupE, pocE = single_cup(PHI_PIP - CUP_DPHI, X_RING_P, +PIP_LANE, axis_center=(0.0, ZP))
    m = m.cut(pocF).cut(pocE)
    cup = (cupF, cupE)
    # wire channel: PIP pin (radial at x 3.2) -> along z -> DIP pin
    m = m.cut(cyly(0.6, 0.0, 3.4, WIRE_HOLE_X["pip"], ZP)).cut(teardrop(0.8, (3.2, 3.0, ZP), (3.2, 3.0, ZD)))
    if PAD_MID:
        m = m.cut(box(*PAD_MID["x"], -12.0, PAD_MID["floor_y"], *PAD_MID["z"]))  # S: BMP581 pad pocket
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
    d = d.cut(cylx(2.5, DIP_BUSHP[0], DIP_BUSHP[1], 0, ZD)).cut(cylx(CB / 2, DIP_BUSHP[1], DIP_CHP[1] + 1.5, 0, ZD))  # S
    # S wire: +x cap -> up the +x cheek -> dorsal cross groove under the nail (z_rel 3.9-5.1) -> -x cheek -> DIP pin -x end
    for g in (box(6.5, 8.6, 2.6, 7.6, ZD - 0.6, ZD + 0.6), box(6.5, 8.6, 6.0, 7.6, ZD - 0.6, ZD + 5.1),
              box(-7.0, 8.6, 6.0, 7.6, ZD + 3.9, ZD + 5.1), box(-7.0, -5.85, 6.0, 7.6, ZD - 0.6, ZD + 5.1),
              box(-7.0, -5.85, 0.0, 7.6, ZD - 0.6, ZD + 0.6)):
        d = d.cut(g)
    d = d.cut(cylx(1.0, DIP_CHN[0] - 1, DIP_CHN[0] + 4.0, Bp[0], Bp[1]))
    d = d.cut(box(DIP_CHP[1] - 1.2, DIP_CHP[1] + 1, -0.6, 0.6, ZD, ZD + 8.3))
    d = d.cut(cylx(0.8, -0.5, DIP_CHP[1] - 0.6, -0.2, ZD + 7.7))
    # sensor cavity with 45 deg undercut (05, scaled: opening +-3.8, widest +-5.2, 4.8 deep), z_rel 10..19.5
    s = lambda v: YF - v  # noqa: E731
    cav = [(-3.8, s(0.0)), (3.8, s(0.0)), (5.2, s(-1.4)), (5.2, s(-6.0)), (-5.2, s(-6.0)), (-5.2, s(-1.4))]  # S: +1.2 for the tip carrier
    cavw = cq.Workplane("XY").workplane(offset=ZD + TS + 10.0).polyline(cav).close().extrude(9.5)
    d = d.cut(cavw)
    d = d.cut(cylz(1.3, ZD + TS + 7.5, ZD + TS + 10.1, 0.0, s(-4.4)))
    d = d.cut(box(-4.5, 4.5, YF, YF + 0.25, ZT - 5.0, ZT + 0.1))  # nail shim slot (0.15 shim)
    # skin (snap-in plug + crowned pad) and refs
    plug = cq.Workplane("XY").workplane(offset=ZD + TS + 10.3).polyline(
        [(-3.8, s(0.0)), (3.8, s(0.0)), (5.2, s(-1.4)), (5.2, s(-2.3)), (-5.2, s(-2.3)), (-5.2, s(-1.4))]).close().extrude(8.9)
    pad = box(-6.4, 6.4, s(3.0), YF, ZD + TS + 9.0, ZT - 0.6).edges("|Y").fillet(3.5)
    crown = cylz(25.0, ZD + TS + 8.0, ZT + 1.0, 0.0, s(3.0) + 25.0)
    skin = pad.intersect(crown).union(plug).cut(cyly(1.4, s(2.4), s(-2.3) + 0.01, 0.0, ZD + TS + 15.0))
    pcb = box(-4.6, 4.6, s(-4.8) - 0.8, s(-4.8), ZD + TS + 10.3, ZD + TS + 19.3)
    chip = box(-1.5, 1.5, s(-4.8) - 1.8, s(-4.8) - 0.8, ZD + TS + 13.5, ZD + TS + 16.5)
    mag = cyly(1.5, s(2.4), s(1.4), 0.0, ZD + TS + 15.0)  # S: Ø3 x 1.0 N42 axial
    shim = box(-4.5, 4.5, YF, YF + 0.15, ZT - 5.0, ZT + 1.5)
    carrier = cq.importers.importStep(str(OUT15 / "tip_carrier.step")).translate((0, 0, ZD - 66.0 + TS))
    return d, dict(tip_skin=skin, tip_pcb=pcb, tip_chip=chip, tip_magnet=mag, nail_shim=shim, tip_carrier=carrier)


def build_base():
    """bench block. Top (under the yoke) at Z_ABD - 6.5; open front bay where the curled finger goes;
    palmar abd support = a low bridge between side pillars holding the fixed Ø3 stub (top <= proximal at MCP 90)."""
    ZTOP = Z_ABD - 6.5
    blk = box(-16.0, 16.0, -9.5, 20.0, Z_BOT, ZTOP)  # rear block (behind the palm plane)
    pillars = U(box(-16.0, -13.0, -18.0, -9.5, Z_BOT, Z_ABD - 3.5), box(13.0, 16.0, -18.0, -9.5, Z_BOT, Z_ABD - 3.5))
    floor = box(-16.0, 16.0, -18.0, -9.5, Z_BOT, -46.0)
    bridge = U(box(-16.0, 16.0, -17.0, -12.9, Z_ABD - 3.5, Z_ABD + 2.4), cyly(3.1, -17.0, -12.9, 0.0, Z_ABD))
    lugD = box(-6.0, 6.0, 14.2, 20.35, ZTOP, Z_ABD + 6.0)  # S: to y 20.35 for the abd sensor cap
    b = U(blk, pillars, floor, bridge, lugD)
    # strand channel and exits through the bottom; window in the front for threading
    b = b.cut(box(-11.0, 11.0, -9.5, 14.0, Z_BOT - 1, ZTOP + 1))
    # abd: fixed stub (palmar, hollow) + MR63 seat (dorsal)
    b = b.cut(cyly(1.5, -18.0, -12.8, 0.0, Z_ABD))
    b = b.cut(cyly(SEAT63 / 2, 14.1, 16.7, 0.0, Z_ABD)).cut(cyly(2.4, 16.6, 18.3, 0.0, Z_ABD))
    b = b.cut(cyly(CB / 2, 18.15, 20.5, 0.0, Z_ABD)).cut(cyly(4.3, 19.6, 20.5, 0.0, Z_ABD))  # S: abd cap counterbore + flange
    # R2: no base guide dowels (the nips on the abd plane do the guiding; strands go straight down)
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
                 box(-9.4 - CLR, -4.8, -7.9 - CLR, -1.5 + CLR, -30.0, Z_ABD + 2.5 + CLR),
                 box(4.8, 9.4 + CLR, 1.5 - CLR, 7.9 + CLR, -30.0, Z_ABD + 2.5 + CLR),
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
    """bought parts, each tagged with its body (R2)."""
    m = {}
    # MCP: nested axle; S: Ø1.2 wire hole at x +3.0 along +y through inner tube, +x Ø4 sleeve and +x brass sleeve
    hole = cyly(0.6, -0.1, 2.7, WIRE_HOLE_X["mcp"], 0.0)
    m["mcp_axle_inner"] = ("yoke", cylx(1.5, AXLE["x0"], AXLE["x1"]).cut(cylx(1.0, AXLE["x0"] - 1, AXLE["x1"] + 1)).cut(hole), "metal")
    for sx, nm in ((-1, "n"), (1, "p")):
        a, b = (-7.4, -1.45) if sx < 0 else (1.45, MCP_SEAT_P[1])
        sl4 = cylx(2.0, a, b).cut(cylx(1.5, a - 1, b + 1))
        a2, b2 = sorted((sx * SLEEVE["x"][0], sx * SLEEVE["x"][1]))
        sl5 = cylx(2.5, a2, b2).cut(cylx(2.0, a2 - 1, b2 + 1))
        if sx > 0:
            sl4, sl5 = sl4.cut(hole), sl5.cut(hole)
        m[f"mcp_sleeve4_{nm}"] = ("yoke", sl4, "metal")
        m[f"mcp_sleeve5_{nm}"] = ("yoke", sl5, "brass")
        if sx < 0:
            bi, bo = bearing(MR84, -MCP_SEAT[1], -MCP_SEAT[0])
            m["mcp_brg_n_in"], m["mcp_brg_n_out"] = ("yoke", bi, "bearing"), ("proximal", bo, "bearing")
        else:
            bi, bo = bearing(ZRO2, *MCP_SEAT_P)
            m["mcp_zro2_in"], m["mcp_zro2_out"] = ("yoke", bi, "ceramic"), ("proximal", bo, "ceramic")
        a, b = sorted((sx * MCP_RING[0], sx * MCP_RING[1]))
        m[f"mcp_ring_{nm}"] = ("proximal", ring_part(RING_RO, RING_RI, a, b, gap_phi=(30, 66)), "brass")
    bi, bo = bearing(MR63_, -IDL_W / 2, IDL_W / 2)
    m["idler_in"], m["idler_out"] = ("yoke", bi, "bearing"), ("yoke", bo, "bearing")
    bi, bo = bearing(MR63_, *ROLLER_X, *ROLLER)
    m["roller_in"], m["roller_out"] = ("yoke", bi, "bearing"), ("yoke", bo, "bearing")
    m["dowel_roller"] = ("yoke", cylx(1.5, -4.0, 4.0, *ROLLER), "metal")
    m["dowel_ext"] = ("yoke", cylx(1.5, *EXT_RD_X, *EXT_RD), "metal")
    for nm, (x, y0, y1, dpin) in nip_pins().items():
        m[nm] = ("yoke", cyly(dpin / 2, y0, y1, x, Z_ABD), "metal")
    # S joint-angle sensors (R3 slim, out/15/sensor_spec.json): magnet in the axle/pin end, chip faces it, Ø6.4 board,
    # printed cap from out/15 (all in the finger frame)
    for key, body_m, body_c, zc, mag_r in (("mcp_flex", "yoke", "proximal", 0.0, 1.0), ("pip", "middle", "proximal", ZP, 1.5),
                                           ("dip", "middle", "distal", ZD, 1.5)):
        sp = SENS[key]
        short = key.split("_")[0]
        m[f"sensor_{short}_magnet"] = (body_m, cylx(mag_r, *sp["magnet"]["x"], 0, zc), "magnet")
        m[f"sensor_{short}_chip"] = (body_c, box(*sp["chip_x"], -1.45, 1.45, zc - 1.4, zc + 1.4), "chip")
        m[f"sensor_{short}_board"] = (body_c, cylx(3.2, *sp["board_x"], 0, zc), "pcb")
        cap = cq.importers.importStep(str(OUT15 / f"joint_cap_{key}.step")).translate((0, 0, zc - {"mcp_flex": 0.0, "pip": 42.0, "dip": 66.0}[key]))
        m[f"sensor_{short}_cap"] = (body_c, cap, "sensorcap")
    # abd
    m["abd_stub_palmar"] = ("base", cyly(1.5, -15.9, -10.1, 0, Z_ABD).cut(cyly(1.0, -18, -9, 0, Z_ABD)), "metal")
    m["abd_stub_dorsal"] = ("yoke", cyly(1.5, 11.5, 16.7, 0, Z_ABD), "brass")  # S: brass/316, not a hardened dowel
    sa = SENS["mcp_abd"]
    m["sensor_abd_magnet"] = ("yoke", cyly(1.5, *sa["magnet"]["y"], 0, Z_ABD), "magnet")
    m["sensor_abd_board"] = ("base", cyly(3.2, *sa["board_y"], 0, Z_ABD), "pcb")
    m["sensor_abd_chip"] = ("base", box(-1.45, 1.45, sa["board_y"][1], sa["board_y"][1] + 1.45, Z_ABD - 1.4, Z_ABD + 1.4), "chip")
    m["sensor_abd_cap"] = ("base", cq.importers.importStep(str(OUT15 / "joint_cap_abd.step")), "sensorcap")
    # knuckle board (abd chip bus + JST SH 4-pin socket) on top of the dorsal lug, and the finger pigtail plug
    m["knuckle_board"] = ("base", box(-7.0, 7.0, 14.5, 20.3, Z_ABD + 6.2, Z_ABD + 7.8), "pcb")
    m["jst_sh_socket"] = ("base", box(-3.0, 3.0, 16.5, 20.75, Z_ABD + 7.8, Z_ABD + 12.05), "connector")
    m["jst_sh_plug"] = ("base", box(-2.5, 2.5, 16.8, 20.45, Z_ABD + 12.05, Z_ABD + 14.5), "connector")
    bi, bo = bearing(MR63, -12.6, -10.1, 0.0, Z_ABD, axis="y")
    m["abd_brg_p_in"], m["abd_brg_p_out"] = ("base", bi, "bearing"), ("yoke", bo, "bearing")
    bi, bo = bearing(MR63, 14.2, 16.7, 0.0, Z_ABD, axis="y")
    m["abd_brg_d_in"], m["abd_brg_d_out"] = ("yoke", bi, "bearing"), ("base", bo, "bearing")
    m["abd_ring"] = ("yoke", cyly(RING_RO, 9.7, 12.7, 0, Z_ABD).cut(cyly(RING_RI, 9, 13.5, 0, Z_ABD)).cut(
        box(-3.4, 3.4, 9.0, 13.5, Z_ABD + 5.0, Z_ABD + 8.0)), "brass")
    # PIP
    m["pip_pin"] = ("middle", cylx(2.0, *PIP_PIN, 0, ZP).cut(cylx(1.5, PIP_PIN[0] - 1, PIP_PIN[1] + 1, 0, ZP)).cut(
        cyly(0.6, 0.0, 2.1, WIRE_HOLE_X["pip"], ZP)), "metal")
    m["pip_bush"] = ("proximal", cylx(2.5, *PIP_BUSH, 0, ZP).cut(cylx(2.0, -9, 0, 0, ZP)), "brass")
    bi, bo = bearing(ZRO2, *PIP_SEAT, 0, ZP)
    m["pip_zro2_in"], m["pip_zro2_out"] = ("middle", bi, "ceramic"), ("proximal", bo, "ceramic")
    m["pip_ring"] = ("middle", ring_part(RING_RO, RING_RI, *PIP_RING, gap_phi=(PHI_PIP - 32, PHI_PIP + 32), zc=ZP), "brass")
    # DIP
    m["dip_pin"] = ("middle", cylx(2.0, *DIP_PIN, 0, ZD).cut(cylx(1.5, DIP_PIN[0] - 1, DIP_PIN[1] + 1, 0, ZD)).cut(
        cyly(0.6, 0.0, 2.1, WIRE_HOLE_X["dip"], ZD)), "metal")
    m["dip_bush_n"] = ("distal", cylx(2.5, *DIP_BUSHN, 0, ZD).cut(cylx(2.0, -9, 0, 0, ZD)), "brass")
    m["dip_bush_p"] = ("distal", cylx(2.5, *DIP_BUSHP, 0, ZD).cut(cylx(2.0, 0, 9, 0, ZD)), "brass")
    A0, B0 = link_pins(0, 0)[2:]
    m["pin_A"] = ("proximal", cylx(1.0, PIP_LUGN[0] + 4.0 - 6.0 - 0.95, PIP_LUGN[0] + 4.0, A0[0], A0[1]), "brass")  # S: 316/brass
    m["pin_B"] = ("distal", cylx(1.0, DIP_CHN[0] + 4.0 - 6.0 - 0.95, DIP_CHN[0] + 4.0, B0[0], B0[1]), "brass")
    # S pad sensors (BMP581 on a 5 x 6 x 0.6 board, chip 2.0 x 2.0 x 0.75 facing -y) + cast silicone pad over each pocket
    for nm, P, body, face in (("prox", PAD_PROX, "proximal", -7.75), ("mid", PAD_MID, "middle", -7.25)):
        if not P:
            continue
        xc, zc = (P["x"][0] + P["x"][1]) / 2, (P["z"][0] + P["z"][1]) / 2
        yb = P["floor_y"]
        m[f"pad_{nm}_board"] = (body, box(xc - 2.5, xc + 2.5, yb - 0.6, yb, zc - 3.0, zc + 3.0), "pcb")
        m[f"pad_{nm}_chip"] = (body, box(xc - 1.0, xc + 1.0, yb - 1.35, yb - 0.6, zc - 1.0, zc + 1.0), "chip")
        fill = box(P["x"][0] + 0.05, P["x"][1] - 0.05, face - 0.2, yb - 1.35, P["z"][0] + 0.05, P["z"][1] - 0.05)
        fill = fill.union(box(P["x"][0] + 0.05, P["x"][1] - 0.05, yb - 1.35, yb - 0.6, P["z"][0] + 0.05, P["z"][1] - 0.05).cut(
            box(xc - 2.55, xc + 2.55, yb - 1.4, yb, zc - 3.05, zc + 3.05)))
        m[f"pad_{nm}_silicone"] = (body, fill, "skin")
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
        "cup_abd_flex": ("yoke", cup_abd[0], "cup", True), "cup_abd_ext": ("yoke", cup_abd[1], "cup", True),
        "cup_mcp_flex": ("proximal", cupF, "cup", True), "cup_mcp_ext": ("proximal", cupE, "cup", True),
        "cup_pip_flex": ("middle", cup_pip[0], "cup", True), "cup_pip_ext": ("middle", cup_pip[1], "cup", True),
    }
    for k, w in tipparts.items():
        parts[k] = ("distal", w, {"tip_skin": "skin", "tip_pcb": "pcb", "tip_chip": "chip", "tip_magnet": "magnet",
                                  "nail_shim": "metal", "tip_carrier": "sensorcap"}[k], k == "tip_skin")
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


# S sensor wire: one 4 x 38 AWG bundle per finger (Ø ~0.8), daisy-chained tip -> DIP -> PIP -> MCP -> abd/knuckle.
# Each waypoint is fixed to one body (finger frame at q = 0); waypoints ON a joint axis are where the bundle crosses
# that joint by twisting inside the hollow pin, so they coincide in both bodies.
WIRE_PATH = [
    ((0.0, -1.4, ZD + TS + 10.3), "distal", "tip board pads"), ((0.0, -1.4, ZD + TS + 7.6), "distal", "Ø2.6 cable bore"),
    ((0.0, -0.2, ZD + 7.7), "distal", ""), ((6.9, -0.2, ZD + 7.7), "distal", "channel to the +x cheek"),
    ((7.4, 0.0, ZD + 6.5), "distal", ""), ((7.4, 0.0, ZD + 0.8), "distal", "groove down to the DIP cap"),
    ((8.0, 0.0, ZD), "distal", "DIP board (sensor_dip)"), ((7.6, 6.8, ZD), "distal", "up the +x cheek"),
    ((7.6, 6.8, ZD + 4.5), "distal", ""), ((-6.4, 6.8, ZD + 4.5), "distal", "dorsal groove under the nail"),
    ((-6.4, 6.8, ZD), "distal", ""), ((-6.4, 0.0, ZD), "distal", "glue point A"),
    ((-5.85, 0.0, ZD), "distal", "DIP pin -x end (on axis: twist)"), ((3.2, 0.0, ZD), "middle", "radial hole x 3.2, glue B"),
    ((3.2, 3.0, ZD), "middle", ""), ((3.2, 3.0, ZP), "middle", "middle channel"),
    ((WIRE_HOLE_X["pip"], 0.0, ZP), "middle", "PIP pin radial hole, glue C"),
    ((-5.65, 0.0, ZP), "proximal", "PIP pin -x end (on axis: twist)"), ((-6.4, 0.0, ZP), "proximal", "glue D"),
    ((-6.4, 7.3, ZP), "proximal", "-x lug groove up"), ((-6.4, 7.3, ZP - 3.0), "proximal", ""),
    ((7.55, 7.3, ZP - 3.0), "proximal", "dorsal groove"), ((7.55, 3.0, ZP - 3.0), "proximal", ""),
    ((8.2, 0.0, ZP), "proximal", "PIP board (sensor_pip)"), ((8.6, 3.0, ZP - 2.0), "proximal", "+x face groove"),
    ((9.0, 3.0, 2.0), "proximal", ""), ((8.35, 0.0, 0.0), "proximal", "MCP board (sensor_mcp)"),
    ((8.9, 7.6, 0.0), "proximal", "+x lug face up"), ((8.9, 7.4, 11.0), "proximal", ""),
    ((-8.9, 7.4, 11.0), "proximal", "dorsal hood groove z 10.4-11.6"), ((-8.9, 7.6, 0.0), "proximal", ""),
    ((-8.9, 0.0, 0.0), "proximal", "glue E"), ((-7.4, 0.0, 0.0), "yoke", "MCP axle -x end (on axis: twist)"),
    ((3.0, 0.0, 0.0), "yoke", "radial hole x +3.0, glue F"), ((3.0, 2.6, 0.0), "yoke", ""), ((3.0, 8.0, -4.0), "yoke", ""),
    ((3.0, 8.0, -3.0), "yoke", "yoke exit (top of the tongue column, dorsal of the idler)"),
    ((1.5, 13.5, -6.0), "loop", "free loop over the abd joint (R >= 8)"),
    ((0.0, 17.0, Z_ABD + 7.8), "base", "knuckle board"), ((0.0, 18.6, Z_ABD + 14.5), "base", "JST SH plug (finger root)"),
]


def sensor_wire(q):
    Tb, dip = body_T(q)
    pts = []
    for i, (p, body, _) in enumerate(WIRE_PATH):
        if body == "loop":
            a = xf(Tb["yoke"], WIRE_PATH[i - 1][0])[0]
            b = np.asarray(WIRE_PATH[i + 1][0], float)
            pts.append(0.5 * (a + b) + np.array([0.0, 1.0, 1.5]))
        else:
            pts.append(xf(Tb[body], p)[0])
    return np.array(pts)


def wire_mesh(P, r=0.4):
    import trimesh
    segs = [trimesh.creation.cylinder(radius=r, segment=[a, b], sections=8) for a, b in zip(P[:-1], P[1:])
            if np.linalg.norm(b - a) > 1e-3]
    return trimesh.util.concatenate(segs)


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


def pitch_analysis(parts, with_caps=True):
    """R2 #4: smallest finger pitch (MCP spacing in x) at which two neighbouring fingers of this design do not touch.
    Finger A at x 0, finger B at x +p (toward the thumb). Both at the same MCP/PIP pose; abd angles (tA, tB).
    Bins of 1 mm in (y, z): need x_max(A) + clearance <= p + x_min(B). Base/bench parts excluded."""
    import trimesh
    from tools.render import tessellate
    res = {}
    cache = {}
    for mcp, pip in ((0, 0), (90, 95), (45, 50)):
        for t in (-15, -7.5, 0, 7.5, 15):
            pp = posed(parts, dict(abd=t, mcp=mcp, pip=pip))
            V = []
            for n, (body, w, col, pr) in pp.items():
                if body == "base" or (not with_caps and n.startswith(("hall_", "mcp_mag", "pip_mag"))):
                    continue
                v, f = tessellate(w, 0.3, 0.5)
                if len(v):
                    V.append(v)
            if not with_caps:
                pass
            cache[(mcp, t)] = np.vstack(V)
        def prof(P):
            iy = np.floor(P[:, 1]).astype(int)
            iz = np.floor(P[:, 2]).astype(int)
            d = {}
            for a, b, x in zip(iy, iz, P[:, 0]):
                k = (a, b)
                lo, hi = d.get(k, (1e9, -1e9))
                d[k] = (min(lo, x), max(hi, x))
            return d
        prof_c = {t: prof(cache[(mcp, t)]) for t in (-15, -7.5, 0, 7.5, 15)}
        for tA in (-15, -7.5, 0, 7.5, 15):
            for tB in (-15, -7.5, 0, 7.5, 15):
                A, B = prof_c[tA], prof_c[tB]
                need = -1e9
                for k, (loA, hiA) in A.items():
                    for dk in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
                        kb = (k[0] + dk[0], k[1] + dk[1])
                        if kb in B:
                            need = max(need, hiA - B[kb][0] + 0.5)
                res[f"mcp{mcp}_pip{pip}_A{tA}_B{tB}"] = need
    return res


def update_travel_table(st):
    p = OUT / "travel_table.json"
    TT = json.loads(p.read_text())
    k = st["coupling_deg_per_deg"]
    meas = {"pip": {"mcp_flex": k["pip_per_mcp"], "mcp_abd": k["pip_per_abd"]}, "mcp_flex": {"mcp_abd": k["mcp_per_abd"]},
            "mcp_abd": {}}
    G = TT["G"]
    avail = 300.0 - TT["servo"]["SCS0009"]["calibration_margin_deg"]
    rng = {"pip": (-5, 95), "mcp_flex": (-10, 90), "mcp_abd": (-15, 15)}
    for name, row in TT["fingers"].items():
        j = name.split("_", 1)[1]
        row["k_measured_signed"] = meas[j]
        tot = (rng[j][1] - rng[j][0]) + sum(abs(v) * (rng[jj][1] - rng[jj][0]) for jj, v in meas[j].items())
        row["servo_deg_without_wrist"] = round(tot * G, 1)
        row["wrist_budget_servo_deg_SCS"] = round(avail - tot * G, 1)
        row["wrist_budget_strand_mm_SCS"] = round((avail - tot * G) * D2R * TT["spool_r_mm"], 2)
        row["firmware"] = f"servo_joint_equiv = q_{j} " + " ".join(f"{v:+.3f}*q_{jj}" for jj, v in meas[j].items())
    TT["measured_from"] = "cad/12_finger.py strand model (exact belt paths, abd in 3D), out/12/checks.json"
    TT["lanes_at_abd_plane_mm"] = {k2: dict(x=float(SDEF[k2]["x"][1]), y=float(SDEF[k2]["nip"][0])) for k2 in SDEF}
    p.write_text(json.dumps(TT, indent=1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args()
    t0 = time.time()
    checks = {}
    st = strand_checks()
    checks["strands"] = st
    update_travel_table(st)
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
    # S: sensor wire centreline vs solids (its own end points sit in boards/caps: those parts are skipped)
    wis = {}
    for q in (dict(abd=0, mcp=0, pip=0), dict(abd=15, mcp=45, pip=50), dict(abd=-15, mcp=90, pip=95)):
        pp = {k: v for k, v in posed(parts, q).items() if not k.startswith(("sensor_", "tip_", "knuckle", "jst", "pad_"))}
        wis[json.dumps(q)] = strands_in_solids(pp, q, {"sensor_wire": sensor_wire(q)}, step=0.3)["sensor_wire"]
    checks["sensor_wire_inside_solids"] = wis
    print("sensor wire in solids:", wis)
    print("strands in solids:", sis)
    checks["dip_link"] = LINK
    if not args.quick:
        pa = pitch_analysis(parts, True)
        checks["pitch_mm_needed"] = dict(with_sensors_R3=pa)
        for lab, d in (("R3 sensors", pa),):
            same = max(v for k, v in d.items() if k.split("_A")[1].split("_B")[0] == k.split("_B")[1])
            print(f"pitch ({lab}): parallel abd worst {same:.1f} mm; all pairs worst {max(d.values()):.1f} mm")
    (OUT / "checks.json").write_text(json.dumps(checks, indent=1, default=float))
    # strands json (polylines at q = 0)
    sj = {}
    for q in (dict(abd=0, mcp=0, pip=0),):
        _, ms = strand_meshes(q)
        sj = {k: np.round(P, 3).tolist() for k, P in ms.items()}
    (OUT / "strands.json").write_text(json.dumps(sj))
    WJ = {"about": "sensor wire bundle (4 x 38 AWG) centreline, finger frame, mm; one polyline per pose. Each waypoint "
                   "is fixed to a body; points on a joint axis are where the bundle twists inside the hollow pin.",
          "waypoints": [dict(p=list(p), body=b, note=n) for p, b, n in WIRE_PATH], "poses": {}}
    for tag, q in (("q0", dict(abd=0, mcp=0, pip=0)), ("pinch", dict(abd=0, mcp=30, pip=45)),
                   ("fist_abd15", dict(abd=15, mcp=90, pip=95)), ("abd_m15", dict(abd=-15, mcp=45, pip=50))):
        P = sensor_wire(q)
        seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
        WJ["poses"][tag] = dict(q=q, points=np.round(P, 3).tolist(), length_mm=float(seg.sum()))
    (OUT / "sensor_wires.json").write_text(json.dumps(WJ, indent=0))
    checks["sensor_wire_length_mm"] = {k: v["length_mm"] for k, v in WJ["poses"].items()}
    # assembly export at q = 0
    from tools.export import export_all
    pp0 = posed(parts, dict(abd=0, mcp=0, pip=0))
    asm = {n: v[1] for n, v in pp0.items()}
    cols = {n: COLORS[v[2]] for n, v in pp0.items()}
    sm, _ = strand_meshes(dict(abd=0, mcp=0, pip=0))
    export_all(asm, OUT / "assembly", name="bench_finger_q0", colors=cols, stl=False)
    # GLB + strands (black) + sensor wire (red), so the 3D viewer shows them
    import trimesh
    sc = trimesh.load(str(OUT / "assembly" / "bench_finger_q0.glb"))
    extra = dict(sm)
    extra["sensor_wire_bundle"] = wire_mesh(sensor_wire(dict(abd=0, mcp=0, pip=0)))
    for k, mesh in extra.items():
        rgb = (214, 39, 40, 255) if k.startswith("sensor") else (32, 32, 32, 255)
        mesh.visual.face_colors = np.tile(rgb, (len(mesh.faces), 1))
        sc.add_geometry(mesh, node_name=k, geom_name=k)
    sc.export(str(OUT / "assembly" / "bench_finger_q0.glb"))
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
            d["sensor_wire"] = wire_mesh(sensor_wire(q))
            c["sensor_wire"] = COLORS["wire"]
            render(d, str(OUT / f"bench_finger_{tag}.png"), views=views, colors=c, size=(1600, 1200),
                   title=f"V2.1 bench finger + sensors (R3), abd {q['abd']} / MCP {q['mcp']} / PIP {q['pip']}", edges=True)
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
        # S: section through the joint axes (y > 0 removed), seen from +y: magnets, chips, boards, wire in the pins
        sec, cs = {}, {}
        cutY = box(-30, 30, 0.0, 40, -60, 100)
        for n, v in pp.items():
            if v[0] == "base":
                continue
            try:
                w = v[1].cut(cutY)
                if w.vals():
                    sec[n], cs[n] = w, COLORS[v[2]]
            except Exception:
                pass
        sec["sensor_wire"], cs["sensor_wire"] = wire_mesh(sensor_wire(dict(abd=0, mcp=0, pip=0)), r=0.3), COLORS["wire"]
        render(sec, str(OUT / "bench_finger_sensor_section.png"), views=("back",), colors=cs, size=(1000, 1700),
               title="section y = 0 (from +y): joint sensors, hollow pins, wire", edges=True)
        render(d, str(OUT / "bench_finger_cut.png"), views=("side",), colors=c, size=(1400, 1400),
               title="cut at x = 0.4 (pip strands, idler, redirect)", edges=True)
    write_bom()
    print(f"done in {time.time() - t0:.0f} s")


def write_bom():
    """R2 #16: what the owner actually pays for a first bench finger (whole packs, EUR, Nov 2026 typical online
    prices from memory; verify). 'per finger' = the share one finger uses, for a later 5-finger hand."""
    rows = [
        # part, qty used, spec, where, pack price, per-finger share
        ("MR84ZZ 4x8x3", 1, "MCP -x lug (the +x and PIP sensor sides are ZrO2 now)", "10-pack", 6.0, 0.6),
        ("MR63ZZ 3x6x2.5", 4, "MCP idler, pip flex roller, abd (yoke) ... + 1 dorsal abd (base) = 4", "10-pack", 5.0, 2.0),
        ("stainless tube 3.0 x 0.5", 2, "MCP inner axle 14.8 mm, palmar abd stub 6.9 mm", "3 x 300 mm", 6.0, 0.6),
        ("stainless tube 4.0 x 0.5", 4, "MCP sleeves 2 x 6.0, PIP pin 12.3, DIP pin 13.4", "3 x 300 mm", 6.0, 0.8),
        ("brass tube 5.0 x 0.5", 5, "cheek sleeves 2 x 2.7, PIP bush 3.0, DIP bushes 2 x 3.0", "300 mm", 4.0, 0.3),
        ("brass tube 14 x 0.5 (verify stock; else 14 x 0.45 / 14.3 x 0.65)", 4, "rings: MCP 2 x 1.6, PIP 3.0, abd 3.0", "300 mm", 7.0, 0.6),
        ("hardened dowel / pin assortment Ø1.5 / Ø2 / Ø3", 1, "Ø3 roller, Ø3 ext dowel, Ø3 dorsal stub; Ø2 x 4 pip nips, Ø1.5 x 4 MCP nips, Ø2 bar pins", "assortment box", 12.0, 2.0),
        ("push-on star washers Ø2", 2, "DIP bar retention", "pack", 3.0, 0.3),
        ("stainless strip 0.8 mm (DIP bar)", 1, "laser-cut service (min. order) or 0.8 sheet + drill/file", "sheet 100 x 100", 8.0, 0.4),
        ("PE#8 UHMWPE 8-strand braid 100 m", 1, "6 strands x 0.45 m; pre-stretch at <= 100 N (08's safety limit)", "fishing shop", 15.0, 0.2),
        ("2 x 2 mm crimp tubes (copper/brass), 100 pcs", 1, "fuse-cup beads (line doubled), measure crushed size", "beading shop", 4.0, 0.2),
        ("diametric magnets Ø2 x 1.5 N45 (MCP) and Ø3 x 1.0 N45 (PIP, DIP, abd), packs of 50", 2, "joint-angle magnets", "AliExpress", 12.0, 1.2),
        ("axial magnets Ø3 x 1.0 N42, pack of 20", 1, "tip skin (was Ø3 x 1.5: saturates the MLX90393)", "AliExpress", 4.0, 0.3),
        ("TMAG5273 A1/B1/C1/D1 bare chips", 4, "abd, MCP, PIP, DIP (fixed addresses 0x35/0x22/0x78/0x44)", "Mouser / LCSC", 8.0, 6.4),
        ("MLX90393-014 QFN + BMP581 x 2", 3, "tip 3-axis + proximal/middle pads", "Mouser / LCSC", 15.0, 9.5),
        ("JLCPCB panel: Ø6.4 joint boards, 9.2 x 9.0 tip, 5 x 6 pad, knuckle board; assembled", 1, "setup + assembly + shipping", "JLCPCB", 45.0, 9.0),
        ("open full-ceramic ZrO2 MR84 4 x 8 x 2", 2, "MCP + PIP sensor side (C0 NOT verified; HLS: iglidur/bronze 4x8x2 flanged bush)", "bearing shop", 14.0, 14.0),
        ("38 AWG PTFE stranded wire, 4 colours (red 3V3, black GND, yellow SDA, blue SCL)", 1, "one parallel bundle per finger, never twisted", "pack", 12.0, 0.5),
        ("JST SH 1.0 mm 4-pin socket + pre-crimped plug leads", 1, "finger root (knuckle board)", "kit", 6.0, 0.6),
        ("316 / brass Ø2 pins + Ø3 brass rod", 1, "DIP bar pins A/B and the dorsal abd stub (no hardened steel within 3 mm of a magnet)", "model shop", 5.0, 0.5),
        ("Sil-Poxy (flexible silicone adhesive)", 1, "wire glue points A-F, skin magnet", "Smooth-On reseller", 12.0, 0.0),
        ("Dragon Skin 20 trial kit (or Smooth-Sil 940)", 1, "tip skin (05 mould)", "Smooth-On reseller", 33.0, 0.6),
        ("2-part epoxy (plastic seats) + Loctite 641 (metal-metal only, review #13)", 1, "", "hardware store", 18.0, 0.0),
        ("M5 x 12 + 2020 T-nuts, M3 x 12 + nuts", 1, "base mount", "assortment", 6.0, 0.0),
        ("2020 extrusion 300 mm + 2 corner brackets", 1, "bench frame", "any", 8.0, 0.0),
        ("0.25 mm stainless wire (threading leader)", 1, "", "pack", 3.0, 0.0),
        ("PETG filament (about 60 g incl. spares; PA-CF only for HLS runs)", 1, "", "already owned", 0.0, 1.5),
        ("SCS0009 servos", 3, "abd, MCP, PIP (owner may have them)", "Feetech reseller", 39.0, 39.0),
        ("spool cartridges (02): 3 x (2 MR63 from the pack + Ø3 dowel + M2 screw/nut)", 3, "printed; r 5 on every joint now", "-", 0.0, 0.0),
    ]
    with open(OUT / "bom.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["part", "qty used", "spec / where used", "buy as", "first-buy EUR (whole pack)", "per-finger share EUR"])
        for r in rows:
            w.writerow(r)
        nos = [r for r in rows if not r[0].startswith("SCS0009")]
        w.writerow(["TOTAL first buy, without servos", "", "", "", round(sum(r[4] for r in nos), 1), round(sum(r[5] for r in nos), 1)])
        w.writerow(["TOTAL first buy, with 3 SCS0009", "", "", "", round(sum(r[4] for r in rows), 1), round(sum(r[5] for r in rows), 1)])


if __name__ == "__main__":
    main()
