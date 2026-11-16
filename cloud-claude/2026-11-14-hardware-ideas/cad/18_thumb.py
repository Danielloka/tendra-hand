"""18 · Thumb V2.1 (study 2026-11-16, round 3 package B): the V2 thumb with the V2.1 rules and sensors.

API (for the integrator, cad/18_assembly.py):
    spec = importlib.util.spec_from_file_location("thumb18", "<study>/cad/18_thumb.py")
    T = importlib.util.module_from_spec(spec); spec.loader.exec_module(T)     # light: numpy + V2 kinematics, no CAD
    T.JOINTS                 -> ("cmc_rot", "cmc_flex", "mcp_flex", "ip")   deg, + = closing (V2 kinematics, unchanged)
    T.RANGES                 -> {joint: (lo, hi)} deg
    T.assembly_parts()       -> {part: (trimesh at q = 0 in the HAND frame, "thumb:<body>", material)}  (reads out/18/thumb/stl)
    T.parts()                -> {part: {"body", "material", "stl", "color"}}                          (out/18/thumb/parts.json)
    T.pose(q)                -> {part: 4x4 numpy}  hand-frame transform of each part from its q = 0 export;
                                q = {"cmc_rot": deg, ...} (keys may also be "thumb_cmc_rot"); missing = 0
    T.body_T(q)              -> {body: 4x4} for the bodies mount, base, metacarpal, proximal, distal
    T.lines(q)               -> {name: (N, 3) array}: 8 tendons "thumb_<joint>_<flex|ext>" from the palm-plate hole
                                (z 30.58, report 13 eye table) to the fuse-cup anchor, and "thumb_sensor_wire"
                                (tip board -> JST plug on the mount), hand frame, mm
    T.build()                -> (re)builds the CAD and exports out/18/thumb/ (~3 min); pose/lines do not need it
Bodies: mount (fixed to the palm: thumb mount, turning block, rollers, neck eyelet, rot sensor, plug), base (cmc_rot),
metacarpal (cmc_flex), proximal (mcp_flex), distal (ip). Hand frame (V2): origin wrist centre, +Z fingers, -Y palm, +X thumb.

What changed vs the V2 thumb (../2026-11-09-v2-hand-cad/cad/thumb.py, which this file imports for its kinematics,
frames and primitives; nothing there is modified): bare PE#8 strands, no PTFE; a 14 x 0.5 brass ring on every drum
(line radius 7.235, servo ratio 1.447 like the fingers); two lanes per drum, a 2 x 2 crimp in a printed fuse cup per
strand (no knots); a joint-angle sensor on all 4 joints (magnet in the axle end + TMAG5273 on a 6.4 board, ZrO2
bearing on the sensor side; cmc_rot: diametric ring magnet in the base cap read side-on); MLX90393 + snap-in
silicone skin at the tip; the sensor wire twists through the hollow axles (4 x 0.5 tubes) and crosses cmc_rot as a
flat clock spring under the base cap (the rot axis carries the six strands); strands leave report 13's palm plate
straight up, turn on MR63 rollers (row A), the six axial strands turn again on MR63 rollers (row B) up the rot
axis into a fixed polished neck eyelet; the two cmc_rot strands turn on two MR63 rollers (C) into the drum plane.
Run: uv run --no-project --with cadquery --with numpy --with trimesh --with scipy --with rtree --with manifold3d
     --with matplotlib --with pillow python cad/18_thumb.py [--quick] [--route]
"""
from __future__ import annotations

import copy
import functools
import importlib.util
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STUDY = HERE.parent
V2 = STUDY.parent / "2026-11-09-v2-hand-cad"
V2CAD = V2 / "cad"
OUT = STUDY / "out" / "18" / "thumb"
STL = OUT / "stl"
ROUTE_FILE = OUT / "route_layout.json"


# =====================================================================================================================
# V2 thumb, loaded with a private copy of V2 params (drum line radius 7.235, PE#8 0.47 line); the shared
# `params` module that the integrator uses is left untouched.
# =====================================================================================================================
def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


def _load_v2_thumb():
    saved = {k: sys.modules.get(k) for k in ("params", "thumb_kinematics")}
    if str(V2CAD) not in sys.path:
        sys.path.insert(0, str(V2CAD))
    Pp = _load("params_v21_thumb", V2CAD / "params.py")
    Pp.R_DRUM = 7.235  # 14 x 0.5 brass ring + 0.235 line radius (report 12/16: G = 1.447)
    Pp.LINE_D = 0.47  # PE#8
    Pp.MM_PER_DEG = np.pi * 2 * Pp.R_DRUM / 360.0
    sys.modules["params"] = Pp
    sys.modules.pop("thumb_kinematics", None)
    try:
        K = _load("thumb_kinematics_v21", V2CAD / "thumb_kinematics.py")
        sys.modules["thumb_kinematics"] = K
        TH = _load("thumb_v2_for18", V2CAD / "thumb.py")
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return Pp, K, TH


PV, K, TH = _load_v2_thumb()
D = TH.D
J0 = TH.J0
JOINTS = TH.JOINT_NAMES
RANGES = {k: tuple(v) for k, v in TH.RANGES.items()}
BODIES = TH.BODIES
BODY_OF_JOINT = TH.BODY_OF_JOINT
to_world, to_local = TH.to_world, TH.to_local
unit = K.unit

# --- V2.1 drum / lanes ---------------------------------------------------------------------------------------------
R = TH.R  # 7.235 line centre
RING = dict(od=14.0, id=13.0, L=3.0)  # brass tube 14 x 0.5, 3 mm long, glued (epoxy) on the drum seat
LANE_DX = 0.7  # flex lane at track - 0.7 * side, ext at + 0.7 (1.4 mm apart, like the finger's pip)
# MCP track moved to -x (V2 +3.9) so that every hinge's wire side is free of the track (see report 18 §2);
# the crossing order at cmc_flex is mirrored with it (mcp strands on -x, ip on +x).
TRACK = {"cmc_flex": -5.4, "mcp_flex": -4.5, "ip": 1.0}
ROT_LANE = {"flex": -15.6, "ext": -14.2}  # C-frame z (= along a1); ring band z -16.4..-13.4
ROT_RING_Z = (-16.4, -13.4)
CUP_DPHI = 14.0  # fuse cup this far past V2's knot angle, in each strand's own wrap direction
CUP = dict(lw=3.5, lt=2.7, h=2.9, ch_w=2.9, ch_t=2.1, lip=0.6)  # report 12: 2 x 2 crimp crushed 2.6 x 1.1, trip 67 N
# --- sensors (report 15 R3-slim) -----------------------------------------------------------------------------------
# sensor side per hinge (sign of the joint-frame x): the TMAG5273 stack sits in the parent's cheek on this side,
# the wire leaves the hollow axle on the other side into a keeper plug.
SENSOR_SIDE = {"cmc_flex": -1, "mcp_flex": -1, "ip": +1}
CHIP_VARIANT = {"cmc_rot": "TMAG5273A1 (0x35)", "cmc_flex": "TMAG5273B1 (0x22)", "mcp_flex": "TMAG5273C1 (0x78)",
                "ip": "TMAG5273D1 (0x44)"}
ZRO2 = dict(id=4.0, od=8.0, w=2.0)  # open full-ceramic MR84 (sensor side)
STACK = dict(gap=0.3, chip=1.45, board=0.4, cap=0.2)  # chip faces the magnet across 0.3 mm (die gap 1.2)
MAG_HINGE = dict(d=3.0, L=1.0)  # Ø3 x 1 N45 diametric in the axle-tube bore (ID 3)
AXLE_TUBE = dict(od=4.0, id=3.0)  # 316 stainless tube 4 x 0.5 (all thumb axles; hollow for the wire)
KEEPER_T = 1.0  # printed wire keeper plug on the wire side (holds the wire on the axis, retains the bearing)
# cmc_rot sensor: diametric ring magnet 12 x 8 x 2 N42 in the base cap, chip side-on in the mount wall
ROT_MAG = dict(ro=6.0, ri=4.0, z=(-27.6, -25.6))
ROT_CHIP_R = 12.1
CAP_Z = (-27.6, -25.4)
CLOCK_Z = (-29.3, -27.7)  # clock-spring cavity (fixed web below, rotating cap above)
WEB_Z = (-31.0, -29.3)  # mount bottom web, carries the neck eyelet
NECK_ZC = -30.4  # neck point (bottom lip of the eyelet) on the rot axis, C-frame z
NECK = dict(r_out=3.0, hole=0.8, lip=0.6)  # polished 303 / 440C eyelet, hole Ø1.6, R0.6 lip (like the eye plate)
MOUNT_Z = (-31.0, -17.2)
# --- palm plate (report 13) -----------------------------------------------------------------------------------------
Z_PLATE_TOP = 30.58  # H_PALM 30 + plate t 1.16 / 2
MR63 = dict(id=3.0, od=6.0, w=2.5)
RS = MR63["od"] / 2 + PV.LINE_D / 2  # strand centre radius on a roller 3.235

C = J0["C"]
A1 = C[3]
P0 = C[0]
NECK_W = to_world(C, [0.0, 0.0, NECK_ZC])
EYE = {r["name"]: (float(r["eye"][0]), float(r["eye"][1]))
       for r in json.loads((STUDY / "out" / "13" / "kin" / "eyeplate_kin.json").read_text())["eye_table"] if r["thumb"]}
STRAND_NAMES = ("cmc_rot_flex", "cmc_rot_ext", "cmc_flex_flex", "cmc_flex_ext", "mcp_flex_flex", "mcp_flex_ext",
                "ip_flex", "ip_ext")


def _v2key(n):
    """report 13 / 21 names (thumb_mcp_flex_flex) -> V2 strand keys (mcp_flex_flex)."""
    return n


# =====================================================================================================================
# Strand layout inside the thumb (V2 nodes with the V2.1 crossing order, lanes and anchors)
# =====================================================================================================================
def _setup_strands():
    TH.CMC_CROSS_X = {"ip_flex": 0.6, "ip_ext": 1.8, "mcp_flex": -0.6, "mcp_ext": -1.8}
    TH.TRACK_X = dict(TRACK)
    S = TH._setup_strands()
    for j in JOINTS:
        for role in ("flex", "ext"):
            dr = S[f"{j}_{role}"]["drum"]
            side = -1 if role == "flex" else +1
            if j == "cmc_rot":
                dr["groove_x"] = ROT_LANE[role]
            else:
                dr["groove_x"] = TRACK[j] + side * LANE_DX
            dr["anchor_v2"] = dr["anchor"]
            dr["anchor"] = float((dr["anchor"] + dr["sense"] * np.radians(CUP_DPHI)) % (2 * np.pi))
            dr["min_wrap_v21"] = dr["min_wrap_deg"] + CUP_DPHI
    return S


STRANDS = _setup_strands()


def _qdict(q):
    q = dict(q or {})
    out = {}
    for j in JOINTS:
        v = q.get(j, q.get(f"thumb_{j}", 0.0))
        out[j] = float(v)
    return out


def body_transforms(q):
    return TH.body_transforms(_qdict(q))


def body_T(q):
    F = body_transforms(q)
    out = {}
    for b, (Rm, t) in F.items():
        M = np.eye(4)
        M[:3, :3], M[:3, 3] = Rm, t
        out[b] = M
    return out


# =====================================================================================================================
# Routing outside the thumb: palm plate -> roller A -> (axial: roller B -> neck) / (rot: roller C -> drum)
# =====================================================================================================================
def _ang(a, b):
    return float(np.degrees(np.arccos(np.clip(unit(a) @ unit(b), -1, 1))))


def pulley(prev, c, nxt, r=RS):
    """Strand from point prev over a pulley (centre c, radius r, plane through prev, c, nxt) to point nxt.
    Returns entry tangent point, exit tangent point, wrap (deg), plane normal (= roller axis), arc points."""
    n = unit(np.cross(prev - c, nxt - c))

    def tan_from(p, sgn):
        v = p - c
        L = np.linalg.norm(v)
        a = np.arccos(min(1.0, r / L))
        e1, e2 = v / L, np.cross(n, v / L)
        return c + r * (np.cos(a) * e1 + sgn * np.sin(a) * e2)

    best = None
    for s in (1, -1):
        Ti, To = tan_from(prev, s), tan_from(nxt, -s)
        L = np.linalg.norm(Ti - prev) + np.linalg.norm(nxt - To)
        if best is None or L < best[0]:
            best = (L, Ti, To)
    _, Ti, To = best
    a0, a1 = unit(Ti - c), unit(To - c)
    ax = unit(np.cross(a0, a1)) if np.linalg.norm(np.cross(a0, a1)) > 1e-9 else n
    ang = np.arccos(np.clip(a0 @ a1, -1, 1))
    arc = [c + r * (np.cos(t) * a0 + np.sin(t) * np.cross(ax, a0)) for t in np.linspace(0, ang, 7)]
    return Ti, To, _ang(Ti - prev, nxt - To), n, arc


@functools.lru_cache(maxsize=1)
def route():
    """Roller centres from route_layout.json (written by optimise_route())."""
    d = json.loads(ROUTE_FILE.read_text())
    return {k: np.array(v, float) for k, v in d["centres"].items()}


def eye_point(name):
    x, y = EYE[f"thumb_{name}"]
    return np.array([x, y, Z_PLATE_TOP])


def _rot_drum_tangent(st, Q, Fq, q):
    Lt, Tw, aT, sense, wrap = TH.drum_segment(st, Q, Fq, q)
    return Tw, aT, sense, wrap


def _drum_arc(st, Tw, aT, sense, wrap, Fq, q, n=9):
    dr = st["drum"]
    fr = TH.joint_frame_q(dr["joint"], q, Fq)
    return [to_world(fr, [dr["groove_x"], R * np.cos(aT + sense * s), R * np.sin(aT + sense * s)])
            for s in np.linspace(0, wrap, n)]


def strand_path(name, q=None, detail=False):
    """World polyline of one strand (palm plate hole -> fuse-cup anchor) at pose q, plus bend data."""
    q = _qdict(q)
    Fq = body_transforms(q)
    st = STRANDS[name]
    cen = route()
    E = eye_point(name)
    info = {}
    if st["joint"] == "cmc_rot":
        cA, cC = cen[f"A_{name}"], cen[f"C_{name}"]
        # iterate: C's exit tangent toward the drum depends on the drum tangent and vice versa
        Q = cC.copy()
        for _ in range(4):
            Tw, aT, sense, wrap = _rot_drum_tangent(st, Q, Fq, q)
            Ai, Ao, wA, nA, arcA = pulley(E, cA, cC)
            Ci, Co, wC, nC, arcC = pulley(Ao, cC, Tw)
            Q = Co
        pts = [E] + arcA + arcC + _drum_arc(st, Tw, aT, sense, wrap, Fq, q)
        info = dict(plate=_ang([0, 0, 1], Ai - E), wA=wA, wC=wC, axes={"A": nA, "C": nC},
                    drum_fleet=float(np.degrees(np.arcsin(abs(unit(Tw - Co) @ A1)))), wrap=float(np.degrees(wrap)))
    else:
        cA, cB = cen[f"A_{name}"], cen[f"B_{name}"]
        Ai, Ao, wA, nA, arcA = pulley(E, cA, cB)
        Bi, Bo, wB, nB, arcB = pulley(Ao, cB, NECK_W)
        inner = [TH._wpt(nd, Fq) for nd in st["nodes"] if nd["lab"] in ("cmc_cross", "mcp_cross")]
        Qlast = inner[-1] if inner else NECK_W
        Lt, Tw, aT, sense, wrap = TH.drum_segment(st, Qlast, Fq, q)
        nxt = inner[0] if inner else Tw
        pts = [E] + arcA + arcB + [NECK_W] + inner + _drum_arc(st, Tw, aT, sense, wrap, Fq, q)
        info = dict(plate=_ang([0, 0, 1], Ai - E), wA=wA, wB=wB, neck=_ang(NECK_W - Bo, nxt - NECK_W),
                    axes={"A": nA, "B": nB}, wrap=float(np.degrees(wrap)))
    pts = np.array(pts)
    if detail:
        return pts, info
    return pts


def strand_length(name, q=None):
    p = strand_path(name, q)
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


# =====================================================================================================================
# Sensor wire (tip board -> plug), waypoints per body at q = 0 (world)
# =====================================================================================================================
def _hinge_x(j):
    c0 = TH.CHEEKS[j][0]
    S = SENSOR_SIDE[j]
    return dict(c0=c0, S=S, brg_out=c0 + TH.BRG_LIP + ZRO2["w"], face_s=c0 + TH.BRG_LIP + ZRO2["w"] + sum(STACK.values()),
                face_w=c0 + TH.BRG_LIP + TH.BRG["w"] + KEEPER_T, wire_hole=(TH.STUB_IN.get(j, 3.2) + 1.0))


def wire_waypoints():
    """(world point at q = 0, body, note) along the 4 x 38 AWG bundle."""
    W = []

    def add(fr, p, body, note):
        W.append((to_world(fr, p), body, note))

    L3 = TH.L3
    Fi, Fm, Fc = J0["ip"], J0["mcp_flex"], J0["cmc_flex"]
    hi, hm, hc = _hinge_x("ip"), _hinge_x("mcp_flex"), _hinge_x("cmc_flex")
    # distal: tip board (cavity) -> channel -> IP axle radial hole (wire side -S)
    zt = L3 - 14.8
    add(Fi, [0.0, -2.2, zt - 4.6], "distal", "tip board pads (proximal edge)")
    add(Fi, [-hi["S"] * 3.0, 0.5, zt - 9.0], "distal", "distal channel Ø1.2")
    add(Fi, [-hi["S"] * 4.2, 3.4, 4.2], "distal", "distal channel, dorsal of the IP ring")
    add(Fi, [-hi["S"] * 4.2, 2.6, 0.0], "distal", "radial hole into the IP axle tube (glue)")
    add(Fi, [-hi["S"] * 4.2, 0.0, 0.0], "distal", "IP axle bore")
    add(Fi, [-hi["S"] * (hi["face_w"] - 0.5), 0.0, 0.0], "proximal", "IP keeper plug on the axis (glue): twist only")
    add(Fi, [-hi["S"] * (hi["face_w"] - 0.3), 3.0, -4.0], "proximal", "keeper slot -> cheek groove")
    # proximal: dorsal cross to the IP board on +S, then down the S face to the MCP stub hole (MCP wire side)
    add(Fi, [-hi["S"] * 5.0, 7.6, -8.0], "proximal", "dorsal cross groove")
    add(Fi, [hi["S"] * 5.0, 7.6, -8.0], "proximal", "dorsal cross groove")
    add(Fi, [hi["S"] * (hi["face_s"] - 0.6), 3.2, -1.0], "proximal", "IP board (pads)")
    pm = to_local(Fm, to_world(Fi, [hi["S"] * 8.6, 4.4, -12.0]))
    add(Fm, [pm[0], pm[1], pm[2]], "proximal", "S face groove of the proximal")
    add(Fm, [-hm["S"] * hm["wire_hole"], 4.6, 6.5], "proximal", "proximal channel to the MCP stub")
    add(Fm, [-hm["S"] * hm["wire_hole"], 2.6, 0.0], "proximal", "radial hole into the MCP wire-side stub (glue)")
    add(Fm, [-hm["S"] * hm["wire_hole"], 0.0, 0.0], "proximal", "MCP stub bore")
    add(Fm, [-hm["S"] * (hm["face_w"] - 0.5), 0.0, 0.0], "metacarpal", "MCP keeper plug on the axis (glue): twist only")
    add(Fm, [-hm["S"] * (hm["face_w"] - 0.3), 3.2, -4.5], "metacarpal", "keeper slot -> cheek groove")
    add(Fm, [-hm["S"] * 5.0, 8.4, -9.0], "metacarpal", "dorsal cross groove (metacarpal)")
    add(Fm, [hm["S"] * 5.0, 8.4, -9.0], "metacarpal", "dorsal cross groove (metacarpal)")
    add(Fm, [hm["S"] * (hm["face_s"] - 0.6), 3.4, -1.0], "metacarpal", "MCP board (pads)")
    pc = to_local(Fc, to_world(Fm, [hm["S"] * 9.6, 6.0, -12.0]))
    add(Fc, list(pc), "metacarpal", "metacarpal side groove")
    add(Fc, [-hc["S"] * hc["wire_hole"], 6.4, 9.0], "metacarpal", "metacarpal channel to the cmc stub")
    add(Fc, [-hc["S"] * hc["wire_hole"], 2.6, 0.0], "metacarpal", "radial hole into the cmc wire-side stub (glue)")
    add(Fc, [-hc["S"] * hc["wire_hole"], 0.0, 0.0], "metacarpal", "cmc stub bore")
    add(Fc, [-hc["S"] * (hc["face_w"] - 0.5), 0.0, 0.0], "base", "cmc keeper plug on the axis (glue): twist only")
    # base: down the wire-side cheek, across under the cheek to the sensor-side cheek board, down to the sleeve hole
    add(C, [-hc["S"] * (hc["face_w"] - 0.3), -3.5, -4.0], "base", "keeper slot -> cheek groove")
    add(C, [-hc["S"] * (hc["face_w"] - 0.6), -7.6, -9.0], "base", "base cheek groove")
    add(C, [hc["S"] * (hc["face_w"] - 0.6), -7.6, -9.0], "base", "groove across the base plate (palmar)")
    add(C, [hc["S"] * (hc["face_s"] - 0.6), -3.4, -1.0], "base", "cmc_flex board (pads)")
    add(C, [hc["S"] * (hc["face_s"] - 0.6), -7.6, -9.0], "base", "base cheek groove (back down)")
    xw, yw = _sleeve_wire_xy()
    add(C, [xw, yw, -10.2], "base", "into the axial hole through drum + sleeve (r 5.6)")
    add(C, [xw, yw, CAP_Z[0] + 0.1], "base", "axial hole exit at the cap")
    # clock spring: 1.5 turns from r 6.5 (cap, rotating) to r 11.6 (mount wall, fixed); modelled at rest
    a0 = np.arctan2(yw, xw)
    for k, t in enumerate(np.linspace(0, 1, 13)[1:]):
        a = a0 + t * 3 * np.pi
        r = 6.6 + 5.0 * t
        add(C, [r * np.cos(a), r * np.sin(a), 0.5 * (CLOCK_Z[0] + CLOCK_Z[1])], "mount", "clock spring (flat, 4 wires side by side)")
    ae = a0 + 3 * np.pi
    add(C, [13.0 * np.cos(ae), 13.0 * np.sin(ae), -27.0], "mount", "through the mount wall to the rot chip board")
    add(C, [ROT_CHIP_R + 1.2, 0.0, -26.6], "mount", "cmc_rot chip board (mount wall)")
    add(C, [16.5, 2.0, -26.0], "mount", "JST SH 4-pin socket on the mount (thumb plug)")
    return W


def _sleeve_wire_xy():
    """Free azimuth for the axial wire hole at r 5.6 in the base drum/sleeve (away from the strand passages and cups)."""
    best, bestd = None, -1
    Fq = body_transforms({})
    segs = []
    for nm, st in STRANDS.items():
        if st["joint"] == "cmc_rot":
            continue
        inner = [TH._wpt(nd, Fq) for nd in st["nodes"] if nd["lab"] in ("cmc_cross", "mcp_cross")]
        if inner:
            segs.append((to_local(C, NECK_W), to_local(C, inner[0])))
        else:
            _, Tw, *_ = TH.drum_segment(st, NECK_W, Fq, {})
            segs.append((to_local(C, NECK_W), to_local(C, Tw)))
    cup_a = [STRANDS[f"cmc_rot_{r}"]["drum"]["anchor"] for r in ("flex", "ext")]
    for a in np.radians(np.arange(0, 360, 5)):
        p = np.array([5.6 * np.cos(a), 5.6 * np.sin(a)])
        d = 1e9
        for A, B in segs:
            for t in np.linspace(0, 1, 30):
                X = A + t * (B - A)
                if -26 < X[2] < -9:
                    d = min(d, np.linalg.norm(X[:2] - p))
        # cups live in the cmc_rot frame (x = a1): anchor angle in its (y, z) = C (x, y) plane
        for ca in cup_a:
            for dd in (-0.4, 0.0, 0.4):
                d = min(d, np.linalg.norm(p - 7.0 * np.array([np.cos(ca + dd), np.sin(ca + dd)])) - 1.0)
        for k in range(3):  # cap screws
            sa = np.radians(30 + 120 * k)
            d = min(d, np.linalg.norm(p - 8.2 * np.array([np.cos(sa), np.sin(sa)])) - 1.0)
        if d > bestd:
            best, bestd = (float(p[0]), float(p[1])), d
    return best


def sensor_wire(q=None):
    Fq = body_transforms(q)
    F0 = body_transforms({})
    out = []
    for p, b, _ in wire_waypoints():
        R0, t0 = F0[b]
        Rq, tq = Fq[b]
        p0 = R0.T @ (p - t0)
        out.append(Rq @ p0 + tq)
    return np.array(out)


def lines(q=None):
    out = {f"thumb_{n}": strand_path(n, q) for n in STRAND_NAMES}
    out["thumb_sensor_wire"] = sensor_wire(q)
    return out


# =====================================================================================================================
# Export contract: parts at q = 0 (hand frame) + pose(q)
# =====================================================================================================================
@functools.lru_cache(maxsize=1)
def parts():
    return json.loads((OUT / "parts.json").read_text())


def pose(q=None):
    T = body_T(q)
    return {n: T[info["body"]] for n, info in parts().items()}


def assembly_parts():
    import trimesh

    out = {}
    for n, info in parts().items():
        m = trimesh.load(STL / f"{n}.stl", force="mesh")
        out[n] = (m, f"thumb:{info['body']}", info["material"])
    return out


# =====================================================================================================================
# Routing optimiser (writes route_layout.json)
# =====================================================================================================================
def _finger_liners():
    """Finger palm liners (package A) as segments: obstacles for the thumb strands and rollers."""
    try:
        F = _load("fingers18_for_thumb", HERE / "18_fingers.py")
        segs = []
        for k, v in F.lines({}).items():
            if "wire" in k:
                continue
            v = np.asarray(v, float)[-4:]
            for i in range(len(v) - 1):
                segs.append((v[i], v[i + 1]))
        return segs
    except Exception as e:  # pragma: no cover
        print("finger liners not available:", e)
        return []


def _axis_rs(p):
    v = p - P0
    s = -(v @ A1)
    return float(np.linalg.norm(v + s * A1)), float(s)


def _seg_pt(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(ab @ ab, 1e-12), 0, 1)
    return float(np.linalg.norm(a + t * ab - p))


def _seg_seg(a0, a1, b0, b1, n=9):
    t = np.linspace(0, 1, n)
    A = a0 + (a1 - a0) * t[:, None]
    B = b0 + (b1 - b0) * t[:, None]
    return float(np.linalg.norm(A[:, None] - B[None], axis=-1).min())


RF = 4.3  # roller + fork keep-out sphere radius (MR63 r 3 + 1.3 for the fork cheeks)


def _route_eval(cen, liners, verbose=False):
    """cost = sliding bends (plate + neck, deg) + 0.03 x rolling bends + penalties (clearances, keep-outs)."""
    paths, infos = {}, {}
    ROUTE_CACHE = cen
    cost, pen = 0.0, 0.0
    spheres = []
    for n in STRAND_NAMES:
        st = STRANDS[n]
        E = eye_point(n)
        if st["joint"] == "cmc_rot":
            cA, cC = cen[f"A_{n}"], cen[f"C_{n}"]
            Ai, Ao, wA, _, _ = pulley(E, cA, cC)
            Fq = body_transforms({})
            Q = cC
            for _ in range(3):
                _, Tw, *_ = TH.drum_segment(st, Q, Fq, {})
                Ci, Co, wC, _, _ = pulley(Ao, cC, Tw)
                Q = Co
            fleet = float(np.degrees(np.arcsin(abs(unit(Tw - Co) @ A1))))
            pts = [E, Ai, Ao, Ci, Co, Tw]
            plate = _ang([0, 0, 1], Ai - E)
            cost += plate + 0.03 * (wA + wC) + 2.0 * fleet
            spheres += [(n, cA), (n, cC)]
            infos[n] = dict(plate=plate, wA=wA, wC=wC, fleet=fleet)
        else:
            cA, cB = cen[f"A_{n}"], cen[f"B_{n}"]
            Ai, Ao, wA, _, _ = pulley(E, cA, cB)
            Bi, Bo, wB, _, _ = pulley(Ao, cB, NECK_W)
            plate = _ang([0, 0, 1], Ai - E)
            neck = _ang(NECK_W - Bo, A1)
            cost += plate + neck + 0.03 * (wA + wB)
            pts = [E, Ai, Ao, Bi, Bo, NECK_W]
            spheres += [(n, cA), (n, cB)]
            infos[n] = dict(plate=plate, neck=neck, wA=wA, wB=wB)
        paths[n] = pts
    segs = []
    for n, pts in paths.items():
        for i in range(len(pts) - 1):
            a, b = np.asarray(pts[i]), np.asarray(pts[i + 1])
            if np.linalg.norm(b - a) < 1e-6:
                continue
            if i == 0:
                a = a + 3.0 * unit(b - a)
            segs.append((n, a, b, np.allclose(b, NECK_W)))
    for i, (n, c0) in enumerate(spheres):
        for (m, a, b, _) in segs:
            if m != n:
                pen += max(0.0, RF + 0.3 - _seg_pt(c0, a, b)) ** 2
        for a, b in liners:
            pen += max(0.0, RF + 1.4 - _seg_pt(c0, a, b)) ** 2
        for j in range(i + 1, len(spheres)):
            pen += max(0.0, 2 * RF - np.linalg.norm(c0 - spheres[j][1])) ** 2
        r_, s = _axis_rs(c0)
        if s < -MOUNT_Z[0] + RF:
            pen += max(0.0, (15.5 if s > 8 else 19.0) + RF - r_) ** 2
        if abs(c0[0]) < 14.6 + RF and abs(c0[1]) < 9.8 + RF:
            pen += max(0.0, Z_PLATE_TOP + RF + 0.6 - c0[2]) ** 2
        pen += max(0.0, abs(c0[1]) - (11.5 - RF)) ** 2 + max(0.0, 17.5 + RF - c0[2]) ** 2 + max(0.0, c0[0] - 27.0) ** 2
    for ii in range(len(segs)):
        n, a, b, nN = segs[ii]
        for (a2, b2) in liners:
            pen += max(0.0, 1.9 - _seg_seg(a, b, a2, b2)) ** 2
        for t in np.linspace(0, 1, 7):
            p = a + (b - a) * t
            if nN and np.linalg.norm(p - NECK_W) < 10:
                continue
            r_, s = _axis_rs(p)
            if 8 < s < -MOUNT_Z[0]:
                pen += max(0.0, 16.0 - r_) ** 2
            elif s <= 8:
                pen += max(0.0, 19.5 - r_) ** 2
        for jj in range(ii + 1, len(segs)):
            m, a2, b2, nN2 = segs[jj]
            if m == n or (nN and nN2):
                continue
            pen += max(0.0, 0.9 - _seg_seg(a, b, a2, b2)) ** 2
    if verbose:
        return cost, pen, infos
    return cost + 200.0 * pen


def optimise_route(x0=None, restarts=3, maxiter=20000):
    from scipy.optimize import minimize

    liners = _finger_liners()
    keys = [f"A_{n}" for n in STRAND_NAMES] + [f"B_{n}" for n in STRAND_NAMES if "cmc_rot" not in n] + \
           [f"C_{n}" for n in STRAND_NAMES if "cmc_rot" in n]
    if x0 is None:
        x0 = []
        for k in keys:
            n = k[2:]
            E = eye_point(n)
            if k.startswith("A"):
                x0 += [E[0] + 9.0, E[1] * 1.6, 44.0 + STRAND_NAMES.index(n)]
            elif k.startswith("B"):
                x0 += list(NECK_W - 9 * A1 + np.array([-5 + 1.2 * STRAND_NAMES.index(n), 0, 0]))
            else:
                x0 += list(to_world(C, [-14.0, 8.0 + 3 * STRAND_NAMES.index(n), ROT_LANE[n.split("_")[-1]]]))
        x0 = np.array(x0)

    def f(v):
        return _route_eval({k: v[3 * i:3 * i + 3] for i, k in enumerate(keys)}, liners)

    rng = np.random.default_rng(0)
    best = None
    for trial in range(restarts):
        v0 = x0 + (rng.normal(0, 2.0, x0.shape) if trial else 0)
        r = minimize(f, v0, method="Powell", options=dict(maxiter=maxiter, xtol=1e-2, ftol=1e-4))
        print(f"  route trial {trial}: {r.fun:.2f}", flush=True)
        if best is None or r.fun < best.fun:
            best = r
    cen = {k: best.x[3 * i:3 * i + 3] for i, k in enumerate(keys)}
    c, pen, infos = _route_eval(cen, liners, verbose=True)
    OUT.mkdir(parents=True, exist_ok=True)
    ROUTE_FILE.write_text(json.dumps({"centres": {k: [float(x) for x in v] for k, v in cen.items()},
                                      "cost": c, "penalty": pen, "bends": infos}, indent=1))
    route.cache_clear()
    return cen, c, pen, infos


if __name__ == "__main__" and "--route" in sys.argv:
    t0 = time.time()
    x0 = None
    if ROUTE_FILE.exists():
        d = json.loads(ROUTE_FILE.read_text())["centres"]
        keys = [f"A_{n}" for n in STRAND_NAMES] + [f"B_{n}" for n in STRAND_NAMES if "cmc_rot" not in n] + \
               [f"C_{n}" for n in STRAND_NAMES if "cmc_rot" in n]
        x0 = np.concatenate([d[k] for k in keys])
    cen, c, pen, infos = optimise_route(x0)
    print(f"route: cost {c:.1f}, penalty {pen:.4f}, {time.time() - t0:.0f} s")
    for n, i in infos.items():
        print(" ", n, {k: round(v, 1) for k, v in i.items()})
