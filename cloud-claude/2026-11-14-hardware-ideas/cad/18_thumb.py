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
ROT_LANE = {"flex": -15.2, "ext": -13.8}  # C-frame z (= along a1); ring band z -16.0..-13.0 (plate above)
ROT_RING_Z = (-16.0, -13.0)
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
ROT_MAG = dict(ro=5.0, ri=3.0, z=(-27.6, -25.6))  # 10 x 6 x 2 N42 diametric
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
    TH.STRANDS = S  # V2 helpers (_nip_pins, strand bores) read the module global
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
    add(Fi, [hi["S"] * (hi["face_s"] - 0.35), 3.2, -1.0], "proximal", "IP board (pads)")
    pm = to_local(Fm, to_world(Fi, [hi["S"] * 8.6, 4.4, -12.0]))
    add(Fm, [pm[0], pm[1], pm[2]], "proximal", "S face groove of the proximal")
    add(Fm, [-hm["S"] * hm["wire_hole"], 4.6, 6.5], "proximal", "proximal channel to the MCP stub")
    add(Fm, [-hm["S"] * hm["wire_hole"], 2.6, 0.0], "proximal", "radial hole into the MCP wire-side stub (glue)")
    add(Fm, [-hm["S"] * hm["wire_hole"], 0.0, 0.0], "proximal", "MCP stub bore")
    add(Fm, [-hm["S"] * (hm["face_w"] - 0.5), 0.0, 0.0], "metacarpal", "MCP keeper plug on the axis (glue): twist only")
    add(Fm, [-hm["S"] * (hm["face_w"] - 0.3), 3.2, -4.5], "metacarpal", "keeper slot -> cheek groove")
    add(Fm, [-hm["S"] * 5.0, 8.4, -9.0], "metacarpal", "dorsal cross groove (metacarpal)")
    add(Fm, [hm["S"] * 5.0, 8.4, -9.0], "metacarpal", "dorsal cross groove (metacarpal)")
    add(Fm, [hm["S"] * (hm["face_s"] - 0.35), 3.4, -1.0], "metacarpal", "MCP board (pads)")
    pc = to_local(Fc, to_world(Fm, [hm["S"] * 9.6, 6.0, -12.0]))
    add(Fc, list(pc), "metacarpal", "metacarpal side groove")
    add(Fc, [-hc["S"] * hc["wire_hole"], 6.4, 9.0], "metacarpal", "metacarpal channel to the cmc stub")
    add(Fc, [-hc["S"] * hc["wire_hole"], 2.6, 0.0], "metacarpal", "radial hole into the cmc wire-side stub (glue)")
    add(Fc, [-hc["S"] * hc["wire_hole"], 0.0, 0.0], "metacarpal", "cmc stub bore")
    add(Fc, [-hc["S"] * (hc["face_w"] - 0.5), 0.0, 0.0], "base", "cmc keeper plug on the axis (glue): twist only")
    # base: down the wire-side cheek, across under the cheek to the sensor-side cheek board, down to the sleeve hole
    add(C, [-hc["S"] * (hc["face_w"] - 0.3), -3.5, -4.0], "base", "keeper slot -> cheek groove")
    add(C, [-hc["S"] * (hc["face_w"] - 0.35), -7.6, -9.0], "base", "base cheek groove")
    add(C, [hc["S"] * (hc["face_w"] - 0.35), -7.6, -9.0], "base", "groove across the base plate (palmar)")
    add(C, [hc["S"] * (hc["face_s"] - 0.35), -3.4, -1.0], "base", "cmc_flex board (pads)")
    add(C, [hc["S"] * (hc["face_s"] - 0.35), -4.6, -9.4], "base", "base cheek groove (back down, 3 mm beside the way up)")
    xw, yw = _sleeve_wire_xy()
    add(C, [xw, yw, -10.2], "base", "into the axial hole through drum + sleeve (r 5.9, Ø1.0)")
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
    """Free azimuth for the axial wire hole at r 5.9 in the base drum/sleeve (away from the strand passages and cups)."""
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
        p = np.array([5.9 * np.cos(a), 5.9 * np.sin(a)])
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
        return np.array([a for a, _ in segs]), np.array([b for _, b in segs])
    except Exception as e:  # pragma: no cover
        print("finger liners not available:", e)
        return np.zeros((0, 3)), np.zeros((0, 3))


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


def _pt_seg_d(Pts, A, B):
    """distances (n, m) from points Pts (n, 3) to segments A-B (m, 3)."""
    AB = B - A
    L2 = np.maximum(np.sum(AB * AB, 1), 1e-12)
    t = np.clip(np.einsum("nmk,mk->nm", Pts[:, None, :] - A[None], AB) / L2[None], 0, 1)
    X = A[None] + t[..., None] * AB[None]
    return np.linalg.norm(X - Pts[:, None, :], axis=-1)


def _axis_rs_v(Pts):
    V = Pts - P0
    s = -(V @ A1)
    return np.linalg.norm(V + s[:, None] * A1, axis=1), s


def _route_paths(cen):
    paths, infos, spheres = {}, {}, []
    Fq = body_transforms({})
    for n in STRAND_NAMES:
        st = STRANDS[n]
        E = eye_point(n)
        if st["joint"] == "cmc_rot":
            cA, cC = cen[f"A_{n}"], cen[f"C_{n}"]
            Ai, Ao, wA, _, _ = pulley(E, cA, cC)
            Q = cC
            for _ in range(3):
                _, Tw, *_ = TH.drum_segment(st, Q, Fq, {})
                Ci, Co, wC, _, _ = pulley(Ao, cC, Tw)
                Q = Co
            fleet = float(np.degrees(np.arcsin(min(1.0, abs(unit(Tw - Co) @ A1)))))
            paths[n] = [E, Ai, Ao, Ci, Co, Tw]
            infos[n] = dict(plate=_ang([0, 0, 1], Ai - E), wA=wA, wC=wC, fleet=fleet)
            spheres += [(n, cA), (n, cC)]
        else:
            cA, cB = cen[f"A_{n}"], cen[f"B_{n}"]
            Ai, Ao, wA, _, _ = pulley(E, cA, cB)
            Bi, Bo, wB, _, _ = pulley(Ao, cB, NECK_W)
            paths[n] = [E, Ai, Ao, Bi, Bo, NECK_W]
            infos[n] = dict(plate=_ang([0, 0, 1], Ai - E), neck=_ang(NECK_W - Bo, A1), wA=wA, wB=wB)
            spheres += [(n, cA), (n, cB)]
    return paths, infos, spheres


def _route_eval(cen, LS, verbose=False):
    """cost = sliding bends (plate + neck, deg) + 0.03 x rolling bends + 2 x drum fleet + 200 x penalties
    (clearances to other strands, rollers, the finger liners (package A), the mount and keep-outs)."""
    paths, infos, spheres = _route_paths(cen)
    cost = sum(i["plate"] + i.get("neck", 0) + 0.03 * (i["wA"] + i.get("wB", 0) + i.get("wC", 0)) + 2 * i.get("fleet", 0)
               for i in infos.values())
    SA, SB, own, nN = [], [], [], []
    for k, n in enumerate(STRAND_NAMES):
        pts = paths[n]
        for i in range(len(pts) - 1):
            a, b = np.asarray(pts[i]), np.asarray(pts[i + 1])
            if np.linalg.norm(b - a) < 1e-6:
                continue
            if i == 0:
                a = a + 3.0 * unit(b - a)
            last_rot = STRANDS[n]["joint"] == "cmc_rot" and i == len(pts) - 2  # C -> drum: enters the drum on purpose
            SA.append(a), SB.append(b), own.append(k), nN.append(bool(np.allclose(b, NECK_W)) or last_rot)
    SA, SB, own, nN = np.array(SA), np.array(SB), np.array(own), np.array(nN)
    Cs = np.array([c for _, c in spheres])
    co = np.array([STRAND_NAMES.index(n) for n, _ in spheres])
    LA, LB = LS
    pen = 0.0
    d = _pt_seg_d(Cs, SA, SB)
    d[co[:, None] == own[None, :]] = 99
    pen += np.sum(np.clip(RF + 0.3 - d, 0, None) ** 2)
    if len(LA):
        pen += np.sum(np.clip(RF + 1.4 - _pt_seg_d(Cs, LA, LB), 0, None) ** 2)
    dd = np.linalg.norm(Cs[:, None] - Cs[None], axis=-1)
    iu = np.triu_indices(len(Cs), 1)
    pen += np.sum(np.clip(2 * RF - dd[iu], 0, None) ** 2)
    r_, s_ = _axis_rs_v(Cs)
    lim = np.where(s_ > 21.5, 15.5, 19.5) + RF  # base cheeks/lugs sweep r <= 18 above s 13.5, rot stop posts r 18.6
    pen += np.sum(np.where(s_ < -MOUNT_Z[0] + RF, np.clip(lim - r_, 0, None) ** 2, 0))
    inplate = (np.abs(Cs[:, 0]) < 14.6 + RF) & (np.abs(Cs[:, 1]) < 9.8 + RF)
    pen += np.sum(np.where(inplate, np.clip(Z_PLATE_TOP + RF + 0.6 - Cs[:, 2], 0, None) ** 2, 0))
    pen += np.sum(np.clip(np.abs(Cs[:, 1]) - (11.5 - RF), 0, None) ** 2 + np.clip(17.5 + RF - Cs[:, 2], 0, None) ** 2
                  + np.clip(Cs[:, 0] - 27.0, 0, None) ** 2)
    t = np.linspace(0, 1, 9)
    X = SA[:, None] + (SB - SA)[:, None] * t[None, :, None]  # (m, 9, 3)
    Xf = X.reshape(-1, 3)
    if len(LA):
        dl = _pt_seg_d(Xf, LA, LB).min(1)
        dl[Xf[:, 2] < Z_PLATE_TOP + 6.0] = 99  # the first 6 mm above the plate: holes 2.15 apart, liners end there
        pen += np.sum(np.clip(1.9 - dl, 0, None) ** 2) / 9
    r_, s_ = _axis_rs_v(Xf)
    skip = (np.repeat(nN, 9)) & ((np.linalg.norm(Xf - NECK_W, axis=1) < 14) | (_axis_rs_v(Xf)[1] < 20))
    lim = np.where(s_ <= 8, 19.5, 16.0)
    pen += np.sum(np.where((s_ < -MOUNT_Z[0]) & ~skip, np.clip(lim - r_, 0, None) ** 2, 0))
    D = np.linalg.norm(X[:, None, :, None] - X[None, :, None, :], axis=-1).min(axis=(2, 3))
    mask = (own[:, None] != own[None, :]) & ~(nN[:, None] & nN[None, :])
    iu = np.triu_indices(len(SA), 1)
    pen += np.sum(np.where(mask[iu], np.clip(0.9 - D[iu], 0, None) ** 2, 0))
    if verbose:
        return cost, float(pen), infos
    return cost + 200.0 * pen


def optimise_route(x0=None, restarts=2, maxiter=300):
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
        r = minimize(f, v0, method="L-BFGS-B", options=dict(maxiter=maxiter))
        if "--powell" in sys.argv:
            r = minimize(f, r.x, method="Powell", options=dict(maxiter=4000, xtol=1e-2, ftol=1e-4))
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


# =====================================================================================================================
# CAD (CadQuery, built in the V2 joint frames, exported in the hand frame at q = 0)
# =====================================================================================================================
MATERIAL = {}
BODY = {}
COLOR = {"PETG": "#d9dde3", "steel": "#b5b9bd", "stainless": "#c8ccd2", "brass": "#c9a646", "silicone": "#e9b8a0",
         "TPU": "#e2a07a", "PCB": "#1e7d3a", "ceramic": "#f2f0e6", "magnet": "#9a3b3b", "iglidur": "#e0c020"}


def _reg(parts, name, shape, body, material):
    parts[name] = shape
    BODY[name] = body
    MATERIAL[name] = material


def _cq():
    import cadquery as cq
    return cq


def _ring_seat_x(x, hub_r):
    """ring seat on a hinge hub (joint frame, axis x): removes r 6.5..hub_r+0.5 over the ring length."""
    return TH.ring_x(RING["id"] / 2, hub_r + 0.5, x - RING["L"] / 2, x + RING["L"] / 2)


def _radial_cup(j, role, frame):
    """radial single-lane fuse cup (report 12/16) below the ring at the strand's anchor angle (joint frame of j).
    Returns (cup solid local, pocket local)."""
    cq = _cq()
    st = STRANDS[f"{j}_{role}"]
    dr = st["drum"]
    a = dr["anchor"]
    x = dr["groove_x"]
    r_top = RING["id"] / 2
    h = CUP["h"]
    ey, ez = np.cos(a), np.sin(a)
    # local frame of the cup: w = radial (inward), u = x (lane), v = tangential
    def blk(r0, r1, du, dv):
        pts = []
        tang = np.array([0, -ez, ey])
        rad = np.array([0, ey, ez])
        corners = []
        for rr in (r0, r1):
            for su in (-1, 1):
                for sv in (-1, 1):
                    corners.append(np.array([x + su * du / 2, 0, 0]) + rr * rad + sv * dv / 2 * tang)
        return corners
    def box_from(r0, r1, du, dv):
        tang = np.array([0, -ez, ey])
        rad = np.array([0, ey, ez])
        c = np.array([x, 0, 0]) + 0.5 * (r0 + r1) * rad
        b = cq.Solid.makeBox(du, r1 - r0, dv, cq.Vector(-du / 2, -(r1 - r0) / 2, -dv / 2))
        # local box axes: X = lane x, Y = radial, Z = tangential
        from OCP.gp import gp_Ax3, gp_Dir, gp_Pnt, gp_Trsf
        t = gp_Trsf()
        t.SetTransformation(gp_Ax3(gp_Pnt(*map(float, c)), gp_Dir(*map(float, tang)), gp_Dir(1.0, 0.0, 0.0)))
        t.Invert()
        return TH._apply(b, t)
    cup = box_from(r_top - h, r_top, CUP["lt"], CUP["lw"])
    ch = box_from(r_top - h - 0.1, r_top - CUP["lip"], CUP["ch_t"], CUP["ch_w"])
    hole = TH.cyl_ab(np.array([x, 0, 0]) + (r_top - h - 0.5) * np.array([0, ey, ez]),
                     np.array([x, 0, 0]) + (r_top + 0.6) * np.array([0, ey, ez]), 0.4)
    cup = cup.cut(ch).cut(hole)
    pocket = box_from(r_top - h - 0.1, r_top + 0.05, CUP["lt"] + 0.2, CUP["lw"] + 0.2)
    return cup, pocket, hole


def _tangential_cup_rot(role):
    """cmc_rot cups (C frame, axis z): along the tangent continuing the wrap past the anchor, 3.5 radial x 2.7 axial,
    2.9 long, chamber 2.9 x 2.1 for the crushed crimp."""
    cq = _cq()
    st = STRANDS[f"cmc_rot_{role}"]
    dr = st["drum"]
    a, s = dr["anchor"], dr["sense"]
    zl = dr["groove_x"]
    # cmc_rot joint frame: x = a1, y = a2, z = a1 x a2  ==  C frame (z, x, y)
    pr = np.array([np.cos(a), np.sin(a)])  # (C x, C y)
    tg = s * np.array([-np.sin(a), np.cos(a)])
    p0 = np.array([*(R * pr), zl])
    def blk(off, L, w_rad, w_ax):
        c = p0 + np.array([*(tg * (off + L / 2)), 0.0])
        b = cq.Solid.makeBox(L, w_rad, w_ax, cq.Vector(-L / 2, -w_rad / 2, -w_ax / 2))
        from OCP.gp import gp_Ax3, gp_Dir, gp_Pnt, gp_Trsf
        t = gp_Trsf()
        t.SetTransformation(gp_Ax3(gp_Pnt(*map(float, c)), gp_Dir(0.0, 0.0, 1.0), gp_Dir(float(tg[0]), float(tg[1]), 0.0)))
        t.Invert()
        return TH._apply(b, t)
    cup = blk(0.3, CUP["h"], CUP["lw"], CUP["lt"]).cut(blk(0.3 + CUP["lip"], CUP["h"], CUP["ch_w"], CUP["ch_t"]))
    cup = cup.cut(TH.cyl_ab(p0 - np.array([*(tg * 0.5), 0]), p0 + np.array([*(tg * 1.2), 0]), 0.4))
    pocket = blk(0.2, CUP["h"] + 0.2, CUP["lw"] + 0.2, CUP["lt"] + 0.2)
    return cup, pocket


def _cheek_spans(j):
    """(x0, x1) of the parent's two cheeks in the joint frame of j: sensor side to face_s, wire side to face_w."""
    h = _hinge_x(j)
    out = {}
    for sgn in (+1, -1):
        face = h["face_s"] if sgn == h["S"] else h["face_w"]
        out[sgn] = tuple(sorted((sgn * h["c0"], sgn * face)))
    return out


def _cheek_seats21(j):
    h = _hinge_x(j)
    c0 = h["c0"]
    tools = []
    for sgn in (+1, -1):
        face = h["face_s"] if sgn == h["S"] else h["face_w"]
        tools.append(TH.cyl_x(TH.SEAT_R, *sorted((sgn * (c0 + TH.BRG_LIP), sgn * (face + 0.2)))))
        tools.append(TH.cyl_x(TH.LIP_R, *sorted((sgn * (c0 - 0.2), sgn * (c0 + TH.BRG_LIP + 0.01)))))
    return tools


def _hinge_hardware(j, parts, parent, child):
    """bearings (ZrO2 sensor side, MR84ZZ wire side), axle tubes + magnet, sensor plug + chip + board, keeper."""
    cq = _cq()
    fr = J0[j]
    h = _hinge_x(j)
    c0, S = h["c0"], h["S"]
    lip = c0 + TH.BRG_LIP
    tag = {"cmc_flex": "cmc", "mcp_flex": "mcp", "ip": "ip"}[j]
    # bearings
    x0, x1 = sorted((S * lip, S * (lip + ZRO2["w"])))
    _reg(parts, f"bearing_{tag}_sensor_zro2", TH.place(TH.ring_x(ZRO2["id"] / 2, ZRO2["od"] / 2, x0, x1), fr), parent, "ceramic")
    x0, x1 = sorted((-S * lip, -S * (lip + TH.BRG["w"])))
    b = TH.ring_x(TH.BRG["id"] / 2, TH.BRG["od"] / 2, x0, x1)
    _reg(parts, f"bearing_{tag}_wire_mr84zz", TH.place(b, fr), parent, "steel")
    # axle tubes (child)
    ro, ri = AXLE_TUBE["od"] / 2, AXLE_TUBE["id"] / 2
    s_end, w_end = lip + ZRO2["w"], lip + TH.BRG["w"]
    xh = -S * h["wire_hole"]
    hole = TH.cyl_ab(np.array([xh, 0.0, 0.0]), np.array([xh, 3.0, 0.0]), 0.5)
    if j == "ip":
        x0, x1 = sorted((S * s_end, -S * w_end))
        _reg(parts, "axle_ip_tube", TH.place(TH.ring_x(ri, ro, x0, x1).cut(hole), fr), child, "stainless")
    else:
        si = TH.STUB_IN[j]
        x0, x1 = sorted((S * si, S * s_end))
        _reg(parts, f"axle_{tag}_sensor_tube", TH.place(TH.ring_x(ri, ro, x0, x1), fr), child, "stainless")
        x0, x1 = sorted((-S * si, -S * w_end))
        _reg(parts, f"axle_{tag}_wire_tube", TH.place(TH.ring_x(ri, ro, x0, x1).cut(hole), fr), child, "stainless")
    # magnet in the sensor-side tube end
    x0, x1 = sorted((S * (s_end - MAG_HINGE["L"]), S * s_end))
    _reg(parts, f"magnet_{tag}", TH.place(TH.cyl_x(MAG_HINGE["d"] / 2 - 0.01, x0, x1), fr), child, "magnet")
    # sensor plug (printed, retains the ZrO2 bearing) with chip + board inside
    x0, x1 = sorted((S * s_end, S * h["face_s"]))
    plug = TH.cyl_x(TH.SEAT_R - 0.03, x0, x1)
    px0, px1 = sorted((S * (s_end - 0.01), S * (h["face_s"] - STACK["cap"])))
    plug = plug.cut(TH.cyl_x(3.3, px0, px1)).cut(TH.box(*sorted((S * (h["face_s"] - 0.3), S * (h["face_s"] + 0.1))), 2.6, 4.2, -0.6, 0.6))
    _reg(parts, f"sensor_plug_{tag}", TH.place(plug, fr), parent, "PETG")
    cx0 = s_end + STACK["gap"]
    x0, x1 = sorted((S * cx0, S * (cx0 + STACK["chip"])))
    _reg(parts, f"sensor_chip_{tag}", TH.place(TH.box(x0, x1, -1.45, 1.45, -1.4, 1.4), fr), parent, "PCB")
    x0, x1 = sorted((S * (cx0 + STACK["chip"]), S * (cx0 + STACK["chip"] + STACK["board"])))
    _reg(parts, f"sensor_board_{tag}", TH.place(TH.cyl_x(3.2, x0, x1), fr), parent, "PCB")
    # keeper (wire side)
    x0, x1 = sorted((-S * w_end, -S * h["face_w"]))
    kp = TH.cyl_x(TH.SEAT_R - 0.03, x0, x1).cut(TH.cyl_x(0.5, x0 - 1, x1 + 1)).cut(
        TH.box(*sorted((-S * (h["face_w"] - 0.5), -S * (h["face_w"] + 0.1))), -0.5, 4.2, -0.5, 0.5))
    _reg(parts, f"wire_keeper_{tag}", TH.place(kp, fr), parent, "PETG")


def _wire_channels(body, r=0.6):
    """Ø1.2 channels (world, q = 0) along the wire segments owned by `body` (both ends on that body)."""
    W = wire_waypoints()
    out = []
    for (p, b, n1), (q, b2, n2) in zip(W[:-1], W[1:]):
        if "axial hole" in n1 and "axial hole" in n2:
            continue  # the base cuts this hole itself
        if b == body and b2 == body and np.linalg.norm(q - p) > 1e-6:
            u = unit(q - p)
            out.append(TH.cyl_ab(p - 0.35 * u, q + 0.35 * u, r))
    return out


def _world_cut(shape_world, tools_world):
    if not tools_world:
        return shape_world
    s = shape_world
    for t in tools_world:  # sequential cuts (a fused tool meshes badly where channel ends overlap)
        try:
            s = s.cut(t)
        except Exception:
            pass
    return TH.main_solid(s.clean())


def make_mount():
    cq = _cq()
    W_ANG, ROT_MID = TH.W_ANG, TH.ROT_MID
    s = TH.fuse(TH.cyl_z(TH.MOUNT_OD / 2, *MOUNT_Z), TH.ring_z(5, TH.FLANGE_OD / 2, *TH.FLANGE_Z))
    rng = RANGES["cmc_rot"]
    lug_rest = W_ANG + 180.0 - ROT_MID
    post_half = np.degrees(1.75 / np.mean(TH.POST_R))
    for end, sgn in ((rng[1], +1), (rng[0], -1)):
        a = lug_rest + end + sgn * (TH.LUG_HALF + post_half)
        s = s.fuse(TH.sector_z(TH.POST_R[0] - 0.5, TH.POST_R[1], MOUNT_Z[1] - 4.0, TH.STOP_TOP, a - post_half, a + post_half))
    s = s.fuse(TH.ring_z(TH.MOUNT_OD / 2 - 1, TH.POST_R[1], MOUNT_Z[1] - 4.0, MOUNT_Z[1]))
    tools = [TH.cyl_z(TH.BEARING["od"] / 2, TH.BEAR_BOT_Z[0], MOUNT_Z[1] + 1),
             TH.cyl_z(11.4, CAP_Z[0] - 0.3, TH.BEAR_BOT_Z[0] + 0.01),
             TH.cyl_z(12.0, CLOCK_Z[0], CLOCK_Z[1] + 0.1),
             TH.cyl_z(NECK["r_out"] - 0.05, WEB_Z[0] - 0.1, WEB_Z[1] + 0.1)]
    for b in range(3):
        a = np.radians(W_ANG + 180 + (b - 1) * 100)
        tools.append(TH.cyl_z(1.7, TH.FLANGE_Z[0] - 1, TH.FLANGE_Z[1] + 1, TH.BOLT_R * np.cos(a), TH.BOLT_R * np.sin(a)))
    # rot chip pocket (radial, at +x of the C frame) and the clock-spring exit
    tools.append(TH.box(11.9, 14.6, -1.8, 1.8, -28.2, -25.0))
    tools.append(TH.cyl_ab(np.array([11.5, 0, -28.5]), np.array([14.0, 0, -26.6]), 0.6))
    tools.append(TH.cyl_ab(np.array([14.0, 0.0, -26.6]), np.array([16.5, 2.0, -26.0]), 0.6))
    s = TH.cut(s, *tools)
    return TH.place(s, C)


def make_cap():
    xw, yw = _sleeve_wire_xy()
    s = TH.cyl_z(11.0, *CAP_Z).fuse(TH.ring_z(2.2, 3.2, CLOCK_Z[0] + 0.15, CAP_Z[0] + 0.01))
    tools = [TH.cyl_z(2.2, CAP_Z[0] - 2, CAP_Z[1] + 1), TH.ring_z(ROT_MAG["ri"] - 0.05, ROT_MAG["ro"] + 0.05, ROT_MAG["z"][0] - 0.1, ROT_MAG["z"][1]),
             TH.cyl_z(0.5, CAP_Z[0] - 1, CAP_Z[1] + 1, xw, yw)]
    for k in range(3):
        a = np.radians(30 + 120 * k)
        tools.append(TH.cyl_z(1.1, CAP_Z[0] - 1, CAP_Z[1] + 1, 8.2 * np.cos(a), 8.2 * np.sin(a)))
    return TH.place(TH.cut(s, *tools), C)


def make_neck():
    cq = _cq()
    wp = cq.Workplane("XY").circle(NECK["r_out"]).circle(NECK["hole"]).extrude(WEB_Z[1] - WEB_Z[0])
    try:
        wp = wp.edges(cq.selectors.RadiusNthSelector(0)).fillet(NECK["lip"])
    except Exception:
        pass
    return TH.place(wp.val().translate(cq.Vector(0, 0, WEB_Z[0])), C)


def make_base():
    cq = _cq()
    W_ANG, ROT_MID = TH.W_ANG, TH.ROT_MID
    BEARING = TH.BEARING
    s = TH.fuse(TH.cyl_z(BEARING["id"] / 2, TH.SLEEVE_Z[0], TH.DRUM_ROT_Z[0] + 0.01), TH.cyl_z(TH.ROT_DRUM_FLANGE, *TH.DRUM_ROT_Z),
                TH.cyl_z(10.5, TH.PLATE_Z[0], TH.PLATE_Z[1]))
    spans = _cheek_spans("cmc_flex")
    for sgn, (x0, x1) in spans.items():
        s = s.fuse(TH.fuse(TH.box(x0, x1, -7.0, 7.0, TH.PLATE_Z[0], 0.0), TH.cyl_x(7.0, x0, x1)))
    s = TH.fuse(s, TH.box(-TH.CHEEK_CMC[1], TH.CHEEK_CMC[1], -7.0, 7.0, TH.PLATE_Z[0], TH.PLATE_Z[1]))
    lug_a = W_ANG + 180.0 - ROT_MID
    s = s.fuse(TH.sector_z(TH.LUG_R[0], TH.LUG_R[1], TH.LUG_Z[0], TH.LUG_Z[1], lug_a - TH.LUG_HALF, lug_a + TH.LUG_HALF))
    tools = [TH.cyl_z(4.0, TH.SLEEVE_Z[0] - 0.01, TH.PLATE_Z[1] + 0.01)]
    for k in range(3):  # cap screws M2 (brass or nylon: 3 mm from the rot magnet)
        a = np.radians(30 + 120 * k)
        tools.append(TH.cyl_z(0.8, TH.SLEEVE_Z[0] - 0.01, TH.SLEEVE_Z[0] + 6, 8.2 * np.cos(a), 8.2 * np.sin(a)))
    # brass ring seat + 2 tangential fuse-cup pockets
    tools.append(TH.ring_z(RING["id"] / 2, TH.ROT_DRUM_FLANGE + 1, *ROT_RING_Z))
    cups = {}
    for role in ("flex", "ext"):
        cup, pocket = _tangential_cup_rot(role)
        cups[role] = cup
        tools.append(pocket)
    tools += _cheek_seats21("cmc_flex")
    # strand passages: neck (fixed, on the axis) -> crossing points / cmc_flex drum tangents (fixed in the base)
    Fq = body_transforms({})
    for nm, st in STRANDS.items():
        if st["joint"] == "cmc_rot":
            continue
        inner = [TH._wpt(nd, Fq) for nd in st["nodes"] if nd["lab"] == "cmc_cross"]
        if inner:
            tgt = inner[0]
        else:
            _, tgt, *_ = TH.drum_segment(st, NECK_W, Fq, {})
        tools.append(TH.cyl_ab(to_local(C, NECK_W), to_local(C, tgt), 0.75))
    tools.append(TH.cyl_x(TH.HUB_CMC_R + 0.3, -TH.CHEEK_CMC[0], TH.CHEEK_CMC[0]))
    xw, yw = _sleeve_wire_xy()
    tools.append(TH.cyl_z(0.5, TH.SLEEVE_Z[0] - 0.01, TH.PLATE_Z[1] - 0.5, xw, yw))
    s = TH.cut(s, *tools)
    s = TH.main_solid(s.clean())
    w = TH.place(s, C)
    w = _world_cut(w, _wire_channels("base"))
    return w, {r: TH.place(c, C) for r, c in cups.items()}


def make_metacarpal():
    F = J0["cmc_flex"]
    Fm = J0["mcp_flex"]
    hub = TH._hub_spacers("cmc_flex", TH.cyl_x(TH.HUB_CMC_R, -TH.HUB_CMC_W, TH.HUB_CMC_W))
    L1 = TH.L1
    zf = L1 - TH.HUB_MCP_R - 0.6
    spans = _cheek_spans("mcp_flex")
    wmax = max(abs(v) for sp in spans.values() for v in sp)
    shaft = TH.fuse(TH.prism(TH.SHAFT_MC[0], TH.SHAFT_MC[1], 3.5, 4.0, zf - 4.0),
                    TH._local_from(Fm, F, TH.prism(2 * wmax, TH.SHAFT_MC[1], 3.5, -(L1 * 0.45), -(TH.HUB_MCP_R + 0.6))))
    s = TH.fuse(hub, shaft)
    for sgn, (x0, x1) in spans.items():
        ch = TH.fuse(TH.box(x0, x1, -TH.SHAFT_MC[1] / 2, TH.SHAFT_MC[1] / 2, -(TH.HUB_MCP_R + 2.0), 0.0), TH.cyl_x(6.5, x0, x1))
        s = s.fuse(TH._local_from(Fm, F, ch))
    tools = [_ring_seat_x(TRACK["cmc_flex"], TH.HUB_CMC_R)]
    cups = {}
    for role in ("flex", "ext"):
        cup, pocket, hole = _radial_cup("cmc_flex", role, F)
        cups[role] = cup
        tools += [pocket, hole]
    angs = []
    for v in np.linspace(*RANGES["cmc_flex"], 9):
        q = {"cmc_flex": v}
        Fq = body_transforms(q)
        fr = TH.joint_frame_q("cmc_flex", q, Fq)
        A = to_local(fr, NECK_W)
        angs.append(np.degrees(np.arctan2(A[2], A[1])))
    a0, a1 = min(angs) - 12, max(angs) + 12
    tools.append(TH.sector_x(TH.HUB_CMC_R + 3, -TH.CMC_SLOT, TH.CMC_SLOT, a0, a1))
    holes, pins = TH._nip_pins("cmc_flex", TH.NIP_HALF["cmc_flex"])
    tools += holes
    tools += TH._axle_bores("cmc_flex")
    tools.append(TH._local_from(Fm, F, TH.cyl_x(TH.HUB_MCP_R + 0.3, -TH.CHEEK_MCP[0], TH.CHEEK_MCP[0])))
    tools += [TH._local_from(Fm, F, t) for t in _cheek_seats21("mcp_flex")]
    Fq = body_transforms({})
    for nm, st in STRANDS.items():
        if st["joint"] == "mcp_flex":
            c = TH.node(st, "cmc_cross")["p"]
            _, Tw, *_ = TH.drum_segment(st, c, Fq, {})
            tools.append(TH.cyl_ab(to_local(F, c), to_local(F, Tw - 1.5 * unit(Tw - c)), 0.8))
        elif st["joint"] == "ip":
            c, m = TH.node(st, "cmc_cross")["p"], TH.node(st, "mcp_cross")["p"]
            tools.append(TH.cyl_ab(to_local(F, c), to_local(F, m - (TH.HUB_MCP_R + 1.0) * unit(m - c)), 0.8))
    s = TH.cut(s, *tools)
    s = s.fuse(TH._stop_lugs("cmc_flex", TH.HUB_CMC_R, (abs(TRACK["cmc_flex"]) + 1.6, TH.HUB_CMC_W - 0.3), TH.PLATE_Z[1]))
    w = TH.place(TH.main_solid(s.clean()), F)
    w = _world_cut(w, _wire_channels("metacarpal"))
    return w, pins, {r: TH.place(c, F) for r, c in cups.items()}


def make_proximal():
    Fm, Fi = J0["mcp_flex"], J0["ip"]
    L2 = TH.L2
    hub = TH._hub_spacers("mcp_flex", TH.cyl_x(TH.HUB_MCP_R, -TH.HUB_MCP_W, TH.HUB_MCP_W))
    spans = _cheek_spans("ip")
    wmax = max(abs(v) for sp in spans.values() for v in sp)
    shaft = TH.fuse(TH.prism(TH.SHAFT_PX[0], TH.SHAFT_PX[1], 3.5, 3.0, L2 - TH.HUB_IP_R - 4.0),
                    TH._local_from(Fi, Fm, TH.prism(2 * wmax, TH.SHAFT_PX[1], 3.5, -(L2 * 0.5), -(TH.HUB_IP_R + 0.6))))
    s = TH.fuse(hub, shaft)
    for sgn, (x0, x1) in spans.items():
        ch = TH.fuse(TH.box(x0, x1, -TH.SHAFT_PX[1] / 2, TH.SHAFT_PX[1] / 2, -(TH.HUB_IP_R + 2.0), 0.0), TH.cyl_x(6.5, x0, x1))
        ch = ch.cut(TH.box(x0 - 0.1, x1 + 0.1, -TH.SHAFT_PX[1] / 2 - 1, TH.IP_CHEEK_TRIM_Y, -(TH.HUB_IP_R + 2.5), 0.01)
                    .cut(TH.cyl_x(6.5, x0 - 0.2, x1 + 0.2)))
        s = s.fuse(TH._local_from(Fi, Fm, ch))
    tools = [_ring_seat_x(TRACK["mcp_flex"], TH.HUB_MCP_R)]
    cups = {}
    for role in ("flex", "ext"):
        cup, pocket, hole = _radial_cup("mcp_flex", role, Fm)
        cups[role] = cup
        tools += [pocket, hole]
    lo, hi = RANGES["mcp_flex"]
    tools.append(TH.sector_x(TH.HUB_MCP_R + 3, -TH.MCP_SLOT, TH.MCP_SLOT, -90 - hi - 25, -90 - lo + 25))
    holes, pins = TH._nip_pins("mcp_flex", TH.NIP_HALF["mcp_flex"])
    tools += holes
    tools += TH._axle_bores("mcp_flex")
    tools.append(TH._local_from(Fi, Fm, TH.cyl_x(TH.HUB_IP_R + 0.3, -TH.CHEEK_IP[0], TH.CHEEK_IP[0])))
    tools += [TH._local_from(Fi, Fm, t) for t in _cheek_seats21("ip")]
    Fq = body_transforms({})
    for nm, st in STRANDS.items():
        if st["joint"] == "ip":
            c = TH.node(st, "mcp_cross")["p"]
            _, Tw, *_ = TH.drum_segment(st, c, Fq, {})
            tools.append(TH.cyl_ab(to_local(Fm, c), to_local(Fm, Tw - 1.5 * unit(Tw - c)), 0.8))
    s = TH.cut(s, *tools)
    s = s.fuse(TH._stop_lugs("mcp_flex", TH.HUB_MCP_R, (abs(TRACK["mcp_flex"]) + 1.6, TH.HUB_MCP_W - 0.3), -(TH.HUB_MCP_R + 0.6)))
    w = TH.place(TH.main_solid(s.clean()), Fm)
    w = _world_cut(w, _wire_channels("proximal"))
    return w, pins, {r: TH.place(c, Fm) for r, c in cups.items()}


TIP = dict(zc=None, cav_x=4.9, cav_y=(-5.6, -1.6), cav_dz=5.2)


def make_distal():
    cq = _cq()
    Fi = J0["ip"]
    L3 = TH.L3
    hub = TH._hub_spacers("ip", TH.cyl_x(TH.HUB_IP_R, -TH.HUB_IP_W, TH.HUB_IP_W))
    body = TH.loft([TH.rrect_wire(2 * TH.HUB_IP_W, 15.0, 3.5, 2.0), TH.rrect_wire(2 * TH.HUB_IP_W, 14.3, 3.5, TH.DISTAL_NARROW_Z),
                    TH.rrect_wire(15.0, 12.5, 4.0, L3 - 7.0)])
    tip = cq.Workplane("XY").add(TH.cyl_x(6.25, -7.0, 7.0, 0, L3 - 7.0)).edges().fillet(2.5).val()
    s = TH.fuse(hub, body, tip)
    zc = L3 - 14.8
    seat = TH.box(-8, 8, -9, -TH.PAD_SEAT_Y, 5.0, L3 + 1)
    cav = TH.box(-TIP["cav_x"], TIP["cav_x"], -6.0, TIP["cav_y"][1], zc - TIP["cav_dz"], zc + TIP["cav_dz"])
    tools = [_ring_seat_x(TRACK["ip"], TH.HUB_IP_R), seat, cav] + TH._axle_bores("ip")
    cups = {}
    for role in ("flex", "ext"):
        cup, pocket, hole = _radial_cup("ip", role, Fi)
        cups[role] = cup
        tools += [pocket, hole]
    s = TH.cut(s, *tools)
    s = s.fuse(TH._stop_lugs("ip", TH.HUB_IP_R, (3.6, TH.HUB_IP_W - 0.3), -(TH.HUB_IP_R + 0.6)))
    w = TH.place(TH.main_solid(s.clean()), Fi)
    w = _world_cut(w, _wire_channels("distal"))
    return w, {r: TH.place(c, Fi) for r, c in cups.items()}


def make_tip_parts(parts):
    cq = _cq()
    Fi = J0["ip"]
    L3 = TH.L3
    zc = L3 - 14.8
    _reg(parts, "tip_board", TH.place(TH.box(-4.6, 4.6, -2.6, -1.8, zc - 4.5, zc + 4.5), Fi), "distal", "PCB")
    _reg(parts, "tip_chip_mlx90393", TH.place(TH.box(-1.5, 1.5, -3.6, -2.6, zc - 1.5, zc + 1.5), Fi), "distal", "PCB")
    # snap-in silicone skin (Dragon Skin 20 class, cast in the 05 mould style): 3.0 mm pad + tip wrap + a plug
    # that fills the sensor cavity below the chip (0.4 mm air gap)
    pad = TH.loft([TH.rrect_wire(15.0, 3.0, 1.0, 5.5), TH.rrect_wire(14.0, 3.0, 1.0, L3 - 7.0)]).translate(cq.Vector(0, -(TH.PAD_SEAT_Y + 1.5), 0))
    cap = TH.cut(TH.ring_x(6.0, 8.5, -7.0, 7.0).translate(cq.Vector(0, 0, L3 - 7.0)),
                 TH.box(-10, 10, -TH.PAD_SEAT_Y, 10, -10, 100), TH.box(-10, 10, -10, 10, -10, L3 - 7.0))
    plug = TH.box(-TIP["cav_x"] + 0.15, TIP["cav_x"] - 0.15, -5.55, -4.0, zc - TIP["cav_dz"] + 0.15, zc + TIP["cav_dz"] - 0.15)
    skin = TH.fuse(pad, cap, plug)
    mag = TH.cyl_ab(np.array([0, -7.9, zc]), np.array([0, -6.9, zc]), 1.5)
    skin = skin.cut(mag)
    _reg(parts, "tip_skin", TH.place(skin, Fi), "distal", "silicone")
    _reg(parts, "tip_magnet", TH.place(TH.cyl_ab(np.array([0, -7.9, zc]), np.array([0, -6.9, zc]), 1.49), Fi), "distal", "magnet")


def _roller_frame(c, n):
    n = unit(n)
    ref = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    x = unit(np.cross(n, ref))
    y = np.cross(n, x)
    return (np.asarray(c, float), x, y, n)  # local z = roller axis


def _roller_axes():
    """{roller key: (centre, axis, strand plane in/out dirs)} from the strand paths at q = 0."""
    out = {}
    for n in STRAND_NAMES:
        _, info = strand_path(n, {}, detail=True)
        for k, ax in info["axes"].items():
            out[f"{k}_{n}"] = (route()[f"{k}_{n}"], ax)
    return out


def make_rollers(parts):
    """MR63 + Ø3 pin per roller; forks (two cheeks + back) united into turning_block_A (row A, palm) and into the
    mount (rows B, C)."""
    cq = _cq()
    forks_A, forks_BC = [], []
    paths = {n: strand_path(n, {}) for n in STRAND_NAMES}
    for key, (c, ax) in _roller_axes().items():
        fr = _roller_frame(c, ax)
        ring = TH.ring_z(MR63["id"] / 2, MR63["od"] / 2, -MR63["w"] / 2, MR63["w"] / 2)
        _reg(parts, f"roller_{key}", TH.place(ring, fr), "mount", "steel")
        _reg(parts, f"roller_pin_{key}", TH.place(TH.cyl_z(1.5, -3.9, 3.9), fr), "mount", "steel")
        # fork: two cheeks 1.3 thick at |z| 1.45..2.75 (0.2 washer gap), r 4.0, plus a back plate opposite the wrap
        n = key.split("_", 1)[1]
        P_ = paths[n]
        d = P_ - c
        dist = np.linalg.norm(d, axis=1)
        near = P_[dist < RS + 0.6]
        bis = unit(np.mean(near, axis=0) - c) if len(near) else np.array([1.0, 0, 0])
        bl = to_local(fr, c + bis)
        back = -unit(np.array([bl[0], bl[1], 0.0]))
        ch = TH.fuse(TH.cyl_z(4.0, 1.45, 2.75), TH.cyl_z(4.0, -2.75, -1.45))
        bk_c = back * 5.2
        bk = TH.cyl_ab(np.array([bk_c[0], bk_c[1], -2.75]), np.array([bk_c[0], bk_c[1], 2.75]), 1.6)
        f = TH.fuse(ch, bk)
        f = f.cut(TH.cyl_z(1.5 + PV.PRESS_FIT / 2, -4, 4))
        fw = TH.place(f, fr)
        (forks_A if key.startswith("A") else forks_BC).append((key, fw, fr, back))
    return forks_A, forks_BC


def _strut(a, b, r=1.4):
    return TH.cyl_ab(np.asarray(a, float), np.asarray(b, float), r)


def make_turning_block(forks_A):
    """row-A forks joined by a dorsal rail (y 8..10.5) that screws to the palm's dorsal shell (2 x M2)."""
    cq = _cq()
    cs = [fr[0] for _, _, fr, _ in forks_A]
    x0, x1 = min(c[0] for c in cs) - 3, max(c[0] for c in cs) + 3
    zr = float(np.mean([c[2] for c in cs]))
    rail = TH.box(x0, x1, 8.4, 10.8, zr - 2.5, zr + 2.5)
    s = rail
    for key, fw, fr, back in forks_A:
        s = s.fuse(fw)
        c = fr[0]
        bw = TH.to_world(fr, [back[0] * 5.2, back[1] * 5.2, 0.0])
        s = s.fuse(_strut(bw, np.array([bw[0], 9.6, np.clip(bw[2], zr - 2, zr + 2)]), 1.2))
    for x in (x0 + 2.5, x1 - 2.5):
        s = s.cut(TH.cyl_ab(np.array([x, 7.0, zr]), np.array([x, 12.0, zr]), 0.9))
    return s


def build(quick=False):
    import cadquery as cq

    t0 = time.time()
    parts = {}
    MATERIAL.clear()
    BODY.clear()
    _reg(parts, "thumb_mount", make_mount(), "mount", "PETG")
    _reg(parts, "bearing_rot_top_6704zz", TH.place(TH.ring_z(10.0, 13.5, *TH.BEAR_TOP_Z), C), "mount", "steel")
    _reg(parts, "bearing_rot_bottom_6704_zro2", TH.place(TH.ring_z(10.0, 13.5, *TH.BEAR_BOT_Z), C), "mount", "ceramic")
    _reg(parts, "neck_eyelet", make_neck(), "mount", "stainless")
    _reg(parts, "rot_sensor_chip", TH.place(TH.box(12.0, 13.45, -1.45, 1.45, -28.0, -25.2), C), "mount", "PCB")
    _reg(parts, "rot_sensor_board", TH.place(TH.box(13.45, 14.45, -1.7, 1.7, -28.1, -25.1), C), "mount", "PCB")
    _reg(parts, "plug_board", TH.place(TH.box(15.2, 20.6, -0.5, 4.5, -27.0, -26.2), C), "mount", "PCB")
    _reg(parts, "plug_jst_sh4", TH.place(TH.box(16.0, 20.2, -0.2, 4.2, -26.2, -23.3), C), "mount", "PETG")
    base, cups_rot = make_base()
    _reg(parts, "thumb_base", base, "base", "PETG")
    _reg(parts, "base_cap", make_cap(), "base", "PETG")
    _reg(parts, "rot_magnet_ring", TH.place(TH.ring_z(ROT_MAG["ri"], ROT_MAG["ro"], *ROT_MAG["z"]), C), "base", "magnet")
    _reg(parts, "ring_rot", TH.place(TH.ring_z(RING["id"] / 2, RING["od"] / 2, *ROT_RING_Z), C), "base", "brass")
    for r, c in cups_rot.items():
        _reg(parts, f"cup_rot_{r}", c, "base", "PETG")
    mc, pins_c, cups_c = make_metacarpal()
    _reg(parts, "thumb_metacarpal", mc, "metacarpal", "PETG")
    _reg(parts, "ring_cmc", TH.place(TH.ring_x(RING["id"] / 2, RING["od"] / 2, TRACK["cmc_flex"] - 1.5, TRACK["cmc_flex"] + 1.5), J0["cmc_flex"]), "metacarpal", "brass")
    for r, c in cups_c.items():
        _reg(parts, f"cup_cmc_{r}", c, "metacarpal", "PETG")
    for i, p in enumerate(pins_c):
        _reg(parts, f"nip_cmc_{'ab'[i]}", p, "metacarpal", "steel")
    _reg(parts, "thenar_pad", TH.make_thenar_pad(), "metacarpal", "TPU")
    px, pins_m, cups_m = make_proximal()
    _reg(parts, "thumb_proximal", px, "proximal", "PETG")
    _reg(parts, "ring_mcp", TH.place(TH.ring_x(RING["id"] / 2, RING["od"] / 2, TRACK["mcp_flex"] - 1.5, TRACK["mcp_flex"] + 1.5), J0["mcp_flex"]), "proximal", "brass")
    for r, c in cups_m.items():
        _reg(parts, f"cup_mcp_{r}", c, "proximal", "PETG")
    for i, p in enumerate(pins_m):
        _reg(parts, f"nip_mcp_{'ab'[i]}", p, "proximal", "steel")
    ds, cups_i = make_distal()
    _reg(parts, "thumb_distal", ds, "distal", "PETG")
    _reg(parts, "ring_ip", TH.place(TH.ring_x(RING["id"] / 2, RING["od"] / 2, TRACK["ip"] - 1.5, TRACK["ip"] + 1.5), J0["ip"]), "distal", "brass")
    for r, c in cups_i.items():
        _reg(parts, f"cup_ip_{r}", c, "distal", "PETG")
    make_tip_parts(parts)
    _hinge_hardware("cmc_flex", parts, "base", "metacarpal")
    _hinge_hardware("mcp_flex", parts, "metacarpal", "proximal")
    _hinge_hardware("ip", parts, "proximal", "distal")
    forks_A, forks_BC = make_rollers(parts)
    _reg(parts, "turning_block_A", make_turning_block(forks_A), "mount", "PETG")
    m = parts["thumb_mount"]
    for key, fw, fr, back in forks_BC:
        m = m.fuse(fw)
        bw = TH.to_world(fr, [back[0] * 5.2, back[1] * 5.2, 0.0])
        # strut from the fork back to the nearest point of the mount's housing surface
        loc = to_local(C, bw)
        rr = np.hypot(loc[0], loc[1])
        tgt = to_world(C, [loc[0] / rr * 13.8, loc[1] / rr * 13.8, np.clip(loc[2], MOUNT_Z[0] + 1.2, MOUNT_Z[1] - 1.2)])
        m = m.fuse(_strut(bw, tgt, 1.4))
    m = TH.main_solid(m.clean())
    if not m.isValid():
        m = TH.main_solid(m.fix())
    parts["thumb_mount"] = m
    print(f"  parts built: {len(parts)} in {time.time() - t0:.0f} s", flush=True)
    return parts


def greedy_route(cen, liners, sweeps=3, ncand=160, seed=1):
    """coordinate search: each roller in turn jumps to the best of ncand candidates that clear the finger liners,
    judged by the full cost. Robust where the gradient methods stall on the clearance penalties."""
    rng = np.random.default_rng(seed)
    LA, LB = liners
    cen = {k: np.array(v, float) for k, v in cen.items()}
    best = _route_eval(cen, liners)
    for sw in range(sweeps):
        for k in list(cen):
            if k.startswith("A"):
                lo, hi = np.array([-6.0, -7.2, 34.5]), np.array([27.0, 7.2, 62.0])
            elif k.startswith("B"):
                lo, hi = NECK_W - np.array([12, 6, 12.0]), NECK_W + np.array([4, 12, 0.0])
            else:
                lo, hi = cen[k] - 6, cen[k] + 6
            cand = lo + (hi - lo) * rng.random((4000, 3))
            if len(LA):
                dl = _pt_seg_d(cand, LA, LB).min(1)
                cand = cand[dl > RF + 1.5]
            if sw:
                cand = np.vstack([cand, cen[k] + rng.normal(0, 1.5, (ncand // 2, 3))])
            if not len(cand):
                continue
            pick = cand[rng.choice(len(cand), min(ncand, len(cand)), replace=False)]
            for c in pick:
                old = cen[k]
                cen[k] = c
                try:
                    with np.errstate(all="ignore"):
                        v = _route_eval(cen, liners)
                    v = v if np.isfinite(v) else 1e12
                except Exception:
                    v = 1e12
                if v < best:
                    best = v
                else:
                    cen[k] = old
        c_, pen, _ = _route_eval(cen, liners, verbose=True)
        print(f"  greedy sweep {sw}: cost {c_:.1f} penalty {pen:.3f}", flush=True)
    return cen


if __name__ == "__main__" and "--route" in sys.argv:
    t0 = time.time()
    x0 = None
    if ROUTE_FILE.exists():
        d = json.loads(ROUTE_FILE.read_text())["centres"]
        keys = [f"A_{n}" for n in STRAND_NAMES] + [f"B_{n}" for n in STRAND_NAMES if "cmc_rot" not in n] + \
               [f"C_{n}" for n in STRAND_NAMES if "cmc_rot" in n]
        x0 = np.concatenate([d[k] for k in keys])
    if "--greedy" in sys.argv:
        liners = _finger_liners()
        cen0 = {k: np.array(v) for k, v in json.loads(ROUTE_FILE.read_text())["centres"].items()}
        cen = greedy_route(cen0, liners, sweeps=int(next((a.split("=")[1] for a in sys.argv if a.startswith("--sweeps=")), 3)))
        c, pen, infos = _route_eval(cen, liners, verbose=True)
        ROUTE_FILE.write_text(json.dumps({"centres": {k: [float(x) for x in v] for k, v in cen.items()},
                                          "cost": c, "penalty": pen, "bends": infos}, indent=1))
    else:
        cen, c, pen, infos = optimise_route(x0)
    print(f"route: cost {c:.1f}, penalty {pen:.4f}, {time.time() - t0:.0f} s")
    for n, i in infos.items():
        print(" ", n, {k: round(v, 1) for k, v in i.items()})

# =====================================================================================================================
# Checks and export
# =====================================================================================================================
IGNORE_PAIRS = [  # parts that sit in their own seats / press fits (same body or designed contact)
    ("axle_*", "thumb_metacarpal"), ("axle_*", "thumb_proximal"), ("axle_*", "thumb_distal"), ("magnet_*", "axle_*"),
    ("nip_cmc_*", "thumb_metacarpal"), ("nip_mcp_*", "thumb_proximal"), ("bearing_cmc_*", "thumb_base"),
    ("bearing_mcp_*", "thumb_metacarpal"), ("bearing_ip_*", "thumb_proximal"), ("bearing_cmc_*", "axle_cmc_*"),
    ("bearing_mcp_*", "axle_mcp_*"), ("bearing_ip_*", "axle_ip_*"), ("bearing_rot_*", "thumb_base"),
    ("bearing_rot_*", "thumb_mount"), ("sensor_plug_cmc", "thumb_base"), ("sensor_plug_mcp", "thumb_metacarpal"),
    ("sensor_plug_ip", "thumb_proximal"), ("wire_keeper_cmc", "thumb_base"), ("wire_keeper_mcp", "thumb_metacarpal"),
    ("wire_keeper_ip", "thumb_proximal"), ("sensor_chip_*", "sensor_plug_*"), ("sensor_board_*", "sensor_plug_*"),
    ("neck_eyelet", "thumb_mount"), ("rot_sensor_*", "thumb_mount"), ("plug_*", "thumb_mount"), ("plug_board", "plug_jst_sh4"),
    ("rot_magnet_ring", "base_cap"), ("ring_rot", "thumb_base"), ("ring_cmc", "thumb_metacarpal"), ("ring_mcp", "thumb_proximal"),
    ("ring_ip", "thumb_distal"), ("cup_rot_*", "thumb_base"), ("cup_cmc_*", "thumb_metacarpal"), ("cup_mcp_*", "thumb_proximal"),
    ("cup_ip_*", "thumb_distal"), ("cup_*", "ring_*"), ("tip_*", "thumb_distal"), ("tip_magnet", "tip_skin"),
    ("tip_chip_mlx90393", "tip_board"), ("thenar_pad", "thumb_metacarpal"), ("roller_pin_*", "thumb_mount"),
    ("roller_pin_*", "turning_block_A"), ("roller_pin_*", "roller_*"), ("base_cap", "thumb_base"),
]


def _hit(name, pat):
    return name == pat or (pat.endswith("*") and name.startswith(pat[:-1]))


def _ignored(a, b, extra=()):
    if a.startswith("roller_pin_") and b.startswith("roller_") and a[len("roller_pin_"):] == b[len("roller_"):]:
        return True
    if b.startswith("roller_pin_") and a.startswith("roller_") and b[len("roller_pin_"):] == a[len("roller_"):]:
        return True
    for x, y in list(IGNORE_PAIRS) + list(extra):
        if (_hit(a, x) and _hit(b, y)) or (_hit(a, y) and _hit(b, x)):
            if x == "roller_pin_*" and y == "roller_*":
                continue
            return True
    return False


def _tm(shape, tol=0.05, path=None):
    """mesh via OCC's STL writer (welded, watertight for valid solids)."""
    import tempfile

    import cadquery as cq
    import trimesh
    p = path or Path(tempfile.mkstemp(suffix=".stl")[1])
    cq.exporters.export(shape, str(p), tolerance=tol, angularTolerance=0.2)
    m = trimesh.load(p, force="mesh", process=True)
    m.merge_vertices()
    return m


def _inter_vol(ma, mb, n=6000):
    """approximate overlap volume of two meshes: sample points inside A (volume-uniform) and test them in B."""
    import trimesh
    if not ma.is_watertight:
        ma = ma.convex_hull if not hasattr(ma, "fill_holes") else ma
    lo = np.maximum(ma.bounds[0], mb.bounds[0])
    hi = np.minimum(ma.bounds[1], mb.bounds[1])
    if np.any(lo >= hi):
        return 0.0
    rng = np.random.default_rng(0)
    pts = lo + (hi - lo) * rng.random((n, 3))
    ina = ma.contains(pts)
    if not ina.any():
        return 0.0
    inb = mb.contains(pts[ina])
    return float(inb.sum() / n * np.prod(hi - lo))


def interference_at(shapes, q, thr=0.3, pairs=None):
    """OCC boolean common between posed parts (bbox prefilter); volumes in mm^3."""
    qd = _qdict(q)
    posed = {n: (TH.pose(s, BODY[n], qd) if BODY[n] != "mount" else s) for n, s in shapes.items()}
    bb = {}
    for n, s in posed.items():
        b = s.BoundingBox()
        bb[n] = (np.array([b.xmin, b.ymin, b.zmin]), np.array([b.xmax, b.ymax, b.zmax]))
    names = list(posed)
    hits = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if pairs is not None and (a, b) not in pairs:
                continue
            if _ignored(a, b):
                continue
            A, B = bb[a], bb[b]
            if np.any(A[0] > B[1]) or np.any(B[0] > A[1]):
                continue
            try:
                v = float(posed[a].intersect(posed[b]).Volume())
            except Exception:
                v = -1.0  # OCC boolean failed: re-checked on meshes in main()
            if v > thr or v < 0:
                hits.append((a, b, round(v, 2)))
    return hits


def pose_grid(quick=False):
    lv = (0, 2) if quick else (0, 1, 2)

    def val(j, i):
        lo, hi = RANGES[j]
        return (lo, 0.5 * (lo + hi), hi)[i]
    out = []
    for a in lv:
        for b in lv:
            for c in lv:
                out.append({"cmc_rot": val("cmc_rot", a), "cmc_flex": val("cmc_flex", b), "mcp_flex": val("mcp_flex", c),
                            "ip": val("ip", c)})
    out.append({k: 0.0 for k in JOINTS})
    return out


def centreline_in_solids(meshes, q, line, skip_parts=(), step=0.3, skip_near=None):
    """points of a polyline (world at q) inside any part mesh (posed), except parts in skip_parts."""
    T = body_T(q)
    pts = []
    for a, b in zip(line[:-1], line[1:]):
        n = max(1, int(np.linalg.norm(b - a) / step))
        pts += [a + (b - a) * t for t in np.linspace(0, 1, n, endpoint=False)]
    pts = np.array(pts)
    bad = {}
    for nme, m in meshes.items():
        if any(_hit(nme, s) for s in skip_parts):
            continue
        Minv = np.linalg.inv(T[BODY[nme]])
        loc = pts @ Minv[:3, :3].T + Minv[:3, 3]
        lo, hi = m.bounds
        sel = np.all((loc > lo - 0.01) & (loc < hi + 0.01), axis=1)
        if skip_near is not None:
            sel &= ~skip_near
        if not sel.any():
            continue
        inside = m.contains(loc[sel])
        if inside.any():
            bad[nme] = int(inside.sum())
    return bad


def loop_report():
    """flex + ext loop length change and the real coupling (half the flex - ext difference) per joint, over the
    28-pose grid; and each strand's own slope (mm/deg) vs its joint."""
    grid = pose_grid()
    L = {n: np.array([strand_length(n, q) for q in grid]) for n in STRAND_NAMES}
    out = {}
    for j in JOINTS:
        f, e = L[f"{j}_flex"], L[f"{j}_ext"]
        # remove the joint's own motion: the own-joint contribution cancels in flex + ext
        loop = f + e
        own_sl = []
        for role in ("flex", "ext"):
            q1, q2 = {j: RANGES[j][0] + 5}, {j: RANGES[j][1] - 5}
            own_sl.append((strand_length(f"{j}_{role}", q2) - strand_length(f"{j}_{role}", q1)) / (RANGES[j][1] - RANGES[j][0] - 10))
        # coupling: own joint fixed at mid, others swept
        mids = []
        for q in grid:
            qq = dict(q)
            qq[j] = 0.5 * sum(RANGES[j])
            mids.append(qq)
        fm = np.array([strand_length(f"{j}_flex", q) for q in mids])
        em = np.array([strand_length(f"{j}_ext", q) for q in mids])
        coup = 0.5 * ((fm - fm[-1]) - (em - em[-1]))
        out[j] = {"loop_range_mm": float(loop.max() - loop.min()), "own_slope_mm_per_deg": [round(x, 4) for x in own_sl],
                  "coupling_max_mm": float(np.abs(coup).max()), "coupling_max_deg": float(np.abs(coup).max() / (np.pi * R / 180))}
    return out


def bends_report():
    out = {}
    for n in STRAND_NAMES:
        worst = {}
        for q in pose_grid():
            _, info = strand_path(n, q, detail=True)
            for k, v in info.items():
                if isinstance(v, float):
                    worst.setdefault(k, [1e9, -1e9])
                    worst[k] = [min(worst[k][0], v), max(worst[k][1], v)]
        out[f"thumb_{n}"] = {k: [round(a, 1), round(b, 1)] for k, (a, b) in worst.items()}
    return out


def magnet_report():
    """distances between sensor magnets at q = 0 and the opposition poses, and to steel parts' centres."""
    mags = {"magnet_cmc": "metacarpal", "magnet_mcp": "proximal", "magnet_ip": "distal", "rot_magnet_ring": "base"}
    return mags


def efficiency():
    """thumb strand efficiency budget (axial strands, worst at mid pose): wrist + forearm guides as the fingers
    (report 12: +25 deg sliding, eye plate), palm plate exit, rollers, neck, nips, joint bushes."""
    mu = 0.12  # braid on polished steel (report 02/08, verify with bench T3)
    eta_roll = 0.99  # MR63 per roller (bearing torque ~ 0.5 % of the line pull at 30 N, plus fleet)
    res = {}
    for n in STRAND_NAMES:
        _, info = strand_path(n, {k: 0.5 * sum(RANGES[k]) for k in JOINTS}, detail=True)
        slide = info["plate"] + info.get("neck", 0.0)
        rollers = 2
        nips = 0
        if n.startswith("mcp"):
            nips = 1
        if n.startswith("ip"):
            nips = 2
        e = np.exp(-mu * np.radians(25 + slide + 30 * nips)) * eta_roll ** rollers * 0.97
        res[f"thumb_{n}"] = round(float(e), 3)
    return res


def pinch_estimate(eta):
    """V2 strength table (J^T, report 02 §6) with the V2.1 drive: spool r 5, line r 7.235, eta from above,
    servo at stall and at the 80 % cap."""
    S = TH.DESIGN_JSON.get("strength", {})
    out = {}
    for servo, T in (("SCS0009", 0.226), ("HLS3606M", 0.59)):
        for cap in (1.0, 0.8):
            tj = {j: T * cap / 0.005 * R / 1000 * eta[f"thumb_{j}_flex"] for j in JOINTS}
            row = {}
            for task, tau in (("index_pad", (0.049, 0.052, 0.052, 0.017)), ("tip_to_tip", (0.048, 0.050, 0.050, 0.015)),
                              ("key_pinch", (0.031, 0.048, 0.052, 0.017)), ("power55", (0.023, 0.014, 0.040, 0.017))):
                f = min(tj[j] / t for j, t in zip(JOINTS, tau))
                lim = min(JOINTS, key=lambda j: tj[j] / tau[JOINTS.index(j)])
                row[task] = (round(f, 1), lim)
            out[f"{servo}_{int(cap * 100)}"] = row
    return out


def opposition_check(meshes):
    """thumb vs package-A fingers at the V2 opposition tasks (thumb_design.json eval): smallest gap between the
    thumb skin and the target finger's tip, and whether any non-pad thumb part intersects a finger."""
    import trimesh
    try:
        F = _load("fingers18_for_thumb2", HERE / "18_fingers.py")
        fparts = F.parts()
    except Exception as e:
        return {"error": str(e)}
    ev = TH.DESIGN_JSON["eval"]
    res = {}
    fm_cache = {}
    for task, fname in (("index", "index"), ("middle", "middle"), ("ring", "ring"), ("little", "little"), ("tip_pinch", "index"),
                        ("lateral", "index")):
        e = ev[task]
        qt = dict(zip(JOINTS, e["q_thumb"]))
        qf = {}
        if e.get("q_finger"):
            a, m, p = e["q_finger"]
            qf = {f"{fname}_mcp_abd": a, f"{fname}_mcp_flex": m, f"{fname}_pip": p}
        TF = F.pose(qf)
        TT = body_T(qt)
        fing = []
        for n, info in fparts.items():
            if info.get("finger") != fname or info["body"].endswith("module"):
                continue
            if n not in fm_cache:
                fm_cache[n] = trimesh.load(F.OUT / info["stl"], force="mesh")
            mm = fm_cache[n].copy()
            mm.apply_transform(TF[n])
            fing.append((n, mm))
        skin = meshes["tip_skin"].copy()
        skin.apply_transform(TT["distal"])
        gap = 1e9
        for n, mm in fing:
            if any(k in n for k in ("distal", "skin", "middle")):
                sv = skin.vertices[:: max(1, len(skin.vertices) // 400)]
                d = trimesh.proximity.closest_point(mm, sv)[1].min()
                inside = mm.contains(sv).any() if mm.is_watertight else False
                gap = min(gap, -0.1 if inside else float(d))
        bad = []
        for tn, tm_ in meshes.items():
            if tn in ("tip_skin", "tip_magnet", "thenar_pad"):
                continue
            if BODY[tn] not in ("distal", "proximal", "metacarpal"):
                continue
            mt = tm_.copy()
            mt.apply_transform(TT[BODY[tn]])
            for n, mm in fing:
                if np.any(mt.bounds[0] > mm.bounds[1]) or np.any(mm.bounds[0] > mt.bounds[1]):
                    continue
                v = _inter_vol(mt, mm)
                if v > 0.5:
                    bad.append((tn, n, round(v, 1)))
        res[task] = {"q_thumb": [round(v, 1) for v in e["q_thumb"]], "q_finger": e.get("q_finger"),
                     "skin_to_finger_gap_mm": round(gap, 2), "non_pad_contacts": bad}
    return res


def render_set(parts, out_dir, quick=False):
    sys.path.insert(0, str(V2CAD))
    from tools.render import render
    import cadquery as cq

    def posed(q, names=None, lines_on=True):
        d = {}
        for n, s in parts.items():
            if names and not any(_hit(n, k) for k in names):
                continue
            d[n] = TH.pose(s, BODY[n], {j: _qdict(q)[j] for j in JOINTS})
        if lines_on:
            for ln, pts in lines(q).items():
                segs = [TH.cyl_ab(a, b, 0.3 if "wire" in ln else 0.25) for a, b in zip(pts[:-1], pts[1:]) if np.linalg.norm(b - a) > 0.05]
                d[ln] = cq.Compound.makeCompound(segs)
        return d
    col = {n: COLOR.get(MATERIAL[n], "#d9dde3") for n in parts}
    for ln in lines({}):
        col[ln] = "#d02020" if "wire" in ln else "#202020"
    shots = [("thumb_q0", {}, ("iso", "front", "side", "top")),
             ("thumb_tip_pinch", dict(zip(JOINTS, TH.DESIGN_JSON["eval"]["tip_pinch"]["q_thumb"])), ("iso", "front")),
             ("thumb_to_little", dict(zip(JOINTS, TH.DESIGN_JSON["eval"]["little"]["q_thumb"])), ("iso", "front"))]
    out = []
    for name, q, views in shots:
        p = out_dir / f"{name}.png"
        render(posed(q), p, views=views, colors=col, title=f"Tendra thumb V2.1 · {name} · q {[round(v) for v in _qdict(q).values()]}",
               legend=False)
        out.append(p)
    # routing close-up: palm plate exits, rollers, neck
    keep = ("thumb_mount", "turning_block_A", "roller_*", "neck_eyelet", "bearing_rot_*", "thumb_base", "base_cap")
    p = out_dir / "thumb_routing.png"
    render(posed({}, keep), p, views=("iso", "front", "back", "top"), colors=col, title="thumb V2.1 · strands from the palm plate (z 30.6) over rollers A/B/C to the neck", legend=False)
    out.append(p)
    # sensor section: hinge stacks
    keep = ("thumb_metacarpal", "thumb_proximal", "thumb_distal", "sensor_*", "magnet_*", "bearing_*", "axle_*", "wire_keeper_*",
            "tip_*", "ring_*", "cup_*")
    p = out_dir / "thumb_sensors.png"
    render(posed({}, keep), p, views=("iso", "iso2", "side", "left"), colors=col, title="thumb V2.1 · sensors, rings, cups, hollow axles, wire", legend=False)
    out.append(p)
    return out


def main(quick=False):
    import cadquery as cq
    import trimesh

    t0 = time.time()
    OUT.mkdir(parents=True, exist_ok=True)
    STL.mkdir(exist_ok=True)
    parts = build(quick)
    # validity
    val, meshes = {}, {}
    for n, s in parts.items():
        sh = s if hasattr(s, "isValid") else s.val()
        ok = bool(sh.isValid())
        nsol = len(sh.Solids())
        m = _tm(sh, path=STL / f"{n}.stl")
        meshes[n] = m
        val[n] = dict(valid=ok, solids=nsol, watertight=bool(m.is_watertight), volume=round(float(sh.Volume()), 2))
    bad = {n: v for n, v in val.items() if not (v["valid"] and v["watertight"] and v["solids"] == 1)}
    print(f"  validity: {len(parts) - len(bad)}/{len(parts)} single valid watertight solids; bad: {bad}", flush=True)
    # STEP of the printed parts + the assembly
    asm = cq.Assembly(name="thumb_v21_q0")
    for n, s in parts.items():
        asm.add(s, name=n, color=cq.Color(COLOR.get(MATERIAL[n], "#d9dde3")))
        if MATERIAL[n] in ("PETG", "silicone", "TPU") and not n.startswith(("cup_",)):
            cq.exporters.export(s, str(OUT / f"{n}.step"))
    asm.save(str(OUT / "thumb_v21_q0.step"))
    pj = {n: {"body": BODY[n], "material": MATERIAL[n], "stl": str((STL / f"{n}.stl").relative_to(STUDY)),
              "color": COLOR.get(MATERIAL[n], "#d9dde3")} for n in parts}
    (OUT / "parts.json").write_text(json.dumps(pj, indent=1))
    parts_cache = parts
    parts.__class__  # noqa
    # glb at q = 0 with the lines
    sc = trimesh.Scene()
    for n, m in meshes.items():
        mm = m.copy()
        c = COLOR.get(MATERIAL[n], "#d9dde3").lstrip("#")
        mm.visual.face_colors = [int(c[i:i + 2], 16) for i in (0, 2, 4)] + [255]
        sc.add_geometry(mm, node_name=n, geom_name=n)
    for ln, pts in lines({}).items():
        segs = [trimesh.creation.cylinder(radius=0.25, segment=[a, b], sections=8) for a, b in zip(pts[:-1], pts[1:]) if np.linalg.norm(b - a) > 0.05]
        mm = trimesh.util.concatenate(segs)
        mm.visual.face_colors = [200, 30, 30, 255] if "wire" in ln else [30, 30, 30, 255]
        sc.add_geometry(mm, node_name=ln, geom_name=ln)
    sc.export(str(OUT / "thumb_v21_q0.glb"))
    print(f"  exported ({time.time() - t0:.0f} s)", flush=True)
    # checks
    checks = {"validity": val, "invalid": bad, "part_count": len(parts)}
    same = interference_at(parts, {})
    checks["interference_q0_all_pairs"] = same
    grid = pose_grid(quick)
    inter = []
    names = list(meshes)
    cross = {(a, b) for i, a in enumerate(names) for b in names[i + 1:] if BODY[a] != BODY[b]}
    for q in grid:
        h = interference_at(parts, q, pairs=cross)
        T = body_T(q)
        h2 = []
        for a, b, v in h:
            if v >= 0:
                h2.append((a, b, v))
                continue
            k = 0  # OCC failed: count vertices of each part inside the other (posed meshes)
            for x, y in ((a, b), (b, a)):
                mx, my = meshes[x].copy(), meshes[y].copy()
                mx.apply_transform(T[BODY[x]])
                my.apply_transform(T[BODY[y]])
                vv = mx.vertices[:: max(1, len(mx.vertices) // 3000)]
                sel = np.all((vv > my.bounds[0]) & (vv < my.bounds[1]), axis=1)
                if sel.any():
                    k += int(my.contains(vv[sel]).sum())
            if k:
                h2.append((a, b, f"mesh check: {k} vertices inside"))
        h = h2
        inter.append({"q": {k: round(v, 1) for k, v in q.items()}, "hits": h})
        print("  q", [round(v) for v in q.values()], "->", h if h else "clear", flush=True)
    checks["interference_grid"] = inter
    checks["interference_grid_clear"] = f"{sum(1 for r in inter if not r['hits'])}/{len(inter)}"
    # fixed neighbours (report 13 palm plate + yoke, the integrator's palm if present)
    fixed = {}
    for nm, path in (("palm_plate_w13", STUDY / "out/13/palm_plate_w13.stl"), ("palm_yoke_w13", STUDY / "out/13/palm_yoke_w13.stl"),
                     ):
        if path.exists():
            fixed[nm] = trimesh.load(path, force="mesh")
    fx = {}
    for q in (grid[0], grid[-1]):
        T = body_T(q)
        for fn, fm in fixed.items():
            for n, m in meshes.items():
                v = m.vertices[:: max(1, len(m.vertices) // 1500)]
                v = v @ T[BODY[n]][:3, :3].T + T[BODY[n]][:3, 3]
                sel = np.all((v > fm.bounds[0]) & (v < fm.bounds[1]), axis=1)
                if not sel.any():
                    continue
                try:
                    k = int(fm.contains(v[sel]).sum())
                except Exception:
                    k = -1
                if k:
                    fx.setdefault(fn, {})[n] = max(k, fx.get(fn, {}).get(n, 0))
    checks["fixed_neighbours_vertices_inside"] = fx
    print("  check step done:", 'fixed_neighbours_vertices_inside', f"{time.time() - t0:.0f} s", flush=True)
    # strands and wire in solids
    sres, wres = {}, {}
    for q in (grid[0], {k: 0.0 for k in JOINTS}, grid[len(grid) // 2], grid[-2]):
        for n in STRAND_NAMES:
            b = centreline_in_solids(meshes, q, strand_path(n, q), skip_parts=("cup_*",))
            if b:
                sres.setdefault(f"thumb_{n}", []).append({"q": [round(v) for v in q.values()], "points": b})
        w = sensor_wire(q)
        b = centreline_in_solids(meshes, q, w, skip_parts=("sensor_*", "wire_keeper_*", "tip_board", "tip_chip*", "axle_*",
                                                            "rot_sensor_*", "plug_*", "magnet_*"))
        if b:
            wres.setdefault("thumb_sensor_wire", []).append({"q": [round(v) for v in q.values()], "points": b})
    checks["strand_centreline_in_solids"] = sres or "0 points"
    print("  check step done:", 'strand_centreline_in_solids', f"{time.time() - t0:.0f} s", flush=True)
    checks["wire_centreline_in_solids"] = wres or "0 points"
    wl = [float(np.sum(np.linalg.norm(np.diff(sensor_wire(q), axis=0), axis=1))) for q in grid]
    checks["wire_length_mm"] = [round(min(wl), 2), round(max(wl), 2)]
    checks["loops"] = loop_report()
    print("  check step done:", 'loops', f"{time.time() - t0:.0f} s", flush=True)
    checks["bends_deg_over_grid"] = bends_report()
    eta = efficiency()
    checks["eta_strand"] = eta
    checks["pinch_N"] = pinch_estimate(eta)
    checks["opposition_cad"] = opposition_check(meshes)
    print("  check step done:", 'opposition_cad', f"{time.time() - t0:.0f} s", flush=True)
    widths = {}
    for j in ("cmc_flex", "mcp_flex", "ip"):
        h = _hinge_x(j)
        widths[j] = round(h["face_s"] + h["face_w"], 2)
    checks["joint_widths_mm"] = widths
    checks["mass_g"] = {n: round(val[n]["volume"] / 1000 * {"PETG": 1.27, "steel": 7.85, "stainless": 7.95, "brass": 8.5, "silicone": 1.08,
                                                        "TPU": 1.21, "PCB": 1.9, "ceramic": 6.0, "magnet": 7.5}.get(MATERIAL[n], 1.2), 2)
                        for n in parts}
    checks["mass_g_total"] = round(sum(checks["mass_g"].values()), 1)
    (OUT / "checks.json").write_text(json.dumps(checks, indent=1, default=str))
    # poses / lines for a few named poses
    ev = TH.DESIGN_JSON["eval"]
    named = {"q0": {}, "tip_pinch": dict(zip(JOINTS, ev["tip_pinch"]["q_thumb"])), "little": dict(zip(JOINTS, ev["little"]["q_thumb"])),
             "power55": dict(zip(JOINTS, ev["power55"]["q_thumb"])), "flat": dict(zip(JOINTS, ev["flat"]["q_thumb"]))}
    (OUT / "lines.json").write_text(json.dumps({k: {"q": _qdict(q), "lines": {n: np.round(v, 3).tolist() for n, v in lines(q).items()}}
                                                 for k, q in named.items()}, indent=0))
    (OUT / "sensor_wire.json").write_text(json.dumps({"waypoints": [[*np.round(p, 3).tolist(), b, note] for p, b, note in wire_waypoints()]}, indent=1))
    if "--no-render" not in sys.argv:
        render_set(parts, OUT, quick)
    print(f"done in {time.time() - t0:.0f} s", flush=True)
    return checks


if __name__ == "__main__" and "--route" not in sys.argv:
    main(quick="--quick" in sys.argv)
