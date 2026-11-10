"""Tendra V2 concept thumb (study 2026-11-09, thumb designer): CadQuery model, tendon routing, checks.

Kinematics come from thumb_kinematics.py (optimised "roll-pitch" CMC, see reports/02-thumb.md):
- cmc_rot: axis a1 through the CMC centre P. The base turns on a hollow printed sleeve (OD 20) in two
  thin-section 6704 bearings held by the fixed `thumb_mount` (bolted into the palm's thenar bay).
- cmc_flex: axis a2, perpendicular to a1 and THROUGH P (the axes intersect, unlike V1's 5.5 mm offset).
  The metacarpal hub turns on two 3 mm steel stub pins pressed into the base's cheeks.
- mcp_flex (axis a3) and ip (axis a4): 2 mm steel stub pins in the parent's fork cheeks.
Every joint has a 7 mm drum (strand centreline) on its child, centred on its axis (params.R_DRUM).

Tendon routing (the point of this design): all 6 strands of cmc_flex, mcp_flex and ip come up the hollow
cmc_rot axis: the 6 PTFE sheaths end in a hex bundle in the mount, the bare strands converge through one
PTFE neck eyelet ON the rot axis (it turns with the base), so cmc_rot only twists them around each other.
The mcp and ip strands then cross the cmc_flex axis exactly ON it (they come up a1, which intersects a2
at P) between two 1 mm steel nip pins, and the ip strands cross the
MCP axis the same way. Nothing in the thumb is a free sheath loop. Numbers: `routing_report()`.

API: build(q) -> {"parts", "joints", "strands", "interface"}; q in degrees (cmc_rot, cmc_flex, mcp_flex,
ip), 0 = rest pose (thumb flat beside the index, see the report), positive = closing.
Run: uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy
     --with pillow python cad/thumb.py [--quick]
"""

from __future__ import annotations

import functools
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cadquery as cq  # noqa: E402

import params as P  # noqa: E402
import thumb_kinematics as K  # noqa: E402

OUT = HERE.parent / "out" / "thumb"
R = P.R_DRUM  # 7.0, strand centreline on every drum
LINE_R = P.LINE_D / 2
GROOVE_FLOOR = R - LINE_R  # 6.82
NIP_PIN_D = 1.0  # steel nip pins (1 mm dowel / music wire)
NIP_RC = NIP_PIN_D / 2 + LINE_R  # strand-centre wrap radius on a nip pin
NIP_OFF = NIP_RC + 0.03  # pin centre offset from the crossing point (0.06 mm gap at the axis)
CL = P.CLEAR_SLIDE  # 0.25

# --- Dimensions (frame-local mm) -------------------------------------------------------------------
# Base frame C: origin P, x = a2 (cmc_flex axis), z = a1 (cmc_rot axis), y = z x x.
HUB_CMC_R, HUB_CMC_W = 9.5, 10.0  # metacarpal hub radius, half-width along a2
CHEEK_CMC = (HUB_CMC_W + CL, HUB_CMC_W + CL + 3.0)  # base cheek x-span (|x|)
CMC_GROOVE_X = 4.0  # cmc_flex grooves at x = -4 (flex) / +4 (ext)
CMC_SLOT = 2.3  # central slot |x| < 2.3 for the 4 crossing strands
CMC_CROSS_X = {"mcp_flex": -1.8, "ip_flex": -0.6, "ip_ext": 0.6, "mcp_ext": 1.8}
STUB_CMC = (5.5, HUB_CMC_W, 3.0)  # stub pin from |x| 5.5 into the hub, through the cheek; diameter 3
PLATE_Z = (-12.6, -HUB_CMC_R - 0.3)  # base plate under the hub
DRUM_ROT_Z = (-17.2, -12.6)  # rot drum disc (its bottom face bears on the upper bearing's inner race)
ROT_GROOVE_Z = {"flex": -16.0, "ext": -14.2}
ROT_DRUM_FLANGE = 8.6
BEARING = {"name": "6704ZZ", "id": 20.0, "od": 27.0, "w": 4.0}  # thin-section, 2 back to back
BEAR_TOP_Z, BEAR_BOT_Z = (-21.2, -17.2), (-25.4, -21.4)
SLEEVE_Z = (-25.4, -17.2)  # printed journal sleeve of the base (OD 20) in the bearings
CAP_Z = (-26.6, -25.4)  # base cap: holds the lower inner race, carries the top neck eyelet
NECK_Z = -26.0  # top neck (PTFE eyelet, 1.6 mm hole) on the rot axis, turns with the base
NECK_R = 0.45  # strand offsets inside the neck
SHEATH_END_Z = -32.0  # the 6 sheaths end here in a hex bundle (r 2.3), clamped in the mount
BUNDLE_R = 2.3
MOUNT_Z = (-37.0, -17.2)
MOUNT_OD = 31.0
FLANGE_Z, FLANGE_OD, BOLT_R = (-30.0, -27.0), 42.0, 18.5
SPIN_ANG = {"cmc_flex_ext": 0, "ip_ext": 60, "ip_flex": 120, "cmc_flex_flex": 180, "mcp_flex": 240,
            "mcp_ext": 300}
ROT_SOCKET_W = (15.3, 23.5)  # rot strand sheath sockets along w (strand exit direction)
POST_R = (15.6, 18.6)
LUG_R = (10.0, 18.0)
LUG_Z = (-12.4, -10.1)
LUG_HALF = 7.0  # deg
STOP_TOP = -10.5

# Metacarpal / MCP / IP
SHAFT_MC = (18.0, 15.0)  # width (x) x height (y)
HUB_MCP_R, HUB_MCP_W = 8.5, 7.5
CHEEK_MCP = (HUB_MCP_W + CL, HUB_MCP_W + CL + 2.0)
MCP_GROOVE_X = 3.5
MCP_SLOT = 1.4
MCP_CROSS_X = {"ip_flex": -0.6, "ip_ext": 0.6}
SHAFT_PX = (16.0, 16.0)
HUB_IP_R, HUB_IP_W = 8.0, 7.5
CHEEK_IP = (HUB_IP_W + CL, HUB_IP_W + CL + 1.8)
IP_GROOVE_X = 1.0
STUB_SMALL_D = 2.0
POCKET_D = 2.2  # crimp-bead anchor pocket


# --- Design -----------------------------------------------------------------------------------------
def load_design():
    p = OUT / "thumb_design.json"
    return K.ThumbDesign.from_json(json.loads(p.read_text()))


D = load_design()
RANGES = D.ranges


def _frames(td):
    p1, a1 = td.joints[0]
    _, a2 = td.joints[1]
    p3, a3 = td.joints[2]
    p4, a4 = td.joints[3]
    m0 = K.unit(p3 - p1)
    md = K.unit(td.tip - p4)
    J = {
        "C": (p1, a2, np.cross(a1, a2), a1),
        "cmc_rot": (p1, a1, a2, np.cross(a1, a2)),
        "cmc_flex": (p1, a2, np.cross(m0, a2), m0),
        "mcp_flex": (p3, a3, np.cross(m0, a3), m0),
        "ip": (p4, a4, np.cross(md, a4), md),
    }
    for k, (o, x, y, z) in J.items():
        x = K.unit(x)
        z = K.unit(z - (z @ x) * x)
        J[k] = (np.asarray(o, float), x, np.cross(z, x), z)
    return J, m0


J0, M0 = _frames(D)
L1 = float(np.linalg.norm(D.joints[2][0] - D.joints[0][0]))
L2 = float(np.linalg.norm(D.joints[3][0] - D.joints[2][0]))
L3 = float(np.linalg.norm(D.tip - D.joints[3][0]))
BODY_OF_JOINT = {"cmc_rot": "base", "cmc_flex": "metacarpal", "mcp_flex": "proximal", "ip": "distal"}
BODIES = ("mount", "base", "metacarpal", "proximal", "distal")
JOINT_NAMES = ("cmc_rot", "cmc_flex", "mcp_flex", "ip")


def to_world(frame, p):
    o, x, y, z = frame
    p = np.asarray(p, float)
    return o + p[..., 0, None] * x + p[..., 1, None] * y + p[..., 2, None] * z if p.ndim > 1 else \
        o + p[0] * x + p[1] * y + p[2] * z


def to_local(frame, pw):
    o, x, y, z = frame
    d = np.asarray(pw, float) - o
    return np.array([d @ x, d @ y, d @ z])


def _trsf(frame):
    """gp_Trsf mapping frame-local coordinates to world."""
    from OCP.gp import gp_Ax3, gp_Dir, gp_Pnt, gp_Trsf
    o, x, y, z = frame
    t = gp_Trsf()
    t.SetTransformation(gp_Ax3(gp_Pnt(*map(float, o)), gp_Dir(*map(float, z)), gp_Dir(*map(float, x))))
    t.Invert()
    return t


def _apply(shape, t):
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, t, True).Shape())


def place(shape, frame):
    """Local-frame solid -> world (rest pose)."""
    return _apply(shape, _trsf(frame))


# --- Primitive solids (local frame) -----------------------------------------------------------------
V = cq.Vector


def cyl_x(r, x0, x1, y=0.0, z=0.0):
    return cq.Solid.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def cyl_ab(a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    return cq.Solid.makeCylinder(r, float(np.linalg.norm(d)), V(*a), V(*d))


def sphere(r, c):
    """Full sphere (cq's makeSphere defaults to a hemisphere)."""
    return cq.Solid.makeSphere(r, V(*c), V(0, 0, 1), -90, 90, 360)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def ring_x(r_in, r_out, x0, x1):
    return cyl_x(r_out, x0, x1).cut(cyl_x(r_in, x0, x1))


def ring_z(r_in, r_out, z0, z1):
    return cyl_z(r_out, z0, z1).cut(cyl_z(r_in, z0, z1))


def _pol(r, a):
    return (r * np.cos(np.radians(a)), r * np.sin(np.radians(a)))


def sector_x(r, x0, x1, a0, a1, n=None):
    """Pie slice about the local x axis, angles in the (y, z) plane (deg, from +y toward +z)."""
    wp = cq.Workplane("YZ", origin=(x0, 0, 0)).moveTo(0, 0).lineTo(*_pol(r, a0)) \
        .threePointArc(_pol(r, 0.5 * (a0 + a1)), _pol(r, a1)).close()
    return wp.extrude(x1 - x0).val()


def _annulus_wp(wp, r_in, r_out, a0, a1):
    am = 0.5 * (a0 + a1)
    return wp.moveTo(*_pol(r_out, a0)).threePointArc(_pol(r_out, am), _pol(r_out, a1)) \
        .lineTo(*_pol(r_in, a1)).threePointArc(_pol(r_in, am), _pol(r_in, a0)).close()


def sector_z(r_in, r_out, z0, z1, a0, a1, n=None):
    """Annular sector about local z, angles in the (x, y) plane from +x toward +y."""
    return _annulus_wp(cq.Workplane("XY", origin=(0, 0, z0)), r_in, r_out, a0, a1).extrude(z1 - z0).val()


def rrect_wire(w, h, c, z, frame_xy=None):
    """Chamfered rectangle (8 points) at height z, optionally in a rotated (x, y) basis (the basis is
    turned by 180 deg if that gives less twist relative to the standard one: the shape is symmetric)."""
    if frame_xy is not None and frame_xy[0][0] < 0:
        frame_xy = (-frame_xy[0], -frame_xy[1])
    pts2 = [(-w / 2 + c, -h / 2), (w / 2 - c, -h / 2), (w / 2, -h / 2 + c), (w / 2, h / 2 - c),
            (w / 2 - c, h / 2), (-w / 2 + c, h / 2), (-w / 2, h / 2 - c), (-w / 2, -h / 2 + c)]
    ex, ey = frame_xy if frame_xy is not None else (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))
    pts = [V(*(px * ex + py * ey + np.array([0, 0, z]))) for px, py in pts2]
    return cq.Wire.makePolygon(pts + [pts[0]])


def prism(w, h, c, z0, z1, y0=0.0):
    """Chamfered rectangular prism along local z."""
    pts = [(-w / 2 + c, -h / 2), (w / 2 - c, -h / 2), (w / 2, -h / 2 + c), (w / 2, h / 2 - c),
           (w / 2 - c, h / 2), (-w / 2 + c, h / 2), (-w / 2, h / 2 - c), (-w / 2, -h / 2 + c)]
    pts = [(x, y + y0) for x, y in pts]
    return cq.Workplane("XY", origin=(0, 0, z0)).polyline(pts).close().extrude(z1 - z0).val()


def loft(wires):
    return cq.Solid.makeLoft(wires, True)


def main_solid(s, vmin=1.0):
    """Drop zero-volume slivers that boolean cuts sometimes leave."""
    sols = [x for x in s.Solids() if x.Volume() > vmin]
    return sols[0] if len(sols) == 1 else cq.Compound.makeCompound(sols)


def fuse(*shapes):
    s = shapes[0]
    for t in shapes[1:]:
        s = s.fuse(t)
    return s.clean()


def cut(s, *tools):
    if DEBUG_CUT:
        for i, t in enumerate(tools):
            s = s.cut(t)
            print("   cut", i, t.Volume(), flush=True)
        return s
    for t in tools:
        s = s.cut(t)
    return s


DEBUG_CUT = False


# --- Strand layout (rest pose, world) ----------------------------------------------------------------
def joint_frame_q(j, q, Fq):
    """Joint frame (world) of joint j at pose q, given body transforms Fq."""
    o, x, y, z = J0[j]
    Rm, t = Fq[BODY_OF_JOINT[j]]
    return (Rm @ o + t, Rm @ x, Rm @ y, Rm @ z)


def body_transforms(q):
    qq = [q.get(n, 0.0) for n in JOINT_NAMES] if isinstance(q, dict) else list(q)
    F = D.chain.frames(np.radians(qq))
    return {"mount": (np.eye(3), np.zeros(3)), "base": F[0], "metacarpal": F[1], "proximal": F[2],
            "distal": F[3]}


def _wpt(node, Fq):
    Rm, t = Fq[node["body"]]
    return Rm @ node["p"] + t


def mid(j):
    return 0.5 * (RANGES[j][0] + RANGES[j][1])


ROT_MID = mid("cmc_rot")


def _spin_pt(name, z, angle_off=0.0, r=BUNDLE_R):
    a = np.radians(SPIN_ANG[name] + angle_off)
    return to_world(J0["C"], [r * np.cos(a), r * np.sin(a), z])


def node(st, lab):
    return next(n for n in st["nodes"] if n["lab"] == lab)


def _w_dir():
    """Rot-strand exit direction w in the C frame's (x, y) plane: toward the wrist (-Z world)."""
    o, x, y, z = J0["C"]
    v = np.array([0, 0, -1.0])
    v = v - (v @ z) * z
    if np.linalg.norm(v) < 0.2:
        v = -y
    v = K.unit(v)
    return np.array([v @ x, v @ y])  # local (x, y) components


W2 = _w_dir()
W_ANG = float(np.degrees(np.arctan2(W2[1], W2[0])))


def strand_layout():
    """Nodes per strand at rest (world points + owning body)."""
    S = {}
    # cmc_rot: from the mount's sheath socket straight to the drum groove (tangent)
    lat = np.array([-W2[1], W2[0]])
    # which lateral side gives a positive (closing) torque when pulled? -> that one is the flex strand
    q0 = np.array([0.0, 0.0])
    Qp = to_local(J0["cmc_rot"], to_world(J0["C"], [*(ROT_SOCKET_W[0] * W2 + R * lat), -18.0]))[1:]
    Tn = min((_tan_pts(Qp)), key=lambda T: np.linalg.norm(T - Qp))
    flex_sgn = +1 if cross2(Tn, Qp) > 0 else -1
    for side, sgn in (("flex", flex_sgn), ("ext", -flex_sgn)):
        z = ROT_GROOVE_Z[side]
        nodes = []
        for lab, w in (("exit", ROT_SOCKET_W[1]), ("socket", ROT_SOCKET_W[0])):
            xy = w * W2 + sgn * R * lat
            nodes.append({"type": "pt", "body": "mount", "p": to_world(J0["C"], [xy[0], xy[1], z]), "lab": lab})
        S[f"cmc_rot_{side}"] = {"joint": "cmc_rot", "role": side, "nodes": nodes,
                                "drum": {"joint": "cmc_rot", "groove_x": z, "body": "base"}, "sheathed_exit": True}
    # strands up the rot axis: sheath bundle end (mount) -> neck (base) -> ...
    for name in ("cmc_flex_flex", "cmc_flex_ext", "mcp_flex", "mcp_ext", "ip_flex", "ip_ext"):
        j = "cmc_flex" if name.startswith("cmc_flex") else ("mcp_flex" if name.startswith("mcp") else "ip")
        role = name.rsplit("_", 1)[1]
        key = name
        nodes = [
            {"type": "pt", "body": "mount", "lab": "exit", "p": _spin_pt(key, MOUNT_Z[0], ROT_MID, BUNDLE_R)},
            {"type": "pt", "body": "mount", "lab": "socket", "p": _spin_pt(key, SHEATH_END_Z, ROT_MID, BUNDLE_R)},
            {"type": "pt", "body": "base", "lab": "neck", "p": _spin_pt(key, NECK_Z, 0.0, NECK_R)},
        ]
        if j in ("mcp_flex", "ip"):
            nodes.append({"type": "nip", "joint": "cmc_flex", "body": "metacarpal", "lab": "cmc_cross",
                          "p": to_world(J0["cmc_flex"], [CMC_CROSS_X[name], 0, 0])})
        if j == "ip":
            nodes.append({"type": "nip", "joint": "mcp_flex", "body": "proximal", "lab": "mcp_cross",
                          "p": to_world(J0["mcp_flex"], [MCP_CROSS_X[name], 0, 0])})
        gx = {"cmc_flex": CMC_GROOVE_X, "mcp_flex": MCP_GROOVE_X, "ip": IP_GROOVE_X}[j] * (-1 if role == "flex" else 1)
        S[f"{j}_{role}"] = {"joint": j, "role": role, "nodes": nodes,
                            "drum": {"joint": j, "groove_x": gx, "body": BODY_OF_JOINT[j]}, "sheathed_exit": True}
    return S


def _tan_pts(Q2):
    d = np.linalg.norm(Q2)
    base = np.arctan2(Q2[1], Q2[0])
    off = np.arccos(R / d)
    return [R * np.array([np.cos(a), np.sin(a)]) for a in (base + off, base - off)]


def cross2(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def _tangent(Q2, side):
    """Tangent point on circle R (origin) from external 2D point Q2; side +1 = positive torque about +x."""
    d = np.linalg.norm(Q2)
    base = np.arctan2(Q2[1], Q2[0])
    off = np.arccos(R / d)
    for a in (base + off, base - off):
        T = R * np.array([np.cos(a), np.sin(a)])
        if np.sign(cross2(T, Q2)) == side:  # torque of a pull toward Q at T about +x
            return T, a
    raise RuntimeError("tangent")


def drum_segment(st, Q_w, Fq, q):
    """Length from the last free point Q to the drum anchor, the tangent point (world) and the wrap."""
    dr = st["drum"]
    j = dr["joint"]
    fr = joint_frame_q(j, q, Fq)
    Ql = to_local(fr, Q_w)
    side = +1 if st["role"] == "flex" else -1
    T2, aT = _tangent(Ql[1:], side)
    # travel direction past T (away from Q): ccw if (T - Q) . ccw_tangent > 0
    ccw = np.array([-np.sin(aT), np.cos(aT)])
    sense = 1 if (T2 - Ql[1:]) @ ccw > 0 else -1
    wrap = None
    if "anchor" in dr:
        wrap = (sense * (dr["anchor"] - aT)) % (2 * np.pi)
    Lt = np.sqrt(max(0.0, Ql[1:] @ Ql[1:] - R * R) + (Ql[0] - dr["groove_x"]) ** 2)
    Tw = to_world(fr, [dr["groove_x"], T2[0], T2[1]])
    return Lt, Tw, aT, sense, wrap


def _belt(A, B, N, pins, gate_dirs):
    """2D: path A -> (between the two nip pins) -> B. pins = centres, gate_dirs = unit from centre to N.
    Returns the path length (wrapping whichever pin the straight line would cross)."""
    straight = np.linalg.norm(B - A)
    best = None
    for c, g in zip(pins, gate_dirs):
        dA, dB = np.linalg.norm(A - c), np.linalg.norm(B - c)
        a, b, gg = (np.arctan2(*(v[::-1])) for v in (A - c, B - c, g))
        ccw = (b - a) % (2 * np.pi)
        sweep = ccw if (gg - a) % (2 * np.pi) < ccw else 2 * np.pi - ccw
        phi = sweep - np.arccos(NIP_RC / dA) - np.arccos(NIP_RC / dB)
        if phi > 0 and (np.pi - sweep) < np.pi:
            Lb = np.sqrt(dA ** 2 - NIP_RC ** 2) + np.sqrt(dB ** 2 - NIP_RC ** 2) + NIP_RC * phi
            # only valid if the straight line passes on the far side of this pin's gate
            best = Lb if best is None else max(best, Lb)
    return best if best is not None else straight


def nip_correction(node, A_w, B_w, Fq, q):
    """Length change from the nip pins vs passing exactly through the crossing point."""
    fr = joint_frame_q(node["joint"], q, Fq)
    A, B, N = (to_local(fr, p) for p in (A_w, B_w, _wpt(node, Fq)))
    g = node["gate"]  # pin offset direction (2D unit) in the joint frame, fixed in the child
    pins = [N[1:] + NIP_OFF * g, N[1:] - NIP_OFF * g]
    gates = [-g, g]
    A2, B2, N2 = A[1:], B[1:], N[1:]
    Lb = _belt(A2, B2, N2, pins, gates)
    Lp = np.linalg.norm(A2 - N2) + np.linalg.norm(B2 - N2)
    dx = abs(B[0] - A[0])
    L3 = np.sqrt(Lp ** 2 + dx ** 2)
    return (Lb - Lp) * Lp / L3


def strand_length(st, q, detail=False):
    Fq = body_transforms(q)
    pts = [_wpt(n, Fq) for n in st["nodes"]]
    Lt, Tw, aT, sense, wrap = drum_segment(st, pts[-1], Fq, q)
    L = sum(np.linalg.norm(pts[i + 1] - pts[i]) for i in range(len(pts) - 1))
    # spinnerets: chord between holes is already in the polyline (twist)
    corr = 0.0
    nxt = pts[1:] + [Tw]
    for i, n in enumerate(st["nodes"]):
        if n["type"] == "nip":
            corr += nip_correction(n, pts[i - 1], nxt[i], Fq, q)
    total = L + corr + Lt + (R * wrap if wrap is not None else 0.0)
    if detail:
        return total, {"poly": L, "nip": corr, "tangent": Lt, "wrap_deg": None if wrap is None else np.degrees(wrap),
                       "points": pts + [Tw]}
    return total


def _setup_strands():
    S = strand_layout()
    # nip gate directions: bisector of in/out directions at the joint's mid-range, in the child's frame
    for st in S.values():
        for i, n in enumerate(st["nodes"]):
            if n["type"] != "nip":
                continue
            q = {k: 0.0 for k in JOINT_NAMES}
            q[n["joint"]] = mid(n["joint"])
            Fq = body_transforms(q)
            fr = joint_frame_q(n["joint"], q, Fq)
            pts = [_wpt(m, Fq) for m in st["nodes"]]
            if i + 1 < len(pts):
                Bw = pts[i + 1]
            else:
                Bw = to_world(joint_frame_q(st["drum"]["joint"], q, Fq), [st["drum"]["groove_x"], 0, 0])
            A, B, N = (to_local(fr, p)[1:] for p in (pts[i - 1], Bw, pts[i]))
            u_in, u_out = K.unit(N - A), K.unit(B - N)
            gdir = K.unit(u_in + u_out)
            n["gate"] = np.array([-gdir[1], gdir[0]])
    # drum anchors: min wrap 25 deg over the joint's range
    for st in S.values():
        j = st["drum"]["joint"]
        angs, senses = [], []
        for v in np.linspace(*RANGES[j], 21):
            q = {k: 0.0 for k in JOINT_NAMES}
            q[j] = v
            Fq = body_transforms(q)
            pts = [_wpt(m, Fq) for m in st["nodes"]]
            _, _, aT, sense, _ = drum_segment(st, pts[-1], Fq, q)
            # express the tangent angle in the child's frame at rest: the child turned by v about +x
            angs.append(aT)
            senses.append(sense)
        assert len(set(senses)) == 1, "wrap sense flips"
        sense = senses[0]
        a = np.unwrap(np.array(angs))
        st["drum"]["anchor"] = float((np.max(sense * a) * sense + sense * np.radians(25.0)) % (2 * np.pi))
        st["drum"]["sense"] = sense
        st["drum"]["max_wrap_deg"] = float(np.degrees(np.ptp(a)) + 25.0)
    return S


STRANDS = _setup_strands()


# --- Parts (rest pose) -------------------------------------------------------------------------------
def _anchor_pocket(j, st):
    """Crimp-bead pocket: radial bore from the hub surface to r 3.5 at the anchor angle (joint frame)."""
    a = st["drum"]["anchor"]
    gx = st["drum"]["groove_x"]
    hub_r = {"cmc_rot": ROT_DRUM_FLANGE, "cmc_flex": HUB_CMC_R, "mcp_flex": HUB_MCP_R, "ip": HUB_IP_R}[j]
    d = np.array([0, np.cos(a), np.sin(a)])
    r0 = 3.6 if j != "cmc_rot" else 4.6
    return cyl_ab(np.array([gx, 0, 0]) + r0 * d, np.array([gx, 0, 0]) + (hub_r + 0.5) * d, POCKET_D / 2)


def _grooves(j, xs, hub_r):
    """Annular strand grooves about the joint frame's x axis (1 mm wide floor, flared outer 1.8 mm)."""
    out = []
    for x in xs:
        out.append(ring_x(GROOVE_FLOOR, 8.0 if hub_r > 8.2 else hub_r + 0.5, x - 0.5, x + 0.5))
        if hub_r > 8.2:
            out.append(ring_x(8.0, hub_r + 0.5, x - 0.9, x + 0.9))
    return out


def _local_strand_bores(fr, segs, r):
    """Bores (in frame-local coordinates) along world segments."""
    return [cyl_ab(to_local(fr, a), to_local(fr, b), r) for a, b in segs]


def make_mount():
    """Fixed part, bolted into the palm's thenar bay: bearing housing (2 x 6704), flange, rot hard-stop
    posts, sheath sockets for the two cmc_rot strands, and the clamp for the 6-sheath bundle."""
    C = J0["C"]
    s = fuse(cyl_z(MOUNT_OD / 2, MOUNT_Z[0], MOUNT_Z[1]), ring_z(5, FLANGE_OD / 2, *FLANGE_Z))
    rng = RANGES["cmc_rot"]
    lug_rest = W_ANG + 180.0 - ROT_MID  # lug angle at q = 0 (at the back at mid-range)
    post_half = np.degrees(1.75 / np.mean(POST_R))
    for end, sgn in ((rng[1], +1), (rng[0], -1)):
        a = lug_rest + end + sgn * (LUG_HALF + post_half)
        s = s.fuse(sector_z(POST_R[0] - 0.5, POST_R[1], MOUNT_Z[1] - 4.0, STOP_TOP, a - post_half, a + post_half))
    s = s.fuse(ring_z(MOUNT_OD / 2 - 1, POST_R[1], MOUNT_Z[1] - 4.0, MOUNT_Z[1]))
    blk = box(ROT_SOCKET_W[0], ROT_SOCKET_W[1], -10.0, 10.0, MOUNT_Z[1] - 4.0, STOP_TOP)
    s = s.fuse(blk.rotate(V(0, 0, 0), V(0, 0, 1), W_ANG))
    tools = [cyl_z(BEARING["od"] / 2, BEAR_BOT_Z[0], MOUNT_Z[1] + 1),  # bearing seat (shoulder below)
             cyl_z(BEARING["od"] / 2 - 1.5, CAP_Z[0] - 0.4, BEAR_BOT_Z[0] + 0.01),  # room for the base cap
             cyl_z(3.0, SHEATH_END_Z, CAP_Z[0] - 0.39)]  # strand fan from the sheath ends to the neck
    for b in range(3):
        a = np.radians(W_ANG + 180 + (b - 1) * 100)
        tools.append(cyl_z(1.7, FLANGE_Z[0] - 1, FLANGE_Z[1] + 1, BOLT_R * np.cos(a), BOLT_R * np.sin(a)))
    for st in STRANDS.values():
        e, k = to_local(C, node(st, "exit")["p"]), to_local(C, node(st, "socket")["p"])
        if st["joint"] == "cmc_rot":
            tools.append(cyl_ab(e + 2.0 * K.unit(e - k), k, P.SHEATH_BORE / 2))
        else:
            tools.append(cyl_ab(e - np.array([0, 0, 0.5]), k, P.SHEATH_BORE / 2))
    # clear the base drum and the rot strands' path inside the block radius
    tools.append(cyl_z(ROT_SOCKET_W[0], MOUNT_Z[1], STOP_TOP + 1))
    s = cut(s, *tools)
    return place(s, C)


def make_cap():
    """Screw-on cap under the base sleeve (3 x M2 into the sleeve): clamps the lower inner race."""
    return place(cut(cyl_z(BEARING["id"] / 2 + 1.0, *CAP_Z), cyl_z(3.05, CAP_Z[0] - 1, CAP_Z[1] + 1)), J0["C"])


def make_neck():
    """PTFE eyelet on the rot axis (all 6 strands pass its 1.6 mm hole), pressed into the cap."""
    return place(cut(cyl_z(3.0, *CAP_Z), cyl_z(0.8, CAP_Z[0] - 1, CAP_Z[1] + 1)), J0["C"])


def make_bearing(z):
    return place(ring_z(BEARING["id"] / 2, BEARING["od"] / 2, *z), J0["C"])


def make_base():
    C = J0["C"]
    s = fuse(cyl_z(BEARING["id"] / 2, SLEEVE_Z[0], DRUM_ROT_Z[0] + 0.01), cyl_z(ROT_DRUM_FLANGE, *DRUM_ROT_Z),
             cyl_z(10.5, PLATE_Z[0], PLATE_Z[1]))
    for sgn in (+1, -1):
        x0, x1 = (CHEEK_CMC[0], CHEEK_CMC[1]) if sgn > 0 else (-CHEEK_CMC[1], -CHEEK_CMC[0])
        s = s.fuse(fuse(box(x0, x1, -7.0, 7.0, PLATE_Z[0], 0.0), cyl_x(7.0, x0, x1)))
    s = fuse(s, box(-CHEEK_CMC[1], CHEEK_CMC[1], -7.0, 7.0, PLATE_Z[0], PLATE_Z[1]))
    lug_a = W_ANG + 180.0 - ROT_MID
    s = s.fuse(sector_z(LUG_R[0], LUG_R[1], LUG_Z[0], LUG_Z[1], lug_a - LUG_HALF, lug_a + LUG_HALF))
    tools = [cyl_z(4.0, SLEEVE_Z[0] - 0.01, PLATE_Z[1] + 0.01)]
    for k in range(3):  # cap screws M2
        a = np.radians(30 + 120 * k)
        tools.append(cyl_z(0.8, SLEEVE_Z[0] - 0.01, SLEEVE_Z[0] + 6, 7.0 * np.cos(a), 7.0 * np.sin(a)))
    for side in ("flex", "ext"):
        z = ROT_GROOVE_Z[side]
        tools.append(ring_z(GROOVE_FLOOR, ROT_DRUM_FLANGE + 1, z - 0.5, z + 0.5))
        a = STRANDS[f"cmc_rot_{side}"]["drum"]["anchor"]
        d = np.array([np.cos(a), np.sin(a), 0])
        tools.append(cyl_ab(np.array([0, 0, z]) + 4.6 * d, np.array([0, 0, z]) + 9.2 * d, POCKET_D / 2))
    tools += [cyl_x(STUB_CMC[2] / 2 + P.PRESS_FIT / 2, CHEEK_CMC[0] - 0.1, CHEEK_CMC[1] + 0.1),
              cyl_x(STUB_CMC[2] / 2 + P.PRESS_FIT / 2, -CHEEK_CMC[1] - 0.1, -CHEEK_CMC[0] + 0.1)]
    # strand passages above the top spinneret
    segs = []
    for st in STRANDS.values():
        if st["joint"] == "cmc_rot":
            continue
        n = st["nodes"]
        t = node(st, "neck")["p"]
        if st["joint"] == "cmc_flex":
            Fq = body_transforms({})
            _, Tw, *_ = drum_segment(st, t, Fq, {})
            segs.append((t, Tw))
        else:
            segs.append((t, node(st, "cmc_cross")["p"]))
    tools += _local_strand_bores(C, segs, 0.75)
    # clearance for the metacarpal hub (cylinder about x) between the cheeks
    tools.append(cyl_x(HUB_CMC_R + 0.3, -CHEEK_CMC[0], CHEEK_CMC[0]))
    s = cut(s, *tools)
    return place(s, C)


def make_metacarpal():
    F = J0["cmc_flex"]
    Fm = J0["mcp_flex"]
    hub = cyl_x(HUB_CMC_R, -HUB_CMC_W, HUB_CMC_W)
    # shaft: loft from the hub (metacarpal frame) to the MCP fork (MCP frame, twisted by tau3)
    ex = to_local(F, F[0] + Fm[1]) - 0.0
    exm = np.array([Fm[1] @ F[1], Fm[1] @ F[2], Fm[1] @ F[3]])
    eym = np.array([Fm[2] @ F[1], Fm[2] @ F[2], Fm[2] @ F[3]])
    zf = L1 - HUB_MCP_R - 0.6
    # shaft: prism in the metacarpal frame + a short prism in the MCP frame (MCP axis twisted by tau3)
    zf = L1 - HUB_MCP_R - 0.6
    shaft = fuse(prism(SHAFT_MC[0], SHAFT_MC[1], 3.5, 4.0, zf - 4.0),
                 _local_from(Fm, F, prism(2 * CHEEK_MCP[1], SHAFT_MC[1], 3.5, -(L1 * 0.45), -(HUB_MCP_R + 0.6))))
    s = fuse(hub, shaft)
    # MCP fork cheeks (MCP frame)
    for sgn in (+1, -1):
        x0, x1 = (CHEEK_MCP[0], CHEEK_MCP[1]) if sgn > 0 else (-CHEEK_MCP[1], -CHEEK_MCP[0])
        ch = fuse(box(x0, x1, -SHAFT_MC[1] / 2, SHAFT_MC[1] / 2, -(HUB_MCP_R + 2.0), 0.0), cyl_x(6.5, x0, x1))
        s = s.fuse(_local_from(Fm, F, ch))
    # TPU thenar pad seat is a separate part; cut grooves, slot, pockets, stub/nip holes, strand channels
    tools = _grooves("cmc_flex", (-CMC_GROOVE_X, CMC_GROOVE_X), HUB_CMC_R)
    for side in ("flex", "ext"):
        tools.append(_anchor_pocket("cmc_flex", STRANDS[f"cmc_flex_{side}"]))
    # fan slot for the 4 crossing strands (incoming from below over the cmc_flex range)
    angs = []
    for v in np.linspace(*RANGES["cmc_flex"], 9):
        q = {"cmc_flex": v}
        Fq = body_transforms(q)
        fr = joint_frame_q("cmc_flex", q, Fq)
        for st in STRANDS.values():
            if st["joint"] in ("mcp_flex", "ip"):
                A = to_local(fr, _wpt(node(st, "neck"), Fq))
                angs.append(np.degrees(np.arctan2(A[2], A[1])))
    a0, a1 = min(angs) - 12, max(angs) + 12
    tools.append(sector_x(HUB_CMC_R + 3, -CMC_SLOT, CMC_SLOT, a0, a1))
    # nip pins (1 mm) across the slot
    nip_holes, pins = _nip_pins("cmc_flex", CMC_SLOT + 1.0)
    tools += nip_holes
    tools += [cyl_x(STUB_CMC[2] / 2 + 0.08, STUB_CMC[0], HUB_CMC_W + 0.1),
              cyl_x(STUB_CMC[2] / 2 + 0.08, -HUB_CMC_W - 0.1, -STUB_CMC[0])]
    # MCP clearance for the proximal hub and the 2 mm stub holes in the cheeks
    tools.append(_local_from(Fm, F, cyl_x(HUB_MCP_R + 0.3, -CHEEK_MCP[0], CHEEK_MCP[0])))
    tools.append(_local_from(Fm, F, cyl_x(STUB_SMALL_D / 2 + P.PRESS_FIT / 2, -CHEEK_MCP[1] - .1, CHEEK_MCP[1] + .1)))
    # strand channels (PTFE-lined 2.2 mm) from the crossing to the MCP
    segs = []
    Fq = body_transforms({})
    for st in STRANDS.values():
        if st["joint"] == "mcp_flex":
            c = node(st, "cmc_cross")["p"]
            _, Tw, *_ = drum_segment(st, c, Fq, {})
            segs.append((c + 6.0 * K.unit(Tw - c), Tw - 2.5 * K.unit(Tw - c)))
            segs.append((c, c + 6.0 * K.unit(Tw - c)))
        elif st["joint"] == "ip":
            c, m = node(st, "cmc_cross")["p"], node(st, "mcp_cross")["p"]
            segs.append((c + 6.0 * K.unit(m - c), m - (HUB_MCP_R + 1.0) * K.unit(m - c)))
            segs.append((c, c + 6.0 * K.unit(m - c)))
    bores = []
    for i, (a, b) in enumerate(segs):
        bores.append(cyl_ab(to_local(F, a), to_local(F, b), (P.SHEATH_BORE if i % 2 == 0 else 1.2) / 2))
    tools += bores
    # hard-stop lugs are on the child hubs (proximal); here: nothing else
    s = cut(s, *tools)
    s = s.fuse(_stop_lugs("cmc_flex", HUB_CMC_R, (STUB_CMC[0] + 0.5, HUB_CMC_W - 0.3), PLATE_Z[1]))
    return place(main_solid(s.clean()), F), pins


def _local_from(fr_src, fr_dst, shape):
    """Shape given in frame fr_src -> coordinates of frame fr_dst."""
    t = _trsf(fr_dst).Inverted()
    t.Multiply(_trsf(fr_src))
    return _apply(shape, t)


def _nip_pins(j, half_len):
    """Holes (child frame of j) and pin solids (world, rest) for the two nip pins of joint j."""
    st = next(s for s in STRANDS.values() if any(n.get("joint") == j and n["type"] == "nip" for n in s["nodes"]))
    g = next(n["gate"] for n in st["nodes"] if n.get("joint") == j and n["type"] == "nip")
    holes, pins = [], []
    for sgn in (+1, -1):
        y, z = sgn * NIP_OFF * g
        holes.append(cyl_x(NIP_PIN_D / 2 + P.PRESS_FIT / 2, -half_len, half_len, y, z))
        pins.append(place(cyl_x(NIP_PIN_D / 2, -half_len + 0.2, half_len - 0.2, y, z), J0[j]))
    return holes, pins


LUG_TRIM = {"cmc_flex": (-10.0, 0.0)}  # empirical (flex, ext) lug angle trims from stop_check() (deg)


def _stop_lugs(j, hub_r, xspan, face_z):
    """Two lugs on the child hub (joint frame of j) that hit the parent's end face (plane z = face_z, i.e.
    -(hub_r + gap) in the parent joint frame) at the range limits. Angles from +y toward +z."""
    lo, hi = RANGES[j]
    Rl = hub_r + 1.5
    psi = np.degrees(np.arccos(min(1.0, abs(face_z) / Rl)))  # contact half-cone about -z
    out = []
    # child rotates by +theta about +x: a point at angle phi (rest) is at phi + theta. Contact when the
    # lug's leading edge reaches -90 +/- psi.
    for lim, sgn in ((hi, +1), (lo, -1)):
        edge = (-90.0 - psi) if sgn > 0 else (-90.0 + psi)  # flex lug comes from the -z/-y side
        lead = edge - lim + LUG_TRIM.get(j, (0.0, 0.0))[0 if sgn > 0 else 1]
        a0, a1 = (lead - 8.0, lead) if sgn > 0 else (lead, lead + 8.0)
        for x0, x1 in (xspan, (-xspan[1], -xspan[0])):
            out.append(_sector_ring_x(hub_r - 0.5, Rl, x0, x1, a0, a1))
    return fuse(*out)


def _sector_ring_x(r0, r1, x0, x1, a0, a1, n=None):
    return _annulus_wp(cq.Workplane("YZ", origin=(x0, 0, 0)), r0, r1, a0, a1).extrude(x1 - x0).val()


def make_proximal():
    Fm, Fi = J0["mcp_flex"], J0["ip"]
    hub = cyl_x(HUB_MCP_R, -HUB_MCP_W, HUB_MCP_W)
    exi = np.array([Fi[1] @ Fm[1], Fi[1] @ Fm[2], Fi[1] @ Fm[3]])
    eyi = np.array([Fi[2] @ Fm[1], Fi[2] @ Fm[2], Fi[2] @ Fm[3]])
    shaft = fuse(prism(2 * HUB_MCP_W, SHAFT_PX[1], 3.5, 3.0, L2 - HUB_IP_R - 4.0),
                 _local_from(Fi, Fm, prism(2 * CHEEK_IP[1], SHAFT_PX[1], 3.5, -(L2 * 0.5), -(HUB_IP_R + 0.6))))
    s = fuse(hub, shaft)
    for sgn in (+1, -1):
        x0, x1 = (CHEEK_IP[0], CHEEK_IP[1]) if sgn > 0 else (-CHEEK_IP[1], -CHEEK_IP[0])
        ch = fuse(box(x0, x1, -SHAFT_PX[1] / 2, SHAFT_PX[1] / 2, -(HUB_IP_R + 2.0), 0.0), cyl_x(6.5, x0, x1))
        s = s.fuse(_local_from(Fi, Fm, ch))
    tools = _grooves("mcp_flex", (-MCP_GROOVE_X, MCP_GROOVE_X), HUB_MCP_R)
    for side in ("flex", "ext"):
        tools.append(_anchor_pocket("mcp_flex", STRANDS[f"mcp_flex_{side}"]))
    lo, hi = RANGES["mcp_flex"]
    tools.append(sector_x(HUB_MCP_R + 3, -MCP_SLOT, MCP_SLOT, -90 - hi - 25, -90 - lo + 25))
    holes, pins = _nip_pins("mcp_flex", MCP_SLOT + 1.2)
    tools += holes
    tools += [cyl_x(STUB_SMALL_D / 2 + 0.06, 4.5, HUB_MCP_W + .1), cyl_x(STUB_SMALL_D / 2 + 0.06, -HUB_MCP_W - .1, -4.5)]
    tools.append(_local_from(Fi, Fm, cyl_x(HUB_IP_R + 0.3, -CHEEK_IP[0], CHEEK_IP[0])))
    tools.append(_local_from(Fi, Fm, cyl_x(STUB_SMALL_D / 2 + P.PRESS_FIT / 2, -CHEEK_IP[1] - .1, CHEEK_IP[1] + .1)))
    Fq = body_transforms({})
    for st in STRANDS.values():
        if st["joint"] == "ip":
            c = node(st, "mcp_cross")["p"]
            _, Tw, *_ = drum_segment(st, c, Fq, {})
            u = K.unit(Tw - c)
            tools.append(cyl_ab(to_local(Fm, c), to_local(Fm, c + 5.5 * u), 0.6))
            tools.append(cyl_ab(to_local(Fm, c + 5.5 * u), to_local(Fm, Tw - 2.0 * u), P.SHEATH_BORE / 2))
    s = cut(s, *tools)
    s = s.fuse(_stop_lugs("mcp_flex", HUB_MCP_R, (4.6, HUB_MCP_W - 0.3), -(HUB_MCP_R + 0.6)))
    return place(main_solid(s.clean()), Fm), pins


def make_distal():
    Fi = J0["ip"]
    hub = cyl_x(HUB_IP_R, -HUB_IP_W, HUB_IP_W)
    body = loft([rrect_wire(2 * HUB_IP_W, 15.0, 3.5, 2.0), rrect_wire(15.0, 12.5, 4.0, L3 - 7.0)])
    tip = cq.Workplane("XY").add(cyl_x(6.25, -7.0, 7.0, 0, L3 - 7.0)).edges().fillet(2.5).val()
    s = fuse(hub, body, tip)
    # pad seat (palmar, -y): a flat recess 2 mm deep for the TPU pad
    pad_seat = box(-8, 8, -9, -PAD_SEAT_Y, 5.0, L3 + 1)
    tools = _grooves("ip", (-IP_GROOVE_X, IP_GROOVE_X), HUB_IP_R) + [pad_seat]
    for side in ("flex", "ext"):
        tools.append(_anchor_pocket("ip", STRANDS[f"ip_{side}"]))
    tools += [cyl_x(STUB_SMALL_D / 2 + 0.06, 3.5, HUB_IP_W + .1), cyl_x(STUB_SMALL_D / 2 + 0.06, -HUB_IP_W - .1, -3.5)]
    s = cut(s, *tools)
    s = s.fuse(_stop_lugs("ip", HUB_IP_R, (3.6, HUB_IP_W - 0.3), -(HUB_IP_R + 0.6)))
    return place(main_solid(s.clean()), Fi)


PAD_SEAT_Y = 5.5  # pad seat plane at y = -5.5 (distal frame); the pad surface is at y = -8


def make_thumb_pad():
    """TPU 95A (or Shore-A 60 silicone) pad: a broad, slightly domed pad on the distal phalanx."""
    Fi = J0["ip"]
    pad = loft([rrect_wire(15.0, 2.6, 1.0, 6.0), rrect_wire(14.0, 2.6, 1.0, L3 - 7.0)])
    pad = pad.translate(V(0, -(PAD_SEAT_Y + 1.25), 0))
    cap = cut(ring_x(6.0, 8.0, -7.0, 7.0).translate(V(0, 0, L3 - 7.0)),
              box(-10, 10, -PAD_SEAT_Y, 10, -10, 100), box(-10, 10, -10, 10, -10, L3 - 7.0))
    pad = fuse(pad, cap)
    return place(pad, Fi)


def make_thenar_pad():
    """TPU pad on the palmar (-y) face of the metacarpal shaft: the thumb's 'thenar' contact for power
    grasps (the cylinder rests on it)."""
    F = J0["cmc_flex"]
    z0, z1 = 12.0, L1 - HUB_MCP_R - 3.0
    pad = loft([rrect_wire(14.0, 3.0, 1.0, z0), rrect_wire(14.0, 3.0, 1.0, z1)])
    pad = pad.translate(V(0, -SHAFT_MC[1] / 2 - 1.5 + 0.0, 0))
    return place(pad, F)


def make_stubs():
    out = {}
    for sgn, tag in ((+1, "a"), (-1, "b")):
        x0, x1 = (STUB_CMC[0] + 0.3, CHEEK_CMC[1]) if sgn > 0 else (-CHEEK_CMC[1], -STUB_CMC[0] - 0.3)
        out[f"pin_cmc_{tag}"] = place(cyl_x(STUB_CMC[2] / 2, x0, x1), J0["cmc_flex"])
        x0, x1 = (4.8, CHEEK_MCP[1]) if sgn > 0 else (-CHEEK_MCP[1], -4.8)
        out[f"pin_mcp_{tag}"] = place(cyl_x(STUB_SMALL_D / 2, x0, x1), J0["mcp_flex"])
        x0, x1 = (3.8, CHEEK_IP[1]) if sgn > 0 else (-CHEEK_IP[1], -3.8)
        out[f"pin_ip_{tag}"] = place(cyl_x(STUB_SMALL_D / 2, x0, x1), J0["ip"])
    return out


@functools.lru_cache(maxsize=1)
def rest_parts():
    t0 = time.time()
    parts = {"thumb_mount": make_mount(), "bearing_top": make_bearing(BEAR_TOP_Z),
             "bearing_bottom": make_bearing(BEAR_BOT_Z), "base_cap": make_cap(), "neck_top": make_neck(),
             "thumb_base": make_base()}
    mc, pins_c = make_metacarpal()
    parts["thumb_metacarpal"] = mc
    parts["thenar_pad"] = make_thenar_pad()
    px, pins_m = make_proximal()
    parts["thumb_proximal"] = px
    parts["thumb_distal"] = make_distal()
    parts["thumb_pad"] = make_thumb_pad()
    for i, p in enumerate(pins_c):
        parts[f"nip_cmc_{'ab'[i]}"] = p
    for i, p in enumerate(pins_m):
        parts[f"nip_mcp_{'ab'[i]}"] = p
    parts.update(make_stubs())
    print(f"  rest parts built in {time.time() - t0:.1f} s")
    return parts


PART_BODY = {"thumb_mount": "mount", "bearing_top": "mount", "bearing_bottom": "mount",
             "base_cap": "base", "neck_top": "base", "thumb_base": "base",
             "pin_cmc_a": "base", "pin_cmc_b": "base", "thumb_metacarpal": "metacarpal", "thenar_pad": "metacarpal",
             "nip_cmc_a": "metacarpal", "nip_cmc_b": "metacarpal", "pin_mcp_a": "metacarpal", "pin_mcp_b": "metacarpal",
             "thumb_proximal": "proximal", "nip_mcp_a": "proximal", "nip_mcp_b": "proximal", "pin_ip_a": "proximal",
             "pin_ip_b": "proximal", "thumb_distal": "distal", "thumb_pad": "distal"}
MATERIAL = {"thumb_mount": "PETG", "thumb_base": "PETG", "thumb_metacarpal": "PETG", "thumb_proximal": "PETG",
            "thumb_distal": "PETG", "thumb_pad": "TPU 95A (or cast silicone A30)", "thenar_pad": "TPU 95A",
            "bearing_top": "6704ZZ 20x27x4", "bearing_bottom": "6704ZZ 20x27x4", "base_cap": "PETG, 3 x M2x6",
            "neck_top": "PTFE eyelet (6 mm rod, 1.6 mm hole, both ends rounded R0.8)",
            "pin_cmc_a": "steel dowel 3 mm", "pin_cmc_b": "steel dowel 3 mm", "nip_cmc_a": "steel pin 1 mm",
            "nip_cmc_b": "steel pin 1 mm", "nip_mcp_a": "steel pin 1 mm", "nip_mcp_b": "steel pin 1 mm",
            "pin_mcp_a": "steel dowel 2 mm", "pin_mcp_b": "steel dowel 2 mm", "pin_ip_a": "steel dowel 2 mm",
            "pin_ip_b": "steel dowel 2 mm"}


def _qdict(q):
    q = dict(q or {})
    return {n: float(q.get(n, 0.0)) for n in JOINT_NAMES}


def pose(shape, body, q):
    """Rest-pose solid -> pose q by rotating about the rest-pose axes (distal joints first)."""
    idx = BODIES.index(body)
    for k in range(idx, 0, -1):
        jn = JOINT_NAMES[k - 1]
        p, a = D.joints[k - 1]
        ang = q[jn]
        if ang:
            shape = shape.rotate(V(*p), V(*(p + a)), ang)
    return shape


def build(q: dict | None = None) -> dict:
    q = _qdict(q)
    parts = {n: pose(s, PART_BODY[n], q) for n, s in rest_parts().items()}
    Fq = body_transforms(q)
    joints = {}
    for k, jn in enumerate(JOINT_NAMES):
        p, a = D.joints[k]
        par = BODIES[k]
        Rm, t = Fq[par]
        joints[f"thumb_{jn}"] = {"point": (Rm @ p + t).tolist(), "axis": (Rm @ a).tolist(),
                                 "range": list(RANGES[jn]), "parent": par, "child": BODIES[k + 1]}
    strands = []
    for name, st in STRANDS.items():
        L, det = strand_length(st, q, detail=True)
        pts = det["points"]
        # drum arc from the tangent to the anchor (in the child's joint frame)
        dr = st["drum"]
        fr = joint_frame_q(dr["joint"], q, Fq)
        aT = np.arctan2(*(to_local(fr, pts[-1])[1:][::-1]))
        arc = [to_world(fr, [dr["groove_x"], R * np.cos(aT + dr["sense"] * s), R * np.sin(aT + dr["sense"] * s)])
               for s in np.linspace(0, np.radians(det["wrap_deg"]), 8)]
        path = list(reversed(arc)) + list(reversed(pts[:-1]))  # anchor -> exit
        strands.append({"joint": f"thumb_{st['joint']}", "role": st["role"], "points": [p.tolist() for p in path],
                        "length": float(L)})
    return {"parts": parts, "joints": joints, "strands": strands, "interface": interface()}


def interface():
    C = J0["C"]
    o, x, y, z = C
    bolts = []
    for b in range(3):
        a = np.radians(W_ANG + 180 + (b - 1) * 100)
        bolts.append({"point": to_world(C, [BOLT_R * np.cos(a), BOLT_R * np.sin(a), FLANGE_Z[0]]).tolist(),
                      "axis": z.tolist(), "size": "M3 (clearance 3.4 in the flange, heat-set insert in the palm)"})
    exits = {}
    for name, st in STRANDS.items():
        e, s = st["nodes"][0]["p"], st["nodes"][1]["p"]
        exits[f"thumb_{name}"] = {"point": e.tolist(), "direction": K.unit(e - s).tolist(), "sheathed": True,
                                  "note": "PTFE 1x2 sheath starts in this socket (2.2 bore, 6 mm deep)"}
    return {
        "mount": {"part": "thumb_mount", "bore_axis_point": o.tolist(), "bore_axis": z.tolist(),
                  "housing": {"od": MOUNT_OD, "z_local": MOUNT_Z, "frame": "C (origin P, z = cmc_rot axis)"},
                  "flange": {"od": FLANGE_OD, "z_local": FLANGE_Z}, "bolts": bolts,
                  "bearings": f"2 x {BEARING['name']} (20 x 27 x 4)"},
        "rest_pose_note": "q = 0: thumb straight (MCP, IP 0), standing out in front of the thenar in a C-ready "
                          "posture; out-of-the-way 'flat' pose = " + str(DESIGN_JSON["eval"]["flat"]["q_thumb"]),
        "cmc_centre": o.tolist(),
        "strand_exits": exits,
        "envelope_stl": str((OUT / "thumb_envelope_hull.stl").relative_to(HERE.parent)),
        "frame_C": {"origin": o.tolist(), "x": x.tolist(), "y": y.tolist(), "z": z.tolist()},
    }


# --- Checks -----------------------------------------------------------------------------------------
IGNORE = [("pin_cmc_*", "thumb_base"), ("pin_cmc_*", "thumb_metacarpal"), ("nip_cmc_*", "thumb_metacarpal"),
          ("pin_mcp_*", "thumb_metacarpal"), ("pin_mcp_*", "thumb_proximal"), ("nip_mcp_*", "thumb_proximal"),
          ("pin_ip_*", "thumb_proximal"), ("pin_ip_*", "thumb_distal"), ("bearing_*", "thumb_base"),
          ("bearing_*", "thumb_mount"), ("neck_top", "base_cap"), ("thumb_pad", "thumb_distal"),
          ("thenar_pad", "thumb_metacarpal")]


def routing_report(n=13):
    """Strand length change per strand vs each joint (others at rest, mid, and the opposite extreme)."""
    res = {}
    for name, st in STRANDS.items():
        row = {}
        for jn in JOINT_NAMES:
            worst = 0.0
            for other in ("rest", "mid", "hi"):
                base_q = {k: (0.0 if other == "rest" else mid(k) if other == "mid" else RANGES[k][1])
                          for k in JOINT_NAMES}
                Ls = []
                for v in np.linspace(*RANGES[jn], n):
                    q = dict(base_q)
                    q[jn] = v
                    Ls.append(strand_length(st, q))
                Ls = np.array(Ls)
                if jn == st["joint"]:
                    slope = np.polyfit(np.linspace(*RANGES[jn], n), Ls, 1)[0]
                    resid = Ls - np.polyval(np.polyfit(np.linspace(*RANGES[jn], n), Ls, 1), np.linspace(*RANGES[jn], n))
                    worst = max(worst, float(np.ptp(resid)))
                    row["slope_mm_per_deg"] = float(slope)
                else:
                    worst = max(worst, float(np.ptp(Ls)))
            row[jn] = worst
        res[name] = row
    # loop sums (flex + ext) per joint over its own range and over everything
    sums = {}
    for jn in JOINT_NAMES:
        f, e = STRANDS[f"{jn}_flex"], STRANDS[f"{jn}_ext"]
        vals = []
        for v in np.linspace(*RANGES[jn], n):
            for other in ("rest", "mid"):
                q = {k: (0.0 if other == "rest" else mid(k)) for k in JOINT_NAMES}
                q[jn] = v
                vals.append(strand_length(f, q) + strand_length(e, q))
        sums[jn] = float(np.ptp(vals))
    # true coupling: a change of (L_flex - L_ext)/2 turns the driven joint by that / (mm per deg)
    coup = {}
    for jn in JOINT_NAMES:
        f, e = STRANDS[f"{jn}_flex"], STRANDS[f"{jn}_ext"]
        row = {}
        for kn in JOINT_NAMES:
            if kn == jn:
                continue
            worst = 0.0
            for other in ("rest", "mid", "hi"):
                base_q = {k: (0.0 if other == "rest" else mid(k) if other == "mid" else RANGES[k][1])
                          for k in JOINT_NAMES}
                d = []
                for v in np.linspace(*RANGES[kn], n):
                    q = dict(base_q)
                    q[kn] = v
                    d.append(0.5 * (strand_length(f, q) - strand_length(e, q)))
                worst = max(worst, float(np.ptp(d)))
            row[kn] = {"mm": worst, "deg": worst / P.MM_PER_DEG}
        coup[jn] = row
    return res, sums, coup


def collision_sweep(quick=False):
    """Interference at min/mid/max combos. Returns list of (q, pairs)."""
    from tools.checks import interference
    combos = []
    lv = (0, 2) if quick else (0, 1, 2)

    def val(j, i):
        lo, hi = RANGES[j]
        return (lo, 0.5 * (lo + hi), hi)[i]
    for a in lv:
        for b in lv:
            for c in lv:
                combos.append({"cmc_rot": val("cmc_rot", a), "cmc_flex": val("cmc_flex", b),
                               "mcp_flex": val("mcp_flex", c), "ip": val("ip", c)})
    combos.append({k: 0.0 for k in JOINT_NAMES})
    out = []
    for q in combos:
        parts = build(q)["parts"]
        pairs = interference(parts, threshold=0.5, ignore=IGNORE)
        out.append((q, pairs))
        print("  q", {k: round(v) for k, v in q.items()}, "->", pairs if pairs else "clear", flush=True)
    return out


def stop_check(step=2.0, beyond=30.0):
    """First contact between each joint's child and parent bodies when driven past its limits (others 0).
    A good hard stop touches 0-6 deg past the soft limit."""
    from tools.checks import interference
    out = {}
    groups = {"cmc_rot": (["thumb_mount"], ["thumb_base", "base_cap"]),
              "cmc_flex": (["thumb_base"], ["thumb_metacarpal", "thenar_pad"]),
              "mcp_flex": (["thumb_metacarpal"], ["thumb_proximal"]),
              "ip": (["thumb_proximal"], ["thumb_distal", "thumb_pad"])}
    for j, (par, ch) in groups.items():
        res = {}
        for end, sgn in ((RANGES[j][1], +1), (RANGES[j][0], -1)):
            hit = None
            for d in np.arange(-6.0, beyond + 0.1, step):
                q = {j: end + sgn * d}
                parts = {k: v for k, v in build(q)["parts"].items() if k in par + ch}
                pairs = [p for p in interference(parts, threshold=0.05) if (p[0] in par) != (p[1] in par)]
                if pairs:
                    hit = float(d)
                    break
            res["max" if sgn > 0 else "min"] = hit
        out[j] = res
        print(f"  stop {j}: first contact {res} deg past the limit (None = no stop within {beyond})", flush=True)
    return out


def envelope(npose=4):
    """Convex hull of everything that moves (base and distal) over the joint ranges -> STL + bbox."""
    import trimesh
    from tools.render import tessellate
    pts = []
    rp = rest_parts()
    meshes = {n: tessellate(s, 0.5, 0.5)[0] for n, s in rp.items() if PART_BODY[n] != "mount"}
    for a in np.linspace(*RANGES["cmc_rot"], npose):
        for b in np.linspace(*RANGES["cmc_flex"], npose):
            for c in np.linspace(*RANGES["mcp_flex"], 3):
                q = {"cmc_rot": a, "cmc_flex": b, "mcp_flex": c, "ip": c}
                Fq = body_transforms(q)
                for n, v in meshes.items():
                    Rm, t = Fq[PART_BODY[n]]
                    pts.append(v[:: max(1, len(v) // 400)] @ Rm.T + t)
    pts = np.concatenate(pts)
    hull = trimesh.convex.convex_hull(pts)
    hull.export(OUT / "thumb_envelope_hull.stl")
    # base-only sweep (what the palm's thenar bay must clear near the mount)
    base_pts = []
    for a in np.linspace(*RANGES["cmc_rot"], 13):
        Fq = body_transforms({"cmc_rot": a})
        Rm, t = Fq["base"]
        base_pts.append(meshes["thumb_base"] @ Rm.T + t)
    bh = trimesh.convex.convex_hull(np.concatenate(base_pts))
    bh.export(OUT / "thumb_base_sweep_hull.stl")
    return hull.bounds.tolist(), bh.bounds.tolist()


# --- Renders ----------------------------------------------------------------------------------------
DESIGN_JSON = json.loads((OUT / "thumb_design.json").read_text())
COLORS = {"thumb_mount": "#5b6b7f", "thumb_base": "#c8a165", "thumb_metacarpal": "#4f8fd6", "thumb_proximal": "#4f8fd6",
          "thumb_distal": "#4f8fd6", "thumb_pad": "#e0703a", "thenar_pad": "#e0703a", "rot_journal": "#9aa3ad",
          "palm": "#d9d9d9", "strands": "#d62728", "cylinder": "#7fbf7f"}


def finger_solids(fname, qf):
    """Simple finger stand-in (capsules) from the kinematic model, plus its pad marker."""
    fi = K.FINGER_OBJS[fname]
    Fk = K.finger_fk(fi, qf)
    out = []
    for a, b, r in Fk["caps"]:
        out.append(cyl_ab(a, b, r).fuse(sphere(r, b)).fuse(sphere(r, a)))
    return fuse(*out)


def palm_solid():
    """Palm stand-in (the integrator owns the real palm): slab over the fingers' part + base."""
    s = box(K.PALM_X[0], K.PALM_X[1], P.PALM_Y[0], P.PALM_Y[1], P.PALM_Z_BASE, 90.0)
    return s


def strand_solids(q, r=0.35):
    res = build(q)
    segs = []
    for st in res["strands"]:
        pts = np.array(st["points"])
        for a, b in zip(pts[:-1], pts[1:]):
            if np.linalg.norm(b - a) > 0.05:
                segs.append(cyl_ab(a, b, r))
    return cq.Compound.makeCompound(segs)


def render_all(views_rest=("iso", "front", "side", "top")):
    from tools.render import render
    ev = DESIGN_JSON["eval"]
    q0 = {k: 0.0 for k in JOINT_NAMES}
    # 1) rest pose with palm and straight fingers
    parts = dict(build(q0)["parts"])
    parts["palm"] = palm_solid()
    for f in K.FINGER_NAMES:
        parts[f"finger_{f}"] = finger_solids(f, (0, 0, 0))
    render(parts, OUT / "thumb_rest_hand.png", views=views_rest, colors=COLORS, legend=False,
           title="V2 thumb at rest (q = 0) with palm/finger stand-ins")
    # 2) thumb alone, with strands
    parts = dict(build(q0)["parts"])
    parts["strands"] = strand_solids(q0)
    render(parts, OUT / "thumb_rest_detail.png", views=("iso", "front", "side", (20, 200)), colors=COLORS,
           legend=False, title="V2 thumb parts + strands (red), rest pose")
    # 3) poses
    keys = [("index", "index pad"), ("middle", "middle pad"), ("ring", "ring pad"), ("little", "little pad"),
            ("tip_pinch", "index tip-to-tip"), ("lateral", "key pinch"), ("power55", "power grasp 55 mm")]
    for k, lab in keys:
        e = ev[k]
        q = dict(zip(JOINT_NAMES, e["q_thumb"]))
        parts = {n: s for n, s in build(q)["parts"].items() if PART_BODY[n] != "mount" or n == "thumb_mount"}
        parts["palm"] = palm_solid()
        for f in K.FINGER_NAMES:
            qf = (0, 0, 0)
            if k in (f,) or (k in ("tip_pinch", "lateral") and f == "index"):
                qf = tuple(e["q_finger"])
            elif k.startswith("power"):
                qf = (0, 75, 80)
            parts[f"finger_{f}"] = finger_solids(f, qf)
        if k.startswith("power"):
            D_ = float(k[5:])
            g, zc, sx, phi = e["cyl"]
            o, c, ey, en = K.cylinder_frame(D_, g, zc)
            parts["cylinder"] = cyl_ab(o - 70 * c, o + 70 * c, D_ / 2)
        title = f"{lab}: gap {e['dist']:.1f} mm, pad angle {e['angle']:.0f} deg, q = " + \
                ", ".join(f"{v:.0f}" for v in e["q_thumb"])
        render(parts, OUT / f"pose_{k}.png", views=((15, 235), (60, 270), (-10, 200), "front"), colors=COLORS,
               legend=False, title=title, size=(1400, 1050))
    # 4) CMC close-up: section through the base along the rot axis
    o, x, y, z = J0["C"]
    big = 200.0
    half = place(box(-big, big, 0, big, -big, big), J0["C"])  # keep y_C < 0 half
    parts = {}
    for n, s_ in build(q0)["parts"].items():
        if PART_BODY[n] in ("mount", "base") or n in ("thumb_metacarpal", "nip_cmc_a", "nip_cmc_b", "pin_cmc_a",
                                                      "pin_cmc_b"):
            try:
                parts[n] = s_.cut(half)
            except Exception:
                parts[n] = s_
    parts["strands"] = strand_solids(q0, 0.3)
    render(parts, OUT / "cmc_section.png", views=((10, 0), (10, 90), (40, 45), (-30, 135)), colors=COLORS,
           legend=False, title="CMC section (half cut away): bearings, neck, nip, drums, strands")


def main():
    quick = "--quick" in sys.argv
    OUT.mkdir(parents=True, exist_ok=True)
    print("thumb design:", D.name, "ranges", RANGES)
    print("lengths L1 L2 L3 =", round(L1, 1), round(L2, 1), round(L3, 1))
    from tools.checks import is_valid
    from tools.export import export_all
    rp = rest_parts()
    print("validity:")
    for n, s in rp.items():
        v = is_valid(s)
        print(f"  {n:18s} ok={v['ok']} solids={v['solids']} vol={v['volume']:.0f}")
    res = build({})
    export_all(res["parts"], OUT, name="thumb_rest")
    print("routing (max length change, mm):")
    rr, sums, coup = routing_report(9 if quick else 13)
    for n, row in rr.items():
        print(f"  {n:15s} " + " ".join(f"{k}={row[k]:.3f}" for k in JOINT_NAMES) +
              f"  own slope {row.get('slope_mm_per_deg', 0):.4f} mm/deg")
    print("  loop sums ptp:", {k: round(v, 3) for k, v in sums.items()})
    for jn, row in coup.items():
        print(f"  coupling into {jn}: " + ", ".join(f"{k} {v['mm']:.3f} mm = {v['deg']:.2f} deg" for k, v in row.items()))
    (OUT / "routing.json").write_text(json.dumps({"per_strand": rr, "loop_sums": sums, "coupling": coup,
                                                  "anchors": {n: {k: v for k, v in st["drum"].items()}
                                                              for n, st in STRANDS.items()}}, indent=1))
    print("interface:")
    itf = res["interface"]
    (OUT / "interface.json").write_text(json.dumps({**itf, "joints": res["joints"],
                                                    "strands": res["strands"]}, indent=1))
    hb, bb = envelope(3 if quick else 4)
    print("  envelope hull bounds", np.round(hb, 1).tolist(), " base sweep bounds", np.round(bb, 1).tolist())
    if "--no-render" not in sys.argv:
        print("renders...")
        render_all()
    print("hard stops:")
    stops = stop_check(3.0 if quick else 2.0)
    (OUT / "stops.json").write_text(json.dumps(stops, indent=1))
    print("collisions:")
    col = collision_sweep(quick)
    (OUT / "collisions.json").write_text(json.dumps([(q, p) for q, p in col], indent=1))
    n_bad = sum(1 for _, p in col if p)
    print(f"  {n_bad} of {len(col)} poses with interference")


if __name__ == "__main__":
    main()
