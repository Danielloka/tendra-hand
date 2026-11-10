"""Thumb + finger kinematics for the Tendra V2 concept hand (study 2026-11-09, thumb designer).

Hand frame (params.py): mm, origin = wrist centre, +Z to the fingers, -Y = palm side, +X = thumb side.
Joint angles in degrees in the public API, positive = closing.

What is here
- `rot`, `Chain`: product-of-exponentials forward kinematics (each joint = an axis line at q = 0).
- `finger_chain(name)`: 4-joint finger model from params.FINGERS (abd about Y at ABD_OFFSET proximal of
  the MCP, flexion about +X, DIP = 0.75 PIP).
- `ThumbDesign`: a 4-joint thumb (cmc_rot, cmc_flex, mcp_flex, ip). `from_vector(x)` is the optimiser's
  parametrisation ("roll-pitch" CMC: cmc_flex axis perpendicular to and intersecting the cmc_rot axis,
  so all strands can come up the hollow rot axis and cross cmc_flex on its axis); `v1_baseline()` is the
  V1 thumb mapped into this frame.
- Tasks: Kapandji opposition to each finger pad, index tip-to-tip pinch, lateral (key) pinch on the
  index middle phalanx, power grasp of 40/55/70 mm cylinders, and a "flat" (thumb out of the way) pose.
  Each task is a bounded least-squares problem over the thumb's (and the finger's) joint angles with
  penetration penalties against the palm slab, the target finger and the cylinder.
- `score(design)` and `optimise()` (scipy differential evolution over the 12 design numbers).
- `strength(design, q, f_dir, point)`: joint torques per newton of pad force (J^T F) vs servo torque.

Run `python thumb_kinematics.py` (from cad/) for the baseline-vs-design table; `--optimise` re-runs DE.
"""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).resolve().parent))
import params as P  # noqa: E402

D2R = np.pi / 180.0
OUT = Path(__file__).resolve().parent.parent / "out" / "thumb"


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def rot(axis, ang):
    """Rotation matrix about a unit axis by ang (rad), Rodrigues."""
    x, y, z = axis
    c, s = np.cos(ang), np.sin(ang)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


class Chain:
    """Serial chain of revolute joints given as (point, axis) lines at q = 0 (world)."""

    def __init__(self, joints):
        self.p = [np.asarray(p, float) for p, _ in joints]
        self.a = [unit(a) for _, a in joints]

    def frames(self, q_rad):
        """List of (R, t) world transforms of body k (k = 1..n): x_world = R x0 + t."""
        R, t = np.eye(3), np.zeros(3)
        out = []
        for p, a, q in zip(self.p, self.a, q_rad):
            Ri = rot(a, q)
            ti = p - Ri @ p
            t = R @ ti + t
            R = R @ Ri
            out.append((R, t))
        return out


# --- Fingers -------------------------------------------------------------------------------------
FINGER_NAMES = ("index", "middle", "ring", "little")


@dataclass
class Finger:
    name: str
    mcp: np.ndarray
    L: tuple
    width: float
    height: float
    chain: Chain = field(repr=False, default=None)

    def pts0(self):
        m = self.mcp
        z = np.array([0, 0, 1.0])
        pip = m + self.L[0] * z
        dip = pip + self.L[1] * z
        tip = dip + self.L[2] * z
        return m, pip, dip, tip


def finger(name, abd_offset=P.ABD_OFFSET_DEFAULT):
    f = P.FINGERS[name]
    m = np.array(f["mcp"], float)
    L = f["lengths"]
    z = np.array([0, 0, 1.0])
    X, Y = np.array([1.0, 0, 0]), np.array([0, 1.0, 0])
    joints = [(m - abd_offset * z, Y), (m, X), (m + L[0] * z, X), (m + (L[0] + L[1]) * z, X)]
    fi = Finger(name, m, L, f["width"], f["height"])
    fi.chain = Chain(joints)
    return fi


FINGER_OBJS = {n: finger(n) for n in FINGER_NAMES}


def finger_fk(fi: Finger, q_deg):
    """q_deg = (abd, mcp_flex, pip). Returns dict of world points/frames."""
    abd, mcp, pip = q_deg
    q = np.array([abd, mcp, pip, P.DIP_RATIO * pip]) * D2R
    F = fi.chain.frames(q)
    m, pip0, dip0, tip0 = fi.pts0()
    Rp, tp = F[1]
    Rm, tm = F[2]
    Rd, td = F[3]
    out = {"mcp": Rp @ m + tp, "pip": Rm @ pip0 + tm, "dip": Rd @ dip0 + td, "tip": Rd @ tip0 + td}
    out["R_prox"], out["R_mid"], out["R_dist"] = Rp, Rm, Rd
    h = fi.height
    d = Rd[:, 2]
    yd = Rd[:, 1]  # dorsal direction of the distal phalanx
    pad_off = 0.5 * h - 1.0
    out["pad"] = out["dip"] + 0.55 * fi.L[2] * d - pad_off * yd
    out["pad_n"] = -yd
    out["tipc"] = out["dip"] + (fi.L[2] - 5.0) * d - (pad_off - 2.0) * yd
    out["tipc_n"] = unit(d - yd)
    # radial (+X) side of the middle phalanx, for the key pinch
    out["side"] = out["pip"] + 0.5 * fi.L[1] * Rm[:, 2] + 0.5 * fi.width * Rm[:, 0]
    out["side_n"] = Rm[:, 0]
    out["caps"] = [(out["mcp"], out["pip"], 0.5 * h), (out["pip"], out["dip"], 0.5 * h - 0.5),
                   (out["dip"], out["tip"], 0.5 * h - 1.0)]
    return out


# --- Thumb ---------------------------------------------------------------------------------------
THUMB_RADII = (9.5, 9.0, 8.5)  # capsule radii: metacarpal, proximal, distal (incl. pad)
THUMB_PAD_OFF = 8.0  # IP-axis line to the pad surface
SEARCH = {"cmc_rot": (-90.0, 90.0), "cmc_flex": (-90.0, 90.0), "mcp_flex": (-10.0, 70.0), "ip": (-15.0, 85.0)}


@dataclass
class ThumbDesign:
    name: str
    joints: list  # 4 (point, axis) at q = 0, order cmc_rot, cmc_flex, mcp_flex, ip
    tip: np.ndarray  # distal end at q = 0
    pad_n0: np.ndarray  # pad normal at q = 0
    ranges: dict  # search ranges (deg)
    cmc_hub: np.ndarray = None  # metacarpal's proximal end (for the capsule)
    x: list = None  # optimiser vector (if any)
    alpha0: float = None  # metacarpal polar angle from the rot axis at q = 0 (deg), roll-pitch only
    chain: Chain = field(repr=False, default=None)

    def __post_init__(self):
        self.chain = Chain(self.joints)
        if self.cmc_hub is None:
            self.cmc_hub = np.asarray(self.joints[1][0], float)

    def to_json(self):
        return {"name": self.name, "joints": [[list(map(float, p)), list(map(float, a))] for p, a in self.joints],
                "tip": list(map(float, self.tip)), "pad_n0": list(map(float, self.pad_n0)),
                "ranges": self.ranges, "cmc_hub": list(map(float, self.cmc_hub)), "x": self.x,
                "alpha0": self.alpha0}

    @staticmethod
    def from_json(d):
        return ThumbDesign(d["name"], [(np.array(p), np.array(a)) for p, a in d["joints"]], np.array(d["tip"]),
                           np.array(d["pad_n0"]), d["ranges"], np.array(d["cmc_hub"]), d.get("x"), d.get("alpha0"))


def thumb_fk(td: ThumbDesign, q_deg):
    q = np.asarray(q_deg, float) * D2R
    F = td.chain.frames(q)
    p = td.chain.p
    out = {"F": F}
    R1, t1 = F[0]
    R2, t2 = F[1]
    R3, t3 = F[2]
    R4, t4 = F[3]
    out["cmc"] = R2 @ td.cmc_hub + t2
    out["mcp"] = R2 @ p[2] + t2
    out["ip"] = R3 @ p[3] + t3
    out["tip"] = R4 @ td.tip + t4
    d0 = unit(td.tip - p[3])
    d = R4 @ d0
    n = R4 @ td.pad_n0
    L3 = np.linalg.norm(td.tip - p[3])
    out["d"], out["n"] = d, n
    out["pad"] = out["ip"] + 0.55 * L3 * d + THUMB_PAD_OFF * n
    out["pad_n"] = n
    out["tipc"] = out["ip"] + (L3 - 5.0) * d + (THUMB_PAD_OFF - 2.0) * n
    out["tipc_n"] = unit(d + n)
    out["axes"] = [R @ a for (R, _), a in zip([(np.eye(3), 0), F[0], F[1], F[2]], td.chain.a)]
    out["points"] = [p[0], R1 @ p[1] + t1, out["mcp"], out["ip"]]
    out["caps"] = [(out["cmc"], out["mcp"], THUMB_RADII[0]), (out["mcp"], out["ip"], THUMB_RADII[1]),
                   (out["ip"], out["tip"], THUMB_RADII[2])]
    return out


def from_vector(x, name="design"):
    """12 numbers -> roll-pitch thumb.
    x = [px, py, pz, a1_theta, a1_phi, psi, alpha0, tau3, tau4, L1, L2, L3] (mm, deg).
    a1 = cmc_rot axis (spherical angles). a2 = cmc_flex axis, perpendicular to a1 and through the same
    point (psi = its angle about a1). m0 = metacarpal = a1 turned about a2 by alpha0. MCP axis = a2 twisted
    about m0 by tau3; IP axis = MCP axis twisted by tau4. Lengths from the CMC centre (axis crossing)."""
    px, py, pz, th, ph, psi, al, t3, t4, L1, L2, L3 = x
    p = np.array([px, py, pz])
    a1 = np.array([np.sin(th * D2R) * np.cos(ph * D2R), np.sin(th * D2R) * np.sin(ph * D2R), np.cos(th * D2R)])
    ref = np.array([0, 1.0, 0]) if abs(a1[1]) < 0.9 else np.array([1.0, 0, 0])
    u = unit(np.cross(a1, ref))
    a2 = np.cos(psi * D2R) * u + np.sin(psi * D2R) * np.cross(a1, u)
    m0 = rot(a2, al * D2R) @ a1
    a3 = rot(m0, t3 * D2R) @ a2
    a4 = rot(m0, t4 * D2R) @ a3
    p3 = p + L1 * m0
    p4 = p3 + L2 * m0
    tip = p4 + L3 * m0
    n0 = unit(np.cross(a4, m0))
    return ThumbDesign(name, [(p, a1), (p, a2), (p3, a3), (p4, a4)], tip, n0, dict(SEARCH), cmc_hub=p.copy(),
                       x=[float(v) for v in x], alpha0=float(al))


def v1_baseline():
    """V1's thumb (hand_v1.json) moved into this frame so the V1 index MCP lands on params' index MCP.
    V1: rot axis along Z at the palm, cmc_flex along X 5.5 mm in front of it, MCP and IP axes at 45 deg in
    the XZ plane, thumb points straight out of the palm (-Y) at q = 0. Ranges from config_v1.h."""
    T = np.array([20 + 14.0306, -4 - 19.023, 92 - 78.9764])
    rot_p = np.array([1.0, 15.5236, 11.3764]) + T
    flex_p = np.array([1.0, 10.0236, 11.3764]) + T
    mcp_p = np.array([-0.4142, -31.9764, 9.9622]) + T
    ip_p = np.array([5.278, -66.9764, 15.6544]) + T
    d = unit(ip_p - mcp_p)
    tip = ip_p + 28.0 * d
    # signs: rot + swings the tip toward -X; flex/mcp/ip + move the tip toward +Z (toward the fingers)
    joints = [(rot_p, np.array([0, 0, -1.0])), (flex_p, np.array([1.0, 0, 0])),
              (mcp_p, unit([1, 0, 1])), (ip_p, unit([1, 0, 1]))]
    for i in (1, 2, 3):
        p, a = joints[i]
        v = np.cross(a, tip - p)
        if v[2] < 0:
            joints[i] = (p, -a)
    n0 = unit(np.cross(joints[3][1], d))
    rng = {"cmc_rot": (-100.0, 40.0), "cmc_flex": (-13.0, 45.0), "mcp_flex": (-5.0, 95.0), "ip": (-5.0, 95.0)}
    return ThumbDesign("V1 baseline", joints, tip, n0, rng, cmc_hub=flex_p + np.array([0, -4.0, 0]))


# --- Collision proxies ---------------------------------------------------------------------------
PALM_X = (-47.0, 29.0)  # finger part of the palm (the thenar x > 29 is the thumb's own bay)
PALM_Z = (P.PALM_Z_BASE, 92.0)
PALM_Y = P.PALM_Y


def _samples(a, b, n=5):
    t = np.linspace(0, 1, n)[:, None]
    return a + t * (b - a)


_T6 = np.linspace(0, 1, 6)[:, None]


def _cap_samples(caps):
    """(N,3) sample points and (N,) radii of a capsule list (6 per capsule)."""
    pts = np.concatenate([a + _T6 * (b - a) for a, b, _ in caps])
    rad = np.repeat([r for _, _, r in caps], 6)
    return pts, rad


def palm_pen(caps):
    """Penetration depths of thumb capsule samples into the palm slab (only the -Y face matters)."""
    s, r = _cap_samples(caps)
    inside = (s[:, 0] > PALM_X[0] - r) & (s[:, 0] < PALM_X[1] + 0.3 * r) & (s[:, 2] > PALM_Z[0]) & (s[:, 2] < PALM_Z[1] + r)
    return list(np.where(inside, np.maximum(0.0, s[:, 1] + r - PALM_Y[0]), 0.0))


def _pt_seg(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0, 1)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def seg_dist(p1, q1, p2, q2):
    """Min distance between segments p1q1 and p2q2 (Ericson, Real-Time Collision Detection 5.1.9)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    den = a * e - b * b
    s = float(np.clip((b * f - c * e) / den, 0, 1)) if den > 1e-9 else 0.0
    t = (b * s + f) / e
    if t < 0:
        t, s = 0.0, float(np.clip(-c / a, 0, 1))
    elif t > 1:
        t, s = 1.0, float(np.clip((b - c) / a, 0, 1))
    return float(np.linalg.norm(p1 + s * d1 - (p2 + t * d2)))


def caps_pen(caps1, caps2, skip=()):
    """Penetration of capsule set 1 (6 samples each) into capsule set 2, max per (i, j) pair."""
    p, r1 = _cap_samples(caps1)  # (6n,3)
    A = np.array([c[0] for c in caps2])
    B = np.array([c[1] for c in caps2])
    r2 = np.array([c[2] for c in caps2])
    AB = B - A  # (m,3)
    t = np.clip(np.einsum("nmk,mk->nm", p[:, None, :] - A[None], AB) / np.einsum("mk,mk->m", AB, AB), 0, 1)
    d = np.linalg.norm(p[:, None, :] - (A[None] + t[..., None] * AB[None]), axis=2)
    pen = np.maximum(0.0, r1[:, None] + r2[None] - 1.0 - d).reshape(len(caps1), 6, len(caps2)).max(1)
    for i, j in skip:
        pen[i, j] = 0.0
    return list(pen.ravel())


# --- Tasks ---------------------------------------------------------------------------------------
KN = 12.0  # mm of residual per unit of |n_t + n_f| (about 2.1 mm per 10 deg)
W_PEN = 4.0


def _angle(n1, n2):
    """Angle (deg) between n1 and -n2 (0 = pads exactly facing)."""
    return float(np.degrees(np.arccos(np.clip(-(n1 @ n2), -1, 1))))


def _bounds(td, extra):
    r = td.ranges
    lo = [r["cmc_rot"][0], r["cmc_flex"][0], r["mcp_flex"][0], r["ip"][0]]
    hi = [r["cmc_rot"][1], r["cmc_flex"][1], r["mcp_flex"][1], r["ip"][1]]
    if td.alpha0 is not None:  # roll-pitch: keep the metacarpal within 95 deg of the rot axis (routing)
        lo[1] = max(lo[1], -ALPHA_MAX - td.alpha0)
        hi[1] = min(hi[1], ALPHA_MAX - td.alpha0)
    for a, b in extra:
        lo.append(a)
        hi.append(b)
    return np.array(lo, float), np.array(hi, float)


def _solve(fun, lo, hi, starts, nfev=120):
    best = None
    for x0 in starts:
        x0 = np.clip(x0, lo + 1e-6, hi - 1e-6)
        try:
            r = least_squares(fun, x0, bounds=(lo, hi), max_nfev=nfev, diff_step=1e-3, x_scale=10.0)
        except ValueError:
            continue
        if best is None or r.cost < best.cost:
            best = r
    return best


def _thumb_starts(lo, hi, n=4, extra=None, rng=None):
    rng = rng or np.random.default_rng(1)
    pts = [0.5 * (lo + hi)]
    for _ in range(n - 1):
        pts.append(lo + rng.random(len(lo)) * (hi - lo))
    return pts


# Kapandji-style finger posture family: the finger curls toward the thumb (PIP and DIP flexed), as a
# human does in the opposition test. (Without this the optimiser "cheats" with MCP 90 / PIP straight.)
OPP_FINGER = ((-5.0, 15.0), (20.0, 75.0), (25.0, 85.0))


def task_oppose(td, fname, mode="pad"):
    """Thumb pad (or tip) to a finger pad (or tip). Returns dict(dist, angle, q_thumb, q_finger, pen)."""
    fi = FINGER_OBJS[fname]
    lo, hi = _bounds(td, list(OPP_FINGER))
    kp, kn = ("pad", "pad_n") if mode == "pad" else ("tipc", "tipc_n")

    def res(x):
        T = thumb_fk(td, x[:4])
        Fk = finger_fk(fi, x[4:])
        r = list(T[kp] - Fk[kp]) + list(KN * (T[kn] + Fk[kn]))
        pen = palm_pen(T["caps"]) + caps_pen(T["caps"], Fk["caps"], skip={(2, 2)})
        return np.array(r + [W_PEN * p for p in pen])

    best = _solve(res, lo, hi, _thumb_starts(lo, hi, 4))
    return _summ(td, best, res, kp, kn, lambda x: finger_fk(fi, x[4:]))


def task_lateral(td):
    """Key pinch: thumb pad on the radial side of the curled index's middle phalanx."""
    fi = FINGER_OBJS["index"]
    lo, hi = _bounds(td, [(-15, 15), (30, 90), (40, 95)])

    def res(x):
        T = thumb_fk(td, x[:4])
        Fk = finger_fk(fi, x[4:])
        r = list(T["pad"] - Fk["side"]) + list(KN * (T["pad_n"] + Fk["side_n"]))
        pen = palm_pen(T["caps"]) + caps_pen(T["caps"], Fk["caps"], skip={(2, 1)})
        return np.array(r + [W_PEN * p for p in pen])

    best = _solve(res, lo, hi, _thumb_starts(lo, hi, 4))
    return _summ(td, best, res, "pad", "pad_n", lambda x: finger_fk(fi, x[4:]), side=True)


def _summ(td, best, res, kp, kn, ffk, side=False):
    x = best.x
    T = thumb_fk(td, x[:4])
    Fk = ffk(x)
    fk_p, fk_n = ("side", "side_n") if side else (kp, kn)
    return {"dist": float(np.linalg.norm(T[kp] - Fk[fk_p])), "angle": _angle(T[kn], Fk[fk_n]),
            "q_thumb": [float(v) for v in x[:4]], "q_finger": [float(v) for v in x[4:]],
            "pen": float(max([0.0] + [v for v in res(x)[6:] / W_PEN])), "cost": float(best.cost)}


def cylinder_frame(D, gamma, zc):
    c = np.array([np.cos(gamma * D2R), 0.0, np.sin(gamma * D2R)])
    o = np.array([0.0, PALM_Y[0] - 2.0 - D / 2, zc])
    ey = np.array([0, 1.0, 0])  # from the axis toward the palm
    en = np.cross(c, ey)
    return o, c, ey, en


def task_power(td, D, phi_min=150.0):
    """Power grasp of a cylinder of diameter D lying on the palm, axis oblique (gamma from +X toward +Z).
    The thumb pad must touch the cylinder's far side (azimuth >= phi_min from the palm contact) with its
    normal toward the axis, no part of the thumb inside the cylinder or the palm."""
    R = D / 2
    lo, hi = _bounds(td, [(15, 50), (45, 85), (-30, 70), (phi_min, 230)])

    def res(x):
        T = thumb_fk(td, x[:4])
        g, zc, s, phi = x[4:]
        o, c, ey, en = cylinder_frame(D, g, zc)
        radial = np.cos(phi * D2R) * ey + np.sin(phi * D2R) * en
        target = o + s * c + R * radial
        r = list(T["pad"] - target) + list(KN * (T["pad_n"] + radial))
        pen = palm_pen(T["caps"])
        pts, rr = _cap_samples(T["caps"])
        v = pts - o
        dax = np.linalg.norm(v - np.outer(v @ c, c), axis=1)
        pen += list(np.maximum(0.0, R + rr - 1.0 - dax))
        # the thumb closes over the fingers: contact across the index/middle region (x -5..30)
        pen += [max(0.0, -5.0 - target[0]), max(0.0, target[0] - 30.0)]
        return np.array(r + [W_PEN * p for p in pen])

    best = _solve(res, lo, hi, _thumb_starts(lo, hi, 4))
    x = best.x
    T = thumb_fk(td, x[:4])
    g, zc, s, phi = x[4:]
    o, c, ey, en = cylinder_frame(D, g, zc)
    radial = np.cos(phi * D2R) * ey + np.sin(phi * D2R) * en
    target = o + s * c + R * radial
    return {"dist": float(np.linalg.norm(T["pad"] - target)), "angle": _angle(T["pad_n"], radial),
            "q_thumb": [float(v) for v in x[:4]], "cyl": [float(v) for v in x[4:]],
            "pen": float(max([0.0] + list(res(x)[6:] / W_PEN))), "cost": float(best.cost)}


def task_flat(td):
    """Thumb out of the way: near the palm plane, swung radially (tip x >= 62), no palm contact."""
    lo, hi = _bounds(td, [])

    def res(x):
        T = thumb_fk(td, x)
        tip = T["tip"]
        r = [0.6 * max(0.0, 62.0 - tip[0]), 0.6 * max(0.0, -30.0 - tip[1]), 0.6 * max(0.0, tip[1] - 5.0)]
        return np.array(r + [W_PEN * p for p in palm_pen(T["caps"])])

    best = _solve(res, lo, hi, _thumb_starts(lo, hi, 4))
    T = thumb_fk(td, best.x)
    return {"q_thumb": [float(v) for v in best.x], "tip": [float(v) for v in T["tip"]], "cost": float(best.cost)}


def evaluate(td, power=(40.0, 55.0, 70.0)):
    out = {f: task_oppose(td, f) for f in FINGER_NAMES}
    out["tip_pinch"] = task_oppose(td, "index", mode="tip")
    out["lateral"] = task_lateral(td)
    for D in power:
        out[f"power{int(D)}"] = task_power(td, D)
    out["flat"] = task_flat(td)
    return out


MOUNT_DEPTH = 36.0  # CMC centre to the bottom of the thumb mount along -a1 (thumb.py's compact stack)
MOUNT_MIN_Z = 18.0  # stay inside the palm (palm base at z = 15) so the wrist gap stays free


def mount_point(td):
    p, a1 = td.joints[0]
    return p - MOUNT_DEPTH * a1


def mount_penalty(td):
    b = mount_point(td)
    p = td.joints[0][0]
    return 3.0 * (max(0.0, MOUNT_MIN_Z - b[2]) + max(0.0, b[1] - 10.0) + max(0.0, -24.0 - b[1])
                  + max(0.0, MOUNT_MIN_Z + 6.0 - p[2]))


def score(td, ev=None):
    ev = ev or evaluate(td)
    s = 0.0
    for f in FINGER_NAMES:
        s += ev[f]["dist"] + 0.2 * ev[f]["angle"] + 5 * ev[f]["pen"]
    for k in ("tip_pinch", "lateral"):
        s += 1.5 * (ev[k]["dist"] + 0.15 * ev[k]["angle"] + 5 * ev[k]["pen"])
    for k in [k for k in ev if k.startswith("power")]:
        s += ev[k]["dist"] + 0.1 * ev[k]["angle"] + 5 * ev[k]["pen"]
    s += 3 * np.sqrt(ev["flat"]["cost"])
    s += mount_penalty(td)
    # CMC motion span used by all tasks (twist in the journal, wrap at the cmc_flex crossing)
    qs = np.array([ev[k]["q_thumb"] for k in ev])
    span = qs.max(0) - qs.min(0)
    s += 0.04 * (span[0] + span[1])
    return float(s), ev


# --- Optimiser -------------------------------------------------------------------------------------
X_BOUNDS = [(18, 42), (-26, 0), (15, 62),  # CMC point (THUMB_CMC_REGION widened in z: the mount must fit)
            (0, 180), (-180, 180),  # a1 spherical
            (-180, 180),  # psi
            (-80, 80),  # alpha0
            (-60, 60), (-20, 20),  # tau3, tau4
            (40, 54), (30, 38), (26, 32)]  # L1 (CMC centre to MCP), L2, L3


def _obj(x):
    try:
        td = from_vector(x)
        return score(td)[0]
    except Exception:
        return 1e3


def optimise(maxiter=25, popsize=8, seed=3, x0=None):
    from scipy.optimize import differential_evolution
    def cb(intermediate_result):
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / "thumb_opt_x_progress.json").write_text(json.dumps(
            {"fun": float(intermediate_result.fun), "x": [float(v) for v in intermediate_result.x]}))
        print("gen best", intermediate_result.fun, flush=True)

    r = differential_evolution(_obj, X_BOUNDS, maxiter=maxiter, popsize=popsize, seed=seed, workers=4,
                               updating="deferred", polish=False, tol=1e-4, x0=x0, disp=False, callback=cb)
    return r


def refine(x0, iters=150):
    from scipy.optimize import minimize
    lo = np.array([b[0] for b in X_BOUNDS])
    hi = np.array([b[1] for b in X_BOUNDS])
    r = minimize(lambda x: _obj(np.clip(x, lo, hi)), x0, method="Nelder-Mead",
                 options={"maxfev": iters, "xatol": 0.2, "fatol": 0.05, "initial_simplex": None})
    return np.clip(r.x, lo, hi), r.fun


# --- Strength ----------------------------------------------------------------------------------------
SERVOS = {"SCS0009": 0.226, "HLS3606M": 0.59}
ETA = 0.9


def joint_torque_capacity(servo):
    """Joint torque (N m) from one servo through 5 mm spool -> 7 mm drum, 90 % efficiency."""
    return SERVOS[servo] * P.SERVO_PER_JOINT * ETA


def strength(td, q_deg, point, f_dir):
    """Joint torques (N m) per 1 N applied BY the thumb at `point` along `f_dir` (unit). The thumb must hold
    tau_i = (a_i x (point - p_i)) . f  (J^T F). Returns (tau per N [4], max force per servo option)."""
    T = thumb_fk(td, q_deg)
    taus = []
    for p, a in zip(T["points"], T["axes"]):
        taus.append(float(np.cross(a, point - p) @ f_dir) * 1e-3)  # mm -> m
    taus = np.array(taus)
    caps = {s: float(joint_torque_capacity(s) / np.max(np.abs(taus))) for s in SERVOS}
    return taus, caps


# --- Reporting -----------------------------------------------------------------------------------------
def table(evs):
    names = list(evs)
    rows = [("index pad", "index"), ("middle pad", "middle"), ("ring pad", "ring"), ("little pad", "little"),
            ("index tip-tip", "tip_pinch"), ("key pinch", "lateral"), ("power 40", "power40"),
            ("power 55", "power55"), ("power 70", "power70")]
    s = "| task | " + " | ".join(f"{n}: gap mm / pad angle deg / max pen mm" for n in names) + " |\n"
    s += "|---" * (len(names) + 1) + "|\n"
    for lab, k in rows:
        s += f"| {lab} | " + " | ".join(
            f"{evs[n][k]['dist']:.1f} / {evs[n][k]['angle']:.0f} / {evs[n][k]['pen']:.1f}" for n in names) + " |\n"
    return s


def load_design(path=OUT / "thumb_design.json"):
    return ThumbDesign.from_json(json.loads(Path(path).read_text()))


def rebase(td, q_rest, flips, ranges, name):
    """New design whose q = 0 is the old q_rest, with axis signs flipped where flips[i] (so + = closing)."""
    F = td.chain.frames(np.asarray(q_rest, float) * D2R)
    joints = [td.joints[0]]
    for k in (1, 2, 3):
        Rm, t = F[k - 1]
        p, a = td.joints[k]
        joints.append((Rm @ p + t, Rm @ a))
    joints = [(p, -a if f else a) for (p, a), f in zip(joints, flips)]
    R4, t4 = F[3]
    R2, t2 = F[1]
    return ThumbDesign(name, joints, R4 @ td.tip + t4, R4 @ td.pad_n0, ranges, R2 @ td.cmc_hub + t2, td.x, None)


def finalize(td, ev, margin=8.0):
    """Rest pose = the 'flat' (thumb out of the way) solution for the CMC, MCP/IP straight. CMC ranges =
    span used by all tasks + margin; sign chosen so that most task angles are positive (closing)."""
    keys = [k for k in ev if k != "flat"]
    qs = np.array([ev[k]["q_thumb"] for k in keys])
    qf = np.array(ev["flat"]["q_thumb"])
    q_rest = np.array([qf[0], qf[1], 0.0, 0.0])
    flips, ranges = [], {}
    for i, jn in enumerate(("cmc_rot", "cmc_flex")):
        rel = qs[:, i] - q_rest[i]
        flip = bool(np.median(rel) < 0)
        rel = -rel if flip else rel
        lo, hi = min(0.0, rel.min()) - margin, max(0.0, rel.max()) + margin
        ranges[jn] = (float(np.floor(lo)), float(np.ceil(hi)))
        flips.append(flip)
    flips += [False, False]
    ranges["mcp_flex"] = SEARCH["mcp_flex"]
    ranges["ip"] = SEARCH["ip"]
    # respect the routing limit on the metacarpal's angle to the rot axis (+-95 deg)
    nd = rebase(td, q_rest, flips, ranges, "V2 design")
    a1 = nd.joints[0][1]
    lo, hi = ranges["cmc_flex"]
    good = []
    for v in np.arange(lo, hi + 0.1, 1.0):
        T = thumb_fk(nd, [0, v, 0, 0])
        al = np.degrees(np.arccos(np.clip(unit(T["mcp"] - T["cmc"]) @ a1, -1, 1)))
        if al <= ALPHA_MAX:
            good.append(v)
    nd.ranges = dict(nd.ranges)
    nd.ranges["cmc_flex"] = (float(min(good)), float(max(good)))
    return nd, q_rest, flips


# Metacarpal angle to the rot axis allowed by the CAD (thumb.py collision sweep: the thenar pad reaches the
# mount at ~90 deg) and by the routing (wrap on the cmc_flex nip pins).
ALPHA_MAX = 86.0


def strength_table(td, ev):
    rows = {}
    for k in ("index", "middle", "little", "tip_pinch", "lateral", "power55"):
        q = ev[k]["q_thumb"]
        T = thumb_fk(td, q)
        pk, nk = ("tipc", "tipc_n") if k == "tip_pinch" else ("pad", "pad_n")
        taus, caps = strength(td, q, T[pk], T[nk])
        rows[k] = {"tau_per_N_Nm": taus.tolist(), "F_max_SCS_N": caps["SCS0009"],
                   "limiting_joint": ["cmc_rot", "cmc_flex", "mcp_flex", "ip"][int(np.argmax(np.abs(taus)))],
                   "F_max_HLS_all_N": caps["HLS3606M"]}
        # mixed: HLS on cmc_rot + cmc_flex, SCS on mcp/ip
        cap = np.array([joint_torque_capacity("HLS3606M")] * 2 + [joint_torque_capacity("SCS0009")] * 2)
        rows[k]["F_max_mixed_N"] = float(np.min(cap / np.maximum(np.abs(taus), 1e-9)))
        cap2 = np.array([joint_torque_capacity("HLS3606M")] * 3 + [joint_torque_capacity("SCS0009")])
        rows[k]["F_max_mixed3_N"] = float(np.min(cap2 / np.maximum(np.abs(taus), 1e-9)))
    return rows


if __name__ == "__main__":
    import time
    OUT.mkdir(parents=True, exist_ok=True)
    src = sys.argv[sys.argv.index("--x") + 1] if "--x" in sys.argv else str(OUT / "thumb_opt_x.json")
    xd = json.loads(Path(src).read_text())
    x = xd["x"] if isinstance(xd, dict) else xd
    base = v1_baseline()
    t0 = time.time()
    sb, eb = score(base)
    raw = from_vector(x, "V2 raw")
    sr, er = score(raw)
    nd, q_rest, flips = finalize(raw, er)
    sd, ed = score(nd)
    print(f"baseline score {sb:.1f}, raw {sr:.1f}, final (re-solved within final ranges) {sd:.1f}"
          f"  ({time.time() - t0:.0f} s)")
    print("q_rest (raw coords)", np.round(q_rest, 1), "flips", flips, "ranges", nd.ranges)
    print(table({"V1 baseline": eb, "V2 design": ed}))
    for k, v in ed.items():
        print(" ", k, {kk: (np.round(vv, 1).tolist() if isinstance(vv, list) else round(vv, 2)) for kk, vv in v.items()})
    st_b, st_d = strength_table(base, eb), strength_table(nd, ed)
    for k in st_d:
        print(f"  strength {k}: V2 tau/N {np.round(st_d[k]['tau_per_N_Nm'], 4)} F_SCS {st_d[k]['F_max_SCS_N']:.1f} N"
              f" (limit {st_d[k]['limiting_joint']}), mixed {st_d[k]['F_max_mixed_N']:.1f}, all HLS "
              f"{st_d[k]['F_max_HLS_all_N']:.1f} | V1 F_SCS {st_b[k]['F_max_SCS_N']:.1f}")
    out = nd.to_json()
    out.update({"q_rest_raw": q_rest.tolist(), "flips": flips, "score": sd, "baseline_score": sb,
                "eval": ed, "eval_baseline": eb, "strength": st_d, "strength_baseline": st_b, "x_source": src})
    (OUT / "thumb_design.json").write_text(json.dumps(out, indent=1))
    print("wrote", OUT / "thumb_design.json")
