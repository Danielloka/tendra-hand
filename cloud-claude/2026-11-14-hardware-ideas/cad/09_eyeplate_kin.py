"""09 · Bare-strand wrist crossing: eye plate on the cardan middle ring (study 2026-11-14, agent 09).

Question: can all 32 strands cross the V2 wrist bare (no sheaths) through an eye plate in the plane of the
two wrist axes, with the length change cancelled by the servos?

Model (V2 frame, mm: origin = wrist centre, +Z to the fingers, -Y palm side, +X thumb side):
- Chain as built in V2 (`wrist_kin.py`): forearm -> wrist_dev (about Y, outer) -> ring -> wrist_flex (about X,
  inner) -> palm. R_ring = Ry(dev), R_palm = Ry(dev) @ Rx(flex). Ranges as built: flex -50..+50, dev -30..+20.
- Each strand: palm anchor A (finger: V2 knuckle entry `start` from routes.json; thumb: a palm guide pin G_t) ->
  palm comb hole C (palm frame, on the straight line eye -> A at q = 0) -> eye E (ring frame, plate z = 0) ->
  forearm comb hole F (forearm frame, on the straight line spool tangent -> eye at q = 0) -> spool tangent S
  (V2 forearm interface). Everything between A and the finger, and between S and the spool, is rigid.
- Fast model: point guides. Exact model: every hole is a stack of discs (the inflated hole surface: torus lip,
  tube bore + flare, comb liner) and the strand is the shortest path through them (frictionless, taut line):
  this captures where the line really pivots (mouths, lips, clearance) - the effect that sets the pretension.
- Pairing: flex and ext strand of one joint use point-symmetric eyes (e, -e), so the first-order length
  changes are equal and opposite (= what one spool turn does) and the loop length stays nearly constant.

Outputs: out/09/eyeplate_kin.json (eye table, firmware coefficients, checks), out/09/eyeplate_kin.txt,
out/09/eyeplate_kin.png (plots).

    uv run --no-project --with numpy --with scipy --with matplotlib python cad/09_eyeplate_kin.py [--quick]
"""

from __future__ import annotations

import json
import math
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
import numpy as np
from scipy.optimize import linear_sum_assignment, minimize

HERE = Path(__file__).resolve().parent
STUDY = HERE.parent
V2 = STUDY.parent / "2026-11-09-v2-hand-cad"
OUT = STUDY / "out" / "09"
QUICK = "--quick" in sys.argv
DEG = math.pi / 180

# ------------------------------------------------------------------------------------------------
# Assumptions (all in one place)
# ------------------------------------------------------------------------------------------------
FLEX_RANGE = (-50.0, 50.0)  # as built in V2 (report 03 §3; hard stops +-52)
DEV_RANGE = (-30.0, 20.0)
N_GRID = 9  # 9 x 9 wrist grid (task)
LINE_D = 0.47  # UHMWPE PE#8 (report 02)
LINE_EA = 11.1e3  # N, secant EA = F_break / eps_break (report 02 §1); soft braid at low load ~0.6 x
EA_SOFT = 0.6
T_STALL = {"SCS0009": 45.2, "HLS3606M": 118.0}  # N strand tension at stall through a r 5 spool (report 02)
R_SPOOL = 5.0  # all servos (V2.1 baseline; report 01 proposes r 3 on the MCP: see travel table)
R_DRUM = 7.0
MU = 0.10  # braid on polished steel (report 01/02); 0.15 pessimistic
MU_HI = 0.15
ETA_ROLLER = 0.99  # per ball-bearing idler (report 01)
SERVO_TRAVEL = 300.0  # SCS0009 position range, deg (verify datasheet)
TRAVEL_MARGIN = 10.0  # deg kept free for calibration / zero offset
H_PALM = 22.0  # palm eye plate z (palm frame), all strands (just above the palm base face z 15)
H_FORE = -30.0  # forearm eye plate z (forearm frame)
# Layout "axial": palm plate, wrist eye plate and forearm plate carry the SAME hole pattern (drill all three as
# one stack), so across the wrist the 32 strands run parallel (they cannot cross); the fan-out to the knuckles
# and to the spools happens beyond the outer plates, inside rigid parts. Layout "straight" (eye -> knuckle and
# spool -> eye in straight lines) was tried first: point-symmetric pairing then makes strands intersect (skew 0).
LAYOUT = "axial"
T0 = 5.0  # loop pretension, N (report 01 bench test 5 +- 3 N)
PRETENSION_TOL = 3.0  # N allowed change over the wrist range

# Servo travel each joint needs for its own motion (V2.1 baseline: r_drum 7 / r_spool 5 = 1.4; pip + 0.60 q_mcp
# for the coaxial MCP idler, report 02 G12; thumb cmc_rot 141 deg range). sid -> (joint, deg, servo)
JOINTS = {
    1: ("index_pip", 200.0, "SCS0009"), 2: ("index_mcp_flex", 140.0, "SCS0009"), 3: ("index_mcp_abd", 42.0, "SCS0009"),
    4: ("thumb_ip", 140.0, "SCS0009"), 5: ("thumb_mcp_flex", 112.0, "SCS0009"), 6: ("thumb_cmc_flex", 140.0, "SCS0009"),
    7: ("thumb_cmc_rot", 197.4, "SCS0009"), 8: ("middle_pip", 200.0, "SCS0009"), 9: ("middle_mcp_flex", 140.0, "SCS0009"),
    10: ("middle_mcp_abd", 42.0, "SCS0009"), 11: ("ring_pip", 200.0, "SCS0009"), 12: ("ring_mcp_flex", 140.0, "SCS0009"),
    13: ("ring_mcp_abd", 42.0, "SCS0009"), 14: ("little_pip", 200.0, "SCS0009"), 15: ("little_mcp_flex", 140.0, "SCS0009"),
    16: ("little_mcp_abd", 42.0, "SCS0009"),
}  # fmt: skip
# Round 2 (report 13): own travel from the shared table out/12/travel_table.json, its 15 deg margin, and the palm
# plate raised to z +30 so that its metal seat stays out of the V2 wrist heel keep-out (z < 22).
ROUND2 = "--round2" in sys.argv
if ROUND2:
    OUT = STUDY / "out" / "13" / "kin"
    _tt = json.load(open(STUDY / "out" / "12" / "travel_table.json"))
    TRAVEL_MARGIN = float(_tt["servo"]["SCS0009"]["calibration_margin_deg"])
    for _sid, (_j, _n, _sv) in list(JOINTS.items()):
        if _j in _tt["fingers"]:
            JOINTS[_sid] = (_j, float(_tt["fingers"][_j]["servo_deg_without_wrist"]), _sv)
        elif _j in _tt.get("thumb", {}):
            JOINTS[_sid] = (_j, float(_tt["thumb"][_j]["servo_deg_without_wrist"]), _sv)
    H_PALM = 30.0
    R_DRUM = float(_tt["drum_R_mm"])
# finger-internal line length from the palm anchor to the drum (for the loop stiffness only), mm
FINGER_LEN = {"pip": 45.0, "mcp_flex": 15.0, "mcp_abd": 8.0, "thumb_ip": 90.0, "thumb_mcp_flex": 60.0,
              "thumb_cmc_flex": 25.0, "thumb_cmc_rot": 20.0}
# Thumb strands leave the thumb mount sleeve sideways (V2 start_dir ~ -X); they need a palm guide pin G_t
# inside the flange opening cone before they can head for the wrist. G_t = (x, clip(y), z):
THUMB_GUIDE = (9.0, (-7.0, 7.0), 26.0)  # 8 guide holes on a 2 mm pitch along y, ordered by the entry y

# Eye / guide geometry options. Hole profile = list of (h, rho): h along the hole axis, rho = largest radius
# the LINE CENTRE may take at that height (surface radius minus d/2). See profile_* below.
EYE_OPTIONS = {
    "torus_r0.6": dict(kind="torus", r_throat=0.42, r_lip=0.6, pitch=2.25,
                       desc="drilled + fully rounded plate eye, 1.2 mm stainless/brass, throat 0.84, lip r 0.6"),
    "torus_r0.6_t0.65": dict(kind="torus", r_throat=0.325, r_lip=0.6, pitch=2.15,
                             desc="as torus_r0.6 but throat Ø0.65 (play 0.09 per side instead of 0.185)"),
    "torus_r0.6_t0.65_7x5": dict(kind="torus", r_throat=0.325, r_lip=0.6, pitch=2.15, nx=7, ny=5,
                                 desc="chosen eye on a 7 x 5 grid minus the centre (34 eyes, 32 used): outer columns at +-6.45"),
    "torus_r0.4": dict(kind="torus", r_throat=0.42, r_lip=0.4, pitch=1.85,
                       desc="as above, 0.8 mm plate, lip r 0.4"),
    "torus_r0.8": dict(kind="torus", r_throat=0.42, r_lip=0.8, pitch=2.65,
                       desc="as above, 1.6 mm plate, lip r 0.8"),
    "tube18G_2.5": dict(kind="tube", r_bore=0.42, length=2.5, r_flare=0.4, pitch=1.6,
                        desc="18G stainless tube eye 2.5 long, flared mouths r 0.4 (report 01 spec)"),
}
COMB_LINER = dict(kind="tube", r_bore=0.595, length=2.0, r_flare=0.3)  # 16G stainless liner in an aimed hole
DOWEL_FAIRLEAD = dict(pitch=3.6, pivot_z=2.0, r_c=1.5 + LINE_D / 2)  # crossed Ø3 dowels (approximate only)


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def R_ring(f, d):
    return Ry(d * DEG)


def R_palm(f, d):
    return Ry(d * DEG) @ Rx(f * DEG)


def wrist_grid(n=N_GRID):
    fl = np.linspace(*FLEX_RANGE, n)
    dv = np.linspace(*DEV_RANGE, n)
    return [(float(f), float(d)) for f in fl for d in dv]


# ------------------------------------------------------------------------------------------------
# Strands from the V2 study
# ------------------------------------------------------------------------------------------------
def load_strands():
    routes = json.load(open(V2 / "out/assembly/routes.json"))["strands"]
    fore = json.load(open(V2 / "out/wrist/forearm_summary.json"))["interface"]["servos"]
    fan = {s["name"]: s for s in json.load(open(V2 / "out/assembly/fan_routes.json"))["strands"]}
    out = []
    for s in routes:
        sid = s["sid"]
        j = JOINTS[sid][0]
        role = s["role"]
        K = np.array(s["start"], float)
        Kdir = np.array(s["start_dir"], float)  # direction the strand leaves the source toward the wrist
        thumb = j.startswith("thumb")
        if thumb:
            A = None  # set below (thumb guide row)
        else:
            A = K.copy()
        sv = fore[str(sid)]
        S = np.array(sv[role]["tangent"], float)
        jt = j.split("_", 1)[1] if not thumb else j
        out.append(dict(name=s["name"], sid=sid, joint=j, role=role, K=K, Kdir=Kdir, A=A, S=S,
                        spool_c=np.array(sv["spool_centre"], float), spool_axis=np.array(sv["spool_axis"], float),
                        thumb=thumb, v2_palm_turn=float(s["turn_deg"]),
                        v2_fan_turn=turning(fan[s["name"]]["points"]) if s["name"] in fan else 0.0,
                        v2_len=float(s["length"]) + (fan[s["name"]]["length"] if s["name"] in fan else 0.0) + 75.0,
                        finger_len=FINGER_LEN[jt if not thumb else j]))
    th = sorted([s for s in out if s["thumb"]], key=lambda s: (s["K"][1], s["K"][0]))
    gx, (y0, y1), gz = THUMB_GUIDE
    for k, s in enumerate(th):
        s["A"] = s["K"].copy() if LAYOUT == "axial" else np.array([gx, y0 + k * (y1 - y0) / (len(th) - 1), gz])
    return out


def turning(points):
    v = np.diff(np.asarray(points, float), axis=0)
    v = v[np.linalg.norm(v, axis=1) > 1e-9]
    v /= np.linalg.norm(v, axis=1)[:, None]
    return float(np.degrees(np.arccos(np.clip((v[:-1] * v[1:]).sum(1), -1, 1))).sum())


def ang(u, v):
    u = u / np.linalg.norm(u)
    v = v / np.linalg.norm(v)
    return math.degrees(math.acos(float(np.clip(u @ v, -1, 1))))


# ------------------------------------------------------------------------------------------------
# Guide points for a given eye (q = 0 geometry)
# ------------------------------------------------------------------------------------------------
def guide_points(st, eye_xy):
    """Comb holes on the straight q = 0 lines: palm comb C on eye->A, forearm comb F on S->eye."""
    E = np.array([eye_xy[0], eye_xy[1], 0.0])
    A, S = st["A"], st["S"]
    if LAYOUT == "axial":
        return E, E + np.array([0, 0, H_PALM]), E + np.array([0, 0, H_FORE])
    if st["thumb"]:  # thumb: the guide pin G_t itself is the palm-side pivot (no comb)
        C = A.copy()
    else:
        t = H_PALM / A[2]
        C = E + t * (A - E)
    t = H_FORE / S[2]
    F = E + t * (S - E)
    return E, C, F


# ------------------------------------------------------------------------------------------------
# Fast model: point guides (vectorised over strands and poses)
# ------------------------------------------------------------------------------------------------
def fast_lengths(E, C, F, poses):
    """E, C, F: (n, 3) in ring / palm / forearm frame. Returns L (n_pose, n) of F->E->C (+ pivot geometry)."""
    out = np.zeros((len(poses), len(E)))
    for k, (f, d) in enumerate(poses):
        Rr, Rp = R_ring(f, d), R_palm(f, d)
        Ew = E @ Rr.T
        Cw = C @ Rp.T
        out[k] = np.linalg.norm(Ew - F, axis=1) + np.linalg.norm(Cw - Ew, axis=1)
    return out


def fast_polylines(strands, eyes, pose):
    """World polylines S, F, E, C, A (+ K for the thumb) for every strand at one pose: list of (m, 3)."""
    f, d = pose
    Rr, Rp = R_ring(f, d), R_palm(f, d)
    polys = []
    for st, e in zip(strands, eyes):
        E, C, F = guide_points(st, e)
        pts = [st["S"], F, Rr @ E, Rp @ C, Rp @ st["A"]]
        if st["thumb"] and LAYOUT != "axial":
            pts.append(Rp @ st["K"])
        polys.append(np.array(pts))
    return polys


def seg_seg_dist(P0, P1, Q0, Q1):
    """Vectorised minimum distance between segments P0P1 and Q0Q1 (arrays (m, 3))."""
    u, v, w0 = P1 - P0, Q1 - Q0, P0 - Q0
    a, b, c = (u * u).sum(1), (u * v).sum(1), (v * v).sum(1)
    d, e = (u * w0).sum(1), (v * w0).sum(1)
    den = a * c - b * b
    sN = np.where(den > 1e-12, (b * e - c * d), 0.0)
    sD = np.where(den > 1e-12, den, 1.0)
    s = np.clip(sN / sD, 0, 1)
    t = np.clip((b * s + e) / np.where(c > 1e-12, c, 1.0), 0, 1)
    s = np.clip((b * t - d) / np.where(a > 1e-12, a, 1.0), 0, 1)
    t = np.clip((b * s + e) / np.where(c > 1e-12, c, 1.0), 0, 1)
    diff = w0 + s[:, None] * u - t[:, None] * v
    return np.linalg.norm(diff, axis=1)


def min_skew(polys, sel=None, all_pairs=False):
    """Smallest centreline distance between any two different strands. sel = segment indices to use
    (0 spool->forearm plate, 1 forearm plate->eye, 2 eye->palm plate, 3 palm plate->anchor)."""
    segs, owner = [], []
    for i, p in enumerate(polys):
        for k in range(len(p) - 1):
            if sel is not None and k not in sel:
                continue
            segs.append((p[k], p[k + 1]))
            owner.append(i)
    P0 = np.array([s[0] for s in segs])
    P1 = np.array([s[1] for s in segs])
    owner = np.array(owner)
    I, J = np.triu_indices(len(segs), 1)
    m = owner[I] != owner[J]
    I, J = I[m], J[m]
    dd = seg_seg_dist(P0[I], P1[I], P0[J], P1[J])
    if all_pairs:
        return dd, owner[I], owner[J]
    k = int(np.argmin(dd))
    return float(dd[k]), (int(owner[I[k]]), int(owner[J[k]]))


# ------------------------------------------------------------------------------------------------
# Hole profiles for the exact model
# ------------------------------------------------------------------------------------------------
def profile(opt, n=15):
    """(h, rho) pairs: rho = max radius of the line CENTRE at axial height h (inflated surface)."""
    rl = LINE_D / 2
    if opt["kind"] == "point":
        return [(0.0, 1e-4)]
    if opt["kind"] == "torus":
        rt, r = opt["r_throat"], opt["r_lip"]
        R0, rr = rt + r, r + rl
        hs = rr * np.sin(np.linspace(-0.5 * math.pi * 0.985, 0.5 * math.pi * 0.985, n))
        return [(float(h), float(R0 - math.sqrt(rr * rr - h * h))) for h in hs]
    if opt["kind"] == "tube":
        rb, L, rf = opt["r_bore"], opt["length"], opt["r_flare"]
        h0 = L / 2 - rf  # end of the straight bore
        rr = rf + rl
        out = []
        for hb in np.linspace(-h0, h0, 5):
            out.append((float(hb), rb - rl))
        for sgn in (-1, 1):
            for a in np.linspace(0.15, 0.5 * math.pi * 0.985, (n - 5) // 2):
                dh = rr * math.sin(a)
                out.append((float(sgn * (h0 + dh)), float(rb + rf - math.sqrt(rr * rr - dh * dh))))
        return sorted(out)
    raise ValueError(opt)


def _basis(n):
    n = n / np.linalg.norm(n)
    a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = np.cross(n, a)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(n, e1)


def exact_path(A, S, stacks, z0=None):
    """Shortest path A -> discs -> S. stacks: list of (O (k,3), e1 (3), e2 (3), rho (k)), in order from A to S.
    Returns total length, points (m, 3), solver x."""
    O = np.vstack([s[0] for s in stacks])
    E1 = np.vstack([np.repeat(s[1][None], len(s[0]), 0) for s in stacks])
    E2 = np.vstack([np.repeat(s[2][None], len(s[0]), 0) for s in stacks])
    rho = np.concatenate([s[3] for s in stacks])
    m = len(O)

    def pts(x):
        u, v = x[:m], x[m:]
        return O + u[:, None] * E1 + v[:, None] * E2

    def f(x):
        P = np.vstack([A, pts(x), S])
        dv = np.diff(P, axis=0)
        ln = np.linalg.norm(dv, axis=1)
        g = dv / np.maximum(ln, 1e-12)[:, None]
        gp = g[:-1] - g[1:]  # dL/dp_k
        return ln.sum(), np.concatenate([(gp * E1).sum(1), (gp * E2).sum(1)])

    cons = {"type": "ineq", "fun": lambda x: rho**2 - x[:m] ** 2 - x[m:] ** 2,
            "jac": lambda x: np.hstack([np.diag(-2 * x[:m]), np.diag(-2 * x[m:])])}
    x0 = np.zeros(2 * m) if z0 is None else z0
    res = minimize(f, x0, jac=True, constraints=[cons], method="SLSQP",
                   options=dict(ftol=1e-12, maxiter=400))
    return float(res.fun), pts(res.x), res.x


def build_stack(centre, axis, prof):
    e1, e2 = _basis(axis)
    a = axis / np.linalg.norm(axis)
    O = np.array([centre + h * a for h, _ in prof])
    return [O, e1, e2, np.array([r for _, r in prof])]


def stack_world(stack, R):
    O, e1, e2, rho = stack
    return [O @ R.T, R @ e1, R @ e2, rho]


def strand_exact(args):
    """All poses for one strand. Returns lengths, wraps at (palm comb, eye, forearm comb) and pivot data."""
    st, eye, eye_opt, poses = args
    E, C, F = guide_points(st, eye)
    A, S = st["A"], st["S"]
    eye_prof = profile(eye_opt)
    eye_prof = sorted(eye_prof, key=lambda t: -t[0])  # palm side (+z) first
    eye_stack = build_stack(E, np.array([0, 0, 1.0]), [(h, r) for h, r in eye_prof])
    stacks_body = []
    if LAYOUT == "axial":  # outer plates = same torus eyes, axis z
        stacks_body.append(("palm", build_stack(C, np.array([0, 0, 1.0]), eye_prof)))
        stacks_body.append(("ring", eye_stack))
        stacks_body.append(("fore", build_stack(F, np.array([0, 0, 1.0]), eye_prof)))
    else:
        comb_prof = profile(COMB_LINER, n=9)
        if not st["thumb"]:
            ax = (E - A) / np.linalg.norm(E - A)
            stacks_body.append(("palm", build_stack(C, ax, sorted(comb_prof, key=lambda t: t[0]))))
        stacks_body.append(("ring", eye_stack))
        axf = (S - E) / np.linalg.norm(S - E)
        stacks_body.append(("fore", build_stack(F, axf, sorted(comb_prof, key=lambda t: t[0]))))
    Ls, wraps, eye_pts = [], [], []
    z = None
    for f, d in poses:
        Rr, Rp = R_ring(f, d), R_palm(f, d)
        Rb = {"palm": Rp, "ring": Rr, "fore": np.eye(3)}
        sts = [stack_world(s, Rb[b]) for b, s in stacks_body]
        Aw = Rp @ A
        L, P, z = exact_path(Aw, S, sts, z)
        # wraps at each stack: angle between segment entering and leaving the stack
        sizes = [len(s[0]) for s in sts]
        idx = np.cumsum([0] + sizes)
        full = np.vstack([Aw, P, S])
        w = []
        for k in range(len(sts)):
            i0, i1 = idx[k] + 1, idx[k + 1]  # in `full`, stack k occupies rows i0..i1
            din = full[i0] - full[i0 - 1]
            dout = full[i1 + 1] - full[i1]
            w.append(ang(din, dout))
        Ls.append(L)
        wraps.append(w)
        # eye contact point (ring frame) for the plots
        ek = [b for b, _ in stacks_body].index("ring")
        eye_pts.append((P[idx[ek]:idx[ek + 1]] @ Rr).tolist())
    names = [b for b, _ in stacks_body]
    return dict(L=np.array(Ls), wraps=np.array(wraps), stacks=names)


# ------------------------------------------------------------------------------------------------
# Eye assignment
# ------------------------------------------------------------------------------------------------
GRID = {"nx": 8, "ny": 4}


def grid_eyes(pitch, nx=None, ny=None):
    nx, ny = nx or GRID["nx"], ny or GRID["ny"]
    xs = (np.arange(nx) - (nx - 1) / 2) * pitch
    ys = (np.arange(ny) - (ny - 1) / 2) * pitch
    pts = np.array([(x, y) for y in ys for x in xs if abs(x) + abs(y) > 1e-9])  # no eye at the centre
    # pairs: (e, -e) with e in the half-plane x > 0 or (x == 0 and y > 0)
    pairs = []
    used = set()
    for i, p in enumerate(pts):
        if i in used:
            continue
        j = int(np.argmin(np.linalg.norm(pts + p, axis=1)))
        used |= {i, j}
        pairs.append((i, j))
    return pts, pairs


def joint_pairs(strands):
    by = {}
    for i, s in enumerate(strands):
        by.setdefault(s["sid"], {})[s["role"]] = i
    return {sid: (v["flex"], v["ext"]) for sid, v in sorted(by.items())}


def eval_joint(strands, fi, xi, ef, ex, poses):
    """Fast-model numbers for one joint with flex eye ef and ext eye ex."""
    st = [strands[fi], strands[xi]]
    G = [guide_points(s, e) for s, e in zip(st, (ef, ex))]
    E = np.array([g[0] for g in G])
    C = np.array([g[1] for g in G])
    F = np.array([g[2] for g in G])
    L = fast_lengths(E, C, F, poses)
    dL = L - fast_lengths(E, C, F, [(0.0, 0.0)])[0]
    diff = (dL[:, 0] - dL[:, 1]) / 2
    common = dL[:, 0] + dL[:, 1]
    travel = (diff.max() - diff.min()) / R_SPOOL / DEG
    return dict(travel=float(travel), common=float(np.abs(common).max()), diff=diff, cm=common)


def assign(strands, pitch, poses, seed=0, iters=4000, verbose=False):
    """Hungarian on travel + common-mode, then local search (swaps, flips) with a skew / comb-spacing penalty."""
    pts, pairs = grid_eyes(pitch)
    jp = joint_pairs(strands)
    sids = list(jp)
    nP = len(pairs)
    cost = np.zeros((len(sids), nP))
    best_o = np.zeros((len(sids), nP), int)
    cache = {}
    for a, sid in enumerate(sids):
        fi, xi = jp[sid]
        need = JOINTS[sid][1]
        for b, (i, j) in enumerate(pairs):
            cs = []
            for o, (e1, e2) in enumerate(((pts[i], pts[j]), (pts[j], pts[i]))):
                r = eval_joint(strands, fi, xi, e1, e2, poses)
                cache[(sid, b, o)] = r
                k_cm = LINE_EA / (2 * 230.0)
                over = max(0.0, need + r["travel"] + TRAVEL_MARGIN - SERVO_TRAVEL)
                cs.append(1000 * over + r["common"] * k_cm + 0.02 * r["travel"])
            o = int(np.argmin(cs))
            cost[a, b], best_o[a, b] = cs[o], o
    rows, cols = linear_sum_assignment(cost)
    sol = {sids[r]: (int(c), int(best_o[r, c])) for r, c in zip(rows, cols)}

    def eyes_of(sol):
        eyes = [None] * len(strands)
        for sid, (b, o) in sol.items():
            fi, xi = jp[sid]
            i, j = pairs[b]
            ef, ex = (pts[i], pts[j]) if o == 0 else (pts[j], pts[i])
            eyes[fi], eyes[xi] = ef, ex
        return eyes

    sk_poses = [(f, d) for f in (FLEX_RANGE[0], 0.0, FLEX_RANGE[1]) for d in (DEV_RANGE[0], 0.0, DEV_RANGE[1])]
    # forearm slots (axial layout: the spool position does not change the wrist kinematics, only the fan)
    slot0 = {}
    for sid, (fi, xi) in jp.items():
        slot0[sid] = dict(S={"flex": strands[fi]["S"].copy(), "ext": strands[xi]["S"].copy()},
                          c=strands[fi]["spool_c"].copy(), ax=strands[fi]["spool_axis"].copy(),
                          level=1 if strands[fi]["spool_c"][2] > -150 else 2)
    slots = {sid: sid for sid in sids}  # joint sid -> physical slot (named by its V2 occupant)
    flips = {sid: False for sid in sids}

    def apply_slots(slots, flips):
        for sid, (fi, xi) in jp.items():
            g = slot0[slots[sid]]
            a, b = ("ext", "flex") if flips[sid] else ("flex", "ext")
            strands[fi]["S"], strands[xi]["S"] = g["S"][a], g["S"][b]
            strands[fi]["spool_c"] = strands[xi]["spool_c"] = g["c"]
            strands[fi]["spool_axis"] = strands[xi]["spool_axis"] = g["ax"]

    def total(sol, slots=None, flips=None):
        c = sum(cost_of(sid, b, o) for sid, (b, o) in sol.items())
        eyes = eyes_of(sol)
        if LAYOUT == "axial":  # wrist spans are parallel; the forearm fan is free space -> must be clear
            apply_slots(slots, flips)
            pl = fast_polylines(strands, eyes, (0.0, 0.0))
            ddf, _, _ = min_skew(pl, sel=(0,), all_pairs=True)
            sk = float(ddf.min())
            ddp, _, _ = min_skew(pl, sel=(3,), all_pairs=True)
            cs = {"palm_cross": int((ddp < LINE_D + 0.5).sum()), "fore_cross": int((ddf < LINE_D + 0.5).sum())}
            pen = 20 * cs["fore_cross"] + 20 * max(0.0, (LINE_D + 0.5) - sk) + 2.0 * cs["palm_cross"]
            # wrist-drive torque when all flex strands pull: sum T * (y, x) of the flex eyes (report 13 §2)
            wsum = np.zeros(2)
            for st, e in zip(strands, eyes):
                if st["role"] == "flex":
                    w = 1.25 if (st["joint"].endswith("mcp_flex") and not st["thumb"]) else 1.0
                    wsum += w * np.array([e[1], e[0]])
            cs["flex_eye_moment_mm"] = wsum.round(2).tolist()
            pen += MOMENT_W * (abs(wsum[0]) + abs(wsum[1]))
        else:
            sk = min(min_skew(fast_polylines(strands, eyes, p))[0] for p in sk_poses)
            cs = comb_spacing(strands, eyes)
            pen = 200 * max(0.0, (LINE_D + 0.5) - sk) + 100 * max(0.0, 2.6 - cs["palm"]) + 100 * max(0.0, 2.6 - cs["fore"])
        return c + pen, sk, cs

    def cost_of(sid, b, o):
        r = cache[(sid, b, o)]
        need = JOINTS[sid][1]
        over = max(0.0, need + r["travel"] + TRAVEL_MARGIN - SERVO_TRAVEL)
        return 1000 * over + r["common"] * LINE_EA / (2 * 230.0) + 0.02 * r["travel"]

    rng = np.random.default_rng(seed)
    cur, sk, cs = total(sol, slots, flips)
    best = (cur, dict(sol), dict(slots), dict(flips))
    T = 1.0
    for it in range(iters):
        new, nsl, nfl = dict(sol), dict(slots), dict(flips)
        a, b = rng.choice(sids, 2, replace=False)
        mv = rng.random()
        if mv < 0.35:
            new[a], new[b] = (new[b][0], new[a][1]), (new[a][0], new[b][1])
            for s_ in (a, b):
                bb = new[s_][0]
                new[s_] = (bb, int(np.argmin([cost_of(s_, bb, 0), cost_of(s_, bb, 1)])))
        elif mv < 0.5:
            new[a] = (new[a][0], 1 - new[a][1])
        elif mv < 0.85 and LAYOUT == "axial":
            # swap two servos between forearm slots of the same level (keeps the HLS room on level 2)
            if slot0[nsl[a]]["level"] == slot0[nsl[b]]["level"]:
                nsl[a], nsl[b] = nsl[b], nsl[a]
        elif LAYOUT == "axial":
            nfl[a] = not nfl[a]
        val, _, _ = total(new, nsl, nfl)
        if val < cur or rng.random() < math.exp(-(val - cur) / max(T, 1e-3)):
            sol, slots, flips, cur = new, nsl, nfl, val
            if cur < best[0]:
                best = (cur, dict(sol), dict(slots), dict(flips))
        T *= 0.9985
    sol, slots, flips = best[1], best[2], best[3]
    val, sk, cs = total(sol, slots, flips)
    if verbose:
        print(f"    assignment pitch {pitch}: cost {val:.2f}, forearm-fan min distance {sk:.2f} mm, crossings {cs}")
    ASSIGN_INFO.update(slots=dict(slots), flips=dict(flips), crossings=cs, fan_min=sk)
    return sol, eyes_of(sol), pts, pairs, cache


ASSIGN_INFO = {}
MOMENT_W = 3.0 if "--round2" in sys.argv else 0.0  # round 2 balances the wrist torque (report 13)


def fan_crossings(strands, eyes, sel, spacing=2.0):
    """Pairs of strands whose straight fan lines come closer than d + 0.5 mm. Fix: a spacer comb at the
    closest-approach height with the two holes `spacing` apart; each line is pushed sideways by
    (spacing - d)/2 and bends by ~ 2 * atan(offset / shorter arm) there (fixed, small)."""
    pl = fast_polylines(strands, eyes, (0.0, 0.0))
    dd, I, J = min_skew(pl, sel=sel, all_pairs=True)
    out = []
    for d, i, j in zip(dd, I, J):
        if d < LINE_D + 0.5:
            k = sel[0]
            a0, a1 = pl[i][k], pl[i][k + 1]
            b0, b1 = pl[j][k], pl[j][k + 1]
            # closest point parameter on a (sampled)
            ts = np.linspace(0, 1, 401)
            pa = a0 + ts[:, None] * (a1 - a0)
            dist = np.array([np.min(np.linalg.norm(b0 + ts[:, None] * (b1 - b0) - q, axis=1)) for q in pa])
            t = ts[int(np.argmin(dist))]
            arm = min(t, 1 - t) * np.linalg.norm(a1 - a0)
            off = (spacing - d) / 2
            bend = math.degrees(2 * math.atan(off / max(arm, 1.0)))
            out.append(dict(a=strands[i]["name"], b=strands[j]["name"], d=float(d),
                            z=float((a0 + t * (a1 - a0))[2]), arm=float(arm), bend_deg=float(bend)))
    return out


def comb_spacing(strands, eyes):
    """Smallest distance between comb hole centres in the palm comb (fingers) and the forearm comb."""
    Cs, Fs = [], []
    for st, e in zip(strands, eyes):
        E, C, F = guide_points(st, e)
        if not st["thumb"]:
            Cs.append(C[:2])
        Fs.append(F[:2])
    out = {}
    for k, arr in (("palm", np.array(Cs)), ("fore", np.array(Fs))):
        d = np.linalg.norm(arr[:, None] - arr[None], axis=2) + np.eye(len(arr)) * 1e9
        out[k] = float(d.min())
    return out


# ------------------------------------------------------------------------------------------------
# Main study
# ------------------------------------------------------------------------------------------------
LOG = []


def p(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    LOG.append(s)


def hdr(t):
    p("")
    p("=" * 100)
    p(t)
    p("=" * 100)


def strand_total_len(st, E, C, F):
    return (np.linalg.norm(st["S"] - F) + np.linalg.norm(F - E) + np.linalg.norm(E - C) + np.linalg.norm(C - st["A"])
            + (np.linalg.norm(st["A"] - st["K"]) if st["thumb"] else 0.0) + st["finger_len"])


def fixed_wraps(st, E, C, F):
    """q = 0 wraps on palm-fixed / forearm-fixed guides (sliding vs rolling) outside the comb-eye-comb chain."""
    out = {}
    if st["thumb"] and LAYOUT == "axial":
        out["thumb_entry_pin"] = ang(st["K"] - C, -st["Kdir"])
        out["sliding"] = out["thumb_entry_pin"]
        out["rollers"] = 0
    elif st["thumb"]:
        # at G_t: from (G_t - E) to (K - G_t); at the thumb entry K: from (K - G_t) to -Kdir
        out["thumb_guide_pin"] = ang(st["A"] - E, st["K"] - st["A"])
        out["thumb_entry_pin"] = ang(st["K"] - st["A"], -st["Kdir"])
        out["sliding"] = out["thumb_guide_pin"] + out["thumb_entry_pin"]
        out["rollers"] = 0
    else:
        kn = ang(st["A"] - C, -st["Kdir"])
        out["knuckle"] = kn
        if "abd" in st["joint"]:
            out["sliding"] = kn  # abd mouth: Ø3 dowel (sliding)
            out["rollers"] = 0
        else:
            out["sliding"] = 0.0  # knuckle idler MR63 (rolling)
            out["rollers"] = 1
    out["spool_fleet"] = ang(st["S"] - F, st["S"] - F - (st["S"] - F) @ st["spool_axis"] * st["spool_axis"])
    return out


def run():
    t_start = time.time()
    OUT.mkdir(parents=True, exist_ok=True)
    strands = load_strands()
    jp = joint_pairs(strands)
    poses = wrist_grid()
    poses_fast = wrist_grid(5 if QUICK else 9)
    RES = dict(assumptions=dict(flex=FLEX_RANGE, dev=DEV_RANGE, grid=N_GRID, line_d=LINE_D, EA=LINE_EA,
                                r_spool=R_SPOOL, travel=SERVO_TRAVEL, margin=TRAVEL_MARGIN, H_palm=H_PALM,
                                H_fore=H_FORE, mu=MU, comb_liner=COMB_LINER, thumb_guide=THUMB_GUIDE), options={})

    hdr("0. First principles: what the eye plate does")
    p("A strand that crosses a joint changes length by (its moment arm about the axis) x (angle).")
    p("Eye at (x, y) on the plate in the plane of both axes: palm segment arm about X = y, forearm segment arm")
    p("about Y = x. So dL ~ y*flex + x*dev (first order), and the servo travel to cancel it over the range is")
    p(f"  T = (|x|*{(DEV_RANGE[1]-DEV_RANGE[0]):.0f} deg + |y|*{(FLEX_RANGE[1]-FLEX_RANGE[0]):.0f} deg) / r_spool"
      f" = {(DEV_RANGE[1]-DEV_RANGE[0])*DEG/R_SPOOL/DEG:.1f}|x| + {(FLEX_RANGE[1]-FLEX_RANGE[0])*DEG/R_SPOOL/DEG:.1f}|y|"
      " servo deg (x, y in mm)")
    p("Pretension: a point-symmetric pair cancels the first order. What is left is second order, and above all")
    p("WHERE the line pivots: a pivot a height h above the axis plane adds ~ h*theta^2/2 per strand, the same sign")
    p("for flex and ext (common mode). The servo cannot cancel common mode: it only becomes pretension.")
    k_cm_nom = LINE_EA / (2 * 260.0)
    p(f"Common-mode loop stiffness ~ EA/(L_flex + L_ext) = {LINE_EA/1e3:.1f} kN / 520 mm = {k_cm_nom:.0f} N/mm "
      f"(soft braid {k_cm_nom*EA_SOFT:.0f}). Tolerance +-{PRETENSION_TOL} N -> +-{PRETENSION_TOL/k_cm_nom:.2f} mm per loop.")
    # pivot height demo (point model)
    for h in (0.0, 0.5, 1.25):
        E = np.array([[0.0, 2.0, h], [0.0, -2.0, h]])
        C = np.array([[0, 2.0, 30.0], [0, -2.0, 30.0]])
        F = np.array([[0, 2.0, -30.0], [0, -2.0, -30.0]])
        L = fast_lengths(E, C, F, [(0, 0), (50, 0)])
        cm = (L[1] - L[0]).sum()
        p(f"  pivot height {h:4.2f} mm, flex 50: pair common mode {cm:+.3f} mm = {cm*k_cm_nom:+.1f} N")
    RES["pivot_demo"] = "see txt"

    # ---------------------------------------------------------------- per eye option
    eye_results = {}
    for name, opt in EYE_OPTIONS.items():
        hdr(f"1. Eye option {name}: {opt['desc']}  (pitch {opt['pitch']} mm, {opt.get('nx', 8)} x {opt.get('ny', 4)} grid)")
        t0 = time.time()
        strands = load_strands()
        jp = joint_pairs(strands)
        GRID.update(nx=opt.get("nx", 8), ny=opt.get("ny", 4))
        sol, eyes, pts, pairs, cache = assign(strands, opt["pitch"], poses_fast, iters=600 if QUICK else 2500,
                                              verbose=True)
        p(f"  assignment done in {time.time()-t0:.0f} s; servo slots (joint sid -> V2 slot of sid): "
          f"{ {k: v for k, v in ASSIGN_INFO.get('slots', {}).items() if k != v} }, spool flips: "
          f"{[k for k, v in ASSIGN_INFO.get('flips', {}).items() if v]}")
        fx = fan_crossings(strands, eyes, (0,))
        px = fan_crossings(strands, eyes, (3,))
        p(f"  near-crossings (centre distance < {LINE_D + 0.5:.2f} mm) in the forearm fan: {len(fx)} pairs, max spacer bend "
          f"{max([c['bend_deg'] for c in fx], default=0):.1f} deg; palm fan: {len(px)} pairs, max bend "
          f"{max([c['bend_deg'] for c in px], default=0):.1f} deg (fixed bends at a spacer comb / trough)")
        info = dict(ASSIGN_INFO, fore_cross=fx, palm_cross=px)
        # exact model, all strands, 9 x 9
        t0 = time.time()
        args = [(st, e, opt, poses + [(0.0, 0.0)]) for st, e in zip(strands, eyes)]
        with Pool(4) as pool:
            ex = pool.map(strand_exact, args)
        p(f"  exact shortest-path model: {len(strands)} strands x {len(poses)} poses in {time.time()-t0:.0f} s")
        i0 = len(poses)  # reference pose q = 0 appended after the 9 x 9 grid
        rows = []
        for sid, (fi, xi) in jp.items():
            Lf, Lx = (ex[fi]["L"] - ex[fi]["L"][i0])[:i0], (ex[xi]["L"] - ex[xi]["L"][i0])[:i0]
            diff = (Lf - Lx) / 2
            cm = Lf + Lx
            th = np.array([[f * DEG, d * DEG] for f, d in poses])
            coef, *_ = np.linalg.lstsq(th, diff, rcond=None)
            resid = diff - th @ coef
            thq = np.c_[th, th[:, 0] ** 2, th[:, 1] ** 2, th[:, 0] * th[:, 1]]
            coefq, *_ = np.linalg.lstsq(thq, diff, rcond=None)
            residq = diff - thq @ coefq
            # loop length for stiffness
            E_, C_, F_ = guide_points(strands[fi], eyes[fi])
            Lf_tot = strand_total_len(strands[fi], E_, C_, F_)
            E2, C2, F2 = guide_points(strands[xi], eyes[xi])
            Lx_tot = strand_total_len(strands[xi], E2, C2, F2)
            k_cm = LINE_EA / (Lf_tot + Lx_tot)
            travel = (diff.max() - diff.min()) / R_SPOOL / DEG
            need = JOINTS[sid][1]
            wr_eye = max(ex[fi]["wraps"][:, ex[fi]["stacks"].index("ring")].max(),
                         ex[xi]["wraps"][:, ex[xi]["stacks"].index("ring")].max())
            rows.append(dict(sid=sid, joint=JOINTS[sid][0], eye_flex=[round(float(v), 3) for v in eyes[fi]],
                             eye_ext=[round(float(v), 3) for v in eyes[xi]],
                             k_flex_mm_per_rad=float(coef[0]), k_dev_mm_per_rad=float(coef[1]),
                             c_flex_deg_per_deg=float(-coef[0] / R_SPOOL), c_dev_deg_per_deg=float(-coef[1] / R_SPOOL),
                             lin_resid_mm=float(np.abs(resid).max()), lin_resid_joint_deg=float(np.abs(resid).max() / R_DRUM / DEG),
                             quad_resid_mm=float(np.abs(residq).max()), quad_coef=[float(c) for c in coefq],
                             diff_max_mm=float(np.abs(diff).max()), travel_deg=float(travel), need_deg=need,
                             total_deg=float(need + travel), fits=bool(need + travel + TRAVEL_MARGIN <= SERVO_TRAVEL),
                             common_max_mm=float(cm.max()), common_min_mm=float(cm.min()), k_cm_N_per_mm=float(k_cm),
                             dT_max_N=float(cm.max() * k_cm), dT_min_N=float(cm.min() * k_cm),
                             dT_soft_N=float(np.abs(cm).max() * k_cm * EA_SOFT), eye_wrap_max_deg=float(wr_eye),
                             loop_len_mm=float(Lf_tot + Lx_tot)))
        p(f"  {'joint':16s} {'eye flex':>14s} {'eye ext':>14s} {'c_flex':>7s} {'c_dev':>7s} {'lin res':>8s} "
          f"{'travel':>7s} {'need+T':>7s} {'cm mm':>14s} {'dT N':>13s} {'eye wrap':>8s}")
        for r in rows:
            p(f"  {r['joint']:16s} ({r['eye_flex'][0]:+5.2f},{r['eye_flex'][1]:+5.2f}) ({r['eye_ext'][0]:+5.2f},{r['eye_ext'][1]:+5.2f}) "
              f"{r['c_flex_deg_per_deg']:+7.3f} {r['c_dev_deg_per_deg']:+7.3f} {r['lin_resid_mm']:8.3f} {r['travel_deg']:7.1f} "
              f"{r['total_deg']:7.1f}{'' if r['fits'] else '!'} {r['common_min_mm']:+6.3f}..{r['common_max_mm']:+6.3f} "
              f"{r['dT_min_N']:+5.1f}..{r['dT_max_N']:+5.1f} {r['eye_wrap_max_deg']:8.1f}")
        dT = np.array([max(abs(r["dT_max_N"]), abs(r["dT_min_N"])) for r in rows])
        # skew on the exact paths is close to the fast polylines; use fast polylines on the full grid
        sk_all = [(min_skew(fast_polylines(strands, eyes, ps), sel=(1, 2)), ps) for ps in poses]
        (skd, (a, b)), skp = min(sk_all, key=lambda t: t[0][0])
        cs = comb_spacing(strands, eyes)
        p(f"  worst pretension change {dT.max():.1f} N (median {np.median(dT):.1f}); all joints fit the "
          f"{SERVO_TRAVEL:.0f} deg servo: {all(r['fits'] for r in rows)}; worst linear residual "
          f"{max(r['lin_resid_mm'] for r in rows):.3f} mm ({max(r['lin_resid_joint_deg'] for r in rows):.2f} joint deg)")
        p(f"  WRIST FREE SPANS (forearm plate -> eye -> palm plate): min centreline distance over the 9x9 grid: {skd:.2f} mm ({strands[a]['name']} / "
          f"{strands[b]['name']} at flex {skp[0]:.0f} dev {skp[1]:.0f}); surface gap {skd-LINE_D:.2f} mm")
        p(f"  comb hole spacing: palm {cs['palm']:.2f} mm, forearm {cs['fore']:.2f} mm (16G liner OD 1.65 + 1 mm web = 2.65)")
        lip = opt.get("r_lip", opt.get("r_flare"))
        pr = {s: T / (lip * LINE_D) for s, T in T_STALL.items()}
        p(f"  lip pressure p = T/(r d): r {lip} -> {pr['SCS0009']:.0f} MPa at SCS stall, {pr['HLS3606M']:.0f} MPa at HLS stall")
        eye_results[name] = dict(opt=opt, rows=rows, assign_info=info, layout=LAYOUT, strands=strands, eyes=[e.tolist() for e in eyes], max_dT=float(dT.max()),
                                 median_dT=float(np.median(dT)), min_skew_mm=skd, skew_pair=[strands[a]["name"], strands[b]["name"]],
                                 skew_pose=skp, comb_spacing=cs, pressure_MPa=pr,
                                 wraps={strands[i]["name"]: dict(stacks=ex[i]["stacks"], q0=ex[i]["wraps"][i0].tolist(),
                                                                 max=ex[i]["wraps"].max(0).tolist(),
                                                                 sum_max=float(ex[i]["wraps"].sum(1).max()))
                                        for i in range(len(strands))},
                                 dL_strand={strands[i]["name"]: dict(min=float((ex[i]["L"] - ex[i]["L"][i0]).min()),
                                                                     max=float((ex[i]["L"] - ex[i]["L"][i0]).max()))
                                            for i in range(len(strands))},
                                 cm_grid={JOINTS[sid][0]: ((ex[fi]["L"] - ex[fi]["L"][i0]) + (ex[xi]["L"] - ex[xi]["L"][i0]))[:i0].tolist()
                                          for sid, (fi, xi) in jp.items()})
        RES["options"][name] = {k: v for k, v in eye_results[name].items() if k not in ("cm_grid", "strands")}

    # ---------------------------------------------------------------- dowel fairlead (approximate)
    GRID.update(nx=8, ny=4)
    hdr("2. Crossed Ø3 dowel fairlead (approximate: point pivots at +-2 mm, pitch 3.6)")
    pitch = DOWEL_FAIRLEAD["pitch"]
    sol, eyes_d, pts, pairs, cache = assign(strands, pitch, poses_fast, iters=300)
    trav = []
    for sid, (fi, xi) in jp.items():
        r = eval_joint(strands, fi, xi, eyes_d[fi], eyes_d[xi], poses_fast)
        trav.append((JOINTS[sid][0], JOINTS[sid][1], r["travel"]))
    nfit = sum(1 for j, n, t in trav if n + t + TRAVEL_MARGIN <= SERVO_TRAVEL)
    p(f"  servo travel for compensation: max {max(t for _, _, t in trav):.0f} deg; joints that still fit 300 deg: {nfit}/16")
    # pivot offsets: palm pivot at +2 (X-dowel layer above), forearm at -2 -> common mode ~ 2*h*theta^2/2 per pair
    E = np.array([[0.0, 3.6, 2.0], [0.0, -3.6, 2.0]])
    C = np.array([[0, 3.6, 30.0], [0, -3.6, 30.0]])
    F = np.array([[0, 3.6, -30.0], [0, -3.6, -30.0]])
    L = fast_lengths(E, C, F, [(0, 0), (50, 0)])
    rc = DOWEL_FAIRLEAD["r_c"]
    b = 50 * DEG
    cm = (L[1] - L[0]).sum() + 2 * rc * (b - math.sin(b))
    p(f"  common mode at flex 50 (pivot +2 mm + wrap on r {rc:.2f}): {cm:+.2f} mm = {cm*k_cm_nom:+.0f} N")
    p(f"  pressure: {T_STALL['SCS0009']/(1.5*LINE_D):.0f} / {T_STALL['HLS3606M']/(1.5*LINE_D):.0f} MPa (SCS / HLS)")
    RES["dowel_fairlead"] = dict(max_travel=max(t for _, _, t in trav), fit=nfit, cm_flex50_mm=cm,
                                 cm_flex50_N=cm * k_cm_nom, travel=trav)

    # ---------------------------------------------------------------- chosen option: report tables
    best = min(eye_results, key=lambda k: (not all(r["fits"] for r in eye_results[k]["rows"]),
                                           eye_results[k]["max_dT"] + 3 * max(0, 0.97 - eye_results[k]["min_skew_mm"])))
    best_choice = "torus_r0.6_t0.65_7x5"
    hdr(f"3. Chosen eye: {best_choice} (best by the numbers: {best}); eye table and firmware coefficients")
    er = eye_results[best_choice]
    strands = er["strands"]
    jp = joint_pairs(strands)
    eyes = [np.array(e) for e in er["eyes"]]
    RES["slots"] = er["assign_info"].get("slots")
    RES["flips"] = er["assign_info"].get("flips")
    RES["fan_crossings"] = dict(forearm=er["assign_info"]["fore_cross"], palm=er["assign_info"]["palm_cross"])
    table = []
    p(f"  {'strand':24s} {'eye x,y':>14s} {'palm comb C':>22s} {'forearm comb F':>22s} {'dL min..max':>14s} "
      f"{'wrap q0 (pc/eye/fc)':>20s} {'eye wrap max':>12s}")
    for i, st in enumerate(strands):
        E, C, F = guide_points(st, eyes[i])
        w = er["wraps"][st["name"]]
        dl = er["dL_strand"][st["name"]]
        table.append(dict(name=st["name"], sid=st["sid"], role=st["role"], eye=eyes[i].round(3).tolist(),
                          palm_comb=C.round(2).tolist(), palm_comb_axis=((E - st["A"]) / np.linalg.norm(E - st["A"])).round(4).tolist(),
                          fore_comb=F.round(2).tolist(), fore_comb_axis=((st["S"] - E) / np.linalg.norm(st["S"] - E)).round(4).tolist(),
                          spool_tangent=st["S"].round(2).tolist(), anchor=st["A"].round(2).tolist(),
                          dL=dl, wraps=w, thumb=st["thumb"]))
        q0 = "/".join(f"{v:4.1f}" for v in w["q0"])
        p(f"  {st['name']:24s} ({eyes[i][0]:+5.2f},{eyes[i][1]:+5.2f}) ({C[0]:+6.2f},{C[1]:+6.2f},{C[2]:+5.1f}) "
          f"({F[0]:+6.2f},{F[1]:+6.2f},{F[2]:+5.1f}) {dl['min']:+6.2f}..{dl['max']:+5.2f} {q0:>20s} {max(w['max']):8.1f}")
    RES["eye_table"] = table
    RES["chosen"] = best_choice
    RES["joints"] = er["rows"]

    hdr("4. Firmware compensation: servo_offset_deg = c_flex * wrist_flex_deg + c_dev * wrist_dev_deg")
    for r in er["rows"]:
        p(f"  ID {r['sid']:2d} {r['joint']:16s} c_flex {r['c_flex_deg_per_deg']:+.3f}  c_dev {r['c_dev_deg_per_deg']:+.3f}"
          f"   (linear residual {r['lin_resid_mm']:.3f} mm = {r['lin_resid_joint_deg']:.2f} joint deg; with quadratic "
          f"terms {r['quad_resid_mm']:.3f} mm)")

    hdr("5. Servo travel budget (SCS0009 300 deg, verify)")
    for r in er["rows"]:
        p(f"  {r['joint']:16s} own {r['need_deg']:6.1f} + wrist {r['travel_deg']:5.1f} = {r['total_deg']:6.1f} deg"
          f"  {'OK' if r['fits'] else 'DOES NOT FIT'}  (spare {SERVO_TRAVEL - r['total_deg']:5.1f})")
    # what if the MCP gets a r 3 metal-hub spool (report 01)? travel x 5/3 for both own and wrist parts
    p("  If the MCP flex servos get report 01's r 3 spool: own 100 deg x 7/3 = 233 deg, wrist part x 5/3:")
    for r in er["rows"]:
        if "mcp_flex" in r["joint"] and not r["joint"].startswith("thumb"):
            t3 = r["travel_deg"] * 5 / 3
            p(f"    {r['joint']:16s} 233 + {t3:5.1f} = {233 + t3:6.1f} deg -> {'OK' if 233 + t3 + TRAVEL_MARGIN <= 300 else 'too much for SCS0009: use HLS (multi-turn, verify) or keep r 5'}")

    hdr("6. Wrap and efficiency per strand: V2 (routes.json + fan) vs bare strand (palm anchor -> spool)")
    p("  V2 = palm turn + forearm fan turn (PTFE); +58 deg in the wrist bundle at the worst pose.")
    p("  New = sliding wraps on comb/eye/comb (exact model) + fixed sliding pins (abd mouth, thumb pins) + rollers.")
    p("  Finger-internal bends are the same in both and not included.")
    p(f"  {'strand':24s} {'V2 q0':>6s} {'V2 worst':>8s} {'eta V2':>7s} {'eta V2 w':>8s} | {'new q0':>6s} {'new worst':>9s} "
      f"{'roll':>4s} {'eta new':>7s} {'eta new w':>9s}")
    cmp_rows = []
    spacer = {}
    for c in er["assign_info"]["fore_cross"]:  # palm-fan crossings sit in rigid troughs (two depths), not added
        for n in (c["a"], c["b"]):
            spacer[n] = spacer.get(n, 0.0) + c["bend_deg"]
    for i, st in enumerate(strands):
        E, C, F = guide_points(st, eyes[i])
        fw = fixed_wraps(st, E, C, F)
        w = er["wraps"][st["name"]]
        v2q0 = st["v2_palm_turn"] + st["v2_fan_turn"]
        v2w = v2q0 + 58.0
        fw["spacer"] = spacer.get(st["name"], 0.0)
        nq0 = sum(w["q0"]) + fw["sliding"] + fw["spacer"]
        nw = w["sum_max"] + fw["sliding"] + fw["spacer"]
        eta = lambda th, mu=MU: math.exp(-mu * th * DEG)  # noqa: E731
        en = eta(nq0) * ETA_ROLLER ** fw["rollers"]
        enw = eta(nw) * ETA_ROLLER ** fw["rollers"]
        cmp_rows.append(dict(name=st["name"], v2_q0=v2q0, v2_worst=v2w, eta_v2=eta(v2q0), eta_v2_worst=eta(v2w),
                             new_q0=nq0, new_worst=nw, rollers=fw["rollers"], eta_new=en, eta_new_worst=enw,
                             eta_new_mu015=eta(nq0, MU_HI) * ETA_ROLLER ** fw["rollers"],
                             eta_v2_mu015=eta(v2q0, MU_HI), fixed=fw))
        p(f"  {st['name']:24s} {v2q0:6.0f} {v2w:8.0f} {eta(v2q0):7.2f} {eta(v2w):8.2f} | {nq0:6.0f} {nw:9.0f} "
          f"{fw['rollers']:4d} {en:7.2f} {enw:9.2f}")
    a = np.array([[r["eta_v2"], r["eta_new"], r["eta_v2_worst"], r["eta_new_worst"]] for r in cmp_rows])
    p(f"  median eta: V2 {np.median(a[:,0]):.2f} -> new {np.median(a[:,1]):.2f} (wrist straight); "
      f"worst pose {np.median(a[:,2]):.2f} -> {np.median(a[:,3]):.2f}; min {a[:,0].min():.2f} -> {a[:,1].min():.2f}")
    RES["wrap_compare"] = cmp_rows

    hdr("7. Wear at the eye")
    lip = EYE_OPTIONS[best_choice]["r_lip"]
    for s, T in T_STALL.items():
        p(f"  {s}: lip pressure {T/(lip*LINE_D):.0f} MPa (Ø3 dowel {T/(1.5*LINE_D):.0f}, 18G rim r 0.43 {T/(0.43*LINE_D):.0f});"
          f" at the 5 N pretension {T0/(lip*LINE_D):.0f} MPa")
    p(f"  D/d at the lip = {2*lip/LINE_D:.1f} (fibre rule: >= 8 static, >= 20 cyclic): the eye is a WEAR point, the")
    p("  loaded wrap only exists while the wrist is bent; sliding per finger stroke = joint travel x 0.122 mm/deg.")
    wr = [max(v["max"][v["stacks"].index("ring")] for v in [er["wraps"][st["name"]]]) for st in strands]
    p(f"  eye wrap over the grid: median {np.median(wr):.0f} deg, max {max(wr):.0f} deg")
    RES["eye_wear"] = dict(lip=lip, Dd=2 * lip / LINE_D, eye_wrap_median=float(np.median(wr)), eye_wrap_max=float(max(wr)))

    # ---------------------------------------------------------------- plots
    try:
        plot(strands, eye_results, best_choice, poses, OUT / "eyeplate_kin.png")
    except Exception as e:  # pragma: no cover
        p(f"plot failed: {e}")
    (OUT / "eyeplate_kin.json").write_text(json.dumps(RES, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    (OUT / "eyeplate_kin.txt").write_text("\n".join(LOG))
    p(f"\ndone in {time.time()-t_start:.0f} s -> {OUT}")


def plot(strands, eye_results, best, poses, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    er = eye_results[best]
    fig, ax = plt.subplots(2, 2, figsize=(14, 11))
    # (a) eye plate map
    a = ax[0, 0]
    pitch = er["opt"]["pitch"]
    col = {"pip": "#1f77b4", "mcp_flex": "#d62728", "mcp_abd": "#2ca02c", "thumb": "#9467bd"}
    for st, e in zip(strands, er["eyes"]):
        k = "thumb" if st["thumb"] else st["joint"].split("_", 1)[1]
        mk = "o" if st["role"] == "flex" else "s"
        a.scatter(e[0], e[1], s=260, marker=mk, facecolor="white", edgecolor=col[k], lw=2)
        a.text(e[0], e[1], f"{st['sid']}{st['role'][0]}", ha="center", va="center", fontsize=7)
    a.set_aspect("equal")
    a.set_title(f"Eye plate ({best}, pitch {pitch}): ID + f/e; circle = flex, square = ext\n"
                "blue pip, red mcp_flex, green abd, purple thumb (ring frame, from the palm)")
    a.set_xlabel("x (thumb +)")
    a.set_ylabel("y (back +)")
    a.grid(alpha=0.3)
    # (b) common mode per joint over the grid
    a = ax[0, 1]
    fl = sorted({q[0] for q in poses})
    for name, cmv in er["cm_grid"].items():
        cm = np.array(cmv).reshape(len(fl), -1)
        a.plot(fl, np.abs(cm).max(1), label=name)
    a.set_xlabel("wrist flex (deg), max over dev")
    a.set_ylabel("|loop length change| (mm)")
    a.axhline(PRETENSION_TOL / (LINE_EA / 520), color="k", ls="--", lw=1)
    a.text(fl[0], PRETENSION_TOL / (LINE_EA / 520) * 1.03, f"+-{PRETENSION_TOL:.0f} N", fontsize=8)
    a.set_title("Common mode (pretension) per joint, exact model")
    a.legend(fontsize=6, ncol=2)
    a.grid(alpha=0.3)
    # (c) options comparison
    a = ax[1, 0]
    names = list(eye_results)
    dT = [eye_results[n]["max_dT"] for n in names]
    trav = [max(r["travel_deg"] for r in eye_results[n]["rows"]) for n in names]
    x = np.arange(len(names))
    a.bar(x - 0.2, dT, 0.4, label="worst pretension change (N)")
    a.bar(x + 0.2, np.array(trav) / 10, 0.4, label="worst compensation travel (servo deg / 10)")
    a.set_xticks(x)
    a.set_xticklabels(names, fontsize=8)
    a.axhline(PRETENSION_TOL, color="k", ls="--", lw=1)
    a.legend(fontsize=8)
    a.set_title("Eye options")
    a.grid(alpha=0.3, axis="y")
    # (d) side view of the paths at q0 and flex 50
    a = ax[1, 1]
    for pose, c in (((0.0, 0.0), "k"), ((50.0, 0.0), "tab:red"), ((-50.0, 0.0), "tab:blue")):
        polys = fast_polylines(strands, [np.array(e) for e in er["eyes"]], pose)
        for pl in polys:
            a.plot(pl[:, 1], pl[:, 2], c, lw=0.5, alpha=0.6)
    a.set_xlim(-40, 40)
    a.set_ylim(-60, 60)
    a.set_aspect("equal")
    a.set_xlabel("y")
    a.set_ylabel("z")
    a.set_title("Strands near the wrist (side view): q0 black, flex +50 red, -50 blue")
    a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


if __name__ == "__main__":
    run()
