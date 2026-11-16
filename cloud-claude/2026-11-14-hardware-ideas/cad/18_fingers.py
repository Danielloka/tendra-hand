"""18 · Four V2.1 fingers with sensors + palm knuckle modules, in the HAND frame (study 2026-11-14, round 3 package A).

API for the integrator (cad/18_assembly.py):
    import importlib.util, sys
    spec = importlib.util.spec_from_file_location("fingers18", "<study>/cad/18_fingers.py")
    F = importlib.util.module_from_spec(spec); spec.loader.exec_module(F)
    F.FINGERS                      -> ("index", "middle", "ring", "little")
    F.JOINTS                       -> ["index_mcp_abd", "index_mcp_flex", "index_pip", ...]  (deg, + = closing / toward thumb)
    F.parts()                      -> {part_name: {"body", "finger", "stl", "color", "printed"}}   (reads out/18/fingers/parts.json)
    F.pose(q)                      -> {part_name: 4x4 numpy array}  hand-frame transform of each part from its q = 0 export;
                                      q = {joint: deg}, missing joints = 0; DIP follows the bar (exact linkage)
    F.lines(q)                     -> {line_name: (N, 3) array} tendons (anchor -> palm plate hole at z 30) and sensor wires
                                      (tip -> JST plug on the knuckle module), hand frame, mm
    F.dip(finger, pip_deg)         -> DIP angle (deg)
    F.build()                      -> (re)builds the CAD and exports out/18/fingers/ (~3 min); pose/lines do not need it
Hand frame (V2): origin = wrist centre, +Z fingers, -Y palm, +X thumb. Parts are exported at q = 0 in that frame.
Body names: <finger>_module (static, palm), <finger>_yoke, _proximal, _middle, _distal, _bar.

How it works: each finger is cad/16_finger.py (round-2 sensor finger) with that finger's lengths patched in (V2
params.FINGERS), its DIP bar re-designed for its middle length (pin A off the web, as in 07/12), and the length-
dependent features scaled; the patched copies live in out/18/fingers/_gen/. The bench base is replaced by a knuckle
module and per-strand PTFE liners to report 13's palm plate (holes = 09/13 eye table, z 30).
Run: uv run --no-project --with cadquery --with numpy --with trimesh --with scipy --with rtree python cad/18_fingers.py
"""
from __future__ import annotations

import importlib.util
import itertools
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STUDY = HERE.parent
OUT = STUDY / "out" / "18" / "fingers"
GEN = OUT / "_gen"
V2CAD = STUDY.parent / "2026-11-09-v2-hand-cad" / "cad"
sys.path.insert(0, str(V2CAD))
sys.path.insert(0, str(STUDY.parents[1] / "hardware" / "cad"))
import params as P  # noqa: E402  (V2 params, read-only)

FINGERS = ("index", "middle", "ring", "little")
JOINTS = [f"{f}_{j}" for f in FINGERS for j in ("mcp_abd", "mcp_flex", "pip")]
EYE = {r["name"]: r["palm_comb"] for r in json.loads((STUDY / "out" / "13" / "kin" / "eyeplate_kin.json").read_text())["eye_table"]}
Z_PLATE = 30.0  # report 13: palm plate at z 30
LINER = dict(od=2.0, id=1.0)  # PTFE 1 x 2 in a printed palm channel (report 02 H5 / 13 §5)
STRAND_EYE = {"mcp_flex": "mcp_flex_flex", "mcp_ext": "mcp_flex_ext", "pip_flex": "pip_flex", "pip_ext": "pip_ext",
              "abd_flex": "mcp_abd_flex", "abd_ext": "mcp_abd_ext"}
MCP = {f: np.array(P.FINGERS[f]["mcp"], float) for f in FINGERS}
# 18: even 20 mm pitch (V2 had ring -19.5 and little -38: the staggered little then hits the ring when both abduct)
MCP["ring"][0], MCP["little"][0] = -20.0, -40.0
LEN = {f: P.FINGERS[f]["lengths"] for f in FINGERS}


# ------------------------------------------------------------------------------------------------ DIP bar per finger
def design_link(L2):
    """07's search with the pin-A-off-the-web rule (Ø2 pins, 1.6 mm walls to the Ø5 bushes)."""
    import dip_linkage as DL
    D2R = math.pi / 180
    DL.HALF_HEIGHT, DL.AXIS_CLEAR = 7.0, 2.0
    a_min = 2.5 + 1.6 + 1.0
    pipg = np.radians(np.linspace(-5, 95, 41))
    tgt = 0.75 * pipg

    def score(p):
        d = DL.dip_curve(L2, *p, pip=pipg)
        if d is None:
            return math.inf, None
        c = DL.check(L2, *p, pipg, d)
        return (math.inf, None) if c is None else (float(np.abs(d - tgt).max()), c)

    best = (math.inf, None)
    for a, b in itertools.product(np.arange(a_min, 5.61, 0.25), np.arange(a_min, 6.81, 0.25)):
        for ga, gb in itertools.product(range(0, 91, 5), range(-120, -59, 5)):
            p = (a, ga * D2R, b, gb * D2R)
            s = score(p)[0]
            if s < best[0]:
                best = (s, p)
    p, step, s0 = np.array(best[1]), np.array([0.05, D2R, 0.05, D2R]), best[0]
    while step[1] > 0.02 * D2R:
        imp = False
        for i, sg in itertools.product(range(4), (1, -1)):
            q = p.copy()
            q[i] += sg * step[i]
            if q[0] < a_min or q[2] < a_min or q[2] > 6.8:
                continue
            sq = score(q)[0]
            if sq < s0:
                p, s0, imp = q, sq, True
        if not imp:
            step /= 2
    a, ga, b, gb = (float(v) for v in p)
    bar = math.hypot(a * math.cos(ga) - L2 - b * math.cos(gb), a * math.sin(ga) - b * math.sin(gb))
    return dict(pin_a=dict(r=a, deg=ga / D2R), pin_b=dict(r=b, deg=gb / D2R), bar=bar, err_deg=s0 / D2R, L2=L2,
                clear=score(p)[1])


# ------------------------------------------------------------------------------------------------ patched 16 per finger
def _patch(src, f, link):
    L1, L2, L3 = LEN[f]
    TS = 0.0  # tip features stay at the index offsets (a shift on the little pushes the carrier into the DIP tongue);
    # little (L3 21): 1.5 mm end wall at the tip cavity

    def rep(a, b, cnt=1):
        nonlocal src
        assert a in src, (f, a[:70])
        src = src.replace(a, b, cnt)
    rep("HERE = Path(__file__).resolve().parent", f"HERE = Path({str(HERE)!r})")
    rep("L1, L2, L3 = 42.0, 24.0, 23.0", f"L1, L2, L3 = {L1}, {L2}, {L3}\nTS = {TS}")
    rep('LINK = BUD07["link"]', f"LINK = {json.dumps(link)}")
    rep("PAD_PROX = dict(x=(1.6, 7.0), z=(17.0, 27.0), floor_y=-3.74)",
        "PAD_PROX = dict(x=(1.6, 7.0), z=(round(0.405 * L1, 2), round(0.405 * L1 + 10.0, 2)), floor_y=-3.74)")
    rep("PAD_MID = dict(x=(-4.0, 4.0), z=(51.6, 57.8), floor_y=-3.25)",
        "PAD_MID = dict(x=(-4.0, 4.0), z=(L1 + 9.6, min(L1 + 15.8, L1 + L2 - 8.2)), floor_y=-3.25)\n"
        "PAD_MID = PAD_MID if PAD_MID['z'][1] - PAD_MID['z'][0] >= 6.0 else None  # no room on a short middle phalanx")
    rep("    m = m.cut(box(*PAD_MID[\"x\"], -12.0, PAD_MID[\"floor_y\"], *PAD_MID[\"z\"]))",
        "    if PAD_MID:\n        m = m.cut(box(*PAD_MID[\"x\"], -12.0, PAD_MID[\"floor_y\"], *PAD_MID[\"z\"]))")
    rep('''    for nm, P, body, face in (("prox", PAD_PROX, "proximal", -7.75), ("mid", PAD_MID, "middle", -7.25)):''',
        '''    for nm, P, body, face in (("prox", PAD_PROX, "proximal", -7.75), ("mid", PAD_MID, "middle", -7.25)):
        if not P:
            continue''')
    rep("30.0, ZP + 12))", "ZP - 12.0, ZP + 12))")
    rep("t30 = (31.0 - a[1]) / d[1]", "t30 = (PAD_PROX['z'][1] + 4.0 - a[1]) / d[1]")
    rep("t29 = (29.0 - a[1]) / d[1]", "t29 = (PAD_PROX['z'][1] + 2.0 - a[1]) / d[1]")
    src = src.replace("HHz(39)", "HHz(ZP - 3.0)").replace("38.4", "(ZP - 3.6)").replace("39.6", "(ZP - 2.4)")
    for a in ("ZD + 10.0", "ZD + 7.5, ZD + 10.1", "ZD + 10.3", "ZD + 9.0, ZT - 0.6", "ZD + 8.0, ZT + 1.0", "ZD + 15.0",
              "ZD + 13.5, ZD + 16.5", "ZD + 19.3"):
        src = src.replace(a, a.replace("ZD + ", "ZD + TS + "))
    rep('carrier = cq.importers.importStep(str(OUT15 / "tip_carrier.step"))',
        'carrier = cq.importers.importStep(str(OUT15 / "tip_carrier.step")).translate((0, 0, ZD - 66.0 + TS))')
    i, j = src.index("WIRE_PATH = ["), src.index("def sensor_wire(")
    src = src[:i] + '''WIRE_PATH = [
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


''' + src[j:]
    rep('m["abd_stub_palmar"] = ("base", cyly(1.5, -17.0, -10.1, 0, Z_ABD)', 'm["abd_stub_palmar"] = ("base", cyly(1.5, -15.9, -10.1, 0, Z_ABD)')
    rep('cap = cq.importers.importStep(str(OUT15 / f"joint_cap_{key}.step"))',
        'cap = cq.importers.importStep(str(OUT15 / f"joint_cap_{key}.step")).translate((0, 0, zc - {"mcp_flex": 0.0, "pip": 42.0, "dip": 66.0}[key]))')
    return src


_MODS = {}


def kin_file():
    return OUT / "fingers_kin.json"


def finger_module(f):
    """the patched 16_finger.py for finger f (imports cadquery; builds nothing until asked)."""
    if f in _MODS:
        return _MODS[f]
    GEN.mkdir(parents=True, exist_ok=True)
    kin = json.loads(kin_file().read_text()) if kin_file().exists() else {}
    if f not in kin.get("links", {}):
        link = design_link(float(LEN[f][1]))
        kin.setdefault("links", {})[f] = link
        kin_file().write_text(json.dumps(kin, indent=1, default=float))
    src = _patch((HERE / "16_finger.py").read_text(), f, kin["links"][f])
    path = GEN / f"16_{f}.py"
    if not path.exists() or path.read_text() != src:
        path.write_text(src)
    spec = importlib.util.spec_from_file_location(f"f16_{f}", path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[f"f16_{f}"] = m
    spec.loader.exec_module(m)
    _MODS[f] = m
    return m


def dip(f, pip_deg):
    return finger_module(f).dip_of(pip_deg)


# ------------------------------------------------------------------------------------------------ kinematics (hand frame)
def _T(f):
    M = np.eye(4)
    M[:3, 3] = MCP[f]
    return M


def _q(f, q):
    return dict(abd=float(q.get(f"{f}_mcp_abd", 0.0)), mcp=float(q.get(f"{f}_mcp_flex", 0.0)), pip=float(q.get(f"{f}_pip", 0.0)))


def _bar_T(m, ql):
    """bar transform (finger-local) from its q = 0 placement: rotate in the yz plane, move pin A."""
    dipd = m.dip_of(ql["pip"])
    A, B, A0, B0 = m.link_pins(ql["pip"], dipd, ql["mcp"])
    a0 = math.atan2(B0[1] - A0[1], B0[0] - A0[0])
    _, _, A00, B00 = m.link_pins(0, m.dip_of(0), 0)
    A0, B0 = m.link_pins(0, m.dip_of(0), 0)[:2]
    a0 = math.atan2(B0[1] - A0[1], B0[0] - A0[0])
    a1 = math.atan2(B[1] - A[1], B[0] - A[0])
    d = a1 - a0  # rotation about +X (in (y, z): positive turns +y toward +z)
    c, s = math.cos(d), math.sin(d)
    M = np.eye(4)
    M[1:3, 1:3] = [[c, -s], [s, c]]
    p0 = np.array([0.0, *A0])
    M[:3, 3] = np.array([0.0, *A]) - M[:3, :3] @ p0
    Ta = m.mat_rot((0, 1, 0), ql["abd"], (0, 0, m.Z_ABD))
    return Ta @ M


def body_transforms(f, q):
    m = finger_module(f)
    ql = _q(f, q)
    Tb, _ = m.body_T(ql)
    Tb = dict(Tb)
    Tb["bar"] = _bar_T(m, ql)
    Th, Thi = _T(f), np.linalg.inv(_T(f))
    out = {"module": np.eye(4)}
    for b in ("yoke", "proximal", "middle", "distal", "bar"):
        out[b] = Th @ Tb[b] @ Thi
    return out


_PARTS = None


def parts():
    global _PARTS
    if _PARTS is None:
        _PARTS = json.loads((OUT / "parts.json").read_text())
    return _PARTS


def pose(q):
    """{part_name: 4x4} in the hand frame for joint angles q (deg)."""
    out = {}
    cache = {f: body_transforms(f, q) for f in FINGERS}
    for name, info in parts().items():
        f = info["finger"]
        out[name] = cache[f][info["body"].split("_", 1)[1]]
    return out


def _liner_path(f, k, entry_local, mid=None):
    """entry (module floor bottom, finger-local) -> 3 mm vertical -> optimised midpoint -> palm plate hole (hand frame)."""
    e = np.asarray(entry_local, float) + MCP[f]
    e2 = e + np.array([0.0, 0.0, -3.0])
    h = np.array(EYE[f"{f}_{STRAND_EYE[k]}"], float)
    if mid is None:
        kin = json.loads(kin_file().read_text()) if kin_file().exists() else {}
        mid = kin.get("liner_mid", {}).get(f"{f}_{k}")
    if mid is None or mid is False:
        return [e, e2, h]
    mid = np.asarray(mid, float).reshape(-1, 3)
    return [e, e2, *mid, h]


def optimise_liners(seed_trials=6):
    """two free midpoints per liner (entry stub -> M1 -> M2 -> palm plate hole): keep every pair of liner centrelines
    >= 2.4 mm apart (Ø2 PTFE + 0.4 printed wall), stay inside the palm (|y| <= 11.5, z between the plate and the
    module), and minimise the total bend (PTFE friction ~ e^(-0.1 bend))."""
    from scipy.optimize import minimize
    E, H, names = [], [], []
    for f in FINGERS:
        ZM, ent = module_geom(f)
        for k, (x, y) in ent.items():
            p = _liner_path(f, k, (x, y, ZM - 4.0), mid=False)
            E.append(p[1])
            H.append(p[-1])
            names.append(f"{f}_{k}")
    E, H = np.array(E), np.array(H)
    n = len(names)
    T = np.linspace(0, 1, 6)
    iu = np.triu_indices(n, 1)
    d0 = np.array([0, 0, -1.0])
    Hup = np.array([0, 0, 1.0])

    def path_pts(M1, M2):
        segs = [(E, M1), (M1, M2), (M2, H)]
        P = [a[:, None, :] + (b - a)[:, None, :] * T[None, :, None] for a, b in segs]
        return np.concatenate([P[0], P[1][:, 1:], P[2][:, 1:]], 1)

    def ang(u, v):
        c = np.sum(u * v, -1) / np.linalg.norm(u, axis=-1) / np.linalg.norm(v, axis=-1)
        return np.degrees(np.arccos(np.clip(c, -1, 1)))

    def cost(v, wp):
        M = v.reshape(2, n, 3)
        M1, M2 = M[0], M[1]
        Pp = path_pts(M1, M2)
        D = np.linalg.norm(Pp[:, None, :, None, :] - Pp[None, :, None, :, :], axis=-1).min(axis=(2, 3))
        pen = np.clip(2.5 - D[iu], 0, None)
        bend = ang(np.tile(d0, (n, 1)), M1 - E) + ang(M1 - E, M2 - M1) + ang(M2 - M1, H - M2)
        box = sum(np.clip(np.abs(Mk[:, 1]) - 11.5, 0, None) ** 2 + np.clip(Mk[:, 2] - (E[:, 2] - 2), 0, None) ** 2
                  + np.clip(34.0 - Mk[:, 2], 0, None) ** 2 for Mk in (M1, M2))
        order = np.clip(M2[:, 2] - M1[:, 2] + 2.0, 0, None) ** 2  # M1 above M2
        return wp * np.sum(pen ** 2) + 0.001 * np.sum(bend ** 2) + 50.0 * np.sum(box) + 50.0 * np.sum(order)
    del Hup
    rng = np.random.default_rng(3)
    best = None
    for trial in range(seed_trials):
        M1 = E + (H - E) * 0.35 + (rng.normal(0, 2.5, E.shape) if trial else 0)
        M2 = E + (H - E) * 0.7 + (rng.normal(0, 2.5, E.shape) if trial else 0)
        v = np.concatenate([M1.ravel(), M2.ravel()])
        for wp in (50.0, 500.0, 5000.0):
            r = minimize(cost, v, args=(wp,), method="L-BFGS-B", options=dict(maxiter=800))
            v = r.x
        if best is None or r.fun < best.fun:
            best = r
    M = best.x.reshape(2, n, 3)
    kin = json.loads(kin_file().read_text())
    kin["liner_mid"] = {nm: [[float(c) for c in M[0][i]], [float(c) for c in M[1][i]]] for i, nm in enumerate(names)}
    kin_file().write_text(json.dumps(kin, indent=1, default=float))
    return best.fun


def module_geom(f):
    m = finger_module(f)
    ZM = m.Z_ABD - 6.5  # floor top (finger-local)
    ent = {}
    for k, s in m.SDEF.items():
        ent[k] = (s["x"][1], float(s["nip"][0]))
    ent["abd_flex"] = (m.R, m.Y_ABD - 1.0)
    ent["abd_ext"] = (-m.R, m.Y_ABD + 1.0)
    return ZM, ent


def lines(q):
    """tendon and sensor-wire polylines in the hand frame (mm)."""
    out = {}
    for f in FINGERS:
        m = finger_module(f)
        ql = _q(f, q)
        ZM, ent = module_geom(f)
        Th = _T(f)
        for k in m.SDEF:
            _, _, P3 = m.strand(k, ql, with_pts=True)
            P3 = P3[P3[:, 2] >= ZM - 1e-6] if (P3[:, 2] >= ZM).sum() > 1 else P3
            x, y = ent[k]
            path = [p + MCP[f] for p in P3] + [np.array([x, y, ZM]) + MCP[f]] + _liner_path(f, k, (x, y, ZM - 4.0))
            out[f"{f}_{k}"] = np.array(path)
        for role in ("flex", "ext"):
            _, _, P = m.abd_strand(role, ql, with_pts=True)
            Ta = m.mat_rot((0, 1, 0), ql["abd"], (0, 0, m.Z_ABD))
            P = m.xf(Ta, P[:-1])
            k = f"abd_{role}"
            x, y = ent[k]
            path = [p + MCP[f] for p in P] + [np.array([x, y, ZM]) + MCP[f]] + _liner_path(f, k, (x, y, ZM - 4.0))
            out[f"{f}_{k}"] = np.array(path)
        out[f"{f}_sensor_wire"] = m.sensor_wire(ql) + MCP[f]
    return out


# ------------------------------------------------------------------------------------------------ CAD
def build_module(f, neighbours):
    """knuckle module (printed, PA-CF or PETG; static in the palm) in finger-local coordinates."""
    m = finger_module(f)
    cq = m.cq
    ZM, ent = module_geom(f)
    hwL, hwR = neighbours  # half widths toward -x / +x (half the pitch, minus 0.15)
    Za = m.Z_ABD
    floor = m.box(-hwL, hwR, -15.9, 20.35, ZM - 4.0, ZM)
    post = m.U(m.box(-4.0, 4.0, -15.9, -12.9, ZM - 4.0, Za + 1.5), m.cyly(3.1, -15.9, -12.9, 0.0, Za))
    lug = m.box(-6.0, 6.0, 14.2, 20.35, ZM - 4.0, Za + 6.0)
    mod = m.U(floor, post, lug)
    mod = mod.cut(m.cyly(1.5, -16.5, -12.8, 0.0, Za))  # fixed palmar stub
    mod = mod.cut(m.cyly(m.FL_R + 0.5, 8.0, 14.1, 0.0, Za))  # abd disc (rotates about its own axis: invariant)
    mod = mod.cut(m.cyly(m.SEAT63 / 2, 14.1, 16.7, 0.0, Za)).cut(m.cyly(2.4, 16.6, 18.3, 0.0, Za))
    mod = mod.cut(m.cyly(m.CB / 2, 18.15, 20.5, 0.0, Za)).cut(m.cyly(4.3, 19.6, 20.5, 0.0, Za))
    for k, (x, y) in ent.items():
        mod = mod.cut(m.cylz(LINER["od"] / 2 + 0.05, ZM - 4.5, ZM - 0.8, x, y))  # liner socket
        mod = mod.cut(m.cylz(0.75, ZM - 1.0, ZM + 0.1, x, y))  # strand passage
        mod = mod.cut(cq.Workplane(obj=cq.Solid.makeCone(0.75, 1.5, 0.75, m.V(x, y, ZM - 0.75), m.V(0, 0, 1))))  # flared mouth
    # yoke swing clearance (abd +-15) above the floor
    return mod


def build_liners(f):
    m = finger_module(f)
    ZM, ent = module_geom(f)
    out = {}
    for k, (x, y) in ent.items():
        pts = _liner_path(f, k, (x, y, ZM - 4.0))
        pts = [p - MCP[f] for p in pts]  # local
        w = None
        for a, b in zip(pts[:-1], pts[1:]):
            seg = m.cyl(LINER["od"] / 2, a, b)
            w = seg if w is None else w.union(seg)
        for a in pts[1:-1]:
            w = w.union(m.cq.Workplane(obj=m.cq.Solid.makeSphere(LINER["od"] / 2, m.V(*a))))
        for a, b in zip(pts[:-1], pts[1:]):
            d = (b - a) / np.linalg.norm(b - a)
            w = w.cut(m.cyl(LINER["id"] / 2, a - 0.6 * d, b + 0.6 * d))
        out[f"liner_{k}"] = w
    return out


def build(export=True, checks=True):
    import cadquery as cq
    import trimesh
    t0 = time.time()
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "stl").mkdir(exist_ok=True)
    xs = {f: MCP[f][0] for f in FINGERS}
    order = sorted(FINGERS, key=lambda f: xs[f])
    nb = {}
    for i, f in enumerate(order):
        lo = (xs[f] - xs[order[i - 1]]) / 2 - 0.15 if i > 0 else 9.6
        hi = (xs[order[i + 1]] - xs[f]) / 2 - 0.15 if i + 1 < len(order) else 9.6
        nb[f] = (min(9.6, lo), min(9.6, hi))
    allparts, info, rep = {}, {}, {"fingers": {}}
    for f in FINGERS:
        finger_module(f)
    rep["liner_opt_cost"] = optimise_liners()
    for f in FINGERS:
        m = finger_module(f)
        P_ = m.build_all()
        P_ = {k: v for k, v in P_.items() if k != "base" and not k.startswith(("abd_brg_d_out",))}
        mod = build_module(f, nb[f])
        P_["knuckle_module"] = ("base", mod, "base", True)
        bi, bo = m.bearing(m.MR63, 14.2, 16.7, 0.0, m.Z_ABD, axis="y")
        P_["abd_brg_d_out"] = ("base", bo, "bearing", False)
        for k, w in build_liners(f).items():
            P_[k] = ("base", w, "liner", False)
        bar, _ = m.bar_shape(dict(abd=0, mcp=0, pip=0))
        P_["dip_bar"] = ("bar", bar, "bar", False)
        # per-finger checks (finger-local): validity + interference at a few poses
        fr = dict(links=m.LINK, lengths=LEN[f], pad_mid=bool(m.PAD_MID), tip_shift=getattr(m, "TS", 0.0))
        if checks:
            bad = [k for k, (b, w, c, pr) in P_.items() if not w.val().isValid() or len(w.solids().vals()) != 1]
            fr["invalid_or_multi"] = bad
            hits = {}
            for q in (dict(abd=0, mcp=0, pip=0), dict(abd=15, mcp=45, pip=50), dict(abd=-15, mcp=45, pip=50),
                      dict(abd=0, mcp=90, pip=95), dict(abd=15, mcp=90, pip=95), dict(abd=-15, mcp=0, pip=0)):
                pp = m.posed({k: v for k, v in P_.items() if k != "dip_bar"}, q)
                hits[json.dumps(q)] = m.interference(pp)
            fr["interference"] = hits
        rep["fingers"][f] = fr
        tr = tuple(MCP[f])
        for k, (body, w, col, pr) in P_.items():
            name = f"{f}_{k}"
            allparts[name] = w.translate(tr)
            info[name] = dict(body=f"{f}_{'module' if body == 'base' else body}", finger=f, stl=f"stl/{name}.stl",
                              color=m.COLORS.get(col, "#999999"), printed=bool(pr) or k == "knuckle_module")
        print(f"{f}: built ({time.time() - t0:.0f} s)", fr.get("invalid_or_multi"),
              {k: v for k, v in fr.get("interference", {}).items() if v})
    (OUT / "parts.json").write_text(json.dumps(info, indent=0))
    global _PARTS
    _PARTS = info
    if export:
        for name, w in allparts.items():
            cq.exporters.export(w, str(OUT / "stl" / f"{name}.stl"), tolerance=0.03, angularTolerance=0.15)
        for name in [n for n, i in info.items() if i["printed"]]:
            cq.exporters.export(allparts[name], str(OUT / f"{name}.step"))
        sc = trimesh.Scene()
        for name, i in info.items():
            mm = trimesh.load(str(OUT / "stl" / f"{name}.stl"))
            h = i["color"].lstrip("#")
            mm.visual.face_colors = np.tile([int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16), 255], (len(mm.faces), 1))
            sc.add_geometry(mm, node_name=name, geom_name=name)
        L0 = lines({})
        for k, Pl in L0.items():
            segs = [trimesh.creation.cylinder(radius=0.25 if "sensor" not in k else 0.4, segment=[a, b], sections=6)
                    for a, b in zip(Pl[:-1], Pl[1:]) if np.linalg.norm(b - a) > 1e-3]
            mm = trimesh.util.concatenate(segs)
            mm.visual.face_colors = np.tile((214, 39, 40, 255) if "sensor" in k else (30, 30, 30, 255), (len(mm.faces), 1))
            sc.add_geometry(mm, node_name="line_" + k, geom_name="line_" + k)
        sc.export(str(OUT / "fingers_q0.glb"))
    if checks:
        rep["neighbours"] = neighbour_checks(allparts, info)
        rep["liners"] = liner_checks()
        rep["module_width"] = {f: dict(toward_minus_x=nb[f][0], toward_plus_x=nb[f][1]) for f in FINGERS}
    rep["build_s"] = time.time() - t0
    (OUT / "checks.json").write_text(json.dumps(rep, indent=1, default=float))
    return rep


def neighbour_checks(allparts, info):
    """interference between neighbouring fingers (printed bodies + caps) at parallel abduction."""
    import cadquery as cq  # noqa: F401
    big = ("yoke", "proximal", "middle", "distal", "knuckle_module", "dip_bar", "sensor_pip_cap", "sensor_dip_cap",
           "sensor_mcp_cap", "pin_A", "pin_B", "tip_skin")
    order = sorted(FINGERS, key=lambda f: MCP[f][0])
    res = {}
    for qa, qm, qp in ((0, 0, 0), (15, 0, 0), (-15, 0, 0), (0, 90, 95), (15, 45, 50), (-15, 45, 50), (15, 90, 95), (-15, 90, 95)):
        q = {}
        for f in FINGERS:
            q.update({f"{f}_mcp_abd": qa, f"{f}_mcp_flex": qm, f"{f}_pip": qp})
        T = pose(q)
        sh = {}
        for n, i in info.items():
            if n.split("_", 1)[1] in big:
                M = T[n]
                w = allparts[n]
                R_, t_ = M[:3, :3], M[:3, 3]
                from OCP.gp import gp_Trsf
                from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
                import cadquery as cqm
                tr = gp_Trsf()
                tr.SetValues(*[float(v) for v in (R_[0, 0], R_[0, 1], R_[0, 2], t_[0], R_[1, 0], R_[1, 1], R_[1, 2], t_[1],
                                                  R_[2, 0], R_[2, 1], R_[2, 2], t_[2])])
                sh[n] = (i["finger"], cqm.Shape.cast(BRepBuilderAPI_Transform(w.val().wrapped, tr, True).Shape()))
        hits = []
        for fa, fb in zip(order[:-1], order[1:]):
            A = {n: s for n, (f, s) in sh.items() if f == fa}
            B = {n: s for n, (f, s) in sh.items() if f == fb}
            for na, sa in A.items():
                ba = sa.BoundingBox()
                for nb_, sb in B.items():
                    bb = sb.BoundingBox()
                    if ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin:
                        continue
                    try:
                        v = sa.intersect(sb).Volume()
                    except Exception:
                        v = 0.0
                    if v > 0.05:
                        hits.append((na, nb_, round(v, 2)))
        res[f"abd{qa}_mcp{qm}_pip{qp}"] = hits
        print("neighbours", qa, qm, qp, hits)
    return res


def liner_checks():
    L = {}
    for f in FINGERS:
        ZM, ent = module_geom(f)
        for k, (x, y) in ent.items():
            L[f"{f}_{k}"] = np.array(_liner_path(f, k, (x, y, ZM - 4.0)))
    nseg = {n: len(v) - 1 for n, v in L.items()}

    def segdist(p1, p2, q1, q2):
        best = 1e9
        for t in np.linspace(0, 1, 41):
            a = p1 + (p2 - p1) * t
            d = q2 - q1
            s = np.clip(np.dot(a - q1, d) / np.dot(d, d), 0, 1)
            best = min(best, np.linalg.norm(a - (q1 + s * d)))
        return best
    names = list(L)
    close = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            dmin = min(segdist(L[a][k], L[a][k + 1], L[b][j], L[b][j + 1]) for k in range(nseg[a]) for j in range(nseg[b]))
            if dmin < LINER["od"] + 0.2:
                close.append((a, b, round(dmin, 2)))
    bends = {}
    for n, Pl in L.items():
        tot = 0.0
        for k in range(len(Pl) - 2):
            d1, d2 = Pl[k + 1] - Pl[k], Pl[k + 2] - Pl[k + 1]
            tot += math.degrees(math.acos(np.clip(np.dot(d1, d2) / np.linalg.norm(d1) / np.linalg.norm(d2), -1, 1)))
        bends[n] = round(tot, 1)
    lengths = {n: round(float(np.linalg.norm(np.diff(Pl, axis=0), axis=1).sum()), 1) for n, Pl in L.items()}
    return dict(pairs_closer_than_2p2=close, n_close=len(close), entry_bend_deg=bends, length_mm=lengths,
                bend_median=float(np.median(list(bends.values()))), bend_max=max(bends.values()))


def render_poses(poses=None):
    """renders from the exported STLs, posed with pose(q), with lines(q) (tendons black, sensor wires red)."""
    import trimesh
    sys.path.insert(0, str(V2CAD))
    from tools.render import render
    info = parts()
    meshes = {n: trimesh.load(str(OUT / i["stl"])) for n, i in info.items()}
    poses = poses or {
        "open": {},
        "fist": {f"{f}_{j}": v for f in FINGERS for j, v in (("mcp_flex", 80), ("pip", 90))},
        "spread": {"index_mcp_abd": 12, "middle_mcp_abd": 0, "ring_mcp_abd": -8, "little_mcp_abd": -15},
    }
    for tag, q in poses.items():
        T = pose(q)
        d, c = {}, {}
        for n, mm in meshes.items():
            d[n] = mm.copy().apply_transform(T[n])
            c[n] = info[n]["color"]
        for k, Pl in lines(q).items():
            segs = [trimesh.creation.cylinder(radius=0.3 if "sensor" in k else 0.22, segment=[a, b], sections=6)
                    for a, b in zip(Pl[:-1], Pl[1:]) if np.linalg.norm(b - a) > 1e-3]
            d["line_" + k] = trimesh.util.concatenate(segs)
            c["line_" + k] = "#d62728" if "sensor" in k else "#202020"
        render(d, str(OUT / f"fingers_{tag}.png"), views=("iso", "front", "side", "top"), colors=c, size=(1800, 1400),
               title=f"18 fingers + knuckle modules + palm liners: {tag}", edges=False)


if __name__ == "__main__":
    r = build()
    render_poses()
    print(json.dumps({k: v for k, v in r.items() if k in ("liners", "module_width")}, default=float)[:3000])
