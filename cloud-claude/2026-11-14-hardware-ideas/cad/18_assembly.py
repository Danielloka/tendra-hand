"""18 · Full V2.1 hand: palm, W13 wrist, forearm, assembly, interference, viewer export (study 2026-11-16, package C).

Bodies (hand frame, mm: origin = wrist centre, +Z fingers, -Y palm side, +X thumb side):
- forearm (fixed): W13 forearm yoke, forearm plate + carrier, wrist drive (V2), shell, frame, rail, 16 servos +
  16 spool cartridges (report 02) in report 13's slot map, electronics, arm flange.
- ring (Ry(dev)): W13 ring, eye plate, dev pins, flex bushes.
- palm (Ry(dev) Rx(flex)): W13 palm yoke + palm plate + flex pins, the new palm (`palm_v21`), hub tray + board,
  palm pads + pad cups; fingers and thumb hang off the palm with their own joint transforms.
Fingers / thumb: package A (`cad/18_fingers.py`, out/18/fingers/) and B (`cad/18_thumb.py`, out/18/thumb/) when
they exist (pose(q) API), otherwise the V2 parts as placeholders (said so in checks.json and report 18).

Outputs (out/18/): hand_v21.glb (all parts at q = 0, node = part name), poses.json, checks.json, parts STL for the new
parts, renders.

    uv run --no-project --with cadquery --with numpy --with trimesh --with scipy --with rtree --with manifold3d \
        --with matplotlib --with pillow python cad/18_assembly.py [--quick]
"""

from __future__ import annotations

import importlib.util
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
STUDY = HERE.parent
V2 = STUDY.parent / "2026-11-09-v2-hand-cad"
V2CAD = V2 / "cad"
V2STL = V2 / "out" / "assembly" / "stl"
OUT = STUDY / "out" / "18"
OUT13 = STUDY / "out" / "13"
sys.path.insert(0, str(V2CAD))
QUICK = "--quick" in sys.argv


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


W13 = _load("w13", HERE / "13_wrist_parts.py")  # patches V2 wrist constants to W13 (Ø5 pins, bushes, ring)
KIN = W13.KIN
W = W13.W
import params as P  # noqa: E402  (V2)
# Package A moved ring/little to an even 20 mm pitch (x -20 / -40; V2: -19.5 / -38). Match the palm to it.
for _f, _x in (("ring", -20.0), ("little", -40.0)):
    _m = list(P.FINGERS[_f]["mcp"]); _m[0] = _x; P.FINGERS[_f]["mcp"] = type(P.FINGERS[_f]["mcp"])(_m)

FINGERS = ("index", "middle", "ring", "little")
DENS = {"PETG": 1.27, "PAHT-CF": 1.20, "7075": 2.81, "steel": 7.85, "stainless": 7.95, "silicone": 1.08, "TPU": 1.21,
        "servo": None, "PCB": 1.9, "brass": 8.5, "bronze": 8.8, "iglidur": 1.46}
COL = {"PETG": "#d9dde3", "PAHT-CF": "#3a3a3a", "7075": "#8fa3b8", "steel": "#b5b9bd", "stainless": "#c8ccd2",
       "silicone": "#e9b8a0", "TPU": "#e2a07a", "servo": "#1f3c66", "PCB": "#1e7d3a", "brass": "#c9a646", "bronze": "#b07a3a",
       "iglidur": "#e0c020"}


class Part:
    def __init__(self, name, mesh, body, material, group, placeholder=False):
        self.name, self.mesh, self.body, self.material, self.group = name, mesh, body, material, group
        self.placeholder = placeholder


def tm(shape, tol=0.1):
    from tools.render import tessellate

    v, f = tessellate(shape, tol, 0.3)
    m = trimesh.Trimesh(v, f, process=True)
    m.merge_vertices()
    return m


def stl(path):
    m = trimesh.load(str(path), force="mesh")
    m.merge_vertices()
    return m


def box(x0, x1, y0, y1, z0, z1):
    return W13.box(x0, x1, y0, y1, z0, z1)


# =====================================================================================================================
# Palm (new): V2 palm envelope with the W13 heel keep-out, hollow shell, knuckle-module seats, thumb-mount bore,
# palm-plate seat clearance, hub tray, pad cups
# =====================================================================================================================
PALM = dict(x=(-47.0, 30.0), y=(-14.0, 14.0), wall=2.5, z_base=15.0, mod_below_mcp=23.6)
HUB = dict(board=(32.0, 26.0, 1.6), mcu=(23.5, 18.0, 3.5), tray=(-18.0, 18.0, 8.5, 13.5, 28.0, 58.0))
PAD_CUPS = {"thenar": (14.0, 44.0), "hypothenar": (-40.0, 38.0), "centre": (-12.0, 44.0), "distal": (-12.0, 62.0)}


def make_palm():
    import cadquery as cq

    # top outline in the xz plane: under each knuckle module (MCP z - 22), sloped between fingers
    xs = []
    for f in ("little", "ring", "middle", "index"):
        mx, _, mz = P.FINGERS[f]["mcp"]
        top = mz - PALM["mod_below_mcp"]
        xs += [(mx - 9.9, top), (mx + 9.9, top)]
    x0, x1 = PALM["x"]
    prof = [(x0, PALM["z_base"]), (x1, PALM["z_base"]), (x1, xs[-1][1])] + xs[::-1][1:] + [(x0, xs[0][1])]
    prof = [(min(max(x, x0), x1), z) for x, z in prof]
    prof = [pt for i, pt in enumerate(prof) if i == 0 or abs(pt[0] - prof[i - 1][0]) + abs(pt[1] - prof[i - 1][1]) > 1e-6]
    if abs(prof[0][0] - prof[-1][0]) + abs(prof[0][1] - prof[-1][1]) < 1e-6:
        prof = prof[:-1]
    outer = cq.Workplane("XZ", origin=(0, PALM["y"][1], 0)).polyline(prof).close().extrude(PALM["y"][1] - PALM["y"][0])
    try:  # round the outer corners only (the 0.2 mm steps between knuckle seats are too short to fillet)
        outer = outer.edges("|Y").edges(">X or <X").fillet(3.0)
    except Exception:
        pass
    # V2 heel keep-out (report 03 / 13): applied with V2's palm_proxy cutter (palm box minus the proxy)
    proxy = W.palm_proxy(notch=True)
    big = box(-47, 30, PALM["y"][0], PALM["y"][1], PALM["z_base"], 60)
    heel_cut = big.cut(proxy)
    outer = outer.cut(heel_cut)
    # hollow shell (open at the base for the wrist, open at the back for the hub tray / dorsal cover)
    w = PALM["wall"]
    inner = cq.Workplane("XZ", origin=(0, PALM["y"][1] - w, 0)).polyline(
        [(x + (w if x == x0 else -w if x == x1 else 0), z - w if z > PALM["z_base"] + 1 else z - 1) for x, z in prof]
    ).close().extrude(PALM["y"][1] - PALM["y"][0] - 2 * w)
    palm = outer.cut(inner)
    # W13 palm-plate seat + legs pass up through the base: clearance pocket (|x| 14.6, |y| 9.8, z 14..30.6)
    palm = palm.cut(box(-14.6, 14.6, -9.8, 9.8, 13.0, 30.8))
    palm = palm.cut(box(-22.0, 22.0, -12.6, 12.6, 9.9, PALM["z_base"] + 0.05))  # the yoke flange (z 10..15) sits under
    for bx, by in W.FLANGE_BOLTS:  # 4 x M3 down into the yoke flange
        palm = palm.cut(W13.cyl((bx, by, PALM["z_base"] - 1), (bx, by, PALM["z_base"] + 6), 1.7))
    # strand/liner openings in the palm top under each knuckle module (package A's liners enter the module bottom)
    for f in FINGERS:
        mx, _, mz = P.FINGERS[f]["mcp"]
        top = mz - PALM["mod_below_mcp"]
        palm = palm.cut(box(mx - 8.5, mx + 8.5, -11.5, 11.5, top - 4.0, top + 1.0))
    # the thumb bay (solid of revolution about cmc_rot) is cut as a mesh boolean in main(): OCC fails on it
    # hub tray pocket in the dorsal shell
    t = HUB["tray"]
    palm = palm.cut(box(t[0], t[1], t[2], PALM["y"][1] + 0.1, t[4], t[5]))
    # pad cups 8 x 10 x 4 (BMP581) in the palmar face
    for _, (x, z) in PAD_CUPS.items():
        palm = palm.cut(box(x - 4.0, x + 4.0, PALM["y"][0] - 0.1, PALM["y"][0] + 4.0, z - 5.0, z + 5.0))
    return palm


def thumb_bay(clear=1.0, step=1.5):
    import cadquery as cq

    a = np.array(P.THUMB_AXES["cmc_rot"], float)
    a /= np.linalg.norm(a)
    Pc = np.array(P.THUMB_P, float)
    names = ["T_thumb_base", "T_bearing_top", "T_bearing_bottom", "T_thumb_mount", "T_base_cap"]
    if (OUT / "thumb" / "parts.json").exists():
        names = []  # package B: its own bay is cut by its parts check (kept simple here)
    ms = [trimesh.load(str(V2STL / f"{n}.stl"), force="mesh") for n in names if (V2STL / f"{n}.stl").exists()]
    V = np.vstack([np.vstack([m.vertices, m.sample(30000)]) for m in ms])  # surface samples: a bearing has vertices only at its faces
    d = V - Pc
    sax = d @ a
    r = np.linalg.norm(d - sax[:, None] * a, axis=1)
    bins = np.arange(sax.min() - step, sax.max() + 2 * step, step)
    prof = []
    for b0, b1 in zip(bins[:-1], bins[1:]):
        m = (sax >= b0) & (sax < b1)
        if m.any():
            rr = r[m].max() + clear
            prof += [(rr, b0), (rr, b1)]
    pts = [(0.0, prof[0][1])] + prof + [(0.0, prof[-1][1])]
    body = cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0))
    z = np.array([0, 0, 1.0])
    ax = np.cross(z, a)
    ang = math.degrees(math.acos(float(np.clip(z @ a, -1, 1))))
    body = body.rotate((0, 0, 0), tuple(ax / np.linalg.norm(ax)), ang).translate(tuple(Pc))
    return body


def make_hub():
    b, m, t = HUB["board"], HUB["mcu"], HUB["tray"]
    yb = t[2] + 0.6
    board = box(-b[0] / 2, b[0] / 2, yb, yb + b[2], 43.0 - b[1] / 2, 43.0 + b[1] / 2)
    mcu = box(-m[0] / 2, m[0] / 2, yb + b[2], yb + b[2] + m[2], 43.0 - m[1] / 2, 43.0 + m[1] / 2)
    tray = box(t[0], t[1], t[2], t[2] + 0.6, t[4], t[5])
    cover = box(t[0] - 1, t[1] + 1, PALM["y"][1] - 1.2, PALM["y"][1], t[4] - 1, t[5] + 1)
    return {"hub_tray": (tray, "PETG"), "hub_board": (board, "PCB"), "hub_rp2040": (mcu, "PCB"),
            "palm_dorsal_cover": (cover, "PETG")}


def make_pad_cups():
    out = {}
    for n, (x, z) in PAD_CUPS.items():
        out[f"pad_cup_{n}"] = (box(x - 3.9, x + 3.9, PALM["y"][0] + 0.1, PALM["y"][0] + 3.9, z - 4.9, z + 4.9), "silicone")
    return out


# =====================================================================================================================
# Forearm: report 02 spool cartridges in report 13's slot map; V2 stagger kept (level 2 on the rail, level 1 on posts)
# =====================================================================================================================
SCS = dict(L=23.3, W=12.1, H=25.25, shaft_off=5.7)
RAIL = 3.0  # rail half thickness
LV = {1: dict(z=-141.0, face=9.5), 2: dict(z=-175.0, face=RAIL)}  # cartridge rail-face |x| per level
CART_W = 14.3
COLS = (-20.25, -6.75, 6.75, 20.25)
SHELL_HALF_X = 52.5  # V2 47.5: +5 per side for the 14.3 mm cartridges (synthesis open item)


def slot_geom(sid_slot):
    import forearm as F  # V2

    side, level, col = F.SLOTS[sid_slot]
    sx = 1 if side == "R" else -1
    return sx, level, COLS[col]


def make_cartridge_parts():
    """Cartridge meshes from report 02 (its frame: axle along +X from the rail face x 0 to the dog face 14.3)."""
    out = {}
    for n in ("spool_02", "driver_02", "keeper_02"):
        out[n] = stl(STUDY / "out" / "02" / f"{n}.stl")
    return out


def place_x(mesh, sx, x_face, y, z):
    m = mesh.copy()
    M = np.eye(4)
    if sx < 0:
        M[0, 0] = -1  # mirror in x (left side): flip with a rotation of 180 deg about Z keeps handedness
        M[1, 1] = -1
    m.apply_transform(M)
    m.apply_translation([sx * x_face, y, z])
    return m


def make_servo(sx, x_face, y, z):
    s = SCS
    body = box(0, s["H"] - 4.0, -s["W"] / 2, s["W"] / 2, -s["L"] + s["shaft_off"], s["shaft_off"])
    tabs = box(s["H"] - 4.0 - 16.8 + 8.0, s["H"] - 4.0 - 16.8 + 9.6, -s["W"] / 2, s["W"] / 2, -s["L"] + s["shaft_off"] - 4.6,
               s["shaft_off"] + 4.6)
    spline = W13.cyl((-4.0, 0, 0), (0.0, 0, 0), 2.5)
    m = tm(body.union(tabs).union(spline))
    M = np.eye(4)
    if sx < 0:
        M[0, 0] = -1
        M[1, 1] = -1
    m.apply_transform(M)
    m.apply_translation([sx * (x_face + 4.0), y, z])
    return m


def make_forearm(slots):
    import forearm as F

    F.SHELL = (SHELL_HALF_X, 33.0, 2.0)
    F.SHELL_TOP = (36.0, 29.5)
    parts = {}
    back, front, sides = F.make_shell()
    parts["forearm_shell_back"] = (tm(back), "PETG")
    parts["forearm_cover_front"] = (tm(front), "PETG")
    for k, v in sides.items():
        parts[f"forearm_side_cover_{k}"] = (tm(v), "PETG")
    wi = W.interface()
    parts["forearm_frame"] = (tm(F.make_frame(wi)), "PAHT-CF")
    parts["arm_flange"] = (tm(F.make_arm_flange()), "PAHT-CF")
    for n in ("esp32_s3", "fe_urt1", "power_board"):
        parts[n] = (stl(V2STL / f"F_{n}.stl"), "PCB")
    # spool rail (PAHT-CF) with level-1 standoff posts (Ø6.4) carrying the outer cartridges' rail faces
    rail = box(-RAIL, RAIL, -27.5, 27.5, -199.0, -128.0)
    for sx in (-1, 1):
        for y in COLS:
            x0, x1 = sorted((sx * RAIL, sx * LV[1]["face"]))
            rail = rail.union(W13.cyl((x0, y, LV[1]["z"]), (x1, y, LV[1]["z"]), 3.2))
    parts["spool_rail"] = (tm(rail), "PAHT-CF")
    cart = make_cartridge_parts()
    for sid, slot in slots.items():
        sx, level, y = slot_geom(slot)
        z = LV[level]["z"]
        face = LV[level]["face"]
        nm = f"{int(sid):02d}"
        parts[f"cart_{nm}_spool"] = (place_x(cart["spool_02"], sx, face, y, z), "PETG")
        parts[f"cart_{nm}_driver"] = (place_x(cart["driver_02"], sx, face, y, z), "PETG")
        parts[f"cart_{nm}_keeper"] = (place_x(cart["keeper_02"], sx, face, y, z), "PETG")
        parts[f"servo_{nm}"] = (make_servo(sx, face + CART_W, y, z), "servo")
    return parts


# =====================================================================================================================
# Fingers and thumb: package A / B if present, else V2 placeholders
# =====================================================================================================================
def _pkg_parts(mod, sub):
    """Package A/B contract: mod.parts() -> {name: {"body", "stl", ...}}; STL paths relative to out/18/<sub>/ or absolute."""
    out = {}
    for n, d in mod.parts().items():
        pth = Path(d["stl"])
        if not pth.is_absolute():
            pth = OUT / sub / pth if (OUT / sub / pth).exists() else STUDY / pth
        body = d.get("body", n)
        mat = d.get("material") or ("silicone" if any(t in n for t in ("skin", "pad")) else
                                    "steel" if any(t in n for t in ("pin", "axle", "bearing", "dowel", "bar", "liner_steel")) else
                                    "PCB" if any(t in n for t in ("pcb", "board", "chip")) else "PETG")
        out[n] = (stl(pth), f"{d.get('finger', sub)}:{body}", mat)
    return out


class FingerSource:
    """Uniform access: parts (name -> (mesh at q0, body, material)), T(q) -> {name: 4x4 hand frame}, lines(q)."""

    def __init__(self):
        self.kind = "V2 placeholder"
        self.mod = None
        a = HERE / "18_fingers.py"
        if a.exists() and (OUT / "fingers" / "parts.json").exists():
            try:
                self.mod = _load("fingers18", a)
                self.kind = "package A (cad/18_fingers.py)"
            except Exception as e:  # pragma: no cover
                print("package A import failed:", e)
                self.mod = None
        if self.mod is None:
            import finger as FG  # V2

            self.FG = FG
            self.parts = {}
            for f in FINGERS:
                for p in V2STL.glob(f"{f}_*.stl"):
                    part = p.stem[len(f) + 1:]
                    if part.startswith(("ptfe", "eyelet")):
                        continue
                    body = FG.BODY_OF.get(part)
                    if body is None:
                        body = "yoke" if "abd" in part else "proximal"
                    mat = "silicone" if part.startswith("pad") else ("steel" if part.startswith(("pin", "post", "bar", "bearing", "tie")) else "PETG")
                    self.parts[p.stem] = (stl(p), f"{f}:{body}", mat)
            self.g = {f: FG.geom(f) for f in FINGERS}
            self.T0 = {f: FG.body_transforms(self.g[f], FG.full_q(self.g[f], {})) for f in FINGERS}
        else:
            self.parts = _pkg_parts(self.mod, "fingers")

    @staticmethod
    def flat(q):
        return {f"{f}_{j}": float(v) for f, d in q.items() for j, v in d.items()}

    def T(self, q):
        if self.mod is not None:
            return self.mod.pose(self.flat(q))
        out = {}
        for f in FINGERS:
            Tq = self.FG.body_transforms(self.g[f], self.FG.full_q(self.g[f], q.get(f, {})))
            for n, (_, b, _) in self.parts.items():
                ff, body = b.split(":")
                if ff == f:
                    out[n] = Tq[body] @ np.linalg.inv(self.T0[f][body])
        return out

    def lines(self, q):
        if self.mod is not None and hasattr(self.mod, "lines"):
            return self.mod.lines(self.flat(q))
        return {}


class ThumbSource:
    def __init__(self):
        self.kind = "V2 placeholder"
        self.mod = None
        b = HERE / "18_thumb.py"
        if b.exists() and (OUT / "thumb" / "parts.json").exists():
            try:
                self.mod = _load("thumb18", b)
                self.kind = "package B (cad/18_thumb.py)"
            except Exception as e:  # pragma: no cover
                print("package B import failed:", e)
                self.mod = None
        if self.mod is None:
            import thumb as TH  # V2

            self.TH = TH
            self.parts = {}
            for p in V2STL.glob("T_*.stl"):
                part = p.stem[2:]
                body = TH.PART_BODY.get(part) or ("distal" if "_ip" in part else "proximal" if "mcp" in part else
                                                  "metacarpal" if "cmc" in part else "mount")
                if part.startswith(("nip", "neck")):
                    continue
                mat = "silicone" if "pad" in part else ("steel" if part.startswith(("pin", "bearing", "tie")) else "PETG")
                self.parts[p.stem] = (stl(p), f"thumb:{body}", mat)
        else:
            self.parts = _pkg_parts(self.mod, "thumb")

    @staticmethod
    def flat(q):
        return {(k if k.startswith("thumb_") else f"thumb_{k}"): float(v) for k, v in q.items()}

    def T(self, q):
        if self.mod is not None:
            try:
                return self.mod.pose(self.flat(q))
            except Exception:
                return self.mod.pose(q)
        # V2 STLs are exported at the V2 "open" pose (thumb at its 'flat' q): posed = F(q) F(q_flat)^-1
        qq = {**THUMB_FLAT, **q}
        F, F0 = self.TH.body_transforms(qq), self.TH.body_transforms(THUMB_FLAT)

        def M4(rt):
            M = np.eye(4)
            M[:3, :3], M[:3, 3] = rt
            return M

        out = {}
        for n, (_, b, _) in self.parts.items():
            body = b.split(":")[1]
            if body in F:
                out[n] = M4(F[body]) @ np.linalg.inv(M4(F0[body]))
            else:
                out[n] = np.eye(4)
        return out

    def lines(self, q):
        if self.mod is not None and hasattr(self.mod, "lines"):
            try:
                return self.mod.lines(self.flat(q))
            except Exception:
                return self.mod.lines(q)
        return {}


_TD = json.loads((V2 / "out" / "thumb" / "thumb_design.json").read_text())["eval"]
THUMB_FLAT = dict(zip(("cmc_rot", "cmc_flex", "mcp_flex", "ip"), _TD["flat"]["q_thumb"]))

# =====================================================================================================================
# Poses
# =====================================================================================================================
def poses_def():
    sol = json.load(open(V2 / "out" / "assembly" / "poses_solved.json"))
    relaxed = {"index": {"mcp_flex": 20, "pip": 30}, "middle": {"mcp_flex": 25, "pip": 35},
               "ring": {"mcp_flex": 30, "pip": 40}, "little": {"mcp_flex": 35, "pip": 45}}
    th_rel = {"cmc_rot": 20.0, "cmc_flex": 10.0, "mcp_flex": 10.0, "ip": 10.0}
    pw = sol["power55"]
    out = {
        "open": dict(f={}, t=dict(THUMB_FLAT), w=(0.0, 0.0)),
        "relaxed": dict(f=relaxed, t=th_rel, w=(0.0, 0.0)),
        "fist": dict(f=sol["fist"]["fingers"], t=sol["fist"]["thumb"], w=(0.0, 0.0)),
        "tip_pinch": dict(f={**{k: {"mcp_flex": 15.0, "pip": 20.0} for k in ("middle", "ring", "little")},
                             "index": sol["pinch_tip_pinch"]["finger_q"]}, t=sol["pinch_tip_pinch"]["thumb"],
                          w=(0.0, 0.0)),
        "thumb_to_little": dict(f={"little": sol["pinch_little"]["finger_q"], "ring": {"mcp_abd": 11.0, "mcp_flex": 30.0, "pip": 30.0},
                                   "middle": {"mcp_abd": 12.0, "mcp_flex": 30.0, "pip": 30.0},
                                   "index": {"mcp_abd": 13.0, "mcp_flex": 30.0, "pip": 30.0}},  # V2 + 2 deg (A's knuckles)
                                t=sol["pinch_little"]["thumb"], w=(0.0, 0.0)),
        "power_grasp": dict(f=pw["fingers"], t=pw.get("thumb", th_rel), w=(0.0, 0.0)),
        "spread": dict(f={"index": {"mcp_abd": 12}, "middle": {"mcp_abd": 4}, "ring": {"mcp_abd": -6}, "little": {"mcp_abd": -12}},
                       t={**THUMB_FLAT, "cmc_rot": -20.0}, w=(0.0, 0.0)),
        "wrist_flex_+40": dict(f={}, t=dict(THUMB_FLAT), w=(40.0, 0.0)),
        "wrist_dev_-20": dict(f={}, t=dict(THUMB_FLAT), w=(0.0, -20.0)),
    }
    return out


def flat_q(pd):
    q = {}
    for f, d in pd["f"].items():
        for j, v in d.items():
            q[f"{f}_{j}"] = float(v)
    for j, v in pd["t"].items():
        q[f"thumb_{j}"] = float(v)
    q["wrist_flex"], q["wrist_dev"] = pd["w"]
    return q


# =====================================================================================================================
# Main
# =====================================================================================================================
def main():
    t0 = time.time()
    OUT.mkdir(parents=True, exist_ok=True)
    k13 = json.load(open(OUT13 / "kin" / "eyeplate_kin.json"))
    slots = {int(k): int(v) for k, v in k13["slots"].items()}
    parts: dict[str, Part] = {}

    def add(name, mesh, body, mat, group, ph=False):
        parts[name] = Part(name, mesh, body, mat, group, ph)

    # --- W13 wrist (out/13) -----------------------------------------------------------------------------------
    w13 = {"forearm_yoke_w13": ("forearm", "7075"), "forearm_plate_w13": ("forearm", "stainless"),
           "forearm_plate_carrier_w13": ("forearm", "PAHT-CF"), "bush_dev_B": ("forearm", "iglidur"),
           "bush_dev_F": ("forearm", "iglidur"), "ring_w13": ("ring", "7075"), "eye_plate_w13": ("ring", "stainless"),
           "pin_dev_B_hollow": ("ring", "steel"), "pin_dev_F": ("ring", "steel"), "bush_flex_R": ("ring", "iglidur"),
           "bush_flex_L": ("ring", "iglidur"), "palm_yoke_w13": ("palm", "7075"), "palm_plate_w13": ("palm", "stainless"),
           "pin_flex_R_hollow": ("palm", "steel"), "pin_flex_L": ("palm", "steel")}
    for n, (b, m) in w13.items():
        add(n, stl(OUT13 / f"{n}.stl"), b, m, "wrist")
    for n in ("wrist_servo_R", "wrist_servo_L", "wrist_gears_R", "wrist_gears_L"):
        add(n, stl(V2STL / f"W_{n}.stl"), "forearm", "servo" if "servo" in n else "steel", "wrist")
    # --- palm ------------------------------------------------------------------------------------------------------
    import cadquery as cq

    palm = make_palm()
    cq.exporters.export(palm, str(OUT / "palm_v21_without_thumb_bay.step"))
    pm = trimesh.boolean.difference([tm(palm), tm(thumb_bay())], engine="manifold")
    pm.export(OUT / "palm_v21.stl")
    add("palm_v21", pm, "palm", "PETG", "palm")
    for n, (s, m) in {**make_hub(), **make_pad_cups()}.items():
        add(n, tm(s), "palm", m, "palm")
    for n in ("P_pad", "P_hypothenar_pad"):
        add(n.replace("P_", "palm_"), stl(V2STL / f"{n}.stl"), "palm", "silicone", "palm", True)
    print(f"palm + wrist in {time.time()-t0:.0f} s")
    # --- forearm ---------------------------------------------------------------------------------------------------
    for n, (m, mat) in make_forearm(slots).items():
        add(n, m, "forearm", mat, "forearm")
    print(f"forearm in {time.time()-t0:.0f} s")
    # --- fingers / thumb -------------------------------------------------------------------------------------------
    FS, TS = FingerSource(), ThumbSource()
    for n, (m, b, mat) in FS.parts.items():
        add(n, m, b, mat, "fingers", FS.mod is None)
    for n, (m, b, mat) in TS.parts.items():
        add(n, m, b, mat, "thumb", TS.mod is None)
    print(f"fingers: {FS.kind} ({len(FS.parts)} parts); thumb: {TS.kind} ({len(TS.parts)} parts); total {len(parts)}")
    # --- validity, mass ---------------------------------------------------------------------------------------------
    mass, bad = {}, []
    for p in parts.values():
        wt = p.mesh.is_watertight
        if not wt:
            bad.append(p.name)
        d = DENS.get(p.material)
        if p.material == "servo":
            g = 13.2 if p.name.startswith("servo_") else 55.0
        else:
            g = abs(p.mesh.volume) / 1000 * (d or 1.2) if wt else p.mesh.convex_hull.volume / 1000 * (d or 1.2) * 0.8
        mass[p.name] = round(g, 2)
    # --- poses -------------------------------------------------------------------------------------------------------
    PD = poses_def()
    pose_out = {"parts": {n: p.body for n, p in parts.items()}, "poses": {}}
    inter = {}
    names = list(parts)
    strands = KIN.load_strands()
    by = {r["name"]: np.array(r["eye"]) for r in k13["eye_table"]}
    eyes = [by[s["name"]] for s in strands]
    for pn, pd in PD.items():
        f, d = pd["w"]
        Rr, Rp = np.eye(4), np.eye(4)
        Rr[:3, :3], Rp[:3, :3] = KIN.R_ring(f, d), KIN.R_palm(f, d)
        Tf = FS.T(pd["f"])
        Tt = TS.T(pd["t"])
        T = {}
        for n, p in parts.items():
            if p.body == "forearm":
                M = np.eye(4)
            elif p.body == "ring":
                M = Rr
            elif p.group == "fingers":
                M = Rp @ Tf.get(n, np.eye(4))
            elif p.group == "thumb":
                M = Rp @ Tt.get(n, np.eye(4))
            else:
                M = Rp
            T[n] = M
        # wrist drive (pose dependent cranks/rods: rebuilt from V2/W13 wrist)
        b = W.build({"wrist_flex": f, "wrist_dev": d}, bundle=False)["parts"]
        dyn = {}
        for k in ("wrist_crank_R", "wrist_crank_L", "wrist_rod_R", "wrist_rod_L"):
            if k in b:
                dyn[k] = tm(b[k])
        # strand lines: wrist path from the 09/13 model + package A/B lines
        lines = {}
        for st, pl in zip(strands, KIN.fast_polylines(strands, eyes, (f, d))):
            lines[f"strand_{st['name']}"] = np.round(pl, 3).tolist()
        for src, Rb in ((FS.lines(pd["f"]), Rp), (TS.lines(pd["t"]), Rp)):
            for k, v in src.items():
                v = np.asarray(v, float)
                lines[k] = np.round(v @ Rb[:3, :3].T, 3).tolist()
        pose_out["poses"][pn] = {"q": flat_q(pd), "T": {n: np.round(M, 6).ravel().tolist() for n, M in T.items()},
                                 "lines": lines}
        if pn == "open":
            for k, m in dyn.items():
                add(k, m, "forearm", "steel", "wrist")
                T[k] = np.eye(4)
                mass[k] = round(abs(m.volume) / 1000 * 7.85, 2)
            pose_out["parts"].update({k: "forearm" for k in dyn})
            pose_out["poses"][pn]["T"].update({k: np.eye(4).ravel().tolist() for k in dyn})
        else:
            # crank / rod: per-pose meshes differ; give their transform as identity and note it (viewer: static)
            pose_out["poses"][pn]["T"].update({k: np.eye(4).ravel().tolist() for k in dyn})
        inter[pn] = interference(parts, T, dyn)
        print(f"  pose {pn:16s}: {len(inter[pn])} contacts > 1 mm3 ({time.time()-t0:.0f} s)")
    # --- exports -----------------------------------------------------------------------------------------------------
    export_glb(parts)
    (OUT / "poses.json").write_text(json.dumps(pose_out))
    total = sum(mass.values())
    groups = {}
    for n, p in parts.items():
        groups[p.group] = round(groups.get(p.group, 0) + mass[n], 1)
    checks = dict(fingers=FS.kind, thumb=TS.kind, part_count=len(parts),
                  part_count_by_group={g: sum(1 for p in parts.values() if p.group == g) for g in groups},
                  placeholders=sorted(n for n, p in parts.items() if p.placeholder),
                  not_watertight=bad, mass_g_total=round(total, 1), mass_g_by_group=groups, mass_g=mass,
                  interference=inter)
    (OUT / "checks.json").write_text(json.dumps(checks, indent=1, default=float))
    render(parts, PD)
    print(f"done in {time.time()-t0:.0f} s; parts {len(parts)}, mass {total:.0f} g")


SKIP_PAIRS = (("pin_", "bush_"), ("pin_dev", "ring_w13"), ("pin_flex", "palm_yoke"), ("bush_flex", "ring_w13"),
              ("bush_dev", "forearm_yoke"), ("eye_plate", "ring_w13"), ("palm_plate", "palm_yoke"),
              ("forearm_plate", "forearm_plate_carrier"), ("palm_yoke", "palm_v21"), ("hub_", "palm_v21"),
              ("hub_", "hub_"), ("palm_dorsal_cover", "palm_v21"), ("pad_cup", "palm_v21"), ("palm_pad", "palm_v21"),
              ("palm_hypothenar", "palm_v21"), ("palm_knuckle", "palm_v21"), ("cart_", "spool_rail"), ("cart_", "cart_"),
              ("cart_", "servo_"), ("servo_", "forearm_frame"), ("wrist_servo", "wrist_gears"), ("wrist_gears", "wrist_crank"),
              ("wrist_rod", "wrist_crank"), ("forearm_yoke", "forearm_frame"), ("forearm_plate_carrier", "forearm_frame"),
              ("forearm_shell", "forearm_frame"), ("forearm_cover", "forearm_frame"), ("forearm_side", "forearm_shell"),
              ("arm_flange", "forearm_"), ("wrist_servo", "forearm_frame"), ("wrist_gears", "forearm_frame"),
              ("wrist_crank", "forearm_frame"), ("esp32", "forearm_frame"), ("fe_urt", "forearm_frame"),
              ("power_board", "forearm_frame"), ("wrist_rod", "palm_yoke"))


def _skip(a, b):
    for x, y in ((a, b), (b, a)):  # package A liners seat on the palm plate / in the yoke (designed contact)
        if "_liner_" in x and y.startswith(("palm_plate", "palm_yoke")):
            return True
    for x, y in SKIP_PAIRS:
        if (a.startswith(x) and b.startswith(y)) or (a.startswith(y) and b.startswith(x)):
            return True
    return False


def interference(parts, T, dyn, thr=1.0):
    """Intersection volume between parts of DIFFERENT rigid bodies (finger/thumb internal pairs are package A/B's).
    Mesh booleans (manifold); bbox prefilter."""
    items = []
    for n, p in parts.items():
        if n in dyn:
            continue
        m = p.mesh.copy()
        m.apply_transform(T[n])
        items.append((n, p.body if p.group not in ("fingers", "thumb") else p.body.split(":")[0], m))
    for n, m in dyn.items():
        items.append((n, "drive", m))
    bb = np.array([np.r_[m.bounds[0], m.bounds[1]] for _, _, m in items])
    out = []
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            ni, bi, mi = items[i]
            nj, bj, mj = items[j]
            if bi == bj and bi not in ("drive",):
                continue
            if (bb[i, 3:] < bb[j, :3]).any() or (bb[j, 3:] < bb[i, :3]).any():
                continue
            if _skip(ni, nj):
                continue
            if not (mi.is_watertight and mj.is_watertight):
                continue
            try:
                v = trimesh.boolean.intersection([mi, mj], engine="manifold").volume
            except Exception:
                continue
            if v > thr:
                out.append((ni, nj, round(float(v), 1)))
    return sorted(out, key=lambda x: -x[2])


def export_glb(parts):
    sc = trimesh.Scene()
    for n, p in parts.items():
        m = p.mesh.copy()
        c = COL.get(p.material, "#cccccc").lstrip("#")
        rgba = [int(c[i:i + 2], 16) for i in (0, 2, 4)] + [255]
        m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(rgba, (len(m.faces), 1)))
        sc.add_geometry(m, node_name=n, geom_name=n)
    (OUT / "hand_v21.glb").write_bytes(sc.export(file_type="glb"))


def render(parts, PD):
    """Matplotlib-free: reuse the V2 three.js renderer with meshes written as temp STLs (only the posed hand)."""
    from tools.render import render as rnd

    tmp = OUT / "_render"
    tmp.mkdir(exist_ok=True)
    pj = json.load(open(OUT / "poses.json"))
    for pn in ("open", "fist", "tip_pinch", "power_grasp", "wrist_flex_+40"):
        if pn not in pj["poses"]:
            continue
        T = pj["poses"][pn]["T"]
        files, cols = {}, {}
        for n, p in parts.items():
            if p.group == "forearm" and not n.startswith(("forearm_yoke", "forearm_plate", "servo", "cart")):
                if pn != "open":
                    continue
            m = p.mesh.copy()
            m.apply_transform(np.array(T[n]).reshape(4, 4))
            f = tmp / f"{n}.stl"
            m.export(f)
            files[n] = str(f)
            cols[n] = COL.get(p.material, "#cccccc")
        try:
            rnd(files, str(OUT / f"hand_{pn}.png"), views=("iso", "front", "side", "top"), colors=cols,
                title=f"Tendra V2.1 full hand: {pn}", edges=False)
        except Exception as e:  # pragma: no cover
            print("render failed", pn, e)


if __name__ == "__main__":
    main()
