"""Tendra V2 concept: full right hand + wrist + forearm assembly (integrator, study 2026-11-09).

    build(pose) -> {"parts": {name: shape}, "material": {name: str}, "group": {name: str}}
pose = {"index": {"mcp_abd", "mcp_flex", "pip"}, "middle": ..., "ring": ..., "little": ...,
        "thumb": {"cmc_rot", "cmc_flex", "mcp_flex", "ip"}, "wrist": {"wrist_flex", "wrist_dev"}, "object": ...}
Part names are prefixed per module: <finger>_*, T_* (thumb), P_* (palm), W_* (wrist), F_* (forearm), OBJ_*.
The hand side (fingers, thumb, palm, palm flange, sheath clamp) moves with R_palm = Ry(dev) @ Rx(flex).

    uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy \
        --with pillow --with rtree python cad/assembly.py [--poses a,b,...] [--no-export] [--no-render]
writes out/assembly/: hand_open.step / .glb (+ stl/), pose_<x>.png, poses.json, mass.json.
Checks: cad/assembly_checks.py.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import params as P  # noqa: E402

OUT = HERE.parent / "out" / "assembly"
FINGERS = ("index", "middle", "ring", "little")
_CACHE: dict = {}

# ----------------------------------------------------------------------------------------------------
# poses
# ----------------------------------------------------------------------------------------------------
_TD = json.loads((HERE.parent / "out" / "thumb" / "thumb_design.json").read_text())["eval"]


def _tq(key):
    return dict(zip(("cmc_rot", "cmc_flex", "mcp_flex", "ip"), _TD[key]["q_thumb"]))


def _fq(key):
    return dict(zip(("mcp_abd", "mcp_flex", "pip"), _TD[key]["q_finger"]))


ZERO = {"mcp_abd": 0.0, "mcp_flex": 0.0, "pip": 0.0}


def _pose(fingers=None, thumb=None, wrist=None, obj=None):
    p = {f: dict(ZERO) for f in FINGERS}
    p.update(fingers or {})
    p["thumb"] = thumb or _tq("flat")
    p["wrist"] = {"wrist_flex": 0.0, "wrist_dev": 0.0, **(wrist or {})}
    if obj:
        p["object"] = obj
    return p


def poses():
    """Fix round: grasp poses from the solver (cad/poses.py -> out/assembly/poses_solved.json): fingers wrap the
    cylinder / curl onto the palm on the real geometry, pinch / opposition backed off to 0..0.5 mm contact."""
    ps_ = json.loads((OUT / "poses_solved.json").read_text())
    curl = {"mcp_abd": 0.0, "mcp_flex": 15.0, "pip": 20.0}
    tp = ps_["pinch_tip_pinch"]
    pinch = _pose({"index": tp["finger_q"], "middle": curl, "ring": curl, "little": curl}, tp["thumb"])
    lt = ps_["pinch_little"]
    # neighbours follow the little within the pair limits (abd_pair_table.json)
    little = _pose({"little": lt["finger_q"], "ring": {"mcp_abd": 9.0, "mcp_flex": 30.0, "pip": 30.0},
                    "middle": {"mcp_abd": 10.5, "mcp_flex": 30.0, "pip": 30.0}, "index": {"mcp_abd": 12.0, "mcp_flex": 30.0, "pip": 30.0}},
                   lt["thumb"])
    pw = ps_["power55"]
    power = _pose(pw["fingers"], pw["thumb"], obj={"cylinder": pw["cylinder"]})
    fs = ps_["fist"]
    return {
        "a_open": _pose(),
        "b_fist": _pose(fs["fingers"], fs["thumb"]),
        "c_tip_pinch": pinch,
        "d_thumb_little": little,
        "e_power55": power,
        "f_flex50_pinch": {**pinch, "wrist": {"wrist_flex": 50.0, "wrist_dev": 0.0}},
        "g_dev-30": _pose(wrist={"wrist_dev": -30.0}),
    }


def cylinder_axis(d, gamma, zc):
    """Same convention as thumb_kinematics.cylinder_frame, but resting on the knuckle pads (y -19.6)."""
    c = np.array([math.cos(math.radians(gamma)), 0.0, math.sin(math.radians(gamma))])
    o = np.array([0.0, -21.6 - d / 2, zc])  # thumb_kinematics with PALM_Y[0] = -19.6 (knuckle pad face)
    return o, c


def power_fingers(d, gamma, zc):
    """Wrap each finger around the cylinder: for mcp in a grid take the largest pip that keeps every palmar
    sample of the finger outside the cylinder (+0.3), prefer the most wrapped pose."""
    import finger
    o, c = cylinder_axis(d, gamma, zc)
    R = d / 2 + 0.3
    out = {}
    for f in FINGERS:
        g = finger.geom(f)
        best = (0.0, 0.0)
        for mcp in np.arange(10, 91, 2.5):
            okp = None
            for pip in np.arange(0, 96, 2.5):
                q = finger.full_q(g, {"mcp_abd": 0.0, "mcp_flex": mcp, "pip": pip})
                Tb = finger.body_transforms(g, q)
                pts = []
                for body, z0, z1 in (("proximal", 3, g["z_pip"]), ("middle", g["z_pip"], g["z_dip"]), ("distal", g["z_dip"], g["z_tip"])):
                    for z in np.linspace(z0, z1, 6):
                        for x in (-g["hw"] + 1, 0, g["hw"] - 1):
                            pts.append(finger.xf(Tb[body], [[x, -g["hh"] - 1.5, z]])[0])
                pts = np.array(pts)
                v = pts - o
                r = np.linalg.norm(v - np.outer(v @ c, c), axis=1)
                if (r >= R).all():
                    okp = pip
                else:
                    break
            if okp is None:
                break
            best = (mcp, okp)
        out[f] = {"mcp_abd": 0.0, "mcp_flex": float(best[0]), "pip": float(best[1])}
    return out


# ----------------------------------------------------------------------------------------------------
# materials / colours
# ----------------------------------------------------------------------------------------------------
COL = {"PETG_finger": "#e6d6b8", "PETG_thumb": "#dcc7a1", "PETG_palm": "#cdd7e2", "PETG_wrist": "#b7c3cf",
       "PETG_forearm": "#c6ccd3", "TPU": "#3b4047", "steel": "#8b9198", "PTFE": "#f3f3ef", "servo": "#26292d",
       "electronics": "#2f6b4f", "object": "#7fb3d5", "bearing": "#a3a9b0"}


def classify(name):
    n = name.lower()
    if n.startswith("obj_"):
        return "object"
    if "pad" in n:
        return "TPU"
    if any(k in n for k in ("ptfe", "sheath", "bundle", "neck_top")):
        return "PTFE"
    if any(k in n for k in ("bearing", "rot_idlers")):
        return "bearing"
    if any(k in n for k in ("tie_liner", "pin", "post", "stub", "nip", "eyelet", "_bar", "thumb_guides", "rod_", "_rod", "barrel",
                            "strands", "gears", "crank")):
        return "steel"
    if "servo" in n:
        return "servo"
    if any(k in n for k in ("esp32", "urt", "power_board")):
        return "electronics"
    grp = {"T": "PETG_thumb", "P": "PETG_palm", "W": "PETG_wrist", "F": "PETG_forearm"}.get(name.split("_")[0])
    return grp or "PETG_finger"


DENSITY = {"PETG": 1.27 * 0.40, "TPU": 1.21, "steel": 7.85, "PTFE": 2.2, "bearing": 7.85 * 0.6}  # g/cm^3


# ----------------------------------------------------------------------------------------------------
# build
# ----------------------------------------------------------------------------------------------------
def _palm():
    if "palm" not in _CACHE:
        import palm
        _CACHE["palm"] = palm.build()
    return _CACHE["palm"]


def _forearm():
    if "forearm" not in _CACHE:
        import forearm
        _CACHE["forearm"] = forearm.build()["parts"]
    return _CACHE["forearm"]


def clamp_stepped():
    """The wrist's palm sheath clamp, fix round: top stepped down to z 15.5 over the 2 cmc_rot cells (steel guide
    tubes) and 6 tilted bores (2.2) for the thumb's PTFE tubes, from their cell at the clamp bottom up toward +x."""
    if "clamp" not in _CACHE:
        import routing
        import wrist
        c = wrist._base_parts()["clamp"]
        z0, z1 = routing.Z_CELL_THUMB, P.SHEATH_CLAMP_Z[1] + 1
        C = routing.cells()
        for cell in routing.ROT_CELLS:
            a, b = C[cell]
            c = c.cut(cq.Workplane("XY").box(2.1, 2.1, z1 - z0).translate((a, b, (z0 + z1) / 2)))
        rr, _ = routing.load()
        for s in rr:
            if s["kind"] == "thumb_axial" and s.get("cell_bottom"):
                cb, d = np.array(s["cell_bottom"]), np.array(s["end_dir"])
                p0, p1 = cb + d * 1.0, cb - d * 9.0
                c = c.cut(cq.Workplane(obj=cq.Solid.makeCylinder(P.SHEATH_BORE / 2, float(np.linalg.norm(p1 - p0)),
                                                                cq.Vector(*p0), cq.Vector(*((p1 - p0) / np.linalg.norm(p1 - p0))))))
        _CACHE["clamp"] = c
    return _CACHE["clamp"]


def build(pose, forearm=True, sheaths=True, verbose=False):
    import finger
    import thumb
    import wrist
    import wrist_kin as K
    t0 = time.time()
    wq = pose.get("wrist", {})
    flex, dev = float(wq.get("wrist_flex", 0.0)), float(wq.get("wrist_dev", 0.0))
    Rp = K.R_palm(flex, dev)
    hand = {}
    for f in FINGERS:
        for k, s in finger.build(f, pose.get(f, {}))["parts"].items():
            hand[f"{f}_{k}"] = s
    tb = thumb.build(pose.get("thumb", {}))["parts"]
    pbm = _palm()["parts"].get("thumb_mount_bored")
    if pbm is not None:  # a bore that grazes the mount's skin can leave an inverted (negative-volume) sliver solid in
        v_ = pbm.val() if hasattr(pbm, "val") else pbm  # the compound; it makes OCC booleans return the other part whole
        good = [q for q in v_.Solids() if q.Volume() > 0.0]
        if len(good) != len(v_.Solids()):
            import cadquery as cq
            pbm = cq.Workplane(obj=good[0] if len(good) == 1 else cq.Compound.makeCompound(good))
    for k, s in tb.items():
        hand[f"T_{k}"] = pbm if (k == "thumb_mount" and pbm is not None) else s
    pb = _palm()
    for k, s in pb["parts"].items():
        if (not sheaths and k in ("palm_sheaths", "thumb_guides")) or k == "thumb_mount_bored":
            continue
        hand[f"P_{k.replace('palm_', '')}"] = s
    if "object" in pose:
        cy = pose["object"]["cylinder"]
        o, c = (np.array(cy["origin"]), np.array(cy["axis"])) if "origin" in cy else cylinder_axis(cy["d"], cy["gamma"], cy["zc"])
        L = 120.0
        hand["OBJ_cylinder55"] = cq.Workplane(obj=cq.Solid.makeCylinder(cy["d"] / 2, L, cq.Vector(*(o - c * L / 2)), cq.Vector(*c)))
    wb = wrist.build({"wrist_flex": flex, "wrist_dev": dev}, bundle=True)["parts"]
    hand["W_palm_flange"] = wrist._base_parts()["flange"]
    hand["W_palm_sheath_clamp"] = clamp_stepped()
    parts = {k: wrist.rot_shape(s, Rp) for k, s in hand.items()}
    for k, s in wb.items():
        if k in ("palm_flange", "palm_sheath_clamp"):
            continue
        parts[f"W_{k}"] = s
    if forearm:
        for k, s in _forearm().items():
            if k.startswith("sheath") or k.startswith("ptfe"):
                parts[f"F_{k}"] = s
            else:
                parts[f"F_{k}"] = s
    if verbose:
        print(f"  assembly built: {len(parts)} parts, {time.time() - t0:.1f} s")
    return parts


def colours(parts):
    return {n: COL[classify(n)] for n in parts}


# ----------------------------------------------------------------------------------------------------
# totals
# ----------------------------------------------------------------------------------------------------
def mass_table(parts):
    """Grams per part (volume x density; servos / electronics from catalogue)."""
    rows = {}
    for n, s in parts.items():
        cl = classify(n)
        if cl == "object":
            continue
        sh = s.val() if hasattr(s, "val") and len(s.vals()) == 1 else (cq.Compound.makeCompound(s.vals()) if hasattr(s, "vals") else s)
        vol = float(sh.Volume()) / 1000.0  # cm^3
        if cl == "bearing" and "bearing" in n:
            g = P.BEARING_MASS_G["MR63ZZ" if "bearing_abd" in n else "MR84ZZ"]  # catalogue, not volume x density
        elif cl == "servo":
            g = 55.0 if "W_" in n else 13.0
        elif cl == "electronics":
            g = {"esp32": 10.0, "urt": 8.0, "power": 12.0}[next(k for k in ("esp32", "urt", "power") if k in n.lower())]
        elif cl.startswith("PETG"):
            g = vol * DENSITY["PETG"]
        else:
            g = vol * DENSITY.get(cl, 1.27)
        rows[n] = dict(material=cl, volume_cm3=round(vol, 2), grams=round(g, 1))
    groups = {}
    for n, r in rows.items():
        grp = ("fingers" if n.split("_")[0] in FINGERS else {"T": "thumb", "P": "palm", "W": "wrist", "F": "forearm"}[n.split("_")[0]])
        groups[grp] = round(groups.get(grp, 0) + r["grams"], 1)
    return rows, groups


def bearing_bill(parts):
    """Count and mass of the joint bearings in the assembly, by type and by hand part (finger / thumb)."""
    bill = {"MR84ZZ": {"count": 0, "mass_g": 0.0, "by": {}}, "MR63ZZ": {"count": 0, "mass_g": 0.0, "by": {}}}
    for n in parts:
        if "bearing" not in n or n.startswith(("W_", "F_")) or n.endswith(("bearing_top", "bearing_bottom")):
            continue
        t = "MR63ZZ" if "bearing_abd" in n else "MR84ZZ"
        who = n.split("_")[0] if n.split("_")[0] in FINGERS else "thumb"
        bill[t]["count"] += 1
        bill[t]["mass_g"] = round(bill[t]["mass_g"] + P.BEARING_MASS_G[t], 2)
        bill[t]["by"][who] = bill[t]["by"].get(who, 0) + 1
    other = {n: ("6704ZZ" if n.endswith(("bearing_top", "bearing_bottom")) else "623ZZ/other") for n in parts
             if "bearing" in n and (n.startswith(("W_", "F_")) or n.endswith(("bearing_top", "bearing_bottom")))}
    bill["other_bearing_parts"] = other
    return bill


def dims(parts, names=None):
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for n, s in parts.items():
        if names and not any(n.startswith(p) for p in names):
            continue
        sh = s.val() if hasattr(s, "val") else s
        bb = (cq.Compound.makeCompound(s.vals()) if hasattr(s, "vals") else s).BoundingBox()
        lo = np.minimum(lo, [bb.xmin, bb.ymin, bb.zmin])
        hi = np.maximum(hi, [bb.xmax, bb.ymax, bb.zmax])
    return lo.round(1).tolist(), hi.round(1).tolist()


# ----------------------------------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------------------------------
VIEWS = {"a_open": ("front", "back", "side", "iso"), "default": ("front", "side", "iso", (20, 120))}


def main(argv=None):
    import argparse
    from tools.render import render
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", default="all")
    ap.add_argument("--no-export", action="store_true")
    ap.add_argument("--no-render", action="store_true")
    a = ap.parse_args(argv)
    OUT.mkdir(parents=True, exist_ok=True)
    T0 = time.time()
    ps = poses()
    (OUT / "poses.json").write_text(json.dumps(ps, indent=1))
    sel = list(ps) if a.poses == "all" else a.poses.split(",")
    for k in sel:
        t = time.time()
        parts = build(ps[k], forearm=(k == "a_open"), verbose=True)
        if k == "a_open" and not a.no_export:
            from tools.export import export_all
            rows, groups = mass_table(parts)
            lo, hi = dims(parts)
            hlo, hhi = dims(parts, names=[*FINGERS, "T_", "P_"])
            bb = bearing_bill(parts)
            (OUT / "mass.json").write_text(json.dumps({"groups": groups, "total_g": round(sum(groups.values()), 1),
                                                       "bearings": bb,
                                                       "dims_all": [lo, hi], "dims_hand": [hlo, hhi], "parts": rows}, indent=1))
            print("  mass groups", groups, "total", round(sum(groups.values()), 1))
            paths = export_all(parts, OUT, "tendra_v2_open", colors=colours(parts), tol=0.12, ang=0.35)
            gl = Path(paths["glb"])
            print("  export", paths, f"glb {gl.stat().st_size / 1e6:.1f} MB")
        if not a.no_render:
            show = parts if k == "a_open" else parts
            if k != "a_open":
                show = {n: s for n, s in parts.items() if not n.startswith("F_")}
            title = f"Tendra V2 - {k}  (wrist flex {ps[k]['wrist']['wrist_flex']:+.0f}, dev {ps[k]['wrist']['wrist_dev']:+.0f})"
            render(show, OUT / f"pose_{k}.png", views=VIEWS.get(k, VIEWS["default"]), colors=colours(show),
                   size=(1800, 1400), title=title, legend=False, edges=False, tol=0.25, ang=0.4)
        print(f"{k}: {time.time() - t:.0f} s")
    print(f"total {time.time() - T0:.0f} s")


if __name__ == "__main__":
    main()
