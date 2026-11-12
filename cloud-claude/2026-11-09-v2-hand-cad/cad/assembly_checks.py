"""Integration checks for the Tendra V2 assembly (study 2026-11-09).

    uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy \
        --with pillow --with rtree python cad/assembly_checks.py [--quick]
writes out/assembly/checks.json (+ prints a summary). Each module's internal pairs were already checked by
its designer, so here only pairs that CROSS modules are tested (fingers x fingers, fingers x palm, thumb x
palm / fingers, hand x wrist / forearm, object x hand). The palm is tested through a proxy without the
internal sheath bores (fingers and thumb cannot reach them); tubes are checked numerically in routing.py.
Threshold 0.5 mm^3 (as the designers).
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import assembly as A  # noqa: E402
import params as P  # noqa: E402
from tools.checks import interference, is_valid  # noqa: E402

OUT = A.OUT
FINGERS = A.FINGERS
F_BODIES = ("yoke", "proximal", "middle", "distal", "pad_tip", "pad_proximal", "pad_middle", "bar")
T_BODIES = ("thumb_mount", "thumb_base", "thumb_metacarpal", "thenar_pad", "thumb_proximal", "thumb_distal", "thumb_pad")
P_BODIES = ("shell", "back_plate", "palm_pad", "hypothenar_pad", "knuckle_pad", "rot_idlers")
INTENDED = [("T_thumb_mount", "P_shell"), ("P_rot_idlers", "P_shell"), ("W_palm_flange", "P_shell"),
            ("W_palm_sheath_clamp", "P_shell"), ("P_back_plate", "P_shell"), ("P_rot_idlers", "T_thumb_mount")]


def simple_shell():
    p = OUT / "palm_cache" / "check_shell_simple.brep"
    return cq.Workplane(obj=cq.Shape.importBrep(str(p)))


def select(parts, fingers=FINGERS, thumb=True, palm=True, wrist=(), extra=()):
    out = {}
    for f in fingers:
        for b in F_BODIES:
            if f"{f}_{b}" in parts:
                out[f"{f}_{b}"] = parts[f"{f}_{b}"]
    if thumb:
        for b in T_BODIES:
            out[f"T_{b}"] = parts[f"T_{b}"]
    if palm:
        for b in P_BODIES:
            k = f"P_{b}"
            if k in parts:
                out[k] = parts[k]
    for k in wrist:
        if k in parts:
            out[k] = parts[k]
    for k in extra:
        if k in parts:
            out[k] = parts[k]
    return out


def ignore_rules():
    ign = [(f"{f}_*", f"{f}_*") for f in FINGERS] + [("T_*", "T_*"), ("W_*", "W_*"), ("F_*", "F_*"),
                                                      ("P_*", "P_*")] + INTENDED
    return ign


def with_proxy(parts, Rp=None):
    import wrist
    sp = simple_shell()
    if Rp is not None:
        sp = wrist.rot_shape(sp, Rp)
    q = dict(parts)
    if "P_shell" in q:
        q["P_shell"] = sp
    return q


def run_pose(name, pose, wrist_parts=(), extra=(), tb=None):
    import wrist_kin as K
    t = time.time()
    parts = A.build(pose, forearm=bool([w for w in wrist_parts if w.startswith("F_")]), sheaths=False)
    Rp = K.R_palm(pose["wrist"]["wrist_flex"], pose["wrist"]["wrist_dev"])
    sel = with_proxy(select(parts, wrist=wrist_parts, extra=extra), Rp)
    hits = interference(sel, threshold=0.5, ignore=ignore_rules(), time_budget=tb)
    print(f"  {name}: {len(sel)} parts, {len(hits)} hits, {time.time() - t:.0f} s  " +
          "; ".join(f"{a}x{b}={v}" for a, b, v in hits[:8]), flush=True)
    return hits


WRIST_SET = ("W_wrist_ring", "W_wrist_base", "W_pin_dev_b", "W_pin_dev_f", "W_pin_flex_r", "W_pin_flex_l",
             "W_wrist_rod_R", "W_wrist_rod_L", "W_wrist_crank_R", "W_wrist_crank_L", "W_palm_flange", "W_palm_sheath_clamp",
             "F_forearm_shell_back", "F_forearm_cover_front", "F_forearm_frame", "F_side_cover_R", "F_side_cover_L")


def main(argv=None):
    quick = "--quick" in (argv or sys.argv)
    T0 = time.time()
    fp = OUT / "checks.json"
    res = json.loads(fp.read_text()) if ("--resume" in sys.argv and fp.exists()) else {}

    def save():
        fp.write_text(json.dumps(res, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    ps = A.poses()
    # 1. validity (open pose, everything incl. forearm)
    if "validity" not in res:
        parts = A.build(ps["a_open"], forearm=True)
        val = {n: is_valid(s) for n, s in parts.items()}
        bad = {n: v for n, v in val.items() if not v["ok"]}
        res["validity"] = {"parts": len(val), "ok": len(val) - len(bad),
                           "not_ok": {n: {k: v[k] for k in ("valid", "watertight", "solids")} for n, v in bad.items()}}
        save()
    print("validity:", res["validity"]["ok"], "/", res["validity"]["parts"], "not ok:", list(res["validity"]["not_ok"]), flush=True)
    # 2. the 7 poses (hand + wrist; wrist/forearm parts only where the wrist moves)
    res.setdefault("poses", {})
    for k, pose in ps.items():
        if k in res["poses"]:
            continue
        wp = WRIST_SET if pose["wrist"]["wrist_flex"] or pose["wrist"]["wrist_dev"] else ("W_palm_flange",)
        res["poses"][k] = run_pose(k, pose, wp, extra=("OBJ_cylinder55",))
        save()
    # 3. thumb base / metacarpal vs palm over the CMC range (MCP, IP straight)
    res.setdefault("thumb_vs_palm", {})
    rr, fr = P.THUMB_RANGES["cmc_rot"], P.THUMB_RANGES["cmc_flex"]
    for rot in (rr[0], (rr[0] + rr[1]) / 2, rr[1]):
        for fl in (fr[0], (fr[0] + fr[1]) / 2, fr[1]):
            key = f"rot{rot:+.0f}_flex{fl:+.0f}"
            if key in res["thumb_vs_palm"]:
                continue
            pose = A._pose(thumb={"cmc_rot": rot, "cmc_flex": fl, "mcp_flex": 0.0, "ip": 0.0})
            parts = A.build(pose, forearm=False, sheaths=False)
            sel = with_proxy(select(parts, fingers=("index",)))
            res["thumb_vs_palm"][key] = interference(sel, threshold=0.5, ignore=ignore_rules())
            print(f"  thumb {key}: {res['thumb_vs_palm'][key][:4]}", flush=True)
            save()
    # 4. palm / thumb / fingers vs wrist + forearm at the wrist extremes
    res.setdefault("wrist_extremes", {})
    for fl in (-50.0, 50.0):
        for dv in (-30.0, 20.0):
            key = f"flex{fl:+.0f}_dev{dv:+.0f}"
            if key in res["wrist_extremes"]:
                continue
            pose = A._pose(wrist={"wrist_flex": fl, "wrist_dev": dv})
            res["wrist_extremes"][key] = run_pose(f"wrist {fl:+.0f}/{dv:+.0f}", pose, WRIST_SET)
            save()
    # 5. wrist module's own grid (ring, base, pins, drive, bundle vs a palm proxy) with the lowered clamp
    if "wrist_grid" not in res:
        import wrist
        r = wrist.check_collisions(quick=True)
        res["wrist_grid"] = {f"{k[0]:+.0f}/{k[1]:+.0f}": v for k, v in r.items()}
        save()
    # 6. sheath bends across the wrist (wrist_kin bundle model) + forearm fan + palm routes
    import wrist_kin as K
    stats = {}
    for f in (-50.0, 0.0, 50.0):
        for d in (-30.0, 0.0, 20.0):
            L, r = K.tube_stats(f, d)
            stats[f"{f:+.0f}/{d:+.0f}"] = {"min_bend_r": round(float(r.min()), 1)}
    res["wrist_bundle"] = stats
    fan = json.loads((OUT / "fan_routes.json").read_text())["strands"]
    res["forearm_fan"] = {"min_radius": min(f["min_radius"] for f in fan), "min_wall_gap": min(f["clearance"] for f in fan),
                          "n_R_lt_15": sum(f["min_radius"] < 15 for f in fan), "n_gap_lt_0.4": sum(f["clearance"] < 0.4 for f in fan)}
    res["abd_pair_table"] = "out/assembly/abd_pair_table.json"
    res["runtime_s"] = round(time.time() - T0)
    save()
    print(f"checks done in {time.time() - T0:.0f} s")


if __name__ == "__main__":
    main()
