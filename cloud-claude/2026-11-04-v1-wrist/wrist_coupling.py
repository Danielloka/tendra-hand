"""Tendra Hand V1 wrist study: tendon coupling, sheath friction, bundle size and wrist torque.

    uv run python cloud-claude/2026-11-04-v1-wrist/wrist_coupling.py

Reads hardware/robot_description/v1_export/tendon_routes.json (the 32 servo strands) and, if
MuJoCo is installed, sim/models/tendra_hand_v1_wrist.xml (hand mass and centre of mass).
Prints the tables used in 02-mechanism-and-maths.md. Units: mm, N, rad inside; degrees in output.

Wrist model (the same as sim/convert_v1_wrist.py): a gimbal whose two axes cross at the wrist
centre C = (-33.5, 25.5) mm in x, y. wrist_flex turns about +X (positive = palm toward -Y),
wrist_dev turns about +Y (positive = toward the thumb, +X). Ranges -60..70 deg and -30..20 deg.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ROUTES = ROOT / "hardware" / "robot_description" / "v1_export" / "tendon_routes.json"
WRIST_XML = ROOT / "sim" / "models" / "tendra_hand_v1_wrist.xml"

DEG = math.pi / 180
C_XY = np.array([-33.5, 25.5])  # wrist centre, x y (mm)
FLEX_RANGE = (-60.0, 70.0)
DEV_RANGE = (-30.0, 20.0)
H_EYELET = 10.0  # mm: bare strand guided by eyelets 10 mm above and below the centre (20 mm gap)
EA_BRAID = 5500.0  # N, 50 lb UHMWPE braid (tendon-mechanics.md 1.3)
FINGER_PART = 60.0  # mm, finger entry -> drum, not in points_mm (rough)
WRIST_GAP = 20.0  # mm added by the wrist model
SPOOL_R = 5.0
T_STALL = 0.226 / (SPOOL_R / 1000)  # N, SCS0009 stall at a 5 mm spool (6 V)
PTFE_E = 500.0  # MPa (N/mm^2), PTFE 0.4-0.6 GPa
TUBE_OD, TUBE_ID, LINE_D = 2.0, 1.0, 0.36


def rot(flex: float, dev: float) -> np.ndarray:
    """Palm orientation in the forearm frame: R = Rx(flex) @ Ry(dev) (gimbal: flex outer)."""
    cf, sf, cd, sd = math.cos(flex), math.sin(flex), math.cos(dev), math.sin(dev)
    rx = np.array([[1, 0, 0], [0, cf, -sf], [0, sf, cf]])
    ry = np.array([[cd, 0, sd], [0, 1, 0], [-sd, 0, cd]])
    return rx @ ry


def grid(n=27):
    f = np.linspace(*FLEX_RANGE, n) * DEG
    d = np.linspace(*DEV_RANGE, n) * DEG
    return [(a, b) for a in f for b in d]


def bare_dl(offsets_xy: np.ndarray, flex: float, dev: float, h: float = H_EYELET,
            guided: bool = False) -> np.ndarray:
    """Length change of bare strands crossing the wrist at (x, y) offsets from C (mm).
    Forearm eyelet at (b, -h), palm eyelet at (b, +h) relative to C, palm eyelet rotated.
    One axis, exact: dL = 2 b sin(t/2) - 2 h (1 - cos(t/2)).
    guided=True: a rounded guide at the centre keeps a centred strand at constant length (the
    -2h(1-cos) term is removed), which is the 'bend over the axis' case."""
    n = len(offsets_xy)
    lo = np.c_[offsets_xy, -h * np.ones(n)]
    up = np.c_[offsets_xy, h * np.ones(n)]
    up_r = up @ rot(flex, dev).T
    dl = np.linalg.norm(up_r - lo, axis=1) - 2 * h
    if guided:
        c = np.array([[0.0, 0.0, h]]) @ rot(flex, dev).T
        dl -= np.linalg.norm(c - np.array([[0.0, 0.0, -h]]), axis=1) - 2 * h
    return dl


def wrap_angle(flex: float, dev: float) -> float:
    """Angle between the forearm axis and the palm axis = the bend a centred sheath gets."""
    return math.acos(max(-1.0, min(1.0, math.cos(flex) * math.cos(dev))))


def load():
    d = json.loads(ROUTES.read_text())
    strands = d["strands"]
    spj = {s["joint"]: s["servo_per_joint"] for s in d["servos"]}
    drums = d["drums_mm"]
    for s in strands:
        p = np.array(s["points_mm"])
        s["L_route"] = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
        s["L_total"] = s["L_route"] + FINGER_PART + WRIST_GAP
        s["turn_palm"] = float(np.degrees(turning(p)))
    return d, strands, spj, drums


def turning(p: np.ndarray) -> float:
    v = np.diff(p, axis=0)
    v = v[np.linalg.norm(v, axis=1) > 1e-9]
    v /= np.linalg.norm(v, axis=1)[:, None]
    return float(np.arccos(np.clip((v[:-1] * v[1:]).sum(1), -1, 1)).sum())


def pairs(strands):
    by = {}
    for s in strands:
        by.setdefault(s["joint"], {})[s["side"]] = s
    return by


def loop_effects(offs: dict, by, drums, spj, label: str, guided: bool = False):
    """For each joint: worst joint-angle error (difference) and tension change (sum)."""
    rows = []
    G = grid()
    for joint, fe in by.items():
        f, e = fe["flex"], fe["ext"]
        r_d = drums[joint]
        k = EA_BRAID / (0.5 * (f["L_total"] + e["L_total"]))  # N/mm
        xy = np.array([offs[f["name"]], offs[e["name"]]])
        q_err, d_t = [], []
        for fl, dv in G:
            dlf, dle = bare_dl(xy, fl, dv, guided=guided)
            q_err.append((dlf - dle) / (2 * r_d))  # rad (joint)
            d_t.append(k * (dlf + dle) / 2)  # N
        q_err, d_t = np.array(q_err), np.array(d_t)
        rows.append((joint, np.degrees(np.abs(q_err).max()),
                     np.degrees(np.abs(q_err).max()) * spj[joint], d_t.min(), d_t.max()))
    print(f"\n### Bare strands, layout: {label} (eyelets at +-10 mm{', centre guide' if guided else ''})")
    print("| joint | max joint error (deg) | servo correction (deg) | tension change min / max (N) |")
    print("|---|---|---|---|")
    for j, qe, se, tmin, tmax in rows:
        print(f"| {j} | {qe:.1f} | {se:.1f} | {tmin:+.0f} / {tmax:+.0f} |")
    a = np.array([r[1:] for r in rows])
    print(f"| **worst** | **{a[:, 0].max():.1f}** | **{a[:, 1].max():.1f}** | "
          f"**{a[:, 2].min():+.0f} / {a[:, 3].max():+.0f}** |")
    return a


def hex_bundle(n: int, pitch: float) -> np.ndarray:
    pts = []
    for i in range(-8, 9):
        for j in range(-8, 9):
            pts.append([pitch * (i + 0.5 * j), pitch * j * math.sqrt(3) / 2])
    pts = np.array(pts)
    pts = pts[np.argsort(np.linalg.norm(pts, axis=1), kind="stable")][:n]
    return pts - pts.mean(0)


def rect_grid(nx: int, ny: int, pitch: float) -> np.ndarray:
    xs = (np.arange(nx) - (nx - 1) / 2) * pitch
    ys = (np.arange(ny) - (ny - 1) / 2) * pitch
    return np.array([[x, y] for x in xs for y in ys])


def assign(strands, slots: np.ndarray, scheme: str) -> dict:
    """Give each strand a slot (offset from C). scheme 'sorted': strands sorted by their palm
    x then y onto slots sorted the same way (keeps them near their natural place, mirrors well);
    'adjacent': flex and ext of a joint on neighbouring slots; 'symmetric': ext = -flex."""
    names = [s["name"] for s in strands]
    if scheme == "sorted":
        nat = np.array([s["wrist_bottom_mm"][:2] for s in strands]) - C_XY
        o_s = np.lexsort((nat[:, 1], np.round(nat[:, 0], 0)))
        o_t = np.lexsort((slots[:, 1], np.round(slots[:, 0], 3)))
        return {names[i]: slots[t] for i, t in zip(o_s, o_t)}
    by = pairs(strands)
    out = {}
    if scheme == "adjacent":
        # greedy: from the outside in, pair each free slot with its nearest free neighbour
        free = list(np.argsort(-np.linalg.norm(slots, axis=1)))
        duo = []
        while free:
            i = free.pop(0)
            j = min(free, key=lambda t: np.linalg.norm(slots[t] - slots[i]))
            free.remove(j)
            duo.append((slots[i], slots[j]))
        for (a, b), (j, fe) in zip(duo, by.items()):
            out[fe["flex"]["name"]], out[fe["ext"]["name"]] = a, b
        return out
    if scheme == "symmetric":
        half = slots[(slots[:, 0] > 1e-9) | ((abs(slots[:, 0]) < 1e-9) & (slots[:, 1] > 0))]
        half = half[np.argsort(np.linalg.norm(half, axis=1))][:16]
        for i, (j, fe) in enumerate(by.items()):
            out[fe["flex"]["name"]] = half[i]
            out[fe["ext"]["name"]] = -half[i]
        return out
    raise ValueError(scheme)


def s_curve_turn(delta: float, height: float) -> tuple[float, float]:
    """Two-arc S-curve with parallel end tangents: total turning (rad), min radius (mm)."""
    if delta < 1e-9:
        return 0.0, math.inf
    a = 2 * math.atan(delta / height)
    return 2 * a, height / (2 * math.sin(a))


def main():
    _, strands, spj, drums = load()
    by = pairs(strands)
    print("# Tendra V1 wrist study (generated by wrist_coupling.py)")
    nat = {s["name"]: np.array(s["wrist_bottom_mm"][:2]) - C_XY for s in strands}
    off = np.array(list(nat.values()))
    print(f"\nStrands today at the wrist plate: x offset {off[:, 0].min():+.1f}..{off[:, 0].max():+.1f}"
          f" mm, y offset {off[:, 1].min():+.1f}..{off[:, 1].max():+.1f} mm from C")
    L = np.array([s["L_total"] for s in strands])
    print(f"Strand length spool->drum (est.) {L.min():.0f}..{L.max():.0f} mm, braid k = EA/L = "
          f"{EA_BRAID / L.max():.0f}..{EA_BRAID / L.min():.0f} N/mm")

    # --- 1. single-strand length change (exact formula check) ---
    print("\n## 1a. Bare strand length change, one axis: dL = 2 b sin(t/2) - 2 h (1 - cos(t/2))")
    print("| offset b (mm) | flex -60 | flex +70 | dev -30 | dev +20 |")
    print("|---|---|---|---|---|")
    for b in (0, 2, 5, 10, 20, 35):
        vals = []
        for ang, axis in ((-60, "f"), (70, "f"), (-30, "d"), (20, "d")):
            xy = np.array([[0, b]]) if axis == "f" else np.array([[b, 0]])
            fl, dv = (ang * DEG, 0) if axis == "f" else (0, ang * DEG)
            vals.append(bare_dl(xy, fl, dv)[0])
        print(f"| {b} | " + " | ".join(f"{v:+.1f}" for v in vals) + " |")

    # per strand, today's positions
    print("\n## 1b. Each strand, bare at today's positions (mm, worst over the range)")
    print("| strand | x off | y off | dL min | dL max |")
    print("|---|---|---|---|---|")
    G = grid()
    allxy = np.array([nat[s["name"]] for s in strands])
    dls = np.array([bare_dl(allxy, f, v) for f, v in G])
    for i, s in enumerate(strands):
        print(f"| {s['name']} | {allxy[i, 0]:+.1f} | {allxy[i, 1]:+.1f} | {dls[:, i].min():+.1f} |"
              f" {dls[:, i].max():+.1f} |")

    loop_effects(nat, by, drums, spj, "today's spread positions (x -69..5, y 15..36)")
    hexb = hex_bundle(32, 2.1)
    print(f"\nRound bundle: 32 tubes at 2.1 mm pitch, outer tube centre at r = "
          f"{np.linalg.norm(hexb, axis=1).max():.2f} mm")
    loop_effects(assign(strands, hexb, "adjacent"), by, drums, spj,
                         "round bundle at the centre, flex+ext neighbours", True)
    loop_effects(assign(strands, hexb, "symmetric"), by, drums, spj,
                         "round bundle at the centre, ext = mirror of flex through C", True)

    # --- Bowden: residual effects ---
    print("\n## 1c. Bowden sheath residuals")
    wmax = max(wrap_angle(f, v) for f, v in G)
    print(f"Max sheath bend at the wrist (angle between forearm and palm axes): {np.degrees(wmax):.1f}"
          " deg (flex 70 + dev 20)")
    print("| mu | force kept e^(-mu*theta), wrist only | + palm route (median strand) | + palm route (worst) |")
    print("|---|---|---|---|")
    tp = np.array([s["turn_palm"] for s in strands]) * DEG
    for mu in (0.05, 0.1, 0.15, 0.25):
        print(f"| {mu} | {math.exp(-mu * wmax):.2f} | {math.exp(-mu * (wmax + np.median(tp))):.2f} |"
              f" {math.exp(-mu * (wmax + tp.max())):.2f} |")
    print(f"Palm routes today: turning {np.degrees(tp.min()):.0f}..{np.degrees(tp.max()):.0f} deg,"
          f" median {np.degrees(np.median(tp)):.0f} deg")
    c = (TUBE_ID - LINE_D) / 2
    k_med = EA_BRAID / np.median(L)
    print(f"Clearance shift: line moves to the inner wall, path -c*theta, c = {c:.2f} mm -> "
          f"{c * wmax:.2f} mm at max bend, both strands -> tension drop ~{k_med * c * wmax:.0f} N"
          f" (braid, k {k_med:.0f} N/mm). With a 0.6 mm ID liner: c = {(0.6 - LINE_D) / 2:.2f} mm"
          f" -> {(0.6 - LINE_D) / 2 * wmax:.2f} mm, ~{k_med * (0.6 - LINE_D) / 2 * wmax:.0f} N")
    A = math.pi / 4 * (TUBE_OD**2 - TUBE_ID**2)
    EI = PTFE_E * math.pi / 64 * (TUBE_OD**4 - TUBE_ID**4)
    print(f"PTFE 1x2 tube: EA = {PTFE_E * A:.0f} N, EI = {EI:.0f} N*mm^2")
    print("| sheath free length (mm) | compression at 5 N | at stall 45 N | joint deg at 6 mm drum (45 N) |"
          " Euler buckling if unloaded inside (N) |")
    print("|---|---|---|---|---|")
    for Ls in (30, 60, 100, 150):
        d5, d45 = 5 * Ls / (PTFE_E * A), T_STALL * Ls / (PTFE_E * A)
        print(f"| {Ls} | {d5:.2f} | {d45:.2f} | {np.degrees(d45 / 6):.1f} |"
              f" {math.pi**2 * EI / Ls**2:.1f} |")
    print("Bend radius and bending moment of the sheaths at max wrist bend (uniform arc):")
    print("| bent length (mm) | radius (mm) | moment per tube (N*mm) | 32 tubes (N*m) |")
    print("|---|---|---|---|")
    for Lb in (20, 30, 40, 60):
        R = Lb / wmax
        print(f"| {Lb} | {R:.0f} | {EI / R:.1f} | {32 * EI / R / 1000:.2f} |")

    # --- mismatch for sheaths with fixed ends (tube shape must absorb b*theta) ---
    print("\n## 1d. Sheath length mismatch = bare dL at the tube's place in the waist (mm)")
    layouts = {
        "today spread (no waist)": allxy,
        "round bundle 32 x 2.1 mm": hexb,
        "oval grid 8 (x) x 4 (y), 2.1 mm": rect_grid(8, 4, 2.1),
        "offset axes, 2 rows (flex) / 2 rows (dev)": None,
    }
    print("| layout | flex only, worst | dev only, worst | both, worst | half-sine bow for the worst, 60 mm span |")
    print("|---|---|---|---|---|")
    for name, xy in layouts.items():
        if xy is None:  # offset axes: |e_y| <= 1.05 at the flex axis, |e_x| <= 1.05 at dev
            ef, ed = 1.05 * 2 * math.sin(70 * DEG / 2), 1.05 * 2 * math.sin(30 * DEG / 2)
            vals = (ef, ed, ef + ed)
        else:
            fo = max(np.abs(bare_dl(xy, a * DEG, 0, guided=True)).max() for a in FLEX_RANGE)
            do = max(np.abs(bare_dl(xy, 0, a * DEG, guided=True)).max() for a in DEV_RANGE)
            bo = max(np.abs(bare_dl(xy, f, v, guided=True)).max() for f, v in G)
            vals = (fo, do, bo)
        bow = math.sqrt(4 * 60 * vals[2]) / math.pi
        print(f"| {name} | {vals[0]:.1f} | {vals[1]:.1f} | {vals[2]:.1f} | {bow:.1f} |")

    # --- routing: crossed hourglass through the centre ---
    print("\n## 1e. Crossed hourglass: straight palm line to C, straight forearm line from C to the")
    print("point-reflected spool (strand from the +x/+y side goes to the -x/-y spool)")
    print("| wrist gap / forearm extra (mm) | palm turn at entry, median / worst (deg) | kink at C,"
          " median / worst | spool fleet angle, worst | force kept at max wrist, mu 0.1 / 0.25 (worst) |")
    print("|---|---|---|---|---|")
    spools = {s["name"]: np.array(s["tangent_mm"]) for s in strands}
    nat3 = {s["name"]: np.array(s["wrist_bottom_mm"]) for s in strands}
    for gap, extra in ((20, 0), (30, 0), (30, 40), (30, 60)):
        cz_p = -30.0 - gap / 2  # centre in the palm frame
        cz_f = -30.0 + gap / 2  # forearm frame: forearm top -30, centre gap/2 above it
        turns, kinks, fleets, keep = [], [], [], []
        for s in strands:
            sb = np.array(s["small_bore_end_mm"])
            c_p = np.array([*C_XY, cz_p])
            u_p = (c_p - sb) / np.linalg.norm(c_p - sb)
            turns.append(math.degrees(math.acos(-u_p[2])))
            # the strand that comes from palm position w goes to the spool at the reflected place
            w = nat3[s["name"]][:2]
            refl = 2 * C_XY - w
            tgt = min(spools.values(), key=lambda t: np.linalg.norm(t[:2] - refl))
            tgt = np.array([*tgt[:2], tgt[2] - extra])
            c_f = np.array([*C_XY, cz_f])
            u_f = (tgt - c_f) / np.linalg.norm(tgt - c_f)
            kink = math.degrees(math.acos(np.clip(np.dot(u_p, u_f), -1, 1)))
            kinks.append(kink)
            fleets.append(math.degrees(math.asin(abs(u_f[1]))))
            keep.append(turns[-1] + kink)
        turns, kinks, fleets, keep = map(np.array, (turns, kinks, fleets, keep))
        worst = (keep.max() + np.degrees(wmax)) * DEG
        print(f"| {gap} / +{extra} | {np.median(turns):.0f} / {turns.max():.0f} | {np.median(kinks):.0f} /"
              f" {kinks.max():.0f} | {fleets.max():.0f} | {math.exp(-0.1 * worst):.2f} /"
              f" {math.exp(-0.25 * worst):.2f} |")
    print("(spools in the current forearm are not exactly point-symmetric; the nearest spool to the"
          " reflected place is used. 'palm turn' is the bend from the finger-entry bore to the line"
          " toward C and replaces today's palm S-curve, which turns 3..120 deg.)")

    # --- 2. bundle size ---
    print("\n## 2. Bundle size (32 tubes, OD 2 mm)")
    r_pack = 6.42  # best known packing of 32 equal circles: R/r ~ 6.42 (Packomania, verify)
    print(f"Ideal packing: D = {2 * r_pack * 1.0:.1f} mm; loose (fill 0.6): "
          f"D = {2 * math.sqrt(32 * 1.0 / 0.6):.1f} mm; 8x4 grid at 2.1 mm: "
          f"{8 * 2.1:.1f} x {4 * 2.1:.1f} mm")

    # --- 3. torque ---
    print("\n## 3. Wrist torque")
    try:
        import mujoco

        m = mujoco.MjModel.from_xml_path(str(WRIST_XML))
        dd = mujoco.MjData(m)
        mujoco.mj_forward(m, dd)
        pid = m.body("palm").id
        mass = float(m.body_subtreemass[pid])
        com = dd.subtree_com[pid] * 1000
        cen = dd.site("wrist_centre").xpos * 1000
        lever = float(np.linalg.norm(com - cen))
        tips = [dd.xpos[m.body(f"{f}_distal").id] * 1000 for f in ("index", "middle", "ring", "little")]
        tip_len = float(np.mean([t[2] - cen[2] for t in tips])) + 20  # + about to the tip
    except Exception as exc:  # noqa: BLE001
        print(f"(MuJoCo not available: {exc}; using stored values)")
        mass, lever, tip_len = 0.289, 71.0, 205.0
    g = 9.81
    print(f"Hand below the wrist (palm + fingers + thumb): {mass * 1000:.0f} g, centre of mass "
          f"{lever:.0f} mm from the wrist centre; fingertips ~{tip_len:.0f} mm")
    t_hand = mass * g * lever / 1000
    print("| case | torque (N*m) |")
    print("|---|---|")
    print(f"| hand only, forearm horizontal | {t_hand:.2f} |")
    for p in (0.5, 1.0):
        for arm, what in ((110, "object in a power grasp, ~110 mm"), (tip_len, "at the fingertips")):
            print(f"| hand + {p} kg {what} | {t_hand + p * g * arm / 1000:.2f} |")
    t_tube = 32 * EI / (40 / wmax) / 1000
    print(f"| sheath bundle spring at max bend (40 mm bent length) | {t_tube:.2f} |")


if __name__ == "__main__":
    main()
