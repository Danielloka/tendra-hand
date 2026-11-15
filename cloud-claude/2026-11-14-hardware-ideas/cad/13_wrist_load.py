"""13 · Wrist load path with bare strands (study 2026-11-15, agent 09, round 2; review 10 #3).

Bare strands make the wrist a compression member: the strand tension that a sheath used to carry now pushes the
palm onto the forearm through the cardan. This script computes the loads and checks V2's cardan against the
redesign "W13" (hand calculations, no FEA; every assumption is a named constant).

Load path (equilibrium of the ring, see report 13 §2): the palm spans push the palm onto the ring via the flex
pins with sum(T) along the palm-span direction; the forearm spans push the forearm onto the ring via the dev
pins with sum(T) along the forearm-span direction; the eye plate is pulled by sum T(u_palm + u_forearm), which the
ring balances. A strand passing the eye at (x, y) also puts a torque T*y about the flex axis and T*x about the
dev axis on the wrist drive (the same moment arms that set the servo compensation).

    uv run --no-project --with numpy python cad/13_wrist_load.py
Writes out/13/wrist_load.json and .txt.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STUDY = HERE.parent
OUT = STUDY / "out" / "13"
spec = importlib.util.spec_from_file_location("kin09", HERE / "09_eyeplate_kin.py")
KIN = importlib.util.module_from_spec(spec)
spec.loader.exec_module(KIN)
K9 = json.load(open(STUDY / "out" / "13" / "kin" / "eyeplate_kin.json"))  # round-2 eyes (shared travel table)
KIN.H_PALM = 30.0

# --- assumptions ---------------------------------------------------------------------------------------------
T_PRE = 5.0  # N per strand (all 32)
T_SCS = 45.2  # N at SCS0009 stall through r 5 (report 02); servo backs off above 80 % after 2 s
T_HLS = 118.0  # N at HLS3606M stall (verify)
FRAC = 0.8
# bearings / bushes (static capacity C0, from memory: verify against NSK / Misumi / igus PDFs)
BEARINGS = {
    "623ZZ 3x10x4 (V2)": dict(C0=220.0, d=3.0, w=4.0, kind="ball"),
    "624ZZ 4x13x5": dict(C0=490.0, d=4.0, w=5.0, kind="ball"),
    "625ZZ 5x16x5": dict(C0=670.0, d=5.0, w=5.0, kind="ball"),
    "6800ZZ 10x19x5": dict(C0=840.0, d=10.0, w=5.0, kind="ball"),
    "HK0509 needle 5x9x9": dict(C0=1400.0, d=5.0, w=9.0, kind="needle (C0 1.4-1.9 kN, verify)"),
    "plain bush iglidur G 5x7x5 (W13)": dict(p_static=80.0, d=5.0, w=5.0, kind="plain, p_static 80 MPa (igus, verify)"),
    "plain bush sintered bronze 5x8x5": dict(p_static=40.0, d=5.0, w=5.0, kind="plain, p_static ~40 MPa (verify)"),
}
PIN_YIELD = {"hardened dowel ISO 8734": 1500.0}  # MPa, through-hardened 58 HRC (conservative)
PIN_LEVER = {"V2 (Ø3, 623 in a printed boss)": (3.0, 3.5), "W13 (Ø5, plain bush + 3 mm steel lug)": (5.0, 4.5)}
MU_BUSH = 0.15  # plain bush on steel, dry, pessimistic
WRIST_DRIVE = {"flex_stall": 3.6, "flex_cont": 1.1}  # N m (V2 report 03, wrist_kin)
# ring out-of-plane bending: each quadrant from an X boss to a Y end, M ~ F * L_q / 8 (load F/2 at each boss)
L_Q = 31.5  # mm, frame path from the flex boss to the dev end
RING = {"V2 printed PA-CF ring, x bar 1.5 x 7": dict(b=1.5, h=7.0, allow=15.0, note="PA-CF sustained ~15 MPa (04 creep rule)"),
        "W13 ring 7075-T6, bar 2.9 x 7": dict(b=2.9, h=7.0, allow=500.0 / 1.5, note="yield 500 / 1.5"),
        "W13 ring 6061-T6, bar 2.9 x 7": dict(b=2.9, h=7.0, allow=275.0 / 1.5, note="yield 275 / 1.5")}
EYE_PLATE = dict(t=1.16, b=19.0, span=22.0, hole_factor=0.8, yield_304_quarter_hard=515.0, yield_304_half_hard=760.0)
M16_SHEAR_AREA = 1.27  # mm^2 (M1.6 stress area)

LOG = []


def p(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    LOG.append(s)


def hdr(t):
    p("")
    p("=" * 96)
    p(t)
    p("=" * 96)


def load_cases(joints):
    """Per-joint FLEX strand tension (N) for each case; ext strands at pretension."""
    cases = {}
    flexT = lambda T: {j["joint"]: T for j in joints}  # noqa: E731
    cases["SCS 80 %, all 16 flex strands (fist squeeze)"] = flexT(FRAC * T_SCS)
    cases["SCS power grasp, 12 strands at 50 %"] = {j["joint"]: (0.5 * T_SCS if "abd" not in j["joint"] else 0.0) for j in joints}
    cases["SCS pinch: index pip+mcp, 4 thumb at 80 %"] = {
        j["joint"]: (FRAC * T_SCS if j["joint"] in ("index_pip", "index_mcp_flex") or j["joint"].startswith("thumb") else 0.0)
        for j in joints}
    cases["HLS 80 %, all 16 flex strands"] = flexT(FRAC * T_HLS)
    return cases


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    joints = K9["joints"]
    eye_tab = {r["name"]: r for r in K9["eye_table"]}
    strands = KIN.load_strands()
    # apply 09's slot map to the spool tangents is not needed: in the axial layout the wrist spans are fixed by the eyes
    RES = {"cases": {}}

    hdr("1. Load cases: compression through the wrist, bearing loads, torque on the wrist drive")
    poses = KIN.wrist_grid()
    for name, ft in load_cases(joints).items():
        # strand tensions (flex + ext)
        T = {}
        for j in joints:
            T[f"{j['joint']}_flex"] = ft[j["joint"]] + T_PRE
            T[f"{j['joint']}_ext"] = T_PRE
        # map to strand names (mcp_flex joints: <f>_mcp_flex_flex / _ext)
        tens = []
        for st in strands:
            key = f"{st['joint']}_{st['role']}"
            tens.append(T[key])
        tens = np.array(tens)
        Fax = float(tens.sum())
        eyes = np.array([eye_tab[st["name"]]["eye"] for st in strands])
        xc, yc = (tens[:, None] * eyes).sum(0) / Fax
        M_flex = float((tens * eyes[:, 1]).sum()) / 1000  # N m about X (palm-span arm = y)
        M_dev = float((tens * eyes[:, 0]).sum()) / 1000
        # bearing split: two flex bearings at x = +-13 (resultant at xc), two dev bearings at y = +-21 (resultant at yc)
        Fb_flex = Fax / 2 * (1 + abs(xc) / 13.0)
        Fb_dev = Fax / 2 * (1 + abs(yc) / 21.0)
        # eye plate load over the wrist grid (ring frame): sum T (u_p + u_f)
        best = (0, None, None)
        for f, d in poses:
            Rr, Rp = KIN.R_ring(f, d), KIN.R_palm(f, d)
            tot = np.zeros(3)
            for st, t, e in zip(strands, tens, eyes):
                E, C, F = KIN.guide_points(st, e)
                Ew = Rr @ E
                up = Rp @ C - Ew
                uf = F - Ew
                tot += t * (up / np.linalg.norm(up) + uf / np.linalg.norm(uf))
            tot_r = Rr.T @ tot
            if np.linalg.norm(tot_r) > best[0]:
                best = (float(np.linalg.norm(tot_r)), tot_r, (f, d))
        # palm plate (palm frame) and forearm plate (forearm frame) loads: sum T (u_in + u_out) at q0 and over the grid
        pl_max = {"palm": (0, None), "fore": (0, None)}
        for f, d in poses + [(0.0, 0.0)]:
            Rr, Rp = KIN.R_ring(f, d), KIN.R_palm(f, d)
            tp, tf = np.zeros(3), np.zeros(3)
            for st, t, e in zip(strands, tens, eyes):
                E, C, F = KIN.guide_points(st, e)
                Ew_p = Rp.T @ (Rr @ E)  # eye in the palm frame
                tp += t * ((Ew_p - C) / np.linalg.norm(Ew_p - C) + (st["A"] - C) / np.linalg.norm(st["A"] - C))
                Ew_f = Rr @ E
                tf += t * ((Ew_f - F) / np.linalg.norm(Ew_f - F) + (st["S"] - F) / np.linalg.norm(st["S"] - F))
            for k, v in (("palm", tp), ("fore", tf)):
                if np.linalg.norm(v) > pl_max[k][0]:
                    pl_max[k] = (float(np.linalg.norm(v)), v.tolist())
        RES["cases"][name + " plates"] = pl_max
        p(f"  palm plate load max {pl_max['palm'][0]:5.0f} N, vector (palm frame) {np.round(pl_max['palm'][1], 0)}; "
          f"forearm plate {pl_max['fore'][0]:5.0f} N {np.round(pl_max['fore'][1], 0)}")
        RES["cases"][name] = dict(F_axial=Fax, resultant_xy=[float(xc), float(yc)], M_flex_Nm=M_flex, M_dev_Nm=M_dev,
                                  F_bearing_flex=Fb_flex, F_bearing_dev=Fb_dev, eye_plate_max_N=best[0],
                                  eye_plate_vec_ring=best[1].tolist(), eye_plate_pose=best[2])
        p(f"{name}:")
        p(f"  axial compression {Fax:6.0f} N (incl. 32 x {T_PRE:.0f} N pretension); line of action at ({xc:+.2f}, {yc:+.2f}) mm")
        p(f"  per bearing: flex {Fb_flex:5.0f} N, dev {Fb_dev:5.0f} N   |  torque on the wrist drive: flex {M_flex:+.2f} N m, "
          f"dev {M_dev:+.2f} N m (drive: flex {WRIST_DRIVE['flex_stall']} stall / {WRIST_DRIVE['flex_cont']} cont)")
        p(f"  eye plate load: max {best[0]:5.0f} N at flex {best[2][0]:.0f} dev {best[2][1]:.0f}; ring-frame vector "
          f"({best[1][0]:+.0f}, {best[1][1]:+.0f}, {best[1][2]:+.0f}) N")

    RES["cases"] = {k: v for k, v in RES["cases"].items()}
    worst = RES["cases"]["HLS 80 %, all 16 flex strands"]
    scs = RES["cases"]["SCS 80 %, all 16 flex strands (fist squeeze)"]

    hdr("2. Bearings / bushes (static), per bearing; s0 = C0 / P0")
    rows = []
    for nm, b in BEARINGS.items():
        cap = b.get("C0") or b["p_static"] * b["d"] * b["w"]
        s_scs = cap / max(scs["F_bearing_flex"], scs["F_bearing_dev"])
        s_hls = cap / max(worst["F_bearing_flex"], worst["F_bearing_dev"])
        rows.append(dict(name=nm, capacity_N=cap, s0_SCS=s_scs, s0_HLS=s_hls, kind=b["kind"]))
        p(f"  {nm:36s} capacity {cap:6.0f} N  s0 SCS {s_scs:4.2f}  HLS {s_hls:4.2f}   ({b['kind']})")
    RES["bearings"] = rows
    fr = MU_BUSH * 2 * worst["F_bearing_flex"] * 2.5 / 1000
    fr_s = MU_BUSH * 2 * scs["F_bearing_flex"] * 2.5 / 1000
    p(f"  plain-bush friction torque on the wrist axis (mu {MU_BUSH}, r 2.5): SCS {fr_s:.2f} N m, HLS {fr:.2f} N m "
      f"(only while the hand squeezes at full force)")
    RES["bush_friction_Nm"] = dict(SCS=fr_s, HLS=fr)

    hdr("3. Pins (hardened dowel), bending at the lug-to-bearing gap: sigma = F * lever / (pi d^3 / 32)")
    RES["pins"] = {}
    for nm, (d, lev) in PIN_LEVER.items():
        Z = math.pi * d**3 / 32
        for cs, F in (("SCS", scs["F_bearing_flex"]), ("HLS", worst["F_bearing_flex"])):
            s = F * lev / Z
            tau = F / (math.pi * d * d / 4)
            p(f"  {nm:40s} {cs}: bending {s:6.0f} MPa, shear {tau:4.0f} MPa (yield ~{PIN_YIELD['hardened dowel ISO 8734']:.0f})")
            RES["pins"][f"{nm} {cs}"] = dict(bending=s, shear=tau)

    hdr("4. Ring out-of-plane bending, M ~ F * L_q / 8 per quadrant (approx.)")
    RES["ring"] = {}
    for nm, r in RING.items():
        Z = r["b"] * r["h"] ** 2 / 6
        for cs, F in (("SCS", scs["F_axial"]), ("HLS", worst["F_axial"])):
            s = F * L_Q / 8 / Z
            p(f"  {nm:38s} {cs}: {s:6.0f} MPa vs allowed {r['allow']:.0f} ({r['note']}) -> SF {r['allow'] / s:4.2f}")
            RES["ring"][f"{nm} {cs}"] = dict(sigma=s, sf=r["allow"] / s)

    hdr("5. Eye plate: bending between the ring bosses and the mounting")
    ep = EYE_PLATE
    RES["eye_plate"] = {}
    for cs, c in (("SCS", scs), ("HLS", worst)):
        v = np.array(c["eye_plate_vec_ring"])
        Fin, Fz = float(np.hypot(v[0], v[1])), float(abs(v[2]))
        Zin = ep["t"] * ep["b"] ** 2 / 6 * ep["hole_factor"]
        Zout = ep["b"] * ep["t"] ** 2 / 6 * ep["hole_factor"]
        s_in = Fin * ep["span"] / 8 / Zin
        s_out = Fz * ep["span"] / 8 / Zout
        # 09 mount: 2 blocks, each 2 x M1.6 into the printed ring boss
        tau_m16 = math.hypot(Fin, Fz) / 4 / M16_SHEAR_AREA
        bear_print = math.hypot(Fin, Fz) / 4 / (1.6 * 4.0)
        # W13 mount: 4 tabs, each bearing 1.16 x 3.0 on an aluminium ledge
        bear_tab = math.hypot(Fin, Fz) / 4 / (ep["t"] * 3.0)
        p(f"  {cs}: in-plane {Fin:5.0f} N -> {s_in:4.0f} MPa; out-of-plane {Fz:4.0f} N -> {s_out:4.0f} MPa "
          f"(304 1/4-hard {ep['yield_304_quarter_hard']:.0f}, 1/2-hard {ep['yield_304_half_hard']:.0f})")
        p(f"      09 mount (4 x M1.6 into a printed boss): screw shear {tau_m16:4.0f} MPa, thread bearing on the print "
          f"{bear_print:4.0f} MPa (PA-CF ~15 sustained) -> FAILS; W13 tabs on aluminium ledges: {bear_tab:4.0f} MPa (6061 yield 275)")
        RES["eye_plate"][cs] = dict(F_in=Fin, F_z=Fz, sigma_in=s_in, sigma_out=s_out, m16_shear=tau_m16,
                                    print_bearing=bear_print, tab_bearing=bear_tab)

    hdr("6. Lug plates (3 mm 304, pin pressed) and their foot on the printed flange / base")
    for cs, c in (("SCS", scs), ("HLS", worst)):
        F = max(c["F_bearing_flex"], c["F_bearing_dev"])
        p(f"  {cs}: pin bearing in the plate {F / (5 * 3):4.0f} MPa (304 yield 215-515); foot 3 x 22 mm on the print "
          f"{F / (3 * 22):4.1f} MPa (PA-CF ~15 sustained, PETG ~9)")
    RES["lug"] = {cs: dict(pin_bearing=max(c["F_bearing_flex"], c["F_bearing_dev"]) / 15,
                           foot=max(c["F_bearing_flex"], c["F_bearing_dev"]) / 66) for cs, c in (("SCS", scs), ("HLS", worst))}

    hdr("7. Alternatives ruled out (numbers)")
    p("  Central compression post / ball at the wrist centre: the 32 strands occupy the centre. A ring of eyes around a")
    p("  Ø10 post puts every eye at r >= 6.5 mm -> compensation travel 10|x| + 20|y| up to ~190 deg: no SCS servo fits.")
    p("  Spherical shell bearing around the bundle (Ø>=40, engaged over +-52 deg): collides with the cardan lugs and")
    p("  the push-rod balls inside the 30 mm gap; not buildable at hobby scale.")
    p("  Keep sheaths across the wrist (report 09 §7 fallback): the coil housings carry the compression instead (V2 path).")
    (OUT / "wrist_load.json").write_text(json.dumps(RES, indent=1, default=float))
    (OUT / "wrist_load.txt").write_text("\n".join(LOG))


if __name__ == "__main__":
    main()
