"""12 · Round-2 budget for the V2.1 bench finger (fixes from reports/10-review.md), agent 07, 2026-11-15.

Run: uv run --no-project --with numpy python cad/12_budget.py
  -> out/12/budget.txt, out/12/budget.json, out/12/travel_table.json (THE shared travel/coupling table, read by 09)

Changes vs 07_budget.py:
  #1  pip flex crosses the MCP on the PALM side of an MR63 idler (moment arm +3.235 mm, HELPS the MCP); a rolling
      MR63 redirect roller pre-wraps it. pip ext on the dorsal side. Full moment balance with every arm and the
      ext pretension, plus the old 07 (dorsal, extensor) case to reproduce the review.
  #2  every spool r 5; MCP idler MR63 on a Ø3 inner tube (coupling 0.447); one travel table.
  #5  fuse cup sized to a real crimp tube (2 x 2 mm, crushed ~2.6 x 1.1); trip >= 1.5 x the 80 %-capped strand.
  #6  nested axle (Ø3x0.5 inner tube + Ø4x0.5 sleeves), wire slot on the neutral axis: stress.
  #12 joint friction (brass bushes, DIP bar pins) in eta.
Units mm, N, N*mm, deg. 'verify' = from memory.
"""
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "out" / "12"
OUT.mkdir(parents=True, exist_ok=True)
L = []


def p(*a):
    s = " ".join(str(x) for x in a)
    L.append(s)
    print(s)


def hdr(t):
    p("")
    p("=" * 100)
    p(t)
    p("=" * 100)


D2R = math.pi / 180
LINE_D, LINE_BREAK = 0.47, 445.0  # PE#8, verify (review #11e: 350-445 N)
R = 7.0 + LINE_D / 2  # drum line radius on the 14 x 0.5 ring
RHO = 3.0 + LINE_D / 2  # MR63 race (idler and roller)
R_S = 5.0  # every spool (review #2a)
G = R / R_S
X_MCP, X_PIP = 6.0, 0.9  # lanes at the abd crossing (|x|)
RANGES = {"mcp_abd": (-15.0, 15.0), "mcp_flex": (-10.0, 90.0), "pip": (-5.0, 95.0)}
SERVO = {"SCS0009": dict(stall=226.0, cont=74.0, travel=300.0), "HLS3606M": dict(stall=590.0, cont=150.0, travel=None)}
MARGIN = 15.0  # deg kept for calibration sweeps
T0 = 5.0  # loop pretension per strand
MU_STEEL, ETA_ROLL = 0.12, 0.995
LEVER = {"pinch": dict(mcp=43.1, pip_drive=46.2), "straight": dict(mcp=81.0, pip_drive=50.2)}  # 01 §3

# ------------------------------------------------------------------ joint friction (#12)
hdr("1. Joint friction (brass bushes, bar pins) -> eta_joint")
MU_BUSH = 0.15  # greased brass on stainless (dry 0.3)
# PIP: -x side on a Ø5 brass bush (pin r 2.0) carrying ~half the joint load (~1.1 T); +x MR84 (negligible)
eta_pip_bush = 1 - MU_BUSH * 2.0 * 0.55 / R
# DIP: two Ø5 bushes (pin r 2) + 2 bar pins (r 1) on a bar force ~ (DIP torque / 6.6 arm); DIP torque share ~ 0.75*15/46
f_dip_share = 0.75 * 15.0 / 46.2
loss_dip = MU_BUSH * (2.0 * 1.6 + 1.0 * 2.0) / 6.6 * f_dip_share
eta_pip_joint = eta_pip_bush * (1 - loss_dip)
eta_mcp_joint = 0.998  # MR84 pair
for mu in (0.15, 0.30):
    e = (1 - mu * 2.0 * 0.55 / R) * (1 - mu * (2.0 * 1.6 + 2.0) / 6.6 * f_dip_share)
    p(f"mu {mu:.2f}: PIP bush loss {1 - (1 - mu * 2.0 * 0.55 / R):.3f}, DIP bushes + bar pins {mu * 5.2 / 6.6 * f_dip_share:.3f}"
      f" -> eta_PIP_joint {e:.3f}")
p(f"used (greased, mu {MU_BUSH}): eta_PIP_joint {eta_pip_joint:.3f}, eta_MCP_joint {eta_mcp_joint:.3f}")

# ------------------------------------------------------------------ route efficiency
hdr("2. Route efficiency (sliding deg on steel, rollers)")
ROUTE = {  # name: (sliding deg, rollers) from the spool to the joint drum
    "mcp_flex bench": (0, 0), "pip_flex bench": (0, 2),  # nips on the abd plane: no bend at abd 0
    "mcp_flex bench abd15": (15, 0), "pip_flex bench abd15": (15, 2),
    "mcp_flex hand": (25, 0), "pip_flex hand": (25, 2),  # + forearm comb 15 + palm guide 10 (07/01), wrist straight
}
ETA = {k: math.exp(-MU_STEEL * d * D2R) * ETA_ROLL ** n for k, (d, n) in ROUTE.items()}
for k, v in ETA.items():
    p(f"{k:24s} eta_route {v:.3f}")


# ------------------------------------------------------------------ tip force with every arm (#1)
def tip_force(sv, where, mode, rho_sign=+1, rho=RHO, eta_pj=eta_pip_joint, eta_mj=eta_mcp_joint):
    tau = SERVO[sv]["cont"] if mode == "cont" else SERVO[sv]["stall"] * (0.8 if mode == "s80" else 1.0)
    Tcap = tau / R_S
    em, ep = ETA[f"mcp_flex {where}"], ETA[f"pip_flex {where}"]
    out = {}
    for pose, Lv in LEVER.items():
        lo, hi = 0.0, 200.0
        for _ in range(60):
            F = (lo + hi) / 2
            # PIP: eta_pj * eta_p * Tp * R - T0 * R >= F * Lpd   (ext pretension opposes)
            Tp = (F * Lv["pip_drive"] / eta_pj + T0 * R) / (R * ep)
            ok_p = Tp <= Tcap
            # MCP: own flex + pip flex over the idler (rho_sign +1 palm side = helps) - ext pretensions
            # pip strand tension at the idler ~ Tp * ep (rollers before it); MCP ext + pip ext pretension oppose
            tau_avail = eta_mj * (Tcap * em * R + rho_sign * Tp * ep * rho) - T0 * R - rho_sign * T0 * rho
            ok_m = tau_avail >= F * Lv["mcp"]
            if ok_p and ok_m:
                lo = F
            else:
                hi = F
        F = lo
        Tp = (F * Lv["pip_drive"] / eta_pj + T0 * R) / (R * ep)
        lim = "PIP" if Tp > 0.999 * Tcap else "MCP"
        # MCP margin: available MCP torque / needed at this F
        tau_avail = eta_mj * (Tcap * em * R + rho_sign * Tp * ep * rho) - T0 * R - rho_sign * T0 * rho
        out[pose] = dict(F=F, limit=lim, Tp=Tp, mcp_margin=tau_avail / max(1e-9, F * Lv["mcp"]))
    return out, Tcap


hdr("3. Fingertip force with every moment arm (pinch MCP 30 / PIP 45; straight push)")
p("terms: PIP eta*Tp*R - T0*R >= F*Lpd ; MCP eta*(Tm*R + s*Tp*rho) - T0*R - s*T0*rho >= F*Lmcp, s = +1 palm side, -1 dorsal")
FORCE = {}
for where in ("bench", "hand", "bench abd15"):
    for sv in SERVO:
        for mode in ("stall", "s80", "cont"):
            r, Tcap = tip_force(sv, where, mode)
            FORCE[f"{where}:{sv}:{mode}"] = r
            p(f"{where:12s} {sv:9s} {mode:5s} strand cap {Tcap:6.1f} N | pinch {r['pinch']['F']:5.1f} N ({r['pinch']['limit']}, "
              f"MCP margin x{r['pinch']['mcp_margin']:.1f}) | straight {r['straight']['F']:5.1f} N ({r['straight']['limit']}, "
              f"MCP margin x{r['straight']['mcp_margin']:.2f})")
p("")
p("Reproducing the review (07 geometry: dorsal pip flex, rho 4.235, r_MCP 4, eta_route 0.954/0.959, no joint friction, no T0):")
for sv in SERVO:
    tau = SERVO[sv]["stall"]
    Tm, Tp = tau / (4.0 if sv == 'SCS0009' else 5.0), tau / 5.0
    for pose, Lv in LEVER.items():
        Fp = Tp * 0.954 * R / Lv["pip_drive"]
        Fm = (Tm * 0.959 * R - Tp * 0.954 * 4.235 * (Fp * Lv["pip_drive"] / (Tp * 0.954 * R))) / Lv["mcp"]
        # MCP must also hold the extensor moment of the pip flex at the tension the PIP needs
        F = 0.0
        for Ft in np.linspace(0, 40, 4001):
            Tpn = Ft * Lv["pip_drive"] / (0.954 * R)
            if Tpn > Tp:
                break
            if Tm * 0.959 * R - Tpn * 0.954 * 4.235 >= Ft * Lv["mcp"]:
                F = Ft
        claim = {("SCS0009", "pinch"): 6.8, ("HLS3606M", "pinch"): 17.6, ("SCS0009", "straight"): 4.8, ("HLS3606M", "straight"): 10.1}
        p(f"  07 {sv} {pose}: {F:.1f} N (07 claimed {claim[(sv, pose)]})")
        del Fm

# ------------------------------------------------------------------ travel (#2)
hdr("4. Servo travel (joint-equivalent degrees, then servo degrees on r 5, G = %.3f)" % G)
rng = {k: v[1] - v[0] for k, v in RANGES.items()}
K = {  # firmware coupling: servo command (joint-equivalent deg) = q_own + sum k_j * q_j ; sign from the strand model
    "mcp_abd": {},
    "mcp_flex": {"mcp_abd": X_MCP / R},  # sign set by the CAD check (12_finger.py writes the measured one)
    "pip": {"mcp_flex": RHO / R, "mcp_abd": X_PIP / R},
}
avail_scs = SERVO["SCS0009"]["travel"] - MARGIN
rows = []
for j in ("mcp_abd", "mcp_flex", "pip"):
    own = rng[j]
    coup = {k: abs(v) * rng[k] for k, v in K[j].items()}
    tot_joint = own + sum(coup.values())
    srv = tot_joint * G
    wrist_left = avail_scs - srv
    rows.append(dict(joint=j, own_deg=own, couplings_joint_deg=coup, servo_deg_no_wrist=srv, wrist_budget_servo_deg=wrist_left,
                     wrist_budget_mm=wrist_left * D2R * R_S))
    p(f"{j:9s} own {own:5.1f} + couplings {sum(coup.values()):5.1f} = {tot_joint:6.1f} joint-deg -> {srv:6.1f} servo deg; "
      f"left for wrist on SCS0009 ({avail_scs:.0f}): {wrist_left:6.1f} deg = {wrist_left * D2R * R_S:.2f} mm of strand")
p("07 (dorsal MR84 idler, MCP r 4): pip 235 servo deg before wrist, MCP 280 incl. 3.6 mm wrist (review: infeasible with 09).")
p("09's eye law (review): wrist travel ~ 9.7|x| + 17.4|y| servo deg on r 5 for an eye at (x, y) mm.")

# ------------------------------------------------------------------ fuse cup (#5)
hdr("5. Fuse cup sized to a real crimp (2 x 2 mm crimp tube, ID ~1.4, OD 2.0, L 2.0; doubled PE#8 = 0.94 fits)")
BEAD = dict(w=2.6, h=1.1, l=2.0)  # crushed flat with flat-nose pliers (measure yours)
perim = 2 * (BEAD["w"] + BEAD["h"])
CH = dict(w=2.9, h=2.1, depth=2.3)  # chamber with 0.15 mm + the un-crushed 2.0 OD fits in 2.1
p(f"bead footprint {BEAD['w']} x {BEAD['h']} -> shear perimeter {perim:.1f} mm; chamber {CH['w']} x {CH['h']} x {CH['depth']} deep")
CUP = {}
for sv, tau, orient in (("SCS0009", 15.0, "flat"), ("HLS3606M", 25.0, "on its side")):
    Tcap80 = 0.8 * SERVO[sv]["stall"] / R_S
    need = 1.5 * Tcap80
    t = math.ceil(need / (perim * tau) / 0.2) * 0.2  # whole 0.2 mm layers
    trip = perim * t * tau
    CUP[sv] = dict(lip_t=round(t, 2), trip_N=trip, cap80_N=Tcap80, ratio=trip / Tcap80, stall_N=SERVO[sv]["stall"] / R_S,
                   low_scatter=0.75 * trip / Tcap80, orient=orient)
    p(f"{sv}: 80 % strand {Tcap80:.1f} N, need >= {need:.1f}; lip t {t:.1f} printed {orient} (tau {tau}) -> trip {trip:.0f} N = "
      f"{trip / Tcap80:.2f} x the 80 % cap ({0.75 * trip / Tcap80:.2f} x at -25 % scatter), {trip / (SERVO[sv]['stall'] / R_S):.2f} x stall")
p(f"crimp on doubled PE#8 holds ~70-90 % of {LINE_BREAK:.0f} N = {0.7 * LINE_BREAK:.0f} N or more >= 2 x the HLS trip: the cup trips first.")
p("share reaching the cup after the drum wrap (56 deg on brass, mu 0.12): e^-0.12*0.98 = 0.89 -> external push that trips the PIP")
for sv in SERVO:
    Ts = CUP[sv]["trip_N"] / 0.89
    p(f"  {sv}: strand {Ts:.0f} N -> tip push {Ts * R / LEVER['pinch']['pip_drive']:.1f} N against a holding finger")

# ------------------------------------------------------------------ axle (#6)
hdr("6. MCP axle: Ø3 x 0.5 inner tube (wires, MR63 idler) + Ø4 x 0.5 sleeves (MR84s, cheeks), slot on the neutral axis")


def I_ring(Do, Di):
    return math.pi / 64 * (Do ** 4 - Di ** 4)


def I_slotted(Do, Di, w, n=4000):
    """bending about the axis perpendicular to the slot direction: slot centred on the neutral axis (+y side)."""
    ys = np.linspace(-Do / 2, Do / 2, n)
    dy = ys[1] - ys[0]
    I = 0.0
    for z in ys:  # strip at height z (bending stress ~ z), chord in y
        ro = math.sqrt(max(0.0, (Do / 2) ** 2 - z * z))
        ri = math.sqrt(max(0.0, (Di / 2) ** 2 - z * z))
        width = 2 * (ro - ri)
        if abs(z) < w / 2:  # the slot removes the +y wall where |z| < w/2
            width -= (ro - ri)
        I += width * z * z * dy
    return I


lev = 5.9 - 2.95
P_drive = 120.0 + 0.5 * (120.0 + 17)
P_lat = 120.0
cases = {"07 tube Ø4x0.5 unslotted": I_ring(4, 3) / 2.0,
         "07 tube with -z slot (review: Z 2.62)": 2.62,
         "12 nested Ø4x0.5 + Ø3x0.5, no slot": (I_ring(4, 3) + I_ring(3, 2)) / 2.0,
         "12 nested, 1.6 mm slot at +y (neutral axis)": (I_slotted(4, 3, 1.6) + I_slotted(3, 2, 1.6)) / 2.0}
AX = {}
for k, Z in cases.items():
    s1, s2 = P_drive * lev / Z, P_lat * lev / Z
    AX[k] = dict(Z=Z, drive=s1, lat20=s2)
    p(f"{k:46s} Z {Z:5.2f} mm^3 -> {s1:5.0f} MPa (HLS stall drive), {s2:4.0f} MPa (20 N sideways tip)")
p("the slot sits where the bending stress is ~0 (loads are along z: strand pulls and the sideways-tip couple), so no Kt 2 there.")

# ------------------------------------------------------------------ bearings
hdr("7. Rolling elements at the 120 N class (C0 from NSK via 02: MR63 74 N, MR84 139 N; verify)")
for nm, wrap, C0 in (("MR63 redirect roller (110 deg)", 110, 74), ("MR63 idler, pinch (MCP 30: 84 deg)", 84, 74),
                     ("MR84 MCP lug bearing", None, 139)):
    for sv, T in (("SCS 80 %", 0.8 * 226 / 5), ("HLS 80 %", 0.8 * 590 / 5)):
        F = 2 * T * math.sin(wrap / 2 * D2R) if wrap else T + 0.5 * (T + 17)
        p(f"{nm:38s} {sv}: {F:5.0f} N -> s0 {C0 / F:.2f}")
p("-> SCS: every rolling element s0 >= 1. HLS: MR63s s0 ~0.4: HLS runs need F683ZZ/MR83 (3x8x3, verify C0) or a")
p("   hardened Ø3 dowel redirect (sliding 110 deg, eta 0.79 -> pinch -21 %). MR84 lugs as in 07 (torque cap).")

J = dict(eta_joint=dict(pip=eta_pip_joint, mcp=eta_mcp_joint), eta_route=ETA, force=FORCE, cup=CUP, axle=AX,
         R=R, RHO=RHO, R_S=R_S, G=G, lanes=dict(mcp=X_MCP, pip=X_PIP))
(OUT / "budget.json").write_text(json.dumps(J, indent=1, default=float))

# ------------------------------------------------------------------ THE travel table
TT = {
    "about": "Shared servo travel + coupling table (agent 07, round 2, 2026-11-15). Finger values from cad/12_budget.py; "
             "measured coupling coefficients are re-written by cad/12_finger.py into 'measured_k'. 09 adds the wrist column.",
    "convention": {
        "sign": "positive = closing. servo_cmd_joint_equiv_deg = q_own + sum_j k_j * q_j ; servo_deg = G * that.",
        "G": "servo deg per joint deg = drum line radius / spool line radius",
        "wrist": "09 adds per-eye wrist compensation (servo deg) = |dL_wrist| / r_spool; budget below is what is left",
    },
    "servo": {"SCS0009": {"travel_deg_datasheet": 300.0, "travel_deg_measured": None, "calibration_margin_deg": MARGIN,
                          "note": "300 deg from memory: MEASURE day 1 (review #11a) and fill travel_deg_measured"},
              "HLS3606M": {"multi_turn": None, "note": "multi-turn mode from memory: verify (review #11b)"}},
    "spool_r_mm": R_S, "drum_R_mm": R, "idler_rho_mm": RHO, "G": G,
    "fingers": {},
}
SID = {"index": (1, 2, 3), "middle": (8, 9, 10), "ring": (11, 12, 13), "little": (14, 15, 16)}
for f, (s_pip, s_mcp, s_abd) in SID.items():
    for sid, j in ((s_pip, "pip"), (s_mcp, "mcp_flex"), (s_abd, "mcp_abd")):
        r = next(x for x in rows if x["joint"] == j)
        TT["fingers"][f"{f}_{j}"] = dict(
            servo_id=sid, own_range_deg=list(RANGES[j]), own_joint_deg=r["own_deg"],
            k=K[j], couplings_joint_deg=r["couplings_joint_deg"],
            servo_deg_without_wrist=round(r["servo_deg_no_wrist"], 1),
            wrist_budget_servo_deg_SCS=round(r["wrist_budget_servo_deg"], 1),
            wrist_budget_strand_mm_SCS=round(r["wrist_budget_mm"], 2),
            total_with_wrist_deg=None)
TT["thumb"] = {"note": "not designed in 12; V2 ranges on r 5 with drum 7.235, couplings unknown (thumb designer)",
               **{f"thumb_{j}": dict(own_range_deg=rg, servo_deg_without_wrist=round((rg[1] - rg[0]) * G, 1))
                  for j, rg in {"cmc_rot": (-31, 110), "cmc_flex": (-37, 63), "mcp_flex": (-10, 70), "ip": (-15, 85)}.items()}}
(OUT / "travel_table.json").write_text(json.dumps(TT, indent=1))
(OUT / "budget.txt").write_text("\n".join(L) + "\n")
p("")
p(f"wrote {OUT / 'travel_table.json'}")
