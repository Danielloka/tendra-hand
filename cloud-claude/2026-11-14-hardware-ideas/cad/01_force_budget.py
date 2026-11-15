"""01 - first-principles force, friction, stiffness and strength budget for the Tendra drivetrain.

Agent 01 (first-principles architect), study 2026-11-14. Hand calculations, no FEA.
Run:  uv run --no-project --with numpy python cad/01_force_budget.py
Prints all tables and writes out/01/01_force_budget.txt (+ .json).

What it does
1. Reads the V2 routes (palm + forearm fan) and adds the finger/thumb internal bends -> total wrap per strand.
2. Efficiency eta = exp(-mu * theta) per strand for the V2 drivetrain and for the proposed architectures.
3. Planar index-finger statics (tau = J^T F) in a tip-pinch pose, with the DIP coupled to the PIP (0.75),
   so the fingertip force is limited by whichever joint runs out of torque first.
4. Actuator / spool ladder: SCS0009, HLS3606M, STS3215 at different spool radii; on-axis eyelet vs an
   "FDP" idler pulley at the MCP that lets the PIP strand help the MCP.
5. Loop stiffness: braid + sheath in series -> how far the finger sags under load.
6. Strength: strand tension, line + knot/clamp safety factor, groove pressure, bearing and shaft loads.
7. Wrist eye-plate (bare strands through the gimbal mid-plane): length change per strand, check that a
   point-symmetric flex/ext pair keeps the loop length (common mode) constant.

Every assumption is a named constant. Units: mm, N, N*mm (printed as N*m), degrees in tables.
"""
from __future__ import annotations

import io
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STUDY = HERE.parent
V2 = STUDY.parent / "2026-11-09-v2-hand-cad"
OUTDIR = STUDY / "out" / "01"
OUTDIR.mkdir(parents=True, exist_ok=True)
BUF = io.StringIO()
RESULT: dict = {}


def p(*a):
    print(*a)
    print(*a, file=BUF)


def hdr(t):
    p("")
    p("=" * 100)
    p(t)
    p("=" * 100)


# ------------------------------------------------------------------------------------------------
# Assumptions
# ------------------------------------------------------------------------------------------------
R_DRUM = 7.0              # V2 joint drum radius (every servo joint)
MU_PTFE = 0.10            # UHMWPE braid sliding in PTFE (tendon-mechanics.md 0.05-0.10; pessimistic 0.15)
MU_PTFE_HI = 0.15
MU_STEEL = 0.10           # braid on polished steel pin / eyelet / hypodermic tube
ETA_IDLER = 0.99          # one ball-bearing idler (bearing friction only), per idler
T0 = 4.0                  # N pretension per strand

# Servo data (datasheets as quoted in research/references/hands/actuators-and-control.md)
SERVOS = {
    # name: stall N*m, rated/continuous N*m, no-load deg/s, mass g, price EUR, travel deg (None = multi-turn),
    #       spline OD mm (sets the smallest spool you can put on it)
    "SCS0009": dict(stall=0.226, cont=0.074, speed=600, mass=13.2, eur=9, travel=300, spline=3.95),
    "HLS3606M": dict(stall=0.59, cont=0.15, speed=666, mass=20.6, eur=28, travel=None, spline=4.95),
    "STS3215-12V": dict(stall=2.94, cont=0.9, speed=270, mass=55, eur=16, travel=None, spline=5.9),
}
# Smallest practical spool radius (line centre) on each spline:
#   metal hub OD = spline OD + 2 x 0.8 mm wall (brass/alu sleeve), + half the line diameter.
LINE_D = 0.46             # 80 lb braid as the default for anything above SCS0009


def r_spool_min(spline):
    return (spline + 1.6) / 2 + LINE_D / 2


# Line data: name: (diameter mm, nominal break N). EA ~ 39 x break (1/3 break at 0.85 % strain, tendon-mechanics §1)
LINES = {
    "50 lb braid": (0.36, 222),
    "80 lb braid": (0.46, 356),
    "150 lb braid": (0.60, 667),
    "200 lb braid": (0.75, 890),
}
EA_PER_BREAK = 39.0       # secant EA / break load (pre-stretched braid, working range)
KNOT_EFF = 0.55           # knot in braid keeps ~40-60 %
CAPSTAN_ANCHOR_EFF = 0.9  # 3 wraps on a steel post + clamp screw: the clamp sees e^(-0.1*6pi) = 15 %, line ~90 %

# Sheaths (review 05): A = pi/4 (2^2 - 1^2) = 2.36 mm^2
A_SHEATH = math.pi / 4 * (2.0**2 - 1.0**2)
E_PTFE, E_PEEK = 500.0, 3600.0       # MPa
K_COIL_PER_MM = 2.0e5 * 2.0           # steel coil housing ~ E_steel x 2 mm^2 effective (closed coils): N per mm/length

# Index finger (params.py FINGERS["index"]): proximal 42, middle 24, distal 23; pad centre 15 mm past the DIP,
# 6 mm to the palm side of the bone axis (pad surface).
L1, L2, L3 = 42.0, 24.0, 23.0
PAD_ALONG, PAD_OFF = 15.0, 6.0
DIP_RATIO = 0.75

# ------------------------------------------------------------------------------------------------
# 1. Wrap angles of the V2 routes
# ------------------------------------------------------------------------------------------------


def turning(points):
    v = np.diff(np.asarray(points, float), axis=0)
    v = v[np.linalg.norm(v, axis=1) > 1e-9]
    v /= np.linalg.norm(v, axis=1)[:, None]
    return float(np.degrees(np.arccos(np.clip((v[:-1] * v[1:]).sum(1), -1, 1))).sum())


# finger / thumb internal sliding bends at a TIP-PINCH pose (MCP 30, PIP 45) and at a FIST (MCP 90, PIP 95).
# pip strands cross the MCP over the 18G eyelet (bend = q_mcp). Thumb: report 02 §4 (neck 20, cmc nip <= 60,
# MCP nip <= 45); pinch uses ~half of the maxima.
INTERNAL = {  # joint-type: (pinch deg, fist deg)
    "mcp_flex": (0, 0), "pip": (30, 90), "mcp_abd": (0, 0),
    "thumb_cmc_rot": (20, 20), "thumb_cmc_flex": (20, 20), "thumb_mcp_flex": (50, 80), "thumb_ip": (75, 125),
}


def joint_of(name):
    for k in ("thumb_cmc_rot", "thumb_cmc_flex", "thumb_mcp_flex", "thumb_ip"):
        if name.startswith(k):
            return k
    for k in ("mcp_flex", "mcp_abd", "pip"):
        if f"_{k}_" in name:
            return k
    raise KeyError(name)


def v2_wraps():
    routes = json.load(open(V2 / "out/assembly/routes.json"))["strands"]
    fan = {s["name"]: s for s in json.load(open(V2 / "out/assembly/fan_routes.json"))["strands"]}
    rows = []
    for s in routes:
        j = joint_of(s["name"])
        palm = s["turn_deg"]
        fanturn = turning(fan[s["name"]]["points"]) if s["name"] in fan else 0.0
        ip, ifist = INTERNAL[j]
        rows.append(dict(name=s["name"], joint=j, role=s["role"], palm=palm, fan=fanturn, pinch=palm + fanturn + ip,
                         fist=palm + fanturn + ifist, sheath_len=s["length"] + (fan[s["name"]]["length"] if s["name"] in fan else 0) + 75))
    return rows


def eta(theta_deg, mu):
    return math.exp(-mu * math.radians(theta_deg))


hdr("1. V2 drivetrain: total sliding wrap per FLEX strand (palm + forearm fan + finger internal), wrist straight")
rows = v2_wraps()
flex = [r for r in rows if r["role"] == "flex"]
p(f"{'strand':26s} {'palm':>6s} {'fan':>6s} {'pinch':>6s} {'fist':>6s} {'eta pinch mu.10':>16s} {'mu.15':>6s}")
for r in flex:
    p(f"{r['name']:26s} {r['palm']:6.0f} {r['fan']:6.0f} {r['pinch']:6.0f} {r['fist']:6.0f} "
      f"{eta(r['pinch'], MU_PTFE):16.2f} {eta(r['pinch'], MU_PTFE_HI):6.2f}")
allw = np.array([r["pinch"] for r in rows])
p(f"All 32 strands, pinch pose: wrap min/median/max = {allw.min():.0f} / {np.median(allw):.0f} / {allw.max():.0f} deg"
  f"  -> eta(mu 0.10) = {eta(allw.max(), .1):.2f} / {eta(np.median(allw), .1):.2f} / {eta(allw.min(), .1):.2f}")
p("Add up to 58 deg at a wrist extreme (review 05).")
RESULT["v2_wrap_pinch"] = dict(min=float(allw.min()), median=float(np.median(allw)), max=float(allw.max()))
idx_mcp = next(r for r in rows if r["name"] == "index_mcp_flex_flex")
idx_pip = next(r for r in rows if r["name"] == "index_pip_flex")

# ------------------------------------------------------------------------------------------------
# 2. Proposed routing architectures: wrap budget per index flex strand (pinch pose)
# ------------------------------------------------------------------------------------------------
hdr("2. Wrap / efficiency budget per architecture, index flex strands, tip-pinch pose, wrist straight")
# Each entry: list of (sliding deg, mu) + number of ball-bearing idlers.
ARCH = {
    "V2 (PTFE Bowden everywhere)": {
        "mcp": ([(idx_mcp["pinch"], MU_PTFE)], 0),
        "pip": ([(idx_pip["pinch"] - 30, MU_PTFE), (30, MU_STEEL)], 0),
    },
    "A: bare strands, rigid path, steel pins": {
        # spool exit -> forearm eye 15 deg, wrist eye plate 0 deg (straight), palm fan 2 x 25 deg on steel pins,
        # knuckle entry 10 deg; pip crosses MCP at the eyelet (30 deg)
        "mcp": ([(15, MU_STEEL), (50, MU_STEEL), (10, MU_STEEL)], 0),
        "pip": ([(15, MU_STEEL), (50, MU_STEEL), (10, MU_STEEL), (30, MU_STEEL)], 0),
    },
    "A+: bare strands on idlers (MR63 / 3 mm pulleys)": {
        # same path, the two palm turns and the MCP crossing on ball-bearing idlers
        "mcp": ([(15, MU_STEEL), (10, MU_STEEL)], 2),
        "pip": ([(15, MU_STEEL), (10, MU_STEEL)], 3),
    },
}
arch_eta = {}
for name, d in ARCH.items():
    out = {}
    for j, (slides, nid) in d.items():
        e = ETA_IDLER**nid
        th = 0.0
        for deg, mu in slides:
            e *= eta(deg, mu)
            th += deg
        out[j] = e
        out[j + "_deg"] = th
    arch_eta[name] = out
    p(f"{name:52s} MCP-flex: {out['mcp_deg']:4.0f} deg sliding, eta {out['mcp']:.2f} | "
      f"PIP-flex: {out['pip_deg']:4.0f} deg, eta {out['pip']:.2f}")
p("Wrist bent 50 deg adds ~50 deg on the eye plate for A/A+ (eta x 0.92) and on the sheath for V2 (x 0.92 at mu .1).")
RESULT["arch_eta"] = arch_eta

# ------------------------------------------------------------------------------------------------
# 3. Planar index statics in a tip pinch
# ------------------------------------------------------------------------------------------------


def finger_pts(qm, qp):
    qd = DIP_RATIO * qp
    a1, a2, a3 = math.radians(qm), math.radians(qm + qp), math.radians(qm + qp + qd)
    # z along the finger at q=0, y up = dorsal; flexion turns toward -y (palm)
    def u(a):
        return np.array([math.cos(a), -math.sin(a)])  # (z, y)
    m = np.zeros(2)
    pip = m + L1 * u(a1)
    dip = pip + L2 * u(a2)
    n_palm = np.array([-math.sin(a3), -math.cos(a3)])  # palm-side normal of the distal phalanx
    pad = dip + PAD_ALONG * u(a3) + PAD_OFF * n_palm
    return m, pip, dip, pad, n_palm


def moment_arms(qm=30.0, qp=45.0):
    """Joint torques needed per newton of tip force. The object pushes on the pad along +n (into the finger,
    i.e. opposite the palm normal); the finger must hold it with flexion torques."""
    m, pip, dip, pad, n_palm = finger_pts(qm, qp)
    F = -n_palm  # force ON the finger from the object (pointing dorsal)

    def cross(r, f):
        return r[0] * f[1] - r[1] * f[0]
    # flexion torque needed = -(moment of the object force about the joint), sign so that it is positive
    a = {}
    for k, j in (("mcp", m), ("pip", pip), ("dip", dip)):
        a[k] = abs(cross(pad - j, F))
    a["pip_drive"] = a["pip"] + DIP_RATIO * a["dip"]   # the PIP servo carries PIP + 0.75 DIP (virtual work)
    return a


hdr("3. Tip-pinch statics (index, MCP 30 / PIP 45 / DIP 34 deg). Torque per newton of tip force (N*mm/N)")
A = moment_arms()
p(f"moment arms: MCP {A['mcp']:.1f} mm, PIP {A['pip']:.1f} mm, DIP {A['dip']:.1f} mm, "
  f"PIP drive (PIP + 0.75 DIP) {A['pip_drive']:.1f} mm")
A_straight = moment_arms(0.0, 0.0)
p(f"straight finger, force normal to the pad: MCP {A_straight['mcp']:.1f}, PIP drive {A_straight['pip_drive']:.1f} mm")
RESULT["moment_arms_pinch"] = A


ETA_TURN = 0.78   # 2:1 reeving: 180 deg turnaround over a hardened 3 mm pin, mu 0.08 -> e^(-0.08 pi)


def tip_force(T_max, eta_m, eta_p, rho=0.0, r_d=R_DRUM, arms=A, reeve=(1, 1)):
    """Max tip force with each flex strand limited to T_max (N) at the servo.
    reeve = (MCP, PIP): 1 = plain drum, 2 = 2:1 reeving at that joint (moment arm r_d * (1 + ETA_TURN)).
    rho = FDP idler radius at the MCP: the PIP flex strand passes the MCP on the palm side and adds rho*T2.
    MCP torque = r_m*eta_m*T1 + rho*eta_p*T2,  PIP drive torque = r_p*eta_p*T2 (PIP + 0.75 DIP)."""
    r_m = r_d * (1 + ETA_TURN) if reeve[0] == 2 else r_d
    r_p = r_d * (1 + ETA_TURN) if reeve[1] == 2 else r_d
    F_pip = r_p * eta_p * T_max / arms["pip_drive"]
    # T2 = what the PIP needs at force F; its MCP share rho*eta*T2 reduces what the MCP strand must give
    k2 = rho * eta_p * arms["pip_drive"] / (r_p * eta_p)
    F_mcp = r_m * eta_m * T_max / max(arms["mcp"] - k2, 1e-9)
    return min(F_pip, F_mcp), F_mcp, F_pip


# ------------------------------------------------------------------------------------------------
# 4. Actuator ladder
# ------------------------------------------------------------------------------------------------
hdr("4. Force ladder: index tip force at stall and continuous, PINCH pose and STRAIGHT push")
V2E = arch_eta["V2 (PTFE Bowden everywhere)"]
AE = arch_eta["A: bare strands, rigid path, steel pins"]
APE = arch_eta["A+: bare strands on idlers (MR63 / 3 mm pulleys)"]
T_CAP = 250.0   # N, max strand tension we allow (200 lb braid with a capstan anchor: SF ~3)
ladder = [
    # label, servo, r_spool, etas, rho, reeve
    ("0  V2 as designed (SCS0009, r 5, PTFE)", "SCS0009", 5.0, V2E, 0.0, (1, 1)),
    ("1  + A: bare strands, rigid path, steel pins", "SCS0009", 5.0, AE, 0.0, (1, 1)),
    ("2  + A+: idlers at the big bends", "SCS0009", 5.0, APE, 0.0, (1, 1)),
    ("3  + 2:1 reeving at the PIP only (r_s 5.5)", "SCS0009", 5.5, APE, 0.0, (1, 2)),
    ("3b + 2:1 reeving at MCP and PIP (r_s 5.5)", "SCS0009", 5.5, APE, 0.0, (2, 2)),
    ("4  HLS3606M, r 5, V2 routing (review's HLS case)", "HLS3606M", 5.0, V2E, 0.0, (1, 1)),
    ("5  HLS3606M, r 5, A+", "HLS3606M", 5.0, APE, 0.0, (1, 1)),
    ("6  HLS3606M, r 5, A+, 2:1 reeving PIP only", "HLS3606M", 5.0, APE, 0.0, (1, 2)),
    ("6b HLS3606M, r 5, A+, 2:1 reeving MCP + PIP", "HLS3606M", 5.0, APE, 0.0, (2, 2)),
    ("7  HLS3606M, r 3.7 metal hub, A+, reeve PIP", "HLS3606M", 3.7, APE, 0.0, (1, 2)),
    ("7b HLS3606M, r 3.7 metal hub, A+, reeve both", "HLS3606M", 3.7, APE, 0.0, (2, 2)),
    ("8  STS3215 12 V, r 7, A+, 2:1, capped 250 N", "STS3215-12V", 7.0, APE, 0.0, (2, 2)),
    ("9  SCS0009 r 5, A+, FDP idler rho 3 (no reeving)", "SCS0009", 5.0, APE, 3.0, (1, 1)),
]
p(f"{'option':50s} {'T N':>5s} {'pinch':>6s} {'p.cont':>6s} {'lim':>4s} {'straight':>8s} {'lim':>4s} {'joint deg/s':>11s} {'servo deg/100':>13s}")
ladder_out = []
for lab, sv, rs, e, rho, rv in ladder:
    S = SERVOS[sv]
    T = min(S["stall"] * 1e3 / rs, T_CAP)
    Tc = min(S["cont"] * 1e3 / rs, T_CAP)
    F, Fm, Fp = tip_force(T, e["mcp"], e["pip"], rho, reeve=rv)
    Fc, _, _ = tip_force(Tc, e["mcp"], e["pip"], rho, reeve=rv)
    Fs, Fsm, Fsp = tip_force(T, e["mcp"], e["pip"], rho, reeve=rv, arms=A_straight)
    lim = "MCP" if Fm < Fp else "PIP"
    lims = "MCP" if Fsm < Fsp else "PIP"
    G = R_DRUM * (2 if rv[1] == 2 else 1) / rs
    travel = 100 * G + (rho / R_DRUM) * 100 * G
    ladder_out.append(dict(option=lab, servo=sv, r_spool=rs, T=T, tip_pinch=F, tip_pinch_cont=Fc, tip_straight=Fs,
                           limit=lim, G=G, joint_speed=S["speed"] / G, servo_travel_per_100=travel))
    p(f"{lab:50s} {T:5.0f} {F:6.1f} {Fc:6.1f} {lim:>4s} {Fs:8.1f} {lims:>4s} {S['speed'] / G:11.0f} {travel:13.0f}")
p("T = strand tension at stall (capped at 250 N). pinch = MCP 30 / PIP 45; straight = finger straight, force normal")
p("to the pad. p.cont = at the servo's rated (continuous) torque. G = servo deg per joint deg. SCS0009 travel <= 300 deg;")
p("HLS3606M and STS3215 are multi-turn. Reeving: segment 2 loses 22 % on a sliding 3 mm turnaround pin (ETA_TURN).")
RESULT["ladder"] = ladder_out

hdr("4d. BALANCED designs: own spool per joint (MCP = smallest metal hub, plain drum; PIP = 2:1 reeved drum)")
p("Why: the MCP drum has the pip strands passing through its middle, so a 2-groove reeved MCP does not fit in the")
p("8.8 mm between the MCP bearings. The PIP drum has room (7 mm). Each servo has its own spool, so each joint gets")
p("its own ratio. In an antagonistic loop the ratio is capped by the EXTENSOR path (its spool radius >= hub minimum),")
p("so reeving must double both strands (shared 2-groove track, as below).")


def balanced(sv, r_m, r_p, reeve_p=True, eta_m=None, eta_p=None, cap=T_CAP, cont=False):
    S = SERVOS[sv]
    tq = S["cont"] if cont else S["stall"]
    eta_m = APE["mcp"] if eta_m is None else eta_m
    eta_p = APE["pip"] if eta_p is None else eta_p
    Tm, Tp = min(tq * 1e3 / r_m, cap), min(tq * 1e3 / r_p, cap)
    a_p = R_DRUM * (1 + ETA_TURN) if reeve_p else R_DRUM
    res = {}
    for pose, arms in (("pinch", A), ("straight", A_straight)):
        Fm = R_DRUM * eta_m * Tm / arms["mcp"]
        Fp = a_p * eta_p * Tp / arms["pip_drive"]
        res[pose] = (min(Fm, Fp), "MCP" if Fm < Fp else "PIP")
    Gm, Gp = R_DRUM / r_m, (2 * R_DRUM if reeve_p else R_DRUM) / r_p
    return Tm, Tp, res, Gm, Gp, S["speed"] / max(Gm, Gp)


designs = [
    ("B0 SCS0009: MCP r 3.0 hub, PIP r 5.5 reeved", "SCS0009", 3.0, 5.5),
    ("B1 HLS3606M: MCP r 3.5 hub, PIP r 5.0 reeved", "HLS3606M", 3.5, 5.0),
    ("B2 STS3215 12 V: MCP r 7, PIP r 7 reeved, 250 N cap", "STS3215-12V", 7.0, 7.0),
    ("B3 STS3215 12 V: as B2, 300 N cap (200 lb line)", "STS3215-12V", 7.0, 7.0),
]
p(f"{'design':52s} {'Tm':>4s} {'Tp':>4s} {'pinch':>6s} {'cont':>5s} {'lim':>4s} {'straight':>8s} {'G mcp/pip':>9s} {'deg/s':>6s}")
bal = []
for lab, sv, rm, rp in designs:
    cap = 300.0 if "300" in lab else T_CAP
    Tm, Tp, r, Gm, Gp, spd = balanced(sv, rm, rp, cap=cap)
    _, _, rc, _, _, _ = balanced(sv, rm, rp, cap=cap, cont=True)
    bal.append(dict(design=lab, Tm=Tm, Tp=Tp, pinch=r["pinch"][0], pinch_cont=rc["pinch"][0], straight=r["straight"][0],
                    G=(Gm, Gp), speed=spd))
    p(f"{lab:52s} {Tm:4.0f} {Tp:4.0f} {r['pinch'][0]:6.1f} {rc['pinch'][0]:5.1f} {r['pinch'][1]:>4s} "
      f"{r['straight'][0]:8.1f} {Gm:4.2f}/{Gp:4.2f} {spd:6.0f}")
p("SCS0009 travel check B0: MCP 100 deg x 2.33 = 233 deg, PIP 100 x 2.55 = 255 deg (+ wrist compensation <= 45 deg).")
RESULT["balanced"] = bal


hdr("4b. FDP idler at the MCP (PIP strand helps the MCP): gain vs radius, SCS0009 r 5, A+")
for rho in (0, 2, 3, 4):
    F, Fm, Fp = tip_force(45.2, APE["mcp"], APE["pip"], rho)
    Fs, Fsm, Fsp = tip_force(45.2, APE["mcp"], APE["pip"], rho, arms=A_straight)
    p(f"rho {rho:.0f} mm: pinch {F:5.2f} N (MCP lim {Fm:5.2f} / PIP lim {Fp:5.2f}); straight {Fs:5.2f} N "
      f"(MCP lim {Fsm:5.2f} / PIP lim {Fsp:5.2f}); coupling {rho / R_DRUM:.2f}")
p("In a pinch with the DIP coupled, the PIP runs out first, so the FDP idler only helps straight / power poses.")

hdr("4c. Which joint limits? Pose sweep, plain drums, equal strand force (N tip per 100 N strand x eta 1)")
for qm, qp in ((0, 0), (15, 20), (30, 45), (45, 60), (60, 80)):
    a = moment_arms(qm, qp)
    p(f"MCP {qm:3d} PIP {qp:3d}: arms MCP {a['mcp']:5.1f}  PIP-drive {a['pip_drive']:5.1f} mm -> "
      f"{'MCP' if a['mcp'] > a['pip_drive'] else 'PIP'} limits")


# ------------------------------------------------------------------------------------------------
# 5. Stiffness / sag
# ------------------------------------------------------------------------------------------------
hdr("5. Loop stiffness and fingertip sag (MCP flex strand, servo holds position, extensor slack under load)")
L_STRAND_V2 = idx_mcp["sheath_len"] + 30   # sheath run + spool/adjuster run
L_STRAND_A = 230.0                          # spool (forearm z -140) -> MCP drum (z 92), straight-ish
p(f"strand length V2 ~{L_STRAND_V2:.0f} mm (sheath {idx_mcp['sheath_len']:.0f}); A ~{L_STRAND_A:.0f} mm")
cases = [
    ("V2: 50 lb braid + PTFE sheath", "50 lb braid", L_STRAND_V2, E_PTFE * A_SHEATH / idx_mcp["sheath_len"]),
    ("V2: 50 lb braid + PEEK sheath", "50 lb braid", L_STRAND_V2, E_PEEK * A_SHEATH / idx_mcp["sheath_len"]),
    ("A: 80 lb braid, bare", "80 lb braid", L_STRAND_A, None),
    ("A: 150 lb braid, bare", "150 lb braid", L_STRAND_A, None),
    ("A: 200 lb braid, bare", "200 lb braid", L_STRAND_A, None),
    ("motor in palm: 80 lb, 40 mm", "80 lb braid", 40.0, None),
    ("A + 2:1 reeving: 80 lb, bare", "80 lb braid", L_STRAND_A, "reeve"),
]
p(f"{'case':34s} {'k N/mm':>7s} {'K_mcp N*m/rad':>13s} {'sag deg @5N tip':>15s} {'@10N':>6s} {'@25N':>6s}")
stiff = []
for lab, line, L, k_sh in cases:
    d, brk = LINES[line]
    k = EA_PER_BREAK * brk / L
    r_eff = R_DRUM
    if k_sh == "reeve":
        r_eff = R_DRUM * (1 + ETA_TURN)
    elif k_sh:
        k = 1 / (1 / k + 1 / k_sh)
    K = (r_eff**2) * k  # N*mm/rad, one strand (reeving: both segments wind, moment arm ~ 2 r)
    sag = [math.degrees(F * A["mcp"] / K) for F in (5, 10, 25)]
    stiff.append(dict(case=lab, k=k, K=K / 1e3, sag=sag))
    p(f"{lab:34s} {k:7.1f} {K / 1e3:13.2f} {sag[0]:15.1f} {sag[1]:6.1f} {sag[2]:6.1f}")
p("Sag = MCP torque / joint stiffness; only the MCP compliance is counted (PIP adds ~40 % more).")
p("A joint-angle sensor (magnet + Hall) at MCP/PIP lets the ESP32 servo the JOINT, and the sag becomes a force sensor:")
k80 = EA_PER_BREAK * LINES["80 lb braid"][1] / L_STRAND_A
p(f"  80 lb bare, 230 mm: {k80:.0f} N/mm -> 1 deg of MCP = {math.radians(1) * R_DRUM * k80:.1f} N strand"
  f" = {math.radians(1) * R_DRUM * k80 * R_DRUM / A['mcp']:.2f} N at the tip. A 12-bit Hall (0.1 deg) resolves ~0.05 N.")
RESULT["stiffness"] = stiff

# ------------------------------------------------------------------------------------------------
# 6. Strength
# ------------------------------------------------------------------------------------------------
hdr("6. Strength checks per load class (strand tension at stall)")
p(f"{'class':34s} {'T N':>5s} {'line':>12s} {'SF knot':>7s} {'SF clamp':>8s} {'groove MPa':>10s} {'MCP brg N':>9s} {'spool shaft N':>13s}")
classes = [("SCS0009 r 5 (V2)", 45.2, "50 lb braid"), ("SCS0009 r 5.5 reeved", 41.1, "50 lb braid"),
           ("HLS3606M r 5 (reeved or not)", 118, "80 lb braid"), ("HLS3606M r 3.7", 159, "150 lb braid"),
           ("STS3215 capped 250 N", 250, "200 lb braid")]
strength = []
for lab, T, line in classes:
    d, brk = LINES[line]
    sf_k = brk * KNOT_EFF / T
    sf_c = brk * CAPSTAN_ANCHOR_EFF / T
    groove = T / (R_DRUM * d * 1.2)          # line flattens to ~1.2 d on the drum
    brg = T + T0 + 0.25 * T                 # drum reaction + pretension + tip-force share (rough)
    shaft = T + T0                          # both strands leave the spool on the same side
    strength.append(dict(cls=lab, T=T, line=line, sf_knot=sf_k, sf_clamp=sf_c, groove=groove, brg=brg))
    p(f"{lab:34s} {T:5.0f} {line:>12s} {sf_k:7.1f} {sf_c:8.1f} {groove:10.0f} {brg:9.0f} {shaft:13.0f}")
p("PETG: ~45 MPa yield, creeps above ~15 MPa sustained -> groove pressure > 15 MPa needs a metal drum surface")
p("(brass tube / steel washer stack) or PA. MR84ZZ static rating C0 ~ 0.2 kN per bearing (verify; two per joint).")
p("MR63ZZ (spool outboard support in V2) C0 ~ 0.1 kN (verify): too small above ~100 N strand -> 684ZZ / 694ZZ.")
RESULT["strength"] = strength

# ------------------------------------------------------------------------------------------------
# 7. Wrist eye plate (bare strands through the gimbal mid-plane)
# ------------------------------------------------------------------------------------------------
hdr("7. Wrist eye plate: strands pass eyes in the gimbal mid-plane (z = 0); forearm guide at z = -H, palm guide z = +H")
H = 30.0       # guide distance from the wrist centre on each side (forearm comb / palm entry)
PITCH = 1.6    # eye pitch (18G tube OD 1.27 + web)


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def strand_len(eye, flex, dev):
    """Forearm guide fixed at (ex, ey, -H). The eye plate sits on the middle ring, which turns by dev about Y
    (forearm side). The palm turns by flex about X relative to the ring; palm guide = (ex, ey, +H) in palm frame."""
    e = np.array([eye[0], eye[1], 0.0])
    fg = np.array([eye[0], eye[1], -H])
    Rr = Ry(dev)
    eye_w = Rr @ e
    pg = Rr @ (Rx(flex) @ np.array([eye[0], eye[1], H]))
    return np.linalg.norm(eye_w - fg) + np.linalg.norm(pg - eye_w)


grid = [(PITCH * (i - 2.5), PITCH * (j - 2.5)) for i in range(6) for j in range(6)]
grid = [g for g in grid if not (abs(g[0]) > 3.5 and abs(g[1]) > 3.5)][:32]
L0 = {g: strand_len(g, 0, 0) for g in grid}
worst_diff, worst_cm = 0.0, 0.0
for fl in np.radians([-50, -25, 25, 50]):
    for dv in np.radians([-30, -15, 0, 20]):
        for g in grid:
            g2 = (-g[0], -g[1])
            d1 = strand_len(g, fl, dv) - L0[g]
            d2 = strand_len(g2, fl, dv) - strand_len(g2, 0, 0)
            worst_diff = max(worst_diff, abs(d1 - d2) / 2)
            worst_cm = max(worst_cm, abs(d1 + d2) / 2)
on_c = strand_len((0, 0), math.radians(50), 0) - strand_len((0, 0), 0, 0)
p(f"{len(grid)} eyes on a {PITCH} mm grid (max offset {max(math.hypot(*g) for g in grid):.1f} mm)")
p(f"worst differential length change (= joint motion the firmware must cancel): {worst_diff:.2f} mm"
  f" = {math.degrees(worst_diff / R_DRUM):.1f} deg of joint, {math.degrees(worst_diff / 5.0):.0f} deg of servo at r 5")
p(f"worst common-mode change for a point-symmetric flex/ext pair (= pretension change): {worst_cm:.3f} mm"
  f" -> {worst_cm * k80:.1f} N with 80 lb bare braid")
p(f"strand through the exact centre, wrist 50 deg: {on_c:.3f} mm")
p("Compare V2 sheaths: -6.7...+6.1 mm per tube taken up by a forearm fan that does not hold it (review #4).")
RESULT["wrist_eye_plate"] = dict(worst_diff_mm=worst_diff, worst_common_mm=worst_cm)

# ------------------------------------------------------------------------------------------------
# 8. Cost / mass of actuator sets
# ------------------------------------------------------------------------------------------------
hdr("8. Actuator sets (16 servo joints): cost, mass, index tip pinch (A+ routing, 2:1 reeving at the PIP)")
STRONG = 11   # 4 mcp_flex + 4 pip + thumb cmc_flex, mcp_flex, ip; weak = 4 abd + cmc_rot
sets = [
    ("16 x SCS0009 (now), r 5.5", [("SCS0009", 16)], "SCS0009", 5.5),
    ("11 x HLS3606M r 3.7 + 5 x SCS0009", [("HLS3606M", 11), ("SCS0009", 5)], "HLS3606M", 3.7),
    ("16 x HLS3606M r 3.7", [("HLS3606M", 16)], "HLS3606M", 3.7),
    ("11 x STS3215 r 7 + 5 x SCS0009", [("STS3215-12V", 11), ("SCS0009", 5)], "STS3215-12V", 7.0),
]
for lab, comp, sv, rs in sets:
    eur = sum(SERVOS[s]["eur"] * n for s, n in comp)
    g = sum(SERVOS[s]["mass"] * n for s, n in comp)
    T = min(SERVOS[sv]["stall"] * 1e3 / rs, T_CAP)
    F, _, _ = tip_force(T, APE["mcp"], APE["pip"], 0.0, reeve=(1, 2))
    Fc, _, _ = tip_force(min(SERVOS[sv]["cont"] * 1e3 / rs, T_CAP), APE["mcp"], APE["pip"], 0.0, reeve=(1, 2))
    p(f"{lab:40s} EUR {eur:4.0f}  servo mass {g:4.0f} g  strand {T:4.0f} N  pinch {F:5.1f} N (cont {Fc:4.1f})")
p("Prices: single-unit retail, rough (SCS0009 ~9, HLS3606M ~28, STS3215 ~16 EUR). Strand capped at 250 N.")

(OUTDIR / "01_force_budget.txt").write_text(BUF.getvalue())
(OUTDIR / "01_force_budget.json").write_text(json.dumps(RESULT, indent=1, default=float))
print(f"\nwrote {OUTDIR / '01_force_budget.txt'}")
