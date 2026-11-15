"""07 · V2.1 bench index finger: settle the SYNTHESIS conflicts with numbers (study 2026-11-14, agent 07).

Run: uv run --no-project --with numpy python cad/07_budget.py   -> out/07/budget.txt, out/07/budget.json
The CAD script (cad/07_finger.py) reads budget.json for the chosen design (spools, DIP linkage pins).

Units mm, N, N*mm, deg. Assumptions are named constants; 'verify' = from memory or a seller listing.
"""
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "out" / "07"
OUT.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(HERE.parents[2] / "hardware" / "cad"))
import dip_linkage as DL  # noqa: E402  (V1 maths, read-only)

LINES = []


def p(*a):
    s = " ".join(str(x) for x in a)
    LINES.append(s)
    print(s)


def hdr(t):
    p("")
    p("=" * 100)
    p(t)
    p("=" * 100)


D2R = math.pi / 180
# ---------------------------------------------------------------- inputs
SERVO = {  # stall / continuous torque N*mm, usable travel deg
    "SCS0009": dict(stall=226.0, cont=74.0, travel=300.0, note="0-1023 over 300 deg (verify on yours)"),
    "HLS3606M": dict(stall=590.0, cont=150.0, travel=360.0 * 7, note="12-bit, multi-turn mode (verify)"),
}
TRAVEL_MARGIN = 15.0  # deg kept free: calibration sweep to the hard stops, 7.5 deg each end
LINE = dict(name="PE#8 UHMWPE braid", d=0.47, break_N=445.0)  # 02, verify
R_DRUM = 7.0 + 0.47 / 2  # 7.235: line centre on a 14 x 0.5 brass tube ring (OD 14, the strand sits on it)
R_IDLER = 4.0 + 0.47 / 2  # 4.235: line centre on an MR84ZZ outer race (OD 8), coaxial on the MCP axle
R_IDLER_MR63 = 3.0 + 0.47 / 2  # if a stepped 4/3 axle carried MR63 (needs lathe work)
X_MCP_LANE = 6.4  # MCP strands on the proximal lugs, x = -6.4 (flex) / +6.4 (ext)
X_PIP_LANE = 1.0  # pip strands on the idler and PIP ring, x = -1.0 (flex) / +1.0 (ext)
RANGES = {"mcp_abd": (-15.0, 15.0), "mcp_flex": (-10.0, 90.0), "pip": (-5.0, 95.0)}
WRIST_SPAN = 3.6  # mm of strand length change across the wrist workspace (eye plate, inner ring, 01 §3.1: |dL|<=1.8)
MU_STEEL = 0.12  # braid on polished hardened steel (02)
ETA_ROLL = 0.995  # per bearing roller
# index pinch statics (01 §3, same 42/24/23 mm finger): torque per N of tip force
LEVER = {"pinch": dict(mcp=43.1, pip_drive=46.2), "straight": dict(mcp=81.0, pip_drive=50.2)}


def eta_slide(deg):
    return math.exp(-MU_STEEL * deg * D2R)


# ---------------------------------------------------------------- 1. strand travel per servo joint
hdr("1. Servo travel budget (strand mm the spool must move over the full joint range, worst case)")
rng = {k: (v[1] - v[0]) * D2R for k, v in RANGES.items()}
abd_half = RANGES["mcp_abd"][1] * D2R
s_abd_mcp = 2 * X_MCP_LANE * math.sin(abd_half)  # differential coupling span from the abd crossing
s_abd_pip = 2 * X_PIP_LANE * math.sin(abd_half)


def travel(joint, reeve=False, r_idler=R_IDLER, wrist=WRIST_SPAN):
    if joint == "mcp_abd":
        return R_DRUM * rng["mcp_abd"] + wrist
    if joint == "mcp_flex":
        return R_DRUM * rng["mcp_flex"] + s_abd_mcp + wrist
    own = (2 if reeve else 1) * R_DRUM * rng["pip"]
    return own + r_idler * rng["mcp_flex"] + s_abd_pip + wrist


p(f"ranges deg {RANGES}; drum line radius {R_DRUM:.3f}; MCP idler line radius {R_IDLER:.3f} (MR84ZZ on the 4 mm axle)")
p(f"abd crossing (strands at x = +-{X_MCP_LANE} / +-{X_PIP_LANE} mm, guide on the abd plane): differential span "
  f"MCP {s_abd_mcp:.2f} mm, PIP {s_abd_pip:.2f} mm; wrist span {WRIST_SPAN} mm (hand only, 0 on the bench)")
p(f"pip coupling through the idler: {R_IDLER / R_DRUM:.3f} PIP deg per MCP deg "
  f"(MR63 on a stepped axle: {R_IDLER_MR63 / R_DRUM:.3f})")
avail = SERVO["SCS0009"]["travel"] - TRAVEL_MARGIN
p(f"SCS0009 usable: {SERVO['SCS0009']['travel']:.0f} - {TRAVEL_MARGIN:.0f} margin = {avail:.0f} deg = {avail * D2R:.3f} rad")
p("")
p(f"{'option':58s} {'joint':9s} {'strand mm':>9s} {'r_s min':>7s} {'r_s':>5s} {'servo deg':>9s} {'G torque':>8s}")
OPTIONS = {}
for name, reeve, ri, wr in (("A  plain PIP, MR84 idler (CHOSEN)", False, R_IDLER, WRIST_SPAN),
                            ("A0 same, bench only (no wrist)", False, R_IDLER, 0.0),
                            ("B  2:1 reeved PIP, MR84 idler", True, R_IDLER, WRIST_SPAN),
                            ("C  plain PIP, MR63 idler on stepped axle", False, R_IDLER_MR63, WRIST_SPAN)):
    rows = {}
    for j in ("mcp_abd", "mcp_flex", "pip"):
        s = travel(j, reeve and j == "pip", ri, wr)
        rmin = s / (avail * D2R)
        rows[j] = dict(strand_mm=s, r_s_min=rmin)
    OPTIONS[name] = rows
    for j, r in rows.items():
        eta_t = math.exp(-MU_STEEL * math.pi)  # 180 deg on a Ø3 turnaround dowel (reeving)
        arm = R_DRUM * (1 + eta_t) if (reeve and j == "pip") else R_DRUM
        p(f"{name:58s} {j:9s} {r['strand_mm']:9.2f} {r['r_s_min']:7.2f} {'':>5s} {'':>9s} {arm / r['r_s_min']:8.2f}")
p("G torque = joint torque per servo torque when the spool is the smallest that fits the travel (eta not included).")
p("Reeving doubles the PIP strand travel but the torque only by 1 + eta_turn = 1.69: per servo degree it is")
p("16 % WORSE than a plain drum on a smaller spool. Its only gain is a lower strand load at the same joint torque.")

# chosen spools
SPOOL = {"SCS0009": {"mcp_abd": 5.0, "mcp_flex": 4.0, "pip": 5.0},
         "HLS3606M": {"mcp_abd": 5.0, "mcp_flex": 5.0, "pip": 5.0}}
p("")
p("Chosen spools (line-centre radius): SCS0009 MCP 4.0 (metal hub / 683ZZ cartridge variant), abd + PIP 5.0 (02 cartridge);")
p("HLS3606M: 5.0 on every joint (multi-turn, so travel is free; 5.0 keeps the strand at the 120 N design class).")
p(f"{'servo':9s} {'joint':9s} {'r_s':>5s} {'servo deg used':>14s} {'of':>5s} {'G':>5s} {'T stall N':>9s} {'T cont N':>8s}")
SERVO_DEG = {}
for sv, sp in SPOOL.items():
    for j, r in sp.items():
        deg = travel(j) / r / D2R
        SERVO_DEG[(sv, j)] = deg
        T = SERVO[sv]["stall"] / r
        Tc = SERVO[sv]["cont"] / r
        lim = avail if sv == "SCS0009" else SERVO[sv]["travel"]
        flag = "OK" if deg <= lim else "OVER"
        p(f"{sv:9s} {j:9s} {r:5.1f} {deg:14.0f} {lim:5.0f} {R_DRUM / r:5.2f} {T:9.1f} {Tc:8.1f}  {flag}")
p(f"SCS PIP check: {R_DRUM:.3f} x 100 deg + idler {R_IDLER:.3f} x 100 deg + abd {s_abd_pip:.2f} mm + wrist {WRIST_SPAN} mm "
  f"= {travel('pip'):.1f} mm on r 5 = {SERVO_DEG[('SCS0009', 'pip')]:.0f} deg <= {avail:.0f}")

# ---------------------------------------------------------------- 2. friction per route
hdr("2. Friction: sliding turn per flex strand (drum and spool: strand anchored, no sliding)")
ROUTES = {  # (sliding deg on steel, rollers)
    "mcp_flex bench": (10 + 10, 0),  # yoke abd-plane dowel 10 + base dowel 10
    "pip_flex bench": (10 + 10, 1),  # same + the MCP idler (dorsal side, no redirect)
    "pip_ext bench": (90 + 10 + 10, 1),  # + the 90 deg redirect dowel (low-load strand)
    "mcp_flex hand": (20 + 15 + 10, 0),  # + forearm comb 15 + palm guide 10, wrist straight
    "pip_flex hand": (20 + 15 + 10, 1),
    "pip_flex hand, wrist 50": (20 + 15 + 10 + 50, 1),
}
ETA = {}
for k, (deg, nr) in ROUTES.items():
    ETA[k] = eta_slide(deg) * ETA_ROLL ** nr
    p(f"{k:28s} sliding {deg:4d} deg, rollers {nr}: eta = {ETA[k]:.3f}")
p("V2 for comparison (01): MCP flex 223 deg in PTFE -> 0.68, pip flex 246 deg -> 0.65.")

# ---------------------------------------------------------------- 3. tip force
hdr("3. Index tip force (pinch MCP 30 / PIP 45; straight push). Limit = the weaker joint")
FORCE = {}
for where in ("bench", "hand"):
    for sv in SERVO:
        for mode in ("stall", "s80", "cont"):
            tau = SERVO[sv]["stall"] * (0.8 if mode == "s80" else 1) if mode != "cont" else SERVO[sv]["cont"]
            Tm = tau / SPOOL[sv]["mcp_flex"]
            Tp = tau / SPOOL[sv]["pip"]
            em, ep = ETA[f"mcp_flex {where}"], ETA[f"pip_flex {where}"]
            res = {}
            for pose, L in LEVER.items():
                fm = Tm * em * R_DRUM / L["mcp"]
                fp = Tp * ep * R_DRUM / L["pip_drive"]
                res[pose] = (min(fm, fp), "MCP" if fm < fp else "PIP", fm, fp)
            FORCE[(where, sv, mode)] = res
            if mode == "cont" or where == "bench" or True:
                p(f"{where:5s} {sv:9s} {mode:5s} strand MCP {Tm:6.1f} N PIP {Tp:6.1f} N | pinch {res['pinch'][0]:5.1f} N "
                  f"({res['pinch'][1]} limits; MCP {res['pinch'][2]:.1f} / PIP {res['pinch'][3]:.1f}) | straight "
                  f"{res['straight'][0]:5.1f} N ({res['straight'][1]})")
p("s80 = SCS0009 80 % of stall: above that it backs off after 2 s, so s80 is the usable peak. cont = rated torque.")
p("Reeved PIP with HLS r 5 (option B): PIP limit rises to "
  f"{118 * 1.686 * R_DRUM * ETA['pip_flex bench'] / LEVER['pinch']['pip_drive']:.1f} N but the MCP stays at "
  f"{118 * R_DRUM * ETA['mcp_flex bench'] / LEVER['pinch']['mcp']:.1f} N: pinch gain "
  f"{(118 * R_DRUM * ETA['mcp_flex bench'] / LEVER['pinch']['mcp']) / FORCE[('bench', 'HLS3606M', 'stall')]['pinch'][0] - 1:+.0%} for a 2nd groove,"
  " a Ø3 180 deg turn (D/d 6.4) and +4 mm of width.")

# ---------------------------------------------------------------- 4. DIP linkage with pin A off the web
hdr("4. DIP bar: pin A moved off the 0.28 mm web (Ø2 pins, >= 1.6 mm wall to the joint bores)")
PIN_D = 2.0
BUSH_R = 2.5  # Ø5 brass bush at the -x PIP lug (bar side) and in both DIP cheeks
L2 = 24.0
a_min = BUSH_R + 1.6 + PIN_D / 2  # 5.1
pipg = np.radians(np.linspace(-5, 95, 41))
tgt = 0.75 * pipg
DL.HALF_HEIGHT, DL.AXIS_CLEAR = 7.0, 2.0


def score(prm):
    d = DL.dip_curve(L2, *prm, pip=pipg)
    if d is None:
        return math.inf, None, None
    c = DL.check(L2, *prm, pipg, d)
    if c is None:
        return math.inf, None, None
    return float(np.abs(d - tgt).max()), d, c


best = (math.inf, None)
for a, b in itertools.product(np.arange(a_min, 5.61, 0.25), np.arange(a_min, 6.81, 0.25)):
    for ga, gb in itertools.product(range(0, 91, 5), range(-120, -59, 5)):
        prm = (a, ga * D2R, b, gb * D2R)
        s = score(prm)[0]
        if s < best[0]:
            best = (s, prm)
prm, step = np.array(best[1]), np.array([0.05, 1 * D2R, 0.05, 1 * D2R])
s0 = best[0]
while step[1] > 0.02 * D2R:
    imp = False
    for i, sg in itertools.product(range(4), (1, -1)):
        q = prm.copy()
        q[i] += sg * step[i]
        if q[0] < a_min or q[2] < a_min or q[2] > 6.8:
            continue
        sq = score(q)[0]
        if sq < s0:
            prm, s0, imp = q, sq, True
    if not imp:
        step /= 2
err, dipc, clr = score(prm)
a, ga, b, gb = (float(v) for v in prm)
bar = math.hypot(a * math.cos(ga) - L2 - b * math.cos(gb), a * math.sin(ga) - b * math.sin(gb))
p(f"pin A r {a:.2f} @ {ga / D2R:.1f} deg (web to the Ø5 bush {a - PIN_D / 2 - BUSH_R:.2f} mm), "
  f"pin B r {b:.2f} @ {gb / D2R:.1f} deg (web {b - PIN_D / 2 - BUSH_R:.2f}), bar {bar:.2f} mm")
p(f"max |DIP - 0.75 PIP| = {err / D2R:.2f} deg; axis clearance {clr['axis_clear_mm']:.2f}; "
  f"min transmission {clr['min_transmission_deg']:.1f} deg (V2: 0.51 deg with pin A in a 0.28 mm web)")
p("Why the -x PIP side is a brass bush, not the 2nd MR84: with an Ø8 seat pin A must sit at r >= 6.6, and then no "
  "linkage inside the finger height reaches 0.75 (best found 3.5 deg error).")
# bar force at the pinch: DIP torque = F * 15 mm (01); bar arm about the DIP axis at pip 45
pip45 = 45 * D2R
d45 = DL.dip_curve(L2, a, ga, b, gb, np.array([0.0, pip45]))[1]
A = np.array([a * math.cos(ga + pip45), a * math.sin(ga + pip45)])
B = np.array([L2 + b * math.cos(gb - d45), b * math.sin(gb - d45)])
u = (B - A) / np.linalg.norm(B - A)
w = B - np.array([L2, 0.0])
arm_B = abs(u[0] * w[1] - u[1] * w[0])
F_tip_max = 30.0
bar_F = F_tip_max * 15.0 / arm_B
p(f"bar arm about the DIP at PIP 45: {arm_B:.2f} mm -> bar force {bar_F:.0f} N at a {F_tip_max:.0f} N tip force")
BAR_T, BAR_W, HOLE = 0.8, 3.2, 2.05
net = (BAR_W - HOLE) * BAR_T
p(f"stainless 0.8 x 3.2 strip, Ø2.05 holes: net section {net:.2f} mm^2 -> {bar_F / net:.0f} MPa (304 yield ~215, "
  f"SF {215 / (bar_F / net):.1f}); bearing on the pin {bar_F / (PIN_D * BAR_T):.0f} MPa (steel on steel)")
EMB = 4.0
# pin in PETG, load at 0.6 mm out from the face (bar centre): Meyer-type edge pressure 6 M / (d L^2) + F/(dL)
M = bar_F * (0.2 + BAR_T / 2)
pmax = bar_F / (PIN_D * EMB) + 6 * M / (PIN_D * EMB ** 2)
p(f"Ø2 pin set {EMB:.0f} mm deep in PETG: peak edge pressure {pmax:.1f} MPa (PETG creep limit ~12, yield ~45) at 30 N tip;"
  f" {pmax * 10 / 30:.1f} MPa at 10 N")
LINK = dict(pin_a=dict(r=a, deg=ga / D2R), pin_b=dict(r=b, deg=gb / D2R), bar=bar, err_deg=err / D2R,
            L2=L2, clear=clr)

# ---------------------------------------------------------------- 5. stresses at the 120 N class
hdr("5. Stresses at the design class: strand T = 120 N (HLS3606M stall on r 5); SCS values in brackets")
T, Ts = 120.0, SERVO["SCS0009"]["stall"] / 4.0
ring_t = 0.5
p_line = T / (R_DRUM * LINE["d"])
p_petg = T / (R_DRUM * (LINE["d"] + 2 * ring_t))
p(f"line on the brass ring: T/(R d) = {p_line:.0f} MPa [{Ts / (R_DRUM * LINE['d']):.0f}] (brass ~150 MPa allowed; "
  f"D/d = {2 * R_DRUM / LINE['d']:.0f}); PETG under the 0.5 mm ring: {p_petg:.1f} MPa [{Ts / (R_DRUM * (LINE['d'] + 1)):.1f}] "
  f"vs ~12 creep: OK")
p(f"ring hoop stress from the wrap (thin ring on a rigid core, worst bound T/t/w): {T / (ring_t * 1.6):.0f} MPa "
  f"for the 1.6 mm MCP ring (brass yield ~300)")
# axle: Ø4 x 0.5 wall stainless tube, fixed in the tongue cheeks (x 1.7..4.2), bearings at x 5.9
Do, Di = 4.0, 3.0
I = math.pi / 64 * (Do ** 4 - Di ** 4)
Z = I / (Do / 2)
lev = 5.9 - 2.95  # bearing centre to cheek centre
P_drive = T + 0.5 * (T + 17)  # lug with the MCP flex strand + half of the pip strand / tip reaction
P_lat = 6.0 * 20.0  # 20 N sideways tip force -> 6 N per N on each MCP bearing (03)
for nm, P in (("drive, HLS stall", P_drive), ("20 N sideways tip", P_lat)):
    p(f"axle Ø4x0.5 tube, {nm}: P {P:.0f} N per bearing, M = P x {lev:.2f} = {P * lev:.0f} N*mm -> "
      f"{P * lev / Z:.0f} MPa (304 tube yield ~215 annealed / ~500 hard drawn; solid Ø4 hardened dowel {P * lev / (math.pi * 4 ** 3 / 32):.0f} MPa)")
p(f"tongue cheek bearing pressure through the Ø5 brass sleeve (2.5 mm cheek): {P_drive / (5 * 2.5):.1f} MPa drive, "
  f"{P_lat / (5 * 2.5):.1f} MPa at 20 N sideways (PETG creep ~12, yield ~45)")
C0 = {"MR84ZZ": 139.0, "MR63ZZ": 74.0}
p(f"MR84ZZ C0 = {C0['MR84ZZ']:.0f} N (NSK, via 02): MCP lug bearing s0 = {C0['MR84ZZ'] / P_drive:.2f} at HLS stall "
  f"[{C0['MR84ZZ'] / (Ts + 0.5 * (45 + 7)):.2f} SCS stall], {C0['MR84ZZ'] / (P_drive * 150 / 590):.1f} at HLS continuous")
for q in (30, 90):
    wrap = 15.9 + q
    Fi = 2 * T * math.sin(wrap / 2 * D2R)
    p(f"MCP idler (pip flex wrap {wrap:.0f} deg at MCP {q}): {Fi:.0f} N -> s0 {C0['MR84ZZ'] / Fi:.2f} "
      f"[SCS {C0['MR84ZZ'] / (Fi * 45 / 120):.2f}]")
p(f"PIP: MR84 (+x) + Ø5 brass bush (-x). Bush pressure at HLS ({T:.0f} N strand, ~{T * 1.1:.0f} N joint load, half "
  f"on the bush) {T * 1.1 / 2 / (5 * 3.0):.1f} MPa on PETG")
p(f"DIP Ø4 pin in two Ø5 x 3 brass bushes: 30 N tip -> joint load ~{bar_F + 30:.0f} N -> "
  f"{(bar_F + 30) / 2 / (5 * 3):.1f} MPa on PETG")
Fr = 2 * T * math.sin(90 / 2 * D2R)
p(f"redirect dowel Ø3 (pip EXT only, 90 deg): {Fr:.0f} N at 120 N, but the ext strand only carries extension "
  f"force + pretension (~10-20 N): {2 * 20 * math.sin(45 * D2R):.0f} N")
p(f"line pressure on Ø3 dowels: {T / (1.5 * LINE['d']):.0f} MPa at 120 N (hardened steel OK; D/d {3 / LINE['d']:.1f}); "
  f"bends there are 10 deg except the ext redirect")
for t, tau, nm in ((0.8, 15.0, "flat (SCS)"), (1.2, 25.0, "on its side (HLS)")):
    p(f"fuse cup lip t {t} {nm}: trips at pi x 1.8 x t x tau = {math.pi * 1.8 * t * tau:.0f} N "
      f"(line breaks at {LINE['break_N']:.0f}, crimp on doubled braid ~70-90 %)")
p(f"line SF at 120 N: {LINE['break_N'] / T:.1f} (bare), behind the fuse it can never see more than the trip load.")

# ---------------------------------------------------------------- 6. decision
hdr("6. Decision")
p("PIP: PLAIN drum (no reeving). With SCS0009 the travel budget, not the strand, is the limit, and reeving is 16 % worse")
p("per servo degree; with HLS the MCP (120 N strand class) limits the pinch, so reeving the PIP adds only ~9 %.")
p("MCP: reverse fork, fixed Ø4 tube axle, ONE MR84ZZ idler for both pip strands (the two strands' contact speeds")
p("are always equal, so one race is enough). pip FLEX passes on the idler's DORSAL side (wrap grows with flexion, no")
p("redirect, no sliding); pip EXT on the palm side with a Ø3 redirect dowel that pre-wraps it 106 deg (a strand on the")
p("palm side of a coaxial idler otherwise bow-strings off it after ~12 deg of MCP: SYNTHESIS item 6 as written")
p("does not work for the palm-side strand). MR63 needs a stepped axle; MR84 gives 0.585 coupling and still fits.")
p("Spools: SCS MCP r 4 / PIP r 5 / abd r 5; HLS r 5 everywhere.")
J = dict(R_DRUM=R_DRUM, R_IDLER=R_IDLER, X_MCP_LANE=X_MCP_LANE, X_PIP_LANE=X_PIP_LANE, RANGES=RANGES, SPOOL=SPOOL,
         servo_deg={f"{k[0]}:{k[1]}": v for k, v in SERVO_DEG.items()},
         eta=ETA, force={f"{k[0]}:{k[1]}:{k[2]}": v for k, v in FORCE.items()}, link=LINK,
         options={k: {j: {kk: round(vv, 3) for kk, vv in r.items()} for j, r in v.items()} for k, v in OPTIONS.items()},
         coupling=dict(pip_per_mcp=R_IDLER / R_DRUM, mcp_per_abd=X_MCP_LANE / R_DRUM, pip_per_abd=X_PIP_LANE / R_DRUM))
(OUT / "budget.json").write_text(json.dumps(J, indent=1, default=str))
(OUT / "budget.txt").write_text("\n".join(LINES) + "\n")
