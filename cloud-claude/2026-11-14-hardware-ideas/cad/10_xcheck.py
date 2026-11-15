"""10 · Adversarial cross-check of reports 07 (bench finger) and 09 (wrist/palm routing). Study 2026-11-14.

Run: python3 cad/10_xcheck.py   (numpy only; reads out/07/*.json and out/09/eyeplate_kin.json, writes nothing)

Each section re-derives one claim with the agents' own inputs, then shows what changes when the two reports
are combined. Units mm, N, N*mm, deg.
"""
import itertools
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "out"
B7 = json.loads((OUT / "07" / "budget.json").read_text())
C7 = json.loads((OUT / "07" / "checks.json").read_text())
K9 = json.loads((OUT / "09" / "eyeplate_kin.json").read_text())
D2R = math.pi / 180


def hdr(t):
    print("\n" + "=" * 96 + "\n" + t + "\n" + "=" * 96)


R_D, R_I = B7["R_DRUM"], B7["R_IDLER"]  # 7.235, 4.235
MU = 0.12

# ------------------------------------------------------------------------------------------------ 1
hdr("1. Sign of the pip-strand crossing at the MCP (07 checks.json)")
s = C7["strands"]["pip"]["flex_mm_per_deg"]
print(f"pip FLEX path length slope: d/d q_mcp = {s['mcp']:+.4f} mm/deg -> moment arm {s['mcp'] / D2R:+.3f} mm")
print("positive = the flex path gets LONGER when the MCP flexes = the pip flex strand acts as an MCP EXTENSOR.")
print("01's force model (tip_force, rho) assumed the opposite (palm-side FDP idler adding rho*T2 to the MCP);")
print("07_budget.py section 3 uses rho = 0 (no term at all).")
rho = -s["mcp"] / D2R  # signed: + = helps MCP flexion

# ------------------------------------------------------------------------------------------------ 2
hdr("2. Index tip force with the pip strand's MCP moment included (same levers, eta, spools as 07)")
LEV = {"pinch": (43.1, 46.2), "straight": (81.0, 50.2)}
ETA = B7["eta"]
SV = {"SCS0009": 226.0, "HLS3606M": 590.0}
SP = B7["SPOOL"]


def tip(sv, pose, where="bench", rho_=rho, eta_override=None):
    Lm, Lp = LEV[pose]
    em = ETA[f"mcp_flex {where}"] if eta_override is None else eta_override
    ep = ETA[f"pip_flex {where}"] if eta_override is None else eta_override
    Tm, Tp = SV[sv] / SP[sv]["mcp_flex"], SV[sv] / SP[sv]["pip"]
    Fp = Tp * ep * R_D / Lp
    # PIP loop differential tension at the drum needed for force F: F*Lp/R_D; at the idler ~ /0.995
    k = rho_ * (Lp / R_D) / 0.995  # MCP torque from the pip loop per N of tip force (negative = steals)
    Fm = Tm * em * R_D / (Lm - k)
    return min(Fm, Fp), Fm, Fp


for sv in SV:
    for pose in LEV:
        F0 = tip(sv, pose, rho_=0.0)
        F1 = tip(sv, pose)
        F2 = tip(sv, pose, rho_=+R_I)
        print(f"{sv:9s} {pose:8s}: 07 (rho 0) {F0[0]:5.1f} N | as built (rho {rho:+.2f}) {F1[0]:5.1f} N "
              f"[MCP {F1[1]:.1f} / PIP {F1[2]:.1f}] | palm-side crossing (rho +{R_I:.2f}) {F2[0]:5.1f} N")
# palm-side pip flex needs a redirect like 07's ext; as a Ø3 dowel 90 deg it costs eta on the high-load strand
e_dowel = math.exp(-MU * (90 + 20) * D2R) * 0.995
e_roll = math.exp(-MU * 20 * D2R) * 0.995 ** 2
for nm, e in (("palm-side + Ø3 redirect dowel 90°", e_dowel), ("palm-side + MR63 redirect roller", e_roll)):
    Lm, Lp = LEV["pinch"]
    Tp = 45.2
    Fp = Tp * e * R_D / Lp
    Fm = 56.5 * ETA["mcp_flex bench"] * R_D / (Lm - R_I * (Lp / R_D))
    print(f"SCS pinch, {nm}: eta_pip {e:.3f} -> PIP {Fp:.1f} N, MCP {Fm:.1f} N -> {min(Fp, Fm):.1f} N")

# ------------------------------------------------------------------------------------------------ 3
hdr("3. Servo travel: 07's own-motion model + 09's wrist compensation (the two reports combined)")
RNG = {"mcp_abd": 30.0, "mcp_flex": 100.0, "pip": 100.0}
s_abd_mcp = 2 * 6.4 * math.sin(15 * D2R)
s_abd_pip = 2 * 1.0 * math.sin(15 * D2R)
own_mm = {"pip": R_D * 100 * D2R + R_I * 100 * D2R + s_abd_pip, "mcp_flex": R_D * 100 * D2R + s_abd_mcp,
          "mcp_abd": R_D * 30 * D2R}
print(f"07 own strand travel (no wrist): pip {own_mm['pip']:.2f} mm, mcp_flex {own_mm['mcp_flex']:.2f}, "
      f"abd {own_mm['mcp_abd']:.2f} mm. 07 adds a flat 3.6 mm for the wrist (01's estimate).")
print("09 assumed own travel on r 5: pip 200 deg (0.60*q_mcp), mcp_flex 140, abd 42; 07's model gives "
      f"pip {own_mm['pip'] / 5 / D2R:.0f}, mcp_flex {own_mm['mcp_flex'] / 5 / D2R:.0f}, "
      f"abd {own_mm['mcp_abd'] / 5 / D2R:.0f} deg on r 5.")
rows = []
for j in K9["joints"]:
    name = j["joint"]
    kind = next((k for k in ("mcp_abd", "mcp_flex", "pip") if name.endswith(k) and not name.startswith("thumb")), None)
    wrist_mm = j["travel_deg"] * D2R * 5.0  # 09 travel is on r 5
    if kind is None:  # thumb: keep 09's numbers
        rs = 5.0
        own = j["need_deg"]
    else:
        rs = SP["SCS0009"][kind]
        own = own_mm[kind] / rs / D2R
    tot = own + wrist_mm / rs / D2R
    rows.append((j["sid"], name, rs, own, wrist_mm, wrist_mm / rs / D2R, tot, j["total_deg"]))
print(f"{'ID':>2s} {'joint':16s} {'r_s':>4s} {'own':>6s} {'wrist mm':>8s} {'wrist deg':>9s} {'TOTAL':>6s}  "
      f"{'09 said':>7s}  vs 285 (07 margin) / 290 (09 margin)")
for r in rows:
    flag = "OVER" if r[6] > 290 else ("over 285" if r[6] > 285 else "ok")
    print(f"{r[0]:2d} {r[1]:16s} {r[2]:4.1f} {r[3]:6.0f} {r[4]:8.2f} {r[5]:9.0f} {r[6]:6.0f}  {r[7]:7.0f}  {flag}")
n_over = sum(r[6] > 285 for r in rows)
print(f"-> {n_over}/16 servos exceed 285 deg; worst {max(r[6] for r in rows):.0f} deg. 07's own index rows with "
      f"09's index wrist: pip {rows[0][6]:.0f}, mcp_flex {rows[1][6]:.0f}.")

# can ANY eye assignment fit? point-symmetric pairs on the 7x5 grid, travel ~ 9.6|x| + 18.2|y| (fitted to 09)
pts = [(x, y) for x in (-6.45, -4.3, -2.15, 0, 2.15, 4.3, 6.45) for y in (-4.3, -2.15, 0, 2.15, 4.3) if (x, y) != (0, 0)]
pairs = sorted({tuple(sorted([(x, y), (-x, -y)])) for x, y in pts})
cal = np.array([[abs(j["eye_flex"][0]), abs(j["eye_flex"][1])] for j in K9["joints"]])
tv = np.array([j["travel_deg"] for j in K9["joints"]])
cx, cy = np.linalg.lstsq(cal, tv, rcond=None)[0]
ptrav = sorted(cx * abs(p[0][0]) + cy * abs(p[0][1]) for p in pairs)
print(f"\nwrist travel model T = {cx:.1f}|x| + {cy:.1f}|y| servo deg on r 5 (fit to 09's table); "
      f"{len(pairs)} eye pairs, travel sorted: {[round(t) for t in ptrav]}")
need = []  # (joint, spare budget on r 5 equivalent deg), 285 limit
for r in rows:
    spare = (285 - r[3]) * r[2] / 5.0  # convert spare servo deg on r_s into r-5-equivalent wrist deg
    need.append((r[1], spare))
need.sort(key=lambda t: t[1])
ok = all(ptrav[i] <= need[i][1] for i in range(len(need)))
print("greedy (tightest joint gets the smallest pair) at 285 deg:")
for i, (nm, sp) in enumerate(need[:9]):
    print(f"   {nm:16s} spare {sp:6.1f} r5-deg  gets pair travel {ptrav[i]:5.1f}  {'ok' if ptrav[i] <= sp else 'NO'}")
print(f"-> every joint fits with the best possible assignment: {ok}")

# ------------------------------------------------------------------------------------------------ 4
hdr("4. Fuse cup: the bead must fit the cup chamber (07_finger.py: chamber Ø1.5, exit Ø0.8)")
for t, tau, nm in ((0.8, 15.0, "flat (SCS)"), (1.2, 25.0, "side (HLS)")):
    for db in (1.8, 1.5):
        print(f"lip t {t} {nm}, bead Ø{db}: trip {math.pi * db * t * tau:5.1f} N")
print("SCS stall strand: pip/abd 45.2 N (r 5), MCP 56.5 N (r 4); 80 % cap: 36 / 45 N.")
print("Trip / stall margin with a Ø1.5 bead, flat t 0.8: MCP "
      f"{math.pi * 1.5 * 0.8 * 15 / 56.5:.2f}, pip {math.pi * 1.5 * 0.8 * 15 / 45.2:.2f} -> a ±25 % print scatter "
      "trips MCP cups at servo stall.")
print("External tip push that trips the SCS pip fuse (servo holding, pinch pose): "
      f"{68 * R_D / 46.2:.1f} N (Ø1.8) / {56.5 * R_D / 46.2:.1f} N (Ø1.5).")

# ------------------------------------------------------------------------------------------------ 5
hdr("5. MCP axle: 4 x 0.5 tube with 07's radial wire slot (box x -3.6..-2.4, y -2.1..0, z -2.1..-1.0)")
n = 801
g = np.linspace(-2.0, 2.0, n)
Y, Zg = np.meshgrid(g, g)
dA = (g[1] - g[0]) ** 2
ring = (Y ** 2 + Zg ** 2 <= 4.0) & (Y ** 2 + Zg ** 2 >= 2.25)
slot = ring & ~((Y >= -2.1) & (Y <= 0) & (Zg >= -2.1) & (Zg <= -1.0))


def zmod(mask, ang):
    u = np.array([math.cos(ang), math.sin(ang)])  # bending axis direction in (y, z)
    A = mask.sum() * dA
    cy_, cz_ = (Y[mask].mean(), Zg[mask].mean())
    d = -(Y[mask] - cy_) * u[1] + (Zg[mask] - cz_) * u[0]  # distance from the neutral axis
    I = (d ** 2).sum() * dA
    return I / np.abs(d).max(), A


worst = min(zmod(slot, a)[0] for a in np.linspace(0, math.pi, 37))
full = zmod(ring, 0)[0]
P = 120 + 0.5 * (120 + 17)
M_slot = P * (5.9 - 3.0)  # moment at the slot centre (x -3), bearing centre x 5.9 (no cheek relief: upper bound)
print(f"Z full {full:.2f} mm^3, Z slotted (worst direction) {worst:.2f} mm^3 ({worst / full:.0%})")
print(f"HLS stall P {P:.0f} N, M at x 3 = {M_slot:.0f} N*mm -> nominal {M_slot / worst:.0f} MPa, x Kt ~2 = "
      f"{2 * M_slot / worst:.0f} MPa (304 hard tube ~500 yield, fatigue far lower); SCS stall: "
      f"{(45.2 + 0.5 * 52) * 2.9 / worst * 2:.0f} MPa")

# ------------------------------------------------------------------------------------------------ 6
hdr("6. Eye lip pressure and MR84 numbers")
for T, nm in ((45.2, "SCS pip r5"), (56.5, "SCS MCP r4"), (118, "HLS r5")):
    print(f"eye lip T/(r d) {nm}: {T / (0.6 * 0.47):.0f} MPa")
print(f"MR84 s0 at the lug, HLS stall: 139/{P:.0f} = {139 / P:.2f} (07: 0.74) -> confirmed; it hinges on C0 139 N")
for c0 in (139, 180, 220):
    print(f"   if C0 were {c0} N: s0 {c0 / P:.2f}")

# ------------------------------------------------------------------------------------------------ 7
hdr("7. Joint friction 07 leaves out (bush + bar pins), SCS pinch")
Fj = 45.2 + 6.8  # PIP joint load ~ strand + tip
for mu in (0.15, 0.3):
    tq = 0.5 * Fj * mu * 2.0
    print(f"PIP -x brass bush on Ø4 stainless pin, mu {mu}: {tq:.1f} N*mm of {45.2 * R_D:.0f} -> "
          f"{tq / (45.2 * R_D):.1%} loss")

# ------------------------------------------------------------------------------------------------ 8
hdr("8. Bare strands put the tendon sum through the wrist cardan (sheaths used to carry it)")
jt = K9["joints"]
for nm, T_mcp, T_oth, frac in (("SCS 80 %, all 16 flex strands", 45.2, 36.2, 1.0),
                               ("SCS power grasp, 12 strands at 50 % stall", 28.3, 22.6, 0.75),
                               ("HLS 80 %, all 16", 94.4, 94.4, 1.0)):
    tens = []
    for j in jt:
        tens.append(T_mcp if j["joint"].endswith("mcp_flex") and not j["joint"].startswith("thumb") else T_oth)
    Fax = frac * sum(tens) + 32 * 5.0
    print(f"{nm}: axial compression ~{Fax:.0f} N -> ~{Fax / 2:.0f} N per cardan bearing (2 per axis); "
          f"623 C0 ~220 N (from memory) -> s0 {220 / (Fax / 2):.2f}")
# in-plane load on the eye plate at wrist 50 deg (eye wrap up to 61 deg): 2 T sin(wrap/2) per strand
w = max(j["eye_wrap_max_deg"] for j in jt)
F_eye = 16 * 36.2 * 2 * math.sin(w / 2 * D2R) + 16 * 5 * 2 * math.sin(w / 2 * D2R)
print(f"eye plate in-plane load at wrist 50 deg, all flex strands at SCS 80 %: ~{F_eye:.0f} N on 2 x M1.6 into "
      f"the PAHT blocks (each held by 2 x M1.6 x 4 in the printed ring boss)")
