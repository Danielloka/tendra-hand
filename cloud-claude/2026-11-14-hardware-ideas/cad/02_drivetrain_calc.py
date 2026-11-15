"""Agent 02 (tendon drivetrain parts): hand calculations for report 02-drivetrain.md.

    uv run --no-project --with numpy python cad/02_drivetrain_calc.py

Everything from servo spool to joint drum: line, housing, guides, anchors, spool, tensioner.
Units: mm, N, MPa (= N/mm^2), N*m, degrees in the printout. Assumptions are named next to each number;
values marked "verify" come from memory / typical catalogue ranges, not a datasheet read today.
Writes out/02/drivetrain_calc.json.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "out" / "02"
OUT.mkdir(parents=True, exist_ok=True)
V2 = HERE.parents[1] / "2026-11-09-v2-hand-cad"
RES: dict = {}

# --- load classes -------------------------------------------------------------------------------
R_SPOOL, R_DRUM = 5.0, 7.0
MM_PER_DEG = 2 * math.pi * R_DRUM / 360  # 0.1222 mm of line per joint degree
SERVOS = {"SCS0009": 0.226, "HLS3606M": 0.59}  # stall N*m (HLS: from memory, verify)
T_STALL = {k: v / (R_SPOOL / 1000) for k, v in SERVOS.items()}  # 45.2 / 118 N
T0 = 5.0  # loop pretension per strand (tendon-mechanics.md 3.2: 3-5 N)
print("strand tension at stall:", {k: round(v, 1) for k, v in T_STALL.items()})
RES["T_stall"] = T_STALL


def hdr(t):
    print("\n" + "=" * 100 + "\n" + t + "\n" + "=" * 100)


# =================================================================================================
# 1. Line: size, strength, stiffness, D/d
# =================================================================================================
hdr("1. Line choice (UHMWPE braid by Japanese PE number; Vectran; steel 7x7)")
# PE number -> nominal diameter is a JIS-style standard (PE #1 ~ 0.165 mm). Break loads are typical
# 8-carrier braid catalogue values (brand dependent, verify). eps_b = elongation at break (braid ~3.5-4.5 %).
LINES = [
    # name, d (mm), F_break (N), eps_break, note
    ("UHMWPE PE#4 (~50 lb)", 0.33, 222, 0.040, "V2 baseline class (0.36 mm 50 lb)"),
    ("UHMWPE PE#6 (~80 lb)", 0.405, 356, 0.040, ""),
    ("UHMWPE PE#8 (~100 lb)", 0.47, 445, 0.040, "recommended"),
    ("UHMWPE PE#10 (~130 lb)", 0.52, 580, 0.040, ""),
    ("Vectran braid 0.5 (~100 lb)", 0.50, 450, 0.035, "low creep, less abrasion-resistant"),
    ("Steel 7x7 0.45 nylon-coated", 0.45, 200, 0.020, "crimp only, D/d >= 25 wanted"),
]
L_LINE = 310.0  # spool -> drum line length (forearm 110 + wrist 75 + palm 80 + finger 45), V2 geometry
rows = []
for name, d, fb, eb, note in LINES:
    ea = fb / eb  # secant EA (tendon-mechanics.md 1.3); real braids are softer at low load -> range 0.6..1.0
    k = ea / L_LINE
    st = {s: (T / k, T / k * 1 / 0.6) for s, T in T_STALL.items()}  # nominal and soft-braid
    dd = {"spool D/d": 2 * R_SPOOL / d, "drum D/d": 2 * R_DRUM / d, "pin3 D/d": 3.0 / d, "eyelet r0.43 D/d": 0.86 / d}
    sf = {s: fb / T for s, T in T_STALL.items()}
    rows.append(dict(name=name, d=d, F_break=fb, EA=ea, k_N_per_mm=k, stretch_mm=st, Dd=dd, SF=sf, note=note))
    print(f"{name:32s} d {d:.3f}  Fb {fb:4.0f} N  EA {ea/1000:5.1f} kN  k {k:5.1f} N/mm  "
          f"stretch@SCS {st['SCS0009'][0]:.2f}-{st['SCS0009'][1]:.2f} mm ({st['SCS0009'][0]/MM_PER_DEG:4.1f}-{st['SCS0009'][1]/MM_PER_DEG:4.1f} deg)  "
          f"@HLS {st['HLS3606M'][0]:.2f}-{st['HLS3606M'][1]:.2f} mm  SF(HLS) {sf['HLS3606M']:.1f}  "
          f"D/d spool {dd['spool D/d']:.0f} pin3 {dd['pin3 D/d']:.1f} eyelet {dd['eyelet r0.43 D/d']:.1f}")
RES["lines"] = rows
print("Rule of thumb used: synthetic fibre over a sheave D/d >= 8 (static) and >= ~20 for long cyclic life;"
      " knot efficiency UHMWPE 50-80 %.")

# =================================================================================================
# 2. Housing compliance (the review's finding #1)
# =================================================================================================
hdr("2. Housing (Bowden outer) shortening at stall, per strand")


def tube_short(T, L, E_gpa, od, idd):
    A = math.pi / 4 * (od**2 - idd**2)
    return T * L / (E_gpa * 1000 * A), T / A


def coil_short(T, L, wire, od, settle_per_100=0.0):
    """Closed-coil spring loaded along its axis: turns touch, so the compliance is the Hertz line contact
    between neighbouring turns (cylinder on cylinder, length = pi * mean diameter per turn).
    delta per contact ~ (2 P'/(pi E*)) * (ln(4 R1 / b) - 0.5) per body; two bodies (both turns).
    settle_per_100: extra bedding-in shortening (mm per 100 mm) from imperfect closing / ovality."""
    E, nu = 193e3, 0.29  # stainless 302/304, MPa
    Es = E / (2 * (1 - nu**2))  # E* for two equal bodies
    Dm = od - wire
    Pp = T / (math.pi * Dm)  # N/mm along the contact line
    R1 = wire / 2
    Rs = R1 / 2
    b = math.sqrt(4 * Pp * Rs / (math.pi * Es))
    d1 = 2 * Pp * (1 - nu**2) / (math.pi * E) * (math.log(4 * R1 / b) - 0.5)
    per_turn = 2 * d1
    n = L / wire
    return n * per_turn + settle_per_100 * L / 100


HOUSE = []
L_V2 = 221.0  # median sheath length yoke -> adjuster in V2 (review 05)
L_NEW = 140.0  # proposed: coil only from the forearm stop block (z -123) to the palm stop plate (z +19)
for s, T in T_STALL.items():
    a, sig = tube_short(T, L_V2, 0.5, 2.0, 1.0)
    b_, sigb = tube_short(T, L_V2, 3.6, 1.5875, 1.0)  # PEEK HPLC 1/16" x 0.040"
    c = coil_short(T, L_NEW, 0.3, 2.0)
    c2 = coil_short(T, L_NEW, 0.3, 2.0, settle_per_100=0.15)
    HOUSE.append(dict(servo=s, T=T, ptfe_1x2_221=(a, sig), peek_116_221=(b_, sigb), coil_140_hertz=c, coil_140_with_settling=c2))
    print(f"{s}: T {T:5.1f} N | PTFE 1x2, 221 mm: {a:5.2f} mm ({a/MM_PER_DEG:4.0f} deg), {sig:4.1f} MPa (PTFE yield ~10-12)"
          f" | PEEK 1/16x1.0, 221 mm: {b_:4.2f} mm ({b_/MM_PER_DEG:3.0f} deg), {sigb:4.0f} MPa (yield ~100+)"
          f" | steel coil 0.3x2.0, 140 mm: Hertz {c:5.3f} mm, + settling allowance {c2:4.2f} mm ({c2/MM_PER_DEG:3.1f} deg)")
RES["housing"] = HOUSE
# PEEK bending strain: outer fibre strain = r_o / R
for R in (15, 19, 25, 40):
    print(f"  PEEK 1/16 in bend R {R:2d}: outer strain {0.79375/R*100:4.1f} %  (PEEK yield strain ~5 %, fatigue << that)"
          f" | PTFE 2.0: {1.0/R*100:4.1f} % (fine)")

# =================================================================================================
# 3. Loop stiffness and joint lag: housing + line in series
# =================================================================================================
hdr("3. Joint lag at stall (flexor carries T, extensor goes slack -> one strand path)")
lag = []
for s, T in T_STALL.items():
    for line in (rows[0], rows[2]):
        k_line = line["k_N_per_mm"]
        x_line = T / k_line
        for hname, x_h in (("PTFE 221", HOUSE[0 if s == "SCS0009" else 1]["ptfe_1x2_221"][0]),
                           ("coil 140", HOUSE[0 if s == "SCS0009" else 1]["coil_140_with_settling"])):
            x = x_line + x_h
            lag.append(dict(servo=s, line=line["name"], housing=hname, mm=x, deg=x / MM_PER_DEG))
            print(f"{s:9s} {line['name']:24s} {hname:9s}: line {x_line:4.2f} + housing {x_h:5.2f} = {x:5.2f} mm"
                  f" -> {x / MM_PER_DEG:5.1f} deg of joint lag")
RES["lag"] = lag
print("Note: with both strands taut (T < 2*T0) the stiffness doubles. Larger drums help with R^2.")

# =================================================================================================
# 4. Friction budget: V2 routes vs. the 'straight, few guides' architecture
# =================================================================================================
hdr("4. Friction budget (capstan e^-mu*theta)")
routes = json.loads((V2 / "out/assembly/routes.json").read_text())["strands"]
old_palm = np.array([r["turn_deg"] for r in routes if r["kind"] in ("free", "abd")])
new_palm = []
for r in routes:
    if r["kind"] not in ("free", "abd"):
        continue
    a, e = np.array(r["start"]), np.array(r["end"])
    v = a - e
    new_palm.append(math.degrees(math.acos(-np.dot(v, r["start_dir"]) / np.linalg.norm(v))))
new_palm = np.array(new_palm)
print(f"V2 palm turning, 24 finger strands: min/median/max {old_palm.min():.0f}/{np.median(old_palm):.0f}/{old_palm.max():.0f} deg")
print(f"Straight bare line from the palm stop plate (bore aimed) to the yoke entry (unchanged direction): "
      f"{new_palm.min():.0f}/{np.median(new_palm):.0f}/{new_palm.max():.0f} deg (0 if the yoke entry is also aimed)")
RES["palm_turn_old"] = old_palm.tolist()
RES["palm_turn_straight"] = new_palm.tolist()

MU = {"ptfe": 0.08, "coil_lined": 0.08, "steel_pin": 0.12, "pla_petg": 0.3, "eyelet_rim": 0.12, "bearing": 0.005}
# Route segments for the index pip flexor at the worst case (fist, wrist at an extreme), degrees.
OLD = [("forearm fan PTFE", 40, "ptfe"), ("wrist bundle PTFE (extreme)", 60, "ptfe"), ("palm PTFE", 107, "ptfe"),
       ("finger sheath + abd span", 30, "ptfe"), ("MCP eyelet (fist)", 90, "eyelet_rim")]
NEW = [("forearm: bare, to stop block (aimed)", 5, "steel_pin"), ("coil housing S-curves + wrist (extreme)", 80, "coil_lined"),
       ("palm: bare, aimed bores", 0, "steel_pin"), ("yoke entry / abd nip pins (abd 15)", 15, "steel_pin"),
       ("MCP nip pins Ø3 (fist)", 90, "steel_pin")]
NEW_ROLL = NEW[:-1] + [("MCP: MR63 roller (fist)", 90, "bearing")]


def eta(route):
    s = sum(MU[m] * math.radians(a) for _, a, m in route)
    return math.exp(-s), sum(a for _, a, _ in route)


for nm, rt in (("V2 (review: PTFE everywhere)", OLD), ("new: bare + coil at wrist + Ø3 pins", NEW), ("new + roller at MCP", NEW_ROLL)):
    e, tot = eta(rt)
    print(f"{nm:40s} total turning {tot:4.0f} deg  eta {e:.2f}")
# fingertip force: MCP-dominated pinch, lever 60 mm, F = T * eta * R_drum / lever (tendon-mechanics.md 4.1)
ftip = {}
for s, T in T_STALL.items():
    for nm, rt in (("V2", OLD), ("new", NEW)):
        e, _ = eta(rt)
        ftip[f"{s}_{nm}"] = T * e * R_DRUM / 60
print("fingertip force at stall, 60 mm lever (index pip-flexor route, worst case):",
      {k: round(v, 1) for k, v in ftip.items()})
RES["eta"] = {"old": eta(OLD)[0], "new": eta(NEW)[0], "new_roller": eta(NEW_ROLL)[0], "ftip": ftip}

# =================================================================================================
# 5. Guides at bends: contact pressure on the line and on the guide, friction, wear
# =================================================================================================
hdr("5. Guides at bends (line d 0.47, PE#8)")
d = 0.47
GUIDES = [
    # name, radius of curvature at the line (mm), mu, guide material allowable pressure (MPa), note
    ("18G eyelet rim (V2)", 0.43, 0.12, 400, "stainless; line frays"),
    ("Ø1.5 needle roller (fixed)", 0.75, 0.12, 1000, "hardened"),
    ("Ø2 polished steel pin", 1.0, 0.12, 1000, ""),
    ("Ø3 ground dowel ISO 8734", 1.5, 0.12, 1000, "Ra ~0.3 um, cheap"),
    ("Ø4 ground dowel", 2.0, 0.12, 1000, "D/d 8.5"),
    ("Ø3/Ø2.1 brass tube sleeve", 1.5, 0.15, 150, "soft, grooves over time"),
    ("PTFE insert, R 3 printed boss", 3.0, 0.07, 6, "PTFE creeps > ~5 MPa"),
    ("bare PETG, R 5", 5.0, 0.30, 15, "abrades, layer lines"),
    ("MR63ZZ roller (Ø6 race)", 3.0, 0.005, 1000, "rolling; bearing friction only"),
    ("681ZZ roller (Ø3 race)", 1.5, 0.008, 1000, "low load rating"),
]
grows = []
for s, T in T_STALL.items():
    print(f"-- {s}: T {T:.0f} N")
    for nm, r, mu, allow, note in GUIDES:
        p = T / (r * d)  # mean line-to-guide pressure for a line wrapping a cylinder (T/(r d))
        strain = d / (2 * r + d)  # outer-fibre bending strain proxy of the line (d/D)
        grows.append(dict(servo=s, guide=nm, r=r, p_MPa=p, Dd=2 * r / d, mu=mu, allow=allow, ok=p < allow))
        print(f"   {nm:32s} D/d {2*r/d:4.1f}  p {p:5.0f} MPa (guide limit ~{allow}) {'OK ' if p < allow else 'NO '}"
              f" loss per 90 deg {100*(1-math.exp(-mu*math.pi/2)):4.1f} %  {note}")
RES["guides"] = grows
# Does a free roller in a plain printed bore actually roll? It rolls only if the line's static friction torque
# beats the journal friction torque: mu_line * r_roll > mu_journal * r_journal.
print("\nFree roller in a plain bore rolls only if mu_line*r_roll > mu_journal*r_journal:")
for nm, r_roll, r_j, mu_j in (("Ø1.5 needle turning in its own PETG holes", 0.75, 0.75, 0.25),
                              ("Ø4 brass tube on a Ø1.5 steel axle", 2.0, 0.75, 0.2),
                              ("MR63 ball bearing (equivalent)", 3.0, 1.5, 0.003)):
    rolls = 0.15 * r_roll > mu_j * r_j
    mu_eq = mu_j * r_j / r_roll if rolls else 0.15
    print(f"   {nm:44s}: {'rolls' if rolls else 'does NOT roll (line slides)'}; effective mu {mu_eq:.3f}")

# =================================================================================================
# 6. Anchors: what the knot / crimp actually carries
# =================================================================================================
hdr("6. Anchors: share of the strand tension that reaches the termination (capstan upper bound on relief)")
ANCH = [
    ("V2 drum tie hole: 40 deg margin, PETG groove", 40, 0.20),
    ("V2 thumb rot: 19.5 deg margin", 19.5, 0.20),
    ("drum + 1 extra turn (helical groove), PETG", 40 + 360, 0.20),
    ("spool: 1 turn before anchor, PETG", 360, 0.20),
    ("spool: 1.5 turns, PETG", 540, 0.20),
    ("Ø3 steel capstan post, 3 turns", 3 * 360, 0.10),
    ("Ø3 steel capstan post, 5 turns", 5 * 360, 0.10),
]
ar = []
for nm, th, mu in ANCH:
    share = math.exp(-mu * math.radians(th))
    for s, T in T_STALL.items():
        ar.append(dict(case=nm, servo=s, share=share, F_term=T * share))
    print(f"{nm:46s} share {share*100:5.1f} %  -> SCS {T_STALL['SCS0009']*share:5.1f} N, HLS {T_STALL['HLS3606M']*share:5.1f} N")
RES["anchors"] = ar
print("Termination strength (PE#8, 445 N): knot 50-80 % -> 220-356 N; doubled-line crimp ~70-90 % (verify on braid);"
      "\n  stopper knot in a brass bead: limited by the knot (~50-60 %).")
# tie-hole liner bearing: knot against a 16G steel tube in PETG, and a crimp sleeve against a printed shoulder
for s, T in T_STALL.items():
    print(f"  {s}: crimp sleeve Ø1.3 (OD) against a shoulder Ø0.8 hole -> annulus {math.pi/4*(1.3**2-0.8**2):.2f} mm²,"
          f" {T*0.85/(math.pi/4*(1.3**2-0.8**2)):4.0f} MPa on PETG  (needs a steel washer / liner)")

# =================================================================================================
# 7. Housing end stop (ferrule) bearing pressure
# =================================================================================================
hdr("7. Housing end stops")
for s, T in T_STALL.items():
    a_coil = math.pi / 4 * (2.0**2 - 1.4**2)  # coil end face on a plate
    a_wash = math.pi / 4 * (4.0**2 - 1.7**2)  # M1.6 washer (ISO 7089: 1.7/4.0/0.3) on PETG
    print(f"{s}: coil end on printed step {T/a_coil:4.0f} MPa (PETG creep limit ~10-15) | on an aluminium stop plate OK |"
          f" via M1.6 washer onto PETG {T/a_wash:4.1f} MPa")
# shared aluminium stop plate: 32 holes Ø1.6 at 2.5 pitch, plate 2 mm, total load
Ttot = 32 * T0 + 4 * T_STALL["HLS3606M"]
print(f"shared stop plate (2 mm 6061, 32 x Ø1.6 at 2.5 pitch): total load ~{Ttot:.0f} N (pretension + 4 strands at HLS stall);"
      f" plate 12 x 22 mm on a printed ledge 2 mm wide: {Ttot/(2*(12+22)*2):.1f} MPa on PETG")

# =================================================================================================
# 8. Spool: radial load, support bearing, axle, flange/keeper, groove, tensioners
# =================================================================================================
hdr("8. Spool")
for s, T in T_STALL.items():
    Fr = T + T0  # both strands leave the spool on the same side (toward the wrist): loads add
    M_axle = Fr * 3.0  # spool cartridge 02: Ø3 axle cantilevered from the rail, load centroid ~3 mm out
    sig = 32 * M_axle / (math.pi * 3.0**3)
    print(f"{s}: spool radial load {Fr:5.0f} N | per bearing (2 in the spool) {Fr/2:4.0f} N vs C0 MR63ZZ 74 N (SF {74/(Fr/2):.1f}), MR84ZZ 139 N (NSK) "
          f"| Ø3 hardened dowel axle, 3 mm lever: {sig:4.0f} MPa (ISO 8734 hardened: yield >1000 MPa)"
          f" | moment into a servo shaft if it carried the spool alone at 8 mm: {Fr*8/1000:.2f} N*m")
# friction-clamp split spool (V2) vs servo torque
for screw, Tq, dsc in (("M2", 0.2, 2.0), ("M3", 0.6, 3.0)):
    F = Tq / (0.2 * dsc / 1000)
    hold = 0.3 * F * 0.0045
    print(f"split-spool friction clamp {screw} @ {Tq} N*m: clamp {F:4.0f} N, holds ~{hold:.2f} N*m "
          f"(SF {hold/0.226:.1f} vs SCS, {hold/0.59:.1f} vs HLS stall)")
# serration resolution vs vernier vs screw
k_loop = rows[2]["k_N_per_mm"] / 2  # loop shortening stretches both strands in series
for nm, step_mm in (("serration 36 teeth", 2 * math.pi * R_SPOOL / 36), ("serration 60 teeth", 2 * math.pi * R_SPOOL / 60),
                    ("vernier 12/13 holes", 2 * math.pi * R_SPOOL / 156), ("chordal M2 screw, 1/8 turn", 0.4 / 8),
                    ("chordal M2 screw, 1 turn", 0.4)):
    print(f"tensioner {nm:28s}: step {step_mm:5.3f} mm of line -> pretension step {step_mm * k_loop:5.2f} N")
# chordal screw: load on the nut after the spool wrap
for s, T in T_STALL.items():
    Fn = T * math.exp(-0.2 * 2 * math.pi)  # after 1 turn on the spool
    print(f"{s}: chordal-screw nut load after 1 spool turn {Fn:4.1f} N; M2 thread stress area 2.07 mm² -> {Fn/2.07:4.1f} MPa;"
          f" M2 square nut on PETG slot face 4x1.6 -> {Fn/(4*1.6):4.1f} MPa")
# flange / keeper
print(f"flange: groove floor r {R_SPOOL - d/2:.2f}, line top r {R_SPOOL + d/2:.2f}; flange r 6.5 -> {6.5-(R_SPOOL+d/2):.2f} mm"
      f" = {(6.5-(R_SPOOL+d/2))/d:.1f} d above the line; keeper ring gap 0.20 < d/2 -> a slack line cannot leave")
# groove geometry: braid flattens to ~1.3-1.6 d wide on a drum; helical pitch for N turns
for turns in (1.5, 2.0, 2.5):
    print(f"helical groove for {turns} turns at pitch {1.3*d:.2f}: groove land width {turns*1.3*d + 0.3:.2f} mm")
# servo travel and line travel per joint
for j, rng in (("mcp_flex", 100), ("pip", 100), ("mcp_abd", 30), ("thumb_cmc_rot", 141)):
    sv = rng * R_DRUM / R_SPOOL
    print(f"{j:14s}: joint {rng:3d} deg -> servo {sv:5.0f} deg, line {rng*MM_PER_DEG:5.1f} mm per strand")

# creep take-up needed
for nm, rate in (("UHMWPE SK75-class braid, 20 % load, room temp (months)", 0.01), ("Vectran / DM20", 0.002)):
    print(f"creep allowance {nm}: {rate*100:.1f} % of {L_LINE:.0f} mm = {rate*L_LINE:.1f} mm")

# =================================================================================================
# 9. Two-pin nip at a joint axis: length change error (strand crossing a joint between two pins)
# =================================================================================================
hdr("9. Strand crossing a joint between two pins (gap = d), error vs passing exactly through the axis")


def taut_len(A, B, C, r, n=720):
    """Length of a taut string from A to B pushed aside by a disk (centre C, radius r): the convex hull of
    A, B and the disk, walked from A to B along the side that touches the disk."""
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    disk = C + r * np.c_[np.cos(ang), np.sin(ang)]
    seg = B - A
    # does the straight segment miss the disk? then the string is straight
    t = np.clip(np.dot(C - A, seg) / np.dot(seg, seg), 0, 1)
    if np.linalg.norm(A + t * seg - C) >= r:
        return float(np.linalg.norm(seg))
    pts = np.vstack([A, B, disk])
    P = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p_ in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p_) <= 0:
            lo.pop()
        lo.append(p_)
    for p_ in P[::-1]:
        while len(up) >= 2 and cross(up[-2], up[-1], p_) <= 0:
            up.pop()
        up.append(p_)
    hull = np.array(lo[:-1] + up[:-1])
    ia = int(np.argmin(np.linalg.norm(hull - A, axis=1)))
    ib = int(np.argmin(np.linalg.norm(hull - B, axis=1)))
    m = len(hull)
    best = []
    for step in (1, -1):
        idx = [ia]
        while idx[-1] != ib:
            idx.append((idx[-1] + step) % m)
        path = hull[idx]
        L = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
        best.append(L)
    return min(best)  # the side of the disk (the other side runs around the whole disk)


def nip_error(r_pin, d_line, thetas_deg, shift=0.0, L1=4.0, L2=4.0):
    """Strand along +y through the joint axis (origin) between two pins at x = +-c, y = shift (fixed side).
    The strand is held in channels on both bodies that end L1 / L2 from the axis (A on the fixed body,
    B on the moving body, rotated by theta). Returns path length - (L1 + L2), i.e. the change against q = 0."""
    re = r_pin + d_line / 2
    c = r_pin + d_line / 2
    out = []
    for th in thetas_deg:
        t = math.radians(th)
        A = np.array([0.0, -L1])
        B = np.array([-L2 * math.sin(t), L2 * math.cos(t)])
        L = L1 + L2
        for sx in (-1, 1):
            Lp = taut_len(A, B, np.array([sx * c, shift]), re)
            L = max(L, Lp) if Lp > np.linalg.norm(B - A) + 1e-9 else L
        # path = straight A->B unless a pin pushes it; with no pin it would be A->B direct (shorter than the corner):
        Ld = float(np.linalg.norm(B - A))
        Ls = [taut_len(A, B, np.array([sx * c, shift]), re) for sx in (-1, 1)]
        out.append(max(Ls) - (L1 + L2) if max(Ls) > Ld + 1e-9 else Ld - (L1 + L2))
    return out


ths = [-10, 0, 15, 30, 45, 60, 90]
nip = {}
for rp in (0.75, 1.0, 1.5):
    e0 = nip_error(rp, d, ths)
    # optimise the nip centre position along the strand (fixed side) to flatten the error over -10..90
    best = min(((max(nip_error(rp, d, ths, s)) - min(nip_error(rp, d, ths, s)), s) for s in np.linspace(-3, 3, 61)))
    e1 = nip_error(rp, d, ths, best[1])
    nip[f"pin_{2*rp}"] = dict(on_axis=e0, shift=best[1], shifted=e1, spread=best[0])
    print(f"pin Ø{2*rp:.1f} on the axis : " + ", ".join(f"{t:+d}:{x:+.2f}" for t, x in zip(ths, e0)) + " mm")
    print(f"pin Ø{2*rp:.1f} shifted {best[1]:+.1f}: " + ", ".join(f"{t:+d}:{x:+.2f}" for t, x in zip(ths, e1))
          + f" mm  -> spread {best[0]:.2f} mm over -10..90 deg")
e_abd = nip_error(1.5, d, [-15, 15])
print(f"abduction crossing (Ø3 pins, +-15 deg): {e_abd[0]:+.3f} / {e_abd[1]:+.3f} mm  -> negligible")
print("Both strands of a loop that cross the same nip change by the same amount (common mode -> pretension change).")
RES["nip"] = nip

# =================================================================================================
# 10. Coaxial idler at the MCP for the pip strands (rolling, known linear coupling)
# =================================================================================================
hdr("10. Coaxial idler on the MCP axis for the pip strands")
for r_i, nm in ((3.0, "MR63ZZ outer race Ø6"), (2.5, "682ZZ / MR52 Ø5"), (3.5, "MR74ZZ Ø7")):
    c = r_i / R_DRUM  # pip degrees per MCP degree if uncompensated (flex on the palmar side, ext dorsal)
    comp = r_i / R_SPOOL * 100  # extra pip-servo degrees over the MCP range of 100 deg
    print(f"{nm:22s}: r {r_i}: pip moves {c:.2f} deg per MCP deg (exactly linear, no common mode); firmware adds "
          f"{r_i/R_SPOOL:.2f} x q_mcp to the pip servo = {comp:3.0f} deg over the MCP range "
          f"(pip servo travel 140 -> {140+comp:3.0f} deg); D/d {2*r_i/d:4.1f}; friction ~0.2 % per bend")
RES["idler_mcp"] = {"r_i": 3.0, "coupling_deg_per_deg": 3.0 / R_DRUM, "extra_servo_deg": 3.0 / R_SPOOL * 100}

# =================================================================================================
# 11. Radical: bare compact bundle across the wrist, compensated in firmware
# =================================================================================================
hdr("11. Bare wrist bundle (no housing): first-order strand length change = offset x wrist angle")
pitch = 1.9
g = (np.arange(6) - 2.5) * pitch
cells = [(x, y) for x in g for y in g if not (abs(x) > 4 and abs(y) > 4)]  # 6x6 minus 4 corners = 32
fl, dv = math.radians(50), math.radians(30)
worst = max(abs(y) * fl + abs(x) * dv for x, y in cells)
print(f"32 strands on a {pitch} mm grid ({len(cells)} cells, {5*pitch:.1f} mm square): worst |dL| ~ {worst:.1f} mm "
      f"= {worst/MM_PER_DEG:.0f} joint deg = {worst/R_SPOOL*180/math.pi:.0f} servo deg of compensation; "
      "flex/ext of one loop on point-symmetric cells -> differential (compensable), common mode is 2nd order")
RES["bare_wrist_worst_mm"] = worst

(OUT / "drivetrain_calc.json").write_text(json.dumps(RES, indent=1, default=float))
print("\nwrote", OUT / "drivetrain_calc.json")
