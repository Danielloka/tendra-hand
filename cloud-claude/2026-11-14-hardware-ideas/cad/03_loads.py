"""03 - joint and structure load maps for the Tendra finger (study 2026-11-14, agent 03).

Hand calculations, no FEA. Run:  uv run --no-project --with numpy python cad/03_loads.py
Prints tables; also writes out/03/03_loads.txt.  Every assumption is a named constant below.
Units: mm, N, N*mm unless stated.
"""
import io
import math
import sys

import numpy as np

OUT = io.StringIO()


def p(*a):
    print(*a)
    print(*a, file=OUT)


# ---------------- assumptions ----------------
R_DRUM = 7.0
R_SPOOL = 5.0
T_SCS = 0.226e3 / R_SPOOL          # N, SCS0009 stall 0.226 N*m on a 5 mm spool = 45.2
T_HLS = 0.59e3 / R_SPOOL           # N, HLS3606M 0.59 N*m = 118
T0 = 4.0                           # N pretension per strand
F_TIP = 10.0                       # N, design tip force (brief)
L_TIP = {"mcp": 85.0, "pip": 47.0, "dip": 18.0}   # lever from joint axis to pad centre (index: 42+24+23)
SPAN = {"mcp": 14.2, "pip": 12.1, "dip": 12.1}    # bearing centre-to-centre along the axis (V2 fits)
PETG_YIELD = 45.0                  # MPa compressive/tensile, printed in-plane (assumption; 40-50)
PETG_CREEP = 12.0                  # MPa sustained bearing stress allowed (about yield/4, assumption)
PETG_SHEAR_XY = 25.0               # MPa shear in the layer plane
PETG_SHEAR_Z = 15.0                # MPa shear across layers
C0_MR84 = 300.0                    # N static rating of MR84ZZ (estimate; datasheets say 200-400, VERIFY)
C0_MR63 = 200.0                    # N (V2 study value)

p("=" * 78)
p("A. What load must a joint carry?")
p("=" * 78)
M_pinch = F_TIP * L_TIP["mcp"] * 0.7   # brief: 10 N at ~60 mm
p(f"Brief: 10 N at 60 mm -> M_MCP = {10*60/1000:.2f} N*m. Needs tendon T = M/R = {600/R_DRUM:.0f} N (eta=1).")
p(f"SCS0009 stall tendon {T_SCS:.0f} N -> {T_SCS*R_DRUM/1000:.3f} N*m -> {T_SCS*R_DRUM/60:.1f} N tip at 60 mm (eta 1)")
p(f"HLS3606M stall tendon {T_HLS:.0f} N -> {T_HLS*R_DRUM/1000:.3f} N*m -> {T_HLS*R_DRUM/60:.1f} N tip at 60 mm (eta 1)")
p("=> the 0.6 N*m / 10 N case is only reachable with HLS-class servos. Size structure for HLS stall x1.5 =",
  f"{1.5*T_HLS:.0f} N tendon.")
p()
p("Pin reaction in a one-loop-per-joint drive = flex strand + ext strand (both run proximal):")
for name, T in (("SCS stall", T_SCS), ("HLS stall", T_HLS), ("HLS x1.5 design", 1.5 * T_HLS)):
    p(f"  {name:16s} F_pin = T + T0 = {T+T0:6.1f} N  (+ ~{F_TIP:.0f} N tip-shear component)")

p()
p("=" * 78)
p("B. Hinge types: friction torque as share of drive torque, and bearing pressure (MCP-class joint)")
p("=" * 78)
rows = [  # name, bore dia d, mu_eff, length engaged total (mm), €/joint, note
    ("printed PETG pin in PETG hole", 4.0, 0.35, 6.0, 0.0),
    ("4 mm steel dowel in reamed PETG", 4.0, 0.25, 6.0, 0.1),
    ("2 mm steel stub in PETG (V2 old)", 2.0, 0.25, 7.7, 0.05),
    ("4 mm steel in brass tube bush", 4.0, 0.15, 6.0, 0.4),
    ("4 mm steel in igus J (dry)", 4.0, 0.12, 6.0, 0.6),
    ("MR84ZZ pair (V2)", 4.0, 0.003, 6.0, 0.5),   # mu referred to bore: ~0.0015 + seals/grease drag
]
F = T_HLS + T0 + F_TIP
drive = (T_HLS - T0) * R_DRUM
p(f"Load case: HLS stall, F_pin = {F:.0f} N, drive torque (T-T0)*R = {drive:.0f} N*mm")
p(f"{'type':36s} {'M_fric N*mm':>11s} {'% of drive':>10s} {'p MPa':>7s} {'vs creep':>9s} {'EUR/jt':>7s}")
for n, d, mu, L, c in rows:
    Mf = mu * F * d / 2
    pr = F / (d * L)
    p(f"{n:36s} {Mf:11.1f} {100*Mf/drive:9.1f}% {pr:7.1f} {'OK' if pr<=PETG_CREEP or 'MR' in n or 'igus' in n or 'brass' in n else 'CREEPS':>9s} {c:7.2f}")
p("Same at idle (pretension only, F_pin = 2*T0 = 8 N): stick-slip matters more than the % above.")
for n, d, mu, L, c in rows:
    p(f"  {n:36s} breakaway torque {mu*8*d/2:6.2f} N*mm  (= {mu*8*d/2/R_DRUM:5.2f} N of tendon force)")
p("Reading: friction is 4-7 % of drive torque for a dowel. That is small next to the 25-55 % lost in the")
p("sheaths (review 05 #2). A bearing buys smoothness (no stick-slip, no wear), not strength.")

p()
p("=" * 78)
p("C. Lateral tip force: the load that really sizes a fork hinge")
p("=" * 78)
p("A sideways tip force F at lever L is a moment F*L about an axis perpendicular to the hinge axle.")
p("A fork joint resists it as a couple on the two bearings: radial load per bearing = F*L/span.")
for j in ("mcp", "pip", "dip"):
    k = L_TIP[j] / SPAN[j]
    p(f"  {j}: L {L_TIP[j]:5.1f} / span {SPAN[j]:4.1f} = {k:4.1f} N radial per bearing per N of tip force")
p()
p("Allowable lateral tip force for each weak point (static, no safety factor):")
d_stub, a_arm = 4.0, 1.5


def stub_allow(Le, p_allow=25.0, a=a_arm, d=d_stub):
    """Rigid pin embedded Le deep in plastic, load P at distance a from the face: P*(a+Le/2) = p*d*Le^2/6"""
    return p_allow * d * Le ** 2 / (6 * (a + Le / 2))


k_mcp = L_TIP["mcp"] / SPAN["mcp"]
p(f"{'weak point':56s} {'P_allow N':>9s} {'tip F N':>8s}")
for lab, P in [
    ("MR84 static rating C0 (assumed 300 N)", C0_MR84),
    ("MCP stub 4 mm, 3 mm in PETG hub (V2: stubs 4x6)", stub_allow(3.0)),
    ("MCP stub, 5 mm engagement", stub_allow(5.0)),
    ("MCP stub, 5 mm engagement + 0.3 mm brass sleeve (p 45)", stub_allow(5.0, 45.0)),
    ("MCP stub, 6 mm engagement + brass sleeve", stub_allow(6.0, 45.0)),
    ("PETG lug ring, 8 mm seat, 3 mm wide, p 20 MPa (cos peak)", 20 * 8 * 3 / 1.5),
    ("steel 4 mm axle bending, 2 supports 14 mm apart (400 MPa)", 400 * math.pi * 4 ** 3 / 32 / (3.0)),
]:
    p(f"{lab:56s} {P:9.0f} {P/k_mcp:8.1f}")
p("Goal: >= 20 N lateral at the tip (2x the 10 N pinch). The V2 stubs (3 mm engagement) fail at ~8 N.")
p("PIP/DIP use a through-pin over the whole 8.3 mm tongue: embedment 8.3 mm:")
for j in ("pip", "dip"):
    P = stub_allow(8.3, 25.0, a=1.5 + 0.0)
    p(f"  {j}: pin embedment OK to {P:.0f} N radial -> tip {P/(L_TIP[j]/SPAN[j]):.0f} N")

p()
p("Cross-drilled axle option (strands pass THROUGH the axle at the axis, so MCP can use a 14 mm through-axle):")
d, dh = 4.0, 1.2
Zfull = math.pi * d ** 3 / 32
Zhole = Zfull - dh * d ** 2 / 6
p(f"  4 mm steel axle, 1.2 mm cross hole: bending section {Zhole:.1f} mm3 vs {Zfull:.1f} (-{100*(1-Zhole/Zfull):.0f} %).")
Pax = 400 * Zhole / 3.0
p(f"  axle bending: loads 3 mm from the lug, 400 MPa -> P_allow {Pax:.0f} N radial -> tip {Pax/k_mcp:.0f} N. PETG embedment over a 14 mm axle is no issue.")

p()
p("=" * 78)
p("D. Click-in (snap-out C-seat) bearing joints: killed by the lateral moment")
p("=" * 78)
t, b, E, L = 1.6, 3.0, 1900.0, 10.0
I = b * t ** 3 / 12
k_arm = 3 * E * I / L ** 3
p(f"C-seat lip arm: t {t}, width {b}, L {L} mm, PETG E {E:.0f} MPa -> k = {k_arm:.1f} N/mm per arm")
p(f"{'lip engagement mm':>18s} {'strain %':>9s} {'pop force N':>12s} {'lateral tip F that pops it (MCP)':>34s}")
for e in (0.3, 0.5, 0.8, 1.0):
    eps = 1.5 * t * e / L ** 2 * 100
    Farm = k_arm * e
    ang = math.radians(45)
    mu = 0.3
    Fp = 2 * Farm * (math.sin(ang) + mu * math.cos(ang)) / (math.cos(ang) - mu * math.sin(ang))
    p(f"{e:18.1f} {eps:9.2f} {Fp:12.1f} {Fp/k_mcp:34.2f}")
p("A bearing pushed out of an open seat by the couple F*85/14.2 = 6 F. To hold 15 N lateral at the tip the lips")
p("must hold 90 N; then the seat needs ~90 N to insert and no longer works as a hand-pressed fuse.")
p("VERDICT: no click-in seats at the MCP/PIP. A fuse must act on a path the lateral moment does not load (tendon, below).")

p()
p("=" * 78)
p("E. Drum groove pressure (line on PETG) and the effect of a liner ring")
p("=" * 78)
for dl, nm in ((0.36, "50 lb braid 0.36"), (0.45, "80 lb braid 0.45")):
    for T, tn in ((T_SCS, "SCS 45 N"), (T_HLS, "HLS 118 N"), (1.5 * T_HLS, "HLS x1.5 177 N")):
        pr = T / (R_DRUM * dl)
        p(f"  {nm:18s} {tn:15s}: bare PETG groove p = T/(R*d) = {pr:5.1f} MPa  ({pr/PETG_CREEP:4.1f}x creep limit)")
p("With a metal ring (wall t) under the line the load spreads at ~45 deg through the ring:  p_PETG = T/(R*(d+2t))")
for dl in (0.36,):
    for t in (0.1, 0.5, 0.8):
        w = dl + 2 * t
        row = f"  ring wall {t:.1f} mm (w_eff {w:.2f}): " + ", ".join(f"{n} {T/(R_DRUM*w):4.1f} MPa" for n, T in (("SCS", T_SCS), ("HLS", T_HLS), ("HLS*1.5", 1.5 * T_HLS)))
        p(row)
p("  A 0.1 mm shim only guards against cutting; a 0.5-0.8 mm ring is what lowers the PETG pressure (3.7x / 2.6x vs bare at 0.36 line).")
p("  (estimate; the real value needs the drum compression test in the report)")
p("Capstan relief: the stress above is for the loaded strand's tangent point; it falls by e^(-mu*theta) around the drum.")
p("Bare PETG groove also wears: UHMWPE braid is hard on soft plastics; metal floor = no wear.")

p()
p("=" * 78)
p("F. Termination: ferrule bead in a keyhole with a printed fuse tab (replaces tie hole + knot)")
p("=" * 78)
p("Bead (crimped 1.5-1.8 mm sleeve, holds 90-100 % of line) pulled against a slot lip of thickness t.")
d_b = 1.8
p(f"  tab shear as a plug of bead diameter {d_b}: F = pi*d_b*t*tau")
for tau, lab in ((PETG_SHEAR_Z, "across layers (tab flat on bed, pull along z)"), (PETG_SHEAR_XY, "in the layer plane")):
    p("  " + lab)
    for t in (0.4, 0.6, 0.8, 1.0, 1.2):
        p(f"    t = {t:.1f} mm: {math.pi*d_b*t*tau:6.0f} N")
p("  Target fuse: 1.5 x actuator stall: SCS -> 68 N (t ~0.8 in-plane), HLS -> 177 N needs 80 lb braid (break ~355 N).")
p("  Printed fuses scatter +-25 %: calibrate with the bench test; the next failure must be >= 2x higher.")
p("  What it protects: the servo. A tip load pushing the finger against its tendon makes T = F*L/R (not F*L/R_spool):")
for F_ext in (2.0, 3.7, 5.0, 10.0):
    T = F_ext * L_TIP["mcp"] / R_DRUM
    p(f"    external tip force {F_ext:5.1f} N against the flexor -> tendon {T:6.0f} N -> spool torque {T*R_SPOOL/1000:.3f} N*m"
      f" ({T*R_SPOOL/226:.1f} x SCS stall)")

p()
p("=" * 78)
p("G. DIP bar pins (printed phalanx, steel pins, bar on the outside)")
p("=" * 78)
F_bar = 10.0 * L_TIP["dip"] / 6.0   # 10 N tip force, pin B about 6 mm from the DIP axis
p(f"Bar load at 10 N tip force: {F_bar:.0f} N (lever 18 mm / pin radius 6 mm)")
bar_opts = [
    ("PETG bar 3.2x1.6, hole 1.5", 3.2 * 1.6 * PETG_YIELD * 0.5),
    ("stainless strip 0.8 x 3.2 (laser cut, 1.5 holes)", (3.2 - 1.5) * 0.8 * 500.0),
    ("music wire 1.0 mm, bent, ends are the pins", math.pi * 0.25 * 1500.0 * 0.5),
]
for n, Fy in bar_opts:
    p(f"  {n:52s} net-section yield ~{Fy:6.0f} N  ({Fy/F_bar:4.1f}x)")
p("Pin cantilevered from the phalanx side wall, bar thickness s, gap 0.2: wall embedment pressure p = 6*M/(d*Le^2)")
for d_pin, Le in ((1.5, 1.5), (1.5, 3.0), (2.0, 3.0), (2.0, 4.0)):
    for s in (0.8, 1.6):
        M = F_bar * (0.2 + s / 2 + Le / 2 * 0.0)
        pr = 6 * F_bar * (0.2 + s / 2 + Le / 2) / (d_pin * Le ** 2)
        p(f"  pin {d_pin} mm, Le {Le} mm, bar {s} mm: p = {pr:5.1f} MPa  ({'OK' if pr<=PETG_CREEP*1.5 else 'too high'})")
p("Pin A web to the bearing seat is 0.28 mm in V2 (not printable: one extrusion line is 0.42-0.45 mm).")

p()
p("=" * 78)
p("H. Wall rule for a 0.4 mm nozzle (line width ~0.42-0.45)")
p("=" * 78)
for n in range(1, 6):
    p(f"  {n} perimeters = {n*0.43:4.2f} mm solid wall")
p("  < 0.86 mm (2 lines): does not print reliably; 1.29 (3 lines) minimum for a load-carrying wall; 1.72 (4 lines) around")
p("  press-fit seats. V2 flags: knot-well wall 0.6, pin A web 0.28, thumb MCP wall 0.7, spacer 0.6.")

p()
p("=" * 78)
p("I. Part count / mass / cost of the joint hardware in V2 vs options (per hand, 16 flex joints on 4 fingers + 4 thumb)")
p("=" * 78)
opts = [
    ("V2: 30 MR84 + 8 MR63 + pins", 30 * 0.7 + 8 * 0.3, 30 * 0.30 + 8 * 0.35, 38),
    ("Hybrid: MR84 at 8 finger MCP+PIP, thumb 3 joints (22 total) ; dowel+brass at DIP/abd", 22 * 0.7, 22 * 0.3 + 12 * 0.35, 22),
    ("All dowel in brass bush (38 bush)", 0.0, 38 * 0.40, 0),
]
for n, mass_b, cost, nb in opts:
    p(f"  {n:84s} bearings {nb:2d}, bearing mass {mass_b:5.1f} g, cost ~EUR {cost:4.1f}")
p("Assembly time (press 8 mm seats by hand, 20-30 s each incl. checking): 38 x 25 s = 16 min per hand;")
p("hybrid 22 x 25 s = 9 min. Total bearing cost is not the issue; the 0.05 mm seat tolerance in FDM is.")

open("out/03/03_loads.txt", "w").write(OUT.getvalue())
