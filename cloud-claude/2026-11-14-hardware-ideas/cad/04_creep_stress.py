"""04_creep_stress.py - sustained stresses in the printed parts vs. what each plastic can carry for hours.
Hand calculations, assumptions in the code. Run: python3 cad/04_creep_stress.py
Long-term stress allowance (rule of thumb, NOT a datasheet value): sigma_lt ~ 0.15-0.25 x short-term yield
for amorphous FDM (PLA/PETG/ASA/PC), ~0.25-0.3 x for CF-nylon, ALONG layers; across layers use x 0.5.
"""
import math

T0 = 5.0        # N, pretension per strand (braid, brief 2.2 x low pretension)
T_STALL = 45.0  # N, SCS0009 stall at the spool 0.226 N.m / 5 mm
YIELD = {"PLA": 50, "PETG": 45, "PETG-CF": 70, "PA12-CF": 80, "ASA": 40, "PC": 60}  # MPa, XY, typical datasheets
LT = {"PLA": 0.15, "PETG": 0.20, "PETG-CF": 0.25, "PA12-CF": 0.25, "ASA": 0.20, "PC": 0.25}
print("long-term allowable stress along layers (MPa):", {k: round(YIELD[k] * LT[k], 1) for k in YIELD})

def row(name, sigma_sust, sigma_peak):
    print(f"{name:55s} sustained {sigma_sust:6.1f} MPa   peak {sigma_peak:6.1f} MPa")

# 1. PTFE sheath stop: 2.0 OD sheath, 1.2 bore (line passes through), annulus bears on the stop
A = math.pi / 4 * (2.0**2 - 1.2**2)
row("1. sheath end on a flat stop (A=%.2f mm2)" % A, T0 / A, T_STALL / A)
# with a stainless ferrule 2.0/2.6 pressed in, stop area becomes the bore shoulder in plastic of OD 2.6 -> 2.2 hole
A2 = math.pi / 4 * (2.6**2 - 1.2**2)
row("   same, brass/steel ferrule OD 2.6 on a shoulder", T0 / A2, T_STALL / A2)
A3 = math.pi / 4 * (4.0**2 - 1.2**2)
row("   same, ferrule with a 4.0 flange (flanged, area %.1f)" % A3, T0 / A3, T_STALL / A3)

# 2. tie hole bearing: 16G liner OD 1.65, 1.2 mm long in PETG; knot pulls ~ 90 % of strand tension on it
A_tie = 1.65 * 1.2
row("2. knot load on an unlined 1.7 mm tie hole (proj. area %.2f)" % A_tie, 0.9 * T0 / A_tie, 0.9 * T_STALL / A_tie)
A_tie2 = 1.65 * 3.0
row("   liner pressed 3 mm deep (proj. %.2f)" % A_tie2, 0.9 * T0 / A_tie2, 0.9 * T_STALL / A_tie2)

# 3. M2 clamp of a split printed spool: preload F = T/(0.2 d), T = 0.2 N.m for an M2 (clamp 500 N)
F = 0.2 / (0.2 * 0.002)
A_head = math.pi / 4 * (4.0**2 - 2.2**2)   # M2 pan head 3.8 -> washer-less bearing annulus
A_w = math.pi / 4 * (5.0**2 - 2.2**2)       # M2 washer OD 5
row("3. M2 clamp screw head on PETG (preload %d N, area %.1f)" % (F, A_head), F / A_head, F / A_head)
row("   with a M2 washer OD 5 (area %.1f)" % A_w, F / A_w, F / A_w)
print("   -> the plastic under the head creeps: preload decays 30-60 % in days; use a steel washer + a Nyloc/thread-lock, or clamp metal-on-metal")

# 4. bearing seat press fit (hoop stress): thick-wall ring, E, interference delta on D=8
for mat, E in {"PETG": 1.9e3, "PLA": 3.3e3, "PETG-CF": 4.5e3, "PA12-CF": 5.0e3}.items():
    for delta in (0.05, 0.10):
        eps = delta / 8.0
        ro, ri = 8.0 / 2 + 3.45, 4.0   # wall 3.45 around the seat (finger lug)
        k = (ro**2 + ri**2) / (ro**2 - ri**2)
        sigma = E * eps / 2 * (1 + 0)  # approx: radial interference shared by bearing (stiff) and ring -> ring takes ~all
        sig_hoop = E * eps * (1 + 0) * 0.5 * (k + 1) / k  # crude
        print(f"4. bearing seat {mat:8s} interference {delta:.2f} mm: initial hoop ~{sig_hoop:5.1f} MPa (relaxes 30-50 % in weeks)")

# 5. bearing hub / axle load: 4 mm steel stub in 8 mm seat is not the weak point; the lug bending is
#    cantilever lug: radial load 2*T_STALL (both strands) = 90 N on one hinge, lug 3.8 wide, arm 4 mm
Fj = 2 * T_STALL
sigma_b = 6 * (Fj / 2 * 4.0) / (3.8 * 6.0**2) * 0 + (Fj / 2) / (3.8 * 6.0)  # shear-ish bearing on a 6 mm web
row("5. lug web shear/bearing, 90 N joint reaction on 2 lugs", (2 * T0 / 2) / (3.8 * 6.0), Fj / 2 / (3.8 * 6.0))

# 6. Z-direction check: finger tip force 2.5 N at 100 mm arm -> bending moment 0.25 N.m on the proximal root
M = 2.5 * 0.100  # N.m  (fingertip force at 100 mm)
for H, W, label in ((16.0, 18.0, "finger root section 18x16 solid (tube-like wall, 3 walls = 1.2 mm)"),):
    # hollow box with 1.2 mm walls
    I = (W * H**3 - (W - 2.4) * (H - 2.4) ** 3) / 12.0
    sig = M * 1000 * (H / 2) / I
    print(f"6. bending at finger root, 2.5 N tip force: {sig:.1f} MPa along the finger axis (in-layer if printed on its side; Z-tension if printed standing)")
    M45 = 20.0 * 0.100  # 20 N tip force goal
    print(f"   at the 20 N goal: {M45*1000*(H/2)/I:.1f} MPa -> needs PETG-CF/PA-CF or thicker walls if the finger stands upright")

# 7. print time and cost estimates for the V2 hand (volumes from out/assembly/mass.json, effective g from there)
masses = {"fingers+pads": 46.9 + 14.7, "thumb": 23.7, "palm": 46.5, "wrist": 5.9, "forearm": 117.5}
tot = sum(masses.values())
print(f"7. printed mass (slicer estimate of mass.json) {tot:.0f} g; PETG at EUR 20/kg = EUR {tot/1000*20:.1f}; PETG-CF at EUR 40/kg = EUR {tot/1000*40:.1f}")
for g_per_h, name in ((12, "small detailed parts (fingers), 0.16-0.2 mm"), (25, "big parts (forearm, palm), 0.2 mm 0.4 nozzle")):
    print(f"   at {g_per_h} g/h ({name}): {tot/g_per_h:.1f} h for the whole {tot:.0f} g")
