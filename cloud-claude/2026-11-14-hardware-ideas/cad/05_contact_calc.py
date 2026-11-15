"""05 contact surfaces: friction vs squeeze, pad stiffness, Hertz patch, magnet signal, wire fatigue.
Run: uv run --no-project --with numpy python cad/05_contact_calc.py
All inputs are assumptions (marked), outputs are hand calculations."""
import numpy as np
g = 9.81
# ---- 1. friction coefficients (dry, vs smooth household objects: glass, painted metal, ABS, cardboard)
MU = {"PETG bare": 0.30, "PLA bare": 0.30, "TPU 95A smooth": 0.60, "TPU 85A smooth": 0.80,
      "TPU 95A + 0.8mm grooves": 0.70, "Silicone Shore A40": 1.0, "Dragon Skin 10 (A10)": 1.3, "Ecoflex 00-30 (00-30)": 1.6}
# Squeeze needed. SF = safety factor 1.5 (dynamic, inertia of arm moves)
SF = 1.5
def need(W, contacts_mu_sum):  # normal force per contact, all contacts equal normal N: N = SF*W / sum(mu)
    return SF * W / contacts_mu_sum
print("== squeeze force per fingertip to hold a vertical weight, SF 1.5")
masses = [0.2, 0.5, 1.0]
grasps = {"2-finger pinch (2 contacts)": 2, "tripod (3 contacts, thumb+2)": 3,
          "power wrap (thumb + 4 fingers + palm ~ 6 eff.)": 6}
for gname, n in grasps.items():
    print(gname)
    for m in masses:
        row = []
        for mat, mu in MU.items():
            N = need(m * g, n * mu)
            row.append(f"{mat}: {N:5.2f}")
        print(f"  {m*1000:4.0f} g | " + " | ".join(row))
# What can a tip with F_tip (sustained 1.7 N / stall 3.4 N / realistic 2.5 N after friction) hold at the 2-finger pinch
print("\n== max mass held by a 2-finger pinch with tip normal force F (SF 1.5)")
for F in (1.7, 2.5, 3.4, 9.0):
    print(f"F={F} N: " + " | ".join(f"{mat}: {2*mu*F/(SF*g)*1000:5.0f} g" for mat, mu in MU.items()))
print("\n== PETG vs silicone A40 ratio of squeeze:", MU["Silicone Shore A40"] / MU["PETG bare"], "; vs Dragon Skin 10:", MU["Dragon Skin 10 (A10)"]/MU["PETG bare"])

# ---- 2. Hertz patch and torsional friction: sphere R on a soft pad, E* ~ pad modulus
def gent_E(shoreA):  # MPa, Gent relation
    return 0.0981 * (56 + 7.62336 * shoreA) / (0.137505 * (254 - 2.54 * shoreA))
print("\n== Young's modulus from Shore A (Gent)")
for s in (10, 30, 40, 50, 85, 95):
    print(f"Shore A {s}: E = {gent_E(s):.2f} MPa")
mats = {  # E in MPa (Ecoflex 00-30: 100% modulus 10 psi=69 kPa -> E ~ 1.7*69 = 0.12 MPa; DS10 22psi -> 0.26; DS30 ~1.0)
    "Ecoflex 00-30": 0.12, "Dragon Skin 10": 0.26, "Dragon Skin 30": 1.0, "Silicone A40 (Smooth-Sil 940)": 1.6,
    "TPU 85A": 9.0, "TPU 95A": 26.0, "PETG": 2000.0}
print("\n== Hertz patch of a fingertip pad (R=8 mm tip) on a FLAT object, normal force N; patch radius a = (3 N R /(4 E*))^(1/3), E*=E/(2(1-nu^2))*... ~E/1.5 (rubber nu .5)")
R = 8e-3
for N in (1.0, 3.0):
    print(f"N={N} N")
    for k, E in mats.items():
        Es = E * 1e6 / 1.5 if E < 100 else E * 1e6 / 1.8
        a = (3 * N * R / (4 * Es)) ** (1 / 3)
        a = min(a, 7e-3)  # a thin pad bottoms out: patch limited by pad width (~14 mm)
        mu = 1.0 if E < 5 else 0.6
        Mz = 2 / 3 * mu * N * a  # twist torque capacity, N m, uniform pressure disc approx
        print(f"  {k:30s} a = {a*1000:4.1f} mm, twist capacity = {Mz*1000:5.2f} N mm")
# ---- 3. pad as force filter. Series stiffness at the fingertip
# Tendon axial stiffness k = E A / L, A = pi/4 (0.4mm)^2 = 0.126 mm2, L ~ 0.4 m (spool to fingertip drum).
#   nylon E~3 GPa -> 0.94 N/mm ; Dyneema braid E_eff~40 GPa -> 12.6 N/mm; plus sheath compression (not counted)
print("\n== stiffness chain at the fingertip (N/m); tendon k reflected to the tip by (drum/lever)^2, drum 7 mm, lever 60 mm")
d_drum, lever = 7.0, 60.0
for name, kt in {"nylon line 0.94 N/mm": 940.0, "Dyneema braid 12.6 N/mm": 12600.0, "ideal stiff 100 N/mm": 100e3}.items():
    k_tip = kt * (d_drum / lever) ** 2
    print(f" tendon {name}: tip stiffness (tendon only) = {k_tip:7.0f} N/m = {k_tip/1000:.2f} N/mm")
    for pname, E, t, A in (("PETG tip (no pad)", 2000e6, 1e-3, 40e-6), ("TPU95 2mm,10x10", 26e6, 2e-3, 100e-6),
                           ("DS10 3mm,10x10", 0.26e6, 3e-3, 100e-6), ("Ecoflex 00-30 5mm,10x10", 0.12e6, 5e-3, 100e-6)):
        kp = E * A / t  # compressive stiffness of a flat slab (upper bound; real is softer for small patches)
        kser = 1 / (1 / k_tip + 1 / kp)
        m = 0.05; v = 0.1  # 50 g effective mass, 0.1 m/s closing speed
        Fpk = v * np.sqrt(kser * m)
        print(f"    + {pname:24s} k_pad={kp:11.0f} N/m  k_series={kser:7.0f}  impact peak {Fpk:5.2f} N  force for 0.3 mm tip jitter {kser*0.3e-3:5.3f} N")
print("  => the tendon is already 30..1000x softer than any fingertip pad: pads do NOT filter at the fingertip.")
print("\n== PALM / phalanx pad vs a stiff ARM (the arm is position-controlled, k_arm ~ 50 N/mm at the hand, assumed)")
k_arm = 50e3
for pname, E, t, A in (("PETG palm", 2000e6, 1e-3, 2400e-6), ("TPU95 2mm, 40x60", 26e6, 2e-3, 2400e-6),
                       ("DS10 3mm, 40x60", 0.26e6, 3e-3, 2400e-6), ("Ecoflex 00-30 10mm, 40x60", 0.12e6, 10e-3, 2400e-6),
                       ("Ecoflex 00-30 10mm, 20x30 (phalanx-size)", 0.12e6, 10e-3, 600e-6)):
    kp = E * A / t
    kser = 1 / (1 / k_arm + 1 / kp)
    for dx in (0.5e-3,):
        print(f"  {pname:42s} k_pad={kp/1000:9.1f} N/mm  series {kser/1000:6.1f} N/mm  force for 0.5 mm arm error {kser*dx:6.2f} N  (rigid: {k_arm*dx:5.1f} N)")
# ---- 4. magnetic tactile: field on axis of a cylindrical magnet
def Bz(z, Rm, L, Br):  # z from magnet face (m)
    return Br / 2 * ((z + L) / np.sqrt(Rm**2 + (z + L)**2) - z / np.sqrt(Rm**2 + z**2))
print("\n== magnet field on axis, N45 (Br 1.34 T)")
for (D, L) in ((3, 1.5), (4, 1.0), (4, 2.0), (5, 2.0)):
    row = []
    for z in (2.0, 3.0, 4.0, 5.0):
        row.append(f"z={z}: {Bz(z*1e-3, D/2e3, L*1e-3, 1.34)*1e3:5.1f} mT")
    print(f"  D{D}x{L}: " + " | ".join(row))
print("  MLX90393 range ~ +-5 .. +-50 mT depending on gain (x/y 0.15-1.2 uT/LSB class); TMAG5273 A1: +-40 mT, A2: +-80 mT")
for (D, L) in ((3, 1.5), (4, 1.0)):
    z0 = 4.0; dz = 1e-3
    s = (Bz((z0 - 1) * 1e-3, D/2e3, L*1e-3, 1.34) - Bz(z0 * 1e-3, D/2e3, L*1e-3, 1.34)) * 1e3
    print(f"  D{D}x{L} dome 4 mm: compress 1 mm raises B by {s:.1f} mT; 1 mT resolution -> ~{1/s:.2f} mm -> force step with DS10 dome k~(E=0.26MPa, A=12 mm2, h=4mm)={0.26e6*12e-6/4e-3:.0f} N/m: {0.26e6*12e-6/4e-3*1e-3/s:.3f} N per mT")
# ---- 5. wire fatigue: torsion of a free bundle along the joint axis vs bending of a loop
print("\n== wires across a joint")
def torsion_strain(r_cond_bundle, theta_deg, L):  # outer-wire helical strain approx (r*theta/L)^2/2
    return (r_cond_bundle * np.radians(theta_deg) / L) ** 2 / 2
for L in (6, 10, 20):
    print(f" bundle r=0.5mm twisted 95 deg over L={L} mm: strain {torsion_strain(0.5, 95, L)*100:.3f} %")
for t, Rb in ((0.1, 3), (0.1, 5), (0.1, 10), (0.05, 3)):
    print(f" flex-PCB RA Cu loop, t={t} mm bend radius {Rb} mm: strain {t/(2*Rb)*100:.2f} % (RA Cu ~1e6 cycles at <= 0.3..0.5 %)")
print(" single 36AWG strand (0.127 mm solid) bent to R=3 mm: ", f"{0.127/2/3*100:.1f} % -> dies in 1e3..1e4 cycles; 7-strand 0.05 mm each: {0.05/2/3*100:.1f} %")
# ---- 6. Hall angle sensor
print("\n== joint angle sensor resolution: 12-bit => 0.088 deg; AS5600 12 bit; MT6701 14 bit => 0.022 deg; fingertip 80 mm lever: 0.088 deg = %.3f mm" % (np.radians(0.088) * 80))
