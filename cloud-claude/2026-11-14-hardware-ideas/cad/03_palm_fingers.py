"""03 - finger spread with tapered phalanges, and a fixed palm arch (study 2026-11-14, agent 03).
uv run --no-project --with numpy python cad/03_palm_fingers.py
Geometry only. Frame: x = across the palm (+ thumb side), z = along the finger, fingers straight.
"""
import math
import numpy as np

print("A. How far can one finger adduct toward a straight neighbour (relative angle)?")
print("   condition at distance z from the pivot: z*sin(th) <= gap(z); gap = pitch - (w_a(z)+w_b(z))/2 (both taper)")
for label, pitch, w0, w_tip in [
    ("index-middle  untapered", 20.0, 18.0, 18.0),
    ("index-middle  taper 18->14", 20.0, 18.0, 14.0),
    ("index-middle  taper 18->13", 20.0, 18.0, 13.0),
    ("ring-little   untapered (17/15.5)", 18.5, 17.0, 17.0),
    ("ring-little   taper 17->13", 18.5, 16.25, 13.0),
]:
    Lf = 95.0  # pivot to tip incl. the 11.5 mm abd offset
    zs = np.linspace(10, Lf, 200)
    w = w0 + (w_tip - w0) * zs / Lf
    gap = pitch - w
    th = np.degrees(np.arcsin(np.minimum(1, gap / zs)))
    i = th.argmin()
    print(f"   {label:38s} gap at pivot {gap[0]:.1f} mm  -> max relative adduction {th[i]:4.1f} deg (limit at z={zs[i]:.0f} mm)")
print("   Tapering the phalanges like a human finger (18 -> 14 mm) buys ~3x more adduction for free (less plastic).")
print("   The ring/little 9 deg limit in V2 is about the staggered YOKES, not the phalanges: yokes need the same taper")
print("   (narrower yoke on the side facing the neighbour) or 2 mm more pitch.")

print()
print("B. Fixed palm arch (zero DOF): rotate ring and little finger modules about the z-line through the middle MCP")
x_mcp = {"index": 20.0, "middle": 0.0, "ring": -19.5, "little": -38.0}
for phi_ring, phi_little in ((0, 0), (5, 10), (6, 12), (8, 15)):
    out = []
    for f, ph in (("ring", phi_ring), ("little", phi_little)):
        x = x_mcp[f]
        dy = -abs(x) * math.sin(math.radians(ph))   # toward the palm side (-y)
        dx = abs(x) * (1 - math.cos(math.radians(ph)))
        out.append(f"{f}: MCP moves {abs(dy):.1f} mm palmward, {dx:.1f} mm toward the middle finger")
    # tip of the little finger when its flexion plane is tilted phi toward the centre: lateral shift at 70 mm from the MCP in a fist
    sh = 70.0 * math.sin(math.radians(phi_little))
    print(f"   ring {phi_ring:2d} deg / little {phi_little:2d} deg: " + "; ".join(out) + f"; little fingertip leans {sh:.1f} mm toward the thumb")
print("   Human metacarpal arch: ring ~5-8 deg, little ~10-15 deg. A 12 deg arch moves the little MCP 7.9 mm palmward;")
print("   the thumb should reach the little fingertip with less cmc rotation (HYPOTHESIS, check in the V2 pose solver).")
print()
print("C. Passive hinged arch (TPU strap + stops) would add: 1 hinge, 8 sheaths crossing it. Not worth it before the fixed arch is tested.")
