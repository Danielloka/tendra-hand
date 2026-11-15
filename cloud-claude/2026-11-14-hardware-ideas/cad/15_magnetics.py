"""15 · Joint-angle magnetics for the Tendra V2.1 sensing plan (agent 15, 2026-11-15).

numpy only:  uv run --no-project --with numpy python cad/15_magnetics.py   (~1-2 min)
Writes out/15/magnetics.json and out/15/magnetics.txt.

What it computes (all fields in mT, distances in mm):
 1. Field at the Hall element from the joint's own diametric magnet (surface-charge model of a uniformly
    magnetised cylinder, Br given), for the magnet / gap variants.
 2. Effect of an MR84ZZ bearing around the magnet: races, shields and balls as soft-steel dipole cells
    (coupled linear solve, chi_int 100 -> cube chi_e 2.9; saturation checked). Gives (a) the field shunting,
    (b) the angle ripple from the balls (they move with the cage at ~half the joint angle, so this part is NOT
    removed by calibration), (c) the error when the chip sits 0.15 mm off the axis, (d) a hysteresis bound.
 3. Crosstalk: field at every chip from every other magnet in the hand (dipole far field), as an angle error
    bound atan(B_other / B_own), at q = 0 and in a key pinch.
Assumptions are printed. Steel model is first-order; the bench test (08 R9) decides.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent.parent / "out" / "15"
OUT.mkdir(parents=True, exist_ok=True)
MU0 = 4e-7 * math.pi
LOG: list[str] = []


def p(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    LOG.append(s)


# ------------------------------------------------------------------ magnet as surface charges (SI inside, mm out)
def magnet_charges(d, t, x0, br=1.32, axis="diametric", n_phi=96, n_z=16, n_r=10):
    """cylinder axis along x from x0 to x0+t, diameter d (mm). diametric: M along +y. axial: M along +x.
    returns (positions m, charges A*m) so that H = sum q (r-r')/(4 pi |r-r'|^3)."""
    M = br / MU0
    R = d / 2 * 1e-3
    pos, q = [], []
    if axis == "diametric":
        ph = (np.arange(n_phi) + 0.5) * 2 * np.pi / n_phi
        xs = x0 * 1e-3 + (np.arange(n_z) + 0.5) * t * 1e-3 / n_z
        P, X = np.meshgrid(ph, xs)
        dA = R * (2 * np.pi / n_phi) * (t * 1e-3 / n_z)
        pos = np.stack([X.ravel(), R * np.cos(P).ravel(), R * np.sin(P).ravel()], 1)
        q = M * np.cos(P).ravel() * dA  # sigma = M . n, n = (0, cos, sin)
    else:
        rr = (np.arange(n_r) + 0.5) * R / n_r
        ph = (np.arange(n_phi) + 0.5) * 2 * np.pi / n_phi
        Rg, P = np.meshgrid(rr, ph)
        dA = Rg * (R / n_r) * (2 * np.pi / n_phi)
        for xf, sgn in ((x0 + t, 1.0), (x0, -1.0)):
            pp = np.stack([np.full(Rg.size, xf * 1e-3), (Rg * np.cos(P)).ravel(), (Rg * np.sin(P)).ravel()], 1)
            pos.append(pp)
            q.append(sgn * M * dA.ravel())
        pos, q = np.concatenate(pos), np.concatenate(q)
    return np.asarray(pos), np.asarray(q)


def H_charges(pos, q, pts):
    pts = np.atleast_2d(pts) * 1e-3
    d = pts[:, None, :] - pos[None, :, :]
    r3 = np.linalg.norm(d, axis=2) ** 3
    return (q[None, :, None] * d / r3[:, :, None]).sum(1) / (4 * np.pi)


def rot_x(v, th):
    c, s = math.cos(th), math.sin(th)
    v = np.atleast_2d(v)
    return np.stack([v[:, 0], c * v[:, 1] - s * v[:, 2], s * v[:, 1] + c * v[:, 2]], 1)


def H_dip(m, src, pts):
    """dipole moment m (A m^2) at src (mm) -> H (A/m) at pts (mm)."""
    pts, src, m = np.atleast_2d(pts) * 1e-3, np.atleast_2d(src) * 1e-3, np.atleast_2d(m)
    d = pts[:, None, :] - src[None, :, :]
    r = np.linalg.norm(d, axis=2)
    r = np.where(r < 1e-12, np.inf, r)
    rh = d / r[:, :, None]
    mdotr = (m[None, :, :] * rh).sum(2)
    return ((3 * mdotr[:, :, None] * rh - m[None, :, :]) / r[:, :, None] ** 3).sum(1) / (4 * np.pi)


# ------------------------------------------------------------------ MR84ZZ steel cells (axis x, centred x0..x0+3)
def mr84_cells(x0, h=0.25, n_balls=7, ball_phase=0.0):
    """inner race r 2.0-2.6, outer race r 3.4-4.0, shields r 2.7-3.6 x 0.2 on both faces, balls Ø1.0 at r 3.0
    (dimensions of a generic MR84ZZ, assumed). returns dict part -> (centres mm, volumes mm^3)."""
    parts = {}
    for name, r0, r1, xa, xb in (("inner", 2.0, 2.6, x0, x0 + 3.0), ("outer", 3.4, 4.0, x0, x0 + 3.0),
                                 ("shield_a", 2.7, 3.6, x0 + 0.05, x0 + 0.25), ("shield_b", 2.7, 3.6, x0 + 2.75, x0 + 2.95)):
        g = np.arange(-r1, r1 + 1e-9, h)
        xs = np.arange(xa + h / 2, xb, h) if xb - xa > h else np.array([(xa + xb) / 2])
        hx = h if xb - xa > h else xb - xa
        Y, Z = np.meshgrid(g, g)
        rr = np.hypot(Y, Z)
        sel = (rr >= r0) & (rr <= r1)
        yz = np.stack([Y[sel], Z[sel]], 1)
        c = np.concatenate([np.column_stack([np.full(len(yz), x), yz]) for x in xs])
        parts[name] = (c, np.full(len(c), h * h * hx))
    ang = ball_phase + np.arange(n_balls) * 2 * np.pi / n_balls
    parts["balls"] = (np.column_stack([np.full(n_balls, x0 + 1.5), 3.0 * np.cos(ang), 3.0 * np.sin(ang)]),
                      np.full(n_balls, 4 / 3 * np.pi * 0.5 ** 3))
    return parts


def solve_steel(cells, H_ext, chi_e=2.9, iters=40, relax=0.6):
    """coupled dipoles: m_i = chi_e V_i (H_ext,i + sum_j H_dip(m_j)). cells (N,3) mm, V (N,) mm^3."""
    c, V = cells
    Vm = V * 1e-9
    m = chi_e * Vm[:, None] * H_ext
    for _ in range(iters):
        Hc = np.zeros_like(H_ext)
        for k in range(0, len(c), 600):
            Hc[k:k + 600] = H_dip(m, c, c[k:k + 600])
        m_new = chi_e * Vm[:, None] * (H_ext + Hc)
        m = relax * m_new + (1 - relax) * m
    M = np.linalg.norm(m, axis=1) / Vm
    return m, M


# ------------------------------------------------------------------ 1+2: own field, bearing effects
def joint_case(name, d, t, x_mag, x_sens, bearing_x0=None, steel_chi=2.9, off_axis=0.0, sens_axis_mm=0.0):
    pos, q = magnet_charges(d, t, x_mag)
    sens = np.array([[x_sens, off_axis, 0.0]])
    res = dict(name=name, magnet=f"Ø{d} x {t} diametric N45 (Br 1.32 T)", x_mag=[x_mag, x_mag + t], x_sensor=x_sens,
               gap_face_to_element=round(x_sens - (x_mag + t), 2))
    th = np.radians(np.arange(-10, 101, 10.0))

    def field_at(theta, steel_parts=None, ball_phase_fn=None):
        out = []
        for a in theta:
            pr = rot_x(pos, a)  # rotate the charge cloud (magnetisation pattern rotates with the joint)
            B = MU0 * H_charges(pr, q, sens)[0]
            if steel_parts is not None:
                Bs = steel_field(steel_parts, pr, a, ball_phase_fn)
                B = B + Bs
            out.append(B * 1e3)
        return np.array(out)

    def steel_field(parts_fn, pr, a, ball_phase_fn):
        parts = parts_fn(ball_phase_fn(a) if ball_phase_fn else 0.0)
        Bt = np.zeros(3)
        for nm, (c, V) in parts.items():
            if nm == "inner_corot":
                c = rot_x(c, a)
            Hx = H_charges(pr, q, c)
            m, M = solve_steel((c, V), Hx, chi_e=steel_chi, iters=12 if len(c) > 50 else 1)
            Bt += MU0 * H_dip(m, c, sens)[0]
        return Bt

    B0 = field_at(th)
    ang0 = np.degrees(np.unwrap(np.arctan2(B0[:, 2], B0[:, 1])))
    res["B_inplane_mT"] = round(float(np.hypot(B0[:, 1], B0[:, 2]).mean()), 1)
    res["B_axial_mT"] = round(float(np.abs(B0[:, 0]).max()), 1)
    err0 = ang0 - np.degrees(th)
    err0 -= err0.mean()
    res["err_no_steel_deg"] = round(float(np.abs(err0).max()), 3)
    if bearing_x0 is not None:
        # linear superposition is not valid for balls moving; do the full thing at each angle but coarse cells
        def parts_fn(phase):
            pr_ = mr84_cells(bearing_x0, h=0.3, ball_phase=phase)
            return pr_

        # magnet + inner race co-rotate (inner race on the magnet's body), outer/shields/balls on the chip side.
        # By symmetry the inner race only scales the field; it is included through the static solve below.
        Bs = field_at(th, steel_parts=parts_fn, ball_phase_fn=lambda a: a / 2)
        angs = np.degrees(np.unwrap(np.arctan2(Bs[:, 2], Bs[:, 1])))
        e = angs - np.degrees(th)
        e -= e.mean()
        res["B_inplane_with_MR84_mT"] = round(float(np.hypot(Bs[:, 1], Bs[:, 2]).mean()), 1)
        res["shunt_fraction"] = round(1 - res["B_inplane_with_MR84_mT"] / res["B_inplane_mT"], 3)
        res["err_with_MR84_ball_ripple_deg"] = round(float(np.abs(e).max()), 3)
        # ball contribution alone: same angles, balls frozen (cage stuck) vs moving -> non-repeatable part
        Bf = field_at(th, steel_parts=parts_fn, ball_phase_fn=lambda a: 0.0)
        angf = np.degrees(np.unwrap(np.arctan2(Bf[:, 2], Bf[:, 1])))
        dnr = angs - angf
        res["non_repeatable_cage_deg"] = round(float(np.ptp(dnr) / 2), 3)
        # hysteresis bound: race lag ~ Hc/H_applied at the race, times the race share of the field
        c_out, _ = mr84_cells(bearing_x0, h=0.3)["outer"]
        Hr = np.linalg.norm(H_charges(pos, q, c_out), axis=1).mean()
        hc = 3000.0  # A/m, hardened 52100 coercivity, assumed 2-4 kA/m [bg]
        res["H_at_outer_race_kA_m"] = round(float(Hr / 1e3), 1)
        res["hysteresis_bound_deg"] = round(float(math.degrees(hc / Hr) * res["shunt_fraction"] + 0.0), 3)
    return res


p("=" * 100)
p("1-2. Own magnet field at the Hall element; MR84ZZ effects (on-axis chip)")
p("=" * 100)
p("Model: Br 1.32 T (N45), magnet axis = joint axis (x). Hall element 0.45 mm under the chip face (SOT-23: verify).")
p("MR84ZZ: inner race r 2.0-2.6, outer r 3.4-4.0, 7 balls Ø1.0 on r 3.0, steel shields; chi_e 2.9 (assumed).")
cases = []
# A: 07 as drawn: magnet Ø3x2 in the axle end x 5.2-7.2, inside MR84 at 4.4-7.4; chip face 7.6 -> element 8.05
cases.append(joint_case("A 07 MCP as drawn (magnet inside race, chip face 0.4)", 3.0, 2.0, 5.2, 8.05, bearing_x0=4.4))
# B: 15 plan: Ø3x1.5 magnet whose face is 1.0 mm proud of the race (race ends 7.4, magnet 6.9-8.4)
cases.append(joint_case("B 15 plan at the MCP: Ø3x1.5 in the axle end, die gap 1.5 (magnet face 1.0 proud of the race end)", 3.0, 1.5, 6.9, 8.4 + 1.5, bearing_x0=4.4))
# C: magnet 2.0 mm proud (review #7 rule), Ø3x1.5 at 7.9-9.4
cases.append(joint_case("C magnet face 2.0 proud, Ø3x1.5, die gap 1.5", 3.0, 1.5, 7.9, 9.4 + 1.5, bearing_x0=4.4))
# D: no steel bearing (brass bush side: PIP -x, DIP both sides, ceramic MR84), Ø3x1.5, gap 0.5
cases.append(joint_case("D no steel (brass bush / ZrO2 MR84), Ø3x1.5, die gap 1.5", 3.0, 1.5, 5.0, 6.5 + 1.5))
# E: abd: Ø3x1 glued on the end of a 316 stub, MR63 not modelled (smaller), gap 0.5
cases.append(joint_case("E abd stub end, Ø3x1, die gap 1.5", 3.0, 1.0, 0.0, 1.0 + 1.5))
# F: off-axis chip 0.15 mm (placement error) for case B
cases.append(joint_case("F = B with the chip 0.15 mm off axis", 3.0, 1.5, 6.9, 9.9, bearing_x0=4.4, off_axis=0.15))
for c in cases:
    p(json.dumps(c))

# ------------------------------------------------------------------ 3: crosstalk in the hand
p("")
p("=" * 100)
p("3. Crosstalk between all magnets of the hand (dipole far field)")
p("=" * 100)
BR = 1.32
def mom(d, t):
    return BR / MU0 * math.pi * (d / 2) ** 2 * t * 1e-9

FING = {"index": ((20.0, -2.0, 92.0), (42, 24, 23)), "middle": ((0.0, -2.0, 96.0), (45, 27, 24)),
        "ring": ((-19.5, -2.0, 93.0), (43, 26, 23)), "little": ((-38.0, -2.0, 85.0), (34, 20, 21))}
mags, chips = [], []  # (name, pos, moment vector)
m_joint = mom(3.0, 1.5)
for f, (o, L) in FING.items():
    o = np.array(o)
    zp, zd = o[2] + L[0], o[2] + L[0] + L[1]
    # joint magnets on the +x end of each axle, chips 1.9 mm further out (case B geometry, local x 8.4 / 9.35)
    for j, z, xm, xc in (("mcp_flex", o[2], 7.65, 9.35), ("pip", zp, 6.8, 8.4), ("dip", zd, 6.3, 7.9)):
        mags.append((f"{f}_{j}", np.array([o[0] + xm, o[1], z]), np.array([0, m_joint, 0])))
        chips.append((f"{f}_{j}", np.array([o[0] + xc, o[1], z]), "x"))
    # abd: axis along y at z -13, magnet on the dorsal stub end y +17.2, chip +18.9
    mags.append((f"{f}_mcp_abd", np.array([o[0], o[1] + 17.2, o[2] - 13]), np.array([m_joint * 0.67, 0, 0])))
    chips.append((f"{f}_mcp_abd", np.array([o[0], o[1] + 18.9, o[2] - 13]), "y"))
    # tip magnet Ø3x1.5 axial (y), 3.9 mm under the chip
    mags.append((f"{f}_tip", np.array([o[0], o[1] - 8.2, zd + 15]), np.array([0, -mom(3.0, 1.5), 0])))
    chips.append((f"{f}_tip", np.array([o[0], o[1] - 4.3, zd + 15]), "tip"))
# thumb: CMC centre P, thumb along u (approx.), joints at 0 / 46.7 / 82.7, tip 114.3
P = np.array([39.79, -17.86, 50.44])
u = np.array([0.35, -0.45, 0.82]); u /= np.linalg.norm(u)
for j, s in (("cmc_rot", -6.0), ("cmc_flex", 6.0), ("mcp_flex", 46.7), ("ip", 82.7)):
    pos = P + u * max(s, 0) + (np.array([0, 0, -6.0]) if j == "cmc_rot" else np.array([8.0, 0, 0]) * (j != "cmc_rot"))
    mags.append((f"thumb_{j}", pos, np.array([0, m_joint, 0])))
    chips.append((f"thumb_{j}", pos + np.array([1.7, 0, 0]), "x"))
mags.append(("thumb_tip", P + u * 104 + np.array([0, -8.2, 0]), np.array([0, mom(3.0, 1.5), 0])))
chips.append(("thumb_tip", P + u * 104 + np.array([0, -4.3, 0]), "tip"))

own_joint = cases[1]["B_inplane_with_MR84_mT"]  # worst case: race-shunted field
own_abd = cases[4]["B_inplane_mT"]
rows = []
for cn, cp, kind in chips:
    Bo = np.zeros(3)
    worst = ("", 0.0)
    for mn, mp, mv in mags:
        if mn == cn:
            continue
        B = MU0 * H_dip(mv, mp, cp)[0] * 1e3
        Bo += B
        if np.linalg.norm(B) > worst[1]:
            worst = (mn, float(np.linalg.norm(B)))
    bmag = float(np.linalg.norm(Bo))
    if kind == "tip":
        # tip: compare with 17 mT per mm of skin compression (05) -> equivalent false force at ~0.05 N/mT
        eq = bmag * 0.045
        rows.append(dict(chip=cn, B_others_mT=round(bmag, 3), worst=worst[0], equiv_force_N=round(eq, 3)))
    else:
        own = own_abd if cn.endswith("abd") else own_joint
        rows.append(dict(chip=cn, B_others_mT=round(bmag, 3), worst=worst[0],
                         angle_err_bound_deg=round(math.degrees(math.atan(bmag / own)), 3)))
for r in rows:
    p(json.dumps(r))
worst_joint = max((r for r in rows if "angle_err_bound_deg" in r), key=lambda r: r["angle_err_bound_deg"])
p(f"worst joint crosstalk (q = 0, static -> mostly calibrated out): {worst_joint}")

# pinch: index tip magnet and thumb tip magnet 10 mm apart, face to face
for dd in (8.0, 10.0, 15.0):
    B = MU0 * H_dip(np.array([0, mom(3.0, 1.5), 0]), np.array([0, dd, 0]), np.array([0, 0.0 - 3.9, 0]))[0] * 1e3
    p(f"pinch: other tip magnet {dd:4.1f} mm from this magnet (coaxial): field at this chip {np.linalg.norm(B):.2f} mT "
      f"= {np.linalg.norm(B) * 0.045:.3f} N false normal force (before model-based subtraction)")

# chip near a tendon? PE line non-magnetic. steel parts in the finger near chips (dowels, axle) listed for the spec.
json.dump(dict(cases=cases, crosstalk=rows, worst_joint=worst_joint), open(OUT / "magnetics.json", "w"), indent=1)
(OUT / "magnetics.txt").write_text("\n".join(LOG) + "\n")
print("wrote", OUT / "magnetics.json")
