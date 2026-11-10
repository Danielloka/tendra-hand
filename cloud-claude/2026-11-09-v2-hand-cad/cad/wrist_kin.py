"""Numeric side of the V2 wrist (no CadQuery): sheath bundle path, tube length change, bend radii,
bore sizing, and the 2-RSS push-rod drive (inverse kinematics, transmission, torque capacity).

Frame: hand frame (mm), origin = wrist centre. Kinematic chain (same as sim/convert_v1_wrist.py):
forearm -> wrist_flex (about +X, + = fingers toward -Y) -> ring -> wrist_dev (about +Y of the ring,
+ = fingers toward +X) -> palm. Palm pose R_palm = Rx(flex) @ Ry(dev).

    uv run --no-project --with numpy --with scipy python cad/wrist_kin.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import params as P  # noqa: E402

DEG = math.pi / 180

# --- bundle --------------------------------------------------------------------------------------
PITCH = 2.15  # tube pitch in the bundle combs (2.0 OD + 0.15)
N_COLS, N_ROWS = 8, 4  # 8 along x, 4 along y = 32 sheaths
CABLE_D = 3.0  # palm sensor cable (4 x 30 AWG in a sleeve), runs at the -X end of the bundle
Z_PALM_CLAMP = 18.0  # palm-side sheath clamp lower face (clamp bridge 3 mm into the palm base)
Z_FORE_COMB = -55.0  # forearm-side bundle comb (slide guide) top face
RING_H_Y = 7.0  # ring height (z) at its +-Y ends
RING_H_X = 7.0  # ring height at its +-X ends


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# Joint order (see report 03): the deviation axis is the OUTER one (forearm lugs at +-Y carry the
# wrist_dev pins), the flexion axis the inner one (palm lugs at +-X). R_palm = Ry(dev) @ Rx(flex).
# Reason: the 130 deg flexion range swings the bundle +-18 mm in Y around the axis, so the ring must be
# ~37 mm deep in Y; that depth fits on the forearm side (forearm ~60 deep), not on the palm (28 deep).
ORDER = "dev_outer"


def R_ring(flex_deg, dev_deg=0.0):
    return Ry(dev_deg * DEG) if ORDER == "dev_outer" else Rx(flex_deg * DEG)


def R_palm(flex_deg, dev_deg):
    if ORDER == "dev_outer":
        return Ry(dev_deg * DEG) @ Rx(flex_deg * DEG)
    return Rx(flex_deg * DEG) @ Ry(dev_deg * DEG)


BUNDLE_ROWS = (4, 6, 6, 6, 6, 4)  # tubes per row (rows along y): a round 32-cell bundle, 12.9 x 12.9


def tube_offsets():
    """(a, b) offset of each of the 32 tube cells in the bundle section (x, y), rows from -y."""
    out = []
    ny = len(BUNDLE_ROWS)
    for j, n in enumerate(BUNDLE_ROWS):
        y = (j - (ny - 1) / 2) * PITCH
        for i in range(n):
            out.append(((i - (n - 1) / 2) * PITCH, y))
    return np.array(out)


def bundle_half():
    o = tube_offsets()
    return float(np.abs(o[:, 0]).max() + 1.0), float(np.abs(o[:, 1]).max() + 1.0)


def _hermite(p0, t0, p1, t1, n):
    s = np.linspace(0, 1, n)[:, None]
    h00, h10 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s
    h01, h11 = -2 * s**3 + 3 * s**2, s**3 - s**2
    return h00 * p0 + h10 * t0 + h01 * p1 + h11 * t1


def _radii(pts):
    a, b, c = pts[:-2], pts[1:-1], pts[2:]
    ab, bc, ca = (np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1),
                  np.linalg.norm(a - c, axis=1))
    cr = np.linalg.norm(np.cross(b - a, c - b), axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        r = ab * bc * ca / (2 * cr)
    return np.where(cr < 1e-12, np.inf, r)


def bundle_centreline(flex, dev, n=121, k=None):
    """Centreline of the bundle from the palm clamp (palm frame z = Z_PALM_CLAMP, leaving along
    -Z_palm) down to the forearm comb (0, 0, Z_FORE_COMB, arriving along -Z). A cubic Hermite whose
    two tangent lengths are chosen to maximise the smallest bend radius (a free sheath bundle settles
    near its least-bent shape). Returns pts (n, 3), frames (n, 3, 3) and the tangent lengths used."""
    Rp = R_palm(flex, dev)
    p0 = Rp @ np.array([0, 0, Z_PALM_CLAMP])
    d0 = Rp @ np.array([0, 0, -1.0])
    p1 = np.array([0, 0, Z_FORE_COMB])
    d1 = np.array([0, 0, -1.0])
    span = np.linalg.norm(p1 - p0)

    def rmin(kk):
        pts = _hermite(p0, kk[0] * span * d0, p1, kk[1] * span * d1, n)
        return _radii(pts).min()

    if k is None:
        if abs(flex) + abs(dev) < 1e-6:
            k = (1.0, 1.0)
        else:
            from scipy.optimize import minimize

            res = minimize(lambda kk: -rmin(kk), x0=[1.0, 1.0], method="Nelder-Mead",
                           bounds=[(0.3, 2.5), (0.3, 2.5)], options=dict(xatol=1e-3, fatol=1e-3))
            k = tuple(res.x)
    pts = _hermite(p0, k[0] * span * d0, p1, k[1] * span * d1, n)
    # section frames: rotation-minimising frame (double reflection, Wang et al. 2008) started at the
    # palm clamp with the palm's orientation, then the leftover twist at the comb (whose orientation
    # is fixed) is spread linearly along the bundle, as a real bundle twists evenly.
    tan = np.gradient(pts, axis=0)
    tan /= np.linalg.norm(tan, axis=1)[:, None]
    t = -tan  # frame z points "up" toward the palm
    r = [Rp[:, 0]]
    for i in range(n - 1):
        v1 = pts[i + 1] - pts[i]
        c1 = v1 @ v1
        rL = r[-1] - (2 / c1) * (v1 @ r[-1]) * v1
        tL = t[i] - (2 / c1) * (v1 @ t[i]) * v1
        v2 = t[i + 1] - tL
        c2 = v2 @ v2
        rn = rL - (2 / c2) * (v2 @ rL) * v2 if c2 > 1e-12 else rL
        r.append(rn / np.linalg.norm(rn))
    r = np.array(r)
    # twist needed at the comb end to match the forearm frame x axis
    xe = r[-1]
    ang = math.atan2(np.cross(xe, np.array([1.0, 0, 0])) @ t[-1], xe @ np.array([1.0, 0, 0]))
    frames = []
    for i, s in enumerate(np.linspace(0, 1, n)):
        a = ang * s
        x = r[i] * math.cos(a) + np.cross(t[i], r[i]) * math.sin(a)
        y = np.cross(t[i], x)
        frames.append(np.c_[x, y, t[i]])
    return pts, np.array(frames), k


def tube_paths(flex, dev, n=121):
    pts, fr, _ = bundle_centreline(flex, dev, n)
    offs = tube_offsets()
    paths = pts[None, :, :] + np.einsum("nij,kj->kni", fr, np.c_[offs, np.zeros(len(offs))])
    return paths  # (32, n, 3)


def tube_stats(flex, dev):
    paths = tube_paths(flex, dev)
    L = np.linalg.norm(np.diff(paths, axis=1), axis=2).sum(axis=1)
    r = np.array([_radii(p).min() for p in paths])
    return L, r


def bore_needed(flex, dev, n=241):
    """Max |x|, |y| (ring frame) of the bundle envelope inside the ring's height."""
    pts, fr, _ = bundle_centreline(flex, dev, n)
    corners = np.array([(a, b, 0) for a, b in tube_offsets()])
    corners = np.vstack([corners + (dx, dy, 0) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))])
    env = pts[:, None, :] + np.einsum("nij,kj->nki", fr, corners)
    env = env.reshape(-1, 3) @ R_ring(flex, dev)  # into the ring frame (row vectors: R^T applied)
    # ring height differs at the X ends and Y ends; use the Y-end height for y, X-end height for x
    my = env[np.abs(env[:, 2]) <= RING_H_Y / 2]
    mx = env[np.abs(env[:, 2]) <= RING_H_X / 2]
    return float(np.abs(mx[:, 0]).max()), float(np.abs(my[:, 1]).max())


def wrist_grid(step_flex=(-60, -30, 0, 35, 70), step_dev=(-30, -15, 0, 20)):
    return [(f, d) for f in step_flex for d in step_dev]


# --- push-rod drive (2-RSS) ----------------------------------------------------------------------
# Right drive (thumb side, +X); the left one is its mirror in x. The crank turns about an axis along
# `axis` ("X" or "Y") through A; crank pin C = A + r_c (cos phi e1 + sin phi e2) with e1 = +Z and
# e2 = +Y (axis X) or e2 = +X (axis Y, right side; mirrored on the left). The rod length is fixed by
# the crank angle phi_mid at the mid pose Q_MID. G = servo turns per crank turn (2:1 spur stage).
Q_MID = (5.0, -5.0)
DRIVE = dict(axis="X", xB=24.0, yB=16.0, zB=-1.4, xA=24.0, yA=-12.0, zA=-48.0, r_c=20.0,
             phi_mid=90.0, G=2.0)
STS3215_STALL = 2.94  # N m at 12 V (30 kg cm class, verify datasheet)
STS3215_CONT = 0.9  # N m, ~30 % of stall, a sustainable holding torque (rule of thumb)


def drive_points(dr, side):
    s = 1 if side == "R" else -1
    B = np.array([s * dr["xB"], dr["yB"], dr["zB"]])
    A = np.array([s * dr["xA"], dr["yA"], dr["zA"]])
    return B, A


def _basis(dr, side):
    e1 = np.array([0, 0, 1.0])
    if dr["axis"] == "X":
        e2 = np.array([0, 1.0, 0])
    else:
        e2 = np.array([1.0 if side == "R" else -1.0, 0, 0])
    return e1, e2


def crank_pin(dr, side, phi):
    _, A = drive_points(dr, side)
    e1, e2 = _basis(dr, side)
    return A + dr["r_c"] * (math.cos(phi) * e1 + math.sin(phi) * e2)


def rod_length(dr, side="R"):
    B0, _ = drive_points(dr, side)
    B = R_palm(*Q_MID) @ B0
    return float(np.linalg.norm(B - crank_pin(dr, side, dr["phi_mid"] * DEG)))


def solve_crank(dr, side, flex, dev, phi_ref=None):
    """Crank angle phi (rad) that keeps the rod length (branch nearest phi_ref); None if
    unreachable. Also returns sin of the transmission angle (crank vs rod in the crank plane)."""
    B0, A = drive_points(dr, side)
    B = R_palm(flex, dev) @ B0
    L = rod_length(dr, side)
    r = dr["r_c"]
    e1, e2 = _basis(dr, side)
    d = B - A
    u, v = d @ e1, d @ e2
    K = (d @ d + r * r - L * L) / (2 * r)  # u cos + v sin = K
    rho = math.hypot(u, v)
    if rho < 1e-9 or abs(K / rho) > 1:
        return None, 0.0
    psi = math.atan2(v, u)
    a = math.acos(K / rho)
    ref = dr["phi_mid"] * DEG if phi_ref is None else phi_ref
    phi = min((psi + a, psi - a), key=lambda p: abs(math.remainder(p - ref, 2 * math.pi)))
    phi = ref + math.remainder(phi - ref, 2 * math.pi)
    # transmission: angle between the crank tangent and the rod
    C = A + r * (math.cos(phi) * e1 + math.sin(phi) * e2)
    t = -math.sin(phi) * e1 + math.cos(phi) * e2
    rod = (B - C) / np.linalg.norm(B - C)
    return phi, abs(float(t @ rod))


def drive_jacobians(dr, flex, dev, eps=1e-4):
    """A[i, j] = u_i . dB_i/dq_j (mm/rad), b[i] = u_i . dC_i/dphi_i (mm/rad); i = R, L."""
    A = np.zeros((2, 2))
    b = np.zeros(2)
    info = {}
    for i, side in enumerate(("R", "L")):
        B0, _ = drive_points(dr, side)
        phi, st = solve_crank(dr, side, flex, dev)
        if phi is None:
            return None
        B = R_palm(flex, dev) @ B0
        C = crank_pin(dr, side, phi)
        u = (B - C) / np.linalg.norm(B - C)
        dBf = (R_palm(flex + eps / DEG, dev) @ B0 - R_palm(flex - eps / DEG, dev) @ B0) / (2 * eps)
        dBd = (R_palm(flex, dev + eps / DEG) @ B0 - R_palm(flex, dev - eps / DEG) @ B0) / (2 * eps)
        A[i] = (u @ dBf, u @ dBd)
        e1, e2 = _basis(dr, side)
        b[i] = u @ (dr["r_c"] * (-math.sin(phi) * e1 + math.cos(phi) * e2))
        info[side] = dict(phi=phi, sin_t=st, B=B, C=C)
    return A, b, info


def capacity(dr, flex, dev, tau_servo=STS3215_STALL):
    """Largest pure flex and pure dev torque (N m) at this pose with each servo limited to tau_servo.
    Rod force f_i (N, + = push on the palm): wrist torque = A^T f (N mm); crank torque = b_i f_i;
    servo torque = crank torque / G."""
    J = drive_jacobians(dr, flex, dev)
    if J is None:
        return None
    A, b, info = J
    if abs(np.linalg.det(A)) < 1e-6:
        return None
    AiT = np.linalg.inv(A.T)
    out = {}
    for name, tw in (("flex", np.array([1.0, 0])), ("dev", np.array([0, 1.0]))):
        ts = np.abs(b * (AiT @ tw)) / dr["G"]
        out[name] = tau_servo / ts.max()
    out["rod_force_per_Nm"] = float(max(np.abs(AiT @ np.array([1000.0, 0])).max(),
                                        np.abs(AiT @ np.array([0, 1000.0])).max()))
    out["sin_t"] = min(info["R"]["sin_t"], info["L"]["sin_t"])
    out["phi_deg"] = (info["R"]["phi"] / DEG, info["L"]["phi"] / DEG)
    out["detA"] = float(np.linalg.det(A))
    out["info"] = info
    return out


def drive_sweep(dr, n_f=27, n_d=11, margin=2.0):
    fl = np.linspace(P.WRIST_RANGES["wrist_flex"][0] - margin, P.WRIST_RANGES["wrist_flex"][1] + margin, n_f)
    dv = np.linspace(P.WRIST_RANGES["wrist_dev"][0] - margin, P.WRIST_RANGES["wrist_dev"][1] + margin, n_d)
    return [(f, d, capacity(dr, f, d)) for f in fl for d in dv]


def drive_summary(dr, verbose=True, n_f=27, n_d=11):
    rows = drive_sweep(dr, n_f, n_d)
    good = [r for r in rows if r[2] is not None]
    if not good:
        return dict(ok=False, unreachable=len(rows))
    phis = np.array([r[2]["phi_deg"] for r in good])
    cf = np.array([r[2]["flex"] for r in good])
    cd = np.array([r[2]["dev"] for r in good])
    s = dict(ok=len(good) == len(rows), unreachable=len(rows) - len(good),
             min_flex_Nm_stall=float(cf.min()), median_flex_Nm_stall=float(np.median(cf)),
             min_dev_Nm_stall=float(cd.min()), median_dev_Nm_stall=float(np.median(cd)),
             min_flex_Nm_cont=float(cf.min() * STS3215_CONT / STS3215_STALL),
             min_dev_Nm_cont=float(cd.min() * STS3215_CONT / STS3215_STALL),
             min_transmission_deg=float(math.degrees(math.asin(min(1, min(r[2]["sin_t"] for r in good))))),
             crank_range_deg=(float(phis.min()), float(phis.max())),
             servo_range_deg=(float((phis.min() - dr["phi_mid"]) * dr["G"]),
                              float((phis.max() - dr["phi_mid"]) * dr["G"])),
             max_rod_force_N_per_Nm=float(max(r[2]["rod_force_per_Nm"] for r in good)),
             rod_length=rod_length(dr), sign_detA=sorted({int(np.sign(r[2]["detA"])) for r in good}))
    if verbose:
        for k, v in s.items():
            print(f"  {k}: {v if not isinstance(v, float) else round(v, 3)}")
    return s


def packaging_penalty(dr):
    """Keep cranks inside the forearm neck (|x| <= 43, |y| <= 30), clear of the bundle (|x| < 12) and
    below the wrist base plate (z < -20)."""
    pen = 0.0
    xA, yA, zA, r = abs(dr["xA"]), dr["yA"], dr["zA"], dr["r_c"]
    if dr["axis"] == "X":  # crank disc in the y-z plane at x = xA
        pen += max(0, abs(yA) + r - 29) + max(0, zA + r + 20) + max(0, 14 - xA) + max(0, xA - 40)
    else:  # crank disc in the x-z plane at y = yA
        # the crank plane is behind the bundle (y > 20), so it only has to stay inside the shell and
        # clear of the other side's crank
        pen += max(0, xA + r - 42) + max(0, 2 - (xA - r)) + max(0, zA + r + 20) + max(0, abs(yA) - 29.5)
        pen += max(0, 26 - yA)
    # palm-side ball (radius 4): outside the palm lugs (x >= 24), or outside the ring and forearm
    # lugs radially (r_x >= 28 from the flex axis) with x >= 11 (clear of the forearm lugs)
    rx = math.hypot(dr["yB"], dr["zB"])
    if dr["xB"] < 24:
        pen += max(0, 28 - rx) + max(0, 11 - dr["xB"])
    return pen


def optimize_drive(axis="X", seed=0, maxiter=60):
    from scipy.optimize import differential_evolution

    keys = ["xB", "yB", "zB", "xA", "yA", "zA", "r_c", "phi_mid", "G"]
    if axis == "X":
        bounds = [(20, 30), (8, 22), (-8, 8), (20, 34), (-25, 25), (-80, -30), (10, 26), (-180, 180),
                  (2.0, 2.0)]
    else:
        bounds = [(24, 30), (6, 30), (-10, 10), (14, 36), (26, 29), (-85, -40), (8, 26), (-180, 180),
                  (1.5, 3.0)]

    def unpack(x):
        d = dict(DRIVE, axis=axis)
        d.update({k: float(v) for k, v in zip(keys, x)})
        return d

    def cost(x):
        d = unpack(x)
        rows = drive_sweep(d, 12, 6)
        good = [r[2] for r in rows if r[2] is not None]
        if len(good) < len(rows):
            return 100 + (len(rows) - len(good))
        cf = min(g["flex"] for g in good)
        cd = min(g["dev"] for g in good)
        st = min(g["sin_t"] for g in good)
        phis = np.array([g["phi_deg"] for g in good])
        swing = np.abs(phis - d["phi_mid"]).max()
        pen = 20 * max(0, 0.5 - st) + 0.1 * max(0, swing * d["G"] - 150) + 2 * packaging_penalty(d)
        if len({int(np.sign(g["detA"])) for g in good}) > 1:
            pen += 50
        return -min(cf, 1.3 * cd) + pen

    res = differential_evolution(cost, bounds, seed=seed, maxiter=maxiter, popsize=20, tol=1e-6,
                                 polish=False)
    return unpack(res.x), res.fun


def main():
    print("== bundle: bore needed (ring frame half-widths x, y), min bend radius, tube dL ==")
    L0, _ = tube_stats(0, 0)
    worst_r, dmin, dmax = 1e9, np.zeros(32), np.zeros(32)
    bx = by = 0
    for f, d in wrist_grid():
        hx, hy = bore_needed(f, d)
        bx, by = max(bx, hx), max(by, hy)
        L, r = tube_stats(f, d)
        dL = L - L0
        dmin, dmax = np.minimum(dmin, dL), np.maximum(dmax, dL)
        worst_r = min(worst_r, r.min())
        print(f"  flex {f:4.0f} dev {d:4.0f}: half x {hx:5.1f} y {hy:5.1f}  r_min {r.min():5.1f}  "
              f"dL {dL.min():+5.1f}..{dL.max():+5.1f}")
    print(f"  bore needed: {2 * bx:.1f} x {2 * by:.1f} mm; worst tube bend radius {worst_r:.1f} mm")
    print(f"  tube dL over the range: {dmin.min():+.1f} .. {dmax.max():+.1f} mm")
    print("== drive ==")
    drive_summary(DRIVE)


if __name__ == "__main__":
    if "--opt" in sys.argv:
        for ax in sys.argv[2:] or ("X", "Y"):
            d, c = optimize_drive(ax)
            print(ax, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in d.items()}, round(c, 3))
            drive_summary(d)
    else:
        main()
