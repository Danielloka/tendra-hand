"""DIP-PIP coupling linkage for the Tendra Hand V1 fingers (owner's choice, 2026-10-01).

Each finger's DIP has no servo. One rigid bar couples it to the PIP: pin A is fixed to the
proximal phalanx near the PIP axis (back side), pin B to the distal phalanx near the DIP axis (palm
side), so the bar runs diagonally along the middle phalanx. Bending the PIP swings pin A back, the
bar pushes pin B, and the DIP bends with it. Nothing to tension, no creep.

The linkage sits in a thin layer outside the finger's side face (LAYOUT, owner's choice 2026-10-02):
the PIP and DIP carry an 8 mm bearing in each side wall of the parent part, so there is no room for
pins near the axes inside the finger. Plate A is glued onto the proximal phalanx's side face (it also
holds the PIP bearing in), plate B onto the distal phalanx's side face (relieved over the middle
phalanx's cheek), each with a 1.5 mm steel pin; the bar runs on the pins outside the plates.

This script finds, per finger, the pin positions that keep DIP = RATIO x PIP over the whole PIP
range, under these rules (mm, in the middle phalanx's plane: x from the PIP axis toward the DIP
axis, y toward the back of the finger):
- pins A and B between PIN_R_MIN and PIN_R_MAX from their axis (pin A clears the recess over the
  PIP bearing's inner race, pin B stays on its plate);
- the bar stays within the phalanx's side outline (|y| <= HALF_HEIGHT) and keeps AXIS_CLEAR from
  both axes;
- the angles between the bar and each crank stay in TRANSMISSION, so it never locks up near a
  straight line (a "toggle" position) and pushes the fingertip well at every angle.

Angles: gamma_a is pin A's direction from the PIP axis and gamma_b pin B's from the DIP axis,
measured from +x toward +y (the back of the finger), at q = 0 (finger straight).

The result goes into tendon_routes.json ("coupling") through tendon_router.py: the MuJoCo model
uses the fitted DIP(PIP) curve, and the Fusion script (stage 'linkage') builds the plates and the
bar from it.

    uv run python hardware/cad/dip_linkage.py      # prints the design per finger
"""

from __future__ import annotations

import functools
import itertools
import json
import math
from pathlib import Path

import numpy as np

SPEC = Path(__file__).resolve().parents[1] / "robot_description" / "v1_export" / "hand_v1.json"
FINGERS = ("index", "middle", "ring", "little")

RATIO = 0.75  # DIP angle per PIP angle (human-like)
PIP_RANGE_DEG = (-5.0, 95.0)  # same as config_v1.h
PIN_R_MIN, PIN_R_MAX = 3.5, 5.5  # pin centre from its joint axis (pin A: 0.75 + 2.75 recess)
HALF_HEIGHT = 5.5  # the middle phalanx is ~14 mm tall; keep the bar's centreline within +-5.5
AXIS_CLEAR = 2.0  # bar centreline to either joint axis
TRANSMISSION = (30.0, 150.0)  # bar-to-crank angles (deg), away from toggle
POLY_DEGREE = 4  # MuJoCo's joint equality takes a quartic

# Outside layer, on the finger's -x side (toward the little finger), mm. x is measured outward from
# the side face of the phalanges (index: x = -14.03).
LAYOUT = {
    "side": "-x",
    "plate_t": 1.5,  # plates A and B
    "gap": 0.2,  # plate to bar
    "bar_t": 1.2,
    "bar_w": 3.2,
    "pin_d": 1.5,  # steel pins; press fit in the plates, running fit (1.6 holes) in the bar
    "bearing_recess_r": 2.75,  # plate A clears the PIP bearing's inner race and the axle stub
    "relief_r": 8.8,  # plate B clears the middle phalanx's DIP cheek (r 7.5 .. 8.5)
}
BAR_PLANE_MM = LAYOUT["plate_t"] + LAYOUT["gap"] + LAYOUT["bar_t"] / 2  # bar centre, out of the face


def middle_lengths(spec_path: Path = SPEC) -> dict[str, float]:
    """PIP axis to DIP axis distance (mm) per finger, from the Fusion export."""
    joints = {j["name"]: j for j in json.loads(spec_path.read_text(encoding="utf-8"))["joints"]}
    out = {}
    for f in FINGERS:
        p, d = (np.array(joints[f"{f}_{k}"]["point_mm"], dtype=float) for k in ("pip", "dip"))
        out[f] = float(np.hypot(*(d - p)[1:]))  # the axes run along x, so measure in y-z
    return out


def dip_curve(length, a, gamma_a, b, gamma_b, pip):
    """DIP angles (rad) for PIP angles `pip` (rad), or None if the bar can't follow.

    Pin A turns with the proximal phalanx: +pip in the middle phalanx's frame. Pin B turns with
    the distal phalanx: -dip. The bar keeps its length (circle-circle intersection)."""
    ax, ay = a * np.cos(gamma_a + pip), a * np.sin(gamma_a + pip)
    bx0, by0 = length + b * math.cos(gamma_b), b * math.sin(gamma_b)
    bar = math.hypot(a * math.cos(gamma_a) - bx0, a * math.sin(gamma_a) - by0)
    rx, ry = ax - length, ay  # pin A seen from the DIP axis
    r = np.hypot(rx, ry)
    k = (r * r + b * b - bar * bar) / (2 * b * r)
    if np.any(np.abs(k) > 1):
        return None
    delta = np.arctan2(ry, rx)
    # two solutions psi = delta +- acos(k): keep the one that starts at gamma_b
    sols = [delta + s * np.arccos(k) for s in (1, -1)]
    i0 = int(np.argmin(np.abs(pip)))
    psi = min(sols, key=lambda p: abs(math.remainder(p[i0] - gamma_b, 2 * math.pi)))
    dip = np.unwrap(gamma_b - psi)
    dip -= 2 * math.pi * round(dip[i0] / (2 * math.pi))  # angles come back modulo a full turn
    return dip if abs(dip[i0]) < 1e-6 else None


def check(length, a, gamma_a, b, gamma_b, pip, dip) -> dict | None:
    """Clearances of a design over the motion, or None if it breaks a rule."""
    ax, ay = a * np.cos(gamma_a + pip), a * np.sin(gamma_a + pip)
    bx, by = length + b * np.cos(gamma_b - dip), b * np.sin(gamma_b - dip)
    if max(np.abs(ay).max(), np.abs(by).max()) > HALF_HEIGHT:
        return None
    seg = np.stack([bx - ax, by - ay], axis=1)
    clear = math.inf
    for cx in (0.0, length):  # distance from each axis to the bar
        t = np.clip(((cx - ax) * seg[:, 0] - ay * seg[:, 1]) / (seg**2).sum(1), 0, 1)
        clear = min(clear, np.hypot(ax + t * seg[:, 0] - cx, ay + t * seg[:, 1]).min())
    if clear < AXIS_CLEAR:
        return None
    angles = []
    for (px, py), (cx, cy) in (((ax, ay), (0.0, 0.0)), ((bx, by), (length, 0.0))):
        crank = np.stack([px - cx, py - cy], axis=1)
        cos = (crank * seg).sum(1) / (np.linalg.norm(crank, axis=1) * np.linalg.norm(seg, axis=1))
        angles.append(np.degrees(np.arccos(np.clip(np.abs(cos), -1, 1))))
    worst = min(a.min() for a in angles)  # 90 deg is ideal, 0 is toggle
    if worst < 90 - (TRANSMISSION[1] - TRANSMISSION[0]) / 2:
        return None
    return {"axis_clear_mm": float(clear), "min_transmission_deg": float(worst)}


def design(length: float) -> dict:
    """Best linkage for a middle phalanx of `length` mm (coarse grid, then a finer one)."""
    pip = np.radians(np.linspace(*PIP_RANGE_DEG, 41))
    target = RATIO * pip

    def score(p):
        dip = dip_curve(length, *p, pip=pip) if p[0] > 0 and p[2] > 0 else None
        if dip is None:
            return math.inf, None, None
        c = check(length, *p, pip, dip)
        if c is None:
            return math.inf, None, None
        return float(np.abs(dip - target).max()), dip, c

    best = (math.inf, None)
    for a, b in itertools.product(np.arange(PIN_R_MIN, PIN_R_MAX + 1e-9, 0.5), repeat=2):
        for ga, gb in itertools.product(range(0, 181, 15), range(-180, 1, 15)):
            p = (a, math.radians(ga), b, math.radians(gb))
            s = score(p)[0]
            if s < best[0]:
                best = (s, p)
    if best[1] is None:
        raise ValueError(f"no linkage fits a {length} mm middle phalanx")
    # local refinement: shrinking pattern search
    p, step = np.array(best[1]), np.array([0.25, math.radians(5), 0.25, math.radians(5)])
    s = best[0]
    while step[1] > math.radians(0.05):
        improved = False
        for i, sign in itertools.product(range(4), (1, -1)):
            q = p.copy()
            q[i] += sign * step[i]
            if not (PIN_R_MIN <= q[0] <= PIN_R_MAX and PIN_R_MIN <= q[2] <= PIN_R_MAX):
                continue
            sq = score(q)[0]
            if sq < s:
                p, s, improved = q, sq, True
        if not improved:
            step /= 2
    err, dip, clear = score(p)
    a, ga, b, gb = (float(v) for v in p)
    bar = math.hypot(
        a * math.cos(ga) - length - b * math.cos(gb), a * math.sin(ga) - b * math.sin(gb)
    )
    # DIP(PIP) as a quartic through 0 (MuJoCo joint equality: dip = c1 pip + ... + c4 pip^4)
    X = np.stack([pip**k for k in range(1, POLY_DEGREE + 1)], axis=1)
    c = np.linalg.lstsq(X, dip, rcond=None)[0]
    fit_err = float(np.abs(X @ c - dip).max())
    return {
        "middle_length_mm": round(length, 4),
        "pin_a": {"r_mm": round(a, 4), "angle_deg": round(math.degrees(ga), 3)},
        "pin_b": {"r_mm": round(b, 4), "angle_deg": round(math.degrees(gb), 3)},
        "bar_mm": round(bar, 4),
        "max_ratio_error_deg": round(math.degrees(err), 3),
        "polycoef": [0.0, *(round(float(v), 6) for v in c)],
        "poly_fit_error_deg": round(math.degrees(fit_err), 4),
        **{k: round(v, 3) for k, v in clear.items()},
    }


@functools.cache
def linkages(spec_path: Path = SPEC) -> dict[str, dict]:
    return {f: design(length) for f, length in middle_lengths(spec_path).items()}


def main() -> None:
    for f, d in linkages().items():
        print(
            f"{f:6s} middle {d['middle_length_mm']:5.1f} mm | pin A r {d['pin_a']['r_mm']:.2f} @ "
            f"{d['pin_a']['angle_deg']:6.1f} deg | pin B r {d['pin_b']['r_mm']:.2f} @ "
            f"{d['pin_b']['angle_deg']:6.1f} deg | bar {d['bar_mm']:.2f} mm | max error "
            f"{d['max_ratio_error_deg']:.2f} deg | axis clear {d['axis_clear_mm']:.1f} mm | "
            f"transmission >= {d['min_transmission_deg']:.0f} deg"
        )


if __name__ == "__main__":
    main()
