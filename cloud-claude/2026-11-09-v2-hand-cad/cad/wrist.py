"""Tendra V2 concept: 2-DOF hollow cardan wrist + push-rod drive (study 2026-11-09, report 03).

Hand frame (mm): origin = wrist centre, +Z to the fingers, -Y palm side, +X thumb side.
Chain (see wrist_kin.py): forearm -> wrist_dev (about +Y, OUTER: forearm lugs at +-Y) -> ring ->
wrist_flex (about +X, INNER: palm lugs at +-X) -> palm.  R_palm = Ry(dev) @ Rx(flex).

build(q) returns {"parts", "joints", "interface"}; q = {"wrist_flex": deg, "wrist_dev": deg}.

    uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy \
        --with pillow python cad/wrist.py [--quick]
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import params as P  # noqa: E402
import wrist_kin as K  # noqa: E402

OUT = HERE.parent / "out" / "wrist"

# --- dimensions ------------------------------------------------------------------------------------
PIN = P.PIN_D_BIG  # 3 mm steel dowels
BRG_OD, BRG_W = 10.0, 4.0  # 623ZZ (3 x 10 x 4)
CL = 0.5  # running clearance between moving printed parts

# ring (moves with wrist_dev): rounded-rectangle frame, flat, z +-3.5
RING_IN = (24.0, 37.0, 1.0)  # inner x, y, corner r: square corners keep the ring at r_x >= 18.5 (outside
# the flange sweep) for |x| < 12; the bundle needs +-9.5 x +-14.7 (wrist_kin, flex +-52)
RING_OUT = (30.0, 43.0, 10.0)
RING_H = 7.0
RING_XCUT = 13.5  # ring body cut back beyond this |x| (only the bearing bosses go further)
XBOSS = (11.0, 15.5, 6.6)  # x from, x to, radius: 623 housing around the flex axis
# forearm lugs (dev axis, +-Y)
FL_Y = (22.0, 27.0)  # y from/to (623 seat from the outer face)
FL_X = 6.6
# palm lugs (flex axis, +-X)
PL_X = (16.0, 19.5)
PL_Y = 6.0
PL_R = 4.5
# plates
BASE_Z = (-18.0, -15.0)  # wrist base plate (forearm side), bolts onto the forearm top
FLANGE_Z = (10.0, 15.0)  # palm flange; palm base face at z = +15
FLANGE_XY = (21.5, 12.5, 5.0)
FLANGE_OPEN = (8.0, 8.0, 3.0)  # half x, half y, r (bundle +-7.5 below the clamp)
FLANGE_HEEL_R = 18.0  # in |x| < 12.5 the flange stays inside r_x <= 18 (clears the ring ends at +-52 deg flex)
BASE_OPEN = (10.0, 12.0, 4.0)  # half x, half y, r (bundle needs +-7.5 x +-9 at the base)
CLAMP_Z = (14.0, 19.0)  # sheath clamp bridge (laminated comb), sits in a pocket of the flange
FLANGE_BOLTS = [(sx * 17.0, sy * 9.5) for sx in (-1, 1) for sy in (-1, 1)]  # M3 heat-set inserts
INSERT_D, INSERT_L = 4.0, 5.0  # M3 x 5.7 short insert hole (4.0 for PETG)
STOP_MARGIN = 2.0  # hard stops 2 deg beyond the software range
DEV_PIN_R, DEV_PIN_D = 5.6, 1.6  # dev stop: steel pin in each ring Y end, riding in an arc slot in the lug
DEV_PIN_HALF = math.degrees(math.atan(DEV_PIN_D / 2 / DEV_PIN_R))
TOOTH_HALF = 12.5  # angular half-width of the flex-stop tooth at r 7 (deg)

# drive (numbers from wrist_kin.DRIVE)
DR = K.DRIVE
SERVO_SHAFT = {"R": (26.0, 9.0, -61.6)}  # STS3215 output shaft (right; left = mirror), pointing +Y
GEAR_C = float(np.hypot(SERVO_SHAFT["R"][0] - DR["xA"], SERVO_SHAFT["R"][2] - DR["zA"]))  # 14T:36T, module 0.7 (pinion r 5, crank gear r 12.9)
STS = P.STS3215
STS_SHAFT_OFF = 11.0  # shaft to body centre along L (verify)
ROD_D, BALL_D, BALL_D_CRANK = 4.0, 8.0, 6.0  # M4 rod; 8 mm ball at the palm, 6 mm ball link at the crank


# --- helpers ---------------------------------------------------------------------------------------
def rrect(w, h, r, t, z0=0.0):
    return cq.Workplane("XY").workplane(offset=z0).rect(w, h).extrude(t).edges("|Z").fillet(r)


def cyl(p0, p1, r):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    return cq.Workplane(cq.Plane(tuple(p0), xDir=_perp(d), normal=tuple(d / L))).circle(r).extrude(L)


def _perp(d):
    d = d / np.linalg.norm(d)
    a = np.array([1.0, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 1.0, 0])
    v = np.cross(d, a)
    return tuple(v / np.linalg.norm(v))


def sphere(c, r):
    return cq.Workplane("XY").sphere(r).translate(tuple(c))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def rot_shape(wp, R):
    """Apply a rotation matrix (about the origin) to a Workplane/Shape."""
    from scipy.spatial.transform import Rotation

    rv = Rotation.from_matrix(R).as_rotvec()
    ang = float(np.linalg.norm(rv))
    if ang < 1e-9:
        return wp
    ax = tuple(rv / ang)
    return wp.rotate((0, 0, 0), ax, math.degrees(ang))


# --- forearm-side parts ----------------------------------------------------------------------------
def make_base():
    """Wrist base: plate on top of the forearm neck + two dev lugs at +-Y (623 bearings) + dev stop pads."""
    bx, by = 15.0, FL_Y[1] + 1.0
    plate = rrect(2 * bx, 2 * by, 6.0, BASE_Z[1] - BASE_Z[0], BASE_Z[0])
    # narrower (|x| <= 12) beside the bundle opening, so the palm ears clear it at extension + deviation
    for sx in (-1, 1):
        x0, x1 = sorted((sx * 12.0, sx * 20.0))
        plate = plate.cut(box(x0, x1, -12.5, 12.5, BASE_Z[0] - 1, BASE_Z[1] + 1))
    ox, oy, orr = BASE_OPEN
    plate = plate.cut(rrect(2 * ox, 2 * oy, orr, 10, BASE_Z[0] - 2))
    for sx in (-1, 1):  # 4 x M3 to the forearm neck
        for sy in (-1, 1):
            plate = plate.cut(cyl((sx * 12.5, sy * 18.0, BASE_Z[0] - 1), (sx * 12.5, sy * 18.0, BASE_Z[1] + 1), 1.6))
    for sy in (-1, 1):
        y0, y1 = (FL_Y[0], FL_Y[1]) if sy > 0 else (-FL_Y[1], -FL_Y[0])
        lug = box(-FL_X, FL_X, y0, y1, BASE_Z[1] - 0.5, 0.0)
        lug = lug.union(cyl((0, y0, 0), (0, y1, 0), FL_X))
        lug = lug.cut(cyl((0, y0 - 1, 0), (0, y1 + 1, 0), PIN / 2 + 0.2))
        yo = y1 if sy > 0 else y0
        lug = lug.cut(cyl((0, yo, 0), (0, yo - sy * BRG_W, 0), BRG_OD / 2))
        plate = plate.union(lug)
    # dev hard stops: arc slot (r 4.6..6.6, 1.5 deep) in each lug's inner face; the ring's stop pin rides in it
    dmin, dmax = K.RANGES["wrist_dev"]
    for sy in (-1, 1):
        y0, y1 = sorted((sy * FL_Y[0], sy * (FL_Y[0] + 1.5)))
        plate = plate.cut(_arc_xz(y0 - (0.01 if sy < 0 else 0), y1 + (0.01 if sy > 0 else 0), DEV_PIN_R - 1.0, DEV_PIN_R + 1.0,
                                  dmin - STOP_MARGIN - DEV_PIN_HALF, dmax + STOP_MARGIN + DEV_PIN_HALF))
    return plate


def _arc_xz(y0, y1, r0, r1, t0, t1, n=40):
    """Annular sector about the Y axis: points (x, z) = (-r sin t, -r cos t), t in t0..t1 (deg), y0..y1."""
    ts = np.radians(np.linspace(t0, t1, n))
    pts = [(-r1 * math.sin(t), -r1 * math.cos(t)) for t in ts] + [(-r0 * math.sin(t), -r0 * math.cos(t)) for t in ts[::-1]]
    wp = cq.Workplane("XZ", origin=(0, y1, 0)).polyline(pts).close().extrude(y1 - y0)
    return wp


# --- ring -------------------------------------------------------------------------------------------
def make_ring():
    ri, ro = RING_IN, RING_OUT
    ring = rrect(ro[0], ro[1], ro[2], RING_H, -RING_H / 2).cut(rrect(ri[0], ri[1], ri[2], RING_H + 2, -RING_H / 2 - 1))
    ring = ring.cut(box(RING_XCUT, 40, -40, 40, -10, 10)).cut(box(-40, -RING_XCUT, -40, 40, -10, 10))
    for sx in (-1, 1):
        x0, x1 = sx * XBOSS[0], sx * XBOSS[1]
        boss = cyl((x0, 0, 0), (x1, 0, 0), XBOSS[2])
        # flex stop tooth (points -Z), rides in the palm lug's arc wall
        boss = boss.union(box(min(sx * 13.8, x1), max(sx * 13.8, x1), -1.5, 1.5, -9.5, -5.0))
        boss = boss.cut(cyl((x1, 0, 0), (x1 - sx * BRG_W, 0, 0), BRG_OD / 2))
        boss = boss.cut(cyl((x0 - sx, 0, 0), (x1 + sx, 0, 0), PIN / 2 + 0.2))
        ring = ring.union(boss)
    # dev pin holes (press fit) in the +-Y ends, plus a local boss
    for sy in (-1, 1):
        yi, yo = sy * ri[1] / 2, sy * ro[1] / 2
        ring = ring.union(cyl((0, yi, 0), (0, yo, 0), 3.5)).union(cyl((0, yo - sy * 2.0, 0), (0, yo, 0), 6.6))
        ring = ring.cut(cyl((0, yi - sy * 0.5, 0), (0, yo + sy, 0), PIN / 2 - 0.05))
        ring = ring.union(cyl((0, yo - sy * 0.5, -DEV_PIN_R), (0, sy * (FL_Y[0] + 1.3), -DEV_PIN_R), DEV_PIN_D / 2))
    return ring


# --- palm flange -------------------------------------------------------------------------------------
def make_flange():
    fx, fy, fr = FLANGE_XY
    fl = rrect(2 * fx, 2 * fy, fr, FLANGE_Z[1] - FLANGE_Z[0], FLANGE_Z[0])
    ox, oy, orr = FLANGE_OPEN
    fl = fl.cut(rrect(2 * ox, 2 * oy, orr, 20, FLANGE_Z[0] - 5))
    # centre (|x| < 11.5): inside r_x <= 18; links (11.5..14.5): |y| <= 3.5 only, so nothing palm-side
    # sweeps into the ring's X-end walls (x 12..13.5) or its Y ends over +-52 deg of flexion
    heel = cyl((-11.5, 0, 0), (11.5, 0, 0), FLANGE_HEEL_R)
    fl = fl.cut(box(-11.5, 11.5, -30, 30, 0, 30).cut(heel))
    for sx in (-1, 1):
        x0, x1 = sorted((sx * 11.5, sx * 14.5))
        fl = fl.cut(box(x0, x1, 3.5, 30, 0, 30)).cut(box(x0, x1, -30, -3.5, 0, 30))
    for bx, by in FLANGE_BOLTS:
        fl = fl.cut(cyl((bx, by, FLANGE_Z[1] - INSERT_L), (bx, by, FLANGE_Z[1] + 1), INSERT_D / 2))
    for sx in (-1, 1):
        x0, x1 = sorted((sx * PL_X[0], sx * PL_X[1]))
        lug = box(x0, x1, -PL_Y, PL_Y, 0.0, FLANGE_Z[0] + 0.5).union(cyl((x0, 0, 0), (x1, 0, 0), PL_R))
        lug = lug.union(box(x0, x1, -PL_Y, PL_Y, 0, 0.1))
        lug = lug.cut(cyl((x0 - 1, 0, 0), (x1 + 1, 0, 0), PIN / 2 - 0.05))
        # flex stop: arc wall on the lug's inner face (r 7..10.5), leaves a free slot for the ring tooth
        # over tooth angles -72-7 .. +62+7 deg (lug frame, from -Z toward +Y)
        xi = sx * 13.8
        fmin, fmax = K.RANGES["wrist_flex"]
        wall = _arc_wall(min(xi, sx * PL_X[0]), max(xi, sx * PL_X[0]), 7.0, 10.5,
                         -fmin + STOP_MARGIN + TOOTH_HALF, 360 - (fmax + STOP_MARGIN + TOOTH_HALF))
        lug = lug.union(wall)
        fl = fl.union(lug)
        # push-rod ear (ball stud at B)
        B = np.array([sx * DR["xB"], DR["yB"], DR["zB"]])
        ex0, ex1 = sorted((sx * 19.9, sx * 21.6))
        ear = box(ex0, ex1, 3.5, 10.0, 0.0, FLANGE_Z[0] + 0.5).union(box(min(ex0, sx * 21.0), max(ex1, sx * 21.0), 3.5, 10.0, FLANGE_Z[0], FLANGE_Z[1]))
        ear = ear.union(cyl((sx * 20.5, B[1], B[2]), (B[0] - sx * 2.5, B[1], B[2]), 2.5))
        fl = fl.union(ear)
    # sheath clamp bridge: legs + laminated comb (6 plates, row joints on the tube centres)
    hx, hy = K.bundle_half()
    cw, ch = hx + 1.2, hy + 1.2
    bridge = box(-cw - 2.5, cw + 2.5, -ch, ch, CLAMP_Z[0], CLAMP_Z[1])
    bridge = bridge.cut(box(-cw - 3, cw + 3, -ch - 1, ch + 1, CLAMP_Z[1] - 0.01, 40))
    for a, b in K.tube_offsets():
        bridge = bridge.cut(cyl((a, b, CLAMP_Z[0] - 1), (a, b, CLAMP_Z[1] + 1), 1.0))  # 2.0 = squeeze on 2.0 OD
    for sx in (-1, 1):  # 2 x M2.5 clamp screws along Y through the laminations
        bridge = bridge.cut(cyl((sx * (cw + 1.25), -ch - 1, (CLAMP_Z[0] + CLAMP_Z[1]) / 2),
                                (sx * (cw + 1.25), ch + 1, (CLAMP_Z[0] + CLAMP_Z[1]) / 2), 1.35))
    # the bridge sits in a pocket of the flange (flange top z 15): cut the pocket
    fl = fl.cut(box(-cw - 2.6, cw + 2.6, -ch - 0.1, ch + 0.1, CLAMP_Z[0], 30))
    return fl, bridge


def _arc_wall(x0, x1, r0, r1, a0, a1, n=48):
    """Annular sector about the X axis between angles a0..a1 (deg from -Z toward +Y), x0..x1."""
    angs = np.radians(np.linspace(a0, a1, n))
    outer = [(r1 * math.sin(a), -r1 * math.cos(a)) for a in angs]
    inner = [(r0 * math.sin(a), -r0 * math.cos(a)) for a in angs[::-1]]
    pts = outer + inner
    wp = cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0)
    return wp


# --- drive -------------------------------------------------------------------------------------------
def drive_state(flex, dev):
    out = {}
    for side in ("R", "L"):
        phi, st = K.solve_crank(DR, side, flex, dev)
        B0, A = K.drive_points(DR, side)
        out[side] = dict(phi=phi, B=K.R_palm(flex, dev) @ B0, C=K.crank_pin(DR, side, phi), A=A, sin_t=st)
    return out


def make_drive_fixed(side):
    """STS3215 + 13T/34T gear pair + crank-axle bearing housing (forearm side, fixed)."""
    s = 1 if side == "R" else -1
    sh = np.array(SERVO_SHAFT["R"]) * (s, 1, 1)
    L, W, H = STS["L"], STS["W"], STS["H"]
    zc = sh[2] - STS_SHAFT_OFF
    servo = box(sh[0] - W / 2, sh[0] + W / 2, sh[1] - H, sh[1], zc - L / 2, zc + L / 2)
    servo = servo.union(cyl(sh, sh + (0, 1.0, 0), 3.0))
    _, A = K.drive_points(DR, side)
    big_r = GEAR_C * DR["G"] / (1 + DR["G"])
    pin_r = GEAR_C - big_r
    gears = cyl(sh + (0, 1.0, 0), sh + (0, 4.0, 0), pin_r).union(cyl((A[0], sh[1] + 1, A[2]), (A[0], sh[1] + 4, A[2]), big_r))
    gears = gears.union(cyl((A[0], sh[1] + 1, A[2]), (A[0], A[1] - 1.5, A[2]), 2.5))  # 5 mm crank axle
    return servo, gears


def make_crank(side, phi):
    _, A = K.drive_points(DR, side)
    C = K.crank_pin(DR, side, phi)
    y0, y1 = A[1] - 1.5, A[1] + 1.5
    arm = cyl((A[0], y0, A[2]), (A[0], y1, A[2]), 5.0).union(cyl((C[0], y0, C[2]), (C[0], y1, C[2]), 4.0))
    d = np.array([C[0] - A[0], C[2] - A[2]])
    n = np.array([-d[1], d[0]]) / np.linalg.norm(d) * 3.0
    pts = [(A[0] + n[0], A[2] + n[1]), (C[0] + n[0], C[2] + n[1]), (C[0] - n[0], C[2] - n[1]), (A[0] - n[0], A[2] - n[1])]
    arm = arm.union(cq.Workplane("XZ", origin=(0, y1, 0)).polyline(pts).close().extrude(y1 - y0))
    return arm


def make_rod(B, C):
    u = (B - C) / np.linalg.norm(B - C)
    rod = cyl(C + u * BALL_D / 2, B - u * BALL_D / 2, ROD_D / 2)
    return rod.union(sphere(B, BALL_D / 2)).union(sphere(C, BALL_D_CRANK / 2))


# --- bundle ------------------------------------------------------------------------------------------
def make_bundle(flex, dev, n=41):
    """The 32-sheath bundle between the palm clamp and the forearm comb as one swept envelope
    (each tube kept in its cell; for collision checks and renders)."""
    pts, fr, _ = K.bundle_centreline(flex, dev, n)
    hx, hy = K.bundle_half()
    r = max(hx, hy) - 0.4  # round bundle 12.9 x 12.9 + tube radius
    path = cq.Workplane("XY").spline([tuple(p) for p in pts], includeCurrent=False)
    prof = cq.Workplane(cq.Plane(tuple(pts[0]), xDir=tuple(fr[0][:, 0]), normal=tuple(pts[1] - pts[0]))).circle(r)
    try:
        return prof.sweep(path, transition="round")
    except Exception:
        solid = None
        for i in range(0, n - 1, 2):
            c = cyl(pts[i], pts[min(i + 2, n - 1)], r).union(sphere(pts[i], r))
            solid = c if solid is None else solid.union(c)
        return solid


# --- assembly ------------------------------------------------------------------------------------------
_CACHE: dict = {}


def _base_parts():
    if not _CACHE:
        _CACHE["base"] = make_base()
        _CACHE["ring"] = make_ring()
        _CACHE["flange"], _CACHE["clamp"] = make_flange()
        for side in ("R", "L"):
            _CACHE[f"servo_{side}"], _CACHE[f"gears_{side}"] = make_drive_fixed(side)
        pins = {}
        for sy in (-1, 1):
            pins[f"pin_dev_{'b' if sy > 0 else 'f'}"] = cyl((0, sy * RING_IN[1] / 2, 0), (0, sy * FL_Y[1], 0), PIN / 2)
        for sx in (-1, 1):
            pins[f"pin_flex_{'r' if sx > 0 else 'l'}"] = cyl((sx * XBOSS[0], 0, 0), (sx * PL_X[1], 0, 0), PIN / 2)
        _CACHE["pins"] = pins
    return _CACHE


def build(q: dict | None = None, bundle: bool = True):
    q = q or {}
    flex = float(q.get("wrist_flex", 0.0))
    dev = float(q.get("wrist_dev", 0.0))
    c = _base_parts()
    Rr, Rp = K.R_ring(flex, dev), K.R_palm(flex, dev)
    parts = {
        "wrist_base": c["base"],
        "wrist_ring": rot_shape(c["ring"], Rr),
        "palm_flange": rot_shape(c["flange"], Rp),
        "palm_sheath_clamp": rot_shape(c["clamp"], Rp),
    }
    for n, p in c["pins"].items():
        parts[n] = rot_shape(p, Rr if "dev" in n else Rr)
    st = drive_state(flex, dev)
    for side in ("R", "L"):
        parts[f"wrist_servo_{side}"] = c[f"servo_{side}"]
        parts[f"wrist_gears_{side}"] = c[f"gears_{side}"]
        if st[side]["phi"] is not None:
            parts[f"wrist_crank_{side}"] = make_crank(side, st[side]["phi"])
            parts[f"wrist_rod_{side}"] = make_rod(st[side]["B"], st[side]["C"])
    if bundle:
        parts["sheath_bundle"] = make_bundle(flex, dev)
    return {"parts": parts, "joints": joints(), "interface": interface()}


def joints():
    return {
        "wrist_dev": {"point": (0, 0, 0), "axis": (0, 1, 0), "range_deg": K.RANGES["wrist_dev"],
                      "parent": "forearm", "child": "wrist_ring", "order": 1},
        "wrist_flex": {"point": (0, 0, 0), "axis": (1, 0, 0), "range_deg": K.RANGES["wrist_flex"],
                       "parent": "wrist_ring", "child": "palm", "order": 2,
                       "note": "axis along X of the ring (rotated by wrist_dev); R_palm = Ry(dev) @ Rx(flex)"},
    }


def interface():
    offs = K.tube_offsets()
    return {
        "palm_flange": {
            "face_z": FLANGE_Z[1],
            "bolts": [{"xy": b, "type": "M3 heat-set insert in the flange, screw from inside the palm", "depth": INSERT_L}
                      for b in FLANGE_BOLTS],
            "outline_xy": {"half_x": FLANGE_XY[0], "half_y": FLANGE_XY[1], "corner_r": FLANGE_XY[2]},
            "bundle_opening": {"half_x": FLANGE_OPEN[0], "half_y": FLANGE_OPEN[1], "corner_r": FLANGE_OPEN[2]},
            "sheath_clamp": {"z": CLAMP_Z, "keepout_box": [[-K.bundle_half()[0] - 4.7, K.bundle_half()[0] + 4.7],
                                                          [-K.bundle_half()[1] - 2.2, K.bundle_half()[1] + 2.2],
                                                          [FLANGE_Z[1], CLAMP_Z[1]]],
                             "tube_cells_xy": offs.round(3).tolist(),
                             "note": "sheaths leave the clamp top (z 21) straight up into the palm; cell index = bundle slot"},
            "sensor_cable": {"xy": (0.0, -K.bundle_half()[1] - 0.2), "d": K.CABLE_D},
            "palm_keepout": "see report 03: rounded heel (palm material must not enter the sweep of the forearm dev lugs)",
        },
        "forearm_top": {"face_z": BASE_Z[0], "bolts": [(sx * 12.5, sy * 18.0) for sx in (-1, 1) for sy in (-1, 1)],
                        "bundle_opening": {"half_x": BASE_OPEN[0], "half_y": BASE_OPEN[1]},
                        "comb_z": K.Z_FORE_COMB},
        "drive": {"servo_shaft": {s: tuple(np.array(SERVO_SHAFT["R"]) * ((1 if s == "R" else -1), 1, 1)) for s in ("R", "L")},
                  "crank_axle": {s: tuple(K.drive_points(DR, s)[1]) for s in ("R", "L")},
                  "crank_r": DR["r_c"], "gear_ratio": DR["G"], "rod_length": K.rod_length(DR),
                  "servo_box": {"L": STS["L"], "W": STS["W"], "H": STS["H"]}},
    }


# --- checks ------------------------------------------------------------------------------------------
def palm_proxy(notch=True):
    """Rough palm + thumb-base volume for collision checks (the integrator owns the real palm).
    notch=True applies the keep-out the wrist needs (report 03): for z < 36 and |x| < 9.5 the palm stays
    inside r_x <= 18 (from the flex axis) or inside a +-20 deg wedge about +Z; for z < 22: inside r_x <= 18 for |x| < 11.5 and
    within |y| <= 3.5 for 11.5 < |x| < 14.5 (same rule as the flange)."""
    palm = box(-47, 30, P.PALM_Y[0], P.PALM_Y[1], P.PALM_Z_BASE, 60)
    if notch:
        heel = cyl((-9.5, 0, 0), (9.5, 0, 0), 18.0)
        t = math.tan(math.radians(20))
        cone = cq.Workplane("YZ", origin=(-9.5, 0, 0)).polyline([(0, 0), (60 * t, 60), (-60 * t, 60)]).close().extrude(19)
        cutter = box(-9.5, 9.5, -30, 30, P.PALM_Z_BASE - 1, 36).cut(heel).cut(cone)
        palm = palm.cut(cutter)
        # small chamfers for the ring ends (|x| < 14.5): r_x <= 18 up to z = 18.5
        heel2 = cyl((-11.5, 0, 0), (11.5, 0, 0), 18.0)  # ring ends and X-end walls
        palm = palm.cut(box(-11.5, 11.5, -30, 30, P.PALM_Z_BASE - 1, 22.0).cut(heel2))
        for sx in (-1, 1):
            x0, x1 = sorted((sx * 11.5, sx * 14.5))
            palm = palm.cut(box(x0, x1, 3.5, 30, P.PALM_Z_BASE - 1, 22.0)).cut(box(x0, x1, -30, -3.5, P.PALM_Z_BASE - 1, 22.0))
    return palm


def check_collisions(quick=False, notch=True):
    from tools.checks import interference

    fl = K.RANGES["wrist_flex"]
    dv = K.RANGES["wrist_dev"]
    fvals = [fl[0], 0.0, fl[1]] if quick else [fl[0], -25.0, 0.0, 25.0, fl[1]]
    dvals = [dv[0], 0.0, dv[1]] if quick else [dv[0], -15.0, 0.0, dv[1]]
    pp = palm_proxy(notch)
    ignore = [("pin_dev_*", "wrist_base"), ("pin_dev_*", "wrist_ring"), ("pin_flex_*", "wrist_ring"),
              ("pin_flex_*", "palm_flange"), ("wrist_servo_*", "wrist_gears_*"), ("wrist_gears_*", "wrist_crank_*"),
              ("wrist_rod_*", "wrist_crank_*"), ("wrist_rod_*", "palm_flange"), ("palm_flange", "palm_sheath_clamp"),
              ("palm_proxy", "palm_flange"), ("palm_proxy", "palm_sheath_clamp"), ("sheath_bundle", "palm_sheath_clamp")]
    res = {}
    for f in fvals:
        for d in dvals:
            b = build({"wrist_flex": f, "wrist_dev": d})
            parts = dict(b["parts"])
            parts["palm_proxy"] = rot_shape(pp, K.R_palm(f, d))
            r = interference(parts, threshold=0.5, ignore=ignore)
            res[(f, d)] = r
            print(f"  flex {f:5.0f} dev {d:5.0f}: " + ("clear" if not r else "; ".join(f"{a} x {b_}={v}" for a, b_, v in r)))
    return res


def check_stops():
    """Beyond the hard-stop angle the stop pair must touch (overlap > 0); inside the range it must not."""
    from tools.checks import interference

    out = {}
    for name, q in (("flex+", {"wrist_flex": K.RANGES["wrist_flex"][1] + STOP_MARGIN + 6}),
                    ("flex-", {"wrist_flex": K.RANGES["wrist_flex"][0] - STOP_MARGIN - 6}),
                    ("dev+", {"wrist_dev": K.RANGES["wrist_dev"][1] + STOP_MARGIN + 6}),
                    ("dev-", {"wrist_dev": K.RANGES["wrist_dev"][0] - STOP_MARGIN - 6})):
        b = build(q, bundle=False)["parts"]
        pair = {"wrist_ring": b["wrist_ring"], ("palm_flange" if "flex" in name else "wrist_base"):
                b["palm_flange" if "flex" in name else "wrist_base"]}
        r = interference(pair, threshold=0.001)
        out[name] = r[0][2] if r else 0.0
        print(f"  stop {name}: overlap {out[name]:.2f} mm^3 beyond the stop ({'engages' if out[name] > 0 else 'NO CONTACT'})")
    return out


def bundle_report():
    L0, _ = K.tube_stats(0, 0)
    rmin, lo, hi = 1e9, np.zeros(32), np.zeros(32)
    worst = None
    for f in (K.RANGES["wrist_flex"][0], -30, 0, 35, K.RANGES["wrist_flex"][1]):
        for d in (K.RANGES["wrist_dev"][0], -15, 0, K.RANGES["wrist_dev"][1]):
            L, r = K.tube_stats(f, d)
            lo, hi = np.minimum(lo, L - L0), np.maximum(hi, L - L0)
            if r.min() < rmin:
                rmin, worst = r.min(), (f, d, int(r.argmin()))
            pts, _, _ = K.bundle_centreline(f, d)
    cr = min(K._radii(K.bundle_centreline(f, d)[0]).min() for f in (-60, 70) for d in (-30, 0, 20))
    return dict(tube_min_bend_radius=float(rmin), worst_pose=worst, centreline_min_radius=float(cr),
                tube_dL_min=lo.round(2).tolist(), tube_dL_max=hi.round(2).tolist(),
                free_span_q0=float(L0.mean()))


def main():
    quick = "--quick" in sys.argv
    OUT.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    b = build({})
    print(f"build q=0: {len(b['parts'])} parts in {time.time() - t0:.1f} s")
    from tools.checks import is_valid

    for n, p in b["parts"].items():
        v = is_valid(p)
        if not (v if isinstance(v, bool) else v.get("valid", True)):
            print("  INVALID", n, v)
    print("== drive ==")
    ds = K.drive_summary(DR)
    print("== bundle ==")
    br = bundle_report()
    print({k: v for k, v in br.items() if "dL" not in k})
    print(f"  tube dL range {min(br['tube_dL_min']):+.1f} .. {max(br['tube_dL_max']):+.1f} mm")
    print("== stops ==")
    stops = check_stops()
    print("== collisions (with notched palm proxy) ==")
    col = check_collisions(quick)
    print("== collisions (plain 28 mm palm, for the report) ==")
    col_plain = check_collisions(True, notch=False)
    summary = dict(drive=ds, bundle=br, stops=stops,
                   collisions={f"{k[0]:.0f},{k[1]:.0f}": v for k, v in col.items()},
                   collisions_plain_palm={f"{k[0]:.0f},{k[1]:.0f}": v for k, v in col_plain.items()},
                   interface=b["interface"], joints=b["joints"])
    (OUT / "wrist_summary.json").write_text(json.dumps(summary, indent=1, default=str))
    if not quick:
        from tools.export import export_all

        export_all({k: v for k, v in b["parts"].items()}, OUT, "wrist")
    print(f"done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
