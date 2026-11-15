"""08 bench-test kit: base plate + printable rigs (CadQuery).

Run:  uv run --no-project --with cadquery --with numpy --with trimesh --with matplotlib python cad/08_rigs.py
Writes STL + STEP per part and a PNG per rig to out/08/.  Units mm.  Plate top surface = z 0.

Conventions
- Base plate 150 x 100 x 5 (x 0..150, y 0..100, z -5..0), M3 clearance holes on a 10 mm grid at x=10..140, y=10..90,
  with an M3 nut pocket on the underside of every hole.
- Every post/block has its mounting holes at multiples of 10 mm from its own origin, so it lands on the grid.
- Standard line height above the plate top: 15 mm (Z_LINE).  Servo rigs: 33 mm (Z_SERVO).
"""
import json
import math
from pathlib import Path

import cadquery as cq
import numpy as np
import trimesh

OUT = Path(__file__).resolve().parent.parent / "out" / "08"
OUT.mkdir(parents=True, exist_ok=True)

Z_LINE = 15.0
Z_SERVO = 33.0
Z_EYE = 43.0            # eye-plate rig: line height (servo cradle on 32 mm pillars)
# Agent 07 round-2 fuse cup (out/12/budget.txt section 5, resized to a real crimp): 2 x 2 mm crimp tube (ID ~1.4, OD 2.0, L 2.0)
# on a doubled PE#8; oval chamber 2.9 x 2.1 x 2.3 deep (bead footprint 2.6 x 1.1); lip t 0.6 flat (SCS, trip ~67 N) or 0.8
# on its side (HLS, ~148 N).  Outer oval is my choice: chamber + 0.7 mm walls.
CUP_CHAMBER = (2.9, 2.1, 2.3)
CUP_OUTER = (4.3, 3.5)
CUP_LIP_HOLE = 0.8
# Agent 07 drum ring: 14 x 0.5 brass tube (ID 13), 3.0 wide here; flange lips 0.5 mm.
RING_OD, RING_ID, RING_W = 14.0, 13.0, 3.0
SCS = {"L": 23.3, "W": 12.1, "H": 25.25, "tab_z": 16.8, "hole_spacing": 28.5}  # from 2026-11-09 params.py (verify)


def box(x, y, z, cx=0, cy=0, z0=0):
    """Box with size (x,y,z), centred on (cx,cy), bottom at z0."""
    return cq.Workplane("XY").box(x, y, z, centered=(True, True, False)).translate((cx, cy, z0))


def cyl_y(d, y0, y1, cx=0, cz=0):
    c = cq.Workplane("XY").circle(d / 2).extrude(y1 - y0).rotate((0, 0, 0), (1, 0, 0), -90)
    return c.translate((cx, y0, cz))


def cyl_z(d, h, cx=0, cy=0, z0=0):
    return cq.Workplane("XY").circle(d / 2).extrude(h).translate((cx, cy, z0))


def tear(axis, d, length, c=(0, 0, 0), teardrop=True):
    """Horizontal hole cutter along axis 'x' or 'y' with a teardrop roof (prints without supports)."""
    s = cq.Workplane("YZ").circle(d / 2).extrude(length / 2, both=True)
    if teardrop:
        r = d / 2
        k = r * math.sqrt(2) / 2
        tri = cq.Workplane("YZ").polyline([(-k, k), (k, k), (0, r * math.sqrt(2))]).close().extrude(length / 2, both=True)
        s = s.union(tri)
    if axis == "y":
        s = s.rotate((0, 0, 0), (0, 0, 1), 90)
    return s.translate(c)


def hole_pts(solid, pts, d=3.4):
    for (x, y) in pts:
        solid = solid.cut(cyl_z(d, 100, x, y, -20))
    return solid


def hexpocket(af, depth, x, y, z0):
    dia = af / math.cos(math.radians(30))
    return cq.Workplane("XY").polygon(6, dia).extrude(depth).translate((x, y, z0))


# ----------------------------------------------------------------------------------------------------
def base_plate():
    p = box(150, 100, 5, 75, 50, -5)
    for x in range(10, 141, 10):
        for y in range(10, 91, 10):
            p = p.cut(cyl_z(3.4, 6, x, y, -5.5)).cut(hexpocket(5.8, 2.4, x, y, -5.0))
    return p


def foot(wx, wy, holes_y, t=4.0):
    f = box(wx, wy, t)
    return hole_pts(f, [(0, y) for y in holes_y])


def roller_post(h_axis=Z_LINE, ox=14.0):
    """U-bracket for a 3 mm dowel (or an MR63 bearing on it) along y.  The dowel sits ox mm in front of the mounting
    holes, so with the foot on the plate's edge row the line falls clear of the plate.  Print as is (foot on the bed)."""
    s = box(32, 48, 5, 6, 0)
    s = hole_pts(s, [(0, -20), (0, 20)])
    for sy in (-1, 1):
        s = s.union(box(16, 5, h_axis, ox, sy * 5.5))
        s = s.union(cq.Workplane("XZ").circle(8).extrude(2.5, both=True).translate((ox, sy * 5.5, h_axis)))
    return s.cut(tear("y", 3.05, 24, (ox, 0, h_axis)))


def eye_post():
    """Column with a vertical 2.7 mm hole for an M3 eye bolt (guide-wear candidate C)."""
    s = foot(20, 48, (-20, 20))
    s = s.union(box(12, 12, 31))
    return s.cut(cyl_z(2.7, 10.1, 0, 0, 21))


def stop_block():
    """Sheath stop: counterbore 2.4 mm (sheath end) on the +x face, 1.0 mm line hole through."""
    s = foot(14, 30, (-10, 10), 4.0)
    s = s.union(box(10, 10, Z_LINE + 5))
    s = s.cut(tear("x", 1.0, 20, (0, 0, Z_LINE)))
    cb = tear("x", 2.4, 3.0, (4.5 - 1.5 + 0.0 + 1.0, 0, Z_LINE))  # spans x 3.0..6.0 -> 2 mm deep from the +x face
    return s.cut(cb)


def clamp_block(two=False):
    """Line clamp: M2 screws + washers press the line into a groove (two lines, 11.5 mm apart, if two=True).
    Nut traps on the underside."""
    ys = (-5.75, 5.75) if two else (0.0,)
    s = foot(20, 48, (-20, 20))
    s = s.union(box(20, 24 if two else 12, 8))
    for y in ys:
        s = s.cut(box(30, 1.4, 1.2, 0, y, 8 - 1.0))
        for x in (-5, 5):
            s = s.cut(cyl_z(2.3, 20, x, y, -2))
            s = s.cut(hexpocket(4.3, 1.9, x, y, 0))
    return s


def gauge_post():
    s = foot(10, 30, (-10, 10), 3.0)
    return s.union(box(6, 14, 26))


def flag_clip():
    """Sits on the line: calliper flag.  Line through a 1 mm hole, an M2 self-tapping screw clamps it."""
    s = box(10, 8, 8)
    s = s.cut(tear("x", 1.0, 20, (0, 0, 4)))
    return s.cut(cyl_z(1.7, 10, 0, 0, -1))


def fuse_seat():
    """Holds one of agent 07's fuse cups (oval 4.3 x 3.5, lip toward +x) or a bare crimp tube against the +x wall.
    Open-top trough, so cups drop in; the pull is on the line through the 1.1 mm hole in the wall.  Print as is."""
    s = foot(26, 48, (-20, 20))
    s = s.union(box(24, 14, 12))
    zc = 8.0
    cw, ch_ = CUP_OUTER[0] + 0.3, CUP_OUTER[1] + 0.3          # the cup lies on its side: its long axis (4.3) is along y
    L = CUP_CHAMBER[2] + 1.2 + 0.4
    s = s.cut(box(L, cw, 12 - (zc - ch_ / 2) + 1, -3.0 + L / 2, 0, zc - ch_ / 2))
    s = s.cut(tear("x", 1.1, 30, (8.0, 0, zc)))
    return s


def hang_drum():
    """Load spool: line wraps 3 turns on the 16 mm core; an M5 bolt through the bore carries the bucket hook.
    Print with the axis vertical (the top flange bridges 5 mm)."""
    pts = [(2.6, 0), (13, 0), (13, 2), (8, 2), (8, 8), (13, 8), (13, 10), (2.6, 10)]
    d = cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0))
    return d.cut(tear("x", 1.6, 40, (0, 0, 5)))


def servo_cradle(pil_h=22.0):
    """Hangs an SCS0009 by its two tab screws (M2) under a ledge; shaft points up.  Line leaves along +x
    (rotate 90 deg about z to run the servo body along the line).  pil_h 22 -> spool line ~33 mm; 32 -> ~43 mm."""
    s = foot(32, 48, (-20, 20))
    for sy in (-1, 1):
        y = sy * SCS["hole_spacing"] / 2
        s = s.union(box(12, 5, pil_h, 0, y))
        s = s.cut(cyl_z(2.3, 40, 0, y, 0))
        s = s.cut(hexpocket(4.3, 1.9, 0, y, 0))
    s = s.union(box(3, 36, 14, -9.0, 0))
    return s


def joint_fork(h_axis=39.0, ox=0.0, gap=12.4):
    """Fork carrying a 4 mm pin along y; cheeks `gap` mm apart (12.4 for one 12 mm hub, 22.4 for two 10 mm pucks)."""
    s = box(32 if ox else 20, 48, 4, 6 if ox else 0, 0)
    s = hole_pts(s, [(0, -20), (0, 20)])
    for sy in (-1, 1):
        yc = sy * (gap / 2 + 2.0)
        s = s.union(box(18, 4, h_axis, ox, yc))
        s = s.union(cq.Workplane("XZ").circle(9).extrude(2, both=True).translate((ox, yc, h_axis)))
    return s.cut(tear("y", 4.05, 40, (ox, 0, h_axis)))


def lever_drum():
    """Drum r6 on two MR84 bearings (4x8x3) + 64 mm lever with a pad nub (lever length to the nub 60 mm).
    Modelled with the hub axis = y.  Print with the hub axis vertical."""
    s = cyl_y(12, -6, 6)
    s = s.union(cyl_y(16, -6, -4)).union(cyl_y(16, 4, 6))
    s = s.union(box(64, 8, 7, 32, 0, -3.5))
    s = s.union(cyl_z(8, 4, 60, 0, -7.5).faces("<Z").chamfer(1.0))
    s = s.cut(cyl_y(7.95, -6.1, -3)).cut(cyl_y(7.95, 3, 6.1)).cut(cyl_y(5.0, -7, 7))
    for x in (30, 50):
        s = s.cut(cyl_z(3.2, 20, x, 0, -10))
    return s.cut(tear("x", 1.6, 30, (0, 0, -4.5)))


def drum_puck(ring=False):
    """Creep puck: flanges D15, 3.2 mm groove, 10 mm wide.  Bare: core D14 (printed groove, same surface radius as the ring).
    ring=True: core D13 so a 14 x 0.5 mm brass ring (agent 07's) slides on (surface D14, lips 0.5 mm)."""
    core = RING_ID if ring else RING_OD
    p = cyl_y(15, -5, -1.6).union(cyl_y(15, 1.6, 5)).union(cyl_y(core, -1.7, 1.7))
    return p.cut(cyl_y(4.1, -6, 6))


def drum_ring():
    """Split ring 14 OD / 13 ID, 3 mm wide, 0.3 mm gap (agent 07's 14 x 0.5 brass tube slice).  Metal in real use."""
    r = cyl_z(RING_OD, RING_W).cut(cyl_z(RING_ID, 8, 0, 0, -2))
    return r.cut(box(0.3, 3, 4, 0, 6.5, -0.5))


def capstan_post():
    s = box(28, 28, 4)
    s = hole_pts(s, [(-10, -10), (10, 10), (-10, 10), (10, -10)])
    s = s.union(cyl_z(12, 30, 0, 0, 0))
    return s.cut(cyl_z(3.05, 40, 0, 0, 0))


def sleeve_petg():
    return cyl_z(8, 22).cut(cyl_z(3.05, 30, 0, 0, -2))


def bend_form(angle=180):
    """Plate with a snap-in keyhole groove (2.05 mm circle, 1.7 mm mouth) on a 15 mm radius, for PTFE / sheath friction.
    Print groove side up, no supports."""
    R, T, zc = 15.0, 6.0, 3.6
    w, h = 50.0, (34.0 if angle == 180 else 48.0)
    plate = box(w, h, T, 0, 0)
    if angle == 180:
        pts = ((-R, -R), (-R, 0), (0, R), (R, 0), (R, -R))
        path = cq.Workplane("XY").moveTo(-R, -30).lineTo(-R, 0).threePointArc((0, R), (R, 0)).lineTo(R, -30)
        start, d0 = (-R, -30), (0, 1, 0)
    else:
        path = cq.Workplane("XY").moveTo(-R, -30).lineTo(-R, 0).threePointArc((-R * math.cos(math.radians(45)), R * math.sin(math.radians(45))), (0, R)).lineTo(30, R)
        start, d0 = (-R, -30), (0, 1, 0)
    path = path.translate((0, 0, zc))
    plane = cq.Plane(origin=(start[0], start[1], zc), xDir=(1, 0, 0), normal=d0)
    circ = cq.Workplane(plane).circle(2.05 / 2).sweep(path, transition="round")
    slot = cq.Workplane(plane).center(0, 0).rect(1.7, 2 * (T - zc) + 0.2).sweep(path.translate((0, 0, 0)), transition="round")
    # slot sweep is centred on the path: lift it so that it spans from the tube centre to above the top face
    slot = slot.translate((0, 0, (T - zc) + 0.1 - 0.0))
    plate = plate.cut(circ).cut(slot)
    big = max(plate.solids().vals(), key=lambda v: v.Volume())
    return cq.Workplane("XY").newObject([big])


def friction_sled():
    """40 x 40 x 10 sled; pad swatch pocket 20.4 x 20.4 x 2.3 on the sliding face; flat top for a 500 g weight; 3.5 mm tie hole.
    Print with the sliding face UP."""
    s = box(40, 40, 10)
    s = s.cut(box(20.4, 20.4, 2.3, 0, 0, 10 - 2.3 + 0.01))      # pocket in the top face (this is the sliding face when flipped)
    s = s.union(box(12, 8, 6, 0, -22, 0))
    s = s.cut(tear("y", 3.5, 20, (0, -22, 3)))
    return s


def cup_coupon(t_lip, n_x=4, n_y=3, pitch=(6.5, 5.5)):
    """12 loose fuse cups (agent 07 round-2 geometry) for the shear / crimp tests.  Lip layer on the bed, chamber open
    upward: prints without supports.  'On its side' = rotate the plate 90 deg in the slicer."""
    out = None
    for i in range(n_x):
        for j in range(n_y):
            x, y = (i - (n_x - 1) / 2) * pitch[0], (j - (n_y - 1) / 2) * pitch[1]
            h = CUP_CHAMBER[2] + t_lip
            c = cq.Workplane("XY").slot2D(CUP_OUTER[0], CUP_OUTER[1], 0).extrude(h).translate((x, y, 0))
            ch = cq.Workplane("XY").slot2D(CUP_CHAMBER[0], CUP_CHAMBER[1], 0).extrude(CUP_CHAMBER[2] + 0.1).translate((x, y, t_lip))
            c = c.cut(ch).cut(cyl_z(CUP_LIP_HOLE, t_lip + 0.2, x, y, -0.1))
            out = c if out is None else out.union(c)
    return out


def eye_platform():
    """Sits on the servo-2 horn (shaft = origin): a disc with an upright plate whose face is normal to the line (x).
    A 12 x 12 x 1.0 mm brass/stainless coupon with the test hole goes in the pocket on the -x face (epoxy).
    Eye centre is 9 mm above the disc bottom.  M2 holes (4 at 14 x 8 mm) are a guess: drill to your horn."""
    d = cyl_z(26, 3.0)
    for x in (-7, 7):
        for y in (-4, 4):
            d = d.cut(cyl_z(2.2, 8, x, y, -1))
    up = box(3.0, 22, 18, 0, 0, 3.0)
    up = up.cut(box(1.05, 12.2, 12.2, -1.5 + 0.525, 0, 9 - 6.1 + 3.0 - 3.0))
    up = up.cut(cq.Workplane("YZ").circle(2.5).extrude(6, both=True).translate((0, 0, 9)))
    return d.union(up)


def guide_post_skew15():
    """Guide post with the dowel axis turned 15 deg in plan: the line meets the dowel at a 15 deg fleet angle (report 10 #8)."""
    s = box(32, 48, 5, 6, 0)
    s = hole_pts(s, [(0, -20), (0, 20)])
    ox, h = 14.0, Z_SERVO
    cheeks = None
    for sy in (-1, 1):
        c = box(16, 5, h, 0, sy * 5.5).union(cq.Workplane("XZ").circle(8).extrude(2.5, both=True).translate((0, sy * 5.5, h)))
        cheeks = c if cheeks is None else cheeks.union(c)
    cheeks = cheeks.cut(tear("y", 3.05, 24, (0, 0, h))).rotate((0, 0, 0), (0, 0, 1), 15).translate((ox, 0, 0))
    return s.union(cheeks)


def protractor_disc():
    """D50 x 3 disc, 4.1 mm bore, engraved ticks every 5 deg (long every 10), for the Hall jig.  Print flat."""
    d = cyl_z(50, 3.0).cut(cyl_z(4.1, 8, 0, 0, -1))
    for k in range(0, 360, 5):
        ln = 7.0 if k % 10 == 0 else 4.0
        t = box(0.6, ln, 1.0, 0, 25 - ln / 2 - 0.5, 2.0 + 0.01).rotate((0, 0, 0), (0, 0, 1), -k)
        d = d.cut(t)
    return d.cut(box(1.2, 10, 1.0, 0, 18, 2.01))


def hall_arm():
    """Holds a Hall breakout (AS5600 / TMAG5273 board) upright in front of the pin end (board face at local y -14, 2-4 mm
    from the magnet; add M2 washers as stand-offs).  Slots for the board screws.  Foot holes at (+-20, 0), x along the plate."""
    s = box(48, 20, 4)
    s = hole_pts(s, [(-20, 0), (20, 0)])
    s = s.union(box(30, 4, 34, 0, -12))
    for xx in (-9, 9):
        s = s.cut(box(2.4, 8, 10, xx, -12, 20))
    return s


def pointer_post():
    """Fixed pointer next to the protractor disc.  Foot holes at (+-10,0)."""
    s = box(30, 10, 4).cut(cyl_z(3.4, 20, -10, 0, -1)).cut(cyl_z(3.4, 20, 10, 0, -1))
    s = s.union(box(6, 6, 52, 0, 0, 0))
    arm = cq.Workplane("XY").polyline([(-1.5, 0), (1.5, 0), (0.3, 12), (-0.3, 12)]).close().extrude(1.6).translate((0, 0, 50.4))
    return s.union(arm)


def hall_spacers():
    """Four spacer rings (axis vertical) 3/5/7/10 mm tall, OD 7.5, ID 4.2: they slide the MR84 away from the magnet.
    Pin protrudes 13.5 mm past the +y fork cheek, magnet flush in its end: magnet-to-bearing distance = 11 - spacer."""
    out = None
    for i, h in enumerate((3, 5, 7, 10)):
        r = cyl_z(7.5, h, i * 10, 0, 0).cut(cyl_z(4.2, h + 2, i * 10, 0, -1))
        out = r if out is None else out.union(r)
    return out


PARTS = {
    "base_plate": (base_plate, "flat, nut-pocket side down (bed side), PETG, 4 walls"),
    "roller_post": (roller_post, "foot on the bed, PETG, teardrop dowel hole"),
    "guide_post_tall": (lambda: roller_post(Z_SERVO), "foot on the bed, PETG/PAHT-CF"),
    "guide_post_eye": (lambda: roller_post(Z_EYE), "foot on the bed (line at 43 mm)"),
    "guide_post_skew15": (guide_post_skew15, "foot on the bed, dowel turned 15 deg"),
    "eye_post": (eye_post, "foot on the bed, PETG"),
    "stop_block": (stop_block, "upright, teardrop holes, PETG"),
    "clamp_block": (clamp_block, "foot on the bed"),
    "clamp_block_2": (lambda: clamp_block(True), "foot on the bed (two lines, for R6)"),
    "gauge_post": (gauge_post, "foot on the bed"),
    "flag_clip": (flag_clip, "as modelled"),
    "fuse_seat": (fuse_seat, "foot on the bed, pocket up"),
    "hang_drum": (hang_drum, "axis vertical, PETG, 5 walls"),
    "servo_cradle": (servo_cradle, "foot on the bed, PETG or PAHT-CF"),
    "servo_cradle_tall": (lambda: servo_cradle(32.0), "foot on the bed (eye-plate rig, line at 43 mm)"),
    "joint_fork": (joint_fork, "foot on the bed"),
    "puck_fork": (lambda: joint_fork(20.0, 14.0, 22.4), "foot on the bed (edge version, two pucks side by side)"),
    "lever_drum": (lever_drum, "hub axis vertical (rotate 90 deg about x), PETG, 5 walls"),
    "puck_bare": (lambda: drum_puck(False), "axis vertical, PETG (the test material)"),
    "puck_ring_seat": (lambda: drum_puck(True), "axis vertical, PETG"),
    "drum_ring": (drum_ring, "flat; metal in real use (slice of 14 x 0.5 mm tube), printed copy for fit check"),
    "capstan_post": (capstan_post, "foot on the bed"),
    "sleeve_petg": (sleeve_petg, "axis vertical"),
    "bend_form_180": (lambda: bend_form(180), "groove side up, no supports"),
    "bend_form_90": (lambda: bend_form(90), "groove side up, no supports"),
    "cup_coupon_t0p6": (lambda: cup_coupon(0.6), "flat on the bed, chamber up, 12 cups"),
    "cup_coupon_t0p8": (lambda: cup_coupon(0.8), "flat on the bed, chamber up, 12 cups"),
    "cup_coupon_t1p0": (lambda: cup_coupon(1.0), "flat on the bed, chamber up, 12 cups"),
    "cup_coupon_t1p2": (lambda: cup_coupon(1.2), "flat on the bed, chamber up, 12 cups"),
    "eye_platform": (eye_platform, "disc on the bed, upright plate up"),
    "hall_fork": (lambda: joint_fork(30.0), "foot on the bed"),
    "hall_arm": (hall_arm, "foot on the bed"),
    "pointer_post": (pointer_post, "foot on the bed"),
    "protractor_disc": (protractor_disc, "flat, ticks up"),
    "hall_spacers": (hall_spacers, "four rings, axis vertical"),
    "friction_sled": (friction_sled, "pocket (sliding) face up"),
}


def check(name, w):
    sol = w.val()
    ok = all(v.isValid() and v.Volume() > 0 for v in w.solids().vals())
    sol = cq.Compound.makeCompound(w.solids().vals()) if len(w.solids().vals()) > 1 else sol
    bb = sol.BoundingBox()
    n_solids = len(w.solids().vals())
    return {"valid": bool(ok), "solids": n_solids, "volume_mm3": round(sol.Volume(), 1),
            "bbox": [round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)], "mass_g_petg_25pct": round(sol.Volume() * 1.27e-3 * 0.55, 1)}


def export_all():
    info = {}
    meshes = {}
    for name, (fn, orient) in PARTS.items():
        w = fn()
        info[name] = check(name, w)
        info[name]["print"] = orient
        cq.exporters.export(w, str(OUT / f"{name}.step"))
        cq.exporters.export(w, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
        meshes[name] = trimesh.load(OUT / f"{name}.stl")
        info[name]["watertight"] = bool(meshes[name].is_watertight)
        print(f"{name:16s} valid={info[name]['valid']} solids={info[name]['solids']} wt={info[name]['watertight']} bbox={info[name]['bbox']}")
    (OUT / "parts.json").write_text(json.dumps(info, indent=1))
    return meshes


# ------------------------------------------------------------------------------------------ rendering
def T(m, pos, rot_z=0.0, rot_x=0.0):
    m = m.copy()
    if rot_x:
        m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(rot_x), [1, 0, 0]))
    if rot_z:
        m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(rot_z), [0, 0, 1]))
    m.apply_translation(pos)
    return m


def rigs(M):
    base = (M["base_plate"], "#b8c0cc")
    HD = lambda x, y=50: (T(M["hang_drum"], (x, y, -42), rot_x=90), "#9b8bd4")
    zl = Z_LINE
    ex = 140 + 14 + 1.6        # x where the line falls past the plate edge (roller dowel at x 154)
    r1 = [base, (T(M["clamp_block"], (12, 50, 0)), "#4c8bf5"), (T(M["stop_block"], (30, 50, 0)), "#e8a33d"),
          (T(M["stop_block"], (95, 50, 0), rot_z=180), "#e8a33d"), (T(M["gauge_post"], (109, 50, 0)), "#7bc47f"),
          (T(M["flag_clip"], (124, 50, zl - 4)), "#d9534f"), (T(M["roller_post"], (140, 50, 0)), "#4c8bf5"), HD(ex)]
    s1 = [[(12, 50, 8), (ex, 50, zl + 1.5)], [(ex, 50, zl), (ex, 50, -30)]]
    r2 = [base, (T(M["bend_form_180"], (65, 50, 0)), "#e8a33d"), (T(M["roller_post"], (50, 10, 0), rot_z=-90), "#4c8bf5"),
          (T(M["roller_post"], (120, 10, 0), rot_z=-90), "#4c8bf5"), (T(M["capstan_post"], (130, 50, 0)), "#7bc47f")]
    s2 = [[(50, 35, 3.6), (50, 50, 3.6), (65, 65, 3.6), (80, 50, 3.6), (80, 35, 3.6)], [(80, 35, 3.6), (80, -30, 3.6)],
          [(50, 35, 3.6), (50, -5.5, 16.5), (50, -5.5, -30)],
          [(120, -5.5, -30), (120, -5.5, 16.5), (120, 40, 16.5), (126, 48, 16.5), (130, 54, 16.5), (134, 48, 16.5), (134, -30, 16.5)]]
    r3 = [base, (T(M["fuse_seat"], (20, 50, 0)), "#e8a33d"), (T(M["roller_post"], (140, 50, 0)), "#4c8bf5"), HD(ex)]
    s3 = [[(20, 50, 10), (ex, 50, zl + 1.5)], [(ex, 50, zl), (ex, 50, -30)]]
    r4 = [base, (T(M["servo_cradle"], (30, 50, 0)), "#4c8bf5"), (T(M["guide_post_tall"], (140, 50, 0)), "#e8a33d"), HD(ex)]
    s4 = [[(36, 50, Z_SERVO), (ex, 50, Z_SERVO + 1.5)], [(ex, 50, Z_SERVO), (ex, 50, -30)]]
    r8 = [base, (T(M["servo_cradle_tall"], (30, 50, 0), rot_z=90), "#4c8bf5"), (T(M["servo_cradle"], (90, 50, 0), rot_z=90), "#7bc47f"),
          (T(M["eye_platform"], (96, 50, 34)), "#d9534f"), (T(M["guide_post_eye"], (140, 50, 0)), "#e8a33d"), HD(ex)]
    s8 = [[(36, 50, Z_EYE), (96, 50, Z_EYE), (ex, 50, Z_EYE + 1.5)], [(ex, 50, Z_EYE), (ex, 50, -30)]]
    r9 = [base, (T(M["hall_fork"], (60, 50, 0)), "#e8a33d"), (T(M["protractor_disc"], (60, 36, 30), rot_x=90), "#cccccc"),
          (T(M["pointer_post"], (60, 20, 0)), "#4c8bf5"), (T(M["hall_arm"], (60, 90, 0)), "#7bc47f"),
          (T(M["hall_spacers"], (10, 90, 0)), "#9b8bd4")]
    s9 = [[(60, 33, 30), (60, 73.7, 30)]]
    r5 = [base, (T(M["servo_cradle"], (25, 50, 0)), "#4c8bf5"), (T(M["joint_fork"], (95, 50, 0)), "#e8a33d"),
          (T(M["lever_drum"], (95, 50, 39)), "#d9534f")]
    s5 = [[(31, 50, Z_SERVO), (95, 50, 33)]]
    r6 = [base, (T(M["clamp_block_2"], (12, 50, 0)), "#4c8bf5"),
          (T(M["puck_fork"], (140, 50, 0)), "#e8a33d"),
          (T(M["puck_bare"], (154, 44.5, 20)), "#7bc47f"), (T(M["puck_ring_seat"], (154, 55.5, 20)), "#d9534f"),
          HD(163, 44.5), HD(163, 55.5)]
    s6 = [[(12, 44.25, 8), (154, 44.5, 28), (162, 44.5, 20), (162, 44.5, -30)], [(12, 55.75, 8), (154, 55.5, 28), (162, 55.5, 20), (162, 55.5, -30)]]
    r7 = [(T(M["friction_sled"], (40, 40, 0)), "#e8a33d")]
    s7 = [[(40, 20, 3), (40, -30, 3)]]
    return [("R1 line and sheath frame", r1, s1), ("R2 capstan and bend forms", r2, s2), ("R3 anchor and fuse pull", r3, s3),
            ("R4 guide wear cycle (servo)", r4, s4), ("R5 joint lever (servo)", r5, s5), ("R6 drum creep", r6, s6),
            ("R7 pad friction sled", r7, s7),
            ("R8 eye-plate wear (two servos)", r8, s8), ("R9 Hall sensor next to a steel bearing", r9, s9)]


def render(rigs_list):
    """Own orthographic painter's-algorithm renderer (matplotlib 3D sorts big parts wrongly)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    light = np.array([0.35, -0.45, 0.85])
    light /= np.linalg.norm(light)

    def view_basis(elev, azim):
        e, a = math.radians(elev), math.radians(azim)
        d = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])   # towards the viewer
        right = np.cross([0, 0, 1], d)
        if np.linalg.norm(right) < 1e-6:
            right = np.array([1.0, 0, 0]) if elev > 0 else np.array([1.0, 0, 0])
        right /= np.linalg.norm(right)
        up = np.cross(d, right)
        return d, right, up

    for i, (title, items, lines) in enumerate(rigs_list, 1):
        fig, axes = plt.subplots(1, 2, figsize=(12, 6.4))
        for ax, (elev, azim) in zip(axes, ((35, -60), (90, -90.001))):
            d, right, up = view_basis(elev, azim)
            polys, cols, depth = [], [], []
            for m, col in items:
                tri = m.triangles
                n = m.face_normals
                vis = n @ d > 0.0
                tri, n = tri[vis], n[vis]
                sh = 0.4 + 0.6 * np.clip(n @ light, 0, 1)
                rgb = np.array(matplotlib.colors.to_rgb(col))
                cols.append(np.clip(rgb[None, :] * sh[:, None], 0, 1))
                polys.append(np.stack([tri @ right, tri @ up], axis=-1))
                depth.append(tri.mean(axis=1) @ d)
            polys = np.concatenate(polys)
            cols = np.concatenate(cols)
            depth = np.concatenate(depth)
            order = np.argsort(depth)
            ax.add_collection(PolyCollection(polys[order], facecolors=cols[order], edgecolors=cols[order], linewidths=0.3))
            for seg in lines:
                a = np.array(seg, float)
                ax.plot(a @ right, a @ up, color="#c0392b", lw=1.8)
            allp = polys.reshape(-1, 2)
            lo, hi = allp.min(0), allp.max(0)
            pad = 0.04 * (hi - lo).max()
            ax.set_xlim(lo[0] - pad, hi[0] + pad)
            ax.set_ylim(lo[1] - pad, hi[1] + pad)
            ax.set_aspect("equal")
            ax.axis("off")
        fig.suptitle(title + "   (red = line; plate 150 x 100 mm)")
        fig.tight_layout()
        fig.savefig(OUT / f"rig_R{i}.png", dpi=110)
        plt.close(fig)
    print("renders done")


if __name__ == "__main__":
    meshes = export_all()
    render(rigs(meshes))
