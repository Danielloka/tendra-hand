"""15 · Sensor parts for the Tendra V2.1 hand (agent 15, 2026-11-15).

Run (study folder):
  uv run --no-project --with cadquery --with numpy --with trimesh --with matplotlib python cad/15_sensor_parts.py
Writes out/15/<part>.step + .stl, out/15/sensor_spec.json (read by the finger builder), checks + renders.

Frames
- Finger parts: the bench-finger frame of cad/07_finger.py (MCP axis = X through the origin, finger +Z,
  -Y palm, +X thumb side; PIP at z 42, DIP at z 66, abd axis along Y at z -13).
- Hub, wrist: hand frame of the V2 study (origin = wrist centre, +Z to the fingers, -Y palm, +X thumb).

Parts
  joint_board         Ø6.4 x 0.6 round sensor board, TMAG5273 (SOT-23-6) on the OUTER face, sensing through the board
  joint_cap_<j>       printed PETG cap that holds the board in the lug's Ø6.75 counterbore, wire slot to +y
  joint_module_<j>    assembly (magnet, keeper, board, chip, cap) for mcp / pip / dip in finger frame
  abd_module          magnet on a non-magnetic stub end + board + cap (palm knuckle side)
  tip_carrier         tray that holds the 9.2 x 9.0 tip board (MLX90393) in the distal cavity, strain relief tab
  pad_board / pad_cup BMP581 pad sensor (board + silicone cup mould insert) for phalanx and palm zones
  hub_tray            palm hub holder: RP2040-Zero on a 32 x 26 carrier, 7 x JST SH, M2 bosses
  wrist_pin           Ø5 hollow wrist pin (bore Ø2.4) with radial cable exits; ring_cable_clip; dorsal_loop_clip
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "out" / "15"
OUT.mkdir(parents=True, exist_ok=True)


def V(*a):
    return cq.Vector(*[float(x) for x in a])


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane().box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl(r, p0, p1):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    h = float(np.linalg.norm(d))
    return cq.Workplane(obj=cq.Solid.makeCylinder(float(r), h, V(*p0), V(*(d / h))))


def cylx(r, x0, x1, y=0.0, z=0.0):
    return cyl(r, (x0, y, z), (x1, y, z))


def cyly(r, y0, y1, x=0.0, z=0.0):
    return cyl(r, (x, y0, z), (x, y1, z))


def cylz(r, z0, z1, x=0.0, y=0.0):
    return cyl(r, (x, y, z0), (x, y, z1))


PARTS: dict[str, cq.Workplane] = {}
CHECK = {}


def save(name, w):
    PARTS[name] = w
    s = w.val()
    CHECK[name] = dict(valid=bool(s.isValid()), volume_mm3=round(s.Volume(), 2),
                       bbox=[round(v, 2) for v in (s.BoundingBox().xmin, s.BoundingBox().xmax, s.BoundingBox().ymin,
                                                    s.BoundingBox().ymax, s.BoundingBox().zmin, s.BoundingBox().zmax)])
    cq.exporters.export(w, str(OUT / f"{name}.step"))
    cq.exporters.export(w, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.15)


# ============================================================ joint sensor module (R3 slim: chip faces the magnet)
# Agreed with the finger builder (12, R2 geometry): no width added at the MCP knuckle.
#  - sensor-side (+x) bearing = OPEN full-ceramic ZrO2 MR84 (4 x 8 x 2, non-magnetic) instead of MR84ZZ (3 wide):
#    frees 1.0 mm; fallback a 4 x 8 x 2 iglidur/bronze flanged bush (~2 % friction)
#  - magnet flush with the bearing's outer face, in the bore of the tube end (MCP: R2 inner tube Ø3 x 0.5 -> Ø2 bore)
#  - 0.30 air gap, TMAG5273 facing the magnet (die ~0.9 under the package top -> die gap 1.2), 0.4 board, 0.2 roof
BORE = 6.75  # counterbore behind every bearing seat (07/12)
BRD_D, BRD_T = 6.4, 0.4
CHIP = (2.9, 2.8, 1.45)  # SOT-23-6 incl. leads [14], max height 1.45 [M]
DIE_BELOW_TOP = 0.90  # [M] die depth under the package top (lead frame near the bottom)
GAP = 0.30
ROOF = 0.20
# name: (z of axis, x of magnet face = sensor-side bearing outer face, magnet (d, t), x of 12's lug outer face, parent, child)
JOINTS = {
    "mcp_flex": (0.0, 6.4, (2.0, 1.5), 9.0, "proximal", "yoke (R2 inner tube, cut to end at x 6.4)"),
    "pip": (42.0, 6.25, (3.0, 1.0), 8.05, "proximal", "middle (pin, ends at x 6.25)"),
    "dip": (66.0, 6.0, (3.0, 1.0), 7.5, "distal", "middle (pin, shortened from 7.5 to 6.0)"),
}
FIELD = {"mcp_flex": 55.9, "pip": 66.5, "dip": 66.5}  # mT at die gap 1.2 (cad/15 surface-charge model, no steel)


def joint_parts(x_face, z, mag):
    xc0 = x_face + GAP
    chip = box(xc0, xc0 + CHIP[2], -1.4, 1.4, z - 1.45, z + 1.45)
    xb0 = xc0 + CHIP[2]
    board = cylx(BRD_D / 2, xb0, xb0 + BRD_T, 0, z)
    x_out = xb0 + BRD_T + ROOF
    cap = cylx(6.70 / 2, x_face + 0.05, x_out, 0, z)  # presses into the Ø6.75 counterbore, also stops the outer race
    cap = cap.cut(cylx(BRD_D / 2 + 0.05, xb0 - 0.01, xb0 + BRD_T, 0, z))  # board pocket
    cap = cap.cut(box(xc0 - 0.3, xb0 + 0.01, -1.6, 1.6, z - 1.65, z + 1.65))  # chip pocket, open toward the magnet
    cap = cap.cut(box(x_face + 0.04, xb0 + 0.01, -2.9, 2.9, z - 2.25, z + 2.25).cut(
        box(xc0 - 0.3, xb0 + 0.02, -1.6, 1.6, z - 1.65, z + 1.65)).intersect(cylx(3.0, x_face, xb0 + 0.02, 0, z)))  # pad bay
    cap = cap.cut(box(xb0 - 0.6, xb0 + 0.01, 0.0, 4.0, z - 0.6, z + 0.6))  # wire slot to +y (dorsal)
    magnet = cylx(mag[0] / 2, x_face - mag[1], x_face, 0, z)
    keeper = cylx(mag[0] / 2 - 0.05, x_face - mag[1] - 1.5, x_face - mag[1], 0, z)
    return cap, board, chip, magnet, keeper, x_out


SPEC = {"frame_finger": "bench-finger frame of cad/07_finger.py (mm): MCP axis X through origin, finger +Z, -Y palm, "
                        "+X thumb side; PIP z 42, DIP z 66, abd axis Y at z -13",
        "frame_hand": "V2 hand frame: origin wrist centre, +Z to fingers, -Y palm, +X thumb",
        "sensors": [], "parts": {}, "wiring": {}, "connectors": {}, "wrist_passage": {}, "notes": []}

for j, (z, x_face, mag, x_lug, parent, child) in JOINTS.items():
    cap, b, chip, magnet, keeper, x_out = joint_parts(x_face, z, mag)
    save(f"joint_cap_{j}", cap)
    save(f"joint_module_{j}", cap.union(b).union(chip).union(magnet).union(keeper))
    SPEC["parts"][f"joint_cap_{j}"] = dict(body=parent, file=f"out/15/joint_cap_{j}.step")
    SPEC["sensors"].append(dict(
        id=f"<finger>_{j}", kind="joint_angle", chip="TMAG5273" + {"mcp_flex": "B1 (0x22)", "pip": "C1 (0x78)", "dip": "D1 (0x44)"}[j],
        range="+-80 mT, X-Y angle mode (CORDIC); field at the die %.0f mT (no steel nearby)" % FIELD[j],
        sensor_side_bearing={"mcp_flex": "OPEN full-ceramic ZrO2 MR84 4x8x2 at x 4.4-6.4 (was MR84ZZ 4.4-7.4); fallback 4x8x2 flanged iglidur/bronze bush",
                             "pip": "OPEN full-ceramic ZrO2 MR84 4x8x2 at x 4.25-6.25 (was MR84ZZ 4.25-7.25); -x keeps the Ø5 brass bush (bar pin A web)",
                             "dip": "brass bushes (unchanged); +x bush x 4.5-6.0"}[j],
        body=parent, magnet=dict(type=f"Ø{mag[0]} x {mag[1]} N45 diametric", body=child, x=[round(x_face - mag[1], 2), x_face], y=0, z=z,
                                 seat="in the tube bore, face flush with the tube end = bearing outer face; keeper plug behind; epoxy"),
        chip_x=[round(x_face + GAP, 2), round(x_face + GAP + CHIP[2], 2)], chip_faces="-x (toward the magnet)",
        board=f"Ø{BRD_D} x {BRD_T} FR4 behind the chip, 4 pads 1.0 x 0.9 on its outer face (bus in + out)",
        board_x=[round(x_face + GAP + CHIP[2], 2), round(x_face + GAP + CHIP[2] + BRD_T, 2)],
        die_gap_to_magnet=round(GAP + DIE_BELOW_TOP, 2),
        pocket=dict(type="Ø6.75 counterbore from the bearing outer face to the lug face", d=6.75, x=[x_face, x_lug]),
        outer_x=round(x_out, 2), lug_face_x=x_lug, width_change=round(max(0.0, x_out - x_lug), 2),
        orientation="chip X-Y plane = finger Y-Z plane; joint axis = chip Z",
        wire_exit="0.6 x 1.2 slot in the cap toward +y, then the dorsal groove (see wiring)"))

# abd module: magnet glued on the end of a non-magnetic Ø3 stub (brass/316) at the dorsal abd stub end
Z_ABD = -13.0
y_end = 16.7
ABD_GAP, DIE_ABOVE_PCB = 0.5, 0.4  # abd is on the palm: room, so chip-out (senses through the board)
mag_abd = cyly(1.5, y_end, y_end + 1.0, 0, Z_ABD)
y0c = y_end + 1.0 + ABD_GAP
brd_abd = cyly(BRD_D / 2, y0c, y0c + BRD_T, 0, Z_ABD)
chip_abd = box(-1.45, 1.45, y0c + BRD_T, y0c + BRD_T + CHIP[2], Z_ABD - 1.4, Z_ABD + 1.4)
cap_abd = cyly(3.35, y0c, y0c + BRD_T + CHIP[2] + 0.3, 0, Z_ABD).union(cyly(4.2, y0c + BRD_T + CHIP[2] - 0.2, y0c + BRD_T + CHIP[2] + 0.3, 0, Z_ABD))
cap_abd = cap_abd.cut(cyly(BRD_D / 2 + 0.05, y0c - 0.1, y0c + BRD_T, 0, Z_ABD)).cut(
    box(-1.65, 1.65, y0c + BRD_T - 0.01, y0c + BRD_T + CHIP[2] + 0.05, Z_ABD - 1.6, Z_ABD + 1.6)).cut(
    box(-0.6, 0.6, y0c + BRD_T - 0.01, y0c + BRD_T + 1.0, Z_ABD, Z_ABD + 5.0))
save("joint_cap_abd", cap_abd)
save("abd_module", cap_abd.union(brd_abd).union(chip_abd).union(mag_abd))
SPEC["sensors"].append(dict(
    id="<finger>_mcp_abd", kind="joint_angle", chip="TMAG5273A1 (0x35)", body="palm knuckle module (07 bench: base dorsal lug)",
    magnet=dict(type="Ø3 x 1 N45 diametric", body="yoke", y=[y_end, y_end + 1.0], x=0, z=Z_ABD,
                seat="glued (epoxy) on the end face of the dorsal abd stub; the stub becomes BRASS or 316 (not a hardened dowel)"),
    board_y=[round(y0c, 2), round(y0c + BRD_T, 2)], die_gap_to_magnet=round(ABD_GAP + BRD_T + DIE_ABOVE_PCB, 2),
    pocket=dict(type="Ø6.75 counterbore in the dorsal lug, coaxial with the abd axis", d=6.75, y=[round(y0c, 2), round(y0c + BRD_T + CHIP[2] + 0.3, 2)],
                note="replaces 07's Ø4.8 x 1.4 bore at y 16.6..18.0; outer face moves from y 17.8 to %.2f" % (y0c + BRD_T + CHIP[2] + 0.3)),
    orientation="chip Z = abd axis (Y)", wire_exit="slot toward +z (distal) into the knuckle board; static wiring (palm side)"))

# ============================================================ fingertip carrier (07 distal cavity)
ZD = 66.0
YF = -5.8
s = lambda v: YF - v  # noqa: E731  (07 helper: s(-4.8) = -1.0 = board top)
# 07: board x +-4.6, y -1.8..-1.0, z ZD+10.3..ZD+19.3 (9.2 x 9.0 x 0.8), chip 3 x 3 x 1 below it (faces -y)
tray = box(-4.75, 4.75, s(-4.8), s(-4.8) + 1.2, ZD + 10.15, ZD + 19.45)
tray = tray.cut(box(-4.65, 4.65, s(-4.8) - 0.01, s(-4.8) + 0.4, ZD + 10.25, ZD + 19.35))  # board recess 0.4 (board 0.8: 0.4 in the tray)
tab = box(-1.1, 1.1, s(-4.8) + 0.2, s(-4.8) + 1.2, ZD + 7.6, ZD + 10.2)  # strain-relief tongue into the Ø2.6 cable bore
tray = tray.union(tab).cut(box(-0.7, 0.7, s(-4.8) + 0.2, s(-4.8) + 0.8, ZD + 7.5, ZD + 12.0))  # 4-wire channel 1.4 x 0.6
save("tip_carrier", tray)
tip_board = box(-4.6, 4.6, s(-4.8) - 0.8, s(-4.8), ZD + 10.3, ZD + 19.3)
tip_chip = box(-1.5, 1.5, s(-4.8) - 1.8, s(-4.8) - 0.8, ZD + 13.5, ZD + 16.5)
tip_mag = cyly(1.5, s(2.4), s(1.4), 0.0, ZD + 15.0)
save("tip_assembly_ref", tray.union(tip_board).union(tip_chip).union(tip_mag))
SPEC["sensors"].append(dict(
    id="<finger>_tip", kind="tip_3axis", chip="MLX90393 QFN-16, option -014 (0x18)", body="distal",
    board=dict(size=[9.2, 9.0, 0.8], x=[-4.6, 4.6], y=[-1.8, -1.0], z=[ZD + 10.3, ZD + 19.3], note="07 geometry unchanged"),
    chip_box=dict(x=[-1.5, 1.5], y=[-2.8, -1.8], z=[ZD + 13.5, ZD + 16.5], faces="-y (palm)"),
    magnet=dict(type="Ø3 x 1.0 N42 AXIAL (was Ø3 x 1.5 N45 in 05/07: too strong, the MLX90393 saturates at 50 mT)",
                body="tip_skin", y=[round(s(2.4), 2), round(s(1.4), 2)], z=ZD + 15.0, seat="05's Ø2.8 hole in the skin, 0.6 silicone over it"),
    carrier=dict(file="out/15/tip_carrier.step", note="tray 9.5 x 9.3 x 1.2 glued into the cavity roof; tongue into the Ø2.6 bore clamps the 4 wires"),
    rate_Hz=200, signal="Bx, By, Bz -> Fx, Fy, Fz by a per-tip 3x3 (or small MLP) calibration on a kitchen scale + shear jig"))

# ============================================================ pad sensor (BMP581 in silicone)
pad_brd = box(-2.5, 2.5, 0.0, 0.6, -3.0, 3.0)
bmp = box(-1.0, 1.0, -0.8, 0.0, -1.0, 1.0)
pad_cup = box(-4.0, 4.0, -3.4, 0.6, -5.0, 5.0).cut(box(-3.2, 3.2, -3.5, 0.0, -4.2, 4.2)).cut(box(-2.6, 2.6, -0.01, 0.61, -3.1, 3.1))
save("pad_cup", pad_cup)
save("pad_assembly_ref", pad_cup.union(pad_brd).union(bmp))
for seg, frame_pos in (("pad_prox", "proximal palm face, centre at z 22, y -8.0 (pocket floor), x 0"),
                       ("pad_mid", "middle palm face, centre at z 54, y -7.0 (pocket floor), x 0")):
    SPEC["sensors"].append(dict(id=f"<finger>_{seg}", kind="pad_pressure", chip=f"BMP581 ({'0x46 SDO low' if seg == 'pad_prox' else '0x47 SDO high'})",
                                body="proximal" if seg == "pad_prox" else "middle", position=frame_pos,
                                pocket=dict(size=[8.0, 4.0, 10.0], note="8 (x) x 10 (z) x 4 (y) pocket in the palm face; board 5 x 6 x 0.6 on the floor, chip faces -y; the phalanx pad silicone (05 #14, rivet-lock) is cast over it and fills the cup"),
                                rate_Hz=200, signal="pressure (Pa) -> normal force, + 0.1-0.6 kHz vibration for slip"))

# ============================================================ palm hub tray (hand frame)
HUB = dict(L=32.0, W=26.0, T=1.0)
hub = box(-18.0, 18.0, 0.0, 2.0, 28.0, 58.0)  # floor 2.0 thick (y 8.5..10.5 in the palm), z 28..58, x -18..18
hub = hub.union(box(-18.0, 18.0, 0.0, 5.0, 28.0, 29.6)).union(box(-18.0, 18.0, 0.0, 5.0, 56.4, 58.0))  # end walls
for x in (-15.0, 15.0):
    for z in (31.0, 55.0):
        hub = hub.union(cyly(2.0, 0.0, 3.5, x, z)).cut(cyly(0.8, -0.1, 3.6, x, z))  # M2 self-tapping bosses
hub = hub.cut(box(-12.0, 12.0, -0.1, 2.1, 33.0, 53.0))  # window: strands / airflow, saves 2 g
# connector notches in the walls (7 x JST SH 4-pin + 1 x 6-pin): 6.2 wide, 3.2 tall
for i, x in enumerate(np.linspace(-13.5, 13.5, 5)):
    hub = hub.cut(box(x - 3.1, x + 3.1, 2.0, 5.1, 56.3, 58.1))  # finger plugs on the distal wall
for x in (-9.0, 0.0, 9.0):
    hub = hub.cut(box(x - 3.6, x + 3.6, 2.0, 5.1, 27.9, 29.7))  # wrist 6-pin + 2 palm-pad plugs on the proximal wall
hub = hub.translate((0, 8.5, 0))
save("hub_tray", hub)
hub_board = box(-16.0, 16.0, 10.5 + 1.5, 10.5 + 2.5, 30.0, 56.0)
rp = box(-9.0, 9.0, 12.0 + 1.0, 12.0 + 2.0, 31.25, 54.75)  # RP2040-Zero 18 x 23.5 on the carrier
save("hub_assembly_ref", hub.union(hub_board).union(rp))
SPEC["parts"]["hub_tray"] = dict(body="palm (dorsal shell, under a removable dorsal cover)", frame="hand",
                                 bbox=dict(x=[-18, 18], y=[8.5, 13.5], z=[28, 58]),
                                 contents="carrier PCB 32 x 26 x 1.0 (on 4 x M2 bosses, 1.5 mm stand-off) with RP2040-Zero (23.5 x 18), "
                                          "3.3 V LDO 300 mA, 7 x JST SH 4-pin (5 fingers + 2 palm-pad buses) and 1 x JST SH 6-pin (wrist)",
                                 keepout="strands pass under the floor (y < 8.5); window 24 x 20 in the floor")

# ============================================================ wrist passage
# Ø5 hollow pin (bore 2.4), 16 long, radial Ø1.6 exits 3 mm from each end (on opposite sides), entry chamfers
pin = cylz(2.5, 0.0, 16.0).cut(cylz(1.2, -0.1, 16.1))
for zz, sgn in ((3.0, 1), (13.0, -1)):
    pin = pin.cut(cyl(0.8, (0, 0, zz), (sgn * 3.0, 0, zz)))
save("wrist_pin_hollow", pin)
# ring cable clip: a 2 x 2 groove cover, 12 long, curved R 12 (follows a ring quadrant), 2 x M1.6
clip = cq.Workplane("XY").circle(14.0).circle(11.0).extrude(3.0).intersect(box(0, 15, 0, 15, 0, 3.0))
clip = clip.cut(cq.Workplane("XY").circle(13.0).circle(12.0).extrude(2.0).translate((0, 0, -0.01)))
save("ring_cable_clip", clip)
# dorsal service-loop clip (fallback): saddle for a Ø1.8 cable, 8 x 6 x 3, one M2
dl = box(-4.0, 4.0, 0.0, 3.0, 0.0, 6.0).cut(cylz(1.0, -0.1, 6.1, 0.0, 2.0)).cut(box(-0.8, 0.8, 2.0, 3.1, -0.1, 6.1))
dl = dl.cut(cyly(1.1, -0.1, 3.1, -2.6, 3.0)).cut(cyly(1.1, -0.1, 3.1, 2.6, 3.0))
save("dorsal_loop_clip", dl)
SPEC["wrist_passage"] = dict(
    chosen="through one hollow flex pin and one hollow dev pin of the wrist cardan (twist only); 6 x 38 AWG PTFE",
    conductors=["5V", "GND", "UART TX (hub->ESP32)", "UART RX (ESP32->hub)", "USB D+ (hub flashing)", "USB D-"],
    pin=dict(od=5.0, bore=2.4, file="out/15/wrist_pin_hollow.step", exits="Ø1.6 radial, 3 mm from each end, on the side of the body that is FIXED to that pin; free coaxial length >= 8 mm",
             stress="bore 2.4 in Ø5: section modulus -5.6 %, 13's W13 numbers 139 -> 147 MPa (SCS), 315 -> 334 MPa (HLS)"),
    path_waypoints_hand=[[0.0, 11.0, 26.0, "hub cable leaves the hub tray (proximal wall), dorsal"],
                         [14.0, 0.0, 6.0, "palm flex lug: enters the flex-pin bore from the palm side (fixed to the palm)"],
                         [16.0, 0.0, 0.0, "inside the flex pin (twist about X, flex -50..+50)"],
                         [14.0, 14.0, 0.0, "around the ring outer face in a 2 x 2 groove under ring_cable_clip (90 deg, static)"],
                         [0.0, 20.0, 0.0, "into the dev-pin bore from the ring side"],
                         [0.0, 20.0, -6.0, "exits into the forearm fork (fixed to the forearm), down the dorsal forearm wall"],
                         [0.0, 25.0, -120.0, "ESP32-S3 + RS-485/UART at forearm level 3"]],
    note="pin x/y positions are placeholders until report 13 fixes the new cardan (W13): keep the cable on the pin axes; "
         "never through the eye plate (centre cell (0,0) is free but a cable there kinks up to 58 deg)",
    fallback="dorsal service loop at x = 0: cable clipped to the palm back (z +25) and the forearm back (z -30), 70 mm free, "
             "bend R >= 15 mm (0.17 % strand strain), spiral wrap; on the dev axis so dev only bends it; flex changes its path by ~15 mm, taken by the loop")

# ============================================================ wiring (finger frame, one finger) and connectors
SPEC["wiring"] = dict(
    wire="4 x 38 AWG PTFE-insulated stranded (7 x 0.04, OD ~0.30), colours: red 3V3, black GND, yellow SDA, blue SCL; PARALLEL, never pre-twisted, through every pin",
    bus="one I2C bus per finger at 400 kHz, daisy-chained through each board's 4 pads (in + out wire on the same pad)",
    order=["tip board", "dip board", "DIP pin twist", "PIP pin twist", "pip board", "mcp board", "MCP axle twist",
           "yoke dorsal exit", "abd free loop", "knuckle board (abd chip + JST SH socket)", "palm harness", "hub"],
    waypoints_finger=[
        [0.0, -1.4, 76.3, "tip board pads (proximal edge) -> tip_carrier channel"],
        [0.0, -1.4, 73.6, "Ø2.6 cable bore (07) toward the DIP"],
        [5.5, 0.0, 72.0, "distal internal channel Ø1.2 up to the +x cheek: dip board pads (static)"],
        [5.5, 4.0, 70.5, "back across the distal dorsal side in a 1.2 x 1.0 groove under the nail"],
        [-6.35, 0.0, 66.0, "distal -x cheek wire layer (x -6.85..-5.85): glue point A (distal)"],
        [-5.85, 0.0, 66.0, "into the DIP pin -x end (bore Ø3)"],
        [3.2, 0.0, 66.0, "radial exit Ø1.2 along +y at x 3.2, z 66 (neutral axis for the drum load): glue point B (middle); free coaxial 9.0 mm"],
        [3.2, 3.0, 54.0, "middle channel (07: teardrop Ø1.6 at x 3.2, y 3.0)"],
        [3.6, 0.0, 42.0, "PIP pin radial entry Ø1.2 along +y at x 3.6: glue point C (middle)"],
        [-5.65, 0.0, 42.0, "PIP pin -x end -> proximal -x lug wire layer: glue point D (proximal); free coaxial 9.2 mm"],
        [-6.0, 7.0, 40.0, "up the -x face, across the dorsal back of the proximal in a 1.2 x 1.0 groove at z 38-40"],
        [7.6, 3.0, 42.0, "pip board (proximal +x lug cap)"],
        [9.4, 3.0, 21.0, "+x face groove of the proximal (1.2 x 1.0) down to the MCP"],
        [8.0, 3.0, 0.0, "mcp board (proximal +x lug cap)"],
        [0.0, 8.0, 6.0, "across the knuckle hood (dorsal) in a groove to -x"],
        [-8.2, 0.0, 0.0, "proximal -x lug counterbore: glue point E (proximal)"],
        [-7.4, 0.0, 0.0, "into the MCP axle -x end"],
        [3.0, 0.0, 0.0, "radial exit Ø1.2 along +y at x +3.0 through the R2 inner tube + +x sleeve + cheek (neutral axis): glue point F (yoke); free coaxial 10.4 mm. R2 has it at x -3.0: only 4.4 mm free -> 0.35 % strain (38 AWG): move it to +3.0"],
        [3.0, 6.0, -8.0, "yoke dorsal channel Ø1.4 to the yoke's dorsal face beside the abd disc"],
        [-4.0, 9.0, -13.0, "yoke exit, 4 mm off the abd axis: free loop (abd +-15 deg -> 1.0 mm path change), R >= 8"],
        [0.0, 19.5, -6.0, "knuckle board (palm side): abd chip + JST SH 4-pin socket, dorsal, under the palm cover"]],
    fatigue="see out/15/budget.txt: 38 AWG parallel bundle 0.05-0.17 % strain in every pin (limit 0.20 % for 1e6 cycles, assumed)",
    glue="glue points: a drop of flexible silicone adhesive (Sil-Poxy) or soft epoxy, not CA, so the wire is held without a hard edge",
    replaces_07="07's axle slot box (x -3.6..-2.4, z -2.1..-1.0, at the max-stress fibre) is replaced by a Ø1.2 hole along +y at x +3.0 (neutral axis: I loss < 1 %)")
SPEC["connectors"] = dict(
    finger="JST SH 1.0 mm 4-pin: socket on the knuckle board (palm), plug on the finger pigtail; swap = remove dorsal palm cover (2 x M2), unplug, pull 2 abd pins",
    thumb="JST SH 4-pin at the thumb mount (palm side of the CMC), same scheme",
    palm_pads="JST SH 4-pin x 2 (palm_a, palm_b buses)",
    wrist="JST SH 6-pin at the hub; the forearm end is soldered to the ESP32-side RS-485/UART board (or a second 6-pin)",
    never="no connector inside a finger: every connector joint is a failure point and adds 3-4 mm")
SPEC["notes"] = [
    "Non-magnetic parts near chips: DIP bar pins A/B in 316 or brass (not hardened ISO 8734), abd dorsal stub brass/316, "
    "keeper plugs PETG or brass. Hardened dowels near a chip must be >= 8 mm away (or be measured on the 08 R9 rig).",
    "Chip variants per finger bus: A1 abd, B1 mcp_flex, C1 pip, D1 dip (fixed factory addresses, no reprogramming at boot).",
    "Thumb: A1 cmc_rot, B1 cmc_flex, C1 mcp_flex, D1 ip; same joint module at each axis end opposite the wire exit; the thumb designer places them "
    "(keep >= 20 mm from the index MCP magnet, or calibrate with the index at its working angles).",
    "Width: MCP stack ends at x 8.75 inside the 9.0 lug face (0 added); PIP 8.6 vs 8.05 (+0.55, taper region has >= 3 mm); DIP 8.35 vs 7.5 (+0.85). No sensor boss on the -x side of any finger."]

# ============================================================ write spec, checks, renders
SPEC["checks"] = CHECK
json.dump(SPEC, open(OUT / "sensor_spec.json", "w"), indent=1)
print(json.dumps(CHECK, indent=1))

try:
    import trimesh
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    def render(names, fname, elev=20, azim=-60, title=""):
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(111, projection="3d")
        cols = ["#4f86c6", "#2e7d32", "#555566", "#c8a040", "#e08a3c", "#8e6fbf"]
        allv = []
        for i, n in enumerate(names):
            m = trimesh.load(OUT / f"{n}.stl")
            pc = Poly3DCollection(m.vertices[m.faces], alpha=0.85, facecolor=cols[i % len(cols)], edgecolor="k", linewidths=0.05)
            ax.add_collection3d(pc)
            allv.append(m.vertices)
        v = np.vstack(allv)
        c, r = v.mean(0), (v.max(0) - v.min(0)).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
        ax.view_init(elev, azim)
        ax.set_title(title, fontsize=9)
        fig.savefig(OUT / fname, dpi=110)
        plt.close(fig)

    render(["joint_module_mcp_flex"], "render_joint_module_mcp.png", 25, 30, "MCP joint module R3 slim (cap, chip facing Ø2x1.5 magnet, Ø6.4 board, keeper)")
    render(["joint_module_pip"], "render_joint_module_pip.png", 25, 30, "PIP joint module R3 slim")
    render(["joint_cap_mcp_flex", "joint_cap_pip", "joint_cap_dip", "joint_cap_abd"], "render_caps.png", 20, -50, "joint caps mcp / pip / dip / abd in finger frame")
    render(["tip_assembly_ref"], "render_tip.png", 15, -70, "tip carrier + board + MLX90393 + skin magnet")
    render(["hub_assembly_ref"], "render_hub.png", 35, -60, "palm hub tray + carrier + RP2040-Zero")
    render(["wrist_pin_hollow"], "render_wrist_pin.png", 20, -40, "Ø5 hollow wrist pin, bore 2.4, radial exits")
    render(["pad_assembly_ref"], "render_pad.png", 20, -40, "BMP581 pad cup")
    print("renders done")
except Exception as e:  # noqa: BLE001
    print("render failed:", e)
