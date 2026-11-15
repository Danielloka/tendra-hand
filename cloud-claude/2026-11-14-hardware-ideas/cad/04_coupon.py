"""04_coupon.py - one-plate FDM test coupon for the Tendra Hand (study 2026-11-14, agent 04).

Run:  uv run --no-project --with cadquery --with numpy python cad/04_coupon.py
0.2 mm layers, 2-3 walls, 15 % infill, NO supports, no brim needed. Orientation matters:
  - vertical holes (axis Z) are the "good" case: perfect circles.
  - horizontal holes (axis X or Y) are the "bad" case and are printed as round, teardrop (apex up) or diamond
    to show which one the real parts should use.
Every hole size is a real number you can read off with pins / drill bits / calipers; see coupon_legend.json.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent.parent / "out" / "04"
OUT.mkdir(parents=True, exist_ok=True)

PW, PD, PT = 140.0, 98.0, 0.6  # plate width (x), depth (y), thickness
LEGEND: dict = {}
SOLIDS: list[cq.Workplane] = []   # added material
CUTS: list[cq.Workplane] = []     # removed through everything


def box(x0, y0, w, d, h, z0=0.0):
    return cq.Workplane("XY").box(w, d, h, centered=False).translate((x0, y0, z0))


def vhole(x, y, dia, z_from, z_to):
    """vertical (axis Z) cylinder cutter between two heights."""
    return cq.Workplane("XY").circle(dia / 2).extrude(z_to - z_from).translate((x, y, z_from))


def profile(kind: str, d: float):
    """2D tunnel profile on the current workplane: round, teardrop (apex +local y) or diamond."""
    r = d / 2
    wp = cq.Workplane("XY")  # placeholder, real plane set by caller via transform
    return r


def tunnel_x(x0, x1, y, z, d, kind):
    """horizontal tunnel along +X, centre (y, z). Profile drawn in the YZ plane."""
    r = d / 2
    wp = cq.Workplane("YZ", origin=(x0, y, z))
    if kind == "round":
        s = wp.circle(r).extrude(x1 - x0)
    elif kind == "teardrop":  # apex up, 45 degree flanks: prints without supports
        c = wp.circle(r).extrude(x1 - x0)
        t = (cq.Workplane("YZ", origin=(x0, y, z))
             .polyline([(0, r * math.sqrt(2)), (-r / math.sqrt(2), r / math.sqrt(2)),
                        (r / math.sqrt(2), r / math.sqrt(2))]).close().extrude(x1 - x0))
        s = c.union(t)
    elif kind == "diamond":
        k = r * math.sqrt(2)
        s = wp.polyline([(0, k), (k, 0), (0, -k), (-k, 0)]).close().extrude(x1 - x0)
    else:
        raise ValueError(kind)
    return s


def hole_y(x, z, y_front, depth, d, kind):
    """horizontal hole along +Y starting at y_front (front face), `depth` deep. Profile in the XZ plane."""
    r = d / 2
    wp = cq.Workplane("XZ", origin=(x, y_front, z))  # normal is -Y, so extrude(-depth) goes +Y
    c = wp.circle(r).extrude(-depth)
    if kind == "round":
        return c
    t = (cq.Workplane("XZ", origin=(x, y_front, z))
         .polyline([(0, r * math.sqrt(2)), (-r / math.sqrt(2), r / math.sqrt(2)),
                    (r / math.sqrt(2), r / math.sqrt(2))]).close().extrude(-depth))
    return c.union(t)


def label(x, y, text, size=2.6, z0=PT, h=0.4):
    """raised text on the plate (prints better than engraved text with a 0.4 nozzle)."""
    return cq.Workplane("XY", origin=(x, y, z0)).text(text, size, h, combine=False, kind="bold",
                                                       halign="left", valign="bottom")


# --- plate -----------------------------------------------------------------------------------------
SOLIDS.append(box(0, 0, PW, PD, PT))

# --- 1. pin / press-fit ladder: nominal 2, 3, 4 mm; offsets in mm ----------------------------------
DELTAS = [-0.10, -0.05, 0.00, 0.05, 0.10, 0.20]
BH1 = 5.0
SOLIDS.append(box(8, 5, 42, 23, BH1))
for r, nom in enumerate([2.0, 3.0, 4.0]):
    for c, dl in enumerate(DELTAS):
        x, y = 11.5 + 7 * c, 9.5 + 7 * r
        CUTS.append(vhole(x, y, nom + dl, -1, BH1 + 1))
        LEGEND[f"pin_{int(nom)}mm_col{c + 1}"] = {"x": x, "y": y, "dia": round(nom + dl, 3),
                                                  "use": "steel dowel: -0.10/-0.05 = press, 0 = snug, +0.05/+0.10 = slide, +0.20 = loose"}
    SOLIDS.append(label(1.0, 8.2 + 7 * r, f"{int(nom)}", 3.2))
for c, dl in enumerate(DELTAS):
    SOLIDS.append(label(9.0 + 7 * c, 29.5, ("+" if dl > 0 else "") + f"{int(round(dl * 100))}", 2.2))

# --- 2. bearing seats MR84ZZ (OD 8, 3 mm wide), vertical axis: 7.90 .. 8.10 -------------------------
BH2 = 6.0
SOLIDS.append(box(56, 5, 58, 12, BH2))
for i, d in enumerate([7.90, 7.95, 8.00, 8.05, 8.10]):
    x, y = 61 + 11 * i, 11
    CUTS.append(vhole(x, y, d, BH2 - 3.0, BH2 + 1))        # seat, 3.0 deep, open on top
    CUTS.append(vhole(x, y, 6.75, -1, BH2))                # shoulder bore stops the outer race only
    LEGEND[f"mr84_seat_vertical_{i + 1}"] = {"x": x, "y": y, "seat_dia": d, "depth": 3.0, "shoulder_bore": 6.75,
                                            "use": "press an MR84ZZ in by hand/vice; note force and whether it falls out after 10 min"}
    SOLIDS.append(label(x - 3.0, 18.5, f"{d:.2f}", 2.0))

# MR63ZZ (OD 6, 2.5 wide)
BH3 = 5.0
SOLIDS.append(box(56, 24, 40, 10, BH3))
for i, d in enumerate([5.90, 5.95, 6.00, 6.05]):
    x, y = 61 + 10 * i, 29
    CUTS.append(vhole(x, y, d, BH3 - 2.5, BH3 + 1))
    CUTS.append(vhole(x, y, 4.9, -1, BH3))
    LEGEND[f"mr63_seat_vertical_{i + 1}"] = {"x": x, "y": y, "seat_dia": d, "depth": 2.5, "shoulder_bore": 4.9}
    SOLIDS.append(label(x - 3.0, 35.0, f"{d:.2f}", 2.0))

# horizontal-axis bearing seats in an upright wall (the "bad" orientation): round vs teardrop
WALL_Y0, WALL_T, WALL_H = 44.0, 6.0, 12.0
SOLIDS.append(box(56, WALL_Y0, 48, WALL_T, WALL_H))
for i, (d, kind) in enumerate([(7.95, "round"), (8.05, "round"), (7.95, "teardrop"), (8.05, "teardrop")]):
    x = 62 + 12 * i
    z = 6.0
    CUTS.append(hole_y(x, z, WALL_Y0 - 0.01, 3.0, d, kind))           # seat from the front face, 3.0 deep
    CUTS.append(hole_y(x, z, WALL_Y0 - 1, WALL_T + 2, 6.75, "round"))  # shoulder bore through
    LEGEND[f"mr84_seat_horizontal_{i + 1}"] = {"x": x, "z": z, "seat_dia": d, "profile": kind,
                                              "use": "same bearing: compare ovality and press force with the vertical seat"}
    SOLIDS.append(label(x - 3.0, 39.0, f"{d:.2f}" + ("r" if kind == "round" else "t"), 2.0))

# --- 3. inserts, screws, liners ---------------------------------------------------------------------
BH4 = 6.0
SOLIDS.append(box(8, 35, 46, 55, BH4))
rows = [
    ("A", "M2 heat-set insert (OD ~3.2) hole", [3.0, 3.1, 3.2, 3.3], 4.0, "insert"),
    ("B", "M2.5 heat-set insert (OD ~3.6) hole", [3.4, 3.5, 3.6, 3.7], 4.5, "insert"),
    ("C", "M3 heat-set insert (OD ~4.5) hole", [4.0, 4.1, 4.2, 4.3], 5.0, "insert"),
    ("D", "M2: self-tap pilots 1.6/1.7/1.8 then clearance 2.2/2.4", [1.6, 1.7, 1.8, 2.2, 2.4], 4.0, "screw"),
    ("E", "M2.5: self-tap pilots 2.1/2.2/2.3 then clearance 2.7/2.9", [2.1, 2.2, 2.3, 2.7, 2.9], 4.0, "screw"),
    ("F", "M3: self-tap pilots 2.5/2.6/2.7 then clearance 3.2/3.4", [2.5, 2.6, 2.7, 3.2, 3.4], 4.0, "screw"),
    ("G", "16G stainless tie-hole liner (OD 1.65): 1.60/1.65/1.70/1.75", [1.60, 1.65, 1.70, 1.75], 6.0, "liner"),
]
for r, (tag, desc, dias, depth, kind) in enumerate(rows):
    y = 40 + 7.5 * r
    SOLIDS.append(label(1.0, y - 1.4, tag, 3.2))
    for c, d in enumerate(dias):
        x = 13 + 8 * c
        CUTS.append(vhole(x, y, d, BH4 - depth, BH4 + 1))
        LEGEND[f"row_{tag}_col{c + 1}"] = {"x": x, "y": y, "dia": d, "depth": depth, "row": desc}

# --- 4. channels: straight tunnels, vertical holes, curved tunnel, snap-in groove ---------------------
BH5 = 6.5
SOLIDS.append(box(58, 56, 26, 28, BH5))
for i, (d, kind) in enumerate([(2.2, "round"), (2.2, "teardrop"), (2.2, "diamond"), (1.2, "teardrop"), (1.2, "round")]):
    y = 59.5 + 5.5 * i
    CUTS.append(tunnel_x(57, 85, y, 3.9, d, kind))
    LEGEND[f"tunnel_x_{i + 1}"] = {"y": y, "dia": d, "profile": kind, "length": 26,
                                  "use": "push 2.0 OD PTFE tube (d=2.2) or 0.36 line / 1 mm wire (d=1.2) through; does it pass freely?"}
# tower with vertical holes (axis Z, 14 mm long)
SOLIDS.append(box(87, 57, 6, 17, 10))
CUTS.append(vhole(90, 61.5, 2.2, -1, 11))
CUTS.append(vhole(90, 69.5, 1.2, -1, 11))
LEGEND["tower_vertical_2.2"] = {"x": 90, "y": 61.5, "dia": 2.2, "length": 10}
LEGEND["tower_vertical_1.2"] = {"x": 90, "y": 69.5, "dia": 1.2, "length": 10}


def arc_band(cx, cy, r_in, r_out, h, q):
    """quarter ring; q = (sx, sy) picks the quadrant (+1/-1)."""
    ring = cq.Workplane("XY").circle(r_out).circle(r_in).extrude(h)
    cut = box(-r_out - 1 if q[0] < 0 else 0, -r_out - 1 if q[1] < 0 else 0, r_out + 1, r_out + 1, h + 2, z0=-1)
    return ring.intersect(cut).translate((cx, cy, 0))


R = 15.0
# curved teardrop tunnel R15 in the build plane (a Bowden bend), centre (97, 57), quadrant +x +y
band1 = arc_band(97, 57, 11.5, 18.5, BH5, (1, 1))
SOLIDS.append(band1)
for k, (d, kind) in enumerate([(2.2, "teardrop")]):
    r = d / 2
    prof = (cq.Workplane("XZ", origin=(97, 57, 3.9)).center(R, 0))
    c = prof.circle(r).revolve(90, (-R, 0, 0), (-R, 1, 0))
    t = (cq.Workplane("XZ", origin=(97, 57, 3.9)).center(R, 0)
         .polyline([(0, r * math.sqrt(2)), (-r / math.sqrt(2), r / math.sqrt(2)), (r / math.sqrt(2), r / math.sqrt(2))])
         .close().revolve(90, (-R, 0, 0), (-R, 1, 0)))
    tun = c.union(t)
    bb = tun.val().BoundingBox()
    if bb.ymax < 57 + 5:  # swept the wrong way: mirror about the XZ plane through the centre
        tun = tun.mirror("XZ", (97, 57, 0))
    CUTS.append(tun)
LEGEND["curved_tunnel_R15"] = {"centre": [97, 57], "R": R, "dia": 2.2, "profile": "teardrop", "angle": 90,
                              "use": "push 2.0 OD PTFE tube through the 90 degree bend; also a 1.0 mm steel wire"}

# snap-in keyhole groove for a 2.0 OD tube: circle Ø2.05 at depth + mouth slot (open on top)
GROOVE_D = 2.05


def keyhole_straight(x0, x1, y, mouth, zc):
    c = cq.Workplane("YZ", origin=(x0, y, zc)).circle(GROOVE_D / 2).extrude(x1 - x0)
    s = box(x0, y - mouth / 2, x1 - x0, mouth, BH5 + 1 - zc, z0=zc)
    return c.union(s)


SOLIDS.append(box(58, 86, 28, 11, BH5))
for i, mouth in enumerate([1.6, 1.8]):
    y = 89.5 + 5.0 * i
    CUTS.append(keyhole_straight(57, 87, y, mouth, 3.9))
    LEGEND[f"snap_groove_straight_{i + 1}"] = {"y": y, "mouth": mouth, "circle": GROOVE_D, "length": 28,
                                              "use": "press a 2.0 OD PTFE tube in with a thumb: snap or crush/rock out? mouth 1.6 holds, 1.8 may fall out"}

# curved snap groove R15, centre (114, 98), quadrant -x -y
band2 = arc_band(114, 98.0, 11.5, 18.5, BH5, (-1, -1))
SOLIDS.append(band2)
ring_c = cq.Workplane("XY").circle(R + GROOVE_D / 2).circle(R - GROOVE_D / 2).extrude(GROOVE_D).translate((0, 0, 3.9 - GROOVE_D / 2))
ring_m = cq.Workplane("XY").circle(R + 0.85).circle(R - 0.85).extrude(BH5 + 1 - 3.9).translate((0, 0, 3.9))
quad = box(-R - 5, -R - 5, R + 6, R + 6, BH5 + 3, z0=-1)
CUTS.append(ring_c.union(ring_m).intersect(quad).translate((114, 98.0, 0)))
LEGEND["snap_groove_curved_R15"] = {"centre": [114, 98], "R": R, "mouth": 1.7, "circle": GROOVE_D,
                                   "use": "press the PTFE tube along the whole curve; does it hold when you turn the plate upside down?"}

# --- 5. extras: drum with grooves + radial tie hole, thin-wall ladder, strength bars -------------------
DR, DH = 7.0, 7.0
drum = cq.Workplane("XY").circle(DR).extrude(DH).translate((129, 12, 0))
SOLIDS.append(drum)
for i, w in enumerate([0.8, 1.0, 1.2]):
    zc = 1.7 + 2.0 * i
    CUTS.append(cq.Workplane("XY").circle(DR + 1).circle(DR - 0.5).extrude(w).translate((129, 12, zc)))
    LEGEND[f"drum_groove_{i + 1}"] = {"width": w, "depth": 0.5, "z": zc, "use": "does a 0.36 braid sit in it? 7 mm drum radius = real drum"}
CUTS.append(vhole(129, 12, 3.0, -1, DH + 1))  # centre bore for a 3 mm axle
CUTS.append(tunnel_x(129 - DR - 1, 129 + 0.0, 12, 3.5, 1.7, "teardrop").translate((0, 0, 0)))  # radial tie hole, teardrop
LEGEND["drum_radial_tie_hole"] = {"dia": 1.7, "profile": "teardrop", "use": "does a 16G liner (OD 1.65) push in? does the knot seat?"}

# thin-wall ladder (fins), height 10
FIN = [0.4, 0.8, 1.2, 1.6, 2.0]
x = 118.0
for t in FIN:
    SOLIDS.append(box(x, 26, t, 10, 8))
    LEGEND[f"fin_{t}"] = {"thickness": t, "height": 10, "length": 10,
                          "use": "0.4 = single wall; 1.2 = 3 walls at 0.4; need >= 1.6 for a load-bearing wall"}
    x += t + 3.0
SOLIDS.append(box(118, 24, 16.5, 1.0, 1.0, z0=PT))  # tie bar so the fins print as one piece

# strength bars: separate pieces on the bed. flat = loads in layers; upright = loads across layers
SOLIDS.append(box(118, 42, 24, 3, 2, z0=0).translate((0, 0, 0)))  # flat bar (XY): 3 wide x 2 tall x 24 long
SOLIDS.append(box(118, 48, 24, 3, 2, z0=0))
# plate under them would weld them: cut the plate around (done below with PLATE_CUT)
for j, y in enumerate([56, 66]):
    SOLIDS.append(box(118, y, 8, 8, 0.6))                  # foot
    SOLIDS.append(box(118 + 2.5, y + 2.75, 3, 2, 12, z0=0.6))  # upright bar 3x2x16 (Z)
LEGEND["bars"] = {"flat_XY": [[118, 42], [118, 48]], "upright_Z": [[118, 56], [118, 66]],
                  "use": "clamp the foot/plate, bend the bar until it breaks; the Z bar breaks along a layer line at a fraction of the force "
                         "of the flat bar. Record the angle at break and whether it split on a layer."}

# --- assemble ----------------------------------------------------------------------------------------
PLATE_HOLES = [box(116, 40, 28, 16, 3, z0=-1)]  # no plate under the flat bars and feet: they print as loose pieces

body = SOLIDS[0]
for s in SOLIDS[1:]:
    body = body.union(s)
# loose-bar zone: remove the plate there, then re-add the bars on the bed
loose = [box(118, 42, 24, 3, 2), box(118, 48, 24, 3, 2)]
for j, y in enumerate([56, 66]):
    loose.append(box(118, y, 8, 8, 0.6))
    loose.append(box(118 + 2.5, y + 2.75, 3, 2, 12, z0=0.6))
body = body.cut(box(116, 40, 28, 38, 30, z0=-1))
for s in loose:
    body = body.union(s)
for c in CUTS:
    body = body.cut(c)

vol = body.val().Volume() / 1000.0
bb = body.val().BoundingBox()
print(f"coupon volume {vol:.2f} cm3 (~{vol * 1.27:.1f} g solid), bbox {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}; valid={body.val().isValid()}")

cq.exporters.export(body, str(OUT / "tendra_coupon.stl"), tolerance=0.02, angularTolerance=0.3)
cq.exporters.export(body, str(OUT / "tendra_coupon.step"))
LEGEND["_meta"] = {"plate": [PW, PD, PT], "volume_cm3": round(vol, 2), "print": "PETG 0.2 mm layers, 0.4 nozzle, 3 walls, 15 % infill, no supports"}
(OUT / "coupon_legend.json").write_text(json.dumps(LEGEND, indent=1))
