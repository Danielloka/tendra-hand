"""Agent 02: prototype of the 'spool cartridge' (report 02-drivetrain.md, deep dive D3).

    uv run --no-project --with cadquery --with numpy python cad/02_spool_cartridge.py

One-piece spool (no split, no friction clamp) that
  - turns on 2 x MR63ZZ on a fixed Ø3 steel axle cantilevered from the spool rail, so the servo only
    gives torque (it drives through two dog pins with 0.3 mm radial float);
  - has two flat-bottom lands (ext on the rail side, flex on the hub side), each 1.6 mm wide for
    up to ~2 turns of PE#8 braid (0.47 mm), floor radius = R_SPOOL - d/2 so the line centre sits at R 5;
  - anchors the ext strand with a stopper (figure-8 + CA, or a crimp sleeve) in a radial pocket in its
    land floor, under >= 1 turn of line (the stopper sees ~28 % of the tension);
  - tensions the loop with a chordal M2 screw: the flex strand goes through the hub-side flange into the
    hub and is tied to an M2 square nut that the screw pulls along a chord (0.4 mm of line per turn,
    5.5 mm of travel, self-locking, reachable from the rim after the servo turns the spool to a service angle);
  - has a witness notch on the hub rim, and a separate keeper ring (0.2 mm gap at the flanges) so a slack
    line cannot jump off.

Frame: spool axis = +X, x = 0 at the rail face; strands leave toward +Z (the wrist) at y = +-R_SPOOL.
Outputs: out/02/spool_cartridge.step (assembly), STL per printed part, spool_cartridge.json (sizes, checks).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "out" / "02"
OUT.mkdir(parents=True, exist_ok=True)

R_SPOOL = 5.0
LINE_D = 0.47  # PE#8 UHMWPE braid
FLOOR_R = R_SPOOL - LINE_D / 2  # 4.765
FLANGE_R = 6.5
LAND_W = 1.6
FL_W = 0.8  # outer flange / divider thickness
MR63 = {"id": 3.0, "od": 6.0, "w": 2.5}
SEAT_D = 5.95  # light press in PETG (tune with a coupon)
AXLE_D = 3.0
HUB_L = 6.0  # hub with the chordal tensioner (the M2 square nut is 4 x 4 across, 1.6 along the screw)
CHORD_H = 3.7  # chord distance from the axis (screw axis along Y at z = CHORD_H)
NUT = {"s": 4.0, "m": 1.6}  # DIN 562 M2 square nut (s 4.0, m 1.2-1.6; slot sized for 1.6)
DOG_R, DOG_D, FLOAT = 4.6, 1.5, 0.3

# axial stations
x_f0 = (0.0, FL_W)  # rail-side flange
x_ext = (x_f0[1], x_f0[1] + LAND_W)
x_div = (x_ext[1], x_ext[1] + FL_W)
x_flex = (x_div[1], x_div[1] + LAND_W)
x_f1 = (x_flex[1], x_flex[1] + FL_W)
x_hub = (x_f1[1], x_f1[1] + HUB_L)
L_SPOOL = x_hub[1]


def xcyl(r, x0, x1, y=0.0, z=0.0):
    return cq.Workplane("YZ").workplane(offset=x0).center(y, z).circle(r).extrude(x1 - x0)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def ycyl(r, y0, y1, x, z):
    return cq.Workplane("XZ").workplane(offset=-y1).center(x, z).circle(r).extrude(y1 - y0)


def spool():
    s = xcyl(FLANGE_R, 0, x_f1[1])
    for x0, x1 in (x_ext, x_flex):  # lands
        s = s.cut(xcyl(FLANGE_R + 1, x0, x1).cut(xcyl(FLOOR_R, x0 - 0.1, x1 + 0.1)))
    s = s.union(xcyl(FLANGE_R, x_hub[0], x_hub[1]))
    # bearing bore from the rail side: 2 x MR63 (2.5) + 0.5 shim between the inner races = 5.5
    bore_end = 2 * MR63["w"] + 0.5
    s = s.cut(xcyl(SEAT_D / 2, -0.1, bore_end))
    s = s.cut(xcyl(AXLE_D / 2 + 0.4, bore_end - 0.1, bore_end + 0.8))  # axle tip clearance
    # chamfer-like edge break on all flange rims is left to the slicer (0.2 mm); line entry is tangential
    # ext stopper pocket: a tangential slot in the ext land floor at the bottom (-Z) of the spool, 1.4 wide
    # (axially, inside the 1.6 land), 1.0 deep, 3.5 long: a crimp sleeve (Ø1.3 x 3) or a figure-8 + CA lies in it,
    # flush with the floor, and the first turn of line covers it
    s = s.cut(box(x_ext[0] + 0.1, x_ext[1] - 0.1, -1.75, 1.75, -FLOOR_R - 0.1, -FLOOR_R + 1.0))
    # chordal tensioner in the hub: screw axis along Y at z = CHORD_H, x mid-hub. Head at the +Y rim; the flex
    # strand pulls the nut toward -Y (away from the head), so the screw is in tension and the head bears on its
    # counterbore floor. Turning the screw in draws the nut toward the head = takes up line.
    xc = (x_hub[0] + x_hub[1]) / 2
    half = math.sqrt(FLANGE_R**2 - CHORD_H**2)
    s = s.cut(ycyl(1.1, -half - 1, half + 1, xc, CHORD_H))  # Ø2.2 clearance for the M2 screw
    s = s.cut(ycyl(2.0, half - 2.0, half + 1, xc, CHORD_H))  # head counterbore Ø4 at the +Y end
    # nut slot (nut 4.0 x 4.0 across in x-z, 1.6 along y): nut centre travels y -3.0 ... +2.5 (5.5 mm of line)
    s = s.cut(box(xc - NUT["s"] / 2 - 0.1, xc + NUT["s"] / 2 + 0.1, -3.9, half - 2.0,
                  CHORD_H - NUT["s"] / 2 - 0.1, CHORD_H + NUT["s"] / 2 + 0.1))
    # flex pass hole: Ø1.0 axial from the end of the flex land (r 4.6, just under the line) through the
    # hub-side flange into the nut slot at its -Y end
    s = s.cut(xcyl(0.5, x_flex[1] - 0.5, xc, y=-3.4, z=math.sqrt(4.6**2 - 3.4**2)))
    # dog slots in the hub's servo face (radial slots, so the drive pins float radially by FLOAT)
    for ang in (0, 180):
        a = math.radians(ang)
        cy, cz = DOG_R * math.cos(a), DOG_R * math.sin(a)
        slot = (cq.Workplane("YZ").workplane(offset=x_hub[1] - 1.6)
                .center(cy, cz).slot2D(DOG_D + 2 * FLOAT + 0.1, DOG_D + 0.1, angle=ang).extrude(1.7))
        s = s.cut(slot)
    # witness notch on the hub rim (at +Z when the loop is at q = 0)
    s = s.cut(box(x_hub[1] - 0.8, x_hub[1] + 0.1, -0.3, 0.3, FLANGE_R - 0.5, FLANGE_R + 0.1))
    return s


def driver():
    """Disc on the servo spline (20T, OD 3.95) held by the horn screw; two Ø1.5 x 4 steel dog pins."""
    x0 = L_SPOOL + 0.3
    d = xcyl(FLANGE_R - 0.3, x0, x0 + 2.4)
    d = d.cut(xcyl(1.975 + 0.05, x0 + 0.6, x0 + 2.5))  # spline bore (print a hair under; or bond the horn)
    d = d.cut(xcyl(1.1, x0 - 0.1, x0 + 0.7))  # horn screw M2 passes toward the servo
    for ang in (0, 180):
        a = math.radians(ang)
        d = d.cut(xcyl(DOG_D / 2 - 0.05, x0 - 0.1, x0 + 2.5, y=DOG_R * math.cos(a), z=DOG_R * math.sin(a)))
    return d


def dog_pins():
    x0 = L_SPOOL + 0.3
    return [xcyl(DOG_D / 2, x0 - 1.5, x0 + 2.4, y=DOG_R * math.cos(math.radians(a)), z=DOG_R * math.sin(math.radians(a)))
            for a in (0, 180)]


def keeper():
    """Fixed ring around the flanges, 0.2 mm radial gap, with two windows where the strands leave tangentially
    (+Z at y = +-R_SPOOL). Screws to the cassette; printed in PETG."""
    k = xcyl(FLANGE_R + 2.0, -0.2, x_f1[1] + 0.2).cut(xcyl(FLANGE_R + 0.2, -0.3, x_f1[1] + 0.3))
    for y in (R_SPOOL, -R_SPOOL):
        k = k.cut(box(x_ext[0] - 0.1, x_flex[1] + 0.1, y - 1.0, y + 1.0, 0, FLANGE_R + 3))
    # index mark (a rib) to read against the witness notch
    k = k.union(box(x_f1[1] - 0.6, x_f1[1] + 0.2, -0.25, 0.25, FLANGE_R + 1.6, FLANGE_R + 2.6))
    # mounting ears with M2 holes
    for z in (-(FLANGE_R + 3.0),):
        k = k.union(box(-0.2, x_f1[1] + 0.2, -6, 6, z - 1.5, z + 1.5).cut(
            cq.Workplane("XY").workplane(offset=z - 2).center(x_f1[1] / 2, 4.2).circle(1.1).extrude(4)).cut(
            cq.Workplane("XY").workplane(offset=z - 2).center(x_f1[1] / 2, -4.2).circle(1.1).extrude(4)))
    return k


def hardware():
    hw = {}
    bore_end = 2 * MR63["w"] + 0.5
    for i, x0 in enumerate((0.0, MR63["w"] + 0.5)):
        hw[f"MR63ZZ_{i+1}"] = xcyl(MR63["od"] / 2, x0, x0 + MR63["w"]).cut(xcyl(MR63["id"] / 2, x0 - 1, x0 + 4))
    hw["axle_3x10_ISO8734"] = xcyl(AXLE_D / 2, -4.5, bore_end - 0.2)  # 4.5 mm pressed in the rail
    xc = (x_hub[0] + x_hub[1]) / 2
    half = math.sqrt(FLANGE_R**2 - CHORD_H**2)
    hw["M2x10_screw"] = ycyl(1.0, half - 2.0 - 10, half - 2.0, xc, CHORD_H).union(
        ycyl(1.9, half - 2.0, half - 0.4, xc, CHORD_H))  # DIN 912 M2 head Ø3.8 x 2 (sits 0.4 under the rim)
    hw["M2_square_nut"] = box(xc - 2, xc + 2, -3.8, -2.2, CHORD_H - 2, CHORD_H + 2).cut(
        ycyl(0.9, -3.9, -2.1, xc, CHORD_H))  # shown at the slack end of its travel
    for i, p in enumerate(dog_pins()):
        hw[f"dog_pin_{i+1}"] = p
    return hw


def main():
    parts = {"spool_02": spool(), "driver_02": driver(), "keeper_02": keeper()}
    hw = hardware()
    asm = cq.Assembly(name="spool_cartridge_02")
    colours = {"spool_02": "orange", "driver_02": "gray", "keeper_02": "blue"}
    for n, p in parts.items():
        asm.add(p, name=n, color=cq.Color(colours[n]))
        cq.exporters.export(p, str(OUT / f"{n}.stl"), tolerance=0.005, angularTolerance=0.05)
    for n, p in hw.items():
        asm.add(p, name=n, color=cq.Color(0.75, 0.75, 0.78))
    asm.save(str(OUT / "spool_cartridge.step"))
    info = {}
    for n, p in {**parts, **hw}.items():
        sol = p.val()
        bb = sol.BoundingBox()
        info[n] = {"valid": sol.isValid(), "volume_mm3": round(sol.Volume(), 2),
                   "bbox": [round(v, 2) for v in (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax)]}
    # interference checks between the moving spool and fixed / mating parts
    checks = {}
    for a, b in (("spool_02", "keeper_02"), ("spool_02", "driver_02"), ("spool_02", "M2_square_nut"),
                 ("spool_02", "axle_3x10_ISO8734"), ("spool_02", "M2x10_screw")):
        A = parts.get(a, hw.get(a)).val()
        B = parts.get(b, hw.get(b)).val()
        checks[f"{a}|{b}"] = round(A.intersect(B).Volume(), 3)
    walls = {
        "bearing seat to land floor (radial)": round(FLOOR_R - SEAT_D / 2, 2),
        "flange height above line top": round(FLANGE_R - (R_SPOOL + LINE_D / 2), 2),
        "keeper radial gap": 0.2,
        "stopper pocket floor to bearing seat": round(FLOOR_R - 1.0 - SEAT_D / 2, 2),
        "nut slot outer face to hub rim at chord centre": round(FLANGE_R - (CHORD_H + NUT["s"] / 2 + 0.1), 2),
        "tensioner travel (mm of line)": 5.5,
    }
    info["_checks_overlap_mm3"] = checks
    info["_walls_mm"] = walls
    info["_stations_x"] = {"rail_flange": x_f0, "ext_land": x_ext, "divider": x_div, "flex_land": x_flex,
                           "hub_flange": x_f1, "hub": x_hub, "length": L_SPOOL}
    (OUT / "spool_cartridge.json").write_text(json.dumps(info, indent=1))
    print(json.dumps({k: v for k, v in info.items() if k.startswith("_")}, indent=1))
    for n in parts:
        print(n, info[n])


if __name__ == "__main__":
    main()
