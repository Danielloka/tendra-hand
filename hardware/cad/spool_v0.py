"""Tendon spool for the V0 hand's 28BYJ-48 steppers (parametric, writes an STL).

Why: the V0 prototype's spools have a 10 mm radius, which gives only about 3 N of tendon pull.
A 5 mm radius doubles the pull (and halves the speed); see research/log.md 2026-09-29.

The spool has two grooves, one per strand of a joint's pull-pull loop (flex and ext are wound in
opposite directions), so the two strands never ride over each other. Each groove has a tie hole
through its outer flange: thread the strand from the groove through the hole and knot it outside.
The bore is a double-D slide fit on the 28BYJ-48 output shaft (5 mm, flats 3 mm apart).

Run:  uv run --with cadquery-ocp python hardware/cad/spool_v0.py            (5 mm radius)
      uv run --with cadquery-ocp python hardware/cad/spool_v0.py --radius 6
Output: hardware/print/v0/spool_r5.stl (world mm, the shaft along +Z, the bottom face at z = 0).
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.GProp import GProp_GProps
from OCP.StlAPI import StlAPI_Writer

OUT_DIR = Path(__file__).resolve().parents[1] / "print" / "v0"

LINE_R = 0.2  # 0.4 mm fishing line
FLANGE_T = 1.2  # each of the 3 flanges
GROOVE_W = 1.2  # each of the 2 grooves
FLANGE_EXTRA = 2.5  # flange radius = centreline radius + this
SHAFT_D, SHAFT_FLATS = 5.0, 3.0  # 28BYJ-48 output shaft
FIT = 0.15  # added to the bore (PLA holes print slightly small)
TIE_D = 1.0


def _cyl(r: float, z0: float, h: float, x: float = 0.0, y: float = 0.0):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), r, h).Shape()


def _box(x0, y0, z0, dx, dy, dz):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), dx, dy, dz).Shape()


def spool(radius: float = 5.0):
    """Solid spool with the tendon centreline at `radius` (mm)."""
    floor = radius - LINE_R  # groove floor
    flange = radius + FLANGE_EXTRA
    height = 3 * FLANGE_T + 2 * GROOVE_W
    body = _cyl(flange, 0.0, height)
    grooves = []
    for i in range(2):  # grooves between the flanges
        z0 = FLANGE_T + i * (GROOVE_W + FLANGE_T)
        ring = BRepAlgoAPI_Cut(
            _cyl(flange + 1, z0, GROOVE_W), _cyl(floor, z0 - 1, GROOVE_W + 2)
        ).Shape()
        body = BRepAlgoAPI_Cut(body, ring).Shape()
        grooves.append(z0)
    # double-D bore: a circle cut to the width across the flats
    bore_r = (SHAFT_D + FIT) / 2
    flats = SHAFT_FLATS + FIT
    bore = BRepAlgoAPI_Common(
        _cyl(bore_r, -1, height + 2),
        _box(-flats / 2, -bore_r - 1, -1, flats, 2 * bore_r + 2, height + 2),
    ).Shape()
    body = BRepAlgoAPI_Cut(body, bore).Shape()
    # tie holes: through the outer flange next to each groove, just outside the groove floor, on
    # opposite sides so the knots don't meet
    r_tie = floor + TIE_D / 2 + 0.1
    for z0, (sign, flange_z0) in zip(grooves, ((+1, 0.0), (-1, height - FLANGE_T))):
        hole = _cyl(TIE_D / 2, flange_z0 - 0.5, FLANGE_T + GROOVE_W / 2 + 0.5, x=sign * r_tie)
        body = BRepAlgoAPI_Cut(body, hole).Shape()
    body = BRepAlgoAPI_Fuse(body, body).Shape()
    return body, {
        "radius": radius,
        "floor": floor,
        "flange": flange,
        "height": height,
        "bore_r": bore_r,
    }


def volume_mm3(shape) -> float:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props.Mass()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--radius", type=float, default=5.0, help="tendon centreline radius (mm)")
    args = ap.parse_args()
    shape, dims = spool(args.radius)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / f"spool_r{args.radius:g}.stl"
    BRepMesh_IncrementalMesh(shape, 0.02, False, 0.2, True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False  # binary STL
    writer.Write(shape, str(out))
    pull = 10.0 / args.radius
    print(
        f"{out.name}: radius {dims['radius']} mm (groove floor {dims['floor']:.1f}), flanges "
        f"r {dims['flange']:.1f}, height {dims['height']:.1f} mm, volume {volume_mm3(shape):.0f} mm3; "
        f"{pull:.1f}x the pull of the 10 mm spool, "
        f"{math.degrees(1.0 / args.radius):.1f} deg of spool per mm of tendon"
    )


if __name__ == "__main__":
    main()
