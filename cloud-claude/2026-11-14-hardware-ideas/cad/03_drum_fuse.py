"""03 - prototype: MCP drum with a metal ring seat, bead termination and replaceable fuse cups,
plus the cross-drilled 4 mm axle (study 2026-11-14, agent 03).

uv run --no-project --with cadquery --with numpy python cad/03_drum_fuse.py     -> out/03/*.step, *.stl, 03_drum_fuse.json

Frame of this puck: axle along X (joint axis), drum centred at origin, +Y = dorsal, -Y = palm side, +Z distal.
One track (owner's way): the flex strand comes from the palm side (-Y,-Z... below), wraps the ring, the extend strand
leaves on the back; both ends finish in a bead at the "top" (anchor angle 50 deg from +Y toward +Z).
Parts: drum_body (PETG), ring (brass/stainless tube slice, 3 mm wide = two strand lanes), fuse_cup (one oval PETG cup, both beads, replaceable), axle (steel, cross-drilled).
"""
import json
import math
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent.parent / "out" / "03"
OUT.mkdir(parents=True, exist_ok=True)

R_LINE = 7.0            # line centre radius (owner: same distance on each degree)
LINE_D = 0.36
RING_T = 0.5            # ring wall
R_RING_OUT = R_LINE - LINE_D / 2          # 6.82 : line rests on the ring's outer face
R_RING_IN = R_RING_OUT - RING_T           # 6.32
RING_W = 3.0
LANE = 1.35            # strand lane offset on the ring (flex -, ext +)
GROOVE_W = 3.4          # ring + 0.2 each side
R_FLANGE = 8.0
W_DRUM = 9.9            # MCP drum width (index)
AXLE_D = 4.0
ANCHOR = 50.0           # deg from +Y toward +Z

# --- drum body: two flanges + hub, groove for the ring; axle bore; ring seat
body = cq.Workplane("YZ").circle(R_FLANGE).extrude(W_DRUM / 2, both=True)
groove = cq.Workplane("YZ").circle(R_FLANGE + 1).circle(R_RING_IN).extrude(GROOVE_W / 2, both=True)
body = body.cut(groove)
# the groove floor is the ring seat: leave a hub of radius R_RING_IN
bore = cq.Workplane("YZ").circle(AXLE_D / 2 - 0.02).extrude(W_DRUM, both=True)   # reamed to the axle
body = body.cut(bore)

# --- ring (tube slice) with a notch where the line dives into the bead hole
ring = cq.Workplane("YZ").circle(R_RING_OUT).circle(R_RING_IN).extrude(RING_W / 2, both=True)

# --- ONE oval fuse cup holds both bead chambers (x = +-LANE). Two lanes on a wide ring: flex lane x=-LANE, ext lane x=+LANE
a = math.radians(ANCHOR)
uy, uz = math.cos(a), math.sin(a)          # radial unit vector in (Y, Z) at the anchor
hole_d = 0.8
CUP_W, CUP_T, CUP_H = 5.2, 2.7, 2.6        # oval cup: width along x, thickness along the tangent, height along the radius
CHAMBER_D, T_F = 1.8, 0.8                  # bead chamber, fuse lip thickness
pl = cq.Plane(origin=(0, 0, 0), xDir=(1, 0, 0), normal=(0, uy, uz))
r_top = R_RING_IN
def oval(offset, w, t, h):
    return cq.Workplane(pl).workplane(offset=offset).slot2D(w, t, 0).extrude(h)
pocket = oval(r_top - CUP_H, CUP_W + 0.2, CUP_T + 0.2, CUP_H)       # +0.1 per side clearance
body = body.cut(pocket)
cup = oval(r_top - CUP_H, CUP_W, CUP_T, CUP_H)
for sx in (-1, 1):
    ch = cq.Workplane(pl).workplane(offset=r_top - CUP_H - 0.1).center(sx * LANE, 0).circle(CHAMBER_D / 2).extrude(CUP_H - T_F + 0.1)
    cup = cup.cut(ch)
    hl = cq.Workplane(pl).workplane(offset=r_top - T_F - 0.1).center(sx * LANE, 0).circle(hole_d / 2).extrude(T_F + 0.2)
    cup = cup.cut(hl)
    ring = ring.cut(cq.Workplane(pl).workplane(offset=R_RING_IN - 0.1).center(sx * LANE, 0).circle(hole_d / 2).extrude(RING_T + 0.4))
cups = [cup]

# --- axle: 4 mm steel, 15 mm long, 1.2 mm cross-hole at the centre along Z (strands cross the axis here)
axle = cq.Workplane("YZ").circle(AXLE_D / 2).extrude(15.0 / 2, both=True)
axle = axle.cut(cq.Workplane("XY").workplane(offset=-3).circle(0.6).extrude(6.0))
# polish chamfer 0.2 on the cross hole edges (both ends)
try:
    axle = axle.faces("%CYLINDER").edges(cq.selectors.BoxSelector((-1, -3, -3), (1, 3, 3))).chamfer(0.15)
except Exception as e:  # noqa
    print("axle chamfer skipped:", e)

parts = {"drum_body": body, "ring": ring, "fuse_cup": cups[0], "axle": axle}
info = {}
for n, w in parts.items():
    s = w.val()
    info[n] = {"valid": bool(s.isValid()), "volume_mm3": round(s.Volume(), 2),
               "bbox": [round(v, 2) for v in (s.BoundingBox().xlen, s.BoundingBox().ylen, s.BoundingBox().zlen)]}
    cq.exporters.export(w, str(OUT / f"03_{n}.stl"))
asm = cq.Assembly()
for n, w in parts.items():
    asm.add(w, name=n)
asm.save(str(OUT / "03_drum_fuse.step"))
# mass: PETG 1.27 g/cm3, brass 8.5, steel 7.85
mass = {"drum_body": 1.27, "ring": 8.5, "fuse_cup": 1.27, "axle": 7.85}
for n in info:
    info[n]["mass_g"] = round(info[n]["volume_mm3"] * mass[n] / 1000, 3)
# interference check between body and ring/cups/axle (should be ~0 except designed press fits)
inter = {}
for n in ("ring", "fuse_cup", "axle"):
    inter[n] = round(body.intersect(parts[n]).val().Volume(), 3) if body.intersect(parts[n]).vals() else 0.0
info["interference_with_body_mm3"] = inter
Path(OUT / "03_drum_fuse.json").write_text(json.dumps(info, indent=1))
print(json.dumps(info, indent=1))
