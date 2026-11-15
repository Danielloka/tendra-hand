"""05 · sensor-ready fingertip for the V2 index (distal phalanx tip), snap-in silicone skin, and a 3D-printable skin mould.
Frame = finger-local frame of cloud-claude/2026-11-09-v2-hand-cad/cad/finger.py: origin on the DIP axis,
x = width (thumb side +), y = thickness (palm face = -y), z along the finger (tip at +23).
Tip body = the part of the distal phalanx distal of the DIP boss (z 7.85..23). The fork/cheeks (z < 7.85) are NOT modelled.
s = depth measured from the tip's palm face, positive OUTWARD (toward the object): y = -5.8 - s.
Run: uv run --no-project --with cadquery --with numpy python cad/05_fingertip.py   -> out/05/*.step, *.stl, summary
"""
import json, os, math
import cadquery as cq

OUT = os.path.join(os.path.dirname(__file__), "..", "out", "05")
os.makedirs(OUT, exist_ok=True)

# ---- numbers taken from the V2 index finger (finger.py): W=18, H=16, hwt=0.92*9, r_boss=7.85, zt=23, PAD_T=2.2
HWT = 8.28          # half width of the tip
YF, YB = -5.8, 8.0  # palm face (after the 2.2 mm pad pocket) and dorsal face
Z0, ZT = 7.85, 23.0 # body starts at the DIP boss tangent, tip end
R_TIP = 4.5         # plan-view corner radius of the distal end (V2 used 6.0)
# ---- sensor cavity (stepped, with 45 deg undercut = overhang printable face-up)
D_CAV = 4.8         # cavity depth from the face (PCB floor at s = -4.8)
OPEN_HW, UNDER_HW, UNDER_DEPTH = 4.8, 6.4, 1.6   # opening half width, widest half width, 45 deg bite
CZ0, CZ1 = 10.0, 19.5                            # cavity z range (wall at the rounded tip >= 1.7 mm)
PCB = dict(w=9.2, l=9.0, t=0.8, z0=10.3)         # 1 x MLX90393 (3x3x1.0 QFN) + 0402 cap + 4 pads on the proximal edge
CHIP_H = 1.0
BORE_D, BORE_S = 2.6, -4.4                       # cable bore (z-direction) from the cavity back toward the DIP boss
# ---- skin
SKIN_T = 3.0                                     # outer surface at s = +3.0 (V2 pad was 3.0 as well: same envelope)
SKIN_HW, SKIN_Z0, SKIN_Z1 = 7.7, 9.0, 22.4
CROWN_R = 25.0
PLUG_BOTTOM_S = -2.3                             # plug bottom (0.7 mm above chip top at s = -3.0)
MAG_D, MAG_L = 3.0, 1.5                          # N45 disc, axially magnetised (poles toward the chip / object)
MAG_BOTTOM_S = 0.9                               # magnet bottom; top at 2.4 -> 0.6 mm silicone over it
MAG_HOLE_D = 2.8                                 # press-fit hole in the silicone (magnet stretches it)
MAG_Z = 15.0
# ---- nail / scoop shim (stainless 0.15 x 9 x 6) in a slot in the distal end
SHIM = dict(w=9.0, t=0.15, depth=5.0, protrude=1.5)

def y_of(s): return YF - s
def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
def cyl_z(x, y, r, z0, z1):
    return cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(r).extrude(z1 - z0)
def prism_z(pts, z0, z1):
    return cq.Workplane("XY").workplane(offset=z0).polyline(pts).close().extrude(z1 - z0)
def cyl_y(x, z, r, y0, y1):
    return cq.Workplane("XZ").workplane(offset=-y0).center(x, z).circle(r).extrude(-(y1 - y0))  # XZ normal is -y

def cavity_profile(hw_open=OPEN_HW, hw_under=UNDER_HW, grow=0.0):
    """x-y polygon of the undercut cavity (opening at the palm face, floor at s=-D_CAV)."""
    ys = [y_of(0.0), y_of(-UNDER_DEPTH), y_of(-D_CAV)]
    return [(-hw_open, ys[0]), (hw_open, ys[0]), (hw_under, ys[1]), (hw_under, ys[2]), (-hw_under, ys[2]), (-hw_under, ys[1])]

# ================= tip body =================
body = box(-HWT, HWT, YF, YB, Z0, ZT)
body = body.edges("|Y and >Z").fillet(R_TIP)           # round distal plan corners
try: body = body.faces(">Y").edges().fillet(2.0)        # dorsal (nail side) round-over
except Exception: pass
cav = prism_z(cavity_profile(), CZ0, CZ1)
bore = cyl_z(0, y_of(BORE_S), BORE_D / 2, Z0 - 0.5, CZ0 + 0.1)
slot_up = box(-1.8, 1.8, y_of(BORE_S) - 1.3, y_of(BORE_S) + 1.3, Z0 - 0.5, CZ0 + 0.1)  # square entry so the PCB tail lays flat
shim_slot = box(-SHIM["w"] / 2, SHIM["w"] / 2, YF - 0.0, YF + SHIM["t"] + 0.1, ZT - SHIM["depth"], ZT + 0.1)
tip = body.cut(cav).cut(bore).cut(shim_slot)
tip_valid = tip.val().isValid()

# ================= reference parts =================
pcb_y0 = y_of(-D_CAV) - 0.0   # PCB bottom touches the floor (y larger = deeper)
pcb = box(-PCB["w"] / 2, PCB["w"] / 2, y_of(-D_CAV) - PCB["t"], y_of(-D_CAV), PCB["z0"], PCB["z0"] + PCB["l"])
chip = box(-1.5, 1.5, y_of(-D_CAV) - PCB["t"] - CHIP_H, y_of(-D_CAV) - PCB["t"], MAG_Z - 1.5, MAG_Z + 1.5)
magnet = cyl_y(0, MAG_Z, MAG_D / 2, y_of(MAG_BOTTOM_S + MAG_L), y_of(MAG_BOTTOM_S))
shim = box(-SHIM["w"] / 2, SHIM["w"] / 2, YF, YF + SHIM["t"], ZT - SHIM["depth"], ZT + SHIM["protrude"])

# ================= skin (the part that gets cast) =================
pad = box(-SKIN_HW, SKIN_HW, y_of(SKIN_T), YF, SKIN_Z0, SKIN_Z1).edges("|Y").fillet(5.0)
crown = cq.Workplane("XY").workplane(offset=SKIN_Z0 - 1).center(0, y_of(SKIN_T) + CROWN_R).circle(CROWN_R).extrude(SKIN_Z1 - SKIN_Z0 + 2)
pad = pad.intersect(crown)
try: pad = pad.faces("<Y").edges().fillet(1.2)
except Exception: pass
# plug = cavity shape, 0.3 mm shorter in z than the cavity, bottom at PLUG_BOTTOM_S, slight width interference by design (stretch fit)
plug_pts = [(-OPEN_HW, y_of(0.0)), (OPEN_HW, y_of(0.0)), (UNDER_HW, y_of(-UNDER_DEPTH)), (UNDER_HW, y_of(PLUG_BOTTOM_S)),
            (-UNDER_HW, y_of(PLUG_BOTTOM_S)), (-UNDER_HW, y_of(-UNDER_DEPTH))]
plug = prism_z(plug_pts, CZ0 + 0.3, CZ1 - 0.3)
try: plug = plug.edges("|Y").fillet(0.6)
except Exception: pass
skin_solid = pad.union(plug)
mag_hole = cyl_y(0, MAG_Z, MAG_HOLE_D / 2, y_of(MAG_BOTTOM_S + MAG_L), y_of(PLUG_BOTTOM_S) + 0.01)
skin = skin_solid.cut(mag_hole)

# ================= mould (print: parting face up; PETG, 0.12 mm layers; release agent) =================
MX, MZ0, MZ1 = 12.5, 3.0, 28.0
cup_y0 = y_of(SKIN_T) - 5.0
cup = box(-MX, MX, cup_y0, YF, MZ0, MZ1).cut(pad)                      # cavity = pad (outer surface faces down)
cov_y1 = y_of(PLUG_BOTTOM_S) + 3.5
cover = box(-MX, MX, YF, cov_y1, MZ0, MZ1).cut(plug.translate((0, 0, 0)))
# magnet-hole pin: a 2.8 mm steel dowel pushed into the cover (modelled as a hole 3 mm deep); the pin runs through plug + pad cavity
pin_hole = cyl_y(0, MAG_Z, MAG_HOLE_D / 2, y_of(PLUG_BOTTOM_S) - 0.0, y_of(PLUG_BOTTOM_S) + 3.0)
cover = cover.cut(pin_hole)
pour = cyl_y(0, CZ0 + 2.0, 2.0, YF, cov_y1)                           # pour hole at the proximal end of the plug
vent_z = CZ1 - 1.0
vents = [cyl_y(sx * 6.4, z, 0.5, YF, cov_y1) for sx in (-1, 1) for z in (SKIN_Z0 + 1.0, SKIN_Z1 - 1.0)] + [cyl_y(0, vent_z, 0.6, YF, cov_y1)]
cover = cover.cut(pour)
for v in vents: cover = cover.cut(v)
# alignment: four 1.75 mm filament pins (holes 1.9 mm) and two M3 clamp holes
for sx in (-1, 1):
    for z in (MZ0 + 2.5, MZ1 - 2.5):
        for blk in ("cup", "cover"):
            h = cyl_y(sx * (MX - 2.5), z, 0.95, cup_y0 if blk == "cup" else YF, YF if blk == "cup" else cov_y1)
            if blk == "cup": cup = cup.cut(h)
            else: cover = cover.cut(h)

# ================= checks =================
def vol(w): return w.val().Volume()
def inter(a, b):
    try: return a.intersect(b).val().Volume()
    except Exception: return float("nan")
summary = {
    "tip_valid": tip_valid, "tip_volume_mm3": round(vol(tip), 1), "tip_mass_g_PETG_100pct": round(vol(tip) * 1.27e-3, 2),
    "skin_volume_mm3": round(vol(skin), 1), "skin_mass_g_silicone_1.07": round(vol(skin) * 1.07e-3, 2),
    "tip_x_pcb_overlap_mm3": round(inter(tip, pcb), 3), "tip_x_chip_overlap_mm3": round(inter(tip, chip), 3),
    "tip_x_magnet_overlap_mm3": round(inter(tip, magnet), 3), "tip_x_shim_overlap_mm3": round(inter(tip, shim), 3),
    "skin_x_tip_interference_mm3_design_stretch_fit": round(inter(skin, tip), 3),
    "skin_x_pcb_mm3": round(inter(skin, pcb), 3), "skin_x_chip_mm3": round(inter(skin, chip), 3),
    "magnet_inside_skin_hole_clear_mm3": round(inter(magnet, skin), 3),
    "min_side_wall_at_undercut_mm": round(HWT - UNDER_HW, 2),
    "wall_at_distal_end_of_cavity_mm": round(ZT - CZ1, 2),
    "tip_bbox": [round(v, 2) for v in (tip.val().BoundingBox().xmin, tip.val().BoundingBox().xmax, tip.val().BoundingBox().ymin, tip.val().BoundingBox().ymax, tip.val().BoundingBox().zmin, tip.val().BoundingBox().zmax)],
    "skin_bbox_y": [round(skin.val().BoundingBox().ymin, 2), round(skin.val().BoundingBox().ymax, 2)],
    "mould_cup_valid": cup.val().isValid(), "mould_cover_valid": cover.val().isValid(),
    "cup_size_mm": [2 * MX, round(MZ1 - MZ0, 1), round(YF - cup_y0, 1)], "cover_size_mm": [2 * MX, round(MZ1 - MZ0, 1), round(cov_y1 - YF, 1)],
}
# chip top (s) and magnet bottom (s) from the geometry itself
chip_top_s = -(D_CAV - PCB["t"] - CHIP_H)
summary["chip_top_s"] = round(chip_top_s, 2)
summary["magnet_bottom_s"] = MAG_BOTTOM_S
summary["gap_chip_to_magnet_mm"] = round(MAG_BOTTOM_S - chip_top_s, 2)
summary["gap_chip_to_plug_mm"] = round(PLUG_BOTTOM_S - chip_top_s, 2)

parts = {"05_tip_body": tip, "05_skin": skin, "05_mould_cup": cup, "05_mould_cover": cover,
         "05_ref_pcb": pcb, "05_ref_chip": chip, "05_ref_magnet": magnet, "05_ref_nail_shim": shim}
for n, w in parts.items():
    cq.exporters.export(w, os.path.join(OUT, n + ".step"))
    cq.exporters.export(w, os.path.join(OUT, n + ".stl"), tolerance=0.02, angularTolerance=0.1)
asm = cq.Assembly()
colors = {"05_tip_body": (0.9, 0.82, 0.66), "05_skin": (0.85, 0.3, 0.3), "05_ref_pcb": (0.1, 0.5, 0.2), "05_ref_chip": (0.1, 0.1, 0.1),
          "05_ref_magnet": (0.6, 0.6, 0.7), "05_ref_nail_shim": (0.8, 0.8, 0.85)}
for n, c in colors.items():
    asm.add(parts[n], name=n, color=cq.Color(*c))
asm.save(os.path.join(OUT, "05_tip_assembly.step"))
json.dump(summary, open(os.path.join(OUT, "05_fingertip_summary.json"), "w"), indent=1)
print(json.dumps(summary, indent=1))
