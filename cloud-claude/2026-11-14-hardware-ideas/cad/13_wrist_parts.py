"""13 · W13 wrist: metal load path for the bare-strand wrist (study 2026-11-15, agent 09, round 2).

The V2 cardan keeps its axes, ranges, gap, drive and outlines; what changes (report 13):
- ring, palm yoke (= V2 palm flange) and forearm yoke (= V2 wrist base) in CNC 7075-T6;
- Ø5 hardened pins (V2: Ø3) running in 4 plain flanged bushes 5 x 7 x 4.5 (V2: 623 ball bearings);
  flex: bushes in the ring X bosses, pins pressed in the palm lugs; dev: pins pressed in the ring Y ends,
  bushes in the forearm lugs;
- ring opening x +-11.5 (bar 3.5 wide), eye-plate ledges inside the ring at |y| 6.7..9.4 (the plate tabs bear on
  aluminium, the M2 screws only retain);
- palm yoke carries the palm plate seat (z 26..30.4, legs at |x| 11.6..14.4 outside the wrist heel keep-out);
- sensing (report 15 request): the +x flex pin and the +y dev pin are hollow (Ø2.4 bore) with Ø1.6 radial exits,
  plus a 2 x 2 cable groove on the ring top face between them;
- reorder plate (stainless, 4th plate of the drilled set) at z +38 in the palm: resolves the palm fan crossings
  with small fixed bends; a printed spacer carries it on the palm plate seat.

Reads out/13/kin/eyeplate_kin.json (round-2 eye table). Writes STEP + STL per part to out/13/, out/13/checks.json,
out/13/reorder.json and renders.

    uv run --no-project --with cadquery --with numpy --with trimesh --with scipy --with rtree --with matplotlib \
        --with pillow python cad/13_wrist_parts.py [--quick]
"""

from __future__ import annotations

import importlib.util
import json
import math
import sys
import time
from pathlib import Path

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
STUDY = HERE.parent
V2CAD = STUDY.parent / "2026-11-09-v2-hand-cad" / "cad"
OUT = STUDY / "out" / "13"
sys.path.insert(0, str(V2CAD))
import wrist as W  # noqa: E402  (V2 wrist, read-only use; constants patched in memory only)
from tools.checks import interference, is_valid  # noqa: E402

spec = importlib.util.spec_from_file_location("kin09", HERE / "09_eyeplate_kin.py")
KIN = importlib.util.module_from_spec(spec)
spec.loader.exec_module(KIN)
KIN.H_PALM = 30.0
QUICK = "--quick" in sys.argv
DATA = json.load(open(OUT / "kin" / "eyeplate_kin.json"))
OPT = DATA["options"][DATA["chosen"]]["opt"]
TABLE = DATA["eye_table"]

# --- W13 changes to the V2 wrist (in memory) ------------------------------------------------------------------
PIN_D = 5.0
BUSH = dict(id=5.0, od=7.0, len=4.5, flange_d=10.0, flange_t=0.75)  # iglidur G flanged / sintered bronze (verify)
W.PIN = PIN_D
W.BRG_OD, W.BRG_W = BUSH["od"], BUSH["len"]
W.RING_IN = (23.0, 37.0, 1.0)  # inner x +-11.5 (V2 +-12): ring bar 11.5..15 = 3.5 wide
W.RING_XCUT = 14.4  # beyond |x| 14.5 the palm flange sweeps (V2 heel band); bar 11.5..14.4 = 2.9 wide
W._CACHE.clear()

PLATE_T = 2 * OPT["r_lip"] - 0.04
R0 = OPT["r_throat"] + OPT["r_lip"]
PLATE = dict(x=9.8, y=9.5, tab_x=11.4, tab_y=(6.7, 9.4), r=1.5)  # one outline for all four drilled plates
TAB_SCREW = [(sx * 10.6, sy * 8.05) for sx in (-1, 1) for sy in (-1, 1)]
LEDGE = dict(x0=9.9, y=(6.6, 9.5), t=1.8)  # ring ledges above/below the plate tabs (aluminium)
H_PALM, H_FORE = 30.0, -30.0
SEAT = dict(z0=26.0, x=14.4, y=9.6, win=(8.4, 5.6), leg_x=(11.6, 14.4), leg_y=3.4)
FORE_WIN = (9.0, 6.3)
FRAME_TOP_UNDERSIDE = -22.0
M2_CLEAR, M2_PILOT = 2.2, 1.7
CABLE = dict(bore=2.4, exit=1.6, groove=2.0)
THUMB_ROLLER_Z = 44.0  # thumb turning roller row (8 x MR63 on a Ø3 dowel along Y at x 17), top tangent
Z_REORDER = float(next((a.split("=")[1] for a in sys.argv if a.startswith("--zr=")), 40.0))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


cyl = W.cyl
rot = W.rot_shape


def plate_outline():
    pl = box(-PLATE["x"], PLATE["x"], -PLATE["y"], PLATE["y"], -PLATE_T / 2, PLATE_T / 2).edges("|Z").fillet(PLATE["r"])
    for sx in (-1, 1):
        for sy in (-1, 1):
            x0, x1 = sorted((sx * (PLATE["x"] - 0.5), sx * PLATE["tab_x"]))
            y0, y1 = sorted((sy * PLATE["tab_y"][0], sy * PLATE["tab_y"][1]))
            pl = pl.union(box(x0, x1, y0, y1, -PLATE_T / 2, PLATE_T / 2))
    for x, y in TAB_SCREW:
        pl = pl.cut(cyl((x, y, -1), (x, y, 1), M2_CLEAR / 2 - 0.1))
    return pl


def drill(pl, holes):
    rl = OPT["r_lip"]
    for ex, ey in holes:
        hole = cq.Workplane("XY").circle(R0 + 0.01).extrude(PLATE_T + 0.02).translate((0, 0, -PLATE_T / 2 - 0.01))
        tor = cq.Workplane("XY").add(cq.Solid.makeTorus(R0, rl))
        pl = pl.cut(hole.cut(tor).translate((ex, ey, 0)))
    return pl


def eyes():
    return sorted({tuple(np.round(r["eye"], 3)) for r in TABLE})


# --- ring -----------------------------------------------------------------------------------------------------
def make_ring():
    ring = W.make_ring()
    # V2's flex stop tooth under each X boss is removed (it would sit in the widened bar's way); stops: report 13 §3
    for sx in (-1, 1):
        x0, x1 = sorted((sx * 13.6, sx * 15.6))
        ring = ring.cut(box(x0, x1, -1.6, 1.6, -9.7, -6.6))
    # thicker press boss for the Ø5 dev pins at the Y ends
    for sy in (-1, 1):
        ring = ring.union(cyl((0, sy * 18.5, 0), (0, sy * 21.5, 0), 5.0).cut(cyl((0, sy * 18.0, 0), (0, sy * 22, 0), PIN_D / 2 - 0.05)))
    # eye-plate ledges (above and below the tabs) at x 9.9..11.6, |y| 6.6..9.5
    g = PLATE_T / 2 + 0.02
    for sx in (-1, 1):
        x0, x1 = sorted((sx * LEDGE["x0"], sx * 11.6))
        for sy in (-1, 1):
            y0, y1 = sorted((sy * LEDGE["y"][0], sy * LEDGE["y"][1]))
            for z0, z1 in ((g, g + LEDGE["t"]), (-g - LEDGE["t"], -g)):
                ring = ring.union(box(x0, x1, y0, y1, z0, z1))
    for x, y in TAB_SCREW:  # M2 retaining screw through the upper ledge + tab, thread in the lower ledge
        ring = ring.cut(cyl((x, y, -g - LEDGE["t"] - 0.1), (x, y, g + LEDGE["t"] + 0.1), M2_PILOT / 2))
    # sensing cable: groove 2 x 2 on the ring top face from the +x flex boss to the +y dev end, holes into both bores
    zt = W.RING_H / 2
    # flex pin (+x) ends at the boss inner face x 11: the cable leaves it axially into a 1.6 x 1.6 slot in that face,
    # rises to the boss top and runs in the groove along the +x bar to the +y end, then down a Ø1.6 hole into the
    # hollow dev pin (pressed in the ring, radial exit at y 19.6)
    ring = ring.cut(box(10.6, 11.8, -0.8, 0.8, PIN_D / 2 - 0.3, 7.0))
    path = [(11.2, 0.0), (13.25, 2.0), (13.25, 14.0), (9.0, 19.0), (0.0, 19.6)]
    zg, rg = zt - 0.8, 1.1  # round-bottom groove: depth 1.9, 1.5 wide at the face
    for (a, b), (c, d) in zip(path[:-1], path[1:]):
        ring = ring.cut(cyl((a, b, zg), (c, d, zg), rg))
    for a, b in path[1:-1]:
        ring = ring.cut(cq.Workplane("XY").sphere(rg).translate((a, b, zg)))
    ring = ring.cut(cyl((11.2, 0, zg), (11.2, 0, 7.0), 0.8))
    ring = ring.cut(cyl((0, 19.6, 0), (0, 19.6, zt + 0.5), CABLE["exit"] / 2))  # into the dev pin seat
    return ring


# --- palm yoke = V2 flange (7075) + palm-plate seat ------------------------------------------------------------
def make_palm_yoke():
    fl, _clamp = W.make_flange()
    # remove V2's flex stop arc wall (x 13.8..16 on the lug's inner face): the 3.5 mm 7075 ring bar needs that space
    for sx in (-1, 1):
        x0, x1 = sorted((sx * 13.0, sx * 15.95))
        fl = fl.cut(box(x0, x1, -12.0, 12.0, -12.0, 9.9))
    s = SEAT
    frame = box(-s["x"], s["x"], -s["y"], s["y"], s["z0"], H_PALM + PLATE_T / 2 - 0.2).edges("|Z").fillet(2.0)
    frame = frame.cut(box(-s["win"][0], s["win"][0], -s["win"][1], s["win"][1], s["z0"] - 1, H_PALM + 2))
    # plate pocket (0.1 clearance), plate rests on the frame at z H_PALM - t/2
    frame = frame.cut(box(-PLATE["tab_x"] - 0.1, PLATE["tab_x"] + 0.1, -PLATE["y"] - 0.1, PLATE["y"] + 0.1,
                          H_PALM - PLATE_T / 2, H_PALM + 2))
    for sx in (-1, 1):  # legs down to the flange top (z 15), inside the band |y| <= 3.4 allowed by the heel keep-out
        x0, x1 = sorted((sx * s["leg_x"][0], sx * s["leg_x"][1]))
        frame = frame.union(box(x0, x1, -s["leg_y"], s["leg_y"], 14.0, s["z0"] + 0.5))
    for x, y in TAB_SCREW:
        frame = frame.cut(cyl((x, y, s["z0"] + 0.5), (x, y, H_PALM), M2_PILOT / 2))
    return fl.union(frame)


def make_fore_carrier():
    """Printed PAHT-CF carrier under the forearm plate (load <= 83 N SCS / ~180 N HLS), legs to the frame top ring."""
    z1 = H_FORE - PLATE_T / 2
    z0 = z1 - 3.0
    body = box(-11.6, 11.6, -10.6, 10.6, z0, z1).edges("|Z").fillet(2.0)
    body = body.cut(box(-FORE_WIN[0], FORE_WIN[0], -FORE_WIN[1], FORE_WIN[1], z0 - 1, z1 + 1))
    for sy in (-1, 1):
        y0, y1 = sorted((sy * 10.0, sy * 21.5))
        body = body.union(box(-3.0, 3.0, y0, y1, z0, z1))
        y0, y1 = sorted((sy * 17.5, sy * 21.5))
        body = body.union(box(-3.0, 3.0, y0, y1, z0, FRAME_TOP_UNDERSIDE))
        body = body.cut(cyl((0, sy * 19.5, z0 - 1), (0, sy * 19.5, FRAME_TOP_UNDERSIDE + 1), M2_CLEAR / 2))
    for x, y in TAB_SCREW:
        body = body.cut(cyl((x, y, z0 - 0.1), (x, y, z1 + 0.1), M2_PILOT / 2))
    return body


def hollow_pin(p0, p1, exit_at):
    pin = cyl(p0, p1, PIN_D / 2).cut(cyl(p0, p1, CABLE["bore"] / 2))
    e = np.asarray(exit_at, float)
    return pin.cut(cyl(e - np.array([0, 0, 3.0]), e + np.array([0, 0, 3.0]), CABLE["exit"] / 2))


# --- reorder plate (palm fan) ----------------------------------------------------------------------------------
FINGER_EXIT = {  # 07 bench-finger strand exits (07 frame, MCP at origin), clipped to a 19 mm module: |dx| <= 6
    "pip_flex": (-1.0, 0.0), "pip_ext": (1.0, 3.1), "mcp_flex_flex": (-6.0, -4.41), "mcp_flex_ext": (6.0, 3.71),
    "mcp_abd_flex": (6.0, 10.2), "mcp_abd_ext": (-6.0, 12.2)}


def targets():
    """Palm-side target per strand: finger module entry (MCP z - 16, vertical) or a thumb roller row (x 17, z 44)."""
    import params as P  # V2 params

    tg = {}
    th = sorted([r for r in TABLE if r["thumb"]], key=lambda r: (r["eye"][1], r["eye"][0]))
    for k, r in enumerate(th):
        tg[r["name"]] = np.array([17.0, -10.5 + 3.0 * k, THUMB_ROLLER_Z])
    for r in TABLE:
        if r["thumb"]:
            continue
        f = r["name"].split("_")[0]
        key = r["name"][len(f) + 1:]
        dx, dy = FINGER_EXIT[key]
        mx, my, mz = P.FINGERS[f]["mcp"]
        tg[r["name"]] = np.array([mx + dx, my + dy, mz - 16.0])
    return tg


def optimise_reorder():
    from scipy.optimize import minimize

    tg = targets()
    names = [r["name"] for r in TABLE]
    Pp = np.array([[r["eye"][0], r["eye"][1], H_PALM + PLATE_T / 2] for r in TABLE])
    T = np.array([tg[n] for n in names])
    s0 = (Z_REORDER - Pp[:, 2]) / (T[:, 2] - Pp[:, 2])
    W0 = (Pp + s0[:, None] * (T - Pp))[:, :2]
    n = len(names)
    I, J = np.triu_indices(n, 1)
    up = np.array([0, 0, 1.0])

    def segs(w):
        Wp = np.c_[w.reshape(n, 2), np.full(n, Z_REORDER)]
        return Wp

    def bends(Wp):
        a = Wp - Pp
        b = T - Wp
        a /= np.linalg.norm(a, axis=1)[:, None]
        b /= np.linalg.norm(b, axis=1)[:, None]
        b_plate = np.degrees(np.arccos(np.clip(a @ up, -1, 1)))
        b_w = np.degrees(np.arccos(np.clip((a * b).sum(1), -1, 1)))
        return b_plate, b_w

    def mind(Wp):
        d1 = KIN.seg_seg_dist(Pp[I], Wp[I], Pp[J], Wp[J])
        d2 = KIN.seg_seg_dist(Wp[I], T[I], Wp[J], T[J])
        d3 = KIN.seg_seg_dist(Pp[I], Wp[I], Wp[J], T[J])
        d4 = KIN.seg_seg_dist(Wp[I], T[I], Pp[J], Wp[J])
        dw = np.linalg.norm(Wp[I, :2] - Wp[J, :2], axis=1)
        return np.minimum.reduce([d1, d2, d3, d4]), dw

    straight = W0.ravel().copy()
    # only pairs that can come close matter: prefilter on the straight-line distances (< 4 mm)
    Ws = np.c_[W0, np.full(n, Z_REORDER)]
    near = np.minimum.reduce([KIN.seg_seg_dist(Pp[I], Ws[I], Pp[J], Ws[J]), KIN.seg_seg_dist(Ws[I], T[I], Ws[J], T[J]),
                              KIN.seg_seg_dist(Pp[I], Ws[I], Ws[J], T[J]), KIN.seg_seg_dist(Ws[I], T[I], Pp[J], Ws[J]),
                              np.linalg.norm(Ws[I, :2] - Ws[J, :2], axis=1)]) < 4.0
    Iall, Jall = I, J
    I, J = I[near], J[near]

    def cost(w):
        Wp = segs(w)
        bp, bw = bends(Wp)
        d, dw = mind(Wp)
        pen = (np.clip(1.10 - d, 0, None) ** 2).sum() * 4000 + (np.clip(2.25 - dw, 0, None) ** 2).sum() * 4000
        return float((np.radians(bw) ** 2).sum() * 10 + (np.radians(bp) ** 2).sum() * 3 + pen)

    res = minimize(cost, straight, method="Powell", options=dict(maxiter=40000, xtol=1e-3, ftol=1e-6))
    res = minimize(cost, res.x, method="Powell", options=dict(maxiter=40000, xtol=1e-4, ftol=1e-8))
    Wp = segs(res.x)
    bp, bw = bends(Wp)
    I, J = Iall, Jall
    d, dw = mind(Wp)
    bp0, bw0 = bends(segs(straight))
    d0, _ = mind(segs(straight))
    out = dict(z=Z_REORDER, names=names, W=Wp.round(3).tolist(), targets=T.round(2).tolist(),
               bend_plate_deg=bp.round(1).tolist(), bend_reorder_deg=bw.round(1).tolist(),
               min_centre_distance=float(d.min()), min_hole_spacing=float(dw.min()),
               straight_min_centre_distance=float(d0.min()), straight_pairs_below_0p97=int((d0 < 0.97).sum()),
               pairs_below_0p97=int((d < 0.97).sum()), max_reorder_bend=float(bw.max()),
               median_reorder_bend=float(np.median(bw)))
    return out


def hull_poly(pts, margin):
    from scipy.spatial import ConvexHull

    pts = np.asarray(pts, float)
    ring = np.vstack([pts + margin * np.c_[np.cos(a), np.sin(a)] for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)])
    h = ConvexHull(ring)
    return [tuple(map(float, ring[i])) for i in h.vertices]


def make_reorder_plate(R):
    holes = [tuple(w[:2]) for w in R["W"]]
    poly = hull_poly(holes, 2.6)
    pl = cq.Workplane("XY").polyline(poly).close().extrude(PLATE_T).translate((0, 0, -PLATE_T / 2))
    pl = drill(pl, holes)
    return pl.translate((0, 0, Z_REORDER)), holes


def make_reorder_spacer(holes):
    """Printed PETG wall between the palm-plate seat and the reorder plate (light load): window = hull of the strand
    points on both plates + 1.2 mm, wall 2.0 mm."""
    z0, z1 = H_PALM + PLATE_T / 2 + 0.05, Z_REORDER - PLATE_T / 2
    pts = list(holes) + [tuple(e) for e in eyes()]
    inner = hull_poly(pts, R0 + 1.2)
    outer = hull_poly(pts, R0 + 3.2)
    o = cq.Workplane("XY").workplane(offset=z0).polyline(outer).close().extrude(z1 - z0)
    i = cq.Workplane("XY").workplane(offset=z0 - 1).polyline(inner).close().extrude(z1 - z0 + 2)
    return o.cut(i)


# --- strands for checks / renders -------------------------------------------------------------------------------
def strand_polys(f, d, R=None):
    strands = KIN.load_strands()
    by = {r["name"]: np.array(r["eye"]) for r in TABLE}
    pl = KIN.fast_polylines(strands, [by[s["name"]] for s in strands], (f, d))
    return strands, pl


def to_mesh(shape):
    import trimesh
    from tools.render import tessellate

    v, fc = tessellate(shape, 0.1, 0.3)
    return trimesh.Trimesh(v, fc, process=True)


def main():
    t0 = time.time()
    OUT.mkdir(parents=True, exist_ok=True)
    rj = OUT / "reorder.json"
    R = json.loads(rj.read_text()) if rj.exists() and json.loads(rj.read_text()).get("z") == Z_REORDER else optimise_reorder()
    (OUT / "reorder.json").write_text(json.dumps(R, indent=1))
    print(f"reorder: straight lines {R['straight_pairs_below_0p97']} pairs < 0.97 (min {R['straight_min_centre_distance']:.2f}); "
          f"after: {R['pairs_below_0p97']} pairs, min {R['min_centre_distance']:.2f} mm, hole spacing {R['min_hole_spacing']:.2f}; "
          f"reorder bend median {R['median_reorder_bend']:.1f}, max {R['max_reorder_bend']:.1f} deg")
    plate = drill(plate_outline(), eyes())
    rplate, rholes = make_reorder_plate(R)
    parts = {
        "ring_w13": make_ring(),
        "eye_plate_w13": plate,
        "palm_yoke_w13": make_palm_yoke(),
        "palm_plate_w13": plate.translate((0, 0, H_PALM)),
        "reorder_plate": rplate,
        "reorder_spacer": make_reorder_spacer(rholes),
        "forearm_yoke_w13": W.make_base(),
        "forearm_plate_w13": plate.translate((0, 0, H_FORE)),
        "forearm_plate_carrier_w13": make_fore_carrier(),
        "pin_flex_R_hollow": cyl((11.0, 0, 0), (21.0, 0, 0), PIN_D / 2).cut(cyl((10.0, 0, 0), (22.0, 0, 0), CABLE["bore"] / 2)),
        "pin_flex_L": cyl((-19.5, 0, 0), (-11.0, 0, 0), PIN_D / 2),
        "pin_dev_B_hollow": hollow_pin((0, 18.5, 0), (0, 29.0, 0), (0, 19.6, 0)),
        "pin_dev_F": cyl((0, -27.0, 0), (0, -18.5, 0), PIN_D / 2),
    }
    for sx in (-1, 1):  # flex bushes in the ring X bosses (x 11..15.5)
        x0, x1 = sorted((sx * 11.0, sx * 15.5))
        parts[f"bush_flex_{'R' if sx > 0 else 'L'}"] = cyl((x0, 0, 0), (x1, 0, 0), BUSH["od"] / 2).cut(cyl((x0 - 1, 0, 0), (x1 + 1, 0, 0), BUSH["id"] / 2))
    for sy in (-1, 1):  # dev bushes in the forearm lugs (outer 4.5 mm of y 22..27)
        y0, y1 = sorted((sy * 22.5, sy * 27.0))
        parts[f"bush_dev_{'B' if sy > 0 else 'F'}"] = cyl((0, y0, 0), (0, y1, 0), BUSH["od"] / 2).cut(cyl((0, y0 - 1, 0), (0, y1 + 1, 0), BUSH["id"] / 2))
    print(f"built {len(parts)} parts in {time.time()-t0:.0f} s")
    body = {"ring_w13": "ring", "eye_plate_w13": "ring", "pin_dev_B_hollow": "ring", "pin_dev_F": "ring",
            "bush_flex_R": "ring", "bush_flex_L": "ring", "palm_yoke_w13": "palm", "palm_plate_w13": "palm",
            "reorder_plate": "palm", "reorder_spacer": "palm", "pin_flex_R_hollow": "palm", "pin_flex_L": "palm"}
    checks = dict(parts={}, notes=[])
    for nm, s in parts.items():
        v = is_valid(s)
        bb = s.val().BoundingBox()
        checks["parts"][nm] = dict(v, bbox=[round(bb.xmin, 2), round(bb.xmax, 2), round(bb.ymin, 2), round(bb.ymax, 2),
                                           round(bb.zmin, 2), round(bb.zmax, 2)])
        print(f"  {nm:28s} valid {v['valid']} watertight {v['watertight']} vol {v['volume']:.1f}")
        cq.exporters.export(s, str(OUT / f"{nm}.stl"))
        cq.exporters.export(s, str(OUT / f"{nm}.step"))
    # moving-part check over the wrist grid: W13 parts + V2 drive (servos, gears, cranks, rods)
    ig = [("pin_*", "ring_w13"), ("pin_flex*", "palm_yoke_w13"), ("pin_dev*", "forearm_yoke_w13"), ("pin_*", "bush_*"),
          ("bush_flex*", "ring_w13"), ("bush_dev*", "forearm_yoke_w13"), ("eye_plate_w13", "ring_w13"),
          ("palm_plate_w13", "palm_yoke_w13"), ("forearm_plate_w13", "forearm_plate_carrier_w13"),
          ("reorder_spacer", "reorder_plate"), ("reorder_spacer", "palm_plate_w13"), ("reorder_spacer", "palm_yoke_w13"),
          ("V2_wrist_servo_*", "V2_wrist_gears_*"), ("V2_wrist_gears_*", "V2_wrist_crank_*"), ("V2_wrist_rod_*", "V2_wrist_crank_*"),
          ("V2_wrist_rod_*", "palm_yoke_w13"), ("V2_wrist_gears_*", "forearm_yoke_w13")]
    grid = KIN.wrist_grid(3 if QUICK else 9)
    res, sclear, worst = {}, {}, {}
    t1 = time.time()
    for f, d in grid:
        Rr, Rp = KIN.R_ring(f, d), KIN.R_palm(f, d)
        Rb = {"ring": Rr, "palm": Rp}
        allp = {n: rot(s, Rb[body[n]]) if body.get(n) in Rb else s for n, s in parts.items()}
        b = W.build({"wrist_flex": f, "wrist_dev": d}, bundle=False)["parts"]
        for n in ("wrist_servo_R", "wrist_servo_L", "wrist_gears_R", "wrist_gears_L", "wrist_crank_R", "wrist_crank_L",
                  "wrist_rod_R", "wrist_rod_L"):
            if n in b:
                allp[f"V2_{n}"] = b[n]
        r = interference(allp, threshold=0.5, ignore=ig)
        r = [x for x in r if not (x[0].startswith("V2_") and x[1].startswith("V2_"))]
        res[f"{f:.1f},{d:.1f}"] = r
        meshes = {n: to_mesh(allp[n]) for n in ("ring_w13", "palm_yoke_w13", "forearm_yoke_w13", "pin_flex_R_hollow",
                                                  "pin_flex_L", "pin_dev_B_hollow", "pin_dev_F", "bush_flex_R", "bush_flex_L")}
        import trimesh

        _, pls = strand_polys(f, d)
        pts = []
        for pl in pls:
            for a, bb_ in zip(pl[1:3], pl[2:4]):
                nn = max(2, int(np.linalg.norm(bb_ - a) / 0.5))
                pts.append(a + np.linspace(0.05, 0.95, nn)[:, None] * (bb_ - a))
        pts = np.vstack(pts)
        sc = {n: float(trimesh.proximity.closest_point(m, pts)[1].min() - KIN.LINE_D / 2) for n, m in meshes.items()}
        sclear[f"{f:.1f},{d:.1f}"] = sc
        for n, v in sc.items():
            worst[n] = min(worst.get(n, 1e9), v)
        print(f"  flex {f:6.1f} dev {d:6.1f}: {'clear' if not r else r}  strand min {min(sc.values()):.2f} mm ({time.time()-t1:.0f} s)")
    checks["wrist_grid_interference"] = res
    checks["wrist_grid_all_clear"] = all(not v for v in res.values())
    checks["strand_clearance_min_mm"] = worst
    checks["strand_clearance_grid"] = sclear
    checks["reorder"] = {k: v for k, v in R.items() if k not in ("W", "targets", "names", "bend_plate_deg", "bend_reorder_deg")}
    (OUT / "checks.json").write_text(json.dumps(checks, indent=1, default=str))
    render_all(parts, body)
    print(f"done in {time.time()-t0:.0f} s")


def render_all(parts, body):
    from tools.render import render

    col = {"ring_w13": "#8a9bb0", "palm_yoke_w13": "#6f8fb8", "forearm_yoke_w13": "#7a7a7a", "eye_plate_w13": "#c8ccd2",
           "palm_plate_w13": "#c8ccd2", "forearm_plate_w13": "#c8ccd2", "reorder_plate": "#c8ccd2",
           "reorder_spacer": "#5aa0e0", "forearm_plate_carrier_w13": "#444444"}
    try:
        for f, d, tag in ((0.0, 0.0, "q0"), (50.0, 20.0, "flex+50_dev+20"), (-50.0, -30.0, "flex-50_dev-30")):
            Rr, Rp = KIN.R_ring(f, d), KIN.R_palm(f, d)
            Rb = {"ring": Rr, "palm": Rp}
            sc = {n: rot(s, Rb[body[n]]) if body.get(n) in Rb else s for n, s in parts.items()}
            _, pls = strand_polys(f, d)
            st = None
            for pl in pls:
                for a, b in zip(pl[:-1], pl[1:]):
                    c = cyl(a, b, 0.3)
                    st = c if st is None else st.union(c, clean=False)
            sc["strands"] = st
            cols = dict(col, strands="#d62728")
            render(sc, str(OUT / f"w13_{tag}.png"), views=("iso", "front", "side", "top"), colors=cols,
                   title=f"13 W13 metal wrist, flex {f:+.0f} dev {d:+.0f} (strands red)")
        render({k: parts[k] for k in ("ring_w13", "eye_plate_w13", "pin_flex_R_hollow", "pin_flex_L", "pin_dev_B_hollow",
                                      "pin_dev_F", "bush_flex_R", "bush_flex_L")},
               str(OUT / "w13_ring_eyeplate.png"), views=("iso", "top", "front", "side"), colors=col,
               title="13 ring (7075) + eye plate with tabs on ledges, Ø5 pins (2 hollow), bushes")
        render({k: parts[k] for k in ("palm_yoke_w13", "palm_plate_w13", "reorder_plate", "reorder_spacer")},
               str(OUT / "w13_palm_yoke_reorder.png"), views=("iso", "front", "side", "top"), colors=col,
               title="13 palm yoke (7075) with palm-plate seat, reorder plate + spacer")
    except Exception as e:  # pragma: no cover
        print("render failed:", e)


if __name__ == "__main__":
    main()
