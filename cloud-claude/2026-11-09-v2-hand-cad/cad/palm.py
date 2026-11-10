"""Tendra V2 concept: the palm (integrator, study 2026-11-09).

The palm joins the four fingers (abduction pivots), the thumb (mount seat + bay), the wrist (palm flange bolts,
sheath clamp pocket, rounded heel) and carries the 32 tendon sheaths from each finger / thumb exit to its cell in
the wrist's palm sheath clamp (paths from routing.py -> out/assembly/routes.json).

Parts (hand frame = palm frame at wrist q = 0, mm):
    palm_shell        PETG, the structure: knuckle lips with the 3 mm abd dowel bores, sheath channels (2.2 bore),
                      free-span pockets under each yoke, thumb seat (mount bore, 3 x M3 inserts), thenar heel
                      (cmc_rot idlers + steel guide tubes), flange seat (4 x M3 from inside), clamp pocket, heel.
                      Print lying on its back (dorsal face down): knuckle lips and channel curves in the build plane.
    palm_back_plate   PETG, 3.5 mm dorsal plate (screwed, 6 x M2): opens the dorsal service bay (flange screw,
                      dorsal channels laid in as open grooves). Print inner face down.
    palm_pad          TPU 95A, 2 mm central pad (covers the palmar flange-screw hatch)
    hypothenar_pad    TPU 95A, up to 4 mm (ulnar heel) -> with the knuckle pad the palm face is cupped
    knuckle_pad       TPU 95A, 1.5 mm over the palmar knuckle lip (human distal palmar pad)
    palm_sheaths      PTFE 1 x 2 tubes from each yoke exit / abd mouth to the clamp top (free span included)
    thumb_guides      18G stainless guide tubes (bent, R >= 4) for the 8 thumb strands, clamp -> mount
    rot_idlers        2 x idler pulley (MR63 + tyre) + 3 mm axles for the cmc_rot strands
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

OUT = HERE.parent / "out" / "assembly"
V = cq.Vector
Y_FACE, Y_BACK = -14.0, 14.0
LIP_PALMAR = P.PALM_KNUCKLE_BULGE["palmar_y"]  # -18.1
LIP_DORSAL = P.PALM_KNUCKLE_BULGE["dorsal_y"]  # 15.6
BACK_PLATE_Y = 10.5  # back plate from here to the dorsal face
PIN_HOLE = P.PIN_D_BIG + P.PRESS_FIT  # 2.9 press fit for the 3 mm abd dowels
FINGERS = ("index", "middle", "ring", "little")


# ----------------------------------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def rbox(x0, x1, y0, y1, z0, z1, r, sel="|Y"):
    b = box(x0, x1, y0, y1, z0, z1)
    try:
        return b.edges(sel).fillet(r)
    except Exception:
        return b


def cyl(p0, p1, r):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    return cq.Workplane(obj=cq.Solid.makeCylinder(r, L, V(*p0), V(*(d / L))))


def xz_prism(segs, y0, y1):
    """Closed (x, z) outline made of segments [("line", pts) | ("spline", pts)], extruded from y0 to y1."""
    start = segs[0][1][0]
    wp = cq.Workplane("XY").moveTo(*start)
    for kind, pts in segs:
        pts = [tuple(map(float, p)) for p in pts]
        if kind == "line":
            for p in pts[1:]:
                wp = wp.lineTo(*p)
        else:
            wp = wp.spline(pts[1:], includeCurrent=True)
    wp = wp.close().extrude(y1 - y0)
    # sketch (x, s) extruded along +z by e -> rotate about X by 90: (x, s, e) -> (x, -e, s)
    wp = wp.rotate((0, 0, 0), (1, 0, 0), 90)
    return wp.translate((0, y1, 0))


def _unused_smooth_xz(ctrl, n=60):
    from scipy.interpolate import CubicSpline
    c = np.asarray(ctrl, float)
    t = np.r_[0, np.cumsum(np.linalg.norm(np.diff(c, axis=0), axis=1))]
    cs = CubicSpline(t, c, bc_type="natural")
    return cs(np.linspace(0, t[-1], n))


def tube_solid(pts, r):
    """Sweep a circle of radius r along a smooth polyline (hand frame)."""
    pts = np.asarray(pts, float)
    keep = np.r_[True, np.linalg.norm(np.diff(pts, axis=0), axis=1) > 0.2]
    pts = pts[keep]
    path = cq.Wire.assembleEdges([cq.Edge.makeSpline([V(*q) for q in pts])])
    d = pts[1] - pts[0]
    d /= np.linalg.norm(d)
    circ = cq.Wire.makeCircle(r, V(*pts[0]), V(*d))
    return cq.Solid.sweep(circ, [], path, makeSolid=True, isFrenet=False, transitionMode="right")


def ext(pts, a, b):
    """Extend a polyline by a mm at the start and b mm at the end along its end tangents."""
    pts = np.asarray(pts, float)
    t0 = pts[0] - pts[1]
    t1 = pts[-1] - pts[-2]
    return np.vstack([pts[0] + t0 / np.linalg.norm(t0) * a, pts, pts[-1] + t1 / np.linalg.norm(t1) * b])


# ----------------------------------------------------------------------------------------------------
# interface data
# ----------------------------------------------------------------------------------------------------
def finger_iface():
    import finger
    return {f: finger.build(f, {}, with_parts=False) for f in FINGERS}


def thumb_iface():
    import thumb
    return thumb.interface()


def heel_cutter():
    """Material the palm must NOT have (report 03 keep-out), as a solid to subtract."""
    H = P.PALM_HEEL
    z0 = P.PALM_Z_BASE - 1
    t = math.tan(math.radians(H["wedge_deg"]))
    heel = cyl((-H["x_half"], 0, 0), (H["x_half"], 0, 0), H["r_x"])
    wedge = (cq.Workplane("YZ", origin=(-H["x_half"], 0, 0))
             .polyline([(0, 0), (70 * t, 70), (-70 * t, 70)]).close().extrude(2 * H["x_half"]))
    c1 = box(-H["x_half"], H["x_half"], -40, 40, z0, H["z_top"]).cut(heel).cut(wedge)
    heel2 = cyl((-H["x_half_low"], 0, 0), (H["x_half_low"], 0, 0), H["r_x"])
    c2 = box(-H["x_half_low"], H["x_half_low"], -40, 40, z0, H["z_low"]).cut(heel2)
    cut = c1.union(c2)
    x0, x1 = H["x_band"]
    for sx in (-1, 1):
        a, b = sorted((sx * x0, sx * x1))
        cut = cut.union(box(a, b, H["band_y"], 40, z0, H["z_low"])).union(box(a, b, -40, -H["band_y"], z0, H["z_low"]))
    return cut


# ----------------------------------------------------------------------------------------------------
# palm body
# ----------------------------------------------------------------------------------------------------
def outline_xz(fi):
    """Palm outline in the (x, z) plane: heel at z 15, scalloped knuckle edge through the abd pins."""
    pins = {f: fi[f]["interface"]["abd_pivot"]["axis_point"] for f in FINGERS}
    top = [(33.0, 74.0), (pins["index"][0] + 4, pins["index"][2] + 3.6), (10.0, pins["index"][2] + 1.5),
           (pins["middle"][0], pins["middle"][2] + 3.6), (-10.0, pins["ring"][2] + 1.5),
           (pins["ring"][0], pins["ring"][2] + 3.6), (-29.5, pins["little"][2] + 2.0),
           (pins["little"][0], pins["little"][2] + 3.6), (-48.5, 70.0), (-51.0, 58.0)]
    return [("spline", top), ("spline", [(-51.0, 58.0), (-51.5, 40.0), (-49.0, 22.0), (-41.0, 15.0)]),
            ("line", [(-41.0, 15.0), (30.0, 15.0)]),
            ("spline", [(30.0, 15.0), (34.5, 24.0), (35.0, 50.0), (33.0, 74.0)])]


def lip_xz(fi):
    pins = {f: fi[f]["interface"]["abd_pivot"]["axis_point"] for f in FINGERS}
    top = [(32.0, 70.0)]
    for f in FINGERS:
        x, z = pins[f][0], pins[f][2]
        top += [(x + 3.5, z + 3.6), (x - 3.5, z + 3.6)]
    top += [(-49.0, 64.0)]
    # dips between the knuckles
    top2 = [top[0]]
    for i in range(1, len(top) - 1, 2):
        top2 += [top[i], top[i + 1]]
        if i + 2 < len(top) - 1:
            top2.append(((top[i + 1][0] + top[i + 2][0]) / 2, min(top[i + 1][1], top[i + 2][1]) - 2.5))
    top2.append(top[-1])
    bot = [(-49.0, 64.0), (-46.0, 57.0), (-10.0, 61.0), (31.0, 59.0), (32.0, 70.0)]
    return [("spline", top2), ("spline", bot)]


def make_core(fi):
    core = xz_prism(outline_xz(fi), Y_FACE, Y_BACK)
    try:
        core = core.faces("<Y").edges().fillet(3.0)
    except Exception:
        pass
    try:
        core = core.faces(">Y").edges().fillet(4.0)
    except Exception:
        pass
    lip_p = xz_prism(lip_xz(fi), LIP_PALMAR, Y_FACE + 1.0)
    try:
        lip_p = lip_p.faces("<Y").edges().fillet(1.6)
    except Exception:
        pass
    lip_d = xz_prism(lip_xz(fi), Y_BACK - 1.0, LIP_DORSAL)
    try:
        lip_d = lip_d.faces(">Y").edges().fillet(1.2)
    except Exception:
        pass
    return core.union(lip_p).union(lip_d)


def thenar_heel():
    """Thenar heel block under the thumb mount (palmar side): carries the cmc_rot idlers and the steel guide
    tubes' palmar run. Human thenar eminence base."""
    b = rbox(8.0, 52.0, -31.0, -10.0, P.PALM_Z_BASE, 27.0, 6.0, "|Z")
    try:
        b = b.faces(">Z").edges().fillet(4.0)
    except Exception:
        pass
    return b


# ----------------------------------------------------------------------------------------------------
# cutters
# ----------------------------------------------------------------------------------------------------
def yoke_cuts(fi):
    """Per finger: the yoke's swept envelope (y between the lips), the free-span sheath pocket below it and
    the bare abd-strand slots from the mouths up to the envelope."""
    import finger
    cuts = []
    for f in FINGERS:
        it = fi[f]["interface"]
        lo, hi = np.array(it["yoke_envelope"]["min"]), np.array(it["yoke_envelope"]["max"])
        cuts.append(rbox(lo[0], hi[0], lo[1] - 0.25, hi[1] + 0.25, lo[2], 130.0, 2.0, "|Y"))
        # abd bare strands: slot from the mouth to the envelope
        for e in it["strand_exits"]:
            if "mcp_abd" in e["strand"]:
                p = np.array(e["point"])
                cuts.append(cyl(p - [0, 0, 0.5], [p[0], p[1], lo[2] + 1.0], 1.6))
    return cuts


def pocket_from_paths(paths, pad=1.6, zmax=None):
    """Rounded box around a set of free-span polylines (sheath bending room)."""
    a = np.vstack(paths)
    x0, y0, z0 = a.min(0) - pad
    x1, y1, z1 = a.max(0) + pad
    if zmax is not None:
        z1 = max(z1, zmax)
    return rbox(x0, x1, y0, y1, z0, z1, min(2.0, (x1 - x0) / 2 - 0.1), "|Z")


def pin_bores(fi):
    out = []
    for f in FINGERS:
        ap = np.array(fi[f]["interface"]["abd_pivot"]["axis_point"])
        out.append(cyl((ap[0], LIP_PALMAR - 0.5, ap[2]), (ap[0], -13.0, ap[2]), PIN_HOLE / 2))
        out.append(cyl((ap[0], 10.6, ap[2]), (ap[0], LIP_DORSAL + 0.5, ap[2]), PIN_HOLE / 2))
    return out


def thumb_cuts(ti, mount_solid):
    """Thumb seat: the mount itself (zero-clearance seat), flange disc + screw heads above the flange plane,
    the base sweep hull (+1 mm), 3 heat-set insert holes."""
    C = ti["frame_C"]
    o, z = np.array(C["origin"]), np.array(C["z"])
    cuts = [cq.Workplane(obj=mount_solid)]
    cuts.append(cyl(o + z * -30.3, o + z * -21.0, 23.0))  # flange disc (OD 42) + M3 heads + tool room
    cuts.append(cyl(o + z * -21.5, o + z * 0.0, 17.0))  # housing above the flange (OD 31) + base cap
    cuts.append(hull_solid(HERE.parent / "out" / "thumb" / "thumb_base_sweep_hull.stl", grow=1.0))
    for b in ti["mount"]["bolts"]:
        p = np.array(b["point"])
        cuts.append(cyl(p + z * 0.5, p - z * 6.5, 2.0))  # M3 heat-set insert (4.0 x 5.7)
    return cuts


def hull_solid(stl, grow=0.0):
    """Convex STL hull -> CadQuery solid (grown along the vertex normals)."""
    import trimesh
    m = trimesh.load(str(stl))
    h = m.convex_hull
    v = h.vertices + h.vertex_normals * grow
    h = trimesh.Trimesh(v, h.faces).convex_hull
    faces = []
    for f in h.faces:
        pts = [V(*h.vertices[i]) for i in f]
        faces.append(cq.Face.makeFromWires(cq.Wire.makePolygon(pts + [pts[0]])))
    sh = cq.Shell.makeShell(faces)
    return cq.Workplane(obj=cq.Solid.makeSolid(sh).fix())


def wrist_cuts():
    """Clamp pocket, 4 flange screws (M3 x 8 from inside the palm: 3.4 through z 15-19.5, 6.5 head pocket),
    service openings: x +17 into the thumb bay (screw in before the thumb), x -17 dorsal under the back plate,
    x -17 palmar under the palm pad."""
    cuts = [box(-11.45, 11.45, -8.95, 8.95, P.PALM_Z_BASE - 1, P.SHEATH_CLAMP_Z[1] + 0.3)]
    for bx, by in P.WRIST_FLANGE_BOLTS:
        cuts.append(cyl((bx, by, P.PALM_Z_BASE - 1), (bx, by, 19.6), 1.7))
        cuts.append(cyl((bx, by, 19.5), (bx, by, 30.0), 3.25))
        if bx < 0:  # open to the dorsal / palmar face (hatch)
            cuts.append(box(bx - 3.25, bx + 3.25, by, 20 * np.sign(by), 19.5, 30.0))
    return cuts


def channel_solids(routes, which="bore"):
    """PTFE bores (2.2) / steel-guide bores (1.8) along each palm path; free span excluded for bores."""
    out = {}
    for s in routes:
        p = s["dense"]
        if s["material"] == "ptfe":
            r = P.SHEATH_BORE / 2 if which == "bore" else P.SHEATH_OD / 2
        else:
            r = 0.9 if which == "bore" else 1.27 / 2
        if which == "bore" and s["kind"] == "free":
            g = np.array(s["guide"])
            i = int(np.argmin(np.linalg.norm(p - g, axis=1)))
            p = p[max(i - 3, 0):]  # channel starts ~1 mm before the guide (inside the pocket)
        q = ext(p, 0.0, 0.6) if which == "bore" else p
        out[s["name"]] = tube_solid(q, r)
    return out


def idler_parts(routes):
    parts = []
    for s in routes:
        if "idler" not in s:
            continue
        c, k = np.array(s["idler"]["centre"]), np.array(s["idler"]["axis"])
        parts.append(cyl(c - k * 1.6, c + k * 1.6, 4.0))  # pulley (MR63 + tyre)
        parts.append(cyl(c - k * 5.0, c + k * 5.0, 1.5))  # 3 mm axle
    return parts


def idler_cuts(routes):
    cuts = []
    for s in routes:
        if "idler" not in s:
            continue
        c, k = np.array(s["idler"]["centre"]), np.array(s["idler"]["axis"])
        cuts.append(cyl(c - k * 2.1, c + k * 2.1, 4.8))
        cuts.append(cyl(c - k * 5.0, c + k * 5.0, 1.45))
        # bare strand from the socket mouth to the pulley
        e = np.array(s["mount_exit"])
        a = np.array(s["idler"]["contact"])
        cuts.append(cyl(e - (a - e) * 0.3, a + (a - e) * 0.2, 1.2))
    return cuts


# ----------------------------------------------------------------------------------------------------
# build
# ----------------------------------------------------------------------------------------------------
_CACHE = {}


def build(routes_path=OUT / "routes.json", verbose=True):
    """-> {"parts": {...}, "interface": {...}} in the palm frame (= hand frame at wrist q = 0)."""
    t0 = time.time()
    import routing
    import thumb
    routes, rdata = routing.load(routes_path)
    fi = finger_iface()
    ti = thumb_iface()
    mount = thumb.rest_parts()["thumb_mount"]
    mount_solid = mount.val() if hasattr(mount, "val") else mount

    shell = make_core(fi).union(thenar_heel())
    t1 = time.time()
    cuts = yoke_cuts(fi) + pin_bores(fi) + thumb_cuts(ti, mount_solid) + wrist_cuts() + idler_cuts(routes)
    # free-span pockets (sheath bending room under each yoke)
    for f in FINGERS:
        fs = [s for s in routes if s["kind"] == "free" and s["name"].startswith(f)]
        paths = []
        for s in fs:
            p = s["dense"]
            g = np.array(s["guide"])
            i = int(np.argmin(np.linalg.norm(p - g, axis=1)))
            paths.append(p[: i + 1])
        lo = np.array(fi[f]["interface"]["yoke_envelope"]["min"])
        cuts.append(pocket_from_paths(paths, pad=1.8, zmax=lo[2] + 1.0))
    bores = channel_solids(routes, "bore")
    cutter = None
    for c in cuts + [cq.Workplane(obj=b) for b in bores.values()]:
        cutter = c if cutter is None else cutter.union(c, clean=False)
    shell = shell.cut(cutter)
    shell = shell.cut(heel_cutter())
    t2 = time.time()
    # back plate: dorsal skin over the central palm, separated from the shell
    bp_region = rbox(-42.0, 6.0, BACK_PLATE_Y, 20.0, 24.0, 63.0, 6.0, "|Y")
    back_plate = shell.intersect(bp_region)
    shell = shell.cut(bp_region)
    screws = [(-38.0, 28.0), (-38.0, 59.0), (-14.0, 60.5), (2.0, 59.0), (2.0, 28.0), (-20.0, 27.0)]
    for sx, sz in screws:
        back_plate = back_plate.cut(cyl((sx, BACK_PLATE_Y - 1, sz), (sx, 20, sz), 1.1))
        shell = shell.cut(cyl((sx, BACK_PLATE_Y - 6, sz), (sx, BACK_PLATE_Y + 0.1, sz), 0.8))
    # TPU pads
    pads = make_pads(fi, shell)
    parts = {"palm_shell": shell, "palm_back_plate": back_plate, **pads}
    # tubes (visible parts)
    tubes = channel_solids(routes, "tube")
    ptfe = [tubes[s["name"]] for s in routes if s["material"] == "ptfe"]
    steel = [tubes[s["name"]] for s in routes if s["material"] == "steel"]
    parts["palm_sheaths"] = cq.Workplane(obj=cq.Compound.makeCompound(ptfe))
    parts["thumb_guides"] = cq.Workplane(obj=cq.Compound.makeCompound(steel))
    ip = idler_parts(routes)
    parts["rot_idlers"] = cq.Workplane(obj=cq.Compound.makeCompound([p.val() for p in ip]))
    if verbose:
        print(f"  palm: body {t1 - t0:.1f}s, cuts {t2 - t1:.1f}s, total {time.time() - t0:.1f}s")
    return {"parts": parts, "routes": routes, "cell_map": rdata["cell_map"],
            "interface": {"back_plate_screws": screws, "print": {
                "palm_shell": "lying on its back (dorsal face down), supports only under the knuckle lips",
                "palm_back_plate": "inner face down", "pads": "TPU 95A, outer face up"}}}


def make_pads(fi, shell):
    face = Y_FACE
    palm_pad = rbox(-30.0, 8.0, face - 2.0, face + 0.01, 26.0, 60.0, 8.0, "|Y")
    try:
        palm_pad = palm_pad.faces("<Y").edges().fillet(1.5)
    except Exception:
        pass
    # hypothenar: wedge, 4 mm at the ulnar edge -> 1 mm at x -30
    prof = [(-51.0, face + 0.01), (-30.0, face + 0.01), (-30.0, face - 1.0), (-40.0, face - 3.2), (-51.0, face - 4.0)]
    hyp = (cq.Workplane("XY").polyline(prof).close().extrude(38.0).translate((0, 0, 20.0)))
    try:
        hyp = hyp.edges("|Z").fillet(0.8)
    except Exception:
        pass
    hyp = hyp.intersect(rbox(-50.5, -29.0, -25, 0, 20.0, 58.0, 8.0, "|Y"))
    lip = xz_prism(lip_xz(fi), LIP_PALMAR - 1.5, LIP_PALMAR + 0.01)
    lip = lip.intersect(box(-60, 40, -30, 0, 0, 200))
    try:
        lip = lip.faces("<Y").edges().fillet(0.6)
    except Exception:
        pass
    # keep pads out of the shell's openings (pin bores stay covered: pads are glued over the dowel ends)
    return {"palm_pad": palm_pad, "hypothenar_pad": hyp, "knuckle_pad": lip}


if __name__ == "__main__":
    from tools.checks import is_valid
    from tools.render import render
    b = build()
    for n, s in b["parts"].items():
        print(n, {k: (round(v, 1) if isinstance(v, float) else v) for k, v in is_valid(s).items()})
    render(b["parts"], OUT / "palm_only.png", views=("front", "back", "side", "iso"), size=(1800, 1400),
           colors={"palm_shell": "#c9d3df", "palm_back_plate": "#a9b8cc", "palm_pad": "#3d4650", "hypothenar_pad": "#3d4650",
                   "knuckle_pad": "#3d4650", "palm_sheaths": "#f4f4f4", "thumb_guides": "#9aa0a6", "rot_idlers": "#666"},
           title="palm (integrator)")
