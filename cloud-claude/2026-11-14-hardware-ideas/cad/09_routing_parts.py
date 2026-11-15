"""09 · Bare-strand routing parts in CadQuery (study 2026-11-14, agent 09).

Parts (V2 frame, mm; built at q = 0 in their own body frame):
- ring body (moves with wrist_dev):  eye_plate (stainless 1.2, 32 torus eyes), eye_block_R / eye_block_L
  (PAHT-CF, screw to the inner faces of the V2 ring's flex-axis bosses at x = +-11).
- palm body: palm_plate (= eye plate copy at z +22) + palm_plate_carrier (PETG), index_trough + index_trough_lid (open
  troughs, screwed lid, comb -> knuckle), index_idler_block (2 x MR63ZZ on 2 x Ø3 x 8 dowels) + the bought parts.
- forearm body: forearm_plate (= eye plate copy at z -30) + forearm_plate_carrier (PAHT-CF, two legs to the
  forearm frame top ring).

Reads out/09/eyeplate_kin.json (eye table from cad/09_eyeplate_kin.py). Writes STL + STEP per part to out/09/,
out/09/checks.json (validity, interference with the V2 wrist parts over the 9 x 9 wrist grid, strand clearance)
and renders out/09/*.png.

    uv run --no-project --with cadquery --with numpy --with trimesh --with scipy --with rtree --with matplotlib \
        --with pillow python cad/09_routing_parts.py [--quick]
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
OUT = STUDY / "out" / "09"
sys.path.insert(0, str(V2CAD))
import wrist as W  # noqa: E402  (V2 wrist, read-only use)
import wrist_kin as WK  # noqa: E402
from tools.checks import interference, is_valid  # noqa: E402

spec = importlib.util.spec_from_file_location("kin09", HERE / "09_eyeplate_kin.py")
KIN = importlib.util.module_from_spec(spec)
spec.loader.exec_module(KIN)

QUICK = "--quick" in sys.argv
DATA = json.load(open(OUT / "eyeplate_kin.json"))
OPT = DATA["options"][DATA["chosen"]]["opt"]
TABLE = DATA["eye_table"]

# --- dimensions ---------------------------------------------------------------------------------------
PLATE_T = 2 * OPT["r_lip"] - 0.04  # lip meets the faces at a small angle (no tangent knife edge); 1.2 stainless 304 (cold rolled), laser cut or drilled with the printed jig
PLATE_X, PLATE_Y = 10.9, 6.0  # half sizes (all three plates identical: drill them as one stack)
R0 = OPT["r_throat"] + OPT["r_lip"]  # torus major radius (mouth radius at the faces)
BOSS_FACE_X = W.XBOSS[0]  # 11.0: inner face of the ring's flex-axis bosses (r 6.6 about X)
BLOCK = dict(x0=8.0, y=5.0, z0=PLATE_T / 2, z1=3.6)  # eye blocks: x 8.8..11, |y| <= 5, z 0.6..3.6 (r <= 6.2)
M16_CLEAR, M16_PILOT, M16_HEAD = 1.8, 1.3, 3.2
PLATE_SCREW_X = 9.2
LINER_OD = 1.65  # 16G stainless tube, glued in an aimed Ø1.7 hole
H_PALM, H_FORE = KIN.H_PALM, KIN.H_FORE  # plate mid-planes (palm frame / forearm frame)
CARRIER_T = 3.0
FRAME_TOP_UNDERSIDE = -22.0  # V2 forearm frame top ring z -22..-18
M2_CLEAR, M2_PILOT = 2.2, 1.7
MR63 = dict(id=3.0, od=6.0, w=2.5)
LINE_R = KIN.LINE_D / 2
# Index finger knuckle entry (V2 params): MCP (20, -2, 92), abd axis 11.5 mm proximal -> z 80.5, along Y.
X_F = 20.0
Z_ABD = 92.0 - 11.5
Z_IDLER = Z_ABD  # roller exit tangent ON the abd axis (a = 0): abd common mode +-0.01 mm per strand vs -0.11 at a = 4.5 (V2 yoke exit z 76)
X_DOWEL = X_F - (MR63["od"] / 2 + LINE_R)  # strand centre tangent at x = X_F (on the abd axis plane)
Y_BRG = {"flex": -9.0, "ext": 5.0}  # mcp_flex flex / ext strand y at the knuckle entry (V2 routes)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def cyl(p0, p1, r):
    return W.cyl(p0, p1, r)


def rot(wp, R):
    return W.rot_shape(wp, R)


# --- ring body -----------------------------------------------------------------------------------------
def make_eye_plate():
    pl = box(-PLATE_X, PLATE_X, -PLATE_Y, PLATE_Y, -PLATE_T / 2, PLATE_T / 2).edges("|Z").fillet(1.5)
    eyes = {tuple(np.round(r["eye"], 3)) for r in TABLE}
    rl = OPT["r_lip"]
    for ex, ey in eyes:
        hole = cq.Workplane("XY").circle(R0 + 0.01).extrude(PLATE_T + 0.02).translate((0, 0, -PLATE_T / 2 - 0.01))
        tor = cq.Workplane("XY").add(cq.Solid.makeTorus(R0, rl))
        cutter = hole.cut(tor).translate((ex, ey, 0))
        pl = pl.cut(cutter)
    for sx in (-1, 1):  # 2 x M1.6 countersunk (ISO 10642) from the plate face into the block / carrier
        c = (sx * PLATE_SCREW_X, 0.0)
        pl = pl.cut(cyl((c[0], c[1], -1), (c[0], c[1], 1), M16_CLEAR / 2))
        pl = pl.cut(cq.Workplane("XY").add(cq.Solid.makeCone(1.55, M16_CLEAR / 2 + 0.05, 0.6))
                    .translate((c[0], c[1], -PLATE_T / 2 - 0.01)))
    return pl


def make_eye_block(sx):
    b = BLOCK
    x0, x1 = sorted((sx * b["x0"], sx * BOSS_FACE_X))
    blk = box(x0, x1, -b["y"], b["y"], b["z0"], b["z1"]).edges("|X").fillet(1.0)
    # relief for the flex pin end (rotates with the palm): r 2.2, 0.6 deep at the boss face
    xf = sx * BOSS_FACE_X
    blk = blk.cut(cyl((xf, 0, 0), (xf - sx * 0.6, 0, 0), 2.2))
    for sy in (-1, 1):  # 2 x M1.6 x 4 socket head along X into the boss (pilot Ø1.3 drilled in the ring boss)
        y, z = sy * 3.6, 1.9 + 0.3
        blk = blk.cut(cyl((sx * (b["x0"] - 1), y, z), (sx * (BOSS_FACE_X + 1), y, z), M16_CLEAR / 2))
        blk = blk.cut(cyl((sx * (b["x0"] - 1), y, z), (sx * (b["x0"] + 1.0), y, z), M16_HEAD / 2 - 0.0))
    blk = blk.cut(cyl((sx * PLATE_SCREW_X, 0, 0), (sx * PLATE_SCREW_X, 0, 3.0), M16_PILOT / 2))  # plate screw pilot
    return blk


# --- palm body -----------------------------------------------------------------------------------------
def outline_plate(pts_xy, z0, z1, margin, extra=()):
    """Plate covering the points (convex hull + margin), rounded."""
    from scipy.spatial import ConvexHull

    pts = np.vstack([pts_xy] + [np.array(e) for e in extra]) if extra else np.asarray(pts_xy)
    ang = np.linspace(0, 2 * math.pi, 16, endpoint=False)
    ring = np.vstack([pts + margin * np.c_[np.cos(a), np.sin(a)] for a in ang])
    h = ConvexHull(ring)
    poly = [tuple(map(float, ring[i])) for i in h.vertices]
    return cq.Workplane("XY").workplane(offset=z0).polyline(poly).close().extrude(z1 - z0)


def aimed_holes(plate, rows, key, axis_key, zc, d):
    for r in rows:
        c = np.array(r[key], float)
        a = np.array(r[axis_key], float)
        a = a / np.linalg.norm(a)
        # comb hole centre on the comb mid-plane, along the strand line
        p0, p1 = c - 4 * a, c + 4 * a
        plate = plate.cut(cyl(p0, p1, d / 2))
    return plate


def fan_hull(z_plate_face, z_far, rows, side, margin=1.0):
    """Convex hulls (at the plate face and at z_far) of the fan lines beyond a plate, for the carrier window."""
    from scipy.spatial import ConvexHull

    secs = []
    for z in (z_plate_face, z_far):
        pts = []
        for r in rows:
            E = np.array(r["eye"] + [0.0])
            if side == "palm":
                P0, P1 = E + [0, 0, H_PALM], np.array(r["anchor"], float)
            else:
                P0, P1 = E + [0, 0, H_FORE], np.array(r["spool_tangent"], float)
            t = (z - P0[2]) / (P1[2] - P0[2])
            q = P0 + t * (P1 - P0)
            for a_ in np.linspace(0, 2 * math.pi, 12, endpoint=False):
                pts.append((q[0] + (R0 + margin) * math.cos(a_), q[1] + (R0 + margin) * math.sin(a_)))
        pts = np.array(pts)
        h = ConvexHull(pts)
        secs.append((z, [tuple(map(float, pts[i])) for i in h.vertices]))
    return secs


def loft_poly(secs):
    w0 = cq.Workplane("XY").workplane(offset=secs[0][0]).polyline(secs[0][1]).close().wires().val()
    w1 = cq.Workplane("XY").workplane(offset=secs[1][0]).polyline(secs[1][1]).close().wires().val()
    return cq.Workplane("XY").add(cq.Solid.makeLoft([w0, w1], True))


def make_plate_carrier(side):
    """Printed carrier for the palm plate (above it, palm frame) or the forearm plate (below it, forearm frame).
    The plate is screwed to it with 4 x M1.6 countersunk from the strand-free face."""
    if side == "palm":
        z0 = H_PALM + PLATE_T / 2
        z1 = z0 + CARRIER_T
        zf, zfar = z0 - 0.01, z1 + 0.01
    else:
        z1 = H_FORE - PLATE_T / 2
        z0 = z1 - CARRIER_T
        zf, zfar = z1 + 0.01, z0 - 0.01
    body = box(-PLATE_X - 1.5, PLATE_X + 1.5, -PLATE_Y - 1.5, PLATE_Y + 1.5, z0, z1).edges("|Z").fillet(2.0)
    if side == "palm":
        for sx in (-1, 1):  # ears to palm bosses, outside the wrist heel zone (|x| > 14.5)
            ear = box(min(sx * 10, sx * 19.5), max(sx * 10, sx * 19.5), -3.0, 3.0, z0, z1)
            body = body.union(ear).cut(cyl((sx * 17.0, 0, z0 - 1), (sx * 17.0, 0, z1 + 1), M2_CLEAR / 2))
    else:
        for sy in (-1, 1):  # legs up to the forearm frame top ring (2 x M2 heat-set inserts in its underside)
            y0, y1 = sorted((sy * (PLATE_Y + 1.0), sy * 21.5))
            body = body.union(box(-3.0, 3.0, y0, y1, z0, z1))
            y0, y1 = sorted((sy * 17.5, sy * 21.5))
            body = body.union(box(-3.0, 3.0, y0, y1, z0, FRAME_TOP_UNDERSIDE))
            body = body.cut(cyl((0, sy * 19.5, z0 - 1), (0, sy * 19.5, FRAME_TOP_UNDERSIDE + 1), M2_CLEAR / 2))
    win = loft_poly(fan_hull(zf, zfar, TABLE, side))
    body = body.cut(win)
    zc = z0 if side == "palm" else z1
    for sx in (-1, 1):
        body = body.cut(cyl((sx * PLATE_SCREW_X, 0, zc - 0.1), (sx * PLATE_SCREW_X, 0, zc + (2.6 if side == "palm" else -2.6)),
                            M16_PILOT / 2))
    return body


def index_lines():
    """Palm-frame strand lines for the index finger: palm comb point -> knuckle entry."""
    out = {}
    for r in TABLE:
        if r["name"].startswith("index"):
            C = np.array(r["palm_comb"], float)
            A = np.array(r["anchor"], float)
            if "mcp_flex" in r["name"]:  # ends on the idler: tangent point at x = X_F, z = Z_IDLER
                A = np.array([X_F, A[1], Z_IDLER])
            out[r["name"]] = (C, A)
    return out


def make_trough():
    lines = index_lines()
    z0, z1 = H_PALM + PLATE_T / 2 + CARRIER_T + 0.5, 71.0
    allp = []
    for C, A in lines.values():
        for z in (z0, z1):
            t = (z - C[2]) / (A[2] - C[2])
            allp.append(C + t * (A - C))
    allp = np.array(allp)
    y_top = float(allp[:, 1].max()) + 1.6
    y_bot = float(allp[:, 1].min()) - 2.2
    xs = allp[:, 0]
    base = box(float(xs.min()) - 2.6, float(xs.max()) + 2.6, y_bot, y_top, z0, z1)
    w = 1.3  # slot width (line 0.47: free, it does not touch the walls when straight)
    for C, A in lines.values():
        secs = []
        for z in (z0 - 0.5, z1 + 0.5):
            t = (z - C[2]) / (A[2] - C[2])
            P = C + t * (A - C)
            secs.append(cq.Workplane("XY").workplane(offset=z).center(P[0], (P[1] - 0.65 + y_top + 1) / 2)
                        .rect(w, y_top + 1 - (P[1] - 0.65)))
        slot = cq.Workplane("XY").add(cq.Solid.makeLoft([secs[0].wires().val(), secs[1].wires().val()], True))
        base = base.cut(slot)
    # lid screws: 4 x M2 pilots in the walls beside the grooves (chosen where the wall is thick)
    holes = [(float(xs.min()) - 1.3, z0 + 4), (float(xs.min()) - 1.3, z1 - 4), (float(xs.max()) + 1.3, z0 + 4),
             (float(xs.max()) + 1.3, z1 - 4)]
    for hx, hz in holes:
        base = base.cut(cyl((hx, y_top - 6, hz), (hx, y_top + 1, hz), M2_PILOT / 2))
    lid = box(float(xs.min()) - 2.6, float(xs.max()) + 2.6, y_top, y_top + 1.6, z0, z1)
    for hx, hz in holes:
        lid = lid.cut(cyl((hx, y_top - 1, hz), (hx, y_top + 3, hz), M2_CLEAR / 2))
    return base, lid, dict(y_top=y_top, y_bot=y_bot, z=(z0, z1), x=(float(xs.min()) - 2.6, float(xs.max()) + 2.6))


def make_idler_block():
    """2 x MR63ZZ on 2 x Ø3 x 8 dowels along Y (parallel to the abd axis), roller rims at x = X_F."""
    parts = {}
    blk = None
    for role, yc in Y_BRG.items():
        y_b0, y_b1 = yc - MR63["w"] / 2 - 0.5, yc + MR63["w"] / 2 + 0.5  # bearing + 2 x 0.5 washers
        for y0, y1 in ((y_b0 - 2.0, y_b0), (y_b1, y_b1 + 2.0)):  # cheeks
            ch = box(X_DOWEL - 3.5, X_F + 0.6, y0, y1, Z_IDLER - 3.0, Z_IDLER + 3.0)
            blk = ch if blk is None else blk.union(ch)
        spine = box(X_DOWEL - 5.5, X_DOWEL - 3.5, y_b0 - 2.0, y_b1 + 2.0, Z_IDLER - 3.0, Z_IDLER + 3.0)
        blk = blk.union(spine)
        dowel = cyl((X_DOWEL, y_b0 - 2.0 + 0.0, Z_IDLER), (X_DOWEL, y_b0 - 2.0 + 8.0, Z_IDLER), 1.5)
        parts[f"index_idler_dowel_{role}"] = dowel
        brg = cyl((X_DOWEL, yc - MR63["w"] / 2, Z_IDLER), (X_DOWEL, yc + MR63["w"] / 2, Z_IDLER), MR63["od"] / 2).cut(
            cyl((X_DOWEL, yc - 2, Z_IDLER), (X_DOWEL, yc + 2, Z_IDLER), MR63["id"] / 2))
        parts[f"bearing_MR63_index_{role}"] = brg
        for yw in (y_b0, y_b1 - 0.5):
            wsh = cyl((X_DOWEL, yw, Z_IDLER), (X_DOWEL, yw + 0.5, Z_IDLER), 2.5).cut(
                cyl((X_DOWEL, yw - 1, Z_IDLER), (X_DOWEL, yw + 2, Z_IDLER), 1.6))
            parts[f"washer_M3_index_{role}_{yw:+.1f}"] = wsh
    # spine bridge between the two stations (leaves the pip slot y -5.5..1.25 open at x > X_DOWEL - 3.5)
    yA = Y_BRG["flex"] + MR63["w"] / 2 + 2.5
    yB = Y_BRG["ext"] - MR63["w"] / 2 - 2.5
    blk = blk.union(box(X_DOWEL - 5.5, X_DOWEL - 3.5, yA, yB, Z_IDLER - 3.0, Z_IDLER + 3.0))
    # dowel bores (press Ø2.95) through the cheeks
    for role, yc in Y_BRG.items():
        blk = blk.cut(cyl((X_DOWEL, yc - 6, Z_IDLER), (X_DOWEL, yc + 6, Z_IDLER), 1.475))
        # bearing + washer gap (clearance 0.6 radially around the bearing)
        blk = blk.cut(cyl((X_DOWEL, yc - MR63["w"] / 2 - 0.5, Z_IDLER), (X_DOWEL, yc + MR63["w"] / 2 + 0.5, Z_IDLER), 3.6))
    # 2 x M2 through the spine along X into the palm
    for yy in (Y_BRG["flex"], Y_BRG["ext"]):
        blk = blk.cut(cyl((X_DOWEL - 7, yy, Z_IDLER), (X_DOWEL - 2.5, yy, Z_IDLER), M2_CLEAR / 2))
    parts["index_idler_block"] = blk
    return parts


# --- checks ----------------------------------------------------------------------------------------------
def body_of(name):
    if name.startswith("eye_"):
        return "ring"
    if name.startswith("forearm"):
        return "fore"
    return "palm"


def place(parts, f, d):
    Rr, Rp = KIN.R_ring(f, d), KIN.R_palm(f, d)
    R = {"ring": Rr, "palm": Rp, "fore": np.eye(3)}
    return {n: rot(s, R[body_of(n)]) for n, s in parts.items()}


def strand_polylines(f, d):
    strands = KIN.load_strands()
    by = {r["name"]: np.array(r["eye"]) for r in TABLE}
    eyes = [by[s["name"]] for s in strands]
    return strands, KIN.fast_polylines(strands, eyes, (f, d))


def strand_clearance(v2_meshes, f, d):
    """Smallest distance from any strand centreline (sampled every 0.5 mm, between the forearm comb and the
    palm comb) to each V2 wrist part, minus the line radius."""
    import trimesh

    strands, polys = strand_polylines(f, d)
    pts = []
    for pl in polys:
        for a, b in zip(pl[1:3], pl[2:4]):  # F -> E -> C (the free spans across the wrist)
            n = max(2, int(np.linalg.norm(b - a) / 0.5))
            pts.append(a + np.linspace(0.03, 0.97, n)[:, None] * (b - a))
    pts = np.vstack(pts)
    out = {}
    for name, m in v2_meshes.items():
        dmin = trimesh.proximity.closest_point(m, pts)[1].min()
        out[name] = float(dmin - LINE_R)
    return out


def to_mesh(shape):
    import trimesh
    from tools.render import tessellate

    v, fc = tessellate(shape, 0.1, 0.3)
    return trimesh.Trimesh(v, fc, process=True)


def abd_crossing_error():
    """2D: strand wraps a roller (centreline radius Rc) whose exit tangent is at x = 0; the abd axis is a
    distance a above the tangent point; the finger-side anchor b = 11.5 above the axis rotates by alpha."""
    out = {}
    Rc = MR63["od"] / 2 + LINE_R
    for a in (0.0, 4.5):
        res = []
        for al in np.radians([-15, -7.5, 7.5, 15]):
            C = np.array([-Rc, 0.0])  # roller centre; tangent point (0, 0); axis at (0, a)
            ax = np.array([0.0, a])
            B0 = np.array([0.0, a + 11.5])
            Rm = np.array([[math.cos(al), -math.sin(al)], [math.sin(al), math.cos(al)]])
            B = ax + Rm @ (B0 - ax)
            # far end of the wrap: fixed point on the roller below, at angle -20 deg (incoming strand)

            def L(Bp):
                dvec = Bp - C
                dist = np.linalg.norm(dvec)
                t = math.sqrt(max(dist**2 - Rc**2, 0))
                # tangent point angle (strand wraps on the +x side, going up)
                phi = math.atan2(dvec[1], dvec[0]) - math.acos(Rc / dist)
                return t + Rc * (phi + math.radians(20))

            res.append(L(B) - L(B0))
        out[f"a={a}"] = [round(float(r), 4) for r in res]
    return out


def main():
    t0 = time.time()
    OUT.mkdir(parents=True, exist_ok=True)
    plate = make_eye_plate()
    parts = {"eye_plate": plate, "eye_block_R": make_eye_block(1), "eye_block_L": make_eye_block(-1),
             "palm_plate": plate.translate((0, 0, H_PALM)), "palm_plate_carrier": make_plate_carrier("palm"),
             "forearm_plate": plate.translate((0, 0, H_FORE)), "forearm_plate_carrier": make_plate_carrier("fore")}
    tr, lid, tinfo = make_trough()
    parts["index_trough"] = tr
    parts["index_trough_lid"] = lid
    parts.update(make_idler_block())
    print(f"built {len(parts)} parts in {time.time()-t0:.0f} s")
    checks = dict(parts={}, notes=[])
    for n, s in parts.items():
        v = is_valid(s)
        bb = s.val().BoundingBox()
        checks["parts"][n] = dict(v, bbox=[round(bb.xmin, 2), round(bb.xmax, 2), round(bb.ymin, 2), round(bb.ymax, 2),
                                          round(bb.zmin, 2), round(bb.zmax, 2)])
        print(f"  {n:34s} valid {v['valid']} watertight {v['watertight']} vol {v['volume']:.1f}")
        if not n.startswith(("bearing", "washer", "index_idler_dowel")):
            cq.exporters.export(s, str(OUT / f"{n}.stl"))
            cq.exporters.export(s, str(OUT / f"{n}.step"))
    # own interference at q = 0
    own = interference(parts, threshold=0.05, ignore=[("index_idler_dowel_*", "index_idler_block"),
                                                       ("index_idler_dowel_*", "bearing_*"), ("washer_*", "index_idler_dowel_*"),
                                                       ("index_trough", "index_trough_lid"), ("eye_plate", "eye_block_*"),
                                                       ("palm_plate", "palm_plate_carrier"), ("forearm_plate", "forearm_plate_carrier")])
    checks["own_interference_q0"] = own
    print("own interference:", own or "none")
    # radial clearances to the moving bodies (analytic): ring-body parts vs palm sweep (r_x >= 10) and forearm (r_y >= 15)
    def max_r(shape, axis):
        m = to_mesh(shape)
        v = m.vertices
        return float(np.hypot(v[:, 1], v[:, 2]).max() if axis == "x" else np.hypot(v[:, 0], v[:, 2]).max())
    rr = {n: dict(r_from_flex_axis_in_x_lt_11p5=max_r(parts[n], "x"), r_from_dev_axis=max_r(parts[n], "y"))
          for n in ("eye_plate", "eye_block_R", "eye_block_L")}
    checks["ring_parts_radii"] = rr
    checks["notes"].append("V2 palm material in |x| < 11.5 stays at r_x >= 10 from the flex axis; V2 forearm material at r >= 15 "
                           "from the dev axis: ring-body parts must stay inside these radii.")
    # wrist grid
    grid = KIN.wrist_grid(3 if QUICK else 9)
    W._CACHE.clear()
    res, sclear = {}, {}
    t1 = time.time()
    v2_static = None
    worst_clear = {}
    for k, (f, d) in enumerate(grid):
        b = W.build({"wrist_flex": f, "wrist_dev": d}, bundle=False)["parts"]
        v2 = {f"V2_{n}": s for n, s in b.items() if n != "palm_sheath_clamp"}
        mine = place(parts, f, d)
        allp = dict(v2)
        allp.update(mine)
        ig = [("V2_pin_*", "V2_wrist_base"), ("V2_pin_*", "V2_wrist_ring"), ("V2_pin_*", "V2_palm_flange"),
              ("V2_wrist_servo_*", "V2_wrist_gears_*"), ("V2_wrist_gears_*", "V2_wrist_crank_*"),
              ("V2_wrist_rod_*", "V2_wrist_crank_*"), ("V2_wrist_rod_*", "V2_palm_flange"),
              ("eye_block_*", "V2_wrist_ring")]
        for a_ in ("V2_*",):
            pass
        # only pairs involving at least one of my parts
        r = interference(allp, threshold=0.5, ignore=ig)
        r = [x for x in r if not (x[0].startswith("V2_") and x[1].startswith("V2_"))]
        r = [x for x in r if not (x[0] in parts and x[1] in parts)]
        res[f"{f:.1f},{d:.1f}"] = r
        # strand clearance to the V2 parts near the wrist
        meshes = {n: to_mesh(s) for n, s in v2.items() if any(t in n for t in ("wrist_base", "wrist_ring", "palm_flange", "pin_"))}
        sc = strand_clearance(meshes, f, d)
        sclear[f"{f:.1f},{d:.1f}"] = sc
        for n, v in sc.items():
            worst_clear[n] = min(worst_clear.get(n, 1e9), v)
        print(f"  pose flex {f:6.1f} dev {d:6.1f}: {'clear' if not r else r}  strand clearance min {min(sc.values()):.2f} "
              f"mm  ({time.time()-t1:.0f} s)")
    checks["wrist_grid_interference"] = res
    checks["wrist_grid_all_clear"] = all(not v for v in res.values())
    checks["strand_clearance_to_V2_wrist_parts_min_mm"] = worst_clear
    checks["strand_clearance_grid"] = sclear
    checks["abd_crossing_common_mode_mm_per_strand"] = abd_crossing_error()
    checks["trough"] = tinfo
    (OUT / "checks.json").write_text(json.dumps(checks, indent=1, default=str))
    print("abd crossing:", checks["abd_crossing_common_mode_mm_per_strand"])
    render_all(parts)
    print(f"done in {time.time()-t0:.0f} s")


def strand_solids(f, d, r=0.35):
    strands, polys = strand_polylines(f, d)
    out = None
    for pl in polys:
        for a, b in zip(pl[:-1], pl[1:]):
            c = cyl(a, b, r)
            out = c if out is None else out.union(c, clean=False)
    return out


def render_all(parts):
    from tools.render import render

    colors = {"eye_plate": "#b0b8c0", "eye_block_R": "#333333", "eye_block_L": "#333333", "palm_plate": "#b0b8c0",
              "forearm_plate": "#b0b8c0", "palm_plate_carrier": "#3a7bd5", "forearm_plate_carrier": "#444444", "index_trough": "#5aa0e0", "index_trough_lid": "#9cc8f0",
              "index_idler_block": "#e08a3a"}
    try:
        render({"eye_plate": parts["eye_plate"], "eye_block_R": parts["eye_block_R"], "eye_block_L": parts["eye_block_L"]},
               str(OUT / "eye_plate.png"), views=("iso", "top", "front", "side"), colors=colors, title="09 eye plate + blocks")
        for f, d, tag in ((0.0, 0.0, "q0"), (50.0, 20.0, "flex+50_dev+20"), (-50.0, -30.0, "flex-50_dev-30")):
            b = W.build({"wrist_flex": f, "wrist_dev": d}, bundle=False)["parts"]
            sc = {f"V2_{n}": s for n, s in b.items() if n != "palm_sheath_clamp" and "servo" not in n and "gears" not in n}
            mine = place({k: v for k, v in parts.items() if k.startswith(("eye_", "palm_plate", "forearm_plate"))}, f, d)
            sc.update(mine)
            sc["strands"] = strand_solids(f, d)
            cols = dict(colors)
            cols["strands"] = "#d62728"
            for n in sc:
                if n.startswith("V2_"):
                    cols[n] = "#cfcfcf"
            render(sc, str(OUT / f"wrist_{tag}.png"), views=("iso", "front", "side", "top"), colors=cols,
                   title=f"09 bare-strand wrist, flex {f:+.0f} dev {d:+.0f} (V2 wrist grey, strands red)")
        idx = {k: v for k, v in parts.items() if k.startswith(("index_", "bearing", "washer"))}
        idx["palm_plate"] = parts["palm_plate"]
        idx["palm_plate_carrier"] = parts["palm_plate_carrier"]
        render(idx, str(OUT / "index_trough_idler.png"), views=("iso", "front", "side", "back"), colors=colors,
               title="09 palm plate + carrier, index trough + lid, knuckle idler block (2 x MR63)")
    except Exception as e:  # pragma: no cover
        print("render failed:", e)


if __name__ == "__main__":
    main()
