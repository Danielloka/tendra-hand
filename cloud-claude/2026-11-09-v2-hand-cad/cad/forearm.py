"""Tendra V2 concept: forearm with the 16 hand servos, spools, tensioners, sheath clamps, the wrist
drive mounts, electronics and the arm flange (study 2026-11-09, report 03).

Layout (hand frame, mm; the forearm runs from the wrist base at z -18 down to the arm flange):
- Two servo "cassettes" (left x < 0, right x > 0), each 4 columns (y) x 2 levels (z) of SCS0009,
  shafts along X pointing at a central spool rail (x +-3). Each spool is split (two discs, one per
  strand, friction-clamped = the tensioner) and runs on a 3 mm stub in the rail (MR63 for level 2,
  a brass bushing post for level 1), so the 2 strands' ~40 N side load does not sit on the servo.
- Level 2 (lower) spools sit 5 mm closer to the rail than level 1, so level-2 strands pass inside
  the level-1 spools. Level 2 takes the six high-load joints (room for HLS3606M, +2.25 mm).
- All 32 strands run straight up (tangent to their spool) to a clamp block of barrel adjusters at
  z -123; from there each strand is inside its PTFE sheath, which fans in to the bundle comb at
  z -60 and runs through the hollow wrist to the palm clamp.
- A servo comes out sideways after removing a side cover and its 2 tab screws: its spline slides out
  of the spool, the spool stays on its rail stub, strung and tensioned.

    uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy \
        --with pillow python cad/forearm.py [--quick]
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
import wrist as WR  # noqa: E402
import wrist_kin as K  # noqa: E402
from wrist import box, cyl, rrect  # noqa: E402

OUT = HERE.parent / "out" / "wrist"

S = P.SCS0009
RAIL_HALF = 3.0
G2, G1 = 5.8, 10.8  # spool groove-centre planes |x| for level 2 (lower) and level 1 (upper)
GROOVE_DX = 1.1  # the two grooves of a split spool sit at g -+ 1.1
SPOOL_FLANGE_R = 6.5
CASE_GAP = 5.0  # groove plane above the servo case top (spline + hub), as V1
COLS = (-20.25, -6.75, 6.75, 20.25)  # column y (pitch 13.5 for a 12.1 wide servo)
Z_S1, Z_S2 = -141.0, -175.0  # spool centre z, level 1 and 2
Z_CLAMP = (-132.0, -123.0)  # barrel-adjuster clamp block
ADJ_RAISE = 6.0  # adjusters of odd columns stand 6 mm higher (head clearance)
Z_COMB = (K.Z_FORE_COMB - 3.0, K.Z_FORE_COMB)
Z_DECK = (-121.0, -118.0)
Z_TRAY = (-216.0, -214.0)
Z_FLANGE = (-230.0, -225.0)
SHELL = (47.5, 33.0, 2.0)  # outer half x, half y, wall (95 x 66: the wrist cranks set the width)
SHELL_TOP = (36.0, 29.5)  # half sizes at the top (z -18); wide enough that the rod balls stay outside
Z_SHELL_TOP, Z_SHELL_WIDE = -18.0, -48.0
ARM_PCD, ARM_BOLTS, ARM_BOLT_D, ARM_BORE = 50.0, 4, 4.5, 24.0  # OpenArm J5 stand-in (verify)

# servo ID -> (side, level, column index); right = +X (thumb side)
SLOTS = {
    4: ("R", 1, 0), 1: ("R", 1, 1), 3: ("R", 1, 2), 10: ("R", 1, 3),
    7: ("R", 2, 0), 6: ("R", 2, 1), 5: ("R", 2, 2), 2: ("R", 2, 3),
    11: ("L", 1, 0), 13: ("L", 1, 1), 14: ("L", 1, 2), 16: ("L", 1, 3),
    8: ("L", 2, 0), 9: ("L", 2, 1), 12: ("L", 2, 2), 15: ("L", 2, 3),
}  # fmt: skip
HLS = {2, 6, 7, 9, 12, 15}  # 4 x mcp_flex, thumb cmc_flex, thumb cmc_rot


def servo_geom(sid):
    side, lvl, col = SLOTS[sid]
    sx = 1 if side == "R" else -1
    g = G1 if lvl == 1 else G2
    zs = Z_S1 if lvl == 1 else Z_S2
    yc = COLS[col]
    top = g + CASE_GAP
    bottom = top + S["H"] + (P.HLS_EXTRA_H if sid in HLS else 0.0)
    zc = zs - S["shaft_offset"]  # shaft near the top end of the case
    return dict(sx=sx, g=g, zs=zs, yc=yc, top=top, bottom=bottom, zc=zc, lvl=lvl, col=col, side=side)


def strand_points(sid):
    """Flex and ext strand: spool tangent point and clamp (adjuster) entry, hand frame."""
    s = servo_geom(sid)
    out = {}
    for name, dx, dy in (("flex", -GROOVE_DX, +1), ("ext", +GROOVE_DX, -1)):
        x = s["sx"] * (s["g"] + dx)
        y = s["yc"] + dy * P.R_SPOOL
        ztop = Z_CLAMP[1] + (ADJ_RAISE if s["col"] % 2 else 0.0)
        out[name] = dict(tangent=(x, y, s["zs"]), clamp_in=(x, y, Z_CLAMP[0]), sheath_start=(x, y, ztop + 1.5))
    return out


# --- parts -----------------------------------------------------------------------------------------
def make_servo(sid):
    s = servo_geom(sid)
    sx = s["sx"]
    x0, x1 = sorted((sx * s["top"], sx * s["bottom"]))
    body = box(x0, x1, s["yc"] - S["W"] / 2, s["yc"] + S["W"] / 2, s["zc"] - S["L"] / 2, s["zc"] + S["L"] / 2)
    tz0 = sx * (s["bottom"] - (S["H"] if sid not in HLS else S["H"]) + S["tab_z"] - (P.HLS_EXTRA_H if sid in HLS else 0) * 0)
    # tabs: S["tab_z"] above the case bottom (for HLS the extra height is assumed below the tabs)
    tab_lo = s["bottom"] - S["tab_z"] - S["tab_T"]
    t0, t1 = sorted((sx * tab_lo, sx * (tab_lo + S["tab_T"])))
    tabs = box(t0, t1, s["yc"] - S["tab_W"] / 2, s["yc"] + S["tab_W"] / 2, s["zc"] - S["tab_L"] / 2, s["zc"] + S["tab_L"] / 2)
    spline = cyl((sx * s["top"], s["yc"], s["zs"]), (sx * (s["top"] - 3.2), s["yc"], s["zs"]), 2.0)
    del tz0
    return body.union(tabs).union(spline)


def make_spool(sid):
    """Split spool: disc A (spline hub, flex strand) + disc B (ext strand), each a 1 mm groove on
    r = R_SPOOL with 6.5 flanges, locked together by an M2 screw through a serrated face (tensioner),
    3 mm steel stub on the rail side."""
    s = servo_geom(sid)
    sx, g = s["sx"], s["g"]
    y, z = s["yc"], s["zs"]
    parts = None
    for k, xc in enumerate((g - GROOVE_DX, g + GROOVE_DX)):
        x0, x1 = xc - 1.1, xc + 1.1
        d = cyl((sx * x0, y, z), (sx * x1, y, z), SPOOL_FLANGE_R)
        d = d.cut(cyl((sx * (xc - 0.5), y, z), (sx * (xc + 0.5), y, z), SPOOL_FLANGE_R + 1).cut(
            cyl((sx * (xc - 0.6), y, z), (sx * (xc + 0.6), y, z), P.R_SPOOL - P.LINE_D / 2)))
        d = d.cut(cyl((sx * (xc - 2), y + 3.2, z), (sx * (xc + 2), y + 3.2, z), 0.6))  # tie / anchor hole
        parts = d if parts is None else parts.union(d)
    hub = cyl((sx * (g + GROOVE_DX + 1.1), y, z), (sx * s["top"], y, z), 4.0)
    stub_end = RAIL_HALF - 2.5 if s["lvl"] == 2 else RAIL_HALF + 0.3
    stub = cyl((sx * (g - GROOVE_DX - 1.1), y, z), (sx * stub_end, y, z), 1.5)
    return parts.union(hub).union(stub)


def make_rail():
    """Central spool rail: x +-3, carries MR63 seats (level 2) and bushing posts (level 1) for every
    spool on both sides, plus a bus-cable channel."""
    zt, zb = Z_CLAMP[0], Z_S2 - 24.0
    rail = box(-RAIL_HALF, RAIL_HALF, -27.5, 27.5, zb, zt)
    for y in COLS:
        for sx in (-1, 1):
            # level 2: MR63 (6 x 3 x 2.5) seat from each face
            rail = rail.cut(cyl((sx * RAIL_HALF, y, Z_S2), (sx * (RAIL_HALF - 2.5), y, Z_S2), 3.0))
            # level 1: bushing post 6.4 x (3 -> 8.0) with a 4 mm bore for a 3 x 4 brass sleeve
            post = cyl((sx * RAIL_HALF, y, Z_S1), (sx * (G1 - GROOVE_DX - 1.4), y, Z_S1), 3.2)
            post = post.cut(cyl((sx * (RAIL_HALF - 1), y, Z_S1), (sx * (G1), y, Z_S1), 2.0))
            rail = rail.union(post)
    rail = rail.cut(box(-1.0, 1.0, -27.6, 27.6, zb - 1, zb + 6))  # cable slot at the bottom
    for sy in (-1, 1):  # lightening / bus cable channel
        rail = rail.cut(cyl((0, sy * 13.5, zb - 1), (0, sy * 13.5, zt + 1), 1.2))
    return rail


def make_cassette(side):
    """Stepped servo plate: level 1 plate on the outer side of the level-1 tabs, level 2 plate 5 mm
    further in. Windows for the bodies, 2 x M2 tab holes per servo."""
    sx = 1 if side == "R" else -1
    plates = None
    for lvl, zs in ((1, Z_S1), (2, Z_S2)):
        g = G1 if lvl == 1 else G2
        bottom = g + CASE_GAP + S["H"]
        tab_out = bottom - S["tab_z"]  # outer face of the tab (towards the case bottom)
        x0, x1 = sorted((sx * tab_out, sx * (tab_out + 3.0)))
        zc = zs - S["shaft_offset"]
        zlo, zhi = (zc - 17.5, zc + 17.0) if lvl == 1 else (zc - 17.5, zc + 17.0)
        pl = box(x0, x1, -29.0, 29.0, zlo, zhi)
        for y in COLS:
            pl = pl.cut(box(x0 - 1, x1 + 1, y - S["W"] / 2 - 0.3, y + S["W"] / 2 + 0.3, zc - S["L"] / 2 - 0.3, zc + S["L"] / 2 + 0.3))
            for dz in (-S["hole_spacing"] / 2, S["hole_spacing"] / 2):
                pl = pl.cut(cyl((x0 - 1, y, zc + dz), (x1 + 1, y, zc + dz), 0.9))
        plates = pl if plates is None else plates.union(pl)
    # step between the two levels
    xa = sx * (G1 + CASE_GAP + S["H"] - S["tab_z"])
    xb = sx * (G2 + CASE_GAP + S["H"] - S["tab_z"])
    zstep = Z_S2 - S["shaft_offset"] + 17.0
    x0, x1 = sorted((xa + sx * 3.0, xb))
    plates = plates.union(box(x0, x1, -29.0, 29.0, zstep - 3.0, zstep))
    return plates


def make_clamp_block():
    """Barrel-adjuster block across the rail top: 32 x M2.5 adjusters (SHCS drilled 1.0 through, the
    sheath end sits in the 2 mm hex socket). Odd columns on 6 mm risers so neighbouring heads clear."""
    blk = box(-13.5, 13.5, -27.5, 27.5, Z_CLAMP[0], Z_CLAMP[1])
    blk = blk.cut(box(-3.6, 3.6, -26, 26, Z_CLAMP[0] - 1, Z_CLAMP[1] + 1))  # open middle (bundle of cables)
    for sid in SLOTS:
        s = servo_geom(sid)
        for name, p in strand_points(sid).items():
            x, y, _ = p["clamp_in"]
            ztop = p["sheath_start"][2] - 1.5
            if ztop > Z_CLAMP[1]:
                blk = blk.union(cyl((x, y, Z_CLAMP[1] - 0.1), (x, y, ztop), 2.4))
            blk = blk.cut(cyl((x, y, Z_CLAMP[0] - 1), (x, y, ztop + 1), 1.05))  # M2.5 tap drill 2.1
    return blk


def make_adjusters():
    out = None
    for sid in SLOTS:
        for p in strand_points(sid).values():
            x, y, z = p["sheath_start"]
            a = cyl((x, y, z - 1.5), (x, y, z + 1.0), 2.25)  # head 4.5 x 2.5 above the block
            out = a if out is None else out.union(a)
    return out


def comb_cells():
    """Assign each of the 32 strands a bundle cell (same cell at the forearm comb and at the palm
    clamp) to minimise the sheath fan's sideways travel (Hungarian assignment)."""
    from scipy.optimize import linear_sum_assignment

    keys, starts = [], []
    for sid in sorted(SLOTS):
        for name, p in strand_points(sid).items():
            keys.append((sid, name))
            starts.append(p["sheath_start"][:2])
    cells = K.tube_offsets()
    starts = np.array(starts)
    # scale the clamp field into the bundle so the order is kept (no crossing)
    sc = starts / np.array([np.abs(starts[:, 0]).max(), np.abs(starts[:, 1]).max()])
    cc = cells / np.abs(cells).max(axis=0)
    cost = ((sc[:, None, :] - cc[None, :, :]) ** 2).sum(-1)
    r, c = linear_sum_assignment(cost)
    return {keys[i]: int(c[j]) for j, i in enumerate(r)}


def sheath_path(start, cell, n=60):
    a, b = K.tube_offsets()[cell]
    p0 = np.array(start, float)
    p1 = np.array([a, b, Z_COMB[0]])
    dz = p1[2] - p0[2]
    t = np.linspace(0, 1, n)[:, None]
    # S-curve: vertical at both ends (cubic Hermite with vertical tangents)
    h00, h01 = 2 * t**3 - 3 * t**2 + 1, -2 * t**3 + 3 * t**2
    h10, h11 = t**3 - 2 * t**2 + t, t**3 - t**2
    T = np.array([0, 0, dz])
    pts = h00 * p0 + h10 * T + h01 * p1 + h11 * T
    # continue straight up through the comb to the wrist base top (z -15) for the render
    return pts


def make_sheaths(cells):
    out = {}
    for (sid, name), cell in cells.items():
        p = strand_points(sid)[name]
        pts = sheath_path(p["sheath_start"], cell, 24)
        solid = None
        for i in range(len(pts) - 1):
            c = cyl(pts[i], pts[i + 1], P.SHEATH_OD / 2)
            solid = c if solid is None else solid.union(c)
        out[f"sheath_{sid:02d}{name[0]}"] = solid
    return out


def make_strands():
    out = None
    for sid in SLOTS:
        for p in strand_points(sid).values():
            c = cyl(p["tangent"], p["clamp_in"], 0.25)
            out = c if out is None else out.union(c)
    return out


def make_comb():
    hx, hy = K.bundle_half()
    comb = rrect(2 * hx + 8, 2 * hy + 8, 3.0, Z_COMB[1] - Z_COMB[0], Z_COMB[0])
    comb = comb.union(box(-hx - 4, hx + 4, -SHELL[1] + SHELL[2] - 0.2, -hy - 3, Z_COMB[0], Z_COMB[1]))
    comb = comb.union(box(-hx - 4, hx + 4, hy + 3, SHELL[1] - SHELL[2] + 0.2, Z_COMB[0], Z_COMB[1]))
    for a, b in K.tube_offsets():
        comb = comb.cut(cyl((a, b, Z_COMB[0] - 1), (a, b, Z_COMB[1] + 1), 1.15))  # 2.3 slide guide
    comb = comb.cut(cyl((0, -hy - 1.8, Z_COMB[0] - 1), (0, -hy - 1.8, Z_COMB[1] + 1), K.CABLE_D / 2 + 0.2))
    return comb


def _section(hx, hy, r, z):
    return cq.Workplane("XY").workplane(offset=z).rect(2 * hx, 2 * hy).val()


def make_shell():
    """Outer shell, split at y = 0: back half (structural, wrist-drive bearings) and a removable front
    cover; side windows for servo removal closed by side covers."""
    ox, oy, w = SHELL
    tx, ty = SHELL_TOP

    def loft(dx):
        s1 = cq.Workplane("XY").workplane(offset=Z_SHELL_TOP).rect(2 * (tx - dx), 2 * (ty - dx))
        s1 = s1.workplane(offset=Z_SHELL_WIDE - Z_SHELL_TOP).rect(2 * (ox - dx), 2 * (oy - dx)).loft()
        st = cq.Workplane("XY").workplane(offset=Z_FLANGE[1]).rect(2 * (ox - dx), 2 * (oy - dx)).extrude(Z_SHELL_WIDE - Z_FLANGE[1])
        return s1.union(st)

    outer = loft(0.0)
    inner = loft(w)
    shell = outer.cut(inner).cut(box(-ox - 1, ox + 1, -oy - 1, oy + 1, Z_SHELL_TOP - 0.01, Z_SHELL_TOP + 5))
    shell = shell.cut(box(-ox - 1, ox + 1, -oy - 1, oy + 1, Z_SHELL_TOP - 0.5, Z_SHELL_TOP + 1))
    # side windows (servo removal) over both levels
    win = (Z_S2 - S["shaft_offset"] - 16.0, Z_S1 - S["shaft_offset"] + 15.0)
    for sx in (-1, 1):
        shell = shell.cut(box(sx * ox - 3, sx * ox + 3, -oy + 5, oy - 5, win[0], win[1]))
    back = shell.cut(box(-ox - 1, ox + 1, -oy - 1, 0.0, Z_FLANGE[1] - 1, 1))
    front = shell.cut(box(-ox - 1, ox + 1, 0.0, oy + 1, Z_FLANGE[1] - 1, 1))
    sides = {}
    for sx, n in ((1, "R"), (-1, "L")):
        x0, x1 = sorted((sx * ox, sx * (ox + 1.5)))
        sides[n] = box(x0, x1, -oy + 3, oy - 3, win[0] - 2, win[1] + 2)
    return back, front, sides


def make_frame(wrist_iface):
    """Internal frame: top ring under the wrist base, deck over the servo bay, the electronics tray
    and the wrist-drive supports (crank-axle bosses on the back wall, servo cradles)."""
    ox, oy, w = SHELL
    tx, ty = SHELL_TOP
    top = rrect(2 * 16.0, 2 * (ty - w), 3, 4.0, Z_SHELL_TOP - 4.0)  # narrow: the push rods pass beside it
    for sx in (-1, 1):
        x0, x1 = sorted((sx * 12.0, sx * 20.0))
        top = top.cut(box(x0, x1, -12.5, 12.5, Z_SHELL_TOP - 6, Z_SHELL_TOP + 1))
    top = top.cut(rrect(2 * WR.BASE_OPEN[0] + 4, 2 * WR.BASE_OPEN[1] + 4, 4, 10, Z_SHELL_TOP - 6))
    for bx, by in wrist_iface["forearm_top"]["bolts"]:
        top = top.cut(cyl((bx, by, Z_SHELL_TOP - 5), (bx, by, Z_SHELL_TOP + 1), 2.0))  # M3 insert
    deck = box(-(ox - w), ox - w, -(oy - w), oy - w, Z_DECK[0], Z_DECK[1])
    deck = deck.cut(box(-15.0, 15.0, -29.0, 29.0, Z_DECK[0] - 1, Z_DECK[1] + 1))  # sheath fan passes
    tray = box(-(ox - w), ox - w, -(oy - w), oy - w, Z_TRAY[0], Z_TRAY[1])
    tray = tray.cut(box(-6, 6, -6, 6, Z_TRAY[0] - 1, Z_TRAY[1] + 1))
    frame = top.union(deck).union(tray)
    # wrist servo cradles (plates under each STS3215) + crank axle bosses on the back wall
    d = wrist_iface["drive"]
    for side in ("R", "L"):
        sh = np.array(d["servo_shaft"][side])
        sx = 1 if side == "R" else -1
        zc = sh[2] - WR.STS_SHAFT_OFF
        L = WR.STS["L"]
        cr = box(min(sx * 14.0, sx * (ox - w)), max(sx * 14.0, sx * (ox - w)), sh[1] - WR.STS["H"] - 3, sh[1] - WR.STS["H"],
                 zc - L / 2, zc + L / 2)
        frame = frame.union(cr)
        A = np.array(d["crank_axle"][side])
        boss = cyl((A[0], oy - w + 0.2, A[2]), (A[0], A[1] + 3.0 if A[1] + 3.0 < oy - w else oy - w - 4.0, A[2]), 5.0)
        frame = frame.union(boss.cut(cyl((A[0], oy, A[2]), (A[0], A[1] - 10, A[2]), 2.5)))
    return frame


def make_electronics():
    z0 = Z_TRAY[1]
    esp = box(-34.5, 34.5, 2.0, 28.0, z0, z0 + 12.0)  # ESP32-S3-DevKitC-1 N16R8 (~69 x 26, with headers)
    urt = box(-40.0, 8.0, -27.0, -4.0, z0, z0 + 10.0)  # FE-URT-1 (~48 x 23, verify)
    pdb = box(11.0, 41.0, -27.0, -4.0, z0, z0 + 14.0)  # power distribution: 3 servo branches + caps
    return {"esp32_s3": esp, "fe_urt1": urt, "power_board": pdb}


def make_arm_flange():
    fl = cq.Workplane("XY").workplane(offset=Z_FLANGE[0]).circle(36.0).extrude(Z_FLANGE[1] - Z_FLANGE[0])
    fl = fl.union(box(-SHELL[0], SHELL[0], -SHELL[1], SHELL[1], Z_FLANGE[1] - 0.01, Z_FLANGE[1] + 2.0).cut(
        box(-SHELL[0] + SHELL[2], SHELL[0] - SHELL[2], -SHELL[1] + SHELL[2], SHELL[1] - SHELL[2], Z_FLANGE[1] - 1, Z_FLANGE[1] + 3)))
    fl = fl.cut(cyl((0, 0, Z_FLANGE[0] - 1), (0, 0, Z_FLANGE[1] + 3), ARM_BORE / 2))
    for i in range(ARM_BOLTS):
        a = 2 * math.pi * i / ARM_BOLTS + math.pi / 4
        x, y = ARM_PCD / 2 * math.cos(a), ARM_PCD / 2 * math.sin(a)
        fl = fl.cut(cyl((x, y, Z_FLANGE[0] - 1), (x, y, Z_FLANGE[1] + 3), ARM_BOLT_D / 2))
    return fl


# --- build -------------------------------------------------------------------------------------------
_C: dict = {}


def build(with_sheaths=True):
    t0 = time.time()
    wi = WR.interface()
    if not _C:
        _C["rail"] = make_rail()
        _C["cassette_R"], _C["cassette_L"] = make_cassette("R"), make_cassette("L")
        _C["clamp"] = make_clamp_block()
        _C["adj"] = make_adjusters()
        _C["comb"] = make_comb()
        _C["back"], _C["front"], _C["sides"] = make_shell()
        _C["frame"] = make_frame(wi)
        _C["flange"] = make_arm_flange()
        _C["servos"] = {sid: make_servo(sid) for sid in SLOTS}
        _C["spools"] = {sid: make_spool(sid) for sid in SLOTS}
        _C["strands"] = make_strands()
        _C["cells"] = comb_cells()
        _C["sheaths"] = make_sheaths(_C["cells"]) if with_sheaths else {}
        _C["elec"] = make_electronics()
    parts = {
        "spool_rail": _C["rail"], "cassette_R": _C["cassette_R"], "cassette_L": _C["cassette_L"],
        "adjuster_block": _C["clamp"], "barrel_adjusters": _C["adj"], "bundle_comb": _C["comb"],
        "forearm_shell_back": _C["back"], "forearm_cover_front": _C["front"],
        "side_cover_R": _C["sides"]["R"], "side_cover_L": _C["sides"]["L"], "forearm_frame": _C["frame"],
        "arm_flange": _C["flange"], "strands_bare": _C["strands"],
    }
    for sid in SLOTS:
        parts[f"servo_{sid:02d}"] = _C["servos"][sid]
        parts[f"spool_{sid:02d}"] = _C["spools"][sid]
    parts.update(_C["sheaths"])
    parts.update(_C["elec"])
    return {"parts": parts, "joints": {}, "interface": interface(), "build_s": time.time() - t0}


def interface():
    cells = _C.get("cells") or comb_cells()
    servos = {}
    for sid in sorted(SLOTS):
        s = servo_geom(sid)
        sp = strand_points(sid)
        servos[sid] = {
            "joint": P.SERVO_IDS[sid], "side": s["side"], "level": s["lvl"], "column_y": s["yc"],
            "hls_room": sid in HLS,
            "spool_centre": (s["sx"] * s["g"], s["yc"], s["zs"]), "spool_axis": (s["sx"] * -1.0, 0, 0),
            "flex": {"tangent": sp["flex"]["tangent"], "clamp": sp["flex"]["sheath_start"], "bundle_cell": cells[(sid, "flex")]},
            "ext": {"tangent": sp["ext"]["tangent"], "clamp": sp["ext"]["sheath_start"], "bundle_cell": cells[(sid, "ext")]},
        }
    return {
        "servos": servos,
        "bundle_cells_xy": K.tube_offsets().round(3).tolist(),
        "comb_z": Z_COMB, "clamp_block_z": Z_CLAMP,
        "arm_mount": {"z": Z_FLANGE[0], "pcd": ARM_PCD, "bolts": ARM_BOLTS, "bolt": "M4", "bore": ARM_BORE,
                      "note": "stand-in for OpenArm J5's flange (sim site forearm_flange); verify the real pattern"},
        "envelope": {"x": (-SHELL[0], SHELL[0]), "y": (-SHELL[1], SHELL[1]), "z": (Z_FLANGE[0], Z_SHELL_TOP)},
    }


# --- checks ------------------------------------------------------------------------------------------
def sheath_report(cells):
    from tools.transform import min_bend_radius

    worst, rows = 1e9, []
    for (sid, name), cell in cells.items():
        p = strand_points(sid)[name]
        pts = sheath_path(p["sheath_start"], cell, 200)
        r, _ = min_bend_radius(pts)
        rows.append((sid, name, cell, r))
        worst = min(worst, r)
    return worst, rows


def spacing_report():
    """Smallest centre distance between adjuster heads at the same height, and between bare strands
    and foreign spools/posts (2D, in each strand's plane)."""
    pts = [(p["sheath_start"], sid) for sid in SLOTS for p in strand_points(sid).values()]
    dmin = 1e9
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            a, b = np.array(pts[i][0]), np.array(pts[j][0])
            if abs(a[2] - b[2]) < 2.6:
                dmin = min(dmin, float(np.linalg.norm(a[:2] - b[:2])))
    return dmin


def check(quick=False):
    from tools.checks import interference

    b = build()
    parts = b["parts"]
    # wrist drive at q = 0 (fixed parts) + wrist base, for the shared space in the neck
    w = WR.build({}, bundle=True)["parts"]
    for k in ("wrist_base", "wrist_servo_R", "wrist_servo_L", "wrist_gears_R", "wrist_gears_L",
              "wrist_crank_R", "wrist_crank_L", "wrist_rod_R", "wrist_rod_L", "sheath_bundle"):
        parts["W_" + k] = w[k]
    ignore = [("servo_*", "cassette_*"), ("spool_*", "servo_*"), ("spool_*", "spool_rail"),
              ("adjuster_block", "barrel_adjusters"), ("strands_bare", "adjuster_block"),
              ("strands_bare", "spool_*"), ("strands_bare", "barrel_adjusters"), ("sheath_*", "barrel_adjusters"),
              ("sheath_*", "bundle_comb"), ("sheath_*", "sheath_*"), ("sheath_*", "W_sheath_bundle"),
              ("W_sheath_bundle", "bundle_comb"), ("forearm_frame", "forearm_shell_back"), ("forearm_frame", "forearm_cover_front"),
              ("bundle_comb", "forearm_shell_back"), ("bundle_comb", "forearm_cover_front"), ("arm_flange", "forearm_*"),
              ("forearm_frame", "W_wrist_servo_*"), ("forearm_frame", "W_wrist_base"), ("W_wrist_gears_*", "W_wrist_servo_*"),
              ("W_wrist_gears_*", "W_wrist_crank_*"), ("W_wrist_crank_*", "W_wrist_rod_*"), ("forearm_frame", "W_wrist_crank_*"),
              ("forearm_frame", "W_wrist_gears_*"), ("cassette_*", "forearm_frame"), ("spool_rail", "adjuster_block"),
              ("esp32_s3", "forearm_frame"), ("fe_urt1", "forearm_frame"), ("power_board", "forearm_frame"),
              ("forearm_shell_back", "forearm_cover_front"), ("side_cover_*", "forearm_*"), ("cassette_*", "forearm_shell_back"),
              ("cassette_*", "forearm_cover_front"), ("spool_rail", "forearm_frame")]
    t0 = time.time()
    res = interference(parts, threshold=0.5, ignore=ignore, time_budget=900)
    print(f"  static interference ({time.time() - t0:.0f} s): " + ("clear" if not res else ""))
    for a, b_, v in res:
        print(f"    {a} x {b_} = {v}")
    # wrist drive moving parts vs the forearm across the wrist range
    moving = []
    fl, dv = K.RANGES["wrist_flex"], K.RANGES["wrist_dev"]
    for f in (fl[0] - 2, 0, fl[1] + 2):
        for d in (dv[0] - 2, 0, dv[1] + 2):
            wq = WR.build({"wrist_flex": f, "wrist_dev": d}, bundle=True)["parts"]
            sub = {k: parts[k] for k in ("forearm_shell_back", "forearm_cover_front", "forearm_frame", "bundle_comb",
                                         "adjuster_block") if k in parts}
            sub.update({k: v for k, v in parts.items() if k.startswith("sheath_")})
            for k in ("wrist_rod_R", "wrist_rod_L", "wrist_crank_R", "wrist_crank_L", "palm_flange", "wrist_ring", "sheath_bundle"):
                if k in wq:
                    sub["M_" + k] = wq[k]
            r = interference(sub, threshold=0.5, ignore=[("forearm_shell_back", "forearm_cover_front"), ("forearm_frame", "forearm_*"),
                                                          ("bundle_comb", "forearm_*"), ("forearm_frame", "M_wrist_crank_*"),
                                                          ("M_wrist_crank_*", "M_wrist_rod_*"), ("M_sheath_bundle", "bundle_comb"),
                                                          ("M_sheath_bundle", "sheath_*"), ("sheath_*", "sheath_*"),
                                                          ("sheath_*", "bundle_comb"), ("M_*", "M_*"), ("adjuster_block", "sheath_*")])
            r = [x for x in r if x[0].startswith("M_") or x[1].startswith("M_")]
            moving.append(((f, d), r))
            print(f"  wrist flex {f:4.0f} dev {d:4.0f}: " + ("clear" if not r else "; ".join(f"{a} x {b_}={v}" for a, b_, v in r)))
    return b, res, moving


def main():
    quick = "--quick" in sys.argv
    OUT.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    b = build()
    print(f"build: {len(b['parts'])} parts in {b['build_s']:.1f} s")
    cells = _C["cells"]
    worst, rows = sheath_report(cells)
    print(f"forearm sheath fan: min bend radius {worst:.1f} mm (limit {P.SHEATH_MIN_BEND_R})")
    print(f"adjuster heads: min spacing at the same height {spacing_report():.2f} mm (head 4.5)")
    from tools.checks import is_valid

    bad = [n for n, p in b["parts"].items() if not (lambda v: v if isinstance(v, bool) else v.get("valid", True))(is_valid(p))]
    print("invalid solids:", bad or "none")
    _, res, moving = check(quick)
    summary = dict(sheath_min_bend=worst, sheaths=[dict(id=r[0], strand=r[1], cell=r[2], min_r=round(r[3], 1)) for r in rows],
                   adjuster_min_spacing=spacing_report(), static_interference=res,
                   moving=[dict(q=q, hits=r) for q, r in moving], interface=b["interface"], invalid=bad)
    (OUT / "forearm_summary.json").write_text(json.dumps(summary, indent=1, default=str))
    if not quick:
        from tools.export import export_all

        export_all(b["parts"], OUT, "forearm")
    print(f"done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
