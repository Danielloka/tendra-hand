"""Checks for the thumb's routing through its 2-axis base (research/experiments/2026-09-29-thumb-routing):
drum directions, strands near the rot axis, the sheath loop over the whole cmc_flex range, and room
in the journal and on the metacarpal boss."""

import itertools
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tendon_router as tr

ROUTES = {r["name"]: r for r in tr.routes()}
QS = np.radians(np.linspace(*tr.CMC_FLEX_RANGE_DEG, 9))
SIDES = ("flex", "ext")


def test_every_thumb_strand_has_a_kind():
    kinds = {n: r["kind"] for n, r in ROUTES.items() if n.startswith("thumb")}
    assert len(kinds) == 8
    assert {n for n, k in kinds.items() if k == "rot"} == {
        "thumb_cmc_rot_flex",
        "thumb_cmc_rot_ext",
    }
    assert {n for n, k in kinds.items() if k == "bare"} == {
        "thumb_cmc_flex_flex",
        "thumb_cmc_flex_ext",
    }


def test_cmc_rot_pull_turns_the_base_the_right_way():
    """Positive cmc_rot turns the base about -Z. A pull toward +Y (along the strand) at the tangent
    point must give torque -Z for flex and +Z for ext."""
    c = np.array((*tr.ROT_AXIS_XY, tr.ROT_DRUM_Z))
    for side, sign in (("flex", -1), ("ext", +1)):
        t = tr.rot_tangent(side)
        torque = np.cross(t - c, (0, 1, 0))
        assert np.sign(torque[2]) == sign
        assert abs(np.linalg.norm((t - c)[:2]) - tr.ROT_DRUM_R) < 1e-9


def test_cmc_rot_strands_run_straight_back_to_the_bay_wall_in_the_groove_plane():
    for side in SIDES:
        r = ROUTES[f"thumb_cmc_rot_{side}"]
        t, e = np.array(r["thumb_side_mm"][0]), np.array(r["entry_mm"])
        assert t[0] == e[0] and t[2] == e[2] == tr.ROT_DRUM_Z and e[1] > t[1]
        # the flange of the drum is the only part of the base in the groove's layer
        assert tr.ROT_FLANGE_R < tr.THUMB_BAY[1][1] - tr.ROT_AXIS_XY[1]


def test_cmc_flex_pull_closes_and_opens():
    """cmc_flex closes about -X. Flex pulls down at the back of the drum, ext pulls up there."""
    r = np.array((0.0, tr.CMC_DRUM_R, 0.0))  # back tangent, relative to the axis
    assert np.cross(r, (0, 0, -1))[0] < 0  # flex: -X = closing
    assert np.cross(r, (0, 0, 1))[0] > 0  # ext: +X = opening


def test_cmc_flex_strands_cross_the_rot_axis_closely():
    """The bare cmc_flex strands are held by the base at the plate top and by the palm where their
    own 1.2 mm bore starts, right under the journal. Turning the base swings the top point around the
    axis; over the whole cmc_rot range the length may change by at most 0.25 mm (2.4 deg of
    cmc_flex on its 6 mm drum)."""
    ax = np.array(tr.ROT_AXIS_XY)
    for side in SIDES:
        r = ROUTES[f"thumb_cmc_flex_{side}"]
        top = np.array((*tr.CMC_J[side], tr.PLATE_Z[1]))
        held = np.array(r["entry_mm"])
        lengths = []
        for q in np.radians(np.linspace(-100, 40, 29)):
            c, s_ = math.cos(q), math.sin(q)
            v = top[:2] - ax
            p = np.array((ax[0] + c * v[0] - s_ * v[1], ax[1] + s_ * v[0] + c * v[1], top[2]))
            lengths.append(np.linalg.norm(p - held))
        assert max(lengths) - min(lengths) < 0.25, (side, max(lengths) - min(lengths))


def test_cmc_flex_ext_strand_clears_the_drum_flange():
    """The ext strand comes down from the back of its pin to the plate; it must pass the drum's
    flange (in its own groove's plane) with room to spare."""
    ax = np.array(tr.CMC_FLEX_YZ)
    py, pz = tr.CMC_PIN_YZ
    rr = tr.PIN_R + tr.LINE_R
    j = np.array((tr.CMC_J["ext"][1], tr.PLATE_Z[1]))
    # tangent point on the pin's back side, seen from the plate point below
    c = np.array((py, pz))
    d = np.linalg.norm(c - j)
    base = math.atan2(*(c - j)[::-1])
    tp = c + rr * np.array((math.cos(base - math.pi / 2 + math.asin(rr / d)),
                            math.sin(base - math.pi / 2 + math.asin(rr / d))))  # fmt: skip
    tp = tp if tp[0] > py else c + rr * np.array((1.0, 0.0))
    seg = [j + (tp - j) * s for s in np.linspace(0, 1, 200)]
    gap = min(np.linalg.norm(p - ax) for p in seg) - tr.CMC_FLANGE_R - tr.LINE_R
    assert gap >= 0.3, gap
    # the rising part of the ext strand is on the pin's front side, tangent to both drum and pin
    assert abs((py - rr) - (ax[0] + tr.CMC_DRUM_R)) < 1e-9


def test_journal_holds_all_eight_strands():
    radius = {"sheath": tr.TUBE_OD / 2, "bare": tr.LINE_R}
    ax = np.array(tr.ROT_AXIS_XY)
    items = []
    for j in tr.JOURNAL_JOINTS:
        for side in SIDES:
            kind = ROUTES[f"thumb_{j}_{side}"]["kind"]
            items.append((f"{j}_{side}", np.array(tr.journal_xy(j, side)), radius[kind]))
    for name, p, r in items:
        assert np.linalg.norm(p - ax) + r <= tr.BORE_R - 0.1, name
    for (na, pa, ra), (nb, pb, rb) in itertools.combinations(items, 2):
        assert np.linalg.norm(pa - pb) >= ra + rb + 0.05, (na, nb)  # sheaths may touch
    # and the journal wall under the cmc_rot groove is thick enough
    assert tr.ROT_DRUM_R - tr.LINE_R - tr.BORE_R >= 1.2


def test_sheath_loop_bends_gently_over_the_whole_cmc_flex_range():
    for j in tr.SHEATH_JOINTS:
        for side in SIDES:
            for q in QS:
                pts, r = tr.sheath_loop(j, side, q)
                assert pts is not None and r >= tr.TUBE_MIN_BEND_R, (j, side, math.degrees(q), r)


def test_sockets_fit_on_the_metacarpal_beside_the_cmc_flex_gap():
    """Sockets keep clear of the slab where the cmc_flex strands, their pin and its hanger live
    (the boss is split there, so it can swing past them), and leave 0.8 mm of wall inside the
    metacarpal's width (x -6.8..8.8, measured). Sockets in one row are a tube apart; the upper one
    is offset up-back."""
    slab_lo = tr.CMC_GROOVE_X["flex"] - 0.5
    slab_hi = tr.CMC_GROOVE_X["ext"] + tr.GROOVE_W / 2 + 0.3 + 0.3  # pin end + clearance
    for side in SIDES:
        pos = {j: (tr.socket_x(j, side), 0.0) for j in tr.SHEATH_JOINTS}
        for (ja, a), (jb, b) in itertools.combinations(pos.items(), 2):
            assert math.dist(a, b) >= tr.TUBE_OD + 0.1 - 1e-9, (side, ja, jb)
        for x, _ in pos.values():
            assert x + tr.TUBE_OD / 2 <= slab_lo or x - tr.TUBE_OD / 2 >= slab_hi, (side, x)
            assert -6.8 + 0.8 <= x - tr.TUBE_OD / 2 and x + tr.TUBE_OD / 2 <= 8.8 - 0.8, (side, x)


def test_sheaths_do_not_pass_through_each_other():
    """The modelled loop shapes of any two sheaths never cross. The shapes are only estimates (real
    tubes shift a little to make room), so centre lines closer than one tube diameter are allowed
    down to 1.5 mm."""
    for q in QS:
        loops = {(j, s): tr.sheath_loop_3d(j, s, q)[0] for j in tr.SHEATH_JOINTS for s in SIDES}
        for (ka, a), (kb, b) in itertools.combinations(loops.items(), 2):
            d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2).min()
            assert d >= 1.5, (ka, kb, math.degrees(q), round(d, 2))


def test_journal_channels_share_a_cavity_only_near_the_top():
    assert tr.fan_bottom_z() >= tr.FAN_Z


def test_firmware_uses_the_cmc_rot_drum_ratio():
    cfg = (Path(__file__).resolve().parents[3] / "firmware" / "include" / "config_v1.h").read_text()
    row = re.search(r'\{"thumb_cmc_rot",[^}]*\}', cfg).group(0)
    ratio = float(row.split(",")[-2].strip().rstrip("f"))  # second-last field = servo_per_joint
    assert ratio == tr.ROT_DRUM_R / tr.SPOOL_R


# ----- Inside the thumb (sockets -> pins -> crossing holes -> drums) ------------------------------

INNER = {f"{j}_{s}": tr.thumb_inner(j, s) for j in tr.SHEATH_JOINTS for s in SIDES}
CL = np.array((tr.THUMB_CL_XZ[0], 0.0, tr.THUMB_CL_XZ[1]))


def _dist_to_line(p, point, direction):
    v = p - point
    return float(np.linalg.norm(v - direction * (v @ direction)))


def test_bores_keep_a_wall_between_each_other():
    """Different strands' 1.2 mm bores inside the thumb keep >= 0.8 mm of wall (centre lines >= 2 mm
    apart), so the strands never touch each other."""
    closest, a, b = tr.thumb_bore_clearance()[0]
    assert closest >= tr.BORE_SMALL + tr.MIN_WALL, (a, b, closest)


def test_ip_lines_cross_the_mcp_axis_exactly():
    """The ip strands pass the MCP joint at a point on its axis, so turning it doesn't change
    their length."""
    mcp = np.array((tr.THUMB_CL_XZ[0], tr.MCP_Y, tr.THUMB_CL_XZ[1]))
    for side in SIDES:
        assert _dist_to_line(INNER[f"ip_{side}"]["cross"], mcp, tr.A_AX) < 1e-9


def test_crossing_holes_and_grooves_leave_walls_on_the_mcp_barrel():
    """On the proximal's MCP barrel (solid at a -4.5..4.5, bearing bosses beyond): the two ip holes
    and the mcp_flex groove keep >= 0.8 mm between each other and to the bosses."""
    r = tr.CROSS_HOLE_D / 2
    feats = [(a, r) for a in tr.IP_GROOVE_A.values()] + [(tr.MCP_GROOVE_A, tr.GROOVE_HALF_W)]
    for (a0, h0), (a1, h1) in itertools.combinations(feats, 2):
        assert abs(a0 - a1) - h0 - h1 >= 0.8 - 1e-9, (a0, a1)
    for a0, h0 in feats:
        assert abs(a0) + h0 <= 4.5 - 0.8, a0


def test_mcp_flex_lines_reach_the_drum_in_its_plane():
    """After pin B the mcp_flex lines run parallel to the thumb at the drum's groove position, tangent
    to the drum on the closing (flex, +w) and opening (ext, -w) side: no rubbing on the flanges."""
    for side, sgn in (("flex", 1), ("ext", -1)):
        t = INNER[f"mcp_flex_{side}"]
        seg = t["bores3"][-1]
        d = (seg[1] - seg[0]) / np.linalg.norm(seg[1] - seg[0])
        assert abs(d @ tr.A_AX) < 1e-9 and abs(d[1] + 1) < 1e-9  # along -Y, in the plane
        expected = tr.thumb_pt(tr.MCP_GROOVE_A, sgn * tr.THUMB_DRUM_R, tr.MCP_Y)
        assert np.allclose(seg[1], expected)


def test_ip_lines_stay_in_their_groove_plane_in_the_proximal():
    for side in SIDES:
        t = INNER[f"ip_{side}"]
        seg = t["bores3"][-1]
        d = seg[1] - seg[0]
        assert abs(d @ tr.A_AX) < 1e-9  # a = const: straight into its IP groove


def test_pins_wrap_gently_and_lines_barely_skew():
    """Steel pins only: pin A turns each line out of its tube, pin B levels the mcp_flex lines. Keep
    the total steel wrap <= 130 deg (capstan friction ~ e^(0.15*2.3) = 1.4) and the sideways skew
    of a line over a pin <= 8 deg."""
    for name, chk in tr.thumb_inner_check().items():
        assert chk["pin_a_wrap_deg"] + chk.get("pin_b_wrap_deg", 0) <= 130, (name, chk)
        assert max(chk.get("skew_on_pin_a_deg", 0), chk.get("skew_a_to_b_deg", 0)) <= 8, (name, chk)


def test_pins_clear_the_other_bores():
    """A pin (2 mm) only touches its own strands: every other bore passes >= 0.8 mm of wall away."""
    bores = tr.thumb_bores()
    rho = tr.PIN2_R + tr.BORE_SMALL / 2 + tr.MIN_WALL
    for name, t in INNER.items():
        side = name.rsplit("_", 1)[1]
        pins = [("A", t["pin_a"], tr.PIN_A_X[side])]
        if "pin_b" in t:
            x0 = t["line_x"] - 1.2 if side == "ext" else tr.META_FILL["x"][0]
            x1 = tr.META_FILL["x"][1] if side == "ext" else t["line_x"] + 1.2
            pins.append(("B", t["pin_b"], (x0, x1)))
        for label, (py, pz), (x0, x1) in pins:
            for other, segs in bores.items():
                if other == f"thumb_{name}":
                    continue
                for s0, s1 in segs:
                    for p in np.linspace(s0, s1, 60):
                        if x0 - rho <= p[0] <= x1 + rho:
                            if label == "A" and other.rsplit("_", 1)[1] == side:
                                continue  # same pin A carries both strands of a side
                            assert math.hypot(p[1] - py, p[2] - pz) >= rho - 1e-6, (
                                name,
                                label,
                                other,
                                p,
                            )


def test_mcp_flex_lines_stay_inside_the_metacarpal_block():
    """In front of the socket block (y <= 0) the mcp_flex lines run inside the added solid block
    with 0.8 mm of wall; behind it they are in the socket block (checked in the CAD)."""
    for side in SIDES:
        t = INNER[f"mcp_flex_{side}"]
        x0, x1 = tr.META_FILL["x"]
        z0, z1 = tr.META_FILL["z"]
        r = tr.BORE_SMALL / 2 + tr.MIN_WALL
        for s0, s1 in t["bores3"][1:3]:
            for p in np.linspace(s0, s1, 40):
                if p[1] <= 0.0:
                    top = z1 if p[1] >= tr.META_FILL["front"][0] else tr.META_FILL["front"][1]
                    assert x0 + r <= p[0] <= x1 - r and z0 + r <= p[2] <= top - r, (side, p)
