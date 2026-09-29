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
    assert len(kinds) == 10
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
        pos = {j: (tr.socket_x(j, side), tr.SOCKETS[j][1]) for j in tr.SHEATH_JOINTS}
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
    ratio = float(row.split(",")[-2].strip().rstrip("f"))
    assert ratio == tr.ROT_DRUM_R / tr.SPOOL_R
