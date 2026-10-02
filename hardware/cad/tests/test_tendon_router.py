"""Checks that the v1 tendon routes are buildable: one separate channel per strand, enough wall
between channels, gentle bends, and a clear straight line from the wrist to every spool."""

import itertools
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tendon_router as tr

ROUTES = tr.routes()
SERVOS = tr.servos()


def dense(points, step=0.25):
    """Resample a polyline every `step` mm."""
    pts = [np.asarray(points[0], float)]
    for a, b in itertools.pairwise(points):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.linalg.norm(b - a) / step))
        pts += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(pts)


def palm_part(route):
    """Channel from the entry to the bottom of the wrist plate, with its bore at each sample."""
    pts = route["points_mm"][:-1]  # last point = spool tangent (straight run in the forearm)
    d = dense(pts)
    small_end_z = route["small_bore_end_mm"][2]
    bore = np.where(d[:, 2] >= small_end_z - 1e-9, tr.BORE_SMALL, tr.BORE_TUBE)
    return d, bore


PALM = {r["name"]: palm_part(r) for r in ROUTES}
JOURNAL = {r["name"] for r in ROUTES if r.get("kind") in ("sheath", "bare")}
SHEATHS = {r["name"] for r in ROUTES if r.get("kind") == "sheath"}


def in_box(p, box, margin=0.0):
    return all(lo - margin <= p[i] <= hi + margin for i, (lo, hi) in enumerate(box))


FIRMWARE_CFG = (
    Path(__file__).resolve().parents[3] / "firmware" / "include" / "config_v1.h"
).read_text()


def test_32_strands_16_servos_ids_match_firmware():
    assert len(ROUTES) == 32 and len({r["name"] for r in ROUTES}) == 32
    assert sorted(s.id for s in SERVOS) == list(range(1, 17))
    rows = {n: int(i) for n, i in re.findall(r'\{"(\w+)",\s*(\d+),', FIRMWARE_CFG)}
    assert rows == {s.joint: s.id for s in SERVOS}


def test_firmware_servo_per_joint_is_drum_over_spool():
    rows = dict(re.findall(r'\{"(\w+)",\s*\d+,[^}]*?,\s*([\d.]+)f,\s*-?\d+\}', FIRMWARE_CFG))
    assert rows.keys() == {s.joint for s in SERVOS}
    for joint, spj in rows.items():
        assert float(spj) == pytest.approx(tr.servo_per_joint(joint), abs=1e-6), joint


def test_dips_are_coupled_not_routed():
    assert not any("_dip" in r["joint"] for r in ROUTES)
    assert tr.COUPLED == {f"{f}_dip": f"{f}_pip" for f in tr.FINGERS}
    c = tr.layout()["coupling"]
    assert c["type"] == "linkage" and c["ratio"] == 0.75 and c["joints"] == tr.COUPLED
    assert set(c["linkages"]) == set(tr.FINGERS)


def test_every_strand_has_its_own_entry_wrist_hole_and_tangent():
    for key in ("entry_mm", "wrist_bottom_mm", "tangent_mm"):
        pts = [tuple(np.round(r[key], 3)) for r in ROUTES]
        assert len(set(pts)) == len(ROUTES), key


def test_channels_keep_a_wall_between_each_other():
    worst = math.inf
    names = list(PALM)
    for a, b in itertools.combinations(names, 2):
        pa, ba = PALM[a]
        pb, bb = PALM[b]
        # quick reject by bounding boxes
        if np.any(pa.min(0) - 5 > pb.max(0)) or np.any(pb.min(0) - 5 > pa.max(0)):
            continue
        if a in SHEATHS and b in SHEATHS:  # they share one cavity down to FAN_Z
            ka, kb = pa[:, 2] < tr.FAN_Z, pb[:, 2] < tr.FAN_Z
            pa, ba, pb, bb = pa[ka], ba[ka], pb[kb], bb[kb]
        d = np.linalg.norm(pa[:, None, :] - pb[None, :, :], axis=2)
        need = (ba[:, None] + bb[None, :]) / 2 + tr.MIN_WALL
        margin = (d - need).min()
        worst = min(worst, margin)
        assert margin >= -1e-6, f"{a} and {b} are {margin:.2f} mm too close"
    print(f"closest channel pair has {worst + tr.MIN_WALL:.2f} mm of wall")


def test_channels_stay_inside_the_palm_and_away_from_the_thumb_bay():
    (x0, x1), (y0, y1) = tr.PALM_X, tr.PALM_Y
    cx, cy = tr.ROT_AXIS_XY
    bearing_r = tr.JOURNAL_R + tr.BEARING_CLEAR
    for name, (pts, bore) in PALM.items():
        for p, b in zip(pts, bore):
            r = b / 2 + tr.MIN_WALL
            if p[2] > tr.Z_WRIST:  # inside the palm
                assert x0 + r <= p[0] <= x1 - r and y0 + r <= p[1] <= y1 - r, (
                    f"{name} leaves the palm at {p}"
                )
                if not name.startswith("thumb"):
                    assert not in_box(p, tr.THUMB_BAY, r), f"{name} enters the thumb bay at {p}"
            if name not in JOURNAL and tr.JOURNAL_BOTTOM_Z - tr.BEARING_CLEAR - r <= p[2]:
                assert math.hypot(p[0] - cx, p[1] - cy) >= bearing_r + r, (
                    f"{name} cuts the journal bearing at {p}"
                )
            if not name.startswith("thumb_cmc_rot"):  # the cmc_rot strands wrap the pin
                py, pz = tr.ROT_PIN_YZ
                if tr.ROT_PIN_X[0] - r <= p[0] <= tr.ROT_PIN_X[1]:
                    assert math.hypot(p[1] - py, p[2] - pz) >= tr.PIN_R + r, (
                        f"{name} hits the pin at {p}"
                    )
            for ix, iy in tr.INSERTS:
                if tr.Z_WRIST_PLATE_BOTTOM <= p[2] <= tr.Z_WRIST + tr.INSERT_DEPTH:
                    assert math.hypot(p[0] - ix, p[1] - iy) >= tr.INSERT_R + r, (
                        f"{name} hits an insert"
                    )


def test_thumb_channels_start_below_the_journal_or_at_the_bay_wall():
    for r in ROUTES:
        if r.get("kind") in ("sheath", "bare"):
            assert r["entry_mm"][2] <= tr.JOURNAL_BOTTOM_Z - tr.BEARING_CLEAR + 1e-9
        elif r.get("kind") == "rot":
            assert r["entry_mm"][1] == tr.THUMB_BAY[1][1] and r["entry_mm"][2] == tr.ROT_DRUM_Z


def test_bends_are_gentle_enough_for_ptfe_tube():
    for r in ROUTES:
        assert r["min_bend_radius_mm"] >= tr.MIN_BEND_R, (r["name"], r["min_bend_radius_mm"])
    # also measure it numerically on the sampled curve (3-point circle radius)
    for name, (pts, _b) in PALM.items():
        route = next(r for r in ROUTES if r["name"] == name)
        if route.get("kind") == "rot":  # skip the wrap around the 3 mm pin
            pts = pts[pts[:, 2] < route["small_bore_end_mm"][2] + 1e-9]
        p = pts[::8]
        for a, b, c in zip(p, p[1:], p[2:]):
            ab, bc, ca = np.linalg.norm(b - a), np.linalg.norm(c - b), np.linalg.norm(a - c)
            area2 = np.linalg.norm(np.cross(b - a, c - a))
            if area2 > 1e-9:
                assert ab * bc * ca / (2 * area2) >= tr.MIN_BEND_R * 0.97, name


def test_strands_are_vertical_below_the_wrist_and_tangent_to_their_spool():
    for r in ROUTES:
        w, t, c = map(np.array, (r["wrist_bottom_mm"], r["tangent_mm"], r["spool_center_mm"]))
        assert np.allclose(w[:2], t[:2]) and t[2] < w[2]
        assert abs(np.linalg.norm(t - c) - tr.SPOOL_R) < 1e-9
        assert abs(t[1] - c[1]) < 1e-9  # in the spool's plane
        assert abs(abs(t[0] - c[0]) - tr.SPOOL_R) < 1e-9  # tangent to a vertical line


def test_no_strand_is_blocked_by_another_servo_or_spool():
    for r in ROUTES:
        x, y, _ = r["tangent_mm"]
        z_top, z_bot = tr.Z_WRIST_PLATE_BOTTOM, r["tangent_mm"][2]
        for s in SERVOS:
            if s.id == r["servo_id"]:
                continue
            for box in (s.case_box(), s.tab_box(), s.spool_box()):
                (bx0, bx1), (by0, by1), (bz0, bz1) = box
                clear = 0.5
                hit = (
                    bx0 - clear < x < bx1 + clear
                    and by0 - clear < y < by1 + clear
                    and bz1 > z_bot
                    and bz0 < z_top
                )
                assert not hit, f"{r['name']} passes through servo {s.id} ({s.joint})"


def test_servos_do_not_collide():
    for a, b in itertools.combinations(SERVOS, 2):
        for ba in (a.case_box(), a.tab_box(), a.spool_box()):
            for bb in (b.case_box(), b.tab_box(), b.spool_box()):
                overlap = all(ba[i][0] < bb[i][1] and bb[i][0] < ba[i][1] for i in range(3))
                assert not overlap, f"servo {a.id} and {b.id} overlap"


def test_loop_fits_the_servo_range():
    """Tendon travel = drum radius x joint range must fit in 300 deg of spool rotation."""
    lims = {
        n: (float(lo), float(hi))
        for n, lo, hi in re.findall(
            r'\{"(\w+)",\s*\d+,\s*(-?\d+) \* kDegToRad,\s*(-?\d+) \* kDegToRad', FIRMWARE_CFG
        )
    }
    assert lims.keys() == {s.joint for s in SERVOS}
    for joint, (lo, hi) in lims.items():
        assert (hi - lo) * tr.servo_per_joint(joint) < 300 * 0.9, joint


@pytest.mark.skipif(not tr.OUT.exists(), reason="run tendon_router.py first")
def test_json_is_up_to_date():
    assert json.loads(tr.OUT.read_text(encoding="utf-8")) == json.loads(json.dumps(tr.layout()))


def test_servos_and_spools_fit_between_the_forearm_side_walls():
    """The forearm's side walls are 3 mm thick at the palm's x limits (found by Fusion's
    interference check when the thumb spool touched the wall)."""
    x0, x1 = tr.PALM_X
    for s in SERVOS:
        for (bx0, bx1), _y, _z in (s.case_box(), s.tab_box(), s.spool_box()):
            assert x0 + 3 + 0.5 <= bx0 and bx1 <= x1 - 3 - 0.5, (
                f"servo {s.id} ({s.joint}) hits a side wall"
            )
