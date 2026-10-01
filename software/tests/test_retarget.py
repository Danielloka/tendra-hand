"""Retargeting tests with synthetic hands: known angles in -> the same angles out."""

import math

import numpy as np
import pytest
from tendra.retarget import FINGERS, OneEuroFilter, Retargeter, bend_cue, palm_frame

# Canonical human hand: wrist at the origin, +X thumb side, +Z along the fingers, palm toward -Y
# (right-hand geometry). Metres.
KNUCKLES = {"index": (0.022, 0, 0.090), "middle": (0.0, 0, 0.095),
            "ring": (-0.020, 0, 0.088), "little": (-0.038, 0, 0.080)}  # fmt: skip
BONES = {"index": (0.040, 0.025, 0.022), "middle": (0.045, 0.028, 0.024),
         "ring": (0.042, 0.027, 0.023), "little": (0.033, 0.020, 0.020)}  # fmt: skip
THUMB_REST = [(0.020, -0.010, 0.025), (0.045, -0.015, 0.045), (0.060, -0.020, 0.065),
              (0.070, -0.025, 0.085)]  # fmt: skip
PALM = np.array([0.0, -1.0, 0.0])


def synthetic_hand(angles=None, thumb=THUMB_REST) -> np.ndarray:
    """21 landmarks; `angles[finger] = (mcp_flex, pip, dip, mcp_abd)` in radians."""
    angles = angles or {}
    lm = np.zeros((21, 3))
    lm[1:5] = thumb
    for finger, (mcp, *rest) in FINGERS.items():
        flex, pip, dip, abd = angles.get(finger, (0, 0, 0, 0))
        k = np.array(KNUCKLES[finger], dtype=float)
        # abd = 0: finger parallel to the palm's up axis (+Z), like Tendra's; + toward the thumb
        d0 = math.cos(abd) * np.array([0.0, 0, 1]) + math.sin(abd) * np.array([1.0, 0, 0])
        lm[mcp] = k
        total = 0.0
        for idx, bend, length in zip(rest, (flex, pip, dip), BONES[finger]):
            total += bend
            lm[idx] = lm[idx - 1] + length * (math.cos(total) * d0 + math.sin(total) * PALM)
    return lm


def random_rotation(rng) -> np.ndarray:
    q, r = np.linalg.qr(rng.normal(size=(3, 3)))
    q *= np.sign(np.diag(r))
    return q if np.linalg.det(q) > 0 else -q


def pose(lm, rng, mirror=False):
    """Rotate (and optionally mirror, making a left hand) the landmarks, as a camera would."""
    if mirror:
        lm = lm * np.array([-1.0, 1.0, 1.0])
    return (lm - lm[0]) @ random_rotation(rng).T + rng.normal(size=3)


# The DIPs are 0.75 x the PIPs, like the v1 robot's coupling, so the v1 fit can recover them exactly.
ANGLES = {
    "index": (0.5, 0.8, 0.6, 0.10),
    "middle": (0.3, 1.0, 0.75, 0.0),
    "ring": (0.7, 0.4, 0.3, -0.08),
    "little": (0.2, 0.6, 0.45, -0.12),
}


@pytest.fixture(scope="module")
def v1():
    return Retargeter("v1", smoothing=False)


@pytest.mark.parametrize("mirror", [False, True])
@pytest.mark.parametrize("label", ["Left", "Right", None])
def test_finger_angles_recovered(v1, mirror, label):
    rng = np.random.default_rng(1)
    names = v1.spec.joint_names
    for _ in range(5):
        q = v1.raw(pose(synthetic_hand(ANGLES), rng, mirror), label)
        for finger, values in ANGLES.items():
            for joint, value in zip(("mcp_flex", "pip", "dip", "mcp_abd"), values):
                if f"{finger}_{joint}" not in names:  # v1 DIPs: coupled, no servo
                    continue
                if joint == "mcp_abd":
                    value *= math.cos(values[0])  # faded with MCP flexion (anatomy)
                assert q[names.index(f"{finger}_{joint}")] == pytest.approx(value, abs=1e-6)


def test_palm_side_found_from_the_bend_not_the_label(v1):
    # A wrong label must not flip the fingers once they bend (the bend cue decides).
    v1.chirality = None
    lm = pose(synthetic_hand(ANGLES), np.random.default_rng(2), mirror=True)
    v1.raw(lm, label="Left")  # wrong: this is left-hand geometry
    assert v1.chirality == -1.0
    assert abs(bend_cue(lm, palm_frame(lm, 1.0).x)) > Retargeter.CUE_THRESHOLD


def test_open_hand_is_zero(v1):
    q = v1.raw(pose(synthetic_hand(), np.random.default_rng(3)))
    fingers = [i for i, n in enumerate(v1.spec.joint_names) if not n.startswith("thumb_")]
    assert np.abs(q[fingers]).max() < 1e-6


def test_calibration_subtracts_the_open_pose():
    rt = Retargeter("v1", smoothing=False)
    bent = {f: (0.1, 0.05, 0.05, 0.03) for f in FINGERS}  # someone's "flat" hand
    rt.calibrate(synthetic_hand(bent))
    q = rt.raw(synthetic_hand(bent))
    fingers = [i for i, n in enumerate(rt.spec.joint_names) if not n.startswith("thumb_")]
    assert np.abs(q[fingers]).max() < 1e-6
    assert not rt.offsets[[i for i, n in enumerate(rt.spec.joint_names) if "thumb" in n]].any()


def test_limits_are_respected(v1):
    wild = {f: (2.5, 2.5, 2.5, 0.6) for f in FINGERS}
    q = Retargeter("v1", smoothing=False)(synthetic_hand(wild), t=0.0)
    assert (q >= v1.spec.lower - 1e-9).all() and (q <= v1.spec.upper + 1e-9).all()


@pytest.mark.parametrize("hand", ["v0", "v1"])
def test_thumb_reproduces_a_robot_thumb_pose(hand):
    """A 'human' hand with the robot's own thumb geometry must be solved back to that pose."""
    rt = Retargeter(hand, smoothing=False)
    solver, spec = rt.thumb, rt.spec
    q_true = np.zeros(spec.num_joints)
    q_true[solver.thumb] = np.clip(
        [{"thumb_cmc_rot": -0.9, "thumb_cmc_flex": 0.4}.get(spec.joint_names[i], 0.3)
         for i in solver.thumb], solver.lower, solver.upper)  # fmt: skip
    p = solver.points(q_true)

    lm = synthetic_hand()
    # Robot and canonical human frames coincide (X thumb, Z fingers, -Y palm), and the thumb is
    # the robot's own, so the scale is 1.
    lm[1] = np.array(THUMB_REST[0])
    for i, key in ((2, "mcp"), (3, "ip"), (4, "tip")):
        lm[i] = lm[1] + (p[key] - p["base"])

    for _ in range(10):  # a few frames: the solver warm-starts from the last one
        q = rt.raw(lm, label="Left")
    got = solver.points(q)
    for key in ("mcp", "ip", "tip"):
        err = np.linalg.norm((got[key] - got["base"]) - (p[key] - p["base"]))
        assert err < 1e-3, f"{key} off by {err * 1000:.1f} mm"


@pytest.mark.parametrize("hand", ["v0", "v1"])
@pytest.mark.parametrize("index_deg", [(47, 60, 70, 5), (60, 50, 60, 5)])
def test_pinch_brings_thumb_to_index(hand, index_deg):
    # Tendra can only pinch with a curled index (an "O" pinch); a straight human index is out of
    # the robot thumb's reach, and the solver then gets as close as it can.
    rt = Retargeter(hand, smoothing=False)
    lm = synthetic_hand({"index": tuple(np.radians(index_deg))})
    lm[4] = lm[8]  # human thumb tip touches the index tip
    lm[3] = lm[8] + np.array([0.012, 0.0, -0.022])
    lm[2] = lm[1] + 0.5 * (lm[3] - lm[1])
    q = rt.raw(lm, label="Left")  # one frame, from a cold start
    assert rt.pinch == 1.0
    p = rt.thumb.points(q)
    gap = np.linalg.norm(p["tip"] - p["index_tip"])
    assert gap < 0.006, f"thumb tip {gap * 1000:.1f} mm from the index tip"


def test_one_euro_filter_smooths_jitter_but_follows_steps():
    f = OneEuroFilter()
    rng = np.random.default_rng(0)
    t = np.arange(0, 2, 1 / 30)
    noisy = [f(np.array([0.5 + 0.02 * rng.normal()]), ti)[0] for ti in t]
    assert np.std(noisy[30:]) < 0.01  # jitter reduced
    out = [f(np.array([1.5]), ti)[0] for ti in t[-1] + t[1:16]]
    assert out[-1] == pytest.approx(1.5, abs=0.05)  # a jump is reached within 0.5 s


# A tight fist: every finger curls far past 90 degrees in total (DIP = 0.75 x PIP, as on v1).
FIST = {
    "index": (1.45, 1.6, 1.2, 0.05),
    "middle": (1.5, 1.7, 1.275, 0.0),
    "ring": (1.4, 1.65, 1.2375, -0.05),
    "little": (1.3, 1.5, 1.125, -0.1),
}


@pytest.mark.parametrize("mirror", [False, True])
@pytest.mark.parametrize("label", ["Left", "Right"])
def test_fist_is_a_fist(mirror, label):
    """Regression: in a fist the old palm-side cue flipped and all fingers went straight."""
    rt = Retargeter("v1", smoothing=False)
    names = rt.spec.joint_names
    rng = np.random.default_rng(7)
    for _ in range(3):
        q = rt.raw(pose(synthetic_hand(FIST), rng, mirror), label)
        for finger, (flex, pip, dip, abd) in FIST.items():
            expected = {"mcp_flex": flex, "pip": pip, "dip": dip,
                        "mcp_abd": abd * math.cos(flex)}  # fmt: skip
            for joint, value in expected.items():
                if f"{finger}_{joint}" not in names:  # v1 DIPs: coupled, no servo
                    continue
                i = names.index(f"{finger}_{joint}")
                want = np.clip(value, rt.spec.lower[i], rt.spec.upper[i])
                assert q[i] == pytest.approx(want, abs=1e-6), (finger, joint)


def test_finger_model_jacobian_matches_finite_differences():
    from tendra.retarget import _finger_model

    rng = np.random.default_rng(5)
    f, side, n = np.eye(3)
    base, lengths = rng.normal(size=3), np.array([0.04, 0.025, 0.022])
    x = np.array([0.1, 1.2, 1.4, 0.9])
    pts, jac = _finger_model(x, base, lengths, f, side, n)
    h = 1e-7
    for k in range(4):
        dx = np.zeros(4)
        dx[k] = h
        numeric = (_finger_model(x + dx, base, lengths, f, side, n)[0] - pts) / h
        assert np.allclose(jac[:, k], numeric, atol=1e-6)


def test_fit_beats_direct_estimate_in_a_noisy_fist():
    """3 mm landmark noise (webcam-like): fitting the whole finger is more precise."""
    from tendra.retarget import _closed_form, finger_angles

    rng = np.random.default_rng(3)
    direct, fitted = [], []
    for _ in range(100):
        lm = pose(synthetic_hand(FIST), rng) + rng.normal(scale=0.003, size=(21, 3))
        frame = palm_frame(lm, 1.0)
        for finger, (flex, pip, dip, abd) in FIST.items():
            truth = np.array([abd, flex, pip, dip])
            direct.append(np.abs(_closed_form(lm, frame, finger) - truth))
            fitted.append(np.abs(finger_angles(lm, frame, finger)[1] - truth))
    direct, fitted = np.mean(direct, axis=0), np.mean(fitted, axis=0)
    assert fitted.sum() < direct.sum()
    assert fitted[0] < 0.6 * direct[0]  # sideways angle: the direct estimate is poor here


@pytest.mark.parametrize("mirror", [False, True])
def test_bend_cue_keeps_its_sign_in_a_fist(mirror):
    rng = np.random.default_rng(11)
    for angles in (ANGLES, FIST):
        lm = pose(synthetic_hand(angles), rng, mirror)
        cue = bend_cue(lm, palm_frame(lm, 1.0).x)
        assert abs(cue) > Retargeter.CUE_THRESHOLD
        assert (cue > 0) == (not mirror)


def test_coupled_fit_puts_the_robot_fingertip_closest():
    """A human DIP that doesn't follow the robot's 0.75 coupling: fitting with the coupling puts
    the robot fingertip nearer the human one than fitting freely and then dropping the DIP."""
    from tendra.retarget import FINGERS, _finger_axes, _finger_model, bone_lengths, finger_angles

    lm = synthetic_hand({"index": (0.5, 1.2, 0.2, 0.0)})  # DIP much straighter than 0.75 x PIP
    frame = palm_frame(lm, 1.0)
    f, side = _finger_axes(lm, frame, "index")
    args = (lm[FINGERS["index"][0]], bone_lengths(lm, "index"), f, side, frame.palm)

    def tip_error(x):
        return np.linalg.norm(_finger_model(x, *args)[0][6:9] - lm[FINGERS["index"][3]])

    coupled = finger_angles(lm, frame, "index", ratio=0.75)[1]
    assert coupled[3] == pytest.approx(0.75 * coupled[2])
    free = finger_angles(lm, frame, "index")[1]
    dropped = free.copy()
    dropped[3] = 0.75 * free[2]  # what the robot does with the free fit's PIP
    assert tip_error(coupled) < 0.8 * tip_error(dropped)
