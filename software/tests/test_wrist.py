"""Wrist pose tests with synthetic hands seen by a synthetic pinhole camera."""

import math
import time

import numpy as np
import pytest
from tendra.hand_tracking import TrackedHand
from tendra.retarget import FINGERS
from tendra.wrist import WristTracker, focal_length, solve_translation

# Canonical hand (as in test_retarget): wrist at the origin, +X thumb side, +Z along the fingers,
# palm toward -Y (right-hand geometry). Metres. Fingers slightly bent so the bend cue is clear.
KNUCKLES = {"index": (0.022, 0, 0.090), "middle": (0.0, 0, 0.095),
            "ring": (-0.020, 0, 0.088), "little": (-0.038, 0, 0.080)}  # fmt: skip
BONES = (0.040, 0.025, 0.022)
THUMB = [(0.020, -0.010, 0.025), (0.045, -0.015, 0.045), (0.060, -0.020, 0.065),
         (0.070, -0.025, 0.085)]  # fmt: skip
BEND = (0.3, 0.4, 0.2)

SIZE = (640, 480)
HFOV = 62.0
F = focal_length(SIZE[0], HFOV)
C = np.array(SIZE) / 2

# Canonical -> camera frame (x right, y down, z away) for a palm facing the camera, fingers up:
# X (thumb, right hand) -> +x, Y (back of the hand) -> +z (away), Z (fingers) -> -y (up).
FACING = np.column_stack([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])


def canonical_hand() -> np.ndarray:
    lm = np.zeros((21, 3))
    lm[1:5] = THUMB
    for finger, (mcp, *rest) in FINGERS.items():
        lm[mcp] = KNUCKLES[finger]
        total = 0.0
        for idx, bend, length in zip(rest, BEND, BONES, strict=True):
            total += bend
            lm[idx] = lm[idx - 1] + length * np.array([0, -math.sin(total), math.cos(total)])
    return lm


def random_rotation(rng) -> np.ndarray:
    q, r = np.linalg.qr(rng.normal(size=(3, 3)))
    q *= np.sign(np.diag(r))
    return q if np.linalg.det(q) > 0 else -q


def camera_hand(R: np.ndarray, palm_centre: np.ndarray, left: bool = False,
                world_noise: float = 0.0, px_noise: float = 0.0, rng=None):  # fmt: skip
    """A TrackedHand whose WRIST (landmark 0, the tracked point) sits at `palm_centre` (camera
    frame) with rotation R. (The name is historical: it used to be the palm centre.)

    Returns the hand and the true translation T (world landmarks are centred on their mean, as
    MediaPipe centres them roughly on the hand).
    """
    lm = canonical_hand()
    if left:
        lm = lm * np.array([-1.0, 1, 1])  # mirror image of a right hand = a left hand
    pts = lm @ R.T
    pts += palm_centre - pts[0]  # true positions, camera frame
    image = F * pts[:, :2] / pts[:, 2:] + C
    world = pts - pts.mean(axis=0)
    T = pts.mean(axis=0)
    if rng is not None:
        world = world + rng.normal(scale=world_noise, size=world.shape)
        image = image + rng.normal(scale=px_noise, size=image.shape)
    label = "Right" if left else "Left"  # MediaPipe assumes a mirrored image
    return TrackedHand(world, image, label, 0.9), T


def tilted(rng, max_deg: float = 60.0) -> np.ndarray:
    """FACING, turned by a random rotation of up to `max_deg` (palm still roughly visible)."""
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    a = math.radians(rng.uniform(0, max_deg))
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return (np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K) @ FACING


def random_centre(rng) -> np.ndarray:
    z = rng.uniform(0.3, 0.9)
    return np.array([rng.uniform(-0.25, 0.25) * z, rng.uniform(-0.2, 0.2) * z, z])


@pytest.mark.parametrize("left", [False, True])
def test_translation_exact_without_noise(left):
    rng = np.random.default_rng(0)
    for k in range(50):
        R = random_rotation(rng) if k % 2 else tilted(rng, 80)
        hand, T = camera_hand(R, random_centre(rng), left)
        got, rms = solve_translation(hand.world, hand.image, F, C)
        assert np.linalg.norm(got - T) < 1e-3
        assert rms < 1e-6


def test_translation_with_webcam_noise():
    """2 px image noise and 3 mm world-landmark noise, hand 0.3-0.9 m from the camera."""
    rng = np.random.default_rng(1)
    errors, depth = [], []
    for _ in range(300):
        centre = random_centre(rng)
        hand, T = camera_hand(tilted(rng), centre, rng.random() < 0.5, 0.003, 2.0, rng)
        got, _ = solve_translation(hand.world, hand.image, F, C)
        errors.append(np.linalg.norm(got[:2] - T[:2]))
        depth.append(abs(got[2] - T[2]) / T[2])
    errors, depth = np.array(errors), np.array(depth)
    print(
        f"\nsideways error: median {np.median(errors) * 1000:.1f} mm, "
        f"95% {np.percentile(errors, 95) * 1000:.1f} mm; depth error: median "
        f"{np.median(depth) * 100:.1f}%, 95% {np.percentile(depth, 95) * 100:.1f}% of distance"
    )
    assert np.median(errors) < 0.005 and np.percentile(errors, 95) < 0.015
    assert np.median(depth) < 0.02 and np.percentile(depth, 95) < 0.06


def test_home_pose_and_orientation_facing_camera():
    """Palm to the camera, fingers up, at the image centre and the nominal distance.

    View axes: x right, y up, z toward the viewer. Fingers up -> Z_hand = +y; back of the hand
    away from the viewer -> Y_hand = -z; X_hand = Y x Z = +x (robot thumb on the screen's
    right). The same for BOTH hands: the robot is always a right hand. A real left hand matches
    it exactly (its mirror image also has the thumb on the right); a real right hand, whose
    mirror image has its thumb on the left, drives it as its mirror image.
    """
    expected = np.column_stack([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])
    for left in (False, True):
        tr = WristTracker(smoothing=False)
        hand, _ = camera_hand(FACING, np.array([0, 0, 0.5]), left)
        pose = tr.update(hand, 0.0)
        assert tr.chirality == (-1.0 if left else 1.0)
        assert np.allclose(pose.pos, 0, atol=1e-6)
        assert pose.distance == pytest.approx(0.5)
        assert np.allclose(pose.rot, expected, atol=1e-6)


@pytest.mark.parametrize("left", [False, True])
def test_orientation_is_a_rotation_that_follows_the_hand(left):
    rng = np.random.default_rng(2)
    tr = WristTracker(smoothing=False)
    mirror = -np.eye(3)  # camera -> view for directions
    for _ in range(20):
        R = random_rotation(rng)
        pose = tr.update(camera_hand(R, np.array([0, 0, 0.5]), left)[0], 0.0)
        assert np.linalg.det(pose.rot) == pytest.approx(1.0)
        assert np.allclose(pose.rot.T @ pose.rot, np.eye(3), atol=1e-9)
        # Fingers (canonical +Z) and back of the hand (canonical +Y) seen in the mirror.
        assert np.allclose(pose.rot[:, 2], mirror @ R[:, 2], atol=1e-6)
        assert np.allclose(pose.rot[:, 1], mirror @ R[:, 1], atol=1e-6)


def test_mirror_mapping_of_position():
    tr = WristTracker(smoothing=False)
    home = tr.update(camera_hand(FACING, np.array([0, 0, 0.5]))[0], 0.0).pos
    # To the camera's left (-x in the unmirrored image) -> right on the mirrored screen (+x).
    p = tr.update(camera_hand(FACING, np.array([-0.1, 0, 0.5]))[0], 0.1).pos
    assert p - home == pytest.approx([0.1, 0, 0], abs=1e-6)
    # Up (-y in the image) -> +y.
    p = tr.update(camera_hand(FACING, np.array([0, -0.05, 0.5]))[0], 0.2).pos
    assert p - home == pytest.approx([0, 0.05, 0], abs=1e-6)
    # Closer to the camera -> toward the viewer (+z).
    p = tr.update(camera_hand(FACING, np.array([0, 0, 0.4]))[0], 0.3).pos
    assert p - home == pytest.approx([0, 0, 0.1], abs=1e-6)


def test_clutch_freezes_and_resumes_without_a_jump():
    tr = WristTracker(smoothing=False, gain=2.0)
    at = lambda x, z=0.5: camera_hand(FACING, np.array([x, 0, z]))[0]
    p0 = tr.update(at(0.0), 0.0).pos
    p1 = tr.update(at(-0.05), 0.1).pos
    assert p1 - p0 == pytest.approx([0.1, 0, 0], abs=1e-6)  # gain 2

    tr.set_engaged(False)
    assert not tr.engaged
    frozen = tr.update(at(-0.15, 0.3), 0.2)
    assert np.allclose(frozen.pos, p1)  # hand moved, output didn't

    tr.set_engaged(True)
    p2 = tr.update(at(-0.15, 0.3), 0.3).pos
    assert np.allclose(p2, p1, atol=1e-9)  # no jump on re-engaging
    p3 = tr.update(at(-0.10, 0.3), 0.4).pos
    assert p3 - p2 == pytest.approx([-0.1, 0, 0], abs=1e-6)  # relative motion, times gain


def test_recenter_and_per_axis_gain():
    """Start anywhere = home; re-centre keeps the output; depth can have its own gain."""
    tr = WristTracker(smoothing=False, gain=(1.5, 1.5, 2.0))
    tr.recenter()  # as grasp_teleop does at start
    at = lambda x, y=0.0, z=0.4: camera_hand(FACING, np.array([x, y, z]))[0]
    p0 = tr.update(at(0.08, 0.05, 0.62), 0.0).pos  # far off-centre, not at nominal distance
    assert np.allclose(p0, 0.0, atol=1e-9)  # ... is still home
    p1 = tr.update(at(0.04, 0.05, 0.57), 0.1).pos  # 4 cm to camera-left, 5 cm closer
    assert p1 == pytest.approx([0.04 * 1.5, 0, 0.05 * 2.0], abs=1e-6)

    tr.recenter()  # hand jumps back to a comfortable spot: output must not move
    p2 = tr.update(at(0.0, 0.0, 0.5), 0.2).pos
    assert np.allclose(p2, p1, atol=1e-9)
    assert tr.engaged
    p3 = tr.update(at(0.0, -0.02, 0.5), 0.3).pos  # 2 cm up in the camera (y down) = +y view
    assert p3 - p2 == pytest.approx([0, 0.03, 0], abs=1e-6)


def test_orientation_is_absolute_after_the_clutch():
    tr = WristTracker(smoothing=False)
    tr.update(camera_hand(FACING, np.array([0, 0, 0.5]))[0], 0.0)
    tr.set_engaged(False)
    R = tilted(np.random.default_rng(4), 60)
    tr.set_engaged(True)
    pose = tr.update(camera_hand(R, np.array([0.05, 0, 0.6]))[0], 1.0)
    assert np.allclose(pose.rot[:, 2], -R[:, 2], atol=1e-6)


def test_garbage_frame_keeps_last_pose():
    tr = WristTracker(smoothing=False)
    good = tr.update(camera_hand(FACING, np.array([0.02, 0, 0.5]))[0], 0.0)
    hand, _ = camera_hand(FACING, np.array([0, 0, 0.5]))
    rng = np.random.default_rng(5)
    bad = TrackedHand(hand.world, rng.uniform(0, 640, size=(21, 2)), "Left", 0.9)  # nonsense
    nan = TrackedHand(hand.world * np.nan, hand.image, "Left", 0.9)
    for frame in (bad, nan):
        pose = tr.update(frame, 0.1)
        assert np.allclose(pose.pos, good.pos) and np.allclose(pose.rot, good.rot)


def test_depth_is_clamped():
    tr = WristTracker(smoothing=False)
    pose = tr.update(camera_hand(FACING, np.array([0, 0, 2.5]))[0], 0.0)
    assert pose.pos[2] == pytest.approx(0.5 - 1.5)


def test_smoothing_reduces_jitter_and_quaternion_sign_flips_are_harmless():
    rng = np.random.default_rng(6)
    tr = WristTracker()
    raw_z, out_z = [], []
    R = tilted(rng, 40)
    for k in range(60):
        hand, _ = camera_hand(R, np.array([0, 0, 0.5]), False, 0.003, 2.0, rng)
        out = tr.update(hand, k / 25)
        raw_z.append(tr.raw(hand).pos[2])
        out_z.append(out.pos[2])
        assert np.linalg.det(out.rot) == pytest.approx(1.0)
    print(
        f"\ndepth jitter at 0.5 m: raw {np.std(raw_z[20:]) * 1000:.1f} mm, "
        f"smoothed {np.std(out_z[20:]) * 1000:.1f} mm"
    )
    assert np.std(out_z[20:]) < 0.6 * np.std(raw_z[20:])
    assert np.allclose(out.rot[:, 2], -R[:, 2], atol=0.05)


def test_update_is_fast():
    rng = np.random.default_rng(7)
    tr = WristTracker()
    hands = [camera_hand(tilted(rng), random_centre(rng), False, 0.003, 2.0, rng)[0]
             for _ in range(200)]  # fmt: skip
    for h in hands[:10]:  # warm-up
        tr.update(h, 0.0, chirality=1.0)
    runs = []
    for _ in range(3):  # best of three: other programs on the laptop only ever slow it down
        t0 = time.perf_counter()
        for k, h in enumerate(hands):
            tr.update(h, k / 25, chirality=1.0)  # as in the teleop loop (Retargeter's chirality)
        runs.append((time.perf_counter() - t0) / len(hands))
    per = min(runs)
    print(f"\nWristTracker.update: {per * 1e6:.0f} us per frame")
    assert per < 0.003  # target < 1 ms; loose bound for a busy laptop
