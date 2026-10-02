"""The grasp scene: floating V1 hand + table + objects (tendra.scene).

One scene is built per test module (building takes ~2 s for the mesh simplification).
"""

import time

import mujoco
import numpy as np
import pytest
from tendra.joints import V1
from tendra.scene import HOME_ROT_VIEW, GraspScene, mat_to_quat, quat_to_mat

pytest.importorskip("fast_simplification")


@pytest.fixture(scope="module")
def scene():
    return GraspScene()


def angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    """Rotation angle (rad) between two quaternions."""
    return 2 * float(np.arccos(np.clip(abs(np.dot(q1, q2)), 0, 1)))


# ----- structure -----


def test_structure(scene):
    m = scene.model
    assert (
        m.nu == 16 and m.ntendon == 36 and m.neq == 5
    )  # 32 strands + 4 DIP bars; 4 DIP couplings + the wrist weld
    assert [m.actuator(i).name for i in range(m.nu)] == list(V1.joint_names)
    for cam in GraspScene.CAMERAS:
        assert m.camera(cam).id >= 0
    for site in ("index_tip", "middle_tip", "ring_tip", "little_tip", "thumb_tip"):
        assert m.site_bodyid[m.site(site).id] != 0
    # The hand's own contact excludes survived the attach.
    assert m.nexclude == 6
    # Nothing of the hand is left on the world body (everything moves with the wrist).
    for b in range(1, m.nbody):
        name = m.body(b).name
        if name not in ("hand_root", "wrist_target", *scene.config.objects):
            assert scene._in_hand(b), name


def test_view_mapping(scene):
    c = scene.view_basis
    assert np.allclose(c.T @ c, np.eye(3), atol=1e-9) and np.isclose(np.linalg.det(c), 1)
    cam = scene.model.camera("view").id
    mujoco.mj_camlight(scene.model, scene.data)
    cam_pos = scene.data.cam_xpos[cam]
    to_cam = cam_pos - scene.home_pos

    # Home: at the home point, palm (model -Y) facing the camera, fingers (model +Z) straight up.
    pos, quat = scene.view_to_world(np.zeros(3), HOME_ROT_VIEW)
    assert np.allclose(pos, scene.home_pos)
    r = quat_to_mat(quat)
    assert np.dot(-r[:, 1], to_cam / np.linalg.norm(to_cam)) > 0.8  # camera is ~30 deg above
    assert r[2, 2] > 0.999  # fingers point straight up in the world

    # The mapping is levelled: view up = world up, toward the viewer = horizontal toward the
    # camera (moving toward the webcam must not lift the sim hand), right = the camera's right.
    cam_x = scene.data.cam_xmat[cam].reshape(3, 3)[:, 0]
    right, up, toward = (scene.view_to_world(s, HOME_ROT_VIEW)[0] - scene.home_pos
                         for s in 0.05 * np.eye(3))  # fmt: skip
    assert np.allclose(up, [0, 0, 0.05], atol=1e-9)
    assert abs(toward[2]) < 1e-9 and abs(right[2]) < 1e-9
    assert np.dot(right, cam_x) > 0.049
    horizontal_to_cam = np.array([to_cam[0], to_cam[1], 0.0])
    assert np.dot(toward, horizontal_to_cam / np.linalg.norm(horizontal_to_cam)) > 0.0499

    # A rotation in the view frame is the same rotation about the camera's axes.
    tilt = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)  # 90 deg about view x
    _, q = scene.view_to_world(np.zeros(3), tilt @ HOME_ROT_VIEW)
    assert np.allclose(quat_to_mat(q), c @ tilt @ HOME_ROT_VIEW, atol=1e-9)


# ----- wrist tracking -----


def test_wrist_tracking(scene):
    scene.reset(np.random.default_rng(1), obj="ball")
    pos, quat = scene.wrist_pose()
    assert np.linalg.norm(pos - scene.home_pos) < 0.003  # holds still at home
    turn = mat_to_quat(quat_to_mat(scene.home_quat) @ quat_to_mat([0.9239, 0.3827, 0, 0]))
    target = scene.home_pos + np.array([0.06, 0.03, 0.04])
    scene.set_wrist_target(target, turn)  # 6 cm right, 45 deg about the thumb axis
    scene.step(0.4)
    pos, quat = scene.wrist_pose()
    assert np.linalg.norm(pos - target) < 0.003
    assert angle_between(quat, turn) < np.radians(2)
    scene.step(1.0)  # stays there against gravity
    pos, quat = scene.wrist_pose()
    assert np.linalg.norm(pos - target) < 0.002
    assert np.linalg.norm(scene.data.qvel[:6]) < 0.01


def test_reset_places_one_object(scene):
    """Every spawn: object at rest on the table, upright, clear of the hand (the forearm reaches
    ~10 cm in front of the wrist; a spawn against it once knocked the cylinder over)."""
    from tendra.scene import quat_to_mat

    rng = np.random.default_rng(3)
    sides = set()
    for kind in scene.config.objects:
        for _ in range(20):
            scene.reset(rng, obj=kind)
            assert scene.active_object == kind
            pos, quat = scene.object_pose()
            lo, hi = scene.config.spawn_lo, scene.config.spawn_hi
            x = abs(pos[0]) if scene.config.spawn_sides else pos[0]
            sides.add(np.sign(pos[0]))
            assert lo[0] - 0.01 <= x <= hi[0] + 0.01
            assert lo[1] - 0.01 <= pos[1] <= hi[1] + 0.01
            table, hand = scene.object_contacts()
            assert table and not hand, (kind, pos)
            if kind != "ball":
                assert quat_to_mat(quat)[2, 2] > 0.98, (kind, pos)  # still upright
            assert not scene.lifted()
            assert np.allclose(scene.finger_positions(), 0, atol=0.05)
    if scene.config.spawn_sides:
        assert sides == {-1.0, 1.0}  # both sides of the hand get used


# ----- grasping -----


def test_scripted_grasp_lifts_cylinder(scene):
    from tendra.scene import scripted_grasp

    scene.reset(np.random.default_rng(0), obj="cylinder")
    scripted_grasp(scene)
    assert scene.lifted(0.05), scene.object_pose()
    # Speed: 1 s of simulation with the object held.
    t0 = time.perf_counter()
    scene.step(1.0)
    wall = time.perf_counter() - t0
    print(f"\n1 s of sim with a held cylinder: {wall:.3f} s wall, ncon {scene.data.ncon}")
    assert scene.lifted(0.05)
    assert wall < 3.0  # generous: the laptop may be busy
