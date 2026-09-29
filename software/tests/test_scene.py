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
    assert m.nu == 21 and m.ntendon == 42
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

    # Home: at the home point, palm (model -Y) facing the camera, fingers (model +Z) up.
    pos, quat = scene.view_to_world(np.zeros(3), HOME_ROT_VIEW)
    assert np.allclose(pos, scene.home_pos)
    r = quat_to_mat(quat)
    assert np.dot(-r[:, 1], to_cam / np.linalg.norm(to_cam)) > 0.95
    assert r[2, 2] > 0.8  # fingers point up in the world

    # Moving right / up / toward the viewer in the view frame moves the hand right / up /
    # toward the view camera in its image.
    def in_camera(p):  # camera coordinates: x right, y up, -z forward
        return c.T @ (p - cam_pos)

    home_cam = in_camera(scene.home_pos)
    for axis in range(3):
        step = np.zeros(3)
        step[axis] = 0.05
        moved = in_camera(scene.view_to_world(step, HOME_ROT_VIEW)[0]) - home_cam
        assert np.allclose(moved, step, atol=1e-9)
    closer = scene.view_to_world(np.array([0, 0, 0.05]), HOME_ROT_VIEW)[0]
    assert np.linalg.norm(cam_pos - closer) < np.linalg.norm(to_cam)

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
    rng = np.random.default_rng(3)
    for kind in scene.config.objects:
        scene.reset(rng, obj=kind)
        assert scene.active_object == kind
        pos, _ = scene.object_pose()
        lo, hi = scene.config.spawn_lo, scene.config.spawn_hi
        assert lo[0] - 0.01 <= pos[0] <= hi[0] + 0.01 and lo[1] - 0.01 <= pos[1] <= hi[1] + 0.01
        table, hand = scene.object_contacts()
        assert table and not hand
        assert not scene.lifted()
        assert np.allclose(scene.finger_positions(), 0, atol=0.05)


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
