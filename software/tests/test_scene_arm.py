"""The grasp scene with the Tendra arm (SceneConfig(arm=True)) and the arm's IK controller.

One scene per module (building takes ~2 s).
"""

import mujoco
import numpy as np
import pytest
from tendra.arm import ARM_JOINTS, ArmIK
from tendra.joints import V1
from tendra.scene import (
    ARM_GRASP,
    SIDE_GRASP_ROT,
    GraspScene,
    SceneConfig,
    mat_to_quat,
    quat_to_mat,
    scripted_grasp,
)

pytest.importorskip("fast_simplification")


@pytest.fixture(scope="module")
def scene():
    return GraspScene(SceneConfig(arm=True))


def rotation_error(q1: np.ndarray, q2: np.ndarray) -> float:
    return 2 * float(np.arccos(np.clip(abs(np.dot(q1, q2)), 0, 1)))


def test_structure(scene):
    m = scene.model
    assert m.nu == len(ARM_JOINTS) + V1.num_joints and m.ntendon == 36
    assert m.neq == 4  # the DIP couplings; no weld: the arm holds the hand
    for name in V1.joint_names:
        assert m.actuator(name).id in scene._act
    with pytest.raises(KeyError):
        m.body("hand_root")
    # Everything of the hand hangs from the forearm; the arm's own bodies are not "hand".
    assert scene._in_hand(m.body("palm").id) and scene._in_hand(m.body("index_base").id)
    assert not scene._in_hand(m.body("openarm_right_link4").id)
    assert scene.arm_positions().shape == (7,) and scene.arm_targets().shape == (7,)


def test_starts_at_home_holding_still(scene):
    scene.reset(np.random.default_rng(0))
    pos, quat = scene.wrist_pose()
    assert np.linalg.norm(pos - scene.home_pos) < 1e-3
    assert rotation_error(quat, scene.home_quat) < 0.03  # the fingers' weight tilts it ~1 deg
    # Home orientation: fingers forward (-y), thumb up, palm toward the body's middle (+x).
    assert np.allclose(quat_to_mat(scene.home_quat), SIDE_GRASP_ROT, atol=1e-6)
    scene.step(1.0)
    assert np.linalg.norm(scene.wrist_pose()[0] - scene.home_pos) < 0.003
    assert scene.data.ncon == 0 or not scene.object_contacts()[1]  # nothing touches the hand
    assert scene.object_pose()[0][2] == pytest.approx(scene.rest_height, abs=1e-3)


def test_wrist_follows_a_target_through_the_arm(scene):
    scene.reset(np.random.default_rng(0))
    target = scene.home_pos + np.array([0.05, -0.05, 0.04])
    turn = mat_to_quat(quat_to_mat(scene.home_quat) @ quat_to_mat([0.9659, 0.2588, 0, 0]))
    scene.set_wrist_target(target, turn)  # 5 cm right/forward/up, 30 deg about the thumb axis
    scene.step(1.5)
    pos, quat = scene.wrist_pose()
    assert np.linalg.norm(pos - target) < 0.004
    assert rotation_error(quat, turn) < 0.03
    assert scene.ik_error < 1e-3
    assert not np.allclose(scene.arm_positions(), scene.arm_ready, atol=0.02)  # the arm moved


def test_unreachable_target_is_followed_as_far_as_possible(scene):
    """Far beyond the arm's reach: the arm stretches toward it, stays inside its limits, is stable."""
    scene.reset(np.random.default_rng(0))
    scene.set_wrist_target(scene.home_pos + np.array([0.0, -1.0, 0.0]), scene.home_quat)
    scene.step(2.0)
    assert scene.ik_error > 0.1  # it reports that the pose is out of reach
    lo, hi = scene._ik.lo, scene._ik.hi
    q = scene.arm_positions()
    assert np.all(q >= lo - 1e-3) and np.all(q <= hi + 1e-3)
    assert np.all(np.isfinite(scene.data.qpos))
    assert scene.wrist_pose()[0][1] < scene.home_pos[1] - 0.1  # it did reach out


def test_wrist_velocity_matches_the_motion(scene):
    """The velocity integrates to the displacement (linear: world frame)."""
    scene.reset(np.random.default_rng(0))
    scene.set_wrist_target(scene.home_pos + np.array([0.0, -0.03, 0.0]), scene.home_quat)
    p0 = scene.wrist_pose()[0]
    travelled = np.zeros(3)
    dt = scene.model.opt.timestep
    for _ in range(100):
        scene.step(dt)
        travelled += scene.wrist_velocity()[:3] * dt
    moved = scene.wrist_pose()[0] - p0
    assert moved[1] < -0.02  # toward the target (-y)
    assert np.linalg.norm(travelled - moved) < 0.1 * np.linalg.norm(moved)
    assert np.all(np.isfinite(scene.wrist_velocity()))


def test_scripted_grasp_lifts_over_the_spawn_box(scene):
    """The arm can reach every object position the scene spawns, with the real servo limits."""
    rng = np.random.default_rng(5)
    for i in range(6):
        scene.reset(rng, obj="cylinder")
        lo, hi = scene.config.arm_spawn_lo, scene.config.arm_spawn_hi
        xy = scene.object_pose()[0][:2]
        assert np.all(xy >= np.array(lo) - 0.02) and np.all(xy <= np.array(hi) + 0.02)
        scripted_grasp(scene)
        assert scene.lifted(0.05), f"spawn {i} at {xy}"


@pytest.mark.parametrize("obj", ["cube", "ball"])
def test_scripted_grasp_of_the_small_objects_needs_the_arm_grasp(scene, obj):
    """The cube and ball sit too low for the level side grasp (the forearm would hit the table);
    a small pitch and height (`ARM_GRASP`) lift them from the whole spawn box."""
    rng = np.random.default_rng(6)
    lifted = 0
    for _ in range(6):
        scene.reset(rng, obj=obj)
        scripted_grasp(scene, hold=0.6, **ARM_GRASP)
        lifted += scene.lifted(0.05)
    assert lifted >= 5


def test_ik_matches_forward_kinematics(scene):
    ik = ArmIK(scene.model, rest=scene.arm_ready)
    rng = np.random.default_rng(1)
    qpos = scene.data.qpos.copy()
    for _ in range(10):
        q = scene.arm_ready + rng.uniform(-0.3, 0.3, 7)
        pos, rot = ik.pose(qpos, q)
        solved, err = ik.solve(qpos, pos, rot, iters=50)  # from the ready pose to a reachable one
        pos2, rot2 = ik.pose(qpos, solved)
        assert err < 1e-3 and np.linalg.norm(pos2 - pos) < 1e-3
        assert np.linalg.norm(rot2 - rot) < 0.01


def test_fingers_work_through_the_arm_scene(scene):
    scene.reset(np.random.default_rng(0))
    scene.set_finger_targets(np.full(V1.num_joints, 0.5) * (V1.upper > 0.6))
    scene.step(1.0)
    assert np.abs(scene.finger_positions() - scene.finger_targets()).max() < 0.15


def test_floating_scene_is_unchanged():
    scene = GraspScene(SceneConfig())
    assert scene.model.nu == V1.num_joints and scene.arm_positions().size == 0
    assert scene.wrist_velocity().shape == (6,)
    pos, mat = scene.wrist_frame()
    assert np.allclose(pos, scene.wrist_pose()[0]) and np.allclose(mat @ mat.T, np.eye(3))
    assert mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, "hand_root") >= 0
