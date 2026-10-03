"""The grasp scene: the two-arm Tendra robot at a table, and the arms' IK controller.

One scene per module (building takes ~5 s: two hands, simplified meshes).
"""

import mujoco
import numpy as np
import pytest
from tendra.arm import ARM_JOINTS, SIDES, WRIST_SITE, ArmIK, ik_model, joint_names
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
    return GraspScene()


def rotation_error(q1: np.ndarray, q2: np.ndarray) -> float:
    return 2 * float(np.arccos(np.clip(abs(np.dot(q1, q2)), 0, 1)))


def mirror_pos(scene, p):
    return np.array([2 * scene.mirror_x - p[0], p[1], p[2]])


def test_structure(scene):
    m = scene.model
    assert m.nu == len(SIDES) * (len(ARM_JOINTS) + V1.num_joints) and m.ntendon == 72
    assert m.neq == 8  # the DIP couplings of both hands; nothing is welded: the arms hold the hands
    for cam in GraspScene.CAMERAS:
        assert m.camera(cam).id >= 0
    for side in SIDES:
        arm = scene.sides[side]
        assert [m.actuator(i).name for i in arm.act] == [f"{side}_{n}" for n in V1.joint_names]
        assert [m.actuator(i).name for i in arm.arm_act] == list(joint_names(side))
        assert m.body(f"{side}_palm").id in arm.hand_bodies
        assert m.body(f"{side}_index_base").id in arm.hand_bodies
        assert m.body(f"openarm_{side}_link4").id not in arm.hand_bodies
        other = scene.sides["left" if side == "right" else "right"]
        assert not (arm.hand_bodies & other.hand_bodies)
    with pytest.raises(KeyError):
        m.body("hand_root")  # nothing floats any more


def test_homes_are_mirror_images(scene):
    right, left = scene.sides["right"], scene.sides["left"]
    assert np.allclose(left.home_pos, mirror_pos(scene, right.home_pos), atol=1e-6)
    assert np.allclose(left.ready, right.ready)  # equal angles = mirrored postures
    # Home orientation of the right hand: fingers forward (-y), thumb up, palm toward +x.
    assert np.allclose(quat_to_mat(right.home_quat), SIDE_GRASP_ROT, atol=1e-6)
    scene.reset(np.random.default_rng(0), side="left")
    assert np.allclose(
        quat_to_mat(scene.home_quat), scene.canon_rot(SIDE_GRASP_ROT), atol=1e-6
    )


def test_canonical_mapping_is_an_involution_and_mirrors(scene):
    rng = np.random.default_rng(0)
    scene.reset(rng, side="left")
    p, v, w = rng.normal(size=(3, 3))
    quat = rng.normal(size=4)
    r = quat_to_mat(quat / np.linalg.norm(quat))  # a random rotation
    assert np.allclose(scene.canon_pos(scene.canon_pos(p)), p)
    assert np.allclose(scene.canon_vec(scene.canon_vec(v)), v)
    assert np.allclose(scene.canon_axial(scene.canon_axial(w)), w)
    assert np.allclose(scene.canon_rot(scene.canon_rot(r)), r)
    assert np.isclose(np.linalg.det(scene.canon_rot(r)), np.linalg.det(r))  # still a rotation
    scene.reset(rng, side="right")
    assert np.array_equal(scene.canon_pos(p), p) and np.array_equal(scene.canon_rot(r), r)


@pytest.mark.parametrize("side", SIDES)
def test_starts_at_home_holding_still(scene, side):
    scene.reset(np.random.default_rng(0), side=side)
    assert scene.side == side
    pos, quat = scene.wrist_pose()
    assert np.linalg.norm(pos - scene.home_pos) < 1e-3
    assert rotation_error(quat, scene.home_quat) < 0.03  # the fingers' weight tilts it ~1 deg
    scene.step(1.0)
    assert np.linalg.norm(scene.wrist_pose()[0] - scene.home_pos) < 0.003
    assert not scene.object_contacts()[1]  # nothing touches the active hand
    assert scene.object_pose()[0][2] == pytest.approx(scene.rest_height, abs=1e-3)
    # The object is on the active hand's side of the table (right hand: x < 0).
    assert (scene.object_pose()[0][0] < scene.mirror_x) == (side == "right")


def test_the_resting_arm_sleeps_and_the_working_one_does_not(scene):
    for side in SIDES:
        scene.reset(np.random.default_rng(0), side=side)
        scene.step(0.5)
        other = "left" if side == "right" else "right"
        assert scene.data.tree_asleep[scene.sides[other].tree] >= 0  # asleep
        assert scene.data.tree_asleep[scene.sides[side].tree] < 0  # awake
        before = scene.wrist_pose(other)[0].copy()
        scene.step(0.5)
        assert np.allclose(scene.wrist_pose(other)[0], before)  # and it stays where it was


def test_without_sleeping_both_arms_follow_their_targets():
    scene = GraspScene(SceneConfig(sleep_idle=False, lite_keep=0.05))
    scene.reset(np.random.default_rng(0), side="right")
    for side in SIDES:
        target = scene.sides[side].home_pos + np.array([0.0, -0.04, 0.03])
        scene.set_wrist_target(target, scene.sides[side].home_quat, side)
    scene.step(1.5)
    for side in SIDES:
        assert np.linalg.norm(scene.wrist_pose(side)[0] - scene.wrist_target(side)[0]) < 0.004


@pytest.mark.parametrize("side", SIDES)
def test_wrist_follows_a_target_through_the_arm(scene, side):
    scene.reset(np.random.default_rng(0), side=side)
    start = scene.canon_vec([0.05, -0.05, 0.04])  # 5 cm toward the middle/forward/up
    target = scene.home_pos + start
    turn = mat_to_quat(
        quat_to_mat(scene.home_quat) @ scene.canon_rot(quat_to_mat([0.9659, 0.2588, 0, 0]))
    )
    scene.set_wrist_target(target, turn)  # and 30 deg about the thumb axis
    scene.step(1.5)
    pos, quat = scene.wrist_pose()
    assert np.linalg.norm(pos - target) < 0.004
    assert rotation_error(quat, turn) < 0.03
    assert scene.ik_error < 1e-3
    assert not np.allclose(scene.arm_positions(), scene.arm_ready, atol=0.02)  # the arm moved


@pytest.mark.parametrize("side", SIDES)
def test_unreachable_target_is_followed_as_far_as_possible(scene, side):
    """Far beyond the arm's reach: the arm stretches toward it, stays inside its limits."""
    scene.reset(np.random.default_rng(0), side=side)
    scene.set_wrist_target(scene.home_pos + np.array([0.0, -1.0, 0.0]), scene.home_quat)
    scene.step(2.0)
    assert scene.ik_error > 0.1  # it reports that the pose is out of reach
    lo, hi = scene.arm.ik.lo, scene.arm.ik.hi
    q = scene.arm_positions()
    assert np.all(q >= lo - 0.01) and np.all(q <= hi + 0.01)  # joint limits are soft
    assert np.all(np.isfinite(scene.data.qpos))
    assert scene.wrist_pose()[0][1] < scene.home_pos[1] - 0.1  # it did reach out


def test_wrist_velocity_matches_the_motion(scene):
    """The velocity integrates to the displacement (linear: world frame)."""
    scene.reset(np.random.default_rng(0), side="right")
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


@pytest.mark.parametrize("side", SIDES)
def test_scripted_grasp_lifts_over_the_spawn_box(scene, side):
    """Either hand reaches every object position the scene spawns, with the real servo limits."""
    rng = np.random.default_rng(5)
    for i in range(5):
        scene.reset(rng, obj="cylinder", side=side)
        xy = scene.canon_pos(scene.object_pose()[0])[:2]  # in the right hand's view
        lo, hi = scene.config.spawn_lo, scene.config.spawn_hi
        assert np.all(xy >= np.array(lo) - 0.02) and np.all(xy <= np.array(hi) + 0.02)
        scripted_grasp(scene, **ARM_GRASP)
        assert scene.lifted(0.05), f"spawn {i} at {xy}"


@pytest.mark.parametrize("obj", ["cube", "ball"])
def test_scripted_grasp_of_the_small_objects(scene, obj):
    """The cube and ball sit too low for a level side grasp (the forearm would hit the table); a
    small pitch and height (`ARM_GRASP`) lift them with either hand."""
    rng = np.random.default_rng(6)
    lifted = 0
    for k in range(6):
        scene.reset(rng, obj=obj, side=SIDES[k % 2])
        scripted_grasp(scene, hold=0.6, **ARM_GRASP)
        lifted += scene.lifted(0.05)
    assert lifted >= 5


def test_the_left_grasp_is_the_mirror_of_the_right_one(scene):
    """The same spawn mirrored: the scripted grasp ends with the hand and object mirrored."""
    ends = {}
    for side in SIDES:
        scene.reset(np.random.default_rng(11), obj="cylinder", side=side)
        scripted_grasp(scene, hold=0.3, **ARM_GRASP)
        ends[side] = (scene.wrist_pose()[0], scene.object_pose()[0], scene.finger_positions())
    assert np.allclose(ends["left"][0], mirror_pos(scene, ends["right"][0]), atol=0.006)
    assert np.allclose(ends["left"][1], mirror_pos(scene, ends["right"][1]), atol=0.008)
    assert np.allclose(ends["left"][2], ends["right"][2], atol=0.3)  # contact decides the grip


@pytest.mark.parametrize("side", SIDES)
def test_ik_model_matches_the_scene(scene, side):
    """The bare kinematic copy puts the wrist where the full model does, at any arm angles."""
    arm = scene.sides[side]
    rng = np.random.default_rng(1)
    data = mujoco.MjData(scene.model)
    for _ in range(8):
        q = arm.ready + rng.uniform(-0.3, 0.3, len(ARM_JOINTS))
        data.qpos[:] = scene.model.qpos0
        data.qpos[arm.arm_qadr] = q
        mujoco.mj_kinematics(scene.model, data)
        pos, rot = arm.ik.pose(q)
        assert np.allclose(pos, data.site_xpos[arm.site], atol=1e-6)
        assert np.allclose(rot, data.site_xmat[arm.site].reshape(3, 3), atol=1e-6)


@pytest.mark.parametrize("side", SIDES)
def test_ik_reaches_reachable_poses(scene, side):
    arm = scene.sides[side]
    ik = ArmIK(ik_model(side), joint_names(side), f"{side}_{WRIST_SITE}", rest=arm.ready)
    rng = np.random.default_rng(1)
    for _ in range(10):
        q = arm.ready + rng.uniform(-0.3, 0.3, len(ARM_JOINTS))
        pos, rot = ik.pose(q)
        solved, err = ik.solve(arm.ready, pos, rot, iters=60)
        pos2, rot2 = ik.pose(solved)
        assert err < 1e-3 and np.linalg.norm(pos2 - pos) < 1e-3
        assert np.linalg.norm(rot2 - rot) < 0.01


@pytest.mark.parametrize("side", SIDES)
def test_fingers_work_through_the_scene(scene, side):
    scene.reset(np.random.default_rng(0), side=side)
    scene.set_finger_targets(np.full(V1.num_joints, 0.5) * (V1.upper > 0.6))
    scene.step(1.0)
    assert np.abs(scene.finger_positions() - scene.finger_targets()).max() < 0.15
    assert scene.all_finger_positions().shape == (len(V1.all_joint_names),)
