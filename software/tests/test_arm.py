"""The Tendra arms: OpenArm's shoulders and elbows + Tendra forearms, wrists and V1 hands, both sides."""

import mujoco
import numpy as np
import pytest
from tendra.arm import (
    ARM_JOINTS,
    POSES,
    SIDES,
    WRIST_SITE,
    ArmIK,
    arm_joint_ids,
    hand_joint_names,
    ik_model,
    joint_names,
    load_arm_model,
)
from tendra.joints import V1

MIRROR = np.diag([1.0, -1.0, 1.0])  # the arms' symmetry plane is y = 0 (OpenArm's frame)
HAND_MIRROR = np.diag([-1.0, 1.0, 1.0])  # the left hand model is the right one mirrored in its X


@pytest.fixture(scope="module")
def model():
    return load_arm_model()


def set_q(model, data, side=None, **angles):
    """All joints at 0 except the arm joints given (names without the side), on one side or both."""
    data.qpos[:] = model.qpos0
    for name, q in angles.items():
        for s in [side] if side else SIDES:
            data.qpos[model.joint(f"{s}_{name}").qposadr[0]] = q
    mujoco.mj_forward(model, data)


def test_joints_and_actuators(model):
    assert model.nu == len(SIDES) * (len(ARM_JOINTS) + V1.num_joints)
    for side in SIDES:
        for name in (*joint_names(side), *hand_joint_names(side)):
            model.actuator(name)  # one actuator per arm joint and per hand servo, by name
            model.joint(name)
        for name in V1.joint_names:  # the DIPs have no servo but are joints
            assert model.joint(f"{side}_{name}").id >= 0
    assert [model.actuator(n).name for n in hand_joint_names("left")] == [
        f"left_{n}" for n in V1.joint_names
    ]


def test_each_arm_holds_its_own_hand(model):
    """Every hand part hangs from its own side's forearm; nothing is shared."""
    for side in SIDES:
        forearm = model.body(f"{side}_forearm").id
        other = "left" if side == "right" else "right"
        for part in ("palm", "index_base", "thumb_metacarpal", "little_distal"):
            b = model.body(f"{side}_{part}").id
            while b not in (0, forearm):
                b = model.body_parentid[b]
            assert b == forearm, (side, part)
        assert model.body(f"{other}_palm").id != model.body(f"{side}_palm").id
    assert model.camera("right_wrist").id != model.camera("left_wrist").id


def test_hands_hang_from_the_elbows(model):
    """Each forearm bolts to J5's flange below OpenArm's elbow link; the hand points down the arm."""
    data = mujoco.MjData(model)
    set_q(model, data)
    for side, thumb_forward in (("right", True), ("left", True)):
        flange = data.site(f"{side}_forearm_flange").xpos
        elbow = data.xanchor[model.joint(f"{side}_elbow").id]
        assert np.allclose(flange[:2], elbow[:2], atol=1e-6)
        assert 0.09 < elbow[2] - flange[2] < 0.1  # OpenArm: J5 is 95.5 mm below the elbow
        palm = data.xmat[model.body(f"{side}_palm").id].reshape(3, 3)
        assert np.allclose(palm[:, 2], [0, 0, -1], atol=1e-6)  # fingers down
        thumb_side = palm[:, 0] if side == "right" else -palm[:, 0]  # the left thumb is at -X
        assert np.allclose(thumb_side, [1, 0, 0], atol=1e-6) and thumb_forward
        # The palm (its face is -Y of the hand model, on both hands) faces the body's middle:
        # +y for the right arm, -y for the left.
        assert np.allclose(-palm[:, 1], [0, 1 if side == "right" else -1, 0], atol=1e-6)
        assert (data.xpos[model.body(f"{side}_palm").id][1] < 0) == (side == "right")


def test_left_is_the_mirror_image_of_right(model):
    """The same angles on both sides give mirrored positions and orientations: the wrist point, the
    fingertips, the hand's axes (R_left = M R_right M_hand) and every strand length."""
    data = mujoco.MjData(model)
    rng = np.random.default_rng(2)
    right = {n: model.joint(f"right_{n}") for n in (*ARM_JOINTS, *V1.joint_names)}
    for _ in range(12):
        q = {n: rng.uniform(*model.jnt_range[j.id]) for n, j in right.items()}
        data.qpos[:] = model.qpos0
        for side in SIDES:
            for n, v in q.items():
                data.qpos[model.joint(f"{side}_{n}").qposadr[0]] = v
        mujoco.mj_forward(model, data)
        for site in (WRIST_SITE, "index_tip", "thumb_tip", "little_tip"):
            r, left = data.site(f"right_{site}").xpos, data.site(f"left_{site}").xpos
            assert np.allclose(left, MIRROR @ r, atol=1e-6), site
        rm = data.site_xmat[model.site(f"right_{WRIST_SITE}").id].reshape(3, 3)
        lm = data.site_xmat[model.site(f"left_{WRIST_SITE}").id].reshape(3, 3)
        assert np.allclose(lm, MIRROR @ rm @ HAND_MIRROR, atol=1e-6)
        for name in ("index_pip_flex", "thumb_cmc_rot_ext", "little_mcp_abd_flex"):
            assert data.ten_length[model.tendon(f"left_{name}").id] == pytest.approx(
                data.ten_length[model.tendon(f"right_{name}").id], abs=1e-6
            )


@pytest.mark.parametrize("side", SIDES)
@pytest.mark.parametrize(
    "joint, outward_sign",
    [("shoulder_pitch", None), ("shoulder_roll", True), ("elbow", None)],
)
def test_arm_signs(model, side, joint, outward_sign):
    """Pitch and elbow move the wrist forward (+x); roll moves it away from the body (out)."""
    data = mujoco.MjData(model)
    out = -1.0 if side == "right" else 1.0  # outward is -y for the right arm, +y for the left
    set_q(model, data, side, shoulder_roll=0.17)
    base = data.site(f"{side}_{WRIST_SITE}").xpos.copy()
    set_q(model, data, side, shoulder_roll=0.17 + (0.2 if joint == "shoulder_roll" else 0.0),
          **({joint: 0.2} if joint != "shoulder_roll" else {}))  # fmt: skip
    move = data.site(f"{side}_{WRIST_SITE}").xpos - base
    if outward_sign:
        assert move[1] * out > 0.02
    else:
        assert move[0] > 0.02


@pytest.mark.parametrize("side", SIDES)
def test_shoulder_yaw_turns_the_forearm_outward(model, side):
    data = mujoco.MjData(model)
    out = -1.0 if side == "right" else 1.0
    set_q(model, data, side, elbow=np.pi / 2)
    base = data.site(f"{side}_{WRIST_SITE}").xpos.copy()
    set_q(model, data, side, elbow=np.pi / 2, shoulder_yaw=0.3)
    assert (data.site(f"{side}_{WRIST_SITE}").xpos[1] - base[1]) * out > 0.03


@pytest.mark.parametrize("side", SIDES)
def test_pronation_turns_the_palm_down(model, side):
    data = mujoco.MjData(model)
    set_q(model, data, side, elbow=np.pi / 2, forearm_rot=np.pi / 2)
    palm = data.xmat[model.body(f"{side}_palm").id].reshape(3, 3)
    assert -palm[:, 1][2] < -0.99  # the palm's face (-Y of the hand model) looks down


@pytest.mark.parametrize("pose", list(POSES))
def test_poses_are_free_and_held(model, pose):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(pose).id)
    mujoco.mj_forward(model, data)
    assert data.ncon == 0
    qpos0 = data.qpos.copy()
    mujoco.mj_step(model, data, round(2.0 / model.opt.timestep))
    for side in SIDES:
        qadr, _ = arm_joint_ids(model, side)
        assert np.degrees(np.abs(data.qpos[qadr] - qpos0[qadr]).max()) < 2.0
    assert np.degrees(np.abs(data.qpos - qpos0).max()) < 2.0  # fingers too


def test_ready_pose_reaches_forward_on_both_sides(model):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("ready").id)
    mujoco.mj_forward(model, data)
    for side in SIDES:
        wrist = data.site(f"{side}_wrist_centre").xpos
        assert 0.2 < wrist[0] < 0.26 and 0.45 < wrist[2] < 0.5
    assert np.allclose(
        data.site("left_wrist_centre").xpos, MIRROR @ data.site("right_wrist_centre").xpos
    )


def test_arm_motion_does_not_pull_the_tendons(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lengths0 = data.ten_length.copy()
    rng = np.random.default_rng(0)
    for _ in range(20):
        data.qpos[:] = model.qpos0
        for side in SIDES:
            qadr, _ = arm_joint_ids(model, side)
            lo, hi = model.jnt_range[[model.joint(n).id for n in joint_names(side)]].T
            data.qpos[qadr] = rng.uniform(lo, hi)
        mujoco.mj_forward(model, data)
        assert np.abs(data.ten_length - lengths0).max() < 1e-9


@pytest.mark.parametrize("side", SIDES)
def test_ik_reaches_poses_on_both_sides(model, side):
    data = mujoco.MjData(model)
    set_q(model, data)
    ik = ArmIK(ik_model(side), joint_names(side), f"{side}_{WRIST_SITE}")
    rng = np.random.default_rng(1)
    for _ in range(6):
        q = rng.uniform(-0.3, 0.3, len(ARM_JOINTS)) + np.array([0, 0.3, 0, 1.2, 0, 0, 0])
        pos, rot = ik.pose(q)
        solved, err = ik.solve(q * 0 + ik.rest, pos, rot, iters=60)
        pos2, rot2 = ik.pose(solved)
        assert (
            err < 1e-3 and np.linalg.norm(pos2 - pos) < 1e-3 and np.linalg.norm(rot2 - rot) < 0.01
        )


def test_lite_model_has_the_same_physics(model):
    lite = load_arm_model(lite=True)
    assert lite.nq == model.nq and lite.nu == model.nu
    assert np.allclose(lite.body_mass, model.body_mass)
