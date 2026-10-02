"""Tests for the Tendra arm: OpenArm's shoulder and elbow + Tendra forearm, wrist, V1 hand."""

import mujoco
import numpy as np
import pytest
from tendra.arm import ARM_JOINTS, POSES, arm_joint_ids, load_arm_model
from tendra.joints import V1


@pytest.fixture(scope="module")
def model():
    return load_arm_model()


def _wrist(model, data, **angles) -> np.ndarray:
    data.qpos[:] = model.qpos0
    for name, q in angles.items():
        data.qpos[model.joint(name).qposadr[0]] = q
    mujoco.mj_forward(model, data)
    return data.site("wrist_centre").xpos.copy()


def test_joints_and_actuators(model):
    assert [model.joint(i).name for i in range(len(ARM_JOINTS))] == list(ARM_JOINTS)
    assert model.nu == len(ARM_JOINTS) + V1.num_joints
    for name in (*ARM_JOINTS, *V1.joint_names):
        model.actuator(name)  # one actuator per arm joint and per hand servo, by name
    assert not any(model.body(i).name.startswith("openarm_left") for i in range(model.nbody))


def test_hand_hangs_from_the_openarm_elbow(model):
    """The forearm bolts to J5's flange below OpenArm's elbow link, the hand points down the arm."""
    data = mujoco.MjData(model)
    _wrist(model, data)
    flange = data.site("forearm_flange").xpos
    elbow = data.xanchor[model.joint("elbow").id]
    assert np.allclose(flange[:2], elbow[:2], atol=1e-6)
    assert 0.09 < elbow[2] - flange[2] < 0.1  # OpenArm: J5 is 95.5 mm below the elbow
    palm = data.xmat[model.body("palm").id].reshape(3, 3)
    assert np.allclose(palm[:, 2], [0, 0, -1], atol=1e-6)  # fingers down
    assert np.allclose(palm[:, 0], [1, 0, 0], atol=1e-6)  # thumb forward
    assert np.allclose(-palm[:, 1], [0, 1, 0], atol=1e-6)  # palm faces the body's middle


@pytest.mark.parametrize(
    "joint, direction",
    [
        ("shoulder_pitch", [1, 0, 0]),  # arm forward
        ("shoulder_roll", [0, -1, 0]),  # right arm out to the side (-y)
        ("elbow", [1, 0, 0]),  # forearm comes forward
    ],
)
def test_arm_signs(model, joint, direction):
    data = mujoco.MjData(model)
    base = _wrist(model, data, shoulder_roll=0.17)
    moved = _wrist(model, data, shoulder_roll=0.17 + (0.2 if joint == "shoulder_roll" else 0),
                   **({joint: 0.2} if joint != "shoulder_roll" else {}))  # fmt: skip
    assert np.dot(moved - base, direction) > 0.02


def test_shoulder_yaw_turns_the_forearm_outward(model):
    data = mujoco.MjData(model)
    base = _wrist(model, data, elbow=np.pi / 2)
    moved = _wrist(model, data, elbow=np.pi / 2, shoulder_yaw=0.3)
    assert moved[1] - base[1] < -0.03  # right arm: outward = -y


def test_pronation_turns_the_palm_down(model):
    data = mujoco.MjData(model)
    _wrist(model, data, elbow=np.pi / 2, forearm_rot=np.pi / 2)
    palm_normal = -data.xmat[model.body("palm").id].reshape(3, 3)[:, 1]
    assert palm_normal[2] < -0.99


@pytest.mark.parametrize("pose", list(POSES))
def test_poses_are_free_and_held(model, pose):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(pose).id)
    mujoco.mj_forward(model, data)
    assert data.ncon == 0
    qpos0 = data.qpos.copy()
    mujoco.mj_step(model, data, round(2.0 / model.opt.timestep))
    qadr, _ = arm_joint_ids(model)
    assert np.degrees(np.abs(data.qpos[qadr] - qpos0[qadr]).max()) < 2.0
    assert np.degrees(np.abs(data.qpos - qpos0).max()) < 2.0  # fingers too


def test_ready_pose_reaches_forward(model):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("ready").id)
    mujoco.mj_forward(model, data)
    wrist = data.site("wrist_centre").xpos
    assert 0.2 < wrist[0] < 0.26 and 0.45 < wrist[2] < 0.5


def test_arm_motion_does_not_pull_the_tendons(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lengths0 = data.ten_length.copy()
    qadr, _ = arm_joint_ids(model)
    rng = np.random.default_rng(0)
    lo, hi = model.jnt_range[[model.joint(n).id for n in ARM_JOINTS]].T
    for _ in range(20):
        data.qpos[:] = model.qpos0
        data.qpos[qadr] = rng.uniform(lo, hi)
        mujoco.mj_forward(model, data)
        assert np.abs(data.ten_length - lengths0).max() < 1e-9


def test_lite_model_has_the_same_physics(model):
    lite = load_arm_model(lite=True)
    assert lite.nq == model.nq and lite.nu == model.nu
    assert np.allclose(lite.body_mass, model.body_mass)
