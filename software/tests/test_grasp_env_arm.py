"""The grasp RL environment with the Tendra arm (GraspEnvConfig(arm=True)). One env per module."""

import mujoco
import numpy as np
import pytest
from tendra.grasp_env import GraspEnv, GraspEnvConfig, config_dict, config_from_dict
from tendra.joints import V1

pytest.importorskip("fast_simplification")


@pytest.fixture(scope="module")
def env():
    return GraspEnv(GraspEnvConfig(arm=True), seed=0)


@pytest.fixture(scope="module")
def demos(env):
    return env.record_scripted_demos(3, seed=0)


def hold_action(env: GraspEnv) -> np.ndarray:
    a = np.zeros(env.action_dim)
    a[6] = 1.0  # "close" synergy
    return a


def test_same_action_more_observation(env):
    plain = GraspEnv(GraspEnvConfig(), seed=0)
    assert env.action_dim == plain.action_dim == 6 + env.syn.k + V1.num_joints
    assert env.obs_dim == plain.obs_dim + 14  # + the arm's 7 joint angles and 7 speeds
    assert env.critic_dim == plain.critic_dim + 14
    obs, critic = env.reset(seed=1)
    assert obs.shape == (env.obs_dim,) and np.all(np.isfinite(obs)) and np.all(np.isfinite(critic))


def test_random_actions_stay_finite_and_inside_the_arm_limits(env):
    env.reset(seed=1)
    rng = np.random.default_rng(0)
    lo, hi = env._arm_range
    for _ in range(60):
        (obs, _), r, term, trunc, info = env.step(rng.uniform(-1, 1, env.action_dim))
        assert np.all(np.isfinite(obs)) and np.isfinite(r)
        assert "arm_limits" in info["terms"]
        q = env.scene.arm_positions()
        assert np.all(q >= lo - 0.01) and np.all(q <= hi + 0.01)  # limits are soft
        if term or trunc:
            env.reset()


def test_reset_is_deterministic(env):
    a = env.reset(seed=7)[1]
    b = env.reset(seed=7)[1]
    c = env.reset(seed=8)[1]
    assert np.array_equal(a[:-10], b[:-10]) and not np.array_equal(a, c)


def test_wrist_moves_as_commanded_through_the_arm(env):
    env.reset(seed=2)
    start, _ = env.scene.wrist_pose()
    a = np.zeros(env.action_dim)
    a[2] = 1.0  # up at wrist_speed
    for _ in range(4):
        env.step(a)
    moved = env.scene.wrist_pose()[0] - start
    expected = 4 * env.dt * env.config.wrist_speed
    assert moved[2] > 0.5 * expected and np.linalg.norm(moved[:2]) < 0.015


def test_target_cannot_run_away_from_the_wrist(env):
    """Position and orientation: the target stays within the lead of where the arm really is."""
    env.reset(seed=2)
    cfg = env.config
    a = np.zeros(env.action_dim)
    a[1] = -1.0  # forward (-y), then turn about the vertical axis, at full speed, for 6 s
    a[5] = 1.0
    for _ in range(120):
        env.step(a)
    pos, quat = env.scene.wrist_pose()
    tpos, tquat = env.scene.wrist_target()
    assert np.linalg.norm(tpos - pos) <= cfg.wrist_lead + 0.03  # the arm lags a little
    angle = 2 * np.arccos(np.clip(abs(np.dot(quat, tquat)), 0, 1))
    assert angle <= cfg.wrist_turn_lead + 0.2
    assert env.scene._ik.lo[0] <= env.scene.arm_positions()[0] <= env.scene._ik.hi[0]


def test_arm_workspace_is_the_arm_box(env):
    env.reset(seed=2)
    assert np.allclose(env._ws_lo, env.scene.home_pos + env.config.arm_workspace_lo)
    a = np.zeros(env.action_dim)
    a[0] = 1.0  # sideways, far
    for _ in range(80):
        env.step(a)
    assert env.scene.wrist_target()[0][0] <= env._ws_hi[0] + 1e-9


def test_scripted_demos_and_demo_resets(env, demos):
    assert len(demos) == 3 and all(len(d) > 50 for d in demos)  # cylinder, cube and ball
    assert demos[0].qpos.shape[1] == env.model.nq  # includes the arm joints
    env.set_demos(demos)
    env.configure(demo_prob=1.0)
    try:
        env.reset(seed=3)
        assert env.from_demo
        demo = demos[0]
        env.demos = [type(demo)(demo.obj, demo.rest_z, *(x[-1:] for x in (
            demo.qpos, demo.qvel, demo.ctrl, demo.mocap_pos, demo.mocap_quat)))]  # fmt: skip
        env.reset(seed=4)
        bonus = 0
        for _ in range(env.hold_steps + 5):
            _, _, term, _, info = env.step(hold_action(env))
            bonus += "success" in info["terms"]
            assert not term
        assert env.success and bonus == 1
    finally:
        env.configure(demo_prob=0.0)
        env.set_demos([])


def test_reward_prefers_a_grasp_over_idling(env, demos):
    demo = demos[0]
    flex = env.scene._act[V1.joint_index("index_mcp_flex")]  # the arm's actuators come first
    k = int(np.argmax(demo.ctrl[:, flex] > 0.5))
    returns = []
    for close in (1.0, -1.0):
        env.demos = [type(demo)(demo.obj, demo.rest_z, *(x[k : k + 1] for x in (
            demo.qpos, demo.qvel, demo.ctrl, demo.mocap_pos, demo.mocap_quat)))]  # fmt: skip
        env.configure(demo_prob=1.0)
        env.reset(seed=5)
        a = np.zeros(env.action_dim)
        a[6] = close
        returns.append(sum(env.step(a)[1] for _ in range(15)))
    env.configure(demo_prob=0.0)
    env.set_demos([])
    assert returns[0] > returns[1] + 5


def test_arm_limit_penalty_only_near_a_limit(env):
    env.reset(seed=1)
    a = np.zeros(env.action_dim)
    assert env._reward(a)[1]["arm_limits"] == 0.0  # the start pose is well inside every range
    env.data.qpos[env.scene._arm_qadr[3]] = env.scene._ik.hi[3] - 0.02  # elbow at its limit
    mujoco.mj_forward(env.model, env.data)
    assert env._reward(a)[1]["arm_limits"] < 0.0


def test_config_roundtrip_and_mismatch():
    cfg = GraspEnvConfig(arm=True, wrist_turn_lead=0.2)
    back = config_from_dict(config_dict(cfg))
    assert back == cfg and back.arm and isinstance(back.arm_workspace_lo, tuple)
    from tendra.scene import GraspScene, SceneConfig

    with pytest.raises(ValueError):
        GraspEnv(GraspEnvConfig(arm=True), scene=GraspScene(SceneConfig(arm=False)))
