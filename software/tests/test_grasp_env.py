"""The grasp RL environment (tendra.grasp_env): the two-arm robot, one hand per episode.

One env per module (building takes ~5 s).
"""

import mujoco
import numpy as np
import pytest
from tendra.arm import SIDES
from tendra.grasp_env import GraspEnv, GraspEnvConfig, config_dict, config_from_dict
from tendra.joints import V1

pytest.importorskip("fast_simplification")


@pytest.fixture(scope="module")
def env():
    return GraspEnv(GraspEnvConfig(), seed=0)


@pytest.fixture(scope="module")
def demos(env):
    return env.record_scripted_demos(6, seed=0)  # cylinder, cube, ball with the right hand, then left


def hold_action(env: GraspEnv) -> np.ndarray:
    a = np.zeros(env.action_dim)
    a[6] = 1.0  # "close" synergy
    return a


def test_dimensions_and_finite_obs(env):
    obs, critic = env.reset(seed=1)
    assert env.action_dim == 6 + env.syn.k + V1.num_joints
    assert obs.shape == (env.obs_dim,) and critic.shape == (env.critic_dim,)
    assert critic.size > obs.size  # privileged extras
    rng = np.random.default_rng(0)
    for _ in range(20):
        (obs, critic), r, _, _, info = env.step(rng.uniform(-2, 2, env.action_dim))
        assert np.all(np.isfinite(obs)) and np.all(np.isfinite(critic)) and np.isfinite(r)
        assert set(info["terms"]) >= {"reach", "contact", "action_rate", "effort", "arm_limits"}


@pytest.mark.parametrize("side", SIDES)
def test_random_actions_stay_finite_and_inside_the_arm_limits(env, side):
    env.reset(seed=1, side=side)
    assert env.side == side
    rng = np.random.default_rng(0)
    lo, hi = env._arm_range
    for _ in range(40):
        (obs, _), r, term, trunc, _ = env.step(rng.uniform(-1, 1, env.action_dim))
        assert np.all(np.isfinite(obs)) and np.isfinite(r)
        q = env.scene.arm_positions()
        assert np.all(q >= lo - 0.01) and np.all(q <= hi + 0.01)  # limits are soft
        if term or trunc:
            env.reset(side=side)


def test_hands_option_picks_the_side(env):
    sides = set()
    for seed in range(12):
        env.reset(seed=seed)  # hands = "any"
        sides.add(env.side)
    assert sides == set(SIDES)
    env.configure(hands="left")
    try:
        for seed in range(3):
            env.reset(seed=seed)
            assert env.side == "left" and env.scene.side == "left"
    finally:
        env.configure(hands="any")
    with pytest.raises(ValueError):
        GraspEnv(GraspEnvConfig(hands="both"))


def test_reset_is_deterministic(env):
    a = env.reset(seed=7, side="right")[1]
    b = env.reset(seed=7, side="right")[1]
    c = env.reset(seed=8, side="right")[1]
    assert np.array_equal(a[:-10], b[:-10]) and not np.array_equal(a, c)


def test_one_policy_sees_both_hands_the_same_way(env):
    """The left episode, mirrored, is the right episode: same seed, same canonical observation
    (the object is on the other side of the table, the hand is the mirror image, the policy
    can't tell). This is what lets one network drive both hands."""
    right_obs, right_critic = env.reset(seed=3, side="right", obj="cube")
    left_obs, left_critic = env.reset(seed=3, side="left", obj="cube")
    # Without the last few entries (time and height, which settle a hair differently).
    assert np.allclose(left_obs[:-2], right_obs[:-2], atol=2e-3)
    assert np.allclose(left_critic[:-8], right_critic[:-8], atol=2e-3)


def test_the_same_actions_do_the_same_on_both_hands(env):
    """Mirrored world, same action in the policy's view: the observation and reward agree."""
    rng = np.random.default_rng(4)
    actions = [rng.uniform(-1, 1, env.action_dim) for _ in range(12)]
    runs = {}
    for side in SIDES:
        env.reset(seed=5, side=side, obj="cylinder")
        obs_list, rewards = [], []
        for a in actions:
            (obs, _), r, term, trunc, _ = env.step(a)
            obs_list.append(obs)
            rewards.append(r)
            if term or trunc:
                break
        runs[side] = (np.array(obs_list), np.array(rewards))
    n = min(len(runs["right"][1]), len(runs["left"][1]))
    assert n >= 8
    assert np.allclose(runs["right"][0][:n, :-2], runs["left"][0][:n, :-2], atol=0.05)
    assert np.allclose(runs["right"][1][:n], runs["left"][1][:n], atol=0.15)


@pytest.mark.parametrize("side", SIDES)
def test_wrist_moves_as_commanded_through_the_arm(env, side):
    env.reset(seed=2, side=side)
    start, _ = env.scene.wrist_pose()
    a = np.zeros(env.action_dim)
    a[2] = 1.0  # up at wrist_speed
    for _ in range(4):
        env.step(a)
    moved = env.scene.wrist_pose()[0] - start
    expected = 4 * env.dt * env.config.wrist_speed
    assert moved[2] > 0.5 * expected and np.linalg.norm(moved[:2]) < 0.015


def test_sideways_action_is_mirrored_for_the_left_hand(env):
    """+x in the policy's view moves the right hand toward +x and the left hand toward -x."""
    moves = {}
    for side in SIDES:
        env.reset(seed=2, side=side)
        start, _ = env.scene.wrist_pose()
        a = np.zeros(env.action_dim)
        a[0] = 1.0
        for _ in range(4):
            env.step(a)
        moves[side] = env.scene.wrist_pose()[0] - start
    assert moves["right"][0] > 0.01 and moves["left"][0] < -0.01
    assert moves["left"][0] == pytest.approx(-moves["right"][0], abs=0.01)


@pytest.mark.parametrize("side", SIDES)
def test_target_cannot_run_away_from_the_wrist(env, side):
    """Position and orientation: the target stays within the lead of where the arm really is."""
    env.reset(seed=2, side=side)
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


@pytest.mark.parametrize("side", SIDES)
def test_workspace_box_is_in_the_policys_view(env, side):
    env.reset(seed=2, side=side)
    assert np.allclose(env._ws_lo, env._home + env.config.workspace_lo)
    a = np.zeros(env.action_dim)
    a[0] = 1.0  # toward the middle, far
    for _ in range(80):
        env.step(a)
    canon = env.scene.canon_pos(env.scene.wrist_target()[0])
    assert canon[0] <= env._ws_hi[0] + 1e-9


def test_scripted_demos_and_demo_resets(env, demos):
    assert len(demos) >= 5 and all(len(d) > 50 for d in demos)  # of 6: cube grasps can slip
    assert {d.side for d in demos} == set(SIDES)
    assert demos[0].qpos.shape[1] == env.model.nq  # includes both arms
    env.set_demos(demos)
    env.configure(demo_prob=1.0)
    try:
        for want in SIDES:
            env.reset(seed=3, side=want)
            assert env.from_demo and env.side == want  # a demo of that hand
        demo = next(d for d in demos if d.side == "left")
        env.demos = [type(demo)(demo.obj, demo.rest_z, *(x[-1:] for x in (
            demo.qpos, demo.qvel, demo.ctrl, demo.mocap_pos, demo.mocap_quat)), side=demo.side)]  # fmt: skip
        env.reset(seed=4)
        assert env.side == "left"
        bonus = 0
        for _ in range(env.hold_steps + 5):
            _, _, term, _, info = env.step(hold_action(env))
            bonus += "success" in info["terms"]
            assert not term
        assert env.success and bonus == 1
    finally:
        env.configure(demo_prob=0.0)
        env.set_demos([])


@pytest.mark.parametrize("side", SIDES)
def test_reward_prefers_a_grasp_over_idling(env, demos, side):
    demo = next(d for d in demos if d.side == side)
    flex = env.scene.sides[side].act[V1.joint_index("index_mcp_flex")]  # arm actuators come first
    k = int(np.argmax(demo.ctrl[:, flex] > 0.5))
    returns = []
    for close in (1.0, -1.0):
        env.demos = [type(demo)(demo.obj, demo.rest_z, *(x[k : k + 1] for x in (
            demo.qpos, demo.qvel, demo.ctrl, demo.mocap_pos, demo.mocap_quat)), side=demo.side)]  # fmt: skip
        env.configure(demo_prob=1.0)
        env.reset(seed=5)
        a = np.zeros(env.action_dim)
        a[6] = close
        returns.append(sum(env.step(a)[1] for _ in range(15)))
    env.configure(demo_prob=0.0)
    env.set_demos([])
    assert returns[0] > returns[1] + 5


def test_blow_up_guard(env):
    """A physics explosion (object flying off) ends the episode as a failure, and no lift
    reward or max height from it counts."""
    env.reset(seed=6, obj="cube")
    dof = env.scene._obj_dof["cube"]
    env.data.qvel[dof + 2] = 50.0  # launch it
    _, r, term, _, info = env.step(np.zeros(env.action_dim))
    assert term and r == -env.config.rewards.fail and info["terms"] == {"fail": r}
    assert info["episode"]["unstable"] and info["episode"]["max_height"] < 0.4


def test_arm_limit_penalty_only_near_a_limit(env):
    env.reset(seed=1, side="right")
    a = np.zeros(env.action_dim)
    assert env._reward(a)[1]["arm_limits"] == 0.0  # the start pose is well inside every range
    arm = env.scene.arm
    env.data.qpos[arm.arm_qadr[3]] = env.scene.arm.ik.hi[3] - 0.02  # elbow at its limit
    mujoco.mj_forward(env.model, env.data)
    assert env._reward(a)[1]["arm_limits"] < 0.0


def test_the_resting_hand_is_not_the_active_one(env):
    """Touch, force and reward only look at the hand that works: the other hand has its own
    geoms, which the active hand's contact groups must not include."""
    env.reset(seed=1, side="right")
    right_geoms = set(np.flatnonzero(env._part >= 0))
    env.reset(seed=1, side="left")
    left_geoms = set(np.flatnonzero(env._part >= 0))
    assert right_geoms and left_geoms and not (right_geoms & left_geoms)


def test_config_roundtrip():
    cfg = GraspEnvConfig(objects=("cube",), grasp_type="precision", hands="left")
    cfg.rewards.lift = 7.0
    back = config_from_dict(config_dict(cfg))
    assert back == cfg and back.objects == ("cube",) and isinstance(back.workspace_lo, tuple)


def test_bad_config():
    with pytest.raises(ValueError):
        GraspEnv(GraspEnvConfig(grasp_type="wave"))
