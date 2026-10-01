"""The grasp RL environment (tendra.grasp_env). One env per module (building takes ~1.5 s)."""

import numpy as np
import pytest
from tendra.grasp_env import GraspEnv, GraspEnvConfig, config_dict, config_from_dict

pytest.importorskip("fast_simplification")


@pytest.fixture(scope="module")
def env():
    return GraspEnv(GraspEnvConfig(), seed=0)


@pytest.fixture(scope="module")
def demos(env):
    return env.record_scripted_demos(3, seed=0)


def hold_action(env: GraspEnv) -> np.ndarray:
    """Keep the wrist still and the hand fully closed (the scripted grasp posture)."""
    a = np.zeros(env.action_dim)
    a[6] = 1.0  # "close" synergy
    return a


def test_dimensions_and_finite_obs(env):
    obs, critic = env.reset(seed=1)
    assert env.action_dim == 6 + env.syn.k + 20
    assert obs.shape == (env.obs_dim,) and critic.shape == (env.critic_dim,)
    assert critic.size > obs.size  # privileged extras
    rng = np.random.default_rng(0)
    for _ in range(20):
        (obs, critic), r, _, _, info = env.step(rng.uniform(-2, 2, env.action_dim))
        assert np.all(np.isfinite(obs)) and np.all(np.isfinite(critic)) and np.isfinite(r)
        assert set(info["terms"]) >= {"reach", "contact", "action_rate", "effort"}


def test_reset_is_deterministic(env):
    a = env.reset(seed=7)[1]
    b = env.reset(seed=7)[1]
    c = env.reset(seed=8)[1]
    assert np.array_equal(a[:-10], b[:-10]) and not np.array_equal(a, c)


def test_wrist_moves_as_commanded_and_stays_in_workspace(env):
    env.reset(seed=2)
    start, _ = env.scene.wrist_pose()
    a = np.zeros(env.action_dim)
    a[2] = 1.0  # up at wrist_speed
    for _ in range(4):
        env.step(a)
    moved = env.scene.wrist_pose()[0] - start
    expected = 4 * env.dt * env.config.wrist_speed
    assert moved[2] > 0.6 * expected and np.linalg.norm(moved[:2]) < 0.01
    for _ in range(60):  # keep going up: stops at the workspace top
        env.step(a)
    top = env.scene.home_pos[2] + env.config.workspace_hi[2]
    assert env.scene.wrist_target()[0][2] <= top + 1e-9


def test_randomisation_changes_physics_and_nominal_restores(env):
    obj = env.scene._obj_body["cylinder"]
    nominal = env._nominal["body_mass"][obj]
    masses = set()
    for seed in range(4):
        env.reset(seed=seed, obj="cylinder")
        masses.add(round(float(env.model.body_mass[obj]), 6))
    assert len(masses) > 1
    env.configure(randomize=False)
    try:
        env.reset(seed=0, obj="cylinder")
        assert env.model.body_mass[obj] == nominal
    finally:
        env.configure(randomize=True)


def test_scripted_demos_and_demo_resets(env, demos):
    assert len(demos) >= 2 and all(len(d) > 50 for d in demos)
    env.set_demos(demos)
    env.configure(demo_prob=1.0)
    try:
        env.reset(seed=3)
        assert env.from_demo
        # From the demo's last state (object up, in the closed hand), holding on succeeds.
        demo = demos[0]
        env.demos = [type(demo)(demo.obj, demo.rest_z, *(x[-1:] for x in (
            demo.qpos, demo.qvel, demo.ctrl, demo.mocap_pos, demo.mocap_quat)))]  # fmt: skip
        env.reset(seed=4)
        total, bonus = 0.0, 0
        for _ in range(env.hold_steps + 5):
            _, r, term, _, info = env.step(hold_action(env))
            total += r
            bonus += "success" in info["terms"]
            assert not term  # success doesn't end the episode: holding on keeps earning
        assert info["terms"]["held"] > 0 and info["terms"].get("opposition", 0) > 0
        assert env.success and bonus == 1  # the bonus is paid once
        assert total > env.config.rewards.success
    finally:
        env.configure(demo_prob=0.0)
        env.set_demos([])


def test_reward_prefers_a_grasp_over_idling(env, demos):
    """At the scripted pre-grasp (hand around the object), closing earns more than opening."""
    demo = demos[0]
    k = int(np.argmax(demo.ctrl[:, 2] > 0.5))  # first frame where the fingers close
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


def test_blow_up_guard(env):
    """A physics explosion (object flying off) ends the episode as a failure, and no lift
    reward or max height from it counts."""
    env.reset(seed=6, obj="cube")
    dof = env.scene._obj_dof["cube"]
    env.data.qvel[dof + 2] = 50.0  # launch it
    _, r, term, _, info = env.step(np.zeros(env.action_dim))
    assert term and r == -env.config.rewards.fail and info["terms"] == {"fail": r}
    assert info["episode"]["unstable"] and info["episode"]["max_height"] < 0.4


def test_config_roundtrip():
    cfg = GraspEnvConfig(objects=("cube",), grasp_type="precision")
    cfg.rewards.lift = 7.0
    back = config_from_dict(config_dict(cfg))
    assert back == cfg and back.objects == ("cube",)


def test_bad_config():
    with pytest.raises(ValueError):
        GraspEnv(GraspEnvConfig(grasp_type="wave"))
