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
    return env.record_scripted_demos(
        6, seed=0
    )  # cylinder, cube, ball with the right hand, then left


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


def test_left_hand_spawn_and_opposition_are_mirrored(env):
    """Regression (2026-10-03): the spawn point and contact points were kept in world
    coordinates but compared with the mirrored object, so a third of the left episodes failed at
    once ("pushed away") and the left hand never got opposition."""
    from tendra.scene import ARM_GRASP, scripted_grasp

    for seed in range(12):  # many spawns far from the mirror plane (|x| > 0.1)
        env.reset(seed=seed, side="left")
        _, _, term, _, info = env.step(np.zeros(env.action_dim))
        assert not term, info["terms"]
        assert np.allclose(env._observe()[1][-2:], 0.0, atol=0.01)  # obj xy - spawn xy
    for side in SIDES:
        env.reset(seed=1, obj="cylinder", side=side)
        scripted_grasp(env.scene, hold=0.3, **ARM_GRASP)
        env._contacts()
        assert env._opposition(env.scene.canon_pos(env.scene.object_pose()[0])), side


def _scripted_grasp_trace(env: GraspEnv, side: str, obj: str, a: np.ndarray) -> list[tuple]:
    """(reward terms, fail, actor obs, critic obs, gaps) at every control step of a scripted
    grasp; gaps = `_part_gaps`."""
    from tendra.scene import ARM_GRASP, scripted_grasp

    env.reset(seed=1, obj=obj, side=side)  # spawn 7 cm from the mirror plane
    scene, d, rows = env.scene, env.data, []
    original, next_t = scene.step, [d.time]

    def stepping(seconds: float) -> None:
        end = d.time + seconds - 1e-9
        while d.time < end:
            original(min(env.dt, end - d.time + 1e-9))
            if d.time >= next_t[0] - 1e-9:
                _, terms, fail = env._reward(a)
                env.prev_action = a
                rows.append((terms, fail, *env._observe(), _part_gaps(env)))
                next_t[0] = d.time + env.dt

    scene.step = stepping
    try:
        scripted_grasp(scene, hold=0.3, **ARM_GRASP)
    finally:
        del scene.step
    return rows


def _mirror_tolerances(env: GraspEnv) -> np.ndarray:
    """Per critic entry: how far the mirrored run may drift (contacts are chaotic: a finger
    touches a step earlier, a symmetric cylinder spins on its axis), far below what a mirror bug
    gives (a flipped coordinate is off by tens of cm, a flag in every step)."""
    k = env.syn.k
    tol = np.full(env.critic_dim, 0.02)  # positions (m), poses, one-hots, actions, time, height
    tol[0:32] = 0.3  # finger angles and targets (normalised): contact decides the grip
    tol[35:41] = 0.05  # wrist rot6d
    for start, n in ((41, 6), (54, 7), (91, 6)):  # wrist, arm and object velocities
        tol[start : start + n] = 0.6
    tol[47:54] = 0.05  # arm angles (normalised)
    tol[85:91] = 0.3  # object rot6d (a cylinder's yaw is free)
    # Which part touches, and how hard (needs the left hand's lite meshes to be exact mirror
    # images, `lite_model.lite_mesh`; before 2026-10-03 5 of 54 pairs differed).
    tol[97:103] = np.inf  # touch flags: checked on their own (>= 95% of steps equal)
    a = env.obs_dim
    tol[a : a + 7] = 4.0  # contact forces (N; 75th percentile measured <= 2.5 N)
    tol[a + 7 : a + 13] = 0.5  # privileged flags (on table, opposition, hold): a quarter of steps
    assert env.obs_dim == 103 + 3 + 3 + 2 + 6 + k + 16 + 2
    return tol


def _part_gaps(env: GraspEnv, distmax: float = 0.002) -> np.ndarray:
    """Per contact part (thumb..little, palm): the signed distance from its nearest geom to the
    active object (m, < 0 = penetrating; `distmax` if farther)."""
    obj = env.scene._obj_geom[env.scene.active_object]
    gaps = np.full(6, distmax)
    for g in np.flatnonzero((env._part >= 0) & (env._part < 6)):
        dist = mujoco.mj_geomDistance(env.model, env.data, int(g), obj, distmax, None)
        gaps[env._part[g]] = min(gaps[env._part[g]], dist)
    return gaps


@pytest.mark.parametrize("obj", ["cylinder", "cube", "ball"])
def test_mirrored_grasps_match_step_by_step(env, obj):
    """Guard for the whole class of mirror bugs (a quantity kept in world coordinates where the
    right hand's view was meant): the same scripted grasp with the right and the left hand, the
    same action, step by step: the same terminations, reward terms and observations, within
    what chaotic contacts explain. Touch flags are compared only where the part is clearly
    touching or clearly apart on both hands: a finger grazing the object (gap within 0.5 mm)
    touches on one side and just misses on the other from tiny physics differences, not from a
    mirror bug (the damped arm, 2026-10-06, left the cylinder grasp's index at -0.1 / +0.3 mm
    for 20 steps)."""
    env.configure(randomize=False, obs_noise=0.0)
    try:
        a = np.random.default_rng(0).uniform(-1, 1, env.action_dim)
        right, left = (_scripted_grasp_trace(env, side, obj, a) for side in SIDES)
    finally:
        env.configure(randomize=True, obs_noise=GraspEnvConfig().obs_noise)
    assert len(right) == len(left) > 60
    assert [r[1] for r in right] == [x[1] for x in left]  # terminations
    for key in set().union(*(r[0] for r in right + left)):
        diff = [abs(r[0].get(key, 0.0) - x[0].get(key, 0.0)) for r, x in zip(right, left)]
        assert np.percentile(diff, 90) <= 0.12, f"reward term {key!r} differs: {max(diff):.3f}"
    actor = np.array([np.abs(r[2] - x[2]) for r, x in zip(right, left)])
    critic = np.array([np.abs(r[3] - x[3]) for r, x in zip(right, left)])
    assert np.allclose(actor, critic[:, : env.obs_dim])  # no noise: the actor sees the clean obs
    grazing = np.array([(np.abs(r[4]) < 5e-4) | (np.abs(x[4]) < 5e-4)
                        for r, x in zip(right, left)])  # fmt: skip
    differs = (critic[:, 97:103] > 0.5) & ~grazing
    touch_differs = differs.mean(axis=0)  # per part (measured: <= 1 step)
    assert touch_differs.max() <= 0.05, f"touch flags differ: {touch_differs}"
    excess = np.percentile(critic, 75, axis=0) / _mirror_tolerances(env)
    worst = int(excess.argmax())
    assert excess.max() <= 1.0, f"critic entry {worst} differs: {critic[:, worst].max():.4f}"


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
    commanded = 0.0  # the travel the (low-pass filtered) wrist command asked for
    for _ in range(4):
        env.step(a)
        commanded += env.wrist_cmd[2] * env.config.wrist_speed * env.dt
    moved = env.scene.wrist_pose()[0] - start
    assert moved[2] > 0.5 * commanded and np.linalg.norm(moved[:2]) < 0.015


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


def _converged(env: GraspEnv, a: np.ndarray) -> None:
    """Put the finger targets (and the servos' ctrl) where action `a` points them."""
    k = env.syn.k
    q = np.clip(env.syn.posture(a[6 : 6 + k]) + env.config.residual_scale * a[6 + k :],
                V1.lower, V1.upper)  # fmt: skip
    env.finger_target = q.copy()
    env.scene.set_finger_targets(q)


def _substep_ctrl(env: GraspEnv, monkeypatch) -> list[np.ndarray]:
    """Record the active hand's finger ctrl at every physics step of the env."""
    import tendra.grasp_env as ge

    rows, step = [], ge.mujoco.mj_step

    def recording(m, d, *n):
        rows.append(d.ctrl[env._act].copy())
        step(m, d, *n)

    monkeypatch.setattr(ge.mujoco, "mj_step", recording)
    return rows


@pytest.mark.parametrize("side", SIDES)
def test_finger_ramp_spreads_a_target_step_over_the_substeps(env, side, monkeypatch):
    """The motion layer: an unchanged command keeps the servo targets still at every physics
    step; a new one is reached in equal small steps across the control step, not in one jump."""
    assert env.config.finger_ramp and env.config.wrist_ramp  # the defaults
    env.reset(seed=2, side=side)
    a = np.zeros(env.action_dim)
    _converged(env, a)
    rows = _substep_ctrl(env, monkeypatch)
    env.step(a)
    assert len(rows) == env.substeps
    assert np.ptp(np.array(rows), axis=0).max() == 0.0  # no change: the targets don't move

    rows.clear()
    before = env.finger_target.copy()
    b = a.copy()
    b[6] = 1.0  # "close"
    env.step(b)
    after = env.finger_target
    assert np.abs(after - before).max() > 0.05
    steps = np.diff(np.array([before, *rows]), axis=0)  # the change at each physics step
    assert np.allclose(steps, (after - before) / env.substeps, atol=1e-12)
    assert np.allclose(rows[-1], after)


def test_wrist_ramp_moves_the_target_per_ik_update(env, monkeypatch):
    env.reset(seed=2, side="right")
    s = env.scene
    seen, drive = [], s.drive_arms

    def recording() -> None:
        seen.append(s.wrist_target()[0])
        drive()

    monkeypatch.setattr(s, "drive_arms", recording)
    start = s.wrist_target()[0]
    a = np.zeros(env.action_dim)
    a[2] = 1.0
    env.wrist_cmd = np.array([0, 0, 1.0, 0, 0, 0])  # filter settled: a straight move up
    env.step(a)
    end = s.wrist_target()[0]
    assert len(seen) == len(env._chunks) == 2 and end[2] > start[2] + 0.005
    shares = [(p[2] - start[2]) / (end[2] - start[2]) for p in seen]
    expected = [(i + n) / env.substeps for i, n in env._chunks]
    assert np.allclose(shares, expected) and np.allclose(seen[-1], end)


def test_action_delay_holds_then_applies_old_actions(env):
    env.configure(action_delay=(1, 1))
    try:
        env.reset(seed=2, side="right")
        assert env.delay == 1
        start = env.finger_target.copy()
        a1, a2 = np.zeros(env.action_dim), np.zeros(env.action_dim)
        a1[6], a2[6] = 1.0, -1.0
        env.step(a1)
        assert np.array_equal(env.finger_target, start)  # nothing in effect yet: hold
        env.step(a2)  # now a1 takes effect
        k = env.syn.k
        q1 = np.clip(env.syn.posture(a1[6 : 6 + k]), V1.lower, V1.upper)
        assert np.allclose(env.finger_target, start + env.config.finger_smoothing * (q1 - start))
        assert np.array_equal(env.prev_action, a2)  # the policy sees its own last action
    finally:
        env.configure(action_delay=(0, 0))


def test_old_configs_keep_the_old_motion(env):
    old = config_from_dict({k: v for k, v in config_dict(GraspEnvConfig()).items()
                            if k not in ("finger_ramp", "wrist_ramp", "action_delay")})  # fmt: skip
    assert not old.finger_ramp and not old.wrist_ramp and old.action_delay == (0, 0)
