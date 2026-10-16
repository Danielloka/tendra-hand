"""The GPU grasp env (`tendra.gpu`: JAX + MuJoCo Warp) checked against the CPU env, on the CPU.

    uv run --group gpu pytest -m gpu software/tests/test_gpu_env.py                  # ~45 min
    uv run --group gpu pytest -m gpu software/tests/test_gpu_env.py -k "not smoke"   # ~10 min

Only with `-m gpu` (the normal suite deselects them) and the `gpu` dependency group. Slow: the
bundle export records scripted grasps, Warp compiles its kernels on first use, and the training
smoke run compiles Brax PPO on the CPU (~35 min).
"""

import subprocess
import sys
from collections import namedtuple
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("fast_simplification")
jax = pytest.importorskip("jax")
pytest.importorskip("mujoco_warp")
pytest.importorskip("mujoco_playground")

import jax.numpy as jp
from tendra.arm import SIDES
from tendra.gpu import grasp_env_jax as G
from tendra.gpu.export import export_bundle
from tendra.grasp_env import OBJECT_KINDS, GraspEnv, GraspEnvConfig
from tendra.scene import ARM_GRASP, GraspScene, SceneConfig, scripted_grasp

pytestmark = pytest.mark.gpu
N_WORLDS = 4

# The data fields the JAX env reads, filled from a CPU MjData (shapes as in mjx.Data).
CpuData = namedtuple("CpuData", "qpos qvel ctrl xpos xquat site_xpos site_xmat sensordata "
                     "actuator_force cvel subtree_com mocap_pos mocap_quat")  # fmt: skip


def cpu_data(d) -> CpuData:
    def f(x):
        return jp.asarray(np.array(x))

    return CpuData(f(d.qpos), f(d.qvel), f(d.ctrl), f(d.xpos), f(d.xquat), f(d.site_xpos),
                   f(d.site_xmat.reshape(-1, 3, 3)), f(d.sensordata), f(d.actuator_force),
                   f(d.cvel), f(d.subtree_com), f(d.mocap_pos), f(d.mocap_quat))  # fmt: skip


@pytest.fixture(scope="module")
def bundle(tmp_path_factory) -> Path:
    return export_bundle(tmp_path_factory.mktemp("gpu") / "tendra_gpu.npz", demos_per_object=1)


@pytest.fixture(scope="module")
def cpu():
    """A CPU env with the same scene (contact sensors, every object) as the bundle."""
    scene = GraspScene(SceneConfig(objects=OBJECT_KINDS, contact_sensors=True))
    return GraspEnv(GraspEnvConfig(randomize=False, obs_noise=0.0), seed=0, scene=scene)


@pytest.fixture(scope="module")
def env(bundle):
    return G.TendraGrasp(bundle, config_overrides={"naconmax": 96 * N_WORLDS, "obs_noise": 0.0})


@pytest.fixture(scope="module")
def fns(env):
    """Jitted pieces of the env (compiled once per module)."""
    return {
        "sense": jax.jit(env.sense),
        "reward": jax.jit(env.reward),
        "observe": jax.jit(env.observe),
        # action -> wrist target, through the wrist command filter (as in `step`)
        "wrist": jax.jit(
            lambda data, info, a: env.wrist_command(data, info, env.wrist_filter(info, a))
        ),
        "fingers": jax.jit(env.finger_command),
        "reset": jax.jit(jax.vmap(env.reset)),
        "step": jax.jit(jax.vmap(env.step)),
    }


def test_bundle(env, cpu):
    meta = env.meta
    assert meta["version"] == G.BUNDLE_VERSION and meta["sides"] == list(SIDES)
    b = env._np
    assert b["act"].shape == (2, 16) and b["arm_act"].shape == (2, 7)
    assert b["chain_axis"].shape == (2, 7, 3) and b["sensor_part"].shape == (2, 3, 7)
    assert (b["demo_count"] > 0).all(), "a scripted demo per side and object"
    assert (b["near_count"] > 5).all(), "and its approach, for near starts"
    import mujoco

    assert not env.mj_model.opt.enableflags & int(mujoco.mjtEnableBit.mjENBL_SLEEP)
    state = jax.jit(env.reset)(jax.random.PRNGKey(0))
    assert state.obs["state"].shape == (cpu.obs_dim,) == (141,)
    assert state.obs["privileged_state"].shape == (cpu.critic_dim,) == (156,)


@pytest.mark.parametrize("side", SIDES)
def test_fk_and_ik_match_arm_ik(env, cpu, side):
    """Forward kinematics of the exported chain = ArmIK.pose; the JAX IK = ArmIK.solve (6
    iterations, as the scene drives it) on reachable and unreachable targets."""
    i = SIDES.index(side)
    ik = cpu.scene.sides[side].ik
    chain = env._side_chain(jp.array(i))
    rng = np.random.default_rng(i)
    fk = jax.jit(lambda q: G.arm_fk(chain, q)[:2])
    solve = jax.jit(lambda q, p, r: env.solve_ik(jp.array(i), q, p, r))
    worst_fk, worst_q, worst_err = 0.0, 0.0, 0.0
    for trial in range(30):
        q = np.clip(ik.rest + rng.normal(0, 0.3, 7), ik.lo, ik.hi)
        pos, rot = ik.pose(q)
        jpos, jrot = fk(jp.asarray(q, dtype=jp.float32))
        worst_fk = max(worst_fk, float(np.abs(np.asarray(jpos) - pos).max()),
                       float(np.abs(np.asarray(jrot) - rot).max()))  # fmt: skip
        # Target: a pose near the start (reachable), or 0.3 m beyond it (unreachable).
        start = np.clip(q + rng.normal(0, 0.1, 7), ik.lo, ik.hi)
        target = pos + (0.0 if trial % 3 else 0.3) * rng.normal(0, 1, 3)
        q_cpu, err_cpu = ik.solve(start, target, rot, iters=6)
        q_jax, err_jax = solve(jp.asarray(start, jp.float32), jp.asarray(target, jp.float32),
                               jp.asarray(rot, jp.float32))  # fmt: skip
        worst_q = max(worst_q, float(np.abs(np.asarray(q_jax) - q_cpu).max()))
        worst_err = max(worst_err, abs(float(err_jax) - err_cpu))
    print(f"{side}: FK max diff {worst_fk:.2e}, IK max joint diff {worst_q:.2e} rad, "
          f"error diff {worst_err:.2e}")  # fmt: skip
    assert worst_fk < 2e-5
    assert worst_q < 2e-3 and worst_err < 1e-3  # float32 vs float64


def test_canon_matches_the_scene_mirror(cpu):
    scene, rng = cpu.scene, np.random.default_rng(0)
    mx = jp.asarray(scene.mirror_x)
    for side in SIDES:
        scene.reset(rng, side=side)
        flip = jp.array(side == "left")
        for _ in range(5):
            p, v = rng.normal(0, 0.3, 3), rng.normal(0, 1, 3)
            r = np.linalg.qr(rng.normal(size=(3, 3)))[0]
            r *= np.linalg.det(r)  # a rotation
            assert np.allclose(G.canon_pos(jp.asarray(p), flip, mx), scene.canon_pos(p), atol=1e-6)
            assert np.allclose(G.canon_vec(jp.asarray(v), flip), scene.canon_vec(v), atol=1e-6)
            assert np.allclose(G.canon_axial(jp.asarray(v), flip), scene.canon_axial(v), atol=1e-6)
            assert np.allclose(G.canon_rot(jp.asarray(r), flip), scene.canon_rot(r), atol=1e-6)
            twice = G.canon_pos(G.canon_pos(jp.asarray(p), flip, mx), flip, mx)
            assert np.allclose(twice, p, atol=1e-6)  # the mirror is its own inverse
            assert np.allclose(G.canon_rot(G.canon_rot(jp.asarray(r), flip), flip), r, atol=1e-6)
            q = G.mat_to_quat(jp.asarray(r))
            assert np.allclose(G.quat_to_mat(q), r, atol=1e-5)


@pytest.mark.parametrize("side", SIDES)
def test_obs_reward_and_commands_match_the_cpu_env(env, cpu, fns, side):
    """Along scripted grasps (approach, touch, lift, hold), the JAX sensing, reward and
    observation on the CPU's state equal the CPU env's, and so do the wrist and finger commands
    of a random action. Known differences: contact forces (|net| vs sum of normal forces),
    opposition (contact centres per part vs every contact point), and the CPU's object-on-table
    flag (false while the resting object sleeps; Warp has no sleeping)."""
    scene, d = cpu.scene, cpu.data
    rng = np.random.default_rng(1)
    i_force = slice(cpu.obs_dim, cpu.obs_dim + 7)
    i_table = cpu.obs_dim + 7 + 2  # obj_on_table in the critic
    stats = {"states": 0, "actor": 0.0, "critic": 0.0, "reward": 0.0, "opp_differs": 0,
             "touch_differs": 0, "wrist": 0.0, "fingers": 0.0}  # fmt: skip

    def check() -> None:
        cpu.finger_target = scene.finger_targets()
        cpu.prev_action = rng.uniform(-1, 1, cpu.action_dim)
        cpu.prev2_action = rng.uniform(-1, 1, cpu.action_dim)
        cpu.wrist_cmd = rng.uniform(-1, 1, 6)
        cpu.t = 7
        a = rng.uniform(-1, 1, cpu.action_dim)
        info = {
            "side": jp.array(SIDES.index(side)), "flip": jp.array(side == "left"),
            "obj": jp.array(OBJECT_KINDS.index(scene.active_object)),
            "rest_z": jp.array(scene.rest_height), "spawn_xy": jp.asarray(cpu.spawn_xy),
            "finger_target": jp.asarray(cpu.finger_target),
            "prev_action": jp.asarray(cpu.prev_action), "hold": jp.array(cpu.hold),
            "prev2_action": jp.asarray(cpu.prev2_action), "wrist_cmd": jp.asarray(cpu.wrist_cmd),
            "success": jp.array(float(cpu.success)), "t": jp.array(cpu.t),
            "obs_bias": jp.zeros(3), "mass_scale": jp.array(1.0), "rng": jax.random.PRNGKey(0),
        }  # fmt: skip
        data = cpu_data(d)
        # Commands: what CPU GraspEnv.step would set (without stepping the CPU physics).
        pos, quat = fns["wrist"](data, info, jp.asarray(a, jp.float32))
        exp_pos, exp_quat = _cpu_wrist_command(cpu, a)
        stats["wrist"] = max(stats["wrist"], float(np.abs(np.asarray(pos) - exp_pos).max()),
                             float(1 - abs(np.dot(np.asarray(quat), exp_quat))))  # fmt: skip
        fingers = fns["fingers"](info, jp.asarray(a, jp.float32))
        k = cpu.syn.k
        q_cmd = cpu.syn.posture(a[6 : 6 + k]) + cpu.config.residual_scale * a[6 + k :]
        q_cmd = np.clip(q_cmd, scene.spec.lower, scene.spec.upper)
        exp = cpu.finger_target + cpu.config.finger_smoothing * (q_cmd - cpu.finger_target)
        stats["fingers"] = max(stats["fingers"], float(np.abs(np.asarray(fingers) - exp).max()))

        s = fns["sense"](data, info)
        r, *_ = fns["reward"](data, info, s, jp.asarray(a, jp.float32))
        r_cpu, terms_cpu, _ = cpu._reward(a)  # (updates the CPU's hold counter, like a step)
        cpu.prev_action = a
        actor_cpu, critic_cpu = cpu._observe()
        o = fns["observe"](data, {**info, "prev_action": jp.asarray(a), "hold": s["hold"]}, s)
        stats["states"] += 1
        stats["touch_differs"] += int(not np.array_equal(np.asarray(s["found"]), cpu.touch[:6]))
        stats["actor"] = max(
            stats["actor"], float(np.abs(np.asarray(o["state"]) - actor_cpu).max())
        )
        diff = np.abs(np.asarray(o["privileged_state"]) - critic_cpu)
        diff[i_force] = 0.0
        diff[i_table] = 0.0
        if bool(s["opposed"]) != bool(terms_cpu.get("opposition", 0.0) > 0):
            stats["opp_differs"] += 1  # the opposition flag (and what it gates) may differ
            return
        stats["critic"] = max(stats["critic"], float(diff.max()))
        stats["reward"] = max(stats["reward"], abs(float(r) - r_cpu))

    for obj in ("cylinder", "cube"):
        cpu.reset(seed=3, obj=obj, side=side)
        _run_with_checks(cpu, check)
        assert scene.lifted(), f"the scripted grasp should lift the {obj}"
    print(side, {k: round(v, 7) if isinstance(v, float) else v for k, v in stats.items()})
    assert stats["states"] > 100
    assert stats["touch_differs"] == 0
    assert stats["actor"] < 1e-4 and stats["critic"] < 1e-4 and stats["reward"] < 1e-4
    assert stats["opp_differs"] <= 0.05 * stats["states"]
    assert stats["wrist"] < 1e-5 and stats["fingers"] < 1e-5


def _cpu_wrist_command(cpu: GraspEnv, a: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """CPU GraspEnv.step's new wrist target (`_command`), leaving the env as it was."""
    saved = cpu.finger_target.copy(), cpu.wrist_cmd.copy()
    try:
        pos, quat = cpu._command(a)
    finally:
        cpu.finger_target, cpu.wrist_cmd = saved
    return pos, quat / np.linalg.norm(quat)


def _run_with_checks(cpu: GraspEnv, check) -> None:
    """The scripted grasp, calling `check` at every control step."""
    scene, d = cpu.scene, cpu.data
    original, next_t = scene.step, [d.time]

    def stepping(seconds: float) -> None:
        end = d.time + seconds - 1e-9
        while d.time < end:
            original(min(cpu.dt, end - d.time + 1e-9))
            if d.time >= next_t[0] - 1e-9:
                check()
                next_t[0] = d.time + cpu.dt

    scene.step = stepping
    try:
        scripted_grasp(scene, hold=0.3, **ARM_GRASP)
    finally:
        del scene.step


def test_warp_matches_cpu_mujoco(env, fns, bundle):
    """The same states (demo frames, both hands) through MuJoCo Warp's forward and CPU MuJoCo's:
    the same observation and touch flags."""
    import mujoco

    b, m = env._np, env.mj_model
    d = mujoco.MjData(m)
    forward = jax.jit(lambda obj, *x: env.state_data(env.blank_data(), obj, 1.0, *x))
    worst = 0.0
    for si, side in enumerate(SIDES):
        for oi in range(len(OBJECT_KINDS)):
            start, count = b["demo_start"][si, oi], b["demo_count"][si, oi]
            for frame in (start + count // 2, start + count - 1):
                state = [b[k][frame] for k in ("demo_qpos", "demo_qvel", "demo_ctrl",
                                               "demo_mocap_pos", "demo_mocap_quat")]  # fmt: skip
                d.qpos, d.qvel, d.ctrl, d.mocap_pos, d.mocap_quat = state
                d.xfrc_applied[:] = 0
                d.xfrc_applied[b["obj_body"][oi], 2] = -b["obj_mass"][oi] * G.GRAVITY
                mujoco.mj_forward(m, d)
                info = {"side": jp.array(si), "flip": jp.array(si == 1), "obj": jp.array(oi),
                        "rest_z": jp.array(b["demo_rest_z"][frame]), "spawn_xy": jp.zeros(2),
                        "finger_target": jp.asarray(state[2][b["act"][si]]),
                        "prev_action": jp.zeros(env.action_size), "hold": jp.array(0),
                        "prev2_action": jp.zeros(env.action_size), "wrist_cmd": jp.zeros(6),
                        "success": jp.array(0.0), "t": jp.array(0), "obs_bias": jp.zeros(3),
                        "mass_scale": jp.array(1.0), "rng": jax.random.PRNGKey(0)}  # fmt: skip
                warp = forward(jp.array(oi), *map(jp.asarray, state))
                s_warp, s_cpu = fns["sense"](warp, info), fns["sense"](cpu_data(d), info)
                o_warp = fns["observe"](warp, info, s_warp)["state"]
                o_cpu = fns["observe"](cpu_data(d), info, s_cpu)["state"]
                assert np.array_equal(np.asarray(s_warp["found"]), np.asarray(s_cpu["found"]))
                assert bool(s_warp["in_hand"]) == bool(s_cpu["in_hand"])
                worst = max(worst, float(np.abs(np.asarray(o_warp) - np.asarray(o_cpu)).max()))
    print(f"Warp vs CPU MuJoCo, actor obs max diff {worst:.2e}")
    assert worst < 1e-3


def _gpu_state(env, data, info) -> "G.mjx_env.State":
    zero = jp.array(0.0)
    obs = {"state": jp.zeros(1), "privileged_state": jp.zeros(1)}
    metrics = {k: zero for k in env._metric_keys()}
    return G.mjx_env.State(data, obs, zero, zero, metrics, info)


@pytest.mark.parametrize("side", SIDES)
def test_motion_layer_matches_the_cpu_env(env, cpu, bundle, side):
    """The ramps (`finger_ramp`, `wrist_ramp`) in both envs: from the same state (a scripted
    grasp's approach, before contact: with contacts both physics drift apart chaotically, ramp or
    not), the same actions for a few control steps, with and without the ramps: the GPU env
    (MuJoCo Warp) ends where the CPU env does (servo and wrist targets, finger angles), and the
    ramps change the motion by much more than that."""
    import mujoco

    b, oi = env._np, OBJECT_KINDS.index("cylinder")
    si = SIDES.index(side)
    frame = int(b["demo_start"][si, oi]) + 5  # approaching
    keys = ("demo_qpos", "demo_qvel", "demo_ctrl", "demo_mocap_pos", "demo_mocap_quat")
    state = [b[k][frame] for k in keys]
    flat = G.TendraGrasp(bundle, config_overrides={"naconmax": 96, "obs_noise": 0.0,
                                                   "finger_ramp": False, "wrist_ramp": False})  # fmt: skip
    start = jax.jit(lambda *x: env.state_data(env.blank_data(), jp.array(oi), 1.0, *x))
    actions = np.random.default_rng(si).uniform(-0.5, 0.5, (3, env.action_size))
    out = {}
    for ramp, e in ((True, env), (False, flat)):
        # CPU
        cpu.configure(finger_ramp=ramp, wrist_ramp=ramp)
        cpu.reset(seed=3, obj="cylinder", side=side)
        d = cpu.data
        d.qpos, d.qvel, d.ctrl, d.mocap_pos, d.mocap_quat = state
        mujoco.mj_forward(cpu.model, d)
        cpu.finger_target = d.ctrl[cpu._act].copy()
        cpu_rows = []
        for a in actions:
            cpu.step(a)
            cpu_rows.append((d.ctrl[cpu._act].copy(), d.mocap_pos[b["mocap"][si]].copy(),
                             cpu.scene.finger_positions()))  # fmt: skip
        # GPU
        info = {"side": jp.array(si), "flip": jp.array(side == "left"), "obj": jp.array(oi),
                "rest_z": jp.array(b["demo_rest_z"][frame]),
                "spawn_xy": jp.asarray(cpu.spawn_xy), "finger_target": jp.asarray(state[2][b["act"][si]]),
                "prev_action": jp.zeros(env.action_size), "hold": jp.array(0),
                "prev2_action": jp.zeros(env.action_size), "wrist_cmd": jp.zeros(6),
                "success": jp.array(0.0), "t": jp.array(0), "obs_bias": jp.zeros(3),
                "mass_scale": jp.array(1.0), "rng": jax.random.PRNGKey(0),
                "from_demo": jp.array(1.0), "from_near": jp.array(0.0)}  # fmt: skip
        st = _gpu_state(e, start(*map(jp.asarray, state)), info)
        step = jax.jit(e.step)
        gpu_rows = []
        for a in actions:
            st = step(st, jp.asarray(a, jp.float32))
            gpu_rows.append((np.asarray(st.data.ctrl[b["act"][si]]),
                             np.asarray(st.data.mocap_pos[b["mocap"][si]]),
                             np.asarray(st.data.qpos[b["qadr"][si]])))  # fmt: skip
        out[ramp] = cpu_rows, gpu_rows
    cpu.configure(finger_ramp=True, wrist_ramp=True)
    worst = {"ctrl": 0.0, "mocap": 0.0, "fingers": 0.0, "ramp_effect": 0.0}
    for ramp in (True, False):
        for (c_ctrl, c_moc, c_q), (g_ctrl, g_moc, g_q) in zip(*out[ramp], strict=True):
            worst["ctrl"] = max(worst["ctrl"], float(np.abs(c_ctrl - g_ctrl).max()))
            worst["mocap"] = max(worst["mocap"], float(np.abs(c_moc - g_moc).max()))
            worst["fingers"] = max(worst["fingers"], float(np.abs(c_q - g_q).max()))
    for (_, _, q_ramp), (_, _, q_flat) in zip(out[True][0], out[False][0], strict=True):
        worst["ramp_effect"] = max(worst["ramp_effect"], float(np.abs(q_ramp - q_flat).max()))
    print(side, {k: f"{v:.2e}" for k, v in worst.items()})
    assert worst["ctrl"] < 1e-4 and worst["mocap"] < 1e-4
    assert worst["fingers"] < 2e-3 and worst["ramp_effect"] > 0.01


def test_action_delay_per_world(bundle):
    """Servo latency: each world draws its delay in `action_delay`; until its first action is in
    effect the finger targets hold, then the action from `delay` steps ago applies."""
    e = G.TendraGrasp(bundle, config_overrides={"naconmax": 96 * N_WORLDS, "obs_noise": 0.0,
                                                "action_delay": [0, 2]})  # fmt: skip
    state = jax.jit(jax.vmap(e.reset))(jax.random.split(jax.random.PRNGKey(2), N_WORLDS * 2))
    delay = np.asarray(state.info["delay"])
    assert set(delay) <= {0, 1, 2} and len(set(delay)) > 1
    a1 = np.zeros((N_WORLDS * 2, e.action_size), np.float32)
    a1[:, 6] = 1.0  # "close"
    step = jax.jit(jax.vmap(e.step))
    targets = [np.asarray(state.info["finger_target"])]
    for _ in range(3):
        state = step(state, jp.asarray(a1))
        targets.append(np.asarray(state.info["finger_target"]))
    moved = [np.abs(t - targets[0]).max(axis=1) > 1e-3 for t in targets[1:]]
    for w, dl in enumerate(delay):
        assert [m[w] for m in moved] == [k + 1 > dl for k in range(3)], (w, dl)
    with pytest.raises(ValueError):
        G.TendraGrasp(bundle, config_overrides={"action_delay": [2, 1]})


def test_auto_reset_starts_fresh_episodes(bundle):
    """Under Playground's auto-reset (+ our staggered starts) a done world gets its cached first
    state back but keeps its info: the env must reset the per-episode info itself. Same start,
    same action: every episode repeats the first one step for step (time, success, wrist
    command, finger target, reward), so a success is rewarded again in each episode."""
    from mujoco_playground import wrapper
    from tendra.gpu.stagger import StaggeredResets

    n, length = 2 * N_WORLDS, 3
    e = G.TendraGrasp(bundle, config_overrides={
        "naconmax": 96 * n, "obs_noise": 0.0, "start_probs.normal": 0.0, "start_probs.demo": 1.0,
        "hold_seconds": 0.01, "lift_height": 0.02, "action_delay": [1, 1]})  # fmt: skip
    wrapped = StaggeredResets(wrapper.wrap_for_brax_training(e, episode_length=length), length,
                              min_left=1)  # fmt: skip
    state = jax.jit(wrapped.reset)(jax.random.split(jax.random.PRNGKey(3), n))
    a = np.zeros((n, e.action_size), np.float32)
    a[:, 6], a[:, 2] = 1.0, 0.5  # close, and lift
    step = jax.jit(wrapped.step)
    episodes = [[[]] for _ in range(n)]  # per world: episodes of per-step rows
    for _ in range(8):
        state = step(state, jp.asarray(a))
        info = state.info
        for w in range(n):
            row = (int(info["t"][w]), float(state.metrics["success"][w]),
                   np.asarray(info["wrist_cmd"][w]), np.asarray(info["finger_target"][w]),
                   float(state.reward[w]))  # fmt: skip
            episodes[w][-1].append(row)
            if state.done[w]:
                episodes[w].append([])
    repeated_success = 0
    for w in range(n):
        done = [ep for ep in episodes[w] if ep]
        assert len(done) >= 3, f"world {w}: episodes {[len(ep) for ep in done]}"
        first = done[1]  # the first full one (episode 0 may be cut short by the stagger)
        assert [r[0] for r in first] == list(range(1, length + 1))
        for ep in done[2:]:
            for r, f in zip(ep, first):
                assert r[0] == f[0] and r[1] == f[1], (w, r[:2], f[:2])
                assert np.allclose(r[2], f[2]) and np.allclose(r[3], f[3], atol=1e-5)
                assert abs(r[4] - f[4]) < 1e-3
        repeated_success += sum(r[1] for r in done[2]) > 0
    assert repeated_success > 0, "some world should succeed again in a later episode"


def test_random_rollout_stays_finite(env, fns):
    state = fns["reset"](jax.random.split(jax.random.PRNGKey(5), N_WORLDS))
    rng = np.random.default_rng(0)
    for _ in range(4):
        action = jp.asarray(rng.uniform(-1, 1, (N_WORLDS, env.action_size)), jp.float32)
        state = fns["step"](state, action)
        for obs in state.obs.values():
            assert np.isfinite(np.asarray(obs)).all()
        assert np.isfinite(np.asarray(state.reward)).all()
    assert float(np.asarray(state.metrics["blowup"]).max()) == 0.0
    assert float(np.asarray(state.metrics["ik_err_mm"]).max()) * env._config.episode_length < 5


def test_both_hands_start(bundle):
    """`hands` picks the side per world; the left hand's spawn is the mirror image."""
    sides = {}
    for hands in ("right", "left"):
        e = G.TendraGrasp(bundle, config_overrides={"naconmax": 96 * N_WORLDS, "hands": hands})
        state = jax.jit(jax.vmap(e.reset))(jax.random.split(jax.random.PRNGKey(0), N_WORLDS))
        sides[hands] = state
        assert (np.asarray(state.info["side"]) == SIDES.index(hands)).all()
        obj_x = np.array([state.data.xpos[w, e._np["obj_body"][int(state.info["obj"][w])], 0]
                          for w in range(N_WORLDS)])  # fmt: skip
        assert (obj_x <= 0.001).all() if hands == "right" else (obj_x >= -0.001).all()
    # Same keys: the same draws, mirrored (canonical observations equal, up to the settling
    # object's last wobble, ~2e-3 rad/s).
    assert np.allclose(sides["right"].obs["state"], sides["left"].obs["state"], atol=5e-3)


def test_start_kinds_and_dr_level(bundle):
    """`start_probs` (flattened keys, as the trainer sends them): near starts are a scripted
    grasp's approach, the hand close but not touching; `dr_level` 0 = the nominal object, no
    noise (with `obs_noise_floor` 0; by default some noise stays). The old `demo_prob` still
    gives demo starts."""
    keys = jax.random.split(jax.random.PRNGKey(1), N_WORLDS)
    near = G.TendraGrasp(bundle, config_overrides={
        "naconmax": 96 * N_WORLDS, "start_probs.normal": 0.0, "start_probs.near": 1.0,
        "dr_level": 0.0, "obs_noise_floor": 0.0})  # fmt: skip
    state = jax.jit(jax.vmap(near.reset))(keys)
    s = jax.vmap(near.sense)(state.data, state.info)
    assert (np.asarray(state.info["from_near"]) == 1).all()
    assert not np.asarray(s["in_hand"]).any()
    gap = np.linalg.norm(np.asarray(s["root_pos"] - s["obj_pos"]), axis=1)
    assert (gap <= 0.15 + 1e-3).all()
    assert (
        np.allclose(state.info["mass_scale"], 1.0) and not np.asarray(state.info["obs_bias"]).any()
    )
    old = G.TendraGrasp(bundle, config_overrides={"naconmax": 96 * N_WORLDS, "demo_prob": 1.0})
    state = jax.jit(jax.vmap(old.reset))(keys)
    assert (np.asarray(state.info["from_demo"]) == 1).all()
    with pytest.raises(ValueError):
        G.TendraGrasp(bundle, config_overrides={"dr_level": 1.5})


def test_friction_per_world(env, bundle):
    """`domain_randomizer` through Playground's training wrapper: each world has its own object
    friction (the critic sees it), and it matters: the objects have contact priority, so a
    slippery cylinder slides out of a grasp that holds the nominal one."""
    import functools

    from mujoco_playground import wrapper
    from mujoco_playground._src import mjx_env

    keys = jax.random.split(jax.random.PRNGKey(0), N_WORLDS)
    wrapped = wrapper.wrap_for_brax_training(
        env, episode_length=8, randomization_fn=functools.partial(env.domain_randomizer, rng=keys)
    )
    state = jax.jit(wrapped.reset)(keys)
    state = jax.jit(wrapped.step)(state, jp.zeros((N_WORLDS, env.action_size)))
    model_friction = np.asarray(wrapped._mjx_model_v.geom_friction[:, env._obj_geom, 0])
    lo, hi = env._config.friction
    assert (model_friction >= lo).all() and (model_friction <= hi).all()
    assert np.ptp(model_friction[:, 0]) > 0.05  # differs per world
    obj = np.asarray(state.info["obj"])
    i = state.obs["state"].shape[-1] + 7 + 1  # critic: actor obs, 7 forces, mass, friction
    seen = np.asarray(state.obs["privileged_state"][:, i])
    assert np.allclose(seen, model_friction[np.arange(N_WORLDS), obj])

    b, oi = env._np, OBJECT_KINDS.index("cylinder")
    k = int(b["demo_start"][0, oi] + b["demo_count"][0, oi] - 1)  # held, right hand
    held = [jp.asarray(b[key][k]) for key in ("demo_qpos", "demo_qvel", "demo_ctrl",
                                              "demo_mocap_pos", "demo_mocap_quat")]  # fmt: skip
    data = jax.jit(jax.vmap(lambda _: env.state_data(env.blank_data(), jp.array(oi), 1.0, *held)))(
        jp.arange(2)
    )
    m, g = env.mjx_model, env._obj_geom[oi]
    models = m.tree_replace(
        {"geom_friction": jp.stack([m.geom_friction.at[g, 0].set(f) for f in (1.0, 0.05)])}
    )
    axes = jax.tree.map(lambda _: None, m).tree_replace({"geom_friction": 0})
    run = jax.vmap(lambda mm, d: mjx_env.step(mm, d, d.ctrl, 150), in_axes=(axes, 0))
    z = np.asarray(jax.jit(run)(models, data).xpos[:, b["obj_body"][oi], 2])
    start = float(data.xpos[0, b["obj_body"][oi], 2])
    assert abs(z[0] - start) < 0.005 and z[1] < start - 0.05, (start, z)


def test_demo_replay_lifts_the_object(env, bundle):
    """The scripted grasps' wrist targets and finger commands, replayed from when the fingers
    close: the JAX IK drives the arm in MuJoCo Warp and the hand lifts the object (both hands)."""
    b = env._np
    oi = OBJECT_KINDS.index("cylinder")
    starts, counts = b["demo_start"][:, oi], b["demo_count"][:, oi]
    first = 38  # frames: approach 1.8 s / 48 ms, then the fingers close
    n = int(counts.min()) - first
    rows = lambda key, k: jp.asarray(np.stack([b[key][s + first + k] for s in starts]))

    def start(qpos, qvel, ctrl, mpos, mquat):
        return env.state_data(env.blank_data(), jp.array(oi), 1.0, qpos, qvel, ctrl, mpos, mquat)

    def replay(data, ctrl, mpos, mquat, side):
        data = data.replace(mocap_pos=mpos, mocap_quat=mquat)
        return env.physics(data, ctrl, side, env._chunks)[0]

    data = jax.jit(jax.vmap(start))(*(rows(k, 0) for k in ("demo_qpos", "demo_qvel", "demo_ctrl",
                                                           "demo_mocap_pos", "demo_mocap_quat")))  # fmt: skip
    step = jax.jit(jax.vmap(replay))
    sides = jp.arange(2)
    for k in range(n):
        data = step(data, rows("demo_ctrl", k), rows("demo_mocap_pos", k),
                    rows("demo_mocap_quat", k), sides)  # fmt: skip
    z = np.asarray(data.xpos[:, b["obj_body"][oi], 2])
    rest = np.array([b["demo_rest_z"][s] for s in starts])
    table = np.asarray(data.sensordata[:, b["sensor_table_obj"][oi]])
    print(f"replayed {n} steps: lifted {np.round(100 * (z - rest), 1)} cm")
    assert (z - rest > 0.05).all() and (table == 0).all()


def test_train_brax_smoke(bundle, tmp_path):
    script = Path(G.__file__).with_name("train_brax.py")
    out = tmp_path / "run"
    result = subprocess.run([sys.executable, str(script), "--bundle", str(bundle), "--out",
                             str(out), "--smoke"], capture_output=True, text=True, check=False,
                            timeout=3600)  # fmt: skip
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    assert any(out.rglob("*.pkl")) or any(out.rglob("*.json"))
