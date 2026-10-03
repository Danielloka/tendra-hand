"""The GPU grasp env (`tendra.gpu`: JAX + MuJoCo Warp) checked against the CPU env, on the CPU.

    uv run --group gpu pytest software/tests/test_gpu_env.py

Skipped without the `gpu` dependency group. Slow (several minutes): the bundle export records
scripted grasps, and Warp compiles its kernels on first use.
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
        "wrist": jax.jit(env.wrist_command),
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
        cpu.t = 7
        a = rng.uniform(-1, 1, cpu.action_dim)
        info = {
            "side": jp.array(SIDES.index(side)), "flip": jp.array(side == "left"),
            "obj": jp.array(OBJECT_KINDS.index(scene.active_object)),
            "rest_z": jp.array(scene.rest_height), "spawn_xy": jp.asarray(cpu.spawn_xy),
            "finger_target": jp.asarray(cpu.finger_target),
            "prev_action": jp.asarray(cpu.prev_action), "hold": jp.array(cpu.hold),
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
    """CPU GraspEnv.step's new wrist target, leaving the simulation and the env as they were."""
    s, d = cpu.scene, cpu.data
    saved = (d.ctrl.copy(), cpu.finger_target.copy(), cpu.prev_action.copy(), cpu.ep_return, cpu.t)
    patched = s.__dict__.get("step")  # the scripted grasp's own wrapper, if any
    out = {}
    s.set_wrist_target = lambda pos, quat: out.update(pos=pos, quat=quat / np.linalg.norm(quat))
    s.step, cpu._reward = (lambda dt: None), (lambda a: (0.0, {}, False))
    try:
        cpu.step(a)
    finally:
        del s.set_wrist_target, s.step, cpu._reward
        if patched is not None:
            s.step = patched
        d.ctrl[:], cpu.finger_target, cpu.prev_action, cpu.ep_return, cpu.t = saved
    return out["pos"], out["quat"]


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
