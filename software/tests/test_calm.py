import sys
from pathlib import Path

import numpy as np
import pytest
from tendra.calm import GRASP, HOLD, REACH, RELEASE, CalmFilter, HoldLatch

DT = 0.05
R = np.radians


def make(**kw):
    args = {"deadband": R(0.3), "release": R(1.0), "max_rate": 10.0, "dt": DT}
    args.update(kw)
    f = CalmFilter(2, **args)
    f.reset(np.zeros(2))
    return f


def test_small_changes_are_ignored():
    f = make()
    for x in (R(0.5), R(-0.7), R(0.9), 0.0):
        assert np.all(f(np.array([x, x])) == 0.0)


def test_hysteresis_start_big_keep_small():
    f = make()
    f(np.array([R(2.0), 0.0]))  # beyond release: joint 0 starts
    assert f.moving[0] and not f.moving[1]
    out = f(np.array([R(2.0) + R(0.5), 0.0]))  # a 0.5 deg change keeps it moving (> deadband)
    assert out[0] > R(2.0) - 1e-9
    f(np.array([R(2.5), 0.0]))  # target still: change < deadband and within release -> latch
    assert not f.moving[0]
    held = f.out[0]
    f(np.array([held + R(0.9), 0.0]))  # inside release: stays latched
    assert f.out[0] == held


def test_rate_limit():
    f = make(max_rate=1.0)
    out = f(np.array([1.0, -1.0]))
    assert out[0] == pytest.approx(DT) and out[1] == pytest.approx(-DT)


def test_smoothing_converges():
    f = make(smoothing=0.5)
    for _ in range(60):
        out = f(np.array([0.5, 0.5]))
    assert out[0] == pytest.approx(0.5, abs=R(1.0))


def test_latch_freezes_and_releases():
    lat = HoldLatch(hold_steps=3)
    q = np.full(4, 0.5)
    lat(q, contact=False)
    assert lat.phase == REACH
    lat(q, contact=True)
    assert lat.phase == GRASP
    for _ in range(3):
        lat(q, lifted=True, contact=True)
    assert lat.phase == HOLD
    # policy noise is blocked while holding
    assert np.all(lat(q + R(2.0), lifted=True, contact=True) == q)
    # slip -> regrip (unfrozen)
    lat(q, lifted=True, contact=True, slip=0.2)
    assert lat.phase == GRASP
    for _ in range(3):
        lat(q, lifted=True, contact=True)
    assert lat.phase == HOLD
    # large commanded change -> release, target passes through
    out = lat(q - 0.5, lifted=True, contact=True)
    assert lat.phase == RELEASE and np.all(out == q - 0.5)
    lat(q - 0.5, lifted=False, contact=False)
    assert lat.phase == REACH
    assert [p for _, p in lat.log][:3] == [REACH, GRASP, HOLD]


def test_eval_grasp_calm_smoke():
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim"))
    ck = Path(__file__).resolve().parents[2] / "runs" / "arm_right" / "best.pt"
    if not ck.exists():
        pytest.skip("no runs/arm_right/best.pt")
    import torch
    from eval_grasp import run_episode
    from tendra.calm import calm_applied
    from tendra.rl.ppo import Policy
    from tendra.rl.train import load_env_for

    c = torch.load(ck, map_location="cpu", weights_only=False)
    env = load_env_for(c, randomize=False, demo_prob=0.0)
    env.configure(objects=["cube"])
    env.config.episode_seconds = 25 * env.dt
    policy = Policy(c)
    plain_step = env.step
    with calm_applied(env, "full") as layer:
        ep = run_episode(env, policy, np.random.default_rng(0), "cube", False)
        assert ep["length"] > 0 and layer.phases
    assert env.step == plain_step and "set_finger_targets" not in env.scene.__dict__


def test_wrist_filter_reaches_mocap_and_nothing_left_patched():
    ck = Path(__file__).resolve().parents[2] / "runs" / "arm_right" / "best.pt"
    if not ck.exists():
        pytest.skip("no runs/arm_right/best.pt")
    import torch
    from tendra.calm import WRIST, calm_applied
    from tendra.rl.train import load_env_for

    c = torch.load(ck, map_location="cpu", weights_only=False)
    env = load_env_for(c, randomize=False, demo_prob=0.0)
    env.configure(objects=["cube"])
    s = env.scene
    before = (set(env.__dict__), set(s.__dict__))
    wrist = {**WRIST, "deadband": 0.0, "release": 10.0}  # nothing may move by 10 m
    with calm_applied(env, "light", wrist=wrist) as layer:
        env.reset()
        start = s.wrist_target()[0].copy()
        a = np.zeros(env.action_dim)
        a[0] = 1.0
        for _ in range(3):
            env.step(a)
        assert layer.raw_wrist is not None
        assert np.linalg.norm(layer.raw_wrist - start) > 1e-3  # the raw target did advance
        assert np.allclose(s.wrist_target()[0], start)  # but the mocap (IK) target held
    with calm_applied(env, "light"):
        env.reset()
        for _ in range(3):
            env.step(a)
        assert np.linalg.norm(s.wrist_target()[0] - layer.raw_wrist) < 0.02  # moves when allowed
    assert (set(env.__dict__), set(s.__dict__)) == before
