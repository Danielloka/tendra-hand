"""PPO, parallel envs and the grasp trainer (tendra.rl). Needs PyTorch (group `train`)."""

import functools

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("fast_simplification")

from tendra.grasp_env import GraspEnvConfig, config_dict, make_env
from tendra.rl.ppo import PPO, Policy, PPOConfig, RunningNorm, gae
from tendra.rl.train import Curriculum, TrainConfig, load_env_for, train
from tendra.rl.vec_env import VecEnv


def test_running_norm_matches_numpy():
    rng = np.random.default_rng(0)
    x = rng.normal(3.0, 2.0, (1000, 4))
    norm = RunningNorm(4)
    for chunk in np.array_split(x, 7):
        norm.update(chunk)
    assert np.allclose(norm.mean, x.mean(axis=0), atol=1e-3)
    assert np.allclose(norm.var, x.var(axis=0), rtol=1e-3)
    assert np.abs(norm(x).mean(axis=0)).max() < 1e-2


def test_gae_by_hand():
    # One env, 3 steps, episode ends after step 1 (index 1); gamma 0.5, lambda 1 = plain returns.
    r = np.array([[1.0], [2.0], [4.0]], np.float32)
    v = np.zeros((3, 1), np.float32)
    done = np.array([[False], [True], [False]])
    adv = gae(r, v, done, last_value=np.array([8.0], np.float32), gamma=0.5, lam=1.0)
    assert np.allclose(adv[:, 0], [1 + 0.5 * 2, 2, 4 + 0.5 * 8])


def test_ppo_learns_a_bandit():
    """Reward = -|a - 0.5|^2 per dim: the mean action must move to 0.5."""
    ppo = PPO(3, 3, 2, PPOConfig(actor_hidden=[32], critic_hidden=[32]))
    obs = np.ones((64, 3), np.float32)
    for _ in range(30):
        a, logp, v = ppo.act(obs, obs)
        r = -((np.clip(a, -1, 1) - 0.5) ** 2).sum(-1)
        buf = {"obs": ppo.obs_norm(obs)[None], "critic": ppo.critic_norm(obs)[None],
               "actions": a[None], "logp": logp[None], "values": v[None],
               "rewards": r[None].astype(np.float32), "dones": np.ones((1, 64), bool)}  # fmt: skip
        ppo.update(buf, np.zeros(64, np.float32))
    mean = ppo.act(obs[:1], obs[:1], deterministic=True)[0]
    assert np.allclose(mean, 0.5, atol=0.15), mean


def test_curriculum_promotes_and_fades_demos():
    cfg = TrainConfig(window=20, promote_at=0.6)
    cur = Curriculum(cfg)
    assert cur.objects == ["cylinder"] and cur.demo_prob == cfg.demo_start
    for _ in range(10):
        cur.add({"demo": True, "object": "cylinder", "success": True})  # demo starts don't count
    assert cur.success_rate == 0.0
    for i in range(10):
        cur.add({"demo": False, "object": "cylinder", "success": i < 7})
    assert cur.demo_prob == pytest.approx(cfg.demo_start * 0.3)
    assert cur.maybe_promote() and cur.stage == 1 and cur.objects == ["cylinder", "cube"]


def test_vec_env_in_worker_processes():
    factory = functools.partial(make_env, config_dict(GraspEnvConfig()))
    with VecEnv(factory, num_envs=3, workers=2, seed=1) as venv:
        obs, critic = venv.reset()
        assert obs.shape[0] == 3 and critic.shape[0] == 3
        venv.call("configure", episode_seconds=0.15)  # 3 steps: forces auto-resets
        ends = 0
        for _ in range(8):
            obs, critic, _, term, trunc, infos = venv.step(np.zeros((3, 28)))  # 6 + 6 + 16
            ends += int(np.sum(term | trunc))
            for i in np.flatnonzero(term | trunc):
                assert "episode" in infos[i] and infos[i]["final_critic"].shape == critic[i].shape
        assert ends >= 3
        assert venv.call("configure", demo_prob=0.0) == [None] * 3


def test_train_resume_and_policy(tmp_path):
    cfg = TrainConfig(run_dir=tmp_path / "run", total_steps=64, num_envs=2, workers=0,
                      horizon=16, scripted_demos=1)  # fmt: skip
    latest = train(cfg, log=lambda *_: None)
    ck = torch.load(latest, weights_only=False)
    assert ck["steps"] == 64 and (tmp_path / "run" / "progress.csv").exists()
    cfg.total_steps = 96
    train(cfg, log=lambda *_: None)  # resumes
    assert torch.load(latest, weights_only=False)["steps"] == 96

    policy = Policy.load(str(latest))
    env = load_env_for(ck)
    obs, _ = env.reset(seed=0)
    a = policy(obs)
    assert a.shape == (env.action_dim,) and np.all(np.abs(a) <= 1)
    assert np.allclose(env.syn.basis, ck["synergies"]["basis"])
