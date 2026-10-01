"""The grasp trainer: PPO on many `GraspEnv`s, with an automatic curriculum.

    cfg = TrainConfig(run_dir=Path("runs/grasp"), total_steps=20_000_000)
    train(cfg)            # resumes by itself if run_dir has a checkpoint

**Curriculum** (like a human child: one easy object first, help that fades):
- Objects come in stages (default: cylinder → + cube → + ball). The next stage starts once the
  success rate on *normal* episodes (not demo starts) passes `promote_at`.
- Demo starts (`GraspEnv.demo_prob`) fade as the policy gets better:
  demo_prob = demo_start * (1 - success), never below `demo_min`.

**Run folder** (`run_dir`): `config.json`, `progress.csv` (one row per update: speed, returns,
success rates, curriculum, losses, mean reward terms), `latest.pt` (to resume), `best.pt`
(highest success on the last stage reached), `ckpt_<steps>.pt` every `save_every` updates, `demos.pkl` (the demo states, reused on resume).
"""

from __future__ import annotations

import csv
import functools
import json
import pickle
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import torch

from ..grasp_env import Demo, GraspEnv, GraspEnvConfig, config_dict, config_from_dict, make_env
from .ppo import PPO, PPOConfig, ReturnScaler
from .vec_env import VecEnv


@dataclass
class TrainConfig:
    run_dir: Path = Path("runs/grasp")
    total_steps: int = 20_000_000
    num_envs: int = 32
    workers: int = 4  # processes; 0 = everything in this process
    horizon: int = 64  # control steps per env per update (64 x 32 = 2048 samples)
    seed: int = 0
    env: GraspEnvConfig = field(default_factory=GraspEnvConfig)
    ppo: PPOConfig = field(default_factory=PPOConfig)
    stages: list[list[str]] = field(default_factory=lambda: [
        ["cylinder"], ["cylinder", "cube"], ["cylinder", "cube", "ball"]])  # fmt: skip
    promote_at: float = 0.6
    window: int = 200  # episodes in the success-rate window
    scripted_demos: int = 60  # scripted grasps to record as demo starts (0 = none)
    datasets: list[str] = field(default_factory=list)  # teleop datasets to use as demos too
    demo_start: float = 0.5
    demo_min: float = 0.05
    penalty_start: float = 0.3  # penalty_scale at the start; reaches 1 with the last stage
    save_every: int = 50  # updates
    log_every: int = 1


def _jsonable(obj: Any) -> Any:
    if isinstance(obj, Path):
        return str(obj)
    if isinstance(obj, dict):
        return {k: _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, list | tuple):
        return [_jsonable(v) for v in obj]
    return obj


class Curriculum:
    def __init__(self, cfg: TrainConfig) -> None:
        self.cfg = cfg
        self.stage = 0
        self.results: deque[bool] = deque(maxlen=cfg.window)

    @property
    def objects(self) -> list[str]:
        return self.cfg.stages[self.stage]

    @property
    def success_rate(self) -> float:
        return float(np.mean(self.results)) if self.results else 0.0

    @property
    def penalty_scale(self) -> float:
        """Grows with the stage and the success within it: 1.0 once the last stage is mastered."""
        c = self.cfg
        progress = (self.stage + min(1.0, self.success_rate / c.promote_at)) / len(c.stages)
        return float(c.penalty_start + (1.0 - c.penalty_start) * min(1.0, progress))

    def env_settings(self) -> dict[str, Any]:
        return {"objects": self.objects, "demo_prob": self.demo_prob,
                "penalty_scale": self.penalty_scale}  # fmt: skip

    @property
    def demo_prob(self) -> float:
        c = self.cfg
        return float(max(c.demo_min, c.demo_start * (1.0 - self.success_rate)))

    def add(self, episode: dict[str, Any]) -> None:
        if not episode["demo"] and episode["object"] in self.objects:
            self.results.append(bool(episode["success"]))

    def maybe_promote(self) -> bool:
        full = len(self.results) >= self.cfg.window // 2
        if (
            full
            and self.success_rate >= self.cfg.promote_at
            and self.stage < len(self.cfg.stages) - 1
        ):
            self.stage += 1
            self.results.clear()
            return True
        return False

    def state_dict(self) -> dict:
        return {"stage": self.stage, "results": list(self.results)}

    def load_state_dict(self, s: dict) -> None:
        self.stage = s["stage"]
        self.results = deque(s["results"], maxlen=self.cfg.window)


def collect_demos(cfg: TrainConfig, env: GraspEnv, log=print) -> list[Demo]:
    from ..grasp_env import demos_from_dataset

    demos = []
    if cfg.scripted_demos:
        t0 = time.perf_counter()
        demos += env.record_scripted_demos(cfg.scripted_demos, seed=cfg.seed)
        kinds = {k: sum(d.obj == k for d in demos) for k in env.scene.config.objects}
        log(f"scripted demos: {len(demos)}/{cfg.scripted_demos} succeeded {kinds} "
            f"({time.perf_counter() - t0:.1f} s)")  # fmt: skip
    for root in cfg.datasets:
        try:
            got = demos_from_dataset(root, env.model)
            demos += got
            log(f"dataset demos: {len(got)} from {root}")
        except (ValueError, FileNotFoundError) as e:
            log(f"skipping dataset {root}: {e}")
    return demos


def train(cfg: TrainConfig, log=print) -> Path:
    run = Path(cfg.run_dir)
    run.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(cfg.seed)
    torch.set_num_threads(1 if cfg.workers else torch.get_num_threads())

    # One local env: sizes, demos, and it stays out of the workers' way.
    probe = GraspEnv(cfg.env, seed=cfg.seed)
    demo_file = run / "demos.pkl"  # recording them takes ~30 s: keep them for resumes
    if demo_file.exists():
        demos = pickle.loads(demo_file.read_bytes())
        log(f"demos: {len(demos)} from {demo_file}")
    else:
        demos = collect_demos(cfg, probe, log)
        demo_file.write_bytes(pickle.dumps(demos))
    ppo = PPO(probe.obs_dim, probe.critic_dim, probe.action_dim, cfg.ppo)
    curriculum = Curriculum(cfg)
    steps, updates, best = 0, 0, -1.0

    latest = run / "latest.pt"
    if latest.exists():
        ck = torch.load(latest, map_location=cfg.ppo.device, weights_only=False)
        ppo.load_state_dict(ck["ppo"])
        curriculum.load_state_dict(ck["curriculum"])
        steps, updates, best = ck["steps"], ck["updates"], ck.get("best", -1.0)
        log(f"resumed from {latest}: {steps:,} steps, stage {curriculum.stage}")
    (run / "config.json").write_text(json.dumps(_jsonable(asdict(cfg)), indent=2))

    factory = functools.partial(make_env, config_dict(cfg.env))
    venv = VecEnv(factory, cfg.num_envs, cfg.workers, seed=cfg.seed + updates)
    try:
        venv.call("set_demos", demos)
        venv.call("configure", **curriculum.env_settings())
        obs, critic = venv.reset()
        scaler = ReturnScaler(cfg.num_envs, cfg.ppo.gamma)
        episodes: deque[dict] = deque(maxlen=cfg.window)
        csv_path = run / "progress.csv"
        writer = None
        csv_file = open(csv_path, "a", newline="")  # noqa: SIM115
        n, t_dim = cfg.num_envs, cfg.horizon

        while steps < cfg.total_steps:
            t0 = time.perf_counter()
            buf = {
                "obs": np.zeros((t_dim, n, probe.obs_dim), np.float32),
                "critic": np.zeros((t_dim, n, probe.critic_dim), np.float32),
                "actions": np.zeros((t_dim, n, probe.action_dim), np.float32),
                "logp": np.zeros((t_dim, n), np.float32),
                "values": np.zeros((t_dim, n), np.float32),
                "rewards": np.zeros((t_dim, n), np.float32),
                "dones": np.zeros((t_dim, n), bool),
            }  # fmt: skip
            term_sums: dict[str, float] = {}
            for t in range(t_dim):
                ppo.obs_norm.update(obs)
                ppo.critic_norm.update(critic)
                action, logp, value = ppo.act(obs, critic)
                buf["obs"][t], buf["critic"][t] = ppo.obs_norm(obs), ppo.critic_norm(critic)
                buf["actions"][t], buf["logp"][t], buf["values"][t] = action, logp, value
                obs, critic, reward, term, trunc, infos = venv.step(action)
                done = term | trunc
                reward = scaler(reward.astype(np.float64), done)
                # Time-limit endings: add back the value of where the episode would have gone.
                for i in np.flatnonzero(trunc & ~term):
                    reward[i] += cfg.ppo.gamma * ppo.value(infos[i]["final_critic"][None])[0]
                buf["rewards"][t], buf["dones"][t] = reward, done
                for info in infos:
                    for k, v in info["terms"].items():
                        term_sums[k] = term_sums.get(k, 0.0) + float(v)
                    if "episode" in info:
                        episodes.append(info["episode"])
                        curriculum.add(info["episode"])
            steps += t_dim * n
            collect_time = time.perf_counter() - t0

            t1 = time.perf_counter()
            stats = ppo.update(buf, ppo.value(critic))
            update_time = time.perf_counter() - t1
            updates += 1
            if curriculum.maybe_promote():
                log(f"== stage {curriculum.stage}: objects {curriculum.objects}")
            venv.call("configure", **curriculum.env_settings())

            normal = [e for e in episodes if not e["demo"]]
            demo_eps = [e for e in episodes if e["demo"]]
            row = {
                "steps": steps, "updates": updates,
                "fps": round(t_dim * n / (time.perf_counter() - t0)),
                "collect_s": round(collect_time, 2), "update_s": round(update_time, 2),
                "return": _mean([e["return"] for e in episodes]),
                "length": _mean([e["length"] for e in episodes]),
                "success": _mean([e["success"] for e in normal]),
                "success_demo": _mean([e["success"] for e in demo_eps]),
                "stage_success": round(curriculum.success_rate, 3),
                "max_height": _mean([e["max_height"] for e in normal]),
                "fail": _mean([e["fail"] for e in episodes]),
                "unstable": _mean([e["unstable"] for e in episodes]),
                "stage": curriculum.stage, "demo_prob": round(curriculum.demo_prob, 3),
                "penalty_scale": round(curriculum.penalty_scale, 3),
                **{k: round(v, 5) for k, v in stats.items()},
                **{f"r_{k}": round(v / (t_dim * n), 4) for k, v in sorted(term_sums.items())},
            }  # fmt: skip
            if writer is None or set(row) - set(writer.fieldnames):
                if writer is not None:  # a new reward term appeared: start a new header block
                    csv_file.write("\n")
                writer = csv.DictWriter(csv_file, fieldnames=list(row), extrasaction="ignore")
                writer.writeheader()
            writer.writerow(row)
            csv_file.flush()
            if updates % cfg.log_every == 0:
                log(f"{steps:>11,} | {row['fps']:>5} sps | ret {row['return']:7.2f} | "
                    f"success {row['success']:.2f} (stage {curriculum.stage} "
                    f"{curriculum.success_rate:.2f}, demo {row['success_demo']:.2f}) | "
                    f"lift {row['max_height'] * 100:4.1f} cm | kl {stats['kl']:.4f} "
                    f"lr {stats['lr']:.1e} std {stats['std']:.2f}")  # fmt: skip

            score = curriculum.stage + curriculum.success_rate  # later stages always beat earlier
            ck = {
                "ppo": ppo.state_dict(), "curriculum": curriculum.state_dict(),
                "steps": steps, "updates": updates, "best": max(best, score),
                "env_config": config_dict(cfg.env), "synergies": _synergies(probe),
            }  # fmt: skip
            _save(ck, latest)
            if score > best and len(curriculum.results) >= cfg.window // 2:
                best = score
                _save(ck, run / "best.pt")
            if updates % cfg.save_every == 0:
                _save(ck, run / f"ckpt_{steps:010d}.pt")
        csv_file.close()
    finally:
        venv.close()
    return latest


def _mean(xs: list) -> float:
    return round(float(np.mean(xs)), 4) if xs else float("nan")


def _synergies(env: GraspEnv) -> dict:
    return {"rest": env.syn.rest, "basis": env.syn.basis, "names": list(env.syn.names)}


def _save(obj: dict, path: Path) -> None:
    tmp = path.with_name(path.name + ".tmp")
    torch.save(obj, tmp)
    tmp.replace(path)


def load_env_for(checkpoint: dict, **overrides: Any) -> GraspEnv:
    """A GraspEnv configured like the one a checkpoint was trained in (synergies included)."""
    from ..synergy import Synergies

    cfg = config_from_dict(checkpoint["env_config"])
    for k, v in overrides.items():
        setattr(cfg, k, v)
    env = GraspEnv(cfg)
    s = checkpoint.get("synergies")
    if s is not None:
        env.syn = Synergies(np.asarray(s["rest"]), np.asarray(s["basis"]), tuple(s["names"]))
    return env
