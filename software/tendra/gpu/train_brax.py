"""Train the grasp policy on a GPU: Brax PPO on `TendraGrasp` (MuJoCo Warp), in curriculum phases.

    python train_brax.py --bundle tendra_gpu.npz --out /content/drive/MyDrive/tendra_runs/gpu1

Phases (each continues from the previous one's weights): A = cylinder only, lots of demo starts,
soft penalties; B = all objects, fewer demos, stronger penalties; C = full penalties, almost no
help. Everything is written to `--out` (put it on Google Drive): per phase `progress.csv` and
`params_latest.pkl` (saved at every evaluation), and `params_final.pkl` + `DONE` at the end.
Running the same command again resumes: finished phases are skipped, an unfinished one
continues from its last saved weights (a free Colab session can disconnect any time).

Standalone like grasp_env_jax.py (Colab gets both files, not the repo).
"""

from __future__ import annotations

import argparse
import csv
import functools
import json
import pickle
import time
from pathlib import Path

import jax
import numpy as np
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo
from mujoco_playground import wrapper

try:
    from .grasp_env_jax import TendraGrasp
except ImportError:  # run as a plain script next to grasp_env_jax.py (Colab)
    from grasp_env_jax import TendraGrasp

ALL = ["cylinder", "cube", "ball"]
PHASES = [
    {"name": "A", "steps": 30_000_000, "objects": ["cylinder"], "demo_prob": 0.5, "penalty_scale": 0.3},
    {"name": "B", "steps": 60_000_000, "objects": ALL, "demo_prob": 0.25, "penalty_scale": 0.6},
    {"name": "C", "steps": 60_000_000, "objects": ALL, "demo_prob": 0.05, "penalty_scale": 1.0},
]  # fmt: skip

PPO = {
    "num_envs": 2048, "unroll_length": 32, "batch_size": 256, "num_minibatches": 32,
    "num_updates_per_batch": 4, "discounting": 0.99, "gae_lambda": 0.95, "learning_rate": 3e-4,
    "entropy_cost": 1e-3, "clipping_epsilon": 0.2, "max_grad_norm": 1.0, "reward_scaling": 0.1,
    "normalize_observations": True, "num_evals": 20, "num_eval_envs": 256,
}  # fmt: skip
NETWORK = {"policy_hidden_layer_sizes": (256, 256, 128), "value_hidden_layer_sizes": (512, 256, 128),
           "policy_obs_key": "state", "value_obs_key": "privileged_state"}  # fmt: skip


def _save(obj: object, path: Path) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(pickle.dumps(obj))
    tmp.replace(path)


def to_numpy(tree):  # noqa: ANN001, ANN201
    return jax.tree.map(lambda x: np.asarray(x), tree)


def run_phase(phase: dict, bundle: str, out: Path, ppo_cfg: dict, start_params, log=print):  # noqa: ANN001, ANN201
    """Train one phase; returns its final params (normalizer, policy, value)."""
    pdir = out / f"phase_{phase['name']}"
    pdir.mkdir(parents=True, exist_ok=True)
    if (pdir / "DONE").exists():
        log(f"phase {phase['name']}: done already")
        return pickle.loads((pdir / "params_final.pkl").read_bytes())

    state_file = pdir / "params_latest.pkl"
    done_steps, params = 0, start_params
    if state_file.exists():
        saved = pickle.loads(state_file.read_bytes())
        done_steps, params = saved["steps"], saved["params"]
        log(f"phase {phase['name']}: resuming at {done_steps:,} steps")
    remaining = phase["steps"] - done_steps
    if remaining <= 0:
        remaining = 1

    n = ppo_cfg["num_envs"]
    overrides = {"objects": phase["objects"], "demo_prob": phase["demo_prob"],
                 "penalty_scale": phase["penalty_scale"], "naconmax": 96 * n}  # fmt: skip
    env = TendraGrasp(bundle, config_overrides=overrides)
    eval_env = TendraGrasp(bundle, config_overrides={**overrides, "demo_prob": 0.0,
                                                     "naconmax": 96 * ppo_cfg["num_eval_envs"]})  # fmt: skip
    t0 = time.time()
    csv_path = pdir / "progress.csv"
    new_file = not csv_path.exists()

    def progress(step: int, metrics: dict) -> None:
        row = {"phase": phase["name"], "steps": done_steps + step,
               "minutes": round((time.time() - t0) / 60, 1),
               **{k: round(float(v), 5) for k, v in metrics.items()
                  if k.startswith("eval/") and not k.endswith("_std")}}  # fmt: skip
        nonlocal new_file
        with open(csv_path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(row))
            if new_file:
                w.writeheader()
                new_file = False
            w.writerow(row)
        log(f"[{phase['name']}] {row['steps']:>12,} steps {row['minutes']:6.1f} min | "
            f"return {row.get('eval/episode_reward', float('nan')):8.1f} | "
            f"success {row.get('eval/episode_success', float('nan')):.2f} | "
            f"lift {row.get('eval/episode_lift_cm', float('nan')):5.2f} cm | "
            f"blowups {row.get('eval/episode_blowup', float('nan')):.3f}")  # fmt: skip

    def save_params(step: int, make_policy, p) -> None:  # noqa: ANN001
        if step > 0:
            _save({"steps": done_steps + step, "params": to_numpy(p)}, state_file)

    make_networks = functools.partial(ppo_networks.make_ppo_networks, **NETWORK)
    kwargs = {k: v for k, v in ppo_cfg.items()}
    _, final, _ = ppo.train(
        environment=env, eval_env=eval_env, num_timesteps=remaining,
        episode_length=env._config.episode_length, wrap_env_fn=wrapper.wrap_for_brax_training,
        network_factory=make_networks, progress_fn=progress, policy_params_fn=save_params,
        restore_params=params, bootstrap_on_timeout=True, seed=hash(phase["name"]) % 1000,
        **kwargs,
    )  # fmt: skip
    final = to_numpy(final)
    _save(final, pdir / "params_final.pkl")
    (pdir / "DONE").write_text("ok")
    return final


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bundle", default="tendra_gpu.npz")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--scale", type=float, default=1.0, help="multiply every phase's steps")
    p.add_argument("--envs", type=int, default=PPO["num_envs"])
    p.add_argument("--smoke", action="store_true", help="tiny run to check the setup")
    args = p.parse_args()

    ppo_cfg = {**PPO, "num_envs": args.envs}
    phases = [{**ph, "steps": int(ph["steps"] * args.scale)} for ph in PHASES]
    if args.smoke:
        ppo_cfg.update(num_envs=8, batch_size=8, num_minibatches=2, unroll_length=4,
                       num_evals=2, num_eval_envs=4, num_updates_per_batch=1)  # fmt: skip
        phases = [{**ph, "steps": 300} for ph in PHASES[:2]]
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "config.json").write_text(json.dumps({"phases": phases, "ppo": ppo_cfg,
                                                      "network": NETWORK}, indent=2))  # fmt: skip
    print(f"device {jax.devices()[0]}, {ppo_cfg['num_envs']} envs, output {args.out}")
    params = None
    for phase in phases:
        params = run_phase(phase, args.bundle, args.out, ppo_cfg, params)
    _save(params, args.out / "params_final.pkl")
    print(f"finished: {args.out / 'params_final.pkl'}")


if __name__ == "__main__":
    main()
