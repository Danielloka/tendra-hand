"""Train the grasp policy on a GPU: Brax PPO on `TendraGrasp` (MuJoCo Warp), adaptive curriculum.

    python train_brax.py --bundle tendra_gpu.npz --out /content/drive/MyDrive/tendra/runs/gpu3

How it works (details in each module):
- **Chunks.** Training runs in chunks of a few million steps (`--chunk-steps`), each a normal
  `ppo.train` call that restores the previous weights. After each chunk `curriculum.Curriculum`
  reads the evaluation success and picks the next chunk's settings (add objects, fewer demo
  starts, stronger penalties, wider randomisation): see `curriculum.py`. Every decision is logged
  and saved in `curriculum.json`.
- **Staggered resets** (`stagger.py`, on by default, `--no-stagger` to turn off): envs start at
  different points of the episode, so rollout batches are not all the same slice of time.
- **PPO settings** follow MuJoCo Playground's tuned dexterous-hand configs (LeapCubeReorient in
  `mujoco_playground/config/manipulation_params.py`): see `PPO` / `POLICY_HIDDEN` below.
- **Colab robustness**: `--envs` is picked from the GPU (override with the flag; halves itself on
  an out-of-memory error); the JAX compilation cache and Warp's kernel cache live on Drive so a
  reconnect skips the long first compile; every evaluation saves the weights atomically; running
  the same command again resumes the exact chunk; `--hours` stops cleanly before Colab's limit; a
  watchdog rolls back to the last good weights (once with half the learning rate) when an
  evaluation returns NaN or the physics blows up.

Outputs in `--out`: `progress.csv` (one file for the whole run), `curriculum.json`,
`params_latest.pkl` / `params_best.pkl` (plain numpy pickles `{"steps", "success", "params"}` that
`policy.BraxPolicy` loads without jax), `policy_latest.npz` / `policy_best.npz` (for the laptop),
`config.json`, and `DONE` when the curriculum finished.

Standalone like grasp_env_jax.py (Colab gets the kit files, not the repo).
"""

from __future__ import annotations

import argparse
import csv
import functools
import json
import math
import os
import pickle
import shutil
import sys
import tempfile
import time
from dataclasses import asdict
from pathlib import Path

# Set before jax/warp are imported (the cache and precision settings below read them).
os.environ.setdefault("JAX_DEFAULT_MATMUL_PRECISION", "highest")  # TF32 hurt Playground results

import jax
import numpy as np
from brax.training import networks as brax_networks
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo
from flax import linen

try:
    from .curriculum import Curriculum, Rules
    from .grasp_env_jax import TendraGrasp, default_config
    from .policy import export_npz
    from .stagger import wrap_for_training
except ImportError:  # run as a plain script next to the other files (Colab)
    from curriculum import Curriculum, Rules
    from grasp_env_jax import TendraGrasp, default_config
    from policy import export_npz
    from stagger import wrap_for_training

# Playground LeapCubeReorient (16-DoF hand, 8192 envs): unroll 40, 32 minibatches of 256
# trajectories, 4 epochs, gamma 0.99, lr 3e-4, hidden (512, 256, 128) for policy and critic,
# asymmetric critic ("privileged_state"). Ours: lr/gamma/GAE/clip/net as there; entropy_cost stays
# at 1e-3 (the earlier T4 run reached 77% with it; Leap uses 1e-2: try `--entropy 1e-2` if the
# policy collapses to a narrow behaviour); reward_scaling 0.1 because our rewards are summed
# larger (success +30); num_resets_per_eval=1 draws fresh start states every evaluation epoch
# (without it each env slot restarts from the same cached state for the whole chunk).
PPO = {
    "unroll_length": 40, "batch_size": 256, "num_minibatches": 32, "num_updates_per_batch": 4,
    "discounting": 0.99, "gae_lambda": 0.95, "learning_rate": 3e-4, "entropy_cost": 1e-3,
    "clipping_epsilon": 0.2, "max_grad_norm": 1.0, "reward_scaling": 0.1,
    "normalize_observations": True, "num_resets_per_eval": 1,
}  # fmt: skip
POLICY_HIDDEN = (512, 256, 128)
VALUE_HIDDEN = (512, 256, 128)
MIN_ENVS = 512


class OutOfTime(Exception):
    """The time budget is used up (raised after a checkpoint, so nothing is lost)."""


class Rollback(Exception):
    """The watchdog saw NaN or a physics blow-up in an evaluation."""


# ----- runtime: GPU, caches, networks -----


def gpu_info() -> dict:
    d = jax.devices()[0]
    mem = None
    try:
        mem = (d.memory_stats() or {}).get("bytes_limit")
    except Exception as e:  # noqa: BLE001 - not every backend has stats
        print(f"no GPU memory stats ({e})")
    return {"platform": d.platform, "name": getattr(d, "device_kind", str(d)),
            "memory_gb": None if mem is None else round(mem / 2**30, 1)}  # fmt: skip


def pick_num_envs(info: dict) -> int:
    """Parallel worlds for this GPU: 2048 on a 16 GB T4, 4096 on a 24 GB L4, 8192 on 40+ GB.
    Must divide batch_size * num_minibatches (8192). Too many only costs an OOM retry."""
    mem = info.get("memory_gb") or 0
    if info["platform"] != "gpu" or mem < 18:
        return 2048 if info["platform"] == "gpu" else 8
    return 4096 if mem < 30 else 8192


def _mirror(src: Path, dst: Path) -> None:
    """Copy files that are missing or differ in size from `src` to `dst` (recursive)."""
    if not src.exists():
        return
    for f in src.rglob("*"):
        if f.is_file():
            t = dst / f.relative_to(src)
            if not t.exists() or t.stat().st_size != f.stat().st_size:
                t.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(f, t)


class Caches:
    """JAX compilation cache + Warp kernel cache: a local copy (fast) mirrored to `--out/cache`
    (Drive), so a reconnect does not pay the ~5 min first compile again."""

    def __init__(self, remote: Path, local: Path) -> None:
        self.remote, self.local = remote, local
        for sub in ("jax", "warp"):
            _mirror(remote / sub, local / sub)
            (local / sub).mkdir(parents=True, exist_ok=True)
        os.environ["WARP_CACHE_PATH"] = str(local / "warp")
        jax.config.update("jax_compilation_cache_dir", str(local / "jax"))
        jax.config.update("jax_persistent_cache_min_compile_time_secs", 2.0)
        try:
            import warp as wp

            wp.config.kernel_cache_dir = str(local / "warp")
            wp.build.init_kernel_cache(str(local / "warp"))
        except Exception as e:  # noqa: BLE001 - the cache is an optimisation only
            print(f"warp kernel cache not set ({e})")

    def push(self) -> None:
        try:
            for sub in ("jax", "warp"):
                _mirror(self.local / sub, self.remote / sub)
        except OSError as e:  # Drive hiccup: not worth stopping training for
            print(f"cache sync failed ({e})")


def make_networks(obs_size, action_size, preprocess_observations_fn,
                  critic_layer_norm: bool = True) -> ppo_networks.PPONetworks:  # fmt: skip
    """`make_ppo_networks` with an optional LayerNorm critic (brax 0.14.2 does not expose it).
    The policy stays a plain swish MLP (what `policy.BraxPolicy` runs on the laptop); the
    critic is only used for training, so LayerNorm there costs nothing at deployment."""
    from brax.training import distribution

    dist = distribution.NormalTanhDistribution(event_size=action_size)
    policy = brax_networks.make_policy_network(
        dist.param_size, obs_size, preprocess_observations_fn=preprocess_observations_fn,
        hidden_layer_sizes=POLICY_HIDDEN, activation=linen.swish, obs_key="state",
        distribution_type="tanh_normal")  # fmt: skip
    if not critic_layer_norm:
        value = brax_networks.make_value_network(
            obs_size, preprocess_observations_fn=preprocess_observations_fn,
            hidden_layer_sizes=VALUE_HIDDEN, activation=linen.swish, obs_key="privileged_state")  # fmt: skip
        return ppo_networks.PPONetworks(policy, value, dist)

    module = brax_networks.MLP(layer_sizes=[*VALUE_HIDDEN, 1], activation=linen.swish,
                               kernel_init=jax.nn.initializers.lecun_uniform(), layer_norm=True)  # fmt: skip
    key = "privileged_state"

    def apply(processor_params, value_params, obs):
        x = preprocess_observations_fn(obs[key], brax_networks.normalizer_select(
            processor_params, key))  # fmt: skip
        return module.apply(value_params, x)[..., 0]

    size = brax_networks._get_obs_state_size(obs_size, key)
    dummy = jax.numpy.zeros((1, size))
    value = brax_networks.FeedForwardNetwork(init=lambda k: module.init(k, dummy), apply=apply)
    return ppo_networks.PPONetworks(policy, value, dist)


def to_numpy(tree):
    return jax.tree.map(lambda x: np.asarray(x), tree)


def _save(obj: object, path: Path) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(pickle.dumps(obj))
    tmp.replace(path)


def _load(path: Path):
    return pickle.loads(path.read_bytes())


def _is_oom(e: BaseException) -> bool:
    s = str(e).lower()
    return "resource_exhausted" in s or "out of memory" in s or "cuda_error_out_of_memory" in s


# ----- the trainer -----


class Trainer:
    def __init__(self, args: argparse.Namespace, out: Path, rules: Rules, ppo_cfg: dict,
                 num_envs: int, num_eval_envs: int, caches: Caches | None = None,
                 log=print) -> None:  # fmt: skip
        self.args, self.out, self.log, self.caches = args, out, log, caches
        self.ppo_cfg, self.num_envs, self.num_eval_envs = ppo_cfg, num_envs, num_eval_envs
        self.has_dr = "dr_scale" in default_config()
        self.cur = Curriculum.load_or_new(out / "curriculum.json", rules, self.has_dr, log)
        self.deadline = time.time() + args.hours * 3600
        self.last_eval_time = time.time()
        self.eval_interval = 0.0  # seconds between evaluations, measured
        self.prev_blowup = 0.0
        self.params = None
        latest = out / "params_latest.pkl"
        if latest.exists():
            saved = _load(latest)
            self.params = saved["params"]
            st = self.cur.state
            st.total_steps = max(st.total_steps, saved["steps"])  # a crash between the two saves
            log(f"resuming from {latest.name} at {saved['steps']:,} steps")
        self._csv_fields: list[str] | None = None
        if (out / "progress.csv").exists():
            with open(out / "progress.csv", newline="") as f:
                self._csv_fields = next(csv.reader(f), None)

    # --- logging ---

    def _write_row(self, row: dict) -> None:
        path = self.out / "progress.csv"
        if self._csv_fields is None:
            self._csv_fields = list(row)
        with open(path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=self._csv_fields, extrasaction="ignore", restval="")
            if f.tell() == 0:
                w.writeheader()
            w.writerow(row)

    # --- one chunk ---

    def env_overrides(self, n: int) -> dict:
        s = self.cur.state
        ov = {"objects": list(s.objects), "demo_prob": s.demo_prob,
              "penalty_scale": s.penalty_scale, "naconmax": 96 * n}  # fmt: skip
        if self.has_dr:
            ov["dr_scale"] = s.dr_scale
        return ov

    def run_chunk(self) -> None:
        """Train the rest of the current chunk (resumes a partly done one)."""
        st, a = self.cur.state, self.args
        remaining = max(1, a.chunk_steps - st.chunk_steps_done)
        base_steps = st.total_steps - st.chunk_steps_done
        num_evals = max(2, round(remaining / a.eval_every) + 1)
        ov = self.env_overrides(self.num_envs)
        env = TendraGrasp(a.bundle, config_overrides=ov)
        eval_env = TendraGrasp(a.bundle, config_overrides={
            **ov, "demo_prob": 0.0, "naconmax": 96 * self.num_eval_envs})  # fmt: skip
        eval_env.no_stagger = True
        wrap = functools.partial(wrap_for_training, stagger=not a.no_stagger)
        cfg = {**self.ppo_cfg, "learning_rate": self.ppo_cfg["learning_rate"] * st.lr_scale}
        self.log(f"chunk {st.chunk}: {remaining:,} steps, {self.num_envs} envs, settings "
                 f"{st.settings()}, lr {cfg['learning_rate']:.2e}")  # fmt: skip
        pending: list = []
        t0 = time.time()

        def policy_params(step, make_policy, p) -> None:
            if step > 0:
                pending[:] = [step, to_numpy(p)]

        def progress(step: int, metrics: dict) -> None:
            m = {k: float(v) for k, v in metrics.items()
                 if k.startswith("eval/") and not k.endswith("_std")}  # fmt: skip
            row = {"chunk": st.chunk, "kind": "start" if step == 0 else "eval",
                   "steps": base_steps + step, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "minutes": round((time.time() - t0) / 60, 1), "n_objects": len(st.objects),
                   "demo_prob": st.demo_prob, "penalty_scale": st.penalty_scale,
                   "dr_scale": st.dr_scale, "lr": cfg["learning_rate"],
                   **{k: round(v, 5) for k, v in m.items()}}  # fmt: skip
            success = m.get("eval/episode_success", float("nan"))
            blowup = m.get("eval/episode_blowup", 0.0)
            self.log(f"[{st.chunk}] {row['steps']:>12,} steps {row['minutes']:6.1f} min | "
                     f"return {m.get('eval/episode_reward', float('nan')):8.1f} | "
                     f"success {success:.2f} | lift {m.get('eval/episode_lift_cm', 0.0):5.2f} cm "
                     f"| blowups {blowup:.3f} | {m.get('eval/sps', 0.0):,.0f} sps"
                     f"{' (start)' if step == 0 else ''}")  # fmt: skip
            self._write_row(row)
            if step == 0:
                return
            bad = self.watchdog(m)
            if bad:
                raise Rollback(bad)
            self.prev_blowup = blowup
            steps_total = base_steps + step
            record = {"steps": steps_total, "success": success, "chunk": st.chunk,
                      "level": list(st.level())}  # fmt: skip
            assert pending and pending[0] == step, "policy_params_fn must precede progress_fn"
            record["params"] = pending[1]
            _save(record, self.out / "params_latest.pkl")
            if self.cur.record_best(success):
                _save(record, self.out / "params_best.pkl")
                self.log(f"new best at this level: success {success:.2f}")
            self.params = pending[1]
            st.total_steps = steps_total
            self.cur.record_eval(step, success)  # saves curriculum.json
            now = time.time()
            self.eval_interval = now - self.last_eval_time
            self.last_eval_time = now
            if self.caches:
                self.caches.push()
            if now + 1.1 * self.eval_interval > self.deadline:
                raise OutOfTime()

        ppo.train(
            environment=env, eval_env=eval_env, num_timesteps=remaining,
            episode_length=env._config.episode_length, wrap_env_fn=wrap,
            network_factory=functools.partial(make_networks,
                                              critic_layer_norm=not self.args.plain_critic),
            progress_fn=progress, policy_params_fn=policy_params, restore_params=self.params,
            seed=a.seed + 1000 * st.chunk + st.rollbacks, num_evals=num_evals,
            num_eval_envs=self.num_eval_envs, deterministic_eval=True,
            num_envs=self.num_envs, **cfg)  # fmt: skip

    def watchdog(self, m: dict) -> str:
        """A reason to roll back, or ''."""
        keys = ("eval/episode_reward", "eval/episode_success", "eval/episode_lift_cm")
        for k in keys:
            if k in m and not math.isfinite(m[k]):
                return f"{k} is {m[k]}"
        blowup = m.get("eval/episode_blowup", 0.0)
        if blowup > max(0.25, 4 * self.prev_blowup):
            return f"blow-ups jumped to {blowup:.2f} per episode (was {self.prev_blowup:.2f})"
        return ""

    # --- the loop ---

    def run(self) -> str:
        """Train until done / out of steps / out of time. Returns why it stopped."""
        st, a = self.cur.state, self.args
        while True:
            if st.done:
                return "curriculum finished"
            if st.total_steps >= a.max_steps:
                return f"step budget reached ({st.total_steps:,})"
            if time.time() + a.min_chunk_minutes * 60 > self.deadline and not st.chunk_steps_done:
                return "time budget used up"
            try:
                self.run_chunk()
            except OutOfTime:
                return "time budget used up (stopped after a checkpoint)"
            except Rollback as e:
                self.rollback(str(e))
                continue
            except Exception as e:
                if _is_oom(e) and self.num_envs > MIN_ENVS:
                    self.num_envs //= 2
                    self.log(f"out of GPU memory: retrying with {self.num_envs} envs")
                    continue
                raise
            self.cur.decide(self.log)

    def rollback(self, reason: str) -> None:
        st = self.cur.state
        st.rollbacks += 1
        self.cur.note("rollback", self.log, reason=reason, rollbacks=st.rollbacks)
        if st.rollbacks > self.args.max_rollbacks:
            self.cur.save()
            raise SystemExit(
                f"stopping: {st.rollbacks} rollbacks. Last: {reason}. "
                "The weights in params_latest.pkl are the last good ones."
            )
        if st.rollbacks == 1:  # lower the learning rate once
            st.lr_scale *= 0.5
        latest = self.out / "params_latest.pkl"
        if latest.exists():
            saved = _load(latest)
            self.params = saved["params"]
            st.total_steps = saved["steps"]
            if saved["chunk"] != st.chunk:  # the good weights are from the chunk before
                st.chunk_steps_done, st.chunk_scores = 0, []
        else:
            self.params = None
            st.total_steps, st.chunk_steps_done, st.chunk_scores = 0, 0, []
        self.prev_blowup = 0.0
        self.cur.save()


# ----- command line -----


def versions() -> dict:
    import brax
    import mujoco

    out = {"jax": jax.__version__, "brax": getattr(brax, "__version__", "?"),
           "mujoco": mujoco.__version__}  # fmt: skip
    try:
        import warp

        out["warp"] = warp.__version__
    except ImportError:
        pass
    return out


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--bundle", default="tendra_gpu.npz")
    p.add_argument("--out", type=Path, required=True, help="run folder (on Drive for Colab)")
    p.add_argument("--envs", type=int, default=0, help="parallel worlds (0 = pick from the GPU)")
    p.add_argument("--eval-envs", type=int, default=512)
    p.add_argument("--chunk-steps", type=int, default=8_000_000, help="steps per curriculum chunk")
    p.add_argument("--eval-every", type=int, default=2_000_000, help="steps between evaluations")
    p.add_argument("--max-steps", type=int, default=400_000_000)
    p.add_argument("--hours", type=float, default=11.0, help="stop cleanly after this long")
    p.add_argument("--min-chunk-minutes", type=float, default=20.0,
                   help="do not start a new chunk with less time than this left")  # fmt: skip
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--entropy", type=float, default=PPO["entropy_cost"])
    p.add_argument("--lr", type=float, default=PPO["learning_rate"])
    p.add_argument("--no-stagger", action="store_true", help="all envs reset together")
    p.add_argument("--plain-critic", action="store_true", help="no LayerNorm in the critic")
    p.add_argument("--max-rollbacks", type=int, default=3)
    p.add_argument("--rules", default="{}", help='JSON overrides of curriculum.Rules, e.g. '
                   '\'{"add_object_at": 0.7}\'')  # fmt: skip
    p.add_argument("--allow-cpu", action="store_true", help="run without a GPU (very slow)")
    p.add_argument("--smoke", action="store_true", help="tiny run to check the setup (CPU ok)")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    info = gpu_info()
    if info["platform"] != "gpu" and not (args.allow_cpu or args.smoke):
        print(
            "No GPU found. On Colab: Runtime > Change runtime type > T4 GPU (or L4 / A100), "
            "then run again. (--allow-cpu forces a very slow CPU run.)",
            file=sys.stderr,
        )
        return 2

    rules = Rules(**json.loads(args.rules))
    ppo_cfg = {**PPO, "entropy_cost": args.entropy, "learning_rate": args.lr}
    num_envs, eval_envs = args.envs or pick_num_envs(info), args.eval_envs
    if args.smoke:
        ppo_cfg.update(batch_size=8, num_minibatches=2, unroll_length=4, num_updates_per_batch=1)
        num_envs, eval_envs = 8, 4
        args.chunk_steps, args.eval_every, args.max_steps = 256, 128, 1024
        rules = Rules(add_object_at=0.0, ramp_at=0.0, demo_final=0.1, score_evals=1)
    batch_envs = ppo_cfg["batch_size"] * ppo_cfg["num_minibatches"]
    if batch_envs % num_envs:  # Brax needs batch_size * num_minibatches to be a multiple of envs
        if num_envs % ppo_cfg["num_minibatches"]:
            raise SystemExit(
                f"--envs {num_envs} must divide {batch_envs} or be a multiple of "
                f"{ppo_cfg['num_minibatches']}"
            )
        ppo_cfg["batch_size"] = num_envs // ppo_cfg["num_minibatches"]

    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    local_cache = Path(
        os.environ.get("TENDRA_LOCAL_CACHE", Path(tempfile.gettempdir()) / "tendra_cache")
    )
    caches = Caches(out / "cache", local_cache)
    (out / "config.json").write_text(json.dumps({
        "args": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        "ppo": ppo_cfg, "policy_hidden": POLICY_HIDDEN, "value_hidden": VALUE_HIDDEN,
        "rules": asdict(rules), "gpu": info, "versions": versions(),
        "num_envs": num_envs}, indent=2))  # fmt: skip
    print(f"device {info}, {num_envs} envs (eval {eval_envs}), output {out}")

    trainer = Trainer(args, out, rules, ppo_cfg, num_envs, eval_envs, caches)
    why = trainer.run()
    caches.push()
    st = trainer.cur.state
    trainer.cur.note("stop", print, why=why)
    trainer.cur.save()
    for name in ("best", "latest"):
        pkl = out / f"params_{name}.pkl"
        if pkl.exists():
            try:
                export_npz(pkl, out / f"policy_{name}.npz")
            except Exception as e:  # noqa: BLE001 - the pickle is still there
                print(f"npz export of {pkl.name} failed: {e}")
    if st.done:
        (out / "DONE").write_text("ok")
    print(f"stopped: {why}. {st.total_steps:,} steps, chunk {st.chunk}, settings {st.settings()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
