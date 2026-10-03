"""Run a Brax PPO policy (trained on the GPU, see `train_brax.py`) with numpy only.

The laptop needs neither jax nor brax: `BraxPolicy.load` reads `params_final.pkl` /
`params_latest.pkl` (what `train_brax.py` writes) or a small `.npz` made by `export_npz`, and
reproduces Brax's deterministic inference exactly:

* `brax.training.acme.running_statistics.normalize`: `(obs - mean) / std` with the *stored* `std`
  (already `sqrt(var + std_eps)` clipped by Brax, so nothing to add here) and no clipping (PPO
  calls it with `max_abs_value=None`). With dict observations the statistics are per key, and
  `brax.training.networks.make_policy_network` picks `obs[policy_obs_key]` and that key's
  statistics (`normalizer_select`).
* `brax.training.networks.MLP`: Dense layers `hidden_0 .. hidden_N`, an activation after every
  layer except the last (default `linen.swish` in `ppo.networks.make_ppo_networks`), no layer
  norm. The last layer outputs `2 * action_size` numbers: `loc` and a raw scale.
* `brax.training.distribution.NormalTanhDistribution`: the deterministic action is
  `tanh(loc)` (`ParametricDistribution.mode`); sampling uses
  `tanh(loc + (softplus(scale) + 0.001) * N(0, 1))`.

Requirement for the pickles: the arrays inside should be numpy arrays (`train_brax.py` converts
them with `to_numpy`). The brax / flax wrapper classes around them (`RunningStatisticsState`,
`UInt64`) need not be installed: `load_params_pickle` unpickles them as inert stubs, and jax
arrays (the normaliser's counter stays one after `to_numpy`) are rebuilt as numpy arrays from
their pickled bytes. Only `mean`, `std` and the policy's kernels / biases are used. For a file
you do not trust, run `export_npz` on a machine you do trust: the `.npz` needs no pickle.

Command line: `python -m tendra.gpu.policy params_final.pkl policy.npz`.
"""

from __future__ import annotations

import io
import json
import pickle
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np

MIN_STD = 0.001  # NormalTanhDistribution(min_std=0.001, var_scale=1)

ACTIVATIONS = {
    "swish": lambda x: x / (1.0 + np.exp(-x)),
    "relu": lambda x: np.maximum(x, 0.0),
    "tanh": np.tanh,
}


class _Stub:
    """Stands in for a brax / flax class while unpickling: keeps the attributes, nothing else."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        self.__dict__.update(kwargs)

    def __setstate__(self, state: Any) -> None:
        if isinstance(state, Mapping):
            self.__dict__.update(state)
        elif isinstance(state, tuple) and len(state) == 2 and isinstance(state[1], Mapping):
            self.__dict__.update(state[1])


class _Unpickler(pickle.Unpickler):
    _STUB_ROOTS = ("brax", "flax", "jax", "jaxlib", "ml_dtypes", "chex")

    def find_class(self, module: str, name: str) -> Any:
        if module.split(".")[0] in self._STUB_ROOTS:
            if name == "_reconstruct_array":
                return _reconstruct_jax_array
            return type(name, (_Stub,), {})
        return super().find_class(module, name)


def _reconstruct_jax_array(fun: Any, args: Any, arr_state: Any, *_: Any) -> np.ndarray:
    """jax pickles an Array as numpy's rebuild call + state; do just that, without jax. (The
    normaliser's sample counter stays a jax array even after `to_numpy`, so real files have them.)
    """
    a = fun(*args)
    a.__setstate__(arr_state)
    return a


def load_params_pickle(path: str | Path) -> Any:
    """The object in a Brax params pickle, with brax / flax classes replaced by stubs."""
    return _Unpickler(io.BytesIO(Path(path).read_bytes())).load()


def _get(obj: Any, key: str) -> Any:
    return obj[key] if isinstance(obj, Mapping) else getattr(obj, key)


def _to_array(x: Any) -> np.ndarray:
    a = np.asarray(x)
    if a.dtype == object:
        raise ValueError("parameters are not plain arrays (jax arrays pickled without numpy?)")
    return a.astype(np.float64)


class BraxPolicy:
    """Observation in, action in [-1, 1] out; same call signature as `tendra.rl.ppo.Policy`."""

    def __init__(self, mean: np.ndarray, std: np.ndarray, kernels: list[np.ndarray],
                 biases: list[np.ndarray], obs_key: str = "state", activation: str = "swish",
                 steps: int | None = None) -> None:  # fmt: skip
        if activation not in ACTIVATIONS:
            raise ValueError(f"unknown activation {activation!r}; known: {sorted(ACTIVATIONS)}")
        self.mean, self.std = mean, std
        self.kernels, self.biases = kernels, biases
        self.obs_key, self.activation, self.steps = obs_key, activation, steps
        self.obs_dim = kernels[0].shape[0]
        out = kernels[-1].shape[1]
        if out % 2 or mean.shape != (self.obs_dim,) or std.shape != (self.obs_dim,):
            raise ValueError(f"inconsistent shapes: obs {mean.shape}, output {out}")
        self.act_dim = out // 2

    # ----- loading -----

    @classmethod
    def load(cls, path: str | Path, obs_key: str = "state",
             activation: str = "swish") -> BraxPolicy:  # fmt: skip
        """From `params_*.pkl` (Brax) or `.npz` (`export_npz`)."""
        path = Path(path)
        if path.suffix == ".npz":
            return cls.from_npz(path)
        return cls.from_params(load_params_pickle(path), obs_key, activation)

    @classmethod
    def from_params(cls, saved: Any, obs_key: str = "state",
                    activation: str = "swish") -> BraxPolicy:  # fmt: skip
        """From Brax's `(normalizer_params, policy_params, value_params)`, or the dict
        `{"steps", "params"}` that `train_brax.py` saves as `params_latest.pkl`."""
        steps = None
        if isinstance(saved, Mapping) and "params" in saved:
            steps, saved = saved.get("steps"), saved["params"]
        if isinstance(saved, Mapping):  # {"normalizer": ..., "policy": ..., "value": ...}
            normalizer, policy = saved["normalizer"], saved["policy"]
        else:
            normalizer, policy = saved[0], saved[1]
        mean, std = _get(normalizer, "mean"), _get(normalizer, "std")
        if isinstance(mean, Mapping):  # dict observations: statistics per key
            mean, std = mean[obs_key], std[obs_key]
        layers = policy.get("params", policy)
        names = sorted(layers, key=lambda n: int(n.rsplit("_", 1)[1]))
        return cls(_to_array(mean), _to_array(std),
                   [_to_array(layers[n]["kernel"]) for n in names],
                   [_to_array(layers[n]["bias"]) for n in names],
                   obs_key, activation, steps)  # fmt: skip

    @classmethod
    def from_npz(cls, path: str | Path) -> BraxPolicy:
        with np.load(path) as f:
            meta = json.loads(bytes(f["meta"]).decode())
            n = meta["layers"]
            return cls(f["mean"].astype(np.float64), f["std"].astype(np.float64),
                       [f[f"kernel_{i}"].astype(np.float64) for i in range(n)],
                       [f[f"bias_{i}"].astype(np.float64) for i in range(n)],
                       meta["obs_key"], meta["activation"], meta.get("steps"))  # fmt: skip

    def save_npz(self, path: str | Path) -> None:
        meta = {"layers": len(self.kernels), "obs_key": self.obs_key,
                "activation": self.activation, "steps": self.steps}  # fmt: skip
        arrays = {"mean": self.mean, "std": self.std,
                  "meta": np.frombuffer(json.dumps(meta).encode(), np.uint8)}  # fmt: skip
        for i, (k, b) in enumerate(zip(self.kernels, self.biases, strict=True)):
            arrays[f"kernel_{i}"], arrays[f"bias_{i}"] = k, b
        np.savez_compressed(path, **arrays)

    # ----- inference -----

    def _logits(self, obs: np.ndarray) -> np.ndarray:
        x = (np.asarray(obs, np.float64) - self.mean) / self.std
        act = ACTIVATIONS[self.activation]
        last = len(self.kernels) - 1
        for i, (k, b) in enumerate(zip(self.kernels, self.biases, strict=True)):
            x = x @ k + b
            if i < last:
                x = act(x)
        return x

    def __call__(self, obs: Any, deterministic: bool = True,
                 rng: np.random.Generator | None = None) -> np.ndarray:  # fmt: skip
        if isinstance(obs, Mapping):
            obs = obs[self.obs_key]
        if np.shape(obs)[-1] != self.obs_dim:
            raise ValueError(
                f"the policy expects {self.obs_dim} observation values, got {np.shape(obs)[-1]}"
                ": the environment does not match the one it was trained in"
            )
        loc, raw_scale = np.split(self._logits(obs), 2, axis=-1)
        if not deterministic:
            rng = rng or np.random.default_rng()
            scale = np.logaddexp(raw_scale, 0.0) + MIN_STD  # softplus
            loc = loc + scale * rng.standard_normal(loc.shape)
        return np.tanh(loc).astype(np.float32)


def export_npz(src: str | Path | Any, npz: str | Path, obs_key: str = "state",
               activation: str = "swish") -> BraxPolicy:  # fmt: skip
    """Convert Brax params to the plain `.npz` format (safe to share and load). `src` is a
    pickle path (Brax tuple, `{"steps", "params"}`, or the dict layout below) or an
    already-loaded params object / dict."""
    if isinstance(src, str | Path):
        policy = BraxPolicy.load(src, obs_key, activation)
    else:
        policy = BraxPolicy.from_params(src, obs_key, activation)
    policy.save_npz(npz)
    return policy


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    p = export_npz(sys.argv[1], sys.argv[2])
    print(f"{sys.argv[2]}: obs {p.obs_dim}, actions {p.act_dim}, layers {len(p.kernels)}")
