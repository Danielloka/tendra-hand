"""CAPS smoothness regularisation for Brax PPO (Mysore et al. 2021, arXiv 2012.06644).

Two extra terms in the PPO loss make the policy network itself smooth, instead of punishing
shaky motion through the reward:

- **temporal** `L_T = mean || mu(s_t) - mu(s_t+1) ||^2`: the next action should be close to this
  one. Pairs are consecutive steps of the same rollout; a pair whose first step ended the episode
  (`discount == 0`: success, failure or truncation) is masked out, because Brax's auto-reset
  wrapper puts the *reset* observation after a done step (playground 0.2.0 `BraxAutoResetWrapper`).
- **spatial** `L_S = mean || mu(s) - mu(s~) ||^2`, `s~ = s + N(0, sigma)`: a slightly noisy
  observation should give almost the same action. The noise is in **normalised** observation
  units (raw noise = sigma * the running std of each input), so one sigma fits every input
  whatever its physical unit. With `normalize_observations=False` the std stays 1 (raw units).

`mu` is the policy's deterministic action: `tanh(loc)` of Brax's `NormalTanhDistribution`, in
[-1, 1] (what `deterministic_eval` and `policy.BraxPolicy` run). `|| . ||^2` is summed over the
action dimensions and averaged over the pairs (32 actions: an RMS change of 0.1 per action gives
0.32). `loss += temporal * L_T + spatial * L_S`; the gradient only reaches the policy weights.

Hook: Brax 0.14.2's `ppo.train` builds its loss from the module attribute
`brax.training.agents.ppo.losses.compute_ppo_loss` each time it is called. `patched(cfg)` swaps in
a wrapper (original loss + CAPS) for the duration of a `with` block and restores it afterwards, so
nothing leaks into later `ppo.train` calls. The jitted training step is built inside `ppo.train`,
so no stale JIT cache keeps the old loss. The two terms come back as metrics `caps_temporal` /
`caps_spatial`, which Brax reports to `progress_fn` as `training/caps_temporal` / `..._spatial`.

The temporal pairs use `observation[t+1]` of the minibatch (Brax keeps each env's unroll in time
order; minibatches shuffle whole trajectories), which equals `next_observation[t]` when the step
was not done, and reuses the forward pass. The last step of each unroll has no pair (1 in
`unroll_length`).

Standalone (numpy/jax/brax only), shipped in the Colab kit.
"""

from __future__ import annotations

import contextlib
import functools
from collections.abc import Callable, Iterator, Mapping
from dataclasses import asdict, dataclass

import jax
import jax.numpy as jnp
from brax.training.agents.ppo import losses as ppo_losses

# Defaults (2026-10-06, research/ai/smooth-control.md): start small, sweep 0.1-0.5. Too large a
# weight gives a sluggish policy.
TEMPORAL = 0.1
SPATIAL = 0.05
SIGMA = 0.05


@dataclass(frozen=True)
class CapsConfig:
    temporal: float = TEMPORAL  # lambda_T
    spatial: float = SPATIAL  # lambda_S
    sigma: float = SIGMA  # spatial noise std, normalised observation units
    obs_key: str = "state"  # the policy's input when observations are a dict

    @property
    def use_temporal(self) -> bool:
        return self.temporal > 0

    @property
    def use_spatial(self) -> bool:
        return self.spatial > 0 and self.sigma > 0

    @property
    def enabled(self) -> bool:
        return self.use_temporal or self.use_spatial

    def to_dict(self) -> dict:
        return {**asdict(self), "enabled": self.enabled}


def policy_mean(ppo_network, normalizer_params, policy_params, obs) -> jax.Array:
    """The deterministic action `tanh(loc)`, shape `batch + (action_size,)`."""
    dist = ppo_network.parametric_action_distribution
    logits = ppo_network.policy_network.apply(normalizer_params, policy_params, obs)
    loc = dist.create_dist(logits).loc
    return dist._postprocessor.forward(loc)  # TanhBijector: tanh (brax 0.14.2 keeps it private)


def temporal_loss(mu: jax.Array, discount: jax.Array) -> jax.Array:
    """`mu` [B, T, A], `discount` [B, T] (0 where the step ended its episode)."""
    d2 = jnp.sum(jnp.square(mu[:, 1:] - mu[:, :-1]), axis=-1)
    mask = discount[:, :-1]
    return jnp.sum(d2 * mask) / jnp.maximum(jnp.sum(mask), 1.0)


def perturb(obs, normalizer_params, sigma: float, key: jax.Array, obs_key: str = "state"):
    """`obs` plus Gaussian noise of std `sigma` in normalised units (only the policy's input)."""
    if isinstance(obs, Mapping):
        x, std = obs[obs_key], normalizer_params.std[obs_key]
        noisy = x + sigma * std * jax.random.normal(key, x.shape, x.dtype)
        return {**obs, obs_key: noisy}
    std = normalizer_params.std
    return obs + sigma * std * jax.random.normal(key, obs.shape, obs.dtype)


def spatial_loss(mu: jax.Array, mu_noisy: jax.Array) -> jax.Array:
    return jnp.mean(jnp.sum(jnp.square(mu_noisy - mu), axis=-1))


def caps_terms(policy_params, normalizer_params, data, key: jax.Array, ppo_network,
               cfg: CapsConfig) -> tuple[jax.Array, jax.Array]:  # fmt: skip
    """`(L_T, L_S)` on a Brax minibatch `data` (leading dims [B, T], as the PPO loss gets it)."""
    mu = policy_mean(ppo_network, normalizer_params, policy_params, data.observation)
    zero = jnp.zeros((), mu.dtype)
    l_t = temporal_loss(mu, data.discount) if cfg.use_temporal else zero
    l_s = zero
    if cfg.use_spatial:
        noisy = perturb(data.observation, normalizer_params, cfg.sigma, key, cfg.obs_key)
        mu_noisy = policy_mean(ppo_network, normalizer_params, policy_params, noisy)
        l_s = spatial_loss(mu, mu_noisy)
    return l_t, l_s


def make_loss(original: Callable, cfg: CapsConfig) -> Callable:
    """`compute_ppo_loss` with the CAPS terms added (same signature and metrics, plus two)."""

    @functools.wraps(original)
    def loss(params, normalizer_params, data, rng, ppo_network, **kwargs):
        total, metrics = original(params, normalizer_params, data, rng, ppo_network, **kwargs)
        l_t, l_s = caps_terms(params.policy, normalizer_params, data,
                              jax.random.fold_in(rng, 0xCA95), ppo_network, cfg)  # fmt: skip
        total = total + cfg.temporal * l_t + cfg.spatial * l_s
        return total, {**metrics, "total_loss": total, "caps_temporal": l_t, "caps_spatial": l_s}

    loss._caps = cfg  # marks the patched loss (tests, and refusing to wrap twice)
    return loss


@contextlib.contextmanager
def patched(cfg: CapsConfig) -> Iterator[None]:
    """Inside the block, `ppo.train` uses the CAPS loss (no-op when `cfg` is disabled)."""
    original = ppo_losses.compute_ppo_loss
    if hasattr(original, "_caps"):
        raise RuntimeError("the PPO loss is already patched with CAPS (nested `patched`)")
    if not cfg.enabled:
        yield
        return
    ppo_losses.compute_ppo_loss = make_loss(original, cfg)
    try:
        yield
    finally:
        ppo_losses.compute_ppo_loss = original
