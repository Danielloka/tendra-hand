"""Staggered episode starts for Brax PPO (Playground wrappers).

Problem: all envs reset together, so every rollout batch sees the same slice of an episode (all
envs at step 0..31, then 32..63, ...) and all episodes end on the same step. A paper on massively
parallel PPO (arXiv 2511.21011, ManiSkill) reports 2-3x faster convergence just by starting the
envs at different points of the episode, with PPO itself unchanged.

How it works here (brax 0.14.2 / playground 0.2.0): `EpisodeWrapper` keeps a per-env step counter
`state.info["steps"]` and ends the episode (truncation) when it reaches `episode_length`;
`BraxAutoResetWrapper` zeroes it again after a reset. `StaggeredResets` sits outermost and, on
`reset` only, writes a random counter in `[0, episode_length - min_left)` into `info["steps"]`.
The physics state is still a fresh start; only the *first* episode of each env is cut short
(it ends after `episode_length - offset` steps). From then on every env runs full-length
episodes, shifted against the others for good, so each batch mixes all episode phases. Brax PPO
calls `reset` again after each evaluation epoch when `num_resets_per_eval > 0`, which re-draws
the offsets. The first short episodes are about 1 in 10 of the samples (with ~10 episodes per
epoch); that is the price, and why the default is a uniform offset, not a bigger one.

Not applied to the evaluation env (evaluation needs complete episodes): mark it with
`eval_env.no_stagger = True`, `wrap_for_training` below checks that.
"""

from __future__ import annotations

from typing import Any

import jax
from mujoco_playground import wrapper


class StaggeredResets(wrapper.Wrapper):
    """Random initial `info["steps"]` per env at reset (module docs)."""

    def __init__(self, env: Any, episode_length: int, min_left: int = 8) -> None:
        super().__init__(env)
        self._episode_length = episode_length
        self._max_offset = max(1, episode_length - min_left)

    def reset(self, rng: jax.Array):
        state = self.env.reset(rng)
        keys = jax.vmap(lambda k: jax.random.fold_in(k, 0x57A6))(rng)
        offset = jax.vmap(lambda k: jax.random.randint(k, (), 0, self._max_offset))(keys)
        steps = state.info["steps"]
        state.info["steps"] = offset.astype(steps.dtype).reshape(steps.shape)
        return state


def wrap_for_training(env: Any, episode_length: int = 1000, action_repeat: int = 1,
                      randomization_fn=None, full_reset: bool = False,
                      stagger: bool = True) -> Any:  # fmt: skip
    """Drop-in for `wrapper.wrap_for_brax_training`; staggers unless the env has `no_stagger`."""
    wrapped = wrapper.wrap_for_brax_training(
        env, episode_length=episode_length, action_repeat=action_repeat,
        randomization_fn=randomization_fn, full_reset=full_reset)  # fmt: skip
    if stagger and not getattr(env, "no_stagger", False):
        wrapped = StaggeredResets(wrapped, episode_length)
    return wrapped
