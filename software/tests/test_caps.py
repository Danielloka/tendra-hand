"""CAPS smoothness loss for Brax PPO (`tendra.gpu.caps`), on the CPU.

    uv run --group gpu pytest -m gpu software/tests/test_caps.py

Fast (under a minute): a tiny network and Brax's trivial `fast` test env, no MuJoCo Warp.
"""

import pytest

jax = pytest.importorskip("jax")
pytest.importorskip("brax")

import jax.numpy as jnp
from brax import envs
from brax.training import types
from brax.training.acme import running_statistics
from brax.training.agents.ppo import losses as ppo_losses
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo
from tendra.gpu import caps as C

pytestmark = pytest.mark.gpu
OBS, ACT, B, T = 3, 2, 4, 6


def network(obs_size=OBS):
    return ppo_networks.make_ppo_networks(
        obs_size, ACT, preprocess_observations_fn=running_statistics.normalize,
        policy_hidden_layer_sizes=(8,), value_hidden_layer_sizes=(8,))  # fmt: skip


def normalizer(obs_size=OBS):
    return running_statistics.init_state(jax.ShapeDtypeStruct((obs_size,), jnp.float32))


def batch(obs, discount=None):
    """A Transition with leading dims [B, T] (as the PPO loss gets a minibatch)."""
    lead = jax.tree_util.tree_leaves(obs)[0].shape[:2]
    disc = jnp.ones(lead) if discount is None else discount
    return types.Transition(observation=obs, action=jnp.zeros((*lead, ACT)), reward=jnp.zeros(lead),
                            discount=disc, next_observation=obs, extras={})  # fmt: skip


def test_zero_for_constant_policy_positive_otherwise():
    net, norm = network(), normalizer()
    obs = jax.random.normal(jax.random.PRNGKey(0), (B, T, OBS))
    data, key = batch(obs), jax.random.PRNGKey(1)
    cfg = C.CapsConfig(temporal=1.0, spatial=1.0, sigma=0.5)
    params = net.policy_network.init(jax.random.PRNGKey(2))
    const = jax.tree_util.tree_map(jnp.zeros_like, params)  # all weights 0: mu = tanh(0) everywhere
    l_t, l_s = C.caps_terms(const, norm, data, key, net, cfg)
    assert float(l_t) == 0.0 and float(l_s) == 0.0
    l_t, l_s = C.caps_terms(params, norm, data, key, net, cfg)
    assert float(l_t) > 1e-6 and float(l_s) > 1e-6
    mu = C.policy_mean(net, norm, params, obs)
    assert mu.shape == (B, T, ACT) and float(jnp.max(jnp.abs(mu))) < 1.0  # tanh-squashed
    # Gradients reach the policy weights.
    g = jax.grad(lambda p: sum(C.caps_terms(p, norm, data, key, net, cfg)))(params)
    assert any(float(jnp.abs(x).sum()) > 0 for x in jax.tree_util.tree_leaves(g))


def test_disabled_terms_are_zero():
    net, norm = network(), normalizer()
    obs = jax.random.normal(jax.random.PRNGKey(0), (B, T, OBS))
    params = net.policy_network.init(jax.random.PRNGKey(2))
    off = C.CapsConfig(temporal=0.0, spatial=1.0, sigma=0.0)
    assert not off.enabled
    l_t, l_s = C.caps_terms(params, norm, batch(obs), jax.random.PRNGKey(1), net, off)
    assert float(l_t) == 0.0 and float(l_s) == 0.0


def test_temporal_mask_at_episode_end():
    # mu is constant within each episode and jumps only after a done step (discount 0).
    mu = jnp.zeros((1, T, ACT)).at[:, 3:].set(1.0)
    done_at_2 = jnp.ones((1, T)).at[0, 2].set(0.0)
    assert float(C.temporal_loss(mu, done_at_2)) == 0.0
    assert float(C.temporal_loss(mu, jnp.ones((1, T)))) == pytest.approx(ACT / (T - 1))
    # All pairs masked: no division by zero.
    assert float(C.temporal_loss(mu, jnp.zeros((1, T)))) == 0.0


def test_dict_observations_and_noise_scale():
    norm = running_statistics.init_state({"state": jax.ShapeDtypeStruct((OBS,), jnp.float32),
                                          "privileged_state": jax.ShapeDtypeStruct((5,), jnp.float32)})  # fmt: skip
    norm = norm.replace(std={"state": jnp.full(OBS, 4.0), "privileged_state": jnp.ones(5)})
    obs = {"state": jnp.zeros((B, T, OBS)), "privileged_state": jnp.zeros((B, T, 5))}
    noisy = C.perturb(obs, norm, 0.5, jax.random.PRNGKey(0))
    assert jnp.all(noisy["privileged_state"] == 0)  # only the policy's input is perturbed
    assert float(jnp.std(noisy["state"])) == pytest.approx(2.0, rel=0.3)  # sigma * running std


def test_ppo_train_uses_the_patch_and_restores_it():
    original = ppo_losses.compute_ppo_loss
    calls = []

    def spy(*args, **kwargs):  # what the CAPS loss wraps: proves ppo.train calls through it
        calls.append(1)
        return original(*args, **kwargs)

    ppo_losses.compute_ppo_loss = spy
    seen = {}
    try:
        with C.patched(C.CapsConfig()):
            assert ppo_losses.compute_ppo_loss._caps == C.CapsConfig()
            ppo.train(
                envs.get_environment("fast", obs_mode="dict_state"), num_timesteps=2**10,
                episode_length=16, num_envs=8, unroll_length=4, batch_size=8, num_minibatches=2,
                num_updates_per_batch=1, normalize_observations=True, num_evals=2, num_eval_envs=4,
                seed=0, progress_fn=lambda step, m: seen.update(m))  # fmt: skip
        assert ppo_losses.compute_ppo_loss is spy  # restored
    finally:
        ppo_losses.compute_ppo_loss = original
    assert calls, "ppo.train did not call the patched loss"
    assert "training/caps_temporal" in seen and "training/caps_spatial" in seen
    assert float(seen["training/caps_spatial"]) > 0


def test_patch_restored_after_error_and_not_nested():
    original = ppo_losses.compute_ppo_loss
    with pytest.raises(ValueError), C.patched(C.CapsConfig()):
        raise ValueError
    assert ppo_losses.compute_ppo_loss is original
    with C.patched(C.CapsConfig(temporal=0.0, spatial=0.0)):  # disabled: no patch at all
        assert ppo_losses.compute_ppo_loss is original
    with C.patched(C.CapsConfig()), pytest.raises(RuntimeError), C.patched(C.CapsConfig()):
        pass
    assert ppo_losses.compute_ppo_loss is original


def test_train_brax_flags():
    pytest.importorskip("mujoco_playground")
    pytest.importorskip("mujoco_warp")
    from tendra.gpu import train_brax

    a = train_brax.build_parser().parse_args(["--out", "x"])
    assert (a.caps_temporal, a.caps_spatial, a.caps_sigma) == (C.TEMPORAL, C.SPATIAL, C.SIGMA)
    a = train_brax.build_parser().parse_args(
        ["--out", "x", "--caps-temporal", "0", "--caps-sigma", "0"]
    )
    assert not C.CapsConfig(a.caps_temporal, a.caps_spatial, a.caps_sigma).enabled
