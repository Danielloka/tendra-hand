"""BraxPolicy: Brax PPO inference in numpy (see tendra/gpu/policy.py). No brax needed, except in
the last test, which compares against the real thing when brax is installed."""

import pickle
from dataclasses import dataclass
from itertools import pairwise

import numpy as np
import pytest
from tendra.gpu.policy import BraxPolicy, export_npz

OBS, ACT = 7, 3
SIZES = (OBS, 16, 8, 2 * ACT)


@dataclass
class FakeStats:  # shaped like brax's RunningStatisticsState (mean, std, ...)
    mean: object
    std: object
    count: int = 5


def make_params(seed=0, keyed=False):
    rng = np.random.default_rng(seed)
    layers = {
        f"hidden_{i}": {"kernel": rng.normal(size=(a, b)) * 0.5, "bias": rng.normal(size=b) * 0.1}
        for i, (a, b) in enumerate(pairwise(SIZES))
    }
    mean, std = rng.normal(size=OBS), rng.uniform(0.5, 2.0, size=OBS)
    if keyed:
        mean, std = (
            {"state": mean, "privileged_state": mean[:3]},
            {"state": std, "privileged_state": std[:3]},
        )
    return (FakeStats(mean, std), {"params": layers}, {"params": {}})


def reference(params, obs):
    stats, policy = params[0], params[1]["params"]
    mean, std = stats.mean, stats.std
    if isinstance(mean, dict):
        mean, std = mean["state"], std["state"]
    x = (obs - mean) / std
    for i in range(len(SIZES) - 1):
        x = x @ policy[f"hidden_{i}"]["kernel"] + policy[f"hidden_{i}"]["bias"]
        if i < len(SIZES) - 2:
            x = x * (1 / (1 + np.exp(-x)))
    return np.tanh(x[:ACT])


@pytest.mark.parametrize("keyed", [False, True])
def test_matches_reference(keyed):
    params = make_params(keyed=keyed)
    pol = BraxPolicy.from_params(params)
    assert (pol.obs_dim, pol.act_dim) == (OBS, ACT)
    for obs in np.random.default_rng(1).normal(size=(5, OBS)):
        assert np.allclose(pol(obs), reference(params, obs), atol=1e-6)
        assert np.allclose(pol({"state": obs, "privileged_state": obs[:3]}), pol(obs))


def test_normalisation_uses_mean_and_std():
    params = make_params()
    pol = BraxPolicy.from_params(params)
    obs = np.random.default_rng(2).normal(size=OBS)
    plain = BraxPolicy.from_params(params)
    plain.mean, plain.std = np.zeros(OBS), np.ones(OBS)
    stats = params[0]
    assert np.allclose(pol(obs), plain((obs - stats.mean) / stats.std), atol=1e-6)
    assert not np.allclose(pol(obs), plain(obs), atol=1e-3)


def test_batch_and_stochastic():
    pol = BraxPolicy.from_params(make_params())
    obs = np.random.default_rng(3).normal(size=(4, OBS))
    a = pol(obs)
    assert a.shape == (4, ACT) and np.all(np.abs(a) <= 1)
    s = pol(obs, deterministic=False, rng=np.random.default_rng(0))
    assert s.shape == a.shape and not np.allclose(s, a)


def test_wrong_observation_size_is_explained():
    pol = BraxPolicy.from_params(make_params())
    with pytest.raises(ValueError, match="expects 7"):
        pol(np.zeros(OBS + 1))


def test_latest_dict_and_pickle_roundtrip(tmp_path):
    params = make_params(keyed=True)
    pkl = tmp_path / "params_latest.pkl"
    pkl.write_bytes(pickle.dumps({"steps": 123, "params": params}))
    pol = BraxPolicy.load(pkl)
    assert pol.steps == 123
    npz = tmp_path / "p.npz"
    export_npz(pkl, npz)
    back = BraxPolicy.load(npz)
    obs = np.random.default_rng(4).normal(size=(6, OBS))
    assert np.array_equal(pol(obs), back(obs))
    assert back.steps == 123 and back.obs_key == "state"


def test_named_dict_layout_and_export_from_object(tmp_path):
    stats, policy, value = make_params(keyed=True)
    saved = {"steps": 5, "success": 0.5, "params": {
        "normalizer": {"mean": stats.mean, "std": stats.std, "count": np.array(3)},
        "policy": policy, "value": value}}  # fmt: skip
    pol = BraxPolicy.from_params(saved)
    assert pol.steps == 5
    export_npz(saved, tmp_path / "d.npz")
    obs = np.random.default_rng(6).normal(size=OBS)
    assert np.array_equal(BraxPolicy.load(tmp_path / "d.npz")(obs), pol(obs))
    assert np.allclose(pol(obs), reference((stats, policy), obs), atol=1e-6)


def test_unknown_classes_are_stubbed(tmp_path):
    """A pickle whose normaliser class (brax's) is not importable still loads."""
    import sys
    import types

    mod = types.ModuleType("brax.fake_stats")
    sys.modules["brax"] = types.ModuleType("brax")
    sys.modules["brax.fake_stats"] = mod
    mod.RunningStatisticsState = FakeStats
    FakeStats.__module__, FakeStats.__qualname__ = "brax.fake_stats", "RunningStatisticsState"
    try:
        data = pickle.dumps(make_params())
    finally:
        del sys.modules["brax.fake_stats"], sys.modules["brax"]
    (tmp_path / "p.pkl").write_bytes(data)
    assert BraxPolicy.load(tmp_path / "p.pkl").obs_dim == OBS


def test_matches_real_brax(tmp_path):
    pytest.importorskip("brax")
    import functools

    import jax
    import jax.numpy as jnp
    from brax.training.acme import running_statistics
    from brax.training.agents.ppo import networks as ppo_networks

    size = {"state": OBS, "privileged_state": 5}
    nets = ppo_networks.make_ppo_networks(
        size, ACT, preprocess_observations_fn=running_statistics.normalize,
        policy_hidden_layer_sizes=(16, 8), value_hidden_layer_sizes=(8, 8),
        policy_obs_key="state", value_obs_key="privileged_state",
    )  # fmt: skip
    k1, k2, k3 = jax.random.split(jax.random.PRNGKey(0), 3)
    spec = jax.tree.map(lambda n: jnp.zeros(n), size)
    norm = running_statistics.init_state(spec)
    data = jax.random.normal(k3, (64, OBS)) * 3 + 1
    norm = running_statistics.update(norm, {"state": data, "privileged_state": data[:, :5]})
    params = (norm, nets.policy_network.init(k1), nets.value_network.init(k2))
    params = jax.tree.map(np.asarray, params)
    # make the output layer non-trivial so tanh(loc) is not ~0
    params[1]["params"]["hidden_2"]["kernel"] = params[1]["params"]["hidden_2"]["kernel"] * 20
    pkl = tmp_path / "params.pkl"
    pkl.write_bytes(pickle.dumps(params))
    pol = BraxPolicy.load(pkl)
    infer = functools.partial(ppo_networks.make_inference_fn(nets), deterministic=True)(params)
    obs = np.asarray(jax.random.normal(jax.random.PRNGKey(5), (32, OBS)) * 3 + 1, np.float32)
    want = np.asarray(
        infer(
            {"state": jnp.asarray(obs), "privileged_state": jnp.zeros((32, 5))},
            jax.random.PRNGKey(0),
        )[0]
    )
    assert np.abs(want).max() > 0.3
    diff = np.abs(pol(obs) - want).max()
    print("max abs diff vs brax:", diff)
    assert diff < 1e-4
