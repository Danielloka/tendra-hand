"""Hand synergies (tendra.synergy)."""

import numpy as np
import pytest
from tendra.joints import V1
from tendra.scene import POWER_GRASP, grasp_targets
from tendra.synergy import Synergies, default_synergies, explained_variance, fit_synergies


def test_default_synergies_span_open_to_power_grasp():
    syn = default_synergies()
    assert syn.basis.shape == (V1.num_joints, 6) and syn.names[0] == "close"
    rest = syn.posture(np.zeros(6))
    assert np.all(rest >= V1.lower) and np.all(rest <= V1.upper)
    assert np.all(rest[[V1.joint_names.index(f"{f}_mcp_flex") for f in ("index", "little")]] > 0.3)
    # "close" at +1 = the scripted power grasp (which holds the cylinder), at -1 = flat open hand.
    closed = syn.posture(np.eye(6)[0])
    flex = [i for i, n in enumerate(V1.joint_names) if not n.endswith(("abd", "rot"))]
    assert np.allclose(closed[flex], grasp_targets(POWER_GRASP)[flex], atol=1e-9)
    opened = syn.posture(-np.eye(6)[0])
    assert np.all(opened[flex] <= 0.0)


def test_project_and_residual():
    syn = default_synergies()
    a = np.array([0.3, -0.2, 0.5, 0.1, -0.1, 0.2])
    q = syn.rest + syn.basis @ a  # unclipped, inside the span
    assert np.allclose(syn.project(q), a, atol=1e-9)
    assert np.allclose(syn.residual(q), 0, atol=1e-9)
    odd = q.copy()
    odd[V1.joint_names.index("ring_dip")] += 0.5  # one joint alone: not a human pattern
    assert np.linalg.norm(syn.residual(odd)) > 0.3


def test_fit_recovers_a_low_dimensional_hand(tmp_path):
    rng = np.random.default_rng(0)
    true = default_synergies()
    coeffs = rng.normal(0, 0.4, (500, 3))
    postures = true.rest + coeffs @ true.basis[:, :3].T + rng.normal(0, 0.005, (500, 20))
    assert explained_variance(postures, 3) > 0.99
    syn = fit_synergies(postures, k=3)
    assert np.allclose(syn.rest, postures.mean(axis=0))
    assert np.abs(syn.residual(postures)).max() < 0.05  # 3 fitted components explain everything
    loaded = Synergies.load(syn.save(tmp_path / "s.npz"))
    assert np.allclose(loaded.basis, syn.basis) and loaded.names == syn.names
    with pytest.raises(ValueError):
        fit_synergies(postures[:4], k=3)
