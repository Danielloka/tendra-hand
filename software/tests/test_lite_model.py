"""The lightweight model must behave exactly like the full one, just with fewer triangles."""

import mujoco
import numpy as np
import pytest
from tendra import SimHand
from tendra.joints import V0, V1
from tendra.lite_model import load_lite_model

pytest.importorskip("fast_simplification")


@pytest.fixture(scope="module", params=[V0, V1], ids=["v0", "v1"])
def models(request):
    path = request.param.model_path
    return mujoco.MjModel.from_xml_path(str(path)), load_lite_model(path)


def test_fewer_triangles_same_structure(models):
    full, lite = models
    assert lite.nmeshface < 0.3 * full.nmeshface
    for attr in ("nq", "nu", "nbody", "njnt", "ntendon", "nsite"):
        assert getattr(lite, attr) == getattr(full, attr), attr
    assert np.array_equal(lite.body_mass, full.body_mass)
    assert np.array_equal(lite.body_inertia, full.body_inertia)


def test_same_kinematics(models):
    full, lite = models
    rng = np.random.default_rng(0)
    d_full, d_lite = mujoco.MjData(full), mujoco.MjData(lite)
    for _ in range(5):
        q = rng.uniform(full.jnt_range[:, 0], full.jnt_range[:, 1])
        for m, d in ((full, d_full), (lite, d_lite)):
            d.qpos[:] = q
            mujoco.mj_kinematics(m, d)
        assert np.allclose(d_lite.xpos, d_full.xpos, atol=1e-9)
        assert np.allclose(d_lite.site_xpos, d_full.site_xpos, atol=1e-9)


def test_sim_hand_lite():
    sim = SimHand(hand="v0", lite=True)
    sim.set_targets(np.full(sim.num_joints, 0.3))
    sim.wait()
    assert np.allclose(sim.positions(), sim.targets(), atol=0.05)
