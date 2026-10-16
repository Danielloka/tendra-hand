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


def test_left_lite_meshes_are_reflected_right_ones():
    """Simplification is not mirror-symmetric, so the left hand's lite meshes must be the right
    ones reflected (same vertex count, positions and faces), or the physics would differ."""
    import sys

    from tendra.arm import WRIST_MODEL_PATHS
    from tendra.lite_model import MIRROR_X_MM, lite_mesh, read_stl

    root = V1.model_path.parents[2] / "hardware" / "robot_description"
    right_dir, left_dir = root / "v1_export", root / "v1_export_left"
    sys.path.insert(0, str(root.parent / "cad"))
    import mirror_export

    assert mirror_export.MIRROR_X_MM == MIRROR_X_MM
    files = sorted(p.name for p in (right_dir / "meshes").glob("*.stl"))
    assert files
    for name in files:
        rv, rf = lite_mesh(right_dir / "meshes" / name)
        lv, lf = lite_mesh(left_dir / "meshes" / name)
        assert len(lv) == len(rv) and len(lf) == len(rf), name
        assert np.allclose(lv[:, 0], 2 * MIRROR_X_MM - rv[:, 0]), name
        assert np.array_equal(lv[:, 1:], rv[:, 1:]), name
        assert np.array_equal(lf, rf[:, [0, 2, 1]]), name  # reversed winding
    # The reflection is the exported left STL (same plane): same vertices up to float32 noise.
    mirrored = mirror_export.mirror_mesh(read_stl(right_dir / "meshes" / files[0])).reshape(-1, 3)
    exported = read_stl(left_dir / "meshes" / files[0]).reshape(-1, 3)
    assert np.allclose(mirrored.min(0), exported.min(0), atol=1e-3)
    assert np.allclose(mirrored.max(0), exported.max(0), atol=1e-3)
    assert np.allclose(mirrored.mean(0), exported.mean(0), atol=1e-3)
    assert WRIST_MODEL_PATHS["left"].exists()
