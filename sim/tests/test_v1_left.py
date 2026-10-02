"""The left hand (and its wrist): the exact mirror image of the right one.

`hardware/cad/mirror_export.py` reflects the Fusion export; `sim/convert_v1.py --side left` and
`sim/convert_v1_wrist.py --side left` build the models with the same code as the right hand.
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "hardware" / "cad"))
import convert_v1 as cv
import convert_v1_wrist as cw
import mirror_export as me

X0 = me.MIRROR_X_MM / 1000  # the mirror plane x = X0 (m)
TIPS = ("index_tip", "middle_tip", "ring_tip", "little_tip", "thumb_tip")


def mirror(p: np.ndarray) -> np.ndarray:
    return np.array([2 * X0 - p[0], p[1], p[2]])


def mirror_dir(v: np.ndarray) -> np.ndarray:
    return np.array([-v[0], v[1], v[2]])


@pytest.fixture(scope="module")
def hands():
    return {s: mujoco.MjModel.from_xml_path(str(cv.OUT_PATHS[s])) for s in cv.SIDES}


@pytest.fixture(scope="module")
def wrists():
    return {s: mujoco.MjModel.from_xml_path(str(cw.OUT_PATHS[s])) for s in cw.SIDES}


def test_left_export_is_up_to_date():
    assert me.stale() == [], "Re-run: uv run python hardware/cad/mirror_export.py"


def test_mirrored_meshes_keep_their_volume_and_outward_normals():
    for name in ("palm", "forearm", "index_distal", "thumb_metacarpal"):
        right = me.read_stl(me.SRC / "meshes" / f"{name}.stl")
        left = me.read_stl(me.DST / "meshes" / f"{name}.stl")

        def volume(t):
            return float(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6)

        assert volume(left) == pytest.approx(volume(right), rel=1e-6) and volume(left) > 0
        assert np.allclose(left[..., 0], 2 * me.MIRROR_X_MM - right[:, [0, 2, 1], 0], atol=1e-3)


def test_generated_models_are_up_to_date():
    for side in cv.SIDES:
        fresh = ET.tostring(cv.build(side=side), encoding="unicode")
        assert fresh in cv.OUT_PATHS[side].read_text(encoding="utf-8"), (
            "Re-run: uv run python sim/convert_v1.py"
        )
        fresh = ET.tostring(cw.build(side=side), encoding="unicode")
        assert fresh in cw.OUT_PATHS[side].read_text(encoding="utf-8"), (
            "Re-run: uv run python sim/convert_v1_wrist.py"
        )


def test_same_structure_limits_and_mass(hands, wrists):
    for models in (hands, wrists):
        r, left = models["right"], models["left"]
        assert (r.nq, r.nu, r.ntendon, r.neq) == (left.nq, left.nu, left.ntendon, left.neq)
        assert [r.joint(i).name for i in range(r.njnt)] == [left.joint(i).name for i in range(left.njnt)]
        assert [r.actuator(i).name for i in range(r.nu)] == [left.actuator(i).name for i in range(left.nu)]
        assert np.allclose(r.jnt_range, left.jnt_range)
        assert np.allclose(r.actuator_ctrlrange, left.actuator_ctrlrange)
        assert np.allclose(r.actuator_forcerange, left.actuator_forcerange)
        assert r.body_subtreemass[0] == pytest.approx(left.body_subtreemass[0], rel=1e-4)


@pytest.mark.parametrize("which", ["hands", "wrists"])
def test_every_pose_is_the_mirror_image(which, hands, wrists):
    """The same joint angles put every point at the mirror image, and every strand has the same
    length (so the same servo angles do the same thing)."""
    models = hands if which == "hands" else wrists
    r, left = models["right"], models["left"]
    rd, ld = mujoco.MjData(r), mujoco.MjData(left)
    rng = np.random.default_rng(3)
    sites = TIPS + (("wrist_centre",) if which == "wrists" else ())
    for _ in range(15):
        q = rng.uniform(r.jnt_range[:, 0], r.jnt_range[:, 1])
        rd.qpos[:], ld.qpos[:] = q, q
        mujoco.mj_forward(r, rd)
        mujoco.mj_forward(left, ld)
        for s in sites:
            assert np.allclose(ld.site(s).xpos, mirror(rd.site(s).xpos), atol=1e-6), s
        assert np.abs(ld.ten_length - rd.ten_length).max() < 1e-6


def test_positive_closes_on_both_hands(hands):
    """A small positive rotation moves each fingertip to the mirror image of where the right
    hand's goes: closing is positive on both hands."""
    r, left = hands["right"], hands["left"]
    rd, ld = mujoco.MjData(r), mujoco.MjData(left)
    for j in range(r.njnt):
        name = r.joint(j).name
        finger = name.split("_")[0]
        rd.qpos[:], ld.qpos[:] = 0, 0
        mujoco.mj_forward(r, rd)
        mujoco.mj_forward(left, ld)
        before_r, before_l = rd.site(f"{finger}_tip").xpos.copy(), ld.site(f"{finger}_tip").xpos.copy()
        rd.qpos[r.joint(name).qposadr[0]] = 0.05
        ld.qpos[left.joint(name).qposadr[0]] = 0.05
        mujoco.mj_forward(r, rd)
        mujoco.mj_forward(left, ld)
        move_r = rd.site(f"{finger}_tip").xpos - before_r
        move_l = ld.site(f"{finger}_tip").xpos - before_l
        assert np.allclose(move_l, mirror_dir(move_r), atol=1e-6), name
        assert np.linalg.norm(move_r) > 1e-3


def test_left_wrist_axes_mean_the_same_things(wrists):
    """Pronation, flexion toward the palm and deviation toward the thumb, in the hand's own terms."""
    left = wrists["left"]
    d = mujoco.MjData(left)

    def tip_move(joint: str, q: float, site: str) -> np.ndarray:
        d.qpos[:] = 0
        mujoco.mj_forward(left, d)
        before = d.site(site).xpos.copy()
        d.qpos[left.joint(joint).qposadr[0]] = q
        mujoco.mj_forward(left, d)
        return d.site(site).xpos - before

    assert tip_move("wrist_flex", 0.2, "middle_tip")[1] < -0.01  # toward the palm side (-Y)
    assert tip_move("wrist_dev", 0.2, "middle_tip")[0] < -0.01  # toward the thumb: -X on the left
    right = mujoco.MjModel.from_xml_path(str(cw.OUT_PATHS["right"]))
    rd = mujoco.MjData(right)
    rd.qpos[right.joint("wrist_dev").qposadr[0]] = 0.2
    mujoco.mj_forward(right, rd)
    assert rd.site("middle_tip").xpos[0] > 0.0  # the right hand's thumb side is +X
