"""Sanity checks for the generated MuJoCo model. Run: uv run pytest"""

import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import convert

MOTOR_ORDER = [
    "index_dip",
    "index_pip",
    "index_mcp_flex",
    "index_mcp_abd",
    "thumb_ip",
    "thumb_mcp",
    "thumb_cmc_flex",
    "thumb_cmc_rot",
]


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(str(convert.OUT_PATH))


def test_generated_model_is_up_to_date():
    links, joints = convert.parse_urdf(convert.URDF_PATH)
    import xml.etree.ElementTree as ET

    fresh = convert.build_mjcf(links, joints)
    ET.indent(fresh)
    on_disk = convert.OUT_PATH.read_text(encoding="utf-8")
    assert ET.tostring(fresh, encoding="unicode") in on_disk, "Re-run: uv run python sim/convert.py"


def test_actuators_follow_motor_order(model):
    assert [model.actuator(i).name for i in range(model.nu)] == MOTOR_ORDER


def test_mass_is_plausible_for_pla(model):
    total_g = model.body_mass.sum() * 1000
    assert 80 < total_g < 200, total_g  # the raw export (steel) was 821 g


def test_no_collisions_when_straight(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.ncon == 0


@pytest.mark.parametrize("finger", ["index", "thumb"])
def test_flexion_moves_tip_toward_palm(model, finger):
    """Positive flexion must move the fingertip toward the palm side (-Y) or toward the index."""
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    start = data.site(f"{finger}_tip").xpos.copy()
    for name in MOTOR_ORDER:
        if name.startswith(finger) and (
            "flex" in name or name.endswith(("dip", "pip", "ip", "mcp"))
        ):
            data.joint(name).qpos = np.radians(30)
    mujoco.mj_kinematics(model, data)
    moved = data.site(f"{finger}_tip").xpos - start
    assert (moved[1] < 0) if finger == "index" else (moved[1] > 0)


def test_actuators_reach_targets(model):
    data = mujoco.MjData(model)
    target = np.radians([60, 60, 45, 10, 45, 45, 30, 20])
    data.ctrl[:] = target
    for _ in range(1500):
        mujoco.mj_step(model, data)
    reached = np.array([data.joint(n).qpos[0] for n in MOTOR_ORDER])
    assert np.isfinite(data.qpos).all()
    assert np.degrees(np.abs(reached - target)).max() < 2.0
