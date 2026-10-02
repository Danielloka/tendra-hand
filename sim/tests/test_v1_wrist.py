"""Tests for the V1 hand on its forearm with twist and wrist (sim/convert_v1_wrist.py).

The key property: the strands cross the wrist in PTFE sheaths through the wrist centre, so turning
the forearm or bending the wrist never changes a strand's length, and the fingers don't move.
"""

import itertools
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import convert_v1_wrist as cw

ARM = list(cw.ARM_JOINTS)
X, Y, Z = np.eye(3)


@pytest.fixture(scope="module")
def hand():
    return mujoco.MjModel.from_xml_path(str(cw.IN_PATH))


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(str(cw.OUT_PATH))


def _q(model, data, **angles):
    data.qpos[:] = 0
    for name, q in angles.items():
        data.qpos[model.joint(name).qposadr[0]] = q
    mujoco.mj_forward(model, data)


def test_generated_model_is_up_to_date():
    fresh = ET.tostring(cw.build(), encoding="unicode")
    assert fresh in cw.OUT_PATH.read_text(encoding="utf-8"), (
        "Re-run: uv run python sim/convert_v1_wrist.py"
    )


def test_counts_and_order(model, hand):
    assert model.njnt == hand.njnt + 3 and model.nu == hand.nu + 3
    assert model.ntendon == hand.ntendon and model.neq == hand.neq
    assert [model.joint(i).name for i in range(3)] == ARM  # the arm joints come first in qpos
    # The servo actuators keep their order (= servo IDs); the arm's are appended.
    assert [model.actuator(i).name for i in range(hand.nu)] == [
        hand.actuator(i).name for i in range(hand.nu)
    ]
    assert [model.actuator(hand.nu + i).name for i in range(3)] == ARM


def test_axes_and_signs(model):
    """forearm_rot about the forearm's long axis (+ = pronation of this right hand: the thumb side
    turns toward the palm side), wrist_flex closes toward the palm, wrist_dev toward the thumb."""
    data = mujoco.MjData(model)
    _q(model, data)
    for name, axis in (("forearm_rot", -Z), ("wrist_flex", X), ("wrist_dev", Y)):
        assert np.allclose(data.xaxis[model.joint(name).id], axis)
    # Both wrist axes and the twist axis cross at the wrist centre: twisting doesn't move it.
    centre = data.site("wrist_centre").xpos.copy()
    for name in ARM:
        assert np.allclose(data.xanchor[model.joint(name).id][:2], centre[:2])
    tip0 = data.site("middle_tip").xpos.copy()
    for name, q, direction in (("wrist_flex", 0.3, -Y), ("wrist_dev", 0.2, X)):
        _q(model, data, **{name: q})
        assert np.dot(data.site("middle_tip").xpos - tip0, direction) > 0.01, name
    _q(model, data)
    thumb0 = data.site("thumb_tip").xpos.copy()
    _q(model, data, forearm_rot=0.3)
    assert np.dot(data.site("thumb_tip").xpos - thumb0, -Y) > 0.005


def test_strands_keep_their_length_at_every_wrist_angle(model):
    data = mujoco.MjData(model)
    _q(model, data)
    lengths0 = data.ten_length.copy()
    ranges = [model.jnt_range[model.joint(n).id] for n in ARM]
    for q in itertools.product(*(np.linspace(lo, hi, 4) for lo, hi in ranges)):
        _q(model, data, **dict(zip(ARM, q, strict=True)))
        assert np.abs(data.ten_length - lengths0).max() < 1e-9


def test_every_servo_strand_goes_through_the_wrist_centre(model):
    centre = model.site("wrist_centre").id
    servo_strands = {model.actuator(i).trnid[0] for i in range(model.nu) if
                     model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_TENDON}  # fmt: skip
    servo_strands |= {model.tendon(model.tendon(t).name.removesuffix("_flex") + "_ext").id
                      for t in list(servo_strands)}  # fmt: skip
    for t in servo_strands:
        adr, num = model.tendon_adr[t], model.tendon_num[t]
        sites = [model.wrap_objid[adr + i] for i in range(num)
                 if model.wrap_type[adr + i] == mujoco.mjtWrap.mjWRAP_SITE]  # fmt: skip
        assert sites.count(centre) == 1, model.tendon(t).name


def test_no_contacts_over_the_wrist_range(model):
    """Known limit: above ~55 deg of flexion with radial deviation, the thumb base hits the
    forearm (the servo box, 91 x 85 mm, is wider than a human forearm). A real collision, kept."""
    data = mujoco.MjData(model)
    flex = (model.jnt_range[model.joint("wrist_flex").id][0], np.radians(50))
    for q in itertools.product(flex, model.jnt_range[model.joint("wrist_dev").id]):
        _q(model, data, **dict(zip(ARM[1:], q, strict=True)))
        assert data.ncon == 0, q


def _settle(model, ctrl: dict[str, float], seconds: float = 1.5) -> np.ndarray:
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("open").id)
    for name, value in ctrl.items():
        data.ctrl[model.actuator(name).id] = value
    mujoco.mj_step(model, data, round(seconds / model.opt.timestep))
    return data.qpos.copy()


def test_open_hand_stays_straight(model):
    qpos = _settle(model, {})
    assert np.degrees(np.abs(qpos).max()) < 1.5  # gravity on the wrist placeholder servos only


def test_wrist_motion_does_not_move_a_gripping_hand(model):
    """Close the fingers, then bend the wrist and turn the forearm: the finger angles stay."""
    fist = {"index_mcp_flex": 1.2, "index_pip": 1.3, "middle_mcp_flex": 1.2, "middle_pip": 1.3,
            "thumb_cmc_flex": 0.5}  # fmt: skip
    still = _settle(model, fist)
    moved = _settle(model, fist | {"forearm_rot": 1.0, "wrist_flex": 0.8, "wrist_dev": -0.3})
    fingers = [model.joint(n).qposadr[0] for n in fist]
    assert np.abs(moved[fingers] - still[fingers]).max() < np.radians(1.0)
    arm = [model.joint(n).qposadr[0] for n in ARM]
    assert np.allclose(moved[arm], [1.0, 0.8, -0.3], atol=np.radians(3))
