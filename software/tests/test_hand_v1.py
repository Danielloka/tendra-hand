"""Tests for hand variants and the v1 (20 servos) protocol. Run: uv run pytest"""

import math
import re

import numpy as np
import pytest
from tendra import V0, V1, HandError, RealHand, SimHand, get_hand
from tendra.fake_esp32 import SERVO_TICKS_PER_RAD, FakeEsp32
from tendra.joints import JOINT_LIMITS, JOINT_NAMES, MODEL_PATH, NUM_JOINTS
from tendra.real_hand import parse_info

needs_v1_model = pytest.mark.skipif(
    not V1.model_path.exists(), reason=f"{V1.model_path.name} not generated yet"
)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def esp(clock):
    return FakeEsp32(clock=clock, hand="v1")


@pytest.fixture
def real(esp):
    return RealHand(connection=esp)


# ----- the variant table -----


def test_v0_spec_keeps_the_old_constants():
    assert V0.joint_names == JOINT_NAMES
    assert V0.num_joints == NUM_JOINTS == 8
    assert V0.joint_limits == JOINT_LIMITS
    assert V0.model_path == MODEL_PATH
    assert not V0.has_feedback


def test_get_hand():
    assert get_hand("v1") is V1
    assert get_hand("V0") is V0
    assert get_hand(V1) is V1
    with pytest.raises(ValueError):
        get_hand("v3")


def test_v1_spec_basics():
    assert V1.num_joints == 20
    assert V1.servo_ids == tuple(range(1, 21))
    assert V1.has_feedback
    assert V1.joint_index("little_mcp_abd") == 19
    with pytest.raises(ValueError):
        V1.joint_index("thumb_mcp")  # v0 name; on v1 it is thumb_mcp_flex


def test_v1_firmware_config_matches_python():
    config = V1.firmware_config.read_text(encoding="utf-8")
    assert f"constexpr int kNumJoints = {V1.num_joints};" in config
    assert f'#define HAND_VARIANT "{V1.firmware_variant}"' in config
    rows = re.findall(r'\{"(\w+)",\s*(\d+),\s*(-?\d+) \* kDegToRad,\s*(-?\d+) \* kDegToRad', config)
    assert tuple(name for name, *_ in rows) == V1.joint_names
    assert tuple(int(sid) for _, sid, _, _ in rows) == V1.servo_ids
    for (_, _, lo, hi), (lo_rad, hi_rad) in zip(rows, V1.joint_limits, strict=True):
        assert math.radians(int(lo)) == pytest.approx(lo_rad)
        assert math.radians(int(hi)) == pytest.approx(hi_rad)


@needs_v1_model
def test_v1_sim_model_matches_python():
    hand = SimHand(hand="v1")
    for name, (lo, hi) in zip(V1.joint_names, V1.joint_limits, strict=True):
        assert hand.model.joint(name).range == pytest.approx([lo, hi], abs=1e-5)


@needs_v1_model
def test_v1_sim_hand_follows_targets():
    hand = SimHand(hand="v1")
    assert hand.num_joints == 20
    q = np.clip(np.radians(np.full(20, 30.0)), V1.lower, V1.upper)
    hand.set_targets(q)
    hand.wait()
    assert np.degrees(np.abs(hand.positions() - q)).max() < 3.0


def test_sim_hand_detects_variant_from_model():
    assert SimHand(MODEL_PATH).spec is V0
    with pytest.raises(ValueError):
        SimHand(MODEL_PATH, hand="v1")


def test_sim_set_positions_mirrors_and_skips_nan():
    hand = SimHand()
    q = np.radians([60, 45, 30, 10, 45, 30, 20, 20])
    q_nan = q.copy()
    q_nan[2] = np.nan
    hand.set_positions(q_nan)
    expected = q.copy()
    expected[2] = 0.0
    assert hand.positions() == pytest.approx(expected)
    assert hand.targets() == pytest.approx(expected)


# ----- parsing the real firmware's I line -----


def test_parse_info_v0_firmware():
    spec, joints = parse_info("I tendra-hand fw 0.1.0 joints=8 index_dip:648.72 index_pip:648.72")
    assert spec is V0
    assert joints[0].name == "index_dip" and joints[0].scale == pytest.approx(648.72)
    assert joints[0].zero_ticks is None


def test_parse_info_v1_firmware():
    spec, joints = parse_info(
        "I tendra-hand fw 0.3.0 hand=v1-servo joints=20 index_dip:195.57:512:1 index_pip:195.57:498:0"
    )
    assert spec is V1
    assert (joints[1].zero_ticks, joints[1].online) == (498, False)


def test_parse_info_rejects_unknown_variant():
    with pytest.raises(HandError):
        parse_info("I tendra-hand fw 9.0 hand=v9-laser joints=99")


# ----- FakeEsp32 v1 speaks the v1 protocol -----


def ask(esp: FakeEsp32, line: str) -> str:
    esp.write((line + "\n").encode())
    return esp.readline().decode().strip()


def test_fake_v1_replies(esp):
    info = ask(esp, "I").split()
    assert info[:6] == ["I", "tendra-hand", "fw", "0.3.0", "hand=v1-servo", "joints=20"]
    assert info[6] == f"index_dip:{SERVO_TICKS_PER_RAD:.2f}:512:1"
    status = ask(esp, "S").split()
    assert len(status) == 22 and status[-1] == "0"
    feedback = ask(esp, "F").split()
    assert len(feedback) == 21 and feedback[1] == "0.0000,0.0,30,5.0"
    assert ask(esp, "B") == "B " + " ".join(str(i) for i in range(1, 21))
    assert ask(esp, "P " + " ".join(["0.1"] * 20)) == "OK"
    assert ask(esp, "P 0.1 0.2") == "ERR P needs 20 values"
    assert ask(esp, "J 21 0.1").startswith("ERR bad command")


def test_fake_v1_offline_servos(clock):
    esp = FakeEsp32(clock=clock, hand="v1", offline={9, "little_mcp_abd"})
    assert ask(esp, "P " + " ".join(["0.1"] * 20)) == f"OK offline {(1 << 8) | (1 << 19)}"
    assert ask(esp, "J 9 0.1") == "ERR joint offline"
    assert ask(esp, "M 20 10") == "ERR joint offline"
    assert ask(esp, "F").split()[9] == "nan,0,0,0"
    assert "9" not in ask(esp, "B").split()
    assert ask(esp, "I").split()[6 + 8].endswith(":0")


def test_fake_v0_has_no_bus_and_no_feedback(clock):
    esp = FakeEsp32(clock=clock)
    assert ask(esp, "B").startswith("ERR")
    assert ask(esp, "F") == "F " + " ".join(["nan,0,0,0"] * 8)


# ----- RealHand v1 over the fake ESP32 -----


def test_auto_detects_v1(real):
    assert real.spec is V1
    assert real.num_joints == 20
    assert "hand=v1-servo" in real.firmware_info


def test_auto_detects_v0(clock):
    assert RealHand(connection=FakeEsp32(clock=clock)).spec is V0


def test_wrong_variant_is_refused(clock):
    with pytest.raises(HandError, match="expected hand v0"):
        RealHand(connection=FakeEsp32(clock=clock, hand="v1"), hand="v0")
    assert RealHand(connection=FakeEsp32(clock=clock, hand="v1"), hand="v1").spec is V1


def test_v1_moves_to_targets_over_time(real, clock):
    q = np.clip(np.radians(np.linspace(0, 60, 20)), V1.lower, V1.upper)
    real.set_targets(q)
    assert real.offline_mask == 0
    assert real.is_moving()
    clock.t += 5.0
    assert real.positions() == pytest.approx(q, abs=1e-3)
    assert not real.is_moving()
    with pytest.raises(ValueError):
        real.set_targets(np.zeros(8))  # a v0-sized command


def test_v1_targets_are_clamped(real, clock):
    real.set_joint("little_mcp_abd", 1.0)  # limit is 20 deg
    clock.t += 5.0
    assert real.positions()[19] == pytest.approx(math.radians(20), abs=1e-3)


def test_v1_limp_joint_is_measured_then_held(real, esp, clock):
    esp.move_by_hand("middle_pip", 0.5)  # limp at start: moved by hand
    assert real.positions()[9] == pytest.approx(0.5)
    real.set_joint("index_dip", 0.2)  # sends all targets; middle_pip's target is still 0
    clock.t += 5.0
    assert real.positions()[9] == pytest.approx(0.0, abs=1e-3)
    with pytest.raises(RuntimeError):
        esp.move_by_hand("middle_pip", 0.3)  # torque on now
    real.release()
    esp.move_by_hand("middle_pip", 0.3)
    assert real.positions()[9] == pytest.approx(0.3)


def test_v1_feedback(real, esp):
    esp.load[3] = -12.5
    esp.set_online("ring_pip", False)
    fb = real.feedback()
    assert len(fb) == 20 and fb[3].name == "index_mcp_abd"
    assert fb[3].load == pytest.approx(-12.5)
    assert fb[3].temperature == 30 and fb[3].voltage == pytest.approx(5.0)
    assert not fb[13].online and math.isnan(fb[13].position)
    online = real.online()
    assert online.sum() == 19 and not online[13]
    assert math.isnan(real.measured_positions()[13])


def test_v1_offline_servos(clock):
    esp = FakeEsp32(clock=clock, hand="v1", offline={5})
    real = RealHand(connection=esp)
    real.set_targets(np.full(20, 0.3))
    assert real.offline_mask == 1 << 4
    clock.t += 5.0
    assert real.positions()[4] == 0.0  # didn't move
    assert real.positions()[0] == pytest.approx(0.3, abs=1e-3)
    with pytest.raises(HandError, match="offline"):
        real.command("J 5 0.1")
    assert real.scan_bus() == [i for i in range(1, 21) if i != 5]
    info = real.info()
    assert info[4].online is False and info[0].online is True
    esp.set_online(5)
    real.set_targets(np.full(20, 0.3))
    assert real.offline_mask == 0


def test_v1_calibration(real, esp, clock):
    real.raw_move("index_dip", 1000)  # clamped to 300 ticks
    clock.t += 5.0
    assert real.positions()[0] == pytest.approx(300 / SERVO_TICKS_PER_RAD, abs=1e-3)
    real.zero()
    assert real.positions() == pytest.approx(np.zeros(20))
    assert real.info()[0].zero_ticks == 512 + 300
    real.raw_move("index_dip", 100)
    clock.t += 5.0
    real.set_scale("index_dip", 2 * SERVO_TICKS_PER_RAD)  # same ticks, half the angle
    assert real.positions()[0] == pytest.approx(50 / SERVO_TICKS_PER_RAD, abs=1e-3)


def test_v0_has_no_servo_bus(clock):
    real = RealHand(connection=FakeEsp32(clock=clock))
    with pytest.raises(HandError):
        real.scan_bus()
    assert not real.online().any()
