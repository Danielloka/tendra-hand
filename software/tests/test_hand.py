"""Tests for the tendra package. Run: uv run pytest"""

import math
import re

import numpy as np
import pytest
from tendra import JOINT_NAMES, HandError, RealHand, SimHand
from tendra.fake_esp32 import FakeEsp32
from tendra.joints import JOINT_LIMITS, REPO_ROOT


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def real(clock):
    return RealHand(connection=FakeEsp32(clock=clock))


# ----- one source of truth: Python, firmware and sim must agree -----


def test_firmware_config_matches_python():
    config = (REPO_ROOT / "firmware" / "include" / "config.h").read_text(encoding="utf-8")
    rows = re.findall(
        r'\{"(\w+)",\s*\{[\d, ]+\},\s*(-?\d+) \* kDegToRad,\s*(-?\d+) \* kDegToRad', config
    )
    assert tuple(name for name, _, _ in rows) == JOINT_NAMES
    for (_, lo, hi), (lo_rad, hi_rad) in zip(rows, JOINT_LIMITS, strict=True):
        assert math.radians(int(lo)) == pytest.approx(lo_rad)
        assert math.radians(int(hi)) == pytest.approx(hi_rad)


def test_sim_model_matches_python():
    hand = SimHand()
    for name, (lo, hi) in zip(JOINT_NAMES, JOINT_LIMITS, strict=True):
        assert hand.model.joint(name).range == pytest.approx([lo, hi], abs=1e-5)


# ----- RealHand over the fake ESP32 -----


def test_connects_and_reads_info(real):
    assert real.firmware_info.startswith("I tendra-hand")


def test_moves_to_targets_over_time(real, clock):
    q = np.radians([60, 45, 30, 10, 45, 30, 20, 20])
    real.set_targets(q)
    assert real.is_moving()
    clock.t += 5.0
    assert real.positions() == pytest.approx(q, abs=1e-3)
    assert not real.is_moving()


def test_targets_are_clamped_to_joint_limits(real, clock):
    real.set_joint("index_mcp_abd", 1.0)  # limit is 15 deg
    clock.t += 5.0
    assert real.positions()[3] == pytest.approx(math.radians(15), abs=1e-3)


def test_rejects_wrong_input(real):
    with pytest.raises(ValueError):
        real.set_targets([0.1, 0.2])
    with pytest.raises(ValueError):
        real.set_targets([float("nan")] * 8)
    with pytest.raises(HandError):
        real.command("J 9 0.1")  # there is no joint 9


def test_calibration_commands(real, clock):
    real.set_scale("index_dip", 1000.0)
    real.raw_move("index_dip", 500)  # 500 steps at 1000 steps/rad = 0.5 rad
    clock.t += 5.0
    assert real.positions()[0] == pytest.approx(0.5, abs=1e-3)
    with pytest.raises(HandError):
        real.set_scale("index_dip", -1)


def test_stop_and_zero(real, clock):
    real.set_joint("thumb_ip", 1.0)
    clock.t += 0.2
    real.stop()
    stopped_at = real.positions()[4]
    clock.t += 2.0
    assert real.positions()[4] == pytest.approx(stopped_at)
    real.zero()
    assert real.positions() == pytest.approx(np.zeros(8))


def test_no_reply_raises():
    class Silent(FakeEsp32):
        def readline(self) -> bytes:
            return b""

    with pytest.raises(HandError, match="no reply"):
        RealHand(connection=Silent())


# ----- SimHand -----


def test_sim_hand_follows_targets():
    hand = SimHand()
    q = np.radians([60, 45, 30, 10, 45, 30, 20, 20])
    hand.set_targets(q)
    hand.wait()
    assert np.degrees(np.abs(hand.positions() - q)).max() < 2.0
    assert not hand.is_moving()
