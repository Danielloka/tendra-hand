"""Digital-twin bridge logic, tested against the software ESP32."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tendra import RealHand
from tendra.fake_esp32 import FakeEsp32
from twin import MirrorBridge, TwinBridge


def make_bridge(rate_hz=20.0):
    esp = FakeEsp32(clock=lambda: 0.0)
    return TwinBridge(RealHand(connection=esp), rate_hz), esp


def sent_commands(esp):
    return [c for c in esp.log if c.startswith("P")]


def test_nothing_sent_until_targets_change():
    bridge, esp = make_bridge()
    for t in np.arange(0, 1, 0.01):
        bridge.update(np.zeros(8), t)
    assert sent_commands(esp) == []


def test_changes_are_sent_and_rate_limited():
    bridge, esp = make_bridge(rate_hz=20)
    # A slider dragged continuously for 1 s, updated at 100 Hz.
    for t in np.arange(0, 1, 0.01):
        bridge.update(np.full(8, t * 0.5), t)
    assert 18 <= len(sent_commands(esp)) <= 21


def test_final_value_is_sent():
    bridge, _ = make_bridge(rate_hz=20)
    bridge.update(np.full(8, 0.3), 0.0)
    assert not bridge.update(np.full(8, 0.4), 0.01)  # too soon
    assert bridge.update(np.full(8, 0.4), 0.06)  # retried later, then sent
    assert np.allclose(bridge.sent, 0.4)


# ----- v1: sim -> real with 20 joints, and real -> sim mirroring -----


def test_v1_targets_are_sent():
    esp = FakeEsp32(clock=lambda: 0.0, hand="v1")
    bridge = TwinBridge(RealHand(connection=esp))
    assert bridge.update(np.full(20, 0.2), 0.0)
    assert len(sent_commands(esp)[0].split()) == 21


def test_mirror_reads_measured_positions_and_never_commands():
    esp = FakeEsp32(clock=lambda: 0.0, hand="v1", offline={"ring_dip"})
    mirror = MirrorBridge(RealHand(connection=esp), rate_hz=10)
    esp.move_by_hand("index_pip", 0.7)
    q = mirror.update(0.0)
    assert q[1] == pytest.approx(0.7)
    assert np.isnan(q[12])  # offline servo (ring_dip): the sim keeps its last pose
    assert mirror.update(0.05) is None  # rate-limited
    assert mirror.update(0.1) is not None
    assert {c.split()[0] for c in esp.log} == {"I", "F"}  # only reads


def test_mirror_needs_feedback():
    with pytest.raises(ValueError, match="feedback"):
        MirrorBridge(RealHand(connection=FakeEsp32()))
