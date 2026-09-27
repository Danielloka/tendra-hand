"""Digital-twin bridge logic, tested against the software ESP32."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tendra import RealHand
from tendra.fake_esp32 import FakeEsp32
from twin import TwinBridge


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
