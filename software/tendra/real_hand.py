"""RealHand: talks to the ESP32 firmware over USB serial (text protocol, firmware/README.md).

Both hands use the same protocol with a different number of joints. The variant (v0: 8
steppers, v1: 21 servos) is read from the firmware's `I` line, or can be given. Example:

    from tendra import RealHand
    with RealHand() as hand:            # finds the ESP32 and the hand variant automatically
        hand.set_joint("index_pip", 0.8)
        print(hand.positions())         # v1: measured by the servos; v0: counted steps
        print(hand.feedback())          # v1: position, load, temperature, voltage per joint
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
import serial
from serial.tools import list_ports

from tendra.hand import Hand, HandError
from tendra.joints import HANDS, V0, HandSpec, get_hand

ESPRESSIF_USB_VID = 0x303A  # native USB of the ESP32-S3


def find_port() -> str:
    """Serial port of the first connected ESP32-S3 (native USB)."""
    for port in list_ports.comports():
        if port.vid == ESPRESSIF_USB_VID:
            return port.device
    found = ", ".join(p.device for p in list_ports.comports()) or "none"
    raise HandError(f"No ESP32-S3 found on USB (ports: {found}). Is it plugged in?")


@dataclass(frozen=True)
class JointFeedback:
    """One joint's measured state (from `F`). Without data (offline servo, or v0): NaN."""

    name: str
    position: float  # rad, NaN if unknown
    load: float  # % of max torque, positive = pulling toward closing
    temperature: float  # deg C
    voltage: float  # V

    @property
    def online(self) -> bool:
        return not math.isnan(self.position)


@dataclass(frozen=True)
class JointInfo:
    """One joint's calibration, from the `I` line."""

    name: str
    scale: float  # native units (steps or servo ticks) per joint radian
    zero_ticks: int | None = None  # v1: servo reading at q = 0 (copy into config_v1.h)
    online: bool | None = None  # v1: does the servo answer?


def parse_info(line: str) -> tuple[HandSpec, list[JointInfo]]:
    """Hand variant and per-joint calibration from the firmware's `I` reply."""
    tokens = line.split()
    if not tokens or tokens[0] != "I":
        raise HandError(f"bad info reply: {line!r}")
    fields = dict(t.split("=", 1) for t in tokens if "=" in t)
    if "hand" in fields:
        spec = next((s for s in HANDS.values() if s.firmware_variant == fields["hand"]), None)
        if spec is None:
            raise HandError(f"unknown hand variant {fields['hand']!r} in {line!r}")
    elif "joints" in fields:
        n = int(fields["joints"])
        spec = next((s for s in HANDS.values() if s.num_joints == n), None)
        if spec is None:
            raise HandError(f"no known hand has {n} joints: {line!r}")
    else:
        spec = V0  # oldest firmware: no fields
    joints = []
    for t in tokens:
        name, *values = t.split(":")
        if name in spec.joint_names and values:
            joints.append(
                JointInfo(
                    name,
                    float(values[0]),
                    int(values[1]) if len(values) > 1 else None,
                    values[2] == "1" if len(values) > 2 else None,
                )
            )
    return spec, joints


class RealHand(Hand):
    def __init__(
        self,
        port: str | None = None,
        *,
        connection=None,
        timeout: float = 1.0,
        hand: str | HandSpec | None = None,
    ):
        """Open `port` (auto-detected if None), or use an existing serial-like `connection`.

        `hand` ("v0", "v1") is detected from the firmware if None; if given, it must match.
        """
        if connection is None:
            connection = serial.Serial(port or find_port(), 921600, timeout=timeout)
        self._conn = connection
        self._conn.reset_input_buffer()
        self.firmware_info = self.command("I")
        detected, _ = parse_info(self.firmware_info)
        if hand is not None and get_hand(hand) != detected:
            raise HandError(
                f"expected hand {get_hand(hand).name}, but the firmware runs {detected.name}: "
                f"{self.firmware_info!r}"
            )
        self.spec = detected
        self._targets = np.zeros(self.num_joints)
        self.offline_mask = 0  # v1: servos that didn't take the last P (bit i = joint i)

    # ----- protocol -----

    def command(self, line: str) -> str:
        """Send one command, return its reply line. Raises HandError on ERR or timeout."""
        self._conn.write((line + "\n").encode("ascii"))
        reply = self._conn.readline().decode("ascii", errors="replace").strip()
        if not reply:
            raise HandError(f"no reply to {line!r} (timeout)")
        if reply.startswith("ERR"):
            raise HandError(f"{line!r} -> {reply}")
        return reply

    def status(self) -> tuple[np.ndarray, int]:
        """(positions, moving bitmask). v0: counted steps; v1: measured by the servos."""
        parts = self.command("S").split()
        if parts[0] != "S" or len(parts) != self.num_joints + 2:
            raise HandError(f"bad status reply: {parts!r}")
        return np.array([float(v) for v in parts[1:-1]]), int(parts[-1])

    def feedback(self) -> list[JointFeedback]:
        """Measured state per joint (`F`). v0 and offline servos have NaN positions."""
        parts = self.command("F").split()
        if parts[0] != "F" or len(parts) != self.num_joints + 1:
            raise HandError(f"bad feedback reply: {parts!r}")
        result = []
        for name, part in zip(self.joint_names, parts[1:], strict=True):
            values = part.split(",")
            if len(values) != 4:
                raise HandError(f"bad feedback for {name}: {part!r}")
            q, load, temp, volt = (float(v) for v in values)
            result.append(JointFeedback(name, q, load, temp, volt))
        return result

    def measured_positions(self) -> np.ndarray:
        """Measured joint positions from `F`, NaN where a servo is offline (v0: all NaN)."""
        return np.array([fb.position for fb in self.feedback()])

    def online(self) -> np.ndarray:
        """Bool per joint: does its motor answer? (v0 steppers can't tell: all False.)"""
        return np.isfinite(self.measured_positions())

    def scan_bus(self) -> list[int]:
        """IDs of the servos that answer a ping (`B`, v1 only; takes up to ~0.5 s)."""
        if not self.spec.has_feedback:
            raise HandError(f"hand {self.spec.name} has no servo bus")
        parts = self.command("B").split()
        if parts[0] != "B":
            raise HandError(f"bad scan reply: {parts!r}")
        return [int(p) for p in parts[1:]]

    def info(self) -> list[JointInfo]:
        """Per-joint calibration (scale; v1 also zero ticks and online), read fresh from `I`."""
        return parse_info(self.command("I"))[1]

    # ----- Hand interface -----

    def set_targets(self, q: Sequence[float]) -> None:
        q = self._check(q)
        reply = self.command("P " + " ".join(f"{v:.4f}" for v in q))
        parts = reply.split()
        self.offline_mask = int(parts[2]) if parts[:2] == ["OK", "offline"] else 0
        self._targets = q.copy()

    def positions(self) -> np.ndarray:
        return self.status()[0]

    def targets(self) -> np.ndarray:
        return self._targets.copy()

    def is_moving(self) -> bool:
        return self.status()[1] != 0

    def stop(self) -> None:
        self.command("X")
        self._targets = self.positions()

    def release(self) -> None:
        self.command("R")

    def close(self) -> None:
        self._conn.close()

    # ----- setup and calibration (see the checklists in firmware/README.md) -----

    def zero(self) -> None:
        """Declare the current pose straight (q = 0). Straighten the joints by hand first."""
        self.command("Z")
        self._targets = np.zeros(self.num_joints)

    def raw_move(self, joint: int | str, steps: int) -> None:
        """Move one joint by raw motor units (steps / servo ticks), ignoring joint limits."""
        self.command(f"M {self.spec.joint_index(joint) + 1} {int(steps)}")

    def set_scale(self, joint: int | str, steps_per_rad: float) -> None:
        """Native units (steps / servo ticks) per joint radian."""
        self.command(f"K {self.spec.joint_index(joint) + 1} {steps_per_rad:.3f}")

    def set_limits(self, max_vel: float, max_acc: float, joint: int | str | None = None) -> None:
        """Velocity [rad/s] and acceleration [rad/s^2] limits for one joint, or all if None."""
        i = 0 if joint is None else self.spec.joint_index(joint) + 1
        self.command(f"V {i} {max_vel:.4f} {max_acc:.4f}")
