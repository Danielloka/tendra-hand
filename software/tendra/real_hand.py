"""RealHand: talks to the ESP32 firmware over USB serial (text protocol v0.1).

See firmware/README.md for the protocol. Example:

    from tendra import RealHand
    with RealHand() as hand:            # finds the ESP32 automatically
        hand.set_joint("index_pip", 0.8)
        print(hand.positions())
"""

from collections.abc import Sequence

import numpy as np
import serial
from serial.tools import list_ports

from tendra.hand import Hand, HandError
from tendra.joints import NUM_JOINTS, joint_index

ESPRESSIF_USB_VID = 0x303A  # native USB of the ESP32-S3


def find_port() -> str:
    """Serial port of the first connected ESP32-S3 (native USB)."""
    for port in list_ports.comports():
        if port.vid == ESPRESSIF_USB_VID:
            return port.device
    found = ", ".join(p.device for p in list_ports.comports()) or "none"
    raise HandError(f"No ESP32-S3 found on USB (ports: {found}). Is it plugged in?")


class RealHand(Hand):
    def __init__(self, port: str | None = None, *, connection=None, timeout: float = 1.0):
        """Open `port` (auto-detected if None), or use an existing serial-like `connection`."""
        if connection is None:
            connection = serial.Serial(port or find_port(), 921600, timeout=timeout)
        self._conn = connection
        self._targets = np.zeros(NUM_JOINTS)
        self._conn.reset_input_buffer()
        self.firmware_info = self.command("I")

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
        """(positions, moving bitmask) from the firmware's step counters."""
        parts = self.command("S").split()
        if parts[0] != "S" or len(parts) != NUM_JOINTS + 2:
            raise HandError(f"bad status reply: {parts!r}")
        return np.array([float(v) for v in parts[1:-1]]), int(parts[-1])

    # ----- Hand interface -----

    def set_targets(self, q: Sequence[float]) -> None:
        q = self._check(q)
        self.command("P " + " ".join(f"{v:.4f}" for v in q))
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

    # ----- setup and calibration (see the checklist in firmware/README.md) -----

    def zero(self) -> None:
        """Declare the current pose straight (q = 0). Straighten the joints by hand first."""
        self.command("Z")
        self._targets = np.zeros(NUM_JOINTS)

    def raw_move(self, joint: int | str, steps: int) -> None:
        """Move one joint by raw motor steps, ignoring joint limits (calibration only)."""
        self.command(f"M {joint_index(joint) + 1} {int(steps)}")

    def set_scale(self, joint: int | str, steps_per_rad: float) -> None:
        self.command(f"K {joint_index(joint) + 1} {steps_per_rad:.3f}")

    def set_limits(self, max_vel: float, max_acc: float, joint: int | str | None = None) -> None:
        """Velocity [rad/s] and acceleration [rad/s^2] limits for one joint, or all if None."""
        i = 0 if joint is None else joint_index(joint) + 1
        self.command(f"V {i} {max_vel:.4f} {max_acc:.4f}")
