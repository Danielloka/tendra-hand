"""A software stand-in for the ESP32 firmware, for testing without hardware.

It speaks the same text protocol (firmware/README.md) and behaves like a serial port, so it can
be passed to `RealHand(connection=FakeEsp32())`. Motion is simplified: each joint moves toward
its target at constant maximum speed (the real firmware also ramps the acceleration).
"""

import time

import numpy as np

from tendra.joints import JOINT_LIMITS, NUM_JOINTS

DEFAULT_STEPS_PER_RAD = 4075.772 / (2 * np.pi)
DEFAULT_MAX_VEL = 800 / DEFAULT_STEPS_PER_RAD  # rad/s, same as the firmware defaults
MAX_RAW_MOVE = 4096


class FakeEsp32:
    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._t = clock()
        self.pos = np.zeros(NUM_JOINTS)
        self.target = np.zeros(NUM_JOINTS)
        self.max_vel = np.full(NUM_JOINTS, DEFAULT_MAX_VEL)
        self.scale = np.full(NUM_JOINTS, DEFAULT_STEPS_PER_RAD)
        self._lo = np.array([lo for lo, _ in JOINT_LIMITS])
        self._hi = np.array([hi for _, hi in JOINT_LIMITS])
        self._replies: list[bytes] = []
        self.log: list[str] = []  # every command received, for tests

    # ----- serial-port-like interface -----

    def write(self, data: bytes) -> int:
        for line in data.decode("ascii").splitlines():
            if line.strip():
                self.log.append(line.strip())
                self._replies.append((self._handle(line.strip()) + "\n").encode("ascii"))
        return len(data)

    def readline(self) -> bytes:
        return self._replies.pop(0) if self._replies else b""

    def reset_input_buffer(self) -> None:
        self._replies.clear()

    def close(self) -> None:
        pass

    # ----- firmware behaviour -----

    def _advance(self) -> None:
        now = self._clock()
        dt, self._t = now - self._t, now
        self.pos += np.clip(self.target - self.pos, -self.max_vel * dt, self.max_vel * dt)

    @staticmethod
    def _joint(token: str) -> int:
        i = int(token)
        if not 1 <= i <= NUM_JOINTS:
            raise ValueError(token)
        return i - 1

    def _handle(self, line: str) -> str:
        self._advance()
        cmd, *args = line.split()
        try:
            match cmd.upper():
                case "P":
                    if len(args) != NUM_JOINTS:
                        return "ERR P needs 8 values"
                    self.target = np.clip([float(a) for a in args], self._lo, self._hi)
                case "J":
                    i = self._joint(args[0])
                    self.target[i] = np.clip(float(args[1]), self._lo[i], self._hi[i])
                case "S":
                    moving = sum(1 << i for i in range(NUM_JOINTS) if self.pos[i] != self.target[i])
                    return "S " + " ".join(f"{p:.4f}" for p in self.pos) + f" {moving}"
                case "X":
                    self.target = self.pos.copy()
                case "R":
                    pass
                case "Z":
                    if np.any(self.pos != self.target):
                        return "ERR joints are moving"
                    self.pos[:] = 0
                    self.target[:] = 0
                case "M":
                    i = self._joint(args[0])
                    steps = max(-MAX_RAW_MOVE, min(MAX_RAW_MOVE, int(args[1])))
                    self.target[i] += steps / self.scale[i]
                case "K":
                    i = self._joint(args[0])
                    if float(args[1]) <= 0:
                        raise ValueError(args[1])
                    self.scale[i] = float(args[1])
                case "V":
                    i, vel, acc = int(args[0]), float(args[1]), float(args[2])
                    if not 0 <= i <= NUM_JOINTS or vel <= 0 or acc <= 0:
                        raise ValueError(line)
                    self.max_vel[slice(None) if i == 0 else i - 1] = vel
                case "I":
                    return "I tendra-hand fw fake joints=8"
                case _:
                    raise ValueError(cmd)
        except (ValueError, IndexError):
            return "ERR bad command (send I for info)"
        return "OK"
