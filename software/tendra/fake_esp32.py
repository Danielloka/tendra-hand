"""A software stand-in for the ESP32 firmware, for testing without hardware.

It speaks the same text protocol (firmware/README.md) and behaves like a serial port, so it can
be passed to `RealHand(connection=FakeEsp32())`. Motion is simplified: each joint moves toward
its target at constant maximum speed (the real firmware also ramps the acceleration).

    FakeEsp32()                              # v0: 8 steppers
    FakeEsp32(hand="v1")                     # v1: 16 servos, all online
    FakeEsp32(hand="v1", offline={9, 16})    # v1 with servos 9 and 16 not answering

v1 emulates the servo firmware (firmware/src/main.cpp, scs0009_servo.cpp): servos start limp
(torque off) and switch on with the first target; offline servos ignore `P`, refuse `J`/`M`
(`ERR joint offline`), report `nan,0,0,0` in `F` and are missing from `B`. A limp servo can be
moved "by hand" with `move_by_hand()`, and `S` reports the measured position.
"""

import time
from collections.abc import Iterable

import numpy as np

from tendra.joints import HandSpec, get_hand

# v0: 28BYJ-48 half-step
DEFAULT_STEPS_PER_RAD = 4075.772 / (2 * np.pi)
DEFAULT_MAX_VEL = 800 / DEFAULT_STEPS_PER_RAD  # rad/s, same as the firmware defaults
MAX_RAW_MOVE = 4096

# v1: SCS0009 (config_v1.h, scs_protocol.h)
SERVO_TICKS_PER_RAD = 1024 / np.radians(300)  # ~195.57
FIRMWARE_VERSION_V1 = "0.4.0"  # firmware/include/config_v1.h
SERVO_TICKS_MIN, SERVO_TICKS_MAX = 0, 1023
SERVO_ZERO_TICKS = 512
SERVO_DEFAULT_MAX_VEL = 1.2  # rad/s
SERVO_MAX_TICKS_PER_S = 400.0  # the firmware's hard cap on its own ramp
SERVO_MAX_RAW_MOVE = 300


class FakeEsp32:
    def __init__(
        self,
        clock=time.monotonic,
        *,
        hand: str | HandSpec = "v0",
        offline: Iterable[int | str] = (),
    ):
        self.spec = get_hand(hand)
        self.servo = self.spec.has_feedback
        n = self.spec.num_joints
        self._clock = clock
        self._t = clock()
        self.pos = np.zeros(n)  # v1: the measured (servo) position
        self.target = np.zeros(n)
        if self.servo:
            # ticks per joint radian = servo_per_joint (drum / spool) x ticks per servo radian
            self.scale = SERVO_TICKS_PER_RAD * np.array(self.spec.servo_per_joint or [1.0] * n)
            self.max_vel = np.full(n, SERVO_DEFAULT_MAX_VEL)
            self.zero_ticks = np.full(n, SERVO_ZERO_TICKS, dtype=int)
        else:
            self.scale = np.full(n, DEFAULT_STEPS_PER_RAD)
            self.max_vel = np.full(n, DEFAULT_MAX_VEL)
        # v0 steppers re-energise on any move, so they always count as "on".
        self.torque = np.full(n, not self.servo)
        self.online = np.ones(n, dtype=bool)
        for joint in offline:
            self.set_online(joint, False)
        # Feedback values reported by F (v1); tests may change them.
        self.load = np.zeros(n)  # % of max torque
        self.temperature = np.full(n, 30.0)  # deg C
        self.voltage = np.full(n, 5.0)  # V
        self._lo, self._hi = self.spec.lower, self.spec.upper
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

    # ----- test helpers (the physical world) -----

    def _index(self, joint: int | str) -> int:
        """Joint name, or 1-based motor number / servo ID."""
        return self.spec.joint_index(joint) if isinstance(joint, str) else self._joint(str(joint))

    def set_online(self, joint: int | str, online: bool = True) -> None:
        """Connect or disconnect a servo (joint name or servo ID)."""
        self.online[self._index(joint)] = online

    def move_by_hand(self, joint: int | str, q: float) -> None:
        """Move a limp (torque-off) joint by hand to `q` rad (joint name or servo ID)."""
        i = self._index(joint)
        self._advance()
        if self.torque[i]:
            raise RuntimeError("the servo is holding its position; release it (R) first")
        self.pos[i] = self.target[i] = q

    # ----- firmware behaviour -----

    def _speed(self) -> np.ndarray:
        if self.servo:
            return np.minimum(self.max_vel, SERVO_MAX_TICKS_PER_S / self.scale)
        return self.max_vel

    def _advance(self) -> None:
        now = self._clock()
        dt, self._t = now - self._t, now
        step = self._speed() * dt
        moving = self.torque
        self.pos[moving] += np.clip(self.target - self.pos, -step, step)[moving]

    def _joint(self, token: str) -> int:
        i = int(token)
        if not 1 <= i <= self.spec.num_joints:
            raise ValueError(token)
        return i - 1

    def _moving_mask(self) -> int:
        return sum(1 << i for i in range(self.spec.num_joints) if self.pos[i] != self.target[i])

    def _offline_mask(self) -> int:
        return sum(1 << i for i in range(self.spec.num_joints) if not self.online[i])

    def _switch_on(self, i: int) -> bool:
        """v1 Scs0009Servo::ensureTorque(): an offline servo can't be switched on."""
        if not self.servo:
            return True
        if not self.online[i]:
            return False
        if not self.torque[i]:
            self.target[i] = self.pos[i]  # hold exactly where it is, then torque on
            self.torque[i] = True
        return True

    def _set_target(self, i: int, q: float) -> None:
        q = float(np.clip(q, self._lo[i], self._hi[i]))
        if self._switch_on(i):
            self.target[i] = q

    def _info(self) -> str:
        n = self.spec.num_joints
        if self.servo:
            head = f"I tendra-hand fw {FIRMWARE_VERSION_V1} hand={self.spec.firmware_variant} joints={n}"
            joints = (
                f"{name}:{self.scale[i]:.2f}:{self.zero_ticks[i]}:{int(self.online[i])}"
                for i, name in enumerate(self.spec.joint_names)
            )
        else:
            head = f"I tendra-hand fw 0.1.0 joints={n}"
            joints = (f"{name}:{self.scale[i]:.2f}" for i, name in enumerate(self.spec.joint_names))
        return " ".join((head, *joints))

    def _feedback(self) -> str:
        parts = []
        for i in range(self.spec.num_joints):
            if self.servo and self.online[i]:
                parts.append(
                    f"{self.pos[i]:.4f},{self.load[i]:.1f},"
                    f"{self.temperature[i]:.0f},{self.voltage[i]:.1f}"
                )
            else:
                parts.append("nan,0,0,0")
        return "F " + " ".join(parts)

    def _handle(self, line: str) -> str:
        self._advance()
        cmd, *args = line.split()
        n = self.spec.num_joints
        try:
            match cmd.upper():
                case "P":
                    if len(args) < n:
                        return f"ERR P needs {n} values"
                    q = [float(a) for a in args[:n]]
                    for i in range(n):
                        self._set_target(i, q[i])
                    offline = self._offline_mask() if self.servo else 0
                    return f"OK offline {offline}" if offline else "OK"
                case "J":
                    i = self._joint(args[0])
                    q = float(args[1])
                    if self.servo and not self.online[i]:
                        return "ERR joint offline"
                    self._set_target(i, q)
                case "S":
                    return "S " + " ".join(f"{p:.4f}" for p in self.pos) + f" {self._moving_mask()}"
                case "F":
                    return self._feedback()
                case "X":
                    self.target = self.pos.copy()
                case "R":
                    if self.servo:  # torque off: limp, nothing left to move to
                        self.target = self.pos.copy()
                        self.torque[:] = False
                case "Z":
                    if np.any(self.pos != self.target):
                        return "ERR joints are moving"
                    if self.servo:
                        self.zero_ticks += np.round(self.pos * self.scale).astype(int)
                    self.pos[:] = 0
                    self.target[:] = 0
                case "M":
                    i = self._joint(args[0])
                    raw = int(args[1])
                    if self.servo:
                        if not self.online[i]:
                            return "ERR joint offline"
                        raw = max(-SERVO_MAX_RAW_MOVE, min(SERVO_MAX_RAW_MOVE, raw))
                        self._switch_on(i)
                        # Clamped to the servo's tick range, not to the joint limits.
                        lo = (SERVO_TICKS_MIN - self.zero_ticks[i]) / self.scale[i]
                        hi = (SERVO_TICKS_MAX - self.zero_ticks[i]) / self.scale[i]
                        self.target[i] = np.clip(self.target[i] + raw / self.scale[i], lo, hi)
                    else:
                        raw = max(-MAX_RAW_MOVE, min(MAX_RAW_MOVE, raw))
                        self.target[i] += raw / self.scale[i]
                case "K":
                    i = self._joint(args[0])
                    scale = float(args[1])
                    if scale <= 0:
                        raise ValueError(args[1])
                    if self.servo:
                        # The servo stays at the same ticks; only the conversion changes.
                        self.pos[i] *= self.scale[i] / scale
                        self.target[i] *= self.scale[i] / scale
                    self.scale[i] = scale
                case "V":
                    i, vel, acc = int(args[0]), float(args[1]), float(args[2])
                    if not 0 <= i <= n or vel <= 0 or acc <= 0:
                        raise ValueError(line)
                    self.max_vel[slice(None) if i == 0 else i - 1] = vel
                case "B" if self.servo:
                    return "B" + "".join(
                        f" {sid}" for sid, on in zip(self.spec.servo_ids, self.online) if on
                    )
                case "I":
                    return self._info()
                case _:
                    raise ValueError(cmd)
        except (ValueError, IndexError):
            return "ERR bad command (send I for info)"
        return "OK"
