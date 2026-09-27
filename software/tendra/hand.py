"""The `Hand` interface: the same API for the real hand and the simulation.

Code written against `Hand` (teleoperation, grasp scripts, AI policies) runs unchanged on
`SimHand` (MuJoCo) or `RealHand` (ESP32 over USB). This is the PC-side counterpart of the
firmware's MotorDriver HAL.
"""

from abc import ABC, abstractmethod
from collections.abc import Sequence

import numpy as np

from tendra.joints import JOINT_NAMES, NUM_JOINTS, joint_index


class HandError(RuntimeError):
    """The hand rejected a command or did not answer."""


class Hand(ABC):
    joint_names = JOINT_NAMES

    @abstractmethod
    def set_targets(self, q: Sequence[float]) -> None:
        """Move all joints to `q` (radians, motor order). Returns immediately."""

    @abstractmethod
    def positions(self) -> np.ndarray:
        """Current joint positions (radians, motor order)."""

    @abstractmethod
    def targets(self) -> np.ndarray:
        """Last commanded targets (radians, motor order)."""

    @abstractmethod
    def is_moving(self) -> bool:
        """True while any joint is still travelling."""

    def stop(self) -> None:
        """Halt all joints smoothly."""

    def release(self) -> None:
        """Remove motor power; joints go limp."""

    def close(self) -> None:
        """Free resources (serial port, ...)."""

    # ----- conveniences built on the methods above -----

    def set_joint(self, joint: int | str, q: float) -> None:
        targets = self.targets()
        targets[joint_index(joint)] = q
        self.set_targets(targets)

    def open(self) -> None:
        """All joints straight."""
        self.set_targets(np.zeros(NUM_JOINTS))

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @staticmethod
    def _check(q: Sequence[float]) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        if q.shape != (NUM_JOINTS,):
            raise ValueError(f"expected {NUM_JOINTS} joint values, got shape {q.shape}")
        if not np.isfinite(q).all():
            raise ValueError("joint targets must be finite")
        return q
