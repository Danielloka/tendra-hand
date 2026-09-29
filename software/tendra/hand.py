"""The `Hand` interface: the same API for the real hand and the simulation.

Code written against `Hand` (teleoperation, grasp scripts, AI policies) runs unchanged on
`SimHand` (MuJoCo) or `RealHand` (ESP32 over USB). This is the PC-side counterpart of the
firmware's MotorDriver HAL.

Each hand object has a `spec` (a `HandSpec`: v0 with 8 joints, or v1 with 21), which fixes the
number, order, names and limits of its joints.
"""

from abc import ABC, abstractmethod
from collections.abc import Sequence

import numpy as np

from tendra.joints import V0, HandSpec


class HandError(RuntimeError):
    """The hand rejected a command or did not answer."""


class Hand(ABC):
    spec: HandSpec = V0  # subclasses set this per instance

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.spec.joint_names

    @property
    def num_joints(self) -> int:
        return self.spec.num_joints

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
        targets[self.spec.joint_index(joint)] = q
        self.set_targets(targets)

    def open(self) -> None:
        """All joints straight."""
        self.set_targets(np.zeros(self.num_joints))

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _check(self, q: Sequence[float]) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        n = self.num_joints
        if q.shape != (n,):
            raise ValueError(f"expected {n} joint values, got shape {q.shape}")
        if not np.isfinite(q).all():
            raise ValueError("joint targets must be finite")
        return q
