"""SimHand: the `Hand` interface on the MuJoCo model.

Physics advances only when `step()` (or `wait()`) is called, so it can run faster or slower
than real time, which is useful for AI training.
"""

from collections.abc import Sequence

import mujoco
import numpy as np

from tendra.hand import Hand
from tendra.joints import JOINT_NAMES, MODEL_PATH


class SimHand(Hand):
    def __init__(self, model_path=MODEL_PATH):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        actuators = tuple(self.model.actuator(i).name for i in range(self.model.nu))
        if actuators != JOINT_NAMES:
            raise ValueError(f"model actuators {actuators} do not match {JOINT_NAMES}")
        self._qpos_adr = np.array([self.model.joint(n).qposadr[0] for n in JOINT_NAMES])
        self._dof_adr = np.array([self.model.joint(n).dofadr[0] for n in JOINT_NAMES])
        mujoco.mj_forward(self.model, self.data)

    def set_targets(self, q: Sequence[float]) -> None:
        # Position actuators clamp to the joint range (ctrlrange), like the firmware does.
        self.data.ctrl[:] = self._check(q)

    def positions(self) -> np.ndarray:
        return self.data.qpos[self._qpos_adr].copy()

    def targets(self) -> np.ndarray:
        return self.data.ctrl.copy()

    def is_moving(self, tol: float = 1e-3) -> bool:
        return bool(np.abs(self.data.qvel[self._dof_adr]).max() > tol)

    def stop(self) -> None:
        self.data.ctrl[:] = self.positions()

    def step(self, seconds: float) -> None:
        """Advance the physics by `seconds`."""
        for _ in range(max(1, round(seconds / self.model.opt.timestep))):
            mujoco.mj_step(self.model, self.data)

    def wait(self, timeout: float = 5.0) -> None:
        """Simulate until the joints settle (or `timeout` simulated seconds pass)."""
        self.step(0.05)
        t_end = self.data.time + timeout
        while self.is_moving() and self.data.time < t_end:
            self.step(0.02)
