"""SimHand: the `Hand` interface on the MuJoCo model.

Physics advances only when `step()` (or `wait()`) is called, so it can run faster or slower
than real time, which is useful for AI training.

    SimHand()                  # v0 model (8 joints)
    SimHand(hand="v1")         # v1 model (20 joints), sim/models/tendra_hand_v1.xml
    SimHand("my_model.xml")    # the variant is recognised from the model's actuators
    SimHand(hand="v1", lite=True)  # simplified meshes: much faster to draw (tendra.lite_model)
"""

from collections.abc import Sequence

import mujoco
import numpy as np

from tendra.hand import Hand
from tendra.joints import HANDS, V0, HandSpec, get_hand


class SimHand(Hand):
    def __init__(self, model_path=None, *, hand: str | HandSpec | None = None, lite: bool = False):
        spec = get_hand(hand) if hand is not None else None
        if model_path is None:
            spec = spec or V0
            model_path = spec.model_path
        if lite:
            from tendra.lite_model import load_lite_model

            self.model = load_lite_model(model_path)
        else:
            self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        actuators = tuple(self.model.actuator(i).name for i in range(self.model.nu))
        if spec is None:
            spec = next((s for s in HANDS.values() if s.joint_names == actuators), None)
            if spec is None:
                raise ValueError(f"model actuators {actuators} match no known hand")
        if actuators != spec.joint_names:
            raise ValueError(f"model actuators {actuators} do not match {spec.joint_names}")
        self.spec = spec
        self._qpos_adr = np.array([self.model.joint(n).qposadr[0] for n in spec.joint_names])
        self._dof_adr = np.array([self.model.joint(n).dofadr[0] for n in spec.joint_names])
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

    def set_positions(self, q: Sequence[float]) -> None:
        """Put the joints at `q` directly (no physics), and hold them there.

        Used to mirror the real hand (digital twin, real -> sim). NaN entries (joints with no
        measurement, e.g. an offline servo) are left where they are.
        """
        q = np.asarray(q, dtype=float)
        if q.shape != (self.num_joints,):
            raise ValueError(f"expected {self.num_joints} joint values, got shape {q.shape}")
        known = np.isfinite(q)
        self.data.qpos[self._qpos_adr[known]] = q[known]
        self.data.qvel[self._dof_adr[known]] = 0.0
        self.data.ctrl[known] = q[known]
        mujoco.mj_forward(self.model, self.data)

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
