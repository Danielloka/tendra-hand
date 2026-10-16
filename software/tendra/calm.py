"""Deploy-time "calm filter": sits between a policy's targets and the motors.

Plain numpy, per joint, no hidden state beyond what is written here, so the same logic can be
ported to the ESP32 firmware for the SCS0009 servos (research/ai/smooth-control.md, "The control
layer between the AI and the motors"). Units are SI (rad, m, s).

- `CalmFilter`: per joint a deadband with hysteresis, a first-order smoother and a rate limit.
- `HoldLatch`: phase machine reach -> grasp -> hold -> release that freezes the finger targets
  while the object is held, so policy noise cannot reach the motors.
- `calm_applied(env, ...)`: context manager that puts both between a `GraspEnv` and its scene.
"""

from __future__ import annotations

from contextlib import contextmanager

import numpy as np

REACH, GRASP, HOLD, RELEASE = "reach", "grasp", "hold", "release"


class CalmFilter:
    """Deadband with hysteresis + smoothing + rate limit, independently for each of n joints.

    A joint is *latched* (output held) until the new target differs from the held output by more
    than `release`. Then it is *moving* and tracks the target, until the target's change per step
    is below `deadband` and the output is within `release` of it; then it latches again. So it takes
    a bigger change to start moving than to keep moving, and it cannot chatter at the edge.
    While moving: `out += smoothing * (target - out)`, the step limited to `max_rate * dt`.
    `smoothing` = 1 means no smoothing; smaller = slower (first-order low-pass).
    """

    def __init__(self, n, deadband, release, max_rate, dt, smoothing=1.0):
        self.n, self.dt = n, float(dt)
        self.deadband = np.broadcast_to(np.asarray(deadband, float), (n,)).copy()
        self.release = np.broadcast_to(np.asarray(release, float), (n,)).copy()
        self.max_rate = np.broadcast_to(np.asarray(max_rate, float), (n,)).copy()
        self.smoothing = float(smoothing)
        if np.any(self.deadband > self.release):
            raise ValueError("deadband must not exceed release (hysteresis)")
        self.reset(np.zeros(n))

    def reset(self, q) -> None:
        self.out = np.array(q, dtype=float)
        self.prev = self.out.copy()
        self.moving = np.zeros(self.n, dtype=bool)

    def __call__(self, target) -> np.ndarray:
        target = np.asarray(target, dtype=float)
        err = target - self.out
        step_change = np.abs(target - self.prev)
        self.prev = target.copy()
        start = np.abs(err) > self.release
        stop = (step_change < self.deadband) & (np.abs(err) <= self.release)
        self.moving = (self.moving | start) & ~(stop & ~start)
        delta = np.clip(self.smoothing * err, -self.max_rate * self.dt, self.max_rate * self.dt)
        self.out = self.out + np.where(self.moving, delta, 0.0)
        return self.out.copy()


class HoldLatch:
    """Phase machine reach -> grasp -> hold -> release that freezes targets while holding.

    Call `__call__(target, lifted, contact, slip)` once per control step:
    - reach: no contact yet. `contact` -> grasp.
    - grasp: fingers closing. `lifted` and the target moving less than `calm` per step (rad, max
      over joints) for `hold_steps` steps -> hold, freezing the target. Contact lost without
      being lifted -> reach.
    - hold: the output is the frozen target. A `slip` (object speed relative to the hand, m/s,
      above `slip_limit`) -> grasp (unfrozen, regrip). The commanded target differing from the
      frozen one by more than `unfreeze` (rad, larger than `calm`: hysteresis) -> release.
    - release: target passes through; once `lifted` and `contact` are both false -> reach.
    `phase` and `log` (list of (step, phase)) record the changes.
    """

    def __init__(self, calm=0.05, hold_steps=4, unfreeze=0.15, slip_limit=0.05):
        self.calm, self.hold_steps = calm, hold_steps
        self.unfreeze, self.slip_limit = unfreeze, slip_limit
        self.reset()

    def reset(self) -> None:
        self.phase, self.frozen, self.prev = REACH, None, None
        self.calm_count, self.steps, self.log = 0, 0, [(0, REACH)]

    def _go(self, phase: str) -> None:
        if phase != self.phase:
            self.phase = phase
            self.log.append((self.steps, phase))

    def __call__(self, target, lifted=False, contact=False, slip=0.0) -> np.ndarray:
        target = np.asarray(target, dtype=float)
        change = 0.0 if self.prev is None else float(np.max(np.abs(target - self.prev)))
        self.prev = target.copy()
        self.steps += 1
        if self.phase == REACH:
            if contact or lifted:
                self._go(GRASP)
        elif self.phase == GRASP:
            self.calm_count = self.calm_count + 1 if (lifted and change < self.calm) else 0
            if self.calm_count >= self.hold_steps:
                self.frozen = target.copy()
                self._go(HOLD)
            elif not contact and not lifted:
                self._go(REACH)
        elif self.phase == HOLD:
            if slip > self.slip_limit:
                self.calm_count = 0
                self._go(GRASP)
            elif np.max(np.abs(target - self.frozen)) > self.unfreeze:
                self._go(RELEASE)
        elif self.phase == RELEASE and not lifted and not contact:
            self._go(REACH)
        return self.frozen.copy() if self.phase == HOLD else target


# Defaults for the 16 finger servos and the wrist position (see sim/eval_grasp.py --calm).
FINGER = {"deadband": np.radians(0.3), "release": np.radians(1.0), "max_rate": 4.0, "smoothing": 0.6}
WRIST = {"deadband": 0.0003, "release": 0.001, "max_rate": 0.3, "smoothing": 0.6}


_NO_LIMIT = {"max_rate": 1e9, "smoothing": 1.0}
PRESETS = {
    # deadband + hysteresis only: no rate limit or smoothing. Still cost the old (unfiltered-
    # trained) CPU policy 20-30 points of success on 2026-10-06: use with policies trained calm.
    "light": {"finger": {**FINGER, **_NO_LIMIT}, "wrist": {**WRIST, **_NO_LIMIT}, "latch": None},
    # + freeze the fingers once the object has been at lift height for 3 steps
    "light+latch": {"finger": {**FINGER, **_NO_LIMIT}, "wrist": {**WRIST, **_NO_LIMIT},
                        "latch": {"calm": float("inf"), "hold_steps": 3}},
    # rate limit + smoothing + latch on a calm target
    "full": {"finger": FINGER, "wrist": WRIST, "latch": {}},
}


@contextmanager
def calm_applied(env, preset="light", finger=None, wrist=None, latch=None):
    """Put the calm layer between `env` (a `GraspEnv`) and its scene while the block runs.

    Hooked at the scene's setters (monkeypatched on the instance, restored afterwards), and
    filtered ONCE per control step: the first setter call after `env.t` changed runs the layer and
    caches its output; further calls in the same control step (e.g. targets ramped across physics
    substeps) get the cached output, so the filter sees one sample per policy decision however
    the env calls the scene. `env.reset` resets the layer. Hooked at the env's `_command` (turns
    the action into targets) and the setters it feeds, before `_physics` ramps and the arm's IK
    run, so those only see the filtered targets. `_command` integrates the wrist target it reads
    back from the scene, so during `_command` `wrist_target()` returns the *unfiltered* target,
    else a deadband would swallow slow velocities. Wrist orientation passes through unfiltered. Yields the state object.
    """
    cfg = PRESETS[preset]
    finger, wrist = finger or cfg["finger"], wrist or cfg["wrist"]
    latch = cfg["latch"] if latch is None else latch
    s = env.scene
    nf = len(s.finger_targets())
    layer = type("Calm", (), {})()
    layer.fingers = CalmFilter(nf, dt=env.dt, **finger)
    layer.wrist = CalmFilter(3, dt=env.dt, **wrist)
    layer.latch = None if latch is None or latch is False else HoldLatch(**(latch or {}))
    layer.raw_wrist = None
    layer.phases = []
    state = {"tf": None, "tw": None, "fingers": None, "wrist": None, "prev_rel": None}
    orig = {n: getattr(s, n) for n in ("set_finger_targets", "set_wrist_target", "wrist_target")}
    orig_reset, orig_command = env.reset, env._command

    def rearm() -> None:
        state["tf"] = state["tw"] = state["prev_rel"] = None
        layer.latch and layer.latch.reset()

    def set_fingers(q, side=None):
        if state["tf"] != env.t or state["fingers"] is None:
            state["tf"] = env.t
            q = np.asarray(q, float)
            if layer.latch is not None:
                opos = s.object_pose()[0] - s.wrist_pose()[0]
                prev = state["prev_rel"]
                slip = 0.0 if prev is None else np.linalg.norm(opos - prev) / env.dt
                state["prev_rel"] = opos
                q = layer.latch(q, lifted=env.hold > 0, contact=s.object_contacts()[1], slip=slip)
                layer.phases.append(layer.latch.phase)
            state["fingers"] = layer.fingers(q)
        orig["set_finger_targets"](state["fingers"], side)

    def set_wrist(pos, quat, side=None):
        if state["tw"] != env.t or state["wrist"] is None:
            state["tw"] = env.t
            layer.raw_wrist = np.asarray(pos, float).copy()
            state["wrist"] = layer.wrist(layer.raw_wrist)
        orig["set_wrist_target"](state["wrist"], quat, side)

    def command(*a, **k):
        # `_command` integrates the wrist target it reads back from the scene: give it the raw
        # one (only the active hand's). The setter then filters what it returns, and the
        # scene (IK, ramps) only ever sees the filtered mocap target.
        def raw_target(side=None):
            p, q = orig["wrist_target"](side)
            if layer.raw_wrist is not None and side in (None, s._side):
                p = layer.raw_wrist.copy()
            return p, q

        s.wrist_target = raw_target
        try:
            return orig_command(*a, **k)
        finally:
            s.wrist_target = orig["wrist_target"]

    def reset(*a, **k):
        out = orig_reset(*a, **k)
        rearm()
        state["fingers"] = state["wrist"] = None
        layer.fingers.reset(s.finger_targets())
        layer.wrist.reset(s.wrist_target()[0])
        layer.raw_wrist = None
        layer.phases = []
        return out

    s.set_finger_targets, s.set_wrist_target = set_fingers, set_wrist
    env.reset, env._command = reset, command
    try:
        yield layer
    finally:
        for name in orig:  # drop the instance attributes: the class methods show again
            s.__dict__.pop(name, None)
        env.__dict__.pop("reset", None)
        env.__dict__.pop("_command", None)
