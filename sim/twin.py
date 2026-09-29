"""Digital twin: the simulation drives the real hand, or (v1) mirrors it.

    uv run python sim/twin.py                           # simulation only (v0 model)
    uv run python sim/twin.py --hand v1                 # simulation only (v1 model)
    uv run python sim/twin.py --fake                    # with a software ESP32 (no hardware)
    uv run python sim/twin.py --fake --hand v1          # software ESP32 running the v1 firmware
    uv run python sim/twin.py --port auto               # real hand, ESP32 and variant detected
    uv run python sim/twin.py --port COM5 --mirror      # v1: the sim follows the real hand

Two directions:

- sim -> real (default): use the sliders under "Control" (right panel). The joint targets are
  sent to the hand at most `--rate` times per second, and only when they change. The terminal
  shows how far the real hand is from the simulation.
- real -> sim (`--mirror`, v1 only): the servos measure their positions, and the simulation
  shows where the real joints are. Nothing is sent to the hand except read requests, so this is
  safe with limp joints: move the fingers by hand and watch the sim follow. Offline servos keep
  their last pose.

`--hand auto` (default) takes the variant from the firmware (or v0 without a hand).
Before driving a v0 hand: straighten all joints by hand and power up (or send Z).
Nothing is sent until a slider moves, so the hand never moves on start-up. A v1 hand starts
from its measured pose, so the first slider move doesn't jerk the other joints.
"""

import argparse
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np
from tendra import Hand, RealHand, SimHand, calibration, get_hand
from tendra.fake_esp32 import FakeEsp32


class TwinBridge:
    """Forwards changed simulation targets to a hand, rate-limited (sim -> real)."""

    def __init__(self, hand: Hand, rate_hz: float = 20.0, tolerance: float = 1e-3):
        self.hand = hand
        self.period = 1.0 / rate_hz
        self.tolerance = tolerance
        self.sent = hand.targets()
        self._next_send = 0.0

    def update(self, targets: np.ndarray, now: float) -> bool:
        """Send `targets` if they changed and the rate limit allows. Returns True if sent."""
        if now < self._next_send or np.allclose(targets, self.sent, atol=self.tolerance):
            return False
        self.hand.set_targets(targets)
        self.sent = targets.copy()
        self._next_send = now + self.period
        return True


class MirrorBridge:
    """Reads the real hand's measured positions, rate-limited (real -> sim). Only reads."""

    def __init__(self, hand: RealHand, rate_hz: float = 30.0):
        if not hand.spec.has_feedback:
            raise ValueError(
                f"hand {hand.spec.name} has no position feedback; mirroring needs servos (v1)"
            )
        self.hand = hand
        self.period = 1.0 / rate_hz
        self.feedback = []  # last JointFeedback list
        self._next_read = 0.0

    def update(self, now: float) -> np.ndarray | None:
        """Measured positions (NaN = offline servo) if it's time for a reading, else None."""
        if now < self._next_read:
            return None
        self._next_read = now + self.period
        self.feedback = self.hand.feedback()
        return np.array([fb.position for fb in self.feedback])


def connect(args) -> RealHand | None:
    hand = None if args.hand == "auto" else args.hand
    if args.fake:
        real = RealHand(connection=FakeEsp32(hand=hand or "v0"), hand=hand)
    elif args.port:
        real = RealHand(None if args.port == "auto" else args.port, hand=hand)
    else:
        return None
    applied = calibration.apply(real)  # scales and speeds from calibration_<hand>.json
    if applied:
        print("Calibration: " + ", ".join(applied))
    return real


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="serial port of the ESP32, or 'auto'")
    parser.add_argument("--fake", action="store_true", help="use a software ESP32")
    parser.add_argument(
        "--hand", default="auto", choices=["auto", "v0", "v1"], help="hand variant (auto: ask)"
    )
    parser.add_argument(
        "--mirror", action="store_true", help="real -> sim: the sim follows the measured pose (v1)"
    )
    parser.add_argument("--rate", type=float, default=20.0, help="max commands per second")
    parser.add_argument("--seconds", type=float, help="quit after this many seconds (testing)")
    args = parser.parse_args()

    hand = connect(args)
    spec = hand.spec if hand else get_hand("v0" if args.hand == "auto" else args.hand)
    if not spec.model_path.exists():
        print(f"No MuJoCo model for hand {spec.name}: {spec.model_path} is missing.")
        return 1
    if args.mirror and not hand:
        print("--mirror needs a hand to follow: add --port or --fake.")
        return 1
    if args.mirror and not spec.has_feedback:
        print(f"--mirror needs position feedback (v1 servos); hand {spec.name} has none.")
        return 1
    sim = SimHand(hand=spec)
    model, data = sim.model, sim.data
    drive = TwinBridge(hand, args.rate) if hand and not args.mirror else None
    mirror = MirrorBridge(hand, args.rate) if args.mirror else None
    if hand:
        print(f"Connected: {hand.firmware_info}")
    if drive and spec.has_feedback:
        # Start the sim (and the "already sent" targets) at the measured pose.
        q = np.nan_to_num(hand.measured_positions(), nan=0.0)
        sim.set_positions(q)
        drive.sent = sim.targets()
    if mirror:
        print("Mirroring the real hand (real -> sim). Move the fingers; close the window to quit.")
    else:
        print("Move the sliders under 'Control' in the viewer. Close the window to quit.")

    frame = 1 / 60
    steps_per_frame = max(1, round(frame / model.opt.timestep))
    t_start = time.perf_counter()
    next_status = 0.0
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                t_frame = time.perf_counter()
                now = t_frame - t_start
                if args.seconds and now > args.seconds:
                    break
                measured = mirror.update(now) if mirror else None
                with viewer.lock():
                    if mirror:
                        if measured is not None:
                            sim.set_positions(measured)
                    else:
                        for _ in range(steps_per_frame):
                            mujoco.mj_step(model, data)
                    targets = data.ctrl.copy()
                viewer.sync()

                if drive:
                    drive.update(targets, now)
                if hand and now >= next_status:
                    print("\r" + status_line(hand, sim, mirror) + "   ", end="")
                    next_status = now + 0.25
                time.sleep(max(0.0, frame - (time.perf_counter() - t_frame)))
    finally:
        if drive:
            hand.stop()
            print("\nHand stopped.")
        if hand:
            hand.close()
    return 0


def status_line(hand: RealHand, sim: SimHand, mirror: MirrorBridge | None) -> str:
    if mirror:
        fb = mirror.feedback
        online = [f for f in fb if f.online]
        load = max((abs(f.load) for f in online), default=0.0)
        return f"mirroring: {len(online)}/{len(fb)} servos online, max load {load:4.1f} %"
    error = np.degrees(np.abs(hand.positions() - sim.positions())).max()
    line = f"real vs sim: max difference {error:5.1f} deg"
    if hand.offline_mask:
        line += f", offline servos: {hand.offline_mask.bit_count()}"
    return line


if __name__ == "__main__":
    sys.exit(main())
