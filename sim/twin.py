"""Digital twin v1: move the MuJoCo sliders and the real hand follows.

    uv run python sim/twin.py                 # simulation only
    uv run python sim/twin.py --fake          # with a software ESP32 (no hardware needed)
    uv run python sim/twin.py --port auto     # with the real hand (ESP32 auto-detected)
    uv run python sim/twin.py --port COM5     # with the real hand on a specific port

Use the sliders under "Control" (right panel). The joint targets are sent to the hand at most
`--rate` times per second, and only when they change. The terminal shows how far the real
hand (its step counters) is from the simulation.

Before starting with a real hand: straighten all joints by hand and power up (or send Z).
Nothing is sent until a slider moves, so the hand never moves on start-up.
"""

import argparse
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np
from tendra import Hand, RealHand, SimHand
from tendra.fake_esp32 import FakeEsp32


class TwinBridge:
    """Forwards changed simulation targets to a hand, rate-limited."""

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


def connect(args) -> Hand | None:
    if args.fake:
        return RealHand(connection=FakeEsp32())
    if args.port:
        return RealHand(None if args.port == "auto" else args.port)
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="serial port of the ESP32, or 'auto'")
    parser.add_argument("--fake", action="store_true", help="use a software ESP32")
    parser.add_argument("--rate", type=float, default=20.0, help="max commands per second")
    parser.add_argument("--seconds", type=float, help="quit after this many seconds (testing)")
    args = parser.parse_args()

    sim = SimHand()
    model, data = sim.model, sim.data
    hand = connect(args)
    bridge = TwinBridge(hand, args.rate) if hand else None
    if hand:
        print(f"Connected: {hand.firmware_info}")
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
                with viewer.lock():
                    for _ in range(steps_per_frame):
                        mujoco.mj_step(model, data)
                    targets = data.ctrl.copy()
                viewer.sync()

                if bridge:
                    bridge.update(targets, now)
                    if now >= next_status:
                        error = np.degrees(np.abs(hand.positions() - sim.positions())).max()
                        print(f"\rreal vs sim: max difference {error:5.1f} deg   ", end="")
                        next_status = now + 0.25
                time.sleep(max(0.0, frame - (time.perf_counter() - t_frame)))
    finally:
        if hand:
            hand.stop()
            hand.close()
            print("\nHand stopped.")


if __name__ == "__main__":
    sys.exit(main())
