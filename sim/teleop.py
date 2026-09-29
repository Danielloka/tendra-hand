"""Webcam teleoperation: move your own hand, and the Tendra digital twin copies it.

    uv run python sim/teleop.py                    # v1 hand (5 fingers)
    uv run python sim/teleop.py --hand v0          # v0: thumb + index
    uv run python sim/teleop.py --camera 1         # another webcam
    uv run python sim/teleop.py --physics          # simulate the servos pulling the tendons
    uv run python sim/teleop.py --tendons          # draw the 40 tendon strands (slower)
    uv run python sim/teleop.py --viewer           # MuJoCo's interactive viewer (slower)
    uv run python sim/teleop.py --fake             # also drive a software ESP32
    uv run python sim/teleop.py --port auto        # also drive the real hand (careful!)

Pipeline: webcam -> MediaPipe hand-landmark network (21 3D points) -> retargeting
(`tendra.retarget`: finger angles + thumb optimisation) -> smoothing (One Euro filter) ->
`SimHand`. With --port/--fake the same targets go to the hand, rate-limited, like
`sim/twin.py` does.

Built for a slow laptop whose CPU and integrated GPU share one power budget, so every bit of
drawing slows the hand tracking down (measured on the i3-1125G4, see research/log.md):
- **Threads:** the camera grabs frames, a tracking thread runs the network and retargeting on
  the newest frame, and the main loop draws.
- **One window:** the camera image and the hand side by side. The hand is drawn only when it
  moved, at most 30 times a second (`tendra.hand_view`). MuJoCo's own viewer (`--viewer`)
  redraws nonstop and cut tracking from 28 to 7 fps.
- **Kinematic mode:** each hand reading puts the joints straight at the tracked angles. No
  physics, no servo delay. `--physics` simulates the servos and tendons instead
  (realistic: the joints lag a little, like real servos).
- **Light model:** simplified meshes (`tendra.lite_model`, ~15 % of the triangles), no shadows
  or reflections, tendons hidden (their ~2,000 segments cost ~10x the whole hand to draw).

Tips: hold your hand 40-70 cm from the camera, palm toward it, in good light. Press C with the
hand open and flat, fingers together, to calibrate the zero pose.

Keys: C calibrate · SPACE pause/resume · M flip palm side · D joint angles · Q/Esc quit.
Mouse on the hand: drag to rotate, wheel to zoom.
Nothing is sent to a real hand while paused or while no hand is seen (it holds its last pose).
"""

import argparse
import contextlib
import sys
import threading
import time
from dataclasses import dataclass
from types import SimpleNamespace

import cv2
import mujoco
import mujoco.viewer
import numpy as np
from tendra import SimHand, get_hand
from tendra.hand_tracking import CameraStream, HandTracker, TrackedHand, draw_hand
from tendra.hand_view import HandView
from tendra.retarget import Retargeter
from twin import TwinBridge, connect

WINDOW = "Tendra teleop"
HELP = "C calibrate  SPACE pause  M flip palm  D angles  Q quit  |  drag/wheel: turn/zoom hand"
LOOP_PERIOD = 1 / 60  # main loop
DRAW_PERIOD = 1 / 30  # hand view: at most this often
CAMERA_SIZE = (640, 480)


@dataclass
class TrackResult:
    seq: int  # increases with every processed camera frame
    frame: np.ndarray  # camera image (BGR, not mirrored)
    hand: TrackedHand | None
    targets: np.ndarray | None  # joint targets, None if no hand was seen
    t_capture: float  # perf_counter time the frame arrived
    t_done: float  # perf_counter time the targets were ready


class TrackingWorker(threading.Thread):
    """Camera frame -> hand landmarks -> joint targets, as fast as the network allows.

    Owns the `Retargeter`; the main thread asks for calibration or a palm flip through
    `request()`, so the retargeter is only ever touched from this thread.
    """

    def __init__(self, camera: CameraStream, retarget: Retargeter):
        super().__init__(name="tracking", daemon=True)
        self.camera, self.retarget = camera, retarget
        self.result: TrackResult | None = None
        self.rate = 0.0  # processed frames per second
        self.error: BaseException | None = None
        self._requests: list[str] = []
        self._lock = threading.Lock()
        self._running = True

    def request(self, what: str) -> None:
        with self._lock:
            self._requests.append(what)

    def stop(self) -> None:
        self._running = False
        self.join(timeout=2.0)

    def run(self) -> None:
        try:
            with HandTracker() as tracker:
                self._loop(tracker)
        except Exception as exc:  # noqa: BLE001 - any failure is reported by the main thread
            self.error = exc

    def _loop(self, tracker: HandTracker) -> None:
        seq, t_last = 0, None
        t0 = time.perf_counter()
        while self._running:
            got = self.camera.wait_newer(seq, timeout=0.5)
            if got is None:
                if self.camera.failed:
                    raise RuntimeError("camera read failed")
                continue
            seq, frame, t_cap = got
            hand = tracker.detect(frame, (t_cap - t0) * 1000)
            with self._lock:
                requests, self._requests = self._requests, []
            for what in requests:
                if what == "flip":
                    self.retarget.flip()
                elif what == "calibrate" and hand:
                    self.retarget.calibrate(hand.world, hand.label)
                    print("Calibrated: this pose is now the open hand (q = 0).")
            targets = self.retarget(hand.world, t_cap, hand.label) if hand else None
            now = time.perf_counter()
            if t_last is not None:
                self.rate = 0.9 * self.rate + 0.1 / max(now - t_last, 1e-3)
            t_last = now
            self.result = TrackResult(seq, frame, hand, targets, t_cap, now)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--hand",
        default="auto",
        choices=["auto", "v0", "v1"],
        help="hand variant (auto: the connected hand's, else v1)",
    )
    parser.add_argument("--camera", type=int, default=0, help="webcam index")
    parser.add_argument(
        "--physics",
        action="store_true",
        help="simulate servos + tendons (else: joints follow directly)",
    )
    parser.add_argument("--tendons", action="store_true", help="draw the tendon strands")
    parser.add_argument("--viewer", action="store_true", help="use MuJoCo's interactive viewer")
    parser.add_argument("--full-meshes", action="store_true", help="don't simplify the meshes")
    parser.add_argument("--port", help="also drive the real hand: serial port or 'auto'")
    parser.add_argument("--fake", action="store_true", help="also drive a software ESP32")
    parser.add_argument("--rate", type=float, default=20.0, help="max commands/s to the hand")
    parser.add_argument("--seconds", type=float, help="quit after this many seconds (testing)")
    args = parser.parse_args()  # fmt: skip

    real = connect(SimpleNamespace(fake=args.fake, port=args.port, hand=args.hand))
    spec = real.spec if real else get_hand("v1" if args.hand == "auto" else args.hand)
    sim = SimHand(hand=spec, lite=not args.full_meshes)
    model, data = sim.model, sim.data
    model.light_castshadow[:] = 0  # shadows and reflections: extra render passes
    model.mat_reflectance[:] = 0
    if args.tendons:  # low-detail capsules: the strands are made of ~2,000 segments
        model.vis.quality.numslices, model.vis.quality.numstacks = 6, 2
    retarget = Retargeter(spec, model=model)
    drive = TwinBridge(real, args.rate) if real else None
    if real:
        print(f"Connected: {real.firmware_info}")

    camera = CameraStream(args.camera, *CAMERA_SIZE)
    worker = TrackingWorker(camera, retarget)
    worker.start()
    print(f"Hand {spec.name}: {spec.num_joints} joints. Show your hand to the camera.")
    print("Keys: " + HELP)

    view = None if args.viewer else HandView(model, height=CAMERA_SIZE[1], tendons=args.tendons)
    paused = False
    show_angles = False
    shown = 0  # seq of the last hand reading applied
    latency = 0.0
    loops = updates = draws = 0
    t_start = time.perf_counter()
    t_stats = None  # stats start with the first tracked frame (skips start-up)
    t_drawn = -1.0
    hand_image = None
    viewer_ctx = (
        mujoco.viewer.launch_passive(model, data) if args.viewer else contextlib.nullcontext()
    )
    try:
        with viewer_ctx as viewer:
            if viewer:
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = args.tendons
            if view:
                cv2.namedWindow(WINDOW)
                cv2.setMouseCallback(
                    WINDOW, lambda e, x, y, f, _: view.mouse(e, x - CAMERA_SIZE[0], y, f)
                )
            while viewer is None or viewer.is_running():
                t_loop = time.perf_counter()
                now = t_loop - t_start
                if (args.seconds and now > args.seconds) or worker.error:
                    break
                if t_stats is None and worker.result is not None:
                    t_stats = t_loop
                loops += t_stats is not None

                result = worker.result
                new = result is not None and result.seq != shown
                apply = new and result.targets is not None and not paused
                with viewer.lock() if viewer else contextlib.nullcontext():
                    moved = step(sim, result.targets if apply else None, now, args.physics)
                if viewer:
                    viewer.sync()

                if new:
                    updates += 1
                    shown = result.seq
                    latency = 0.8 * latency + 0.2 * (result.t_done - result.t_capture)
                    if drive and apply:
                        drive.update(result.targets, now)
                # The hand view: only when something changed, at most 30 times a second.
                if view and (moved or hand_image is None) and now - t_drawn >= DRAW_PERIOD:
                    # Mirror the robot (a right hand) only for a right hand, so it always looks
                    # like the hand in the (mirrored) camera image.
                    view.mirror = retarget.chirality is None or retarget.chirality > 0
                    hand_image = view.render(data)
                    t_drawn = now
                    draws += 1
                    new = True
                if new and result is not None:
                    angles = sim.targets() if show_angles else None
                    show(result, hand_image, retarget, paused, worker.rate, latency, angles)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord(" "):
                    paused = not paused
                elif key == ord("m"):
                    worker.request("flip")
                elif key == ord("d"):
                    show_angles = not show_angles
                elif key == ord("c"):
                    worker.request("calibrate")
                if shown and cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
                time.sleep(max(0.0, LOOP_PERIOD - (time.perf_counter() - t_loop)))
    finally:
        elapsed = max(time.perf_counter() - (t_stats or t_start), 1e-3)
        print(f"Main loop {loops / elapsed:.0f} Hz, hand drawn {draws / elapsed:.0f}/s, "
              f"hand updates {updates / elapsed:.1f}/s, tracking {worker.rate:.1f} fps, "
              f"delay {latency * 1000:.0f} ms")  # fmt: skip
        worker.stop()
        camera.close()
        if view:
            view.close()
        cv2.destroyAllWindows()
        if drive:
            real.stop()
            print("Hand stopped.")
        if real:
            real.close()
    if worker.error:
        print(f"Tracking stopped: {worker.error}")
        return 1
    return 0


def step(sim: SimHand, targets, now: float, physics: bool) -> bool:
    """Advance the twin to `now` (new `targets` if not None). Returns True if the hand moved."""
    model, data = sim.model, sim.data
    if physics:
        if targets is not None:
            sim.set_targets(targets)
        # Keep the sim clock in step with the wall clock (capped after stalls).
        behind = min(now - data.time, 0.1)
        for _ in range(max(0, round(behind / model.opt.timestep))):
            mujoco.mj_step(model, data)
        data.time = max(data.time, now - 0.1)
        return sim.is_moving()
    if targets is None:
        return False
    sim.set_positions(targets)
    return True


def show(result: TrackResult, hand_image, retarget: Retargeter, paused: bool, rate: float,
         latency: float, angles: np.ndarray | None = None) -> None:  # fmt: skip
    """The camera view (mirrored, like a mirror) with status, and the hand next to it."""
    frame = result.frame.copy()
    if result.hand:
        draw_hand(frame, result.hand)
    frame = cv2.flip(frame, 1)
    if result.hand:
        side = "right" if retarget.chirality > 0 else "left"  # camera image isn't mirrored
        status, color = f"{side} hand   pinch {retarget.pinch:.0%}", (120, 230, 150)
    else:
        status, color = "no hand - holding last pose", (60, 190, 255)
    if paused:
        status, color = "PAUSED (space to resume)", (90, 90, 255)
    lines = [
        (f"tracking {rate:4.1f} fps   delay {latency * 1000:3.0f} ms   {status}", color),
        (HELP, (235, 235, 235)),
    ]
    text_panel(frame, lines, top=True)
    if angles is not None:
        text_panel(frame, angle_lines(retarget, angles), top=False)
    if hand_image is not None and hand_image.shape[0] == frame.shape[0]:
        frame = np.hstack([frame, hand_image])
    cv2.imshow(WINDOW, frame)


def angle_lines(retarget: Retargeter, q: np.ndarray) -> list[tuple[str, tuple]]:
    """One line per finger with its joint targets in degrees (what the robot is told)."""
    names = retarget.spec.joint_names
    deg = dict(zip(names, np.degrees(q), strict=True))
    lines = []
    for finger in ("thumb", "index", "middle", "ring", "little"):
        joints = [n for n in names if n.startswith(finger + "_")]
        if joints:
            parts = [f"{n.removeprefix(finger + '_')} {deg[n]:4.0f}" for n in joints]
            lines.append((f"{finger:6s} " + "  ".join(parts), (235, 235, 235)))
    return lines


def text_panel(frame: np.ndarray, lines, top: bool) -> None:
    """Text on a dark, see-through bar, readable on any background (in place)."""
    font, scale, height = cv2.FONT_HERSHEY_SIMPLEX, 0.45, 19
    h = 8 + height * len(lines)
    y0 = 0 if top else frame.shape[0] - h
    bar = frame[y0 : y0 + h]
    bar[:] = (bar * 0.35).astype(frame.dtype)
    for i, (text, color) in enumerate(lines):
        y = y0 + 4 + height * (i + 1) - 5
        cv2.putText(frame, text, (8, y), font, scale, color, 1, cv2.LINE_AA)


if __name__ == "__main__":
    sys.exit(main())
