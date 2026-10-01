"""Grasp teleoperation: pick up objects with the floating Tendra hand, and record demonstrations.

    uv run python sim/grasp_teleop.py                     # record into ~/tendra-data/datasets/grasp-sim
    uv run python sim/grasp_teleop.py --dataset my-demos  # another dataset
    uv run python sim/grasp_teleop.py --no-record         # just play
    uv run python sim/grasp_teleop.py --object cylinder   # always the same object

Your hand drives everything: the wrist pose comes from `tendra.wrist.WristTracker` (where your
palm is and how it's turned, from the webcam), the 16 finger servo joints from `tendra.retarget`.
The hand floats in `tendra.scene.GraspScene`: an invisible, ideal "arm" (a mocap weld) moves the
wrist, standing in for the real arm to come. The screen works like a mirror: move your hand
toward the webcam and the sim hand moves toward its camera.

Reaching is made easy: wherever your hand is when it's first seen becomes "home" (no fixed spot
to hold), the sim moves 1.6x your hand sideways/up-down and 2x toward/away from the camera
(`--gain`, `--depth-gain`), E re-centres at any time, and the camera view shows a box to keep
your hand in, with a warning near the edges.

Recording (`tendra.dataset.EpisodeRecorder`): only the simulation state is saved, 30 times per
simulated second (cheap); camera images are rendered afterwards by replaying it
(`uv run python sim/export_lerobot.py <dataset>`). Each frame stores
observation.state = 16 finger angles + wrist position (3) + wrist quaternion (4) and
action = 16 finger targets + wrist target position (3) + quaternion (4), plus your hand's
landmarks. An episode ends as a success once the object has been held up for `--hold` seconds.

Keys:
    R        start recording an episode / stop it (saved; success if the object is up)
    X        throw the current episode away
    N        new round: hand home, a new object at a new spot (throws a running episode away)
    E        re-centre: the sim hand stays put, your hand's current spot becomes the reference
             (use it whenever your hand gets near the edge of the camera image)
    W        wrist clutch on/off: while off, the sim wrist stays put and you can move your hand
             back to a comfortable spot (like lifting a mouse off its pad)
    C        calibrate the fingers (hand open and flat)
    SPACE    pause           M  flip palm side          Q / Esc  quit
"""

import argparse
import sys
import time

import cv2
import numpy as np
from teleop import TrackingWorker, text_panel
from tendra.dataset import EpisodeRecorder, default_dataset_root
from tendra.hand_tracking import CameraStream, draw_hand
from tendra.hand_view import HandView
from tendra.retarget import Retargeter
from tendra.scene import GraspScene, SceneConfig
from tendra.wrist import WristTracker

WINDOW = "Tendra grasp teleop"
HELP = "R rec  X discard  N new  E re-centre  W clutch  C calib  SPACE pause  M flip  Q quit"
CAMERA_SIZE = (640, 480)
LOOP_PERIOD = 1 / 60
DRAW_PERIOD = 1 / 25
MARGIN = 0.12  # comfort box: this fraction of the image is kept free on each side
NEAR, FAR = 0.30, 0.85  # m: camera -> palm distances where tracking gets poor


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--camera", type=int, default=0, help="webcam index")
    parser.add_argument("--dataset", default="grasp-sim", help="dataset name or folder")
    parser.add_argument("--no-record", action="store_true", help="don't open a dataset")
    parser.add_argument("--object", choices=SceneConfig().objects, help="always this object")
    parser.add_argument("--fps", type=float, default=30.0, help="recorded frames per sim second")
    parser.add_argument("--hold", type=float, default=1.0, help="s held up = success")
    parser.add_argument(
        "--gain",
        type=float,
        default=1.6,
        help="sim wrist motion per hand motion, sideways and up/down",
    )
    parser.add_argument("--depth-gain", type=float, default=2.0,
                        help="the same toward/away from the camera")  # fmt: skip
    parser.add_argument("--hfov", type=float, default=62.0, help="webcam horizontal FOV (deg)")
    parser.add_argument("--seed", type=int, help="random seed for object placement")
    parser.add_argument("--seconds", type=float, help="quit after this many seconds (testing)")
    args = parser.parse_args()

    print("Building the scene...")
    scene = GraspScene()
    rng = np.random.default_rng(args.seed)
    scene.reset(rng, args.object)
    model, data = scene.model, scene.data
    recorder = None
    if not args.no_record:
        root = default_dataset_root(args.dataset) if "/" not in args.dataset else args.dataset
        try:
            recorder = EpisodeRecorder(
                root, model, fps=args.fps, state_names=state_names(scene),
                action_names=action_names(scene),
                info={"hand": scene.spec.name, "scene": "grasp", "source": "sim/grasp_teleop.py"},
            )  # fmt: skip
        except ValueError as exc:  # e.g. the scene changed since this dataset was started
            print(f"Can't use dataset {root}: {exc}\nUse --dataset <new name>.")
            return 1
        print(f"Dataset: {root} ({recorder.num_episodes} episodes so far)")

    retarget = Retargeter(scene.spec)
    wrist = WristTracker(CAMERA_SIZE, hfov_deg=args.hfov,
                         gain=(args.gain, args.gain, args.depth_gain))  # fmt: skip
    wrist.recenter()  # wherever the hand is first seen = home
    camera = CameraStream(args.camera, *CAMERA_SIZE)
    worker = TrackingWorker(camera, retarget)
    worker.start()
    view = HandView(model, camera="view", width=CAMERA_SIZE[1], height=CAMERA_SIZE[1])
    view.mirror = False  # the scene camera already is the mirror
    print("Show your hand to the camera. Keys: " + HELP)

    state = LoopState()
    t_start = time.perf_counter()
    t_drawn = -1.0
    hand_image = None
    frame_period = 1.0 / args.fps
    try:
        while True:
            t_loop = time.perf_counter()
            now = t_loop - t_start
            if (args.seconds and now > args.seconds) or worker.error:
                break

            # 1. New hand reading -> wrist pose + finger targets.
            result = worker.result
            if result is not None and result.seq != state.seen:
                state.seen = result.seq
                if result.hand is not None and not state.paused:
                    pose = wrist.update(result.hand, result.t_capture, retarget.chirality)
                    scene.set_wrist_from_view(pose.pos, pose.rot)
                    scene.set_finger_targets(result.targets)
                    state.landmarks = result.hand.world
                    state.distance = pose.distance
                state.latency = 0.8 * state.latency + 0.2 * (result.t_done - result.t_capture)

            # 2. Physics in steps of one recorded frame, catching up with the wall clock.
            if not state.paused:
                behind = min(now - state.sim_clock, 0.15)
                while behind >= frame_period:
                    scene.step(frame_period)
                    state.sim_clock += frame_period
                    behind -= frame_period
                    if recorder and recorder.recording:
                        record_frame(recorder, scene, state)
                        check_success(recorder, scene, state, args.hold, frame_period)
                state.sim_clock = max(state.sim_clock, now - 0.15)
            else:
                state.sim_clock = now

            # 3. Draw (the scene camera costs ~25 ms, so at most DRAW_PERIOD).
            if now - t_drawn >= DRAW_PERIOD and result is not None:
                hand_image = view.render(data)
                t_drawn = now
                show(result, hand_image, scene, wrist, recorder, state, worker.rate)

            # 4. Keys.
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            handle_key(key, scene, wrist, worker, recorder, state, rng, args.object)
            if hand_image is not None and cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            time.sleep(max(0.0, LOOP_PERIOD - (time.perf_counter() - t_loop)))
    finally:
        if recorder and recorder.recording:
            recorder.discard_episode()
            print("Unfinished episode discarded.")
        worker.stop()
        camera.close()
        view.close()
        cv2.destroyAllWindows()
        if recorder:
            print(f"Dataset now has {recorder.num_episodes} episodes.")
    if worker.error:
        print(f"Tracking stopped: {worker.error}")
        return 1
    return 0


class LoopState:
    def __init__(self) -> None:
        self.seen = 0  # last hand reading used
        self.paused = False
        self.latency = 0.0
        self.sim_clock = 0.0  # wall-clock time the physics has reached
        self.landmarks = np.zeros((21, 3))  # last human landmarks (recorded as an extra)
        self.held_for = 0.0  # s the object has been up without a break
        self.message = ""  # last event, shown on screen
        self.successes = 0
        self.distance = 0.0  # camera -> palm (m), from the wrist tracker


def state_names(scene: GraspScene) -> list[str]:
    return [*scene.spec.joint_names, "wrist_x", "wrist_y", "wrist_z",
            "wrist_qw", "wrist_qx", "wrist_qy", "wrist_qz"]  # fmt: skip


def action_names(scene: GraspScene) -> list[str]:
    return [f"{n}_target" for n in state_names(scene)]


def record_frame(recorder: EpisodeRecorder, scene: GraspScene, state: LoopState) -> None:
    pos, quat = scene.wrist_pose()
    target_pos, target_quat = scene.wrist_target()
    obs = np.concatenate([scene.finger_positions(), pos, quat])
    action = np.concatenate([scene.finger_targets(), target_pos, target_quat])
    obj_pos, obj_quat = scene.object_pose()
    extras = {"human.landmarks": state.landmarks, "object.pos": obj_pos, "object.quat": obj_quat}
    recorder.add_frame(scene.data, obs, action, extras)


def check_success(recorder: EpisodeRecorder, scene: GraspScene, state: LoopState, hold: float,
                  dt: float) -> None:  # fmt: skip
    """End the episode as a success once the object has been held up for `hold` seconds."""
    state.held_for = state.held_for + dt if scene.lifted() else 0.0
    if state.held_for >= hold:
        path = recorder.end_episode(success=True)
        state.successes += 1
        state.held_for = 0.0
        state.message = f"SUCCESS - saved {path.name}. N for a new round."


def handle_key(key: int, scene: GraspScene, wrist: WristTracker, worker: TrackingWorker,
               recorder: EpisodeRecorder | None, state: LoopState, rng, obj) -> None:  # fmt: skip
    if key == ord(" "):
        state.paused = not state.paused
    elif key == ord("m"):
        worker.request("flip")
    elif key == ord("c"):
        worker.request("calibrate")
    elif key == ord("e"):
        wrist.recenter()
        state.message = "re-centred: carry on from here"
    elif key == ord("w"):
        wrist.set_engaged(not wrist.engaged)
        state.message = "wrist follows your hand" if wrist.engaged else "wrist clutch OFF"
    elif key == ord("n"):
        if recorder and recorder.recording:
            recorder.discard_episode()
        scene.reset(rng, obj)
        # The sim hand is home again: continue from there, wherever the real hand is now.
        wrist.reset()
        wrist.recenter()
        state.held_for = 0.0
        state.message = f"new round: {scene.active_object}"
    elif key == ord("r") and recorder:
        if not recorder.recording:
            recorder.start_episode(f"pick up the {scene.active_object}",
                                   extra={"object": scene.active_object})  # fmt: skip
            state.held_for = 0.0
            state.message = "recording..."
        else:
            success = scene.lifted()
            path = recorder.end_episode(success=success)
            state.successes += success
            state.message = f"saved {path.name} ({'success' if success else 'failure'})"
    elif key == ord("x") and recorder and recorder.recording:
        recorder.discard_episode()
        state.message = "episode discarded"


def show(result, hand_image, scene: GraspScene, wrist: WristTracker,
         recorder: EpisodeRecorder | None, state: LoopState, rate: float) -> None:  # fmt: skip
    frame = result.frame.copy()
    if result.hand is not None:
        draw_hand(frame, result.hand)
    frame = cv2.flip(frame, 1)
    warning = reach_warning(result.hand, state.distance, frame.shape)
    draw_comfort_box(frame, warning is not None)
    track = "hand seen" if result.hand is not None else "no hand - holding"
    clutch = "wrist ON" if wrist.engaged else "wrist OFF (W)"
    lines = [
        (f"tracking {rate:4.1f} fps  delay {state.latency * 1000:3.0f} ms  {track}  {clutch}",
         (120, 230, 150) if result.hand is not None else (60, 190, 255)),
        (HELP, (235, 235, 235)),
    ]  # fmt: skip
    if state.paused:
        lines.insert(0, ("PAUSED (space)", (90, 90, 255)))
    text_panel(frame, lines, top=True)

    if recorder is None:
        rec = "not recording (--no-record)"
    elif recorder.recording:
        rec = f"REC  {state.held_for:3.1f} s held"
    else:
        rec = f"R to record  |  {recorder.num_episodes} episodes, {state.successes} successes now"
    up = "object UP" if scene.lifted() else "object down"
    bottom = [(f"{scene.active_object}: {up}   {rec}",
               (90, 90, 255) if recorder and recorder.recording else (235, 235, 235))]  # fmt: skip
    if warning:
        bottom.append((warning, (60, 190, 255)))
    elif state.message:
        bottom.append((state.message, (120, 230, 150)))
    text_panel(frame, bottom, top=False)

    if hand_image is not None and hand_image.shape[0] == frame.shape[0]:
        if recorder and recorder.recording:  # red frame while recording
            cv2.rectangle(hand_image, (0, 0), (hand_image.shape[1] - 1, hand_image.shape[0] - 1),
                          (0, 0, 255), 4)  # fmt: skip
        frame = np.hstack([frame, hand_image])
    cv2.imshow(WINDOW, frame)


def reach_warning(hand, distance: float, shape) -> str | None:
    """A hint when the hand is about to leave the camera's good tracking area, else None."""
    if hand is None:
        return None
    h, w = shape[:2]
    x, y = hand.image[:, 0], hand.image[:, 1]  # unmirrored pixels; mirror x for the hint
    edge = []
    if x.max() > w * (1 - MARGIN):
        edge.append("left")  # right in the camera image = left on the mirrored screen
    if x.min() < w * MARGIN:
        edge.append("right")
    if y.min() < h * MARGIN:
        edge.append("top")
    if y.max() > h * (1 - MARGIN):
        edge.append("bottom")
    if edge:
        return f"hand near the {'/'.join(edge)} edge - press E to re-centre"
    if distance and distance < NEAR:
        return "hand too close to the camera - press E, then move back"
    if distance > FAR:
        return "hand far from the camera - press E, then come closer"
    return None


def draw_comfort_box(frame: np.ndarray, warn: bool) -> None:
    """The area to keep the whole hand in (thin, orange when the hand is near the edge)."""
    h, w = frame.shape[:2]
    color = (60, 190, 255) if warn else (170, 170, 170)
    x0, y0 = int(w * MARGIN), int(h * MARGIN)
    cv2.rectangle(frame, (x0, y0), (w - x0, h - y0), color, 2 if warn else 1, cv2.LINE_AA)


if __name__ == "__main__":
    sys.exit(main())
