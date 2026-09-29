"""Webcam hand tracking with MediaPipe's hand-landmark network.

MediaPipe Hand Landmarker (Google, Apache-2.0) is two small neural networks: a palm detector
that finds the hand in the image, and a landmark model that predicts 21 keypoints on it, both as
image coordinates and as 3D "world" coordinates in metres around the hand's centre. It runs in
real time on a laptop CPU. The model file (~8 MB) is downloaded once to ~/.cache/tendra/.

    with HandTracker() as tracker:
        hand = tracker.detect(frame_bgr, timestamp_ms)   # TrackedHand or None
"""

import sys
import threading
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/"
    "float16/latest/hand_landmarker.task"
)
CACHE_DIR = Path.home() / ".cache" / "tendra"

# Bones between landmarks, for drawing.
CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11),
    (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19),
    (19, 20),
)  # fmt: skip


def model_path() -> Path:
    """The hand-landmarker model file, downloaded on first use."""
    path = CACHE_DIR / "hand_landmarker.task"
    if not path.exists():
        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        print(f"Downloading the MediaPipe hand model to {path} ...")
        tmp = path.with_suffix(".part")
        urllib.request.urlretrieve(MODEL_URL, tmp)
        tmp.replace(path)
    return path


@dataclass
class TrackedHand:
    world: np.ndarray  # 21 x 3, metres, around the hand's centre (for retargeting)
    image: np.ndarray  # 21 x 2, pixel coordinates in the frame (for drawing)
    label: str  # "Left" / "Right", as MediaPipe reports it (it assumes a mirrored image)
    score: float  # handedness confidence 0..1


class HandTracker:
    def __init__(self, min_confidence: float = 0.5):
        from mediapipe.tasks.python import vision
        from mediapipe.tasks.python.core.base_options import BaseOptions

        options = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model_path())),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=min_confidence,
            min_hand_presence_confidence=min_confidence,
            min_tracking_confidence=min_confidence,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)
        self._last_ms = -1

    def detect(self, frame_bgr: np.ndarray, timestamp_ms: int) -> TrackedHand | None:
        """Find the hand in a BGR frame (OpenCV). Timestamps must increase."""
        import cv2
        import mediapipe as mp

        timestamp_ms = max(int(timestamp_ms), self._last_ms + 1)
        self._last_ms = timestamp_ms
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(image, timestamp_ms)
        if not result.hand_world_landmarks:
            return None
        h, w = frame_bgr.shape[:2]
        world = np.array([(p.x, p.y, p.z) for p in result.hand_world_landmarks[0]])
        image_xy = np.array([(p.x * w, p.y * h) for p in result.hand_landmarks[0]])
        category = result.handedness[0][0]
        return TrackedHand(world, image_xy, category.category_name, category.score)

    def close(self) -> None:
        self._landmarker.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def open_camera(index: int = 0, width: int = 640, height: int = 480):
    """An OpenCV camera. On Windows DirectShow opens much faster than the default backend."""
    import cv2

    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always the newest frame: less lag
    if not cap.isOpened():
        raise RuntimeError(f"could not open camera {index}")
    return cap


class CameraStream:
    """Reads the camera in a background thread, so nobody waits for the next frame.

    Grabbing a frame blocks until the camera delivers one (~33 ms at 30 fps). Doing that in its
    own thread lets hand tracking work on one frame while the camera exposes the next.
    """

    def __init__(self, index: int = 0, width: int = 640, height: int = 480):
        self._cap = open_camera(index, width, height)
        self._cond = threading.Condition()
        self._frame = None
        self._time = 0.0
        self._seq = 0
        self.failed = False
        self._running = True
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while self._running:
            ok, frame = self._cap.read()
            with self._cond:
                if not ok:
                    self.failed = True
                    self._cond.notify_all()
                    return
                self._frame, self._time = frame, time.perf_counter()
                self._seq += 1
                self._cond.notify_all()

    def wait_newer(self, seq: int, timeout: float = 1.0):
        """(seq, frame, capture time) of the newest frame after `seq`, or None on timeout/failure."""
        with self._cond:
            self._cond.wait_for(lambda: self._seq > seq or self.failed, timeout)
            if self._seq <= seq:
                return None
            return self._seq, self._frame, self._time

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._cap.release()


def draw_hand(frame: np.ndarray, hand: TrackedHand, color=(80, 200, 120)) -> None:
    """Draw the 21 landmarks and bones on the frame (in place)."""
    import cv2

    pts = hand.image.astype(int)
    for a, b in CONNECTIONS:
        cv2.line(frame, tuple(pts[a]), tuple(pts[b]), color, 2, cv2.LINE_AA)
    for i, p in enumerate(pts):
        cv2.circle(frame, tuple(p), 5 if i in (4, 8, 12, 16, 20) else 3, (255, 255, 255), -1)
