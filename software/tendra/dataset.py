"""Record teleoperation demonstrations and replay them into camera images.

Rendering a camera costs ~20 ms on the owner's laptop, far too slow for two cameras at 30 fps
during teleop. So recording and rendering are split:

1. **During teleop** `EpisodeRecorder` stores only the simulation state of every frame (qpos,
   qvel, ctrl, mocap poses) plus the policy's state/action vectors. That costs microseconds.
2. **Afterwards** `render_episode` replays the states (set qpos + mocap, `mj_forward`) and
   renders the cameras. Kinematics are deterministic, so the images are exactly what the
   cameras would have seen.

    rec = EpisodeRecorder(default_dataset_root("grasp"), model, fps=30,
                          state_names=names, action_names=names, info={"hand": "v1"})
    rec.start_episode("pick up the red cube")
    ...  # every 1/30 s of sim time:
    rec.add_frame(data, state=q, action=q_target, extras={"object.pos": obj_pos})
    rec.end_episode(success=True)          # or rec.discard_episode()

    ds = Dataset(root)
    for meta in ds.filter(success=True):
        frames = render_episode(root, meta["episode_index"], ["front", "wrist"])

Dataset layout (format version 1):

    <root>/
      info.json            fps, state/action names and dims, model sizes, cameras, user info
      model.mjb            the compiled MuJoCo model (saved once), so replay needs no scene code
      episodes.jsonl       one JSON line per finished episode (task, success, length, ...)
      episodes/episode_000012.npz   compressed arrays, one row per frame (keys: EPISODE_KEYS
                                    plus the recorder's extras, e.g. "human.landmarks")

Every file is written to a temporary name and then renamed, so a crash (or Ctrl-C) during an
episode never leaves a half-written file and never touches earlier episodes.
"""

from __future__ import annotations

import json
import os
import re
import warnings
from collections.abc import Iterator, Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

FORMAT = "tendra-episodes"
FORMAT_VERSION = 1
MODEL_FILE = "model.mjb"
INFO_FILE = "info.json"
EPISODES_FILE = "episodes.jsonl"
EPISODES_DIR = "episodes"
# Arrays in every episode file. dtypes: time/qpos/qvel/ctrl/mocap_* float64 (MuJoCo's own),
# observation.state and action float32 (what a policy trains on).
EPISODE_KEYS = (
    "time",
    "qpos",
    "qvel",
    "ctrl",
    "mocap_pos",
    "mocap_quat",
    "observation.state",
    "action",
)
_EPISODE_RE = re.compile(r"episode_(\d{6})\.npz$")


def default_dataset_root(name: str) -> Path:
    """`~/tendra-data/datasets/<name>`: outside the git repo, because datasets get big."""
    return Path.home() / "tendra-data" / "datasets" / name


def episode_file(root: str | Path, index: int) -> Path:
    return Path(root) / EPISODES_DIR / f"episode_{index:06d}.npz"


def _now() -> str:
    return datetime.now(UTC).isoformat(timespec="seconds")


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "wb") as f:
        f.write(payload)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)  # atomic on Windows and POSIX


def _atomic_write_json(path: Path, obj: Any) -> None:
    _atomic_write_bytes(path, (json.dumps(obj, indent=2) + "\n").encode())


def _model_bytes(model: mujoco.MjModel) -> bytes:
    buf = np.zeros(mujoco.mj_sizeModel(model), np.uint8)
    mujoco.mj_saveModel(model, None, buf)
    return buf.tobytes()


def _read_jsonl(path: Path) -> list[dict]:
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


class EpisodeRecorder:
    """Stores simulation states frame by frame and writes one file per episode.

    Opening an existing dataset folder appends to it: episode numbers continue, and the model,
    fps and state/action names must match what the dataset was recorded with.
    """

    def __init__(
        self,
        root: str | Path,
        model: mujoco.MjModel,
        fps: float = 30.0,
        state_names: list[str] | None = None,
        action_names: list[str] | None = None,
        info: dict | None = None,
    ) -> None:
        self.root = Path(root)
        self.fps = float(fps)
        (self.root / EPISODES_DIR).mkdir(parents=True, exist_ok=True)
        for stale in (self.root / EPISODES_DIR).glob("*.tmp"):  # left by a crash
            stale.unlink()

        model_path = self.root / MODEL_FILE
        blob = _model_bytes(model)
        if model_path.exists():
            if model_path.read_bytes() != blob:
                raise ValueError(
                    f"{self.root} was recorded with a different MuJoCo model; "
                    "use a new dataset folder"
                )
        else:
            _atomic_write_bytes(model_path, blob)

        info_path = self.root / INFO_FILE
        if info_path.exists():
            self._info = json.loads(info_path.read_text())
            if self._info.get("format") != FORMAT:
                raise ValueError(f"{info_path} is not a {FORMAT} dataset")
            if abs(self._info["fps"] - self.fps) > 1e-9:
                raise ValueError(f"dataset fps is {self._info['fps']}, not {self.fps}")
            for key, names in (("state_names", state_names), ("action_names", action_names)):
                if names is not None and self._info.get(key) not in (None, list(names)):
                    raise ValueError(f"{key} differ from the dataset's {key}")
                if names is not None:
                    self._info[key] = list(names)
            if info:
                self._info["info"] = {**self._info.get("info", {}), **info}
        else:
            self._info = {
                "format": FORMAT,
                "format_version": FORMAT_VERSION,
                "created": _now(),
                "fps": self.fps,
                "model_file": MODEL_FILE,
                "nq": model.nq,
                "nv": model.nv,
                "nu": model.nu,
                "nmocap": model.nmocap,
                "cameras": [model.camera(i).name for i in range(model.ncam)],
                "state_names": list(state_names) if state_names is not None else None,
                "action_names": list(action_names) if action_names is not None else None,
                "state_dim": len(state_names) if state_names is not None else None,
                "action_dim": len(action_names) if action_names is not None else None,
                "info": dict(info or {}),
            }
        _atomic_write_json(info_path, self._info)

        self._episodes = _read_jsonl(self.root / EPISODES_FILE)
        # Next index: past both the log and any episode file (a file whose log line was lost in
        # a crash must never be overwritten).
        indices = [e["episode_index"] for e in self._episodes]
        indices += [
            int(m.group(1))
            for p in (self.root / EPISODES_DIR).iterdir()
            if (m := _EPISODE_RE.search(p.name))
        ]
        self._next_index = max(indices, default=-1) + 1
        self._frames: dict[str, list[np.ndarray]] | None = None
        self._task = ""
        self._extra: dict = {}
        self._started = ""
        self._last_time = -np.inf
        self._warned_time = False

    @property
    def recording(self) -> bool:
        return self._frames is not None

    @property
    def num_episodes(self) -> int:
        """Finished episodes in the dataset (including ones from earlier sessions)."""
        return len(self._episodes)

    @property
    def next_episode_index(self) -> int:
        return self._next_index

    def start_episode(self, task: str, extra: dict | None = None) -> None:
        if self.recording:
            raise RuntimeError("an episode is already being recorded; end or discard it first")
        self._frames = {k: [] for k in EPISODE_KEYS}
        self._task = task
        self._extra = dict(extra or {})
        self._started = _now()
        self._last_time = -np.inf

    def add_frame(
        self,
        data: mujoco.MjData,
        state: np.ndarray,
        action: np.ndarray,
        extras: dict[str, np.ndarray] | None = None,
    ) -> None:
        """Append one frame. Call it every 1/fps of simulation time. Costs a few µs."""
        f = self._frames
        if f is None:
            raise RuntimeError("start_episode() first")
        t = data.time
        if t <= self._last_time and not self._warned_time:
            # Kinematic-only loops (mj_forward, no mj_step) don't advance data.time. That's
            # fine: frame timestamps come from index / fps; data.time is only kept as a record.
            warnings.warn("data.time did not increase between frames", stacklevel=2)
            self._warned_time = True
        self._last_time = t
        f["time"].append(np.float64(t))
        f["qpos"].append(data.qpos.copy())
        f["qvel"].append(data.qvel.copy())
        f["ctrl"].append(data.ctrl.copy())
        f["mocap_pos"].append(data.mocap_pos.copy())
        f["mocap_quat"].append(data.mocap_quat.copy())
        f["observation.state"].append(np.array(state, np.float32))
        f["action"].append(np.array(action, np.float32))
        if extras:
            n = len(f["time"]) - 1
            for key, value in extras.items():
                lst = f.get(key)
                if lst is None:
                    if key in EPISODE_KEYS or n:
                        raise ValueError(f"extra {key!r} must be given from the first frame on")
                    lst = f[key] = []
                lst.append(np.array(value))
        # Missing extras / shape changes are caught in end_episode (np.stack), keeping this cheap.

    def discard_episode(self) -> None:
        self._frames = None

    def end_episode(self, success: bool) -> Path:
        """Write the episode (atomically) and log it. Returns the episode file's path."""
        f = self._frames
        if f is None:
            raise RuntimeError("no episode is being recorded")
        length = len(f["time"])
        if length == 0:
            raise ValueError("episode has no frames; use discard_episode()")
        arrays = {}
        for key, lst in f.items():
            if len(lst) != length:
                raise ValueError(f"{key!r} has {len(lst)} frames, expected {length}")
            try:
                arrays[key] = np.stack(lst)
            except ValueError as e:
                raise ValueError(f"{key!r} changed shape during the episode") from e
        for key, dim_key, names_key in (
            ("observation.state", "state_dim", "state_names"),
            ("action", "action_dim", "action_names"),
        ):
            dim = int(arrays[key].shape[1]) if arrays[key].ndim == 2 else 1
            if self._info[dim_key] is None:
                self._info[dim_key] = dim
                _atomic_write_json(self.root / INFO_FILE, self._info)
            elif self._info[dim_key] != dim:
                raise ValueError(f"{key} has {dim} values, dataset has {self._info[dim_key]}")
            names = self._info[names_key]
            if names is not None and len(names) != dim:
                raise ValueError(f"{key} has {dim} values but {len(names)} names")

        index = self._next_index
        path = episode_file(self.root, index)
        tmp = path.with_name(path.name + ".tmp")
        with open(tmp, "wb") as fh:
            np.savez_compressed(fh, **arrays)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)

        meta = {
            "episode_index": index,
            "file": f"{EPISODES_DIR}/{path.name}",
            "task": self._task,
            "success": bool(success),
            "length": length,
            "fps": self.fps,
            "duration": length / self.fps,
            "sim_time": [float(arrays["time"][0]), float(arrays["time"][-1])],
            "recorded": self._started,
            "extra": self._extra,
            "extras_keys": [k for k in arrays if k not in EPISODE_KEYS],
        }
        self._episodes.append(meta)
        # Rewrite the whole log atomically (small: one line per episode).
        payload = "".join(json.dumps(e) + "\n" for e in self._episodes).encode()
        _atomic_write_bytes(self.root / EPISODES_FILE, payload)
        self._next_index = index + 1
        self._frames = None
        return path


def load_episode(path: str | Path) -> dict[str, np.ndarray]:
    """All arrays of one episode file, keyed like EPISODE_KEYS (+ extras)."""
    with np.load(path) as z:
        return {k: z[k] for k in z.files}


class Dataset:
    """Read access to a recorded dataset: metadata, episodes and the model."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        info_path = self.root / INFO_FILE
        if not info_path.exists():
            raise FileNotFoundError(f"no {INFO_FILE} in {self.root}")
        self.info: dict = json.loads(info_path.read_text())
        # Only episodes whose file exists (the log line is written after the file).
        self.episodes: list[dict] = [
            e for e in _read_jsonl(self.root / EPISODES_FILE) if (self.root / e["file"]).exists()
        ]

    @property
    def fps(self) -> float:
        return float(self.info["fps"])

    def __len__(self) -> int:
        return len(self.episodes)

    def __iter__(self) -> Iterator[dict]:
        return iter(self.episodes)

    def filter(self, success: bool | None = None, task: str | None = None) -> list[dict]:
        """Episode metadata, optionally only (un)successful ones and/or one task."""
        return [
            e
            for e in self.episodes
            if (success is None or e["success"] == success) and (task is None or e["task"] == task)
        ]

    def meta(self, episode: int) -> dict:
        for e in self.episodes:
            if e["episode_index"] == episode:
                return e
        raise KeyError(f"episode {episode} not in {self.root}")

    def load(self, episode: int) -> dict[str, np.ndarray]:
        return load_episode(self.root / self.meta(episode)["file"])

    def load_model(self) -> mujoco.MjModel:
        return mujoco.MjModel.from_binary_path(str(self.root / self.info["model_file"]))


# --- Offline rendering ---------------------------------------------------------------------


def prepare_render_model(model: mujoco.MjModel, size: tuple[int, int]) -> mujoco.MjModel:
    """Cheap-to-draw settings (in place): no shadows or reflections, big enough framebuffer."""
    model.light_castshadow[:] = 0
    model.mat_reflectance[:] = 0
    w, h = size
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, w)
    model.vis.global_.offheight = max(model.vis.global_.offheight, h)
    return model


def render_options() -> mujoco.MjvOption:
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = 0  # ~2,000 tendon segments: very slow to draw
    return opt


def replay_frames(
    model: mujoco.MjModel, episode: dict[str, np.ndarray]
) -> Iterator[tuple[int, mujoco.MjData]]:
    """Yield (frame index, MjData) with each recorded state restored and `mj_forward` run."""
    data = mujoco.MjData(model)
    for i in range(len(episode["qpos"])):
        data.time = episode["time"][i]
        data.qpos[:] = episode["qpos"][i]
        data.qvel[:] = episode["qvel"][i]
        data.ctrl[:] = episode["ctrl"][i]
        if model.nmocap:
            data.mocap_pos[:] = episode["mocap_pos"][i]
            data.mocap_quat[:] = episode["mocap_quat"][i]
        mujoco.mj_forward(model, data)
        yield i, data


def render_episode(
    root: str | Path,
    episode: int,
    cameras: Sequence[str] | None = None,
    size: tuple[int, int] = (224, 224),
) -> dict[str, np.ndarray]:
    """Render named cameras for every frame of an episode.

    size = (width, height). cameras=None renders every camera in the model.
    Returns {camera: uint8 array (T, H, W, 3), RGB}.
    """
    ds = Dataset(root)
    model = prepare_render_model(ds.load_model(), size)
    ep = ds.load(episode)
    if cameras is None:
        cameras = [model.camera(i).name for i in range(model.ncam)]
    for cam in cameras:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam) < 0:
            raise ValueError(f"camera {cam!r} not in the model")
    w, h = size
    out = {cam: np.empty((len(ep["qpos"]), h, w, 3), np.uint8) for cam in cameras}
    opt = render_options()
    renderer = mujoco.Renderer(model, height=h, width=w)
    try:
        for i, data in replay_frames(model, ep):
            for cam in cameras:
                renderer.update_scene(data, camera=cam, scene_option=opt)
                renderer.render(out=out[cam][i])
    finally:
        renderer.close()
    return out


def write_preview(frames: dict[str, np.ndarray], path: str | Path, fps: float) -> Path:
    """Write an MP4 with the cameras side by side (for a quick look, not for training)."""
    import cv2  # only needed here

    videos = list(frames.values())
    if not videos:
        raise ValueError("no frames")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    h = max(v.shape[1] for v in videos)
    w = sum(v.shape[2] for v in videos)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    if not writer.isOpened():
        raise OSError(f"OpenCV could not open {path} for writing")
    try:
        canvas = np.zeros((h, w, 3), np.uint8)
        for i in range(len(videos[0])):
            x = 0
            for v in videos:
                canvas[: v.shape[1], x : x + v.shape[2]] = v[i]
                x += v.shape[2]
            writer.write(cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))
    finally:
        writer.release()
    return path
