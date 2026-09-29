"""End to end without a webcam: a scripted grasp stands in for the operator and goes through the
same recording code as `sim/grasp_teleop.py`, then the episode is replayed and rendered."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from grasp_teleop import (
    LoopState,
    action_names,
    check_success,
    handle_key,
    record_frame,
    state_names,
)
from tendra.dataset import Dataset, EpisodeRecorder, render_episode, write_preview
from tendra.scene import GraspScene, scripted_grasp

FPS = 30.0


class FakeWrist:
    engaged = True

    def set_engaged(self, engaged):
        self.engaged = engaged

    def reset(self):
        pass


@pytest.fixture(scope="module")
def recorded(tmp_path_factory):
    root = tmp_path_factory.mktemp("grasp-dataset")
    scene = GraspScene()
    rng = np.random.default_rng(0)
    scene.reset(rng, "cylinder")
    recorder = EpisodeRecorder(root, scene.model, fps=FPS, state_names=state_names(scene),
                               action_names=action_names(scene))  # fmt: skip
    state = LoopState()
    handle_key(ord("r"), scene, FakeWrist(), None, recorder, state, rng, "cylinder")
    assert recorder.recording

    # Route the scripted grasp's physics through the teleop recording loop, frame by frame.
    step, pending = scene.step, [0.0]

    def recording_step(seconds: float) -> None:
        pending[0] += seconds
        while pending[0] >= 1 / FPS - 1e-9:
            step(1 / FPS)
            pending[0] -= 1 / FPS
            if recorder.recording:
                record_frame(recorder, scene, state)
                check_success(recorder, scene, state, hold=1.0, dt=1 / FPS)

    scene.step = recording_step
    scripted_grasp(scene)
    return root, scene, state


def test_scripted_grasp_is_recorded_as_a_success(recorded):
    root, _, state = recorded
    assert state.successes == 1, state.message
    ds = Dataset(root)
    assert len(ds) == 1
    meta = ds.meta(0)
    assert meta["success"] and meta["task"] == "pick up the cylinder"
    ep = ds.load(0)
    n = len(ep["qpos"])
    assert n > 60  # a few seconds at 30 fps
    assert ep["observation.state"].shape == (n, 28) and ep["action"].shape == (n, 28)
    assert ep["human.landmarks"].shape == (n, 21, 3)
    # The object ends up higher than it started.
    assert ep["object.pos"][-1, 2] > ep["object.pos"][0, 2] + 0.04


def test_episode_replays_and_renders(recorded, tmp_path):
    root, _, _ = recorded
    frames = render_episode(root, 0, cameras=["view", "wrist"], size=(96, 96))
    n = len(Dataset(root).load(0)["qpos"])
    for cam in ("view", "wrist"):
        assert frames[cam].shape == (n, 96, 96, 3)
        assert frames[cam].std() > 5  # not a blank image
    path = write_preview(frames, tmp_path / "preview.mp4", FPS)
    assert path.exists() and path.stat().st_size > 1000
