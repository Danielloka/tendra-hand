"""Episode recording, loading and offline rendering (tendra.dataset).

Uses a tiny self-made scene (free-floating box, a mocap "wrist", two cameras), so the tests
don't depend on the grasp scene or the hand model.
"""

import json
import time

import mujoco
import numpy as np
import pytest
from tendra.dataset import (
    EPISODE_KEYS,
    Dataset,
    EpisodeRecorder,
    episode_file,
    load_episode,
    render_episode,
    write_preview,
)

SCENE = """
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <light pos="0 0 2" castshadow="true"/>
    <geom name="floor" type="plane" size="1 1 0.1" rgba="0.8 0.8 0.8 1"/>
    <camera name="front" pos="0 -1 0.5" xyaxes="1 0 0 0 0.45 0.9"/>
    <body name="wrist" mocap="true" pos="0 0 0.3">
      <geom type="sphere" size="0.04" rgba="0.2 0.4 0.9 1" contype="0" conaffinity="0"/>
      <camera name="wrist_cam" pos="0 0 -0.05" euler="0 0 0"/>
    </body>
    <body name="box" pos="0.1 0 0.1">
      <freejoint/>
      <geom type="box" size="0.03 0.03 0.03" rgba="0.9 0.2 0.2 1"/>
    </body>
    <body name="arm" pos="-0.2 0 0.2">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.1 0 0" size="0.01"/>
    </body>
  </worldbody>
  <actuator><position joint="hinge" kp="1"/></actuator>
</mujoco>
"""
FPS = 30.0
NAMES = ["hinge"]


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_string(SCENE)


def record(rec: EpisodeRecorder, model, n: int, task="push the box", success=True, seed=0):
    """Simulate n frames at FPS, moving the mocap wrist and the hinge. Returns the frames."""
    rng = np.random.default_rng(seed)
    data = mujoco.MjData(model)
    steps = round(1 / FPS / model.opt.timestep)
    rec.start_episode(task, extra={"seed": seed})
    ref = []
    for i in range(n):
        data.ctrl[0] = np.sin(i / 5)
        data.mocap_pos[0] = [0.1 * np.sin(i / 7), 0, 0.3]
        for _ in range(steps):
            mujoco.mj_step(model, data)
        extras = {"object.pos": data.xpos[2].copy(), "human.landmarks": rng.random((21, 3))}
        state, action = data.qpos[7:8], data.ctrl[:1]
        rec.add_frame(data, state=state, action=action, extras=extras)
        ref.append((data.qpos.copy(), data.mocap_pos.copy(), extras, state.copy()))
    path = rec.end_episode(success)
    return path, ref


def test_round_trip(tmp_path, model):
    rec = EpisodeRecorder(tmp_path, model, FPS, NAMES, NAMES, info={"scene": "test"})
    path, ref = record(rec, model, 12)
    assert path == episode_file(tmp_path, 0) and path.exists()
    ep = load_episode(path)
    assert set(ep) == set(EPISODE_KEYS) | {"object.pos", "human.landmarks"}
    assert ep["qpos"].shape == (12, model.nq) and ep["qpos"].dtype == np.float64
    assert ep["mocap_quat"].shape == (12, 1, 4)
    assert ep["observation.state"].dtype == np.float32 and ep["action"].shape == (12, 1)
    assert ep["human.landmarks"].shape == (12, 21, 3)
    assert np.all(np.diff(ep["time"]) > 0)
    for i, (qpos, mocap, extras, state) in enumerate(ref):
        assert np.array_equal(ep["qpos"][i], qpos)
        assert np.array_equal(ep["mocap_pos"][i], mocap)
        assert np.array_equal(ep["object.pos"][i], extras["object.pos"])
        assert np.array_equal(ep["human.landmarks"][i], extras["human.landmarks"])
        assert np.array_equal(ep["observation.state"][i], state.astype(np.float32))


def test_metadata(tmp_path, model):
    rec = EpisodeRecorder(tmp_path, model, FPS, NAMES, NAMES, info={"hand": "v1"})
    record(rec, model, 9, task="lift", success=False)
    info = json.loads((tmp_path / "info.json").read_text())
    assert info["fps"] == FPS and info["state_names"] == NAMES and info["state_dim"] == 1
    assert info["cameras"] == ["front", "wrist_cam"] and info["info"] == {"hand": "v1"}
    assert (tmp_path / info["model_file"]).exists()
    ds = Dataset(tmp_path)
    (meta,) = list(ds)
    assert meta["task"] == "lift" and meta["success"] is False and meta["length"] == 9
    assert meta["duration"] == pytest.approx(9 / FPS) and meta["extra"] == {"seed": 0}
    assert ds.filter(success=True) == [] and ds.filter(success=False) == [meta]
    assert ds.load_model().nq == model.nq


def test_numbering_continues_and_discard(tmp_path, model):
    rec = EpisodeRecorder(tmp_path, model, FPS, NAMES, NAMES)
    record(rec, model, 3)
    rec.start_episode("oops")
    rec.add_frame(mujoco.MjData(model), np.zeros(1), np.zeros(1))
    rec.discard_episode()
    assert not rec.recording
    record(rec, model, 3)
    assert rec.num_episodes == 2

    rec2 = EpisodeRecorder(tmp_path, model, FPS)  # a new session on the same dataset
    assert rec2.num_episodes == 2
    path, _ = record(rec2, model, 4, success=False)
    assert path == episode_file(tmp_path, 2)
    ds = Dataset(tmp_path)
    assert [e["episode_index"] for e in ds] == [0, 1, 2]
    assert len(ds.filter(success=True)) == 2
    assert len(ds.load(2)["time"]) == 4


def test_rejects_mismatch(tmp_path, model):
    EpisodeRecorder(tmp_path, model, FPS, NAMES, NAMES)
    with pytest.raises(ValueError, match="fps"):
        EpisodeRecorder(tmp_path, model, 15.0)
    with pytest.raises(ValueError, match="state_names"):
        EpisodeRecorder(tmp_path, model, FPS, state_names=["other"])
    other = mujoco.MjModel.from_xml_string(SCENE.replace('size="0.04"', 'size="0.05"'))
    with pytest.raises(ValueError, match="different MuJoCo model"):
        EpisodeRecorder(tmp_path, other, FPS)


def test_crash_leaves_no_partial_files(tmp_path, model, monkeypatch):
    rec = EpisodeRecorder(tmp_path, model, FPS, NAMES, NAMES)
    record(rec, model, 5)
    before = (tmp_path / "episodes.jsonl").read_bytes()

    # Crash in the middle of writing episode 1.
    def boom(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(np, "savez_compressed", boom)
    with pytest.raises(KeyboardInterrupt):
        record(rec, model, 5)
    monkeypatch.undo()

    assert (tmp_path / "episodes.jsonl").read_bytes() == before
    assert not episode_file(tmp_path, 1).exists()
    assert len(Dataset(tmp_path)) == 1 and len(load_episode(episode_file(tmp_path, 0))["qpos"]) == 5
    # Reopening cleans the leftover temp file and continues numbering.
    rec2 = EpisodeRecorder(tmp_path, model, FPS)
    assert list((tmp_path / "episodes").glob("*.tmp")) == []
    assert record(rec2, model, 2)[0] == episode_file(tmp_path, 1)


def test_add_frame_is_cheap(tmp_path, model):
    rec = EpisodeRecorder(tmp_path, model, FPS)  # dims taken from the first episode
    data = mujoco.MjData(model)
    state, landmarks = np.zeros(21), np.zeros((21, 3))  # v1 hand sizes
    rec.start_episode("speed")
    n = 2000
    t0 = time.perf_counter()
    for i in range(n):
        data.time = i / FPS
        rec.add_frame(data, state, state, extras={"human.landmarks": landmarks})
    us = (time.perf_counter() - t0) / n * 1e6
    t0 = time.perf_counter()
    rec.end_episode(True)
    write_ms = (time.perf_counter() - t0) * 1e3
    print(f"add_frame: {us:.1f} us/frame; end_episode ({n} frames): {write_ms:.0f} ms")
    assert us < 200


@pytest.mark.filterwarnings("ignore:data.time did not increase")
def test_extras_must_be_consistent(tmp_path, model):
    rec = EpisodeRecorder(tmp_path, model, FPS, NAMES, NAMES)
    data = mujoco.MjData(model)
    rec.start_episode("x")
    rec.add_frame(data, np.zeros(1), np.zeros(1))
    with pytest.raises(ValueError, match="first frame"):
        rec.add_frame(data, np.zeros(1), np.zeros(1), extras={"late": np.zeros(3)})
    rec.discard_episode()
    rec.start_episode("x")
    rec.add_frame(data, np.zeros(1), np.zeros(1), extras={"obj": np.zeros(3)})
    rec.add_frame(data, np.zeros(1), np.zeros(1))  # missing extra
    with pytest.raises(ValueError, match="frames"):
        rec.end_episode(True)


# --- Rendering -----------------------------------------------------------------------------


@pytest.fixture(scope="module")
def recorded(tmp_path_factory, model):
    try:
        mujoco.Renderer(model, 32, 32).close()
    except Exception as e:  # noqa: BLE001 (no OpenGL context, e.g. headless CI)
        pytest.skip(f"no OpenGL rendering: {e}")
    root = tmp_path_factory.mktemp("render")
    rec = EpisodeRecorder(root, model, FPS, NAMES, NAMES)
    record(rec, model, 6)
    return root


def test_render_episode(recorded):
    t0 = time.perf_counter()
    frames = render_episode(recorded, 0, ["front", "wrist_cam"], size=(64, 48))
    ms = (time.perf_counter() - t0) / 12 * 1e3
    print(f"render: {ms:.1f} ms per camera image (64x48, incl. setup)")
    assert set(frames) == {"front", "wrist_cam"}
    for v in frames.values():
        assert v.shape == (6, 48, 64, 3) and v.dtype == np.uint8
    assert frames["front"].std() > 0  # not a blank image
    # The wrist moves, so the front view changes over the episode.
    assert not np.array_equal(frames["front"][0], frames["front"][-1])
    again = render_episode(recorded, 0, ["front", "wrist_cam"], size=(64, 48))
    for cam in frames:
        assert np.array_equal(frames[cam], again[cam])


def test_render_unknown_camera(recorded):
    with pytest.raises(ValueError, match="camera"):
        render_episode(recorded, 0, ["nope"])


def test_preview(recorded, tmp_path):
    frames = render_episode(recorded, 0, size=(64, 48))  # all cameras
    path = write_preview(frames, tmp_path / "prev" / "ep0.mp4", FPS)
    assert path.exists() and path.stat().st_size > 0
