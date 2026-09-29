"""Export recorded teleop episodes (tendra.dataset) to a Hugging Face LeRobot dataset.

    # LeRobot runs in a throwaway environment; it is not a project dependency (it pulls torch).
    uv run --with "lerobot[dataset]" python sim/export_lerobot.py ~/tendra-data/datasets/grasp \
        --repo-id tendra/grasp-sim --out ~/tendra-data/lerobot/grasp-sim

    # Only MP4 previews (no LeRobot needed), written to <dataset>/previews/:
    uv run python sim/export_lerobot.py ~/tendra-data/datasets/grasp --preview

Options: --cameras front wrist (default: every camera in the model) · --size 224 224 (width
height) · --all (also failed episodes; default: successful only) · --images (store PNG frames
instead of videos) · --overwrite (replace --out) · --max-episodes N (quick tests).

Written against LeRobot 0.6.1 (dataset format v3.0): `LeRobotDataset.create(repo_id, fps,
features, root, robot_type, use_videos)`, then per frame `add_frame({...features, "task": str})`,
per episode `save_episode()`, and at the end `finalize()` (required in v3: it writes the parquet
footers). Timestamps are computed by LeRobot as frame_index / fps. Tested end to end on
Windows with a small synthetic dataset (2026-09-29); videos are AV1 (libsvtav1, LeRobot's
default). Messages about torchcodec DLLs failing to load are harmless: LeRobot falls back to
PyAV for decoding.

Features: `observation.state` and `action` (float32, with joint names from info.json) and
`observation.images.<camera>` (video, H x W x 3). Images are rendered offline by replaying the
recorded simulation states (`tendra.dataset.render_episode`).
"""

import argparse
import shutil
import sys
import time
from pathlib import Path

import numpy as np
from tendra.dataset import Dataset, render_episode, write_preview


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("dataset", type=Path, help="recorded dataset folder (has info.json)")
    p.add_argument("--repo-id", default="tendra/grasp-sim", help="LeRobot repo id (user/name)")
    p.add_argument("--out", type=Path, help="output folder (default ~/tendra-data/lerobot/<name>)")
    p.add_argument("--cameras", nargs="+", help="cameras to render (default: all)")
    p.add_argument("--size", nargs=2, type=int, default=(224, 224), metavar=("W", "H"))
    p.add_argument("--all", action="store_true", help="include failed episodes")
    p.add_argument("--preview", action="store_true", help="only write MP4 previews")
    p.add_argument("--images", action="store_true", help="store PNG images, not videos")
    p.add_argument("--overwrite", action="store_true", help="delete --out first if it exists")
    p.add_argument("--max-episodes", type=int, help="export at most this many episodes")
    p.add_argument("--robot-type", help="default: tendra_hand_<hand> from the dataset info")
    return p.parse_args(argv)


def select_episodes(ds: Dataset, args: argparse.Namespace) -> list[dict]:
    episodes = list(ds) if args.all else ds.filter(success=True)
    if args.max_episodes is not None:
        episodes = episodes[: args.max_episodes]
    return episodes


def cameras_of(ds: Dataset, args: argparse.Namespace) -> list[str]:
    cameras = args.cameras or ds.info["cameras"]
    if not cameras:
        sys.exit("the model has no cameras; add some to the scene or pass --cameras")
    return list(cameras)


def build_features(ds: Dataset, cameras: list[str], size: tuple[int, int], video: bool) -> dict:
    info = ds.info
    w, h = size
    features = {}
    for key, dim_key, names_key in (
        ("observation.state", "state_dim", "state_names"),
        ("action", "action_dim", "action_names"),
    ):
        dim = info[dim_key]
        names = info[names_key] or [f"{key.split('.')[-1]}_{i}" for i in range(dim)]
        # Shapes must be tuples: LeRobot compares them with ndarray.shape.
        features[key] = {"dtype": "float32", "shape": (dim,), "names": list(names)}
    for cam in cameras:
        features[f"observation.images.{cam}"] = {
            "dtype": "video" if video else "image",
            "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        }
    return features


def export_previews(ds: Dataset, episodes: list[dict], args: argparse.Namespace) -> None:
    cameras = cameras_of(ds, args)
    out = args.out or ds.root / "previews"
    for meta in episodes:
        i = meta["episode_index"]
        t0 = time.perf_counter()
        frames = render_episode(ds.root, i, cameras, tuple(args.size))
        path = write_preview(frames, out / f"episode_{i:06d}.mp4", ds.fps)
        dt = time.perf_counter() - t0
        print(f"episode {i}: {meta['length']} frames, {meta['task']!r} -> {path} ({dt:.1f} s)")


def export_lerobot(ds: Dataset, episodes: list[dict], args: argparse.Namespace) -> Path:
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:
        sys.exit(
            "LeRobot is not installed. Run this script with\n"
            '  uv run --with "lerobot[dataset]" python sim/export_lerobot.py ...'
        )

    fps = ds.fps
    if abs(fps - round(fps)) > 1e-9:
        sys.exit(f"LeRobot needs an integer fps, the dataset has {fps}")
    cameras = cameras_of(ds, args)
    size = tuple(args.size)
    features = build_features(ds, cameras, size, video=not args.images)
    out = args.out or Path.home() / "tendra-data" / "lerobot" / args.repo_id.split("/")[-1]
    if out.exists():
        if not args.overwrite:
            sys.exit(f"{out} exists; pass --overwrite to replace it")
        shutil.rmtree(out)
    robot_type = args.robot_type or f"tendra_hand_{ds.info['info'].get('hand', 'v1')}"

    lr = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=round(fps),
        features=features,
        root=out,
        robot_type=robot_type,
        use_videos=not args.images,
    )
    try:
        for meta in episodes:
            i = meta["episode_index"]
            t0 = time.perf_counter()
            ep = ds.load(i)
            frames = render_episode(ds.root, i, cameras, size)
            t_render = time.perf_counter() - t0
            for t in range(meta["length"]):
                frame = {
                    "observation.state": ep["observation.state"][t].astype(np.float32),
                    "action": ep["action"][t].astype(np.float32),
                    "task": meta["task"],
                }
                for cam in cameras:
                    frame[f"observation.images.{cam}"] = frames[cam][t]
                lr.add_frame(frame)
            # Serial encoding: on Windows the process pool's startup cost ~20 s per episode,
            # versus ~2 s for encoding the videos one after another (measured on the i3).
            lr.save_episode(parallel_encoding=False)
            dt = time.perf_counter() - t0
            print(
                f"episode {i}: {meta['length']} frames, {meta['task']!r} "
                f"(render {t_render:.1f} s, total {dt:.1f} s)"
            )
    finally:
        lr.finalize()  # v3 datasets are invalid without it
    return out


def main(argv=None) -> None:
    args = parse_args(argv)
    ds = Dataset(args.dataset.expanduser())
    episodes = select_episodes(ds, args)
    kind = "all" if args.all else "successful"
    print(f"{ds.root}: {len(ds)} episodes, exporting {len(episodes)} ({kind})")
    if not episodes:
        sys.exit("nothing to export (use --all to include failed episodes)")
    if args.preview:
        export_previews(ds, episodes, args)
        return
    out = export_lerobot(ds, episodes, args)
    print(f"LeRobot dataset written to {out}")


if __name__ == "__main__":
    main()
