"""Make the files for GPU training on Google Colab:

    uv run python sim/colab/make_train_notebook.py --run gpu2         # export bundle + pack
    uv run python sim/colab/make_train_notebook.py --keep-bundle      # repack code, same bundle

writes to runs/colab/ (git-ignored):
- `tendra_gpu_kit_<hash>.zip`: the bundle `tendra_gpu.npz` (`tendra.gpu.export`: model with
  contact sensors, constants, synergies, scripted demo states) + `grasp_env_jax.py` +
  `train_brax.py`. The hash of the contents is in the name, and the notebook asks for exactly
  that name, so a stale kit on Drive is never reused;
- `train_grasp_gpu.ipynb`: a small notebook that installs MuJoCo Warp + Brax, mounts Google
  Drive, takes the kit (from Drive, or an upload that is then kept on Drive) and runs
  `train_brax.py` with all output on Drive (`MyDrive/tendra/runs/<run>`), so a disconnected
  session resumes where it stopped.
Re-run after changing the scene, the hand model or the GPU env; use a new `--run` name when the
hand changes (old weights don't fit a different hand).
"""

import argparse
import hashlib
import json
import zipfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "runs" / "colab"
GPU = REPO / "software" / "tendra" / "gpu"
DRIVE_DIR = "/content/drive/MyDrive/tendra"
CODE = ("grasp_env_jax.py", "train_brax.py")
# Brax 0.14 needs jax < 0.10 (jax.device_put_replicated was removed); tested with 0.9.
PIP = (
    '!pip install -q "mujoco==3.14.*" "mujoco-mjx==3.14.*" warp-lang mujoco-warp playground '
    'brax "jax[cuda12]==0.9.*"'
)

INTRO = """
# Tendra Hand: train grasping on a GPU

The Tendra V1 hand learns to pick up a cylinder, a cube and a ball by itself: reinforcement
learning (Brax PPO) with the physics in MuJoCo Warp, 2,048 hands in parallel on the free T4.
Made by `sim/colab/make_train_notebook.py` in the Tendra repo; design: `research/ai/grasp-rl.md`.

1. *Runtime > Change runtime type > T4 GPU*, then *Runtime > Run all*.
2. Allow Google Drive access when asked (results and checkpoints are saved there).
3. The first time, upload `KIT_NAME` (repo: `runs/colab/`); it is then kept on Drive.

If the session disconnects, run all cells again: training resumes from the last save.
"""

SETUP = """
import os, shutil, zipfile
from google.colab import drive
drive.mount("/content/drive")
DRIVE = "DRIVE_DIR"
os.makedirs(DRIVE, exist_ok=True)
KIT = os.path.join(DRIVE, "KIT_NAME")
if not os.path.exists(KIT):
    from google.colab import files
    uploaded = files.upload()  # choose KIT_NAME
    shutil.copy(next(iter(uploaded)), KIT)
zipfile.ZipFile(KIT).extractall("/content")
BUNDLE = "/content/tendra_gpu.npz"
print(sorted(os.listdir("/content")))
"""

ABOUT_TRAINING = """
Training: phase A (cylinder, 30 M steps), B (all objects, 60 M), C (full penalties, 60 M).
Change `RUN` for a fresh run, add `--scale 0.3` for a shorter one. Progress prints at every
evaluation (~every 7 minutes): `success` is the share of evaluation episodes (normal starts,
no demo help) that lifted the object 6 cm and held it 0.5 s.
"""

TRAIN = """
RUN = DRIVE + "/runs/RUN_NAME"
!cd /content && python -u train_brax.py --bundle "$BUNDLE" --out "$RUN" 2>&1 | grep --line-buffered -v -E "^Module|^Warp|^   "
"""

PLOT = """
import glob, pandas as pd, matplotlib.pyplot as plt
df = pd.concat([pd.read_csv(f) for f in sorted(glob.glob(RUN + "/phase_*/progress.csv"))])
fig, ax = plt.subplots(1, 3, figsize=(15, 3.5))
for a, col in zip(ax, ["eval/episode_success", "eval/episode_lift_cm", "eval/episode_reward"]):
    a.plot(df["steps"] / 1e6, df[col]); a.set_title(col); a.set_xlabel("M steps")
plt.show()
"""


def cell(kind: str, text: str) -> dict:
    c = {"cell_type": kind, "metadata": {}, "source": text.strip("\n").splitlines(keepends=True)}
    if kind == "code":
        c.update(outputs=[], execution_count=None)
    return c


def make_kit(out: Path, bundle: Path) -> Path:
    """Zip the bundle + code as `tendra_gpu_kit_<hash>.zip`; older kits in `out` are removed."""
    digest = hashlib.sha256(bundle.read_bytes())
    for name in CODE:
        digest.update((GPU / name).read_bytes())
    for old in out.glob("tendra_gpu_kit*.zip"):
        old.unlink()
    path = out / f"tendra_gpu_kit_{digest.hexdigest()[:8]}.zip"
    with zipfile.ZipFile(path, "w", zipfile.ZIP_STORED) as z:  # the .npz is compressed already
        z.write(bundle, "tendra_gpu.npz")
        for name in CODE:
            z.write(GPU / name, name)
    return path


def make_notebook(path: Path, kit: str, run: str) -> Path:
    def fill(text: str) -> str:
        return (
            text.replace("KIT_NAME", kit).replace("DRIVE_DIR", DRIVE_DIR).replace("RUN_NAME", run)
        )

    cells = [
        cell("markdown", fill(INTRO)),
        cell("code", "!nvidia-smi --query-gpu=name,memory.total --format=csv\n" + PIP),
        cell("code", fill(SETUP)),
        cell("markdown", ABOUT_TRAINING),
        cell("code", fill(TRAIN)),
        cell("code", PLOT),
    ]
    nb = {
        "cells": cells,
        "nbformat": 4,
        "nbformat_minor": 5,
        "metadata": {
            "accelerator": "GPU",
            "colab": {"provenance": []},
            "kernelspec": {"name": "python3", "display_name": "Python 3"},
        },
    }
    path.write_text(json.dumps(nb, indent=1), encoding="utf-8")
    return path


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--keep-bundle", action="store_true", help="reuse runs/colab/tendra_gpu.npz")
    p.add_argument("--run", default="gpu2", help="run folder on Drive (new name for a new hand)")
    args = p.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    bundle = OUT / "tendra_gpu.npz"
    if not args.keep_bundle:
        from tendra.gpu.export import export_bundle

        export_bundle(bundle)
    kit = make_kit(OUT, bundle)
    nb = make_notebook(OUT / "train_grasp_gpu.ipynb", kit.name, args.run)
    print(f"{kit} ({kit.stat().st_size / 1e6:.1f} MB)\n{nb}")


if __name__ == "__main__":
    main()
