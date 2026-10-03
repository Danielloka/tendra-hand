"""Make the files for GPU training on Google Colab:

    uv run python sim/colab/make_train_notebook.py --run gpu3         # export bundle + pack
    uv run python sim/colab/make_train_notebook.py --keep-bundle      # repack code, same bundle

writes to runs/colab/ (git-ignored):
- `tendra_gpu_kit_<hash>.zip`: the bundle `tendra_gpu.npz` (`tendra.gpu.export`: model with
  contact sensors, constants, synergies, scripted demo states) + the standalone code in
  `software/tendra/gpu/` (`CODE` below: the env, the trainer, the curriculum, staggered resets, the
  policy loader, the video maker). The hash of the contents is in the name, and the notebook asks
  for exactly that name, so a stale kit on Drive is never reused;
- `train_grasp_gpu.ipynb`: the notebook. It installs the pinned MuJoCo Warp + Brax versions,
  checks the GPU, mounts Google Drive, takes the kit (from Drive, or an upload that is then kept
  on Drive), runs a 2-minute smoke test, starts `train_brax.py` in the background with all output
  on Drive (`MyDrive/tendra/runs/<run>`: checkpoints, curriculum, compile caches), plots the
  progress live, films the best policy and exports it for the laptop. A disconnected session
  resumes where it stopped by running the same cells again.
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
# Everything the kit carries besides the bundle. All standalone (no `tendra` imports).
CODE = (
    "grasp_env_jax.py",
    "train_brax.py",
    "curriculum.py",
    "stagger.py",
    "rollout_video.py",
    "policy.py",
)
# Exact pins from research/ai/gpu-training-research.md (checked on PyPI 2026-10-03): Brax 0.14.2
# calls jax.device_put_replicated, removed in jax 0.10; mujoco-mjx 3.14 pins warp-lang 1.17.0.
PIP = (
    '!pip install -q "jax[cuda12]==0.9.2" "brax==0.14.2" "mujoco==3.14.0" '
    '"mujoco-mjx==3.14.0" "mujoco-warp==3.14.0" "warp-lang==1.17.0" "playground==0.2.0" '
    '"orbax-checkpoint>=0.11.22" mediapy'
)

INTRO = """
# Tendra Hand: train grasping on a GPU

Two Tendra V1 arms with hands learn to pick up a cylinder, a cube and a ball by themselves:
reinforcement learning (Brax PPO), with the physics in MuJoCo Warp so that thousands of hands
practise at the same time on one GPU. Made by `sim/colab/make_train_notebook.py` in the Tendra
repo; design: `research/ai/grasp-rl.md`.

**What happens, in plain words.** The trainer plays thousands of short grasp attempts in parallel
(a *chunk* is a few million of them), tests the policy on fresh attempts, and then decides by
itself how hard to make the next chunk: first only the cylinder; once it works, the cube, then the
ball; then fewer "helped" starts and stronger penalties for sloppy moves. Every decision is
printed and saved (`curriculum.json`), and the weights are saved after every test, so nothing is
lost if Colab disconnects.

**Which GPU?** *Runtime > Change runtime type*:
- **T4 (free tier)**: works, about 4,000 simulation steps per second, so ~100 M steps take 7 h.
  Free sessions last up to ~12 h but can end earlier, and a GPU is not always available.
- **L4 (Colab Pro)**: the sweet spot. Newer, 24 GB, so the trainer uses 4,096 worlds; costs
  about 1.4x the T4 in compute units.
- **A100 (Colab Pro, 40 GB)**: 8,192 worlds, the fastest, but about 4-5x the T4's compute units.
  Worth it only for long runs.
(Unit prices are approximate; check *Resources* in Colab. The trainer picks the number of worlds
for you.)

**Steps.** 1. Run the cells from the top (*Runtime > Run all* works, but read the smoke test).
2. Allow Google Drive access when asked. 3. The first time, upload `KIT_NAME` (repo:
`runs/colab/`); it is then kept on Drive. If the session disconnects, run the cells again:
training resumes from the last save.
"""

CHECK_GPU = """
import subprocess, sys
r = subprocess.run([sys.executable, "-c",
    "import jax; d = jax.devices(); print(jax.__version__, d); "
    "raise SystemExit(0 if d[0].platform == 'gpu' else 3)"])
if r.returncode:
    raise SystemExit("No GPU found. Choose Runtime > Change runtime type > T4 GPU (or L4 / A100), "
                     "then run the install cell again.")
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
RUN = DRIVE + "/runs/RUN_NAME"
os.makedirs(RUN, exist_ok=True)
print(sorted(os.listdir("/content")))
"""

ABOUT_SMOKE = """
**Smoke test (1-3 minutes).** A tiny run with 8 worlds that goes through everything once:
compile, train, evaluate, save, change the curriculum, export the policy. If this fails, stop
here and fix it; a real run would fail the same way, only later. The first compile of the real
run takes about 5 minutes (it is cached on Drive afterwards, so a reconnect is fast).
"""

SMOKE = """
!cd /content && python -u train_brax.py --bundle "$BUNDLE" --out /content/smoke_run --smoke 2>&1 | grep --line-buffered -v -E "^Module|^Warp|^   "
"""

ABOUT_TRAINING = """
**Training.** Starts `train_brax.py` in the background (so you can close the plot cell without
stopping it). `HOURS` is the time budget: the trainer stops cleanly after a save before it runs
out (Colab's hard limit is 12 h, keep some margin; on a free session that tends to end sooner,
the save after every test covers that). Leave `ENVS = 0` to let the trainer pick the number of
parallel worlds from the GPU. `success` in the log is the share of test episodes (normal starts,
no demo help) that lifted the object 6 cm and held it 0.5 s.

If the log says it rolled back: the test returned NaN or the physics blew up, so the trainer went
back to the last good weights and halved the learning rate (once). After 3 rollbacks it stops.
"""

TRAIN = """
import os, subprocess
HOURS = 10.5   # time budget for this session
ENVS = 0       # 0 = pick from the GPU (T4 2048, L4 4096, A100 8192)
EXTRA = ""     # e.g. "--chunk-steps 4000000" or "--no-stagger"
LOG = RUN + "/train.log"
running = subprocess.run(["pgrep", "-f", "train_brax.py"], capture_output=True).stdout.strip()
if running:
    print("training is already running; use the live-plot cell below")
else:
    cmd = (f'cd /content && exec python -u train_brax.py --bundle "{BUNDLE}" --out "{RUN}" '
           f'--hours {HOURS} --envs {ENVS} {EXTRA}')
    subprocess.Popen(["bash", "-c", cmd], stdout=open(LOG, "a"), stderr=subprocess.STDOUT)
    print("started; log:", LOG)
"""

ABOUT_PLOT = """
**Live plot.** Refreshes every minute and shows the success rate, the lift height and the return,
with a vertical line wherever the curriculum changed something (and what), plus the end of the
log. Stop the cell (square button) whenever you like: training keeps running. It ends by itself
when training stops.
"""

PLOT = """
import glob, json, subprocess, time
import pandas as pd
import matplotlib.pyplot as plt
from IPython.display import clear_output

def show():
    try:
        df = pd.read_csv(RUN + "/progress.csv")
        df = df[df["kind"] == "eval"]
    except Exception:
        print("no evaluation yet (the first one comes after the compile, ~10 min)"); return
    fig, ax = plt.subplots(1, 3, figsize=(16, 3.8))
    for a, col in zip(ax, ["eval/episode_success", "eval/episode_lift_cm", "eval/episode_reward"]):
        a.plot(df["steps"] / 1e6, df[col]); a.set_title(col); a.set_xlabel("M steps")
        a.grid(alpha=0.3)
    try:
        log = json.load(open(RUN + "/curriculum.json"))["log"]
        for e in log:
            ch = e.get("change")
            if e["event"] == "decision" and ch:
                for a in ax:
                    a.axvline(e["steps"] / 1e6, color="gray", ls=":", lw=1)
                ax[0].text(e["steps"] / 1e6, 0.02, ch["what"].replace("_scale", "")[:5],
                           rotation=90, fontsize=7, color="gray")
    except Exception:
        pass
    plt.tight_layout(); plt.show()
    print(subprocess.run(["tail", "-n", "12", RUN + "/train.log"], capture_output=True, text=True).stdout)

while True:
    clear_output(wait=True); show()
    if not subprocess.run(["pgrep", "-f", "train_brax.py"], capture_output=True).stdout.strip():
        print("training is not running (finished, stopped or not started)"); break
    time.sleep(60)
"""

ABOUT_VIDEO = """
**Video.** Rolls out the best policy for a few episodes in the training simulation and films
it (camera "view", from the operator's side). Run it after training has stopped or while it
runs (it uses little GPU memory). The file is kept on Drive in `videos/`.
"""

VIDEO = """
import time, subprocess
from IPython.display import Video, display
os.makedirs(RUN + "/videos", exist_ok=True)
mp4 = RUN + "/videos/best_" + time.strftime("%Y%m%d_%H%M%S") + ".mp4"
env = "MUJOCO_GL=egl XLA_PYTHON_CLIENT_PREALLOCATE=false XLA_PYTHON_CLIENT_MEM_FRACTION=0.15"
!cd /content && {env} python rollout_video.py --bundle "$BUNDLE" --params "$RUN/params_best.pkl" --out "$mp4" --episodes 4 2>&1 | tail -n 5
display(Video(mp4, embed=True, width=640))
"""

ABOUT_EXPORT = """
**Export for the laptop.** `policy_best.npz` (written by the trainer, refreshed here) is the policy
as plain numbers: the laptop needs neither jax nor brax. Download it, put it in the Tendra repo's
`runs/` folder and load it with `tendra.gpu.policy.BraxPolicy.load("runs/policy_best.npz")`.
`params_best.pkl` has the full weights for resuming or later fine-tuning.
"""

EXPORT = """
!cd /content && python policy.py "$RUN/params_best.pkl" "$RUN/policy_best.npz"
from google.colab import files
files.download(RUN + "/policy_best.npz")
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
        cell(
            "markdown",
            "**1. Install** the exact library versions (about 2 minutes), then check "
            "that a GPU is attached.",
        ),
        cell("code", "!nvidia-smi --query-gpu=name,memory.total --format=csv\n" + PIP),
        cell("code", CHECK_GPU),
        cell(
            "markdown",
            "**2. Google Drive and the kit.** Results, checkpoints and compile caches "
            "are kept in `MyDrive/tendra/runs/RUN_NAME`.".replace("RUN_NAME", run),
        ),
        cell("code", fill(SETUP)),
        cell("markdown", ABOUT_SMOKE),
        cell("code", SMOKE),
        cell("markdown", ABOUT_TRAINING),
        cell("code", TRAIN),
        cell("markdown", ABOUT_PLOT),
        cell("code", PLOT),
        cell("markdown", ABOUT_VIDEO),
        cell("code", VIDEO),
        cell("markdown", ABOUT_EXPORT),
        cell("code", EXPORT),
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
    p.add_argument("--run", default="gpu3", help="run folder on Drive (new name for a new hand)")
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
