"""Make the files for the Colab GPU benchmark (`warp_benchmark.py`):

    uv run python sim/colab/make_bench_file.py

writes to runs/colab/ (git-ignored):
- `tendra_warp_bench.npz`: the compiled grasp scene (meshes included) + one scripted cylinder
  grasp (states and commands), so Colab needs nothing else from this repo;
- `warp_benchmark.ipynb`: the notebook (installs MuJoCo Warp, asks for the .npz, runs
  warp_benchmark.py for 256, 1024, 4096 parallel hands).
Upload the notebook to Google Colab (File > Upload notebook), choose a T4 GPU runtime
(Runtime > Change runtime type), run all cells, and upload the .npz when asked.
"""

import json
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "runs" / "colab"


def make_bench_npz(path: Path) -> Path:
    from tendra.grasp_env import GraspEnv

    env = GraspEnv(seed=0)
    demo = next(d for d in env.record_scripted_demos(6, seed=1) if d.obj == "cylinder")
    env.scene.reset(np.random.default_rng(0), obj="cylinder")  # active-object flags in the model
    mjb = path.with_suffix(".mjb")
    mujoco.mj_saveModel(env.model, str(mjb), None)
    np.savez_compressed(
        path, model=np.frombuffer(mjb.read_bytes(), np.uint8), qpos=demo.qpos, qvel=demo.qvel,
        ctrl=demo.ctrl, mocap_pos=demo.mocap_pos, mocap_quat=demo.mocap_quat,
        substeps=env.substeps, obj_z_adr=env.scene._obj_qpos["cylinder"] + 2,
    )  # fmt: skip
    mjb.unlink()
    return path


def cell(kind: str, text: str) -> dict:
    c = {"cell_type": kind, "metadata": {}, "source": text.strip("\n").splitlines(keepends=True)}
    if kind == "code":
        c.update(outputs=[], execution_count=None)
    return c


def make_notebook(path: Path) -> Path:
    script = (Path(__file__).parent / "warp_benchmark.py").read_text(encoding="utf-8")
    cells = [
        cell("markdown", """
# Tendra Hand: GPU physics benchmark (MuJoCo Warp)

How fast does the grasp scene run on a GPU? This runs many copies of the Tendra V1 hand grasping
and lifting a cylinder in parallel and reports **control steps per second** (the laptop's CPU
does ~270-650). Made by `sim/colab/make_bench_file.py` in the Tendra repo.

1. *Runtime > Change runtime type > T4 GPU*, then *Runtime > Run all*.
2. Upload `tendra_warp_bench.npz` (from `runs/colab/` in the repo) when the upload button appears.
"""),
        cell("code", """
!nvidia-smi --query-gpu=name,memory.total --format=csv
!pip install -q "mujoco==3.14.*" "mujoco-mjx==3.14.*" warp-lang mujoco-warp "jax[cuda12]"
"""),
        cell("code", """
import os
if not os.path.exists("tendra_warp_bench.npz"):
    from google.colab import files
    files.upload()  # choose tendra_warp_bench.npz
"""),
        cell("code", "%%writefile warp_benchmark.py\n" + script),
        cell("code", "!python warp_benchmark.py tendra_warp_bench.npz 256 1024 4096 2>&1 | grep -v '^Module'"),
        cell("markdown", """
Copy the lines with **worlds** back to Claude. `lifted` should be 100% (the physics still holds
the grasp); `control steps/s` is the speed. If a size fails (out of memory), the smaller ones
still tell us a lot.
"""),
    ]  # fmt: skip
    nb = {
        "cells": cells, "nbformat": 4, "nbformat_minor": 5,
        "metadata": {"accelerator": "GPU", "colab": {"provenance": []},
                     "kernelspec": {"name": "python3", "display_name": "Python 3"}},
    }  # fmt: skip
    path.write_text(json.dumps(nb, indent=1), encoding="utf-8")
    return path


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    npz = make_bench_npz(OUT / "tendra_warp_bench.npz")
    nb = make_notebook(OUT / "warp_benchmark.ipynb")
    print(f"{npz} ({npz.stat().st_size / 1e6:.1f} MB)\n{nb}")


if __name__ == "__main__":
    main()
