"""Render the robot's cameras and an overview into model/renders/ (needs an OpenGL backend: MUJOCO_GL=egl|osmesa).

    uv run python model/render_views.py
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", os.environ["MUJOCO_GL"])

import mujoco
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_robot as br

RENDERS = Path(__file__).resolve().parent / "renders"


def render(model, data, camera, size, depth=False):
    r = mujoco.Renderer(model, *size)
    r.update_scene(data, camera=camera)
    rgb = r.render()
    out = rgb
    if depth:
        r.enable_depth_rendering()
        r.update_scene(data, camera=camera)
        out = (rgb, r.render())
    r.close()
    return out


def set_pose(model, key="table", **joints):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(key).id)
    for name, deg in joints.items():
        data.qpos[model.joint(f"right_{name}").qposadr[0]] = math.radians(deg) if not name.endswith("_rad") else deg
    mujoco.mj_forward(model, data)
    return data


def main() -> None:
    RENDERS.mkdir(exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(br.SCENE_XML))
    d = set_pose(model, "table")
    Image.fromarray(render(model, d, "overview", (720, 960))).save(RENDERS / "overview_table.png")
    d = set_pose(model, "ready")
    Image.fromarray(render(model, d, "overview", (720, 960))).save(RENDERS / "overview_ready.png")
    d = set_pose(model, "rest")
    Image.fromarray(render(model, d, "overview", (720, 960))).save(RENDERS / "overview_rest.png")
    d = set_pose(model, "table")
    rgb, depth = render(model, d, "head_cam", (720, 1280), depth=True)
    Image.fromarray(rgb).save(RENDERS / "head_cam.png")
    dn = np.clip((depth - 0.3) / 1.7, 0, 1)
    Image.fromarray((255 * (1 - dn)).astype(np.uint8)).save(RENDERS / "head_cam_depth.png")
    # wrist camera at several wrist poses, open and closed fingers, stitched into one sheet
    tiles = []
    for flex, dev, closed in [(0, 0, 0), (-45, 0, 0), (60, 0, 0), (0, -25, 0), (0, 18, 0), (0, 0, 1.2), (40, 0, 1.2),
                              (-45, -20, 0)]:  # fmt: skip
        d = set_pose(model, "table", wrist_flex=flex, wrist_dev=dev)
        if closed:
            for f in ("index", "middle", "ring", "little"):
                d.qpos[model.joint(f"right_{f}_mcp_flex").qposadr[0]] = closed
                d.qpos[model.joint(f"right_{f}_pip").qposadr[0]] = closed
            d.qpos[model.joint("right_thumb_cmc_rot").qposadr[0]] = 0.6
            mujoco.mj_forward(model, d)
        img = Image.fromarray(render(model, d, "right_wrist_cam", (480, 640)))
        Image.Image.paste  # noqa: B018
        tiles.append(img)
    w, h = tiles[0].size
    sheet = Image.new("RGB", (w * 4, h * 2))
    for i, t in enumerate(tiles):
        sheet.paste(t, ((i % 4) * w, (i // 4) * h))
    sheet.save(RENDERS / "wrist_cam_sheet.png")
    d = set_pose(model, "table")
    Image.fromarray(render(model, d, "right_wrist_cam", (480, 640))).save(RENDERS / "wrist_cam_table.png")
    print("renders in", RENDERS)


if __name__ == "__main__":
    main()
