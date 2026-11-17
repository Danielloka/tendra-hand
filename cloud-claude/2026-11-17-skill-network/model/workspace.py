"""Workspace analysis: where can the right hand put its palm, facing down, over the table?

    uv run python model/workspace.py                # analysis on model/models/v21_robot.xml, prints a report,
                                                    # writes model/renders/workspace.png
    uv run python model/workspace.py --height 0.10  # palm height above the table top (m)

For every point of an (x, y) grid over the table, damped-least-squares IK (`build_robot.solve_arm_pose`) puts the
palm centre there with the palm facing down and the fingers pointing forward (+x) turned by a yaw in YAWS. A point
counts as reachable when IK converges inside the joint limits (arm joints and wrist) and the posed robot touches
nothing but itself's allowed neighbours (no table, floor or self-collision). Output: the reachable area, its
extent, and the largest axis-aligned rectangle (>= 40 x 30 cm wanted for pick and place).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_robot as br  # noqa: E402

ROOT = Path(__file__).resolve().parent
RENDERS = ROOT / "renders"
YAWS_DEG = (-30.0, -15.0, 0.0, 15.0, 30.0)
STEP = 0.025  # grid spacing (m)




def analyse(model, height: float = br.PALM_HEIGHT, x_range=(0.10, 0.90), y_range=(-0.60, 0.30), yaws_deg=YAWS_DEG,
            step: float = STEP, side: str = br.SIDE) -> dict:
    xs = np.arange(x_range[0], x_range[1] + 1e-9, step)
    ys = np.arange(y_range[0], y_range[1] + 1e-9, step)
    names = br.arm_joint_names(side)
    jid = np.array([model.joint(n).id for n in names])
    qadr = model.jnt_qposadr[jid]
    data = mujoco.MjData(model)
    z = br.TABLE_TOP + height
    q_nom = np.array([0.3, 0.17, 0.0, 1.6, 1.0, 0.0, 0.0])
    reach = np.zeros((len(yaws_deg), len(xs), len(ys)), bool)
    for k, yaw in enumerate(yaws_deg):
        rot = br.palm_target_rotation(math.radians(yaw))
        for i, x in enumerate(xs):
            prev = q_nom
            for j, y in enumerate(ys):
                q, err = br.solve_arm_pose(model, np.array([x, y, z]), rot, side, q_nom=prev, seeds=5, iters=120)
                if q is None or err > 2e-3:
                    continue
                data.qpos[:] = model.qpos0
                data.qpos[qadr] = q
                mujoco.mj_forward(model, data)
                if data.ncon:
                    continue
                reach[k, i, j] = True
                prev = q
    union = reach.any(axis=0)
    return {"xs": xs, "ys": ys, "reach": reach, "yaws_deg": list(yaws_deg), "union": union, "straight": reach[list(yaws_deg).index(0.0)], "height": height}


def largest_rectangles(mask: np.ndarray, xs, ys, min_w=0.40, min_h=0.30):
    """All maximal-ish axis-aligned rectangles fully reachable with side >= (min_w along x, min_h along y):
    returns the one with the largest area plus the list of top candidates [(area, x0, x1, y0, y1)]."""
    step = xs[1] - xs[0]
    out = []
    nx, ny = mask.shape
    for i0 in range(nx):
        for j0 in range(ny):
            if not mask[i0, j0]:
                continue
            for i1 in range(i0, nx):
                if not mask[i1, j0]:
                    break
                # extend in y as long as the whole row segment is reachable
                j1 = j0
                while j1 + 1 < ny and mask[i0 : i1 + 1, j1 + 1].all():
                    j1 += 1
                w, h = xs[i1] - xs[i0], ys[j1] - ys[j0]
                if w >= min_w - 1e-9 and h >= min_h - 1e-9:
                    out.append((w * h, xs[i0], xs[i1], ys[j0], ys[j1]))
    out.sort(reverse=True)
    return out


def save_png(res: dict, path: Path, table=None) -> None:
    from PIL import Image, ImageDraw

    xs, ys, mask, straight = res["xs"], res["ys"], res["union"], res["straight"]
    scale = 600 / (ys[-1] - ys[0] + 0.1)
    w = int((ys[-1] - ys[0] + 0.1) * scale)
    h = int((xs[-1] - xs[0] + 0.1) * scale)
    img = Image.new("RGB", (w + 20, h + 20), "white")
    dr = ImageDraw.Draw(img)

    def px(x, y):  # world (x forward, y left) -> image: forward up, left to the left
        return (10 + (ys[-1] + 0.05 - y) * scale, 10 + (xs[-1] + 0.05 - x) * scale)

    if table:
        cx, cy, hx, hy = table
        a, b = px(cx + hx, cy + hy), px(cx - hx, cy - hy)
        dr.rectangle([a, b], fill=(230, 215, 195), outline=(120, 90, 60))
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            if mask[i, j]:
                a, b = px(x + STEP / 2, y + STEP / 2), px(x - STEP / 2, y - STEP / 2)
                dr.rectangle([a, b], fill=(60, 160, 90) if straight[i, j] else (170, 210, 120))
    sx, sy = px(0.0, -0.0935)
    dr.ellipse([sx - 6, sy - 6, sx + 6, sy + 6], fill=(30, 30, 200))  # right shoulder
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=str(br.ROBOT_XML))
    ap.add_argument("--height", type=float, default=br.PALM_HEIGHT)
    ap.add_argument("--step", type=float, default=STEP)
    args = ap.parse_args(argv)
    model = mujoco.MjModel.from_xml_path(args.model)
    res = analyse(model, args.height, step=args.step)
    xs, ys = res["xs"], res["ys"]
    cell = args.step**2
    for name, m in (("palm down, fingers forward", res["straight"]), ("any yaw within +-30 deg", res["union"])):
        ii, jj = np.nonzero(m)
        print(f"{name}: {m.sum()} cells = {m.sum() * cell * 1e4:.0f} cm2; x {xs[ii].min():.3f}..{xs[ii].max():.3f}, "
              f"y {ys[jj].min():.3f}..{ys[jj].max():.3f}")
    for name, m in (("fingers forward", res["straight"]), ("yaw +-30", res["union"])):
        rects = largest_rectangles(m, xs, ys)
        if rects:
            a, x0, x1, y0, y1 = rects[0]
            print(f"largest rectangle ({name}): x {x0:.3f}..{x1:.3f} ({x1 - x0:.2f} m) y {y0:.3f}..{y1:.3f} ({y1 - y0:.2f} m)")
        else:
            print(f"no >= 40x30 cm rectangle ({name})")
    save_png(res, RENDERS / "workspace.png", (br.TABLE_CENTRE[0], br.TABLE_CENTRE[1], br.TABLE_HALF[0], br.TABLE_HALF[1]))
    print("wrote", RENDERS / "workspace.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
