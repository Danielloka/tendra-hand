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
import build_robot as br

ROOT = Path(__file__).resolve().parent
RENDERS = ROOT / "renders"
YAWS_DEG = (-30.0, -15.0, 0.0, 15.0, 30.0)
STEP = 0.025  # grid spacing (m)
POS_TOL = 0.005  # palm centre within 5 mm of the grid point
ROT_TOL_DEG = 15.0  # palm frame within 15 deg of the target (palm down, fingers at the yaw)




def analyse(model, height: float = br.PALM_HEIGHT, x_range=(0.10, 0.65), y_range=(-0.60, 0.20), yaws_deg=YAWS_DEG,
            step: float = STEP, side: str = br.SIDE, pitches_deg=(0.0,), rot_tol_deg: float = ROT_TOL_DEG) -> dict:
    xs = np.arange(x_range[0], x_range[1] + 1e-9, step)
    ys = np.arange(y_range[0], y_range[1] + 1e-9, step)
    names = br.arm_joint_names(side)
    jid = np.array([model.joint(n).id for n in names])
    qadr = model.jnt_qposadr[jid]
    data = mujoco.MjData(model)
    z = br.TABLE_TOP + height
    palm_site, palm_body = model.site(f"{side}_palm_centre").id, model.body(f"{side}_palm").id
    q_nom = np.array([0.3, 0.17, 0.0, 1.6, 1.0, 0.0, 0.0])
    combos = [(yaw, pitch) for pitch in pitches_deg for yaw in yaws_deg]
    reach = np.zeros((len(combos), len(xs), len(ys)), bool)
    for k, (yaw, pitch) in enumerate(combos):
        rot = br.palm_target_rotation(math.radians(yaw), math.radians(pitch))
        for i, x in enumerate(xs):
            prev = q_nom
            for j, y in enumerate(ys):
                target = np.array([x, y, z])
                q, _ = br.solve_arm_pose(model, target, rot, side, q_nom=prev, seeds=6, iters=120)
                if q is None:
                    continue
                data.qpos[:] = model.qpos0
                data.qpos[qadr] = q
                mujoco.mj_forward(model, data)
                pos_err = np.linalg.norm(data.site_xpos[palm_site] - target)
                cos_t = np.clip((np.trace(data.xmat[palm_body].reshape(3, 3) @ rot.T) - 1) / 2, -1, 1)
                if pos_err > POS_TOL or math.degrees(math.acos(cos_t)) > rot_tol_deg or data.ncon:
                    continue
                reach[k, i, j] = True
                prev = q
    union = reach.any(axis=0)
    return {"xs": xs, "ys": ys, "reach": reach, "yaws_deg": list(yaws_deg), "pitches_deg": list(pitches_deg), "union": union, "straight": reach[combos.index((0.0, 0.0))], "height": height}


def largest_rectangles(mask: np.ndarray, xs, ys, min_w=0.40, min_h=0.30):
    """Fully reachable axis-aligned rectangles that are at least 40 x 30 cm in either orientation
    (min_w x min_h along x/y, or the other way round), best first: [(area, x0, x1, y0, y1)]."""
    out = _rects(mask, xs, ys, min_w, min_h) + _rects(mask, xs, ys, min_h, min_w)
    out.sort(reverse=True)
    return out


def _rects(mask, xs, ys, min_w, min_h):
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


HEIGHTS = (0.10, 0.15, 0.20)  # palm centre heights above the table top that count for pick and place


def analyse_heights(model, heights=HEIGHTS, **kw) -> dict:
    """analyse() at several palm heights; "union" = reachable at any of them, "straight" likewise (yaw 0 only)."""
    runs = [analyse(model, h, **kw) for h in heights]
    out = dict(runs[0])
    out["union"] = np.any([r["union"] for r in runs], axis=0)
    out["straight"] = np.any([r["straight"] for r in runs], axis=0)
    out["per_height"] = {h: r["union"] for h, r in zip(heights, runs)}
    out["heights"] = list(heights)
    return out


def report(res: dict, step: float) -> None:
    xs, ys = res["xs"], res["ys"]
    cell = step**2
    for h, m in res["per_height"].items():
        r = largest_rectangles(m, xs, ys)
        print(f"palm {h * 100:.0f} cm above the table, yaw +-30: {m.sum() * cell * 1e4:.0f} cm2; best 40x30 rectangle: "
              + (f"x {r[0][1]:.3f}..{r[0][2]:.3f}, y {r[0][3]:.3f}..{r[0][4]:.3f}" if r else "none"))
    for name, m in (("fingers forward (yaw 0)", res["straight"]), (f"any yaw +-30, heights {res['heights']}", res["union"])):
        ii, jj = np.nonzero(m)
        print(f"{name}: {m.sum() * cell * 1e4:.0f} cm2; x {xs[ii].min():.3f}..{xs[ii].max():.3f}, y {ys[jj].min():.3f}..{ys[jj].max():.3f}")
    rects = largest_rectangles(res["union"], xs, ys)
    if rects:
        _, x0, x1, y0, y1 = rects[0]
        print(f"largest fully reachable rectangle: x {x0:.3f}..{x1:.3f} ({x1 - x0:.2f} m) y {y0:.3f}..{y1:.3f} ({y1 - y0:.2f} m)")
    else:
        print("no fully reachable rectangle of at least 40 x 30 cm")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=str(br.ROBOT_XML))
    ap.add_argument("--heights", type=float, nargs="*", default=list(HEIGHTS))
    ap.add_argument("--step", type=float, default=STEP)
    ap.add_argument("--yaws", type=float, nargs="*", default=list(YAWS_DEG), help="finger yaw angles to include (deg)")
    ap.add_argument("--pitches", type=float, nargs="*", default=[0.0], help="fingers-down tilts to include (deg)")
    ap.add_argument("--dump", default="", help="save the masks to this .npz")
    args = ap.parse_args(argv)
    model = mujoco.MjModel.from_xml_path(args.model)
    res = analyse_heights(model, tuple(args.heights), step=args.step, pitches_deg=tuple(args.pitches),
                          yaws_deg=tuple(args.yaws))  # fmt: skip
    if args.dump:
        np.savez(args.dump, union=res["union"], straight=res["straight"], xs=res["xs"], ys=res["ys"])
    report(res, args.step)
    save_png(res, RENDERS / "workspace.png", (br.TABLE_CENTRE[0], br.TABLE_CENTRE[1], br.TABLE_HALF[0], br.TABLE_HALF[1]))
    print("wrote", RENDERS / "workspace.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
