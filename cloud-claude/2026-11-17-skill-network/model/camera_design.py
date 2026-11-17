"""Wrist-camera design tools: metrics used by the mount search and by tests/test_robot.py.

    uv run python model/camera_design.py            # search the mount (prints the best candidates)

A candidate mount = camera position in the forearm (= hand) frame, tilt toward the palm side and yaw toward the
thumb. For each one we measure, with the CAD `open` thumb and the wrist straight unless stated:
  * clearance: the housing's lowest point with the palm facing down (= its most negative hand-Y) must not be below
    the hand's lowest collision point (review B1: otherwise the camera is the first thing to touch the table);
  * grasp-space visibility: fraction of an 8 x 6 x 8 cm box in front of the palm (hand x +-40, y -30..-90,
    z 60..140 mm) that is inside the image and not hidden by the forearm, wrist ring or palm;
  * fingertip coverage: all five fingertips visible over the grasping wrist range (flex -30..40, dev -20..15 deg),
    open and closed fingers.
Optimised in that order: clearance (hard), grasp space, then fingertips.
"""

from __future__ import annotations

import itertools
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_robot as br

RAY_MASK = np.array([1, 1, 0, 1, 0, 1], dtype=np.uint8)  # skip groups 2 (visual) and 4 (camera housing, sites)
OCCLUDERS = ("right_forearm", "right_wrist_ring", "right_palm")


def set_cam(model, pos, tilt_deg: float, yaw_deg: float) -> None:
    """Move the wrist camera, its mount site and its housing geoms in `model` (arrays only, no recompile)."""
    tilt, yaw = math.radians(tilt_deg), math.radians(yaw_deg)
    direction = [math.sin(yaw) * math.cos(tilt), -math.sin(tilt), math.cos(yaw) * math.cos(tilt)]
    xy = np.array(br.cam_xyaxes_from_dir(direction, [0, 1, 0]))
    x, y = xy[:3], xy[3:]
    z = np.cross(x, y)
    rot = np.column_stack([x, y, z])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rot.ravel())
    cam = model.camera("right_wrist_cam").id
    model.cam_pos[cam], model.cam_quat[cam] = pos, quat
    site = model.site("right_wrist_cam_mount").id
    model.site_pos[site], model.site_quat[site] = pos, quat
    centre = np.asarray(pos) + z * br.CAM_HOUSING_HALF[2]
    for name in ("right_wrist_cam_housing", "right_wrist_cam_housing_vis"):
        g = model.geom(name).id
        model.geom_pos[g], model.geom_quat[g] = centre, quat


def hand_pose_data(model, flex_deg=0.0, dev_deg=0.0, closed=0.0, thumb_open=True) -> mujoco.MjData:
    d = mujoco.MjData(model)
    d.qpos[:] = model.qpos0
    if thumb_open:
        for j, v in br.open_pose().items():
            d.qpos[model.joint(f"right_{j}").qposadr[0]] = v
    d.qpos[model.joint("right_wrist_flex").qposadr[0]] = math.radians(flex_deg)
    d.qpos[model.joint("right_wrist_dev").qposadr[0]] = math.radians(dev_deg)
    if closed:
        for f in br.FINGERS:
            d.qpos[model.joint(f"right_{f}_mcp_flex").qposadr[0]] = closed
            d.qpos[model.joint(f"right_{f}_pip").qposadr[0]] = closed
            d.qpos[model.joint(f"right_{f}_dip").qposadr[0]] = 0.75 * closed
    mujoco.mj_kinematics(model, d)
    mujoco.mj_camlight(model, d)
    return d


def geom_vertices_world(model, data, g: int) -> np.ndarray:
    """World coordinates of a geom's extreme points: mesh vertices, box corners, else a bounding sphere's 6 poles."""
    R = data.geom_xmat[g].reshape(3, 3)
    p = data.geom_xpos[g]
    t = model.geom_type[g]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        mid = model.geom_dataid[g]
        a, n = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        local = model.mesh_vert[a : a + n] @ _mesh_rot(model, mid).T + model.mesh_pos[mid]
    elif t == mujoco.mjtGeom.mjGEOM_BOX:
        local = np.array(list(itertools.product(*[(-s, s) for s in model.geom_size[g]])))
    else:
        r = model.geom_rbound[g]
        local = np.array([[r, 0, 0], [-r, 0, 0], [0, r, 0], [0, -r, 0], [0, 0, r], [0, 0, -r]])
    return local @ R.T + p


def _mesh_rot(model, mid) -> np.ndarray:
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, model.mesh_quat[mid])
    return m.reshape(3, 3)


def hand_collision_geoms(model, side="right", exclude_housing=True) -> list[int]:
    out = []
    for g in range(model.ngeom):
        body = model.body(model.geom_bodyid[g]).name
        if not body.startswith(f"{side}_") or body == f"{side}_forearm_twist_link":
            continue
        if not (model.geom_contype[g] or model.geom_conaffinity[g]):
            continue
        if exclude_housing and "cam_housing" in model.geom(g).name:
            continue
        out.append(g)
    return out


def lowest_points(model, data, axis_world) -> tuple[float, float]:
    """(lowest hand collision point, lowest housing point) along -axis_world (smaller = lower), in metres."""
    axis = np.asarray(axis_world)
    hand = min((geom_vertices_world(model, data, g) @ axis).min() for g in hand_collision_geoms(model))
    house = (geom_vertices_world(model, data, model.geom("right_wrist_cam_housing").id) @ axis).min()
    return float(hand), float(house)


def clearance_hand_frame(model) -> float:
    """housing lowest - hand lowest along the palm-down direction (hand -Y), wrist straight, fingers open, the
    worse of the CAD open thumb and the straight thumb: >= 0 means the housing is never the lowest part. Metres."""
    worst = np.inf
    for thumb_open in (True, False):
        d = hand_pose_data(model, thumb_open=thumb_open)
        up = d.xmat[model.body("right_forearm").id].reshape(3, 3)[:, 1]  # hand +Y in the world = up when palm down
        hand, house = lowest_points(model, d, up)
        worst = min(worst, house - hand)
    return float(worst)


def housing_clear_of_hand(model, min_dist=0.002) -> bool:
    d = hand_pose_data(model)
    h = model.geom("right_wrist_cam_housing").id
    fromto = np.zeros(6)
    for g in hand_collision_geoms(model):
        if mujoco.mj_geomDistance(model, d, h, g, 0.05, fromto) < min_dist:
            return False
    return True


def grasp_space_visibility(model) -> float:
    d = hand_pose_data(model)
    cam = model.camera("right_wrist_cam").id
    fa = model.body("right_forearm").id
    R, p0 = d.xmat[fa].reshape(3, 3), d.xpos[fa]
    W, H = model.cam_resolution[cam]
    f = (H / 2) / math.tan(math.radians(model.cam_fovy[cam]) / 2)
    cam_R = d.cam_xmat[cam].reshape(3, 3)
    geom = np.zeros(1, dtype=np.int32)
    ok = tot = 0
    for x, y, z in itertools.product(np.linspace(-0.04, 0.04, 5), np.linspace(-0.03, -0.09, 4), np.linspace(0.06, 0.14, 5)):
        p = p0 + R @ [x, y, z]
        rel = cam_R.T @ (p - d.cam_xpos[cam])
        depth = -rel[2]
        good = depth > 0.02 and 0 < W / 2 + f * rel[0] / depth < W and 0 < H / 2 - f * rel[1] / depth < H
        if good:
            vec = p - d.cam_xpos[cam]
            dist = np.linalg.norm(vec)
            hit = mujoco.mj_ray(model, d, d.cam_xpos[cam], vec / dist, RAY_MASK, 1, -1, geom)
            if 0 <= hit < dist - 0.003 and model.body(model.geom_bodyid[geom[0]]).name in OCCLUDERS:
                good = False
        ok += good
        tot += 1
    return ok / tot


def tip_coverage(model, flexes_deg, devs_deg, closed=0.0, margin=0.04) -> float:
    """Fraction of wrist poses where all five fingertip sites are in the image and not hidden by the forearm, wrist
    ring or palm."""
    tips = [model.site(f"right_{f}_tip").id for f in (*br.FINGERS, "thumb")]
    cam = model.camera("right_wrist_cam").id
    W, H = model.cam_resolution[cam]
    f = (H / 2) / math.tan(math.radians(model.cam_fovy[cam]) / 2)
    geom = np.zeros(1, dtype=np.int32)
    ok = total = 0
    for flex in flexes_deg:
        for dev in devs_deg:
            d = hand_pose_data(model, flex, dev, closed)
            cam_R = d.cam_xmat[cam].reshape(3, 3)
            good = True
            for t in tips:
                vec = d.site_xpos[t] - d.cam_xpos[cam]
                dist = np.linalg.norm(vec)
                rel = cam_R.T @ vec
                depth = -rel[2]
                u, v = W / 2 + f * rel[0] / depth, H / 2 - f * rel[1] / depth
                good &= depth > 0.02 and margin * W < u < (1 - margin) * W and margin * H < v < (1 - margin) * H
                hit = mujoco.mj_ray(model, d, d.cam_xpos[cam], vec / dist, RAY_MASK, 1, -1, geom)
                if 0 <= hit < dist and model.body(model.geom_bodyid[geom[0]]).name in OCCLUDERS:
                    good = False
            ok += good
            total += 1
    return ok / total


MID_FLEX, MID_DEV = [-30, -15, 0, 15, 30, 40], [-20, -10, 0, 10, 15]


def search(model, candidates, top=10):
    rows = []
    for pos, tilt, yaw in candidates:
        set_cam(model, pos, tilt, yaw)
        if clearance_hand_frame(model) < -0.0005 or not housing_clear_of_hand(model):
            continue
        gs = grasp_space_visibility(model)
        rows.append((gs, pos, tilt, yaw))
    rows.sort(key=lambda r: -r[0])
    out = []
    for gs, pos, tilt, yaw in rows[: top * 4]:
        set_cam(model, pos, tilt, yaw)
        o, c = tip_coverage(model, MID_FLEX, MID_DEV), tip_coverage(model, MID_FLEX, MID_DEV, closed=1.2)
        out.append((round(gs, 3), round((o + c) / 2, 3), pos, tilt, yaw, round(clearance_hand_frame(model) * 1000, 1)))
    out.sort(key=lambda r: (-r[0], -r[1]))
    return out[:top]


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(br.ROBOT_XML))
    xs = (-0.07, -0.065, -0.055, 0.0, 0.055, 0.065, 0.07)
    ys = (-0.05, -0.045, -0.04, -0.035, -0.03, -0.02, 0.0, 0.04)
    zs = (-0.12, -0.09, -0.07, -0.05)
    cands = [
        ((x, y, z), t, w)
        for x, y, z, t, w in itertools.product(xs, ys, zs, (0, 5, 10, 15, 20, 30), (-25, 0, 25))
        if (w >= 0 or x >= 0) and (w <= 0 or x <= 0)  # yaw turns toward the middle
    ]
    print(len(cands), "candidates")
    for row in search(model, cands):
        print("grasp-space {:.3f}  tips {:.3f}  pos {} tilt {} yaw {}  clearance {:+.1f} mm".format(*row))


if __name__ == "__main__":
    main()
