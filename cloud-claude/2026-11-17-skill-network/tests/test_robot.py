"""Tests for the robot assembly (model/build_robot.py): names, keyframes, holding, step response, collisions,
cameras, workspace, MuJoCo Warp. They run unchanged with the stand-in hand and with the real V2.1 hand.

    uv run pytest tests/test_robot.py                       # fast ones (< 60 s)
    uv run pytest tests/test_robot.py -m slow               # workspace sweep
    uv run --group warp pytest tests/test_robot.py -m warp  # MuJoCo Warp put_model on CPU
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

STUDY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STUDY / "model"))
import build_robot as br

ROBOT_XML = br.ROBOT_XML
SCENE_XML = br.SCENE_XML
ARM = br.arm_joint_names()
HAND = br.hand_joint_names()
KEYS = ("rest", "ready", "table")


@pytest.fixture(scope="module")
def robot() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(ROBOT_XML))


@pytest.fixture(scope="module")
def scene() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(SCENE_XML))


def _reset(model, key):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(key).id)
    mujoco.mj_forward(model, data)
    return data


def _robot_geom_ids(model) -> np.ndarray:
    """Geoms that belong to the robot (arm, hand, pedestal): everything except floor, table and the test objects."""
    other = {"floor", "table"} | {n for n in br.OBJECTS}
    ids = []
    for g in range(model.ngeom):
        body = model.body(model.geom_bodyid[g]).name
        if model.geom(g).name in other or body in other:
            continue
        ids.append(g)
    return np.array(ids)


# ------------------------------------------------------------------------------------------------ structure
def test_actuator_names_and_order(robot, scene):
    expected = [*ARM, *HAND]
    assert len(expected) == 7 + 16 == 23
    for model in (robot, scene):
        assert model.nu == 23
        assert [model.actuator(i).name for i in range(model.nu)] == expected
        for i in range(model.nu):  # every actuator drives the joint of the same name
            assert model.joint(model.actuator_trnid[i, 0]).name == expected[i]
            assert model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT


def test_joint_ranges_and_openarm_limits(robot):
    from openarm_mujoco.v2 import asset_path

    oa = mujoco.MjModel.from_xml_path(asset_path("openarm_bimanual.xml"))
    for ours, theirs in (("shoulder_pitch", 1), ("shoulder_roll", 2), ("shoulder_yaw", 3), ("elbow", 4),
                         ("forearm_twist", 5)):  # fmt: skip
        np.testing.assert_allclose(robot.joint(f"right_{ours}").range, oa.joint(f"openarm_right_joint{theirs}").range)
    for name in ARM[:5]:
        assert robot.jnt_limited[robot.joint(name).id]
    # the contract's wrist signs: + = toward the palm (flex), + = toward the thumb (dev)
    assert robot.joint("right_wrist_flex").range[1] > 0.5 and robot.joint("right_wrist_dev").range[0] < -0.3


def test_options_and_gravcomp(robot, scene):
    for model in (robot, scene):
        assert model.opt.timestep == pytest.approx(0.002)
        assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
        assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        assert model.opt.impratio >= 5
    arm_bodies = [b for b in range(robot.nbody) if robot.body(b).name.startswith(("openarm_right_link", "right_"))]
    assert arm_bodies and all(robot.body_gravcomp[b] == 1.0 for b in arm_bodies)
    for name in br.OBJECTS:  # objects feel gravity
        assert scene.body_gravcomp[scene.body(name).id] == 0.0


def test_contract_sites_and_cameras(robot):
    for site in ("forearm_flange", "wrist_centre", "palm_centre", "index_tip", "middle_tip", "ring_tip",
                 "little_tip", "thumb_tip", "wrist_cam_mount"):  # fmt: skip
        assert robot.site(f"right_{site}").id >= 0
    assert robot.camera("right_wrist_cam").id >= 0 and robot.camera("head_cam").id >= 0
    assert robot.body("head_pole").id >= 0


def test_scene_objects(scene):
    for name, (_, _, mass, _, _) in br.OBJECTS.items():
        b = scene.body(name)
        assert scene.body_mass[b.id] == pytest.approx(mass, rel=1e-6)
        assert scene.body_jntnum[b.id] == 1 and scene.jnt_type[scene.body_jntadr[b.id]] == mujoco.mjtJoint.mjJNT_FREE
    assert scene.geom("table").id >= 0 and scene.geom("floor").id >= 0
    assert scene.nlight >= 2


def test_mounting_orientation(robot):
    """Arm hanging (all zero): fingers point down, thumb side forward (+x), palm faces the body's middle (+y).
    Elbow 90 deg: fingers forward, thumb up. Positive twist = pronation: the palm turns toward down."""
    palm = robot.body("right_palm").id
    d = mujoco.MjData(robot)
    mujoco.mj_forward(robot, d)
    R = d.xmat[palm].reshape(3, 3)  # hand axes in the world: columns x (thumb), y (back), z (fingers)
    np.testing.assert_allclose(R[:, 2], [0, 0, -1], atol=1e-9)
    np.testing.assert_allclose(R[:, 0], [1, 0, 0], atol=1e-9)
    np.testing.assert_allclose(-R[:, 1], [0, 1, 0], atol=1e-9)
    d = _reset(robot, "ready")
    R = d.xmat[palm].reshape(3, 3)
    np.testing.assert_allclose(R[:, 2], [1, 0, 0], atol=1e-4)  # fingers forward
    np.testing.assert_allclose(R[:, 0], [0, 0, 1], atol=1e-4)  # thumb up
    palm_normal = lambda data: -data.xmat[palm].reshape(3, 3)[:, 1]
    before = palm_normal(d)[2]
    d.qpos[robot.jnt_qposadr[robot.joint("right_forearm_twist").id]] = 0.5
    mujoco.mj_forward(robot, d)
    assert palm_normal(d)[2] < before - 0.4  # pronation turns the palm down
    # the forearm's long axis lies on J5's axis: twisting does not move the wrist centre
    d0 = _reset(robot, "ready")
    wc0 = d0.site_xpos[robot.site("right_wrist_centre").id].copy()
    flange = d0.site_xpos[robot.site("right_forearm_flange").id].copy()
    d0.qpos[robot.jnt_qposadr[robot.joint("right_forearm_twist").id]] = 0.8
    mujoco.mj_forward(robot, d0)
    np.testing.assert_allclose(d0.site_xpos[robot.site("right_wrist_centre").id], wc0, atol=1e-9)
    np.testing.assert_allclose(d0.site_xpos[robot.site("right_forearm_flange").id], flange, atol=1e-9)


# ------------------------------------------------------------------------------------------------ keyframes
@pytest.mark.parametrize("key", KEYS)
def test_keyframes_exist(robot, scene, key):
    for model in (robot, scene):
        k = model.key(key)
        assert k.id >= 0
        assert len(model.key_qpos[k.id]) == model.nq and len(model.key_ctrl[k.id]) == model.nu
        for j in ARM:  # inside the joint limits
            lo, hi = model.joint(j).range
            assert lo - 1e-9 <= model.key_qpos[k.id][model.joint(j).qposadr[0]] <= hi + 1e-9
        np.testing.assert_allclose(model.key_ctrl[k.id][7:], 0.0)  # fingers open


def test_table_keyframe_palm_down_over_table(robot):
    d = _reset(robot, "table")
    pc = d.site_xpos[robot.site("right_palm_centre").id]
    normal = d.site_xmat[robot.site("right_palm_centre").id].reshape(3, 3)[:, 2]  # site z = out of the palm
    assert pc[2] - br.TABLE_TOP == pytest.approx(br.PALM_HEIGHT, abs=0.01)
    np.testing.assert_allclose(pc[:2], br.TABLE_POSE_XY, atol=0.01)
    assert normal[2] < -0.99  # facing down
    cx, cy = br.TABLE_CENTRE
    assert abs(pc[0] - cx) < br.TABLE_HALF[0] and abs(pc[1] - cy) < br.TABLE_HALF[1]  # over the table top
    tips = np.array([d.site_xpos[robot.site(f"right_{f}_tip").id] for f in ("index", "middle", "ring", "little")])
    assert tips[:, 2].min() > br.TABLE_TOP + 0.03  # fingers open, above the table


@pytest.mark.parametrize("key", KEYS)
def test_keyframe_clear_of_floor_table_and_self(robot, key):
    """No contact of any kind in the three poses (self-collision between arm and hand included), and the robot is
    clear of the floor by >= 3 cm."""
    d = _reset(robot, key)
    names = [(robot.geom(c.geom1).name, robot.geom(c.geom2).name, c.dist) for c in d.contact[: d.ncon]]
    assert d.ncon == 0, names
    assert br.lowest_point_z(robot, d.qpos.copy()) > 0.03


@pytest.mark.parametrize("key", KEYS)
def test_hold_keyframe(scene, key):
    """3 s simulated holding the keyframe's targets: no NaN, every joint within 1 deg (objects excluded)."""
    d = _reset(scene, key)
    q0 = d.qpos.copy()
    for _ in range(int(3.0 / scene.opt.timestep)):
        mujoco.mj_step(scene, d)
    assert np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))
    drift = 0.0
    for name in (*ARM, *HAND):
        a = scene.joint(name).qposadr[0]
        drift = max(drift, abs(d.qpos[a] - q0[a]))
    assert math.degrees(drift) < 1.0, f"{key}: drift {math.degrees(drift):.3f} deg"
    for name in br.OBJECTS:  # the objects stay on the table
        assert d.qpos[scene.joint(f"{name}_free").qposadr[0] + 2] > br.TABLE_TOP


# ------------------------------------------------------------------------------------------------ dynamics
def _step_response(model, key, joint_index, step):
    """Overshoot (fraction of the step) and 2 % settling time of one arm joint, contacts disabled so only the
    servo and the arm's inertia are measured."""
    m = model
    d = _reset(m, key)
    jid = m.joint(ARM[joint_index]).id
    adr = m.jnt_qposadr[jid]
    q0 = d.qpos[adr]
    d.ctrl[joint_index] = q0 + step
    qs = []
    for _ in range(int(1.5 / m.opt.timestep)):
        mujoco.mj_step(m, d)
        qs.append(d.qpos[adr])
    qs = (np.array(qs) - q0) * math.copysign(1.0, step)
    overshoot = (qs.max() - abs(step)) / abs(step)
    err = np.abs(qs - abs(step))
    idx = np.nonzero(err > 0.02 * abs(step))[0]
    settle = (idx[-1] + 1) * m.opt.timestep if len(idx) else 0.0
    return overshoot, settle, err[-1] / abs(step)


@pytest.mark.parametrize("key", ["ready", "table"])
def test_arm_step_response_not_underdamped(robot, key):
    model = robot  # a copy with contacts off: the response of the servos themselves
    model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    checked = 0
    for i, name in enumerate(ARM):
        lo, hi = model.joint(name).range
        q0 = model.key_qpos[model.key(key).id][model.joint(name).qposadr[0]]
        for sign in (+1, -1):
            step = 0.15 * sign
            if not (lo + 0.02 < q0 + step < hi - 0.02):
                continue
            overshoot, settle, final = _step_response(model, key, i, step)
            assert overshoot < 0.05, f"{name} {step:+.2f} rad: overshoot {100 * overshoot:.1f} %"
            assert final < 0.02, f"{name} {step:+.2f} rad did not settle ({100 * final:.1f} % left)"
            assert settle < 0.8, f"{name} settles in {settle:.2f} s"
            checked += 1
    assert checked >= 10


def test_stock_gains_would_fail_our_check():
    """The damping tuning matters: OpenArm's stock kv leaves the shoulder visibly under-damped, so the test above
    is meaningful (the measured stock overshoot is > 5 %)."""
    from openarm_mujoco.v2 import asset_path

    model = mujoco.MjModel.from_xml_path(asset_path("openarm_bimanual.xml"))
    model.opt.timestep = 0.002
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    d = mujoco.MjData(model)
    d.qpos[model.joint("openarm_right_joint4").qposadr[0]] = 1.2
    d.ctrl[model.actuator("right_joint4_ctrl").id] = 1.2
    for _ in range(500):
        mujoco.mj_step(model, d)
    d.ctrl[model.actuator("right_joint4_ctrl").id] = 1.5
    peak = 0.0
    for _ in range(750):
        mujoco.mj_step(model, d)
        peak = max(peak, d.qpos[model.joint("openarm_right_joint4").qposadr[0]])
    assert (peak - 1.5) / 0.3 > 0.05


# ------------------------------------------------------------------------------------------------ cameras
def project(model, data, cam: str, point) -> tuple[float, float, float]:
    """(u, v, depth) pixel coordinates of a world point in a camera (square pixels, MuJoCo's fovy)."""
    c = model.camera(cam).id
    W, H = model.cam_resolution[c]
    fovy = math.radians(model.cam_fovy[c])
    f = (H / 2) / math.tan(fovy / 2)
    rel = data.cam_xmat[c].reshape(3, 3).T @ (np.asarray(point) - data.cam_xpos[c])
    depth = -rel[2]
    return W / 2 + f * rel[0] / depth, H / 2 - f * rel[1] / depth, depth


def wrist_cam_coverage(model, flexes_deg, devs_deg, closed: float = 0.0, margin: float = 0.04) -> float:
    """Fraction of wrist poses where all five fingertip sites are in the wrist camera image AND not hidden behind
    the forearm, wrist ring or palm (a ray from the camera to each tip must not hit them first)."""
    d = mujoco.MjData(model)
    tips = [model.site(f"right_{f}_tip").id for f in ("index", "middle", "ring", "little", "thumb")]
    cam = model.camera("right_wrist_cam").id
    W, H = model.cam_resolution[cam]
    occluders = {"right_forearm", "right_wrist_ring", "right_palm"}
    group_mask = np.array([1, 1, 0, 1, 1, 1], dtype=np.uint8)  # skip group 2 (visual-only camera bracket)
    ok = total = 0
    for flex in flexes_deg:
        for dev in devs_deg:
            d.qpos[:] = model.qpos0
            d.qpos[model.joint("right_wrist_flex").qposadr[0]] = math.radians(flex)
            d.qpos[model.joint("right_wrist_dev").qposadr[0]] = math.radians(dev)
            if closed:
                for f in ("index", "middle", "ring", "little"):
                    d.qpos[model.joint(f"right_{f}_mcp_flex").qposadr[0]] = closed
                    d.qpos[model.joint(f"right_{f}_pip").qposadr[0]] = closed
            mujoco.mj_kinematics(model, d)
            mujoco.mj_camlight(model, d)
            good = True
            for t in tips:
                u, v, depth = project(model, d, "right_wrist_cam", d.site_xpos[t])
                good &= depth > 0.02 and margin * W < u < (1 - margin) * W and margin * H < v < (1 - margin) * H
                vec = d.site_xpos[t] - d.cam_xpos[cam]
                dist = np.linalg.norm(vec)
                geom = np.zeros(1, dtype=np.int32)
                hit = mujoco.mj_ray(model, d, d.cam_xpos[cam], vec / dist, group_mask, 1, -1, geom)
                if 0 <= hit < dist and model.body(model.geom_bodyid[geom[0]]).name in occluders:
                    good = False
            ok += good
            total += 1
    return ok / total


def test_wrist_camera_specs(robot):
    c = robot.camera("right_wrist_cam")
    W, H = robot.cam_resolution[c.id]
    assert (W, H) == (640, 480)
    hfov = 2 * math.degrees(math.atan(math.tan(math.radians(robot.cam_fovy[c.id]) / 2) * W / H))
    assert 90 <= hfov <= 110.5
    # rigidly on the forearm (not on the palm or fingers), at the palm side, along the fingers
    assert robot.body(robot.cam_bodyid[c.id]).name == "right_forearm"
    d = mujoco.MjData(robot)
    mujoco.mj_forward(robot, d)
    look = -d.cam_xmat[c.id].reshape(3, 3)[:, 2]
    fingers = d.xmat[robot.body("right_palm").id].reshape(3, 3)[:, 2]
    palm_side = -d.xmat[robot.body("right_palm").id].reshape(3, 3)[:, 1]
    assert look @ fingers > math.cos(math.radians(15))  # along the fingers ...
    assert look @ palm_side > 0.02  # ... tilted toward the palm side
    mount = robot.site("right_wrist_cam_mount")
    np.testing.assert_allclose(d.site_xpos[mount.id], d.cam_xpos[c.id], atol=1e-6)


def test_wrist_camera_sees_the_fingertips(robot):
    # grasping range of the wrist (flex -30..40 deg, dev -20..15 deg), fingers open and closed: all five tips visible
    mid_f, mid_d = [-30, -15, 0, 15, 30, 40], [-20, -10, 0, 10, 15]
    assert wrist_cam_coverage(robot, mid_f, mid_d) == 1.0
    assert wrist_cam_coverage(robot, mid_f, mid_d, closed=1.2) == 1.0
    # the whole wrist range: extreme flexion/extension turns the hand away from a camera on the palm side
    lo_f, hi_f = np.degrees(robot.joint("right_wrist_flex").range)
    lo_d, hi_d = np.degrees(robot.joint("right_wrist_dev").range)
    fl, dv = np.linspace(lo_f, hi_f, 10), np.linspace(lo_d, hi_d, 6)
    full, full_closed = wrist_cam_coverage(robot, fl, dv), wrist_cam_coverage(robot, fl, dv, closed=1.2)
    print(f"wrist camera: all fingertips visible over {100 * full:.0f} % (open) / {100 * full_closed:.0f} % (closed) "
          "of the full wrist range")  # fmt: skip
    assert full >= 0.6 and full_closed >= 0.7


def test_head_camera_sees_the_table(robot):
    c = robot.camera("head_cam")
    W, H = robot.cam_resolution[c.id]
    assert (W, H) == (1280, 720)
    hfov = 2 * math.degrees(math.atan(math.tan(math.radians(robot.cam_fovy[c.id]) / 2) * W / H))
    assert hfov == pytest.approx(69.0, abs=0.5) and robot.cam_fovy[c.id] == pytest.approx(42.3, abs=0.3)
    d = _reset(robot, "ready")
    cam_pos = d.cam_xpos[c.id]
    assert cam_pos[0] < -0.1 and cam_pos[2] > 1.0  # behind the shoulders (shoulder x = 0), above them (0.698)
    cx, cy = br.TABLE_CENTRE
    for point in ([cx, cy, br.TABLE_TOP], [*br.TABLE_POSE_XY, br.TABLE_TOP],
                  [0.28, -0.43, br.TABLE_TOP], [0.52, -0.30, br.TABLE_TOP]):  # fmt: skip
        u, v, depth = project(robot, d, "head_cam", point)
        assert 0.05 * W < u < 0.95 * W and 0.05 * H < v < 0.95 * H and depth > 0.3, (point, u, v)
    # the right workspace (the 40 x 30 cm region) is fully in view
    for x, y in [(0.2, -0.5), (0.5, -0.5), (0.2, -0.1), (0.5, -0.1)]:
        u, v, _ = project(robot, d, "head_cam", [x, y, br.TABLE_TOP])
        assert 0 < u < W and 0 < v < H, (x, y, u, v)


_RENDER_SCRIPT = """
import json, sys, numpy as np, mujoco
m = mujoco.MjModel.from_xml_path(sys.argv[1])
d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, m.key("table").id)
mujoco.mj_forward(m, d)
out = {}
for name, size in (("right_wrist_cam", (480, 640)), ("head_cam", (720, 1280))):
    r = mujoco.Renderer(m, *size)
    r.update_scene(d, camera=name)
    rgb = r.render()
    out[name] = [list(rgb.shape), float(rgb.std())]
    if name == "head_cam":
        r.enable_depth_rendering()
        r.update_scene(d, camera=name)
        depth = r.render()
        out["depth"] = [bool(np.isfinite(depth).all()), float(np.median(depth))]
    r.close()
print("RESULT" + json.dumps(out))
"""


def test_cameras_render():
    """Both cameras render offscreen (each GL backend is tried in its own process: a missing EGL can abort)."""
    errors = []
    for backend in ("osmesa", "egl"):
        env = {**os.environ, "MUJOCO_GL": backend, "PYOPENGL_PLATFORM": backend}
        run = subprocess.run([sys.executable, "-c", _RENDER_SCRIPT, str(SCENE_XML)], capture_output=True, text=True,
                             env=env, timeout=120, check=False)  # fmt: skip
        if run.returncode == 0 and "RESULT" in run.stdout:
            import json

            out = json.loads(run.stdout.split("RESULT")[1])
            assert out["right_wrist_cam"][0] == [480, 640, 3] and out["right_wrist_cam"][1] > 5
            assert out["head_cam"][0] == [720, 1280, 3] and out["head_cam"][1] > 5
            assert out["depth"][0] and 0.3 < out["depth"][1] < 2.5
            return
        errors.append(backend)
    pytest.skip("no OpenGL backend (EGL/OSMesa) here: camera geometry is covered by the projection tests")


# ------------------------------------------------------------------------------------------------ workspace
def test_table_workspace_quick(robot):
    """Palm down over the centre of the 40 x 30 cm pick region (and its corners) is reachable by IK."""
    import workspace as ws  # noqa: F401  (the sweep lives there)

    xs, ys = (0.2, 0.4), (-0.45, -0.15)
    d = mujoco.MjData(robot)
    jid = [robot.joint(n).id for n in ARM]
    qadr = robot.jnt_qposadr[jid]
    palm_site = robot.site("right_palm_centre").id
    palm_body = robot.body("right_palm").id
    z = br.TABLE_TOP + br.PALM_HEIGHT
    reached = 0
    pts = [(x, y) for x in (*xs, 0.30) for y in (*ys, -0.30)]
    for x, y in pts:
        best = None
        for yaw in (0.0, -15.0, 15.0, -30.0):
            rot = br.palm_target_rotation(math.radians(yaw))
            q, _ = br.solve_arm_pose(robot, np.array([x, y, z]), rot, seeds=6, iters=120)
            if q is None:
                continue
            d.qpos[:] = robot.qpos0
            d.qpos[qadr] = q
            mujoco.mj_forward(robot, d)
            pos_err = np.linalg.norm(d.site_xpos[palm_site] - [x, y, z])
            tilt = math.degrees(math.acos(np.clip(d.xmat[palm_body].reshape(3, 3)[2, 1], -1, 1)))  # palm normal vs down
            if pos_err < 0.005 and tilt < 8.0 and d.ncon == 0:
                best = yaw
                break
        reached += best is not None
    assert reached >= len(pts) - 1, f"only {reached}/{len(pts)} sample points reachable palm-down"


@pytest.mark.slow
def test_workspace_at_least_40_by_30(robot):
    """Palm down (within 15 deg), fingers within +-30 deg of forward, palm centre 10, 15 or 20 cm above the table:
    a fully reachable 40 x 30 cm rectangle (either orientation) lies on the table. ~3 min."""
    import workspace as ws

    res = ws.analyse_heights(robot, step=0.05)
    rects = ws.largest_rectangles(res["union"], res["xs"], res["ys"])
    assert rects, "no fully reachable 40 x 30 cm rectangle"
    _, x0, x1, y0, y1 = rects[0]
    cx, cy = br.TABLE_CENTRE
    hx, hy = br.TABLE_HALF
    assert cx - hx + 0.03 < x0 and x1 < cx + hx - 0.03 and cy - hy < y0 and y1 < cy + hy  # on the table, with margin


# ------------------------------------------------------------------------------------------------ build
def test_build_check_passes():
    out = subprocess.run([sys.executable, str(STUDY / "model" / "build_robot.py"), "--check"], capture_output=True,
                         text=True, cwd=STUDY, check=False)  # fmt: skip
    assert out.returncode == 0, out.stdout + out.stderr


def test_build_is_deterministic():
    a, _ = br.build_all("auto")
    b, _ = br.build_all("auto")
    assert {p: br._normalise(t) for p, t in a.items()} == {p: br._normalise(t) for p, t in b.items()}


@pytest.mark.warp
def test_warp_put_model_cpu(scene):
    wp = pytest.importorskip("warp")
    mjw = pytest.importorskip("mujoco_warp")
    with wp.ScopedDevice("cpu"):
        mw = mjw.put_model(scene)
        data = mujoco.MjData(scene)
        mujoco.mj_resetDataKeyframe(scene, data, scene.key("table").id)
        dw = mjw.put_data(scene, data, nworld=2, nconmax=200, njmax=800)
        for _ in range(5):
            mjw.step(mw, dw)
        q = dw.qpos.numpy()
        assert q.shape == (2, scene.nq) and np.all(np.isfinite(q))
        np.testing.assert_allclose(q[0], q[1])  # identical worlds stay identical
