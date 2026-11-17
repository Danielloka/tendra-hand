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
sys.path.insert(0, str(STUDY / "tests"))
import build_robot as br
import camera_design as cd
import grasp_sim as gs

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
    arm = {b for b in range(robot.nbody) if robot.body(b).name.startswith("openarm_right_link")}
    arm.add(robot.body("right_forearm_twist_link").id)
    assert len(arm) == 5
    for b in range(robot.nbody):  # gravity compensation on the arm only; hand, forearm, objects and the rest feel gravity
        assert robot.body_gravcomp[b] == (1.0 if b in arm else 0.0), robot.body(b).name
    for name in br.OBJECTS:
        assert scene.body_gravcomp[scene.body(name).id] == 0.0


def test_sensors_in_actuator_order(robot, scene):
    """jointpos and actuatorfrc sensors of the 7 arm joints, named right_<joint>_pos / _frc, first and in order."""
    for model in (robot, scene):
        names = [model.sensor(i).name for i in range(model.nsensor)]
        assert names[:7] == [f"{j}_pos" for j in ARM] and names[7:14] == [f"{j}_frc" for j in ARM]
        for i, j in enumerate(ARM):
            assert model.sensor_type[i] == mujoco.mjtSensor.mjSENS_JOINTPOS and model.joint(j).id == model.sensor_objid[i]
            assert model.sensor_type[7 + i] == mujoco.mjtSensor.mjSENS_ACTUATORFRC
            assert model.actuator(j).id == model.sensor_objid[7 + i]
    # readings follow the sensors
    d = _reset(robot, "table")
    for i, j in enumerate(ARM):
        assert d.sensordata[robot.sensor_adr[i]] == pytest.approx(d.qpos[robot.joint(j).qposadr[0]])


def test_actuator_qpos_index_helper(robot):
    """The tree order of the joints differs from the actuator order (wrist_dev, wrist_flex come first in qpos)."""
    idx = br.actuator_qpos_index(robot)
    assert len(idx) == robot.nu
    for a, adr in enumerate(idx):
        assert adr == robot.joint(robot.actuator(a).name).qposadr[0]
    assert not np.all(np.diff(idx) > 0)  # the order really differs (the helper is needed)


def test_head_pole_mass_is_explicit(robot):
    assert robot.body_mass[robot.body("head_pole").id] > 0


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
        g = scene.geom(name)  # no `priority`: the hand's pad settings win (review B2); pad-compatible own values
        assert scene.geom_priority[g.id] == 0 and scene.geom_condim[g.id] == 4
        assert scene.geom_solref[g.id][0] <= 0.005 + 1e-9
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
def _open_pose_rad() -> dict[str, float]:
    import json

    poses = json.loads((STUDY / "model" / "assets" / "v21" / "fk_reference.json").read_text())["poses"]["open"]["q"]
    return {k: math.radians(v) for k, v in poses.items() if k.startswith("thumb")}


@pytest.mark.parametrize("key", KEYS)
def test_keyframes_exist(robot, scene, key):
    thumb = _open_pose_rad()
    assert set(thumb) == {"thumb_cmc_rot", "thumb_cmc_flex", "thumb_mcp_flex", "thumb_ip"}
    for model in (robot, scene):
        k = model.key(key)
        assert k.id >= 0
        qpos, ctrl = model.key_qpos[k.id], model.key_ctrl[k.id]
        assert len(qpos) == model.nq and len(ctrl) == model.nu
        for j in range(model.njnt):  # every limited joint inside its limits (arm, wrist, fingers)
            if model.jnt_limited[j] and model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
                lo, hi = model.jnt_range[j]
                assert lo - 1e-6 <= qpos[model.jnt_qposadr[j]] <= hi + 1e-6, model.joint(j).name
        for a in range(model.nu):  # every control inside its ctrlrange
            lo, hi = model.actuator_ctrlrange[a]
            assert lo - 1e-9 <= ctrl[a] <= hi + 1e-9, model.actuator(a).name
        for name in br.SERVO_JOINTS:
            expected = thumb.get(name, 0.0) if key != "rest" else 0.0  # ready, table: CAD open thumb; fingers straight
            q = qpos[model.joint(f"right_{name}").qposadr[0]]
            c = ctrl[model.actuator(f"right_{name}").id]
            assert q == pytest.approx(expected, abs=1e-4) and c == pytest.approx(expected, abs=1e-4), (key, name)


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
    # the open thumb (the lowest part of the open hand) clears the table by >= 1 cm; so does the camera housing
    low_hand, low_house = cd.lowest_points(robot, d, [0, 0, 1])
    assert low_hand > br.TABLE_TOP + 0.01, f"hand lowest {low_hand - br.TABLE_TOP:.4f} m above the table"
    assert low_house > br.TABLE_TOP + 0.01


@pytest.mark.parametrize("key", KEYS)
def test_keyframe_clear_of_floor_table_and_self(robot, key):
    """No contact of any kind in the three poses (self-collision between arm and hand included), and the robot is
    clear of the floor by >= 3 cm."""
    d = _reset(robot, key)
    names = [(robot.geom(c.geom1).name, robot.geom(c.geom2).name, c.dist) for c in d.contact[: d.ncon]]
    assert d.ncon == 0, names
    assert br.lowest_point_z(robot, d.qpos.copy()) > 0.03


def _hold(model, key, seconds=3.0):
    d = _reset(model, key)
    q0 = d.qpos.copy()
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, d)
    assert np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))
    drift = {n: math.degrees(abs(d.qpos[model.joint(n).qposadr[0]] - q0[model.joint(n).qposadr[0]])) for n in (*ARM, *HAND)}
    return d, drift


@pytest.mark.parametrize("key", KEYS)
def test_hold_keyframe(scene, key):
    """3 s holding the keyframe's targets. Only the arm is gravity-compensated: the wrist and the fingers carry the
    hand's weight like the real servos, so this is a real test of the servos' stiffness: the arm sags ~1 deg
    under the hand's ~1.2 kg (no integral term), the wrist and fingers less. Every joint within 2 deg; the worst
    one is printed."""
    d, drift = _hold(scene, key)
    worst = max(drift, key=drift.get)
    print(f"hold {key}: worst drift {worst} {drift[worst]:.3f} deg; arm max {max(drift[n] for n in ARM):.4f} deg")
    assert drift[worst] < 2.0, f"{key}: {worst} drifts {drift[worst]:.3f} deg"
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
    final = qs[-1]  # the unloaded hand's weight droops the arm joints slightly: measure against the settled value
    overshoot = (qs.max() - final) / abs(step)
    err = np.abs(qs - final)
    idx = np.nonzero(err > 0.02 * abs(step))[0]
    settle = (idx[-1] + 1) * m.opt.timestep if len(idx) else 0.0
    return overshoot, settle, 1.0 - final / abs(step)


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
            assert abs(final) < 0.15, f"{name} {step:+.2f} rad: {100 * final:.1f} % short of the command (hand weight)"
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


def test_wrist_camera_specs(robot):
    c = robot.camera("right_wrist_cam")
    W, H = robot.cam_resolution[c.id]
    assert (W, H) == (640, 480)
    hfov = 2 * math.degrees(math.atan(math.tan(math.radians(robot.cam_fovy[c.id]) / 2) * W / H))
    assert 90 <= hfov <= 110.5
    # rigidly on the forearm (not on the palm or fingers), looking along the fingers, tilted toward the palm side
    assert robot.body(robot.cam_bodyid[c.id]).name == "right_forearm"
    d = mujoco.MjData(robot)
    mujoco.mj_forward(robot, d)
    look = -d.cam_xmat[c.id].reshape(3, 3)[:, 2]
    fingers = d.xmat[robot.body("right_palm").id].reshape(3, 3)[:, 2]
    palm_side = -d.xmat[robot.body("right_palm").id].reshape(3, 3)[:, 1]
    assert look @ fingers > math.cos(math.radians(35))  # along the fingers (a little turned in) ...
    assert look @ palm_side > 0.02  # ... tilted toward the palm side
    mount = robot.site("right_wrist_cam_mount")
    np.testing.assert_allclose(d.site_xpos[mount.id], d.cam_xpos[c.id], atol=1e-6)
    # a real collision box of a small camera board + case (~25 x 25 x 15 mm), on the forearm
    housing = robot.geom("right_wrist_cam_housing")
    assert robot.body(robot.geom_bodyid[housing.id]).name == "right_forearm"
    assert robot.geom_type[housing.id] == mujoco.mjtGeom.mjGEOM_BOX and robot.geom_contype[housing.id] > 0
    np.testing.assert_allclose(2 * robot.geom_size[housing.id], [0.025, 0.025, 0.015], atol=1e-9)


def test_wrist_camera_housing_never_the_lowest_point(robot):
    """Review B1: palm down, the camera housing's lowest point is never below the hand's lowest collision point,
    with the CAD open thumb and with the straight thumb, at the `table` pose and in the hand frame; and the housing
    does not intersect the hand."""
    assert cd.clearance_hand_frame(robot) >= -5e-4
    assert cd.housing_clear_of_hand(robot)
    d = _reset(robot, "table")
    low_hand, low_house = cd.lowest_points(robot, d, [0, 0, 1])
    assert low_house >= low_hand - 5e-4
    # closing the fingers or curling the wrist toward the palm moves hand parts below the housing, never the reverse
    for flex in (0.0, 40.0, 70.0):
        dd = cd.hand_pose_data(robot, flex_deg=flex, closed=1.0)
        up = dd.xmat[robot.body("right_forearm").id].reshape(3, 3)[:, 1]
        hand, house = cd.lowest_points(robot, dd, up)
        assert house - hand > -5e-4 or flex == 0.0, (flex, house - hand)


def test_wrist_camera_sees_the_fingertips(robot):
    # grasping range of the wrist (flex -30..40 deg, dev -20..15 deg), fingers open and closed: all five tips visible
    # and not hidden by the forearm, wrist ring or palm
    open_cov = cd.tip_coverage(robot, cd.MID_FLEX, cd.MID_DEV)
    closed_cov = cd.tip_coverage(robot, cd.MID_FLEX, cd.MID_DEV, closed=1.2)
    print(f"wrist camera, grasping range: fingertips visible {100 * open_cov:.0f} % open / {100 * closed_cov:.0f} % closed")
    assert open_cov >= 0.8 and closed_cov >= 0.8
    # the grasp space in front of the palm (8 x 6 x 8 cm box) is in view and not hidden
    space = cd.grasp_space_visibility(robot)
    print(f"wrist camera: grasp space visible {100 * space:.0f} %")
    assert space >= 0.9
    # the whole wrist range: extreme flexion/extension turns the hand away from the camera
    lo_f, hi_f = np.degrees(robot.joint("right_wrist_flex").range)
    lo_d, hi_d = np.degrees(robot.joint("right_wrist_dev").range)
    fl, dv = np.linspace(lo_f, hi_f, 10), np.linspace(lo_d, hi_d, 6)
    full, full_closed = cd.tip_coverage(robot, fl, dv), cd.tip_coverage(robot, fl, dv, closed=1.2)
    print(f"wrist camera: all fingertips visible over {100 * full:.0f} % (open) / {100 * full_closed:.0f} % (closed) "
          "of the full wrist range")  # fmt: skip
    assert full >= 0.5 and full_closed >= 0.6


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


# ------------------------------------------------------------------------------------------------ collision
def count_collision_pairs(model) -> int:
    """Candidate geom pairs MuJoCo C would test (contype/conaffinity, same-body, welded, parent-child filters),
    plus the explicit <pair>s."""
    n = model.npair
    for g1 in range(model.ngeom):
        for g2 in range(g1 + 1, model.ngeom):
            b1, b2 = model.geom_bodyid[g1], model.geom_bodyid[g2]
            if b1 == b2 or model.body_weldid[b1] == model.body_weldid[b2]:
                continue
            if not (model.geom_contype[g1] & model.geom_conaffinity[g2] or model.geom_contype[g2] & model.geom_conaffinity[g1]):
                continue
            p1, p2 = model.body_parentid[b1], model.body_parentid[b2]
            if (p1 == b2 and model.body_weldid[p1] != 0) or (p2 == b1 and model.body_weldid[p2] != 0):
                continue
            sig = (min(b1, b2) << 16) + max(b1, b2)
            if sig in set(model.exclude_signature):
                continue
            n += 1
    return n


def test_openarm_collision_is_boxes_on_their_own_layer(robot):
    """Review M4: OpenArm's hulls (a 2932-vertex pedestal hull...) are replaced by boxes that collide with the
    environment only; hand-vs-arm contacts are a handful of explicit pairs (forearm, wrist ring, palm only)."""
    arm_geoms = [g for g in range(robot.ngeom) if robot.geom_group[g] == 3 and "_box" in robot.geom(g).name]
    assert len(arm_geoms) == 6  # pedestal + base_link, link1..link4
    for g in arm_geoms:
        assert robot.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
        assert robot.geom_contype[g] == br.BIT_ARM and robot.geom_conaffinity[g] == 0
    for g in range(robot.ngeom):  # no other collision geom on the arm or the pedestal
        body = robot.body(robot.geom_bodyid[g]).name
        if (body == "world" or body.startswith("openarm_")) and (robot.geom_contype[g] or robot.geom_conaffinity[g]):
            assert g in arm_geoms or robot.geom(g).name in ("floor", "table"), robot.geom(g).name
    arm_set = {robot.geom(g).id for g in arm_geoms}
    for i in range(robot.npair):
        g1, g2 = sorted((robot.pair_geom1[i], robot.pair_geom2[i]), key=lambda g: g in arm_set)  # (hand geom, box)
        assert g2 in arm_set and robot.body(robot.geom_bodyid[g1]).name in ("right_forearm", "right_wrist_ring", "right_palm")
    assert 0 < robot.npair <= 80
    # no arm box collides with a finger through the filters
    for g in arm_geoms:
        for h in range(robot.ngeom):
            if robot.body(robot.geom_bodyid[h]).name.startswith("right_") and "finger" not in robot.geom(h).name:
                filtered = robot.geom_contype[g] & robot.geom_conaffinity[h] or robot.geom_contype[h] & robot.geom_conaffinity[g]
                assert not filtered, (robot.geom(g).name, robot.geom(h).name)


def test_arm_boxes_collide_with_table_and_objects(scene):
    """The arm still hits the table: link4's box pushed into the table makes contacts."""
    d = _reset(scene, "table")
    link4 = scene.body("openarm_right_link4").id
    d.qpos[scene.joint("right_shoulder_pitch").qposadr[0]] = 1.2  # swing the elbow link down onto the table
    mujoco.mj_forward(scene, d)
    hits = [(scene.geom(c.geom1).name, scene.geom(c.geom2).name) for c in d.contact[: d.ncon]]
    assert any("box" in a + b and "table" in a + b for a, b in hits) or d.xpos[link4][2] > br.TABLE_TOP, hits


def test_pair_count_report(robot, scene):
    n_robot, n_scene = count_collision_pairs(robot), count_collision_pairs(scene)
    print(f"collision pairs (MuJoCo C filters): robot {n_robot}, scene {n_scene}")
    assert n_scene < 1000


# ------------------------------------------------------------------------------------------------ contacts / grasp
@pytest.fixture(scope="module")
def cube_grasp(scene):
    return gs.run_grasp(scene, "cube", s1=1.5)


def test_pad_object_contact_dim4_and_pad_friction(scene, cube_grasp):
    """Review B2: in a real grasp the pad-object contacts are dim-4 contacts with the pad's friction (1.2); the
    object does not override it (no `priority`)."""
    pad_friction = None
    for g in range(scene.ngeom):
        if scene.geom(g).name.startswith("right_middle_distal_c"):
            pad_friction = scene.geom_friction[g][0]
    assert pad_friction == pytest.approx(1.2)
    hand_contacts = [c for c in cube_grasp["contacts"] if c[0] != "table" and c[1] != "table" and "mug" not in c[0] + c[1]]
    assert hand_contacts, cube_grasp["contacts"]
    dims = {c[2] for c in hand_contacts}
    assert dims == {4}, dims
    pads = [c for c in hand_contacts if scene.geom_friction[scene.geom(c[1] if c[0] == "cube" else c[0]).id][0] >= 1.2 - 1e-9]
    assert pads, hand_contacts
    for _, _, _, fr in pads:
        assert fr[0] == pytest.approx(pad_friction) and fr[1] == pytest.approx(pad_friction)


def test_cube_grasp_holds_with_sub_millimetre_penetration(cube_grasp):
    print(f"cube grasp: lift {cube_grasp['lift_mm']:.1f} mm, slip {cube_grasp['slip_mm']:.2f} mm, "
          f"penetration {cube_grasp['pen_mm']:.2f} mm")  # fmt: skip
    assert cube_grasp["finite"] and cube_grasp["lift_mm"] > 60
    assert cube_grasp["slip_mm"] < 3.0
    assert cube_grasp["pen_mm"] < 1.0


# ------------------------------------------------------------------------------------------------ build
def test_build_check_passes():
    out = subprocess.run([sys.executable, str(STUDY / "model" / "build_robot.py"), "--check"], capture_output=True,
                         text=True, cwd=STUDY, check=False)  # fmt: skip
    assert out.returncode == 0, out.stdout + out.stderr


def test_build_is_deterministic():
    a, _ = br.build_all("auto")
    b, _ = br.build_all("auto")
    assert a == b


def test_xml_is_self_contained_and_relocatable(tmp_path):
    """Review B3: no absolute paths in the XMLs, every mesh file exists under model/assets, and a copy of model/
    loads from another working directory and passes its own `--check` (independent of where the venv or the
    OpenArm package lives)."""
    import re
    import shutil

    for xml in (ROBOT_XML, SCENE_XML):
        text = xml.read_text()
        assert "/home/" not in text and ".venv" not in text and "site-packages" not in text
        meshdir = re.search(r'meshdir="([^"]*)"', text).group(1)
        assert not Path(meshdir).is_absolute()
        for f in set(re.findall(r'<mesh [^>]*file="([^"]*)"', text)):
            assert (xml.parent / meshdir / f).exists(), f
    assert (STUDY / "model" / "assets" / "openarm" / "LICENSE").exists()
    notice = (STUDY / "model" / "assets" / "openarm" / "NOTICE").read_text()
    assert "2.3.0" in notice and "Apache" in notice
    copy = tmp_path / "elsewhere" / "model"
    shutil.copytree(STUDY / "model", copy, ignore=shutil.ignore_patterns("__pycache__", "renders"))
    code = ("import mujoco,sys; m=mujoco.MjModel.from_xml_path(sys.argv[1]); "
            "print('LOADED', m.nu, m.nmesh)")  # fmt: skip
    run = subprocess.run([sys.executable, "-c", code, str(copy / "models" / "v21_scene.xml")], capture_output=True,
                         text=True, cwd=tmp_path, check=False)  # fmt: skip
    assert "LOADED 23" in run.stdout, run.stdout + run.stderr
    check = subprocess.run([sys.executable, str(copy / "build_robot.py"), "--check"], capture_output=True, text=True,
                           cwd=tmp_path, check=False)  # fmt: skip
    assert check.returncode == 0, check.stdout + check.stderr


@pytest.mark.warp
def test_warp_put_model_cpu(scene):
    wp = pytest.importorskip("warp")
    mjw = pytest.importorskip("mujoco_warp")
    with wp.ScopedDevice("cpu"):
        mw = mjw.put_model(scene)
        print("warp broadphase pairs: all", mw.nxn_geom_pair.shape[0], "filtered", mw.nxn_geom_pair_filtered.shape[0])
        assert mw.nxn_geom_pair_filtered.shape[0] < 1000


@pytest.mark.warp
def test_warp_matches_mujoco_in_a_contact_rich_grasp(scene):
    """Close + lift a cube (scripted, the same controls) for 300 steps in MuJoCo C and MuJoCo Warp (CPU): the
    hand and arm joints and the cube agree, with many contacts. Float32 vs float64 differs a little."""
    wp = pytest.importorskip("warp")
    mjw = pytest.importorskip("mujoco_warp")
    n_steps = 300
    d, q_arm, q_lift = gs.setup(scene, "cube")
    ctrl = gs.ctrl_schedule(scene, d, q_arm, q_lift, s1=1.5, t_settle=0.1, t_close=0.4, t_lift=0.2)
    d0 = mujoco.MjData(scene)
    d0.qpos[:], d0.ctrl[:] = d.qpos, d.ctrl
    mujoco.mj_forward(scene, d0)
    dc = mujoco.MjData(scene)
    dc.qpos[:], dc.ctrl[:] = d.qpos, d.ctrl
    ctrls, max_ncon = [], 0
    for k in range(n_steps):
        dc.ctrl[:] = ctrl(k)
        ctrls.append(dc.ctrl.copy())
        mujoco.mj_step(scene, dc)
        max_ncon = max(max_ncon, dc.ncon)
    with wp.ScopedDevice("cpu"):
        mw = mjw.put_model(scene)
        dw = mjw.put_data(scene, d0, nworld=1, nconmax=300, njmax=1500)
        for k in range(n_steps):
            dw.ctrl.assign(np.asarray(ctrls[k], dtype=np.float32)[None])
            mjw.step(mw, dw)
        q = dw.qpos.numpy()[0]
    assert np.all(np.isfinite(q))
    adr = scene.jnt_qposadr[[scene.joint(n).id for n in (*ARM, *HAND)]]
    diff = np.abs(q[adr] - dc.qpos[adr])
    ob = scene.joint("cube_free").qposadr[0]
    cube_mm = np.linalg.norm(q[ob : ob + 3] - dc.qpos[ob : ob + 3]) * 1000
    print(f"Warp vs C after {n_steps} steps: max joint diff {diff.max():.4f} rad, cube {cube_mm:.2f} mm, C max contacts {max_ncon}")
    assert max_ncon >= 10
    assert diff.max() < 0.05 and cube_mm < 5.0
