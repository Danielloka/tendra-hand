"""Build the robot assembly for training: OpenArm v2 (right arm J1-J4) + forearm twist + V2.1 hand.

    uv run python model/build_robot.py                 # write model/models/v21_robot.xml, v21_scene.xml
    uv run python model/build_robot.py --check         # exit 1 if the committed XMLs are stale
    uv run python model/build_robot.py --hand stub     # force the stand-in hand (default: auto)

Pipeline (see model/CONTRACT.md):  v21_hand.xml (or the stub)  ->  this file  ->  v21_robot.xml / v21_scene.xml.

Layout (OpenArm's world: +x forward, +z up, the right arm on the -y side, floor at z = 0):

    pedestal (static) -> right_shoulder_pitch/roll/yaw -> right_elbow (link4, with J5's motor)
      -> right_forearm_twist (OUR hinge at J5's place, + = pronation)
        -> V2.1 forearm (root body of the hand model, prefix `right_`) -> wrist_dev -> wrist_flex -> palm -> fingers

Mounting orientation (same as the main project's V1 arm, `software/tendra/arm.py`): the hand model's frame is
turned a half turn about its X axis, so the forearm's long axis (+Z of the hand = toward the fingers) lies along
J5's axis pointing away from the elbow (-z of link4, i.e. down when the arm hangs), the thumb side (+X) points
forward and the palm (-Y of the hand) faces the body's middle, like a human arm hanging relaxed ("handshake
position"). From there the elbow bent 90 deg gives forearm forward, thumb up, and pronating 90 deg gives palm down.

Everything is deterministic (fixed seeds, rounded keyframes) so `--check` can compare text.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from build_hand import PHYSICS

MODELS = ROOT / "models"
ASSETS = ROOT / "assets"
OPENARM_ASSETS = ASSETS / "openarm"
FK_REFERENCE = ASSETS / "v21" / "fk_reference.json"  # CAD poses (from the hardware-ideas study's poses.json)
HAND_XML = MODELS / "v21_hand.xml"
STUB_XML = MODELS / "_stub_hand.xml"
ROBOT_XML = MODELS / "v21_robot.xml"
SCENE_XML = MODELS / "v21_scene.xml"

SIDE = "right"
TIMESTEP = 0.002

# ----------------------------------------------------------------------------------------------------------
# Names (contract)
# ----------------------------------------------------------------------------------------------------------
SERVO_JOINTS = (
    "index_pip", "index_mcp_flex", "index_mcp_abd",
    "thumb_ip", "thumb_mcp_flex", "thumb_cmc_flex", "thumb_cmc_rot",
    "middle_pip", "middle_mcp_flex", "middle_mcp_abd",
    "ring_pip", "ring_mcp_flex", "ring_mcp_abd",
    "little_pip", "little_mcp_flex", "little_mcp_abd",
)  # fmt: skip
OPENARM_JOINTS = {"shoulder_pitch": 1, "shoulder_roll": 2, "shoulder_yaw": 3, "elbow": 4}
ARM_JOINTS = (*OPENARM_JOINTS, "forearm_twist", "wrist_flex", "wrist_dev")
FINGERS = ("index", "middle", "ring", "little")

# OpenArm's left J1-J3 turn the opposite way to the right ones; a future `side="left"` flips them.
_LEFT_FLIPPED = (1, 2, 3)

# Hand frame in J5's frame (link5 = the forearm twist body): half turn about x. quaternion wxyz.
HAND_IN_TWIST = np.array([0.0, 1.0, 0.0, 0.0])
J5_IN_LINK4 = np.array([0.0, 0.0, -0.0955])  # OpenArm's link5 position in link4 (J5 axis = link z)

# ----------------------------------------------------------------------------------------------------------
# Scene numbers
# ----------------------------------------------------------------------------------------------------------
TABLE_TOP = 0.40  # m; OpenArm's own tables (shoulders at 0.698 = 0.30 above the top)
TABLE_CENTRE = (0.45, 0.0)  # x, y of the table's middle (symmetric about the body, ready for a left arm)
TABLE_HALF = (0.33, 0.65)  # half size x, y
PALM_HEIGHT = 0.10  # `table` keyframe: palm centre this far above the table top
TABLE_POSE_XY = (0.30, -0.30)  # where the `table` keyframe puts the palm centre (above the table)

# Wrist camera: pose in the forearm (= hand frame at q = 0). A small camera board + housing (25 x 25 x 15 mm) bolted
# to the forearm's little-finger side, looking along the fingers, turned in toward the thumb and down toward the palm
# side. Chosen by model/camera_design.py: grasp-space visibility first, fingertips second, subject to "the housing's
# lowest point (palm down) is never below the hand's lowest collision point" (review B1).
WRIST_CAM_POS = (-0.070, -0.050, -0.050)
WRIST_CAM_TILT_DEG = 5.0  # tilted this far toward the palm side (-Y)
WRIST_CAM_YAW_DEG = 25.0  # turned this far toward the thumb side (+X)
CAM_HOUSING_HALF = (0.0125, 0.0125, 0.0075)  # camera x (image right), y (image up), z (behind the lens)
WRIST_CAM_FOVY_DEG = 93.0  # 640 x 480, horizontal FOV 110 deg
WRIST_CAM_RES = (640, 480)
# Head camera on a pole behind the shoulders.
POLE_XY = (-0.32, 0.0)
HEAD_CAM_POS = (-0.32, 0.0, 1.38)
HEAD_CAM_TARGET = (0.40, -0.20, 0.40)
HEAD_CAM_RES = (1280, 720)
HEAD_CAM_FOVY_DEG = 2 * math.degrees(math.atan(math.tan(math.radians(69.0 / 2)) * 720 / 1280))  # 42.3

ZETA = 1.0  # damping ratio the arm servos are tuned to (critical)

# Collision layers (contype/conaffinity bits). The hand file's layout (CONTRACT.md): bit 0 = the world (table,
# floor, objects: contype 1, conaffinity 1), bits 1-7 = its own parts. The arm's boxes get bit 8: they collide
# with the environment (whose conaffinity includes bit 8) and with nothing else; hand-vs-upper-arm contacts that
# can really happen are explicit <pair>s. The camera housing is hand-side: contype 1, conaffinity 0 (hits the
# environment, not the hand it is bolted to).
BIT_ARM = 256  # bit 8: the hand file uses bit 0 (the world) and bits 1-7 (its own parts), see CONTRACT.md
# Boxes (xmin, xmax, ymin, ymax, zmin, zmax) in the body frame: the extents of OpenArm's own collision hulls.
ARM_BOXES = {
    # the pedestal (OpenArm: one 2932-vertex hull; its cross-sections in the world frame): base plate, thin column, head
    "pedestal_base": (-0.155, 0.095, -0.095, 0.095, 0.0, 0.065),
    "pedestal_column": (-0.03, 0.03, -0.03, 0.03, 0.065, 0.585),
    "pedestal_head": (-0.085, 0.065, -0.08, 0.08, 0.585, 0.773),
    "base_link": (-0.049, 0.049, -0.062, 0.0, -0.061, 0.061),
    "link1": (-0.046, 0.049, -0.109, 0.0, -0.049, 0.049),
    "link2": (-0.048, 0.035, -0.049, 0.049, -0.066, 0.049),
    "link3": (-0.042, 0.035, -0.033, 0.033, -0.182, 0.0),
    "link4": (-0.042, 0.032, -0.039, 0.039, -0.097, 0.029),
}
HAND_ARM_PAIR_BOXES = (
    "link2",
    "link3",
    "pedestal_column",
    "pedestal_head",
)  # hand parts that may touch these: forearm, wrist ring, palm
HAND_ARM_PAIR_BODIES = ("forearm", "wrist_ring", "palm")


# ----------------------------------------------------------------------------------------------------------
# Small spec helpers
# ----------------------------------------------------------------------------------------------------------
def _position_actuator(spec, name, target, kp, kv, ctrlrange, forcerange=None):
    act = spec.add_actuator(name=name, target=target, trntype=mujoco.mjtTrn.mjTRN_JOINT)
    act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    act.gainprm[0] = kp
    act.biasprm[:3] = [0.0, -kp, -kv]
    act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
    act.ctrlrange = list(ctrlrange)
    if forcerange is not None:
        act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        act.forcerange = list(forcerange)
    return act


def _inertial(body, mass, pos, inertia) -> None:
    body.mass, body.ipos, body.inertia = mass, pos, inertia
    body.explicitinertial = True


def _quat_x(deg: float) -> list[float]:
    h = math.radians(deg) / 2
    return [math.cos(h), math.sin(h), 0.0, 0.0]


def look_at_xyaxes(pos, target, up) -> list[float]:
    """MuJoCo camera `xyaxes` (camera x, y in the parent frame) for a camera at `pos` looking at `target`."""
    pos, target, up = (np.asarray(v, float) for v in (pos, target, up))
    back = pos - target
    back /= np.linalg.norm(back)
    x = np.cross(up, back)
    x /= np.linalg.norm(x)
    return [*x, *np.cross(back, x)]


def cam_xyaxes_from_dir(direction, up) -> list[float]:
    direction, up = np.asarray(direction, float), np.asarray(up, float)
    return look_at_xyaxes(np.zeros(3), direction, up)


# ----------------------------------------------------------------------------------------------------------
# The stand-in hand (STUB) -- follows the contract's names exactly; sizes are rough
# ----------------------------------------------------------------------------------------------------------
def stub_hand_spec() -> mujoco.MjSpec:
    """A crude V2.1 hand + 2-DOF wrist + forearm (boxes and capsules) with the contract's names and order.
    Not the real design: replaced automatically by `model/models/v21_hand.xml` once that exists."""
    s = mujoco.MjSpec()
    s.modelname = "v21_stub_hand"
    s.option.timestep = TIMESTEP
    s.compiler.degree = False
    hinge = mujoco.mjtJoint.mjJNT_HINGE
    box, cap = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CAPSULE
    rgba_shell, rgba_skin = [0.82, 0.83, 0.86, 1], [0.95, 0.78, 0.6, 1]

    fa = s.worldbody.add_body(name="forearm")
    # 105 x 72 mm shell from the arm flange (z = -230 mm) to the wrist (z = -30 mm)
    fa.add_geom(name="forearm_shell", type=box, size=[0.0525, 0.036, 0.100], pos=[0, 0, -0.130], mass=0.40,
                rgba=rgba_shell, contype=1, conaffinity=1)  # fmt: skip
    fa.add_site(name="wrist_centre", pos=[0, 0, 0], size=[0.003])
    fa.add_site(name="forearm_flange", pos=[0, 0, -0.230], size=[0.003])

    ring = fa.add_body(name="wrist_ring")
    ring.add_joint(name="wrist_dev", type=hinge, axis=[0, 1, 0], range=[-math.radians(30), math.radians(20)],
                   limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=0.02, armature=0.002)  # fmt: skip
    ring.add_geom(name="wrist_ring_g", type=box, size=[0.045, 0.030, 0.015], pos=[0, 0, -0.015], mass=0.05,
                  rgba=rgba_shell, contype=0, conaffinity=0)  # fmt: skip
    palm = ring.add_body(name="palm")
    palm.add_joint(name="wrist_flex", type=hinge, axis=[1, 0, 0], range=[-math.radians(60), math.radians(70)],
                   limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=0.02, armature=0.002)  # fmt: skip
    palm.add_geom(name="palm_g", type=box, size=[0.040, 0.014, 0.040], pos=[0, 0, 0.040], mass=0.15,
                  rgba=rgba_shell, contype=1, conaffinity=1, friction=[1.0, 0.01, 0.001])  # fmt: skip
    palm.add_site(
        name="palm_centre", pos=[0, -0.014, 0.045], quat=_quat_x(90), size=[0.003]
    )  # z = out of palm

    servo_kw = {"kp": 1.5, "kv": 0.04, "forcerange": (-0.25, 0.25)}

    def joint(body, name, axis, rng):
        body.add_joint(name=name, type=hinge, axis=axis, range=list(rng),
                       limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=0.01, armature=2e-4)  # fmt: skip

    def seg(body, name, length, radius=0.0085, mass=0.012):
        body.add_geom(name=name, type=cap, fromto=[0, 0, 0, 0, 0, length], size=[radius, 0, 0], mass=mass,
                      rgba=rgba_skin, contype=1, conaffinity=1, friction=[1.0, 0.01, 0.001])  # fmt: skip

    xs = {"index": 0.030, "middle": 0.010, "ring": -0.010, "little": -0.029}
    lens = {"index": (0.045, 0.027, 0.022), "middle": (0.050, 0.030, 0.022),
            "ring": (0.046, 0.028, 0.022), "little": (0.038, 0.022, 0.020)}  # fmt: skip
    for f in FINGERS:
        yoke = palm.add_body(name=f"{f}_yoke", pos=[xs[f], 0, 0.090])
        joint(yoke, f"{f}_mcp_abd", [0, 1, 0], (-0.35, 0.35))
        _inertial(yoke, 0.004, [0, 0, 0], [2e-7] * 3)
        prox = yoke.add_body(name=f"{f}_proximal")
        joint(prox, f"{f}_mcp_flex", [1, 0, 0], (-0.2, 1.6))
        seg(prox, f"{f}_proximal_g", lens[f][0])
        mid = prox.add_body(name=f"{f}_middle", pos=[0, 0, lens[f][0]])
        joint(mid, f"{f}_pip", [1, 0, 0], (0.0, 1.9))
        seg(mid, f"{f}_middle_g", lens[f][1])
        dist = mid.add_body(name=f"{f}_distal", pos=[0, 0, lens[f][1]])
        joint(dist, f"{f}_dip", [1, 0, 0], (0.0, 1.4))
        seg(dist, f"{f}_distal_g", lens[f][2], mass=0.008)
        dist.add_site(name=f"{f}_tip", pos=[0, -0.005, lens[f][2]], size=[0.003])
    # thumb: at the thumb-side edge of the palm, pointing along the fingers and 35 deg toward the palm side
    tb = palm.add_body(name="thumb_base", pos=[0.052, -0.010, 0.020], quat=_quat_x(35))
    joint(tb, "thumb_cmc_rot", [0, -1, 0], (-0.6, 1.3))
    _inertial(tb, 0.006, [0, 0, 0], [5e-7] * 3)
    tm = tb.add_body(name="thumb_metacarpal")
    joint(tm, "thumb_cmc_flex", [1, 0, 0], (-0.3, 1.2))
    seg(tm, "thumb_metacarpal_g", 0.035, mass=0.015)
    tp = tm.add_body(name="thumb_proximal", pos=[0, 0, 0.035])
    joint(tp, "thumb_mcp_flex", [1, 0, 0], (-0.2, 1.4))
    seg(tp, "thumb_proximal_g", 0.032, mass=0.012)
    td = tp.add_body(name="thumb_distal", pos=[0, 0, 0.032])
    joint(td, "thumb_ip", [1, 0, 0], (-0.2, 1.4))
    seg(td, "thumb_distal_g", 0.026, mass=0.008)
    td.add_site(name="thumb_tip", pos=[0, -0.005, 0.026], size=[0.003])

    for f in FINGERS:  # DIP follows the PIP (the real hand: a rigid bar)
        eq = s.add_equality(
            name=f"{f}_dip_couple", type=mujoco.mjtEq.mjEQ_JOINT, name1=f"{f}_dip", name2=f"{f}_pip"
        )
        eq.data[:5] = [0.0, 0.75, 0.0, 0.0, 0.0]

    for name in SERVO_JOINTS:
        j = s.joint(name)
        _position_actuator(s, name, name, ctrlrange=j.range, **servo_kw)
    _position_actuator(s, "wrist_flex", "wrist_flex", 12.0, 0.3, s.joint("wrist_flex").range, (-4, 4))
    _position_actuator(s, "wrist_dev", "wrist_dev", 12.0, 0.3, s.joint("wrist_dev").range, (-4, 4))
    return s


# ----------------------------------------------------------------------------------------------------------
# OpenArm
# ----------------------------------------------------------------------------------------------------------
def trim_openarm(side: str = SIDE, vendored: bool = True) -> mujoco.MjSpec:
    """OpenArm's pedestal with one arm: base, J1-J4 (renamed `<side>_shoulder_pitch` ...), the J5 motor in link4;
    link5 and below (forearm, wrist, gripper) removed; the other arm removed; its cameras/lights/keyframes too.
    `vendored`: collision = boxes (ARM_BOXES) and the visual meshes are the unchanged copies in
    model/assets/openarm/ (so no file outside the study is needed); False: OpenArm's own geoms (for vendoring)."""
    from openarm_mujoco.v2 import asset_path

    spec = mujoco.MjSpec.from_file(asset_path("pedestal/openarm_pedestal.xml"))
    other = "left" if side == "right" else "right"
    keep = {f"openarm_{side}_joint{i}" for i in range(1, 5)}
    for act in list(spec.actuators):
        if act.target not in keep:
            spec.delete(act)
    for collection in (spec.equalities, spec.keys, spec.cameras, spec.lights, spec.excludes):
        for item in list(collection):
            spec.delete(item)
    spec.delete(spec.body(f"openarm_{other}_base_link"))
    spec.delete(spec.body(f"openarm_{side}_link5"))
    for name, number in OPENARM_JOINTS.items():
        old, new = f"openarm_{side}_joint{number}", f"{side}_{name}"
        joint = spec.joint(old)
        acts = [a for a in spec.actuators if a.target == old]
        if side == "left" and number in _LEFT_FLIPPED:
            lo, hi = joint.range
            joint.axis = [-a for a in joint.axis]
            joint.range = [-hi, -lo]
            for act in acts:
                lo, hi = act.ctrlrange
                act.ctrlrange = [-hi, -lo]
        joint.name = new
        for act in acts:
            act.name, act.target = new, new
    if vendored:
        for geom in list(spec.geoms):  # OpenArm's collision hulls (group 3): replaced by boxes
            if geom.group == 3:
                spec.delete(geom)
        _drop_unused_meshes(spec)
        for mesh in spec.meshes:
            suffix = Path(mesh.file).suffix  # OpenArm's own files, copied unchanged (model/vendor_openarm.py)
            mesh.file = str(
                OPENARM_ASSETS / f"{mesh.name}{suffix}"
            )  # absolute while building; relativised later
        spec.meshdir = str(ASSETS)
        _add_arm_boxes(spec, side)
    else:
        _drop_unused_meshes(spec)
    return spec


def _add_arm_boxes(spec: mujoco.MjSpec, side: str) -> None:
    for key, (x0, x1, y0, y1, z0, z1) in ARM_BOXES.items():
        pedestal = key.startswith("pedestal")
        body = spec.worldbody if pedestal else spec.body(f"openarm_{side}_{key}")
        name = f"{key}_box" if pedestal else f"{side}_{key}_box"
        body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=[(x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2],
                      pos=[(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2], contype=BIT_ARM, conaffinity=0, group=3,
                      friction=[1.0, 0.01, 0.01], rgba=[0.9, 0.3, 0.3, 0.3])  # fmt: skip


def _drop_unused_meshes(spec: mujoco.MjSpec) -> None:
    used = {g.meshname for g in spec.geoms if g.meshname}
    for mesh in list(spec.meshes):
        if mesh.name not in used:
            spec.delete(mesh)


# ----------------------------------------------------------------------------------------------------------
# Assembly
# ----------------------------------------------------------------------------------------------------------
def load_hand(which: str) -> tuple[mujoco.MjSpec, str]:
    """(spec, label) for `stub`, `real` or `auto` (the real file when it exists)."""
    if which == "auto":
        which = "real" if HAND_XML.exists() else "stub"
    if which == "stub":
        return stub_hand_spec(), "stub"
    if not HAND_XML.exists():
        raise SystemExit(f"{HAND_XML} does not exist (use --hand stub|auto)")
    hand = mujoco.MjSpec.from_file(str(HAND_XML))
    return hand, "real"


def _hand_mesh_paths(hand: mujoco.MjSpec, assets_dir: Path, xml_dir: Path) -> None:
    """Re-point the hand's mesh files at paths relative to model/assets (the merged spec has one meshdir)."""
    meshdir = Path(hand.meshdir) if hand.meshdir else Path(".")
    if not meshdir.is_absolute():
        meshdir = xml_dir / meshdir
    for mesh in hand.meshes:
        if mesh.file:
            mesh.file = str(
                (meshdir / mesh.file).resolve()
            )  # absolute while building; the XML text is relativised
    hand.meshdir = str(assets_dir)


def build_robot_spec(hand: mujoco.MjSpec, side: str = SIDE) -> mujoco.MjSpec:
    """Arm + twist + hand + cameras + table + floor + lights. No keyframes, no test objects."""
    assets = ASSETS
    spec = trim_openarm(side)
    spec.modelname = "v21_robot"
    spec.meshdir = str(assets)
    # --- solver / physics ---------------------------------------------------------------------------------
    opt = spec.option
    opt.timestep = PHYSICS["timestep"]
    opt.integrator = getattr(
        mujoco.mjtIntegrator, f"mjINT_{PHYSICS['integrator'].upper()}"
    )  # implicit damping
    opt.cone = getattr(
        mujoco.mjtCone, f"mjCONE_{PHYSICS['cone'].upper()}"
    )  # true friction cone: exact grasp friction
    opt.impratio = PHYSICS["impratio"]  # stiff friction: held objects do not creep
    opt.solver = getattr(mujoco.mjtSolver, f"mjSOL_{PHYSICS['solver'].upper()}")
    opt.iterations = 50
    opt.ls_iterations = 30
    opt.tolerance = 1e-8
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 720

    # --- forearm twist at J5 ------------------------------------------------------------------------------
    link4 = spec.body(f"openarm_{side}_link4")
    twist = link4.add_body(name=f"{side}_forearm_twist_link", pos=list(J5_IN_LINK4))
    _inertial(twist, 0.15, [0, 0, -0.01], [2e-4, 2e-4, 1e-4])  # J5's output flange + adapter
    # axis +z (link frame) = pronation for the right arm: the hand's fingers point along link -z, and rotating about
    # the finger direction by -a is rotating about +z by +a; palm medial (+y) turns toward down (-z) as a grows.
    sign = 1.0 if side == "right" else -1.0
    twist.add_joint(name=f"{side}_forearm_twist", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, sign],
                    range=[-math.pi / 2, math.pi / 2], limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                    damping=0.01, armature=0.01, frictionloss=0.04)  # fmt: skip

    # --- actuators in the order the policy wants: the 7 arm joints first, then (by the attach) the 16 servos.
    # The wrist actuators come from the hand file: keep their parameters, drop them from the hand, re-create them
    # here (targets resolve by name at compile time, so the joints may not exist yet).
    # OpenArm J5's torque limit (+-7 Nm) but kp 80 (stock 30): the hand and forearm are heavier than a gripper and,
    # not being gravity-compensated, sag a little with the thumb up (kp 30 gave 2.2 deg)
    _position_actuator(
        spec,
        f"{side}_forearm_twist",
        f"{side}_forearm_twist",
        80.0,
        1.5,
        (-math.pi / 2, math.pi / 2),
        (-7.0, 7.0),
    )
    for name in ("wrist_flex", "wrist_dev"):
        old = hand.actuator(name)
        new = spec.add_actuator(
            name=f"{side}_{name}", target=f"{side}_{name}", trntype=mujoco.mjtTrn.mjTRN_JOINT
        )
        for attr in (
            "gaintype",
            "biastype",
            "dyntype",
            "gainprm",
            "biasprm",
            "dynprm",
            "ctrllimited",
            "ctrlrange",
            "forcelimited",
            "forcerange",
            "actlimited",
            "actrange",
            "gear",
            "group",
        ):
            setattr(new, attr, getattr(old, attr))
        hand.delete(old)

    # --- sensors of the 7 arm joints (joint angle, actuator force), in actuator order; created before the attach so
    # they come first. The hand's own wrist sensors are dropped (the wrist joints are ours here).
    for sensor in list(hand.sensors):
        if sensor.objname in ("wrist_flex", "wrist_dev"):
            hand.delete(sensor)
    for joint in ARM_JOINTS:
        spec.add_sensor(name=f"{side}_{joint}_pos", type=mujoco.mjtSensor.mjSENS_JOINTPOS,
                        objtype=mujoco.mjtObj.mjOBJ_JOINT, objname=f"{side}_{joint}")  # fmt: skip
    for joint in ARM_JOINTS:
        spec.add_sensor(name=f"{side}_{joint}_frc", type=mujoco.mjtSensor.mjSENS_ACTUATORFRC,
                        objtype=mujoco.mjtObj.mjOBJ_ACTUATOR, objname=f"{side}_{joint}")  # fmt: skip

    # --- the hand ------------------------------------------------------------------------------------------
    flange = np.array(hand.site("forearm_flange").pos)
    hand_xml_dir = HAND_XML.parent if HAND_XML.exists() else MODELS
    if hand.meshes:
        _hand_mesh_paths(hand, assets, hand_xml_dir)
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, HAND_IN_TWIST)
    frame = twist.add_frame(pos=list(-rot.reshape(3, 3) @ flange), quat=list(HAND_IN_TWIST))
    spec.attach(hand, frame=frame, prefix=f"{side}_")
    # the twist body is massless geometry-wise: link4 and the forearm must not collide with each other
    spec.add_exclude(
        name=f"{side}_link4_forearm", bodyname1=f"openarm_{side}_link4", bodyname2=f"{side}_forearm"
    )

    _add_hand_arm_pairs(spec, side)
    _add_gravcomp(spec, side)
    _add_cameras(spec, side)
    _add_scene_basics(spec)
    return spec


def _add_hand_arm_pairs(spec: mujoco.MjSpec, side: str) -> None:
    """Explicit contact pairs between the hand's big parts (forearm shell, wrist ring, palm) and the upper-arm /
    pedestal boxes: the only hand-vs-arm contacts that can physically happen (wrist and elbow bent far); the fingers
    cannot reach them. Everything else of the arm collides with the environment only (BIT_ARM)."""
    model = spec.compile()
    hand_bodies = {f"{side}_{b}" for b in HAND_ARM_PAIR_BODIES}
    boxes = [f"{k}_box" if k.startswith("pedestal") else f"{side}_{k}_box" for k in HAND_ARM_PAIR_BOXES]
    for g in range(model.ngeom):
        body = model.body(model.geom_bodyid[g]).name
        if body in hand_bodies and (model.geom_contype[g] or model.geom_conaffinity[g]):
            for box in boxes:
                spec.add_pair(geomname1=model.geom(g).name, geomname2=box)


def _add_gravcomp(spec: mujoco.MjSpec, side: str) -> None:
    """Gravity compensation for the arm only: OpenArm's links and the twist link (the arm is a stand-in for a real
    arm with a gravity-compensating controller). The forearm, wrist, palm and fingers carry their own weight, as the
    real servos do; objects are not compensated either (review m1)."""
    for body in spec.bodies:
        if body.name.startswith(f"openarm_{side}_link") or body.name == f"{side}_forearm_twist_link":
            body.gravcomp = 1.0


def _add_cameras(spec: mujoco.MjSpec, side: str) -> None:
    forearm = spec.body(f"{side}_forearm")
    tilt, yaw = math.radians(WRIST_CAM_TILT_DEG), math.radians(WRIST_CAM_YAW_DEG)
    direction = [math.sin(yaw) * math.cos(tilt), -math.sin(tilt), math.cos(yaw) * math.cos(tilt)]
    xyaxes = cam_xyaxes_from_dir(direction, [0, 1, 0])  # back of the hand up in the image
    cam = forearm.add_camera(name=f"{side}_wrist_cam", pos=list(WRIST_CAM_POS), fovy=WRIST_CAM_FOVY_DEG,
                             resolution=list(WRIST_CAM_RES))  # fmt: skip
    cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
    cam.alt.xyaxes = xyaxes
    site = forearm.add_site(name=f"{side}_wrist_cam_mount", pos=list(WRIST_CAM_POS), size=[0.004],
                            rgba=[0.1, 0.8, 0.2, 1], group=4)  # fmt: skip
    site.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
    site.alt.xyaxes = xyaxes
    # housing: a real collision box (camera board + case) behind the lens, plus a visual twin and a lens disc
    x = np.array(xyaxes[:3])
    y = np.array(xyaxes[3:])
    z = np.cross(x, y)  # camera z points backward (the camera looks along -z)
    centre = np.array(WRIST_CAM_POS) + z * CAM_HOUSING_HALF[2]
    rot = np.column_stack([x, y, z])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rot.ravel())
    for suffix, group, contype in (("housing", 4, 1), ("housing_vis", 2, 0)):  # collision box + visual twin
        forearm.add_geom(name=f"{side}_wrist_cam_{suffix}", type=mujoco.mjtGeom.mjGEOM_BOX, size=list(CAM_HOUSING_HALF),
                         pos=list(centre), quat=list(quat), mass=0.015 if contype else 0.0, group=group,
                         contype=contype, conaffinity=0, rgba=[0.1, 0.25, 0.1, 1])  # fmt: skip
    forearm.add_geom(name=f"{side}_wrist_cam_lens", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.005, 0.002, 0],
                     pos=list(np.array(WRIST_CAM_POS) - z * 0.0015), quat=list(quat), contype=0, conaffinity=0,
                     group=2, rgba=[0.02, 0.02, 0.05, 1])  # fmt: skip

    # head camera on a pole behind and above the shoulders
    pole = spec.worldbody.add_body(name="head_pole", pos=[POLE_XY[0], POLE_XY[1], 0.0])
    _inertial(
        pole, 5.0, [0, 0, 0.7], [1.0, 1.0, 0.01]
    )  # static (no joint): the mass is only explicit, not used
    pole.add_geom(name="head_pole_post", type=mujoco.mjtGeom.mjGEOM_CYLINDER, fromto=[0, 0, 0.0, 0, 0, HEAD_CAM_POS[2]],
                  size=[0.018, 0, 0], rgba=[0.2, 0.2, 0.22, 1], contype=0, conaffinity=0, group=2)  # fmt: skip
    pole.add_geom(name="head_pole_foot", type=mujoco.mjtGeom.mjGEOM_CYLINDER, fromto=[0, 0, 0.0, 0, 0, 0.012],
                  size=[0.11, 0, 0], rgba=[0.2, 0.2, 0.22, 1], contype=0, conaffinity=0, group=2)  # fmt: skip
    pole.add_geom(name="head_cam_body", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.012, 0.045, 0.012],
                  pos=[0, 0, HEAD_CAM_POS[2] + 0.02], rgba=[0.05, 0.05, 0.05, 1], contype=0, conaffinity=0, group=2)  # fmt: skip
    cam_pos = np.array(HEAD_CAM_POS) - np.array([POLE_XY[0], POLE_XY[1], 0.0])
    cam = pole.add_camera(
        name="head_cam", pos=list(cam_pos), fovy=HEAD_CAM_FOVY_DEG, resolution=list(HEAD_CAM_RES)
    )
    cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
    cam.alt.xyaxes = look_at_xyaxes(HEAD_CAM_POS, HEAD_CAM_TARGET, [0, 0, 1])


def _add_scene_basics(spec: mujoco.MjSpec) -> None:
    wb = spec.worldbody
    wb.add_light(name="key", pos=[0.6, -0.6, 2.2], dir=[-0.2, 0.3, -1.0], castshadow=True, diffuse=[0.65, 0.65, 0.65],
                 specular=[0.2, 0.2, 0.2])  # fmt: skip
    wb.add_light(
        name="fill", pos=[-0.8, 0.8, 1.8], dir=[0.4, -0.4, -1.0], castshadow=False, diffuse=[0.3, 0.3, 0.3]
    )
    spec.add_material(name="floor_mat", rgba=[0.62, 0.64, 0.67, 1], reflectance=0.1)
    spec.add_material(name="table_mat", rgba=[0.78, 0.66, 0.52, 1])
    spec.add_material(name="leg_mat", rgba=[0.25, 0.25, 0.27, 1])
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1], material="floor_mat",
                friction=[0.8, 0.01, 0.0001], contype=1, conaffinity=1 | BIT_ARM)  # fmt: skip
    ov = wb.add_camera(
        name="overview", pos=[1.25, 0.95, 1.25], fovy=45.0, resolution=[960, 720]
    )  # for renders
    ov.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
    ov.alt.xyaxes = look_at_xyaxes([1.25, 0.95, 1.25], [0.25, -0.2, 0.5], [0, 0, 1])
    cx, cy = TABLE_CENTRE
    hx, hy = TABLE_HALF
    wb.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX, size=[hx, hy, 0.02], pos=[cx, cy, TABLE_TOP - 0.02],
                material="table_mat", friction=[0.8, 0.01, 0.0001], contype=1, conaffinity=1 | BIT_ARM)  # fmt: skip
    for i, (sx, sy) in enumerate(
        ((1, 1), (1, -1), (-1, 1), (-1, -1))
    ):  # legs (visual only; kept out of the arm's way)
        wb.add_geom(name=f"table_leg{i}", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.025, 0.025, (TABLE_TOP - 0.04) / 2],
                    pos=[cx + sx * (hx - 0.04), cy + sy * (hy - 0.04), (TABLE_TOP - 0.04) / 2], material="leg_mat",
                    contype=0, conaffinity=0)  # fmt: skip


# test objects: name -> (geom type, size, mass kg, rgba, position (x, y), half height above table)
OBJECTS = {
    "cube": (mujoco.mjtGeom.mjGEOM_BOX, [0.025, 0.025, 0.025], 0.12, [0.2, 0.45, 0.85, 1], (0.28, -0.43)),
    "cylinder": (mujoco.mjtGeom.mjGEOM_CYLINDER, [0.030, 0.050], 0.15, [0.85, 0.25, 0.2, 1], (0.42, -0.43)),
    "ball": (mujoco.mjtGeom.mjGEOM_SPHERE, [0.030], 0.06, [0.95, 0.75, 0.15, 1], (0.30, -0.25)),
    "mug_box": (mujoco.mjtGeom.mjGEOM_BOX, [0.040, 0.040, 0.0475], 0.30, [0.3, 0.7, 0.4, 1], (0.52, -0.30)),
}


def add_objects(spec: mujoco.MjSpec) -> None:
    for name, (gtype, size, mass, rgba, (x, y)) in OBJECTS.items():
        half_h = {
            mujoco.mjtGeom.mjGEOM_BOX: 2,
            mujoco.mjtGeom.mjGEOM_CYLINDER: 1,
            mujoco.mjtGeom.mjGEOM_SPHERE: 0,
        }[gtype]
        half_h = size[half_h]
        size = [*size, 0.0, 0.0][:3]
        body = spec.worldbody.add_body(name=name, pos=[x, y, TABLE_TOP + half_h + 1e-3])
        body.add_freejoint(name=f"{name}_free")
        # no `priority`: the hand's pad settings (condim 4, friction, solref) win in a contact (review B2); the
        # object's own values are pad-compatible anyway
        body.add_geom(name=name, type=gtype, size=size, mass=mass, rgba=rgba, friction=[1.0, 0.02, 0.002],
                      solref=[0.005, 1.0], condim=4, contype=1, conaffinity=1 | BIT_ARM)  # fmt: skip


# ----------------------------------------------------------------------------------------------------------
# Kinematics helpers, IK for the keyframes, damping
# ----------------------------------------------------------------------------------------------------------
def arm_joint_names(side: str = SIDE) -> list[str]:
    return [f"{side}_{n}" for n in ARM_JOINTS]


def hand_joint_names(side: str = SIDE) -> list[str]:
    return [f"{side}_{n}" for n in SERVO_JOINTS]


def _rot_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    m = current @ target.T
    return 0.5 * np.array([m[1, 2] - m[2, 1], m[2, 0] - m[0, 2], m[0, 1] - m[1, 0]])


def palm_target_rotation(yaw: float = 0.0, pitch: float = 0.0) -> np.ndarray:
    """Palm frame (hand axes: x thumb side, y back of the hand, z along the fingers) with the palm facing down and
    the fingers pointing forward (+x world) turned by `yaw` about the vertical, and tilted down by `pitch`."""
    c, s = math.cos(yaw), math.sin(yaw)
    z = np.array([c * math.cos(pitch), s * math.cos(pitch), -math.sin(pitch)])
    up = np.array([0.0, 0.0, 1.0])
    y = up - (up @ z) * z
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def solve_arm_pose(model, pos, rot, side=SIDE, q_nom=None, seeds=24, rng_seed=0, iters=200, rot_weight=0.5):
    """Damped-least-squares IK of the 7 arm joints so that the palm (body `<side>_palm`, site `<side>_palm_centre`)
    sits at `pos` with orientation `rot`. Returns (q_arm, error). Multi-start, deterministic."""
    names = arm_joint_names(side)
    jid = np.array([model.joint(n).id for n in names])
    qadr, dof = model.jnt_qposadr[jid], model.jnt_dofadr[jid]
    lo, hi = model.jnt_range[jid].T
    site = model.site(f"{side}_palm_centre").id
    body = model.body(f"{side}_palm").id
    data = mujoco.MjData(model)
    q_nom = np.zeros(len(names)) if q_nom is None else np.asarray(q_nom, float)
    rng = np.random.default_rng(rng_seed)
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    best = (None, np.inf)
    for k in range(seeds):
        q = q_nom.copy() if k == 0 else np.clip(q_nom + rng.normal(0, 0.5, len(names)), lo, hi)
        for _ in range(iters):
            data.qpos[qadr] = q
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            err = np.concatenate([pos - data.site_xpos[site],
                                  rot_weight * _rot_error(data.xmat[body].reshape(3, 3), rot)])  # fmt: skip
            if np.linalg.norm(err) < 1e-4:
                break
            mujoco.mj_jacSite(model, data, jp, jr, site)
            jac = np.vstack([jp[:, dof], rot_weight * jr[:, dof]])
            pinv = jac.T @ np.linalg.inv(jac @ jac.T + 1e-4 * np.eye(6))
            dq = pinv @ err + (np.eye(len(names)) - pinv @ jac) @ (0.05 * (q_nom - q))
            step = np.abs(dq).max()
            if step > 0.2:
                dq *= 0.2 / step
            q = np.clip(q + dq, lo, hi)
        data.qpos[qadr] = q
        mujoco.mj_kinematics(model, data)
        err = np.concatenate(
            [pos - data.site_xpos[site], rot_weight * _rot_error(data.xmat[body].reshape(3, 3), rot)]
        )
        e = float(np.linalg.norm(err))
        # prefer small error, then close to the nominal posture
        score = e + 1e-3 * np.linalg.norm(q - q_nom)
        if e < 2e-3 and (best[0] is None or score < best[1]):
            best = (q.copy(), score)
        elif best[0] is None and e < best[1]:
            best = (q.copy(), e)
    return best[0], best[1]


def pose_qpos(model, joint_values: dict[str, float]) -> np.ndarray:
    qpos = model.qpos0.copy()
    for name, v in joint_values.items():
        qpos[model.joint(name).qposadr[0]] = v
    return qpos


def lowest_point_z(model, qpos, side=SIDE) -> float:
    """Lowest z of all hand/forearm geoms (centre z - bounding radius) at this pose."""
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)
    zs = []
    for g in range(model.ngeom):
        b = model.body(model.geom_bodyid[g]).name
        if b.startswith(f"{side}_") and model.geom_contype[g]:
            zs.append(data.geom_xpos[g][2] - model.geom_rbound[g])
    return min(zs)


def tune_damping(spec: mujoco.MjSpec, keys: dict, side: str = SIDE, zeta: float = ZETA) -> dict[str, float]:
    """Raise the actuator damping (kv) of the 7 arm joints so each is at least `zeta` damped at the worst pose of
    the keyframes (largest effective inertia). kp is not touched. Returns {joint: kv}."""
    model = spec.compile()
    poses = [pose_qpos(model, {f"{side}_{k}": v for k, v in p.items()}) for p in keys.values()]
    data = mujoco.MjData(model)
    mass = np.zeros((model.nv, model.nv))
    result = {}
    inertia = {n: 0.0 for n in arm_joint_names(side)}
    for qpos in poses:
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        mujoco.mj_fullM(model, data, mass)
        for n, current in inertia.items():
            d = model.jnt_dofadr[model.joint(n).id]
            inertia[n] = max(current, mass[d, d])
    for n, inert in inertia.items():
        act = spec.actuator(n)
        kp = act.gainprm[0]
        damping = model.dof_damping[model.jnt_dofadr[model.joint(n).id]]
        kv_needed = 2.0 * zeta * math.sqrt(kp * inert) - damping
        kv = max(-act.biasprm[2], kv_needed)
        act.biasprm[2] = -round(kv, 3)
        result[n] = round(kv, 3)
    return result


# Keyframe joint values that do not need IK. `table` is solved (needs the compiled model).
KEY_JOINTS: dict[str, dict[str, float]] = {
    "rest": {
        "shoulder_roll": 0.6,
        "elbow": 0.1,
    },  # arm hangs, 34 deg out: hand clear of floor, pedestal and table
    "ready": {"shoulder_pitch": 0.6, "elbow": math.pi / 2 - 0.6},  # forearm level, raised: clear of the table
}


def open_pose() -> dict[str, float]:
    """The CAD's `open` pose of the hand (rad): the thumb is not straight but in its open position (review M2)."""
    q = json.loads(FK_REFERENCE.read_text())["poses"]["open"]["q"]
    return {k: math.radians(v) for k, v in q.items() if k in SERVO_JOINTS}


def solve_keyframes(spec: mujoco.MjSpec, side: str = SIDE) -> dict[str, dict[str, float]]:
    """All keyframes' joint values: rest, ready (constants) and table (IK: palm 10 cm above the table, palm down).
    `ready` and `table` carry the CAD open thumb; hand joints are keyed by their short names (`thumb_ip`)."""
    model = spec.compile()
    q_nom = np.array(
        [0.3, 0.17, 0.0, 1.6, 1.0, 0.0, 0.0]
    )  # a comfortable guess: elbow bent, forearm pronated
    target = np.array([TABLE_POSE_XY[0], TABLE_POSE_XY[1], TABLE_TOP + PALM_HEIGHT])
    q, err = solve_arm_pose(model, target, palm_target_rotation(0.0), side, q_nom)
    if q is None or err > 2e-3:
        raise RuntimeError(f"IK for the `table` keyframe failed (error {err:.4f})")
    thumb = {k: round(v, 5) for k, v in open_pose().items()}
    keys = {k: dict(joints) for k, joints in KEY_JOINTS.items()}
    keys["ready"].update(thumb)
    keys["table"] = {n: round(float(v), 4) for n, v in zip(ARM_JOINTS, q)} | thumb
    return keys


def actuator_qpos_index(model: mujoco.MjModel) -> np.ndarray:
    """qpos address of the joint each actuator drives, in actuator order (the kinematic tree order differs from the
    actuator order: e.g. the wrist joints come before the servo joints in qpos but after the arm in ctrl)."""
    return np.array([model.jnt_qposadr[model.actuator_trnid[a, 0]] for a in range(model.nu)])


def actuator_qvel_index(model: mujoco.MjModel) -> np.ndarray:
    return np.array([model.jnt_dofadr[model.actuator_trnid[a, 0]] for a in range(model.nu)])


def add_keyframes(spec: mujoco.MjSpec, keys: dict[str, dict[str, float]], side: str = SIDE) -> None:
    model = spec.compile()
    for key, joints in keys.items():
        qpos = pose_qpos(model, {f"{side}_{n}": v for n, v in joints.items()})
        ctrl = np.zeros(model.nu)
        for n, v in joints.items():
            ctrl[model.actuator(f"{side}_{n}").id] = v
        for a in range(
            model.nu
        ):  # hand servos and anything else: hold q = 0 (fingers open); clamp to ctrlrange
            lo, hi = model.actuator_ctrlrange[a]
            ctrl[a] = min(max(ctrl[a], lo), hi)
        spec.add_key(
            name=key, qpos=[round(float(v), 6) for v in qpos], ctrl=[round(float(v), 6) for v in ctrl]
        )


# ----------------------------------------------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------------------------------------------
def _to_xml(spec: mujoco.MjSpec) -> str:
    """Spec -> XML text with a portable meshdir (relative to model/models)."""
    spec.compile()
    text = spec.to_xml()
    rel = os.path.relpath(ASSETS.resolve(), MODELS.resolve())  # "../assets": the XMLs are self-contained
    text = re.sub(r'meshdir="[^"]*"', f'meshdir="{rel}"', text)
    text = text.replace(f'file="{ASSETS.resolve()}/', 'file="')  # mesh files relative to meshdir
    return _unwrap_anonymous_defaults(text)


def _unwrap_anonymous_defaults(text: str) -> str:
    """MjSpec.attach writes the attached model's top-level <default> as a nameless nested <default>, which MuJoCo
    refuses to load: replace each by its children."""
    import xml.etree.ElementTree as ET

    root = ET.fromstring(text)

    def unwrap(parent):
        out = []
        for child in list(parent):
            if child.tag == "default" and "class" not in child.attrib and parent.tag == "default":
                out.extend(child)
            else:
                out.append(child)
        parent[:] = out
        for child in parent:
            unwrap(child)

    for element in root.iter("default"):
        pass
    unwrap(root)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def build_all(which: str = "auto") -> tuple[dict[Path, str], str]:
    """(files, hand label): the texts of v21_robot.xml, v21_scene.xml and (stub only) _stub_hand.xml."""
    files: dict[Path, str] = {}
    hand, label = load_hand(which)
    if label == "stub":
        stub = stub_hand_spec()
        stub.compile()
        files[STUB_XML] = stub.to_xml()
    robot = build_robot_spec(hand)
    keys = solve_keyframes(robot)
    tune_damping(robot, keys)
    scene = robot.copy()  # the scene gets the same tuned gains
    add_objects(scene)
    for spec in (robot, scene):
        add_keyframes(spec, keys)
    files[ROBOT_XML] = _to_xml(robot)
    files[SCENE_XML] = _to_xml(scene)
    return files, label


def _normalise(text: str) -> str:
    return text


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--hand", choices=["stub", "real", "auto"], default="auto")
    ap.add_argument("--check", action="store_true", help="exit 1 if the committed XMLs are stale")
    args = ap.parse_args(argv)
    files, label = build_all(args.hand)
    if args.check:
        stale = [p for p, t in files.items() if not p.exists() or _normalise(p.read_text()) != _normalise(t)]
        if stale:
            print("STALE:", ", ".join(str(p.relative_to(ROOT)) for p in stale))
            return 1
        print(f"up to date ({label} hand)")
        return 0
    MODELS.mkdir(parents=True, exist_ok=True)
    for path, text in files.items():
        path.write_text(text)
        print("wrote", path.relative_to(ROOT.parent))
    print(f"hand: {label}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
