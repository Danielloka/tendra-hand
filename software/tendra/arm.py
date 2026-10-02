"""The Tendra robot body: two arms, OpenArm's shoulders and elbows, Tendra's forearms, wrists, hands.

One MuJoCo model with both arms, like a human (CLAUDE.md: two arms with two Tendra hands on a
fixed pole). The shoulders and elbows come from OpenArm v2 (github.com/enactic/openarm,
Apache-2.0, the `openarm-mujoco` package): its pedestal, J1-J3 (shoulder), J4 (elbow) and the J5
motor at the bottom of each elbow link. Everything from J5's flange down is Tendra's: the forearm
(turned by J5), a 2-way wrist and the hand, built by sim/convert_v1_wrist.py (the left one is the
mirror image of the right one, see hardware/cad/mirror_export.py).

    model = load_arm_model()                 # or arm_spec() to add a scene around it
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("ready").id)

Every name carries its side: `right_shoulder_pitch`, `left_index_pip`, `right_palm`, ... Per arm,
7 arm joints (each driven by a position actuator of the same name), then the hand's 16 servos
(V1 order, `tendra.joints.V1`):

    shoulder_pitch  OpenArm J1, + = arm forward (flexion)
    shoulder_roll   OpenArm J2, + = arm out to the side (abduction)
    shoulder_yaw    OpenArm J3, + = upper arm turns outward (external rotation)
    elbow           OpenArm J4, + = bend (0 = straight)
    forearm_rot     OpenArm J5's motor, + = pronation (palm down when the forearm points forward)
    wrist_flex      + = the palm bends toward the palm side
    wrist_dev       + = toward the thumb (radial deviation)

The angles mean the same on both sides: a left posture with the same numbers is the mirror image
of the right one. (OpenArm's own left arm has J1-J3 with the opposite sign; they are flipped here.)

World frame (OpenArm's): +x forward (where the arms reach), +z up, the right arm on the -y side,
the left on +y, shoulders (pedestal top) at z = 0.698 m.
"""

from __future__ import annotations

import functools
from pathlib import Path

import mujoco
import numpy as np

from .joints import REPO_ROOT, V1
from .lite_model import KEEP, copy_inertia, simplify_meshes

SIDES = ("right", "left")
MODELS = REPO_ROOT / "sim" / "models"
WRIST_MODEL_PATHS = {
    "right": MODELS / "tendra_hand_v1_wrist.xml",
    "left": MODELS / "tendra_hand_v1_wrist_left.xml",
}

# The wrist point in the hand model's frame (m): middle of the palm's lower edge, where the palm
# meets the forearm. The arm controls this point, so the hand turns about the wrist like a human
# one. (The model origin is at the thumb-side edge of the palm, x spans -79..12 mm, y 12..40 mm.
# It is on the left hand's mirror plane too.)
WRIST_IN_MODEL = np.array([-0.0335, 0.026, -0.030])
WRIST_SITE = "wrist"  # on the palm at WRIST_IN_MODEL; with the side prefix: `right_wrist`

# Our names for OpenArm's joints J1-J4 (the same on both sides) and the joints Tendra adds.
OPENARM_JOINTS = {"shoulder_pitch": 1, "shoulder_roll": 2, "shoulder_yaw": 3, "elbow": 4}
WRIST_JOINTS = ("forearm_rot", "wrist_flex", "wrist_dev")
ARM_JOINTS = (*OPENARM_JOINTS, *WRIST_JOINTS)
# OpenArm's left J1-J3 turn the opposite way to the right ones (J4 is the same); flipped here so
# that equal angles mean mirrored postures.
_OPENARM_LEFT_FLIPPED = (1, 2, 3)

_OPENARM_ELBOW_LINK = "openarm_{side}_link4"  # holds the J5 motor; J5's output is link5
_OPENARM_FOREARM = "openarm_{side}_link5"

# The hand model's frame (X thumb side, Y back of the hand, Z along the fingers) in the elbow
# link's frame: fingers along the forearm (-z when the arm hangs), thumb forward (+x), palm facing
# the body's middle, like a human arm hanging relaxed. Right: a half turn about x. The left hand is
# mirrored (its thumb side is -X), so it needs a half turn about y instead.
HAND_IN_ELBOW_LINK = {
    "right": np.array([0.0, 1.0, 0.0, 0.0]),
    "left": np.array([0.0, 0.0, 1.0, 0.0]),
}  # quaternions wxyz

# Named poses (keyframes), the same numbers for both arms. rest: the arm hangs, 10 deg out to the
# side (straight down, the thumb touches the pedestal: this arm is longer than OpenArm's). ready:
# upper arm down, elbow at 90 deg (forearm forward, level), thumb up; the wrist is then ~0.48 m
# above the floor, 0.23 m forward. table: the pose the grasp scene starts in (and the IK pulls
# toward): ready with the elbow out.
POSES: dict[str, dict[str, float]] = {
    "rest": {"shoulder_roll": 0.17},
    "ready": {"elbow": np.pi / 2},
    # ready with the elbow out 20 deg (reaches the objects beside the hand); the forearm twist takes
    # out the 20 deg roll that the shoulder adds, so the hand is still level: thumb up.
    "table": {"shoulder_roll": 0.35, "elbow": np.pi / 2, "forearm_rot": -0.35},
}

TIMESTEP = 0.002  # s, the hand scene's (OpenArm uses 0.001)


def joint_names(side: str) -> tuple[str, ...]:
    """The 7 arm joints of one side, e.g. `right_shoulder_pitch` (also the actuators' names)."""
    return tuple(f"{side}_{n}" for n in ARM_JOINTS)


def hand_joint_names(side: str) -> tuple[str, ...]:
    """The 16 hand servos of one side in V1 order, e.g. `left_index_pip` (also the actuators)."""
    return tuple(f"{side}_{n}" for n in V1.joint_names)


def _look_at(pos: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    """MuJoCo camera `xyaxes` (camera x and y in the parent frame) for looking at `target`."""
    back = pos - target
    back /= np.linalg.norm(back)  # camera z points backward (it looks along -z)
    x = np.cross(up, back)
    x /= np.linalg.norm(x)
    return np.concatenate([x, np.cross(back, x)])


@functools.lru_cache(maxsize=4)
def full_hand_model(path: Path) -> mujoco.MjModel:
    """The unsimplified hand (on its wrist), only used for its masses and inertias (cached,
    loading is slow)."""
    return mujoco.MjModel.from_xml_path(str(path))


def hand_spec(path: Path, lite: bool = True, keep: float = KEEP) -> mujoco.MjSpec:
    """A hand model (V1 on its wrist) as an MjSpec, ready to be attached (no floor, lights or
    keyframes), with a wrist camera and the `wrist` site on the palm. `lite`: simplified meshes,
    see lite_model."""
    hand = mujoco.MjSpec.from_file(str(path))
    hand.meshdir = str((path.parent / hand.meshdir).resolve())
    if lite:
        simplify_meshes(hand, Path(hand.meshdir), keep)
    for geom in list(hand.worldbody.geoms):  # the floor plane: the scene has a table instead
        hand.delete(geom)
    for light in list(hand.worldbody.lights):
        hand.delete(light)
    for key in list(hand.keys):  # keyframes have hand-only qpos sizes
        hand.delete(key)
    palm = hand.body("palm")
    palm.add_site(name=WRIST_SITE, pos=list(WRIST_IN_MODEL), size=[0.004], group=4)
    # Wrist camera on the palm side near the wrist, looking along the fingers (at what the hand
    # grasps), with the back of the hand up in the image.
    pos = WRIST_IN_MODEL + np.array([0.0, -0.05, 0.0])
    cam = palm.add_camera(name="wrist", fovy=75.0, pos=pos)
    cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
    cam.alt.xyaxes = _look_at(pos, np.array([-0.0335, -0.015, 0.10]), np.array([0, 1.0, 0]))
    return hand


def _trim_openarm() -> mujoco.MjSpec:
    """OpenArm's bimanual pedestal with both arms' base, shoulder and elbow only (joints renamed
    `<side>_shoulder_pitch`.., the left J1-J3 flipped to our sign convention)."""
    from openarm_mujoco.v2 import asset_path

    spec = mujoco.MjSpec.from_file(asset_path("pedestal/openarm_pedestal.xml"))
    keep = {f"openarm_{side}_joint{i}" for side in SIDES for i in range(1, 5)}
    for act in list(spec.actuators):
        if act.target not in keep:
            spec.delete(act)
    for eq in list(spec.equalities):  # the grippers' finger mimics
        spec.delete(eq)
    for key in list(spec.keys):
        spec.delete(key)
    for side in SIDES:
        spec.delete(spec.body(_OPENARM_FOREARM.format(side=side)))
        for name, number in OPENARM_JOINTS.items():
            old, new = f"openarm_{side}_joint{number}", f"{side}_{name}"
            joint = spec.joint(old)
            actuators = [a for a in spec.actuators if a.target == old]
            if side == "left" and number in _OPENARM_LEFT_FLIPPED:
                lo, hi = joint.range
                joint.axis = [-a for a in joint.axis]
                joint.range = [-hi, -lo]
                for act in actuators:
                    lo, hi = act.ctrlrange
                    act.ctrlrange = [-hi, -lo]
            joint.name = new
            for act in actuators:
                act.name, act.target = new, new
    return spec


def _attach_hand(spec: mujoco.MjSpec, side: str, hand: mujoco.MjSpec,
                 bimanual: mujoco.MjSpec) -> None:  # fmt: skip
    """Bolt a hand (on its forearm and wrist) to J5's flange below the side's elbow link."""
    flange = np.array(hand.site("forearm_flange").pos)  # forearm frame = hand model frame
    # J5's place in the elbow link: the deleted forearm link's frame (read from the full model).
    j5 = np.array(bimanual.body(_OPENARM_FOREARM.format(side=side)).pos)
    quat = HAND_IN_ELBOW_LINK[side]
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, quat)
    frame = spec.body(_OPENARM_ELBOW_LINK.format(side=side)).add_frame(
        pos=j5 - rot.reshape(3, 3) @ flange, quat=quat
    )
    spec.attach(hand, frame=frame, prefix=f"{side}_")


def arm_spec(lite: bool = False, keep: float = KEEP, floor: bool = True) -> mujoco.MjSpec:
    """Both arms and hands as an MjSpec (no keyframes), to compile or add a scene to. `floor`: with
    a floor and a light (False: for attaching to a scene that has its own).
    `lite`: simplified hand meshes (faster drawing, same physics after `copy_inertia`)."""
    from openarm_mujoco.v2 import asset_path

    spec = _trim_openarm()
    bimanual = mujoco.MjSpec.from_file(asset_path("openarm_bimanual.xml"))
    spec.modelname = "tendra_arms"
    # The hand scene's settings (also OpenArm's, which <attach> did not carry over): implicit
    # damping, elliptic friction cones and stiff friction hold grasped objects without creep.
    opt = spec.option
    opt.timestep = TIMESTEP
    opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    opt.impratio = 10.0

    for side in SIDES:
        _attach_hand(spec, side, hand_spec(WRIST_MODEL_PATHS[side], lite, keep), bimanual)

    if floor:
        spec.worldbody.add_light(pos=[0.5, -0.8, 2.0], dir=[-0.25, 0.4, -1.0], castshadow=False,
                                 diffuse=[0.6, 0.6, 0.6])  # fmt: skip
        spec.add_material(name="arm_floor", rgba=[0.6, 0.62, 0.65, 1])
        spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1],
                                material="arm_floor")  # fmt: skip
    return spec


def load_arm_model(lite: bool = False, keep: float = KEEP) -> mujoco.MjModel:
    """The compiled arms, with a keyframe per pose in POSES (fingers open), both arms alike."""
    spec = arm_spec(lite, keep)
    model = spec.compile()
    for name, pose in POSES.items():
        qpos = model.qpos0.copy()
        ctrl = np.zeros(model.nu)
        for side in SIDES:
            for joint, q in pose.items():
                qpos[model.joint(f"{side}_{joint}").qposadr[0]] = q
                ctrl[model.actuator(f"{side}_{joint}").id] = q
        spec.add_key(name=name, qpos=qpos, ctrl=ctrl)
    model = spec.compile()
    if lite:
        for side in SIDES:
            copy_inertia(model, full_hand_model(WRIST_MODEL_PATHS[side]), prefix=f"{side}_")
    if model.nu != len(SIDES) * (len(ARM_JOINTS) + V1.num_joints):
        raise RuntimeError(f"arm model has {model.nu} actuators, expected both arms + hands")
    return model


def ik_model(side: str, frame_pos=(0.0, 0.0, 0.0), frame_quat=(1.0, 0.0, 0.0, 0.0)) -> mujoco.MjModel:
    """A bare kinematic copy of one arm, from the pedestal to the palm, for `ArmIK`: the same
    bodies, joints and transforms as the full model, but no fingers, servos, tendons, meshes or
    contacts, so a kinematics call costs a few microseconds instead of a few hundred.

    `frame_pos` / `frame_quat`: where the arm model's origin sits in the world of the scene it is
    for, so poses come out in the scene's world frame.
    """
    from openarm_mujoco.v2 import asset_path

    spec = _trim_openarm()
    other = "left" if side == "right" else "right"
    spec.delete(spec.body(f"openarm_{other}_base_link"))
    for act in list(spec.actuators):
        spec.delete(act)
    for geom in list(spec.geoms):
        spec.delete(geom)
    for mesh in list(spec.meshes):
        spec.delete(mesh)

    path = WRIST_MODEL_PATHS[side]
    hand = mujoco.MjSpec.from_file(str(path))
    hand.meshdir = str((path.parent / hand.meshdir).resolve())
    for collection in (hand.tendons, hand.actuators, hand.equalities, hand.keys, hand.excludes):
        for item in list(collection):
            hand.delete(item)
    keep = {"forearm", "wrist_gimbal", "palm"}
    for parent in (hand.body("forearm"), hand.body("wrist_gimbal"), hand.body("palm")):
        for child in list(parent.bodies):
            if child.name not in keep:
                hand.delete(child)
    for geom in list(hand.geoms):
        hand.delete(geom)
    for mesh in list(hand.meshes):
        hand.delete(mesh)
    for name in ("forearm", "palm"):  # bodies that lost their meshes still need some mass
        hand.body(name).add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.005], mass=0.01,
                                 contype=0, conaffinity=0)  # fmt: skip
    hand.body("palm").add_site(name=WRIST_SITE, pos=list(WRIST_IN_MODEL), size=[0.004])
    for options in (spec.option, hand.option):  # the same everywhere, so attaching doesn't warn
        options.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        options.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        options.impratio = 10.0
    _attach_hand(spec, side, hand, mujoco.MjSpec.from_file(asset_path("openarm_bimanual.xml")))
    world = mujoco.MjSpec()
    world.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    world.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    world.option.impratio = 10.0
    frame = world.worldbody.add_frame(pos=list(frame_pos), quat=list(frame_quat))
    world.attach(spec, frame=frame, prefix="")
    model = world.compile()
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)  # kinematics only
    return model


def arm_joint_ids(model: mujoco.MjModel, side: str = "right") -> tuple[np.ndarray, np.ndarray]:
    """(qpos addresses, actuator ids) of one side's 7 arm joints, in ARM_JOINTS order."""
    names = joint_names(side)
    qadr = np.array([model.joint(n).qposadr[0] for n in names])
    act = np.array([model.actuator(n).id for n in names])
    return qadr, act


def openarm_package_path() -> Path:
    """Where the installed OpenArm model lives (for docs and error messages)."""
    from openarm_mujoco.v2 import asset_path

    return Path(asset_path("openarm_bimanual.xml")).parent


def _rot_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """World-frame rotation vector that turns `current` into `target` (both 3x3), small-angle."""
    m = current @ target.T  # sum of cross(current_i, target_i) = the antisymmetric part of this
    return 0.5 * np.array([m[1, 2] - m[2, 1], m[2, 0] - m[0, 2], m[0, 1] - m[1, 0]])


class ArmIK:
    """Damped-least-squares inverse kinematics: from a wrist pose to joint targets.

    The arm is a stand-in for whatever arm the hand will sit on (OpenArm's shoulder and elbow
    now, the owner's own design later), so nothing here is OpenArm-specific: give it a kinematic
    model (`ik_model`: only the arm's chain), the names of the arm's joints and the site to
    control. The policy and the scene only ever ask "put the wrist here"; the arm's joint angles
    stay inside this class and the position servos.

    It works on its own tiny model and its own data, so it never disturbs the simulation, and
    returns joint targets for the position actuators. The redundant 7th joint is resolved by
    pulling toward `rest` (the elbow stays down, like a human's).
    """

    def __init__(self, model: mujoco.MjModel, joints: tuple[str, ...], site: str,
                 rest: np.ndarray | None = None,
                 rot_weight: float = 0.5, damping: float = 0.01, rest_gain: float = 0.2,
                 max_step: float = 0.3) -> None:  # fmt: skip
        self.model, self.site = model, model.site(site).id
        self.joints = np.array([model.joint(n).id for n in joints])
        self.qadr = model.jnt_qposadr[self.joints]
        self.dofs = model.jnt_dofadr[self.joints]
        self.lo, self.hi = model.jnt_range[self.joints].T
        self.rest = np.zeros(len(joints)) if rest is None else np.asarray(rest, dtype=float)
        self.rot_weight, self.damping, self.rest_gain, self.max_step = (
            rot_weight, damping, rest_gain, max_step)  # fmt: skip
        self._lambda = damping**2 * np.eye(6)
        self._data = mujoco.MjData(model)
        self._jacp = np.zeros((3, model.nv))
        self._jacr = np.zeros((3, model.nv))
        self._eye = np.eye(len(joints))

    def pose(self, q_arm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(position, rotation matrix) of the controlled site at these arm joint angles."""
        d = self._data
        d.qpos[self.qadr] = q_arm
        mujoco.mj_kinematics(self.model, d)
        return d.site_xpos[self.site].copy(), d.site_xmat[self.site].reshape(3, 3).copy()

    def solve(self, q_arm: np.ndarray, pos: np.ndarray, rot: np.ndarray, iters: int = 12,
              tol: float = 3e-4) -> tuple[np.ndarray, float]:  # fmt: skip
        """Arm joint targets for the wrist pose (`pos` m, `rot` 3x3 world), starting from the arm
        angles `q_arm`. Returns (targets inside the joint limits, the remaining error: metres +
        rad x rot_weight, so ~0 if the pose is reachable)."""
        m, d = self.model, self._data
        q = np.asarray(q_arm, dtype=float).copy()
        err_norm = np.inf
        for _ in range(iters):
            d.qpos[self.qadr] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            p = d.site_xpos[self.site]
            r = d.site_xmat[self.site].reshape(3, 3)
            err = np.concatenate([pos - p, self.rot_weight * _rot_error(r, rot)])
            err_norm = float(np.linalg.norm(err))
            if err_norm < tol:
                break
            mujoco.mj_jacSite(m, d, self._jacp, self._jacr, self.site)
            jac = np.vstack([self._jacp[:, self.dofs], self.rot_weight * self._jacr[:, self.dofs]])
            pinv = jac.T @ np.linalg.inv(jac @ jac.T + self._lambda)
            dq = pinv @ err + (self._eye - pinv @ jac) @ (self.rest_gain * (self.rest - q))
            step = np.abs(dq).max()
            if step > self.max_step:
                dq *= self.max_step / step
            q = np.clip(q + dq, self.lo, self.hi)
        return q, err_norm
