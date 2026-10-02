"""The Tendra arm: OpenArm's shoulder and elbow, then Tendra's forearm, wrist and V1 hand (MuJoCo).

The first body is two arms with Tendra hands on a fixed pole (CLAUDE.md). The shoulder and elbow
come from OpenArm v2 (github.com/enactic/openarm, Apache-2.0, the `openarm-mujoco` package): its
base, J1-J3 (shoulder), J4 (elbow) and the J5 motor at the bottom of the elbow link. Everything from
J5's flange down is Tendra's: the forearm (turned by J5), a 2-way wrist and the hand, built by
sim/convert_v1_wrist.py. For now one right arm on OpenArm's pedestal; a left arm needs a mirrored
hand.

    model = load_arm_model()                 # or arm_spec() to add a scene around it
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("ready").id)

Joints (rad). The 7 arm joints, each driven by a position actuator of the same name, then the
hand's 16 servos (V1 order, `tendra.joints.V1`):

    shoulder_pitch  OpenArm J1, + = arm forward (flexion)
    shoulder_roll   OpenArm J2, + = arm out to the side (abduction)
    shoulder_yaw    OpenArm J3, + = upper arm turns outward (external rotation)
    elbow           OpenArm J4, + = bend (0 = straight)
    forearm_rot     OpenArm J5's motor, + = pronation (palm down when the forearm points forward)
    wrist_flex      + = the palm bends toward the palm side
    wrist_dev       + = toward the thumb (radial deviation)

World frame (OpenArm's): +x forward (where the arm reaches), +z up, the right arm on the -y side,
shoulders (pedestal top) at z = 0.698 m. OpenArm's tables: top at z = 0.40 m, centre x = 0.47 m.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from .joints import REPO_ROOT, V1
from .lite_model import KEEP, copy_inertia
from .scene import WRIST_IN_MODEL, full_hand_model, hand_spec

WRIST_MODEL_PATH = REPO_ROOT / "sim" / "models" / "tendra_hand_v1_wrist.xml"

# Our names for OpenArm's right-arm joints (its actuators are renamed to match).
OPENARM_JOINTS = {
    "shoulder_pitch": "openarm_right_joint1",
    "shoulder_roll": "openarm_right_joint2",
    "shoulder_yaw": "openarm_right_joint3",
    "elbow": "openarm_right_joint4",
}
WRIST_JOINTS = ("forearm_rot", "wrist_flex", "wrist_dev")
ARM_JOINTS = (*OPENARM_JOINTS, *WRIST_JOINTS)

# OpenArm parts that Tendra replaces: the left arm (until there is a left hand) and, on the right,
# everything below J5 (OpenArm's forearm, wrist and gripper).
_OPENARM_LEFT = "openarm_left_base_link"
_OPENARM_ELBOW_LINK = "openarm_right_link4"  # holds the J5 motor; J5's output is link5
_OPENARM_FOREARM = "openarm_right_link5"

# The hand model's frame (X thumb side, Y back of the hand, Z along the fingers) in the elbow
# link's frame: fingers along the forearm (-z when the arm hangs), thumb forward (+x), palm facing
# the body's middle (+y), like a human arm hanging relaxed. A half turn about x.
HAND_IN_ELBOW_LINK = np.array([0.0, 1.0, 0.0, 0.0])  # quaternion wxyz

# Named poses (keyframes). rest: the arm hangs, 10 deg out to the side (straight down, the thumb
# touches the pedestal: this arm is longer than OpenArm's). ready: upper arm down, elbow at 90 deg
# (forearm forward, level), thumb up; the wrist is then ~0.48 m above the floor, 0.23 m forward.
# table: the pose the grasp scene starts in (and the IK pulls toward): ready with the elbow out.
POSES: dict[str, dict[str, float]] = {
    "rest": {"shoulder_roll": 0.17},
    "ready": {"elbow": np.pi / 2},
    # ready with the elbow out 20 deg (reaches the objects beside the hand); the forearm twist takes
    # out the 20 deg roll that the shoulder adds, so the hand is still level: thumb up.
    "table": {"shoulder_roll": 0.35, "elbow": np.pi / 2, "forearm_rot": -0.35},
}

TIMESTEP = 0.002  # s, the hand scene's (OpenArm uses 0.001)


def _trim_openarm() -> mujoco.MjSpec:
    """OpenArm's pedestal with the right arm's base, shoulder and elbow only."""
    from openarm_mujoco.v2 import asset_path

    spec = mujoco.MjSpec.from_file(asset_path("pedestal/openarm_pedestal.xml"))
    keep = {f"openarm_right_joint{i}" for i in range(1, 5)}
    for act in list(spec.actuators):
        if act.target not in keep:
            spec.delete(act)
    for eq in list(spec.equalities):  # the grippers' finger mimics
        spec.delete(eq)
    for key in list(spec.keys):
        spec.delete(key)
    spec.delete(spec.body(_OPENARM_LEFT))
    spec.delete(spec.body(_OPENARM_FOREARM))
    for name, joint in OPENARM_JOINTS.items():
        spec.joint(joint).name = name
        for act in spec.actuators:
            if act.target == joint:
                act.name, act.target = name, name
    return spec


WRIST_SITE = "wrist"  # on the palm, at WRIST_IN_MODEL: the point the arm controls (like hand_root)


def arm_spec(lite: bool = False, keep: float = KEEP, floor: bool = True) -> mujoco.MjSpec:
    """The arm and hand as an MjSpec (no keyframes), to compile or add a scene to. `floor`: with a
    floor and a light (False: for attaching to a scene that has its own).
    `lite`: simplified hand meshes (faster drawing, same physics after `copy_inertia`)."""
    from openarm_mujoco.v2 import asset_path

    spec = _trim_openarm()
    # Read J5's place before it went: the deleted forearm's frame in the elbow link.
    j5 = np.array(mujoco.MjSpec.from_file(asset_path("openarm_bimanual.xml"))
                  .body(_OPENARM_FOREARM).pos)  # fmt: skip
    spec.modelname = "tendra_arm"
    # The hand scene's settings (also OpenArm's, which <attach> did not carry over): implicit
    # damping, elliptic friction cones and stiff friction hold grasped objects without creep.
    opt = spec.option
    opt.timestep = TIMESTEP
    opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    opt.impratio = 10.0

    hand = hand_spec(WRIST_MODEL_PATH, lite, keep)
    hand.body("palm").add_site(name=WRIST_SITE, pos=list(WRIST_IN_MODEL), size=[0.004], group=4)
    flange = np.array(hand.site("forearm_flange").pos)  # forearm frame = hand model frame
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, HAND_IN_ELBOW_LINK)
    frame = spec.body(_OPENARM_ELBOW_LINK).add_frame(
        pos=j5 - rot.reshape(3, 3) @ flange, quat=HAND_IN_ELBOW_LINK
    )
    spec.attach(hand, frame=frame, prefix="")

    if floor:
        spec.worldbody.add_light(pos=[0.5, -0.8, 2.0], dir=[-0.25, 0.4, -1.0], castshadow=False,
                                 diffuse=[0.6, 0.6, 0.6])  # fmt: skip
        spec.add_material(name="arm_floor", rgba=[0.6, 0.62, 0.65, 1])
        spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1],
                                material="arm_floor")  # fmt: skip
    return spec


def load_arm_model(lite: bool = False, keep: float = KEEP) -> mujoco.MjModel:
    """The compiled arm, with a keyframe per pose in POSES (fingers open)."""
    spec = arm_spec(lite, keep)
    model = spec.compile()
    for name, pose in POSES.items():
        qpos = model.qpos0.copy()
        ctrl = np.zeros(model.nu)
        for joint, q in pose.items():
            qpos[model.joint(joint).qposadr[0]] = q
            ctrl[model.actuator(joint).id] = q
        spec.add_key(name=name, qpos=qpos, ctrl=ctrl)
    model = spec.compile()
    if lite:
        copy_inertia(model, full_hand_model(WRIST_MODEL_PATH))
    if model.nu != len(ARM_JOINTS) + V1.num_joints:
        raise RuntimeError(f"arm model has {model.nu} actuators, expected arm + hand")
    return model


def arm_joint_ids(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    """(qpos addresses, actuator ids) of the 7 arm joints, in ARM_JOINTS order."""
    qadr = np.array([model.joint(n).qposadr[0] for n in ARM_JOINTS])
    act = np.array([model.actuator(n).id for n in ARM_JOINTS])
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
    now, the owner's own design later), so nothing here is OpenArm-specific: give it a model, the
    names of the arm's joints and the site to control. The policy and the scene only ever ask
    "put the wrist here"; the arm's joint angles stay inside this class and the position servos.

    `solve` works on a private copy of the state (kinematics only, ~0.1 ms per iteration), so it
    never disturbs the simulation, and returns joint targets for the position actuators. The
    redundant 7th joint is resolved by pulling toward `rest` (the elbow stays down, like a human's).
    """

    def __init__(self, model: mujoco.MjModel, joints: tuple[str, ...] = ARM_JOINTS,
                 site: str = WRIST_SITE, rest: np.ndarray | None = None,
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

    def pose(self, qpos: np.ndarray, q_arm: np.ndarray | None = None) -> tuple[np.ndarray,
                                                                             np.ndarray]:  # fmt: skip
        """(position, rotation matrix) of the controlled site, optionally at other arm angles."""
        d = self._data
        d.qpos[:] = qpos
        if q_arm is not None:
            d.qpos[self.qadr] = q_arm
        mujoco.mj_kinematics(self.model, d)
        return d.site_xpos[self.site].copy(), d.site_xmat[self.site].reshape(3, 3).copy()

    def solve(self, qpos: np.ndarray, pos: np.ndarray, rot: np.ndarray, iters: int = 12,
              tol: float = 3e-4) -> tuple[np.ndarray, float]:  # fmt: skip
        """Arm joint targets for the wrist pose (`pos` m, `rot` 3x3 world), starting from the arm
        angles in `qpos` (the full state). Returns (targets inside the joint limits, the remaining
        error: metres + rad x rot_weight, so ~0 if the pose is reachable)."""
        m, d = self.model, self._data
        d.qpos[:] = qpos
        q = d.qpos[self.qadr].copy()
        n = len(q)
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
            dq = pinv @ err + (np.eye(n) - pinv @ jac) @ (self.rest_gain * (self.rest - q))
            step = np.abs(dq).max()
            if step > self.max_step:
                dq *= self.max_step / step
            q = np.clip(q + dq, self.lo, self.hi)
        return q, err_norm
