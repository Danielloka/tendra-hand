"""A grasping scene: the Tendra robot (two arms, two V1 hands) at a table with an object to pick up.

The robot is the body of CLAUDE.md: OpenArm's pedestal with both shoulders and elbows, then
Tendra's forearms, 2-way wrists and V1 hands (`tendra.arm`). It stands behind the table (+y) and
faces the operator's side (-y). One hand is *active* in an episode (right or left, chosen at
`reset`): the object is on its side of the table and it is the one that picks it up; the other
arm rests. The robot's action is "wrist pose + 16 finger joint targets" for the active hand, for a
learned policy and for a person alike.

The wrist target is where the arm should put the wrist; inverse kinematics (`tendra.arm.ArmIK`)
turns it into joint targets for the arm's position servos, so the wrist moves like a real arm
would: joint limits, speed, and poses it simply cannot reach. (The arm is a stand-in: the owner
will build their own, so what is trained here is the *hand*.)

World frame: table top at z = 0, units m, rad, quaternions wxyz. The robot's symmetry plane is
x = `arm_base[0]` (0): the right hand works at x < 0, the left at x > 0, mirror images of each
other. `canon_*` map the left side onto the right one so that one policy can drive both hands.

    scene = GraspScene()
    scene.reset(np.random.default_rng(0), obj="cylinder", side="right")
    scene.set_wrist_target(pos, quat)       # the active hand's wrist
    scene.set_finger_targets(q)             # its 16 joint targets (servos), V1 order
    scene.step(0.02)
    scene.lifted()                          # success check
"""

import math
from dataclasses import dataclass

import mujoco
import numpy as np

from .arm import (
    ARM_JOINTS,
    POSES,
    SIDES,
    WRIST_MODEL_PATHS,
    WRIST_SITE,
    ArmIK,
    _look_at,
    arm_spec,
    full_hand_model,
    hand_joint_names,
    ik_model,
    joint_names,
)
from .joints import V1
from .lite_model import KEEP, copy_inertia

# Bodies of the five fingers (not servos or spools), without the side prefix: `thumb_base`, ...
FINGERS = ("thumb_", "index_", "middle_", "ring_", "little_")
TIP_FINGERS = ("thumb", "index", "middle", "ring", "little")

# The arm model's shoulder height above its own origin (OpenArm's pedestal top, m).
ARM_SHOULDER_Z = 0.698

# Objects, sized for this hand (knuckle pitch 19 mm, index ~86 mm, palm ~90 mm wide).
# kind: (MuJoCo geom type, size, mass kg, rgba)
OBJECTS: dict[str, tuple[mujoco.mjtGeom, tuple[float, ...], float, tuple[float, ...]]] = {
    "cylinder": (mujoco.mjtGeom.mjGEOM_CYLINDER, (0.022, 0.045), 0.05, (0.85, 0.25, 0.2, 1)),
    "cube": (mujoco.mjtGeom.mjGEOM_BOX, (0.02, 0.02, 0.02), 0.04, (0.2, 0.45, 0.85, 1)),
    "ball": (mujoco.mjtGeom.mjGEOM_SPHERE, (0.028,), 0.04, (0.95, 0.75, 0.15, 1)),
}


@dataclass
class SceneConfig:
    objects: tuple[str, ...] = ("cylinder", "cube", "ball")  # object kinds available
    lite: bool = True  # simplified meshes (much faster to draw, same physics)
    lite_keep: float = KEEP  # fraction of triangles kept by the simplification
    table_half: tuple[float, float] = (0.30, 0.25)  # table top half size x, y (m)
    # The robot: the arm model's frame origin on the floor. It faces -y (toward the operator's
    # side of the table), its symmetry plane is x = arm_base[0]; the pedestal stands behind the
    # table (y >= 0.33 clears the table's edge at 0.25). `arm_shoulder_height` is above the table
    # top (OpenArm's own tables: 0.30).
    arm_base: tuple[float, float] = (0.0, 0.35)
    arm_shoulder_height: float = 0.30
    # Objects spawn uniformly in this box (x, y, world m) for the RIGHT hand, and mirrored for the
    # left one: where the hand can reach with a level forearm. `test_scene` checks that the
    # scripted grasp works over the whole box. At home the right hand is at x ~ -0.23.
    spawn_lo: tuple[float, float] = (-0.14, -0.07)
    spawn_hi: tuple[float, float] = (0.0, 0.09)
    arm_ik_dt: float = 0.025  # s between inverse-kinematics updates while stepping
    object_friction: tuple[float, float, float] = (1.0, 0.01, 0.0002)  # slide, spin, roll
    object_condim: int = 4  # 4 = with torsional friction (holds an object from turning)
    impratio: float = 10.0  # >1 makes friction harder than normal force: less slip
    settle: float = 0.2  # seconds simulated at reset so the object comes to rest
    # The resting arm falls asleep (MuJoCo's sleeping: no physics, tendons or collisions for it
    # while nothing touches it), which makes a step ~2x cheaper. Only for one-hand episodes: an
    # asleep arm ignores its wrist target. Set False for tasks that use both hands.
    sleep_idle: bool = True
    # Contact sensors (`contact_sensor_name`), for the GPU training env, which has no contact loop.
    contact_sensors: bool = False
    # The "view" camera: from the operator's side, both arms and the table in view.
    view_fovy: float = 60.0
    view_pos: tuple[float, float, float] = (0.0, -0.62, 0.55)
    view_lookat: tuple[float, float, float] = (0.0, 0.12, 0.15)


# Hand parts with a contact sensor each (per side and object): (part, MuJoCo object type, body
# without the side prefix). A finger is the subtree of its base body; the palm is the palm body
# alone (its subtree holds the fingers); "hand" is everything from the forearm down (the CPU env's
# "in the hand", and its "other" parts = hand minus the others).
CONTACT_PARTS: tuple[tuple[str, mujoco.mjtObj, str], ...] = (
    *((f, mujoco.mjtObj.mjOBJ_XBODY, f"{f}_base") for f in TIP_FINGERS),
    ("palm", mujoco.mjtObj.mjOBJ_BODY, "palm"),
    ("hand", mujoco.mjtObj.mjOBJ_XBODY, "forearm"),
)
_FOUND, _FORCE, _POS = (1 << int(mujoco.mjtConDataField.mjCONDATA_FOUND),
                        1 << int(mujoco.mjtConDataField.mjCONDATA_FORCE),
                        1 << int(mujoco.mjtConDataField.mjCONDATA_POS))  # fmt: skip
_NETFORCE = 3  # contact sensor reduce mode: one net contact (force-weighted position, world frame)


def contact_sensor_name(side: str, part: str, obj: str) -> str:
    """Hand part (`CONTACT_PARTS`) <-> object: found, net force (3, world), position (3)."""
    return f"{side}_touch_{part}_{obj}"


def table_sensor_name(what: str) -> str:
    """Table <-> an object kind, or <-> a side's hand ("right" / "left"): found (1 value)."""
    return f"table_{what}"


def _add_contact_sensors(scene: mujoco.MjSpec, objects: tuple[str, ...]) -> None:
    geom = mujoco.mjtObj.mjOBJ_GEOM
    contact = mujoco.mjtSensor.mjSENS_CONTACT
    for obj in objects:
        for side in SIDES:
            for part, objtype, body in CONTACT_PARTS:
                scene.add_sensor(name=contact_sensor_name(side, part, obj), type=contact,
                                 objtype=objtype, objname=f"{side}_{body}", reftype=geom,
                                 refname=obj, intprm=[_FOUND | _FORCE | _POS, _NETFORCE, 1])  # fmt: skip
        scene.add_sensor(name=table_sensor_name(obj), type=contact, objtype=geom, objname=obj,
                         reftype=geom, refname="table", intprm=[_FOUND, 0, 1])  # fmt: skip
    for side in SIDES:
        scene.add_sensor(name=table_sensor_name(side), type=contact,
                         objtype=mujoco.mjtObj.mjOBJ_XBODY, objname=f"{side}_forearm",
                         reftype=geom, refname="table", intprm=[_FOUND, 0, 1])  # fmt: skip


def mat_to_quat(mat: np.ndarray) -> np.ndarray:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(mat, dtype=float).ravel())
    return quat


def quat_to_mat(quat: np.ndarray) -> np.ndarray:
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, np.asarray(quat, dtype=float))
    return mat.reshape(3, 3)


@dataclass
class ArmSide:
    """Everything the scene needs to know about one arm and its hand (ids in the compiled model)."""

    name: str
    root: int  # the forearm body: the hand's root
    hand_bodies: set[int]  # every body from the forearm down
    act: np.ndarray  # the hand's 16 servo actuators, V1 order
    qadr: np.ndarray  # their joints' qpos addresses (the 16 servo joints)
    arm_qadr: np.ndarray  # the arm's 7 joints: qpos addresses
    arm_dofs: np.ndarray  # and dof addresses
    arm_act: np.ndarray  # and actuators
    tree: int  # its kinematic tree (for sleeping)
    ik: ArmIK
    site: int  # the wrist site in the scene's model
    mocap: int  # index of this side's wrist target in the mocap arrays
    tips: np.ndarray  # fingertip sites: thumb, index, middle, ring, little
    ready: np.ndarray  # arm joint angles at the start pose
    home_pos: np.ndarray  # wrist position at the start pose (world, m)
    home_quat: np.ndarray  # and orientation (wxyz)


class GraspScene:
    """The robot at a table with objects (see module docs)."""

    CAMERAS = ("view", "right_wrist", "left_wrist")

    def __init__(self, config: SceneConfig | None = None) -> None:
        self.config = config = config or SceneConfig()
        for kind in config.objects:
            if kind not in OBJECTS:
                raise ValueError(f"unknown object {kind!r}, expected one of {sorted(OBJECTS)}")
        self.spec = V1
        self.model = self._build()
        self.data = mujoco.MjData(self.model)
        m = self.model

        self._table = m.geom("table").id
        self._obj_body = {k: m.body(k).id for k in config.objects}
        self._obj_qpos = {k: m.jnt_qposadr[m.body_jntadr[b]] for k, b in self._obj_body.items()}
        self._obj_dof = {k: m.jnt_dofadr[m.body_jntadr[b]] for k, b in self._obj_body.items()}
        self._obj_geom = {k: m.body_geomadr[b] for k, b in self._obj_body.items()}
        self._obj_contype = {k: (m.geom_contype[g], m.geom_conaffinity[g])
                             for k, g in self._obj_geom.items()}  # fmt: skip

        self.mirror_x = config.arm_base[0]  # the robot's symmetry plane
        self.sides = {side: self._init_side(side, i) for i, side in enumerate(SIDES)}
        self._side = "right"
        self.ik_error = 0.0  # last inverse-kinematics residual of the active arm (m + 0.5 x rad)
        self._active = config.objects[0]
        self._rest_z = 0.0
        self.reset(np.random.default_rng(0))

    # ----- building the model -----

    def _init_side(self, side: str, mocap: int) -> ArmSide:
        m = self.model
        root = m.body(f"{side}_forearm").id
        hand_bodies = set()
        for b in range(m.nbody):
            body = b
            while body:
                if body == root:
                    hand_bodies.add(b)
                    break
                body = m.body_parentid[body]
        arm_joints = joint_names(side)
        ready = np.array([POSES["table"].get(n, 0.0) for n in ARM_JOINTS])
        # The IK works on a bare kinematic copy of the arm, placed like the arm in this scene.
        ik = ArmIK(ik_model(side, *self._arm_frame()), arm_joints, f"{side}_{WRIST_SITE}", rest=ready)
        pos, rot = ik.pose(ready)
        return ArmSide(
            name=side,
            root=root,
            hand_bodies=hand_bodies,
            act=np.array([m.actuator(n).id for n in hand_joint_names(side)]),
            qadr=np.array([m.jnt_qposadr[m.joint(n).id] for n in hand_joint_names(side)]),
            arm_qadr=np.array([m.joint(n).qposadr[0] for n in arm_joints]),
            arm_dofs=np.array([m.jnt_dofadr[m.joint(n).id] for n in arm_joints]),
            arm_act=np.array([m.actuator(n).id for n in arm_joints]),
            tree=int(m.body_treeid[root]),
            ik=ik,
            site=m.site(f"{side}_{WRIST_SITE}").id,
            mocap=m.body_mocapid[m.body(f"wrist_target_{side}").id],
            tips=np.array([m.site(f"{side}_{f}_tip").id for f in TIP_FINGERS]),
            ready=ready,
            home_pos=pos,
            home_quat=mat_to_quat(rot),
        )

    def _arm_frame(self) -> tuple[list[float], list[float]]:
        """Where the arm model's origin is in the world (position, quaternion): on the floor behind
        the table, turned so the arms' +x (forward) points along world -y."""
        cfg = self.config
        base_z = cfg.arm_shoulder_height - ARM_SHOULDER_Z
        return [*cfg.arm_base, base_z], [math.cos(math.pi / 4), 0, 0, -math.sin(math.pi / 4)]

    def _build(self) -> mujoco.MjModel:
        cfg = self.config
        scene = mujoco.MjSpec()
        scene.modelname = "tendra_grasp_scene"
        opt = scene.option
        opt.timestep = 0.002
        opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        opt.impratio = cfg.impratio
        scene.visual.headlight.ambient = [0.35, 0.35, 0.35]
        scene.visual.headlight.diffuse = [0.5, 0.5, 0.5]
        scene.visual.global_.offwidth = 640
        scene.visual.global_.offheight = 640
        scene.stat.center = [0.0, 0.0, 0.12]
        scene.stat.extent = 0.5

        world = scene.worldbody
        world.add_light(pos=[0.3, -0.5, 1.0], dir=[-0.3, 0.5, -1.0], castshadow=False,
                        diffuse=[0.5, 0.5, 0.5])  # fmt: skip
        scene.add_texture(name="scene_sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                          builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT, rgb1=[0.93, 0.95, 0.98],
                          rgb2=[0.55, 0.6, 0.68], width=64, height=64)  # fmt: skip
        scene.add_material(name="scene_table", rgba=[0.78, 0.66, 0.52, 1])
        scene.add_material(name="scene_floor", rgba=[0.55, 0.57, 0.6, 1])
        hx, hy = cfg.table_half
        world.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX, size=[hx, hy, 0.02],
                       pos=[0, 0, -0.02], material="scene_table", friction=[0.8, 0.01, 0.0001])  # fmt: skip
        base_z = cfg.arm_shoulder_height - ARM_SHOULDER_Z  # the arm model's origin, on the floor
        world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1],
                       pos=[0, 0, base_z - 0.004], material="scene_floor", contype=0, conaffinity=0)  # fmt: skip

        cam = world.add_camera(name="view", fovy=cfg.view_fovy, pos=list(cfg.view_pos))
        cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
        cam.alt.xyaxes = _look_at(np.array(cfg.view_pos), np.array(cfg.view_lookat),
                                  np.array([0, 0, 1.0]))  # fmt: skip

        # One wrist target per hand (mocap bodies: where the IK should put the wrist).
        for side in SIDES:
            target = world.add_body(name=f"wrist_target_{side}", mocap=True, pos=[0, 0, 0.1])
            target.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.004],
                            rgba=[0.1, 0.6, 1, 0.5], contype=0, conaffinity=0, group=2)  # fmt: skip

        # The robot: OpenArm's pedestal, shoulders and elbows, the Tendra forearms, wrists and
        # hands. Its +x (forward) points along world -y, toward the operator's side.
        frame_pos, frame_quat = self._arm_frame()
        frame = world.add_frame(pos=frame_pos, quat=frame_quat)
        scene.attach(arm_spec(cfg.lite, cfg.lite_keep, floor=False), frame=frame, prefix="")

        # Objects, each with a free joint. Parked ones float (gravity compensated) far away.
        for i, kind in enumerate(cfg.objects):
            gtype, size, mass, rgba = OBJECTS[kind]
            body = world.add_body(name=kind, pos=self._park_pos(i), gravcomp=1.0)
            body.add_freejoint(name=kind)
            body.add_geom(name=kind, type=gtype, size=list(size), mass=mass, rgba=list(rgba),
                          friction=list(cfg.object_friction), condim=cfg.object_condim)  # fmt: skip
        if cfg.contact_sensors:
            _add_contact_sensors(scene, cfg.objects)

        model = scene.compile()
        if cfg.lite:
            for side in SIDES:
                copy_inertia(model, full_hand_model(WRIST_MODEL_PATHS[side]), prefix=f"{side}_")
        model.light_castshadow[:] = 0
        model.mat_reflectance[:] = 0
        # A real arm carries its own weight (gravity compensation in its controller). Without it
        # the position servos sag 5 mm under the hand, and the wrist would be off by that. The
        # fingers and objects are not compensated: grip and lift load the hand as usual.
        finger_prefixes = tuple(f"{side}_{f}" for side in SIDES for f in FINGERS)
        for b in range(1, model.nbody):
            name = model.body(b).name
            if (name not in OBJECTS and not name.startswith("wrist_target")
                    and not name.startswith(finger_prefixes)):  # fmt: skip
                model.body_gravcomp[b] = 1.0
        model.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_SLEEP)
        mujoco.mj_setConst(model, mujoco.MjData(model))
        strands = len(SIDES) * (2 * V1.num_joints + len(V1.couplings))  # servo loops + DIP bars
        nu = len(SIDES) * (V1.num_joints + len(ARM_JOINTS))
        if model.nu != nu or model.ntendon != strands or model.neq < len(SIDES) * len(V1.couplings):
            raise RuntimeError("scene model lost arm or hand actuators, tendons or DIP couplings")
        return model

    @staticmethod
    def _park_pos(i: int) -> list[float]:
        return [1.0 + 0.2 * i, 1.5, 0.3]  # far behind the table, out of every camera's view

    # ----- the active side and the mirror -----

    @property
    def side(self) -> str:
        """The hand that picks up the object in this episode: "right" or "left"."""
        return self._side

    @property
    def arm(self) -> ArmSide:
        return self.sides[self._side]

    @property
    def flip(self) -> bool:
        """True for the left hand: its world is the mirror image of the right one's."""
        return self._side == "left"

    # The mirror, both ways (it is its own inverse): the left side's world quantities as the right
    # hand would have them (positions mirrored in the plane x = mirror_x, rotations as M R M_hand,
    # M = flip x in the world, M_hand = flip x in the hand model's frame, which is mirrored too).
    # For the right hand they do nothing. The policy sees and acts in this canonical right-hand view.
    def canon_pos(self, p: np.ndarray) -> np.ndarray:
        p = np.asarray(p, dtype=float)
        return np.array([2 * self.mirror_x - p[0], p[1], p[2]]) if self.flip else p.copy()

    def canon_vec(self, v: np.ndarray) -> np.ndarray:
        """A polar vector (velocity, offset)."""
        v = np.asarray(v, dtype=float)
        return np.array([-v[0], v[1], v[2]]) if self.flip else v.copy()

    def canon_axial(self, w: np.ndarray) -> np.ndarray:
        """An axial vector (angular velocity, turn rate): mirrored axial vectors flip the other
        two components."""
        w = np.asarray(w, dtype=float)
        return np.array([w[0], -w[1], -w[2]]) if self.flip else w.copy()

    def canon_rot(self, r: np.ndarray) -> np.ndarray:
        r = np.asarray(r, dtype=float)
        return np.diag([-1.0, 1.0, 1.0]) @ r @ np.diag([-1.0, 1.0, 1.0]) if self.flip else r.copy()

    # Aliases for the active side, so callers don't need `scene.arm.x` everywhere.
    @property
    def _act(self) -> np.ndarray:
        return self.arm.act

    @property
    def _qadr(self) -> np.ndarray:
        return self.arm.qadr

    @property
    def _hand_bodies(self) -> set[int]:
        return self.arm.hand_bodies

    @property
    def _ik(self) -> ArmIK:
        return self.arm.ik

    @property
    def home_pos(self) -> np.ndarray:
        return self.arm.home_pos

    @property
    def home_quat(self) -> np.ndarray:
        return self.arm.home_quat

    @property
    def arm_ready(self) -> np.ndarray:
        return self.arm.ready

    # ----- episode -----

    def reset(self, rng: np.random.Generator | None = None, obj: str | None = None,
              side: str | None = None) -> None:  # fmt: skip
        """Both arms at home (fingers open), one object at a random spot on the active hand's side
        of the table (`side`: "right" or "left", random if None), the others parked."""
        rng = rng or np.random.default_rng()
        m, d, cfg = self.model, self.data, self.config
        if obj is None:
            obj = cfg.objects[int(rng.integers(len(cfg.objects)))]
        if obj not in self._obj_body:
            raise ValueError(f"object {obj!r} is not in this scene ({cfg.objects})")
        if side is None:
            side = SIDES[int(rng.integers(len(SIDES)))]
        if side not in SIDES:
            raise ValueError(f"side must be one of {SIDES}, not {side!r}")
        self._side = side
        self._active = obj
        mujoco.mj_resetData(m, d)
        for arm in self.sides.values():  # the resting arm may sleep, the working one never
            sleepy = cfg.sleep_idle and arm.name != side
            m.tree_sleep_policy[arm.tree] = int(
                mujoco.mjtSleepPolicy.mjSLEEP_ALLOWED if sleepy else mujoco.mjtSleepPolicy.mjSLEEP_NEVER
            )

        for arm in self.sides.values():
            d.qpos[arm.arm_qadr] = arm.ready
            d.ctrl[arm.arm_act] = arm.ready
            d.mocap_pos[arm.mocap] = arm.home_pos
            d.mocap_quat[arm.mocap] = arm.home_quat
        d.ctrl[self.sides["right"].act] = 0.0  # fingers open
        d.ctrl[self.sides["left"].act] = 0.0

        for i, kind in enumerate(cfg.objects):
            adr, geom, body = self._obj_qpos[kind], self._obj_geom[kind], self._obj_body[kind]
            active = kind == obj
            m.body_gravcomp[body] = 0.0 if active else 1.0
            m.geom_contype[geom], m.geom_conaffinity[geom] = (
                self._obj_contype[kind] if active else (0, 0)
            )
            if active:
                x, y = rng.uniform(cfg.spawn_lo, cfg.spawn_hi)
                xy = self.canon_pos([x, y, 0.0])[:2]  # the left hand's box is the mirror image
                yaw = rng.uniform(-np.pi, np.pi) * (-1.0 if self.flip else 1.0)  # mirrored too
                d.qpos[adr : adr + 3] = [xy[0], xy[1], self._half_height(kind) + 0.001]
                d.qpos[adr + 3 : adr + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
            else:
                d.qpos[adr : adr + 3] = self._park_pos(i)
        mujoco.mj_forward(m, d)
        self.step(cfg.settle)
        self._rest_z = float(d.xpos[self._obj_body[obj]][2])

    def _half_height(self, kind: str) -> float:
        """From the model (not `OBJECTS`), so resized objects (randomisation) spawn correctly."""
        size = self.model.geom_size[self._obj_geom[kind]]
        return float(size[1] if OBJECTS[kind][0] == mujoco.mjtGeom.mjGEOM_CYLINDER else
                     size[0] if OBJECTS[kind][0] == mujoco.mjtGeom.mjGEOM_SPHERE else size[2])  # fmt: skip

    @property
    def rest_height(self) -> float:
        """Height (world z, m) of the active object's centre when it rested after the reset."""
        return self._rest_z

    @property
    def active_object(self) -> str:
        return self._active

    # ----- wrist -----

    def _arm(self, side: str | None) -> ArmSide:
        return self.sides[side] if side else self.arm

    def set_wrist_target(self, pos: np.ndarray, quat: np.ndarray, side: str | None = None) -> None:
        """Where the arm should put the wrist: world position (m) and quaternion (wxyz). The
        active hand's, unless `side` says otherwise."""
        arm = self._arm(side)
        quat = np.asarray(quat, dtype=float)
        self.data.mocap_pos[arm.mocap] = pos
        self.data.mocap_quat[arm.mocap] = quat / np.linalg.norm(quat)

    def wrist_target(self, side: str | None = None) -> tuple[np.ndarray, np.ndarray]:
        arm = self._arm(side)
        return self.data.mocap_pos[arm.mocap].copy(), self.data.mocap_quat[arm.mocap].copy()

    def wrist_pose(self, side: str | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Actual pose of the wrist point (the `<side>_wrist` site): world position, quat wxyz."""
        pos, mat = self.wrist_frame(side)
        return pos, mat_to_quat(mat)

    def wrist_frame(self, side: str | None = None) -> tuple[np.ndarray, np.ndarray]:
        """The wrist point and the hand model's axes (X thumb side on the right hand, -X on the
        left; Y back of the hand; Z along the fingers) as world position and 3x3 matrix."""
        arm, d = self._arm(side), self.data
        return d.site_xpos[arm.site].copy(), d.site_xmat[arm.site].reshape(3, 3).copy()

    def wrist_velocity(self, side: str | None = None) -> np.ndarray:
        """Wrist velocity (6): linear in the world frame, angular in the hand's own frame."""
        arm, m, d = self._arm(side), self.model, self.data
        vel = np.zeros(6)  # [angular, linear], world frame, at the site
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_SITE, arm.site, vel, 0)
        mat = d.site_xmat[arm.site].reshape(3, 3)
        return np.concatenate([vel[3:], mat.T @ vel[:3]])

    # ----- arms -----

    def drive_arms(self) -> None:
        """Inverse kinematics: set both arms' servo targets so each wrist goes to its target pose.
        Called by `step`; call it yourself if you step the physics directly."""
        for arm in self.sides.values():
            if self.data.tree_asleep[arm.tree] >= 0:  # asleep: nothing to do
                continue
            pos, quat = self.wrist_target(arm.name)
            q, err = arm.ik.solve(self.data.qpos[arm.arm_qadr], pos, quat_to_mat(quat), iters=6)
            self.data.ctrl[arm.arm_act] = q
            if arm.name == self._side:
                self.ik_error = err

    def arm_positions(self, side: str | None = None) -> np.ndarray:
        """The 7 arm joint angles (`tendra.arm.ARM_JOINTS` order, rad)."""
        return self.data.qpos[self._arm(side).arm_qadr].copy()

    def arm_targets(self, side: str | None = None) -> np.ndarray:
        """The arm servos' current targets (what the IK last asked for)."""
        return self.data.ctrl[self._arm(side).arm_act].copy()

    def arm_velocities(self, side: str | None = None) -> np.ndarray:
        return self.data.qvel[self._arm(side).arm_dofs].copy()

    # ----- fingers -----

    def set_finger_targets(self, q: np.ndarray, side: str | None = None) -> None:
        """16 joint targets in V1 order (rad), clipped to the joint limits."""
        q = np.asarray(q, dtype=float)
        if q.shape != (V1.num_joints,):
            raise ValueError(f"expected {V1.num_joints} targets, got shape {q.shape}")
        self.data.ctrl[self._arm(side).act] = np.clip(q, V1.lower, V1.upper)

    def finger_targets(self, side: str | None = None) -> np.ndarray:
        return self.data.ctrl[self._arm(side).act].copy()

    def finger_positions(self, side: str | None = None) -> np.ndarray:
        """16 measured joint angles in V1 order (rad)."""
        return self.data.qpos[self._arm(side).qadr].copy()

    def all_finger_positions(self, side: str | None = None) -> np.ndarray:
        """All 20 joint angles, the coupled DIPs last (`V1.all_joint_names` order, rad)."""
        side = side or self._side
        adr = [self.model.joint(f"{side}_{n}").qposadr[0] for n in V1.all_joint_names]
        return self.data.qpos[adr].copy()

    # ----- simulation and checks -----

    def step(self, seconds: float) -> None:
        n = max(1, round(seconds / self.model.opt.timestep))
        chunk = max(1, round(self.config.arm_ik_dt / self.model.opt.timestep))
        while n > 0:  # the arms follow their targets through IK, refreshed every `arm_ik_dt`
            self.drive_arms()
            mujoco.mj_step(self.model, self.data, min(chunk, n))
            n -= chunk

    def object_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """World position (m) and quaternion (wxyz) of the active object."""
        b = self._obj_body[self._active]
        return self.data.xpos[b].copy(), self.data.xquat[b].copy()

    def object_contacts(self) -> tuple[bool, bool]:
        """(touches the table, touches the active hand) for the active object."""
        m, d = self.model, self.data
        geom = self._obj_geom[self._active]
        g1, g2 = d.contact.geom1[: d.ncon], d.contact.geom2[: d.ncon]
        other = np.concatenate([g2[g1 == geom], g1[g2 == geom]])
        table = bool(np.any(other == self._table))
        hand = any(int(m.geom_bodyid[g]) in self._hand_bodies for g in other)
        return table, hand

    def lifted(self, height: float = 0.05) -> bool:
        """True if the object is >= `height` above its resting height, held by the hand only."""
        z = self.data.xpos[self._obj_body[self._active]][2]
        table, hand = self.object_contacts()
        return bool(z - self._rest_z >= height and hand and not table)


# ----- a scripted grasp (a baseline and a test that the hand can hold something) -----

# Power grasp from the side, in the right hand's (canonical) view: fingers horizontal and pointing
# toward the operator's side (-Y), thumb up (+Z), palm facing +X (toward the body's middle). The
# object stands in front of the knuckles; the fingers wrap around it and the thumb (cmc_flex)
# presses it from the wrist side (found by trial, holds for every spawn tried). Columns = hand
# model axes in the world. This is also the start pose's orientation (the arm's "table" pose).
SIDE_GRASP_ROT = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
# Wrist minus cylinder centre (world, m) in that pose. The forearm and palm reach 45 mm below the
# wrist on the little-finger side, so the wrist stays >= 50 mm above the table.
SIDE_GRASP_OFFSET = np.array([-0.040, 0.085, 0.0065])

# The arm can't put its wrist as low as the grasp wants for the small cube and ball (the forearm
# would touch the table), so the fingers tip down a little and the wrist rides a little higher.
# Found by sweeping both: all three objects lift from every spawn tried, with exact IK all the way.
ARM_GRASP = {"pitch": math.radians(8.0), "raise_": 0.01}

POWER_GRASP: dict[str, float] = {
    **{f"{f}_{j}": q for f in ("index", "middle", "ring", "little")
       for j, q in (("mcp_flex", 1.4), ("pip", 1.5), ("mcp_abd", 0.0))},  # DIP: 0.75 x PIP
    "thumb_cmc_rot": 0.0,
    "thumb_cmc_flex": 0.75,  # V1 limit is 45 deg (0.785)
    "thumb_mcp_flex": 0.4,
    "thumb_ip": 0.4,
}  # fmt: skip


def grasp_targets(grasp: dict[str, float] = POWER_GRASP) -> np.ndarray:
    """16 finger targets (V1 order) from a {joint name: rad} dict; missing joints = 0."""
    return np.array([grasp.get(name, 0.0) for name in V1.joint_names])


def move_wrist(scene: GraspScene, pos: np.ndarray, quat: np.ndarray, seconds: float,
               dt: float = 0.02) -> None:  # fmt: skip
    """Move the active hand's wrist target along a straight line (and turn it) to `pos`, `quat`."""
    p0, q0 = scene.wrist_target()
    quat = np.asarray(quat, dtype=float)
    if np.dot(q0, quat) < 0:
        quat = -quat
    steps = max(1, round(seconds / dt))
    for i in range(1, steps + 1):
        s = i / steps
        s = s * s * (3 - 2 * s)  # smooth start and stop
        q = (1 - s) * q0 + s * quat  # normalised lerp (set_wrist_target normalises)
        scene.set_wrist_target(p0 + s * (np.asarray(pos) - p0), q)
        scene.step(dt)


def scripted_grasp(scene: GraspScene, lift: float = 0.10, hold: float = 2.0, pitch: float = 0.0,
                   raise_: float = 0.0) -> None:  # fmt: skip
    """Pick up the active object with a side power grasp, lift, hold (with either hand).

    `pitch` (rad) tips the fingers down about the x axis and `raise_` (m) lifts the wrist: pass
    `**ARM_GRASP` for the cube and the ball. Works out the grasp for the right hand and maps it to
    the left one with the scene's mirror.
    """
    c, s = np.cos(pitch), np.sin(pitch)
    tip = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    obj = scene.canon_pos(scene.object_pose()[0])
    grasp = obj + tip @ SIDE_GRASP_OFFSET + np.array([0.0, 0.0, raise_])
    side = np.array([-0.05, 0.0, 0.0])  # pre-grasp: beside the object, palm facing it
    home_z = scene.canon_pos(scene.home_pos)[2]

    def go(pos: np.ndarray, seconds: float) -> None:
        quat = mat_to_quat(scene.canon_rot(tip @ SIDE_GRASP_ROT))
        move_wrist(scene, scene.canon_pos(pos), quat, seconds)

    scene.set_finger_targets(np.zeros(V1.num_joints))
    go(np.array([*(grasp + side)[:2], max(home_z, grasp[2])]), 0.8)
    go(grasp + side, 0.5)
    go(grasp, 0.5)
    scene.set_finger_targets(grasp_targets())
    scene.step(0.8)
    go(grasp + np.array([0.0, 0.0, lift]), 1.0)
    scene.step(hold)


__all__ = [
    "ARM_GRASP",
    "CONTACT_PARTS",
    "OBJECTS",
    "POWER_GRASP",
    "SIDE_GRASP_OFFSET",
    "SIDE_GRASP_ROT",
    "ArmSide",
    "GraspScene",
    "SceneConfig",
    "contact_sensor_name",
    "grasp_targets",
    "mat_to_quat",
    "move_wrist",
    "quat_to_mat",
    "scripted_grasp",
    "table_sensor_name",
]
