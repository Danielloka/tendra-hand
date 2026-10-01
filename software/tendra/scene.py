"""A grasping scene: the Tendra V1 hand floating above a table with one object to pick up.

This is a stand-in for the future robot arm. An invisible, ideal "arm" moves the wrist: the
hand's fixed parts (forearm, palm, servos, spools) hang from one free body `hand_root`, which a
weld constraint pulls toward a mocap body `wrist_target`. So the robot's action is
"wrist pose + 20 finger joint targets", the same for a human operator (teleoperation) and for a
learned policy later.

World frame: table top at z = 0, the robot stands across the table (+Y), the "view" camera looks
at it from the operator's side (-Y), slightly from above. Units m, rad, quaternions wxyz.

    scene = GraspScene()
    scene.reset(np.random.default_rng(0), obj="cylinder")
    scene.set_wrist_target(pos, quat)       # or set_wrist_from_view(...) for teleoperation
    scene.set_finger_targets(q)             # 20 joint targets, V1 order
    scene.step(0.02)
    scene.lifted()                          # success check

The view frame (shared with the webcam wrist tracker): x = right on screen, y = up, z = toward
the viewer, origin = the home point. The screen shows the webcam image mirrored, next to the
"view" camera render, so the view camera acts like a mirror: see `set_wrist_from_view`.
"""

import functools
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .joints import V1, HandSpec
from .lite_model import KEEP, copy_inertia, simplify_meshes

# The wrist point in the hand model's frame (m): middle of the palm's lower edge, where the palm
# meets the forearm. `hand_root` sits here, so the hand turns about the wrist like a human one.
# (The model origin is at the thumb-side edge of the palm, x spans -79..12 mm, y 12..40 mm.)
WRIST_IN_MODEL = np.array([-0.0335, 0.026, -0.030])

# Home orientation in the view frame: palm facing the camera (model -Y = view +z), fingers up
# (model +Z = view +y), so the thumb side (model +X) is view +x. Columns = model axes.
HOME_ROT_VIEW = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])

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
    # Home wrist point, world (m): low and close behind the objects, so the operator only needs
    # a small, comfortable hand motion to reach them (from 0.2 m up, reaching the table meant
    # moving the real hand out of the webcam image). Measured at this pose: the forearm (servo
    # pack, on the palm side) reaches down to z 0.022 and forward to y ~-0.078, so the table
    # and the objects spawned in front (y <= -0.13) stay clear; `test_scene` checks it.
    home: tuple[float, float, float] = (0.0, 0.02, 0.15)
    # Objects spawn uniformly in this box (x, y, world m). With `spawn_sides`, x is the distance
    # to the side and the side (left/right of the hand) is random. Beside the hand, at about its
    # depth, the operator reaches by moving sideways and down: the directions a webcam tracks
    # best. (In front of the hand, toward the camera, they had to bring their real hand very
    # close to the webcam.) The hand at home spans x +-0.046 (measured), so |x| >= 0.09 keeps
    # even the ball (r 28 mm) clear of it.
    spawn_lo: tuple[float, float] = (0.09, -0.07)
    spawn_hi: tuple[float, float] = (0.15, 0.0)
    spawn_sides: bool = True
    # Weld between hand_root and wrist_target: time constant (s), damping ratio. 0.02 s tracks
    # a hand motion without visible lag and holds hand + object against gravity; MuJoCo's soft
    # constraints scale with mass, so it stays stable on contact with the table.
    weld_solref: tuple[float, float] = (0.02, 1.0)
    weld_solimp: tuple[float, ...] = (
        0.95,
        0.99,
        0.001,
        0.5,
        2.0,
    )  # stiffer than default: < 1 mm sag
    object_friction: tuple[float, float, float] = (1.0, 0.01, 0.0002)  # slide, spin, roll
    object_condim: int = 4  # 4 = with torsional friction (holds an object from turning)
    impratio: float = 10.0  # >1 makes friction harder than normal force: less slip
    settle: float = 0.2  # seconds simulated at reset so the object comes to rest
    # MuJoCo contact sensors (`CONTACT_PARTS` x objects, hand/object vs. table), for the GPU
    # training env: MuJoCo Warp reports contacts through sensors, not a Python contact loop.
    contact_sensors: bool = False
    view_fovy: float = 60.0  # degrees: fingertips at home + the spawn area both in view
    # View camera, world (m): frames the hand at home and both spawn sides, ~25 deg from above,
    # ~0.4 m away (closer made the objects on the sides leave the image).
    view_pos: tuple[float, float, float] = (0.0, -0.38, 0.32)
    view_lookat: tuple[float, float, float] = (0.0, -0.02, 0.15)


# Hand parts with a contact sensor each: (sensor name, MuJoCo object type, name). A finger is
# the subtree of its base body; the palm is the palm body alone (its subtree holds the fingers).
CONTACT_PARTS: tuple[tuple[str, mujoco.mjtObj, str], ...] = (
    *((f, mujoco.mjtObj.mjOBJ_XBODY, f"{f}_base") for f in ("thumb", "index", "middle", "ring",
                                                            "little")),
    ("palm", mujoco.mjtObj.mjOBJ_BODY, "palm"),
)  # fmt: skip
_FOUND, _FORCE, _POS = (1 << int(mujoco.mjtConDataField.mjCONDATA_FOUND),
                        1 << int(mujoco.mjtConDataField.mjCONDATA_FORCE),
                        1 << int(mujoco.mjtConDataField.mjCONDATA_POS))  # fmt: skip
_NETFORCE = 3  # contact sensor reduce mode: one net contact (force-weighted position)


def contact_sensor_name(part: str, obj: str) -> str:
    return f"touch_{part}_{obj}"


def _add_contact_sensors(scene: mujoco.MjSpec, objects: tuple[str, ...]) -> None:
    """Per object: one sensor per hand part (found, net force, position: 7 values), the
    object on the table (found) and the hand on the table (found)."""
    geom = mujoco.mjtObj.mjOBJ_GEOM
    contact = mujoco.mjtSensor.mjSENS_CONTACT
    for obj in objects:
        for part, objtype, body in CONTACT_PARTS:
            scene.add_sensor(name=contact_sensor_name(part, obj), type=contact, objtype=objtype,
                             objname=body, reftype=geom, refname=obj,
                             intprm=[_FOUND | _FORCE | _POS, _NETFORCE, 1])  # fmt: skip
        scene.add_sensor(name=f"table_{obj}", type=contact, objtype=geom, objname=obj,
                         reftype=geom, refname="table", intprm=[_FOUND, 0, 1])  # fmt: skip
    scene.add_sensor(name="table_hand", type=contact, objtype=mujoco.mjtObj.mjOBJ_XBODY,
                     objname="hand_root", reftype=geom, refname="table",
                     intprm=[_FOUND, 0, 1])  # fmt: skip


def _look_at(pos: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    """MuJoCo camera `xyaxes` (camera x and y in the parent frame) for looking at `target`."""
    back = pos - target
    back /= np.linalg.norm(back)  # camera z points backward (it looks along -z)
    x = np.cross(up, back)
    x /= np.linalg.norm(x)
    return np.concatenate([x, np.cross(back, x)])


def mat_to_quat(mat: np.ndarray) -> np.ndarray:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(mat, dtype=float).ravel())
    return quat


def quat_to_mat(quat: np.ndarray) -> np.ndarray:
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, np.asarray(quat, dtype=float))
    return mat.reshape(3, 3)


@functools.lru_cache(maxsize=2)
def _full_hand_model(path: Path) -> mujoco.MjModel:
    """The unsimplified hand, only used for its masses and inertias (cached, loading is slow)."""
    return mujoco.MjModel.from_xml_path(str(path))


def _hand_spec(spec: HandSpec, config: SceneConfig) -> mujoco.MjSpec:
    """The hand model as an MjSpec, ready to be attached (no floor, lights or keyframes)."""
    path = spec.model_path
    hand = mujoco.MjSpec.from_file(str(path))
    hand.meshdir = str((path.parent / hand.meshdir).resolve())
    if config.lite:
        simplify_meshes(hand, Path(hand.meshdir), config.lite_keep)
    for geom in list(hand.worldbody.geoms):  # the floor plane: the scene has a table instead
        hand.delete(geom)
    for light in list(hand.worldbody.lights):
        hand.delete(light)
    for key in list(hand.keys):  # keyframes have hand-only qpos sizes
        hand.delete(key)
    # Wrist camera on the palm side near the wrist, looking along the fingers (at what the hand
    # grasps), with the back of the hand up in the image.
    pos = WRIST_IN_MODEL + np.array([0.0, -0.05, 0.0])
    cam = hand.body("palm").add_camera(name="wrist", fovy=75.0, pos=pos)
    cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
    cam.alt.xyaxes = _look_at(pos, np.array([-0.0335, -0.015, 0.10]), np.array([0, 1.0, 0]))
    return hand


class GraspScene:
    """The V1 hand on an ideal floating wrist, above a table with objects (see module docs)."""

    CAMERAS = ("view", "wrist")

    def __init__(self, config: SceneConfig | None = None) -> None:
        self.config = config = config or SceneConfig()
        for kind in config.objects:
            if kind not in OBJECTS:
                raise ValueError(f"unknown object {kind!r}, expected one of {sorted(OBJECTS)}")
        self.spec = V1
        self.model = self._build()
        self.data = mujoco.MjData(self.model)
        m = self.model

        self._root = m.body("hand_root").id
        self._root_qpos = m.jnt_qposadr[m.body_jntadr[self._root]]
        self._mocap = m.body_mocapid[m.body("wrist_target").id]
        self._act = np.array([m.actuator(n).id for n in V1.joint_names])
        self._qadr = np.array([m.jnt_qposadr[m.joint(n).id] for n in V1.joint_names])
        self._table = m.geom("table").id
        # Every body of the hand (the root's subtree), for contact checks.
        self._hand_bodies = {b for b in range(m.nbody) if self._in_hand(b)}
        self._obj_body = {k: m.body(k).id for k in config.objects}
        self._obj_qpos = {k: m.jnt_qposadr[m.body_jntadr[b]] for k, b in self._obj_body.items()}
        self._obj_dof = {k: m.jnt_dofadr[m.body_jntadr[b]] for k, b in self._obj_body.items()}
        self._obj_geom = {k: m.body_geomadr[b] for k, b in self._obj_body.items()}
        self._obj_contype = {k: (m.geom_contype[g], m.geom_conaffinity[g])
                             for k, g in self._obj_geom.items()}  # fmt: skip

        # The view camera's basis C = [right, up, backward] as world columns (fixed camera).
        mujoco.mj_kinematics(m, self.data)
        mujoco.mj_camlight(m, self.data)
        # The mirror mapping uses the view camera's axes *levelled*: right = the camera's right
        # (horizontal), up = world up, backward = horizontal toward the camera. With the camera's
        # own (tilted, ~30 deg down) axes, moving the real hand toward the webcam also lifted the
        # sim hand, so the operator had to hold their hand very low to reach the table.
        cam_x = self.data.cam_xmat[m.camera("view").id].reshape(3, 3)[:, 0]
        right = np.array([cam_x[0], cam_x[1], 0.0]) / np.linalg.norm(cam_x[:2])
        up = np.array([0.0, 0.0, 1.0])
        self.view_basis = np.column_stack([right, up, np.cross(right, up)])
        self.home_pos = np.array(config.home, dtype=float)
        self.home_quat = mat_to_quat(self.view_basis @ HOME_ROT_VIEW)

        self._active = config.objects[0]
        self._rest_z = 0.0
        self.reset(np.random.default_rng(0))

    # ----- building the model -----

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
        world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1],
                       pos=[0, 0, -0.75], material="scene_floor", contype=0, conaffinity=0)  # fmt: skip

        cam = world.add_camera(name="view", fovy=cfg.view_fovy, pos=list(cfg.view_pos))
        cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
        cam.alt.xyaxes = _look_at(np.array(cfg.view_pos), np.array(cfg.view_lookat),
                                  np.array([0, 0, 1.0]))  # fmt: skip

        # The floating hand: a free body whose frame is the wrist point.
        root = world.add_body(name="hand_root", pos=list(cfg.home))
        root.add_freejoint(name="hand_root")
        frame = root.add_frame(pos=list(-WRIST_IN_MODEL))
        scene.attach(_hand_spec(V1, cfg), frame=frame, prefix="")
        target = world.add_body(name="wrist_target", mocap=True, pos=list(cfg.home))
        target.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.004], rgba=[0.1, 0.6, 1, 0.5],
                        contype=0, conaffinity=0, group=2)  # fmt: skip
        scene.add_equality(type=mujoco.mjtEq.mjEQ_WELD, name="wrist", name1="hand_root",
                           name2="wrist_target", objtype=mujoco.mjtObj.mjOBJ_BODY,
                           solref=list(cfg.weld_solref), solimp=list(cfg.weld_solimp),
                           data=[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1])  # identity offset  # fmt: skip

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
            copy_inertia(model, _full_hand_model(V1.model_path))
        model.light_castshadow[:] = 0
        model.mat_reflectance[:] = 0
        mujoco.mj_setConst(model, mujoco.MjData(model))
        if model.nu != V1.num_joints or model.ntendon != 2 * V1.num_joints:
            raise RuntimeError("scene model lost hand actuators or tendons")
        return model

    @staticmethod
    def _park_pos(i: int) -> list[float]:
        return [1.0 + 0.2 * i, 1.5, 0.3]  # far behind the table, out of every camera's view

    def _in_hand(self, body: int) -> bool:
        while body:
            if body == self._root:
                return True
            body = self.model.body_parentid[body]
        return False

    # ----- episode -----

    def reset(self, rng: np.random.Generator | None = None, obj: str | None = None) -> None:
        """Hand home (fingers open), one object at a random spot on the table, others parked."""
        rng = rng or np.random.default_rng()
        m, d = self.model, self.data
        if obj is None:
            obj = self.config.objects[int(rng.integers(len(self.config.objects)))]
        if obj not in self._obj_body:
            raise ValueError(f"object {obj!r} is not in this scene ({self.config.objects})")
        self._active = obj
        mujoco.mj_resetData(m, d)

        d.qpos[self._root_qpos : self._root_qpos + 3] = self.home_pos
        d.qpos[self._root_qpos + 3 : self._root_qpos + 7] = self.home_quat
        self.set_wrist_target(self.home_pos, self.home_quat)
        self.set_finger_targets(np.zeros(V1.num_joints))

        for i, kind in enumerate(self.config.objects):
            adr, geom, body = self._obj_qpos[kind], self._obj_geom[kind], self._obj_body[kind]
            active = kind == obj
            m.body_gravcomp[body] = 0.0 if active else 1.0
            m.geom_contype[geom], m.geom_conaffinity[geom] = (
                self._obj_contype[kind] if active else (0, 0)
            )
            if active:
                xy = rng.uniform(self.config.spawn_lo, self.config.spawn_hi)
                if self.config.spawn_sides and rng.random() < 0.5:
                    xy[0] = -xy[0]  # the other side of the hand
                yaw = rng.uniform(-np.pi, np.pi)
                d.qpos[adr : adr + 3] = [xy[0], xy[1], self._half_height(kind) + 0.001]
                d.qpos[adr + 3 : adr + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
            else:
                d.qpos[adr : adr + 3] = self._park_pos(i)
        mujoco.mj_forward(m, d)
        self.step(self.config.settle)
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

    def set_wrist_target(self, pos: np.ndarray, quat: np.ndarray) -> None:
        """Where the ideal arm should put the wrist: world position (m) and quaternion (wxyz)."""
        quat = np.asarray(quat, dtype=float)
        self.data.mocap_pos[self._mocap] = pos
        self.data.mocap_quat[self._mocap] = quat / np.linalg.norm(quat)

    def view_to_world(self, pos_view: np.ndarray, rot_view: np.ndarray
                      ) -> tuple[np.ndarray, np.ndarray]:  # fmt: skip
        """A view-frame wrist pose as a world pose (pos, quat), the view camera as a mirror.

        world_pos = home + C @ pos_view, world_R = C @ rot_view, C = `view_basis` = [right, up,
        backward] of the view camera, levelled (up = world up, backward = horizontal toward the
        camera). `rot_view` columns are the hand model's axes (X thumb side, Y back of the hand,
        Z fingers) in the view frame; `HOME_ROT_VIEW` = palm facing the camera, fingers up.
        """
        c = self.view_basis
        pos = self.home_pos + c @ np.asarray(pos_view, dtype=float)
        return pos, mat_to_quat(c @ np.asarray(rot_view, dtype=float))

    def set_wrist_from_view(self, pos_view: np.ndarray, rot_view: np.ndarray) -> None:
        self.set_wrist_target(*self.view_to_world(pos_view, rot_view))

    def wrist_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Actual pose of hand_root (the wrist point): world position, quaternion wxyz."""
        return self.data.xpos[self._root].copy(), self.data.xquat[self._root].copy()

    def wrist_target(self) -> tuple[np.ndarray, np.ndarray]:
        return self.data.mocap_pos[self._mocap].copy(), self.data.mocap_quat[self._mocap].copy()

    # ----- fingers -----

    def set_finger_targets(self, q: np.ndarray) -> None:
        """20 joint targets in V1 order (rad), clipped to the joint limits."""
        q = np.asarray(q, dtype=float)
        if q.shape != (V1.num_joints,):
            raise ValueError(f"expected {V1.num_joints} targets, got shape {q.shape}")
        self.data.ctrl[self._act] = np.clip(q, V1.lower, V1.upper)

    def finger_targets(self) -> np.ndarray:
        return self.data.ctrl[self._act].copy()

    def finger_positions(self) -> np.ndarray:
        """20 measured joint angles in V1 order (rad)."""
        return self.data.qpos[self._qadr].copy()

    # ----- simulation and checks -----

    def step(self, seconds: float) -> None:
        n = max(1, round(seconds / self.model.opt.timestep))
        mujoco.mj_step(self.model, self.data, n)

    def object_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """World position (m) and quaternion (wxyz) of the active object."""
        b = self._obj_body[self._active]
        return self.data.xpos[b].copy(), self.data.xquat[b].copy()

    def object_contacts(self) -> tuple[bool, bool]:
        """(touches the table, touches the hand) for the active object."""
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

# Power grasp of the standing cylinder from the side: fingers horizontal and pointing toward
# the view camera (-Y), thumb side up (+Z), palm facing +X. The cylinder stands in front of the
# knuckles; the fingers wrap around it and the thumb (cmc_flex) presses it from the wrist side
# (found by trial, holds for every spawn tried). Columns = hand model axes in the world.
SIDE_GRASP_ROT = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
# Wrist minus cylinder centre (world, m) in that pose. The forearm and palm reach 45 mm below
# the wrist on the little-finger side, so the wrist stays >= 50 mm above the table.
SIDE_GRASP_OFFSET = np.array([-0.040, 0.085, 0.0065])

POWER_GRASP: dict[str, float] = {
    **{f"{f}_{j}": q for f in ("index", "middle", "ring", "little")
       for j, q in (("mcp_flex", 1.4), ("pip", 1.5), ("dip", 1.2), ("mcp_abd", 0.0))},
    "thumb_cmc_rot": 0.0,
    "thumb_cmc_flex": 0.75,  # V1 limit is 45 deg (0.785)
    "thumb_mcp_flex": 0.4,
    "thumb_ip": 0.4,
}  # fmt: skip


def grasp_targets(grasp: dict[str, float] = POWER_GRASP) -> np.ndarray:
    """20 finger targets (V1 order) from a {joint name: rad} dict; missing joints = 0."""
    return np.array([grasp.get(name, 0.0) for name in V1.joint_names])


def move_wrist(scene: GraspScene, pos: np.ndarray, quat: np.ndarray, seconds: float,
               dt: float = 0.02) -> None:  # fmt: skip
    """Move the wrist target along a straight line (and turn it) to `pos`, `quat`."""
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


def scripted_grasp(scene: GraspScene, lift: float = 0.10, hold: float = 2.0) -> None:
    """Pick up the active object (a standing cylinder) with a side power grasp, lift, hold."""
    obj, _ = scene.object_pose()
    quat = mat_to_quat(SIDE_GRASP_ROT)
    grasp = obj + SIDE_GRASP_OFFSET
    side = np.array([-0.05, 0.0, 0.0])  # pre-grasp: beside the object, palm facing it
    scene.set_finger_targets(np.zeros(V1.num_joints))
    move_wrist(scene, np.array([*(grasp + side)[:2], scene.home_pos[2]]), quat, 0.8)
    move_wrist(scene, grasp + side, quat, 0.5)
    move_wrist(scene, grasp, quat, 0.5)
    scene.set_finger_targets(grasp_targets())
    scene.step(0.8)
    move_wrist(scene, grasp + np.array([0.0, 0.0, lift]), quat, 1.0)
    scene.step(hold)
