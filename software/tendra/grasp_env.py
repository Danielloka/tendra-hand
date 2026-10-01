"""A reinforcement-learning environment: the floating V1 hand learns to grasp and lift objects.

Built on `tendra.scene.GraspScene` (the same scene as webcam grasp teleop), so learned policies,
human demos and the future real arm all share one action: "move the wrist, shape the fingers".

**Action** (all in [-1, 1], `action_dim` = 6 + k + 16):

    [0:3]    wrist velocity, world frame (x `wrist_speed` m/s)
    [3:6]    wrist turn rate about the world axes (x `wrist_turn_speed` rad/s)
    [6:6+k]  finger synergy coefficients (`tendra.synergy`): a whole-hand posture, like a human
    [6+k:]   per-joint residual (x `residual_scale` rad), for what the synergies can't express

Fingers get *absolute* posture targets, low-pass filtered (`finger_smoothing`) like a human's
smooth muscle response; the wrist gets velocity commands (it has to travel).

**Observation**: `reset`/`step` return `(actor_obs, critic_obs)`. The actor sees what the real
robot could know (joint angles, wrist pose, an object pose estimate with noise, which fingers
touch). The critic also gets privileged simulator facts (clean object pose, contact forces,
mass, friction): an *asymmetric actor-critic* (Pinto et al. 2018, OpenAI 2019) learns faster and
still yields a policy that runs without them.

**Reward** (`RewardWeights`), human-like by design:
- reach: bring the object into the hand's grasp zone, palm facing it (humans approach palm first)
- touch with the thumb + fingers, *opposition* (contacts on opposite sides of the object)
- lift it without the table, hold it: success
- penalties for knocking the object over or pushing it around, pressing on the table, jerky
  actions (humans move with minimum jerk), servo effort (humans grip just hard enough), and
  postures outside the synergy space

**Curriculum and demos**: `set_demos` gives a bank of simulator states from successful grasps
(scripted, teleop or earlier policies). With probability `demo_prob` an episode starts from a
random state of a demo, e.g. with the fingers already around the object ("reverse curriculum",
Florensa 2017; demo state resets, Peng 2018 / Nair 2018): the policy sees success early and
works backward to the start. The trainer lowers `demo_prob` as the policy improves.

**Domain randomisation** (Tobin 2017, OpenAI 2019): object size, mass and friction, servo
stiffness and observation noise change every episode, so the policy doesn't overfit one
simulator and has a chance on the real hand.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .joints import V1
from .scene import OBJECTS, GraspScene, SceneConfig, mat_to_quat, quat_to_mat, scripted_grasp
from .synergy import Synergies, default_synergies

PARTS = ("thumb", "index", "middle", "ring", "little", "palm")  # contact groups seen by the actor
OTHER = len(PARTS)  # hand geoms that are not a finger or the palm (forearm, servos, spools)
GRASP_TYPES = ("power", "precision")
MAX_OBJECT_SPEED = 3.0  # m/s; faster = a physics blow-up, not something the hand did on purpose
OBJECT_KINDS = tuple(OBJECTS)  # fixed order for the one-hot, whatever the env's object set

# Points in the hand_root frame (= the hand model's axes, origin at the wrist point): the palm's
# front face is 14 mm in front of the wrist (model -Y), the power-grasp zone ~85 mm up the palm
# (model +Z), where the proximal phalanges close (the scripted grasp holds the cylinder there).
PALM_FACE_Y = -0.014
PALM_CENTRE = np.array([-0.0065, PALM_FACE_Y, 0.06])
POWER_ZONE = np.array([-0.0065, PALM_FACE_Y, 0.085])  # + object radius along -Y, see _grasp_centre


@dataclass
class RewardWeights:
    reach: float = 1.0  # object in the grasp zone (a wide + a narrow term: pulls from afar)
    tips: float = 0.5  # fingertips near the object's surface
    palm_facing: float = 0.3  # palm normal points at the object (while approaching)
    contact: float = 0.5  # fingers + thumb touching (the grasp type's fingers)
    opposition: float = 1.0  # contacts on opposite sides of the object
    lift: float = 4.0  # height toward `lift_height`, while held
    held: float = 2.0  # off the table, in the hand
    success: float = 30.0  # once, when held at `lift_height` for `hold_seconds`
    action_rate: float = 0.1  # squared change of the action (smoothness)
    residual: float = 0.2  # squared per-joint residual (stay in the synergy space)
    effort: float = 0.05  # squared servo force / force limit (grip just hard enough)
    knock: float = 0.3  # object speed while not grasped (pushing it around)
    tilt: float = 0.3  # standing object leaning (before it's lifted); falling over = fail
    table: float = 0.5  # hand touching the table
    fail: float = 10.0  # once: object knocked over, pushed off the table or far away


@dataclass
class GraspEnvConfig:
    objects: tuple[str, ...] = ("cylinder", "cube", "ball")
    grasp_type: str = "power"  # "power", "precision", or "any" (random per episode, in the obs)
    control_hz: float = 20.0
    sim_timestep: float = 0.004  # 4 ms: 2x faster than teleop's 2 ms, grasps still hold (tested)
    episode_seconds: float = 5.0
    lift_height: float = 0.06  # m above the resting height = success height
    hold_seconds: float = 0.5
    wrist_speed: float = 0.3  # m/s at |action| = 1
    wrist_turn_speed: float = 2.0  # rad/s at |action| = 1
    wrist_lead: float = 0.03  # the target may run at most this far ahead of the real wrist (m)
    workspace_lo: tuple[float, float, float] = (-0.25, -0.25, -0.14)  # wrist box, rel. to home
    workspace_hi: tuple[float, float, float] = (0.25, 0.20, 0.15)
    finger_smoothing: float = 0.4  # per step: target += smoothing * (commanded - target)
    residual_scale: float = 0.25  # rad at |residual action| = 1
    synergies: str | None = None  # .npz from `Synergies.save` (e.g. fit on teleop); None = default
    # Randomisation (each episode, uniform in these ranges); False = the nominal model.
    randomize: bool = True
    size_scale: tuple[float, float] = (0.85, 1.15)
    mass_scale: tuple[float, float] = (0.5, 2.0)
    friction: tuple[float, float] = (0.6, 1.2)
    stiffness_scale: tuple[float, float] = (0.8, 1.2)  # servo position gain
    obs_noise: float = 0.003  # m, object position noise per step (+ the same again as a bias)
    demo_prob: float = 0.0  # chance of starting from a demo state (set by the trainer)
    # Multiplies the "care" penalties (smoothness, residual, effort, knock, tilt, table). The
    # trainer starts low so a clumsy beginner still dares to touch the object (with full
    # penalties the first policy learned to keep the hand up and away), then raises it.
    penalty_scale: float = 1.0
    rewards: RewardWeights = field(default_factory=RewardWeights)


def rot6d(quat: np.ndarray) -> np.ndarray:
    """A rotation as its matrix's first two columns: continuous, unlike quaternions or Euler
    angles, which makes it much easier for a network (Zhou et al. 2019)."""
    return quat_to_mat(quat)[:, :2].T.ravel()


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, a, b)
    return out


@dataclass
class Demo:
    """One successful grasp as simulator states (one row per frame), for demo state resets.
    The same arrays a teleop dataset records, so human demos work too (`demos_from_dataset`)."""

    obj: str
    rest_z: float  # resting height of the object's centre before the grasp
    qpos: np.ndarray
    qvel: np.ndarray
    ctrl: np.ndarray
    mocap_pos: np.ndarray
    mocap_quat: np.ndarray

    def __len__(self) -> int:
        return len(self.qpos)


class GraspEnv:
    """Grasp-and-lift environment (module docs). Gymnasium-like: reset() / step(action)."""

    def __init__(self, config: GraspEnvConfig | None = None, seed: int | None = None,
                 scene: GraspScene | None = None) -> None:  # fmt: skip
        self.config = cfg = config or GraspEnvConfig()
        if cfg.grasp_type not in (*GRASP_TYPES, "any"):
            raise ValueError(f"grasp_type must be one of {GRASP_TYPES} or 'any'")
        self.scene = scene or GraspScene(SceneConfig(objects=cfg.objects))
        self.model, self.data = m, _ = self.scene.model, self.scene.data
        m.opt.timestep = cfg.sim_timestep
        self.substeps = max(1, round(1.0 / (cfg.control_hz * cfg.sim_timestep)))
        self.dt = self.substeps * cfg.sim_timestep
        self.syn = Synergies.load(cfg.synergies) if cfg.synergies else default_synergies()
        self.action_dim = 6 + self.syn.k + V1.num_joints
        self.rng = np.random.default_rng(seed)
        self.demos: list[Demo] = []

        s = self.scene
        self._root = s._root
        self._root_dof = m.jnt_dofadr[m.body_jntadr[self._root]]
        self._tips = np.array([m.site(f"{p}_tip").id for p in PARTS[:5]])
        self._act = s._act
        self._frc_max = np.abs(m.actuator_forcerange[self._act]).max(axis=1)
        # Every geom's contact group: 0..4 fingers (PARTS), 5 palm, 6 other hand part, -1 not hand.
        self._part = np.full(m.ngeom, -1)
        for g in range(m.ngeom):
            b = int(m.geom_bodyid[g])
            if b not in s._hand_bodies:
                continue
            name = m.body(b).name
            self._part[g] = next((i for i, p in enumerate(PARTS[:5]) if name.startswith(p + "_")),
                                 PARTS.index("palm") if name == "palm" else OTHER)  # fmt: skip
        self._nominal = {
            "geom_size": m.geom_size.copy(), "geom_rbound": m.geom_rbound.copy(),
            "geom_aabb": m.geom_aabb.copy(), "geom_friction": m.geom_friction.copy(),
            "body_mass": m.body_mass.copy(), "body_inertia": m.body_inertia.copy(),
            "gainprm": m.actuator_gainprm.copy(), "biasprm": m.actuator_biasprm.copy(),
        }  # fmt: skip
        self._home = s.home_pos.copy()
        self._ws_lo = self._home + np.array(cfg.workspace_lo)
        self._ws_hi = self._home + np.array(cfg.workspace_hi)

        self.reset()
        a, c = self._observe()
        self.obs_dim, self.critic_dim = a.size, c.size

    @property
    def max_steps(self) -> int:
        return round(self.config.episode_seconds / self.dt)

    @property
    def hold_steps(self) -> int:
        return max(1, round(self.config.hold_seconds / self.dt))

    # ----- configuration from the trainer -----

    def configure(self, **kwargs: Any) -> None:
        """Change config fields between episodes (curriculum), e.g. objects=..., demo_prob=..."""
        for key, value in kwargs.items():
            if not hasattr(self.config, key):
                raise AttributeError(f"GraspEnvConfig has no field {key!r}")
            if key == "objects":
                value = tuple(value)
                missing = set(value) - set(self.scene.config.objects)
                if missing:
                    raise ValueError(f"objects {missing} are not in the scene")
            setattr(self.config, key, value)

    def set_demos(self, demos: list[Demo]) -> None:
        self.demos = [dm for dm in demos if dm.obj in self.scene.config.objects]

    # ----- episode -----

    def reset(self, seed: int | None = None, obj: str | None = None,
              grasp_type: str | None = None) -> tuple[np.ndarray, np.ndarray]:  # fmt: skip
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        cfg, rng, s = self.config, self.rng, self.scene
        demos = [
            dm for dm in self.demos if dm.obj in cfg.objects and (obj is None or dm.obj == obj)
        ]
        self.from_demo = bool(demos) and rng.random() < cfg.demo_prob
        self.grasp_type = grasp_type or (
            str(rng.choice(GRASP_TYPES)) if cfg.grasp_type == "any" else cfg.grasp_type
        )
        if self.from_demo:
            demo = demos[int(rng.integers(len(demos)))]
            self._randomize(nominal=True)  # demo states were recorded with the nominal model
            s.reset(rng, obj=demo.obj)
            k, d = int(rng.integers(len(demo))), self.data
            d.qpos[:], d.qvel[:], d.ctrl[:] = demo.qpos[k], demo.qvel[k], demo.ctrl[k]
            d.mocap_pos[:], d.mocap_quat[:] = demo.mocap_pos[k], demo.mocap_quat[k]
            mujoco.mj_forward(self.model, d)
            s._rest_z = demo.rest_z
        else:
            self._randomize(nominal=not cfg.randomize)
            if obj is None:
                obj = cfg.objects[int(rng.integers(len(cfg.objects)))]
            s.reset(rng, obj=obj)

        self.t = 0
        self.hold = 0
        self.success = False
        self.ep_return = 0.0
        self.spawn_xy = s.object_pose()[0][:2].copy()
        self.finger_target = s.finger_targets()
        self.prev_action = np.zeros(self.action_dim)
        self.obs_bias = rng.normal(0, cfg.obs_noise, 3)
        self.max_height = 0.0
        self.blew_up = False
        self._contacts()
        return self._observe()

    def _randomize(self, nominal: bool) -> None:
        m, n, cfg, rng = self.model, self._nominal, self.config, self.rng
        for key in ("geom_size", "geom_rbound", "geom_aabb", "geom_friction", "body_mass",
                    "body_inertia"):  # fmt: skip
            getattr(m, key)[:] = n[key]
        m.actuator_gainprm[:] = n["gainprm"]
        m.actuator_biasprm[:] = n["biasprm"]
        if nominal:
            return
        k = rng.uniform(*cfg.stiffness_scale)
        m.actuator_gainprm[self._act, 0] *= k
        m.actuator_biasprm[self._act, :3] *= k
        for kind in self.scene.config.objects:
            g, b = self.scene._obj_geom[kind], self.scene._obj_body[kind]
            size, mass = rng.uniform(*cfg.size_scale), rng.uniform(*cfg.mass_scale)
            m.geom_size[g] *= size
            m.geom_rbound[g] *= size
            m.geom_aabb[g] *= size
            m.geom_friction[g, 0] = rng.uniform(*cfg.friction)
            m.body_mass[b] *= mass
            m.body_inertia[b] *= mass * size**2

    # ----- stepping -----

    def step(self, action: np.ndarray) -> tuple[tuple[np.ndarray, np.ndarray], float, bool, bool,
                                                 dict[str, Any]]:  # fmt: skip
        cfg, s, d = self.config, self.scene, self.data
        a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        k = self.syn.k

        # Wrist: move the target, but never far ahead of the real wrist, inside the workspace.
        pos, quat = s.wrist_target()
        wrist, _ = s.wrist_pose()
        pos = pos + a[0:3] * cfg.wrist_speed * self.dt
        lead = pos - wrist
        dist = np.linalg.norm(lead)
        if dist > cfg.wrist_lead:
            pos = wrist + lead * (cfg.wrist_lead / dist)
        pos = np.clip(pos, self._ws_lo, self._ws_hi)
        turn = a[3:6] * cfg.wrist_turn_speed * self.dt
        angle = np.linalg.norm(turn)
        if angle > 1e-9:
            dq = np.concatenate([[np.cos(angle / 2)], np.sin(angle / 2) * turn / angle])
            quat = _quat_mul(dq, quat)  # world-frame turn
        s.set_wrist_target(pos, quat)

        # Fingers: synergy posture + residual, smoothed.
        q_cmd = self.syn.posture(a[6 : 6 + k]) + cfg.residual_scale * a[6 + k :]
        q_cmd = np.clip(q_cmd, V1.lower, V1.upper)
        self.finger_target += cfg.finger_smoothing * (q_cmd - self.finger_target)
        s.set_finger_targets(self.finger_target)

        mujoco.mj_step(self.model, d, self.substeps)
        self.t += 1

        reward, terms, fail = self._reward(a)
        self.prev_action = a
        # Success does NOT end the episode: holding on keeps earning. (When it did, run try2
        # learned to hover just below the success height, where the holding reward never stops.)
        terminated = fail
        truncated = not terminated and self.t >= self.max_steps
        self.ep_return += reward
        info: dict[str, Any] = {"terms": terms}
        if terminated or truncated:
            info["episode"] = {
                "return": self.ep_return, "length": self.t, "success": self.success,
                "object": s.active_object, "grasp_type": self.grasp_type,
                "demo": self.from_demo, "max_height": self.max_height, "fail": fail,
                "unstable": self.blew_up,
            }  # fmt: skip
        return self._observe(), reward, terminated, truncated, info

    # ----- contacts -----

    def _contacts(self) -> None:
        """Per-part touch of the object, contact points and forces, hand-table contact."""
        m, d = self.model, self.data
        geom = self.scene._obj_geom[self.scene.active_object]
        table = self.scene._table
        self.touch = np.zeros(OTHER + 1, dtype=bool)
        self.force = np.zeros(OTHER + 1)
        self.points: list[tuple[int, np.ndarray]] = []
        self.obj_on_table = False
        self.hand_on_table = False
        n = d.ncon
        if n == 0:
            return
        g1, g2 = d.contact.geom1[:n], d.contact.geom2[:n]
        on_obj = np.flatnonzero((g1 == geom) | (g2 == geom))
        wrench = np.zeros(6)
        for i in on_obj:
            other = g2[i] if g1[i] == geom else g1[i]
            if other == table:
                self.obj_on_table = True
                continue
            part = self._part[other]
            if part < 0:
                continue
            mujoco.mj_contactForce(m, d, int(i), wrench)
            self.touch[part] = True
            self.force[part] += abs(wrench[0])
            self.points.append((int(part), d.contact.pos[i].copy()))
        hand_table = ((g1 == table) & (self._part[g2] >= 0)) | (
            (g2 == table) & (self._part[g1] >= 0)
        )
        self.hand_on_table = bool(np.any(hand_table))

    def _opposition(self, centre: np.ndarray, parts: set[int] | None = None) -> bool:
        """Two contacts (different hand parts) on opposite sides of the object's centre, i.e.
        the object is squeezed, not just touched: a cheap force-closure proxy."""
        pts = [(p, x - centre) for p, x in self.points if parts is None or p in parts]
        for i, (pa, va) in enumerate(pts):
            for pb, vb in pts[i + 1 :]:
                if pa != pb and np.dot(va, vb) < -0.3 * np.linalg.norm(va) * np.linalg.norm(vb):
                    return True
        return False

    # ----- geometry helpers -----

    def _object_radius(self) -> float:
        """Horizontal half-width of the active object (m)."""
        size = self.model.geom_size[self.scene._obj_geom[self.scene.active_object]]
        return float(size[0] * (1.2 if OBJECTS[self.scene.active_object][0] ==
                                mujoco.mjtGeom.mjGEOM_BOX else 1.0))  # fmt: skip

    def _grasp_centre(self, root_pos: np.ndarray, root_mat: np.ndarray) -> np.ndarray:
        """Where the object's centre should be for this grasp type (world)."""
        if self.grasp_type == "precision":  # between the thumb and index tips
            return self.data.site_xpos[self._tips[:2]].mean(axis=0)
        zone = POWER_ZONE - np.array([0.0, self._object_radius() + 0.003, 0.0])
        return root_pos + root_mat @ zone

    # ----- reward -----

    def _reward(self, a: np.ndarray) -> tuple[float, dict[str, float], bool]:
        w, cfg, d, s = self.config.rewards, self.config, self.data, self.scene
        self._contacts()
        root_pos, root_mat = d.xpos[self._root], d.xmat[self._root].reshape(3, 3)
        obj_pos, obj_quat = s.object_pose()
        obj_vel = d.qvel[s._obj_dof[s.active_object] : s._obj_dof[s.active_object] + 3]
        radius = self._object_radius()
        precision = self.grasp_type == "precision"
        fingers = (0, 1, 2) if precision else (0, 1, 2, 3, 4)
        terms: dict[str, float] = {}

        # Approach: object into the grasp zone, fingertips to its surface, palm facing it.
        d_centre = np.linalg.norm(obj_pos - self._grasp_centre(root_pos, root_mat))
        terms["reach"] = w.reach * (0.5 * (1 - np.tanh(d_centre / 0.2))
                                    + 0.5 * (1 - np.tanh(d_centre / 0.04)))  # fmt: skip
        tips = d.site_xpos[self._tips[list(fingers)]]
        d_tips = np.maximum(np.linalg.norm(tips - obj_pos, axis=1) - radius, 0.0)
        terms["tips"] = w.tips * (1 - np.tanh(d_tips.mean() / 0.04))
        if not precision:
            palm = root_pos + root_mat @ PALM_CENTRE
            to_obj = obj_pos - palm
            facing = np.dot(-root_mat[:, 1], to_obj) / max(np.linalg.norm(to_obj), 1e-6)
            terms["palm_facing"] = w.palm_facing * max(facing, 0.0) * (1 - np.tanh(d_centre / 0.1))

        # Touch and squeeze: the thumb counts as much as all fingers (humans grasp with it).
        touch = self.touch
        n_fingers = int(touch[1 : fingers[-1] + 1].sum())
        need = 2 if precision else 3
        terms["contact"] = w.contact * 0.5 * (min(n_fingers, need) / need + float(touch[0]))
        allowed = set(fingers) if precision else None  # precision: fingertip parts only
        opposed = self._opposition(obj_pos, allowed)
        terms["opposition"] = w.opposition * float(opposed)

        # Lift and hold.
        height = float(obj_pos[2] - s.rest_height)
        # Physics blow-up guard: a solver explosion once threw the object kilometres (run try2).
        # End the episode as a failure before any of it counts.
        speed = float(np.linalg.norm(obj_vel))
        if not np.all(np.isfinite(obj_pos)) or speed > MAX_OBJECT_SPEED or height > 0.4:
            self.blew_up = True
            return -w.fail, {"fail": -w.fail}, True
        self.max_height = max(self.max_height, height)
        in_hand = bool(touch[: OTHER + 1].any())
        held = in_hand and not self.obj_on_table and height > 0.01
        if in_hand and (opposed or held):
            terms["lift"] = w.lift * float(np.clip(height / cfg.lift_height, 0.0, 1.0))
        terms["held"] = w.held * float(held)
        if held and height >= cfg.lift_height:
            self.hold += 1
        else:
            self.hold = 0
        if self.hold >= self.hold_steps and not self.success:
            self.success = True
            terms["success"] = w.success

        # Human-likeness and care (scaled by the curriculum's penalty_scale).
        k = cfg.penalty_scale
        terms["action_rate"] = -k * w.action_rate * float(np.mean((a - self.prev_action) ** 2))
        res = a[6 + self.syn.k :]
        terms["residual"] = -k * w.residual * float(np.mean(res**2))
        frc = d.actuator_force[self._act] / self._frc_max
        terms["effort"] = -k * w.effort * float(np.mean(frc**2))
        if not (opposed or held):
            terms["knock"] = -k * w.knock * min(speed / 0.1, 1.0)
        upright = OBJECTS[s.active_object][0] != mujoco.mjtGeom.mjGEOM_SPHERE
        up_z = quat_to_mat(obj_quat)[2, 2]
        if upright and height < 0.01:
            terms["tilt"] = -k * w.tilt * (1 - up_z)
        terms["table"] = -k * w.table * float(self.hand_on_table)
        if precision and touch[PARTS.index("palm")]:
            terms["contact"] -= 0.5 * w.contact  # a precision grasp holds in the fingertips

        knocked_over = upright and up_z < 0.7 and not in_hand and height < 0.02
        fail = bool(knocked_over or obj_pos[2] < -0.03
                    or np.linalg.norm(obj_pos[:2] - self.spawn_xy) > 0.2)  # fmt: skip
        if fail:
            terms["fail"] = -w.fail
        return float(sum(terms.values())), terms, fail

    # ----- observation -----

    def _observe(self) -> tuple[np.ndarray, np.ndarray]:
        cfg, d, s = self.config, self.data, self.scene
        lo, hi = V1.lower, V1.upper
        mid, half = (hi + lo) / 2, (hi - lo) / 2
        root_pos, root_quat = s.wrist_pose()
        root_mat = quat_to_mat(root_quat)
        obj_pos, obj_quat = s.object_pose()
        dof = s._obj_dof[s.active_object]
        obj_vel = d.qvel[dof : dof + 6]
        noise = self.obs_bias + self.rng.normal(0, cfg.obs_noise, 3)
        kind = s.active_object
        size = self.model.geom_size[s._obj_geom[kind]]
        centre = self._grasp_centre(root_pos, root_mat)
        tips = d.site_xpos[self._tips]

        def object_view(p: np.ndarray) -> list[np.ndarray]:
            return [p - root_pos, root_mat.T @ (p - root_pos), p - centre, (tips - p).ravel()]

        common = [
            (s.finger_positions() - mid) / half,
            (self.finger_target - mid) / half,
            root_pos - self._home,
            rot6d(root_quat),
            d.qvel[self._root_dof : self._root_dof + 6],
        ]
        tail = [
            rot6d(obj_quat),
            obj_vel,
            self.touch[:OTHER].astype(float),
            np.eye(len(OBJECT_KINDS))[OBJECT_KINDS.index(kind)],
            size,
            np.eye(len(GRASP_TYPES))[GRASP_TYPES.index(self.grasp_type)],
            self.prev_action,
            [self.t / self.max_steps, obj_pos[2] - s.rest_height],
        ]
        actor = np.concatenate([*common, *object_view(obj_pos + noise), *tail])
        clean = np.concatenate([*common, *object_view(obj_pos), *tail])
        body, geom = s._obj_body[kind], s._obj_geom[kind]
        mass_scale = self.model.body_mass[body] / self._nominal["body_mass"][body]
        privileged = [
            self.force,
            [mass_scale, self.model.geom_friction[geom, 0], float(self.obj_on_table), float(self.hand_on_table),
             float(self._opposition(obj_pos)), self.hold / self.hold_steps],
            obj_pos[:2] - self.spawn_xy,
        ]  # fmt: skip
        critic = np.concatenate([clean, *privileged])
        return actor.astype(np.float32), critic.astype(np.float32)

    # ----- demos -----

    def record_scripted_demos(self, n: int, seed: int = 0) -> list[Demo]:
        """Run the scene's scripted side power grasp on random spawns; keep the successful ones as
        state sequences (one state per control step). Uses the nominal model."""
        rng = np.random.default_rng(seed)
        demos = []
        s, d = self.scene, self.data
        for i in range(n):
            obj = self.config.objects[i % len(self.config.objects)]
            self._randomize(nominal=True)
            s.reset(rng, obj=obj)
            states: list[tuple[np.ndarray, ...]] = []
            next_t = self.data.time
            original = s.step

            def recording_step(seconds: float, _step=original, states=states) -> None:
                nonlocal next_t
                end = self.data.time + seconds - 1e-9
                while self.data.time < end:  # in pieces, so long steps still give every state
                    _step(min(self.dt, end - self.data.time + 1e-9))
                    if self.data.time >= next_t - 1e-9:
                        states.append((d.qpos.copy(), d.qvel.copy(), d.ctrl.copy(),
                                       d.mocap_pos.copy(), d.mocap_quat.copy()))  # fmt: skip
                        next_t = self.data.time + self.dt

            s.step = recording_step  # type: ignore[method-assign]
            try:
                scripted_grasp(s, hold=0.6)
            finally:
                del s.step
            if s.lifted(0.05) and states:
                demos.append(Demo(obj, s.rest_height, *map(np.array, zip(*states, strict=True))))
        return demos


def demos_from_dataset(root: str | Path, model: mujoco.MjModel,
                       success_only: bool = True) -> list[Demo]:  # fmt: skip
    """Teleop episodes (`tendra.dataset`) as demos. The dataset's model must have the same sizes
    as the env's (the same scene); otherwise its states can't be loaded and it's skipped."""
    from .dataset import Dataset

    ds = Dataset(root)
    saved = ds.load_model()
    if (saved.nq, saved.nv, saved.nu, saved.nmocap) != (model.nq, model.nv, model.nu,
                                                        model.nmocap):  # fmt: skip
        raise ValueError(f"dataset {root} was recorded with a different scene model")
    demos = []
    for meta in ds.filter(success=True if success_only else None):
        ep = ds.load(meta["episode_index"])
        obj = meta.get("object") or meta.get("extra", {}).get("object")
        if obj is None or "object.pos" not in ep:
            continue
        demos.append(Demo(obj, float(ep["object.pos"][0, 2]), ep["qpos"], ep["qvel"],
                          ep["ctrl"], ep["mocap_pos"], ep["mocap_quat"]))  # fmt: skip
    return demos


def make_env(env_config: dict[str, Any], seed: int) -> GraspEnv:
    """Top level and picklable, for worker processes. Lives here (not in tendra.rl) so the
    workers never import PyTorch: ~300 MB less memory each."""
    return GraspEnv(config_from_dict(env_config), seed=seed)


def config_dict(config: GraspEnvConfig) -> dict[str, Any]:
    return asdict(config)


def config_from_dict(d: dict[str, Any]) -> GraspEnvConfig:
    d = dict(d)
    rewards = RewardWeights(**d.pop("rewards", {}))
    tuples = {k: tuple(v) for k, v in d.items() if isinstance(v, list)}
    return GraspEnvConfig(**{**d, **tuples, "rewards": rewards})


__all__ = [
    "Demo",
    "GraspEnv",
    "GraspEnvConfig",
    "RewardWeights",
    "demos_from_dataset",
    "mat_to_quat",
    "rot6d",
]
