"""Tendra grasp env for GPU training: the JAX twin of `tendra.grasp_env.GraspEnv`.

Physics runs in MuJoCo Warp (through MJX), thousands of worlds in parallel on one GPU, and
training uses Brax PPO through MuJoCo Playground's `MjxEnv` interface. Standalone on purpose
(numpy, jax, mujoco, playground only, no `tendra` imports): Colab gets this file and a bundle
from `tendra.gpu.export`, nothing else from the repo.

The robot of the CPU env: two arms (OpenArm shoulders and elbows, Tendra forearms, wrists and V1
hands) at a table, one hand working per episode (`hands`: "right", "left" or "any" = random per
world). Same task and design as the CPU env (see its docs and research/ai/grasp-rl.md), and the
same observation element for element (grasp type "power"), so a policy trained here runs in the
CPU env unchanged:
- action = wrist velocity (6) + synergies (k) + residual (16), in the right hand's view; for the
  left hand everything is mirrored (`canon_*`, as `GraspScene.canon_*`);
- the wrist target moves (velocity, lead limits, workspace box), inverse kinematics in JAX (the
  exported arm chain, `ArmIK`'s damped least squares, refreshed every `arm_ik_dt` inside the
  physics substeps like `GraspScene.step`) turns it into the working arm's servo targets; the
  idle arm holds its start pose;
- human-like reward with the arm-limit penalty; asymmetric observations ("state" for the policy,
  "privileged_state" for the critic); demo-state starts with either hand.
Differences:
- power grasp only;
- contacts come from contact sensors (one net contact per hand part and object): touch flags
  are exact, but contact forces are |net force| (CPU: sum of normal forces) and opposition uses
  each part's force-weighted contact centre (CPU: every contact point);
- one model for all worlds: the active object gets gravity as an applied force, the others
  float parked far away; object mass randomisation scales that force (inertia unchanged); size,
  friction and servo stiffness are not randomised;
- no sleeping (MuJoCo Warp has none): the idle arm is simulated, held by its servos;
- curriculum settings (objects, demo_prob, penalty_scale) are fixed per training phase.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path
from typing import Any

import jax
import jax.numpy as jp
import mujoco
import numpy as np
from ml_collections import config_dict
from mujoco import mjx
from mujoco_playground._src import mjx_env

GRAVITY = 9.81
MAX_OBJECT_SPEED = 3.0  # m/s; faster = a physics blow-up
NUM_PARTS = 6  # thumb, index, middle, ring, little, palm (the sensors add "hand" = all of it)
SIDES = ("right", "left")
BUNDLE_VERSION = 2


def default_config() -> config_dict.ConfigDict:
    return config_dict.create(
        ctrl_dt=0.048,
        sim_dt=0.004,
        episode_length=104,  # 5 s
        impl="warp",
        naconmax=160 * 2048,  # contact buffer for ALL worlds: set to 160 x num_envs
        njmax=900,  # constraint rows per world
        hands="any",  # "right", "left" or "any" (random per world)
        objects=["cylinder", "cube", "ball"],  # allowed objects (curriculum)
        demo_prob=0.0,
        penalty_scale=1.0,
        lift_height=0.06,
        hold_seconds=0.5,
        wrist_speed=0.3,
        wrist_turn_speed=2.0,
        wrist_lead=0.03,
        wrist_turn_lead=0.3,
        arm_ik_dt=0.025,
        settle=0.2,  # s simulated at reset so the object comes to rest
        finger_smoothing=0.4,
        residual_scale=0.25,
        obs_noise=0.003,
        mass_scale=[0.5, 2.0],
        rewards=config_dict.create(
            reach=1.0,
            tips=0.5,
            palm_facing=0.3,
            contact=0.5,
            opposition=1.0,
            lift=4.0,
            held=2.0,
            success=30.0,
            action_rate=0.1,
            residual=0.2,
            effort=0.05,
            knock=0.3,
            tilt=0.3,
            table=0.5,
            arm_limits=0.2,
            fail=10.0,
        ),
    )


def load_bundle(path: str | Path) -> tuple[mujoco.MjModel, dict[str, np.ndarray], dict]:
    f = dict(np.load(path))
    meta = json.loads(bytes(f.pop("meta")).decode())
    if meta.get("version") != BUNDLE_VERSION:
        raise ValueError(f"bundle {path} is version {meta.get('version')}, need {BUNDLE_VERSION}"
                         " (re-export it with tendra.gpu.export)")  # fmt: skip
    with tempfile.TemporaryDirectory() as tmp:
        mjb = Path(tmp) / "model.mjb"
        mjb.write_bytes(f.pop("model").tobytes())
        model = mujoco.MjModel.from_binary_path(str(mjb))
    return model, f, meta


# ----- rotations (quaternions wxyz, like MuJoCo) -----


def quat_to_mat(q: jax.Array) -> jax.Array:
    w, x, y, z = q
    return jp.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


def mat_to_quat(r: jax.Array) -> jax.Array:
    """Rotation matrix -> unit quaternion (either sign), branch-free for jit and vmap."""
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    cands = []
    for i, s2 in enumerate((1 + tr, 1 + r[0, 0] - r[1, 1] - r[2, 2],
                            1 + r[1, 1] - r[0, 0] - r[2, 2], 1 + r[2, 2] - r[0, 0] - r[1, 1])):  # fmt: skip
        s = 2 * jp.sqrt(jp.maximum(s2, 1e-12))
        q = [
            jp.array([s / 4, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                      (r[1, 0] - r[0, 1]) / s]),
            jp.array([(r[2, 1] - r[1, 2]) / s, s / 4, (r[0, 1] + r[1, 0]) / s,
                      (r[0, 2] + r[2, 0]) / s]),
            jp.array([(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, s / 4,
                      (r[1, 2] + r[2, 1]) / s]),
            jp.array([(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s,
                      (r[1, 2] + r[2, 1]) / s, s / 4]),
        ][i]  # fmt: skip
        cands.append(q)
    best = jp.argmax(jp.array([tr, r[0, 0], r[1, 1], r[2, 2]]))
    q = jp.stack(cands)[best]
    return q / jp.linalg.norm(q)


def quat_mul(a: jax.Array, b: jax.Array) -> jax.Array:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return jp.array([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw])  # fmt: skip


def axis_angle(axis: jax.Array, angle: jax.Array) -> jax.Array:
    return jp.concatenate([jp.cos(angle / 2)[None], jp.sin(angle / 2) * axis])


def rot6d(mat: jax.Array) -> jax.Array:
    """The first two columns (as `tendra.grasp_env.rot6d`)."""
    return mat[:, :2].T.ravel()


def limit_lead(target: jax.Array, actual: jax.Array, max_angle: float) -> jax.Array:
    """`target` turned back toward `actual` if it leads by more than `max_angle` rad (as
    `tendra.grasp_env._limit_lead`)."""
    dot = jp.clip(jp.dot(target, actual), -1.0, 1.0)
    actual = jp.where(dot < 0, -actual, actual)
    dot = jp.abs(dot)
    theta = jp.arccos(dot)
    angle = 2 * theta
    f = max_angle / jp.maximum(angle, 1e-9)
    out = (jp.sin((1 - f) * theta) * actual + jp.sin(f * theta) * target) / jp.maximum(
        jp.sin(theta), 1e-9
    )
    out = out / jp.maximum(jp.linalg.norm(out), 1e-9)
    return jp.where(angle <= max_angle, target, out)


# ----- the mirror: the left hand's world as the right hand would have it (scene.canon_*) -----

_FLIP = jp.array([-1.0, 1.0, 1.0])


def canon_pos(p: jax.Array, flip: jax.Array, mirror_x: jax.Array) -> jax.Array:
    """Positions (..., 3): mirrored in the plane x = mirror_x for the left hand."""
    return jp.where(flip, p * _FLIP + jp.array([2.0, 0.0, 0.0]) * mirror_x, p)


def canon_vec(v: jax.Array, flip: jax.Array) -> jax.Array:
    """Polar vectors (velocity, offset)."""
    return jp.where(flip, v * _FLIP, v)


def canon_axial(w: jax.Array, flip: jax.Array) -> jax.Array:
    """Axial vectors (angular velocity, turn rate)."""
    return jp.where(flip, -w * _FLIP, w)


def canon_rot(r: jax.Array, flip: jax.Array) -> jax.Array:
    """Rotation matrices: M R M_hand with both = flip x."""
    return jp.where(flip, _FLIP[:, None] * r * _FLIP[None, :], r)


# ----- the arm: forward and inverse kinematics on the exported chain -----


def arm_fk(chain: dict[str, jax.Array], q: jax.Array):
    """Wrist site (position, matrix) and every joint's world anchor and axis (7 x 3 each), at
    arm angles `q` (MuJoCo's hinge kinematics)."""
    pos, quat = jp.zeros(3), jp.array([1.0, 0.0, 0.0, 0.0])
    anchors, axes = [], []
    for i in range(q.shape[0]):
        pos = pos + quat_to_mat(quat) @ chain["body_pos"][i]
        quat = quat_mul(quat, chain["body_quat"][i])
        rot = quat_to_mat(quat)
        anchor = pos + rot @ chain["anchor"][i]
        anchors.append(anchor)
        axes.append(rot @ chain["axis"][i])
        quat = quat_mul(quat, axis_angle(chain["axis"][i], q[i]))
        quat = quat / jp.linalg.norm(quat)
        pos = anchor - quat_to_mat(quat) @ chain["anchor"][i]
    site = pos + quat_to_mat(quat) @ chain["site_pos"]
    site_mat = quat_to_mat(quat_mul(quat, chain["site_quat"]))
    return site, site_mat, jp.stack(anchors), jp.stack(axes)


def _rot_error(current: jax.Array, target: jax.Array) -> jax.Array:
    m = current @ target.T
    return 0.5 * jp.array([m[1, 2] - m[2, 1], m[2, 0] - m[0, 2], m[0, 1] - m[1, 0]])


def arm_ik(chain: dict[str, jax.Array], q: jax.Array, pos: jax.Array, rot: jax.Array,
           lo: jax.Array, hi: jax.Array, rest: jax.Array, ik: dict[str, float]):  # fmt: skip
    """Damped least squares from the arm angles `q` toward the wrist pose (`pos`, `rot`): the
    same steps as `tendra.arm.ArmIK.solve` (rest pull in the null space, step limit, early stop
    at `tol`). Returns (joint targets, remaining error)."""
    rw = ik["rot_weight"]
    n = q.shape[0]
    eye_n = jp.eye(n)
    lam = ik["damping"] ** 2 * jp.eye(6)

    def body(_, carry):
        q, err_norm, done = carry
        site, site_mat, anchors, axes = arm_fk(chain, q)
        err = jp.concatenate([pos - site, rw * _rot_error(site_mat, rot)])
        e = jp.linalg.norm(err)
        done_now = done | (e < ik["tol"])
        jac = jp.concatenate([jp.cross(axes, site - anchors).T, rw * axes.T])  # 6 x n
        pinv = jac.T @ jp.linalg.inv(jac @ jac.T + lam)
        dq = pinv @ err + (eye_n - pinv @ jac) @ (ik["rest_gain"] * (rest - q))
        step = jp.max(jp.abs(dq))
        dq = dq * jp.where(step > ik["max_step"], ik["max_step"] / jp.maximum(step, 1e-12), 1.0)
        q_new = jp.clip(q + dq, lo, hi)
        return jp.where(done_now, q, q_new), jp.where(done, err_norm, e), done_now

    q, err, _ = jax.lax.fori_loop(0, ik["iters"], body, (q, jp.array(jp.inf), jp.array(False)))
    return q, err


def chunks(n: int, chunk: int) -> tuple[int, ...]:
    """`n` substeps in IK updates of `chunk` (the last one shorter), like `GraspScene.step`."""
    out = []
    while n > 0:
        out.append(min(chunk, n))
        n -= chunk
    return tuple(out)


class TendraGrasp(mjx_env.MjxEnv):
    """Grasp-and-lift with the two-arm Tendra robot, one hand per world (module docs)."""

    def __init__(self, bundle: str | Path, config: config_dict.ConfigDict | None = None,
                 config_overrides: dict[str, Any] | None = None) -> None:  # fmt: skip
        super().__init__(config or default_config(), config_overrides)
        cfg = self._config
        self._mj_model, b, self.meta = load_bundle(bundle)
        self._mj_model.opt.timestep = cfg.sim_dt
        self._mjx_model = mjx.put_model(self._mj_model, impl=cfg.impl)
        self._b = {k: jp.asarray(v) for k, v in b.items()}
        self._np = b
        kinds = self.meta["objects"]
        unknown = set(cfg.objects) - set(kinds)
        if unknown:
            raise ValueError(f"objects {unknown} not in the bundle ({kinds})")
        if cfg.hands not in (*SIDES, "any"):
            raise ValueError(f"hands must be one of {SIDES} or 'any'")
        self._allowed = jp.array([k in cfg.objects for k in kinds], dtype=float)
        self._n_obj = len(kinds)
        self._k = b["syn_basis"].shape[1]
        self._n_joints = b["qadr"].shape[1]
        self._action_size = 6 + self._k + self._n_joints
        self._hold_steps = max(1, round(cfg.hold_seconds / cfg.ctrl_dt))
        chunk = max(1, round(cfg.arm_ik_dt / cfg.sim_dt))
        self._chunks = chunks(self.n_substeps, chunk)
        self._settle_chunks = chunks(max(1, round(cfg.settle / cfg.sim_dt)), chunk)
        self._ik = {k: float(v) for k, v in self.meta["ik"].items()}
        self._ik["iters"] = int(self._ik["iters"])
        self._chain = {k.removeprefix("chain_"): v for k, v in self._b.items()
                       if k.startswith("chain_")}  # fmt: skip
        # Home: both wrist targets (mocap) at their start poses.
        mocap_pos = np.zeros((self._mj_model.nmocap, 3))
        mocap_quat = np.tile([1.0, 0, 0, 0], (self._mj_model.nmocap, 1))
        mocap_pos[b["mocap"]], mocap_quat[b["mocap"]] = b["home_pos"], b["home_quat"]
        self._home_mocap = jp.asarray(mocap_pos), jp.asarray(mocap_quat)
        # The wrist box in the right hand's view (the left hand's is its mirror image).
        self._ws_lo = self._b["home_pos"][0] + self._b["workspace_lo"]
        self._ws_hi = self._b["home_pos"][0] + self._b["workspace_hi"]

    # ----- Playground interface -----

    @property
    def xml_path(self) -> str:
        return "tendra_gpu_bundle"

    @property
    def action_size(self) -> int:
        return self._action_size

    @property
    def mj_model(self) -> mujoco.MjModel:
        return self._mj_model

    @property
    def mjx_model(self) -> mjx.Model:
        return self._mjx_model

    # ----- the arm in the physics loop -----

    def _side_chain(self, side: jax.Array) -> dict[str, jax.Array]:
        return {k: v[side] for k, v in self._chain.items()}

    def solve_ik(self, side: jax.Array, q: jax.Array, pos: jax.Array, rot: jax.Array):
        """The `side` arm's joint targets for a world wrist pose, starting at angles `q`."""
        b = self._b
        return arm_ik(self._side_chain(side), q, pos, rot, b["arm_lo"][side], b["arm_hi"][side],
                      b["ready"][side], self._ik)  # fmt: skip

    def physics(self, data: mjx.Data, ctrl: jax.Array, side: jax.Array,
                steps: tuple[int, ...]) -> tuple[mjx.Data, jax.Array]:  # fmt: skip
        """Simulate `steps` substep chunks; before each, the working arm's IK turns its wrist
        target (mocap) into servo targets (`GraspScene.step`). The idle arm keeps its targets."""
        b = self._b
        m_id = b["mocap"][side]
        pos = data.mocap_pos[m_id]
        rot = quat_to_mat(data.mocap_quat[m_id])
        err = jp.array(0.0)
        for n in steps:
            q, err = self.solve_ik(side, data.qpos[b["arm_qadr"][side]], pos, rot)
            ctrl = ctrl.at[b["arm_act"][side]].set(q)
            data = mjx_env.step(self._mjx_model, data, ctrl, n)
        return data, err

    # ----- reset -----

    def reset(self, rng: jax.Array) -> mjx_env.State:
        cfg, b = self._config, self._b
        keys = jax.random.split(rng, 10)
        if cfg.hands == "any":
            side = jax.random.randint(keys[9], (), 0, len(SIDES))
        else:
            side = jp.array(SIDES.index(cfg.hands))
        flip = side == 1
        obj = jax.random.choice(keys[0], self._n_obj, p=self._allowed / self._allowed.sum())
        body = b["obj_body"][obj]
        weight = b["obj_mass"][obj] * GRAVITY

        blank = mjx_env.make_data(self._mj_model, impl=cfg.impl, naconmax=cfg.naconmax,
                                  njmax=cfg.njmax)  # fmt: skip

        def start(qpos, qvel, ctrl, mpos, mquat, mass_scale) -> mjx.Data:
            frc = blank.xfrc_applied.at[body, 2].set(-weight * mass_scale)
            data = blank.replace(qpos=qpos, qvel=qvel, ctrl=ctrl, mocap_pos=mpos,
                                 mocap_quat=mquat, xfrc_applied=frc)  # fmt: skip
            return mjx.forward(self._mjx_model, data)

        # Normal start: arms at home, the object at a random spot on the working hand's side
        # (the right hand's box, mirrored for the left), settled on the table.
        xy = jax.random.uniform(keys[1], (2,), minval=b["spawn_lo"], maxval=b["spawn_hi"])
        xyz = canon_pos(jp.array([xy[0], xy[1], 0.0]), flip, b["mirror_x"])
        yaw = jax.random.uniform(keys[3], minval=-jp.pi, maxval=jp.pi) * jp.where(flip, -1.0, 1.0)
        z = b["obj_half_height"][obj] + 0.001
        obj_q = jp.array([xyz[0], xyz[1], z, jp.cos(yaw / 2), 0.0, 0.0, jp.sin(yaw / 2)])
        qpos = b["home_qpos"].at[b["obj_qadr"][obj] + jp.arange(7)].set(obj_q)
        mass_scale = jax.random.uniform(keys[6], minval=cfg.mass_scale[0],
                                        maxval=cfg.mass_scale[1])  # fmt: skip
        normal = start(qpos, jp.zeros(self._mj_model.nv), b["home_ctrl"], *self._home_mocap,
                       mass_scale)  # fmt: skip
        normal, _ = self.physics(normal, b["home_ctrl"], side, self._settle_chunks)

        # Demo start: a random frame of a successful scripted grasp of the same object with the
        # same hand, with the nominal mass (as recorded).
        count = b["demo_count"][side, obj]
        frame = b["demo_start"][side, obj] + jax.random.randint(keys[4], (), 0,
                                                               jp.maximum(count, 1))  # fmt: skip
        demo = start(b["demo_qpos"][frame], b["demo_qvel"][frame], b["demo_ctrl"][frame],
                     b["demo_mocap_pos"][frame], b["demo_mocap_quat"][frame], 1.0)  # fmt: skip
        use_demo = (jax.random.uniform(keys[5]) < cfg.demo_prob) & (count > 0)
        data = jax.tree.map(lambda d, n: jp.where(use_demo, d, n), demo, normal)
        mass_scale = jp.where(use_demo, 1.0, mass_scale)
        obj_pos = canon_pos(data.xpos[body], flip, b["mirror_x"])
        rest_z = jp.where(use_demo, b["demo_rest_z"][frame], obj_pos[2])

        info = {
            "rng": keys[7], "side": side, "flip": flip, "obj": obj, "rest_z": rest_z,
            "spawn_xy": obj_pos[:2], "finger_target": data.ctrl[b["act"][side]],
            "prev_action": jp.zeros(self._action_size), "hold": jp.array(0),
            "success": jp.array(0.0), "t": jp.array(0),
            "obs_bias": cfg.obs_noise * jax.random.normal(keys[8], (3,)),
            "mass_scale": mass_scale, "from_demo": use_demo.astype(float),
        }  # fmt: skip
        sense = self.sense(data, info)
        obs = self.observe(data, info, sense)
        metrics = {k: jp.array(0.0) for k in self._metric_keys()}
        return mjx_env.State(data, obs, jp.array(0.0), jp.array(0.0), metrics, info)

    @staticmethod
    def _metric_keys() -> tuple[str, ...]:
        return ("success", "held", "lift_cm", "from_demo", "blowup", "fail", "left", "ik_err_mm",
                "reach", "tips", "palm_facing", "contact", "opposition", "lift", "held_r",
                "action_rate", "residual", "effort", "knock", "tilt", "table", "arm_limits")  # fmt: skip

    # ----- step -----

    def step(self, state: mjx_env.State, action: jax.Array) -> mjx_env.State:
        cfg, b, data, info = self._config, self._b, state.data, state.info
        a = jp.clip(action, -1.0, 1.0)
        k = self._k
        side, flip, mx = info["side"], info["flip"], b["mirror_x"]

        # Wrist target: velocity command, never far ahead of the real wrist, in the workspace;
        # all in the right hand's view (`GraspEnv.step`).
        m_id, site = b["mocap"][side], b["wrist_site"][side]
        pos, quat = data.mocap_pos[m_id], data.mocap_quat[m_id]
        wrist = canon_pos(data.site_xpos[site], flip, mx)
        wrist_quat = mat_to_quat(data.site_xmat[site].reshape(3, 3))
        pos = canon_pos(pos, flip, mx) + a[0:3] * cfg.wrist_speed * cfg.ctrl_dt
        lead = pos - wrist
        dist = jp.linalg.norm(lead)
        pos = jp.where(dist > cfg.wrist_lead, wrist + lead * cfg.wrist_lead / (dist + 1e-9), pos)
        pos = canon_pos(jp.clip(pos, self._ws_lo, self._ws_hi), flip, mx)
        turn = canon_axial(a[3:6], flip) * cfg.wrist_turn_speed * cfg.ctrl_dt
        angle = jp.linalg.norm(turn)
        dq = axis_angle(turn / jp.maximum(angle, 1e-12), angle)
        quat = jp.where(angle > 1e-9, quat_mul(dq, quat), quat)  # world-frame turn
        quat = limit_lead(quat, wrist_quat, cfg.wrist_turn_lead)
        quat = quat / jp.linalg.norm(quat)

        # Fingers: synergy posture + residual, low-pass filtered.
        q_cmd = b["syn_rest"] + b["syn_basis"] @ a[6 : 6 + k] + cfg.residual_scale * a[6 + k :]
        q_cmd = jp.clip(q_cmd, b["lower"], b["upper"])
        target = info["finger_target"] + cfg.finger_smoothing * (q_cmd - info["finger_target"])
        ctrl = data.ctrl.at[b["act"][side]].set(target)
        data = data.replace(mocap_pos=data.mocap_pos.at[m_id].set(pos),
                            mocap_quat=data.mocap_quat.at[m_id].set(quat))  # fmt: skip
        data, ik_err = self.physics(data, ctrl, side, self._chunks)

        info = {**info, "finger_target": target, "t": info["t"] + 1}
        sense = self.sense(data, info)
        reward, terms, fail, blowup, new_success = self.reward(data, info, sense, a)
        info = {**info, "prev_action": a, "hold": sense["hold"],
                "success": jp.maximum(info["success"], new_success)}  # fmt: skip
        obs = self.observe(data, info, sense)
        done = (fail | blowup).astype(float)
        n = self._config.episode_length
        metrics = {
            **{key: terms[key] for key in self._metric_keys() if key in terms and key != "held"},
            "held_r": terms["held"], "success": new_success, "held": sense["held"].astype(float),
            "lift_cm": 100 * jp.clip(sense["height"], 0.0, 0.4) / n,
            "from_demo": info["from_demo"] / n, "left": flip.astype(float) / n,
            "ik_err_mm": 1000 * jp.clip(jp.nan_to_num(ik_err), 0.0, 1.0) / n,
            "blowup": blowup.astype(float), "fail": fail.astype(float),
        }  # fmt: skip
        metrics = {**state.metrics, **metrics}  # keep keys added by wrappers (e.g. "reward")
        return state.replace(data=data, obs=obs, reward=reward, done=done, metrics=metrics,
                             info=info)  # fmt: skip

    # ----- sensing (shared by reward and observation), in the right hand's view -----

    def sense(self, data: mjx.Data, info: dict) -> dict[str, jax.Array]:
        cfg, b = self._config, self._b
        side, flip, obj, mx = info["side"], info["flip"], info["obj"], b["mirror_x"]
        sd = data.sensordata
        idx = b["sensor_part"][side, obj][:, None] + jp.arange(7)  # parts + hand: found, F, pos
        part = sd[idx]
        found = part[:NUM_PARTS, 0] > 0
        in_hand = part[NUM_PARTS, 0] > 0
        net = part[:, 1:4]
        # |net force| per part; the 7th ("other": forearm, servos..) = the hand's minus the parts'.
        force = jp.concatenate([jp.linalg.norm(net[:NUM_PARTS], axis=1),
                                jp.linalg.norm(net[NUM_PARTS] - net[:NUM_PARTS].sum(0))[None]])  # fmt: skip
        body = b["obj_body"][obj]
        obj_pos = canon_pos(data.xpos[body], flip, mx)
        obj_mat_world = quat_to_mat(data.xquat[body])
        dof = b["obj_dof"][obj]
        vel = jax.lax.dynamic_slice(data.qvel, (dof,), (6,))
        obj_vel = jp.concatenate([canon_vec(vel[:3], flip), canon_axial(vel[3:], flip)])
        height = obj_pos[2] - info["rest_z"]
        # Opposition: two touching parts on opposite sides of the object's centre.
        v = canon_pos(part[:NUM_PARTS, 4:7], flip, mx) - obj_pos
        nrm = jp.linalg.norm(v, axis=1) + 1e-9
        cos = (v @ v.T) / (nrm[:, None] * nrm[None, :])
        pair = found[:, None] & found[None, :] & ~jp.eye(NUM_PARTS, dtype=bool)
        opposed = jp.any(pair & (cos < -0.3))
        table_obj = sd[b["sensor_table_obj"][obj]] > 0
        held = in_hand & ~table_obj & (height > 0.01)
        hold = jp.where(held & (height >= cfg.lift_height), info["hold"] + 1, 0)
        site = b["wrist_site"][side]
        root_pos = canon_pos(data.site_xpos[site], flip, mx)
        site_mat = data.site_xmat[site].reshape(3, 3)
        root_mat = canon_rot(site_mat, flip)
        zone = b["power_zone"] - jp.array([0.0, b["obj_radius"][obj] + 0.003, 0.0])
        # Wrist velocity (`GraspScene.wrist_velocity`): linear in the world, angular in the hand
        # frame; from the body's com-based velocity, moved to the site.
        cvel = data.cvel[b["wrist_body"][side]]
        dif = data.site_xpos[site] - data.subtree_com[b["wrist_tree_root"][side]]
        lin = cvel[3:] - jp.cross(dif, cvel[:3])
        wrist_vel = jp.concatenate([canon_vec(lin, flip), canon_axial(site_mat.T @ cvel[:3], flip)])
        return {
            "found": found, "force": force, "opposed": opposed, "in_hand": in_hand,
            "table_obj": table_obj, "held": held, "hold": hold,
            "table_hand": sd[b["sensor_table_hand"][side]] > 0, "obj_pos": obj_pos,
            "obj_mat": canon_rot(obj_mat_world, flip), "up_z": obj_mat_world[2, 2],
            "obj_vel": obj_vel, "height": height, "root_pos": root_pos, "root_mat": root_mat,
            "centre": root_pos + root_mat @ zone, "wrist_vel": wrist_vel,
            "tips": canon_pos(data.site_xpos[b["tips"][side]], flip, mx),
            "fingers": data.qpos[b["qadr"][side]], "arm_q": data.qpos[b["arm_qadr"][side]],
            "arm_v": data.qvel[b["arm_dofs"][side]],
        }  # fmt: skip

    # ----- reward (`GraspEnv._reward`, power grasp) -----

    def reward(self, data: mjx.Data, info: dict, s: dict, a: jax.Array):
        cfg, b, w = self._config, self._b, self._config.rewards
        obj, side = info["obj"], info["side"]
        t: dict[str, jax.Array] = {}
        d_centre = jp.linalg.norm(s["obj_pos"] - s["centre"])
        t["reach"] = w.reach * (0.5 * (1 - jp.tanh(d_centre / 0.2))
                                + 0.5 * (1 - jp.tanh(d_centre / 0.04)))  # fmt: skip
        d_tips = jp.maximum(jp.linalg.norm(s["tips"] - s["obj_pos"], axis=1)
                            - b["obj_radius"][obj], 0.0)  # fmt: skip
        t["tips"] = w.tips * (1 - jp.tanh(d_tips.mean() / 0.04))
        palm = s["root_pos"] + s["root_mat"] @ b["palm_centre"]
        to_obj = s["obj_pos"] - palm
        facing = jp.dot(-s["root_mat"][:, 1], to_obj) / jp.maximum(jp.linalg.norm(to_obj), 1e-6)
        t["palm_facing"] = w.palm_facing * jp.maximum(facing, 0.0) * (1 - jp.tanh(d_centre / 0.1))
        n_fingers = s["found"][1:5].sum()
        t["contact"] = w.contact * 0.5 * (jp.minimum(n_fingers, 3) / 3 + s["found"][0])
        t["opposition"] = w.opposition * s["opposed"]
        lifting = s["in_hand"] & (s["opposed"] | s["held"])
        t["lift"] = w.lift * jp.clip(s["height"] / cfg.lift_height, 0.0, 1.0) * lifting
        t["held"] = w.held * s["held"]
        new_success = ((s["hold"] >= self._hold_steps) & (info["success"] < 1)).astype(float)
        t["success"] = w.success * new_success

        p = cfg.penalty_scale
        t["action_rate"] = -p * w.action_rate * jp.mean((a - info["prev_action"]) ** 2)
        t["residual"] = -p * w.residual * jp.mean(a[6 + self._k :] ** 2)
        frc = data.actuator_force[b["act"][side]] / b["frc_max"][side]
        t["effort"] = -p * w.effort * jp.mean(frc**2)
        speed = jp.linalg.norm(s["obj_vel"][:3])
        t["knock"] = -p * w.knock * jp.minimum(speed / 0.1, 1.0) * ~(s["opposed"] | s["held"])
        upright = b["obj_upright"][obj]
        t["tilt"] = -p * w.tilt * (1 - s["up_z"]) * (upright & (s["height"] < 0.01))
        t["table"] = -p * w.table * s["table_hand"]
        lo, hi = b["arm_lo"][side], b["arm_hi"][side]
        near = jp.abs(2 * (s["arm_q"] - lo) / (hi - lo) - 1)  # 0 = middle, 1 = at a limit
        t["arm_limits"] = -p * w.arm_limits * jp.mean(jp.maximum(near - 0.8, 0.0) ** 2)

        knocked = upright & (s["up_z"] < 0.7) & ~s["in_hand"] & (s["height"] < 0.02)
        away = jp.linalg.norm(s["obj_pos"][:2] - info["spawn_xy"]) > 0.2
        fail = knocked | (s["obj_pos"][2] < -0.03) | away
        t["fail"] = -w.fail * fail
        blowup = (~jp.all(jp.isfinite(s["obj_pos"])) | (speed > MAX_OBJECT_SPEED)
                  | (s["height"] > 0.4))  # fmt: skip
        total = sum(t.values())
        reward = jp.where(blowup, -w.fail, total)
        reward = jp.nan_to_num(reward, nan=-w.fail)
        return reward, t, fail, blowup, new_success

    # ----- observation (`GraspEnv._observe`, grasp type "power") -----

    def observe(self, data: mjx.Data, info: dict, s: dict) -> dict[str, jax.Array]:
        b, cfg = self._b, self._config
        lo, hi = b["lower"], b["upper"]
        mid, half = (hi + lo) / 2, (hi - lo) / 2
        obj, side = info["obj"], info["side"]
        noise = info["obs_bias"] + cfg.obs_noise * jax.random.normal(
            jax.random.fold_in(info["rng"], info["t"]), (3,))  # fmt: skip

        def object_view(p: jax.Array) -> list[jax.Array]:
            return [p - s["root_pos"], s["root_mat"].T @ (p - s["root_pos"]), p - s["centre"],
                    (s["tips"] - p).ravel()]  # fmt: skip

        arm_lo, arm_hi = b["arm_lo"][side], b["arm_hi"][side]
        common = [
            (s["fingers"] - mid) / half, (info["finger_target"] - mid) / half,
            s["root_pos"] - b["home_pos"][0], rot6d(s["root_mat"]), s["wrist_vel"],
            2 * (s["arm_q"] - arm_lo) / (arm_hi - arm_lo) - 1, s["arm_v"],
        ]  # fmt: skip
        tail = [rot6d(s["obj_mat"]), s["obj_vel"], s["found"].astype(float),
                jax.nn.one_hot(obj, self._n_obj), b["obj_size"][obj],
                jp.array([1.0, 0.0]),  # grasp type one-hot: power
                info["prev_action"], jp.array([info["t"] / cfg.episode_length, s["height"]])]  # fmt: skip
        actor = jp.concatenate([*common, *object_view(s["obj_pos"] + noise), *tail])
        clean = jp.concatenate([*common, *object_view(s["obj_pos"]), *tail])
        privileged = jp.concatenate([
            clean, s["force"],
            jp.array([info["mass_scale"], b["obj_friction"][obj], s["table_obj"],
                      s["table_hand"], s["opposed"], s["hold"] / self._hold_steps]),
            s["obj_pos"][:2] - info["spawn_xy"],
        ])  # fmt: skip

        def clean_up(x: jax.Array) -> jax.Array:  # a blown-up world must not poison the stats
            return jp.clip(jp.nan_to_num(x, nan=0.0, posinf=0.0, neginf=0.0), -100.0, 100.0)

        return {"state": clean_up(actor), "privileged_state": clean_up(privileged)}
