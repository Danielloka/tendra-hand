"""Tendra grasp env for GPU training: the JAX twin of `tendra.grasp_env.GraspEnv`.

Physics runs in MuJoCo Warp (through MJX), thousands of worlds in parallel on one GPU, and
training uses Brax PPO through MuJoCo Playground's `MjxEnv` interface. Standalone on purpose
(numpy, jax, mujoco, playground only, no `tendra` imports): Colab gets this file and a bundle
from `tendra.gpu.export`, nothing else from the repo.

Same task and design as the CPU env (see its docs and research/ai/grasp-rl.md): action = wrist
velocity (6) + synergies (k) + residual (20); human-like reward; asymmetric observations
("state" for the policy, "privileged_state" for the critic); demo-state starts. Differences:
- power grasp only (no grasp-type input yet);
- contacts come from contact sensors (one net contact per hand part and object);
- one model for all worlds: the active object gets gravity as an applied force, the others
  float parked far away; object mass randomisation scales that force (inertia unchanged);
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
NUM_PARTS = 6  # thumb, index, middle, ring, little, palm


def default_config() -> config_dict.ConfigDict:
    return config_dict.create(
        ctrl_dt=0.048,
        sim_dt=0.004,
        episode_length=104,  # 5 s
        impl="warp",
        naconmax=96 * 2048,  # contact buffer for ALL worlds: set to 96 x num_envs
        njmax=600,  # constraint rows per world
        objects=["cylinder", "cube", "ball"],  # allowed objects (curriculum)
        demo_prob=0.0,
        penalty_scale=1.0,
        lift_height=0.06,
        hold_seconds=0.5,
        wrist_speed=0.3,
        wrist_turn_speed=2.0,
        wrist_lead=0.03,
        finger_smoothing=0.4,
        residual_scale=0.25,
        obs_noise=0.003,
        mass_scale=[0.5, 2.0],
        rewards=config_dict.create(
            reach=1.0, tips=0.5, palm_facing=0.3, contact=0.5, opposition=1.0, lift=4.0,
            held=2.0, success=30.0, action_rate=0.1, residual=0.2, effort=0.05, knock=0.3,
            tilt=0.3, table=0.5, fail=10.0,
        ),  # fmt: skip
    )


def load_bundle(path: str | Path) -> tuple[mujoco.MjModel, dict[str, np.ndarray], dict]:
    f = dict(np.load(path))
    meta = json.loads(bytes(f.pop("meta")).decode())
    with tempfile.TemporaryDirectory() as tmp:
        mjb = Path(tmp) / "model.mjb"
        mjb.write_bytes(f.pop("model").tobytes())
        model = mujoco.MjModel.from_binary_path(str(mjb))
    return model, f, meta


def quat_to_mat(q: jax.Array) -> jax.Array:
    w, x, y, z = q
    return jp.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


def quat_mul(a: jax.Array, b: jax.Array) -> jax.Array:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return jp.array([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw])  # fmt: skip


def rot6d(mat: jax.Array) -> jax.Array:
    return mat[:, :2].T.ravel()


class TendraGrasp(mjx_env.MjxEnv):
    """Grasp-and-lift with the floating Tendra V1 hand (module docs)."""

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
        self._allowed = jp.array([k in cfg.objects for k in kinds], dtype=float)
        self._n_obj = len(kinds)
        self._k = b["syn_basis"].shape[1]
        self._action_size = 6 + self._k + len(b["qadr"])
        self._hold_steps = max(1, round(cfg.hold_seconds / cfg.ctrl_dt))

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

    # ----- reset -----

    def reset(self, rng: jax.Array) -> mjx_env.State:
        cfg, b = self._config, self._b
        keys = jax.random.split(rng, 9)
        obj = jax.random.choice(keys[0], self._n_obj, p=self._allowed / self._allowed.sum())

        # Normal start: hand at home, the object at a random spot beside the hand.
        xy = jax.random.uniform(keys[1], (2,), minval=b["spawn_lo"], maxval=b["spawn_hi"])
        xy = xy.at[0].multiply(jp.where(jax.random.uniform(keys[2]) < 0.5, -1.0, 1.0))
        yaw = jax.random.uniform(keys[3], minval=-jp.pi, maxval=jp.pi)
        z = b["obj_half_height"][obj] + 5e-4
        obj_q = jp.array([xy[0], xy[1], z, jp.cos(yaw / 2), 0.0, 0.0, jp.sin(yaw / 2)])
        qpos = b["home_qpos"].at[b["obj_qadr"][obj] + jp.arange(7)].set(obj_q)
        nv, nu = self._mj_model.nv, self._mj_model.nu
        normal = (qpos, jp.zeros(nv), jp.zeros(nu), b["home_pos"], b["home_quat"], z)

        # Demo start: a random frame of a successful scripted grasp of the same object.
        count = b["demo_count"][obj]
        frame = b["demo_start"][obj] + jax.random.randint(keys[4], (), 0, jp.maximum(count, 1))
        demo = (b["demo_qpos"][frame], b["demo_qvel"][frame], b["demo_ctrl"][frame],
                b["demo_mocap_pos"][frame], b["demo_mocap_quat"][frame], b["demo_rest_z"][frame])  # fmt: skip
        use_demo = (jax.random.uniform(keys[5]) < cfg.demo_prob) & (count > 0)
        qpos, qvel, ctrl, mpos, mquat, rest_z = (jp.where(use_demo, d, n)
                                                 for d, n in zip(demo, normal, strict=True))  # fmt: skip

        data = mjx_env.make_data(self._mj_model, qpos=qpos, qvel=qvel, ctrl=ctrl,
                                 mocap_pos=mpos, mocap_quat=mquat, impl=cfg.impl,
                                 naconmax=cfg.naconmax, njmax=cfg.njmax)  # fmt: skip
        mass_scale = jax.random.uniform(keys[6], minval=cfg.mass_scale[0],
                                        maxval=cfg.mass_scale[1])  # fmt: skip
        weight = b["obj_mass"][obj] * mass_scale * GRAVITY
        data = data.replace(xfrc_applied=data.xfrc_applied.at[b["obj_body"][obj], 2].set(-weight))
        data = mjx.forward(self._mjx_model, data)

        info = {
            "rng": keys[7], "obj": obj, "rest_z": rest_z, "spawn_xy": qpos[b["obj_qadr"][obj] + jp.arange(2)],
            "finger_target": ctrl[b["act"]], "prev_action": jp.zeros(self._action_size),
            "hold": jp.array(0), "success": jp.array(0.0), "t": jp.array(0),
            "obs_bias": cfg.obs_noise * jax.random.normal(keys[8], (3,)),
            "mass_scale": mass_scale, "from_demo": use_demo.astype(float),
        }  # fmt: skip
        sense = self._sense(data, info)
        obs = self._observe(data, info, sense)
        metrics = {k: jp.array(0.0) for k in self._metric_keys()}
        return mjx_env.State(data, obs, jp.array(0.0), jp.array(0.0), metrics, info)

    @staticmethod
    def _metric_keys() -> tuple[str, ...]:
        return ("success", "held", "lift_cm", "from_demo", "blowup", "fail", "reach", "tips",
                "palm_facing", "contact", "opposition", "lift", "held_r", "action_rate",
                "residual", "effort", "knock", "tilt", "table")  # fmt: skip

    # ----- step -----

    def step(self, state: mjx_env.State, action: jax.Array) -> mjx_env.State:
        cfg, b, data, info = self._config, self._b, state.data, state.info
        a = jp.clip(action, -1.0, 1.0)
        k = self._k

        # Wrist target: velocity command, never far ahead of the real wrist, in the workspace.
        m_id = b["mocap"]
        pos, quat = data.mocap_pos[m_id], data.mocap_quat[m_id]
        wrist = data.xpos[b["root_body"]]
        pos = pos + a[0:3] * cfg.wrist_speed * cfg.ctrl_dt
        lead = pos - wrist
        dist = jp.linalg.norm(lead)
        pos = jp.where(dist > cfg.wrist_lead, wrist + lead * cfg.wrist_lead / (dist + 1e-9), pos)
        pos = jp.clip(pos, b["home_pos"] + b["workspace_lo"], b["home_pos"] + b["workspace_hi"])
        turn = a[3:6] * cfg.wrist_turn_speed * cfg.ctrl_dt
        angle = jp.linalg.norm(turn)
        dq = jp.concatenate([jp.cos(angle / 2)[None], jp.sin(angle / 2) * turn / (angle + 1e-9)])
        quat = quat_mul(dq, quat)
        quat = quat / jp.linalg.norm(quat)

        # Fingers: synergy posture + residual, low-pass filtered.
        q_cmd = b["syn_rest"] + b["syn_basis"] @ a[6 : 6 + k] + cfg.residual_scale * a[6 + k :]
        q_cmd = jp.clip(q_cmd, b["lower"], b["upper"])
        target = info["finger_target"] + cfg.finger_smoothing * (q_cmd - info["finger_target"])
        ctrl = data.ctrl.at[b["act"]].set(target)
        data = data.replace(mocap_pos=data.mocap_pos.at[m_id].set(pos),
                            mocap_quat=data.mocap_quat.at[m_id].set(quat))  # fmt: skip
        data = mjx_env.step(self._mjx_model, data, ctrl, self.n_substeps)

        info = {**info, "finger_target": target, "t": info["t"] + 1}
        sense = self._sense(data, info)
        reward, terms, fail, blowup, new_success = self._reward(data, info, sense, a)
        info = {**info, "prev_action": a, "hold": sense["hold"],
                "success": jp.maximum(info["success"], new_success)}  # fmt: skip
        obs = self._observe(data, info, sense)
        done = (fail | blowup).astype(float)
        metrics = {
            **{key: terms.get(key, jp.array(0.0)) for key in self._metric_keys()
               if key not in ("success", "held", "lift_cm", "from_demo", "blowup", "fail",
                              "held_r")},
            "held_r": terms["held"], "success": new_success, "held": sense["held"].astype(float),
            "lift_cm": 100 * jp.clip(sense["height"], 0.0, 0.4) / self._config.episode_length,
            "from_demo": info["from_demo"] / self._config.episode_length,
            "blowup": blowup.astype(float), "fail": fail.astype(float),
        }  # fmt: skip
        return state.replace(data=data, obs=obs, reward=reward, done=done, metrics=metrics,
                             info=info)  # fmt: skip

    # ----- sensing (shared by reward and observation) -----

    def _sense(self, data: mjx.Data, info: dict) -> dict[str, jax.Array]:
        b, obj = self._b, info["obj"]
        sd = data.sensordata
        idx = b["sensor_part"][obj][:, None] + jp.arange(7)  # (6 parts, found + force + pos)
        part = sd[idx]
        found = part[:, 0] > 0
        obj_pos = data.xpos[b["obj_body"][obj]]
        obj_quat = data.xquat[b["obj_body"][obj]]
        dof = b["obj_dof"][obj]
        obj_vel = jax.lax.dynamic_slice(data.qvel, (dof,), (6,))
        height = obj_pos[2] - info["rest_z"]
        # Opposition: two touching parts on opposite sides of the object's centre.
        v = part[:, 4:7] - obj_pos
        nrm = jp.linalg.norm(v, axis=1) + 1e-9
        cos = (v @ v.T) / (nrm[:, None] * nrm[None, :])
        pair = found[:, None] & found[None, :] & ~jp.eye(NUM_PARTS, dtype=bool)
        opposed = jp.any(pair & (cos < -0.3))
        in_hand = jp.any(found)
        table_obj = sd[b["sensor_table_obj"][obj]] > 0
        held = in_hand & ~table_obj & (height > 0.01)
        hold = jp.where(held & (height >= self._config.lift_height), info["hold"] + 1, 0)
        root_pos = data.xpos[b["root_body"]]
        root_mat = data.xmat[b["root_body"]].reshape(3, 3)
        zone = b["power_zone"] - jp.array([0.0, b["obj_radius"][obj] + 0.003, 0.0])
        return {
            "found": found, "force": jp.linalg.norm(part[:, 1:4], axis=1), "opposed": opposed,
            "in_hand": in_hand, "table_obj": table_obj, "held": held, "hold": hold,
            "table_hand": sd[b["sensor_table_hand"]] > 0, "obj_pos": obj_pos,
            "obj_mat": quat_to_mat(obj_quat), "obj_vel": obj_vel, "height": height,
            "root_pos": root_pos, "root_mat": root_mat, "centre": root_pos + root_mat @ zone,
            "tips": data.site_xpos[b["tips"]],
        }  # fmt: skip

    # ----- reward -----

    def _reward(self, data: mjx.Data, info: dict, s: dict, a: jax.Array):  # noqa: ANN202
        cfg, b, w = self._config, self._b, self._config.rewards
        obj = info["obj"]
        t: dict[str, jax.Array] = {}
        d_centre = jp.linalg.norm(s["obj_pos"] - s["centre"])
        t["reach"] = w.reach * (0.5 * (1 - jp.tanh(d_centre / 0.2))
                                + 0.5 * (1 - jp.tanh(d_centre / 0.04)))  # fmt: skip
        d_tips = jp.maximum(jp.linalg.norm(s["tips"] - s["obj_pos"], axis=1)
                            - b["obj_radius"][obj], 0.0)  # fmt: skip
        t["tips"] = w.tips * (1 - jp.tanh(d_tips.mean() / 0.04))
        palm = s["root_pos"] + s["root_mat"] @ b["palm_centre"]
        to_obj = s["obj_pos"] - palm
        facing = jp.dot(-s["root_mat"][:, 1], to_obj) / (jp.linalg.norm(to_obj) + 1e-6)
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
        frc = data.actuator_force[b["act"]] / b["frc_max"]
        t["effort"] = -p * w.effort * jp.mean(frc**2)
        speed = jp.linalg.norm(s["obj_vel"][:3])
        t["knock"] = -p * w.knock * jp.minimum(speed / 0.1, 1.0) * ~(s["opposed"] | s["held"])
        upright = b["obj_upright"][obj]
        up_z = s["obj_mat"][2, 2]
        t["tilt"] = -p * w.tilt * (1 - up_z) * (upright & (s["height"] < 0.01))
        t["table"] = -p * w.table * s["table_hand"]

        knocked = upright & (up_z < 0.7) & ~s["in_hand"] & (s["height"] < 0.02)
        away = jp.linalg.norm(s["obj_pos"][:2] - info["spawn_xy"]) > 0.2
        fail = knocked | (s["obj_pos"][2] < -0.03) | away
        t["fail"] = -w.fail * fail
        blowup = (~jp.all(jp.isfinite(s["obj_pos"])) | (speed > MAX_OBJECT_SPEED)
                  | (s["height"] > 0.4))  # fmt: skip
        total = sum(t.values())
        reward = jp.where(blowup, -w.fail, total)
        reward = jp.nan_to_num(reward, nan=-w.fail)
        return reward, t, fail, blowup, new_success

    # ----- observation -----

    def _observe(self, data: mjx.Data, info: dict, s: dict) -> dict[str, jax.Array]:
        b, cfg = self._b, self._config
        lo, hi = b["lower"], b["upper"]
        mid, half = (hi + lo) / 2, (hi - lo) / 2
        obj = info["obj"]
        q = data.qpos[b["qadr"]]
        root_vel = jax.lax.dynamic_slice(data.qvel, (b["root_dof"],), (6,))
        noise = info["obs_bias"] + cfg.obs_noise * jax.random.normal(
            jax.random.fold_in(info["rng"], info["t"]), (3,))  # fmt: skip

        def object_view(p: jax.Array) -> list[jax.Array]:
            return [p - s["root_pos"], s["root_mat"].T @ (p - s["root_pos"]), p - s["centre"],
                    (s["tips"] - p).ravel()]  # fmt: skip

        common = [(q - mid) / half, (info["finger_target"] - mid) / half,
                  s["root_pos"] - b["home_pos"], rot6d(s["root_mat"]), root_vel]  # fmt: skip
        tail = [rot6d(s["obj_mat"]), s["obj_vel"], s["found"].astype(float),
                jax.nn.one_hot(obj, self._n_obj), b["obj_size"][obj], info["prev_action"],
                jp.array([info["t"] / cfg.episode_length, s["height"]])]  # fmt: skip
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
