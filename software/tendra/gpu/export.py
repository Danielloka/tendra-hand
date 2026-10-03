"""Export everything the GPU grasp env needs into one file (a "bundle").

The GPU env (`grasp_env_jax.py`) runs on Colab without this repo, so the bundle carries:
the compiled two-arm grasp scene (meshes included) with contact sensors, both sides' indices and
limits, each arm's kinematic chain (for inverse kinematics in JAX), the synergies, the reward
geometry, and demo states from scripted grasps with either hand.

    uv run python -c "from tendra.gpu.export import export_bundle; export_bundle('runs/colab/tendra_gpu.npz')"

Arrays that differ per side have a leading axis of 2: [right, left] (`tendra.arm.SIDES`).

Differences from the CPU scene, because every GPU world must share one model:
- every object collides and has gravity compensation; the env pulls the *active* object down
  with an applied force (`xfrc_applied`), and parks the others far behind the table;
- no sleeping (a CPU-only MuJoCo feature): the idle arm holds its start pose with its position
  servos and gravity compensation, like a sleeping arm would;
- contacts come from MuJoCo contact sensors (`SceneConfig.contact_sensors`), not a contact loop.
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

import mujoco
import numpy as np

from ..arm import ARM_JOINTS, SIDES, ArmIK
from ..grasp_env import (
    OBJECT_KINDS,
    PALM_CENTRE,
    PARTS,
    POWER_ZONE,
    GraspEnv,
    GraspEnvConfig,
)
from ..joints import V1
from ..scene import (
    CONTACT_PARTS,
    OBJECTS,
    GraspScene,
    SceneConfig,
    contact_sensor_name,
    table_sensor_name,
)

BUNDLE_VERSION = 2
IK_ITERS = 6  # per IK update, as in `GraspScene.drive_arms`


def _sensor_adr(model: mujoco.MjModel, name: str) -> int:
    return int(model.sensor_adr[model.sensor(name).id])


def _mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, a, b)
    return out


def _relative(pos0, quat0, pos1, quat1) -> tuple[np.ndarray, np.ndarray]:
    """Frame 1 in frame 0 (both given in the world)."""
    inv = np.zeros(4)
    mujoco.mju_negQuat(inv, quat0)
    rel = np.zeros(3)
    mujoco.mju_rotVecQuat(rel, pos1 - pos0, inv)
    return rel, _mul(inv, quat1)


def arm_chain(ik: ArmIK) -> dict[str, np.ndarray]:
    """One arm's kinematic chain, for forward kinematics without MuJoCo: per joint the body's
    frame in the previous joint's body frame (the first in the world) at zero angles, the hinge
    axis and anchor in its own body frame, then the wrist site in the last joint's body frame.
    Each joint turns its body about the axis through the anchor (MuJoCo's hinge)."""
    m = ik.model
    d = mujoco.MjData(m)
    d.qpos[:] = 0.0
    mujoco.mj_kinematics(m, d)
    if np.any(m.qpos0[ik.qadr]) or np.any(m.jnt_type[ik.joints] != mujoco.mjtJoint.mjJNT_HINGE):
        raise ValueError("the IK chain must be hinges with zero reference angles")
    pos, quat = np.zeros(3), np.array([1.0, 0, 0, 0])
    out: dict[str, list] = {"body_pos": [], "body_quat": [], "axis": [], "anchor": []}
    for j in ik.joints:
        b = m.jnt_bodyid[j]
        rel_pos, rel_quat = _relative(pos, quat, d.xpos[b], d.xquat[b])
        out["body_pos"].append(rel_pos)
        out["body_quat"].append(rel_quat)
        out["axis"].append(m.jnt_axis[j].copy())
        out["anchor"].append(m.jnt_pos[j].copy())
        pos, quat = d.xpos[b].copy(), d.xquat[b].copy()
    if m.site_bodyid[ik.site] != m.jnt_bodyid[ik.joints[-1]]:
        raise ValueError("the wrist site must be on the last joint's body")
    site_quat = np.zeros(4)
    mujoco.mju_mat2Quat(site_quat, d.site_xmat[ik.site])
    site_pos, site_quat = _relative(pos, quat, d.site_xpos[ik.site], site_quat)
    return {
        **{k: np.array(v) for k, v in out.items()},
        "site_pos": site_pos,
        "site_quat": site_quat,
    }


def export_bundle(path: str | Path, demos_per_object: int = 10, seed: int = 0) -> Path:
    """Write the bundle; `demos_per_object` scripted grasps per object *and hand* are tried
    (the successful ones are kept)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    cfg = GraspEnvConfig(randomize=False, hands="any")
    scene = GraspScene(SceneConfig(objects=OBJECT_KINDS, contact_sensors=True))
    env = GraspEnv(cfg, seed=seed, scene=scene)
    m = env.model
    assert tuple(p for p, _, _ in CONTACT_PARTS[: len(PARTS)]) == PARTS

    # Demo states (recorded with the CPU model; positions and controls carry over). With
    # hands="any" the recorder alternates the hands, a full round of objects each.
    demos = env.record_scripted_demos(2 * demos_per_object * len(OBJECT_KINDS), seed=seed)
    frames = {(s, k): [d for d in demos if d.side == s and d.obj == k]
              for s in SIDES for k in OBJECT_KINDS}  # fmt: skip

    # The shared GPU model: every object collides and floats (gravity comes from the env), no
    # sleeping. Reset first: `scene.reset` itself switches collisions and gravity compensation.
    scene.reset(np.random.default_rng(seed), obj=OBJECT_KINDS[0], side="right")
    for kind in OBJECT_KINDS:
        g, b = scene._obj_geom[kind], scene._obj_body[kind]
        m.geom_contype[g], m.geom_conaffinity[g] = scene._obj_contype[kind]
        m.body_gravcomp[b] = 1.0
    m.opt.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_SLEEP)
    for arm in scene.sides.values():  # the compiler's value (Warp refuses the ones reset sets)
        m.tree_sleep_policy[arm.tree] = int(mujoco.mjtSleepPolicy.mjSLEEP_AUTO_NEVER)
    m.opt.timestep = cfg.sim_timestep
    # Home: both arms in the start pose, fingers open, every object parked (the env places one).
    home = mujoco.MjData(m)
    for arm in scene.sides.values():
        home.qpos[arm.arm_qadr] = arm.ready
        home.ctrl[arm.arm_act] = arm.ready
    for i, kind in enumerate(OBJECT_KINDS):
        adr = scene._obj_qpos[kind]
        home.qpos[adr : adr + 7] = [*scene._park_pos(i), 1, 0, 0, 0]

    for kind in OBJECT_KINDS:  # the shared GPU model must hold for every object (run gpu1 bug)
        g, b = scene._obj_geom[kind], scene._obj_body[kind]
        assert m.geom_contype[g] and m.geom_conaffinity[g] and m.body_gravcomp[b] == 1.0, kind
    mjb = path.with_suffix(".mjb")
    mujoco.mj_saveModel(m, str(mjb), None)
    model_bytes = np.frombuffer(mjb.read_bytes(), np.uint8)
    mjb.unlink()

    order = [(s, k) for s in SIDES for k in OBJECT_KINDS]

    def demo_array(key: str) -> np.ndarray:
        return np.concatenate([getattr(d, key) for sk in order for d in frames[sk]])

    counts = np.array([sum(len(d) for d in frames[sk]) for sk in order])
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    n_side, n_obj = len(SIDES), len(OBJECT_KINDS)
    sides = [scene.sides[s] for s in SIDES]
    chains = [arm_chain(a.ik) for a in sides]
    ik = sides[0].ik
    gtypes = [OBJECTS[k][0] for k in OBJECT_KINDS]

    def per_side(f) -> np.ndarray:
        return np.array([f(s, a) for s, a in zip(SIDES, sides, strict=True)])

    # Contact sensors: [side, object, part] -> address (found, force xyz, pos xyz); the parts
    # are PARTS + "hand" (everything from the forearm down).
    sensor_part = np.array([[[_sensor_adr(m, contact_sensor_name(s, p, k))
                              for p, _, _ in CONTACT_PARTS] for k in OBJECT_KINDS]
                            for s in SIDES])  # fmt: skip
    ik_params = {"rot_weight": ik.rot_weight, "damping": ik.damping, "rest_gain": ik.rest_gain,
                 "max_step": ik.max_step, "iters": IK_ITERS, "tol": 3e-4}  # fmt: skip

    arrays = {
        "model": model_bytes,
        "home_qpos": home.qpos.copy(),
        "home_ctrl": home.ctrl.copy(),
        "mirror_x": np.array(scene.mirror_x),
        # Per side.
        "act": per_side(lambda s, a: a.act),
        "qadr": per_side(lambda s, a: a.qadr),
        "frc_max": per_side(lambda s, a: np.abs(m.actuator_forcerange[a.act]).max(axis=1)),
        "arm_qadr": per_side(lambda s, a: a.arm_qadr),
        "arm_dofs": per_side(lambda s, a: a.arm_dofs),
        "arm_act": per_side(lambda s, a: a.arm_act),
        "arm_lo": per_side(lambda s, a: a.ik.lo),
        "arm_hi": per_side(lambda s, a: a.ik.hi),
        "ready": per_side(lambda s, a: a.ready),
        "home_pos": per_side(lambda s, a: a.home_pos),
        "home_quat": per_side(lambda s, a: a.home_quat),
        "root_body": per_side(lambda s, a: a.root),
        "wrist_site": per_side(lambda s, a: a.site),
        "wrist_body": per_side(lambda s, a: m.site_bodyid[a.site]),
        "wrist_tree_root": per_side(lambda s, a: m.body_rootid[m.site_bodyid[a.site]]),
        "mocap": per_side(lambda s, a: a.mocap),
        "tips": per_side(lambda s, a: a.tips),
        **{f"chain_{k}": np.array([c[k] for c in chains]) for k in chains[0]},
        "sensor_part": sensor_part,
        "sensor_table_obj": np.array([_sensor_adr(m, table_sensor_name(k)) for k in OBJECT_KINDS]),
        "sensor_table_hand": np.array([_sensor_adr(m, table_sensor_name(s)) for s in SIDES]),
        # Objects.
        "obj_body": np.array([scene._obj_body[k] for k in OBJECT_KINDS]),
        "obj_qadr": np.array([scene._obj_qpos[k] for k in OBJECT_KINDS]),
        "obj_dof": np.array([scene._obj_dof[k] for k in OBJECT_KINDS]),
        "obj_size": np.array([m.geom_size[scene._obj_geom[k]] for k in OBJECT_KINDS]),
        "obj_mass": np.array([m.body_mass[scene._obj_body[k]] for k in OBJECT_KINDS]),
        "obj_half_height": np.array([scene._half_height(k) for k in OBJECT_KINDS]),
        "obj_radius": np.array(
            [
                m.geom_size[scene._obj_geom[k]][0]
                * (1.2 if t == mujoco.mjtGeom.mjGEOM_BOX else 1.0)
                for k, t in zip(OBJECT_KINDS, gtypes, strict=True)
            ]
        ),
        "obj_upright": np.array([t != mujoco.mjtGeom.mjGEOM_SPHERE for t in gtypes]),
        "obj_friction": np.array([m.geom_friction[scene._obj_geom[k], 0] for k in OBJECT_KINDS]),
        # The right hand's spawn box and wrist box (the left hand's are their mirror images).
        "spawn_lo": np.array(scene.config.spawn_lo),
        "spawn_hi": np.array(scene.config.spawn_hi),
        "workspace_lo": np.array(cfg.workspace_lo),
        "workspace_hi": np.array(cfg.workspace_hi),
        "lower": V1.lower,
        "upper": V1.upper,
        "syn_rest": env.syn.rest,
        "syn_basis": env.syn.basis,
        "palm_centre": PALM_CENTRE,
        "power_zone": POWER_ZONE,
        # Demos: [side, object] -> frames start / count in the demo arrays.
        "demo_qpos": demo_array("qpos"),
        "demo_qvel": demo_array("qvel"),
        "demo_ctrl": demo_array("ctrl"),
        "demo_mocap_pos": demo_array("mocap_pos"),
        "demo_mocap_quat": demo_array("mocap_quat"),
        "demo_rest_z": np.concatenate(
            [np.full(len(d), d.rest_z) for sk in order for d in frames[sk]]
        ),
        "demo_start": starts.reshape(n_side, n_obj),
        "demo_count": counts.reshape(n_side, n_obj),
        "meta": np.frombuffer(
            json.dumps(
                {
                    "version": BUNDLE_VERSION,
                    "sides": list(SIDES),
                    "objects": list(OBJECT_KINDS),
                    "parts": list(PARTS),
                    "arm_joints": list(ARM_JOINTS),
                    "synergies": list(env.syn.names),
                    "joint_names": list(V1.joint_names),
                    "sim_dt": cfg.sim_timestep,
                    "substeps": env.substeps,
                    "settle": scene.config.settle,
                    "arm_ik_dt": scene.config.arm_ik_dt,
                    "ik": ik_params,
                    "demos": {f"{s}/{k}": len(frames[s, k]) for s, k in order},
                    "env_config": {k: v for k, v in asdict(cfg).items() if k != "rewards"},
                    "rewards": asdict(cfg.rewards),
                }
            ).encode(),
            np.uint8,
        ),
    }
    np.savez_compressed(path, **arrays)
    return path


def load_meta(bundle: dict) -> dict:
    return json.loads(bytes(bundle["meta"]).decode())
