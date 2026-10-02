"""Export everything the GPU grasp env needs into one file (a "bundle").

The GPU env (`grasp_env_jax.py`) runs on Colab without this repo, so the bundle carries:
the compiled grasp scene (meshes included) with contact sensors, the hand's indices and limits,
the synergies, the reward geometry, and demo states from scripted grasps.

    uv run python -c "from tendra.gpu.export import export_bundle; export_bundle('runs/colab/tendra_gpu.npz')"

Differences from the CPU scene, because every GPU world must share one model:
- every object collides and has gravity compensation; the env pulls the *active* object down
  with an applied force (`xfrc_applied`), and parks the others far behind the table;
- contacts come from MuJoCo contact sensors (`SceneConfig.contact_sensors`), not a contact loop.
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

import mujoco
import numpy as np

from ..grasp_env import (
    OBJECT_KINDS,
    PALM_CENTRE,
    PARTS,
    POWER_ZONE,
    GraspEnv,
    GraspEnvConfig,
)
from ..joints import V1
from ..scene import CONTACT_PARTS, OBJECTS, GraspScene, SceneConfig, contact_sensor_name

BUNDLE_VERSION = 1


def _sensor_adr(model: mujoco.MjModel, name: str) -> int:
    return int(model.sensor_adr[model.sensor(name).id])


def export_bundle(path: str | Path, demos_per_object: int = 20, seed: int = 0) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    cfg = GraspEnvConfig(randomize=False)
    scene = GraspScene(SceneConfig(objects=OBJECT_KINDS, contact_sensors=True))
    env = GraspEnv(cfg, seed=seed, scene=scene)
    m = env.model
    assert tuple(p for p, _, _ in CONTACT_PARTS) == PARTS

    # Demo states (recorded with the CPU model; positions and controls carry over).
    demos = env.record_scripted_demos(demos_per_object * len(OBJECT_KINDS), seed=seed)
    frames = {k: [d for d in demos if d.obj == k] for k in OBJECT_KINDS}

    # The shared GPU model: every object collides and floats (gravity comes from the env).
    # Reset first: `scene.reset` itself switches collisions and gravity compensation per object.
    scene.reset(np.random.default_rng(seed), obj=OBJECT_KINDS[0])
    home_qpos = scene.data.qpos.copy()
    for kind in OBJECT_KINDS:
        g, b = scene._obj_geom[kind], scene._obj_body[kind]
        m.geom_contype[g], m.geom_conaffinity[g] = scene._obj_contype[kind]
        m.body_gravcomp[b] = 1.0
    for i, kind in enumerate(OBJECT_KINDS):
        adr = scene._obj_qpos[kind]
        home_qpos[adr : adr + 7] = [*scene._park_pos(i), 1, 0, 0, 0]

    for kind in OBJECT_KINDS:  # the shared GPU model must hold for every object (run gpu1 bug)
        g, b = scene._obj_geom[kind], scene._obj_body[kind]
        assert m.geom_contype[g] and m.geom_conaffinity[g] and m.body_gravcomp[b] == 1.0, kind
    mjb = path.with_suffix(".mjb")
    mujoco.mj_saveModel(m, str(mjb), None)
    model_bytes = np.frombuffer(mjb.read_bytes(), np.uint8)
    mjb.unlink()

    def demo_array(key: str) -> np.ndarray:
        return np.concatenate([getattr(d, key) for k in OBJECT_KINDS for d in frames[k]])

    counts = np.array([sum(len(d) for d in frames[k]) for k in OBJECT_KINDS])
    sensors = np.array([[_sensor_adr(m, contact_sensor_name(p, k)) for p in PARTS]
                        for k in OBJECT_KINDS])  # fmt: skip
    gtypes = [OBJECTS[k][0] for k in OBJECT_KINDS]
    arrays = {
        "model": model_bytes,
        "home_qpos": home_qpos,
        "home_pos": scene.home_pos,
        "home_quat": scene.home_quat,
        "qadr": scene._qadr,
        "act": scene._act,
        "lower": V1.lower,
        "upper": V1.upper,
        "frc_max": np.abs(m.actuator_forcerange[scene._act]).max(axis=1),
        "root_body": np.array(scene._root),
        "root_dof": np.array(m.jnt_dofadr[m.body_jntadr[scene._root]]),
        "mocap": np.array(scene._mocap),
        "tips": np.array([m.site(f"{p}_tip").id for p in PARTS[:5]]),
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
        "sensor_part": sensors,
        "sensor_table_obj": np.array([_sensor_adr(m, f"table_{k}") for k in OBJECT_KINDS]),
        "sensor_table_hand": np.array(_sensor_adr(m, "table_hand")),
        "spawn_lo": np.array(scene.config.spawn_lo),
        "spawn_hi": np.array(scene.config.spawn_hi),
        "workspace_lo": np.array(cfg.workspace_lo),
        "workspace_hi": np.array(cfg.workspace_hi),
        "syn_rest": env.syn.rest,
        "syn_basis": env.syn.basis,
        "palm_centre": PALM_CENTRE,
        "power_zone": POWER_ZONE,
        "demo_qpos": demo_array("qpos"),
        "demo_qvel": demo_array("qvel"),
        "demo_ctrl": demo_array("ctrl"),
        "demo_mocap_pos": demo_array("mocap_pos"),
        "demo_mocap_quat": demo_array("mocap_quat"),
        "demo_rest_z": np.concatenate(
            [np.full(len(d), d.rest_z) for k in OBJECT_KINDS for d in frames[k]]
        ),
        "demo_start": np.concatenate([[0], np.cumsum(counts)[:-1]]),
        "demo_count": counts,
        "meta": np.frombuffer(
            json.dumps(
                {
                    "version": BUNDLE_VERSION,
                    "objects": list(OBJECT_KINDS),
                    "parts": list(PARTS),
                    "synergies": list(env.syn.names),
                    "joint_names": list(V1.joint_names),
                    "sim_dt": cfg.sim_timestep,
                    "substeps": env.substeps,
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
