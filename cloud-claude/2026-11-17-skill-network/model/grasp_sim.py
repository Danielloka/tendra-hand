"""Scripted top grasps in v21_scene.xml, for tests and for comparing MuJoCo C with MuJoCo Warp.

The hand starts around the object (palm down) at `s0` x the CAD `power_grasp` pose, closes to `s1` x that pose,
the arm lifts 10 cm, then holds. `ctrl_schedule` returns the control vector for each step (so the same schedule can
drive MuJoCo C and Warp).
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_robot as br

D2R = math.pi / 180
# object centre in the hand frame (m) for a palm-down grasp that fits the hand; cylinder: not used
FIT = {"ball": (0.005, -0.0475, 0.0975), "cube": (0.0, -0.0425, 0.075), "cylinder": (0.0, -0.0675, 0.075)}
PALM_CENTRE_IN_HAND = np.array([-8.5, -16.0, 43.7]) / 1000
PARK = {"cube": [0.65, 0.3, 0.43], "cylinder": [0.65, 0.45, 0.455], "ball": [0.65, 0.15, 0.435], "mug_box": [0.65, 0.6, 0.45]}


def power_grasp() -> dict[str, float]:
    ref = json.loads((br.ASSETS / "v21" / "fk_reference.json").read_text())
    return ref["poses"]["power_grasp"]["q"]


def setup(model: mujoco.MjModel, obj: str = "cube", xy=(0.32, -0.25), s0: float = 0.6, on_mug: bool | None = None):
    """(data, q_arm, q_lift): `table` keyframe, `obj` at xy, the other objects parked, arm placed so the object sits
    in the hand, fingers at s0 x power grasp. The 5 cm cube is too low for a palm-down power grasp (the thumb would
    pierce the table), so by default it sits on top of the mug-sized box (`on_mug`)."""
    on_mug = (obj == "cube") if on_mug is None else on_mug
    d = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d, model.key("table").id)
    for o in br.OBJECTS:
        a = model.joint(f"{o}_free").qposadr[0]
        if o == obj:
            d.qpos[a : a + 2] = xy
            if on_mug:
                d.qpos[a + 2] = br.TABLE_TOP + 2 * br.OBJECTS["mug_box"][1][2] + br.OBJECTS[obj][1][2] + 1e-3
        elif on_mug and o == "mug_box":
            d.qpos[a : a + 2] = xy
        else:
            d.qpos[a : a + 3] = PARK[o]
    mujoco.mj_forward(model, d)
    p_obj = d.xpos[model.body(obj).id].copy()
    rot = br.palm_target_rotation(0.0)
    p_palm = p_obj - rot @ (np.array(FIT[obj]) - PALM_CENTRE_IN_HAND)
    q_nom = model.key_qpos[model.key("table").id][:7]
    q_arm, e1 = br.solve_arm_pose(model, p_palm, rot, q_nom=q_nom, seeds=12)
    q_lift, e2 = br.solve_arm_pose(model, p_palm + [0, 0, 0.10], rot, q_nom=q_arm, seeds=12)
    assert e1 < 3e-3 and e2 < 3e-3, (e1, e2)
    arm = [model.actuator(n).id for n in br.arm_joint_names()]
    for i, n in enumerate(br.arm_joint_names()):
        d.qpos[model.joint(n).qposadr[0]] = q_arm[i]
        d.ctrl[arm[i]] = q_arm[i]
    for j, v in power_grasp().items():
        if j.startswith("wrist"):
            continue
        d.qpos[model.joint("right_" + j).qposadr[0]] = s0 * v * D2R
        d.ctrl[model.actuator("right_" + j).id] = s0 * v * D2R
    mujoco.mj_forward(model, d)
    return d, np.array(q_arm), np.array(q_lift)


def ctrl_schedule(model, d, q_arm, q_lift, s1=1.3, t_close=1.0, t_lift=1.5, t_settle=0.3):
    """Function k -> ctrl vector (settle, close, lift), step index k at the model's timestep."""
    dt = model.opt.timestep
    hand = {j: model.actuator("right_" + j).id for j in br.SERVO_JOINTS}
    start = {j: d.ctrl[a] for j, a in hand.items()}
    pg = power_grasp()
    arm = [model.actuator(n).id for n in br.arm_joint_names()]
    n0, n1, n2 = int(t_settle / dt), int(t_close / dt), int(t_lift / dt)

    def ctrl(k: int) -> np.ndarray:
        c = d.ctrl.copy()
        if k >= n0:
            s = min(1.0, (k - n0 + 1) / (0.8 * n1))
            for j, a in hand.items():
                lo, hi = model.actuator_ctrlrange[a]
                tgt = min(max(s1 * pg.get(j, 0.0) * D2R, lo), hi)
                c[a] = start[j] + s * (tgt - start[j])
        if k >= n0 + n1:
            s = min(1.0, (k - n0 - n1 + 1) / (0.8 * n2))
            c[arm] = q_arm + (0.5 - 0.5 * math.cos(math.pi * s)) * (q_lift - q_arm)
        return c

    ctrl.n_close_end = n0 + n1
    ctrl.n_total = n0 + n1 + n2
    return ctrl


def run_grasp(model, obj="cube", hold=2.0, s1=1.3, xy=(0.32, -0.25)):
    """Run the grasp in MuJoCo C. Returns a dict: lift (mm), hold slip (mm), deepest penetration while holding (mm),
    contact dims/friction of pad-object contacts at the end, success."""
    d, q_arm, q_lift = setup(model, obj, xy)
    ctrl = ctrl_schedule(model, d, q_arm, q_lift, s1)
    ob = model.body(obj).id
    palm = model.body("right_palm").id
    z0 = d.xpos[ob][2]
    n_hold = int(hold / model.opt.timestep)
    minpen = 0.0
    rel0 = None
    for k in range(ctrl.n_total + n_hold):
        d.ctrl[:] = ctrl(min(k, ctrl.n_total))
        mujoco.mj_step(model, d)
        if k == ctrl.n_total:
            rel0 = d.xmat[palm].reshape(3, 3).T @ (d.xpos[ob] - d.xpos[palm])
        if k >= ctrl.n_total and d.ncon:
            for c in d.contact[: d.ncon]:
                b = {model.body(model.geom_bodyid[c.geom1]).name, model.body(model.geom_bodyid[c.geom2]).name}
                if obj in b and len(b) == 2 and any(x.startswith("right_") for x in b):
                    minpen = min(minpen, float(c.dist))
    rel1 = d.xmat[palm].reshape(3, 3).T @ (d.xpos[ob] - d.xpos[palm])
    pads = []
    for c in d.contact[: d.ncon]:
        g1, g2 = model.geom(c.geom1).name, model.geom(c.geom2).name
        b = {model.body(model.geom_bodyid[c.geom1]).name, model.body(model.geom_bodyid[c.geom2]).name}
        if obj in b and len(b) == 2:
            pads.append((g1, g2, int(c.dim), tuple(np.round(c.friction[:3], 4))))
    return {
        "lift_mm": float((d.xpos[ob][2] - z0) * 1000),
        "slip_mm": float(np.linalg.norm(rel1 - rel0) * 1000),
        "pen_mm": float(-minpen * 1000),
        "contacts": pads,
        "finite": bool(np.isfinite(d.qpos).all()),
    }
