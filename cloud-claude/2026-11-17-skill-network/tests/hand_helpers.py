"""Shared helpers for the V2.1 hand tests: load the model, set CAD joint angles, compare body poses with the CAD."""

from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "model" / "models" / "v21_hand.xml"
ASSETS = ROOT / "model" / "assets" / "v21"
POSES_JSON = ROOT.parent / "2026-11-14-hardware-ideas" / "out" / "18" / "poses.json"

SERVO_JOINTS = (
    "index_pip",
    "index_mcp_flex",
    "index_mcp_abd",
    "thumb_ip",
    "thumb_mcp_flex",
    "thumb_cmc_flex",
    "thumb_cmc_rot",
    "middle_pip",
    "middle_mcp_flex",
    "middle_mcp_abd",
    "ring_pip",
    "ring_mcp_flex",
    "ring_mcp_abd",
    "little_pip",
    "little_mcp_flex",
    "little_mcp_abd",
)
WRIST_JOINTS = ("wrist_flex", "wrist_dev")
FINGERS = ("index", "middle", "ring", "little")


@lru_cache(maxsize=1)
def kin() -> dict:
    return json.loads((ASSETS / "kin.json").read_text())


@lru_cache(maxsize=1)
def fk_reference() -> dict:
    return json.loads((ASSETS / "fk_reference.json").read_text())


def build_hand_module():
    """model/build_hand.py as a module (PHYSICS, actuator_qpos_index, collision bits, actuator loader)."""
    import importlib.util
    import sys

    if "build_hand" not in sys.modules:
        spec = importlib.util.spec_from_file_location("build_hand", ROOT / "model" / "build_hand.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules["build_hand"] = mod
        spec.loader.exec_module(mod)
    return sys.modules["build_hand"]


def load_model() -> mujoco.MjModel:
    """The hand model with the scene's physics options (build_hand.PHYSICS), as it is simulated in the robot."""
    m = mujoco.MjModel.from_xml_path(str(XML))
    build_hand_module().apply_physics(m)
    return m


def poly(c: list[float], x: float) -> float:
    return sum(ck * x**k for k, ck in enumerate(c))


def set_cad_q(m: mujoco.MjModel, d: mujoco.MjData, q_deg: dict[str, float]) -> None:
    """qpos from CAD joint angles (deg, missing = 0); DIP and bar from the couplings."""
    d.qpos[:] = 0
    for j in (*SERVO_JOINTS, *WRIST_JOINTS):
        d.qpos[m.joint(j).qposadr[0]] = math.radians(float(q_deg.get(j, 0.0)))
    for j, c in kin()["couplings"].items():
        d.qpos[m.joint(j).qposadr[0]] = poly(c["poly"], d.qpos[m.joint(c["leader"]).qposadr[0]])


def _corners(m: mujoco.MjModel, b: str) -> np.ndarray:
    """Body-frame corners (m) of the body's visual mesh bounding box: where a rotation error shows up most."""
    g = m.geom(f"{b}_vis")
    c, h = m.geom_aabb[g.id][:3], m.geom_aabb[g.id][3:]
    loc = np.array([c + h * np.array([sx, sy, sz]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, m.geom_quat[g.id])
    return loc @ R.reshape(3, 3).T + m.geom_pos[g.id]


def pose_errors(
    m: mujoco.MjModel, d: mujoco.MjData, T_cad: dict[str, list[float]]
) -> dict[str, tuple[float, float]]:
    """Per body (position error mm, rotation error deg) of the MuJoCo body vs the CAD transform.

    Position error = max displacement over the body origin and its mesh bounding-box corners.

    The CAD gives each body's transform from its q = 0 pose (3x4, mm). At q = 0 the MuJoCo body frame sits at the
    body's pivot with identity rotation, so its CAD-predicted pose is (R, R pivot + t).
    """
    mujoco.mj_kinematics(m, d)
    out = {}
    for b, flat in T_cad.items():
        T = np.asarray(flat).reshape(3, 4)
        piv = np.asarray(kin()["bodies"][b]["pivot_m"]) * 1000.0
        p_exp = T[:, :3] @ piv + T[:, 3]
        bid = m.body(b).id
        R = d.xmat[bid].reshape(3, 3)
        cb = _corners(m, b) * 1000.0
        cad_pts = np.vstack([p_exp, (cb + piv) @ T[:, :3].T + T[:, 3]])
        mj_pts = np.vstack([d.xpos[bid] * 1000.0, cb @ R.T + d.xpos[bid] * 1000.0])
        dp = float(np.linalg.norm(cad_pts - mj_pts, axis=1).max())
        E = T[:, :3].T @ R
        # angle from the skew part (robust near 0; the trace/acos form turns 1e-8 rounding into ~0.06 deg)
        s = 0.5 * np.linalg.norm([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
        ang = math.asin(min(1.0, s)) if np.trace(E) > 1.0 else math.pi - math.asin(min(1.0, s))
        out[b] = (dp, math.degrees(ang))
    return out
