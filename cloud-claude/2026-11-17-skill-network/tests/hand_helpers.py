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
    "index_pip", "index_mcp_flex", "index_mcp_abd", "thumb_ip", "thumb_mcp_flex", "thumb_cmc_flex", "thumb_cmc_rot",
    "middle_pip", "middle_mcp_flex", "middle_mcp_abd", "ring_pip", "ring_mcp_flex", "ring_mcp_abd", "little_pip",
    "little_mcp_flex", "little_mcp_abd",
)
WRIST_JOINTS = ("wrist_flex", "wrist_dev")
FINGERS = ("index", "middle", "ring", "little")


@lru_cache(maxsize=1)
def kin() -> dict:
    return json.loads((ASSETS / "kin.json").read_text())


@lru_cache(maxsize=1)
def fk_reference() -> dict:
    return json.loads((ASSETS / "fk_reference.json").read_text())


def load_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(XML))


def poly(c: list[float], x: float) -> float:
    return sum(ck * x**k for k, ck in enumerate(c))


def set_cad_q(m: mujoco.MjModel, d: mujoco.MjData, q_deg: dict[str, float]) -> None:
    """qpos from CAD joint angles (deg, missing = 0); DIP and bar from the couplings."""
    d.qpos[:] = 0
    for j in (*SERVO_JOINTS, *WRIST_JOINTS):
        d.qpos[m.joint(j).qposadr[0]] = math.radians(float(q_deg.get(j, 0.0)))
    for j, c in kin()["couplings"].items():
        d.qpos[m.joint(j).qposadr[0]] = poly(c["poly"], d.qpos[m.joint(c["leader"]).qposadr[0]])


def pose_errors(m: mujoco.MjModel, d: mujoco.MjData, T_cad: dict[str, list[float]]) -> dict[str, tuple[float, float]]:
    """Per body (position error mm, rotation error deg) of the MuJoCo body frame vs the CAD transform.

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
        dp = float(np.linalg.norm(d.xpos[bid] * 1000.0 - p_exp))
        R = d.xmat[bid].reshape(3, 3)
        c = (np.trace(T[:, :3].T @ R) - 1) / 2
        out[b] = (dp, math.degrees(math.acos(max(-1.0, min(1.0, c)))))
    return out
