"""Joint definitions, shared by the firmware protocol, the simulation and the AI code.

Order = motor order M1..M8 = firmware protocol order = MuJoCo actuator order.
Sign convention: positive = closing the hand, 0 = straight. Units: radians.
Must match firmware/include/config.h and sim/convert.py (checked by software/tests).
"""

import math
from pathlib import Path

JOINT_NAMES: tuple[str, ...] = (
    "index_dip",       # M1
    "index_pip",       # M2
    "index_mcp_flex",  # M3
    "index_mcp_abd",   # M4
    "thumb_ip",        # M5
    "thumb_mcp",       # M6
    "thumb_cmc_flex",  # M7
    "thumb_cmc_rot",   # M8
)
NUM_JOINTS = len(JOINT_NAMES)

_LIMITS_DEG = [(-5, 95), (-5, 95), (-5, 95), (-15, 15), (-5, 95), (-5, 95), (-13, 80), (-100, 40)]
# (min, max) in radians, the same limits the firmware clamps to.
JOINT_LIMITS: tuple[tuple[float, float], ...] = tuple(
    (math.radians(lo), math.radians(hi)) for lo, hi in _LIMITS_DEG
)

REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = REPO_ROOT / "sim" / "models" / "tendra_hand.xml"


def joint_index(joint: int | str) -> int:
    """0-based index from a joint name or a 0-based index."""
    if isinstance(joint, str):
        return JOINT_NAMES.index(joint)
    if not 0 <= joint < NUM_JOINTS:
        raise IndexError(f"joint index {joint} out of range 0..{NUM_JOINTS - 1}")
    return joint
