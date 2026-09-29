"""Joint definitions, shared by the firmware protocol, the simulation and the AI code.

There are two hands ("variants"), each described by a `HandSpec`:

- `V0`: the 8-joint prototype (index + thumb) on 28BYJ-48 steppers, firmware config.h.
- `V1`: the full 20-joint hand (4-DOF thumb) on Feetech SCS0009 servos, firmware config_v1.h.

Order = motor order M1..MN = firmware protocol order = MuJoCo actuator order (v1: = servo ID).
Sign convention: positive = closing the hand, 0 = straight. Units: radians.
Must match the firmware config and the MuJoCo models (checked by software/tests).

The module-level names (`JOINT_NAMES`, `NUM_JOINTS`, `JOINT_LIMITS`, `MODEL_PATH`,
`joint_index`) describe v0 and stay as they were, so older code keeps working.
"""

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class HandSpec:
    """Everything the PC side needs to know about one hand variant."""

    name: str  # "v0", "v1": what users type (--hand v1)
    firmware_variant: str | None  # "hand=..." in the firmware's I line (None: v0 sends none)
    joint_names: tuple[str, ...]
    joint_limits: tuple[tuple[float, float], ...]  # (min, max) rad, what the firmware clamps to
    model_path: Path  # MuJoCo model (MJCF)
    firmware_config: Path  # C++ header with the joint table
    servo_ids: tuple[int, ...] = ()  # bus ID per joint; empty = no servos (steppers)

    @property
    def num_joints(self) -> int:
        return len(self.joint_names)

    @property
    def has_feedback(self) -> bool:
        """True if the firmware reports measured positions (servos), not counted steps."""
        return bool(self.servo_ids)

    @property
    def lower(self) -> np.ndarray:
        return np.array([lo for lo, _ in self.joint_limits])

    @property
    def upper(self) -> np.ndarray:
        return np.array([hi for _, hi in self.joint_limits])

    def joint_index(self, joint: int | str) -> int:
        """0-based index from a joint name or a 0-based index."""
        if isinstance(joint, str):
            try:
                return self.joint_names.index(joint)
            except ValueError:
                raise ValueError(f"no joint {joint!r} on hand {self.name}") from None
        if not 0 <= joint < self.num_joints:
            raise IndexError(f"joint index {joint} out of range 0..{self.num_joints - 1}")
        return joint


def _radians(limits_deg) -> tuple[tuple[float, float], ...]:
    return tuple((math.radians(lo), math.radians(hi)) for lo, hi in limits_deg)


# ----- v0: index + thumb, steppers -----

# fmt: off
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
# fmt: on
NUM_JOINTS = len(JOINT_NAMES)

_LIMITS_DEG = [(-5, 95), (-5, 95), (-5, 95), (-15, 15), (-5, 95), (-5, 95), (-13, 80), (-100, 40)]
# (min, max) in radians, the same limits the firmware clamps to.
JOINT_LIMITS: tuple[tuple[float, float], ...] = _radians(_LIMITS_DEG)

MODEL_PATH = REPO_ROOT / "sim" / "models" / "tendra_hand.xml"

V0 = HandSpec(
    name="v0",
    firmware_variant=None,
    joint_names=JOINT_NAMES,
    joint_limits=JOINT_LIMITS,
    model_path=MODEL_PATH,
    firmware_config=REPO_ROOT / "firmware" / "include" / "config.h",
)


# ----- v1: full hand, SCS0009 servos (servo ID = motor number) -----

_FLEX = (-5, 95)
# fmt: off
_V1_JOINTS_DEG: tuple[tuple[str, tuple[int, int]], ...] = (
    ("index_dip", _FLEX),          # ID 1
    ("index_pip", _FLEX),          # ID 2
    ("index_mcp_flex", _FLEX),     # ID 3
    ("index_mcp_abd", (-15, 15)),  # ID 4
    ("thumb_ip", _FLEX),           # ID 5
    ("thumb_mcp_flex", _FLEX),     # ID 6
    ("thumb_cmc_flex", (-13, 80)), # ID 7
    ("thumb_cmc_rot", (-100, 40)), # ID 8
    ("middle_dip", _FLEX),         # ID 9
    ("middle_pip", _FLEX),         # ID 10
    ("middle_mcp_flex", _FLEX),    # ID 11
    ("middle_mcp_abd", (-15, 15)), # ID 12
    ("ring_dip", _FLEX),           # ID 13
    ("ring_pip", _FLEX),           # ID 14
    ("ring_mcp_flex", _FLEX),      # ID 15
    ("ring_mcp_abd", (-15, 15)),   # ID 16
    ("little_dip", _FLEX),         # ID 17
    ("little_pip", _FLEX),         # ID 18
    ("little_mcp_flex", _FLEX),    # ID 19
    ("little_mcp_abd", (-20, 20)), # ID 20
)  # mcp_abd: positive = toward the thumb, for every finger
# fmt: on

V1 = HandSpec(
    name="v1",
    firmware_variant="v1-servo",
    joint_names=tuple(name for name, _ in _V1_JOINTS_DEG),
    joint_limits=_radians(lim for _, lim in _V1_JOINTS_DEG),
    model_path=REPO_ROOT / "sim" / "models" / "tendra_hand_v1.xml",
    firmware_config=REPO_ROOT / "firmware" / "include" / "config_v1.h",
    servo_ids=tuple(range(1, len(_V1_JOINTS_DEG) + 1)),
)

HANDS: dict[str, HandSpec] = {spec.name: spec for spec in (V0, V1)}


def get_hand(hand: "str | HandSpec") -> HandSpec:
    """A `HandSpec` from a name ("v0", "v1") or a spec (returned as is)."""
    if isinstance(hand, HandSpec):
        return hand
    try:
        return HANDS[hand.lower()]
    except KeyError:
        raise ValueError(f"unknown hand {hand!r}, expected one of {sorted(HANDS)}") from None


def joint_index(joint: int | str) -> int:
    """0-based index from a v0 joint name or a 0-based index (use `HandSpec.joint_index`)."""
    return V0.joint_index(joint)
