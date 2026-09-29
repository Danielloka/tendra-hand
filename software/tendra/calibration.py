"""Per-joint calibration kept on the PC and sent to the firmware on connect.

The v0 firmware forgets `K` (scale) and `V` (speed) settings on every reset, so the measured
values live in `calibration_<hand>.json` next to this file and are applied by `apply()`.
"""

import json
from pathlib import Path

from tendra.real_hand import RealHand

CALIBRATION_DIR = Path(__file__).parent


def load(hand_name: str) -> dict | None:
    """Calibration for hand `hand_name` ("v0", "v1"), or None if there is no file."""
    path = CALIBRATION_DIR / f"calibration_{hand_name}.json"
    return json.loads(path.read_text()) if path.exists() else None


def apply(hand: RealHand) -> list[str]:
    """Send the stored calibration to `hand`. Returns a short description of what was set."""
    cal = load(hand.spec.name)
    if cal is None:
        return []
    done = []
    for joint, scale in cal.get("steps_per_rad", {}).items():
        hand.set_scale(joint, scale)
        done.append(f"{joint} {scale:g}/rad")
    if "max_vel" in cal and "max_acc" in cal:
        hand.set_limits(cal["max_vel"], cal["max_acc"])
        done.append(f"speed {cal['max_vel']:g} rad/s")
    return done
