"""Open the Tendra Hand model in the interactive MuJoCo viewer.

    uv run python sim/view.py          # v0: thumb + index, 8 joints
    uv run python sim/view.py --v1     # v1: full hand, 20 joints, tendon-driven
    uv run python sim/view.py --wrist  # v1 on its forearm: + forearm twist and 2-way wrist
    uv run python sim/view.py --arm    # OpenArm shoulder + elbow, then forearm, wrist, v1 hand

Move the joints with the sliders in the right-hand panel under "Control" (one per motor/servo, in
ID order). In v1 each slider is a servo angle (rad); the servo pulls the joint's tendon loop.
Mouse: left-drag rotates the view, right-drag moves it, scroll zooms. Double-click a part to
select it, then Ctrl + right-drag pushes it.

v1 extras: tendons are drawn red (flex) and blue (ext). Press 3 to toggle the joint drums
(geom group 3); keyframes "open" and "fist" are under Simulation > Key.

Arm: the first 7 sliders are the arm (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
forearm_rot, wrist_flex, wrist_dev), then the 16 hand servos. Keyframes "rest" and "ready".
"""

import argparse
from pathlib import Path

import mujoco
import mujoco.viewer

MODELS = Path(__file__).resolve().parent / "models"
MODEL_PATH = MODELS / "tendra_hand.xml"
MODEL_V1_PATH = MODELS / "tendra_hand_v1.xml"
MODEL_WRIST_PATH = MODELS / "tendra_hand_v1_wrist.xml"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    which = parser.add_mutually_exclusive_group()
    which.add_argument("--v1", action="store_true", help="open the 20-joint v1 hand")
    which.add_argument("--wrist", action="store_true", help="v1 on its forearm with a wrist")
    which.add_argument("--arm", action="store_true", help="v1 on the OpenArm shoulder and elbow")
    args = parser.parse_args()
    if args.arm:
        from tendra.arm import load_arm_model

        name, model = "Tendra arm", load_arm_model()
    else:
        path = MODEL_V1_PATH if args.v1 else MODEL_WRIST_PATH if args.wrist else MODEL_PATH
        name, model = path.name, mujoco.MjModel.from_xml_path(str(path))
    extra = f", {model.ntendon} tendon strands" if model.ntendon else ""
    print(f"Loaded {name}: {model.nu} actuators{extra}, {model.body_mass.sum() * 1000:.0f} g")
    print("Use the sliders under 'Control' (right panel) to move the joints.")
    mujoco.viewer.launch(model)


if __name__ == "__main__":
    main()
