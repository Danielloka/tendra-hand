"""Open the Tendra Hand model in the interactive MuJoCo viewer.

    uv run python sim/view.py          # v0: thumb + index, 8 joints
    uv run python sim/view.py --v1     # v1: full hand, 21 joints, tendon-driven

Move the joints with the sliders in the right-hand panel under "Control" (one per motor/servo, in
ID order). In v1 each slider is a servo angle (rad); the servo pulls the joint's tendon loop.
Mouse: left-drag rotates the view, right-drag moves it, scroll zooms. Double-click a part to
select it, then Ctrl + right-drag pushes it.

v1 extras: tendons are drawn red (flex) and blue (ext). Press 3 to toggle the joint drums
(geom group 3); keyframes "open" and "fist" are under Simulation > Key.
"""

import argparse
from pathlib import Path

import mujoco
import mujoco.viewer

MODELS = Path(__file__).resolve().parent / "models"
MODEL_PATH = MODELS / "tendra_hand.xml"
MODEL_V1_PATH = MODELS / "tendra_hand_v1.xml"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--v1", action="store_true", help="open the 21-joint v1 hand")
    args = parser.parse_args()
    path = MODEL_V1_PATH if args.v1 else MODEL_PATH
    model = mujoco.MjModel.from_xml_path(str(path))
    extra = f", {model.ntendon} tendon strands" if model.ntendon else ""
    print(f"Loaded {path.name}: {model.nu} joints{extra}, {model.body_mass.sum() * 1000:.0f} g")
    print("Use the sliders under 'Control' (right panel) to move the joints.")
    mujoco.viewer.launch(model)


if __name__ == "__main__":
    main()
