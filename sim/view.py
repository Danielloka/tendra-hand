"""Open the Tendra Hand model in the interactive MuJoCo viewer.

    uv run python sim/view.py

Move the joints with the sliders in the right-hand panel under "Control" (one per motor, M1..M8).
Mouse: left-drag rotates the view, right-drag moves it, scroll zooms. Double-click a part to
select it, then Ctrl + right-drag pushes it.
"""

from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = Path(__file__).resolve().parent / "models" / "tendra_hand.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    print(f"Loaded {MODEL_PATH.name}: {model.nu} joints, {model.body_mass.sum() * 1000:.0f} g")
    print("Use the sliders under 'Control' (right panel) to move the joints.")
    mujoco.viewer.launch(model)


if __name__ == "__main__":
    main()
