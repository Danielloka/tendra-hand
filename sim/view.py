"""Open the Tendra Hand model in the interactive MuJoCo viewer.

    uv run python sim/view.py          # v0: thumb + index, 8 joints
    uv run python sim/view.py --v1     # v1: full hand, 20 joints, tendon-driven
    uv run python sim/view.py --wrist  # v1 on its forearm: + forearm twist and 2-way wrist
    uv run python sim/view.py --arm    # both arms: OpenArm shoulder + elbow, forearm, wrist, v1 hand
    uv run python sim/view.py --v1 --left   # the mirrored left hand (also with --wrist)
    uv run python sim/view.py --file PATH   # any MJCF/URDF; a URDF gets a motor slider per joint

Move the joints with the sliders in the right-hand panel under "Control" (one per motor/servo, in
ID order). In v1 each slider is a servo angle (rad); the servo pulls the joint's tendon loop.
Mouse: left-drag rotates the view, right-drag moves it, scroll zooms. Double-click a part to
select it, then Ctrl + right-drag pushes it.

v1 extras: tendons are drawn red (flex) and blue (ext). Press 3 to toggle the joint drums
(geom group 3); keyframes "open" and "fist" are under Simulation > Key.

Arms: sliders are named right_/left_ + shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
forearm_rot, wrist_flex, wrist_dev, then the 16 hand servos, per arm. Keyframes "rest", "ready".
"""

import argparse
from pathlib import Path

import mujoco
import mujoco.viewer

MODELS = Path(__file__).resolve().parent / "models"
MODEL_PATH = MODELS / "tendra_hand.xml"
MODEL_V1_PATH = MODELS / "tendra_hand_v1.xml"
MODEL_WRIST_PATH = MODELS / "tendra_hand_v1_wrist.xml"


def load_with_motors(path: Path, kp: float = 10.0) -> mujoco.MjModel:
    """Load a model; if it has no actuators (e.g. a URDF), add a stiff position motor per joint.

    Each motor holds its joint at the slider's angle, so the model stays still until moved.
    Collisions are switched off too: MuJoCo collides meshes as convex hulls, and a palm's hull
    swallows the finger bases, which would jam the joints.
    Joints that copy another joint (URDF "mimic", a joint equality) get no motor of their own.
    """
    spec = mujoco.MjSpec.from_file(str(path))
    if not spec.actuators:
        spec.option.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST  # stable stiff motors
        followers = {eq.name1 for eq in spec.equalities if eq.type == mujoco.mjtEq.mjEQ_JOINT}
        movable = (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
        for joint in spec.joints:
            if joint.type not in movable or joint.name in followers:
                continue
            motor = spec.add_actuator(name=joint.name, target=joint.name)
            motor.trntype = mujoco.mjtTrn.mjTRN_JOINT
            motor.set_to_position(kp=kp, kv=0.1 * kp)
            if joint.range[0] < joint.range[1]:
                motor.ctrlrange = joint.range
                motor.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
    return spec.compile()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    which = parser.add_mutually_exclusive_group()
    which.add_argument("--v1", action="store_true", help="open the 20-joint v1 hand")
    which.add_argument("--wrist", action="store_true", help="v1 on its forearm with a wrist")
    which.add_argument(
        "--arm", action="store_true", help="both arms (OpenArm shoulder and elbow) with v1 hands"
    )
    which.add_argument("--file", type=Path, help="open any MJCF (.xml) or URDF file")
    parser.add_argument("--left", action="store_true", help="the left hand (with --v1 or --wrist)")
    args = parser.parse_args()
    if args.file:
        path = args.file.expanduser().resolve()
        if not path.is_file():
            parser.error(f"file not found: {path}")
        name, model = path.name, load_with_motors(path)
    elif args.arm:
        from tendra.arm import load_arm_model

        name, model = "Tendra arms", load_arm_model()
    else:
        path = MODEL_V1_PATH if args.v1 else MODEL_WRIST_PATH if args.wrist else MODEL_PATH
        if args.left:
            if path == MODEL_PATH:
                parser.error("--left needs --v1 or --wrist")
            path = path.with_name(path.stem + "_left.xml")
        name, model = path.name, mujoco.MjModel.from_xml_path(str(path))
    extra = f", {model.ntendon} tendon strands" if model.ntendon else ""
    print(f"Loaded {name}: {model.nu} actuators{extra}, {model.body_mass.sum() * 1000:.0f} g")
    print("Use the sliders under 'Control' (right panel) to move the joints.")
    mujoco.viewer.launch(model)


if __name__ == "__main__":
    main()
