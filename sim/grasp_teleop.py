"""Webcam grasp teleoperation: DISABLED until it is ported to the arm scene.

It drove a hand floating over the table (an ideal mocap weld). Since 2026-10-02 the grasp scene
has no floating hand: the wrist is always held by an arm (`tendra.scene.GraspScene`, two arms),
and the webcam's home pose is not reachable for the arm as it was. Porting it means mapping the
webcam wrist pose through `scene.set_wrist_target(pos, quat, side)` (the IK does the rest), with
a reachable home pose, for either hand.

The last working version is in git history (the commit before "Two arms, no floating hand").
"""

import sys


def main() -> int:
    print(__doc__.strip(), file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
