"""The arm servos are well damped: a wrist-target step must not overshoot (tendra.arm.ARM_KV_SCALE,
research/ai/smooth-control.md; the stock OpenArm gains overshot ~38%)."""

import numpy as np
import pytest
from tendra.scene import GraspScene, SceneConfig


@pytest.mark.parametrize("axis", [0, 1, 2])
def test_wrist_step_overshoot_below_5_percent(axis):
    scene = GraspScene(SceneConfig())
    scene.reset(np.random.default_rng(0), "cylinder", "right")
    scene.step(1.0)
    pos, quat = scene.wrist_pose("right")
    step = 0.014  # two full actions at wrist_speed 0.15 m/s x 0.048 s
    delta = np.zeros(3)
    delta[axis] = step
    scene.set_wrist_target(pos + delta, quat, "right")
    x = []
    for _ in range(100):  # 0.4 s
        scene.step(0.004)
        x.append((scene.wrist_pose("right")[0] - pos)[axis])
    x = np.array(x)
    final = x[-1]
    assert abs(final) > 0.5 * step  # it actually moved
    assert x.max() / final - 1.0 < 0.05 if final > 0 else x.min() / final - 1.0 < 0.05
