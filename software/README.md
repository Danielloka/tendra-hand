# Software

PC-side Python for Tendra Hand: the `tendra` package. It holds everything that talks to the hand, real or simulated, and later the control and AI code.

## The `Hand` API

The same interface works on the simulation and on the real hand, so a script or AI policy written for one runs on the other:

```python
from tendra import RealHand, SimHand

hand = SimHand()                 # MuJoCo simulation
# hand = RealHand()              # real hand; finds the ESP32 on USB automatically

hand.set_joint("index_pip", 0.8)             # radians, positive = closing
hand.set_targets([0.5] * 8)                  # all joints, motor order M1..M8
print(hand.positions())
```

| Module | What it is |
|---|---|
| `tendra/joints.py` | Joint names, order and limits: the single Python source of truth |
| `tendra/hand.py` | `Hand` base class (the interface) |
| `tendra/real_hand.py` | `RealHand`: USB serial to the ESP32 firmware, plus calibration helpers (`zero`, `raw_move`, `set_scale`, `set_limits`) |
| `tendra/sim_hand.py` | `SimHand`: the same API on MuJoCo; `step()`/`wait()` advance the physics |
| `tendra/fake_esp32.py` | Software stand-in for the ESP32, to test without hardware: `RealHand(connection=FakeEsp32())` |

Tests: `uv run pytest` (from the repository root). They also check that the joint names and limits match the firmware (`config.h`) and the MuJoCo model.

## Planned
- Calibration tool (guided, per joint), with results saved to a file
- Trajectory generation and grasp primitives (pinch, point, power grasp)
- Webcam teleoperation, learning-based control, vision-based grasping
