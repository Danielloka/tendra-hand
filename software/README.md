# Software

PC-side Python: high-level control and AI.

Planned:
- A `Hand` API with interchangeable backends: `SimHand` (MuJoCo), `RealHand` (USB serial to the ESP32), and a digital-twin mirror mode
- Serial protocol client, calibration tools (steps per radian per joint)
- Trajectory generation, grasp primitives
- Learning-based control (RL / imitation learning), later vision-based grasping

Python 3.12, managed with `uv`.
