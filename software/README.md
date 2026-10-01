# Software

PC-side Python for Tendra Hand: the `tendra` package. It holds everything that talks to the hand, real or simulated, and later the control and AI code.

## The `Hand` API

The same interface works on the simulation and on the real hand, so a script or AI policy written for one runs on the other:

```python
from tendra import RealHand, SimHand

hand = SimHand()  # MuJoCo simulation
# hand = RealHand()              # real hand; finds the ESP32 on USB automatically

hand.set_joint("index_pip", 0.8)  # radians, positive = closing
hand.set_targets([0.5] * 8)  # all joints, motor order M1..M8
print(hand.positions())
```

## Two hands: v0 and v1

| Variant | Joints | Motors | Firmware config | MuJoCo model |
|---|---|---|---|---|
| `v0` | 8 (index + thumb) | 28BYJ-48 steppers, open loop | `firmware/include/config.h` | `sim/models/tendra_hand.xml` |
| `v1` | 20 (all five fingers, 4-DOF thumb) | Feetech SCS0009 servos, IDs 1–20 = motor order | `firmware/include/config_v1.h` | `sim/models/tendra_hand_v1.xml` |

Each variant is a `HandSpec` in `tendra/joints.py` (`V0`, `V1`, or `get_hand("v1")`): joint names, limits, model path, servo IDs. Every hand object has a `.spec`, so `hand.joint_names` and `hand.num_joints` always fit the hand in use. The old module constants (`JOINT_NAMES`, `NUM_JOINTS`, `JOINT_LIMITS`, `MODEL_PATH`) still describe v0.

```python
from tendra import RealHand, SimHand

sim = SimHand(hand="v1")  # the 20-joint model
hand = RealHand()  # the variant is read from the firmware's "I" line
print(hand.spec.name)  # "v0" or "v1"; RealHand(hand="v1") refuses a v0 board

# v1 only: the servos measure where they are
hand.positions()  # measured positions (S); on v0 these are counted steps
for fb in hand.feedback():  # F: position, load %, temperature, voltage per joint
    print(fb.name, fb.position, fb.load, fb.temperature, fb.voltage, fb.online)
hand.scan_bus()  # B: IDs of the servos that answer, e.g. [1, 2, 3, ...]
hand.info()  # I: per joint scale, zero_ticks, online (for calibration)
hand.offline_mask  # after set_targets: servos that did not take it (bit i = joint i)
```

Offline servos (unplugged, no power) report `nan` in `feedback()` and ignore `set_targets` (see `offline_mask`); `raw_move` (`M`) and `J` commands to them raise `HandError`. v1 servos start limp and switch on at their measured position with the first target, so nothing jumps.

| Module | What it is |
|---|---|
| `tendra/joints.py` | Joint names, order and limits per hand variant (`HandSpec`, `V0`, `V1`): the single Python source of truth |
| `tendra/hand.py` | `Hand` base class (the interface) |
| `tendra/real_hand.py` | `RealHand`: USB serial to the ESP32 firmware, variant auto-detect, v1 feedback (`feedback`, `scan_bus`, `info`), plus calibration helpers (`zero`, `raw_move`, `set_scale`, `set_limits`) |
| `tendra/sim_hand.py` | `SimHand`: the same API on MuJoCo; `step()`/`wait()` advance the physics; `set_positions()` puts the joints at a measured pose (digital-twin mirror) |
| `tendra/retarget.py` | **Retargeting**: 21 human hand landmarks → Tendra joint angles. Fingers by direct angle measurement, thumb by damped-least-squares optimisation on the MuJoCo model (with pinch handling), One Euro smoothing, open-hand calibration. Works for v0 and v1 |
| `tendra/lite_model.py` | `load_lite_model(path)`: the same model with simplified meshes (~15 % of the triangles, masses copied), much faster to draw. `SimHand(..., lite=True)` |
| `tendra/hand_view.py` | `HandView`: draws the hand into an image on demand (offscreen renderer), mirrored, with mouse rotate/zoom, or from a fixed model camera (`camera="view"`). For OpenCV windows |
| `tendra/scene.py` | `GraspScene`: the V1 hand floating over a table (free body `hand_root` welded to a mocap target, the stand-in for an arm), objects, cameras `view` and `wrist`, `set_wrist_from_view` (the mirror mapping), `lifted()` success check, `scripted_grasp()` |
| `tendra/synergy.py` | Hand synergies (eigengrasps): `default_synergies()` (6 human-like patterns: close, oppose, spread, hook, roll, thumb curl), `fit_synergies()` (PCA from recorded postures), `project`/`residual` |
| `tendra/grasp_env.py` | `GraspEnv`: the RL environment on `GraspScene` (actions: wrist velocity + synergies + residual; actor/critic observations; human-like reward; domain randomisation; demo-state resets from scripted grasps or teleop datasets) |
| `tendra/rl/` | `ppo.py` (PPO with asymmetric critic, normalisation, adaptive LR; `Policy` for deployment), `vec_env.py` (envs in worker processes), `train.py` (trainer with object/demo/penalty curriculum, checkpoints, `progress.csv`) |
| `tendra/wrist.py` | `WristTracker`: the operator's wrist pose from the webcam. Hand translation by Gauss-Newton on the pinhole reprojection of MediaPipe's 3D landmarks, orientation from the palm frame, mirrored into the view frame; One Euro smoothing, clutch, gain |
| `tendra/dataset.py` | `EpisodeRecorder` (records the sim state per frame, cheap), `Dataset`, `load_episode`, `render_episode` (offline, by replay), `write_preview`. Format: `info.json`, `model.mjb`, `episodes.jsonl`, `episodes/episode_NNNNNN.npz` |
| `tendra/hand_tracking.py` | Webcam + MediaPipe Hand Landmarker (pretrained network, model downloaded to `~/.cache/tendra/`), drawing helpers |
| `tendra/fake_esp32.py` | Software stand-in for the ESP32, to test without hardware: `RealHand(connection=FakeEsp32())`. v1: `FakeEsp32(hand="v1", offline={9})` emulates the servo firmware, including offline servos and joints moved by hand while limp (`move_by_hand`) |

Tests: `uv run pytest` (from the repository root). They also check that the joint names and limits match the firmware (`config.h`, `config_v1.h`) and the MuJoCo models. The v1 model tests are skipped until `sim/models/tendra_hand_v1.xml` exists.

## Planned
- Calibration tool (guided, per joint), with results saved to a file
- Trajectory generation and grasp primitives (pinch, point, power grasp)
- Learning-based control, vision-based grasping (webcam teleop: done, `sim/teleop.py`)
