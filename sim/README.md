# Simulation

**MuJoCo** models and tools for simulating the hand and running its **digital twin** (a simulated copy that mirrors, and can drive, the real hand).

## Quick start

From the repository root (needs [uv](https://docs.astral.sh/uv/)):

```bash
uv sync                          # create the Python 3.12 environment (first time only)
uv run python sim/view.py        # open the hand in the MuJoCo viewer
uv run python sim/view.py --v1   # open the full v1 hand (20 joints, tendon-driven)
uv run python sim/twin.py --fake # digital twin with a software ESP32
uv run python sim/twin.py --port auto   # digital twin driving the real hand
uv run python sim/twin.py --fake --hand v1          # v1 (20 joints) with a software ESP32
uv run python sim/twin.py --port auto --mirror      # v1: the sim follows the real hand
uv run python sim/teleop.py      # webcam teleop: your hand moves the v1 twin (--hand v0, --fake, --port)
uv run python sim/grasp_teleop.py                 # pick up objects with the floating hand, record demos
uv run python sim/export_lerobot.py ~/tendra-data/datasets/grasp-sim --preview   # MP4 previews
uv run --with "lerobot[dataset]" python sim/export_lerobot.py ~/tendra-data/datasets/grasp-sim --repo-id tendra/grasp-sim --out <dir>
uv run python sim/train_grasp.py --run runs/grasp  # RL: the hand learns to grasp by itself (re-run = resume)
uv run python sim/eval_grasp.py --run runs/grasp --watch   # success table / watch / --video / --record
uv run python sim/fit_synergies.py grasp-sim       # your own eigengrasps from teleop demos (for --synergies)
```

`--hand auto|v0|v1` picks the hand (auto: whatever the firmware reports, v0 without a hand). `--mirror` reverses the direction (real → sim): the sim shows the servos' measured positions and nothing is sent to the hand, so you can move the limp fingers by hand and watch. It needs position feedback, so v1 only.

Move the joints with the sliders under **Control** in the right-hand panel.

## Files

| File | What it does |
|---|---|
| `convert.py` | Turns the raw Fusion export (`hardware/robot_description/fusion_export/`) into `models/tendra_hand.xml` and fixes it along the way: PLA masses, recomputed inertias, joint names, sign convention, actuators |
| `models/tendra_hand.xml` | The generated MuJoCo model (**don't edit by hand**; re-run `convert.py`) |
| `convert_v1.py` | Turns the v1 export (`hardware/robot_description/v1_export/`: `hand_v1.json`, `meshes/`, optional `tendon_routes.json`) into `models/tendra_hand_v1.xml`: bodies, closing-positive axis signs, PLA masses, 42 tendon strands, 21 servo actuators |
| `models/tendra_hand_v1.xml` | The generated v1 model (**don't edit by hand**; re-run `convert_v1.py`) |
| `view.py` | Interactive viewer (`--v1` for the full hand) |
| `twin.py` | **Digital twin**: slider targets are sent to the real hand (rate-limited, only on change); the terminal shows real-vs-sim difference. v1 adds `--mirror` (real → sim) |
| `teleop.py` | **Webcam teleoperation**: MediaPipe hand tracking → `tendra.retarget` → the twin (and optionally the real hand). One window: camera image + the hand, drawn only when it moves (≤ 30 fps). Defaults are tuned for a slow laptop: kinematic mode (no servo delay; `--physics` for the tendon sim), simplified meshes, no shadows, tendons hidden (`--tendons`), MuJoCo's viewer only with `--viewer`. Mouse on the hand: drag rotates, wheel zooms. Prints a speed summary on exit. Keys in the camera window: C calibrate (hand open and flat), SPACE pause, M flip palm side, Q quit |
| `grasp_teleop.py` | **Grasp teleoperation + demo recording.** The floating V1 hand (`tendra.scene`) over a table with a cylinder, cube or ball. Your wrist pose (`tendra.wrist`) moves the hand, your fingers (`tendra.retarget`) close it. Records episodes (`tendra.dataset`) to `~/tendra-data/datasets/<name>`; success = object held up for 1 s. Keys: R record/stop, X discard, N new round, W wrist clutch, C calibrate, SPACE pause, M flip, Q quit |
| `export_lerobot.py` | Renders recorded episodes offline (replaying the saved sim state) and exports them to a Hugging Face **LeRobot** dataset (v3.0 format, lerobot 0.6.1, run with `uv run --with "lerobot[dataset]"`), or only MP4 previews (`--preview`). Successful episodes only unless `--all` |
| `train_grasp.py` | **Reinforcement learning:** PPO trains the floating V1 hand to grasp and lift (`tendra.grasp_env`, `tendra.rl`): synergy actions, human-like rewards, demo-state starts, object and penalty curriculum, domain randomisation, parallel worker processes. Writes `runs/<name>/` (`progress.csv`, `latest.pt`, `best.pt`); `--smoke` for a 30 s check. Design and research: `research/ai/grasp-rl.md` |
| `eval_grasp.py` | Evaluates a trained policy on fixed spawns (success, lift, time, smoothness, effort, synergy residual); `--watch` live window, `--video out.mp4`, `--record <name>` saves successful rollouts as a teleop-format dataset (RL teacher → imitation student) |
| `fit_synergies.py` | PCA "eigengrasps" from recorded teleop datasets → `.npz` for `train_grasp.py --synergies` |
| `tests/` | Model sanity checks, twin, grasp pipeline end to end (`test_grasp_teleop.py`): `uv run pytest` |

After a new Fusion export, or a change to `convert.py` / `convert_v1.py`:
```bash
uv run python sim/convert.py && uv run pytest       # v0
uv run python sim/convert_v1.py && uv run pytest    # v1
```
The tests fail if a generated model is out of date.

## Model conventions

- Joints and actuators are listed in **motor order** (M1…M8): `index_dip, index_pip, index_mcp_flex, index_mcp_abd, thumb_ip, thumb_mcp, thumb_cmc_flex, thumb_cmc_rot`
- **Positive = closing the hand**, and 0 = straight. Angles are in radians.
- Actuators are simple position controllers standing in for the stepper + tendon, and will be tuned against the real hand later.

## v1 model (full hand, tendon-driven)

- **20 joints, 16 actuators** in **servo-ID order** (same as `firmware/include/config_v1.h`): `index_pip, index_mcp_flex, index_mcp_abd, thumb_ip, thumb_mcp_flex, thumb_cmc_flex, thumb_cmc_rot`, then middle, ring, little (`pip, mcp_flex, mcp_abd` each). Actuator names = joint names. The four finger DIPs have no actuator: each follows its PIP (DIP = 0.75 × PIP) through a joint equality `<f>_dip_coupling`.
- **Sign convention** (positive = closing), worked out automatically from the geometry, because Fusion's axis signs are arbitrary: finger flexion bends toward the palm (-Y); finger `mcp_abd` moves toward the thumb (+X); `thumb_cmc_rot` swings the thumb across the palm (-X); thumb flexion (`cmc_flex`, `mcp_flex`, `ip`) curls it toward the fingers (+Z).
- **Tendons.** Every servo joint has one loop of two strands, `<joint>_flex` and `<joint>_ext` (32 strands). The loop wraps a drum on the joint's child segment (7 mm for finger `mcp_flex`, 7.5 mm for `thumb_cmc_rot`, 6 mm otherwise) and is tied to it, so the joint changes the strand length by exactly the drum radius per radian: the flex strand shortens and the ext strand lengthens by the same amount, and the loop stays tight. Strands of more distal joints cross every joint they pass **exactly on its axis**, so those joints don't change their length (no coupling). In the palm and forearm the strands follow `tendon_routes.json` down to the spool tangent points. Each DIP has a passive **coupling loop** (`<f>_dip_flex`, `<f>_dip_ext`, 8 strands): tied in the proximal phalanx, around a 4.5 mm hub on the PIP axis (opposite side), crossed in the middle phalanx, around the 6 mm DIP drum. Its length is constant when DIP = 4.5 / 6 × PIP (tested), which is the real hand's coupling. MuJoCo draws flex strands red and ext strands blue; press 3 in the viewer to show the drums.
- **Actuation.** One actuator per servo pulls its joint's flex strand. `ctrl` is the **joint angle in radians** (0 = joint straight); the servo turns `servo_per_joint` = drum / 5 mm spool times that (1.2–1.5). The gains are SCS0009-like: kp 2 N·m/rad, torque limit 0.095 N·m at the servo (half of stall, ≈19 N of tendon force), both scaled to the joint by the ratio. MuJoCo tendons can push as well as pull, so one actuator on the flex strand stands in for the whole loop. Pretension, friction and line stretch are not modelled.
- Keyframes `open` and `fist`.
- **Known limits:** collisions use convex hulls. Palm ↔ thumb_metacarpal is excluded because the palm's hull fills the pocket for the thumb base. The real parts only meet near `thumb_cmc_flex` = 80°, which the sim therefore misses. Toward a straight neighbour, a finger can adduct only about 4–5° before touching it (8 mm gap). That is real geometry.

NVIDIA Isaac Sim/Lab may be added later on a machine with an NVIDIA GPU.
