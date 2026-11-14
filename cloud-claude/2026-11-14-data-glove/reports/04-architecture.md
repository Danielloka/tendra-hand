# 04 — Concepts, design, accuracy and calibration (lead design agent, Opus)

No web searches; part details were from memory and were later checked in report 05.

## Concepts compared

| Concept | Cheap | Slim | Accurate | Task freedom | Verdict |
|---|---|---|---|---|---|
| A. Magnetic dorsal skin (magnet + 3D Hall across each joint) | ★★★ | ★★★ | ★★ (3–5°) | ★★★ | **chosen** |
| B. Reverse-tendon strings to spring spools | ★★★ | ★★ | ★ | ★★ | rejected: strings cross several joints with skin-dependent moment arms, so no clean mapping to robot servo space |
| C. Minimal exoskeleton with Tendra's kinematics | ★★ | ★ | ★★★ | ★ | too bulky for everyday tasks |
| D. Camera first + sparse glove | ★★★ | ★★★ | ★ | ★★★ | its useful part (camera auto-calibration) is merged into A |

## Mapping to Tendra V1

MCP flex + abd: one 3D sensor on the hand plate per finger. PIP: sensor on the proximal ring, magnet on the middle ring. DIP: still measured (magnet on a press-on nail) for fingertip matching. Thumb IP / MCP: Hall pairs. Thumb CMC: IMU pair (relative orientation). Thumb 5→4 DOF: solve thumb-tip position with `ThumbSolver`.

## Error budget (per joint, after calibration)

Sensor noise ~0.2°; neighbour magnets ~1° after regression; Earth/steel detected by a reference sensor; **ring slip 3–5° (dominant)**; calibration residual ~2° → MCP 3–4°, abd 3°, PIP 3–4°, DIP 4–5°, thumb MCP/IP 4°, thumb CMC 5–7°; fingertip 6–9 mm.

## Calibration

Flat-hand zero (2 s) → printed 0/30/60/90° jig (first time) → thumb-to-fingertip pinches → 60 s free motion fitted to camera hand tracking (Huber loss); per-user model stored; raw fields always logged. During recording a per-joint Kalman filter uses vision to remove slow slip bias.

## Pipeline

ESP32 raw fields 200 Hz → calibration → human angles → fusion → retargeting (abd copy with limits, MCP copy, PIP by fingertip matching under the DIP coupling, thumb by `ThumbSolver`) → `tendra.dataset` → LeRobot v3. Image gap: segment the human hand and render the MuJoCo Tendra hand in its place.

## Phases and first tests

P0 one finger (~250 NOK): ring slip (< 1 mm after 50 fists), steel pan/knife at 5/3/1 cm, neighbour crosstalk, accuracy on held-out jig poses (< 3°), comfort over 30 min of chores, LED/tap sync (< 5 ms).
