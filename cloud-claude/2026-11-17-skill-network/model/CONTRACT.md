# Stage 0 model contract (V2.1 hand + wrist + forearm on OpenArm)

The interface between the parts of the model pipeline. Change it only together with every consumer.

```
V2.1 CAD study (../2026-11-14-hardware-ideas, read only)
   │  model/extract_v21.py   (needs the `cad` group: uv run --group cad python model/extract_v21.py)
   ▼
model/assets/v21/            generated, committed: kin.json + meshes/ (+ report.json)
   │  model/build_hand.py    (numpy + mujoco only)      ◄── model/actuators.json (physical parameters)
   ▼
model/models/v21_hand.xml    hand + 2-DOF wrist + forearm, root fixed
   │  model/build_robot.py   (+ openarm-mujoco)
   ▼
model/models/v21_robot.xml   OpenArm pedestal, right arm J1–J4, forearm twist, V2.1 hand, cameras, table
model/models/v21_scene.xml   v21_robot + table + a few test objects (for smoke tests)
```

Run everything from the study root (`cloud-claude/2026-11-17-skill-network/`) with `uv run ...`. Generated files
are committed, so the tests and later training need no CAD tools. Every builder has `--check` (exit 1 if the
committed output is stale) and a test calls it.

## Frames and units

- MJCF in **SI** (m, kg, rad). The CAD is in mm and degrees: convert once, in `extract_v21.py`.
- **Hand frame** = the CAD's: origin = wrist centre, **+Z toward the fingers, −Y palm side, +X thumb side.**
  `v21_hand.xml` uses it as its world frame at q = 0 (all joints 0, wrist straight).
- **Sign convention: positive = closing the hand** (flexion toward the palm, abduction toward the thumb,
  `thumb_cmc_rot` + = opposition), exactly as the CAD's `pose(q)` APIs. `wrist_flex` + = toward the palm,
  `wrist_dev` + = toward the thumb (match the CAD's poses `wrist_flex_+40`, `wrist_dev_-20`).

## kin.json (written by extract_v21.py)

```json
{
  "source": {"study": "...", "git_rev": "...", "poses_sha256": "..."},
  "bodies": {
    "<body>": {"parent": "<body>|null", "joint": "<joint>|null",
               "axis": [x,y,z], "pivot_m": [x,y,z],          // hand frame at q = 0, unit axis, + = closing
               "range_rad": [lo, hi],
               "mass_kg": m, "com_m": [..], "inertia": [ixx,iyy,izz,ixy,ixz,iyz],   // about com, hand axes
               "visual_mesh": "meshes/<body>.stl", "collision_meshes": ["meshes/<body>_c0.stl", ...],
               "fit_residual": {"rot_deg": .., "pos_mm": ..}}
  },
  "couplings": {"<f>_dip": {"leader": "<f>_pip", "poly": [c0..c4]},     // rad -> rad, MuJoCo joint equality
                "<f>_bar": {"leader": "<f>_pip", "poly": [c0..c4]}},
  "sites": {"<name>": {"body": "<body>", "pos_m": [..]}}
}
```

Body names (MJCF names equal these): `forearm` (root), `wrist_ring`, `palm`, `<f>_yoke`, `<f>_proximal`,
`<f>_middle`, `<f>_distal`, `<f>_bar` for f in index/middle/ring/little; `thumb_base`, `thumb_metacarpal`,
`thumb_proximal`, `thumb_distal`. CAD bodies that are fixed to another body (`<f>_module`, `thumb:mount`) are
merged into their parent (`palm`). Meshes are stored **in the body frame** (hand frame at q = 0 minus the
body's pivot, no rotation), so `body pos = pivot - parent pivot` and every body quat is identity.

## v21_hand.xml (written by build_hand.py)

- Root body `forearm` with **no joint** (fixed). Everything is attachable with `MjSpec.attach` + a prefix
  (e.g. `right_`): no global `<option>`/`<size>`/`<visual>` that a parent must keep, default classes prefixed
  `v21_`, mesh files found through `meshdir` relative to the XML.
- Joints (hinge, `limited`, rad):
  - 16 servo joints, in this order (= actuator order = servo IDs, same as V1):
    `index_pip, index_mcp_flex, index_mcp_abd, thumb_ip, thumb_mcp_flex, thumb_cmc_flex, thumb_cmc_rot,
    middle_pip, middle_mcp_flex, middle_mcp_abd, ring_pip, ring_mcp_flex, ring_mcp_abd, little_pip,
    little_mcp_flex, little_mcp_abd`;
  - 4 coupled `<f>_dip` and 4 passive `<f>_bar` (driven by joint equalities from `couplings`);
  - `wrist_dev` (forearm → wrist_ring), `wrist_flex` (wrist_ring → palm).
- Actuators: one `position` actuator per servo joint, **named like the joint, in the order above**, then
  `wrist_flex`, `wrist_dev`. `ctrl` = **servo setpoint in joint-angle units** (rad): equal to the joint target
  when unloaded, but it may go beyond the joint range by up to `τ_cap / kp` (≈ 0.6 rad) on each side, limited
  by the servo's real travel, because the tendon stretches and grip force comes from the setpoint error
  (review M1, 2026-11-17). `ctrlrange` says exactly this. Gains, force limits, damping, armature from
  `model/actuators.json`.
- Sensors, **in actuator order**: `jointpos` (`<joint>_pos`, the V2.1 joint-angle sensors) and `actuatorfrc`
  (`<joint>_frc`, a stand-in for the servo current) for the 16 servo joints and the 2 wrist joints. Plus a helper
  in `build_hand.py` (`actuator_qpos_index(model)`) that maps actuator order to qpos addresses, since the
  kinematic tree order differs.
- **No gravity compensation on any hand, wrist or forearm body** (the real servos carry that load).
- Sites: `wrist_centre` (forearm, origin), `forearm_flange` (forearm: centre of the arm-flange mounting face,
  site z axis along the forearm toward the hand), `palm_centre` (palm, centre of the palm surface, z axis out
  of the palm = −Y), `<f>_tip` and `thumb_tip` (centre of each fingertip pad), `wrist_cam_mount` (forearm,
  chosen by build_robot's camera work; may be added by build_robot instead).
- Collision: convex pieces only (every collision mesh convex, ≤ 64 vertices), contype/conaffinity so the hand
  touches objects and the table but not itself across adjacent bodies; the `<f>_bar` bodies never collide.
  Fingertip and palm pads get their own geom class (`v21_pad`, rubber-like friction).
- Must load in **MuJoCo 3.14** and in **MuJoCo Warp 3.14** (`mujoco_warp.put_model`), no unsupported features.

## Robot (build_robot.py)

- OpenArm v2 (`openarm-mujoco==2.3.0`, Apache-2.0, not copied into the repo): pedestal + **right arm** J1–J4,
  named `shoulder_pitch, shoulder_roll, shoulder_yaw, elbow` (OpenArm's signs, as in `software/tendra/arm.py`),
  OpenArm's forearm/wrist/gripper removed, the left arm removed for now (one hand first).
- **`forearm_twist`** hinge between OpenArm's J5 flange and the V2.1 `forearm_flange` site, axis = the forearm's
  long axis, + = pronation. The hand model is attached at that flange with prefix `right_`.
- 7 arm joints (`shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, forearm_twist, wrist_flex, wrist_dev`) +
  16 hand servos, all position actuators; names prefixed `right_`.
- Contacts: object geoms never override the hand's pad settings (no `priority` on objects, or objects with
  explicit pad-compatible condim/friction/solref); a scene test checks a real pad–object contact.
- **Self-contained files**: every mesh the XMLs use lives under `model/assets/` (OpenArm's used meshes vendored
  into `model/assets/openarm/` with its Apache-2.0 LICENSE + NOTICE), all paths relative to the study, so the
  XMLs load from any working directory, any venv location and inside Docker.
- Cameras: `right_wrist_cam` (plain RGB, on the forearm, sees the fingers and the grasp space in front of the
  palm) and `head_cam` (RGB-D, on the pole above and behind the shoulders, sees the table).
- Keyframes: `rest`, `ready`, `table` (hand above the table, palm down, fingers open, **thumb in the CAD `open`
  pose**). The wrist camera housing has a collision geom and never sits below the hand's lowest collision
  point when palm-down.
- Timestep 2 ms. World: OpenArm's (+x forward, +z up).
