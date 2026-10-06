# CLAUDE.md — Project context for Claude

> Keep this file up to date. When hardware, pins, conventions or decisions change, update it in the same step.

## Project

- **Name:** **Tendra Hand** (chosen 2026-09-26). Note: an unrelated open-source project "Tendra H1" (github.com/aymankhayat/tendra-h1-robotic-hand) already uses the name. The owner chose to keep it; revisit before registering a company or trademark.
- **Ultimate goal (owner, 2026-09-28):** a robot that can do what humans do (cook, do chores, use tools) **as well as a human**, with the **hand as the focus**. Not just picking things up, but understanding and carrying out complex, multi-step tasks. Path: hand → senses → learned skills → task understanding (VLA models, planning) → arm and body in real homes (`docs/roadmap.md`).
- **First body (owner, 2026-09-28):** two arms with two Tendra hands on a fixed pole (bimanual station). AI research hub: `research/ai/` (vision + layered AI architecture, staged roadmap, maths, resources, ideas). Keep it current.
- **State-of-the-art hand research** (2026-09-30): `research/references/hands/` (README = overview + prioritised plan; the main gap vs 1X NEO and others is fingertip force, ~3.4 N vs 45–80 N). Check it before making hand design decisions.
- **First step:** an open-source, tendon-driven, 3D-printed robotic hand with the same DOF as a human hand. The owner wants to become a robotics entrepreneur; this repo is the main research hub and project-management center.
- **Everything is open source:** CAD/3D files, firmware, software, AI models, research. It is published in a public GitHub repo and on a project website.
- **Language:** all docs, code and comments in English.
- **Hand versions (owner, 2026-09-28):** **V1** = the full 5-finger hand, 20 joints on 16 SCS0009 servos (the real first version). **V0** = the thumb + index stepper build, "just a start" (prototype). In code: `v0` / `v1` (firmware envs `tendra_s3` = V0, `hand_v1_servo` = V1; `tendra.joints.V0` / `V1`; `--hand v0|v1`).
- **Current phase:** Phase 1 (V0: make the thumb and index finger move smoothly and cleanly) and Phase 2 (build Tendra Hand V1, the first full hand; designed, not built). See `docs/roadmap.md`. Roadmap order changed 2026-09-28: V1 is Phase 2, right after V0.

## Working with the owner

- They are **learning**, so briefly explain key concepts (what and why) as you go, without lecturing.
- **Ask before big structural changes**: moving or renaming folders, changing the robot model, changing protocols, adding licenses, pushing to GitHub.
- Work **step by step**. After each step, say what you did, what you verified, and what's next.
- Hardware safety: never write firmware that moves motors on boot without a command. Keep coil current and power limits in mind.
- Log findings, experiments and decisions in `research/log.md` (dated entries).
- **Agents and subagents are allowed** (owner, 2026-09-29): Claude may spawn agents/subagents (e.g. Explore, Plan, general-purpose, forks, and the project agents in `.claude/agents/`) whenever they help, such as parallel research, broad codebase searches, or reviews, without asking first. The rules above still apply to their work.
- **Model choice for agents** (owner, 2026-10-02): the main session runs on **Opus** (the owner starts it that way). Agents, subagents and agent-team members should use **Sonnet for most tasks** (searches, reading, routine edits, tests, docs, website checks) and **Opus only when needed** (hard design or maths decisions, tricky debugging, architecture, safety-critical firmware, anything where a wrong answer is costly). Set `model: "sonnet"` or `"opus"` on the Agent call accordingly; default to Sonnet.

## Owner's machine

| Item | Value |
|---|---|
| OS | Windows 11 Pro Education (Norwegian locale) |
| CPU / RAM | Intel i3-1125G4 (4 cores), 16 GB |
| GPU | Intel UHD (integrated). **No NVIDIA**, so Isaac Sim/Lab can't run locally; use MuJoCo on CPU, and cloud GPUs for heavy training later |
| Python | system 3.14.6; `uv` installed. Project uses a **uv-managed venv with Python 3.12** for library compatibility |
| Firmware tools | **PlatformIO** with the Arduino framework: VS Code extension (owner) + PlatformIO Core 6.2 CLI via `uv tool` (`pio`, used by Claude to build and flash) |
| Animation effects | **Off** in Windows (checked 2026-09-28), so browsers report `prefers-reduced-motion: reduce` and the website shows its static, no-motion version. Use the homepage's "Play animation" button (remembered per browser), or turn Animation effects on in Settings → Accessibility → Visual effects. |
| Smart App Control | Turned **off** (2026-09-26); it had blocked MuJoCo's DLLs. Verified: MuJoCo 3.14.0 imports on Python 3.12 via uv. |

Shell notes: use forward slashes / Git Bash syntax in the Bash tool. The project path has no spaces.
Tidy root: `.git`, `.venv`, `.gitignore` and `.gitattributes` have the Windows *hidden* attribute. pytest/ruff caches live in `.venv/` (set in `pyproject.toml`). Keep new tool caches out of the root too.

## Hardware (V0 prototype: thumb + index)

- 3D-printed rigid skeleton, PLA/PETG. TPU fingertip pads are planned.
- **Tendon-driven.** Each joint is driven independently by **one motor that both flexes and extends** it (an antagonistic tendon loop on one spool).
- **Tendon routing:** each joint's own loop wraps a drum on the child segment (PIP drum groove radius ≈ 6 mm, from the STEP) and is tied through a hole in it. Loops for more distal joints pass through slots near the joint axes; open slots cause coupling, so pass-through tendons should cross each joint **exactly on its axis** (owner's choice, 2026-09-27). Done for the whole index in Fusion (PIP by `TendraPipPinch`, MCP flex + sideways by `TendraIndexPinch`, sideways slit edges filleted 0.4 mm; not yet printed); the thumb is still open. Tendon: 0.4 mm fishing line.
- Motor spool: **radius 1 cm** (diameter 2 cm) on the printed prototype; a 5 mm spool (2× pull, `hardware/print/v0/spool_r5.stl`, from `hardware/cad/spool_v0.py`) was made on 2026-09-29 for the index knuckle, which extends hard and grips weakly, so one output revolution pulls ≈62.8 mm of tendon, or ≈0.031 mm per full step. The joint-side moment arm is unknown, so steps-per-radian is **calibrated empirically** per joint.
- **Actuators now:** 8× **28BYJ-48 (5 V)** unipolar steppers + **ULN2003** driver boards.
  - Gear ratio ≈ 63.68:1, so **≈2038 steps/output rev (full-step)** and ≈4076 (half-step). Top speed ≈ 15 rpm.
  - Open loop, with no position feedback. **Homing is manual:** before power-up the owner straightens every joint, and that pose is `q = 0`.
- **Power:** separate **5 V / 2 A** supply for the motors. Each energised 28BYJ-48 draws ~200–250 mA per phase, so 8 motors is close to the limit. Firmware should **release coils of idle motors** and avoid 2-phase-on for all 8 at once.
- **Controller:** **ESP32-S3-N16R8** (16 MB quad flash, **8 MB octal PSRAM**), connected to the PC over **USB-C (native USB CDC)**.
- **Planned upgrade:** **Feetech SCS0009** smart servos (half-duplex TTL bus, daisy-chained, addressable IDs, 5 V, position feedback), via the **FE-URT-1** signal converter (owner has it). This fixes homing, feedback and the pin-count problem.

## Tendra Hand V1 (designed 2026-09-28, not built yet)

- **20 joints / 16 SCS0009 (since 2026-10-01):** 4 fingers × (`mcp_abd`, `mcp_flex`, `pip`, `dip`) + a **4-DOF thumb** (`cmc_rot`, `cmc_flex`, `mcp_flex`, `ip`; the owner had the 5th thumb joint `thumb_mcp_abd` removed on 2026-09-29). **Each finger's DIP is coupled to its PIP** (DIP ≈ 0.75 × PIP, owner's decision 2026-10-01; mechanism: a rigid bar, chosen 2026-10-02), so it has no servo; the thumb IP stays independent. v0's `thumb_mcp` is `thumb_mcp_flex` in v1. Servo ID = protocol order = MuJoCo actuator order: 1–3 index (pip, mcp_flex, mcp_abd), 4–7 thumb (ip, mcp_flex, cmc_flex, cmc_rot), 8–10 middle, 11–13 ring, 14–16 little (each pip, mcp_flex, mcp_abd). Firmware 0.4.0 (`RealHand` refuses the old 20-servo 0.3.x). Source: `firmware/include/config_v1.h`; Python: `tendra.joints.V1` (`couplings`, `all_joint_names`, `expand()`).
- **Fusion design "Tendra Hand V1"** (cloud copy of "Hand assebly", which stays untouched), built by `hardware/cad/fusion_scripts/TendraHandV1/` in stages. The fingers are index copies with human length ratios; the thumb keeps the prototype's one-piece proximal phalanx (stage `thumb_unhinge` removed the 5th-DOF hinge built earlier).
- **Tendon routing:** every servo joint = one antagonistic loop (flex + ext strand) on a drum on the child segment and a **5 mm spool** on its servo (same spool on all servos). Drums: finger `mcp_flex` **7 mm** (14 mm groove, the most that fits), `thumb_cmc_rot` 7.5 mm, all others 6 mm, so `servo_per_joint` = drum / spool = 1.4 / 1.5 / 1.2. **DIP coupling (linkage, 2026-10-02):** one rigid bar per finger crosses the middle phalanx diagonally: pin A on the proximal phalanx ≈ 3.5 mm from the PIP axis (back side), pin B on the distal phalanx ≈ 4.5 mm from the DIP axis (palm side); bar 17.7–24.9 mm per finger. Pin positions per finger from `hardware/cad/dip_linkage.py` (searched within the phalanx, axis clearance ≥ 2 mm, no toggle), written into `tendon_routes.json` (`coupling.linkages`); DIP stays within 0.6° of 0.75 × PIP over −5…95°. Chosen over a coupling tendon: no tensioning, no creep or slack (the DIP has no servo to take it up). **Modelled in Fusion (2026-10-03, stage `linkage`):** per finger two plates (on the proximal and distal phalanx, −x side, with pegs) and the bar outside them; interference check against the finger parts: none. Not yet exported or printed. Each of the 32 servo strands has its own channel: 1.2 mm bore for 6 mm at the entry, then 2.2 mm for a 1 × 2 mm PTFE tube, S-curves with bend radius ≥ 15 mm, through the palm and a 12 mm wrist plate, then a straight line down to its spool. Single source of truth: `hardware/cad/tendon_router.py` → `v1_export/tendon_routes.json` (tested in `hardware/cad/tests/`); the CAD channels and the MuJoCo tendons both read it.
- **Forearm** (under the palm, z −153…−30): servo shafts point inward, two levels × front/back (the third level is now empty); deeper levels sit closer to the centre so no strand is blocked. Fingers: `mcp_flex` front level 1, `pip` front level 2, `mcp_abd` back level 2 (the DIP servos' old slot, for a gentler S-curve; min bend 17 mm). Room for the ESP32-S3 + FE-URT-1 on level 3.
- **Power:** 16 servos need a 5–6 V supply of ≥ 15 A (e.g. Mean Well LRS-100-5), split into 3 branches with capacitors; the 5 V / 2 A supply is only for v0. SCS0009 backs off by itself after 2 s above 80% load, so keep loop pretension low.
- **Firmware:** env `hand_v1_servo` (v0 stays `tendra_s3`, the default). FE-URT-1 on UART1: ESP32 TX GPIO 17 → URT TX, RX GPIO 18 → URT RX (not crossed), 1 Mbps. Torque off at boot, nothing moves before a command. Extra commands `F` (feedback) and `B` (bus scan). PSRAM could be re-enabled for v1 (no motor pins on 35–37), but isn't yet.
- **Known limits:** fingers adduct only ~4–5° toward a straight neighbour; index strands bend up to 56° entering the palm (they must pass beside the thumb bay); the thumb's routing is being reworked (2026-09-29, concept A: base plate 4 mm lower, hollow journal with PTFE sheaths up the rot axis to a boss on the metacarpal, cmc_rot drum r 7.5 so **`thumb_cmc_rot` servo_per_joint = 1.5** (5 mm spool), thumb servos re-slotted; router + tests done; base and the metacarpal's cmc_flex drum built in Fusion; palm/forearm need a rebuild for the 16-servo layout; how the sheaths enter the metacarpal is still open (its front half is too thin); see `research/experiments/2026-09-29-thumb-routing/`).
- **Sensing:** none yet. A camera for vision-based grasping comes later.

### Motor ↔ joint ↔ pin map

Sign convention: **positive = closing the hand, negative = opening.** `q = 0` means the joint is straight (manual homing pose).
- Flexion joints: positive bends toward the palm side (−Y in the model).
- `thumb_cmc_rot`: positive swings the thumb across the palm (opposition, toward −X where the other fingers will be). Range −100°…+40°. At 0° the thumb points straight out of the palm.
- `index_mcp_abd`: positive moves the index toward the thumb side (+X).
- The physical motor direction for "positive" must be verified per motor during calibration.

| Motor | IN1 IN2 IN3 IN4 | Joint (new name) | URDF original | Anatomy |
|---|---|---|---|---|
| 1 | 4, 5, 6, 7 | `index_dip` | Revolute 4 | Index fingertip joint |
| 2 | 15, 16, 17, 18 | `index_pip` | Revolute 3 | Index middle joint |
| 3 | 8, 14, **46**, 9 | `index_mcp_flex` | Revolute 2 | Index knuckle bend |
| 4 | 10, 11, 12, 13 | `index_mcp_abd` | Revolute 1 | Index sideways |
| 5 | 1, 2, 42, 41 | `thumb_ip` | Revolute 8 | Thumb tip joint |
| 6 | 40, 39, 38, **37** | `thumb_mcp` | Revolute 7 | Thumb 2nd joint from tip |
| 7 | **36, 35, 0**, 45 | `thumb_cmc_flex` | Revolute 6 | Thumb 3rd joint from tip |
| 8 | **48**, 47, 21, **43** | `thumb_cmc_rot` | Revolute 5 | Thumb base rotation ("other direction") |

The mapping and the joint directions were confirmed by the owner in the MuJoCo viewer (2026-09-26).

### ⚠️ Pin constraints (ESP32-S3-N16R8)

- **GPIO 35, 36, 37:** wired to the **octal PSRAM** on N16R8. They only work as motor pins if **PSRAM is disabled** in the firmware build. Never enable PSRAM with this pin map. (Planned fix: servo bus.)
- **GPIO 0:** strapping pin; LOW at reset means download mode. The ULN2003 input pulls it low, so the board may fail to boot normally with motor 7 connected. Watch for this.
- **GPIO 45, 46:** strapping pins, but LOW is their default, so they're OK (46 = motor 3 IN3 since 2026-09-29; GPIO 3 is no longer used).
- **GPIO 43:** UART0 TX. The boot ROM prints here, so motor 8 may twitch at reset. Firmware must not log on UART0 (use USB CDC).
- **GPIO 48:** often the onboard RGB LED. Don't drive the LED in firmware.
- **GPIO 19/20:** native USB. Never use them for motors.
- All motor outputs must be driven LOW (coils off) as the very first thing in `setup()`.

## Repository layout

```
hardware/   cad/ (f3d, STEP, fusion_scripts/ = Fusion API scripts, tendon_router.py + tests/) · print/ (STL) · electronics/ · robot_description/fusion_export/ (raw Fusion URDF export), v1_export/ (v1 STLs, joints, tendon routes)
firmware/   PlatformIO project for the ESP32-S3
sim/        MuJoCo models, URDF→MJCF converter, digital twin
software/   PC-side Python: Hand API, control, calibration, AI
docs/       roadmap and guides
research/   log.md, experiments, references
website/    project website (Next.js app + design system)
media/      photos, videos, screenshots, diagrams + catalog.json (the one place for project pictures)
```

Every folder has a README explaining what goes there. Keep them current.

## Media (photos, videos, screenshots)

- **All project pictures live in `media/`** (added 2026-09-29): `photos/`, `videos/`, `screenshots/`, `diagrams/`, described in `media/catalog.json`. Rules and fields: `media/README.md`.
- The owner drops raw files in `media/inbox/` (git-ignored). "Process the media inbox" means: run `uv run python media/process_inbox.py` (upright, ≤ 2000 px, **EXIF/GPS stripped**, date-named, catalog entry with `todo`), then **look at each file** with Read, rename it to `YYYY-MM-DD-short-subject.ext`, fill in `description`, `alt`, `caption`, `tags`, `hand`, remove `todo`, and suggest where to use it (e.g. which `gallery.json` placeholder it replaces).
- To find a picture, search `catalog.json` (descriptions, tags) rather than opening files. When a picture is used somewhere, add that file to its `used_in`.
- The website gets a copy in `website/public/media/` (git-ignored, copied by `scripts/sync-styleguide.mjs` on `npm run dev`/`build`), served at `/media/<folder>/<file>`. Repo Markdown links use relative paths to `media/`.
- Never commit media with GPS data; no faces or private homes without permission. Media is CC BY 4.0.

## Robot model

- Raw export location: `hardware/robot_description/fusion_export/` (moved from the root `Hand_description/` on 2026-09-26, with the owner's approval).
- Source of truth: the owner's **Fusion 360** design. It is exported with the **fusion2urdf** plugin (a ROS 1 package: xacro, `package://` paths, Gazebo files).
- The raw export is kept **unedited** so it can be re-exported at any time. A **conversion script** produces the clean models (renamed joints, flexion-positive axes, fixed mass/inertia, MuJoCo MJCF with tendons/actuators). Put fixes in the script, not in the raw export.
- Known export problems (found 2026-09-26):
  - Material is **steel** (7850 kg/m³, total 821 g); it should be PLA/PETG.
  - Inertias are rounded to 1e-6 and two are invalid (`Indexfix_1`, `Indextip_1`). Recompute them from meshes.
  - Placeholder effort/velocity limits of 100.
  - Joints 7 and 8 have opposite axis signs.
  - `Indexfix_1.stl` has 2 open edges.
  - The export's `LICENSE` and `package.xml` are fusion2urdf template leftovers, not the owner's.
- Joint limits in the URDF are **close to the real hand's** (owner-confirmed).
- **Generated model:** `sim/models/tendra_hand.xml`, produced by `uv run python sim/convert.py`. It is committed so people can use it without running the converter. After changing `convert.py` or the export, re-run it; `uv run pytest` fails if the model is stale.
  - PLA density 1240 kg/m³ (solid), 131 g total. Printed parts are lighter; update after weighing.
  - Inertia computed from the meshes (`exact`; `legacy` for the defective `Indexfix_1`).
  - Position actuators in motor order, kp 0.5 N·m/rad, ±0.05 N·m (placeholders until system identification).
  - Contact excludes: palm ↔ direct children (the palm is welded to the world, so MuJoCo's parent filter doesn't apply), and palm ↔ thumb_metacarpal (a convex-hull artifact). Proper fix: convex decomposition of the palm.
- DOF now: 8 (index 4, thumb 4). A human thumb has 5, so the thumb design needs a revisit before the full hand.
- **v1 model:** `sim/models/tendra_hand_v1.xml`, generated by `uv run python sim/convert_v1.py` from `hardware/robot_description/v1_export/` (STL per part + `hand_v1.json` + `tendon_routes.json`, all exported by the Fusion script). 20 joints, 16 actuators, 36 spatial tendons (32 servo strands + 4 DIP coupling bars); one actuator per servo on its flex strand (ctrl = joint angle; the servo angle is `servo_per_joint` × that: 1.4 mcp_flex, 1.5 `thumb_cmc_rot`, 1.2 others). DIP coupling = a joint equality with the linkage's fitted DIP(PIP) quartic for the physics; the bar is a passive tendon `<f>_dip_link` between its two pin sites (36 tendons in all), and a test proves it keeps its designed length along that curve. Until Fusion is re-exported, the forearm meshes are the old 20-servo ones (DIP servos left out; cosmetic). Parts that are not joint children (palm, forearm, servos, spools) are welded to the world. `sim/tests/test_v1.py` checks moment arms, coupling, tracking, signs, limits vs `config_v1.h`.
- **Wrist model** (2026-10-02): `sim/models/tendra_hand_v1_wrist.xml`, generated by `uv run python sim/convert_v1_wrist.py` **from `tendra_hand_v1.xml`** (re-run it after every `convert_v1.py` run; `uv run pytest` fails if stale). Adds `forearm_rot` (twist, + = pronation, axis = forearm centre x −33.5, y 25.5 mm), `wrist_flex` (+ = toward the palm, −60…+70°) and `wrist_dev` (+ = toward the thumb, −30…+20°) on a gimbal whose axes cross at the wrist centre, in a 20 mm gap (the palm and fingers sit 20 mm higher than in the hand model). Every servo strand passes one site `wrist_centre` (PTFE sheaths through a hollow wrist, owner's choice), so wrist motion never changes a strand length. Site `forearm_flange` = forearm bottom (z −153 mm) = where OpenArm's J5 flange bolts on. Wrist actuators and gimbal mass are placeholders. Known limit: > ~55° flexion with radial deviation, the thumb base hits the (too wide) forearm.
- **Arms** (2026-10-02, owner: OpenArm's shoulder and elbow for AI training now, an own version later; **two arms, no floating hand** since the same day): `tendra.arm` (`load_arm_model()`, `uv run python sim/view.py --arm`) builds OpenArm v2's pedestal (dependency `openarm-mujoco==2.3.0` in the default `arm` group, Apache-2.0, not copied into the repo) with both arms' J1–J4 renamed `shoulder_pitch`, `shoulder_roll`, `shoulder_yaw`, `elbow` (OpenArm's signs: + forward / out / outward / bend; **the left arm's J1–J3 are sign-flipped** so equal angles mean mirrored postures), OpenArm's forearm, wrist and gripper removed, and the wrist model bolted to J5's flange (thumb forward, palm toward the body when hanging). Names are `right_*` / `left_*`; 7 arm joints + 16 hand servos per arm = 46 position actuators; keyframes `rest`, `ready`, `table`; 2 ms step. World: +x forward, shoulders at z 0.698, OpenArm tables at z 0.40.
  - **Left hand = exact mirror of the right** (mirror plane x = -33.5 mm in the hand frame, = the body's middle): `hardware/cad/mirror_export.py` reflects `v1_export/` into `hardware/robot_description/v1_export_left/` (generated, committed; `--check` and a test fail if stale), `sim/convert_v1.py` / `convert_v1_wrist.py` take `--side right|left` (both by default) and write `tendra_hand_v1_left.xml`, `tendra_hand_v1_wrist_left.xml`. Same limits, masses and tendon lengths; "positive = closing" is unchanged; `sim/tests/test_v1_left.py` checks it. Re-run the converters after changing `v1_export/`: `mirror_export.py` first, then `convert_v1.py`, then `convert_v1_wrist.py`.
  - **The arm is a sim stand-in for training the hand** (owner: no arm hardware for a long time, probably an own design later), so everything is arm-agnostic: `ArmIK` (damped-least-squares IK) runs on `ik_model(side)`, a bare kinematic copy of one arm (~0.2 ms a solve, matches the full model), and turns a wrist target into joint targets. In the grasp scene (`tendra.scene.GraspScene`, `SceneConfig`): pedestal behind the table (`arm_base` y 0.35), shoulders `arm_shoulder_height` 0.30 m above the table, start pose `POSES["table"]` (elbow 90°, forearm twisted back, thumb up), gravity-compensated arms, the resting arm asleep (`sleep_idle`). One hand works per episode (`scene.reset(rng, obj, side)`); `scene.canon_pos/vec/axial/rot` map the left hand onto the right (the mirror), so `wrist_frame()`, `set_wrist_target(pos, quat, side)`, `arm_positions(side)`... take the side and the policy sees only a "right hand". The scripted grasp needs `ARM_GRASP` (8° pitch, +1 cm) for the cube and ball. `GraspEnvConfig(hands="right"|"left"|"any")` / `sim/train_grasp.py --hands`: one policy for both hands (observation 141, + arm joints/speeds, `arm_limits` penalty, orientation lead limit); trained on one hand per episode, a model that drives both at once comes later. Not supported: the GPU path (`tendra.gpu`: `contact_sensors=True` raises `NotImplementedError`; needs porting to two arms) and webcam grasp teleop (`sim/grasp_teleop.py` is a stub; needs a reachable home pose). Design: `research/ai/grasp-rl.md`.

## Software architecture

- **PC:** high-level control, AI, simulation, digital twin (Python).
- **ESP32:** low-level motor control only (C++/Arduino, PlatformIO).
- **Hardware abstraction layer (HAL):**
  - Firmware has a `MotorDriver` interface with `Uln2003StepperDriver` now and `Scs0009ServoDriver` later. Joint-level code never talks to pins directly.
  - PC has a `Hand` interface with interchangeable backends: `SimHand` (MuJoCo), `RealHand` (serial), and a mirror/digital-twin mode.
- **All joint commands are in joint space**, SI units (rad). The ESP32 converts to steps or servo ticks using per-joint calibration.
- **Firmware v0.1** (`firmware/`, see its README): `MotorDriver` HAL, `Uln2003Stepper` (half-step, 4076 steps/rev), trapezoidal `MotionProfile` (defaults 800 steps/s, 1600 steps/s², hard cap 1000 steps/s), coils released after 1 s idle, text protocol (`P/J/S/X/R/Z/M/K/V/I`), joint targets clamped to limits. Scale defaults to 1:1 (648.7 steps/rad) until calibrated per joint.
- Build: `pio run` in `firmware/`. Motion-profile unit tests run on the PC with `uvx --from ziglang` (command in `firmware/test/host/test_motion_profile.cpp`).
- **PC side:** Python package `tendra` in `software/tendra` (installed editable by `uv sync`). `Hand` interface with `SimHand` / `RealHand`; `FakeEsp32` emulates the firmware protocol for tests. Joint names and limits live in `tendra/joints.py`; tests check that they match `config.h` and the MJCF.
- **Digital twin:** `sim/twin.py` (`--fake` / `--port auto`, `--hand auto|v0|v1`). v0: sim → real. v1 also `--mirror` (real → sim from measured positions). `tendra.joints` has `HandSpec` variants `V0` / `V1`; `RealHand` auto-detects the variant from the `I` line.
- **Webcam teleop:** `sim/teleop.py` (MediaPipe Hand Landmarker, dependency group `teleop`, default in `uv run`; model cached in `~/.cache/tendra/`) → `tendra.retarget` (fingers: Gauss-Newton fit of a finger model to all 4 landmarks, all fingers batched, palm plane by SVD through wrist + knuckles, running bone lengths; fit with the DIP coupled to the PIP, so the robot fingertip lands closest to the human one; thumb: DLS on the MuJoCo model, pinch aims at the index tip; palm side from the joints' bend axis (sum of sin(bend), fist-safe), not MediaPipe's label) → `SimHand`, optionally `RealHand` via `TwinBridge`. Speed on the owner's laptop: the CPU and integrated GPU share power, so drawing slows tracking; MuJoCo's viewer redraws nonstop (tracking 28 → 7 fps) and the tendons' ~2,000 capsules cost ~10× the hand. Teleop therefore draws the hand itself (`tendra.hand_view`, only on change, ≤ 30 fps), in kinematic mode, with lite meshes (`tendra.lite_model`), no shadows, tendons hidden. Tendra can only pinch with a curled index ("O" pinch).
- **Grasp demos (sim):** the webcam teleop `sim/grasp_teleop.py` is **disabled** (2026-10-02; it drove the floating hand, removed; the old version is in git history; port it to `scene.set_wrist_target` with a reachable home pose). Still valid: the webcam wrist tracker `tendra.wrist` (view frame = mirror: x right, y up, z toward the camera) and `tendra.retarget` for fingers. Datasets (`tendra.dataset`, 30 fps) record the sim state to `~/tendra-data/datasets/<name>` (outside the repo): state/action = 16 finger servos + wrist pos + quat (+ 7 arm joints from RL rollouts, with the working `side` in each episode's extra); datasets recorded before 2026-10-01 have 27 and are refused by `synergy.postures_from_datasets`. Images are rendered afterwards by replay; `sim/export_lerobot.py` exports to LeRobot v3.0 (`uv run --with "lerobot[dataset]"`). A scripted power grasp lifts the cylinder, cube and ball with the real servo limits, with either hand.
- **Grasp RL (sim):** `sim/train_grasp.py` (PPO, `tendra.rl`) on `tendra.grasp_env.GraspEnv` (20 Hz, 4 ms physics): action = wrist velocity (6, in the right hand's view; the left hand is mirrored) + finger synergies (`tendra.synergy`, 6) + residual (20); human-like reward (grasp zone, palm facing, thumb opposition, lift/hold; penalties for knocking, jerk, effort); asymmetric critic; two arms, one hand per episode (`--hands right|left|any`); domain randomisation; demo-state starts (scripted grasps, teleop datasets) and object/penalty curriculum. `sim/eval_grasp.py` evaluates on fixed seeds, watches, films, records rollouts as datasets. PyTorch is in the default `train` group (CPU). Runs go to `runs/` (git-ignored). Physics limits speed (~300–650 steps/s on the laptop; a held object costs ~3× a free hand). Design: `research/ai/grasp-rl.md`.
- **Serial link:** native USB CDC. The baud rate setting is ignored for native USB (it always runs at USB speed), but keep it at 921600 for tools that need a value.

## Website

- **Stack (chosen 2026-09-27):** Next.js 16 (App Router) + TypeScript + Tailwind v4, React Three Fiber + drei (3D hand), GSAP ScrollTrigger + Lenis (scroll story), MDX content. The app lives in `website/`; plan, file ownership and the scroll timeline are in `website/PLAN.md`. Hosting not chosen yet.
- **Design system v0.2** in `website/assets/css` is the source of truth (imported into Tailwind layers; tokens map to utilities). Style guide: `website/styleguide/index.html`, served at `/styleguide/`. Its grid helper is `.auto-grid` (not `.grid`, which clashes with Tailwind).
- **Journey page** (2026-10-03): `/journey` shows only what is **done** (no plans, they change) plus ideas we **dropped**, each with `why` and `lesson`. Coloured subway map (5 lanes, one column per day), day-by-day chapters, and a "dropped ideas" section. Data: `website/content/journey.json` (`days`, `events` with `status` done / abandoned, `replaces`); components `JourneyMap/Days/Dropped.tsx`. Add an event there with every new `research/log.md` entry, and a `why`/`lesson` when something is dropped.
- **All text lives in `website/content/`** (JSON + MDX); components don't hard-code copy. Placeholders are marked `TODO`.
- **3D hand:** `src/lib/handState.ts` is the scroll ↔ 3D contract (plain numbers GSAP tweens, R3F reads per frame). The real model is `public/models/hand.glb`, made from the STEP by `website/scripts/step-to-glb.py` (OpenCascade, one mesh per part; run `uv run --with cadquery-ocp --with pygltflib python website/scripts/step-to-glb.py`) and set in `HAND_MODEL_URL` (`src/components/three/model.ts`). The script also rigs it: parts are matched to MuJoCo bodies by bounding box, and joint nodes sit at the URDF pivots with the sim's signs, so bending, labels, tendons and explode work (naming rules in `public/models/README.md`). Re-run it after a new STEP export. On desktop all hero text is centred; the hand starts small, under the headline and to the right of the text (measured in `ScrollStory` `measureHero()`, fitted via `HandState.shiftY`/`zoom` in `heroFit()`), then glides into and fills the right column for the rest of the story (text on the left, never over the hand) and turns 2 full turns over the story, driven by the scroll (`storyTimeline.ts`). The canvas loads on first interaction to keep mobile Lighthouse ≥ 90. `/dev/hand` is the hand lab. The **project page** hero shows the full 5-finger V1 as a static model (`public/models/hand-v1.glb`, from `website/scripts/v1-to-glb.py`) that spins by itself and can be dragged (`SpinHand.tsx`); it follows the same motion switch (`html.motion`), as does the page's CSS animation.
- **Docs site (2026-10-03):** the docs are their own section, meant for `docs.<domain>`: Docs, Hardware, Software and Build log live under `/docs/*` (`/docs/hardware`, `/docs/software`, `/docs/log`); the old `/hardware`, `/software`, `/log` redirect there. Same Next.js app: `next.config.ts` rewrites the docs host (`docs.*`) to `/docs/*` and redirects `/docs/x` to `/x` on it. Route groups: `src/app/(site)` (own nav and footer: Project, Journey, Gallery, Contribute, Docs) and `src/app/docs` (docs header, sidebar, own footer); nav lists are in `content/site.json` (`nav`, `docsNav`). Set `NEXT_PUBLIC_DOCS_URL` (e.g. `https://docs.example.com`) and `NEXT_PUBLIC_SITE_URL` once the domain exists; until then everything runs on one host. Test locally at `docs.localhost:3000`.
- **Hosting: Vercel** (prepared 2026-10-05, Root Directory `website`, "include files outside the root" on so `../media` is copied; steps in `website/DEPLOY.md`). Site URL comes from env (`siteUrl` in `src/lib/site.ts`: `NEXT_PUBLIC_SITE_URL` → Vercel's production URL), not from `site.json`. Security headers + a static CSP live in `next.config.ts`; any new external origin, worker or WebAssembly use must be added there (`'wasm-unsafe-eval'` is for the meshopt decoder in drei's GLTF loader). SEO: JSON-LD in `src/components/seo/`, `/llms.txt` and `/llms-full.txt` (`KEY_FACTS` in `src/components/seo/llms.ts` is hand-written: update it when specs or status change), off-site steps in `website/SEO.md`. Things removed for launch that need real content: `website/FILL-IN.md`. Media is only published if it's in `media/catalog.json` without `"public": false`.
- Commands (in `website/`): `npm run dev`, `npm run build`, `npm run lint`, `npm run typecheck`. Review agents for new pages: `.claude/agents/{design-reviewer,accessibility-checker,performance-checker,code-reviewer}.md`.
- Style: **clean, open, friendly, Apple-like** (owner rejected a dark "robotic/nerdy" look on 2026-09-27). **Light default**, dark opt-in (`data-theme="dark"`). One calm **blue accent `#0066CC`**, only for clickable things. No orange. Font: Inter (JetBrains Mono only for code). Pill buttons, rounded cards, soft shadows. Plain words for a general audience.
- Colours only through tokens; motion must respect `prefers-reduced-motion` and work without JS. The homepage story keys on `html.motion` (`src/lib/scroll/motion.ts`): on unless the system asks for reduced motion, and those visitors can opt back in with the "Play animation" button (it disappears once the story plays). The Hand lab (`/dev/hand`, no longer in the nav since 2026-10-03, reachable by URL) always animates.

## Licensing (decided 2026-09-26)

- Code (firmware, Python, sim, AI code/models): **Apache-2.0**, in root `LICENSE`
- Hardware (`hardware/`): **CERN-OHL-S-2.0**, in `hardware/LICENSE`
- Docs/research/website text (`docs/`, `research/`): **CC BY 4.0**, in `docs/LICENSE`
- Never use NonCommercial licenses. Check third-party licenses before copying code or designs in.

## Conventions

- Units: SI everywhere in code (m, kg, s, rad). Degrees only in UIs and human-facing docs.
- Joint names: `<finger>_<joint>[_<motion>]`, e.g. `index_mcp_flex`, `thumb_cmc_rot`.
- Python: 3.12, `uv`, formatted with `ruff`, type hints, `pytest` for tests.
- Firmware: PlatformIO, Arduino framework, C++17, one class per driver.
- Commits: small, descriptive, imperative ("Add ULN2003 stepper driver"). Never push without asking.
- Research: dated entries in `research/log.md`.
