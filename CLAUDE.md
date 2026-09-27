# CLAUDE.md — Project context for Claude

> Keep this file up to date. When hardware, pins, conventions or decisions change, update it in the same step.

## Project

- **Name:** **Tendra Hand** (chosen 2026-09-26). Note: an unrelated open-source project "Tendra H1" (github.com/aymankhayat/tendra-h1-robotic-hand) already uses the name. The owner chose to keep it; revisit before registering a company or trademark.
- **Goal:** an open-source, tendon-driven, 3D-printed robotic hand with the same DOF as a human hand, able to do complex tasks. The owner wants to become a robotics entrepreneur; this repo is the main research hub and project-management center.
- **Everything is open source:** CAD/3D files, firmware, software, AI models, research. It is published in a public GitHub repo and on a project website.
- **Language:** all docs, code and comments in English.
- **Current phase:** Phase 1 — make the thumb and index finger move smoothly and cleanly (see `docs/roadmap.md`).

## Working with the owner

- They are **learning**, so briefly explain key concepts (what and why) as you go, without lecturing.
- **Ask before big structural changes**: moving or renaming folders, changing the robot model, changing protocols, adding licenses, pushing to GitHub.
- Work **step by step**. After each step, say what you did, what you verified, and what's next.
- Hardware safety: never write firmware that moves motors on boot without a command. Keep coil current and power limits in mind.
- Log findings, experiments and decisions in `research/log.md` (dated entries).

## Owner's machine

| Item | Value |
|---|---|
| OS | Windows 11 Pro Education (Norwegian locale) |
| CPU / RAM | Intel i3-1125G4 (4 cores), 16 GB |
| GPU | Intel UHD (integrated). **No NVIDIA**, so Isaac Sim/Lab can't run locally; use MuJoCo on CPU, and cloud GPUs for heavy training later |
| Python | system 3.14.6; `uv` installed. Project uses a **uv-managed venv with Python 3.12** for library compatibility |
| Firmware tools | **PlatformIO** with the Arduino framework: VS Code extension (owner) + PlatformIO Core 6.2 CLI via `uv tool` (`pio`, used by Claude to build and flash) |
| Smart App Control | Turned **off** (2026-09-26); it had blocked MuJoCo's DLLs. Verified: MuJoCo 3.14.0 imports on Python 3.12 via uv. |

Shell notes: use forward slashes / Git Bash syntax in the Bash tool. The project path has no spaces.
Tidy root: `.git`, `.venv`, `.gitignore` and `.gitattributes` have the Windows *hidden* attribute. pytest/ruff caches live in `.venv/` (set in `pyproject.toml`). Keep new tool caches out of the root too.

## Hardware (current prototype: thumb + index)

- 3D-printed rigid skeleton, PLA/PETG. TPU fingertip pads are planned.
- **Tendon-driven.** Each joint is driven independently by **one motor that both flexes and extends** it (an antagonistic tendon loop on one spool).
- Motor spool: **radius 1 cm** (diameter 2 cm), so one output revolution pulls ≈62.8 mm of tendon, or ≈0.031 mm per full step. The joint-side moment arm is unknown, so steps-per-radian is **calibrated empirically** per joint.
- **Actuators now:** 8× **28BYJ-48 (5 V)** unipolar steppers + **ULN2003** driver boards.
  - Gear ratio ≈ 63.68:1, so **≈2038 steps/output rev (full-step)** and ≈4076 (half-step). Top speed ≈ 15 rpm.
  - Open loop, with no position feedback. **Homing is manual:** before power-up the owner straightens every joint, and that pose is `q = 0`.
- **Power:** separate **5 V / 2 A** supply for the motors. Each energised 28BYJ-48 draws ~200–250 mA per phase, so 8 motors is close to the limit. Firmware should **release coils of idle motors** and avoid 2-phase-on for all 8 at once.
- **Controller:** **ESP32-S3-N16R8** (16 MB quad flash, **8 MB octal PSRAM**), connected to the PC over **USB-C (native USB CDC)**.
- **Planned upgrade:** **Feetech SCS0009** smart servos (half-duplex TTL bus, daisy-chained, addressable IDs, 5 V, position feedback), via the **FE-URT-1** signal converter (owner has it). This fixes homing, feedback and the pin-count problem.
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
| 3 | 8, 3, 14, 9 | `index_mcp_flex` | Revolute 2 | Index knuckle bend |
| 4 | 10, 11, 12, 13 | `index_mcp_abd` | Revolute 1 | Index sideways |
| 5 | 1, 2, 42, 41 | `thumb_ip` | Revolute 8 | Thumb tip joint |
| 6 | 40, 39, 38, **37** | `thumb_mcp` | Revolute 7 | Thumb 2nd joint from tip |
| 7 | **36, 35, 0**, 45 | `thumb_cmc_flex` | Revolute 6 | Thumb 3rd joint from tip |
| 8 | **48**, 47, 21, **43** | `thumb_cmc_rot` | Revolute 5 | Thumb base rotation ("other direction") |

The mapping and the joint directions were confirmed by the owner in the MuJoCo viewer (2026-09-26).

### ⚠️ Pin constraints (ESP32-S3-N16R8)

- **GPIO 35, 36, 37:** wired to the **octal PSRAM** on N16R8. They only work as motor pins if **PSRAM is disabled** in the firmware build. Never enable PSRAM with this pin map. (Planned fix: servo bus.)
- **GPIO 0:** strapping pin; LOW at reset means download mode. The ULN2003 input pulls it low, so the board may fail to boot normally with motor 7 connected. Watch for this.
- **GPIO 3, 45:** strapping pins, but LOW is their default, so they're OK.
- **GPIO 43:** UART0 TX. The boot ROM prints here, so motor 8 may twitch at reset. Firmware must not log on UART0 (use USB CDC).
- **GPIO 48:** often the onboard RGB LED. Don't drive the LED in firmware.
- **GPIO 19/20:** native USB. Never use them for motors.
- All motor outputs must be driven LOW (coils off) as the very first thing in `setup()`.

## Repository layout

```
hardware/   cad/ (f3d, STEP) · print/ (STL) · electronics/ · robot_description/fusion_export/ (raw Fusion URDF export)
firmware/   PlatformIO project for the ESP32-S3
sim/        MuJoCo models, URDF→MJCF converter, digital twin
software/   PC-side Python: Hand API, control, calibration, AI
docs/       roadmap and guides
research/   log.md, experiments, references
website/    project website (Next.js app + design system)
```

Every folder has a README explaining what goes there. Keep them current.

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
- **Digital twin v1:** `sim/twin.py` (`--fake` / `--port auto`). Sim → real only; real → sim (measured positions) comes with the servos.
- **Serial link:** native USB CDC. The baud rate setting is ignored for native USB (it always runs at USB speed), but keep it at 921600 for tools that need a value.

## Website

- **Stack (chosen 2026-09-27):** Next.js 16 (App Router) + TypeScript + Tailwind v4, React Three Fiber + drei (3D hand), GSAP ScrollTrigger + Lenis (scroll story), MDX content. The app lives in `website/`; plan, file ownership and the scroll timeline are in `website/PLAN.md`. Hosting not chosen yet.
- **Design system v0.2** in `website/assets/css` is the source of truth (imported into Tailwind layers; tokens map to utilities). Style guide: `website/styleguide/index.html`, served at `/styleguide/`. Its grid helper is `.auto-grid` (not `.grid`, which clashes with Tailwind).
- **All text lives in `website/content/`** (JSON + MDX); components don't hard-code copy. Placeholders are marked `TODO`.
- **3D hand:** `src/lib/handState.ts` is the scroll ↔ 3D contract (plain numbers GSAP tweens, R3F reads per frame). The real model swaps in via `HAND_MODEL_URL` in `src/components/three/model.ts` (naming rules in `public/models/README.md`). The canvas loads on first interaction to keep mobile Lighthouse ≥ 90. `/dev/hand` is the hand lab.
- Commands (in `website/`): `npm run dev`, `npm run build`, `npm run lint`, `npm run typecheck`. Review agents for new pages: `.claude/agents/{design-reviewer,accessibility-checker,performance-checker,code-reviewer}.md`.
- Style: **clean, open, friendly, Apple-like** (owner rejected a dark "robotic/nerdy" look on 2026-09-27). **Light default**, dark opt-in (`data-theme="dark"`). One calm **blue accent `#0066CC`**, only for clickable things. No orange. Font: Inter (JetBrains Mono only for code). Pill buttons, rounded cards, soft shadows. Plain words for a general audience.
- Colours only through tokens; motion must respect `prefers-reduced-motion` and work without JS.

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
