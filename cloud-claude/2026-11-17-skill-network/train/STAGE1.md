# Stage 1 design: the skill environment and the training pipeline

Binding interface for the Stage 1 agents. Goal (README §2–4, 6b): **one shared skill network** that executes
commands (`hold, reach, grasp, carry, place, release`) on the V2.1 hand on OpenArm, trained with PPO on MuJoCo
Warp (thousands of worlds on a RunPod GPU, `impl="warp"`), the same code on CPU here for tests. Pick-and-place =
a scripted planner chaining commands. Right hand only.

## Layout (study root `cloud-claude/2026-11-17-skill-network/`)

```
skills/                 the environment (python package, JAX; imports mujoco, mujoco.mjx, jax, playground, numpy)
  robot.py              load model/models/v21_scene.xml, names → index maps (actuators, qpos, sensors, sites), constants
  kinematics.py         JAX FK/IK of the 7-joint arm chain (shoulder…wrist_dev) to a palm pose; quaternion helpers
  control.py            the low-level layer that the real firmware will also run (below)
  commands.py           skill ids, command vector layout, sampling of commands/goals, per-skill success checks
  perception.py         the "estimated object state" model (noise, delay, dropout, size error) of README §6b
  rewards.py            per-skill reward terms + shared calm terms
  randomize.py          domain randomisation (model fields per world + per-episode values)
  env.py                SkillEnv(mjx_env.MjxEnv): reset/step/observe; chained starts; curriculum hooks
  starts.py             start-state bank: scripted controllers → npz of states per skill (CPU, MuJoCo C)
  planner.py            text planner: "pick cube place red_spot" → command sequence with success checks
train/
  STAGE1.md             this file
  configs/*.yaml        run configs (smoke_cpu.yaml, runpod_h100.yaml, …)
  train.py              Brax PPO driver: config, curriculum, checkpoints, resume, TensorBoard/W&B, rollback
  evaluate.py           fixed-seed evaluation per skill + full pick-and-place chain; metrics; video
  policy.py             numpy-only policy runner (load checkpoint, act) for the laptop / later the robot
docker/Dockerfile       RunPod image (CUDA 12, jax[cuda12]==0.9.2, the study's pinned stack)
runpod/                 launch.sh, RUNPOD.md (how to start, resume, download results; cost estimate)
tests/test_env*.py      env tests;  tests/test_train*.py  pipeline tests
```

## Timing

- Physics `dt` = `build_hand.PHYSICS` timestep (2 ms) unless the env agent measures that 4 ms keeps grasps and
  contacts equally good (then document it). Policy at **20 Hz** (50 ms). The control layer runs inside the
  physics substeps at its own rate (≥ 100 Hz).

## Action (policy output, all in [−1, 1], 22 values)

- 6: palm velocity command in the **palm frame** (3 linear, 3 angular), scaled by config limits; integrated
  into a palm target pose, kept within a lead limit of the actual pose and a workspace box, turned into the 7
  arm joint targets by JAX damped-least-squares IK (`kinematics.py`, refreshed every few substeps).
- 16: hand servo **joint targets** (map [−1, 1] → joint range). Not synergies (those were V1's).
- The control layer adds what the real firmware will do:
  - **Joint-sensor outer loop** (README decision): reads the joint angle with sensor noise (≈ 0.5°, from the
    V2.1 TMAG5273 plan) at its rate and moves the servo setpoint (`ctrl`, which may exceed the joint range up to
    `ctrlrange`) so the joint reaches the policy's target: PI with anti-windup; gains in config.
  - **Speed limits**: setpoint slew ≤ `velocity_limit_rad_s` per joint (`model/actuators.json`); wrist commands
    filtered (smooth-control lesson); finger targets ramped over substeps.
  - **Thumb range rule** if the CAD fix leaves one (see `model/CONTRACT.md` / study README).
  - Optional latency (per-episode, 10–50 ms) and servo deadband from `actuators.json` → `randomization`.

## Command (observation part, constant during an episode)

`[one-hot skill (6), params (P)]`, with P fixed size (unused = 0), all in the **palm/robot frame of the hand at
episode start** or the robot base frame (choose one, document it, be consistent):
- `reach`: target palm position + orientation (6D rotation) for pre-grasp.
- `grasp`: grip type one-hot (`power, pinch, tripod`) (+ the object is the one of the episode).
- `carry`: target object position (+ yaw).
- `place`: target object position on a surface.
- `release`: retreat distance.
- `hold`: none.

## Observation

- **Actor** ("state"): proprioception (16 joint-sensor readings with noise, 7 arm joints, servo setpoints, last
  action, servo force readings `_frc` as the current stand-in, palm pose in the base frame), the command,
  **estimated object state** from `perception.py` (pose, velocity, size estimate, a "seen" flag; noise, delay,
  dropout when the hand occludes it — README §6b, the real robot will get this from the perception→twin path),
  fingertip/palm touch flags (from the V2.1 pressure/MLX sensors: on/off with noise). Normalised.
- **Critic** ("privileged_state"): the actor's + exact object state, contact forces, randomisation values,
  goal errors. Asymmetric actor-critic.
- Keep both as flat float32 vectors; sizes in `env.py` constants; document every slice.

## Episode and starts

- One object per world (cube, cylinder, ball, box, capsule; size/mass/friction randomised), others parked.
  Lessons from the old env (`/home/user/tendra-hand/software/tendra/gpu/grasp_env_jax.py`, read it): one model for
  all worlds, gravity on the active object only, per-world friction via Brax `randomization_fn`, Playground
  auto-reset does not reset `info` → reset per-episode info yourself (the old "+30 bonus once per chunk" bug).
- Each world samples a skill (curriculum weights) and its command. Starts come from `starts.py`'s bank: a
  `grasp` starts where a `reach` ended, `carry` / `place` from holding states, `release` from placed states,
  `reach` / `hold` from free states. Bank v1 = scripted controllers (MuJoCo C, CPU); later refreshed from policy
  rollouts.
- Success per skill (`commands.py`), episode ends on success + a short "be still" hold, on failure (object
  dropped/knocked away, unsafe contact like the wrist camera or forearm hitting the table hard), or timeout.

## Reward

Per-skill task terms (dense progress + success bonus) + shared terms: action rate and jerk (wrist weighted),
`still` while the goal is met, effort (servo force), arm-limit, table-impact penalty, object-knocked penalty.
Lessons in `/home/user/tendra-hand/research/ai/smooth-control.md`. Every term logged separately.

## Training (`train/train.py`)

Brax PPO (`brax==0.14.2`) through Playground's `wrap_for_brax_training`, asymmetric obs, LayerNorm critic,
settings from Playground's dexterous configs, CAPS smoothness loss (reuse the idea of
`/home/user/tendra-hand/software/tendra/gpu/caps.py`), chunked curriculum per skill (success-driven, logged),
staggered resets, NaN/blow-up rollback, atomic checkpoints, resume by re-running the same command, `--hours`
budget, TensorBoard always + W&B when `WANDB_API_KEY` is set. Outputs in `runs/<name>/` (git-ignored) or a
RunPod volume. A `smoke_cpu.yaml` config trains a few thousand steps on CPU in minutes (tests use it).

## RunPod

`docker/Dockerfile` + `runpod/launch.sh` + `runpod/RUNPOD.md`: one command starts or resumes a run on a network
volume; target GPU class H100/A100-80GB (~$2–3/h); compile caches (JAX, Warp) on the volume. Verified here by:
building nothing that needs a GPU, but running the exact entrypoint with `JAX_PLATFORMS=cpu` and the smoke config.

## Quality bar

Tests for every module; an env test that compares a few hundred MjxEnv steps (warp impl, CPU) against MuJoCo C
for the same actions; determinism with a fixed seed; throughput benchmark on CPU (steps/s for 1, 64, 256
worlds); ruff clean; no hard-coded model numbers (read `actuators.json`, the model, `CONTRACT.md`).
