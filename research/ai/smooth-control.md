# Smooth, steady, reliable control: research and plan

*Written 2026-10-06 by an agent team: three researchers (smooth RL, state-of-the-art robot learning,
the control layer) plus a diagnosis of our own code. Raw notes with every source, the full
diagnosis and the measuring scripts are in
[`research/experiments/2026-10-06-smooth-control/`](../experiments/2026-10-06-smooth-control/).*

**The problem (owner, 2026-10-06):** after 18 M training steps the grasp policy can lift objects,
but it is shaky. It moves even when it should be still. The goal is a calm robot that rests when it
should and does its tasks comfortably and reliably, so that one day it can cook and do the laundry.

## Status (2026-10-06, end of day)

Built by an agent team (arm damping, CAPS, motion layer, calm filter) plus an independent review,
which found 3 serious bugs (all fixed, see the log). Tests: 370 pass, GPU parity 16/16, CAPS 7/7.

| Step | Status | What exists |
|---|---|---|
| 0 Measure | **done** (the 18 M policy itself is on Drive, not measured yet) | `sim/eval_grasp.py`: success with 95 % Wilson interval; `rate`, `jerk` (fixed wrist weight 4); `fft` (CAPS score); while holding at lift height: `wrist` mm / `finger` deg per step and their back-and-forth part `wshake` / `fshake` (= the shaking); `jitter_probe.py` |
| 1 Fair viewing | **done** | `tendra.gpu.policy.trained_settings`: eval and video use the run's objects, noise level and action settings (old runs: the old ones, `LEGACY_ENV`) |
| 2 Calm v2 env | **done, not trained yet** | Both envs: wrist 0.15 m/s / 0.8 rad/s, filtered wrist command (0.4), fingers 0.25, `action_rate` 0.5 (wrist × 4) + `jerk` 0.2 outside the curriculum, `still` reward 1.0, GPU obs-noise floor 0.5. Arm servos damped (`arm.py` `ARM_KV_SCALE` 4, wrist 2): overshoot 29–48 % → 0–1 %. CAPS loss in `train_brax.py` (λT 0.1, λS 0.05, σ 0.05) |
| 2b GPU reset bug | **fixed** | Since the first GPU runs, an episode end on the GPU reset only the physics, not the env's per-episode memory: `t` kept growing (the policy's time input reached ~75+), and **the +30 success bonus was paid only once per world per training chunk**. `new_episode` in `grasp_env_jax.py` now resets it. The 18 M policy was trained with this bug. |
| 3 Motion layer | **done, not trained yet** | `finger_ramp`, `wrist_ramp` (targets move every 4 ms instead of jumping every 48 ms: finger peak acceleration −47 %); `action_delay` knob (default 0) |
| 3b Deploy filter | **built, not used** | `tendra/calm.py` (`CalmFilter`, `HoldLatch`, presets `light`, `light+latch`, `full`), `eval_grasp --calm`. On the old CPU policy no preset costs success, but none calms it either (wrist unchanged, cube fingers 0.31 → 0.22–0.25°): **filtering what a shaky policy sends doesn't fix it**; the calm parts must be in training (Steps 2–3). Kept as the reference for the firmware. |
| Colab kit | **ready** | `runs/colab/tendra_gpu_kit_c9b3d006.zip` + `train_grasp_gpu.ipynb`, run `calm1` on Drive. The 18 M run's bundle is kept as `runs/colab/tendra_gpu_gpu3.npz` |
| 4–7 | open | Next: train `calm1` on Colab, then compare with the baseline below |

**Baseline** (old CPU policy `runs/arm_right` on the damped arm, 10 episodes per object, no
filter): success 100 % (cylinder, cube); while holding, the wrist moves 2.3–3.2 mm per step, of
which **1.0–1.5 mm is shaking** (`wshake`); the fingers' back-and-forth (0.30–0.41°) is larger than
their net motion (0.23–0.31°), so nearly all finger motion while holding is shaking. Target: `wshake`
< 0.3 mm, `fshake` < 0.1° (section 5).

Old policies still run as they were trained: new settings have "legacy" values that old saved
configs get (`LEGACY_CONFIG` / `LEGACY_REWARDS` in `grasp_env.py`, `LEGACY_ENV` in
`gpu/policy.py`). The arm damping is physics and applies to every policy.

---

**Short answer:** the shaking is not a mystery and it is not the physics. It comes from **how we
set up the training**: big, twitchy wrist commands on a springy arm, a reward that pays almost
nothing for being calm, and a test setup that adds noise the policy never saw in training. The best
robots are smooth because they **stack several layers**, and each layer removes a different kind of
shake. We can build the same layers, one step at a time.

---

## 1. What is wrong today (diagnosis)

The diagnosis agent ran experiments on the CPU twin of the training environment
(`notes/diagnosis.md`).

| Test | Result |
|---|---|
| Action = 0 for 5 s | Perfectly still (wrist 0.000 mm per step) |
| Holding a scripted grasp with a **constant** action | Holds 100 %, wrist 0.001 mm per step |
| Tiny random noise (std 0.05) on the wrist actions | Wrist moves 1 mm **every step**, which is visible |
| One full wrist action | The arm overshoots by **38 %** and rings at **4.8 Hz** |
| A simple "correct the full error each step" controller | Goes into a 2 cm, 4.8 Hz wobble, even with no noise |

So the robot **can** be perfectly still. It shakes because the policy's output changes a little
every step, and our setup turns those small changes into large, visible motion.

### The causes, ranked

1. **Wrist commands are too big for a springy arm** (`grasp_env_jax.py:82-85, 540-549`). One full
   action moves the wrist target 14.4 mm or 5.5° in a single 50 ms step. The simulated arm servos
   are under-damped (damping ratio ≈ 0.29), so they overshoot and ring. *Plain words:* it is like
   driving a car whose steering is far too sensitive, on soft springs. Every small twitch of the
   wheel makes the car wobble.
2. **The reward pays almost nothing for calm.** The smoothness penalty is 0.1 × `penalty_scale`,
   and the curriculum still has `penalty_scale` at 0.3 at 18 M steps. Visible jitter therefore costs
   about 0.0001 per step, while holding the object earns about 9 per step. **There is no reward for
   being still while holding**, and after success the episode keeps running with nothing to aim
   for, so jitter is "free". *Plain words:* we never told the robot that calm matters.
3. **The viewer adds noise that training never had.** At 18 M steps the curriculum had not turned on
   domain randomisation yet (`dr_level` 0, so no observation noise). But `rollout_video.py` runs at
   `dr_level` 1.0 (3 mm position noise, mass × 0.5–2), and `eval_grasp.py` always adds 3 mm noise. In
   the experiments, 3 mm of noise turned a calm controller into 11 mm per step of jitter. **Some of
   the shaking you saw may come from the viewer, not from the policy.**
4. **Exploration noise taught it big corrections.** PPO tries random variations of every action at
   every step (white noise). The policy learned to fight that noise with large corrections. When we
   watch it without the noise (the mean action, which we already use), those large corrections act
   on tiny errors.
5. **Possible every-step flicker** through `prev_action` in the observation (the policy reacts to its
   own last action). This would show up as a 10 Hz peak in the measurements.
6. **Finger targets are absolute and only lightly filtered** (smoothing 0.4). To hold still, the
   policy must output exactly the same 26 finger numbers every step.
7. **Normaliser blow-up on untrained objects.** If the eval uses an object the policy never trained
   on, a never-changing input is divided by about 1e-6, which explodes. Evaluate only trained objects.

**First thing to do:** download `policy_best.npz` from Drive and run the probe. It separates the
causes by their "fingerprint" (noise vs no noise, 4.8 Hz vs 10 Hz):

```
uv run python research/experiments/2026-10-06-smooth-control/scripts/jitter_probe.py policy_best.npz runs/colab/tendra_gpu.npz --objects cylinder cube
```

---

## 2. How the best robots get smooth and reliable

No lab gets smoothness from one trick. They stack layers, and each layer catches a different kind
of shake:

```
  "Make breakfast"                         slow brain (VLM, 5-10 Hz): what to do next
        |
  plan of motion, a chunk of ~0.3-1 s      fast policy (10-200 Hz): how to move, outputs a short trajectory
        |
  smooth set-points every 4-10 ms          motion layer: interpolation, jerk limits, hold latch, safety filter
        |
  servo PD loop                            servo: deadband, gentle gains, current/load limits
```

| System | What makes it smooth | What makes it reliable | Source |
|---|---|---|---|
| **Figure Helix** | 80 M-parameter fast network at 200 Hz under a 7 B "brain" at 7–9 Hz | One set of weights for all tasks, ~500 h of teleop data | figure.ai/news/helix |
| **NVIDIA GR00T N1** | Flow-matching action head, chunks of 16 actions at 120 Hz | Real + synthetic + video data | arXiv 2503.14734 |
| **π0 / π0.5 / π\*0.6** (Physical Intelligence) | Chunks of 50 actions; real-time chunking blends chunk boundaries | π\*0.6 learns from its own mistakes plus human corrections: 13 h of espresso making non-stop, failures halved | arXiv 2511.14759, 2506.07339 |
| **ACT / Diffusion Policy** | Predicts a whole trajectory, averages overlapping predictions | Learns from smooth human demos | arXiv 2304.13705 |
| **DextrAH (NVIDIA hands)** | Policy outputs into a classical motion generator ("fabrics") that enforces smooth, safe motion | Privileged teacher → student, heavy randomisation | arXiv 2412.01791 |
| **DeepMind soccer** | Strong action filter (0.8 old + 0.2 new at 40 Hz), mean action at deploy | Random delays and pushes in training | arXiv 2304.13653 |
| **HIL-SERL** | — | ~100 % success after 1–2.5 h of real training with a human correcting | arXiv 2410.21845 |
| **Tesla FSD** | End-to-end net trained on huge amounts of smooth human driving | **Data engine:** find failures, collect more like them, retrain; **shadow mode:** a new net runs silently and is compared before it is trusted; regression tests on every release | secondary sources only |

**The most important lesson for us** (DPPO paper, diffusion-ppo.github.io): a plain PPO policy like
ours reached **88 % success in sim but failed completely on the real robot because of jittery
behaviour**. The same task with a smooth chunked policy, fine-tuned with PPO, got **80 % on the real
robot zero-shot**. That is exactly our symptom, and it tells us where we need to go long term.

### Methods that make an RL policy itself smooth

| Method | Idea in plain words | Evidence | Cost |
|---|---|---|---|
| **Action-rate + jerk penalties** | Pay a price for changing the action, and more for changing it back and forth | Standard in every legged robot (ETH, Unitree, Berkeley) | Trivial |
| **Hold-still reward** | While holding, reward low speed of the wrist and object | Standard "stand still" term in locomotion | Trivial |
| **CAPS** | Two extra loss terms: the next action should be close to this one, and a slightly noisy observation should give almost the same action | Real drone: ~80 % less power, smoother | Small (an extra term in the PPO loss) |
| **Lipschitz-constrained policy (LCP)** | Limit how sharply the network can react to its inputs | Jitter 42 → **3.2** (smoothness reward alone 5.7, low-pass filter 7.9) | Moderate (gradient penalty; spectral normalisation is a cheaper variant) |
| **Coloured / pink exploration noise** | Explore with slowly drifting noise instead of a fresh random kick each step | PPO works best with β ≈ 0.5 (arXiv 2312.11091) | Small, but a custom Brax change |
| **Action filter / delta actions** | Low-pass the commands, or let the policy output *changes*, so "0" means "stay" | OpenAI Dactyl (EMA 0.3), DeepMind soccer (0.2) | Trivial; adds some lag |
| **Action chunking** | Output a short planned trajectory instead of one step | ACT, π0, GR00T, Helix | Big change (new policy type) |

### The control layer between the AI and the motors

- **Two speeds:** the policy decides at 20 Hz, a simple **set-point generator** runs at the physics
  rate (4 ms) and turns each decision into a smooth path (a critically damped filter or Ruckig, a
  jerk-limited trajectory generator). This removes "stair-step" motion. **The policy must be trained
  with this layer in place**, not only run with it.
- **Hold latch (state machine):** reach → grasp → hold → release. In "hold" the finger targets are
  frozen unless the object starts to slip. Policy noise then cannot reach the motors at all.
- **Deadband with hysteresis:** don't send a command smaller than ~0.3–1°; it takes a bigger change
  to start moving than to keep moving, so it can't chatter at the edge.
- **SCS0009 servo settings** (partly unverified, test on hardware): dead zone registers 26/27
  (1–3 units ≈ 0.4–1.1°), P/D gains at 21/22 (lower them if a servo buzzes), goal time/speed at
  44/46 (the servo ramps by itself). The acceleration register (41) is disputed between sources.
- **Safety filter:** joint limits, a rate limit, load and temperature limits (the SCS0009 trips at
  80 % load for 2 s), a watchdog, and rejecting NaN commands.
- **Servo model in sim:** measure the real servo (delay, quantisation, deadband, gains) and put it
  in MuJoCo with randomisation, so sim and reality behave the same.

---

## 3. The plan

Each step has a **done when**, so we know when to move on. Steps 1–3 fix the shaking. Steps 4–7 are
the road to a reliable household robot.

### Step 0: Measure first (≈ 1 day)
- Download `policy_best.npz` and run `jitter_probe.py` (with and without noise; FFT peak).
- Add calm metrics to `sim/eval_grasp.py`: **rest jitter** (std of joint angles and wrist position
  while holding), **action rate** (mean |aₜ − aₜ₋₁|), **jerk RMS**, an **FFT smoothness score**
  (CAPS), **hold drops**, and success with a **95 % confidence interval** (200 episodes per object
  gives about ±4 points).
- Save today's numbers as the **baseline**.
- **Done when:** we know which causes dominate, and we have baseline numbers.

### Step 1: Quick fixes, no retraining (≈ 1 day)
- Make the viewers match training: `rollout_video.py` and `eval_grasp.py` read `dr_level` and the
  trained objects from `curriculum.json`; add `--obs-noise`.
- Evaluate only trained objects; clip the normalised inputs to ±5 in `BraxPolicy`.
- Try a deploy-only filter + deadband on the current policy to see how much it helps.
- **Done when:** we know how calm the current policy really is when watched fairly.

### Step 2: Retrain "calm v2" (≈ 1 week incl. Colab runs)
All changes go into **both** the CPU and the GPU env, so the parity tests still pass.

| Change | From | To |
|---|---|---|
| `wrist_speed` / `wrist_turn_speed` | 0.3 m/s / 2.0 rad/s | **0.15 / 0.8** |
| Wrist command | integrated directly | **filtered velocity state** (0.4), shown to the policy |
| Arm servo damping (kv) | −2.7 / −2.2 | **≈ −9 / −7** (overshoot < 5 %, checked by a step test) |
| `finger_smoothing` | 0.4 | **0.25** |
| `action_rate` | 0.1 × `penalty_scale` | **0.5, not curriculum-scaled, wrist × 4** |
| Jerk (second difference) | none | **0.2** |
| Hold-still reward | none | **+1.0 × held × calm** (low wrist and object speed) |
| After success | nothing to aim for | keep the **hold-still reward** running (calm = reward) |
| Observation noise | only at the last curriculum step | **its own schedule, from the start** |
| PPO loss | plain | **+ CAPS** (temporal and spatial, start λ ≈ 0.1–0.5, sweep) |

- A **fresh run** is cleaner than fine-tuning the 18 M policy, because the meaning of the wrist
  action and the observation change.
- **Done when:** in the eval suite, rest jitter < 0.2° for the fingers and < 0.5 mm per step for the
  wrist while holding, no FFT peak at 4–10 Hz, and success no worse than the baseline.

### Step 3: Motion layer and servo model (≈ 1–2 weeks)
- **Set-point generator** on the 4 ms physics tick (critically damped, per joint) in both envs.
  Train with it.
- **Hold latch** state machine (reach / grasp / hold / release) with hysteresis, logged.
- **Servo model:** delay 1–3 physics steps, 1024-tick quantisation, deadband, randomised gains.
  Later, the real numbers from system identification.
- If jitter is still there: **coloured exploration noise** (β ≈ 0.5) and **Lipschitz / spectral
  normalisation** on the actor.
- **Done when:** the policy stays calm with servo delay and noise on, at full domain randomisation.

### Step 4: Smooth by design: teacher → chunked student (≈ 1 month)
- The calm PPO policy becomes a **teacher**. It produces thousands of grasp demos in sim for free.
- Train a **student** that outputs **action chunks** (ACT or a flow policy, about 8–16 steps) and
  sees only what the real hand will have (servo positions and load, later a camera).
- Use **real-time chunking** at inference so chunk boundaries don't jerk.
- Then **bounded residual RL** (ResiP) or **DPPO** on top of the student, for robustness.
- **Done when:** the student matches the teacher's success and is smoother (lower FFT score).

### Step 5: The real hand (when V1 is built)
- Firmware: the same set-point generator at 100–200 Hz, command deadband + hysteresis, SCS0009 dead
  zone 1–3 units, moderate P, load/temperature limits, watchdog (torque stays off at boot).
- **System identification** of the SCS0009 → servo model in sim → retrain/fine-tune.
- **Hardware tests:** 1 h idle drift, 100× A↔B repeatability, 30 s hold test, endurance cycles.
- Later, **HIL-SERL-style learning**: a human corrects the robot when it goes wrong, and the
  corrections become training data.
- **Done when:** rest jitter < 1° (≈ the servo deadband), and a hold test of 100 grasps with
  0 drops.

### Step 6: Reliability machine (from now on, growing)
This is our version of Tesla's approach, at a small scale:
- **Regression suite:** fixed seeds × objects × both hands × randomisation levels. Every new policy
  must pass it, and a drop in any bucket blocks it.
- **Shadow mode:** a new policy runs next to the working one (sim, later real), and only replaces it
  after winning on the suite.
- **Data engine:** every failed episode is saved (seed, object, state) and trained on more often.
- **Automatic domain randomisation:** widen ranges (friction, mass, delay, servo gains) only when
  the policy succeeds.

### Step 7: Toward cooking and laundry
- **Two-brain architecture** (Helix / GR00T style): a slow language-vision model decides the next
  step ("pick up the pan"), and the fast smooth policy does it.
- **Skills one at a time**, each gated by the regression suite: pick and place → drawers → wipe →
  pour → fold a towel → tool use.
- **Learn from corrections** (π\*0.6 style): deploy, log, correct, retrain.
- **Hardware blocker:** fingertip force is ~3.4 N against 45–80 N in leading hands
  (`research/references/hands/`). Cooking and tool use need that fixed, plus touch sensing for
  slip detection.

---

## 4. Creative ideas

- **A "calm budget" reward:** each episode starts with a fixed amount of allowed motion energy.
  Motion that serves the task is cheap; wasted motion drains the budget. This is a constraint, not
  a weight to tune (similar to the DeCap soft barrier, arXiv 2609.24552, unverified).
- **Let the policy choose its stiffness:** add one "grip firmness" output. A firm grip while lifting
  and a soft hand while reaching, like a human (VICES, arXiv 1906.08880).
- **Freeze test in every eval:** after success, the policy must hold the object for 3 s with a
  target rest jitter. A grasp is only counted as a success if it passes. "Calm" then becomes part of
  what success means.
- **Smoothness from humans:** retarget the webcam teleop recordings and use them as demos, so the
  student copies human movement style (humans move with minimum jerk).
- **Policy "heartbeat" monitor:** live FFT of the action stream on the robot. If energy appears at
  4–10 Hz, the safety layer softens the gains and logs the event as data for the data engine.

---

## 5. Targets (the "calm" spec)

| Metric | Sim target | Real target |
|---|---|---|
| Finger shaking while holding (`fshake`) | < 0.1° per step | < 1° (≈ servo deadband) |
| Wrist shaking while holding (`wshake`) | < 0.3 mm per step | < 1 mm per step |
| FFT peak 4–10 Hz in actions | none | none |
| Success (95 % CI lower bound) | ≥ 90 % per object, both hands | ≥ 90 % over 100 trials |
| Hold test (3 s, sim / 30 s, real) | 0 drops in 200 | 0 drops in 100 |
| Time to settle after grasp | < 0.5 s | < 1 s |

---

## Sources (main)

Full lists with notes on what was read in full and what is uncertain are in `notes/`.

- CAPS: https://arxiv.org/abs/2012.06644
- Lipschitz-constrained policies: https://arxiv.org/abs/2410.11825, spectral norm variant https://arxiv.org/abs/2504.08246
- Coloured noise in PPO: https://arxiv.org/abs/2312.11091; pink noise: https://openreview.net/forum?id=hQ9V5QN27eS; gSDE: https://arxiv.org/abs/2005.05719
- DeepMind soccer: https://arxiv.org/abs/2304.13653; legged_gym: https://github.com/leggedrobotics/legged_gym
- DPPO: https://diffusion-ppo.github.io/; ResiP: https://arxiv.org/abs/2407.16677; RL-100: https://arxiv.org/abs/2510.14830
- HIL-SERL: https://arxiv.org/abs/2410.21845; π\*0.6: https://arxiv.org/abs/2511.14759; real-time chunking: https://arxiv.org/abs/2506.07339
- ACT: https://arxiv.org/abs/2304.13705; GR00T N1: https://arxiv.org/abs/2503.14734; Helix: https://www.figure.ai/news/helix
- DextrAH: https://arxiv.org/abs/2412.01791; VICES: https://arxiv.org/abs/1906.08880; Ruckig: https://github.com/pantor/ruckig
- SCS0009 registers: https://github.com/ftservo/FTServo_Arduino (SCSCL.h), https://github.com/pollen-robotics/rustypot/pull/132
- Evaluation statistics: https://www.therobotreport.com/nvidia-shares-how-evaluate-general-purpose-robot-policies-real-world-deployment/

Caveats: some numbers come from search summaries, not full papers (marked in `notes/`). Tesla's and
1X's internals are mostly not public. The SCS0009 register map is partly community-reversed and must
be checked on hardware.
