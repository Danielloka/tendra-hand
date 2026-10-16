# Why the 18M-step Brax grasp policy jitters: diagnosis (2026-10-06, read-only)

No repo files were edited. Scripts are in `../scripts/`: `gains.py`, `exp1.py`,
`exp2.py`, `exp3.py`, `jitter_probe.py`.

**The trained policy is not on the laptop.** `runs/` and `~/tendra-data` hold only the CPU runs
and smoke or test pickles, and there are no `policy_*.npz` / `params_*.pkl` files from gpu3. So I
measured the plant and the reward instead. `jitter_probe.py` is ready for when
`policy_best.npz` is downloaded from Drive (it was tested on a random-weight policy):

    uv run python research/experiments/2026-10-06-smooth-control/scripts/jitter_probe.py policy_best.npz runs/colab/tendra_gpu.npz --episodes 5 --objects cylinder cube

## What the experiments show (CPU GraspEnv, nominal model, right hand, cylinder)

| Experiment | Result |
|---|---|
| A. Zero action for 104 steps (`exp1.py`) | Perfectly still: wrist 0.000 mm/step, arm \|v\| 0.001 rad/s. Zero wrist action = "stay" (the velocity is integrated into the mocap target). Zero synergy = the rest posture. |
| A2. Holding a scripted grasp with a **constant** action (synergy close = +1.0, others 0, residual 0) from demo frames 60 and 71 (`exp2.py`) | Holds 100%. Wrist 0.001 mm/step, object 0.06 mm/step (slow slip, −0.5 mm in 2 s), fingers 0.001 rad/step. **Physics, IK and servos do not jitter by themselves.** |
| B. i.i.d. noise on the 6 wrist actions | std 0.05 → 1.0 mm/step wrist motion (22 mm/s). std 0.1 → 2.5 mm/step. std 0.3 → 8.3 mm/step. Very sensitive. |
| C. i.i.d. noise on the 22 finger actions | std 0.1 → finger speed 0.63 rad/s, wrist unaffected. |
| D. Arm step response to one full wrist action (14.4 mm target jump) | Peak 19.9 mm at 108 ms: **38% overshoot** (damping ratio ζ ≈ 0.29), settles after ~0.4 s, rings at ~4.8 Hz. Turn step 5.5° → 6% overshoot. |
| E. A proportional "policy" holding the wrist on a point from the observed wrist position, a = g·err/(wrist_speed·dt) (`exp3.py`) | g 0.3: still (0.02 mm). **g ≥ 1.0: limit cycle of 17–20 mm at 4.8 Hz even with no noise.** g 0.6 + 3 mm obs noise: 9 mm error and 11 mm/step jitter. With a velocity (D) term, noise-free is stable up to g 1, but with 3 mm noise it jitters 3–8 mm/step at 7–10 Hz. |

Conclusion: the system is quiet when the action is constant. The jitter is the policy's actions
changing from step to step, and several design choices (1 to 3 below) make that likely and
strongly amplified.

## Ranked root causes

### 1. A high-gain wrist action on an underdamped arm leaves almost no stability margin (very likely, the main visible jitter)
**Evidence**
- `grasp_env_jax.py:82-85, 540-549`: |a| = 1 moves the target 0.3 m/s × 0.048 s = **14.4 mm per step** and turns it 2 rad/s × 0.048 = **0.096 rad (5.5°) per step**. At a fingertip ~0.17 m from the wrist, that is ~16 mm. Lead limits are 30 mm and 0.3 rad (17°).
- Arm servos (OpenArm model, printed by `gains.py`): shoulder kp 230, kv 2.7 (+ joint damping 1.0), elbow kp 190, kv 2.2. Measured overshoot is 38% (ζ ≈ 0.29) with a ~4.8 Hz ring, which is inside the 21 Hz policy's bandwidth (Nyquist 10.4 Hz). The log entry from 2026-10-02 also noted "~30% overshoot".
- Experiment E: any controller that corrects ≥ 100% of the position error per step (what a greedy RL policy learns during reaching) goes into a 4.8 Hz limit cycle of about 2 cm. At 60% gain, 3 mm of noise already gives 11 mm/step. Experiment B: wrist action noise of only 0.05 gives 1 mm/step of visible motion.
- The policy also sees `wrist_vel` and `arm_v` (`grasp_env_jax.py:680-681`). These carry the arm's 4.8 Hz ringing back into the policy and close a loop with the underdamped plant.

**Verify**: `jitter_probe.py`. A peak of 4–5 Hz in the wrist action's FFT, and wrist motion of several mm/step while held, mean this cause. Also replay the policy with `wrist_speed`/`wrist_turn_speed` halved at test time: the jitter amplitude should drop, though the policy will be slower.

**Fix**
- Lower the action gain: `wrist_speed` 0.3 → **0.15** m/s and `wrist_turn_speed` 2.0 → **0.8** rad/s (≈7 mm and 2.3° per step). Optionally make `wrist_lead` 0.03 → 0.015 and `wrist_turn_lead` 0.3 → 0.15.
- Filter the wrist command like the fingers: keep a wrist velocity state `v ← v + 0.4·(a·speed − v)` in `info` (and the CPU env) and integrate `v`, not `a`. Put `v` in the observation (or replace `prev_action`'s wrist part with it).
- Damp the arm: raise the arm servos' kv (biasprm[2]) from −2.7/−2.2 to about **−9/−7** (target ζ ≈ 0.7; check with experiment D2: overshoot < 5%), or raise `dof_damping` on the 4 OpenArm joints. Do it in `scene.py`/the bundle export, so the CPU and GPU envs stay the same.

### 2. The reward does not pay for stillness, and the smoothness penalty is about 0 (very likely, the reason training never removed it)
**Evidence**
- `grasp_env_jax.py:639`: action_rate = −p·0.1·mean((a−a_prev)²). The curriculum starts at `penalty_scale` 0.3 (`curriculum.py:43`) and only raises it after all 3 objects pass 60% (`curriculum.py:221-231`). At 18M steps (chunks of 8M, so ≤ 2 decisions, both "add object") it must still be **0.3**, giving an effective weight of 0.03.
- Example: wrist actions jittering with std 0.1 (about 2.5 mm/step, clearly visible) cost 0.03 × (2·0.01·6/28) ≈ **1.3e-4 per step**. While holding, the policy earns about 9 per step (lift 4 + held 2 + opposition 1 + reach ~1 + contact 0.5 + tips 0.5). That is 0.001%, far below PPO's advantage noise. Even full ±1 bang-bang on every action costs 0.12/step (1.3%).
- There is no penalty on wrist, arm or object velocity or acceleration. `lift` is clipped at `lift_height` (`:633`), so above 6 cm nothing changes with motion. The success bonus is paid once (`:635`). The episode keeps running after success (`:515`; CPU `grasp_env.py:397-399`). So after success, ~50 steps are "free": jitter costs nothing as long as the object stays in hand.
- Effort (0.05 × 0.3) and residual (0.2 × 0.3) are also scaled down to near nothing.

**Verify**: in `progress.csv`, compare `eval/episode_action_rate` with `eval/episode_reward`. Expect the ratio to be < 0.1%.

**Fix**
- Take `action_rate` out of `penalty_scale` (or start `penalty_start` at 1.0 for it). Raise it to **0.5–1.0**, applied to the wrist dims with an extra ×4 (e.g. `0.5·mean(Δa_finger²) + 2.0·mean(Δa_wrist²)`).
- Add an action-acceleration term `−0.2·mean((a_t − 2a_{t−1} + a_{t−2})²)`. This needs `prev_prev_action` in `info`.
- Add a hold-still term while `held`: `+1.0·held·exp(−(|v_obj|/0.03)²)` or a penalty `−0.5·held·(|v_wrist_lin|/0.05)²` and the same for angular speed / 0.5 rad/s. This rewards a still grasp directly, which nothing does now.
- Apply these equally in `grasp_env.py:535` so the CPU and GPU envs stay identical (the parity tests will need the new terms).

### 3. Train/eval mismatch: the policy never saw observation noise or mass randomisation, but the viewers add them (likely, if the jitter was seen in the Colab video or `eval_grasp.py`)
**Evidence**
- GPU noise = `obs_noise × dr_level` (`grasp_env_jax.py:316, 479, 670`). DR is the **last** curriculum step (`curriculum.py:241-243`), and `State.dr_level` starts at 0.0 (`:65`), so at 18M steps training had **no** obs noise, no obs bias and nominal mass.
- `rollout_video.py:97-100` builds `TendraGrasp` with only `demo_prob`/`naconmax` overrides, so `dr_level` = **1.0** (default, `:97`). The video therefore adds 3 mm per-step noise, a 3 mm bias and mass ×0.5–2 that the policy never trained on.
- `eval_grasp.py:57-61` → `GraspEnv`: the CPU env **always** adds `obs_noise` 0.003 + bias (`grasp_env.py:329, 576`), even with `randomize=False`.
- Experiment E: 3 mm position noise turns a stable controller (gain 0.6, 0.03 mm) into 11 mm/step jitter. The noisy object position enters 24 actor features (`object_view`: p−root, R^T(p−root), p−centre, tips−p).

**Verify**: run the policy with `env.config.obs_noise = 0` vs 0.003 (`jitter_probe.py` does both and prints them side by side). In the video, pass `dr_level=0` in the overrides. If the jitter mostly goes away at 0 noise, this is it.

**Fix**
- Make every viewer match training: `rollout_video.film()` should read `dr_level` (and `objects`) from `curriculum.json` / the params record's `level`. `eval_grasp.load_brax` should set `cfg.obs_noise = 0.003 × dr_level` (add a `--obs-noise` flag).
- Training: give observation noise its own schedule that starts early (e.g. `obs_noise_level` 0.5 from chunk 0, independent of mass/friction DR). A policy trained with noise learns to filter it. This is also needed for the real hand.

### 4. Corrective behaviour learned under heavy exploration noise, with tanh saturation (medium)
**Evidence**: PPO trains on `tanh(loc + σ·ε)` (`policy.py:189-194`; entropy 1e-3, `train_brax.py:81`). With σ ≈ 0.3–0.7 on the wrist dims, the training wrist random-walks ~4–10 mm/step (experiment B), so the mean policy learns large, fast corrections. With the deterministic mean (`policy.py:194`, `train_brax.py:395`, `eval_grasp.py:81`) those high-gain corrections act on tiny errors: the high-gain loop of cause 1. Saturated `loc` (|tanh| > 0.95) gives bang-bang wrist moves of 14 mm/step.

**Verify**: `jitter_probe.py` prints the "saturated" share and the policy's own std per group. A wrist saturation share > 10% while holding, or std > 0.3 at 18M, supports this.

**Fix**: keep the deterministic mean at deployment (that is already done). Smoother exploration: cap the log-std, or use lower initial std (scale bias init of the std head) and time-correlated noise (pink / OU noise or gSDE, if switching away from Brax's default). Mostly, causes 1 and 2 remove the incentive. Raising entropy is not the fix here.

### 5. The `prev_action` input can self-excite a period-2 flicker (medium-low)
**Evidence**: `info["prev_action"]` is part of the actor observation (`grasp_env_jax.py:686`), and the penalty on its change is about 0 (cause 2). If ∂a_t/∂a_{t−1} has an eigenvalue < −1 at holding states, the policy alternates every step (10.4 Hz).

**Verify**: `jitter_probe.py` prints the spectral radius of d a/d prev_action at held states (with the tanh slope), plus FFT peaks. A peak near 10.4 Hz with a negative dominant eigenvalue of magnitude > 1 confirms it.

**Fix**: a stronger action_rate penalty (cause 2) fixes it during training. Alternatively, feed the filtered command state (`finger_target`, which is already there, and a wrist velocity state from fix 1) instead of the raw `prev_action`.

### 6. Fingers: absolute posture targets and a weak filter (medium, smaller visual effect)
**Evidence**: `finger_command` (`grasp_env_jax.py:556-558`) maps the action to an absolute posture. To hold still, the policy must output the same non-zero synergy values every step. The power grasp is close = +1.0, so it saturates (a constant hold is stable, experiment A2), but the other 5 synergies and 16 residuals sit inside the range, and each step's variation goes straight to the targets. One close unit = 0.91 rad at the MCP. The EMA α = 0.4 (≈ 1.7 Hz corner) only cuts i.i.d. noise std by half (√(α/(2−α)) = 0.5). Experiment C: finger action noise of 0.1 → 0.63 rad/s finger motion. Finger jitter also shakes the object, and the policy then reacts with the wrist.

**Verify**: `jitter_probe.py` synergy/residual `mean|da|` while held.

**Fix**: `finger_smoothing` 0.4 → **0.25** (corner ≈ 1 Hz, still fast enough for a 5 s grasp), and include synergy/residual in the stronger action_rate penalty. Optionally use a delta-posture action for the fingers after the grasp is closed (harder; leave it for later).

### 7. Observation normaliser blow-up on features that never varied in training (low, but check)
**Evidence**: Brax clips the normaliser std at **1e-6** (`brax/training/acme/running_statistics.py:124, 233`). Any feature that was constant in training is multiplied by about 1e6 when it differs even slightly at evaluation, for example the object one-hot for an object that was never trained (the ball, if it was added late or not at all), or `obj_size` with `--randomize` (size DR is CPU-only). `eval_grasp.py` evaluates **every** scene object by default (`:133`).

**Verify**: in `policy_best.npz`, list the `std` entries < 1e-3 and check which features they are.

**Fix**: evaluate only the trained objects (`--objects cylinder cube`). In `BraxPolicy._logits`, clip the normalised input to ±5 (training never clips, so this is only a safety net). Or floor `std` at 1e-2 at export.

### 8. Ruled out: physics, the IK loop or the servos oscillating on their own
Zero action (A) and a constant holding action (A2) are still to < 0.01 mm/step. IK is refreshed every 25 ms from the measured q (6 iterations, tolerance 3e-4), the same on GPU and CPU (`scene.py:560-565`, `grasp_env_jax.py:387-400`, `export.py:49`). Gravity compensation is on for the arm. Object slip is slow creep (0.25 mm/s), not jitter. The finger tendon actuators (kp 2.9–3.9, kv 0.017–0.024) are well damped.

## Suggested order
1. Download `policy_best.npz` and run `jitter_probe.py` (1 min). Read the noise 0 vs 3 mm rows and the FFT peak: they separate cause 3 (noise) from 1 (4.8 Hz arm loop) and 5 (10.4 Hz flicker).
2. Cheap, no retraining: fix the viewers' `dr_level`/noise (cause 3) and evaluate only the trained objects (cause 7).
3. Retrain with: wrist_speed 0.15, turn 0.8, a wrist command filter (0.4), arm kv ×3 (check the step response), action_rate 0.5 (wrist ×4) without penalty_scale, a jerk term of 0.2, a hold-still reward of 1.0, finger_smoothing 0.25, and an obs-noise schedule from the start. Change the CPU env at the same time and re-run the GPU↔CPU parity tests (`uv run --group gpu pytest -m gpu -k "not smoke"`). Re-export the bundle and the kit (`make_train_notebook.py`, new `--run`).
