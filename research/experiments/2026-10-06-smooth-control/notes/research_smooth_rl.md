# Making RL policies smooth and steady: research notes (2026-10-06)

Method: web search + page fetches. Several PDFs could not be text-extracted, so some numbers come from HTML versions or search summaries. [UNCERTAIN] marks anything I could not verify. Reward weights from other codebases depend on their reward scaling (per-step vs per-second, dt multipliers), so treat them as orders of magnitude only.

Our setup for reference: Brax PPO, 20 Hz, tanh-Gaussian, action = 6 wrist velocity + 6 synergies + 20 residuals, EMA alpha 0.4 on finger targets, action-rate penalty 0.1.

---
## 1. CAPS (Mysore et al., ICRA 2021)
- What: add two regularisers to the actor loss. Temporal: consecutive actions should be similar. Spatial: nearby states should give similar actions.
- Why it works: it directly flattens the state-to-action map, so observation noise and small state changes cannot flip the action. A reward penalty only reacts after the shake has happened and is traded against task reward. CAPS is a loss on the policy network itself.
- Formula (verified from paper HTML): J_CAPS = J - lambda_T * L_T - lambda_S * L_S.
  L_T = D(pi(s_t), pi(s_{t+1})), L_S = D(pi(s), pi(s_bar)) with s_bar ~ N(s, sigma). D = Euclidean (L2) distance, using the deterministic mean action. sigma = expected measurement noise.
- PPO: add the terms to the PPO actor loss, computed on minibatch pairs (s_t, s_{t+1}) taken from consecutive rollout steps (need next-obs in the batch; mask episode boundaries) and on s plus Gaussian-perturbed s.
- Weights: the paper's main text does not list them (ablation is on the project site) [UNCERTAIN]. Start small and sweep: lambda_T ~ 0.1-1, lambda_S ~ 0.1-1 with L2 distances on actions in [-1,1]. Follow-ups found that temporal smoothness has the larger effect (alphaxiv summary of follow-ups, not read in full). Dual-Window Smoothing paper uses lambda_S = 0.10 by default (different method, a hint of scale).
- Evidence: lower FFT smoothness score on Pendulum, LunarLander, Reacher, Ant with DDPG/SAC/TD3/PPO. Real drone: ~80% less power use vs Neuroflight at similar tracking, 100% repeatable training.
- Cost/risk: extra forward pass (perturbed state), small. Too large a weight gives sluggish policies. Needs actions normalised so weights transfer.
- Source: https://arxiv.org/abs/2012.06644 , project http://ai.bu.edu/caps/

## 2. Lipschitz-constrained policies (LCP, Chen/He et al. 2024) and spectral normalisation
- What: penalise how fast the policy output changes with the observation, via a gradient penalty on the actor.
- Why: a Lipschitz-bounded network cannot respond with large, fast action changes to noisy observation wiggles, so the closed loop has bounded bandwidth. Unlike a filter it acts inside the network and does not slow exploration.
- Formula: fetched summary gave "max J - lambda_gp E||grad_s log pi(a|s)||^2" but the paper's penalty is most likely on the gradient of the action (mean) w.r.t. observation [UNCERTAIN, check the paper]. Implement with autograd (jax.grad of the mean action w.r.t. obs, squared norm, averaged over a minibatch; in JAX use a jvp/vjp with random vectors to avoid the full Jacobian).
- Weights: lambda_gp = 0.002 chosen; 0.001 still jittery; 0.01 "sluggish", lower return (humanoid, their reward scale).
- Evidence (sim, action jitter / return): LCP 3.21 / 26.0; smoothness reward 5.74 / 26.6; low-pass filter 7.86 / 25.0; none 42.19 / 28.9. Real robots: jitter ~1.1-1.7 rad/s^3 over terrains. Claims LCP beats filtering because filters damp exploration.
- Spectral normalisation variant (arXiv 2504.08246): constrain spectral norm of each weight matrix; reported comparable to gradient penalty with less GPU memory and easier parallel training. Cheaper for Brax/JAX (no double backward). Details/numbers not extracted [UNCERTAIN].
- Cost/risk: gradient penalty adds a second-order backward pass (memory, ~1.3-2x actor update time); SN is cheap but caps capacity (tanh output + SN needs scale care). Too strong a constraint blurs grasp-contact decisions (sharp switches such as "close now").
- Sources: https://arxiv.org/abs/2410.11825 , https://lipschitz-constrained-policy.github.io/ , https://arxiv.org/abs/2504.08246

## 3. Exploration noise: gSDE, pink/colored noise
Why white Gaussian noise trains jittery policies: PPO samples independent noise at every step, so every step the executed action is mean + fresh random jitter, and the policy is optimised to do well under this jitter (and ends up needing high-frequency corrections). The jitter averages out in the dynamics, so there is no pressure to remove it. Also, a state-independent learned std (typical in Brax PPO) keeps a constant noise floor even when the arm should hold still, and the stochastic policy is what you see if you watch with sampled actions.
- gSDE (Raffin, Kober, Stulp 2020/CoRL 2021): noise = theta_eps . latent features(s), with theta_eps resampled every n steps (n=1 step-based, n=episode = original SDE). Exploration is state-dependent and correlated in time; the noise is a deterministic function of state between resamples. Paper: jerky real-robot behaviour disappears; for SAC an interval of 8 gave good smoothness/performance trade-off; for PPO the interval needs tuning. Trained on a tendon-driven neck, a quadruped, an RC car. Source: https://arxiv.org/abs/2005.05719
  Cost: not in Brax PPO, needs custom noise head; risk: interaction with PPO ratio (sampling fixed noise matrix within an interval; log-prob must be computed given the same matrix).
- Pink/colored noise (Eberhard et al., ICLR 2023): action noise with 1/f^beta spectrum, beta = 1 for off-policy (SAC/TD3/MPO). Pink "significantly outperforms white, OU" and is recommended as default for off-policy. Source: https://openreview.net/forum?id=hQ9V5QN27eS , code https://github.com/martius-lab/pink-noise-rl
- Colored noise in PPO (AAAI 2024, https://arxiv.org/abs/2312.11091): best beta = 0.5 averaged over 16 envs; preferred correlation grows with more parallel envs; on-policy ratio needs the noise to stay near Gaussian per step (it does) but long correlation raises variance. Implementation: generate colored noise sequences eps_t (unit-variance per step), use a = tanh(mu + sigma*eps_t); log-prob unchanged (marginal still Gaussian) so the PPO ratio still works approximately [the paper does this, UNCERTAIN on exact handling].
  Cost: tiny. Does NOT remove jitter that comes from the mean (obs noise sensitivity); it mainly stops noise-induced jitter and makes exploration purposeful. Moderate expected gain on its own.
- Hold-still point: also lower the final std: entropy coefficient small/decayed, bound log_std (e.g. max std 0.3 in action space late in training).

## 4. Action filtering and parameterisation (with reference weights)
- Exponential filter in the loop: u_t = (1-a) u_{t-1} + a * a_t. DeepMind soccer (https://arxiv.org/abs/2304.13653): u_t = 0.8 u_{t-1} + 0.2 a_t at 40 Hz, joint-position targets, deterministic mean action at deploy, random 10-50 ms delays, pushes. OpenAI Dexterous (https://journals.sagepub.com/doi/full/10.1177/0278364919887447): EMA coefficient 0.3 applied to targets (per 80 ms), shown in search summary [UNCERTAIN on exact wording]. Our alpha 0.4 at 20 Hz = 50 ms is a weaker filter than both: time constant ~ 50ms*(1-0.4)/0.4 ~ 75 ms vs soccer 25ms*0.8/0.2 = 100 ms. Lower alpha (0.2-0.3) is an easy knob but adds lag.
  LCP paper found filters give worse jitter (7.86 vs 3.21) and hurt exploration; but they are the only guaranteed hard bound. Use filter + learned smoothness, not either alone.
- Delta/velocity parameterisation: policy outputs change of target (target += scale * a). Natural low-pass: a zero output means hold, so rest is "do nothing" (and the policy's mean noise integrates to a random walk, so combine with an integral-free hold penalty/leak or clip). We already use wrist velocity; the 20 residuals + synergies are probably absolute. Making fingers delta-based makes "stay still" the default.
- Penalty weights stated in papers (scaling differs, compare ratios):
  - legged_gym default (ANYmal config in repo): action_rate -0.01, torques -1e-5, dof_acc -2.5e-7, lin_vel_z -2.0, ang_vel_xy -0.05, tracking_lin_vel +1.0; action_scale 0.5; decimation 4. (https://github.com/leggedrobotics/legged_gym , legged_robot_config.py)
  - Rudin et al. 2022 (https://arxiv.org/abs/2109.11978): action rate 0.25 dt, joint motion 0.001 dt, torques 2e-5 dt, tracking 1 dt / 0.5 dt, collisions 0.001 dt (50 Hz). Action rate is large relative to tracking here.
  - Humanoid soccer striker paper (https://arxiv.org/abs/2512.06571): action rate ||a_t - a_{t-1}||^2 weight -1.0 among 14+ regularisers, but made a constraint (see below).
  - Unitree/Berkeley humanoid: could not extract numbers [UNCERTAIN, only sources named: https://arxiv.org/abs/2407.21781].
  Ours: 0.1 on action rate; ratio to a typical task reward unknown, but "rests when it should" suggests it is too small or applied to the wrong quantity (see section 5).
- Second difference (jerk/acceleration) penalty: ||a_t - 2a_{t-1} + a_{t-2}||^2. DeCap paper found going from first-order to first+second-order terms lowered smoothness metrics (search summary, https://arxiv.org/abs/2609.24552). First-order alone permits ping-pong oscillation at constant amplitude only if amplitude cost is small; second-order penalises high-frequency oscillation harder (a +-x zigzag has cost ~4x^2 per step vs 2x^2... for first diff). Suggested start: second-diff weight about the same size as first-diff.
- Constraint instead of weight: DeCap (https://arxiv.org/abs/2609.24552) enforces limits per joint group with a bounded soft barrier phi = 1-(1-u)^kappa, kappa = 2, activating at 70% of limit; limits (humanoid upper body) action rate 5.0 rad/s, accel 20 rad/s^2. Real G1: 2.5x lower upper-body action rate, 2.18x lower accel than reward-based smoothing. Soccer striker paper: constraint threshold = 1.5x the walking gait's average regularisation cost, solved with N-P3O. Advantage: no weight tuning, no penalty that blocks fast needed motions (e.g. the moment of grasp closure). Cost: CMDP machinery; a simpler cousin is a hinge penalty max(0, |da| - d_max)^2.

## 5. Hold / stillness
- Reward zero motion when the task phase says so: when object is lifted and held (or goal reached), add +r * exp(-||dq_target||^2/s), or penalise ||a||^2 and ||dq||^2 only in the "hold" phase (gated by a task-phase flag). legged_gym has a `stand_still` reward term (penalise joint deviation at zero command; default scale 0) [seen in the config listing, definition not read]. For us a "settled" gate: grasp zone reached and contacts stable.
- Make "no movement" a representable action: delta parametrisation, deadband on the policy output (|a| < eps -> 0) after training (a deadband at deploy is a cheap fix; apply in training too so the policy learns to exploit it), tanh saturation avoided.
- Observation noise causes jitter: the policy tracks noise. Fixes: CAPS spatial term with sigma = obs noise; LCP; train with realistic noise but also feed the policy history/filtered velocities; remove high-frequency observation inputs (contact force, finger velocity) or low-pass them; velocity observations are the usual culprits (differentiated positions are noisy).
- Deterministic action at deploy: DeepMind soccer deploys the Gaussian mean. Do the same for eval. Check that the shake persists with the mean action: if not, it is exploration noise (sections 3 and std bound); if yes, it is the mean's sensitivity to observations (CAPS/LCP/obs filter).
- Entropy/std: tanh-Gaussian PPO with fixed-ish std keeps noise on rest phases; anneal std or use a state-dependent std and add an std-penalty during hold.

## 6. Control frequency and action repeat
- Search result: lower action frequency makes point-to-point stop-go motion (velocity loss, discontinuity); high frequency gives dense targets and smooth control (https://arxiv.org/html/2605.19592, preprint, not verified in depth). Jitter measured as ||a_t - a_{t-1}||.
- Position targets plus a servo PD act as a physical low-pass; velocity commands integrate the noise (random-walk drift) but "zero = still" which is good for rest. Mixed is fine: keep fingers on filtered position targets, wrist on velocity with a deadband.
- Dual-Window Smoothing (same paper): policy action a_t is expanded to h=3 execution steps with a modulation window (zero-order hold or decay) + actor regulariser with lambda_S = 0.10. Reported AFR 0.56 -> 0.10 on Reacher-Hard (-80%), steering jitter -88.7% in driving. Comparable in spirit to action repeat at 20 Hz -> effective 10 Hz or to chunking. Risk: a coarser effective control rate hurts precise grasp closure; preprint, treat numbers as unverified.
- Action repeat 2 (10 Hz policy, hold target between) halves the number of decisions and shrinks jitter by roughly the repeat factor in time but not in amplitude; cheap to test in the env (also speeds PPO samples/second).

## 7. Curriculum and fine-tuning for smoothness
- Common practice: start penalty small so that the task is learned (large early penalties produce "do nothing" policies), then ramp up (e.g. linear from 0 to full over 20-50% of training, or scale by curriculum level as in legged_gym `reward_curriculum`/ Rudin's terrain-based difficulty). [No single paper quantified this; general practice from legged robotics, UNCERTAIN as a cited result.]
- For our already-trained 18M-step policy: continue PPO (warm start actor+critic, reset optimiser, lower learning rate) with the new smoothness terms ramped up from 0 over the first ~20% of fine-tuning; add CAPS/LCP as loss terms (they do not change the reward scale so the critic stays valid; reward-weight changes invalidate the value function and need a critic re-fit warm-up with frozen actor for a few updates).
- DeCap result: removing the smoothness penalty improved tracking but raised action jerk a lot, "reward-level smoothness penalties are conservative for tracking but necessary" (search summary of https://arxiv.org/abs/2609.24552). Expect a modest task-return cost; verify success rate on fixed seeds.
- Optional distillation: train a student MLP on the policy's mean actions with a smoothness loss (supervised, cheap). Not found in literature search specifically; standard DAgger-like practice [UNCERTAIN as a cited method].

## 8. Metrics
- CAPS smoothness score: Sm = (2/(n f_s)) * sum_i M_i f_i, FFT amplitude M_i at frequency f_i over an action sequence; lower = smoother (weights high frequencies; picks up jitter and ignores slow motion). Implement per action dimension, average over episodes, compute on deterministic rollouts. https://arxiv.org/abs/2012.06644
- Action jitter / rate: mean ||a_t - a_{t-1}|| (or rad/s on joint targets) and second difference (jerk); LCP reports "action jitter" in rad/s^3.
- Rest-specific: mean |dq| and std of joint targets during the hold phase (last N steps after lift), plus fraction of time |da| < eps. This is the metric closest to the owner's complaint.
- SPARC (spectral arc length of the speed profile; more negative = less smooth, independent of amplitude/duration), from biomechanics (https://jneuroengrehab.biomedcentral.com/articles/10.1186/s12984-018-0398-3); also LDLJ (log dimensionless jerk). Use for endpoint/wrist speed during reaches, not for jitter in a still hold (SPARC needs a movement with a clear start and end).

---
## Ranked: 8 most promising for Tendra (PPO, 20 Hz, tanh-Gaussian, shaky when it should rest)
1. Diagnose first, then deterministic mean + std bound + eval metrics (free): run with the mean action; measure FFT score and hold-phase |dq|. Tells whether shake is noise (fix by std/exploration) or mean sensitivity (fix by CAPS/LCP).
2. Hold-phase gated stillness reward + deadband: reward low target change/velocity when the object is held or the goal reached; deadband on tiny outputs. Direct hit on "rests when it should". Cost low.
3. CAPS temporal (+spatial with sigma = obs noise) loss on the PPO actor: simple, proven (drone, 80% less power), no reward-scale change so fine-tuning from the 18M-step policy works. Start lambda_T ~ 0.1-0.5 and sweep.
4. First+second-difference penalties on the filtered target (not only action rate), ramped up; delta-parameterise fingers (hold = zero action). Ramp from 0 over fine-tuning.
5. Lower EMA alpha (0.4 -> 0.25) or two-stage filter, optionally action repeat 2: guaranteed hard bound, instant to test; accept some lag. Risk: LCP found filters hurt exploration, so apply only at fine-tune/deploy first.
6. Colored noise (beta ~0.5) for PPO exploration and an annealed/bounded std: cheap, removes noise-induced jitter during training.
7. Lipschitz penalty or spectral normalisation on the actor (LCP lambda ~0.002 at humanoid scale; SN cheaper in JAX): stronger robustness to observation noise than CAPS; try if CAPS is insufficient.
8. Constraint-style soft barrier (DeCap, kappa 2, onset at 70% of an action-rate limit) instead of a fixed weight: avoids tuning and does not block fast grasp closure; implement as a hinge penalty first. gSDE is lower priority (needs custom PPO noise head).

Also: low-pass/denoise velocity and contact observations; check that observation noise in training matches the real sensors.
