# How leading teams get smooth, steady, reliable robot behaviour (research, 2026-10-06)

Legend: [S] = stated in a source I fetched or saw in search results (URL given). [M] = from my own background knowledge, not verified in this session, treat as uncertain.

## The big picture (one paragraph)
Smoothness and reliability almost never come from a clever low-level controller alone. They come from four things stacked: (1) the policy outputs a short *trajectory chunk* instead of one action per tick, so motion is planned a bit ahead and cannot jitter step to step; (2) a *fast low-level layer* (50-200 Hz) tracks that trajectory while a slow "brain" (5-10 Hz) decides what to do; (3) the policy is trained from *human demonstrations* (which are smooth by nature) and *then* refined with RL on the real thing; (4) reliability is *measured* (many trials, interventions per hour) and fed back into a *data loop* (fleet or robot rollouts, failures become training data). A PPO policy trained from scratch in sim (what Tendra has) lacks 1, 2 and 3, which is why it is shaky.

---

## 1. Imitation policies and why they are smooth

### ACT (Action Chunking with Transformers) / ALOHA
- What: a transformer that predicts a chunk of future joint targets from camera images; trained on ~50 teleoperated demos per task. Data recorded at 50 Hz [S: https://arxiv.org/pdf/2304.13705 via search snippet]. Chunk size 100 and ensembling weight exp(-0.01 i) are from the paper [M].
- Why smooth: predicting k steps at once means the policy commits to a plan; *temporal ensembling* averages overlapping chunk predictions for the current step, which low-pass filters the output. Chunking also cuts compounding error (fewer decisions per task).
- Numbers: public repo says ~90% on sim transfer-cube and ~50% on sim insertion [S: https://github.com/tonyzhaozh/act via search]. Real-task numbers (e.g. 80-90% on slotting a battery) [M].
- Caveat: temporal ensembling can blend incompatible modes into an invalid action [S: search snippet, arXiv 2605.24433 area].
- Tendra can copy: yes. ACT is small (tens of M params), trainable on a Colab GPU. Needs demonstrations, which Tendra can produce *in sim* (scripted grasps + webcam teleop already exist; datasets at 30 fps).
- Source: https://arxiv.org/pdf/2304.13705

### Diffusion Policy / flow matching
- What: generate the action chunk by denoising noise (diffusion) or integrating a flow (flow matching), conditioned on observations. Handles multimodal demos (two valid ways to grasp) without averaging them into a bad middle. Chunk typically 8-16 actions executed receding-horizon [M].
- Why smooth: a whole trajectory is sampled jointly, so it is internally consistent; action-space is trajectory not per-step noise.
- Relevant comparison: with DPPO, a Gaussian PPO policy had 88% sim success but failed entirely on the real robot due to "volatile and jittery behaviour", while the diffusion policy fine-tuned with PPO got 80% zero-shot sim-to-real [S: https://diffusion-ppo.github.io/ , search summary]. This is directly the Tendra symptom.
- Tendra can copy: Diffusion Policy / flow policy training is feasible on Colab; inference is OK on a laptop CPU with few denoising steps (or consistency distillation, see RL-100).

### π0 / π0.5 / π0-FAST (Physical Intelligence)
- π0: VLM backbone + flow-matching "action expert", outputs chunks of 50 actions at up to 50 Hz [M, from the openpi/π0 paper; chunk of 50 also stated in RTC summary: "e.g. 50 steps" S: https://huggingface.co/docs/lerobot/rtc]. π0-FAST uses frequency-space (DCT) token compression to make autoregressive action tokens [M].
- π0.5: co-trained on ~400 hours of mobile-manipulator household data from ~100 homes plus web and other robot data; does 10-15 minute tasks like cleaning a kitchen/bedroom in unseen homes [S: https://www.pi.website/blog/pi05, search summary].
- π*0.6 + RECAP: a VLA improved from its own deployment experience using an advantage-conditioned policy plus human corrections. Tasks: laundry folding (T-shirts/shorts, 11-item diverse set), espresso making, factory box assembly. Roughly doubled throughput and halved failure rate on the hardest tasks; ran 13 hours straight making espresso, over 2 hours folding novel laundry in new homes without interruption [S: https://arxiv.org/html/2511.14759 (fetched)]. Pre-training data "tens of thousands of hours" [S, same].
- Tendra can copy: not the scale. Can copy the *recipe*: pre-train on demos, then collect own rollouts + corrections, train a value/advantage signal, condition the policy. The weights of π0/π0.5 are open (openpi) [S: https://github.com/Physical-Intelligence/openpi], but Tendra's 16-servo hand is not in their data; fine-tuning would need its own demos and a GPU much larger than free Colab.

### Real-Time Chunking (RTC)
- What: an inference-time trick for diffusion/flow chunk policies. While executing the current chunk, the next one is generated asynchronously; the first actions that will certainly run are "frozen" and the rest "inpainted" so the new chunk continues smoothly from the old. No retraining. Fixes pauses and jerks at chunk boundaries caused by inference latency [S: https://arxiv.org/abs/2506.07339 (fetched), NeurIPS 2025, Black, Galliker, Levine].
- Evaluation: 12 dynamic simulated tasks (Kinetix) and 6 real bimanual tasks, incl. lighting a match; robust to injected delay [S, same].
- LeRobot has an RTC implementation [S: https://huggingface.co/docs/lerobot/rtc].
- Newer follow-ups reduce chunk-boundary jerk further (e.g. SEAM: -28% boundary jerk on LIBERO-10 with π0.5) [S: https://arxiv.org/html/2607.04609 via search snippet; I did not read the paper].
- Tendra can copy: yes, once Tendra has a chunk policy. Not relevant to PPO per-step policy.

### Gemini Robotics (DeepMind)
- Cloud backbone with query latency under 160 ms plus a local action decoder on the robot; ~250 ms end-to-end, 50 actions/s; trained mainly on ALOHA bimanual robots [S: https://arxiv.org/html/2503.20020v1 via search summary]. An On-Device variant runs fully on the robot [S: https://www.infoq.com/news/2025/07/google-gemini-robotics/].
- Lesson: same split as Helix and GR00T, slow semantic brain plus fast local action head.

### NVIDIA GR00T N1 / N1.5
- Dual system: System 2 = Eagle-2 VLM at ~10 Hz; System 1 = flow-matching Diffusion Transformer, 120 Hz, chunks of 16 actions, cross-attending to the VLM tokens; trained on real teleop + synthetic sim data + human video [S: https://arxiv.org/abs/2503.14734 via search; the "synthetic + video data pyramid" is from my memory M]. Open weights; small enough for fine-tuning on one GPU [M].
- Tendra can copy: GR00T N1.x is the most realistic "pretrained VLA" to fine-tune later since it targets humanoid hands; Isaac Sim data generation is not possible locally (no NVIDIA GPU), but fine-tuning on Colab/cloud is.

### Figure Helix (and Helix 02)
- S2: 7B VLM at 7-9 Hz; S1: 80M-parameter transformer at 200 Hz, fully convolutional multi-scale vision backbone initialised from sim pretraining; controls 35 DOF (wrists, fingers, torso, head); ~500 hours of teleop data, <5% of typical VLA datasets; a single set of weights for all tasks; also predicts task completion [S: https://www.figure.ai/news/helix (fetched)].
- Helix 02: whole-body autonomy incl. locomotion, dishes [S: https://www.figure.ai/news/helix-02, only titles seen; I did not read it].
- Why smooth: 200 Hz small fast network + decoupled slow planner.
- Tendra can copy: the architecture idea, at small scale: a small fast policy (MLP/small transformer, 50-100 Hz) conditioned on a slow latent/goal.

### 1X, Tesla Optimus, Boston Dynamics + TRI LBM
- Tesla Optimus: vision-first, same end-to-end stack as FSD; primary data is first-person video of humans doing tasks (claimed) [S: https://optimusk.blog/blog/ai-training-for-tesla-optimus/ , a blog, low reliability; treat as uncertain].
- BD + TRI: a single Large Behavior Model controls whole Atlas body, hands and feet treated alike, handles mid-task perturbations (lid closed, box slid); policies could be sped up 1.5-2x without significant loss [S: https://www.therobotreport.com/boston-dynamics-tri-use-large-behavior-models-train-atlas-humanoid/ and https://bostondynamics.com/blog/large-behavior-models-atlas-find-new-footing/ via search]. TRI's separate LBM paper stresses blind A/B testing with many rollouts and statistical tests [M, could not confirm; worth reading].
- 1X: I did not find a source in this session. [M] NEO is teleop-assisted ("expert mode") to collect data, and 1X trains world models for evaluation. Unverified.

---

## 2. Tesla FSD: what transfers
- What is documented/credible: end-to-end neural net (video in, controls out) replaced ~300k lines of hand-written C++ in v12 [S, secondary: https://www.fredpope.com/blog/machine-learning/tesla-fsd-12]; data engine = iterate: find failure cases, mine similar clips from the fleet, label/train, redeploy [S: Wikipedia-style summaries in https://arxiv.org/pdf/2502.13184 ("data engine ... data-centric iterative development")]; shadow mode = the new network runs silently in customer cars, compared with what the human did; disagreements trigger data upload [S: https://www.notateslaapp.com/news/3108/teslas-fsd-shadow-mode-what-it-is-and-how-it-improves-fsd]. The "400,000 clips per second" figure appeared in a secondary source [S, low confidence].
- Reliability validation: miles per intervention, per-release regression suites on replayed clips, simulation of rare events [M; Tesla publishes little rigorous data; its public safety stats are contested. Uncertain].
- What transfers to Tendra:
  1. Build a **fixed regression suite** (N seeds x objects x both hands) and run it on every new policy. Tendra already has `sim/eval_grasp.py` on fixed seeds; extend to a gating rule (no release if any bucket drops).
  2. **Shadow mode** = run a candidate policy next to the working one in sim/real, log disagreement, only promote after it wins.
  3. **Data engine** = automatically save every failed episode (seed, object, state) and oversample those starts in training; Tendra's curriculum/demo-state starts are a start.
  4. Do not copy the fleet; Tendra has no fleet. Sim is the "fleet", so sim fidelity (contacts, tendon friction) is the weak link.

---

## 3. RL fine-tuning of imitation policies, residual RL

### ResiP (From Imitation to Refinement, residual RL)
- Frozen chunked BC (diffusion) model + a small closed-loop residual policy trained with on-policy RL (PPO). The chunked base acts as a trajectory planner; the residual adds the reactivity chunking lacks. Tested on precise visual assembly (furniture-bench) [S: https://arxiv.org/abs/2407.16677, https://residual-assembly.github.io/].
- Smooth/reliable because the base is human-smooth and the residual is small and bounded.
- Tendra: **very relevant**. The current env already has a "synergies + 20 residual" action. Replace the synergy base with a learned BC/ACT policy from scripted+teleop demos, keep a *bounded* residual, penalise residual magnitude/jerk.

### DPPO (Diffusion Policy Policy Optimization)
- Treat denoising steps as an inner MDP and run PPO over it; fine-tune only last K' (10-20 of 100) steps. Stable, structured exploration; sim-to-real zero-shot 80% on a real robot where a Gaussian PPO policy (88% sim) completely failed from jitter [S: https://diffusion-ppo.github.io/ , https://arxiv.org/pdf/2409.00588].
- Tendra: possible on Colab; it is the "PPO but smooth" route. More compute-heavy than a residual.

### PA-RL (Policy-Agnostic RL)
- Fine-tunes any policy class (diffusion, transformer) by sampling many actions, picking the best with a critic, and distilling back with supervised learning [M, could not verify in this session; paper "Policy Agnostic RL", Mark et al. 2024].

### RL-100
- Three stages: imitation pre-training, iterative offline RL, online RL with a PPO-style objective inside the denoising process; consistency distillation to a one-step policy for real-time control. 100% success over 900 real-world episodes across 7 tasks (Push-T, bowling, pouring, cloth folding, dexterous unscrewing, orange juicing) [S: https://arxiv.org/abs/2510.14830].

### HIL-SERL / SERL
- Human-in-the-loop sample-efficient RL: 20-30 demos seed the buffer, a visual binary reward classifier (~200 positive / ~1000 negative images) gives reward, a human takes over via SpaceMouse when the robot goes wrong; corrections go into the buffer. 13 tasks, ~100% success after 1-2.5 h of real training (belt assembly 6 h), vs 49.7% for the HG-DAgger imitation baseline, with faster cycle times [S: https://arxiv.org/html/2410.21845v1 (fetched), Science Robotics: https://www.science.org/doi/10.1126/scirobotics.ads5033]. Hardware was arms with parallel grippers, not a dexterous hand [S].
- Why reliable: RL on the *real* robot optimises exactly the real success metric and self-corrects; interventions keep it from drifting into bad habits.
- Tendra: needs a real arm. Not yet. But the recipe (small demo set + classifier reward + human intervention) is the later "real-hand" stage.

### ConRFT
- Offline then online RL fine-tuning of a VLA via a consistency policy, with human interventions; 96.3% average success within 45-90 min of online fine-tuning, 1.9x shorter episodes [S: https://arxiv.org/pdf/2502.05450].

---

## 4. Dexterous-hand sim-to-real RL: how they get stable grasps and holding

### OpenAI Rubik's cube (ADR)
- Automatic Domain Randomization widens the randomisation ranges automatically as the policy succeeds; trained huge LSTM policy in sim; Shadow hand + vision pose estimator; solved cube on hardware [S: https://shadowrobot.com/wp-content/uploads/2023/04/solving-rubiks-cube.pdf via search]. Success on the hardest scramble was only ~20% and 60% on easier ones [M]; compute was a large CPU cluster [M]. Control at ~12 Hz policy [M].
- Tendra can copy: ADR idea. Tendra already has a `dr_level` curriculum; make it *automatic* (raise a randomisation range only when success > threshold, the repo's adaptive curriculum partly does this).

### DextrAH-G / DextrAH-RGB (NVIDIA)
- Teacher policy with privileged state trained in sim (GPU massive parallel, Isaac Lab); a "fabric"-guided action space (geometric fabrics) gives smooth, safe, collision-aware arm-hand motion; student distilled with online DAgger into a depth/RGB policy; arm + Allegro-class hand, real success per object from 0-100% (e.g. 100% Pringles/pot, 20% pitcher, 0% airplane in the table found in a search snippet; a later paper quotes 77% average with a state teacher) [S: https://arxiv.org/abs/2412.01791 (PDF too big to fetch, from search summary), https://arxiv.org/pdf/2506.14317].
- Why smooth: the policy outputs targets *into a classical motion generator* (fabrics) that enforces smoothness/limits, plus teacher-student distillation + heavy DR. That is exactly "classical controller + learned policy".
- Tendra can copy: teacher-student (privileged teacher already exists as asymmetric critic), plus **action smoothing layer** (a rate/jerk-limited filter between policy and servos). Isaac Lab is out locally; MJX/Warp on Colab is what Tendra has.

### DexPBT, AnyRotate, others
- DexPBT: population-based training for hard dexterous tasks (arm + hand, regrasping) [M; the search did not return it, unverified details].
- AnyRotate: in-hand rotation in any axis with dense tactile sensing, sim-to-real [M; I found only a related tactile in-hand paper https://arxiv.org/pdf/2303.04705].
- Common lessons across these [M, consistent with DextrAH]: privileged teacher, student distillation, DR of friction/mass/gains/latency, action smoothness penalty, low control rate (10-30 Hz) feeding a PD loop at higher rate, object-in-hand reward shaping, **hold** rewarded through time.

### LEAP Hand, DexMachina, Visual Dexterity
- LEAP Hand: low-cost 16-DOF open hand used by many sim-to-real papers (Tendra has 16 servos too, similar scale) [M]. Not searched. DexMachina (functional retargeting of human-object demos to dexterous hands with a curriculum of virtual object controllers) and Visual Dexterity (in-hand reorientation from depth, Chen et al.) [M], not verified here.

---

## 5. Measuring reliability, layered safety

What the sources give:
- Success over many trials: RL-100 reports 100% over 900 episodes [S]; HIL-SERL reports per-task 100% with fixed eval trials [S]; π*0.6 reports throughput (successful tasks per hour) plus failure rate and hours of uninterrupted running (13 h espresso) [S].
- Metrics to adopt (common practice, partly [M]): success rate with a confidence interval (e.g. 0 failures in 100 trials only bounds failure rate below ~3% at 95%; "rule of three"); interventions per hour; mean time between failures/interventions; time to recover after perturbation; throughput; worst-bucket success (not just average).
- Failure recovery: HIL-SERL and RECAP specifically train on corrections; BD's LBM demo interjects disturbances [S].
- Layered safety (design pattern, [M] except where cited): classical low-level controller (position PD, torque/current limits, SCS0009's own overload backoff in Tendra), a learned policy that only proposes targets, a **safety filter** clipping targets (joint limits, max velocity/acceleration, workspace, force/current), watchdog + e-stop, rate limit between chunks. DextrAH's geometric fabrics are the research example of the filter [S: via search].

---

## 6. What Tendra can realistically copy (ranked)
1. Rate/jerk-limited action filter + jerk and action-delta penalties, evaluate **jerk and held-grasp time** as metrics. Cheap, now.
2. Chunked output: let the PPO policy (or a distilled student) output k=8-16 future targets at 20 Hz executed at 100 Hz with interpolation. Cheap; changes only the action layer.
3. BC from demos then bounded residual PPO (ResiP): demos from the scripted grasp + any teleop; policy = chunked ACT/flow base + residual.
4. Teacher-student distillation: privileged state teacher (existing PPO) -> student that sees only what the real hand will have (servo positions/load, camera) with DAgger; the student is a chunked ACT/diffusion/flow policy trained on teacher rollouts (a cheap large demonstration source, no human).
5. Regression-gated evaluation (fixed seeds x objects x hands x DR levels), CI-style; confidence intervals; log failures and re-sample them.
6. ADR-style automatic randomisation including latency, servo backlash, tendon stretch, friction.
7. Later: real hand with HIL-SERL-style correction learning (needs an arm or at least a fixed rig).

Not realistic soon: foundation-scale pretraining (tens of thousands of hours), fleet data, humanoid whole-body.

## 7. Recommended staged path: "RL grasp in sim" -> "cooks and does laundry"
Stage 0 (now, weeks): make the existing PPO calm. Add smoothness metrics (jerk, action delta, dwell time holding), action low-pass/rate limits, action penalties, longer hold requirement. Define a fixed eval suite and a pass bar (e.g. >=95% over 200 episodes per object, both hands, at DR level X).
Stage 1: teacher -> chunked student. Generate 10k+ successful episodes from the teacher (+ scripted) with images rendered afterwards (the repo already has datasets/LeRobot export), train ACT/diffusion/flow student on Colab, compare smoothness vs PPO. Add RTC-style async inference tests with simulated latency.
Stage 2: residual RL or DPPO on top of the student for hold/regrasp robustness; ADR for physics and sensing noise, including servo dynamics identified from the real SCS0009.
Stage 3: sim-to-real on the **hand alone** on a fixed rig (hand on a table stand or simple arm): system identification, safety filter, per-servo current limits, 1-DOF then full-hand tests; metrics: success/100, interventions/hour. Then HIL-SERL-style fine-tuning with a human correcting.
Stage 4: arm in the loop (real or cheap arm), grasp+place, pouring, tool use; add a language/task layer (small VLA fine-tune like GR00T N1.x or SmolVLA/π0 via LeRobot on rented GPU) with the S1/S2 split.
Stage 5: household skills, one at a time, each gated by the regression suite: pick-place dishes, open drawers, wipe, then deformable (laundry: fold a towel) with RECAP-style iteration (deploy, log, correct, retrain). Cooking (stirring, pouring, tool use) needs force/tactile sensing: add fingertip sensing (the repo's own research found ~3.4 N fingertip force vs 45-80 N in other hands; that is a hardware blocker for reliable cooking and tool use).
Stage 6: data engine: every deployment failure becomes training data; shadow-mode a new policy before promoting.

## Uncertainty notes
- Several fetches returned only summaries (search snippets) and the DextrAH PDF was too large; items marked [M] are from memory and should be verified before quoting publicly.
- Tesla and 1X internals are largely non-public; claims above are secondary-source level.
