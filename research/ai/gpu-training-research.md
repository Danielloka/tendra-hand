# Faster, stronger GPU training for the grasp policy (research, 2026-10-03)

Question: how do we make the arm + V1 hand grasp training faster and stronger on one Colab GPU?
Current setup: MuJoCo Warp + Brax PPO + Playground, free T4, ~4,000 steps/s, 77% after 11 M steps
(floating hand). Context: [grasp-rl.md](grasp-rl.md), [resources.md](resources.md).

How to read the evidence: **[V]** = I checked it in the source (PyPI JSON, paper page, repo page).
**[S]** = from a search summary or an abstract only; check before relying on it. Numbers from other
papers come from other simulators and GPUs, so treat them as direction, not promise.

## 1. Algorithms beyond vanilla PPO

**Off-policy at scale (FastTD3, FastSAC, FlashSAC).**
- FastTD3: huge batch (32,768), distributional critic (C51), critic 1024-512-256, many envs, noise
  schedule; solves HumanoidBench in under 3 h on one A100, and ran on a real humanoid. AMP + `torch.compile`
  gave up to ~70% speed-up. The paper warns that a PPO-tuned reward can make TD3 jerky, so rewards
  may need re-tuning. [FastTD3 paper](https://arxiv.org/html/2505.22642v1) [V], [code](https://github.com/younggyoseo/FastTD3) (PyTorch; Playground and IsaacLab supported) [S].
- FastSAC: humanoid sim-to-real in ~15 min on one RTX 4090 ([paper](https://arxiv.org/abs/2512.01996)) [V abstract].
- FlashSAC (RSS 2026): 2.5 M-parameter network, only 2 updates per 1,024 new transitions, 10 M replay,
  weight/feature normalisation. Beats PPO and FastTD3 across 60+ tasks, with the largest gains on
  high-dimensional tasks like dexterous manipulation; FastTD3 often failed where FlashSAC converged.
  **PyTorch, MIT licence, has a MuJoCo Playground adapter**, defaults 1,024 envs / batch 2,048.
  [paper](https://arxiv.org/html/2604.04539v1), [repo](https://github.com/Holiday-Robot/FlashSAC) [V]. Their wall-clock figures use an RTX 5090, so a T4 will be several times slower.
- **Catch for us:** all of these are PyTorch. Our stack is JAX/Brax. Options: (a) run the PyTorch
  learner and keep the sim in Warp, passing tensors by DLPack (Warp supports this, see
  [MJWarp docs](https://mujoco.readthedocs.io/en/latest/mjwarp/) [S]); (b) the JAX route
  [PQN / purejaxql](https://github.com/mttga/purejaxql) (Apache-2.0) has a DDPG-style continuous version
  tested on 50 Playground tasks [V], but it is a research baseline, not a dexterous result.
- Our twist: demo-state starts and a changing curriculum make the data non-stationary. Off-policy
  replay mixes old and new curriculum stages. This is untested and a real risk (**my judgement**, no source).

**SAPG** (ICML 2024): splits the envs into groups with different policies and aggregates their data.
On AllegroKuka reorientation, +66% over DexPBT, while PPO and PQL barely learned; code exists.
[paper](https://arxiv.org/html/2407.20230v1), [code](https://github.com/jayeshs999/sapg) [S]. It is built for
sparse, hard exploration at 24k+ envs. We have ~2k envs on a T4, so I would not start here.

**Staggered environment resets** (Nov 2025): with short rollouts, all envs reset at the same time and every
batch sees the same slice of the episode. Starting envs at different offsets (groups shifted by
multiples of the rollout length) gave 2-3x faster convergence on ManiSkill StackCube and kept scaling
past ~6,000 envs, with no change to PPO itself. [paper](https://arxiv.org/html/2511.21011) [V]. No code
link; it is a few lines in the env wrapper (random initial step counter / pre-advance each group).
Best gain per effort on the list. Caveat: ManiSkill, not MuJoCo Warp; locomotion gained little.

**Network tricks.** SimbaV2 (hyperspherical normalisation, distributional critic, reward scaling)
scales smoothly without resets ([paper](https://arxiv.org/abs/2502.15280)) [S]; it is aimed at off-policy.
For PPO, a larger critic with LayerNorm reduces run-to-run variance (general result, weak evidence for
our case) [S]. Cheap to try because the critic is already separate (asymmetric).

**Large-batch PPO.** Minibatches of tens of thousands stabilise massively parallel PPO
([Rudin et al. 2021](https://arxiv.org/pdf/2109.11978)) [S]. MuJoCo Playground's LEAP-hand PPO config is reported
as: 4,096 envs, batch 131,072, minibatch 32,768, lr 1e-4, 4 epochs, gamma 0.98, GAE lambda 0.2 (sic),
clip 0.01 (sic), value coef 1.2, grad-norm 10, entropy 0, hidden 512
([Playground report via search](https://playground.mujoco.org/assets/playground_technical_report.pdf)) [S, from a
search summary; the PDF was too large to fetch, so confirm lambda and clip in
`mujoco_playground/config/manipulation_params.py` before copying].

**Mirror symmetry.** Data augmentation (feed the mirrored transitions) beat symmetry losses and
hard-equivariant networks: faster convergence, higher returns on cube and dexterous tasks, with the
note to use the *original* action probability in the PPO ratio and small initial weights
([Mittal et al. 2024](https://arxiv.org/html/2403.04359)) [V]. **We already get most of this for free:**
the left hand is mapped onto the right (`canon_*`), so one policy sees only a right hand. Augmentation
would only matter once a model drives both hands at once. Not now.

## 2. Dexterous grasping lessons

- **DextrAH-RGB** (NVIDIA): palm pose (6) + a **5-D PCA hand action** through a geometric-fabric controller;
  reward = hand-to-object distance + lift + finger-open penalty; automatic domain randomisation (ADR);
  teacher 62 h on 8x H100, student (online DAgger) 53 h on 4x L40S
  ([paper](https://arxiv.org/html/2412.01791)) [V]. Takeaways: our synergy action space matches what works;
  the **simple reward** is a hint that ours (many terms) may be heavier than needed; the full teacher-student
  pipeline is far beyond a free T4, so do vision distillation with a small recorded dataset instead.
- **DemoGrasp** (2025): treat grasping as a **one-step problem**: take one demo trajectory, RL only picks the
  edit (wrist pose offset + hand joint offsets), reward = success + table-collision penalty. 95% on
  DexGraspNet with a Shadow Hand; real-robot via imitation distillation ([paper](https://arxiv.org/abs/2509.22149)) [V abstract].
  This fits us unusually well: **our scripted grasp already lifts ~29/30**. A one-step policy over
  (object pose -> wrist offset + synergies) needs no credit assignment over 100 steps; each sample is one
  scripted rollout. Good for a fast first strong policy and as demo source for distillation.
- **OmniReset** (Mar 2026): PPO at 64k+ envs with **no curriculum or reward shaping**, only diverse
  resets: near-goal, **grasped**, near-object, and reaching starts; >300x wider range of initial
  conditions than baselines ([paper](https://arxiv.org/abs/2603.15789), [site](https://weirdlabuw.github.io/omnireset/), code UWLab) [V].
  We only reset to "grasped". Adding "near-object" (hand a few cm away, palm facing) and "reaching"
  (random wrist pose) bridges the gap our run-2 hit (it learned to hold, not to approach).
- **Pen-spinning lessons**: RL privileged oracle -> trajectory dataset -> pre-train sensorimotor policy;
  matches our teacher -> student plan ([paper](https://arxiv.org/pdf/2407.18902)) [S].
- **Action space for arm + hand**: DextrAH-style Cartesian palm pose with a low-level controller is what
  works for arm-hand; ours (wrist target + IK) is the same idea. No evidence found for joint-space being better.
- **Bimanual**: Bi-DexHands shows PPO masters simple two-hand tasks, multi-agent methods help with tight
  coordination ([paper](https://arxiv.org/abs/2206.08686)) [S]. Not needed while one hand works per episode.

## 3. Curriculum and exploration

- Reverse curriculum / demo resets are well supported (OmniReset above; DAPG, Florensa 2017, already cited in grasp-rl.md).
- ADR (DextrAH) widens randomisation automatically when success is high. We have a stage-based object
  curriculum; **ADR on the DR ranges** (friction, mass, noise) is the natural next step, with no new infrastructure: one
  scalar "difficulty" that moves when success > threshold.
- Reward that works for grasp+lift is short: approach distance + lift height + finger regulariser
  (DextrAH), or only success + collision (DemoGrasp). Try **removing** terms before adding them.
- Staggered resets (above) matter more when a curriculum and demo resets already vary the episode start.

## 4. Domain randomisation in MuJoCo Warp

- Many `mjw.Model` fields have a leading batch dimension; `field[worldid % field.shape[0]]`, set at
  creation with `mjw.put_model(m, batch_sizes={"dof_damping": N})` [V, [docs](https://mujoco.readthedocs.io/en/latest/mjwarp/)].
  Only a documented subset is safe to change on-device; others need a recompile through `mjSpec`, and
  per-world meshes need manual per-world arrays (`geom_dataid`, `geom_size`, `body_mass`, inertia) [V].
  For us: friction, mass, damping and actuator gains are the cheap ones; object size scaling (we do +-15%) is
  the hard one: use a few fixed size variants as separate geoms or keep scaling `geom_size` for primitives
  (cylinder/box/sphere need no mesh). **Not verified:** how Playground's `randomization_fn` plugs into
  `--impl warp`; test early with a 2-line experiment.
- Raise `naconmax` / `njmax` when randomising (contacts vary by world) [V]; a held object plus mesh
  contacts is already our slowest case.
- Observation noise, action delay (a 1-2 step action buffer) and the per-episode bias are env-code,
  not physics, so they cost almost nothing and work the same in JAX.
- Teacher-student: record the teacher's rollouts, then train the vision student (see DextrAH-RGB; our
  `eval_grasp.py --record` + LeRobot path). Keep the dataset small; one T4 cannot do 50 h of DAgger.

## 5. Colab in 2026

- Free: T4, up to 12 h per session, 90 min idle disconnect, availability not guaranteed. Pro / Pro+: T4, L4,
  A100 (40/80 GB), up to 24 h; roughly 1.2 / 1.7 / 5.4 compute units per hour (T4 / L4 / A100). H100 is
  not listed. ([comparison, Oct 2026](https://www.thundercompute.com/blog/colab-alternatives-for-cheap-deep-learning-in-2025)) [S].
  **L4 is the sweet spot** if the owner pays: newer architecture, 24 GB, only ~1.4x the T4's unit cost.
- Long runs: save a checkpoint to Drive every N updates (Brax `progress_fn` + orbax, which Brax already depends
  on), write the curve CSV there, make the script resume from the last checkpoint, and run in chunks of < 11 h.
  Idle disconnect is about no browser activity, not GPU load; do not rely on it staying alive.
- Free T4 caveat: compute capability 7.5, so JAX Pallas/Triton kernels do not run
  ([jax#26799](https://github.com/jax-ml/jax/issues/26799)) [S]; MJWarp itself works (your 4,000 steps/s run).
  Ampere+ GPUs should set `JAX_DEFAULT_MATMUL_PRECISION=highest` (TF32 hurt Playground results)
  ([Playground README](https://github.com/google-deepmind/mujoco_playground)) [V].
- **Breaking changes to know:**
  - JAX **0.10.0 (2026-04-16) removed `device_put_replicated`**, which Brax's PPO/SAC/APG call; 0.9.2 is the last
    version with it ([JAX changelog](https://docs.jax.dev/en/latest/changelog.html)) [S]. Brax 0.14.2 is still the
    latest release (2026-03-15) and declares `jax>=0.4.6` with no upper bound, so pip will pull a broken JAX [V, PyPI].
  - `warp-lang>=1.10.0.dev20251007` broke `--impl warp` in the JAX PPO trainer
    ([mujoco#2894](https://github.com/google-deepmind/mujoco/issues/2894)) [V]. Fixed since: `mujoco-mjx[warp]` 3.14 pins exactly `warp-lang==1.17.0` [V, PyPI].
  - Playground 0.1.0 (Jan 2026): `mjx_env.init` -> `mjx_env.make_data(MjModel)`; 0.2.0 (Mar 2026): Warp is
    the default impl, `nconmax/nccdmax` renamed `naconmax/naccdmax`
    ([releases](https://github.com/google-deepmind/mujoco_playground/releases)) [V].
  - MuJoCo 3.5 (Feb 2026) made MJWarp an official release ([discussion](https://github.com/google-deepmind/mujoco/discussions/3094)) [S].
  - A second report: the Warp MJX backend raised `undefined symbol: wp_cuda_graph_launch` on a 2nd jitted call
    ([mujoco#2865](https://github.com/google-deepmind/mujoco/issues/2865)) [S]; appears older, check if you see it.

## What we should do (ranked by expected gain per effort for THIS project)

| # | Action | Effort | Why / expected gain |
|---|---|---|---|
| 1 | **Staggered resets** in the env wrapper | hours | Algorithm-agnostic, reported 2-3x faster on manipulation (ManiSkill); our demo/curriculum resets make batches even more clustered |
| 2 | **Pin versions** (below) and checkpoint to Drive | hours | Avoids the two known breakages; makes 12 h free sessions safe |
| 3 | **Copy Playground's LEAP PPO config** (big minibatch, gamma 0.98, small lr); confirm odd values in the repo first | hours | Same hand class (16-DoF, 4,096 envs); tuned by the Playground authors |
| 4 | **Reset distributions** à la OmniReset: add near-object and reaching starts next to the grasped demo starts | 1 day | Fixes "holds but never approaches"; removes need for much shaping |
| 5 | **Trim the reward** toward approach + lift + finger regulariser; ablate terms one by one | 1 day | DextrAH and DemoGrasp both succeed with tiny rewards; fewer terms = less hacking (try2 hacked the height) |
| 6 | **One-step "edit the scripted grasp" policy** (DemoGrasp) | 2-3 days | Scripted grasp already lifts ~29/30; a bandit over wrist offset + synergies gives a strong policy and demo set fast |
| 7 | **ADR on the DR ranges** (single difficulty scalar driven by success) | 1 day | Robust policy without hand-picking ranges; DextrAH used it |
| 8 | **Bigger critic with LayerNorm**, and an observation/action delay buffer | hours | Cheap; lower variance. Evidence is general, not dexterity-specific |
| 9 | **Try FlashSAC** (PyTorch, Playground adapter) on our env via a torch<->warp bridge | 1-2 weeks | Biggest upside on high-dim tasks (reported, other GPUs); risky with curricula and a PyTorch port |
| 10 | **Move to an L4** (paid Colab) once runs are stable | minutes | Newer GPU, 24 GB, more envs; cost ~1.4x T4 units |

Not now: SAPG (needs 24k+ envs), mirror augmentation (the canonical-right-hand design already shares data), full
DextrAH-RGB teacher-student (62 + 53 GPU-hours on 8 and 4 big GPUs).

## Versions (checked on PyPI 2026-10-03 [V])

| Package | Pin | Why |
|---|---|---|
| python | 3.12 | jax>=0.9 needs >=3.11; Colab ships 3.12 (a Colab report shows 3.12.13) |
| jax / jaxlib | `jax[cuda12]==0.9.2` | last release with `device_put_replicated`; Brax 0.14.2 breaks on >=0.10 (released 2026-03-18) |
| brax | `0.14.2` | latest; PPO fixes (adaptive KL, running stats for 20B+ steps) |
| mujoco | `3.14.0` | released 2026-09-22; matches mjx and mujoco-warp |
| mujoco-mjx | `3.14.0` | same release; its `warp` extra pins the Warp version below |
| mujoco-warp | `3.14.0` | requires `mujoco>=3.12`, `warp-lang>=1.15` |
| warp-lang | `1.17.0` | exact pin required by `mujoco-mjx[warp]` 3.14; avoids the Oct-2025 dev-build breakage |
| playground | `0.2.0` | Warp default, `naconmax` names; asks `brax>=0.14.2`, `mujoco-mjx>=3.6`, `warp-lang>=1.11` |
| orbax-checkpoint | `>=0.11.22` | Brax requirement; use for Drive checkpoints |

These match your current working set (mujoco 3.14, warp 1.17, jax 0.9, brax 0.14). Install in one `uv pip install`
call with all pins, then restart the Colab runtime. Dependency metadata alone cannot prove the combination
runs on a T4; your 4,000 steps/s run is the evidence for that.

## Gaps (not found or not verified)

- No source found that reports FastSAC/FlashSAC on an arm + tendon hand; the dexterous numbers are Shadow Hand in IsaacLab.
- Brax PPO + Warp randomisation (`randomization_fn` with `--impl warp`): not verified.
- Actuator-latency randomisation results for hands: only general sim-to-real literature, not checked here.
- Playground LEAP PPO values come from a search summary; two of them look odd.
