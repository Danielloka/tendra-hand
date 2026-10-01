# AI roadmap: from zero to a bimanual robot

This is the detailed plan behind Phases 5–8 of `docs/roadmap.md`, and it runs **in parallel** with the hardware phases 1–4. Stages A–C ≈ Phases 5–6, Stage D ≈ the first body in Phase 8 (the pole station), Stage E ≈ Phase 7.
Expect years, not months. Each stage ends with something that works and can be shown.

## Stage A: Foundations (now → ~3 months)
Learn the core ideas and tools while the hand hardware is being built.

- [ ] Maths track, part 1 (see `math.md`): linear algebra, transforms, kinematics
- [ ] Deep-learning basics: build and train a small network from scratch (e.g. Karpathy's "Zero to Hero")
- [ ] Install **LeRobot**; train an ACT or Diffusion Policy on a public simulated task (e.g. ALOHA sim insertion) on a free cloud GPU (Colab / Kaggle)
- [x] **Webcam teleop of Tendra V0/V1 in MuJoCo** (`sim/teleop.py`, 2026-09-28; live tuning with a real hand pending): MediaPipe hand tracking → retargeting → `SimHand`. Then the same on the real hand via `RealHand`
- [ ] Forward kinematics + Jacobian of the Tendra hand from the MJCF, checked against MuJoCo

- [x] **Grasp demos in simulation** (2026-09-29): floating V1 hand + table + objects, wrist and fingers from the webcam, episodes recorded and exportable to LeRobot (`sim/grasp_teleop.py`)
- [ ] Record ~50 grasp demos and train ACT on them (free cloud GPU); measure the success rate in sim
- [ ] **RL grasping in sim** (system built 2026-09-29: `sim/train_grasp.py`, `research/ai/grasp-rl.md`): cylinder, cube and ball ≥ 80% success from normal starts; then RL teacher → vision student

**Done when:** you can move the simulated (and real) Tendra hand with your own hand in front of a webcam, and you have trained one policy yourself.

## Stage B: One arm, one gripper, imitation learning (~3–9 months)
Learn the full "demonstrate → train → deploy" loop on cheap, proven hardware.

- [ ] Build an **SO-101 leader + follower** pair (LeRobot), plus a wrist camera and a fixed camera
- [ ] Mount it on the first version of the **pole**
- [ ] Record ~50 demos of a pick-and-place task; train ACT; measure success rate over 20 trials
- [ ] Add L1 basics: Cartesian (end-effector) control with inverse kinematics
- [ ] Try fine-tuning a small open VLA (SmolVLA) with language instructions

**Done when:** the arm picks up and places 3 different objects on command with ≥ 80 % success.

## Stage C: Arm + Tendra hand (~9–18 months)
Put our hand on an arm and control all fingers.

- [ ] Choose/build an arm with enough payload for V1 (see `resources.md` → Arms); mount V1 with a wrist camera
- [ ] **Hand teleop device:** VR hand tracking (Quest 3) and/or the Tendra kinematic-twin glove (`ideas.md`)
- [ ] Retargeting human hand → Tendra hand (fingertip-position optimisation)
- [ ] Imitation learning with the hand: power grasp, pinch, cylinder grasp
- [ ] **RL in simulation** (cloud GPU): in-hand cube rotation with the V1 model, domain randomisation, then sim-to-real
- [ ] System identification so the V1 sim matches the real hand (hardware Phase 3)

**Done when:** the arm + hand picks up 10 everyday objects of different shapes, and does one in-hand skill learned in simulation.

## Stage D: Two arms on the pole (~18–30 months)
- [ ] Second arm + mirrored left hand; head camera
- [ ] Bimanual teleop (two gloves / VR)
- [ ] Bimanual policies: handover, open a jar, fold a towel
- [ ] Fine-tune a large open VLA (π0 / GR00T / RDT style) on our data with cloud GPUs
- [ ] Publish the **Tendra dataset** on Hugging Face

**Done when:** it does 3 bimanual tasks reliably and the dataset is public.

## Stage E: Generalist (30 months →)
- [ ] L3 planner: language → steps → skills, with failure detection and retry
- [ ] Touch sensing in the policies
- [ ] Autonomous practice with self-reset, learning from its own successes/failures
- [ ] Long-horizon tasks (tidy a table, make a simple snack)

## The Tendra Ladder (benchmark tasks)

We measure progress on a fixed ladder. Record the success rate over 20 trials for each rung, in the sim and on the real robot.

| Rung | Task | Tests |
|---|---|---|
| 0 | Reach a point / follow a trajectory | Kinematics, control |
| 1 | Pick up a cube | Basic grasp |
| 2 | Pick and place 10 different objects | Generalisation |
| 3 | Precision pinch (coin, pen) | Fine finger control |
| 4 | Handover between hands | Bimanual coordination |
| 5 | In-hand rotation of a cube to a target face | Dexterity (RL) |
| 6 | Use a tool: spray bottle, screwdriver | Force + dexterity |
| 7 | Deformables: fold a towel, tie a knot | Hard perception + planning |
| 8 | Long task from a sentence ("clear the table") | Planning + everything |
