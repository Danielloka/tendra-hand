# Skill network: one shared "motor memory" for Tendra V2.1 on OpenArm

*Study by Claude in a cloud session, 2026-11-17. A study, not a decision: nothing in the main project
(`hardware/`, `sim/`, `software/`, `CLAUDE.md`) is changed. Everything this study builds will live in this
folder.*

**The owner's idea:** the robot learns movement patterns ("algorithms") the way children learn movements
by trying. A higher AI then uses those patterns by sending commands, and the patterns work on many
different objects, not just one.

**Short answer:** possible, and it matches how leading labs build robot brains today (Figure's Helix with a
slow "System 2" and a fast "System 1", Physical Intelligence's π0.5 picking a sub-step and then actions,
skill-space work such as ASE). We build it as **one shared skill network** with a command interface,
trained by RL in MuJoCo, with a planner on top. Pick-and-place is the first proof.

Owner's choices (2026-11-17):
- Hand: **V2.1** (from `cloud-claude/2026-11-14-hardware-ideas/out/18/`), kept on the side, not moved into
  `hardware/`.
- Arm: OpenArm shoulder + elbow (as in `tendra.arm`), **plus a forearm twist**, plus V2.1's 2-DOF wrist.
- One hand first (right).
- **Option 2:** one shared network for all skills, built in stages.
- Human motion prior (webcam recordings): **later, out of scope for now.**
- Training: MuJoCo (Warp) on GPU, **RunPod-ready** (~$2–3/h class GPU), but not run there yet.
- Logs: TensorBoard **and** Weights & Biases (W&B optional, needs an API key).
- Cameras: a **plain colour (RGB) wrist camera** on every hand, plus a head RGB-D (depth) camera.
- Grip types for stage 1: `power`, `pinch`, `tripod`.
- The scripted planner gets a small text interface from the start (`pick cube place red_spot`).
- Servos: **HLS3606M** on all 16 hand joints (not SCS0009), capped at 80 % torque; wrist 2 × STS3215
  (`model/actuators.json`, `model/ACTUATORS.md`).
- Tendons: **keep PE#8 Dyneema** (it stretches: ≈ 25° of joint angle at 94 N). The sim models the stretch
  truthfully (the actuator `kp` is servo stiffness in series with the tendon), and the training env adds what
  the real firmware will do: an **outer loop on the V2.1 joint-angle sensors** (TMAG5273 in every joint) that
  corrects the servo target so the *joint* reaches its target, with realistic sensor noise (≈ 0.5°) and rate.
  A stiffer line (thicker Dyneema) is a later option that only changes numbers in `actuators.json`.
- **On the real robot, perception turns camera + depth into a MuJoCo world, and the skills run on that
  world** (section 6b).

---

## Stage 0 status: the model (done 2026-11-18)

V2.1 hand + 2-DOF wrist + forearm on OpenArm's right arm (J1–J4) with a new forearm twist, wrist and head cameras,
a table scene. Built by three agents, attacked by an independent reviewer, fixed, verified.
Interface: [`model/CONTRACT.md`](model/CONTRACT.md). Servo numbers: [`model/ACTUATORS.md`](model/ACTUATORS.md).

```
uv sync                                          # in this folder (standalone project, Python 3.12)
uv run --group warp pytest -m "not cad"          # 60 tests, ~2 min (MUJOCO_GL=osmesa for the render test)
uv run python model/build_hand.py --check        # also build_robot.py --check
uv run --group cad python model/extract_v21.py   # only after the V2.1 CAD changes (~2.5 min)
```

| What | Measured |
|---|---|
| Fidelity to the CAD | 9 CAD poses + 150 random poses: every part within **0.022 mm / 0.065°**; all 28 joints are pure hinges (axes taken from the CAD's FK) |
| DIP/bar linkage | fit ≤ 0.065°; under 15 N at the fingertip ≤ 0.22° |
| Mass | 1112.9 g = CAD |
| Servos | HLS3606M, 80 % cap (0.72–0.76 N·m at finger joints, index pinch ≈ 12–15 N), 30° step in ~0.15 s, no overshoot; command may go past the joint range (tendon stretch) |
| Sensors | joint angle + servo force per actuator, in actuator order |
| Grasp | scripted cube grasp lifts 84 mm, 1 mm slip, 0.7 mm penetration; pads give condim 4 contacts |
| Arm | no overshoot (critically damped), gravity compensation on the arm only |
| Workspace (palm down) | 30 × 45 cm fully reachable at 10 cm palm height |
| Wrist camera (RGB, 110°) | little-finger side of the forearm, collision housing, never the lowest point; grasp space 100 % visible, fingertips 92 % |
| Speed | ~14,000 steps/s on one CPU thread (28× real time) |
| GPU path | MuJoCo Warp = MuJoCo C within 0.13 mm over 300 steps of a contact-rich grasp, 64 worlds identical; hand self-collision pairs 767 → 236 |
| Portable | self-contained (OpenArm meshes copied unchanged with its Apache-2.0 LICENSE + NOTICE), loads from any folder |

**Design findings for the V2.1 hardware:**
- The `_render` STLs of the CAD study are saved at `wrist_flex_+40`, not at q = 0 (the model undoes this).
- The thumb metacarpal enters the index knuckle module by up to 5.1 mm when `cmc_rot` < −5° and `cmc_flex` > 35°
  (inside the joint ranges). Needs a CAD fix or a range rule (open question to the owner).
- Palm vs forearm shell 0.8 mm at the wrist-range corners (dev −30°, flex ±50°).
- In the fist pose the little fingertip skin is 1.8 mm inside its own knuckle module.
- At full HLS3606M stall (118 N per strand) the SCS fuse cups, MR63 rollers and the wrist eye plate are marginal.

**Open for Stage 1:** joint-speed limiting and the joint-sensor outer loop live in the env; real HLS3606M travel
(assumed 360°) and servo stiffness need a bench test; left arm not built.

## 1. The layers

| Layer | Human version | Tendra version | Rate |
|---|---|---|---|
| **L3 Planner** | "Make tea: get the cup, then pour" | First a script, later an LLM/VLM that calls skills like functions | ~1 Hz |
| **L2 Skill network** (this study) | Learned movement patterns: reach, grip, twist, let go | One neural network: command + senses → arm and finger targets | 20–50 Hz |
| **L1 Servo control** | Spinal reflexes, muscles | Position control of each servo (firmware, sim actuators) | ~1 kHz |

The planner never moves a finger. It sends a **command** and waits for **done / failed**. The skill network
turns the command into calm, coordinated motion. Because each skill is trained on many objects, sizes,
masses and positions, it works on objects it has never seen.

## 2. The command interface (the most important design choice)

A command = **skill id + goal parameters (+ later a style vector)**. Draft for stage 1:

| Skill | Parameters | Done when | Fails when |
|---|---|---|---|
| `hold` | none | (default) hand and arm still | n/a |
| `reach` | pre-grasp palm pose (pos + quat) | palm within 1 cm / 10° of the pose | timeout, hits the table or object |
| `grasp` | object id, grip type (`power`, `pinch`, `tripod`) | stable contact on ≥ 2 sides, object follows a small lift | object knocked away, no closure |
| `carry` | target pose of the object | object within 2 cm of the target, still held | object dropped |
| `place` | target pose on a surface | object supported by the surface and still | object tips over or drops from height |
| `release` | retreat distance | fingers open, hand away, object still in place | object moves > 1 cm |

Why this shape:
- **Goal-based** (where, which object, which grip), not motion-based. The network decides *how*.
- **Every skill reports its own success**, so the planner can retry or pick another skill. This is what makes
  a language planner reliable later (`research/ai/ideas.md`, idea 9).
- Skills **start where the previous one ends** (section 4), so chaining works.
- The parameters are numbers a vision system or an LLM can produce.

Pick-and-place = `reach → grasp → carry → place → release → hold`, chained by a scripted planner.

## 3. The skill network

**Input**
- Body sense (proprioception): 16 servo positions and targets, wrist and arm joints, the last actions.
- The command: a one-hot skill id + the goal parameters, expressed in the hand's frame.
- Object information:
  - **Teacher** (trains first): exact object pose, size and contacts from the simulator ("privileged" state).
  - **Student** (trained from the teacher): what the cameras see (section 5) instead of the exact state.

**Output** (50 ms steps)
- Wrist motion: a small change of the palm pose, turned into arm joint targets by the existing damped
  least-squares IK idea (`ArmIK` in `software/tendra/arm.py`, ported to JAX).
- **16 servo targets directly.** The V1 synergies came from V1 data, so they are dropped.
- Everything passes through the calm layer: a filtered wrist command, ramped finger targets, rate limits.

**Shape:** an MLP with a learned **command embedding** in stage 1 (fast on GPU with Brax PPO, as used
before). A transformer over a short history comes later if needed (useful once there are cameras and many
skills). One network, many skills: grasping helps placing, and a new skill starts from motion the network
already knows.

**Later (stage 3+, not now):** a "style" vector z, so the planner can ask for different ways to do the same
skill, and a natural-movement prior from human motion recorded with the webcam teleop.

## 4. Training

- **Simulator:** MuJoCo Warp on the GPU, JAX env, Brax PPO, as in the old `software/tendra/gpu/` stack,
  but a new, cleaner env. **The same env runs on CPU** (laptop, tests) and GPU (RunPod), so there is no
  CPU twin to keep in sync.
- **Multi-skill RL:** every parallel world gets a random skill and a random goal. One reward per skill
  (reaching, contact quality, lift, distance to target, stillness at the end), all with the same calm terms
  (action rate, jerk, `still`) and CAPS from the start. Lessons from `research/ai/smooth-control.md` apply.
- **Chained starts:** many episodes start from the *end state of the previous skill*, recorded from earlier
  rollouts (a `grasp` starts where `reach` finished, `place` where `carry` finished). This teaches clean
  hand-offs between skills.
- **Curriculum:** large, light objects and close targets first, then smaller, heavier, slippery objects and
  far targets. Advance on the measured success, per skill (like `curriculum.py`).
- **Domain randomisation:** object shape, size, mass and friction, servo strength (V2.1 force budget: about
  5.6 N pinch with SCS0009), servo delay, sensor noise, camera pose and lighting. This gives the skills a
  chance to work on the real hand.
- **Teacher → student:** train the state-based teacher first (fast). Then distil it into the camera-based
  student (DAgger: the student drives, the teacher labels what it should have done). Rendering is the slow
  part, so it is only needed in this second phase.
- **Evaluation:** a fixed set of seeds per skill and for the full pick-and-place chain; success with a 95 %
  interval and the shaking metrics (`wshake`, `fshake`).

## 5. Cameras

**Wrist cameras: yes, on every hand.** Almost all recent manipulation systems use them (ALOHA-style
robots, π0, Helix). They see the object close up and from the hand's own view, which makes grasping and
placing much easier to learn and to transfer than with a head camera alone.

| Camera | Where | What for | In the sim |
|---|---|---|---|
| Wrist camera (one per hand), **plain RGB** (owner, 2026-11-17) | on the forearm/wrist, looking past the palm at the fingers | final approach, grip, release; helps track the object when the hand hides it from the head camera | a MuJoCo `<camera>` fixed to the forearm body |
| Head camera (RGB-D) | on the pole, above and behind the hands | where things are, the target spot, the planner's view | a fixed camera with depth |

- **Depth comes from the head camera only**; the wrist cameras are colour. A camera student (stage 3) uses
  head depth + a segmentation mask of the target object, and wrist RGB with strong colour/lighting
  randomisation. With section 6b this student becomes a backup, not the main path.
- **GPU rendering:** MuJoCo Warp has a batch renderer (ray tracing on the GPU, built for throughput, not
  realism). It is enough for low-resolution depth (e.g. 64 × 64) for training. Its depth output still has
  to be confirmed in the official docs before we rely on it.
- **Hardware (to check):** a small RGB camera on the wrist (owner's choice: smaller and cheaper than a
  depth camera). The V2.1 forearm (x ±52.5 mm shell) has room on its back
  side, but the mount, the field of view past the thumb and the cable through the wrist are open questions.
  The pole camera can be any RGB-D camera (RealSense D435 class, OAK-D).

## 6. Using real-world 3D scans and generative 3D models for training

**Yes, and it is a strong idea, if each model is used for what it is good at.**

| Tool | What it gives | Use in training | Limit |
|---|---|---|---|
| **Scene scan from the depth camera** (TSDF fusion, or Gaussian splatting) | The real table, shelf, kitchen as a 3D model | Build the sim scene from your real space ("real-to-sim"), so the robot trains where it will work | The scan gives shape, not physics: mass and friction must be guessed and randomised |
| **Image-to-3D object models** (e.g. Meta's SAM 3D Objects: one photo + mask → textured mesh) | Meshes of real objects (cups, tools, food) | Hundreds of realistic training objects; collision shapes by convex decomposition (V-HACD / CoACD) | Back sides are guessed; scale must be fixed from depth |
| **Gaussian splatting renderers** (e.g. GS-Playground, RL-GSBridge) | Photo-realistic images of the scanned scene | Later, for an RGB student that sees "real" pixels | Heavier to render; newer research |
| **Generative world models** (video models that "imagine" scenes) | Endless varied scenes and videos | Ideas, extra visual variety, testing a planner | **Not physics:** contacts and forces are made up, so they can't replace MuJoCo for learning a grip |

Plan:
1. Start with simple random objects (boxes, cylinders, spheres, capsules) in random sizes.
2. Add an **object library** made from photos of real household things (image-to-3D), plus free object
   sets (e.g. YCB-type scans).
3. Scan the real workspace with the head depth camera and rebuild it in MuJoCo, then train and test there.
   Train on **"digital cousins"** (many varied copies of the scan, not one exact copy), so the skills don't
   overfit to one table.
4. Later: Gaussian splatting for realistic RGB images for the student.

Points 2 and 3 run on the laptop or RunPod, outside the training loop, and produce ordinary MuJoCo assets.

## 6b. On the real robot: camera + depth → a MuJoCo world → the trained skills

**Owner's idea (2026-11-17):** the real robot does not feed raw pixels to the skills. A perception model
turns the cameras into the same kind of MuJoCo world the skills were trained in, and the skills run on
that.

```
head RGB-D + wrist RGB ──► perception ──► live MuJoCo "twin" ──► object state ──► skill network ──► servos
                         (find objects,     (meshes + poses,        (same inputs as          ▲
                          make meshes,       table, hand from        the teacher)             │
                          track poses)       servo feedback)                                  │
                                    planner (L3) can also test a plan in the twin first ──────┘
```

**Why it is smart:**
- The state-based **teacher can run on the real robot directly**. No pixel student is needed for many
  tasks, and the slow, hard camera-training phase becomes optional.
- The twin is **readable**: you can see what the robot thinks the world is, and debug it. The digital-twin
  code (`sim/twin.py`, the `--mirror` mode) already points this way.
- The **planner can try a plan in the twin before doing it** ("imagine, then act"), and reject plans that
  knock things over.
- Perception and skills can be improved separately.

**How (well-known building blocks):**
1. **Find objects:** segmentation (SAM-type models) on the head camera's colour image.
2. **Make meshes** for new objects: image-to-3D (SAM 3D Objects) plus the depth image for the true scale;
   known objects come from the object library (section 6).
3. **Find and track their pose** (position + rotation) at 20–30 Hz: a 6-D pose tracker for new objects
   (e.g. NVIDIA's FoundationPose, which takes a mesh or a few reference photos), on head depth + wrist RGB.
4. **The table and big furniture** from the depth scan (section 6).
5. **The robot itself** from servo feedback (SCS0009 positions), not from the cameras.

**The catch, and how training handles it:**
- **The hand hides the object** while grasping, so the estimated pose gets worst exactly when it matters.
  Wrist RGB helps; the twin's physics also helps (an object in a closed hand moves with the hand).
- **Estimates are noisy, late and sometimes wrong.** The skills must be trained on **estimated state, not
  perfect state**: in training, the object state the policy sees gets realistic pose noise, delay, dropouts
  during occlusion and wrong sizes. We measure these errors on the real perception system and copy them
  into the sim. This is the most important rule for making 6b work.
- **Physics is guessed** (mass, friction), so the skills stay robust through randomisation and feel the
  object through the servo loads instead of trusting the guess.
- **Best of both:** the skill network gets the estimated object state *and*, later, a small wrist-RGB input
  for the last centimetres. If the estimate is wrong, the camera can still correct the grip.
- **Compute:** segmentation + mesh + pose tracking need a GPU on the robot side (a desktop GPU or a
  Jetson-class board), not the ESP32 or the owner's laptop. Mesh generation runs once per new object, only
  the tracking is live.

## 7. Stages

| Stage | What | Proof it works |
|---|---|---|
| **0. Model** | V2.1 → MJCF converter (in this folder), arm with forearm twist, wrist and head cameras | Every part matches the CAD in all 9 poses of `poses.json`; signs, limits, mass |
| **1. Skills, state-based** | Env + command interface + `reach`, `grasp`, `carry`, `place`, `release`, `hold`; scripted planner | Pick-and-place chain ≥ 80 % on held-out objects and targets, calm (`wshake` < 0.3 mm) |
| **2. More skills** | `rotate_in_hand`, `push`, `press`, … in the same network | A new skill learns faster than from scratch |
| **3. Perception → twin** (6b) | Training on estimated state (noise, delay, occlusion); a perception pipeline that builds the MuJoCo twin from head RGB-D + wrist RGB, first tested in sim with rendered images | Skills keep ≥ 90 % of their success on estimated state |
| **3b. Camera student** (backup) | Depth + wrist-RGB student by DAgger, for what the twin can't see | Within ~10 points of the teacher |
| **4. Real-to-sim** | Object library from photos, scanned workspace | Success on scanned objects and scenes |
| **5. Planner** | LLM/VLM calls skills like functions | Multi-step tasks from a sentence |
| Later | Human motion prior, two hands at once, style vector | |

**RunPod:** Stage 1 gets a Dockerfile with the pinned versions, a one-command launch script, a config file,
checkpoints and logs on a network volume, and resume. It is checked here by tests and a small CPU run. The
first real proof is a ~10-minute smoke run on RunPod, done when the owner decides.

## 8. Honest limits

- RL alone, "trying like a child", is slow. Curriculum, chained starts and (later) demonstrations make it
  practical.
- The skills are learned networks: testable, not readable like code.
- The command interface will change once we see what is hard to train. That is expected, and it is why it is
  written down first.
- Sim-to-real stays the biggest risk: V2.1's friction, tendon stretch and servo strength are estimates until
  the bench tests.

## 9. Owner's answers (2026-11-17) and open questions

Answered: the stage-1 skill list is right; grips `power`, `pinch`, `tripod`; wrist cameras plain RGB; the
scripted planner takes text commands; on the real robot perception builds a MuJoCo twin (6b).

Open:
1. Which computer runs perception next to the robot (a desktop GPU or a Jetson-class board)? Not needed
   until stage 3.
2. Which head RGB-D camera (RealSense D435 class, OAK-D, …)? Also not needed until stage 3.

## Sources

- [MuJoCo Warp (PyPI)](https://pypi.org/project/mujoco-warp/), [batch renderer discussion #1337](https://github.com/google-deepmind/mujoco_warp/discussions/1337)
- [GS-Playground, RSS 2026](https://roboticsproceedings.org/rss22/p093.pdf)
- [Splatting Physical Scenes (MuJoCo + 3DGS)](https://arxiv.org/abs/2506.04120v2)
- [RL-GSBridge](https://arxiv.org/html/2409.20291v1)
- [YUBI Real2Sim (wrist-view 3DGS + MuJoCo)](https://radiancefields.com/papers/yubi-real2sim-3dgs-scene-verification-for-reliable-policy-evaluation)
- [SAM 3D (arXiv 2511.16624)](https://hyper.ai/papers/2511.16624), [MARS: SAM 3D to simulation](https://nalinraut.github.io/blog/2026/mars)
