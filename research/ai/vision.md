# Vision: a bimanual Tendra system on a pole

Working name: **Tendra Duo** (suggestion only, not decided).

## 1. The body

```
            [ head: stereo / RGB-D camera, pan-tilt ]
                          │
                 ┌────────┴────────┐   shoulder bar
     left arm ───┤                 ├─── right arm
     (7 DOF)     │     pole /      │     (7 DOF)
                 │   linear lift   │
  left Tendra ◄──┘   (1 DOF, Z)    └──► right Tendra
  hand (21 DOF)          │                hand (21 DOF)
                   heavy base + table
```

| Part | Plan | Why |
|---|---|---|
| **Pole** | Aluminium extrusion (e.g. 40×40 mm) on a heavy base, fixed in front of a table | Cheap, stiff, easy to mount things on. A fixed base means no balance or walking problem, so all effort goes into manipulation |
| **Linear lift** (optional) | Belt or lead screw moving the shoulder bar up/down the pole | One cheap DOF that makes the workspace much bigger (floor to shelf) |
| **Arms** | 2 × 7 DOF, human layout: shoulder 3, elbow 1, wrist 3 | 6 DOF is the minimum to reach any position + orientation; the 7th gives *redundancy*, so the elbow can move out of the way while the hand stays still (like yours) |
| **Hands** | 2 × Tendra V1 (21 DOF each), left one mirrored | Our own design; the whole point of the project |
| **Head camera** | Stereo or RGB-D (e.g. Intel RealSense, Luxonis OAK-D, or two cheap USB cams), ideally pan-tilt | Sees the whole table; depth helps with grasping |
| **Wrist cameras** | One small wide-angle USB camera per hand | Almost every successful imitation-learning robot (ALOHA, UMI, π0) uses wrist cameras; they see the fingers and object up close |
| **Touch** | Later: fingertip tactile skins, plus servo load readings | Needed for delicate and in-hand tasks (see `ideas.md` for "free" touch from tendon tension) |
| **Compute** | Laptop now → edge GPU (Jetson Orin) or a desktop GPU for running policies; cloud GPUs for training | The dev laptop has no NVIDIA GPU |

**Total DOF:** 2 × 7 (arms) + 2 × 21 (hands) + 1 (lift) + 2 (head) = **59 DOF**. That is a lot. This is why the roadmap starts with one arm and a simple gripper.

### Key hardware questions

- **Hand weight vs. arm payload.** V1 puts 21 servos in the forearm. Weigh the built hand (estimate 0.5–0.8 kg). The arm must carry the hand *plus* the object at full reach, so we need ≥ 1–1.5 kg payload. Cheap hobby arms (SO-101) can't do that; the arm needs stronger actuators (see `resources.md` → Arms).
- **Where do the hand's servos live?** In the forearm (V1 now, simple) or on the pole/upper arm with long Bowden-style tendons (lighter arm, but tendon friction and coupling with elbow/wrist motion). Open question in `ideas.md`.
- **Wrist:** V1 has no wrist joint. The arm's last 3 joints act as the wrist.

## 2. The brain: a layered AI system

Humans don't think about each muscle. Top robotics systems (Figure's *Helix*, NVIDIA *GR00T N1*, Physical Intelligence *π0.5*, Google *Gemini Robotics*) split the brain into fast and slow layers. We copy that idea:

```mermaid
flowchart TB
    U[Person: "put the cup in the sink"] --> L3
    L3["L3 Planner (~1 Hz)<br/>Vision-language model / LLM<br/>splits the task into steps, checks progress"]
    L3 -->|"step: 'grasp the red cup with the right hand'"| L2
    L2["L2 Visuomotor policy (10–50 Hz)<br/>cameras + joint state → action chunk<br/>ACT / Diffusion Policy / VLA (π0, SmolVLA)"]
    SK["Skill library<br/>RL-trained in simulation<br/>(in-hand rotate, precision pinch…)"] --> L2
    L2 -->|"target poses + finger targets"| L1
    L1["L1 Whole-body controller (100–500 Hz, PC)<br/>inverse kinematics, joint limits,<br/>collision avoidance, safety, force limits"]
    L1 -->|"joint targets (rad)"| L0
    L0["L0 Firmware (ESP32 + servo internal loops, ~1 kHz)<br/>MotorDriver HAL, already exists"]
    L0 --> HW[(Arms + hands)]
    HW -->|"joint positions, loads, camera images, touch"| L1
    HW --> L2
    HW --> L3
```

| Layer | Job | Main maths / methods | Status |
|---|---|---|---|
| **L0 Firmware** | Turn joint targets into motor motion, report feedback | Control loops, communication protocols | ✅ v0/v1 firmware exists |
| **L1 Controller** | Keep motion safe and smooth; convert "hand at pose X" into joint angles | Rigid-body transforms, Jacobians, inverse kinematics, quadratic programming | To build (Pinocchio / MuJoCo) |
| **L2 Policy** | See the scene and decide the next ~1 s of motion | Deep learning: transformers, diffusion / flow matching, imitation learning | To build (LeRobot) |
| **Skills** | Dexterous finger tricks humans can't easily demonstrate | Reinforcement learning in simulation, sim-to-real | To build (MuJoCo Playground / Isaac Lab on cloud GPU) |
| **L3 Planner** | Understand language, plan steps, notice failure and retry | Large vision-language models, prompting, tool use | To build (use an existing model, e.g. Claude or an open VLM) |

**Why layers?** Each layer can be built, tested and improved on its own, and swapped out later. It also matches our existing architecture: the PC `Hand` API already sits between L1 and L0, and `SimHand` / `RealHand` let every layer above run in simulation first.

## 3. The data engine (the part that decides success)

Language models learned from the internet. There is no internet of robot motion, so we must **make our own data**. Sources, from cheapest to most valuable:

1. **Simulation** (MuJoCo): unlimited, free, but not quite real. Best for RL skills.
2. **Human videos** of hands doing tasks (public datasets + our own phone videos). Huge, but a human hand is not our hand, so it needs *retargeting*.
3. **Teleoperation demos** on the real robot: expensive in time, but exactly right. Methods: webcam hand tracking → VR headset hand tracking → a custom data glove / leader arms.
4. **The robot's own experience**: autonomous practice with automatic resets, labelled success/failure.

Every episode we record should be saved in **LeRobot dataset format** so it works with open tools and can be shared on Hugging Face. An open Tendra dataset is also a community and business asset.

## 4. Principles

- **Simulation first, always.** Every new ability is tried in MuJoCo with the same `Hand` API before it touches hardware.
- **Decouple AI progress from hardware progress.** Learn and build the AI stack on cheap existing hardware (an SO-101 arm with a gripper) while the Tendra hands mature.
- **Measure everything.** A fixed ladder of benchmark tasks (see `roadmap.md`) with success rates, so we know if we are getting better.
- **Safety.** Force and speed limits in L1, an emergency stop that works without software, torque off at boot.
- **Open source**, like the rest of the project: models, datasets, training code.
