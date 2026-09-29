# Resources

Compiled 2026-09-28 from knowledge up to mid-2026. The field moves fast: check for newer versions and add what you find. ⭐ = start here. Papers are listed by title, group and year; search the title to find them (arXiv, project pages).

## Books and courses (free unless noted)

| Resource | Covers | Link |
|---|---|---|
| ⭐ *Modern Robotics* — Lynch & Park (book + Coursera videos) | Transforms, kinematics, Jacobians, dynamics, control, grasping | modernrobotics.northwestern.edu |
| ⭐ *Robotic Manipulation* — Russ Tedrake, MIT 6.4210 | Manipulation end to end: perception, grasping, planning, learning, with code | manipulation.csail.mit.edu |
| *Underactuated Robotics* — Russ Tedrake | Dynamics, optimal control, MPC, trajectory optimisation | underactuated.mit.edu |
| *A Mathematical Introduction to Robotic Manipulation* — Murray, Li, Sastry | Rigorous grasp theory and kinematics (hand-focused) | free PDF online |
| ⭐ *Mathematics for Machine Learning* — Deisenroth, Faisal, Ong | Linear algebra, calculus, probability for ML | mml-book.github.io |
| 3Blue1Brown: *Essence of Linear Algebra*, *Essence of Calculus*, *Neural Networks* | Visual intuition | YouTube |
| ⭐ Andrej Karpathy: *Neural Networks: Zero to Hero* | Build backprop, MLPs and a GPT from scratch | YouTube |
| *Reinforcement Learning: An Introduction* — Sutton & Barto | RL theory | incompleteideas.net/book |
| OpenAI *Spinning Up in Deep RL* | Practical RL (PPO, SAC) | spinningup.openai.com |
| Berkeley CS285 *Deep RL* — Sergey Levine | RL + imitation learning, research level | YouTube |
| Stanford CS231n | Vision with deep learning | YouTube / course site |
| ⭐ Hugging Face LeRobot docs + tutorials | Practical robot learning, datasets, SO-101 | github.com/huggingface/lerobot |
| *Probabilistic Robotics* — Thrun, Burgard, Fox (paid) | Estimation, Kalman filters | — |

## Key papers and systems

### Imitation learning and teleoperation
- ⭐ **ALOHA / ACT** — *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware*, Zhao et al., Stanford 2023. Cheap bimanual setup + Action Chunking Transformer. Our Stage B/D template.
- **Mobile ALOHA** (2024), **ALOHA Unleashed** (Google DeepMind 2024): scaling ALOHA-style data.
- ⭐ **Diffusion Policy** — Chi et al., Columbia/TRI 2023. Actions generated like images in diffusion models.
- **3D Diffusion Policy (DP3)** — 2024. Point clouds instead of images; data-efficient.
- **UMI** — *Universal Manipulation Interface*, Chi et al. 2024. Hand-held gripper with camera to collect data without a robot. **DexUMI** (2025) does the same for dexterous hands with a wearable exoskeleton.
- **GELLO** — 2023. Cheap 3D-printed leader arms for teleop.
- **AnyTeleop** (NVIDIA 2023), **DexPilot** (NVIDIA 2020): vision-based hand teleop and retargeting.
- **Open-TeleVision** (2024), **Bunny-VisionPro** (2024): VR/AR headset teleop of bimanual dexterous hands.
- **HATO** — *Learning Visuotactile Skills with Two Multifingered Hands*, 2024. Bimanual hands with touch, low-cost.
- **DexCap** (Stanford 2024): mocap glove data for dexterous imitation.

### Vision-Language-Action models (robot foundation models)
- ⭐ **π0 / π0.5** — Physical Intelligence 2024/2025. Flow-matching action expert on a VLM; open weights (openpi). Strong on dexterous, bimanual tasks.
- ⭐ **SmolVLA** — Hugging Face 2025. Small (~0.5B) open VLA, trains on consumer GPUs, in LeRobot.
- **GR00T N1** — NVIDIA 2025. Open humanoid foundation model with fast/slow (System 1/System 2) design.
- **OpenVLA** (2024), **Octo** (2024), **RDT-1B** (bimanual diffusion transformer, 2024).
- **RT-2** (Google DeepMind 2023): VLM → actions, the idea that started VLAs.
- **Helix** (Figure 2025), **Gemini Robotics** (Google DeepMind 2025): closed, but their architecture write-ups are worth reading.

### Dexterous hands and RL / sim-to-real
- ⭐ **OpenAI Dactyl** — *Learning Dexterous In-Hand Manipulation* (2018) and *Solving Rubik's Cube with a Robot Hand* (2019). Domain randomisation at scale.
- **DeXtreme** (NVIDIA 2022): in-hand reorientation with Isaac Gym, sim-to-real.
- **HORA** — *In-Hand Object Rotation via Rapid Motor Adaptation*, Qi et al. 2022. Teacher-student, cheap and robust.
- **Visual Dexterity** — Chen et al., MIT, *Science Robotics* 2023. Reorienting new objects with a camera.
- **Humanoid Policy ~ Human Policy (HAT)** (2025), **EgoMimic** (2024): learning from human egocentric video together with robot data.
- **DexGraspNet** (2023) and follow-ups: large synthetic grasp datasets.

### Low-cost, open-source hands (study their design choices)
- ⭐ **LEAP Hand** (CMU 2023): cheap, direct-drive Dynamixel hand, widely used in research.
- ⭐ **ORCA Hand** (ETH 2025): open-source tendon-driven hand, very close to Tendra's approach.
- **DexHand**, **InMoov**, **AmazingHand** (Pollen Robotics / Hugging Face 2025).
- Commercial references: Shadow Hand (tendon-driven), Allegro, Inspire, Tesla Optimus hand.

### Touch sensing
- **GelSight / DIGIT**: camera-based high-resolution tactile fingertips.
- **ReSkin** (2021) and **AnySkin** (2024): cheap magnetic skins, replaceable.
- *Learning the signatures of the human grasp using a scalable tactile glove* — Sundaram et al., *Nature* 2019.

## Datasets
- **Open X-Embodiment** (2023): >1M robot episodes, many robots.
- **DROID** (2024): large diverse single-arm dataset.
- **AgiBot World** (2025): large dual-arm dataset.
- **LeRobot Hub** on Hugging Face: community SO-100/101 and ALOHA datasets in the format we will use.
- Human hands: **Ego4D**, **EgoDex** (Apple 2025, Vision Pro hand tracking), **HOI4D**, **DexYCB**, **ARCTIC**, **OakInk**.
- Objects: **YCB** object set (standard benchmark objects you can buy or 3D print).

## Software and simulators

| Tool | Use | Runs on our laptop? |
|---|---|---|
| ⭐ **MuJoCo** (already used) | Physics sim, our digital twin | Yes |
| **MuJoCo Playground / MJX / MuJoCo Warp** | GPU-parallel MuJoCo for fast RL | Cloud GPU |
| **Isaac Lab** (NVIDIA) | Large-scale RL, photorealistic rendering | Cloud GPU only |
| **ManiSkill3**, **Genesis** | Fast GPU manipulation sims | Cloud GPU |
| ⭐ **LeRobot** (Hugging Face) | Datasets, teleop, ACT, Diffusion Policy, SmolVLA, π0 | Yes (training on cloud) |
| **Pinocchio** | Fast kinematics and dynamics, Jacobians | Yes |
| **dex-retargeting** | Human hand pose → robot hand joints | Yes |
| **MediaPipe Hands** | Webcam hand tracking | Yes |
| **mink** | Differential IK on MuJoCo models | Yes |
| **PyTorch** | Deep learning | Yes (CPU; slow for training) |
| **OSQP** / **CVXPY** | QP / convex optimisation | Yes |

**GPU compute:** Google Colab and Kaggle (free, limited), then pay-per-hour clouds (RunPod, Lambda, Vast.ai) for bigger runs. For running policies on the robot later: NVIDIA Jetson Orin (edge) or a desktop with a used RTX 3060/3090.

## Hardware for the bimanual system

### Arms
| Option | Notes |
|---|---|
| ⭐ **SO-101** (LeRobot, open source, Feetech STS3215) | Cheapest way to learn; 3D-printed; leader + follower. Payload too low for a Tendra V1 hand, so use it with a gripper for Stage B |
| **Koch v1.1** | Similar, Dynamixel-based, pricier |
| **OpenArm** (Enactic, open source, 2025) | 7-DOF human-like arm designed for bimanual research, CAN motors; a strong candidate for Stage C/D |
| **AgileX PiPER**, **ARX** arms | Commercial, low-cost 6-DOF arms with ~1–1.5 kg payload |
| **Build our own** | Quasi-direct-drive BLDC actuators with planetary/cycloidal gearboxes (moteus, ODrive, CubeMars/Damiao-style motors). Most learning, most work |

### Teleop devices
- Webcam + MediaPipe (free, start now)
- Meta Quest 3 hand tracking (affordable VR)
- Data glove / exoskeleton glove (see Tendra glove idea in `ideas.md`)
- Leader arms (SO-101 leader, GELLO)

### Cameras
- Intel RealSense D405/D435, Luxonis OAK-D (depth); small wide-angle USB cameras for wrists
