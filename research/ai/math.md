# Maths and theory, in learning order

You don't need all of this before starting. Learn each block **when a project step needs it**, and try it on the Tendra code right away. Free resources are in `resources.md`.

## Block 1: The language of robots (start here)

| Topic | What it is | Where we use it |
|---|---|---|
| **Linear algebra**: vectors, matrices, dot/cross product, solving `Ax = b`, eigenvalues, SVD, pseudo-inverse | The basic toolbox for positions, rotations and data | Everything |
| **Rotations**: rotation matrices, axis-angle, quaternions | Ways to describe orientation; quaternions avoid "gimbal lock" | MuJoCo, camera poses, wrist orientation |
| **Rigid transforms (SE(3))**: 4×4 homogeneous matrices, frames, chaining transforms | "Where is the fingertip relative to the palm / camera / table?" | Kinematics, calibration, vision |
| **Calculus**: derivatives, partial derivatives, chain rule, gradients | How outputs change when inputs change | Jacobians, backpropagation, optimisation |

**Exercise:** compute the index fingertip position from the four joint angles by hand (forward kinematics) and compare with MuJoCo's `data.site_xpos`.

## Block 2: Kinematics and control

| Topic | What it is | Where we use it |
|---|---|---|
| **Forward kinematics** | Joint angles → hand pose | Sim, visualisation |
| **Jacobian** | Matrix mapping joint *speeds* to hand speed; its transpose maps hand *forces* to joint torques | Inverse kinematics, force control, tendon force estimation |
| **Inverse kinematics** (damped least squares, null-space for 7-DOF redundancy) | Desired hand pose → joint angles | L1 controller, teleop retargeting |
| **Tendon kinematics**: tendon Jacobian (moment-arm matrix), capstan friction `T_out = T_in · e^(μθ)` | How tendon length and tension relate to joint angles and torques; why friction grows with each bend | Tendra routing, force estimation |
| **Dynamics**: Newton-Euler, Lagrange, mass matrix, gravity compensation | How forces create motion | Arm control, sim tuning |
| **Control**: PID, impedance/admittance control, trajectory generation | Making motion stable, smooth and compliant (soft, not stiff) | Firmware (already: trapezoidal profiles), L1 |
| **Grasp theory**: contact models, friction cones, grasp matrix, force closure | When does a grasp hold an object? | Grasp planning, understanding failures |

## Block 3: Optimisation and probability

| Topic | What it is | Where we use it |
|---|---|---|
| **Optimisation**: gradient descent, least squares, convex problems, quadratic programs (QP), constraints | Finding the best answer under limits | IK, retargeting, L1 safety filter, training networks |
| **Probability & statistics**: distributions, Gaussians, Bayes' rule, expectation, sampling | Reasoning with noise and uncertainty | Sensor fusion, diffusion models, RL |
| **Estimation**: least-squares fitting, Kalman filter | Estimating hidden state from noisy sensors | System identification, calibration, object tracking |
| **Camera geometry**: pinhole model, intrinsics/extrinsics, calibration, PnP, depth | From pixels to 3D positions | Hand-eye calibration, grasping |

## Block 4: Machine learning

| Topic | What it is | Where we use it |
|---|---|---|
| **Neural networks & backpropagation** | Functions with millions of parameters, tuned by gradients | All learned policies |
| **CNNs / Vision Transformers** | Networks that understand images | Camera input to policies |
| **Transformers & attention** | The architecture behind LLMs; handles sequences | ACT, VLAs |
| **Imitation learning**: behaviour cloning, action chunking, covariate shift | Learning to copy demonstrations; predicting chunks of future actions fixes a lot of jitter | Stage B–D |
| **Generative models**: VAEs, diffusion, flow matching | Learning a *distribution* of good actions, not one average (averaging two good grasps gives a bad one) | Diffusion Policy, π0 |
| **Reinforcement learning**: MDPs, policy gradients, PPO, SAC, reward design | Learning by trial and error from a reward | Dexterous skills in sim |
| **Sim-to-real**: domain randomisation, teacher-student distillation, system identification | Making sim-trained skills work on the real robot | Stage C |
| **Vision-language models** | Models that read images and text | L3 planner, VLAs |

## Block 5: Advanced (later, as needed)

- **Lie groups for robotics** (SO(3), SE(3), exponential map, twists/wrenches): the clean way to do kinematics; *Modern Robotics* is built on it.
- **Model predictive control (MPC)** and trajectory optimisation, including contact-implicit methods.
- **Differentiable simulation** (MuJoCo MJX / Warp): gradients through physics.
- **Representation learning**: self-supervised pretraining on video (masked autoencoders, contrastive learning).
- **Scaling**: distributed training, mixed precision, dataset curation.

## Suggested weekly rhythm

- 3–5 hours of maths/theory from one book or course
- The rest building: every concept becomes a small script in `research/experiments/` or a feature in `software/tendra`
- One log entry per week in `research/log.md`: what you learned, what worked, what didn't
