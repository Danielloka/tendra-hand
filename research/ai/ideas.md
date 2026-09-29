# Ideas, creative bets and open questions

Big labs have money and GPUs. We have openness, cheap hardware, a tendon hand we fully understand, and the freedom to try unusual things. These are the bets that could give Tendra an edge.
Status: 💡 idea · 🔬 investigating · ✅ tried (link the log entry) · ❌ dropped (say why).

## Data: the real bottleneck

### 1. The Tendra kinematic-twin glove 💡
A 3D-printed exoskeleton glove whose joints match the Tendra hand one to one (same DOF, same joint axes as close as a human hand allows), with a cheap angle sensor on each joint (magnetic encoders such as AS5600, or potentiometers).
- **Why:** no retargeting maths and no tracking errors. A glove joint angle *is* a Tendra joint angle. Demos are exact and fast to record.
- **Bonus:** put the servo load readings back on the glove as small vibration motors → the operator "feels" the grip (haptic feedback).
- **Business:** a glove + hand kit is a product on its own.
- Related work: DexUMI, DOGlove, HOMIE, DexCap.

### 2. Robot-free data collection (Tendra UMI) 💡
Wear the glove + a wrist camera identical to the robot's wrist camera, and do tasks at home without the robot. The camera view and joint angles match what the robot will see and do. Idea from UMI / DexUMI.

### 3. Learn from your own videos 💡
Record egocentric video (phone on a head strap) of everyday hand tasks. Extract 3D hand poses (e.g. HaMeR, MediaPipe) and retarget to Tendra. Use it to *pretrain* policies, then fine-tune on a small amount of real robot data (the HAT / EgoMimic recipe).

### 4. Autonomous practice with self-reset 💡
The pole setup is static, which makes resets easy to automate: a tilting tray or a funnel returns dropped objects to a known area. The robot can practise overnight, and a vision model labels each try as success/failure. That turns night hours into data.

### 5. Open data flywheel 💡
Publish the Tendra dataset and a "how to record" guide. Everyone who builds a Tendra hand can contribute demos in the same format. A shared dataset across many identical open hands is something no single lab has, and it's a strong community + business moat.

## Sensing for free

### 6. Tendon tension = touch 💡
Each SCS0009 reports its **load**. With the tendon Jacobian (moment arms, already computed by `tendon_router.py` + MuJoCo), servo loads → joint torques → an estimate of contact force at the fingertips (`τ = Jᵀ F`). That gives force feedback without any skin sensors.
- First experiment: press a fingertip on a kitchen scale at several joint angles; compare the load reading with the scale. Measure how much friction and hysteresis hide the signal.

### 7. Hear and feel with microphones 💡
A cheap contact microphone in the palm picks up slip, contact and texture as vibration. Audio + vision policies have been shown to help on contact-rich tasks.

## Simulation and learning

### 8. Real → sim → real digital cousins 🔬
The digital twin already mirrors the real hand. Next step: after every real episode, replay the commands in MuJoCo and automatically tune friction, tendon stiffness and damping until sim matches reality (system identification by optimisation). A better sim → better RL skills → more transfer. Makes use of work we have already done.

### 9. Skills as tools for a language model 💡
L3 planner = an LLM (Claude or an open model) that calls robot skills like functions: `grasp(object, hand)`, `handover()`, `rotate_in_hand(axis, angle)`. Each skill is a learned policy with a success check. A clean, testable way to get language-driven behaviour long before an end-to-end VLA works on our hardware.

### 10. Morphology co-design 💡
Since both the hand design (Fusion scripts, `tendon_router.py`) and the sim are generated from code, we can search over the design itself: finger lengths, drum radii, joint limits, thumb axis, evaluated by how well an RL policy performs in sim. Let the AI help design its own body.

### 11. Curriculum through the Tendra Ladder 💡
Train skills in ladder order (roadmap), reusing each skill as a starting point for the next. Keep one fixed evaluation set so every change is measured.

## Open questions (research needed)

- **Arm payload:** real weight of V1 → which arm? (weigh V1 once printed)
- **Servo placement:** forearm (V1 now) vs. remote servos with long tendons through the arm. Measure capstan friction for a Bowden sheath over 0°, 90°, 180° of bend.
- **Compliance:** tendons + servos are compliant; is that good (safe grasping) or bad (imprecise)? Measure stiffness per joint.
- **Latency budget:** camera → policy → firmware → motion. What loop rate does the SCS0009 bus allow for 21 servos (sync write/read)?
- **Compute on the robot:** which policy sizes run at ≥ 10 Hz on the laptop CPU vs. a Jetson?
- **Left hand:** mirror the Fusion script, or a single design that works both ways?
- **Wrist:** is the arm's wrist enough, or does the hand need its own wrist flex (like a human's) close to the palm?
