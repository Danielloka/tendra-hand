# 02 — Robot-learning gloves: prior art and lessons (research agent, Sonnet)

About 10 searches, mostly abstracts.

| System | Sensing / wrist pose | Embodiment gap | Lessons |
|---|---|---|---|
| **DexUMI** (CoRL 2025) | exoskeleton per robot hand with joint encoders, fingertip tactile | exo limits the human to the robot's joint space; SAM2 mask + ProPainter inpaint + robot hand overlay | 86 % success, 3.2× faster than teleop; exo flexes under strong grip; new exo per hand |
| **DOGlove** (RSS 2025) | 21-DoF capture, 5-DoF force feedback, < $600, open source | retarget from human | force feedback decisive for teleop |
| **DexCap** (2024) | Rokoko EMF gloves + T265 SLAM per wrist + chest RGB-D, 60 Hz | fingertip IK retargeting (LEAP) | EMF gloves costly and metal-sensitive; T265 discontinued |
| **DexWild** (RSS 2025) | mocap glove + palm camera; 9,290 demos, ~201/h | co-training ~1 robot : 2 human | human data helps generalisation; a small robot set is still needed |
| HOMIE / AirExo-2 | ~$0.5–0.6k exoskeletons | matched kinematics; image adaptation | cheap matched hardware works |
| EgoMimic / EgoDex | head video / Vision Pro hand tracking | masking | large data, occlusion errors |
| LucidGloves | 1 string + pot per finger, ~$23 | — | curl only, no accuracy numbers |

**Answers:** (1) Matched exoskeleton = executable actions but bulky; slim measure-and-retarget suits a human-proportioned hand like Tendra. (2) Same camera mount as the robot + masking/inpainting closes the image gap. (3) Contact labels are mostly missing; tactile gloves (OSMO, DexTac, UniTacHand) are new and alignment is still weak. (4) Sync: one host clock, glove counter, cross-correlate a visible event (LED/tap). (5) **Gap:** no cheap, slim, per-joint glove that keeps the palm free, with published accuracy and an open calibration protocol, for a human-proportioned robot hand with a coupled DIP.

Sources: https://dex-umi.github.io · https://arxiv.org/html/2502.07730v1 · https://dex-cap.github.io · https://arxiv.org/html/2505.07813v2 · https://arxiv.org/abs/2503.03081 · https://arxiv.org/html/2505.11709v1 · https://hackaday.io/project/178243-lucidgloves-vr-haptic-gloves-on-a-budget
