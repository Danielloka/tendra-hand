# Prior art: passing many tendons through a 2-DOF wrist

Date: 2026-11-04. Method: web search only (no paper PDFs could be decoded; WebFetch worked on arXiv HTML pages). Tags: [C] confirmed in a primary source I read, [R] reported by secondary sources or search snippets, [U] uncertain / not verified. Already covered in `research/references/hands/` (Tesla wrist router, 1X 3 wrist DOF, ORCA GT2 wrist, RUKA-v2 existence); this note adds detail and the physics.

## 0. The one idea that matters

A tendon in a sheath (a Bowden cable) is a flexible tube with a cable inside. If both tube ends are fixed, bending the tube in the middle does not change the cable's length at the ends (the tube is incompressible along its axis, so the cable follows a path of the same length). So **wrist motion does not pull or release the fingers if the sheaths run from the forearm to the hand and the wrist only bends the sheath in the middle.** Two things spoil this:
1. The sheath ends are not really fixed: if the sheath is anchored at the forearm and at the palm, a bent sheath of fixed length is fine; but a sheath that can slide, or a cable that leaves the sheath and crosses the wrist in free air off the axis, changes length with wrist angle (length change = offset from the axis x angle).
2. Friction (capstan effect) grows with the total bend angle.

Open (sheathless) routing needs the strand to cross the wrist axis **exactly on the axis** for zero length change. That is Tesla's rule and what Tendra already does at finger joints. Sheaths through a common wrist centre (Tendra's wrist model: all strands pass site `wrist_centre`) make it a mix of both.

## 1. Human wrist [R mostly]

- Nine long flexor tendons (4 FDS, 4 FDP, FPL) pass in the **carpal tunnel**; the roof is the transverse carpal ligament (flexor retinaculum). The extensors run in six compartments under the extensor retinaculum on the back. The retinacula act as **pulleys**: they hold the tendons close to the wrist axes so they cannot "bowstring" (jump away from the bones) when the wrist bends. Evidence: after cutting the transverse carpal ligament, bowstringing appears and flexor excursion efficiency falls, especially with the wrist flexed. [R] https://pubmed.ncbi.nlm.nih.gov/9471069/ (search snippet only, numbers not read)
- Tendons cross close to the wrist centre, so their moment arm about the wrist is small and finger force changes little with wrist angle. I did not find a primary number for the FDP wrist moment arm (not verified, [U]); do not quote one.
- Excursion numbers found: in vivo FDP excursion 24.3 mm (active full fist), 14.0 mm (straight to full fist) [R] (2010 ultrasound study, snippet). Cadaver flexor excursion at wrist level roughly 10-14 mm under simulated active flexion [R]. Relative tendon/connective-tissue shear rises ~32% from 30 deg extension to 30 deg flexion [R] https://media.isbweb.org/images/conferences/isb-congresses/2013/oral/cb-wrist-hand.04.pdf
- **Tenodesis**: the small leftover length change makes fingers close when you extend the wrist and open when you flex it (used by people with spinal injury to grip). A June 2026 paper models it as tendon length conservation: wrist-induced length change is roughly linear in wrist angle with a wrap-based moment arm. [R, from search excerpt, paper not opened] https://arxiv.org/pdf/2606.13601. Lesson for Tendra: some leftover coupling is natural and small; the controller can cancel it from the wrist angle (feed-forward), if it is measurable.
- A wrist-fixed hand therapy fact worth knowing: wrist position changes finger force in humans too (grip is strongest in mild extension) [U, general knowledge, not sourced here].

## 2. Robots and hands, one by one

| System | What they do at the wrist | Key numbers | Tag |
|---|---|---|---|
| **Tesla Optimus V3** (WO2026080687A1) | Cables run as a **lateral stack** in the forearm and a **vertical stack** in the hand; the switch happens exactly at the wrist joint, which keeps each cable's lever arm about the pitch and yaw axes small (less crosstalk, stretch, friction). Two linear actuators drive yaw and pitch through a universal joint. | 25 linear actuators per forearm (23 hand, 2 wrist) [R]. Secondary sources disagree on which axis the lateral stack protects; patent drawings not read. | [C] concept (from earlier notes) / [R] details. https://droids.substack.com/p/the-forearm-is-the-new-hand-inside https://patents.google.com/patent/WO2026080687A1/en |
| **1X NEO** | Motors in forearm pull "proprietary tendons through the wrist". 3 wrist DOF. Low gear ratios (~5:1 to 15:1). | Wrist validated >2 million high-load cycles (company claim). | [R] https://1x.tech/discover/neos-hands https://interestingengineering.com/ai-robotics/1x-unveils-robot-hands-neo-humanoid |
| **RUKA-v2** (NYU, Mar 2026) | **Decoupled parallel 2-DOF wrist.** A passive spherical ball joint sets one shared centre. Each axis has its own rectangular linkage chain driven by its own forearm motor just below the wrist. Finger tendons go through a **hole in the ball joint near the rotation centre**, then a **bearing-supported routing plate** redirects them outward into the forearm (no sharp bends, less tangling). Guides keep tension consistent. | Flexion 45, extension 30, radial 35, ulnar 35 deg. Holds 1215 g (pronation/supination), 835 g (radial/ulnar side up) for 20 s. Wrist motors were the hottest: 46 C peak, +7.25 C. Axial depth 106 mm. No torque or tendon-length-vs-angle numbers published (the paper only says routing "limits" the variation). Hand cost ~$1.5K. | [C] https://arxiv.org/html/2603.26660v2 https://ruka-hand-v2.github.io |
| **CRAFT-Wrist** (arXiv 2609.22681) | Concentric 2-DOF wrist between arm and hand; two servos with **short rigid transmissions** (not tendons), common centre. Claims it is insensitive to tendon pretension/friction/elongation, but gives no measurements. Finger travel was reached at wrist 15/30/45/60 deg; peak finger-motor current 300 mA at 60 deg (17% of limit); out/return current traces differed (friction or stick-slip, not tested). | F/E -18..+80 deg, R/U +-20 deg, depth 79.75 mm. Comparison table: SoftHand Model-W +-90 / +-30, depth 41 mm; RUKA-v2 -30/+45, +-35, 106 mm; DexWrist +-40 / +-40, 178 mm. | [C] https://arxiv.org/html/2609.22681 |
| **ORCA** (ETH) | Only 1 wrist DOF: one XC430-T240BB-T through a **GT2 belt**. Tendons through the tower below the hand; PTFE tubes only for nonlinear runs (e.g. thumb base to wrist). | ~60 deg flexion/extension per one wiki; the paper's number not verified. Humans-side value not needed. | [R]/[U] https://arxiv.org/pdf/2504.04259 https://www.aiwiki.ai/wiki/orca_hand/raw |
| **Shadow Dexterous Hand** | 2 wrist joints (WRJ1 deviation, WRJ0 flexion). Simulation limits (Farama, not hardware): deviation about -28..+8 deg, flexion -40..+28 deg. I could NOT find how Shadow routes its conduits or its torque. | See left. | [U] https://robotics.farama.org/envs/shadow_dexterous_hand/reach/ |
| **DLR Hand Arm System** | 2-DOF wrist within a 26-DOF system, using variable-stiffness joints (BAVS, cam-disk). Tendon routing through the wrist: not found. | n/a | [R]/[U] (search only) |
| **NASA Robonaut 2** | Hand + forearm self-contained; each finger actuation = actuator, tendon, **conduit**, tension sensor, terminator (US 8,467,903); "open wrist structure" lets conduits pass through. Wrist patent US 7,784,363 not read. | n/a | [R] https://www.techbriefs.com/component/content/article/46494-msc-tops-102 |
| **MM-Hand** (arXiv 2604.17245) | 21 DOF, remote motor hub, 1 mm Dyneema in sheaths; compares PTFE tube, Bambu feeder tube, metal spring tube, Shimano SP41. | Measured at wrap 0-180 deg, disk 10-100 mm, 2 kg load. Plain PTFE tube: relatively high friction (extrusion quality). Bambu feeder tube (better PTFE): lower. Metal spring tube: similarly low (point contact), chosen. Shimano SP41: low kinetic but **high static** friction. Friction follows T(L)=T(0)exp(mu*phi). The paper text I read gives no mu values or wrist routing. | [C] qualitative https://arxiv.org/html/2604.17245 |
| **MCR-Bionic Hand** (2606.13601) | Models tenodesis on purpose (wrist-finger coupling as a design feature). | See section 1. | [R] https://arxiv.org/pdf/2606.13601 |
| **SoftHand Model-W** | Carpal-tunnel-like channel routes finger tendons through the wrist; 2-DOF tendon-driven wrist, depth 41 mm. | +-90 / +-30 deg | [R] via CRAFT table and search |
| **InMoov, DexHand, Clone, Allegro, ILDA** | I found nothing citable about their wrist tendon routing. InMoov: wrist is servo-driven, finger lines run in the forearm [U]. Allegro has no wrist [U]. ILDA, Clone: not found. | | [U] |
| Hollow gimbal / rolling-contact / continuum wrist | Search did not surface a dedicated paper. Tesla's rolling-contact joint was dropped by Musk (see earlier notes). Treat as [U]. | | [U] |

## 3. Sheath friction and hysteresis (the strength question)

- Capstan formula: tension ratio = exp(mu x total bend angle). Bend angle is in radians and **adds up** along the whole sheath (forearm S-curves + wrist bend + palm entry).
- Values for mu: 0.04 used in one exoskeleton paper from "prior Bowden studies" [R] https://arxiv.org/pdf/2604.20893. Rule of thumb 0.1-0.2 for PTFE with line is common in my own memory but I did not find a primary measurement for hand-size PTFE; MM-Hand shows standard PTFE tube is clearly worse than better PTFE or a spring tube. **So: measure mu on your own tube and line.**
- Worked numbers (my arithmetic, not from a source): extra wrist bend of 60 deg (1.05 rad): mu 0.04 -> x1.04 (4% loss); mu 0.15 -> x1.17; mu 0.2 -> x1.23. Both ways (flex strand loaded one way, ext strand the other), so a 60 deg wrist swing costs 17% extra at mu 0.15, in addition to what the S-curves already cost (Tendra bends up to 56 deg entering the palm). Total wrap of ~110 deg at mu 0.15 is x1.33.
- Hysteresis: Coulomb friction means going out vs back differ (CRAFT saw different current traces on out and return; KIT notes preload raises friction and hysteresis grows with bend [R] https://publikationen.bibliothek.kit.edu/1000143255/148440045). A Coulomb model with a velocity-sign term and preload offset is the usual fit. With antagonistic loops, preload is the enemy: it adds normal force in every bend, so keep it low (also SCS0009 backs off above 80% load).
- Sheath compression: a long flexible sheath under high tension shortens slightly and can bow; a steel spring sheath is stiffer in compression than PTFE alone, but a spring tube is stiffer to bend [R]. Practical fix: make the sheath ends rigid anchors on both sides of the wrist.
- Wrist-angle dependence is **zero in theory** only if the sheath length and end points are fixed and the cable never leaves the sheath. Any free span across the wrist must pass through the axes (Tesla rule).

## 4. Wrist actuation options seen

| Option | Used by | Notes |
|---|---|---|
| Two servos, each through a rigid linkage into a ball/universal joint (parallel, decoupled) | RUKA-v2 (Dynamixel XM430-class), CRAFT-Wrist (XC430 x2, short rigid) | Compact, strong, no tendons in the wrist drive; CRAFT 2.4x cheaper than RUKA-v2's. Wrist motors ran hottest in RUKA-v2. |
| Direct servo on each axis (serial) | Many hobby hands (InMoov style) [U] | Simple; the second motor rides on the first; bulky at the wrist. |
| GT2 belt | ORCA (1 DOF) | Moves the motor into the forearm tower. |
| Linear actuators + universal joint | Tesla (2 of 25) | Needs forearm room; all-tendon-free wrist drive. |
| Differential bevel gears | Common in arms [U], not confirmed for these hands | Two motors share both axes: double torque for one axis, compact, but backlash. |
| Tendon-driven wrist | SoftHand Model-W, DLR [U] | Same friction problems as the fingers. |
Torque needs: no paper in these results published wrist torque in N m. Evidence of load: RUKA-v2 wrist holds ~0.8-1.2 kg at ~20 cm lever [R, my estimate of lever not given]. For Tendra I estimate: V1 hand + palm mass is a few hundred grams plus a 0.5 kg payload at ~15 cm gives ~1 N m at the wrist peak (my estimate, [U]); verify with the model.

## 5. Not verified / gaps

- Tesla patent drawings (which axis the lateral stack protects); Shadow conduit design; DLR wrist tendon path; 1X wrist mechanism; ILDA, Clone, InMoov details; the ORCA paper's own wrist range; any primary mu for hand-scale PTFE; human FDP wrist moment arm. Anyone with PDF access should check the MM-Hand and RUKA-v2 papers directly (their binary PDFs could not be parsed here).

## What Tendra should copy

1. **Cross every strand on the wrist axes** (Tesla): stack strands in a row along one axis in the forearm, and switch to the perpendicular stack at the wrist. Enforce it as a test in `tendon_router.py`.
2. **Shared centre with a through-hole and a bearing routing plate** (RUKA-v2): all 32 strands pass one hole near the wrist centre, then a plate redirects them into the forearm without sharp bends.
3. **Fixed sheath anchors on both sides of the wrist**, strand not leaving the sheath, so wrist angle cannot change inner length (sheath = constant length).
4. **Drive the wrist with two servos through rigid short linkages into a ball/universal joint** (RUKA-v2/CRAFT), not tendons: stiff, no pretension, easy to calibrate. Give them cooling (they ran hottest).
5. **Use low-friction sheaths**: better-quality PTFE or spring tube, never plain cheap PTFE; measure mu on the real parts and budget for exp(mu x total angle) including the wrist swing. Keep pretension low.
6. **Limit the swing to what helps**: RUKA-v2 uses -30/+45 deg F/E and +-35 R/U; Tendra's -60..+70 / -30..+20 is generous, so check friction at the extremes.
7. **Add a feed-forward correction** (tenodesis-style) from the wrist angle for the small leftover coupling, using servo feedback to measure it.
8. **Plan thermal logging** on wrist servos and show the gear/linkage depth (80-110 mm in the CRAFT/RUKA-v2 comparison) in the forearm budget.
