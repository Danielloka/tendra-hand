# Research log

Lab notebook for Tendra Hand. Newest entries at the top.

**Entry template**
```
## YYYY-MM-DD: Short title
**Goal:** what I wanted to find out / do
**Setup:** hardware, software versions, settings
**Result:** what happened (numbers, photos, plots)
**Conclusion / next:** what I learned and what to try next
```

---

## 2026-09-29: Thumb tendon routing, design study (V1)
**Goal:** close V1's last open routing item: get the 10 thumb strands from the bay floor to their drums.
**Setup:** Fusion design "Tendra Hand V1" via the Fusion MCP (read-only sections, 1 mm grid). Owner's choice: PTFE sheaths through the 2-axis thumb base, on-axis crossings further out. Details: `research/experiments/2026-09-29-thumb-routing/`.
**Result:**
- The thumb base is a frame (bottom plate, side walls, top arm, 4 mm pins top and bottom). The metacarpal sits right on the plate; the only free space is inside the frame, behind the metacarpal. The cmc_flex axis is 5.5 mm in front of the cmc_rot axis.
- **No path for sheaths** in the current base: anything crossing from the palm into the rotating frame is cut, except along the rot axis, and that's blocked by the solid bottom pin.
- **Bug:** the two `cmc_rot` strands rise vertically from the floor, parallel to the rot axis, so they can't turn the base. They must reach the drum groove horizontally. The MuJoCo model hides this with a virtual guide point on the palm.
- Two concepts: **A** rework the base (hollow bottom journal, tubes up the rot axis, room to loop them into the metacarpal, horizontal feed for cmc_rot); **B** an outside tube loop from the palm edge to the metacarpal (ORCA style, needs ~100 mm slack).
- Owner chose **A**. A 2D loop search (`sheath_loop_search.py`) sized it: with the base plate and bay floor lowered **4 mm** along the rot axis (no kinematic change), the tubes can rise up the hollow pivot, bow over and enter a boss on the metacarpal's top (just above the cmc_flex axis, 45° up-forward), with a worst-case bend radius of **15.5 mm** over the whole cmc_flex range (11.0 mm without lowering).
**Conclusion / next:** build in 6 tested stages (router → base → palm → metacarpal → on-axis holes → export + checks), listed in the experiment README.
- **Stage 1 done (same day):** the router now has the whole thumb base: hollow journal (bore Ø11.6), cmc_rot drum r 7.5 fed horizontally over a Ø3 pin in the palm (**servo_per_joint 1.25**, firmware and sim updated; sim ctrl is now the joint angle for every joint), cmc_flex drum over the rot axis with a Ø3 pin on a hanger, sheaths in 2 × 3 in the bore to a split boss on the metacarpal (loop bend ≥ 14.3 mm, no crossing), bare cmc_flex strands held under their mid-range position (≤ 0.25 mm length change over −100…40°). Thumb servos re-slotted (6 → F1, 7 → F2, 8 → B1). 13 new tests; full suite 191 passed.
- **Stage 2 started:** `thumb_base` built in Fusion (one solid, checked in section). Rebuilding the palm and forearm needs a bulk delete of their timeline features, which the permission check refused; waiting for the owner.

## 2026-09-29: One place for project media
**Goal:** a single folder for photos and videos that the website, the READMEs and Claude can all use.
**Setup:** new top-level `media/` (`photos/`, `videos/`, `screenshots/`, `diagrams/`, `catalog.json`, git-ignored `inbox/`), `media/process_inbox.py` (Pillow), website copy via `scripts/sync-styleguide.mjs`.
**Result:** tested with a fake 4000×3000 phone photo with GPS, rotation and date in its EXIF data. It came out upright at 1500×2000, with no EXIF, named by the date it was taken and catalogued. The website copy lands in `public/media/`.
**Conclusion / next:** drop the first real photos of the V0 prototype in `media/inbox/` and replace the gallery placeholders.

## 2026-09-29: Teleop screenshots: sideways angles, mirrored view
**Goal:** check the owner's three teleop screenshots (open hand, spread hand, rock sign; the owner used the **left** hand). They are in `media/inbox/` and are not committed: one shows faces (see `media/README.md` rules).
**Result:**
- Tracking looked solid (20–23 fps, 57–72 ms delay), and the rock sign came out roughly right.
- **Abduction reference was wrong.** Sideways angles were measured from each finger's wrist → knuckle line. On a human hand those lines fan out (index ≈ 37° in screenshot 2), so a spread index read ≈ 0° and fingers held together read as bent toward the middle. The robot showed the index and middle converging while the human's spread. Now every finger is measured from the palm's up axis, which matches Tendra (fingers parallel at `mcp_abd` = 0).
- **Abduction fades with flexion:** robot `mcp_abd` = human abduction × cos(MCP flexion), because the collateral ligaments stop sideways motion near 90° of knuckle bend. This also keeps curled fingers from converging into each other.
- **View:** the robot (a right hand) is shown mirrored only when a right hand is tracked, so it always looks like the hand in the mirrored camera image. Status text now sits on a dark bar; **D** shows every joint target in degrees.
**Conclusion / next:** owner re-tests: spread vs together, fist, and compares the D readout with their own hand.

## 2026-09-29: Fist bug fixed, more precise finger maths
**Goal:** the owner's test: closing the hand into a fist made every finger go straight, as if they bent backwards. Also make the joint angles more precise.
**Setup:** `software/tendra/retarget.py`; synthetic hands with known angles, random rotations, left and right hands, 2–3 mm Gaussian landmark noise (webcam-like).
**Result:**
- **Cause:** the palm side was found each frame from `sum(dir(DIP-PIP) - dir(PIP-MCP))`, the direction the PIP joints bend toward. In a fist the fingertips point back at the wrist, so that sum flips and the palm is taken to face the other way. Every angle becomes negative and is clipped to -5 deg ("straight"). Reproduced by a new test first (4/4 failing).
- **Fix: bend-axis cue.** For successive bones a, b: (a x b) . x = +sin(bend) on a right hand (x = thumb-side axis) for any bend from 0 to 180 deg, and negative on a mirrored hand. Summed over all 12 finger joints, it keeps its sign in a fist.
- **Palm plane:** least-squares plane (SVD) through the wrist and all four knuckles, instead of three points.
- **Finger model fit:** each finger is a small model (MCP = sideways, then flexion; PIP/DIP hinges; one bending plane). Gauss-Newton with light Levenberg-Marquardt damping fits (abd, mcp, pip, dip) to the PIP, DIP and tip landmarks, starting from the closed-form estimate or last frame's fit, whichever fits better. Analytic Jacobian, checked against finite differences. All four fingers are fitted in one batched NumPy computation. PIP/DIP in the closed form are now signed angles about the hinge axis, with the bones projected onto the bending plane.
- **Bone lengths:** running average (rate 0.05), since bones don't change length. Known lengths cut the error by a further 0.5–1 deg in simulation.
- **Precision** (3 mm noise, mean abs error, deg, [abd, mcp, pip, dip]): relaxed hand, direct [7.0 6.0 10.7 14.4] vs fit [5.9 6.1 11.2 15.2] (about equal); **fist, direct [13.6 6.5 9.8 13.8] vs fit [5.6 4.8 8.7 12.0]** (sideways error more than halved). The remaining error is noise on short bones (22 mm fingertip bone); the One Euro filter smooths it over time.
- **Speed:** a 1e-3 rad stop tolerance (far below the noise) and the batched fit give ~7.5 ms per frame for v1 (was 6 ms with the less precise method). Measurements earlier that day were distorted: Bambu Studio and Fusion 360 held the CPU at 100 %.
- Tests: 25 retargeting tests (fist regression, Jacobian vs finite differences, fit beats the direct estimate in a noisy fist, bend cue sign), 178 total.
**Conclusion / next:** owner re-tests the fist. Remaining limit: the robot's PIP stops at 95 deg, while a human fist reaches ~100–110 deg (clipped). Next precision step would be a temporal model (e.g. a Kalman filter on the joint angles with a motion model) instead of per-frame fits + One Euro.

## 2026-09-29: Faster hand model in teleop (drawing, not maths, was the bottleneck)
**Goal:** tracking now felt fast and accurate, but the MuJoCo hand lagged behind.
**Setup:** benchmarks on the i3-1125G4 with Intel UHD: MuJoCo 3.14 offscreen renderer and the passive viewer, v1 model.
**Result:**
- Cheap parts: kinematic pose update (`mj_forward`) 0.19 ms, physics 1.2 ms per 60 Hz frame, retargeting 6 ms.
- **Drawing was the problem.** One v1 frame (offscreen, 640×480): 2,050 ms with shadows and reflections, 770 ms without. Of that, the **42 tendons** are ~2,000 capsule segments at 28×16 slices each: hiding them gives 70 ms. The meshes have 238k triangles (palm 71k from the channels, forearm 56k, spools 34k). Simplified to 15 % (43k, `tendra.lite_model`, fast-simplification) plus tendons hidden: **21 ms**.
- **The CPU and integrated GPU share one power budget.** The hand network alone runs at 28 fps. With MuJoCo's viewer open and idle: 17 fps. Viewer + lite meshes + pose updates: **7 fps**. The cheaper each frame, the more frames the viewer draws (it redraws nonstop, with no rate cap in the API), and the more it throttles the CPU.
- Fix: teleop draws the hand itself next to the camera image (`tendra.hand_view`), only when the pose changed and at most 30 fps. Network speed with this view: 23–26 fps. Kinematic mode (joints set straight from tracking) by default, so the servo simulation adds no lag; `--physics` keeps it. The in-between gliding was removed: with readings at 20–28/s and drawing at ≤ 30 fps it only added ~25–50 ms of delay.
- A render costs ~20 ms no matter the size (240 px: 15 ms, 480 px: 20 ms), so it's a fixed GPU round trip, not pixels.
- Tests: `software/tests/test_lite_model.py` (same structure, masses and kinematics, < 30 % of the triangles). 170 passed.
**Conclusion / next:** measurements without a hand in view vary run to run (palm detection on every frame). Owner to re-test with their hand. If more speed is needed: a lighter hand-tracking model, or rendering in its own thread.

## 2026-09-29: Smoother webcam teleop
**Goal:** the owner's first live test worked (the twin follows their hand) but felt slow and laggy.
**Setup:** profiled each stage on the i3-1125G4 (v1 model).
**Result:**
- Everything ran one after another in one loop: waiting for a camera frame ~34 ms, network 20–45 ms, retargeting 6 ms, physics (v1: 0.25–0.65 ms per 2 ms step). So the whole loop, including the MuJoCo view, updated only ~10 times per second.
- Not the cause: the simulated servos (v1 reaches 90 % of a 60° step in 70 ms), the 1,966 hidden routing sites (not drawn), and the GIL (MediaPipe releases it; the main thread still gets ~600 turns/s while it runs).
- Fix: three threads. The camera grabs frames (`CameraStream`), a tracking thread runs the network and retargeting on the newest frame, and the main loop steps physics and syncs the viewer at up to 60 Hz with the latest targets, keeping sim time in step with wall time.
- One Euro filter 1.5 Hz / β 0.4 → 2.0 Hz / β 0.8: lag on a 1 Hz motion 67 → 33 ms, jitter at rest 0.34 → 0.35° (simulated noise).
- Measured (no hand in view, so the palm detector runs on every frame, the slowest case): main loop 41–46 Hz (was ~10), hand updates 18–21/s (was ~10), camera-to-targets delay ~65 ms. The app prints these numbers when it closes.
**Conclusion / next:** owner re-tests. If it's still not smooth: interpolate targets between hand updates, and check whether the webcam drops to 15 fps in dim light (auto exposure).

## 2026-09-28: Webcam teleoperation of the digital twin
**Goal:** move the MuJoCo Tendra hand with your own hand in front of a webcam (Stage A in `research/ai/roadmap.md`).
**Setup:** MediaPipe 1.0.1 Hand Landmarker (pretrained, float16 model, CPU) + OpenCV 5; `software/tendra/retarget.py`, `software/tendra/hand_tracking.py`, `sim/teleop.py`.
**Result:**
- **Fingers:** joint angles measured straight from the 3D landmarks. MCP flexion is the bone's elevation out of the palm plane; abduction is its direction within that plane; PIP/DIP are signed bends about the finger's hinge axis. Synthetic hands with known angles, in random rotations and as left or right hands, come back exact (1e-6 rad).
- **Thumb:** the Tendra thumb points straight out of the palm at q = 0, so it can't copy human angles. Damped least squares on the MuJoCo Jacobians matches the thumb's MCP, IP and tip (weights 0.3/0.6/1.0) to the human ones, scaled by MCP→IP→tip length. Scaling from the base was wrong: the two thumb-base axes don't meet, so base→MCP changes with the pose. A robot thumb pose fed back in is recovered within 1 mm.
- **Pinch:** when the human thumb and index tips are closer than 5 cm, the tip target blends toward the robot's index tip. From a cold start it reaches 1–4.5 mm in one frame. **Finding:** Tendra can only pinch with a curled index (an "O" pinch). With the index at (34°, 34°, 17°) the thumb gets no closer than 24 mm (v0 and v1), so the thumb's reach is worth a look in the next thumb revision.
- **Palm side:** MediaPipe's left/right label assumes a mirrored image, so it's the opposite on a normal webcam. We only use it as a first guess; the palm side comes from the direction the PIP joints bend.
- **Smoothing:** One Euro filter (min cutoff 1.5 Hz, beta 0.4). Calibration: press C with the hand open and flat.
- **Speed (i3-1125G4):** retargeting 3.7 ms (v0) / 6.1 ms (v1) per frame. The network takes 42 ms per frame while looking for a hand (~10 fps incl. camera). The first run's camera frame was dark (mean 11/255), so no live hand has been tracked yet.
- Tests: 17 new tests in `software/tests/test_retarget.py`.
**Conclusion / next:** owner's live test: check that the palm side is detected, that the MCP-flex zero is right after calibrating, and how the filter feels. Then record teleop sessions to the LeRobot dataset format (the first data for imitation learning) and try `--fake` / the real v0 hand.

## 2026-09-28: Embodied-AI research hub and the bimanual pole robot
**Goal:** start long-term research on the AI system that will control two arms with two Tendra hands on a pole, and gather resources and ideas.
**Setup:** new folder `research/ai/` (desk research, no experiments yet).
**Result:**
- **Body:** fixed pole (aluminium extrusion) with an optional linear lift, 2 × 7-DOF arms, 2 × Tendra V1 hands (left one mirrored), a head camera and one wrist camera per hand. About 59 DOF in total.
- **Brain:** four layers, like Helix / GR00T / π0.5: L3 language planner (~1 Hz) → L2 visuomotor policy (10–50 Hz, ACT / Diffusion Policy / VLA) plus RL skills trained in sim → L1 whole-body controller (IK, safety, 100–500 Hz) → L0 firmware (exists).
- **Main conclusion:** data is the bottleneck, not algorithms. Record everything in LeRobot format.
- **Stages A–E**, plus a benchmark ladder of 9 tasks. Start the AI stack now on a cheap SO-101 arm with a gripper, separate from the hand hardware.
- **Creative bets** (`ideas.md`): a kinematic-twin data glove, robot-free data collection, pretraining on your own egocentric video, fingertip force estimated from servo load through the tendon Jacobian, self-reset for overnight practice, an LLM that calls skills as tools, and co-design of the hand's shape in sim.
**Conclusion / next:** Stage A: maths block 1, LeRobot + ACT on a simulated task (free cloud GPU), and webcam (MediaPipe) teleop of the Tendra hand in MuJoCo. Open question: V1 weight vs. arm payload (weigh V1 once it's printed).

## 2026-09-28: New ultimate goal — a robot that does what humans do
**Goal:** widen the project's goal beyond a dexterous hand.
**Result:** the ultimate goal is now a robot that can do what humans do (cook, do chores, use tools) as well as a human, and that *understands* complex tasks instead of only picking things up. The hand stays the focus, since it's the hardest and most important part.
**Conclusion / next:** `docs/roadmap.md` now opens with this goal and gains two new phases: task understanding (VLA models, planning, recovery, learning from demos) and arm, body and real kitchen and household chores. Later the same day the phases were renumbered (owner): **Tendra Hand V1 is now Phase 2**, right after the V0 prototype (Phase 1), since V1 is the first real hand. The old "servo upgrade" and "full hand" phases merged into it; the V0 servo upgrade is dropped (V1 goes straight to servos). New order: 0 Foundation, 1 V0, 2 V1, 3 Mechanics + system ID, 4 Sensing, 5 AI control, 6 Vision + grasping, 7 Task understanding, 8 Arm, body and chores. The website says the same: roadmap title and new phases (`website/content/roadmap.json`), the goal on `/project`, the homepage lead, the site description, Getting started, and a new FAQ "What is the end goal?".

## 2026-09-28: Tendra Hand V1 — 5 fingers, 21 DOF, forearm servos, one channel per tendon
**Goal:** a full hand in the prototype's style: 4 fingers + a 5-DOF thumb, each joint on its own Feetech SCS0009, with every tendon strand in its own route through the palm and wrist to a servo in the forearm.

**Setup:** Fusion design "Tendra Hand V1" (a cloud copy of "Hand assebly"; the original is untouched), built in stages by `hardware/cad/fusion_scripts/TendraHandV1/`. Routes and servo layout from `hardware/cad/tendon_router.py` → `hardware/robot_description/v1_export/tendon_routes.json`. Research: `research/experiments/2026-09-28-full-hand/research.md`.

**Result:**
- **Fingers:** middle, ring and little are copies of the index. Only the plain, prismatic middle of the proximal/middle phalanx is lengthened or shortened, so joints, holes and tendon slots keep their size. Proximal/middle length vs index: middle +4/+3 mm, ring +1/+2, little −8/−4 (human ratios, Buryanov & Kotiuk 2010). Knuckles 19 mm apart, knuckle arc: middle +4 mm, ring +1, little −7.
- **Thumb 5th DOF:** `thumb_mcp_abd`, a hinge in the proximal phalanx in the index-knuckle style: two stub pins at the ends of the axis, a 1 mm slot on the axis for the pass-through tendons, a groove on the barrel as the drum. No interference at 0 and ±20°.
- **Palm:** full width (x −79…12, y 12.5…40, z −30…knuckles), knuckle posts copied from the index post. 42 channels: 6 mm of 1.2 mm bore at the entry (bare line, the step stops the tube), then 2.2 mm for a 1 × 2 mm PTFE tube along an S-curve (bend radius ≥ 15.7 mm) to the wrist. Index strands must go sideways beside the thumb bay (like the prototype).
- **Forearm:** 21 SCS0009 in two levels × front/back + one on a third level; shafts point inward, each deeper level sits closer to the centre, so every strand is a straight, unblocked line from the wrist to its spool (radius 6 mm, 1:1 with the 6 mm joint drums). Room on the third level for the ESP32-S3 and FE-URT-1.
- **Checks in Fusion:** 0 interferences among the 44 static parts; no interference of finger bases (±15°) or the thumb (whole −100…40° rotation) with the palm; all 42 strand centre lines clear from entry to spool (sampled every 0.5 mm). Two issues found and fixed: thumb spools hit the forearm wall (column pitch now 16 mm, flange radius 7.5 mm), and a leftover old thumb body.
- **Tests:** router 12 tests (wall ≥ 0.8 mm between channels, bends, clear lines, servo fit); MuJoCo v1 (`sim/convert_v1.py`): coupling between joints 5.6e-10 mm/rad, own-joint moment arm exactly ±6.000 mm, loop length constant to 2e-13 mm, tracking ≤ 0.31° with gravity, no contacts open/fist. Full suite: 148 passed. Firmware: both envs build, 356 SCS host checks pass. Nothing tried on real servos yet.

**Conclusion / next:**
- Known limits: fingers can only adduct ~4–5° toward a straight neighbour (8 mm gap); index strands bend up to 56° entering the palm (friction; consider a metal pin or PTFE guide there); thumb internal routing (on-axis crossings) is still open; the thumb-metacarpal/palm contact is excluded in the sim (convex hull), so the real contact at cmc_flex ≈ 80° is missed.
- Forearm is 85 mm thick (servos front and back); a slimmer layout would need idler pulleys or Bowden tubes.
- Next: print a test finger + palm section to check the 1.2/2.2 mm channels and PTFE fit, buy SCS0009 + hub board + 5 V ≥ 15 A supply, measure the real spline/tab sizes, pretension study.

---

## 2026-09-28: Real hand model on the website
**Goal:** replace the placeholder hand on the homepage with the real CAD, spinning with the scroll.
**Setup:** `hardware/cad/Hand assebly.step` → `website/scripts/step-to-glb.py` (OpenCascade via `cadquery-ocp`, `RWGltf_CafWriter`) → `website/public/models/hand.glb`.
**Result:**
- The first export had one mesh per CAD face (~700 meshes) and froze the browser. `SetMergeFaces(True)` gives one mesh per part: 9 meshes, ~13k triangles, 0.4 MB.
- Fusion's STEP (Z-up, mm) converted to glTF Y-up already matches the site frame: fingers +Y, thumb out toward +Z, palm plate toward −X.
- The homepage timeline now turns the hand one full turn per story section, linearly with scroll.
- Rigging (same day): the STEP and the URDF export share the design frame. Every STEP part's bounding box matches a MuJoCo body to <0.1 mm, so the script matches them automatically and adds joint nodes at the URDF pivots, using the MJCF axes (already flipped so positive = closing). Checked by posing the GLB offline: the index curls toward the palm and the thumb swings across to it. The hand lab shows the bend and the joint labels.
- The site's `thumb_cmc_rot` range was 55°; lowered to the real +40° limit.
**Conclusion / next:** the real model now drives every scroll effect. The STEP must stay in sync with the URDF export (same Fusion design), or matching fails loudly.

## 2026-09-27: Website v1 (Next.js + scroll-driven 3D hand)

**Goal:** Build the project website on the design system, with a 3D hand that tells the story as you scroll.

**Setup:** Next.js 16 (App Router, Turbopack), TypeScript, Tailwind v4 mapped to the design tokens, React Three Fiber 9 + drei 10, GSAP 3 ScrollTrigger + Lenis, MDX via `@next/mdx`. Built by a lead agent plus Content, 3D, Docs, Pages and Scroll agents, then design/accessibility/performance/code reviews (plan in `website/PLAN.md`).

**Result:**
- 26 static routes: home, project, hardware, software, docs (7 pages with search, table of contents, prev/next), build log (3 posts), gallery (lightbox), contribute, 404, sitemap, robots, OG image. A hand lab at `/dev/hand` has sliders for every animation value.
- Placeholder hand built from primitives with all 17 joints of the planned full hand; the real `hand.glb` swaps in with one line (`src/components/three/model.ts`).
- Lighthouse (production build, local): every route 90+ on mobile and 100 on desktop for performance; 100 for accessibility, best practices and SEO. axe: no serious/critical issues in light or dark.
- The homepage first scored 66 on mobile: parsing three.js (~270 kB gzipped) and building the scene blocked the main thread for ~3.5 s under Lighthouse's CPU throttling. Deferring the environment map / edge geometry and compiling shaders asynchronously didn't help much. What worked: loading the 3D scene on the first interaction (scroll, touch, mouse, key; or after 10 s). Mobile went to 95, TBT 130 ms.
- Design-system changes: `.grid` → `.auto-grid` (clashed with Tailwind's `grid`), mobile menu below 1024px, lighter secondary buttons in dark mode. Code blocks use Shiki's high-contrast GitHub themes (the normal ones failed contrast).

**Conclusion / next:** Replace the placeholders (photos, videos, `hand.glb`, a static hand render for no-JS/reduced motion), choose hosting and a domain (`content/site.json`), add CONTRIBUTING.md and a code of conduct.

---

## 2026-09-27: Pinch points on the index knuckle (MCP flex + sideways), design change, not yet printed

**Goal:** Finish the index so every pass-through tendon crosses every joint on its axis; the owner wants to test the whole finger. The DIP needs nothing (no tendon passes it).

**Setup:** joint-zero STEP, OpenCASCADE prototype and occupancy maps (`research/experiments/2026-09-27-index-pinch/`).

**Result:**
- Key fact: an axis is a line, so several tendons can sit side by side along it and all be "on the axis". For the flex joints (axis across the finger) tendons go side by side; for the sideways joint (axis through the palm) they stack vertically.
- **MCP flex** (proximal segment tongue, x −11.04…−6.03): drum groove for the MCP's own loop at x ≈ −9.5 (tie hole next to it); **one** 1 mm pass-through slot at x −8.03…−7.04 carrying the PIP and DIP loops (4 strands), open to the palm and splitting into two channels further on. Change: same plug as the PIP, with a 1.4 mm hole on the axis (0.8 mm walls to the tongue side and the drum groove). Checks: plug never exposed outside the part (the only "unenclosed" plug edges are inside the existing internal funnel), tendon path clear from −5° to 100°.
- **MCP sideways** (index base): three palm channels (x ≈ −11.9, −8.5, −5.25; one per loop) feed a wide horizontal slot (5.4 × 3.6 mm) at the sideways axis. The side loops crossed it about 2 mm off the axis, so ±15° of sideways movement changed their length by up to 0.73 mm (~5° of joint movement). Change: two 1.2 mm thick blocks leaving a 1 mm vertical slit on the axis; length change through the slit is 0.00 mm. At full ±15° the side strands touch the *existing* slot edge ~5 mm before the joint (unchanged by this).
- Fusion script: `hardware/cad/fusion_scripts/TendraIndexPinch`. **Applied in Fusion (2026-09-27):** both changes report "slot filled: yes, tendon opening clear: yes", checked on the result in component coordinates.

![before/after](experiments/2026-09-27-index-pinch/index_pinch_before_after.png)

- **Re-checked in the live design via the Fusion MCP (2026-09-28):** all joints at 0, all six Tendra features healthy. PIP and MCP holes are clear along their whole length; the tendon channel past the PIP plug angles ~0.07 mm/mm toward the thumb side on its way to the DIP (normal). Sideways slit clear and centred.
- **Added `Tendra ABD slit fillet` (2026-09-28):** 0.4 mm fillet on the 4 vertical slit edges, so the side strands (bent up to ~35°) run over rounded edges instead of sharp corners. The slit is still 1 mm wide in the middle (on the axis), and 0.4 mm of flat remains on each 1.2 mm block. The PIP/MCP hole entrances were left sharp on purpose: rounding them moves the tendon's contact point off the axis. Deburr them by hand after printing.

**Conclusion / next:** Owner saves the design, prints the index base, proximal and middle segments, and tests every joint for coupling.

---

## 2026-09-27: Pinch point on the index PIP axis (design change, not yet printed)

**Goal:** Make the fingertip (DIP) tendon cross the PIP joint exactly on its axis, so its length no longer depends on the PIP angle (owner's choice over idler pulleys).

**Setup:** STEP export, OpenCASCADE prototype (`research/experiments/2026-09-27-pip-pinch/pinch.py`). Owner confirmed: only the DIP loop passes through the PIP. Each joint's own loop wraps a drum on the child segment and is tied through a hole in it.

**Result:**
- PIP drum (the middle segment's own tendon groove): groove bottom radius ≈ 6 mm, so the PIP lever arm is ≈ 6 mm. With the 10 mm spool, that's ≈ 389 half-steps per joint radian (1 : 0.6 of the current 1:1 default), still to be confirmed by calibration.
- The DIP tendon slot in the middle segment is 1 mm wide (next to the drum groove) and open toward the palm from the axis to ~6 mm distal.
- Change: fill that slot from the axis to 6.5 mm distal, leaving a 1.2 mm hole whose entrance is on the axis. The plug's palm side leans back 15°, so the tendon has clearance up to 100° of flexion.
- Checks: plug fully enclosed by the segment walls; tendon path clear from −5° to 100°; tendon length across the PIP constant (16.30 mm) versus up to 4.8 mm shorter at 90° with the open slot. Remaining error from the 0.6 mm hole radius is ≈ 1 mm at 90°.
- Fusion script `hardware/cad/fusion_scripts/TendraPipPinch` applies it. It first checks that the open design matches the STEP geometry, then asks before changing anything.

![before/after](experiments/2026-09-27-pip-pinch/pip_pinch_before_after.png)

- **Applied in Fusion (2026-09-27):** the script's own final check reported "NO", but TendraInspect confirmed the change. The middle segment grew by 28 mm³ (the plug), and a new r = 0.6 mm cylinder runs along the segment at the PIP axis height and at the predicted position across the segment (−7.50 mm, predicted −7.499). The check was wrong because adding features makes Fusion re-solve the joints, and the finger snapped from the STEP export's pose (index ~6° sideways, PIP ~5° bent) back to all joints at 0. The final check now tests the result in the component's own coordinates.
- **Re-exported STEP** (all joints at 0) and re-checked on the real geometry: the hole centre is 0.000 mm from the PIP axis height, the plug is present, and the DIP tendon path is clear from −5° to 100°.
- **Lesson:** export STEP with all joints at 0, and write CAD scripts against the live design geometry, not a posed export.

**Conclusion / next:** Owner prints the middle segment, and tests whether the fingertip still moves when only the PIP moves. Deburr the hole entrance so it doesn't cut the line. If it works, do the same at the MCP (PIP and DIP loops pass through) and on the thumb.

---

## 2026-09-27: Tendon routing review and joint coupling

**Goal:** Understand the tendon drive and find what limits smooth, clean motion.

**Setup:** Owner's description plus the URDF/MJCF. Each joint has one 28BYJ-48 in the forearm driving a closed pull-pull loop (flexor and extensor strands wound in opposite directions on one 10 mm radius spool). Each joint has 2 bearings (one per side) and a hole at the joint; tendons for more distal joints pass through that hole. Tendon: 0.4 mm fishing line.

**Result:**
- **Coupling confirmed by the owner:** moving one joint moves the more distal joints, because tendon path lengths change when the joints they cross move.
- Why a hole at the joint doesn't fully fix it: the path length is constant only if the tendon's contact point is *exactly* on the joint axis. Any offset e (hole beside the pin, hole wider than the tendon so it slides to one edge, strands at different spots) changes the length by ≈ e·θ, and the offset can flip sides with the bend direction. Over a ~95° range, each 1 mm of offset is ~1.7 mm of tendon, which is ~20° of unwanted motion at a joint with a 5 mm moment arm. If the palm-side and back-side strands of one loop have different offsets, loop tension also changes (slack or over-tight).
- **STEP analysis** (`hardware/cad/Hand assebly.step`, read with OpenCASCADE, index finger sliced along and across): joints are forks, with 4 mm axle bosses on the child running in 8 mm bearing seats (4×8 mm, e.g. MR84) in the parent's side lugs. There is no through-axle; the child's knuckle has a central slot, and the tendons run in ~1 × 4 mm channels at about axle height, with funnel-shaped openings at each joint (about ±4 mm toward palm and back). The channels narrow ~13–15 mm on either side of each axis. When a joint bends, the straight line between those narrow points wants to pass far to the palm side (≈10 mm from the axis at 90°, outside the finger), so the tendons press on the palm-side funnel walls. Estimated effect: pass-through strands shorten by ≈ 4 mm × θ (~6 mm at 90°), both strands of a loop in the same direction (slack), plus high friction at the funnel edges. Routing and anchor points are still to be confirmed with the owner.
- The firmware maps joint → motor independently (`kDefaultStepsPerRad` per joint), so it can't compensate.
- Other limits: 28BYJ-48 gearbox backlash (a few degrees at the output, ~0.5 mm of tendon on the spool); low force (~3 N of tendon force at 10 mm spool radius); nylon monofilament stretches and creeps under load.

**Conclusion / next:**
- Mechanical: put a small **idler pulley on each joint axis** (e.g. on the pin between the bearings) and wrap the pass-through strands around it, palm strand on one side, back strand on the other. Then each crossed joint changes the tendon by exactly ±ρ·θ: the loop length stays constant and the coupling becomes linear.
- Software: replace per-joint scaling with a **coupling matrix**, `motor = A · q`, measured per tendon.
- Tendon: consider braided PE (Dyneema) instead of nylon monofilament if it's mono; add a tension adjuster per loop.
- Sim: model the tendons as MuJoCo fixed tendons so the sim shows the same coupling.
- Consider a smaller spool (4–5 mm radius) once the joint moment arms are measured.

---

## 2026-09-27: Website design system v0.2

**Goal:** Define the visual identity of the project website before building real pages.

**Setup:** `website/assets/` (tokens, base, components CSS + `site.js`), reviewed on `website/styleguide/index.html`. Plain CSS + vanilla JS, no build step; site framework still undecided.

**Result:**
- v0.1 was "precision lab at night": dark, signal orange, mono labels, grain, blueprint grid. Owner feedback: **too robotic and nerdy**. They want light, clean, Apple-like, not orange, open to everyone.
- v0.2: light default (white / `#F5F5F7` bands), optional dark mode, one blue accent `#0066CC` for clickable things only, Inter for all text, pill buttons, 22 px rounded cards, soft shadows, plain-language copy.
- Contrast measured in the browser, all WCAG AA or better. Light: text 15.5:1, muted 4.7:1, blue text 5.1:1, button label 5.6:1. Dark: all at least 6.3:1.
- Checked: no horizontal scroll at 390 px, no console errors, reveals and counters show instantly with reduced motion or with JS off.

**Conclusion / next:** Audience matters more than tech flavour; the site should feel approachable. Next: owner reviews, then choose the site stack (e.g. Astro on GitHub Pages) and self-host fonts.

---

## 2026-09-27: Python `tendra` package + digital twin v1

**Goal:** Control the hand from the PC with one API for sim and real, and drive the real hand from the MuJoCo sliders.

**Setup:** `software/tendra` (Hand, SimHand, RealHand, FakeEsp32), `sim/twin.py`, pyserial 3.5.

**Result:**
- 20 tests pass, including consistency checks that Python, `config.h` and the MJCF agree on joint names and limits.
- Twin smoke test with the software ESP32: viewer, simulation, bridge and status run together and shut down cleanly. Targets are sent only when changed, at most 20 Hz.
- At rest the sim thumb sags ~1.2° under gravity (soft position actuator), while a stepper holds rigidly. Tune the actuator kp against the real hand.

**Conclusion / next:** Waiting for the finished hand: first power-on checklist, direction check, then scale calibration per joint (build a guided calibration tool).

---

## 2026-09-26: Firmware v0.1 (steppers)

**Goal:** Smooth, non-blocking control of 8 steppers from the PC, behind a HAL that the SCS0009 servos can later plug into.

**Setup:** PlatformIO, espressif32 / Arduino, `esp32-s3-devkitc-1` board with 16 MB flash, PSRAM off, USB CDC on boot.

**Result:**
- Builds cleanly: RAM 5.9%, flash 4.1%.
- Motion profile (trapezoidal, step-by-step: v² ± 2a per step) tested on the PC: ends exactly on target; a 4000-step move takes 5.506 s (ideal 5.5 s); reverses cleanly mid-move; worst step-to-step acceleration 1761 vs a 1600 steps/s² limit (discretisation).
- Half-step mode for smoothness. Coils are released after 1 s idle to respect the 5 V / 2 A supply. The coil phase is kept continuous across `Z` (re-zeroing), so there's no jump when re-energizing.
- **Not yet tested on hardware**; the hand isn't finished.

**Conclusion / next:** PC-side Python client + digital twin v1 (MuJoCo sliders drive the real hand). On hardware: first power-on checklist in `firmware/README.md`, direction check and scale calibration per joint.

---

## 2026-09-26: First MuJoCo model

**Goal:** Load the thumb + index in MuJoCo with correct physics and joint conventions.

**Setup:** `sim/convert.py` (raw Fusion export → `sim/models/tendra_hand.xml`), MuJoCo 3.14.0, Python 3.12.

**Result:**
- Mass recomputed from meshes at PLA density: **131 g** (the export said 821 g, steel).
- `Indexfix_1.stl` has inconsistent face orientation, so MuJoCo's exact inertia rejects it. Legacy inertia is used for it (it overestimates slightly: 3.9 g vs ~2.5 g expected).
- Joint directions checked with fingertip kinematics. The thumb base rotation in the export was already "opposition = positive"; thumb MCP and CMC flex were flipped.
- Collisions: palm ↔ finger bases overlap because the palm is welded to the world, so MuJoCo's parent-child filter doesn't apply. Palm ↔ thumb metacarpal "collides" at any thumb rotation of 10° or more, which is a convex-hull artifact. Both are excluded. Random-pose sampling shows the remaining contacts (thumb ↔ index) are physically plausible.
- Position actuators (kp 0.5 N·m/rad, ±0.05 N·m) reach targets within 0.8°; the simulation is stable.

**Conclusion / next:** Owner compares the viewer motions with the real hand. Then: firmware skeleton + stepper HAL (Phase 1). Later: convex decomposition of the palm, measured masses, system identification.

---

## 2026-09-26: Project setup and URDF inspection

**Goal:** Understand the Fusion 360 URDF export of the thumb + index prototype and set up the project.

**Setup:** `hardware/robot_description/fusion_export/`, exported with the fusion2urdf plugin.

**Result:**
- 9 links, 8 revolute joints = **8 DOF** (index: MCP abduction, MCP flex, PIP, DIP; thumb: base rotation, CMC flex, MCP, IP).
- Joint limits match the real hand closely (owner-confirmed).
- **Masses are wrong:** every part has a density of about 7850 kg/m³ (steel, Fusion's default material). Total 821 g; about 130 g expected for PLA at 100% infill.
- **Inertias are rounded** to 1e-6 kg·m². `Indexfix_1` and `Indextip_1` get zero principal moments, which is physically invalid.
- Placeholder effort/velocity limits (100). No damping or friction.
- Thumb joints 7 and 8 have opposite axis signs, so flexion is negative on one and positive on the other.
- `Indexfix_1.stl` has 2 open edges (minor).
- Pin review for the ESP32-S3-**N16R8**: GPIO 35–37 belong to the octal PSRAM (usable only with PSRAM disabled), and GPIO 0 is a boot strapping pin on motor 7.

**Conclusion / next:**
- Keep the raw export unedited, and generate a cleaned model with a script (fix mass/inertia, rename joints, flexion-positive).
- Decisions: name **Tendra Hand**; MuJoCo first (no NVIDIA GPU, so no local Isaac Sim); PlatformIO; Python 3.12 via uv; USB CDC serial link.
- Licensing: Apache-2.0 (code), CERN-OHL-S-2.0 (hardware), CC BY 4.0 (docs). NonCommercial and GPL options were considered and rejected. The business edge should come from brand, kits, quality and speed; share-alike on hardware keeps improvements open.
- Next: git + licenses, Python environment, URDF → MJCF converter, joint-slider viewer.
