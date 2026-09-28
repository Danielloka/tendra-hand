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
