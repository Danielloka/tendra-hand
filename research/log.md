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
