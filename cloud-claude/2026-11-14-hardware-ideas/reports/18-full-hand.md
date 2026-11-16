# 18 · Full V2.1 hand: palm, W13 wrist, forearm, assembly (package C)

Integrator (agent 09), round 3, 2026-11-16. Study only; nothing outside this folder was changed.

- Script: [`cad/18_assembly.py`](../cad/18_assembly.py), about 100 s:
  ```
  uv run --no-project --with cadquery --with numpy --with trimesh --with scipy --with rtree --with manifold3d \
      --with matplotlib --with pillow python cad/18_assembly.py
  ```
- Viewer export (`out/18/`, exactly per BRIEF-3):
  - `hand_v21.glb`: 496 nodes (node = part name), q = 0, mm, Z up, coloured by material.
  - `poses.json`: `parts` → body, and 9 poses, each with `q`, `T` (16 floats per part, row-major) and `lines`
    (32 strands from the forearm spools through the wrist plates, plus package A's tendons and sensor wires).
  - `checks.json`: validity, interference per pose, mass, part counts.
- Renders: `hand_open.png`, `hand_fist.png`, `hand_tip_pinch.png`, `hand_power_grasp.png`,
  `hand_wrist_flex_+40.png`.
- New part file: `palm_v21.stl`. `palm_v21_without_thumb_bay.step` is the same palm without the thumb bay,
  because the bay is cut as a mesh boolean (OCC fails on it).

## 1. What is in it

| Group | Parts | Source | Mass (g) |
|---|---|---|---|
| Fingers (4) + knuckle modules + palm liners + sensors | 361 | **package A**, `cad/18_fingers.py` (pose and lines API) | 135 |
| Thumb | 27 | **V2 placeholder**: package B (`cad/18_thumb.py`) did not exist when this ran. The script swaps it in automatically when `out/18/thumb/parts.json` appears | 103 |
| Palm | 11 | **new** `palm_v21` + hub + pads | 35 |
| Wrist | 23 | W13 (report 13) + V2 drive | 248 |
| Forearm | 74 | **new layout** | 599 |
| **Total** | **496** | | **1125** |

Palm details:
- `palm_v21` is a hollow PETG shell with a 2.5 mm wall. It respects the W13 heel keep-out.
- It carries:
  - **4 knuckle-module seats** (MCP z − 23.6) with liner openings under each module;
  - a **thumb bay**: a solid of revolution about cmc_rot that holds the mount, both 6704 bearings and the swept
    thumb base, +1 mm;
  - a clearance pocket for the W13 palm-plate seat;
  - 4 × M3 down to the yoke flange;
  - the **hub tray** (x ±18, y 8.5–13.5, z 28–58: RP2040-Zero on a 32 × 26 board, dorsal cover);
  - **4 pad cups** 8 × 10 × 4 for the BMP581s (thenar, hypothenar, centre, distal).
- The V2 palm and hypothenar silicone pads are kept as placeholders.

Wrist details:
- From `out/13`: ring, palm yoke (with the palm-plate seat), forearm yoke, Ø5 pins (2 hollow, for the sensor
  cable), 4 plain bushes, eye / palm / forearm plates, forearm-plate carrier.
- From V2: the 2 × STS3215 drive, gears, and the cranks and rods (rebuilt for every pose).

Forearm details:
- 16 SCS0009 + **16 report-02 spool cartridges** in report 13's slot map.
- The cartridges are 14.3 mm wide, so V2's stagger is kept:
  - level 2 (z −175) sits on the central rail (x ±3 → 17.3);
  - level 1 (z −141) sits on Ø6.4 standoff posts (x ±9.5 → 23.8), so the level-2 strands pass inside it.
  - **The shell grows from 95 to 105 mm** (x ±52.5).
- Rail and frame in PAHT-CF. V2's frame, flange, ESP32-S3 / FE-URT-1 / power board.
- **Deleted:** adjuster block, barrel adjusters, sheaths, comb.

## 2. Checks (`out/18/checks.json`)

Interference is checked between different rigid bodies: mesh boolean (manifold), bounding-box prefilter, threshold
1 mm³. Contacts inside one finger or the thumb are package A's and B's own checks. Designed seats (pins in bushes,
plates in seats, liners on the palm plate) are skipped.

| Pose | Result |
|---|---|
| open, relaxed, fist, tip_pinch, power_grasp, wrist_flex_+40, wrist_dev_−20 | **clear** (apart from A's liner overlaps, below) |
| spread (abd +12 / +4 / −6 / −12) | 1.2 mm³: ring knuckle module vs little yoke |
| thumb_to_little | **ring yoke vs little proximal 32.8 mm³**, ring proximal vs little proximal 21.2 mm³ (A's knuckles are wider than V2's), little tip skin vs thumb pad 6.6 mm³ (the pinch contact itself), small ones ≤ 2.8 mm³ |
| All poses | **23 liner-to-liner overlaps ≤ 6.6 mm³ inside the palm**: package A's PTFE liners cross each other (report 13 §5: they need 2 depths) |

Fixed on the way:
- The palm top was 1.6 mm into A's modules.
- V2's knuckle pad clashed with A's yokes (removed).
- The thumb bay (first a plain bore, then a vertex-only revolve) left 42–120 mm³.
- The thumb_to_little neighbours violated the abd pair rule. I abducted them 2° more than V2 did:
  ring 11°, middle 12°, index 13°.

Validity: all meshes are closed except 7 (W13 ring groove seam, 5 of A's liners, V2 thumb mount). Their mass uses
0.8 × the convex-hull volume, and they are skipped in the booleans.

## 3. Mass and part count

- **1125 g, 496 parts** in all:

  | Group | Mass (g) | Parts |
  |---|---|---|
  | Forearm | 599 | 74 |
  | Wrist | 248 | 23 |
  | Fingers | 135 | 361 |
  | Thumb | 103 | 27 |
  | Palm | 35 | 11 |

- Servo masses are datasheet values (SCS0009 13.2 g, STS3215 55 g); everything else is volume × density.
- V2 was 783 g. The rise comes from:
  - the metal wrist: 7075 ring, yokes and pins, about +80 g;
  - the wider forearm shell;
  - the cartridges;
  - the sensors.

## 4. Cost (rough, for one hand; prices from reports 01, 02, 13 and 15, verify)

| Item | € |
|---|---|
| 16 × SCS0009 + 2 × STS3215 | 175 |
| W13 metal kit: 3 CNC 7075 parts, 4 laser-cut stainless plates, pins, bushes | 110 |
| Sensing (report 15: 40 chips, hub, non-magnetic bearings, wires, plugs) | 220 |
| Spool cartridges (2 MR63, axle, M2 per servo), finger bearings, dowels, fasteners | 60 |
| Electronics (ESP32-S3, FE-URT-1, power), PE#8 line, PTFE liners, silicone | 75 |
| Filament (PETG, PAHT-CF, TPU) | 20 |
| **Total** | **≈ 660** (goal < 600; sensing is a third of it) |

## 5. Open items

1. **Thumb:** package B was missing at run time; the V2 thumb is the placeholder. Re-run when
   `out/18/thumb/parts.json` exists. Its strands still need the roller for their 68–96° turn at the palm plate.
2. **Package A:**
   - Liner crossings in the palm need two depths (23 overlaps).
   - The ring-little clearance at thumb-to-little needs 1–2 mm more knuckle clearance, or a firmware abd pair limit.
3. **Palm:**
   - The palm is a functional shell, not a styled one.
   - The pad layout and the thenar side need the thumb's final envelope.
   - The STEP has no thumb bay (the mesh does).
4. **Forearm:**
   - It is 10 mm wider than V2 because of the cartridge width (synthesis open item). The V2 frame was widened
     but not re-optimised.
   - The forearm fan's spacer comb (report 09/13) is not modelled.
5. **Viewer:**
   - Cranks and rods are exported in the open pose only; their `T` is identity in the other poses (their shape
     changes per pose).
   - Strands are polylines in `lines`.
