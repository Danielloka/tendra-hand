# Better hardware for the Tendra Hand: "V2.1" (2026-11-14 … 11-15)

The owner asked for better hardware, mostly the 3D-printed parts: be creative, think from first
principles ("like Elon Musk"), go wide and narrow, think like an engineer, use an agent team.
Brief: [`BRIEF.md`](BRIEF.md). Lead's synthesis after the wide phase: [`SYNTHESIS.md`](SYNTHESIS.md).

**This is a study, not a decision.** Nothing in `hardware/`, `sim/`, `software/` or `CLAUDE.md` was
changed. Nothing was printed or tested on hardware; all numbers are hand calculations, CAD checks and
scripts.

## Summary and recommendation

**Print the bench finger after three CAD fixes, and run five bench tests first. V2.1 does not yet close
as a whole hand.**

The study fixes the main problem found in the V2 review: the tendon *drivetrain*. Its main ideas:

1. **No Bowden sheaths.** Bare tendons run straight between hard metal guides (polished Ø3 dowels,
   MR63/MR84 rollers) on rigid parts, and cross the wrist through a plate of rounded steel eyes at the
   wrist centre (report 09). The servos cancel the wrist's effect with one linear term each.
   Efficiency knuckle → spool: 0.74 → 0.92; loop stiffness about 12× higher (01).
2. **Strong knuckle:** a fixed full-length Ø4 steel axle in a "reverse fork" holds ~69 N sideways tip
   load, vs ~8 N for V2's stubs (03). Metal ring/sleeve in every loaded drum groove (PETG creeps).
3. **No bare knots:** ≥ 1 capstan wrap, then a clamp or a printed **fuse cup** that breaks before the
   servo gears do (03). **Spool cartridge** on its own bearings with a screw tensioner (02).
4. **Ready for touch and joint sensing:** snap-in silicone fingertip skin with a printable mould and a
   magnetic tactile pocket (05); wires through hollow pins on the axis; Hall angle sensors at MCP/PIP.
   Joint-angle sensing becomes *required*: with a strong drive the tendon itself is the soft part.
5. **Designed for HLS3606M loads**, with MCP slots that can later take STS3215 servos (the 45 N path, 01).

Fingertip pinch of the bench finger (07, corrected by the review 10):

| | V2 | V2.1 bench finger now | V2.1 after review fix #1 |
|---|---|---|---|
| SCS0009 (stall) | ~2–2.5 N | 5.6 N | 6.7 N |
| HLS3606M (stall) | ~8 N (unrealistic parts) | 11.6 N | 17.6 N |

What the review (10) found, ranked: the pip flex strand acts as an MCP extensor (force overstated);
07 and 09 used different servo-travel models (8 of 16 servos then exceed SCS0009's travel; fix: MCP
spool back to r 5); bare strands push 0.5–1.7 kN through the wrist, which the V2 cardan can't carry;
the knuckle module is too wide for the finger pitch; the fuse cup trips at SCS stall; the bench kit
(08) tests sheaths first, but the eye plate is now the deciding test.

## Next steps for the owner (from review 10, "pre-print order")

1. **Measure, don't print:** the SCS0009 range (0 → 1023 with a protractor), HLS3606M multi-turn (datasheet),
   SCS0009 gear strip torque.
2. Apply the three CAD fixes to the bench finger (pip flex over an MR63 roller on the palm side; MCP spool
   r 5 and wire slot moved; fuse-cup chamber sized to a measured crimp).
3. Print the **test coupon** [`out/04/tendra_coupon.stl`](out/04/) (fits for your printer).
4. Bench tests: line friction μ → crimp + fuse cup → **eye-plate wear** (architecture decider; fallback:
   steel-coil housing across the wrist only) → line creep.
5. Build the bench finger with pass/kill numbers. Only then: palm knuckle module, palm reorder block,
   wrist rework, thumb.

## Owner decisions

| Decision | Options |
|---|---|
| Tendon architecture | Bare strands + eye plate (recommended, if the eye test passes) / steel-coil housing at the wrist only / spring-tube sheaths |
| Load class | SCS0009 first (~5–7 N) / HLS3606M on 11 force joints (~12–18 N, ~€350) / STS3215 on MCPs (~45 N path, 12 V, +380 g) |
| Adopt for V1 now? | Silicone skin, Ø3 dowel guides, metal drum rings, spool cartridge, coupon: all independent of the architecture choice |

## Round 2 (2026-11-15): fixes and sensors

Owner: "Fix what didn't work. And also make a plan and add the sensor for seeing angle on each joint.
And pressure etc. find ways to wire and placements." Brief: [`BRIEF-2.md`](BRIEF-2.md).

**Fixed (reports 12, 13, 08):**
- **No crossed tendons:** the PIP flex tendon runs on the palm side over an MR63 roller. Pinch, counting
  every lever, pretension and bush friction: **5.9 N SCS0009 / 16.6 N HLS3606M** (round 1: 5.6 / 11.7).
- **One shared servo-travel table** (`out/12/travel_table.json`, wrist column in `out/13/`): all spools
  r 5; all 16 servos fit SCS0009 (worst 280° of 285°; the 300° range is still to be measured).
- **Wrist load path ("W13"):** bare strands push 0.74 kN (SCS) / 1.67 kN (HLS) through the wrist. The
  cardan becomes metal: CNC 7075 ring and yokes, Ø5 hardened pins in iglidur bushes, eye plate on ring
  ledges. About €85–125 in metal parts. The palm reorder plate failed; use per-strand liners instead.
- Fuse cup sized for a 2 × 2 mm crimp (trips 67 N SCS / 148 N HLS); knuckle axle stress halved; steel nips
  on the abduction axis; knuckle fits the 19.5–20 mm finger pitch (needs 18.9–20.0 mm).
- Bench kit reordered: the wrist-eye wear test decides the architecture; day-1 servo measurements.

**Sensors (reports 14, 15, 16):**
- **Joint angle on all 20 joints:** a diametric magnet in each joint pin end, a TMAG5273 on a Ø6.4 board
  in the existing counterbore (adds 0–0.85 mm width). ≈ 0.5° after an automatic 2-minute calibration.
  Needs a **non-magnetic bearing** beside each magnet (ceramic ZrO2 MR84 for SCS, iglidur/bronze bush for HLS).
- **Touch:** MLX90393 3-axis tip sensor under the silicone skin (0–12 N); 14 BMP581 pressure pads
  (phalanges + 4 palm zones). **Tendon force** from spool angle − joint angle (no new parts).
- **Wiring:** 4 × 38 AWG PTFE per finger through the hollow joint pins (twist only, ≤ 0.17 % strain),
  JST SH plug at the finger root; 6 wires through two hollow Ø5 wrist pins.
- **Electronics:** palm RP2040 hub, 7 I²C buses, 500 Hz angles / 200 Hz touch, 2 Mbaud UART to the ESP32-S3,
  112 mA. About €220 per hand.
- Bench finger with all sensors: `cad/16_finger.py`, `out/16/` (84 parts, all valid). 3D viewer source in
  `out/viewer/` (published as a private claude.ai page).

**Still open:** a verified load rating for the ZrO2 bearing; HLS margins (wrist ring 1.2); the hollow wrist
pins need cold-drawn tube or lathe work; the palm crossing liners; thumb and palm module; all bench tests.
Round 2 has not had an adversarial review yet.

| # | Report | Model |
|---|---|---|
| 12 | [finger-fixes](reports/12-finger-fixes.md) | Opus |
| 13 | [wrist-load](reports/13-wrist-load.md) | Opus |
| 14 | [sensor-parts](reports/14-sensor-parts.md) | Sonnet |
| 15 | [sensing-plan](reports/15-sensing-plan.md) | Opus |
| 16 | [finger-sensors](reports/16-finger-sensors.md) | Opus |

## Reports (round 1)

| # | Report | Model | What |
|---|---|---|---|
| 01 | [first-principles](reports/01-first-principles.md) | Opus | 22 architecture ideas; force budget; the path from 2 to 10 to 45 N |
| 02 | [drivetrain](reports/02-drivetrain.md) | Opus | Guides, line, anchors, housing, spool cartridge (CAD) |
| 03 | [joints-structure](reports/03-joints-structure.md) | Sonnet | 31 joint/structure ideas; reverse-fork MCP; drum ring + fuse cup (CAD) |
| 04 | [print-dfm](reports/04-print-dfm.md) | Sonnet | Materials, creep, orientation, palm grooves; **test coupon** (CAD) |
| 05 | [contact-sensing](reports/05-contact-sensing.md) | Sonnet | Pads, fingertip, skin + mould, sensor pockets (CAD) |
| 06 | [prior-art](reports/06-prior-art.md) | Sonnet | 42 tricks from real hands, papers and patents, with sources |
| 07 | [bench-finger](reports/07-bench-finger.md) | Opus | Complete V2.1 index finger on a bench base: 57 parts, BOM, checks |
| 08 | [bench-kit](reports/08-bench-kit.md) | Sonnet | 7 printable test rigs, 11 tests with beginner steps and pass/kill |
| 09 | [wrist-routing](reports/09-wrist-routing.md) | Opus | Bare-strand wrist: eye plate, palm/forearm plates, trough, idler block |
| 10 | [review](reports/10-review.md) | Opus | Adversarial review: 17 findings, pre-print order |

Scripts are in `cad/NN_*.py`, outputs (STL, STEP, GLB, renders, JSON) in `out/NN/`. Most run with
`uv run --no-project --with cadquery --with numpy --with trimesh python cad/NN_xxx.py` (each script's
header or report says which packages).

Licence: text CC BY 4.0, code Apache-2.0, CAD output CERN-OHL-S-2.0 (like `hardware/`).
