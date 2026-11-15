# 08 · Bench test kit for the Tendra Hand (agent 08; round 2 revision, 2026-11-15)

Study only. Nothing outside this folder was changed. CAD: `cad/08_rigs.py` (CadQuery). Parts: `out/08/*.stl|*.step` (36 parts, all valid closed solids, no supports; `out/08/parts.json`). Pictures: `out/08/rig_R1..R9.png`. Helpers: `cad/08_capstan_mu.py` (μ, stretch, housing, η → pinch, servo hold), `cad/08_hall_error.py` (Hall angle error), `cad/08_wear_cycle.py` (servo cycling, now with an eye-plate rocker). Results sheet: `out/08/results_template.csv`.
Run the CAD: `uv run --no-project --with cadquery --with numpy --with trimesh --with matplotlib python cad/08_rigs.py`

## Changes in round 2 (review #10 and the pre-print order)

1. **New order.** Sheath compression (old T1) is demoted to **T12, "only if the eye fails"**. The architecture decider is now the **eye-plate wear test (T4, new rig R8)**, run after μ (T1), crimp/cup (T2, T3).
2. **Day-1 measurements added (D1–D4):** SCS0009 range with a protractor, hold torque against the 80 % back-off, gear strip torque on one sacrificial servo, line diameter and datasheet look-ups (HLS3606M multi-turn, MR84 and 623 static ratings). They decide review findings #2, #5 and #11.
3. **Agent 07's parts, not 03's.** Fuse cup: **round-2 geometry from `out/12/budget.txt` §5**: oval chamber 2.9 × 2.1 × 2.3 deep for a 2 × 2 mm crimp tube (ID 1.4, OD 2.0) on doubled PE#8, outer oval 4.3 × 3.5 (my wall choice), lip hole Ø0.8, lip t 0.6 / 0.8 / 1.0 / 1.2 printed flat on 12-cup coupons (07 expects t 0.6 flat = 67 N for SCS, t 0.8 on its side = 148 N for HLS). Drum ring: 14 × 0.5 brass (ID 13), flange lips 0.5 mm, puck core Ø13/Ø14. `out/12/` appeared while I finished, so the numbers above are 07's current ones; `CUP_CHAMBER` / `CUP_OUTER` at the top of the script change them in one line if 07 revises again.
4. **Crimp-in-cup slip test (T2)** added: doubled PE#8 in a 2 × 2 mm crimp tube, held in the same seat, with and without a cup.
5. **Hall sensor next to a steel bearing (T6, new rig R9):** a protractor jig, a magnet on a pin end, an MR84 that slides 1 to 8 mm away. It answers review #7 before the finger is printed.
6. **Every pass/kill number now names the budget number it protects** (column "Protects" below). T8 η passes at ≥ 0.93 (rig, bearings only) because the budget carries 0.90 with the PIP bush; T4 eye η ≥ 0.88 is 09's route-budget assumption; D2 hold is 80 % of the SCS0009 stall, not 100 %.
7. **Pre-stretch vs the 100 N rule fixed (section 5):** SCS pre-stretch at 90 N fits the 100 N limit; 07's 133 N (HLS class) is allowed only as a named special procedure.
8. **Loop creep** now has a per-loop pass (≤ 0.1 mm per week on a 533 mm loop at 5 N), replacing the "0.2 % in 24 h" that would have lost all pretension four times over (review #9).
9. **15° fleet-angle guide** (`guide_post_skew15`) added to the wear test (review #8).

## 1. The idea in one paragraph

One base plate (150 × 100 × 5 mm, M3 holes on a 10 mm grid, nut pocket under every hole) and **nine small rigs**. Every post has its holes on the grid, so a rig goes up in five minutes. Clamp the plate with its right end over the table edge; the line falls past the plate to a bucket. Do the cheap measurements first, then the three tests that decide the architecture (T1, T4, T5), then the Hall test, and only then print the bench finger.

## 2. Tests in running order, with what each number protects

"Protects" is the budget or design number that the pass line comes from. IDs match `results_template.csv`.

| # | Test | Rig | Buy (beyond common list) | Time | Pass | Kill / consequence | Protects | From |
|---|---|---|---|---|---|---|---|---|
| T0 | Calibrate the scales | none | 5 L jug | 15 min | within 2 % | correct all readings | every number | tendon §8.6 |
| D1 | **SCS0009 range** (0 and 1023 ticks, protractor) | none (R5 optional) | paper protractor | 30 min | ≥ 300° | < 285°: the travel table (07/09) cannot close on SCS; HLS only | travel budget 285 / 290° (review #2) | review #11a |
| D2 | **Servo hold vs back-off**: weights on the 60 mm lever | R5 | none | 1 h | holds 3.0 N for 10 s (0.18 N·m = 80 % of 0.226) with the load register ≤ 80 % and no back-off; stall ≥ 3.4 N | backs off below 2.6 N: re-budget at the measured torque | pinch at the 80 % cap | 03 test 4 |
| D3 | **Gear strip torque** (one sacrificial servo), backdrive torque | R5 | spare SCS0009 (~€10) | 1 h | strip ≥ 0.34 N·m (1.5 × stall) | below 0.28 N·m: fuse must trip at ≤ 0.8 × strip ÷ 5 mm | fuse protects gears (review #5, #11f) | review #11 |
| D4 | PE#8 diameter, datasheets (HLS multi-turn, MR84 C0, 623 C0) | none | calliper | 30 min | 0.47 ± 0.03 mm | other size: rerun drum numbers | line, HLS fallback, bearing s0 | review #11 |
| T1 | **Capstan μ** (steel, MR63 roller, brass ring, PETG) | R2, R6 pucks | Ø3 dowels, brass tube slice, MR63 | 2 h | steel / brass ≤ 0.12; roller ≤ 0.05 | steel > 0.2: rollers at every bend | η of every guide, 0.90 route | 01, tendon §8.1 |
| T2 | **Anchors incl. crimp slip**: doubled PE#8 in a 2 × 2 mm crimp tube, M2 clamp, wraps, knot | R3 | 2 × 2 mm aluminium crimp tubes + crimper, M2 screws, serrated washers | 2 h | crimp holds ≥ 1.5 × cup trip **and** ≥ 100 N, line breaks (not slips) | slips below the trip: cup is not the weak link; redo the crimp | fuse is the weak link | 02 B3, 06 #1, review #5 |
| T3 | **Fuse-cup trip** with 07's cup (4 thicknesses × 10) | R3 | none | 2 h | SCS: trip ≥ 54 N (07 expects 67 N at t 0.6 flat), CV < 15 %; HLS: ≥ 142 N (07: 148 N at t 0.8 on its side) | CV > 30 %: shear pin; trip < 54 N: thicker lip | trip ≥ 1.5 × the 80 %-capped strand (36 N) | 03 test 3, review #5 |
| T4 | **Eye-plate wear + friction + pretension swing** | **R8** | 2 SCS0009, stainless and brass sheet 1 mm, 0.65 / 0.84 drills | 1 day + 8 h run | 16 000 cycles with no broken filament; η ≥ 0.88 at 50°; swing 5 ± 3 N; used line holds 150 N | fray < 10 000 cycles or swing > 10 N: use the coil fallback → do T12, T13 | wrist route η 0.86, pretension (09 §8) | 09 §8 |
| T5 | **Line stretch, pre-stretch, loop creep** | R1 (+ a second plate) | optional Vectran | 1 h + 24 h + 1 week | strain ≤ 1.0 % at 45 N; settled < 0.1 mm after the pre-stretch; creep ≤ 0.1 mm per week on 533 mm at 5 N | creep above that: add a bottoming pretension spring | pretension 5 ± 2 N (review #9); angle error from stretch | 02 B6 |
| T6 | **Hall angle next to a steel bearing** | **R9** | AS5600 and TMAG5273 breakouts, Ø3×2 diametric magnets, MR84, brass and steel rod Ø4 | 3 h | error < 2° at the 07 geometry; find the smallest distance with < 1° | > 2° at 1 mm: move the magnet ≥ N mm proud on a brass stub | Hall error < 2° (review pre-print step 5) | review #7 |
| T7 | **Guide wear**: Ø3 dowel, MR63 roller, 15° skew dowel | R4 | MR63, dowels | 24–48 h | ≥ 50 000 (dowel, roller), ≥ 10 000 at 15° | skew fails: steel x-guide at the abd neck | redirect and abd guides | 02 B2, review #8 |
| T8 | **Joint lever η** (drum on two MR84) | R5 | MR84, Ø4 rod | 3 h | η ≥ 0.93; table: η 0.90 → 5.1 N, 0.80 → 4.5 N, 0.62 → 3.5 N pinch at the SCS 80 % cap | η < 0.80: bearings or drum wrong, fix before the finger | pinch ≥ 4.5 N pass, < 3.5 N kill; η 0.90 carried | 01 §3.2, review #12 |
| T9 | **Drum groove creep**: bare vs brass ring | R6 | brass tube 14 × 0.5 | 7 days | ring puck < 0.05 mm | bare > 0.2 mm confirms ring is required | drum ring at stall (11.3 MPa vs 12 creep) | 03 test 2 |
| T10 | **Pad friction** | R7 | silicone, TPU, glass | 1 h | silicone ≥ 3 × PETG | < 2×: gel or texture | fingertip pad | 05 test 1 |
| T11 | (merged into T2 and T3) | | | | | | | |
| T12 | **Sheath compression** (only if T4 fails) | R1 | PTFE 1×2, steel coil | 3 h | ≤ 0.6 mm at 140 mm, 45 N | fail: no sheaths at all | fallback only | 02 B1 |
| T13 | **PTFE keyhole groove** (only if T4 fails) | R2 | PTFE | 1 h | no pop-out at 2 N | pops out: lid plate | fallback only | 04 |

Not in the kit (needs a built finger or electronics): the bench-finger pass/kill list of review step 5 (pinch at SCS 80 % ≥ 4.5 N, abd ±15° × 10 k sweeps, loop pretension after a week), magnetic tactile calibration, wrist-cardan axial load (review #3).

## 3. The rigs (all on the same 150 × 100 plate)

All parts print in PETG, 0.2 mm layers, 4–5 walls, 40 % infill; no supports (horizontal holes are teardrops, grooves face up). Orientation per part: `parts.json` field `print`. About 230 g in total. **Print the plate and one post first** and check the Ø3.4 holes and nut pockets.

| Rig | Parts (STL names) | Tests | Picture |
|---|---|---|---|
| R1 line frame | `clamp_block`, 2 × `stop_block`, `gauge_post`, `flag_clip`, `roller_post`, `hang_drum` | T5, T12 | `rig_R1.png` |
| R2 capstan + bends | 2 × `roller_post`, `capstan_post`, `sleeve_petg`, `bend_form_90/180` | T1, T13 | `rig_R2.png` |
| R3 anchor, crimp, cup | `fuse_seat`, 4 × `cup_coupon_t*`, `roller_post`, `hang_drum` | T2, T3 | `rig_R3.png` |
| R4 guide wear | `servo_cradle`, `guide_post_tall`, `guide_post_skew15`, `eye_post`, `hang_drum` | T7 | `rig_R4.png` |
| R5 joint lever | `servo_cradle`, `joint_fork`, `lever_drum` | D2, D3, T8 | `rig_R5.png` |
| R6 drum creep | `clamp_block_2`, `puck_fork`, `puck_bare`, `puck_ring_seat`, `drum_ring`, 2 × `hang_drum` | T9 (also T1 ring) | `rig_R6.png` |
| R7 pad sled | `friction_sled` | T10 | `rig_R7.png` |
| **R8 eye-plate wear** | `servo_cradle_tall` (servo 1, strand), `servo_cradle` (servo 2, rocker), `eye_platform`, `guide_post_eye`, `hang_drum` | T4 | `rig_R8.png` |
| **R9 Hall jig** | `hall_fork`, `protractor_disc`, `pointer_post`, `hall_arm`, `hall_spacers` | T6 | `rig_R9.png` |

What the CAD assumes (check on the first print):
- SCS0009 body 23.3 × 12.1 × 25.25 mm, tab holes 28.5 mm apart, tab 16.8 mm above the bottom, shaft 5.7 mm from the end (from the V2 `params.py`; not measured). Measure and edit `SCS` in the script.
- **R8 eye platform:** the 4 M2 horn holes (14 × 8 mm) are a guess; drill to your horn. The eye centre is 9 mm above the disc and must sit at the line height of 43 mm (servo 1 cradle on 32 mm pillars); shim the platform with washers until the line runs level. The eye is a 12 × 12 × 1 mm stainless or brass coupon, drilled Ø0.65 or Ø0.84, countersunk to a lip of r 0.4–0.8 mm, epoxied in the pocket (the 09 design wants r 0.8 lip at Ø0.65 throat for HLS). The rig tests the strand through the eye with the plate rocked ±50° about the vertical axis through the eye: the same bend the strand sees at the wrist.
- **R9:** the pin protrudes 13.5 mm past the +y fork cheek, the magnet (Ø3 × 2, diametric) is glued flush in its end. The MR84 slides on the pin with a spacer of 3 / 5 / 7 / 10 mm: magnet-to-bearing-centre distance = 11 − spacer = 8 / 6 / 4 / 1 mm (1 mm = agent 07's present design, magnet inside the race). Board stand-offs: M2 washers; sensor 1.5–3 mm from the magnet.
- Fuse cup and ring: `CUP_*` and `RING_*` at the top of the script (07's round-2 numbers). `drum_ring` printed is only a fit check; use a metal slice.
- Edge posts hold the dowel 14 mm in front of the holes; bolt them on the last column (x = 140).

## 4. Shopping list (about €80–110 from scratch; you own the servo, ESP32, FE-URT-1, supply)

- Digital **luggage scale** 50 kg (€8–12), **kitchen scale** 5 kg 1 g (€10), **digital calliper** (€12–20), loupe ×10, phone + stack of books as tripod, G-clamp, 2 buckets 10 L, 5 L jug, carabiner.
- **PE#8 braid ~0.47 mm** (€10–15); optional Vectran 0.5 mm.
- Metal: Ø3 × 20 dowels, brass tube OD 4 / ID 3.1, **brass tube 14 × 0.5** (3 mm slices for the ring), Ø4 steel and brass rod (pin, 50 mm each), 1 mm stainless and brass sheet (eye coupons) with 0.65 / 0.84 mm drills, 2 × 2 mm aluminium crimp tubes + crimper, M5 × 60 bolts, M3 and M2 screws, nuts, flat and serrated washers, M3 eye bolts, MR63ZZ (10), MR84ZZ (6).
- Electronics: AS5600 breakout (€2), TMAG5273 breakout (€5–8), 4 × Ø3 × 2 diametric N45 magnets, 3 SCS0009 (2 for R8 plus one sacrificial for D3).
- PTFE 1 × 2 mm and a steel coil (only if T4 fails). Pad samples: silicone, TPU, PETG, a glass pane, hand cream.

## 5. Safety (read first)

- **Safety glasses for every test.** Weights hang **10–20 cm above a padded floor** (towel, cardboard), never over feet, never a long drop.
- **Hanging loads ≤ 100 N (10 kg).** Hand-pulled tests on the luggage scale ≤ 150 N, other hand on the plate, face away from the line.
- **Pre-stretch (fixes the conflict with 07's 133 N):** for SCS-class hands pre-stretch at **90 N for 24 h** (2 × the 45 N stall strand, 20 % of the 445 N line; inside the 100 N limit). 07's **133 N** is the HLS-class figure: allowed only as a named special procedure: ≤ 14 kg bucket, hung ≤ 10 cm above a padded floor, a slack **second safety line** from the load to the plate, glasses on, the area closed (no children or pets) during the 24 h, and the load added and removed while you watch. If those cannot be met, use 100 N.
- Servo tests: ≤ **25 N** line load on the 5 mm spool in long runs (0.125 N·m; the SCS0009 backs off after 2 s above 80 % load). Fingers out of the spool and line path while powered. Power from the bench supply.
- D3 destroys a servo: wear glasses, a cloth over the servo (plastic gear teeth can fly).
- Magnets: keep Ø3 magnets away from the ESP32 flash and from each other (pinch).

## 6. Procedures

Habits: write every reading in `out/08/results_template.csv`, three readings per row, photo of each set-up. N = kg × 9.81. Calculator: `python cad/08_capstan_mu.py -h`.

### T0. Calibrate (15 min)
Hang a 5 L jug (weigh it on the kitchen scale) on the luggage scale. If the readings differ by more than 2 %, correct every later number by that factor.

### D1. SCS0009 range (30 min, no printing)
1. Wire one servo to the FE-URT-1 (or the ESP32 firmware) with its own supply. Tape a toothpick on the horn; hold the body in `servo_cradle` or by hand. Draw or print a 360° protractor and put it under the horn.
2. Command ticks 0 and 1023 (Feetech FD software on the FE-URT-1, or `M 1 ±300` from the centre; ticks are 195.6 per radian, 300° = 1023 ticks).
3. Read both end positions on the protractor. Range = difference.
4. Pass ≥ 300°. 285–300°: the travel table in 07 and 09 has no margin: use one shared table (review #2). < 285°: it cannot close on SCS0009; plan HLS3606M and check its datasheet for a multi-turn mode (D4).

### D2. Servo hold and stall (R5, 1 h)
1. Servo in `servo_cradle`, 5 mm spool or any arm of known length: the quickest is a printed `lever_drum` on the horn pattern you drill, 60 mm arm. (Or tie a 60 mm lever from a stick.)
2. Torque on, servo holding the arm horizontal. Hang a small bag from the 50–60 mm point and add water in 0.3 N steps (kitchen scale). After each step wait 10 s and read the load with the `F` command.
3. Record (a) the weight at which the load reads 80 %, (b) whether the servo backs off after 2 s above 80 %, (c) the weight at which the arm yields (stall).
4. `python cad/08_capstan_mu.py hold --force <N> --arm 60`. Pass: holds 3.0 N for 10 s with load ≤ 80 % and no back-off, stall ≥ 3.4 N.
Use: the 80 % number (0.18 N·m → 36 N strand) is what every pinch figure assumes. If it backs off below 2.6 N, re-budget at the measured torque.

### D3. Gear strip torque (sacrificial servo, 1 h)
1. Same set-up, servo you can lose, glasses on, cloth over it.
2. Torque on. Add weight in 0.5 N steps at a 50 mm arm (hang point on the `lever_drum`), 5 s each, up to 10 N (0.5 N·m). Note where the arm slips or you hear a click.
3. Then torque off: add weight until the arm turns (backdrive torque).
4. Pass: no slip below 0.34 N·m (6.8 N at 50 mm). Use: the fuse trip (T3) is 56–68 N × 5 mm = 0.28–0.34 N·m. If the gears strip lower, the fuse does not protect them: set the trip at ≤ 0.8 × strip torque ÷ 5 mm, or accept that the fuse protects the finger only.

### D4. Paper checks (30 min)
Measure the PE#8 in three places. Read the HLS3606M datasheet for multi-turn, the NSK or Misumi PDFs for MR84 C0 (budget uses 139 N) and 623 C0 (220 N), and write them in the CSV.

### T1. Capstan μ (R2, 2 h)
1. Edge `roller_post` with a Ø3 dowel (and a variant with an MR63 on it). `capstan_post` with the Ø3 steel dowel (A), a brass tube sleeve (B), `sleeve_petg` (C). For the ring, wrap 90° around `puck_ring_seat` with the brass ring on it (R6 parts, lay the puck flat).
2. 5 kg jug on one end (49 N). Take the line half a turn (180°) round the post and pull the other end slowly with the luggage scale. Read the scale **while the jug moves steadily**. Three pulls per variant, pulling up and down, average.
3. `python cad/08_capstan_mu.py mu --hold-kg 5 --pull-kg <reading> --degrees 180`.
4. Pass: steel and brass ≤ 0.12; roller ≤ 0.05. It sets η = e^(−μθ) for every guide and drum in the budget.

### T2. Anchors and crimp slip (R3, 2 h)
1. `fuse_seat` at x = 20, edge roller at x = 140. Lower end: 3 turns round the `hang_drum` and half hitches (stronger than what you test).
2. Variants, 5 samples each: (a) **doubled PE#8 in a 2 × 2 mm aluminium crimp tube, crimped, dropped in the seat pocket without a cup** (the line exits through the 1.1 mm hole); (b) the same with a printed cup from T3 (cups last in order); (c) M2 clamp, flat and serrated washers (`clamp_block`); (d) 3 wraps + half hitches; (e) an overhand knot for reference.
3. Load by adding water to the bucket in 10 N steps, 10 s each, up to 100 N. For loads above 100 N pull by hand on the luggage scale to 150 N and stop.
4. Record the load and the place of failure (line breaking at the sleeve mouth or the clamp edge is a bad sign).
5. Pass: crimp holds ≥ 1.5 × the cup trip you will get in T3 (SCS: ≥ 100 N; 07 expects the crimp to hold 70–90 % of 445 N = 312 N, which is above what the kit may pull by hand: the SCS test is fully testable, the HLS one only to 150 N); clamp ≥ 100 N. If a crimp slips below the trip, practise the crimp (two squeezes, flat jaws) and test again before trusting the cup.

### T3. Fuse-cup trip (R3, 2 h)
1. Print the four `cup_coupon_t*` plates (12 cups each, flat, chamber up). Pick ten cups per thickness; the coupon uses 07's round-2 chamber (2.9 × 2.1 × 2.3).
2. Doubled line + crimp tube in the chamber, line out through the Ø0.8 lip hole, cup in `fuse_seat`, line to the roller and the hand-held luggage scale. Pull slowly (5 N per second) and note the peak when the crimp pulls through.
3. Compute mean, SD, CV. Pass (SCS): mean ≥ 54 N (1.5 × the 36 N capped strand), 07 predicts 67 N at t 0.6 flat, and CV < 15 % (at −25 % scatter 07 still gets 1.38 ×). HLS class: t 0.8 printed on its side (rotate the plate in the slicer) must trip ≥ 142 N (1.5 × 94 N): that is above the 150 N hand-pull limit with margin gone, so test it with the luggage scale up to 150 N only; a cup that has not tripped by 150 N is recorded as "> 150 N, plausible", not as a pass.
4. If no thickness gives CV < 15 %: use a calibrated 0.8 mm PETG shear pin (same maths, report 03).

### T4. Eye-plate wear, friction, pretension swing (R8, one day + 8 h run)
Goal: kill or confirm the bare-strand wrist (09 §8). Bolt R8: servo 1 (strand, tall cradle) at x = 30, servo 2 (rocker) at x = 90 with its shaft under the eye platform, guide post at x = 140. IDs 1 and 2.
1. **Coupons.** Cut four 12 × 12 mm coupons: stainless with a Ø0.65 hole, brass with Ø0.65, stainless with Ø0.84, brass Ø0.84. Break the hole edge with a countersink or a round file to a lip of about r 0.4–0.8 mm and polish with a pipe cleaner and toothpaste. Epoxy one into the platform pocket.
2. **Friction (30 min).** Rock the platform to 50° by hand (servo off, protractor disc from R9 under it). Hang a 2 kg weight on the line end, pull the servo end with the luggage scale at 5 / 20 / 45 N hold loads, 3 pulls each. η = hold ÷ pull. Pass η ≥ 0.88 at 50°. This is the eye η in the route budget (09: median route 0.86).
3. **Pretension swing.** Tie a loop: spool → eye → guide post → around a Ø14 puck on a bearing → back. Set 5 N with the luggage scale on the puck arm. Sweep the platform ±50° with `M 2` steps and read the scale at each 10°. Pass 5 ± 3 N. If it swings more than 10 N: kill.
4. **Wear run.** 20 N weight on the line (half the real load; wear is optimistic). Run `python cad/08_wear_cycle.py --port <port> --cycles 16000 --ticks 300 --rock-id 2 --rock-ticks 171`: servo 1 moves the strand ±7.7 mm, servo 2 rocks the plate ±50°. 16 000 cycles = the sliding distance of 10 000 cycles of a 12 mm finger stroke; about 8 h at 0.6 cycles/s. Stop each 1000 cycles, slide the line a few mm, inspect under the loupe for broken filaments. Run stainless and brass, throat 0.65 and 0.84.
5. **Strength after wear.** Hang a used piece and a fresh piece at 150 N for 10 s (hand pull on the luggage scale; never above 150 N). Both must hold.
6. Pass: no broken filament at 16 000 cycles on at least one coupon, η ≥ 0.88, swing 5 ± 3 N, used line holds 150 N. Kill: fraying under 10 000 cycles or swing > 10 N: the eye plate is dead, use the coil fallback (T12, T13) and redesign the wrist route.
Use: this is the architecture decider (review "pre-print order" step 4c).

### T5. Line stretch, pre-stretch, loop creep (R1, 1 h + 24 h + 1 week)
1. R1 without stop blocks. Clamp at x = 12, flag 100 mm from the clamp, gauge post right of it. Pre-load 5 kg for 1 min.
2. Gap at 0, 5, 15, 30, 45 N (30 s each) with the calliper. `python cad/08_capstan_mu.py stretch --force 45 --length 100 --delta <mm>`. Pass strain ≤ 1.0 % (angle error from stretch is what the budget carries: ~13° at SCS stall, report 02).
3. **Pre-stretch** a full 500 mm piece for the strands: 90 N for 24 h (section 5), reading at 1, 3, 24 h. Pass: the extension between hour 3 and 24 is < 0.1 mm.
4. **Loop creep** (review #9): print a second plate and bolt a `clamp_block` on it 350 mm behind plate 1 (or clamp a 500 mm length of board): the flag 500 mm from the clamp. Hold **5 N** (0.5 kg bottle) for a week, read the gap daily. Pass ≤ 0.1 mm per week on 533 mm (= 0.019 %). Above that the loop loses its pretension: add the bottoming pretension spring of review #9.

### T6. Hall angle next to a steel bearing (R9, 3 h)
1. Press the Ø3 × 2 diametric magnet flush into the end of the Ø4 pin (steel rod first, brass second). Pin through `hall_fork` (pin height 30 mm), protractor disc on the −y end (disc axis = pin), `pointer_post` at (60, 20). The magnet is at the +y end, 13.5 mm past the cheek.
2. Board (AS5600 first) on `hall_arm` with M2 washers; sensor 1.5–3 mm from the magnet. ESP32: read the 12-bit angle (register 0x0E/0x0F) with the usual Arduino `AS5600` library example; print degrees. Also log the status flags (magnet detected, too weak, too strong) and AGC.
3. Variants (angle sweep each): **no bearing (control)**; MR84 on the pin with the spacer 10 (distance 1 mm: agent 07's present design, magnet inside the race), spacer 7 (4 mm), 5 (6 mm), 3 (8 mm); then steel pin vs brass pin at 1 mm.
4. Sweep the protractor from 0 to 180° in 10° steps (hand-turn the disc), writing the reference and the sensor angle in a CSV (`variant,ref_deg,sensor_deg`), 2 sweeps up and down.
5. `python cad/08_hall_error.py out/08/hall_runs.csv`. The sensor zero is arbitrary, so the script removes the offset.
6. Pass: error < 2° (the finger's Hall budget) at the geometry you choose; control < 0.5°. Report the smallest distance with < 1°. If 1 mm fails: the magnet goes ≥ that distance proud of the race on a brass or plastic stub (review #7). Repeat the best variant with the TMAG5273 (a 3-axis chip tolerates more distortion).

### T7. Guide wear (R4, 24–48 h)
1. Servo in `servo_cradle`, 5 mm spool, line 3 turns on the spool and clamped (T2 method). Guide candidates, one per run: (A) Ø3 dowel in `guide_post_tall`, (B) MR63 on the dowel, (C) `guide_post_skew15` (15° fleet angle, report 10 #8). 90° wrap, 2 kg on the line.
2. `python cad/08_wear_cycle.py --port <port> --cycles 10000 --ticks 300` (≈ 5 h per 10 000). Every 1000 cycles check under the loupe.
3. Pass: A and B ≥ 50 000 cycles, C ≥ 10 000. 20 N is half the real load: this screens, it does not prove.
Use: if A fails, only rollers; if C fails, the abd neck needs a steel x-guide.

### T8. Joint lever η (R5, 3 h)
1. Two MR84 in `lever_drum`, Ø4 pin through `joint_fork`. Servo cradle at x = 25. Line from the spool to the tunnel hole in the drum (under the hub), 45° round the bottom.
2. Arm horizontal, pad on the kitchen scale (plate on books so the pad sits flat). Pull the line with the luggage scale (servo off) at 5 / 10 / 20 N and read the pad force (g × 0.00981).
3. η = (pad force × 60 mm) ÷ (line tension × 6 mm). `python cad/08_capstan_mu.py pinch --eta <η>`.
4. Pass η ≥ 0.93: the finger adds the PIP bush (−2.4 to −4.8 %, review #12), so the finger lands at 0.90 = the number carried in the budget and 5.1 N pinch at the SCS 80 % cap. Kill η < 0.80 (4.5 N). Grease the bearings first.

### T9. Drum groove creep (R6, 7 days)
`puck_bare` (core Ø14 printed groove) and `puck_ring_seat` + a real 14 × 0.5 brass ring (gap 0.3 mm) on the shared pin of `puck_fork`; lines from the double clamp over each puck, 90° down to 5 kg each (49 N). Measure the groove depth at 3 spots with a 0.5 mm drill shank and the calliper depth rod before and after 24 h, 3 days, 7 days. Pass: ring puck < 0.05 mm. Expect the bare groove to creep (17.9 MPa vs the ~12 MPa creep value).

### T10. Pad friction (R7, 1 h)
20 × 20 mm pad in the sled pocket, 500 g jar on top (4.9 N), pull with the luggage scale at ~10 mm/s over glass; μ = force ÷ 4.9. Five pulls, dry and with hand cream, for PETG, TPU, silicone. Pass silicone ≥ 3 × PETG on glass. Skin pull-out (report 05): vice + scale, > 10 N each direction.

### T12 and T13 (only if T4 fails)
- **T12 sheath compression:** R1 with `stop_block` A (x = 30 or 60) and B (x = 95), flag and gauge as in T5, bare run and PTFE runs at 35 and 90 mm sheath lengths at 5 / 15 / 30 / 45 N. `python cad/08_capstan_mu.py housing --load 45 --l1 35 --d1 <> --l2 90 --d2 <> --bare <>`. Pass ≤ 0.6 mm at 140 mm; below that the sheath is dead.
- **T13 keyhole groove:** 100 mm PTFE in `bend_form_180`, upside down and shaken, side push 2 N: must not pop out; the line friction must match T1.

## 7. Order of work

1. **Day 1 (measure, don't print):** T0, D1, D2, D3, D4. Print plate, R2, R3, R5 parts in the evening.
2. **Day 2:** T1, T2, T3 (07's cup). Print R8 and R9 parts.
3. **Day 3–4:** T4 (the long run overnight), T5 start. Day 4: T6.
4. **Day 5+:** T7 overnight, T8, T9 (a week), T10.
5. Then build the bench finger (review step 5). T12 and T13 only if T4 fails.

## 8. What would change the plan

| Result | Plan change |
|---|---|
| D1 < 285° | SCS0009 cannot close the travel table: HLS3606M (multi-turn check) or bigger spools |
| D2 back-off < 2.6 N | re-budget every pinch figure at the measured hold torque |
| D3 strip < 0.28 N·m | fuse protects the finger only; set trip ≤ 0.8 × strip |
| T1 steel > 0.2 | MR63 rollers at every bend |
| T2 crimp < 1.5 × trip | do not use the cup: crimp problem first |
| T3 CV > 30 % | calibrated shear pin instead of a cup lip |
| T4 fails | bare strands are dead at the wrist: coil fallback (T12, T13), wrist redesign |
| T5 creep > 0.1 mm/week | bottoming pretension spring at each spool |
| T6 > 2° at 1 mm | magnet ≥ the tested distance proud of the race, or a TMAG5273 |
| T7 skew fails | steel x-guide at the abd neck |
| T8 η < 0.80 | fix bearings or drum before the finger |
| T9 bare fine | ring may be skipped for SCS-class loads only |

## 9. Limits

- **Nothing was printed or run.** All geometry is checked in CAD only: valid solids, watertight STL, no supports. The servo dimensions, horn hole pattern and the 07 cup size are assumptions (section 3).
- The wear tests use 20 N, half the real load: optimistic.
- `08_wear_cycle.py` is syntax-checked only; it uses `M i n` from `firmware/README.md` and the 400 ticks/s limit (≈ 0.6 cycles/s). Its stroke (±7.7 mm) is shorter than a 12 mm finger stroke, so the eye run uses 16 000 cycles for the sliding distance of 10 000.
- The Hall jig reads a protractor by eye (about ±0.5°); the control run (no bearing) shows the floor.
- Eye coupons made at home do not have the exact lip of a machined eye: use them to find the failure mode and the order of magnitude.
