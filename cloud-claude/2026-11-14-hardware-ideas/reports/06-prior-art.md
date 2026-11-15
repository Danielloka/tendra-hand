# 06 Prior-art scout: mechanical tricks from real hands (2026-11-14)

Agent 06. Goal: find concrete mechanical tricks from real hands (papers, patents, shops, hobby hardware) that Tendra can copy in printed parts and the tendon drivetrain. Focus on parts and how they are made.

Confidence tags:
- **V** = I read the source page in this session and the number or fact is in it.
- **S** = only seen in a search-result summary (not opened in full).
- **M** = from my memory or general engineering knowledge. Check before you rely on it.

What the repo already covers (so I did not repeat it as news): braided UHMWPE line, capstan maths, steel dowels at bends, split spool with a friction clamp, ORCA ratchet spool and pop-out joints (as ideas), DIP coupling, antagonistic pairs, on-axis crossings (`research/references/hands/`). Where a trick is already in the repo I say so and only add new numbers.

Limits of this scout: many hands (Wuji, Sharpa, PSYONIC, Clone, Sanctuary, Inspire, Dexmate, Mimic, Allegro, Shadow internals) publish no part-level drawings. Wuji and Sharpa are direct-drive (motors in the finger), so they have little to teach a forearm-tendon hand. I did not find any real teardown of PSYONIC, Inspire or Clone, so I list nothing invented about them. The Tesla patents were read only through press coverage (S), not the patent text.

---

## 1. Top 10 for Tendra (ranked)

Ranked by: how much it fixes the problems in `2026-11-09-v2-hand-cad/reports/05-review.md` (sheath creep, 45-75 % efficiency, knots, frayed eyelets, no sheath stops, no strand replacement, wrist gear too small), and how easy it is to print.

### 1. Clamp-bolt tendon anchor instead of a knot (Ruka-v2 / "Model-W", 2026) [V]
- **What:** knots were replaced by an M2 bolt and two washers. The tendon wraps once around the bolt shank and is clamped between the two washers when the bolt is tightened. The authors say knots make tensioning hard.
- **Who:** SoftHand Model-W (arXiv 2604.00738) [V]. The same paper proposes turnbuckle terminations for fine trim [V].
- **Why it works:** the clamp loads the line over a flat area, not a sharp knot curve. A knot in braid keeps only 40-60 % of strength (sailing data, see #9). A bolt clamp is adjustable and you can undo it to swap a strand. This answers the "knots take ~90 % of the pull" and "strands cannot be replaced" findings.
- **Number:** knot efficiency 40-60 % [V, US Sailing note + Dave Richards via animatedknots]. A clamp has no published efficiency, so test it (see test below).
- **Tendra adaptation:** on each drum put a printed pocket with an M2 captive nut and an M2 x 5 pan-head screw with a small brass or steel washer on each side (washer OD 4.5-5 mm, 0.3 mm thick). Run the line 1.5 turns round the drum first (capstan keeps the clamp load low), then under the washers. Print the pocket with the screw axis along the drum axis, so the screw loads the print in compression, not by layer peeling. Strand change = loosen one screw.
- **Test:** pull-test 0.36 mm braid in a bolt clamp to failure on a luggage scale / hanging weights. Pass = holds > 100 N (~ 45 % of the 222 N break load, enough above the 38-88 N working loads). Try serrated vs flat washers and 0.2 N·m vs 0.1 N·m screw torque.
- **Source:** https://arxiv.org/html/2604.00738v1

### 2. Spring-reinforced PTFE sheath, or a metal spring tube alone (antagonistic-Bowden hand, MM-Hand) [V]
- **What:** a close-wound spring (2.0 mm ID x 3.2 mm OD) over a PTFE liner (1 mm ID x 2 mm OD), so the sheath cannot collapse or shorten under tension. MM-Hand tested four sheaths with a 1 mm Dyneema line and chose the **metal spring tube** (lowest friction, because the corrugations touch the line at discrete points). Plain PTFE was relatively high friction. A **Bambu feeder tube** (PTFE with a spring jacket) was better than plain PTFE. Shimano SP41 shift housing had low kinetic but **high static** friction, which they called undesirable for control.
- **Who:** arXiv 2512.24657 (spring + PTFE) [V]; arXiv 2604.17245 (MM-Hand, friction test) [V].
- **Why:** a coil spring carries compression along its length as a column of turns and does not creep like PTFE alone. This attacks the "8 mm sheath compression at 45 N" problem directly.
- **Number:** MM-Hand fingertip force 33 N peak with a 0.1 m sheath vs 25 N with 1 m sheath [V] (so ~25 % loss over a long, slightly curved sheath).
- **Tendra adaptation:** the V2 channels are 2.2 mm bores. A 3.2 mm spring does not fit. Options: (a) move to a 3.6 mm channel for the long run through the forearm only, where the sheath is a separate part anyway; (b) use a smaller guitar-string-type close-wound spring (needs a ~1.4/2.0 mm part, I did not find a catalogue part, so check suppliers); (c) in the printed palm keep PTFE but add a brass hypodermic tube (see #8) at the ends. Test one 300 mm strand of each type on a bench.
- **Source:** https://arxiv.org/html/2512.24657v1 , https://arxiv.org/html/2604.17245

### 3. Use the sheath end stop as a load cell (conduit reaction-force sensing) [V]
- **What:** the sheath pushes back on its end support with a force that matches the tendon tension, so a sensor at the sheath end reads the tension without cutting the line. Patent US8056423: a cylindrical strain-gauge element over a force member, tendon passes through the middle.
- **Who:** US8056423 [V]. Shadow's motor unit measures torque another way: the tendons turn 90 deg round a bar held by load cells at both ends, torque = difference of the two readings [V].
- **Why:** the SCS0009 reports PWM-duty "load", not current, so Tendra has no honest force sense per strand. The sheath end stop (which the review says is missing anyway) can be that sensor. It solves two problems with one part.
- **Number:** none in the page. Gauge on a 1-2 mm thin printed or machined flexure gives µstrain-level signals; load cell + HX711 costs ~€2-5 each (M). 32 strands is a lot of channels; start with the 4-8 flexors of the index and thumb.
- **Tendra adaptation:** a printed (PETG-CF or PA-CF) sheath-end seat with two thin beams; a cheap strain gauge or a hall sensor + magnet on the beam deflection (0.05-0.1 mm per 40 N). Pretension then becomes a number, not a pluck test.
- **Test:** hang known masses on a strand and plot reading vs load. Pass = repeatable to ±0.5 N.
- **Source:** https://patents.google.com/patent/US8056423 ; https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/2.2.0/user_guide/md_motor_unit.html

### 4. Three-piece keyed bobbin with an octagonal key; flexor barrel nearest the servo [V]
- **What:** each bobbin = lower barrel on the motor shaft + partition wall + upper barrel for one antagonistic pair. The parts interlock through an octagonal keyway; turning one part against the other changes the cable length of the pair, setting preload while assembling. The flexor sits on the lower barrel, which lowers the bending moment on the motor shaft because flexion has the higher load.
- **Who:** arXiv 2512.24657 [V]. Shadow's split spool is the same idea with a bolt (loosen, rotate the top half, tighten) [V].
- **Why:** a keyed split gives discrete preload steps (octagon = 45 deg = at r 2.5 mm, 1.96 mm of line per step, too coarse alone; combine with a clamp for fine trim). The partition also stops the two strands from tangling at the spool, which the Shadow docs warn about.
- **Tendra adaptation:** spool r = 2.5 mm is tiny. Use a 16-tooth or 24-tooth serration between the two halves (24 teeth = 15 deg = 0.65 mm line per click) plus the horn screw as clamp. Put the flexor half closest to the servo case. Flanges 0.6-0.8 mm.
- **Source:** https://arxiv.org/html/2512.24657v1 ; https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/2.2.0/user_guide/md_motor_unit.html

### 5. Fiberglass-reinforced GT2 belt wrist drive (ORCA) [V for belt; no sizes given]
- **What:** ORCA drives the wrist with a GT2 timing belt with fiberglass reinforcement; flexion/extension 60 deg, durability tested at 40 deg [V].
- **Why:** a printed gear trains at wrist scale is the weak point in V2 (3-5x too small). A belt spreads the load over many teeth, has no backlash, and is cheap. A 6 mm wide GT2 belt is rated well above what a 25 mm pulley sees (M: typical 6 mm GT2 working load ~ 100-200 N, check the maker).
- **Tendra adaptation:** a 20-30 tooth pulley (12-19 mm pitch dia) on the wrist axis, a small pulley on the servo, an idler or slot for tension. Print the big pulley in PA-CF, 0.2 mm layer, teeth profile from a GT2 template.
- **Source:** https://arxiv.org/html/2504.04259v2

### 6. Quick-connect tendon box in the palm + phalanges split vertically for printing (MM-Hand) [V]
- **What:** hand-side tendons end in an in-palm quick-connector box on the bottom cover; detachable connectors split the hand-side and arm-side segments, so a finger or sheath can be swapped without re-routing. Phalanges are split in 2-3 pieces and bolted together; the split gives a better print orientation for smooth internal tendon channels and makes installing tendons and sensor wires easy.
- **Who:** arXiv 2604.17245 [V]. CRAFT also swaps a finger by pulling dowel pins, no rewiring [S/V].
- **Why:** answers "strands cannot be replaced without teardown". Split halves also let you print the channel as an open groove (no bridging, no support inside a 2.2 mm bore), then close it with the cover.
- **Tendra adaptation:** every phalanx = two clamshell halves with M2 screws, the channel is a half-round groove 2.2 mm wide on each half, sheath ends pinned in pockets. For the quick connector, a plug with two brass tubes and an M2 clamp per strand.
- **Source:** https://arxiv.org/html/2604.17245 ; https://craft-hand.github.io (via arXiv 2603.12120)

### 7. Pop-out joints and snap-fit knuckle (ORCA, CRAFT, PSYONIC) [V for ORCA/CRAFT; S for PSYONIC]
- **What:** ORCA: bearings sit in circular arc-shaped grooves and dislocate under strong radial or axial overload instead of breaking; loosening the tendons also lets joints pop out for quick part swaps [V]. CRAFT: a cylindrical snap-fit MCP "pops out under excessive force" [V]. PSYONIC: fingers dislocate and snap back [S].
- **Why:** at 45 N tendon pull and 8 more cycles per day the weakest printed ear breaks. A pop-out sets the maximum load.
- **Number:** none published (no pop-out force). Tune by printing 3 snap depths (0.3 / 0.5 / 0.7 mm) and measuring with a luggage scale.
- **Tendra adaptation:** make the pin ears of the proximal phalanx a C-clip (open on the palm side), hold the pin with a 0.4-0.6 mm detent. Re-fit by hand, no tool. Only on MCP and CMC first.
- **Source:** https://arxiv.org/html/2504.04259v2 ; https://arxiv.org/html/2603.12120v1

### 8. Brass tube sheath ends + tangential spool exit (MM-Hand, bike-cable practice) [V + M]
- **What:** (a) each tendon leaves the spool tangentially and enters the sheath directly, so there is no bend inside the hub [V, MM-Hand]. (b) Sheath ends get a positive stop: bicycle brake housing uses a metal ferrule on every housing end that seats in the frame stop (M). Tendra can do this with a 2 mm OD x 8 mm brass or stainless hypodermic tube pushed on the PTFE, with the tube end flanged or seated in a stepped pocket (M).
- **Why:** the brass ferrule stops the PTFE from creeping out of its bore and gives the load a hard path. It also gives a polished mouth, which fixes the frayed 0.43 mm eyelet rims.
- **Number:** none. Cost ~€0.10-0.30 each (M).
- **Tendra adaptation:** step the bore: 1.2 mm for the line, 2.2 mm for the tube, 3.0 mm x 1.5 mm deep pocket for the flange. Glue with a drop of CA or epoxy.
- **Source:** https://arxiv.org/html/2604.17245 (a); bicycle practice (b) is memory.

### 9. Splice (or crimp) instead of a knot for the strand [V for numbers, adaptation untested]
- **What:** in sailing HMPE (Dyneema), a knot keeps 40-50 % of line strength, a splice 90-100 % (US Sailing). Dave Richards measured knots at ~60-70 % retained. A simple overhand knot kept 35 % in one test. A Brummel lock alone does not add much; the bury length matters. A tapered bury tail adds ~20 % (Practical Sailor). One real test of a 12-strand loop broke 12 % below the predicted splice load.
- **Tendra adaptation:** 0.36 mm 8-carrier braid is too thin and too stiff to splice by hand without a fid (M); an alternative is a pre-made loop: bury 25-30 mm of tail in the line itself (a "locked eye"). Even if you only get 70 %, it beats a Palomar knot at 50-60 % (M). A cheaper path is trick #1 (clamp) plus 1.5 wraps.
- **Source:** https://www.ussailing.org/wp-content/uploads/2018/01/Dyneema-Spectra-Lifeline-Revision-Jan-2014.pdf ; https://www.practical-sailor.com/sails-rigging-deckgear/working-with-high-tech-ropes/ ; https://www.animatedknots.com/hundred-percent-strength

### 10. Choose drum radii so flexor and extensor lengths cancel (computed, not guessed) [V]
- **What:** the Bowden-hand authors optimise the rolling radius so flexor and extensor length changes cancel, |Δcf + Δce| ≈ 0 across the whole range; the residual is about 0.03 mm. Cross-pairing: flexor-ulnar with extensor-radial, flexor-radial with extensor-ulnar, because only the radial/ulnar cables change length together [V].
- **Who:** arXiv 2512.24657 [V]. (Tendra's own `tendon_router.py` already computes lengths, and `tendon-mechanics.md` has a 0.35 mm mismatch example for 0.2 mm radius difference.)
- **Why:** with stiff braid, 0.1 mm of length error is 2-3 N of tension (repo note). A 0.03 mm residual means the loop never goes slack or over-tight across the range, so you can use a very low pretension (less friction, less creep).
- **Tendra adaptation:** add a check in `tendon_router.py` that prints the loop mismatch per joint and tunes the groove radius (printing it to 0.05 mm resolution) so |Δ| < 0.1 mm across the joint range.
- **Source:** https://arxiv.org/html/2512.24657v1

---

## 2. Full table (36 tricks)

Tag: V / S / M as above. "Repo" = already described in `research/references/hands/`.

| # | Trick | Who | Why / number | Tendra adaptation | Conf. | Source |
|---|---|---|---|---|---|---|
| 1 | M2 bolt + two washers clamp as tendon end | Model-W | no knot, adjustable | pocket + captive nut on drum | V | arxiv.org/html/2604.00738v1 |
| 2 | Coil-spring-reinforced PTFE liner (1x2 mm PTFE in 2x3.2 spring) | Bowden hand | no sheath collapse | spring or hypo tube for long run | V | arxiv.org/html/2512.24657v1 |
| 3 | Metal spring tube lowest friction of 4 sheaths | MM-Hand | corrugation = point contact; 25 N @1 m sheath vs 33 N @0.1 m | test a spring-tube strand | V | arxiv.org/html/2604.17245 |
| 4 | Bambu PTFE feeder tube lower friction than plain PTFE | MM-Hand | better extrusion | buy feeder tube, 2 mm OD / 1 mm ID | V | same |
| 5 | Shimano SP41 housing: low kinetic, high static friction (avoid) | MM-Hand | stick-slip bad for control | do not use | V | same |
| 6 | Sheath end as load cell (conduit reaction force) | US8056423 | no tendon cut, compact | flexure + strain gauge at sheath seat | V | patents.google.com/patent/US8056423 |
| 7 | Tendon turned 90 deg around a bar with load cells each end; torque = difference | Shadow | torque sense | too bulky; use #6 | V | shadow docs motor unit |
| 8 | 3-part keyed bobbin, octagon key sets preload; flexor barrel nearest shaft | Bowden hand | less shaft bending; preload at assembly | serrated split spool | V | arxiv 2512.24657 |
| 9 | Split spool, loosen bolt, rotate top, retighten | Shadow | continuous trim | repo has it | V (repo) | shadow docs |
| 10 | Ratchet spool: turns one way, locks other; retension in seconds | ORCA, CRAFT | no tools | repo says do it; adds mass to servo horn | V | arxiv 2504.04259v2 |
| 11 | Small tensioner so loose tendon winds neatly (does not fix backlash) | Shadow | stops tangles | light torsion spring on a spool flange | V | shadow docs |
| 12 | Spring-loaded tensioner in servo sleeve, continuous adjust | BiDexHand | slack and hysteresis | printed spring-slide at wrist plate | V | alphaxiv.org/abs/2504.14712v2 |
| 13 | Software pretension: motor reels in when angle shows slack; release speed capped at spring recovery rate; current spike stops system | MM-Hand | no hardware tensioner (pulley tensioner and concentric dual spool tried and rejected) | firmware with HLS3606M current | V | arxiv 2604.17245 |
| 14 | Tension to taut in a jig at boot, motor position = zero | RUKA | no sensors | add a jig print | V | ar5iv 2308.02453 (RUKA zero is from the Faive page, see note) |
| 15 | Auto-calibrate by driving each joint to stalls; ρ = Δm / Δθ | ORCA | no external sensors | firmware, uses servo load | V | arxiv 2504.04259v2 |
| 16 | Tendons touch only smooth metal pins/rods and PTFE | ORCA | no PLA wear | repo has it | V | same |
| 17 | Tendon through joint centre: independent joints, auto-calibration | ORCA | linear | repo (on-axis crossing) | V | same |
| 18 | Pop-out bearing-in-arc-groove joints | ORCA | overload dislocates | C-clip pin ears | V | same |
| 19 | Cylindrical snap-fit MCP pops out | CRAFT | overload safety | MCP detent | V | arxiv 2603.12120v1 |
| 20 | TPU PIP/DIP, rigid PLA links; single tendon + bidirectional TPU linkage for equal PIP/DIP | CRAFT | impact tolerance; $<$600, 800 g; pull-out 15.29 N vs LEAP 8.67 N | TPU fingertip joint as the "overload fuse" | V | same |
| 21 | Elastic band returns PIP/DIP to neutral when tendon is slack | CRAFT | passive open | extensor spring on weak joints | V | same |
| 22 | Internal springs, hand defaults to open; rolling-contact joints; 3 cables per finger | Tesla patent | press report only | note, Tendra keeps antagonistic loops | S | teslanorth.com article |
| 23 | Cables in lateral stack in the forearm become a vertical stack at the wrist centre | Tesla patent | cables barely change length when wrist moves | repo has `wrist_centre` | S | teslanorth.com ; driveteslacanada.ca |
| 24 | Wrist 2 DOF by 2 motors: both together = tilt, opposed = rotate | Tesla patent | halves motors | already studied in `2026-11-04-v1-wrist`; mention for 2-servo wrist | S | teslanorth.com |
| 25 | Fiberglass-reinforced GT2 belt wrist | ORCA | no printed gears | GT2 + 25 mm pulley | V | arxiv 2504.04259v2 |
| 26 | Wrist: fixed-end PTFE sheaths through the carpal tunnel; 8 mm axle + 16 mm OD bearings; constant-radius cams; U-channel keeps tendon on cam | Model-W | sheath length does not change with wrist | repo has fixed ends; add U-channels | V | arxiv 2604.00738v1 |
| 27 | Three 5 mm U-groove bearings per side in a phalanx, equilateral triangle, M2x8 | Model-W | ball-bearing pulley | on the PIP of thumb only | V | same |
| 28 | Quick-connect tendon box in palm | MM-Hand | swap finger without re-routing | plug with brass tubes | V | arxiv 2604.17245 |
| 29 | Phalanges split vertically into 2-3 bolted pieces for print orientation | MM-Hand | smooth channels | clamshell | V | same |
| 30 | Finger module swaps by pulling dowel pins | CRAFT, Aero | maintenance | 2 steel dowels per finger | V | arxiv 2603.12120v1 ; github TetherIA |
| 31 | Rolling-contact joints in PLA, no bearings; Spectra thread holds them; half-angle 50 deg; thumb 21.9 N, index-little 18.0 N, 25 kg dumbbell | Bowden hand | much stronger than Tendra's 3.4 N | consider on one test finger; Tesla reportedly dropped rolling joints (repo note, Musk April 2026) | V | arxiv 2512.24657v1 |
| 32 | Rubber tape on PLA finger/palm surfaces (PLA slippery) | Bowden hand | grip | TPU 95A pad on each phalanx pad | V | same |
| 33 | Crossed four-bar (anti-parallelogram) DIP link, sigmoid-tuned; thumb IP-MCP anti-parallelogram | BiDexHand | no DIP servo | Tendra already has a bar; try anti-parallelogram if bar collides | V | alphaxiv 2504.14712v2 |
| 34 | N-configuration endless loop route per joint, PTFE guides | BiDexHand | simple, 2.14 N tip, 4.54 kg lift | repo has loops | V | same |
| 35 | Spiral spool + series-elastic spring changes speed and force with posture | MDPI Biomimetics 10(3), 155 | grip strength up vs rigid spool (search summary; page not opened, 403) | non-round spool half: bigger radius when open (fast), smaller closed (strong); could be a printed cam | S | mdpi.com/2313-7673/10/3/155 |
| 36 | Friction switch pulley (silicone + steel pins): low friction at light load, high at heavy | arXiv 1401.5232 | holds grasp without motor current | idea only; not tested on small scale | S | arxiv.org/pdf/1401.5232 |
| 37 | Dyneema splice (90-100 %) beats knots (40-60 %) | Sailing | see #9 | pre-made loop | V (numbers) | US Sailing note |
| 38 | Brass/steel hypodermic tube as sheath end/ferrule | Bike housing ferrule practice | positive stop | step bore | M | - |
| 39 | Barrel adjuster at sheath end (inline trim) | Bicycles | fine adjust; repo says M5-M7 thread is too big | print an M4 barrel in PETG-CF at the wrist plate only (32 would be big) | M (repo notes size) | repo tendon-mechanics |
| 40 | Servo-saver: friction slip in the horn so a hard knock slips the spool, not the gear | RC cars/planes | protects gears (the SCS0009 gears are metal, plastic case) | clutch disc with PTFE washer between spool and horn | M | - |
| 41 | Print-in-place / vertical print orientation, 0.2 mm layer, PLA, tree supports on plate only | Aero Hand Open | fine finish on mating faces; 389 g, $314 BOM | match orientation rule | V | github.com/TetherIA/aero-hand-open |
| 42 | LEAP v2 Adv: soft printed skin over bones; PIP-DIP coupling by one tendon per finger; powered palm | CMU | details not found (only summary) | none | S | search summaries |

(Items 14: the "jig at boot, zero = taut" is from RUKA via the Faive page text I fetched; I fetched ar5iv 2308.02453, which was the Faive hand and returned the jig statement, so the attribution to RUKA vs Faive is **uncertain**. Treat as "a published research hand does this".)

### Not found / not usable
- Wuji and Sharpa put motors in the finger (direct drive, four-bar for Wuji [S]). No tendon tricks. Wuji thermal limit ~30-40 min [S].
- Inspire RH56: linkage + linear actuators, 10 N per finger [from repo]. No part-level source found.
- Clone (hydraulic muscles), Sanctuary, Dexmate, Mimic: no mechanical detail found by search.
- Halodi/1X patent WO2018149499: not retrievable here. The repo's own `1x-neo.md` has it.
- Aero Hand Open: repo page has no spool/pulley detail (it is in the CAD and the docs at docs.tetheria.ai, not read). It warns that the tendon mechanism "requires regular tension adjustment" [V].
- Amazing Hand: page gives no geometry; "ball joint rods adjusted by hand one at a time"; listed deviations come from printed-part error, rod adjustment, horn rework and plastic flexibility [V]. Lesson: linkage hands also need tuning.

## 3. Bench tests that would settle the top picks
1. Bolt-clamp anchor vs Palomar knot vs 1.5-wrap + knot: 5 samples each, pull to failure.
2. Sheath types (PTFE 1x2, PTFE in spring, spring tube, Bambu feeder): same 300 mm S-bend, same weight, measure μ with the repo's capstan test (`tendon-mechanics.md` §8).
3. Sheath-end flexure + strain gauge: calibrate against hanging masses.
4. Pop-out detent: 3 depths, measure force.

## 4. Sources (all opened unless noted)
- ORCA, arXiv 2504.04259v2: https://arxiv.org/html/2504.04259v2 [V]
- CRAFT hand, arXiv 2603.12120: https://arxiv.org/html/2603.12120v1 [V]
- SoftHand Model-W, arXiv 2604.00738: https://arxiv.org/html/2604.00738v1 [V]
- Lightweight antagonistic Bowden hand, arXiv 2512.24657: https://arxiv.org/html/2512.24657v1 [V]
- MM-Hand, arXiv 2604.17245: https://arxiv.org/html/2604.17245 [V]
- BiDexHand, arXiv 2504.14712: https://www.alphaxiv.org/abs/2504.14712v2 [V]
- Faive hand, arXiv 2308.02453: https://ar5iv.labs.arxiv.org/html/2308.02453 [V]
- Aero Hand Open repo: https://github.com/TetherIA/aero-hand-open [V]
- Amazing Hand repo: https://github.com/pollen-robotics/AmazingHand [V]
- RUKA site: https://ruka-hand.github.io [V, high level only]; paper https://arxiv.org/abs/2504.13165 [S]
- Shadow docs: https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/2.2.0/user_guide/md_motor_unit.html and .../stable/user_guide/mh_mechanical_maintenance.html [V]
- US8056423: https://patents.google.com/patent/US8056423 [V]
- Tesla patent coverage: https://teslanorth.com/2026/04/16/teslas-robot-hand-patent-reveals-the-clever-engineering-inside-optimuss-fingers/ [V as press], https://driveteslacanada.ca/news/new-tesla-patents-reveal-optimus-v3-hand-human-muscles-tendons/ [S]
- Sailing splice data: https://www.ussailing.org/wp-content/uploads/2018/01/Dyneema-Spectra-Lifeline-Revision-Jan-2014.pdf , https://www.practical-sailor.com/sails-rigging-deckgear/working-with-high-tech-ropes/ , https://www.animatedknots.com/hundred-percent-strength , https://jimmygreen.com/news/post/86-12-strand-dyneema-break-strength-testing [S]
- Spiral spool: https://www.mdpi.com/2313-7673/10/3/155 [S, page 403]
- Friction-switch pulley: https://arxiv.org/pdf/1401.5232 [S]
- LEAP v2 Advanced (Humanoids 2025): https://www.originofbots.com/humanoid-robot-hand/leap-hand-v2-by-carnegie-mellon-university-robot-hand-specifications [S]
