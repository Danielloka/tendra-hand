# V1 wrist: build guide and materials (2-DOF, 32 tendon strands in PTFE tubes)

Date: 2026-11-04. Part of the "V1 wrist" study. Scope: what to buy, how to print it, how to thread and tension 32 strands through a 2-DOF wrist (bend = flexion/extension, wave = radial/ulnar deviation), at home, on a budget.

**Source marks.** [C] = confirmed by a source I opened or a search result listed in this session (link given). [R] = my engineering reasoning or a standard rule of thumb, not checked against a datasheet. [U] = unverified: from memory, or a rough price. Treat [R] and [U] numbers as "test before you trust".

**What the web search could and could not find (honest note).** It found the ORCA ratchet-spool paper, braid diameters for PowerPro/Sufix, PTFE bend-radius data points for *other* tube sizes, InMoov anchoring tips. It found **no datasheet** for 1 x 2 mm PTFE minimum bend radius, no crimp spec for 0.3 mm braid, and no print-in-place hollow 2-DOF wrist design to copy. Those parts are [R] and need a bench test (section 9).

Facts taken from the repo: V1 has 16 servos, 32 strands, each in a 1 mm bore / 2 mm OD PTFE tube; today's line is 0.4 mm nylon; `research/references/hands/tendon-mechanics.md` already recommends 50 lb 8-carrier braid (0.36 mm), pre-stretched; the wrist model (`tendra_hand_v1_wrist.xml`) has a 20 mm gap, all strands pass one point `wrist_centre`, flexion -60..+70 deg, deviation -30..+20 deg, forearm twist as a third axis; the palm sensor MCU needs 4 wires (`sensing.md` 3.3).

---

## 0. Summary

1. **Tendon:** switch to **braided UHMWPE (PowerPro / Sufix 832), 30 lb (0.28-0.29 mm) or 40-50 lb (0.33-0.36 mm)**. 30 lb is the easiest fit for a 1 mm bore plus knots, and still breaks at ~130 N nominal, far above the ~38 N stall pull of an SCS0009 on a 5 mm spool (figures from tendon-mechanics.md). Pre-stretch it. Nylon mono stays only for first test fits.
2. **Tubes:** keep **PTFE 1 x 2 mm** (not 2 x 4 mm: 32 of those will not fit a wrist). Do not bend tighter than **R = 15 mm** (design value already in the repo [R]); the one hard data point I found is for a *bigger* tube (SMC 4 mm OD: 15 mm min radius [C]).
3. **Wrist:** a **Cardan ring gimbal**: an inner ring and an outer ring with two pivot axes crossing at the wrist centre, a **~22-24 mm clear bore**, steel dowel pins or M3 shoulder bolts running in small bearings (623 or MR106). Print in **PETG** (or PA-CF if you can) with layers laid so pivots are not pulled apart along layer lines.
4. **Tube management:** tubes are **clamped at both ends** and **free (loose, not zip-tied) through the wrist**, with a slack allowance. A comb plate at each end keeps order. Colour code or number tags every strand.
5. **Tensioning:** build a **barrel adjuster (screw tensioner) into the tube end at the forearm side** for every strand, or an ORCA-style ratchet on the spool. Do not rely on knots to set tension.
6. **Cost:** wrist hardware plus tubes plus line is about **USD 60-120** (section 7), not counting the servos.

---

## 1. Tendon material

### 1.1 Nylon mono vs braided UHMWPE

| | Nylon mono 0.4 mm (today) | Braided UHMWPE (PowerPro, Sufix 832) |
|---|---|---|
| Stretch | High: 25 % at break [R]; at 10 N a 200 mm strand stretches several mm | Low: ~3-4 % at break [R]; ~0.4 mm at 10 N on 200 mm (repo estimate, tendon-mechanics.md 1.3) |
| Creep (slow lengthening) | High, and grows with humidity [R] | Low at small fractions of break load; creeps if held above ~20-30 % of break load or warm [R] |
| Friction on PTFE | Medium | Very low [R] |
| Knots | Easy, strong (85-95 %) [R] | Slippery: 50-80 %; use Palomar or a many-turn uni knot [R]; InMoov builders confirm braid slips and recommend a Palomar or tying around a washer [C: https://groups.google.com/g/inmoov/c/9E-mELHNH_s] |
| Wear risk | Abrades slowly | Cuts printed plastic edges ("thin line will cut into plastic" [C, same InMoov thread]); it saws if it runs over a sharp edge under load |

*Creep in plain words:* a loaded line slowly gets longer even at constant load. Then the loop goes slack and the joint gets play. UHMWPE is much better than nylon; pre-stretching removes the first, fastest part.

### 1.2 Sizes (diameter is what matters for the 1 mm bore)

Search results: PowerPro 20 lb = 0.23 mm [C: https://www.basspro.com/shop/en/77484]; Sufix 832 20 lb = 0.23 mm, 15 lb = 0.22 mm, 30 lb = 0.29 mm [C, retailer list found via search; Sufix PDF https://www.landbigfish.com/images/store/other/sufix_832.pdf has only the small sizes]. The repo's own table (tendon-mechanics.md 1.5, checked 2026-09-30): 30 lb = 0.28, 40 lb = 0.33, 50 lb = 0.36, 65 lb = 0.41 mm [R from catalogues].

Note: fishing "lb" labels are often knotted or nominal strength, so real breaks vary [U, forum claim: https://barbel.co.uk/forum/goto/post?id=240872]. Buy by **diameter**, measure with calipers (0.01 mm), and test-pull a sample.

**Recommendation for the wrist:** 30 lb (0.28 mm) in 1.0 mm bore leaves 0.7 mm clearance, which keeps friction down when the tube is bent and tolerates the small kinks of an S-curve. Use 40-50 lb only on strands that need it (finger mcp_flex). Keep one spool of 0.4 mm nylon for trial threading, because you can push it through a tube without it fraying.

### 1.3 Termination: knots, ferrules, crimps

- **Best practice for braid:** wrap the line at least one turn around a post or drum next to the tie hole, then tie. The capstan effect means the knot sees only a small part of the load (repo calculation, 1.4: ~15 % after 1.5 turns on PLA). Seal the knot with thin CA, chamfer every hole (>= 0.5 mm) [R].
- **Crimps/ferrules:** standard aluminium or copper sleeves (0.8-1.0 mm bore) work well on **mono** and on steel cable. On 0.3 mm **braid** they tend to slip or cut fibres unless double-crimped; I found no source giving a rated spec [U]. Use a knot plus a sleeve as a "stopper" only if you test it by hanging 5 kg.
- **Servo end:** a screw-and-washer clamp on the spool (InMoov builders do this [C, same thread]) or a tie hole plus extra wrap. Do it identically for all 32 strands.
- **Finger end:** the repo ties the line through a hole in the drum; keep that.

### 1.4 How others anchor tendons

- **ORCA hand (ETH Zurich):** 17 DOF, tendon-driven, built for < 2000 CHF, assembled in < 8 h; a **ratchet on the spool** lets you re-tension "in seconds without unscrewing the spool or tendon" [C: https://arxiv.org/html/2504.04259v1 and the summary in search results]. Tendon is 0.4 mm braided nylon per the repo's earlier read [R, not re-checked here].
- **DexHand:** repo notes say Sufix 832 80 lb; I found **no** confirmation of its anchoring in this search [U].
- **InMoov:** fishing line (often braid), knots at the servo pulley with screws and washers, and fingertips tied round a small washer [C: https://groups.google.com/g/inmoov/c/9E-mELHNH_s; https://inmoov.fr/lining-and-tighting-the-tendons/ lists the routine, I did not open its details].

### 1.5 Pre-stretch (do before the first install)

Hang ~7 kg on a 1 m length of braid for a few hours, or cycle 10 kg x 20 times; mark the line before and after to see how much it grew [R, same as tendon-mechanics.md]. Cut all 32 strands from the pre-stretched length, plus spares.

---

## 2. Sheaths (PTFE tubes)

### 2.1 Which tube

| Option | OD | Verdict |
|---|---|---|
| **PTFE 1 x 2 mm** (ID 1.0, OD 2.0) | 2.0 | **Use.** 32 tubes pack into a bundle of about 14-16 mm diameter (hex packing, 32 x pi x 1 mm^2 = 100 mm^2 area / 0.7 fill) [R], which fits a 22 mm bore with room for slack |
| PTFE 2 x 4 mm (the common "Bowden" 3D-printer size) | 4.0 | Too fat: 32 tubes need > 35 mm. Sold widely, e.g. https://west3d.com/en-au/products/bowden-ptfe-tube-4mm-od-2mm-id [C] |
| **Capricorn XS** | ~4 mm OD, 1.9 mm ID (filament 1.75 mm) [C: https://grobotronics.com/capricorn-xs-series-ptfe-bowden-tubing-for-1.75mm-filament.html] | Wrong size for us. It is just low-friction PTFE. Its "low friction" claim does not help a 0.3 mm line in a 1 mm tube. |

So "Capricorn style" is not available in 1 x 2; buy generic PTFE 1 x 2 mm (hobby-grade is fine) or FEP/PTFE capillary tubing [U on exact vendors].

### 2.2 Minimum bend radius (kink point)

- No 1 x 2 mm datasheet found [C: search said so]. Data points for other sizes: SMC 4 mm OD tube, min bend radius 15 mm [C: https://uk.rs-online.com/web/p/hose-pipe-flexible-tubes/1961220]; Festo 4 mm OD tube, 32 mm [C: https://uk.rs-online.com/web/p/hose-pipe-flexible-tubes/0651940]. Scaling by OD gives 8-16 mm for a 2 mm tube [R, search tool's own estimate].
- **Design rule:** R >= 15 mm (repo already uses this), **test** by bending a 20 cm sample around a 20 mm, 15 mm and 10 mm round object with a line in it and pulling; the radius where pull force jumps or the tube flattens is your limit. A kinked PTFE tube stays kinked: throw it away.
- Friction rises with total bend angle (capstan: tension drops by e^(mu x theta)). Rough [R]: braid in PTFE has mu ~0.05-0.1, so a total of 180 deg of bend costs 15-30 % per strand. Keep total bend angle through the arm + wrist low.

### 2.3 Anchoring the sheath at both ends

The sheath must not move or the strand length changes. Options, in order of preference:

1. **Printed clamp pocket:** a 2.05 mm hole with a slot and an M2 grub/cap screw that pinches the tube slightly (do not squash it, PTFE flows and the line will drag). Add a short **stainless or brass 2 mm OD / 1.2 mm ID insert tube** (K&S style brass tube) over the last 10 mm so the clamp bears on metal, not on soft PTFE [R].
2. **Stop collar + bore:** the tube pushes into a 2.0 mm hole until it bottoms on a step with a 1.2 mm through-bore (the repo already designs "1.2 mm bore for 6 mm at the entry, then 2.2 mm for the tube"). A drop of CA at the mouth locks it. PTFE does not bond to CA, so rely on **mechanical** retention (a bump or flare) not glue [R].
3. **Heat-flared end:** warm the tube tip with a hot air gun and press it flat for a 3 mm flange; it cannot slip through the hole. Works for hobby builds [R].
4. At the forearm end the anchor is also the **tensioner** (section 6).

Cut tubes square with a fresh blade or a rotary cutter, deburr the bore with a 0.8 mm drill by hand so the line does not snag.

### 2.4 Keeping 32 tubes tidy

- **Comb plates:** a 3 mm printed plate with 32 numbered holes (2.1 mm) at each end of the forearm-to-palm run. This stops crossings and gives each strand an address.
- **Bundle with loose sleeving:** in the straight sections a **braided PET expandable sleeve** (6-10 mm for sub-bundles, 20 mm for the whole bundle) keeps order without clamping. Zip-tie only in straight sections, never across the wrist [R].
- **Sub-bundles by finger** in different sleeve colours: thumb 4 servos = 8 strands, index/middle/ring/little 3 servos each = 6 strands each (8 + 24 = 32).
- **Cable chain** is not needed for a wrist this small; a loose sleeve with a service loop is lighter and quieter [R].
- **Labelling:** servo ID = strand label (e.g. `07F` = servo 7 flex strand, `07E` = extension). Use coloured heat-shrink on both ends of each tube **before** threading, or a printed numbered comb. Write the code on the tube with a paint marker too. Servo IDs and colours match `config_v1.h` [from repo].

### 2.5 Slack loops at the wrist

Why: when the wrist bends, a tube that is off the centre line needs a different length. If the tube is too short, it tightens and drags the line (the joint moves by itself). [R]

- Rule: extra tube length ~ (distance of the tube from the wrist centre) x (max bend angle in rad). For the worst tube at 8 mm radius and 70 deg (1.22 rad): ~10 mm extra [R].
- Free (unclamped) tube span across the wrist **>= 30 mm** so a 70 deg bend needs R = 30/1.22 = 25 mm, comfortably above 15 mm [R].
- Do not zip-tie the bundle in the free span. A loose braided sleeve is OK.
- The sim already routes every strand through one `wrist_centre`, which is the right goal: keep tubes close to the pivot crossing so the offset is small.
- Check by moving the wrist through its range with the fingers locked: no finger joint should move more than ~1 deg. That is acceptance test #1.

---

## 3. Wrist joint hardware

### 3.1 Layout idea (2 DOF, hollow)

- **Cardan ring:** forearm-side yoke -> **outer ring** (pivot axis A, e.g. flexion) -> **inner ring** (pivot axis B, deviation) -> palm-side plate. Axes cross at the wrist centre; the ring bore is the tube passage (>= 22 mm). Hollow-wrist concepts exist in the patent literature (e.g. https://patents.google.com/patent/US4744264 [C: found, not read in detail]); print-ready hollow 2-DOF wrists are rare, so expect to design it yourself in Fusion [C: search found only a conduit gimbal https://www.myminifactory.com/object/36438].
- Which axis is outer? Flexion has the bigger range (-60..+70), deviation the smaller (-30..+20) [repo]. Put **flexion on the outer** ring so the biggest swing has the largest clearance, deviation inner [R].
- **Twist** (`forearm_rot`) is upstream of this wrist (where OpenArm's flange bolts on) and is not covered here.

### 3.2 Bearings and pins (cheap, hobby-grade)

| Option | Size | Use | Note |
|---|---|---|---|
| **Steel dowel pin 3 mm** in a **printed reamed hole** (drill 3.0 then ream) | 3 x 20 mm | The cheapest pivot; PETG on steel runs fine with a drop of PTFE or lithium grease | Wear grows over time; fine for a first version [R] |
| **M3 shoulder bolt** (3 mm shoulder, M2.5 thread, or 4 mm shoulder, M3) | ISO 7379 | Pivots with a defined shoulder; tighten against a lock nut with a **spring washer** to remove play | Easy to buy [U on exact part number] |
| **623 bearings** (3 x 10 x 4 mm) pair per pivot | Cheap in bulk | Two per pivot, pressed into 10 mm pockets, shaft = M3 shoulder bolt or 3 mm dowel | Smooth, low backlash if preloaded [R] |
| **MR106 / 686** | 6 x 10 x 3 / 6 x 13 x 5 | Bigger pivots if the load gets high | |
| **688** (8 x 16 x 5) | | If you use 8 mm axles | Bulky for a wrist this small |
| Thin-section **61806 / 6806** (30 x 42 x 7) | | Only for a slewing ring, i.e. the *twist* axis, not the gimbal pivots | The bore is 30 mm: too big for our 22 mm tube passage [R] |

I did not find a build with exactly these parts; the recommendation is engineering judgement [R].

### 3.3 Material and print orientation

- **PLA**: stiff but brittle and creeps near warm servos; use for prototypes only.
- **PETG**: tougher, a little flexible, good default for the load-bearing rings [R].
- **PA-CF (nylon carbon fibre)**: stiffest and strongest, needs a hardened nozzle and a dry filament box [R].
- **Orientation:** layer lines are the weak direction. Print each ring **flat** (axis of the bore vertical) so the ring is made of continuous loops; put the pivot holes' load across the layers' strong direction, which means pivot loads go **in the plane of the layers** (not trying to peel layers apart). Where a pivot boss sticks up, print it so the boss is in the XY plane or use a **steel pin that carries the load** rather than a printed shaft [R].
- Use 4+ perimeters and ~50 % gyroid or cubic infill around pivots, 0.16-0.2 mm layers [R].
- Pivot-pin load check: wrist torque at the tip of the hand is small (< 1 N·m for hand+object ~1 kg at 0.1 m); a 3 mm steel pin holds this with a huge margin; the printed bearing pocket is the weak point [R].

### 3.4 Fasteners, backlash, stops

- **Heat-set inserts** (M2, M2.5, M3, brass, ~4 mm long): put them into the rings wherever a screw will be removed more than a few times. Press in with a soldering iron at ~230 deg for PETG [R]. Never screw directly into printed plastic at a pivot.
- **No backlash:** (a) press-fit bearings (print the pocket 0.1 mm undersize and ream), (b) a **preload**: M3 shoulder bolt + wave/spring washer + nyloc, (c) keep the bearing spacing >= 2 x shaft diameter, (d) if the wrist is driven by servos with tendons, **keep a small pretension** on the driving strands so it cannot rattle [R].
- **End stops:** hard stops printed into the rings at the software limits +2 deg (-62..+72 deg flexion, -32..+22 deg deviation, from the sim). The motor's torque cannot then push the ring beyond a safe tube bend. Add a 1 mm TPU bumper on the stop faces to stop noise [R]. The tubes must still be happy at the stop positions: check R >= 15 mm there.

### 3.5 Sensor wires (4 wires for the future palm MCU)

- 4 wires: +V, GND, and a data pair (UART/RS-485 or I2C-lite) [sensing.md 3.3].
- Use **30 AWG silicone-insulated stranded wire** (flexible, survives repeated bending) in a **separate 1 x 2 mm PTFE tube or a 1.5 mm braided sleeve** through the same wrist bore [R].
- Strain relief: leave a service loop with the **same free span as the tubes**, clamp the cable with a printed saddle (or zip tie) at both ends **on the sleeve**, not on the wires, and put a drop of CA or flexible glue where the wire meets the solder joint. Avoid soldering at a bend; use a **JST-SH 4-pin** or similar connector at both ends so the wrist can be taken apart [R].
- Keep the data pair away from the servo power wires in the bundle to cut noise; twist the data pair [R]. Do not run it as a straight single strand through the wrist: a wire fixed at both ends and bent dozens of thousands of times breaks (fatigue). A coiled loop is better [R].

---

## 4. Assembly order (so tendons can be threaded and re-tensioned without taking the wrist apart)

The aim: all threading and tensioning happen from the **forearm end** or the **finger end**, with the wrist closed.

1. Print and test-fit all parts. Chamfer holes. Label strand numbers on the comb plates.
2. Assemble the **wrist gimbal first**, empty: pins, bearings, stops. Check it moves freely over its whole range without binding, with no tubes inside. Fix play now.
3. Pre-cut 32 tubes (+ 2 spares) to length with the slack allowance (section 2.5); put label sleeves on both ends.
4. **Install the tubes** (bundle by finger) from the **palm side through the wrist to the forearm comb**, with the wrist at neutral. Clamp the palm end (and the forearm end for now) in their pockets. Do not clamp anything across the free span.
5. **Thread the lines**: push a pre-stretched 0.28-0.4 mm line through each tube from the spool end to the finger end (use a 0.4 mm nylon "leader" first for tight tubes, or a stiff 0.1 mm wire loop). Do the 16 loops: flex and ext strands of the same joint go in as one loop on its drum.
6. Tie the line at the finger drum (an extra wrap around the post) with the finger in its neutral pose. Seal with CA.
7. **Spool end:** wrap on the 5 mm spool (same direction convention for all), clamp or tie. Servos in the middle of their range (torque off, `q = 0` pose).
8. **Tension** (section 6): use the barrel adjusters or ratchets until the strand has the target light pretension (tendon-mechanics.md 3.2: keep it low).
9. Sweep the wrist through its full range (acceptance test #1, section 2.5). Fix any strand that moves a finger joint > 1 deg.
10. Add sleeves and combs, ties on straight sections only. Last: connect sensor wires.

If the palm and forearm are separate parts, put **the plug-in split** at the wrist: a bolted flange with the tubes already clamped on each side and a **loose, longer** free span. The tubes then never need to be cut to reassemble.

---

## 5. Tensioning methods

| Method | How it works | Pros | Cons |
|---|---|---|---|
| **Ratchet spool (ORCA)** | A one-way ratchet on the spool; turn to take up slack, locks the other way [C: https://arxiv.org/html/2504.04259v1] | Re-tension in seconds, no knots to retie | Needs space on the spool; has a step size (one ratchet tooth) |
| **Barrel adjuster on the sheath end** (bike brake cable style) | A threaded sleeve in the forearm comb plate with the tube clamped inside; turn it to move the tube end ~0.5 mm per turn (M3 thread 0.5 mm pitch), which changes the loop tension or slack of the strand | Fine, repeatable, **can be done with the wrist closed**; each strand has its own screw | 32 adjusters = 32 x M3 nut + locknut; take care not to rotate the tube (PTFE twists) [R] |
| **Turnbuckle** (in line) | Left/right-hand thread | Precise | Bulky; too big for 0.3 mm line in a tube [R] |
| **Screw tensioner at the finger** | Slider on the drum | Fine control | Space in the finger is tiny; not recommended |
| **Software slack-take-up** | Servo reads load, turns a bit to restore pretension | Free | Only compensates stretch within one strand's range; the SCS0009 backs off after 2 s above 80 % load (repo), so keep pretension low |

**Recommendation:** forearm-side **barrel adjusters** as the main, plus a servo spool with a **tie point plus screw clamp** for coarse adjustment. If you have time, make one ratchet spool and try it on one servo first.

**Target pretension:** just enough that both strands of a loop are taut at q = 0 with no slack, no visible bowing. Measure with a luggage scale or a 50 g hanging weight on a spare strand [R].

---

## 6. Replacing one broken tendon

Plan for this at design time: a broken line must be replaceable **without opening the wrist**. [R]

1. Torque off. Move the joint to neutral; note the label (e.g. `12F`).
2. At the spool end, free the old line and pull it out of the tube **from the finger end** (or cut at the finger end and pull from the spool end).
3. Thread a new pre-stretched length through the **same tube**: tie it to the tail of the old line and pull it through (the old line acts as a leader), or push a 0.4 mm nylon leader through first.
4. Tie it at the finger drum, wrap on the spool, tension with the barrel adjuster (section 5).
5. Wave the wrist and the finger through the range; confirm acceptance test #1.
6. If the **tube** is damaged or kinked: pull the tube and the line together out of its clamp pockets (the clamp screws are accessible from the outside), push in a new tube. This works only if the free span is long enough.

Keep 5 pre-cut spare strands and 2 spare tubes with labels already on [R].

---

## 7. Bill of materials (rough, per wrist)

Prices are **rough, USD, mid-2026 guesses [U]**; NOK is about 10-11 x USD. Buy locally or from AliExpress / Amazon / Digi-Key / Mouser / Misumi as available.

| Item | Qty | Spec | Rough price | Where (typical) |
|---|---|---|---|---|
| Braided UHMWPE line, 30 lb (0.28 mm) | 1 spool, 100 m+ | PowerPro / Sufix 832 [C diameters above] | 15-25 | Fishing shop, Amazon, Bass Pro |
| Braided line 50 lb (0.36 mm) | 1 spool | for mcp_flex strands | 15-25 | same |
| Nylon mono 0.4 mm | 1 spool | trial threading | 5 | same |
| PTFE tube 1 x 2 mm | 15-20 m | 32 strands x ~0.4-0.6 m + spares | 15-30 [U] | AliExpress, eBay, Misumi, RS |
| Brass tube 2 x 1.2 mm (K&S) | 1 m | clamp inserts (optional) | 5-8 | hobby shop |
| 623 bearings (3x10x4) or MR106 | 8-12 | pivots | 5-10 | AliExpress |
| M3 shoulder bolts / 3 mm steel dowels | 4-8 | pivots | 5-10 | Misumi, Amazon |
| M3 heat-set inserts (assorted M2-M3) | 1 kit | | 8-12 | Amazon |
| M3/M2 screws, locknuts, wave washers | 1 kit | | 8 | |
| Barrel adjusters (M3 bolt drilled 1.2 mm, or bike inline adjusters) | 32 | | 3-15 (DIY bolts: ~3) | hardware store |
| Aluminium sleeves 1 mm (optional) | 100 | | 5 | AliExpress |
| Braided PET expandable sleeve 6, 10, 20 mm | 3 m each | | 10 | Amazon |
| Heat shrink / label tape / paint marker | | colour coding | 8 | |
| 30 AWG silicone wire (4 colours), JST-SH 4-pin pair | | sensor line | 8 | |
| Filament: PETG 1 kg (+ optional PA-CF 0.5 kg) | | | 20 (+35) | |
| TPU 95A small spool | 1 | bumpers | 15 | |
| Thin CA glue, PTFE/lithium grease | | | 8 | |
| **Total (without PA-CF)** | | | **about 150-210 USD** gross, of which ~60-120 is wrist-specific; the rest overlaps with the finger build |

---

## 8. Build checklist

- [ ] Measure the real braid diameter with calipers; confirm it slides through a 1 m piece of the tube by hand.
- [ ] Pre-stretch all line; mark and re-measure.
- [ ] Bend test: one sample tube round 15 / 20 mm radius with line; log pull force.
- [ ] Print gimbal rings flat; ream pivot holes; test-fit bearings; deburr.
- [ ] Gimbal moves smoothly through the full range empty; end stops work; no play (< 0.5 deg at the palm edge).
- [ ] Cut and label 32 + 2 tubes, label both ends (strand `NNF/NNE`).
- [ ] Combs and clamps fit; clamp screws reach from the outside.
- [ ] Thread, knot with an extra wrap, CA, and spool, all strands the same winding direction.
- [ ] Tension each strand with its adjuster; both strands of every loop taut at q = 0.
- [ ] Wrist sweep test: no finger moves > 1 deg (all axes, all limits).
- [ ] Sensor wires with service loop and connectors.
- [ ] Run a first servo motion at low speed and low current; read loads (`F` command) and check no strand goes near 80 % load.
- [ ] Photo + log in `research/log.md`.

---

## 9. Common mistakes

1. **Zip-tying the tube bundle across the wrist.** Tubes then cannot slide or slacken; strands tug fingers.
2. **Tubes too short or the same length for all.** The outer ones are pulled tight at full bend.
3. **Kinked PTFE.** Once creased it stays creased; replace it.
4. **Using 2 x 4 mm Bowden tube** because it is what shops stock.
5. **Hole edges not chamfered**: braid saws through the print in days.
6. **Trusting the knot** with braid. Always an extra wrap, CA, and a pull test of 5 kg.
7. **Over-tightening**: the SCS0009 backs off after 2 s above 80 % load; high pretension looks like a stalled joint.
8. **Loading pivots along layer lines**, or screwing straight into plastic at a pivot (use inserts).
9. **No labels**: with 32 strands, a swapped pair costs hours.
10. **Sensor wires fixed straight across the wrist**; they break from fatigue after a few thousand cycles. Use a loop.
11. **No hard stops**: software limits fail once; stops don't.
12. **Not pre-stretching the line**, then blaming the servos for "drift" in the first week.

---

## Sources

- [C] ORCA hand paper (ratchet spool, 17 DOF, < 2000 CHF): https://arxiv.org/html/2504.04259v1
- [C] SMC PTFE tube, 15 mm min bend: https://uk.rs-online.com/web/p/hose-pipe-flexible-tubes/1961220 ; Festo PTFE tube, 32 mm: https://uk.rs-online.com/web/p/hose-pipe-flexible-tubes/0651940
- [C] 2 x 4 mm Bowden tube: https://west3d.com/en-au/products/bowden-ptfe-tube-4mm-od-2mm-id ; Capricorn XS dimensions: https://grobotronics.com/capricorn-xs-series-ptfe-bowden-tubing-for-1.75mm-filament.html
- [C] PowerPro 20 lb = 0.23 mm: https://www.basspro.com/shop/en/77484 ; Sufix 832 chart (small sizes): https://www.landbigfish.com/images/store/other/sufix_832.pdf
- [C] InMoov anchoring forum thread: https://groups.google.com/g/inmoov/c/9E-mELHNH_s ; tendon routine: https://inmoov.fr/lining-and-tighting-the-tendons/
- [C, not read in depth] hollow wrist patent: https://patents.google.com/patent/US4744264 ; hobby gimbal: https://www.myminifactory.com/object/36438
- [U] Fishing-line "lb" is knotted strength (forum claim): https://barbel.co.uk/forum/goto/post?id=240872
- Repo: `research/references/hands/tendon-mechanics.md`, `research/references/hands/sensing.md` (3.3), `CLAUDE.md` (V1, wrist model).
