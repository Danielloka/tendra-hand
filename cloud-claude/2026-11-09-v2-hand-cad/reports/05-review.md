# 05 · Adversarial review (V2 concept hand)

Reviewer, 2026-11-12. Read: BRIEF, reports 01–04 (incl. §11 fix round), `cad/params.py`, skimmed `finger.py`,
`thumb.py`, `wrist.py`, `wrist_kin.py`, `routing.py`, the JSON outputs and the pose renders. My numbers come from
`out/assembly/routes.json`, `fan_routes.json`, `checks.json`, `out/finger/summary.json`, plus two small scripts
(scratch: `fric.py`, `comp.py`; hand calculations, not FEA). I edited nothing in the study except this file.

## Verdict

The geometry work is good. Every drum is a true 7 mm circle, the loop lengths are exact, the strand-coupling
budgets of ≤ 0.2 mm are really in the files, the poses are interference-free, and the thumb kinematics are much
better than V1's. **But the drivetrain physics was not checked, and it decides how this hand behaves.** The plain
PTFE 1 × 2 sheaths are the compression member of every Bowden cable. At SCS0009 stall load they are stressed above
PTFE's compressive yield, and they shorten by about 8 mm (about 70° of joint angle). The total wrap angle gives
45–75 % efficiency, not the 90 % the force numbers assume. The wrist gear and crank are undersized by 3–5×. Do not
print it as a "precise position hand" until finding #1 is fixed or a test proves it wrong. As a geometry concept
it is a sound base.

## Findings (ranked)

| # | Sev. | What | Evidence (numbers / files) | Suggested fix | Effort |
|---|---|---|---|---|---|
| 1 | **critical** | **The PTFE sheaths are the compression member, and they are too soft and too weak.** In a Bowden drive the housing carries the full strand tension. No report computed this. | PTFE 1 × 2: A = 2.36 mm², E ≈ 0.4–0.6 GPa. Sheath length yoke→adjuster = free span + palm + wrist (~75) + fan = **156–246 mm (median 221)** from routes/fan_routes. At 45 N (SCS0009 stall through a 5 mm spool): **19 MPa** (PTFE compressive yield ≈ 10–12 MPa, and it creeps well below that) and **≈ 8.4 mm of shortening ≈ 69° of joint angle**. At 118 N (HLS): 50 MPa, 22 mm. At a 5 N pretension: 0.9 mm (7.7°). Add the braid's own stretch (EA ≈ 2.8 kN: 4.8 mm at 45 N) and the loop is a spring of ≈ 3–4 N/mm, i.e. joint stiffness ≈ 0.17 N·m/rad: under full load the joint lags its commanded angle by ~1–1.7 rad. The 0.19 mm decoupling budget is 40× smaller than this compliance. Creep turns it into permanent slack (loss of pretension within days). | Use a **compressionless housing**: a PTFE-lined stainless coil (bike-style, ~2.5 mm OD, bore 2.7) or **PEEK 1 × 2 tube** (E 3.6 GPa, yield ~100 MPa: 7× stiffer, 2× stronger than the load). **Bond or capture** the sheath in every rigid section (palm bores), so only the free spans (22 mm yoke + ~75 mm wrist + fan) are compliant. **Add joint-angle sensing** (Hall/magnet at each MCP/PIP) if precise positions matter. Bench test first: 300 mm of the real tube, a hanging weight of 5/15/45 N, a dial gauge. | M |
| 2 | **major** | **The friction budget is far worse than the 90 % used for every force number.** | Total turning per strand (palm `turn_deg` + fan turns from `fan_routes.json` points + finger/thumb internal bends + 58° wrist at the extreme): **162–462°, median ≈ 300°**. Worst: thumb_ip (457–462°), thumb_mcp_flex (356–399°), index_pip_flex (356°), little_mcp_flex_flex (palm alone **242°**, more than the 202° in report 04 §4). Efficiency e^(−μθ): **0.45 / 0.60 / 0.75 (min/median/max) at μ 0.1; 0.30 / 0.46 / 0.65 at μ 0.15**. Fingertip force with SCS0009 is therefore ≈ **2.0–2.5 N**, not 3.4 N. The thumb pinch is ≈ 3 N, not 5.5 N. Pretension drag and a direction-dependent force hysteresis come on top (tendon-mechanics.md §2). This is *lower* than V1's 3.4 N, while the research README names fingertip force as the main gap (3.4 vs 45–80 N). | Put the real η into the force tables. Cut the palm turns (no strand > 120°: the little-finger strands and the thumb fan). Re-route the forearm fan so its curves are only gentle S-curves. Accept that SCS0009/HLS on a 5 mm spool cannot close the force gap: the strength roadmap needs a different actuator class or ratio, not routing tweaks. | M |
| 3 | **major** | **The wrist gear stage and crank cannot carry the torque the reports claim.** | 14T m0.7 pinion, 3 mm face (`wrist.py` l. 263–265; the gears are toothless cylinders). At STS3215 stall (2.94 N·m) the tangential force is 600 N, so the Lewis bending stress is **≈ 1030 MPa**. At "continuous" 0.9 N·m it is 316 MPa: beyond hardened steel, and 10× beyond any printed gear. Crank torque at stall = 7.6 N·m. The **5 mm crank axle sees 308 MPa** of torsional shear, and the **3 × 5 mm crank root sees ≈ 600 MPa** of bending. The claimed "flex ≥ 3.6 N·m stall" (report 03 §4) is not deliverable. | Size it properly: module ≥ 1.0, face ≥ 8 mm, steel or brass gears; an 8 mm axle; a crank web ≥ 6 × 10 mm. Or drop the gear stage: a longer servo horn plus a smaller crank radius, or a timing belt. Then re-run `optimize_drive` with the real part sizes. | S–M |
| 4 | **major** | **The forearm fan model does not hold the sheath length it must absorb.** | Report 03: "cut each sheath at q = 0 path + 7 mm", and the wrist changes each sheath's path by −6.7…+6.1 mm, taken up by the fan. But the routed fan (`fan_routes.json`) is almost tight: length minus chord = **0.0 / 0.5 / 6.1 mm (min/median/max)**, over 53–71 mm of tube. The real fan therefore holds **0.3–13.7 mm of extra tube** in a 55 mm tall bay. Its static min R is already **11.3 mm (< 15)**. The real bends, tangling of 32 tubes and the extra friction (up to ~+100° of turning at a wrist extreme) were never checked. If a tube binds in the Ø2.3 comb, the "wrist does not move the fingers" claim fails (the wrist length change then reaches the strand). | Model the fan at the 3 cases (wrist ΔL −6.7 / 0 / +6.1, plus 7 mm pre-bow). Better: give the take-up its own place (a long, gentle, guided loop, ~100 mm, or a sliding sheath channel), or end the sheaths on both sides of the wrist and cross the centre bare, as Optimus does. | M |
| 5 | **major** | **The strands bend over very tight radii at the MCP eyelet and the thumb nip pins, and they slide there under load.** | 18G eyelet rim r 0.43 mm (D/d ≈ 2.4), up to 90° of wrap at a fist. Contact pressure T/(r·d) ≈ **290 MPa at 45 N, 760 MPa at 118 N**, and the strand slides over the rim on every MCP and PIP move. The thumb nip pins (Ø1, r 0.68) under cmc_flex take up to 60° of wrap. UHMWPE braid at D/d < 5 loses about 40 % of its strength and frays quickly. Report 01 itself lists this as "what fails first", but nothing mitigates it, and the force numbers do not include the eyelet's extra friction (e^(0.1·π/2) = ×1.17). | Use a **Ø3 polished pin or a Ø3–4 roller** whose surface is at the axis (the error grows to ~0.3–0.5 mm, which is still small next to #1). Use 80 lb braid everywhere. Put the 10 k-fist cycle test *before* any other print. | S |
| 6 | **major** | **Nothing positively holds the sheath ends against the compression load.** | The palm clamp bores are Ø2.0 on a 2.0 OD tube, "2.0 = squeeze" (`wrist.py` l. 214): a friction grip on PTFE (μ ≈ 0.05–0.1). To hold 45 N it would need 450–900 N of clamp force per tube. The thumb tubes are gripped over only 2.5 mm, and §11.9 #8 says "needs glue". PTFE does not bond without etching (sodium naphthalene). It is not clear whether the palm and wrist sheath segments are one tube or two. If they are two, the clamp is a load path that slips. | Give every sheath end a positive axial stop: a step in the bore, a flared end, or a crimped brass ferrule (Ø2/Ø3) seated against a shoulder. State whether the tubes are continuous from yoke to adjuster. Use etched PTFE or mechanical ferrules, never "glue". | S |
| 7 | **major** | **The HLS3606M upgrade path is blocked by the parts around it.** | Everything is sized for SCS-level loads. At 118 N strand load: PTFE 50 MPa (#1), eyelet 760 MPa (#5), line pressure in the PETG drum groove T/(R·d) ≈ 47 MPa (PETG yield ~50 MPa, it creeps), and the 2 mm MCP stubs at 15 MPa in PETG (report 01). The thumb's own HLS request (ID 5, thumb_mcp_flex) sits in a slot with **no HLS room** (report 04 §8). HLS3606M torque and range (the thumb cmc_rot needs 197° of servo travel) come from memory. | Decide the load class first (SCS or HLS). For HLS: coil housing, larger pins, steel-lined drum grooves (or a groove 2× wider), brass bushes at the MCP. Re-slot ID 5. Verify the HLS3606M datasheet. | M |
| 8 | **major** | **Replacing a strand is a teardown.** | The drum anchor posts are captured by the cheeks, and the MCP stubs are pressed into PETG, so a finger strand cannot be changed without taking the finger apart. A limp 0.36 mm braid must be threaded through **156–246 mm of curved PTFE** (up to 242° of turning in the palm alone) and the palm's internal Ø2.2 swept bores. **2 of the 6 thumb-mount bores were never cut** (failed OCC booleans, §11.9 #4), so those two thumb strands have no channel as printed. After the fix round the palm clamp sits inside the flange opening, and it is not shown that its screws can be reached with the palm mounted. | Make pins removable (screw-retained shoulder pins, not presses). Use open palm troughs with a lid instead of buried bores. Plan a stiff threading leader (0.3 mm steel wire) in every tube. Re-model the thumb mount as a sleeve with all 6 bores. Show clamp screw access. | M |
| 9 | minor | **Wrist bundle: the strand shifts inside the bore.** | Bore 1.0 vs line 0.36: the strand hugs the inside of each bend, so the path changes by (1.0 − 0.36)/2 × Δθ ≈ 0.32 mm over a 58° wrist bend. That is mostly common-mode slack (≈ 1 N less pretension), with ≤ ~1° of real coupling. Report 03's "~0.1 mm per 10°" is about right. | A 0.8 mm bore liner, or a small extensor spring. | S |
| 10 | minor | **The knot load is optimistic.** | "Knot sees ≈ 25 %" (report 01): 1–1.5 wraps on a steel post at μ ≈ 0.06–0.1 leave e^(−0.1·2π·1.25) ≈ 46 % (55 % at μ 0.08). At the MCP the drum has only 40° of wrap margin at the range end. UHMWPE knots hold only 50–80 % of the line's strength. | Use crimp sleeves (the thumb already does), or 3 wraps plus CA. | S |
| 11 | minor | **The thumb's numbers are not final.** | The opposition score regressed from 32.2 to **37.25** after the palm change and was not re-optimised (§11.4). The pinch forces assume 90 % efficiency (see #2). The thumb-vs-palm check covers only 9 poses at MCP/IP = 0 (`checks.json`), and one corner hits the palm (650 mm³), so it needs a firmware limit. cmc_rot still uses an idler plus a bent steel tube. | Re-run the optimiser on the real palm and with η. Do a full 4-joint collision sweep for the firmware limits. | S |
| 12 | minor | **Finger spread is limited.** | Ring/little abduction is capped at **9°** (`abd_pair_table.json`). Neighbouring fingers may only spread apart. | Widen the ring–little spacing by 2 mm (report 04). | M |
| 13 | minor | **The wrist bearings and rods are marginal.** | 623 bearings: C0 ≈ 0.22 kN against ≈ 235 N at peak load (already noted). The rods (≈ 470 N) and ball ends are exposed across the gap. Euler buckling of the M4 rod at the thread root is ≈ 1.2 kN: OK. | 4 mm pins with 624 bearings after a ring rework; metal ball links; a bellows. | M |
| 14 | minor | **Printability.** | The palm has 24 buried, swept Ø2.2 bores for 2.0 OD tube. FDM bores come out 0.1–0.2 mm small and faceted, so pushing a tube through a 240° turn will likely jam. The thumb seat has thin walls (§10 #5). The thumb has a ~0.8–1.2 mm wall between the cmc groove and the central slot (report 02), under the 1.6 mm minimum. The finger renders show many slots and recesses that are fine when printed on the side (report 01), but no slicer check was done. | Use open channels with a snap lid, and ream them with a Ø2.2 drill. Thicken the thumb hub wall, or add steel washers as groove flanges. | S |
| 15 | minor | **The report has stale numbers.** | The report 04 §4 table (round 1) disagrees with `routes.json`: for example, index_mcp_flex_flex is in cell 30 in the table but cell 8 in the file, and the little_mcp_flex palm turn is 202° in the table but 242° in the file. §11 says it supersedes these, but the old table is still the most detailed one. | Regenerate §4 from routes.json. | S |

## Claims verified

- **Common 7 mm drum and a constant mm per degree:** `out/finger/summary.json`: own slope ±0.12217 mm/° on all three
  finger servo joints, non-linearity and loop-sum variation ~1e-14 mm. The thumb uses the same 7 mm (report 02,
  `routing.json`).
- **Finger decoupling numbers:** pip strand vs MCP **0.192 / 0.128 mm** (index), as reported. The other cross terms
  are 0. The polyline check agrees to 0.002 mm.
- **DIP bar:** index bar 22.87 mm, max ratio error 0.51°, transmission ≥ 38.6°: matches `summary.json`.
- **Validity:** 223/225 parts pass (`checks.json`). The 2 that fail are valid B-reps whose meshes are not watertight, as reported.
- **Poses a–g:** 0 interference except a 0.92 mm³ thenar graze. The wrist grid is clear, with 1.8 + 2.3 mm³ at
  +50/+20, as reported.
- **Wrist bundle:** min tube R **18.8 mm** at the extremes. **Forearm fan:** min R 11.3, 5 tubes < 15, 4 gaps < 0.4
  mm: as reported, and honestly listed as open.
- **M4 push rods do not buckle** (≈ 1.2 kN at the root against ≈ 470 N).
- **The routing decoupling holds for rigid-body geometry:** both sheath ends are fixed and the strand stays inside.
  This holds only under the assumptions that #1, #4 and #9 break.

## Claims not verified, or contradicted

- **"Wrist motion does not move the fingers"**: true for rigid, incompressible sheaths. Not shown with the real fan
  (#4), sheath compliance (#1) or bore clearance (#9).
- **Force numbers (3.4 N finger, 5.5 N thumb, 8.7 / 14 N with HLS)** use η = 0.9. The routed wrap angles give
  0.45–0.75 (#2). The HLS numbers also assume parts that fail at 118 N (#7).
- **"Flex ≥ 3.6 N·m stall, ≥ 1.1 N·m continuous"** (wrist): the gear stage and crank as modelled cannot carry it (#3).
- **"Knot sees ≈ 25 %"** (#10). **"Glue" for the PTFE clamp** (#6).
- **Servo data from memory:** STS3215 continuous torque, shaft offset, HLS3606M torque, height and range,
  FE-URT-1 size, OpenArm J5 flange. All are still marked "verify".
- **Eyelet error 0.19 mm, nip-pin error ≤ 0.123 mm:** these are analytic. Rim tolerance, braid flattening and the
  0.36 mm line on a 0.43 mm rim are unmeasured, and the reports themselves ask for the bench test.
- **Strand replaceability / "nothing is restrung" on a servo swap:** plausible for the servo swap (split spool
  stays on the rail). Not true for a strand swap (#8).

## What I would do before the first print (order)

1. A sheath bench test (#1): 300 mm of PTFE 1 × 2 against PEEK 1 × 2 against a lined coil, 5/15/45 N, shortening
   and creep over 24 h. This decides the whole architecture.
2. An eyelet / nip-pin cycle test (#5) with the real braid.
3. Fix the sheath end stops (#6) and the wrist gear/crank sizing (#3). Both are cheap.
4. Re-run the force tables with real η, and decide SCS vs HLS as one load class (#2, #7).
5. Only then re-route the fan with the real take-up (#4).
