# 10 · Adversarial review of the V2.1 build phase (reports 07, 08, 09)

Agent 10, 2026-11-15. Study only. I wrote only this file and `cad/10_xcheck.py`, a numpy-only cross-check that
reads `out/07/*.json` and `out/09/eyeplate_kin.json`. Run it with `python3 cad/10_xcheck.py`; it writes nothing.
Every number below comes from that script, from the agents' own scripts and outputs, or from a measurement of
the exported STLs. Where a number is quoted from memory, I say so.

## Verdict

**The bench finger (07) is worth printing after three changes. V2.1 does not yet close as a whole hand.**

- **07 is a good design with one wrong sign in its force budget.** The pip flex strand crosses the MCP on the
  idler's dorsal side, which makes it an MCP **extensor** (+4.235 mm moment arm, read from 07's own
  `checks.json`). The force budget leaves this term out, so:
  - the headline pinch is **5.6 N, not 6.8 N**, on SCS0009, and **11.6 N, not 17.6 N**, on HLS3606M;
  - the MCP, not the PIP, is the limiting joint.
- **07 and 09 used different travel models, and neither checked the other's.**
  - 09 assumed the pips need 200° of their own travel; 07's geometry needs 235°.
  - 07 assumed 3.6 mm of wrist compensation; 09 computes 3.3–12 mm.
  - 07 moved the MCP spool to r 4, but 09 assumed r 5.
  - Combined: **8 of 16 servos exceed SCS0009's travel (worst 374°)**, and no eye assignment on 09's grid can fix it.
- **Integration gaps for the hand.**
  - The knuckle module (07's base, 32 mm wide) does not fit the 19.5–20 mm finger pitch.
  - 09's idler block sits inside 07's yoke.
  - The palm reorder block is not designed.
  - The bare-strand architecture now pushes **~0.45–0.8 kN (SCS) and ~1.7 kN (HLS)** of tendon tension through the
    wrist cardan. That is the V2 cardan: 623 bearings on Ø3 pins. Nobody checked it, because with sheaths the
    sheaths carried that load.

## Ranked findings

Severity: **C** = a headline claim or decision is wrong; **H** = it blocks the hand or a part fails;
**M** = fix before the full build; **L** = polish.

| # | Sev | What | Evidence | Fix | Effort |
|---|---|---|---|---|---|
| 1 | **C** | The pip flex strand is an MCP extensor, and the budget ignores it. | `out/07/checks.json`: pip flex slope +0.0739 mm per MCP-degree, a +4.235 mm arm. `07_budget.py` §3 has no ρ term; 01's `tip_force()` had ρ but assumed a palm-side (helping) crossing. With the term, pinch is SCS **5.6 N** (07: 6.8) and HLS **11.6 N** (07: 17.6); straight push is SCS 3.5 (4.8) and HLS 7.4 (10.1). The MCP limits in every case. | Cross the pip flex strand on the **palm side** of the idler, with an **MR63 roller** as the redirect, not a Ø3 dowel. That restores 6.7 N (SCS) and 17.6 N (HLS), and the MCP then has 3.6× margin. A Ø3 dowel redirect on the flex strand would cost η 0.79, giving 5.6 N. Move the ext strand to the dorsal side; it carries only 10–20 N. | CAD: one roller and the strand swap in the yoke, ~1 day |
| 2 | **C** | The servo travel budget does not close on SCS0009 when 07 and 09 are combined. | `10_xcheck.py` §3: index pip 314°, index MCP 301°, middle MCP 374°, ring MCP 348°, little MCP 353°, all 4 pips 298–314°, against 285° (07's margin) or 290° (09's). 09's "worst 279°" used pip own travel 200° (07's geometry: 235°), MCP on r 5 (07: r 4) and 0.60 idler coupling (07: 0.847 servo-deg per MCP-degree). 07's flat 3.6 mm wrist span is 1–3× too small (09: 3.3–12 mm). The 17 point-symmetric eye pairs have wrist travels 21, 37, 41, 58… on r 5. The 8 pip/MCP joints have 45–50° of spare travel, and only 3 pairs fit: **infeasible for any assignment**. | (a) MCP spool back to **r 5** (with fix #1 the MCP no longer limits the pinch, so this costs only straight push). (b) Cut pip own travel: an MR63 idler on a stepped Ø4/Ø3 axle (0.447 coupling) gives ~215°. With (a) and (b), all 8 fit on the 7 × 5 grid. (c) One shared table for all couplings and signs, used by both the CAD and the firmware. | Re-run 09's assignment with 07's own travels: ½ day |
| 3 | **H** | The wrist cardan now carries the tendon sum, unchecked. | §8: compression across the wrist is ~775 N (SCS 80 %, all 16 flex strands), ~450 N (a realistic power grasp) and ~1670 N (HLS 80 %). That is 225–835 N per 623 bearing against C0 ≈ 220 N (from memory): s0 0.26–0.98. The V2 Ø3 wrist pins were already flagged as weak in V2's `params.py:113`. The eye plate also sees **~670 N in-plane** at wrist 50° (SCS), carried by M1.6 screws in printed bosses. | A wrist rework is required, not optional: Ø4–5 pins, 624/684 or larger bearings, a metal ring or a PAHT-CF ring with metal bosses, and M2.5–M3 for the eye blocks. Add a check to 09: axial load path and eye-plate mounting. | Wrist redesign, 2–3 days |
| 4 | **H** | V2.1 knuckle and palm do not close. | 07's base is x ±16 (32 mm) and carries the abd lugs, the bridge and 4 cantilever dowels pressed into side walls at \|x\| 16, but the finger pitch is 19.5–20 mm (V2 `params.py`). In hand coordinates, 09's `index_idler_block` (x 11.3–20.6, z 77.5–83.5) lies inside 07's yoke envelope (x 11–29, z 71.5–98) and inside 07's base volume. 07 moved the abd axis to 13 below the MCP (V2: 11.5). 09's trough (z 26–71) overlaps 07's base (z 38–84). | Drop 09's idler block: 07's yoke abd-plane dowels already do its job. Design one **palm knuckle module** per finger (07's base guides at ≤ 19 mm width, shared walls between neighbours) and give 09 a defined strand-exit table at its bottom. Then design the reorder block (30–40 near-crossings). | 2–3 days |
| 5 | **H** | Fuse cup: 07 shrank the chamber but kept 03's trip load. | `07_finger.py:234`: chamber Ø1.5. 03 had Ø1.8 (`03_drum_fuse.py:49`). With a Ø1.5 bead: trip **56.5 N** (flat t 0.8), which equals the SCS MCP stall strand of 56.5 N (r 4). Trip/stall ratio 1.00 (MCP), 1.25 (pip): ±25 % print scatter trips cups at stall. The BOM's "1.3 mm crimp sleeve" for a doubled 0.47 line (≥ 0.94 ID) cannot have an OD ≤ 1.5, and crimping flattens it wider. The cup sees 72–89 % of the strand (only 40–56° of drum wrap; SYNTHESIS #4 wanted ≥ 1 turn, i.e. ~28 %). An external push of **~9–11 N** on a holding fingertip trips the pip fuse. | Size the chamber to the real crimped bead (measure it). Set the trip by lip thickness at ≥ 1.5 × the 80 %-capped strand. Add a crimp-slip test (doubled PE#8 in a sleeve ≥ 1.5 × trip). Decide whether the fuse protects the servo gears (strip torque unknown, see #11) or the finger. | Small |
| 6 | M | The MCP axle wire slot weakens the axle where the bending is highest. | Slot box x −3.6…−2.4, z −2.1…−1.0, inside the tongue cheek. Section modulus 4.30 → 2.62 mm³ (61 %). HLS stall: 209 MPa nominal, ~420 MPa with Kt 2, against ~500 MPa yield for hard-drawn 304 (fatigue far lower). SCS stall: ~160 MPa with Kt. 07 reported 129 MPa (unslotted). | Put the slot at the neutral axis for the dominant load (±y, not −z), or route the wires through the axle end, or use a solid hardened dowel with the wire beside it. | Small |
| 7 | M | Hall magnets sit inside steel bearing races. | The MCP magnet (Ø3 × 2) is at x 5.2–7.2 inside the axle, which is inside the MR84 inner race (seat x 4.4–7.4). The PIP magnet is likewise in the pin end at the MR84. Chrome-steel races and steel shields concentric with a diametric magnet shunt and distort its field. Joint sensing is "required" (SYNTHESIS #12). | Put the magnet ≥ 2 mm proud of the race on a brass or plastic stub, or test the angle error on the bench first (an AS5600 or TMAG5273 sweep against a protractor). | Small |
| 8 | M | Abd crossing: the fleet angle is ~15°, not 4°, and it bends over printed edges. | The abd-plane dowels lie along x and rotate with the yoke. The strand below them is fixed to the base, so the skew between strand and dowel normal is asin(cos 10° · sin 15°) = **14.7°** at ±15° abd. 07's slots flare at 17° (`07_finger.py:596`), so the x-bend pivots on the printed slot neck: a sliding bend on plastic (against SYNTHESIS #2), with a lateral force of ~0.25 T. Pip lanes are only 2 mm apart on a flangeless MR84 race. | Add a steel x-guide (a Ø3 pin or roller with its axis along y) at the neck, or lay the strands in on a vertical pin pair on the abd axis. Bench test 07 risk 4 is the right test; add the 15° number to it. | Small CAD + bench |
| 9 | M | Loop pretension cannot survive creep. | 09: loop stiffness ~21 N/mm, so 5 N of pretension = **0.24 mm** of loop length. 08 T2 passes creep ≤ 0.2 % in 24 h, which is 1.1 mm on a 533 mm loop (all pretension gone 4× over). Ring/PETG creep, crimp bedding and braid creep each reach 0.1 mm or more. | Change T2's pass to a per-loop number (≤ 0.1 mm per week at 5 N on a full loop). Add a **bottoming pretension spring** at each spool (5 N preload, goes solid at ~6 N; ~10 mJ stored, safe) or a scheduled re-tension procedure. | Small |
| 10 | M | Bench kit (08) predates 07/09 and tests the wrong thing first. | T1 (sheath compression) ranks first, but sheaths are only the fallback now. 09's eye-plate test (09 §8, the main architecture risk: 160–200 MPa lip at SCS, 418 at HLS) is "not in the kit". There is no integrated finger test, no crimp test, no Hall-near-bearing test. T5 uses 03's cup (Ø1.8, 0.4–0.8, "on its side"), not 07's (Ø1.5, flat 0.8). T9 uses 03's ring (ID 12.64), not 07's (OD 14). T7 passes at η ≥ 0.85, but the budget assumes 0.95, so a "pass" still breaks the 6.8 N claim. T8's pass (hold 0.21 N·m) conflicts with the SCS 80 % back-off. 07's pre-stretch (133 N for 24 h) conflicts with 08's 100 N hanging-load safety limit. | Reorder (see "Pre-print order"). Tie each pass to the budget number it protects. | Edit the plan |
| 11 | M | Decisions that rest on values from memory. | (a) **SCS0009 300°**: decides #2 and the r 4 spool. (b) **HLS3606M multi-turn**: the whole "HLS solves travel" fallback (07, 09) depends on it. (c) **MR84 C0 139 N**: s0 0.74 at HLS stall; it stays below 1 even at C0 180 N, so the decision (80 % torque cap) is robust. (d) **623 C0 ~220 N**: #3. (e) PE#8 0.47 mm / 445 N: typical PE#8 ratings are 80–100 lb (350–445 N); this does not change a decision at SCS, and the HLS line safety factor drops from 3.7 to ~2.9. (f) SCS0009 **gear strip torque** (unknown) sets the fuse level. (g) 14 × 0.5 brass tube availability: verify, because 03/08 use a different ring. | Measure (a), (b), (f) on day 1; (f) with one sacrificial servo. Read (c), (d) from a real NSK or Misumi PDF. | 1 day |
| 12 | M | Joint friction left out of η. | The PIP −x brass bush on the stainless pin, plus the two DIP bushes, plus the bar pins: 2.4–4.8 % loss at the PIP bush alone (μ 0.15–0.3), ~3 % more from the DIP. A realistic bench η is ~0.90, not 0.95. | Grease the bushes and carry η 0.90 in the budget. T7 measures it. | Trivial |
| 13 | M | Anaerobic retaining compound (641) on PETG. | 07's assembly uses 641 for rings, sleeves, bearing seats and the axle in PETG. Anaerobics barely cure on plastic without an activator, and are known to stress-crack amorphous thermoplastics (PC/ABS/PETG class). | Use 641 metal-to-metal only (axle in the brass sleeves). In plastic seats use epoxy or a CA/primer that is safe on the plastic, or a retaining lip. Test it on the 04 coupon. | Trivial |
| 14 | L | Printability is optimistic. | STL face analysis. Proximal on its −x face: only 163 mm² touches the bed (MCP lug); the bar face (257 mm² at x −6.85) floats 2.15 mm up; 990 mm² of > 45° overhang. The bar slides 0.2 mm from that face, so support scars will rub. Middle: hub and DIP-tongue faces 4.2 mm up (2 × 120 mm²); 410 mm² of overhang. Yoke "on its side": 25 mm² of bed and 610 mm² of overhang; dorsal-down gives 126 / 310. | Add a 2.15 mm sacrificial raft pad under the bar face, or print the proximal in two halves. Print the yoke dorsal face down. Sand or face the bar running surface. | Small |
| 15 | L | Firmware coupling signs. | `checks.json` says pip_per_mcp **−0.585**; report 07's text says "pip **+=** 0.585 q_mcp"; 09 uses **+0.60** (02's palm-side model) and G = 1.4 (07: 1.447 / 1.809). A wrong sign doubles the error (~100° of PIP at MCP 90). | One coupling table (signs + G) generated from the CAD; verify it with the Hall sweep before the hand. | Trivial |
| 16 | L | BOM is pro-rated, not what the owner pays. | €54.8 counts fractions of packs. Missing items: the two custom PCBs (JLCPCB setup + assembly + shipping ≈ €25–40), a Dragon Skin trial kit (~€30–35), Loctite/epoxy (~€10), 3 spool cartridges (02) and an r 4 cartridge variant (not designed), 3 servos, the 2020 frame, the 0.25 mm wire leader. | First-buy cost for one bench finger is ≈ **€130–160 plus servos**. Say so in the plan. | Trivial |
| 17 | L | Fuse cups have no retention when slack. | Cups sit in radial or tangential pockets held only by strand tension. | A small snap rib or a dab of glue. | Trivial |

## Claims verified (re-derived)

- **Arithmetic of 07's budget:** with 07's inputs I get the same η (0.954 pip flex bench, 0.959 MCP), the same
  strand forces (45.2 / 56.5 / 118 N) and the same tip forces with ρ = 0. The error is the missing term (#1),
  not the arithmetic.
- **Levers 43.1 / 46.2 mm:** 01 defines them as the moment of a pad-normal force; "pip_drive" = PIP + 0.75 × DIP
  by virtual work. Consistent.
- **Sign convention:** `bench_finger_pinch.png` (MCP 30 / PIP 45) bends toward −Y (left in the side view, the
  green +Y axis points right), and the silicone pad faces the palm side. The cut view agrees. The strand slopes
  in `checks.json` have the right signs for own flexion (−0.126 mm/° on every joint).
- **Abd loop invariance:** loop-length variation 0.021 / 0.029 / 0.000 mm in `checks.json` (target 0.036).
- **DIP linkage:** 0.43° max error and 36° minimum transmission, from 07's own run of the repo's `dip_linkage`
  (not re-run, to avoid overwriting 07's outputs).
- **MR84 s0:** 139 / 188 = 0.74 at HLS stall (lug), as claimed.
- **Eye lip pressure:** T/(r·d) = 160 MPa (SCS, r 5) and 418 MPa (HLS), as 09 claims. It is **200 MPa** on the r 4
  MCP strands, which 09 did not consider.
- **Pivot-offset law:** for a pivot h off the axis plane, the length gain is ≈ h·θ²/2. I derived it
  (|A′ − P|² = L² + h² − 2Lh cos θ). At h 1.25 and 50° it gives 0.95 mm per pair ≈ 20 N, matching 09.
- **09's wrist travel table:** it fits T ≈ 9.7|x| + 17.4|y| servo-degrees on r 5 (09 says ≈ 10|x| + 20|y|).
- **DIP bar and pins:** 74 MPa net section at 30 N tip; 16.3 MPa PETG edge pressure at 30 N, but only ~4 MPa at the
  real SCS pinch. OK.
- **Drum PETG under the ring:** 11.3 MPa at HLS stall, 5.3 at SCS. OK, but it rests on the 12 MPa creep value
  quoted from memory.
- **Fuse lip formula:** π·d·t·τ is right, but with the wrong d (#5).

## Claims not verified

- 09's exact eye-plate model (2.9 / 3.9 N swing, ≥ 0.85 mm strand gap, 0 interferences). I only checked the
  physics behind it.
- 07's OCC interference and strand-in-solid checks (I trust the method; I did not re-run them, ~90 s plus
  CadQuery).
- That a strand can actually be laid into the open +x proximal grooves and onto the MCP rings in the stated 5 min.
  The MCP ear slots are closed windows, so feeding end-first works, but the crimp must be made outside and fed
  back. The MCP ring is reached across a 1.2 mm core band (no inner flange); plausible, untested.
- PETG creep 12 MPa, braid μ 0.12 on steel, crimp hold 70–90 %, Dragon Skin fit: all from memory or untested.
- Hall angle accuracy, wire fatigue in the hollow pins, eye wear: bench only.

## What is missing for a whole V2.1 hand

1. A palm knuckle module at ≤ 19 mm width that replaces 07's bench base and 09's idler block (#4).
2. The palm reorder block (30–40 near-crossings) and its fixed bends in η.
3. The thumb: routing through the palm plate (68–96° turns need a roller) and its finger mechanics.
4. A wrist that carries 0.5–1.7 kN of axial compression and a 0.7 kN eye-plate load (#3).
5. An r 4 (or, after fix #2, no r 4) spool cartridge, and the cartridge width (14.3 mm) against the cassette
   pitch. That conflict is still open from SYNTHESIS.
6. One coupling and travel table shared by 07, 09 and the firmware (#2, #15).
7. Pretension take-up (#9).

## Pre-print order for the owner

1. **Day 1, measure, don't print:**
   - the SCS0009 range (send 0 and 1023, measure with a protractor);
   - whether HLS3606M has a multi-turn mode (datasheet);
   - SCS0009 gear strip torque (one sacrificial servo on the 08 R5 lever: add water until it slips or strips).

   These three numbers decide #2 and #5.
2. **Ask agent 07 for two CAD changes before printing:**
   - pip flex on the palm side over an MR63 redirect roller (#1);
   - MCP spool r 5 and the wire slot moved (#2a, #6).

   Also: cup chamber sized to a measured crimp, magnet proud of the race, yoke printed dorsal-down (#5, #7, #14).
3. **Print the 04 coupon.** Test the seat fits with epoxy, not 641, on PETG (#13).
4. **Bench tests, in this order:**
   1. T3 μ;
   2. crimp-in-cup slip plus the T5 trip with **07's** cup (pass: trip ≥ 1.5 × the capped strand, CV < 15 %);
   3. 09's eye-plate test (09 §8). This is the architecture decider: kill if fraying in < 10 k cycles or a swing
      > 10 N, then use the coil fallback;
   4. T2 with a loop-length pass (#9).

   Demote T1 (sheaths) to "only if the eye fails".
5. **Print and build the bench finger.** Pass/kill numbers:
   - Pinch at SCS 80 %: pass ≥ 4.5 N; below 3.5 N, kill and re-check η.
   - Hall angle error: < 2° over the range.
   - Abd ±15° × 10 k sweeps: no lane crossing.
   - Loop pretension after 1 week: 5 ± 2 N.
6. **Only then:** the palm knuckle module, the reorder block and the wrist rework (#3, #4), and re-run 09 with 07's
   travel numbers.

## Sources

Reports and outputs 01, 02, 03, 07, 08, 09 in this folder (`out/07/checks.json`, `budget.json`, `bom.csv`, STLs,
renders; `out/09/eyeplate_kin.json`). `cad/07_budget.py`, `07_finger.py` (lines 74–77, 213–250, 586–637,
841–848), `09_eyeplate_kin.py` (lines 53–80), `03_drum_fuse.py:49`, `01_force_budget.py:222–262`.
V2 `cad/params.py` (finger pitch, ABD_OFFSET 11.5, wrist pins line 113). Bearing and servo values from memory are
marked in the table.
