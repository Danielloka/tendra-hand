# 02 · Tendon drivetrain parts: from servo spool to joint drum

Agent 02, 2026-11-14. Scope: the parts that carry the tendon. That is the line, the housing (the Bowden
"outer"), the guides at bends, the anchors, the spool and the tensioner, and how a strand gets replaced.
Joints, drums and the actuator choice belong to other reports. Here they are inputs only.

Read: BRIEF, `CLAUDE.md`, `research/references/hands/tendon-mechanics.md`, the V2 study (README, reports 05,
06, 07; `cad/params.py`, `forearm.py`, `finger.py`, `routing.py`, `out/assembly/routes.json`), and the top
recommendations of reports 01 and 03 of this study.

Scripts (all in this folder):
- `cad/02_drivetrain_calc.py`: every number in this report (`uv run --no-project --with numpy python cad/02_drivetrain_calc.py`).
  The printout is in `out/02/drivetrain_calc.txt` and the values in `out/02/drivetrain_calc.json`.
- `cad/02_spool_cartridge.py`: CadQuery prototype of the spool cartridge (`uv run --no-project --with cadquery --with numpy python cad/02_spool_cartridge.py`).
  It writes `out/02/spool_cartridge.step` (the assembly), `spool_02.stl`, `driver_02.stl`, `keeper_02.stl`
  and `spool_cartridge.json`. All parts are valid and watertight, and nothing overlaps.
  `cad/02_render_sections.py` draws `out/02/spool_cartridge_sections.png`.

Load classes used throughout. Strand tension at servo stall through the 5 mm spool:
**SCS0009: 45 N. HLS3606M: 118 N** (the HLS torque is from memory, so verify it). Loop pretension T0 = 5 N per strand.
1 mm of line on the R 7 drum = 8.2° of joint angle.

Some terms used below:
- **Capstan law:** when a line slides around a curve, the tension on the far side is T·e^(−μθ). Here μ is
  the friction coefficient and θ is the wrap angle in radians.
- **D/d:** the bend diameter divided by the line diameter. Below about 8, a fibre line loses strength and frays.
- **Housing:** the outer tube of a Bowden cable. It carries the full line tension, in compression.

---

## 1. Top 3 recommendations

**1. Every bend goes over hard, polished metal (or a roller) with r ≥ 1.5 mm, on a PE#8 (≈100 lb, 0.47 mm)
UHMWPE braid.**
The 18G eyelet rim (r 0.43 mm) presses the line at **224 MPa at SCS stall and 584 MPa at HLS stall**, at
D/d 1.8. That is the fraying point the review flagged. A Ø3 ground dowel pin (ISO 8734, about €0.05) cuts
this to **64 / 167 MPa** at D/d 6.4. An MR63ZZ roller cuts it to **32 / 84 MPa** and drops the friction from
17 % to **0.8 % per 90°**.

Rules:
- Never let the line bend over PTFE, brass or bare print at a point contact. They creep or wear at 19–84 MPa.
- A "free" needle roller turning in printed holes **does not roll**. The line's friction torque (μ 0.15 · r)
  is smaller than the journal's (μ 0.25 · r), so the line just slides over it. Only real bearings roll.
- At the MCP, carry the pip strands over a **coaxial MR63 idler** instead of the eyelet. This gives a fixed,
  exactly linear coupling of 0.43° of pip per MCP degree. The firmware cancels it by adding 0.60 · q_mcp to
  the pip servo, which costs 60° of extra servo travel. In return there is no slack, rolling contact and D/d 12.8.
- PE#8 gives SF 3.8 at HLS stall and is 2× stiffer than the 50 lb baseline. Together these raise the
  efficiency of the worst finger route from **0.59 to 0.85** and the fingertip force from **3.1 to 4.5 N**
  with SCS0009 (8.2 → 12 N with HLS).

**2. Build the spool as a cartridge (prototype in `out/02/`).**
- **One-piece spool** on a **fixed Ø3 hardened axle** with **2 × MR63ZZ**, so the servo only gives torque.
  It drives through two Ø1.5 dog pins that float radially by 0.3 mm. Without this the servo shaft takes a
  50 / 123 N side load, which is a 0.4 / 1.0 N·m bending moment on the shaft.
- A **chordal M2 screw tensioner** in the hub replaces the split spool, the serrations and the friction clamp:
  - 0.4 mm of line per turn. 1/8 of a turn ≈ **0.9 N** of pretension.
  - **5.5 mm of travel**, more than the expected ~3 mm of UHMWPE creep.
  - It is positive and self-locking. The V2 M2 friction clamp holds only **1.1×** the HLS stall torque.
- The line wraps **≥ 1 turn** before each anchor, so the anchor sees ≤ 28 %.
- A **keeper ring** sits 0.2 mm from the flanges, so a slack line cannot jump off.
- A **witness notch** shows the spool angle.
- The spool weighs about 1.3 g of PETG. It costs MR63 €0.3 × 2 + a dowel + an M2 screw and nut.

**3. If any housing remains, use a PTFE-lined closed steel coil with metal end stops, never plain PTFE or PEEK.**
Report 01 proposes deleting the housings (bare straight strands, and firmware cancels the wrist). I agree,
and the guide parts in rec. 1 make that buildable. Where a housing is still needed (a fallback wrist, or the
thumb), the numbers are:

| Housing (per strand, at SCS / HLS stall) | Shortening |
|---|---|
| PTFE 1 × 2, 221 mm (V2) | **8.5 / 22 mm** |
| Steel coil 0.3 × 2.0, 140 mm | **0.33 / 0.48 mm**, including a bedding-in allowance |

- PEEK 1/16" tube is not the answer. It still shortens 2.3 / 6.1 mm, and it yields in a R 15–19 mm bend (outer strain 4.2–5.3 %).
- The coil ends must sit on **metal**. A coil end on a printed step presses the plastic at 28 / 74 MPa. On
  an M1.6 washer it is 4.4 / 11.5 MPa, and on a shared drilled aluminium stop plate it is 4.6 MPa.

---

## 2. Idea list (wide)

The gain is per strand unless noted. Verdicts: ✅ do, 🔬 test first, ❌ drop.

### Line

| # | Idea | How it works | Gain (number) | Cost / effort | Risk | Verdict |
|---|---|---|---|---|---|---|
| L1 | PE#8 UHMWPE braid (0.47 mm, ~445 N) | Thicker 8-carrier braid | EA 11 kN vs 5.5 kN. Stretch at SCS stall over 310 mm: 1.3–2.1 mm (was 2.5–4.2). SF 3.8 at HLS (was 1.9) | €15 per spool | D/d drops 20 % everywhere | ✅ |
| L2 | PE#10 (0.52 mm, ~580 N) | Thicker again | Stretch 1.0–1.6 mm | same | D/d 5.8 on Ø3 pins | 🔬 HLS only |
| L3 | Vectran 0.5 mm braid | LCP fibre, near-zero creep | Creep ~0.2 % vs ~1 % (0.6 vs 3.1 mm of take-up) | €20–30 | Abrades faster on guides | 🔬 run a creep test next to L1 |
| L4 | Dyneema SK99 / DM20 line | Higher strength / low-creep grade | SK99 ≈ +20 % strength. DM20 creep ≈ 0 | Hard to get under 0.5 mm | Availability | 🔬 if creep test fails |
| L5 | Kevlar | Aramid, no creep | — | cheap | Poor bending fatigue at D/d < 20, self-abrasion | ❌ |
| L6 | Steel 7×7, 0.45 nylon-coated | Wire rope + crimps | EA ≈ 10 kN, cannot creep | cheap | Wants D/d ≥ 25 (it gets 6–22). Only 200 N | ❌ (only for a straight forearm run) |
| L7 | Pre-stretch the line | Hang 30 % of break for 24 h before cutting | Removes most of the initial bedding (~1–2 %) | free | none | ✅ |

### Housing

| # | Idea | How it works | Gain (number) | Cost / effort | Risk | Verdict |
|---|---|---|---|---|---|---|
| H1 | No housing: bare straight runs + guides (report 01) | Only rigid structure between the guides | Housing compliance 8.5 → 0 mm. Palm turning median 96° → 16° (straight from an aimed stop plate; 0° if both ends aimed) | Deletes ~70 parts | Wrist coupling must be cancelled in firmware (≤ 5.6 mm, 46 joint-° worst on a 1.9 mm-pitch eye grid) | ✅ (with rec. 1 parts) |
| H2 | PTFE-lined closed steel coil | Coil turns touch, so axial load goes steel-on-steel | 0.12 mm (Hertz) + ~0.2 mm bedding per 140 mm at SCS stall | €5–10 per 32 m | Seller sizes vary. Bend shifts the path ≤ ~0.5 mm | ✅ fallback |
| H3 | Bike shift housing (SP41-type, compressionless) | Lengthwise wires + liner | ≈ 0 shortening | €1 per m | 4 mm OD, stiff in bending (R ≥ 40) | ❌ too big for 32 strands |
| H4 | PEEK 1/16" HPLC tube | Stiffer polymer | 3.6× PTFE (2.3 mm at SCS) | €3–30 per m | Yields in R 15–19 bends | ❌ |
| H5 | PTFE liner loose in a rigid printed channel | The palm carries the bend force. The liner is not in the load path when both stops are on the same rigid part | Removes the palm from the compliance | free | Only valid inside one rigid part | ✅ where a liner is still wanted |
| H6 | Shared aluminium stop plate per bundle end | 2 mm 6061, 32 × Ø1.6 drilled through a printed jig | 4.6 MPa on PETG (vs 28–74 MPa for a coil on a print) | €2 | Drilling accuracy | ✅ with H2 |
| H7 | Brass-tube ferrule + M1.6 washer (ISO 7089 1.7/4.0/0.3) | Positive stop for single tubes | 4.4 / 11.5 MPa on PETG | cents | Needs Ø4 of space per strand | ✅ for single strands (thumb) |
| H8 | Bent stainless hypodermic tube (16G dispensing needle) | A rigid curved guide | Incompressible, μ ≈ 0.1, bend R ≥ 5 | €0.10 each | Burr at the cut end: deburr and flare | ✅ for short fixed thumb bends |

### Guides at bends

| # | Idea | How it works | Gain (number) | Cost / effort | Risk | Verdict |
|---|---|---|---|---|---|---|
| G1 | Ø3 ground dowel pins (ISO 8734 m6) | Fixed, polished, hardened | p 64 / 167 MPa, D/d 6.4, loss 17 % per 90° | €0.05 | none | ✅ default |
| G2 | Ø4 dowel | Same | D/d 8.5, 48 / 126 MPa | €0.05 | Space | ✅ where it fits |
| G3 | MR63ZZ roller (3 × 6 × 2.5), flanged by washers | Rolling | 0.8 % per 90°, D/d 12.8. C0 = 74 N | €0.3 | Wide (2.5 + flanges). Static rating limits HLS at large wraps | ✅ at big fixed wraps (MCP idler, thumb cmc_rot, spool exit) |
| G4 | 681ZZ (1 × 3 × 1.5) roller | Tiny rolling guide | 1.2 % per 90° | €0.5 | Low C0 (verify) | 🔬 |
| G5 | Ceramic textile thread-guide eyelets (alumina, polished, flared) | Made for running yarn. Press-fit in a plate | μ ≈ 0.1–0.15, lip r ≈ 0.5–1 mm (verify), any direction | €0.1–0.5 | Lip radius is small (D/d 2–4) | 🔬 for the wrist eye plate (H1) |
| G6 | SiC fishing-rod guide rings | Designed for braid | very low wear | €0.3 | Ring section r ≈ 0.75 → D/d 3.2 | 🔬 alternative to G5 |
| G7 | Brass tube sleeve over a print | Soft metal liner | 64 MPa, but brass allows ~150 | cents | Grooves at HLS (167 MPa) | ❌ for HLS |
| G8 | PTFE insert at a bend | Low μ | μ 0.07 | cents | p 32–84 ≫ 6 MPa → creeps | ❌ (only for bends with R ≥ 20) |
| G9 | Free needle roller in printed holes ("rolling nip", report 06) | — | **Does not roll**: μ_line·r < μ_journal·r | — | False sense of rolling | ❌ (or make it a fixed pin) |
| G10 | Brass tube roller on a Ø1.5 steel axle | Roller radius 2× the journal radius | Rolls, effective μ 0.075 | cents | Needs space | 🔬 |
| G11 | Two-pin nip on the joint axis | Line held between two pins | Abduction (±15°): ±0.006 mm error. MCP at 90°: ≥ 1.2 mm common mode | cents | Good only for small angles | ✅ abd crossing, ❌ MCP |
| G12 | Coaxial idler at the MCP for the pip strands | Classic tendon-hand layout | Exactly linear 0.43°/°, no slack, rolling | 2 × MR63 | Needs 60° of extra pip servo travel (140 → 200°; check the SCS0009 range) | ✅ owner decision |

### Anchors

| # | Idea | How it works | Gain (number) | Cost / effort | Risk | Verdict |
|---|---|---|---|---|---|---|
| A1 | ≥ 1 full turn before every termination | Capstan relief | Share 87 % (V2 drum, 40° margin) → 28 % (1 turn on PETG) → 15 % (1.5 turns) | free | Capstan relief is an upper bound under cycling | ✅ |
| A2 | Helical drum groove with 1 extra turn | Same on the joint drum | 87 % → 25 % at the knot | +0.6 mm of drum width | Axial travel 0.17 mm | 🔬 vs report 03's fuse cup |
| A3 | Steel capstan post, 3–5 turns | Wrap a Ø3 pin, then tie | 15 % / 4 % | Space | Room in the finger | ✅ spool / forearm only |
| A4 | Crimped beads / sleeves (report 03 fuse cup) | Positive stop, no knot | Crimp on doubled braid ~70–90 % (verify) | cents | Braid slips in a single crimp: double it back | ✅ |
| A5 | Figure-8 + CA stopper in a pocket | Knot as a stop | 50–80 % knot strength | free | Weakest. Fine after A1 relief | ✅ (spool ext land, after ≥ 1 turn) |
| A6 | Burnt "mushroom" end of braid | Melt the tag | Unknown | free | UHMWPE melts at 145 °C into a weak blob | ❌ |
| A7 | Ball end: solder bead | — | — | — | Solder at 250 °C melts UHMWPE | ❌ |
| A8 | Cam cleat / wedge clamp | V-jaw locks the line | Adjustable, no knot | Small print | Braid is slippery: the cleat creeps | ❌ |
| A9 | Set screw on the line | — | Adjustable | cents | Cuts the braid. Only on a wrapped post | ❌ |

### Spool and tensioner

| # | Idea | How it works | Gain (number) | Cost / effort | Risk | Verdict |
|---|---|---|---|---|---|---|
| S1 | Spool on a fixed axle + 2 bearings, dog drive | Servo gives torque only | Servo side load 50 / 123 N → 0 | €0.7 | MR63 C0 = 74 N: SF 2.9 (SCS), 1.2 (HLS) → use 683ZZ / MR74ZZ for HLS | ✅ |
| S2 | Chordal M2 screw + square nut tensioner | Screw pulls the anchor nut along a chord | 0.05 mm / 0.9 N per 1/8 turn, 5.5 mm of travel, self-locking | €0.1 | Reach the head at a service angle | ✅ (built) |
| S3 | Split spool + friction clamp (V2) | Clamp face friction | M2: holds 0.67 N·m = 1.1× HLS stall | free | Slips at HLS. Setting moves while tightening | ❌ |
| S4 | Serrated split spool | Teeth | 36 T → 15.7 N steps, 60 T → 9.4 N | free | Far too coarse | ❌ |
| S5 | Vernier pin lock (12 / 13 holes) | Pins through matching holes | 0.2 mm / 3.6 N steps, positive | free | Pin bearing on PETG 55 MPa at HLS. No room on Ø13 | ❌ |
| S6 | Printed worm on the spool rim | Worm + wheel | Self-locking, fine | — | Tooth load 2T ≈ 240 N on a printed 0.5 mm pitch | ❌ |
| S7 | Pretension spring with a hard stop on the extensor | Spring keeps the extensor taut, then bottoms out | Takes up creep automatically | €0.1 | Adds a deadband when the extensor is loaded. Small energy (5 N × 3 mm = 15 mJ, safe) | 🔬 (tendon-mechanics §3.4) |
| S8 | Keeper ring, 0.2 mm gap | A slack line cannot leave the groove | Prevents derailing | free | none | ✅ |
| S9 | Flat-bottom land 1.6 mm wide | Braid flattens. Room for ~2 turns at pitch 0.61 | No pinching (a V-groove pinches and saws the braid) | free | — | ✅ |
| S10 | Witness notch + index rib | See creep at a glance | — | free | — | ✅ |
| S11 | Service angle in the firmware | Servo turns the spool so the tensioner head faces the access window | Tension without teardown | free | — | ✅ |

### Strand replacement

| # | Idea | How it works | Gain (number) | Cost / effort | Risk | Verdict |
|---|---|---|---|---|---|---|
| R1 | Straight runs + a removable palm lid | No buried bores | Threading 300 mm of straight path vs 156–246 mm of curved tube | free | Lid design | ✅ |
| R2 | Threading leader | 0.25 mm stainless wire loop or a fly-tying bobbin threader | Pull through eyes and coils in seconds | €2 | — | ✅ |
| R3 | ISO 7379 Ø4 shoulder screws (M3) as removable axles | Unscrew instead of pressing out | Idler / pin swaps without a press | €0.3 | Needs a nut or insert | ✅ for the MCP idler axle |
| R4 | Stoppers that face outward | Anchors at spool (rim-side pocket) and drum (report 03 cup) | ~5 min per strand | free | — | ✅ |

---

## 3. Deep dives (narrow)

### D1 · Guides: what touches the line, and how hard

Line on a cylinder of radius r: mean pressure p = T/(r·d), and the bending strain is about d/D.
All for PE#8 (d 0.47), from §5 of the script:

| Guide | D/d | p at SCS / HLS (MPa) | Loss per 90° | Limit of the guide material | Use |
|---|---|---|---|---|---|
| 18G eyelet rim (V2) | 1.8 | 224 / 584 | 17 % | steel OK, line frays | ❌ remove |
| Ø3 ground dowel | 6.4 | 64 / 167 | 17 % (μ 0.12) | hardened steel ≫ | default guide |
| Ø4 ground dowel | 8.5 | 48 / 126 | 17 % | ≫ | where it fits |
| MR63ZZ roller | 12.8 | 32 / 84 | 0.8 % | C0 74 N per bearing | big fixed wraps |
| Brass sleeve Ø3 | 6.4 | 64 / 167 | 21 % | ~150 | SCS only |
| PTFE at R 3 | 12.8 | 32 / 84 | 10 % | ~6 (creeps) | ❌ |
| bare PETG at R 5 | 21 | 19 / 50 | 38 % | ~15 | ❌ |

**MCP crossing (pip strands).** I modelled a two-pin nip on the axis (§9). For the abduction crossing
(±15°), a line passing between two Ø3 pins on the axis changes length by only ±0.006 mm. But at a 90° MCP
fist, the same nip lets the line cut the corner by ≥ 1.2 mm. Both pip strands do the same, so the loop goes
slack by ~2.5 mm. The V2 eyelet was tuned to 0.19 mm, but its rim is the fraying problem.

The clean answer is the classic tendon-hand layout: an **idler pulley coaxial with the MCP axis** (§10).
- The flex strand runs on the palm side of an MR63 (r 3), the ext strand on the back side.
- When the MCP bends by θ, flex gains 3θ and ext loses 3θ, so the loop length stays exactly constant.
  The pip turns by a fixed 3/7 = **0.43° per MCP degree**.
- The firmware adds 0.60 × q_mcp to the pip servo. The pip servo travel grows from 140° to 200°.
  Check the SCS0009 range (≈ 300°, verify).
- What you get: rolling contact, no slack, D/d 12.8 and 0.8 % loss.
- Space: two MR63 (2.5 wide) at x ±1.4 inside the 4.4 mm centre gap between the MCP stubs. Each needs a 3 mm
  axle on the MCP axis, which means the MCP stubs change to a through-axle or a stepped one. This is the
  finger designer's call. Report 03 already proposes a full-length Ø4 axle ("reverse fork"); a Ø4 → Ø3 step
  carries the idlers.
- This is the same "known linear coupling" that report 01 accepts for the wrist.

**Wrist eye plate (if housings are deleted).** The line bends by up to the full wrist angle at one eye. The
pressure does not depend on the angle, only on the lip radius.
- Polished ceramic textile eyelets (alumina, flared, ID ~1 mm) or SiC rod-guide rings have a lip radius of
  ~0.5–1 mm. That gives 96–190 MPa at SCS stall and D/d 2–4: better than the 18G rim, worse than a pin.
- Because the line slides through the eye on every finger move, this is the wear point to test first
  (bench test B2).
- A larger lip (r ≥ 1.5) means ≥ Ø5 per eye. That is a 32-eye plate of ~30 × 30 mm, which makes the
  firmware-cancelled length change worse (≈ d·θ). So the eye size is a trade-off; test it.

**Failure mode:** a line abraded at the eye or pin. **Bench test B2** (below) finds it first.

### D2 · Housing, only where it can't be avoided

Shortening per strand at stall (§2):

| | SCS 45 N | HLS 118 N | Note |
|---|---|---|---|
| PTFE 1 × 2, 221 mm (V2) | 8.48 mm = 69°, 19 MPa | 22 mm, 50 MPa | above PTFE's ~10–12 MPa yield: creeps |
| PEEK 1/16" × 1.0, 221 mm | 2.32 mm = 19° | 6.1 mm = 50° | outer bend strain 5.3 % at R 15 (yield ~5 %) |
| Steel coil 0.3 × 2.0 (ID 1.4), 140 mm | 0.12 mm (Hertz contact between turns) + 0.2 bedding = 0.33 mm = 2.7° | 0.48 mm = 4° | steel |

Joint lag at stall, line + housing in series (§3, flexor only):

| Servo | Line | PTFE 221 mm | Steel coil 140 mm |
|---|---|---|---|
| SCS | 50 lb | 90° | 23° |
| SCS | PE#8 | 80° | **13°** |
| HLS | PE#8 | 208° | 31° |

So once the housing is fixed, the **line's stretch is the limit**. That is why PE#8 and joint-angle sensing
(report 06's magnet idea) are the next steps, not a stiffer housing.

**Fallback build (lined coil):**
- Closed-coil stainless spring stock, wire 0.3, OD 2.0, ID 1.4 (sold as long extension-spring stock or
  "spring tube"; verify the seller's size).
- PTFE capillary liner ~0.8 × 1.2 inside (verify the size). The line is 0.47.
- Coil ends on a **2 mm aluminium stop plate** drilled Ø1.6 on a 2.5 mm pitch (32 holes ≈ 12 × 22 mm),
  through a printed drill jig. The plate sits on a printed ledge: 632 N total → 4.6 MPa.
- Single strands (thumb) get a Ø3 × 0.45 brass tube ferrule plus an M1.6 washer.
- The liner floats. It is not in the load path, so its ends only need a printed cap.
- The **wrist length change of ±6.7 mm per housing** (V2 report 03) still needs a bow space. That was
  review finding #4, and it is why H1 (no housing) is preferred.

**Failure mode:** the coil bedding in more than assumed, or the liner wearing through. **Bench test B1.**

### D3 · Spool cartridge (prototype built)

The spool's axis is X, with x 0 at the rail face. The station numbers are in `out/02/spool_cartridge.json`.

```
  rail | flange 0.8 | ext land 1.6 | divider 0.8 | flex land 1.6 | flange 0.8 | hub 6.0 (tensioner) | dog face
       |<--- 2 x MR63ZZ (3x6x2.5) + 0.5 shim, bore Ø5.95 --->|            |  M2 chordal screw      | 2 x Ø1.5 dog pins
  Ø3 hardened dowel axle, pressed 4.5 mm into the rail, ends inside the spool         driver disc on the servo spline
  flanges r 6.5 (1.26 mm = 2.7 d above the line); land floor r 4.765 (line centre at R 5); keeper ring gap 0.2
```

- **Support.** Both strands leave toward the wrist, so their loads add: 50 N (SCS) / 123 N (HLS) on the spool.
  - Per MR63ZZ that is 25 / 61 N, against C0 = 74 N (NSK/Misumi). SF 2.9 for SCS, **1.2 for HLS**.
  - For HLS, use 683ZZ (3 × 7 × 3) or MR74ZZ, which leaves a 1.27 mm wall under the land.
  - Ø3 hardened axle at a 3 mm lever: 57 / 139 MPa (fine). The servo sees torque only.
- **Lands.** Flat bottom, 1.6 mm wide, for up to ~2 turns at a 0.61 mm pitch. Each strand needs 1 turn of
  anchor wrap plus the travel: 140° for the fingers, 197° for thumb cmc_rot. That is ≤ 1.55 turns.
  D/d = 21.
- **Ext anchor.** A tangential pocket (1.4 × 1.0 × 3.5) in the ext land floor at the bottom of the spool
  holds a crimp sleeve or a figure-8 + CA. The first turn of line covers it, so it carries ~28 %: 13 / 34 N.
  Wall to the bearing seat: 0.79 mm.
- **Flex anchor and tensioner.**
  - The flex strand leaves the flex land through a Ø1.0 hole in the hub flange (r 4.6, just under the line)
    and is tied to an **M2 square nut (DIN 562)** in a slot. An **M2 × 10 screw (DIN 912)** runs along a
    chord 3.7 mm from the axis, with its head at the +Y rim.
  - The line pulls the nut away from the head, so the screw is in tension (16 MPa at HLS) and the head bears
    on its counterbore.
  - One turn = 0.4 mm of line. 1/8 turn ≈ 0.9 N of pretension (loop stiffness 18 N/mm, both strands in series).
    Travel 5.5 mm.
  - Expected creep: ~1 % of 310 mm ≈ 3.1 mm for UHMWPE, 0.6 mm for Vectran.
  - Nut on the PETG slot face: 5.2 MPa at HLS.
- **Drive.** A printed driver disc sits on the servo's 20T spline and is held by the horn screw. Two Ø1.5 ×
  4 steel pins at r 4.6 engage radial slots in the hub (0.3 mm radial float). A servo comes out by its 2 tab
  screws: the spool stays on its axle, strung and tensioned.
- **Keeper.** A PETG ring (r 6.7–8.5) with two windows where the strands leave, two M2 ears and an index rib
  for the witness notch.
- **Length.** The spool is 11.6 mm plus the 2.4 mm driver = 14.3 mm from the rail face to the horn. That is
  about 3 mm more per side than V2's split spool, so the forearm cassette pitch must be re-spaced (integrator).

**Failure modes:**
1. The MR63 under HLS (SF 1.2): fixed by the 683 swap.
2. The 0.7 mm cap over the nut slot at the hub rim. It is not in the load path, but if it cracks, print the
   hub in PA or PETG-CF (no line slides on the hub).
3. The dog slots wearing. Pin load at stall = 0.59 N·m / (2 × 4.6 mm) = 64 N per pin (HLS) on 1.5 × 1.6 mm
   = 27 MPa on PETG: marginal. Use brass tube liners in the slots for HLS, or 3 dogs.

### D4 · Anchors: let the wrap carry the load

Share of the strand tension that reaches the termination (capstan upper bound, §6):

| Configuration | Share | At SCS / HLS stall |
|---|---|---|
| V2 drum tie hole, 40° margin | 87 % | 39 / 103 N |
| V2 thumb rot, 19.5° margin | 93 % | 42 / 110 N |
| Drum + 1 extra turn (helical groove) | 25 % | 11 / 29 N |
| Spool, 1 turn | 28 % | 13 / 34 N |
| Spool, 1.5 turns | 15 % | 7 / 18 N |
| Ø3 steel post, 3 turns | 15 % | 7 / 18 N |
| Ø3 steel post, 5 turns | 4 % | 2 / 5 N |

What it means:
- At the **spool**, wrap is free, so every spool anchor gets ≥ 1 turn.
- At the **drum**, space is tight. Report 03's crimped-bead fuse cup (it trips at ~68 N) is compatible with
  this: it is the termination, and the helical extra turn (A2) is an option if the cup's lip must carry
  less.
- A crimp on doubled braid holds ~70–90 % (verify). A UHMWPE knot holds 50–80 %. With PE#8 (445 N), a knot
  holds 220–356 N, which is ≥ 1.9× HLS stall even with no wrap relief.

### D5 · Friction and force (what the parts buy)

Index pip-flexor route at the worst case (fist, wrist at an extreme), from §4:

| Route | Total turning | η | Tip force at stall, 60 mm lever (SCS / HLS) |
|---|---|---|---|
| V2 (PTFE everywhere, eyelet) | 327° | 0.59 | 3.1 / 8.2 N |
| Bare + coil at wrist + Ø3 pins | 190° | 0.71 | 3.7 / 9.8 N |
| Same, with the MR63 idler at the MCP | 190° (most of it rolling) | **0.85** | **4.5 / 11.7 N** |

Assumed μ: lined coil 0.08, steel 0.12, bearing 0.005.

In the V2 palm, the 24 finger strands turn **20 / 96 / 175°** (min / median / max). A straight bare line from
an aimed stop plate to the unchanged yoke entry turns **1 / 16 / 40°**, and 0° if the yoke entry is aimed too.
Obstacles such as the thumb mount are not checked, so this needs the palm integrator.

---

## 4. Bench tests (each one afternoon; do B1–B3 before printing a hand)

- **B1 · Housing compression and creep.**
  - Rig: a printed rail with two aluminium stop plates 140 mm and 300 mm apart. The line runs over an MR63
    to a hanging weight; a dial gauge or calliper sits on the line end.
  - Samples: PTFE 1 × 2, PEEK 1/16", the lined steel coil. Loads 5 / 15 / 45 / 120 N, then 24 h at 45 N.
  - **Kill criterion:** the coil > 0.6 mm at 45 N over 140 mm.
- **B2 · Guide wear.**
  - Rig: a servo cycles a strand ±12 mm at 1 Hz with 45 N on the far end (a weight), over the candidate
    guide at a 90° wrap.
  - Candidates: 18G eyelet, Ø3 dowel, ceramic eyelet, MR63 roller.
  - Measure: cycles to the first broken filament (a loupe every 1 k), and the friction from two luggage
    scales (μ = ln(T1/T2)/θ).
  - **Pass:** ≥ 50 k cycles on the dowel and the roller.
- **B3 · Anchor pull-out.**
  - Pull PE#8 to failure with a luggage scale or hanging buckets: plain knot, figure-8 + CA, doubled crimp,
    and 1 turn on the printed spool + stopper.
  - **Pass:** ≥ 2× HLS stall (236 N), or the termination is downgraded to SCS use.
- **B4 · Spool cartridge.**
  - Print `spool_02` / `driver_02` / `keeper_02`. Press in 2 × MR63 with a 3 × 10 dowel in a printed rail
    block.
  - Hang 5 kg on one strand for 24 h. Watch the witness notch and the nut. Check that 1/8 turn of the screw
    moves the pluck pitch audibly.
  - Slack the loop fully and spin the servo: the line must stay inside the keeper.
- **B5 · MCP idler coupling.**
  - A printed proximal mock-up with an MR63 on the axis. Measure the pip angle against the MCP angle (expect
    0.43°/°) and the loop length (expect a constant within 0.1 mm).
- **B6 · Creep, 7 days.** PE#8 against Vectran at 1 kg on 300 mm, measured daily (tendon-mechanics.md §8).

---

## 5. What to build as CAD next

1. **Spool cartridge v02 → integrate.**
   - Take `cad/02_spool_cartridge.py` as the master. Make a 683ZZ variant (3 × 7 × 3) for the HLS slots.
   - Add brass liners to the dog slots for HLS.
   - Re-space the forearm cassette for the 14.3 mm stack. Add an access window in the forearm shell at the
     tensioner's service angle.
2. **MCP idler kit (with the finger designer).**
   - 2 × MR63ZZ at x ±1.4 on a stepped axle (Ø4 in the MR84s, Ø3 under the idlers), or report 03's
     full-length Ø4 axle with Ø4 → Ø3 sleeves.
   - Flanges: M3 washers 0.5 mm (DIN 433) between the idlers and the lugs.
   - The strands come from the palm side (flex) and the back side (ext). Firmware coupling: pip += 0.60 · q_mcp.
3. **Stop plate + aimed spinneret (palm side).**
   - A printed block with 32 bores, each aimed at its finger yoke entry (or at the wrist eye if housings are
     deleted), and a drilled 2 mm aluminium plate (12 × 22, Ø1.6 at a 2.5 pitch).
   - Plus a printed drill jig. Sized from `routes.json` start points.
4. **Guide-pin kit.**
   - Standard pockets for Ø3 × 8 ISO 8734 dowels (press Ø2.95, 0.4 mm proud edge break) at every bend over
     20°: yoke entry, abduction nip (two pins at x ±1.73 on the abd axis), and the thumb cmc_rot deflection.
5. **Bench rigs B1/B2** as printable parts: a rail with stop-plate slots and a guide holder that takes a
   pin, an eyelet or an MR63.

---

## 6. Sources

- Repo: `research/references/hands/tendon-mechanics.md` (EA estimate, μ table, capstan, the 3–5 N
  pretension rule, the M2 clamp rule); `cloud-claude/2026-11-09-v2-hand-cad/` reports 05–07, `params.py`,
  `forearm.py`, `out/assembly/routes.json`; this study's reports 01 and 03.
- MR63ZZ: Cr 208–230 N, C0r 74 N: [Misumi](https://th.misumi-ec.com/en/vona2/detail/221301228090),
  [NSK mr63](https://www.nsk.com/engineering/mr63mc3p6-apn.html), [NSK mf63zz](https://www.nsk.com/engineering/mf63zzmc5-apn.html).
  Other listings: [miniaturebearings.com.au](https://miniaturebearings.com.au/products/3x6x2-bearings-mr63zzseco/data?view=data), [SKD](https://www.skdin.com/products/productid-MR63zz).
- MR84ZZ: Cr 435 N, C0r 139 N: [NSK mr84zz](https://www.oss.nsk.com/mr84zzmc5-apn.html); others
  [SKD](https://www.skdin.com/products/productid-MR84zz), [Zoro](https://www.zoro.com/ezo-ball-bearing-01575in-dia-31-lb-shielded-smr84zzp6mc3af2/i/G2402200),
  [SMB](https://www.smbbearings.com/firebrick/ckeditor/plugins/upload/Uploads/Documents/bearingpdfs/SMR84ZZ-stainless-steel-miniature-bearing-4x8x3mm.pdf).
- Standards: ISO 8734 (hardened dowel pins), ISO 7379 (shoulder screws Ø4 / M3), ISO 7089 (M1.6 washer
  1.7/4.0/0.3), DIN 562 (M2 square nut), DIN 912 (M2 cap screw).
- From memory, to verify: the PE-number → diameter table (JIS-style, PE#1 ≈ 0.165 mm) and braid break loads;
  the UHMWPE creep rate (~1 % over months at ~20 % load, room temperature); D/d ≥ 8 for HMPE over sheaves
  (rope makers' guidance); PEEK yield strain ~5 %, E 3.6 GPa; PTFE compressive yield 10–12 MPa; ceramic
  textile-eyelet and SiC-ring sizes; the SCS0009 position range; the HLS3606M torque.
- Hertz line contact for the closed coil: standard cylinder-on-cylinder formulas (Johnson, *Contact
  Mechanics*). The bedding-in allowance (0.15 mm per 100 mm) is an assumption for B1 to replace.
