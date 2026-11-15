# 03 · Joints and finger structure (hardware ideas study, 2026-11-14)

Agent 03. Scope: how the printed finger, thumb and palm are built and jointed. Everything here is a hand
calculation with the assumptions named in the scripts (no FEA, nothing printed). Scripts and outputs:

| File | What |
|---|---|
| `cad/03_loads.py` → `out/03/03_loads.txt` | Joint loads, friction, lateral-moment map, drum pressure, fuse tab, DIP pins, walls, bearing count |
| `cad/03_palm_fingers.py` → `out/03/03_palm_fingers.txt` | Finger spread with tapered phalanges, fixed palm arch |
| `cad/03_drum_fuse.py` → `out/03/03_*.stl`, `03_drum_fuse.step`, `.json` | CadQuery prototype: drum puck + metal ring + oval fuse cup + cross-drilled axle (all solids valid) |

Run: `uv run --no-project --with numpy python cad/03_loads.py`; the CAD one needs `--with cadquery`.

## Reading the V2 baseline in one paragraph

V2's joints are mechanically fine at the loads the servos make, and weak at the loads the *outside world*
makes. A fork joint has to resist a sideways fingertip push as a couple on its two bearings. At the MCP that
couple is **6 N on each bearing for every 1 N at the fingertip** (85 mm lever / 14.2 mm bearing span). Almost
nothing else in the finger sees that factor. It decides the stub, the lug and the "pop-out" question.

---

## 1. Top 3 recommendations

**1. Fix the MCP axle before anything else: the 4 × 6 mm press-in stubs fail at about 8 N of sideways tip force.**
Only 3 mm of each stub is in the PETG hub. A rigid pin in plastic, loaded 1.5 mm out from the hub face,
crushes the plastic at about 50 N radial (25 MPa) = **8.4 N at the tip** (`03_loads.py` §C). The goal is
≥ 20 N (twice the 10 N pinch). Three fixes, cheapest first: stub engagement 3 → 5 mm (**17 N**); plus a
0.3 mm brass sleeve in the hub (**31 N**); or a V3 "reverse fork" with a fixed, full-length Ø4 steel axle
cross-drilled for the pip strands (**69 N**; see §3.2). PIP and DIP already use through-pins and are fine
(52 N / 137 N). Zero cost for the first fix, €0.02 for the second.

**2. Put a metal ring under the line in every drum, and end the strand in a replaceable "fuse cup" instead of a knot.**
A 0.36 mm line on a bare PETG groove presses at T/(R·d) = **17.9 MPa at SCS stall, 46.8 MPa at HLS stall**
(PETG yields at about 45 MPa and creeps above about 12). A 0.5 mm brass or stainless ring (13.64 mm OD,
3 mm wide, split so it snaps on and off) spreads that to **4.7 / 12.4 MPa**, and stops the braid cutting into
the groove. The same drum gets a two-bead cup (5.2 × 2.7 × 2.6 mm, 0.03 g) holding crimped beads under a 0.8 mm
lip. That lip shears at about **68 N** (it is a printed fuse, cup printed flat so the lip is 4 layers: 0.6 mm ≈ 51 N, 0.8 mm ≈ 68 N, 1.0 mm ≈ 85 N,
±25 % scatter). It replaces the tie hole + 0.6 mm wall + knot (a knot holds 50–80 % of the line and sits at 80 MPa on PETG, review 05 #10). When a finger is overloaded the cup trips and the **servo and gears
are never loaded above the fuse**. Fix = swap a 0.03 g cup and re-crimp two beads, about 2 minutes.
Prototype built: `out/03/03_drum_*.stl`.

**3. Do not use click-in (pop-out) bearing seats on the MCP/PIP. Keep 20 MR84 pairs (not 30), fuse the tendon, and bumper the stops.**
The ORCA-style open C-seat looked like the answer for "what breaks in a fall". It is not, for a finger this
long and narrow: a lip that holds 90 N (to survive a 15 N sideways touch, with the 6× lateral couple) can't be
pressed in by hand, and a lip that can be pressed in (6–22 N) pops at **1–3.6 N of sideways tip force**
(§D of the script). Instead: (a) tendon fuse (rec. 2) protects the servo, (b) replaceable TPU bumpers on the
extension stops absorb hyperextension, (c) bearings stay at the loaded joints (MCP, PIP, thumb cmc_flex and
MCP = 20 × MR84 + 8 × MR63) and the DIP/thumb IP drop to a Ø4 steel pin in a brass bush (saves 10 bearings,
€3, 7 g, 4 min assembly, and ten fewer 7.95 mm press seats to get right). Also worth stealing now at zero cost: **tapered
phalanges (18 → 14 mm)** triple the legal finger-to-finger adduction (1.2° → 3.6°), and a **fixed 12° palm arch
for the little finger** moves its MCP 7.9 mm palmward at zero DOF.

---

## 2. Idea list (wide): 31 ideas

Numbers are from the scripts; "≈" means an estimate with a stated assumption. Effort: S = hours, M = days, L = weeks.

### 2.1 Joint types

| # | Idea | How it works | Gain / number | Cost / effort | Risk | Verdict |
|---|---|---|---|---|---|---|
| 1 | Printed pin in a PETG hole | pin printed with the child | friction 11.6 % of drive torque; breakaway 0.8 N of tendon force (stick-slip) | free, S | wears loose in 100s of cycles, layers across a 4 mm pin | **Reject** |
| 2 | Ø4 steel dowel in reamed PETG | reamed 4.0 H8 hole, running fit | 8.3 % loss; 5.5 MPa bearing at HLS stall (under the 12 MPa creep limit) | €0.10, S | PETG bore wears, ream needed | **OK for prototypes, thumb IP** |
| 3 | Dowel in a brass tube bush (3-4 mm, pressed) | brass sleeve takes the wear | 5.0 % loss; breakaway 0.34 N | €0.40/joint, S | press fit tolerance | **Take for DIP and thumb IP** |
| 4 | igus iglidur J flanged bush | dry-running polymer bush | 4.0 % loss; PV far below limit | €0.6/joint | vendor-specific, ordering | Equivalent to 3; use whichever is stocked |
| 5 | PTFE sleeve | low friction liner | μ 0.1 | cheap | cold flow at 15 MPa | **Reject** |
| 6 | MR84ZZ pair (V2) | 2 × 4×8×3 bearings per joint | 0.1 % loss, breakaway 0.01 N, no wear, no play | €0.3–0.5/joint, M (seat tolerance) | 7.95 mm press seats in FDM are a guess | **Keep at MCP, PIP, thumb cmc_flex, MCP** |
| 7 | One bearing + one bushing per joint | half the seats | −1 bearing/joint | S | the two lugs are unequal in stiffness: worse under the 6× lateral couple | Reject for MCP; fine at DIP (couple only 1.5×) |
| 8 | Click-in open C-seat (ORCA pop-out) | bearing snaps radially into an arc seat | pop force 6.5–22 N per seat | M | **pops at 1–3.6 N sideways tip force at the MCP** (6× couple) | **Kill (see §3.5)** |
| 9 | Print-in-place hinge | joint printed assembled, 0.3 mm gap | no assembly | S | PETG/PETG μ 0.35, fuses, weak across layers | Reject |
| 10 | TPU flexure / PP living hinge | bending strip is the joint | no friction, no wear | S | axis drifts under load, spring torque fights the drum; PP prints poorly | Reject for flexion; use TPU for tip pad, palm pad, bumpers |
| 11 | Rolling-contact joint | two curved faces roll, strap holds | no friction | L | Tesla dropped it; ETH moved back to pin joints | Reject |
| 12 | Magnet breakaway joint | magnets hold, pops on overload | click-back | S | magnet holds 1–2 N; the tendon load alone is 50–180 N | Reject for joints; fine for a **snap-on fingertip cap** (#26) |
| 13 | Shoulder screw / E-clip retained axle | Ø4 axle, one E-clip or screw; removable | axle out in 20 s with tweezers | €0.2, S | head or clip must fit inside the Ø5.2 inner-race land | **Take** (E-clip, not a screw head) |
| 14 | Cross-drilled through-axle | Ø4 steel, 1.2 mm cross hole, strand passes through at the axis | 69 N lateral vs 8 N for V2 stubs; replaces 2 eyelets | €0.5, M (needs a "reverse fork") | the line still bends over the hole edge: same fatigue as the eyelet | **V3 option, test one** |

### 2.2 Finger construction

| # | Idea | How it works | Gain / number | Cost | Risk | Verdict |
|---|---|---|---|---|---|---|
| 15 | One-piece side-printed phalanx (V2) | print with the joint axis vertical | holes round, loads in-plane | S | closed strand bores 1.2 mm print as faceted ovals; can't see inside | Keep for distal phalanx |
| 16 | **Core + one side cover** (not a symmetric clamshell) | strand channels are open grooves in the core, closed by a 1.2 mm cover on the −x face (where the DIP bar is) | channels print without sag; threading = lay the line in; cover shear flux is 0.3 MPa at 20 N lateral (trivial) | +1 flat part/phalanx (0.3 g), 2 × M1.6 or snap | the cover must not carry the DIP bar pin load | **Take** for proximal and middle phalanx |
| 17 | Symmetric clamshell split at x = 0 | two halves | – | M | V2's strand channels sit at x = ±1.4, so no channel lies on the split plane | Reject (see 16) |
| 18 | Finger cartridge swap | abd axle on one E-clip; 4 sheath ends + 2 bead cups lift out | ≈ 5 min vs a teardown | M | needs open palm channels with lids (review 05 #14) and push-in sheath ends (drivetrain agent) | **Take**, design the palm for it |
| 19 | Standard hinge ends | all four fingers: same fork/tongue (17 mm width, Ø8 seats, 3.8 lugs); only the shaft between differs | 4 variants → 1 for the joint ends; jigs and spares shared | S | needs the little finger at 17 not 15.5 mm | **Take** (taper hides the width, #22) |
| 20 | Same phalanx scaled | scale the part for each finger | – | – | scaling changes bearing seats | Reject; scale length only |
| 21 | Heat-set inserts vs self-tapping | M2 inserts need a Ø3.2 hole in a ≥ 6.4 mm boss | M2 × 6 self-tapping in PETG holds ≈ 80–120 N (estimate) | S | M2 inserts too bulky in a 16 mm finger | Self-tapping M1.6/M2 in fingers; heat-set M3 only in palm, forearm, servo mounts |
| 22 | Tapered phalanges | 18 mm at the knuckle → 14 mm at the tip | relative adduction 1.2° → 3.6° (index/middle), 0.9° → 3.3° (ring/little) | free, less plastic | – | **Take** |
| 23 | Bonded pins (CA) | glue instead of press | – | – | can't be driven out | Knurled/pressed pins, CA only as backup |
| 24 | Wall rule | 0.43 mm per line: 2 lines 0.86 (min for non-load), 3 lines 1.29 (load), 4 lines 1.72 (press seats) | delete V2's 0.28/0.6/0.7 mm walls | S | – | **Take** |

### 2.3 Drum, termination, DIP bar

| # | Idea | Gain / number | Cost | Risk | Verdict |
|---|---|---|---|---|---|
| 25 | Metal ring in the drum groove | PETG pressure 17.9 → 4.7 MPa (SCS), 46.8 → 12.4 (HLS); no braid cutting | €0.10/ring, 0.52 g | R_line 7.0 needs a 13.64 OD ring (turn it, or accept 14.0 OD tube and R_line 7.18 → servo ratio 1.436) | **Take** |
| 26 | Bead + replaceable fuse cup in place of knot | fuse at 68 N (0.8 mm lip); next-weakest part ≥ 2× | 0.03 g, €0 | scatter ±25 %; ring retains the cup | **Take**; this is also the snap-on-tip idea in drum form |
| 27 | Steel bar + 2 mm pins, 4 mm in | bar 3.8× (PETG) to 22× (steel) the 30 N bar load; pin wall pressure 14.6–16.9 MPa | €0.3 | local boss, or a through-pin | **Take** (below) |
| 28 | Crossed-tendon DIP coupling | – | – | needs tensioning, creeps, DIP has no servo to take up slack | Reject (V1 reasoning stands) |
| 29 | Music-wire Z-link whose bent ends are the pins | no separate pins; bend radius ≥ 1 mm | €0.05 | wire ends turn in PETG holes (wear) | Prototype alternative to 27 |

### 2.4 Palm, abduction, overload

| # | Idea | Number | Verdict |
|---|---|---|---|
| 30 | Fixed palm arch: rotate ring 6° and little 12° about z | little MCP 7.9 mm palmward, 0.8 mm toward the middle; fingertip leans 14.6 mm toward the thumb | **Take as a CAD experiment**; check thumb reach in the V2 pose solver (hypothesis: less cmc rotation needed) |
| 31 | Hinged passive arch (TPU strap + stops) | adds a hinge and 8 sheaths crossing it | Defer until the fixed arch is tested |
| – | Rigid palm skeleton + 2 mm TPU pad | stays (V2) | Keep |
| – | Replaceable TPU bumpers on the extension heels | stop contact force = F_tip × 85/7 = 12 × the tip force | Take (cost 0) |
| – | Snap-on fingertip cap (hard core, TPU skin) | DIP couple is only 1.5 N/N, so a 20 N lip works | Take |

---

## 3. Deep dives

### 3.1 What loads must the structure carry? (the number that frames everything)

- Brief: 10 N at 60 mm = **0.6 N·m at the MCP** → tendon **86 N** on a 7 mm drum (efficiency 1). SCS0009 stall (0.226 N·m on a 5 mm spool) = **45 N** = 5.3 N at 60 mm. HLS3606M (0.59 N·m) = **118 N** = 13.8 N. So 10 N pinch needs an HLS-class servo even at 100 % efficiency.
- The joint axle sees **flex + ext tension** (both strands go proximal): 49 N (SCS), 122 N (HLS), 181 N (HLS × 1.5 = my design load). Pin shear is nothing (Ø4 steel, double shear: 7 MPa at 181 N); it is **plastic bearing pressure** that matters: Ø2 stubs at 132 N → 8.6 MPa (over the PETG creep limit I assume, 12 MPa, for any HLS-class load once stress concentrations count); Ø4 over 6 mm → 5.5 MPa.
- **Friction is small**: a dowel loses 4–8 % of drive torque; a bearing 0.1 %. The sheaths lose 25–55 %. So bearings are for smoothness, play and wear, not for force.
- **Sideways/impact loads are bigger than drive loads.** A fork resists a lateral tip force F as radial loads on the bearings of F·L/span: MCP 6.0 F, PIP 3.9 F, DIP 1.5 F. 20 N sideways at the MCP = 120 N on each bearing/stub.
- An external push of 3.7 N against the flexor at the MCP already equals SCS stall; 10 N is 2.7× stall (tendon T = F·85/7). SCS0009 gear strength is unknown (verify, bench test 4). If the gears are plastic, this is the first thing to break in a jam.

### 3.2 MCP axle: stubs, sleeves, or a reverse fork with a cross-drilled axle

Table (static allowable sideways tip force, no safety factor; `03_loads.py` §C):

| Weak point | P_allow | Tip force |
|---|---|---|
| V2 stub Ø4, 3 mm in the hub | 50 N | **8.4 N** |
| stub, 5 mm in the hub | 104 N | 17.4 N |
| stub, 5 mm + 0.3 mm brass sleeve (pressure 45 MPa on the sleeve) | 188 N | 31.3 N |
| stub, 6 mm + sleeve | 240 N | 40.1 N |
| MR84ZZ static rating (assume C0 = 300 N: **verify**) | 300 N | 50 N |
| PETG lug ring around the Ø8 seat, 3 mm wide | 320 N | 53 N |
| steel Ø4 axle in bending | 838 N | 140 N |
| **Cross-drilled Ø4 through-axle** (1.2 mm hole, −51 % bending section) | 411 N | **69 N** |

Where does the "stubs" geometry come from? V2 leaves the centre (|x| < 2.2) free for the two pip strands that cross
the MCP axis, so a through-axle can't pass. Options:

A. **Longer, sleeved stubs (do now).** Keep V2's geometry; make the hub stub bore 5 mm deep in a local boss (the MCP knuckle is 9.9 mm wide between lugs, there is room: stubs start at |x| = 2.65 and the hub is at least 3 mm), steel Ø4 × 8 stub, 0.3 mm brass sleeve glued in the bore. 31 N.

B. **Reverse fork + fixed cross-drilled axle (V3 test).** The yoke has a 10 mm central tongue carrying one Ø4 × 15 steel axle (fixed). The *proximal phalanx* gets the two lugs with the MR84 pairs. The span stays 14.2 mm (bearings still at x = ±7.1), so the 6× couple is unchanged, but the axle is now supported along 10 mm in the yoke and loaded by bearings at its ends. The pip strands run through two Ø1.2 cross holes at x = ±1.4 (web between the holes 1.6 mm), polished, chamfered 0.15 mm (the prototype `03_axle.stl` has one centred hole). Strand length is then the exact distance from the on-axis point to the PIP drum tangent: constant by geometry; the residual comes only from the bend radius at the hole edge (r_f 0.5 mm → ≲ 0.3 mm, same class as the 0.19 mm eyelet). It removes 2 eyelets per finger and the eyelet slot. Fatigue is not better (bend radius 0.5 vs 0.43 mm): **the eyelet-fatigue question (review #5) needs its own cycle test either way.**

C. Do not move the bearings into the child (a "short span" fork): the span falls from 14.2 to ≈ 7.4 mm and the couple rises to 11.5 N per N.

Layer direction: the phalanx lies on its side (x-normal). Lug loads from the lateral couple are in-plane (along z), good. Axial thrust on the bearing shoulder (annulus Ø6.75…Ø7.95, 0.8 mm = 4 layers) is across layers: shear plug area π·6.75·0.8 = 17 mm² × 15 MPa ≈ 255 N, fine. The stub sleeve and axle bores are drilled/reamed, not printed to size (print 0.1–0.2 mm under).

**Bench test 1 (lateral):** clamp the yoke, hang a luggage scale on the fingertip pad pulling along ±x, step 5 N, measure tip deflection with a dial gauge. Variants: 3 mm stubs, 5 mm stubs, 5 mm + sleeve, B. Pass: ≥ 20 N with < 0.5° permanent set. The 3 mm stub should visibly set at 8–10 N. This takes one afternoon and four printed knuckles.

### 3.3 Drum ring + bead/fuse cup (prototype built)

Section (index MCP track, looking along the axle, +Y dorsal; not to scale):

```
         flange R8         bead cup (oval, 5.2 x 2.7 x 2.6, PETG)
          ______                  lip 0.8 (the fuse)   hole Ø0.8
        /        \        ring OD 13.64, 3.0 wide, 0.5 wall, split (C-ring)
       |  hub     |  ====[line lane flex x=-1.35 | ext x=+1.35]====
       | R6.32    |        |bead|  |bead|   <- 2 chambers Ø1.8 under the lip
        \______ /          |  cup (one piece)  |
        axle Ø4 (press fit in the child, or fixed in V3)
```

Parts and mass (CadQuery, all valid solids): drum body 2.0 g, ring 0.52 g, cup 0.029 g, axle 1.44 g. Interference with the body: ring 0, cup 0, axle 2.4 mm³ (the designed 0.04 mm press fit).

- **Why a ring:** `p = T / (R·(d + 2t))`. Bare groove (0.36 mm line): 17.9 / 46.8 / 70.2 MPa at SCS / HLS / HLS × 1.5. With a 0.5 mm wall: 4.7 / 12.4 / 18.6. With 0.8 mm: 3.3 / 8.6 / 12.9. A 0.1 mm stainless shim only fixes cutting (11.5 / 30 / 45 MPa). 80 lb braid (0.45 mm) helps the bare groove only 20 %. The ring also gives a hard, smooth surface for UHMWPE (the tendon note: "never let a tendon slide on CF parts"; the same is true for soft PETG, which it grooves).
- **Why two lanes on a 3 mm ring:** each strand runs on its own lane (x = ∓1.35), they never cross, and the beads sit directly below their lanes. The sheath exits already sit ∓3.3 mm from the finger centre in V2.
- **Why the ring is split:** the cup is held under the ring. A solid pressed ring would need a puller to change a cup. A C-ring (0.3 mm gap, sprung 0.1 mm undersize) snaps on and comes off with a screwdriver. The ring hole rim must be deburred (the line turns 90° into the cup hole; tension there is only the part not shed by the 40° wrap, still ≈ 90 %): chamfer 0.2 mm. **This corner is a risk; test it (bench test 3).**
- **Fuse lip:** shear plug of the bead diameter, `F = π·d_b·t·τ` with d_b = 1.8 mm and τ = 15 MPa (across layers) to 25 MPa (in the layer plane):

  | lip t (mm) | 0.4 | 0.6 | 0.8 | 1.0 | 1.2 |
  |---|---|---|---|---|---|
  | F, in-plane τ 25 | 57 N | 85 | 113 | 141 | 170 |
  | F, across layers τ 15 | 34 | 51 | 68 | 85 | 102 |

  Default: print the cup **flat** (lip = 4 layers of 0.2 mm), use the across-layers row: **t = 0.8 mm ≈ 68 N, the SCS target (1.5 × 45 N)**. That is conservative; if the real cups trip high, the in-plane row applies. For HLS (1.5 × 118 = 177 N) print the cup on its side (in-plane row, t = 1.2 mm ≈ 170 N) and use **80 lb braid** (break ≈ 355 N; 50 lb braid breaks at 222 N and a bead sleeve holds ~90 %).
- **What it protects against:** an external tip force pushing the finger against its flexor. Tendon force `T = F·85/7`: 5.6 N at the tip trips a 68 N fuse; 9 N breaks a knotted 50 lb line (111 N at a 50 % knot); 10 N is 2.7× SCS stall. The fuse sits between the two, so the failure is a 2-minute swap instead of a snapped line or a damaged gearbox.
- **Cost:** ring €0.10, 20 rings for the hand ≈ 10 g and €2; beads (aluminium single-barrel crimp sleeves, 100 for €3).
- **Servo ratio note:** a 13.64 mm OD ring puts the line centre at R = 7.0 exactly (owner's "same distance on each degree"). A stock 14.0 mm OD tube puts it at 7.18 mm (+2.6 %): servo_per_joint becomes 1.436 and the sim/firmware constant changes. A lathe-turned ring from 14 mm stock costs about €1 each.

**Bench test 2 (drum creep):** two printed drum pucks (bare groove, ring), line over each, hang 5 kg (49 N ≈ SCS stall) for 7 days; measure the groove depth with a Ø0.5 pin gauge and the line slack. Then 10 k cycles at 45 N. Pass: < 0.05 mm groove set on the ring puck. Expect the bare groove to creep visibly (17.9 MPa is 1.5× my assumed limit).

**Bench test 3 (fuse calibration):** 10 cups each at t = 0.4 / 0.6 / 0.8 / 1.0, printed on their side. Pull the beaded line with a luggage scale (or hanging weights) until the bead goes through. Record mean and spread. Pass: coefficient of variation < 15 %, and the 0.6 mm cup trips at 50–90 N. Kill criterion: spread > 30 %: then use a calibrated shear pin instead (a sacrificial 0.8 mm PETG pin, the same maths).

### 3.4 DIP bar and pins

- Bar load at 10 N tip force: 10 × 18 mm / 6 mm = **30 N** (V2 estimated 35 N).
- Net-section capacity: PETG 3.2 × 1.6 mm bar with a 1.5 hole: 115 N (3.8×: OK for a first print, marginal when printed in the weak direction); laser-cut 0.8 × 3.2 mm stainless strip: 680 N (22×); Ø1 music wire: 589 N (20×).
- **The real weak point is the pin in the phalanx wall, not the bar.** A 1.5 mm pin 1.5 mm deep in PETG, bar 0.8 mm: edge pressure `6M/(d·Le²)` = **72 MPa** (yield 45). 1.5 mm × 3 mm: 28 MPa. 2.0 mm × 3 mm: 21 MPa. **2.0 mm × 4 mm: 14.6 MPa (OK).** So: Ø2 steel pins, ≥ 4 mm embedded in a local boss, or one through-pin across the phalanx (best; both walls support it). The V2 pin A web to the bearing seat is 0.28 mm (not printable: a single extrusion line is 0.43 mm): move pin A out until the web is ≥ 1.29 mm (3 lines), accept a slightly worse DIP fit, then re-run `dip_linkage.design()`; V2's own note says pin A at r 5.25 raises the error to ~1.5°.
- **Recommended bar:** 0.8 mm × 3.2 mm stainless strip, laser cut (service, €0.3 each), 2.05 mm holes on Ø2 pins. Backlash with a Ø2 pin in a 2.05 hole = 0.05/6 rad ≈ 0.5°, same as V2's 0.6°. Steel on steel (bar/pin), so no wear; the only plastic wear is the pin seat (cycle test: bench test 5).
- **Bench test 5:** 10 000 PIP cycles 0–95° on a finger with the bar loaded by a spring (30 N); measure play before and after with a dial gauge on the fingertip. Pass: < 0.5° increase.

### 3.5 Why the click-in joint is dead (and what replaces it)

ORCA's pop-out bearing seats work in a hand where the couple is small. With our proportions:

| lip engagement | strain in a 1.6 × 10 mm arm | pop force | lateral tip force that pops it at the MCP |
|---|---|---|---|
| 0.3 mm | 0.7 % | 6.5 N | **1.1 N** |
| 0.5 mm | 1.2 % | 10.8 N | 1.8 N |
| 0.8 mm | 1.9 % | 17.3 N | 2.9 N |
| 1.0 mm | 2.4 % | 21.7 N | 3.6 N |

To survive 15 N sideways the lips would have to hold 90 N, and a thumb can't snap a bearing in against 90 N.
(A seat that opens toward the distal end *is* naturally seated by the tendon pull, because both strands pull
proximally; that is a real advantage but it is irrelevant when the lateral couple dominates.) Replacement
overload strategy, in order of what fails first, by design:

1. **Tendon fuse cup** (§3.3): trips at ~68 N tendon force (SCS) = 5.6 N external push against the flexor.
2. **TPU stop bumpers** on the extension heels (0.8 mm TPU, snap-in): the stop sees `F_tip × 85/7` = 12 × the tip force: a 5 N tip hit is 60 N on a 3.8 × 2 mm heel. Replaceable, cost 0.
3. **Snap-on fingertip cap** (hard PETG core, TPU skin): at the DIP the lateral couple is only 1.5 N/N, so a 20 N lip pops at ≈ 13 N sideways tip force and 20 N axially.
4. Then the printed lugs (≈ 53 N sideways), bearings (≈ 50 N at the MCP, static rating), axles (≥ 69 N).

### 3.6 Fixed palm arch and finger spread (`03_palm_fingers.py`)

- V2 lets neighbouring fingers adduct only about 1° because parallel 18 mm fingers on 20 mm pitch leave a 2 mm gap and the tips swing 95 mm. A human finger tapers; the fingertip is 13–14 mm. With a linear taper 18 → 14 mm the legal relative adduction is **3.6°** (4.2° at 18 → 13 mm); ring/little 0.9° → **3.3°**. Spread (moving apart) is free either way. The limit that remains for ring/little in V2 (9°) is the staggered yokes, so give the yoke the same taper on the neighbour side, or add 2 mm of pitch.
- Rotating the ring finger module 6° and the little finger module 12° about a z-line through the middle MCP (a metacarpal arch, human ≈ 5–8° and 10–15°): ring MCP 2.0 mm palmward, little MCP **7.9 mm palmward**, 0.8 mm closer to the middle finger, little fingertip 14.6 mm toward the thumb for a 70 mm reach. Cost: palm and abd axes are no longer parallel to y; it is just new numbers in `params.FINGERS` (a rotation per finger) and a re-run of the pose solver. Do not build the hinged, passive version (idea 31) first.

---

## 4. What to build as CAD next (in order)

1. **`mcp_knuckle_v3.py`: MCP knuckle with a 5 mm sleeved stub** (variant A), and the **reverse fork with a cross-drilled axle** (variant B), as one parametric switch. Dimensions: Ø4 h9 steel axle 15 mm; stub variant: 4 × 8 mm stub, 5 mm in a Ø5.6 boss, brass sleeve 4.0 / 4.6 mm OD × 5; B: two cross holes Ø1.2 at x = ±1.4, 0.15 mm chamfer; MR84 seats Ø7.95 × 3.0, shoulder 0.8, bore 6.75; lugs 3.8 mm; central tongue 10 mm. Output: 4 test knuckles for bench test 1.
2. **`drum_ring_fuse.py`** (prototype exists, `cad/03_drum_fuse.py`): integrate into `finger.py` for MCP, PIP, abd (abd drum: 5.6 mm wide). Ring: 13.64 OD × 12.64 ID × 3.0, 0.3 mm split; groove 3.4 wide; hub R 6.32; flange R 8; oval cup 5.2 × 2.7 × 2.6, chambers Ø1.8 at x = ±1.35, holes Ø0.8, lip t as a parameter (0.4…1.2); pocket +0.1 per side. Remove the tie hole, the Ø1.6 knot well and the 0.6 mm wall. Add a `fuse_lip` parameter to `params.py`.
3. **Core + side cover phalanx** for the proximal and middle phalanx (idea 16): open 1.2 × 1.6 mm channels in the core, 1.2 mm cover with 2 × M1.6; all walls ≥ 1.29 mm; pin A moved out of the 0.28 mm web; Ø2 pins × 4 mm in local bosses (or a through-pin).
4. **Tapered phalanges + tapered yoke**: width(z) = 18 → 14 mm (index/middle/ring), 17 → 13 (little); same fork/tongue ends on all four (17 mm knuckle, Ø8 seats).
5. **Palm arch experiment**: per-finger rotation about z (0° index, 0° middle, 6° ring, 12° little) in `params.py` and a re-run of the poses (thumb to little finger first).
6. **Parts list update**: bearings 20 × MR84 + 8 × MR63 (was 30 + 8); DIP and thumb IP on Ø4 pin in brass bush; 20 steel/brass rings; 20 fuse cups.

---

## 5. Open questions and honest limits

- MR84ZZ C0 = 300 N is my estimate (V2 used 200 N for MR63). Datasheets vary; look it up before relying on §C rows 5, 7.
- PETG allowables (45 MPa yield, 12 MPa creep, shear 15–25 MPa) are typical values, not measured on the owner's printer or filament. The bench tests exist to replace them.
- The ring-spreading law `d + 2t` is a 45° estimate. Bench test 2 settles it.
- I did not check the fuse cup against the V2 palm/finger interference, or add the ring to the existing finger CAD (not allowed to edit it).
- The SCS0009 gear material and backdrive strength are unknown (bench test 4: hang weights on a 5 mm spool lever with torque off and torque on, find where it slips or strips). If it is metal and slips earlier, the fuse can be set higher.
- Layer-line direction and fit tolerances (7.95 mm seats, Ø4 pin holes at 3.9 / 4.0) still need the coupon test from report 06.
- The cross-drilled axle idea does not fix the eyelet's fatigue problem. A 10 k-cycle strand test on each guide type is needed regardless.
- Bench test 4 (servo): see above. Bench test 6 (taper): print two tapered fingers on 20 mm pitch and sweep the abd range; check at 1° steps.

## 6. Sources

- Cloud-claude/2026-11-09-v2-hand-cad: README, `reports/01-finger.md`, `05-review.md`, `06-bearings-track.md`, `cad/params.py`, `cad/finger.py` (dimensions: lugs 3.8, seats Ø7.95, tongue 8.3, bar pins, MCP stubs 4 × 6).
- `research/references/hands/README.md`, `tendon-mechanics.md` §5 (joint table, print orientation, tolerances), §7 (materials), `dexterous-and-open-source-hands.md` (ORCA pop-out joints and 10 000-cycle durability; PSYONIC pop-out fingers).
- Standard relations: capstan, Euler buckling, bearing-pressure `p = F/(d·L)`, embedded-pin moment balance, cantilever snap-fit strain `ε = 1.5 t δ / L²`, shear plug `F = π d t τ`. Values for PETG, brass, steel are textbook typicals (named in the scripts).
- The brief's constants: SCS0009 0.226 N·m at 6 V, HLS3606M 0.59 N·m, 5 mm spool, 7 mm drum.
