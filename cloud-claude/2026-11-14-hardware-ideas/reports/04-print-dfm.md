# 04 · Printing, materials and assembly (design for manufacture)

Agent 04, 2026-11-14. Study only: nothing outside this folder was changed. Scripts: `cad/04_coupon.py` (the test coupon), `cad/04_creep_stress.py` (stress and print-time hand calculations). Outputs: `out/04/`.

Plain-English words used below:
- **Layer lines / anisotropy**: an FDM part is a stack of welded lines. It is strong along the lines (XY) and weaker across them (Z).
- **Creep**: a plastic under constant load slowly stretches, even far below its yield. Our tendon pretension is a constant load for hours.
- **HDT**: heat deflection temperature, roughly where a loaded plastic starts to go soft.
- **Teardrop**: a round hole with a pointed top, so a horizontal hole prints without supports.

## 1. Top 3 recommendations

**1. Keep PETG for the hand, but move the four places where plastic is squeezed onto metal, and make the hot forearm frame out of PAHT-CF (or aluminium).**
The plastic bodies are barely loaded: sustained stress in the finger and lug webs is about 0.2–1 MPa (`04_creep_stress.py`), against a long-term allowance of about 9 MPa for PETG (my rule: 0.2 × the 47 MPa yield, not a datasheet value). Creep will not bend a finger. It will, however, relax four concentrated contacts: (a) the **M2 clamp screw on the split spool** (500 N preload under a 4 mm head is 57 MPa, and 32 MPa even with a washer, vs 9 MPa allowed: the clamp *will* loosen, so clamp metal on metal or use a steel washer plus thread-lock); (b) the **sheath end on a flat stop** (22 MPa at the 45 N stall, 2.5 MPa with 5 N pretension: use a 2.6 mm ferrule on a shoulder, 11 MPa peak, or a flanged one, 4 MPa); (c) the **knot in a tie hole** (the 16G liner pressed 3 mm deep cuts 20 MPa to 8 MPa peak); (d) **bearing press fits** (0.05 mm interference gives roughly 9 MPa hoop stress in PETG that relaxes 30–50 % over weeks: use line-to-line seats plus a dot of retaining compound or a retaining lip). Separately, 16 servos in a closed 150 mm forearm shell dissipate about 10 W. My estimate (natural convection, 8 W/m²K over 0.045 m²) is 25–35 K above the room, so **45–55 °C inside**. Prusament PETG has an HDT of 68 °C at both 0.45 and 1.8 MPa, so the loaded servo frame sits at about 70 % of it. Use **Bambu PAHT-CF** (HDT 194/170 °C, tensile modulus 3.9 GPa XY / 2.2 GPa Z) for `F_forearm_frame`, `F_spool_rail`, the 2 cassettes, the adjuster block and `F_arm_flange`: 48.7 g, about **EUR 2.5** of filament. Or cut those parts from 3 mm aluminium.

**2. Do not print 24 buried Ø2.2 curved bores in the palm. Print open channels face-up and snap the PTFE in.**
A 0.4 mm nozzle prints small holes 0.1–0.2 mm under size and faceted, so Ø2.2 comes out about 2.0–2.1 for a 2.0 OD tube. That is zero clearance on a 240° swept path: it jams (review #14). Replace the bore with a **keyhole groove**: a Ø2.05 circle with a 1.7 mm mouth open to the top face. Its lips overhang only 0.17 mm per side, so it prints with no supports, and the tube snaps in with a thumb. Routes that leave a single plane go in a **laminated palm**: 2–3 flat plates (each 4–5 mm) with grooves on one face, stacked and screwed, like a hydraulic manifold. Every plate prints groove-side up. Test it first on the coupon (`snap_groove_*`, 1.6 and 1.8 mouths, plus a 90° R15 curve).

**3. Print every finger lying on its side (joint axes vertical), design all horizontal holes as teardrops, and fix the print sizes from one coupon.**
Lying on the side makes the Ø8 bearing seats true circles *in* the layers (no ovality), puts the 7 mm drum rims in hoop direction (the tendon pulls along the layers) and stops the bending of the finger from loading layer interfaces. Standing the finger up would put the tip force across layers, and Prusament's own interlayer adhesion number is **18 ± 4 MPa against 47 MPa yield: Z is about 0.4 × XY** (design Z at 0.4, not the 0.7 some tables show). One coupon (about 40 cm3 of solid volume; with 15 % infill I expect 25-30 g and roughly 60-75 min on a Bambu/Prusa class printer, not 30: halve it by deleting the arcs, wall and bars in `04_coupon.py`) (`out/04/tendra_coupon.stl`) fixes: pin holes 2/3/4 mm at −0.10…+0.20, bearing seats MR84 7.90–8.10 and MR63 5.90–6.05 (vertical and horizontal), M2/M2.5/M3 inserts and pilots, 16G liner holes, tunnels, snap grooves, a 7 mm drum with grooves, a thin-wall ladder, and flat vs upright break bars. Then write the numbers into one `PRINT_COMP` table in `params.py`.

## 2. Idea list (wide, 30 ideas)

Gain figures are estimates from the hand calculations here unless a source is named.

| # | Idea | How it works | Gain (number) | Cost / effort | Risk | Verdict |
|---|---|---|---|---|---|---|
| 1 | PETG everywhere (keep) | E 1.5 GPa, yield 47 MPa, HDT 68 °C, easy | baseline | none | creep in hot frame | **Keep** for fingers, thumb, palm, covers |
| 2 | PAHT-CF for forearm frame, rails, cassettes, flange | PA CF composite, HDT 194/170 °C, 3.9 GPa XY | frame stiffness 2.6× PETG, no softening at 55 °C | EUR 2.5, hardened nozzle, dry 80 °C, 290 °C nozzle, enclosure | Z weak (2.2 GPa), moisture changes size | **Do** (or aluminium) |
| 3 | PETG-CF for fingers | 2.5× stiffer | tip deflection at 20 N: 1.6 → ~0.6 mm | carbon dust, abrasive: wrecks any tendon edge | line abrasion, Z strength drops | **No** for anything the line touches; optional for structural bodies |
| 4 | PLA anywhere in the hand | HDT ~55–60 °C | easier prints | creeps; tendon-mechanics.md rates it high creep | deforms in forearm | **Test parts only** |
| 5 | ASA / PC | Tg 100 / ~147 °C | HDT +30–50 K over PETG | enclosure, 260–280 °C, warp | big parts warp (palm, shells) | **Skip**; PAHT-CF already covers the hot parts |
| 6 | Print fingers on their side | joint axes vertical, seats are in-layer circles | seat roundness, no Z-loaded lugs | slicer support in pockets | knot-well pocket faces sideways (overhang) | **Do** |
| 7 | Teardrop / diamond for every horizontal hole | apex up at 45° | no sag, no supports | one CAD helper | bore centre shifts slightly up | **Do** |
| 8 | Open trough + snap-in keyhole groove in palm | Ø2.05 circle, 1.7 mouth | removes 24 jam-prone bores | medium CAD change | tube may pop out: add lid plate | **Do** (test on coupon) |
| 9 | Laminated palm (2–3 grooved plates) | 3D routes become stacked 2D routes | any route, face-up printing | adds plates + 6–8 screws | route stack-up thickness | **Do** if routes are not planar |
| 10 | Clamshell palm split at the channel plane | half-round grooves both faces | easy for planar routes | alignment pins | routes here are 3D | **Maybe** (only the planar sheaths) |
| 11 | Reamable straight segments | bores cut as straight runs ≤ 25 mm, Ø2.2 drill by hand | fixes size, not facets | tiny | curves still unreamable | **Do** for the few straight runs |
| 12 | Water-soluble support in the bore (PVA/BVOH) | dissolve to leave a smooth channel | smooth round bore | 2nd material, 24 h+ to dissolve 100 mm of Ø2.2 | blocks, swells | **No** |
| 13 | Oversize bore, tube free inside | Ø3.5 channel, tube takes own path | zero jam | tube position varies, changes strand length | routing accuracy | **No** (breaks constant-length claim) |
| 14 | Brass / steel tube drum sleeve (14 mm OD, 0.5 wall) | drum radius set by tube, not printer | radius error ±0.1 mm → ±0.02; 7 mm exact; no line wear | EUR 3 per hand, bond | glue joint | **Do** for MCP drums (or calibrate) |
| 15 | Epoxy / XTC-3D coat on printed drums | 0.05–0.1 mm coat | smooth groove, μ down | skill | radius changes by coat thickness | **Maybe** (only after calibration) |
| 16 | Captive hex-nut traps instead of heat-set inserts | hex pocket opens from side | strongest thread, 0 s install, 0 inserts | pocket needs a side opening | overhang, nut slides | **Do** for structural joints |
| 17 | Heat-set inserts M2 (hole 3.2) / M2.5 (3.5) / M3 (4.0–4.2) | soldering iron, 20 s each | repeated service | 60 inserts ≈ 20 min | hole size, pull-out in Z | **Do** for covers (hole sizes on coupon) |
| 18 | Self-tapping screws direct into PETG | pilot ≈ 0.8 × major | zero extra parts | strips after 3–5 cycles | wear | **Only** for never-removed parts |
| 19 | Pins with press fit instead of screws | Ø2/3/4 steel dowel, hole −0.05 | 0 fasteners | tolerance-sensitive | loosens from creep | **Do** for hinge axles with bearing, not for retention |
| 20 | Standardise fasteners: M2 for covers, M2.5 for structure, nothing else | fewer drivers/lengths | assembly time | none | none | **Do** |
| 21 | Merge small parts (cassette L/R, adjuster block, rail) | one part instead of 5 | −6 parts, −12 screws | bigger prints | print failure costs more | **Do** where < 100 mm |
| 22 | TPU pad printed separately, dovetail slide-on | mechanical key, no glue | pad can be replaced | tolerance 0.1 | pad slides out | **Do** |
| 23 | TPU printed on PETG in one job | multi-material, same nozzle swaps | no assembly | purge waste, direct-drive printer | bond unknown | **Test** on a coupon |
| 24 | Cast silicone pad (Dragon Skin 10 / Ecoflex 00-30) over a lattice key | μ ≈ 1+ | needed pinch force /3 (tendon-mechanics.md §7) | mould print + cast 4–5 h | tears | **Do** for fingertips (idea from agent on contact) |
| 25 | Print-in-place hinges | gap 0.3 mm | no bearings | rub, stiction | contradicts bearing design | **No** |
| 26 | Snap-fit lids for knot wells and palm covers | 0.3–0.5 mm hook | fewer screws | tune | breaks | **Do** for non-structural covers |
| 27 | Anneal PAHT-CF 80–130 °C 5–12 h | Bambu TDS recommendation | full stated HDT and strength | oven (not kitchen) | warp 0.5–1 % | **Do** for the frame |
| 28 | Variable layer height 0.12 mm around seats | finer circle, better inter-layer weld | seat roundness | +40 % time on fingers | none | **Do** (slicer height range modifier) |
| 29 | Extra perimeters (4–5) around seats and holes, seam hidden in a corner | stronger hoop | hoop stress 2× area | +5 % time | none | **Do** |
| 30 | Printed press jigs and assembly fixtures | bearing press tool, pin pusher, tube cutter guide | −50 % damaged bearings | 3 small parts | none | **Do** |

## 3. Deep dives

### 3.1 Materials: what the numbers say

Datasheet values (printed bars, XY unless noted). Items marked **(web)** I checked on 2026-11-14; the rest are typical vendor values from memory, so confirm on the spool you buy.

| Material | Tensile modulus | Yield / break | Z behaviour | Heat | Creep under steady load | Moisture | Printability (hobby) |
|---|---|---|---|---|---|---|---|
| PLA | 2.9–3.6 GPa | ~50 MPa | interlayer ≈ 0.4–0.6 × XY | HDT ~55–60 °C | worst of the group, visible at 40 °C | low | easiest |
| **PETG (Prusament)** | **1.5 GPa XY, 1.6 GPa Z (web)** | **47 ± 2 MPa XY, 50 vertical (web)**; **interlayer 18 ± 4 MPa (web)** | interlayer 18 MPa = 0.38 × yield | **HDT 68 °C at 0.45 and 1.8 MPa (web)** | moderate; Tg ~80 °C, so 50 °C is 60 % of Tg | slow, dry 65 °C 6 h | easy, stringy |
| PETG-CF | 3.5–6 GPa | 50–70 MPa | Z worst of the group (fibres lie in layers) | HDT 75–85 °C unannealed | low (fibre carries load) | like PETG | hardened nozzle; abrasive |
| **PA6-CF / PAHT-CF (Bambu PAHT-CF)** | **3.86 GPa XY, 2.18 GPa Z (web)** | 70–100 MPa XY | modulus Z/XY = 0.56 (web) | **HDT 194 °C at 0.45, 170 °C at 1.8 MPa; Vicat 220 °C (web)** | low | **absorbs 1–3 % water**: dry before print, store sealed | 290 °C nozzle, 100 °C bed, enclosure |
| PA12-CF | 4–6 GPa | 70–90 MPa | like PAHT-CF | HDT 140–160 °C | low | less water than PA6 (~0.7 %) | 270–290 °C |
| ASA | 2.0–2.2 GPa | ~40 MPa | 0.6–0.8 | HDT ~95 °C | moderate | low | warps, enclosure |
| PC | 2.0–2.3 GPa | 60+ MPa | 0.6 | HDT 110–115 °C | low | high | 260–280 °C, enclosure |
| TPU 95A | ~20–40 MPa (soft) | 500 % elongation | n/a | softens ~80 °C | high, fine for pads | high | direct drive, 20–30 mm/s |
| Cast silicone (Dragon Skin 10 / Ecoflex 00-30) | < 1 MPa | tear-prone | n/a | n/a | n/a | none | pot life 20–45 min |

How I used these:
- **Creep rule.** For amorphous plastics (PLA, PETG, ASA, PC) I keep sustained stress below about 0.2 × the short-term yield, and 0.1 × for Z-loaded features. This is an engineering rule, not a measurement. PETG then allows 9 MPa along the layers and about 4 MPa across them. Everything in the finger is far below that. The exceptions are the four contacts in the first recommendation (see `04_creep_stress.py` rows 1–4).
- **Stiffness.** A finger root of 18 × 16 mm with an assumed 1.2 mm solid wall has I ≈ 2 870 mm⁴. A 20 N tip load at 100 mm bends it 20·100³ / (3·1500·2870) ≈ **1.6 mm in PETG** and about 0.6 mm in a 4 GPa CF grade. The real finger has slots and pockets, so call it 2–4 mm. This matters for tip position at 20 N, not for breakage (stress at 20 N is about 6 MPa).
- **Heat.** My estimate of 45–55 °C inside the forearm assumes about 10 W of servo heat (unverified: measure with a thermocouple on one servo held at load) and natural convection. If true, only the CF-nylon grades keep their stiffness with margin. The wrist STS3215 servos are warmer still.
- **Pads.** TPU 95A is easy but its friction is only moderate (μ about 0.5–0.7, "verify" per tendon-mechanics.md). Cast silicone (μ ≈ 1) cuts the needed pinch force about 3×. Since fingertip force is the hand's gap vs commercial hands, the pad matters as much as any structural choice. Print the pad core in TPU with a lattice and cast Ecoflex 00-30 or Dragon Skin 10 on it.
- **Filament cost.** The 255 g of printed plastic in `mass.json` (PETG 240 g, TPU 15 g) is **EUR 5–6 of PETG** at EUR 20/kg. Even three iterations plus failures cost under EUR 30. Cost is not the constraint: print time and assembly time are.

### 3.2 Orientation and anisotropy

Rule: the load path should follow the layers, and no round feature that carries a bearing, pin or screw should have its axis in the horizontal plane.

Where Z would fail first:

| Feature | Failure if axis is wrong | Fix |
|---|---|---|
| Bearing hub / printed axle stub (Ø4–5, cantilevered) | bending is tension across layers; the repo already warns of this (tendon-mechanics.md §6) | steel stubs / through-pins (V2 already does this) |
| Lugs of the yoke (3.8 mm walls) | tip force bends a lug about an axis parallel to the layers if the finger stands upright | print on the side so the lug plate lies in the layer plane |
| Knot post / tie-hole wall (0.6–0.7 mm walls in review) | wall below 1.6 mm = 1–2 perimeters, layers peel | steel liner (already planned) plus ≥ 1.6 mm wall |
| M2 screw pulled in Z in a printed boss | pull-out across layers | heat-set insert or nut trap with the thread axis horizontal |
| Gear teeth (wrist, review #3) | teeth in a vertical gear are Z-loaded | print gears flat, teeth in the layer plane, in PAHT-CF; better, a steel gear |
| Palm shell | tension across layers where sheath stops push on the shell | printing "on its back" (as `palm.py` says) puts sheath axes in the build plane for planar runs |

### 3.3 Tolerances and the coupon

FDM rule of thumb for a 0.4 mm nozzle (my experience, to be confirmed): holes come out smaller than modelled, by roughly 0.1–0.2 mm for Ø2–4, 0.05–0.15 mm for Ø8; pins and bosses come out slightly larger; horizontal holes sag at the top. Starting values to check on the coupon:

| Feature | Model value to try first | Coupon ladder |
|---|---|---|
| Ø2 / Ø3 / Ø4 steel pin, press | hole 1.90 / 2.90 / 3.90 | −0.10 … +0.20 in six steps |
| Ø2 / Ø3 / Ø4 pin, sliding | 2.10 / 3.10 / 4.10 | same ladder |
| MR84ZZ seat (OD 8, vertical axis) | 7.95 (current `BEARING_SEAT_D`) | 7.90, 7.95, 8.00, 8.05, 8.10, 3.0 deep, Ø6.75 shoulder |
| MR84ZZ seat, horizontal axis | 8.00, teardrop | 7.95 / 8.05 round and teardrop |
| MR63ZZ seat (OD 6) | 5.95 | 5.90 … 6.05, 2.5 deep, Ø4.9 shoulder |
| M2 heat-set insert | 3.2 | 3.0 – 3.3 |
| M2.5 insert | 3.5 | 3.4 – 3.7 |
| M3 insert | 4.0–4.2 | 4.0 – 4.3 |
| M2 / M2.5 / M3 self-tap pilot | 1.7 / 2.2 / 2.6 | 1.6–1.8, 2.1–2.3, 2.5–2.7 plus clearance holes |
| 16G liner (OD 1.65) tie hole | 1.65–1.70 | 1.60, 1.65, 1.70, 1.75 |
| PTFE 2.0 OD sheath bore | 2.3 straight, round | Ø2.2 round, teardrop, diamond; Ø1.2 teardrop and round; vertical 2.2 and 1.2; R15 90° teardrop |
| Snap-in groove for 2.0 OD tube | circle 2.05, mouth 1.7 | mouths 1.6 and 1.8 straight, 1.7 on R15 curve |
| Line groove on the 7 mm drum | 1.0 wide | 0.8, 1.0, 1.2 on a true r = 7 drum, plus a Ø1.7 radial tie hole |
| Walls | ≥ 1.6 | fins 0.4, 0.8, 1.2, 1.6, 2.0 |
| Layer strength | n/a | 2 flat and 2 upright 3 × 2 mm break bars |

**The coupon** (`cad/04_coupon.py`, run with `uv run --no-project --with cadquery --with numpy python cad/04_coupon.py`; build takes about 5 minutes). One plate of about 140 × 98 mm, 0.6 mm base, blocks 5–6 mm high, tower and wall 10–12 mm. Built volume: 39.8 cm3, bounding box 142 × 98 × 12.6 mm, valid solid (the STL is 2.2 MB). Print in PETG, 0.2 mm layers, 3 walls, 15 % infill, no supports, no brim. The build log prints the real volume; `coupon_legend.json` lists the position, diameter and purpose of every feature, and raised labels on the plate give the offsets in hundredths of a mm and the row letters. How to read it: push the real steel pin, the real bearing and the real tube into each hole. Record the first size that (a) presses in with hand force, (b) slides, (c) falls out. Put the result in `params.py` as `PRINT_COMP`.

### 3.4 Buried curved channels (the palm)

Why they jam: the V2 palm has 24 swept Ø2.2 bores for 2.0 OD PTFE (`palm.py` `channel_solids`). If a Ø2.2 hole prints 0.1–0.2 mm small, the clearance on a 2.0 tube is 0 to 0.1 mm. The tube has to pass up to 240° of turning at R ≥ 15, and the facets from the slicer add 0.03–0.05 mm of roughness. A thin PTFE tube pushed through that wedges. Additionally a horizontal circular bore sags at its top, so the roof is rough.

| Option | Prints how | Supports | Tube fit | Verdict |
|---|---|---|---|---|
| Buried round bore (V2) | bores in the build plane for planar runs; stair-steps on rising runs | roof sags | zero clearance | reject |
| Buried teardrop bore | apex up | none | clearance if bore is 2.3+ | OK only for short straight runs |
| **Open trough face-up, snap-in keyhole** | groove up | **none** | tube snaps in, replaceable | **best** |
| Clamshell halves | half-round grooves on two mating faces | none | needs planar routes | partial |
| **Laminated plates** | each plate groove-up | none | tube sandwiched | **best for 3D routes** |
| Soluble support | printed-in | PVA | smooth | reject |
| Oversize free bore | any | none | tube free | reject (length changes) |

Keyhole geometry: tube OD 2.0, circle Ø2.05 centred 1.3 mm below the face, mouth 1.7 wide. The lips overhang 0.17 mm per side. That overhang is below one layer of 0.2 mm, so it prints cleanly. Wall under the groove ≥ 1.6, wall between neighbouring grooves ≥ 1.6, so pitch ≥ 2.05 + 1.6 = 3.7 mm. At R15 a tube lies along the outer wall; the snap lip keeps it from lifting under 45 N Bowden load when the groove is also closed by a lid or the next plate (which should press 0.1 mm on the tube).

Bench test (on the coupon): press 100 mm of tube into the R15 groove, turn the plate upside down, shake. Then pull a 0.36 mm line with a spring scale through the tube in the curve and compare to a straight tube: friction should match the capstan value for μ = 0.1 (tendon-mechanics.md). Kills the idea if the tube pops out under 2 N of side load or the lip cracks.

### 3.5 Part count, fasteners and assembly time

Inventory from `out/assembly/mass.json` (266 parts): **69 printed** (53 PETG, 16 TPU) and **197 others**: 82 steel items (pins, eyelets, liners, screws), 53 PTFE pieces, 41 bearings, 18 servos, 3 electronics. So the hand is assembled from far more small bought parts than printed parts. Time estimate (mine; needs a stopwatch run): 41 bearings and 25 pins at about 1 min each = 1.1 h; 32 strands (cut, thread through both sheaths, tie the knot, route, tension at the spool) at 8–10 min = **4.5–5.5 h**; sheath fitting and stops 1.5 h; 16 spools clamped 1 h; palm, forearm, wrist screws 1.5 h; total about **10–12 h**, dominated by the strands. Where I can cut it:
- Strands: ferruled sheath stops that click into the keyhole, and a pre-made line with a swaged stop on one end (pre-tied loops are made at the bench from one 1 m length each, 32 in an evening).
- Bearings: a printed press jig (a Ø8.5 guide with a Ø6 punch) avoids cocked bearings and cracked seats.
- Fasteners: M2 pan head for covers, M2.5 + nut trap for structure. Heat-set inserts only where a part is unscrewed more than ~5 times.

### 3.6 Printing times and cost

From `mass.json`: PETG parts 240 g in total (fingers 47, thumb 24, palm 46, wrist 6, forearm 118), TPU 15 g. Estimate: big forearm and palm parts (164 g) at 25 g/h = 6.5 h; fingers, thumb, wrist (91 g, many small parts, 0.16 mm layers) at 10–12 g/h = 8–9 h; plus supports, bed changes and failures +30 % → **about 20 h of printing, over 3–4 days, EUR 6 of filament** for one hand; for a realistic three iterations, 1 kg of PETG, 0.25 kg of PAHT-CF and one TPU spool. Plastic is cheap; the 41 bearings, 18 servos and PTFE are the cost.

### 3.8 Table: part, material, orientation, infill and walls, why

Settings: 0.4 mm nozzle; "walls" = perimeters; layer 0.2 mm unless noted. Masses are from `mass.json`.

| Part (V2 name) | Material | Orientation | Infill / walls | Why |
|---|---|---|---|---|
| `*_proximal`, `*_middle`, `*_distal`, `*_yoke` (16 finger parts, 47 g) | PETG | on the **side** (joint axes vertical; DIP-bar side down if it is the flat one) | 25 % gyroid, 4 walls, 0.16 mm layers | seats are in-layer circles; drum rims and lugs load along layers; 4 walls = 1.6 mm shell; teardrop bores; supports only in knot wells |
| Finger pads `*_pad_*` (16 parts, 15 g TPU) | TPU 95A (or cast silicone over a TPU lattice) | flat, pad face on the bed | 100 %, 2 walls, 20 mm/s | compliant, soft side up or down per mould; dovetail key into the finger |
| `T_thumb_mount`, `T_thumb_base` (sleeve OD 20) | PETG | sleeve axis **vertical** | 30 %, 4 walls | journal and bearing seats are hoops in the layers; heat-set or nut-trap pockets with vertical axis |
| `T_thumb_metacarpal`, `_proximal`, `_distal` | PETG | on the side, as fingers | 25 %, 4 walls | same as fingers; thin wall 0.8–1.2 mm near the cmc groove must go to ≥ 1.6 or steel washer flanges |
| `P_shell` (palm, 44 g) | PETG | **back down**, channels open face-up | 20 %, 3 walls | grooves face up need no supports; knuckle lips and curves in the build plane (as `palm.py` states) |
| `P_back_plate`, laminated plates (new) | PETG | flat, groove side up | 20 %, 3 walls | snap-in keyhole grooves print support-free |
| `W_wrist_base`, `W_wrist_ring`, `W_palm_flange` (6 g) | PETG (gear parts: PAHT-CF or metal) | ring axis vertical; gears flat | 40 %, 5 walls | bearing seats in-layer; gear teeth in-layer; wrist load is the highest in the hand (review #3) |
| `F_forearm_shell_back`, `F_forearm_cover_front` (29 g each) | PETG | outer face down | 15 %, 3 walls, vent holes | large and thin; low load; hot inside, so vents; ribs instead of infill |
| `F_side_cover_R/L` | PETG | flat | 15 %, 3 walls | snap fit |
| `F_forearm_frame`, `F_spool_rail`, `F_cassette_R/L`, `F_adjuster_block`, `F_arm_flange` (49 g) | **PAHT-CF** (or 3 mm aluminium) | flat on the bed; servo screw holes vertical with metal inserts or through-bolts | 40 %, 5 walls, annealed | hottest region, sustained spool side loads, needs stiffness; CF never touches the line |
| `F_spool_01…16` (0.3 g each) | PETG or PA12 | axis vertical | 100 %, 6 walls | tiny, creep-sensitive; groove is a hoop; horn bore Ø4 on the coupon value |
| Sheath ferrules, tie liners, pins, eyelets | brass / stainless | bought | n/a | metal where the line or high stress touches |
| Press jigs (new) | PETG | flat | 100 % | cheap fixtures |

### 3.7 Post-processing

- **Reaming**: Ø1.2 and Ø2.2 twist drills in a pin vise for straight bores; an 8.00 H7 reamer or a Ø8 drill by hand for seats only if a coupon shows < 7.95; a 90° countersink on tube mouths removes the sharp edge.
- **Polishing tendon paths**: wet-sand 400 → 1 000 grit along the groove; do not use CF parts there. Flame-polish nothing: PETG goes cloudy and PTFE emits fumes.
- **Vapour smoothing**: not for PETG (acetone barely works, and cracks bearing seats on ABS/ASA). Skip.
- **Epoxy-coating drums**: works but sets the final radius; I prefer a brass sleeve (idea 14). If used, measure the coated drum with calipers and write its radius into the calibration.
- **Annealing**: PAHT-CF 80–130 °C for 5–12 h (Bambu TDS). PLA annealing shrinks and warps by 2–5 %: not for precision parts. PETG: skip.

## 4. What to build as CAD next

1. **Print the coupon** (`out/04/tendra_coupon.stl`), fill `PRINT_COMP`, rebuild nothing else yet.
2. **`snap_groove(tube_od, mouth, depth)`** helper in `params.py` or `palm.py`: Ø2.05 keyhole, mouth 1.7, wall ≥ 1.6; replace `channel_solids("bore")` for planar runs; for 3D runs split the palm into 2–3 plates, each ≥ 4 mm thick, with M2.5 nut traps at 8 positions.
3. **`hole(d, axis)`** helper: returns a round hole for Z-axis holes, a teardrop for X/Y axis; applies `PRINT_COMP`. Use it in every finger and thumb bearing seat.
4. **Brass drum sleeve** (14 OD × 0.5 wall × 6 long, groove turned or a printed flange pair): seat Ø13.2, press and retain with CA.
5. **Ferruled sheath stops**: Ø2.6 × 3 long brass or steel ferrule on a Ø2.7 shoulder, flanged (Ø4.0) where the wall allows.
6. **Spool clamp**: steel washer Ø5 under each M2 head, thread-lock; or a captive M2 nut, so the clamp is metal-on-metal.
7. **Forearm frame, spool rail, cassettes, adjuster, flange** as one PAHT-CF set (or 3 mm aluminium).
8. **Press jig** for MR84ZZ and MR63ZZ (3 small parts).
9. Walls to thicken (review list): thumb MCP tie wall 0.7, thumb spacer boss 0.6, finger knot-well wall 0.6, pin A web 0.28 → ≥ 1.6, or use a steel bushing.
10. **A DFM lint script** (`cad/04_dfm_lint.py`, not written here): load all STEP parts, flag walls < 1.6, holes < 1.0, unsupported overhangs > 45°, bores that are not round or teardrop.

## 5. Sources

Checked on 2026-11-14:
- Prusa Research, Prusament PETG technical data sheet (v1.1, 2022): HDT 68 °C at 0.45 and 1.8 MPa, yield 47 ± 2 MPa (horizontal) and 50 ± 1 MPa (vertical), modulus 1.5 / 1.6 GPa, interlayer adhesion 18 ± 4 MPa. https://cdn.shopify.com/s/files/1/1745/6181/files/PETG_TechSheet_ENG.pdf
- Bambu Lab, PAHT-CF technical data sheet V2.0: modulus 3 860 ± 230 MPa XY and 2 180 ± 130 MPa Z, HDT 194 °C (0.45 MPa) / 170 °C (1.8 MPa), Vicat 220 °C, print 290 °C nozzle / 100 °C bed, anneal 80–130 °C for 5–12 h (dry-state values). https://uwaterloo.ca/rapid-prototyping-centre/sites/default/files/uploads/documents/bambu_paht-cf_technical_data_sheet_v2.pdf

From the repo (read): `research/references/hands/tendon-mechanics.md` §6–7 (printing materials, pads, CF abrasion), `cloud-claude/2026-11-09-v2-hand-cad/reports/05-review.md` #14, `06-bearings-track.md` (seats Ø7.95, shoulder 6.75, MR63 in the palm), `cad/params.py`, `palm.py`, `out/assembly/mass.json`.

From memory, not checked (verify on the spool or TDS you buy): PLA, PETG-CF, PA12-CF, ASA, PC, TPU values in the table; insert hole sizes (vendor guidance such as Ruthex/CNC Kitchen); Smooth-On Dragon Skin 10 / Ecoflex 00-30 hardness and pot life; my estimate that FDM holes print 0.1–0.2 mm small. The 45–55 °C forearm temperature is a hand estimate, not a measurement. The coupon exists to replace the "from memory" numbers with your printer's.
