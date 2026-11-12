# 02 — Thumb (Tendra V2 concept, study 2026-11-09)

Thumb designer's report. Code: `cad/thumb_kinematics.py` (kinematics, tasks, optimiser, strength) and
`cad/thumb.py` (CadQuery model, tendon routing, decoupling + collision checks, renders, interface).
Outputs: `out/thumb/` (STEP assembly `thumb_rest.step`, `stl/*.stl`, `thumb_rest.glb`, renders `*.png`,
`thumb_design.json`, `routing.json`, `interface.json`, `collisions.json`, envelope hulls).

Run: `uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy
--with pillow python cad/thumb.py` (add `--quick` for a 9-pose collision sweep, `--no-render`).
`python cad/thumb_kinematics.py --x out/thumb/thumb_opt_x.json` re-finalises the design from an optimiser
vector; `--optimise` re-runs differential evolution (~1–2 h on 4 cores).

## 1. Summary

- **Kinematics: a "roll-pitch" CMC.** `cmc_rot` turns the whole thumb about an axis a1 that points
  distal-radial-palmar (44° from +Z), almost along the metacarpal (8.7° off at rest), like the human
  trapezium's long axis. `cmc_flex` is perpendicular to a1 and **intersects it** at the CMC centre P. Turning
  about a1 sweeps the metacarpal on a cone and pronates it at the same time, which is what human opposition
  (circumduction + conjunct pronation) does. The MCP axis is twisted 51.5° about the metacarpal relative to
  the cmc_flex axis, the IP axis another 6.7° (the human IP axis is oblique too). All 12 geometry numbers came
  out of a differential-evolution search (`thumb_kinematics.py`).
- **Opposition (Kapandji):** the thumb pad meets each finger pad within **0.1–2.0 mm** with pads facing within
  **2–40°** (V1 thumb: 2.4–19 mm and 69–156°, i.e. V1 cannot oppose the ring or little finger at all).
  Index tip-to-tip 0.0 mm / 0°, key pinch 0.9 mm / 12°, power grasp of 40/55/70 mm cylinders with the pad on
  the far side (153–194° around the cylinder).
- **Routing, the V1 problem, is solved by geometry, not by sheath loops.** Every strand of cmc_flex, mcp_flex
  and ip comes up **the cmc_rot axis** through one PTFE neck eyelet, the mcp/ip strands cross cmc_flex **on its
  axis** (which a1 hits, so they arrive straight up the rot axis) and the ip strands cross the MCP on its axis.
  There is **no free sheath anywhere in the thumb**. Worst strand length change vs any other joint:
  **0.123 mm**; worst real coupling (half the flex–ext difference) **0.011 mm = 0.09°**.
- **Strength is the weak point**, as for every SCS0009 joint: ~**5.5 N** pad force in a pinch (V1 4.4 N), 7 N in
  the power grasp. HLS3606M on all four thumb joints gives ~14 N. Details and the options in §6.
- **Mechanics:** 2 × 6704 thin-section bearings on a printed Ø20 sleeve at the CMC (compact: the whole stack
  ends 37 mm below P, inside the palm), 3 mm steel stubs for cmc_flex, 2 mm stubs at MCP/IP, 1 mm steel nip
  pins at the two on-axis crossings, crimp-bead anchors, printed hard stops on every joint (they engage 2–10°
  past the soft limits), TPU tip pad and thenar pad. All 21 parts are single valid, watertight solids.

## 2. What I took from humans and from other hands

| Source | Idea | Used here |
|---|---|---|
| Human CMC (saddle joint, Hollister 1992) | two non-orthogonal, non-intersecting axes; opposition = abduction + flexion + **pronation**, the pronation coming from the oblique axes | the oblique rot axis a1 along the metacarpal gives the pronation for free; I kept the two CMC axes **intersecting and orthogonal** on purpose (routing, see §4) and put the human obliquity into the MCP twist (51.5°) instead |
| Human proportions | metacarpal 46, proximal 33, distal 28 mm | optimiser chose 46.7 / 36.0 / 31.6 mm (bounds 40–54 / 30–38 / 26–32) |
| Kapandji test | thumb tip to each fingertip pad | the optimiser's main objective (§3) |
| 1X NEO (thumb CMC 3.5 N·m, "biased toward a thumb that genuinely opposes") | the thumb base is the most loaded joint; layout first | strength table (§6) shows cmc_flex/cmc_rot need the most torque |
| Tesla Optimus V3 patent | cables switch over **exactly at the joint axis** (wrist) to kill crosstalk | the same idea at both CMC joints and the MCP |
| Halodi / 1X, Shadow | antagonistic pair on one motor, Bowden sheaths | antagonistic loops kept; Bowden sheaths only from the forearm to the thumb mount, none in the thumb |
| ORCA / V1 concept B | sheath loops outside the thumb | rejected: 100 mm of moving slack tube, snags, ≥ 0.3 mm/rad length drift |
| V1 thumb (`research/experiments/2026-09-29-thumb-routing`) | hollow rot journal (concept A) | kept and pushed further: with a1 ∩ a2 the strands never need a sheath loop around cmc_flex |

## 3. Kinematics and the optimisation

**Model** (`thumb_kinematics.py`): product-of-exponentials chain for the thumb; the fingers from
`params.FINGERS` (abduction about Y 10 mm proximal of the MCP, flexion about X, DIP = 0.75 PIP). Pads are
points on the palmar surface (thumb 8 mm from the IP axis line incl. a 2.5 mm pad, fingers h/2 − 1) with
normals. Tasks, each a bounded least-squares over the thumb's (and the finger's) joints with penetration
penalties (thumb capsules vs the palm slab, the target finger, the cylinder):

- Kapandji pad-to-pad for each finger, finger in a human "curled" family (abd −5…15, MCP 20…75, PIP 25…85,
  without that the optimiser cheated with MCP 90° / PIP straight);
- index tip-to-tip; key pinch (thumb pad on the radial side of the curled index's middle phalanx);
- power grasp of Ø40/55/70 cylinders lying on the palm, axis oblique, pad on the far side (≥ 150° around it),
  contact over the index/middle region;
- a "flat" pose (thumb out of the way, tip x ≥ 62, near the palm plane).

Score = Σ gaps + 0.2·pad angles (+ weights) + 0.04·(CMC range used) + a mount penalty (the CMC stack, 36 mm
along −a1, must stay above z 18, i.e. inside the palm, not in the wrist gap). 12 design numbers: P (3), a1 (2),
a2 angle about a1, metacarpal angle α0, MCP twist τ3, IP twist τ4, L1, L2, L3. Differential evolution
(4 cores), then Nelder–Mead. Raw vector: `out/thumb/thumb_opt_x.json`; the finalised design (rest pose,
ranges, signs) `out/thumb/thumb_design.json`.

**Result** (hand frame, mm; q = 0 rest):

| | point | axis | range (deg) |
|---|---|---|---|
| cmc_rot | P = (39.8, −17.9, 50.4) | (0.410, −0.565, 0.716) | −31 … 110 |
| cmc_flex | P (same point) | (0.132, −0.740, −0.659) | −37 … 63 |
| mcp_flex | (52.3, −46.5, 85.1) | (−0.829, 0.246, 0.502) | −10 … 70 |
| ip | (61.9, −68.6, 111.8) | (−0.881, 0.156, 0.447) | −15 … 85 |
| tip | (70.4, −88.0, 135.3) | | |

P is inside `THUMB_CMC_REGION` in x and y but **z 50.4 is above its 45 limit** (requested change, §9): with
the CMC lower, the 37 mm bearing stack would hang into the wrist gap.

**Rest pose q = 0:** MCP and IP straight, the thumb stands out in front of the thenar (C-ready posture) with
its pad facing back toward the palm. The "out of the way" pose (palm open flat) is
q = (26, 2, 40, 47). Positive = closing for every joint (checked: positive cmc_rot moves the tip across the
palm toward −X, positive flexions curl toward the fingers).

### Opposition table (gap mm / pad-normal angle deg; 0/0 = pads touch and face each other)

| task | V1 thumb (mapped into this frame, V1 ranges) | **V2 design** | V2 joint angles (rot, flex, mcp, ip) |
|---|---|---|---|
| index pad | 2.4 / 69 | **2.0 / 40** | −16, 29, 23, −15 |
| middle pad | 6.0 / 84 | **0.5 / 20** | −17, 29, −10, −15 |
| ring pad | 14.6 / 156 | **1.2 / 16** | −13, 41, −10, −15 |
| little pad | 19.0 / 93 (+7.9 mm into the palm) | **0.1 / 2** | −22, 54, −10, −15 |
| index tip-to-tip | 0.5 / 21 | **0.0 / 0** | 20, 29, 26, 35 |
| key pinch | 1.6 / 28 | **0.9 / 12** | 103, −30, 55, −15 |
| power Ø40 / Ø55 / Ø70 (pad on far side) | 0 / 0 (V1 can reach a cylinder) | **0.0 / 0, 0.1 / 1, 0.1 / 1** | e.g. Ø55: 49, 51, 50, 51 |
| score (lower = better) | 191 | **32** | |

Renders: `pose_index.png`, `pose_middle.png`, `pose_ring.png`, `pose_little.png`, `pose_tip_pinch.png`,
`pose_lateral.png`, `pose_power55.png` (finger and palm stand-ins from the kinematic model),
`thumb_rest_hand.png`, `thumb_rest_detail.png`, `cmc_section.png`.

Notes: the index pad is the hardest (40°: the thumb meets the index pad a bit side-on), the pad oppositions use
MCP −10° and IP −15° (the human thumb also hyperextends its IP in a pad pinch), and the key pinch needs
cmc_rot near 103°, which is what makes the rot range 141° wide. Dropping the key pinch would shrink it to
~85°.

## 4. Tendon routing and decoupling

**Layout** (see `cmc_section.png`, `thumb_rest_detail.png`; every number from `routing.json`):

1. **cmc_rot** (2 strands): the drum is a 7 mm groove pair on the base just above the bearings; the strands
   leave tangentially in the drum plane into two sheath sockets in the mount (straight, Ø2.2). They end on
   the base, so nothing else moves them.
2. **Up the rot axis** (6 strands: cmc_flex, mcp, ip × flex/ext): the 6 PTFE sheaths from the forearm end in a
   hex bundle (r 2.3) in the mount, 6 mm below the **neck**: a PTFE eyelet (Ø1.6 hole, rounded) on the rot axis
   that turns with the base. cmc_rot only twists the six bare strands about each other between the sheath ends
   and the neck: **0.109 mm** worst length change over the 141° range.
3. **cmc_flex** (2 strands): from the neck straight to the tangent points of the 7 mm drum grooves on the
   metacarpal hub (x = ∓4 mm). Base-fixed path, so only cmc_rot (0.109) touches them.
4. **Crossing cmc_flex on its axis** (4 strands): because a1 passes through the cmc_flex axis at P, the mcp
   and ip strands arrive at the axis straight from the neck and cross it at x = −1.8, −0.6, 0.6, 1.8 mm in a
   central slot of the metacarpal hub, between two **1 mm steel nip pins** (centres 0.71 mm from the axis,
   aligned with the mid-range bisector). The only error left is the wrap on the pin:
   **0.103–0.123 mm** over 100° of cmc_flex, and it is the same on the flex and ext strand of a loop.
5. **Crossing the MCP on its axis** (ip strands): same nip at the MCP hub, **0.076–0.080 mm** over 80°.
6. **Drums**: one 7 mm drum per joint, centred on its axis, on the child; travel **0.1222 mm/deg** for every
   strand (= `params.MM_PER_DEG`), flex + ext constant to **≤ 0.10 mm** over the joint's own range.

**Decoupling table** (max length change of each strand while the column joint sweeps its range, worst of the
other joints at rest / mid / max; mm):

| strand | cmc_rot | cmc_flex | mcp_flex | ip |
|---|---|---|---|---|
| cmc_rot flex / ext | own (0.1222 mm/°) | 0.000 | 0.000 | 0.000 |
| cmc_flex flex / ext | 0.109 | own | 0.000 | 0.000 |
| mcp_flex flex / ext | 0.109 | 0.103 / 0.123 | own | 0.000 |
| ip flex / ext | 0.109 | 0.106 / 0.109 | 0.080 / 0.076 | own |

All are under the 0.2 mm target. What actually turns a joint is half the flex−ext difference:
**≤ 0.011 mm = 0.09°** (mcp_flex from cmc_flex), everything else ≤ 0.03°. The rest is a common-mode change in
loop tension (≤ 0.12 mm), which a soft extensor spring or the split-spool tensioner absorbs.

**Sheath bend radius:** no sheath bends inside the thumb (the only PTFE in the thumb are straight Ø2.2
liners in the metacarpal/proximal channels). The 15 mm rule applies to the palm integrator's path from the
exits below (all 6 bundle sheaths leave straight along −a1).

**Friction budget (ip strand, the worst):** neck convergence ~20°, cmc_flex nip up to ~60°, MCP nip up to
~45°, all on PTFE / polished steel (μ ≈ 0.1) → e^(0.1·2.2) ≈ 1.25, i.e. ~80 % of the spool force reaches the
IP drum, before the palm/wrist sheaths. V1's thumb ip strand was ~37–63 % (`tendon-mechanics.md`).

## 5. Mechanics

| item | choice | why |
|---|---|---|
| cmc_rot bearing | 2 × 6704ZZ (20 × 27 × 4) back to back on a printed Ø20 sleeve; base cap with 3 × M2 clamps the lower inner race | the thumb's biggest moment (~1 N·m at 10 N tip force) on a 27 mm bearing pair; the hollow Ø8 bore takes the strands; the stack is only 37 mm deep |
| cmc_flex | two Ø3 steel stubs pressed into the base cheeks, 4.5 mm into the hub (running fit +0.15) | the centre must stay free for the 4 crossing strands (no through-pin on the axis) |
| MCP / IP | Ø2 steel stubs in the fork cheeks | same reason at the MCP; IP for symmetry |
| crossings | 1 mm steel nip pins (music wire or dowel), 4.6–6.6 mm long | strand touches only steel; keeps it on the axis in both bend directions |
| anchors | crimp bead in a radial Ø2.2 pocket at ≥ 25° of wrap past the last tangent point | no knots in PETG; the capstan wrap protects the crimp |
| hard stops | lugs on each child hub hitting the parent's end face; rot: a lug on the base plate between two posts on the mount | measured first contact: cmc_rot +2/+2°, cmc_flex +4/+10° (lug trimmed −10° after a check), mcp +2/+2°, ip +2/+2° past the soft limits (`stops.json`) |
| pads | TPU 95A (or cast silicone A30) tip pad 2.5 mm, broad (15 mm) and wrapping the tip; TPU thenar pad on the metacarpal's palmar face | thenar pad is the contact for power grasps (the cylinder rests on it, §3) |
| material | PETG, walls ≥ 1.6 (thinnest wall: ~0.8 mm between the cmc grooves and the central slot) | see failure modes |

**Collisions** (`collisions.json`): 28 poses (rot × flex at min/mid/max, MCP+IP at min/mid/max, and rest):
**0 of 28 poses with interference** (threshold 0.5 mm³; pins in their own holes, bearings in their seats and
pads on their seats are the only pairs excluded). Before the stop-lug trim the cmc_flex lugs touched the plate
8° early (4.4 mm³ at the limit); the trim fixed it.

**Swept envelope for the palm integrator:** `out/thumb/thumb_envelope_hull.stl` (convex hull of everything
that moves, bounds x −48…139, y −129…27, z −12…159) and `thumb_base_sweep_hull.stl` (the base alone over the rot
range, bounds x 18…55, y −33…5, z 25…65: the bay must clear this). The envelope's z −12 comes from one corner
(rot 110, flex 63, MCP/IP extended: the thumb points back past the wrist); that corner is useless and should
be blocked in firmware (§7).

## 6. Strength

Joint torque per newton of pad force (J^T·n at the task pose), and the pad force at stall
(SCS0009 0.226 N·m → 0.285 N·m at the joint through 5 → 7 mm at 90 %; HLS3606M 0.59 → 0.743 N·m):

| pose | τ/N cmc_rot | cmc_flex | mcp | ip | F, all SCS | limit | F, all HLS |
|---|---|---|---|---|---|---|---|
| index pad | 0.049 | 0.052 | 0.052 | 0.017 | **5.4 N** | cmc_flex | 14.2 N |
| middle pad | 0.050 | 0.050 | 0.052 | 0.017 | 5.5 N | mcp | 14.3 N |
| little pad | 0.072 | 0.050 | 0.052 | 0.017 | 3.9 N | cmc_rot | 10.3 N |
| tip-to-tip | 0.048 | 0.050 | 0.050 | 0.015 | 5.7 N | cmc_flex | 14.8 N |
| key pinch | 0.031 | 0.048 | 0.052 | 0.017 | 5.5 N | mcp | 14.3 N |
| power Ø55 | 0.023 | 0.014 | 0.040 | 0.017 | 7.1 N | mcp | 16.4–18.6 N |
| V1 thumb, index pad | | | | | 4.4 N | | |

(Hold force without overheating is about half of these.) The three proximal joints are loaded almost equally
(~0.05 N·m per N), the IP only a third of that. So:

- **Stronger servos go on cmc_rot, cmc_flex and mcp_flex** (HLS3606M; the IP can stay SCS0009). That gives
  ~14 N pinch, 3× V1, at the 80 lb braid's safety margin (118 N line force; `README.md` §2).
- **Differential CMC (proposal, not modelled):** in opposition the rot and flex servos are both near their
  limit; in power grasp rot is idle. A 2-input differential (the two CMC servos' strands summed on the two
  drums, like a differential wrist) would let both servos share either joint: +~40 % pad force at no extra
  servo. Needs a pulley differential in the forearm, so I left it out of this concept.
- **A 5th DOF does not pay** here: the 4-DOF chain already reaches every pad within 2 mm / 40°; a 17th servo
  would buy pad orientation (index 40° → ~10°) but costs a servo slot, two strands through the neck (8 in a
  Ø1.6 eyelet is too many) and torque budget. A passive compliant MCP abduction (TPU flexure under the
  proximal hub, ±8°) would give the index pad some of that for free; worth a print test.
- 1X's thumb CMC does 3.5 N·m; ours does 0.29 (SCS) / 0.74 N·m (HLS). The human-level gap stays (see the hand
  research README).

## 7. Failure modes and mitigations

| what fails first | why | mitigation in this design / next step |
|---|---|---|
| strand at the cmc_flex nip | up to ~60° bend over a Ø1 pin under up to 45 N (118 N with HLS) | polished hardened pins; bend radius 0.68 mm is tight for UHMWPE fatigue. Cycle-test; fall back to Ø1.5 pins (error grows to ~0.18 mm) |
| strands rubbing in the neck | 6 strands twist about each other over ±70° | UHMWPE on UHMWPE, low normal force; check wear on a 10 k-cycle rig. Option: a 7-hole PTFE spinneret instead of one hole |
| thin wall between the cmc groove (x 3.5–4.5) and the central slot (x ±2.3) | 1.2 mm of PETG carrying groove pressure | print the metacarpal on its side so the wall is in-plane; or steel washers as groove flanges |
| cmc_flex stubs (Ø3, 4.5 mm in the hub) | bending under a 10–15 N tip load (~70 N on the stubs) | fine for steel; the PETG bore is the weak part: add brass bushings |
| tip-pad shear | TPU on PETG under key pinch | dovetail seat (not modelled), or cast silicone in a recess |
| wrist collision | the thumb can point back past the palm at one range corner | firmware joint-space limit (rot > 90 and flex > 50 and MCP < 0 not together) |
| rot range 141° | the key pinch needs it; 197° of servo travel at 1.4:1 | within the SCS0009's 300°; if the HLS (270°?) is shorter, check |

## 8. Interface for the palm (`out/thumb/interface.json`)

- **Mount:** `thumb_mount`, Ø31 bearing housing along a1 from 17.2 to 37 mm below P, flange Ø42 at 27–30 mm
  below P with **3 × M3** on r 18.5 (points in the JSON; heat-set inserts in the palm). Frame "C": origin P,
  z = a1, x = a2 (all in the JSON).
- **Strand exits (8, all sheathed, world mm, direction = where the sheath leaves):**
  - the 6 bundle sheaths leave the mount's bottom together at ~(22.6…26.6, 1.2…4.9, 22.4…25.5) going
    **−a1 = (−0.410, 0.565, −0.716)**, i.e. down, dorsal and ulnar, toward the wrist. A gentle S (R ≥ 15) in the
    palm brings them into the wrist bundle;
  - the 2 cmc_rot sheaths leave the mount's side block at (37.4, −26.6, 22.6) and (49.5, −19.4, 23.9) going
    (0.42, −0.58, −0.70) (in the drum plane, pointing as close to the wrist as the plane allows).
- **Clearance:** the base sweep hull (bay) and the full envelope hull (STL). The mount's bottom is at
  z ≈ 24.7 (x 25, y 2.5): it sits under the index MCP column, so the index strands must pass beside it
  (x ≤ 12 or on the dorsal side).
- **Joint data** for the integrator and the sim: `build(q)["joints"]` / `interface.json` (`point`, `axis`,
  `range`, `parent`, `child`).

## 9. Requested params changes

1. `THUMB_CMC_REGION["z"]` → **(10, 55)**, and record the CMC centre **P = (39.8, −17.9, 50.4)** with
   `THUMB_AXES` (cmc_rot (0.410, −0.565, 0.716), cmc_flex (0.132, −0.740, −0.659), through P).
2. `THUMB_LENGTHS_HINT` → **(46.7, 36.0, 31.6)** (CMC centre→MCP, MCP→IP, IP→tip).
3. `THUMB_RANGES = {"cmc_rot": (-31, 110), "cmc_flex": (-37, 63), "mcp_flex": (-10, 70), "ip": (-15, 85)}`.
4. `THUMB_MOUNT = {"bolt_circle_r": 18.5, "bolts": "3 x M3", "depth_below_P": 37}` and the 8 exits from
   `interface.json`.
5. Servo choice: HLS3606M on `thumb_cmc_rot`, `thumb_cmc_flex`, `thumb_mcp_flex` (IDs 7, 6, 5).

## 10. Open issues / honest weaknesses

- **Strength** is still ~5 N with SCS0009, ~14 N with HLS: far below the 45–80 N of 1X / humans.
- The finger model is a stand-in (straight phalanges, pads as points). Re-run the opposition score with the
  finger designer's final geometry (`OPP_FINGER`, `finger_fk`) before freezing.
- Contact quality = gap + normal angle only; no friction cone or grasp-stability metric, no thenar contact in
  the pinch tasks.
- The nip-pin wrap model is 2D (axial offsets in quadrature) and assumes the strand rides the pin; the real
  numbers need a bench test with the 0.36 mm braid (stretch over the same angles).
- Pin and bearing seats are modelled nominal (no print compensation); the mount's flange and the sheath
  bundle cavity are placeholders for the palm integrator's bay.
- The rest pose (C-ready, thumb standing out) is not the flat-hand pose; renders and the sim should use
  q_flat for a flat hand.
- Not modelled: tensioners (forearm), tactile sensors, cable for a tip sensor (it could share the neck).

