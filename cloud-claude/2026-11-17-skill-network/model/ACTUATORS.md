# Actuator parameters (V2.1, joint level): 16 x Feetech HLS3606M + 2 x STS3215 wrist

Owner decision 2026-11-17: **all 16 hand servos are the HLS3606M** (SCS0009 dropped). `actuators.json` (`servo_model: "HLS3606M"`) holds one entry per servo joint (16)
and wrist joint (2), all **at the joint, + = closing**, for a MuJoCo `position` actuator (`kp`, `forcerange`), joint `damping`, `armature`, `frictionloss`.
Validate and step-test: `uv run python model/check_actuators.py`. Status: datasheet + V2.1 study, **no bench measurement yet**; estimates are marked.

## 1. Servo data (datasheet)

| | HLS3606M (16 hand joints) | STS3215 12 V (wrist) |
|---|---|---|
| Voltage | 4.5-7.4 V (design 6 V) | 12 V version (7.4 V version also exists) |
| Stall torque | 6 kg·cm @ 6 V = **0.588 N·m** (4.8 kg·cm @ 4.8 V); study: 0.59 N·m, rated 0.15 N·m | 30 kg·cm = **2.94 N·m** (7.4 V: 19.5 kg·cm) |
| No-load speed | 0.09 s/60° @ 6 V = 11.6 rad/s (111 rpm); 0.11 s/60° @ 4.8 V | 0.222 s/60° = 4.72 rad/s |
| Stall / no-load current | **1.3 A** / 150 mA @ 6 V | 2.7 A / 180 mA |
| Gearbox | **1:205**, metal gears, aluminium case, coreless motor, 20.6 g (C001) | 1:345 |
| Encoder | 12-bit magnetic (4096 counts, 0.088°), 360° | 12-bit |
| Modes | angle servo, constant speed, **constant current ("constant force", torque target register)** | position |

Sources: <https://openelab.io/products/feetech-hl3606-ttl-servo-motor> (specs above), <https://evelta.com/hl-3606-c001-6v-6kg-360deg-metal-gear-single-shaft-ttl-servo-motor>,
<https://www.switch-science.com/products/9823> (12-bit, 0.088°, 25T/4.95 mm horn), <https://rcdrone.top/products/feetech-hl-3606-servo-motor>; STS3215:
<https://www.dfrobot.com/product-2962.html>, <https://www.waveshare.com/product/st3215-servo.htm?sku=22414>, <https://servodatabase.com/servo/feetech/sts3215>.
Project notes: `research/log.md` 2026-09-30/10-01 (0.59 N·m, 1:205, 25T).
**Hedges:** (a) the torque is quoted only at 6 V ("6 kg" in some shops may be a constant-force rating, not stall); (b) naming is mixed (HL-3606, HLS3606M, one SKU HLS3612);
(c) multi-turn mode, current-sensing resolution (STS-family registers are ~6.5 mA/unit) and angle range are **not on any shop page I could read**: get the Feetech memory-table manual
before writing the driver. Review item 11b (multi-turn) is still open; it is not needed here because the spool/drum ratio is unchanged (servo travel 280° fits one turn).
Retailers mix the 6 / 7.4 / 12 V STS3215 figures; the V2 study's 12 V part is used.

## 2. Transmission and efficiency (HLS does **not** change the spool or drum)

- Spool line radius 5 mm, drum line radius 7.235 mm (7 mm + 14x0.5 brass ring) on every servo joint: **G = 1.447** (reports 12, 16, 18; `travel_table.json`; the thumb uses the same).
  The study checks HLS at "r 5" (spool); report 01 notes a smaller spool is impossible on the 4.95 mm spline. The 25T spline needs new hubs, not a new ratio.
  Caveat `thumb_cmc_rot`: CAD flex/ext slopes 0.216 / 0.120 mm/° (loop not closed), 1.447 used.
- Efficiency eta (unchanged by the servo, capstan loss does not depend on tension): finger route 0.92 x 0.97 liners = 0.89; joint friction MCP/abd 0.998, PIP 0.949 (incl. the DIP bar) =>
  **mcp 0.888, pip 0.845**; thumb (CAD eta_strand x 0.92) **0.79 / 0.60 / 0.72 / 0.61** (cmc_rot / cmc_flex / mcp / ip). Reports 09, 12 §1, 13, `out/18/thumb/checks.json`.
- **Joint torque limit** = 0.588 x 1.447 x eta: mcp/abd **0.756**, pip **0.719**, thumb **0.672 / 0.507 / 0.612 / 0.516 N·m** (peak stall). `forcerange_cap80` (80 %, the firmware/constant-current cap recommended in §8) is also in the JSON.
  `continuous_torque_Nm` (rated 0.15 x G x eta ~ 0.19) for sustained holds. HLS back-off: same 80 % / 2 s behaviour assumed (STS-family rule; verify).
- **Pinch cross-check (index, PIP-limited):** (0.719 - 0.036 pretension) / 0.0462 m = **14.8 N** at stall, **11.7 N** at the 80 % cap. Reports: 11.6 N (review 10, dorsal crossing), 15.7 N (hand), 16.6 N (bench), 17.6 N (CAD fix),
  80 %: 12.4-13.1 N. Inside the range; the lower value is the extra 0.97 liner loss. Thumb cmc_flex: 0.507 N·m vs the report's 10.6 N index-pad pinch (lever ~48 mm, plausible).
- **Speed limit** = 0.6 x 11.6 / 1.447 = **4.8 rad/s** (loaded, 6 V; no-load joint speed 8.0 rad/s). At 5 V scale by ~0.8.

## 3. Stiffness, damping, armature, friction (fingers, thumb)

- **Tendon stiffness (series):** K_t = r² EA / L = 7.235 mm² x 7 kN / (0.28-0.40 m) = **0.9-1.3 N·m/rad** (PE#8, EA 5.5-11 kN in report 02). At HLS stall this gives a lag of 0.7 N·m / 1.2 = 0.6 rad = **35°**;
  report 02 quotes 31° at HLS with a steel coil and 208° with PTFE, so the line (not the servo) is the compliance. Joint-angle sensing is mandatory (report 01 §3.3).
- **Servo P stiffness at the joint:** full stall torque at 10° shaft error (**estimate**) => k_s = 0.588/0.1745 = 3.37 N·m/rad; at the joint eta G² k_s = **6.0-6.3 N·m/rad**. kp (MuJoCo) = series(k_joint, K_t) = **0.75-1.08 N·m/rad**
  (index mcp 1.02, pip 0.96, thumb 0.75-1.0). The tendon dominates, so the HLS is only ~25 % stiffer than the SCS case despite 2.6x torque.
- **Armature** (reflected rotor): coreless rotor ~3e-9 kg·m² (**estimate**, 1-6e-9) x 205² x G² = **2.6e-4 kg·m²** at the joint (SCS case was 7.2e-4 with 1:416: gearbox ratio² is 4.1x lower).
  Still larger than the finger links (6e-6 to 1e-4).
- **Link inertia** (`load_inertia_kgm2`): unchanged from the CAD-mass estimate (index yoke 3.85 g, proximal 10.98, middle 4.64, distal 4.19 + skin; thumb 6.9 / 17.6 / 11.1 / 6.3 g at assumed COM distances):
  index MCP/abd 6.3e-5, PIP 9e-6, middle x1.3, ring x1.15, little x0.55; thumb 1.0e-4 / 8.6e-5 / 1.6e-5 / 1.5e-6.
- **Damping** = 2 zeta sqrt(kp (I + armature)) with **zeta = 1.8** (0.051-0.070 N·m·s/rad). The HLS is fast (11.6 rad/s no load) and a critically damped sim joint would take ~50 ms for a 30° step, faster than the real,
  speed-limited servo (30° at 4.8 rad/s = 109 ms minimum); zeta 1.8 reproduces ~150 ms. The terminal speed at full torque is still ~11 rad/s, so the **env must rate-limit commands** to `velocity_limit_rad_s`.
- **Frictionloss** = 3 % of the torque limit (0.015-0.023 N·m): 1:205 metal gearbox, not backdrivable + bush/guide friction (**estimate**; randomised x0.4-3).

## 4. Wrist (2 x STS3215 12 V through the V2 gear + crank + rod drive), unchanged

Joint capacity at stall flex min 3.63 / median 5.08 N·m, dev 16.1 / 17.9 N·m (V2 `wrist_summary.json`), effective torque ratio flex 1.73, dev 5.5 (median), eta 0.85 => forcerange **4.32 / 15.2 N·m**;
speed 0.8 x 4.72 / ratio = 2.2 / 0.69 rad/s; kp 21 / 217 N·m/rad (0.5 backlash factor); armature 5e-9 x 345² x ratio² = 1.8e-3 / 1.8e-2; load inertia 1.8e-3 kg·m²; zeta 1.3. With HLS fingers the fist
loads the wrist more: drive torque 1.01 N·m (report 13, HLS fist at 80 %) vs 3.63 N·m flex capacity: OK. Open: one vs both servos in the capacity figure; effective angular ratio (flex up to 2.9).

## 5. Step response (`check_actuators.py`, 30° fingers / 20° wrist, 2 ms step)

| group | rise 10-90 % | overshoot | 2 % settling | steady error |
|---|---|---|---|---|
| finger/thumb joints | 144-166 ms | 0 % | 0.53-0.64 s | -0.08..-0.27 % |
| wrist_flex | 72 ms | 0 % | 0.17 s | 0 |
| wrist_dev | 104 ms | 0 % | 0.16 s | 0 |

## 6. Randomisation (`randomization` in the JSON)

Torque x0.6-1.05 (cap at ~80 %, 5 V = x0.8, eta spread, supply sag), kp x0.5-1.6, damping x0.5-2, armature x0.4-2.5, frictionloss x0.4-3, thumb eta extra x0.8-1.1, velocity x0.6-1.0, link inertia x0.7-1.4;
plus deadband 0-1.5° (12-bit encoder, 0.088°/tick; gearbox backlash dominates), latency 10-50 ms (1 Mbps bus, 16 servos), backlash 0-2°. Sample once per episode.

## 7. Power (16 x HLS3606M)

- Stall current **1.3 A @ 6 V** each (shop figure; hedge) => **20.8 A** if all 16 stall at once; no-load 0.15 A each (2.4 A). The reports size for >= 15 A (V1) at 5-6 V; with HLS:
  - a **6 V supply of >= 25 A** peak (e.g. 6 V / 30 A) for the hand, or 7.4 V is the servo maximum (4.5-7.4 V) so keep it at 6 V;
  - realistic grasp load: fist = 16 flex strands at <= 80 % torque ~ 16 x 1.0 A = 16 A; power grasp (12 strands at 50 %) ~ 8-10 A; pinch ~ 2-3 A; sustained holds are limited to the rated 0.15 N·m (~25 % torque, ~0.4 A each);
  - **4 branches** (5 servos on the 6 V bus + 4 per branch, thick 16 AWG star wiring), 1000-2200 µF low-ESR per branch and a TVS, because the bus drops 0.3 V per 5 A over ~0.1 ohm wiring and torque falls ~20 % at 5 V;
  - constant-current mode (firmware cap at 80 %) also caps the supply peak at ~16 A;
  - **wrist: 2 x STS3215 at 12 V, 2.7 A stall each** (5.4 A) on a separate 12 V / >= 8 A branch (ground shared).
- Heat: HLS holding > 80 % for > 2 s backs off; a 1.3 A stall in a closed forearm needs airflow; the forearm already holds 599 g of servos.

## 8. Part ratings vs the HLS stall pull (flags)

Strand tension at stall = 0.588 N·m / 5 mm = **117.6 N on every one of the 16 joints** (80 % cap 94 N); the V2.1 design class is ~118-120 N, so nothing is above it, but these parts are marginal (reports 02, 10, 12, 13):

| Part | Rating / margin at HLS | Action |
|---|---|---|
| **Fuse cup** (report 12 §5) | HLS cup (0.8 mm lip on its side) trips at **148 N** = 1.26 x stall, 1.57 x the 80 % cap; at -25 % scatter 111 N < stall. **The SCS cup (67 N) would trip at 0.57 x stall** | print the HLS cups only; cap at 80 % in firmware (constant-current mode) |
| MR63 redirect roller / idler | s0 **0.48 / 0.59** at 80 % (155 / 126 N) | F683ZZ/MR83 or a hardened dowel (eta 0.79 => pinch -21 %) |
| MR84 MCP lug bearing | s0 0.93 at stall (150 N at 80 %) | torque cap 80 % is what keeps it below 1 |
| Spool dog pins | 64 N per pin = 27 MPa on PETG (report 02) | brass liners in the slots or 3 dogs |
| Wrist-eye lip | 418 MPa at HLS (160 at SCS), report 09/10 | the architecture decider; bench test the eye plate first |
| abd x-nip pins | line on Ø2 / Ø1.5 pin 255 / 340 MPa (report 12 #8) | only when abducted; bench |
| Wrist compression | 1670 N (fist at 80 %): ring 7075 SF 1.2, bronze bush s0 1.1 (iglidur 2.2), hollow pins SF 1.6 | iglidur G bushes + 7075 (report 13) |
| PE#8 line | 445 N break: SF 3.8 at stall, ~2.9 if the real rating is 350 N | measure |
| Drum PETG under the ring | 11.3 MPa vs 12 MPa creep limit | marginal; metal ring is in the design |
| MCP axle | 98 MPa (nested tube), fine | - |

## 9. Biggest uncertainties, measure first

1. Servo P-stiffness / saturation angle (10° assumed) and the **constant-current register** (units, limit) on one HLS3606M.
2. HLS stall torque vs voltage (6 V only quoted) and stall current (1.3 A shops); the 80 %/2 s back-off and whether it applies.
3. Rotor/gear inertia (armature 2.6e-4) and speed under load (4.8 rad/s).
4. Real efficiency per joint (thumb 0.6) and pinch force (pass >= 11 N at 80 %); tendon EA and creep (35° stretch lag at stall).
5. Fuse cup trip distribution, MR63/eye-plate wear (§8).
6. Wrist drive: one vs both servos, effective ratio, backlash.
