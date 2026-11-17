# Actuator parameters (V2.1, joint level)

`actuators.json` holds one entry per servo joint (16) and per wrist joint (2), all **at the joint, + = closing**, for a MuJoCo
`position` actuator (`kp`, `forcerange`), joint `damping`, `armature`, `frictionloss`. Validate and step-test: `uv run python model/check_actuators.py`.
Status: derived from datasheets plus the V2.1 study, **no bench measurement yet**. Every estimate is marked.

## 1. Servo data (datasheet)

| | SCS0009 (fingers, thumb) | STS3215 12 V (wrist) |
|---|---|---|
| Stall torque | 2.3 kg·cm @ 6 V = **0.226 N·m** (rated 0.7 kg·cm = 0.074 N·m) | 30 kg·cm = **2.94 N·m** (7.4 V version: 19.5 kg·cm = 1.91 N·m) |
| No-load speed | 0.1 s/60° @ 6 V = 10.5 rad/s | 0.222 s/60° = 4.72 rad/s (7.4 V: 0.192 s/60°) |
| Gearbox / encoder | 1:416 metal, 10 bit (0.29°/tick) | 1:345, 12 bit |
| Stall current | 1.0 A | 2.7 A |

Sources: <https://partsbaba.com/products/scs0009-6v-2-3kg-300deg-serial-bus-servo-motor>, <https://openelab.io/products/feetech-scs0009-ultra-compact-servo>,
<https://www.dfrobot.com/product-2962.html> (12 V), <https://www.waveshare.com/product/st3215-servo.htm?sku=22414> (7.4 V),
<https://servodatabase.com/servo/feetech/sts3215>, <https://core-electronics.com.au/attachments/uploads/sts3215-smart-servo-datasheet-translated.pdf>.
Disagreement: retailers mix the 6 V / 7.4 V / 12 V STS3215 figures; the V2 study uses the 12 V part (2.94 N·m). The project's own V1 supply is 5-6 V:
at 5 V expect about 0.8x the 6 V torque (covered by the torque randomisation). The study's 0.236 / 0.226 N·m and 236 deg/s joint speed agree with the above.

## 2. Transmission and efficiency (fingers and thumb)

- Spool line radius 5 mm (the review fix "every spool r 5"), drum line radius 7 mm + 0.235 = 7.235 mm (brass ring), so **ratio G = 1.447 on all 16 servo joints**
  (reports 12, 16; `travel_table.json`). Thumb drums use the same 1.447 (`cad/18_thumb.py`). Caveat: for `thumb_cmc_rot` the CAD slopes are
  0.216 / 0.120 mm/deg (flex/ext strand, loop not closed, `out/18/thumb/checks.json: loops`), i.e. flex arm up to 12 mm. 1.447 used (the conservative ext value is 1.38).
- Efficiency eta (fraction of servo torque that reaches the joint):
  - route servo to knuckle: 0.92 median with bare strands over metal guides and wrist eyes (report 09/13: median 0.90-0.92), x 0.97 for the PTFE liner bends in the palm (13 §5) = **0.89**;
  - joint friction (report 12 §1, greased brass bushes mu 0.15): PIP incl. the DIP bar 0.949, MCP 0.998, abd 0.998;
  - so mcp_flex / mcp_abd **0.888**, pip **0.845**;
  - thumb: per-strand eta from the CAD (`out/18/thumb/checks.json: eta_strand`, flex strands 0.859 / 0.648 / 0.781 / 0.659 for cmc_rot / cmc_flex / mcp / ip, 8 sliding turns of 60-170 deg) x 0.92 forearm side = **0.79 / 0.60 / 0.72 / 0.61**.
- **Joint torque limit** = 0.226 x 1.447 x eta: finger mcp 0.291, pip 0.276 N·m; thumb 0.258 / 0.195 / 0.235 / 0.198 N·m. This is the stall peak. The servo backs off after 2 s above 80 %, so
  `continuous_torque_Nm` (rated 0.074 x G x eta ~ 0.09 N·m) is also in the JSON for later use as a sustained limit. Loop pretension (5 N x 7.2 mm = 0.036 N·m) is not subtracted: it cancels
  in a balanced loop and acts equally on both directions.
- **Cross-check:** index pinch at stall = (0.276 - 0.036) / 0.0462 m (PIP drive lever, report 12) = **5.2 N**; reports: 5.6 N (hand, report 12), 5.9 N (bench), 5.4-6.8 N per finger (report 18-fingers). Within 10 %, the lower eta
  (liners, hand case) explains it. Thumb: cmc_flex limits at 0.195 N·m; report 18 thumb gives 4.1-4.4 N pinch. OK.
- **Speed limit** = 0.6 x no-load / G = 0.6 x 10.47 / 1.447 = **4.3 rad/s** (loaded; report 01 uses 236 deg/s = 4.1 rad/s). No-load joint speed would be 7.2 rad/s.

## 3. Stiffness, damping, armature, friction (fingers, thumb)

- **Tendon stiffness (series):** K_t = r^2 EA / L, r = 7.235 mm, EA = 7 kN (PE#8 braid, report 02 gives 5.5-11 kN; 7 kN allows for crimps/knots), L = strand length servo to joint 0.28-0.40 m
  (abd 0.28, mcp 0.30, pip 0.32, thumb 0.30-0.40). Gives **K_t ~ 0.9-1.3 N·m/rad** (report 01: 80 lb braid 3.0 N·m/rad for a short 230 mm single strand; report 02: line stretch ~13 deg at SCS stall = ~1.4 N·m/rad;
  18 N/mm loop = 0.94). Reported per joint as `tendon_stiffness_Nm_rad`.
- **Servo P stiffness at the joint:** assume the servo asks for full stall torque at 10 deg shaft error (typical hobby bus servo, **estimate**): k_s = 0.226/0.1745 = 1.30 N·m/rad at the shaft;
  at the joint k = eta G^2 k_s = 2.4 N·m/rad. kp (MuJoCo) = series(k, K_t) = **0.59-0.85 N·m/rad** (index mcp 0.81, pip 0.76; thumb lower from eta and longer strands).
  Torque saturates at about 20 deg error, about like the reports' "sag" numbers. MuJoCo has no separate tendon spring, so kp is the series value.
- **Armature** (reflected rotor): coreless rotor ~2e-9 kg·m^2 (**estimate**, 0.8-5e-9) x 416^2 x G^2 = **7.2e-4 kg·m^2** at the joint. It dominates the finger links (6e-6 .. 1e-4), which is why a 30 deg step takes ~0.12 s.
- **Link inertia** (`load_inertia_kgm2`): point masses (+ rod terms) from the CAD masses (`out/18/checks.json`: index yoke 3.85 g, proximal 10.98, middle 4.64, distal 4.19 + skin 0.66, little 8.6 / 3.8 / 3.7; thumb base 6.9,
  metacarpal 17.6, proximal 11.1, distal 6.3, +1-1.5 g pins/rings) at assumed phalanx COM distances (index 22 / 58 / 83 mm from the MCP, PIP 13 / 38 mm; thumb 20-80 mm). Index MCP/abd 6.3e-5, PIP 9e-6;
  middle x1.3, ring x1.15, little x0.55 (lengths scale estimated). abd uses the MCP value (finger straight). Thumb: cmc_rot 1.0e-4, cmc_flex 8.6e-5, mcp 1.6e-5, ip 1.5e-6.
- **Damping** = 2 zeta sqrt(kp (I_load + armature)), zeta = 1: **0.041-0.052 N·m·s/rad**. It also gives a terminal speed at full torque = 0.29/0.05 = 5.8 rad/s, close to the real servo speed, so no extra rate limit is needed in MuJoCo.
- **Frictionloss** = 3 % of the torque limit (0.006-0.009 N·m): gearbox 1:416 (not backdrivable) plus brass bush/guide friction (1 - eta is load-dependent and already in the torque limit). Gives 0.2 deg steady-state error. **Estimate**; randomised x0.4-3.

## 4. Wrist (2 x STS3215 12 V through the V2 gear + crank + rod drive)

- V2 report 03 / `out/wrist/wrist_summary.json`: 14T:36T spur (2.57:1) + crank + M4 rod, DE-optimised. Joint torque at servo stall: **flex min 3.63, median 5.08 N·m; dev min 16.1, median 17.9 N·m**; continuous (30 % rated) flex 1.11 min (1.56 median), dev 4.9.
  The reports' effective torque ratio (capacity / 2.94): flex 1.23-1.73 (median **1.73**), dev 5.5-6.1 (**5.5**); servo travel -144..+150 deg.
  The ratio varies with pose (crank angle, transmission angle >= 30 deg); the JSON takes the median. **Open: whether the flex figure uses one or both servos** (the report does not say, the rods give 187 N per N·m).
- eta **0.85** (spur 0.95 x 2 ball joints 0.95 x plain bushes; the W13 bushes lose 0.28 N·m at a full SCS squeeze, report 13). forcerange flex +-4.32, dev +-15.2 N·m (dev is oversized by design: the palm balls sit far out).
  Real sustained loads are far lower (hand alone 0.2 N·m, fist at SCS 0.39 N·m, report 13), so `continuous_torque_Nm` (1.56 / 4.9 x eta) is provided.
- Speed: 0.8 x 4.72 / ratio = **2.2 rad/s flex, 0.69 rad/s dev** (dev ratio large: slow). **Uncertain**, the effective angular ratio flex could be up to 2.9 (travel 294 deg over 100 deg).
- kp = 0.5 x eta ratio^2 (2.94/0.1745) with factor 0.5 for gear backlash and rod compliance = **21 (flex), 217 (dev) N·m/rad**. Load inertia of palm + fingers + thumb ~ 0.28 kg at ~75 mm: **1.8e-3 kg·m^2**.
  Armature = 5e-9 x 345^2 x ratio^2 = **1.8e-3 (flex), 1.8e-2 (dev)**. Damping zeta 1.3 (0.72, 5.4). Frictionloss 3 % of the limit.
- Wrist joints take **large** action limits from `velocity_limit_rad_s`; in MuJoCo the position actuator can slew faster (dev 20 deg in 0.1 s vs 0.5 s real), so the env must rate-limit commands.

## 5. Step response (30 deg fingers / 20 deg wrist, 2 ms step, `check_actuators.py`)

| group | rise 10-90 % | overshoot | 2 % settling | steady error |
|---|---|---|---|---|
| finger/thumb joints | 114-144 ms | 0 % | 0.29-0.36 s | -0.5..-0.8 % (friction) |
| wrist_flex | 72 ms | 0 % | 0.17 s | 0 |
| wrist_dev | 104 ms | 0 % | 0.16 s | 0 |

Critically damped by design; real servos have a small overshoot (PD, backlash).

## 6. Randomisation (see `randomization` in the JSON)

Torque x0.7-1.1 (5 V supply, back-off, eta spread), kp x0.5-1.6 (log), damping x0.5-2, armature x0.4-2.5, frictionloss x0.4-3, thumb eta extra x0.8-1.1, velocity limit x0.6-1.0,
link inertia x0.7-1.4, plus non-MuJoCo effects to inject in the env: deadband 0-1.5 deg, latency 10-50 ms (1 Mbps bus, 16 servos), backlash 0-2 deg. Sample once per episode.

## 7. Biggest uncertainties, measure first

1. **Servo P-stiffness / saturation angle** (10 deg assumed): step test one SCS0009 on a free inertia through a loaded tendon, read position feedback. Sets kp (x2 either way).
2. **Rotor/gear inertia** (armature 7e-4): measure the step rise time with no tendon load; drives the whole timing.
3. **Real efficiency per joint** (esp. thumb cmc_flex/ip 0.6, wrist eyes) and the pinch force: first bench finger test (pass >= 4.5 N at 80 %).
4. **Tendon EA / creep** (K_t, and loop pretension).
5. **Wrist drive**: torque per servo (one or two), true effective ratio vs pose, backlash, STS3215 speed under load; `thumb_cmc_rot` drum arm.
6. SCS0009 supply (5 V vs 6 V) and back-off behaviour (2 s above 80 %).
