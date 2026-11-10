# Study 04: how a real 2-DOF wrist fits into the Tendra Hand repo

Date: 2026-11-04. Written for the owner (plain English). Nothing in the repo was edited; this file is the only thing added.

Words used here: **bend** = `wrist_flex`, **wave** = `wrist_dev`, **strand** = one tendon line (32 of them), **sheath** = the PTFE tube a strand slides in (like a bicycle brake cable), **bundle** = all sheaths gathered together, **gap** = the 20 mm space between forearm and palm where the wrist turns.

Baseline check: `uv run pytest -q sim/tests hardware/cad/tests` takes 30 s. Result: 156 passed, 1 failed. The failure is `test_json_is_up_to_date` and it is not caused by anything here. `tendon_router.py` writes `0.5735764363510462` where the saved JSON has `0.573576436351046` (the last digit of three thumb `anchor_dir` numbers differs). It is the same maths on a different CPU/Python. Worth making that test compare with a small tolerance before starting wrist work, or the new JSON checks will fail for the same reason.

---

## 1. Short version

1. The sim already has a complete 2-DOF wrist, but it is an **ideal** one. Every strand is forced through a single point (`wrist_centre`), so bending never changes a strand length. A real wrist cannot do that for 32 strands at once.
2. The real layout is far from that ideal. At the wrist plate the strands are spread over **74 mm (x) by 21 mm (y)**, up to **40 mm from the forearm axis**. A hollow wrist needs them gathered into a bundle about **14 mm wide**. The palm and forearm channels do not do that today.
3. Even a perfect 14 mm bundle changes strand lengths by up to about **10 mm** over the wrist range (my quick calculation, section 3). On a 6 mm drum, 1 mm of strand is about 9.5 degrees of finger. So the "fingers don't move when the wrist moves" claim in the sim will not hold on hardware unless we design for it. This is the biggest risk and the first thing to settle.
4. The SCS0009 servo (0.226 N·m at 6 V) is too weak to drive a wrist directly. The sim's placeholder wrist motors are 2 N·m, about 9 times more. The wrist needs a stronger actuator (or a big mechanical advantage), and that decides where it goes and how it is wired.
5. The RL code needs **no size changes**: `wrist_flex` and `wrist_dev` are already two of the 7 arm joints the IK drives. Only ranges, gains and workspace change, and those need the "legacy value" rule.

---

## 2. What exists today

### Sim (`sim/convert_v1_wrist.py`, `sim/models/tendra_hand_v1_wrist[_left].xml`)

| Item | Value |
|---|---|
| Joints | `forearm_rot` (twist, + = pronation, axis through forearm centre x -33.5 mm, y 25.5 mm), `wrist_flex` (+ = toward palm, about X), `wrist_dev` (+ = toward thumb, about Y) |
| Ranges | twist -90..90 deg, bend -60..70 deg, wave -30..20 deg |
| Gimbal | body `wrist_gimbal`, mass **0.05 kg placeholder**, inertia 1e-5, centre `wrist_centre` at (-33.5, 25.5, -20) mm |
| Gap | `WRIST_GAP` 20 mm; palm and fingers sit 20 mm higher than in the hand model |
| Pins | cosmetic capsules: `wrist_flex_pin` runs x -30..30 mm and `wrist_dev_pin` y -16..16 mm. **Both pass straight through the wrist centre**, exactly where the bundle must go (no collision in the sim because contype is 0) |
| Actuators | position servos, kp 10, kv 0.3, limit **2.0 N·m** (placeholders); `forearm_rot` uses OpenArm J5 (DM4310) values, kp 30, limit 7 N·m |
| Strands | all 32 servo strands get one extra site `wrist_centre` inserted after the last forearm point. Test: length change < 1e-9 m at any angle |
| Flange | site `forearm_flange` at z -153 mm = where OpenArm J5 bolts on |
| Left hand | exact mirror; twist and wave axes flip sign, bend does not |
| Known limit (in code) | above ~55 deg bend with wave, the thumb base hits the forearm, which is 91 x 85 mm, wider than a human forearm |

`software/tendra/arm.py` treats the 3 as part of the 7 arm joints (`WRIST_JOINTS`, `ARM_JOINTS`), damps them (`WRIST_KV_SCALE` 2), and puts a `wrist` site on the palm at `WRIST_IN_MODEL = (-33.5, 26, -30)` mm. The IK (`ArmIK`) uses all 7.

Possible small mismatch to check: the palm moves up 20 mm in the wrist model, so that control point sits at z -10 mm while the real wrist centre is at z -20 mm (and y 26 vs 25.5). So pure wrist rotation moves the control point by about 10 mm x angle. Probably accepted so far, but it matters if the gap or the centre changes.

### Hardware layout the wrist has to live with (`tendon_router.py` -> `tendon_routes.json`)

- Palm x -79..12, y 12.5..40. Forearm box y -17..68, z **-153..-30**. Wrist plate z **-42..-30** (12 mm, part of the forearm). Channels end vertical at z -42.
- Servo levels (spool height): level 1 z -55, level 2 z -90, level 3 z -125. Sheets: F1 y15/L1, F2 y20/L2, F3 y25/L3, B2 y31/L2, B1 y36/L1. Columns (x): little -64, ring -48, middle -32, index -16, thumb 0.
- Used: F1 (mcp_flex x5), F2 (pip x4 + thumb cmc_flex), B2 (mcp_abd x4 + thumb ip), B1 (thumb cmc_rot). **All of level 3 is empty** (F3 unused; B-sheets at level 3 free). CLAUDE.md earmarks it for the ESP32-S3 and FE-URT-1.
- Strand positions at the wrist plate bottom (`wrist_bottom_mm`), 32 of them: x from -69 to +5, y 15, 20, 31, 36; each strand is 10 mm from its partner (flex/ext = the two sides of a 5 mm spool). Distance from the forearm axis: 6.5 mm (middle pip flex) up to 40 mm (thumb mcp_flex ext).
- `min_bend_radius_mm` per strand ranges from 17 mm (thumb ip/cmc_flex) to several hundred mm; the router checks >= 15 mm for the channel (PTFE tube limit is 12 mm). The thumb ones are the tightest (16.96, 17.32, 17.57).
- Palm and forearm are still the old layout: CLAUDE.md says "palm/forearm need a rebuild for the 16-servo layout", and the sim's forearm meshes are the old 20-servo ones.

### Firmware / PC

- `config_v1.h`: 16 servos, firmware 0.4.0, IDs 1..16 in protocol order. `P` takes exactly `kNumJoints` values; `RealHand` refuses a different count. Bus: UART1 1 Mbps via FE-URT-1, torque off at boot.
- `joints.py`: `V1` HandSpec has 16 motor joints + 4 coupled DIPs. Tests check it against `config_v1.h` and the MJCF.
- Power: >= 15 A supply planned for 16 servos (stall about 1 A each at 6 V), three branches with capacitors.

---

## 3. Conflicts found

**C1. The strands are far too spread out for a hollow wrist.** 74 x 21 mm at the wrist plate vs a bundle of about 14 mm (32 PTFE tubes of 2 mm; my hexagonal packing gives a 12 mm ideal, 14 mm realistic). All 32 must converge toward the forearm axis (-33.5, 25.5). Today they go straight down from the plate to their spools. Converging needs sideways room, and the existing S-curves already use most of the palm and forearm height.
Rough numbers (cosine S-curve, minimum radius R = 2h^2/(d x pi^2)): moving a strand 35 mm sideways with R >= 15 mm needs about **51 mm of height** for that extra bend alone. Level 1 spools are only 13 mm below the plate (z -55 vs -42), so there is no room to fan back out on that side. Options in section 5, step 2.

**C2. An ideal point is not a real bundle.** The sim sends all strands through one point. In a 14 mm bundle the strands are up to 6 to 7 mm off the axes. My quick script (32 points, 2 mm pitch, 20 mm gap, bend -60..70, wave -30..20) gives a worst change in end-to-end distance of **about 9.8 mm** (11.8 mm for a 30 mm gap). A stiff sheath cannot follow that: the outer ones would be pulled, the inner ones buckle, and any bare strand in the gap gets a real length change. Because drum radius is 6 to 7 mm, a 1 mm error is about 9 degrees of finger. This must be solved by design, not hoped away. Candidate fixes: (a) per-strand compensation from the wrist angles in firmware (the error is almost linear in offset x angle, so it is computable, but 5 to 10 mm of strand is 1 to 2 rad of servo, a big share of the servo's 300 degrees and of the 133 degrees fingers already use); (b) a short gap and a very small bundle (fewer strands through the wrist, e.g. fewer servos or smaller tubes); (c) a flexible, floating guide (spiral, bellows or sliding sleeves) that lets sheath ends move; (d) measure first on a printed test wrist (step 4 below).

**C3. Gimbal pins block the centre.** The cosmetic flex and dev pins cross the wrist centre. A real gimbal must be a ring around the bundle with pins on its sides, so the ring inner diameter must be at least bundle + clearance, about 16 to 18 mm. That also sets the wrist's minimum size (current 20 mm gap is about right; the bundle is the problem, not the pins).

**C4. Bend radius in the gap.** With a 14 mm bundle bending 70 degrees over a 20 mm gap, the bundle centre radius is 20 / 1.22 = 16 mm, so inner tubes bend at about 9 to 10 mm, below the 12 mm limit used in the router (`TUBE_MIN_BEND_R`). Either the gap grows to about 23 mm or more for the bend range, or the bend range shrinks, or the bundle gets thinner.

**C5. Servo strength.** SCS0009: 0.226 N·m at 6 V, used at <= 80 % (about 0.18 N·m). Wrist need (my estimate): hand about 0.4 kg with centre of mass 70 mm out gives 0.28 N·m just for gravity, plus a 0.5 kg object 100 mm out gives 0.49 N·m; sim placeholder 2 N·m. A tendon wrist driven by a 5 mm spool: force = 0.18 / 0.005 = 36 N; on a wrist drum of radius r_w the torque is 36 x r_w. Servo range limits r_w: bend range 130 degrees x (r_w / 5 mm) must fit in about 280 servo degrees, so r_w <= about 10.7 mm, torque <= about 0.39 N·m. That is not enough margin. So either a stronger servo (the repo notes mention Feetech SCS2332 at about 0.44 N·m and HLS3606M at 0.59 N·m, same size class) or a geared wrist actuator. This is a decision for the owner (section 7).

**C6. Where the wrist motors can go.** Two wrist actuators fit physically only in the free third level (z about -142..-119) or lower, as the two wrist servos or something bigger; the tendons from there to the wrist would run 100 mm up and would have to share the bundle (more strands in C1). A different option: put the wrist motors outside the forearm box (on the elbow side, OpenArm's J5 area) or in the gap itself (small direct-drive). The level 3 space is also where the ESP32-S3 and FE-URT-1 are planned: 2 servos + electronics must share about 91 x 27 x 24 mm. Columns are 16 mm pitch with 12.1 mm servos, so 5 columns are available on each of the 3 sheet positions. It is tight but possible for two servos plus a small board.

**C7. Forearm is wider than a human arm.** 91 x 85 mm. It already limits wrist range (the 55 degree note). A real wrist range will be limited by the same box unless the forearm is slimmed, as the roadmap line says.

**C8. Wiring and twist.** The ESP32, bus and power sit in the forearm, which twists +-90 degrees with J5. The cable from the elbow link to the forearm must have slack or a slip ring; the same cable carries up to 15+ A. Two more servos: ID 17 and 18 on the same bus (daisy chain), adding roughly 2 A at stall (more if bigger servos). 15 A supply is probably still enough on average, but check peak.

**C9. Test and generator coupling.** `test_every_servo_strand_goes_through_the_wrist_centre` and `test_strands_keep_their_length_at_every_wrist_angle` encode the ideal. They will need to be rewritten or marked ideal-only once strands go through a bundle of finite width.

---

## 4. What changes in each layer

### 4.1 `hardware/cad/tendon_router.py` (+ tests, + `tendon_routes.json`)

- Add wrist constants: wrist centre (x -33.5, y 25.5, z), gap, bundle radius, bundle pitch, sheath OD 2.0.
- Add a **bundle layout**: assign each of the 32 strands a slot in the hex bundle at the wrist plane (nearest-fit, avoid crossings; reuse the idea of `_best_assignment`).
- Add a **convergence stage**: S-curves from each strand's current `wrist_bottom` (palm side) to its bundle slot (forearm and palm sides both), with `min_bend_radius >= 15` for channels and >= 12 for tubes. Decide where the extra height comes from (step 2 below). Update `entries()`, `wrist_bottom` fields, new field e.g. `wrist_bundle_mm` and `wrist_top_mm`.
- Fan-out in the forearm: from the bundle to the spool tangent points; the "no strand is blocked by another servo or spool" test (`test_no_strand_is_blocked_by_another_servo_or_spool`) must be rerun with slanted lines, plus a **fleet-angle** limit (how far a line may leave the spool groove plane, say <= 3 degrees).
- Move or add servo slots for the wrist actuators (IDs 17, 18; `SHEETS` and `Z_LEVEL` level 3).
- New tests (suggestions):
  1. every strand at the wrist plane lies within R_bundle of the forearm axis (e.g. <= 7 mm);
  2. tubes in the bundle keep a wall of >= 0 mm (no overlap, pitch >= 2.0 mm) and the bundle fits the gimbal ring bore;
  3. bend radius >= 15 mm (channel) and >= 12 mm (tube) for every convergence curve;
  4. fleet angle at each spool <= limit;
  5. **chord-change test**: for each strand, distance between its two sheath ends over the full wrist range (bend, wave, twist) stays within X mm of nominal, where X is the compensation budget the owner accepts (start with 1.0 mm and report the real number);
  6. a servo-range check: wrist joint range x ratio fits in servo ticks (like the existing "every joint fits 0..1023" host test).
- Make `test_json_is_up_to_date` tolerant (rounding) first.

### 4.2 Fusion stages (`hardware/cad/fusion_scripts/TendraHandV1/TendraHandV1.py`)

Existing stages: fingers, thumb, names, thumb_base, thumb_meta, palm, forearm, thumb_inner, knuckle, linkage; extras: export, thumb_unhinge. Add (all idempotent like the others, each "skips itself if applied"):

- `palm` rebuild: new channel ends (bundle exit), spaced for the gimbal.
- `forearm` rebuild for the 16-servo layout + wrist servo pockets (this is already on the to-do list in CLAUDE.md) + bundle guide.
- new stage `wrist`: gimbal ring (hollow, ID >= bundle + clearance), two pin axes, hard stops for the ranges, tendon drums or horns for the wrist actuators, mounting to the forearm and the palm.
- `export` must write the new parts (and the wrist joints and ranges into `hand_v1.json`) so `convert_v1.py` can read them.
- Keep the order: mirror (`mirror_export.py`) -> `convert_v1.py` -> `convert_v1_wrist.py`.

### 4.3 `sim/convert_v1_wrist.py` (+ `sim/tests/test_v1_wrist.py`, `test_v1_left.py`)

- Replace the cosmetic pins by real gimbal geometry or at least keep them off the centre.
- Read wrist dimensions from the router JSON (single source of truth) instead of constants; real gimbal mass, inertia, ranges, limits.
- Strand path: instead of one site `wrist_centre`, insert one site per strand (at its bundle slot, fixed to the forearm) and one at its palm-side exit, so the model sees the real length change, or keep the ideal sheath and add a **compensation model** if firmware compensates. A bundle slot site per strand plus tendon friction (`frictionloss` or a simple capstan factor) is the minimum.
- Real wrist actuators: kp, kv, torque limit from the chosen servo; `WRIST_KV_SCALE` in `arm.py` has to be re-tuned (the 2026-10-06 damping work assumed these values).
- Offsets: `finish()` already re-measures strands at q = 0; it keeps working once strands have more points.
- Tests: swap "every strand through one site" for "every strand through its slot", and "length keeps < 1e-9" for "length change <= budget over the range"; keep `test_no_contacts_over_the_wrist_range` and recheck the 55 degree limit; add a test for the left hand.
- Regenerate in order: `mirror_export.py`, `convert_v1.py`, `convert_v1_wrist.py`. `uv run pytest` fails if stale.

### 4.4 `firmware/include/config_v1.h` and firmware

- Decide first: are the wrist servos part of the hand firmware or the arm controller? (Section 7, decision 2.)
- If in the hand firmware: `kNumJoints` 16 -> 18; add two `ServoJointConfig` rows for IDs 17 and 18 (`wrist_flex`, `wrist_dev`) with ranges and `servo_per_joint` (wrist drum / spool); bump the version (0.5.0). The `P` command, `J`, `V`, `Z`, `M` loops all use `kNumJoints`, so most code follows automatically. **This changes the serial protocol (P needs 18 values), so per CLAUDE.md the owner must approve it first.**
- `FakeEsp32` and `RealHand` refuse the old count: update both, plus `firmware/test/host/test_scs.cpp`, and the "every joint fits 0..1023 ticks" host test.
- If compensation is in firmware (C2 fix a): add a small coupling table (strand offsets x wrist angle) and run it every goal cycle; needs the wrist angle from servo feedback; add a host test.
- Torque off at boot stays; add a rule that wrist goals are ramped from the measured position at the first command (no jump).

### 4.5 `software/tendra/joints.py`

- Either a new `HandSpec` variant (`v1w`: 18 motor joints) or add `wrist_flex`, `wrist_dev` to `V1` (changes everything that counts 16). A new variant is safer: `get_hand`, `--hand`, the auto-detect from the `I` line, and `servo_ids` are already variant-based.
- Add limits and `servo_per_joint`; tests that compare with `config_v1.h` and the MJCF must be updated.
- Keep `forearm_rot` out of it: it belongs to OpenArm's J5 (a different motor and controller).

### 4.6 RL and scene (`arm.py`, `scene.py`, `grasp_env.py`, `gpu/`)

- Observation and action sizes: **unchanged** while the wrist joints keep their names, because `wrist_flex` and `wrist_dev` are already among the 7 arm joints.
- Things that do change: joint ranges (`arm_limits` penalty, IK limits), `WRIST_IN_MODEL` / wrist site if the centre moves, wrist kp, kv, torque, gimbal mass (gravity compensation of the arm), maybe a wrist speed limit.
- Per the legacy-key rule: every new env key needs a legacy value (`LEGACY_CONFIG`, `LEGACY_REWARDS` in `grasp_env.py`; `ACTION_KEYS`, `LEGACY_ENV` in `gpu/policy.py`). If the wrist model has an extra config such as friction scale or wrist range scale, give the old value.
- Saved policies were trained with the ideal wrist: re-evaluate them on the new model (`eval_grasp.py`) before deciding whether to retrain. The GPU bundle must be regenerated (`make_train_notebook.py --run <new name>`).
- A friction/compensation model adds new behaviour the policy has not seen; domain randomisation of wrist friction is a candidate.

### 4.7 Docs and website

- `docs/roadmap.md` Phase 8: the line "Hardware: wrist gimbal with a hollow centre..., wrist actuators, slimmer forearm" becomes a few checkboxes (those in section 6). Phase 2 stays as is.
- `CLAUDE.md`: Hardware section (wrist plate, forearm, servo list, IDs), wrist model paragraph, firmware version. Keep in sync in the same step.
- `research/log.md` entry per step; a new experiment folder, e.g. `research/experiments/2026-11-xx-wrist/`, like the thumb routing one.
- Website: `KEY_FACTS` in `src/components/seo/llms.ts` and `/docs/specs` only if specs change (servo count 16 -> 18, DOF), and the journey page only when a step in `now.steps` changes. The site text is future-focused, so leave it alone until the wrist is built.
- `hardware/cad/README.md` and `firmware/README.md`: new IDs and stage names.

---

## 5. Proposed staged plan (each step small and checkable)

0. **Housekeeping (1 hour).** Make `test_json_is_up_to_date` tolerant. Check: all 157 tests pass.
1. **Quantify the wrist (no CAD).** Write `research/experiments/2026-11-xx-wrist/` with a script that computes, for a hex bundle, gap g, and ranges: chord change per strand, tube bend radius, bundle diameter (my quick numbers: 14 mm bundle, up to 9.8 mm change at g = 20). Plot change vs gap and bundle size. Check: reproduces the numbers here, and gives the gap and bundle that meets a stated error budget. This tells the owner whether option (a), (b) or (c) of C2 is needed.
2. **Pick the convergence strategy** (decision 3, below), then add the bundle layout + convergence S-curves to `tendon_router.py` with the tests in 4.1. Check: all new tests pass, JSON regenerated, bend radii >= 15.
3. **Pick the wrist actuator** (decision 1) and add the two wrist servo slots (level 3) to the router, with servo-range and force-budget tests. Check: slots do not collide with other servos or the ESP32 and FE-URT-1 space.
4. **Print a test wrist** (not the full hand): a short forearm top, a gimbal ring and a palm bottom with 32 sheaths. Measure real strand length change at several angles with a caliper or by watching a marked line. Check: matches step 1 within 1 mm; if not, update the model.
5. **Sim update:** `convert_v1_wrist.py` reads the router JSON, uses per-strand slots, real wrist actuators and mass; update the two test files. Check: all sim tests pass; the sim shows the same finger error as step 4 measured.
6. **Fusion stages:** rebuild `palm` and `forearm` for 16 servos + wrist, add `wrist`, export. Check: Fusion interference check is clean; STL export matches the router JSON.
7. **Firmware and PC (after owner approval of the protocol change):** `config_v1.h` (18 servos, version bump), `joints.py` variant, `FakeEsp32`, `RealHand`, host tests. Check: `pio run -e hand_v1_servo` builds, `pytest` passes, the `B` bus scan lists 18 IDs on the bench.
8. **Closed loop:** if firmware compensation is used, calibrate it on the printed wrist. Check: finger angle stays within 3 degrees over the wrist range.
9. **RL/scene re-check:** run `eval_grasp.py` for the existing best policy on the new wrist; tune `WRIST_KV_SCALE` and ranges; only retrain if the success rate drops.
10. **Docs/CLAUDE.md/roadmap/log** after each step, not at the end.

Steps 1 to 3 are pure computation and can happen in the sandbox with no hardware.

---

## 6. Open questions the plan cannot answer without measurements

- How much sheath slack a real 14 mm bundle needs, and whether the PTFE tubes rub or kink at 70 degrees.
- How much friction the bent sheaths add (it grows with the wrist angle, like a rope on a pulley, and it differs between strands). This changes the finger servo load and the 80 % self-protection of the SCS0009.
- Whether hollow-gimbal strength is enough in 3D-printed PETG for 0.5+ N·m.

---

## 7. Decisions for the owner

1. **Wrist actuator:** keep SCS0009 (too weak by my estimate), move to a stronger servo (SCS2332 / HLS3606M-class), or a separate geared motor? And where does it sit (level 3 of the forearm, in the gap, or on the arm side)?
2. **Whose controller:** are the 2 wrist motors on the hand's servo bus and firmware (IDs 17 and 18, protocol changes to 18 values), or do they belong to the arm (like J5), keeping the hand firmware at 16? This is a protocol change, so I will not touch it until you say yes.
3. **Convergence strategy:** (A) make the forearm and palm channels longer so the 32 strands gather into a 14 mm bundle (more height, likely a longer forearm, which collides with the OpenArm geometry: our forearm is 123 mm and replaces OpenArm's 120.5 mm), (B) accept fewer strands through the wrist (e.g. thinner tubes or move some servos into the palm/hand), or (C) accept an error budget and compensate in firmware.
4. **Acceptable finger error while the wrist moves** (for example 3 degrees = about 0.3 mm of strand on a 6 mm drum).
5. **Wrist ranges** for the real design (the sim has bend -60..70, wave -30..20). Smaller ranges make the bundle problem much easier.
6. **Forearm width:** slim it now (the known 55 degree collision) or later with the own arm design?
7. **Hollow gimbal vs ring-and-pins** for the first print, and which material.
8. **Order of work:** wrist now, or finish building V1 (servo bus wiring, printing the palm/forearm) first? The wrist needs the palm/forearm rebuild anyway, so I suggest doing the rebuild once, with the wrist in it.
9. **Protocol/firmware version** (0.5.0) and approval to bump `HandSpec`.
10. **Left hand:** mirrored automatically, but the compensation table and the wrist actuator placement must be mirrored too; confirm that is wanted.
