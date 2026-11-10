# V1 wrist study: two joints (bend + wave) with all tendons passing through

**Date:** 2026-11-04 · **Status:** research, nothing decided or built yet · **Agent team:** 4 agents (prior art, mechanism + maths, build + materials, repo fit)

The goal: a wrist (*håndledd*) with two joints, **bend** (`wrist_flex`, flexion/extension) and
**wave** (`wrist_dev`, side to side). All 16 servo tendon loops must pass through it. That is
**32 strands**, because each servo pulls one loop with a flex strand and an extend strand. The
servos stay in the forearm. The DIP joints are coupled to the PIP by the rigid bar inside each
finger, so they need no tendon through the wrist.

| File | What's in it |
|---|---|
| [01-prior-art.md](01-prior-art.md) | How others do it: human carpal tunnel, RUKA-v2, CRAFT-Wrist, ORCA, Tesla patent, MM-Hand friction |
| [02-mechanism-and-maths.md](02-mechanism-and-maths.md) | Coupling maths, bundle size, routing, torque, the recommended concept |
| [wrist_coupling.py](wrist_coupling.py) | The script behind the numbers in 02 (`uv run python cloud-claude/2026-11-04-v1-wrist/wrist_coupling.py`) |
| [03-build-and-materials.md](03-build-and-materials.md) | Tendon line, PTFE tubes, bearings, printing, assembly order, tensioning, BOM, checklist |
| [04-repo-integration.md](04-repo-integration.md) | What exists in the repo, conflicts, changes per layer, an 11-step plan, owner decisions |

## The short answer

1. **Every strand must run in a PTFE sheath across the wrist, with both sheath ends clamped.** This
   is the Bowden-cable idea used on bike brakes. The sheath length is fixed, so the line inside
   keeps its length however the wrist bends, and the fingers don't move. Bare strands are hopeless:
   at today's positions, waving the wrist would move finger joints by up to **25°**, and bending
   it would need hundreds of newtons of extra tension. (02, §1)
2. **Gather the 32 tubes into one bundle and pass it through a hollow wrist.** Today the strands
   are spread over 74 × 21 mm at the wrist plate. The bundle needs about 13–15 mm; with room for
   the sensor cable and clearance, plan a **≈ 22 × 14 mm oval opening**. Route the strands as a
   "crossed hourglass": each runs straight to the wrist centre, then on to its spool on the
   opposite side. (02, §2 and routing)
3. **Make the wrist gap ≥ 30 mm, not 20.** The tubes then bend at a radius of about 24 mm instead
   of 16 mm (the PTFE limit is about 15 mm), and they push the wrist back less. Leave some slack
   in each tube, because tubes at the edge of the bundle need up to about 6–7 mm more length at
   full bend. (02, §4–5; 03 slack section)
4. **Joint: a hollow oval cardan (ring cross).** Bend goes on the outer ring and wave on the
   inner ring, with 3 mm steel pins or M3 shoulder bolts in small 623/MR-class bearings, printed
   in PETG (PA-CF if you can), with hard stops. Make it symmetric about the hand's mirror plane so
   the left hand uses the same parts. (02, 03)
5. **Motors: two stronger servos, not SCS0009.** The hand weighs about 289 g with its centre of
   mass about 71 mm out, which is 0.20 N·m just to hold it level. A 0.5 kg object at the fingertips
   brings that to 1.2 N·m, and 1 kg to 2.2 N·m. Target **≈ 1.3 N·m continuous, 2.5 N·m peak**. The
   recommended drive is **2 × STS3215 with a 2:1 belt each**, moving the hand through two push rods
   with ball joints (the RUKA-v2 / CRAFT-Wrist pattern). SCS0009 and SCS2332 are too weak.
   *(Servo specs are from memory: verify them before buying.)* (02, §3; 01)
6. **Good news: gripping hard puts no load on the wrist** when the sheaths are used, because the
   line's pull and the sheath's push cancel inside the wrist. (02, §3)
7. **Use braided UHMWPE line, about 0.28 mm (30 lb) for most strands and 40–50 lb for `mcp_flex`**,
   instead of 0.4 mm nylon. It creeps and stretches less. Pre-stretch it, wrap it once around a
   post before the knot, and seal the knot with CA glue. Tension it with a **barrel adjuster on
   each sheath** at the forearm end. (03)

## Things to watch (the parts that can go wrong)

- **Friction grows with the wrist bend.** At full bend a sheath keeps about 88 % of the force
  (μ 0.1) or 73 % (μ 0.25). With the palm bends added, the worst thumb strand keeps only 43–71 %.
  So grip is weaker at extreme wrist angles. Measure μ on real parts. (02, 01)
- **The line shortcuts the bend inside the tube.** In a bend the line moves to the inside wall of
  the 1 mm bore, which loosens the loop by about 0.4 mm (about 12 N of pretension lost). A thinner
  liner (0.6 mm bore) cuts this to about 4 N. (02)
- **The tube squashes under load.** Over a 60 mm free span a 1 × 2 PTFE tube shortens by about
  2.3 mm at stall, which is about 22° of give at a 6 mm drum. That is softness, not coupling. Keep
  the free span short, or use a steel-coil sheath there. (02)
- **The forearm gets longer.** The strands need height to gather into the bundle and fan back out
  to the spools. Expect +40–60 mm of forearm, or a guide at each spool. (02, 04)
- **The sim is too ideal.** Today all 32 strands pass one point, so the fingers never feel the
  wrist. Make the sim match the real wrist once it is measured. (04)
- **The thumb strands are the hardest case** (up to 60° kink), as everywhere else in V1.

## Decisions for you

1. **Wrist motors:** 2 × STS3215 + belts (recommended), or another servo? Where do they sit (forearm
   level 3, beside the ESP32)?
2. **Whose controller:** wrist servos on the hand bus (IDs 17–18, a protocol change to firmware
   0.5.0) or on the arm side, like OpenArm's J5? I won't change the protocol until you say yes.
3. **Wrist ranges:** keep the sim's bend −60…+70° and wave −30…+20°? Smaller ranges make everything
   easier (less friction, less tube slack).
4. **Finger error you accept** while the wrist moves (for example ≤ 1–3°).
5. **Order:** the palm and forearm need a rebuild for 16 servos anyway, so I'd **build the wrist into
   that same rebuild** rather than doing it twice.

## Suggested next steps (small and checkable)

0. Fix the one failing test (`test_json_is_up_to_date`, float noise in three thumb numbers).
1. Decide the points above.
2. Add the bundle layout and the "crossed hourglass" routing to `tendon_router.py`, with tests
   (every strand crosses the wrist inside the opening, bend radius ≥ 15 mm).
3. **Print a test wrist first** (a short forearm top, the cardan, a palm bottom, 32 tubes). Sweep it
   while holding the fingers: no finger joint should move more than 1°. Measure friction and tube
   slack.
4. Then the sim (real wrist geometry, mass, motors), then the Fusion palm/forearm rebuild, then
   the firmware.

The full 11-step plan is in [04-repo-integration.md §5](04-repo-integration.md).

## Disagreements between the agents

- **Servo choice.** 04 suggested SCS2332/HLS3606M class; 02's torque maths shows they are too weak.
  This summary follows 02 (STS3215 + 2:1 belt).
- **Length change with sheaths.** 04's quick estimate (up to 9.8 mm in a 20 mm gap) is the change
  in each sheath's end-to-end distance. With fixed sheath ends it shows up as **tube slack, not
  finger motion**, which matches 02's tube-length mismatch (§5). Both point the same way: give the
  tubes slack and use a ≥ 30 mm gap.
