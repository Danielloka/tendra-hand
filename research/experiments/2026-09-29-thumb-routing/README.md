# Thumb tendon routing (Tendra Hand V1), design study

**Status:** concept A chosen (2026-09-29). Stage 1 (router + tests) done; stage 2: the base is rebuilt in Fusion, the palm and forearm still need a re-run (see "Build status").

## What exists (measured in the Fusion design "Tendra Hand V1", world mm)

| Joint | Axis | Drum on the child | Notes |
|---|---|---|---|
| `thumb_cmc_rot` | Z through (1, 15.5) | groove r 6.5 in the base's bottom plate, z −1.5…0.5 | base turns on 4 mm pins: bottom z −7.5…−4.4 (in the bay floor), top z 35.2…38.5 |
| `thumb_cmc_flex` | X through y 10.0, z 11.4 | r 6 groove on the metacarpal, x ≈ 5.9 | 5.5 mm in front of the rot axis: **the two base axes don't meet** |
| `thumb_mcp_flex` | (1,0,1)/√2 at y −32.0 | r 6 on `thumb_mcp_link` | index-style knuckle |
| `thumb_mcp_abd` | (1,0,−1)/√2 at y −49.5 | r 6 groove on `thumb_proximal` | V1's 5th thumb DOF |
| `thumb_ip` | (−1,0,−1)/√2 at y −67.0 | r 6 on `thumb_distal` | |

The 10 thumb strands end at the bay floor (z −4.7), in two rows at x −5 and +6.5 (`tendon_router.py`). Nothing routes them further yet.

Sections through the base (x = 1 and z = 20, 11, 6, 0, −3, −6):
- The base is a **frame**: a bottom plate (z −4.4…2.5, y 6…25) with the rot drum groove in it, two side walls (x ≈ −9 and ≈ 10), a top arm (z 31…35) and the two pins.
- The metacarpal's proximal block sits **directly on top of the plate** (bottom at z ≈ 4, back face at y ≈ 11), in front of the rot axis.
- Free space: only the frame's inside **behind** the metacarpal (x −8…9, y 12…28, z 3…30).
- The side walls sweep out to r ≈ 14 mm around the rot axis over the −100…+40° range.

## Problems found

1. **No path for a sheath into the metacarpal.** The rotating frame cuts anything that crosses from the fixed palm into the frame, except along the rot axis. The rot axis is blocked by the solid bottom pin (and the top pin). The 1.5 mm gap between the plate and the metacarpal leaves no room for a tube loop.
2. **`cmc_rot` can't be driven as routed.** Its two strands come up **vertically** from the floor. A strand parallel to the Z axis makes no torque about it; it has to reach the drum horizontally, in the groove's plane. (The MuJoCo model hides this: its "guide" point is a virtual pulley on the palm.) The side walls also sit in the groove's layer at x ≈ 10…11, so a horizontal strand leaving the groove toward the palm would be cut by the wall at some angles.
3. **Bare strands crossing `cmc_flex` on its axis** would bend by 90° + q (up to about 170°) over a PLA edge, because they arrive from below and leave along −Y. That's the reason for sheaths here.

## Concepts

**A. Rework the thumb base (hollow pivot + tube loop), recommended**
- Bottom pivot → hollow journal (about Ø10 outside / Ø7 inside) turning in the bay floor. 8 tubes (cmc_flex, mcp_flex, mcp_abd, ip × flex/ext) come up the rot axis. On the axis, cmc_rot only twists them, which a sheath doesn't care about.
- The metacarpal pivot moves up about 10–12 mm (or the plate gets thinner around the axis), so the 6 distal tubes can loop (bend radius ≥ 8–10 mm) from the journal into sockets in the metacarpal's back/top, over the whole cmc_flex range (−13…+80°).
- The cmc_flex tubes end on the base, tangent to the metacarpal drum.
- cmc_rot: its palm channels end **horizontally** in the bay wall, tangent to the base drum (tube stop in the wall). Keep the side walls out of the groove's layer.
- Beyond the metacarpal: 1.2 mm holes / 1 mm slits on the mcp_flex, mcp_abd and ip axes, like `TendraIndexPinch`.
- Cost: redesign of the base and the metacarpal's proximal end, a new bay floor, re-routed thumb channels. The thumb base's kinematics change a little (re-export + re-generate the sim).

**B. Outside loop (ORCA style)**
- Keep the base. The 8 tubes leave the palm's thumb side (x ≈ 12, below the bay) and loop in the open to sockets on the metacarpal's side.
- Cost: least CAD. But the loop has to follow a point about 28 mm from the rot axis through 140° (≈ 70 mm of travel), so it needs roughly 100 mm of slack tube. It hangs outside the hand where it can snag objects, and longer tubes mean more friction.

## Open numbers to check before cutting

- Minimum bend radius of the 1 × 2 mm PTFE tube without kinking (datasheet or a quick test; ORCA-type designs use about 10 mm).
- Tube friction vs. wrap angle (capstan: T_out = T_in · e^(μθ), μ ≈ 0.1–0.2 line-on-PTFE). 180° of total bend costs about 27–47%.
- The coupling left in a sheath: the line moves across the 1 mm bore as the bend changes, up to about 0.3 mm × Δθ (≈ 0.5 mm, about 5° at a 6 mm drum, worst case). Measure it on the test print.

## Sizing concept A: sheath loop search (2026-09-29)

`sheath_loop_search.py` (2D, in the plane x = const): a tube leaves the hollow pivot on the rot axis at the plate top, pointing up, and ends in a socket on the metacarpal. For socket positions near the cmc_flex axis, socket directions, and the tube's "bow" (Hermite tangent lengths), it keeps the loop that stays in the free space (y ≤ 27, z ≤ 29, above the plate), doesn't pass through the metacarpal block, and has the largest **worst-case bend radius** over cmc_flex −13…+80°.

| Plate lowered by | Best worst-case bend radius | Socket (q = 0) | Socket direction |
|---|---|---|---|
| 0 mm | 11.0 mm | y 11, z 20 | 45° up-forward |
| 4 mm | **15.5 mm** | y 9, z 23 | 45° up-forward |
| 8 mm | 16.1 mm | y 9, z 23 | 45° up-forward |

Only 2D (x spread of the 6 tubes, twist and tube stiffness ignored). The shape: the tubes rise up the rot axis behind the metacarpal, bow over, and enter a **boss on the metacarpal's top, just above and in front of the cmc_flex axis** (the top is at z ≈ 18 now, so the boss is about 5 mm tall).

**Chosen geometry for the build:** lower the base plate and bay floor **4 mm** along the rot axis (no kinematic change: moving along a rotation axis doesn't move the joint). That meets the palm's own 15 mm bend rule. The index channels then have to stay beside the bay down to about z −13 instead of −9 (re-check the router's bend-radius test).

Build order (each a `TendraHandV1` stage, tested before the next):
1. `tendon_router.py`: the thumb's own paths (sheath ends, sockets, on-axis crossing points, drum tangents) + tests, and the new bay floor.
2. Base: plate −4 mm, hollow journal (Ø10 / Ø7), side walls kept out of the drum groove's layer, cmc_flex sheath stops.
3. Palm: deeper bay, journal bearing, twist chamber, new thumb channels, horizontal feed for cmc_rot.
4. Metacarpal: socket boss + 6 curved tube channels ending before mcp_flex.
5. mcp_link / proximal: 1.2 mm hole on the mcp_flex axis and slit on the mcp_abd axis (like `TendraIndexPinch`).
6. Re-export, regenerate the sim, interference and clearance checks over the whole range.

## Build status (2026-09-29)

**Stage 1, router: done.** `hardware/cad/tendon_router.py` (constants under "Thumb base and routing", `thumb_layout()` → the `"thumb"` section of `tendon_routes.json`), tests in `hardware/cad/tests/test_thumb_routing.py`. What the search and tests settled:

| Item | Value | Why |
|---|---|---|
| Plate / bay floor | 4 mm lower (plate z −8.4…−1.5, floor −8.7) | tube loop bend ≥ 14.3 mm |
| Hollow journal | Ø14 outside, bore Ø11.6, bearing 3 mm deep in the floor (U-slot to the front) | 6 sheaths + 2 bare strands |
| cmc_rot drum | r 7.5 in the plate (z −4.7), flange r 9, tie hole at 240° (20° wrap left at both limits) | the bore takes the middle, so **servo_per_joint = 1.25** (`config_v1.h`, sim) |
| cmc_rot feed | horizontal to +Y, through the bay's back wall, over a Ø3 steel pin (y 30.8, z −6.4, blind hole from the thumb side), then down | a strand parallel to Z can't turn it |
| cmc_flex drum | on the metacarpal over the rot axis, grooves at x 0 (flex) and 2 (ext); both strands leave its back; ext goes up over a Ø3 pin (y 17.7, z 18.5) on a hanger from the top arm, then down | lands both within 1.1 mm of the rot axis |
| cmc_flex hold | palm entry under the strand's plate crossing at cmc_rot = −30° (mid-range) | length change ≤ 0.25 mm over the whole rot range |
| Sheaths | per side 3 in the bore (flex strands −X, ext +X), sockets on a split boss on the metacarpal: 2 side by side + 1 above, entering 45° up-forward | ≥ 14.3 mm bend over cmc_flex −13…80°, no crossing |
| Palm | the sheath channels share one cavity down to z −25 (room to twist); the sheaths stop at the wrist plate bottom (last 2 mm 1.2 mm) | |
| Servo slots | thumb sheets F1 mcp_flex, F2 cmc_flex, F3 mcp_abd, B2 ip, B1 cmc_rot (servos 6, 7, 8 move) | channel order = journal order, no crossings |

**Stage 2, Fusion: base done, palm/forearm pending.** The `thumb_base` stage ran on "Tendra Hand V1" (base 4060 mm³, one solid, section checked). The palm and forearm must be rebuilt from the new routes. That means deleting the existing palm/forearm features (timeline item 93 "v1 palm trim tools" onward: the palm, forearm, servos, rigid group and the old-proximal removal) and re-running `palm` and `forearm`. That bulk delete was refused by the Claude Code permission check, so it waits for the owner. Until then the old palm still has the old floor (the new base overlaps it) and the old thumb channels.

**Still to do:** metacarpal (drum, cut the slab behind it, socket boss, internal channels to mcp_flex), on-axis holes at mcp_flex / mcp_abd / ip, export + sim, interference over the whole range, and the Ø3 pins (2.8 mm long for cmc_flex) in the BOM.
