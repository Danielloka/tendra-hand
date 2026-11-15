# Round 2 brief (2026-11-15): fix what didn't work + sensors on every joint and touch

Owner (verbatim): "Fix what didn't work. And also make a plan and add the sensor for seeing angle on each
joint. And pressure etc. find ways to wire and placements. Use agent team with suitable model."

Same rules as `BRIEF.md`: write only inside this study folder; numbers, not words; plain English.
Read `reports/10-review.md` first: it is the list of what didn't work.

## Work packages

| Report | Agent | Model | What |
|---|---|---|---|
| 12-finger-fixes | finger builder (07) | Opus | Review #1 pip flex on the palm side over an MR63 roller (no crossing), #2 MCP spool r 5 and ONE shared travel table with 09, #5 fuse cup sized to a real crimp, #6 axle wire slot, Hall magnet away from bearing races, abd slot-edge bends, bushing friction in η, real BOM cost, knuckle module width vs 19.5–20 mm finger pitch. Then (phase 2) integrate the sensors from report 15. |
| 13-wrist-load | wrist routing (09) | Opus | Review #3: bare strands push 0.5–1.7 kN through the wrist cardan and ~0.7 kN sideways on the eye plate. Redesign the load path; re-run 09 with the travel table from 12. |
| 08 (revised) | bench kit (08) | Sonnet | Reorder the kit per review #10: eye-plate wear test, crimp/fuse test with 07's cup, Hall-near-bearing test, pass limits tied to budget numbers. |
| 14-sensor-parts | sensor scout | Sonnet | Market and datasheet survey of angle, touch/pressure and tendon-force sensors, buses, wires and connectors, with prices and sizes. |
| 15-sensing-plan | sensing architect | Opus | Where every sensor goes (all 20 joints, 5 tips, phalanx pads, palm, tendon force), how it is wired through joints and the wrist, bus and MCU layout, power, firmware data rates; CAD of the key sensor parts. |

## Sensing scope (owner: "angle on each joint, pressure etc.")
- **Joint angle on every joint**: 16 servo joints + the 4 coupled DIPs (check the bar) + thumb. Absolute,
  ≤ 1° error, works next to steel bearings and magnets in neighbouring joints.
- **Pressure/touch**: 5 fingertips (3-axis force), pads on proximal/middle phalanges, palm.
- **Tendon force** (optional, if cheap): servo load or a strain element.
- **Wiring**: through joint axes (twist, not bend), service loops, connectors so a finger can be swapped,
  how many wires cross the wrist, fatigue life ≥ 1 M cycles target.
- Electronics stay off-the-shelf (breakouts/modules) for a first build; say what a custom flex PCB would
  improve later.
