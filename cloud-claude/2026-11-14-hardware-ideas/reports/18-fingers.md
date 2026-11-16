# 18 · Four V2.1 fingers with sensors + palm knuckle modules (round 3, package A)

Agent 07, 2026-11-16. Study only.

> **Status:** the final regeneration of `out/18/fingers/` was still running when I handed back. That run adds the
> layered liners, the abduction rule and the integrator-pose neighbour checks. Its log is
> `out/18/fingers/build.log`; it ends with `BUILD_DONE`. To rerun:
> `uv run --no-project --with cadquery --with numpy --with trimesh --with scipy --with rtree python cad/18_fingers.py`
> (~8 min). The liner layout and the rule below were already computed and checked on their own (numbers from those
> runs).

## API (top of `cad/18_fingers.py`)

```python
spec = importlib.util.spec_from_file_location("fingers18", "<study>/cad/18_fingers.py")
F = importlib.util.module_from_spec(spec); spec.loader.exec_module(F)
F.FINGERS, F.JOINTS        # ("index","middle","ring","little"); ["index_mcp_abd","index_mcp_flex","index_pip", ...]
F.parts()                  # {part: {body, finger, stl, color, printed}}  (out/18/fingers/parts.json)
F.pose(q)                  # {part: 4x4} hand frame, from the q = 0 export; q in deg, DIP from the bar (exact)
F.lines(q)                 # {name: (N,3)} tendons (drum anchor -> palm plate hole, z 30) + sensor wires (tip -> JST plug)
F.clamp(q), F.abd_limits() # the firmware abduction rule (below)
F.dip(finger, pip_deg)
F.build()                  # regenerates out/18/fingers/ (~8 min); pose/lines/clamp do not need it
```

- **Frame and bodies:** V2 hand frame (origin = wrist centre, +Z fingers, −Y palm, +X thumb). Body names are
  `<finger>_module` (static, palm; includes the liners), `_yoke`, `_proximal`, `_middle`, `_distal` and `_bar`.
- **How each finger is built:** each finger is `cad/16_finger.py` with its V2 lengths patched in. The patched copies
  are in `out/18/fingers/_gen/`.
- **Re-derived per finger:**
  - the DIP bar, from each finger's middle length. Error 0.43 / 0.52 / 0.33 / 0.43° for index / middle / ring /
    little; bar 21.6 / 24.9 / 23.7 / 17.1 mm.
  - the scaled pad positions.
  - the joint caps from out/15, moved to each finger's PIP and DIP.
- **No middle pad on the little finger:** its 20 mm middle phalanx has no room for one.
- **Finger positions:** V2 `params.FINGERS`, with **one change: ring x −20.0 and little x −40.0** (V2: −19.5 / −38),
  which makes the pitch an even 20 mm. With V2's 18.5 mm ring–little pitch and the little MCP 8 mm lower, the two
  fingers overlap (up to 407 mm³) when both abduct the same way.

## Palm knuckle module (replaces the bench base)

The module is a static printed part per finger (PA-CF or PETG), up to ±9.6 mm wide and limited to half the pitch
toward each neighbour.
- **Floor** (4 mm), 6.5 mm below the abd axis. Per strand it has a flared Ø1.5 passage and a socket for the Ø2 PTFE
  liner. It is relieved around the abd disc.
- **Palmar post** with the fixed Ø3 abd stub, standing 1.9 mm proud of the V2 palm face as a knuckle bump.
- **Dorsal lug** with the MR63 seat, the abd sensor cap, the knuckle board and the JST SH socket/plug.

**Liners** (report 13 §5) run from the module floor to the palm plate.
- **Method:** each Ø2/1 PTFE liner goes entry → its depth layer (8 layers over |y| ≤ 10.5) → straight run → liner end
  at z 40–49 above its plate hole. The last part to the steel plate eye at z 30 is bare strand; the eye takes that
  bend. Per strand the layer, the drop height and the end height were chosen by simulated annealing (best of 4 seeds).
- **Result:** the worst liner centreline distance is **2.45 mm**, against Ø2 liners plus a 0.4 mm wall. **No overlaps.**
  The integrator had found 23 overlapping liners, up to 6.6 mm³; that came from the straight-line version.
- **Bends inside the liners:** 64° median, 111° worst, so η ≈ 0.82 worst at μ 0.1 for PTFE. The hole order on
  report 13's plate is set by the wrist eye pairing; reordering the plate would cut these bends.

## Firmware abduction rule (from the neighbour scan, 20 mm pitch)

1. Neighbours may only spread: abd(radial finger) − abd(ulnar neighbour) ≥ 0. Every relative adduction of 7.5° or more
   collides; this is the same rule as V2's software limit.
2. `index_mcp_abd ≥ −10°`: index and middle both at −15° touch (7 vertices), because the index MCP is 4 mm lower.
3. `little_mcp_abd ≤ +7.5°`: ring 15 / little 10 touches (6–10 vertices), because the little MCP is 8 mm lower.
   Ring 11 / little 9 is clear.

`F.clamp(q)` applies the rule. With the 20 mm pitch build, the integrator's `spread` and `thumb_to_little` poses
are clear in the vertex scan. The ring/little contacts the integrator reported (33/21 mm³ and 1.2 mm³) match the
earlier V2-pitch build. The final build re-checks these poses with OCC solids (`checks.json → neighbours`).

## Checks (per finger, last complete run)

- **Validity:** every part is a single valid solid.
- **Interference** at q0, abd ±15 with MCP 45 / PIP 50, the fists MCP 90 / PIP 95, and abd −15: clear, with two
  exceptions.
  - **Index:** at abd 15 with MCP 90 / PIP 95, the nail shim touches its knuckle module (0.7 mm³).
  - **Little: a full fist (MCP 90 / PIP 95) folds its pad onto its own knuckle module and yoke (up to 130 mm³).**
    A 34 mm proximal puts the fingertip on the palm-side knuckle bump. The integrator's fist (little MCP 87.5 /
    PIP 87.5) is close to this. Either limit the little to about MCP 80 + PIP 85 or accept it as palm contact; it
    would also be cleaner to drop the palmar post below the palm face.
- **Loops, couplings and the travel table:** unchanged from 12/16 (same drums, idler and lanes). Pip loop
  0.00002 mm, MCP loop 0.001 mm; `out/12/travel_table.json` still applies.
- **Tip cavity on the little finger:** the end wall is 1.5 mm (distal 21 mm long).

## Pinch per finger (estimate)

This is the index value scaled by the PIP drive lever, taken as proportional to L2 + L3. It is not a new statics run.

| Finger | SCS0009 stall | HLS3606M stall |
|---|---|---|
| index (from 12) | 5.9 N | 16.6 N |
| middle | ~5.4 N | ~15.3 N |
| ring | ~5.7 N | ~15.9 N |
| little | ~6.8 N | ~19.0 N |

## Files

- `cad/18_fingers.py`
- `out/18/fingers/`:
  - `parts.json`, `stl/<finger>_<part>.stl` (hand frame, q = 0), STEP for the printed parts
  - `fingers_q0.glb` (parts + tendon lines + sensor wires)
  - `fingers_kin.json` (DIP links, liner layout)
  - `checks.json`, `abd_pair_scan.json`
  - `fingers_{open,fist,spread}.png` (written at the end of the build)
