# Synthesis after the wide phase (lead, 2026-11-14)

Six reports (01–06). This file fixes what the build phase (07–09) uses. Still a study, not a decision.

## Direction chosen for the build phase ("V2.1")

1. **Bare strands, no Bowden sheaths** (01 + 02 agree; 06's spring tube is the fallback).
   Strands run straight between hard guides on rigid parts. The wrist is crossed through an
   **eye plate** at the wrist centre (01 §3.1); the length change is linear in the wrist angles and is
   cancelled by the servos. Fallback where a housing must stay: PTFE-lined closed steel coil on an
   aluminium stop plate (02).
2. **Every bend over hard metal:** Ø3 ground dowel (D/d ≥ 6) for small fixed wraps, MR63ZZ rollers
   for big wraps (02 G1–G3). No eyelets with r < 1.5 mm, no PTFE/brass/print at point bends.
3. **Line:** UHMWPE braid PE#8 (≈ 0.47 mm, ≈ 445 N) (02). Verify size.
4. **Anchors with wraps, never a bare knot:** ≥ 1 capstan turn before the anchor (anchor sees ≈ 28 %),
   then a clamp (M2 + washers, 06 #1) or beads in a printed fuse cup that shears at ≈ 68 N to protect
   the servo (03). Fuse = a deliberate weak link that is cheap to replace.
5. **Spool cartridge** (02, built in `out/02/`): fixed Ø3 axle on 2 bearings, dog-pin drive (servo
   shaft sees torque only), chordal M2 screw tensioner, keeper ring, witness notch.
6. **MCP:** a strong, fixed full-length Ø4 axle in a "reverse fork" (03 §3.2, 69 N sideways vs 8 N).
   The pip strands cross the MCP over **coaxial MR63 idlers** on that axle (02 G12) instead of
   eyelets or cross holes (cross holes = same tight bend as the eyelet).
7. **Drums:** metal groove sleeve or split ring on every loaded drum (03, 01): PETG creeps above
   ≈ 12–15 MPa, the groove sees 18 MPa (SCS) to 47 MPa (HLS).
8. **Force:** ratio per joint (01 §3.2: PIP is the limiting joint in a pinch, not the MCP).
9. **Bearings:** MR84 pairs at MCP and PIP; DIP and thumb IP on Ø4 pin in a brass bush (03).
10. **DIP bar:** laser-cut/filed 0.8 × 3.2 stainless strip on Ø2 pins set 4 mm deep; pin A moved
    out of the 0.28 mm web (03).
11. **Printing:** fingers on their side, teardrop horizontal holes, open keyhole grooves instead of
    buried channels, forearm frame/spool rail in PAHT-CF (heat), metal-on-metal clamps, line-to-line
    bearing seats + retaining compound (04). Print the coupon `out/04/tendra_coupon.stl` first.
12. **Contact + sensing:** snap-in silicone skin with a printable mould and a magnetic tactile
    pocket (05, `out/05/`); wires through hollow pins on the axis; Hall joint-angle sensor pockets at
    MCP and PIP (01, 05). Joint-angle sensing is **required** once the drive is stiff and strong:
    line stretch alone is ≈ 13° at SCS stall (02).
13. **Load class:** design every part for HLS3606M (≈ 120 N strand); run SCS0009 first. MCP slots
    sized so STS3215 (45 N path, 12 V) is a later swap (01 §3.4).

## Conflicts the build phase must settle (with numbers)

- **Servo travel budget.** The pip servo already spends 140° (02); the MCP idler adds 0.60 × q_mcp
  (→ 200°); a 2:1 reeved PIP (01) doubles the pip strand travel; wrist compensation needs up to ~45°.
  SCS0009 has ≈ 300° (verify). Not all of these fit at once. Choose: reeve OR bigger spool OR
  smaller pip drum, and show the travel for every servo joint.
- **Reeved PIP turnaround pin (Ø3)** vs fatigue rules (D/d 6.4 with PE#8): acceptable?
- **Spool cartridge width** 14.3 mm vs the V2 forearm cassette pitch (+3 mm per side).
- **Eye plate wear** and minimum skew distance between 32 strands over the wrist range.
