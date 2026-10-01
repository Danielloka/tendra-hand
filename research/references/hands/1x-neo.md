# 1X Technologies and the NEO hand

Research note for Tendra Hand. Written 2026-09-30.

> **Caveat on method.** The research session broke off early: after the first four web searches and two page fetches, the search/fetch tools kept failing on a permission check that returned no verdict. So this note rests on **one primary source** (1X's own hand page, 9 July 2026), one secondary article, and search-result snippets. Every claim is tagged:
>
> - **[C]** confirmed from a source listed at the end (URL given)
> - **[S]** from search-result snippets or secondary press only; likely right, but not checked against 1X
> - **[M]** from the author's background knowledge (up to mid-2026), **not re-checked this session**. Verify before relying on it.
> - **[I]** inferred or speculative (my own reasoning)
>
> Follow-ups are listed in the section "Open questions / what to verify next".

---

## 1. The company in one paragraph

- 1X Technologies started out as **Halodi Robotics**, founded in Norway in **2014** by **Bernt Øivind Børnich** (still CEO) and **Nguyen Ho Quoc Phuong** **[S]** (CB Insights snippet; both are the inventors on Halodi's 2017 patent **[C]**). Moss location and the US presence (Palo Alto/Sunnyvale) **[M]**.
- Robots:
  - **EVE**: a wheeled humanoid, used in security-guard and logistics pilots from about 2022–2023. **[M]**
  - **NEO Beta** (shown in Aug 2024): the first legged prototype for homes. **[M]**
  - **NEO Gamma** (Feb 2025): a home prototype with a soft knitted "suit" over the body. **[M]**
  - **NEO** (the consumer product): announced for preorder on **28 Oct 2025**. **[M]** **[S]**
- Funding and deals: OpenAI Startup Fund (2023), a $100M Series B led by EQT (Jan 2024), and the purchase of Kind Humanoid (early 2025). **[M]**
- Consumer NEO: **$20,000 Early Access**, with priority delivery in 2026, or **$499/month** as a subscription that ships later. **[S]** ([Robot Report](https://www.therobotreport.com/1x-announces-pre-order-launch-neo-humanoid-robot/), [botinfo](https://botinfo.ai/articles/1x-neo-home-robot))
- Delivery status: first deliveries were targeted for 2026, US first (Canada is also mentioned), with other markets from 2027. One secondary source says **no customer delivery had been verified as of 16 July 2026**. 1X's July wording was "some this year, some later", against more than 10,000 preorders. **[S]** ([robozaps](https://blog.robozaps.com/b/1x-neo-review))
- Body specs often quoted for the consumer NEO:
  - ~30 kg (66 lb) and ~168 cm tall
  - "22 dB" quiet
  - lifts 154 lb (~70 kg) and carries 55 lb (~25 kg)
  - 18 lb (~8 kg) payload per arm
  
  **[S]** for the lift, carry and arm payload ([Robot Report](https://www.therobotreport.com/1x-announces-pre-order-launch-neo-humanoid-robot/), [robozaps](https://blog.robozaps.com/b/1x-neo-review)); **[M]** for weight and height.
- Teleoperation ("Expert Mode"): 1X said remote human operators can take over NEO for tasks it can't yet do autonomously. That drew privacy criticism at launch. **[M]** **[S]** ([INT News](https://intnews.it/en/neo-humanoid-robot-1x-opens-pre-orders-between-cost-and-privacy/))

## 2. The NEO hand (revealed 9 July 2026)

1X published a dedicated page, **"NEO's Hands | An API to the Physical World"**, dated **9 Jul 2026**. **[C]** https://www.1x.tech/discover/neos-hands

### Spec table

| Item | Value | Status |
|---|---|---|
| Total DOF | **25**: 22 fully actuated in fingers + palm, 3 at the wrist | [C] 1X page |
| DOF per finger / joint layout | **Not published.** 1X says the layout is "biased toward a thumb that genuinely opposes". | [C] that it's unpublished |
| Palm DOF | Yes, the "palm" counts toward the 22 (e.g. a palm arch/cupping joint). Exact joints not given. | [C] palm counted; [I] what it is |
| Actuation | "1X Motors" (in-house, high torque density) pulling tendons: the **"1X Tendon Drive"**, described as quasi-direct-drive | [C] |
| Motor location | **Forearm** | [C] |
| Transmission ratio | **~5:1 to 15:1** | [C] |
| Backdrivability | **All 25 joints fully backdrivable** | [C] |
| Control mode | **All 25 DOF "natively force-controlled"** (motor current = joint force, with no separate force sensor per joint) | [C] force control; [I] current-based mechanism (also described that way by [S] aiweekly) |
| Peak torque | **3.5 N·m** thumb CMC, **2.6 N·m** finger MCP | [C] |
| Fingertip (distal) flexion force | **up to 45 N** | [C] |
| Wrist torque | **17.75 N·m** | [C] |
| Positioning accuracy | **±0.2 mm** | [C] |
| Grip / pull | "can deadlift 70 kg" | [S] droids.substack |
| Tendon material | 1X: "**proprietary tendons**", with "tendon materials" made in-house **[C]**; humanoid.guide: "**polymer tendons**" **[S]**. Grade not named. Halodi's 2017 cable-drive patent specifies **Vectran** fibre cables in tubular sleeves **[C]**, so Vectran (or UHMWPE) is a good guess **[I]** | see left |
| Tactile sensing | High-resolution tactile skin on fingertips and hand surfaces: **normal force, contact location, shear**; **real-time slip detection** (e.g. "feel a glass slipping") | [C] |
| Cameras in palm | Not mentioned | unknown |
| Joint encoders | Not stated. With force-controlled BLDC-style motors they must have motor-side position sensing; joint-side sensing unknown. | [I] |
| Covering | Soft polymers with integrated tactile skin; "pinch-proof joints" | [C] soft polymer + skin; [S] pinch-proof |
| Sealing / hygiene | **IP68**, **food-safe** materials (can be washed) | [C] |
| Durability | Parts tested to millions of cycles; the wrist to "well beyond **2 million cycles** under high loads"; tested at extreme temperatures | [C] |
| Hand weight | Not published | unknown |
| Finger speed | Not published | unknown |
| Manufacturing | Fully in-house, "end-to-end ... from tendon materials and 1X Motors to the final soft polymers, skin, and tactile sensing stack"; "**hundreds** of these hands have already come off a scalable production line"; capacity **10,000 hands in 2026** | [C] (1X page re-read 2026-09-30) |
| Noise | NEO as a whole: 22 dB | [S] |

### How it works (plain words)

1. **Motors in the forearm, tendons to the fingers.** This keeps the hand light and slim, and lets the motors be big enough to be strong without a big gearbox. It's the same basic layout as Tendra V1 (servos in the forearm, 40 strands through the wrist). **[C]**
2. **Low gear ratio (5–15:1) instead of 100–300:1.**
   - A hobby servo like our SCS0009 has a very high gear reduction. The motor is small and fast, and the gears trade that speed for torque. The cost is friction and reflected inertia: you can't push the finger back by hand (not backdrivable), and the motor can't "feel" what the finger touches.
   - With only 5–15:1, a force on the fingertip travels back to the motor almost unchanged, so the **motor current tells you the contact force**. 1X calls every joint an "active force sensor". **[C]** (1X page; aiweekly)
3. **Backdrivable = safe and robust.** If the hand hits something or someone, or the robot falls, the joint gives way instead of breaking or hurting. This is 1X's core argument for a home robot. They have described NEO as "naturally compliant to withstand falls". **[C]** backdrivable; **[S]** fall quote
4. **Force control, not position control.** The controller commands a torque (or an impedance: stiffness + damping around a target), not a rigid angle. The learned policy can then touch, press and slide gently. **[C]** force control; **[I]** impedance form
5. **Tactile skin on top** adds what motor current can't give: *where* the contact is, *shear* (sideways force), and *slip*. **[C]**

### 1X's design philosophy (why tendons, why low gear ratio)

- **Since the Halodi days, 1X has built its own high-torque-density motors** so it can run low gear ratios. EVE used an in-house direct-drive servo called **"Revo1"** **[S]**. Halodi's patent describes such a motor: a **Halbach-array rotor** with the field directed toward the rotation axis **[C]**. The hand page calls the current ones "1X Motors". **[C]**
- Their line: safety around people, and learning, both need a body that is **compliant and backdrivable**. Rigid, high-ratio gearboxes and harmonic drives are what they avoid (droids.substack notes "no rigid gearboxes or harmonic drives"). **[S]**
- **Tendons act like muscles**: the heavy parts (motors) sit near the body, and light, strong strings do the pulling. This is the classic anthropomorphic-hand approach. **[C]**/**[S]**
- **Data comes first.** Børnich has argued many times that general-purpose robots need huge amounts of real-world data, and that a safe, soft robot living in homes is the way to collect it. **[M]**

## 3. Control and learning stack

These are all **[M]**: 1X published them before mid-2026, but they were not re-fetched this session.

- **Teleoperation data collection:** VR teleoperation by in-house operators (the "Android operators" in 1X's EVE-era job posts). Consumer NEO keeps a remote "Expert Mode".
- **1X World Model** (from Sept 2024): a learned video/action simulator used to *evaluate* policies without running them on real robots. 1X ran a public "World Model Challenge" with a dataset on GitHub/Hugging Face.
- **Redwood AI** (announced mid-2025): 1X's onboard vision-language-action model for mobile manipulation in homes (walking + grasping). It is trained on teleop plus autonomous data and runs on NEO's own computer.
- **Low level:** torque/force-controlled joints (confirmed for the hand **[C]**). A learned policy outputs targets; a fast inner loop turns them into motor currents. **[I]** for the exact split.

## 4. Patents (Halodi / 1X)

Searched 2026-09-30 (Google Patents, FreePatentsOnline, web search).

- **WO2018149499A1, "Human-like direct drive robot"** (also US 2020/0083763 A1, CA3052893A1; family filings in US, CA, EP, JP). Applicant **Halodi Robotics AS**; inventors **Phuong Nguyen, Bernt Øivind Børnich**; priority **16 Feb 2017**, published 23 Aug 2018 **[C]**. What it claims/describes:
  - a compact high-torque motor with a **Halbach-array rotor** (magnet thickness chosen to avoid demagnetisation at max current and temperature);
  - **cable-driven joints: two driving cables per joint, wound in opposite directions** on the driving and driven axles (one tightens as the other pays out), i.e. exactly Tendra's **antagonistic loop on one spool**;
  - motors placed proximally (toward the torso), symmetric about the limb axis, to cut moving mass;
  - gear ratio **at most 1:5–1:10, ideally 1:3, 1:2 or 1:1** (vs 1:50–1:200 in conventional robots) for backdrivability;
  - cables of **Vectran** synthetic fibre in tubular sleeves, with ball-bearing termination housings.
- **No 1X/Halodi patent specifically on the NEO hand was found.** A PatSnap 2026 landscape of tendon-hand patents lists no 1X or Halodi filings **[C]**. A hand filed in 2024–2025 may still be unpublished (applications publish ~18 months after filing) **[I]**.
- Lesson for Tendra: 1X's hand is the scaled-up descendant of the same idea Tendra uses (one motor, two opposite cables, low friction), but with a ~1:5–1:15 ratio and a motor strong enough to need no big gearbox.

## 5. Comparison with Tendra V1 (rough numbers)

These are all **[I]**, my own estimates. SCS0009 stall torque is ~0.23 N·m, tendons are on 1:1 drums/spools, and I take the 6 mm figure as a radius (if it's a diameter, the strand forces double and the joint torques stay the same at 1:1).

| | NEO hand | Tendra V1 |
|---|---|---|
| Actuated DOF (hand) | 22 (+3 wrist) | 20 (no wrist) |
| Actuator ratio | 5–15:1, backdrivable | SCS0009 high internal gearing, **not backdrivable** in practice |
| MCP peak torque | 2.6 N·m | ~0.23 N·m at stall (≈ 11× less) |
| Fingertip force | up to 45 N | a few N (0.23 N·m / ~0.09 m lever from MCP ≈ 2.5 N) |
| Force sensing | motor current at every joint + tactile skin | SCS0009 "present load" register (coarse) |
| Tendon | proprietary polymer, in-house (Halodi patent: Vectran) | 0.4 mm mono fishing line in PTFE |
| Motor site | forearm | forearm |

Takeaway: the **layout is the same idea** (tendons from forearm motors). The big gap is the **actuator**, where 1X gets strength, backdrivability and force sensing together, plus **tactile skin**. The gap is also partly by design: Tendra is a hobby-cost research hand. **[I]**

## 6. What Tendra can learn / implement

Ordered by value for effort on a hobby budget.

### Priority 1: cheap, do now

1. **Use the SCS0009's load feedback as a "poor man's force sensor".**
   - The firmware already has `F` (feedback). Log `present load` together with position during grasps in the V1 sim-to-real tests.
   - Detect contact as load rising while the position stalls. Then stop, or hold at a set load instead of driving to the target angle.
   - This is a crude version of 1X's "every joint is a force sensor". Cost: €0.
2. **Software compliance / soft position mode.**
   - Command a target just past contact and cap the servo's torque limit register, which gives a spring-like grip.
   - Keep pretension low (SCS0009 backs off after 2 s above 80% load, as noted in CLAUDE.md).
   - This mimics 1X's force/impedance control on a position servo.
3. **Upgrade the tendon to braided UHMWPE (Dyneema/PE braid, ~0.3–0.4 mm).**
   - 1X uses polymer tendons. Nylon monofilament stretches and creeps, which hurts open-loop accuracy.
   - PE braid is far stiffer and slides well in PTFE. ~€10 per spool. Test it on the V0 index first, and measure stretch under load.
4. **Soft, pinch-proof covering.** Add TPU fingertip pads (already planned) and a thin knit or nitrile glove over the hand. 1X puts a lot of weight on soft skin for safety and grip. Cost: a few €.
5. **Cycle-test rig.**
   - Script the V0 index to flex and extend for thousands of cycles and log tendon slack or position drift.
   - 1X's "millions of cycles" shows durability data is part of the product. This makes a good research-log experiment.

### Priority 2: moderate cost, next few months

6. **Cheap tactile fingertips (normal + shear + slip).**
   - Magnetic skin (ReSkin/AnySkin-style): a small magnet or magnetised elastomer over a 3-axis Hall sensor (MLX90393, ~€5–10 per fingertip, I²C).
   - This gives normal and shear force, and slip from high-frequency changes. Start with the index and thumb tips.
   - A cheaper but cruder option is Velostat piezoresistive pads.
7. **Put the tactile data and servo load into the sim and RL.**
   - Add MuJoCo touch sensors on the fingertips to `GraspEnv`, so policies learn to use contact like NEO's slip detection.
   - Tactile observations also help the asymmetric critic.
8. **Add a wrist.** NEO has 3 wrist DOF with serious torque (17.75 N·m). For the bimanual station, the arm or wrist will do much of the dexterous positioning. Plan at least 2 wrist DOF for the arms.

### Priority 3: experiments toward the 1X approach (later, ~€30–80 per joint)

9. **One quasi-direct-drive tendon finger as a research build.**
   - Parts: a small gimbal BLDC (e.g. 2204/2804), SimpleFOC driver + AS5600/AS5047 encoder, and a ~5–10:1 capstan or belt stage onto the tendon spool.
   - This gives current-based force sensing and real backdrivability on one joint (e.g. the index MCP).
   - Compare it with the SCS0009 on force accuracy, speed, heat and cost. It shows whether the 1X route is affordable for Tendra V2.
10. **Force-controlled loop pretension.** With backdrivable motors, a slack tendon can be detected and taken up automatically, so manual re-tensioning goes away.
11. **Data strategy like 1X.** Keep investing in teleop (webcam and later a glove) and dataset recording (`tendra.dataset`, LeRobot export). 1X's bet is that data, not mechanism alone, makes the hand useful.

### What not to copy (yet)

- Full in-house motors, IP68 sealing and food-safe production are factory-scale work. They don't fit a hobby budget or this stage.

## 7. Open questions / what to verify next

- The exact 22-DOF layout: which fingers get abduction, whether each DIP is independent or coupled, the thumb DOF count, and the palm joint(s).
- The tendon grade, the routing (sheaths vs pulleys), and how the forearm motors are arranged.
- Hand mass, finger speed, and the tactile sensor technology (capacitive? magnetic? optical?).
- Patents: done 2026-09-30 (see section 4); re-check for new 1X hand filings in 2027.
- Delivery status of consumer NEO after July 2026, and any teardown.
- Sources to re-fetch: the Forbes article (9 Jul 2026), The Next Web, humanoid.guide/1x-neo-hands, Interesting Engineering, Robotics & Automation News (17 Jul 2026), aitechinsights. All were found but could not be opened this session.

## Sources

Opened and read:
- 1X, "NEO's Hands | An API to the Physical World", 9 Jul 2026. https://www.1x.tech/discover/neos-hands
- DROIDS! (Substack), "These Might Be the Best Robot Hands Ever Built", 10 Jul 2026. https://droids.substack.com/p/these-might-be-the-best-robot-hands

Seen in search results only (snippets):
- AI Weekly, "1X Debuts Neo's Tendon-Driven Hands With 25 Degrees of Freedom". https://aiweekly.co/alerts/1x-debuts-neos-tendon-driven-hands-with-25-degrees-of-freedom
- Forbes (J. Koetsier), 9 Jul 2026. https://www.forbes.com/sites/johnkoetsier/2026/07/09/human-level-hands-1x-just-gave-humanoid-robot-neo-something-close/
- The Next Web. https://thenextweb.com/news/1x-neo-robot-tendon-driven-hands
- Interesting Engineering. https://interestingengineering.com/ai-robotics/1x-unveils-robot-hands-neo-humanoid
- humanoid.guide, "1X NEO Hands – 25 Degrees of Force Control". https://humanoid.guide/1x-neo-hands/
- Robotics & Automation News, 17 Jul 2026. https://roboticsandautomationnews.com/2026/07/17/1x-unveils-25-degree-of-freedom-humanoid-robot-hands-for-neo/103405/
- AI Tech Insights. https://aitechinsights.com/articles/what-specific-engineering-breakthroughs-allow-the-1x-neo-s-new-hands-to-replicate-human-dexterity/
- 1X NEO product page. https://www.1x.tech/discover/neo-home-robot
- The Robot Report, NEO preorder. https://www.therobotreport.com/1x-announces-pre-order-launch-neo-humanoid-robot/
- RoboZaps review. https://blog.robozaps.com/b/1x-neo-review
- botinfo.ai, NEO price. https://botinfo.ai/articles/1x-neo-home-robot
- INT News, preorders and privacy. https://intnews.it/en/neo-humanoid-robot-1x-opens-pre-orders-between-cost-and-privacy/
- humanoid.guide NEO product page. https://humanoid.guide/product/neo/

Sources added in the 2026-09-30 fact-check:
- 1X, "NEO's Hands" (re-read): https://www.1x.tech/discover/neos-hands ; humanoid.guide summary: https://humanoid.guide/1x-neo-hands/ (no new layout, weight or speed data on either page)
- Halodi patent WO2018149499A1: https://patents.google.com/patent/WO2018149499A1/en ; US 2020/0083763: https://www.freepatentsonline.com/y2020/0083763.html
- PatSnap tendon-hand patent landscape 2026 (no 1X/Halodi entries): https://www.patsnap.com/resources/blog/articles/tendon-driven-robot-hand-patents-2026-landscape/
- Founders/year (snippet): https://www.cbinsights.com/company/halodi-robotics

Fact-checked 2026-09-30 (partial): NEO hand page specs (unchanged, plus "proprietary tendons" and "hundreds" built), Halodi/1X patents, founders. Still [M]: company history (EVE, NEO Beta/Gamma dates, funding), Redwood AI, World Model; hand weight, finger speed, DOF layout and tactile technology remain unpublished.
