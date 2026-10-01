# Humanoid company hands (2025–2026)

Status: research report, 2026-09-30. It covers the hands of the leading humanoid companies, the trends behind them, and what Tendra Hand can learn. 1X NEO has its own report ([1x-neo.md](1x-neo.md)) and only gets a comparison row here.

**How to read the tags**

- **[C] confirmed**: stated by the company, a patent, or a reputable news outlet quoting the company. The source is linked.
- **[R] reported**: from a secondary source (spec aggregator, blog, reseller) that doesn't show where it got the number. Probably right, but check before relying on it.
- **[S] speculative / inference**: my own reasoning or an unverified supply-chain claim.

Numbers not in this file were not found. Nothing is filled in from guesswork.

---

## 1. Summary table

"DOF" means actuated DOF unless it says otherwise. "Where" = where the motors sit.

| Company / hand | DOF (hand) | Actuation, where | Transmission | Sensing | Force / payload | Weight | Tag |
|---|---|---|---|---|---|---|---|
| **Tesla Optimus Gen 1 hand** (2022) | 11 DOF, 6 actuators | Electric, in the hand | Metallic tendons, non-backdrivable drive (holds grip with power off) | In-hand controller with sensor feedback | n/a | n/a | [C] |
| **Tesla Optimus Gen 2** (Dec 2023) | 11 | Electric, in the hand | Tendons | Tactile sensing on all fingers | n/a | n/a | [C] |
| **Tesla Optimus V3 hand** (patents 2026; robot not unveiled as of mid-Sept 2026) | 22 on the hand + 2–3 wrist; 4 per finger | Electric, **all in the forearm**; patent: 25 linear actuators per forearm (23 hand, 2 wrist) | 3 cables per finger, braided polymer, in lubricated lined tubes; **springs extend the fingers**; ball-screw linear actuators [R] | "Much more" tactile coverage planned | n/a | n/a | [C] patent / [R] counts |
| **Figure 02** (Aug 2024) | 16 | Electric, reportedly a self-contained motor unit per finger [R] | n/a | n/a | 25 kg robot payload | n/a | [C] DOF / [R] layout |
| **Figure 03** (Oct 2025) | Not published officially (Wikipedia says 20) | n/a | n/a | Fingertip tactile down to **3 g**; **palm camera** in each hand; softer fingertips | n/a | n/a | [C] sensing |
| **Boston Dynamics Atlas** (GR2 gripper, 2025; product Atlas CES 2026) | **7 DOF, 7 actuators, 3 fingers** (2 per finger + thumb swivel) | Electric, self-contained gripper module | n/a | Tactile fingertips in elastomer; palm cameras | Robot: 50 kg max payload | n/a | [C] |
| **Sanctuary AI Phoenix** | **21** | **Hydraulic**, miniature valves | Hydraulic actuators | Fingertip arrays on micro-barometers (7 cells/pad [R]) | n/a | n/a | [C] |
| **Apptronik Apollo 2** (June 2026) | Uses **SharpaWave** (22 DOF) and Inspire hands | See SharpaWave | n/a | SharpaWave: 1,000+ taxels per fingertip | Sharpa: 20 N fingertip | Sharpa: 1.3 kg | [C] |
| **Agility Digit 5** (Sept 2026) | Swappable end effectors, ISO flange; currently a 2-finger gripper | Electric | n/a | n/a | 22.7 kg (50 lb) robot lift | n/a | [C] |
| **Unitree Dex3-1** | 7 (3 fingers) | Electric, in the hand: 6 direct-drive + 1 geared brushless joints | Direct drive | 33 pressure sensors (10 g–2.5 kg) | 500 g max object | 710 g | [C] |
| **Unitree Dex5-1** | 20 (16 active + 4 passive) | Electric, coreless ("hollow-cup") motors in the hand | 12 composite force-controlled joints + 4 geared joints | 94 tactile sensors (Dex5-1P) | 10 N fingertip; 3.5–4.5 kg | 1.1 kg | [C] |
| **Unitree Dex5-S** (21 Sept 2026) | **22 active**, all backdrivable | Motor-driven every joint, dual encoders | n/a | Tactile defined per quote | ~2 kg peak payload [R] | n/a | [C] DOF / [R] rest |
| **Fourier GR-3** (Aug 2025) | 12-DOF hands; FDH-6 hand: 6 DOF, 11 joints | Electric | n/a | 31 body pressure sensors (robot skin) | n/a | n/a | [C] / [R] |
| **UBTech Walker S2** (July 2025) | 11 (4th-gen hand) | Electric | n/a | Fingertip tactile | 15 kg robot load | n/a | [R] |
| **Xpeng Iron** (Nov 2025 → Sept 2026) | 22 (2025) → **21** (Sept 2026 production spec) | Electric, "extremely compact harmonic joints" [R] | Harmonic reducers [R] | n/a | n/a | n/a | [C] DOF |
| **Xiaomi CyberOne V2** (Apr 2026) | "22–27" | Electric, **motors in the forearm** | **Tendon ropes**, 150,000 grasp cycles | **Full-palm tactile, 8,200 mm²** | n/a | Human-male size | [C] via 36Kr |
| **Clone Robotics hand** | 27 | **Hydraulic artificial muscles** (Myofiber) in the forearm, 36 valves, 500 W pump | Muscles pull tendons | Pressure sensors (body: 320) | 6.8 kg grip [R] | < 0.9 kg | [C] / [R] |
| **AGILINK (Agibot spin-off) OmniHand 3 Ultra-M** (June 2026) | 20 active | Direct drive | Direct drive (a tendon "Ultra-T" also exists) | **Vision-based tactile fingertips**, ~0.005 N | 0–30 N fingertip range | 630 g | [C] |
| **Agibot OmniHand Pro 2025** | 19 (12 active) | Electric | Tendon [R] | 3-axis fingertip force, 150+ taxels | 20 N fingertip | 820 g | [R] |
| **Kepler Forerunner K2** (Oct 2024) | 11 (active + passive) | Electric | **"Rope-driven"** (tendon) | 96-point array per fingertip | 15 kg per hand | n/a | [C] |
| **1X NEO** (for comparison) | 22 hand + 3 wrist | "1X Motors" in the forearm | "1X Tendon Drive", polymer tendons, quasi-direct-drive | Tactile skin: normal, shear, slip | see 1x-neo.md | n/a | [C] |
| **Tendra V1** (ours, not built) | 20 | 20 × SCS0009 in the forearm | 40 strands, antagonistic loops, PTFE tubes | none yet (servo load only) | ~0.23 N·m per servo | n/a | — |

---

## 2. Per-company sections

### 2.1 Tesla Optimus

**Generations**

- **Gen 1 (AI Day, Sept 2022):** 6 actuators, 11 DOF, metallic tendons, an in-hand controller, and a **non-backdrivable** finger drive that holds objects with the motors off. [C] [notateslaapp reference](https://www.notateslaapp.com/tesla-reference/1000/everything-we-know-about-optimus-the-tesla-robot)
- **Gen 2 (13 Dec 2023):** "faster, 11-DoF brand-new hands" with tactile sensing on all fingers (the egg demo). [C] [notateslaapp](https://www.notateslaapp.com/news/1821/tesla-unveils-optimus-robot-gen-2-tesla-designed-fingertip-sensors-actuators-and-ten-other-improvements-video)
- **Upgraded hand (Nov 2024):** 22 DOF on the hand plus 3 on the wrist/forearm, tendons, **all actuators moved to the forearm**. The demo was teleoperated. Program head Milan Kovac listed the remaining work as "extended tactile sensing integration (much more surface coverage than the previous hand), very fine controls through tendons, and shaving some weight off the forearm", and named the trade-off between "enough squishiness/compliance and a protective layer on the fingers and palm, without affecting tactile sensing too much." [C] [Electrek](https://electrek.co/2024/11/29/tesla-unveils-upgraded-optimus-robot-hand-but-impressive-demo-is-again-teleoperated/)
- **V3 / Gen 3:** Musk posted a hand demo ("This bot got hands") on 17 Feb 2026 [R]. The full V3 robot had **not** been unveiled as of mid-September 2026. [R] [theroboticlife scorecard](https://theroboticlife.com/robot-of-the-week-tesla-optimus-gen-3/), [Mike Kalil](https://mikekalil.com/blog/tesla-optimus-gen-3-delayed-again/)

**V3 patents (the best technical window)**

WO2026080687A1, "Mechanically Actuated Robotic Hand", applicant Tesla Inc, priority 10 Oct 2024, published 16 Apr 2026. Companion filings: "Robotic Appendage" and "Joint Assembly for Robotic Appendage". [C] [Google Patents](https://patents.google.com/patent/WO2026080687A1/en)

- **Cables:** three per finger. One goes to the distal member (curls the upper two segments together, so these joints are **coupled**). Two go to the middle member and share the flexion and abduction/adduction work between them. [C] patent; [teslanorth](https://teslanorth.com/2026/04/16/teslas-robot-hand-patent-reveals-the-clever-engineering-inside-optimuss-fingers/)
- **Extension:** "a series of biasing members (e.g., springs)" return the finger to straight when the cables relax. So Tesla uses **pull cables + return springs**, not antagonistic cable loops like Tendra. [C] patent
- **Cable material and routing:** braided polymer cables in low-friction lined tubes that can be lubricated; grooves/channels in each finger segment keep the cables laterally aligned; the channels open at the joints so cables pass across them. [C] patent; teslanorth
- **Wrist "router":** cables run in a lateral (horizontal) stack in the forearm and a vertical stack in the hand, and switch over **exactly at the wrist joint**. This keeps each cable's moment arm about the pitch axis and the yaw axis small, which limits crosstalk (wrist motion accidentally moving fingers). [C] patent
- **Wrist actuation:** two linear actuators drive yaw and pitch through a universal joint. [C] patent
- **Actuator count:** 25 linear actuators per forearm (23 hand, 2 wrist), arranged in concentric rings around a central rotary actuator. [R] [droids.substack](https://droids.substack.com/p/the-forearm-is-the-new-hand-inside), [teslarati](https://www.teslarati.com/tesla-optimus-v3-hand-arm-details-revealed-new-patents/)
- **Linear actuator type:** ball screws per one blog, "planetary gearbox + planetary roller screw + tendon" per a China supply-chain tracker, "coreless motors + planetary gearboxes" per another blog. All three are **[R]/[S]**; the patent summary we read does not settle it. [basenor](https://www.basenor.com/blogs/news/tesla-optimus-gen-3-hands-22-dof-50-actuators-explained), [TechBuzz China](https://robotics.techbuzzchina.com/reports/robot-hands-china.html)
- **Rolling-contact joints:** the patent's finger joints have curved contact surfaces that roll on each other instead of turning on a pin. **But on 19 Apr 2026 Musk said: "We already changed the design. This one didn't actually work."** What replaced it is not public. [C] [teslarati](https://www.teslarati.com/elon-musk-reveals-shocking-tesla-optimus-patent-detail/), [notateslaapp](https://www.notateslaapp.com/news/4010/engineering-optimus-hands-and-knees)

**Stated reasons**

- In-hand actuators made "giant hands that look weird". [C] droids.substack quoting Musk
- Musk called the hand "the majority of the engineering difficulty of the entire robot", with no existing supply chain for precision tendon hardware. [C] teslarati, droids.substack

### 2.2 Figure (Figure 02, Figure 03, Helix)

- **Figure 02 (Aug 2024):** 16-DOF hands, robot payload 25 kg. [C] [Interesting Engineering](https://interestingengineering.com/innovation/figure-02-worlds-most-advanced-humanoid-robot). One source says each finger is a self-contained motor + sensor unit with wiring through a human-like wrist. [R] [Robotics & Automation News](https://roboticsandautomationnews.com/2025/05/06/spotlight-on-humanoids-a-deep-dive-into-figure-ai/90373/)
- **Figure 03 (Oct 2025):** [C] [Figure](https://www.figure.ai/news/introducing-figure-03)
  - In-house first-generation tactile sensor, designed for "extreme durability, long-term reliability, and high-fidelity sensing". Each fingertip "can detect forces as small as three grams of pressure".
  - "Softer, more adaptive fingertips increase surface contact area", giving more stable grasps on sheet metal and poly bags.
  - "Each hand now integrates an embedded palm camera with a wide field of view and low-latency sensing" so the robot still sees during grasps when the head cameras are blocked (e.g. inside a cabinet).
  - Hand DOF is **not** on Figure's page. Wikipedia says 20 per hand. [R] [Wikipedia](https://en.wikipedia.org/wiki/Figure_AI)
- **Helix 02 (27 Jan 2026):** with the fingertip tactile sensing and palm cameras, Figure shows unscrewing bottle caps, picking single pills from an organizer, dispensing precise syringe volumes, and picking small metal parts from clutter. A learned whole-body controller ("System 0") is trained on 1,000+ hours of human motion data and sim-to-real RL. [C] [Figure Helix 02](https://www.figure.ai/news/helix-02)

**Takeaway:** Figure's jump in dexterity came from **sensing** (touch + palm camera) and soft fingertips, not from more DOF.

### 2.3 Boston Dynamics electric Atlas

- **GR1 gripper:** three fingers in a row, no opposable thumb, built to survive falls onto the hand. [C] [Boston Dynamics LinkedIn](https://www.linkedin.com/posts/boston-dynamics_whats-in-a-humanoid-hand-boston-dynamics-activity-7381690601454452736-7Lbj)
- **GR2 gripper (2025):** three fingers including an opposable thumb, **7 DOF, 7 actuators** (two per finger, one extra to swivel the thumb), all in a self-contained, quick-attach module. Fingers bend inward up to 90° **and backward**; left and right are mirrored. [C] [heise](https://www.heise.de/en/news/Three-finger-gripper-Atlas-robot-grips-more-efficiently-with-rotating-thumb-10749451.html), [Boston Dynamics blog](https://bostondynamics.com/blog/ask-a-roboticist-meet-karl/)
- **Sensing:** tactile sensors in a high-friction elastomer fingertip, so the same rubber is both the grip surface and the force sensor; palm cameras. [C] same sources
- **Product Atlas (CES 2026):** 56 DOF robot, 50 kg max payload, "human-scale hands with tactile sensing", three-fingered, field-replaceable limbs. [C] [Hyundai](https://www.hyundainews.com/releases/4664). One outlet says the CES hand is "a significantly more traditional three-fingered hand with an opposable thumb" because not using human tools was a commercial blocker. [R] [RoboHorizon](https://robohorizon.com/en-us/news/2026/01/boston-dynamics-atlas-got-new-hands-and-we-have-so-many-questions/)
- **Why three fingers:** "three fingers including an opposable thumb gives us a lot of flexibility to manipulate different objects while minimizing points of failure, but the gripper could definitely look different in the future." More fingers add complexity, cost, failure points and development time without enough gain for car-factory tasks, which need "finesse, but also high force output". [C] Boston Dynamics blog, LinkedIn
- Grip force, weight and transmission are not published.

### 2.4 Sanctuary AI Phoenix

- **21-DOF hydraulic hand** with finger abduction and in-hand manipulation. [C] [Sanctuary](https://sanctuary.ai/news/sanctuary-ai-controlling-advanced-hydraulic-hands/)
- **Why hydraulics:** "an order of magnitude higher power density than cable and electromechanical-based systems", plus speed, strength, controllability, cycle life, impact resistance and heat handling. Miniature valves were tested for **over 2 billion cycles** "without any signs of leakage or degradation". [C] [Automation Magazine, Dec 2024](https://www.automationmag.com/sanctuary-ais-hydraulic-actuation-brings-human-level-dexterity-to-general-purpose-robots/)
- **Tactile:** a new tactile sensor generation (2025), described as close to human fingertip sensitivity; built on **micro-barometers** (the same pressure-sensor family as in phones), 7 cells per finger pad. [C] that it exists; [R] the barometer and 7-cell detail [Inspenet](https://inspenet.com/en/news/sanctuary-ai-advances-with-21-degree-of-freedom-hydraulic-hands/)
- **Learning:** an in-hand reorientation policy trained in simulation with RL transferred to the real hand and handled a 500 g load not seen in training. [C] Sanctuary
- **Phoenix Gen 8:** wheeled base (customers said legs were too frail for a strong torso), lower bill of materials. [C] [Sanctuary Gen 8](https://sanctuary.ai/news/sanctuary-ai-releases-new-generation-of-ai-robots-for-high-quality-data-capture/)

### 2.5 Apptronik Apollo 2 and Google DeepMind

- **Apollo 2** was unveiled at the end of June 2026 (bipedal and wheeled versions), mainly as a data-collection platform ("Robot Park"). Apptronik has not published hand or wrist specs. [C] [Humanoids Daily](https://www.humanoidsdaily.com/news/substance-over-hype-apptronik-quietly-unveils-apollo-2-humanoid-via-website-update), [The Robot Report](https://www.therobotreport.com/apptronik-unveils-apollo-2-flagship-data-collection-training-facility/)
- **Gemini Robotics 2 (30 July 2026):** DeepMind ran it on Apollo 2 with **SharpaWave** hands (22 DOF, five fingers) and Inspire hands. Success rates on SharpaWave: unscrew bulb 92%, screw bulb 36%, tie trash bag 44%, dustpan 32%, seal ziplock 40%. [C] [DeepMind](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/)
  - These modest numbers are a useful reality check: even a 22-DOF, heavily sensed hand with a frontier model fails most deformable-object tasks today.
- **SharpaWave (Sharpa, Singapore):** 22 active DOF, 1.3 kg, 208 × 90 × 50 mm, 20 N max fingertip force, >4 Hz motion across gestures, 500 Hz control, Gigabit Ethernet, each finger with a tactile sensor ("Dynamic Tactile Array", 1,000+ taxels per fingertip, ~0.005 N) and a torque sensor. About **$50,000** per a third-party interview. [C] [CNX Software](https://www.cnx-software.com/2026/06/02/sharpa-wave-high-end-dexterous-robotic-hand-with-22-dof-high-sensitivity-dynamic-tactile-array/); price [R]
- Apptronik makes the body and buys the hands: a sign that hands are becoming a **separate product category**.

### 2.6 Agility Digit

- Digit (ProMat 2025) got a **swappable gripper with ISO-standard mounting flanges**; it currently uses a two-fingered gripper, and the flange can take a paddle, pincher, claw or five-fingered hand. [C] [Business Wire](https://www.businesswire.com/news/home/20250331588956/en/Agility-Robotics-Announces-New-Innovations-for-Market-Leading-Humanoid-Robot-Digit), [The Robot Report](https://www.therobotreport.com/agility-robotics-announces-latest-advances-digit-humanoid-robot/)
- **Digit 5 (15 Sept 2026):** lifts up to 50 lb (22.7 kg) repeatedly; keeps swappable end effectors. [C] [Agility](https://www.agilityrobotics.com/content/agility-unveils-digit-5-humanoid-robot-built-for-cooperatively-safe-work-at-scale)
- **Reason:** Agility's view is that no hand on the market can pick up 25 kg bins and still manipulate them, so for tote work a simple, strong tool wins. [C] (quoted in the Robotics 24/7 / Robot Report coverage)

### 2.7 Unitree (Dex3-1, Dex5-1, Dex5-S)

- **Dex3-1** (G1 hand): 3 fingers (thumb 3 DOF, index 2, middle 2) = 7 DOF; 6 direct-drive + 1 geared micro brushless joints; 710 g; 175 × 88 × 77 mm; max object 500 g; up to 3.1 N·m on the geared joint; 33 pressure sensors (10 g–2.5 kg); ±2 mm repeatability; USB 2.0 at 1 kHz. [C] [Unitree Dex3-1](https://www.unitree.com/mobile/Dex3-1/)
- **Dex5-1** (H1/H2 hand): 20 DOF (16 active + 4 passive: thumb 4 active, each finger 3 active + coupled), coreless ("hollow-cup") motors with high-precision encoders and a low-backlash reducer, 12 composite force-controlled joints + 4 geared joints, 1.1 kg, 217 × 128 × 72 mm, 10 N fingertip, 3.5 kg payload palm-down / 4.5 kg side-grasp of a 5 cm cylinder, ±1 mm repeatability, four-finger sideways swing ±22°, min object 10 mm, 94 tactile sensors (Dex5-1P), 24–60 V. [C] [Unitree Dex5-1](https://www.unitree.com/mobile/Dex5-1/)
- **Dex5-S (21 Sept 2026):** exact human-hand size, **22 active DOF, every joint motor-driven and backdrivable**, impact-torque limiting on every joint, dual encoders. China price about RMB 39,900 (~US$6,500); peak payload ~2 kg. [C] DOF/backdrivable [Robotlar](https://www.robotlar.org/en/unitree-dex5-s-dexterous-hand); [R] price/payload [Pandaily](https://pandaily.com/unitree-dex5-s-22-dof-dexterous-hand-39900-rmb)
- Unitree keeps motors **in the hand** (direct drive / small reducers) and pays for it with a heavy (1.1 kg), bulky hand and low fingertip force (10 N). Backdrivability and current-based force control are its main selling points.

### 2.8 Fourier (GR-3, FDH hands)

- **GR-3 (Aug 2025):** care-focused humanoid, 165 cm, 71 kg, up to 55 DOF, **12-DOF dexterous hands**, soft upholstered shell with 31 pressure sensors for safe touch. [C] [PR Newswire](https://www.prnewswire.com/news-releases/meet-gr-3-beyond-function-designed-to-care-fourier-to-unveil-its-first-care-centric-humanoid-302523202.html)
- **FDH-6:** 6 DOF, 11 joints, silicone pads, fingertip repeatability 0.5 mm. [R] [Fourier FDH-6](https://www.fftai.com/products-fdh6) (page did not render for us), [Gasgoo](https://autonews.gasgoo.com/articles/market-industry/can-a-500-yuan-dexterous-hand-grasp-the-industrys-lifeline-2017491844613427201). Another source gives 16 DOF and 400 tactile points at RMB 4,999, which looks like a mix-up with Agibot's OmniHand, so treat it as unreliable.
- Fourier has not shown a 20+ DOF hand. Its choice fits a care robot: gentle, cheap, few DOF.

### 2.9 UBTech Walker S2

- Unveiled 23 July 2025; 176 cm, 73 kg, 52 DOF, autonomous battery hot-swap. [R] [Humanoid.guide](https://humanoid.guide/product/walker-s2/)
- **4th-generation 11-DOF hands** with fingertip tactile sensors, sub-millimetre precision, 80,000+ cycle durability claim; up to 15 kg load. [R] same + [RoboZaps](https://blog.robozaps.com/b/ubtech-walker-s-review)
- Transmission and actuator placement are not published. Deployed in car and electronics factories (BYD, Geely, Foxconn and others). [R]

### 2.10 Xpeng Iron

- **Nov 2025 (AI Day):** "bone–muscle–skin" design, flexible spine, soft skin, **22-DOF hands**. [C] [CnEVPost](https://cnevpost.com/2025/11/05/xpeng-unveils-next-gen-iron-humanoid-robot/)
- **8 Sept 2026 (production line opened):** now **21 DOF per hand** (76 DOF total, down from 82); mass production by end of 2026, deliveries 2027. [C] [CnEVPost](https://cnevpost.com/2026/09/08/xpeng-opens-iron-humanoid-robot-production-line/). The reason for dropping one DOF is not given.
- Hands built around "extremely compact harmonic joints". [R] [Humanoids Daily](https://www.humanoidsdaily.com/robots/xpeng-iron). A claim that the 22-DOF hands "wore out within a month" in internal testing appears only on a review blog. [S] [Anton Robots](https://antonrobots.com/xpeng-iron-review/)

### 2.11 Xiaomi (CyberOne V2 hand)

Shown at Xiaomi's investor conference, late April 2026. [C] via [36Kr](https://eu.36kr.com/en/p/3789078876151044)

- **22–27 DOF** (active DOF up 64–83% vs the previous hand, depending on the source).
- **Motors in the forearm, tendon ropes** to the fingers. Hand volume cut by 60%, now the size of an adult male hand.
- **Durability:** 150,000 real grasp cycles; the article says typical tendon hands fail around 10,000 cycles.
- **Heat:** one forearm's motors draw >100 W, ~30 W of it waste heat. A metal-3D-printed micro liquid-cooling channel "sweats" ~0.5 mL of water per minute for ~10 W of cooling.
- **Full-palm tactile skin, 8,200 mm²**, so data from human demonstrations can include touch.

### 2.12 Clone Robotics

- **Clone Hand:** 27 DOF, **hydraulic Myofiber artificial muscles** (a mesh tube around a balloon: pressurise it and the tube gets fatter and shorter, like a muscle), antagonistic muscle pairs, 36 electro-hydraulic valves, 500 W water pump. [C] [Clone](https://clonerobotics.com/hand/), [Interesting Engineering](https://interestingengineering.com/ai-robotics/clone-demos-creepy-humanoid-hand)
- **Myofiber claims:** contracts >30% in <50 ms; a 3 g fiber pulls >1 kg; 650,000+ cycles without fatigue. [C] company claims
- Hand weight under 0.9 kg; grip 6.8 kg. [R] [sudoremove](https://sudoremove.com/en/knowledge/hardware/hands/clone/)
- Nov 2025: glove-teleoperated hand demo and a "Neural Joint V2" controller trained on human hand videos. [C] Clone on X
- Most anatomical approach (bones, ligaments, muscles in the forearm), but far from a product.

### 2.13 Agibot / AGILINK

- Agibot spun its hand business off (AGILINK), which says it has shipped **8,000+ hands**. [C] [Pandaily](https://pandaily.com/agilink-launches-omni-hand-3-series-ships-over-8-000-dexterous-hands)
- **OmniHand 3** (standard): 10 active + 6 passive DOF, 510 g, 5 kg payload, 0.5 s open/close, ±0.3 mm, 300+ taxels on fingertips, palm **and back of the hand**. [C] [AGILINK](https://www.agilink-ai.com/omni-hand3.html)
- **OmniHand 3 Ultra-M** (ICRA, June 2026): 20 active DOF, **direct drive**, 630 g, **vision-based tactile fingertips** (a camera films a soft skin from inside) with ~0.005 N resolution, 0–30 N range, and 3D tactile palm; listed at ~US$16,000. A tendon-driven "Ultra-T" variant exists. A quick-release tendon system allows part swaps in 10 minutes [R]. [C] [Humanoids Daily](https://www.humanoidsdaily.com/news/the-contact-problem-how-agilink-s-new-omnihand-3-ultra-m-is-teaching-robots-to-feel); price [R] [Humanoid.guide](https://humanoid.guide/welcome-omnihand-3-ultra-m/)
- **OmniHand Pro 2025:** 19 DOF (12 active), 820 g, 20 N fingertip, 3-axis fingertip force, 150+ taxels, CAN-FD 1 kHz. [R] [OpenELAB](https://openelab.io/products/agibot-omnihand-pro-2025-hand)

### 2.14 Kepler Forerunner K2

- Five-finger **"rope-driven" (tendon) hands**, 11 DOF (active + passive), 96-point tactile array per fingertip, 15 kg per hand. The body uses planetary roller-screw actuators. Launched Oct 2024 (Gitex). [C] [New Atlas](https://newatlas.com/ai-humanoids/kepler-forerunner-k2-humanoid-robot/)

### 2.15 Hand makers behind the humanoids (context)

- China now has specialist hand vendors on three technical routes: **screw + linkage**, **tendon-driven**, and **high-perception** hands. LinkerBot (tendon, O6 at RMB 6,666 ≈ US$920) claims 80%+ share of high-DOF hands and 1,000+ per month; Inspire sells both linkage and tendon models; Zhaowei and others do linkage. [R] [TechBuzz China](https://robotics.techbuzzchina.com/reports/robot-hands-china.html)
- Chinese hands now range ~US$1,400 to US$15,000+; the 2024 global average was ~US$7,960. [R] [Sourcebotics](https://sourcebotics.com/guides/chinese-dexterous-hands-2026-buyers-guide/)

---

## 3. Trends

1. **Actuators are moving into the forearm.** [C] Tesla (all 25 per forearm), Xiaomi, 1X, Clone and Kepler (tendons) put motors or muscles in the forearm. Tesla's stated reason: in-hand motors made "giant hands". Xiaomi cut hand volume by 60% this way. The main holdout is **direct drive in the hand** (Unitree Dex5, AGILINK Ultra-M, Boston Dynamics' module), which wins on simplicity, backdrivability and swap-ability, and loses on size and fingertip force. [S] Expect both to survive: forearm tendons for human-like hands, in-hand modules for rugged or swappable grippers.

2. **Tendon vs linkage vs direct drive.** [C]/[R]
   - Tendon: Tesla, 1X, Xiaomi, Kepler, LinkerBot, Agibot Pro/Ultra-T. Slim fingers, light hand; problems are friction, stretch, creep, crosstalk and **durability** (Xiaomi's "typical failure at ~10,000 cycles" vs its 150,000).
   - Linkage / screw: many Chinese low-cost hands (Inspire, Zhaowei). Stiff, precise, cheap, but usually 6–12 active DOF with coupled joints.
   - Direct drive / small reducers in the joints: Unitree, AGILINK Ultra-M, Xpeng (harmonic joints [R]).
   - Hydraulic: Sanctuary (valves) and Clone (muscles). Highest power density, hardest plumbing.

3. **Rolling-contact joints: tried and (at least once) dropped.** Tesla patented rolling-contact knuckles, then Musk said the design "didn't actually work" and was changed. [C] Nobody else in this list advertises them. [S] For a printed hobby hand, plain pin joints stay the safe choice.

4. **Tactile everywhere, and it is now the main differentiator.** [C] Figure 03 (3 g fingertips), Atlas (elastomer fingertip sensors), Sanctuary (micro-barometer arrays), Sharpa (1,000+ taxels/fingertip), AGILINK (vision-based fingertips, palm, back of hand), Unitree (33–94 sensors), Kepler (96/fingertip), Xiaomi (full palm), 1X (normal + shear + slip). Figure's and Helix 02's best new skills are credited to touch and palm cameras, not to more DOF.

5. **Palm cameras.** [C] Figure 03 and Atlas both put cameras in the palm to see the grasp when the head cameras are blocked.

6. **Soft, compliant fingertips.** [C] Figure 03 (bigger, softer fingertips), Atlas (high-friction elastomer), Tesla (protective "squishy" layer that must not kill touch).

7. **DOF converging on ~20–22 for the flagship hands.** [C] Tesla 22, SharpaWave 22, Unitree Dex5-S 22, 1X 22, Sanctuary 21, Xpeng 22 → 21, AGILINK 20, Unitree Dex5-1 20 (16 active), Xiaomi 22–27. Industrial robots stay much lower: Atlas 7, UBTech 11, Kepler 11, Fourier 6–12, Digit a gripper. Coupling the DIP to the PIP (Tesla's one distal cable curls two segments; Unitree's 4 passive joints) is common.

8. **Wrist DOF: 2–3.** [C] Tesla's patent: 2-DOF wrist driven by two linear actuators, with the tendons crossing the wrist centre. 1X: 3 wrist DOF. Tesla's routing trick (switching the cable stack exactly at the wrist axes) is the known fix for wrist–finger crosstalk.

9. **Durability is now advertised.** [C] Sanctuary 2 billion valve cycles, Clone 650,000 muscle cycles, Xiaomi 150,000 grasps, UBTech 80,000+ [R]. Quick-change tendons (AGILINK, 10 min [R]) and swappable end effectors (Digit ISO flange, Atlas quick-attach, field-replaceable limbs) show the same concern: hands break, so make them easy to fix.

10. **Hands are becoming a separate product.** [C] Apptronik uses Sharpa and Inspire hands; Agibot spun off AGILINK; Unitree sells hands alone; LinkerBot ships 1,000+/month [R]. Prices are falling fast in China (sub-US$1,000 to ~US$16,000; SharpaWave ~US$50,000 at the top).

11. **Learning drives the hardware.** [C] Sanctuary (sim-to-real RL for in-hand rotation), Figure (Helix 02, 1,000+ h of human data), DeepMind (Gemini Robotics 2 on SharpaWave), Clone (models trained on human hand videos), Xiaomi (palm tactile so demos carry touch data). Human-size, human-DOF hands make it easier to learn from human video and teleoperation.

---

## 4. What Tendra can learn / implement

Tendra V1 already matches the main trend: 20 DOF, motors in the forearm, tendons through PTFE tubes. The ideas below are ordered by value per effort for a hobby budget. All are **[S]** suggestions based on the findings above.

### Priority 1: cheap, high value, fits the current plan

1. **Keep pin joints; don't chase rolling-contact knuckles.** Tesla patented them and then dropped them. Tendra's pin joints with drums are the right call.
2. **Validate the PTFE-tube routing, and add lubrication.** Tesla's patent uses braided polymer cables in low-friction lined tubes "that can be lubricated", which is the same idea as Tendra's 1 × 2 mm PTFE tubes. Try a dry PTFE or silicone lubricant on the V0 tendons and measure friction (servo load at a fixed pose) before and after.
3. **Switch to braided polymer tendon (UHMWPE/Dyneema braid, ~0.3–0.4 mm).** Tesla (braided polymer) and 1X (polymer) both avoid monofilament. Braid stretches and creeps less than nylon fishing line, which matters for open-loop accuracy. Cost: a few euros.
4. **Soft, high-friction fingertip pads.** Figure 03 and Atlas both credit soft, grippy fingertips. The TPU pads already planned are right; try a softer silicone skin (e.g. Shore 10–20A cast into a printed mould) and compare grasp success in the sim and on V0.
5. **Run a cycle-life test on V0.** Everyone now advertises cycle counts, and Xiaomi says typical tendon hands fail around 10,000 cycles. Script the V0 index to flex and extend thousands of times and log drift and servo load, then note where it wears (tendon at the drum hole? slot edges?). This finds the weak spots before V1 is printed.
6. **Plan a quick tendon change.** AGILINK claims 10-minute tendon swaps. For V1, design each strand's anchor (drum hole and spool) so one tendon can be replaced without taking the forearm apart, e.g. a knot pocket plus a screw-clamp on the spool.

### Priority 2: sensing, where the leaders are pulling ahead

7. **Barometer fingertips (Sanctuary-style).** Sanctuary's fingertip arrays use micro-barometers from the phone sensor family. The hobby version is well known: a BMP280/BMP390/LPS22 chip under a few mm of cast silicone, read over I²C (the ESP32-S3 has spare I²C). Start with one sensor in the V0 index tip, then five for V1. About €2–5 per fingertip.
8. **A palm camera (Figure 03 / Atlas-style).** A small wide-angle camera in the palm sees the grasp when the main camera can't. Cheap route: an OV2640/OV5640 module on a separate ESP32-S3-CAM streaming to the PC, or a USB endoscope camera. It also feeds the grasp RL and imitation-learning datasets directly.
9. **Use servo load as a crude force sense now.** Unitree and 1X lean on motor current for force control. The SCS0009 "present load" register is coarse, but already enough to detect contact and to limit squeeze force. Log it in the `F` feedback stream and add it to the sim observation.
10. **Put touch into the sim and RL.** MuJoCo touch sensors (sites on the fingertip geoms) cost almost nothing to add to `tendra_hand_v1.xml`. Train with them so the policy is ready when the real barometer tips arrive.

### Priority 3: design decisions for V1.x and the bimanual station

11. **Consider coupling DIP to PIP (20 → 16 servos).** Tesla (one cable curls two segments) and Unitree Dex5-1 (4 passive joints) both couple the last joint. For V1 this would save 4 servos, 8 strands and a lot of forearm space and routing, at a small dexterity cost. Test it in the sim first: couple `dip` to `pip` with a MuJoCo equality or tendon and check whether the grasp RL success rate drops.
12. **Wrist: route strands through the wrist axes.** When Tendra gets a wrist, copy Tesla's rule: cross the wrist exactly at the pitch and yaw axes (Tendra already does this at the finger joints). `tendon_router.py` can enforce it as a test, as it already does for the fingers. Plan 2 wrist DOF for the bimanual station (Tesla 2, 1X 3).
13. **Keep antagonistic loops (don't copy Tesla's springs).** Tesla returns fingers with springs; Tendra's antagonistic loop gives active extension and no spring force to fight, which suits weak SCS0009 servos. The cost is pretension, so keep it low (SCS0009 backs off above 80% load for 2 s).
14. **Watch heat, not liquid cooling.** Xiaomi needed liquid cooling at >100 W per forearm. Twenty SCS0009s are far below that, but they sit packed together in the forearm: leave air gaps and vent slots, and log servo temperature in the `F` feedback.
15. **Treat the hand as a product and a module.** The market is splitting into body makers and hand makers (Sharpa, AGILINK, LinkerBot). An open, cheap, well-documented 20-DOF tendon hand with a standard wrist flange (ISO-style, like Digit) and a sim model is a niche the commercial players don't cover. That's worth keeping in mind for the entrepreneurial side.

---

## 5. Open questions

- Tesla V3: what replaced the rolling-contact joint, the real actuator type (ball screw vs roller screw vs geared motor), hand weight and grip force. Wait for the V3 unveiling.
- Figure 03: official hand DOF and where the hand motors sit.
- Atlas product hand: DOF, whether it differs from GR2, grip force.
- Sanctuary: where the hydraulic valves sit (in the hand or forearm) and the pump location.
- Xpeng: why it went from 22 to 21 DOF.
- Worth checking later: Google Patents searches for assignees Figure AI, Boston Dynamics, Sanctuary AI, Xiaomi, with keywords *tendon*, *finger*, *tactile*.

---

## 6. Sources

Tesla
- WO2026080687A1 "Mechanically Actuated Robotic Hand": https://patents.google.com/patent/WO2026080687A1/en
- droids.substack, "The Forearm Is the New Hand": https://droids.substack.com/p/the-forearm-is-the-new-hand-inside
- Teslarati, V3 patents: https://www.teslarati.com/tesla-optimus-v3-hand-arm-details-revealed-new-patents/
- Teslarati, Musk on the rolling joint: https://www.teslarati.com/elon-musk-reveals-shocking-tesla-optimus-patent-detail/
- Not a Tesla App, patents: https://www.notateslaapp.com/news/4010/engineering-optimus-hands-and-knees
- TeslaNorth, patent details: https://teslanorth.com/2026/04/16/teslas-robot-hand-patent-reveals-the-clever-engineering-inside-optimuss-fingers/
- Electrek, Nov 2024 hand: https://electrek.co/2024/11/29/tesla-unveils-upgraded-optimus-robot-hand-but-impressive-demo-is-again-teleoperated/
- Not a Tesla App, Gen 2: https://www.notateslaapp.com/news/1821/tesla-unveils-optimus-robot-gen-2-tesla-designed-fingertip-sensors-actuators-and-ten-other-improvements-video
- Not a Tesla App, reference (Gen 1): https://www.notateslaapp.com/tesla-reference/1000/everything-we-know-about-optimus-the-tesla-robot
- The Robotic Life, Sept 2026 scorecard: https://theroboticlife.com/robot-of-the-week-tesla-optimus-gen-3/
- Mike Kalil, V3 delay: https://mikekalil.com/blog/tesla-optimus-gen-3-delayed-again/
- Basenor (supply-chain claims): https://www.basenor.com/blogs/news/tesla-optimus-gen-3-hands-22-dof-50-actuators-explained

Figure
- Introducing Figure 03: https://www.figure.ai/news/introducing-figure-03
- Helix 02: https://www.figure.ai/news/helix-02
- Interesting Engineering, Figure 02: https://interestingengineering.com/innovation/figure-02-worlds-most-advanced-humanoid-robot
- Robotics & Automation News, Figure deep dive: https://roboticsandautomationnews.com/2025/05/06/spotlight-on-humanoids-a-deep-dive-into-figure-ai/90373/
- Wikipedia, Figure AI: https://en.wikipedia.org/wiki/Figure_AI

Boston Dynamics
- Ask a Roboticist, Karl: https://bostondynamics.com/blog/ask-a-roboticist-meet-karl/
- LinkedIn, gripper evolution: https://www.linkedin.com/posts/boston-dynamics_whats-in-a-humanoid-hand-boston-dynamics-activity-7381690601454452736-7Lbj
- heise: https://www.heise.de/en/news/Three-finger-gripper-Atlas-robot-grips-more-efficiently-with-rotating-thumb-10749451.html
- Hyundai, CES 2026: https://www.hyundainews.com/releases/4664
- RoboHorizon, CES hands: https://robohorizon.com/en-us/news/2026/01/boston-dynamics-atlas-got-new-hands-and-we-have-so-many-questions/

Sanctuary AI
- RL on hydraulic hands: https://sanctuary.ai/news/sanctuary-ai-controlling-advanced-hydraulic-hands/
- Gen 8: https://sanctuary.ai/news/sanctuary-ai-releases-new-generation-of-ai-robots-for-high-quality-data-capture/
- Automation Magazine: https://www.automationmag.com/sanctuary-ais-hydraulic-actuation-brings-human-level-dexterity-to-general-purpose-robots/
- Inspenet: https://inspenet.com/en/news/sanctuary-ai-advances-with-21-degree-of-freedom-hydraulic-hands/
- Mike Kalil: https://mikekalil.com/blog/sanctuary-ai-phoenix-hand/

Apptronik / DeepMind / Sharpa
- DeepMind, Gemini Robotics 2: https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/
- Humanoids Daily, Apollo 2: https://www.humanoidsdaily.com/news/substance-over-hype-apptronik-quietly-unveils-apollo-2-humanoid-via-website-update
- The Robot Report, Apollo 2: https://www.therobotreport.com/apptronik-unveils-apollo-2-flagship-data-collection-training-facility/
- CNX Software, Sharpa Wave: https://www.cnx-software.com/2026/06/02/sharpa-wave-high-end-dexterous-robotic-hand-with-22-dof-high-sensitivity-dynamic-tactile-array/

Agility
- Digit 5: https://www.agilityrobotics.com/content/agility-unveils-digit-5-humanoid-robot-built-for-cooperatively-safe-work-at-scale
- ProMat 2025: https://www.businesswire.com/news/home/20250331588956/en/Agility-Robotics-Announces-New-Innovations-for-Market-Leading-Humanoid-Robot-Digit
- The Robot Report: https://www.therobotreport.com/agility-robotics-announces-latest-advances-digit-humanoid-robot/

Unitree
- Dex3-1: https://www.unitree.com/mobile/Dex3-1/
- Dex5-1: https://www.unitree.com/mobile/Dex5-1/
- Dex5-S (Robotlar): https://www.robotlar.org/en/unitree-dex5-s-dexterous-hand
- Dex5-S (Pandaily): https://pandaily.com/unitree-dex5-s-22-dof-dexterous-hand-39900-rmb

Fourier, UBTech, Xpeng, Xiaomi
- Fourier GR-3: https://www.prnewswire.com/news-releases/meet-gr-3-beyond-function-designed-to-care-fourier-to-unveil-its-first-care-centric-humanoid-302523202.html
- Fourier FDH-6: https://www.fftai.com/products-fdh6
- Gasgoo, low-cost hands: https://autonews.gasgoo.com/articles/market-industry/can-a-500-yuan-dexterous-hand-grasp-the-industrys-lifeline-2017491844613427201
- Humanoid.guide, Walker S2: https://humanoid.guide/product/walker-s2/
- RoboZaps, Walker S2: https://blog.robozaps.com/b/ubtech-walker-s-review
- CnEVPost, Iron Nov 2025: https://cnevpost.com/2025/11/05/xpeng-unveils-next-gen-iron-humanoid-robot/
- CnEVPost, Iron production line Sept 2026: https://cnevpost.com/2026/09/08/xpeng-opens-iron-humanoid-robot-production-line/
- Humanoids Daily, Iron: https://www.humanoidsdaily.com/robots/xpeng-iron
- Anton Robots, Iron review: https://antonrobots.com/xpeng-iron-review/
- 36Kr, Xiaomi hand: https://eu.36kr.com/en/p/3789078876151044

Clone, Agibot/AGILINK, Kepler
- Clone Hand: https://clonerobotics.com/hand/
- Interesting Engineering, Clone hand: https://interestingengineering.com/ai-robotics/clone-demos-creepy-humanoid-hand
- sudoremove, Clone hand: https://sudoremove.com/en/knowledge/hardware/hands/clone/
- AGILINK OmniHand 3: https://www.agilink-ai.com/omni-hand3.html
- Humanoids Daily, OmniHand 3 Ultra-M: https://www.humanoidsdaily.com/news/the-contact-problem-how-agilink-s-new-omnihand-3-ultra-m-is-teaching-robots-to-feel
- Humanoid.guide, Ultra-M: https://humanoid.guide/welcome-omnihand-3-ultra-m/
- Pandaily, AGILINK: https://pandaily.com/agilink-launches-omni-hand-3-series-ships-over-8-000-dexterous-hands
- OpenELAB, OmniHand Pro 2025: https://openelab.io/products/agibot-omnihand-pro-2025-hand
- New Atlas, Kepler K2: https://newatlas.com/ai-humanoids/kepler-forerunner-k2-humanoid-robot/

Industry
- TechBuzz China, robot hands in China: https://robotics.techbuzzchina.com/reports/robot-hands-china.html
- Sourcebotics, Chinese hands 2026: https://sourcebotics.com/guides/chinese-dexterous-hands-2026-buyers-guide/
- 1X NEO: see [1x-neo.md](1x-neo.md)
