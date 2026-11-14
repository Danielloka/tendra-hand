# Tendra Skin — a slim magnetic data glove (2026-11-14)

**Goal (owner):** a cheap, slim, accurate glove to record human demonstrations for Tendra's imitation learning, worn while doing everyday tasks, together with a depth camera. Two gloves (left + right). Budget ≈ 4,000 NOK including cameras.

## Summary and recommendation

- **Sensing:** a small N52 magnet (Ø4 × 2 mm, standing, pointing along the bone) on one finger segment and a 3D Hall sensor (TI TMAG5273) on the next. The field direction gives the joint angle; the knuckle sensor gives **bend and spread from one chip**. All parts sit on the **back** of the hand; palm and fingertip pads stay bare (open-palm glove, magnets for the DIP / thumb IP on press-on nails).
- **Per glove:** 15 Hall sensors (14 joints + 1 reference that detects steel/stray fields), 14 magnets, 2 IMUs (thumb base cmc_rot/cmc_flex from the relative orientation of a thumb-metacarpal IMU and a hand-plate IMU). Maps onto Tendra V1's 16 servo DOF; the human 5-DOF thumb is retargeted by thumb-tip position (`ThumbSolver`).
- **Electronics:** ESP32-S3-MINI-1 + 500 mAh LiPo in a wrist pod, ESP-NOW to one USB dongle for both gloves, ~3.3 h per charge, 200 Hz (100 Hz fallback). Four small PCBs, all assembled by JLCPCB (no hand-soldering of tiny chips).
- **Expected accuracy** (estimate, not measured): 3–4° per finger joint, 5–7° thumb base, fingertip 6–9 mm; ring slip on skin is the largest error, corrected by self-calibration against camera hand tracking.
- **Cost:** ≈ 2,360 NOK for two gloves (estimate). That leaves too little for a RealSense D405 inside 4,000 NOK: start with cheap wrist cameras.
- **Not built or tested.** Next step: a one-finger bench test (~250 NOK) for slip, steel interference, crosstalk and accuracy.

Owner decisions so far: slim cantilever tabs OK, open-palm glove, standing magnets, battery + wireless in a wrist pod, no haptic buzzers, two gloves.

## Contents

| Path | What |
|---|---|
| [`design.md`](design.md) | Electronics design package rev A (draft): BOM, power, pinout, I2C tree, harness, PCB rules, netlist, firmware, open items, bring-up |
| [`concept/tendra-skin-glove-3d.html`](concept/tendra-skin-glove-3d.html) | Interactive 3D concept (v0.3): every part at real size, live dipole field at each sensor; open in a browser (loads three.js from jsDelivr) |
| [`kicad/`](kicad/README.md) | KiCad 9 project. GW-POD (wrist pod) schematic rev A0: ERC 0/0, netlist verified 231/231 pins. Other 3 boards not started |
| [`firmware/`](firmware/) | PlatformIO: `glove` (ESP32-S3) and `dongle` (XIAO ESP32-S3). Both build; 40/40 host tests; PC reader `tools/glove_reader.py`. Not run on hardware |
| [`reports/`](reports/) | One report per agent |

## Agent team

| Report | Agent (model) | Job |
|---|---|---|
| [`01-sensing.md`](reports/01-sensing.md) | research (Sonnet) | Finger-angle sensing technologies |
| [`02-prior-art.md`](reports/02-prior-art.md) | research (Sonnet) | Robot-learning gloves and lessons |
| [`03-sourcing.md`](reports/03-sourcing.md) | research (Sonnet) | Parts, prices, Norway sourcing |
| [`04-architecture.md`](reports/04-architecture.md) | lead designer (Opus) | Concepts, chosen design, accuracy maths, calibration, pipeline |
| [`05-bom-verification.md`](reports/05-bom-verification.md) | component check (Sonnet) | MPNs, LCSC numbers, datasheet facts |
| [`06-firmware.md`](reports/06-firmware.md) | firmware (Opus) | Glove + dongle firmware, tests |

The main session (Opus) wrote the electronics architecture, power/PCB rules, netlist and the KiCad schematic, and checked the datasheets itself where agents could not read the PDFs.

## Open questions for the owner

1. Build order: one-finger test → full right glove → copy for the left?
2. Measure finger-bone widths so the rings fit.
3. Cheap wrist cameras first, D405 later?
4. Next KiCad board: GW-PLATE (I2C switch, 4 knuckle sensors, hand IMU).
