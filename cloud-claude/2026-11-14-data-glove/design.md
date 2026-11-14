# Tendra Glove — electronics design package (rev A draft)

Scope: one glove (right; the left is the mirror and uses the same electronics with `SIDE_SEL` jumpered). Two gloves + one USB dongle make a set. The glove has **no actuators**: no motor drivers, no high-current paths. The design problems are a clean low-noise sensor bus over moving cables, a safe 1S LiPo power path, a 2.4 GHz link next to a body, and keeping ferromagnetic parts away from the magnetic sensors.

Status words used below: **verified** = checked against a datasheet or distributor page by the BOM agent; **VERIFY** = must be checked before ordering.

---

## 0. System overview

```
 finger nodes (6x8 mm)      hand plate PCB (GW-PLATE)              wrist pod PCB (GW-POD)           PC
 ┌──────┐   ┌──────┐   4w   ┌──────────────────────────┐   6w     ┌──────────────────────────┐   ESP-NOW   ┌──────────────┐  USB  ┌────┐
 │ PIP  │──▶│ DIP  │◀──────▶│ TCA9548A 8-ch I2C switch │◀────────▶│ ESP32-S3-MINI-1-N8       │ ~~~~~~~~~~▶ │ dongle       │──────▶│ PC │
 │TMAG  │   │TMAG  │  ×4    │ 4× TMAG5273 (MCP)        │  JST-SH  │ BQ24074 charger/powerpath│  2.4 GHz    │ XIAO ESP32-S3│       └────┘
 │ A1   │   │ B1   │ fingers│ ICM-42688-P (hand IMU)   │          │ TPS63802 3.3 V buck-boost│             └──────────────┘
 └──────┘   └──────┘        │ TMAG5273 reference       │          │ MAX17048 fuel gauge      │
 thumb: IMU board → A1 → B1 │ ESD, pull-ups, 6 conn.   │          │ USB-C, 1S LiPo 500 mAh    │
                            └──────────────────────────┘          └──────────────────────────┘
```

Four PCB designs:

| Board | Qty / glove | Size | Stackup | Contents |
|---|---|---|---|---|
| **GW-POD** | 1 | 34 × 30 mm | 4-layer, 0.8 mm | ESP32-S3 module, charger, buck-boost, fuel gauge, USB-C, battery, switch, LEDs, buttons |
| **GW-PLATE** | 1 | ~66 × 40 mm, shaped with 4 knuckle tabs | 2-layer, 0.6 mm | I2C switch, 4 MCP sensors on the tabs, hand IMU, reference sensor, pull-ups, ESD, 6 connectors |
| **GW-NODE** | 10 | 6.0 × 8.0 mm | 2-layer, 0.6 mm | one TMAG5273 + 100 nF, 4 in-pads, 4 out-pads (variants NODE-A = A1, NODE-B = B1) |
| **GW-THUMB** | 1 | 10 × 12 mm | 2-layer, 0.6 mm | ICM-42688-P + decoupling, in/out pads |

All four go on **one JLCPCB panel order** (assembly included), so a beginner never hand-solders an LGA or SOT-23-6.

---

## 1. Bill of materials (per glove)

Active parts. LCSC numbers marked ✓ were found on LCSC/JLCPCB by the verification agent; the rest must be looked up (never guessed).

| Category | Ref | Function | MPN | Package | Qty | Board |
|---|---|---|---|---|---|---|
| MCU | U1 | Wi-Fi/BLE MCU module, native USB | ESP32-S3-MINI-1-N8 | module 15.4×20.5 | 1 | POD |
| Power | U2 | Li-ion charger with power path | BQ24074RGTR | VQFN-16 3×3 | 1 | POD |
| Power | U3 | 3.3 V buck-boost, 2 A, Iq 11 µA, adjustable (VFB 0.5 V) | TPS63802DLAR | 10-pin HotRod DFN 2×3 | 1 | POD |
| Power | L1 | 0.47 µH power inductor (TI's listed part) | XFL4015-471ME | 4×4×1.5 mm | 1 | POD |
| Power | U4 | Fuel gauge (ModelGauge) | MAX17048G+T10 | TDFN-8 2×2 | 1 | POD |
| Protection | U5 | USB D+/D− ESD | USBLC6-2SC6 | SOT-23-6 | 1 | POD |
| Protection | D1 | VBUS TVS | SMF6.0A | SOD-123FL | 1 | POD |
| Protection | Q1 | Battery reverse-polarity P-FET | DMG2305UX-7 | SOT-23 | 1 | POD |
| Interface | U10 | 8-ch I2C switch, 400 kHz, active-low reset | TCA9548APWR | TSSOP-24 | 1 | PLATE |
| Sensor | U11, U20 | 6-axis IMU | ICM-42688-P | LGA-14 2.5×3 | 2 | PLATE, THUMB |
| Sensor | U12–U16, node | 3D Hall | TMAG5273A1/B1/C1/D1QDBVR | SOT-23-6 | 15 | PLATE (5), NODE (10) |
| Protection | U17–U21 | 4-ch ESD on each external bus | TPD4E1U06DCKR | SC70-6 | 6 | PLATE |
| Connectors | J1 | USB-C receptacle (USB 2.0) | USB4105-GF-A | SMD/THT | 1 | POD |
| Connectors | J2 | Battery | S2B-PH-SM4-TB(LF)(SN) | JST-PH 2.0 SMD | 1 | POD |
| Connectors | J3 / J10 | Pod ↔ plate | SM06B-SRSS-TB | JST-SH 1.0, 6p | 2 | POD, PLATE |
| Connectors | J11–J15 | Plate ↔ finger / thumb | SM04B-SRSS-TB | JST-SH 1.0, 4p | 5 | PLATE |
| UI | SW1 | Power slide switch | JS102011SAQN | SMD SPDT | 1 | POD |
| UI | SW2, SW3 | BOOT/USER, RESET | PTS810 SJM 250 SMTR LFS | 4.2×3.2 SMD | 2 | POD |
| UI | D2, D3, D4 | Sync (red), status (green), charge (red) | 150060RS75000 / 150060GS75000 | 0603 | 3 | POD |
| Battery | BT1 | 1S LiPo 3.7 V ~500 mAh with PCM, 2-wire, JST-PH | 502535-class | 5×25×35 mm | 1 | — |

Passives are listed in the netlist (section 3.4) with values; all are 0402 X7R/X5R or 1 % thick-film unless noted.

---

## 2. System architecture

### 2.1 Power distribution

```
USB-C VBUS 5 V ──F──D1(TVS)──▶ BQ24074 IN ─┬─ OUT (4.4 V when USB present, = VBAT otherwise) ──▶ TPS63802 VIN ──▶ 3V3 rail
                                           │                                                        EN ◀── SW1 (on = OUT, off = GND)
                                           └─ BAT ◀──Q1 (rev-pol P-FET)── J2 ◀── 1S LiPo 500 mAh (PCM inside)
                                                         │
                                                    MAX17048 CELL/VDD (measures VBAT, I2C1)
3V3 rail ──▶ ESP32-S3 module, TCA9548A, LEDs, pull-ups ──J3──▶ plate 3V3 ──J11..J15──▶ nodes, thumb IMU
```

| Rail | Source | Nominal | Max load | Typical load | Notes |
|---|---|---|---|---|---|
| VBUS | USB-C | 5.0 V | 500 mA (USB 2.0 default, 5.1 k CC pull-downs) | 0 (unplugged when worn) | TVS + BQ24074 input OVP |
| SYS (BQ OUT) | BQ24074 | 3.0–4.4 V | 1.0 A (R_ILIM) | 150 mA | power path: glove runs while charging |
| VBAT | LiPo | 3.0–4.2 V | PCM limit (~2 A) | 140 mA | charge 250 mA (0.5 C) |
| 3V3 | TPS63802 | 3.30 V ±2 % | 2 A | 138 mA avg, 450 mA peak (Wi-Fi TX) | buck-boost: full battery range usable |

Current budget at 3V3 (200 Hz streaming): ESP32-S3 with ESP-NOW ~100 mA avg (VERIFY on the bench); 15 × TMAG5273 continuous 2.3 mA = 34.5 mA (verified); 2 × ICM-42688-P ~0.9 mA; TCA9548A, MAX17048, pull-ups, LEDs ~2 mA. **Total ≈ 138 mA → ≈ 137 mA from the cell at 90 % efficiency → ≈ 3.3 h on 500 mAh** (90 % usable). Firmware can halve the TMAG current with duty-cycled (trigger) conversions later.

Bulk and decoupling:
- BQ24074: IN 1 µF + 10 µF (0805, 10 V), OUT 4.7 µF + 10 µF, BAT 4.7 µF — all next to the pins.
- TPS63802: VIN 2 × 10 µF (0603, 10 V), VOUT 2 × 22 µF (0805, 6.3 V, X5R; account for DC-bias derating: ≥ 15 µF effective), inductor and caps in the tightest possible loop on L1.
- ESP32-S3 module: 22 µF + 100 nF at the 3V3 pin (Espressif reference), + 1 µF on EN.
- Every IC: 100 nF 0402 within 1 mm of its VDD pin, via straight to GND plane. TMAG5273: 100 nF (+ 1 µF on the plate side of each connector). ICM-42688-P: 100 nF on VDD and VDDIO, + 2.2 µF bulk per datasheet (VERIFY value).
- Plate: 10 µF at J10 (incoming 3V3) to absorb the cable inductance.

Protection:
- **USB ESD**: USBLC6-2SC6 on D+/D− and VBUS within 3 mm of J1.
- **VBUS surge**: SMF6.0A TVS; the BQ24074's input OVP is 10.5 V (verified; the BQ24072/3/5 use 6.6 V) and the IN pin is rated to 28 V.
- **Battery reverse polarity**: Q1 DMG2305UX in the BAT line (drain to battery +, source to BQ BAT, gate to GND through 100 Ω; body diode orients so the FET enhances only with correct polarity and conducts both ways for charging). Loss at 300 mA ≈ 6 mW.
- **Over-discharge**: cell PCM (2.5–2.8 V) as the last line; firmware stops at 3.30 V and deep-sleeps; TPS63802 UVLO as the middle line.
- **Charge temperature**: BQ24074 TS pin → 10 kΩ (R25) NTC in the battery pack: **buy a 3-wire pack with NTC**. A fixed 10 kΩ to GND makes the charger think the cell is always at 25 °C, so charge-temperature protection is lost (VERIFY the BQ24074's TS behaviour in the datasheet's device-comparison table before using that fallback).
- **External bus ESD**: every cable that touches skin (5 finger/thumb buses, pod link) gets a TPD4E1U06 on SDA/SCL/3V3 at the plate connector.

### 2.2 Pinout and signal map — ESP32-S3-MINI-1-N8 (GW-POD)

| GPIO | Net | Direction | Function / peripheral | Connected to | Level | Protocol | Notes |
|---|---|---|---|---|---|---|---|
| GPIO0 | BTN_BOOT_N | in | Boot strap / user button | SW2 to GND, 10 k to 3V3 | 3.3 V | GPIO | short press = MARK, long press = start/stop |
| GPIO1 | I2C1_SDA | io | Fuel-gauge bus | U4 SDA, 4.7 k to 3V3 | 3.3 V | I2C 100 kHz | local, never leaves the pod |
| GPIO2 | I2C1_SCL | out | Fuel-gauge bus | U4 SCL, 4.7 k to 3V3 | 3.3 V | I2C 100 kHz | |
| GPIO4 | I2C0_SDA | io | Sensor tree | J3 pin 3 → plate | 3.3 V | I2C 400 kHz | 3.3 k pull-up on pod |
| GPIO5 | I2C0_SCL | out | Sensor tree | J3 pin 4 → plate | 3.3 V | I2C 400 kHz | 3.3 k pull-up on pod |
| GPIO6 | MUX_RST_N | out | Switch reset | J3 pin 6 → TCA9548A RESET | 3.3 V | GPIO | 10 k pull-up on plate |
| GPIO7 | IMU_INT1 | in | Hand IMU data-ready / tap | J3 pin 5 ← U11 INT1 | 3.3 V | GPIO IRQ | push-pull from IMU |
| GPIO10 | FG_ALRT_N | in | Fuel-gauge alert | U4 ALRT, 10 k to 3V3 | 3.3 V | GPIO IRQ | open-drain |
| GPIO11 | PGOOD_N | in | USB power present | U2 PGOOD, 10 k to 3V3 | 3.3 V | GPIO | open-drain, low = USB |
| GPIO12 | LED_SYNC | out | Camera sync LED | D2 via 130 Ω | 3.3 V | GPIO | ~10 mA, drive strength 3 |
| GPIO13 | LED_STATUS | out | Status LED | D3 via 330 Ω | 3.3 V | GPIO | ~3 mA |
| GPIO14 | SIDE_SEL | in | Glove side | JP1 to GND = left | 3.3 V | GPIO | internal pull-up |
| GPIO19 | USB_DN | io | Native USB D− | J1 A7/B7 via U5 | USB | USB FS 12 Mb/s | 90 Ω diff |
| GPIO20 | USB_DP | io | Native USB D+ | J1 A6/B6 via U5 | USB | USB FS | 90 Ω diff |
| GPIO43 | U0TXD | out | Debug UART | TP1 | 3.3 V | UART | no logging (boot ROM only) |
| GPIO44 | U0RXD | in | Debug UART | TP2 | 3.3 V | UART | |
| GPIO15–18 | EXP0–3 | — | Expansion (fingertip FSR ADC later) | TP3–TP6 | 3.3 V | ADC1 capable | |
| GPIO3, 45, 46 | — | — | Strapping | NC | — | — | leave floating (defaults) |
| EN | CHIP_EN | in | Reset | 10 k to 3V3, 1 µF to GND, SW3 | 3.3 V | — | RC ≥ 10 ms |

No level translation anywhere: every device runs from the same 3V3 rail.

### 2.3 I2C tree and addresses

| TCA9548A ch | Location | Devices (7-bit address) |
|---|---|---|
| upstream | plate | TCA9548A 0x70 (A0–A2 = GND) |
| CH0 | index cable | PIP TMAG5273A1 0x35, DIP TMAG5273B1 0x22 |
| CH1 | middle cable | same |
| CH2 | ring cable | same |
| CH3 | little cable | same |
| CH4 | thumb cable | ICM-42688-P 0x68, MCP TMAG5273A1 0x35, IP TMAG5273B1 0x22 |
| CH5 | plate | MCP index A1 0x35, middle B1 0x22, ring C1 0x78, little D1 0x44 |
| CH6 | plate | hand ICM-42688-P 0x68, reference TMAG5273A1 0x35 |
| CH7 | spare connector J16 (4-pin) | expansion |
| I2C1 | pod only | MAX17048 0x36 |

Why a switch: same-address sensors repeat on every finger (identical NODE-A/NODE-B on all fingers), and a short-circuited or torn finger cable only takes down its own channel.

**Pull-ups and rise time.** The binding limit is the TMAG5273: its SDA output is specified at V_OL 0.4 V at only **2 mA** (verified). So the parallel pull-up seen when a channel is selected must be ≥ 3.3 V / 2 mA = 1.65 kΩ. Choice: **3.3 kΩ upstream (pod) + 3.3 kΩ per channel (plate) = 1.65 kΩ → 2.0 mA**. Rise time with ~160 pF total (pod trace, pod–plate cable, plate trace, finger cable, 2–3 inputs, switch): t_r = 0.8473 · 1650 Ω · 160 pF ≈ **224 ns < 300 ns** (I2C Fast-mode limit). The bus is fixed at **400 kHz**: the TCA9548A tops out there, and 1 MHz would need ~4× stronger pull-ups than the TMAG5273 can sink. (TMAG5273 itself supports 1 MHz Fm+ at VCC ≥ 2.3 V, verified, but not with this bus capacitance and sink limit.)

**Sweep timing at 400 kHz** (one 200 Hz frame = 5 ms): TMAG5273 1-byte-command read of X/Y/Z + status ≈ 8 bytes × 9 bits = 180 µs × 15 = 2.70 ms; channel switches 7 × 45 µs = 0.32 ms; IMU burst (14 bytes) 2 × 382 µs = 0.76 ms → **3.8 ms (76 % of the frame)**. At 1 MHz: 1.5 ms.

### 2.4 Wiring and harness

| Harness | From → to | Conductors | Wire | Connectors | Length |
|---|---|---|---|---|---|
| H1 pod–plate | GW-POD J3 → GW-PLATE J10 | 6: 3V3, GND, SDA, SCL, IMU_INT, MUX_RST_N | 30 AWG high-flex silicone (≥ 30 strands of 0.05 mm) | JST-SH SHR-06V-S + SSH-003T-P0.2 both ends | 90 mm |
| H2–H5 finger buses | PLATE J11–J14 → NODE-A (PIP) → NODE-B (DIP) | 4: 3V3, GND, SDA, SCL | 30 AWG high-flex silicone, twisted pairs SDA+GND, SCL+3V3 | JST-SH SHR-04V-S at plate; soldered to node pads | 75 mm + 30 mm |
| H6 thumb bus | PLATE J15 → GW-THUMB → NODE-A → NODE-B | 4 | same | same | 60 + 25 + 28 mm |
| H7 battery | pack → GW-POD J2 | 2 | pack leads (26–28 AWG) | JST-PH 2-pin | 40 mm |

Pin-to-pin:

| Pin | J3 / J10 (6-pin) | J11–J16 (4-pin) | Node pads (IN = OUT, pass-through) |
|---|---|---|---|
| 1 | GND | GND | GND |
| 2 | 3V3 | 3V3 | 3V3 |
| 3 | SDA | SDA | SDA |
| 4 | SCL | SCL | SCL |
| 5 | IMU_INT | — | — |
| 6 | MUX_RST_N | — | — |

GND on pin 1 everywhere, so a reversed housing puts GND, not 3V3, on a signal line.

Moving-cable rules:
- Wires ride on the ring saddles in TPU clips; over each joint they form a **service loop** of ≥ 1.5× the skin stretch (≈ 8 mm extra at the MCP, 6 mm at the PIP/DIP) with bend radius ≥ 5 mm.
- Solder joints on the node pads get a drop of flexible UV resin or silicone (strain relief): the conductor must flex in the free span, never at the joint.
- No connectors on the fingers: connectors are the most frequent failure point under flexing and they add weight. The fingers have soldered, replaceable harnesses; the plate end is pluggable.
- Target: > 100 000 flex cycles at 5 mm radius (verify with a bench flex test on a sample before the full build).

---

## 3. PCB design

### 3.1 Stackups

- **GW-POD (4-layer, 0.8 mm, 1 oz outer / 0.5 oz inner)**: L1 signals + components; L2 solid GND; L3 3V3 pour + SYS/VBAT pours; L4 signals + battery connector + switch. The solid L2 gives the USB pair and the module a clean reference and keeps the buck-boost loop current local.
- **GW-PLATE (2-layer, 0.6 mm)**: L1 components + routing; L2 GND pour, stitched every 5 mm. Thin board so it can sit on a curved TPU cradle.
- **GW-NODE / GW-THUMB (2-layer, 0.6 mm)**: L1 component; L2 GND pour; pads on L1.

### 3.2 Critical routing rules

| Net / area | Rule |
|---|---|
| USB D+/D− | 90 Ω differential (on JLC 0.8 mm 4-layer, ~0.20 mm width / 0.15 mm gap over L2; take numbers from the JLC impedance calculator for the chosen stackup), length < 15 mm, matched within 0.5 mm, no vias, ESD first then module |
| VBUS, SYS, VBAT | ≥ 0.4 mm (IPC-2221 external 1 oz: 1 A at +10 °C needs 0.30 mm; 2 A needs 0.78 mm, so use pours on the buck-boost input path) |
| TPS63802 loop | VIN caps, IC, L1 and VOUT caps on one side within a 6 × 6 mm area; switch nodes (L1 pins) as small as possible, no traces under L1 on L1/L2 other than GND |
| BQ24074 | thermal pad to GND with 5 vias Ø0.3 mm into L2; ISET/ILIM resistors at the pins with a quiet GND return |
| I2C | SDA/SCL routed together with GND alongside; series 33 Ω at the pod connector for edge damping; keep 1 mm from LED/switch nets |
| Antenna | ESP32-S3-MINI-1 antenna end at the board edge, overhanging or with a full keep-out (no copper on any layer) per Espressif layout guide; the battery and the cuff must not sit under the antenna; antenna faces away from the skin |
| Ground | single GND (no DGND/PGND split: there is no motor current). Return paths stay under their signals; the only noisy loop (buck-boost) is local on L1/L2 |
| Magnetics (plate) | no nickel-plated or ferrous parts within 5 mm of a TMAG5273 (ceramic capacitor terminations and some connector shells contain nickel: they add a **static** offset that calibration removes, but avoid them near the sensors anyway); no current loops above 10 mA near the sensors (a 10 mA trace at 1 mm gives ~2 µT) |

### 3.3 Thermal

There are no high-dissipation parts. Worst case is charging while streaming: BQ24074 dissipates (VOUT − VBAT)·I_chg + (VBUS − VOUT)·I_sys ≈ (4.4 − 3.4)·0.25 + (5.0 − 4.4)·0.4 ≈ 0.49 W. On the 4-layer board with a via-stitched thermal pad (θ_JA ≈ 45–50 °C/W, VERIFY), the rise is ≈ +24 °C. The pod touches skin, so firmware flags the IMU temperature above 45 °C, and the charge current is limited to 250 mA. TPS63802 at 138 mA dissipates < 50 mW. SOA: all parts run far below ratings (Q1 0.3 A in a 4 A part; TVS standoff 6 V vs 5.25 V max VBUS).

### 3.4 Netlist (GW-POD)

| Net | Connections |
|---|---|
| VBUS | J1 A4/A9/B4/B9, D1 cathode, U5 pin 5, C1 1 µF, C2 4.7 µF (IN total ≤ 10 µF for USB inrush, verified), U2 IN |
| CC1 / CC2 | J1 A5 / B5 → R1 / R2 5.1 kΩ to GND |
| USB_DP / USB_DN | J1 A6+B6 / A7+B7 → U5 I/O → U1 GPIO20 / GPIO19 |
| SYS | U2 OUT, C3 4.7 µF, C4 10 µF, U3 VIN, C5 C6 10 µF, SW1 common (through 100 k to U3 EN) |
| VBAT | U2 BAT, C7 4.7 µF, Q1 source, U4 CELL, U4 VDD |
| VBAT_CONN | J2 pin 1 (+), Q1 drain |
| Q1 gate | R3 100 Ω to GND |
| ISET | U2 ISET → R4 3.57 kΩ 1 % to GND (I_chg = K_ISET/R = 890/3570 ≈ 0.25 A; K_ISET 890 AΩ typ., verified) |
| ILIM | U2 ILIM → R5 1.50 kΩ 1 % to GND (I_lim = K_ILIM/R = 1610/1500 ≈ 1.07 A; K_ILIM 1610 AΩ typ. for 0.5–1.5 A, verified) |
| EN1 / EN2 | EN1 → 10 k to SYS (high), EN2 → GND (low): EN2 = 0, EN1 = 1 → 500 mA USB input limit (verified truth table; both pins have ~285 kΩ internal pull-downs, so never leave EN1 floating) |
| CE_N | U2 CE → GND (charging enabled) |
| TS | U2 TS → battery NTC (3-wire pack) or R6 10 kΩ to GND |
| TMR | U2 TMR left **open** → default safety timers: 30 min pre-charge, 5 h fast charge (verified; GND would disable them) |
| CHG_N | U2 CHG → D4 (red) → R7 1.5 kΩ → SYS |
| PGOOD_N | U2 PGOOD → U1 GPIO11, R8 10 k to 3V3 |
| SW_L1 / SW_L2 | U3 L1 / L2 ↔ L1 0.47 µH |
| 3V3 | U3 VOUT, C8 C9 22 µF, FB divider R21 511 kΩ (top) / R22 91 kΩ (bottom), TI's values: V_OUT = 0.5 V · (1 + 511/91) = 3.31 V (verified, SLVSEU9D Table 10-5), U1 3V3, C10 22 µF, C11 100 nF, U4 pull-ups, R pull-ups, J3 pin 2 |
| U3_EN | SW1 throw → SYS (on) / GND (off), R9 100 k pull-down |
| I2C0_SDA / SCL | U1 GPIO4 / GPIO5 → R10 R11 33 Ω → J3 pin 3 / 4; R12 R13 3.3 kΩ to 3V3 (on the bus side of the 33 Ω resistors) |
| I2C1_SDA / SCL | U1 GPIO1 / GPIO2 ↔ U4 SDA / SCL; R14 R15 4.7 kΩ to 3V3 |
| FG_ALRT_N | U4 ALRT → U1 GPIO10; R16 10 k to 3V3 |
| MUX_RST_N | U1 GPIO6 → J3 pin 6 |
| IMU_INT | J3 pin 5 → U1 GPIO7 |
| LED_SYNC | U1 GPIO12 → R17 130 Ω → D2 red → GND |
| LED_STATUS | U1 GPIO13 → R18 330 Ω → D3 green → GND |
| BTN_BOOT_N | U1 GPIO0, SW2 → GND, R19 10 k to 3V3 |
| CHIP_EN | U1 EN, R20 10 k to 3V3, C12 1 µF to GND, SW3 → GND |
| SIDE_SEL | U1 GPIO14 → JP1 (solder jumper) → GND |
| GND | everything else, L2 plane |

GW-PLATE netlist (short): J10 → 3V3/GND/SDA/SCL/IMU_INT/MUX_RST_N; C20 10 µF + C21 100 nF at J10; U17 TPD4E1U06 on J10 lines; U10 TCA9548A SDA/SCL upstream, A0–A2 GND (0x70), RESET ← MUX_RST_N with R30 10 k pull-up; SDn/SCn (n = 0…7) → 3.3 kΩ pull-ups to 3V3 each (16 resistors) → J11–J16 via TPD4E1U06 (one per connector); CH5 → U12–U15 (TMAG5273 A1/B1/C1/D1 on the four knuckle tabs, 100 nF each); CH6 → U11 ICM-42688-P (AD0 GND = 0x68, CS high for I2C, INT1 → IMU_INT) and U16 TMAG5273A1 (reference, ≥ 35 mm from any magnet).


### 1.1 Sourcing of the active parts

| MPN | LCSC | JLC class | Note |
|---|---|---|---|
| ESP32-S3-MINI-1-N8 | C2913206 ✓ | Extended | ~$3.3–4.7 |
| TMAG5273A1QDBVR | C3716049 ✓ | — | ~$0.35; B1, C1, D1 variants: look up (same package) |
| ICM-42688-P | C1850418 ✓ | Extended | ~5.5 k in stock at check |
| BQ24074RGTR | C54313 ✓ | — | RGTT (small reel) C2861252 |
| TPS63802DLAR | C2845237 ✓ | — | ~$0.64 |
| MAX17048G+T10 | C2682616 ✓ | — | |
| TCA9548APWR | look up | — | replaces PCA9848PWJ (C2674603, 0 in stock at check) |
| XFL4015-471ME | DigiKey/Mouser (Coilcraft) | — | TI's listed inductor for TPS63802 |
| USBLC6-2SC6, SMF6.0A, TPD4E1U06DCKR, DMG2305UX-7, USB4105-GF-A, JST SM04B/SM06B-SRSS-TB, S2B-PH-SM4-TB, JS102011SAQN, PTS810, Würth 150060xS75000 | look up | — | real parts, C-numbers not yet confirmed |

### 1.2 Passives (GW-POD)

| Ref | Value | Package | Rating | Function |
|---|---|---|---|---|
| C1 | 10 µF | 0805 | 10 V X5R | VBUS bulk |
| C2 | 1 µF | 0402 | 10 V X5R | BQ IN decoupling |
| C3, C7 | 4.7 µF | 0603 | 10 V X5R | BQ OUT, BAT |
| C4 | 10 µF | 0603 | 10 V X5R | BQ OUT bulk |
| C5, C6 | 10 µF | 0603 | 10 V X5R | TPS63802 VIN |
| C8, C9 | 22 µF | 0805 | 6.3 V X5R | TPS63802 VOUT (≥ 15 µF after DC-bias derating) |
| C10 | 22 µF | 0805 | 6.3 V X5R | ESP32 3V3 bulk |
| C11, C13–C16 | 100 nF | 0402 | 16 V X7R (GRM155R71C104KA88D) | per-IC decoupling |
| C12 | 1 µF | 0402 | 10 V X5R | EN reset RC |
| R1, R2 | 5.1 kΩ | 0402 | 1 % | USB-C CC sink |
| R3 | 100 Ω | 0402 | 1 % | Q1 gate |
| R4 | 3.57 kΩ | 0402 | 1 % | ISET → 0.25 A charge |
| R5 | 1.50 kΩ | 0402 | 1 % | ILIM → 1.07 A |
| R6 | 10 kΩ | 0402 | 1 % | TS fallback (DNP with 3-wire pack) |
| R7 | 1.5 kΩ | 0402 | 1 % | charge LED |
| R8, R16, R19, R20 | 10 kΩ | 0402 | 1 % | PGOOD, ALRT, BOOT, EN pull-ups |
| R9 | 100 kΩ | 0402 | 1 % | TPS EN pull-down |
| R10, R11 | 33 Ω | 0402 | 1 % | I2C0 series damping |
| R12, R13 | 3.3 kΩ | 0402 | 1 % | I2C0 upstream pull-ups |
| R14, R15 | 4.7 kΩ | 0402 | 1 % | I2C1 pull-ups |
| R17 | 130 Ω | 0402 | 1 % | sync LED ~10 mA |
| R18 | 330 Ω | 0402 | 1 % | status LED ~3 mA |
| R21 / R22 | 560 kΩ / 100 kΩ | 0402 | 1 % | TPS63802 FB → 3.30 V |
| R23 / R24 | 100 kΩ / 100 kΩ | 0402 | 1 % | EN1 high / EN2 low |
| JP1 | solder jumper | — | — | SIDE_SEL (closed = left) |
| TP1–TP6 | test pads | Ø1 mm | — | UART0, GPIO15–18 |

GW-PLATE passives: 16 × 3.3 kΩ (channel pull-ups), 1 × 10 kΩ (RESET pull-up), 10 µF 0603 at J10, 100 nF per IC (7), 1 µF at each external connector (6). GW-NODE: 1 × 100 nF 0402. GW-THUMB: 100 nF on VDD and VDDIO + 2.2 µF (VERIFY against ICM-42688-P datasheet).

---

## 4. Firmware

PlatformIO, Arduino-ESP32 3.3 (ESP-IDF 5.5), `espressif32@8.0.0`, C++17. Two environments: `glove` (custom pod board) and `dongle` (stock Seeed XIAO ESP32-S3). 3,430 lines incl. tests and the host reader. **Build: both envs compile with zero project warnings (`-Wall -Wextra`), re-run and confirmed: glove RAM 16.8 % / flash 29.8 %, dongle RAM 13.8 % / flash 26.8 %. Host tests: 40/40 pass (also clean under ASan/UBSan/TSan). Not yet run on hardware.**

### 4.1 Structure

```
firmware/
  platformio.ini                 envs: glove, dongle
  include/  glove_config.h       pins, thresholds, task layout, SensorMap (single source of truth, compile-time checked)
            protocol.h           all packets, static_assert on every offset
            crc16.h cobs.h spsc_ring.h
  src/glove/  i2c_bus  tca9548a  tmag5273  icm42688  max17048  leds  button      (one class per driver)
              sampler  radio  health  usb_console  glove_state.h  main.cpp      (tasks)
  src/dongle/main.cpp            ESP-NOW ⇄ USB CDC bridge for 2 gloves
  test/host/test_protocol.cpp    CRC, COBS, packet layout, ring buffer stress
  tools/glove_reader.py          PC reader: decode, drops, per-glove clock offset + drift
```

### 4.2 Tasks (FreeRTOS)

| Task | Core | Prio | Trigger | Job |
|---|---|---|---|---|
| sampler | 1 | 20 | esp_timer 5 ms (200 Hz) | the only bus user; reads 15 TMAG + 2 IMU channel by channel (alternating direction → 6 switches); timestamps sweep start; overrun flag |
| radio | 0 | 10 | ring buffer | ESP-NOW at 6 Mb/s, ≤ 4 frames in flight, pairing (HELLO → MAC in NVS), commands from the paired dongle only |
| health | 0 | 5 | 10 ms tick | battery, temperature, sensor fault retries, watchdog feeding, LEDs, button |
| console | 0 | 2 | USB CDC | `i` info, `s` scan, `d` dump, `b` battery, `r` raw stream, `t` toggle, `p` unpair |

The sampler never blocks: frames go through lock-free single-producer rings; a full ring drops the newest frame and counts it (`ring_drops` in every packet).

### 4.3 Data packet (155 bytes, little-endian, CRC-16/CCITT-FALSE)

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | magic 0x4754 ("TG") |
| 2 | 1 | protocol version |
| 3 | 1 | type (1 = data) |
| 4 | 1 | side 'L' / 'R' |
| 5 | 1 | rate / 10 |
| 6 | 4 | seq |
| 10 | 8 | t_us (sweep start, glove clock) |
| 18 | 2 | sweep_us |
| 20 | 4 | sensor_ok bitmask |
| 24 | 2 | mag_saturated bitmask |
| 26 | 2 | status flags (bus error, low/critical battery, charging, overrun, over-temperature, fuel-gauge fault, saturated, recovered, stale, USB raw) |
| 28 | 1 | battery SOC % |
| 29 | 2 | battery mV |
| 31 | 2 | ring drops |
| 33 | 2 | TX failures |
| 35 | 90 | magnetometers int16 [15][3] |
| 125 | 24 | IMUs int16 [2][6] (accel, gyro) |
| 149 | 4 | IMU temperature int16 [2] |
| 153 | 2 | CRC |

Dongle → PC: COBS frames with a 12-byte header (kind, side, dongle rx_t_us, RSSI, length). The PC fits each glove's clock to the dongle clock (offset + drift, lower-envelope fit: recovered drift within 2 ppm and offset within 300 µs in the synthetic test). The SYNC command makes the glove blink a start bit + 4-bit code on the sync LED (50 ms per bit) and send an event with the exact time of the first edge, which lines the glove up with the cameras.

### 4.4 Sensor settings

- TMAG5273: continuous XYZ, ±40 mT, 16× averaging (1.25 ms per XYZ set, 4 fresh sets per frame), 1-byte read mode (X, Y, Z, CONV_STATUS in one 7-byte read), ID check (manufacturer 0x5449), re-init on power-on-reset or diagnostic flag, saturation flag near ±32767.
- ICM-42688-P: WHO_AM_I 0x47 check (datasheet-confirmed), accel ±4 g, gyro ±1000 dps, 1 kHz low-noise mode, one burst read of temperature, accel and gyro.
- MAX17048 on its own 100 kHz bus: SOC, VCELL, CRATE, alert pin.

### 4.5 Safety and fault handling

There are no actuators, so "safety" here means the cell, the skin and data integrity:

| Condition | Detection | Action |
|---|---|---|
| Sensor errors | 5 consecutive failed reads | mark sensor faulty (bit cleared in `sensor_ok`), retry init every 1 s, at most one re-init per sweep; the frame still goes out |
| Bus stuck | 3 sweeps with every read failing | bus recovery (9 SCL clocks + STOP), TCA9548A hardware reset, stuck-SDA check per channel |
| Magnet too close / steel | raw value at full scale | `mag_saturated` bit per sensor |
| Low battery | SOC < 15 % or VCELL < 3.50 V | flag + status LED blink |
| Critical battery | VCELL < 3.30 V ×3 readings, no USB | critical event, stop streaming, TMAGs to sleep (1 µA), IMUs off, deep sleep, wake by button (back to sleep below 3.40 V) |
| Hot pod (skin) | IMU temperature > 45 °C | `over_temp` flag |
| Task hang | task watchdog 2 s on sampler, radio, health | panic + reboot |
| Radio loss | 200 sends without ACK | back to pairing; frames counted as TX failures |
| Brown-out | ESP32 BOD (left enabled) | reset |
| Overrun | sweep > 5 ms | `overrun` flag; host can send SET_RATE 100 |

Last lines of defence in hardware: cell PCM (over-discharge, short), BQ24074 thermal regulation and safety timers, TPS63802 UVLO, TVS and ESD diodes.

### 4.6 Timing per 200 Hz sweep (400 kHz)

Mux selects 6 × 50 µs = 300 µs; TMAG reads 15 × 185 µs = 2,775 µs; IMU bursts 2 × 390 µs = 780 µs → **3.86 ms of wire time out of 5.0 ms**. Driver overhead per transaction is not measured yet (~40 µs × 23 would give ~4.8 ms). This is the tightest number in the design: `sweep_us` in every packet shows the real figure on the first board, and 100 Hz is the fallback (still above the 30 fps of the cameras).

---

## 5. Open items before ordering (VERIFY list)

1. Measure the I2C driver overhead on the first board (decides 200 vs 100 Hz).
2. ICM-42688-P: re-check the register values against datasheet rev 1.9 (the firmware used rev 1.2) and its recommended decoupling.
3. TMAG5273: saturation threshold and SET_COUNT behaviour on real parts.
4. BQ24074: TS-pin behaviour without an NTC (buy a 3-wire pack to avoid the question).
5. TPS63802: land pattern (datasheet size statements conflict) and the feed-forward capacitor.
6. ESP32-S3-MINI-1: check the module's pin table for GPIO 1–18 availability before layout.
7. LCSC numbers for every "look up" part; TCA9548APWR stock.
8. ESP-NOW encryption (off now): enable before use outside your home.
9. MAX17048 default battery model vs the chosen cell.
10. Flex-life test of the 30 AWG harness on a bench rig (> 100 k cycles at 5 mm radius).

## 5a. KiCad

GW-POD schematic rev A0 is in `kicad/glove_pod/` (KiCad 9): ERC 0 errors / 0 warnings, netlist verified pin by pin (231 pins, 44 nets). See `kicad/README.md`.

## 6. Bring-up order

1. GW-POD alone on USB: 3V3 rail, current draw, USB CDC, `i` command.
2. Battery: charge LED, PGOOD, fuel gauge (`b`), switch off/on.
3. GW-PLATE: `s` bus scan must list 0x70, CH5 four sensors, CH6 IMU + reference.
4. One finger harness (NODE-A + NODE-B) on CH0: `d` frame dump, move a magnet by hand.
5. Full glove: `r` raw stream, `glove_reader.py` rates; then the dongle and ESP-NOW; then the SYNC blink filmed by a camera.
