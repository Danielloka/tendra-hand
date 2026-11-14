# 05 — BOM and datasheet verification (component agent, Sonnet + main session)

Checked on 2026-11-14 against datasheets and LCSC/JLCPCB pages. Prices and stock change daily.

| Part | Result |
|---|---|
| ESP32-S3-MINI-1-N8 | exists, LCSC C2913206 (extended) |
| TMAG5273A1QDBVR | exists, LCSC C3716049; addresses A1 0x35, B1 0x22, C1 0x78, D1 0x44; ±40/80 mT; **SDA sink only 2 mA at 0.4 V**; 1 MHz needs VCC ≥ 2.3 V; address re-programmable (0x0C, not persistent); 2.3 mA active, 1 µA sleep; I2C_RD 1-byte mode in DEVICE_CONFIG_1[1:0]; register map 0x00–0x18 |
| PCA9848PWJ | exists (C2674603) but **0 in stock** → replaced by **TCA9548APWR** (400 kHz) |
| ICM-42688-P | exists, LCSC C1850418; 0x68, WHO_AM_I 0x47 (firmware agent confirmed from datasheet rev 1.2) |
| BQ24074RGTR | exists, LCSC C54313; K_ISET 890 AΩ; **K_ILIM 1610 AΩ** (not 1550); EN2/EN1 table 00 = 100 mA, 01 = 500 mA, 10 = ILIM, 11 = standby; TMR open = 30 min / 5 h timers; OVP 10.5 V |
| TPS63802DLAR | exists, LCSC C2845237; adjustable, VFB 0.5 V; TI inductor **XFL4015-471ME** |
| MAX17048G+T10 | exists, LCSC C2682616 |

Main session, from the PDFs read locally (the agent's fetch tool could not parse them):
- **TPS63802 (SLVSEU9D):** pins 1 EN, 2 MODE, 3 AGND, 4 FB, 5 PG, 6 VOUT, 7 L2, 8 GND, 9 L1, 10 VIN; no exposed pad; R1 511 kΩ / R2 91 kΩ for 3.3 V; 10 µF in, 22 µF out; DLA0010A land pattern (pads copied into the custom footprint).
- **BQ24074 (SLUS810N):** IN capacitance 1–10 µF (USB inrush); OUT 4.7–47 µF; ~CE, EN1, EN2 must not float; ITERM open = 10 % termination; 10 kΩ on TS allowed without NTC; thermal pad = VSS.
- **MAX17048 (19-6171):** 1 CTG → GND, 2 CELL n.c. (MAX17048), 3 VDD → cell + 0.1 µF, 4 GND, 5 ALRT, 6 QSTRT → GND, 7 SCL, 8 SDA, EP → GND; address 0x36; 400 kHz.

Not looked up yet: LCSC numbers of USBLC6-2SC6, SMF6.0A, TPD4E1U06DCKR, DMG2305UX-7, USB4105-GF-A, JST connectors, C&K switches, Würth LEDs, TCA9548APWR.
