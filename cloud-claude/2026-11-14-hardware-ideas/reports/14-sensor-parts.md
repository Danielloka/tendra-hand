# 14 - Sensor parts survey (angle, touch, tendon force, buses, wires)

Agent 14 "sensor scout", 2026-11-15. Scope: what a hobbyist can actually buy and fit in a 20-joint, 5-fingertip open-source hand. Extends `research/references/hands/sensing.md` and reports 05 and 07; it does not repeat their concepts.

**Tags.** **V** = read this session from a datasheet PDF (downloaded, text-extracted) or a product page. **V2** = read this session from a secondary page (shop, blog, search result), so treat as likely. **M** = from memory, not checked. **C** = my own calculation (script `scratchpad/mag.py`, method in section 6). Prices are 2026 single-unit retail from the pages named; they move.

Terms: **I2C address** = the 7-bit number a chip answers to; two chips with the same address cannot share a bus. **Diametric magnet** = disc magnetised across its diameter. **INL** = integral non-linearity (how far the angle reading is from a straight line). **Taxel** = one touch point.

---

## 1. Top recommendations

**1. Joint angle: TMAG5273 (TI), bare SOT-23-6 chip on a 4.6 mm board, Ø3 x 2 mm diametric N45 magnet, 1.5-2 mm gap.** It is the only candidate that satisfies all four of our constraints at once: tiny (package 2.9 x 2.8 mm including leads [V]), four factory addresses (A1 = 0x35, B1 = 0x22, C1 = 0x78, D1 = 0x44 [V]) plus a writable address register, 2.3 mA active [V], and a built-in angle engine (CORDIC) with 1/16 degree steps [V] and a spec of +-0.5 degree error in the X-Y plane (the "chip faces the pin end" case) and +-1.0 degree in the other two planes at 25 C [V]. Two catches: (a) supply is **1.7-3.6 V, absolute max 4 V** [V], so it must run from a 3.3 V rail, never from the 5 V servo rail; (b) the new address only works if you write it right after power-up at the *default* address, and it is lost on reset [V2, Adafruit page], so use four different variants (A1/B1/C1/D1) per bus instead of reprogramming. My field calculation says Ø3 x 2 gives ~55 mT in-plane at a 1.5 mm gap (C), so use the **+-80 mT** range of the x1 parts, not +-40 mT.

**2. Fingertip: MLX90393 (QFN-16 3 x 3 x 1 mm, 4 addresses by pins A0/A1) under a magnetised silicone skin; TMAG5273 is the cheaper fallback.** The MLX90393 is what ReSkin/AnySkin use, so there is working open-source calibration work to copy. Verified: 2.2-3.6 V, 1.5-3 mA while converting, 43 uA standby, 16-bit, sensitivity down to 0.161 uT/LSB in X/Y, I2C up to 400 kHz, **saturates at 50 mT** (standard parts; the "-1xx" thin-concentrator option saturates at 25 mT) [V]. The Adafruit board now ships at **0x18** base address (the 013/014 option), not the old 0x0C [V2]. The TMAG5273 gives the same 3-axis answer at ~half the chip cost with +-40/+-80 mT range, so for a Ø3 x 1.5 magnet 3.9 mm away (20 mT, report 05) either works; MLX90393 has lower noise at small signals (selectable gain), TMAG5273 is faster and simpler.

**3. Palm / phalanx pads: barometer in rubber, BMP581 (2.0 x 2.0 x 0.8 mm metal-lid LGA, <0.1 Pa RMS noise) potted in Ecoflex/Dragon Skin.** Verified size and noise [V]. It is the only barometer that is both small enough for a phalanx pocket and low-noise enough to feel a few grams through 3-5 mm of rubber. Avoid the BMP585 for fingers: it is **3.25 x 3.25 x 1.96 mm** [V] (it is the port/water-resistant one). For a big flat palm area, **FSR 402-class** pads (12.7 mm round, 0.3-10 N [V2]) are simpler and cost ~$4, but they drift and read only normal force.

**4. Bus plan (recommended): one I2C bus per finger, run by a small MCU in the palm that speaks UART to the ESP32-S3.** 21 magnetic chips (5 tips + 16 joint chips: 3 per finger x 4 + 4 thumb) split into 5 buses of 4-5 chips each. Per bus a 4-5 chip scan takes ~1.4 ms at 400 kHz (C), so 5 buses in parallel on an RP2040 (2 hardware I2C + PIO) give >500 Hz per sensor; a TCA9548A mux alone would serialise them to ~140 Hz. Only **4 wires** (3.3 V or 5 V, GND, TX, RX) cross the wrist. Alternatives in section 5.

**5. Wire and connector: 38 AWG-class silicone or PTFE flexible wire on the joint axis (twist, not bend), JST SH 1.0 mm 4-pin at each finger root.** Hard data is thin (see 7); the strongest evidence remains the ORCA failure (0.2 mm solid copper dead in 4 500-7 000 bend cycles, report 05) versus cable-maker claims of 10 million cycles for fine-stranded silicone cable [V2, Cicoil]. So: stranded, fine-strand (>= 19 strands), on the axis. JST SH is 1.0 A and 50 V per contact [V2]; keep a servo-style plug (JST SH/GH) only at finger root, never mid-finger.

**6. Don't buy:** rotary pots (Alps RDC506 / Bourns 3382 are 11-12 mm square, 2.2 mm tall [V2]; the finger is ~10 mm wide), AS5600 with the stock 6 mm magnet (SOIC-8 5 x 4 mm), MT6835 (TSSOP-16, 21-bit is overkill, 4.4 x 5 mm [V]).

**One stray-field warning that changes the layout (C).** A Ø3 x 2 magnet leaks ~3 mT (on its axis) at 10 mm, ~0.4 mT at 20 mm. With a 50 mT working field that is up to ~3.4 degrees of error at 10 mm and ~0.4 degree at 20 mm. So **neighbouring joint magnets must be >= 20 mm apart, or alternate polarity and be calibrated** (a two-point calibration of all 21 chips per finger pair on the bench). The 19.5-20 mm finger pitch is borderline; the MCP/PIP of the same finger (~25-40 mm) is fine. Steel bearings (MR63/MR84) next to a chip bend the field: not measured by anyone; test with report 08's "Hall near bearing" rig.

---

## 2. Joint angle sensors (category 1)

Fit test: Ø3-4 mm diametric magnet, 1-2 mm gap, board <= 5 x 5 mm.

| Part | Package (mm) | Interface / address | Resolution / accuracy | Rate | Supply / current | Price (chip / breakout) | Fits 3-4 mm magnet at 1-2 mm? | Notes |
|---|---|---|---|---|---|---|---|---|
| **TMAG5273 (TI)** | SOT-23-6, 2.9 x 2.8 incl. leads [V] | I2C to 1 MHz [V]; A1 0x35, B1 0x22, C1 0x78, D1 0x44 [V]; x1: +-40/+-80 mT, x2: +-133/+-266 mT [V] | 12-bit ADC, angle 1/16 deg per LSB [V]; angle error +-0.5 deg (X-Y), +-1.0 deg (Y-Z, X-Z) at 25 C, 32x average [V] | 10 kSPS 3-axis, 20 kSPS 1-axis, no averaging [V] | 1.7-3.6 V (abs max 4 V) [V]; 2.3 mA active, 1 uA sleep, 0.45 mA standby [V] | ~$1-2 chip [M]; Adafruit A1 $5.95 25.3 x 17.7 x 4.6 mm 1.7 g [V2]; SparkFun mini $5.50, 12.7 x 25.4 mm [V2] | **Yes** (needs 20-90 mT; Ø3x2 at 1.5 mm = ~55 mT in-plane, C) | Best overall. Sensitivity drift 5 % (ratio of X/Y cancels it) [V]. No 5 V. |
| **TMAG5170 (TI)** | VSSOP-8, 3.0 x 3.0 [V] | SPI to 10 MHz [V] | angle error +-0.5 deg (all planes, 25 C) [V]; 12-bit | ~kSPS | 3.4 mA active [V] | ~$4-5 [M] | Yes | SPI daisy-chain is nicer for a MCU-less run, but 4 SPI wires + CS per finger; no benefit over 5273 here. |
| **MLX90393 (Melexis)** | QFN-16 3 x 3 x 1 [V]; UTDFN-8 2.5 x 2 (fixed address, no pins) [V] | I2C <= 400 kHz or SPI [V]; 4 addresses via A0/A1; option -011 = 0x0C-0x0F, -012 = 0x10-0x13, -013 = 0x14-0x17, -014 = 0x18-0x1B [V] | 16-bit, 0.161 uT/LSB (X/Y), 0.294 (Z) at best gain [V]; no on-chip angle (compute atan2) | ~700 Hz burst (sensing.md, V) | 2.2-3.6 V [V]; 2.3-3 mA conversion, 43 uA standby [V] | ~$4-5 [M]; Adafruit $9.95 STEMMA, base 0x18 [V2] | Saturates at **50 mT** [V]: use a smaller/farther magnet (Ø2 x 1 at 1.5 mm = 22 mT, C) | Best as tip sensor. Works for angle too but costs 2x. |
| **MLX90395** | SOIC-8 / TSSOP-16 / QFN-16 [V] | I2C <= 1 MHz or SPI <= 10 MHz [V]; QFN has A0/A1 [V] | 2.5 uT/LSB (50 mT version), 7.14 uT/LSB (120 mT) [V] | burst up to 2 kHz [V] | 2.3-3 mA [V] | ~$5-6 [M] | Yes (50 or 120 mT versions) | Newer, wider range (50/120 mT), BAA = not recommended for new designs, use BBA [V]. Good if a magnet is strong. |
| **AS5600 (ams-OSRAM)** | SOIC-8 ~5 x 4 [M] | I2C **fixed 0x36** [V2]; analog / PWM out | 12-bit; INL +-1 deg, noise 0.015 deg (slow) [V, from the AS5600L sheet] | 150 us sampling [V] | 3.3 V or 5 V; ~6.4 mA [V] | ~$1-2 chip, $1-3 module [M] | Only with a bigger magnet: reference is Ø6 x 2.5, gap 0.5-3 mm, Bz 30-90 mT, centring tolerance 0.25 mm [V]. Ø3x2 gives Bz ~41 mT at 1.5 mm if centred (C), so it works, but alignment is tight | Fixed address means one chip per bus (or use analog output into an ADC). 6.4 mA x 20 = 130 mA. |
| **AS5600L** | SOIC-8 and **WLCSP 2.07 x 2.63 x 0.6** [V] | I2C **0x40**, address **one-time programmable** (I2CADDR is OTP, "R/W/P") [V] | same as AS5600 [V] | same | same | ~$2-3 [M] | as AS5600 | OTP address burn needs 3.3-3.5 V and is permanent [V]. WLCSP is the smallest, but needs reflow. |
| **AS5048B (ams)** | TSSOP-14 [M] | I2C, 4 addresses via pins [M] | 14-bit, +-0.5 deg [M] | | 3.3/5 V, ~15 mA [M] | ~$5-10 [M] | Needs bigger magnet [M] | Big package and current. Skip. (Datasheet download failed, all M.) |
| **MT6701 (MagnTek)** | SOP-8 or QFN-16 3 x 3 [V2] | I2C 0x06 (reprogrammable to 0x46 [V2]), SSI, ABZ, UVW, PWM, analog [V2] | 14-bit; accuracy +-1 deg typ, +-1.5 deg max [V2] | ~5 us delay [V2] | 3.0-5.5 V; ~10 mA [V2] | ~$1-2 chip (LCSC), ~$3-8 module [M] | Reference Ø6 x 2.5, 0.5-2 mm [V2] | Only 2 addresses, 10 mA each. Official PDF blocked (all V2). |
| **MT6835 (MagnTek)** | TSSOP-16 [V] | SPI to 16 MHz, 3.3-5 V [V] | 21-bit core, INL target < +-0.07 deg after calibration [V] | prop. delay < 10 us [V] | | LCSC ~$4.6-6.7, DigiKey EUR 3.2 on reel [V2]; module $17-22 [V2] | Needs bigger magnet [M] | Overkill; 5 chips would need 5 CS lines. |
| **TLV493D-A1B6 (Infineon)** | PG-TSOP-6, ~2.9 x 1.6 body [M] | I2C <= 1 Mbit/s, **two** addresses (0x5E / 0x1F, by SDA level at start) [M] | 12-bit, +-130 mT [V] | | 2.8-3.5 V, 10 uA at 10 Hz [V] | ~$1.5 [M] | Yes | Few addresses; datasheet is old (2016 era); no angle engine. Fallback only. |
| **KMZ60 (AMR, NXP)** | SO8 [M] | analog sin/cos [M] | ~0.1 deg [M] | analog | 5 V [M] | ~$5 [M] | needs ~25 mT in-plane [M] | Analog + ADC pair per joint (20 joints = 40 ADC lines). Not worth it. |
| **DRV5055 (TI) + ADC** | SOT-23 2.92 x 2.37 or TO-92 4 x 1.52 [V] | analog, ratiometric; options 100/50/25/12.5 mV/mT, +-21/+-42/+-85/+-169 mT [V]; 20 kHz bandwidth [V] | one axis only: one needs a linear arc sweep instead of rotation | | 3.3 V or 5 V [V] | ~$1 [M] | Yes | For the "arc sweep" idea (H2 in report 05), two DRV5055 or one TMAG5273. Not for full rotation. |
| **Potentiometer / resistive: Alps RDC506, Bourns 3382, Murata SV01** | 11 x 12 x 2.1-2.2 mm [V2] | analog, 10 kOhm, 333.3 deg range, +-2 % linearity [V2] | | | | $2-4 [M] | n/a | Too big for a 10 mm wide finger; life 1-2 M cycles (conflicting) [V2]; SV01 appears discontinued [V2]. Only useful on the wrist/forearm. |
| **Optical encoder (e.g. reflective strip)** | n/a | n/a | | | | | no | Not found as a hobby part under 8 mm. Skip. |

**Angle noise vs target.** The spec is +-1 degree absolute. 12 bit (AS5600) is 0.088 degree per step, 1/16 degree (TMAG5273) is 0.0625, so the chip resolution is never the limit; **the limits are magnet centring, tilt and stray fields**. Typical in-package error budget (C): TMAG5273 +-0.5 deg + centring 0.2 mm on a Ø3 magnet ~ +-1 to +-2 deg (needs a one-time per-joint calibration by sweeping against the servo's own position), + stray 0.4 deg. So "<= 1 degree" is reachable **after calibration**, not out of the box.

## 3. Touch and pressure (category 2)

| Part | Size (mm) | Interface / address | Signal | Rate | Supply | Price | Fingertip? Palm? | Notes |
|---|---|---|---|---|---|---|---|---|
| **MLX90393 + magnetised silicone (ReSkin / AnySkin)** | chip 3 x 3 x 1 [V]; AnySkin board 5 chips, skin 20 x 20 x 2 mm (sensing.md V) | I2C, 4 addresses [V] | 3-axis B, normal + shear | ~400 Hz for 5 chips (sensing.md V) | 2.2-3.6 V [V] | ReSkin <$30 / sensor (sensing.md V) | **Both**; tip use one chip + one magnet | Magnet saturates chip if too close; chose gain per magnet. |
| **TMAG5273 + one magnet** | 2.9 x 2.8 [V] | I2C, 4 addresses [V] | 3-axis B | up to 10 kSPS [V] | 1.7-3.6 V [V] | $1-2 chip | Fingertip | Simplest. +-40/80 mT range suits Ø3 x 1.5 at 3.9 mm (~20 mT, report 05). |
| **BMP581** | **2.0 x 2.0 x 0.8** metal-lid LGA [V] | I2C <= 1 MHz, SPI 12 MHz, I3C 12.5 MHz [V]; I2C 0x46 / 0x47 by SDO [V2] | absolute pressure, 30-125 kPa, noise **<0.1 Pa RMS** native, 0.08 Pa at highest OSR; relative accuracy +-6 Pa (BMP580/585) or +-0.4 Pa (BMP581 at 900-1100 hPa) [V, V2] | up to 622 Hz forced, 480 Hz continuous [V] | VDD 1.71-3.6 V, VDDIO 1.08-3.6 V; 1.3 uA at 1 Hz [V] | ~$2-3 chip [M]; Adafruit board [V2] | **Both** (pads on phalanges, palm) | Only 2 addresses per bus: use SPI + CS lines or a mux per finger. Not watertight in rubber: seal the port or use the metal lid pot. Normal force only. |
| **BMP585** | **3.25 x 3.25 x 1.96** LGA (port) [V] | same | same noise class, +-30 Pa absolute [V] | 622 Hz | same | | Palm only | Too big for phalanges. |
| **BMP388** | 2.0 x 2.0 x 0.75 LGA [V] | I2C 0x76/0x77 [M] | | | | ~$2 [M] | Both | Older, noisier than 581 (M). |
| **LPS22HH (ST)** | HLGA-10, 2.0 x 2.0 x 0.73 [V2] | I2C 0x5C / 0x5D [M] | 26-126 kPa, 0.5 hPa abs. accuracy [V2] | up to 200 Hz [M] | 1.7 V min [V2] | ~$2 [M] | Both | ST pages returned 503/reset this session; only package was confirmed. |
| **LPS28DFW (ST)** | 2.8 x 2.8 x 1.7 [M] | I2C/I3C/SPI [M] | water-resistant, to 4 bar [M] | | | | Palm | Not verified (ST blocked). |
| **MS5837 (TE)** | 3.3 x 3.3 x 2.75, gel-filled [M] | I2C 0x76 fixed [M] | 0.2-0.3 mbar at best [M] | ~1 kHz [M] | | ~$5-8 [M] | Palm | Gel fill is already waterproof but 2.75 mm tall. (Datasheet download failed.) |
| **TakkTile concept (Harvard)** | array 5 x 8 at 6 mm pitch, 6 mm Vytaflex 20 [V2] | chip ~$1 [V2] | ~3600 counts/N on one flexible unit; "1 g" claims are marketing [V2] | | | | Proof the idea works | Needs vacuum-degassed cast for consistency [V2]. |
| **FSR 402 (Interlink) / Adafruit Alpha** | 12.7 mm round active area; 0.56 mm thick; 0.3-10 N; >20 MOhm unloaded [V2] | analog | normal force +-10-25 % | kHz | | $3.2-4 [V2] | Palm / big pads | One ADC channel each. Drifts. |
| **Velostat matrix** | sheet | row/column scan | | | | ~$5 per sheet [M] | Palm | DIY, unreliable calibration. |
| **MPR121 (capacitive)** | QFN-20 3 x 3 x 0.65 [V] | I2C 0x5A-0x5D by ADDR pin [V] | 12 electrodes, touch / proximity | | 1.71-3.6 V (VDD 2.0-3.6 with regulator) [V] | ~$2 chip, $4-8 board [M] | Touch yes, force no | Good for "is something touching" skins; humidity drift (sensing.md). |
| **AD7147 / piezo / PVDF** | | | | | | | Slip vibration | M only; skip for v1. |

**How these differ.** Magnetic (MLX/TMAG + magnet): force *and* shear, on I2C, easy to replace the skin. Barometer: only normal force, but excellent for slip *vibration* (up to 480-622 Hz sampling [V]). FSR: simplest, worst accuracy. Suggested mix: magnetic on 5 tips; barometer on 2 proximal pads per finger if space; FSR or barometer in the palm.

## 4. Tendon force (category 3)

| Option | Size | Interface / data rate | Resolution | Notes |
|---|---|---|---|---|
| **Servo load register (SCS0009 / SCSCL family)** | free | UART bus. Present Load at address 60 (0x3C), 2 bytes, **0.1 % of PWM duty**, bit 10 = direction; voltage 0x3E (0.1 V), temperature 0x3F (C) [V2: Waveshare SC-servo map, aifitlab SCSCL table] | 0.1 % duty | **This is duty cycle, not current**, so it reads motor drive, not tendon tension: friction and speed pollute it. No separate current register on the SC servo page [V2]. Overload protect: threshold 80 %, time 2 s default (reg 0x27, 0x26) [V2]. Exact SCS0009 table was **not found**, expect the SCSCL layout [M]. |
| **Servo current (HLS3606M / HLS series)** | free | Present Current at address 69 (0x45), 2 bytes, **6.5 mA/LSB**, phase current [V2: aifitlab HLS table]; present load 60 (0.1 % PWM), position 56 (0.087 deg/LSB), speed 58 (0.732 rpm/LSB) [V2]; torque/current limit registers 16, 28, 44, 48; **mode 2 = constant current** (Target Current register 44, +-2047 x 6.5 mA) [V2] | 6.5 mA step | Real current feedback + a true current mode would give force control through the tendon. Confirms the case for the HLS upgrade. Still affected by friction (45-75 % efficiency per V2 review). |
| **Strain gauge on a flexure + ADC** | gauge 3-6 mm [M]; ADC below | **NAU7802** 24-bit, 2 channels, I2C fixed **0x2A**, SparkFun Qwiic $5.95 [V2]; **ADS1220** 24-bit, up to 2 kSPS, gain 1-128, 20 effective bits, SPI, 4-ch, internal ref [V]; **HX711** 24-bit, 10 or 80 SPS, 2.6-5.5 V, <1.5 mA, SOP-16 [V] | with 80 SPS HX711: ~mN class after calibration [M] | HX711 and NAU7802 both slow; one per strand = 32 channels: too many. |
| **Photo-reflector tension sensor (arXiv 2507.00464)** | **13 x 7 x 6.5 mm**, up to 200 N, 9.9 mN resolution [V2] | analog, 1 photo-reflector | RMSE 0.07-0.6 N in force-control tests [V2] | The interesting 2025 design for tendon hands; elastomer flexure (printable) + a cheap photo-reflector. Too big per strand inside a finger; fits at the **forearm spool end**. |
| **Small load cells (TAL220 1-10 kg, button cells)** | TAL220 ~ 80 x 12.7 x 12.7 [M]; button 12-20 mm [M] | HX711 / NAU7802 | | Bench use: calibrate the servo-load model on 1 strand. |
| **FSR under the strand turning point** | 12.7 mm [V2] | analog | +-10-25 % [V2] | Cheap indicator, not a measurement. |

Recommendation: use the **servo's own current** (HLS) as the per-joint force proxy, and add **tension sensors only at the forearm on 2-4 chosen strands** (index flex + thumb) to teach the model; do not instrument all 32.

## 5. Buses and hubs (category 4)

| Part | Size / package | Key facts | Use |
|---|---|---|---|
| **TCA9548A (TI)** | TSSOP-24 7.8 x 4.4, VQFN-24 4 x 4 [V] | 8 channels, 3 address pins (8 per bus), 0-400 kHz, **1.65-5.5 V**, all pins 5 V tolerant, power-up with all channels off [V] | One channel per finger; only one channel active at a time, so reading is sequential. $1-5 [M]. |
| **PCA9546A (TI)** | TSSOP-16 5 x 4.4 [V] | 4 channels, 3 address pins, reset pin [V]; header says 0-400 kHz (one table row shows 100 kHz at a low-voltage condition) [V] | If only 4 branches needed. |
| **P82B715 (TI/NXP)** | SOIC-8 4.9 x 3.9 [V] | 400 pF on the I2C side and **3000 pF** on the long side [V]; **VCC 4.5-12 V** [V] (not 3.3 V) | Long cables to 5 V systems; not needed for a 30 cm wrist run. |
| **PCA9615 (NXP)** | TSSOP-10 [V] | Differential I2C, up to 1 MHz Fm+, 30 mA drive, hot-swap, **cable to at least 3 m** [V]; VDD(A) 2.3-5.5 V, VDD(B) 3.0-5.5 V [V] | Best I2C-through-wrist if one bus must cross: 2 twisted pairs + power = 6 wires. SparkFun Qwiic board exists [V2]. |
| **LTC4311 (ADI)** | SOT-23-6 [M] | active I2C pull-up that speeds edges [M] | If long bus capacitance. Datasheet download failed. |
| **Small MCU boards (palm / per finger)** | | | |
| - XIAO RP2040 | ~20 x 17.5 mm [V2] | $5.40 [V2]; 2 I2C + PIO | |
| - Waveshare RP2040-Zero | ~23.5 x 18 mm [M] | EUR 3.5-4.4 [V2] | Best price/size for a palm hub. |
| - ESP32-C3 SuperMini | ~22.5 x 18 mm [M] | ~$2-3 [M]; 1 hardware I2C master | WiFi not needed; can talk ESP-NOW/UART. |
| - STM32G0 boards / ATtiny1616 | small [M] | cheap, tiny; 1 I2C [M] | Per-finger local MCU option (SOIC/QFN on a flex): one per finger reduces wires to 3-4 on a shared UART/RS-485/CAN stub. |
| **CAN / RS-485** | | MCP2515+TJA1050 or STM32 CAN, SN65HVD / MAX3485 [M] | Needed only if > 5 nodes on one pair; UART point-to-point is simpler for 1-2 palm hubs. |

**Bus plans for 21 magnetic chips + pads (V = verified parts).**

| Plan | Parts | Wires over the wrist | Rate | Risk |
|---|---|---|---|---|
| **A. Palm MCU, 5 local I2C buses, UART to ESP32 (recommended)** | RP2040-Zero + 4-5 chips per bus (A1/B1/C1 TMAG5273 + 1 MLX90393 per finger; thumb adds D1) | **4** (power, GND, TX, RX) | >= 500 Hz each (C) | MCU firmware to write; RP2040 has 2 hardware I2C + PIO for the other 3 buses. |
| **B. TCA9548A in the forearm, ESP32-S3 reads directly** | one mux, 5 channels | 20 (5 x SDA, SCL, shared VCC/GND = 4 per finger if no mux in the palm; mux then in the forearm) | ~140 Hz per sensor (sequential, C) | long I2C runs (>= 30 cm) hit capacitance; add PCA9615 or LTC4311. |
| **C. Differential I2C (PCA9615 pair) + TCA9548A in the palm** | 2 x PCA9615, 1 mux | 6 | ~140 Hz | extra chips; no MCU firmware. |
| **D. One MCU per finger on a flex, shared RS-485/CAN** | 5 x STM32G0 / ATtiny | 4-5 (shared bus) | 1 kHz local | 5 firmware images / more chips. Best long-term; not a first build. |

Addresses are not a problem if you take the factory variants: per finger TMAG5273 A1 0x35, B1 0x22, C1 0x78, D1 0x44 [V] and the MLX90393 with pin-set 0x18-0x1B or 0x14-0x17 [V]. Hobby boards mostly stock only A1/A2 [V2], so B1/C1/D1 must be bought as bare chips from Mouser/DigiKey/LCSC [M] and put on your own 4.6 mm boards.

**Power.** 21 chips x 2.5 mA = ~55 mA active. AS5600 x 20 would be 130 mA (6.4 mA each [V]). Feed the sensors from their own 3.3 V LDO (TMAG5273 abs max 4 V [V]) off the 5 V logic supply, not off the servo rail.

## 6. Method notes for the numbers I computed (C)

- **Magnet field.** Surface-charge model of a diametrically magnetised cylinder, Br = 1.35 T (N45), sensor point at the magnet top face + gap + 0.3 mm (die is slightly inside the package). Results at gap 1.0 / 1.5 / 2.0 mm: Ø3 x 2: in-plane |B| 87 / 55 / 36 mT, Bz 71 / 41 / 25 mT at 1 mm off axis. Ø4 x 2: 105 / 73 / 51 mT. Ø6 x 2.5: 131 / 103 / 81 mT. Ø2 x 1: 42 / 23 / 14 mT. This agrees with the AS5600 sheet window (Bz 30-90 mT for a Ø6 x 2.5 at 0.5-3 mm [V]). N35/N52 changes the numbers by about -20 % / +10 %.
- **Stray field.** Dipole on axis = Br x V / (2 pi d^3): Ø3 x 2 (V = 14.1 mm^3) gives 3.0 mT at 10 mm, 0.38 mT at 20 mm; angle error ~ B_stray / B_signal (rad).
- **I2C timing.** 11 bytes (write addr, reg, read addr, 8 data) x 9 bits at 400 kHz = ~0.25 ms + start/stop.
- Nothing here was bench-measured. The 1-degree claim needs report 08's Hall test.

## 7. Wires and connectors (category 5)

| Item | Facts | Tag | Fit for Tendra |
|---|---|---|---|
| **Fine stranded silicone wire** (McMaster: 36 AWG, 25/50 strands, OD 0.02 in = 0.5 mm, 50 V) | silicone insulation described as ultra-flexible for tight turns | V2 | Good choice for the on-axis runs. |
| **36 AWG PTFE / hook-up (Alpha Wire ThermoThin 2636, Daburn 2451: 7/44 strand, 0.25 mm jacket)** | thin jacket, but 7 strands of 44 AWG is fragile | V2 | PTFE jackets give a bend radius ~2x silicone in the same cable (Cicoil catalog) [V2]; fine for static or twisting runs. |
| **Cicoil high-flex silicone cables** | 10 million-cycle minimum life, strands 40 AWG or finer [V2] | V2 | Vendor claim under their test setup; LAPP's guide warns that a cycle rating only holds for the test bend radius and speed [V2]. |
| **UL 1582 flex test** | stops at 15 000 cycles [V2] | V2 | Not meaningful for our >= 1 M-cycle target. |
| **0.2 mm solid copper (ORCA)** | failed in 4 500-7 000 bend cycles | from report 05 | Counter-example: stay stranded and on the axis. |
| **Flex PCB (polyimide, RA copper)** | dynamic bend radius: IPC-2223 as quoted by JLCPCB = **100x thickness** (6x static); Hackster: 100-150x for 2 layers on 0.5 oz; one source says 10x (outlier) [V2]; EPEC: 1-2 copper layers for dynamic [V2] | V2 | A 0.1 mm single-layer flex wants >= 10 mm radius: only for *twist/hinge on axis*, not for tight 3 mm loops. No cycle number found tied to radius. |
| **Twist on axis** | report 05: 1 mm bundle twisted 95 deg over 10 mm = 0.34 % strain | from report 05 | Matches the "on-axis" rule. |
| **JST SH 1.0 mm** | 28 AWG pre-made jumper 102 / 305 mm, **1.0 A, 50 V**, tin contacts [V2]; mating-cycle number not retrieved (JST site 403) | V2 / M | 4-pin version: smallest practical board-to-wire plug; use at the finger root. |
| **JST GH 1.25 mm / Molex PicoBlade 1.25 mm** | PicoBlade durability **30 cycles**, ~1 A (0.8 A with 32 AWG) [V2] | V2 | Fine for assembly; do not plan on daily swaps. Gold contacts preferred. |
| **Pogo pins / slip rings** | not researched in depth | M | A twisted-pair on the axis (+- 100 deg) is better than a slip ring for a hand; slip rings are for the wrist's continuous forearm twist only. |

Wire count per finger: 4 (3.3 V, GND, SDA, SCL) for Plan A/B, shared GND/VCC could cut to 2 + 2 x 5. A 4 x 38 AWG round bundle is ~1.5 mm OD (M), the same as the 2.6 mm bore in report 05 with ample margin.

## 8. What to buy for the first bench test (about EUR 70)

1. 6 x TMAG5273A1 + 4 x each of B1, C1, D1 (bare chips, Mouser/DigiKey) and 4.6 mm test boards, or 2 Adafruit A1 boards ($5.95 each, 25.3 x 17.7 mm [V2]) for the first rotation test.
2. 2 x MLX90393 breakouts (Adafruit $9.95 [V2]) for tip/skin work, 1 x BMP581 breakout.
3. Ø3 x 2 and Ø4 x 2 diametric N45 magnets (10 each), Ø3 x 1.5 axial for the tips.
4. 1 x RP2040-Zero (EUR ~4), 1 x TCA9548A breakout.
5. 10 m of 36-38 AWG silicone wire, 10 JST SH 4-pin pigtails.
6. 1 x NAU7802 Qwiic scale ($5.95 [V2]) + a 1 kg load cell for the servo-load calibration.

## 9. Open items (not verified)

- MT6701 official datasheet (blocked), LPS22HH/LPS28DFW (ST site refused), MS5837, AS5048B, LTC4311, TLV493D addresses, RP2040-Zero / ESP32-C3 sizes, prices for bare chips. All marked M or V2 above.
- SCS0009 specific register map (found only the SCSCL and HLS tables); the "Load" value is PWM duty, so tendon force must come from HLS current or a dedicated sensor.
- Stray-field effect of 440C steel bearings next to a TMAG5273: nobody has published it; bench test needed.
- Flex-circuit cycles vs radius: no number found; ask a fab for test data.

## 10. Sources

Datasheets read this session (downloaded, text-extracted):
- TMAG5273 SLYS045C Apr 2026: https://www.ti.com/lit/ds/symlink/tmag5273.pdf
- TMAG5170: https://www.ti.com/lit/ds/symlink/tmag5170.pdf
- DRV5055: https://www.ti.com/lit/ds/symlink/drv5055.pdf
- TCA9548A: https://www.ti.com/lit/ds/symlink/tca9548a.pdf
- PCA9546A: https://www.ti.com/lit/ds/symlink/pca9546a.pdf
- P82B715: https://www.ti.com/lit/ds/symlink/p82b715.pdf
- ADS1220: https://www.ti.com/lit/ds/symlink/ads1220.pdf
- MLX90393: https://www.melexis.com/-/media/files/documents/datasheets/mlx90393-datasheet-melexis.pdf
- MLX90395: https://media.melexis.com/-/media/files/documents/datasheets/mlx90395-datasheet-melexis.pdf
- AS5600L (ams, v1-12): https://static.chipdip.ru/lib/717/DOC011717888.pdf
- MT6835 Rev 1.3: https://cdn.promelec.ru/upload/items/2026/01/26/magntek~mt6835gt-std.pdf
- TLV493D-A1B6: https://www.infineon.com/dgdl/Infineon-TLV493D-A1B6-DataSheet-v01_10-EN.pdf?fileId=5546d462525dbac40152a6b85c760e80
- BMP581: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf
- BMP585: https://www1.futureelectronics.com/doc/Bosch%20Sensortec/bst-bmp585-ds003.pdf
- BMP388: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf
- MPR121: https://www.sparkfun.com/datasheets/Components/MPR121.pdf
- HX711: https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/hx711_english.pdf
- PCA9615 Rev 2: https://www.farnell.com/datasheets/4000002.pdf

Product / secondary pages (V2):
- Adafruit TMAG5273 A1: https://www.adafruit.com/product/6489
- SparkFun Mini TMAG5273: https://www.sparkfun.com/sparkfun-mini-linear-3d-hall-effect-sensor-tmag5273-qwiic.html
- Adafruit MLX90393: https://www.adafruit.com/product/4022 and https://learn.adafruit.com/mlx90393-wide-range-3-axis-magnetometer/pinout
- Adafruit BMP580/581/585 guide: https://learn.adafruit.com/adafruit-bmp580-bmp581-and-bmp585-temperature-and-pressure-sensor/overview
- MT6701 article: https://curiousscientist.tech/blog/mt6701-magnetic-encoder
- Adafruit FSR (Alpha MF01A): https://www.adafruit.com/product/166
- SparkFun Qwiic Scale NAU7802: https://www.sparkfun.com/sparkfun-qwiic-scale-nau7802.html
- Feetech HLS memory table: https://aifitlab-wiki.super.site/feetech-servo-motor-docs/feetech-hls-servo-memory-table-analysis
- SC servo memory map (Waveshare): https://docs.waveshare.com/Memory_Map_Explanation/SC_Servo_Memory_Map_Explanation
- Photo-reflector tendon tension sensor: https://arxiv.org/html/2507.00464v1
- TakkTile (Harvard): https://www.biorobotics.harvard.edu/takktile.html
- Cicoil high-flex catalog: https://www.cicoil.com/high-flex-cables-catalog ; LAPP flex-life guide: https://jj-lapp.com/?p=4030 ; UL 1582 note: https://www.machinedesign.com/cables-connectors-enclosures/when-flexible-cable-doesn-t-flex-long
- Flex PCB bend radius: https://jlcpcb.com/blog/flexible-pcb-bend-radius ; https://www.hackster.io/Hassnain/flex-circuits-under-motion-radius-copper-and-layout-decis-cba82d
- PicoBlade / JST SH: https://dev4.molex.com/content/dam/molex/molex-dot-com/en_us/pdf/datasheets/987652-6322.pdf ; https://www.datasheets.com/jst/asshssh28k305
- Alps RDC506 / Bourns 3382 / Murata SV01 listings via Mouser/DigiKey/RS (see search results).
