# Tendra Glove — KiCad

KiCad 9 project for the glove electronics. Board 1 of 4 is done: **GW-POD** (wrist pod).

| Board | Status |
|---|---|
| GW-POD (ESP32-S3, charger, 3.3 V, fuel gauge) | schematic rev A0: ERC clean, netlist verified, no layout yet |
| GW-PLATE (TCA9548A, 4 MCP sensors, hand IMU) | not started |
| GW-NODE (one TMAG5273) | not started |
| GW-THUMB (ICM-42688-P) | not started |

## Files

```
glove_pod/
  glove_pod.kicad_pro / .kicad_sch   open this in KiCad 9 (KiCad 8 should also open it)
  glove_pod.pdf                      printable schematic (A2)
  glove_pod.net                      netlist exported by kicad-cli
  glove_pod_bom.csv                  grouped BOM (refs, value, footprint, MPN, LCSC)
  erc.rpt                            ERC report: 0 errors, 0 warnings (all severities)
  intended_nets.tsv                  every pin -> net, as the generator meant it
  Tendra_Glove.kicad_sym             custom symbols: MAX17048G+T10, TPS63802DLA
  Tendra_Glove.pretty/               custom footprint: TI DLA0010A (TPS63802)
  sym-lib-table, fp-lib-table        project library tables (${KIPRJMOD})
tools/
  gw_pod.py     the design: every part, value, MPN and pin->net (edit this, then regenerate)
  schgen.py     tiny schematic generator (KiCad symbols, label-style connections)
  projlib.py    writes the project library, tables and .kicad_pro, then the schematic
  sexp.py       S-expression reader/writer
```

Regenerate: `cd tools && python3 projlib.py`. Then check:

```
cd ../glove_pod
kicad-cli sch erc --severity-all -o erc.rpt glove_pod.kicad_sch
kicad-cli sch export netlist -o glove_pod.net glove_pod.kicad_sch
```

The schematic is **label-style**: every pin has a short wire and a net label or power symbol, so connectivity is explicit. Once it is final, you can tidy it by hand in KiCad (the generator then stops being the source of truth; note that in this file when it happens).

## How it was checked

1. **ERC** (KiCad 9.0.9, all severities): 0 errors, 0 warnings.
2. **Netlist comparison**: KiCad's exported netlist vs. `intended_nets.tsv`: 231/231 pins on the intended net, every no-connect pin unconnected, no single-pin nets (44 nets).
3. **Datasheets** (read locally as PDFs on 2026-11-14):
   - TPS63802 (TI SLVSEU9D): pinout, VFB 0.5 V, R1 511 kΩ / R2 91 kΩ for 3.3 V, 10 µF in / 22 µF out, XFL4015-471ME, DLA0010A land pattern (pads copied exactly into the footprint).
   - BQ24074 (TI SLUS810N): IN cap 1–10 µF (USB inrush), OUT 4.7–47 µF, ~CE/EN1/EN2 must not float, ITERM open = 10 %, TMR open = default timers, 10 kΩ on TS allowed without NTC, thermal pad = VSS.
   - MAX17048 (Maxim 19-6171): CTG and QSTRT to GND, CELL not connected (MAX17048), VDD to the cell with 0.1 µF, EP to GND, address 0x36, 400 kHz.

## Open before layout

- **L1 footprint**: Coilcraft XFL4015-471ME is not in the KiCad libraries. Make it from the Coilcraft land pattern.
- **U2 / U4 exposed pads**: KiCad's generic VQFN-16 (EP 1.6 mm) and TDFN-8 (EP 0.8 × 1.2 mm) footprints are assigned; compare them with TI RGT0016 and Maxim's TDFN land patterns.
- **U1**: KiCad uses the ESP32-S2-MINI-1 footprint for the S3-MINI-1 (same 65-pad land). Check against Espressif's S3-MINI-1 datasheet, and keep the antenna keep-out.
- **D1**: drawn with a Zener-style diode symbol (cathode on VBUS). KiCad's "SMF6V0A" symbol is bidirectional, which does not match the unidirectional SMF6.0A.
- LCSC numbers for the parts still marked "look up" in design.md.

## Changes from design.md (rev A draft) made while drawing

- TPS63802 feedback: **511 kΩ / 91 kΩ** (TI's table) instead of 560 k / 100 k.
- BQ24074 IN capacitance: **1 µF + 4.7 µF** (≤ 10 µF for USB inrush) instead of 10 µF + 1 µF.
- EN1: 10 kΩ pull-up to SYS; EN2 and ~CE tied to GND.
- I2C0 pull-ups (3.3 kΩ) sit on the **bus side** of the 33 Ω series resistors.
- Q1 uses the `Transistor_FET:DMG2301L` symbol (same SOT-23 G-S-D pinout as DMG2305UX).
