"""15 · Sensing budget: bus timing, link load, wire twist fatigue, power, cost (agent 15, 2026-11-15).

Plain Python 3:  python3 cad/15_budget.py   -> out/15/budget.json, out/15/budget.txt
Prices: report 14 (V/V2) where it gives them, else [M] (memory, verify). Euro ~ dollar here.
"""
import json
import math
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "out" / "15"
OUT.mkdir(parents=True, exist_ok=True)
L = []


def p(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    L.append(s)


# ----------------------------------------------------------------------------------------- sensor list
FINGERS = ["index", "middle", "ring", "little"]
sensors = []
for f in FINGERS:
    for j, var, addr in (("mcp_abd", "TMAG5273A1", 0x35), ("mcp_flex", "TMAG5273B1", 0x22), ("pip", "TMAG5273C1", 0x78),
                         ("dip", "TMAG5273D1", 0x44)):
        sensors.append(dict(id=f"{f}_{j}", kind="angle", chip=var, addr=addr, bus=f, rate=500))
    sensors.append(dict(id=f"{f}_tip", kind="tip3", chip="MLX90393-014 (0x18)", addr=0x18, bus=f, rate=200))
    sensors.append(dict(id=f"{f}_pad_prox", kind="pad", chip="BMP581 (SDO low)", addr=0x46, bus=f, rate=200))
    sensors.append(dict(id=f"{f}_pad_mid", kind="pad", chip="BMP581 (SDO high)", addr=0x47, bus=f, rate=200))
for j, var, addr in (("cmc_rot", "TMAG5273A1", 0x35), ("cmc_flex", "TMAG5273B1", 0x22), ("mcp_flex", "TMAG5273C1", 0x78),
                     ("ip", "TMAG5273D1", 0x44)):
    sensors.append(dict(id=f"thumb_{j}", kind="angle", chip=var, addr=addr, bus="thumb", rate=500))
sensors.append(dict(id="thumb_tip", kind="tip3", chip="MLX90393-014 (0x18)", addr=0x18, bus="thumb", rate=200))
sensors.append(dict(id="thumb_pad_prox", kind="pad", chip="BMP581 (SDO low)", addr=0x46, bus="thumb", rate=200))
sensors.append(dict(id="thumb_pad_meta", kind="pad", chip="BMP581 (SDO high)", addr=0x47, bus="thumb", rate=200))
for i, z in enumerate(("palm_thenar", "palm_hypothenar", "palm_centre", "palm_distal")):
    sensors.append(dict(id=z, kind="pad", chip=f"BMP581 ({'SDO low' if i % 2 == 0 else 'SDO high'})",
                        addr=0x46 + i % 2, bus="palm_a" if i < 2 else "palm_b", rate=200))
sensors.append(dict(id="palm_tof", kind="range", chip="VL53L1X (0x29)", addr=0x29, bus="palm_a", rate=50))

# address clash check
buses = {}
for s in sensors:
    buses.setdefault(s["bus"], []).append(s)
for b, ss in buses.items():
    a = [s["addr"] for s in ss]
    assert len(a) == len(set(a)), (b, a)
p(f"{len(sensors)} sensors on {len(buses)} I2C buses, no address clash")
p("counts:", {k: sum(1 for s in sensors if s["kind"] == k) for k in ("angle", "tip3", "pad", "range")})

# ----------------------------------------------------------------------------------------- I2C timing
# bits per transaction: 9 bits/byte + ~2 bit-times per START/STOP/repeated START; plus 15 us of MCU gap
TX = {"angle": 5, "tip3": 2 + 9, "pad": 6, "range": 2 + 2 + 3}  # bytes (TMAG: addr reg addr d d; MLX: RM cmd + 1+6 data + status; BMP 3-byte press)
STARTS = {"angle": 3, "tip3": 5, "pad": 3, "range": 4}
GAP_US = 15.0
p("\nI2C load per bus (bus clock 400 kHz: MLX90393 limit; TMAG5273/BMP581 allow 1 MHz)")
timing = {}
for f_clk in (400e3, 1e6):
    for b, ss in buses.items():
        if f_clk > 500e3 and any(s["kind"] == "tip3" for s in ss):
            continue
        t = sum(s["rate"] * ((TX[s["kind"]] * 9 + 2 * STARTS[s["kind"]]) / f_clk + GAP_US * 1e-6) for s in ss)
        timing[f"{b}@{int(f_clk / 1e3)}k"] = round(t * 100, 1)
for k, v in timing.items():
    p(f"  {k:16s} {v:5.1f} % busy")
worst = max(v for k, v in timing.items() if k.endswith("400k"))
p(f"  worst bus at 400 kHz: {worst} % (target < 60 %)")
# single-bus alternative (TCA9548A, everything sequential at 400 kHz)
tot = sum(v for k, v in timing.items() if k.endswith("400k"))
p(f"  ALL buses on one 400 kHz master through a TCA9548A (+1 mux write per bus per 500 Hz frame): "
  f"{tot + len(buses) * 500 * (2 * 9 / 400e3 + GAP_US * 1e-6) * 100:.0f} % -> does not fit")

# ----------------------------------------------------------------------------------------- link to the ESP32-S3
frame = 2 + 4 + 20 * 2 + 5 * 6 + 14 * 2 + 1 * 2 + 2 * 1 + 2  # sync, timestamp, angles, tips xyz, pads, tof, status, CRC16
baud = 2_000_000
load = frame * 10 * 500 / baud
p(f"\nlink frame {frame} B at 500 Hz -> {frame * 500 / 1e3:.1f} kB/s; UART {baud / 1e6:.0f} Mbaud load {load * 100:.0f} %")
p(f"  latency: frame time on the wire {frame * 10 / baud * 1e6:.0f} us + one 2 ms bus cycle")

# ----------------------------------------------------------------------------------------- wire twist fatigue
# conductor axial strain in a parallel (untwisted) bundle twisted by theta over free coaxial length Lf:
#   eps = sqrt(1 + (r theta / Lf)^2) - 1   (r = conductor centre distance from the bundle axis)
WIRES = {"38 AWG PTFE (7x0.04), OD 0.30": 0.30, "36 AWG silicone (7x0.05), OD 0.50": 0.50}
JOINTS = [  # name, range deg, free coaxial length mm (pin bore between the two glue points), conductors
    ("finger dip (pin bore -x end to radial x +3.2)", 76.0, 9.0),
    ("finger pip (pin bore -x end -5.65 to radial x +3.6)", 100.0, 9.2),
    ("finger mcp (axle bore radial x +3.0 to -x end -7.4)", 100.0, 10.4),
    ("finger abd (palmar stub Ø3 tube, 6.8 long)", 30.0, 6.0),
    ("thumb ip / mcp (same module as the pip)", 100.0, 9.0),
    ("thumb cmc_flex / cmc_rot (Ø5 hollow pin, 10 mm)", 141.0, 10.0),
    ("wrist flex (Ø5 hollow pin, bore 2.4, 6 wires)", 100.0, 9.0),
    ("wrist dev (Ø5 hollow pin, bore 2.4, 6 wires)", 50.0, 9.0),
]
LIMIT = 0.20  # % strain range for >= 1e6 cycles, fine-stranded annealed copper [M, conservative; verify on bench]
p(f"\nwire twist strain (parallel bundle, square for 4 wires, hexagon ring for 6); limit {LIMIT} % for 1e6 cycles [M]")
fat = []
for jn, rng, Lf in JOINTS:
    n = 6 if "wrist" in jn else 4
    for wn, od in WIRES.items():
        r = od / math.sqrt(2) if n == 4 else od  # square: centre distance od/sqrt2; 6 around one: od
        e = (math.sqrt(1 + (r * math.radians(rng) / Lf) ** 2) - 1) * 100
        fat.append(dict(joint=jn, wire=wn, range_deg=rng, L_mm=Lf, strain_pct=round(e, 3), ok=e <= LIMIT))
        p(f"  {jn:52s} {wn:34s} {e:6.3f} %  {'OK' if e <= LIMIT else 'TOO HIGH'}")
# the same wire bent in a loop of radius R (for comparison): eps = (d_strand/2)/R
for R in (3.0, 10.0, 15.0):
    p(f"  bend, R {R:4.1f} mm: strand Ø0.05 -> {0.025 / R * 100:.2f} %, strand Ø0.127 (solid 36 AWG) -> {0.0635 / R * 100:.2f} %")

# ----------------------------------------------------------------------------------------- power
I = {"angle": 2.3, "tip3": 3.0, "pad": 0.6, "range": 18.0}  # mA active (14: TMAG 2.3, MLX 1.5-3, BMP581 ~0.6 at 200 Hz [M], VL53L1X 16-19 [M])
i3 = sum(I[s["kind"]] for s in sensors)
i_mcu = 25.0
p(f"\npower: sensors {i3:.0f} mA at 3.3 V, RP2040 ~{i_mcu:.0f} mA -> {i3 + i_mcu:.0f} mA from the hub LDO "
  f"({(i3 + i_mcu) * 5 / 1000:.2f} W from 5 V)")
r36 = 1.36  # ohm/m, 36 AWG [M]; doubled GND + 5V pair in the 6-core cable
p(f"  drop over 0.5 m of 36 AWG (out + back): {(i3 + i_mcu) / 1000 * r36 * 1.0 * 1000:.0f} mV of 5 V: fine")

# ----------------------------------------------------------------------------------------- cost
BOM = [  # item, qty, unit EUR, source tag
    ("TMAG5273 A1/B1/C1/D1 bare chip (5 of each variant + spares)", 20, 1.6, "M"),
    ("MLX90393 QFN -014 bare chip", 5, 4.5, "M"),
    ("BMP581 bare chip", 14, 2.5, "M"),
    ("VL53L1X module (palm range)", 1, 4.0, "05 [bg]"),
    ("Diametric magnets Ø2 x 1.5 + Ø3 x 1 N45 (packs of 50)", 1, 12.0, "M"),
    ("Open full-ceramic ZrO2 MR84 4x8x2 (8) + MR63 (4), sensor side", 12, 4.5, "M"),
    ("Axial magnet Ø3 x 1 N42 (tips, pack 20)", 1, 4.0, "M"),
    ("Sensor boards: 21 x 4.6 mm joint, 5 tip, 14 pad (one JLCPCB 0.6 mm panel, 5 copies min)", 1, 18.0, "M (10-review: setup ~25-40 incl. assembly)"),
    ("RP2040-Zero", 1, 4.0, "14 V2"),
    ("Palm hub carrier PCB (or perfboard)", 1, 6.0, "M"),
    ("3.3 V LDO 300 mA (on the hub) + caps", 1, 1.0, "M"),
    ("SN65HVD / MAX3485 RS-485 pair (optional, if the UART is noisy)", 2, 1.5, "M"),
    ("38 AWG PTFE stranded wire, 4 colours x 5 m", 1, 12.0, "M"),
    ("6-core 36 AWG silicone cable 1 m (wrist link)", 1, 4.0, "M"),
    ("JST SH 1.0 4-pin pigtail pairs (fingers) + 6-pin (wrist)", 7, 0.6, "14 V2 / M"),
    ("Silicone (Dragon Skin 20 trial kit, shared with skins)", 0, 35.0, "05 (already budgeted)"),
    ("Brass rod Ø3 + 316 Ø2 pins (non-magnetic pins A/B, abd stubs)", 1, 5.0, "M"),
]
tot = sum(q * u for _, q, u, _ in BOM)
p("\nBOM (whole hand sensing)")
for n, q, u, s in BOM:
    p(f"  {q:3d} x {u:6.2f}  {q * u:7.2f}  {n}  [{s}]")
p(f"  TOTAL ~ EUR {tot:.0f}")
per_finger = 4 * 1.6 + 4.5 + 2 * 2.5 + 0.8 + 3.0 + 2.5
p(f"  per finger module (chips, magnets, boards, wire, plug): ~EUR {per_finger:.0f}")

json.dump(dict(sensors=sensors, timing_pct=timing, link=dict(frame_B=frame, rate_Hz=500, baud=baud, load=round(load, 3)),
               fatigue=fat, power_mA=round(i3 + i_mcu, 1), bom=[dict(item=n, qty=q, eur=u, src=s) for n, q, u, s in BOM],
               total_eur=round(tot)), open(OUT / "budget.json", "w"), indent=1)
(OUT / "budget.txt").write_text("\n".join(L) + "\n")
