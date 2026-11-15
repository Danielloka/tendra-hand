"""Hall-sensor angle error from the R9 jig.  Plain Python 3.
CSV columns: variant,ref_deg,sensor_deg   (ref = protractor reading, sensor = chip angle, both in degrees)
  python cad/08_hall_error.py out/08/hall_runs.csv
For each variant it removes the best constant offset (any magnet position gives an arbitrary zero) and, separately,
offset + gain, then prints the worst error over the measured range.  Pass = worst error < 2 deg (offset removed)."""
import csv
import statistics as st
import sys

rows = {}
for r in csv.DictReader(open(sys.argv[1], newline="")):
    rows.setdefault(r["variant"], []).append((float(r["ref_deg"]), float(r["sensor_deg"])))
for v, pts in rows.items():
    pts.sort()
    ref = [p[0] for p in pts]
    sen = [p[1] for p in pts]
    for i in range(1, len(sen)):                      # unwrap 360 jumps
        while sen[i] - sen[i - 1] > 180:
            sen[i] -= 360
        while sen[i] - sen[i - 1] < -180:
            sen[i] += 360
    d = [s - r for s, r in zip(sen, ref)]
    off = st.mean(d)
    e1 = max(abs(x - off) for x in d)
    n = len(ref)
    mr, ms = st.mean(ref), st.mean(sen)
    g = sum((r - mr) * (s - ms) for r, s in zip(ref, sen)) / sum((r - mr) ** 2 for r in ref)
    e2 = max(abs(s - (g * (r - mr) + ms)) for r, s in zip(ref, sen))
    print(f"{v:36s} n={n:2d}  worst error (offset removed) {e1:5.2f} deg   (offset+gain removed, gain {g:.3f}) {e2:5.2f} deg   {'PASS' if e1 < 2 else 'FAIL'}")
