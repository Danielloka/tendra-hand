"""08 bench-kit calculator.  Plain Python 3, no packages.

  python cad/08_capstan_mu.py mu --hold-N 49.0 --pull-N 91.0 --turns 0.5     # capstan friction (T3)
  python cad/08_capstan_mu.py mu --hold-kg 5 --pull-kg 9.3 --degrees 180
  python cad/08_capstan_mu.py csv out/08/mu_runs.csv                          # columns: variant,hold_N,pull_N,degrees
  python cad/08_capstan_mu.py stretch --force 45 --length 120 --delta 1.1     # line stretch (T2)
  python cad/08_capstan_mu.py housing --load 45 --l1 35 --d1 0.30 --l2 90 --d2 0.52 --bare 0.10   # sheath compression (T1)
  python cad/08_capstan_mu.py pinch --eta 0.9                              # eta -> SCS pinch at 80 % cap (T7)
  python cad/08_capstan_mu.py hold --force 3.0 --arm 60                    # servo torque vs 80 % back-off (T8)
  python cad/08_capstan_mu.py chain --mu 0.10 --bends 90,90,45               # efficiency of a routed strand
Formula: pull = hold * exp(mu * theta)  ->  mu = ln(pull/hold) / theta.   efficiency = exp(-mu * theta).
"""
import argparse
import csv
import math
import statistics as st

G = 9.81


def mu_of(hold, pull, theta):
    return math.log(pull / hold) / theta


def cmd_mu(a):
    hold = a.hold_N if a.hold_N else a.hold_kg * G
    pull = a.pull_N if a.pull_N else a.pull_kg * G
    th = a.turns * 2 * math.pi if a.turns else math.radians(a.degrees)
    mu = mu_of(hold, pull, th)
    print(f"hold {hold:.1f} N  pull {pull:.1f} N  wrap {math.degrees(th):.0f} deg  ->  mu = {mu:.3f}")
    print(f"efficiency over this wrap = {hold / pull:.2f}   (per 90 deg: {math.exp(-mu * math.pi / 2):.2f})")
    print("PASS (mu <= 0.12)" if mu <= 0.12 else "above 0.12: check guide material / try a roller")


def cmd_csv(a):
    groups = {}
    with open(a.file, newline="") as f:
        for r in csv.DictReader(f):
            groups.setdefault(r["variant"], []).append(mu_of(float(r["hold_N"]), float(r["pull_N"]), math.radians(float(r["degrees"]))))
    for v, mus in groups.items():
        sd = st.stdev(mus) if len(mus) > 1 else 0.0
        print(f"{v:28s} n={len(mus)}  mu mean {st.mean(mus):.3f}  sd {sd:.3f}  min {min(mus):.3f}  max {max(mus):.3f}")


def cmd_stretch(a):
    strain = a.delta / a.length
    print(f"strain {100 * strain:.2f} %   EA = F*L/dL = {a.force / strain:.0f} N")
    print("PASS (<= 1.0 % at the test load)" if strain <= 0.01 else "stretchy: joint angle sensors needed / try Vectran")
    print(f"At a 6 mm drum, 300 mm of free line at this load gives about {math.degrees(300 * strain / 6):.1f} deg of joint error")


def cmd_housing(a):
    e1, e2 = a.d1 - a.bare, a.d2 - a.bare
    c = (e2 - e1) / (a.l2 - a.l1)
    e0 = e1 - c * a.l1
    for L in (140, 300):
        print(f"predicted extra length change at {L} mm housing, {a.load:.0f} N: {e0 + c * L:.2f} mm   (per mm of housing {c * 1000:.2f} um/mm, end seating {e0:.2f} mm)")
    print("PASS (<= 0.6 mm at 140 mm)" if e0 + c * 140 <= 0.6 else "KILL for this sheath: use bare strands over guides")


def cmd_chain(a):
    eta = 1.0
    for b in a.bends.split(","):
        eta *= math.exp(-a.mu * math.radians(float(b)))
    print(f"total wrap {sum(float(b) for b in a.bends.split(',')):.0f} deg, mu {a.mu}: efficiency {eta:.2f}")


def cmd_pinch(a):
    """Round 2: tie a measured eta to the budget number it protects (review #10).
    Budget: SCS0009 pinch 6.7 N at stall with eta 0.95 (report 12 / review fix #1); firmware caps at 80 % load."""
    pinch = 6.7 * 0.80 * a.eta / 0.95
    print(f"eta {a.eta:.2f} -> SCS0009 pinch at the 80 % cap = {pinch:.1f} N   (bench-finger pass >= 4.5 N, kill < 3.5 N)")
    print("PASS" if pinch >= 4.5 else ("MARGINAL: re-check eta" if pinch >= 3.5 else "KILL: fix bearings/drum"))


def cmd_hold(a):
    """SCS0009 holding torque on the R5 lever.  Stall 0.226 N m at 6 V; it backs off after 2 s above 80 % load."""
    t = a.force * a.arm / 1000
    print(f"{a.force:.2f} N at {a.arm:.0f} mm = {t:.3f} N m = {100 * t / 0.226:.0f} % of the datasheet stall (0.226)")
    print(f"usable hold limit (80 pct) = {0.8 * 0.226:.3f} N m = {0.8 * 0.226 * 1000 / a.arm:.2f} N at this arm")


p = argparse.ArgumentParser()
s = p.add_subparsers(dest="c", required=True)
m = s.add_parser("mu"); m.add_argument("--hold-N", type=float); m.add_argument("--hold-kg", type=float)
m.add_argument("--pull-N", type=float); m.add_argument("--pull-kg", type=float)
m.add_argument("--turns", type=float); m.add_argument("--degrees", type=float); m.set_defaults(f=cmd_mu)
c = s.add_parser("csv"); c.add_argument("file"); c.set_defaults(f=cmd_csv)
t = s.add_parser("stretch"); t.add_argument("--force", type=float, required=True); t.add_argument("--length", type=float, required=True)
t.add_argument("--delta", type=float, required=True); t.set_defaults(f=cmd_stretch)
h = s.add_parser("housing")
for k in ("load", "l1", "d1", "l2", "d2", "bare"):
    h.add_argument("--" + k, type=float, required=True)
h.set_defaults(f=cmd_housing)
pc = s.add_parser("pinch"); pc.add_argument("--eta", type=float, required=True); pc.set_defaults(f=cmd_pinch)
hd = s.add_parser("hold"); hd.add_argument("--force", type=float, required=True); hd.add_argument("--arm", type=float, default=60); hd.set_defaults(f=cmd_hold)
ch = s.add_parser("chain"); ch.add_argument("--mu", type=float, required=True); ch.add_argument("--bends", required=True); ch.set_defaults(f=cmd_chain)
a = p.parse_args()
a.f(a)
