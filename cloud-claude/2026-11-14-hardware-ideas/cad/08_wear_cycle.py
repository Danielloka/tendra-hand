"""(Round 2: --rock-id 2 also rocks the eye plate on servo 2: R8 test T6.)  Cycle servo 1 back and forth for the guide-wear test (R4).  NOT tested on hardware: check on a free servo first.
Needs: pip install pyserial.  Firmware hand_v1_servo (README protocol): 'M i n' = move servo i by n raw ticks (max +-300).
  python cad/08_wear_cycle.py --port COM5 --cycles 10000 --ticks 300
One cycle = +ticks, -ticks.  39 ticks = 1 mm of line on the 5 mm spool (195.6 ticks/rad).  The firmware limits speed
(<= 400 ticks/s), so 300 ticks takes ~0.75 s each way: expect ~0.6 cycles/s.  Stops on Ctrl+C or when the servo reports >80 % load.
"""
import argparse
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument("--port", required=True)
ap.add_argument("--cycles", type=int, default=10000)
ap.add_argument("--ticks", type=int, default=300)
ap.add_argument("--pause", type=float, default=0.9, help="seconds to wait after each move (>= ticks/400)")
ap.add_argument("--log", default="wear_cycles.txt")
ap.add_argument("--rock-id", type=int, default=0, help="eye-plate test: second servo ID that rocks the eye plate (0 = off)")
ap.add_argument("--rock-ticks", type=int, default=171, help="171 ticks = +-50 deg of servo angle (195.6 ticks/rad)")
a = ap.parse_args()
s = serial.Serial(a.port, 921600, timeout=1)
time.sleep(1.5)


def ask(cmd):
    s.write((cmd + "\n").encode())
    return s.readline().decode().strip()


print(ask("B"), ask("I"))
n = 0
try:
    while n < a.cycles:
        for sign in (1, -1):
            if a.rock_id:
                ask(f"M {a.rock_id} {sign * a.rock_ticks}")   # eye plate swings 50 deg one way, then 100 deg back (re-centres)
            r = ask(f"M 1 {sign * a.ticks}")
            if not r.startswith("OK"):
                raise SystemExit(f"servo refused: {r}")
            time.sleep(a.pause)
        n += 1
        if n % 100 == 0:
            f = ask("F").split()
            load = float(f[1].split(",")[1]) if len(f) > 1 else 0.0   # joint 1 = servo ID 1
            open(a.log, "w").write(f"{n}\n")
            print(n, "load %", load, flush=True)
            if abs(load) > 80:
                raise SystemExit("load > 80 %: stopping")
except KeyboardInterrupt:
    pass
ask("R")
print("done, cycles:", n)
