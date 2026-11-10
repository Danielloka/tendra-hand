"""Demo / self-test: python -m tools.demo  (from cad/) -> out/tools_demo/"""
import sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cadquery as cq
import numpy as np
from tools.render import render
from tools.checks import is_valid, interference, sweep, report
from tools.transform import rotate_about, arc_on_drum, tangent_point, check_sheath, polyline_length
from tools.export import export_all

OUT = Path(__file__).resolve().parents[2] / "out" / "tools_demo"

def build(q=0.0):
    base = cq.Workplane("XY").box(40, 20, 10).edges("|Z").fillet(3).translate((0, 0, 5))
    arm = cq.Workplane("XY").box(8, 8, 40).translate((0, 0, 30)).edges("|X").fillet(1.5)
    arm = rotate_about(arm, (0, 0, 20), (1, 0, 0), q)
    pin = cq.Workplane("YZ").circle(2).extrude(23).translate((-15, 0, 20))
    drum = cq.Workplane("YZ").circle(7).extrude(6).translate((-3, 0, 20))
    return {"base": base, "arm": arm, "pin": pin, "drum": drum,
            "overlap_post": cq.Workplane("XY").cylinder(25, 6).translate((14, 0, 17))}

if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    parts = build(0)
    for n, s in parts.items():
        print(n, {k: (round(v, 1) if isinstance(v, float) else v) for k, v in is_valid(s).items()})
    t = time.time()
    hits = interference(parts, ignore=[("pin", "arm"), ("pin", "drum"), ("drum", "arm")])
    print("interference:", hits, f"{time.time()-t:.2f}s")
    res = sweep(build, "q", [-30, 0, 30], ignore=[("pin", "arm"), ("pin", "drum"), ("drum", "arm")])
    report(res)
    t = time.time()
    render(parts, OUT / "demo.png", title="tools demo (post overlaps base)", colors={"overlap_post": "#d04040"})
    print(f"render {time.time()-t:.2f}s")
    render(parts, OUT / "demo_numpy.png", views=("iso", "front"), size=(1000, 500), backend="numpy")
    print(export_all(parts, OUT, "demo"))
    # transform helpers
    c, ax, r = (0, 0, 20), (1, 0, 0), 7.0
    p = tangent_point(c, ax, r, (0, -30, 40), +1)
    print("tangent", np.round(p, 2), "arc 90deg =", round(arc_on_drum(c, ax, r, (0, 7, 20), (0, 0, 27), sense=1), 3), "expect", round(np.pi * 7 / 2, 3))
    pts = [(0, 0, 0), (0, 10, 30), (0, 0, 60), (0, -10, 90)]
    print("sheath", check_sheath(pts, 15.0), "len", round(polyline_length(pts), 1))
