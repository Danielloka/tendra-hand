"""ASCII occupancy maps of the index finger in the joint-zero STEP.
usage: mapper.py STEP plane(xy|xz|zy) fixed_coord v1 v2 ... ; grid set in code"""
import sys, numpy as np
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID, TopAbs_IN
from OCP.TopExp import TopExp_Explorer
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
rd = STEPControl_Reader(); rd.ReadFile(sys.argv[1]); rd.TransferRoots()
S = []
e = TopExp_Explorer(rd.OneShape(), TopAbs_SOLID)
while e.More(): S.append(e.Current()); e.Next()
names = {0: "r", 1: "g", 2: "b", 6: "p", 7: "#"}  # red middle, green proximal, blue index base, purple tip, palm
cls = {k: BRepClass3d_SolidClassifier(S[k]) for k in names}
def who(p):
    for k, c in cls.items():
        c.Perform(gp_Pnt(*p), 1e-4)
        if c.State() == TopAbs_IN: return names[k]
    return "."
mode = sys.argv[2]
exec(sys.argv[3])  # defines: fixed values list F, ranges U (x-like), V (rows)
for f in F:
    print(f"\n--- {mode} at {f} ; columns {U[0]:.2f}..{U[-1]:.2f} step {U[1]-U[0]:.2f}")
    for v in V:
        row = "".join(who(P(f, u, v)) for u in U)
        print(f"{v:+7.2f} {row}")
