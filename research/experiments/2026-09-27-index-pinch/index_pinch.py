"""Prototype + verify the MCP-flex and MCP-abduction pinch points on the joint-zero STEP (world mm)."""
import math
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Section
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeHalfSpace
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.gp import gp_Ax2, gp_Dir, gp_Pln, gp_Pnt
from OCP.GProp import GProp_GProps
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_EDGE, TopAbs_IN, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

rd = STEPControl_Reader()
rd.ReadFile(sys.argv[1])
rd.TransferRoots()
S = []
e = TopExp_Explorer(rd.OneShape(), TopAbs_SOLID)
while e.More():
    S.append(e.Current())
    e.Next()
red, green, blue, palm = S[0], S[1], S[2], S[7]


def P(v):
    return gp_Pnt(*map(float, v))


def D(v):
    return gp_Dir(*map(float, v))


Y0 = 19.023
ZM = 78.9764
ZA = 59.9764
XA = -8.5306


def vol(s):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(s, g)
    return g.Mass()


def wbox(x0, x1, y0, y1, z0, z1):
    return BRepPrimAPI_MakeBox(P((x0, y0, z0)), P((x1, y1, z1))).Shape()


def ins(s, p):
    c = BRepClass3d_SolidClassifier(s)
    c.Perform(P(p), 1e-4)
    return c.State() == TopAbs_IN


def rot(v, k, a):
    k = np.asarray(k, float)
    k = k / np.linalg.norm(k)
    return v * math.cos(a) + np.cross(k, v) * math.sin(a) + k * (k @ v) * (1 - math.cos(a))


def cut_half(shape, normal, remove_pt, origin):
    f = BRepBuilderAPI_MakeFace(gp_Pln(P(origin), D(normal))).Face()
    return BRepAlgoAPI_Cut(shape, BRepPrimAPI_MakeHalfSpace(f, P(remove_pt)).Solid()).Shape()


# ---------- MCP flex pinch (green) ----------
M = dict(X0=-8.13, X1=-6.94, Y0=-7.45, Y1=4.3, Z1=6.5, TILT=15.0, HX=-7.535, HD=1.4, HZ0=-1.0, HZ1=9.5)
box = wbox(M["X0"], M["X1"], Y0 + M["Y0"], Y0 + M["Y1"], ZM - 3, ZM + M["Z1"])
t = math.tan(math.radians(M["TILT"]))
O_m = (0, Y0, ZM)
plug = cut_half(box, (0, 0, 1), (0, Y0, ZM - 5), O_m)
plug = cut_half(plug, (0, t, 1), (0, Y0, ZM - 5), O_m)
hole = BRepPrimAPI_MakeCylinder(
    gp_Ax2(P((M["HX"], Y0, ZM + M["HZ0"])), D((0, 0, 1))), M["HD"] / 2, M["HZ1"] - M["HZ0"]
).Shape()
green_new = BRepAlgoAPI_Cut(BRepAlgoAPI_Fuse(green, plug).Shape(), hole).Shape()
print(f"MCP: green volume {vol(green):.1f} -> {vol(green_new):.1f} (+{vol(green_new) - vol(green):.1f} mm3)")
bad = 0
for y in np.arange(M["Y0"], M["Y1"] + 1e-6, 0.1):
    for z in np.arange(0, M["Z1"] + 1e-6, 0.1):
        if z < -y * t:
            continue
        if not (ins(green, (M["X0"] - 0.1, Y0 + y, ZM + z)) and ins(green, (M["X1"] + 0.1, Y0 + y, ZM + z))):
            bad += 1
print("MCP: plug points not enclosed by tongue walls:", bad)
H = np.array([M["HX"], Y0, ZM])
blue_ch = np.array([M["HX"], Y0 + 0.5, ZM - 9.0])  # blue channel (-8..-7, y -1.5..3.5) at MCP-9
print("MCP: proximal segment inside blue:", sum(ins(blue, H + s * (blue_ch - H)) for s in np.linspace(0.03, 1, 60)))
print(
    "MCP: hole centre line inside new green (want 0):",
    sum(ins(green_new, (M["HX"], Y0, ZM + z)) for z in np.arange(0.05, 9.4, 0.1)),
)
flexdir = +1 if rot(np.array([0, 0, 10.0]), (1, 0, 0), 0.3)[1] < 0 else -1
print("MCP: flex angle -> proximal segment points inside new green (want 0):")
for q in [-5, 0, 30, 60, 90, 95, 100]:
    pts = [H + rot(s * (blue_ch - H), (1, 0, 0), -flexdir * math.radians(q)) for s in np.linspace(0.03, 1, 60)]
    print(f"   {q:4d}: {sum(ins(green_new, p) for p in pts)}")
wall_side = -6.03 - (M["HX"] + M["HD"] / 2)
wall_groove = (M["HX"] - M["HD"] / 2) - (-9.03)
print(f"MCP: wall hole->tongue side {wall_side:.2f} mm, hole->drum groove {wall_groove:.2f} mm")

# ---------- ABD pinch: two blocks in blue leaving a 1 mm vertical slit on the axis ----------
A = dict(SLIT=1.0, XL=-11.38, XR=-5.68, YB=-1.0, YT=3.0, ZB=-0.2, ZT=1.0)
left = wbox(A["XL"], XA - A["SLIT"] / 2, Y0 + A["YB"], Y0 + A["YT"], ZA + A["ZB"], ZA + A["ZT"])
right = wbox(XA + A["SLIT"] / 2, A["XR"], Y0 + A["YB"], Y0 + A["YT"], ZA + A["ZB"], ZA + A["ZT"])
blue_new = BRepAlgoAPI_Fuse(BRepAlgoAPI_Fuse(blue, left).Shape(), right).Shape()
print(f"\nABD: blue volume {vol(blue):.1f} -> {vol(blue_new):.1f} (+{vol(blue_new) - vol(blue):.1f} mm3)")
bad = 0
for x in np.arange(A["XL"], A["XR"], 0.1):
    if abs(x - XA) < A["SLIT"] / 2:
        continue
    for z in np.arange(A["ZB"], A["ZT"] + 1e-6, 0.1):
        if not (ins(blue, (x, Y0 + A["YB"] - 0.05, ZA + z)) and ins(blue, (x, Y0 + A["YT"] + 0.05, ZA + z))):
            bad += 1
for y in np.arange(A["YB"], A["YT"] + 1e-6, 0.1):
    for z in np.arange(A["ZB"], A["ZT"] + 1e-6, 0.1):
        if not (ins(blue, (A["XL"] - 0.05, Y0 + y, ZA + z)) and ins(blue, (A["XR"] + 0.05, Y0 + y, ZA + z))):
            bad += 1
print("ABD: block points not enclosed by blue slot walls:", bad)
palm_exits = {"left": (-11.9, 1.25), "centre": (-8.5, 1.25), "right": (-5.25, 1.25)}
blue_exits = {"MCP loop": (-9.5, 0.0), "PIP+DIP loops": (-7.5, 0.5)}
axis_pt = np.array([XA, 0, ZA])
for yk in [0.0, 1.0, 2.0]:
    Sp = np.array([XA, Y0 + yk, ZA])
    for nm, (px, py) in palm_exits.items():
        for q in [-15, 0, 15]:
            pe = axis_pt + rot(np.array([px, Y0 + py, ZA - 10.0]) - axis_pt, (0, 1, 0), math.radians(-q))
            hits = sum(ins(blue_new, Sp + s * (pe - Sp)) for s in np.linspace(0.02, 0.6, 40))
            if hits:
                print(f"ABD: strand y={yk} from {nm} palm channel at {q:+} deg: {hits} points inside new blue")
    for nm, (bx, by) in blue_exits.items():
        be = np.array([bx, Y0 + by, ZM - 9.0])
        hits = sum(ins(blue_new, Sp + s * (be - Sp)) for s in np.linspace(0.02, 0.98, 60))
        if hits:
            print(f"ABD: strand y={yk} to {nm} blue channel: {hits} points inside new blue")
print("ABD: path checks done (only collisions are printed)")
for nm, (px, py) in palm_exits.items():
    bx, by = (-9.5, 0.0) if nm == "left" else (-7.5, 0.5)
    be = np.array([bx, Y0 + by, ZM - 9.0])
    L = []
    for q in [-15, 0, 15]:
        pe = axis_pt + rot(np.array([px, Y0 + py, ZA - 10.0]) - axis_pt, (0, 1, 0), math.radians(-q))
        Sp = np.array([XA, Y0 + 1, ZA])
        L.append((np.linalg.norm(be - pe), np.linalg.norm(Sp - pe) + np.linalg.norm(be - Sp)))
    print(
        f"ABD {nm:6s}: old straight {L[0][0] - L[1][0]:+.2f}/{L[2][0] - L[1][0]:+.2f} mm at -15/+15 deg; "
        f"new via slit {L[0][1] - L[1][1]:+.2f}/{L[2][1] - L[1][1]:+.2f} mm"
    )


def draw(ax, shp, pln, O, u, v, col):
    sec = BRepAlgoAPI_Section(shp, pln)
    sec.Build()
    ex = TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
    while ex.More():
        c = BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
        dd = GCPnts_QuasiUniformDeflection(c, 0.02)
        pts = np.array([[dd.Value(i).X(), dd.Value(i).Y(), dd.Value(i).Z()] for i in range(1, dd.NbPoints() + 1)])
        if len(pts):
            q = pts - O
            ax.plot(q @ u, q @ v, color=col, lw=1.2)
        ex.Next()


fig, axs = plt.subplots(2, 2, figsize=(14, 12))
for ax, shp, title in [
    (axs[0, 0], green, "MCP BEFORE (side cut through the slot)"),
    (axs[0, 1], green_new, "MCP AFTER: plug + 1.4 mm hole on the axis"),
]:
    draw(ax, shp, gp_Pln(P((M["HX"], 0, 0)), D((1, 0, 0))), np.array([M["HX"], Y0, ZM]),
         np.array([0, 0, 1.0]), np.array([0, 1.0, 0]), "#2ca02c")
    ax.plot(0, 0, "k+", ms=16)
    ax.set_xlim(-10, 16); ax.set_ylim(-10, 10); ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_title(title); ax.set_xlabel("mm toward fingertip"); ax.set_ylabel("mm (up = back of hand)")
for ax, shp, title in [
    (axs[1, 0], blue, "SIDEWAYS JOINT BEFORE (top-down cut)"),
    (axs[1, 1], blue_new, "SIDEWAYS JOINT AFTER: 1 mm slit on the axis"),
]:
    draw(ax, shp, gp_Pln(P((0, Y0 + 1.0, 0)), D((0, 1, 0))), np.array([XA, Y0, ZA]),
         np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), "#1f77b4")
    ax.plot(0, 0, "k+", ms=16)
    ax.set_xlim(-10, 16); ax.set_ylim(-8, 8); ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_title(title); ax.set_xlabel("mm toward fingertip"); ax.set_ylabel("mm sideways")
fig.tight_layout()
fig.savefig("index_pinch_before_after.png", dpi=80)
print("picture saved")
