"""Prototype the PIP pinch point on the STEP geometry and verify it."""
import sys, math
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID, TopAbs_IN, TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeHalfSpace
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Cut, BRepAlgoAPI_Section
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.gp import gp_Pnt, gp_Dir, gp_Ax2, gp_Pln

rd = STEPControl_Reader(); rd.ReadFile(sys.argv[1]); rd.TransferRoots()
solids = []
e = TopExp_Explorer(rd.OneShape(), TopAbs_SOLID)
while e.More():
    solids.append(e.Current()); e.Next()
red, green = solids[0], solids[1]
d = np.array([0.994288, 0, 0.106731])
O = np.array([-12.7571, 20.2348, 118.8432])
MCP = np.array([-8.4898, 19.023, 79.0899])
DIP = np.array([-15.2084, 19.0036, 141.679])
al = DIP - O; al -= (al @ d) * d; al /= np.linalg.norm(al)
side = np.cross(al, d)
gal = O - MCP; gal -= (gal @ d) * d; gal /= np.linalg.norm(gal)
def W(x, y, z):  # red-frame (x along axis, y back, z along middle phalanx) -> world
    return O + x * d + y * side + z * al
P = lambda v: gp_Pnt(*map(float, v))
D = lambda v: gp_Dir(*map(float, v))

# parameters (mm)
X0, X1 = -1.6, -0.5      # slot is x -1.5..-0.6
Y0, Y1 = -7.45, 2.3      # palm .. back
Z1 = 6.5                 # plug ends where the slot has narrowed into the channel
TILT = math.radians(15)  # proximal face tilts back on the palm side (flexion clearance past 90 deg)
HX, HR = -1.05, 0.6      # hole centre x and radius (1.2 mm hole on the axis, y = 0)

box = BRepPrimAPI_MakeBox(gp_Ax2(P(W(X0, Y0, -3)), D(al), D(d)), X1 - X0, Y1 - Y0, Z1 + 3).Shape()
# keep only z >= -y*tan(TILT) (and z >= 0 for y > 0 comes from a second half space)
def halfspace_keep(normal, ref_inside):
    f = BRepBuilderAPI_MakeFace(gp_Pln(P(O), D(normal))).Face()
    return BRepPrimAPI_MakeHalfSpace(f, P(ref_inside)).Solid()
n1 = math.tan(TILT) * side + al          # plane z + y tan = 0
n2 = al                                   # plane z = 0
outside1 = W(0, 0, -5)                    # point to REMOVE
plug = BRepAlgoAPI_Cut(box, halfspace_keep(n1, outside1)).Shape()
plug = BRepAlgoAPI_Cut(plug, halfspace_keep(n2, W(0, 0, -5))).Shape()
hole = BRepPrimAPI_MakeCylinder(gp_Ax2(P(W(HX, 0, -1)), D(al)), HR, 10).Shape()
new_red = BRepAlgoAPI_Cut(BRepAlgoAPI_Fuse(red, plug).Shape(), hole).Shape()

def vol(s):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(s, g); return g.Mass()
print(f"red volume {vol(red):.1f} -> {vol(new_red):.1f} mm3 (plug adds {vol(new_red)-vol(red):.1f})")

# 1) plug must stay inside red's outer envelope: slot walls on both sides of every plug point
cr = BRepClass3d_SolidClassifier(red)
def inside(c, p):
    c.Perform(P(p), 1e-4); return c.State() == TopAbs_IN
bad = 0
for y in np.arange(Y0, Y1 + 0.01, 0.1):
    for z in np.arange(0.0, Z1 + 0.01, 0.1):
        if z < -y * math.tan(TILT):
            continue
        if not (inside(cr, W(X0 - 0.15, y, z)) and inside(cr, W(X1 + 0.15, y, z))):
            bad += 1
print("plug points not enclosed by red walls:", bad)

# 2) tendon path: proximal segment is fixed in green, distal in red. Check clearance over the range.
cn = BRepClass3d_SolidClassifier(new_red); cg = BRepClass3d_SolidClassifier(green)
def rot(v, axis, ang):  # Rodrigues
    k = axis / np.linalg.norm(axis)
    return v * math.cos(ang) + np.cross(k, v) * math.sin(ang) + k * (k @ v) * (1 - math.cos(ang))
hitg = sum(inside(cg, W(HX, 0, 0) - t * gal) for t in np.arange(0.2, 8.0, 0.1))
print("proximal tendon segment points inside green:", hitg)
flex_sign = None
for sg in (+1, -1):
    if (rot(DIP - O, d, sg * 0.3) - (DIP - O)) @ side < 0:
        flex_sign = sg
pip_now = flex_sign * math.degrees(math.atan2(np.cross(gal, al) @ d, gal @ al))
print(f"flexion = {'+' if flex_sign > 0 else '-'} rotation about d; PIP angle in STEP pose ~ {pip_now:.1f} deg")
def seg_hits(q_deg, classifier, pinch=W(HX, 0, 0)):
    g = rot(gal, d, -flex_sign * math.radians(q_deg - pip_now))  # green direction seen from red
    return sum(inside(classifier, pinch - t * g) for t in np.arange(0.3, 8.0, 0.1))
print("PIP angle -> proximal tendon points inside modified red (want 0):")
for q in [-5, 0, 30, 60, 90, 95, 100]:
    print(f"  {q:4d} deg: {seg_hits(q, cn)}")
# distal segment: from pinch along red channel, must be clear in new red
print("distal segment points inside new red (want 0):", sum(inside(cn, W(HX, 0, z)) for z in np.arange(0.0, 12, 0.1)))

# length change: tendon from green channel exit to red channel point, via pinch (new) vs straight (old)
A_g = W(HX, 0, 0) - 8.3 * gal   # green channel exit, in STEP pose (fixed in green)
B_r = W(HX, 0, 8.0)             # red channel point (fixed in red)
print("tendon length vs PIP angle: new (via pinch point) / old (straight line if unobstructed)")
for q in [0, 30, 60, 90]:
    g = rot(gal, d, -flex_sign * math.radians(q - pip_now))
    A = W(HX, 0, 0) - 8.3 * g
    new = np.linalg.norm(A - W(HX, 0, 0)) + np.linalg.norm(B_r - W(HX, 0, 0))
    old = np.linalg.norm(B_r - A)
    print(f"  {q:3d} deg: new {new:.2f}  old-straight {old:.2f}")

# before/after picture: section at the hole centre plane (normal = joint axis)
fig, axs = plt.subplots(1, 2, figsize=(14, 6))
for ax, shp, title in [(axs[0], red, "BEFORE: slot open to the palm"), (axs[1], new_red, "AFTER: plug + 1.2 mm hole on the axis")]:
    sec = BRepAlgoAPI_Section(shp, gp_Pln(P(W(HX, 0, 0)), D(d))); sec.Build()
    ex = TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
    while ex.More():
        c = BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
        dd = GCPnts_QuasiUniformDeflection(c, 0.02)
        pts = np.array([[dd.Value(i).X(), dd.Value(i).Y(), dd.Value(i).Z()] for i in range(1, dd.NbPoints() + 1)])
        if len(pts):
            q = pts - O
            ax.plot(q @ al, q @ side, color="#d62728", lw=1.2)
        ex.Next()
    ax.plot(0, 0, "k+", ms=16); ax.annotate("PIP axis", (0, 0), xytext=(-40, 12), textcoords="offset points")
    ax.set_xlim(-10, 16); ax.set_ylim(-10, 10); ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("mm along middle segment (toward fingertip)"); ax.set_ylabel("mm (up = back of hand, down = palm)")
    ax.set_title(title)
fig.tight_layout(); fig.savefig("pip_pinch_before_after.png", dpi=90)
print("saved picture")
