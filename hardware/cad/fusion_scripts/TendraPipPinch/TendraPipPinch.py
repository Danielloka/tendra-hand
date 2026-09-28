"""Tendra Hand: add a tendon pinch point on the index PIP axis.

The fingertip (DIP) tendon passes through the PIP joint in a slot that is open toward the palm, so
when the PIP bends the tendon cuts the corner and changes length (joint coupling). This script fills
that slot in the middle segment and leaves a small hole whose entrance sits exactly on the PIP axis.
A tendon held on the axis keeps the same length at every joint angle.

Geometry comes from the STEP export of 2026-09-27 (hardware/cad/Hand assebly.step), world mm, and was
checked there with OpenCASCADE: the plug stays inside the segment and the tendon path is clear from
-5 to 100 degrees. The script checks that the open design matches that geometry before changing
anything, asks for confirmation, and adds one base feature and one combine (join) feature.
"""

import math
import traceback

import adsk.core
import adsk.fusion

# ---- PIP joint frame, from the STEP export (world, mm) ----
AXIS_DIR = (0.994288, 0.0, 0.106731)  # PIP joint axis
AXIS_PT = (-12.7571, 20.2348, 118.8432)  # point on the PIP axis
DIP_PT = (-15.2084, 19.0036, 141.679)  # point on the DIP axis (defines the middle segment direction)
BOSS_RADIUS = 1.95  # the middle segment's axle bosses at the PIP

# ---- Plug and hole, in the PIP frame: x along the axis, y toward the back, z along the segment ----
X0, X1 = -1.6, -0.5  # slot is x -1.5 .. -0.6; overlap the walls a little
Y0, Y1 = -7.45, 2.3  # palm side .. back side
Z1 = 6.5  # plug ends where the slot has narrowed into the tendon channel
TILT_DEG = 15.0  # proximal face leans back on the palm side: clearance past 90 deg of flexion
HOLE_X, HOLE_D = -1.05, 1.2  # hole centre on the slot centre line, y = 0 (on the axis)
HOLE_Z0, HOLE_Z1 = -1.0, 9.0  # hole runs from before the axis into the channel

FEATURE_NAME = "Tendra PIP pinch"


def vec(a):
    return adsk.core.Vector3D.create(*a)


def unit(v):
    v = v.copy()
    v.normalize()
    return v


def cross(a, b):
    return a.crossProduct(b)


def frame():
    """Return (origin_mm, x_axis, y_back, z_along) as plain tuples/Vector3D."""
    d = unit(vec(AXIS_DIR))
    al = vec(tuple(DIP_PT[i] - AXIS_PT[i] for i in range(3)))
    proj = d.copy()
    proj.scaleBy(al.dotProduct(d))
    al.subtract(proj)
    al = unit(al)
    side = unit(cross(al, d))
    return AXIS_PT, d, side, al


def world_pt(x, y, z):
    """PIP-frame mm -> Fusion world Point3D (cm)."""
    o, d, side, al = frame()
    p = [o[i] + x * d.asArray()[i] + y * side.asArray()[i] + z * al.asArray()[i] for i in range(3)]
    return adsk.core.Point3D.create(p[0] / 10, p[1] / 10, p[2] / 10)


def box(center_pt, length_dir, width_dir, length_mm, width_mm, height_mm):
    obb = adsk.core.OrientedBoundingBox3D.create(
        center_pt, length_dir, width_dir, length_mm / 10, width_mm / 10, height_mm / 10
    )
    return adsk.fusion.TemporaryBRepManager.get().createBox(obb)


def build_tool_body():
    """Plug minus the cut-back regions minus the hole, as a temporary body in world coordinates."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    _, d, side, al = frame()
    diff = adsk.fusion.BooleanTypes.DifferenceBooleanType

    zc = (Z1 - 3.0) / 2  # plug box spans z -3 .. Z1 before trimming
    plug = box(world_pt((X0 + X1) / 2, (Y0 + Y1) / 2, zc), d, side, X1 - X0, Y1 - Y0, Z1 + 3.0)

    # Remove z < 0 (keeps the proximal side open for the tendon to swing).
    big = 60.0
    tbm.booleanOperation(plug, box(world_pt(0, 0, -big / 2), d, side, big, big, big), diff)

    # Remove the palm-side wedge z < -y*tan(tilt): a big box whose face is that tilted plane.
    t = math.tan(math.radians(TILT_DEG))
    n = unit(vec(tuple(t * side.asArray()[i] + al.asArray()[i] for i in range(3))))
    w = unit(cross(n, d))
    c = [-(big / 2) * n.asArray()[i] for i in range(3)]  # centre in frame-free world offset (mm)
    o = world_pt(0, 0, 0)
    center = adsk.core.Point3D.create(o.x + c[0] / 10, o.y + c[1] / 10, o.z + c[2] / 10)
    tbm.booleanOperation(plug, box(center, d, w, big, big, big), diff)

    hole = tbm.createCylinderOrCone(
        world_pt(HOLE_X, 0, HOLE_Z0), HOLE_D / 20, world_pt(HOLE_X, 0, HOLE_Z1), HOLE_D / 20
    )
    tbm.booleanOperation(plug, hole, diff)
    return plug


def all_bodies(root):
    for b in root.bRepBodies:
        yield b, None
    for occ in root.allOccurrences:
        for b in occ.bRepBodies:  # proxies in world coordinates
            yield b, occ


def contains(body, pt):
    return body.pointContainment(pt) == adsk.fusion.PointContainment.PointInsidePointContainment


def has_pip_boss(body):
    """True if the body has a cylindrical face of the boss radius on the PIP axis (tolerance 0.05 mm)."""
    d = unit(vec(AXIS_DIR))
    for face in body.faces:
        if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
            continue
        cyl = adsk.core.Cylinder.cast(face.geometry)
        if abs(cyl.radius * 10 - BOSS_RADIUS) > 0.02:
            continue
        if abs(abs(unit(cyl.axis).dotProduct(d)) - 1) > 1e-4:
            continue
        rel = vec(tuple(AXIS_PT[i] - cyl.origin.asArray()[i] * 10 for i in range(3)))
        if cross(rel, d).length < 0.05:
            return True
    return False


def find_middle_segment(root):
    """The body that has the PIP boss, solid tongue material at two probe points, and an empty slot."""
    tongue_a, tongue_b = world_pt(-4.0, 0, 3), world_pt(0.2, 0, 3)
    slot = world_pt(HOLE_X, -3.0, 3)
    matches = []
    for body, occ in all_bodies(root):
        if body.isSolid and contains(body, tongue_a) and contains(body, tongue_b) and has_pip_boss(body):
            matches.append((body, occ))
    if len(matches) != 1:
        return None, None, f"expected 1 matching middle segment body, found {len(matches)}"
    body, occ = matches[0]
    if contains(body, slot):
        return None, None, "the slot is already filled (was this script run before?)"
    return body, occ, None


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Open the hand design first (Design workspace), then run this script.")
            return

        body, occ, problem = find_middle_segment(design.rootComponent)
        if problem:
            ui.messageBox(
                "Nothing was changed.\n\nThe open design doesn't match the geometry this script was made "
                f"for: {problem}.\n\nThe index finger must be in the same position as in the STEP export."
            )
            return

        answer = ui.messageBox(
            f"Found the index middle segment: '{body.name}'"
            + (f" in '{occ.name}'" if occ else "")
            + ".\n\nThis fills the fingertip tendon slot at the PIP joint and adds a "
            f"{HOLE_D} mm hole on the joint axis.\n\nHave you saved a copy of the design? Continue?",
            "Tendra PIP pinch",
            adsk.core.MessageBoxButtonTypes.YesNoButtonType,
            adsk.core.MessageBoxIconTypes.QuestionIconType,
        )
        if answer != adsk.core.DialogResults.DialogYes:
            ui.messageBox("Cancelled. Nothing was changed.")
            return

        # Check points in the component's own coordinates. Adding features makes Fusion re-solve the
        # joints, which can move the parts, so the result is checked on the native body, not by world position.
        probe_plug, probe_hole = world_pt(HOLE_X, -3.0, 3), world_pt(HOLE_X, 0, 3)
        tool = build_tool_body()
        native = body.nativeObject if occ else body
        comp = native.parentComponent
        if occ:  # world -> component coordinates
            m = occ.transform2.copy()
            m.invert()
            adsk.fusion.TemporaryBRepManager.get().transform(tool, m)
            probe_plug.transformBy(m)
            probe_hole.transformBy(m)

        parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
        if parametric:
            base = comp.features.baseFeatures.add()
            base.name = FEATURE_NAME + " plug"
            base.startEdit()
            tool_body = comp.bRepBodies.add(tool, base)
            base.finishEdit()
            tool_body = base.bodies.item(0)
        else:
            tool_body = comp.bRepBodies.add(tool)

        tools = adsk.core.ObjectCollection.create()
        tools.add(tool_body)
        comb_in = comp.features.combineFeatures.createInput(native, tools)
        comb_in.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        comb_in.isKeepToolBodies = False
        comb = comp.features.combineFeatures.add(comb_in)
        if parametric:
            comb.name = FEATURE_NAME + " join"

        result = comb.bodies.item(0) if comb.bodies.count else native
        ok_plug = contains(result, probe_plug)
        ok_hole = not contains(result, probe_hole)
        ui.messageBox(
            "Done.\n\n"
            f"Slot filled: {'yes' if ok_plug else 'NO'}\n"
            f"Hole on the axis open: {'yes' if ok_hole else 'NO'}\n\n"
            "Undo: delete the two 'Tendra PIP pinch' features at the end of the timeline."
        )
    except Exception:
        if ui:
            ui.messageBox("TendraPipPinch failed:\n" + traceback.format_exc())
