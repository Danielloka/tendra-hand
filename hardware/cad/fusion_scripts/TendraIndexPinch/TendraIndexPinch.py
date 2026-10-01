"""Tendra Hand: tendon pinch points at the index MCP (knuckle), flex and sideways axes.

Together with TendraPipPinch this makes every tendon that passes through an index joint cross it on
the joint axis, so its length no longer depends on that joint's angle (no joint coupling).

- MCP flex (proximal segment): fill the pass-through slot, which is open toward the palm, and leave a
  1.4 mm hole on the flex axis for the PIP and DIP loops (4 strands).
- MCP sideways / abduction (index base): add two blocks in the wide horizontal tendon slot, leaving a
  1 mm vertical slit on the sideways axis. All 6 strands stack vertically in it, which is on the axis.

Geometry is from the STEP export with all joints at 0 (2026-09-27), world mm, checked with
OpenCASCADE (research/experiments/2026-09-27-index-pinch/). The script checks that the open design
matches before changing anything, asks for confirmation, and adds a base feature and a combine
(join) feature per change. The index finger must be straight (all joints at 0).
"""

import math
import traceback

import adsk.core
import adsk.fusion

Y0 = 19.023  # height of the MCP flex axis (and the finger's tendon line)
ZM = 78.9764  # MCP flex axis, along the finger
ZA = 59.9764  # sideways axis, along the finger
XA = -8.5306  # sideways axis, across the finger
BIG = 60.0  # size of the boxes used to trim the plug, mm


def pt(x, y, z):
    return adsk.core.Point3D.create(x / 10, y / 10, z / 10)  # mm -> Fusion's cm


def vec(x, y, z):
    v = adsk.core.Vector3D.create(x, y, z)
    v.normalize()
    return v


def world_box(x0, x1, y0, y1, z0, z1):
    obb = adsk.core.OrientedBoundingBox3D.create(
        pt((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
        vec(1, 0, 0),
        vec(0, 1, 0),
        (x1 - x0) / 10,
        (y1 - y0) / 10,
        (z1 - z0) / 10,
    )
    return adsk.fusion.TemporaryBRepManager.get().createBox(obb)


def mcp_tool():
    """Plug for the MCP pass-through slot, minus the proximal side, minus a tilted palm wedge, minus the hole."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    diff = adsk.fusion.BooleanTypes.DifferenceBooleanType
    plug = world_box(-8.13, -6.94, Y0 - 7.45, Y0 + 4.3, ZM - 3.0, ZM + 6.5)
    tbm.booleanOperation(
        plug, world_box(-BIG, BIG, -BIG, BIG, ZM - BIG, ZM), diff
    )  # keep z >= axis
    # Keep z' >= -y' * tan(15 deg) (z', y' relative to the axis): clearance past 90 deg of flexion.
    t = math.tan(math.radians(15.0))
    n = vec(0, t, 1)
    w = n.crossProduct(vec(1, 0, 0))  # so that length x width = n (the box's height direction)
    c = [0, Y0 - n.y * BIG / 2, ZM - n.z * BIG / 2]
    obb = adsk.core.OrientedBoundingBox3D.create(
        pt(-8.0, c[1], c[2]), vec(1, 0, 0), w, BIG / 10, BIG / 10, BIG / 10
    )
    tbm.booleanOperation(plug, tbm.createBox(obb), diff)
    hole = tbm.createCylinderOrCone(
        pt(-7.535, Y0, ZM - 1.0), 0.07, pt(-7.535, Y0, ZM + 9.5), 0.07
    )  # 1.4 mm
    tbm.booleanOperation(plug, hole, diff)
    return [plug]


def abd_tool():
    """Two blocks leaving a 1 mm vertical slit centred on the sideways axis."""
    left = world_box(-11.38, XA - 0.5, Y0 - 1.0, Y0 + 3.0, ZA - 0.2, ZA + 1.0)
    right = world_box(XA + 0.5, -5.68, Y0 - 1.0, Y0 + 3.0, ZA - 0.2, ZA + 1.0)
    return [left, right]


PINCHES = [
    {
        "name": "Tendra MCP pinch",
        "label": "knuckle bend (MCP flex), proximal segment: 1.4 mm hole on the axis",
        "boss": (1.98, (1, 0, 0), (0.0, Y0, ZM)),  # radius, axis direction, point on the axis
        "solid": [(-10.5, Y0, ZM + 3), (-6.5, Y0, ZM + 3)],  # must be inside the body
        "empty_before": (-7.535, Y0 - 3.0, ZM + 3),  # the slot: empty now, filled after
        "empty_after": (-7.535, Y0, ZM + 3),  # the hole: must stay empty
        "tool": mcp_tool,
    },
    {
        "name": "Tendra ABD pinch",
        "label": "knuckle sideways (MCP abduction), index base: 1 mm slit on the axis",
        "boss": (1.98, (0, 1, 0), (XA, 0.0, ZA)),
        "solid": [(-12.5, Y0, ZA), (-5.0, Y0, ZA)],
        "empty_before": (-10.0, Y0 + 1.0, ZA + 0.4),
        "empty_after": (XA, Y0 + 1.0, ZA + 0.4),
        "tool": abd_tool,
    },
]


def all_bodies(root):
    for b in root.bRepBodies:
        yield b, None
    for occ in root.allOccurrences:
        for b in occ.bRepBodies:  # proxies: world coordinates
            yield b, occ


def contains(body, p):
    return body.pointContainment(p) == adsk.fusion.PointContainment.PointInsidePointContainment


def has_boss(body, radius, direction, on_axis):
    d = vec(*direction)
    for face in body.faces:
        if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
            continue
        cyl = adsk.core.Cylinder.cast(face.geometry)
        if abs(cyl.radius * 10 - radius) > 0.02:
            continue
        axis = cyl.axis.copy()
        axis.normalize()
        if abs(abs(axis.dotProduct(d)) - 1) > 1e-4:
            continue
        o = cyl.origin
        rel = adsk.core.Vector3D.create(
            on_axis[0] - o.x * 10, on_axis[1] - o.y * 10, on_axis[2] - o.z * 10
        )
        if rel.crossProduct(d).length < 0.05:
            return True
    return False


def find_target(root, spec):
    """Return (body, occurrence, problem)."""
    matches = [
        (b, o)
        for b, o in all_bodies(root)
        if b.isSolid
        and all(contains(b, pt(*p)) for p in spec["solid"])
        and has_boss(b, *spec["boss"])
    ]
    if len(matches) != 1:
        return None, None, f"expected 1 matching body, found {len(matches)}"
    body, occ = matches[0]
    if contains(body, pt(*spec["empty_before"])):
        return None, None, "already applied"
    return body, occ, None


def apply(design, spec, body, occ):
    """Add the tool body and join it. Returns (slot_filled, opening_clear) checked on the result."""
    tool_parts = spec["tool"]()
    probe_fill, probe_open = pt(*spec["empty_before"]), pt(*spec["empty_after"])
    native = body.nativeObject if occ else body
    comp = native.parentComponent
    if occ:  # world -> component coordinates (adding features can re-solve joints and move parts)
        m = occ.transform2.copy()
        m.invert()
        for part in tool_parts:
            adsk.fusion.TemporaryBRepManager.get().transform(part, m)
        probe_fill.transformBy(m)
        probe_open.transformBy(m)

    parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
    if parametric:
        base = comp.features.baseFeatures.add()
        base.name = spec["name"] + " plug"
        base.startEdit()
        for part in tool_parts:
            comp.bRepBodies.add(part, base)
        base.finishEdit()
        tool_bodies = [base.bodies.item(i) for i in range(base.bodies.count)]
    else:
        tool_bodies = [comp.bRepBodies.add(part) for part in tool_parts]

    tools = adsk.core.ObjectCollection.create()
    for tb in tool_bodies:
        tools.add(tb)
    comb_in = comp.features.combineFeatures.createInput(native, tools)
    comb_in.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    comb_in.isKeepToolBodies = False
    comb = comp.features.combineFeatures.add(comb_in)
    if parametric:
        comb.name = spec["name"] + " join"
    result = comb.bodies.item(0) if comb.bodies.count else native
    return contains(result, probe_fill), not contains(result, probe_open)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Open the hand design first (Design workspace), then run this script.")
            return
        root = design.rootComponent

        todo, notes = [], []
        for spec in PINCHES:
            body, occ, problem = find_target(root, spec)
            if problem:
                notes.append(f"- {spec['label']}: skipped ({problem})")
            else:
                todo.append((spec, body, occ))
        if not todo:
            ui.messageBox(
                "Nothing was changed.\n\n"
                + "\n".join(notes)
                + "\n\nIf this is unexpected: the index finger must "
                "be straight (all joints at 0), as in the STEP export."
            )
            return

        lines = [f"- {s['label']} ('{b.name}')" for s, b, _o in todo]
        answer = ui.messageBox(
            "This will change:\n"
            + "\n".join(lines)
            + ("\n\nSkipped:\n" + "\n".join(notes) if notes else "")
            + "\n\nHave you saved a copy of the design? Continue?",
            "Tendra index pinch",
            adsk.core.MessageBoxButtonTypes.YesNoButtonType,
            adsk.core.MessageBoxIconTypes.QuestionIconType,
        )
        if answer != adsk.core.DialogResults.DialogYes:
            ui.messageBox("Cancelled. Nothing was changed.")
            return

        report = []
        for spec, body, occ in todo:
            filled, clear = apply(design, spec, body, occ)
            report.append(
                f"{spec['label']}\n   slot filled: {'yes' if filled else 'NO'}, "
                f"tendon opening clear: {'yes' if clear else 'NO'}"
            )
        ui.messageBox(
            "Done.\n\n"
            + "\n".join(report)
            + "\n\nUndo: delete the 'Tendra MCP pinch' / 'Tendra ABD pinch' features at the end of the timeline."
        )
    except Exception:
        if ui:
            ui.messageBox("TendraIndexPinch failed:\n" + traceback.format_exc())
