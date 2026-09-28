"""Tendra Hand: read-only inspection of the open Fusion design.

Writes a text report (components, occurrences, bodies, joints and cylindrical faces, in world
millimetres) next to this script, so the bodies can be matched to the STEP export.
It does not change the design.
"""

import os
import traceback

import adsk.core
import adsk.fusion

OUT_NAME = "inspect_output.txt"


def mm(v):
    return v * 10.0  # Fusion's API always works in cm


def fmt_pt(p):
    return f"({mm(p.x):.2f}, {mm(p.y):.2f}, {mm(p.z):.2f})"


def fmt_dir(v):
    return f"({v.x:.3f}, {v.y:.3f}, {v.z:.3f})"


def describe_body(body, lines, indent):
    bb = body.boundingBox
    lines.append(
        f"{indent}body '{body.name}' visible={body.isVisible} solid={body.isSolid} "
        f"vol={body.volume * 1000:.0f} mm3 bbox {fmt_pt(bb.minPoint)} .. {fmt_pt(bb.maxPoint)}"
    )
    cylinders = {}
    for face in body.faces:
        if face.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
            continue
        cyl = adsk.core.Cylinder.cast(face.geometry)
        key = (round(mm(cyl.radius), 2), fmt_dir(cyl.axis), fmt_pt(cyl.origin))
        cylinders[key] = cylinders.get(key, 0) + 1
    for (radius, axis, origin), count in sorted(cylinders.items()):
        lines.append(f"{indent}  cyl r={radius} axis={axis} origin={origin} faces={count}")


def walk(occurrences, lines, depth):
    for occ in occurrences:
        indent = "  " * depth
        t = occ.transform2.translation
        lines.append(
            f"{indent}occurrence '{occ.name}' component='{occ.component.name}' "
            f"visible={occ.isVisible} grounded={occ.isGrounded} "
            f"translation=({mm(t.x):.2f}, {mm(t.y):.2f}, {mm(t.z):.2f})"
        )
        for body in occ.bRepBodies:  # proxies: geometry in world coordinates
            describe_body(body, lines, indent + "  ")
        walk(occ.childOccurrences, lines, depth + 1)


def describe_joint(kind, joint, lines):
    try:
        motion = joint.jointMotion
        text = f"{kind} '{joint.name}' type={motion.jointType}"
        one, two = joint.occurrenceOne, joint.occurrenceTwo
        text += f" one='{one.name if one else 'root'}' two='{two.name if two else 'root'}'"
        if motion.jointType == adsk.fusion.JointTypes.RevoluteJointType:
            rev = adsk.fusion.RevoluteJointMotion.cast(motion)
            text += f" angle={rev.rotationValue:.4f} rad"
            if rev.rotationAxisVector:
                text += f" axis={fmt_dir(rev.rotationAxisVector)}"
        geo = getattr(joint, "geometryOrOriginOne", None) or getattr(joint, "geometry", None)
        if geo is not None and hasattr(geo, "origin") and geo.origin:
            text += f" origin={fmt_pt(geo.origin)}"
        lines.append(text)
    except Exception as exc:  # report and keep going; this script must never fail halfway
        lines.append(f"{kind} '{getattr(joint, 'name', '?')}' (could not read: {exc})")


# ---- Index PIP probe points, world mm, with all joints at 0 (the pose Fusion snaps back to).
# Same x/y/z frame as TendraPipPinch, whose numbers are for the tilted pose of the STEP export.
PIP_AXIS_DIR = (1.0, 0.0, 0.0)
PIP_AXIS_PT = (-6.449, 19.02, 118.98)
DIP_AXIS_PT = (-6.449, 19.02, 141.98)
PIP_PROBES = {  # name: (x along axis, y toward back, z along middle segment), mm
    "tongue_a": (-4.0, 0.0, 3.0),
    "tongue_b": (0.2, 0.0, 3.0),
    "plug (slot, palm side)": (-1.05, -3.0, 3.0),
    "hole centre": (-1.05, 0.0, 3.0),
}


def _norm(v):
    n = sum(c * c for c in v) ** 0.5
    return tuple(c / n for c in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def pip_point(x, y, z):
    d = _norm(PIP_AXIS_DIR)
    al = tuple(DIP_AXIS_PT[i] - PIP_AXIS_PT[i] for i in range(3))
    k = sum(al[i] * d[i] for i in range(3))
    al = _norm(tuple(al[i] - k * d[i] for i in range(3)))
    side = _cross(al, d)
    p = [PIP_AXIS_PT[i] + x * d[i] + y * side[i] + z * al[i] for i in range(3)]
    return adsk.core.Point3D.create(p[0] / 10, p[1] / 10, p[2] / 10)


def probe_bodies(root, lines):
    inside = adsk.fusion.PointContainment.PointInsidePointContainment
    bodies = [(b, "root") for b in root.bRepBodies]
    for occ in root.allOccurrences:
        bodies += [(b, occ.fullPathName) for b in occ.bRepBodies]
    for name, (x, y, z) in PIP_PROBES.items():
        pt = pip_point(x, y, z)
        hits = [f"'{b.name}' in {where}" for b, where in bodies if b.pointContainment(pt) == inside]
        lines.append(f"  {name} {fmt_pt(pt)}: inside {hits if hits else 'no body'}")


def describe_timeline(design, lines, last=12):
    tl = design.timeline
    for i in range(max(0, tl.count - last), tl.count):
        item = tl.item(i)
        try:
            health = item.healthState
            msg = item.errorOrWarningMessage
        except Exception:
            health, msg = "?", ""
        lines.append(
            f"  [{i}] '{item.name}' health={health} suppressed={item.isSuppressed} "
            f"rolled_back={item.isRolledBack} {msg}"
        )


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
        lines = [
            f"document: {app.activeDocument.name}",
            f"design type: {'parametric' if design.designType == adsk.fusion.DesignTypes.ParametricDesignType else 'direct'}",
            f"default length unit: {design.unitsManager.defaultLengthUnits}",
            f"root component: '{root.name}'",
        ]
        if design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
            lines.append(f"timeline features: {design.timeline.count}")

        lines.append("\n== user parameters ==")
        for p in design.userParameters:
            lines.append(f"{p.name} = {p.expression}")

        lines.append("\n== bodies in root ==")
        for body in root.bRepBodies:
            describe_body(body, lines, "  ")

        lines.append("\n== occurrences (world coordinates, mm) ==")
        walk(root.occurrences, lines, 0)

        lines.append("\n== joints ==")
        for comp in design.allComponents:
            for j in comp.joints:
                describe_joint(f"[{comp.name}] joint", j, lines)
            for j in comp.asBuiltJoints:
                describe_joint(f"[{comp.name}] as-built joint", j, lines)

        lines.append("\n== index PIP probe points (which bodies contain them) ==")
        probe_bodies(root, lines)

        if design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
            lines.append("\n== last timeline items (health: 0 ok, 1 warning, 2 error) ==")
            describe_timeline(design, lines)

        out = os.path.join(os.path.dirname(os.path.realpath(__file__)), OUT_NAME)
        with open(out, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        ui.messageBox(f"Done. Nothing in the design was changed.\n\nReport written to:\n{out}")
    except Exception:
        if ui:
            ui.messageBox("TendraInspect failed:\n" + traceback.format_exc())
