"""Tendra Hand V1: build the full 5-finger, 21-DOF hand from the thumb + index prototype.

Run on the "Tendra Hand V1" design (a copy of "Hand assebly"), with every joint at 0. The script works
in stages, and each stage skips itself if it has already been applied:

- fingers: copy the index into middle, ring and little fingers, with human-like lengths (the prismatic
  mid-part of the proximal and middle phalanges is stretched or shortened, so joints, holes and
  tendon slots keep their exact size), and add their revolute joints.

Geometry is in world mm. All index parts have identity transforms, so component space = world space
(Fusion's API works in cm; `pt()` converts).

Run it from Fusion (Utilities -> Scripts and Add-Ins) to apply all stages, or stage by stage from
Claude Code through the Fusion MCP: exec the file with TENDRA_STAGE set to a stage name.
"""

import math
import traceback

import adsk.core
import adsk.fusion

# ---------------------------------------------------------------------------------------------------
# Index geometry (from the design, all joints at 0)
# ---------------------------------------------------------------------------------------------------

INDEX = {  # part -> occurrence name in the prototype
    "base": "Body30 (1):1",  # MCP sideways yoke
    "proximal": "Body30 (2):1",
    "middle": "Body30 (1) (1) (1):1",
    "distal": "Body30 (1) (1):1",
}
PALM = "Body30 (1) (2):1"
EXPORT_DIR = "C:/Users/07dan/Documents/Projects/Robotic-hand/hardware/robot_description/v1_export"
X_ABD, Y_ABD, Z_ABD = -8.531, 12.523, 59.976  # index sideways axis (along Y)
X_FLEX, Y_FLEX = -14.031, 19.023  # flex axes (along X) pass through this x (face of the boss), y
Z_MCP, Z_PIP, Z_DIP = 78.976, 118.976, 141.976
Z_STRETCH_PROX = 103.0  # centre of the prismatic part of the proximal phalanx (z 96..110)
Z_STRETCH_MID = 130.5  # centre of the prismatic part of the middle phalanx (z 128..133)
PRISM_PROX = (96.0, 110.0)
PRISM_MID = (128.0, 133.0)

# New fingers: sideways offset of the knuckle (x, towards the little finger = -X), knuckle height
# change (z), and length change of the proximal / middle phalanx, all mm. Knuckle pitch 19 mm.
# Lengths follow human ratios relative to the index (proximal: middle 1.10, ring 1.02, little 0.80;
# middle phalanx: 1.13, 1.08, 0.80); see research/experiments/2026-09-28-full-hand/.
FINGERS = {
    "middle": {"dx": -19.0, "dz": 4.0, "prox": 4.0, "mid": 3.0},
    "ring": {"dx": -38.0, "dz": 1.0, "prox": 1.0, "mid": 2.0},
    "little": {"dx": -57.0, "dz": -7.0, "prox": -8.0, "mid": -4.0},
}


def pt(x, y, z):
    return adsk.core.Point3D.create(x / 10, y / 10, z / 10)


def vec(x, y, z):
    v = adsk.core.Vector3D.create(x, y, z)
    v.normalize()
    return v


def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def box(x0, x1, y0, y1, z0, z1):
    obb = adsk.core.OrientedBoundingBox3D.create(
        pt((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
        vec(1, 0, 0),
        vec(0, 1, 0),
        (x1 - x0) / 10,
        (y1 - y0) / 10,
        (z1 - z0) / 10,
    )
    return tbm().createBox(obb)


def translate(body, dx, dy, dz):
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(dx / 10, dy / 10, dz / 10)
    tbm().transform(body, m)
    return body


def cut_z(body, z0, z1):
    """Copy of `body` limited to z0 <= z <= z1 (mm)."""
    part = tbm().copy(body)
    tbm().booleanOperation(
        part, box(-500, 500, -500, 500, z0, z1), adsk.fusion.BooleanTypes.IntersectionBooleanType
    )
    return part


def union(a, b):
    tbm().booleanOperation(a, b, adsk.fusion.BooleanTypes.UnionBooleanType)
    return a


def _op(a, b, kind):
    t = {
        "-": adsk.fusion.BooleanTypes.DifferenceBooleanType,
        "+": adsk.fusion.BooleanTypes.UnionBooleanType,
        "&": adsk.fusion.BooleanTypes.IntersectionBooleanType,
    }[kind]
    tbm().booleanOperation(a, b, t)
    return a


def change_length(body, zc, d, prism):
    """Stretch (d > 0) or shorten (d < 0) a segment by d mm at z = zc, inside a prismatic region.

    Everything above zc moves by d. The gap is filled with a shifted copy of the prismatic part, whose
    cross-section is constant, so the result is exactly the same shape, only longer or shorter.
    """
    if abs(d) < 1e-9:
        return tbm().copy(body)
    z0, z1 = prism
    if d > 0:
        assert d <= (z1 - z0), "stretch longer than the prismatic region"
        low = cut_z(body, -500, zc)
        high = translate(cut_z(body, zc, 500), 0, 0, d)
        s = d / 2  # the shifted prism covers zc..zc+d
        assert z0 + s <= zc and zc + d <= z1 + s
        fill = translate(cut_z(body, z0, z1), 0, 0, s)
        return union(union(low, fill), high)
    d = -d
    assert zc - d / 2 >= z0 and zc + d / 2 <= z1, "shortening outside the prismatic region"
    low = cut_z(body, -500, zc - d / 2)
    high = translate(cut_z(body, zc + d / 2, 500), 0, 0, -d)
    return union(low, high)


# ---------------------------------------------------------------------------------------------------
# Design helpers
# ---------------------------------------------------------------------------------------------------


def find_occ(root, name):
    for o in root.allOccurrences:
        if o.name == name:
            return o
    return None


def find_comp_occ(root, comp_name):
    for o in root.occurrences:
        if o.component.name == comp_name:
            return o
    return None


def new_part(root, name, body):
    """New component at the root holding one body (world coordinates = component coordinates)."""
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    bf = comp.features.baseFeatures.add()
    bf.name = name + " body"
    bf.startEdit()
    comp.bRepBodies.add(body, bf)
    bf.finishEdit()
    comp.bRepBodies.item(0).name = name
    return occ


def pivot_face(occ, axis, through, radius_range=(0.5, 9.0)):
    """Cylindrical face of `occ` (proxy) whose axis is parallel to `axis` and passes through `through` (mm)."""
    d = vec(*axis)
    best = None
    for body in occ.bRepBodies:
        for f in body.faces:
            if f.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
                continue
            cyl = adsk.core.Cylinder.cast(f.geometry)
            r = cyl.radius * 10
            if not radius_range[0] <= r <= radius_range[1]:
                continue
            a = cyl.axis.copy()
            a.normalize()
            if abs(abs(a.dotProduct(d)) - 1) > 1e-4:
                continue
            o = cyl.origin
            rel = adsk.core.Vector3D.create(
                through[0] - o.x * 10, through[1] - o.y * 10, through[2] - o.z * 10
            )
            if rel.crossProduct(d).length > 0.05:
                continue
            if best is None or f.area > best.area:
                best = f
    return best


def add_revolute(root, name, child, parent, axis, through):
    """As-built revolute joint (child moves relative to parent) about the pivot cylinder of `child`."""
    for j in root.asBuiltJoints:
        if j.name == name:
            return j
    face = pivot_face(child, axis, through) or pivot_face(parent, axis, through)
    if face is None:
        raise RuntimeError(f"{name}: no pivot cylinder through {through} along {axis}")
    geo = adsk.fusion.JointGeometry.createByNonPlanarFace(
        face, adsk.fusion.JointKeyPointTypes.MiddleKeyPoint
    )
    ji = root.asBuiltJoints.createInput(child, parent, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


# ---------------------------------------------------------------------------------------------------
# Stage: fingers
# ---------------------------------------------------------------------------------------------------


def stage_fingers(design, log):
    root = design.rootComponent
    if all(find_comp_occ(root, f"{f}_base") for f in FINGERS):
        log("fingers: already there, skipped")
        return
    # prototype occurrence names, or the names given by stage 'names'
    src = {k: find_occ(root, v) or find_comp_occ(root, f"index_{k}") for k, v in INDEX.items()}
    if any(o is None for o in src.values()):
        raise RuntimeError(
            "index parts not found: " + str({k: v is not None for k, v in src.items()})
        )
    palm = find_occ(root, PALM) or find_comp_occ(root, "palm")
    bodies = {k: o.bRepBodies.item(0) for k, o in src.items()}  # proxies, world coordinates
    for finger, p in FINGERS.items():
        if find_comp_occ(root, f"{finger}_base"):
            log(f"{finger}: already there, skipped")
            continue
        dx, dz, dp, dm = p["dx"], p["dz"], p["prox"], p["mid"]
        parts = {
            "base": translate(tbm().copy(bodies["base"]), dx, 0, dz),
            "proximal": translate(
                change_length(bodies["proximal"], Z_STRETCH_PROX, dp, PRISM_PROX), dx, 0, dz
            ),
            "middle": translate(
                change_length(bodies["middle"], Z_STRETCH_MID, dm, PRISM_MID), dx, 0, dz + dp
            ),
            "distal": translate(tbm().copy(bodies["distal"]), dx, 0, dz + dp + dm),
        }
        occ = {k: new_part(root, f"{finger}_{k}", b) for k, b in parts.items()}
        x_abd, z_abd = X_ABD + dx, Z_ABD + dz
        add_revolute(root, f"{finger}_mcp_abd", occ["base"], palm, (0, 1, 0), (x_abd, Y_ABD, z_abd))
        add_revolute(
            root,
            f"{finger}_mcp_flex",
            occ["proximal"],
            occ["base"],
            (1, 0, 0),
            (X_FLEX + dx, Y_FLEX, Z_MCP + dz),
        )
        add_revolute(
            root,
            f"{finger}_pip",
            occ["middle"],
            occ["proximal"],
            (1, 0, 0),
            (X_FLEX + dx, Y_FLEX, Z_PIP + dz + dp),
        )
        add_revolute(
            root,
            f"{finger}_dip",
            occ["distal"],
            occ["middle"],
            (1, 0, 0),
            (X_FLEX + dx, Y_FLEX, Z_DIP + dz + dp + dm),
        )
        vols = {k: round(o.bRepBodies.item(0).volume * 1000, 0) for k, o in occ.items()}
        log(f"{finger}: added (volumes mm3 {vols})")


# ---------------------------------------------------------------------------------------------------
# Stage: thumb. V1 keeps the prototype's 4-DOF thumb (owner, 2026-09-29): cmc_rot, cmc_flex, mcp_flex,
# ip, with the one-piece proximal phalanx. A 5th DOF (a sideways hinge in the proximal phalanx,
# "thumb_mcp_abd") was built on 2026-09-28 and removed again; `thumb_unhinge` migrates a design that
# still has it.
# ---------------------------------------------------------------------------------------------------

THUMB_PROX_BODY = "Body30 (1) (1) (2)"  # the prototype's proximal phalanx (raw body / component)
THUMB_META = "Body30 (1) (3):1"
THUMB_FLEX_AXIS = (0.707107, 0.0, 0.707107)
THUMB_MCP_PT = (-4.303, -31.976, 6.073)
THUMB_IP_PT = (-4.303, -66.976, 6.073)
# Where the old raw body (left at its pre-move position when the hinged version replaced it) goes:
# its MCP / IP axes are the lines (x, 22.5, 1.5) / (x, -12.5, 1.5), its centre x = 83.5. Turning it
# 135 deg about Y and moving the centre to (1.0, ., 11.3764) and y by -54.4764 puts both axes on the
# V1 thumb's (checked: same bounding box as the hinged parts, best overlap of the two turns).
PROX_TURN_DEG, PROX_OLD_CENTRE, PROX_NEW_CENTRE, PROX_DY = (
    135.0,
    (83.5, 1.5),
    (1.0, 11.3764),
    -54.4764,
)


def stage_thumb(design, log):
    log("thumb: 4-DOF thumb, nothing to add")


def _placed_old_proximal(design):
    """The prototype's proximal phalanx body, moved onto the V1 thumb's axes (temporary body)."""
    root = design.rootComponent
    f = next(f for f in root.features.removeFeatures if f.name == "v1 remove old thumb proximal")
    tl = design.timeline
    try:
        f.timelineObject.rollTo(True)
        body = tbm().copy(adsk.fusion.BRepBody.cast(f.itemToRemove))
    finally:
        tl.moveToEnd()
    t = math.radians(PROX_TURN_DEG)
    c, s = math.cos(t), math.sin(t)
    (ox, oz), (nx, nz) = PROX_OLD_CENTRE, PROX_NEW_CENTRE
    xp, zp = ox * c + oz * s, -ox * s + oz * c
    m = adsk.core.Matrix3D.create()
    m.setWithArray(
        [c, 0, s, (nx - xp) / 10, 0, 1, 0, PROX_DY / 10, -s, 0, c, (nz - zp) / 10, 0, 0, 0, 1]
    )
    tbm().transform(body, m)
    return body


def stage_thumb_unhinge(design, log):
    """Replace the hinged proximal (thumb_mcp_link + thumb_proximal, joint thumb_mcp_abd) by the
    prototype's one-piece phalanx, and re-add the thumb_mcp_flex / thumb_ip joints to it."""
    root = design.rootComponent
    link = find_comp_occ(root, "thumb_mcp_link")
    if link is None and find_comp_occ(root, "thumb_proximal"):
        log("thumb_unhinge: one-piece proximal already there, skipped")
        return
    body = _placed_old_proximal(design)
    for j in list(root.asBuiltJoints):
        if j.name in ("thumb_mcp_flex", "thumb_mcp_abd", "thumb_ip"):
            j.deleteMe()
    for name in ("thumb_proximal", "thumb_mcp_link"):  # the hinge parts, if still there
        occ = find_comp_occ(root, name)
        if occ:
            occ.deleteMe()
    prox = new_part(root, "thumb_proximal", body)
    meta, tip = find_comp_occ(root, "thumb_metacarpal"), find_comp_occ(root, "thumb_distal")
    add_revolute(root, "thumb_mcp_flex", prox, meta, THUMB_FLEX_AXIS, THUMB_MCP_PT)
    add_revolute(root, "thumb_ip", tip, prox, THUMB_FLEX_AXIS, THUMB_IP_PT)
    log(
        f"thumb_unhinge: one-piece proximal ({prox.bRepBodies.item(0).volume * 1000:.0f} mm3), joints re-added"
    )


# ---------------------------------------------------------------------------------------------------
# Stage: names (prototype components and joints get v1 names)
# ---------------------------------------------------------------------------------------------------

RENAME_COMPONENTS = {
    "Body30 (1) (2)": "palm",
    "Body30 (1)": "index_base",
    "Body30 (2)": "index_proximal",
    "Body30 (1) (1) (1)": "index_middle",
    "Body30 (1) (1)": "index_distal",
    "Body21": "thumb_base",
    "Body30 (1) (3)": "thumb_metacarpal",
    "Body30 (1) (1) (1) (1)": "thumb_distal",
    THUMB_PROX_BODY: "thumb_proximal",
}
RENAME_JOINTS = {
    "Revolute 1": "index_mcp_abd",
    "Revolute 2": "index_mcp_flex",
    "Revolute 3": "index_pip",
    "Revolute 4": "index_dip",
    "Revolute 5": "thumb_cmc_rot",
    "Revolute 6": "thumb_cmc_flex",
    "Revolute 7": "thumb_mcp_flex",
    "Revolute 8": "thumb_ip",
}


def stage_names(design, log):
    root = design.rootComponent
    n = 0
    for occ in root.occurrences:
        new = RENAME_COMPONENTS.get(occ.component.name)
        if new:
            occ.component.name = new
            for b in occ.component.bRepBodies:
                b.name = new
            n += 1
    for j in root.joints:
        if j.name in RENAME_JOINTS:
            j.name = RENAME_JOINTS[j.name]
            n += 1
    log(f"names: {n} renamed")


# ---------------------------------------------------------------------------------------------------
# Stage: export (STL per part in world coordinates at q = 0, plus joints as JSON) for the simulation
# ---------------------------------------------------------------------------------------------------


def _joint_record(j, as_built):
    m = adsk.fusion.RevoluteJointMotion.cast(j.jointMotion)
    if as_built:
        o = j.geometry.origin
    else:
        o = j.geometryOrOriginOne.origin
    a = m.rotationAxisVector
    return {
        "name": j.name,
        "child": j.occurrenceOne.component.name,
        "parent": j.occurrenceTwo.component.name,
        "point_mm": [round(o.x * 10, 4), round(o.y * 10, 4), round(o.z * 10, 4)],
        "axis": [round(a.x, 6), round(a.y, 6), round(a.z, 6)],
        "angle_rad": m.rotationValue,
    }


def stage_export(design, log):
    import json
    import os

    root = design.rootComponent
    rev = adsk.fusion.JointTypes.RevoluteJointType
    joints = [_joint_record(j, False) for j in root.joints if j.jointMotion.jointType == rev]
    joints += [_joint_record(j, True) for j in root.asBuiltJoints if j.jointMotion.jointType == rev]
    if any(abs(j["angle_rad"]) > 1e-9 for j in joints):
        raise RuntimeError("set every joint to 0 before exporting")
    os.makedirs(EXPORT_DIR + "/meshes", exist_ok=True)
    em = design.exportManager
    parts = []
    for occ in root.occurrences:
        if not occ.isLightBulbOn and occ.component.name not in RENAME_COMPONENTS.values():
            continue
        name = occ.component.name
        if "_link_" in name:  # DIP coupling plates and bar: the sim models the bar as a tendon
            continue
        body = occ.bRepBodies.item(0)  # proxy: world coordinates
        opts = em.createSTLExportOptions(body, f"{EXPORT_DIR}/meshes/{name}.stl")
        opts.isBinaryFormat = True
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
        em.execute(opts)
        pp = body.physicalProperties
        parts.append(
            {"name": name, "mesh": f"meshes/{name}.stl", "volume_mm3": round(pp.volume * 1000, 1)}
        )
    data = {
        "source": "Fusion design 'Tendra Hand V1', exported by TendraHandV1.py stage 'export'",
        "units": "mm, world frame of the design, all joints at 0",
        "note": "Fusion's joint axis signs are arbitrary; the simulation converter applies the "
        "closing-positive convention (CLAUDE.md).",
        "parts": sorted(parts, key=lambda p: p["name"]),
        "joints": sorted(joints, key=lambda j: j["name"]),
    }
    with open(EXPORT_DIR + "/hand_v1.json", "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    log(f"export: {len(parts)} parts, {len(joints)} joints -> {EXPORT_DIR}")


# ---------------------------------------------------------------------------------------------------
# Tendon routes and servo layout: hardware/cad/tendon_router.py writes them (and tests them)
# ---------------------------------------------------------------------------------------------------

ROUTES_JSON = EXPORT_DIR + "/tendon_routes.json"


def load_routes():
    import json

    with open(ROUTES_JSON, encoding="utf-8") as f:
        return json.load(f)


def _cyl_pts(a, b, r):
    return tbm().createCylinderOrCone(pt(*a), r / 10, pt(*b), r / 10)


def _sphere(c, r):
    return tbm().createSphere(pt(*c), r / 10)


def _sk_line(sk, a, b):
    return sk.sketchCurves.sketchLines.addByTwoPoints(
        sk.modelToSketchSpace(pt(*a)), sk.modelToSketchSpace(pt(*b))
    )


def _sk_spline(sk, pts):
    """Fitted spline; points closer than 0.3 mm to the previous one are dropped (they kink it)."""
    keep = [pts[0]]
    for q in pts[1:-1]:
        if math.dist(q, keep[-1]) >= 0.3 and math.dist(q, pts[-1]) >= 0.3:
            keep.append(q)
    keep.append(pts[-1])
    coll = adsk.core.ObjectCollection.create()
    for q in keep:
        coll.add(sk.modelToSketchSpace(pt(*q)))
    return sk.sketchCurves.sketchFittedSplines.add(coll)


def _sk_curve(sk, pts):
    """A line if the points are (nearly) collinear, else a fitted spline through them."""
    a, b = pts[0], pts[-1]
    ab = [b[i] - a[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in ab))
    off = max(
        math.sqrt(
            sum(
                (q[i] - a[i] - ab[i] * sum((q[k] - a[k]) * ab[k] for k in range(3)) / n / n) ** 2
                for i in range(3)
            )
        )
        for q in pts
    )
    return _sk_line(sk, a, b) if off < 1e-3 else _sk_spline(sk, pts)


SHEATH_STOP_Z = (
    -40.0
)  # sheaths: the last 2 mm of the wrist plate is a 1.2 mm bore (the tube's stop)


def _thumb_paths(sk, st, curves_only):
    """Sketch paths (curve, diameter, label) for the thumb strands (see tendon_router._thumb_path)."""
    P = st["points_mm"][:-1]  # without the spool tangent
    kind, name = st["kind"], st["name"]
    small = P.index(st["small_bore_end_mm"])
    out = []
    if kind == "rot" and not curves_only:  # from the bay wall over the pin, then down
        e = P[0]
        out += [
            (_sk_line(sk, (e[0], e[1] - 1.0, e[2]), P[1]), 0.12, name + " wall"),
            (_sk_spline(sk, P[1:6]), 0.12, name + " wrap"),
            (_sk_line(sk, P[5], P[small]), 0.12, name + " drop"),
        ]
    if kind == "bare" and not curves_only:  # own 1.2 mm bore: down, then along the S-curve
        out += [
            (_sk_line(sk, P[0], P[1]), 0.12, name + " hold"),
            (_sk_curve(sk, P[1 : small + 1]), 0.12, name + " entry"),
        ]
    tube = P[small:] if kind != "sheath" else P
    if kind == "sheath" and curves_only:  # stop at the wrist plate bottom
        tube = [q for q in tube if q[2] >= SHEATH_STOP_Z] + [
            [P[-1][0], P[-1][1], SHEATH_STOP_Z - 0.01]
        ]
        out.append(
            (_sk_line(sk, tube[-1], (P[-1][0], P[-1][1], P[-1][2] - 1.0)), 0.12, name + " stop")
        )
    out.append((_sk_curve(sk, tube), 0.22, name))
    return out


def cut_channels(comp, body, strands, name, curves_only=False):
    """Cut one channel per strand into `body` with Pipe features along 3D sketch paths:
    a 1.2 mm bore for the bare-line entry section (plus a 0.6 mm entry chamfer), then a 2.2 mm bore
    for the PTFE tube along the S-curve down to the wrist plate bottom. Pipes outside the body cut
    nothing, so the same call works for the palm and the forearm's wrist plate. Thumb strands have
    their own shapes (`kind`: rot / bare / sheath)."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = name + " paths"
    sk.isComputeDeferred = True
    paths = []
    for st in strands:
        if "kind" in st:
            paths += _thumb_paths(sk, st, curves_only)
            continue
        e, small_end = st["entry_mm"], st["small_bore_end_mm"]
        top = sk.modelToSketchSpace(pt(e[0], e[1], e[2] + 1.5))
        line = sk.sketchCurves.sketchLines.addByTwoPoints(
            top, sk.modelToSketchSpace(pt(*small_end))
        )
        if not curves_only:
            paths.append((line, 0.12, st["name"] + " entry"))
        # straight part (index strands drop beside the thumb bay first), then the S-curve on its own
        small_end, curve_start = st["small_bore_end_mm"], st["curve_start_mm"]
        if math.dist(small_end, curve_start) > 1e-6 and not curves_only:
            drop = sk.sketchCurves.sketchLines.addByTwoPoints(
                sk.modelToSketchSpace(pt(*small_end)), sk.modelToSketchSpace(pt(*curve_start))
            )
            paths.append((drop, 0.22, st["name"] + " drop"))
        curve = st["points_mm"][
            st["points_mm"].index(curve_start) : -1
        ]  # curve_start .. wrist bottom
        lateral = math.dist(curve[0][:2], curve[-1][:2])
        if lateral < 1e-3:
            seg = sk.sketchCurves.sketchLines.addByTwoPoints(
                sk.modelToSketchSpace(pt(*curve[0])), sk.modelToSketchSpace(pt(*curve[-1]))
            )
        else:
            seg = _sk_spline(sk, curve)
        paths.append((seg, 0.22, st["name"]))
    sk.isComputeDeferred = False
    pipes = comp.features.pipeFeatures
    for curve, dia, sname in paths:
        pi = pipes.createInput(
            comp.features.createPath(curve, False),
            adsk.fusion.FeatureOperations.CutFeatureOperation,
        )
        pi.sectionSize = adsk.core.ValueInput.createByReal(dia)
        pi.participantBodies = [body]
        try:
            f = pipes.add(pi)
        except RuntimeError as exc:
            raise RuntimeError(f"pipe '{name} {sname}' failed: {exc}") from None
        f.name = f"{name} {sname}"
    # entry chamfers: small cones at the top of every finger channel
    cones = []
    for st in strands:
        if "kind" in st:
            continue
        e = st["entry_mm"]
        cones.append(
            tbm().createCylinderOrCone(
                pt(e[0], e[1], e[2] + 0.3), 0.15, pt(e[0], e[1], e[2] - 0.6), 0.06
            )
        )
    return cones


def _feature_with_tools(comp, target, tools, op, name):
    """Add temporary bodies as a base feature and combine them into `target` (join or cut)."""
    bf = comp.features.baseFeatures.add()
    bf.name = name + " tools"
    bf.startEdit()
    for t in tools:
        comp.bRepBodies.add(t, bf)
    bf.finishEdit()
    coll = adsk.core.ObjectCollection.create()
    for i in range(bf.bodies.count):
        coll.add(bf.bodies.item(i))
    ci = comp.features.combineFeatures.createInput(target, coll)
    ci.operation = op
    ci.isKeepToolBodies = False
    cf = comp.features.combineFeatures.add(ci)
    cf.name = name
    return cf.bodies.item(0) if cf.bodies.count else target


# ---------------------------------------------------------------------------------------------------
# Stage: palm (full-width palm with knuckle posts for all fingers and one channel per strand)
# ---------------------------------------------------------------------------------------------------

# Palm top under each finger's knuckle post (z = 38.5 + knuckle height), x ranges between the posts.
PALM_STEPS = [  # (x0, x1, top z)
    (-22.0, -16.0, 38.5),  # index strand gap, beside the thumb bay
    (-38.0, -22.0, 42.5),  # middle
    (-57.5, -38.0, 39.5),  # ring
    (-79.0, -57.5, 31.5),  # little
]
PALM_BACK = 40.0  # palm thickened from y 32.5 to 40 (room for the channel rows)
PALM_BOTTOM = -30.0


def stage_palm(design, log):
    root = design.rootComponent
    occ = find_comp_occ(root, "palm")
    comp = occ.component
    if comp.features.itemByName("v1 palm channels"):
        log("palm: already built, skipped")
        return
    routes = load_routes()
    body = comp.bRepBodies.item(0)
    # knuckle posts for the new fingers: copies of the index post (translated like the fingers)
    post = _op(tbm().copy(body), box(-16.0, -1.0, -50, 60, Z_PLATE_TOP_PALM, 90), "&")
    posts = [translate(tbm().copy(post), p["dx"], 0, p["dz"]) for p in FINGERS.values()]
    # 1. remove the prototype's stand: its left block (x < -16) and legs (z < -7.6)
    cut = box(-200, -16.0, -200, 200, -200, 200)
    _op(cut, box(-200, 200, -200, 200, -200, -7.6), "+")
    body = _feature_with_tools(
        comp, body, [cut], adsk.fusion.FeatureOperations.CutFeatureOperation, "v1 palm trim"
    )
    # 2. add the full palm: base down to the wrist, steps under the knuckles, thicker back, posts
    x0, x1 = routes["palm"]["x"]
    y0 = routes["palm"]["y"][0]
    add = [
        box(x0, x1, y0, PALM_BACK, PALM_BOTTOM, -7.6),
        box(-16.0, x1, 32.5, PALM_BACK, -7.6, 38.5),
    ]
    add += [box(a, b, y0, PALM_BACK, -7.6, top) for a, b, top in PALM_STEPS]
    body = _feature_with_tools(
        comp, body, add + posts, adsk.fusion.FeatureOperations.JoinFeatureOperation, "v1 palm body"
    )
    # 3. thumb (research/experiments/2026-09-29-thumb-routing): deeper bay floor, bearing for the
    #    base's hollow journal (a U-slot open to the front, the base slides in from there), and the
    #    blind hole for the 3 mm steel pin that turns the cmc_rot strands down
    body = _feature_with_tools(
        comp,
        body,
        palm_thumb_tools(routes["thumb"]),
        adsk.fusion.FeatureOperations.CutFeatureOperation,
        "v1 palm thumb bay",
    )
    # 4. one channel per strand, and M3 heat-set insert holes in the bottom (wrist) face
    tools = cut_channels(comp, body, routes["strands"], "v1 palm channel")
    body = comp.bRepBodies.item(0)
    p = routes["palm"]
    for x, y in p["inserts_xy"]:
        tools.append(
            _cyl_pts(
                (x, y, PALM_BOTTOM - 1), (x, y, PALM_BOTTOM + p["insert_depth"]), p["insert_r"]
            )
        )
    _feature_with_tools(
        comp, body, tools, adsk.fusion.FeatureOperations.CutFeatureOperation, "v1 palm channels"
    )
    log(
        f"palm: built, {len(routes['strands'])} channels, volume {comp.bRepBodies.item(0).volume * 1000:.0f} mm3"
    )


Z_PLATE_TOP_PALM = 38.5
OLD_BAY_FLOOR_Z = -4.7


def palm_thumb_tools(th):
    ax, ay = th["rot_axis_xy"]
    j = th["journal"]
    rb, zb = j["r"] + j["clearance"], j["bottom_z"] - j["clearance"]
    floor = th["bay_floor_z"]
    tools = [
        box(-16.0, 13.0, 12.0, 28.5, floor, OLD_BAY_FLOOR_Z + 0.5),  # deeper bay floor
        _cyl_pts((ax, ay, zb), (ax, ay, floor + 0.5), rb),  # journal bearing
        box(ax - rb, ax + rb, 5.0, ay, zb, floor + 0.5),
    ]  # its U-slot to the front face
    py, pz = th["rot_pin_yz"]
    x0, x1 = th["rot_pin_x"]
    tools.append(_cyl_pts((x0, py, pz), (x1 + 1.0, py, pz), th["pin_r"]))
    return tools


# ---------------------------------------------------------------------------------------------------
# Stage: forearm (wrist plate, servo decks, 21 SCS0009 servos with spools)
# ---------------------------------------------------------------------------------------------------

FOREARM_Y = (-17.0, 68.0)
FOREARM_BOTTOM = -153.0
DECK_T = 3.0
POCKET_CLEAR = 0.2  # per side, PLA print tolerance for the servo case
PILOT_R = 0.8  # M2 self-tapping screw pilot hole (1.6 mm)


def _servo_frame(s):
    """(x, y_sheet, z_spool, shaft_sign) of a servo from the routes JSON."""
    x, y, z = s["spool_center_mm"]
    return x, y, z, s["shaft_dir"][1]


def _deck_for(s, sv):
    """Deck plate (y range) that the servo's tabs rest on: on the far side of the tabs from the shaft."""
    (ty0, ty1) = s["tab_box_mm"][1]
    _x, _y, _z, sh = _servo_frame(s)
    return (ty0 - DECK_T, ty0) if sh > 0 else (ty1, ty1 + DECK_T)


def forearm_body(routes):
    sv = routes["servo"]
    x0, x1 = routes["palm"]["x"]
    ya, yb = FOREARM_Y
    zw, zp = routes["palm"]["z_wrist"], routes["palm"]["z_wrist_plate_bottom"]
    body = box(x0, x1, ya, yb, zp, zw)  # wrist plate
    _op(body, box(x0, x0 + 3, ya, yb, FOREARM_BOTTOM, zp), "+")  # side walls
    _op(body, box(x1 - 3, x1, ya, yb, FOREARM_BOTTOM, zp), "+")
    _op(body, box(x0, x1, ya, yb, FOREARM_BOTTOM, FOREARM_BOTTOM + 3), "+")  # bottom plate
    # one deck per sheet, spanning the z range of that sheet's servos (tabs + 3 mm)
    decks = {}
    for s in routes["servos"]:
        yr = _deck_for(s, sv)
        z0, z1 = s["tab_box_mm"][2]
        key = s["sheet"]
        d = decks.setdefault(key, [yr, z0 - 2, z1 + 2])
        d[1], d[2] = min(d[1], z0 - 2), max(d[2], z1 + 2)
    for (ya_, yb_), z0, z1 in decks.values():
        _op(body, box(x0, x1, ya_, yb_, max(z0, FOREARM_BOTTOM), min(z1, zp)), "+")
    # servo pockets and screw pilot holes
    for s in routes["servos"]:
        (cx0, cx1), _cy, (cz0, cz1) = s["case_box_mm"]
        yr = _deck_for(s, sv)
        _op(
            body,
            box(
                cx0 - POCKET_CLEAR,
                cx1 + POCKET_CLEAR,
                yr[0] - 1,
                yr[1] + 1,
                cz0 - POCKET_CLEAR,
                cz1 + POCKET_CLEAR,
            ),
            "-",
        )
        x = (cx0 + cx1) / 2
        zc = (cz0 + cz1) / 2
        for dz in (-sv["hole_spacing"] / 2, sv["hole_spacing"] / 2):
            _op(body, _cyl_pts((x, yr[0] - 1, zc + dz), (x, yr[1] + 1, zc + dz), PILOT_R), "-")
    # tendon channels through the wrist plate, screw clearance holes to the palm inserts
    for x, y in routes["palm"]["inserts_xy"]:
        _op(body, _cyl_pts((x, y, zp - 1), (x, y, zw + 1), 1.7), "-")
    return body


def servo_body(s, sv):
    (cx0, cx1), (cy0, cy1), (cz0, cz1) = s["case_box_mm"]
    (tx0, tx1), (ty0, ty1), (tz0, tz1) = s["tab_box_mm"]
    body = box(cx0, cx1, cy0, cy1, cz0, cz1)
    tab = box(tx0, tx1, ty0, ty1, tz0, tz1)
    xc, zc = (tx0 + tx1) / 2, (tz0 + tz1) / 2
    for dz in (-sv["hole_spacing"] / 2, sv["hole_spacing"] / 2):
        _op(tab, _cyl_pts((xc, ty0 - 1, zc + dz), (xc, ty1 + 1, zc + dz), 1.0), "-")
    _op(body, tab, "+")
    x, y, z, sh = _servo_frame(s)
    case_top = y - sh * sv["spool_plane_above_case"]
    _op(
        body, _cyl_pts((x, case_top - sh * 0.5, z), (x, case_top + sh * 3.2, z), 1.975), "+"
    )  # spline
    return body


def spool_body(s, sv):
    """Spool on the servo spline: groove radius 5.8 (tendon centreline 6.0), flanges r 8, one tie-off
    hole. The loop leaves tangentially on both sides and wraps the bottom half."""
    x, y, z, _sh = _servo_frame(s)
    ht, fr = sv["spool_half_t"], sv["spool_flange_r"]
    body = _cyl_pts((x, y - ht, z), (x, y + ht, z), fr)
    ring = _cyl_pts((x, y - 0.6, z), (x, y + 0.6, z), fr + 0.1)
    _op(ring, _cyl_pts((x, y - 0.7, z), (x, y + 0.7, z), 5.8), "-")
    _op(body, ring, "-")
    _op(
        body, _cyl_pts((x, y - ht - 1, z), (x, y + ht + 1, z), 2.0), "-"
    )  # spline bore: 4.0 mm, prints ~3.85 = press fit on the 3.95 mm spline
    _op(
        body, _cyl_pts((x, y - ht - 1, z - 4.2), (x, y + ht + 1, z - 4.2), 0.6), "-"
    )  # tie-off hole
    _op(body, box(x - 0.4, x + 0.4, y - 0.6, y + 0.6, z - 6.0, z - 4.2), "-")  # slot groove -> hole
    return body


def stage_forearm(design, log):
    root = design.rootComponent
    if find_comp_occ(root, "forearm"):
        log("forearm: already there, skipped")
        return
    routes = load_routes()
    sv = routes["servo"]
    fo = new_part(root, "forearm", forearm_body(routes))
    cut_channels(
        fo.component,
        fo.component.bRepBodies.item(0),
        routes["strands"],
        "v1 wrist channel",
        curves_only=True,
    )  # only the S-curves reach down into the wrist plate
    occs = [find_comp_occ(root, "palm"), fo]
    for s in routes["servos"]:
        tag = f"{s['id']:02d}_{s['joint']}"
        occs.append(new_part(root, f"servo_{tag}", servo_body(s, sv)))
        occs.append(new_part(root, f"spool_{tag}", spool_body(s, sv)))
    coll = adsk.core.ObjectCollection.create()
    for o in occs:
        coll.add(o)
    root.rigidGroups.add(coll, False).name = "palm + forearm + servos"
    log(f"forearm: built with {len(routes['servos'])} servos and spools")


# ---------------------------------------------------------------------------------------------------
# Stage: thumb_base (research/experiments/2026-09-29-thumb-routing)
# The base's bottom plate and pivot move 4 mm down along the rot axis (no kinematic change). The
# bottom pin becomes a hollow journal that the thumb's sheaths and the bare cmc_flex strands come
# up through; the cmc_rot drum (r 7.5) is a groove in the plate, fed horizontally from the palm.
# A hanger from the top arm holds the 3 mm pin that turns the cmc_flex ext strand down.
# ---------------------------------------------------------------------------------------------------

OLD_PLATE_TOP = 2.5  # the prototype's base plate: z -4.4 .. 2.5, bottom pin below it
PLATE_OUTLINE_Z = -3.0  # a height where the old plate has its full outline
ROT_ANCHOR_DEG = (
    240.0  # tie hole of the cmc_rot loop (angle from +X): 20 deg of wrap left at both limits
)
HANGER_X = (0.2, 1.4)


def to_comp(occ, body):
    """Move a world-space temporary body into the occurrence's component space."""
    m = occ.transform2.copy()
    m.invert()
    tbm().transform(body, m)
    return body


def _stack(section, z0, z1, dz_list):
    out = None
    for dz in dz_list:
        piece = translate(tbm().copy(section), 0, 0, dz)
        out = piece if out is None else union(out, piece)
    return out


def thumb_base_tools(world_body, th):
    """(cut, join, cut-after) temporary bodies in world space."""
    ax, ay = th["rot_axis_xy"]
    zb, zt = th["plate_z"]  # new plate bottom / top
    rd = th["rot_drum"]
    groove = (rd["z"] - rd["groove_w"] / 2, rd["z"] + rd["groove_w"] / 2)
    walls = cut_z(
        world_body, OLD_PLATE_TOP, OLD_PLATE_TOP + 1.0
    )  # side walls just above the old plate
    walls = _stack(
        walls, 0, 0, [-k for k in range(0, 6)]
    )  # down to 2.5 - 5 = -2.5 (into the new plate)
    outline = cut_z(world_body, PLATE_OUTLINE_Z - 0.5, PLATE_OUTLINE_Z + 0.5)
    lo, hi = groove[1], zt  # the new upper plate: from the groove's top to the plate top
    plate = _stack(
        outline,
        0,
        0,
        [lo + 0.5 - PLATE_OUTLINE_Z + k * 0.5 for k in range(int((hi - lo) / 0.5) + 1)],
    )
    _op(plate, box(-50, 50, -50, 50, lo, hi), "&")
    j = th["journal"]
    join = [
        walls,
        plate,
        _cyl_pts(
            (ax, ay, groove[0]), (ax, ay, groove[1] + 0.01), rd["r"] - th["line_r"]
        ),  # groove floor
        _cyl_pts((ax, ay, groove[0] - 1.0), (ax, ay, groove[0]), rd["flange_r"]),  # lower flange
        _cyl_pts((ax, ay, j["bottom_z"]), (ax, ay, groove[0] - 0.99), j["r"]),  # journal
        box(
            *HANGER_X,
            th["cmc_pin_yz"][0] - 2.3,
            th["cmc_pin_yz"][0] + 2.3,
            th["cmc_pin_yz"][1] - 2.3,
            31.5,
        ),
    ]
    cut = [box(-40, 40, -20, 50, -30, OLD_PLATE_TOP)]
    a = math.radians(ROT_ANCHOR_DEG)
    r_tie = rd["r"] + 0.3
    py, pz = th["cmc_pin_yz"]
    after = [
        _cyl_pts((ax, ay, j["bottom_z"] - 1), (ax, ay, zt + 1), th["bore_r"]),  # the bore
        _cyl_pts(
            (ax + r_tie * math.cos(a), ay + r_tie * math.sin(a), groove[0] - 1.5),
            (ax + r_tie * math.cos(a), ay + r_tie * math.sin(a), groove[1] - 0.2),
            0.5,
        ),  # tie hole
        _cyl_pts((HANGER_X[0] - 0.5, py, pz), (HANGER_X[1] + 0.5, py, pz), th["pin_r"]),
    ]  # pin hole
    return cut, join, after


def stage_thumb_base(design, log):
    root = design.rootComponent
    occ = find_comp_occ(root, "thumb_base")
    comp = occ.component
    if comp.features.itemByName("v1 base rework"):
        hanger_round(occ, load_routes()["thumb"], log)
        log("thumb_base: already reworked, skipped")
        return
    th = load_routes()["thumb"]
    cut, join, after = thumb_base_tools(tbm().copy(occ.bRepBodies.item(0)), th)
    body = comp.bRepBodies.item(0)
    ops = adsk.fusion.FeatureOperations
    body = _feature_with_tools(
        comp, body, [to_comp(occ, t) for t in cut], ops.CutFeatureOperation, "v1 base cut old plate"
    )
    body = _feature_with_tools(
        comp, body, [to_comp(occ, t) for t in join], ops.JoinFeatureOperation, "v1 base new plate"
    )
    _feature_with_tools(
        comp, body, [to_comp(occ, t) for t in after], ops.CutFeatureOperation, "v1 base rework"
    )
    log(f"thumb_base: reworked, volume {comp.bRepBodies.item(0).volume * 1000:.0f} mm3")
    hanger_round(occ, th, log)


def hanger_round(occ, th, log):
    """Round the hanger's bottom around the pin (2.3 mm boss), so it stays > 8 mm from the cmc_flex
    axis, clear of the drum's flange (found by the interference sweep)."""
    comp = occ.component
    if comp.features.itemByName("v1 base hanger round"):
        return
    py, pz = th["cmc_pin_yz"]
    tool = box(HANGER_X[0] - 0.1, HANGER_X[1] + 0.1, py - 2.4, py + 2.4, pz - 2.4, pz)
    _op(tool, _cyl_pts((HANGER_X[0] - 1, py, pz), (HANGER_X[1] + 1, py, pz), 2.3), "-")
    _feature_with_tools(
        comp,
        comp.bRepBodies.item(0),
        [to_comp(occ, tool)],
        adsk.fusion.FeatureOperations.CutFeatureOperation,
        "v1 base hanger round",
    )
    log("thumb_base: hanger bottom rounded")


# ---------------------------------------------------------------------------------------------------
# Stage: thumb_meta (metacarpal side of cmc_flex, research/experiments/2026-09-29-thumb-routing)
# Two drum grooves over the rot axis (one per strand, the loop crosses between them through a tie
# hole), and the metacarpal cleared around them behind and above the axis so it swings past the
# strands, the pin and its hanger.
# ---------------------------------------------------------------------------------------------------

META_CLEAR_X = (-0.7, 3.3)  # slab kept to the drum (flanges included)
META_CLEAR_DEG = (-100.0, 130.0)  # sector (from +Y toward +Z, about the cmc_flex axis) cleared...
META_CLEAR_EXTRA_DEG = (130.0, 150.0)  # ...plus this (found by the interference sweep at 80 deg)
CMC_ANCHOR_DEG = 200.0  # tie hole: >= 120 deg of wrap left on both strands over -13..80


def _halfspace_yz(x0, x1, c, n):
    """Box = {p : n . (p - c) >= 0} in the y-z plane (n unit, (ny, nz)), limited to x0..x1."""
    big = 200.0
    ny, nz = n
    obb = adsk.core.OrientedBoundingBox3D.create(
        pt((x0 + x1) / 2, c[0] + ny * big / 2, c[1] + nz * big / 2),
        vec(1, 0, 0),
        vec(0, ny, nz),
        (x1 - x0) / 10,
        big / 10,
        big / 10,
    )
    return tbm().createBox(obb)


def _sector_yz(x0, x1, c, a0, a1):
    """Wedge a0..a1 (deg, < 180 wide) about the X-parallel line through c = (y, z)."""
    r0, r1 = math.radians(a0), math.radians(a1)
    w = _halfspace_yz(x0, x1, c, (-math.sin(r0), math.cos(r0)))
    return _op(w, _halfspace_yz(x0, x1, c, (math.sin(r1), -math.cos(r1))), "&")


def thumb_meta_tools(th):
    cy, cz = th["cmc_flex_axis_yz"]
    d = th["cmc_drum"]
    x0, x1 = META_CLEAR_X
    a0, a1 = META_CLEAR_DEG
    mid = (a0 + a1) / 2
    clear = union(_sector_yz(x0, x1, (cy, cz), a0, mid), _sector_yz(x0, x1, (cy, cz), mid, a1))
    _op(clear, _cyl_pts((x0 - 1, cy, cz), (x1 + 1, cy, cz), d["flange_r"]), "-")
    tools = [clear]
    for gx in d["groove_x"].values():
        ring = _cyl_pts(
            (gx - d["groove_w"] / 2, cy, cz), (gx + d["groove_w"] / 2, cy, cz), d["flange_r"] + 0.1
        )
        tools.append(
            _op(ring, _cyl_pts((gx - 1, cy, cz), (gx + 1, cy, cz), d["r"] - th["line_r"]), "-")
        )
    a = math.radians(CMC_ANCHOR_DEG)
    ry = d["r"] - th["line_r"] - 0.2
    gxs = sorted(d["groove_x"].values())
    tools.append(
        _cyl_pts(
            (gxs[0], cy + ry * math.cos(a), cz + ry * math.sin(a)),
            (gxs[-1], cy + ry * math.cos(a), cz + ry * math.sin(a)),
            0.5,
        )
    )
    return tools


def stage_thumb_meta(design, log):
    root = design.rootComponent
    occ = find_comp_occ(root, "thumb_metacarpal")
    comp = occ.component
    th = load_routes()["thumb"]
    cut = adsk.fusion.FeatureOperations.CutFeatureOperation
    if not comp.features.itemByName("v1 meta cmc drum"):
        tools = [to_comp(occ, t) for t in thumb_meta_tools(th)]
        _feature_with_tools(comp, comp.bRepBodies.item(0), tools, cut, "v1 meta cmc drum")
        log(f"thumb_meta: drum cut, volume {comp.bRepBodies.item(0).volume * 1000:.0f} mm3")
    if not comp.features.itemByName("v1 meta clear top"):
        cy, cz = th["cmc_flex_axis_yz"]
        tool = _sector_yz(*META_CLEAR_X, (cy, cz), *META_CLEAR_EXTRA_DEG)
        _op(
            tool,
            _cyl_pts(
                (META_CLEAR_X[0] - 1, cy, cz),
                (META_CLEAR_X[1] + 1, cy, cz),
                th["cmc_drum"]["flange_r"],
            ),
            "-",
        )
        _feature_with_tools(
            comp, comp.bRepBodies.item(0), [to_comp(occ, tool)], cut, "v1 meta clear top"
        )
        log(f"thumb_meta: top cleared, volume {comp.bRepBodies.item(0).volume * 1000:.0f} mm3")


# ---------------------------------------------------------------------------------------------------
# Stage: thumb_inner (the thumb's tendon paths past the base, research/experiments/2026-09-29-thumb-
# routing). All geometry comes from tendon_routes.json "thumb" (tendon_router.thumb_inner()):
# - metacarpal: a solid block (it was thin in front), a socket block on each side of the cmc_flex
#   gap with the 4 sheath sockets (2.2 mm, the step to 1.2 mm stops the tube), 2 mm steel pins
#   (pin A per side, pin B per mcp_flex line) with a groove around each where a line wraps, and
#   1.2 mm bores along the straight runs. The slab behind the cmc_flex drum is cleared again.
# - proximal: the MCP drum groove (r 5.8 floor), its tie hole, and for each ip line a 1.2 mm hole
#   exactly on the MCP axis, a fan-shaped slot behind it (the line swings with the joint) and a
#   bore straight on to its IP groove.
# - distal: the two IP drum grooves (r 5.8 floor) and their tie hole.
# ---------------------------------------------------------------------------------------------------

S2 = math.sqrt(0.5)
THUMB_A = (S2, 0.0, S2)  # MCP / IP axis direction
THUMB_W = (-S2, 0.0, S2)  # across it, the closing side
TIE_ANGLE_DEG = -45.0  # MCP and IP loop tie point: >= 20 deg of wrap on both strands at -5 / 95


def _v(*a):
    return tuple(float(x) for x in a)


def _add(p, *terms):
    x, y, z = p
    for k, d in terms:
        x, y, z = x + k * d[0], y + k * d[1], z + k * d[2]
    return (x, y, z)


def _halfspace(c, n, size=200.0):
    """Box = {p : n . (p - c) >= 0}, n a unit 3-tuple."""
    nv = vec(*n)
    other = vec(1, 0, 0) if abs(n[0]) < 0.9 else vec(0, 1, 0)
    w = nv.crossProduct(other)
    w.normalize()
    obb = adsk.core.OrientedBoundingBox3D.create(pt(*_add(c, (size / 2, n))), nv, w, size / 10, size / 10, size / 10)
    return tbm().createBox(obb)


def _thumb_pt(th, a, w, y):
    cx, cz = th["cl_xz"]
    return _add((cx, y, cz), (a, THUMB_A), (w, THUMB_W))


def _ring(center, axis, a0, a1, r_in, r_out):
    """Groove tool: annulus r_in..r_out around `axis` through `center`, from a0 to a1 along it."""
    ring = tbm().createCylinderOrCone(pt(*_add(center, (a0, axis))), r_out / 10, pt(*_add(center, (a1, axis))), r_out / 10)
    core = tbm().createCylinderOrCone(pt(*_add(center, (a0 - 1, axis))), r_in / 10, pt(*_add(center, (a1 + 1, axis))), r_in / 10)
    return _op(ring, core, "-")


def _seg(a, b, r):
    return tbm().createCylinderOrCone(pt(*a), r / 10, pt(*b), r / 10)


def _poly_prism_yz(x0, x1, pts):
    """Convex polygon (y, z) points, counter-clockwise in (y right, z up), extruded along x."""
    body = None
    n = len(pts)
    for i in range(n):
        (y0, z0), (y1, z1) = pts[i], pts[(i + 1) % n]
        dy, dz = y1 - y0, z1 - z0
        L = math.hypot(dy, dz)
        inward = (-dz / L, dy / L)  # left of the edge = inside for a CCW polygon
        hs = _halfspace_yz(x0, x1, (y0, z0), inward)
        body = hs if body is None else _op(body, hs, "&")
    return body


def _tie_point(center, r, ang_deg):
    u, w = (0.0, -1.0, 0.0), THUMB_W
    a = math.radians(ang_deg)
    return _add(center, (r * math.cos(a), u), (r * math.sin(a), w))


def thumb_inner_tools(th):
    """{part: (join tools, cut tools)} in world space."""
    inn, fill = th["inner"], th["meta_fill"]
    sd = th["socket_dir_yz"]
    rho = th["pin2_r"] + th["line_r"]
    x0, x1 = fill["x"]
    (y0, y1), (z0, z1) = fill["y"], fill["z"]
    yf, zf = fill["front"]
    join_m = [box(x0, x1, yf, y1, z0, z1), box(x0, x1, y0, yf, z0, zf)]
    cut_m, cut_p, cut_d = [], [], []
    e = (S2, S2)  # (y, z) across the socket direction, up-back
    for side in ("flex", "ext"):
        strands = [k for k in inn if k.endswith("_" + side)]
        s0 = inn[strands[0]]
        sy, sz = s0["socket"]
        ty, tz = s0["tube_end"]
        ay, az = s0["pin_a"]
        p1, p2 = (sy - 2.0 * e[0], sz - 2.0 * e[1]), (sy + 2.0 * e[0], sz + 2.0 * e[1])
        p3, p4, p5 = (ty + 2.0 * e[0], tz + 2.0 * e[1]), (ay, az + 2.3), (ay - 2.3, az)
        p6, p7 = (ay - 2.3, 20.0), (p1[0], 20.0)
        # two convex pieces (counter-clockwise in y right, z up); the notch under p1 stays open for
        # the incoming tube
        bx0, bx1 = th["pin_a_x"][side]
        bx = (x0 + 0.1, bx1) if side == "flex" else (bx0, x1 - 0.1)
        join_m.append(_poly_prism_yz(*bx, [p1, p2, p3, p4, p5]))
        join_m.append(_poly_prism_yz(*bx, [p5, p6, p7, p1]))
        cut_m.append(_seg((bx0 - 1.0, ay, az), (bx1 + 1.0, ay, az), th["pin2_r"]))  # pin A hole
    for name, t in inn.items():
        side = name.rsplit("_", 1)[1]
        x = t["x"]
        sy, sz = t["socket"]
        ty, tz = t["tube_end"]
        cut_m.append(_seg((x, sy - 1.0 * sd[0], sz - 1.0 * sd[1]), (x, ty, tz), th["bore_tube"] / 2))  # socket
        ay, az = t["pin_a"]
        cut_m.append(_seg((x - 0.6, ay, az), (x + 0.6, ay, az), rho + 0.6))  # line groove around pin A
        for p0, p1 in th["bores"][name]:
            cut_m.append(_seg(tuple(p0), tuple(p1), th["bore_small"] / 2))
        if "pin_b" in t:
            by, bz = t["pin_b"]
            lx = t["line_x"]
            px0, px1 = (x0 - 1.0, lx + 1.2) if side == "flex" else (lx - 1.2, x1 + 1.0)
            cut_m.append(_seg((px0, by, bz), (px1, by, bz), th["pin2_r"]))  # pin B hole, from outside
            cut_m.append(_seg((lx - 0.6, by, bz), (lx + 0.6, by, bz), rho + 0.6))
    # proximal: MCP drum groove + tie hole, ip crossing holes, fan slots and bores
    ga = th["mcp_groove_a"]
    hw = th["groove_half_w"]
    r_floor = th["thumb_drum_r"] - th["line_r"]
    mcp_c = _thumb_pt(th, 0.0, 0.0, th["mcp_y"])
    ip_c = _thumb_pt(th, 0.0, 0.0, th["ip_y"])
    cut_p.append(_ring(mcp_c, THUMB_A, ga - hw, ga + hw, r_floor, 7.7))
    tie = _tie_point(_thumb_pt(th, ga, 0.0, th["mcp_y"]), r_floor - 0.3, TIE_ANGLE_DEG)
    cut_p.append(_seg(_add(tie, (-0.5, THUMB_A)), _add(tie, (3.5, THUMB_A)), 0.5))
    for side in ("flex", "ext"):
        t = inn["thumb_ip_" + side]
        a = t["cross_a"]
        cross = tuple(t["cross"])
        cut_p.append(_seg(cross, tuple(t["ip_tangent"]), th["cross_hole_d"] / 2))  # bore to the IP groove
        # fan behind the hole: the incoming line (from pin A) turns with the MCP joint, -5..95 deg
        start = (t["x"], *th["pin_a_out"][f"thumb_ip_{side}"])
        d = [start[i] - cross[i] for i in range(3)]
        L = math.sqrt(sum(c * c for c in d))
        d = [c / L for c in d]
        for q_deg in range(-10, 101, 4):  # rotate d about the closing axis (-A) by -q (proximal's view)
            q = math.radians(-q_deg)
            k = (-THUMB_A[0], -THUMB_A[1], -THUMB_A[2])
            kd = sum(k[i] * d[i] for i in range(3))
            kxd = (k[1] * d[2] - k[2] * d[1], k[2] * d[0] - k[0] * d[2], k[0] * d[1] - k[1] * d[0])
            dr = tuple(d[i] * math.cos(q) + kxd[i] * math.sin(q) + k[i] * kd * (1 - math.cos(q)) for i in range(3))
            cut_p.append(_seg(_add(cross, (0.4, dr)), _add(cross, (12.0, dr)), th["cross_hole_d"] / 2 + 0.1))
        # distal: its groove
        cut_d.append(_ring(ip_c, THUMB_A, a - hw, a + hw, r_floor, 7.7))
    a_lo, a_hi = sorted(th["ip_groove_a"].values())
    tie = _tie_point(_thumb_pt(th, a_lo, 0.0, th["ip_y"]), r_floor - 0.3, TIE_ANGLE_DEG)
    cut_d.append(_seg(_add(tie, (-0.5, THUMB_A)), _add(tie, (a_hi - a_lo + 0.5, THUMB_A)), 0.5))
    return {"thumb_metacarpal": (join_m, cut_m), "thumb_proximal": ([], cut_p), "thumb_distal": ([], cut_d)}


def _rot_about(v, k, ang):
    """Rotate vector v about unit axis k by ang (rad), Rodrigues."""
    kd = sum(k[i] * v[i] for i in range(3))
    kxv = (k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0])
    c, s_ = math.cos(ang), math.sin(ang)
    return tuple(v[i] * c + kxv[i] * s_ + k[i] * kd * (1 - c) for i in range(3))


def mcp_relief_tools(th):
    """Proximal: a 1.4 mm slot where each mcp_flex line sweeps past the back of the phalanx as the
    MCP closes (-10..100 deg). The line is fixed to the metacarpal; seen from the proximal it turns
    the other way about the MCP axis (found by the clearance check: it touched from 70 deg)."""
    mcp_c = _thumb_pt(th, 0.0, 0.0, th["mcp_y"])
    k = (-THUMB_A[0], -THUMB_A[1], -THUMB_A[2])  # closing axis
    tools = []
    for side in ("flex", "ext"):
        tan = th["inner"]["thumb_mcp_flex_" + side]["drum_tangent"]
        # from 3 mm behind the tangent (so the slot never cuts the drum's groove floor) to 16 mm
        start = _add(tuple(tan), (3.0, (0.0, 1.0, 0.0)))
        back = _add(tuple(tan), (16.0, (0.0, 1.0, 0.0)))
        for q_deg in range(-10, 101, 3):
            q = math.radians(-q_deg)
            p0 = _add(mcp_c, (1.0, _rot_about(tuple(start[i] - mcp_c[i] for i in range(3)), k, q)))
            p1 = _add(mcp_c, (1.0, _rot_about(tuple(back[i] - mcp_c[i] for i in range(3)), k, q)))
            tools.append(_seg(p0, p1, 0.7))
    return tools


def stage_thumb_inner(design, log):
    root = design.rootComponent
    meta = find_comp_occ(root, "thumb_metacarpal")
    if meta.component.features.itemByName("v1 meta inner"):
        mcp_relief(design, load_routes()["thumb"], log)
        log("thumb_inner: already built, skipped")
        return
    th = load_routes()["thumb"]
    ops = adsk.fusion.FeatureOperations
    for part, (join, cut) in thumb_inner_tools(th).items():
        occ = find_comp_occ(root, part)
        comp = occ.component
        body = comp.bRepBodies.item(0)
        if join:
            body = _feature_with_tools(comp, body, [to_comp(occ, t) for t in join], ops.JoinFeatureOperation, "v1 meta block")
        name = "v1 meta inner" if part == "thumb_metacarpal" else f"v1 {part.split('_', 1)[1]} inner"
        _feature_with_tools(comp, body, [to_comp(occ, t) for t in cut], ops.CutFeatureOperation, name)
        log(f"thumb_inner: {part} done, volume {comp.bRepBodies.item(0).volume * 1000:.0f} mm3")
    # the block covers the slab behind the cmc_flex drum again: clear it like stage thumb_meta did
    cy, cz = th["cmc_flex_axis_yz"]
    tools = []
    for a0, a1 in ((META_CLEAR_DEG[0], 15.0), (15.0, META_CLEAR_EXTRA_DEG[1])):
        w = _sector_yz(*META_CLEAR_X, (cy, cz), a0, a1)
        _op(w, _cyl_pts((META_CLEAR_X[0] - 1, cy, cz), (META_CLEAR_X[1] + 1, cy, cz), th["cmc_drum"]["flange_r"]), "-")
        tools.append(w)
    comp = meta.component
    _feature_with_tools(comp, comp.bRepBodies.item(0), [to_comp(meta, t) for t in tools], ops.CutFeatureOperation, "v1 meta clear slab again")
    log(f"thumb_inner: metacarpal slab cleared, volume {comp.bRepBodies.item(0).volume * 1000:.0f} mm3")
    mcp_relief(design, th, log)


def mcp_relief(design, th, log):
    prox = find_comp_occ(design.rootComponent, "thumb_proximal")
    comp = prox.component
    if comp.features.itemByName("v1 proximal mcp relief"):
        return
    tools = [to_comp(prox, t) for t in mcp_relief_tools(th)]
    _feature_with_tools(comp, comp.bRepBodies.item(0), tools, adsk.fusion.FeatureOperations.CutFeatureOperation, "v1 proximal mcp relief")
    log(f"thumb_inner: proximal relief for the mcp_flex lines, volume {comp.bRepBodies.item(0).volume * 1000:.0f} mm3")


# ---------------------------------------------------------------------------------------------------
# Stage: knuckle (owner, 2026-10-01): the finger mcp_flex drum grows from r 6 to the router's r 7
# (14 mm tendon circle, servo_per_joint 7 / 5 = 1.4). The drum is the proximal phalanx's rounded end
# (r 7.5, tongue x +3..+8 from the flex boss face X_FLEX) with a 1 mm groove at x +4..+5 that runs all
# the way round: a tunnel over the top, and a side window at x +3..+4 there to thread and tie the line.
# The new groove keeps that shape: floor r 6.8 (line centre r 7), 1 mm rims at r 8 on both sides
# (0.5 mm proud of the finger's front and back faces; owner OK'd, 2026-10-02).
# ---------------------------------------------------------------------------------------------------

KNUCKLE_GROOVE_X = (4.0, 5.0)  # from X_FLEX (index: -10.03 .. -9.03)
KNUCKLE_RIMS_X = ((3.0, 4.0), (5.0, 5.6))  # the right rim stops 0.5 mm short of the on-axis slit
KNUCKLE_RIM_R = 8.0
KNUCKLE_TIE_HALF_Y = 2.3  # tie window over the top: +-19 deg of the groove, like the prototype's


def knuckle_tools(drum_r, line_r, dx, dz):
    """(join, cut) temporary bodies for one finger's knuckle drum, in its component space (which is
    world space at q = 0). dx, dz: the finger's offset from the index (FINGERS)."""
    y, z = Y_FLEX, Z_MCP + dz
    x = X_FLEX + dx
    floor = drum_r - line_r

    def cyl(x0, x1, r):
        return _cyl_pts((x + x0, y, z), (x + x1, y, z), r)

    g0, g1 = KNUCKLE_GROOVE_X
    t0 = KNUCKLE_RIMS_X[0][0]
    # fill the old groove, its top tunnel and tie window up to the new floor
    join = [cyl(t0, g1, floor), box(x + t0, x + g1, y - 6.5, y + 6.5, z, z + floor)]
    join += [cyl(a, b, KNUCKLE_RIM_R) for a, b in KNUCKLE_RIMS_X]
    cut = [_ring((x, y, z), (1.0, 0.0, 0.0), g0, g1, floor, KNUCKLE_RIM_R + 0.6)]
    window = _ring((x, y, z), (1.0, 0.0, 0.0), t0 - 0.5, g0 + 0.01, floor, KNUCKLE_RIM_R + 0.6)
    _op(window, box(x - 1, x + 10, y - KNUCKLE_TIE_HALF_Y, y + KNUCKLE_TIE_HALF_Y, z, z + 20), "&")
    cut.append(window)
    return join, cut


def stage_knuckle(design, log):
    root = design.rootComponent
    routes = load_routes()
    line_r = routes["thumb"]["line_r"]
    ops = adsk.fusion.FeatureOperations
    for finger in ("index", *FINGERS):
        occ = find_comp_occ(root, f"{finger}_proximal")
        comp = occ.component
        if comp.features.itemByName("v1 knuckle drum"):
            log(f"knuckle: {finger} already done, skipped")
            continue
        drum_r = routes["drums_mm"][f"{finger}_mcp_flex"]
        off = FINGERS.get(finger, {"dx": 0.0, "dz": 0.0})
        # the old r 6 groove must be where we expect it (else the design changed: stop)
        x, z = X_FLEX + off["dx"], Z_MCP + off["dz"]
        found = False
        for f in comp.bRepBodies.item(0).faces:
            g = f.geometry
            if not isinstance(g, adsk.core.Cylinder) or abs(g.radius * 10 - 6.0) > 0.01:
                continue
            bb = f.boundingBox
            if (
                abs(g.origin.y * 10 - Y_FLEX) < 0.05
                and abs(g.origin.z * 10 - z) < 0.05
                and bb.minPoint.x * 10 > x + 3.9
                and bb.maxPoint.x * 10 < x + 5.1
            ):
                found = True
        if not found:
            raise RuntimeError(f"knuckle: no r 6 groove on {finger}_proximal at x {x + 4:.2f}")
        join, cut = knuckle_tools(drum_r, line_r, off["dx"], off["dz"])
        body = _feature_with_tools(comp, comp.bRepBodies.item(0), join, ops.JoinFeatureOperation, "v1 knuckle fill")
        _feature_with_tools(comp, body, cut, ops.CutFeatureOperation, "v1 knuckle drum")
        log(f"knuckle: {finger} drum r {drum_r} (floor {drum_r - line_r:.1f}), volume {comp.bRepBodies.item(0).volume * 1000:.0f} mm3")


# ---------------------------------------------------------------------------------------------------
# Stage: linkage (owner, 2026-10-02): each finger's DIP-PIP coupling bar (hardware/cad/dip_linkage.py)
# in a layer outside the finger's -x side face, because the PIP and DIP bearings fill the joints:
# plate A glued on the proximal phalanx (over the PIP bearing, which it holds in), plate B glued on
# the distal phalanx (relieved over the middle phalanx's DIP cheek), 1.5 mm steel pins, and the bar.
# Each plate has a 1.5 mm peg hole into its phalanx to line it up. Pin positions: middle phalanx
# frame, x from the PIP axis toward the DIP axis (= +z here), y toward the back (= +y).
# ---------------------------------------------------------------------------------------------------

PLATE_R = 6.0  # plate disc around its joint axis
PLATE_HALF_Y = 5.5
PLATE_A_DOWN = 12.0  # plate A runs this far down the proximal phalanx from the PIP axis
PLATE_B_UP = 14.0  # plate B runs this far up the distal phalanx from the DIP axis
PEG_A, PEG_B = 8.0, 11.5  # peg holes: below the PIP axis / beyond the DIP axis
PEG_D, PEG_DEPTH = 1.5, 3.0
PIN_FIT_D, PIN_RUN_D = 1.45, 1.6  # press fit in the plates, running fit in the bar
PIN_SKIN = 0.4  # plate material left behind each pin hole


def finger_axes(finger):
    """(x of the -x side face, y of the flex axes, z of the PIP axis, z of the DIP axis), mm."""
    f = FINGERS.get(finger, {"dx": 0.0, "dz": 0.0, "prox": 0.0, "mid": 0.0})
    z_pip = Z_PIP + f["dz"] + f["prox"]
    return X_FLEX + f["dx"], Y_FLEX, z_pip, Z_DIP + f["dz"] + f["prox"] + f["mid"]


def _pin(center_yz, pin):
    g = math.radians(pin["angle_deg"])
    return center_yz[0] + pin["r_mm"] * math.sin(g), center_yz[1] + pin["r_mm"] * math.cos(g)


def linkage_tools(finger, link, lay):
    """World-space temporary bodies: {"plate_a", "plate_b", "bar": body, "cut_proximal",
    "cut_distal": [tools]} and the pin centres (y, z)."""
    x0, y, zp, zd = finger_axes(finger)
    t, g, bt = lay["plate_t"], lay["gap"], lay["bar_t"]
    xa, xb = x0 - t, x0  # plates
    pa, pb = _pin((y, zp), link["pin_a"]), _pin((y, zd), link["pin_b"])

    def xcyl(x_lo, x_hi, yy, zz, r):
        return _cyl_pts((x_lo, yy, zz), (x_hi, yy, zz), r)

    def pin_hole(p):  # blind, from the outside
        return xcyl(xa - 0.5, xb - PIN_SKIN, p[0], p[1], PIN_FIT_D / 2)

    plate_a = _op(xcyl(xa, xb, y, zp, PLATE_R), box(xa, xb, y - PLATE_HALF_Y, y + PLATE_HALF_Y, zp - PLATE_A_DOWN, zp), "+")
    for tool in (
        xcyl(xb - 0.3, xb + 0.1, y, zp, lay["bearing_recess_r"]),
        pin_hole(pa),
        xcyl(xa - 0.5, xb + 0.1, y, zp - PEG_A, PEG_D / 2),
    ):
        _op(plate_a, tool, "-")
    plate_b = _op(xcyl(xa, xb, y, zd, PLATE_R), box(xa, xb, y - PLATE_HALF_Y, y + PLATE_HALF_Y, zd, zd + PLATE_B_UP), "+")
    for tool in (
        xcyl(xb - 0.3, xb + 0.1, y, zd, lay["relief_r"]),
        pin_hole(pb),
        xcyl(xa - 0.5, xb + 0.1, y, zd + PEG_B, PEG_D / 2),
    ):
        _op(plate_b, tool, "-")
    # bar: stadium from pin A to pin B, with running-fit holes
    bx0, bx1 = xa - g - bt, xa - g
    w = lay["bar_w"] / 2
    (ya, za), (yb, zb) = pa, pb
    length = math.hypot(yb - ya, zb - za)
    uy, uz = (yb - ya) / length, (zb - za) / length
    ny, nz = -uz, uy
    corners = [(ya + ny * w, za + nz * w), (ya - ny * w, za - nz * w), (yb - ny * w, zb - nz * w), (yb + ny * w, zb + nz * w)]
    if (corners[1][0] - corners[0][0]) * (corners[2][1] - corners[1][1]) - (corners[1][1] - corners[0][1]) * (corners[2][0] - corners[1][0]) < 0:
        corners.reverse()  # counter-clockwise for _poly_prism_yz
    bar = _poly_prism_yz(bx0, bx1, corners)
    for p in (pa, pb):
        _op(bar, xcyl(bx0, bx1, p[0], p[1], w), "+")
    for p in (pa, pb):
        _op(bar, xcyl(bx0 - 1, bx1 + 1, p[0], p[1], PIN_RUN_D / 2), "-")
    cut_prox = [xcyl(x0 - 0.1, x0 + PEG_DEPTH, y, zp - PEG_A, PEG_D / 2)]
    cut_dist = [xcyl(x0 - 0.1, x0 + PEG_DEPTH, y, zd + PEG_B, PEG_D / 2)]
    return {"plate_a": plate_a, "plate_b": plate_b, "bar": bar, "cut_proximal": cut_prox, "cut_distal": cut_dist}, (pa, pb, length)


def _rigid(root, name, a, b):
    for j in root.asBuiltJoints:
        if j.name == name:
            return
    ji = root.asBuiltJoints.createInput(a, b, None)
    ji.setAsRigidJointMotion()
    root.asBuiltJoints.add(ji).name = name


def _check_contact(occ, pts, what):
    """Every point (world mm) must be inside `occ`'s body: the plate sits on material."""
    body = occ.bRepBodies.item(0)
    inside = adsk.fusion.PointContainment.PointInsidePointContainment
    bad = [p for p in pts if body.pointContainment(pt(*p)) != inside]
    if bad:
        raise RuntimeError(f"linkage: {what}: {len(bad)} of {len(pts)} contact points are not on {occ.component.name}, e.g. {bad[0]}")


def stage_linkage(design, log):
    root = design.rootComponent
    coupling = load_routes()["coupling"]
    lay = coupling["layout"]
    for finger in ("index", *FINGERS):
        if find_comp_occ(root, f"{finger}_link_bar"):
            log(f"linkage: {finger} already there, skipped")
            continue
        link = coupling["linkages"][finger]
        x0, y, zp, zd = finger_axes(finger)
        prox, dist = (find_comp_occ(root, f"{finger}_{k}") for k in ("proximal", "distal"))
        # the plates must sit on their own phalanx (0.2 mm inside the side face). Plate A: below the
        # PIP axis and on the cheek's two horns (above the bearing the cheek is a U-slot, open toward
        # the fingertip, that the middle phalanx's axle slides into). Plate B: beyond its relief.
        xc = x0 + 0.2
        on_a = [(dy, dz) for dy in (-5.0, -2.5, 0.0, 2.5, 5.0) for dz in (-11.5, -9.0, -6.5)]
        on_a += [(dy, dz) for dy in (-5.0, 5.0) for dz in (-1.0, 1.0)]
        on_b = [(dy, dz) for dy in (-4.0, -2.0, 0.0, 2.0, 4.0) for dz in (10.0, 11.5, 13.0)]
        _check_contact(prox, [(xc, y + dy, zp + dz) for dy, dz in on_a], "plate A")
        _check_contact(dist, [(xc, y + dy, zd + dz) for dy, dz in on_b], "plate B")
        tools, (pa, pb, length) = linkage_tools(finger, link, lay)
        if abs(length - link["bar_mm"]) > 0.01:
            raise RuntimeError(f"linkage: {finger} bar {length:.3f} mm, design {link['bar_mm']}")
        ops = adsk.fusion.FeatureOperations
        for occ, key, label in ((prox, "cut_proximal", "proximal"), (dist, "cut_distal", "distal")):
            comp = occ.component
            _feature_with_tools(comp, comp.bRepBodies.item(0), tools[key], ops.CutFeatureOperation, f"v1 link peg {label}")
        plate_a = new_part(root, f"{finger}_link_plate_a", tools["plate_a"])
        plate_b = new_part(root, f"{finger}_link_plate_b", tools["plate_b"])
        new_part(root, f"{finger}_link_bar", tools["bar"])
        _rigid(root, f"{finger}_link_plate_a_fix", plate_a, prox)
        _rigid(root, f"{finger}_link_plate_b_fix", plate_b, dist)
        log(f"linkage: {finger} plates + bar {length:.2f} mm, pin A (y {pa[0]:.2f}, z {pa[1]:.2f}), pin B (y {pb[0]:.2f}, z {pb[1]:.2f})")


STAGES = {
    "fingers": stage_fingers,
    "thumb": stage_thumb,
    "names": stage_names,
    "thumb_base": stage_thumb_base,
    "thumb_meta": stage_thumb_meta,
    "palm": stage_palm,
    "forearm": stage_forearm,
    "thumb_inner": stage_thumb_inner,
    "knuckle": stage_knuckle,
    "linkage": stage_linkage,
}
EXTRA_STAGES_MIGRATE = {"thumb_unhinge": stage_thumb_unhinge}  # one-off, for designs built before
EXTRA_STAGES = {"export": stage_export}  # run on demand, not by run()


def run_stage(name):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    doc = app.activeDocument
    doc_name = (
        doc.dataFile.name if doc.dataFile else doc.name
    )  # the tab title updates only on reopen
    if doc_name != "Tendra Hand V1":
        raise RuntimeError("open the 'Tendra Hand V1' design first (not the original prototype)")
    lines = []
    {**STAGES, **EXTRA_STAGES, **EXTRA_STAGES_MIGRATE}[name](design, lines.append)
    return lines


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        answer = ui.messageBox(
            "Build the Tendra Hand V1 stages (" + ", ".join(STAGES) + ") in this design?",
            "Tendra Hand V1",
            adsk.core.MessageBoxButtonTypes.YesNoButtonType,
        )
        if answer != adsk.core.DialogResults.DialogYes:
            return
        out = []
        for name in STAGES:
            out += run_stage(name)
        ui.messageBox("Done.\n" + "\n".join(out))
    except Exception:
        ui.messageBox("TendraHandV1 failed:\n" + traceback.format_exc())


if "TENDRA_STAGE" in globals():  # driven from the Fusion MCP
    print("\n".join(run_stage(TENDRA_STAGE)))  # noqa: F821
