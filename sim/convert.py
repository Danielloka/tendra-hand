"""Convert the raw Fusion 360 URDF export into a clean MuJoCo model (MJCF).

The export in hardware/robot_description/fusion_export/ is never edited by hand.
All fixes live here, so a fresh export from Fusion only needs a re-run:

    uv run python sim/convert.py

Fixes applied:
- Descriptive joint names (index_dip, thumb_cmc_rot, ...)
- Flexion-positive sign convention on every joint (q = 0 is a straight joint)
- Mass and inertia recomputed from the meshes at PLA density
  (the export uses steel and rounds inertias to 1e-6)
- Joint damping/armature and position actuators (one per motor, in motor order)
"""

from __future__ import annotations

import math
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EXPORT_DIR = ROOT / "hardware" / "robot_description" / "fusion_export"
URDF_PATH = EXPORT_DIR / "urdf" / "Hand.xacro"
MESH_DIR = EXPORT_DIR / "meshes"
OUT_PATH = ROOT / "sim" / "models" / "tendra_hand.xml"

# Solid PLA is ~1240 kg/m^3. Printed parts are lighter (infill); update once parts are weighed.
DENSITY = 1240.0


@dataclass(frozen=True)
class JointSpec:
    urdf_name: str
    name: str
    flip: bool  # True: invert axis so that "closing the hand" is positive (see below)
    motor: int  # ESP32 motor number, see CLAUDE.md pin map


# Sign convention: positive = closing the hand. Flexion joints bend toward the palm (-Y);
# thumb_cmc_rot swings the thumb across the palm (opposition, toward -X where the other fingers
# will be); index_mcp_abd moves the index toward the thumb side (+X).
# Order = motor order (M1..M8), which is also the actuator/control order.
JOINTS = [
    JointSpec("Revolute 4", "index_dip", flip=False, motor=1),
    JointSpec("Revolute 3", "index_pip", flip=False, motor=2),
    JointSpec("Revolute 2", "index_mcp_flex", flip=False, motor=3),
    JointSpec("Revolute 1", "index_mcp_abd", flip=False, motor=4),
    JointSpec("Revolute 8", "thumb_ip", flip=False, motor=5),
    JointSpec("Revolute 7", "thumb_mcp", flip=True, motor=6),
    JointSpec("Revolute 6", "thumb_cmc_flex", flip=True, motor=7),
    JointSpec("Revolute 5", "thumb_cmc_rot", flip=False, motor=8),
]

LINK_NAMES = {
    "base_link": "palm",
    "Indexfix_1": "index_base",
    "Indexend_1": "index_proximal",
    "Indexmiddle_1": "index_middle",
    "Indextip_1": "index_distal",
    "Thumbroll_1": "thumb_base",
    "Thumbend_1": "thumb_metacarpal",
    "Thumbmiddle_1": "thumb_proximal",
    "Thumbtip_1": "thumb_distal",
}

# Position actuators stand in for the stepper + tendon: stiff position control, limited torque.
# 28BYJ-48 gives ~0.03 N*m; through a ~1 cm spool and a small joint moment arm this is a few
# hundredths of N*m at the joint. Tune after system identification (roadmap phase 3).
ACTUATOR_KP = 0.5  # N*m/rad
ACTUATOR_FORCE_LIMIT = 0.05  # N*m
JOINT_DAMPING = 0.01  # N*m*s/rad
JOINT_ARMATURE = 1e-4  # kg*m^2, adds rotor-like inertia and keeps tiny links numerically stable

# Meshes with small defects that "exact" inertia rejects. Indexfix_1.stl has 2 open edges;
# fix it in Fusion and remove it from this list.
LEGACY_INERTIA_MESHES = {"Indexfix_1"}

# Body pairs that only collide because of convex-hull approximation (checked by joint sweeps).
HULL_ARTIFACT_EXCLUDES = [("palm", "thumb_metacarpal")]


def _floats(text: str | None, default: str = "0 0 0") -> np.ndarray:
    return np.array([float(v) for v in (text or default).split()])


def _fmt(values) -> str:
    return " ".join(f"{v:.6g}" for v in np.asarray(values, dtype=float).ravel())


def rpy_to_quat(rpy: np.ndarray) -> np.ndarray:
    """URDF roll-pitch-yaw (fixed-axis XYZ) to a MuJoCo quaternion (w, x, y, z)."""
    r, p, y = rpy / 2.0
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def stl_vertices(path: Path, scale: np.ndarray) -> np.ndarray:
    """Vertices (N x 3, metres) of a binary STL."""
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    tris = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return tris["v"].reshape(-1, 3).astype(float) * scale


def parse_urdf(path: Path):
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall("link"):
        visual = link.find("visual")
        mesh = visual.find("geometry/mesh")
        links[link.get("name")] = {
            "mesh": Path(mesh.get("filename")).name,
            "scale": _floats(mesh.get("scale"), "1 1 1"),
            "pos": _floats(visual.find("origin").get("xyz")),
            "rpy": _floats(visual.find("origin").get("rpy")),
        }
    joints = {}
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            raise ValueError(f"Unsupported joint type {joint.get('type')!r} for {joint.get('name')!r}")
        origin = joint.find("origin")
        limit = joint.find("limit")
        joints[joint.get("name")] = {
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "pos": _floats(origin.get("xyz")),
            "rpy": _floats(origin.get("rpy")),
            "axis": _floats(joint.find("axis").get("xyz"), "1 0 0"),
            "range": (float(limit.get("lower")), float(limit.get("upper"))),
        }
    return links, joints


def build_mjcf(links: dict, joints: dict) -> ET.Element:
    specs = {s.urdf_name: s for s in JOINTS}
    missing = set(joints) ^ set(specs)
    if missing:
        raise ValueError(f"Joint map and URDF disagree on: {sorted(missing)}")

    mujoco = ET.Element("mujoco", model="tendra_hand")
    ET.SubElement(mujoco, "compiler", angle="radian", meshdir=_relpath(MESH_DIR, OUT_PATH.parent), autolimits="true")
    ET.SubElement(mujoco, "option", timestep="0.002", integrator="implicitfast")

    default = ET.SubElement(mujoco, "default")
    ET.SubElement(default, "joint", damping=_fmt([JOINT_DAMPING]), armature=_fmt([JOINT_ARMATURE]))
    ET.SubElement(default, "geom", type="mesh", density=_fmt([DENSITY]), material="pla", friction="0.8 0.02 0.001")
    ET.SubElement(
        default, "position",
        kp=_fmt([ACTUATOR_KP]), forcerange=_fmt([-ACTUATOR_FORCE_LIMIT, ACTUATOR_FORCE_LIMIT]), inheritrange="1",
    )

    visual = ET.SubElement(mujoco, "visual")
    ET.SubElement(visual, "global", azimuth="135", elevation="-20")
    ET.SubElement(visual, "headlight", ambient="0.4 0.4 0.4", diffuse="0.6 0.6 0.6")

    asset = ET.SubElement(mujoco, "asset")
    ET.SubElement(asset, "material", name="pla", rgba="0.85 0.85 0.82 1", specular="0.2")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", rgb1="0.2 0.25 0.3",
                  rgb2="0.3 0.35 0.4", width="512", height="512")
    ET.SubElement(asset, "material", name="floor", texture="grid", texrepeat="8 8", reflectance="0.1")
    for link in links.values():
        mesh_name = Path(link["mesh"]).stem
        ET.SubElement(asset, "mesh", name=mesh_name, file=link["mesh"], scale=_fmt(link["scale"]),
                      inertia="legacy" if mesh_name in LEGACY_INERTIA_MESHES else "exact")

    world = ET.SubElement(mujoco, "worldbody")
    palm = links["base_link"]
    floor_z = stl_vertices(MESH_DIR / palm["mesh"], palm["scale"])[:, 2].min() + palm["pos"][2] - 0.005
    ET.SubElement(world, "light", pos="0 -0.5 0.8", dir="0 0.5 -0.8")
    ET.SubElement(world, "geom", name="floor", type="plane", size="0.3 0.3 0.01", pos=f"0 0 {floor_z:.4f}",
                  material="floor", contype="0", conaffinity="0", density="0")

    children: dict[str, list[str]] = {}
    for jname, j in joints.items():
        children.setdefault(j["parent"], []).append(jname)

    def add_body(parent_el: ET.Element, link_name: str, joint_name: str | None) -> None:
        link = links[link_name]
        attrs = {"name": LINK_NAMES.get(link_name, link_name)}
        if joint_name:
            j = joints[joint_name]
            attrs["pos"] = _fmt(j["pos"])
            if np.any(j["rpy"]):
                attrs["quat"] = _fmt(rpy_to_quat(j["rpy"]))
        body = ET.SubElement(parent_el, "body", attrs)
        if joint_name:
            j, spec = joints[joint_name], specs[joint_name]
            axis, (lo, hi) = j["axis"], j["range"]
            if spec.flip:
                axis, (lo, hi) = -axis, (-hi, -lo)
            ET.SubElement(body, "joint", name=spec.name, axis=_fmt(axis), range=_fmt([lo, hi]))
        geom = {"name": attrs["name"], "mesh": Path(link["mesh"]).stem, "pos": _fmt(link["pos"])}
        if np.any(link["rpy"]):
            geom["quat"] = _fmt(rpy_to_quat(link["rpy"]))
        ET.SubElement(body, "geom", geom)
        if link_name in ("Indextip_1", "Thumbtip_1"):
            # Fingertip = mesh point farthest from the joint (visual meshes have no rotation here).
            verts = stl_vertices(MESH_DIR / link["mesh"], link["scale"]) + link["pos"]
            tip = verts[np.argmax(np.linalg.norm(verts, axis=1))]
            ET.SubElement(body, "site", name=f"{attrs['name'].split('_')[0]}_tip", pos=_fmt(tip),
                          size="0.003", rgba="1 0.3 0.1 1")
        for child_joint in children.get(link_name, []):
            add_body(body, joints[child_joint]["child"], child_joint)

    add_body(world, "base_link", None)

    # The palm is fixed to the world, so MuJoCo's automatic parent-child contact filter does not
    # apply to it. Its convex hull overlaps the finger base parts at the joints.
    contact = ET.SubElement(mujoco, "contact")
    for jname, j in joints.items():
        if j["parent"] == "base_link":
            ET.SubElement(contact, "exclude", body1=LINK_NAMES["base_link"], body2=LINK_NAMES[j["child"]])
    # MuJoCo collides meshes as convex hulls; the palm hull fills the space around the thumb base
    # and "hits" the metacarpal at any rotation. Remove once the palm is split into convex parts.
    for body1, body2 in HULL_ARTIFACT_EXCLUDES:
        ET.SubElement(contact, "exclude", body1=body1, body2=body2)

    actuator = ET.SubElement(mujoco, "actuator")
    for spec in JOINTS:
        ET.SubElement(actuator, "position", name=spec.name, joint=spec.name)

    return mujoco


def _relpath(target: Path, start: Path) -> str:
    import os
    return Path(os.path.relpath(target, start)).as_posix()


def main() -> None:
    links, joints = parse_urdf(URDF_PATH)
    mjcf = build_mjcf(links, joints)
    ET.indent(mjcf)
    header = (
        "<!-- GENERATED by sim/convert.py from hardware/robot_description/fusion_export. "
        "Do not edit by hand: change convert.py and re-run it. -->\n"
    )
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(header + ET.tostring(mjcf, encoding="unicode") + "\n", encoding="utf-8")
    print(f"Wrote {OUT_PATH.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
