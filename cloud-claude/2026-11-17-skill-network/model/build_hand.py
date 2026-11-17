"""Build ``model/models/v21_hand.xml`` (V2.1 hand + 2-DOF wrist + forearm, root fixed) from the extracted assets.

Inputs: ``model/assets/v21/kin.json`` (from ``extract_v21.py``) and ``model/actuators.json`` (servo gains, force
limits, joint damping/armature/friction; written by the actuator work). Output follows ``model/CONTRACT.md``:
names, actuator order, signs, attachable with ``MjSpec.attach`` + prefix (only ``compiler`` settings that the
attach carries per element: radian ranges, meshdir), default classes prefixed ``v21_``.

Plain, deterministic XML generation (ElementTree) instead of MjSpec: the file is reviewed in diffs and compared
byte for byte by ``--check``; the build then compiles it with MuJoCo to fail early.

    uv run python model/build_hand.py [--check]
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
KIN = ROOT / "model" / "assets" / "v21" / "kin.json"
ACT = ROOT / "model" / "actuators.json"
OUT = ROOT / "model" / "models" / "v21_hand.xml"

SERVO_JOINTS = (
    "index_pip", "index_mcp_flex", "index_mcp_abd", "thumb_ip", "thumb_mcp_flex", "thumb_cmc_flex", "thumb_cmc_rot",
    "middle_pip", "middle_mcp_flex", "middle_mcp_abd", "ring_pip", "ring_mcp_flex", "ring_mcp_abd", "little_pip",
    "little_mcp_flex", "little_mcp_abd",
)
WRIST_JOINTS = ("wrist_flex", "wrist_dev")
FINGERS = ("index", "middle", "ring", "little")

# Placeholder actuator values, used ONLY if model/actuators.json is missing (same keys, joint level).
PLACEHOLDER_SERVO = dict(kp=0.8, forcerange=[-0.3, 0.3], damping=0.05, armature=7e-4, frictionloss=0.008)
PLACEHOLDER_WRIST = dict(kp=20.0, forcerange=[-4.0, 4.0], damping=0.7, armature=2e-3, frictionloss=0.1)
# Passive coupled joints (DIP, bar): no servo. A tiny armature keeps the stiff joint equality well conditioned on
# the light distal phalanx / 0.4 g bar; small next to the servo armature reflected at the PIP.
PASSIVE = dict(damping=0.002, armature=1e-5, frictionloss=0.0)
EQ_SOLREF = "0.01 1"  # couplings: 10 ms time constant (>= 2 x the 2 ms robot timestep)

FRICTION = "0.8 0.01 0.001"  # PETG / PAHT-CF on typical objects
PAD_FRICTION = "1.2 0.02 0.002"  # silicone skin
MESH_DIR = "../assets/v21"


def load_actuators(path: Path = ACT) -> tuple[dict[str, dict], str]:
    """Per joint {kp, forcerange, damping, armature, frictionloss} from actuators.json (else marked placeholders).

    Hand servos use ``forcerange_cap80`` when present (the firmware caps current at 80 % of stall; the fuse cups and
    rollers are marginal at full stall), the wrist uses ``forcerange``. Unknown extra fields are ignored.
    Returns (params, source description).
    """
    keys = ("kp", "forcerange", "damping", "armature", "frictionloss")
    if path.exists():
        d = json.loads(path.read_text())
        out = {j: {k: d["servos"][j][k] for k in keys} for j in SERVO_JOINTS}
        for j in SERVO_JOINTS:  # lead decision: hand servos capped at 80 % stall in firmware (constant current)
            out[j]["forcerange"] = d["servos"][j].get("forcerange_cap80", d["servos"][j]["forcerange"])
        out.update({j: {k: d["wrist"][j][k] for k in keys} for j in WRIST_JOINTS})
        return out, f"model/actuators.json ({d.get('servo_model', 'servo model not stated')})"
    out = {j: dict(PLACEHOLDER_SERVO) for j in SERVO_JOINTS}
    out.update({j: dict(PLACEHOLDER_WRIST) for j in WRIST_JOINTS})
    return out, "PLACEHOLDER values (model/actuators.json missing)"


def f(x: float) -> str:
    """Compact, stable float formatting."""
    s = f"{float(x):.9g}"
    return "0" if s in ("-0", "0") else s


def vec(v) -> str:
    return " ".join(f(x) for x in v)


def adjacent_excludes(kin: dict) -> list[tuple[str, str]]:
    """Body pairs that must not collide: parent-child and grandparent-grandchild (through a small link).

    Parent-child is filtered by MuJoCo's default ``filterparent`` already; listed explicitly so the model keeps it
    even inside a scene that disables that flag. Grandparent pairs (palm-proximal over the yoke, palm-metacarpal
    over the thumb base, proximal-distal over the middle phalanx, forearm-palm over the wrist ring, ...) sit
    within millimetres of each other at the joints, and their hulls overlap there by construction.
    """
    parent = {b: d["parent"] for b, d in kin["bodies"].items()}
    pairs = []
    for b, p in parent.items():
        if p is None:
            continue
        pairs.append((p, b))
        if parent.get(p):
            pairs.append((parent[p], b))
    return pairs


# Non-adjacent pairs whose convex hulls touch in designed CAD poses although the CAD parts do not (found by
# tests/test_hand.py::test_no_self_contact_in_cad_poses and listed here with the reason).
EXTRA_EXCLUDES: list[tuple[str, str, str]] = []


def build(kin: dict, act: dict[str, dict], act_src: str) -> str:
    """The MJCF text."""
    bodies = kin["bodies"]
    root = ET.Element("mujoco", model="v21_hand")
    root.append(ET.Comment(
        f" Generated by model/build_hand.py from model/assets/v21/kin.json (CAD git {kin['source']['git_rev'][:12]},"
        f" poses sha256 {kin['source']['poses_sha256'][:12]}) and {act_src}. Do not edit. "))
    ET.SubElement(root, "compiler", angle="radian", meshdir=MESH_DIR, autolimits="true")

    dflt = ET.SubElement(ET.SubElement(root, "default"), "default", {"class": "v21"})
    ET.SubElement(dflt, "joint", type="hinge", limited="true")
    ET.SubElement(dflt, "geom", contype="1", conaffinity="1", condim="3", friction=FRICTION, group="3")
    ET.SubElement(dflt, "site", group="4", size="0.003", rgba="1 0.3 0.1 1")
    vis = ET.SubElement(dflt, "default", {"class": "v21_visual"})
    ET.SubElement(vis, "geom", type="mesh", contype="0", conaffinity="0", group="2", density="0",
                  rgba="0.85 0.87 0.89 1")
    col = ET.SubElement(dflt, "default", {"class": "v21_col"})
    ET.SubElement(col, "geom", type="mesh", group="3", density="0", rgba="0.4 0.6 0.9 0.4")
    pad = ET.SubElement(dflt, "default", {"class": "v21_pad"})
    ET.SubElement(pad, "geom", type="mesh", group="3", density="0", condim="4", friction=PAD_FRICTION,
                  solref="0.01 1", solimp="0.9 0.95 0.002", rgba="0.9 0.5 0.4 0.5")

    asset = ET.SubElement(root, "asset")
    for b, d in bodies.items():
        ET.SubElement(asset, "mesh", name=f"v21_{b}_vis", file=d["visual_mesh"], inertia="shell")
        for k, c in enumerate(d["collision_meshes"]):
            ET.SubElement(asset, "mesh", name=f"v21_{b}_c{k}", file=c)

    world = ET.SubElement(root, "worldbody")
    elems: dict[str, ET.Element] = {}
    sites_by_body: dict[str, list] = {}
    for n, s in kin["sites"].items():
        sites_by_body.setdefault(s["body"], []).append((n, s))
    for b, d in bodies.items():
        parent_el = world if d["parent"] is None else elems[d["parent"]]
        ppiv = [0.0, 0.0, 0.0] if d["parent"] is None else bodies[d["parent"]]["pivot_m"]
        attrs = {"name": b, "pos": vec([a - p for a, p in zip(d["pivot_m"], ppiv)])}
        if d["parent"] is None:
            attrs["childclass"] = "v21"
        el = ET.SubElement(parent_el, "body", attrs)
        elems[b] = el
        I = d["inertia"]
        ET.SubElement(el, "inertial", pos=vec(d["com_m"]), mass=f(d["mass_kg"]), fullinertia=vec(I))
        if d["joint"]:
            j = d["joint"]
            p = act.get(j, PASSIVE)
            ET.SubElement(el, "joint", name=j, axis=vec(d["axis"]), range=vec(d["range_rad"]),
                          damping=f(p["damping"]), armature=f(p["armature"]), frictionloss=f(p["frictionloss"]))
        ET.SubElement(el, "geom", {"class": "v21_visual", "name": f"{b}_vis", "mesh": f"v21_{b}_vis"})
        for k, cls in enumerate(d["collision_class"]):
            g = {"class": "v21_pad" if cls == "pad" else "v21_col", "name": f"{b}_c{k}", "mesh": f"v21_{b}_c{k}"}
            ET.SubElement(el, "geom", g)
        for n, s in sites_by_body.get(b, []):
            ET.SubElement(el, "site", name=n, pos=vec(s["pos_m"]), quat=vec(s["quat"]))

    contact = ET.SubElement(root, "contact")
    for b1, b2 in adjacent_excludes(kin):
        ET.SubElement(contact, "exclude", body1=b1, body2=b2)
    for b1, b2, _why in EXTRA_EXCLUDES:
        ET.SubElement(contact, "exclude", body1=b1, body2=b2)

    eq = ET.SubElement(root, "equality")
    for j, c in kin["couplings"].items():
        ET.SubElement(eq, "joint", name=f"{j}_coupling", joint1=j, joint2=c["leader"], polycoef=vec(c["poly"]),
                      solref=EQ_SOLREF)

    acts = ET.SubElement(root, "actuator")
    joint_range = {d["joint"]: d["range_rad"] for d in bodies.values() if d["joint"]}
    for j in (*SERVO_JOINTS, *WRIST_JOINTS):
        p = act[j]
        ET.SubElement(acts, "position", {"name": j, "joint": j, "class": "v21", "kp": f(p["kp"]),
                                         "ctrlrange": vec(joint_range[j]), "forcerange": vec(p["forcerange"])})
    ET.indent(root, space="  ")
    return '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="exit 1 if models/v21_hand.xml is not up to date")
    args = ap.parse_args()
    kin = json.loads(KIN.read_text())
    act, src = load_actuators()
    xml = build(kin, act, src)
    if args.check:
        if not OUT.exists() or OUT.read_text() != xml:
            print("model/models/v21_hand.xml is stale: run uv run python model/build_hand.py")
            return 1
        print("model/models/v21_hand.xml is up to date")
        return 0
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(xml)
    import mujoco

    m = mujoco.MjModel.from_xml_path(str(OUT))
    print(f"wrote {OUT.relative_to(ROOT)}: {m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, {m.ngeom} geoms, "
          f"mass {sum(m.body_mass):.4f} kg ({src})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
