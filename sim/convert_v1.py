"""Convert the Tendra Hand V1 Fusion export into a tendon-driven MuJoCo model (MJCF).

Input (never edited by hand; a fresh export from Fusion only needs a re-run of this script):

    hardware/robot_description/v1_export/hand_v1.json     parts, meshes and joints (mm, world frame, q = 0)
    hardware/robot_description/v1_export/meshes/*.stl
    hardware/robot_description/v1_export/tendon_routes.json  (optional) strand paths in the palm/forearm

Output: sim/models/tendra_hand_v1.xml

    uv run python sim/convert_v1.py

What the converter does:
- One body per part. Parts that are not the child of any joint (palm, later forearm, servo
  mounts...) are fixed to the world. Body frames are world-aligned and sit on the joint axis.
- Closing-positive sign convention (CLAUDE.md): Fusion's axis signs are arbitrary, so each axis
  is flipped until a small positive rotation moves the child's parts in the "closing" direction
  given by CLOSING_DIRECTION below.
- Mass and inertia from the meshes at PLA density (meshes with open edges fall back to MuJoCo's
  "legacy" inertia, which tolerates them).
- Tendons: two strands per servo-driven joint (<joint>_flex, <joint>_ext) that model the real
  routing rule, see build_strand(). Each of the 16 servos is a position actuator on its flex strand.
- DIP coupling (2026-10-01): the four finger DIPs have no servo. Their two strands are tied in the
  proximal phalanx, wrap a hub on the PIP axis and the DIP drum, crossed in the middle phalanx, so the
  DIP turns COUPLING_RATIO x the PIP (see build_coupling_strand()). The physics uses a joint equality
  (dip = ratio * pip, the same force law as an inextensible coupling loop); the passive strands give
  the real geometry, and the tests check that their lengths stay constant along the coupling.
"""

from __future__ import annotations

import json
import math
import os
import re
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EXPORT_DIR = ROOT / "hardware" / "robot_description" / "v1_export"
SPEC_PATH = EXPORT_DIR / "hand_v1.json"
ROUTES_PATH = EXPORT_DIR / "tendon_routes.json"
OUT_PATH = ROOT / "sim" / "models" / "tendra_hand_v1.xml"

MM = 0.001
DEG = math.pi / 180.0

# Solid PLA is ~1240 kg/m^3. Printed parts are lighter (infill); update once parts are weighed.
DENSITY = 1240.0

# Actuator (= servo bus ID) order. Must match firmware/include/config_v1.h and software/tendra.
# 16 servos since 2026-10-01: the finger DIPs are coupled to their PIPs.
ACTUATOR_ORDER = [
    "index_pip",
    "index_mcp_flex",
    "index_mcp_abd",  # 1-3
    "thumb_ip",
    "thumb_mcp_flex",
    "thumb_cmc_flex",
    "thumb_cmc_rot",  # 4-7
    "middle_pip",
    "middle_mcp_flex",
    "middle_mcp_abd",  # 8-10
    "ring_pip",
    "ring_mcp_flex",
    "ring_mcp_abd",  # 11-13
    "little_pip",
    "little_mcp_flex",
    "little_mcp_abd",  # 14-16
]
FINGERS = ("index", "middle", "ring", "little")
COUPLED = {f"{f}_dip": f"{f}_pip" for f in FINGERS}  # passive joint -> the joint that drives it
JOINT_ORDER = ACTUATOR_ORDER + list(COUPLED)  # every joint of the export (20)

# Joint limits in degrees (positive = closing). Same as config_v1.h.
LIMITS_DEG = {
    "dip": (-5, 95),
    "pip": (-5, 95),
    "mcp_flex": (-5, 95),
    "mcp_abd": (-15, 15),
    "little_mcp_abd": (-20, 20),
    "thumb_ip": (-5, 95),
    "thumb_mcp_flex": (-5, 95),
    "thumb_cmc_flex": (-13, 45),  # V1: the metacarpal block touches the base at 46 deg
    "thumb_cmc_rot": (-100, 40),
}


# Closing direction per joint, in the world frame at q = 0 (fingers point +Z, palm face at -Y,
# thumb sticks out toward -Y, the other fingers are at -X of the index):
# - finger flexion (mcp_flex, pip, dip): the finger bends toward the palm side, -Y
# - finger mcp_abd: toward the thumb side, +X
# - thumb_cmc_rot: swings the thumb across the palm (opposition), -X
# - thumb flexion (cmc_flex, mcp_flex, ip): the thumb curls toward the fingers, +Z
def closing_direction(joint: str) -> np.ndarray:
    finger, motion = joint.split("_", 1)
    if finger == "thumb":
        if motion == "cmc_rot":
            return np.array([-1.0, 0.0, 0.0])
        return np.array([0.0, 0.0, 1.0])
    if motion == "mcp_abd":
        return np.array([1.0, 0.0, 0.0])
    return np.array([0.0, -1.0, 0.0])


def joint_limits_deg(joint: str) -> tuple[float, float]:
    if joint in LIMITS_DEG:
        return LIMITS_DEG[joint]
    return LIMITS_DEG[joint.split("_", 1)[1]]


# ----- Tendons and actuation -----------------------------------------------------------------
# Drum, spool and coupling sizes come from tendon_routes.json (hardware/cad/tendon_router.py, the
# single source of truth); the fallbacks are the same numbers, for a model without routes.
_ROUTES_JSON = json.loads(ROUTES_PATH.read_text(encoding="utf-8")) if ROUTES_PATH.exists() else {}
SPOOL_RADIUS = _ROUTES_JSON.get("spool_radius_mm", 5.0) * MM  # every servo spool
_DRUMS_MM = _ROUTES_JSON.get("drums_mm") or {}
_COUPLING = _ROUTES_JSON.get("coupling") or {}
COUPLING_HUB_RADIUS = _COUPLING.get("hub_r_mm", 4.5) * MM  # hub on the PIP axis (proximal)
COUPLING_RATIO = _COUPLING.get("ratio", 0.75)  # DIP angle per PIP angle
COUPLING_SOLREF = (0.01, 1.0)  # stiff, critically damped (>= 2 x the 4 ms RL timestep)


def drum_radius(joint: str) -> float:
    """Tendon centreline radius on a joint's drum (m): finger mcp_flex 7 mm, thumb_cmc_rot 7.5 mm
    (the hollow journal takes the base's middle), every other drum 6 mm."""
    if joint in _DRUMS_MM:
        return _DRUMS_MM[joint] * MM
    if joint == "thumb_cmc_rot":
        return 7.5 * MM
    if joint.endswith("_mcp_flex") and not joint.startswith("thumb"):
        return 7.0 * MM
    return 6.0 * MM


def servo_per_joint(joint: str) -> float:
    """Servo radians per joint radian (the firmware's servo_per_joint)."""
    return drum_radius(joint) / SPOOL_RADIUS


DRUM_HALF_WIDTH = 1.5 * MM
ANCHOR_OFFSET = (
    0.5 * MM
)  # anchor site just outside the drum (MuJoCo needs sites outside wrap geoms)
WRAP_MARGIN = 20 * DEG  # smallest wrap left on the drum at the joint limits
GUIDE_DISTANCE = (
    8.0 * MM
)  # the strand runs tangent to the drum for this far before it (parent side)
START_DISTANCE = 10.0 * MM  # without tendon_routes.json, strands start this far into the palm
HUB_CROSS = 8.0 * MM  # coupling strand: crossing point along the middle phalanx (<= 40 % of it)

# SCS0009: 0.19 N*m stall at 5 V. Use half of it as the continuous limit (19 N tendon force at the
# 5 mm spool). The servo's P controller saturates after roughly 0.1 rad of error, so kp ~ 2 N*m/rad;
# kv = force limit / no-load speed (~8 rad/s). All in servo units (N*m, rad); the actuators scale
# them to the joint by n = servo_per_joint.
SERVO_KP = 2.0  # N*m per rad of servo error
SERVO_KV = 0.012  # N*m*s/rad
SERVO_FORCE_LIMIT = 0.095  # N*m at the servo = 19 N tendon force at r = 5 mm
JOINT_DAMPING = 0.005  # N*m*s/rad (joint friction)
JOINT_ARMATURE = 1e-4  # kg*m^2, reflected servo/gearbox inertia, also keeps small links stable

# Body pairs that only collide because MuJoCo collides meshes as convex hulls. The palm has a pocket
# for the thumb base that its hull fills, so the hull "hits" the metacarpal at almost any thumb pose
# although the real meshes don't touch (checked with an exact mesh test over the CMC range; the real
# parts only meet near cmc_flex = 80 deg, where the thumb folds onto the palm). Proper fix: convex
# decomposition of the palm. Pairs whose parts don't exist are skipped.
HULL_ARTIFACT_EXCLUDES = [("palm", "thumb_metacarpal")]

# Named poses (degrees) for keyframes and tests, servo-driven joints only (each DIP follows its
# PIP). Joints not listed are 0.
FIST_DEG = {
    "mcp_flex": 80,
    "pip": 90,
    "mcp_abd": 0,
    "thumb_cmc_rot": 0,
    "thumb_cmc_flex": 30,
    "thumb_mcp_flex": 20,
    "thumb_ip": 30,
}


def pose_deg(pose: dict[str, float], joint: str) -> float:
    if joint in pose:
        return pose[joint]
    return pose.get(joint.split("_", 1)[1], 0.0)


# ----- Helpers -------------------------------------------------------------------------------


def _fmt(values, digits: int = 6) -> str:
    return " ".join(f"{v:.{digits}g}" for v in np.asarray(values, dtype=float).ravel())


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def _relpath(target: Path, start: Path) -> str:
    return Path(os.path.relpath(target, start)).as_posix()


def read_stl(path: Path) -> np.ndarray:
    """Triangles (N x 3 x 3) of a binary STL, in the file's units."""
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    dtype = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(data[84 : 84 + n * 50], dtype=dtype)["v"].astype(float)


def open_edge_count(tris: np.ndarray) -> int:
    """Edges not shared by exactly two triangles (0 for a watertight mesh)."""
    _, inv = np.unique(tris.reshape(-1, 3), axis=0, return_inverse=True)
    f = inv.reshape(-1, 3)
    edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return int((counts != 2).sum())


# ----- Geometry ------------------------------------------------------------------------------


@dataclass
class Joint:
    name: str
    parent: str
    child: str
    center: np.ndarray  # point on the axis (m), where the drum and body origin sit
    axis: np.ndarray  # unit axis, closing-positive
    u: np.ndarray  # unit, from the axis toward the child (distal), perpendicular to the axis
    w: np.ndarray  # axis x u: where a distal point moves when the joint closes
    lo: float  # limits (rad)
    hi: float
    flipped: bool  # True if Fusion's axis was inverted


@dataclass
class Hand:
    parts: dict[str, dict]  # name -> {"mesh": relative path, "tris": (N,3,3) m}
    joints: dict[str, Joint]  # in JOINT_ORDER (servo-driven, then the coupled DIPs)
    parent_joint: dict[str, str]  # child part -> joint
    fixed: list[str]  # parts fixed to the world
    body_pos: dict[str, np.ndarray]  # world position of each body frame (m)


# Servo and spool parts of joints that no longer have a servo. Until the Fusion design is
# re-exported with 16 servos, the export still holds the 20-servo forearm: its DIP servos are left
# out (the others are welded to the world, so their old names and places are only cosmetic).
_STALE_PART = re.compile(r"^(servo|spool)_\d\d_(\w+)$")


def load_hand(spec_path: Path = SPEC_PATH) -> Hand:
    spec = json.loads(spec_path.read_text(encoding="utf-8"))
    base = spec_path.parent
    parts = {}
    for p in spec["parts"]:
        m = _STALE_PART.match(p["name"])
        if m and m.group(2) in COUPLED:
            continue
        parts[p["name"]] = {"mesh": p["mesh"], "tris": read_stl(base / p["mesh"]) * MM}

    raw = {j["name"]: j for j in spec["joints"]}
    if set(raw) != set(JOINT_ORDER):
        raise ValueError(
            f"Joints in the export and JOINT_ORDER disagree: {sorted(set(raw) ^ set(JOINT_ORDER))}"
        )
    for j in raw.values():
        if abs(j.get("angle_rad", 0.0)) > 1e-9:
            raise ValueError(f"{j['name']}: export must be at q = 0 (angle_rad = {j['angle_rad']})")
        for part in (j["parent"], j["child"]):
            if part not in parts:
                raise ValueError(f"{j['name']}: unknown part {part!r}")
    parent_joint = {j["child"]: j["name"] for j in raw.values()}
    if len(parent_joint) != len(raw):
        raise ValueError("A part is the child of more than one joint")

    children: dict[str, list[str]] = {}
    for j in raw.values():
        children.setdefault(j["parent"], []).append(j["child"])

    def subtree(part: str) -> list[str]:
        out = [part]
        for c in children.get(part, []):
            out += subtree(c)
        return out

    joints = {}
    for name in JOINT_ORDER:
        j = raw[name]
        point = np.array(j["point_mm"], dtype=float) * MM
        a0 = _unit(np.array(j["axis"], dtype=float))
        verts = np.concatenate([parts[p]["tris"].reshape(-1, 3) for p in subtree(j["child"])])
        centroid = verts.mean(axis=0)
        center = point + a0 * np.dot(centroid - point, a0)
        radial = centroid - center
        motion = _unit(
            np.cross(a0, radial)
        )  # where the child moves for a positive rotation about a0
        closing = closing_direction(name)
        alignment = float(np.dot(motion, closing))
        if abs(alignment) < 0.25:
            raise ValueError(
                f"{name}: cannot tell the closing direction (motion {motion}, rule {closing})"
            )
        axis = a0 if alignment > 0 else -a0
        u = _unit(radial)
        lo, hi = (v * DEG for v in joint_limits_deg(name))
        joints[name] = Joint(
            name,
            j["parent"],
            j["child"],
            center,
            axis,
            u,
            np.cross(axis, u),
            lo,
            hi,
            flipped=alignment < 0,
        )

    fixed = [p for p in parts if p not in parent_joint]
    if not fixed:
        raise ValueError("No fixed part (palm): every part is the child of a joint")
    body_pos = {p: np.zeros(3) for p in fixed}
    body_pos.update({j.child: j.center for j in joints.values()})
    return Hand(parts, joints, parent_joint, fixed, body_pos)


def ancestors(hand: Hand, joint: str) -> list[str]:
    """Joints from the root (at a fixed part) down to and including `joint`."""
    chain = [joint]
    while hand.joints[chain[-1]].parent in hand.parent_joint:
        chain.append(hand.parent_joint[hand.joints[chain[-1]].parent])
    return chain[::-1]


def finger_of(joint: str) -> str:
    return joint.split("_", 1)[0]


def load_routes(path: Path = ROUTES_PATH) -> dict[str, dict] | None:
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    out = {s["name"]: s for s in data["strands"]}
    sim = (data.get("thumb") or {}).get("sim")
    if sim:  # the thumb's real paths (tendon_router.thumb_sim()), keyed by strand / joint
        out["_thumb_paths"] = sim["strands"]
        out["_thumb_drums"] = sim["drums"]
    return out


def thumb_drum(routes: dict | None, joint: str):
    """(centre (m), half width (m)) of a thumb drum from tendon_routes.json, or None."""
    d = (routes or {}).get("_thumb_drums", {}).get(joint)
    if not d:
        return None
    return np.array(d["center_mm"]) * MM, d["half_width_mm"] * MM


@dataclass
class PathPoint:
    body: str | None  # None = worldbody (fixed parts)
    pos: np.ndarray  # world position at q = 0 (m)
    kind: str  # route / start / cross / guide / anchor
    drum: str | None = None  # set on the anchor: wrap this drum just before it
    side_body: str | None = None  # sidesite body/position for the drum
    side_pos: np.ndarray | None = None


def build_strand(hand: Hand, joint: str, side: str, routes: dict | None) -> list[PathPoint]:
    """Path of one strand, proximal (spool/palm) to distal (anchor on the drum).

    The routing rule of the real hand:
    - The joint's own loop wraps a drum of radius r = drum_radius() on the CHILD segment, centred on the
      joint axis, and is anchored (tied) on it. The flex strand arrives tangent to the closing side
      of the drum (+w), the ext strand tangent to the opening side (-w), both running along the
      parent segment. Rotating the child by dq wraps/unwraps r * dq of strand:
      dL_flex/dq = -r and dL_ext/dq = +r, and L_flex + L_ext stays constant (tight loop).
    - Strands of more distal joints cross every joint they pass exactly on that joint's axis. A
      point on the axis does not move when the joint turns, so these joints don't change the
      strand length (no coupling). The crossing point is the point on the axis nearest to the
      next (more distal) path point.
    - In the palm, the strand follows tendon_routes.json (entry hole at the finger base down to the
      spool tangent point) when it exists; otherwise it starts START_DISTANCE into the palm.
    """
    j = hand.joints[joint]
    r = drum_radius(joint)
    anchor_r = r + ANCHOR_OFFSET
    sgn = 1.0 if side == "flex" else -1.0
    drum = thumb_drum(routes, joint)
    center = drum[0] if drum else j.center
    # Anchor angle on the drum, measured from the tangent point toward u (distal). It shrinks by q
    # for the flex strand and grows by q for the ext strand; keep WRAP_MARGIN of wrap at the limits,
    # plus the extra angle lost because the anchor site sits just outside the drum.
    lost = math.acos(r / anchor_r)
    phi = (j.hi if side == "flex" else -j.lo) + WRAP_MARGIN + lost
    real = (routes or {}).get("_thumb_paths", {}).get(f"{joint}_{side}")
    if real:  # the thumb's real path: drum, then fixed points on the parts, then the palm route
        if real["anchor_dir"] is None:  # the usual rule, around the real drum centre
            anchor = center + anchor_r * (sgn * math.cos(phi) * j.w + math.sin(phi) * j.u)
            side_dir = sgn * math.cos(math.pi / 4) * j.w + math.sin(math.pi / 4) * j.u
            side_pos = center + 2 * r * side_dir
        else:
            anchor = center + anchor_r * np.array(real["anchor_dir"])
            side_pos = np.array(real["side_mm"]) * MM
        points = [
            PathPoint(
                j.child,
                anchor,
                "anchor",
                drum=f"{joint}_drum",
                side_body=j.parent,
                side_pos=side_pos,
            )
        ]
        points += [PathPoint(b, np.array(p) * MM, "real") for b, p in real["points"]]
        route = routes.get(f"{joint}_{side}")
        points += [
            PathPoint(None, np.array(p, dtype=float) * MM, "route") for p in route["points_mm"]
        ]
        return points[::-1]
    anchor = center + anchor_r * (sgn * math.cos(phi) * j.w + math.sin(phi) * j.u)
    chain = ancestors(hand, joint)
    parent_body = j.parent
    if len(chain) > 1:
        prev_center = hand.joints[chain[-2]].center
        guide = min(GUIDE_DISTANCE, 0.6 * abs(np.dot(j.center - prev_center, j.u)))
    else:
        guide = GUIDE_DISTANCE
    side_dir = sgn * math.cos(math.pi / 4) * j.w + math.sin(math.pi / 4) * j.u
    points = [
        PathPoint(
            j.child,
            anchor,
            "anchor",
            drum=f"{joint}_drum",
            side_body=parent_body,
            side_pos=center + 2 * r * side_dir,
        ),
        PathPoint(parent_body, center - guide * j.u + sgn * r * j.w, "guide"),
    ]
    for k in reversed(chain[:-1]):
        kj = hand.joints[k]
        prev = points[-1].pos
        points.append(
            PathPoint(kj.child, kj.center + kj.axis * np.dot(prev - kj.center, kj.axis), "cross")
        )
    root = hand.joints[chain[0]]
    route = routes.get(f"{joint}_{side}") if routes else None
    if route:
        points += [
            PathPoint(None, np.array(p, dtype=float) * MM, "route") for p in route["points_mm"]
        ]
    else:
        points.append(PathPoint(None, points[-1].pos - START_DISTANCE * root.u, "start"))
    return points[::-1]


def build_coupling_strand(hand: Hand, joint: str, side: str) -> list[PathPoint]:
    """Path of one strand of a DIP's coupling loop, proximal (tie in the proximal phalanx) to distal
    (anchor on the DIP drum). `joint` is the DIP, p = its PIP.

    - DIP end: as in build_strand(): the flex strand is tangent to the DIP drum's closing side (+w),
      the ext strand to its opening side, so dL_flex/dq_dip = -r_dip.
    - PIP end: the strand wraps a hub of radius r_hub that is fixed to the PROXIMAL phalanx, on the
      PIP axis, on the opposite side: flex on the hub's back (-w), ext on its palm side. A strand on
      the outside of a bend gets longer as the joint closes: dL_flex/dq_pip = +r_hub.
    - The loop's length is fixed, so r_hub * dq_pip = r_dip * dq_dip: DIP = (r_hub / r_dip) x PIP.
      The two strands cross inside the middle phalanx (crossing site -> DIP guide).
    """
    d, p = hand.joints[joint], hand.joints[COUPLED[joint]]
    sgn = 1.0 if side == "flex" else -1.0
    r_dip, r_hub = drum_radius(joint), COUPLING_HUB_RADIUS
    # DIP drum: anchor and guide as in build_strand().
    anchor_r = r_dip + ANCHOR_OFFSET
    phi = (d.hi if side == "flex" else -d.lo) + WRAP_MARGIN + math.acos(r_dip / anchor_r)
    anchor = d.center + anchor_r * (sgn * math.cos(phi) * d.w + math.sin(phi) * d.u)
    middle_len = abs(np.dot(d.center - p.center, p.u))
    guide = min(GUIDE_DISTANCE, 0.6 * middle_len)
    side_dir = sgn * math.cos(math.pi / 4) * d.w + math.sin(math.pi / 4) * d.u
    # Hub: the strand leaves the hub tangent at -sgn * w, runs along the middle phalanx to a crossing
    # point, and wraps back toward the palm to its tie. The wrap at q = 0 leaves WRAP_MARGIN where it
    # is smallest (flex: PIP fully open, ext: PIP fully closed). The side site sits straight out
    # from the hub on the strand's side (-sgn * w): a parameter sweep (crossing distance, side site
    # angle and distance, tie offset) found this keeps MuJoCo's wrap on the right side over the
    # whole PIP range; other side-site angles flip it at large bends.
    tie_r = r_hub + ANCHOR_OFFSET
    psi = (-p.lo if side == "flex" else p.hi) + WRAP_MARGIN + math.acos(r_hub / tie_r)
    tie = p.center + tie_r * (-sgn * math.cos(psi) * p.w - math.sin(psi) * p.u)
    cross = p.center + min(HUB_CROSS, 0.4 * middle_len) * p.u - sgn * r_hub * p.w
    return [
        PathPoint(p.parent, tie, "tie"),
        PathPoint(
            p.child,
            cross,
            "cross",
            drum=f"{joint}_hub",
            side_body=p.parent,
            side_pos=p.center - sgn * 2 * r_hub * p.w,
        ),
        PathPoint(d.parent, d.center - guide * d.u + sgn * r_dip * d.w, "guide"),
        PathPoint(
            d.child,
            anchor,
            "anchor",
            drum=f"{joint}_drum",
            side_body=d.parent,
            side_pos=d.center + 2 * r_dip * side_dir,
        ),
    ]


# ----- MJCF ----------------------------------------------------------------------------------


def build_mjcf(
    hand: Hand,
    routes: dict | None,
    lengths0: dict[str, float] | None = None,
    meshdir: str | None = None,
    qpos_order: list[str] | None = None,
) -> ET.Element:
    """MJCF tree. `lengths0` are the flex strand lengths at q = 0 (from a first compile); they set the
    actuator offset so that ctrl = 0 means a straight joint. `qpos_order` is MuJoCo's joint order
    (body tree order, from that compile); the keyframes are only written when it is given."""
    mujoco = ET.Element("mujoco", model="tendra_hand_v1")
    ET.SubElement(
        mujoco,
        "compiler",
        angle="radian",
        autolimits="true",
        meshdir=meshdir or _relpath(EXPORT_DIR, OUT_PATH.parent),
    )
    ET.SubElement(mujoco, "option", timestep="0.002", integrator="implicitfast")

    default = ET.SubElement(mujoco, "default")
    ET.SubElement(default, "joint", damping=_fmt([JOINT_DAMPING]), armature=_fmt([JOINT_ARMATURE]))
    ET.SubElement(
        default,
        "geom",
        type="mesh",
        density=_fmt([DENSITY]),
        material="pla",
        friction="0.8 0.02 0.001",
    )
    ET.SubElement(default, "site", size="0.0008", group="4", rgba="0.9 0.9 0.2 1")
    drum = ET.SubElement(default, "default", {"class": "drum"})
    ET.SubElement(
        drum,
        "geom",
        type="cylinder",
        contype="0",
        conaffinity="0",
        group="3",
        density="0",
        material="drum",
    )
    ET.SubElement(drum, "site", group="5")

    visual = ET.SubElement(mujoco, "visual")
    ET.SubElement(visual, "global", azimuth="135", elevation="-20")
    ET.SubElement(visual, "headlight", ambient="0.4 0.4 0.4", diffuse="0.6 0.6 0.6")
    ET.SubElement(visual, "scale", framelength="0.02", framewidth="0.001")

    asset = ET.SubElement(mujoco, "asset")
    ET.SubElement(asset, "material", name="pla", rgba="0.85 0.85 0.82 1", specular="0.2")
    ET.SubElement(asset, "material", name="drum", rgba="0.2 0.5 0.9 0.4")
    ET.SubElement(
        asset,
        "texture",
        name="grid",
        type="2d",
        builtin="checker",
        rgb1="0.2 0.25 0.3",
        rgb2="0.3 0.35 0.4",
        width="512",
        height="512",
    )
    ET.SubElement(
        asset, "material", name="floor", texture="grid", texrepeat="8 8", reflectance="0.1"
    )
    for name, part in sorted(hand.parts.items()):
        inertia = "exact" if open_edge_count(part["tris"]) == 0 else "legacy"
        ET.SubElement(
            asset, "mesh", name=name, file=part["mesh"], scale=_fmt([MM] * 3), inertia=inertia
        )

    world = ET.SubElement(mujoco, "worldbody")
    floor_z = min(hand.parts[p]["tris"][..., 2].min() for p in hand.fixed) - 0.005
    ET.SubElement(world, "light", pos="0 -0.5 0.8", dir="0 0.5 -0.8")
    ET.SubElement(
        world,
        "geom",
        name="floor",
        type="plane",
        size="0.4 0.4 0.01",
        pos=f"0 0 {floor_z:.4f}",
        material="floor",
        contype="0",
        conaffinity="0",
        density="0",
    )

    body_el: dict[str | None, ET.Element] = {None: world}

    def add_body(parent_el: ET.Element, part: str, parent_pos: np.ndarray) -> None:
        pos = hand.body_pos[part]
        attrs = {"name": part}
        if np.any(pos - parent_pos):
            attrs["pos"] = _fmt(pos - parent_pos)
        body = ET.SubElement(parent_el, "body", attrs)
        body_el[part] = body
        if part in hand.parent_joint:
            j = hand.joints[hand.parent_joint[part]]
            ET.SubElement(body, "joint", name=j.name, axis=_fmt(j.axis), range=_fmt([j.lo, j.hi]))
            drum = thumb_drum(routes, j.name)
            c, h = (drum[0] - pos, drum[1]) if drum else (np.zeros(3), DRUM_HALF_WIDTH)
            ET.SubElement(
                body,
                "geom",
                {
                    "name": f"{j.name}_drum",
                    "class": "drum",
                    "fromto": _fmt(np.concatenate([c - h * j.axis, c + h * j.axis])),
                    "size": _fmt([drum_radius(j.name)]),
                },
            )
        # Coupling hubs: fixed to this part, on the axis of the PIP whose parent it is.
        for dip, pip in COUPLED.items():
            pj = hand.joints[pip]
            if pj.parent == part:
                c = pj.center - pos
                ET.SubElement(
                    body,
                    "geom",
                    {
                        "name": f"{dip}_hub",
                        "class": "drum",
                        "fromto": _fmt(
                            np.concatenate(
                                [c - DRUM_HALF_WIDTH * pj.axis, c + DRUM_HALF_WIDTH * pj.axis]
                            )
                        ),
                        "size": _fmt([COUPLING_HUB_RADIUS]),
                    },
                )
        geom = {"name": part, "mesh": part}
        if np.any(pos):
            geom["pos"] = _fmt(-pos)
        ET.SubElement(body, "geom", geom)
        for child_part in sorted(
            c for c, jn in hand.parent_joint.items() if hand.joints[jn].parent == part
        ):
            add_body(body, child_part, pos)

    for part in hand.fixed:
        add_body(world, part, np.zeros(3))

    # Fingertip sites: on each distal part, the mesh point farthest along the finger.
    for j in hand.joints.values():
        if any(o.parent == j.child for o in hand.joints.values()):
            continue
        verts = hand.parts[j.child]["tris"].reshape(-1, 3)
        tip = verts[np.argmax((verts - j.center) @ j.u)]
        ET.SubElement(
            body_el[j.child],
            "site",
            name=f"{finger_of(j.name)}_tip",
            pos=_fmt(tip - hand.body_pos[j.child]),
            size="0.003",
            group="0",
            rgba="1 0.3 0.1 1",
        )

    def local(body: str | None, p: np.ndarray) -> np.ndarray:
        return p - (hand.body_pos[body] if body else 0.0)

    # Servo strands in actuator order, then the passive DIP coupling strands.
    tendon = ET.SubElement(mujoco, "tendon")
    strands = [
        (j, s, build_strand(hand, j, s, routes)) for j in ACTUATOR_ORDER for s in ("flex", "ext")
    ]
    strands += [(j, s, build_coupling_strand(hand, j, s)) for j in COUPLED for s in ("flex", "ext")]
    for joint, side, path in strands:
        strand = f"{joint}_{side}"
        rgba = "0.85 0.2 0.15 1" if side == "flex" else "0.15 0.35 0.85 1"
        spatial = ET.SubElement(tendon, "spatial", name=strand, width="0.0004", rgba=rgba)
        for i, p in enumerate(path):
            site = f"{strand}_{i}_{p.kind}"
            attrs = {"name": site, "pos": _fmt(local(p.body, p.pos))}
            if p.drum:
                side_site = f"{strand}_hub_side" if p.drum.endswith("_hub") else f"{strand}_side"
                ET.SubElement(
                    body_el[p.side_body],
                    "site",
                    {
                        "name": side_site,
                        "class": "drum",
                        "pos": _fmt(local(p.side_body, p.side_pos)),
                    },
                )
                ET.SubElement(spatial, "geom", geom=p.drum, sidesite=side_site)
            ET.SubElement(body_el[p.body], "site", attrs)
            ET.SubElement(spatial, "site", site=site)

    # The fixed parts are welded to the world, so MuJoCo's parent-child contact filter does not
    # apply to them; their convex hulls overlap the first moving part at the joint.
    contact = ET.SubElement(mujoco, "contact")
    for j in hand.joints.values():
        if j.parent in hand.fixed:
            ET.SubElement(contact, "exclude", body1=j.parent, body2=j.child)
    for body1, body2 in HULL_ARTIFACT_EXCLUDES:
        if body1 in hand.parts and body2 in hand.parts:
            ET.SubElement(contact, "exclude", body1=body1, body2=body2)

    # DIP coupling: dip = COUPLING_RATIO * pip. An inextensible coupling loop applies exactly this
    # constraint's force law (lambda * [1, -ratio] on dip, pip); the strands above are passive.
    equality = ET.SubElement(mujoco, "equality")
    for dip, pip in COUPLED.items():
        ET.SubElement(
            equality,
            "joint",
            name=f"{dip}_coupling",
            joint1=dip,
            joint2=pip,
            polycoef=_fmt([0, COUPLING_RATIO, 0, 0, 0]),
            solref=_fmt(COUPLING_SOLREF),
        )

    # Actuation. Each servo turns a spool (radius SPOOL_RADIUS) that pulls the flex strand and pays
    # out the ext strand by the same amount. Approximation: MuJoCo tendon actuators can push as
    # well as pull, so one bilateral actuator on the flex strand stands in for the whole
    # antagonistic loop (pulling the ext strand = pushing the flex strand). This is exact for a
    # tight, inextensible loop; pretension, friction and line stretch are not modelled.
    # ctrl is the JOINT angle (the Hand API is in joint space; the firmware converts to servo
    # angle = servo_per_joint * joint angle). With the drum radius r and n = r / r_spool:
    #   joint angle  q   = (L0_flex - L_flex) / r      (actuator length = gear * L, gear = -1/r)
    #   servo torque     = kp * n * (ctrl - q)  ->  joint torque f = n^2 * kp * (ctrl - q) - ...
    # Actuator force f is the joint torque; the servo's limit becomes n * its torque limit.
    actuator = ET.SubElement(mujoco, "actuator")
    for joint in ACTUATOR_ORDER:
        j = hand.joints[joint]
        r, n = drum_radius(joint), servo_per_joint(joint)
        kp, kv = SERVO_KP * n * n, SERVO_KV * n * n
        offset = (lengths0 or {}).get(f"{joint}_flex", 0.0) / r
        ET.SubElement(
            actuator,
            "general",
            {
                "name": joint,
                "tendon": f"{joint}_flex",
                "gear": _fmt([-1.0 / r], 10),
                "gaintype": "fixed",
                "biastype": "affine",
                "gainprm": _fmt([kp]),
                "biasprm": f"{-kp * offset:.12g} {-kp:.6g} {-kv:.6g}",
                "ctrlrange": _fmt([j.lo, j.hi]),
                "forcerange": _fmt([-SERVO_FORCE_LIMIT * n, SERVO_FORCE_LIMIT * n]),
            },
        )

    if qpos_order is not None:
        # ctrl is in actuator order; qpos in MuJoCo's joint order, each DIP on its coupling.
        def fist_q(n: str) -> float:
            if n in COUPLED:
                return COUPLING_RATIO * fist_q(COUPLED[n])
            return pose_deg(FIST_DEG, n) * DEG

        keyframe = ET.SubElement(mujoco, "keyframe")
        ET.SubElement(
            keyframe,
            "key",
            name="open",
            qpos=_fmt(np.zeros(len(qpos_order))),
            ctrl=_fmt(np.zeros(len(ACTUATOR_ORDER))),
        )
        ET.SubElement(
            keyframe,
            "key",
            name="fist",
            qpos=_fmt([fist_q(n) for n in qpos_order]),
            ctrl=_fmt([fist_q(n) for n in ACTUATOR_ORDER]),
        )
    return mujoco


def build(
    spec_path: Path = SPEC_PATH, routes_path: Path = ROUTES_PATH, meshdir: str | None = None
) -> ET.Element:
    """Full build: a first compile measures the flex strand lengths at q = 0 for the actuator offsets.

    `meshdir` overrides the mesh path (default: relative to OUT_PATH), e.g. to load the result from
    a string."""
    import mujoco

    hand = load_hand(spec_path)
    routes = load_routes(routes_path)
    probe = build_mjcf(hand, routes, meshdir=spec_path.parent.as_posix())
    model = mujoco.MjModel.from_xml_string(ET.tostring(probe, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lengths0 = {model.tendon(i).name: float(data.ten_length[i]) for i in range(model.ntendon)}
    qpos_order = [model.joint(i).name for i in range(model.njnt)]
    mjcf = build_mjcf(hand, routes, lengths0, meshdir, qpos_order)
    ET.indent(mjcf)
    return mjcf


def main() -> None:
    mjcf = build()
    header = (
        "<!-- GENERATED by sim/convert_v1.py from hardware/robot_description/v1_export. "
        "Do not edit by hand: change convert_v1.py and re-run it. -->\n"
    )
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(
        header + ET.tostring(mjcf, encoding="unicode") + "\n", encoding="utf-8", newline="\n"
    )
    print(
        f"Wrote {OUT_PATH.relative_to(ROOT)}"
        + (
            ""
            if ROUTES_PATH.exists()
            else " (no tendon_routes.json: strands end at the finger base)"
        )
    )


if __name__ == "__main__":
    main()
