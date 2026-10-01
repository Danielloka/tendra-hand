"""Tendon routes and forearm servo layout for the Tendra Hand V1 (single source of truth).

Every joint is driven by one SCS0009 servo in the forearm through an antagonistic loop: two strands
(`flex` and `ext`) that leave the finger base, run through their own channel in the palm, cross the
wrist, and go straight down to the two sides of the servo's spool (radius 6 mm, the loop wraps the
bottom half of the spool). 20 joints, 40 strands, 40 separate routes.

Layout idea (see research/experiments/2026-09-28-full-hand/):
- Servo shafts point inward, towards a central "core" where all strands run. Each spool's two strands
  are tangent at x_s -/+ 6 mm, in the spool's plane (a constant y, called a sheet).
- Servos sit in levels along the forearm. Deeper levels sit closer to the core (a staircase), so a
  strand going to a deeper spool always passes inside the upper spools and servos: no strand is ever
  blocked, and every strand is a straight line from the wrist to its spool.
- One column (x_s) per finger, the thumb gets the fifth column plus one slot on the third level.

In the palm each strand gets its own channel: 6 mm of 1.2 mm bore at the entry (bare line, tight
grid under the finger), then a 2.2 mm bore for a 1 x 2 mm PTFE tube (the step stops the tube). The
channel is a smooth S-curve (bend radius >= 15 mm) that ends vertical at the bottom of the wrist
plate, then the strand runs straight to the spool.

Run: `uv run python hardware/cad/tendon_router.py` -> hardware/robot_description/v1_export/tendon_routes.json
Tests: hardware/cad/tests/test_tendon_router.py (spacing, walls, bend radius, no blocked strand).
Units: mm, the Fusion design's world frame (fingers +Z, palm face -Y, little finger -X).
"""

from __future__ import annotations

import itertools
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parents[1] / "robot_description" / "v1_export" / "tendon_routes.json"

# --- Tendon and channel sizes (research.md) --------------------------------------------------------
SPOOL_R = 6.0  # tendon centreline radius on the spool = joint drum radius (1:1)
SPOOL_FLANGE_R = 7.5
SPOOL_HALF_T = 2.5  # spool half-thickness along its axis, around the groove plane
BORE_SMALL, SMALL_LEN = 1.2, 6.0  # bare-line entry section
BORE_TUBE = 2.2  # hole for a 1 x 2 mm PTFE tube
MIN_WALL = 0.8
MIN_BEND_R = 15.0

# --- Palm (see TendraHandV1.py stage 'palm') ---------------------------------------------------------
PALM_X = (-79.0, 12.0)
PALM_Y = (12.5, 40.0)
Z_WRIST = -30.0  # palm bottom = top of the forearm's wrist plate
Z_WRIST_PLATE_BOTTOM = -42.0  # channels end vertical here
INDEX_BELOW_BAY_Z = -9.5  # index channels stay beside the thumb bay until below its floor

# --- Thumb base and routing (research/experiments/2026-09-29-thumb-routing) --------------------------
# The base turns about a vertical axis (cmc_rot); its bottom plate and the bay floor sit BASE_DROP
# lower than in the prototype (moving along the rotation axis changes no kinematics). The bottom
# pivot is a hollow journal: 4 PTFE sheaths (mcp_flex, ip) and the 2 bare cmc_flex strands
# come up through it on the axis, where turning the base only twists them.
ROT_AXIS_XY = (1.0, 15.5236)  # thumb_cmc_rot axis (Z), hand_v1.json
CMC_FLEX_YZ = (10.0236, 11.3764)  # thumb_cmc_flex axis (along X)
BASE_DROP = 4.0
BAY_FLOOR_Z = -4.7 - BASE_DROP
THUMB_BAY = ((-16.0, 8.0), (12.5, 28.5), (BAY_FLOOR_Z, 35.4))
PLATE_Z = (-4.4 - BASE_DROP, 2.5 - BASE_DROP)  # base bottom plate (bottom, top)
BORE_R = 5.8  # through the plate and the journal
JOURNAL_R, JOURNAL_BOTTOM_Z = 7.0, BAY_FLOOR_Z - 3.0  # 3 mm deep bearing in the bay floor
BEARING_CLEAR = 0.25  # per side
# cmc_rot drum: groove in the plate, r 7.5 (larger than the spool: servo_per_joint = 7.5 / 6 = 1.25,
# needed because the bore takes the middle). Its strands leave the groove horizontally toward +Y.
ROT_DRUM_R, ROT_DRUM_Z, ROT_FLANGE_R, GROOVE_W = 7.5, -4.7, 9.0, 1.0
PIN_R, LINE_R = 1.5, 0.2  # 3 mm steel dowels as deflection pins; 0.4 mm line
ROT_PIN_YZ = (30.8, ROT_DRUM_Z - PIN_R - LINE_R)  # in the palm behind the bay's back wall, along X
ROT_PIN_X = (-9.0, 12.0)  # blind hole from the palm's thumb-side face
# cmc_flex drum on the metacarpal, centred over the rot axis: one groove per strand. Both strands
# leave its back side: flex straight down, ext up and over a pin on the base, then down to the axis.
CMC_DRUM_R, CMC_FLANGE_R = 6.0, 7.5
CMC_GROOVE_X = {"flex": 0.0, "ext": 2.0}  # 2 mm apart: room for both 1.2 mm palm bores
CMC_PIN_YZ = (CMC_FLEX_YZ[0] + CMC_DRUM_R + PIN_R + LINE_R, 18.5)  # low: lands ext near the axis
CMC_J = {
    "flex": (0.0, CMC_FLEX_YZ[0] + CMC_DRUM_R),
    "ext": (2.0, 15.9),
}  # plate-top crossing (x, y)
# Sheaths: 3 per side in the bore (flex strands on the -X side, ext on +X; |dx|, dy from the rot
# axis), looping over to a boss on top of the metacarpal that is split around the cmc_flex strands.
# There each side's 3 sockets sit side by side in x (sized by
# research/experiments/2026-09-29-thumb-routing/sheath_loop_search.py).
SHEATH_BORE = {  # (dx, dy) from the rot axis, found by a grid search (min distances in the tests)
    "flex": {"mcp_flex": (-4.6, -0.8), "ip": (-2.7, 2.4)},
    "ext": {"mcp_flex": (3.1, -0.7), "ip": (3.1, 2.3)},
}
SHEATH_JOINTS = ("mcp_flex", "ip")
# The bare cmc_flex strands are held by the palm right under the journal, below where they cross
# the plate at mid cmc_rot (-30 deg): the length change over the whole range is then smallest.
ROT_MID_DEG = -30.0
# Sockets: each sheath ends in the metacarpal's top block, 2 per side of the cmc_flex gap, 2.1 mm
# apart. The ip lines start on the inside (they head for the thumb's centre line), the mcp_flex
# lines on the outside (they reach their drum's sides). Metacarpal x -6.8..8.8 (measured).
SOCKET_X = {("ip", "flex"): -1.6, ("mcp_flex", "flex"): -4.8,
            ("ip", "ext"): 4.2, ("mcp_flex", "ext"): 7.0}  # fmt: skip
SOCKET_YZ, SOCKET_DIR = (9.0, 23.0), (-math.sqrt(0.5), math.sqrt(0.5))  # q = 0, into the boss
TUBE_OD, TUBE_MIN_BEND_R = 2.0, 12.0  # free PTFE tube (1 x 2 mm) in the loop
CMC_FLEX_RANGE_DEG = (-13.0, 45.0)  # V1 limit since the metacarpal got its tendon block
BASE_ENVELOPE = {"y_max": 24.0, "z_max": 30.0}  # loop stays inside the turning frame
FAN_Z = -25.0  # the sheath channels share one cavity from the journal down to here (fan_bottom_z())

# --- Inside the thumb, past the base (research/experiments/2026-09-29-thumb-routing, "Inside the
# thumb") ------------------------------------------------------------------------------------------
# The sheaths end in sockets on the metacarpal's top block; from there each bare line turns over a
# 2 mm steel pin ("pin A", along X) and runs straight on. The two mcp_flex lines pass under a second
# pin ("pin B") that levels them out, so they reach the MCP drum parallel to the thumb and in its
# plane. The ip lines cross the MCP axis through a 1.2 mm hole each (exactly on the axis) and run
# straight on, each in its own plane (a = const), to its own groove on the IP drum. A line only
# ever bends over steel, in a drum groove, or on a joint axis.
# Thumb frame: the MCP and IP axes are along A_AX = (1, 0, 1)/sqrt2 and both pass through the
# thumb's centre line (x 1.0, z 11.3764). a = position along the axis, w = across it (+w = the
# closing side of both drums, toward -x +z).
THUMB_CL_XZ = (1.0, 11.3764)
MCP_Y, IP_Y = -31.9764, -66.9764
THUMB_DRUM_R = 6.0  # tendon centreline on the MCP and IP drums (grooves deepened to r 5.8)
# Positions along the axes (searched for the most room between bores, see the experiment README):
# a new MCP drum groove on the proximal's barrel (solid at a -4.5..0.5; its bearing bosses start at
# |a| 4.5), the ip crossing holes on the MCP axis, and the matching IP drum grooves on the distal.
MCP_GROOVE_A = -2.0
IP_GROOVE_A = {"flex": 0.0, "ext": 3.0}
TUBE_IN = 3.0  # tube length inside its socket (the socket's step is the tube stop)
PIN_A_GAP = 1.5  # bare line from the tube end to pin A
PIN2_R = 1.0  # 2 mm steel dowels
PIN_B_Y = {"flex": -15.0, "ext": -19.5}  # the mcp_flex lines level out under these
GROOVE_HALF_W = 0.5  # thumb drum grooves are 1 mm wide, floor at THUMB_DRUM_R - LINE_R
PIN_A_X = {"flex": (-5.9, -0.5), "ext": (3.1, 8.0)}  # pin A, split around the cmc_flex gap
META_FILL = {
    "x": (-6.8, 8.8),
    "y": (-21.0, 6.0),
    "z": (3.0, 25.0),
    "front": (-6.0, 23.0),  # in front of y -6 the block is only 23 high (room when the thumb folds)
}  # solid block added to the metacarpal
CROSS_HOLE_D = 1.2
INSERTS = [
    (-64.0, 28.0),
    (-48.0, 28.0),
    (-32.0, 28.0),
    (0.0, 28.0),
]  # M3 heat-set inserts, palm bottom
INSERT_R, INSERT_DEPTH = 2.1, 6.0

# Finger sideways axes (x) and knuckle height changes (z) as built by TendraHandV1.py stage 'fingers'.
X_INDEX_ABD = -8.531
FINGER_DX = {"index": 0.0, "middle": -19.0, "ring": -38.0, "little": -57.0}
FINGER_DZ = {"index": 0.0, "middle": 4.0, "ring": 1.0, "little": -7.0}
Z_PLATE_TOP = 38.5  # index palm top under the knuckle post

# --- Forearm servo layout ----------------------------------------------------------------------------
COLUMNS = {"little": -64.0, "ring": -48.0, "middle": -32.0, "index": -16.0, "thumb": 0.0}
Z_LEVEL = {1: -55.0, 2: -90.0, 3: -125.0}
# sheet name -> (y of the spool/strand plane, level, shaft direction along y)
SHEETS = {
    "F1": (15.0, 1, +1),
    "F2": (20.0, 2, +1),
    "F3": (25.0, 3, +1),
    "B2": (31.0, 2, -1),
    "B1": (36.0, 1, -1),
}
FINGER_SLOTS = {"mcp_flex": "F1", "pip": "F2", "dip": "B2", "mcp_abd": "B1"}
# cmc_rot's strands come from the bay's back wall, so it takes the back sheet; the 8 strands that
# come up the journal take the 4 nearer sheets, in the same front-to-back order as they sit in the
# journal, so their channels don't cross (re-assigned 2026-09-29).
THUMB_SLOTS = {"mcp_flex": "F1", "cmc_flex": "F2", "ip": "B2", "cmc_rot": "B1"}  # F3 left free
JOURNAL_JOINTS = ("cmc_flex", "mcp_flex", "ip")

# SCS0009 (Feetech datasheet, research.md): body 23.3 (L) x 12.1 (W) x 25.25 (H to case top), tabs
# 32.5 long, 1.6 thick, underside 16.8 above the bottom, holes 28.5 apart; shaft 5.7 from the body
# centre along L, spline top 3.2 above the case top.
SERVO_L, SERVO_W, SERVO_H = 23.3, 12.1, 25.25
SERVO_TAB_L, SERVO_TAB_W, SERVO_TAB_T, SERVO_TAB_Z = 32.5, 11.0, 1.6, 16.8
SERVO_HOLE_SPACING = 28.5
SERVO_SHAFT_OFFSET = 5.7
SPOOL_PLANE_ABOVE_CASE = 5.0  # groove plane above the case top (spline 3.2 + hub)

# Servo IDs = firmware motor numbers (firmware/include/config_v1.h).
SERVO_IDS = {
    "index_dip": 1, "index_pip": 2, "index_mcp_flex": 3, "index_mcp_abd": 4,
    "thumb_ip": 5, "thumb_mcp_flex": 6, "thumb_cmc_flex": 7, "thumb_cmc_rot": 8,
    "middle_dip": 9, "middle_pip": 10, "middle_mcp_flex": 11, "middle_mcp_abd": 12,
    "ring_dip": 13, "ring_pip": 14, "ring_mcp_flex": 15, "ring_mcp_abd": 16,
    "little_dip": 17, "little_pip": 18, "little_mcp_flex": 19, "little_mcp_abd": 20,
}  # fmt: skip


@dataclass
class Servo:
    id: int
    joint: str
    sheet: str
    x: float  # spool centre
    y: float
    z: float
    shaft: int  # +1: output shaft points +Y, -1: -Y

    def case_box(self):
        """Axis-aligned box of the servo case (without tabs): ((x0,x1),(y0,y1),(z0,z1))."""
        top = self.y - self.shaft * SPOOL_PLANE_ABOVE_CASE
        bottom = top - self.shaft * SERVO_H
        zc = self.z - SERVO_SHAFT_OFFSET  # shaft is near the top end of the case
        return (
            (self.x - SERVO_W / 2, self.x + SERVO_W / 2),
            tuple(sorted((top, bottom))),
            (zc - SERVO_L / 2, zc + SERVO_L / 2),
        )

    def tab_box(self):
        top = self.y - self.shaft * SPOOL_PLANE_ABOVE_CASE
        bottom = top - self.shaft * SERVO_H
        t0 = bottom + self.shaft * SERVO_TAB_Z
        t1 = t0 + self.shaft * SERVO_TAB_T
        zc = self.z - SERVO_SHAFT_OFFSET
        return (
            (self.x - SERVO_TAB_W / 2, self.x + SERVO_TAB_W / 2),
            tuple(sorted((t0, t1))),
            (zc - SERVO_TAB_L / 2, zc + SERVO_TAB_L / 2),
        )

    def spool_box(self):
        return (
            (self.x - SPOOL_FLANGE_R, self.x + SPOOL_FLANGE_R),
            (self.y - SPOOL_HALF_T, self.y + SPOOL_HALF_T),
            (self.z - SPOOL_FLANGE_R, self.z + SPOOL_FLANGE_R),
        )


def servos() -> list[Servo]:
    out = []
    for joint, sid in SERVO_IDS.items():
        finger, _, rest = joint.partition("_")
        sheet = (THUMB_SLOTS if finger == "thumb" else FINGER_SLOTS)[rest]
        y, level, shaft = SHEETS[sheet]
        out.append(Servo(sid, joint, sheet, COLUMNS[finger], y, Z_LEVEL[level], shaft))
    return sorted(out, key=lambda s: s.id)


def _segment_clearance(a0, a1, b0, b1) -> float:
    """Smallest plan-view distance between two strands that move from a0->a1 and b0->b1 together
    (all channels follow the same S-profile, so at every height they are at the same fraction)."""
    d0, d1 = a0 - b0, a1 - b1
    dd = d1 - d0
    t = 0.0 if dd @ dd < 1e-12 else float(np.clip(-(d0 @ dd) / (dd @ dd), 0.0, 1.0))
    return float(np.linalg.norm(d0 + t * dd))


def _best_assignment(strands, slots, exits):
    """Assign strands to entry slots so the channels keep the most room between each other
    (brute force over all assignments, with the pair clearances precomputed)."""
    n, m = len(strands), len(slots)
    c = np.empty((n, m, n, m))
    for a, i, b, j in itertools.product(range(n), range(m), range(n), range(m)):
        c[a, i, b, j] = _segment_clearance(slots[i], exits[strands[a]], slots[j], exits[strands[b]])
    pairs = list(itertools.combinations(range(n), 2))
    best, best_perm = -1.0, None
    for perm in itertools.permutations(range(m), n):
        worst = math.inf
        for a, b in pairs:
            v = c[a, perm[a], b, perm[b]]
            if v < worst:
                worst = v
                if worst <= best:
                    break
        if worst > best:
            best, best_perm = worst, perm
    return {s: slots[best_perm[i]] for i, s in enumerate(strands)}


def exits_xy() -> dict[tuple[str, str], np.ndarray]:
    """Plan position (x, y) of every strand below the wrist (= its spool tangent point)."""
    out = {}
    for sv in servos():
        for side, s in (("flex", -1), ("ext", +1)):
            out[(sv.joint, side)] = np.array((sv.x + s * SPOOL_R, sv.y))
    return out


def entries() -> dict[tuple[str, str], np.ndarray]:
    """Entry hole (top of the channel) for every (joint, side)."""
    ex = exits_xy()
    out = {}
    for finger in ("index", "middle", "ring", "little"):
        xf = X_INDEX_ABD + FINGER_DX[finger]
        z = Z_PLATE_TOP + FINGER_DZ[finger]
        strands = [(f"{finger}_{j}", side) for j in FINGER_SLOTS for side in ("flex", "ext")]
        if finger == "index":
            # 2 x 4 grid in the gap between the index and middle knuckle posts (x -22..-16), beside
            # the thumb bay (the owner's prototype also routes the index strands sideways).
            slots = [np.array((x, y)) for x in (-18.2, -21.2) for y in (15.0, 18.2, 21.4, 24.6)]
        else:
            # 3 x 3 grid (3.4 mm pitch) in the open window of the knuckle post, under the tendon slit.
            # The window spans x_f - 6 .. x_f + 2.5 and y 13.5 .. 23.5 (measured on the prototype).
            slots = [np.array((xf + dx, y)) for dx in (-5.1, -1.7, 1.7) for y in (15.0, 18.4, 21.8)]
        for key, xy in _best_assignment(strands, slots, ex).items():
            out[key] = np.array((xy[0], xy[1], z))
    for side in ("flex", "ext"):
        out[("thumb_cmc_rot", side)] = rot_tangent(side) * (1, 0, 1) + (0, THUMB_BAY[1][1], 0)
        out[("thumb_cmc_flex", side)] = cmc_flex_hold(side)
        for j in SHEATH_JOINTS:
            jx, jy = journal_xy(j, side)
            s0, w = np.array((jx, jy, PLATE_Z[0])), wrist_bottom_of(f"thumb_{j}", side)
            out[(f"thumb_{j}", side)] = s_curve_at(s0, w, JOURNAL_BOTTOM_Z - BEARING_CLEAR)
    return out


# --- Thumb geometry ----------------------------------------------------------------------------------


def rot_tangent(side: str) -> np.ndarray:
    """Where a cmc_rot strand leaves its drum groove, heading +Y. Positive cmc_rot turns the base
    about -Z (the thumb swings toward -X), which a pull toward +Y on the -X side of the drum does."""
    s = -1 if side == "flex" else +1
    return np.array((ROT_AXIS_XY[0] + s * ROT_DRUM_R, ROT_AXIS_XY[1], ROT_DRUM_Z))


def journal_xy(joint: str, side: str) -> tuple[float, float]:
    """Plan position of a journal strand as it crosses the base plate (at cmc_rot = 0)."""
    if joint == "cmc_flex":
        return CMC_J[side]
    dx, dy = SHEATH_BORE[side][joint]
    return ROT_AXIS_XY[0] + dx, ROT_AXIS_XY[1] + dy


def cmc_flex_tube_z() -> float:
    """The two bare cmc_flex strands stay in 1.2 mm bores (6 mm straight down from their holds,
    then along their S-curves) until the channels are far enough apart for 2.2 mm tube bores."""
    curves = []
    for side in ("flex", "ext"):
        drop = cmc_flex_hold(side) + (0, 0, -SMALL_LEN)
        curves.append((drop, wrist_bottom_of("thumb_cmc_flex", side)))
    z = curves[0][0][2]
    while True:
        a, b = (s_curve_at(p0, p1, z) for p0, p1 in curves)
        if np.linalg.norm(a - b) >= BORE_TUBE + MIN_WALL:
            return round(float(z), 2)
        z -= 0.25


def cmc_flex_hold(side: str) -> np.ndarray:
    """Palm entry of a bare cmc_flex strand: under its plate crossing turned to cmc_rot = ROT_MID
    (positive cmc_rot turns the base about -Z)."""
    ax = np.array(ROT_AXIS_XY)
    a = -math.radians(ROT_MID_DEG)
    v = np.array(CMC_J[side]) - ax
    xy = ax + (v[0] * math.cos(a) - v[1] * math.sin(a), v[0] * math.sin(a) + v[1] * math.cos(a))
    return np.array((*xy, JOURNAL_BOTTOM_Z - BEARING_CLEAR))


def socket_x(joint: str, side: str) -> float:
    return SOCKET_X[(joint, side)]


def wrist_bottom_of(joint: str, side: str) -> np.ndarray:
    ex = exits_xy()[(joint, side)]
    return np.array((ex[0], ex[1], Z_WRIST_PLATE_BOTTOM))


def s_curve_at(p0, p1, z) -> np.ndarray:
    """Point of s_curve(p0, p1) at height z."""
    s = (p0[2] - z) / (p0[2] - p1[2])
    lat = (p1 - p0) * (1, 1, 0)
    p = p0 + lat * (1 - math.cos(math.pi * s)) / 2
    p[2] = z
    return p


def _rot(v, q):
    """Rotate (y, z) by thumb_cmc_flex = q (closing turns about -X)."""
    c, s = math.cos(-q), math.sin(-q)
    return np.array((v[0] * c - v[1] * s, v[0] * s + v[1] * c))


def socket_yz(joint: str, q: float = 0.0):
    """Sheath socket on the metacarpal boss (position, direction into the boss) at cmc_flex = q."""
    ax = np.array(CMC_FLEX_YZ)
    return ax + _rot(np.array(SOCKET_YZ) - ax, q), _rot(np.array(SOCKET_DIR), q)


_S = np.linspace(0.0, 1.0, 121)[:, None]


def _hermite(p0, t0, p1, t1, k0, k1):
    """Cubic Hermite curve in the (y, z) plane, tangent lengths k * chord; returns points and the
    smallest bend radius along it."""
    L = np.linalg.norm(p1 - p0)
    m0, m1, s = t0 * L * k0, t1 * L * k1, _S
    pts = (2 * s**3 - 3 * s**2 + 1) * p0 + (s**3 - 2 * s**2 + s) * m0
    pts = pts + (-2 * s**3 + 3 * s**2) * p1 + (s**3 - s**2) * m1
    d1 = (6 * s * s - 6 * s) * p0 + (3 * s * s - 4 * s + 1) * m0 + (-6 * s * s + 6 * s) * p1
    d1 = d1 + (3 * s * s - 2 * s) * m1
    d2 = (12 * s - 6) * p0 + (6 * s - 4) * m0 + (-12 * s + 6) * p1 + (6 * s - 2) * m1
    k = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.linalg.norm(d1, axis=1) ** 3
    return pts, 1.0 / max(k.max(), 1e-9)


def _hits_metacarpal(pts, q, sock):
    """Tube (radius TUBE_OD/2) inside the metacarpal? Its proximal block is y <= 11, 4 <= z <= 18
    (measured) and the reshaped part in front of it is y <= META_FILL["y"][1], z <= META_FILL["z"][1]
    (q = 0 pose); points within 2 mm of the socket are skipped."""
    ax = np.array(CMC_FLEX_YZ)
    back = np.array([_rot(p - ax, -q) for p in pts]) + ax
    r = TUBE_OD / 2
    inside = (back[:, 0] <= 11 + r) & (back[:, 1] >= 4 - r) & (back[:, 1] <= 18 + r)
    inside |= (back[:, 0] <= META_FILL["y"][1] + r) & (back[:, 1] <= META_FILL["z"][1] + r)
    return bool((inside & (np.linalg.norm(pts - sock, axis=1) > 2.0)).any())


def sheath_loop_3d(joint: str, side: str, q: float):
    """sheath_loop() with x: the sheaths spread sideways right above the plate (smoothly, over the
    first 40% of the loop), then loop in their own planes to the sockets (mm)."""
    pts, r = sheath_loop(joint, side, q)
    if pts is None:
        return None, r
    t = np.clip(np.linspace(0.0, 1.0, len(pts)) / 0.4, 0.0, 1.0)
    x0, x1 = journal_xy(joint, side)[0], socket_x(joint, side)
    x = x0 + (x1 - x0) * t * t * (3 - 2 * t)
    return np.column_stack((x, pts)), r


def sheath_loop(joint: str, side: str, q: float):
    """Best loop shape (largest smallest bend radius) for one sheath at cmc_flex = q, in the plane
    x = const: from the plate top (pointing up) to its socket, inside the base frame, clear of the
    metacarpal. Returns (points (y, z), bend radius) or (None, 0) if no shape fits."""
    _, jy = journal_xy(joint, side)
    p0, t0 = np.array((jy, PLATE_Z[1])), np.array((0.0, 1.0))
    p1, t1 = socket_yz(joint, q)
    best = (None, 0.0)
    for k0, k1 in itertools.product((1.0, 1.5, 2.0, 3.0), repeat=2):
        pts, r = _hermite(p0, t0, p1, t1, k0, k1)
        if r <= best[1]:
            continue
        free = (
            pts[:, 0].max() <= BASE_ENVELOPE["y_max"] and pts[:, 1].max() <= BASE_ENVELOPE["z_max"]
        )
        if free and pts[:, 1].min() >= PLATE_Z[1] - 1e-6 and not _hits_metacarpal(pts, q, p1):
            best = (pts, r)
    return best


def fan_bottom_z(rts=None) -> float:
    """Below this height every sheath channel has its own wall; above it (down from the journal)
    they share one cavity, where the sheaths are free to twist as the base turns."""
    rts = rts or routes()
    js = [np.array(r["points_mm"][:-1]) for r in rts if r.get("kind") == "sheath"]
    bores = [BORE_TUBE] * len(js)
    z = JOURNAL_BOTTOM_Z
    for zz in np.arange(JOURNAL_BOTTOM_Z, Z_WRIST_PLATE_BOTTOM, -0.25):
        xy = [np.array([np.interp(-zz, -p[:, 2], p[:, i]) for i in (0, 1)]) for p in js]
        ok = all(
            np.linalg.norm(xy[a] - xy[b]) >= (bores[a] + bores[b]) / 2 + MIN_WALL
            for a, b in itertools.combinations(range(len(js)), 2)
        )
        if not ok:
            z = zz - 0.25
    return round(float(z), 2)


A_AX = np.array((math.sqrt(0.5), 0.0, math.sqrt(0.5)))
W_AX = np.array((-math.sqrt(0.5), 0.0, math.sqrt(0.5)))


def thumb_pt(a: float, w: float, y: float) -> np.ndarray:
    """World point from thumb-frame coordinates (a along the MCP/IP axes, w across, y)."""
    cx, cz = THUMB_CL_XZ
    return np.array((cx, y, cz)) + a * A_AX + w * W_AX


def _tangent_from(c, r, q, left: bool):
    """2-D (y, z): tangent point on circle (c, r) for a line leaving the circle toward point q, with
    the circle centre on the left (left=True) or right of the direction of travel."""
    v = q - c
    d = np.linalg.norm(v)
    base = math.atan2(v[1], v[0])
    for sgn in (1, -1):
        ang = base + sgn * math.acos(r / d)
        t = c + r * np.array((math.cos(ang), math.sin(ang)))
        dirn = (q - t) / np.linalg.norm(q - t)
        lnorm = np.array((-dirn[1], dirn[0]))
        if ((c - t) @ lnorm > 0) == left:
            return t
    raise ValueError("no tangent")


def _cross_tangent(ca, cb, r):
    """2-D internal tangent from circle A (centre on the left of travel) to circle B (centre on the
    right), both radius r, travelling from A to B. Returns the two tangent points."""
    d = cb - ca
    base = math.atan2(d[1], d[0])
    th = base - math.asin(2 * r / np.linalg.norm(d))
    n = np.array((-math.sin(th), math.cos(th)))  # left normal of the travel direction
    return ca - r * n, cb + r * n


def thumb_inner(joint: str, side: str) -> dict:
    """Path of a sheathed thumb strand inside the thumb (q = 0, world mm), from its tube end to its
    drum. `points` lists (body, point) from the tube end outward; a crossing point on a joint axis
    belongs to both parts."""
    x = SOCKET_X[(joint, side)]
    sd = np.array(SOCKET_DIR)
    socket = np.array(SOCKET_YZ)
    tube_end = socket + TUBE_IN * sd
    rho = PIN2_R + LINE_R
    pa = tube_end + PIN_A_GAP * sd
    ca = pa + rho * np.array((-math.sqrt(0.5), -math.sqrt(0.5)))  # pin A below the line
    out = {"x": x, "socket": socket, "tube_end": tube_end, "pin_a": ca, "pin_a_in": pa}
    yz = [tube_end, pa]
    if joint == "mcp_flex":
        w = THUMB_DRUM_R if side == "flex" else -THUMB_DRUM_R
        tangent = thumb_pt(MCP_GROOVE_A, w, MCP_Y)  # on the drum, +w top / -w bottom
        cb = np.array((PIN_B_Y[side], tangent[2] + rho))  # pin B above the line
        p_out, p_in = _cross_tangent(ca, cb, rho)
        p_lvl = np.array((cb[0], tangent[2]))
        lx = float(tangent[0])  # from pin B on, the line runs at the drum's x (in its plane)
        out.update(
            pin_b=cb,
            pin_a_out=p_out,
            pin_b_in=p_in,
            pin_b_out=p_lvl,
            line_z=float(tangent[2]),
            line_x=lx,
            drum_tangent=tangent,
        )
        a3 = [np.array((x, *p)) for p in yz + [p_out]]
        b3 = [np.array((lx, *p)) for p in (p_in, p_lvl)]
        out["points"] = [("thumb_metacarpal", p) for p in a3 + b3]
        out["bores3"] = [(a3[1], a3[2]), (a3[2], b3[0]), (b3[0], b3[1]), (b3[1], tangent)]
    else:
        a = IP_GROOVE_A[side]
        cross = thumb_pt(a, 0.0, MCP_Y)  # on the MCP axis
        p_out = _tangent_from(ca, rho, np.array((cross[1], cross[2])), left=True)
        yz.append(p_out)
        c_ip = thumb_pt(a, 0.0, IP_Y)
        v = cross - c_ip
        d = np.linalg.norm(v)
        sgn = 1 if side == "flex" else -1
        ang = sgn * math.acos(THUMB_DRUM_R / d)
        tan_pt = c_ip + THUMB_DRUM_R * (math.cos(ang) * v / d + math.sin(ang) * W_AX)
        out.update(pin_a_out=p_out, cross=cross, cross_a=a, ip_tangent=tan_pt)
        a3 = [np.array((x, *p)) for p in yz]
        out["points"] = [("thumb_metacarpal", p) for p in a3] + [("thumb_proximal", cross)]
        out["bores3"] = [(a3[1], a3[2]), (a3[2], cross), (cross, tan_pt)]
    return out


def thumb_bores() -> dict:
    """Straight 1.2 mm bore segments (world mm, q = 0) of every sheathed strand inside the thumb."""
    return {
        f"thumb_{j}_{s}": thumb_inner(j, s)["bores3"]
        for j in SHEATH_JOINTS
        for s in ("flex", "ext")
    }


def _seg_dist(p0, p1, q0, q1, n=80) -> float:
    a, b = np.linspace(p0, p1, n), np.linspace(q0, q1, n)
    return float(np.linalg.norm(a[:, None] - b[None], axis=2).min())


def thumb_bore_clearance() -> list[tuple[float, str, str]]:
    """Closest approach between the bores of different strands (centre lines, mm), smallest first."""
    bores = thumb_bores()
    out = []
    for (ka, sa), (kb, sb) in itertools.combinations(bores.items(), 2):
        out.append((min(_seg_dist(*a, *b) for a in sa for b in sb), ka, kb))
    return sorted(out)


def _axis_point(p, axis_pt, axis_dir):
    d = np.asarray(axis_dir, float)
    return axis_pt + d * ((p - axis_pt) @ d)


def thumb_sim() -> dict:
    """The thumb's real tendon paths for the simulation (q = 0, world mm).

    For each strand: `points` from the drum outward (part name, point), `side_mm` (a point on the
    side the strand wraps the drum) and `anchor_dir` (unit vector from the drum centre to where the
    loop is tied, chosen so >= 20 deg of wrap is left at both joint limits). The sheaths are
    modelled by points on the two base axes: a point on a joint's axis doesn't move when it turns,
    which is what a sheath of fixed length does. `drums`: centre and half width per joint."""
    flex_c = np.array((0.0, *CMC_FLEX_YZ))
    x_ = np.array((1.0, 0.0, 0.0))
    strands, drums = {}, {}
    # MCP and IP: drums on the proximal / distal, centred on their grooves
    gm = thumb_pt(MCP_GROOVE_A, 0.0, MCP_Y)
    drums["thumb_mcp_flex"] = {"center_mm": gm.tolist(), "half_width_mm": GROOVE_HALF_W + 0.5}
    a0, a1 = sorted(IP_GROOVE_A.values())
    gi = thumb_pt((a0 + a1) / 2, 0.0, IP_Y)
    drums["thumb_ip"] = {
        "center_mm": gi.tolist(),
        "half_width_mm": (a1 - a0) / 2 + GROOVE_HALF_W + 0.5,
    }
    drums["thumb_cmc_flex"] = {"center_mm": [ROT_AXIS_XY[0], *CMC_FLEX_YZ], "half_width_mm": 1.8}
    drums["thumb_cmc_rot"] = {"center_mm": [*ROT_AXIS_XY, ROT_DRUM_Z], "half_width_mm": GROOVE_W}
    for j in SHEATH_JOINTS:
        for side in ("flex", "ext"):
            t = thumb_inner(j, side)
            sgn = 1 if side == "flex" else -1
            pts = [(b, p) for b, p in t["points"]][::-1]  # outward from the drum -> inward
            tube_end = t["points"][0][1]
            # sheath: through the cmc_flex axis and the rot axis, to the palm entry
            p_flex = _axis_point(tube_end, flex_c, x_)
            jx, jy = journal_xy(j, side)
            p_rot = np.array((*ROT_AXIS_XY, PLATE_Z[1]))
            pts += [("thumb_metacarpal", p_flex), ("thumb_base", p_rot)]
            if j == "ip":
                pts = [("thumb_proximal", t["cross"])] + [
                    pp for pp in pts if pp[0] != "thumb_proximal"
                ]
            strands[f"thumb_{j}_{side}"] = {
                "points": [(b, np.round(p, 4).tolist()) for b, p in pts],
                "side_mm": None,  # the simulation's usual drum rule (strand on the +w / -w side)
                "anchor_dir": None,
            }
    # cmc_flex: bare strands from the back of the drum, flex straight down, ext up over the pin
    cfc = np.array((ROT_AXIS_XY[0], *CMC_FLEX_YZ))
    for side in ("flex", "ext"):
        gx = CMC_GROOVE_X[side]
        jx, jy = CMC_J[side]
        jpt = np.array((jx, jy, PLATE_Z[1]))
        pts = []
        if side == "ext":
            py, pz = CMC_PIN_YZ
            rho = PIN_R + LINE_R
            pts.append(("thumb_base", np.array((gx, py - rho, pz))))  # up the pin's front
            pts.append(("thumb_base", np.array((gx, py, pz + rho))))  # over its top
            pts.append(("thumb_base", np.array((gx, py + rho, pz))))  # down its back
        pts.append(("thumb_base", jpt))
        # The real loop is tied once, at the front (200 deg), so the two wraps add up to 360 deg.
        # MuJoCo can't wrap a cylinder by more than 180 deg, so the model ties each strand on its
        # own: flex 120 deg over the top, ext 55 deg under the back (angles from +Y toward +Z;
        # closing turns them down). Both stay 20..160 deg over the range, with the same arms.
        a_anchor, a_side = (120.0, 60.0) if side == "flex" else (-55.0, -27.0)
        strands[f"thumb_cmc_flex_{side}"] = {
            "points": [(b, np.round(p, 4).tolist()) for b, p in pts],
            "side_mm": (
                cfc
                + 2
                * CMC_DRUM_R
                * np.array((0.0, math.cos(math.radians(a_side)), math.sin(math.radians(a_side))))
            ).tolist(),
            "anchor_dir": [0.0, math.cos(math.radians(a_anchor)), math.sin(math.radians(a_anchor))],
        }
    # cmc_rot: from the drum straight back (+Y) to the bay wall
    rc = np.array((*ROT_AXIS_XY, ROT_DRUM_Z))
    for side in ("flex", "ext"):
        sgn = -1 if side == "flex" else 1
        strands[f"thumb_cmc_rot_{side}"] = {
            "points": [],  # straight from the drum to the bay wall (MuJoCo finds the tangent)
            "side_mm": (rc + np.array((sgn * ROT_DRUM_R, -ROT_DRUM_R, 0.0))).tolist(),  # front side
            "anchor_dir": [math.cos(math.radians(240)), math.sin(math.radians(240)), 0.0],
        }
    return {"strands": strands, "drums": drums}


def _unit(v):
    return v / np.linalg.norm(v)


def thumb_inner_check() -> dict:
    """Numbers the tests look at: how far each line bends over its pins, and the ip lines' skew on
    pin A (they drift sideways from their socket to their crossing hole)."""
    res = {}
    sd = np.array(SOCKET_DIR)
    for j in SHEATH_JOINTS:
        for side in ("flex", "ext"):
            t = thumb_inner(j, side)
            if j == "mcp_flex":
                d3 = t["bores3"][1][1] - t["bores3"][1][0]
                d1 = d3[1:] / np.linalg.norm(d3[1:])
                r = {
                    "pin_a_wrap_deg": math.degrees(math.acos(np.clip(sd @ d1, -1, 1))),
                    "pin_b_wrap_deg": math.degrees(
                        math.acos(np.clip(d1 @ np.array((-1.0, 0.0)), -1, 1))
                    ),
                    "skew_a_to_b_deg": math.degrees(math.asin(abs(d3[0]) / np.linalg.norm(d3))),
                }
            else:
                start = np.array((t["x"], *t["pin_a_out"]))
                d1 = t["cross"] - start
                d1 = d1 / np.linalg.norm(d1)
                r = {
                    "pin_a_wrap_deg": math.degrees(
                        math.acos(np.clip(sd @ d1[1:] / np.linalg.norm(d1[1:]), -1, 1))
                    ),
                    "skew_on_pin_a_deg": math.degrees(math.asin(abs(d1[0]))),
                }
            res[f"thumb_{j}_{side}"] = {k: round(v, 1) for k, v in r.items()}
    return res


def thumb_layout() -> dict:
    qs = np.radians(np.linspace(*CMC_FLEX_RANGE_DEG, 7))
    loops = {}
    for j in SHEATH_JOINTS:
        for side in ("flex", "ext"):
            loops[f"thumb_{j}_{side}"] = min(sheath_loop(j, side, q)[1] for q in qs)
    return {
        "note": "cmc_rot = cmc_flex = 0; see research/experiments/2026-09-29-thumb-routing",
        "base_drop_mm": BASE_DROP,
        "bay_floor_z": BAY_FLOOR_Z,
        "plate_z": PLATE_Z,
        "rot_axis_xy": ROT_AXIS_XY,
        "cmc_flex_axis_yz": CMC_FLEX_YZ,
        "bore_r": BORE_R,
        "journal": {"r": JOURNAL_R, "bottom_z": JOURNAL_BOTTOM_Z, "clearance": BEARING_CLEAR},
        "rot_drum": {"r": ROT_DRUM_R, "z": ROT_DRUM_Z, "flange_r": ROT_FLANGE_R, "groove_w": GROOVE_W,
                     "servo_per_joint": ROT_DRUM_R / SPOOL_R},
        "rot_pin_yz": ROT_PIN_YZ,
        "rot_pin_x": ROT_PIN_X,
        "cmc_drum": {"r": CMC_DRUM_R, "flange_r": CMC_FLANGE_R, "groove_x": CMC_GROOVE_X,
                     "groove_w": GROOVE_W},
        "cmc_pin_yz": CMC_PIN_YZ,
        "pin_r": PIN_R,
        "line_r": LINE_R,
        "journal_xy": {f"thumb_{j}_{s}": journal_xy(j, s) for j in JOURNAL_JOINTS for s in ("flex", "ext")},
        "sockets": {f"thumb_{j}_{s}": [socket_x(j, s), *np.round(socket_yz(j)[0], 3).tolist()]
                    for j in SHEATH_JOINTS for s in ("flex", "ext")},
        "socket_dir_yz": SOCKET_DIR,
        "loop_min_bend_mm": {k: round(float(v), 2) for k, v in loops.items()},
        "fan_bottom_z": fan_bottom_z(),
        "inner": {k: {kk: (np.round(vv, 4).tolist() if isinstance(vv, np.ndarray) else vv)
                      for kk, vv in thumb_inner(*k.removeprefix("thumb_").rsplit("_", 1)).items()
                      if kk in ("x", "socket", "tube_end", "pin_a", "pin_b", "line_x", "line_z", "cross", "cross_a", "ip_tangent", "drum_tangent")}
                  for k in [f"thumb_{j}_{s}" for j in SHEATH_JOINTS for s in ("flex", "ext")]},
        "pin_a_x": PIN_A_X,
        "pin_b_y": PIN_B_Y,
        "pin2_r": PIN2_R,
        "tube_in": TUBE_IN,
        "meta_fill": META_FILL,
        "mcp_groove_a": MCP_GROOVE_A,
        "ip_groove_a": IP_GROOVE_A,
        "groove_half_w": GROOVE_HALF_W,
        "thumb_drum_r": THUMB_DRUM_R,
        "sim": thumb_sim(),
        "cl_xz": THUMB_CL_XZ,
        "mcp_y": MCP_Y,
        "ip_y": IP_Y,
        "cross_hole_d": CROSS_HOLE_D,
        "bore_tube": BORE_TUBE,
        "bore_small": BORE_SMALL,
        "bores": {k: [[np.round(p, 4).tolist() for p in seg] for seg in v] for k, v in thumb_bores().items()},
        "pin_a_out": {f"thumb_{j}_{s}": np.round(thumb_inner(j, s)["pin_a_out"], 4).tolist()
                      for j in SHEATH_JOINTS for s in ("flex", "ext")},
    }  # fmt: skip


def s_curve(p0: np.ndarray, p1: np.ndarray, n: int = 40) -> np.ndarray:
    """Vertical-in, vertical-out S-curve from p0 down to p1 (lateral move follows 1 - cos)."""
    s = np.linspace(0.0, 1.0, n)
    lat = (p1 - p0).copy()
    lat[2] = 0.0
    pts = p0 + np.outer((1 - np.cos(math.pi * s)) / 2, lat)
    pts[:, 2] = p0[2] + (p1[2] - p0[2]) * s
    return pts


def s_curve_min_radius(p0, p1) -> float:
    d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    h = abs(p1[2] - p0[2])
    return math.inf if d < 1e-9 else 2 * h * h / (math.pi**2 * d)


def _thumb_path(joint: str, side: str, e: np.ndarray, wrist_bottom: np.ndarray):
    """Thumb strands: (kind, points_mm, small_end, curve_start, S-curve start, thumb-side points).

    - cmc_rot: from its drum groove straight back (+Y) through the bay's back wall, over the pin,
      down 6 mm of bare-line bore, then the usual tube S-curve.
    - journal strands: down through the hollow journal on the rot axis; the S-curve starts at the
      plate bottom. The sheaths (mcp_flex, ip) run 2.2 mm all the way and share one
      cavity at first (fan_bottom_z()); the bare cmc_flex strands get the usual 1.2 mm entry, each
      in its own bore."""
    j = joint.removeprefix("thumb_")
    if j == "cmc_rot":
        py, pz = ROT_PIN_YZ
        rr = PIN_R + LINE_R
        arc = [
            np.array((e[0], py + rr * math.sin(a), pz + rr * math.cos(a)))
            for a in np.linspace(0, math.pi / 2, 5)
        ]
        small_end = arc[-1] + (0, 0, -SMALL_LEN)
        pts = [e, *arc, small_end]
        return "rot", pts, small_end, small_end, small_end, [rot_tangent(side)]
    jx, jy = journal_xy(j, side)
    s0 = np.array((jx, jy, PLATE_Z[0]))
    full = s_curve(s0, wrist_bottom, 80)
    below = [p for p in full if p[2] < e[2] - 1e-9]
    top = [np.array((jx, jy, PLATE_Z[1])), s0] + [p for p in full[1:] if p[2] > e[2] + 1e-9]
    if j == "cmc_flex":  # own 1.2 mm bore right under the journal, then the tube's S-curve
        drop = e + (0, 0, -SMALL_LEN)
        z_t = cmc_flex_tube_z()
        small_end = s_curve_at(drop, wrist_bottom, z_t)
        below = [p for p in s_curve(drop, wrist_bottom, 80)[1:] if p[2] < z_t - 1e-9]
        mid = [p for p in s_curve(drop, wrist_bottom, 80)[1:] if p[2] > z_t + 1e-9]
        top = [np.array((jx, jy, PLATE_Z[1]))]
        return "bare", [e, drop, *mid, small_end, *below], small_end, small_end, drop, top
    return "sheath", [e, *below], e, e, s0, top


def routes() -> list[dict]:
    ent = entries()
    out = []
    for sv in servos():
        for side, s in (("flex", -1), ("ext", +1)):
            e = ent[(sv.joint, side)]
            tangent = np.array((sv.x + s * SPOOL_R, sv.y, sv.z))
            wrist_bottom = np.array((tangent[0], tangent[1], Z_WRIST_PLATE_BOTTOM))
            extra = {}
            if sv.joint.startswith("thumb"):
                kind, pts, small_end, curve_start, s0, thumb_side = _thumb_path(
                    sv.joint, side, e, wrist_bottom
                )
                extra = {
                    "kind": kind,
                    "thumb_side_mm": [np.round(p, 3).tolist() for p in thumb_side],
                }
            else:
                small_end = e + (0, 0, -SMALL_LEN)
                z_a = small_end[2]
                if sv.joint.startswith("index"):
                    z_a = min(z_a, INDEX_BELOW_BAY_Z)
                s0 = curve_start = np.array((e[0], e[1], z_a))
                pts = [e, small_end]
                if z_a < small_end[2]:
                    pts.append(curve_start)
            if not sv.joint.startswith("thumb") or extra["kind"] == "rot":
                pts = pts + list(s_curve(curve_start, wrist_bottom)[1:])
            pts.append(tangent)
            out.append({
                "name": f"{sv.joint}_{side}",
                "joint": sv.joint,
                "side": side,
                "servo_id": sv.id,
                "entry_mm": e.round(3).tolist(),
                "small_bore_end_mm": small_end.round(3).tolist(),
                "curve_start_mm": np.round(curve_start, 3).tolist(),
                "wrist_bottom_mm": wrist_bottom.round(3).tolist(),
                "tangent_mm": tangent.round(3).tolist(),
                "points_mm": [np.round(p, 3).tolist() for p in pts],
                "min_bend_radius_mm": round(s_curve_min_radius(s0, wrist_bottom), 2),
                "spool_center_mm": [sv.x, sv.y, sv.z],
                "spool_axis": [0, sv.shaft, 0],
                **extra,
            })  # fmt: skip
    return out


def layout() -> dict:
    return {
        "source": "hardware/cad/tendon_router.py",
        "units": "mm, world frame of the Fusion design 'Tendra Hand V1', all joints at 0",
        "spool_radius_mm": SPOOL_R,
        "bores_mm": {"entry": BORE_SMALL, "entry_length": SMALL_LEN, "tube": BORE_TUBE},
        "palm": {"x": PALM_X, "y": PALM_Y, "z_wrist": Z_WRIST, "z_wrist_plate_bottom": Z_WRIST_PLATE_BOTTOM,
                 "inserts_xy": INSERTS, "insert_r": INSERT_R, "insert_depth": INSERT_DEPTH},
        "servo": {"L": SERVO_L, "W": SERVO_W, "H": SERVO_H, "tab_L": SERVO_TAB_L, "tab_W": SERVO_TAB_W,
                  "tab_T": SERVO_TAB_T, "tab_z": SERVO_TAB_Z, "hole_spacing": SERVO_HOLE_SPACING,
                  "shaft_offset": SERVO_SHAFT_OFFSET, "spool_plane_above_case": SPOOL_PLANE_ABOVE_CASE,
                  "spool_flange_r": SPOOL_FLANGE_R, "spool_half_t": SPOOL_HALF_T},
        "servos": [
            {"id": s.id, "joint": s.joint, "sheet": s.sheet, "spool_center_mm": [s.x, s.y, s.z],
             "shaft_dir": [0, s.shaft, 0], "case_box_mm": s.case_box(), "tab_box_mm": s.tab_box()}
            for s in servos()
        ],
        "strands": routes(),
        "thumb": thumb_layout(),
    }  # fmt: skip


def main() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    data = layout()
    OUT.write_text(json.dumps(data, indent=1), encoding="utf-8")
    worst = min(s["min_bend_radius_mm"] for s in data["strands"])
    print(
        f"{len(data['strands'])} strands, {len(data['servos'])} servos -> {OUT} (min bend {worst} mm)"
    )


if __name__ == "__main__":
    main()
