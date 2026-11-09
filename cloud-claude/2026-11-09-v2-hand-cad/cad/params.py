"""Shared skeleton and constants for the Tendra V2 concept hand (study, 2026-11-09).

Frame (mm): origin = wrist centre (the wrist's two axes cross here), +Z toward the fingers,
-Y = palm side (flexion bends toward -Y), +Y = back of the hand, +X = thumb side. Right hand.
Positive joint angle = closing the hand. All values at q = 0 (hand flat, fingers straight).

Only the integrator edits this file. Other agents: request changes in your report.
Proportions: about an adult male hand (wrist centre to middle fingertip ~ 190 mm), close to V1.
"""

from __future__ import annotations

# --- Tendon drive -------------------------------------------------------------------------------
R_SPOOL = 5.0  # servo spool radius (same spool on every servo)
R_DRUM = 7.0  # common joint drum radius (owner: "same distance on each degree")
SERVO_PER_JOINT = R_DRUM / R_SPOOL  # 1.4 servo degrees per joint degree, for every joint
MM_PER_DEG = 3.141592653589793 * 2 * R_DRUM / 360.0  # 0.1222 mm of strand per joint degree
GROOVE_W = 1.0  # drum groove width per strand
LINE_D = 0.36  # braided UHMWPE, 50 lb
SHEATH_ID, SHEATH_OD, SHEATH_BORE = 1.0, 2.0, 2.2
BARE_BORE = 1.2
SHEATH_MIN_BEND_R = 15.0

# --- Print / hardware -----------------------------------------------------------------------------
WALL_MIN = 1.6
CLEAR_SLIDE = 0.25
PRESS_FIT = -0.1
PIN_D_FINGER = 2.0
PIN_D_BIG = 3.0  # thumb base, wrist

# --- Fingers ---------------------------------------------------------------------------------------
# mcp: MCP flexion axis centre (x, y, z); axes along -X/+X (flexion), abduction axis along Y.
# abd_offset: MCP abduction axis sits this far proximal (-Z) of the flexion axis (V1 used 19 mm; a
# human MCP has them nearly coincident; designers may shrink it).
# lengths: proximal (MCP->PIP), middle (PIP->DIP), distal (DIP->tip) in mm. width/height of the phalanx.
FINGERS = {
    "index": {"mcp": (20.0, -4.0, 92.0), "lengths": (42.0, 24.0, 23.0), "width": 18.0, "height": 16.0},
    "middle": {"mcp": (0.0, -4.0, 96.0), "lengths": (45.0, 27.0, 24.0), "width": 18.0, "height": 16.0},
    "ring": {"mcp": (-19.5, -4.0, 93.0), "lengths": (43.0, 26.0, 23.0), "width": 17.0, "height": 15.5},
    "little": {"mcp": (-38.0, -4.0, 85.0), "lengths": (34.0, 20.0, 21.0), "width": 15.5, "height": 14.0},
}
ABD_OFFSET_DEFAULT = 10.0
# Joint ranges, degrees (min, max); positive = closing. From V1 config_v1.h, adjust with reason.
FINGER_RANGES = {"mcp_abd": (-15.0, 15.0), "mcp_flex": (-10.0, 90.0), "pip": (-5.0, 95.0)}
DIP_RATIO = 0.75  # DIP = 0.75 * PIP via a rigid bar (owner)

# --- Thumb ----------------------------------------------------------------------------------------
# The thumb designer owns the thumb kinematics (axis directions, CMC placement). Budget/constraints:
THUMB_JOINTS = ("cmc_rot", "cmc_flex", "mcp_flex", "ip")  # 4 servos (IDs 4-7 in V1)
THUMB_CMC_REGION = {"x": (15.0, 40.0), "y": (-25.0, 5.0), "z": (10.0, 45.0)}
THUMB_LENGTHS_HINT = (46.0, 33.0, 28.0)  # metacarpal, proximal, distal (human-ish)

# --- Palm -----------------------------------------------------------------------------------------
PALM_Y = (-14.0, 14.0)  # palm thickness (palm face at -14, back at +14), may arch
PALM_Z_BASE = 15.0  # palm's proximal face (wrist gap 15 mm on each side of the centre = 30 mm)

# --- Wrist (cloud-claude/2026-11-04-v1-wrist) -------------------------------------------------------
WRIST_RANGES = {"wrist_flex": (-60.0, 70.0), "wrist_dev": (-30.0, 20.0)}  # flex + = toward palm, dev + = thumb
WRIST_GAP = 30.0  # palm base (z=+15) to forearm top (z=-15)
WRIST_OPENING = (22.0, 14.0)  # hollow oval for the sheath bundle (x, y)
FOREARM_Z_TOP = -15.0

# --- Servos ---------------------------------------------------------------------------------------
SCS0009 = {"L": 23.3, "W": 12.1, "H": 25.25, "tab_L": 32.5, "tab_W": 11.0, "tab_T": 1.6, "tab_z": 16.8,
           "hole_spacing": 28.5, "shaft_offset": 5.7}
HLS_EXTRA_H = 2.25
STS3215 = {"L": 45.2, "W": 24.7, "H": 35.0}

SERVO_IDS = {  # protocol order = V1
    1: "index_pip", 2: "index_mcp_flex", 3: "index_mcp_abd",
    4: "thumb_ip", 5: "thumb_mcp_flex", 6: "thumb_cmc_flex", 7: "thumb_cmc_rot",
    8: "middle_pip", 9: "middle_mcp_flex", 10: "middle_mcp_abd",
    11: "ring_pip", 12: "ring_mcp_flex", 13: "ring_mcp_abd",
    14: "little_pip", 15: "little_mcp_flex", 16: "little_mcp_abd",
}
