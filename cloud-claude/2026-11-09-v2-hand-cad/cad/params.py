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
ABD_OFFSET_DEFAULT = 11.5  # integrator 2026-11-10: finger request (palmar abd boss clears the proximal at 90 deg)
KNUCKLE_MIN_W = 17.0  # MCP knuckle (yoke + drum) width is max(width, 17): little finger keeps 1.2 mm drum flanges
# Finger MCPs stay at y = -4 (thumb opposition was optimised against it). The palmar abd pin (hand y -16.5..-13.25)
# sits below the -14 palm face, so the palm bulges locally at the knuckle row (human distal palmar pad):
PALM_KNUCKLE_BULGE = {"palmar_y": -18.1, "dorsal_y": 15.6}  # pin ends + 1.6 wall
# Adjacent yokes collide only when neighbours abduct toward each other; spacing kept (~19.5-20 mm), limit in software:
ABD_PAIR_LIMIT = "q_abd(radial finger) - q_abd(ulnar neighbour) >= -14 deg (yoke envelopes touch beyond)"
# Joint ranges, degrees (min, max); positive = closing. From V1 config_v1.h, adjust with reason.
FINGER_RANGES = {"mcp_abd": (-15.0, 15.0), "mcp_flex": (-10.0, 90.0), "pip": (-5.0, 95.0)}
DIP_RATIO = 0.75  # DIP = 0.75 * PIP via a rigid bar (owner)

# --- Thumb ----------------------------------------------------------------------------------------
# The thumb designer owns the thumb kinematics (axis directions, CMC placement). Budget/constraints:
THUMB_JOINTS = ("cmc_rot", "cmc_flex", "mcp_flex", "ip")  # 4 servos (IDs 4-7 in V1)
THUMB_CMC_REGION = {"x": (15.0, 40.0), "y": (-25.0, 5.0), "z": (10.0, 55.0)}  # z to 55: thumb request
THUMB_LENGTHS_HINT = (46.7, 36.0, 31.6)  # CMC centre->MCP, MCP->IP, IP->tip (thumb optimiser)
# Thumb kinematics (out/thumb/thumb_design.json is the source; recorded here for the other modules):
THUMB_P = (39.79, -17.86, 50.44)  # CMC centre: cmc_rot and cmc_flex axes intersect here
THUMB_AXES = {"cmc_rot": (0.4098, -0.5650, 0.7161), "cmc_flex": (0.1319, -0.7401, -0.6594)}  # at q = 0, through P
THUMB_RANGES = {"cmc_rot": (-31.0, 110.0), "cmc_flex": (-37.0, 63.0), "mcp_flex": (-10.0, 70.0), "ip": (-15.0, 85.0)}
THUMB_MOUNT = {"bolt_circle_r": 18.5, "bolts": "3 x M3 (heat-set inserts in the palm)", "depth_below_P": 37.0}
THUMB_HLS = ("thumb_cmc_rot", "thumb_cmc_flex", "thumb_mcp_flex")  # HLS3606M recommended (IDs 7, 6, 5)

# --- Palm -----------------------------------------------------------------------------------------
PALM_Y = (-14.0, 14.0)  # palm thickness (palm face at -14, back at +14), may arch
PALM_Z_BASE = 15.0  # palm's proximal face (wrist gap 15 mm on each side of the centre = 30 mm)

# --- Wrist (cloud-claude/2026-11-04-v1-wrist) -------------------------------------------------------
# flex +-50 accepted with the 30 mm gap (report 03). Alternative for -60..+70: a 45-50 mm gap (not designed).
WRIST_RANGES = {"wrist_flex": (-50.0, 50.0), "wrist_dev": (-30.0, 20.0)}  # flex + = toward palm, dev + = thumb
WRIST_RANGES_ALT = {"wrist_flex": (-60.0, 70.0), "needs_gap_mm": (45.0, 50.0)}
WRIST_ORDER = ("wrist_dev", "wrist_flex")  # dev = OUTER (forearm side), flex = inner: R_palm = Ry(dev) @ Rx(flex)
WRIST_GAP = 30.0  # palm base (z=+15) to forearm top (z=-15)
WRIST_OPENING = (24.0, 37.0)  # ring opening (x, y); the bundle sweep needs 19 x 29.4
FOREARM_Z_TOP = -18.0  # wrist base plate z -18..-15 sits on the forearm top
# Palm heel keep-out (report 03, needed for flex +-52): palm material for z < 36 and |x| < 9.5 only inside
# r_x <= 18 (distance from the flex axis) or inside a +-20 deg wedge about +Z; for z < 22 inside r_x <= 18 for
# |x| < 11.5 and |y| <= 3.5 for 11.5 < |x| < 14.5. Push-rod balls: keep x 19.9..28.5, y 3.5..10, z -2..15 free.
PALM_HEEL = {"z_top": 36.0, "x_half": 9.5, "r_x": 18.0, "wedge_deg": 20.0, "z_low": 22.0, "x_half_low": 11.5,
             "x_band": (11.5, 14.5), "band_y": 3.5}
WRIST_FLANGE_BOLTS = [(sx * 17.0, sy * 9.5) for sx in (-1, 1) for sy in (-1, 1)]  # M3, inserts in the flange
SHEATH_CLAMP_Z = (14.0, 19.0)  # sheaths leave the clamp top (z 19) along +Z in their bundle cells
FOREARM_ENVELOPE = {"x": (-47.5, 47.5), "y": (-33.0, 33.0), "z": (-230.0, -18.0)}
ARM_FLANGE = {"z": -230.0, "bolts": "4 x M4", "pcd": 50.0, "bore": 24.0}  # stand-in for OpenArm J5: verify
PIN_D_WRIST = 3.0  # 4 mm pins + 624 bearings accepted in principle, but 624 (13 OD) needs ring bosses r >= 8.1
# against r 6.6 in a 7 mm ring: does not fit without a wrist rework -> stays 3 mm + 623 (open issue)

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
