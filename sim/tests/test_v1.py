"""Checks for the v1 (20-DOF, tendon-driven) model. Run: uv run pytest sim/tests/test_v1.py -s

The tendon tests prove the routing rule of the real hand (see sim/convert_v1.py, build_strand):
each joint's loop changes length only when its own joint moves (6 mm per rad on the drum), and
every other joint leaves it alone.
"""

import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import convert_v1 as cv

ROOT = Path(__file__).resolve().parents[2]
CONFIG_V1 = ROOT / "firmware" / "include" / "config_v1.h"

# Servo ID order (ID = index + 1), the contract with firmware/include/config_v1.h and software/.
SERVO_ORDER = [
    "index_dip",
    "index_pip",
    "index_mcp_flex",
    "index_mcp_abd",
    "thumb_ip",
    "thumb_mcp_flex",
    "thumb_cmc_flex",
    "thumb_cmc_rot",
    "middle_dip",
    "middle_pip",
    "middle_mcp_flex",
    "middle_mcp_abd",
    "ring_dip",
    "ring_pip",
    "ring_mcp_flex",
    "ring_mcp_abd",
    "little_dip",
    "little_pip",
    "little_mcp_flex",
    "little_mcp_abd",
]

# Where the fingertip must move for a small positive (closing) rotation, world frame at q = 0.
X, Y, Z = np.eye(3)
EXPECTED_TIP_MOTION = {
    **{
        f"{f}_{j}": -Y
        for f in ("index", "middle", "ring", "little")
        for j in ("mcp_flex", "pip", "dip")
    },
    **{f"{f}_mcp_abd": X for f in ("index", "middle", "ring", "little")},  # toward the thumb
    "thumb_cmc_rot": -X,  # across the palm (opposition)
    "thumb_cmc_flex": Z,  # thumb curls toward the fingers
    "thumb_mcp_flex": Z,
    "thumb_ip": Z,
}


def drum_mm(joint):
    return cv.drum_radius(joint) * 1000


COUPLING_LIMIT_MM_PER_RAD = 0.3
MOMENT_ARM_TOL_MM_PER_RAD = 0.1
LOOP_TOL_MM = 0.5
TRACKING_TOL_DEG = 2.0


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(str(cv.OUT_PATH))


def firmware_joints() -> list[tuple[str, int, float, float]]:
    """(name, servo_id, min_deg, max_deg) from config_v1.h, in file order."""
    text = CONFIG_V1.read_text(encoding="utf-8")
    pattern = r'\{"(\w+)",\s*(\d+),\s*(-?\d+(?:\.\d+)?)\s*\*\s*kDegToRad,\s*(-?\d+(?:\.\d+)?)\s*\*\s*kDegToRad'
    return [(n, int(i), float(lo), float(hi)) for n, i, lo, hi in re.findall(pattern, text)]


def qadr(model, joint: str) -> int:
    return model.jnt_qposadr[model.joint(joint).id]


def tendon_lengths(model, data, qpos) -> np.ndarray:
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    mujoco.mj_tendon(model, data)
    return data.ten_length.copy()


def length_jacobian(model, data, qpos, h=1e-4) -> np.ndarray:
    """d(strand length)/dq by central differences (ntendon x nq, m/rad). Independent of ten_J."""
    jac = np.zeros((model.ntendon, model.nq))
    for k in range(model.nq):
        dq = np.zeros(model.nq)
        dq[k] = h
        jac[:, k] = (
            tendon_lengths(model, data, qpos + dq) - tendon_lengths(model, data, qpos - dq)
        ) / (2 * h)
    return jac


def random_poses(model, n, seed=0):
    rng = np.random.default_rng(seed)
    lo, hi = model.jnt_range[:, 0], model.jnt_range[:, 1]
    return [np.zeros(model.nq)] + [rng.uniform(lo, hi) for _ in range(n)]


def settle(model, data, ctrl, steps):
    data.ctrl[:] = ctrl
    for _ in range(steps):
        mujoco.mj_step(model, data)


def actuated_q(model, data) -> np.ndarray:
    return np.array([data.qpos[qadr(model, n)] for n in SERVO_ORDER])


def contact_report(model, data) -> list[str]:
    out = []
    for c in data.contact[: data.ncon]:
        b1, b2 = (model.body(model.geom_bodyid[g]).name for g in (c.geom1, c.geom2))
        out.append(f"{b1} <-> {b2} ({c.dist * 1000:+.2f} mm)")
    return out


# ----- Model up to date, structure, contract with the firmware --------------------------------


def test_generated_model_is_up_to_date():
    fresh = ET.tostring(cv.build(), encoding="unicode")
    assert fresh in cv.OUT_PATH.read_text(encoding="utf-8"), (
        "Re-run: uv run python sim/convert_v1.py"
    )


def test_counts_and_names(model):
    assert model.njnt == 20 and model.nu == 20 and model.ntendon == 40
    assert sorted(model.joint(i).name for i in range(model.njnt)) == sorted(SERVO_ORDER)
    strands = [model.tendon(i).name for i in range(model.ntendon)]
    assert strands == [f"{j}_{s}" for j in SERVO_ORDER for s in ("flex", "ext")]


def test_actuators_follow_servo_ids_in_config_v1(model):
    fw = firmware_joints()
    assert [n for n, _, _, _ in fw] == SERVO_ORDER == cv.ACTUATOR_ORDER
    assert [i for _, i, _, _ in fw] == list(range(1, 21))
    assert [model.actuator(i).name for i in range(model.nu)] == SERVO_ORDER
    for i, name in enumerate(SERVO_ORDER):  # each servo pulls its own joint's flex strand
        assert model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_TENDON
        assert model.tendon(model.actuator_trnid[i, 0]).name == f"{name}_flex"


def test_limits_match_config_v1(model):
    for name, _, lo, hi in firmware_joints():
        rng = np.degrees(model.jnt_range[model.joint(name).id])
        assert np.allclose(rng, [lo, hi], atol=1e-3), (name, rng, lo, hi)
        ctrl = np.degrees(model.actuator_ctrlrange[model.actuator(name).id])
        assert np.allclose(ctrl, [lo, hi], atol=1e-3), (name, ctrl)  # 1:1 drive


def test_mass_is_plausible_for_pla(model):
    """Solid PLA (100% infill). The moving fingers + thumb matter for the dynamics; the palm,
    forearm, servos and spools are welded to the world, so their mass has no effect."""
    names = [model.body(i).name for i in range(model.nbody)]
    moving_g = (
        sum(model.body_mass[i] for i, n in enumerate(names) if model.body_jntnum[i] > 0) * 1000
    )
    palm_g = model.body_mass[model.body("palm").id] * 1000
    assert 70 < moving_g < 150, moving_g  # 98 g at the 2026-09-28 export
    assert 100 < palm_g < 260, palm_g  # 188 g solid; printed with ~30% infill it is far lighter


def test_no_contacts_when_straight(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.ncon == 0, contact_report(model, data)


# ----- Sign convention ----------------------------------------------------------------------


@pytest.mark.parametrize("joint", SERVO_ORDER)
def test_positive_closes(model, joint):
    data = mujoco.MjData(model)
    tip = f"{joint.split('_')[0]}_tip"
    mujoco.mj_kinematics(model, data)
    start = data.site(tip).xpos.copy()
    data.qpos[qadr(model, joint)] = np.radians(10)
    mujoco.mj_kinematics(model, data)
    moved = data.site(tip).xpos - start
    assert np.linalg.norm(moved) > 1e-3
    assert np.dot(moved / np.linalg.norm(moved), EXPECTED_TIP_MOTION[joint]) > 0.5, (joint, moved)


# ----- Tendon routing -----------------------------------------------------------------------


def test_no_coupling(model):
    """Each strand's length depends only on its own joint, at any pose."""
    data = mujoco.MjData(model)
    own = np.array(
        [model.joint(model.tendon(t).name.rsplit("_", 1)[0]).id for t in range(model.ntendon)]
    )
    own_col = model.jnt_qposadr[own]
    worst, where = 0.0, None
    for q in random_poses(model, 30):
        jac = length_jacobian(model, data, q) * 1000  # mm/rad
        jac[np.arange(model.ntendon), own_col] = 0
        t, k = np.unravel_index(np.abs(jac).argmax(), jac.shape)
        if abs(jac[t, k]) > worst:
            worst, where = abs(jac[t, k]), (model.tendon(t).name, k)
    print(f"\nworst coupling: {worst:.2e} mm/rad ({where})")
    assert worst < COUPLING_LIMIT_MM_PER_RAD, where


@pytest.mark.parametrize("joint", SERVO_ORDER)
def test_own_moment_arm_and_tight_loop(model, joint):
    """Over the full joint range: dL_flex/dq = -6 mm/rad, dL_ext/dq = +6 mm/rad, L_flex + L_ext const."""
    data = mujoco.MjData(model)
    flex, ext = model.tendon(f"{joint}_flex").id, model.tendon(f"{joint}_ext").id
    k = qadr(model, joint)
    lo, hi = model.jnt_range[model.joint(joint).id]
    arms, loop = [], []
    for base in random_poses(model, 3, seed=1):
        for qj in np.linspace(lo, hi, 25):
            q = base.copy()
            q[k] = qj
            jac = length_jacobian(model, data, q)[:, k] * 1000
            arms.append((jac[flex], jac[ext]))
            lengths = tendon_lengths(model, data, q) * 1000
            loop.append(lengths[flex] + lengths[ext])
    arms = np.array(arms)
    spread = max(loop) - min(loop)
    print(
        f"\n{joint}: flex arm {arms[:, 0].min():+.4f}..{arms[:, 0].max():+.4f}, "
        f"ext arm {arms[:, 1].min():+.4f}..{arms[:, 1].max():+.4f} mm/rad, loop spread {spread:.2e} mm"
    )
    assert np.abs(arms[:, 0] + drum_mm(joint)).max() < MOMENT_ARM_TOL_MM_PER_RAD
    assert np.abs(arms[:, 1] - drum_mm(joint)).max() < MOMENT_ARM_TOL_MM_PER_RAD
    assert spread < LOOP_TOL_MM


def test_palm_path_does_not_move(model):
    """Strand points in the palm/forearm (fixed parts) stay put whatever the joints do."""
    data = mujoco.MjData(model)
    fixed = [i for i in range(model.nsite) if model.body_weldid[model.site_bodyid[i]] == 0]
    assert fixed, "expected strand start/route sites in the fixed parts"
    positions = []
    for q in random_poses(model, 5):
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        positions.append(data.site_xpos[fixed].copy())
    assert np.ptp(np.array(positions), axis=0).max() < 1e-12


@pytest.mark.skipif(not cv.ROUTES_PATH.exists(), reason="no tendon_routes.json yet")
def test_model_uses_tendon_routes_json(model):
    """Each strand's proximal end is its spool tangent point from tendon_routes.json."""
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    for strand in json.loads(cv.ROUTES_PATH.read_text(encoding="utf-8"))["strands"]:
        site = model.site(model.wrap_objid[model.tendon_adr[model.tendon(strand["name"]).id]])
        assert np.allclose(data.site_xpos[site.id] * 1000, strand["points_mm"][-1], atol=1e-3), (
            strand["name"]
        )


def test_routes_file_is_followed(tmp_path):
    """With tendon_routes.json, strands run through its points; the routing rule still holds."""
    hand = cv.load_hand()
    strands = []
    for j in cv.ACTUATOR_ORDER:
        for side in ("flex", "ext"):
            start = cv.build_strand(hand, j, side, None)[0].pos * 1000  # finger-base start point
            spool = [start[0], start[1] + 5, -150.0]
            strands.append(
                {
                    "name": f"{j}_{side}",
                    "joint": j,
                    "side": side,
                    "points_mm": [start.tolist(), [start[0], start[1] + 5, -60.0], spool],
                    "servo_id": cv.ACTUATOR_ORDER.index(j) + 1,
                    "spool_center_mm": [spool[0], spool[1] + 6, spool[2]],
                    "spool_axis": [1, 0, 0],
                }
            )
    routes = tmp_path / "tendon_routes.json"
    routes.write_text(json.dumps({"spool_radius_mm": 6, "strands": strands}))
    mjcf = cv.build(routes_path=routes, meshdir=cv.EXPORT_DIR.as_posix())
    model = mujoco.MjModel.from_xml_string(ET.tostring(mjcf, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    last = data.site("index_dip_flex_0_route").xpos * 1000
    assert np.allclose(last, strands[0]["points_mm"][-1], atol=1e-3)  # proximal end = spool point
    for q in random_poses(model, 3):
        jac = length_jacobian(model, data, q) * 1000
        for t in range(model.ntendon):
            joint = model.tendon(t).name.rsplit("_", 1)[0]
            own = qadr(model, joint)
            sign = -1 if model.tendon(t).name.endswith("_flex") else 1
            assert abs(jac[t, own] - sign * drum_mm(joint)) < MOMENT_ARM_TOL_MM_PER_RAD
            assert np.abs(np.delete(jac[t], own)).max() < COUPLING_LIMIT_MM_PER_RAD


# ----- Actuation ----------------------------------------------------------------------------

# Finger abduction is tested in a fan pose: toward a straight neighbour a finger only has ~4-5 deg
# before it touches (8 mm gap between fingers), a real limit of the geometry.
FAN_DEG = {"index_mcp_abd": 12, "middle_mcp_abd": 4, "ring_mcp_abd": -4, "little_mcp_abd": -15}


def _track(model, targets_deg: dict[str, float]):
    data = mujoco.MjData(model)
    ctrl = np.array([np.radians(targets_deg.get(n, 0.0)) for n in SERVO_ORDER])
    settle(model, data, ctrl, 1000)  # 2 s, gravity on
    err = np.degrees(actuated_q(model, data) - ctrl)
    return data, err


@pytest.mark.parametrize("joint", [j for j in SERVO_ORDER if not j.endswith("mcp_abd")])
@pytest.mark.parametrize("fraction", [0.6, -0.6])
def test_servo_angle_drives_its_joint(model, joint, fraction):
    lo, hi = np.degrees(model.jnt_range[model.joint(joint).id])
    target = fraction * (hi if fraction > 0 else -lo)
    data, err = _track(model, {joint: target})
    i = SERVO_ORDER.index(joint)
    assert data.ncon == 0, contact_report(model, data)
    assert abs(err[i]) < TRACKING_TOL_DEG, (joint, target, err[i])
    assert np.abs(np.delete(err, i)).max() < 1.0  # the other joints hold still


def test_finger_abduction_fan(model):
    data, err = _track(model, FAN_DEG)
    print(f"\nfan pose worst tracking error {np.abs(err).max():.2f} deg")
    assert data.ncon == 0, contact_report(model, data)
    assert np.abs(err).max() < TRACKING_TOL_DEG


def test_fist_then_open_settle(model):
    data = mujoco.MjData(model)
    fist = model.key("fist").ctrl
    settle(model, data, fist, 1500)
    err = np.degrees(actuated_q(model, data) - fist)
    contacts = contact_report(model, data)
    print(
        f"\nfist: worst tracking error {np.abs(err).max():.2f} deg, "
        f"max |qvel| {np.abs(data.qvel).max():.2e} rad/s, contacts: {contacts or 'none'}"
    )
    assert np.isfinite(data.qpos).all()
    assert np.abs(data.qvel).max() < 0.05
    assert np.abs(err).max() < 3.0
    settle(model, data, np.zeros(model.nu), 1500)
    err = np.degrees(actuated_q(model, data))
    print(
        f"open: worst error {np.abs(err).max():.2f} deg, contacts: {contact_report(model, data) or 'none'}"
    )
    assert np.abs(data.qvel).max() < 0.05
    assert np.abs(err).max() < 1.0
    assert data.ncon == 0


def test_actuator_drives_only_its_own_joint(model):
    """ctrl is the joint angle: the moment of each actuator on its own joint is 1, on the others
    0 (also for thumb_cmc_rot, whose larger drum only changes the servo gain)."""
    data = mujoco.MjData(model)
    for q in random_poses(model, 3, seed=2):
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        moment = np.zeros((model.nu, model.nv))
        mujoco.mju_sparse2dense(
            moment, data.actuator_moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind
        )
        expected = np.zeros_like(moment)
        for i, n in enumerate(SERVO_ORDER):
            expected[i, model.jnt_dofadr[model.joint(n).id]] = 1.0
        assert np.abs(moment - expected).max() < 1e-3


def test_servo_ratio_matches_the_firmware():
    """servo_per_joint in firmware/include/config_v1.h = drum radius / spool radius."""
    import re

    cfg = (Path(__file__).resolve().parents[2] / "firmware" / "include" / "config_v1.h").read_text()
    for name, ratio in re.findall(r'\{"(\w+)",\s*\d+,[^}]*?,\s*([\d.]+)f,\s*-?\d+\}', cfg):
        assert float(ratio) == pytest.approx(cv.servo_per_joint(name)), name


THUMB = ["thumb_cmc_rot", "thumb_cmc_flex", "thumb_mcp_flex", "thumb_ip"]


def test_thumb_tendons_keep_their_length_at_every_thumb_angle(model):
    """Owner's requirement (2026-09-29): every thumb tendon's length changes only with its own
    joint, and each loop (flex + ext) keeps the same total length, at every thumb pose. Checked on
    a grid of all 4 thumb joints (5 angles each over the full ranges, 625 poses)."""
    data = mujoco.MjData(model)
    grids = [np.linspace(*model.jnt_range[model.joint(j).id], 5) for j in THUMB]
    k = [qadr(model, j) for j in THUMB]
    worst_coupling = worst_loop = 0.0
    ref = {}
    for combo in np.array(np.meshgrid(*grids, indexing="ij")).reshape(4, -1).T:
        q = np.zeros(model.nq)
        q[k] = combo
        lengths = tendon_lengths(model, data, q) * 1000
        for i, joint in enumerate(THUMB):
            f, e = model.tendon(f"{joint}_flex").id, model.tendon(f"{joint}_ext").id
            # the same joint angle alone, all other thumb joints at 0
            q_own = np.zeros(model.nq)
            q_own[k[i]] = combo[i]
            key = (joint, round(float(combo[i]), 6))
            if key not in ref:
                ref[key] = tendon_lengths(model, data, q_own)[[f, e]] * 1000
            worst_coupling = max(worst_coupling, np.abs(lengths[[f, e]] - ref[key]).max())
            loop0 = ref[(joint, round(float(combo[i]), 6))].sum()
            worst_loop = max(worst_loop, abs(lengths[f] + lengths[e] - loop0))
    print(
        f"\nthumb: worst length change from other joints {worst_coupling:.2e} mm, loop {worst_loop:.2e} mm"
    )
    assert worst_coupling < 0.05
    assert worst_loop < 0.05
