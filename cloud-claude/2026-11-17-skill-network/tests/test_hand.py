"""Tests for the V2.1 hand model (model/models/v21_hand.xml) against the contract and the CAD.

Fast by default (no CAD tools needed: CAD reference transforms are cached in model/assets/v21/fk_reference.json).
Markers: ``cad`` (needs the cad group: live CAD FK, extract --check), ``warp`` (MuJoCo Warp on CPU).
"""

from __future__ import annotations

import json
import math
import subprocess
import sys

import mujoco
import numpy as np
import pytest
from hand_helpers import (
    ASSETS,
    FINGERS,
    POSES_JSON,
    ROOT,
    SERVO_JOINTS,
    WRIST_JOINTS,
    build_hand_module,
    fk_reference,
    kin,
    load_model,
    poly,
    pose_errors,
    set_cad_q,
)

TOL_MM, TOL_DEG = 0.1, 0.1
CONTRACT_BODIES = (
    ["forearm", "wrist_ring", "palm"]
    + [f"{f}_{b}" for f in FINGERS for b in ("yoke", "proximal", "middle", "distal", "bar")]
    + [f"thumb_{b}" for b in ("base", "metacarpal", "proximal", "distal")]
)


@pytest.fixture(scope="module")
def model() -> mujoco.MjModel:
    return load_model()


def _act():
    return build_hand_module().load_actuators()[0]


# ------------------------------------------------------------------------------------------- contract structure
def test_names_order_limits(model):
    names = {model.body(i).name for i in range(model.nbody)} - {"world"}
    assert names == set(CONTRACT_BODIES)
    assert model.body("forearm").parentid[0] == 0 and model.body_jntnum[model.body("forearm").id] == 0
    joints = {model.joint(i).name for i in range(model.njnt)}
    assert joints == set(SERVO_JOINTS) | set(WRIST_JOINTS) | {f"{f}_dip" for f in FINGERS} | {
        f"{f}_bar" for f in FINGERS
    }
    assert all(
        model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE and model.jnt_limited[i] for i in range(model.njnt)
    )
    acts = [model.actuator(i).name for i in range(model.nu)]
    assert acts == [*SERVO_JOINTS, "wrist_flex", "wrist_dev"]
    act = _act()
    for i, a in enumerate(acts):
        jid = model.actuator_trnid[i, 0]
        assert model.joint(jid).name == a
        if a in SERVO_JOINTS:  # servo setpoint range = joint range +- tau_cap/kp, clipped to the shaft travel
            exp, _ = build_hand_module().servo_ctrlrange(act[a], list(model.jnt_range[jid]))
            np.testing.assert_allclose(model.actuator_ctrlrange[i], exp, atol=1e-8)
            assert model.actuator_ctrlrange[i, 0] < model.jnt_range[jid, 0] - 0.25
            assert model.actuator_ctrlrange[i, 1] > model.jnt_range[jid, 1] + 0.25
        else:
            np.testing.assert_allclose(model.actuator_ctrlrange[i], model.jnt_range[jid])
        np.testing.assert_allclose(model.actuator_gainprm[i, 0], act[a]["kp"], rtol=1e-6)
        np.testing.assert_allclose(model.actuator_forcerange[i], act[a]["forcerange"], rtol=1e-6)
        np.testing.assert_allclose(model.dof_damping[model.jnt_dofadr[jid]], act[a]["damping"], rtol=1e-6)
        np.testing.assert_allclose(model.dof_armature[model.jnt_dofadr[jid]], act[a]["armature"], rtol=1e-6)
    for d in kin()["bodies"].values():
        if d["joint"]:
            np.testing.assert_allclose(model.jnt_range[model.joint(d["joint"]).id], d["range_rad"], atol=1e-9)
    for s in ("wrist_centre", "forearm_flange", "palm_centre", "thumb_tip", *(f"{f}_tip" for f in FINGERS)):
        model.site(s)


def test_cad_ranges(model):
    """Servo/wrist ranges equal the CAD's (deg): fingers 16_finger RANGE + F.ABD_RANGE, thumb T.RANGES, wrist 09."""
    exp = {
        "wrist_flex": (-50, 50),
        "wrist_dev": (-30, 20),
        "thumb_cmc_rot": (-31, 110),
        "thumb_cmc_flex": (-37, 63),
        "thumb_mcp_flex": (-10, 70),
        "thumb_ip": (-15, 85),
        "index_mcp_abd": (-10, 15),
        "middle_mcp_abd": (-15, 15),
        "ring_mcp_abd": (-15, 15),
        "little_mcp_abd": (-15, 7.5),
    }
    exp.update({f"{f}_mcp_flex": (-10, 90) for f in FINGERS})
    exp.update({f"{f}_pip": (-5, 95) for f in FINGERS})
    for j, r in exp.items():
        np.testing.assert_allclose(np.degrees(model.jnt_range[model.joint(j).id]), r, atol=1e-6)


def test_signs(model):
    """+ = closing: flexion moves the tip to the palm side (-Y), abduction toward the thumb (+X)."""
    d = mujoco.MjData(model)

    def tip(site, j, a):
        set_cad_q(model, d, {j: a})
        mujoco.mj_kinematics(model, d)
        return d.site(site).xpos.copy()

    for f in FINGERS:
        for j in (f"{f}_mcp_flex", f"{f}_pip"):
            assert tip(f"{f}_tip", j, 20)[1] < tip(f"{f}_tip", j, 0)[1] - 1e-3
        assert tip(f"{f}_tip", f"{f}_mcp_abd", 10)[0] > tip(f"{f}_tip", f"{f}_mcp_abd", 0)[0] + 1e-3

    # thumb (its q = 0 is not an anatomical rest pose, so test from the CAD poses): flexion (+) brings the thumb
    # tip closer to the palm centre, opposition (+) toward the palm and the middle fingertip
    def dist(q, site):
        set_cad_q(model, d, q)
        mujoco.mj_kinematics(model, d)
        return float(np.linalg.norm(d.site("thumb_tip").xpos - d.site(site).xpos))

    for pose in ("open", "relaxed", "tip_pinch"):
        q = fk_reference()["poses"][pose]["q"]
        for j in ("thumb_cmc_flex", "thumb_mcp_flex", "thumb_ip", "thumb_cmc_rot"):
            assert dist({**q, j: q[j] + 10}, "palm_centre") < dist(q, "palm_centre"), (pose, j)
    q = fk_reference()["poses"]["open"]["q"]
    assert dist({**q, "thumb_cmc_rot": q["thumb_cmc_rot"] + 10}, "middle_tip") < dist(q, "middle_tip")
    assert tip("middle_tip", "wrist_flex", 20)[1] < tip("middle_tip", "wrist_flex", 0)[1] - 1e-3
    assert tip("middle_tip", "wrist_dev", 10)[0] > tip("middle_tip", "wrist_dev", 0)[0] + 1e-3
    # couplings close with the PIP
    for f in FINGERS:
        assert poly(kin()["couplings"][f"{f}_dip"]["poly"], 1.0) > 0.5


# ------------------------------------------------------------------------------------------- kinematic fidelity
def _check_configs(model, configs) -> tuple[float, float, str]:
    d = mujoco.MjData(model)
    worst_p, worst_r, wp, wr = 0.0, 0.0, "", ""
    for name, q, T in configs:
        set_cad_q(model, d, q)
        for b, (ep, er) in pose_errors(model, d, T).items():
            if ep > worst_p:
                worst_p, wp = ep, f"{name}:{b}"
            if er > worst_r:
                worst_r, wr = er, f"{name}:{b}"
    return worst_p, worst_r, f"mm at {wp}, deg at {wr}"


def test_fidelity_cad_poses(model):
    """All 9 CAD poses: every body within 0.1 mm / 0.1 deg of the CAD (poses.json T of its parts)."""
    ref = fk_reference()["poses"]
    if POSES_JSON.exists():  # the cache must equal the CAD's own per-part transforms
        pj = json.loads(POSES_JSON.read_text())
        kb = kin()["bodies"]
        part_body = {}
        for n, cb in pj["parts"].items():
            g = cb.split(":")
            part_body[n] = {"forearm": "forearm", "ring": "wrist_ring", "palm": "palm"}.get(cb) or (
                "palm"
                if g[1] in ("mount",) or g[1].endswith("_module")
                else f"thumb_{g[1]}"
                if g[0] == "thumb"
                else g[1]
            )
        for pn, p in pj["poses"].items():
            for n, b in part_body.items():
                if b == "forearm" or n.startswith(("wrist_crank", "wrist_rod")):
                    continue
                T = np.array(p["T"][n]).reshape(4, 4)[:3]
                assert np.abs(T - np.array(ref[pn]["T"][b]).reshape(3, 4)).max() < 1e-4, (pn, n)
        assert set(kb) == set(ref["open"]["T"])
    wp, wr, where = _check_configs(model, [(n, p["q"], p["T"]) for n, p in ref.items()])
    print(f"\n9 CAD poses: max body error {wp:.4f} mm, {wr:.4f} deg ({where})")
    assert wp <= TOL_MM and wr <= TOL_DEG


def test_fidelity_random(model):
    """100 random in-range configurations (cached CAD FK): within 0.1 mm / 0.1 deg."""
    ref = fk_reference()["random"]
    assert len(ref) >= 100
    wp, wr, where = _check_configs(model, [(f"rand{i}", r["q"], r["T"]) for i, r in enumerate(ref)])
    print(f"\n{len(ref)} random configs: max body error {wp:.4f} mm, {wr:.4f} deg ({where})")
    assert wp <= TOL_MM and wr <= TOL_DEG


@pytest.mark.cad
def test_fidelity_live_cad_fk(model):
    """Fresh random configurations through the live CAD FK (needs the cad group)."""
    pytest.importorskip("cadquery")
    sys.path.insert(0, str(ROOT / "model"))
    import extract_v21 as X

    fk = X.CadFK()
    cfgs = []
    for i, q in enumerate(X.random_configs(fk.ranges_deg(), 50, seed=99)):
        Tb = fk.body_T(q)
        cfgs.append((f"live{i}", q, {b: Tb[b][:3].ravel().tolist() for b in CONTRACT_BODIES}))
    wp, wr, where = _check_configs(model, cfgs)
    print(f"\n50 live CAD configs: max body error {wp:.4f} mm, {wr:.4f} deg ({where})")
    assert wp <= TOL_MM and wr <= TOL_DEG


# ------------------------------------------------------------------------------------------- mass, meshes
def test_mass(model):
    total = float(model.body_mass.sum())
    assert abs(total - 1.1129) < 0.0005, total
    rep = json.loads((ASSETS / "report.json").read_text())
    assert abs(rep["mass"]["total_g"] - rep["mass"]["checks_json_total_g"]) < 0.5
    assert all(model.body_mass[i] > 0 for i in range(1, model.nbody))


def test_collision_meshes_convex():
    trimesh = pytest.importorskip("trimesh")
    ConvexHull = pytest.importorskip("scipy.spatial").ConvexHull

    n = 0
    for b, d in kin()["bodies"].items():
        for c in d["collision_meshes"]:
            m = trimesh.load(ASSETS / c, force="mesh")
            assert len(m.vertices) <= 64, c
            assert m.is_watertight, c
            assert abs(m.volume - ConvexHull(m.vertices).volume) < 1e-6 * max(1.0, m.volume) + 1e-12, c
            n += 1
        if b.endswith("_bar"):
            assert d["collision_meshes"] == []
    assert n > 20


def test_geoms_and_bar(model):
    """Bars never collide; pads are class v21_pad; visual geoms never collide."""
    for g in range(model.ngeom):
        b = model.body(model.geom_bodyid[g]).name
        if b.endswith("_bar") or model.geom(g).name.endswith("_vis"):
            assert model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0
    pads = [model.geom(g).name for g in range(model.ngeom) if model.geom_condim[g] == 4]
    assert {"index_distal", "thumb_distal", "palm"} <= {p.rsplit("_c", 1)[0] for p in pads}


def test_sensors_and_qpos_index(model):
    """jointpos + actuatorfrc sensors in actuator order; actuator_qpos_index maps actuators to qpos."""
    names = [model.actuator(i).name for i in range(model.nu)]
    assert [model.sensor(i).name for i in range(model.nsensor)] == [f"{n}_pos" for n in names] + [
        f"{n}_frc" for n in names
    ]
    idx = build_hand_module().actuator_qpos_index(model)
    d = mujoco.MjData(model)
    rng = np.random.default_rng(1)
    d.qpos[:] = rng.uniform(-0.5, 0.5, model.nq)
    d.ctrl[:] = rng.uniform(-0.3, 0.3, model.nu)
    mujoco.mj_forward(model, d)
    np.testing.assert_allclose(d.sensordata[: model.nu], d.qpos[idx])
    np.testing.assert_allclose(d.sensordata[model.nu :], d.actuator_force)
    for i, a in enumerate(idx):
        assert model.jnt_qposadr[model.actuator_trnid[i, 0]] == a


def test_no_gravcomp(model):
    assert np.all(model.body_gravcomp == 0)


def test_collision_bits(model):
    """Only meaningful hand self-contacts are possible; every collision geom touches the world (bit 0)."""

    def can(b1, b2):
        g1 = [
            g
            for g in range(model.ngeom)
            if model.geom_bodyid[g] == model.body(b1).id and model.geom_group[g] == 3
        ]
        g2 = [
            g
            for g in range(model.ngeom)
            if model.geom_bodyid[g] == model.body(b2).id and model.geom_group[g] == 3
        ]
        return any(
            (model.geom_contype[a] & model.geom_conaffinity[b])
            or (model.geom_contype[b] & model.geom_conaffinity[a])
            for a in g1
            for b in g2
        )

    for g in range(model.ngeom):
        if model.geom_group[g] == 3:
            assert model.geom_contype[g] & 1 and not model.geom_conaffinity[g] & 1
    for f in FINGERS:
        for seg in ("proximal", "middle", "distal"):
            assert can("thumb_distal", f"{f}_{seg}") and can("thumb_proximal", f"{f}_{seg}")
        assert (
            can(f"{f}_distal", "palm") and not can(f"{f}_proximal", "palm") and not can(f"{f}_yoke", "palm")
        )
    for a, b in (("index", "middle"), ("middle", "ring"), ("ring", "little")):
        assert can(f"{a}_distal", f"{b}_distal") and can(f"{a}_middle", f"{b}_middle")
    assert not can("index_distal", "ring_distal") and not can("index_distal", "little_middle")
    assert (
        not can("palm", "thumb_metacarpal") and not can("forearm", "palm") and not can("palm", "thumb_base")
    )
    assert can("thumb_distal", "palm")


# ------------------------------------------------------------------------------------------- self contact
def _self_contacts(m, d) -> list[tuple[str, str, float]]:
    out = []
    for c in d.contact[: d.ncon]:
        b1, b2 = m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name
        out.append((b1, b2, float(c.dist)))
    return out


def test_no_self_contact_at_rest(model):
    """Open hand (q = 0) and the CAD 'open' / 'spread' / wrist poses: no hand self-contact at all."""
    d = mujoco.MjData(model)
    for name, p in [("q0", {"q": {}})] + [
        (n, fk_reference()["poses"][n]) for n in ("open", "spread", "wrist_flex_+40", "wrist_dev_-20")
    ]:
        set_cad_q(model, d, p["q"])
        mujoco.mj_forward(model, d)
        assert _self_contacts(model, d) == [], name


# Contacts that are in the CAD itself: the designed pinches (checks.json interference: tip_pinch
# index_tip_skin/tip_skin 3.0 mm^3, thumb_to_little little_tip_skin/tip_skin 7.1 mm^3), and a CAD problem found
# here: in the 'fist' pose the little fingertip skin is 1.77 mm inside the little knuckle module (trimesh signed
# distance on the CAD meshes; the hull gives 2.27 mm). The CAD's interference check does not report it.
CAD_CONTACTS = {
    "tip_pinch": {("index_distal", "thumb_distal")},
    "thumb_to_little": {("thumb_distal", "little_distal")},
    "fist": {("palm", "little_distal")},
}


def test_self_contact_cad_poses(model):
    """The 9 CAD poses: no self-contact except the ones the CAD has (pinches, little tip in the fist), < 2.5 mm."""
    d = mujoco.MjData(model)
    for name, p in fk_reference()["poses"].items():
        set_cad_q(model, d, p["q"])
        mujoco.mj_forward(model, d)
        allowed = CAD_CONTACTS.get(name, set())
        for b1, b2, dist in _self_contacts(model, d):
            assert (b1, b2) in allowed or (b2, b1) in allowed, (name, b1, b2, dist)
            assert dist > -0.0025, (name, b1, b2, dist)
        for pair in allowed:  # and the pinch really touches
            assert any({b1, b2} == set(pair) for b1, b2, _ in _self_contacts(model, d)), (name, pair)


# ------------------------------------------------------------------------------------------- physics
def _sim(model, ctrl: dict[str, float], t: float, record=None):
    d = mujoco.MjData(model)
    for i in range(model.nu):
        d.ctrl[i] = ctrl.get(model.actuator(i).name, 0.0)
    rec = []
    for _ in range(round(t / model.opt.timestep)):
        mujoco.mj_step(model, d)
        if record:
            rec.append(record(d))
    return d, rec


def test_hold_q0_gravity(model):
    """2 s under gravity, every actuator holding 0: finite and no joint drifts > 2 deg."""
    d, _ = _sim(model, {}, 2.0)
    assert np.all(np.isfinite(d.qpos))
    worst = {model.joint(j).name: math.degrees(abs(d.qpos[model.jnt_qposadr[j]])) for j in range(model.njnt)}
    assert max(worst.values()) < 2.0, sorted(worst.items(), key=lambda x: -x[1])[:3]


def test_step_mid_range(model):
    """Each actuator alone commanded to mid-range reaches it within 2 deg in 1 s; DIP follows within 1 deg."""
    worst_dip = 0.0
    for i in range(model.nu):
        a = model.actuator(i).name
        lo, hi = model.actuator_ctrlrange[i]
        target = 0.5 * (lo + hi)
        adr = model.jnt_qposadr[model.actuator_trnid[i, 0]]
        rec_fn = None
        if a.endswith("_pip") and not a.startswith("thumb"):
            f = a[: -len("_pip")]
            c = kin()["couplings"][f"{f}_dip"]["poly"]
            dadr = model.jnt_qposadr[model.joint(f"{f}_dip").id]

            def rec_fn(d, c=c, dadr=dadr, adr=adr):
                return abs(d.qpos[dadr] - poly(c, d.qpos[adr]))

        d, rec = _sim(model, {a: target}, 1.0, rec_fn)
        err = math.degrees(abs(d.qpos[adr] - target))
        assert err < 2.0, (a, err)
        if rec:
            worst_dip = max(worst_dip, math.degrees(max(rec)))
    print(f"\nDIP coupling max error in motion {worst_dip:.3f} deg")
    assert worst_dip < 1.0


def test_coupling_under_tip_load(model):
    """DIP and bar follow the PIP within 1.5 deg with 10-15 N pushing the fingertip pad open (review M3)."""
    c = kin()["couplings"]
    idx = build_hand_module().actuator_qpos_index(model)
    worst = 0.0
    for f in FINGERS:
        for force in (10.0, 15.0):
            d = mujoco.MjData(model)
            set_cad_q(model, d, {f"{f}_mcp_flex": 30, f"{f}_pip": 30})
            d.ctrl[:] = d.qpos[idx]
            mujoco.mj_forward(model, d)
            b, s = model.body(f"{f}_distal").id, model.site(f"{f}_tip").id
            for k in range(500):
                n = d.site_xmat[s].reshape(3, 3)[:, 2]  # pad normal: push the pad back (opening)
                fv = -n * force * min(1.0, k / 200)
                d.xfrc_applied[b, :3] = fv
                d.xfrc_applied[b, 3:] = np.cross(d.site_xpos[s] - d.xipos[b], fv)
                mujoco.mj_step(model, d)
            pip = d.qpos[model.joint(f"{f}_pip").qposadr[0]]
            for j in (f"{f}_dip", f"{f}_bar"):
                err = math.degrees(abs(d.qpos[model.joint(j).qposadr[0]] - poly(c[j]["poly"], pip)))
                worst = max(worst, err)
    print(f"\ncoupling error under 10-15 N tip load: {worst:.3f} deg")
    assert worst <= 1.5


def test_blocked_finger_reaches_torque_cap():
    """An object blocks the index fingertip at PIP ~80-85 deg: the setpoint can still go far enough beyond the
    joint to reach the servo's force cap (review M1: ctrlrange = joint range + tau_cap/kp).

    Not tested at the PIP limit itself: there the assumed 360 deg servo travel clips the setpoint to 2.17 rad, so the
    torque is limited to kp x (2.17 - 1.66) = 0.49 N m (85 % of the cap); see report.
    """
    from hand_helpers import XML

    m0 = mujoco.MjModel.from_xml_path(str(XML))
    d0 = mujoco.MjData(m0)
    set_cad_q(m0, d0, {"index_pip": 80})  # where the object stops the fingertip (PIP limit 95)
    mujoco.mj_kinematics(m0, d0)
    j = m0.joint("index_pip").id
    v = np.cross(d0.xaxis[j], d0.site("index_tip").xpos - d0.xanchor[j])
    n = v / np.linalg.norm(v)  # where the tip moves when the PIP closes
    spec = mujoco.MjSpec.from_file(str(XML))
    spec.worldbody.add_geom(
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.006, 0, 0],
        pos=d0.site("index_tip").xpos + n * 0.0065,
        contype=1,
        conaffinity=1,
    )
    m = spec.compile()
    build_hand_module().apply_physics(m)
    m.opt.gravity[:] = 0
    d = mujoco.MjData(m)
    set_cad_q(m, d, {"index_pip": 72})
    a = m.actuator("index_pip").id
    d.ctrl[a] = m.actuator_ctrlrange[a, 1]
    for _ in range(500):
        mujoco.mj_step(m, d)
    pip = math.degrees(d.qpos[m.joint("index_pip").qposadr[0]])
    cap = m.actuator_forcerange[a, 1]
    print(f"\nblocked index: pip {pip:.1f} deg, actuator force {d.actuator_force[a]:.3f} / cap {cap:.3f} N m")
    assert pip < 88.0 and d.actuator_force[a] >= 0.999 * cap


def test_bar_never_collides(model):
    """Fist motion: no contact involves a bar body."""
    targets = {j: model.actuator_ctrlrange[i, 1] * 0.9 for i, j in enumerate(SERVO_JOINTS)}
    bars = {model.body(f"{f}_bar").id for f in FINGERS}

    def rec(d):
        return any(
            model.geom_bodyid[c.geom1] in bars or model.geom_bodyid[c.geom2] in bars
            for c in d.contact[: d.ncon]
        )

    d, r = _sim(model, targets, 0.6, rec)
    assert np.all(np.isfinite(d.qpos)) and not any(r)


# ------------------------------------------------------------------------------------------- warp, builders
@pytest.mark.warp
def test_warp_put_model_and_step(model):
    """MuJoCo Warp (CPU) vs MuJoCo C, same physics options: 250 steps from the tip_pinch pose (thumb-index contact
    active, squeezed), and the hand-self broadphase pair count (review M4)."""
    wp = pytest.importorskip("warp")
    mjw = pytest.importorskip("mujoco_warp")
    idx = build_hand_module().actuator_qpos_index(model)
    d = mujoco.MjData(model)
    set_cad_q(model, d, fk_reference()["poses"]["tip_pinch"]["q"])
    d.ctrl[:] = d.qpos[idx]
    for j in ("index_pip", "thumb_ip"):  # squeeze the pinch: thumb-index contact stays active every step
        d.ctrl[model.actuator(j).id] += 0.15
    mujoco.mj_forward(model, d)
    q0 = d.qpos.copy()
    with wp.ScopedDevice("cpu"):
        mw = mjw.put_model(model)
        npairs = mw.nxn_geom_pair_filtered.shape[0]
        dw = mjw.put_data(model, d, nconmax=256, njmax=1024)
        for _ in range(250):
            mjw.step(mw, dw)
        q = dw.qpos.numpy()[0]
    ncon = []
    for _ in range(250):
        mujoco.mj_step(model, d)
        ncon.append(d.ncon)
    err = np.abs(q - d.qpos).max()
    print(
        f"\nwarp vs C after 250 steps: max |dq| {err:.2e} rad (motion {np.abs(d.qpos - q0).max():.3f} rad, "
        f"C contacts {min(ncon)}..{max(ncon)}); hand-self broadphase pairs {npairs}"
    )
    assert np.all(np.isfinite(q)) and min(ncon) > 0
    assert err < 2e-3
    assert npairs <= 240


def test_attach_with_prefix(model, tmp_path):
    """Attachable: MjSpec.attach with a prefix into a degree-angle parent keeps radian ranges and finds the meshes."""
    from hand_helpers import XML

    child = mujoco.MjSpec.from_file(str(XML))
    parent = mujoco.MjSpec.from_string(
        '<mujoco><worldbody><body name="arm" pos="0 0 1"><site name="flange"/></body></worldbody></mujoco>'
    )
    parent.attach(child, site=parent.site("flange"), prefix="right_")
    m = parent.compile()
    assert m.nu == model.nu and m.actuator(0).name == "right_index_pip"
    for j in ("index_pip", "thumb_cmc_rot", "wrist_flex"):
        np.testing.assert_allclose(m.joint(f"right_{j}").range, model.joint(j).range)
    text = XML.read_text()
    assert "<option" not in text and "<size" not in text and "<visual" not in text


def test_build_hand_check():
    r = subprocess.run(
        [sys.executable, str(ROOT / "model" / "build_hand.py"), "--check"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert r.returncode == 0, r.stdout + r.stderr


@pytest.mark.cad
def test_extract_check():
    """extract_v21.py --check (slow: re-extracts everything, ~1-2 min; needs the cad group)."""
    pytest.importorskip("cadquery")
    r = subprocess.run(
        [sys.executable, str(ROOT / "model" / "extract_v21.py"), "--check"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert r.returncode == 0, r.stdout + r.stderr
