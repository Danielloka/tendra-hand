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
    ASSETS, FINGERS, POSES_JSON, ROOT, SERVO_JOINTS, WRIST_JOINTS, fk_reference, kin, load_model, poly, pose_errors,
    set_cad_q,
)

TOL_MM, TOL_DEG = 0.1, 0.1
CONTRACT_BODIES = (["forearm", "wrist_ring", "palm"]
                   + [f"{f}_{b}" for f in FINGERS for b in ("yoke", "proximal", "middle", "distal", "bar")]
                   + [f"thumb_{b}" for b in ("base", "metacarpal", "proximal", "distal")])


@pytest.fixture(scope="module")
def model() -> mujoco.MjModel:
    return load_model()


def _act():
    sys.path.insert(0, str(ROOT / "model"))
    import build_hand

    return build_hand.load_actuators()[0]


# ------------------------------------------------------------------------------------------- contract structure
def test_names_order_limits(model):
    names = {model.body(i).name for i in range(model.nbody)} - {"world"}
    assert names == set(CONTRACT_BODIES)
    assert model.body("forearm").parentid[0] == 0 and model.body_jntnum[model.body("forearm").id] == 0
    joints = {model.joint(i).name for i in range(model.njnt)}
    assert joints == set(SERVO_JOINTS) | set(WRIST_JOINTS) | {f"{f}_dip" for f in FINGERS} | {f"{f}_bar" for f in FINGERS}
    assert all(model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE and model.jnt_limited[i] for i in range(model.njnt))
    acts = [model.actuator(i).name for i in range(model.nu)]
    assert acts == [*SERVO_JOINTS, "wrist_flex", "wrist_dev"]
    act = _act()
    for i, a in enumerate(acts):
        jid = model.actuator_trnid[i, 0]
        assert model.joint(jid).name == a
        np.testing.assert_allclose(model.actuator_ctrlrange[i], model.jnt_range[jid])
        np.testing.assert_allclose(model.actuator_gainprm[i, 0], act[a]["kp"], rtol=1e-6)
        np.testing.assert_allclose(model.actuator_forcerange[i], act[a]["forcerange"], rtol=1e-6)
        np.testing.assert_allclose(model.dof_damping[model.jnt_dofadr[jid]], act[a]["damping"], rtol=1e-6)
        np.testing.assert_allclose(model.dof_armature[model.jnt_dofadr[jid]], act[a]["armature"], rtol=1e-6)
    for b, d in kin()["bodies"].items():
        if d["joint"]:
            np.testing.assert_allclose(model.jnt_range[model.joint(d["joint"]).id], d["range_rad"], atol=1e-9)
    for s in ("wrist_centre", "forearm_flange", "palm_centre", "thumb_tip", *(f"{f}_tip" for f in FINGERS)):
        model.site(s)


def test_cad_ranges(model):
    """Servo/wrist ranges equal the CAD's (deg): fingers 16_finger RANGE + F.ABD_RANGE, thumb T.RANGES, wrist 09."""
    exp = {"wrist_flex": (-50, 50), "wrist_dev": (-30, 20), "thumb_cmc_rot": (-31, 110), "thumb_cmc_flex": (-37, 63),
           "thumb_mcp_flex": (-10, 70), "thumb_ip": (-15, 85), "index_mcp_abd": (-10, 15),
           "middle_mcp_abd": (-15, 15), "ring_mcp_abd": (-15, 15), "little_mcp_abd": (-15, 7.5)}
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
    for j in ("thumb_cmc_flex", "thumb_mcp_flex", "thumb_ip"):  # thumb flexion: tip moves toward the palm (-Y)
        assert tip("thumb_tip", j, 20)[1] < tip("thumb_tip", j, 0)[1]
    # opposition: the thumb tip swings across the palm, toward the fingers (-X)
    assert tip("thumb_tip", "thumb_cmc_rot", 40)[0] < tip("thumb_tip", "thumb_cmc_rot", 0)[0]
    assert tip("middle_tip", "wrist_flex", 20)[1] < tip("middle_tip", "wrist_flex", 0)[1] - 1e-3
    assert tip("middle_tip", "wrist_dev", 10)[0] > tip("middle_tip", "wrist_dev", 0)[0] + 1e-3
    # couplings close with the PIP
    for f in FINGERS:
        assert poly(kin()["couplings"][f"{f}_dip"]["poly"], 1.0) > 0.5


# ------------------------------------------------------------------------------------------- kinematic fidelity
def _check_configs(model, configs) -> tuple[float, float, str]:
    d = mujoco.MjData(model)
    worst_p, worst_r, where = 0.0, 0.0, ""
    for name, q, T in configs:
        set_cad_q(model, d, q)
        for b, (ep, er) in pose_errors(model, d, T).items():
            if ep > worst_p or er > worst_r:
                where = f"{name}:{b}" if ep > worst_p else where
            worst_p, worst_r = max(worst_p, ep), max(worst_r, er)
    return worst_p, worst_r, where


def test_fidelity_cad_poses(model):
    """All 9 CAD poses: every body within 0.1 mm / 0.1 deg of the CAD (poses.json T of its parts)."""
    ref = fk_reference()["poses"]
    if POSES_JSON.exists():  # the cache must equal the CAD's own per-part transforms
        pj = json.loads(POSES_JSON.read_text())
        kb = kin()["bodies"]
        part_body = {}
        for n, cb in pj["parts"].items():
            g = cb.split(":")
            part_body[n] = ({"forearm": "forearm", "ring": "wrist_ring", "palm": "palm"}.get(cb) or
                            ("palm" if g[1] in ("mount",) or g[1].endswith("_module") else
                             f"thumb_{g[1]}" if g[0] == "thumb" else g[1]))
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
    from scipy.spatial import ConvexHull

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
    for name, p in [("q0", {"q": {}})] + [(n, fk_reference()["poses"][n])
                                         for n in ("open", "spread", "wrist_flex_+40", "wrist_dev_-20")]:
        set_cad_q(model, d, p["q"])
        mujoco.mj_forward(model, d)
        assert _self_contacts(model, d) == [], name


def test_self_contact_cad_poses(model):
    """Grasp poses: only fingertip-thumb style contacts (non-adjacent), each shallow (< 2 mm, hull artefacts)."""
    d = mujoco.MjData(model)
    for name, p in fk_reference()["poses"].items():
        set_cad_q(model, d, p["q"])
        mujoco.mj_forward(model, d)
        sc = _self_contacts(model, d)
        print(name, sorted({(a, b, round(x * 1000, 2)) for a, b, x in sc}))
        assert all(x > -0.002 for _, _, x in sc), (name, sc)


# ------------------------------------------------------------------------------------------- physics
def _sim(model, ctrl: dict[str, float], t: float, record=None):
    d = mujoco.MjData(model)
    model.opt.timestep = 0.002
    for i in range(model.nu):
        d.ctrl[i] = ctrl.get(model.actuator(i).name, 0.0)
    rec = []
    for _ in range(int(round(t / model.opt.timestep))):
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


def test_bar_never_collides(model):
    """Fist motion: no contact involves a bar body."""
    targets = {j: model.actuator_ctrlrange[i, 1] * 0.9 for i, j in enumerate(SERVO_JOINTS)}
    bars = {model.body(f"{f}_bar").id for f in FINGERS}

    def rec(d):
        return any(model.geom_bodyid[c.geom1] in bars or model.geom_bodyid[c.geom2] in bars
                   for c in d.contact[: d.ncon])

    d, r = _sim(model, targets, 0.6, rec)
    assert np.all(np.isfinite(d.qpos)) and not any(r)


# ------------------------------------------------------------------------------------------- warp, builders
@pytest.mark.warp
def test_warp_put_model_and_step(model):
    wp = pytest.importorskip("warp")
    mjw = pytest.importorskip("mujoco_warp")
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    with wp.ScopedDevice("cpu"):
        mw = mjw.put_model(model)
        dw = mjw.put_data(model, d)
        for _ in range(5):
            mjw.step(mw, dw)
        q = dw.qpos.numpy()
    assert np.all(np.isfinite(q))


def test_build_hand_check():
    r = subprocess.run([sys.executable, str(ROOT / "model" / "build_hand.py"), "--check"], capture_output=True,
                       text=True)
    assert r.returncode == 0, r.stdout + r.stderr


@pytest.mark.cad
def test_extract_check():
    """extract_v21.py --check (slow: re-extracts everything, ~1-2 min; needs the cad group)."""
    pytest.importorskip("cadquery")
    r = subprocess.run([sys.executable, str(ROOT / "model" / "extract_v21.py"), "--check"], capture_output=True,
                       text=True)
    assert r.returncode == 0, r.stdout + r.stderr
