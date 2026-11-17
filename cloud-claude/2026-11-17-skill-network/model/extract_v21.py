"""Extract the Tendra Hand V2.1 kinematics, mass properties, meshes and sites from the CadQuery study.

Reads the V2.1 CAD study (``../2026-11-14-hardware-ideas``, read only) and writes ``model/assets/v21/``:

- ``kin.json``: the contract's bodies (parent, joint, axis, pivot, range, mass, COM, inertia, meshes), couplings
  and sites (see ``model/CONTRACT.md``);
- ``meshes/``: one visual STL and a few convex collision STLs per body, in the body frame, metres;
- ``report.json``: how everything was derived (fit residuals, mass checks, mesh stats, sources);
- ``fk_reference.json``: CAD body transforms for the 9 CAD poses and 100 random configurations, so the
  kinematic-fidelity test runs without the CAD tools.

Why the joints are *fitted* instead of copied from constants: the CAD FK (``F.body_transforms``, ``T.body_T``,
``R_palm``) is the ground truth the poses were made with. Sweeping one joint, taking child-relative-to-parent
transforms and fitting a fixed screw axis both gives the axis and *proves* the joint is a pure hinge (residuals).

Part meshes: ``out/18/_render/<part>.stl`` are the assembly's meshes, but written by the CAD's renderer at the
*last rendered pose* (``wrist_flex_+40``), not at q = 0 (the forearm-only parts were written at ``open``, identity).
We undo that pose with the exact CAD FK; the result matches the finger/thumb packages' own q = 0 STLs to float32
precision (checked below and reported).

    uv run --group cad python model/extract_v21.py [--check]
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import shutil
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path
from types import ModuleType

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CAD_STUDY = ROOT.parent / "2026-11-14-hardware-ideas"
CAD = CAD_STUDY / "cad"
OUT18 = CAD_STUDY / "out" / "18"
ASSETS = ROOT / "model" / "assets" / "v21"
MESH_POSE = "wrist_flex_+40"  # the pose the CAD renderer left the _render STLs in (see module docstring)

FINGERS = ("index", "middle", "ring", "little")
SERVO_JOINTS = (
    "index_pip",
    "index_mcp_flex",
    "index_mcp_abd",
    "thumb_ip",
    "thumb_mcp_flex",
    "thumb_cmc_flex",
    "thumb_cmc_rot",
    "middle_pip",
    "middle_mcp_flex",
    "middle_mcp_abd",
    "ring_pip",
    "ring_mcp_flex",
    "ring_mcp_abd",
    "little_pip",
    "little_mcp_flex",
    "little_mcp_abd",
)
THUMB_JOINTS = ("cmc_rot", "cmc_flex", "mcp_flex", "ip")
THUMB_BODY = {
    "base": "thumb_base",
    "metacarpal": "thumb_metacarpal",
    "proximal": "thumb_proximal",
    "distal": "thumb_distal",
}

# Visual triangle budget per body (whole hand ~150k). The forearm is seen from far away: coarse.
VIS_BUDGET = {
    "forearm": 15000,
    "wrist_ring": 6000,
    "palm": 20000,
    "yoke": 2000,
    "proximal": 6000,
    "middle": 5000,
    "distal": 5000,
    "bar": 400,
    "thumb_base": 5000,
    "thumb_metacarpal": 6000,
    "thumb_proximal": 5000,
    "thumb_distal": 5000,
}
MAX_HULL_VERTS = 64
MIN_VIS_PART_MM = (
    2.0  # parts smaller than this (chips, nip pins) are left out of the visual mesh (not the mass)
)
PAD_PARTS = (
    "tip_skin",
    "pad_prox_silicone",
    "pad_mid_silicone",
    "palm_pad",
    "palm_hypothenar_pad",
    "thenar_pad",
)


# ===================================================================================================== CAD access
def _load(name: str, path: Path) -> ModuleType:
    """Load a CAD script as a module (registered in sys.modules first: the scripts use dataclasses/lru_cache)."""
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def body_of_part(cad_body: str) -> str:
    """CAD body name (poses.json ``parts``) -> contract body name; fixed sub-bodies merge into ``palm``."""
    if cad_body == "forearm":
        return "forearm"
    if cad_body == "ring":
        return "wrist_ring"
    if cad_body == "palm":
        return "palm"
    group, b = cad_body.split(":")
    if group == "thumb":
        return "palm" if b == "mount" else THUMB_BODY[b]
    if b.endswith("_module"):
        return "palm"
    return b  # e.g. index_proximal


class CadFK:
    """The CAD's forward kinematics, mapped to the contract bodies (hand frame, mm, transform from q = 0)."""

    def __init__(self) -> None:
        self.T = _load("thumb18", CAD / "18_thumb.py")
        self.F = _load("fingers18", CAD / "18_fingers.py")
        # 13_wrist_parts.KIN is this module (it only patches H_PALM, unused by the rotations); loading 13 directly
        # would pull in cadquery and the V2 wrist builder for two rotation matrices.
        self.W = _load("kin09", CAD / "09_eyeplate_kin.py")
        self.fm = {f: self.F.finger_module(f) for f in FINGERS}

    def ranges_deg(self) -> dict[str, tuple[float, float]]:
        """Joint ranges from the CAD: finger RANGE (16_finger) with F.ABD_RANGE overrides, thumb RANGES, wrist."""
        r: dict[str, tuple[float, float]] = {}
        for f in FINGERS:
            rr = self.fm[f].RANGE
            r[f"{f}_mcp_abd"] = tuple(map(float, self.F.ABD_RANGE[f]))
            r[f"{f}_mcp_flex"] = tuple(map(float, rr["mcp"]))
            r[f"{f}_pip"] = tuple(map(float, rr["pip"]))
        for j in THUMB_JOINTS:
            r[f"thumb_{j}"] = tuple(map(float, self.T.RANGES[j]))
        r["wrist_flex"] = tuple(map(float, self.W.FLEX_RANGE))
        r["wrist_dev"] = tuple(map(float, self.W.DEV_RANGE))
        return r

    def body_T(self, q: dict[str, float]) -> dict[str, np.ndarray]:
        """{contract body: 4x4} for joint angles q (deg, CAD names; missing = 0)."""
        fl, dv = float(q.get("wrist_flex", 0.0)), float(q.get("wrist_dev", 0.0))
        Rr, Rp = np.eye(4), np.eye(4)
        Rr[:3, :3] = self.W.R_ring(fl, dv)
        Rp[:3, :3] = self.W.R_palm(fl, dv)
        out = {"forearm": np.eye(4), "wrist_ring": Rr, "palm": Rp}
        for f in FINGERS:
            bt = self.F.body_transforms(f, q)
            for b in ("yoke", "proximal", "middle", "distal", "bar"):
                out[f"{f}_{b}"] = Rp @ bt[b]
        tq = {f"thumb_{j}": float(q.get(f"thumb_{j}", 0.0)) for j in THUMB_JOINTS}
        for b, M in self.T.body_T(tq).items():
            if b != "mount":
                out[THUMB_BODY[b]] = Rp @ M
        return out


# ===================================================================================================== geometry
def rot(axis: np.ndarray, ang: float) -> np.ndarray:
    """Rotation matrix about a unit axis (right hand)."""
    a = np.asarray(axis, float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def rot_angle(R: np.ndarray) -> float:
    """Rotation angle (rad) of a rotation matrix."""
    return math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))


def rotvec(R: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(R).as_rotvec()


def fit_hinge(
    Trel: list[np.ndarray], lead: np.ndarray, ref_point: np.ndarray, test_pts: np.ndarray, coupled: bool
) -> dict:
    """Fit a fixed screw axis to child-in-parent transforms (all from q = 0) of one swept joint.

    lead: the swept (servo) angle per sample, rad. For a servo joint the hinge angle must *equal* it; for a
    coupled joint (DIP, bar) the hinge angle is measured per sample and returned. The point on the axis is the one
    closest to the CAD's stated pivot ``ref_point`` (any point on the axis is kinematically equivalent).
    Residuals: rotation (deg) and displacement of ``test_pts`` (child mesh bounding-box corners, mm) between the
    CAD transform and the fitted hinge - the honest "is it a pure hinge" number.
    """
    rv = np.array([rotvec(T[:3, :3]) for T in Trel])
    ang = np.linalg.norm(rv, axis=1)
    big = ang > math.radians(2.0)
    sgn = np.sign(lead[big])
    ax_i = rv[big] / ang[big][:, None] * sgn[:, None]
    # for a coupled joint the sign of the hinge angle vs the leader is unknown: align to the first sample
    if coupled:
        ref_dir = ax_i[np.argmax(ang[big])]
        ax_i *= np.sign(ax_i @ ref_dir)[:, None]
    axis = ax_i.mean(0)
    axis /= np.linalg.norm(axis)
    spread = float(np.degrees(np.arccos(np.clip(ax_i @ axis, -1, 1))).max())
    theta = np.sign(rv @ axis) * ang if coupled else lead.copy()
    A = np.vstack([np.eye(3) - T[:3, :3] for T in Trel])
    b = np.concatenate([T[:3, 3] for T in Trel])
    p0 = np.linalg.lstsq(A, b, rcond=None)[0]
    p = p0 + axis * float((ref_point - p0) @ axis)
    rot_res, pos_res = 0.0, 0.0
    for T, th in zip(Trel, theta):
        Rf = rot(axis, th)
        tf = p - Rf @ p
        rot_res = max(rot_res, math.degrees(rot_angle(Rf.T @ T[:3, :3])))
        d = (test_pts @ T[:3, :3].T + T[:3, 3]) - (test_pts @ Rf.T + tf)
        pos_res = max(pos_res, float(np.linalg.norm(d, axis=1).max()))
    return {
        "axis": axis,
        "pivot": p,
        "theta": theta,
        "rot_res_deg": rot_res,
        "pos_res_mm": pos_res,
        "axis_spread_deg": spread,
        "pivot_off_ref_mm": float(np.linalg.norm(np.cross(ref_point - p, axis))),
    }


def mass_props(m, mass_g: float, pitch: float = 0.25) -> tuple[np.ndarray, np.ndarray, str]:
    """COM (mm) and inertia about the COM (g mm^2) of a part mesh scaled to its real mass.

    Watertight -> exact (signed-volume integrals). Otherwise weld vertices to 1e-4 mm; if it is still
    open, voxelise at ``pitch`` mm (filled) and use the voxel centres (a convex hull would fill ring bores).
    """
    method = "exact"
    mm = m
    if not mm.is_watertight:
        mm = m.copy()
        mm.merge_vertices(digits_vertex=4)  # welds float32 seams (no networkx needed, unlike fill_holes)
        method = "welded"
    if mm.is_watertight and abs(mm.volume) > 1e-6:
        if mm.volume < 0:
            mm.invert()
        s = mass_g / mm.volume
        return np.asarray(mm.center_mass), np.asarray(mm.moment_inertia) * s, method
    vox = m.voxelized(pitch).fill()
    pts = vox.points
    com = pts.mean(0)
    d = pts - com
    w = mass_g / len(pts)
    inertia = w * (np.eye(3) * (d * d).sum() - d.T @ d) + np.eye(3) * w * len(pts) * pitch**2 / 6
    return com, inertia, f"voxel {pitch} mm ({len(pts)} voxels)"


def combine_inertia(
    items: list[tuple[float, np.ndarray, np.ndarray]],
) -> tuple[float, np.ndarray, np.ndarray]:
    """Sum (mass, com, inertia-about-com) of parts into one body (parallel-axis theorem)."""
    M = sum(i[0] for i in items)
    c = sum(i[0] * i[1] for i in items) / M
    I = np.zeros((3, 3))
    for m, ci, Ii in items:
        d = ci - c
        I += Ii + m * (np.eye(3) * (d @ d) - np.outer(d, d))
    return M, c, I


def hull_reduce(points: np.ndarray, max_verts: int = MAX_HULL_VERTS) -> tuple[np.ndarray, float]:
    """Convex hull with at most ``max_verts`` vertices, chosen greedily to minimise the lost volume.

    Start from the 6 axis extremes, then repeatedly add the full-hull vertex that lies farthest outside the
    current hull (plane distance) until the budget is used. Returns (vertices, max deviation in input units):
    the reduced hull is inside the true hull by at most that much. Deterministic.
    """
    from scipy.spatial import ConvexHull

    hv = points[ConvexHull(points).vertices]
    if len(hv) <= max_verts:
        return hv, 0.0
    sel = sorted({int(i) for i in np.concatenate([hv.argmin(0), hv.argmax(0)])})
    while len(sel) < 4:
        sel.append(int(np.argmax(np.linalg.norm(hv - hv[sel].mean(0), axis=1))))
    while True:
        h = ConvexHull(hv[sel])
        dist = (hv @ h.equations[:, :3].T + h.equations[:, 3]).max(1)
        k = int(np.argmax(dist))
        if len(sel) >= max_verts or dist[k] <= 1e-9:
            return hv[sel], float(max(dist.max(), 0.0))
        sel.append(k)


def hull_faces(v: np.ndarray) -> np.ndarray:
    """Outward-oriented triangle faces of the convex hull of v (all v are hull vertices)."""
    from scipy.spatial import ConvexHull

    h = ConvexHull(v)
    f = h.simplices.copy()
    c = v.mean(0)
    for i, tri in enumerate(f):
        n = np.cross(v[tri[1]] - v[tri[0]], v[tri[2]] - v[tri[0]])
        if n @ (v[tri[0]] - c) < 0:
            f[i] = tri[[0, 2, 1]]
    return f


def write_stl(path: Path, v: np.ndarray, f: np.ndarray, name: str) -> None:
    """Binary STL with a fixed header (byte-identical output for identical input)."""
    v32 = np.asarray(v, np.float32)
    tri = v32[np.asarray(f)]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]).astype(np.float64)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = (n / np.where(ln > 0, ln, 1)).astype(np.float32)
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tri
    head = f"tendra v21 {name}".encode()[:80].ljust(80, b" ")
    path.write_bytes(head + np.uint32(len(f)).tobytes() + rec.tobytes())


def decimate(v: np.ndarray, f: np.ndarray, target: int) -> tuple[np.ndarray, np.ndarray]:
    """Quadric decimation (fast-simplification) to about ``target`` faces; drops degenerate faces."""
    import fast_simplification

    if len(f) > target:
        v, f = fast_simplification.simplify(
            np.ascontiguousarray(v, np.float64),
            np.ascontiguousarray(f, np.int32),
            target_reduction=1.0 - target / len(f),
        )
    tri = v[f]
    area = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    return v, f[area > 1e-12]


def cluster(v: np.ndarray, f: np.ndarray, cell: float) -> tuple[np.ndarray, np.ndarray]:
    """Vertex-clustering simplification on a ``cell`` mm grid (always reduces; coarser look than quadric)."""
    key = np.floor(v / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    nv = np.zeros((inv.max() + 1, 3))
    np.add.at(nv, inv, v)
    nv /= np.bincount(inv)[:, None]
    nf = inv[f]
    ok = (nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])
    nf = np.unique(np.sort(nf[ok], axis=1), axis=0, return_index=True)[1]
    return nv, inv[f][ok][np.sort(nf)]


def visual_part(m, target: int) -> tuple[np.ndarray, np.ndarray, str]:
    """One part's visual mesh with about ``target`` faces.

    Quadric decimation first. Some CAD meshes (spools, sleeves, tessellated cylinders) stall far above the target;
    then small or nearly convex parts become their (decimated) convex hull - pins, rollers and bearings look the
    same - and the rest is vertex-clustered on a growing grid.
    """
    v, f = decimate(m.vertices, m.faces, target)
    if len(f) <= 1.5 * target:
        return v, f, "quadric"
    hull = m.convex_hull
    if np.linalg.norm(np.ptp(m.vertices, 0)) < 12.0 or abs(m.volume) > 0.75 * hull.volume:
        v, f = decimate(hull.vertices, hull.faces, target)
        return v, f, "hull"
    cell = float(np.linalg.norm(np.ptp(m.vertices, 0))) / 200.0
    while True:
        v, f = cluster(m.vertices, m.faces, cell)
        if len(f) <= 1.5 * target:
            return v, f, "cluster"
        cell *= 1.3


def visual_mesh(
    body: str, names: list[str], meshes: dict, budget: int
) -> tuple[np.ndarray, np.ndarray, dict]:
    """Merged visual mesh of a body (hand frame, mm): faces shared by part surface area, tiny parts dropped."""
    keep = [n for n in names if np.linalg.norm(np.ptp(meshes[n][1].vertices, 0)) >= MIN_VIS_PART_MM]
    area = np.array([meshes[n][1].area for n in keep])
    vs, fs, off, how = [], [], 0, defaultdict(int)
    for n, a in zip(keep, area):
        v, f, h = visual_part(meshes[n][1], max(16, int(budget * a / area.sum())))
        how[h] += 1
        vs.append(v)
        fs.append(f + off)
        off += len(v)
    return (
        np.vstack(vs),
        np.vstack(fs),
        {"parts": len(keep), "dropped_tiny": len(names) - len(keep), "method": dict(how)},
    )


def minimax_fit(A: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Coefficients c minimising max |A c - y| (linear programme): the coupling's worst-case angle error matters
    for fingertip placement, not its RMS."""
    from scipy.optimize import linprog

    n = A.shape[1]
    cost = np.r_[np.zeros(n), 1.0]
    A_ub = np.block([[A, -np.ones((len(y), 1))], [-A, -np.ones((len(y), 1))]])
    res = linprog(cost, A_ub=A_ub, b_ub=np.r_[y, -y], bounds=[(None, None)] * n + [(0, None)], method="highs")
    if not res.success:
        raise RuntimeError(res.message)
    return res.x[:n]


def rnd(x, nd: int = 10):
    """Round floats (recursively) to ``nd`` significant digits so JSON output is stable and readable.

    Significant digits, not decimals: inertias are ~1e-9 kg m^2 (a 0.4 g bar)."""
    if isinstance(x, dict):
        return {k: rnd(v, nd) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [rnd(v, nd) for v in x]
    if isinstance(x, np.ndarray):
        return rnd(x.tolist(), nd)
    if isinstance(x, (float, np.floating)):
        v = float(f"{float(x):.{nd}g}")
        return 0.0 if abs(v) < 1e-13 else v
    if isinstance(x, np.integer):
        return int(x)
    return x


# ===================================================================================================== extraction
def joint_table(fk: CadFK) -> list[dict]:
    """Every joint: (name, parent, child, leader, CAD-stated pivot point and axis for the cross-check)."""
    rows = [
        {
            "joint": "wrist_dev",
            "parent": "forearm",
            "child": "wrist_ring",
            "lead": "wrist_dev",
            "ref": (0, 0, 0),
            "ref_axis": (0, 1, 0),
        },
        {
            "joint": "wrist_flex",
            "parent": "wrist_ring",
            "child": "palm",
            "lead": "wrist_flex",
            "ref": (0, 0, 0),
            "ref_axis": (1, 0, 0),
        },
    ]
    for f in FINGERS:
        m, c = fk.fm[f], np.asarray(fk.F.MCP[f], float)
        A0 = m.link_pins(0.0, m.dip_of(0.0), 0.0)[0]
        rows += [
            {
                "joint": f"{f}_mcp_abd",
                "parent": "palm",
                "child": f"{f}_yoke",
                "lead": f"{f}_mcp_abd",
                "ref": c + (0, 0, m.Z_ABD),
                "ref_axis": (0, 1, 0),
            },
            {
                "joint": f"{f}_mcp_flex",
                "parent": f"{f}_yoke",
                "child": f"{f}_proximal",
                "lead": f"{f}_mcp_flex",
                "ref": c,
                "ref_axis": (1, 0, 0),
            },
            {
                "joint": f"{f}_pip",
                "parent": f"{f}_proximal",
                "child": f"{f}_middle",
                "lead": f"{f}_pip",
                "ref": c + (0, 0, m.ZP),
                "ref_axis": (1, 0, 0),
            },
            {
                "joint": f"{f}_dip",
                "parent": f"{f}_middle",
                "child": f"{f}_distal",
                "lead": f"{f}_pip",
                "ref": c + (0, 0, m.ZD),
                "ref_axis": (1, 0, 0),
            },
            {
                "joint": f"{f}_bar",
                "parent": f"{f}_proximal",
                "child": f"{f}_bar",
                "lead": f"{f}_pip",
                "ref": c + (0, A0[0], A0[1]),
                "ref_axis": (1, 0, 0),
            },  # pin A; x set to the bar's centre below
        ]
    chain = fk.T.D.chain
    parents = ["palm", "thumb_base", "thumb_metacarpal", "thumb_proximal"]
    for k, j in enumerate(THUMB_JOINTS):
        rows.append(
            {
                "joint": f"thumb_{j}",
                "parent": parents[k],
                "child": THUMB_BODY[fk.T.BODY_OF_JOINT[j]],
                "lead": f"thumb_{j}",
                "ref": chain.p[k],
                "ref_axis": chain.a[k],
            }
        )
    for r in rows:
        r["ref"] = np.asarray(r["ref"], float)
        r["ref_axis"] = np.asarray(r["ref_axis"], float) / np.linalg.norm(r["ref_axis"])
    return rows


def load_parts(fk: CadFK, poses: dict) -> tuple[dict, dict]:
    """Every part's mesh at q = 0 (hand frame, mm), and the q=0-recovery checks."""
    import trimesh

    qpose = poses["poses"][MESH_POSE]["q"]
    Tb = fk.body_T(qpose)
    Tjson = poses["poses"][MESH_POSE]["T"]
    meshes, worst_json, src_dev = {}, 0.0, {}
    for name, cad_body in poses["parts"].items():
        b = body_of_part(cad_body)
        M = Tb[b]
        worst_json = max(worst_json, float(np.abs(M - np.array(Tjson[name]).reshape(4, 4)).max()))
        m = trimesh.load(OUT18 / "_render" / f"{name}.stl", force="mesh", process=False)
        m = trimesh.Trimesh(m.vertices, m.faces, process=True)
        m.apply_transform(np.linalg.inv(M))
        meshes[name] = (b, m)
        for sub in ("fingers", "thumb"):
            sp = OUT18 / sub / "stl" / f"{name}.stl"
            if sp.exists():
                s = trimesh.load(sp, force="mesh")
                if len(s.vertices) == len(m.vertices):
                    src_dev[name] = float(np.abs(np.sort(s.vertices, 0) - np.sort(m.vertices, 0)).max())
                else:
                    src_dev[name] = float(np.abs(s.bounds - m.bounds).max())
    checks = {
        "mesh_pose": MESH_POSE,
        "fk_vs_posesjson_T_max_abs": worst_json,
        "parts_with_package_stl": len(src_dev),
        "package_stl_max_dev_mm": max(src_dev.values()),
        "package_stl_worst_part": max(src_dev, key=src_dev.get),
    }
    return meshes, checks


def fk_vs_poses(fk: CadFK, poses: dict) -> float:
    """Max |T| difference between our CAD-FK body mapping and every part's T in poses.json (all 9 poses)."""
    worst = 0.0
    for p in poses["poses"].values():
        Tb = fk.body_T(p["q"])
        for name, cad_body in poses["parts"].items():
            T = np.array(p["T"][name]).reshape(4, 4)
            if np.allclose(T, np.eye(4)) and poses["parts"][name] == "forearm":
                continue
            worst = max(worst, float(np.abs(Tb[body_of_part(cad_body)] - T).max()))
    return worst


def tip_site(distal_mesh, axis: np.ndarray, pivot: np.ndarray, skin) -> tuple[np.ndarray, np.ndarray]:
    """Fingertip pad centre: from the skin's centroid, along the pad normal, to the outer skin surface.

    Pad normal = the direction the tip moves when the last joint closes (axis x lever), so it works for the fingers
    (-Y at q = 0) and the thumb alike.
    """
    import trimesh

    c = skin.centroid
    n = np.cross(axis, c - pivot)
    n /= np.linalg.norm(n)
    loc, _, _ = trimesh.ray.ray_triangle.RayMeshIntersector(skin).intersects_location([c], [n])
    if len(loc) == 0:
        raise RuntimeError("tip ray missed the skin")
    p = loc[np.argmax((loc - c) @ n)]
    return p, n


def quat_z_to(n: np.ndarray) -> np.ndarray:
    """Quaternion (w, x, y, z) rotating +Z onto unit vector n."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, n)
    s, c = np.linalg.norm(v), float(z @ n)
    if s < 1e-12:
        return np.array([1.0, 0, 0, 0]) if c > 0 else np.array([0.0, 1, 0, 0])
    a = math.atan2(s, c)
    return np.concatenate([[math.cos(a / 2)], v / s * math.sin(a / 2)])


def collision_plan(body: str, parts: list[str]) -> list[dict]:
    """Which convex pieces a body gets (and why): see report.json ``collision.rationale``."""
    pads = [p for p in parts if p.endswith(PAD_PARTS)]
    rest = [p for p in parts if p not in pads]
    if body == "forearm":
        return [
            {"kind": "hull", "parts": rest, "zmax": -45.0, "cls": "geom"},
            {"kind": "hull", "parts": rest, "zmin": -45.0, "cls": "geom"},
        ]
    if body == "wrist_ring":
        return [{"kind": "coacd", "parts": ["ring_w13"], "n": 4, "cls": "geom"}]
    if body == "palm":
        # knuckle modules: index/middle/ring sit at MCP z 92-96, the little 8-11 mm lower: one hull over all four
        # bridges that step and the curled little fingertip hits it in a fist (CAD: no contact), so two hulls
        mods3 = [f"{f}_knuckle_module" for f in ("index", "middle", "ring")]
        return [
            {"kind": "coacd", "parts": ["palm_v21"], "n": 3, "cls": "geom"},
            {"kind": "hull", "parts": mods3, "cls": "geom"},
            {"kind": "hull", "parts": ["little_knuckle_module"], "cls": "geom"},
            # the thumb mount sits behind the thumb base: world contact only, no hand self-contact (review M4)
            {"kind": "hull", "parts": ["thumb_mount"], "cls": "geom", "self": False},
        ] + [{"kind": "hull", "parts": [p], "cls": "pad"} for p in pads]
    if body.endswith("_bar"):
        return []
    out = [{"kind": "hull", "parts": rest, "cls": "geom"}]
    return out + [{"kind": "hull", "parts": [p], "cls": "pad"} for p in pads]


def make_collision(plan: dict, meshes: dict, pivot: np.ndarray) -> list[tuple[np.ndarray, float]]:
    """Convex pieces (vertices in mm, hand frame) for one plan entry, each <= 64 vertices; with deviation."""
    import coacd

    if plan["kind"] == "hull":
        v = np.vstack([meshes[p][1].vertices for p in plan["parts"]])
        c = np.array([meshes[p][1].centroid[2] for p in plan["parts"]])
        if "zmax" in plan or "zmin" in plan:
            keep = [
                p for p, z in zip(plan["parts"], c) if plan.get("zmin", -1e9) <= z < plan.get("zmax", 1e9)
            ]
            v = np.vstack([meshes[p][1].vertices for p in keep])
        return [hull_reduce(v)]
    m = meshes[plan["parts"][0]][1]
    coacd.set_log_level("error")
    res = coacd.run_coacd(
        coacd.Mesh(m.vertices, m.faces),
        threshold=0.05,
        max_convex_hull=plan["n"],
        seed=7,
        max_ch_vertex=MAX_HULL_VERTS,
        preprocess_mode="auto",
    )
    pieces = sorted(res, key=lambda r: tuple(np.round(np.asarray(r[0]).mean(0), 3)))
    return [hull_reduce(np.asarray(vs, float)) for vs, _ in pieces]


def random_configs(ranges: dict, n: int, seed: int = 2026) -> list[dict[str, float]]:
    """Uniform random joint angles (deg) inside every servo + wrist range."""
    rng = np.random.default_rng(seed)
    names = [*SERVO_JOINTS, "wrist_flex", "wrist_dev"]
    return [{j: float(rng.uniform(*ranges[j])) for j in names} for _ in range(n)]


def git_rev() -> str:
    """Last commit that touched the CAD study (not HEAD: unrelated commits must not make the assets stale)."""
    try:
        r = subprocess.run(
            ["git", "log", "-1", "--format=%H", "--", str(CAD_STUDY)],
            cwd=CAD_STUDY,
            capture_output=True,
            text=True,
            check=True,
        )
        return r.stdout.strip() or "unknown"
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def extract(out: Path) -> None:
    """Run the whole extraction into ``out`` (a fresh directory)."""
    poses_bytes = (OUT18 / "poses.json").read_bytes()
    poses = json.loads(poses_bytes)
    checks = json.loads((OUT18 / "checks.json").read_text())
    fk = CadFK()
    ranges = fk.ranges_deg()
    meshes, load_checks = load_parts(fk, poses)
    load_checks["fk_vs_posesjson_all_poses_max_abs"] = fk_vs_poses(fk, poses)
    by_body: dict[str, list[str]] = defaultdict(list)
    for name, (b, _) in meshes.items():
        by_body[b].append(name)

    # ---------------------------------------------------------------- joints
    joints, report_joints, couplings = {}, {}, {}
    for row in joint_table(fk):
        child_pts = np.vstack([meshes[p][1].bounds for p in by_body[row["child"]]])
        lo, hi = child_pts.min(0), child_pts.max(0)
        test = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        if row["joint"].endswith("_bar"):
            row["ref"][0] = 0.5 * (lo[0] + hi[0])
        coupled = row["lead"] != row["joint"]
        lead_lo, lead_hi = ranges[row["lead"]]
        sweep = np.linspace(lead_lo, lead_hi, 201 if coupled else 41)
        Trel = []
        for a in sweep:
            Tb = fk.body_T({row["lead"]: float(a)})
            Trel.append(np.linalg.inv(Tb[row["parent"]]) @ Tb[row["child"]])
        fit = fit_hinge(Trel, np.radians(sweep), row["ref"], test, coupled)
        j = row["joint"]
        rep = {
            "parent": row["parent"],
            "child": row["child"],
            "leader": row["lead"],
            "samples": len(sweep),
            "axis": fit["axis"],
            "pivot_mm": fit["pivot"],
            "residual_rot_deg": fit["rot_res_deg"],
            "residual_pos_mm": fit["pos_res_mm"],
            "axis_spread_deg": fit["axis_spread_deg"],
            "cad_axis_angle_deg": math.degrees(
                math.acos(min(1.0, abs(float(fit["axis"] @ row["ref_axis"]))))
            ),
            "cad_axis_same_sign": bool(fit["axis"] @ row["ref_axis"] > 0),
            "cad_pivot_off_axis_mm": fit["pivot_off_ref_mm"],
        }
        if coupled:
            x = np.radians(sweep)
            y = fit["theta"]
            Amat = np.column_stack(
                [x, x**2, x**3, x**4]
            )  # c0 = 0: exact at q = 0 (all transforms are I there)
            c = minimax_fit(Amat, y)
            poly = [0.0, *c.tolist()]
            err = np.degrees(np.abs(Amat @ c - y))
            rep["poly_max_err_deg"] = float(err.max())
            rep["angle_range_deg"] = [float(np.degrees(y.min())), float(np.degrees(y.max()))]
            if j.endswith("_dip"):
                f = j[: -len("_dip")]
                cad_dip = np.radians([fk.F.dip(f, float(a)) for a in sweep])
                rep["fitted_angle_vs_F_dip_max_deg"] = float(np.degrees(np.abs(cad_dip - y)).max())
            couplings[j] = {"leader": row["lead"], "poly": poly}
            margin = math.radians(1.0)
            rng_rad = [float(y.min()) - margin, float(y.max()) + margin]
        else:
            rng_rad = [math.radians(lead_lo), math.radians(lead_hi)]
        joints[j] = {
            "parent": row["parent"],
            "child": row["child"],
            "axis": fit["axis"],
            "pivot": fit["pivot"] / 1000.0,
            "range": rng_rad,
            "res": {"rot_deg": fit["rot_res_deg"], "pos_mm": fit["pos_res_mm"]},
        }
        report_joints[j] = rep

    # bar must be a pure rotation about pin A *independently of the MCP/abd angles*: check at other configs
    bar_indep = {}
    for f in FINGERS:
        worst = [0.0, 0.0]
        for pip in np.linspace(*ranges[f"{f}_pip"], 21):
            T0 = fk.body_T({f"{f}_pip": float(pip)})
            R0 = np.linalg.inv(T0[f"{f}_proximal"]) @ T0[f"{f}_bar"]
            for other in (
                {f"{f}_mcp_flex": 70.0, f"{f}_mcp_abd": 10.0},
                {f"{f}_mcp_flex": -10.0, "wrist_flex": 30.0},
            ):
                Tb = fk.body_T({**other, f"{f}_pip": float(pip)})
                Tr = np.linalg.inv(Tb[f"{f}_proximal"]) @ Tb[f"{f}_bar"]
                worst[0] = max(worst[0], math.degrees(rot_angle(R0[:3, :3].T @ Tr[:3, :3])))
                worst[1] = max(worst[1], float(np.linalg.norm(R0[:3, 3] - Tr[:3, 3])))
        bar_indep[f] = {"rot_deg": worst[0], "pos_mm": worst[1]}

    # ---------------------------------------------------------------- bodies: pivots, mass, meshes
    body_joint = {r["child"]: j for j, r in joints.items()}
    body_order = ["forearm", "wrist_ring", "palm"]
    for f in ("index", "thumb", "middle", "ring", "little"):
        body_order += (
            [f"thumb_{b}" for b in ("base", "metacarpal", "proximal", "distal")]
            if f == "thumb"
            else [f"{f}_{b}" for b in ("yoke", "proximal", "middle", "distal", "bar")]
        )
    pivot = {
        b: (joints[body_joint[b]]["pivot"] * 1000 if b in body_joint else np.zeros(3)) for b in body_order
    }
    meshdir = out / "meshes"
    meshdir.mkdir(parents=True)
    bodies, rep_mass, rep_mesh, rep_coll = {}, {}, {}, {}
    total_vis = 0
    for b in body_order:
        names = sorted(by_body[b])
        items, methods = [], {}
        for n in names:
            com, I, meth = mass_props(meshes[n][1], checks["mass_g"][n])
            items.append((checks["mass_g"][n], com, I))
            if meth != "exact":
                methods[n] = meth
        M, com, I = combine_inertia(items)
        rep_mass[b] = {"mass_g": M, "parts": len(names), "non_exact": methods}
        # visual
        key = b if b in VIS_BUDGET else b.split("_", 1)[1]
        v2, f2, vis_rep = visual_mesh(b, names, meshes, VIS_BUDGET[key])
        write_stl(meshdir / f"{b}.stl", (v2 - pivot[b]) / 1000.0, f2, b)
        total_vis += len(f2)
        rep_mesh[b] = dict(
            source_faces=sum(len(meshes[n][1].faces) for n in names), visual_faces=len(f2), **vis_rep
        )
        # collision
        coll, coll_cls, coll_self, devs = [], [], [], []
        for plan in collision_plan(b, names):
            for hv, dev in make_collision(plan, meshes, pivot[b]):
                k = len(coll)
                fname = f"{b}_c{k}.stl"
                write_stl(meshdir / fname, (hv - pivot[b]) / 1000.0, hull_faces(hv - pivot[b]), fname)
                coll.append(f"meshes/{fname}")
                coll_cls.append(plan["cls"])
                coll_self.append(plan.get("self", True))
                devs.append(
                    {
                        "file": fname,
                        "kind": plan["kind"],
                        "cls": plan["cls"],
                        "verts": len(hv),
                        "hull_reduction_dev_mm": dev,
                        "parts": plan["parts"] if len(plan["parts"]) < 4 else f"{len(plan['parts'])} parts",
                    }
                )
        rep_coll[b] = devs
        j = body_joint.get(b)
        jd = joints.get(j) if j else None
        bodies[b] = {
            "parent": jd["parent"] if jd else None,
            "joint": j,
            "axis": jd["axis"] if jd else None,
            "pivot_m": pivot[b] / 1000.0,
            "range_rad": jd["range"] if jd else None,
            "mass_kg": M / 1000.0,
            "com_m": (com - pivot[b]) / 1000.0,
            "inertia": [
                I[0, 0] * 1e-9,
                I[1, 1] * 1e-9,
                I[2, 2] * 1e-9,
                I[0, 1] * 1e-9,
                I[0, 2] * 1e-9,
                I[1, 2] * 1e-9,
            ],
            "visual_mesh": f"meshes/{b}.stl",
            "collision_meshes": coll,
            "collision_class": coll_cls,
            "collision_self": coll_self,
            "fit_residual": {"rot_deg": jd["res"]["rot_deg"], "pos_mm": jd["res"]["pos_mm"]} if jd else None,
        }

    # ---------------------------------------------------------------- sites
    sites = {"wrist_centre": {"body": "forearm", "pos_m": [0.0, 0.0, 0.0], "quat": [1.0, 0, 0, 0]}}
    fl = meshes["arm_flange"][1]
    down = fl.face_normals[:, 2] < -0.99
    zmin = fl.vertices[:, 2].min()
    face_z = fl.triangles_center[:, 2]
    sel = down & (np.abs(face_z - zmin) < 1e-3)
    fc = (fl.triangles_center[sel] * fl.area_faces[sel, None]).sum(0) / fl.area_faces[sel].sum()
    sites["forearm_flange"] = {"body": "forearm", "pos_m": fc / 1000.0, "quat": [1.0, 0, 0, 0]}
    import trimesh

    palm_surf = trimesh.util.concatenate(
        [meshes[n][1] for n in ("palm_v21", "palm_pad", "palm_hypothenar_pad")]
    )
    pb = meshes["palm_v21"][1].bounds
    origin = np.array([0.5 * (pb[0, 0] + pb[1, 0]), pb[1, 1] + 5.0, 0.5 * (pb[0, 2] + pb[1, 2])])
    loc, _, _ = trimesh.ray.ray_triangle.RayMeshIntersector(palm_surf).intersects_location(
        [origin], [[0, -1, 0]]
    )
    pc = loc[np.argmin(loc[:, 1])]
    sites["palm_centre"] = {
        "body": "palm",
        "pos_m": (pc - pivot["palm"]) / 1000.0,
        "quat": quat_z_to(np.array([0.0, -1.0, 0.0])),
    }
    rep_sites = {
        "palm_centre_ray_origin_mm": origin,
        "forearm_flange_face_area_mm2": float(fl.area_faces[sel].sum()),
    }
    for f in (*FINGERS, "thumb"):
        db = "thumb_distal" if f == "thumb" else f"{f}_distal"
        jn = "thumb_ip" if f == "thumb" else f"{f}_dip"
        skin = meshes["tip_skin" if f == "thumb" else f"{f}_tip_skin"][1]
        p, n = tip_site(None, joints[jn]["axis"], joints[jn]["pivot"] * 1000, skin)
        sites[f"{f}_tip"] = {"body": db, "pos_m": (p - pivot[db]) / 1000.0, "quat": quat_z_to(n)}
        rep_sites[f"{f}_tip_normal"] = n

    # ---------------------------------------------------------------- FK reference for the tests
    def tf_list(Tb: dict) -> dict:
        return {b: np.round(Tb[b][:3, :], 7).ravel().tolist() for b in body_order}

    ref = {
        "units": "mm, 3x4 row-major hand-frame transform of each body from its q = 0 pose",
        "poses": {},
        "random": [],
    }
    for pn, p in poses["poses"].items():
        ref["poses"][pn] = {"q": p["q"], "T": tf_list(fk.body_T(p["q"]))}
    for q in random_configs(ranges, 100):
        ref["random"].append({"q": q, "T": tf_list(fk.body_T(q))})

    # ---------------------------------------------------------------- write
    src = {
        "study": "cloud-claude/2026-11-14-hardware-ideas",
        "git_rev": git_rev(),
        "poses_sha256": hashlib.sha256(poses_bytes).hexdigest(),
    }
    kin = {
        "source": src,
        "conventions": "hand frame (origin wrist centre, +Z fingers, -Y palm, +X thumb), SI. pivot_m: hand frame "
        "at q = 0. com_m: body frame (= hand frame minus pivot). inertia about the COM, hand "
        "axes [ixx, iyy, izz, ixy, ixz, iyz]. Meshes in the body frame, metres. Couplings: "
        "joint = sum c_k * leader^k (rad).",
        "bodies": bodies,
        "couplings": couplings,
        "sites": sites,
    }
    total_mass = sum(v["mass_g"] for v in rep_mass.values())
    report = {
        "source": src,
        "part_meshes": load_checks,
        "joints": report_joints,
        "bar_pure_rotation_about_pin_A_at_other_mcp_abd": {
            "max_change": bar_indep,
            "note": "bar-in-proximal transform at MCP 70/abd 10 and MCP -10/wrist 30 vs MCP 0, pip swept: it depends on the pip only, and the pip sweep fits a pure hinge about pin A (joints.<f>_bar residuals)",
        },
        "mass": {
            "total_g": total_mass,
            "checks_json_total_g": checks["mass_g_total"],
            "by_body_g": rep_mass,
            "method": "per part: real mass from checks.json mass_g, COM/inertia from its q = 0 mesh scaled to "
            "that mass (exact for watertight meshes; non-watertight: repaired or voxelised, listed)",
        },
        "meshes": {
            "visual_faces_total": total_vis,
            "by_body": rep_mesh,
            "decimation": "fast-simplification (quadric)",
        },
        "collision": {
            "by_body": rep_coll,
            "pieces_total": sum(len(v) for v in rep_coll.values()),
            "rationale": (
                "Hand-chosen convex hulls of the load-bearing printed parts, one per phalanx, plus a separate hull "
                "per silicone pad (fingertip skin, proximal/middle pads, palm/hypothenar/thenar pads) in class "
                "v21_pad: the pads are the real contact surfaces and get rubber friction, and a pad hull sticks "
                "out of the phalanx hull on the palm side, so grasps touch pads first. CoACD only where one hull "
                "would be badly wrong: the palm shell (thumb bay cut) and the wrist ring (a loop). Hulls reduced "
                "to <= 64 vertices by greedy farthest-outside insertion (deviation listed per piece, inside the "
                "true hull). Few small convex pieces keep GPU (MJX/Warp) narrow-phase cost low; the forearm is "
                "2 coarse hulls (it only meets the table)."
            ),
        },
        "sites": rep_sites,
        "notes": [
            (
                "The _render STLs are at pose wrist_flex_+40 (the CAD renderer's last pose), forearm-only parts at open "
                "(identity); q = 0 meshes recovered with the exact FK (see part_meshes)."
            ),
            (
                "Wrist cranks/rods (wrist_crank_*, wrist_rod_*) move with the wrist in reality; the CAD poses them as "
                "static forearm parts (identity), so they are merged into the forearm here (21 g)."
            ),
            "Wrist rotations: 09_eyeplate_kin.R_ring/R_palm (= 13_wrist_parts.KIN), ranges FLEX_RANGE/DEV_RANGE.",
        ],
    }
    (out / "kin.json").write_text(json.dumps(rnd(kin, 10), indent=1) + "\n")
    (out / "report.json").write_text(json.dumps(rnd(report, 6), indent=1) + "\n")
    (out / "fk_reference.json").write_text(json.dumps(rnd(ref, 8), separators=(",", ":")) + "\n")


def compare_dirs(a: Path, b: Path) -> list[str]:
    """Relative paths that differ between two directory trees."""
    fa = {p.relative_to(a) for p in a.rglob("*") if p.is_file()}
    fb = {p.relative_to(b) for p in b.rglob("*") if p.is_file()}
    diff = sorted(str(p) for p in fa ^ fb)
    diff += sorted(str(p) for p in fa & fb if (a / p).read_bytes() != (b / p).read_bytes())
    return diff


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="exit 1 if model/assets/v21 is not up to date")
    args = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "v21"
        out.mkdir()
        extract(out)
        if args.check:
            diff = compare_dirs(out, ASSETS) if ASSETS.exists() else ["(missing)"]
            if diff:
                print("model/assets/v21 is stale:", ", ".join(diff[:20]))
                return 1
            print("model/assets/v21 is up to date")
            return 0
        if ASSETS.exists():
            shutil.rmtree(ASSETS)
        shutil.copytree(out, ASSETS)
    print(f"wrote {ASSETS.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
