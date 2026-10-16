"""Lightweight copies of the MuJoCo models, with simplified meshes, for fast drawing.

The exported CAD meshes are very detailed (v1: 238k triangles, most of them in the palm's
tendon channels, the forearm and the spools). A laptop's integrated GPU struggles to draw that
60 times a second. For live views such as teleoperation, this module loads the same model with
every large mesh simplified (quadric decimation, `fast-simplification`) to a fraction of its
triangles. The shapes look the same from a normal viewing distance.

Everything else is identical: bodies, joints, tendons, sites and actuators. Masses and inertias
are copied from the full model, so the physics doesn't change either (only the contact shapes
are slightly coarser). The model files in the repo are never touched.

    model = load_lite_model(path)            # MjModel, ~15 % of the triangles
"""

import functools
import struct
from pathlib import Path

import mujoco
import numpy as np

KEEP = 0.15  # fraction of triangles to keep
MIN_FACES = 1000  # meshes smaller than this are left alone
MIRROR_X_MM = -33.5  # the left hand's mirror plane, as in hardware/cad/mirror_export.py
LEFT_SUFFIX = "_left"  # the left export directory is the right one's name + this


def read_stl(path: Path) -> np.ndarray:
    """Triangles (N x 3 x 3) of a binary STL, in the file's units."""
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    dtype = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(data[84 : 84 + n * 50], dtype=dtype)["v"].astype(float)


def simplify(tris: np.ndarray, keep: float = KEEP) -> tuple[np.ndarray, np.ndarray]:
    """(vertices, faces) with about `keep` of the triangles. Shared corners are merged first."""
    import fast_simplification

    corners = tris.reshape(-1, 3)
    verts, faces = np.unique(np.round(corners, 6), axis=0, return_inverse=True)
    faces = faces.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
                  & (faces[:, 0] != faces[:, 2])]  # fmt: skip
    if len(faces) < MIN_FACES:
        return verts, faces
    verts, faces = fast_simplification.simplify(
        verts.astype(np.float32), faces.astype(np.int32), target_reduction=1.0 - keep
    )
    return verts.astype(float), faces


@functools.lru_cache(maxsize=256)
def simplify_cached(path: Path, keep: float) -> tuple[np.ndarray, np.ndarray]:
    return simplify(read_stl(path), keep)


def lite_mesh(path: Path, keep: float = KEEP) -> tuple[np.ndarray, np.ndarray]:
    """(vertices, faces) of the STL at `path`, simplified. A mesh of the left export
    (`<dir>_left/`, made by `mirror_export.py`) is the *reflection* of its right counterpart's
    simplified mesh, not simplified on its own: simplification is not mirror-symmetric, and the
    two hands must have identical physics."""
    path = Path(path)
    parts = list(path.parts)
    for i, part in enumerate(parts[:-1]):
        right = Path(*parts[:i], part.removesuffix(LEFT_SUFFIX), *parts[i + 1 :])
        if part.endswith(LEFT_SUFFIX) and right.exists():
            verts, faces = simplify_cached(right, keep)
            verts = verts.copy()
            verts[:, 0] = 2 * MIRROR_X_MM - verts[:, 0]
            return verts, faces[:, [0, 2, 1]]
    return simplify_cached(path, keep)


def simplify_meshes(spec: mujoco.MjSpec, mesh_dir: Path, keep: float = KEEP) -> None:
    """Replace every STL mesh of `spec` (files in `mesh_dir`) by a simplified copy, in place.

    Mesh inertia is switched to `legacy` because simplified meshes may not be watertight: copy
    the masses and inertias from the full model afterwards (`copy_inertia`).
    """
    for mesh in spec.meshes:
        if not mesh.file.lower().endswith(".stl"):
            continue
        verts, faces = lite_mesh(Path(mesh_dir) / mesh.file, keep)
        mesh.file = ""
        mesh.uservert = verts.ravel().tolist()
        mesh.userface = faces.ravel().astype(int).tolist()
        mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_LEGACY


INERTIA_FIELDS = ("body_mass", "body_inertia", "body_ipos", "body_iquat")


def copy_inertia(dst: mujoco.MjModel, src: mujoco.MjModel, prefix: str = "") -> None:
    """Copy mass and inertia of every body of `src` to the body named `prefix` + its name in `dst`.

    Call `mujoco.mj_setConst` on `dst` afterwards.
    """
    for b in range(1, src.nbody):
        target = dst.body(prefix + src.body(b).name).id
        for name in INERTIA_FIELDS:
            getattr(dst, name)[target] = getattr(src, name)[b]


def load_lite_model(path: str | Path, keep: float = KEEP) -> mujoco.MjModel:
    """The model at `path`, with simplified meshes (see the module docstring)."""
    path = Path(path)
    full = mujoco.MjModel.from_xml_path(str(path))
    spec = mujoco.MjSpec.from_file(str(path))
    simplify_meshes(spec, path.parent / spec.meshdir, keep)
    lite = spec.compile()
    if lite.nbody != full.nbody:
        raise RuntimeError("lite model has a different body structure")
    for name in INERTIA_FIELDS:
        getattr(lite, name)[:] = getattr(full, name)
    mujoco.mj_setConst(lite, mujoco.MjData(lite))
    return lite
