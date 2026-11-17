"""One-off: copy the OpenArm v2 visual meshes this study uses into model/assets/openarm/ (decimated), with
OpenArm's Apache-2.0 LICENSE and a NOTICE, so the generated XMLs are self-contained.

    uv run --with fast-simplification python model/vendor_openarm.py

Only the visual meshes of the pedestal and of the right arm's base, link1..link4 are kept (collision is done with
boxes, see build_robot.py). Each mesh is quadric-decimated to about `KEEP` of its triangles (minimum
`MIN_FACES`) and written as a binary STL in the source's own units. The generated files and the build use the
mesh *names* of OpenArm's MJCF (e.g. `link3_right_00.stl`).
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_robot as br

OUT = br.OPENARM_ASSETS
KEEP = 0.05
MIN_FACES = 600


def decimate(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    import fast_simplification

    faces = len(mesh.faces)
    if faces <= MIN_FACES:
        return mesh
    target = max(MIN_FACES, int(faces * KEEP))
    verts, tri = fast_simplification.simplify(
        mesh.vertices.astype(np.float32), mesh.faces.astype(np.int32), target_count=target, agg=7
    )
    return trimesh.Trimesh(verts.astype(float), tri, process=False)


def main() -> None:
    from openarm_mujoco.v2 import asset_path

    pkg_assets = Path(asset_path("openarm_bimanual.xml")).parent / "assets"
    spec = br.trim_openarm(vendored=False)
    OUT.mkdir(parents=True, exist_ok=True)
    lines, total_in, total_out = [], 0, 0
    for mesh in spec.meshes:
        used = any(g.meshname == mesh.name and g.group != 3 for g in spec.geoms)
        if not used:
            continue
        src = pkg_assets / mesh.file
        tm = trimesh.load(src, force="mesh", process=False)
        small = decimate(tm)
        small.export(OUT / f"{mesh.name}.stl")
        total_in += len(tm.faces)
        total_out += len(small.faces)
        lines.append(f"  {mesh.name:34s} {len(tm.faces):7d} -> {len(small.faces):6d} triangles  (from {mesh.file})")
    lic = next(Path(br.ROOT.parent / ".venv").glob("lib/python3.*/site-packages/openarm_mujoco-*.dist-info/licenses/LICENSE"))
    shutil.copy(lic, OUT / "LICENSE")
    notice = f"""NOTICE

The meshes in this folder are derived from the OpenArm v2 MuJoCo description (PyPI package `openarm-mujoco`
2.3.0, https://github.com/enactic/openarm_mujoco), Copyright Enactic, Inc., licensed under the Apache License 2.0
(see LICENSE in this folder).

What was copied: the visual meshes of the pedestal (`body_link0`) and of the right arm's base_link, link1, link2,
link3 and link4 (the parts the Tendra robot model keeps). OpenArm's collision meshes, link5 and below, the
gripper and the left arm are NOT included.

What was changed: every mesh was quadric-decimated to about {int(KEEP * 100)} % of its triangles (at least {MIN_FACES} where it
had more) with `fast-simplification`, converted to binary STL and renamed after its MJCF mesh name, by
`model/vendor_openarm.py`. Units and frames are those of the source. No other change.

Triangles (source -> here): {total_in} -> {total_out}
{chr(10).join(lines)}
"""
    (OUT / "NOTICE").write_text(notice)
    print(notice)


if __name__ == "__main__":
    main()
