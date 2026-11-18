"""One-off: copy the OpenArm v2 visual meshes this study uses into model/assets/openarm/, unchanged, with OpenArm's
Apache-2.0 LICENSE and a NOTICE, so the generated XMLs are self-contained (any working directory, any venv, Docker).

    uv run python model/vendor_openarm.py

Only the visual meshes of the pedestal and of the right arm's base, link1..link4 are kept (collision is done with
boxes, see build_robot.py). Files are copied byte for byte and renamed after OpenArm's MJCF mesh names (e.g.
`link3_right_00.obj`). An earlier version decimated them, which flipped faces and made the arm render
see-through, so nothing is modified now.
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_robot as br

OUT = br.OPENARM_ASSETS


def main() -> None:
    from importlib.metadata import distribution

    from openarm_mujoco.v2 import asset_path

    pkg_assets = Path(asset_path("openarm_bimanual.xml")).parent / "assets"
    spec = br.trim_openarm(vendored=False)
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    lines = []
    for mesh in spec.meshes:
        if not any(g.meshname == mesh.name and g.group != 3 for g in spec.geoms):
            continue
        src = pkg_assets / mesh.file
        dst = OUT / f"{mesh.name}{src.suffix}"
        shutil.copyfile(src, dst)
        lines.append(f"  {dst.name:34s} <- {mesh.file}")
    dist = distribution("openarm-mujoco")
    lic = next(f for f in dist.files if f.name == "LICENSE")
    shutil.copyfile(dist.locate_file(lic), OUT / "LICENSE")
    notice = f"""NOTICE

The meshes in this folder are copied from the OpenArm v2 MuJoCo description (PyPI package `openarm-mujoco`
{dist.version}, https://github.com/enactic/openarm_mujoco), Copyright Enactic, Inc., licensed under the Apache License
2.0 (see LICENSE in this folder).

What was copied: the visual meshes of the pedestal (`body_link0`) and of the right arm's base_link, link1, link2,
link3 and link4 (the parts the Tendra robot model keeps). OpenArm's collision meshes, link5 and below, the gripper
and the left arm are NOT included.

What was changed: nothing in the files; they are only renamed after their MJCF mesh names by
`model/vendor_openarm.py`.

Files:
{chr(10).join(lines)}
"""
    (OUT / "NOTICE").write_text(notice)
    print(notice)


if __name__ == "__main__":
    main()
