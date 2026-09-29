"""Build website/public/models/hand-v1.glb, a static (unrigged) model of Tendra Hand V1 for the
project page's spinning hand.

    uv run --with trimesh --with fast-simplification --with pygltflib python website/scripts/v1-to-glb.py

Reads the Fusion export in hardware/robot_description/v1_export (STL per part, mm, all joints at 0),
keeps the palm, fingers and thumb (no servos, spools or forearm), decimates the big parts, converts
to glTF (metres, Y up, palm facing +Z, fingertips toward +Y) and writes one mesh per part.
"""

import json
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
EXPORT = ROOT / "hardware/robot_description/v1_export"
OUT = ROOT / "website/public/models/hand-v1.glb"

# Same frame change as step-to-glb.py: design frame (Z up, palm facing -Y) -> glTF (Y up, palm +Z).
URDF_TO_GLTF = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float)
MAX_FACES = {"palm": 30000}  # everything else is small enough
DEFAULT_MAX_FACES = 6000


def main() -> None:
    spec = json.loads((EXPORT / "hand_v1.json").read_text())
    scene = trimesh.Scene()
    for part in spec["parts"]:
        name = part["name"]
        if name == "forearm" or name.startswith(("servo_", "spool_")):
            continue
        mesh = trimesh.load(EXPORT / part["mesh"], force="mesh")
        limit = MAX_FACES.get(name, DEFAULT_MAX_FACES)
        if len(mesh.faces) > limit:
            mesh = mesh.simplify_quadric_decimation(face_count=limit)
        mesh.apply_scale(0.001)
        mesh.vertices = mesh.vertices @ URDF_TO_GLTF.T
        mesh.merge_vertices()
        scene.add_geometry(mesh, node_name=name, geom_name=name)
    lo, hi = scene.bounds
    print("size (m):", hi - lo)
    scene.apply_translation([-(lo[0] + hi[0]) / 2, -lo[1], -(lo[2] + hi[2]) / 2])
    OUT.write_bytes(trimesh.exchange.gltf.export_glb(scene))
    print(f"{OUT} {OUT.stat().st_size / 1e6:.2f} MB, {len(scene.geometry)} parts")


if __name__ == "__main__":
    main()
