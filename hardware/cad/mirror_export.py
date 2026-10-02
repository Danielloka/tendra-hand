"""Mirror the Tendra Hand V1 export into a LEFT hand (hardware/robot_description/v1_export_left/).

The V1 is a right hand. A left hand is its mirror image: same parts, same servos, same joint limits,
same tendon routes, reflected in a plane. This script reflects the Fusion export that the simulation
reads (`v1_export/`), so `sim/convert_v1.py --side left` can build the left hand with the same code:

    uv run python hardware/cad/mirror_export.py            # write v1_export_left/
    uv run python hardware/cad/mirror_export.py --check    # exit 1 if it is out of date

The mirror plane is x = MIRROR_X_MM in the hand model's frame (X = thumb side, Y = back of the hand,
Z = along the fingers): the plane through the forearm's long axis and the wrist point, so the wrist,
the forearm flange and the arm attachment are the same points for both hands. Run it again after
every re-export from Fusion (a test fails while the left export is out of date).

What is mirrored:
- `meshes/*.stl`: every vertex reflected, triangle winding reversed (so the normals stay outward;
  the printable left hand = these STLs).
- `hand_v1.json`: joint points and axes.
- `tendon_routes.json`: only what the simulation reads (strand paths, the thumb's sim paths and
  drums). The CAD-only fields (bores, servo boxes, palm sizes...) are left out on purpose: they are
  not meaningful for the reflected hand until a left hand is designed in CAD.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "hardware" / "robot_description" / "v1_export"
DST = ROOT / "hardware" / "robot_description" / "v1_export_left"

MIRROR_X_MM = -33.5  # the plane x = -33.5 mm: through the forearm axis and the wrist point


def point(p) -> list[float]:
    """A position (mm) reflected in the mirror plane."""
    return [round(2 * MIRROR_X_MM - float(p[0]), 6), float(p[1]), float(p[2])]


def direction(v) -> list[float]:
    """A direction reflected in the mirror plane."""
    return [-float(v[0]), float(v[1]), float(v[2])]


# ----- STL ---------------------------------------------------------------------------------------


def read_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    dtype = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(data[84 : 84 + n * 50], dtype=dtype)["v"].astype(np.float64)


def stl_bytes(tris: np.ndarray, header: bytes = b"Tendra Hand V1 left (mirrored)") -> bytes:
    tris = np.asarray(tris, dtype=np.float32)
    normal = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    length = np.linalg.norm(normal, axis=1, keepdims=True)
    normal = np.where(length > 0, normal / np.maximum(length, 1e-30), 0).astype(np.float32)
    dtype = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    out = np.zeros(len(tris), dtype=dtype)
    out["n"], out["v"] = normal, tris
    return header.ljust(80, bytes(1)) + struct.pack("<I", len(tris)) + out.tobytes()


def mirror_mesh(tris: np.ndarray) -> np.ndarray:
    """Triangles (N, 3, 3) in mm reflected in the plane, winding reversed."""
    out = tris.copy()
    out[..., 0] = 2 * MIRROR_X_MM - out[..., 0]
    return out[:, [0, 2, 1], :]


# ----- JSON --------------------------------------------------------------------------------------


def mirror_spec(spec: dict) -> dict:
    out = dict(spec)
    out["note"] = (
        "LEFT hand: mirror image of v1_export/ in the plane x = "
        f"{MIRROR_X_MM} mm, made by hardware/cad/mirror_export.py. Do not edit."
    )
    out["joints"] = [
        {**j, "point_mm": point(j["point_mm"]), "axis": direction(j["axis"])}
        for j in spec["joints"]
    ]
    return out


def mirror_routes(routes: dict) -> dict:
    strands = []
    for s in routes["strands"]:
        m = dict(s)
        for key in ("entry_mm", "small_bore_end_mm", "curve_start_mm", "wrist_bottom_mm",
                    "tangent_mm", "spool_center_mm"):  # fmt: skip
            if key in m:
                m[key] = point(m[key])
        m["points_mm"] = [point(p) for p in s["points_mm"]]
        if "spool_axis" in m:
            m["spool_axis"] = direction(m["spool_axis"])
        strands.append(m)
    out: dict = {
        "source": routes.get("source", ""),
        "note": "LEFT hand: only the fields the simulation reads, mirrored (see mirror_export.py)",
        "strands": strands,
    }
    sim = (routes.get("thumb") or {}).get("sim")
    if sim:
        paths = {}
        for name, path in sim["strands"].items():
            side_mm, anchor_dir = path["side_mm"], path["anchor_dir"]
            paths[name] = {
                **path,
                "points": [[body, point(p)] for body, p in path["points"]],
                "side_mm": None if side_mm is None else point(side_mm),
                "anchor_dir": None if anchor_dir is None else direction(anchor_dir),
            }
        drums = {n: {**d, "center_mm": point(d["center_mm"])} for n, d in sim["drums"].items()}
        out["thumb"] = {"sim": {"strands": paths, "drums": drums}}
    return out


def dump(obj: dict) -> str:
    return json.dumps(obj, indent=1) + "\n"


def build(dst: Path = DST, src: Path = SRC) -> dict[Path, bytes]:
    """The files of the left export as {path: content} (nothing is written)."""
    files: dict[Path, bytes] = {}
    spec = json.loads((src / "hand_v1.json").read_text(encoding="utf-8"))
    files[dst / "hand_v1.json"] = dump(mirror_spec(spec)).encode()
    routes = json.loads((src / "tendon_routes.json").read_text(encoding="utf-8"))
    files[dst / "tendon_routes.json"] = dump(mirror_routes(routes)).encode()
    for part in spec["parts"]:
        rel = Path(part["mesh"])
        files[dst / rel] = stl_bytes(mirror_mesh(read_stl(src / rel)))
    return files


def stale(dst: Path = DST, src: Path = SRC) -> list[Path]:
    """Files of the left export that differ from a fresh mirror (or are missing)."""
    return [path for path, content in build(dst, src).items()
            if not path.exists() or path.read_bytes() != content]  # fmt: skip


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="only check, don't write")
    args = parser.parse_args()
    if args.check:
        old = stale()
        for path in old:
            print(f"out of date: {path.relative_to(ROOT)}")
        print("left export is up to date" if not old else "run hardware/cad/mirror_export.py")
        return 1 if old else 0
    files = build()
    for path, content in files.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    print(f"Wrote {len(files)} files to {DST.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
