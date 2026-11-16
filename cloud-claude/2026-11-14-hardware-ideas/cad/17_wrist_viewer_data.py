"""Export the W13 metal wrist for the 3D viewer: a GLB of its parts (failed reorder plate left out), the
part -> body map, ring/palm rotations and the 32 strand polylines over a wrist grid.
Run: uv run --no-project --with cadquery --with numpy --with trimesh --with scipy python cad/17_wrist_viewer_data.py"""
import importlib.util, json, sys
from pathlib import Path
import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "out"
sys.argv = [sys.argv[0]]
spec = importlib.util.spec_from_file_location("w13", HERE / "13_wrist_parts.py")
W = importlib.util.module_from_spec(spec); sys.modules["w13"] = W; spec.loader.exec_module(W)
KIN = W.KIN

body = {"ring_w13": "ring", "eye_plate_w13": "ring", "pin_dev_B_hollow": "ring", "pin_dev_F": "ring",
        "bush_flex_R": "ring", "bush_flex_L": "ring", "palm_yoke_w13": "palm", "palm_plate_w13": "palm",
        "pin_flex_R_hollow": "palm", "pin_flex_L": "palm"}
cols = {"ring_w13": "#8a9bb0", "palm_yoke_w13": "#6f8fb8", "forearm_yoke_w13": "#7a7a7a", "eye_plate_w13": "#c8ccd2",
        "palm_plate_w13": "#c8ccd2", "forearm_plate_w13": "#c8ccd2", "forearm_plate_carrier_w13": "#555555",
        "pin_flex_R_hollow": "#c07090", "pin_dev_B_hollow": "#c07090", "pin_flex_L": "#b0b4bb", "pin_dev_F": "#b0b4bb"}
sc = trimesh.Scene()
for f in sorted((OUT / "13").glob("*.stl")):
    n = f.stem
    if n.startswith("reorder"):
        continue
    m = trimesh.load(f)
    c = cols.get(n, "#d9a441" if n.startswith("bush") else "#999999").lstrip("#")
    m.visual.face_colors = [int(c[i:i + 2], 16) for i in (0, 2, 4)] + [255]
    sc.add_geometry(m, node_name=n, geom_name=n)
    body.setdefault(n, "fore")
(OUT / "viewer").mkdir(exist_ok=True)
glb = sc.export(file_type="glb")


def rdp(P, eps=0.1):
    P = np.asarray(P, float)
    if len(P) < 3:
        return P
    a, b = P[0], P[-1]; ab = b - a; n = np.linalg.norm(ab)
    d = np.linalg.norm(np.cross(P - a, ab), axis=1) / n if n > 1e-9 else np.linalg.norm(P - a, axis=1)
    i = int(np.argmax(d))
    return np.vstack([rdp(P[: i + 1], eps)[:-1], rdp(P[i:], eps)]) if d[i] > eps else np.vstack([a, b])


F = list(range(-50, 51, 10)); D = list(range(-30, 21, 10))
poses = {}
names = None
for f in F:
    for d in D:
        strands, pls = W.strand_polys(float(f), float(d))
        names = names or [s["name"] for s in strands]
        poses[f"{f},{d}"] = {"ring": np.round(KIN.R_ring(f, d), 6).tolist(), "palm": np.round(KIN.R_palm(f, d), 6).tolist(),
                             "lines": [np.round(rdp(p), 1).tolist() for p in pls]}
data = {"body": body, "flex": F, "dev": D, "strand_names": names, "poses": poses}
import base64
(OUT / "viewer" / "wrist_w13.glb.b64.txt").write_text(base64.b64encode(glb).decode())
(OUT / "viewer" / "wrist_w13_data.json").write_text(json.dumps(data, separators=(",", ":")))
print(len(glb) // 1024, "kB glb;", (OUT / "viewer" / "wrist_w13_data.json").stat().st_size // 1024, "kB data;", names[:4])
