"""(Round 2: also takes --finger 16 and adds the sensor wire.)
Export data for the 3D web viewer: which body each part moves with, joint axes, the DIP(PIP) table,
DIP-bar pin positions and tendon polylines over a pose grid. Writes out/viewer/finger_data.json.
Run: uv run --no-project --with cadquery --with numpy python cad/11_viewer_data.py"""
import argparse, importlib.util, json, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser(); ap.add_argument("--finger", default="07"); ARGS = ap.parse_args()
spec = importlib.util.spec_from_file_location("f07", HERE / f"{ARGS.finger}_finger.py")
F = importlib.util.module_from_spec(spec); sys.modules["f07"] = F; spec.loader.exec_module(F)

body = {k: v[0] for k, v in F.build_metal().items()}
body.update({"base": "base", "yoke": "yoke", "proximal": "proximal", "middle": "middle", "distal": "distal",
             "cup_abd": "yoke", "cup_mcp_flex": "proximal", "cup_mcp_ext": "proximal", "cup_pip": "middle",
             "cup_abd_flex": "yoke", "cup_abd_ext": "yoke", "cup_pip_flex": "middle", "cup_pip_ext": "middle",
             "tip_skin": "distal", "tip_pcb": "distal", "tip_chip": "distal", "tip_magnet": "distal",
             "tip_carrier": "distal", "nail_shim": "distal", "dip_bar": "bar"})
pips = np.arange(-5, 96, 5.0)
dips = [float(F.dip_of(p)) for p in pips]
A, B, _, _ = F.link_pins(0.0, F.dip_of(0.0), 0.0)

MCP = np.arange(-10, 91, 10.0); PIP = np.arange(-5, 96, 10.0); ABD = np.arange(-15, 16, 5.0)
t0 = time.time(); grid = {}
names = list(F.SDEF)
for a in ABD:
    for m in MCP:
        for p in PIP:
            q = {"abd": float(a), "mcp": float(m), "pip": float(p)}
            lines = []
            for k in names:
                _, _, P = F.strand(k, q, with_pts=True); lines.append(np.round(np.asarray(P), 2).tolist())
            for role in ("flex", "ext"):
                _, _, P = F.abd_strand(role, q, with_pts=True)
                P = np.asarray(P, float).copy()
                P[:-1] = F.xf(F.mat_rot((0, 1, 0), q["abd"], (0, 0, F.Z_ABD)), P[:-1])
                lines.append(np.round(P, 2).tolist())
            if hasattr(F, "sensor_wire"):
                lines.append(np.round(F.sensor_wire(q), 2).tolist())
            grid[f"{int(a)},{int(m)},{int(p)}"] = lines
print("strands", time.time() - t0, "s")
out = {"body": body, "axes": {"abd": {"p": [0, 0, F.Z_ABD], "d": [0, 1, 0]}, "mcp": {"p": [0, 0, 0], "d": [1, 0, 0]},
                              "pip": {"p": [0, 0, F.ZP], "d": [1, 0, 0]}, "dip": {"p": [0, 0, F.ZD], "d": [1, 0, 0]}},
       "dip_table": {"pip": pips.tolist(), "dip": dips}, "pinA": [float(v) for v in A], "pinB": [float(v) for v in B],
       "strand_names": names + ["abd_flex", "abd_ext"] + (["sensor_wire"] if hasattr(F, "sensor_wire") else []),
       "grid": {"abd": ABD.tolist(), "mcp": MCP.tolist(), "pip": PIP.tolist(), "lines": grid}}
d = HERE.parent / "out" / "viewer"; d.mkdir(parents=True, exist_ok=True)
(d / f"finger_data_{ARGS.finger}.json").write_text(json.dumps(out, separators=(",", ":")))
print("written", (d / f"finger_data_{ARGS.finger}.json").stat().st_size // 1024, "kB")


def _rdp(P, eps=0.08):
    P = np.asarray(P, float)
    if len(P) < 3:
        return P
    a, b = P[0], P[-1]; ab = b - a; n = np.linalg.norm(ab)
    d = (np.linalg.norm(np.cross(P - a, ab), axis=1) / n) if n > 1e-9 else np.linalg.norm(P - a, axis=1)
    i = int(np.argmax(d))
    if d[i] > eps:
        return np.vstack([_rdp(P[: i + 1], eps)[:-1], _rdp(P[i:], eps)])
    return np.vstack([a, b])


# compact pass: simplify each polyline (0.08 mm) and round to 0.1 mm
for k, lines in out["grid"]["lines"].items():
    out["grid"]["lines"][k] = [np.round(_rdp(l), 1).tolist() for l in lines]
(d / f"finger_data_{ARGS.finger}.json").write_text(json.dumps(out, separators=(",", ":")))
print("compact", (d / f"finger_data_{ARGS.finger}.json").stat().st_size // 1024, "kB")
