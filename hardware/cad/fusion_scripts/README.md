# Fusion scripts

Python scripts that run inside Fusion (Autodesk Fusion API). Each script lives in its own folder with a `.py` and a `.manifest` of the same name.

**Run one:** Fusion → **Utilities → Scripts and Add-Ins** → **+** (add from device) → pick the script's folder → select it → **Run**.

Run design-changing scripts on a **copy** of the design, or check the version history afterwards.

| Script | What it does | Changes the design? |
|---|---|---|
| `TendraInspect` | Writes `inspect_output.txt` with components, bodies, joints and axis positions (world mm) | No |
| `TendraPipPinch` | Fills the fingertip tendon slot at the index PIP and adds a 1.2 mm hole on the joint axis (see `research/experiments/2026-09-27-pip-pinch/`) | Yes: 2 timeline features, asks first |
| `TendraIndexPinch` | Index knuckle: 1.4 mm hole on the MCP flex axis and a 1 mm slit on the sideways axis (see `research/experiments/2026-09-27-index-pinch/`). Needs the finger straight | Yes: 2 features per change, asks first |

Output files (`*_output.txt`) are not committed.
