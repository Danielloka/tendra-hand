# Fusion scripts

Python scripts that run inside Fusion (Autodesk Fusion API). Each script lives in its own folder with a `.py` and a `.manifest` of the same name.

**Run one:** Fusion → **Utilities → Scripts and Add-Ins** → **+** (add from device) → pick the script's folder → select it → **Run**.

Run design-changing scripts on a **copy** of the design, or check the version history afterwards.

| Script | What it does | Changes the design? |
|---|---|---|
| `TendraInspect` | Writes `inspect_output.txt` with components, bodies, joints and axis positions (world mm) | No |
| `TendraPipPinch` | Fills the fingertip tendon slot at the index PIP and adds a 1.2 mm hole on the joint axis (see `research/experiments/2026-09-27-pip-pinch/`) | Yes: 2 timeline features, asks first |
| `TendraIndexPinch` | Index knuckle: 1.4 mm hole on the MCP flex axis and a 1 mm slit on the sideways axis (see `research/experiments/2026-09-27-index-pinch/`). Needs the finger straight | Yes: 2 features per change, asks first |
| `TendraHandV1` | Builds the v1 hand in the "Tendra Hand V1" design, in stages: `fingers` (middle/ring/little from the index), `thumb` (5th DOF hinge), `names`, `thumb_base` (plate 4 mm lower, hollow journal, r 7.5 cmc_rot drum, hanger for the cmc_flex pin; see `research/experiments/2026-09-29-thumb-routing/`), `palm` (full palm, deeper thumb bay with journal bearing and pin hole, 42 tendon channels), `forearm` (21 SCS0009 + spools, sheath stops in the wrist plate); `export` writes STLs + joints to `hardware/robot_description/v1_export/`. Reads `tendon_routes.json` | Yes: refuses to run on any other design |

Output files (`*_output.txt`) are not committed.
