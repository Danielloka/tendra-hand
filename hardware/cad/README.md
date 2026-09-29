# CAD

Source design files for the hand.

- `*.f3d`: Fusion 360 archives (the source of truth)
- `*.step`: neutral exports so people without Fusion can open and modify the design
- `fusion_scripts/`: Fusion API scripts (see its README). `TendraHandV1/` builds the full v1 hand.
- `tendon_router.py`: v1 tendon routes + forearm servo layout (single source of truth for the CAD channels and the MuJoCo tendons). Run `uv run python hardware/cad/tendon_router.py`; tests in `tests/` (run by `uv run pytest`).

When the design changes, export both, and re-export the URDF into `../robot_description/fusion_export/`.

## Tendra Hand V1

Fusion design **"Tendra Hand V1"** (a copy of "Hand assebly"). Workflow after a change:
1. `uv run python hardware/cad/tendon_router.py` (if routes/layout changed) and `uv run pytest hardware/cad/tests`
2. In Fusion, run the `TendraHandV1` stages (each skips itself if already applied), then stage `export`
3. `uv run python sim/convert_v1.py && uv run pytest`
