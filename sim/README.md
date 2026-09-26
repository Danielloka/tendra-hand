# Simulation

**MuJoCo** models and tools for simulating the hand and running its **digital twin** (a simulated copy that mirrors, and can drive, the real hand).

## Quick start

From the repository root (needs [uv](https://docs.astral.sh/uv/)):

```bash
uv sync                          # create the Python 3.12 environment (first time only)
uv run python sim/view.py        # open the hand in the MuJoCo viewer
```

Move the joints with the sliders under **Control** in the right-hand panel.

## Files

| File | What it does |
|---|---|
| `convert.py` | Turns the raw Fusion export (`hardware/robot_description/fusion_export/`) into `models/tendra_hand.xml` and fixes it along the way: PLA masses, recomputed inertias, joint names, sign convention, actuators |
| `models/tendra_hand.xml` | The generated MuJoCo model (**don't edit by hand**; re-run `convert.py`) |
| `view.py` | Interactive viewer |
| `tests/` | Model sanity checks: `uv run pytest` |

After a new Fusion export, or a change to `convert.py`:
```bash
uv run python sim/convert.py && uv run pytest
```

## Model conventions

- Joints and actuators are listed in **motor order** (M1…M8): `index_dip, index_pip, index_mcp_flex, index_mcp_abd, thumb_ip, thumb_mcp, thumb_cmc_flex, thumb_cmc_rot`
- **Positive = closing the hand**, and 0 = straight. Angles are in radians.
- Actuators are simple position controllers standing in for the stepper + tendon, and will be tuned against the real hand later.

NVIDIA Isaac Sim/Lab may be added later on a machine with an NVIDIA GPU.
