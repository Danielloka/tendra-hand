# Smooth, steady control: research notes and jitter diagnosis (2026-10-06)

**Status:** research and diagnosis done; nothing in the training code has changed yet. The summary
and the plan are in [`research/ai/smooth-control.md`](../../ai/smooth-control.md).

## notes/

| File | What it is |
|---|---|
| `diagnosis.md` | Why the 18 M-step Brax grasp policy shakes: experiments on the CPU env, 7 ranked causes with file:line evidence and fixes |
| `research_smooth_rl.md` | Methods that make RL policies smooth (CAPS, Lipschitz, coloured noise, filters, penalty weights, metrics) |
| `research_sota.md` | How leading systems get smooth and reliable (Helix, GR00T, π0, ACT, DPPO, HIL-SERL, Tesla FSD, DextrAH) and a staged path to household tasks |
| `research_control.md` | The layer between the policy and the motors (interpolation, impedance, SCS0009 registers, hold latch, safety, sim-to-real, test statistics) |

Claims are tagged by how well they were checked ([S] source seen, [M] from memory, [UNCERTAIN] / [?]).
Check [M] items before quoting them publicly.

## scripts/

Run from the repo root with `uv run python research/experiments/2026-10-06-smooth-control/scripts/<file>`.

| Script | What it does |
|---|---|
| `jitter_probe.py POLICY.npz BUNDLE.npz [--objects ...]` | Measures a trained Brax policy's jitter on the CPU env: action rate per group, wrist motion while holding, saturation, FFT peak, exploration std, feedback through `prev_action`; with 0 and 3 mm observation noise. Not yet run on the real 18 M policy (it is on Drive). |
| `gains.py` | Prints the arm and finger actuator gains |
| `exp1.py` | Zero action: is the scene still? |
| `exp2.py` | Holding a scripted grasp with a constant action; noise injected on wrist and finger actions |
| `exp3.py` | Arm step response and a proportional "policy" holding the wrist (finds the 4.8 Hz limit cycle) |

These were written as one-off measurements, not as maintained tools.
