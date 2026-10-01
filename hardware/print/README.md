# Print files

Printable parts (STL/3MF), grouped by finger/assembly.

Current materials:
- Rigid skeleton: **PLA or PETG**
- Fingertip pads (planned): **TPU**

Document print settings (layer height, infill, walls, orientation, supports) next to the files.

## `v0/`: V0 prototype parts

| File | What | Print |
|---|---|---|
| `spool_r5.stl` | Tendon spool for the 28BYJ-48, 5 mm radius: **2× the pull** of the original 10 mm spool, half the speed. Two grooves (flex / ext strand), a tie hole through the outer flange next to each, double-D bore for the 5 mm shaft (slide fit, add a drop of glue if loose). Made by `hardware/cad/spool_v0.py` (`--radius` for other sizes). | PLA/PETG, flat side down, 0.12–0.16 mm layers, 100% infill (it's tiny), no supports |

After fitting a 5 mm spool, the joint moves about **twice as far per motor step**, so re-calibrate that joint's steps per radian (`K i s`, roughly double the old value). Thread each strand from its groove through its tie hole and knot it on the outside; wind flex and ext in opposite directions.
