#!/usr/bin/env bash
# Bearings round pipeline (2026-11-13), resumable: each stage logs to out/assembly/<stage>.log and writes a marker
# out/assembly/.stage_<name> when done; re-running skips finished stages. Run: nohup bash run_bearings_pipeline.sh &
cd "$(dirname "$0")"
R="uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy --with pillow --with rtree python -u"
O=../out/assembly
stage() { # name, command...
  local n=$1; shift
  [ -f $O/.stage_$n ] && { echo "skip $n"; return; }
  "$@" > $O/$n.log 2>&1 && touch $O/.stage_$n && echo "done $n" >> $O/pipeline.log
}
stage routing   $R routing.py
stage polish    $R routing.py --polish
stage palm      $R palm.py
stage poses     $R poses.py --resume
stage assembly  $R assembly.py
gzip -kf $O/tendra_v2_open.step
stage checks    $R assembly_checks.py --resume
echo "PIPELINE DONE" >> $O/pipeline.log
