#!/usr/bin/env bash
# Fix-round pipeline (resumable): waits for a running palm build, then poses -> assembly -> checks -> step.gz.
# Each stage logs to out/assembly/<stage>.log and writes its results to disk as it goes.
cd "$(dirname "$0")"
R="uv run --no-project --with cadquery --with trimesh --with numpy --with matplotlib --with scipy --with pillow --with rtree python -u"
O=../out/assembly
until [ -f $O/palm_cache/meta.json ]; do
  if grep -q "^exit" $O/palm_build.log 2>/dev/null && [ ! -f $O/palm_cache/meta.json ]; then
    $R palm.py > $O/palm_build.log 2>&1; echo "exit $?" >> $O/palm_build.log
  fi
  sleep 20
done
$R poses.py --resume > $O/poses.log 2>&1; echo "exit $?" >> $O/poses.log
$R assembly.py > $O/assembly.log 2>&1; echo "exit $?" >> $O/assembly.log
gzip -kf $O/tendra_v2_open.step && echo "gz done" >> $O/assembly.log
$R assembly_checks.py --resume > $O/checks.log 2>&1; echo "exit $?" >> $O/checks.log
echo "PIPELINE DONE" >> $O/checks.log
