#!/bin/sh
# Copy the full-hand export (out/18) into the viewer: gzip + base64 the GLB (the page decompresses it), copy poses.
set -e
cd "$(dirname "$0")/../out"
gzip -9 -c 18/hand_v21.glb | base64 -w0 > viewer/hand_v21.glb.gz.b64.txt
cp 18/poses.json viewer/hand_v21_poses.json
ls -la viewer/hand_v21*
