# 3D models

The website shows a **procedural placeholder hand** until a real model is put
here. To switch to the real hand:

1. Export the model as `public/models/hand.glb` (steps below).
2. In `src/components/three/model.ts`, change the one line to
   `export const HAND_MODEL_URL: string | null = "/models/hand.glb";`
3. Open `/dev/hand` (the hand lab) and check every preset and joint slider.

Keep the file small: aim for **under 3 MB** (it loads on the homepage).

## Coordinate frame

| | |
|---|---|
| Up (+Y) | toward the fingertips (hand pointing up) |
| Toward the camera (+Z) | the **palm** side |
| +X | the **thumb** side (it's a right hand seen from the palm) |
| Units | metres (the site rescales the model to a fixed height anyway) |

## Node names (this is what makes the hand move)

Each joint needs an **empty/node named exactly like the joint**, sitting at the
joint's pivot. Everything that should move with the joint (the next finger
segment, and all segments after it) is a **child** of that node.

- The node's **origin** is the pivot point.
- The node's **local X axis** is the rotation axis.
- A **positive** rotation about local X is "closing the hand" (the project's
  sign convention, see `CLAUDE.md`): fingers bend toward the palm, the thumb
  swings across the palm, `index_mcp_abd` moves the index toward the thumb.
  If a joint bends the wrong way, flip that node's X axis in Blender
  (rotate the empty 180° about its Y or Z axis and re-parent).

Joint names (all 17; the first 8 exist on the real prototype):

```
thumb_cmc_rot  thumb_cmc_flex  thumb_mcp  thumb_ip
index_mcp_abd  index_mcp_flex  index_pip  index_dip
middle_mcp_flex  middle_pip  middle_dip
ring_mcp_flex    ring_pip    ring_dip
little_mcp_flex  little_pip  little_dip
```

Hierarchy example for the index finger:

```
palm
└─ index_mcp_abd        (empty at the knuckle)
   └─ index_mcp_flex    (empty at the knuckle)
      ├─ index_proximal (mesh)
      └─ index_pip      (empty at the middle joint)
         ├─ index_middle (mesh)
         └─ index_dip   (empty at the fingertip joint)
            ├─ index_distal (mesh)
            └─ index_tip    (optional empty at the fingertip)
```

Optional extras:

- `thumb_tip`, `index_tip`, `middle_tip`, `ring_tip`, `little_tip`: empties at
  the fingertips. The glowing tendon lines run through the joint nodes and end
  at these; without them a tendon ends at the last joint.
- The tendon lines sit ~1 cm behind each joint along the node's **local −Z**
  (the back of the hand), so keep local +Z pointing to the palm side.

What happens without the joint nodes: the model still shows, lights, glows
and turns, but nothing bends, and there are no labels, tendons or exploded
view.

How the effects use the nodes:

| Effect | Uses |
|---|---|
| Joint bending | rotation about local X, on top of the node's rest pose |
| Labels (MCP / PIP / DIP / CMC / IP) | pinned to the joint node's origin |
| Tendons | a curve through the joint nodes (+ `*_tip`), offset to the back |
| Exploded view | each joint node slides outward along its offset from its parent |
| Wireframe | automatic outline + sharp edges of every mesh |

Joint ranges (how far `1.0` on a slider turns) are in
`src/components/three/rig.ts` (`JOINT_RANGE`).

## Export: Fusion 360 → Blender → glTF

1. **Fusion 360:** export each body, or the whole design, as **OBJ** or
   **STEP**/**FBX** (File → Export). Use the assembly where each finger
   segment is its own body.
2. **Blender** (free): import it. Check the scale (1 unit = 1 m; Fusion often
   exports millimetres, so scale by 0.001 and apply the scale with
   Ctrl+A → Scale).
3. Rotate the hand so it matches the frame above (fingers up +Y, palm facing
   the front). Blender is Z-up; the glTF exporter converts to Y-up for you when
   **+Y Up** is ticked (the default).
4. Add an **Empty** (Shift+A → Empty → Plain Axes) at each joint, name it,
   rotate it so its X axis is the joint axis, and parent the segments to it as
   in the hierarchy above (select child, then parent, Ctrl+P → Object (Keep
   Transform)).
5. Reduce detail: a Decimate modifier (ratio 0.2–0.5) on dense CAD meshes, and
   apply it. Screws and tiny parts can go.
6. Materials: one simple Principled BSDF per material (the site adds its own
   lighting, rim glow and wireframe on top). No textures needed.
7. File → Export → **glTF 2.0**: Format **glTF Binary (.glb)**, Include →
   *Selected Objects* or *Visible Objects*, Transform → **+Y Up**, Data →
   Mesh: *Apply Modifiers*. **Draco compression** is optional: it makes the
   file much smaller, but the decoder is then downloaded from Google's CDN
   (gstatic.com) on first load.
8. Save as `public/models/hand.glb`, set `HAND_MODEL_URL`, and test in
   `/dev/hand`.

## Current model

`hand.glb` is made straight from the STEP file by a script (no Blender):

```
uv run --with cadquery-ocp --with pygltflib python website/scripts/step-to-glb.py
```

It reads `hardware/cad/Hand assebly.step` (thumb + index prototype, 9 parts,
~13k triangles, 0.4 MB) and rigs it automatically: each part is matched to a
body of the MuJoCo model (`sim/models/tendra_hand.xml`) by its bounding box, and
the joint nodes are placed at the URDF joint pivots with the sim's axis signs
(positive = closing). `index_tip` / `thumb_tip` mark the fingertips. The STEP
and the URDF export must come from the same Fusion design; the script stops
if a sim body has no matching part.

`index_mcp_abd` and `thumb_cmc_rot` have sideways axes, so the tendon line
skips them and runs through the next joint (see `gltfRig.ts`).
