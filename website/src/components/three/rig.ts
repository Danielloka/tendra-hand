import type { BufferGeometry, Object3D, Quaternion, Vector3 } from "three";
import { EdgesGeometry, LineSegments, MathUtils, Mesh } from "three";
import type { JointId } from "@/lib/handState";
import type { HandMaterials } from "./materials";
import { onIdle } from "./idle";

const deg = MathUtils.degToRad;

/**
 * How far each joint turns at `handState.joints[id] = 1`. Roughly human
 * ranges; positive = closing the hand (CLAUDE.md sign convention).
 */
export const JOINT_RANGE: Record<JointId, number> = {
  thumb_cmc_rot: deg(55), // opposition: thumb swings across the palm
  thumb_cmc_flex: deg(40),
  thumb_mcp: deg(55),
  thumb_ip: deg(75),
  index_mcp_abd: deg(15), // toward the thumb
  index_mcp_flex: deg(90),
  index_pip: deg(100),
  index_dip: deg(80),
  middle_mcp_flex: deg(90),
  middle_pip: deg(100),
  middle_dip: deg(80),
  ring_mcp_flex: deg(90),
  ring_pip: deg(100),
  ring_dip: deg(80),
  little_mcp_flex: deg(90),
  little_pip: deg(100),
  little_dip: deg(80),
};

export type RigJoint = {
  node: Object3D;
  /** Rest orientation; the joint angle is applied on top of it. */
  rest: Quaternion;
  /** Unit rotation axis in the node's local space (positive = closing). */
  axis: Vector3;
};

/** A point fixed to a node, e.g. the back of a finger segment. */
export type Anchor = { node: Object3D; offset: Vector3 };

export type RigLabel = {
  text: string;
  /** The label shows while any of these joints is bent. */
  ids: JointId[];
  anchor: Object3D;
  /** Which side of the joint the pill sits on (screen space): 1 = right, -1 = left. */
  side: 1 | -1;
};

/** Everything the scene needs to animate a hand model, procedural or GLB. */
export type Rig = {
  /** Model root, already offset so the hand's centre is at the origin. */
  root: Object3D;
  /** Bounding-box size of the assembled hand (for camera framing). */
  size: Vector3;
  joints: Partial<Record<JointId, RigJoint>>;
  /** Exploded view: position = rest * (1 + explode * factor). */
  explode: { node: Object3D; rest: Vector3; factor: number }[];
  /** One anchor chain per finger, wrist → fingertip, along the back of the hand. */
  tendons: Anchor[][];
  labels: RigLabel[];
  dispose: () => void;
};

/**
 * Wireframe-view overlays for each mesh (hidden until wireframe > 0): an
 * outline shell for the silhouette plus sharp feature edges.
 * Returns a function that frees the generated geometry.
 */
export function addLineOverlays(meshes: Mesh[], mats: HandMaterials, thresholdDeg = 40): () => void {
  const cache = new Map<BufferGeometry, EdgesGeometry>();
  // Built in idle time: edge extraction is slow and the overlays are only
  // needed once the wireframe view is reached, well after the first frame.
  const cancel = onIdle(() => {
    for (const mesh of meshes) {
      let edges = cache.get(mesh.geometry);
      if (!edges) {
        edges = new EdgesGeometry(mesh.geometry, thresholdDeg);
        cache.set(mesh.geometry, edges);
      }
      const lines = new LineSegments(edges, mats.line);
      const shell = new Mesh(mesh.geometry, mats.outline);
      for (const o of [lines, shell]) {
        o.renderOrder = 1; // after the (depth-writing) solids
        o.raycast = () => {};
        mesh.add(o);
      }
    }
  }, 3000);
  return () => {
    cancel();
    cache.forEach((g) => g.dispose());
  };
}
