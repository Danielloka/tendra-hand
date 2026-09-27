import {
  type BufferGeometry,
  CapsuleGeometry,
  CylinderGeometry,
  Group,
  type Material,
  Matrix4,
  Mesh,
  Object3D,
  Vector3,
} from "three";
import { RoundedBoxGeometry } from "three/examples/jsm/geometries/RoundedBoxGeometry.js";
import type { JointId } from "@/lib/handState";
import type { HandMaterials } from "./materials";
import { type Anchor, type Rig, type RigLabel, addLineOverlays } from "./rig";

/*
 * Procedural placeholder hand (a right hand), in metres.
 * Frame: +Y = fingers, +Z = palm side (facing the camera at rotation 0),
 * +X = thumb side. Every joint is a pivot group sitting at the joint; its
 * flexion axis is local +X, so a positive angle bends the finger toward the
 * palm (+Z).
 */

type FingerSpec = {
  name: "index" | "middle" | "ring" | "little";
  /** Knuckle (MCP) position on the palm. */
  x: number;
  y: number;
  /** Rest fan angle around Z (positive leans toward the little finger). */
  splay: number;
  radius: number;
  /** Proximal, middle, distal phalanx lengths. */
  lengths: [number, number, number];
  joints: [JointId, JointId, JointId];
  abduction?: JointId;
};

const FINGERS: FingerSpec[] = [
  { name: "index", x: 0.0285, y: 0.093, splay: -0.07, radius: 0.0094, lengths: [0.041, 0.025, 0.021], joints: ["index_mcp_flex", "index_pip", "index_dip"], abduction: "index_mcp_abd" },
  { name: "middle", x: 0.0092, y: 0.097, splay: -0.015, radius: 0.0097, lengths: [0.045, 0.029, 0.022], joints: ["middle_mcp_flex", "middle_pip", "middle_dip"] },
  { name: "ring", x: -0.0105, y: 0.094, splay: 0.05, radius: 0.0091, lengths: [0.042, 0.027, 0.021], joints: ["ring_mcp_flex", "ring_pip", "ring_dip"] },
  { name: "little", x: -0.0292, y: 0.086, splay: 0.12, radius: 0.0081, lengths: [0.033, 0.021, 0.019], joints: ["little_mcp_flex", "little_pip", "little_dip"] },
];

const PALM = { width: 0.086, height: 0.1, depth: 0.028, radius: 0.012 };
/** Taper of the three phalanges (radius multiplier). */
const TAPER = [1, 0.92, 0.85];
/** Tendons ride just above the back surface. */
const TENDON_LIFT = 0.0016;
/** Exploded view: how far each part moves, relative to its distance from its parent joint. */
const EXPLODE = { finger: 0.42, phalanx: 0.4, forearm: 0.6 };

const X = new Vector3(1, 0, 0);

export function buildPlaceholderHand(mats: HandMaterials): Rig {
  const geometries: BufferGeometry[] = [];
  const meshes: Mesh[] = [];
  const joints: Rig["joints"] = {};
  const explode: Rig["explode"] = [];
  const tendons: Anchor[][] = [];
  const labels: RigLabel[] = [];

  const geo = <T extends BufferGeometry>(g: T) => (geometries.push(g), g);
  const mesh = (g: BufferGeometry, m: Material, parent: Object3D) => {
    const me = new Mesh(g, m);
    meshes.push(me);
    parent.add(me);
    return me;
  };
  const anchor = (node: Object3D, x: number, y: number, z: number): Anchor => ({ node, offset: new Vector3(x, y, z) });
  const labelAt = (parent: Object3D, text: string, ids: JointId[], side: 1 | -1, y = 0) => {
    const a = new Object3D();
    a.position.set(0, y, 0);
    parent.add(a);
    labels.push({ text, ids, anchor: a, side });
  };
  const joint = (id: JointId, node: Object3D, axis = X) => {
    joints[id] = { node, rest: node.quaternion.clone(), axis: axis.clone().normalize() };
  };
  const exploding = (node: Object3D, factor: number) => explode.push({ node, rest: node.position.clone(), factor });

  /** One phalanx: a capsule from the pivot (y=0) to the next joint (y=len), plus the joint pin. */
  const phalanx = (pivot: Object3D, len: number, r: number, m: Material, tip: boolean, pin: boolean) => {
    const inset = r * 0.35; // leaves a visible notch at each joint
    const start = inset;
    const end = tip ? len - r : len - inset;
    const body = mesh(geo(new CapsuleGeometry(r, Math.max(end - start, 0.001), 4, 12)), m, pivot);
    body.position.y = (start + end) / 2;
    if (pin) {
      const p = mesh(geo(new CylinderGeometry(r * 0.42, r * 0.42, r * 2.1, 14)), mats.pin, pivot);
      p.rotation.z = Math.PI / 2;
    }
  };

  const root = new Group();
  root.name = "placeholder_hand";

  // ---- Palm + wrist mount ---------------------------------------------------
  const palm = new Group();
  palm.name = "palm";
  root.add(palm);
  const palmMesh = mesh(geo(new RoundedBoxGeometry(PALM.width, PALM.height, PALM.depth, 4, PALM.radius)), mats.real, palm);
  palmMesh.position.y = PALM.height / 2 - 0.002;

  const forearm = new Group();
  forearm.name = "forearm";
  forearm.position.y = -0.03;
  palm.add(forearm);
  const forearmMesh = mesh(geo(new CylinderGeometry(0.03, 0.032, 0.05, 32)), mats.real, forearm);
  forearmMesh.scale.z = 0.56;
  exploding(forearm, EXPLODE.forearm);
  const backZ = -(PALM.depth / 2 + TENDON_LIFT);
  const forearmBackZ = -(0.031 * 0.56 + TENDON_LIFT);

  // ---- Fingers ----------------------------------------------------------------
  for (const f of FINGERS) {
    const m = f.name === "index" ? mats.real : mats.planned;
    const base = new Group();
    base.name = `${f.name}_base`;
    base.position.set(f.x, f.y, 0);
    base.rotation.z = f.splay;
    palm.add(base);
    exploding(base, EXPLODE.finger);

    let parent: Object3D = base;
    if (f.abduction) {
      const abd = new Group();
      abd.name = f.abduction;
      base.add(abd);
      // Positive = toward the thumb (+X): rotation about -Z.
      joint(f.abduction, abd, new Vector3(0, 0, -1));
      parent = abd;
    }

    const lx = f.x * 0.9; // tendon lane on the back of the palm
    const path: Anchor[] = [
      anchor(forearm, lx * 0.75, -0.02, forearmBackZ),
      anchor(forearm, lx * 0.85, 0.018, forearmBackZ),
      anchor(palm, lx, 0.035, backZ),
      anchor(palm, lx, 0.07, backZ),
    ];

    f.joints.forEach((id, i) => {
      const r = f.radius * TAPER[i];
      const pivot = new Group();
      pivot.name = id;
      if (i > 0) pivot.position.y = f.lengths[i - 1];
      parent.add(pivot);
      joint(id, pivot);
      if (i > 0) exploding(pivot, EXPLODE.phalanx);
      const last = i === 2;
      phalanx(pivot, f.lengths[i], r, m, last, true);
      labelAt(pivot, ["MCP", "PIP", "DIP"][i], i === 0 && f.abduction ? [f.abduction, id] : [id], -1);

      const back = -(r + TENDON_LIFT);
      path.push(anchor(pivot, 0, 0, back), anchor(pivot, 0, f.lengths[i] * 0.5, back));
      if (last) path.push(anchor(pivot, 0, f.lengths[i] - r * 0.7, back * 0.75));
      parent = pivot;
    });
    tendons.push(path);
  }

  // ---- Thumb --------------------------------------------------------------------
  const thumbRoot = new Group();
  thumbRoot.name = "thumb_base";
  thumbRoot.position.set(0.027, 0.016, 0.004);
  palm.add(thumbRoot);
  exploding(thumbRoot, EXPLODE.finger * 1.2);

  // Opposition: swings the thumb from beside the palm to in front of it (toward +Z, then -X).
  const cmcRot = new Group();
  cmcRot.name = "thumb_cmc_rot";
  thumbRoot.add(cmcRot);
  joint("thumb_cmc_rot", cmcRot, new Vector3(-0.3, -1, 0.1));

  // Thumb frame: +Y along the thumb, +Z toward the palm/fingers (its flexion side).
  const along = new Vector3(0.48, 0.84, 0.24).normalize();
  const flexSide = new Vector3(-0.75, 0.2, 0.62);
  flexSide.addScaledVector(along, -flexSide.dot(along)).normalize();
  const side = new Vector3().crossVectors(along, flexSide);
  const frame = new Group();
  frame.quaternion.setFromRotationMatrix(new Matrix4().makeBasis(side, along, flexSide));
  cmcRot.add(frame);

  const thumb = {
    joints: ["thumb_cmc_flex", "thumb_mcp", "thumb_ip"] as JointId[],
    labels: ["CMC", "MCP", "IP"],
    lengths: [0.046, 0.032, 0.027],
    radii: [0.0125, 0.0104, 0.0097],
  };
  const thumbPath: Anchor[] = [
    anchor(forearm, 0.021, -0.02, forearmBackZ),
    anchor(forearm, 0.024, 0.018, forearmBackZ),
    anchor(palm, 0.03, 0.03, backZ),
  ];
  let thumbParent: Object3D = frame;
  thumb.joints.forEach((id, i) => {
    const r = thumb.radii[i];
    const pivot = new Group();
    pivot.name = id;
    if (i > 0) pivot.position.y = thumb.lengths[i - 1];
    thumbParent.add(pivot);
    joint(id, pivot);
    if (i > 0) exploding(pivot, EXPLODE.phalanx);
    const last = i === 2;
    phalanx(pivot, thumb.lengths[i], r, mats.real, last, i > 0);
    labelAt(pivot, thumb.labels[i], i === 0 ? ["thumb_cmc_rot", id] : [id], 1, i === 0 ? 0.012 : 0);

    const back = -(r + TENDON_LIFT);
    if (i > 0) thumbPath.push(anchor(pivot, 0, 0, back));
    thumbPath.push(anchor(pivot, 0, thumb.lengths[i] * 0.5, back));
    if (last) thumbPath.push(anchor(pivot, 0, thumb.lengths[i] - r * 0.7, back * 0.75));
    thumbParent = pivot;
  });
  tendons.push(thumbPath);

  const disposeEdges = addLineOverlays(meshes, mats);

  // Centre the assembled hand on the origin (the scene rotates around it).
  const min = new Vector3(-0.075, -0.055, -0.02);
  const max = new Vector3(0.075, 0.193, 0.02);
  const size = new Vector3().subVectors(max, min);
  root.position.copy(min.add(max).multiplyScalar(-0.5));

  return {
    root,
    size,
    joints,
    explode,
    tendons,
    labels,
    dispose: () => {
      geometries.forEach((g) => g.dispose());
      disposeEdges();
    },
  };
}
