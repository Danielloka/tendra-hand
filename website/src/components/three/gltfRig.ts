import { Box3, Group, type Material, Mesh, type Object3D, Vector3 } from "three";
import { JOINT_IDS, JOINT_LABELS, type JointId } from "@/lib/handState";
import type { HandMaterials } from "./materials";
import { type Anchor, type Rig, type RigLabel, addLineOverlays } from "./rig";

/** The GLB is scaled so the hand is this tall (same framing as the placeholder). */
const TARGET_HEIGHT = 0.248;
/** Tendons sit this far behind each joint node (along its local -Z), in metres after scaling. */
const TENDON_BACK = 0.011;
const EXPLODE_FACTOR = 0.45;

/** Joint chains from wrist to tip; a `<finger>_tip` node (optional) ends the chain. */
const CHAINS: { finger: string; joints: JointId[]; side: 1 | -1 }[] = [
  { finger: "thumb", joints: ["thumb_cmc_rot", "thumb_cmc_flex", "thumb_mcp", "thumb_ip"], side: 1 },
  { finger: "index", joints: ["index_mcp_abd", "index_mcp_flex", "index_pip", "index_dip"], side: -1 },
  { finger: "middle", joints: ["middle_mcp_flex", "middle_pip", "middle_dip"], side: -1 },
  { finger: "ring", joints: ["ring_mcp_flex", "ring_pip", "ring_dip"], side: -1 },
  { finger: "little", joints: ["little_mcp_flex", "little_pip", "little_dip"], side: -1 },
];

/** Joints that share a pivot with the next one; they get one combined label. */
const MERGED_LABEL: Partial<Record<JointId, JointId>> = { thumb_cmc_rot: "thumb_cmc_flex", index_mcp_abd: "index_mcp_flex" };

/**
 * Builds a Rig from a loaded glTF scene (see public/models/README.md for the
 * naming convention). Nodes named after a JointId rotate about their local X
 * axis. Without such nodes the model still shows, lights, glows and turns;
 * only the per-joint effects (bending, labels, tendons, explode) are skipped.
 */
export function rigFromGltf(source: Object3D, mats: HandMaterials): Rig {
  const model = source.clone(true);

  // Own copies of the materials, so rim glow / wireframe fade don't leak into the loader cache.
  const cloned = new Map<Material, Material>();
  const meshes: Mesh[] = [];
  model.traverse((o) => {
    if (!(o instanceof Mesh)) return;
    meshes.push(o);
    const swap = (m: Material) => {
      let c = cloned.get(m);
      if (!c) {
        c = mats.addSolid(m.clone());
        cloned.set(m, c);
      }
      return c;
    };
    o.material = Array.isArray(o.material) ? o.material.map(swap) : swap(o.material);
  });
  const disposeEdges = addLineOverlays(meshes, mats);

  // Normalise: scale to the target height, centre on the origin.
  const root = new Group();
  root.name = "gltf_hand";
  root.add(model);
  const box = new Box3().setFromObject(model);
  const size = box.getSize(new Vector3());
  const scale = size.y > 0 ? TARGET_HEIGHT / size.y : 1;
  model.scale.multiplyScalar(scale);
  size.multiplyScalar(scale);
  model.position.copy(box.getCenter(new Vector3()).multiplyScalar(-scale));
  root.updateWorldMatrix(true, true);

  const joints: Rig["joints"] = {};
  const explode: Rig["explode"] = [];
  for (const id of JOINT_IDS) {
    const node = model.getObjectByName(id);
    if (!node) continue;
    joints[id] = { node, rest: node.quaternion.clone(), axis: new Vector3(1, 0, 0) };
    if (node.position.lengthSq() > 0) explode.push({ node, rest: node.position.clone(), factor: EXPLODE_FACTOR });
  }

  const labels: RigLabel[] = [];
  const tendons: Anchor[][] = [];
  const worldScale = new Vector3();
  for (const chain of CHAINS) {
    const path: Anchor[] = [];
    for (const id of chain.joints) {
      const node = joints[id]?.node;
      if (!node) continue;
      const merged = MERGED_LABEL[id];
      if (!merged || !joints[merged]) {
        const ids = (Object.keys(MERGED_LABEL) as JointId[]).filter((k) => MERGED_LABEL[k] === id).concat(id);
        labels.push({ text: JOINT_LABELS[id], ids, anchor: node, side: chain.side });
      }
      node.getWorldScale(worldScale);
      path.push({ node, offset: new Vector3(0, 0, -TENDON_BACK / (worldScale.z || 1)) });
    }
    const tip = model.getObjectByName(`${chain.finger}_tip`);
    if (tip) {
      tip.getWorldScale(worldScale);
      path.push({ node: tip, offset: new Vector3(0, 0, -TENDON_BACK / (worldScale.z || 1)) });
    }
    if (path.length >= 2) tendons.push(path);
  }

  return {
    root,
    size,
    joints,
    explode,
    tendons,
    labels,
    dispose: () => {
      cloned.forEach((m) => m.dispose());
      disposeEdges();
    },
  };
}

