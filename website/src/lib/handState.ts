/**
 * The contract between the scroll timeline and the 3D hand.
 *
 * The GSAP timeline (Scroll Animation Agent) WRITES these numbers as you
 * scroll; the R3F scene (3D Agent) READS them every frame in useFrame. It's a
 * plain mutable object, not React state, so scrolling never re-renders React.
 *
 * All values are plain numbers so GSAP can tween them directly, e.g.
 *   tl.to(handState, { explode: 1 })
 *   tl.to(handState.rotation, { y: Math.PI })
 *   tl.to(handState.joints, { index_pip: 1 })
 *
 * Owned by the lead: change it only through PLAN.md.
 */

/** Joints of the full human-like hand. The first 8 exist on the real prototype today. */
export const JOINT_IDS = [
  // Real prototype (see CLAUDE.md motor map)
  "thumb_cmc_rot",
  "thumb_cmc_flex",
  "thumb_mcp",
  "thumb_ip",
  "index_mcp_abd",
  "index_mcp_flex",
  "index_pip",
  "index_dip",
  // Planned fingers (placeholder model only)
  "middle_mcp_flex",
  "middle_pip",
  "middle_dip",
  "ring_mcp_flex",
  "ring_pip",
  "ring_dip",
  "little_mcp_flex",
  "little_pip",
  "little_dip",
] as const;

export type JointId = (typeof JOINT_IDS)[number];

/** Short anatomical label shown next to a joint in the "Joints" section. */
export const JOINT_LABELS: Record<JointId, string> = {
  thumb_cmc_rot: "CMC",
  thumb_cmc_flex: "CMC",
  thumb_mcp: "MCP",
  thumb_ip: "IP",
  index_mcp_abd: "MCP",
  index_mcp_flex: "MCP",
  index_pip: "PIP",
  index_dip: "DIP",
  middle_mcp_flex: "MCP",
  middle_pip: "PIP",
  middle_dip: "DIP",
  ring_mcp_flex: "MCP",
  ring_pip: "PIP",
  ring_dip: "DIP",
  little_mcp_flex: "MCP",
  little_pip: "PIP",
  little_dip: "DIP",
};

export type HandState = {
  /** Whole-model rotation in radians (applied on top of the idle spin). */
  rotation: { x: number; y: number; z: number };
  /** 0..1 — how much of the slow idle rotation/breathing is applied (1 in the hero). */
  idle: number;
  /** 0..1 — accent rim light strength (1 = dramatic hero lighting). */
  rim: number;
  /** 0..1 — tendon paths visible and glowing. */
  tendons: number;
  /** 0..1 per joint — fraction of that joint's flexion range (0 = straight, 1 = fully bent). */
  joints: Record<JointId, number>;
  /** 0..1 — joint labels (MCP/PIP/DIP…) visible. */
  labels: number;
  /** 0..1 — exploded view: parts move apart along their axes. */
  explode: number;
  /** 0..1 — solid → wireframe crossfade. */
  wireframe: number;
  /** Camera zoom multiplier (1 = default framing). */
  zoom: number;
  /** Moves the hand down in its canvas, as a fraction of the canvas height (0 = centred). */
  shiftY: number;
};

function initialJoints(): Record<JointId, number> {
  return Object.fromEntries(JOINT_IDS.map((id) => [id, 0])) as Record<JointId, number>;
}

export function createHandState(): HandState {
  return {
    rotation: { x: 0, y: 0, z: 0 },
    idle: 1,
    rim: 1,
    tendons: 0,
    joints: initialJoints(),
    labels: 0,
    explode: 0,
    wireframe: 0,
    zoom: 1,
    shiftY: 0,
  };
}

/** The single shared instance used by the homepage. */
export const handState: HandState = createHandState();
