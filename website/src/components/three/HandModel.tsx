"use client";

import { Html } from "@react-three/drei";
import { createPortal, useFrame, useThree } from "@react-three/fiber";
import { lazy, useEffect, useMemo, useRef } from "react";
import { type Group, MathUtils, type Object3D, type PerspectiveCamera, Quaternion } from "three";
import { JOINT_IDS, type HandState } from "@/lib/handState";
import type { HandMaterials } from "./materials";
import { HAND_MODEL_URL } from "./model";
import { buildPlaceholderHand } from "./placeholderHand";
import { JOINT_RANGE, type Rig, type RigLabel } from "./rig";
import { Tendons } from "./tendons";

export type ModelProps = { state: HandState; mats: HandMaterials; animated: boolean };

// The glTF loaders live in their own chunk, fetched only once a real model is set.
const GltfHand = lazy(() => import("./GltfHand"));

/** The hand: the real GLB when HAND_MODEL_URL is set, else the procedural placeholder. */
export function HandModel(props: ModelProps) {
  return HAND_MODEL_URL ? <GltfHand url={HAND_MODEL_URL} {...props} /> : <PlaceholderHand {...props} />;
}

function PlaceholderHand(props: ModelProps) {
  const rig = useMemo(() => buildPlaceholderHand(props.mats), [props.mats]);
  useEffect(() => rig.dispose, [rig]);
  return <RigView rig={rig} {...props} />;
}


const FOV_TAN = Math.tan(MathUtils.degToRad(30) / 2);
const clamp01 = (v: number) => MathUtils.clamp(v, 0, 1);
const q = new Quaternion();

/** Below this canvas width, neighbouring labels on one finger alternate sides so they never stack. */
const COMPACT_LABELS_PX = 1024; // = the phone/tablet story layout

/** True when `node` sits somewhere below `ancestor` in the scene graph. */
function isBelow(node: Object3D | null, ancestor: Object3D | null) {
  for (let n = node; n && ancestor; n = n.parent) if (n === ancestor) return true;
  return false;
}

/** Label sides; on small canvases every second label along one finger chain moves to the other side. */
function labelSides(labels: RigLabel[], compact: boolean): (1 | -1)[] {
  const out: (1 | -1)[] = [];
  labels.forEach((l, i) => {
    const prev = labels[i - 1];
    const sameChain = compact && prev && prev.side === l.side && isBelow(l.anchor.parent, prev.anchor.parent);
    out.push(sameChain && out[i - 1] === l.side ? (-l.side as 1 | -1) : l.side);
  });
  return out;
}

/**
 * Applies HandState to a rig every frame (no React re-renders): pose,
 * explode, rim, wireframe, tendons, labels and camera framing.
 */
export function RigView({ rig, state, mats, animated }: ModelProps & { rig: Rig }) {
  const turntable = useRef<Group>(null);
  const labelEls = useRef<(HTMLDivElement | null)[]>([]);
  const labelShown = useRef<number[]>([]);
  const tendons = useMemo(() => new Tendons(rig.tendons, mats), [rig, mats]);
  useEffect(() => () => tendons.dispose(), [tendons]);
  const compact = useThree((s) => s.size.width < COMPACT_LABELS_PX);
  const sides = useMemo(() => labelSides(rig.labels, compact), [rig, compact]);

  // Runs before drei's <Html> (priority -1) so the labels use this frame's pose.
  useFrame(({ camera, size, clock }) => {
    const hand = turntable.current;
    if (!hand) return;
    const t = clock.elapsedTime;
    const idle = animated ? clamp01(state.idle) : 0;

    // Whole-model rotation + idle sway and "breathing".
    hand.rotation.set(
      state.rotation.x + idle * 0.05 * Math.sin(t * 0.45),
      state.rotation.y + idle * 0.42 * Math.sin(t * 0.22),
      state.rotation.z + idle * 0.02 * Math.sin(t * 0.31),
    );
    hand.position.y = idle * 0.003 * Math.sin(t * 0.9);

    // Joints: rest orientation × (axis, fraction of range). Idle adds a tiny, always-closing flutter.
    JOINT_IDS.forEach((id, i) => {
      const j = rig.joints[id];
      if (!j) return;
      const micro = idle * 0.045 * (0.5 + 0.5 * Math.sin(t * 0.8 - i * 0.55));
      q.setFromAxisAngle(j.axis, (clamp01(state.joints[id]) + micro) * JOINT_RANGE[id]);
      j.node.quaternion.copy(j.rest).multiply(q);
    });

    const explode = clamp01(state.explode);
    for (const p of rig.explode) p.node.position.copy(p.rest).multiplyScalar(1 + explode * p.factor);

    mats.setRim(clamp01(state.rim));
    mats.setWireframe(clamp01(state.wireframe));

    // Frame the whole hand for the canvas aspect; explode needs more room.
    const cam = camera as PerspectiveCamera;
    const aspect = size.width / Math.max(size.height, 1);
    const fitH = (rig.size.y * 1.12) / (2 * FOV_TAN);
    const fitW = (rig.size.x * 1.35) / (2 * FOV_TAN * aspect);
    const dist = (Math.max(fitH, fitW) * (1 + 0.3 * explode)) / Math.max(state.zoom, 0.1);
    cam.position.set(0, 0, dist);
    cam.lookAt(0, 0, 0);

    tendons.update(hand, clamp01(state.tendons));

    // Labels: visible while labels > 0 and their joint is bent; written straight to the DOM.
    const labels = clamp01(state.labels);
    rig.labels.forEach((label, i) => {
      const el = labelEls.current[i];
      if (!el) return;
      const bend = Math.max(...label.ids.map((id) => clamp01(state.joints[id])));
      const v = labels * MathUtils.smoothstep(bend, 0.03, 0.14);
      if (Math.abs((labelShown.current[i] ?? -1) - v) < 0.004) return;
      labelShown.current[i] = v;
      el.style.opacity = v.toFixed(3);
      el.style.visibility = v > 0 ? "visible" : "hidden";
      el.style.setProperty("--pop", (0.8 + 0.2 * v).toFixed(3));
    });
  }, -1);

  return (
    <>
      <group ref={turntable}>
        <primitive object={rig.root} />
        {tendons.group.map((m) => (
          <primitive key={m.uuid} object={m} />
        ))}
      </group>
      {rig.labels.map((label, i) => (
        <JointLabel key={i} label={label} side={sides[i]} setRef={(el) => void (labelEls.current[i] = el)} />
      ))}
    </>
  );
}

/**
 * A pill label pinned to a joint (drei <Html>, so the text is crisp DOM styled
 * with the design tokens). Decorative: the page copy explains the joints.
 */
function JointLabel({ label, side, setRef }: { label: RigLabel; side: 1 | -1; setRef: (el: HTMLDivElement | null) => void }) {
  const right = side === 1;
  return createPortal(
    <Html pointerEvents="none" zIndexRange={[2, 0]}>
      <div
        ref={setRef}
        aria-hidden="true"
        style={{
          display: "flex",
          flexDirection: right ? "row" : "row-reverse",
          alignItems: "center",
          transform: right ? "translate(-4px, -50%)" : "translate(calc(-100% + 4px), -50%)",
          opacity: 0,
          visibility: "hidden",
          pointerEvents: "none",
          userSelect: "none",
        }}
      >
        <span style={{ width: 8, height: 8, flex: "none", borderRadius: "50%", background: "var(--text)", boxShadow: "0 0 0 2px var(--surface)" }} />
        <span style={{ width: 14, height: 1, flex: "none", background: "var(--border-strong)" }} />
        <span
          style={{
            transform: "scale(var(--pop, 1))",
            transformOrigin: right ? "left center" : "right center",
            padding: "3px 9px",
            borderRadius: "var(--radius-pill)",
            background: "var(--surface)",
            color: "var(--text)",
            boxShadow: "var(--shadow-sm)",
            border: "1px solid var(--border)",
            font: "600 11px/1.3 var(--font-sans)",
            letterSpacing: "0.02em",
            whiteSpace: "nowrap",
          }}
        >
          {label.text}
        </span>
      </div>
    </Html>,
    label.anchor,
  );
}
