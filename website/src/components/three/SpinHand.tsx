"use client";

import { useGLTF } from "@react-three/drei";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { useEffect, useMemo, useRef, useState } from "react";
import {
  type Group,
  type Mesh,
  NeutralToneMapping,
  PMREMGenerator,
} from "three";
import { RoomEnvironment } from "three/examples/jsm/environments/RoomEnvironment.js";
import { useTheme } from "@/lib/theme";
import { HandMaterials, readPalette } from "./materials";

/** Static model of the full 5-finger Tendra Hand V1 (made by scripts/v1-to-glb.py). */
export const HAND_V1_URL = "/models/hand-v1.glb";

const SPIN = 0.5; // rad/s
const HEIGHT = 0.2055; // model height in metres

/** Pointer-drag state shared between the DOM handlers and the render loop. */
class Drag {
  active = false;
  private x = 0;
  private pending = 0;
  start(x: number) {
    this.active = true;
    this.x = x;
  }
  move(x: number) {
    if (!this.active) return;
    this.pending += x - this.x;
    this.x = x;
  }
  stop() {
    this.active = false;
  }
  /** Sideways pixels dragged since the last call. */
  take() {
    const p = this.pending;
    this.pending = 0;
    return p;
  }
}

type Props = { spin?: boolean; className?: string };

/**
 * The full Tendra Hand V1 turning slowly on its own, like the homepage hand.
 * Drag sideways to turn it by hand. Client-only (load with next/dynamic, ssr: false);
 * stops rendering while off-screen. `spin={false}` = one still render (reduced motion).
 */
export default function SpinHand({ spin = true, className }: Props) {
  const box = useRef<HTMLDivElement>(null);
  const [visible, setVisible] = useState(true);
  const [drag] = useState(() => new Drag());

  useEffect(() => {
    const el = box.current;
    if (!el) return;
    const io = new IntersectionObserver(([e]) => setVisible(e.isIntersecting), {
      rootMargin: "100px",
    });
    io.observe(el);
    return () => io.disconnect();
  }, []);

  return (
    <div
      ref={box}
      className={className}
      style={{
        width: "100%",
        height: "100%",
        touchAction: "pan-y",
        cursor: "grab",
      }}
      onPointerDown={(e) => {
        drag.start(e.clientX);
        e.currentTarget.setPointerCapture(e.pointerId);
      }}
      onPointerMove={(e) => {
        drag.move(e.clientX);
      }}
      onPointerUp={() => drag.stop()}
      onPointerCancel={() => drag.stop()}
    >
      <Canvas
        frameloop={spin ? (visible ? "always" : "never") : "demand"}
        dpr={[1, 1.75]}
        gl={{ antialias: true, alpha: true }}
        camera={{ fov: 30, near: 0.01, far: 10, position: [0, 0, 0.5] }}
        onCreated={({ gl }) => {
          gl.toneMapping = NeutralToneMapping;
        }}
        aria-hidden="true"
      >
        <Scene spin={spin} drag={drag} />
      </Canvas>
    </div>
  );
}

function Scene({ spin, drag }: { spin: boolean; drag: Drag }) {
  const theme = useTheme();
  const { scene } = useGLTF(HAND_V1_URL);
  const turn = useRef<Group>(null);
  const invalidate = useThree((s) => s.invalidate);
  const mats = useMemo(() => {
    const m = new HandMaterials();
    m.rimUniforms.uRimStrength.value = 0.8; // calm blue edge glow, as on the homepage
    return m;
  }, []);

  useEffect(() => () => mats.dispose(), [mats]);
  useEffect(() => {
    mats.setPalette(readPalette());
    invalidate();
  }, [theme, mats, invalidate]);

  // Same white-PLA material as the homepage hand on every part.
  const model = useMemo(() => {
    const copy = scene.clone(true);
    copy.traverse((o) => {
      if ((o as Mesh).isMesh) (o as Mesh).material = mats.real;
    });
    return copy;
  }, [scene, mats]);

  // Soft studio reflections, no HDR download.
  const get = useThree((s) => s.get);
  useEffect(() => {
    const { gl, scene: threeScene } = get();
    const pmrem = new PMREMGenerator(gl);
    const room = new RoomEnvironment();
    const env = pmrem.fromScene(room, 0.04, 0.1, 100, { size: 64 }).texture;
    threeScene.environment = env;
    threeScene.environmentIntensity = 0.6;
    room.dispose();
    pmrem.dispose();
    invalidate();
    return () => {
      threeScene.environment = null;
      env.dispose();
    };
  }, [get, invalidate]);

  useFrame((_, dt) => {
    const g = turn.current;
    if (!g) return;
    const dx = drag.take();
    if (dx) g.rotation.y += dx * 0.012;
    else if (spin && !drag.active) g.rotation.y += Math.min(dt, 0.05) * SPIN;
  });

  return (
    <>
      <directionalLight position={[-0.5, 0.8, 1]} intensity={1.7} />
      <directionalLight position={[0.8, -0.1, 0.6]} intensity={0.4} />
      <directionalLight
        position={[0, 0.4, -1]}
        intensity={1.2}
        color={mats.accent}
      />
      <group
        ref={turn}
        rotation={[0, spin ? 0 : -0.6, 0]}
        position={[0, -HEIGHT / 2, 0]}
      >
        <primitive object={model} />
      </group>
    </>
  );
}
