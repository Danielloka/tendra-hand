"use client";

import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Suspense, useEffect, useMemo, useRef, useState } from "react";
import { type DirectionalLight, NeutralToneMapping, PMREMGenerator, type Scene as Scene3D, type WebGLRenderer } from "three";
import { RoomEnvironment } from "three/examples/jsm/environments/RoomEnvironment.js";
import { type HandState, handState } from "@/lib/handState";
import { useTheme } from "@/lib/theme";
import { HandModel } from "./HandModel";
import { onIdle } from "./idle";
import { HandMaterials, readPalette } from "./materials";

type Props = {
  /** The numbers to render; defaults to the shared homepage instance. */
  state?: HandState;
  /** "static" renders the current state once (no idle motion), e.g. for reduced motion. */
  mode?: "animated" | "static";
  className?: string;
};

/**
 * The 3D hand. Client-only: load it with next/dynamic(..., { ssr: false }).
 * Fills its parent box, has a transparent background, and stops rendering
 * while it is off-screen or the tab is hidden.
 */
export default function HandCanvas({ state = handState, mode = "animated", className }: Props) {
  const box = useRef<HTMLDivElement>(null);
  const [onScreen, setOnScreen] = useState(true);
  const [tabVisible, setTabVisible] = useState(true);

  useEffect(() => {
    const el = box.current;
    if (!el) return;
    const io = new IntersectionObserver(([entry]) => setOnScreen(entry.isIntersecting), { rootMargin: "100px" });
    io.observe(el);
    return () => io.disconnect();
  }, []);

  useEffect(() => {
    const update = () => setTabVisible(!document.hidden);
    update();
    document.addEventListener("visibilitychange", update);
    return () => document.removeEventListener("visibilitychange", update);
  }, []);

  const frameloop = mode === "static" ? "demand" : onScreen && tabVisible ? "always" : "never";

  return (
    <div ref={box} className={className} style={{ position: "relative", width: "100%", height: "100%" }}>
      <Canvas
        frameloop={frameloop}
        dpr={[1, 1.75]}
        gl={{ antialias: true, alpha: true, powerPreference: "high-performance" }}
        camera={{ fov: 30, near: 0.01, far: 10, position: [0, 0, 0.9] }}
        onCreated={({ gl }) => {
          gl.toneMapping = NeutralToneMapping;
        }}
        aria-hidden="true"
      >
        <Scene state={state} animated={mode === "animated"} />
      </Canvas>
    </div>
  );
}

function Scene({ state, animated }: { state: HandState; animated: boolean }) {
  const theme = useTheme();
  const invalidate = useThree((s) => s.invalidate);
  const get = useThree((s) => s.get);
  const mats = useMemo(() => new HandMaterials(), []);
  const [ready, setReady] = useState(false);
  useEffect(() => () => mats.dispose(), [mats]);

  // Colours come from the CSS tokens; re-read them whenever the theme flips.
  useEffect(() => {
    mats.setPalette(readPalette());
    invalidate();
  }, [theme, mats, invalidate]);

  // Startup work is spread out so no single frame blocks the page: first the
  // reflections (idle time), then the shaders, compiled in parallel where the
  // browser supports it. The hand stays hidden until both are done.
  useEffect(() => {
    let cancelled = false;
    const disposeEnv: { current?: () => void } = {};
    const cancel = onIdle(() => {
      const { gl, scene, camera } = get();
      disposeEnv.current = addStudioEnvironment(gl, scene);
      gl.compileAsync(scene, camera)
        .catch(() => {}) // fall back to compiling on first draw
        .then(() => {
          if (cancelled) return;
          setReady(true);
          invalidate();
        });
    });
    return () => {
      cancelled = true;
      cancel();
      disposeEnv.current?.();
    };
  }, [get, invalidate]);

  return (
    <>
      <Lights state={state} mats={mats} />
      <group visible={ready}>
        <Suspense fallback={null}>
          <HandModel state={state} mats={mats} animated={animated} />
        </Suspense>
      </group>
    </>
  );
}

/** Soft studio reflections from three's built-in room (no HDR download). Returns a cleanup. */
function addStudioEnvironment(gl: WebGLRenderer, scene: Scene3D): () => void {
  const pmrem = new PMREMGenerator(gl);
  const room = new RoomEnvironment();
  // 64 px is plenty for soft reflections on matte plastic, and ~16× cheaper than the default 256.
  const env = pmrem.fromScene(room, 0.04, 0.1, 100, { size: 64 }).texture;
  scene.environment = env;
  scene.environmentIntensity = 0.55;
  room.dispose();
  pmrem.dispose();
  return () => {
    scene.environment = null;
    env.dispose();
  };
}

/** Key + fill for a clean product look; two accent back lights scale with `rim`. */
function Lights({ state, mats }: { state: HandState; mats: HandMaterials }) {
  const rimA = useRef<DirectionalLight>(null);
  const rimB = useRef<DirectionalLight>(null);
  useFrame(() => {
    const rim = Math.min(Math.max(state.rim, 0), 1);
    for (const l of [rimA.current, rimB.current]) {
      if (!l) continue;
      l.color.copy(mats.accent);
      l.intensity = rim * 2.4;
    }
  });
  return (
    <>
      <directionalLight position={[-0.5, 0.8, 1]} intensity={1.7} />
      <directionalLight position={[0.8, -0.1, 0.6]} intensity={0.35} />
      <directionalLight ref={rimA} position={[-0.9, 0.5, -1]} intensity={0} />
      <directionalLight ref={rimB} position={[0.9, 0.3, -1]} intensity={0} />
    </>
  );
}
