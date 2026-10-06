"use client";

import dynamic from "next/dynamic";
import { useEffect, useState } from "react";

// three.js (~270 kB gzipped) is only fetched after the first interaction (see below), never on the critical path.
const HandCanvas = dynamic(() => import("@/components/three/HandCanvas"), { ssr: false, loading: () => null });

type Props = {
  canvasRef: React.Ref<HTMLDivElement>;
  indicatorRef: React.Ref<HTMLOListElement>;
  chapters: { id: string; label: string }[];
  /** False = one still render (reduced motion, see src/lib/scroll/motion.ts). */
  motion: boolean;
};

/**
 * The box the 3D hand lives in. Its size comes from CSS (home.css), so it is
 * reserved before the canvas loads and nothing shifts. Until the 3D hand is
 * ready the stage shows its soft glow.
 *
 * The canvas mounts on the visitor's first interaction (scroll, touch, mouse,
 * key), or after 10 s at the latest. Parsing three.js and building the scene
 * takes about a second on a mid-range phone, so doing it before the page is
 * interactive would make the page feel stuck (and cost ~30 Lighthouse points).
 */
export function HandStage({ canvasRef, indicatorRef, chapters, motion }: Props) {
  const [ready, setReady] = useState(false);

  useEffect(() => {
    const events = ["pointermove", "pointerdown", "touchstart", "wheel", "keydown", "scroll"] as const;
    let defer = 0;
    const show = () => {
      stop();
      // Yield once, so the interaction itself (e.g. the first scroll frame) stays snappy.
      defer = window.setTimeout(() => setReady(true), 50);
    };
    const timer = window.setTimeout(show, 10_000);
    const stop = () => {
      window.clearTimeout(timer);
      events.forEach((e) => window.removeEventListener(e, show));
    };
    events.forEach((e) => window.addEventListener(e, show, { once: true, passive: true }));
    return () => {
      stop();
      window.clearTimeout(defer);
    };
  }, []);

  return (
    <div className="story-stage">
      <div ref={canvasRef} className="story-canvas" aria-hidden="true">
        <div className="story-canvas__glow" />
        {ready && <HandCanvas mode={motion ? "animated" : "static"} className="story-canvas__gl" />}
      </div>
      <noscript>
        {/* Without JavaScript there is no 3D hand, so show a photo of the prototype instead. */}
        {/* eslint-disable-next-line @next/next/no-img-element -- next/image needs JS to load lazily */}
        <img
          src="/media/photos/2026-09-29-v0-prototype-top-view.jpg"
          alt="The thumb and index finger prototype seen from above, with the motor column and tendon spools below the palm."
          width={1125}
          height={2000}
          loading="lazy"
          className="story-stage__fallback"
        />
      </noscript>
      {/* Where you are in the story. Not interactive, so it stays neutral. */}
      <ol ref={indicatorRef} className="story-indicator" aria-hidden="true">
        {chapters.map((c) => (
          <li key={c.id} data-chapter={c.id}>
            {c.label}
          </li>
        ))}
      </ol>
    </div>
  );
}
