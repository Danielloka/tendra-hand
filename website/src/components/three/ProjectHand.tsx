"use client";

import dynamic from "next/dynamic";
import { useEffect, useState } from "react";
import { home } from "@/components/home/content";
import { setMotionOptIn, useStoryMotion } from "@/lib/scroll/motion";

// three.js is only fetched once the hand is near the viewport, never on the critical path.
const SpinHand = dynamic(() => import("./SpinHand"), { ssr: false, loading: () => null });

/**
 * The spinning 5-finger hand for the project page. It follows the site's motion switch
 * (same as the homepage): still when the system asks for reduced motion, with a
 * "Play animation" button that turns the spin (and the page's animations) on.
 */
export function ProjectHand({ className }: { className?: string }) {
  const spin = useStoryMotion();
  const [near, setNear] = useState(false);
  const [awake, setAwake] = useState(false);
  const [box, setBox] = useState<HTMLDivElement | null>(null);

  // Mount only after the first interaction (or 10 s), and once the box is near the viewport.
  // Parsing three.js + a 1.6 MB model on load costs ~40 Lighthouse points (same rule as the homepage).
  // Like the homepage: load three.js on the first interaction (or after 4 s), not on the critical path.
  useEffect(() => {
    const events = ["pointermove", "pointerdown", "touchstart", "wheel", "keydown", "scroll"] as const;
    const wake = () => {
      window.clearTimeout(timer);
      events.forEach((e) => window.removeEventListener(e, wake));
      setAwake(true);
    };
    const timer = window.setTimeout(wake, 4000);
    events.forEach((e) => window.addEventListener(e, wake, { once: true, passive: true }));
    return () => {
      window.clearTimeout(timer);
      events.forEach((e) => window.removeEventListener(e, wake));
    };
  }, []);

  useEffect(() => {
    if (!box) return;
    const events = ["pointermove", "pointerdown", "touchstart", "wheel", "keydown", "scroll"] as const;
    let io: IntersectionObserver | undefined;
    let defer = 0;
    const show = () => {
      stop();
      defer = window.setTimeout(() => {
        io = new IntersectionObserver(([e]) => e.isIntersecting && (setNear(true), io?.disconnect()), { rootMargin: "200px" });
        io.observe(box);
      }, 50);
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
      io?.disconnect();
    };
  }, [box]);

  return (
    <div className={className}>
      <div ref={setBox} className="project-hand__stage" role="img" aria-label="3D model of the full Tendra Hand V1, with five fingers.">
        {near && awake && <SpinHand spin={spin} />}
      </div>
      {!spin && (
        <button type="button" className="btn btn--sm btn--secondary project-hand__play" onClick={() => setMotionOptIn(true)}>
          {home.motionToggle.play}
        </button>
      )}
    </div>
  );
}
