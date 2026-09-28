"use client";

import { setMotionOptIn } from "@/lib/scroll/motion";
import { useReducedMotion } from "@/lib/scroll/useReducedMotion";
import { home } from "./content";

/**
 * "Play animation" for visitors whose system asks for reduced motion (they get
 * the still hand by default). Gone once the story plays, and hidden for
 * everyone else. The choice is remembered (src/lib/scroll/motion.ts).
 */
export function MotionToggle({ motion }: { motion: boolean }) {
  const reduced = useReducedMotion();
  if (!reduced || motion) return null;
  const t = home.motionToggle;
  return (
    <div className="motion-toggle">
      <p className="motion-toggle__note">{t.note}</p>
      <button type="button" className="btn btn--sm btn--secondary" onClick={() => setMotionOptIn(true)}>
        {t.play}
      </button>
    </div>
  );
}
