"use client";

import { useEffect, useRef } from "react";

/**
 * Thin accent line along the bottom edge of the nav showing how far the page
 * has been read. Updated once per frame from a passive scroll listener
 * (Lenis moves the native scroll position, so window.scrollY is correct).
 */
export function ScrollProgress() {
  const ref = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const bar = ref.current;
    if (!bar) return;
    let raf = 0;
    const update = () => {
      raf = 0;
      const max = document.documentElement.scrollHeight - window.innerHeight;
      bar.style.transform = `scaleX(${max > 0 ? Math.min(1, Math.max(0, window.scrollY / max)) : 0})`;
    };
    const schedule = () => {
      if (!raf) raf = requestAnimationFrame(update);
    };
    update();
    window.addEventListener("scroll", schedule, { passive: true });
    // The page grows as images load and sections expand: track its size too.
    const ro = new ResizeObserver(schedule);
    ro.observe(document.body);
    return () => {
      cancelAnimationFrame(raf);
      window.removeEventListener("scroll", schedule);
      ro.disconnect();
    };
  }, []);

  return <div ref={ref} className="scroll-progress" aria-hidden="true" />;
}
