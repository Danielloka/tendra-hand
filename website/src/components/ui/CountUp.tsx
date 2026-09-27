"use client";

import { useEffect, useRef } from "react";

/**
 * Counts from 0 to `to` when scrolled into view. The final value is in the
 * server HTML, so it's correct without JS and with reduced motion.
 */
export function CountUp({ to, duration = 1400 }: { to: number; duration?: number }) {
  const ref = useRef<HTMLSpanElement>(null);

  useEffect(() => {
    const el = ref.current;
    if (!el || window.matchMedia("(prefers-reduced-motion: reduce)").matches) return;
    const box = el.getBoundingClientRect();
    if (box.top < window.innerHeight && box.bottom > 0) return; // already on screen: don't flash to 0
    el.textContent = "0";
    let raf = 0;
    const io = new IntersectionObserver(([entry]) => {
      if (!entry.isIntersecting) return;
      io.disconnect();
      const start = performance.now();
      const tick = (now: number) => {
        const t = Math.min(1, (now - start) / duration);
        el.textContent = String(Math.round(to * (1 - Math.pow(1 - t, 3))));
        if (t < 1) raf = requestAnimationFrame(tick);
      };
      raf = requestAnimationFrame(tick);
    });
    io.observe(el);
    return () => {
      io.disconnect();
      cancelAnimationFrame(raf);
      el.textContent = String(to);
    };
  }, [to, duration]);

  return (
    <span className="tabular">
      <span className="visually-hidden">{to}</span>
      <span ref={ref} aria-hidden="true">
        {to}
      </span>
    </span>
  );
}
