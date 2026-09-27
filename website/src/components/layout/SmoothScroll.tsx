"use client";

import "lenis/dist/lenis.css";
import "@/lib/scroll/lenis.css";
import type Lenis from "lenis";
import { usePathname } from "next/navigation";
import { useEffect, useRef } from "react";
import { getLenis, setLenis } from "@/lib/scroll/lenis";

/**
 * Site-wide smooth scrolling with Lenis, driven by GSAP's ticker so the
 * homepage's ScrollTrigger animations stay in step with it.
 *
 * - Off entirely with `prefers-reduced-motion: reduce` (and it follows changes).
 * - Lenis and GSAP core load after hydration, so they're never on the critical
 *   path. ScrollTrigger is NOT loaded here; the homepage story loads it.
 * - Same-page `#hash` links scroll smoothly and honour `scroll-padding-top`.
 * - Route changes: inertia stops and the new page starts at the top (or at its #hash).
 * - Elements with `data-lenis-prevent` keep native scrolling, and `overflow: hidden`
 *   on <html> (scroll lock for menus/lightbox) pauses Lenis (`autoToggle`).
 */
export function SmoothScroll({ children }: { children: React.ReactNode }) {
  const pathname = usePathname();
  const popped = useRef(false);

  useEffect(() => {
    const media = window.matchMedia("(prefers-reduced-motion: reduce)");
    let alive = true;
    let stop: (() => void) | null = null;

    async function start() {
      const [{ default: LenisClass }, { gsap }] = await Promise.all([import("lenis"), import("gsap")]);
      if (!alive || media.matches || stop) return;
      const lenis: Lenis = new LenisClass({ autoRaf: false, lerp: 0.1, stopInertiaOnNavigate: true, autoToggle: true });
      const tick = (time: number) => lenis.raf(time * 1000);
      gsap.ticker.add(tick);
      gsap.ticker.lagSmoothing(0);
      setLenis(lenis);
      stop = () => {
        gsap.ticker.remove(tick);
        gsap.ticker.lagSmoothing(500, 33); // GSAP's default
        lenis.destroy();
        setLenis(null);
        stop = null;
      };
    }

    const onMotionChange = () => {
      if (media.matches) stop?.();
      else void start();
    };

    // Same-page hash links (including the skip link). Capture phase on window
    // runs before Next's <Link> handler, which then sees defaultPrevented.
    const onClick = (e: MouseEvent) => {
      const lenis = getLenis();
      if (!lenis || e.defaultPrevented || e.button !== 0 || e.metaKey || e.ctrlKey || e.shiftKey || e.altKey) return;
      const link = (e.target as Element | null)?.closest?.("a[href]");
      if (!(link instanceof HTMLAnchorElement) || (link.target && link.target !== "_self")) return;
      const url = new URL(link.href);
      if (url.origin !== location.origin || url.pathname !== location.pathname || !url.hash) return;
      const target = document.getElementById(decodeURIComponent(url.hash.slice(1)));
      if (!target) return;
      e.preventDefault();
      lenis.scrollTo(target); // Lenis subtracts scroll-padding-top / scroll-margin-top itself
      if (url.hash !== location.hash) history.pushState(null, "", url.hash);
      if (!target.matches("a, button, input, select, textarea, [tabindex]")) target.setAttribute("tabindex", "-1");
      target.focus({ preventScroll: true });
    };

    const onPop = () => {
      popped.current = true;
    };

    void start();
    media.addEventListener("change", onMotionChange);
    window.addEventListener("click", onClick, true);
    window.addEventListener("popstate", onPop);
    return () => {
      alive = false;
      stop?.();
      media.removeEventListener("change", onMotionChange);
      window.removeEventListener("click", onClick, true);
      window.removeEventListener("popstate", onPop);
    };
  }, []);

  // New page: drop any leftover inertia. Links start at the top (or their #hash);
  // back/forward keeps the browser's restored position.
  useEffect(() => {
    const lenis = getLenis();
    if (!lenis) return;
    const target = location.hash ? document.getElementById(decodeURIComponent(location.hash.slice(1))) : null;
    // An immediate scrollTo also resets Lenis' velocity and target.
    lenis.scrollTo(popped.current ? window.scrollY : (target ?? 0), { immediate: true, force: true });
    popped.current = false;
  }, [pathname]);

  return <>{children}</>;
}
