import type Lenis from "lenis";

/*
 * The site-wide Lenis instance, shared without React state.
 * SmoothScroll (layout) creates and destroys it; the homepage story subscribes
 * so it can hook ScrollTrigger.update to Lenis' scroll event. `null` means
 * native scrolling (reduced motion, or Lenis not loaded yet).
 */

let current: Lenis | null = null;
const listeners = new Set<(lenis: Lenis | null) => void>();

export function getLenis(): Lenis | null {
  return current;
}

export function setLenis(lenis: Lenis | null) {
  current = lenis;
  listeners.forEach((fn) => fn(lenis));
}

/** Calls `fn` now and whenever the instance changes. Returns an unsubscribe function. */
export function onLenisChange(fn: (lenis: Lenis | null) => void): () => void {
  listeners.add(fn);
  fn(current);
  return () => {
    listeners.delete(fn);
  };
}

/** Height of the sticky nav + breathing room, read from `scroll-padding-top` on <html>. */
export function scrollPaddingTop(): number {
  const value = Number.parseFloat(getComputedStyle(document.documentElement).scrollPaddingTop);
  return Number.isNaN(value) ? 0 : value;
}
