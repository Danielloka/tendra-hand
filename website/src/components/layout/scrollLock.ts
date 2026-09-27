"use client";

/**
 * Stops the page behind an overlay (mobile menu, lightbox) from scrolling.
 * `overflow: hidden` on <html> stops native scrolling; the overlay must also
 * carry `data-lenis-prevent` so Lenis (smooth scroll) ignores wheel and touch
 * events inside it. The scrollbar's width is kept as padding so the page
 * doesn't shift sideways.
 */
let locks = 0;

export function lockScroll(): () => void {
  const root = document.documentElement;
  if (locks++ === 0) {
    const bar = window.innerWidth - root.clientWidth;
    root.style.overflow = "hidden";
    if (bar > 0) root.style.paddingRight = `${bar}px`;
  }
  let released = false;
  return () => {
    if (released) return;
    released = true;
    if (--locks === 0) {
      root.style.overflow = "";
      root.style.paddingRight = "";
    }
  };
}
