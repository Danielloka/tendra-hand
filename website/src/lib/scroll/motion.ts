"use client";

import { useSyncExternalStore } from "react";

/*
 * Whether the homepage scroll story animates.
 *
 * On by default; off when the visitor's system asks for reduced motion
 * (prefers-reduced-motion: reduce). Those visitors can still switch it back on
 * with the "Play animation" button; that choice is remembered in localStorage.
 *
 * The result is mirrored as the `motion` class on <html> (set before first
 * paint by the boot script in app/layout.tsx), which the story CSS keys on.
 */

export const MOTION_KEY = "tendra-motion";
const REDUCE = "(prefers-reduced-motion: reduce)";

const listeners = new Set<() => void>();

function optedIn(): boolean {
  try {
    return localStorage.getItem(MOTION_KEY) === "on";
  } catch {
    return false;
  }
}

function storyMotion(): boolean {
  return !window.matchMedia(REDUCE).matches || optedIn();
}

function apply() {
  document.documentElement.classList.toggle("motion", storyMotion());
  listeners.forEach((fn) => fn());
}

/** Turns the story animation on or off for a visitor whose system asks for reduced motion. */
export function setMotionOptIn(on: boolean) {
  try {
    if (on) localStorage.setItem(MOTION_KEY, "on");
    else localStorage.removeItem(MOTION_KEY);
  } catch {
    // Storage blocked: the choice still applies until the page is reloaded.
    document.documentElement.classList.toggle("motion", on);
    listeners.forEach((fn) => fn());
    return;
  }
  apply();
}

function subscribe(onChange: () => void) {
  listeners.add(onChange);
  const media = window.matchMedia(REDUCE);
  media.addEventListener("change", apply);
  return () => {
    listeners.delete(onChange);
    media.removeEventListener("change", apply);
  };
}

/** True while the homepage story animates (reads the <html> class). Server render assumes false. */
export function useStoryMotion(): boolean {
  return useSyncExternalStore(
    subscribe,
    () => document.documentElement.classList.contains("motion"),
    () => false,
  );
}
