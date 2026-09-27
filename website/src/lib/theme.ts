"use client";

import { useSyncExternalStore } from "react";

/** Light is the default; dark is opt-in via <html data-theme="dark">. Same storage key as the style guide. */
export type Theme = "light" | "dark";
const THEME_KEY = "tendra-theme";
const EVENT = "themechange";

export function getTheme(): Theme {
  return document.documentElement.dataset.theme === "dark" ? "dark" : "light";
}

export function setTheme(theme: Theme) {
  const root = document.documentElement;
  if (theme === "dark") root.dataset.theme = "dark";
  else delete root.dataset.theme;
  try {
    localStorage.setItem(THEME_KEY, theme);
  } catch {
    /* storage blocked: the choice just won't be remembered */
  }
  document.dispatchEvent(new CustomEvent(EVENT, { detail: { theme } }));
}

function subscribe(onChange: () => void) {
  document.addEventListener(EVENT, onChange);
  return () => document.removeEventListener(EVENT, onChange);
}

/** Current theme; re-renders on change. Server render assumes light. */
export function useTheme(): Theme {
  return useSyncExternalStore(subscribe, getTheme, () => "light");
}
