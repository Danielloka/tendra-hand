"use client";

import Link from "next/link";
import { useEffect } from "react";
import { siteConfig } from "@/lib/site";
import { lockScroll } from "./scrollLock";

const BUTTON_ID = "menu-button";
const SHEET_ID = "menu-sheet";

/** `/docs` is current on /docs and every /docs/… page. */
export const isCurrent = (pathname: string, href: string) => pathname === href || pathname.startsWith(`${href}/`);

/** Menu / close button, shown below the 64rem nav breakpoint. */
export function MenuButton({ open, onToggle }: { open: boolean; onToggle: () => void }) {
  return (
    <button id={BUTTON_ID} type="button" className="icon-btn menu-button" aria-expanded={open} aria-controls={SHEET_ID} aria-label="Menu" onClick={onToggle}>
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" aria-hidden="true">
        <path className="menu-button__top" d="M4 8h16" />
        <path className="menu-button__bottom" d="M4 16h16" />
      </svg>
    </button>
  );
}

/**
 * Full-width sheet under the nav with the main links. While open: page scroll
 * is locked, the rest of the page is inert, Esc closes it (focus goes back to
 * the button). Closed, it is inert so its links can't be tabbed to.
 */
export function MobileMenu({ open, pathname, onClose }: { open: boolean; pathname: string; onClose: () => void }) {
  useEffect(() => {
    if (!open) return;
    const unlock = lockScroll();
    const behind = document.querySelectorAll<HTMLElement>("#main, #site-footer");
    behind.forEach((el) => (el.inert = true));

    const onKey = (e: KeyboardEvent) => {
      if (e.key !== "Escape") return;
      onClose();
      document.getElementById(BUTTON_ID)?.focus();
    };
    // Growing past the breakpoint hides the sheet, so close it too.
    const wide = window.matchMedia("(min-width: 64rem)");
    const onWide = () => wide.matches && onClose();
    document.addEventListener("keydown", onKey);
    wide.addEventListener("change", onWide);
    return () => {
      unlock();
      behind.forEach((el) => (el.inert = false));
      document.removeEventListener("keydown", onKey);
      wide.removeEventListener("change", onWide);
    };
  }, [open, onClose]);

  return (
    <div id={SHEET_ID} className="menu-sheet" data-open={open ? "" : undefined} inert={!open} data-lenis-prevent="">
      <nav aria-label="Menu" className="container menu-sheet__inner">
        <ul className="menu-sheet__links" role="list">
          {siteConfig.nav.map((item, i) => (
            <li key={item.href} style={{ "--i": i } as React.CSSProperties}>
              <Link className="menu-sheet__link" href={item.href} aria-current={isCurrent(pathname, item.href) ? "page" : undefined} onClick={onClose}>
                {item.label}
              </Link>
            </li>
          ))}
        </ul>
        <div className="menu-sheet__footer" style={{ "--i": siteConfig.nav.length } as React.CSSProperties}>
          <a className="btn btn--secondary" href={siteConfig.github}>
            View on GitHub
          </a>
        </div>
      </nav>
    </div>
  );
}
