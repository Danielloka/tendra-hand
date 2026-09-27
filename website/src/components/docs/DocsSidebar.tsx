"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { useState } from "react";
import type { DocGroup } from "@/lib/docs";
import { DocsSearch } from "./DocsSearch";

/**
 * Docs sidebar: search + page list. Below 1024px the list folds into a
 * "Docs menu" button (open by default without JS, so nothing is lost).
 */
export function DocsSidebar({ groups }: { groups: DocGroup[] }) {
  const pathname = usePathname();
  // Remember which page the menu was opened on, so it closes by itself after navigating.
  const [openOn, setOpenOn] = useState<string | null>(null);
  const open = openOn === pathname;
  const current = groups.flatMap((g) => g.pages).find((p) => p.href === pathname);

  return (
    <div className="docs-sidebar__inner">
      <DocsSearch />
      <button type="button" className="docs-menu-btn" aria-expanded={open} aria-controls="docs-nav" onClick={() => setOpenOn(open ? null : pathname)}>
        <span>
          <span className="docs-menu-btn__label">Docs menu</span>
          {current && <span className="docs-menu-btn__current">{current.title}</span>}
        </span>
        <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
          <path d="m6 9 6 6 6-6" />
        </svg>
      </button>
      <nav id="docs-nav" className="docs-nav" aria-label="Docs" data-open={open ? "" : undefined} data-lenis-prevent="">
        <Link href="/docs" className="docs-nav__link docs-nav__home" aria-current={pathname === "/docs" ? "page" : undefined}>
          Overview
        </Link>
        {groups.map((group) => (
          <div key={group.title} className="docs-nav__group">
            <p className="docs-nav__title">{group.title}</p>
            <ul role="list">
              {group.pages.map((page) => (
                <li key={page.slug}>
                  <Link href={page.href} className="docs-nav__link" aria-current={page.href === pathname ? "page" : undefined}>
                    {page.title}
                  </Link>
                </li>
              ))}
            </ul>
          </div>
        ))}
      </nav>
    </div>
  );
}
