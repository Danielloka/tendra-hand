"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { useCallback, useEffect, useState } from "react";
import { siteConfig, type NavItem } from "@/lib/site";
import { isCurrentIn, MenuButton, MobileMenu, stripDocs } from "./MobileMenu";
import { ScrollProgress } from "./ScrollProgress";
import { ThemeToggle } from "./ThemeToggle";
import "./layout.css";

// Long reading pages get a progress line under the nav (the homepage tells its own story).
const PROGRESS_ROUTES = ["/project", "/journey", "/contribute"];
const hasProgress = (pathname: string, docs: boolean) => (docs ? stripDocs(pathname) !== "/" : PROGRESS_ROUTES.includes(pathname));

/** `docs` = the docs header: brand links to the docs home and gets a "Docs" tag. */
export function Nav({ items, docs = false }: { items: NavItem[]; docs?: boolean }) {
  const pathname = usePathname();
  const [scrolled, setScrolled] = useState(false);
  // The menu remembers the page it was opened on, so it closes by itself on navigation.
  const [menuPath, setMenuPath] = useState<string | null>(null);
  const menuOpen = menuPath === pathname;
  const closeMenu = useCallback(() => setMenuPath(null), []);

  useEffect(() => {
    const onScroll = () => setScrolled(window.scrollY > 8);
    onScroll();
    window.addEventListener("scroll", onScroll, { passive: true });
    return () => window.removeEventListener("scroll", onScroll);
  }, []);

  return (
    <>
      <header className={`nav${scrolled || menuOpen ? " is-scrolled" : ""}${menuOpen ? " is-menu-open" : ""}`} data-lenis-prevent={menuOpen ? "" : undefined}>
        <div className="container nav__inner">
          <Link className="nav__brand" href={docs ? "/docs" : "/"} aria-label={docs ? undefined : `${siteConfig.name}, home`}>
            <span className="nav__brand-mark" aria-hidden="true" />
            {siteConfig.name}
            {docs && <span className="nav__brand-tag">Docs</span>}
          </Link>
          <nav aria-label="Main">
            <ul className="nav__links">
              {items.map((item) => (
                <li key={item.href}>
                  <Link className="nav__link" href={item.href} aria-current={isCurrentIn(docs, pathname, item.href) ? "page" : undefined}>
                    {item.label}
                  </Link>
                </li>
              ))}
            </ul>
          </nav>
          <div className="nav__actions">
            <ThemeToggle />
            <a className="btn btn--secondary btn--sm nav__github" href={siteConfig.github}>
              GitHub
            </a>
            <MenuButton open={menuOpen} onToggle={() => setMenuPath(menuOpen ? null : pathname)} />
          </div>
        </div>
        {hasProgress(pathname, docs) && <ScrollProgress key={pathname} />}
      </header>
      <MobileMenu open={menuOpen} pathname={pathname} onClose={closeMenu} items={items} docs={docs} />
    </>
  );
}
