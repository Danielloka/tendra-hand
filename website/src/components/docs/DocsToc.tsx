"use client";

import { useEffect, useState } from "react";
import type { TocItem } from "@/lib/docs";

/** Highlights the section currently at the top of the viewport. */
function useActiveHeading(idList: string, enabled: boolean) {
  const [active, setActive] = useState<string | null>(null);

  useEffect(() => {
    if (!enabled) return;
    const els = idList
      .split(" ")
      .map((id) => document.getElementById(id)).filter((el): el is HTMLElement => el !== null);
    if (!els.length) return;
    const visible = new Set<string>();
    const io = new IntersectionObserver(
      (entries) => {
        for (const e of entries) {
          if (e.isIntersecting) visible.add(e.target.id);
          else visible.delete(e.target.id);
        }
        // First heading inside the band wins; if none is, keep the last one we passed.
        const inBand = els.find((el) => visible.has(el.id));
        if (inBand) setActive(inBand.id);
        else {
          const passed = els.filter((el) => el.getBoundingClientRect().top < 120);
          setActive(passed.at(-1)?.id ?? null);
        }
      },
      // A band from just under the sticky nav to 35% down the screen.
      { rootMargin: "-72px 0px -65% 0px" },
    );
    els.forEach((el) => io.observe(el));
    return () => io.disconnect();
  }, [idList, enabled]);

  return active;
}

function TocList({ items, active }: { items: TocItem[]; active: string | null }) {
  return (
    <ul role="list" className="toc__list">
      {items.map((item) => (
        <li key={item.id} className={item.depth === 3 ? "toc__item toc__item--sub" : "toc__item"}>
          <a href={`#${item.id}`} className="toc__link" aria-current={item.id === active ? "location" : undefined}>
            {item.text}
          </a>
        </li>
      ))}
    </ul>
  );
}

/**
 * "On this page". `aside`: sticky column on wide screens with scroll-spy.
 * `inline`: a collapsible list at the top of the article on smaller screens.
 */
export function DocsToc({ items, variant = "aside" }: { items: TocItem[]; variant?: "aside" | "inline" }) {
  const active = useActiveHeading(items.map((i) => i.id).join(" "), variant === "aside");
  if (items.length < 2) return null;

  if (variant === "inline") {
    return (
      <details className="toc toc--inline">
        <summary>On this page</summary>
        <TocList items={items} active={null} />
      </details>
    );
  }
  return (
    <nav className="toc toc--aside" aria-label="On this page" data-lenis-prevent="">
      <p className="toc__title">On this page</p>
      <TocList items={items} active={active} />
    </nav>
  );
}
