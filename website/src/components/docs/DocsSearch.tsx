"use client";

import { useRouter } from "next/navigation";
import { useEffect, useId, useMemo, useRef, useState } from "react";
import type { SearchDoc } from "@/lib/docs";
import { search, type SearchHit } from "./search";

let indexPromise: Promise<SearchDoc[]> | null = null;
/** The index is fetched once, on first focus (kept out of every page's HTML). */
function loadIndex() {
  indexPromise ??= fetch("/docs/search-index.json")
    .then((r) => (r.ok ? r.json() : Promise.reject(new Error(String(r.status)))))
    .catch((err) => {
      indexPromise = null; // allow a retry
      throw err;
    });
  return indexPromise;
}

/**
 * Docs search box (ARIA combobox + listbox).
 * Keys: "/" or Ctrl/Cmd+K focus it, ↑/↓ move, Enter opens, Esc closes.
 */
export function DocsSearch() {
  const router = useRouter();
  const inputRef = useRef<HTMLInputElement>(null);
  const listId = useId();
  const [index, setIndex] = useState<SearchDoc[] | null>(null);
  const [failed, setFailed] = useState(false);
  const [query, setQuery] = useState("");
  const [open, setOpen] = useState(false);
  const [active, setActive] = useState(0);
  const [isMac, setIsMac] = useState(false);

  const hits = useMemo(() => (index ? search(index, query) : []), [index, query]);

  function prepare() {
    if (index) return;
    loadIndex().then(setIndex, () => setFailed(true));
  }

  // Global shortcuts. Also pick the right shortcut hint for the platform.
  useEffect(() => {
    // eslint-disable-next-line react-hooks/set-state-in-effect -- platform is only known on the client
    setIsMac(/Mac|iPhone|iPad/.test(navigator.platform));
    function onKey(e: KeyboardEvent) {
      const target = e.target as HTMLElement;
      const typing = target.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(target.tagName);
      if ((e.key === "k" && (e.metaKey || e.ctrlKey)) || (e.key === "/" && !typing)) {
        e.preventDefault();
        inputRef.current?.focus();
        inputRef.current?.select();
      }
    }
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  function go(hit: SearchHit) {
    setOpen(false);
    setQuery("");
    inputRef.current?.blur();
    router.push(hit.href);
  }

  function onKeyDown(e: React.KeyboardEvent<HTMLInputElement>) {
    if (e.key === "ArrowDown" || e.key === "ArrowUp") {
      e.preventDefault();
      setOpen(true);
      if (hits.length) setActive((a) => (a + (e.key === "ArrowDown" ? 1 : hits.length - 1)) % hits.length);
    } else if (e.key === "Enter" && open && hits[active]) {
      e.preventDefault();
      go(hits[active]);
    } else if (e.key === "Escape") {
      if (open && query) setOpen(false);
      else {
        setQuery("");
        inputRef.current?.blur();
      }
    }
  }

  const showList = open && query.trim().length > 0;
  const optionId = (i: number) => `${listId}-opt-${i}`;

  return (
    <div className="docs-search" role="search">
      <label htmlFor={`${listId}-input`} className="visually-hidden">
        Search the docs
      </label>
      <svg className="docs-search__icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" aria-hidden="true">
        <circle cx="11" cy="11" r="7" />
        <path d="m20 20-3.5-3.5" />
      </svg>
      <input
        ref={inputRef}
        id={`${listId}-input`}
        className="docs-search__input"
        type="search"
        placeholder="Search docs"
        autoComplete="off"
        spellCheck={false}
        role="combobox"
        aria-expanded={showList}
        aria-controls={listId}
        aria-autocomplete="list"
        aria-activedescendant={showList && hits[active] ? optionId(active) : undefined}
        value={query}
        onFocus={() => {
          prepare();
          setOpen(true);
        }}
        onBlur={() => setOpen(false)}
        onChange={(e) => {
          setQuery(e.target.value);
          setActive(0);
          setOpen(true);
        }}
        onKeyDown={onKeyDown}
      />
      <kbd className="docs-search__kbd" aria-hidden="true">
        {isMac ? "⌘K" : "Ctrl K"}
      </kbd>

      <div className="docs-search__panel" hidden={!showList} data-lenis-prevent="">
        <ul id={listId} role="listbox" aria-label="Search results" className="docs-search__list">
          {hits.map((hit, i) => (
            <li
              key={hit.href}
              id={optionId(i)}
              role="option"
              aria-selected={i === active}
              className="docs-search__hit"
              // mousedown (not click) so the input's blur doesn't close the list first
              onMouseDown={(e) => {
                e.preventDefault();
                go(hit);
              }}
              onMouseMove={() => setActive(i)}
            >
              <span className="docs-search__title">
                {hit.title}
                {hit.heading && <span className="docs-search__heading"> › {hit.heading}</span>}
              </span>
              {hit.snippet.length > 0 && (
                <span className="docs-search__snippet">
                  {hit.snippet.map((part, j) => (j % 2 ? <mark key={j}>{part}</mark> : part))}
                </span>
              )}
            </li>
          ))}
        </ul>
        {hits.length === 0 && (
          <p className="docs-search__empty" role="status">
            {failed ? "Search couldn't load. Check your connection and try again." : index ? `No results for “${query.trim()}”.` : "Loading…"}
          </p>
        )}
      </div>
    </div>
  );
}
