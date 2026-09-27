"use client";

import Image from "next/image";
import { useCallback, useEffect, useRef, useState } from "react";
import { lockScroll } from "@/components/layout/scrollLock";
import { isPlaceholder } from "./format";
import "./pages.css";

export type GalleryItem = {
  type: "image" | "video";
  src: string;
  poster?: string;
  alt: string;
  caption: string;
  width: number;
  height: number;
  todo?: string;
};

// A "video" whose src is still an SVG stand-in shows the picture with a badge instead of a player.
const isVideoPlaceholder = (item: GalleryItem) => item.type === "video" && item.src.endsWith(".svg");
const thumbOf = (item: GalleryItem) => (item.type === "video" ? (item.poster ?? item.src) : item.src);

function Icon({ d }: { d: string }) {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d={d} />
    </svg>
  );
}
const PREV = "m15 5-7 7 7 7";
const NEXT = "m9 5 7 7-7 7";
const CLOSE = "M6 6l12 12M18 6 6 18";

/** `placeholder`: sits below the centre so it doesn't cover the placeholder's own label. */
function PlayBadge({ label, placeholder = false }: { label: string; placeholder?: boolean }) {
  return (
    <span className={placeholder ? "play-badge play-badge--low" : "play-badge"}>
      <span className="play-badge__icon">
        <svg viewBox="0 0 10 12" fill="currentColor" aria-hidden="true">
          <path d="M0 0v12l10-6z" />
        </svg>
      </span>
      {label}
    </span>
  );
}

/**
 * Masonry grid of content/data/gallery.json with an accessible lightbox
 * (native <dialog>: focus stays inside, Esc closes and focus returns to the
 * picture that opened it, ←/→ move between items). Without JS, each tile is a
 * plain link to the file.
 */
export function Gallery({ items }: { items: GalleryItem[] }) {
  const [index, setIndex] = useState<number | null>(null);
  const dialogRef = useRef<HTMLDialogElement>(null);
  const triggers = useRef<(HTMLAnchorElement | null)[]>([]);
  const lastIndex = useRef(0);
  const open = index !== null;

  const go = useCallback((step: number) => setIndex((i) => (i === null ? i : (i + step + items.length) % items.length)), [items.length]);

  useEffect(() => {
    const dialog = dialogRef.current;
    if (!open || !dialog) return;
    if (!dialog.open) dialog.showModal();
    const unlock = lockScroll();
    const tiles = triggers.current;
    return () => {
      unlock();
      if (dialog.open) dialog.close();
      tiles[lastIndex.current]?.focus({ preventScroll: true });
    };
  }, [open]);

  // Remember the item on screen, so closing returns focus to its tile.
  useEffect(() => {
    if (index !== null) lastIndex.current = index;
  }, [index]);

  const onKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === "ArrowLeft") go(-1);
    else if (e.key === "ArrowRight") go(1);
    else return;
    e.preventDefault();
  };

  const item = index === null ? null : items[index];

  return (
    <>
      <ul className="gallery" role="list">
        {items.map((it, i) => (
          <li key={it.src} className="gallery__item">
            <figure className="gallery__figure">
              <a
                ref={(el) => {
                  triggers.current[i] = el;
                }}
                className="gallery__link"
                href={it.src}
                aria-label={`${it.type === "video" ? "Play video" : "Open picture"}: ${it.alt}`}
                onClick={(e) => {
                  e.preventDefault();
                  setIndex(i);
                }}
              >
                <Image src={thumbOf(it)} alt={it.alt} width={it.width} height={it.height} sizes="(min-width: 72rem) 22rem, (min-width: 40rem) 45vw, 100vw" />
                {/* TODO: placeholder media; replace it as described in the item's `todo` field (content/data/gallery.json). */}
                {isPlaceholder(it.src) && <span className="tag media-flag">Placeholder</span>}
                {it.type === "video" && <PlayBadge label={isVideoPlaceholder(it) ? "Video coming" : "Play"} placeholder={isVideoPlaceholder(it)} />}
              </a>
              <figcaption className="caption">{it.caption}</figcaption>
            </figure>
          </li>
        ))}
      </ul>

      <dialog
        ref={dialogRef}
        className="lightbox"
        aria-label="Gallery viewer"
        data-lenis-prevent=""
        onClose={() => setIndex(null)}
        onKeyDown={onKeyDown}
        onClick={(e) => {
          // A click on the empty area around the picture closes the viewer.
          const t = e.target as HTMLElement;
          if (t === e.currentTarget || t.classList.contains("lightbox__media") || t.classList.contains("lightbox__stage")) setIndex(null);
        }}
      >
        {item && (
          <div className="lightbox__frame">
            <div className="lightbox__bar">
              <p className="lightbox__counter" aria-live="polite">
                {index! + 1} / {items.length}
                <span className="visually-hidden">: {item.alt}</span>
              </p>
              <button type="button" className="icon-btn lightbox__btn" aria-label="Close" onClick={() => setIndex(null)} autoFocus>
                <Icon d={CLOSE} />
              </button>
            </div>

            <div className="lightbox__stage">
              <button type="button" className="icon-btn lightbox__btn lightbox__nav" aria-label="Previous" onClick={() => go(-1)}>
                <Icon d={PREV} />
              </button>
              <div className="lightbox__media">
                <div className="lightbox__media-inner" key={index}>
                  {item.type === "video" && !isVideoPlaceholder(item) ? (
                    <video src={item.src} poster={item.poster} controls playsInline preload="none" aria-label={item.alt} width={item.width} height={item.height} />
                  ) : (
                    <>
                      <Image src={thumbOf(item)} alt={item.alt} width={item.width} height={item.height} sizes="90vw" />
                      {isVideoPlaceholder(item) && <PlayBadge label="Video coming" placeholder />}
                      {isPlaceholder(item.src) && <span className="tag media-flag">Placeholder</span>}
                    </>
                  )}
                </div>
              </div>
              <button type="button" className="icon-btn lightbox__btn lightbox__nav" aria-label="Next" onClick={() => go(1)}>
                <Icon d={NEXT} />
              </button>
            </div>

            <div>
              <p className="lightbox__caption">{item.caption}</p>
              <div className="lightbox__phone-nav">
                <button type="button" className="icon-btn lightbox__btn" aria-label="Previous" onClick={() => go(-1)}>
                  <Icon d={PREV} />
                </button>
                <button type="button" className="icon-btn lightbox__btn" aria-label="Next" onClick={() => go(1)}>
                  <Icon d={NEXT} />
                </button>
              </div>
            </div>
          </div>
        )}
      </dialog>
    </>
  );
}
