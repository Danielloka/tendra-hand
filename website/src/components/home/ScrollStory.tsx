"use client";

import { useEffect, useRef } from "react";
import { handState } from "@/lib/handState";
import { onLenisChange } from "@/lib/scroll/lenis";
import { STORY_IDS, applyHandState, buildStoryTimeline, scrollToStoryTime, staticPose } from "@/lib/scroll/storyTimeline";
import { HandStage } from "./HandStage";

const DESKTOP = "(min-width: 1024px) and (prefers-reduced-motion: no-preference)";
const COMPACT = "(max-width: 1023.98px) and (prefers-reduced-motion: no-preference)";
const REDUCE = "(prefers-reduced-motion: reduce)";

type Props = {
  /** Server-rendered hero (sits over the canvas on desktop). */
  hero: React.ReactNode;
  /** Server-rendered StorySections, in STORY_IDS order. */
  children: React.ReactNode;
  chapters: { id: string; label: string }[];
};

/**
 * The homepage scroll story: hero + sticky hand canvas + story sections.
 *
 * GSAP and ScrollTrigger load after hydration (dynamic import) and only here.
 * gsap.matchMedia builds one timeline per layout and tears it down when the
 * breakpoint or the motion preference changes; everything is reverted on
 * unmount (route change). Scrolling writes to `handState` and one transform,
 * never to React state.
 */
export function ScrollStory({ hero, children, chapters }: Props) {
  const root = useRef<HTMLDivElement>(null);
  const canvas = useRef<HTMLDivElement>(null);
  const indicator = useRef<HTMLOListElement>(null);

  useEffect(() => {
    const el = root.current;
    if (!el) return;
    // Reduced motion needs no GSAP at all: one still pose for the static render.
    if (window.matchMedia(REDUCE).matches) applyHandState(handState, staticPose());

    let alive = true;
    let cleanup = () => {};

    void Promise.all([import("gsap"), import("gsap/ScrollTrigger")]).then(([{ gsap }, { ScrollTrigger }]) => {
      if (!alive) return;
      gsap.registerPlugin(ScrollTrigger);
      const sections = Array.from(el.querySelectorAll<HTMLElement>("[data-story-section]"));
      const diagram = el.querySelector<HTMLElement>("[data-diagram]");
      const mm = gsap.matchMedia();

      mm.add({ desktop: DESKTOP, compact: COMPACT, reduce: REDUCE }, (ctx) => {
        const { desktop, reduce } = ctx.conditions as Record<string, boolean>;
        if (reduce) {
          applyHandState(handState, staticPose());
          return;
        }

        const tl = buildStoryTimeline(gsap, {
          state: handState,
          layout: desktop ? "desktop" : "compact",
          canvas: canvas.current,
          indicator: indicator.current,
        });

        // Each section's scroll range, measured (and re-measured on resize) by ScrollTrigger.
        // A section "has the stage" while its top has passed the reading line.
        const line = desktop ? "55%" : "72%";
        const ranges = sections.map((section) => ScrollTrigger.create({ trigger: section, start: `top ${line}`, end: `bottom ${line}` }));
        const bounds = () => [0, ...ranges.map((r) => r.start), ranges[ranges.length - 1].end];

        let active = "";
        const markActive = (time: number) => {
          const id = STORY_IDS[Math.floor(time) - 1] ?? "";
          if (id === active || !indicator.current) return;
          active = id;
          indicator.current.querySelectorAll("li").forEach((li) => li.toggleAttribute("data-active", li.dataset.chapter === id));
        };

        // Scrub: ease the timeline towards the scroll position (like ScrollTrigger's `scrub: 0.7`).
        const drive = (immediate: boolean) => {
          const time = Math.min(scrollToStoryTime(window.scrollY, bounds()), tl.duration());
          markActive(time);
          if (immediate) {
            gsap.killTweensOf(tl);
            tl.time(time);
          } else {
            gsap.to(tl, { time, duration: 0.7, ease: "power3.out", overwrite: true });
          }
        };
        ScrollTrigger.create({ start: 0, end: "max", onUpdate: () => drive(false), onRefresh: () => drive(false) });
        drive(true);

        // Diagram: nodes appear and connectors draw on as it scrolls into view (done once fully visible).
        if (diagram) {
          const links = diagram.querySelectorAll("[data-diagram-link]");
          const dtl = gsap.timeline({ scrollTrigger: { trigger: diagram, start: "top 90%", end: "bottom 90%", scrub: 0.6 } });
          diagram.querySelectorAll("[data-diagram-node]").forEach((node, i) => {
            dtl.fromTo(node, { autoAlpha: 0, y: 14 }, { autoAlpha: 1, y: 0, duration: 1, ease: "power2.out" }, i * 1.6);
            const paths = links[i]?.querySelectorAll("path");
            if (paths) dtl.fromTo(paths, { strokeDashoffset: 1 }, { strokeDashoffset: 0, duration: 0.8, ease: "none", stagger: 0.4 }, i * 1.6 + 0.7);
          });
        }

        return () => gsap.killTweensOf(tl);
      });

      // Keep ScrollTrigger in step with Lenis' smoothed scroll.
      let unhook = () => {};
      const stopWatching = onLenisChange((lenis) => {
        unhook();
        unhook = lenis ? lenis.on("scroll", ScrollTrigger.update) : () => {};
      });

      // Layout can still move after mount (web fonts, route transition): re-measure.
      const late = window.setTimeout(() => ScrollTrigger.refresh(), 900);
      void document.fonts?.ready.then(() => alive && ScrollTrigger.refresh());

      cleanup = () => {
        window.clearTimeout(late);
        stopWatching();
        unhook();
        mm.revert();
      };
    });

    return () => {
      alive = false;
      cleanup();
    };
  }, []);

  return (
    <div ref={root} className="story">
      {hero}
      <HandStage canvasRef={canvas} indicatorRef={indicator} chapters={chapters} />
      <div className="story-sections">{children}</div>
    </div>
  );
}
