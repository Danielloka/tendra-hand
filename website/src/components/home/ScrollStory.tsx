"use client";

import { useEffect, useRef } from "react";
import { handState } from "@/lib/handState";
import { onLenisChange } from "@/lib/scroll/lenis";
import { useStoryMotion } from "@/lib/scroll/motion";
import { STORY_IDS, applyHandState, buildStoryTimeline, scrollToStoryTime, staticPose } from "@/lib/scroll/storyTimeline";
import { HandStage } from "./HandStage";
import { MotionToggle } from "./MotionToggle";

const DESKTOP = "(min-width: 1024px)";
const COMPACT = "(max-width: 1023.98px)";

const clamp = (v: number, lo: number, hi: number) => Math.min(Math.max(v, lo), hi);

/**
 * Desktop hero spot for the hand, measured from the layout: `headline` = how
 * far down the canvas (0..1) the headline reaches (the hand starts below it);
 * `xPercent` = how far right to move the canvas so the hand is centred in the
 * space right of the centred text. The stage is sticky from the start, so its
 * box doesn't move while the story scrolls; the headline's top-of-page
 * position is rect + scrollY.
 */
function measureHero(root: HTMLElement, canvas: HTMLElement) {
  const title = root.querySelector(".story-hero__title");
  const text = root.querySelectorAll(".story-hero__lead, .story-hero__cta > *");
  // Measure the canvas box without the timeline's transform.
  const transform = canvas.style.transform;
  canvas.style.transform = "none";
  const box = canvas.getBoundingClientRect();
  canvas.style.transform = transform;
  const bottom = (title?.getBoundingClientRect().bottom ?? box.top) + window.scrollY;
  const textRight = Math.max(box.left + box.width / 2, ...Array.from(text, (el) => el.getBoundingClientRect().right));
  const handCentre = (textRight + box.right) / 2;
  return {
    headline: clamp((bottom - box.top) / Math.max(box.height, 1), 0, 0.6),
    xPercent: clamp(((handCentre - (box.left + box.width / 2)) / Math.max(box.width, 1)) * 100, 0, 38),
  };
}

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
 * breakpoint changes; the whole story is rebuilt when motion is switched on
 * or off (src/lib/scroll/motion.ts), and reverted on unmount (route change). Scrolling writes to `handState` and one transform,
 * never to React state.
 */
export function ScrollStory({ hero, children, chapters }: Props) {
  const root = useRef<HTMLDivElement>(null);
  const indicator = useRef<HTMLOListElement>(null);
  const canvas = useRef<HTMLDivElement>(null);
  const motion = useStoryMotion();

  useEffect(() => {
    const el = root.current;
    if (!el) return;
    // Without motion no GSAP is needed at all: one still pose for the static render.
    if (!motion) {
      applyHandState(handState, staticPose());
      return;
    }

    let alive = true;
    let cleanup = () => {};

    void Promise.all([import("gsap"), import("gsap/ScrollTrigger")]).then(([{ gsap }, { ScrollTrigger }]) => {
      if (!alive) return;
      gsap.registerPlugin(ScrollTrigger);
      const sections = Array.from(el.querySelectorAll<HTMLElement>("[data-story-section]"));
      const diagram = el.querySelector<HTMLElement>("[data-diagram]");
      const mm = gsap.matchMedia();

      mm.add({ desktop: DESKTOP, compact: COMPACT }, (ctx) => {
        const { desktop } = ctx.conditions as Record<string, boolean>;
        const tl = buildStoryTimeline(gsap, {
          state: handState,
          layout: desktop ? "desktop" : "compact",
          indicator: indicator.current,
          canvas: desktop ? canvas.current : null,
          hero: desktop && canvas.current ? measureHero(el, canvas.current) : undefined,
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
  }, [motion]);

  return (
    <div ref={root} className="story">
      {hero}
      <HandStage canvasRef={canvas} indicatorRef={indicator} chapters={chapters} motion={motion} />
      <MotionToggle motion={motion} />
      <div className="story-sections">{children}</div>
    </div>
  );
}
