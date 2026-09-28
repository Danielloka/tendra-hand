import homeData from "@content/home.json";
import type { StoryId } from "@/lib/scroll/storyTimeline";

export type StorySectionData = { id: StoryId; kicker: string; title: string; body: string[]; points?: string[] };
export type DiagramNode = { id: "pc" | "esp32" | "motors"; label: string; detail: string };
export type DiagramData = { nodes: DiagramNode[]; caption: string };
type Cta = { label: string; href: string };

export type HomeContent = {
  hero: { kicker: string; title: string; lead: string; primaryCta: Cta; secondaryCta: Cta };
  story: StorySectionData[];
  diagram: DiagramData;
  stats: { kicker: string; title: string; items: { value: number; unit: string; label: string }[] };
  cta: { title: string; lead: string; label: string; href: string };
  /** Shown only when the visitor's system asks for reduced motion (MotionToggle). */
  motionToggle: { play: string; note: string };
};

/** content/home.json, typed (see PLAN.md, "Content formats"). */
export const home = homeData as HomeContent;

/** External links get a plain <a>; internal ones use next/link. */
export const isExternal = (href: string) => /^https?:\/\//.test(href);
