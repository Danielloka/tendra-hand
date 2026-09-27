import type { gsap as GsapType } from "gsap";
import { type HandState, JOINT_IDS, type JointId, createHandState } from "@/lib/handState";

type Gsap = typeof GsapType;

/*
 * The homepage scroll story as ONE paused GSAP timeline.
 *
 * Timeline time is measured in "sections": time i..i+1 is the stretch of
 * scrolling that belongs to section i (0 = hero, 1 = intro, 2 = tendons,
 * 3 = joints, 4 = exploded, 5 = electronics; 6 = the end). ScrollStory maps
 * the scroll position onto this scale piecewise, from each section's measured
 * ScrollTrigger range, so tall and short sections both get the whole segment
 * and nothing has to be rebuilt when the text reflows.
 *
 * Only plain numbers are tweened (handState + one DOM transform on desktop),
 * so scrolling never touches React state.
 */

export const STORY_IDS = ["intro", "tendons", "joints", "exploded", "electronics"] as const;
export type StoryId = (typeof STORY_IDS)[number];
export const STORY_LENGTH = STORY_IDS.length + 1; // + the hero segment

export type StoryLayout = "desktop" | "compact";

const TAU = Math.PI * 2;

/**
 * Hero → story canvas travel on desktop, as a percentage of the canvas box.
 * Translate only: scaling would make R3F re-measure and resize the WebGL canvas
 * every frame. Keep CANVAS_HERO in sync with home.css (first paint).
 */
const CANVAS_HERO = { x: 0, y: 0, xPercent: 0, yPercent: 62 };
const CANVAS_STORY = { x: 0, y: 0, xPercent: 23, yPercent: 0 };

/** Flexion (0..1 of range) per joint in the "Joints" section; matches the hand lab's `joints` preset. */
const BEND: Partial<Record<JointId, number>> = {
  index_mcp_flex: 0.55,
  index_pip: 0.6,
  index_dip: 0.5,
  thumb_cmc_rot: 0.6,
  thumb_cmc_flex: 0.4,
  thumb_mcp: 0.5,
  thumb_ip: 0.5,
};

/** Desktop order: index MCP → PIP → DIP, then thumb CMC → MCP → IP. */
const SEQUENCE: JointId[][] = [["index_mcp_flex"], ["index_pip"], ["index_dip"], ["thumb_cmc_rot", "thumb_cmc_flex"], ["thumb_mcp"], ["thumb_ip"]];

const pick = (ids: JointId[]) => Object.fromEntries(ids.map((id) => [id, BEND[id] ?? 0]));
const straight = () => Object.fromEntries(JOINT_IDS.map((id) => [id, 0]));

/** Copies every number of `from` into `state` (the shared object must keep its identity). */
export function applyHandState(state: HandState, from: HandState) {
  Object.assign(state, { ...from, rotation: state.rotation, joints: state.joints });
  Object.assign(state.rotation, from.rotation);
  Object.assign(state.joints, from.joints);
}

/** The single pose shown with reduced motion: the intro's three-quarter view, standing still. */
export function staticPose(): HandState {
  return { ...createHandState(), idle: 0, rim: 0.6, rotation: { x: 0, y: -0.5, z: 0 } };
}

type Options = {
  state: HandState;
  layout: StoryLayout;
  /** Desktop only: the element that carries the canvas from the hero to the right column. */
  canvas?: HTMLElement | null;
  /** Section-name indicator; fades in once the story starts. */
  indicator?: HTMLElement | null;
};

export function buildStoryTimeline(gsap: Gsap, { state, layout, canvas, indicator }: Options) {
  const desktop = layout === "desktop";
  applyHandState(state, createHandState());

  const tl = gsap.timeline({ paused: true, defaults: { ease: "power1.inOut" } });
  const r = state.rotation;

  // 0 → 1 · Hero → Intro: calm down, turn to a three-quarter palm view, canvas slides right.
  tl.to(state, { idle: 0.2, rim: 0.6, duration: 1 }, 0).to(r, { y: -0.5, duration: 1 }, 0);
  if (desktop && canvas) {
    tl.fromTo(canvas, { ...CANVAS_HERO }, { ...CANVAS_STORY, duration: 1, ease: "power2.inOut" }, 0);
  }
  if (indicator) tl.fromTo(indicator, { autoAlpha: 0 }, { autoAlpha: 1, duration: 0.3 }, 0.7);

  // 1.75 → 2.3 · Tendons: turn to the back of the hand, cords light up.
  tl.to(r, { y: Math.PI, duration: 0.55 }, 1.75)
    .to(state, { rim: 0.3, idle: 0.1, duration: 0.4 }, 1.75)
    .to(state, { tendons: 1, duration: 0.3, ease: "power2.out" }, 2.05);

  // 2.75 → 3.2 · Joints: keep turning the same way to a raised three-quarter view.
  tl.to(state, { tendons: 0, idle: 0, duration: 0.3 }, 2.75).to(r, { x: 0.2, y: TAU - 0.8, duration: 0.45 }, 2.75);
  tl.to(state, { labels: 1, duration: 0.1 }, 3.2);
  if (desktop) {
    // One joint after the other, each label popping as its joint bends.
    SEQUENCE.forEach((ids, i) => tl.to(state.joints, { ...pick(ids), duration: 0.1, ease: "power2.inOut" }, 3.22 + i * 0.09));
  } else {
    // Simpler on small screens: the index bends (labelled), then the thumb.
    tl.to(state.joints, { ...pick(["index_mcp_flex", "index_pip", "index_dip"]), duration: 0.2 }, 3.2)
      .to(state, { labels: 0, duration: 0.1 }, 3.45)
      .to(state.joints, { ...pick(["thumb_cmc_rot", "thumb_cmc_flex", "thumb_mcp", "thumb_ip"]), duration: 0.2 }, 3.5);
  }
  // …then everything lets go.
  tl.to(state, { labels: 0, duration: 0.1 }, 3.8).to(state.joints, { ...straight(), duration: 0.25 }, 3.8);

  // 4.0 → 4.85 · Exploded view: apart over the first half, hold, back together.
  // Pulled back while apart so the spread-out parts stay in frame.
  tl.to(r, { x: 0.1, y: TAU - 0.6, duration: 0.25 }, 4.0)
    .to(state, { explode: 1, zoom: 0.8, duration: 0.3, ease: "power2.inOut" }, 4.15)
    .to(state, { explode: 0, zoom: 1, duration: 0.28, ease: "power2.inOut" }, 4.55);
  if (desktop) tl.to(r, { y: TAU - 0.15, duration: 0.65, ease: "none" }, 4.25); // slow drift while apart (ends 4.9)

  // 4.85 → 5.2 · Electronics: wireframe, pulled back a little for the diagram.
  tl.to(state, { wireframe: 1, zoom: 0.85, rim: 0.45, duration: 0.35 }, 4.85).to(r, { x: 0, y: TAU - 0.5, duration: 0.35 }, 4.9);

  tl.set({}, {}, STORY_LENGTH); // the timeline always spans the whole story
  return tl;
}

/**
 * Scroll position → timeline time. `bounds` are the scroll positions where
 * each segment starts, plus the end of the last one (STORY_LENGTH + 1 values).
 */
export function scrollToStoryTime(scroll: number, bounds: number[]): number {
  if (scroll <= bounds[0]) return 0;
  for (let i = 0; i < bounds.length - 1; i++) {
    if (scroll < bounds[i + 1]) return i + (scroll - bounds[i]) / Math.max(1, bounds[i + 1] - bounds[i]);
  }
  return bounds.length - 1;
}
