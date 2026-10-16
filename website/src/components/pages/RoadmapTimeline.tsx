import roadmapData from "@content/roadmap.json";
import { Reveal } from "@/components/ui/Reveal";
import { type Status, StatusBadge } from "@/components/ui/Tag";
import "./roadmap.css";

export type Milestone = { id: string; title: string; summary: string; status: Status };
export type Roadmap = { kicker: string; title: string; lead: string; milestones: Milestone[] };

export const roadmap = roadmapData as Roadmap;

type Props = {
  /** Tighter spacing (for the homepage). */
  compact?: boolean;
  /** Level of each milestone title, so it fits under the surrounding heading. */
  headingLevel?: 3 | 4;
  className?: string;
};

function Marker({ status }: { status: Status }) {
  return (
    <span className={`roadmap__marker roadmap__marker--${status}`} aria-hidden="true">
      {status === "done" && (
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.6" strokeLinecap="round" strokeLinejoin="round">
          <path d="m5 12.5 4.5 4.5L19 7.5" />
        </svg>
      )}
    </span>
  );
}

/**
 * The big milestones from content/roadmap.json as a vertical timeline. Server component;
 * used by the <Roadmap /> MDX component (project page) and the homepage (compact).
 */
export function RoadmapTimeline({ compact = false, headingLevel = 3, className = "" }: Props) {
  const H = `h${headingLevel}` as "h3" | "h4";
  return (
    <ol className={`roadmap not-prose${compact ? " roadmap--compact" : ""} ${className}`} aria-label="Milestones">
      {roadmap.milestones.map((m, i) => (
        <Reveal as="li" key={m.id} id={m.id} className={`roadmap__phase roadmap__phase--${m.status}`} delay={Math.min(i, 3) * 60}>
          <Marker status={m.status} />
          <div className="roadmap__card">
            <div className="roadmap__head">
              <p className="roadmap__label" aria-hidden="true">Step {i + 1}</p>
              <StatusBadge status={m.status} />
            </div>
            <H className="roadmap__title">
              <span className="visually-hidden">Step {i + 1}: </span>
              {m.title}
            </H>
            <p className="roadmap__summary">{m.summary}</p>
          </div>
        </Reveal>
      ))}
    </ol>
  );
}
