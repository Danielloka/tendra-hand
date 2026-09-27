import roadmapData from "@content/roadmap.json";
import { Reveal } from "@/components/ui/Reveal";
import { type Status, StatusBadge } from "@/components/ui/Tag";
import "./roadmap.css";

export type RoadmapPhase = { id: string; title: string; summary: string; status: Status; items: string[] };
export type Roadmap = { kicker: string; title: string; lead: string; phases: RoadmapPhase[] };

export const roadmap = roadmapData as Roadmap;

type Props = {
  /** Tighter spacing, and only the phase in progress lists its items (for the homepage). */
  compact?: boolean;
  /** Level of each phase title, so it fits under the surrounding heading. */
  headingLevel?: 3 | 4;
  phases?: RoadmapPhase[];
  className?: string;
};

/** "Phase 1: Thumb and index" → ["Phase 1", "Thumb and index"]. */
function splitTitle(title: string): [label: string | null, name: string] {
  const m = title.match(/^(Phase\s+\d+):\s*(.+)$/i);
  return m ? [m[1], m[2]] : [null, title];
}

// In the phase that's in progress, items starting with "Still to do" are open.
const isOpen = (phase: RoadmapPhase, item: string) => phase.status === "planned" || (phase.status === "in-progress" && /^still to do\b/i.test(item));

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
 * Vertical roadmap timeline from content/roadmap.json. Server component; used by
 * the <Roadmap /> MDX component (project page) and the homepage (compact).
 */
export function RoadmapTimeline({ compact = false, headingLevel = 3, phases = roadmap.phases, className = "" }: Props) {
  const H = `h${headingLevel}` as "h3" | "h4";
  return (
    <ol className={`roadmap not-prose${compact ? " roadmap--compact" : ""} ${className}`} aria-label="Roadmap phases">
      {phases.map((phase, i) => {
        const [label, name] = splitTitle(phase.title);
        const showItems = !compact || phase.status === "in-progress";
        return (
          <Reveal as="li" key={phase.id} id={phase.id} className={`roadmap__phase roadmap__phase--${phase.status}`} delay={Math.min(i, 3) * 60}>
            <Marker status={phase.status} />
            <div className="roadmap__card">
              <div className="roadmap__head">
                {label && <p className="roadmap__label">{label}</p>}
                <StatusBadge status={phase.status} />
              </div>
              <H className="roadmap__title">
                {label && <span className="visually-hidden">{label}: </span>}
                {name}
              </H>
              <p className="roadmap__summary">{phase.summary}</p>
              {showItems && phase.items.length > 0 && (
                <ul className="roadmap__items" role="list">
                  {phase.items.map((item) => (
                    <li key={item} className={isOpen(phase, item) ? "roadmap__item" : "roadmap__item roadmap__item--done"}>
                      {item}
                    </li>
                  ))}
                </ul>
              )}
            </div>
          </Reveal>
        );
      })}
    </ol>
  );
}
