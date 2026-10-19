import { roadmap } from "@/components/pages/RoadmapTimeline";
import { Reveal } from "@/components/ui/Reveal";
import { journey } from "@/lib/journey";
import "./journey.css";

const STEP_LABEL = { done: "Done", current: "Now", next: "Next" } as const;

/** The five roadmap milestones with a "We are here" marker, and the steps inside the current one. */
export function JourneyProgress() {
  const { now } = journey;
  const here = roadmap.milestones.findIndex((m) => m.id === now.milestone);
  return (
    <div className="jnow">
      <ol className="jroad" aria-label="The five big milestones">
        {roadmap.milestones.map((m, i) => {
          const state = i < here ? "done" : i === here ? "here" : "ahead";
          return (
            <li key={m.id} className={`jroad__step jroad__step--${state}`} aria-current={state === "here" ? "step" : undefined}>
              <span className="jroad__num" aria-hidden="true">
                {i + 1}
              </span>
              <span className="jroad__title">
                <span className="visually-hidden">Step {i + 1}: </span>
                {m.title}
                {state === "done" && <span className="visually-hidden"> (done)</span>}
                {state === "ahead" && <span className="visually-hidden"> (coming up)</span>}
              </span>
              {state === "here" && <span className="jroad__here">We are here</span>}
            </li>
          );
        })}
      </ol>

      <Reveal className="jnow__card">
        <div className="jnow__text">
          <p className="kicker">{now.kicker}</p>
          <h2 id="journey-now" className="jnow__title">
            {now.title}
          </h2>
          <p className="jnow__lead">{now.text}</p>
        </div>
        <ol className="jsteps" aria-label="Steps in this milestone">
          {now.steps.map((s) => (
            <li key={s.label} className={`jsteps__item jsteps__item--${s.status}`}>
              <span className="jsteps__mark" aria-hidden="true">
                {s.status === "done" && (
                  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.8" strokeLinecap="round" strokeLinejoin="round">
                    <path d="m5 12.5 4.5 4.5L19 7.5" />
                  </svg>
                )}
              </span>
              <span className="jsteps__label">{s.label}</span>
              <span className="jsteps__status">{STEP_LABEL[s.status]}</span>
            </li>
          ))}
        </ol>
      </Reveal>
    </div>
  );
}
