import { Reveal } from "@/components/ui/Reveal";
import { formatDate, journey, trackLabel } from "@/lib/journey";
import "./journey.css";

/** Every idea we tried and dropped: what it was, why it failed, what we learned, what came instead. */
export function JourneyDropped() {
  const dropped = journey.events.filter((e) => e.status === "abandoned");
  return (
    <ul className="jdrop">
      {dropped.map((e, i) => {
        const next = journey.events.find((n) => n.replaces === e.id);
        return (
          <Reveal as="li" key={e.id} className={`jdrop__card lane--${e.track}`} delay={Math.min(i, 3) * 60}>
            <p className="jdrop__meta">
              {trackLabel(e.track)} · {formatDate(e.date, true)}
            </p>
            <h3 className="jdrop__title">
              <span className="jdrop__x" aria-hidden="true">×</span>
              {e.title}
            </h3>
            <p className="jdrop__note">{e.note}</p>
            <p className="jdrop__why">
              <strong>Why it did not work.</strong> {e.why}
            </p>
            <p className="jdrop__lesson">
              <strong>Lesson.</strong> {e.lesson}
            </p>
            {next && (
              <p className="jdrop__next">
                <strong>Instead:</strong> {next.title}
              </p>
            )}
          </Reveal>
        );
      })}
    </ul>
  );
}
