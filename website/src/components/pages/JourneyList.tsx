import { Reveal } from "@/components/ui/Reveal";
import { StatusBadge } from "@/components/ui/Tag";
import { formatWhen, journey } from "@/lib/journey";
import "./journey.css";

/** The same journey as plain text, one list per track. Always shown; it is the phone layout. */
export function JourneyList() {
  return (
    <div className="jlist">
      {journey.tracks.map((track) => (
        <Reveal as="section" key={track.id} className="jlist__track" aria-labelledby={`jl-${track.id}`}>
          <h3 id={`jl-${track.id}`} className="jlist__heading">
            {track.label}
          </h3>
          <ol className="jlist__items">
            {journey.events
              .filter((e) => e.track === track.id)
              .map((e) => (
                <li key={e.id} className={`jlist__item jlist__item--${e.status}`}>
                  <span className="jlist__dot" aria-hidden="true" />
                  <div>
                    <p className="jlist__when">
                      {formatWhen(e)}
                      {e.status === "abandoned" ? <StatusBadge status="planned">Replaced</StatusBadge> : e.status !== "done" ? <StatusBadge status={e.status} /> : null}
                    </p>
                    <p className="jlist__title">{e.title}</p>
                    <p className="jlist__note">{e.note}</p>
                  </div>
                </li>
              ))}
          </ol>
        </Reveal>
      ))}
    </div>
  );
}
