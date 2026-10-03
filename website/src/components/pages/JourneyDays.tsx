import { Reveal } from "@/components/ui/Reveal";
import { formatDate, journey, trackLabel } from "@/lib/journey";
import "./journey.css";

/** The journey as chapters, one per day, with every step as text. It is the phone layout. */
export function JourneyDays() {
  return (
    <ol className="jdays">
      {journey.days.map((day, i) => {
        const events = journey.events.filter((e) => e.date === day.date);
        return (
          <Reveal as="li" key={day.date} className="jday" delay={Math.min(i, 3) * 60}>
            <div className="jday__head">
              <span className="jday__num">Day {i + 1}</span>
              <span className="jday__date">{formatDate(day.date, true)}</span>
            </div>
            <h3 className="jday__title">{day.title}</h3>
            <p className="jday__blurb">{day.blurb}</p>
            <ul className="jday__events">
              {events.map((e) => (
                <li key={e.id} className={`jday__event lane--${e.track} jday__event--${e.status}`}>
                  <span className="jday__dot" aria-hidden="true" />
                  <div>
                    <p className="jday__event-title">
                      {e.title}
                      {e.status === "abandoned" && <span className="jmap__chip">Dropped</span>}
                    </p>
                    <p className="jday__event-note">
                      <span className="jday__track">{trackLabel(e.track)}.</span> {e.note}
                    </p>
                  </div>
                </li>
              ))}
            </ul>
          </Reveal>
        );
      })}
    </ol>
  );
}
