import { Reveal } from "@/components/ui/Reveal";
import { journey } from "@/lib/journey";
import "./journey.css";

/** Ideas we dropped, each as: what we tried, why it failed, the lesson. */
export function JourneyLessons() {
  return (
    <ul className="jlessons">
      {journey.lessons.map((l, i) => (
        <Reveal as="li" key={l.id} className="jlesson" delay={Math.min(i, 3) * 60}>
          <p className="jlesson__tried">
            <span className="visually-hidden">We tried: </span>
            {l.tried}
          </p>
          <p className="jlesson__why">{l.why}</p>
          <p className="jlesson__lesson">
            <span className="jlesson__label">Lesson</span>
            {l.lesson}
          </p>
        </Reveal>
      ))}
    </ul>
  );
}
