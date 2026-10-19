import Image from "next/image";
import { Reveal } from "@/components/ui/Reveal";
import { journey } from "@/lib/journey";
import "./journey.css";

/** The big steps so far: one picture, one sentence and a few highlights each. Picture and text swap sides. */
export function JourneyChapters() {
  return (
    <ol className="jchapters">
      {journey.chapters.map((c, i) => (
        <Reveal as="li" key={c.id} id={c.id} className="jchapter">
          <figure className={`jchapter__media${c.image.fit === "contain" ? " jchapter__media--contain" : ""}`}>
            <Image src={c.image.src} alt={c.image.alt} width={c.image.width} height={c.image.height} sizes="(min-width: 56rem) 34rem, 100vw" />
          </figure>
          <div className="jchapter__body">
            <p className="jchapter__meta">
              <span className="jchapter__num">{String(i + 1).padStart(2, "0")}</span>
              {c.when}
            </p>
            <h3 className="jchapter__title">{c.title}</h3>
            <p className="jchapter__text">{c.text}</p>
            <ul className="jchapter__list">
              {c.highlights.map((h) => (
                <li key={h}>{h}</li>
              ))}
            </ul>
          </div>
        </Reveal>
      ))}
    </ol>
  );
}
