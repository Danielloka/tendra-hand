import { Reveal } from "@/components/ui/Reveal";
import type { StorySectionData } from "./content";

/**
 * One chapter of the scroll story: real heading, paragraphs and points in
 * reading order. ScrollStory finds it by `data-story-section` and uses its
 * scroll range to drive the hand. `children` goes after the text (the diagram).
 */
export function StorySection({ section, children }: { section: StorySectionData; children?: React.ReactNode }) {
  const titleId = `${section.id}-title`;
  return (
    <section id={section.id} data-story-section={section.id} className={`story-section story-section--${section.id}`} aria-labelledby={titleId}>
      <div className="container">
        <div className="story-copy">
          <p className="kicker">{section.kicker}</p>
          <Reveal as="h2" id={titleId} className="story-copy__title">
            {section.title}
          </Reveal>
          {section.body.map((text, i) => (
            <Reveal as="p" key={i} className="story-copy__body" delay={100 + i * 80}>
              {text}
            </Reveal>
          ))}
          {section.points && section.points.length > 0 && (
            <Reveal delay={260}>
              <ul className="story-copy__points" role="list">
                {section.points.map((point) => (
                  <li key={point} className="tag">
                    {point}
                  </li>
                ))}
              </ul>
            </Reveal>
          )}
          {children}
        </div>
      </div>
    </section>
  );
}
