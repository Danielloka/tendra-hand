import { CountUp } from "@/components/ui/CountUp";
import { Reveal } from "@/components/ui/Reveal";
import { SectionHead } from "@/components/ui/SectionHead";
import { home } from "./content";

/** "By the numbers": count-ups from content/home.json `stats` (final values are in the HTML). */
export function Stats() {
  const { kicker, title, items } = home.stats;
  return (
    <section className="section section--alt home-stats">
      <div className="container">
        <SectionHead kicker={kicker} title={title} />
        <div className="auto-grid home-stats__grid">
          {items.map((item, i) => (
            <Reveal key={item.label} className="stat" delay={i * 80}>
              <span className="stat__value">
                <CountUp to={item.value} />
                {item.unit && <span className="stat__unit">{item.unit}</span>}
              </span>
              <span className="stat__label">{item.label}</span>
            </Reveal>
          ))}
        </div>
      </div>
    </section>
  );
}
