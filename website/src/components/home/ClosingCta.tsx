import { Reveal } from "@/components/ui/Reveal";
import { home } from "./content";

/** Closing band. Its button is the only primary button in this view. */
export function ClosingCta() {
  const { title, lead, label, href } = home.cta;
  return (
    <section className="section section--alt home-cta">
      <div className="container home-cta__inner">
        <Reveal as="h2" className="home-cta__title">
          {title}
        </Reveal>
        <Reveal as="p" className="lead home-cta__lead" delay={100}>
          {lead}
        </Reveal>
        <Reveal delay={200}>
          <a className="btn btn--primary btn--lg" href={href}>
            {label}
          </a>
        </Reveal>
      </div>
    </section>
  );
}
