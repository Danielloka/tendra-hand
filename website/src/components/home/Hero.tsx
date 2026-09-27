import Link from "next/link";
import { Tag } from "@/components/ui/Tag";
import { home, isExternal } from "./content";

const Chevron = () => (
  <svg className="btn__chevron" viewBox="0 0 14 14" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
    <path d="M5 2.5 9.5 7 5 11.5" />
  </svg>
);

/**
 * Homepage hero. Plain server-rendered text with no reveal animation, so the
 * headline paints immediately and is the page's LCP element. The hand sits in
 * the story's sticky canvas behind/below it (ScrollStory).
 */
export function Hero() {
  const { kicker, title, lead, primaryCta, secondaryCta } = home.hero;
  return (
    <section className="story-hero" aria-labelledby="hero-title">
      <div className="container story-hero__inner">
        <Tag tone="accent" dot>
          {kicker}
        </Tag>
        <h1 id="hero-title" className="display story-hero__title">
          {title}
        </h1>
        <p className="lead story-hero__lead">{lead}</p>
        <div className="story-hero__cta">
          <Link className="btn btn--primary btn--lg" href={primaryCta.href}>
            {primaryCta.label}
          </Link>
          {isExternal(secondaryCta.href) ? (
            <a className="btn btn--ghost" href={secondaryCta.href}>
              {secondaryCta.label}
              <Chevron />
            </a>
          ) : (
            <Link className="btn btn--ghost" href={secondaryCta.href}>
              {secondaryCta.label}
              <Chevron />
            </Link>
          )}
        </div>
      </div>
    </section>
  );
}
