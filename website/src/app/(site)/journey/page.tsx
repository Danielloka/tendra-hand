import type { Metadata } from "next";
import { JourneyChapters } from "@/components/pages/JourneyChapters";
import { JourneyLessons } from "@/components/pages/JourneyLessons";
import { JourneyProgress } from "@/components/pages/JourneyProgress";
import { PageHero } from "@/components/pages/PageHero";
import { JsonLd } from "@/components/seo/JsonLd";
import { gitLastModified } from "@/components/seo/lastmod";
import { breadcrumbs, webPage } from "@/components/seo/schemas";
import { socialMeta } from "@/components/seo/social";
import { mainLoc } from "@/components/seo/urls";
import Link from "@/components/ui/Link";
import { Reveal } from "@/components/ui/Reveal";
import { SectionHead } from "@/components/ui/SectionHead";
import { journey } from "@/lib/journey";
import "@/components/pages/journey.css";

const TITLE = "Journey: Building an Open-Source Robotic Hand";
const description = "How far Tendra Hand has come: where we are on the roadmap, the big steps we have built, and the ideas we dropped and what they taught us.";

export const metadata: Metadata = socialMeta({ title: TITLE, description, url: "/journey" });

export default function JourneyPage() {
  const { chaptersHead, lessonsHead, next } = journey;
  return (
    <div className="page page--journey">
      <JsonLd
        data={[
          webPage({ name: TITLE, description, url: mainLoc("/journey"), dateModified: gitLastModified("content/journey.json") }),
          breadcrumbs([
            { name: "Tendra Hand", url: mainLoc("/") },
            { name: "The journey", url: mainLoc("/journey") },
          ]),
        ]}
      />
      <PageHero kicker={journey.kicker} title={journey.title} lead={journey.lead} />

      <section className="page-band section--alt" aria-labelledby="journey-now">
        <div className="container">
          <JourneyProgress />
        </div>
      </section>

      <section className="page-band" aria-label={chaptersHead.kicker}>
        <div className="container">
          <SectionHead kicker={chaptersHead.kicker} title={chaptersHead.title} />
          <JourneyChapters />
        </div>
      </section>

      <section className="page-band section--alt" aria-label={lessonsHead.kicker}>
        <div className="container">
          <SectionHead kicker={lessonsHead.kicker} title={lessonsHead.title} lead={lessonsHead.lead} />
          <JourneyLessons />
        </div>
      </section>

      <section className="page-band" aria-labelledby="journey-next">
        <div className="container">
          <Reveal className="jnext">
            <p className="kicker">{next.kicker}</p>
            <h2 id="journey-next" className="jnext__title">
              {next.title}
            </h2>
            <p className="jnext__text">{next.text}</p>
            <div className="jnext__actions">
              {next.actions.map((a) => (
                <Link key={a.href} className={`btn btn--${a.variant}`} href={a.href}>
                  {a.label}
                </Link>
              ))}
            </div>
          </Reveal>
        </div>
      </section>
    </div>
  );
}
