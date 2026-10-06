import type { Metadata } from "next";
import { JourneyDays } from "@/components/pages/JourneyDays";
import { JourneyDropped } from "@/components/pages/JourneyDropped";
import { JourneyMap } from "@/components/pages/JourneyMap";
import { OG_IMAGE } from "@/components/pages/content";
import { PageHero } from "@/components/pages/PageHero";
import { JsonLd } from "@/components/seo/JsonLd";
import { breadcrumbs, webPage } from "@/components/seo/schemas";
import { mainLoc } from "@/components/seo/urls";
import { SectionHead } from "@/components/ui/SectionHead";
import { journey } from "@/lib/journey";
import "@/components/pages/journey.css";

const description = "A map of the road so far across hardware, firmware, simulation, AI and the website, including the ideas we dropped and what we learned from them.";

export const metadata: Metadata = {
  title: "The journey",
  description,
  alternates: { canonical: "/journey" },
  openGraph: { type: "website", title: "The journey", description, url: "/journey", images: [OG_IMAGE] },
};

export default function JourneyPage() {
  const stats = [
    { n: journey.days.length, label: "days so far" },
    { n: journey.events.filter((e) => e.status === "done").length, label: "steps done" },
    { n: journey.events.filter((e) => e.status === "abandoned").length, label: "ideas dropped" },
    { n: journey.tracks.length, label: "tracks at once" },
  ];
  return (
    <div className="page page--journey">
      <JsonLd
        data={[
          webPage({ name: "The journey", description, url: mainLoc("/journey") }),
          breadcrumbs([
            { name: "Tendra Hand", url: mainLoc("/") },
            { name: "The journey", url: mainLoc("/journey") },
          ]),
        ]}
      />
      <PageHero kicker={journey.kicker} title={journey.title} lead={journey.lead}>
        <ul className="jstats" aria-label="The journey in numbers">
          {stats.map((s) => (
            <li key={s.label} className="jstats__item">
              <span className="jstats__num">{s.n}</span>
              <span className="jstats__label">{s.label}</span>
            </li>
          ))}
        </ul>
      </PageHero>

      <section className="page-band section--alt" aria-labelledby="journey-map">
        <div className="container">
          <h2 id="journey-map" className="visually-hidden">
            Map of the journey
          </h2>
          <JourneyMap />
        </div>
      </section>

      <section className="page-band" aria-label="Day by day">
        <div className="container">
          <SectionHead kicker="Day by day" title="What happened, chapter by chapter" />
          <JourneyDays />
        </div>
      </section>

      <section className="page-band section--alt" aria-label="Dropped ideas">
        <div className="container">
          <SectionHead kicker="Dropped ideas" title="What we tried, why it failed, and what we learned" lead="Mistakes are part of an open project. Here is each one, so you can skip it." />
          <JourneyDropped />
        </div>
      </section>
    </div>
  );
}
