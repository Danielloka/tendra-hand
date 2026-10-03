import type { Metadata } from "next";
import { JourneyList } from "@/components/pages/JourneyList";
import { JourneyMap } from "@/components/pages/JourneyMap";
import { OG_IMAGE } from "@/components/pages/content";
import { PageHero } from "@/components/pages/PageHero";
import { SectionHead } from "@/components/ui/SectionHead";
import { journey } from "@/lib/journey";

const description = "A map of the road so far: the hardware, firmware, simulation, AI and website paths, including the ideas we tried and dropped.";

export const metadata: Metadata = {
  title: "The journey",
  description,
  alternates: { canonical: "/journey" },
  openGraph: { type: "website", title: "The journey", description, url: "/journey", images: [OG_IMAGE] },
};

export default function JourneyPage() {
  return (
    <div className="page page--journey">
      <PageHero kicker={journey.kicker} title={journey.title} lead={journey.lead} />
      <section className="page-band section--alt" aria-labelledby="journey-map">
        <div className="container">
          <h2 id="journey-map" className="visually-hidden">
            Map
          </h2>
          <JourneyMap />
        </div>
      </section>
      <section className="page-band" aria-label="Journey as text">
        <div className="container">
          <SectionHead kicker="As text" title="Every step, track by track" />
          <JourneyList />
        </div>
      </section>
    </div>
  );
}
