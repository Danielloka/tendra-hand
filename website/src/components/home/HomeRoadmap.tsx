import Link from "next/link";
import { RoadmapTimeline, roadmap } from "@/components/pages/RoadmapTimeline";
import { SectionHead } from "@/components/ui/SectionHead";

/** Roadmap preview: the Pages Agent's timeline in compact form, plus a link to the full plan. */
export function HomeRoadmap() {
  return (
    <section className="section home-roadmap">
      <div className="container">
        <div className="home-roadmap__layout">
          <div className="home-roadmap__head">
            <SectionHead kicker={roadmap.kicker} title={roadmap.title} lead={roadmap.lead} />
            <Link className="btn btn--secondary" href="/project">
              See the full plan
            </Link>
          </div>
          <RoadmapTimeline compact />
        </div>
      </div>
    </section>
  );
}
