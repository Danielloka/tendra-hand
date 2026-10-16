import Link from "next/link";
import { RoadmapTimeline, roadmap } from "@/components/pages/RoadmapTimeline";
import { SectionHead } from "@/components/ui/SectionHead";

/** The milestones in compact form, with links to the project page and the journey. */
export function HomeRoadmap() {
  return (
    <section className="section home-roadmap">
      <div className="container">
        <div className="home-roadmap__layout">
          <div className="home-roadmap__head">
            <SectionHead kicker={roadmap.kicker} title={roadmap.title} lead={roadmap.lead} />
            <div className="home-roadmap__actions">
              <Link className="btn btn--secondary" href="/project">
                More about the project
              </Link>
              <Link className="btn btn--outline" href="/journey">
                See the whole journey
              </Link>
            </div>
          </div>
          <RoadmapTimeline compact />
        </div>
      </div>
    </section>
  );
}
