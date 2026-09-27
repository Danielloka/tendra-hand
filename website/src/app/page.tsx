import type { Metadata } from "next";
import { ClosingCta } from "@/components/home/ClosingCta";
import { home } from "@/components/home/content";
import { ElectronicsDiagram } from "@/components/home/ElectronicsDiagram";
import { Hero } from "@/components/home/Hero";
import { HomeRoadmap } from "@/components/home/HomeRoadmap";
import { ScrollStory } from "@/components/home/ScrollStory";
import { Stats } from "@/components/home/Stats";
import { StorySection } from "@/components/home/StorySection";
import { siteConfig } from "@/lib/site";
import "@/components/home/home.css";

export const metadata: Metadata = {
  title: { absolute: `${siteConfig.name}: ${home.hero.title.replace(/\.$/, "").toLowerCase()}` },
  description: siteConfig.description,
  alternates: { canonical: "/" },
};

/*
 * Homepage: hero → scroll story (sticky 3D hand + five chapters) → stats →
 * roadmap → closing CTA. All text is server-rendered from content/home.json;
 * the client parts (ScrollStory, HandStage) only add motion and the canvas.
 */
export default function Home() {
  const chapters = home.story.map((s) => ({ id: s.id, label: s.kicker }));
  return (
    <>
      <ScrollStory hero={<Hero />} chapters={chapters}>
        {home.story.map((section) => (
          <StorySection key={section.id} section={section}>
            {section.id === "electronics" && <ElectronicsDiagram diagram={home.diagram} />}
          </StorySection>
        ))}
      </ScrollStory>
      <Stats />
      <HomeRoadmap />
      <ClosingCta />
    </>
  );
}
