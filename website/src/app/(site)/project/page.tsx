import { ContentPage } from "@/components/pages/ContentPage";
import { getPage, pageMetadata } from "@/components/pages/content";
import { JsonLd } from "@/components/seo/JsonLd";
import { gitLastModified } from "@/components/seo/lastmod";
import { breadcrumbs, webPage } from "@/components/seo/schemas";
import { mainLoc } from "@/components/seo/urls";
import { siteConfig } from "@/lib/site";
import { ProjectHand } from "@/components/three/ProjectHand";
import { CountUp } from "@/components/ui/CountUp";
import "./project.css";

export const generateMetadata = () => pageMetadata("project");

const STATS = [
  { to: 20, label: "joints in the full hand" },
  { to: 5, label: "big steps to a robot at home" },
  { to: 2, label: "hands on the first robot" },
  { to: 100, label: "% open source", suffix: "%" },
];

export default async function ProjectPage() {
  const { title, description } = (await getPage("project")).frontmatter;
  const url = mainLoc("/project");
  return (
    <ContentPage
      slug="project"
      heroExtra={
        <>
          <JsonLd
            data={[
              webPage({ name: title, description, url, type: "AboutPage", dateModified: gitLastModified("content/pages/project.mdx") }),
              breadcrumbs([
                { name: siteConfig.name, url: mainLoc("/") },
                { name: title, url },
              ]),
            ]}
          />
          <ProjectHand className="project-hand" />
          <ul className="project-stats" aria-label="Project at a glance">
            {STATS.map((s) => (
              <li key={s.label} className="project-stats__item">
                <span className="project-stats__num">
                  <CountUp to={s.to} />
                  {s.suffix}
                </span>
                <span className="project-stats__label">
                  {s.label.replace(/^% /, "")}
                </span>
              </li>
            ))}
          </ul>
        </>
      }
    />
  );
}
