import { ContentPage } from "@/components/pages/ContentPage";
import { pageMetadata } from "@/components/pages/content";
import { ProjectHand } from "@/components/three/ProjectHand";
import { CountUp } from "@/components/ui/CountUp";
import "./project.css";

export const generateMetadata = () => pageMetadata("project");

const STATS = [
  { to: 20, label: "joints in the full hand" },
  { to: 8, label: "joints in the prototype" },
  { to: 3, label: "open licenses" },
  { to: 100, label: "% open source", suffix: "%" },
];

export default function ProjectPage() {
  return (
    <ContentPage
      slug="project"
      heroExtra={
        <>
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
