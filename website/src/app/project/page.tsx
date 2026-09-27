import { ContentPage } from "@/components/pages/ContentPage";
import { pageMetadata } from "@/components/pages/content";

export const generateMetadata = () => pageMetadata("project");

export default function ProjectPage() {
  return <ContentPage slug="project" />;
}
