import { ContentPage } from "@/components/pages/ContentPage";
import { getPage } from "@/components/pages/content";
import { JsonLd } from "@/components/seo/JsonLd";
import { docsPageMetadata } from "@/components/seo/meta";
import { breadcrumbs, docsCrumbs, techArticle } from "@/components/seo/schemas";
import { docsLoc } from "@/components/seo/urls";

export const generateMetadata = () => docsPageMetadata("hardware", "/docs/hardware");

export default async function HardwarePage() {
  const { title, description } = (await getPage("hardware")).frontmatter;
  const url = docsLoc("/docs/hardware");
  return (
    <>
      <JsonLd data={[techArticle({ title, description, url, section: "Hardware" }), breadcrumbs(docsCrumbs({ name: title, url }))]} />
      <ContentPage slug="hardware" toc />
    </>
  );
}
