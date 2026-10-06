import { ContentPage } from "@/components/pages/ContentPage";
import { getPage } from "@/components/pages/content";
import { JsonLd } from "@/components/seo/JsonLd";
import { docsPageMetadata } from "@/components/seo/meta";
import { breadcrumbs, docsCrumbs, techArticle } from "@/components/seo/schemas";
import { docsLoc } from "@/components/seo/urls";

export const generateMetadata = () => docsPageMetadata("software", "/docs/software");

export default async function SoftwarePage() {
  const { title, description } = (await getPage("software")).frontmatter;
  const url = docsLoc("/docs/software");
  return (
    <>
      <JsonLd data={[techArticle({ title, description, url, section: "Software" }), breadcrumbs(docsCrumbs({ name: title, url }))]} />
      <ContentPage slug="software" toc />
    </>
  );
}
