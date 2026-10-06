import { ContentPage } from "@/components/pages/ContentPage";
import { getPage, pageMetadata } from "@/components/pages/content";
import { JsonLd } from "@/components/seo/JsonLd";
import { breadcrumbs, webPage } from "@/components/seo/schemas";
import { mainLoc } from "@/components/seo/urls";
import { githubUrl } from "@/lib/site";

export const generateMetadata = () => pageMetadata("contribute");

export default async function ContributePage() {
  const { title, description } = (await getPage("contribute")).frontmatter;
  return (
    <>
      <JsonLd
        data={[
          webPage({ name: title, description, url: mainLoc("/contribute") }),
          breadcrumbs([
            { name: "Tendra Hand", url: mainLoc("/") },
            { name: title, url: mainLoc("/contribute") },
          ]),
        ]}
      />
      <ContentPage
        slug="contribute"
        actions={
          <>
            <a className="btn btn--primary btn--lg" href={githubUrl}>
              Open GitHub
            </a>
            <a className="btn btn--ghost" href={`${githubUrl}/issues`}>
              See open issues <span aria-hidden="true">›</span>
            </a>
          </>
        }
      />
    </>
  );
}
