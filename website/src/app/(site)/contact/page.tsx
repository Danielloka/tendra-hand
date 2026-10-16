import { ContactCards } from "@/components/pages/ContactCards";
import { ContentPage } from "@/components/pages/ContentPage";
import { getPage, pageMetadata } from "@/components/pages/content";
import { JsonLd } from "@/components/seo/JsonLd";
import { breadcrumbs, webPage } from "@/components/seo/schemas";
import { mainLoc } from "@/components/seo/urls";
import { siteConfig } from "@/lib/site";

export const generateMetadata = () => pageMetadata("contact");

export default async function ContactPage() {
  const { title, description } = (await getPage("contact")).frontmatter;
  return (
    <ContentPage slug="contact">
      <JsonLd
        data={[
          webPage({ name: title, description, url: mainLoc("/contact") }),
          breadcrumbs([
            { name: siteConfig.name, url: mainLoc("/") },
            { name: title, url: mainLoc("/contact") },
          ]),
        ]}
      />
      <ContactCards />
    </ContentPage>
  );
}
