import gallery from "@content/data/gallery.json";
import { ContentPage } from "@/components/pages/ContentPage";
import { getPage, pageMetadata } from "@/components/pages/content";
import { Gallery, type GalleryItem } from "@/components/pages/Gallery";
import { JsonLd } from "@/components/seo/JsonLd";
import { breadcrumbs, imageGallery } from "@/components/seo/schemas";
import { mainLoc } from "@/components/seo/urls";
import { siteConfig } from "@/lib/site";

export const generateMetadata = () => pageMetadata("gallery");

export default async function GalleryPage() {
  const { title, description } = (await getPage("gallery")).frontmatter;
  const images = (gallery.items as GalleryItem[]).filter((i) => i.type === "image");
  return (
    <ContentPage slug="gallery">
      <JsonLd
        data={[
          imageGallery({ name: `${siteConfig.name} ${title}`, description, url: mainLoc("/gallery"), images }),
          breadcrumbs([
            { name: siteConfig.name, url: mainLoc("/") },
            { name: title, url: mainLoc("/gallery") },
          ]),
        ]}
      />
      <div className="container">
        <Gallery items={gallery.items as GalleryItem[]} />
      </div>
    </ContentPage>
  );
}
