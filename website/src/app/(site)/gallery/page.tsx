import gallery from "@content/data/gallery.json";
import { ContentPage } from "@/components/pages/ContentPage";
import { pageMetadata } from "@/components/pages/content";
import { Gallery, type GalleryItem } from "@/components/pages/Gallery";

export const generateMetadata = () => pageMetadata("gallery");

export default function GalleryPage() {
  return (
    <ContentPage slug="gallery">
      <div className="container">
        <Gallery items={gallery.items as GalleryItem[]} />
      </div>
    </ContentPage>
  );
}
