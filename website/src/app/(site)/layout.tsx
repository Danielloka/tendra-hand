import { Footer } from "@/components/layout/Footer";
import { Nav } from "@/components/layout/Nav";
import { SmoothScroll } from "@/components/layout/SmoothScroll";
import { mainNav } from "@/lib/site";

/** Main site: homepage, project, journey, gallery, contribute. The hand lab (/dev/hand) is not in the nav. The docs have their own shell in app/docs. */
export default function SiteLayout({ children }: { children: React.ReactNode }) {
  return (
    <SmoothScroll>
      <Nav items={mainNav} />
      <main id="main">{children}</main>
      <Footer items={mainNav} />
    </SmoothScroll>
  );
}
