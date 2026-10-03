import { DocsSidebar } from "@/components/docs/DocsSidebar";
import { Footer } from "@/components/layout/Footer";
import { Nav } from "@/components/layout/Nav";
import { SmoothScroll } from "@/components/layout/SmoothScroll";
import { getDocsTree, type DocGroup } from "@/lib/docs";
import { docsNav, siteConfig } from "@/lib/site";
import "@/components/docs/docs.css";

/** Project reference pages that live next to the guides (Hardware, Software, Build log). */
const reference: DocGroup = {
  title: "Project",
  pages: siteConfig.docsNav
    .filter((item) => item.href !== "/docs/getting-started")
    .map((item) => ({ slug: item.href.replace("/docs/", ""), href: item.href, title: item.label, description: "", group: "Project" })),
};

/** Docs shell (docs.<domain>): own header, sidebar | page. Each doc page adds its own "On this page" column. */
export default async function DocsLayout({ children }: { children: React.ReactNode }) {
  const groups = [...(await getDocsTree()), reference];
  return (
    <SmoothScroll>
      <Nav items={docsNav} docs />
      <main id="main">
        <div className="docs">
          <aside className="docs-sidebar" aria-label="Docs navigation">
            <DocsSidebar groups={groups} />
          </aside>
          <div className="docs-main">{children}</div>
        </div>
      </main>
      <Footer items={docsNav} />
    </SmoothScroll>
  );
}
