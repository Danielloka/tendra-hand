import { DocsSidebar } from "@/components/docs/DocsSidebar";
import { getDocsTree } from "@/lib/docs";
import "@/components/docs/docs.css";

/** Docs shell: sidebar | page. Each doc page adds its own "On this page" column. */
export default async function DocsLayout({ children }: { children: React.ReactNode }) {
  const groups = await getDocsTree();
  return (
    <div className="docs">
      <aside className="docs-sidebar" aria-label="Docs navigation">
        <DocsSidebar groups={groups} />
      </aside>
      <div className="docs-main">{children}</div>
    </div>
  );
}
