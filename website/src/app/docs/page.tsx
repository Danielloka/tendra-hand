import type { Metadata } from "next";
import Link from "next/link";
import { JsonLd } from "@/components/seo/JsonLd";
import { breadcrumbs, collectionPage, docsCrumbs } from "@/components/seo/schemas";
import { docsLoc } from "@/components/seo/urls";
import { getDocsTree } from "@/lib/docs";

export const metadata: Metadata = {
  title: "Docs",
  description: "Guides for Tendra Hand: get the code running, build the thumb and index prototype, wire the electronics and run the MuJoCo simulation.",
  alternates: { canonical: docsLoc("/docs") },
};

export default async function DocsHome() {
  const groups = await getDocsTree();
  const first = groups[0]?.pages[0];
  const url = docsLoc("/docs");

  return (
    <div className="docs-home">
      <JsonLd
        data={[
          collectionPage({
            name: "Tendra Hand docs",
            description: metadata.description ?? undefined,
            url,
            items: groups.flatMap((g) => g.pages).map((p) => ({ name: p.title, url: docsLoc(p.href) })),
          }),
          breadcrumbs(docsCrumbs()),
        ]}
      />
      <header className="docs-article__head">
        <p className="kicker">Docs</p>
        <h1 className="docs-article__title">Build it, run it, understand it.</h1>
        <p className="lead">
          Everything you need to get the software running, build the thumb and index prototype, and find your way around the project. The docs grow with the hand, so some
          pages are still short. We say so where that&apos;s the case.
        </p>
        {first && (
          <p className="docs-home__cta">
            <Link className="btn btn--primary" href={first.href}>
              Start with {first.title}
            </Link>
            <span className="caption">
              Tip: press <kbd>/</kbd> to search.
            </span>
          </p>
        )}
      </header>

      {groups.map((group) => {
        const id = `group-${group.title.toLowerCase().replace(/\W+/g, "-")}`;
        return (
          <section key={group.title} className="docs-home__group" aria-labelledby={id}>
            <h2 id={id} className="docs-home__group-title">
              {group.title}
            </h2>
            <div className="auto-grid" style={{ "--grid-min": "15rem" } as React.CSSProperties}>
              {group.pages.map((page) => (
                <Link key={page.slug} href={page.href} className="card">
                  <h3 className="card__title">{page.title}</h3>
                  <p className="card__body">{page.description}</p>
                  <span className="card__more">Read</span>
                </Link>
              ))}
            </div>
          </section>
        );
      })}
    </div>
  );
}
