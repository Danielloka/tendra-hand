import type { Metadata } from "next";
import { notFound } from "next/navigation";
import { DocsPager } from "@/components/docs/DocsPager";
import { DocsToc } from "@/components/docs/DocsToc";
import { OG_IMAGE } from "@/components/pages/content";
import { JsonLd } from "@/components/seo/JsonLd";
import { h2Sections, plain, readMdx } from "@/components/seo/mdxText";
import { breadcrumbs, docsCrumbs, faqPage, techArticle } from "@/components/seo/schemas";
import { docsLoc } from "@/components/seo/urls";
import { docHref, getAllDocSlugs, getDoc, getPrevNext, getToc } from "@/lib/docs";

type Props = { params: Promise<{ slug: string[] }> };

export const dynamicParams = false;

export function generateStaticParams() {
  return getAllDocSlugs().map((slug) => ({ slug: slug.split("/") }));
}

export async function generateMetadata({ params }: Props): Promise<Metadata> {
  const slug = (await params).slug.join("/");
  const doc = await getDoc(slug);
  if (!doc) return {};
  const { title, description } = doc.frontmatter;
  const url = docsLoc(docHref(slug));
  return {
    title,
    description,
    alternates: { canonical: url },
    openGraph: { type: "article", title, description, url, modifiedTime: doc.frontmatter.updated, images: [OG_IMAGE] },
  };
}

/** The FAQ page's "## Question" sections as question/answer pairs (FAQPage schema). */
function faqPairs() {
  return h2Sections(readMdx("docs/faq").body).map(([q, a]) => ({ q, a: plain(a) }));
}

export default async function DocPage({ params }: Props) {
  const slug = (await params).slug.join("/");
  const doc = await getDoc(slug);
  if (!doc) notFound();

  const { default: Content, frontmatter: fm } = doc;
  const { page, prev, next } = await getPrevNext(slug);
  const toc = getToc(slug);

  const url = docsLoc(docHref(slug));
  const schemas = [
    techArticle({ title: fm.title, description: fm.description, url, updated: fm.updated, section: page?.group }),
    breadcrumbs(docsCrumbs({ name: fm.title, url })),
    ...(slug === "faq" ? [faqPage(faqPairs())] : []),
  ];

  return (
    <div className="docs-page">
      <JsonLd data={schemas} />
      <article className="docs-article">
        <header className="docs-article__head">
          {page && <p className="kicker">{page.group}</p>}
          <h1 className="docs-article__title">{fm.title}</h1>
          {fm.description && <p className="lead">{fm.description}</p>}
        </header>
        <DocsToc items={toc} variant="inline" />
        <div className="prose">
          <Content />
        </div>
        <DocsPager slug={slug} prev={prev} next={next} updated={fm.updated} />
      </article>
      <div className="docs-toc">
        <DocsToc items={toc} />
      </div>
    </div>
  );
}
