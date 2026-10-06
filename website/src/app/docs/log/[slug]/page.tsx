import type { Metadata } from "next";
import Image from "next/image";
import Link from "next/link";
import { notFound } from "next/navigation";
import { getLogPost, getLogPosts, OG_IMAGE } from "@/components/pages/content";
import { PostMeta } from "@/components/pages/LogCard";
import { PageHero } from "@/components/pages/PageHero";
import { JsonLd } from "@/components/seo/JsonLd";
import { breadcrumbs, docsCrumbs, techArticle } from "@/components/seo/schemas";
import { docsLoc, mainLoc } from "@/components/seo/urls";

type Props = { params: Promise<{ slug: string }> };

export const dynamicParams = false;

export async function generateStaticParams() {
  return (await getLogPosts()).map((post) => ({ slug: post.slug }));
}

export async function generateMetadata({ params }: Props): Promise<Metadata> {
  const { slug } = await params;
  const post = await getLogPost(slug);
  if (!post) return {};
  const { title, summary, date, tags, cover } = post.frontmatter;
  const url = docsLoc(`/docs/log/${slug}`);
  return {
    title,
    description: summary,
    alternates: { canonical: url },
    openGraph: {
      type: "article",
      title,
      description: summary,
      url,
      publishedTime: date,
      tags,
      images: [cover ?? OG_IMAGE],
    },
  };
}

export default async function LogPostPage({ params }: Props) {
  const { slug } = await params;
  const post = await getLogPost(slug);
  if (!post) notFound();

  const { default: Content, frontmatter: fm } = post;
  const posts = await getLogPosts(); // newest first
  const i = posts.findIndex((p) => p.slug === slug);
  const older = posts[i + 1];
  const newer = posts[i - 1];

  const url = docsLoc(`/docs/log/${slug}`);
  return (
    <article className="post">
      <JsonLd
        data={[
          techArticle({
            title: fm.title,
            description: fm.summary,
            url,
            published: fm.date,
            keywords: fm.tags,
            section: "Build log",
            image: mainLoc(fm.cover ?? OG_IMAGE.url),
          }),
          breadcrumbs(docsCrumbs({ name: "Build log", url: docsLoc("/docs/log") }, { name: fm.title, url })),
        ]}
      />
      <PageHero
        narrow
        title={fm.title}
        lead={fm.summary}
        top={
          <Link className="post-back" href="/docs/log">
            Build log
          </Link>
        }
      >
        <PostMeta date={fm.date} tags={fm.tags} />
      </PageHero>

      {fm.cover && (
        <figure className="post-cover">
          <Image src={fm.cover} alt="" width={1600} height={900} sizes="(min-width: 64rem) 60rem, 100vw" priority />
        </figure>
      )}

      <div className="container post-body">
        <div className="prose">
          <Content />
        </div>

        {(older || newer) && (
          <nav className="post-pager" aria-label="More posts">
            {older && (
              <Link className="card post-pager__link" href={older.href} rel="prev">
                <span className="post-pager__dir">‹ Previous post</span>
                <span className="post-pager__title">{older.title}</span>
              </Link>
            )}
            {newer && (
              <Link className="card post-pager__link post-pager__link--next" href={newer.href} rel="next">
                <span className="post-pager__dir">Next post ›</span>
                <span className="post-pager__title">{newer.title}</span>
              </Link>
            )}
          </nav>
        )}
      </div>
    </article>
  );
}
