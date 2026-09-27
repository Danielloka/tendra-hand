import type { Metadata } from "next";
import { cache } from "react";
import { listMdxSlugs, loadMdx } from "@/lib/mdx";

/**
 * Loading helpers for content/pages/*.mdx and content/log/*.mdx (server only).
 * Formats are described in PLAN.md → "Content formats".
 */

export type PageFrontmatter = { title: string; kicker?: string; lead?: string; description?: string };
export type PageSlug = "project" | "hardware" | "software" | "gallery" | "log" | "contribute";

export type LogFrontmatter = { title: string; date: string; summary: string; cover?: string; tags?: string[] };
export type LogPost = LogFrontmatter & { slug: string; href: string };

/**
 * The site-wide share image (app/opengraph-image.tsx). A page that sets its own
 * `openGraph` replaces the root one, file image included, so pages add it back.
 */
export const OG_IMAGE = { url: "/opengraph-image", width: 1200, height: 630, alt: "Tendra Hand, an open-source, tendon-driven robotic hand" };

export const getPage = cache(async (slug: PageSlug) => {
  const mod = await loadMdx<PageFrontmatter>(`pages/${slug}`);
  if (!mod.frontmatter?.title) throw new Error(`content/pages/${slug}.mdx needs a "title" in its frontmatter.`);
  return mod;
});

/** Metadata for a content page, straight from its frontmatter. */
export async function pageMetadata(slug: PageSlug): Promise<Metadata> {
  const { title, description } = (await getPage(slug)).frontmatter;
  const url = `/${slug}`;
  return {
    title,
    description,
    alternates: { canonical: url },
    openGraph: { type: "website", title, description, url, images: [OG_IMAGE] },
  };
}

/** Every build-log post, newest first (by date, then slug descending). */
export const getLogPosts = cache(async (): Promise<LogPost[]> => {
  const posts = await Promise.all(
    listMdxSlugs("log").map(async (slug) => {
      const { frontmatter: fm } = await loadMdx<LogFrontmatter>(`log/${slug}`);
      if (!fm?.title || !fm.date) throw new Error(`content/log/${slug}.mdx needs a "title" and a "date" in its frontmatter.`);
      return { ...fm, slug, href: `/log/${slug}` };
    }),
  );
  return posts.sort((a, b) => b.date.localeCompare(a.date) || b.slug.localeCompare(a.slug));
});

export const getLogPost = cache(async (slug: string) => {
  if (!listMdxSlugs("log").includes(slug)) return null;
  return loadMdx<LogFrontmatter>(`log/${slug}`);
});

