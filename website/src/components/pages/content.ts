import type { Metadata } from "next";
import { cache } from "react";
import { OG_IMAGE, socialMeta } from "@/components/seo/social";
import { listMdxSlugs, loadMdx } from "@/lib/mdx";

export { OG_IMAGE };

/**
 * Loading helpers for content/pages/*.mdx and content/log/*.mdx (server only).
 * Formats are described in PLAN.md → "Content formats".
 */

export type PageFrontmatter = { title: string; kicker?: string; lead?: string; description?: string; /** <title> when the H1 is not the best search title (no brand). */ seoTitle?: string; updated?: string };
export type PageSlug = "project" | "hardware" | "software" | "gallery" | "log" | "contact";

export type LogFrontmatter = { title: string; date: string; summary: string; cover?: string; tags?: string[] };
export type LogPost = LogFrontmatter & { slug: string; href: string };

export const getPage = cache(async (slug: PageSlug) => {
  const mod = await loadMdx<PageFrontmatter>(`pages/${slug}`);
  if (!mod.frontmatter?.title) throw new Error(`content/pages/${slug}.mdx needs a "title" in its frontmatter.`);
  return mod;
});

/** Metadata for a content page, from its frontmatter (`seoTitle` wins over `title` for <title> and the share cards). */
export async function pageMetadata(slug: PageSlug, override: { title?: string; description?: string; url?: string } = {}): Promise<Metadata> {
  const { title, seoTitle, description } = (await getPage(slug)).frontmatter;
  return socialMeta({
    title: override.title ?? seoTitle ?? title,
    description: override.description ?? description,
    url: override.url ?? `/${slug}`,
  });
}

/** Every build-log post, newest first (by date, then slug descending). */
export const getLogPosts = cache(async (): Promise<LogPost[]> => {
  const posts = await Promise.all(
    listMdxSlugs("log").map(async (slug) => {
      const { frontmatter: fm } = await loadMdx<LogFrontmatter>(`log/${slug}`);
      if (!fm?.title || !fm.date) throw new Error(`content/log/${slug}.mdx needs a "title" and a "date" in its frontmatter.`);
      return { ...fm, slug, href: `/docs/log/${slug}` };
    }),
  );
  return posts.sort((a, b) => b.date.localeCompare(a.date) || b.slug.localeCompare(a.slug));
});

export const getLogPost = cache(async (slug: string) => {
  if (!listMdxSlugs("log").includes(slug)) return null;
  return loadMdx<LogFrontmatter>(`log/${slug}`);
});

