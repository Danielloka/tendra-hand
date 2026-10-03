import type { MetadataRoute } from "next";
import { getLogPosts } from "@/components/pages/content";
import { docHref, getAllDocSlugs } from "@/lib/docs";
import { siteConfig } from "@/lib/site";

// Every public route. /dev/* (internal tools) and /styleguide are left out on purpose.
export default async function sitemap(): Promise<MetadataRoute.Sitemap> {
  const url = (path: string) => `${siteConfig.url}${path}`;
  const pages = ["", "/project", "/journey", "/hardware", "/software", "/docs", "/log", "/gallery", "/contribute"];
  const posts = await getLogPosts();

  return [
    ...pages.map((path) => ({ url: url(path), changeFrequency: "weekly" as const, priority: path === "" ? 1 : 0.8 })),
    ...getAllDocSlugs().map((slug) => ({ url: url(docHref(slug)), changeFrequency: "monthly" as const, priority: 0.6 })),
    ...posts.map((post) => ({ url: url(post.href), lastModified: post.date, changeFrequency: "yearly" as const, priority: 0.5 })),
  ];
}
