import type { MetadataRoute } from "next";
import { getLogPosts } from "@/components/pages/content";
import { docHref, getAllDocSlugs } from "@/lib/docs";
import { docsLoc, mainLoc } from "@/components/seo/urls";

// Every public route. /dev/* (internal tools) and /styleguide are left out on purpose.
export default async function sitemap(): Promise<MetadataRoute.Sitemap> {
  const pages = ["", "/project", "/journey", "/gallery", "/contribute"];
  const docsPages = ["/docs", "/docs/hardware", "/docs/software", "/docs/log"];
  const posts = await getLogPosts();

  return [
    ...pages.map((path) => ({ url: mainLoc(path), changeFrequency: "weekly" as const, priority: path === "" ? 1 : 0.8 })),
    ...docsPages.map((path) => ({ url: docsLoc(path), changeFrequency: "weekly" as const, priority: 0.8 })),
    ...getAllDocSlugs().map((slug) => ({ url: docsLoc(docHref(slug)), changeFrequency: "monthly" as const, priority: 0.6 })),
    ...posts.map((post) => ({ url: docsLoc(post.href), lastModified: post.date, changeFrequency: "yearly" as const, priority: 0.5 })),
  ];
}
