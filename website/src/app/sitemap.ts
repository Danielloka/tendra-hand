import type { MetadataRoute } from "next";
import { getLogPosts } from "@/components/pages/content";
import { docHref, getAllDocSlugs } from "@/lib/docs";
import { docsAbsolute, docsUrl, siteConfig } from "@/lib/site";

// Every public route. /dev/* (internal tools) and /styleguide are left out on purpose.
export default async function sitemap(): Promise<MetadataRoute.Sitemap> {
  const url = (path: string) => `${siteConfig.url}${path}`;
  const pages = ["", "/project", "/journey", "/gallery", "/contribute"];
  const docsPages = ["/docs", "/docs/hardware", "/docs/software", "/docs/log"];
  const posts = await getLogPosts();

  // Docs pages sit at docs.<domain>/<page> once NEXT_PUBLIC_DOCS_URL is set, else at /docs/<page>.
  const docsUrlFor = (path: string) => (docsUrl ? docsAbsolute(path) : url(path));

  return [
    ...pages.map((path) => ({ url: url(path), changeFrequency: "weekly" as const, priority: path === "" ? 1 : 0.8 })),
    ...docsPages.map((path) => ({ url: docsUrlFor(path), changeFrequency: "weekly" as const, priority: 0.8 })),
    ...getAllDocSlugs().map((slug) => ({ url: docsUrlFor(docHref(slug)), changeFrequency: "monthly" as const, priority: 0.6 })),
    ...posts.map((post) => ({ url: docsUrlFor(post.href), lastModified: post.date, changeFrequency: "yearly" as const, priority: 0.5 })),
  ];
}
