import type { MetadataRoute } from "next";
import { getLogPosts, getPage, type PageSlug } from "@/components/pages/content";
import { gitLastModified, newest } from "@/components/seo/lastmod";
import { docsLoc, mainLoc } from "@/components/seo/urls";
import { docHref, getAllDocSlugs, getDoc } from "@/lib/docs";

/** Last change of a content page: its frontmatter `updated`, else its last git commit, else unknown (never "now"). */
async function pageDate(slug: PageSlug, ...extraFiles: string[]) {
  const { updated } = (await getPage(slug)).frontmatter;
  return updated ?? newest(gitLastModified(`content/pages/${slug}.mdx`), ...extraFiles.map(gitLastModified));
}

// Every public route. /dev/* (internal tools) and /styleguide are left out on purpose.
// changeFrequency and priority are left out too: Google ignores them.
export default async function sitemap(): Promise<MetadataRoute.Sitemap> {
  const posts = await getLogPosts();
  const main: [string, string | undefined][] = [
    ["", newest(gitLastModified("content/home.json"), gitLastModified("content/site.json"))],
    ["/project", await pageDate("project")],
    ["/journey", gitLastModified("content/journey.json")],
    ["/gallery", await pageDate("gallery", "content/data/gallery.json")],
    ["/contact", await pageDate("contact")],
  ];
  const docsPages: [string, string | undefined][] = [
    ["/docs", newest(...getAllDocSlugs().map((s) => gitLastModified(`content/docs/${s}.mdx`)))],
    ["/docs/hardware", await pageDate("hardware")],
    ["/docs/software", await pageDate("software")],
    ["/docs/log", newest(...posts.map((p) => p.date))],
  ];
  const docs = await Promise.all(
    getAllDocSlugs().map(async (slug) => ({
      url: docsLoc(docHref(slug)),
      lastModified: (await getDoc(slug))?.frontmatter.updated ?? gitLastModified(`content/docs/${slug}.mdx`),
    })),
  );

  return [
    ...main.map(([path, lastModified]) => ({ url: mainLoc(path), lastModified })),
    ...docsPages.map(([path, lastModified]) => ({ url: docsLoc(path), lastModified })),
    ...docs,
    ...posts.map((post) => ({ url: docsLoc(post.href), lastModified: post.date })),
  ];
}
