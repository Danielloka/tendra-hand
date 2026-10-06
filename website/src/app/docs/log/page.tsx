import { ContentPage } from "@/components/pages/ContentPage";
import { getLogPosts, getPage } from "@/components/pages/content";
import { LogCard } from "@/components/pages/LogCard";
import { JsonLd } from "@/components/seo/JsonLd";
import { docsPageMetadata } from "@/components/seo/meta";
import { breadcrumbs, collectionPage, docsCrumbs } from "@/components/seo/schemas";
import { docsLoc } from "@/components/seo/urls";

export const generateMetadata = () => docsPageMetadata("log", "/docs/log");

export default async function LogIndexPage() {
  const posts = await getLogPosts();
  const { title, description } = (await getPage("log")).frontmatter;
  const url = docsLoc("/docs/log");
  return (
    <>
      <JsonLd
        data={[
          collectionPage({ name: title, description, url, items: posts.map((p) => ({ name: p.title, url: docsLoc(p.href) })) }),
          breadcrumbs(docsCrumbs({ name: title, url })),
        ]}
      />
      <ContentPage slug="log">
        <div className="container">
          <ol className="log-list" reversed>
            {posts.map((post, i) => (
              <li key={post.slug}>
                <LogCard post={post} featured={i === 0} />
              </li>
            ))}
          </ol>
        </div>
      </ContentPage>
    </>
  );
}
