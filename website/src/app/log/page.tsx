import { ContentPage } from "@/components/pages/ContentPage";
import { getLogPosts, pageMetadata } from "@/components/pages/content";
import { LogCard } from "@/components/pages/LogCard";

export const generateMetadata = () => pageMetadata("log");

export default async function LogIndexPage() {
  const posts = await getLogPosts();
  return (
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
  );
}
