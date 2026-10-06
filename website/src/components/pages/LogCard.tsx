import Image from "next/image";
import Link from "next/link";
import { Tag } from "@/components/ui/Tag";
import type { LogPost } from "./content";
import { formatDate } from "./format";
import "./pages.css";

/** Date · tags line used on log cards and post pages. */
export function PostMeta({ date, tags = [] }: { date: string; tags?: string[] }) {
  return (
    <div className="post-meta">
      <time dateTime={date}>{formatDate(date)}</time>
      {tags.length > 0 && (
        <ul className="post-meta__tags" aria-label="Tags">
          {tags.map((tag) => (
            <li key={tag}>
              <Tag>{tag}</Tag>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}

/** Build-log card: cover, date, title, summary, tags. `featured` = wide layout for the newest post. */
export function LogCard({ post, featured = false }: { post: LogPost; featured?: boolean }) {
  return (
    <Link href={post.href} className={`card log-card${featured ? " log-card--featured" : ""}`}>
      {post.cover && (
        <div className="card__media">
          <Image src={post.cover} alt="" fill sizes={featured ? "(min-width: 64rem) 40rem, 100vw" : "(min-width: 52rem) 24rem, 100vw"} priority={featured} />
        </div>
      )}
      <div className="log-card__text">
        <PostMeta date={post.date} tags={post.tags} />
        <h2 className="log-card__title">{post.title}</h2>
        <p className="card__body">{post.summary}</p>
        <span className="card__more">Read the post</span>
      </div>
    </Link>
  );
}
