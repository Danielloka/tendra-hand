import Link from "next/link";
import type { DocLink } from "@/lib/docs";
import { siteConfig } from "@/lib/site";

function formatDate(iso: string) {
  const d = new Date(`${iso}T00:00:00Z`);
  return Number.isNaN(d.getTime()) ? iso : d.toLocaleDateString("en-GB", { day: "numeric", month: "long", year: "numeric", timeZone: "UTC" });
}

/** Doc footer: "Edit on GitHub", last updated, and previous / next page. */
export function DocsPager({ slug, prev, next, updated }: { slug: string; prev: DocLink | null; next: DocLink | null; updated?: string }) {
  return (
    <footer className="docs-footer">
      <div className="docs-footer__meta">
        <a className="docs-footer__edit" href={`${siteConfig.github}/edit/main/website/content/docs/${slug}.mdx`}>
          <svg viewBox="0 0 24 24" width="15" height="15" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
            <path d="M12 20h9M16.5 3.5a2.1 2.1 0 0 1 3 3L7 19l-4 1 1-4Z" />
          </svg>
          Edit this page on GitHub
        </a>
        {updated && (
          <p className="caption">
            Last updated <time dateTime={updated}>{formatDate(updated)}</time>
          </p>
        )}
      </div>

      {(prev || next) && (
        <nav className="docs-pager" aria-label="Previous and next page">
          {prev ? (
            <Link href={prev.href} className="docs-pager__link" rel="prev">
              <span className="docs-pager__dir">‹ Previous</span>
              <span className="docs-pager__title">{prev.title}</span>
            </Link>
          ) : (
            <span />
          )}
          {next && (
            <Link href={next.href} className="docs-pager__link docs-pager__link--next" rel="next">
              <span className="docs-pager__dir">Next ›</span>
              <span className="docs-pager__title">{next.title}</span>
            </Link>
          )}
        </nav>
      )}
    </footer>
  );
}
