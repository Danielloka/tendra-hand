import Link from "next/link";

/** Responsive grid of <Card>s. */
export function CardGrid({ children, min = "15rem" }: { children: React.ReactNode; min?: string }) {
  return (
    <div className="auto-grid not-prose prose-cards" style={{ "--grid-min": min } as React.CSSProperties}>
      {children}
    </div>
  );
}

type CardProps = { title: string; href?: string; kicker?: string; more?: string; children?: React.ReactNode };

/** Design-system card. With `href` the whole card is a link (internal links use next/link). */
export function Card({ title, href, kicker, more = "Read more", children }: CardProps) {
  const inner = (
    <>
      {kicker && <p className="card__kicker">{kicker}</p>}
      <h3 className="card__title">{title}</h3>
      {children && <div className="card__body">{children}</div>}
      {href && <span className="card__more">{more}</span>}
    </>
  );
  if (!href) return <div className="card">{inner}</div>;
  return href.startsWith("/") ? (
    <Link className="card" href={href}>
      {inner}
    </Link>
  ) : (
    <a className="card" href={href}>
      {inner}
    </a>
  );
}
