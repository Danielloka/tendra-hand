import Link from "next/link";

/** Responsive grid of <Card>s. */
export function CardGrid({ children, min = "15rem" }: { children: React.ReactNode; min?: string }) {
  return (
    <div className="auto-grid not-prose prose-cards" style={{ "--grid-min": min } as React.CSSProperties}>
      {children}
    </div>
  );
}

// 24px line icons for <Card icon="…">, drawn with currentColor.
const ICONS = {
  tendon: <path d="M4 7c5 0 5 5 8 5s3-5 8-5M4 17c5 0 5-5 8-5s3 5 8 5" />,
  motor: (
    <>
      <circle cx="12" cy="12" r="3" />
      <path d="M12 3v3M12 18v3M3 12h3M18 12h3M5.6 5.6l2.1 2.1M16.3 16.3l2.1 2.1M18.4 5.6l-2.1 2.1M7.7 16.3l-2.1 2.1" />
    </>
  ),
  servo: (
    <>
      <rect x="4" y="8" width="12" height="9" rx="2" />
      <path d="M16 12.5h4M8 8V5h4v3" />
    </>
  ),
  chip: (
    <>
      <rect x="7" y="7" width="10" height="10" rx="2" />
      <path d="M10 3v4M14 3v4M10 17v4M14 17v4M3 10h4M3 14h4M17 10h4M17 14h4" />
    </>
  ),
} as const;
export type CardIcon = keyof typeof ICONS;

type CardProps = { icon?: CardIcon; title: string; href?: string; kicker?: string; more?: string; children?: React.ReactNode };

/** Design-system card. With `href` the whole card is a link (internal links use next/link). */
export function Card({ icon, title, href, kicker, more = "Read more", children }: CardProps) {
  const inner = (
    <>
      {icon && (
        <span className="card__icon" aria-hidden="true">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round">
            {ICONS[icon]}
          </svg>
        </span>
      )}
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
