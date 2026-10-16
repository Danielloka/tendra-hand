import { githubUrl, siteConfig } from "@/lib/site";

const ICONS: Record<string, React.ReactNode> = {
  x: <path d="M4 4l16 16M20 4L4 20" />,
  instagram: (
    <>
      <rect x="4" y="4" width="16" height="16" rx="5" />
      <circle cx="12" cy="12" r="3.5" />
      <circle cx="16.8" cy="7.2" r="0.6" />
    </>
  ),
  email: (
    <>
      <rect x="3.5" y="5.5" width="17" height="13" rx="2.5" />
      <path d="M4 7.5l8 6 8-6" />
    </>
  ),
};

/** Social and email cards from site.json `contact.items`. An item without an href (not filled in yet) shows as a plain, non-clickable card. */
export function ContactCards() {
  const { items } = siteConfig.contact;
  return (
    <div className="container contact-cards">
      <div className="auto-grid" style={{ "--grid-min": "16rem" } as React.CSSProperties}>
        {items.map((item) => {
          const inner = (
            <>
              <span className="card__icon" aria-hidden="true">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round">
                  {ICONS[item.id]}
                </svg>
              </span>
              <h2 className="card__title">{item.label}</h2>
              <div className="card__body">
                <p>{item.blurb}</p>
                <p>{item.href ? item.handle : "Coming soon"}</p>
              </div>
              {item.href && <span className="card__more">Open</span>}
            </>
          );
          return item.href ? (
            <a key={item.id} className="card" href={item.href}>
              {inner}
            </a>
          ) : (
            <div key={item.id} className="card">
              {inner}
            </div>
          );
        })}
      </div>
      <p className="caption" style={{ marginTop: "2rem" }}>
        Everything is open on <a href={githubUrl} style={{ textDecoration: "underline" }}>GitHub</a>: the designs, the code and the research.
      </p>
    </div>
  );
}
