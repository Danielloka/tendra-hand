type CalloutType = "note" | "tip" | "warning";

const DEFAULT_TITLE: Record<CalloutType, string> = { note: "Note", tip: "Tip", warning: "Warning" };

// 24px line icons, drawn with currentColor.
const ICON: Record<CalloutType, React.ReactNode> = {
  note: (
    <>
      <circle cx="12" cy="12" r="9" />
      <path d="M12 11v5M12 8h.01" />
    </>
  ),
  tip: (
    <>
      <path d="M9 18h6M10 21h4" />
      <path d="M12 3a6 6 0 0 0-3.6 10.8c.6.5 1 1.2 1.1 2V16h5v-.2c.1-.8.5-1.5 1.1-2A6 6 0 0 0 12 3Z" />
    </>
  ),
  warning: (
    <>
      <path d="M10.3 4.2 2.6 17.5A2 2 0 0 0 4.3 20.5h15.4a2 2 0 0 0 1.7-3L13.7 4.2a2 2 0 0 0-3.4 0Z" />
      <path d="M12 10v4M12 17h.01" />
    </>
  ),
};

/** Highlighted aside: note (neutral), tip (green) or warning (amber). */
export function Callout({ type = "note", title, children }: { type?: CalloutType; title?: string; children: React.ReactNode }) {
  return (
    <aside className={`callout callout--${type}`} aria-label={title ?? DEFAULT_TITLE[type]}>
      <svg className="callout__icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
        {ICON[type]}
      </svg>
      <div className="callout__body">
        <p className="callout__title">{title ?? DEFAULT_TITLE[type]}</p>
        {children}
      </div>
    </aside>
  );
}
