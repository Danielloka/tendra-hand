/**
 * Clearly-marked stand-in for a photo, render or video that doesn't exist yet.
 * Every usage must carry a TODO comment (or `todo` field in content) saying
 * what the real media should show.
 */
export function Placeholder({ label, ratio = "16 / 10", className = "" }: { label: string; ratio?: string; className?: string }) {
  return (
    <div
      role="img"
      aria-label={`Placeholder: ${label}`}
      className={`grid place-items-center rounded-lg bg-bg-alt p-6 text-center ${className}`}
      style={{ aspectRatio: ratio, backgroundImage: "radial-gradient(ellipse 60% 70% at 50% 60%, var(--accent-soft), transparent 70%)" }}
    >
      <div className="grid justify-items-center gap-3">
        <svg width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="var(--text-faint)" strokeWidth="1.4" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
          <rect x="3" y="3" width="18" height="18" rx="3" />
          <circle cx="9" cy="9" r="2" />
          <path d="m21 15-5-5L5 21" />
        </svg>
        <p className="caption">{label}</p>
      </div>
    </div>
  );
}
