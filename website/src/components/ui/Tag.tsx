type Tone = "neutral" | "accent" | "success" | "warm";

export function Tag({ tone = "neutral", dot, live, children }: { tone?: Tone; dot?: boolean; live?: boolean; children: React.ReactNode }) {
  const cls = ["tag", tone !== "neutral" && `tag--${tone}`, live && "tag--live"].filter(Boolean).join(" ");
  return (
    <span className={cls}>
      {(dot || live) && <span className="tag__dot" aria-hidden="true" />}
      {children}
    </span>
  );
}

export type Status = "done" | "in-progress" | "planned";
const STATUS: Record<Status, { tone: Tone; label: string }> = {
  done: { tone: "success", label: "Done" },
  "in-progress": { tone: "warm", label: "In progress" },
  planned: { tone: "neutral", label: "Planned" },
};

/** Status badge used across the site: done / in progress / planned. */
export function StatusBadge({ status, children }: { status: Status; children?: React.ReactNode }) {
  const s = STATUS[status];
  return (
    <Tag tone={s.tone} dot live={status === "in-progress"}>
      {children ?? s.label}
    </Tag>
  );
}
