import { Reveal } from "./Reveal";

export function SectionHead({ kicker, title, lead, center, as: H = "h2" }: { kicker?: string; title: string; lead?: React.ReactNode; center?: boolean; as?: "h1" | "h2" }) {
  return (
    <header className={`section-head${center ? " section-head--center" : ""}`}>
      {kicker && <p className="kicker">{kicker}</p>}
      <Reveal as={H}>{title}</Reveal>
      {lead && (
        <Reveal as="p" className="section-head__lead" delay={120}>
          {lead}
        </Reveal>
      )}
    </header>
  );
}
