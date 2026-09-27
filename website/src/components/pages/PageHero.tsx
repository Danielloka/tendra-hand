import { Reveal } from "@/components/ui/Reveal";
import "./pages.css";

type Props = {
  kicker?: string;
  title: string;
  lead?: React.ReactNode;
  /** Buttons under the lead (a page has at most one primary button). */
  actions?: React.ReactNode;
  /** Extra content under the lead: intro text, meta line, inline TOC. */
  children?: React.ReactNode;
  /** Small line above the kicker, e.g. a back link. */
  top?: React.ReactNode;
  narrow?: boolean;
};

/** Left-aligned page header shared by every content page, log post and the 404 page. */
export function PageHero({ kicker, title, lead, actions, children, top, narrow }: Props) {
  return (
    <header className={`page-hero${narrow ? " page-hero--narrow" : ""}`}>
      <div className="container page-hero__inner">
        {top}
        {kicker && <p className="kicker">{kicker}</p>}
        <h1 className="page-hero__title">{title}</h1>
        {lead && (
          <Reveal as="p" className="lead page-hero__lead" delay={80}>
            {lead}
          </Reveal>
        )}
        {actions && (
          <Reveal className="page-hero__actions" delay={160}>
            {actions}
          </Reveal>
        )}
        {children}
      </div>
    </header>
  );
}
