import { slug } from "github-slugger";

/** Numbered list of steps (numbers come from a CSS counter). */
export function Steps({ children }: { children: React.ReactNode }) {
  return <ol className="steps">{children}</ol>;
}

/**
 * One step. The title is an h3 with id "step-<slug>", so steps show up in the
 * on-page table of contents and can be linked to.
 */
export function Step({ title, children }: { title: string; children?: React.ReactNode }) {
  const id = `step-${slug(title)}`;
  return (
    <li className="steps__item">
      <h3 id={id} className="steps__title">
        {title}
        <a className="heading-anchor" href={`#${id}`} aria-label="Link to this step">
          #
        </a>
      </h3>
      <div className="steps__body">{children}</div>
    </li>
  );
}
