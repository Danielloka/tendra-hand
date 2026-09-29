import { Children, Fragment, isValidElement, type ReactNode } from "react";
import { DocsToc } from "@/components/docs/DocsToc";
import { H2 } from "@/components/mdx/Heading";
import type { TocItem } from "@/lib/docs";
import { getPage, type PageSlug } from "./content";
import { PageHero } from "./PageHero";
import "@/components/docs/docs.css"; // .toc styles for DocsToc
import "./pages.css";

type Props = {
  slug: PageSlug;
  /** "On this page" navigation: a sticky column on wide screens, a collapsible list on small ones. */
  toc?: boolean;
  /** Buttons in the hero. */
  actions?: ReactNode;
  /** Extra hero content under the intro (e.g. a stats row). */
  heroExtra?: ReactNode;
  /** Rendered after the MDX sections, in its own white band (gallery grid, log list). */
  children?: ReactNode;
};

type Band = { id?: string; title: string; nodes: ReactNode[] };

/** Plain text of a rendered heading (for the TOC). */
function textOf(node: ReactNode): string {
  if (node == null || typeof node === "boolean") return "";
  if (typeof node === "string" || typeof node === "number") return String(node);
  if (Array.isArray(node)) return node.map(textOf).join("");
  if (isValidElement<{ children?: ReactNode }>(node)) return textOf(node.props.children);
  return "";
}

const isBlank = (n: ReactNode) => n == null || typeof n === "boolean" || (typeof n === "string" && !n.trim());

/**
 * Render the MDX body and split its top-level nodes at every `##` heading, so
 * each section can sit in its own full-width band. The compiled MDX component
 * is a plain function that returns a fragment of the page's top-level elements
 * (@next/mdx gets its components from mdx-components.tsx, not from context),
 * so calling it here is safe in a server component.
 */
function splitSections(Content: (props: object) => ReactNode): { intro: ReactNode[]; bands: Band[] } {
  const tree = Content({});
  const nodes = isValidElement<{ children?: ReactNode }>(tree) && tree.type === Fragment ? Children.toArray(tree.props.children) : [tree];
  const intro: ReactNode[] = [];
  const bands: Band[] = [];
  for (const node of nodes) {
    if (isValidElement<{ id?: string; children?: ReactNode }>(node) && node.type === H2) {
      bands.push({ id: node.props.id, title: textOf(node.props.children), nodes: [node] });
    } else if (bands.length) bands.at(-1)!.nodes.push(node);
    else if (!isBlank(node)) intro.push(node);
  }
  return { intro, bands };
}

/**
 * Shared layout for the content pages (content/pages/<slug>.mdx): a hero from
 * the frontmatter, any text before the first `##` as the hero's intro, then one
 * band per section, alternating grey and white.
 */
export async function ContentPage({ slug, toc = false, actions, heroExtra, children }: Props) {
  const { default: Content, frontmatter: fm } = await getPage(slug);
  const { intro, bands } = splitSections(Content as (props: object) => ReactNode);
  const tocItems: TocItem[] = toc ? bands.filter((b) => b.id).map((b) => ({ id: b.id!, text: b.title, depth: 2 })) : [];

  return (
    <div className={`page page--${slug}`}>
      <PageHero kicker={fm.kicker} title={fm.title} lead={fm.lead} actions={actions}>
        {intro.length > 0 && <div className="prose page-hero__intro">{intro}</div>}
        {toc && (
          <div className="page-hero__toc">
            <DocsToc items={tocItems} variant="inline" />
          </div>
        )}
        {heroExtra}
      </PageHero>

      {bands.length > 0 && (
        <div className={`page-body${toc ? " page-body--toc" : ""}`}>
          {bands.map((band, i) => (
            <section key={band.id ?? i} data-band={band.id} className={`page-band${i % 2 === 0 ? " section--alt" : ""}`} aria-labelledby={band.id}>
              <div className="container">
                <div className="prose page-prose">{band.nodes}</div>
              </div>
            </section>
          ))}
          {toc && (
            <div className="page-toc">
              <div className="container page-toc__inner">
                <DocsToc items={tocItems} />
              </div>
            </div>
          )}
        </div>
      )}

      {children && <div className="page-extra">{children}</div>}
    </div>
  );
}
