import { existsSync, readdirSync, readFileSync } from "node:fs";
import { join } from "node:path";
import GithubSlugger, { slug as slugify } from "github-slugger";
import { cache } from "react";
import nav from "@content/docs/_nav.json";
import { loadMdx } from "./mdx";

/**
 * Docs tree, table of contents and search index (server only).
 *
 * content/docs/_nav.json sets the sidebar groups and page order. Every page is
 * content/docs/<slug>.mdx with frontmatter: title, description, updated? (YYYY-MM-DD).
 */

export type DocFrontmatter = { title: string; description: string; updated?: string };
export type DocLink = { slug: string; href: string; title: string; description: string; group: string };
export type DocGroup = { title: string; pages: DocLink[] };
export type TocItem = { id: string; text: string; depth: 2 | 3 };

/** Compact search record: slug, title, description, sections [id, heading, text]. */
export type SearchDoc = { s: string; t: string; d: string; h: [id: string, heading: string, text: string][] };

type NavFile = { groups: { title: string; pages: string[] }[] };

const DOCS_DIR = join(process.cwd(), "content", "docs");

export const docHref = (slug: string) => `/docs/${slug}`;

function checkNav(file: NavFile): NavFile {
  const seen = new Set<string>();
  for (const group of file.groups) {
    for (const slug of group.pages) {
      if (seen.has(slug)) throw new Error(`content/docs/_nav.json lists "${slug}" twice.`);
      seen.add(slug);
      if (!existsSync(join(DOCS_DIR, `${slug}.mdx`))) {
        throw new Error(`content/docs/_nav.json lists "${slug}", but content/docs/${slug}.mdx doesn't exist. Add the file or remove the entry.`);
      }
    }
  }
  return file;
}

const navFile = checkNav(nav as NavFile);

/** Every doc slug: the ones in _nav.json first, then any unlisted files (built, but not in the sidebar). */
export function getAllDocSlugs(): string[] {
  const listed = navFile.groups.flatMap((g) => g.pages);
  const files = readdirSync(DOCS_DIR)
    .filter((f) => f.endsWith(".mdx") && !f.startsWith("_"))
    .map((f) => f.replace(/\.mdx$/, ""));
  return [...listed, ...files.filter((f) => !listed.includes(f))];
}

export const getDoc = cache(async (slug: string) => {
  if (!getAllDocSlugs().includes(slug)) return null;
  const mod = await loadMdx<DocFrontmatter>(`docs/${slug}`);
  if (!mod.frontmatter?.title) throw new Error(`content/docs/${slug}.mdx needs a "title" in its frontmatter.`);
  return mod;
});

/** Sidebar groups with each page's title and description. */
export const getDocsTree = cache(async (): Promise<DocGroup[]> => {
  return Promise.all(
    navFile.groups.map(async (group) => ({
      title: group.title,
      pages: await Promise.all(
        group.pages.map(async (slug) => {
          const { frontmatter: fm } = (await getDoc(slug))!;
          return { slug, href: docHref(slug), title: fm.title, description: fm.description ?? "", group: group.title };
        }),
      ),
    })),
  );
});

/** Previous/next page in sidebar order (null at the ends, or for unlisted pages). */
export async function getPrevNext(slug: string) {
  const pages = (await getDocsTree()).flatMap((g) => g.pages);
  const i = pages.findIndex((p) => p.slug === slug);
  if (i === -1) return { page: null, prev: null, next: null };
  return { page: pages[i], prev: pages[i - 1] ?? null, next: pages[i + 1] ?? null };
}

// ---- Parsing the raw MDX (for the TOC and search) ---------------------------

type Section = { id: string; heading: string; depth: number; lines: string[] };

function readSource(slug: string): string {
  return readFileSync(join(DOCS_DIR, `${slug}.mdx`), "utf8")
    .replace(/\r\n?/g, "\n")
    .replace(/^---\n[\s\S]*?\n---\n/, "") // frontmatter
    .replace(/\{\/\*[\s\S]*?\*\/\}/g, ""); // MDX comments
}

/** Markdown inline syntax → plain text (same text rehype-slug sees for a heading). */
function inlineText(s: string): string {
  return s
    .replace(/!\[[^\]]*\]\([^)]*\)/g, "") // images
    .replace(/\[([^\]]+)\]\([^)]*\)/g, "$1") // links
    .split(/(`[^`]*`)/) // odd parts are inline code: keep their text as is
    .map((part, i) =>
      i % 2
        ? part.slice(1, -1)
        : part
            .replace(/(\*\*|\*|~~)(?=\S)(.+?)(?<=\S)\1/g, "$2") // emphasis
            .replace(/(^|[^\w])(__|_)(?=\S)(.+?)(?<=\S)\2(?![\w])/g, "$1$3") // _emphasis_ (not snake_case)
            .replace(/<[^>]*>/g, " ") // JSX/HTML tags
            .replace(/\\([\\`*_{}[\]()#+\-.!|<>])/g, "$1"), // escapes
    )
    .join("")
    .trim();
}

const HEADING = /^(#{1,6})\s+(.+?)\s*#*\s*$/;
const STEP = /<Step\b[^>]*\btitle=(?:"([^"]*)"|'([^']*)'|\{\s*["'`]([^"'`]*)["'`]\s*\})/;
const ATTR_TEXT = /\b(?:title|caption)=(?:"([^"]*)"|'([^']*)')/g;

/**
 * Split a doc into sections at h2/h3 headings and <Step title> (h3). Ids match
 * what the page renders: rehype-slug (GitHub slugs, deduplicated over every
 * markdown heading) and "step-<slug>" for steps.
 */
const parseDoc = cache((slug: string): Section[] => {
  const slugger = new GithubSlugger();
  const sections: Section[] = [{ id: "", heading: "", depth: 1, lines: [] }];
  let fence: string | null = null;

  for (const line of readSource(slug).split("\n")) {
    const fenceMark = line.match(/^\s*(`{3,}|~{3,})/)?.[1];
    if (fence) {
      if (fenceMark && fenceMark[0] === fence[0] && fenceMark.length >= fence.length) fence = null;
      else sections.at(-1)!.lines.push(line); // code is searchable too
      continue;
    }
    if (fenceMark) {
      fence = fenceMark;
      continue;
    }

    const h = line.match(HEADING);
    if (h) {
      const text = inlineText(h[2]);
      const id = slugger.slug(text);
      const depth = h[1].length;
      if (depth === 2 || depth === 3) sections.push({ id, heading: text, depth, lines: [] });
      else sections.at(-1)!.lines.push(text);
      continue;
    }

    const step = line.match(STEP);
    if (step) {
      const text = step[1] ?? step[2] ?? step[3];
      sections.push({ id: `step-${slugify(text)}`, heading: text, depth: 3, lines: [] });
      continue;
    }

    sections.at(-1)!.lines.push(line);
  }
  return sections;
});

/** h2/h3 (and steps) for the "On this page" list. */
export function getToc(slug: string): TocItem[] {
  return parseDoc(slug)
    .filter((s) => s.id)
    .map((s) => ({ id: s.id, text: s.heading, depth: s.depth as 2 | 3 }));
}

/** Body lines → one line of plain text. */
function plainText(lines: string[]): string {
  return lines
    .filter((l) => !/^\s*(import|export)\s/.test(l) && !/^\s*\|?\s*:?-{3,}/.test(l))
    .map((l) => {
      const attrs = [...l.matchAll(ATTR_TEXT)].map((m) => m[1] ?? m[2]).join(" ");
      const text = inlineText(l.replace(/^\s*(>\s*)+/, "").replace(/^\s*([-*+]|\d+\.)\s+(\[[ x]\]\s+)?/, ""));
      return `${attrs} ${text.replace(/\|/g, " ").replace(/[{}]/g, "")}`;
    })
    .join(" ")
    .replace(/\s+/g, " ")
    .trim();
}

/** Search records for every doc, built at build time and served from /docs/search-index.json. */
export async function buildSearchIndex(): Promise<SearchDoc[]> {
  return Promise.all(
    getAllDocSlugs().map(async (slug) => {
      const { frontmatter: fm } = (await getDoc(slug))!;
      const h = parseDoc(slug)
        .map((s) => [s.id, s.heading, plainText(s.lines)] as [string, string, string])
        .filter(([id, , text]) => id || text);
      return { s: slug, t: fm.title, d: fm.description ?? "", h };
    }),
  );
}
