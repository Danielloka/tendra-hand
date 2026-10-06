import { readFileSync } from "node:fs";
import { join } from "node:path";
import { docsLoc, mainLoc } from "./urls";

/**
 * Reads content/<path>.mdx and turns it into plain Markdown (for llms-full.txt and the FAQ schema):
 * frontmatter, imports and comments are dropped, the few MDX components become Markdown.
 */
export type RawDoc = { frontmatter: Record<string, string>; body: string };

const attr = (tag: string, name: string) => tag.match(new RegExp(`\\b${name}=(?:"([^"]*)"|'([^']*)')`))?.slice(1).find(Boolean);

export function readMdx(path: string): RawDoc {
  const raw = readFileSync(join(process.cwd(), "content", `${path}.mdx`), "utf8").replace(/\r\n?/g, "\n");
  const fm: Record<string, string> = {};
  const m = raw.match(/^---\n([\s\S]*?)\n---\n/);
  for (const line of (m?.[1] ?? "").split("\n")) {
    const kv = line.match(/^(\w+):\s*(.*)$/);
    if (kv) fm[kv[1]] = kv[2].replace(/^["'](.*)["']$/, "$1");
  }
  return { frontmatter: fm, body: toMarkdown(m ? raw.slice(m[0].length) : raw) };
}

const MOVED: Record<string, string> = { "/hardware": "/docs/hardware", "/software": "/docs/software", "/log": "/docs/log" };

/** A site path from the content ("/docs/faq", "/log/x") as the public absolute address. */
function absolute(path: string): string {
  const p = MOVED[path] ?? path.replace(/^\/log\//, "/docs/log/");
  return p.startsWith("/docs") ? docsLoc(p) : mainLoc(p);
}

export function toMarkdown(src: string): string {
  let inFence = false;
  // line index of the open multi-line <Card> / <Callout>: its text is appended there
  let block: number | null = null;
  const out: string[] = [];
  for (const line of src.replace(/\{\/\*[\s\S]*?\*\/\}/g, "").split("\n")) {
    if (/^\s*(`{3,}|~{3,})/.test(line)) {
      inFence = !inFence;
      out.push(line.replace(/\s+title="[^"]*"/, ""));
      continue;
    }
    if (inFence) {
      out.push(line);
      continue;
    }
    if (/^(import|export)\s/.test(line)) continue;
    const t = line.trim();
    if (/^<(Steps|CardGrid|Roadmap|BomTable)\b[^>]*\/?>$/.test(t) || /^<\/(Steps|CardGrid|Step)>$/.test(t)) continue;
    if (/^<\/(Card|Callout)>$/.test(t)) {
      block = null;
      continue;
    }
    let tag = t.match(/^<Step\b[^>]*>/);
    if (tag) {
      out.push("", `### ${attr(tag[0], "title") ?? "Step"}`, t.slice(tag[0].length).replace(/<\/Step>$/, ""));
      continue;
    }
    tag = t.match(/^<(Callout|Card)\b[^>]*>/);
    if (tag) {
      const callout = tag[1] === "Callout";
      const title = attr(tag[0], "title") ?? (callout ? (attr(tag[0], "type") ?? "Note") : "");
      const href = attr(tag[0], "href");
      const label = callout ? title[0].toUpperCase() + title.slice(1) : href ? `[${title}](${href})` : title;
      const rest = t.slice(tag[0].length);
      const closed = new RegExp(`</${tag[1]}>$`).test(rest);
      out.push(`${callout ? "> " : "- "}**${label}${callout ? ":" : ""}**${callout ? "" : ":"} ${rest.replace(/<\/(Callout|Card)>$/, "")}`.trimEnd());
      if (!closed) block = out.length - 1;
      continue;
    }
    tag = t.match(/^<Figure\b[^>]*\/?>/);
    if (tag) {
      const cap = attr(tag[0], "caption") ?? attr(tag[0], "alt");
      if (cap) out.push(`*Figure: ${cap}*`);
      continue;
    }
    if (block !== null) {
      if (t) out[block] += ` ${t.replace(/<\/(Callout|Card)>$/, "")}`;
      continue;
    }
    out.push(line.replace(/^ {2}(?=\S)/, ""));
  }
  return out
    .join("\n")
    .replace(/<StatusBadge status="([^"]*)"\s*\/>/g, (_, st: string) => st.replace(/-/g, " "))
    .replace(/<[A-Z][A-Za-z]*\b[^>]*\/>/g, "")
    .replace(/\]\((\/[^)#\s]*)/g, (_, path: string) => `](${absolute(path)}`)
    .replace(/\n{3,}/g, "\n\n")
    .trim();
}

/** Markdown inline syntax to plain text. */
export function plain(md: string): string {
  return md
    .replace(/!\[[^\]]*\]\([^)]*\)/g, "")
    .replace(/\[([^\]]+)\]\([^)]*\)/g, "$1")
    .replace(/\*\*([^*]+)\*\*/g, "$1")
    .replace(/(^|\W)\*([^*\s][^*]*)\*/g, "$1$2")
    .replace(/`([^`]*)`/g, "$1")
    .replace(/^\s*>\s?/gm, "")
    .replace(/^\s*[-*]\s+/gm, "")
    .replace(/\s*\n\s*/g, " ")
    .trim();
}

/** Splits Markdown at "## " headings (fence-aware): [heading, body] pairs. */
export function h2Sections(md: string): [string, string][] {
  const res: [string, string[]][] = [];
  let fence = false;
  for (const line of md.split("\n")) {
    if (/^\s*(`{3,}|~{3,})/.test(line)) fence = !fence;
    const h = !fence && line.match(/^##\s+(.+?)\s*$/);
    if (h) res.push([h[1], []]);
    else res.at(-1)?.[1].push(line);
  }
  return res.map(([h, b]) => [h, b.join("\n").trim()]);
}
