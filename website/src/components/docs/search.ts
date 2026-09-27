import type { SearchDoc } from "@/lib/docs";

export type SearchHit = {
  href: string;
  title: string;
  heading: string;
  /** Snippet split at matches: even items are plain text, odd items are highlighted. */
  snippet: string[];
  score: number;
};

const MAX_HITS = 8;
const MAX_PER_DOC = 3;
const SNIPPET = 140;

/** Lowercase and strip accents so "Calibracion" finds "Calibración". */
const norm = (s: string) => s.normalize("NFD").replace(/\p{M}/gu, "").toLowerCase();

const escapeRe = (s: string) => s.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");

/** Points for one term in one field: whole word > start of word > anywhere. */
function fieldScore(field: string, term: string, weight: number): number {
  const i = field.indexOf(term);
  if (i === -1) return 0;
  const startsWord = i === 0 || /\W/.test(field[i - 1]);
  const endsWord = i + term.length === field.length || /\W/.test(field[i + term.length]);
  return weight * (startsWord ? (endsWord ? 1.5 : 1.2) : 1);
}

/** ~140 characters around the first match, cut at word boundaries, split for highlighting. */
function makeSnippet(text: string, terms: string[]): string[] {
  const lower = norm(text);
  const first = Math.min(...terms.map((t) => lower.indexOf(t)).filter((i) => i >= 0));
  let start = Number.isFinite(first) ? Math.max(0, first - 40) : 0;
  if (start > 0) start = text.indexOf(" ", start) + 1 || start;
  let end = Math.min(text.length, start + SNIPPET);
  if (end < text.length) end = text.lastIndexOf(" ", end) > start ? text.lastIndexOf(" ", end) : end;
  const cut = `${start > 0 ? "… " : ""}${text.slice(start, end)}${end < text.length ? " …" : ""}`;

  // Split on the terms (accent-insensitive: match on the normalised copy, cut the original).
  // NFD can change string length, so only highlight when lengths agree.
  const cutNorm = norm(cut);
  if (cutNorm.length !== cut.length) return [cut];
  const re = new RegExp(terms.map(escapeRe).sort((a, b) => b.length - a.length).join("|"), "g");
  const parts: string[] = [];
  let last = 0;
  for (const m of cutNorm.matchAll(re)) {
    parts.push(cut.slice(last, m.index), cut.slice(m.index, m.index + m[0].length));
    last = m.index + m[0].length;
  }
  parts.push(cut.slice(last));
  return parts;
}

/**
 * Small scoring search over the docs index. Every term must appear somewhere
 * in the page section (title, heading, description or text); matches in titles
 * and headings count most.
 */
export function search(index: SearchDoc[], query: string): SearchHit[] {
  const terms = [...new Set(norm(query).split(/\s+/).filter(Boolean))];
  if (!terms.length) return [];

  const hits: SearchHit[] = [];
  for (const doc of index) {
    const title = norm(doc.t);
    const desc = norm(doc.d);
    const docHits: SearchHit[] = [];

    for (const [id, heading, text] of doc.h) {
      const h = norm(heading);
      const body = norm(text);
      let score = 0;
      let all = true;
      for (const term of terms) {
        const s =
          fieldScore(title, term, id ? 0.5 : 8) + // a title match is mostly about the page top
          fieldScore(h, term, 6) +
          (id ? 0 : fieldScore(desc, term, 3)) + // the description belongs to the page top
          fieldScore(body, term, 1) +
          Math.min(3, body.split(term).length - 1) * 0.3;
        if (s === 0) {
          all = false;
          break;
        }
        score += s;
      }
      if (!all) continue;
      if (!id) score += 0.5; // prefer the page itself when everything else is equal
      const source = text || (id ? "" : doc.d);
      docHits.push({
        href: `/docs/${doc.s}${id ? `#${id}` : ""}`,
        title: doc.t,
        heading,
        snippet: source ? makeSnippet(source, terms) : [],
        score,
      });
    }
    docHits.sort((a, b) => b.score - a.score);
    hits.push(...docHits.slice(0, MAX_PER_DOC));
  }
  return hits.sort((a, b) => b.score - a.score).slice(0, MAX_HITS);
}
