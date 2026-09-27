// Shiki for MDX code blocks, plus the code-fence meta the <pre> override needs.
//
// @shikijs/rehype drops the fence meta (```sh title="Terminal"```) unless it gets a
// `parseMetaString` function, and functions can't be passed through Turbopack's
// serialisable loader options. This wrapper adds it here instead. Use it in
// next.config.ts in place of "@shikijs/rehype" (absolute path, same options):
//
//   [join(process.cwd(), "src/components/mdx/rehype-code.mjs"), { themes: {...}, defaultColor: false }]
//
// Result on the <pre>: data-title="Terminal" (if given) and data-lang="sh".
import rehypeShiki from "@shikijs/rehype";

const TITLE = /(?:title|filename)=(?:"([^"]*)"|'([^']*)'|(\S+))/;

/** Pull `title="…"` (or `filename=…`) out of a code-fence meta string. */
function parseMeta(meta, node) {
  const out = {};
  const m = TITLE.exec(meta);
  if (m) out.dataTitle = m[1] ?? m[2] ?? m[3];
  const lang = languageOf(node);
  if (lang) out.dataLang = lang;
  return out;
}

function languageOf(pre) {
  const code = pre.children?.[0];
  const classes = code?.properties?.className;
  const cls = Array.isArray(classes) ? classes.find((c) => typeof c === "string" && c.startsWith("language-")) : undefined;
  return cls ? cls.slice("language-".length) : undefined;
}

export default function rehypeCode(options = {}) {
  // addLanguageClass puts `language-<lang>` on <code>, so blocks without meta still know their language.
  return rehypeShiki.call(this, { ...options, addLanguageClass: true, parseMetaString: parseMeta });
}
