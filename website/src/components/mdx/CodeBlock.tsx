import { Children, isValidElement, type ReactNode } from "react";
import { CopyButton } from "@/components/ui/CopyButton";

// Friendly names for the language label when a fence has no title="…".
const LANG_LABEL: Record<string, string> = {
  sh: "Terminal",
  bash: "Terminal",
  shell: "Terminal",
  zsh: "Terminal",
  console: "Terminal",
  powershell: "PowerShell",
  ps1: "PowerShell",
  py: "Python",
  python: "Python",
  cpp: "C++",
  "c++": "C++",
  c: "C",
  ini: "Config",
  toml: "TOML",
  json: "JSON",
  jsonc: "JSON",
  yaml: "YAML",
  yml: "YAML",
  xml: "XML",
  ts: "TypeScript",
  tsx: "TypeScript",
  js: "JavaScript",
  mdx: "MDX",
  md: "Markdown",
  txt: "Text",
  text: "Text",
};

/** Plain text of a rendered element tree (Shiki's spans), for the copy button. */
function textOf(node: ReactNode): string {
  if (node == null || typeof node === "boolean") return "";
  if (typeof node === "string" || typeof node === "number") return String(node);
  if (Array.isArray(node)) return node.map(textOf).join("");
  if (isValidElement<{ children?: ReactNode }>(node)) return textOf(node.props.children);
  return "";
}

function langOf(children: ReactNode, dataLang?: string): string | undefined {
  if (dataLang) return dataLang;
  const code = Children.toArray(children)[0];
  if (!isValidElement<{ className?: string }>(code)) return undefined;
  return code.props.className?.match(/language-([\w+#-]+)/)?.[1];
}

type PreProps = React.ComponentProps<"pre"> & { "data-title"?: string; "data-lang"?: string };

/**
 * `pre` override for MDX: the design system's .code block (label + copy bar)
 * around Shiki's highlighted <pre>. The label is the fence's title="…", else
 * the language name. Colours come from Shiki's CSS variables (globals.css).
 */
export function CodeBlock({ children, className = "", style, "data-title": title, "data-lang": dataLang, ...rest }: PreProps) {
  const lang = langOf(children, dataLang);
  const label = title ?? (lang ? (LANG_LABEL[lang] ?? lang.toUpperCase()) : "Code");
  // Shiki sets its own background variables; the design system's surface wins.
  const vars = { ...style } as Record<string, string>;
  delete vars["--shiki-light-bg"];
  delete vars["--shiki-dark-bg"];

  return (
    <figure className="code not-prose">
      <div className="code__bar">
        <figcaption className="code__file">{label}</figcaption>
        <CopyButton text={textOf(children).replace(/\n$/, "")} />
      </div>
      <pre className={className} style={vars} {...rest} tabIndex={0}>
        {children}
      </pre>
    </figure>
  );
}
