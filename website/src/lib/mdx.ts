import { readdirSync } from "node:fs";
import { join } from "node:path";
import type { ComponentType } from "react";

/**
 * Shared MDX loading helpers (server only).
 *
 * Every .mdx file under content/ may start with YAML frontmatter; it is exposed
 * as the `frontmatter` export (remark-mdx-frontmatter), and the file's default
 * export is the rendered React component.
 */
export type MdxModule<F> = { default: ComponentType; frontmatter: F };

const CONTENT_DIR = join(process.cwd(), "content");

/** Slugs (file names without .mdx) of every .mdx file in content/<dir>. */
export function listMdxSlugs(dir: string): string[] {
  return readdirSync(join(CONTENT_DIR, dir))
    .filter((f) => f.endsWith(".mdx") && !f.startsWith("_"))
    .map((f) => f.replace(/\.mdx$/, ""));
}

/**
 * Import content/<path>.mdx. The import path must keep a static prefix
 * ("@content/") so the bundler can include every file in that folder.
 */
export async function loadMdx<F>(path: string): Promise<MdxModule<F>> {
  return (await import(`@content/${path}.mdx`)) as MdxModule<F>;
}
