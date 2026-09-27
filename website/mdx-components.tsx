import type { MDXComponents } from "mdx/types";

// Global MDX component map. Page/Docs agents register shared components here
// (see src/components/mdx/index.tsx).
import { mdxComponents } from "@/components/mdx";

export function useMDXComponents(components: MDXComponents): MDXComponents {
  return { ...mdxComponents, ...components };
}
