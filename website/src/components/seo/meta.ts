import type { Metadata } from "next";
import { pageMetadata, type PageSlug } from "@/components/pages/content";
import { docsLoc } from "./urls";

/**
 * pageMetadata() for a page that lives in the docs section: the canonical and og:url point at the
 * public docs address (docs.<domain> once it exists), not at the main host's /docs path.
 */
export function docsPageMetadata(slug: PageSlug, path: string): Promise<Metadata> {
  return pageMetadata(slug, { url: docsLoc(path) });
}
