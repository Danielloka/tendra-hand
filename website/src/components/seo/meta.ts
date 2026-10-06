import type { Metadata } from "next";
import { pageMetadata, type PageSlug } from "@/components/pages/content";
import { docsLoc } from "./urls";

/**
 * pageMetadata() for a page that lives in the docs section: the canonical and og:url point at the
 * public docs address (docs.<domain> once it exists), not at the main host's /docs path.
 */
export async function docsPageMetadata(slug: PageSlug, path: string): Promise<Metadata> {
  const meta = await pageMetadata(slug);
  const url = docsLoc(path);
  return { ...meta, alternates: { canonical: url }, openGraph: { ...meta.openGraph, url } };
}
