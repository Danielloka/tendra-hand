import type { Metadata } from "next";
import { siteConfig } from "@/lib/site";

/**
 * The site-wide share image (app/opengraph-image.tsx). A page that sets its own `openGraph` or
 * `twitter` replaces the root one, file image included, so every page adds it back.
 */
export const OG_IMAGE = { url: "/opengraph-image", width: 1200, height: 630, alt: "Tendra Hand, an open-source, tendon-driven robotic hand" };

/** The home page's one wording for <title>, og:title and twitter:title. */
export const HOME_TITLE = `${siteConfig.name}: Open-Source Tendon-Driven Robotic Hand`;
export const HOME_DESCRIPTION =
  "An open-source, 3D-printed robotic hand with the joints of a human hand: tendon-driven, simulated, trained with AI. The hand first, then a home robot.";

type Image = { url: string; width?: number; height?: number; alt?: string };

type Options = {
  /** Page title without the brand; the template adds " · Tendra Hand". */
  title: string;
  description?: string;
  /** Canonical and og:url (a path, or an absolute address for the docs host). */
  url: string;
  /** `title` is complete as written (home page): no brand is added. */
  absoluteTitle?: boolean;
  image?: Image;
  /** Extra Open Graph fields: article dates, tags. */
  article?: { publishedTime?: string; modifiedTime?: string; tags?: string[] };
};

/**
 * Title, description, canonical, Open Graph and Twitter card for one page, all from the same
 * three values, so the tags can never disagree. Social titles carry the brand like the <title>.
 */
export function socialMeta({ title, description, url, absoluteTitle, image = OG_IMAGE, article }: Options): Metadata {
  const full = absoluteTitle ? title : `${title} · ${siteConfig.name}`;
  return {
    title: absoluteTitle ? { absolute: title } : title,
    description,
    alternates: { canonical: url },
    openGraph: {
      ...(article ? { type: "article" as const, ...article } : { type: "website" as const }),
      siteName: siteConfig.name,
      locale: "en_US",
      title: full,
      description,
      url,
      images: [image],
    },
    twitter: { card: "summary_large_image", title: full, description, images: [image.url] },
  };
}
