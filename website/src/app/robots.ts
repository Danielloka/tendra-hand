import type { MetadataRoute } from "next";
import { siteUrl } from "@/lib/site";

/**
 * Search engines and AI crawlers are welcome: the project is open source and should be easy to find
 * and to cite. They are named explicitly so a crawler that only reads its own group still gets the rules.
 * /dev/ (the hand lab) is blocked; it and /styleguide also send `noindex`. Preview deployments block everything.
 */
const crawlers = [
  "Googlebot",
  "Bingbot",
  "GPTBot",
  "OAI-SearchBot",
  "ChatGPT-User",
  "ClaudeBot",
  "Claude-SearchBot",
  "Claude-User",
  "PerplexityBot",
  "Perplexity-User",
  "Google-Extended",
  "Applebot-Extended",
  "CCBot",
];

export default function robots(): MetadataRoute.Robots {
  if (process.env.VERCEL_ENV === "preview") return { rules: { userAgent: "*", disallow: "/" } };
  return {
    rules: [
      { userAgent: crawlers, allow: "/", disallow: "/dev/" },
      { userAgent: "*", allow: "/", disallow: "/dev/" },
    ],
    sitemap: `${siteUrl}/sitemap.xml`,
  };
}
