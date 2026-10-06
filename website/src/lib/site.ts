import site from "@content/site.json";

export const siteConfig = site;
export const githubUrl = site.github;

/**
 * The docs live at /docs in this app. Once the docs site has its own domain (docs.<domain>),
 * set NEXT_PUBLIC_DOCS_URL (e.g. https://docs.example.com) and NEXT_PUBLIC_SITE_URL (the main
 * site). Until then both stay empty and everything works on one host.
 */
const trim = (u: string | undefined) => (u ?? "").replace(/\/$/, "");
export const docsUrl = trim(process.env.NEXT_PUBLIC_DOCS_URL);
export const mainUrl = trim(process.env.NEXT_PUBLIC_SITE_URL);

/**
 * Canonical address of the main site, for metadataBase, robots.txt, the sitemap and OG images.
 * NEXT_PUBLIC_SITE_URL wins; on Vercel it falls back to the production domain, then the deployment URL.
 * Server-only: the VERCEL_* variables are not in the browser bundle.
 */
export const siteUrl =
  mainUrl ||
  (process.env.VERCEL_PROJECT_PRODUCTION_URL && `https://${process.env.VERCEL_PROJECT_PRODUCTION_URL}`) ||
  (process.env.VERCEL_URL && `https://${process.env.VERCEL_URL}`) ||
  "http://localhost:3000";

/** Public address of a docs path ("/docs/hardware"): on the docs host the /docs prefix is dropped. */
export const docsAbsolute = (path: string) => (docsUrl ? `${docsUrl}${path.replace(/^\/docs(?=\/|$)/, "")}` : path);

export type NavItem = { label: string; href: string };

/** Main-site nav: the "Docs" link points at the docs host once there is one. */
export const mainNav: NavItem[] = site.nav.map((i) => (i.href === "/docs" ? { ...i, href: docsAbsolute("/docs") || "/" } : i));

/** Docs nav, with a way back to the main site. */
export const docsNav: NavItem[] = [...site.docsNav, { label: "Main site", href: mainUrl || "/" }];
