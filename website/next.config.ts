import { join } from "node:path";
import createMDX from "@next/mdx";
import type { NextConfig } from "next";

const isDev = process.env.NODE_ENV === "development";
// Vercel preview deployments inject the Vercel toolbar (comments, feedback) from vercel.live.
const isPreview = process.env.VERCEL_ENV === "preview";

/**
 * Content-Security-Policy without nonces, so every page stays statically rendered.
 * Everything the site loads is same-origin: next/font self-hosts Inter and JetBrains Mono,
 * the GLB models have no Draco/meshopt compression (no decoder fetched from a CDN),
 * and media is served from /media and /_next/image.
 * - script-src 'unsafe-inline': Next's inline bootstrap/RSC payload scripts and the theme boot script.
 *   'unsafe-eval' only in dev (React uses eval for debug stacks).
 * - script-src 'wasm-unsafe-eval': drei's useGLTF sets up three's meshopt decoder, a WebAssembly
 *   module bundled inline (no CDN). Without it the 3D hand never loads. It allows WebAssembly only, not eval().
 * - style-src 'unsafe-inline': React style props, Shiki token colours, GSAP inline transforms.
 * - img-src data: blob:: inline SVG data URIs in CSS; three.js GLTFLoader turns embedded textures into blob: URLs.
 */
function csp(extra: { style?: string[]; font?: string[] } = {}) {
  const vercel = isPreview ? ["https://vercel.live"] : [];
  const directives: Record<string, string[]> = {
    "default-src": ["'self'"],
    "script-src": ["'self'", "'unsafe-inline'", "'wasm-unsafe-eval'", ...(isDev ? ["'unsafe-eval'"] : []), ...vercel],
    "style-src": ["'self'", "'unsafe-inline'", ...(extra.style ?? []), ...vercel],
    "img-src": ["'self'", "data:", "blob:", ...(isPreview ? ["https://vercel.live", "https://vercel.com"] : [])],
    "font-src": ["'self'", "data:", ...(extra.font ?? []), ...(isPreview ? ["https://vercel.live", "https://assets.vercel.com"] : [])],
    "media-src": ["'self'", "blob:"],
    "connect-src": ["'self'", ...(isDev ? ["ws:", "wss:"] : []), ...(isPreview ? ["https://vercel.live", "wss://ws-us3.pusher.com"] : [])],
    "worker-src": ["'self'", "blob:"],
    "frame-src": isPreview ? ["https://vercel.live"] : ["'none'"],
    "object-src": ["'none'"],
    "base-uri": ["'self'"],
    "form-action": ["'self'"],
    "frame-ancestors": ["'none'"],
  };
  return Object.entries(directives)
    .map(([k, v]) => `${k} ${v.join(" ")}`)
    .join("; ");
}

const securityHeaders = [
  { key: "Strict-Transport-Security", value: "max-age=63072000; includeSubDomains" },
  { key: "X-Content-Type-Options", value: "nosniff" },
  { key: "Referrer-Policy", value: "strict-origin-when-cross-origin" },
  {
    key: "Permissions-Policy",
    value:
      "camera=(), microphone=(), geolocation=(), payment=(), usb=(), serial=(), hid=(), bluetooth=(), midi=(), " +
      "accelerometer=(), gyroscope=(), magnetometer=(), display-capture=(), browsing-topics=(), fullscreen=(self)",
  },
  { key: "X-Frame-Options", value: "DENY" },
  { key: "Cross-Origin-Opener-Policy", value: "same-origin" },
  { key: "Content-Security-Policy", value: csp() },
];

const noindex = [{ key: "X-Robots-Tag", value: "noindex, nofollow" }];
// Public files don't have content-hashed names, so cache for a day and revalidate in the background.
const assetCache = [{ key: "Cache-Control", value: "public, max-age=86400, stale-while-revalidate=604800" }];

const nextConfig: NextConfig = {
  pageExtensions: ["ts", "tsx", "md", "mdx"],
  reactStrictMode: true,
  poweredByHeader: false,
  productionBrowserSourceMaps: false,
  // Lets several agents/devs build in parallel without sharing a cache folder.
  distDir: process.env.NEXT_DIST_DIR || ".next",
  images: { formats: ["image/avif", "image/webp"] },
  // three.js ships ES modules only; transpile keeps older browsers happy.
  transpilePackages: ["three"],

  // docs.<domain> is the same app: its URLs have no /docs prefix, so rewrite them to /docs/*.
  // Requests for the docs host match `has: host`; localhost testing works with docs.localhost:3000.
  // When two entries set the same key on a path, the later one wins.
  async headers() {
    return [
      { source: "/:path*", headers: securityHeaders },
      // The static style guide (/styleguide and below) loads Inter from Google Fonts.
      {
        source: "/styleguide/:path*",
        headers: [
          { key: "Content-Security-Policy", value: csp({ style: ["https://fonts.googleapis.com"], font: ["https://fonts.gstatic.com"] }) },
          ...noindex,
        ],
      },
      // Internal pages: reachable, never indexed.
      { source: "/dev/:path*", headers: noindex },
      { source: "/docs/search-index.json", headers: noindex },
      { source: "/models/:path*", headers: assetCache },
      { source: "/media/:path*", headers: assetCache },
    ];
  },

  async rewrites() {
    const onDocsHost = [{ type: "host" as const, value: "docs\\..+" }];
    return {
      beforeFiles: [
        { source: "/", has: onDocsHost, destination: "/docs" },
        // everything except /docs itself and the app's own assets
        { source: "/:path((?!docs/|docs$|_next/|media/|models/|styleguide|icon|opengraph-image|robots\\.txt|sitemap\\.xml|llms\\.txt|llms-full\\.txt).+)", has: onDocsHost, destination: "/docs/:path" },
      ],
      // public/ doesn't serve index.html for a folder; the style guide's relative ../assets links work from /styleguide.
      afterFiles: [{ source: "/styleguide", destination: "/styleguide/index.html" }],
      fallback: [],
    };
  },

  async redirects() {
    const onDocsHost = [{ type: "host" as const, value: "docs\\..+" }];
    const docsUrl = (process.env.NEXT_PUBLIC_DOCS_URL ?? "").replace(/\/$/, "");
    return [
      // docs host: /docs/x -> /x (the address stays short; the rewrite above serves it)
      { source: "/docs/:path*", has: onDocsHost, destination: "/:path*", permanent: true },
      // the pages that moved into the docs
      { source: "/hardware", missing: onDocsHost, destination: `${docsUrl}${docsUrl ? "" : "/docs"}/hardware`, permanent: true },
      { source: "/software", missing: onDocsHost, destination: `${docsUrl}${docsUrl ? "" : "/docs"}/software`, permanent: true },
      { source: "/log", missing: onDocsHost, destination: `${docsUrl}${docsUrl ? "" : "/docs"}/log`, permanent: true },
      { source: "/log/:slug", missing: onDocsHost, destination: `${docsUrl}${docsUrl ? "" : "/docs"}/log/:slug`, permanent: true },
      // main host: /docs/x -> docs.<domain>/x once the docs have their own domain
      ...(docsUrl ? [{ source: "/docs/:path*", missing: onDocsHost, destination: `${docsUrl}/:path*`, permanent: true }] : []),
    ];
  },
};

// Plugins are given by name (strings) so they also work with Turbopack.
const withMDX = createMDX({
  extension: /\.(md|mdx)$/,
  options: {
    remarkPlugins: ["remark-gfm", "remark-frontmatter", ["remark-mdx-frontmatter", { name: "frontmatter" }]],
    rehypePlugins: [
      "rehype-slug",
      // Shiki + code-fence titles (```sh title="Terminal"```). Absolute path: @next/mdx
      // resolves plugin paths relative to each .mdx file.
      [join(process.cwd(), "src/components/mdx/rehype-code.mjs"), { themes: { light: "github-light-high-contrast", dark: "github-dark-high-contrast" }, defaultColor: false }],
    ],
  },
});

export default withMDX(nextConfig);
