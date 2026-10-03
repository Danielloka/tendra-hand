import { join } from "node:path";
import createMDX from "@next/mdx";
import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  pageExtensions: ["ts", "tsx", "md", "mdx"],
  reactStrictMode: true,
  // Lets several agents/devs build in parallel without sharing a cache folder.
  distDir: process.env.NEXT_DIST_DIR || ".next",
  images: { formats: ["image/avif", "image/webp"] },
  // three.js ships ES modules only; transpile keeps older browsers happy.
  transpilePackages: ["three"],

  // docs.<domain> is the same app: its URLs have no /docs prefix, so rewrite them to /docs/*.
  // Requests for the docs host match `has: host`; localhost testing works with docs.localhost:3000.
  async rewrites() {
    const onDocsHost = [{ type: "host" as const, value: "docs\\..+" }];
    return {
      beforeFiles: [
        { source: "/", has: onDocsHost, destination: "/docs" },
        // everything except /docs itself and the app's own assets
        { source: "/:path((?!docs/|docs$|_next/|media/|models/|styleguide|icon|opengraph-image|robots\\.txt|sitemap\\.xml).+)", has: onDocsHost, destination: "/docs/:path" },
      ],
      afterFiles: [],
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
