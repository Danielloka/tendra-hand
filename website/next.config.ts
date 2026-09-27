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
