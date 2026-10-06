# SEO and AI discoverability

How the Tendra Hand site is set up to be found by search engines and cited by AI assistants (ChatGPT, Claude, Perplexity, Gemini), and what you still need to do off-site.

## What is built in

- **Metadata on every page:** unique title and description, canonical URL, Open Graph and Twitter cards. Docs pages use the docs host (`docs.<domain>`) as canonical once `NEXT_PUBLIC_DOCS_URL` is set.
- **Structured data (JSON-LD)** in `src/components/seo/`:
  - every page: Organization, WebSite and SoftwareSourceCode (links the GitHub repo and the three licenses);
  - docs and build-log posts: TechArticle + BreadcrumbList; the FAQ also gets FAQPage (built from `content/docs/faq.mdx`, so every `## Question` becomes a Q&A);
  - `/gallery`: ImageGallery; `/docs` and `/docs/log`: CollectionPage; `/journey`, `/contribute`: WebPage.
- **`/llms.txt`** (llmstxt.org): a short Markdown map with key facts and a link plus one-line description for every page, doc and log post. **`/llms-full.txt`**: all docs, reference pages, the roadmap and log posts as one Markdown file. Both are generated at build time from `content/`. The "Key facts" list is in `src/components/seo/llms.ts` (`KEY_FACTS`): update it when the hardware or status changes.
- **robots.txt allows AI crawlers**, `sitemap.xml` lists every public page.
- All text on the home and project pages is server-rendered HTML, so crawlers that do not run JavaScript still read it.

## Do these after the domain exists

1. Set `NEXT_PUBLIC_SITE_URL` (and `NEXT_PUBLIC_DOCS_URL` for the docs host) in Vercel, then redeploy. Until then canonical URLs and the sitemap point at the Vercel address.
2. **Google Search Console:** add the domain, verify with the DNS record, submit `https://<domain>/sitemap.xml`. Request indexing for the home page.
3. **Bing Webmaster Tools:** add the site (you can import it from Search Console), submit the sitemap. Bing feeds ChatGPT search and Copilot, so this matters for AI answers.
4. **GitHub repo:** fill in the description (one sentence, like the site's), the website link, and topics such as `robotic-hand`, `tendon-driven`, `open-source-hardware`, `robotics`, `mujoco`, `3d-printing`, `esp32`, `dexterous-manipulation`. Put a short README intro with the same facts as `llms.txt`.
5. **Share it where makers and AI crawlers look:** a Hackaday.io project page, r/robotics and r/3Dprinting, Hacker News ("Show HN: Tendra Hand, an open-source tendon-driven robotic hand"), and a write-up on Instructables or Printables. Each is a backlink and a place AI tools learn from.
6. **Later:** a Wikidata entry once there are independent sources (articles, videos), and a YouTube video of the hand moving with the site link in the description.

## Keep it consistent

- Always write the name as **Tendra Hand** (two words, capital T and H), the same in the repo, the site, social posts and videos. Assistants cite what they can match across sources.
- Say the same facts everywhere: open-source, tendon-driven, 3D-printed, same joints as a human hand, V0 = thumb + index (8 joints), V1 = five fingers (20 joints, 16 servos), licenses Apache-2.0 / CERN-OHL-S-2.0 / CC BY 4.0.
- **Name clash:** an unrelated open-source project "Tendra H1" (github.com/aymankhayat/tendra-h1-robotic-hand) uses a similar name. `llms.txt` says they are different. Revisit the name before registering a company or trademark (see `CLAUDE.md`).
- Date your pages: `updated:` in doc frontmatter becomes `dateModified` in the schema.

## Check it

- Rich results: https://search.google.com/test/rich-results (paste a live URL).
- Schema validity: https://validator.schema.org.
- Ask ChatGPT, Claude and Perplexity "What is Tendra Hand?" a few weeks after launch and note what they get wrong; fix it in `llms.txt` and the home page text.
