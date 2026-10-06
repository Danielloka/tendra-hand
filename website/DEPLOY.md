# Deploying the website on Vercel

The site is a static Next.js app in `website/`. It also copies `../media` (the project photos) into
`public/media/` during the build, so Vercel must be allowed to read files outside `website/`.

## First deploy

1. Push the repo to GitHub (`Danielloka/tendra-hand`).
2. On [vercel.com](https://vercel.com): **Add New → Project → Import** `Danielloka/tendra-hand`.
3. Settings on the import screen:
   - **Root Directory:** `website`
   - **Include files outside the root directory in the Build Step:** on (the default; needed for `../media`)
   - **Framework Preset:** Next.js (also set in `vercel.json`)
   - Build, install and output commands: leave as they are (`vercel.json` sets `npm ci` and `npm run build`)
   - **Node.js version** (Project Settings → Build and Deployment): 22.x or newer (`package.json` needs ≥ 20.9)
4. Environment variables: none are needed for the first deploy. The site uses the Vercel production
   address (`<project>.vercel.app`) for links, the sitemap and social previews until you set a domain.
5. Click **Deploy**.

## After the first deploy, check

- The build log shows `sync-styleguide: copied N media files`. A `WARNING sync-styleguide` line means
  the photos are missing: turn on "Include files outside the root directory" and redeploy.
- Home, Project, Journey, Gallery (photos load), Docs, a build-log post, and a page that doesn't exist (404).
- `/robots.txt` and `/sitemap.xml` show your real address, not `localhost`.
- `/models/README.md` returns 404 (it is left out by `.vercelignore`).
- Security headers: paste the address into [securityheaders.com](https://securityheaders.com).
- Open the browser console on the homepage and scroll the story: there should be no
  "Content-Security-Policy" errors.
- Preview deployments (from branches and pull requests) are kept out of search engines automatically.

Pushes that change nothing in `website/` or `media/` skip the build (`ignoreCommand` in `vercel.json`).

## Adding a domain later

1. Project → **Settings → Domains → Add**, e.g. `tendrahand.com`. Follow the DNS steps Vercel shows.
2. Add the docs subdomain the same way: `docs.tendrahand.com` (same project; the app serves the docs
   there by itself).
3. Project → **Settings → Environment Variables** (Production):
   - `NEXT_PUBLIC_SITE_URL` = `https://tendrahand.com`
   - `NEXT_PUBLIC_DOCS_URL` = `https://docs.tendrahand.com`
4. **Redeploy** (these values are baked in at build time).
5. Check that `/docs` on the main domain redirects to the docs subdomain, and that the docs' "Main site"
   link goes back.

## Notes

- Security headers and the Content-Security-Policy live in `next.config.ts`. Everything the site loads
  comes from its own domain; only `/styleguide` also loads Google Fonts. If you add something from
  another site (analytics, a video embed, a Draco-compressed 3D model whose decoder comes from
  gstatic.com), add its address to the policy there.
- Only pictures listed in `media/catalog.json` are published, and never one marked `"public": false`.
- `/dev/hand` and `/styleguide` are online but marked `noindex`, and are not in the sitemap.
