# Website build plan (v1)

> **Status 2026-09-27:** waves 0–3 done. All 26 routes build statically; tsc/eslint clean; Lighthouse 90+ mobile / 100 desktop on every route; axe clean (light + dark). Ownership rules below applied while the agents ran in parallel; they're kept as the map of who built what.

The lead agent owns this file. Agents read it before starting and report back
instead of editing files they don't own.

## Stack

Next.js 16 (App Router, Turbopack) · TypeScript · Tailwind v4 · React Three Fiber 9 + drei 10 · GSAP 3 + ScrollTrigger · Lenis · MDX via `@next/mdx` (frontmatter, GFM, heading ids, Shiki dual-theme highlighting are already wired in `next.config.ts`).

**Design system = `assets/css/*`** (tokens, base, components). It is imported by `src/app/globals.css`; Tailwind utilities map to the tokens (`bg-bg-alt`, `text-text-muted`, `text-accent-ink`, `rounded-lg`, `shadow-md`, `text-h2`, …). Read `styleguide/index.html` and `README.md` first.

Rules from the design system (non-negotiable):
- Light default, dark opt-in (`<html data-theme="dark">`). Colours **only** through tokens — no hex values in components (exception: three.js materials must read tokens at runtime via `getComputedStyle`).
- Blue (`--accent`) **only for clickable things** (plus the 3D rim light / tendon glow, which is the project's signature). One primary button per view. No orange.
- Clean, open, friendly, Apple-like. Lots of white space. Pill buttons, rounded cards, soft shadows. **Not** a dark "robotic/nerdy" look.
- Plain words for a general audience.
- Motion respects `prefers-reduced-motion`; content is readable without JS.
- Use existing design-system classes (`btn btn--primary`, `card`, `tag`, `section`, `section--alt`, `section-head`, `spec`, `stat`, `panel`, `code`, `kicker`, `lead`, `caption`, `display`, `auto-grid`, `container`) before inventing new styles.
- ⚠️ The design system's grid helper is `.auto-grid` (renamed from `.grid` so it doesn't clash with Tailwind's `grid`). `.container` is pinned in `globals.css`.

## File structure

```
website/
  PLAN.md                    this file
  assets/ styleguide/        design system (source of truth; copied to public/ by scripts/sync-styleguide.mjs)
  content/                   ALL editable text lives here
    site.json                name, url, GitHub, nav, footer            (lead)
    home.json                homepage copy + story sections + stats + CTA
    roadmap.json             roadmap phases (home + project page)
    pages/<page>.mdx         project, hardware, software, gallery, log, contribute (frontmatter + body)
    data/bom.json            bill of materials
    data/gallery.json        gallery items
    log/<yyyy-mm-dd-slug>.mdx build-log posts
    docs/**                  docs pages (+ docs nav file)
  public/models/             hand.glb goes here (README explains the node naming)
  public/images/placeholders/ labelled placeholder SVGs
  src/
    app/                     routes
      layout.tsx globals.css (lead)
      page.tsx               home
      project/ hardware/ software/ gallery/ contribute/ log/ log/[slug]/
      docs/ docs/[...slug]/
      dev/hand/              "hand lab": sliders for every HandState field (noindex, not in sitemap)
      template.tsx           route transition
      sitemap.ts robots.ts opengraph-image.tsx not-found.tsx
    components/
      ui/                    shared primitives (lead): Reveal, CountUp, Tag/StatusBadge, SectionHead, Placeholder, CopyButton
      layout/                Nav, Footer, ThemeToggle, MobileMenu, ScrollProgress, SmoothScroll
      three/                 R3F canvas, placeholder hand, GLB loader, lights, tendons, labels
      home/                  homepage sections + scroll story + electronics diagram + roadmap timeline
      pages/                 page-specific components (BomTable, Gallery grid + lightbox, LogCard, …)
      docs/                  sidebar, search, TOC, prev/next
      mdx/                   components usable in every .mdx file (index.tsx registers them)
    lib/
      handState.ts           ★ scroll ↔ 3D contract (lead)
      mdx.ts                 listMdxSlugs(), loadMdx() (lead)
      site.ts theme.ts       (lead)
      scroll/                GSAP/Lenis setup + homepage timeline
      docs.ts                docs tree, search index
    styles/prose.css         long-form MDX styles
```

## Content formats (Content Agent writes, everyone else reads)

- `content/pages/<page>.mdx` frontmatter: `title`, `kicker`, `lead`, `description` (SEO, ≤160 chars). Body is MDX.
- `content/log/<yyyy-mm-dd-slug>.mdx` frontmatter: `title`, `date` (YYYY-MM-DD), `summary`, `cover` (path, optional), `tags` (string[]). Newest first by `date`.
- `content/home.json`:
  ```jsonc
  {
    "hero": { "kicker": "", "title": "", "lead": "", "primaryCta": { "label": "", "href": "" }, "secondaryCta": { "label": "", "href": "" } },
    "story": [ // exactly these ids, in this order
      { "id": "intro" | "tendons" | "joints" | "exploded" | "electronics", "kicker": "", "title": "", "body": ["paragraph", "..."], "points": ["optional short bullet"] }
    ],
    "diagram": { "nodes": [ { "id": "pc" | "esp32" | "motors", "label": "", "detail": "" } ], "caption": "" },
    "stats": { "kicker": "", "title": "", "items": [ { "value": 8, "unit": "", "label": "" } ] },
    "cta": { "title": "", "lead": "", "label": "", "href": "" }
  }
  ```
- `content/roadmap.json`: `{ "kicker", "title", "lead", "phases": [ { "id", "title", "summary", "status": "done" | "in-progress" | "planned", "items": [string] } ] }`
- `content/data/bom.json`: `{ "updated": "YYYY-MM-DD", "groups": [ { "name", "items": [ { "part", "qty", "spec", "notes", "link"? , "status": "have" | "planned" } ] } ] }`
- `content/data/gallery.json`: `{ "items": [ { "type": "image" | "video", "src", "poster"?, "alt", "caption", "width", "height", "todo" } ] }` — `todo` says what real photo/video should replace the placeholder.
- Real media: files in the repo's `media/` folder (see `media/README.md`, described in `media/catalog.json`), copied to `public/media/` by `scripts/sync-styleguide.mjs` and referenced as `/media/photos/…`. Replace a placeholder by pointing `src` there and removing its `todo`.
- Placeholders: SVGs in `public/images/placeholders/` with the subject written on them. Every placeholder in MDX/JSON/components has a `TODO:` marker (JSX comment, MDX comment `{/* TODO: … */}`, or a `todo` field).

### MDX components (available in every .mdx without import)

Built by the Docs Agent (wave 1) unless noted; registered in `src/components/mdx/index.tsx`:

| Component | Props |
|---|---|
| `<Callout type="note" \| "tip" \| "warning" title?>` | children |
| `<Figure src alt caption? width? height?>` | image with caption (next/image) |
| `<Video src poster? caption?>` | muted, playsinline, controls; lazy |
| `<Placeholder label ratio?>` | wraps `ui/Placeholder` |
| `<StatusBadge status="done" \| "in-progress" \| "planned">` | wraps `ui/StatusBadge` |
| `<CardGrid>` + `<Card title href? kicker?>` | children = body text |
| `<Steps>` + `<Step title>` | numbered assembly steps |
| `pre`/`code` override | Shiki block with filename bar + copy button (`title` from the code fence meta if possible) |
| `<BomTable />` | (Pages Agent, wave 2) renders `data/bom.json` |
| `<Roadmap />` | (Pages Agent, wave 2) renders `roadmap.json` via the home RoadmapTimeline |

## Scroll-animation plan (homepage)

The homepage is one long scroll story. A **sticky canvas** holds the hand; the
GSAP timeline, scrubbed by ScrollTrigger (with Lenis smoothing), tweens the
plain numbers in `src/lib/handState.ts`. The R3F scene reads them every frame.

| # | Section | Layout (desktop ≥ 1024px) | `handState` targets |
|---|---|---|---|
| 0 | **Hero** | Canvas full-width, centred behind/under the headline | `idle 1, rim 1, rotation 0, zoom 1` — slow idle spin + breathing, accent rim light |
| 1 | **Intro** ("every joint, its own motor") | Canvas slides to the right half; text column scrolls on the left | `idle → 0.2, rim → 0.6, rotation.y → -0.5` (three-quarter palm view) |
| 2 | **Tendons** | right | `rotation.y → π` (back of the hand), `tendons 0 → 1` (glowing lines along the back of each finger), `rim → 0.3` |
| 3 | **Joints / DOF** | right | `rotation.y → -0.35, rotation.x → 0.25`, `tendons → 0`; joints bend **one by one** (index MCP → PIP → DIP, then thumb CMC → MCP → IP), `labels → 1` (MCP/PIP/DIP pills pop at each joint as it bends), then all return to 0 |
| 4 | **Exploded view** | right (may switch to left once if it reads better) | `labels → 0`; `explode 0 → 1` over the first half, hold, `1 → 0` over the second half; slow `rotation.y` drift |
| 5 | **Electronics** | right | `wireframe 0 → 1`, `zoom → 0.85`; HTML/SVG diagram **PC → ESP32-S3 → motors** fades in over/next to the canvas (real text, not in WebGL) |
| 6 | Stats → Roadmap → CTA | canvas un-pins and scrolls away; rendering pauses off-screen | — |

Text sections: each story section is ≥ 100vh tall on desktop so there's time for the animation; headings/paragraphs reveal as they enter.

**Mobile (< 768px):** the canvas is sticky at the top (~42svh) and text scrolls underneath (never side by side). Simpler animation: no one-by-one joint sequence (all bend together), no rotation drift, labels only for the index finger.
**Tablet (768–1023px):** same as mobile but a larger canvas.
**`prefers-reduced-motion`:** no Lenis, no scrubbing. A single static render (the three-quarter pose) at the top of the story, then the sections as normal content, the diagram shown statically.
**No JS:** all text is server-rendered; the canvas area shows a placeholder.

### 3D component API (3D Agent delivers, Scroll Agent consumes)

- `src/components/three/HandCanvas.tsx` — `export default function HandCanvas(props: { state?: HandState; mode?: "animated" | "static"; className?: string })`. Client-only; the homepage loads it with `next/dynamic(..., { ssr: false })`. It fills its parent box. Internally: `dpr={[1, 1.75]}`, pauses rendering when off-screen (IntersectionObserver → `frameloop="never"`) and when the tab is hidden, `mode="static"` renders one frame (`frameloop="demand"`). Reads `--accent`, `--text`, `--bg` from CSS at runtime and re-reads on theme change (`useTheme()` from `src/lib/theme.ts`).
- `src/components/three/model.ts` — **the one line to swap in the real model**: `export const HAND_MODEL_URL: string | null = null; // TODO: set to "/models/hand.glb"`. `null` = procedural placeholder hand.
- GLB convention (documented in `public/models/README.md`): a node named after each `JointId` (e.g. `index_pip`) whose origin is the joint pivot and whose local X axis is the flexion axis. If those nodes exist they're animated; otherwise the whole model only rotates (fallback). Optional `tendon_*` curve nodes; otherwise tendons are generated from joint positions.
- Placeholder hand: low-poly five-finger hand from primitives (rounded boxes/capsules), PLA-white material, correct joint hierarchy for all 17 `JointId`s, dorsal tendon paths, joint label anchors.

## Agents, waves and file ownership

An agent edits **only** files it owns. Shared files are owned by the lead; ask the lead (report it) instead of editing. No `git commit`, no `git push`.

| Wave | Agent | Owns | Done when |
|---|---|---|---|
| 0 | **Lead** | `package.json`, configs, `src/app/layout.tsx`, `src/app/globals.css`, `src/lib/{handState,mdx,site,theme}.ts`, `src/components/ui/**`, `content/site.json`, `PLAN.md` | scaffold builds ✅ |
| 1 | **Content** | `content/home.json`, `content/roadmap.json`, `content/pages/**`, `content/data/**`, `content/log/**`, `public/images/placeholders/**` | all files match the formats above; facts match the repo; every placeholder has a TODO |
| 1 | **3D** | `src/components/three/**`, `src/app/dev/hand/**`, `public/models/**` | `/dev/hand` shows the placeholder hand; every HandState field visibly works via sliders; tsc + eslint clean |
| 1 | **Docs** | `src/app/docs/**`, `src/components/docs/**`, `src/components/mdx/**`, `src/lib/docs.ts`, `src/styles/prose.css`, `content/docs/**` | `/docs` with sidebar, search, TOC, prev/next and starter pages; all MDX components in the table above (except wave-2 ones) work |
| 2 | **Scroll** | `src/app/page.tsx`, `src/components/home/**`, `src/lib/scroll/**`, `src/components/layout/SmoothScroll.tsx` | homepage story works on desktop, mobile and reduced motion; stats, roadmap, CTA |
| 2 | **Pages** | `src/app/{project,hardware,software,gallery,contribute,log}/**`, `src/app/{template,not-found,sitemap,robots,opengraph-image}.*`, `src/components/pages/**`, `src/components/layout/**` (not SmoothScroll), and takes over `src/components/mdx/index.tsx` | all pages render from `content/`; mobile menu; scroll progress bar; route transitions; OG images; sitemap |
| 3 | Design review → Accessibility → Performance → Code review | (sequential, so fixes never collide) | see `.claude/agents/*.md` |

### Working rules for build agents

- **Parallel builds:** never use the default `.next` folder. Use your own dist dir and port:
  `NEXT_DIST_DIR=.next-<agent> npx next dev -p <port>` — ports: 3D 3101, Docs 3102, Scroll 3103, Pages 3104, reviewers 3105. Always stop your server when done (`netstat -ano | grep :<port>` → `taskkill //PID <pid> //F //T`).
- **See your work:** `npx -y playwright@1 screenshot --channel chrome --viewport-size=1440,900 --wait-for-timeout=4000 http://localhost:<port>/<path> <scratchpad>/<name>.png`, then open the PNG with the Read tool. Also try `--viewport-size=390,844` for mobile. (`--full-page` for long pages.)
- **Checks before reporting done:** `npx tsc --noEmit` and `npx eslint <your files>` must be clean for your files.
- No new npm dependencies without asking the lead, except the Docs Agent may add **one** small search library if really needed.
- Code style: TypeScript strict, function components, server components by default (`"use client"` only where needed), comment density like the existing files, no dead code. Mark every placeholder with `TODO:`.
- Report back: what you built (files), how you verified it (screenshots taken, checks run), open problems, anything you needed from another agent.
