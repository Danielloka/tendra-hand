# Website

Source for the Tendra Hand project website, where all open-source files, docs and research are published.

Built with **Next.js 16** (App Router) + TypeScript + Tailwind v4, **React Three Fiber** for the 3D hand, **GSAP ScrollTrigger + Lenis** for the scroll story, and **MDX** for content. Hosting is still to be decided. Architecture, file ownership and the scroll-animation plan: [`PLAN.md`](PLAN.md).

## Run it

```sh
cd website
npm install
npm run dev        # http://localhost:3000  (style guide: /styleguide/, hand lab: /dev/hand)
npm run build      # production build (all pages are static)
npm run lint && npm run typecheck
```

## Edit content (no code needed)

| What | Where |
|---|---|
| Homepage text, story sections, stats, CTA | `content/home.json` |
| Roadmap | `content/roadmap.json` |
| Project / Hardware / Software / Gallery / Build log / Contribute pages | `content/pages/*.mdx` |
| Docs (sidebar order in `_nav.json`) | `content/docs/*.mdx` |
| Build-log posts (newest first by `date`) | `content/log/YYYY-MM-DD-slug.mdx` |
| Parts list, gallery items | `content/data/bom.json`, `content/data/gallery.json` |
| Site name, nav, footer, GitHub URL, domain | `content/site.json` |
| The 3D model | put `public/models/hand.glb`, then set `HAND_MODEL_URL` in `src/components/three/model.ts` (see `public/models/README.md`) |

MDX pages can use `<Callout>`, `<Figure>`, `<Video>`, `<Placeholder>`, `<StatusBadge>`, `<CardGrid>`/`<Card>`, `<Steps>`/`<Step>`, `<BomTable />` and `<Roadmap />` without importing them. Code fences take a title: ```` ```sh title="Terminal" ````. Search `TODO` to find every placeholder.

The design system below is still plain CSS + vanilla JS, and the Next.js app imports it directly.

## Design system

Aesthetic: clean, open and friendly (Apple-like). Light by default with an optional dark mode, lots of white space, one calm blue accent (`#0066CC`) used only for things you can click, Inter for all text, pill buttons and softly rounded cards.

| File | What |
|---|---|
| `assets/css/tokens.css` | All design tokens as CSS variables: colour (light + dark), type scale, spacing, radius, shadows, motion |
| `assets/css/base.css` | Reset, typography, layout, focus styles, scroll-reveal states, reduced motion |
| `assets/css/components.css` | Nav, buttons, cards, spec table, section header, tags, code block, stats, panels |
| `assets/css/tailwind-theme.css` | Tailwind v4 mapping of the tokens (used by the Next.js app) |
| `assets/js/site.js` | Theme toggle, frosted nav, scroll reveals, count-up, copy buttons |
| `styleguide/index.html` | The style guide: every token and component on one page |

**View it:** open `website/styleguide/index.html` in a browser, or serve the folder:

```sh
cd website && python -m http.server 8000   # then open http://localhost:8000/styleguide/
```

Rules of thumb:
- Colours only through tokens. Blue means "clickable"; don't use it for decoration.
- One primary (blue) button per view.
- Plain, friendly words for everyone: explain the tech rather than showing jargon.
- Dark mode is `<html data-theme="dark">`; the toggle remembers the choice in `localStorage`.
- Anything animated must still be readable with `prefers-reduced-motion: reduce` and with JS off.
- Fonts come from Google Fonts for now (Inter, plus JetBrains Mono for code; both OFL). Self-host them for the production site.
