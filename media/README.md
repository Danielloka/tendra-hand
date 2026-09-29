# media/

All photos, videos, screenshots and diagrams of Tendra Hand live here. It's the single place for
project pictures: the website, the READMEs, the research log and Claude all use these files.

```
media/
  inbox/          drop new, unprocessed files here (not committed to git)
  photos/         photos of the real hardware
  videos/         short clips (MP4), each with a poster frame in photos/
  screenshots/    MuJoCo viewer, Fusion, website, terminal output
  diagrams/       wiring diagrams, drawings, charts (SVG or PNG)
  catalog.json    one entry per file: what it shows, alt text, caption, date, tags
  process_inbox.py  prepares the files in inbox/ and adds them to the catalog
```

## Adding pictures

1. Copy the files into `media/inbox/`. Anything straight from the phone or camera is fine.
2. Run:

   ```sh
   uv run python media/process_inbox.py
   ```

   For each picture, the script:
   - rotates it upright and shrinks it to at most 2000 px on the long side
   - **removes EXIF metadata, including GPS location** (important: the repo is public)
   - saves it as a JPEG (PNG stays PNG, so screenshots stay sharp) with a clean name like
     `2026-09-29-img-4312.jpg` in the right folder
   - adds an entry to `catalog.json` with `"todo"` fields for the description

   Videos (`.mp4`, `.mov`, `.webm`) are only moved and catalogued, not shrunk. Keep clips short
   (5–20 s) and under ~10 MB. HEIC photos (iPhone) need an extra package:
   `uv run --with pillow-heif python media/process_inbox.py`.
3. Fill in the `todo` fields in `catalog.json`, or ask Claude: *"describe the new pictures in
   media/"*. Claude looks at each file, writes the description, alt text and caption, gives it a
   better name and suggests where to use it.

Or skip the script and just tell Claude *"process the media inbox"*.

## catalog.json

Each entry describes one file. It's what lets someone (or Claude) find the right picture without
opening every file.

| Field | Meaning |
|---|---|
| `file` | Path inside `media/`, e.g. `photos/2026-09-29-index-prototype.jpg` |
| `type` | `photo`, `video`, `screenshot` or `diagram` |
| `date` | When it was taken (from the photo, otherwise the day it was added), `YYYY-MM-DD` |
| `hand` | `v0`, `v1` or `null` if it's not about one hand version |
| `description` | What's in the picture, in detail, for people and for Claude |
| `alt` | Short alt text for screen readers (one sentence) |
| `caption` | Caption to show under it on the website |
| `tags` | Topics, e.g. `index`, `thumb`, `tendons`, `electronics`, `sim`, `cad`, `printing` |
| `width`, `height` | Size in pixels (filled in by the script) |
| `poster` | Videos only: the poster frame, e.g. `photos/...-poster.jpg` |
| `used_in` | Where it's used, e.g. `website/content/data/gallery.json` |
| `todo` | What's still missing; remove it when the entry is complete |

## Using a picture

- **Website:** the files are copied to `website/public/media/` before `npm run dev` and
  `npm run build` (`website/scripts/sync-styleguide.mjs`), so `media/photos/a.jpg` is served at
  `/media/photos/a.jpg`. Use that path in `content/data/gallery.json`, `<Figure src=… />` in MDX,
  or a log post's `cover:`. Copy `alt`, `caption`, `width` and `height` from the catalog.
- **Markdown in the repo:** link with a relative path, e.g. from `research/log.md`:
  `![Index prototype](../media/photos/2026-09-29-index-prototype.jpg)`.

Experiment folders in `research/experiments/` can keep their own generated plots. Photos of the
hardware go here.

## Rules

- Never commit a photo that still has GPS data. Run the script, or strip EXIF another way.
- Don't show people's faces or private homes without their permission.
- Names: `YYYY-MM-DD-short-subject.ext`, lowercase, hyphens.
- Licence: like the other docs and research material, the media is CC BY 4.0 (`docs/LICENSE`).
