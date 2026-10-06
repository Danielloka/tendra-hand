// Copies the static style guide and the design-system assets into public/
// so they are served at /styleguide/ and /assets/ (same relative paths as in
// the repo). The originals in ../styleguide and ../assets stay the source of truth.
// Also copies the project media (../../media, catalogued public files only)
// to public/media/, so media/photos/a.jpg is served at /media/photos/a.jpg.
import { copyFileSync, cpSync, existsSync, mkdirSync, readdirSync, readFileSync, rmSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
for (const dir of ["styleguide", "assets"]) {
  rmSync(join(root, "public", dir), { recursive: true, force: true });
  cpSync(join(root, dir), join(root, "public", dir), { recursive: true });
}

// Only files listed in media/catalog.json are published, and never an entry with "public": false
// (e.g. pictures with people who haven't agreed to be online). Unlisted files are skipped with a warning.
const media = join(root, "..", "media");
rmSync(join(root, "public", "media"), { recursive: true, force: true });
const catalogPath = join(media, "catalog.json");
const items = existsSync(catalogPath) ? JSON.parse(readFileSync(catalogPath, "utf8")).items ?? [] : [];
const allowed = new Set();
const hidden = new Set();
for (const item of items) {
  const files = [item.file, item.poster].filter(Boolean);
  for (const f of files) (item.public === false ? hidden : allowed).add(f);
}
for (const f of hidden) allowed.delete(f);

/** Paths relative to media/ (forward slashes) of every file under dir. */
const walk = (dir, rel = "") =>
  readdirSync(join(dir, rel), { withFileTypes: true }).flatMap((e) =>
    e.isDirectory() ? walk(dir, `${rel}${e.name}/`) : [`${rel}${e.name}`],
  );

let copied = 0;
for (const dir of ["photos", "videos", "screenshots", "diagrams"]) {
  if (!existsSync(join(media, dir))) continue;
  for (const file of walk(media, `${dir}/`)) {
    if (hidden.has(file)) continue;
    if (!allowed.has(file)) {
      console.warn(`sync-styleguide: skipped media/${file} (not in media/catalog.json)`);
      continue;
    }
    const dest = join(root, "public", "media", file);
    mkdirSync(dirname(dest), { recursive: true });
    copyFileSync(join(media, file), dest);
    copied++;
  }
}
if (hidden.size) console.log(`sync-styleguide: kept ${hidden.size} non-public media file(s) out of public/media/`);
// ../media sits outside website/. On Vercel it is only there when "Include files outside the
// Root Directory" is on (the default). Don't fail the build, but say loudly that images will be missing.
if (copied === 0) {
  console.warn(
    `\nWARNING sync-styleguide: no project media found in ${media}.\n` +
      "  Gallery and log images under /media/ will be broken in this build.\n" +
      "  On Vercel: Project Settings → Build and Deployment → Root Directory → enable\n" +
      '  "Include files outside the root directory in the Build Step".\n',
  );
} else {
  console.log(`sync-styleguide: copied ${copied} media files to public/media/`);
}
