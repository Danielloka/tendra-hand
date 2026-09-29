// Copies the static style guide and the design-system assets into public/
// so they are served at /styleguide/ and /assets/ (same relative paths as in
// the repo). The originals in ../styleguide and ../assets stay the source of truth.
// Also copies the project media (../../media, without the inbox and tooling)
// to public/media/, so media/photos/a.jpg is served at /media/photos/a.jpg.
import { cpSync, existsSync, rmSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
for (const dir of ["styleguide", "assets"]) {
  rmSync(join(root, "public", dir), { recursive: true, force: true });
  cpSync(join(root, dir), join(root, "public", dir), { recursive: true });
}

const media = join(root, "..", "media");
rmSync(join(root, "public", "media"), { recursive: true, force: true });
for (const dir of ["photos", "videos", "screenshots", "diagrams"]) {
  const src = join(media, dir);
  if (existsSync(src)) cpSync(src, join(root, "public", "media", dir), { recursive: true });
}
