// Copies the static style guide and the design-system assets into public/
// so they are served at /styleguide/ and /assets/ (same relative paths as in
// the repo). The originals in ../styleguide and ../assets stay the source of truth.
import { cpSync, rmSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
for (const dir of ["styleguide", "assets"]) {
  rmSync(join(root, "public", dir), { recursive: true, force: true });
  cpSync(join(root, dir), join(root, "public", dir), { recursive: true });
}
