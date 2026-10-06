# Fill in later

Things removed or left out of the public site so it reads as finished. Add them back when you have the material. Paths are relative to `website/`.

## Gallery (`content/data/gallery.json`, intro in `content/pages/gallery.mdx`)
Six items were dropped because they only had a placeholder picture. Add each back with a real file in `media/` (see `media/README.md`), then an entry with `src`, `alt`, `caption`, `width`, `height` (videos also `poster`):
- Close-up of the back of the index finger showing the tendon loops (landscape 16:10).
- Video, 5-10 s, 16:9: the index finger closing and opening smoothly (plus a poster frame). Needs the first power-on.
- Screenshot of `uv run python sim/view.py` with the hand in a pinch pose and the Control sliders visible (landscape 16:10).
- Video, 10-15 s, portrait 4:5: laptop running `sim/twin.py --port auto` next to the real hand, joints following the sliders (plus poster).
- Photo of the real hand holding a pinch pose, ideally holding a small object (portrait 4:5).
- Screenshot of the Fusion 360 thumb and index design on a clean background, maybe a section view through a joint (landscape 16:10).

## Build log (`content/log/*.mdx`)
- Covers: the firmware post uses the electronics photo; the MuJoCo post and the Python post use two teleop screenshots as stand-ins. Replace the `cover:` line with a better picture when you have one (a screenshot of the model in the MuJoCo viewer; `sim/twin.py --fake` showing the viewer and terminal).
- Removed figures: MuJoCo viewer screenshot in `2026-09-26-first-mujoco-model.mdx` (after "the tests fail if the committed model is out of date") and the digital twin screenshot in `2026-09-27-python-library-digital-twin.mdx` (after the `--fake` paragraph).

## Hardware page (`content/pages/hardware.mdx`)
- Links to each STL/3MF and to the .f3d/STEP files, once committed (removed hidden TODO).
- Print settings: the "Tested value" column was removed. Add it back with the settings actually used, per part (hidden TODO removed).
- Wiring diagram: the placeholder diagram was replaced by one sentence. Add a real diagram of the ESP32-S3, 8 ULN2003 boards and the 5 V supply.
- The "What is in the repository today" callout should change once print-ready files are in `hardware/print/`.

## Bill of materials (`content/data/bom.json`)
The TODO cells were replaced with rough, honest wording. Replace with measured values:
- PLA/PETG filament: grams per hand, brand and colour (now "a few hundred grams").
- Tendon line: length per joint (now "a few metres per hand"; 0.4 mm fishing line).
- TPU filament: amount per pad and shore hardness (now "a few grams", "flexible filament").
- Servo power supply: now "5 to 6 V, at least 15 A" from the V1 design; confirm the part you buy (e.g. Mean Well LRS-100-5).
- Prices: no prices anywhere yet.

## Docs
- `content/docs/assembly.mdx`: five photo/diagram placeholders removed. Add: all printed parts laid out and labelled; assembled index finger straight and bent; motor bank labelled M1-M8; tendon routing diagram for one joint (material, length, how to tie off); wiring photo and diagram.
- `content/docs/build-guide.mdx`: add the STL/3MF files with per-part print settings (Printing section), and tendon material, diameter, routing and pre-tension (Tendons section).
- `content/docs/faq.mdx`: add a cost estimate in "What does it cost to build?" once the BOM has prices.

## Contribute page (`content/pages/contribute.mdx`)
- Add `CONTRIBUTING.md` to the repository (code style, tests, commit messages) and link it.
- Adopt a code of conduct (e.g. Contributor Covenant), add `CODE_OF_CONDUCT.md` and link it.

## Software page (`content/pages/software.mdx`)
- Add a screenshot or short video of the digital twin (the real finger following a slider in MuJoCo).

## Site-wide
- `content/site.json`: `url` is still `https://tendra-hand.example.com` with a `_todo_url` note (the security agent handles it); set the real domain.
- `media/screenshots/2026-09-29-teleop-open-hand-with-people.png` is not used anywhere, but `scripts/sync-styleguide.mjs` copies all of `media/` into `public/media/`, so it is publicly reachable by URL. Check it has no faces or anyone's private space before deploying, or move it out of `media/`.
