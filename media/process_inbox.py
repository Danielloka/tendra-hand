"""Prepare new files in media/inbox/ and add them to media/catalog.json.

Photos are turned upright, shrunk to at most 2000 px on the long side and saved without EXIF
metadata (so no GPS location ends up in the public repo). Videos and SVGs are moved as they are.
Every new file gets a catalog entry with a `todo` for the description, alt text and caption.

Usage:  uv run python media/process_inbox.py [--dry-run]
HEIC:   uv run --with pillow-heif python media/process_inbox.py
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import shutil
from pathlib import Path

from PIL import Image, ImageOps

MEDIA = Path(__file__).resolve().parent
INBOX = MEDIA / "inbox"
CATALOG = MEDIA / "catalog.json"
MAX_SIDE = 2000
JPEG_QUALITY = 85

PHOTO_EXT = {".jpg", ".jpeg", ".heic", ".heif", ".webp", ".tif", ".tiff", ".bmp"}
SCREENSHOT_EXT = {".png"}
VIDEO_EXT = {".mp4", ".mov", ".webm", ".m4v"}
DIAGRAM_EXT = {".svg"}
# Where each type is stored inside media/.
FOLDER = {"photo": "photos", "screenshot": "screenshots", "video": "videos", "diagram": "diagrams"}

EXIF_IFD = 0x8769
DATE_TAKEN = 36867  # DateTimeOriginal
DATE_TIME = 306  # DateTime

try:  # optional: iPhone photos
    from pillow_heif import register_heif_opener

    register_heif_opener()
except ImportError:
    pass


def kind_of(path: Path) -> str | None:
    ext = path.suffix.lower()
    if ext in PHOTO_EXT:
        return "photo"
    if ext in SCREENSHOT_EXT:
        return "screenshot"
    if ext in VIDEO_EXT:
        return "video"
    if ext in DIAGRAM_EXT:
        return "diagram"
    return None


def slugify(stem: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", stem.lower()).strip("-")
    return slug or "file"


def date_taken(img: Image.Image, fallback: dt.date) -> dt.date:
    exif = img.getexif()
    raw = exif.get_ifd(EXIF_IFD).get(DATE_TAKEN) or exif.get(DATE_TIME)
    if isinstance(raw, str):
        try:
            year, month, day = (int(part) for part in raw.strip()[:10].split(":"))
            return dt.date(year, month, day)
        except ValueError:
            pass
    return fallback


def unique_path(folder: Path, stem: str, ext: str) -> Path:
    path = folder / f"{stem}{ext}"
    n = 2
    while path.exists():
        path = folder / f"{stem}-{n}{ext}"
        n += 1
    return path


def save_image(src: Path, kind: str, today: dt.date, dry_run: bool) -> tuple[Path, dict]:
    with Image.open(src) as img:
        taken = date_taken(img, today)
        img = ImageOps.exif_transpose(img)  # apply the camera's rotation, then drop EXIF
        img.thumbnail((MAX_SIDE, MAX_SIDE), Image.Resampling.LANCZOS)
        keep_png = kind == "screenshot"
        ext = ".png" if keep_png else ".jpg"
        stem = f"{taken.isoformat()}-{slugify(src.stem)}"
        dest = unique_path(MEDIA / FOLDER[kind], stem, ext)
        if not dry_run:
            dest.parent.mkdir(exist_ok=True)
            if keep_png:
                img.save(dest, optimize=True)  # no pnginfo, so text chunks are dropped too
            else:
                img = img.convert("RGB")
                icc = img.info.get("icc_profile")
                img.save(
                    dest, quality=JPEG_QUALITY, optimize=True, progressive=True, icc_profile=icc
                )
        return dest, {"date": taken.isoformat(), "width": img.width, "height": img.height}


def move_as_is(src: Path, kind: str, today: dt.date, dry_run: bool) -> tuple[Path, dict]:
    stem = f"{today.isoformat()}-{slugify(src.stem)}"
    dest = unique_path(MEDIA / FOLDER[kind], stem, src.suffix.lower())
    if not dry_run:
        dest.parent.mkdir(exist_ok=True)
        shutil.move(src, dest)
    return dest, {"date": today.isoformat(), "width": None, "height": None}


def entry(dest: Path, kind: str, info: dict) -> dict:
    item = {
        "file": dest.relative_to(MEDIA).as_posix(),
        "type": kind,
        "date": info["date"],
        "hand": None,
        "description": "",
        "alt": "",
        "caption": "",
        "tags": [],
        "width": info["width"],
        "height": info["height"],
        "used_in": [],
        "todo": "Describe it: description, alt, caption, tags, hand; give it a descriptive name.",
    }
    if kind == "video":
        item["poster"] = None
        item["todo"] += " Add a poster frame in photos/ and fill in width/height."
    return item


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dry-run", action="store_true", help="show what would happen")
    args = parser.parse_args()

    INBOX.mkdir(exist_ok=True)
    catalog = json.loads(CATALOG.read_text(encoding="utf-8"))
    today = dt.datetime.now().astimezone().date()
    added = 0

    for src in sorted(p for p in INBOX.iterdir() if p.is_file() and not p.name.startswith(".")):
        kind = kind_of(src)
        if kind is None:
            print(f"skip   {src.name} (unknown file type)")
            continue
        try:
            if kind in ("photo", "screenshot"):
                dest, info = save_image(src, kind, today, args.dry_run)
                if not args.dry_run:
                    src.unlink()
            else:
                dest, info = move_as_is(src, kind, today, args.dry_run)
        except (OSError, ValueError) as exc:  # keep going; leave the file in the inbox
            heic = src.suffix.lower() in {".heic", ".heif"}
            hint = " (install pillow-heif, see the docstring)" if heic else ""
            print(f"error  {src.name}: {exc}{hint}")
            continue
        catalog["items"].append(entry(dest, kind, info))
        added += 1
        size = f"{info['width']}x{info['height']}" if info["width"] else "as is"
        print(f"added  {src.name} -> {dest.relative_to(MEDIA).as_posix()} ({size})")

    if added and not args.dry_run:
        catalog["items"].sort(key=lambda it: (it["date"], it["file"]))
        CATALOG.write_text(
            json.dumps(catalog, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    verb = "would add" if args.dry_run else "added"
    print(f"{verb} {added} file(s). Next: fill in the todo fields in media/catalog.json.")


if __name__ == "__main__":
    main()
