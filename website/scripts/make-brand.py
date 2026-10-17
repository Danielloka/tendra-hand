"""Make every Tendra logo file from one drawing of the mark ("Wave", chosen 2026-10-17).

The mark: an open hand, four fingers with a knuckle joint and a thumb swung out,
tilted forward as if it is moving. Edit HAND below, then re-run:

    uv run --with resvg-py --with pillow python website/scripts/make-brand.py

Writes:
    public/brand/tendra-mark{,-blue,-white}.svg   the bare mark (dark / accent / white)
    public/brand/tendra-icon.svg, tendra-icon-512.png   app icon: white mark on the blue tile
    src/app/icon.svg, apple-icon.png, favicon.ico       picked up by Next.js as the site icons
and prints the data URI used by .nav__brand-mark in assets/css/components.css.
"""

import io
from pathlib import Path
from urllib.parse import quote

import resvg_py
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
DARK, ACCENT, ACCENT_2 = "#1d1d1f", "#0066cc", "#5aa9ff"  # tokens.css: --text, --accent, the brand gradient's end

# 64 x 64 grid. Fingers: (x, y, height) of each phalanx, index -> little; thumb: two phalanges, rotated.
FINGERS = [
    (21.5, 40.88, 15.12), (21.5, 20.00, 18.68),
    (30.5, 39.62, 16.38), (30.5, 17.00, 20.42),
    (39.5, 40.46, 15.54), (39.5, 19.00, 19.26),
    (48.5, 42.98, 13.02), (48.5, 25.00, 15.78),
]
THUMB = [(15.3, 40.00, 16.00), (15.3, 26.80, 11.00)]


def rects(parts):
    return "".join(f'<rect x="{x}" y="{y}" width="7" height="{h}" rx="3.5"/>' for x, y, h in parts)


HAND = (
    '<g transform="translate(3.95 -4.48) scale(0.9524)"><g transform="rotate(14 36 40)">'
    f'{rects(FINGERS)}<g transform="rotate(-38 18.8 56.5)">{rects(THUMB)}</g></g></g>'
)


def mark_svg(color: str) -> str:
    return f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64"><g fill="{color}">{HAND}</g></svg>\n'


def icon_svg(radius: float = 14, scale: float = 0.74) -> str:
    """White mark on the blue gradient tile. radius 0 = full square (Apple rounds it itself)."""
    off = 32 * (1 - scale)
    return (
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64">'
        f'<defs><linearGradient id="g" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="{ACCENT}"/>'
        f'<stop offset="1" stop-color="{ACCENT_2}"/></linearGradient></defs>'
        f'<rect width="64" height="64" rx="{radius}" fill="url(#g)"/>'
        f'<g fill="#fff" transform="translate({off:.2f} {off:.2f}) scale({scale})">{HAND}</g></svg>\n'
    )


def png(svg: str, size: int) -> Image.Image:
    sized = svg.replace('viewBox="0 0 64 64"', f'viewBox="0 0 64 64" width="{size}" height="{size}"', 1)
    return Image.open(io.BytesIO(bytes(resvg_py.svg_to_bytes(svg_string=sized)))).convert("RGBA")


def main() -> None:
    brand = ROOT / "public" / "brand"
    brand.mkdir(parents=True, exist_ok=True)
    for name, color in [("tendra-mark", DARK), ("tendra-mark-blue", ACCENT), ("tendra-mark-white", "#ffffff")]:
        (brand / f"{name}.svg").write_text(mark_svg(color), encoding="utf-8")
    (brand / "tendra-icon.svg").write_text(icon_svg(), encoding="utf-8")
    png(icon_svg(), 512).save(brand / "tendra-icon-512.png", optimize=True)

    app = ROOT / "src" / "app"
    (app / "icon.svg").write_text(icon_svg(), encoding="utf-8")
    png(icon_svg(radius=0), 180).save(app / "apple-icon.png", optimize=True)
    # Tiny sizes: a slightly bigger hand reads better at 16-48 px.
    png(icon_svg(scale=0.8), 256).save(app / "favicon.ico", sizes=[(16, 16), (32, 32), (48, 48)])

    nav = mark_svg("#fff").replace("\n", "")
    print("nav mark data URI:\n" + "url('data:image/svg+xml," + quote(nav, safe=' =:/"') + "')")


if __name__ == "__main__":
    main()
