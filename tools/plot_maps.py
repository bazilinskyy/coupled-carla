"""Figure of the built maps: roads, cycleways and OSM buildings per city, in a grid, each title with its
country's flag.

    python tools/plot_maps.py docs/img/cities.png

Flags are drawn from the Noto Color Emoji font (OFL; its flag images are public domain); without that font the
titles stay as they are.
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib.text import Text  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from coupledcarla.cities import ISO2, flag  # noqa: E402
from coupledcarla.preview import draw  # noqa: E402  (shared with the map maker's preview pictures)

EMOJI_FONTS = ["/usr/share/fonts/truetype/noto/NotoColorEmoji.ttf",
               "/usr/share/fonts/noto/NotoColorEmoji.ttf",
               "/usr/share/fonts/google-noto-emoji/NotoColorEmoji.ttf"]


def flag_image(code, height_px):
    """RGBA array of a country's flag, height_px high; None without a colour emoji font or a valid code."""
    text = flag(code=code)
    if not text:
        return None
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return None
    for path in EMOJI_FONTS:
        if Path(path).exists():
            break
    else:
        return None
    font = ImageFont.truetype(path, 109)          # the bitmap size of Noto Color Emoji
    im = Image.new("RGBA", (160, 160), (0, 0, 0, 0))
    ImageDraw.Draw(im).text((0, 0), text, font=font, embedded_color=True)
    im = im.crop(im.getbbox())
    width = max(1, round(im.width * height_px / im.height))
    return np.asarray(im.resize((width, int(height_px)), Image.LANCZOS))


def add_title_flags(fig, axes_codes, gap_px=4):
    """Put each axes' country flag left of the first line of its title (figure pixels, after layout)."""
    renderer = fig.canvas.get_renderer()
    for ax, code in axes_codes:
        title = ax.title
        lines = title.get_text().split("\n")
        if not code or not lines[0]:
            continue
        box = title.get_window_extent(renderer)
        probe = Text(text=lines[0], fontproperties=title.get_fontproperties(), figure=fig)
        first = probe.get_window_extent(renderer)
        line_h = box.height / len(lines)
        img = flag_image(code, round(line_h * 0.8))
        if img is None:
            return
        cx = (box.x0 + box.x1) / 2
        x = cx - first.width / 2 - gap_px - img.shape[1]
        y = box.y1 - line_h + (line_h - img.shape[0]) / 2
        fig.figimage(img, xo=round(x), yo=round(y), origin="upper", zorder=5)


def main(out):
    cat = yaml.safe_load((ROOT / "experiments" / "cities.yaml").read_text())
    entries = [("eindhoven_centre", "Eindhoven\nEurope, use case", "NL")]
    for c in cat["cities"]:
        entries.append((c["slug"], f"{c['name']}\n{c['continent']}, {c['size']}, ~{c['population'] / 1e6:.2g} M",
                        ISO2.get(c.get("country", ""))))
    entries = [e for e in entries if (ROOT / "maps" / e[0] / "geo.net.xml").exists()]
    cols = 5
    rows = (len(entries) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 2.6, rows * 2.9), dpi=150)
    for ax in axes.flat[len(entries):]:
        ax.axis("off")
    for ax, (slug, title, _) in zip(axes.flat, entries):
        draw(ax, ROOT / "maps" / slug, title)
    fig.suptitle("Maps built from OpenStreetMap: drivable roads (dark), cycleways (red), buildings (beige); "
                 "1.4 km squares, north up", fontsize=9)
    fig.tight_layout()
    fig.canvas.draw()
    add_title_flags(fig, [(ax, code) for ax, (_, _, code) in zip(axes.flat, entries)])
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=fig.dpi)
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "docs/img/cities.png")
