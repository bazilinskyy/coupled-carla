"""Figure of the built maps: roads, cycleways and OSM buildings per city, in a grid.

    python tools/plot_maps.py docs/img/cities.png
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from coupledcarla.preview import draw  # noqa: E402  (shared with the map maker's preview pictures)


def main(out):
    cat = yaml.safe_load((ROOT / "experiments" / "cities.yaml").read_text())
    entries = [("eindhoven_centre", "Eindhoven (NL, Europe)\nuse case")]
    for c in cat["cities"]:
        entries.append((c["slug"], f"{c['name']}\n{c['continent']}, {c['size']}, ~{c['population'] / 1e6:.2g} M"))
    entries = [(s, t) for s, t in entries if (ROOT / "maps" / s / "geo.net.xml").exists()]
    cols = 5
    rows = (len(entries) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 2.6, rows * 2.9), dpi=150)
    for ax in axes.flat[len(entries):]:
        ax.axis("off")
    for ax, (slug, title) in zip(axes.flat, entries):
        draw(ax, ROOT / "maps" / slug, title)
    fig.suptitle("Maps built from OpenStreetMap: drivable roads (dark), cycleways (red), buildings (beige); "
                 "1.4 km squares, north up", fontsize=9)
    fig.tight_layout()
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "docs/img/cities.png")
