"""Figure of the built maps: roads, cycleways and OSM buildings per city, in a grid.

    python tools/plot_maps.py docs/img/cities.png
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import sumolib  # noqa: E402
import yaml  # noqa: E402
from matplotlib.collections import LineCollection, PolyCollection  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent


def draw(ax, map_dir, title):
    net = sumolib.net.readNet(str(map_dir / "geo.net.xml"), withInternal=False)
    roads, bikes = [], []
    for e in net.getEdges():
        for lane in e.getLanes():
            pts = [(x, -y) for x, y in lane.getShape()]
            if lane.allows("passenger"):
                roads.append(pts)
            elif lane.allows("bicycle"):
                bikes.append(pts)
    if (map_dir / "buildings.json").exists():
        polys = [p["p"] for p in json.loads((map_dir / "buildings.json").read_text())]
        ax.add_collection(PolyCollection(polys, facecolors="#d9d2c5", edgecolors="none"))
    ax.add_collection(LineCollection(bikes, colors="#b5523f", linewidths=0.6))
    ax.add_collection(LineCollection(roads, colors="#3b4048", linewidths=0.9))
    xs = [x for r in roads for x, _ in r]
    ys = [y for r in roads for _, y in r]
    if xs:
        cx, cy, half = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, 720
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy + half, cy - half)   # CARLA y points south: north up
    ax.set_aspect("equal")
    ax.set_xticks([]), ax.set_yticks([])
    ax.set_title(title, fontsize=8)
    for s in ax.spines.values():
        s.set_color("#c8ced6")


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
