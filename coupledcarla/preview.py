"""Pictures of built maps from above: roads, cycleways, buildings, trees and water (matplotlib, sumolib).

Used by tools/plot_maps.py (the grid of all cities) and the map maker (one preview.png next to each map).
"""
import json
from pathlib import Path

ROAD, BIKE, BUILDING, TREE, WATER, EDGE = "#3b4048", "#b5523f", "#d9d2c5", "#5f9e4f", "#9cc3e6", "#c8ced6"


def _layers(map_dir):
    """Roads and cycleways ([(x, y), ...] in CARLA metres), building footprints, trees and water from a map folder."""
    import sumolib
    map_dir = Path(map_dir)
    net = sumolib.net.readNet(str(map_dir / "geo.net.xml"), withInternal=False)
    roads, bikes = [], []
    for e in net.getEdges():
        for lane in e.getLanes():
            pts = [(x, -y) for x, y in lane.getShape()]
            if lane.allows("passenger"):
                roads.append(pts)
            elif lane.allows("bicycle"):
                bikes.append(pts)
    polys = []
    if (map_dir / "buildings.json").exists():
        polys = [p["p"] for p in json.loads((map_dir / "buildings.json").read_text())]
    trees, water = [], []
    if (map_dir / "scenery.json").exists():
        items = json.loads((map_dir / "scenery.json").read_text()).get("items", [])
        trees = [(it[1], it[2], it[5]) for it in items if it[6] in ("tree", "bush")]
        water = [(it[1], it[2]) for it in items if it[6] == "water"]
    return net, roads, bikes, polys, trees, water


def box_xy(net, bbox):
    """The map's [south, west, north, east] box as CARLA (x0, x1, y0, y1)."""
    s, w, n, e = bbox
    x0, y0 = net.convertLonLat2XY(w, s)
    x1, y1 = net.convertLonLat2XY(e, n)
    return min(x0, x1), max(x0, x1), min(-y0, -y1), max(-y0, -y1)


def draw(ax, map_dir, title=None, half=720.0, trees=False, bbox=None, fontsize=8):
    """Draw a built map on a matplotlib axis, north up. half: half the shown square (m) around the roads' centre;
    with bbox the map's own box is shown instead. trees: also trees (green) and water (blue) from scenery.json."""
    from matplotlib.collections import LineCollection, PolyCollection
    net, roads, bikes, polys, tree_pts, water = _layers(map_dir)
    if trees and water:
        ax.scatter([x for x, _ in water], [y for _, y in water], s=14, marker="s", c=WATER, linewidths=0, zorder=0)
    if polys:
        ax.add_collection(PolyCollection(polys, facecolors=BUILDING, edgecolors="none", zorder=1))
    if trees and tree_pts:
        ax.scatter([t[0] for t in tree_pts], [t[1] for t in tree_pts], s=[3 + 4 * t[2] for t in tree_pts],
                   c=TREE, linewidths=0, alpha=0.8, zorder=2)
    ax.add_collection(LineCollection(bikes, colors=BIKE, linewidths=0.6, zorder=3))
    ax.add_collection(LineCollection(roads, colors=ROAD, linewidths=0.9, zorder=4))
    if bbox:
        x0, x1, y0, y1 = box_xy(net, bbox)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y1, y0)                 # CARLA y points south: north up
    else:
        xs = [x for r in roads for x, _ in r]
        ys = [y for r in roads for _, y in r]
        if xs:
            cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
            ax.set_xlim(cx - half, cx + half)
            ax.set_ylim(cy + half, cy - half)
    ax.set_aspect("equal")
    ax.set_xticks([]), ax.set_yticks([])
    if title:
        ax.set_title(title, fontsize=fontsize)
    for s in ax.spines.values():
        s.set_color(EDGE)
    return ax


def picture(map_dir, out, title="", subtitle="", bbox=None, px=900):
    """One map as a PNG of about px x px: roads, cycleways, buildings, trees and water, a 200 m scale bar."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    dpi = 100
    fig, ax = plt.subplots(figsize=(px / dpi, px / dpi * 1.06), dpi=dpi)
    draw(ax, map_dir, trees=True, bbox=bbox)
    x0, x1 = ax.get_xlim()
    y1, y0 = ax.get_ylim()
    bar = 200.0 if (x1 - x0) > 700 else 100.0
    bx, by = x0 + 0.04 * (x1 - x0), y1 - 0.04 * (y1 - y0)
    ax.plot([bx, bx + bar], [by, by], color="black", linewidth=3, solid_capstyle="butt", zorder=6)
    ax.text(bx + bar / 2, by - 0.015 * (y1 - y0), f"{bar:.0f} m", ha="center", va="bottom", fontsize=9, zorder=6)
    ax.text(x1 - 0.04 * (x1 - x0), y0 + 0.05 * (y1 - y0), "N\n↑", ha="center", va="top", fontsize=11,
            zorder=6)
    if title:
        fig.suptitle(title, fontsize=13, y=0.985)
    if subtitle:
        ax.set_title(subtitle, fontsize=8.5, color="#555")
    fig.tight_layout()
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)
    return Path(out)
