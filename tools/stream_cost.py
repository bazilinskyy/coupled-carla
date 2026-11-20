"""What smaller streaming radii cost in realism along a trial's route, without a CARLA server.

    python tools/stream_cost.py EXPERIMENT.yaml TRIAL_DIR [--view 150] [--scenery-radii 200 150 100]
        [--parked-radii 250 150 100] [--lod-radii 250 150] [--animate-radii 60 30 0] [--md out.md]

Replays the scenery's and the parked cars' cell streaming (scenery.Scenery.update: 100 m cells loaded when within
the radius of the participant, kept until beyond radius + one cell, updated once a second) along the driver's path in
TRIAL_DIR/frames.csv, for each radius: the actors loaded on average (what every tick's snapshot carries) and the
objects within --view m of the driver that are not loaded (lost near the route), by kind. From the trial's SUMO
output (sumo/sumo_fcd.xml.gz): the SUMO vehicles within --view m that a smaller simulation.lod_radius_m leaves out of
CARLA, and those shown without physics (no rolling wheels, no pedalling) at a smaller animate_radius_m. The route and
the traffic are the trial's own (the same seed gives the same ones), so the numbers are per setting, not per run.
"""
import argparse
import csv
import gzip
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from coupledcarla import config, parking  # noqa: E402

CELL = 100.0


def driver_path(trial_dir, every=50):
    """[(frame index, x, y)] of the driver every `every` ticks."""
    out = []
    with open(Path(trial_dir) / "frames.csv", newline="") as f:
        rows = [r for r in csv.DictReader(f, delimiter=";") if r["role"] == "driver"]
    for i, r in enumerate(rows):
        if i % every == 0:
            out.append((i, float(r["pos_x"]), float(r["pos_y"])))
    return out


def cells_of(items):
    cells = defaultdict(list)
    for it in items:
        if it is None:
            continue
        cx, cy = it[8] if len(it) > 8 else it[1:3]
        cells[(int(cx // CELL), int(cy // CELL))].append((cx, cy, it[6]))
    return cells


def cell_dist(i, j, x, y):
    dx = max(0.0, abs((i + 0.5) * CELL - x) - CELL / 2)
    dy = max(0.0, abs((j + 0.5) * CELL - y) - CELL / 2)
    return math.hypot(dx, dy)


def stream(cells, path, radius, view):
    """(mean actors loaded, mean objects within view not loaded, mean objects within view, lost by kind)."""
    loaded = set()
    n_loaded, lost, near, kinds = [], [], [], Counter()
    k = int((max(radius, view) + CELL) // CELL) + 1
    for _, x, y in path:
        cx, cy = int(x // CELL), int(y // CELL)
        want, keep = set(), set()
        for i in range(cx - k, cx + k + 1):
            for j in range(cy - k, cy + k + 1):
                if (i, j) not in cells:
                    continue
                d = cell_dist(i, j, x, y)
                if d <= radius:
                    want.add((i, j))
                if d <= radius + CELL:
                    keep.add((i, j))
        loaded = (loaded & keep) | want
        n_loaded.append(sum(len(cells[c]) for c in loaded))
        n_near = n_lost = 0
        kv = int(view // CELL) + 1
        for i in range(cx - kv, cx + kv + 1):
            for j in range(cy - kv, cy + kv + 1):
                for ox, oy, kind in cells.get((i, j), ()):
                    if math.hypot(ox - x, oy - y) <= view:
                        n_near += 1
                        if (i, j) not in loaded:
                            n_lost += 1
                            kinds[kind] += 1
        near.append(n_near)
        lost.append(n_lost)
    m = max(1, len(path))
    return sum(n_loaded) / m, sum(lost) / m, sum(near) / m, {k: v / m for k, v in kinds.most_common(4)}


def traffic(trial_dir, path, geo_net, view, lod_radii, animate_radii):
    """{('lod', r): mean SUMO vehicles within view beyond r, ('animate', r): mean shown within lod but beyond r}."""
    import sumolib
    net = sumolib.net.readNet(str(geo_net), withInternal=False, withPedestrianConnections=False)
    at = {round(i * 0.02, 1): (x, y) for i, x, y in path}        # driver every 1 s, keyed by sim time
    sums, n = Counter(), 0
    with gzip.open(Path(trial_dir) / "sumo" / "sumo_fcd.xml.gz") as f:
        for _, el in ET.iterparse(f):
            if el.tag != "timestep":
                continue
            t = round(float(el.get("time")), 1)
            if t in at:
                dx, dy = at[t]
                n += 1
                ds = []
                for v in el.findall("vehicle"):
                    if v.get("id", "").startswith("ego_"):
                        continue
                    gx, gy = net.convertLonLat2XY(float(v.get("x")), float(v.get("y")))
                    ds.append(math.hypot(gx - dx, -gy - dy))      # CARLA (x, y) = geo.net (x, -y)
                for r in lod_radii:
                    sums[("lod", r)] += sum(1 for d in ds if r < d <= view)
                    sums[("shown", r)] += sum(1 for d in ds if d <= min(r, view))
                for r in animate_radii:
                    sums[("animate", r)] += sum(1 for d in ds if r < d <= min(max(lod_radii), view))
                    sums[("physics", r)] += sum(1 for d in ds if d <= r)
            el.clear()
    return {k: v / max(1, n) for k, v in sums.items()}, n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("experiment")
    ap.add_argument("trial_dir")
    ap.add_argument("--trial", type=int, default=0)
    ap.add_argument("--view", type=float, default=150.0)
    ap.add_argument("--scenery-radii", type=float, nargs="+", default=[400, 200, 150, 100])
    ap.add_argument("--parked-radii", type=float, nargs="+", default=[250, 150, 100])
    ap.add_argument("--lod-radii", type=float, nargs="+", default=[250, 150, 100])
    ap.add_argument("--animate-radii", type=float, nargs="+", default=[60, 30, 0])
    ap.add_argument("--md", default=None)
    a = ap.parse_args()
    cfg = config.load(a.experiment)
    tc = config.trial_config(cfg, cfg["trials"][a.trial])
    mdir = config.map_dir(tc)
    path = driver_path(a.trial_dir)
    lines = [f"Route: {len(path)} s of the driver in {a.trial_dir}; objects counted within {a.view:.0f} m.", "",
             "| what | radius (m) | actors loaded (mean) | lost within view (mean) | of those in view | lost by kind |",
             "|---|---|---|---|---|---|"]
    scen = json.loads((mdir / "scenery.json").read_text())
    kinds = set((tc["simulation"].get("scenery") or {}).get("kinds") or [])
    items = [it for it in scen["items"] if it and (not kinds or it[6] in kinds or it[6] == "landmark")]
    for r in a.scenery_radii:
        nl, lost, near, by = stream(cells_of(items), path, r, a.view)
        lines.append(f"| scenery | {r:.0f} | {nl:.0f} | {lost:.0f} | {100 * lost / max(near, 1):.0f} % | "
                     + ", ".join(f"{k} {v:.0f}" for k, v in by.items()) + " |")
    pdata = parking.trial_data(mdir, tc, lambda m: None)
    if pdata and pdata["items"]:
        for r in a.parked_radii:
            nl, lost, near, _ = stream(cells_of(pdata["items"]), path, r, a.view)
            lines.append(f"| parked cars | {r:.0f} | {nl:.0f} | {lost:.0f} | {100 * lost / max(near, 1):.0f} % | |")
    tr, n = traffic(a.trial_dir, path, mdir / "geo.net.xml", a.view, a.lod_radii, a.animate_radii)
    lines += ["", f"SUMO traffic within {a.view:.0f} m ({n} samples):", "",
              "| setting | radius (m) | vehicles in CARLA within view | left out within view |", "|---|---|---|---|"]
    for r in a.lod_radii:
        lines.append(f"| lod_radius_m | {r:.0f} | {tr[('shown', r)]:.1f} | {tr[('lod', r)]:.1f} |")
    lines += ["", "| setting | radius (m) | vehicles with physics | shown without physics (no wheels/pedalling) |",
              "|---|---|---|---|"]
    for r in a.animate_radii:
        lines.append(f"| animate_radius_m | {r:.0f} | {tr[('physics', r)]:.1f} | {tr[('animate', r)]:.1f} |")
    text = "\n".join(lines)
    print(text)
    if a.md:
        Path(a.md).write_text(text + "\n")


if __name__ == "__main__":
    main()
