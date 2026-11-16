"""Actors inside buildings: scan a trial's frames.csv against the map's building footprints (tools/trial_buildings.py).

A vehicle or walker inside a building on screen is the most visible fault a demo clip can have (a bus driving into
a station building in the Shibuya twin). This reads only the logs, so it runs anywhere:

* frames.csv: every actor the logger wrote (participants and scripted agents every frame, AI vehicles, cyclists and
  walkers while near the participants), in CARLA x, y;
* the footprints: for a baked twin, maps/<name>/twin_buildings.json, the twin's solid footprints *after* its road
  clearance cut (carla-digitaltwins tools/road_clearance.py writes <Name>.buildings.json next to the twin's
  OpenDRIVE; mapbuild copies it here), so a lane that runs through a passage cut into a building does not count;
  otherwise maps/<name>/buildings.json (the OSM / city-model footprints of the map).

A vehicle counts as inside when at least two corners of its body (blueprint box, shrunk by SHRINK_M on every side)
are inside one footprint, a walker when its centre is. Each actor's stays inside are merged into episodes (time,
place, footprint id). The same footprint file serves coupledcarla.clipcheck (feat-traffic) for its "building" check:
footprints_path() picks it.
"""
import csv
import json
import math
from pathlib import Path

from . import drivecheck

SHRINK_M = 0.3          # vehicle boxes are shrunk by this on every side (mirrors, a wheel on the kerb line)
WALKER_IN_M = 0.2       # a walker counts only this far inside a footprint (one standing at the wall is fine)
GRID_M = 20.0
GAP_S = 0.5             # stays closer than this in time are one episode
DEFAULT_BOX = {"vehicle": (4.6, 1.9), "bus": (12.0, 2.55), "bike": (1.8, 0.7)}


def footprints_path(map_dir):
    """The footprint file of a map folder: the twin's cut footprints when there are some, else buildings.json."""
    map_dir = Path(map_dir)
    for name in ("twin_buildings.json", "buildings.json"):
        if (map_dir / name).exists():
            return map_dir / name
    return None


def map_dir_of(trial, root=None):
    """maps/<map.name> of a trial (its config.yaml), in ``root`` or the checkout the trial's logs are in."""
    trial = Path(trial)
    try:
        import yaml
        cfg = yaml.safe_load((trial / "config.yaml").read_text()) or {}
    except (OSError, ValueError, ImportError):
        return None
    name = (cfg.get("map") or {}).get("name")
    if not name:
        return None
    bases = [Path(root)] if root else []
    bases += [p for p in trial.resolve().parents]
    for base in bases:
        if (base / "maps" / name).is_dir():
            return base / "maps" / name
    return None


def load_footprints(path):
    """[(xmin, ymin, xmax, ymax, ring, id)] of a footprint file ([{p: [[x, y], ...], id?}], CARLA x, y)."""
    try:
        raw = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return []
    out = []
    for i, b in enumerate(raw):
        p = b.get("p") if isinstance(b, dict) else None
        if not p or len(p) < 3:
            continue
        ring = [(float(q[0]), float(q[1])) for q in p]
        xs, ys = [q[0] for q in ring], [q[1] for q in ring]
        out.append((min(xs), min(ys), max(xs), max(ys), ring, str(b.get("id", i))))
    return out


def point_in_ring(x, y, ring):
    inside = False
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def dist_to_ring(x, y, ring):
    best = float("inf")
    n = len(ring)
    for i in range(n):
        (ax, ay), (bx, by) = ring[i], ring[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        t = 0.0 if L < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L))
        best = min(best, math.hypot(x - ax - t * dx, y - ay - t * dy))
    return best


class Footprints:
    """Footprints on a grid for point queries."""

    def __init__(self, items):
        self.items = items
        self.grid = {}
        for i, b in enumerate(items):
            for gx in range(int(b[0] // GRID_M), int(b[2] // GRID_M) + 1):
                for gy in range(int(b[1] // GRID_M), int(b[3] // GRID_M) + 1):
                    self.grid.setdefault((gx, gy), []).append(i)

    def __len__(self):
        return len(self.items)

    def containing(self, x, y):
        """Indices of the footprints that hold the point."""
        out = []
        for i in self.grid.get((int(x // GRID_M), int(y // GRID_M)), ()):
            b = self.items[i]
            if b[0] <= x <= b[2] and b[1] <= y <= b[3] and point_in_ring(x, y, b[4]):
                out.append(i)
        return out


def body_of(type_id, sizes):
    """(half length, half width) of a vehicle blueprint, from the measured sizes or a default by kind."""
    s = sizes.get(type_id) if sizes else None
    if s:
        return s["length"] / 2.0, min(s["width"], 2.55) / 2.0
    t = type_id.lower()
    L, W = DEFAULT_BOX["bus" if ("bus" in t or "fusorosa" in t) else
                       "bike" if any(k in t for k in ("bike", "cycle", "bh.", "gazelle", "diamondback", "vespa",
                                                      "yamaha", "kawasaki", "harley"))
                       else "vehicle"]
    return L / 2.0, W / 2.0


def actor_inside(row, fp, sizes):
    """Footprint index a frames.csv row is inside, or None."""
    x, y = float(row["pos_x"]), float(row["pos_y"])
    t = row["type_id"]
    if t.startswith("walker."):
        for i in fp.containing(x, y):
            if dist_to_ring(x, y, fp.items[i][4]) >= WALKER_IN_M:
                return i
        return None
    hx, hy = body_of(t, sizes)
    corners = drivecheck.obb_corners(x, y, float(row["rot_yaw"]), max(0.0, hx - SHRINK_M), max(0.0, hy - SHRINK_M))
    cand = set(fp.containing(x, y))
    for cx, cy in corners:
        cand.update(fp.containing(cx, cy))
    for i in cand:
        if sum(point_in_ring(cx, cy, fp.items[i][4]) for cx, cy in corners) >= 2:
            return i
    return None


def scan_trial(trial, map_dir=None, footprints=None, sizes=None, t0=None, t1=None, every=1):
    """Actors inside buildings over a trial (or its window t0..t1, trial time). Returns
    {footprints, rows, actors, episodes: [{actor, kind, type, building, t, t_end, x, y}], inside_rows}."""
    trial = Path(trial)
    path = Path(footprints) if footprints else footprints_path(map_dir or map_dir_of(trial) or trial)
    fp = Footprints(load_footprints(path) if path else [])
    if sizes is None:
        try:
            from . import drivers
            sizes = drivers.sizes()
        except Exception:  # noqa: BLE001 - the defaults by kind
            sizes = {}
    rep = {"footprints": str(path) if path else None, "buildings": len(fp), "rows": 0, "inside_rows": 0,
           "actors": 0, "episodes": []}
    if not len(fp):
        return rep
    open_ep = {}
    actors = set()
    frame_no = 0
    last_frame = None
    with open(trial / "frames.csv", newline="") as f:
        for row in csv.DictReader(f, delimiter=";"):
            t = float(row["timestamp"])
            if (t0 is not None and t < t0) or (t1 is not None and t > t1):
                continue
            if row["frame"] != last_frame:
                last_frame = row["frame"]
                frame_no += 1
            if every > 1 and frame_no % every:
                continue
            ty = row["type_id"]
            if not (ty.startswith("vehicle.") or ty.startswith("walker.")):
                continue
            if float(row["pos_z"]) < -10.0:      # a held car waits under the road
                continue
            rep["rows"] += 1
            actors.add(row["actor_id"])
            i = actor_inside(row, fp, sizes)
            aid = row["actor_id"]
            if i is None:
                continue
            rep["inside_rows"] += 1
            ep = open_ep.get(aid)
            if ep is not None and ep["building"] == fp.items[i][5] and t - ep["t_end"] <= GAP_S:
                ep["t_end"] = t
                continue
            ep = {"actor": aid, "kind": row["kind"], "type": ty, "building": fp.items[i][5], "t": t, "t_end": t,
                  "x": round(float(row["pos_x"]), 2), "y": round(float(row["pos_y"]), 2),
                  "lat": row.get("lat"), "lon": row.get("lon")}
            open_ep[aid] = ep
            rep["episodes"].append(ep)
    rep["actors"] = len(actors)
    return rep


def summary_lines(name, rep):
    eps = rep["episodes"]
    head = (f"{name}: {len(eps)} stays of {len({e['actor'] for e in eps})} actors inside buildings "
            f"({rep['inside_rows']} of {rep['rows']} actor positions; {rep['buildings']} footprints from "
            f"{Path(rep['footprints']).name if rep['footprints'] else 'none'})")
    lines = [head]
    for e in sorted(eps, key=lambda e: -(e["t_end"] - e["t"]))[:12]:
        lines.append(f"    {e['kind']} {e['actor']} ({e['type']}) in building {e['building']} at "
                     f"t {e['t']:.1f}-{e['t_end']:.1f} s, x {e['x']} y {e['y']}"
                     + (f" ({e['lat']}, {e['lon']})" if e.get("lat") else ""))
    return lines
