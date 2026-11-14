"""Build a drivable CARLA + SUMO map for a bounding box from OpenStreetMap.

    python -m coupledcarla.mapbuild experiments/eindhoven_centre.yaml

Steps (all with SUMO's netconvert; carla.Osm2Odr segfaults in 0.9.16 on Linux and Windows):
1. OSM extract from Overpass (cached in the map folder).
2. OSM -> SUMO net (geo.net.xml), keeping the configured road types, cycleways and guessed sidewalks.
3. Pruning: dead-end chains are removed and only the largest strongly connected road graph is kept, so a car
   that follows lanes can always go on (at the cut edge of the box, roads would otherwise end in the void).
4. Pruned net -> OpenDRIVE (map.xodr), validated with carla.Map.
5. OpenDRIVE -> SUMO net with CARLA's netconvert_carla.py (map.net.xml), so SUMO and CARLA share road ids.
6. Scenery from real-world data (scenery.py, realworld.py): buildings, trees, street furniture, ground.
geo.net.xml keeps the projection, and CARLA (x, y) = (geo x, -geo y), see geo.py.

With map.elevation, every road node gets its ground height from a terrain model (AHN in the Netherlands,
the AWS Terrain Tiles elsewhere) before step 2, relative to the lowest point of the box, so the roads,
the OpenDRIVE and the SUMO net climb and fall as the real streets do. Bridges and tunnels keep a straight
grade between their ends (the terrain model would drop a bridge into the river), bridges raised by layer.

    python -m coupledcarla.mapbuild experiments/lyon.yaml --scenery    # redo only step 6
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import requests

from . import config, realworld

OVERPASS = ["https://overpass-api.de/api/interpreter", "https://overpass.openstreetmap.fr/api/interpreter",
            "https://overpass.kumi.systems/api/interpreter",
            "https://overpass.private.coffee/api/interpreter",
            "https://maps.mail.ru/osm/tools/overpass/api/interpreter"]


def log(msg, t0=[time.time()]):
    print(f"[mapbuild {time.time() - t0[0]:6.1f}s] {msg}", flush=True)


def fetch_osm(bbox, path):
    if path.exists() and path.stat().st_size > 0:
        return
    s, w, n, e = bbox
    query = f'[out:xml][timeout:180];(way["highway"]({s},{w},{n},{e});>;);out body;'
    for attempt in range(2):
        for url in OVERPASS:
            try:
                r = requests.post(url, data={"data": query}, headers={"User-Agent": "coupled-carla/0.1"},
                                  timeout=120)
                r.raise_for_status()
                if b"<way" in r.content:
                    path.write_bytes(r.content)
                    log(f"OSM from {url}: {len(r.content) / 1e6:.1f} MB")
                    return
            except requests.RequestException as err:
                log(f"Overpass {url} failed: {err}")
        time.sleep(15)
    raise RuntimeError("no Overpass server answered")


def _number(text):
    """'12', '12 m', '12.5;14' -> the first number, or None."""
    import re
    m = re.match(r"\s*([0-9]+(?:\.[0-9]+)?)", str(text or ""))
    return float(m.group(1)) if m else None


def fetch_buildings(bbox, geo_net, path):
    """OSM building footprints in CARLA metres with a height estimate, for the trial viewer and figures."""
    import sumolib
    if not path.exists():
        s, w, n, e = bbox
        query = f'[out:json][timeout:180];way["building"]({s},{w},{n},{e});out geom tags;'
        data = None
        for url in OVERPASS:
            try:
                r = requests.post(url, data={"data": query}, headers={"User-Agent": "coupled-carla/0.1"}, timeout=120)
                r.raise_for_status()
                data = r.json()
                break
            except (requests.RequestException, ValueError) as err:
                log(f"Overpass buildings {url} failed: {err}")
        if data is None:
            return 0
        net = sumolib.net.readNet(str(geo_net), withInternal=False)
        out = []
        for el in data.get("elements", []):
            geom, tags = el.get("geometry"), el.get("tags", {})
            if not geom or len(geom) < 3:
                continue
            pts = []
            for g in geom:
                x, y = net.convertLonLat2XY(g["lon"], g["lat"])
                pts.append([round(x, 2), round(-y, 2)])
            h = _number(tags.get("height"))
            if h is None:
                levels = _number(tags.get("building:levels"))
                h = 3.2 * levels + 1.0 if levels is not None else 9.0
            out.append({"h": round(h, 1), "p": pts})
        path.write_text(json.dumps(out, separators=(",", ":")))
        log(f"{len(out)} building footprints")
    return len(json.loads(path.read_text()))


def add_elevation(osm_in, osm_out, dem, base):
    """Copy the OSM file with an `ele` tag (metres above `base`) on every node of a highway."""
    import math
    import xml.etree.ElementTree as ET
    import numpy as np
    tree = ET.parse(osm_in)
    root = tree.getroot()
    nodes = {n.get("id"): n for n in root.iter("node")}
    used, ways = set(), []
    for w in root.iter("way"):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        if "highway" not in tags:
            continue
        refs = [nd.get("ref") for nd in w.findall("nd") if nd.get("ref") in nodes]
        used.update(refs)
        level = tags.get("bridge", "no") not in ("no", "false") or tags.get("tunnel", "no") not in ("no", "false")
        ways.append((refs, level, tags))
    ids = sorted(used)
    lat = np.array([float(nodes[i].get("lat")) for i in ids])
    lon = np.array([float(nodes[i].get("lon")) for i in ids])
    ele = dict(zip(ids, (dem.sample(lat, lon) - base).tolist()))
    # bridges and tunnels: chains of such ways get heights interpolated between their abutments
    ground_nodes = set(r for refs, level, _ in ways if not level for r in refs)
    parent = {}            # union-find over the nodes of connected bridge/tunnel ways

    def find(a):
        while parent.setdefault(a, a) != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for refs, level, tags in ways:
        if level:
            for a, b in zip(refs, refs[1:]):
                parent[find(a)] = find(b)
    groups = {}
    layer_of = {}
    for refs, level, tags in ways:
        if level:
            try:
                layer = float(tags.get("layer", "1" if tags.get("bridge", "no") != "no" else "-1"))
            except ValueError:
                layer = 0.0
            for r in refs:
                groups.setdefault(find(r), set()).add(r)
                layer_of[r] = layer

    def metres(a, b):
        la, lb = float(nodes[a].get("lat")), float(nodes[b].get("lat"))
        dx = (float(nodes[b].get("lon")) - float(nodes[a].get("lon"))) * 111320 * math.cos(math.radians(la))
        return math.hypot(dx, (lb - la) * 110540)
    levelled = 0
    for members in groups.values():
        ends = [r for r in members if r in ground_nodes]
        if not ends:
            continue
        for r in members:
            if r in ground_nodes:
                continue
            d = [(metres(r, e), e) for e in ends]
            wsum = sum(1.0 / max(di, 1.0) ** 2 for di, _ in d)
            z = sum(ele[e] / max(di, 1.0) ** 2 for di, e in d) / wsum
            near = min(di for di, _ in d)
            if layer_of.get(r, 0) > 0:           # a bridge over another road: up to 5 m per layer, ramped
                z += 5.0 * layer_of[r] * min(1.0, near / 40.0)
            ele[r] = z
            levelled += 1
    for i, z in ele.items():
        n = nodes[i]
        for t in n.findall("tag"):
            if t.get("k") == "ele":
                n.remove(t)
        ET.SubElement(n, "tag", k="ele", v=f"{z:.2f}")
    tree.write(osm_out, encoding="utf-8", xml_declaration=True)
    log(f"elevation on {len(ele)} road nodes ({levelled} on bridges or in tunnels levelled)")


def netconvert(*args):
    subprocess.check_call(["netconvert", "--no-warnings", *args])


def prune(net_file):
    """Edge ids to keep: the largest strongly connected car graph without dead-end chains, plus bike-only edges."""
    import networkx as nx
    import sumolib

    net = sumolib.net.readNet(str(net_file))
    car = [e for e in net.getEdges() if e.allows("passenger")]
    other = [e.getID() for e in net.getEdges() if not e.allows("passenger")]
    und = nx.Graph()
    for e in car:
        und.add_edge(e.getFromNode().getID(), e.getToNode().getID())
    dead = set()
    while True:
        leaves = [n for n in und.nodes if und.degree(n) <= 1]
        if not leaves:
            break
        dead.update(leaves)
        und.remove_nodes_from(leaves)
    dig = nx.DiGraph()
    for e in car:
        a, b = e.getFromNode().getID(), e.getToNode().getID()
        if a not in dead and b not in dead:
            dig.add_edge(a, b)
    core = max(nx.strongly_connected_components(dig), key=len) if dig.number_of_nodes() else set()
    keep = [e.getID() for e in car if e.getFromNode().getID() in core and e.getToNode().getID() in core]
    log(f"pruning: {len(car)} car edges -> {len(keep)} kept ({len(dead)} dead-end nodes, "
        f"core of {len(core)} junctions); {len(other)} bike/foot-only edges kept")
    return keep + other


def name_signals(xodr):
    """netconvert writes <signal> without a name, and netconvert_carla.py skips every unnamed traffic light,
    so SUMO would get none; give each signal its id as name."""
    import re
    text = xodr.read_text(encoding="utf-8")
    text, n = re.subn(r'<signal id="([^"]*)"(?![^>]*\bname=)', r'<signal id="\1" name="\1"', text)
    xodr.write_text(text, encoding="utf-8")
    log(f"named {n} OpenDRIVE signals")


def check_xodr(xodr):
    import carla
    m = carla.Map("check", xodr.read_text(encoding="utf-8"))
    wps = m.generate_waypoints(5.0)
    ends = sum(1 for w in wps if w.lane_type == carla.LaneType.Driving and not w.next(5.0))
    tls = len(m.get_all_landmarks_of_type("1000001"))
    log(f"OpenDRIVE: {len(m.get_topology())} lane links, {len(wps)} waypoints, {ends} lane ends, "
        f"{tls} traffic-light signals")
    return {"lane_links": len(m.get_topology()), "waypoints": len(wps), "lane_ends": ends, "tl_signals": tls}


def build_scenery(cfg, out, meta, cmap=None):
    """Scenery and buildings for a built map (scenery.py), recorded in meta."""
    from . import scenery
    from .geo import Geo
    mc = cfg["map"]
    elev = meta.get("elevation")
    dem = realworld.Dem.load(out / "real" / "dem.npz") if elev else None
    res = scenery.build(out, cfg, Geo(out / "geo.net.xml"), dem, elev["base_m"] if elev else 0.0, cmap, log)
    meta["scenery"] = res["counts"]
    meta["buildings"] = len(json.loads((out / "buildings.json").read_text()))
    srcs = set(res["sources"]) | ({elev["source"]} if elev else set())
    meta["sources"] = {k: {"what": realworld.SOURCES[k][0], "licence": realworld.SOURCES[k][1]}
                       for k in sorted(srcs) if k in realworld.SOURCES}
    return meta


def build(cfg, force=False, scenery_only=False):
    mc = cfg["map"]
    out = config.map_dir(cfg)
    out.mkdir(parents=True, exist_ok=True)
    if scenery_only and (out / "meta.json").exists():
        meta = json.loads((out / "meta.json").read_text())
        build_scenery(cfg, out, meta)
        (out / "meta.json").write_text(json.dumps(meta, indent=2))
        log(f"scenery rebuilt: {out}")
        return out
    if (out / "meta.json").exists() and not force:
        log(f"{out} already built (use --force to rebuild)")
        return out
    if not mc["bbox"]:
        sys.exit("map.bbox is required")

    osm = out / "map.osm"
    fetch_osm(mc["bbox"], osm)
    elevation = None
    if mc.get("elevation"):
        import numpy as np
        dem, source = realworld.dem(mc["bbox"], out / "real" / "dem.npz")
        s, w, n, e = mc["bbox"]
        grid = dem.sample(*np.meshgrid(np.linspace(s, n, 40), np.linspace(w, e, 40)))
        base = float(np.floor(grid.min()))
        elevation = {"source": source, "base_m": base, "min_m": round(float(grid.min()), 1),
                     "max_m": round(float(grid.max()), 1)}
        add_elevation(osm, out / "map_ele.osm", dem, base)
        osm = out / "map_ele.osm"

    types = ["highway." + t for t in mc["road_types"]] + (["highway.cycleway"] if mc["cycleways"] else [])
    common = ["--geometry.remove", "--ramps.guess", "--edges.join", "--junctions.join", "--roundabouts.guess",
              "--tls.guess-signals", "--tls.discard-simple", "--tls.join", "--junctions.corner-detail", "5",
              "--default.lanewidth", str(mc["lane_width"])]
    geo_net = out / "geo.net.xml"
    netconvert("--osm-files", str(osm), *(["--osm.elevation"] if elevation else []),
               "--keep-edges.by-type", ",".join(types), "--remove-edges.isolated",
               "--osm.bike-access", "--proj.utm", *(["--sidewalks.guess"] if mc["sidewalks"] else []),
               *(["--lefthand"] if mc.get("lefthand") else []),
               *common, "-o", str(out / "osm.net.xml"))
    log("OSM -> SUMO net")

    if mc["trim_dead_ends"]:
        keep = out / "keep_edges.txt"
        keep.write_text("\n".join(prune(out / "osm.net.xml")))
        netconvert("--sumo-net-file", str(out / "osm.net.xml"), "--keep-edges.input-file", str(keep),
                   "--keep-edges.components", "1", "-o", str(geo_net))
    else:
        geo_net.write_bytes((out / "osm.net.xml").read_bytes())

    xodr = out / "map.xodr"
    netconvert("--sumo-net-file", str(geo_net), "--opendrive-output", str(xodr))
    name_signals(xodr)
    stats = check_xodr(xodr)

    # vendored (CARLA 1.0 does not ship it); the SUMO net never gets the sidewalks (see config.py)
    subprocess.check_call([sys.executable, str(Path(__file__).parent / "bridge" / "util" / "netconvert_carla.py"),
                           str(xodr), "--output", str(out / "map.net.xml"), "--guess-tls", "--no-sidewalks"])
    log("OpenDRIVE -> CARLA-consistent SUMO net")
    meta = {"name": mc["name"], "lefthand": bool(mc.get("lefthand")), "buildings": 0, "bbox": mc["bbox"],
            "built": time.strftime("%Y-%m-%d %H:%M:%S"),
            "files": {"osm": "map.osm", "geo_net": "geo.net.xml", "xodr": "map.xodr", "sumo_net": "map.net.xml"},
            "xodr": stats, "elevation": elevation}
    sc = mc.get("scenery") or {}
    if sc.get("enabled", True):
        try:
            build_scenery(cfg, out, meta)
        except Exception as err:  # noqa: BLE001 - a map without scenery still drives
            import traceback
            traceback.print_exc()
            log(f"scenery failed ({err}); the map has roads only (redo with --scenery)")
    elif mc.get("buildings"):
        meta["buildings"] = fetch_buildings(mc["bbox"], geo_net, out / "buildings.json")
    (out / "meta.json").write_text(json.dumps(meta, indent=2))
    log(f"done: {out}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("experiment")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--scenery", action="store_true", help="redo only the scenery of a built map")
    args = ap.parse_args()
    build(config.load(args.experiment), args.force, args.scenery)


if __name__ == "__main__":
    main()
