"""Build a drivable CARLA + SUMO map for a bounding box from OpenStreetMap.

    python -m coupledcarla.mapbuild experiments/eindhoven_centre.yaml

Steps (all with SUMO's netconvert; carla.Osm2Odr segfaults in 0.9.16 on Linux and Windows):
1. OSM extract from Overpass (cached in the map folder).
2. OSM -> SUMO net (geo.net.xml), keeping the configured road types, cycleways and guessed sidewalks.
3. Pruning: dead-end chains are removed and only the largest strongly connected road graph is kept, so a car
   that follows lanes can always go on (at the cut edge of the box, roads would otherwise end in the void).
4. Pruned net -> OpenDRIVE (map.xodr), validated with carla.Map.
5. OpenDRIVE -> SUMO net with CARLA's netconvert_carla.py (map.net.xml), so SUMO and CARLA share road ids.
geo.net.xml keeps the projection, and CARLA (x, y) = (geo x, -geo y), see geo.py.
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import requests

from . import config

OVERPASS = ["https://overpass-api.de/api/interpreter", "https://overpass.kumi.systems/api/interpreter",
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


def build(cfg, force=False):
    mc = cfg["map"]
    out = config.map_dir(cfg)
    out.mkdir(parents=True, exist_ok=True)
    carla_root = Path(os.environ["CARLA_ROOT"])
    if (out / "meta.json").exists() and not force:
        log(f"{out} already built (use --force to rebuild)")
        return out
    if not mc["bbox"]:
        sys.exit("map.bbox is required")

    osm = out / "map.osm"
    fetch_osm(mc["bbox"], osm)

    types = ["highway." + t for t in mc["road_types"]] + (["highway.cycleway"] if mc["cycleways"] else [])
    common = ["--geometry.remove", "--ramps.guess", "--edges.join", "--junctions.join", "--roundabouts.guess",
              "--tls.guess-signals", "--tls.discard-simple", "--tls.join", "--junctions.corner-detail", "5",
              "--default.lanewidth", str(mc["lane_width"])]
    geo_net = out / "geo.net.xml"
    netconvert("--osm-files", str(osm), "--keep-edges.by-type", ",".join(types), "--remove-edges.isolated",
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

    subprocess.check_call([sys.executable, str(carla_root / "Co-Simulation/Sumo/util/netconvert_carla.py"),
                           str(xodr), "--output", str(out / "map.net.xml"), "--guess-tls"])
    log("OpenDRIVE -> CARLA-consistent SUMO net")

    n_buildings = fetch_buildings(mc["bbox"], geo_net, out / "buildings.json") if mc.get("buildings") else 0

    meta = {"name": mc["name"], "lefthand": bool(mc.get("lefthand")), "buildings": n_buildings, "bbox": mc["bbox"], "built": time.strftime("%Y-%m-%d %H:%M:%S"),
            "files": {"osm": "map.osm", "geo_net": "geo.net.xml", "xodr": "map.xodr", "sumo_net": "map.net.xml"},
            "xodr": stats}
    (out / "meta.json").write_text(json.dumps(meta, indent=2))
    log(f"done: {out}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("experiment")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    build(config.load(args.experiment), args.force)


if __name__ == "__main__":
    main()
