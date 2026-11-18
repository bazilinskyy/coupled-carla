"""SUMO on the part of a city-sized map around the participants (traffic.crop_radius_m).

SUMO's cost per step grows with the vehicles in its whole network: on EhvCity (70 km2) at 20,000 cars/h and
6,000 bikes/h the network fills by ~400 vehicles a minute, and one 0.02 s step costs ~0.021 ms per vehicle with the
sublane model (78 ms at 3,700 vehicles, lin4080), far beyond a real-time tick. Only the vehicles near the
participants reach CARLA (simulation.lod_radius_m), so SUMO runs on the network within ``radius`` of the
participants' starts instead: netconvert keeps the edges in that area (``crop_net``, cached per area next to the
map), and SUMO's cutRoutes.py cuts the trial's whole-map demand to it (``cut_demand``): a trip that crosses the area
enters at the cut edge, at the time it would have reached it (from the full net's lengths and speed limits), and
leaves where its route leaves the area. So the traffic inside follows the same demand as on the whole map.

Edge, lane and traffic-light ids stay those of map.net.xml, and node positions are not shifted (the bridge's offset
is unchanged). Lights of junctions on the cut lose links and get netconvert's default programme; they lie ~radius
from the participants. A participant who drives out of the area has no SUMO lanes there (logged; its SUMO twin
stays at the edge).

"auto" crops maps whose box covers at least AUTO_KM2 (city-scale twins) to AUTO_RADIUS_M; smaller maps run whole,
as before.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import sys
from pathlib import Path

AUTO_KM2 = 25.0        # "auto": crop maps whose box is at least this big
AUTO_RADIUS_M = 1500.0


def box_km2(bbox):
    """Area (km2) of [south, west, north, east]."""
    s, w, n, e = bbox
    lat = math.radians((s + n) / 2.0)
    return (n - s) * 111.32 * (e - w) * 111.32 * math.cos(lat)


def radius_for(setting, bbox):
    """Crop radius (m) for traffic.crop_radius_m: a number, 0/false/None (no crop) or "auto"."""
    if setting in (None, False, 0, "off", "none"):
        return 0.0
    if setting == "auto":
        return AUTO_RADIUS_M if bbox and box_km2(bbox) >= AUTO_KM2 else 0.0
    return float(setting)


def centres(tc):
    """[(lat, lon)] the area is cut around: each participant's spawn, the first point of each agent's path and of
    each participant's route or destination."""
    out = []
    for p in tc.get("participants") or []:
        sp = p.get("spawn") or {}
        if "lat" in sp and "lon" in sp:
            out.append((float(sp["lat"]), float(sp["lon"])))
        for lat, lon in (p.get("route") or [])[:1]:
            out.append((float(lat), float(lon)))
        dest = p.get("destination") or {}
        if "lat" in dest and "lon" in dest:
            out.append((float(dest["lat"]), float(dest["lon"])))
    for a in tc.get("agents") or []:
        path = a.get("path") or []
        if path and len(path[0]) >= 2:
            out.append((float(path[0][0]), float(path[0][1])))
    return out


def area_polygon(points_xy, radius, segments=48):
    """One polygon (net x, y) covering a disc of ``radius`` around each point: the convex hull of the discs
    (netconvert takes one boundary polygon). [] without points."""
    if not points_xy:
        return []
    ring = []
    for x, y in points_xy:
        for k in range(segments):
            a = 2.0 * math.pi * k / segments
            ring.append((x + radius * math.cos(a), y + radius * math.sin(a)))
    return _hull(ring)


def _hull(pts):
    """Convex hull (counter-clockwise, Andrew's monotone chain)."""
    pts = sorted(set((round(x, 2), round(y, 2)) for x, y in pts))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def boundary_arg(poly):
    """netconvert's --keep-edges.in-boundary value: x1,y1,x2,y2,... (closed)."""
    ring = list(poly) + [poly[0]]
    return ",".join(f"{x:.2f},{y:.2f}" for x, y in ring)


def _key(net_file, poly):
    st = Path(net_file).stat()
    h = hashlib.sha1(f"{Path(net_file).resolve()}|{st.st_size}|{int(st.st_mtime)}|{boundary_arg(poly)}".encode())
    return h.hexdigest()[:12]


def crop_net(net_file, centres_ll, radius, cache_dir, log=print):
    """(cropped net path, stats) of map.net.xml within ``radius`` of the (lat, lon) centres, made once per area
    (netconvert, ~1 min for EhvCity) into cache_dir. None when there is no centre."""
    import sumolib
    net_file = Path(net_file)
    if not centres_ll or radius <= 0:
        return None, {}
    net = sumolib.net.readNet(str(net_file), withInternal=False, withPedestrianConnections=False)
    pts = [net.convertLonLat2XY(lon, lat) for lat, lon in centres_ll]
    poly = area_polygon(pts, radius)
    cache_dir = Path(cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)
    key = _key(net_file, poly)
    out = cache_dir / f"crop_{key}.net.xml"
    meta = cache_dir / f"crop_{key}.json"
    if out.exists() and meta.exists():
        stats = json.loads(meta.read_text())
        log(f"SUMO area: {stats['edges']} of {stats['edges_all']} edges within {radius:.0f} m of "
            f"{len(pts)} point(s) (cached {out.name})")
        return out, stats
    tmp = out.with_suffix(".tmp.xml")
    subprocess.check_call([sumolib.checkBinary("netconvert"), "--no-warnings", "--sumo-net-file", str(net_file),
                           "--keep-edges.in-boundary", boundary_arg(poly),
                           "--keep-edges.components", "1",      # no islands cut off by the boundary
                           "--offset.disable-normalization", "true",
                           "--output-file", str(tmp)])
    os.replace(tmp, out)
    small = sumolib.net.readNet(str(out), withInternal=False, withPedestrianConnections=False)
    stats = {"edges": len(small.getEdges()), "edges_all": len(net.getEdges()), "radius_m": radius,
             "centres": [list(c) for c in centres_ll], "tls": len(small.getTrafficLights()),
             "tls_all": len(net.getTrafficLights()),
             "offset_same": [round(v, 2) for v in small.getLocationOffset()] ==
                            [round(v, 2) for v in net.getLocationOffset()]}
    meta.write_text(json.dumps(stats, indent=1))
    log(f"SUMO area: {stats['edges']} of {stats['edges_all']} edges, {stats['tls']} of {stats['tls_all']} traffic "
        f"lights within {radius:.0f} m of {len(pts)} point(s) ({out.name})")
    return out, stats


def _cut_routes_script():
    import sumo
    return Path(sumo.SUMO_HOME) / "tools" / "route" / "cutRoutes.py"


def cut_demand(small_net, full_net, route_files, work, stops=None, log=print):
    """Cuts the route files (names in work/) to the small net with SUMO's cutRoutes.py: one sorted route file
    (work/area.rou.xml) and, with ``stops`` (an additional file of bus stops), work/area_stops.add.xml with the
    stops left. Returns (route file name, stops file name or None, vehicles kept)."""
    import sumo
    work = Path(work)
    out = work / "area.rou.xml"
    cmd = [sys.executable, str(_cut_routes_script()), str(small_net), *[str(work / r) for r in route_files],
           "--orig-net", str(full_net), "--routes-output", str(out),
           "--disconnected-action", "keep", "--min-length", "1",
           "--default.departLane", "best", "--default.departSpeed", "max"]
    stops_out = None
    if stops:
        stops_out = work / "area_stops.add.xml"
        cmd += ["--additional-input", str(work / stops), "--stops-output", str(stops_out)]
    env = dict(os.environ, SUMO_HOME=sumo.SUMO_HOME)
    subprocess.check_call(cmd, cwd=str(work), env=env, stdout=subprocess.DEVNULL)
    n = sum(1 for line in open(out, encoding="utf-8") if "<vehicle " in line)
    return out.name, (stops_out.name if stops_out else None), n


def filter_additional(path, net, out_path):
    """Writes the additional file without elements that name lanes, edges or traffic lights the small net lacks
    (variable speed signs, scramble-crossing programmes, bus stops). A programme of a light whose links the cut
    changed is dropped too (its states would not fit). Returns (kept, dropped)."""
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    lanes = {lane.getID() for e in net.getEdges() for lane in e.getLanes()}
    edges = {e.getID() for e in net.getEdges()}
    links = {t.getID(): 1 + max([c[2] for c in t.getConnections()] or [-1]) for t in net.getTrafficLights()}
    kept = dropped = 0
    for el in list(root):
        ok = True
        if el.tag == "tlLogic":
            phases = el.findall("phase")
            ok = el.get("id") in links and all(len(p.get("state", "")) == links[el.get("id")] for p in phases)
        else:
            if el.get("lane") is not None:
                ok = el.get("lane") in lanes
            if el.get("lanes") is not None:      # a speed sign keeps the lanes the small net still has
                left = [lane for lane in el.get("lanes").split() if lane in lanes]
                ok = ok and bool(left)
                if left:
                    el.set("lanes", " ".join(left))
            if el.get("edge") is not None:
                ok = ok and el.get("edge") in edges
            for child in el:
                if child.get("lane") is not None and child.get("lane") not in lanes:
                    ok = False
        if ok:
            kept += 1
        else:
            root.remove(el)
            dropped += 1
    ET.ElementTree(root).write(out_path, encoding="utf-8", xml_declaration=True)
    return kept, dropped
