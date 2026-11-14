"""Cyclists where real cyclists ride, at real volumes.

Routes (`traffic.cycling.routes`). The OSM cycle network of the box: the route=bicycle relations (local, regional,
national and international routes, and the Dutch and Belgian knooppunten networks, which OSM tags as rcn/lcn node
networks) and the ways tagged as cycle infrastructure (highway=cycleway, paths with bicycle=designated,
cycleway[:left|right|both]=track|lane, cycle streets). Each is laid on the SUMO net's bike lanes with transit.py's
heading-aware matcher, in both directions (cyclists ride a route both ways), and cached as cycling.json in the map
folder. In a trial `route_share` of the bike trips start and end on that network (a cycle-route edge draws them
`od_weight` times as much as a street, other tracks and lanes half that, and all three times more where they enter
or leave the map, where the routes bring cyclists in), and every cyclist routes with a perceived travel time of
`prefer_route` times the real one on cycle routes and `prefer_infra` on other cycle tracks and lanes, so trips
follow the corridors as cyclists do. The rest are random trips as before (with the same route choice), so side
streets still have their cyclists.

Counts (`traffic.cycling.counts`). Open bicycle counters that need no key, in the box or within NEAR_M of it:
Métropole de Lyon (Eco-Counter and video sensors, ODbL), NYC DOT (Bicycle and Pedestrian Counts), TfL's Active
Travel Counts Programme (London), the Flemish roads agency's counters (AWV, Flanders) and the City of Sydney's
twice-yearly counts. Each counter's typical hourly count for the trial's day type and hour (transit.trial_time) is
compared with the share of simulated trips that pass it (a calibration draw of the same demand), and
bikes_per_hour becomes the median over the counters of count / share. Elsewhere bikes_per_hour stays as configured
(the Netherlands publishes its bike counts, NDW's Fietsdata, only with an account).
"""
import csv
import io
import json
import math
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from collections import defaultdict
from datetime import date, datetime, timedelta
from pathlib import Path

import requests

from . import realworld, transit

NEAR_M = 300            # counters this far outside the box still count when the net reaches them
COUNTER_RADIUS_M = 25   # bike edges this close to a counter (and parallel to the nearest) are the counted section
DAYS = ("weekday", "saturday", "sunday")
BIKE_SPEED = 5.2        # mean free speed of the bike types (maxSpeed 6.5 x speedFactor 0.8), for routing costs
CAL_TRIPS = 2000        # trips in the calibration draw for the counters
REFRESH_DAYS = 30       # counter profiles are fetched again after this long

SOURCES = {
    "osm": ("OpenStreetMap contributors via the Overpass API (cycle routes and infrastructure)", "ODbL 1.0"),
    "lyon": ("Métropole de Lyon, comptages mobilités (sites, channels, measures) via data.grandlyon.com", "ODbL 1.0"),
    "nyc": ("NYC DOT Bicycle and Pedestrian Counts (data.cityofnewyork.us ct66-47at, sensors 6up2-gnw8)",
            "NYC Open Data terms of use"),
    "tfl": ("Transport for London, Active Travel Counts Programme (cycling.data.tfl.gov.uk)",
            "TfL open data terms (OGL v2 based), powered by TfL Open Data"),
    "flanders": ("Agentschap Wegen en Verkeer, fietstellingen (opendata.apps.mow.vlaanderen.be)",
                 "Modellicentie Gratis Hergebruik v1.0"),
    "sydney": ("City of Sydney, twice-yearly bicycle counts (data.cityofsydney.nsw.gov.au)", "CC BY 4.0"),
}

ROUTE_ROLES = {"", "forward", "backward", "main"}


def log(msg, t0=[time.time()]):
    print(f"[cycling    {time.time() - t0[0]:6.1f}s] {msg}", flush=True)


# --- the OSM cycle network -------------------------------------------------------------------------------------

def osm_cycling(bbox, path):
    """Bicycle route relations and cycle-infrastructure ways in the box, geometry clipped to it (cached)."""
    if path.exists():
        return json.loads(path.read_text())
    s, w, n, e = bbox
    b = f"({s},{w},{n},{e})"
    query = f"""[out:json][timeout:180];
relation["type"="route"]["route"~"^(bicycle|mtb)$"]{b};
out geom{b};
(way["highway"="cycleway"]{b};
 way["highway"~"^(path|footway|track|pedestrian)$"]["bicycle"="designated"]{b};
 way["cycleway"~"^(track|lane|opposite_track|opposite_lane)$"]{b};
 way["cycleway:both"~"^(track|lane)$"]{b};
 way["cycleway:left"~"^(track|lane|opposite_track|opposite_lane)$"]{b};
 way["cycleway:right"~"^(track|lane|opposite_track|opposite_lane)$"]{b};
 way["cyclestreet"="yes"]{b};
 way["bicycle_road"="yes"]{b};);
out geom{b};"""
    data = realworld.overpass_json(query, timeout=200)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))
    return data


def _runs(geom):
    """Overpass geometry (points outside the box are null) -> runs of (lat, lon), each followed by [] (a gap)."""
    out, run = [], []
    for p in geom or []:
        if p is None:
            if run:
                out += [run, []]
            run = []
        else:
            run.append((p["lat"], p["lon"]))
    if run:
        out.append(run)
    return out


def infra_kind(tags):
    """What cycle infrastructure an OSM way is: cycleway, track, lane, cyclestreet, or None."""
    hw = tags.get("highway", "")
    if hw == "cycleway" or (hw in ("path", "footway", "track", "pedestrian") and tags.get("bicycle") == "designated"):
        return "cycleway"
    sides = [tags.get(k, "") for k in ("cycleway", "cycleway:both", "cycleway:left", "cycleway:right")]
    if any(v in ("track", "opposite_track") for v in sides):
        return "track"
    if any(v in ("lane", "opposite_lane") for v in sides):
        return "lane"
    if tags.get("cyclestreet") == "yes" or tags.get("bicycle_road") == "yes":
        return "cyclestreet"
    return None


def parse(data):
    """Overpass JSON -> (routes, infra): routes with their tags and track pieces, infra ways with kind and pieces."""
    routes, infra = [], []
    for el in data.get("elements", []):
        tags = el.get("tags", {})
        if el.get("type") == "relation":
            ways = []
            for mem in el.get("members", []):
                if mem.get("type") == "way" and mem.get("role", "") in ROUTE_ROLES:
                    ways += _runs(mem.get("geometry")) + [[]]
            pieces = [p for p in transit._chain(ways) if transit._length(p) >= 50]
            if pieces:
                routes.append({"osm_id": el["id"], "name": tags.get("name", ""),
                               "ref": tags.get("ref") or tags.get("rcn_ref") or tags.get("lcn_ref") or "",
                               "network": tags.get("network", ""), "node_network":
                               tags.get("network:type") == "node_network" or bool(tags.get("rcn_ref")),
                               "pieces": pieces})
        elif el.get("type") == "way":
            kind = infra_kind(tags)
            pieces = [p for p in _runs(el.get("geometry")) if len(p) >= 2 and transit._length(p) >= 15]
            if kind and pieces:
                infra.append({"osm_id": el["id"], "kind": kind, "pieces": pieces})
    return routes, infra


def _match_pieces(pieces, net, min_m):
    """Track pieces -> connected parts (lists of edge ids) on the net's bike lanes, both directions."""
    parts = []
    for piece in pieces:
        xy = [net.convertLonLat2XY(lon, lat) for lat, lon in piece]
        for pts in (xy, xy[::-1]):
            trace = transit._densify(pts, 10.0)
            seq = transit._nearest_edges(trace, net, radius=15.0, vclass="bicycle")
            for part in transit._connected(seq, net, vclass="bicycle"):
                if sum(e.getLength() for e in part) >= min_m:
                    parts.append([e.getID() for e in part])
    return parts


def match(routes, infra, net):
    """Routes and infra ways on the net -> the cycle network: matched routes, route edges, other infra edges."""
    out, route_edges = [], set()
    for r in routes:
        parts = _match_pieces(r["pieces"], net, 100.0)
        if not parts:
            continue
        ids = {e for p in parts for e in p}
        route_edges |= ids
        out.append({k: r[k] for k in ("osm_id", "name", "ref", "network", "node_network")} |
                   {"parts": parts, "length_m": round(sum(net.getEdge(e).getLength() for e in ids))})
    infra_edges, kinds = set(), defaultdict(int)
    for w in infra:
        parts = _match_pieces(w["pieces"], net, 15.0)
        if parts:
            kinds[w["kind"]] += 1
            infra_edges |= {e for p in parts for e in p}
    infra_edges -= route_edges

    def km(ids):
        return round(sum(net.getEdge(e).getLength() for e in ids) / 1000, 2)
    bike = [e for e in net.getEdges() if e.allows("bicycle")]
    nets = defaultdict(int)
    for r in out:
        nets[r["network"] or "?"] += 1
    summary = {"relations": len(routes), "relations_matched": len(out), "networks": dict(nets),
               "node_network_routes": sum(1 for r in out if r["node_network"]),
               "infra_ways": len(infra), "infra_ways_matched": sum(kinds.values()), "infra_kinds": dict(kinds),
               "route_km": km(route_edges), "infra_km": km(infra_edges),
               "bike_km": round(sum(e.getLength() for e in bike) / 1000, 2)}
    return {"routes": out, "route_edges": sorted(route_edges), "infra_edges": sorted(infra_edges),
            "summary": summary}


def network(map_dir, net, bbox):
    """The map's cycle network (cycling.json in the map folder, redone when map.net.xml changes)."""
    map_dir = Path(map_dir)
    st = (map_dir / "map.net.xml").stat()
    stamp = f"{st.st_size}:{int(st.st_mtime)}"
    path = map_dir / "cycling.json"
    if path.exists():
        data = json.loads(path.read_text())
        if data.get("net") == stamp:
            return data
    routes, infra = parse(osm_cycling(bbox, map_dir / "real" / "osm_cycling.json"))
    data = match(routes, infra, net) | {"net": stamp}
    path.write_text(json.dumps(data, indent=1))
    return data


# --- open bicycle counters -------------------------------------------------------------------------------------

def _near(bbox, lat, lon, m=NEAR_M):
    s, w, n, e = bbox
    dlat = m / 111_320
    dlon = m / (111_320 * max(0.1, math.cos(math.radians((s + n) / 2))))
    return s - dlat <= lat <= n + dlat and w - dlon <= lon <= e + dlon


def _in(bbox, s, w, n, e):
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    return s <= lat <= n and w <= lon <= e


def profile(hourly, daytype=transit.day_type):
    """{(day, hour): bikes} of one counter -> ({day type: [mean bikes in that hour, or None] * 24},
    {day type: days}). Days with no bike at all (a broken or covered counter) are left out."""
    per_day = defaultdict(float)
    for (d, _), n in hourly.items():
        per_day[d] += n
    sums = {t: [0.0] * 24 for t in DAYS}
    ns = {t: [0] * 24 for t in DAYS}
    days = {t: set() for t in DAYS}
    for (d, h), n in hourly.items():
        if per_day[d] <= 0:
            continue
        t = daytype(d)
        sums[t][h] += n
        ns[t][h] += 1
        days[t].add(d)
    prof = {t: [round(sums[t][h] / ns[t][h], 1) if ns[t][h] else None for h in range(24)] for t in DAYS}
    return prof, {t: len(days[t]) for t in DAYS}


def typical(site, day, hour):
    """A counter's typical count (bikes per hour) for a day type and hour, or None where it has none."""
    v = (site.get("profile") or {}).get(day, [None] * 24)[hour]
    return v


def _get(url, params=None, timeout=120, stream=False):
    r = requests.get(url, params=params, headers=realworld.UA, timeout=timeout, stream=stream)
    r.raise_for_status()
    return r


def _site(source, sid, name, lat, lon, hourly, kind="section", daytype=transit.day_type, note=""):
    prof, days = profile(hourly, daytype)
    return {"source": source, "id": str(sid), "name": name, "lat": round(lat, 6), "lon": round(lon, 6),
            "kind": kind, "profile": prof, "days": days, "note": note}


LYON_WFS = "https://data.grandlyon.com/geoserver/metropole-de-lyon/ows"
LYON_WS = "https://data.grandlyon.com/fr/datapusher/ws"


def _lyon(bbox, now, days=56):
    feats = _get(LYON_WFS, {"SERVICE": "WFS", "VERSION": "2.0.0", "request": "GetFeature", "outputFormat":
                            "application/json", "SRSNAME": "EPSG:4326",
                            "typename": "metropole-de-lyon:pvo_patrimoine_voirie.pvocomptagesite"}).json()
    near = {}
    for f in feats.get("features", []):
        p = f.get("properties", {})
        try:
            lat, lon = float(p["ylat"]), float(p["xlong"])
        except (KeyError, TypeError, ValueError):
            continue
        if _near(bbox, lat, lon):
            near[str(p["site_id"])] = (p.get("site_name") or str(p["site_id"]), lat, lon)
    if not near:
        return []
    chans = _get(f"{LYON_WS}/rdata/pvo_patrimoine_voirie.pvocomptagechannel/all.json",
                 {"maxfeatures": 10000}).json().get("values", [])
    fresh = (now - timedelta(days=14)).strftime("%Y-%m-%d")
    site_of = {str(c["channel_id"]): str(c["site_id"]) for c in chans
               if str(c.get("site_id")) in near and c.get("mobility_type") == "BIKE"
               and (c.get("last_updated_at") or "")[:10] >= fresh}
    if not site_of:
        return []
    since = (now - timedelta(days=days)).strftime("%Y-%m-%dT00:00:00+01:00")
    rows = _get(f"{LYON_WS}/timeseries/pvo_patrimoine_voirie.pvocomptagemeasure/all.json",
                {"maxfeatures": 500000, "channel_id__in": ",".join(sorted(site_of)),
                 "start_datetime__gte": since}, timeout=300).json().get("values", [])
    hourly = defaultdict(lambda: defaultdict(float))
    for v in rows:
        sid = site_of.get(str(v.get("channel_id")))
        t = str(v.get("start_datetime", ""))
        if sid and len(t) >= 13 and v.get("count") is not None:
            d = date.fromisoformat(t[:10])
            hourly[sid][(d, int(t[11:13]))] += float(v["count"])
    return [_site("lyon", sid, near[sid][0], near[sid][1], near[sid][2], h) for sid, h in hourly.items()]


NYC_SODA = "https://data.cityofnewyork.us/resource"


def _nyc(bbox, now, days=56):
    sensors = _get(f"{NYC_SODA}/6up2-gnw8.json", {"$limit": 5000}).json()
    fresh = (now - timedelta(days=14)).strftime("%Y-%m-%d")
    near = {}
    for s in sensors:
        try:
            lat, lon = float(s["lat"]), float(s["lon"])
        except (KeyError, TypeError, ValueError):
            continue
        if "bike" in (s.get("travelmodes") or "") and (s.get("lastdata") or "")[:10] >= fresh \
                and _near(bbox, lat, lon):
            near[str(s["id"])] = (s.get("name") or str(s["id"]), lat, lon)
    if not near:
        return []
    since = (now - timedelta(days=days)).strftime("%Y-%m-%dT00:00:00")
    ids = ",".join(f"'{i}'" for i in sorted(near))
    rows = _get(f"{NYC_SODA}/ct66-47at.json", {
        "$select": "sensor_id,flowid,direction,date_trunc_ymd(timestamp) as d,date_extract_hh(timestamp) as h,"
                   "sum(counts) as n",
        "$where": f"sensor_id in({ids}) AND travelmode='bike' AND timestamp >= '{since}'",
        "$group": "sensor_id,flowid,direction,d,h", "$limit": 200000}, timeout=300).json()
    # a sensor may report the same direction twice (an old and a new flow pair): per direction the busiest flow
    flows = defaultdict(lambda: defaultdict(float))
    for r in rows:
        flows[(r["sensor_id"], r.get("direction", ""))][r["flowid"]] += float(r.get("n") or 0)
    pick = {k: max(v, key=v.get) for k, v in flows.items()}
    hourly = defaultdict(lambda: defaultdict(float))
    for r in rows:
        if pick[(r["sensor_id"], r.get("direction", ""))] == r["flowid"]:
            hourly[r["sensor_id"]][(date.fromisoformat(r["d"][:10]), int(r["h"]))] += float(r.get("n") or 0)
    return [_site("nyc", sid, near[sid][0], near[sid][1], near[sid][2], h) for sid, h in hourly.items()]


TFL = "https://cycling.data.tfl.gov.uk/ActiveTravelCountsProgramme/"
TFL_LIST = "https://s3-eu-west-1.amazonaws.com/cycling.data.tfl.gov.uk"
TFL_MODES = {"Conventional cycles", "Cargo bikes", "Cycle hire bikes", "Private cycles", "E-bikes",
             "Private e-bikes", "Hire e-bikes"}


def _tfl(bbox, now):
    locs = csv.DictReader(io.StringIO(_get(TFL + "1%20Monitoring%20locations.csv").content.decode("utf-8-sig")))
    near, areas = {}, set()
    for r in locs:
        try:
            lat, lon = float(r["Latitude"]), float(r["Longitude"])
        except (KeyError, TypeError, ValueError):
            continue
        if _near(bbox, lat, lon):
            near[r["Site ID"]] = (r.get("Location description") or r["Site ID"], lat, lon)
            areas.add((r.get("Functional area for monitoring") or "").split(" ", 1)[-1].strip())
    if not near:
        return []
    keys = re.findall(r"<Key>ActiveTravelCountsProgramme/([^<]+\.csv)</Key>",
                      _get(TFL_LIST, {"list-type": "2", "prefix": "ActiveTravelCountsProgramme/"}).text)
    files = []
    for area in sorted(areas):   # the latest wave of each area (Inner comes in parts)
        mine = sorted(k for k in keys if re.match(r"\d{4} W\d", k) and f"-{area}" in k)
        if mine:
            wave = mine[-1].split("-")[0]
            files += [k for k in mine if k.startswith(wave + "-")]
    hourly = defaultdict(lambda: defaultdict(float))
    for f in files:
        r = _get(TFL + requests.utils.quote(f), stream=True, timeout=600)
        lines = (ln.decode("utf-8-sig") for ln in r.iter_lines() if ln)
        for row in csv.DictReader(lines):
            if row.get("SiteID") in near and row.get("Mode") in TFL_MODES:
                try:
                    d = datetime.strptime(row["Date"], "%d/%m/%Y").date()
                    hourly[row["SiteID"]][(d, int(row["Time"][:2]))] += float(row["Count"] or 0)
                except (KeyError, ValueError):
                    continue
    return [_site("tfl", sid, near[sid][0], near[sid][1], near[sid][2], h, note="survey day")
            for sid, h in hourly.items()]


FLANDERS = "https://opendata.apps.mow.vlaanderen.be/fietstellingen/"


def _flanders(bbox, now):
    near = {}
    for r in csv.reader(io.StringIO(_get(FLANDERS + "sites.csv").text)):
        try:
            lon, lat = float(r[2]), float(r[3])
        except (IndexError, ValueError):
            continue
        if _near(bbox, lat, lon):
            near[r[0]] = (r[4] or r[0], lat, lon)
    if not near:
        return []
    first = now.date().replace(day=1)
    months = [(first - timedelta(days=1)).replace(day=1), first]
    hourly = defaultdict(lambda: defaultdict(float))
    for m in months:
        try:
            r = _get(FLANDERS + f"data-{m:%Y-%m}.csv", stream=True, timeout=600)
        except requests.HTTPError:
            continue
        for ln in r.iter_lines():
            row = ln.decode("utf-8", "replace").split(",")
            if len(row) >= 6 and row[0] in near and row[2] == "FIETSERS":
                try:
                    hourly[row[0]][(date.fromisoformat(row[3][:10]), int(row[3][11:13]))] += float(row[5])
                except ValueError:
                    continue
    return [_site("flanders", sid, near[sid][0], near[sid][1], near[sid][2], h) for sid, h in hourly.items()]


SYDNEY = "https://services1.arcgis.com/cNVyNtjGVZybOQWZ/arcgis/rest/services/"


def _sydney(bbox, now, surveys=4):
    sites = _get(SYDNEY + "Twice_Yearly_Bicycle_Count_Locations/FeatureServer/0/query",
                 {"where": "1=1", "outFields": "*", "outSR": 4326, "f": "geojson"}).json()
    near = {}
    for f in sites.get("features", []):
        lon, lat = f["geometry"]["coordinates"][:2]
        if _near(bbox, lat, lon):
            near[int(f["properties"]["SiteID"])] = (f["properties"].get("Intersection") or "", lat, lon)
    if not near:
        return []
    rows = _get(SYDNEY + "Twice_Yearly_Bicycle_Count_Data/FeatureServer/0/query",
                {"where": f"SiteID IN ({','.join(map(str, sorted(near)))})", "outFields": "*",
                 "f": "json", "resultRecordCount": 2000}).json().get("features", [])
    per = defaultdict(list)
    for f in rows:
        a = f["attributes"]
        per[a["SiteID"]].append(a)
    out = []
    for sid, recs in per.items():
        recs.sort(key=lambda a: (a["Year"], a["Month"] == "October"))
        hourly = {}
        for a in recs[-surveys:]:            # the latest surveys, each one weekday in March or October
            for h in (6, 7, 8, 16, 17, 18):
                v = a.get(f"Time_{h:02d}00")
                if v is not None:
                    hourly[((a["Year"], a["Month"]), h)] = float(v)
        out.append(_site("sydney", sid, near[sid][0], near[sid][1], near[sid][2], hourly, kind="junction",
                         daytype=lambda d: "weekday", note="survey day, intersection"))
    return out


# (source, box around its coverage [s, w, n, e], fetcher)
FETCHERS = [
    ("lyon", (45.55, 4.65, 45.95, 5.15), _lyon),
    ("nyc", (40.49, -74.27, 40.92, -73.68), _nyc),
    ("tfl", (51.28, -0.52, 51.70, 0.34), _tfl),
    ("flanders", (50.68, 2.54, 51.51, 5.92), _flanders),
    ("sydney", (-33.93, 151.17, -33.85, 151.24), _sydney),
]


def counters(bbox, real_dir, now=None):
    """Open bicycle counters in or near the box with their hourly profiles (real/bike_counters.json, fetched again
    after REFRESH_DAYS). A source that fails is skipped with one log line; nothing found is an empty list."""
    path = Path(real_dir) / "bike_counters.json"
    if path.exists() and time.time() - path.stat().st_mtime < REFRESH_DAYS * 86400:
        return json.loads(path.read_text())["sites"]
    now = now or datetime.now()
    sites, used, ok = [], [], True
    for name, box, fetch in FETCHERS:
        if not _in(bbox, *box):
            continue
        try:
            got = fetch(bbox, now)
        except Exception as err:  # noqa: BLE001 - a broken service: no counts from it this time
            log(f"bike counters of {name} not reached ({err})")
            ok = False
            continue
        sites += got
        if got:
            used.append({"source": name, "what": SOURCES[name][0], "licence": SOURCES[name][1]})
    if ok:      # a failed fetch is tried again next time
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"fetched": now.isoformat(timespec="minutes"), "sources": used,
                                    "sites": sites}, indent=1))
    return sites


def counter_edges(net, site, radius=COUNTER_RADIUS_M):
    """The bike edges a counter counts: at a section, the bike edges within radius that run parallel to the nearest
    (both directions, a road's lanes and its cycle tracks); at an intersection, the bike edges entering the nearest
    junction. A counter just outside the map counts the street that runs on toward it: the nearest bike edge
    within NEAR_M whose end points at the counter (it lies within radius of that end segment's line), taken where
    it ends. Empty when the net does not reach the counter."""
    x, y = net.convertLonLat2XY(site["lon"], site["lat"])
    near = [(d, e) for e, d in net.getNeighboringEdges(x, y, radius if site.get("kind") != "junction" else 40.0)
            if e.allows("bicycle")]
    if not near and site.get("kind") != "junction":
        ends = []
        for e, d in net.getNeighboringEdges(x, y, NEAR_M):
            sh = e.getShape()
            if not e.allows("bicycle") or len(sh) < 2:
                continue
            for a, b in ((sh[1], sh[0]), (sh[-2], sh[-1])):       # b: the end point, a: the point before it
                dx, dy = b[0] - a[0], b[1] - a[1]
                n = math.hypot(dx, dy)
                ahead = ((x - b[0]) * dx + (y - b[1]) * dy) / n if n > 0 else -1
                if ahead > 0 and abs((x - b[0]) * dy - (y - b[1]) * dx) / n <= radius:
                    ends.append((math.hypot(x - b[0], y - b[1]), b))
        if not ends:
            return []
        x, y = min(ends)[1]
        near = [(d, e) for e, d in net.getNeighboringEdges(x, y, radius) if e.allows("bicycle")]
    if not near:
        return []
    if site.get("kind") == "junction":
        nodes = {n for _, e in near for n in (e.getFromNode(), e.getToNode())}
        node = min(nodes, key=lambda n: math.hypot(n.getCoord()[0] - x, n.getCoord()[1] - y))
        return sorted(e.getID() for e in node.getIncoming() if e.allows("bicycle"))
    _, e0 = min(near, key=lambda t: t[0])
    h0 = transit._heading(e0.getShape(), x, y)
    out = []
    for _, e in near:
        h = transit._heading(e.getShape(), x, y)
        if h and h0 and abs(h[1] * h0[1] + h[2] * h0[2]) >= 0.8:
            out.append(e.getID())
    return sorted(out)


# --- trial demand ----------------------------------------------------------------------------------------------

def _meandata(path, values, attr, end):
    rows = [f'<meandata>\n  <interval begin="0" end="{int(end) + 86400}">']
    rows += [f'    <edge id="{e}" {attr}="{v:.3f}"/>' for e, v in values.items()]
    rows.append("  </interval>\n</meandata>\n")
    Path(path).write_text("\n".join(rows))


def write_costs(net, path, route_edges, infra_edges, prefer_route, prefer_infra, end):
    """duarouter edge costs: a bike edge's perceived travel time, prefer_route times the real one on cycle routes
    and prefer_infra on other cycle infrastructure. The router raises any time below the free-flow time to it, so
    the times are written divided by prefer_route (cycle routes at free flow, everything else slower)."""
    r, i = set(route_edges), set(infra_edges)
    vals = {}
    for e in net.getEdges():
        if e.allows("bicycle"):
            k = prefer_route if e.getID() in r else prefer_infra if e.getID() in i else 1.0
            vals[e.getID()] = e.getLength() / max(0.5, min(e.getSpeed(), BIKE_SPEED)) * k / prefer_route
    _meandata(path, vals, "traveltime", end)


def bike_component(net):
    """For bicycles: (the largest strongly connected part of the net, the edges a trip can start on: those that
    reach it, the edges it can end on: those reached from it), as edge id sets. A trip to or from a cycle track
    the net reaches but cannot leave, or one cut off at the map edge, would fail."""
    def ok(c):
        return c.getFromLane().allows("bicycle") and c.getToLane().allows("bicycle")
    succ, pred = defaultdict(list), defaultdict(list)
    nodes = [e.getID() for e in net.getEdges() if e.allows("bicycle")]
    for e in net.getEdges():
        for to, conns in e.getOutgoing().items():
            if any(ok(c) for c in conns):
                succ[e.getID()].append(to.getID())
                pred[to.getID()].append(e.getID())
    order, seen = [], set()
    for s in nodes:                      # Kosaraju, iteratively: finishing order on the graph ...
        if s in seen:
            continue
        seen.add(s)
        stack = [(s, iter(succ[s]))]
        while stack:
            v, it = stack[-1]
            nxt = next((w for w in it if w not in seen), None)
            if nxt is None:
                order.append(v)
                stack.pop()
            else:
                seen.add(nxt)
                stack.append((nxt, iter(succ[nxt])))
    best, done = set(), set()
    for s in reversed(order):            # ... then components on the reversed graph
        if s in done:
            continue
        comp, stack = set(), [s]
        done.add(s)
        while stack:
            v = stack.pop()
            comp.add(v)
            for w in pred[v]:
                if w not in done:
                    done.add(w)
                    stack.append(w)
        if len(comp) > len(best):
            best = comp

    def reach(graph):
        seen, stack = set(best), list(best)
        while stack:
            for w in graph[stack.pop()]:
                if w not in seen:
                    seen.add(w)
                    stack.append(w)
        return seen
    return best, reach(pred), reach(succ)


def od_weights(net, route_edges, infra_edges, weight, keep=None):
    """randomTrips source and destination weights -> (src, dst) {edge: weight}: as randomTrips weighs edges (each
    the same, three times that at the edge of the map: the fringe factor for bikes; no trip starts on an edge that
    only leaves the map, none ends on one that only enters it), times weight on cycle routes and (1 + weight) / 2
    on other cycle tracks and lanes; only edges in keep (bike_component's starts and ends) when given."""
    r, i = set(route_edges), set(infra_edges)
    src, dst = {}, {}
    for e in net.getEdges():
        if not e.allows("bicycle"):
            continue
        w = weight if e.getID() in r else (1 + weight) / 2 if e.getID() in i else 1.0
        enters, leaves = not e.getIncoming(), not e.getOutgoing()
        if not leaves and (keep is None or e.getID() in keep[0]):
            src[e.getID()] = w * (3 if enters else 1)
        if not enters and (keep is None or e.getID() in keep[1]):
            dst[e.getID()] = w * (3 if leaves else 1)
    return src, dst


def set_inflow(srcs, inflow, rate, max_share=0.5):
    """Counters where bikes enter the map ({edge: bikes per hour}): each of these edges gets count / rate of the
    trip starts of every group (src weights changed in place), so the counted inflow enters there. The counted
    edges take at most max_share of all trips (rate is raised when they would take more); returns the rate."""
    total = sum(inflow.values())
    rate = max(rate, total / max_share)
    share = total / rate
    for src in srcs:
        if not src:
            continue
        rest = sum(w for e, w in src.items() if e not in inflow)
        for e, c in inflow.items():
            src[e] = c / rate * rest / (1 - share)
    return rate


def _write_od(prefix, src, dst, end):
    _meandata(f"{prefix}.src.xml", src, "value", end)
    _meandata(f"{prefix}.dst.xml", dst, "value", end)


def routes_of(files):
    """Route files -> list of (vehicle id, edge list)."""
    out = []
    for f in files:
        for _, el in ET.iterparse(str(f)):
            if el.tag == "vehicle":
                r = el.find("route")
                if r is not None:
                    out.append((el.get("id"), r.get("edges", "").split()))
                el.clear()
    return out


def km_share(routes, net, edges):
    """Share of the bike-kilometres of these routes on the given edges (planned routes, whole length)."""
    s = set(edges)
    tot = on = 0.0
    lengths = {}
    for _, es in routes:
        for e in es:
            ln = lengths.get(e)
            if ln is None:
                ln = lengths[e] = net.getEdge(e).getLength()
            tot += ln
            if e in s:
                on += ln
    return on / tot if tot else 0.0


def calibrate(routes, generated, counted):
    """bikes_per_hour that makes the simulated flow past each counter its real count: counted is a list of
    (edge ids, count per hour); a counter no simulated trip passes is left out. Returns (rate, [per-counter
    rates]) or (None, [])."""
    if not routes or not generated:
        return None, []
    valid = len(routes) / generated
    rates = []
    for edges, count in counted:
        s = set(edges)
        p = sum(1 for _, es in routes if s.intersection(es)) / generated
        rates.append(count / p if p > 0 else None)
    good = sorted(r for r in rates if r)
    if not good:
        return None, rates
    mid = len(good) // 2
    med = good[mid] if len(good) % 2 else (good[mid - 1] + good[mid]) / 2
    return med / valid, rates


def _trips(net_file, work, name, per_hour, end, seed, types, extra=()):
    """One randomTrips call for bikes, as traffic.prepare makes them; returns (route file, trips generated)."""
    import sumo
    rou = Path(work) / f"{name}.rou.xml"
    trips = Path(work) / f"{name}.trips.xml"
    subprocess.check_call(
        [sys.executable, str(Path(sumo.SUMO_HOME) / "tools" / "randomTrips.py"), "-n", str(net_file), "-r",
         str(rou), "-o", str(trips), "-b", "0", "-e", str(end), "-p", f"{3600.0 / per_hour:.4f}", "--seed",
         str(seed), "--vclass", "bicycle", "--prefix", name,
         "--trip-attributes", 'type="bikes" departLane="best" departSpeed="max"',
         "--additional-file", str(types), "--validate", "--fringe-factor", "3", *extra],
        stdout=subprocess.DEVNULL)
    return rou, int(round(end * per_hour / 3600.0))


def merge_routes(files, out):
    """Route files of one vehicle class -> one file (SUMO refuses the vehicle types each of them defines twice),
    vehicles sorted by departure."""
    types, vehicles, seen = [], [], set()
    for f in files:
        for el in ET.parse(str(f)).getroot():
            if el.tag in ("vType", "vTypeDistribution"):
                if el.get("id") not in seen:
                    seen.add(el.get("id"))
                    types.append(el)
            elif el.tag == "vehicle":
                vehicles.append(el)
    vehicles.sort(key=lambda el: float(el.get("depart", 0)))
    root = ET.Element("routes")
    root.extend(types + vehicles)
    ET.indent(root)
    Path(out).write_text(ET.tostring(root, encoding="unicode") + "\n")
    return Path(out)


def _generate(net_file, work, tag, rate, share, end, seed, types, costs, od_random, od_route):
    """Random bike trips (1 - share) and bike trips along the cycle network (share) -> route files, generated.
    od_random/od_route: weights prefixes of the two groups (None: randomTrips' own weights)."""
    files, gen = [], 0
    route_opts = ["--duarouter-weight-files", str(costs)] if costs else []
    for name, r, s, od in ((tag, rate * (1 - share), seed, od_random),
                           (f"{tag}route", rate * share, seed + 7, od_route)):
        if r > 0:
            rou, n = _trips(net_file, work, name, r, end, s, types,
                            route_opts + (["--weights-prefix", str(od)] if od else []))
            files.append(rou)
            gen += n
    return files, gen


def _on(v):
    return v not in (False, None, "off", "false", 0)


def demand(cfg, map_dir, work, types, end, log=log):
    """Bike route files for a trial following traffic.cycling; returns their names (in work/), or None when
    traffic.prepare should make its plain random bike trips (cycling off, or anything failed)."""
    tc = cfg["traffic"]
    cc = tc.get("cycling") or {}
    if not tc.get("bikes_per_hour") or not (_on(cc.get("routes", "auto")) or _on(cc.get("counts", "auto"))):
        return None
    try:
        return _demand(cfg, cc, Path(map_dir), Path(work), types, end, log)
    except Exception as err:  # noqa: BLE001 - the trial still runs, with random bike trips
        log(f"cycling failed ({err}); random bike trips")
        return None


def _demand(cfg, cc, map_dir, work, types, end, log):
    import sumolib
    from . import fleet
    tc = cfg["traffic"]
    net_file = map_dir / "map.net.xml"
    net = sumolib.net.readNet(str(net_file), withInternal=False, withPedestrianConnections=False)
    bbox = json.loads((map_dir / "meta.json").read_text())["bbox"]
    route_edges, infra_edges = [], []
    if _on(cc.get("routes", "auto")):
        try:
            cn = network(map_dir, net, bbox)
            route_edges, infra_edges = cn["route_edges"], cn["infra_edges"]
            s = cn["summary"]
            log(f"cycle network: {s['relations_matched']} of {s['relations']} OSM cycle routes matched "
                f"({s['route_km']} km), {s['infra_ways_matched']} cycle tracks/lanes ({s['infra_km']} km more) "
                f"of {s['bike_km']} km bike lanes")
        except Exception as err:  # noqa: BLE001 - offline: random bike trips, maybe scaled by counts
            log(f"cycle network not available ({err}); random bike trips")
    share = float(cc.get("route_share", 0.7)) if (route_edges or infra_edges) else 0.0
    comp, starts, ends = bike_component(net)
    nbike = sum(1 for e in net.getEdges() if e.allows("bicycle"))
    if len(comp) < nbike / 2:     # weighted trip ends and counted volumes would mostly fail to route
        log(f"the net is fragmented for bikes (its largest strongly connected part has {len(comp)} of {nbike} bike "
            "edges); random bike trips")
        return None
    keep = (starts, ends)
    od_r, od_c = work / "bikerand", work / "bikenet"
    src_r, dst_r = od_weights(net, [], [], 1.0, keep)
    src_c, dst_c = od_weights(net, route_edges, infra_edges, float(cc.get("od_weight", 10)), keep)
    costs = None
    if share > 0:
        costs = work / "bike_costs.xml"
        write_costs(net, costs, route_edges, infra_edges, float(cc.get("prefer_route", 0.5)),
                    float(cc.get("prefer_infra", 0.7)), end)
    rate = float(tc["bikes_per_hour"])
    seed = int(tc["seed"])
    if _on(cc.get("counts", "auto")):
        when = transit.trial_time(cfg["simulation"].get("time"), ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2),
                                  map_dir / "real")
        day, hour = transit.day_type(when), when.hour
        inner, inflow, names = [], {}, []
        for site in counters(bbox, map_dir / "real"):
            c = typical(site, day, hour)
            edges = counter_edges(net, site) if c is not None else []
            if not edges or c <= 0:
                continue
            names.append(f"{site['name']} {c:.0f}/h")
            entering = [e for e in edges if not net.getEdge(e).getIncoming() and e in starts]
            if entering:        # where bikes enter the map: the counted bikes start there (their share of it)
                for e in entering:
                    inflow[e] = inflow.get(e, 0.0) + c / len(edges)
            else:
                inner.append((edges, c))
        new = rate
        if inner:               # inside: total demand so that the simulated flows past the counters fit
            _write_od(od_r, src_r, dst_r, end)
            _write_od(od_c, src_c, dst_c, end)
            files, gen = _generate(net_file, work, "bikecal", CAL_TRIPS * 3600.0 / end, share, end, seed + 101,
                                   types, costs, od_r, od_c)
            fitted, _ = calibrate(routes_of(files), gen, inner)
            for f in files:
                f.unlink(missing_ok=True)
            if fitted:
                new = fitted
        if inflow:
            new = set_inflow([src_r, src_c], inflow, new)
        if names:
            cap = float(cc.get("max_bikes_per_hour", 3000))
            capped = f", capped from {new:.0f}" if new > cap else ""
            new = min(max(new, 30.0), cap)
            log(f"bike counters ({day} {hour:02d}:00): {'; '.join(names)} -> bikes_per_hour {rate:.0f} -> {new:.0f} "
                f"({len(inner)} counted inside, {len(inflow)} entering edges{capped})")
            rate = new
        else:
            log(f"no open bike counter with a count for {day} {hour:02d}:00 in reach; bikes_per_hour {rate:.0f}")
    _write_od(od_r, src_r, dst_r, end)
    _write_od(od_c, src_c, dst_c, end)
    files, _ = _generate(net_file, work, "bike", rate, share, end, seed, types, costs, od_r, od_c)
    rou = merge_routes(files, work / "bike.rou.xml")
    fleet.paint_routes(rou, fleet.Painter({"colors": fleet.BIKE_COLORS}, seed + 1))
    if share:
        rs = routes_of([rou])
        log(f"bikes: {rate:.0f}/h, {100 * share:.0f}% along the cycle network; planned bike-km on cycle routes "
            f"{100 * km_share(rs, net, route_edges):.0f}%, on all cycle infrastructure "
            f"{100 * km_share(rs, net, set(route_edges) | set(infra_edges)):.0f}%")
    return [rou.name]
