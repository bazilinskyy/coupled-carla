"""Measured traffic counts: car volumes that match the real counts where open counts exist near the map, and a
road-network estimate of car demand elsewhere.

Where open counts cover the map (`traffic.counts: auto`), each counter is mapped to the SUMO edge it counts (by
position, travel direction and road name), its count is turned into the hourly flow of the trial's local hour and
day type, and SUMO's routeSampler picks car routes out of many random candidates so that the counted edges carry
those flows. routeSampler then tops the demand up to the map's total with routes that pass no counter, so the
streets without counters keep their traffic.

Sources (free, no account or key; the counters inside the map box are cached in maps/<name>/real/counts.json):
- New York City: NYC DOT Automated Traffic Volume Counts (15-minute counts per direction, several days per site).
- Great Britain: DfT road traffic statistics, annual average daily flow by direction (AADF) of each count point.
- Lyon: Métropole de Lyon Criter loops (average working-day flow of a reference week).
- Montevideo: Intendencia de Montevideo video detectors (5-minute counts per lane, the latest month).
- Auckland: Auckland Transport traffic counts (average daily traffic, two-way, with the share of cars).
- Portland, Oregon: PBOT traffic volume counts (24-48 h counts per direction, ADT).
- Vermont: VTrans AADT stations.

Counts that only give a daily total become hourly flows with a typical urban profile of the share of a day's traffic
in each hour (PROFILE, weekday, Saturday and Sunday) and day factors (DAY_FACTOR): the shape of DfT's traffic
distribution by time of day (table TRA0307, cars, Great Britain), checked against the hourly NYC and Montevideo
counts (see the README). Counts of all motor vehicles are scaled by LIGHT_SHARE to the cars and vans that SUMO's
car fleet is (buses run as their own lines).

Elsewhere (`traffic.cars_per_hour: auto`) the car demand follows the road network: the lane-kilometres of each road
class times a typical daily flow per lane of that class (ADT_PER_LANE, the median over the counters matched on the
maps above) gives the vehicle-kilometres in the trial's hour, and dividing by the mean length of a random trip on
the map gives the cars per hour.
"""
import csv
import io
import json
import math
import re
import subprocess
import sys
import time
import unicodedata
import zipfile
from datetime import date, datetime
from pathlib import Path

import requests

from . import realworld

CACHE = Path.home() / ".cache" / "coupled-carla" / "counts"
MAX_AGE_DAYS = 90            # refetch the counters of a map after this long

SOURCES = {
    "nyc_dot": ("NYC DOT Automated Traffic Volume Counts (NYC Open Data, 7ym2-wayt)", "NYC Open Data terms of use"),
    "dft": ("Department for Transport road traffic statistics, AADF by direction (roadtraffic.dft.gov.uk)",
            "Open Government Licence v3.0"),
    "grandlyon": ("Métropole de Lyon, comptages Criter (data.grandlyon.com)", "Licence Ouverte 2.0"),
    "montevideo": ("Intendencia de Montevideo, conteo vehicular en las principales avenidas (catalogodatos.gub.uy)",
                   "Licencia de Datos Abiertos de Uruguay"),
    "auckland": ("Auckland Transport traffic counts (TrafficService)", "CC BY 4.0"),
    "portland": ("City of Portland PBOT traffic volume counts (Portland Maps open data)", "City of Portland open data"),
    "vtrans": ("VTrans historic AADT stations (Vermont Agency of Transportation)", "VTrans open data"),
}

DAYS = ("weekday", "saturday", "sunday")
# Share of the day's motor traffic in each hour (percent; normalised below), after DfT TRA0307 (cars, GB).
_PROFILE_PCT = {
    "weekday": [0.7, 0.4, 0.3, 0.3, 0.5, 1.1, 2.8, 6.0, 7.1, 5.6, 5.3, 5.6,
                5.8, 5.9, 6.1, 6.9, 7.5, 7.6, 6.0, 4.3, 3.2, 2.6, 2.0, 1.3],
    "saturday": [1.5, 1.0, 0.7, 0.5, 0.5, 0.8, 1.5, 2.6, 4.0, 5.4, 6.6, 7.4,
                 7.6, 7.4, 7.1, 6.9, 6.7, 6.3, 5.5, 4.4, 3.4, 2.9, 2.5, 2.0],
    "sunday": [2.0, 1.3, 0.9, 0.6, 0.5, 0.6, 1.0, 1.6, 2.7, 4.2, 6.0, 7.3,
               7.9, 7.9, 7.6, 7.4, 7.2, 6.7, 5.9, 4.8, 3.8, 3.0, 2.2, 1.3],
}
PROFILE = {d: [v / sum(p) for v in p] for d, p in _PROFILE_PCT.items()}
# a day's traffic relative to the annual average day (5 weekdays + Saturday + Sunday = 7)
DAY_FACTOR = {"weekday": 1.05, "saturday": 0.95, "sunday": 0.80}
LIGHT_SHARE = 0.9            # cars and vans in counts of all motor vehicles: about nine in ten on urban roads;
                             # 0.77 on the 42 DfT counts in central Westminster (many buses and motorcycles)
# Typical annual average daily flow of cars and vans per lane by OSM road class, for the network estimate: the
# medians over 1,080 counter-edge pairs matched on seven maps (NYC, London, Lyon, Montevideo, Auckland, Portland,
# Burlington; primary 3,371, secondary 2,437, tertiary 2,982, minor 1,056; secondary and tertiary pooled).
ADT_PER_LANE = {"primary": 3400, "secondary": 2600, "tertiary": 2600, "minor": 1000}
AUTO_MAX_PER_HOUR = 5000     # random trips crowd onto a few junctions: at 6,000 cars/h Taipei teleported 66 and Tokyo
                             # 29 vehicles in 300 s (12,000 and 10,600 uncapped: 114 and 38); fitted routes spread
                             # better and are not capped
TRIP_KM_PER_BOX_KM = 0.72    # mean random-trip length / side of the map box (randomTrips, fringe factor 5; 0.61 to
                             # 0.86 on eight maps)

COMPASS = {"N": 0.0, "NB": 0.0, "NE": 45.0, "E": 90.0, "EB": 90.0, "SE": 135.0, "S": 180.0, "SB": 180.0,
           "SW": 225.0, "W": 270.0, "WB": 270.0, "NW": 315.0}


def log(msg, t0=[time.time()]):
    print(f"[counts {time.time() - t0[0]:6.1f}s] {msg}", flush=True)


# ---------------------------------------------------------------- time of day

def day_weight(day, hour):
    """Share of an annual average day's traffic that falls in this hour of this day type."""
    return DAY_FACTOR[day] * PROFILE[day][hour]


def hourly_flow(c, day, hour):
    """A counter's flow of cars and vans in vehicles/hour at that hour of that day type, or None.

    Hourly counts of the same day type are used as they are; hourly counts of another day type are carried over
    with the profile; a daily total (annual average "aadt" or working-day "weekday") is spread with the profile."""
    v = None
    hourly = c.get("hourly") or {}
    if hourly.get(day) and hourly[day][hour] is not None:
        v = hourly[day][hour]
    else:
        for d in DAYS:
            if hourly.get(d) and hourly[d][hour] is not None:
                v = hourly[d][hour] * day_weight(day, hour) / day_weight(d, hour)
                break
    if v is None and c.get("daily"):
        aadt = c["daily"] / DAY_FACTOR["weekday"] if c.get("daily_kind") == "weekday" else c["daily"]
        v = aadt * day_weight(day, hour)
    return None if v is None else v * c.get("light_share", 1.0)


# ---------------------------------------------------------------- names and headings

_GENERIC = set("""street st avenue ave av avda avenida road rd boulevard blvd bd bv bvd drive dr lane ln place pl
square sq way the of de del la le les du des d l rue quai cours pont pt calle rua west east north south w e n s nw ne
sw se y and et highway hwy plaza circus terrace crescent gardens court ct parkway pkwy""".split())


def name_tokens(name):
    """The distinctive words of a street name: lower case, no accents, no generic words, "5th" -> "5"."""
    if not name:
        return set()
    s = unicodedata.normalize("NFKD", str(name)).encode("ascii", "ignore").decode().lower()
    s = re.sub(r"(\d+)(st|nd|rd|th|e|er|eme)\b", r"\1", s)
    return {t for t in re.split(r"[^a-z0-9]+", s) if t and t not in _GENERIC}


def name_score(a, b):
    """0..1: how much two street names share (the shorter one's words found in the longer one)."""
    ta, tb = name_tokens(a), name_tokens(b)
    if not ta or not tb:
        return 0.0
    return len(ta & tb) / min(len(ta), len(tb))


def bearing(x0, y0, x1, y1):
    """Compass bearing (degrees, 0 = north, clockwise) from (x0, y0) to (x1, y1) in a north-up metric frame."""
    return math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0


def angle_diff(a, b):
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def _closest(shape, x, y):
    """(distance, heading of the segment, point) of the point on a polyline nearest to (x, y)."""
    best = (float("inf"), 0.0, None)
    for (x0, y0), (x1, y1) in zip(shape, shape[1:]):
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / L2))
        px, py = x0 + t * dx, y0 + t * dy
        d = math.hypot(x - px, y - py)
        if d < best[0] and L2 > 0:
            best = (d, bearing(x0, y0, x1, y1), (px, py))
    return best


class _Grid:
    """Edges bucketed by the cells their shapes pass, for neighbour queries."""

    def __init__(self, edges, cell=50.0):
        self.cell, self.cells = cell, {}
        for e in edges:
            seen = set()
            for (x0, y0), (x1, y1) in zip(e["shape"], e["shape"][1:]):
                n = max(1, int(math.hypot(x1 - x0, y1 - y0) / cell) + 1)
                for i in range(n + 1):
                    k = (int((x0 + (x1 - x0) * i / n) // cell), int((y0 + (y1 - y0) * i / n) // cell))
                    if k not in seen:
                        seen.add(k)
                        self.cells.setdefault(k, []).append(e)

    def near(self, x, y, r):
        out, ids = [], set()
        c = self.cell
        for i in range(int((x - r) // c), int((x + r) // c) + 1):
            for j in range(int((y - r) // c), int((y + r) // c) + 1):
                for e in self.cells.get((i, j), ()):
                    if e["id"] not in ids:
                        ids.add(e["id"])
                        out.append(e)
        return out


# ---------------------------------------------------------------- counters to edges

def _desired_heading(c, x, y, grid, max_dist=400.0):
    """The counter's direction of travel: given, or from the cross streets it counts from and to."""
    if c.get("heading") is not None:
        return c["heading"]
    for key, towards in (("to", True), ("from", False)):
        if not c.get(key):
            continue
        best = None
        for e in grid.near(x, y, max_dist):
            if e.get("name") and name_score(e["name"], c[key]) >= 0.5 and name_score(e["name"], c.get("road")) < 1:
                d, _, p = _closest(e["shape"], x, y)
                if d <= max_dist and d > 5.0 and (best is None or d < best[0]):
                    best = (d, p)
        if best:
            px, py = best[1]
            return bearing(x, y, px, py) if towards else bearing(px, py, x, y)
    return None


def match(counters, edges, xy, max_dist=40.0, max_angle=60.0):
    """Maps each counter onto the SUMO edge(s) it counts.

    counters: dicts with lat, lon, road (name), heading (compass degrees of travel) or from/to (cross street names),
    edges: dicts with id, shape [(x, y)] in the net's metric frame, name (OSM, may be None), lanes (car lanes),
    xy: (lon, lat) -> (x, y). Returns [(counter, [(edge id, share)], distance m)] for the matched counters: a
    directed counter maps to one edge; an undirected (two-way) count is split evenly over the two directions,
    or goes whole to a one-way street."""
    grid = _Grid(edges)
    out = []
    for c in counters:
        x, y = xy(c["lon"], c["lat"])
        want = _desired_heading(c, x, y, grid)
        cands = []
        for e in grid.near(x, y, max_dist):
            d, head, _ = _closest(e["shape"], x, y)
            if d > max_dist:
                continue
            s = _road_score(c, e)
            if s == 0 and d > (5.0 if c.get("ref") and e.get("ref") else 15.0):
                continue                                 # another street, not just a nameless stretch
            if want is not None and angle_diff(head, want) > max_angle:
                continue
            cost = d if s is None else d + 15.0 * (1.0 - s) - 10.0 * s   # a matching name outweighs 25 m
            cands.append((cost, d, head, s, e))
        if not cands:
            continue
        cands.sort(key=lambda k: k[0])
        cost, d, head, s, e = cands[0]
        if want is not None:
            out.append((c, [(e["id"], 1.0)], d))
            continue
        back = [k for k in cands[1:] if angle_diff(k[2], head) >= 120.0 and (k[3] or 0) >= (s or 0) - 1e-9
                and k[1] <= d + 15.0]
        if back:
            out.append((c, [(e["id"], 0.5), (back[0][4]["id"], 0.5)], d))
        else:
            out.append((c, [(e["id"], 1.0)], d))
    return out


def _road_score(c, e):
    """How well an edge's OSM name/ref fits the counter's road name/ref: 0..1, None when nothing to compare."""
    scores = []
    if c.get("road") and e.get("name"):
        scores.append(name_score(e["name"], c["road"]))
    if c.get("ref"):
        refs = {r.strip().upper() for r in (e.get("ref") or "").split(";") if r.strip()}
        if refs or e.get("name"):                        # a named street without the ref is another road
            scores.append(1.0 if c["ref"].upper() in refs else 0.0)
    return max(scores) if scores else None


def edge_flows(matches, day, hour):
    """{edge id: hourly flow} for the trial hour. Several counters on one edge: the mean of the most recent year."""
    per = {}
    for c, parts, _ in matches:
        f = hourly_flow(c, day, hour)
        if f is None:
            continue
        for eid, share in parts:
            per.setdefault(eid, []).append((c.get("year") or 0, f * share))
    out = {}
    for eid, vals in per.items():
        latest = max(y for y, _ in vals)
        sel = [f for y, f in vals if y == latest]
        out[eid] = sum(sel) / len(sel)
    return out


def write_edgedata(path, flows, begin, end):
    """routeSampler's count file: one interval with each counted edge's expected vehicles in it."""
    rows = ["<data>", f'  <interval id="counts" begin="{begin:g}" end="{end:g}">']
    for eid, f in sorted(flows.items()):
        n = int(round(f * (end - begin) / 3600.0))
        rows.append(f'    <edge id="{_x(eid)}" entered="{n}"/>')
    rows += ["  </interval>", "</data>"]
    Path(path).write_text("\n".join(rows) + "\n")


def geh(m, c):
    """The GEH statistic of a modelled vs counted hourly flow (under 5 is a good fit)."""
    return 0.0 if m + c == 0 else math.sqrt(2.0 * (m - c) ** 2 / (m + c))


def _x(s):
    return str(s).replace("&", "&amp;").replace('"', "&quot;").replace("<", "&lt;")


# ---------------------------------------------------------------- fetchers

def _get(url, params=None, timeout=120, tries=3, **kw):
    for i in range(tries):
        try:
            r = requests.get(url, params=params, headers=realworld.UA, timeout=timeout, **kw)
            r.raise_for_status()
            return r
        except (requests.ConnectionError, requests.Timeout):
            if i == tries - 1:
                raise
            time.sleep(5 * (i + 1))


def _in_box(lat, lon, bbox, margin_deg=0.002):
    s, w, n, e = bbox
    return s - margin_deg <= lat <= n + margin_deg and w - margin_deg <= lon <= e + margin_deg


def _centre(bbox):
    s, w, n, e = bbox
    return (s + n) / 2, (w + e) / 2


def _day_of(d):
    return DAYS[0] if d.weekday() < 5 else DAYS[1] if d.weekday() == 5 else DAYS[2]


def _hourly_means(sums):
    """{(day, hour): [values]} -> {day: [24 means or None]}"""
    out = {}
    for day in DAYS:
        row = [None] * 24
        for h in range(24):
            v = sums.get((day, h))
            if v:
                row[h] = round(sum(v) / len(v), 1)
        if any(x is not None for x in row):
            out[day] = row
    return out


def _nyc(bbox):
    from pyproj import Transformer
    url = "https://data.cityofnewyork.us/resource/7ym2-wayt.json"
    segs = _get(url, {"$select": "segmentid,wktgeom,street,fromst,tost,direction,max(yr) as yr",
                      "$group": "segmentid,wktgeom,street,fromst,tost,direction", "$limit": 50000}, 300).json()
    to_ll = Transformer.from_crs(2263, 4326, always_xy=True)
    keys = {}
    for r in segs:
        m = re.match(r"POINT \(([-\d.]+) ([-\d.]+)\)", r.get("wktgeom") or "")
        if not m or r.get("direction") not in COMPASS:
            continue
        lon, lat = to_ll.transform(float(m.group(1)), float(m.group(2)))
        if _in_box(lat, lon, bbox):
            k = (r["segmentid"], r["direction"])
            keys.setdefault(k, {"lats": [], "lons": [], "street": r.get("street"), "from": r.get("fromst"),
                                "to": r.get("tost")})
            keys[k]["lats"].append(lat)
            keys[k]["lons"].append(lon)
    if not keys:
        return []
    ids = ",".join(f"'{s}'" for s in sorted({k[0] for k in keys}))
    rows = _get(url, {"$select": "segmentid,direction,yr,m,d,hh,sum(vol) as v,count(*) as n",
                      "$where": f"segmentid in ({ids})", "$group": "segmentid,direction,yr,m,d,hh",
                      "$limit": 200000}, 300).json()
    newest = max(int(r["yr"]) for r in rows) if rows else 0
    sums, years = {}, {}
    for r in rows:
        k = (r["segmentid"], r["direction"])
        if k not in keys or int(r["n"]) != 4 or int(r["yr"]) < newest - 10:   # whole hours of the last 10 years
            continue
        day = _day_of(date(int(r["yr"]), int(r["m"]), int(r["d"])))
        sums.setdefault(k, {}).setdefault((day, int(r["hh"])), []).append(float(r["v"]))
        years[k] = max(years.get(k, 0), int(r["yr"]))
    out = []
    for k, v in keys.items():
        if k not in sums:
            continue
        street = re.sub(r"^[NSEW]/B\s+", "", v["street"] or "").split(" @ ")[0]
        out.append({"id": f"{k[0]}{k[1]}", "lat": sum(v["lats"]) / len(v["lats"]),
                    "lon": sum(v["lons"]) / len(v["lons"]), "road": street, "heading": COMPASS[k[1]],
                    "from": v["from"], "to": v["to"], "hourly": _hourly_means(sums[k]), "year": years[k],
                    "light_share": LIGHT_SHARE})
    return out


_DFT = "https://roadtraffic.dft.gov.uk/api"
_DFT_REGIONS = {1: (50.9, -3.5), 2: (52.9, -0.9), 3: (56.8, -4.2), 4: (52.3, -3.8), 5: (54.0, -2.6),
                6: (51.507, -0.128), 7: (52.2, 0.4), 8: (53.9, -1.3), 9: (51.3, -0.6), 10: (52.5, -2.0),
                11: (55.0, -1.8)}


def _dft_pages(path, params):
    out, page = [], 1
    while True:
        d = _get(f"{_DFT}/{path}", {**params, "page[size]": 250, "page[number]": page}).json()
        out += d["data"]
        if page >= d.get("last_page", 1):
            return out
        page += 1


def _dft(bbox):
    lat, lon = _centre(bbox)
    regions = sorted(_DFT_REGIONS, key=lambda r: math.hypot(lat - _DFT_REGIONS[r][0],
                                                            (lon - _DFT_REGIONS[r][1]) * 0.6))
    points = []
    for rid in regions[:2]:                     # the nearest region, and the next one for boxes on a border
        pts = _cached(f"dft_points_region{rid}", lambda: _dft_pages("count-points", {"filter[region_id]": rid}))
        points += [p for p in pts if _in_box(float(p["latitude"]), float(p["longitude"]), bbox, 0.0)
                   and int(p.get("aadf_year") or 0) >= date.today().year - 11]
        if points:
            break
    out = []
    for p in points:
        rows = _dft_pages("average-annual-daily-flow-by-direction", {"filter[count_point_id]": p["count_point_id"]})
        if not rows:
            continue
        year = max(r["year"] for r in rows)
        for r in rows:
            if r["year"] != year or r["direction_of_travel"] not in COMPASS:
                continue
            light = (r.get("cars_and_taxis") or 0) + (r.get("lgvs") or 0)
            out.append({"id": f"{p['count_point_id']}{r['direction_of_travel']}", "lat": float(p["latitude"]),
                        "lon": float(p["longitude"]), "road": None, "ref": p["road_name"],
                        "heading": COMPASS[r["direction_of_travel"]], "daily": light, "daily_kind": "aadt",
                        "all_motor": r.get("all_motor_vehicles"), "year": year,
                        "estimated": r.get("estimation_method") != "Counted"})
    return out


def _lyon(bbox):
    s, w, n, e = bbox
    url = "https://data.grandlyon.com/geoserver/metropole-de-lyon/ows"
    d = _get(url, {"SERVICE": "WFS", "VERSION": "2.0.0", "request": "GetFeature", "outputFormat": "application/json",
                   "typename": "metropole-de-lyon:pvo_patrimoine_voirie.pvocomptagecriter", "SRSNAME": "EPSG:4326",
                   "bbox": f"{s},{w},{n},{e},urn:ogc:def:crs:EPSG::4326"}).json()
    sites = {}
    for f in d.get("features", []):
        p = f["properties"]
        if "tous" not in (p.get("typepostemesure") or "") or not p.get("moyennejoursouvrable"):
            continue
        parts = (p.get("nom") or "").split("_")
        if len(parts) < 4 or re.search(r"\s(T[àa]?\s?[GD])$", parts[-1]):
            continue                                 # turning-movement lanes count part of an approach
        lon, lat = f["geometry"]["coordinates"][:2]
        k = parts[0]
        site = sites.setdefault(k, {"id": f"criter{k}", "lats": [], "lons": [], "road": parts[1],
                                    "from": parts[2], "to": "_".join(parts[3:]),
                                    "daily": p["moyennejoursouvrable"], "daily_kind": "weekday",
                                    "peak": (p.get("debithorairemax"), p.get("horairedebitmax")),
                                    "year": p.get("anneereference"), "light_share": LIGHT_SHARE})
        site["lats"].append(lat)
        site["lons"].append(lon)
    out = []
    for site in sites.values():
        lats, lons = site.pop("lats"), site.pop("lons")
        out.append({**site, "lat": sum(lats) / len(lats), "lon": sum(lons) / len(lons)})
    return out


def _montevideo(bbox):
    meta = _get("https://catalogodatos.gub.uy/api/3/action/package_show",
                {"id": "intendencia-montevideo-conteo-vehicular-en-las-principales-avenidas-de-montevideo"}).json()
    res = [r for r in meta["result"]["resources"]
           if (r.get("url") or "").endswith(".zip") and "ckan-data.montevideo" in r["url"]]
    res.sort(key=lambda r: r.get("created") or "", reverse=True)
    if not res:
        return []
    raw = _get(res[0]["url"], timeout=600).content
    z = zipfile.ZipFile(io.BytesIO(raw))
    name = next(n for n in z.namelist() if n.lower().endswith(".csv"))
    lanes, five = {}, {}
    with z.open(name) as f:
        for r in csv.DictReader(io.TextIOWrapper(f, encoding="utf-8", errors="replace")):
            try:
                lat, lon = float(r["latitud"]), float(r["longitud"])
            except (TypeError, ValueError):
                continue
            if not _in_box(lat, lon, bbox):
                continue
            appr = (r["dsc_avenida"].strip(), r["dsc_int_anterior"].strip(), r["dsc_int_siguiente"].strip())
            lanes.setdefault(appr, {})[r["cod_detector"]] = (lat, lon)
            k = (appr, r["cod_detector"], r["fecha"], int(r["hora"][:2]))
            n, v = five.get(k, (0, 0))
            five[k] = (n + 1, v + int(float(r["volumen_hora"] or 0)))
    # whole hours (12 five-minute counts) of every lane of an approach, summed over its lanes
    hours = {}
    for (appr, det, day, h), (n, v) in five.items():
        if n == 12:
            hours.setdefault((appr, day, h), {})[det] = v
    sums = {}
    for (appr, day, h), per in hours.items():
        if len(per) == len(lanes[appr]):
            d = _day_of(datetime.strptime(day, "%Y-%m-%d").date())
            sums.setdefault(appr, {}).setdefault((d, h), []).append(sum(per.values()))
    out = []
    year = int(res[0].get("created", "0000")[:4] or 0)
    for appr, dets in lanes.items():
        if appr not in sums:
            continue
        pts = list(dets.values())
        out.append({"id": "mvd" + min(dets), "lat": sum(p[0] for p in pts) / len(pts),
                    "lon": sum(p[1] for p in pts) / len(pts), "road": appr[0], "from": appr[1], "to": appr[2],
                    "lanes": len(dets), "hourly": _hourly_means(sums[appr]), "year": year,
                    "light_share": LIGHT_SHARE})
    return out


def _arcgis(url, bbox, margin=0.002):
    s, w, n, e = bbox
    d = _get(url, {"where": "1=1", "geometry": f"{w - margin},{s - margin},{e + margin},{n + margin}",
                   "geometryType": "esriGeometryEnvelope", "inSR": 4326, "spatialRel": "esriSpatialRelIntersects",
                   "outFields": "*", "outSR": 4326, "f": "json"}).json()
    return d.get("features", [])


def _auckland(bbox):
    url = "https://services2.arcgis.com/JkPEgZJGxhSjYOo0/arcgis/rest/services/TrafficService/FeatureServer/0/query"
    out = []
    for f in _arcgis(url, bbox):
        a, g = f["attributes"], f.get("geometry") or {}
        if a.get("latest") != "Yes" or not a.get("adt") or "x" not in g:
            continue
        year = datetime.utcfromtimestamp(a["count_date"] / 1000).year if a.get("count_date") else 0
        out.append({"id": f"at{a['OBJECTID']}", "lat": g["y"], "lon": g["x"], "road": a.get("road_name"),
                    "daily": a["adt"], "daily_kind": "aadt", "year": year,
                    "light_share": (a["pccar"] + (a.get("pclcv") or 0)) / 100.0 if a.get("pccar") else LIGHT_SHARE})
    return out


def _portland(bbox):
    url = ("https://hub.arcgis.com/api/v3/datasets/0064041a68ae40adae798bfbd9aaaeeb_250/downloads/data"
           "?format=geojson&spatialRefId=4326&where=1%3D1")
    data = _cached("portland_counts", lambda: _get(url, timeout=600).json(), days=MAX_AGE_DAYS)
    latest = {}
    for f in data.get("features", []):
        p, g = f["properties"], f.get("geometry")
        bound = (p.get("Bound") or "").upper()
        if not g or bound not in COMPASS or not p.get("ADTVolume"):
            continue
        lon, lat = g["coordinates"][:2]
        if not _in_box(lat, lon, bbox):
            continue
        k = (p.get("LocationID") or p.get("LocationDesc"), bound)
        if k not in latest or (p.get("StartDate") or "") > (latest[k][0].get("StartDate") or ""):
            latest[k] = (p, lat, lon)
    out = []
    for (loc, bound), (p, lat, lon) in latest.items():
        year = int((p.get("StartDate") or "0")[:4])
        if year < date.today().year - 15:
            continue
        road = re.split(r"\s+(?:[NSEW] OF|BETWEEN|AT|@)\s+", p.get("LocationDesc") or "")[0]
        out.append({"id": f"pbot{loc}{bound}", "lat": lat, "lon": lon, "road": road, "heading": COMPASS[bound],
                    "daily": p["ADTVolume"], "daily_kind": "weekday" if "Weekday" in (p.get("ExceptType") or "")
                    else "aadt", "year": year, "light_share": LIGHT_SHARE})
    return out


def _vtrans(bbox):
    url = "https://services1.arcgis.com/NXmBVyW5TaiCXqFs/arcgis/rest/services/HistoricAADT/FeatureServer/15/query"
    latest = {}
    for f in _arcgis(url, bbox):
        a = f["attributes"]
        if not a.get("AADT") or a.get("LATITUDE") is None:
            continue
        k = a.get("LOCAL_ID")
        actual = "Actual" in (a.get("DESCRIPTION") or "")
        rank = (actual, a.get("AADT_YEAR") or 0)
        if k not in latest or rank > latest[k][0]:
            latest[k] = (rank, a)
    return [{"id": f"vt{k}", "lat": a["LATITUDE"], "lon": a["LONGITUDE"], "road": a.get("ON_ROAD"),
             "daily": a["AADT"], "daily_kind": "aadt", "year": a.get("AADT_YEAR"), "light_share": LIGHT_SHARE,
             "estimated": not rank[0]} for k, (rank, a) in latest.items()]


def _within(lat0, lat1, lon0, lon1):
    return lambda bbox: lat0 <= _centre(bbox)[0] <= lat1 and lon0 <= _centre(bbox)[1] <= lon1


FETCHERS = [   # (source, covers(bbox), fetch(bbox))
    ("nyc_dot", _within(40.49, 40.92, -74.27, -73.68), _nyc),
    ("dft", lambda b: _within(49.9, 60.9, -8.2, 1.8)(b) and not _within(51.4, 55.4, -10.6, -5.9)(b), _dft),
    ("grandlyon", _within(45.55, 45.95, 4.65, 5.15), _lyon),
    ("montevideo", _within(-34.95, -34.70, -56.45, -56.00), _montevideo),
    ("auckland", _within(-37.35, -36.10, 174.40, 175.40), _auckland),
    ("portland", _within(45.42, 45.66, -122.84, -122.47), _portland),
    ("vtrans", _within(42.72, 45.02, -73.44, -71.46), _vtrans),
]


def _cached(name, fetch, days=30):
    path = CACHE / f"{name}.json"
    if path.exists() and time.time() - path.stat().st_mtime < days * 86400:
        return json.loads(path.read_text())
    data = fetch()
    CACHE.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))
    return data


def load(map_dir, bbox, refresh=False):
    """The open counters in and around the map box: {source, licence, fetched, counters} or None.

    Cached in maps/<name>/real/counts.json (also an empty answer, so maps without counts do not ask again);
    offline or a failing service: None, with one log line."""
    path = Path(map_dir) / "real" / "counts.json"
    if path.exists() and not refresh:
        data = json.loads(path.read_text())
        age = (datetime.now() - datetime.fromisoformat(data.get("fetched", "2000-01-01"))).days
        if age < MAX_AGE_DAYS:
            return data if data.get("counters") else None
    for src, covers, fetch in FETCHERS:
        if not covers(bbox):
            continue
        try:
            counters = fetch(bbox)
        except Exception as err:  # noqa: BLE001 - offline or the service changed: random demand
            log(f"{src}: counts not reached ({type(err).__name__}: {err}); random car demand is used")
            return None
        data = {"source": src, "what": SOURCES[src][0], "licence": SOURCES[src][1],
                "fetched": datetime.now().isoformat(timespec="seconds"), "counters": counters}
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=1))
        return data if counters else None
    return None


# ---------------------------------------------------------------- the SUMO net

def road_class(osm_type):
    t = (osm_type or "").replace("highway.", "").replace("_link", "")
    return "primary" if t in ("motorway", "trunk", "primary") else t if t in ("secondary", "tertiary") else "minor"


def net_edges(map_dir):
    """The car edges of map.net.xml (CARLA's road ids) with the OSM name and road class of the geo.net.xml edge
    they were built from (found by position and direction, as the OpenDRIVE round trip drops the names)."""
    import sumolib
    map_dir = Path(map_dir)
    net = sumolib.net.readNet(str(map_dir / "map.net.xml"), withInternal=False, withPedestrianConnections=False)
    edges = []
    for e in net.getEdges():
        lanes = sum(1 for ln in e.getLanes() if ln.allows("passenger"))
        if lanes:
            edges.append({"id": e.getID(), "shape": [p[:2] for p in e.getShape()], "lanes": lanes,
                          "length": e.getLength(), "name": None, "cls": "minor"})
    geo_path = map_dir / "geo.net.xml"
    if geo_path.exists():
        geo = sumolib.net.readNet(str(geo_path), withInternal=False, withPedestrianConnections=False)
        (mx, my), (gx, gy) = net.getLocationOffset(), geo.getLocationOffset()
        tags = osm_names(map_dir / "map.osm")
        osm = []
        for g in geo.getEdges():
            if g.allows("passenger"):
                name, ref = tags.get(re.sub(r"^-?(\d+).*", r"\1", g.getID()), (None, None))
                osm.append({"id": g.getID(), "shape": [(p[0] - gx + mx, p[1] - gy + my) for p in g.getShape()],
                            "name": g.getName() or name, "ref": ref, "cls": road_class(g.getType())})
        label(edges, osm)
    return net, edges


def osm_names(path):
    """{OSM way id: (name, ref)} of the roads in map.osm (netconvert keeps the way id in the edge id, not the
    name)."""
    import xml.etree.ElementTree as ET
    out = {}
    if not Path(path).exists():
        return out
    for _, el in ET.iterparse(str(path)):
        if el.tag == "way":
            tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
            if tags.get("highway") and (tags.get("name") or tags.get("ref")):
                out[el.get("id")] = (tags.get("name"), tags.get("ref"))
            el.clear()
        elif el.tag == "node":
            el.clear()
    return out


def label(edges, osm, max_dist=12.0, max_angle=35.0):
    """Gives each edge the name and class of the nearest OSM edge running the same way at its middle."""
    grid = _Grid(osm)
    for e in edges:
        x, y, head = _middle(e["shape"])
        best = None
        for o in grid.near(x, y, max_dist):
            d, h, _ = _closest(o["shape"], x, y)
            if d <= max_dist and angle_diff(h, head) <= max_angle and (best is None or d < best[0]):
                best = (d, o)
        if best:
            e["name"], e["ref"], e["cls"] = best[1]["name"], best[1].get("ref"), best[1]["cls"]


def _middle(shape):
    segs = [(math.hypot(x1 - x0, y1 - y0), (x0, y0, x1, y1)) for (x0, y0), (x1, y1) in zip(shape, shape[1:])]
    half = sum(L for L, _ in segs) / 2.0
    for L, (x0, y0, x1, y1) in segs:
        if L >= half and L > 0:
            t = half / L
            return x0 + t * (x1 - x0), y0 + t * (y1 - y0), bearing(x0, y0, x1, y1)
        half -= L
    (x0, y0), (x1, y1) = shape[0], shape[-1]
    return x1, y1, bearing(x0, y0, x1, y1)


# ---------------------------------------------------------------- demand

def lane_km(edges):
    out = {k: 0.0 for k in ADT_PER_LANE}
    for e in edges:
        out[e.get("cls", "minor")] += e["lanes"] * e["length"] / 1000.0
    return out


def auto_cars_per_hour(edges, bbox, day, hour):
    """Cars entering the map per hour so that its roads carry typical flows for their class at this hour:
    vehicle-km per hour (lane-km x daily flow per lane x the hour's share) / mean trip length."""
    vkt = sum(km * ADT_PER_LANE[k] for k, km in lane_km(edges).items()) * day_weight(day, hour)
    s, w, n, e = bbox
    side_km = math.sqrt(abs(n - s) * 111.2 * abs(e - w) * 111.2 * math.cos(math.radians((s + n) / 2)))
    return min(AUTO_MAX_PER_HOUR, vkt / (TRIP_KM_PER_BOX_KM * side_km))


def car_routes(net_file, work, types, end, rate, flows, seed, log=print):
    """Cars fitted to the counted edges: random candidate routes, routeSampler picks those that reproduce the
    counted flows in [0, end] and fills up to `rate` cars/hour with routes that pass no counter.
    Returns the route file, or None (no SUMO tools, routeSampler failed: the caller falls back)."""
    try:
        return _car_routes(net_file, work, types, end, rate, flows, seed, log)
    except Exception as err:  # noqa: BLE001
        log(f"counts: routeSampler not used ({type(err).__name__}: {err}); random car trips")
        return None


def _car_routes(net_file, work, types, end, rate, flows, seed, log):
    import sumo
    tools = Path(sumo.SUMO_HOME) / "tools"
    counted = sum(flows.values())
    cand_rate = max(rate, counted) * 8.0          # plenty of candidates through every counter
    cand = work / "car_candidates.rou.xml"
    subprocess.check_call(
        [sys.executable, str(tools / "randomTrips.py"), "-n", str(net_file), "-r", str(cand),
         "-o", str(work / "car_candidates.trips.xml"), "-b", "0", "-e", str(end), "-p", f"{3600.0 / cand_rate:.4f}",
         "--seed", str(seed), "--vclass", "passenger", "--prefix", "cand", "--fringe-factor", "5",
         "--trip-attributes", 'type="cars" departLane="best" departSpeed="max"', "--additional-file", str(types),
         "--validate"], stdout=subprocess.DEVNULL)
    data = work / "counts.edgedata.xml"
    write_edgedata(data, flows, 0, end)
    rou = work / "car.rou.xml"
    total = int(round(rate * end / 3600.0))

    def sample(extra):
        out = subprocess.run(
            [sys.executable, str(tools / "routeSampler.py"), "-r", str(cand), "-d", str(data), "-o", str(rou),
             "-b", "0", "-e", str(end), "--seed", str(seed), "--prefix", "car", "--keep-attributes",
             "--mismatch-output", str(work / "counts.mismatch.xml"), "-v", *extra],
            capture_output=True, text=True)
        if out.returncode != 0 or not rou.exists():
            raise RuntimeError((out.stderr.strip().splitlines() or [f"exit {out.returncode}"])[-1])
        return out.stdout, len(re.findall(r"<vehicle ", rou.read_text()))
    # the counts alone first; then, if they need fewer cars than the map's total, again with routes that pass no
    # counter filling up to it (--total-count caps the sampled cars, so it cannot be given in the first pass)
    stdout, n = sample([])
    fitted = n
    if n < total:
        stdout, n = sample(["--total-count", str(total)])
    _copy_vtypes(cand, rou)
    rou.write_text(rou.read_text().replace('<vehicle id="cand', '<vehicle id="car'))
    m = re.search(r"GEH<\S+ for ([\d.]+)%", stdout)
    log(f"routeSampler: {n} cars in {end} s ({fitted} fitted to the counts, {n - fitted} elsewhere)"
        + (f", GEH ok on {m.group(1)}% of the counted edges" if m else ""))
    return rou


def _copy_vtypes(src, dst):
    """routeSampler writes the vehicles only: put the vehicle types they use (duarouter wrote them into the
    candidates) in front."""
    types = []
    for t in re.findall(r"<vType\b.*?(?:/>|</vType>)", Path(src).read_text(), re.S):
        if t not in types:
            types.append(t)
    d = Path(dst).read_text()
    if types and "<vType" not in d:
        d = re.sub(r"(<routes[^>]*>)", lambda g: g.group(1) + "\n    " + "\n    ".join(types), d, count=1)
        Path(dst).write_text(d)


def plan(cfg_traffic, map_dir, bbox, when, log=print):
    """What traffic.prepare needs for the cars: (cars per hour, edge flows or None, edges, summary)."""
    from .transit import day_type
    day, hour = day_type(when), when.hour
    mode = cfg_traffic.get("counts", "auto")
    per_hour = cfg_traffic.get("cars_per_hour", "auto")
    if mode in (False, None, "off") and per_hour != "auto":
        return float(per_hour), None, None, None
    net, edges = net_edges(map_dir)
    if per_hour == "auto":
        per_hour = auto_cars_per_hour(edges, bbox, day, hour)
        km = lane_km(edges)
        log(f"cars: {per_hour:.0f}/h from the road network ({day} {hour:02d}:00; lane-km primary {km['primary']:.1f}, "
            f"secondary {km['secondary']:.1f}, tertiary {km['tertiary']:.1f}, minor {km['minor']:.1f})")
    per_hour = float(per_hour)
    if mode in (False, None, "off"):
        return per_hour, None, edges, None
    data = load(map_dir, bbox)
    if not data:
        return per_hour, None, edges, None
    matches = match(data["counters"], edges, lambda lon, lat: net.convertLonLat2XY(lon, lat))
    flows = edge_flows(matches, day, hour)
    summary = {"source": data["source"], "counters": len(data["counters"]), "matched": len(matches),
               "edges": len(flows), "day": day, "hour": hour}
    log(f"counts: {data['what']}: {len(matches)} of {len(data['counters'])} counters matched to {len(flows)} edges "
        f"({day} {hour:02d}:00, {sum(flows.values()):.0f} veh/h on them)")
    return per_hour, (flows or None), edges, summary
