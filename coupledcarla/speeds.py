"""Driving style from measured speeds: how fast the city's drivers go relative to the posted limit.

SUMO gives every car a speed factor drawn from a distribution (`speedFactor`): its desired speed is the lane's limit
times that factor. Where open speed measurements cover the map (`traffic.speed_factor: auto`), the distribution's
mean and spread come from them; elsewhere the documented default N(1.0, 0.1) cut to [0.8, 1.2] stays.

Sources (free, no account or key; the sites inside the map box are cached in maps/<name>/real/speeds.json):
- Portland, Oregon: PBOT traffic speed counts. Tube counts in the middle of a block over 24-48 h: the 50th and 85th
  percentile speed of all vehicles and the posted speed. The best kind of data for this: spot speeds of single
  vehicles, mostly in free flow.
- Montevideo: the Intendencia's video detectors (the same detectors as its counts, counts.py): 5-minute mean speed
  per lane at the approaches of the main junctions. 45 and 90 km/h are the detectors' fill values (8 and 2 times as
  frequent as their neighbours, also in intervals without a vehicle) and are dropped.
- Taipei: the Traffic Control Center's vehicle detectors (5-minute mean speed per detector section). Live only:
  every fetch adds the current hour to the cached profile.
- The Netherlands: NDW's travel times of floating-car-data segments (Eindhoven centre has 147). Live only, as
  Taipei. A segment's speed (length / travel time) includes the waits at its junctions, so it is not a desired speed:
  these sites check the simulation (tools/speeds_check.py) but do not set the factor.

The speed factor of a site is its free-flow speed over the limit of the SUMO edge it is matched to (counts.match,
by position, direction and street name): for PBOT the median speed of the count; for detector profiles the median
of the fastest quarter of its hourly speeds (the quiet hours, mostly at night, when nobody holds a driver up). The
map's factor is the median over its spot sites, its spread the median of (85th - 50th percentile) / 1.036 / limit
where the source gives single-vehicle percentiles (PBOT), else the default 0.1. The hourly speeds themselves are not
put into the factor: the slower daytime speeds are congestion, which SUMO makes from the demand.

A SUMO car also slows down at the junctions of its own edge, so with the measured factor the simulated spot speeds
come out below the measured ones (Portland at noon: 0.87 of them). `tools/speeds_check.py --calibrate --write` fits
the factor in SUMO (a secant step from the measured factor to a median simulated / measured ratio of 1, then a check
run) and stores it in data/speed_calibration.json; a map listed there uses that mean (and the measured spread,
scaled with it).

Limits first (`traffic.speed_limits: auto`). Where OSM has no maxspeed on a road, netconvert gives it its own
defaults (osmNetconvert.typ.xml: 100 km/h on primary and secondary roads, 80 on tertiary), which are rural
speeds: 57 secondary edges of Midtown Manhattan and 98 primary and secondary edges around Taipei Main Station
drove at 100 km/h. In a trial these roads get the urban default limit of the country (URBAN_KMH, after the OSM
wiki's "Default speed limits"; 50 km/h, the most common one and WHO's recommended maximum, for countries not in
the table) through a variable speed sign on their lanes (vss.add.xml); the map stays as built. The speed factor
is then relative to these limits.

Nothing open gives accelerations or headways (EIDM's accel, decel, tau), so those stay at their defaults.
Checked: no keyless street speeds for New York (NYC DOT's real-time speeds cover only highways, bridges and
tunnels: FDR, West Side Highway, Lincoln Tunnel), London (TfL's need a key; DfT's average speeds are per local
authority), Lyon (Criter speeds need an account; only multi-km journey times are open), Leuven (the nearest
Flemish loops are on the E314, 1.9 km away), Auckland, Sydney (key), Tokyo, Burlington; Uber Movement is gone.
"""
import csv
import gzip
import io
import json
import math
import re
import statistics
import time
import xml.etree.ElementTree as ET
import zipfile
from datetime import date, datetime
from pathlib import Path

import requests

from . import realworld

CACHE = Path.home() / ".cache" / "coupled-carla" / "speeds"
MAX_AGE_DAYS = 90            # static sources (PBOT, Montevideo) are fetched again after this long
LIVE_REFRESH_H = 3           # live sources (Taipei, NDW) add a new snapshot when the cached one is this old
DEFAULT = {"mean": 1.0, "sd": 0.1, "min": 0.8, "max": 1.2}   # SUMO's normc(mean, sd, min, max)
MIN_SITES = 3                # fewer matched spot sites than this: the default stays
P85_Z = 1.036                # the 85th percentile of a normal distribution lies 1.036 sd above its median

SOURCES = {
    "pbot": ("City of Portland PBOT traffic speed counts (Portland Maps open data, ArcGIS item 914a58eac976)",
             "City of Portland open data"),
    "montevideo": ("Intendencia de Montevideo, velocidad promedio vehicular en las principales avenidas "
                   "(catalogodatos.gub.uy)", "Licencia de Datos Abiertos de Uruguay"),
    "taipei": ("Taipei City Traffic Control Center, vehicle detector sections (GetVD.xml.gz, data.taipei)",
               "Open Government Data License, version 1.0 (Taiwan)"),
    "ndw": ("NDW, travel times of floating-car-data segments (opendata.ndw.nu, traveltime.xml.gz)", "CC0 1.0"),
}
LIVE = {"taipei", "ndw"}
SPOT = {"pbot", "montevideo", "taipei"}     # spot speeds: a desired speed in free flow; NDW: segment speeds
DAYS = ("weekday", "saturday", "sunday")
COMPASS = {"N": 0.0, "NE": 45.0, "E": 90.0, "SE": 135.0, "S": 180.0, "SW": 225.0, "W": 270.0, "NW": 315.0}
MVD_FILL = {45.0, 90.0}


def log(msg, t0=[time.time()]):
    print(f"[speeds     {time.time() - t0[0]:6.1f}s] {msg}", flush=True)


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


def _cached(name, fetch, days=30):
    path = CACHE / f"{name}.json"
    if path.exists() and time.time() - path.stat().st_mtime < days * 86400:
        return json.loads(path.read_text())
    data = fetch()
    CACHE.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))
    return data


def _in_box(lat, lon, bbox, margin_deg=0.002):
    s, w, n, e = bbox
    return s - margin_deg <= lat <= n + margin_deg and w - margin_deg <= lon <= e + margin_deg


def _centre(bbox):
    s, w, n, e = bbox
    return (s + n) / 2, (w + e) / 2


def _day_of(d):
    return DAYS[0] if d.weekday() < 5 else DAYS[1] if d.weekday() == 5 else DAYS[2]


def bearing(lat0, lon0, lat1, lon1):
    """Compass bearing (degrees) from one WGS84 point to another (short distances)."""
    dx = (lon1 - lon0) * math.cos(math.radians((lat0 + lat1) / 2))
    return math.degrees(math.atan2(dx, lat1 - lat0)) % 360.0


def _median(vals):
    vals = [v for v in vals if v is not None]
    return statistics.median(vals) if vals else None


# ---------------------------------------------------------------- fetchers

def _pbot_features():
    url = ("https://hub.arcgis.com/api/v3/datasets/914a58eac9764e9eb8fa5553255d2699_249/downloads/data"
           "?format=geojson&spatialRefId=4326&where=1%3D1")
    return _cached("pbot_speed_counts", lambda: _get(url, timeout=600).json(), days=MAX_AGE_DAYS)


def parse_pbot(data, bbox, newest_year=None):
    """PBOT speed counts in the box: the latest count of each location and direction (within 15 years)."""
    newest_year = newest_year or date.today().year
    latest = {}
    for f in data.get("features", []):
        p, g = f.get("properties") or {}, f.get("geometry")
        bound = (p.get("Bound") or "").upper()
        if not g or not p.get("Spd50th") or not p.get("PostedSpeed") or (p.get("PostedSpeed") or 0) < 15:
            continue
        lon, lat = g["coordinates"][:2]
        if not _in_box(lat, lon, bbox):
            continue
        k = (p.get("LocationID") or p.get("LocationDesc"), bound)
        if k not in latest or (p.get("StartDate") or "") > (latest[k][0].get("StartDate") or ""):
            latest[k] = (p, lat, lon)
    out = []
    mph = 1.609344
    for (loc, bound), (p, lat, lon) in latest.items():
        year = int((p.get("StartDate") or "0")[:4])
        if year < newest_year - 15:
            continue
        road = re.split(r"\s+(?:[NSEW] OF|BETWEEN|AT|@)\s+", p.get("LocationDesc") or "", flags=re.I)[0]
        out.append({"id": f"pbot{loc}{bound}", "lat": lat, "lon": lon, "road": road,
                    "heading": COMPASS.get(bound), "kind": "spot", "year": year,
                    "v50": round(p["Spd50th"] * mph, 1),
                    "v85": round(p["Spd85th"] * mph, 1) if p.get("Spd85th") else None,
                    "limit_kmh": round(p["PostedSpeed"] * mph, 1)})
    return out


def _pbot(bbox, real_dir):
    return parse_pbot(_pbot_features(), bbox)


def parse_montevideo(rows, bbox):
    """5-minute lane speeds (csv rows of the Intendencia's file) -> one site per junction approach with its hourly
    median speed per day type (fill values, blanks, 0 and >= 120 km/h dropped)."""
    lanes, vals = {}, {}
    for r in rows:
        try:
            lat, lon = float(r["latitud"]), float(r["longitud"])
            v = float(r["velocidad"])
        except (TypeError, ValueError, KeyError):
            continue
        if not _in_box(lat, lon, bbox) or v <= 0 or v >= 120 or v in MVD_FILL:
            continue
        appr = (r["dsc_avenida"].strip(), r["dsc_int_anterior"].strip(), r["dsc_int_siguiente"].strip())
        lanes.setdefault(appr, {})[r["cod_detector"]] = (lat, lon)
        try:
            day = _day_of(datetime.strptime(r["fecha"], "%Y-%m-%d").date())
        except ValueError:
            continue
        vals.setdefault(appr, {}).setdefault((day, int(r["hora"][:2])), []).append(v)
    out = []
    for appr, dets in lanes.items():
        hourly = {}
        for (day, h), vs in vals[appr].items():
            if len(vs) >= 6:         # half an hour of lane-intervals at least
                hourly.setdefault(day, [None] * 24)[h] = round(statistics.median(vs), 1)
        if not hourly:
            continue
        pts = list(dets.values())
        out.append({"id": "mvd" + min(dets), "lat": sum(p[0] for p in pts) / len(pts),
                    "lon": sum(p[1] for p in pts) / len(pts), "road": appr[0], "from": appr[1], "to": appr[2],
                    "kind": "spot", "hourly": hourly})
    return out


def _montevideo(bbox, real_dir):
    meta = _get("https://catalogodatos.gub.uy/api/3/action/package_show",
                {"id": "intendencia-montevideo-velocidad-promedio-vehicular-en-las-principales-avenidas-de-montevideo1"}
                ).json()
    res = [r for r in meta["result"]["resources"]
           if (r.get("url") or "").endswith(".zip") and "ckan-data.montevideo" in r["url"]]
    res.sort(key=lambda r: r.get("created") or "", reverse=True)
    out = []
    for r in res[:2]:     # the newest month is partial: take the month before it too, if the newest is thin
        z = zipfile.ZipFile(io.BytesIO(_get(r["url"], timeout=900).content))
        name = next(n for n in z.namelist() if n.lower().endswith(".csv"))
        with z.open(name) as f:
            out = parse_montevideo(csv.DictReader(io.TextIOWrapper(f, encoding="utf-8", errors="replace")), bbox)
        if sum(len([v for d in s["hourly"].values() for v in d if v]) for s in out) >= 24 * max(1, len(out)):
            break
    return out


_VD = "{http://www.iii.org.tw/dax/vd}"


def parse_taipei(xml_bytes, bbox):
    """Taipei's GetVD.xml (detector sections) -> (sites with the current speed, local time of the data)."""
    root = ET.fromstring(xml_bytes)
    t = root.findtext(f"{_VD}ExchangeTime") or ""
    try:
        when = datetime.strptime(t.strip(), "%Y/%m/%dT%H:%M:%S")
    except ValueError:
        when = None
    out = []
    for s in root.iter(f"{_VD}SectionData"):
        def num(k):
            try:
                return float(s.findtext(f"{_VD}{k}"))
            except (TypeError, ValueError):
                return None
        x0, y0, x1, y1 = (num(k) for k in ("StartWgsX", "StartWgsY", "EndWgsX", "EndWgsY"))
        v, vol = num("AvgSpd"), num("TotalVol")
        if None in (x0, y0, x1, y1) or not (_in_box(y0, x0, bbox) or _in_box(y1, x1, bbox)):
            continue
        name = (s.findtext(f"{_VD}SectionName") or "").replace("　", " ")
        road = name.split()[0] if name.split() else name
        out.append({"id": "tpe" + (s.findtext(f"{_VD}SectionId") or name), "lat": (y0 + y1) / 2,
                    "lon": (x0 + x1) / 2, "road": road, "heading": round(bearing(y0, x0, y1, x1), 1),
                    "kind": "spot", "now": v if v and vol and v > 0 and vol > 0 else None})
    return out, when


def _taipei(bbox, real_dir):
    raw = gzip.decompress(_get("https://tcgbusfs.blob.core.windows.net/blobtisv/GetVD.xml.gz").content)
    sites, when = parse_taipei(raw, bbox)
    return sites, when


def parse_ndw_sites(stream, bbox):
    """NDW measurement_current.xml (DATEX II) -> the floating-car-data segments in the box: id, name, length and
    the coordinates of its pieces."""
    out = []
    for _, el in ET.iterparse(stream):
        if not el.tag.endswith("measurementSiteRecord"):
            continue
        kind = name = None
        pts, length = [], 0.0
        for x in el.iter():
            tag = x.tag.rsplit("}", 1)[-1]
            if tag == "measurementEquipmentTypeUsed":
                kind = (x.findtext(".//{*}value") or "").strip()
            elif tag == "measurementSiteName" and name is None:
                name = (x.findtext(".//{*}value") or "").strip()
            elif tag == "lengthAffected":
                length += float(x.text or 0)
            elif tag in ("linearCoordinatesStartPoint", "linearCoordinatesEndPoint"):
                lat, lon = x.findtext(".//{*}latitude"), x.findtext(".//{*}longitude")
                if lat and lon:
                    p = (float(lat), float(lon))
                    if not pts or pts[-1] != p:
                        pts.append(p)
        if kind == "fcd" and len(pts) >= 2 and length > 20 and any(_in_box(a, b, bbox, 0.0) for a, b in pts):
            parts = (name or "").split("_")
            mid = pts[len(pts) // 2] if len(pts) > 2 else ((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2)
            out.append({"id": el.get("id"), "lat": mid[0], "lon": mid[1], "road": parts[0].replace("-", " "),
                        "from": parts[1] if len(parts) > 1 else None, "to": parts[2] if len(parts) > 2 else None,
                        "heading": round(bearing(pts[0][0], pts[0][1], pts[-1][0], pts[-1][1]), 1),
                        "kind": "segment", "length_m": round(length, 1),
                        "points": [[round(a, 6), round(b, 6)] for a, b in pts]})
        el.clear()
    return out


def parse_ndw_times(stream, ids):
    """NDW traveltime.xml -> {site id: travel time in s} for these sites (only real measurements)."""
    out = {}
    for _, el in ET.iterparse(stream):
        if not el.tag.endswith("siteMeasurements"):
            continue
        ref = el.find("{*}measurementSiteReference")
        sid = ref.get("id") if ref is not None else None
        if sid in ids:
            d = el.find(".//{*}basicData/{*}travelTime/{*}duration")
            try:
                v = float(d.text) if d is not None else None
            except (TypeError, ValueError):
                v = None
            if v and v > 0:
                out[sid] = v
        el.clear()
    return out


def _ndw(bbox, real_dir):
    from . import transit
    key = "ndw_sites_" + "_".join(f"{v:.3f}" for v in bbox)
    sites = _cached(key, lambda: parse_ndw_sites(gzip.GzipFile(fileobj=io.BytesIO(
        _get("https://opendata.ndw.nu/measurement_current.xml.gz", timeout=300).content)), bbox), days=30)
    if not sites:
        return [], None
    times = parse_ndw_times(gzip.GzipFile(fileobj=io.BytesIO(
        _get("https://opendata.ndw.nu/traveltime.xml.gz").content)), {s["id"] for s in sites})
    for s in sites:
        tt = times.get(s["id"])
        s["now"] = round(s["length_m"] / tt * 3.6, 1) if tt else None
    return sites, transit.place_now(*_centre(bbox), real_dir)


def _within(lat0, lat1, lon0, lon1):
    return lambda bbox: lat0 <= _centre(bbox)[0] <= lat1 and lon0 <= _centre(bbox)[1] <= lon1


FETCHERS = [   # (source, covers(bbox), fetch(bbox, real dir))
    ("pbot", _within(45.42, 45.66, -122.84, -122.47), _pbot),
    ("montevideo", _within(-34.95, -34.70, -56.45, -56.00), _montevideo),
    ("taipei", _within(24.96, 25.21, 121.45, 121.67), _taipei),
    ("ndw", realworld.in_netherlands, _ndw),
]


# ---------------------------------------------------------------- cache

def merge_live(old_sites, new_sites, when):
    """Adds a live snapshot (each site's "now" speed) to the cached hourly profiles: a running mean per day type
    and hour, with the number of snapshots in "n"."""
    by_id = {s["id"]: s for s in old_sites}
    day, hour = _day_of(when), when.hour
    out = []
    for s in new_sites:
        site = dict(by_id.get(s["id"], {}))
        site.update({k: v for k, v in s.items() if k not in ("now", "hourly", "n")})
        hourly = site.setdefault("hourly", {})
        n = site.setdefault("n", {})
        v = s.get("now")
        if v is not None:
            row = hourly.setdefault(day, [None] * 24)
            cnt = n.setdefault(day, [0] * 24)
            row[hour] = round(((row[hour] or 0.0) * cnt[hour] + v) / (cnt[hour] + 1), 1)
            cnt[hour] += 1
        out.append(site)
    ids = {s["id"] for s in new_sites}
    out += [s for s in old_sites if s["id"] not in ids]
    return out


def load(map_dir, bbox, refresh=False):
    """The open speed sites in and around the map box: {source, what, licence, fetched, sites} or None.

    Cached in maps/<name>/real/speeds.json (an empty answer too); live sources add a snapshot when the cache is
    LIVE_REFRESH_H old. Offline or a failing service: the cache as it is, or None, with one log line."""
    real = Path(map_dir) / "real"
    path = real / "speeds.json"
    data = json.loads(path.read_text()) if path.exists() else None
    if data and not refresh:
        age_h = (datetime.now() - datetime.fromisoformat(data.get("fetched", "2000-01-01"))).total_seconds() / 3600
        if age_h < (LIVE_REFRESH_H if data.get("source") in LIVE else MAX_AGE_DAYS * 24):
            return data if data.get("sites") else None
    for src, covers, fetch in FETCHERS:
        if not covers(bbox):
            continue
        try:
            res = fetch(bbox, real)
            if src in LIVE:
                sites, when = res
                old = (data or {}).get("sites", []) if (data or {}).get("source") == src else []
                sites = merge_live(old, sites, when or datetime.now()) if when else old
            else:
                sites = res
        except Exception as err:  # noqa: BLE001 - offline or the service changed: the cache or the default
            log(f"{src}: speeds not reached ({type(err).__name__}: {err}); "
                + ("the cached speeds are used" if data and data.get("sites") else "default speed factor"))
            return data if data and data.get("sites") else None
        data = {"source": src, "what": SOURCES[src][0], "licence": SOURCES[src][1],
                "fetched": datetime.now().isoformat(timespec="seconds"), "sites": sites}
        real.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=1))
        return data if sites else None
    return None


# ---------------------------------------------------------------- limits of roads OSM gives none

# netconvert's speed (m/s) for OSM roads without maxspeed (SUMO's osmNetconvert.typ.xml); motorways keep theirs
NETCONVERT_DEFAULT = {"trunk": 27.78, "trunk_link": 27.78, "primary": 27.78, "primary_link": 27.78,
                      "secondary": 27.78, "secondary_link": 27.78, "tertiary": 22.22, "tertiary_link": 22.22,
                      "unclassified": 13.89, "residential": 13.89}
# Urban default limits (km/h) where no sign says otherwise, by OSM road type ("*": all), after the OSM wiki's
# "Default speed limits" and the national codes: US cities 25 mph (New York City since 2014, Oregon residence
# districts, Burlington); Brazil's CTB by road function (arterial 60, collector 40, local 30); Argentina avenues
# 60, streets 40; Japan 60 on general roads.
URBAN_KMH = {
    "NL": {"*": 50}, "BE": {"*": 50}, "FR": {"*": 50}, "NC": {"*": 50}, "GB": {"*": 48.3}, "US": {"*": 40.2},
    "TW": {"*": 50}, "JP": {"*": 60}, "AU": {"*": 50}, "NZ": {"*": 50}, "ZA": {"*": 60}, "UY": {"*": 45},
    "BR": {"trunk": 60, "primary": 60, "secondary": 60, "tertiary": 40, "*": 30},
    "AR": {"trunk": 60, "primary": 60, "secondary": 60, "*": 40},
}
GENERIC_URBAN_KMH = 50.0
COUNTRY_CODE = {"Netherlands": "NL", "Belgium": "BE", "France": "FR", "New Caledonia": "NC",
                "United Kingdom": "GB", "USA": "US", "United States": "US", "Taiwan": "TW", "Japan": "JP",
                "Australia": "AU", "New Zealand": "NZ", "South Africa": "ZA", "Uruguay": "UY", "Brazil": "BR",
                "Argentina": "AR", "Egypt": "EG", "Rwanda": "RW", "Laos": "LA"}


def country_of(cfg, bbox):
    """ISO code of the map's country: the city's (experiments/cities.yaml) or, for the Eindhoven experiments, NL."""
    name = (cfg.get("city") or {}).get("country")
    if name:
        return COUNTRY_CODE.get(name)
    return "NL" if realworld.in_netherlands(bbox) else None


def urban_limit(country, osm_type):
    """The urban default limit (km/h) of a country for an OSM road type."""
    t = (osm_type or "").replace("highway.", "")
    table = URBAN_KMH.get(country) or {}
    return float(table.get(t, table.get(t.replace("_link", ""), table.get("*", GENERIC_URBAN_KMH))))


def osm_types(net, geo):
    """{edge id of net: OSM highway type of the geo.net.xml edge it was built from}, found by id (left-hand maps run
    geo.net.xml itself) or by position and direction at the edge's middle (the OpenDRIVE round trip renames them)."""
    from . import counts
    gx, gy = geo.getLocationOffset()
    mx, my = net.getLocationOffset()
    by_id = {g.getID(): g for g in geo.getEdges()}
    osm = [{"id": g.getID(), "shape": [(p[0] - gx + mx, p[1] - gy + my) for p in g.getShape()],
            "type": g.getType(), "speed": g.getSpeed()} for g in geo.getEdges() if g.allows("passenger")]
    grid = counts._Grid(osm)
    out = {}
    for e in net.getEdges():
        if not e.allows("passenger"):
            continue
        g = by_id.get(e.getID())
        if g is not None:
            out[e.getID()] = (g.getType(), g.getSpeed())
            continue
        x, y, head = counts._middle([p[:2] for p in e.getShape()])
        best = None
        for o in grid.near(x, y, 12.0):
            d, h, _ = counts._closest(o["shape"], x, y)
            if d <= 12.0 and counts.angle_diff(h, head) <= 35.0 and (best is None or d < best[0]):
                best = (d, o)
        if best:
            out[e.getID()] = (best[1]["type"], best[1]["speed"])
    return out


def implicit_limits(net, geo, country):
    """{edge id: limit m/s} for the edges whose OSM road has no maxspeed (its speed is netconvert's default for its
    type) and whose country's urban default is lower."""
    out = {}
    for eid, (typ, speed) in osm_types(net, geo).items():
        t = (typ or "").replace("highway.", "")
        if t in NETCONVERT_DEFAULT and abs(speed - NETCONVERT_DEFAULT[t]) < 0.05:
            v = urban_limit(country, t) / 3.6
            if v < net.getEdge(eid).getSpeed() - 0.1:
                out[eid] = round(v, 3)
    return out


def write_vss(path, net, limits):
    """A variable speed sign per limit value over the lanes of those edges, active from t = 0."""
    groups = {}
    for eid, v in sorted(limits.items()):
        groups.setdefault(v, []).extend(ln.getID() for ln in net.getEdge(eid).getLanes())
    rows = ["<additional>"]
    for i, (v, lanes) in enumerate(sorted(groups.items())):
        rows.append(f'  <variableSpeedSign id="urban_limit{i}" lanes="{" ".join(lanes)}">')
        rows.append(f'    <step time="0" speed="{v:g}"/>')
        rows.append("  </variableSpeedSign>")
    rows.append("</additional>")
    Path(path).write_text("\n".join(rows) + "\n")


# ---------------------------------------------------------------- speed factors

def free_flow(site):
    """(free-flow speed in km/h, single-vehicle spread in km/h or None) of a site, or (None, None)."""
    if site.get("v50"):
        spread = (site["v85"] - site["v50"]) / P85_Z if site.get("v85") else None
        return site["v50"], spread
    vals = sorted(v for row in (site.get("hourly") or {}).values() for v in row if v)
    if not vals:
        return None, None
    return statistics.median(vals[int(0.75 * len(vals)):]), None


def measured(site, day, hour):
    """A site's speed (km/h) at that hour of that day type: the hour of the same day type, else of another day
    type, else the count's median (PBOT); None when the site has nothing for that hour."""
    hourly = site.get("hourly") or {}
    for d in (day,) + tuple(x for x in DAYS if x != day):
        if hourly.get(d) and hourly[d][hour] is not None:
            return hourly[d][hour]
    return site.get("v50")


def match_sites(sites, net, edges, limits=None):
    """Each site on the SUMO edge it measures (counts.match) -> [(site, edge id, edge limit km/h, distance m)];
    limits: {edge id: m/s} that replace the net's (implicit_limits)."""
    from . import counts
    limits = limits or {}
    ms = counts.match(sites, edges, lambda lon, lat: net.convertLonLat2XY(lon, lat))
    out = []
    for site, parts, d in ms:
        eid = max(parts, key=lambda p: p[1])[0]
        out.append((site, eid, limits.get(eid, net.getEdge(eid).getSpeed()) * 3.6, d))
    return out


def fit(matched, source):
    """The speed factor distribution from matched sites: {mean, sd, min, max, sites, ...} or None (too few)."""
    if source not in SPOT:
        return None
    ratios, spreads = [], []
    for site, _, limit, _ in matched:
        v, spread = free_flow(site)
        if v is None or limit < 5:
            continue
        ratios.append(v / limit)
        if spread is not None:
            spreads.append(spread / limit)
    if len(ratios) < MIN_SITES:
        return None
    mean = statistics.median(ratios)
    sd = statistics.median(spreads) if spreads else DEFAULT["sd"]
    q = statistics.quantiles(ratios, n=4) if len(ratios) >= 4 else (min(ratios), mean, max(ratios))
    return {"mean": round(mean, 3), "sd": round(sd, 3),
            "min": round(max(0.4, mean - 2.5 * sd), 2), "max": round(min(2.0, mean + 2.5 * sd), 2),
            "sites": len(ratios), "iqr": [round(q[0], 3), round(q[-1], 3)]}


def by_class(matched, edges):
    """Median free-flow ratio per OSM road class (counts.road_class) of the matched edges, with the site count."""
    cls = {e["id"]: e.get("cls", "minor") for e in edges}
    per = {}
    for site, eid, limit, _ in matched:
        v, _ = free_flow(site)
        if v is not None and limit >= 5:
            per.setdefault(cls.get(eid, "minor"), []).append(v / limit)
    return {k: (round(statistics.median(v), 3), len(v)) for k, v in sorted(per.items())}


CALIBRATION = Path(__file__).resolve().parent / "data" / "speed_calibration.json"


def calibration(map_name, source):
    """The speed factor tools/speeds_check.py --calibrate --write fitted in SUMO for this map and source, or None."""
    try:
        cal = json.loads(CALIBRATION.read_text()).get(map_name)
    except (OSError, ValueError):
        return None
    return cal if cal and cal.get("source") == source else None


def distribution(f):
    return f"normc({f['mean']:g},{f['sd']:g},{f['min']:g},{f['max']:g})"


def plan(cfg, map_dir, work, bbox, log=log):
    """traffic.speed_limits and traffic.speed_factor for a trial -> (SUMO's speedFactor for the cars, the name of the
    variable speed sign file written into work/ or None). Never fails: the defaults on any error."""
    tc = cfg["traffic"]
    default = distribution(DEFAULT)
    want = tc.get("speed_factor", "auto")
    factor = str(want) if want not in ("auto", None, False, "off") else default
    vss, limits, net, geo_net = None, {}, None, None
    map_dir = Path(map_dir)
    if tc.get("speed_limits", "auto") == "auto" and (map_dir / "geo.net.xml").exists():
        try:
            import sumolib
            net = sumolib.net.readNet(str(map_dir / "map.net.xml"), withInternal=False,
                                      withPedestrianConnections=False)
            geo_net = sumolib.net.readNet(str(map_dir / "geo.net.xml"), withInternal=False,
                                          withPedestrianConnections=False)
            country = country_of(cfg, bbox)
            limits = implicit_limits(net, geo_net, country)
            if limits:
                Path(work).mkdir(parents=True, exist_ok=True)
                write_vss(Path(work) / "vss.add.xml", net, limits)
                vss = "vss.add.xml"
                kmh = sorted({round(v * 3.6) for v in limits.values()})
                log(f"speed limits: {len(limits)} roads without an OSM maxspeed (netconvert's 80-100 km/h) get the "
                    f"urban default of {country or 'most countries'}: {', '.join(map(str, kmh))} km/h")
        except Exception as err:  # noqa: BLE001
            log(f"speed limits: left as built ({type(err).__name__}: {err})")
            limits = {}
    if want != "auto":
        return factor, vss
    try:
        data = load(map_dir, bbox)
        if not data:
            return factor, vss
        from . import counts
        net2, edges = counts.net_edges(map_dir)
        matched = match_sites(data["sites"], net2, edges, limits)
        f = fit(matched, data["source"])
        cal = calibration(cfg["map"].get("name") or map_dir.name, data["source"])
        if cal and not f:
            log(f"speeds: {data['what']}: {len(matched)} of {len(data['sites'])} sites matched; speed factor fitted in "
                f"SUMO to the measured speeds (tools/speeds_check.py, {cal['when'][:10]}, MAPE "
                f"{cal['mape'].get('limits')} -> {cal['mape'].get('calibrated')}%) -> speedFactor {distribution(cal)}")
            return distribution(cal), vss
        if not f:
            log(f"speeds: {data['what']}: {len(matched)} of {len(data['sites'])} sites matched"
                + ("" if data["source"] in SPOT else " (segment speeds: a check, not a factor)")
                + f"; default speed factor {default}")
            return factor, vss
        cls = ", ".join(f"{k} {v[0]:.2f} ({v[1]})" for k, v in by_class(matched, edges).items())
        msg = (f"speeds: {data['what']}: {f['sites']} of {len(data['sites'])} sites matched; free-flow speed / limit "
               f"median {f['mean']:.2f} (IQR {f['iqr'][0]:.2f}-{f['iqr'][1]:.2f}; by road class {cls})")
        if cal:
            log(f"{msg}; fitted in SUMO to the measured speeds (tools/speeds_check.py, {cal['when'][:10]}, MAPE "
                f"{cal['mape'].get('limits')} -> {cal['mape'].get('calibrated')}%) -> speedFactor {distribution(cal)}")
            return distribution(cal), vss
        log(f"{msg} -> speedFactor {distribution(f)}")
        return distribution(f), vss
    except Exception as err:  # noqa: BLE001 - a refinement: never fail the trial for it
        log(f"speeds: not used ({type(err).__name__}: {err}); default speed factor {default}")
        return factor, vss
