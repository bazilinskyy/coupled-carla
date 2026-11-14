"""Pedestrians that behave like the real pedestrians of the map's country and are where real pedestrians are.

The crowd (traffic.walkers) is calibrated with CROWD, City Road Observations With Dashcams (crowd-dataset/crowd:
YouTube dashcam driving in 238 countries, with YOLO tracks; data CC BY 4.0), whose numbers per country and city are
in data/crowd_pedestrians.json (built by tools/crowd_table.py, which lists the versions used):

  how many      walkers per km of kerb in proportion to CROWD's pedestrians per minute of driving footage in the
                place (city, else country, else continent), relative to the world rate; at night the place's
                night/day ratio
  how fast      walking speeds drawn from crowd-city's median crossing speed of the place and its quartiles (m/s)
  how often     crossings per km walked in proportion to the share of the tracked pedestrians that crossed in front
  they cross    of the dashcam (crowd-city), relative to the world share
  where they    at a marked crossing (OSM highway=crossing) or a junction, or mid-block with the country's share of
  cross         crossings without a marked crosswalk or pedestrian light (crowd-jaywalking; elsewhere modelled from
                CROWD's pedestrians per minute, see data/crowd_pedestrians.json "jaywalk_model")
  gap           CROWD measures no gap acceptance: a walker steps out when the next vehicle needs at least the HCM
  acceptance    critical headway t_c = crossing width / walking speed + 2 s (Highway Capacity Manual, two-way stop
                control, pedestrian critical headway), shrinking with waiting (impatience, to 60 % after 60 s); at a
                marked crossing also when every approaching vehicle can still stop
  where they    walkers start and walk where Overture Maps places (shops, food, education, culture, transit
  are           stations) and OSM bus stops are dense, and near open pedestrian counts where a city publishes them
                keyless (NYC DOT bi-annual counts, City of Sydney walking counts)

Everything but the CARLA side (CrowdSim) is plain Python on the SUMO net (geo.net.xml, whose x, y are CARLA's
x, -y), so a crowd can be planned and measured without a server (tools/crowdpeds_check.py).

traffic.walkers: {count, radius_m, seed, behaviour: auto | off, places: auto, walkers_per_km, max_count,
                  crossings_per_km, speed_mps: auto, jaywalk_share: auto}
"""
import bisect as _bisect
import json
import math
import random
import time
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

DATA = Path(__file__).resolve().parent / "data" / "crowd_pedestrians.json"
SOURCES = {
    "crowd": ("CROWD: City Road Observations With Dashcams (crowd-dataset/crowd), with crowd-city crossing speeds "
              "and crowd-jaywalking crossing checks; Alam, Bazilinska & Bazilinskyy (2026), arXiv:2604.01044",
              "CC BY 4.0"),
    "overture_places": ("Overture Maps Foundation places (Meta, Microsoft, Foursquare, ...)", "CDLA Permissive 2.0"),
    "nyc_ped_counts": ("NYC DOT Bi-Annual Pedestrian Counts (NYC Open Data cqsj-cfgu)", "NYC Open Data terms of use"),
    "sydney_walking_counts": ("City of Sydney walking count surveys", "CC BY 4.0"),
}
OVERTURE_PLACES = "s3://overturemaps-us-west-2/release/{rel}/theme=places/type=place/*"
NYC_PED = "https://data.cityofnewyork.us/resource/cqsj-cfgu.json"
SYD = "https://services1.arcgis.com/cNVyNtjGVZybOQWZ/arcgis/rest/services"
NOMINATIM = "https://nominatim.openstreetmap.org/reverse"

FACTOR_MAX = 4.0                     # crossing and density factors are clipped to [1/4, 4] x the world's
HCM_START_S = 2.0                    # HCM pedestrian start-up and clearance time in t_c = L / S_p + t_s
DEFAULTS = {"behaviour": "auto", "places": "auto", "walkers_per_km": 8.0, "max_count": 80,
            "crossings_per_km": 2.0, "speed_mps": "auto", "jaywalk_share": "auto"}
# place weights by Overture's top category (taxonomy.hierarchy[0]); stations and stops are found by name
PLACE_WEIGHT = {"food_and_drink": 3.0, "shopping": 2.0, "arts_and_entertainment": 2.0, "cultural_and_historic": 2.0,
                "education": 2.0, "community_and_government": 1.5, "lodging": 1.5, "sports_and_recreation": 1.0,
                "lifestyle_services": 1.0, "health_care": 1.0, "travel_and_transportation": 1.0,
                "services_and_business": 0.5, "geographic_entities": 0.0, None: 0.5}
STATION_WORDS = ("train_station", "subway_station", "metro_station", "railway_station", "light_rail", "tram_station",
                 "bus_station", "transportation_hub", "ferry_terminal")


def _log(msg):
    print(f"[crowdpeds] {msg}", flush=True)


# --- the CROWD table and the map's place --------------------------------------------------------------------------

@lru_cache(maxsize=1)
def table():
    return json.loads(DATA.read_text(encoding="utf-8"))


ALIASES = {"usa": "USA", "us": "USA", "united states of america": "USA", "america": "USA", "uk": "GBR",
           "great britain": "GBR", "england": "GBR", "scotland": "GBR", "wales": "GBR", "holland": "NLD",
           "the netherlands": "NLD", "korea": "KOR", "republic of korea": "KOR", "czech republic": "CZE",
           "turkey": "TUR", "ivory coast": "CIV", "russian federation": "RUS", "viet nam": "VNM"}


def country_code(name):
    """ISO3 of a country name, ISO2 or ISO3 code (as in the experiment's city.country), else None."""
    if not name:
        return None
    key = str(name).strip().lower()
    if key in ALIASES:
        return ALIASES[key]
    for iso3, c in table()["countries"].items():
        if key in (iso3.lower(), (c.get("iso2") or "").lower(), c["name"].lower()):
            return iso3
    return None


def _km(lat1, lon1, lat2, lon2):
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return 6371.0 * math.hypot(x, math.radians(lat2 - lat1))


def nearest_place(lat, lon):
    """(iso3, km) of the nearest CROWD locality."""
    best = (None, 1e9)
    for iso3, pts in table()["places"].items():
        for plat, plon in pts:
            if abs(plat - lat) < 3 and abs(plon - lon) < 4:
                d = _km(lat, lon, plat, plon)
                if d < best[1]:
                    best = (iso3, d)
    return best


def _reverse_geocode(lat, lon, cache):
    """ISO2 of the country at (lat, lon) from OSM Nominatim (no key; one request, cached), else None."""
    if cache is not None and cache.exists():
        return json.loads(cache.read_text()).get("iso2")
    try:
        import requests
        from .realworld import UA
        r = requests.get(NOMINATIM, params={"lat": f"{lat:.4f}", "lon": f"{lon:.4f}", "format": "jsonv2",
                                            "zoom": 3}, headers=UA, timeout=20)
        r.raise_for_status()
        iso2 = (r.json().get("address") or {}).get("country_code", "").upper() or None
    except Exception as err:  # noqa: BLE001 - offline: the world's values
        _log(f"reverse geocoding failed ({err})")
        return None
    if cache is not None:
        cache.parent.mkdir(parents=True, exist_ok=True)
        cache.write_text(json.dumps({"lat": lat, "lon": lon, "iso2": iso2, "source": "OSM Nominatim (ODbL)"}))
    return iso2


def locate(cfg, bbox, cache=None):
    """The map's country: (iso3 or None, how). The experiment's city.country, else the nearest CROWD locality within
    25 km, else OSM Nominatim (cached in `cache`), else the nearest CROWD locality within 150 km."""
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    named = country_code(((cfg or {}).get("city") or {}).get("country"))
    if named:
        return named, "experiment city.country"
    iso3, km = nearest_place(lat, lon)
    if iso3 and km <= 25:
        return iso3, f"nearest CROWD locality ({km:.0f} km)"
    iso2 = _reverse_geocode(lat, lon, cache)
    if iso2 and country_code(iso2):
        return country_code(iso2), "OSM Nominatim"
    if iso3 and km <= 150:
        return iso3, f"nearest CROWD locality ({km:.0f} km)"
    return None, "unknown (world values)"


def _nearest_locality(lat, lon, iso3, max_km=25.0):
    best, bd = None, max_km
    for loc in table()["localities"]:
        if loc["iso3"] == iso3 and abs(loc["lat"] - lat) < 0.5 and abs(loc["lon"] - lon) < 0.7:
            d = _km(lat, lon, loc["lat"], loc["lon"])
            if d < bd:
                best, bd = loc, d
    return best


def behaviour(wcfg, cfg, bbox, night=False, cache=None):
    """The crowd's parameters for the map, each with where it came from (locality, country, continent, world)."""
    w = {**DEFAULTS, **(wcfg or {})}
    t = table()
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    iso3, how = locate(cfg, bbox, cache)
    country = t["countries"].get(iso3) or {}
    cont = t["continents"].get(country.get("continent")) or {}
    loc = _nearest_locality(lat, lon, iso3) if iso3 else None
    world = t["world"]
    levels = [("locality " + loc["name"], loc)] if loc else []
    levels += [("country " + country["name"], country)] if country else []
    levels += [("continent " + country["continent"], cont)] if cont else []
    levels += [("world", world)]

    def pick(key, ok):
        for name, d in levels:
            if d.get(key) is not None and ok(d):
                return d[key], name, d
        return None, None, None

    ppm, ppm_src, ppm_d = pick("persons_per_min", lambda d: d.get("hours", 1e9) >= (5 if d is loc else 10))
    spd, spd_src, spd_d = pick("speed_mps", lambda d: d.get("speed_n", 0) >= 30)
    # the crossing share comes from the same crowd-city table, at the same level as the speed
    share = spd_d.get("crossing_share") if "crossing_share" in spd_d else \
        spd_d["crossing_tracks"] / max(1, spd_d["person_tracks"])
    share_src = spd_src
    # jaywalking: measured for the country, else modelled from its pedestrians per minute, else the world median
    jay, jay_src = None, None
    if w["jaywalk_share"] != "auto":
        jay, jay_src = float(w["jaywalk_share"]), "traffic.walkers.jaywalk_share"
    elif country.get("jaywalk_share") is not None and country.get("jaywalk_n", 0) >= 50:
        jay, jay_src = country["jaywalk_share"], f"measured, {country['name']} ({country['jaywalk_n']} crossings)"
    elif t.get("jaywalk_model") and country.get("persons_per_min") and country.get("hours", 0) >= 10:
        m = t["jaywalk_model"]
        z = m["a"] + m["b"] * math.log(country["persons_per_min"])
        jay, jay_src = 1 / (1 + math.exp(-z)), f"modelled from {country['name']}'s pedestrians per minute"
    else:
        jay, jay_src = world.get("jaywalk_share") or 0.02, "world median of the measured countries"
    density = min(FACTOR_MAX, max(1 / FACTOR_MAX, ppm / world["persons_per_min"]))
    night_factor = 1.0
    if night:
        d = next((d for _, d in levels if d.get("persons_per_min_night") and d.get("persons_per_min_day")), world)
        night_factor = d["persons_per_min_night"] / d["persons_per_min_day"]
    ref = world.get("crossing_share_city_median") or world["crossing_share"]
    cross = min(FACTOR_MAX, max(1 / FACTOR_MAX, share / ref)) if share else 1.0
    if w["speed_mps"] != "auto":
        median, sd, spd_src = float(w["speed_mps"]), 0.15, "traffic.walkers.speed_mps"
    else:
        median, sd = spd, max(0.05, (spd_d["speed_q75"] - spd_d["speed_q25"]) / 1.349)
    return {
        "country": iso3, "country_name": country.get("name"), "located_by": how,
        "persons_per_min": ppm, "persons_per_min_from": ppm_src,
        "density_factor": round(density, 3), "night_factor": round(night_factor, 3),
        "speed_median": round(median, 3), "speed_sd": round(sd, 3), "speed_from": spd_src,
        "crossing_share": round(share, 4) if share else None, "crossing_share_from": share_src,
        "crossing_factor": round(cross, 3), "crossings_per_km": round(float(w["crossings_per_km"]) * cross, 3),
        "jaywalk_share": round(jay, 4), "jaywalk_from": jay_src,
        "walkers_per_km": float(w["walkers_per_km"]) * density * night_factor, "max_count": int(w["max_count"]),
        "crowd_version": t["source"]["crowd_version"],
    }


# --- kerbs from the SUMO net ----------------------------------------------------------------------------------------

class Segment:
    """One road between junctions, as its two kerbs: sides[0] and sides[1] are lists of (x, y) every `step` metres,
    aligned so that sides[0][i] faces sides[1][i]. `crossings` holds the sample indices of marked crossings,
    `junction` whether each end (start, end) meets a junction."""

    def __init__(self, sid, a, b, step=2.5, crossings=(), junction=(True, True), name=""):
        self.id, self.sides, self.step, self.name = sid, [list(a), list(b)], step, name
        self.crossings = sorted(set(crossings))
        self.junction = tuple(junction)

    @property
    def n(self):
        return len(self.sides[0])

    @property
    def length(self):
        return (self.n - 1) * self.step

    def width(self, i):
        (ax, ay), (bx, by) = self.sides[0][i], self.sides[1][i]
        return math.hypot(bx - ax, by - ay)

    def mid(self, i):
        (ax, ay), (bx, by) = self.sides[0][i], self.sides[1][i]
        return (ax + bx) / 2, (ay + by) / 2


def _resample(pts, step):
    out, carry = [pts[0]], 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        d = step - carry
        while d <= seg:
            out.append((x0 + (x1 - x0) * d / seg, y0 + (y1 - y0) * d / seg))
            d += step
        carry = seg - (d - step)
    return out


def _offset(pts, dist):
    """The polyline moved `dist` metres to its right (negative: left)."""
    out = []
    for i, (x, y) in enumerate(pts):
        (x0, y0), (x1, y1) = pts[max(0, i - 1)], pts[min(len(pts) - 1, i + 1)]
        dx, dy = x1 - x0, y1 - y0
        n = math.hypot(dx, dy) or 1.0
        out.append((x + dy / n * dist, y - dx / n * dist))
    return out


def _project(p, pts):
    """The point of polyline pts closest to p."""
    best, bd = pts[0], 1e18
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        dx, dy = x1 - x0, y1 - y0
        L = dx * dx + dy * dy
        u = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - x0) * dx + (p[1] - y0) * dy) / L))
        q = (x0 + u * dx, y0 + u * dy)
        d = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2
        if d < bd:
            best, bd = q, d
    return best


def _mean(pts):
    return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)


def _road_edge(e):
    t = e.getType() or ""
    if "cycleway" in t or "footway" in t or "pedestrian" in t or "path" in t:
        return False
    return any(l.allows("passenger") or l.allows("bus") for l in e.getLanes())


def _outer_kerb(lane, away_from, kerb_m):
    """The kerb beside `lane`, on the side away from the point `away_from` (the rest of the road)."""
    shape = lane.getShape()
    if not lane.allows("passenger") and not lane.allows("bus") and lane.allows("pedestrian"):
        return list(shape)                         # a sidewalk lane: walk on it
    d = lane.getWidth() / 2 + kerb_m
    right = _offset(shape, d)
    mx, my = _mean(shape)
    rx, ry = _mean(right)
    ax, ay = away_from
    return right if (rx - mx) * (mx - ax) + (ry - my) * (my - ay) >= 0 else _offset(shape, -d)


def kerb_segments(net, step=2.5, kerb_m=1.5, min_len=25.0):
    """Segments (see Segment) of the SUMO net's roads (not cycleways): a two-way road's kerbs are its two edges'
    outer lanes moved out by half a lane and kerb_m; a one-way road's are its outermost lanes'."""
    edges = {e.getID(): e for e in net.getEdges() if _road_edge(e)}
    done, out = set(), []
    for eid, e in sorted(edges.items()):
        if eid in done or e.getLength() < min_len:
            continue
        opp = edges.get(eid[1:] if eid.startswith("-") else "-" + eid)
        done.add(eid)
        lanes = e.getLanes()
        if opp is not None:
            done.add(opp.getID())
            a = _outer_kerb(lanes[0], _mean(opp.getLanes()[0].getShape()), kerb_m)
            b = _outer_kerb(opp.getLanes()[0], _mean(lanes[0].getShape()), kerb_m)[::-1]
        elif len(lanes) > 1:
            a = _outer_kerb(lanes[0], _mean(lanes[-1].getShape()), kerb_m)
            b = _outer_kerb(lanes[-1], _mean(lanes[0].getShape()), kerb_m)
        else:
            shape = lanes[0].getShape()
            d = lanes[0].getWidth() / 2 + kerb_m
            a, b = _offset(shape, d), _offset(shape, -d)
        a = _resample(a, step)
        if len(a) < 4:
            continue
        b = [_project(p, b) for p in a]
        jn = (len(e.getFromNode().getIncoming()) + len(e.getFromNode().getOutgoing()) > 2 + (opp is not None) * 2,
              len(e.getToNode().getIncoming()) + len(e.getToNode().getOutgoing()) > 2 + (opp is not None) * 2)
        out.append(Segment(eid.lstrip("-"), a, b, step, junction=jn, name=e.getName() or ""))
    return out


def osm_crossings(osm_path):
    """[(lat, lon, signals)] of the OSM highway=crossing nodes in map.osm."""
    out = []
    try:
        for _, el in ET.iterparse(str(osm_path)):
            if el.tag == "node":
                tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
                if tags.get("highway") == "crossing" and tags.get("crossing") != "no":
                    sig = tags.get("crossing") == "traffic_signals" or tags.get("crossing_ref") == "pelican"
                    out.append((float(el.get("lat")), float(el.get("lon")), sig))
                el.clear()
            elif el.tag in ("way", "relation"):
                el.clear()
    except (OSError, ET.ParseError) as err:
        _log(f"no OSM crossings ({err})")
    return out


def snap_crossings(segments, points_xy, max_off=4.0):
    """Marks each crossing point (x, y) on the segment it lies on: within the road (half its width + max_off)."""
    cell, grid = 20.0, {}
    for s in segments:
        for i in range(s.n):
            mx, my = s.mid(i)
            grid.setdefault((int(mx // cell), int(my // cell)), []).append((mx, my, s, i))
    n = 0
    for x, y in points_xy:
        best = None
        gx, gy = int(x // cell), int(y // cell)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for mx, my, s, i in grid.get((gx + dx, gy + dy), ()):
                    d = math.hypot(mx - x, my - y)
                    if d <= s.width(i) / 2 + max_off and (best is None or d < best[0]):
                        best = (d, s, i)
        if best:
            s, i = best[1], best[2]
            s.crossings = sorted(set(s.crossings) | {i})
            n += 1
    return n


def crop(segments, centres, radius):
    """The part of each segment within `radius` of any centre (its longest run of samples); [] centres: all."""
    if not centres:
        return list(segments)
    out = []
    for s in segments:
        inside = [any(math.hypot(s.mid(i)[0] - cx, s.mid(i)[1] - cy) <= radius for cx, cy in centres)
                  for i in range(s.n)]
        best, start = (0, -1), None
        for i, ok in enumerate(inside + [False]):
            if ok and start is None:
                start = i
            elif not ok and start is not None:
                if i - start > best[1] - best[0] + 1:
                    best = (start, i - 1)
                start = None
        lo, hi = best
        if hi - lo < 3:
            continue
        c = Segment(s.id, s.sides[0][lo:hi + 1], s.sides[1][lo:hi + 1], s.step,
                    [i - lo for i in s.crossings if lo <= i <= hi],
                    (s.junction[0] and lo == 0, s.junction[1] and hi == s.n - 1), s.name)
        out.append(c)
    return out


# --- where pedestrians are ----------------------------------------------------------------------------------------

def overture_places(bbox, path, release=None):
    """[[lat, lon, weight, kind]] of the Overture places in the box (cached in path), or None without DuckDB."""
    if path.exists():
        return json.loads(path.read_text())
    try:
        import duckdb
    except ImportError:
        _log("Overture places skipped: pip install duckdb")
        return None
    from .realworld import OVERTURE_RELEASE
    s, w, n, e = bbox
    try:
        con = duckdb.connect()
        con.execute("INSTALL httpfs; LOAD httpfs; INSTALL spatial; LOAD spatial; SET s3_region='us-west-2';")
        q = f"""SELECT taxonomy.hierarchy[1], basic_category, taxonomy.primary, confidence, ST_Y(geometry), ST_X(geometry)
                FROM read_parquet('{OVERTURE_PLACES.format(rel=release or OVERTURE_RELEASE)}', hive_partitioning=1)
                WHERE bbox.xmin < {e} AND bbox.xmax > {w} AND bbox.ymin < {n} AND bbox.ymax > {s}
                  AND (operating_status IS NULL OR operating_status = 'open')"""
        rows = con.execute(q).fetchall()
    except Exception as err:  # noqa: BLE001 - S3 down or offline: no places
        _log(f"Overture places failed ({err})")
        return None
    out = []
    for top, basic, primary, conf, lat, lon in rows:
        cat = f"{basic or ''} {primary or ''}"
        station = any(k in cat for k in STATION_WORDS)
        wgt = 10.0 if station else PLACE_WEIGHT.get(top, 0.5)
        if wgt > 0:
            out.append([round(lat, 6), round(lon, 6), round(wgt * (conf if conf is not None else 0.5), 3),
                        "station" if station else (top or "other")])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, separators=(",", ":")))
    _log(f"Overture places: {len(out)} ({sum(p[3] == 'station' for p in out)} stations)")
    return out


def osm_places(bbox, path):
    """The same from OSM (shops, food, schools, stations) where DuckDB is missing; cached in path."""
    if path.exists():
        return json.loads(path.read_text())
    from .realworld import overpass_json
    s, w, n, e = bbox
    b = f"({s},{w},{n},{e})"
    q = f"""[out:json][timeout:120];
(nwr["shop"]{b};
 nwr["amenity"~"^(restaurant|cafe|bar|pub|fast_food|food_court|school|university|college|library|theatre|cinema|marketplace|place_of_worship|townhall|hospital|bank)$"]{b};
 nwr["railway"~"^(station|halt|subway_entrance|tram_stop)$"]{b};
 nwr["public_transport"="station"]{b};);
out center tags;"""
    try:
        data = overpass_json(q)
    except Exception as err:  # noqa: BLE001
        _log(f"OSM places failed ({err})")
        return None
    out = []
    for el in data.get("elements", []):
        lat, lon = el.get("lat") or (el.get("center") or {}).get("lat"), el.get("lon") or (el.get("center") or {}).get("lon")
        if lat is None:
            continue
        t = el.get("tags", {})
        if t.get("railway") or t.get("public_transport") == "station":
            out.append([lat, lon, 10.0, "station"])
        elif t.get("amenity") in ("restaurant", "cafe", "bar", "pub", "fast_food", "food_court"):
            out.append([lat, lon, 3.0, "food_and_drink"])
        elif "shop" in t:
            out.append([lat, lon, 2.0, "shopping"])
        else:
            out.append([lat, lon, 1.5, "other"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, separators=(",", ":")))
    _log(f"OSM places: {len(out)}")
    return out


def transit_stops(real_dir):
    """[[lat, lon, weight, 'stop']] of the bus/tram stops in the map's cached OSM scenery."""
    p = real_dir / "osm_scenery.json"
    if not p.exists():
        return []
    out = []
    for el in json.loads(p.read_text()).get("elements", []):
        t = el.get("tags", {})
        if el.get("type") == "node" and (t.get("highway") == "bus_stop" or t.get("public_transport") == "platform"):
            out.append([el["lat"], el["lon"], 5.0, "stop"])
    return out


def _num(v):
    try:
        return float(str(v).replace(",", ""))
    except (TypeError, ValueError):
        return None


def _nyc_counts(bbox):
    s, w, n, e = bbox
    import requests
    from .realworld import UA
    r = requests.get(NYC_PED, params={"$where": f"within_box(the_geom,{n + 0.005},{w - 0.006},{s - 0.005},{e + 0.006})",
                                      "$limit": 500}, headers=UA, timeout=60)
    r.raise_for_status()
    out = []
    for row in r.json():
        g = row.get("the_geom") or {}
        if not g.get("coordinates"):
            continue
        # the survey columns are named like may_07_am, sept_19_md, oct24_pm: take the latest year's mean
        by_year = {}
        for k, v in row.items():
            parts = k.replace("_p_m", "_pm").split("_")
            tag = "".join(ch for ch in parts[0] + (parts[1] if len(parts) > 2 else "") if ch.isdigit())
            if len(tag) == 2 and parts[-1] in ("am", "pm", "md") and _num(v) is not None:
                by_year.setdefault(int(tag), []).append(_num(v))
        if by_year:
            yr = max(by_year)
            out.append({"lat": g["coordinates"][1], "lon": g["coordinates"][0], "count": sum(by_year[yr]) / len(by_year[yr]),
                        "year": 2000 + yr, "name": f"{row.get('from_stree', '')} {row.get('to_street', '')}".strip()})
    return out


def _sydney_counts(bbox):
    s, w, n, e = bbox
    import requests
    from .realworld import UA
    env = f"{w - 0.006},{s - 0.005},{e + 0.006},{n + 0.005}"
    r = requests.get(f"{SYD}/WalkingCountSites/FeatureServer/0/query",
                     params={"where": "1=1", "outFields": "Site_ID,Location", "geometry": env, "inSR": 4326,
                             "geometryType": "esriGeometryEnvelope", "spatialRel": "esriSpatialRelIntersects",
                             "outSR": 4326, "f": "json"}, headers=UA, timeout=60)
    r.raise_for_status()
    sites = {f["attributes"]["Site_ID"]: (f["geometry"]["y"], f["geometry"]["x"], f["attributes"]["Location"])
             for f in r.json().get("features", []) if f.get("geometry")}
    if not sites:
        return []
    ids = ",".join(str(i) for i in sites)
    r = requests.get(f"{SYD}/Walking_count_surveys/FeatureServer/0/query",
                     params={"where": f"SiteID IN ({ids}) AND Period='weekday'", "outFields": "SiteID,Year,TotalCount",
                             "f": "json", "resultRecordCount": 5000}, headers=UA, timeout=60)
    r.raise_for_status()
    latest = {}
    for f in r.json().get("features", []):
        a = f["attributes"]
        if a.get("TotalCount") and (a["SiteID"] not in latest or a["Year"] > latest[a["SiteID"]][0]):
            latest[a["SiteID"]] = (a["Year"], a["TotalCount"])
    return [{"lat": sites[i][0], "lon": sites[i][1], "count": c, "year": y, "name": sites[i][2]}
            for i, (y, c) in latest.items()]


PED_COUNTS = [   # (name, area (s, w, n, e), fetch, source key)
    ("nyc", (40.49, -74.27, 40.92, -73.68), _nyc_counts, "nyc_ped_counts"),
    ("sydney", (-33.93, 151.15, -33.84, 151.25), _sydney_counts, "sydney_walking_counts"),
]


def ped_counts(bbox, path):
    """Open pedestrian counts in and around the box: {source, counts: [{lat, lon, count, year, name}]}; cached."""
    if path.exists():
        return json.loads(path.read_text())
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    data = {"source": None, "counts": []}
    for name, (s, w, n, e), fetch, key in PED_COUNTS:
        if s <= lat <= n and w <= lon <= e:
            try:
                data = {"source": key, "counts": fetch(bbox)}
            except Exception as err:  # noqa: BLE001 - the service is down: places alone, not cached
                _log(f"{name} pedestrian counts failed ({err})")
                return data
            _log(f"{name} pedestrian counts: {len(data['counts'])} sites")
            break
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))
    return data


def attraction(segments, pois_xy, counts_xy=(), sigma=40.0, floor_share=0.25, count_sigma=150.0):
    """Weights per segment side and sample: the places' weights with a Gaussian of sigma metres, plus a floor of
    floor_share x the mean (streets without places keep some walkers), times the relative pedestrian count of the
    count sites nearby (count / median; Gaussian of count_sigma) where there are any."""
    cell = 3 * sigma
    grid = {}
    for x, y, w in pois_xy:
        grid.setdefault((int(x // cell), int(y // cell)), []).append((x, y, w))
    k2 = 2 * sigma * sigma
    att = []
    for s in segments:
        sides = []
        for side in s.sides:
            vals = []
            for x, y in side:
                gx, gy = int(x // cell), int(y // cell)
                v = 0.0
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for px, py, w in grid.get((gx + dx, gy + dy), ()):
                            v += w * math.exp(-((px - x) ** 2 + (py - y) ** 2) / k2)
                vals.append(v)
            sides.append(vals)
        att.append(sides)
    total = [v for s in att for side in s for v in side]
    mean = sum(total) / len(total) if total else 0.0
    floor = floor_share * mean if mean > 0 else 1.0
    counts = [(x, y, c) for x, y, c in counts_xy if c and c > 0]
    med = sorted(c for _, _, c in counts)[len(counts) // 2] if counts else None
    kc = 2 * count_sigma * count_sigma
    for s, sides in zip(segments, att):
        for side, vals in zip(s.sides, sides):
            for i, (x, y) in enumerate(side):
                f = 1.0
                if counts:
                    ws = [(math.exp(-((cx - x) ** 2 + (cy - y) ** 2) / kc), c / med) for cx, cy, c in counts]
                    wsum = sum(w for w, _ in ws)
                    if wsum > 0.05:     # near a count site: its relative count, blended to 1 further away
                        f = (sum(w * r for w, r in ws) + max(0.0, 1 - wsum)) / max(1.0, wsum)
                vals[i] = (vals[i] + floor) * f
    return att


# --- the walker plan ------------------------------------------------------------------------------------------------

def plan(segments, att, params, count, duration_s, rng):
    """Plans `count` walkers: [{seg, side, i, speed, legs}], legs = [("walk", seg, side, i0, i1) |
    ("cross", seg, from_side, i, kind)] with kind crosswalk (a marked crossing), junction (at the junction end of a
    segment without a marked crossing) or midblock. A walker walks along its kerb, turning at the ends of its
    stretch; the wish to cross comes as a Poisson process in the distance walked (params["crossings_per_km"]). It
    then crosses where it is with probability params["jaywalk_share"] (mid-block, not within 10 m of a marked
    crossing or an end), else walks to the nearest marked crossing of its segment, else to a junction end."""
    rate = params["crossings_per_km"] / 1000.0
    jay = params["jaywalk_share"]
    choices, cum, acc = [], [], 0.0
    for k, sides in enumerate(att):
        for side in (0, 1):
            choices.append((k, side))
            acc += sum(sides[side])
            cum.append(acc)
    walkers = []
    if not choices or acc <= 0:
        return walkers
    for _ in range(count):
        k, side = choices[min(len(choices) - 1, _bisect.bisect(cum, rng.random() * acc))]
        s = segments[k]
        i = rng.choices(range(s.n), weights=att[k][side])[0]
        speed = min(2.4, max(0.6, rng.gauss(params["speed_median"], params["speed_sd"])))
        d = rng.choice((-1, 1))
        start, legs, walked = (k, side, i), [], 0.0
        need = speed * duration_s + 30.0
        wish = rng.expovariate(rate) if rate > 0 else float("inf")    # metres until the next wish to cross

        def walk(j):
            nonlocal i, walked
            if j != i:
                legs.append(("walk", k, side, i, j))
                walked += abs(j - i) * s.step
                i = j

        for _ in range(5000):
            if walked >= need:
                break
            end = s.n - 1 if d > 0 else 0
            stretch = abs(end - i) * s.step
            if wish > stretch:
                walk(end)
                wish -= stretch
                d = -d
                continue
            walk(max(0, min(s.n - 1, i + d * int(round(wish / s.step)))))
            wish = rng.expovariate(rate)
            kind = None
            if rng.random() < jay and 4 <= i <= s.n - 5 and all(abs(i - c) > 4 for c in s.crossings):
                kind = "midblock"
            elif s.crossings:
                walk(min(s.crossings, key=lambda c: abs(c - i)))
                kind = "crosswalk"
            else:
                ends = [e for e, jn in ((0, s.junction[0]), (s.n - 1, s.junction[1])) if jn]
                if ends:
                    walk(min(ends, key=lambda e: abs(e - i)))
                    kind = "junction"
            if kind:
                legs.append(("cross", k, side, i, kind))
                side = 1 - side
                d = rng.choice((-1, 1))
        walkers.append({"seg": start[0], "side": start[1], "i": start[2], "speed": round(speed, 3), "legs": legs})
    return walkers


def plan_calibrated(segments, att, params, count, duration_s, seed=1, rounds=6, sample=1500):
    """plan() with its crossing rate and mid-block probability adjusted so that the walkers cross
    params["crossings_per_km"] per km walked in the trial and params["jaywalk_share"] of the crossings are mid-block:
    on a short segment a walker cannot cross mid-block, on one without a marked crossing or a junction end it cannot
    cross legally, and walking to a crossing adds distance. Calibrated on `sample` walkers; each is raised at most 4
    times. Returns (walkers, rate used, mid-block probability used)."""
    target, jay = params["crossings_per_km"], params["jaywalk_share"]
    p = dict(params)
    for _ in range(rounds if target > 0 else 0):
        st = plan_stats(segments, plan(segments, att, p, sample, duration_s, random.Random(seed + 7)), duration_s)
        got, got_j = st["crossings_per_km"], st["midblock_share"]
        ok = got and abs(got / target - 1) < 0.05
        ok_j = jay <= 0 or (got_j is not None and abs(got_j - jay) < max(0.1 * jay, 0.003))
        if ok and ok_j:
            break
        if got and not ok:
            p["crossings_per_km"] = min(4 * target, p["crossings_per_km"] * target / got)
        if jay > 0 and not ok_j:
            p["jaywalk_share"] = min(1.0, 4 * jay, p["jaywalk_share"] * (jay / got_j if got_j else 2.0))
    return plan(segments, att, p, count, duration_s, random.Random(seed)), p["crossings_per_km"], p["jaywalk_share"]


def plan_stats(segments, walkers, duration_s):
    """Counts and shares of a plan, for the log and the checks: crossings by kind within the trial's duration."""
    kinds = {"crosswalk": 0, "junction": 0, "midblock": 0}
    walked = 0.0
    for w in walkers:
        t = 0.0
        for leg in w["legs"]:
            s = segments[leg[1]]
            if leg[0] == "walk":
                t += abs(leg[4] - leg[3]) * s.step / w["speed"]
                walked += abs(leg[4] - leg[3]) * s.step if t <= duration_s else 0.0
            else:
                t += s.width(leg[3]) / w["speed"]
                if t <= duration_s:
                    kinds[leg[4]] += 1
            if t > duration_s:
                break
    n = sum(kinds.values())
    return {"walkers": len(walkers), "crossings": n, **kinds,
            "midblock_share": round(kinds["midblock"] / n, 3) if n else None,
            "crossings_per_km": round(n / (walked / 1000), 2) if walked else None,
            "km_walked": round(walked / 1000, 2),
            "median_speed": round(sorted(w["speed"] for w in walkers)[len(walkers) // 2], 3) if walkers else None}


# --- gap acceptance ------------------------------------------------------------------------------------------------

def critical_gap(width, speed, waited_s=0.0):
    """HCM pedestrian critical headway t_c = L / S_p + t_s (s), reduced with waiting to 60 % after 60 s."""
    return (width / max(0.3, speed) + HCM_START_S) * max(0.6, 1.0 - 0.4 * waited_s / 60.0)


def gap_ok(a, b, vehicles, t_c, marked=False, decel=3.0, react=1.0):
    """Whether a walker may step out from a to cross to b: no vehicle reaches the crossing line within t_c seconds
    (vehicles: (x, y, vx, vy)); at a marked crossing a vehicle that can still stop (comfortable deceleration after a
    reaction time) does not count. A vehicle standing on the crossing line blocks it."""
    ax, ay = a
    bx, by = b
    ux, uy = bx - ax, by - ay
    L = math.hypot(ux, uy) or 1.0
    ux, uy = ux / L, uy / L
    for x, y, vx, vy in vehicles:
        # position along and across the crossing line
        along = (x - ax) * ux + (y - ay) * uy
        off = -(x - ax) * uy + (y - ay) * ux        # signed distance from the line
        v = math.hypot(vx, vy)
        if -2.0 <= along <= L + 2.0 and abs(off) < 2.5:
            return False                             # standing or driving on the crossing
        if v < 0.5:
            continue
        closing = -(vx * -uy + vy * ux) * (1 if off > 0 else -1)   # speed towards the line
        if closing <= 0.3:
            continue
        tta = abs(off) / closing
        hit = along + (vx * ux + vy * uy) * tta
        if not (-3.0 <= hit <= L + 3.0) or abs(off) > 120:
            continue
        if tta >= t_c:
            continue
        if marked and abs(off) > v * react + v * v / (2 * decel) + 3.0:
            continue
        return False
    return True


def replay(segments, walkers, frames, duration_s, dt=0.5):
    """Walks the planned walkers through recorded traffic without CARLA: frames = sorted [(t, [(x, y, vx, vy)])] in
    CARLA coordinates (e.g. SUMO's FCD). Returns one dict per crossing started within duration_s: kind, the wait at
    the kerb (s), whether the walker gave up waiting before the trial ended (then wait is censored), the gap that
    was accepted (time until the nearest approaching vehicle reached the crossing line, s; None: no vehicle came)."""
    times = [f[0] for f in frames]

    def at(t):
        k = max(0, min(len(frames) - 1, _bisect.bisect_right(times, t) - 1))
        return frames[k][1] if frames else []

    def xy(seg, side, i):
        x, y = segments[seg].sides[side][i]
        return x, -y

    out = []
    for w in walkers:
        t, v = 0.0, w["speed"]
        for leg in w["legs"]:
            if t > duration_s:
                break
            s = segments[leg[1]]
            if leg[0] == "walk":
                t += abs(leg[4] - leg[3]) * s.step / v
                continue
            a, b = xy(leg[1], leg[2], leg[3]), xy(leg[1], 1 - leg[2], leg[3])
            marked = leg[4] in ("crosswalk", "junction")
            t0 = t
            while t <= duration_s and not gap_ok(a, b, at(t), critical_gap(s.width(leg[3]), v, t - t0), marked):
                t += dt
            if t > duration_s:
                out.append({"kind": leg[4], "wait": t - t0, "censored": True, "gap": None})
                break
            out.append({"kind": leg[4], "wait": round(t - t0, 2), "censored": False,
                        "gap": _next_arrival(a, b, at(t))})
            t += s.width(leg[3]) / v
    return out


def _next_arrival(a, b, vehicles):
    """Seconds until the first vehicle heading for the crossing line a-b reaches it (None: none within 120 m)."""
    ax, ay = a
    ux, uy = b[0] - ax, b[1] - ay
    L = math.hypot(ux, uy) or 1.0
    ux, uy = ux / L, uy / L
    best = None
    for x, y, vx, vy in vehicles:
        off = -(x - ax) * uy + (y - ay) * ux
        closing = -(vx * -uy + vy * ux) * (1 if off > 0 else -1)
        if closing <= 0.3 or abs(off) > 120:
            continue
        tta = abs(off) / closing
        hit = (x - ax) * ux + (y - ay) * uy + (vx * ux + vy * uy) * tta
        if -3.0 <= hit <= L + 3.0 and (best is None or tta < best):
            best = tta
    return round(best, 2) if best is not None else None


# --- set-up for a trial ---------------------------------------------------------------------------------------------

def prepare(tc, mdir, geo_net, centres_xy, duration_s, night=False, log=_log, fetch=True):
    """Everything about the crowd that needs no server: (params, segments, walkers, stats). centres_xy in geo.net
    x, y (the participants; [] for the whole map)."""
    wc = {**DEFAULTS, **(tc["traffic"].get("walkers") or {})}
    bbox = tc["map"]["bbox"]
    real = Path(mdir) / "real"
    params = behaviour(wc, tc, bbox, night, real / "crowdpeds_country.json")
    t0 = time.time()
    segs = kerb_segments(geo_net)
    xings = osm_crossings(Path(mdir) / "map.osm")
    n_x = snap_crossings(segs, [geo_net.convertLonLat2XY(lon, lat) for lat, lon, _ in xings])
    segs = crop(segs, centres_xy, float(wc.get("radius_m", 120)) + 30.0)
    pois, counts, used = [], [], []
    if wc["places"] not in (False, "off", None):
        got = overture_places(bbox, real / "overture_places.json") if fetch or (real / "overture_places.json").exists() else None
        if got is None and (fetch or (real / "osm_places.json").exists()):
            got = osm_places(bbox, real / "osm_places.json")
            used += ["osm"] if got else []
        elif got is not None:
            used.append("overture_places")
        pois = (got or []) + transit_stops(real)
        pc = ped_counts(bbox, real / "ped_counts.json") if fetch or (real / "ped_counts.json").exists() else {}
        counts = pc.get("counts") or []
        if counts:
            used.append(pc["source"])
    to_xy = geo_net.convertLonLat2XY
    att = attraction(segs, [(*to_xy(lon, lat), w) for lat, lon, w, _ in pois],
                     [(*to_xy(c["lon"], c["lat"]), c["count"]) for c in counts])
    kerb_km = sum(2 * s.length for s in segs) / 1000.0
    count = wc.get("count", 0)
    if count == "auto":
        count = int(round(min(params["max_count"], params["walkers_per_km"] * kerb_km)))
    count = int(count)
    walkers, rate, jay = plan_calibrated(segs, att, params, count, duration_s, int(wc.get("seed", 1)))
    stats = plan_stats(segs, walkers, duration_s)
    stats["rate_used"], stats["jaywalk_used"] = round(rate, 3), round(jay, 4)
    stats.update({"kerb_km": round(kerb_km, 2), "segments": len(segs), "osm_crossings": len(xings),
                  "crossings_snapped": n_x, "segments_with_crossing": sum(1 for s in segs if s.crossings),
                  "places": len(pois), "count_sites": len(counts), "place_sources": used,
                  "plan_s": round(time.time() - t0, 1)})
    log(f"crowd: {params['country_name'] or 'unknown country'} ({params['located_by']}): {count} walkers "
        f"({params['persons_per_min']} pedestrians/min, {params['persons_per_min_from']}), speed "
        f"{params['speed_median']} ± {params['speed_sd']} m/s ({params['speed_from']}), "
        f"{params['crossings_per_km']} crossings/km, mid-block share {params['jaywalk_share']:.3f} "
        f"({params['jaywalk_from']}); {stats['crossings']} crossings planned "
        f"({stats['crosswalk']} marked, {stats['junction']} at junctions, {stats['midblock']} mid-block); "
        f"{len(pois)} places, {len(counts)} count sites")
    return params, segs, walkers, stats


class CrowdSim:
    """The CARLA side: spawns the planned walkers and walks them, waiting at the kerb for a gap before each crossing.
    Walkers on the road are returned by on_road(), so SUMO traffic and scripted cars yield to them."""

    def __init__(self, world, geo, segments, walkers, params, seed=1, log=_log):
        import carla
        self.carla, self.world = carla, world
        self.params, self.events = params, []
        cmap = world.get_map()
        rng = random.Random(seed)
        bps = sorted(world.get_blueprint_library().filter("walker.pedestrian.*"), key=lambda b: b.id)

        def xy(seg, side, i):
            x, y = segments[seg].sides[side][i]
            return x, -y                                 # geo.net -> CARLA

        self.walkers, self.state = [], []
        for w in walkers:
            x, y = xy(w["seg"], w["side"], w["i"])
            bp = rng.choice(bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            bp.set_attribute("role_name", "crowd")
            wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Any)
            z = wp.transform.location.z if wp is not None else 0.0      # the road's height there (geo.ground)
            actor = world.try_spawn_actor(bp, carla.Transform(carla.Location(x=x, y=y, z=z + 1.0)))
            if actor is None:
                continue
            route = []                               # [(kind, [(x, y), ...], width, marked)]
            for leg in w["legs"]:
                s = segments[leg[1]]
                if leg[0] == "walk":
                    step = 1 if leg[4] > leg[3] else -1
                    route.append(("walk", [xy(leg[1], leg[2], j) for j in range(leg[3] + step, leg[4] + step, step)],
                                  0.0, False))
                else:
                    route.append(("cross", [xy(leg[1], leg[2], leg[3]), xy(leg[1], 1 - leg[2], leg[3])],
                                  s.width(leg[3]), leg[4] in ("crosswalk", "junction")))
            self.walkers.append(actor)
            self.state.append({"route": route, "leg": 0, "pt": 0, "speed": w["speed"], "wait_from": None,
                               "crossing": False, "kinds": [leg[4] for leg in w["legs"] if leg[0] == "cross"]})
        log(f"crowd: {len(self.walkers)} of {len(walkers)} walkers spawned")

    def on_road(self):
        return [a for a, st in zip(self.walkers, self.state) if st["crossing"]]

    def _vehicles(self):
        out = []
        for v in self.world.get_actors().filter("vehicle.*"):
            try:
                loc, vel = v.get_location(), v.get_velocity()
            except RuntimeError:
                continue
            out.append((loc.x, loc.y, vel.x, vel.y))
        return out

    def step(self, t):
        carla = self.carla
        vehicles = None
        for actor, st in zip(self.walkers, self.state):
            try:
                if st["leg"] >= len(st["route"]):
                    actor.apply_control(carla.WalkerControl(speed=0.0))
                    continue
                kind, pts, width, marked = st["route"][st["leg"]]
                loc = actor.get_location()
                if kind == "cross" and st["pt"] == 0:
                    # at the kerb: wait for a gap
                    if st["wait_from"] is None:
                        st["wait_from"] = t
                    if vehicles is None:
                        vehicles = self._vehicles()
                    t_c = critical_gap(width, st["speed"], t - st["wait_from"])
                    if not gap_ok(pts[0], pts[1], vehicles, t_c, marked):
                        actor.apply_control(carla.WalkerControl(speed=0.0))
                        continue
                    self.events.append(("crowd_crosses", f"{actor.id} {'marked' if marked else 'mid-block'} "
                                        f"{width:.1f} m after {t - st['wait_from']:.1f} s"))
                    st["pt"], st["crossing"], st["wait_from"] = 1, True, None
                tx, ty = pts[st["pt"]]
                dx, dy = tx - loc.x, ty - loc.y
                d = math.hypot(dx, dy)
                if d < 0.8:
                    st["pt"] += 1
                    if st["pt"] >= len(pts):
                        st["leg"], st["pt"], st["crossing"] = st["leg"] + 1, 0, False
                    continue
                actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(dx / d, dy / d, 0.0),
                                                        speed=st["speed"]))
            except RuntimeError:
                pass


def make_crowd(world, geo, tc, mdir, centres, night, duration_s, log=_log):
    """The trial's crowd: None without walkers, ai.Crowd (beside the roads, never crossing) with behaviour off, else
    a CrowdSim planned by prepare()."""
    wc = {**DEFAULTS, **(tc["traffic"].get("walkers") or {})}
    count = wc.get("count", 0)
    if count != "auto" and int(count or 0) <= 0:
        return None
    def legacy():
        from .ai import Crowd
        return Crowd(world, centres, int(count) if count != "auto" else 30, float(wc.get("radius_m", 120)),
                     int(wc.get("seed", 1)), log=log)
    if wc["behaviour"] in ("off", False, None):
        return legacy()
    centres_xy = [(c.x, -c.y) for c in centres]
    try:
        params, segs, walkers, stats = prepare(tc, mdir, geo.net, centres_xy, duration_s, night, log)
    except Exception as err:  # noqa: BLE001 - a map the planner cannot read: the crowd of before
        log(f"crowd: CROWD behaviour failed ({err!r}); walkers beside the roads instead")
        return legacy()
    sim = CrowdSim(world, geo, segs, walkers, params, int(wc.get("seed", 1)), log)
    sim.stats = stats
    return sim
