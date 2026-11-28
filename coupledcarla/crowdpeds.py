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

from .geo import nearest_waypoint

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
# walkers_per_km (per km of kerb, at the world's CROWD pedestrians per minute): 14. A car passing a crowd of lambda
# walkers per kerb-km at u km/min meets about 2 * lambda * u new walkers a minute, a CROWD dashcam's count; the
# re-filmed twin clips (2026-11-24) saw 1.5 x that (walkers gather at shops and stops). The world's 18.1 persons a
# minute at an urban 25 km/h then needs about 14.5. With 8 the clips saw about half their place's CROWD rate
# (Eindhoven 8.7 vs 15.8 a minute, Salamanca 12.2 vs Spain's 21.6); CROWD records no dashcam speed, so this rests on
# the 25 km/h (tools/crowd_seen.py measures a trial against CROWD).
DEFAULTS = {"behaviour": "auto", "places": "auto", "walkers_per_km": 14.0, "max_count": 80,
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


ROUTE_AHEAD_MAX_M = 1500.0       # m of a driving participant's route the crowd also covers (crowd planned at the start)
ROUTE_SPEED_MPS = 13.9           # m/s: how far a driver gets in the trial, for that length (50 km/h)
ROUTE_STEP_M = 60.0              # m between the corridor's centres (the crop radius is 120 m)


def route_reach(duration_s, cap=ROUTE_AHEAD_MAX_M):
    """m of a driving participant's route ahead that its crowd covers: what it can drive in the trial, capped (the
    rolling window, crowdwindow.py, prepares kerbs further: traffic.walkers.window.route_max_m)."""
    return min(float(cap), ROUTE_SPEED_MPS * float(duration_s or 0.0))


def route_centres(polylines, step=ROUTE_STEP_M):
    """Points every `step` m along each polyline [(x, y), ...] (CARLA x, y), first and last included: extra crowd
    centres along a driving participant's route. The crowd was planned only around where the participants start, so a
    car that drove off left it behind (TU/e twin clip, 2026-11-24: no walker seen in 13 s of driving)."""
    out = []
    for line in polylines:
        pts = [(float(x), float(y)) for x, y in (line or [])]
        if not pts:
            continue
        out.append(pts[0])
        carry = 0.0
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            seg = math.hypot(bx - ax, by - ay)
            d = step - carry
            while d <= seg:
                f = d / seg
                out.append((ax + f * (bx - ax), ay + f * (by - ay)))
                d += step
            carry = (carry + seg) % step if seg else carry
        if out[-1] != pts[-1]:
            out.append(pts[-1])
    return out


def edges_xy(net, edges, offset, max_m, start_pos=0.0):
    """The shapes of `edges` (SUMO ids, in driving order) in CARLA x, y, up to max_m along them from start_pos on the
    first edge; edges the net lacks are skipped."""
    out, left = [], float(max_m)
    for k, e in enumerate(edges):
        try:
            edge = net.getEdge(e)
        except (KeyError, AttributeError):
            continue
        shape = list(edge.getShape())
        skip = start_pos if k == 0 else 0.0
        for (ax, ay), (bx, by) in zip(shape, shape[1:]):
            seg = math.hypot(bx - ax, by - ay)
            if skip >= seg:
                skip -= seg
                continue
            f0 = skip / seg if seg else 0.0
            skip = 0.0
            p0 = (ax + f0 * (bx - ax), ay + f0 * (by - ay))
            if not out:
                out.append((p0[0] - offset[0], offset[1] - p0[1]))
            run = seg * (1.0 - f0)
            if run >= left:
                f1 = f0 + (left / seg if seg else 0.0)
                out.append((ax + f1 * (bx - ax) - offset[0], offset[1] - (ay + f1 * (by - ay))))
                return out
            out.append((bx - offset[0], offset[1] - by))
            left -= run
    return out


def driver_route_xy(net, offset, route, pos, live, max_m):
    """A SUMO driver's route ahead in CARLA x, y up to max_m (sumodriver.SumoDriver.route_xy): live = (SUMO's route,
    the index of the edge it is on) once SUMO drives the car, from the start of that edge; else the planned route
    (edges) from pos m into its first edge."""
    if net is None:
        return []
    if live:
        edges, idx = live
        return edges_xy(net, list(edges[max(0, idx):]), offset, max_m, 0.0)
    if not route:
        return []
    return edges_xy(net, route, offset, max_m, float(pos or 0.0))


def crop(segments, centres, radius):
    """The part of each segment within `radius` of any centre (its longest run of samples); [] centres: all."""
    if not centres:
        return list(segments)
    cell = max(1.0, float(radius))
    grid = {}
    for cx, cy in centres:
        grid.setdefault((int(cx // cell), int(cy // cell)), []).append((cx, cy))

    def near(x, y):
        gx, gy = int(x // cell), int(y // cell)
        return any(math.hypot(x - cx, y - cy) <= radius for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                   for cx, cy in grid.get((gx + dx, gy + dy), ()))

    out = []
    for s in segments:
        inside = [near(*s.mid(i)) for i in range(s.n)]
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
        i = rng.choices(range(segments[k].n), weights=att[k][side])[0]
        walkers.append(plan_from(segments, params, k, side, i, duration_s, rng))
    return walkers


def plan_from(segments, params, k, side, i, duration_s, rng):
    """One walker of plan() that starts at kerb sample (k, side, i) (crowdwindow.py places walkers ahead of a
    driver with it); the same draws, in the same order, as plan() makes after choosing the start."""
    rate = params["crossings_per_km"] / 1000.0
    jay = params["jaywalk_share"]
    s = segments[k]
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
    return {"seg": start[0], "side": start[1], "i": start[2], "speed": round(speed, 3), "legs": legs}


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


def replay(segments, walkers, frames, duration_s, dt=0.5, decider=None):
    """Walks the planned walkers through recorded traffic without CARLA: frames = sorted [(t, [(x, y, vx, vy)])] in
    CARLA coordinates (e.g. SUMO's FCD). Returns one dict per crossing started within duration_s: kind, the wait at
    the kerb (s), whether the walker gave up waiting before the trial ended (then wait is censored), the gap that
    was accepted (time until the nearest approaching vehicle reached the crossing line, s; None: no vehicle came).
    decider: a perception model (embodiedped.EmbodiedCrossing) in place of gap_ok; its frames are (t, vehicles,
    vehicle ids)."""
    times = [f[0] for f in frames]

    def at(t):
        k = max(0, min(len(frames) - 1, _bisect.bisect_right(times, t) - 1))
        return frames[k][1] if frames else []

    def ok(n, t, a, b, t_c, marked):
        if decider is None:
            return gap_ok(a, b, at(t), t_c, marked)
        k = max(0, min(len(frames) - 1, _bisect.bisect_right(times, t) - 1))
        return decider.decide(n, t, a, b, t_c, frames[k][1], frames[k][2], marked)

    def xy(seg, side, i):
        x, y = segments[seg].sides[side][i]
        return x, -y

    out = []
    for n, w in enumerate(walkers):
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
            while t <= duration_s and not ok(n, t, a, b, critical_gap(s.width(leg[3]), v, t - t0), marked):
                t += dt
            if t > duration_s:
                out.append({"kind": leg[4], "wait": t - t0, "censored": True, "gap": None})
                break
            out.append({"kind": leg[4], "wait": round(t - t0, 2), "censored": False,
                        "gap": _next_arrival(a, b, at(t)), "gap_speed": _next_arrival(a, b, at(t), speed=True),
                        "t": round(t, 2), "t_c": round(critical_gap(s.width(leg[3]), v, t - t0), 2),
                        "cross_s": round(s.width(leg[3]) / v, 2)})
            t += s.width(leg[3]) / v
    return out


def _next_arrival(a, b, vehicles, speed=False):
    """Seconds until the first vehicle heading for the crossing line a-b reaches it (None: none within 120 m); with
    speed, that vehicle's speed (m/s) instead."""
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
            best, v_best = tta, math.hypot(vx, vy)
    if best is None:
        return None
    return round(v_best, 2) if speed else round(best, 2)


# --- set-up for a trial ---------------------------------------------------------------------------------------------

def prepare(tc, mdir, geo_net, centres_xy, duration_s, night=False, log=_log, fetch=True, crossings_xy=None,
            want_att=False, region=None, extend=None):
    """Everything about the crowd that needs no server: (params, segments, walkers, stats). centres_xy in geo.net
    x, y (the participants; [] for the whole map). crossings_xy: crossings in geo.net x, y for a map without
    map.osm (a baked level: its OpenDRIVE crosswalks, see make_crowd). region(segments) -> [[side 0 bools, side 1
    bools] per segment]: plan the walkers only on those kerb samples, count "auto" from their length (the drivers'
    windows, crowdwindow.initial_mask); the kerbs outside stay in `segments` for walkers placed later. extend: a
    dict filled with what kerbs further away need (all the map's segments, the places and count sites in geo.net
    x, y): crowdwindow adds them when a driver's route grows past the prepared ones."""
    wc = {**DEFAULTS, **(tc["traffic"].get("walkers") or {})}
    bbox = tc["map"]["bbox"]
    real = Path(mdir) / "real"
    params = behaviour(wc, tc, bbox, night, real / "crowdpeds_country.json")
    t0 = time.time()
    segs = kerb_segments(geo_net)
    osm = Path(mdir) / "map.osm"
    xings = osm_crossings(osm) if osm.exists() else []
    pts = [geo_net.convertLonLat2XY(lon, lat) for lat, lon, _ in xings]
    if not xings and crossings_xy:
        pts = list(crossings_xy)
    n_x = snap_crossings(segs, pts)
    all_segs = segs
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
    pois_xy = [(*to_xy(lon, lat), w) for lat, lon, w, _ in pois]
    counts_xy = [(*to_xy(c["lon"], c["lat"]), c["count"]) for c in counts]
    att = attraction(segs, pois_xy, counts_xy)
    if extend is not None:
        extend.update({"segments": all_segs, "pois": pois_xy, "counts": counts_xy})
    kerb_km = sum(2 * s.length for s in segs) / 1000.0
    att_plan, plan_km = att, kerb_km
    if region is not None:
        got, km = masked(segs, att, region(segs))
        if km > 0:          # a window without kerbs (a motorway): the whole corridor as before
            att_plan, plan_km = got, km
    count = wc.get("count", 0)
    if count == "auto":
        count = int(round(min(params["max_count"], params["walkers_per_km"] * plan_km)))
    count = int(count)
    walkers, rate, jay = plan_calibrated(segs, att_plan, params, count, duration_s, int(wc.get("seed", 1)))
    stats = plan_stats(segs, walkers, duration_s)
    stats["rate_used"], stats["jaywalk_used"] = round(rate, 3), round(jay, 4)
    stats.update({"kerb_km": round(kerb_km, 2), "plan_kerb_km": round(plan_km, 2),
                  "walkers_per_kerb_km": round(count / plan_km, 2) if plan_km else None, "segments": len(segs), "osm_crossings": len(xings),
                  "xodr_crossings": len(crossings_xy or []) if not xings else 0, "crossings_snapped": n_x, "segments_with_crossing": sum(1 for s in segs if s.crossings),
                  "places": len(pois), "count_sites": len(counts), "place_sources": used,
                  "plan_s": round(time.time() - t0, 1)})
    log(f"crowd: {params['country_name'] or 'unknown country'} ({params['located_by']}): {count} walkers "
        f"({params['persons_per_min']} pedestrians/min, {params['persons_per_min_from']}), speed "
        f"{params['speed_median']} ± {params['speed_sd']} m/s ({params['speed_from']}), "
        f"{params['crossings_per_km']} crossings/km, mid-block share {params['jaywalk_share']:.3f} "
        f"({params['jaywalk_from']}); {stats['crossings']} crossings planned "
        f"({stats['crosswalk']} marked, {stats['junction']} at junctions, {stats['midblock']} mid-block); "
        f"{len(pois)} places, {len(counts)} count sites"
        + (f"; planned on {plan_km:.2f} of {kerb_km:.2f} kerb-km (the drivers' windows), "
           f"{stats['walkers_per_kerb_km']} walkers per kerb-km" if plan_km != kerb_km else ""))
    if want_att:        # the living city plans walks of its own on the same weights (livingcity.walk_from)
        return params, segs, walkers, stats, att
    return params, segs, walkers, stats


def masked(segments, att, mask):
    """(att with the samples outside mask set to 0, kerb-km of the samples inside): plan() then starts walkers only
    inside (prepare's region)."""
    km = sum(s.step for s, m in zip(segments, mask) for side in m for on in side if on) / 1000.0
    return [[[v if on else 0.0 for v, on in zip(vals, m_side)] for vals, m_side in zip(sides, m)]
            for sides, m in zip(att, mask)], km


class CrowdSim:
    """The CARLA side: spawns the planned walkers and walks them, waiting at the kerb for a gap before each crossing.
    Walkers on the road are returned by on_road(), so SUMO traffic and scripted cars yield to them.

    The living city (livingcity.py) adds to a plan's walkers (all optional, absent: as before):
      bp, wheelchair      the blueprint (a child's) and CARLA 0.9.16's wheelchair (use_wheelchair) for the walker
      speed_factor        times its speed (a phone user, a group at group pace)
      follow              (leader index, right m, back m): walks beside its group's leader instead of its own route
      phone               at the kerb looks again only every PHONE_RECHECK_S (fewer looks; Thompson et al. 2013)
      xy, yaw             a start in CARLA x, y (a door, a bus door) instead of a kerb sample
      legs                besides walk and cross: ("to", [(x, y), ...]) walks to CARLA points, ("stop", (x, y) or
                          None, seconds, face yaw or None) stands there, ("vanish", reason) leaves the street (a door,
                          a bus, a car), ("wait", key) stands until release(index) (a bus stop, a taxi)
    and add() spawns such a walker during the trial. Hooks: on_stand(index, actor, state, kind) when a walker starts
    standing (a stop, a wait, the kerb before a crossing), on_walk(index, actor, state) when it walks on again,
    on_vanish(index, actor, reason). Reactions (reactions.py) set state "hurry"/"hurry_until" (faster on the road)
    and "back_until" (a step back from the kerb)."""

    PHONE_RECHECK_S = 2.0

    def __init__(self, world, geo, segments, walkers, params, seed=1, log=_log):
        self.send = None   # (actor, WalkerControl) -> None; the runner sends them with the tick's batch
        self.velocities = None   # {actor id: (vx, vy)}: SUMO vehicles' speeds (CARLA reads 0 m/s for them)
        self.signals = None      # pedsignals.PedSignals: at a signal, cross only when the cars crossing it have red
        self.poses = None        # the bridge's SUMO vehicle poses: walkers on the road wait for a car in their way
        self.decider = None      # embodiedped.EmbodiedCrossing: crossing decisions from what each walker sees
        self.on_stand = self.on_walk = self.on_vanish = None
        # hidden(x, y, occluder actor id or None) -> bool: whether a walker may vanish there unseen (livingcity.py:
        # beyond every participant's and camera's sight, or behind a bus or car); None: at once
        self.hidden = None
        self._ids = []           # actor ids of the vehicles of the last _vehicles()
        self.crossings = None    # scramble.CrossingCrowd: crowds at busy and scramble crossings (start_crossings)
        self.extend = None       # prepare()'s extend: the map's kerbs, places and count sites
        import carla
        self.carla, self.world = carla, world
        self.params, self.events = params, []
        self.segments = segments
        self.cmap = world.get_map()
        self.rng = random.Random(seed)
        self.lib = world.get_blueprint_library()
        self.bps = sorted(self.lib.filter("walker.pedestrian.*"), key=lambda b: b.id)
        self.actors, self.state = [], []
        self.log = log
        index = {}               # plan index -> state index (CARLA can refuse a spot, which shifts the indices)
        for i, w in enumerate(walkers):
            k = self.add(w)
            if k is not None:
                index[i] = k
        for st in self.state:    # a group member's leader by its index in the plan: re-pointed, or alone
            f = st["follow"]
            if f is not None:
                st["follow"] = (index[f[0]], f[1], f[2]) if f[0] in index else None
        n = len(index)
        log(f"crowd: {n} of {len(walkers)} walkers spawned")

    @property
    def walkers(self):
        """The walkers on the street now (not those gone through a door or onto a bus), the crossing crowds'
        included."""
        own = [a for a, st in zip(self.actors, self.state) if st["alive"]]
        return own + (self.crossings.walkers if self.crossings is not None else [])

    def start_crossings(self, tc, mdir, geo, centres_xy, queue=None, on_spawn=None, viewers=None, log=None):
        """Crowds at the busy and scramble crossings near centres_xy (CARLA x, y: the participants and their routes;
        scramble.py), once self.signals is set. Returns the CrossingCrowd or None."""
        from . import scramble
        log = log or self.log
        try:
            cc = scramble.make(self, tc, mdir, geo, self.signals, centres_xy, log)
            if cc is None:
                return None
            cc.queue, cc.viewers = queue, viewers
            cc.vehicles = self._vehicles
            cc.start(on_spawn)
        except Exception as err:  # noqa: BLE001 - the trial without them rather than none
            import traceback
            traceback.print_exc()
            log(f"crossings: not started ({type(err).__name__}: {err})")
            return None
        self.crossings = cc
        return cc

    def xy(self, seg, side, i):
        x, y = self.segments[seg].sides[side][i]
        return x, -y                                 # geo.net -> CARLA

    def route_of(self, w):
        """[(kind, [(x, y), ...], width or seconds, marked or face yaw or reason)] of a planned walker."""
        route = []
        for leg in w["legs"]:
            if leg[0] == "walk":
                step = 1 if leg[4] > leg[3] else -1
                route.append(("walk", [self.xy(leg[1], leg[2], j) for j in range(leg[3] + step, leg[4] + step, step)],
                              0.0, False))
            elif leg[0] == "cross":
                s = self.segments[leg[1]]
                route.append(("cross", [self.xy(leg[1], leg[2], leg[3]), self.xy(leg[1], 1 - leg[2], leg[3])],
                              s.width(leg[3]), leg[4] in ("crosswalk", "junction")))
            elif leg[0] == "to":
                route.append(("walk", [tuple(p) for p in leg[1]], 0.0, False))
            elif leg[0] == "stop":
                route.append(("stop", [tuple(leg[1])] if leg[1] else [], float(leg[2]), leg[3]))
            elif leg[0] == "wait":
                route.append(("wait", [], 0.0, leg[1]))
            elif leg[0] == "vanish":
                route.append(("vanish", [], leg[2] if len(leg) > 2 else None, leg[1] if len(leg) > 1 else ""))
        return route

    def add(self, w):
        """Spawns one walker of a plan (see the class doc); its index, or None when CARLA refused the spot."""
        carla = self.carla
        x, y = w["xy"] if "xy" in w else self.xy(w["seg"], w["side"], w["i"])
        if w.get("bp"):
            bp = self.lib.find(w["bp"])
        else:       # the place's look (walkerlooks.py), else a uniform pick
            from . import walkerlooks
            bp = walkerlooks.choose_blueprint(self.lib, self.rng, self.bps, plan=w)
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "false")
        if w.get("wheelchair") and bp.has_attribute("use_wheelchair"):
            bp.set_attribute("use_wheelchair", "true")
        bp.set_attribute("role_name", w.get("role", "crowd"))
        wp = nearest_waypoint(self.cmap, x, y, carla.LaneType.Any, carla=carla)
        z = wp.transform.location.z if wp is not None else 0.0      # the road's height there (geo.ground)
        at = carla.Location(x=x, y=y, z=z + 1.0)
        actor = self.world.try_spawn_actor(bp, carla.Transform(at, carla.Rotation(yaw=float(w["yaw"])))
                                           if "yaw" in w else carla.Transform(at))
        if w.get("wheelchair") and bp.has_attribute("use_wheelchair"):
            bp.set_attribute("use_wheelchair", "false")
        if actor is None:
            return None
        self.actors.append(actor)
        self.state.append({"route": self.route_of(w), "leg": 0, "pt": 0,
                           "speed": w["speed"] * float(w.get("speed_factor", 1.0)), "wait_from": None,
                           "crossing": False, "kinds": [leg[4] for leg in w["legs"] if leg[0] == "cross"],
                           "alive": True, "follow": w.get("follow"), "phone": bool(w.get("phone")),
                           "tag": w.get("tag", "crowd"), "standing": None, "stop_until": None, "pos": (x, y),
                           "heading": (1.0, 0.0), "cmd": 0.0, "hurry": 1.0, "hurry_until": -1.0,
                           "back_until": -1.0, "next_check": -1.0, "released": False, "plan": w})
        return len(self.actors) - 1

    def release(self, k):
        """A walker waiting at a ("wait", key) leg walks on (a bus came, its taxi stopped)."""
        if 0 <= k < len(self.state):
            self.state[k]["released"] = True

    def redirect(self, k, legs):
        """Walker k walks these legs (see route_of) from now on instead of the rest of its route."""
        st = self.state[k]
        st["route"] = self.route_of({"legs": legs})
        st["leg"], st["pt"], st["crossing"], st["wait_from"], st["stop_until"] = 0, 0, False, None, None

    def remove(self, k, reason="", occluder=None, force=False):
        """Takes walker k off the street (through a door, onto a bus, into a car) once nobody sees it go (hidden(),
        unless force): until then it stands where it is. True when it is gone."""
        st = self.state[k]
        if not st["alive"]:
            return True
        if not force and self.hidden is not None:
            x, y = st["pos"]
            try:
                unseen = self.hidden(x, y, occluder)
            except Exception:  # noqa: BLE001 - when in doubt, let it go
                unseen = True
            if not unseen:
                st["leaving"], st["crossing"] = (reason, occluder), False
                return False
        st["alive"], st["crossing"] = False, False
        actor = self.actors[k]
        try:
            actor.destroy()
        except RuntimeError:
            pass
        if self.on_vanish is not None:
            self.on_vanish(k, actor, reason)
        return True

    def tick(self):
        """Every tick (the walkers' own controls go every 4th): what moves with a walker (pushed bicycles)."""
        if self.crossings is not None:
            self.crossings.tick()

    def on_road(self):
        own = [a for a, st in zip(self.actors, self.state) if st["alive"] and st["crossing"]]
        return own + (self.crossings.on_road() if self.crossings is not None else [])

    def _vehicles(self):
        out, self._ids = [], []
        skip = self.crossings.not_vehicles if self.crossings is not None else ()
        for v in self.world.get_actors().filter("vehicle.*"):
            if v.id in skip:                      # a bicycle pushed by a crossing walker
                continue
            try:
                loc = v.get_location()
                sumo_v = self.velocities.get(v.id) if self.velocities else None
                vx, vy = sumo_v if sumo_v is not None else (lambda u: (u.x, u.y))(v.get_velocity())
            except RuntimeError:
                continue
            out.append((loc.x, loc.y, vx, vy))
            self._ids.append(v.id)
        return out

    def open_log(self, out):
        if self.decider is not None:
            self.decider.open_log(out)
        if self.crossings is not None:
            self.crossings.open_log(out)

    def close(self):
        if self.decider is not None:
            self.decider.close()
        if self.crossings is not None:
            self.crossings.close()

    def _stand(self, k, actor, st, kind, send, face=None):
        carla = self.carla
        send(actor, carla.WalkerControl(speed=0.0))
        st["moved"] = None                  # standing on purpose: the stuck clock starts again when it walks
        if face is not None and st.get("face") != face:   # turn to face (a shop window, the road), once
            st["face"] = face
            try:
                actor.set_transform(carla.Transform(actor.get_location(), carla.Rotation(yaw=face)))
            except (RuntimeError, AttributeError):
                pass
        st["cmd"] = 0.0
        if st["standing"] != kind:
            st["standing"] = kind
            if self.on_stand is not None:
                self.on_stand(k, actor, st, kind)

    def _walking(self, k, actor, st):
        if st["standing"] is not None:
            st["standing"], st["face"] = None, None
            if self.on_walk is not None:
                self.on_walk(k, actor, st)

    def _check_stuck(self, k, actor, st, t, loc, after_s=20.0):
        """A walker told to walk that has not moved 0.5 m in after_s: one crowd_stuck event (a wall, a prop, a car)."""
        mx, my, mt = st.get("moved") or (loc.x, loc.y, t)
        if (loc.x - mx) ** 2 + (loc.y - my) ** 2 > 0.25:
            st["moved"], st["stuck"] = (loc.x, loc.y, t), False
        else:
            st["moved"] = (mx, my, mt)
            if t - mt > after_s and not st.get("stuck"):
                st["stuck"] = True
                self.events.append(("crowd_stuck", f"{actor.id} {st['tag']} at {loc.x:.1f}, {loc.y:.1f} for "
                                                   f"{t - mt:.0f} s"))
                if st["tag"] != "crowd":       # a living-city walker that cannot walk on leaves the street
                    self.remove(k, "stuck")

    def _follow(self, k, actor, st, t, send):
        """A group member: beside its leader (offset right/back in the leader's heading), at the leader's pace."""
        carla = self.carla
        li, right, back = st["follow"]
        if not 0 <= li < len(self.state) or li == k:      # no such leader: walks its own route
            st["follow"] = None
            return
        lead = self.state[li]
        if not lead["alive"]:
            self.remove(k, "with its group")
            return
        loc = actor.get_location()
        st["pos"] = (loc.x, loc.y)
        hx, hy = lead["heading"]
        lx, ly = lead["pos"]
        tx, ty = lx - hx * back - hy * right, ly - hy * back + hx * right
        dx, dy = tx - loc.x, ty - loc.y
        d = math.hypot(dx, dy)
        if d > 3.0:              # fallen behind (a post, a wall at its side place): on the leader's trail instead
            tx, ty = lx - hx * 1.0, ly - hy * 1.0
            dx, dy = tx - loc.x, ty - loc.y
            d = math.hypot(dx, dy)
        st["crossing"] = lead["crossing"]
        if lead["cmd"] <= 0.0 and d < 1.5:
            self._stand(k, actor, st, lead["standing"] or "group", send,
                        face=lead.get("face") if lead["standing"] == "stop" else None)
            return
        self._walking(k, actor, st)
        self._check_stuck(k, actor, st, t, loc)
        speed = min(2.5, max(0.0, lead["cmd"]) + 0.8 * d) if d > 0.3 else lead["cmd"]
        if d < 1e-3:
            dx, dy, d = hx, hy, 1.0
        st["heading"], st["cmd"] = (dx / d, dy / d), speed
        send(actor, carla.WalkerControl(direction=carla.Vector3D(dx / d, dy / d, 0.0), speed=speed))

    def step(self, t):
        carla = self.carla
        send = self.send or (lambda actor, ctl: actor.apply_control(ctl))
        vehicles = None
        for k, (actor, st) in enumerate(zip(self.actors, self.state)):
            if not st["alive"]:
                continue
            try:
                if st.get("leaving") is not None:      # waits, standing, until it can go unseen
                    self._stand(k, actor, st, "leaving", send)
                    self.remove(k, *st["leaving"])
                    continue
                if st["follow"] is not None:
                    self._follow(k, actor, st, t, send)
                    continue
                if st["leg"] >= len(st["route"]):
                    self._stand(k, actor, st, "end", send)
                    continue
                kind, pts, width, marked = st["route"][st["leg"]]
                if kind == "vanish":
                    self.remove(k, marked, width)
                    continue
                if kind == "wait":
                    if st["released"]:
                        st["released"] = False
                        st["leg"], st["pt"] = st["leg"] + 1, 0
                    else:
                        self._stand(k, actor, st, "wait", send)
                    continue
                loc = actor.get_location()
                st["pos"] = (loc.x, loc.y)
                if kind == "stop" and st["pt"] >= len(pts):
                    if st["stop_until"] is None:
                        st["stop_until"] = t + width
                    if t < st["stop_until"]:
                        self._stand(k, actor, st, "stop", send, face=marked)
                        continue
                    st["stop_until"], st["face"] = None, None
                    st["leg"], st["pt"] = st["leg"] + 1, 0
                    continue
                if t < st["back_until"]:          # a step back from the kerb (reactions.py)
                    hx, hy = st["heading"]
                    send(actor, carla.WalkerControl(direction=carla.Vector3D(-hx, -hy, 0.0), speed=0.8))
                    continue
                if kind == "cross" and st["pt"] == 0:
                    # at the kerb: wait for a gap
                    if st["wait_from"] is None:
                        st["wait_from"] = t
                    if st["phone"] and t < st["next_check"]:     # looking at the phone, not at the road
                        self._stand(k, actor, st, "kerb", send)
                        continue
                    if vehicles is None:
                        vehicles = self._vehicles()
                    t_c = critical_gap(width, st["speed"], t - st["wait_from"])
                    light = self.signals.may_cross(pts[0], pts[1]) if self.signals is not None else None
                    if light is False or not (gap_ok(pts[0], pts[1], vehicles, t_c, marked) if self.decider is None
                                              else self.decider.decide(actor.id, t, pts[0], pts[1], t_c, vehicles,
                                                                       self._ids, marked)):
                        if st["phone"]:
                            st["next_check"] = t + self.PHONE_RECHECK_S
                        self._stand(k, actor, st, "kerb", send)
                        continue
                    seen = self.decider.last if self.decider is not None else {}
                    self.events.append(("crowd_crosses", f"{actor.id} {'marked' if marked else 'mid-block'} "
                                        f"{width:.1f} m after {t - st['wait_from']:.1f} s"
                                        f"{' on its light' if light else ''}"
                                        f"{' (phone)' if st['phone'] else ''}" + (
                                            f"; believed gap {_fmt(seen.get('believed_gap'))} s, true "
                                            f"{_fmt(seen.get('true_gap'))} s, {seen.get('looks', 0)} looks"
                                            if seen else "")))
                    st["pt"], st["crossing"], st["wait_from"] = 1, True, None
                tx, ty = pts[st["pt"]]
                dx, dy = tx - loc.x, ty - loc.y
                d = math.hypot(dx, dy)
                if d < 0.8:
                    st["pt"] += 1
                    if st["pt"] >= len(pts) and kind != "stop":
                        st["leg"], st["pt"], st["crossing"] = st["leg"] + 1, 0, False
                    continue
                if blocked(loc.x + 0.7 * dx / d, loc.y + 0.7 * dy / d, self.poses) and \
                        not blocked(loc.x, loc.y, self.poses, margin=0.0):
                    self._stand(k, actor, st, "blocked", send)     # a vehicle stands in the way: wait for it
                    continue
                # (a vehicle that drove onto the walker: it walks on out of it, not stand inside it: Tokyo clip, a
                # walker stood inside a bus for 2.6 s)
                self._walking(k, actor, st)
                self._check_stuck(k, actor, st, t, loc)
                speed = st["speed"] * (st["hurry"] if t < st["hurry_until"] else 1.0)
                st["heading"], st["cmd"] = (dx / d, dy / d), speed
                send(actor, carla.WalkerControl(direction=carla.Vector3D(dx / d, dy / d, 0.0), speed=speed))
            except RuntimeError:
                pass
        if self.crossings is not None:          # the crossing crowds, with the same batch and the same eyes
            cc = self.crossings
            cc.send, cc.hidden, cc.poses = send, self.hidden, self.poses
            cc.step(t)
            self.events.extend(cc.events)
            cc.events.clear()


def _fmt(x):
    return "-" if x is None else f"{x:.1f}"


def blocked(x, y, poses, margin=0.3):
    """Whether (x, y) lies within a SUMO vehicle's box (poses: the bridge's {id: (x, y, yaw, speed, half length,
    half width)}) grown by margin: a walker on the road does not walk into a car standing on its crossing."""
    for vx, vy, yaw, _sp, hl, hw in (poses or {}).values():
        if (x - vx) ** 2 + (y - vy) ** 2 > (hl + margin + 1.0) ** 2:
            continue
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        f, l = (x - vx) * c + (y - vy) * s, -(x - vx) * s + (y - vy) * c
        if abs(f) <= hl + margin and abs(l) <= hw + margin:
            return True
    return False


def make_crowd(world, geo, tc, mdir, centres, night, duration_s, log=_log, parts=None):
    """The trial's crowd: None without walkers, ai.Crowd (beside the roads, never crossing) with behaviour off, else
    a CrowdSim planned by prepare(). parts (run.py's participants): with traffic.walkers.window on and a driving
    participant, the walkers are planned in the drivers' windows and kept there (crowdwindow.py, sim.window)."""
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
    xodr_x = None
    if not (Path(mdir) / "map.osm").exists() and (Path(mdir) / "map.xodr").exists():
        try:        # a baked level has no map.osm: its OpenDRIVE's crosswalks and pedestrian signals instead
            from .parking import _xodr_points
            xp = _xodr_points(Path(mdir) / "map.xodr", world.get_map())
            xodr_x = [(c.centroid.x, -c.centroid.y) for c in xp["crossing"]]       # CARLA -> geo.net
        except Exception as err:  # noqa: BLE001 - then junction and mid-block crossings only, as before
            log(f"crowd: no OpenDRIVE crossings ({type(err).__name__}: {err})")
    from . import crowdwindow
    wcfg, region, starts = crowdwindow.settings(tc), None, ([], [])
    radius = float(wc.get("radius_m", 120))
    if wcfg["enabled"] and parts:
        starts = crowdwindow.anchors_at_start(parts, wcfg)
        if starts[0]:
            region = lambda segs: crowdwindow.initial_mask(segs, starts[0], starts[1], wcfg, radius)  # noqa: E731
    extend = {}
    try:
        params, segs, walkers, stats, att = prepare(tc, mdir, geo.net, centres_xy, duration_s, night, log,
                                                    crossings_xy=xodr_x, want_att=True, region=region,
                                                    extend=extend)
    except Exception as err:  # noqa: BLE001 - a map the planner cannot read: the crowd of before
        log(f"crowd: CROWD behaviour failed ({err!r}); walkers beside the roads instead")
        return legacy()
    living = None
    from . import livingcity    # groups, children, phones, wheelchairs, shop stops, doors (living_city on)
    lc = livingcity.settings(tc)
    if lc["enabled"]:
        try:
            info, outs, doors = livingcity.dress_plan(tc, lc, mdir, geo, segs, att, params, walkers, duration_s,
                                                      world, log)
            living = {"info": info, "outs": outs, "doors": doors, "att": att}
        except Exception as err:  # noqa: BLE001 - the crowd as planned rather than none
            log(f"living city: the crowd was not dressed ({type(err).__name__}: {err})")
    if (tc.get("map") or {}).get("carla_town"):     # a CARLA town: kerbs onto its pavements (walls, sunken roads)
        from . import towns
        towns.snap_kerbs(world.get_map(), segs, log)
    from . import walkerlooks      # who they are: walkers that look like the people of the place
    looks = walkerlooks.for_library(world.get_blueprint_library(), int(wc.get("seed", 1)), log)
    if looks is not None:
        looks.dress(walkers)
    sim = CrowdSim(world, geo, segs, walkers, params, int(wc.get("seed", 1)), log)
    walkerlooks.report(sim.actors, log)
    sim.stats = stats
    sim.extend = extend
    sim.living = living
    sim.window = None
    if region is not None and stats.get("walkers_per_kerb_km"):
        used = dict(params, crossings_per_km=stats["rate_used"], jaywalk_share=stats["jaywalk_used"])
        sim.window = crowdwindow.make(sim, segs, att, used, parts, wcfg, radius, stats["walkers_per_kerb_km"],
                                      duration_s, log, int(wc.get("seed", 1)), extend=extend)
        if sim.window is not None:
            log(f"crowd: rolling window around {len(sim.window.anchors)} driving participant(s): "
                f"{wcfg['ahead_m']:.0f} m of route ahead (+-{wcfg['band_m']:.0f} m), {wcfg['behind_m']:.0f} m "
                f"behind, else {radius:.0f} m around; walkers left behind reappear ahead")
    from . import embodiedped      # traffic.walkers.perception: crossings from what each walker sees (embodied-human)
    sim.decider = embodiedped.make(tc, sim.events, log)
    if sim.decider is not None:
        log("crowd: crossing decisions from what each walker sees (embodied-human eyes, vision, response)")
    return sim
