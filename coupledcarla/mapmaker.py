"""Make a drivable map of any place in the world, for people who do not want to write YAML or read logs.

    python -m coupledcarla.mapmaker "Markt, Eindhoven"                 # find it, build it, show a summary
    python -m coupledcarla.mapmaker "51.4416, 5.4697" --size large --open
    python -m coupledcarla.mapmaker "Plaza Mayor, Salamanca" --drive    # then drive it (needs CARLA)
    python -m coupledcarla.mapmaker --web                              # the same as a page in the browser
    python -m coupledcarla.mapmaker "Plaza Mayor, Salamanca" --twin     # a baked CARLA 1.0 digital twin instead
    python -m coupledcarla.mapmaker "Dam, Amsterdam" --realistic        # the twin + Google's 3D tiles + live data
    python -m coupledcarla.mapmaker                                    # asks for the place (in a terminal)

From Python:

    from coupledcarla.mapmaker import make_map
    exp = make_map("Shibuya crossing", size="small")     # -> experiments/places/<name>.yaml, map in maps/<name>/

What it does for you: finds the place with OpenStreetMap's Nominatim (Photon as a fallback; keyless, at most one
request a second, cached), guesses what a non-expert cannot know (driving side from the country, traffic from the
size of the town, a readable name), writes a commented experiment file with the same city block as
experiments/cities/*.yaml, builds the map with coupledcarla.mapbuild while it reports each step in plain words
(the raw log goes to maps/<name>/build.log), then draws a preview picture (maps/<name>/preview.png) and writes a
summary with the data sources and their licences (maps/<name>/summary.json and summary.txt).

With --twin (and a digital-twin toolchain configured, coupledcarla.twinmaker) it makes a CARLA 1.0 digital twin of
the same square instead: carla-digitaltwins' TwinModel pipeline with the country's best open data, baked into the
CARLA 1.0 editor tree, and an experiment experiments/places/<name>_twin.yaml that drives it (map.opendrive,
map.carla_level), whose SUMO net is then built from the twin's OpenDRIVE.
"""
import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import unicodedata
from pathlib import Path

try:
    import yaml
except ImportError:      # run before the one-time setup
    sys.exit("The map maker needs its Python packages first: do the one-time setup in docs/make_a_map.md.")

ROOT = Path(__file__).resolve().parent.parent
PLACES = ROOT / "experiments" / "places"
CACHE = Path.home() / ".cache" / "coupled-carla" / "geocode.json"
NOMINATIM = "https://nominatim.openstreetmap.org"
PHOTON = "https://photon.komoot.io"

# Box sizes (side of the square, km). The 19 cities use 1.4 km.
SIZES = {"small": 1.0, "medium": 1.4, "large": 2.0}
SLOW_KM = 2.5            # above this the build and the CARLA world get slow
MAX_KM = 4.0             # above this Overpass and CARLA struggle: refused

# Traffic class by population (the 19 cities: Stellenbosch 0.2 M small, Lyon 0.5 M medium, Taipei 2.6 M medium,
# Sydney 5.3 M large). Demand per class comes from experiments/cities.yaml (size_traffic), as for the cities.
TOWN_CLASSES = [(3_000_000, "large"), (300_000, "medium"), (0, "small")]
TOWN_WORDS = {"large": "a big city", "medium": "a city", "small": "a town"}
PLACE_TYPE_CLASS = {"city": "medium", "town": "small", "village": "small", "hamlet": "small",
                    "isolated_dwelling": "small", "suburb": "medium", "quarter": "medium", "neighbourhood": "medium",
                    "borough": "medium", "municipality": "small"}

# Countries and territories that drive on the left (ISO 3166-1 alpha-2), after Wikipedia's "Left- and right-hand
# traffic" (list of countries and territories, checked 2026-11-15) and the CIA World Factbook. Everything else drives
# on the right. Samoa switched to the left in 2009; the US Virgin Islands are the only US territory on the left.
LEFT_HAND = set("""
    GB IE IM JE GG MT CY
    IN PK BD LK NP BT MV JP MY SG BN ID TH TL HK MO
    ZA NA BW ZW ZM MW MZ LS SZ KE UG TZ MU SC SH
    AU NZ FJ PG SB WS TO TV KI NR CK NU NF CX CC PN TK
    JM BS BB TT GY SR AG DM GD KN LC VC AI BM VG KY MS TC VI FK
""".split())

# Continent of each country (ISO alpha-2), for the city block (experiments/cities.yaml uses the same names).
_CONTINENTS = {
    "Africa": "DZ AO BJ BW BF BI CV CM CF TD KM CG CD CI DJ EG GQ ER SZ ET GA GM GH GN GW KE LS LR LY MG MW ML MR MU "
              "YT MA MZ NA NE NG RE RW SH ST SN SC SL SO ZA SS SD TZ TG TN UG EH ZM ZW",
    "Asia": "AF AM AZ BH BD BT BN KH CN GE HK IN ID IR IQ IL JP JO KZ KW KG LA LB MO MY MV MN MM NP KP OM PK PS PH QA "
            "SA SG KR LK SY TW TJ TH TL TR TM AE UZ VN YE IO",
    "Europe": "AL AD AT BY BE BA BG HR CY CZ DK EE FO FI FR DE GI GR GG HU IS IE IM IT JE XK LV LI LT LU MT MD MC ME "
              "NL MK NO PL PT RO RU SM RS SK SI ES SJ SE CH UA GB VA AX",
    "North America": "AI AG AW BS BB BZ BM BQ VG CA KY CR CU CW DM DO SV GL GD GP GT HT HN JM MQ MX MS NI PA PR BL "
                     "KN LC MF PM VC SX TT TC US VI UM",
    "South America": "AR BO BR CL CO EC FK GF GY PY PE SR UY VE GS",
    "Oceania": "AS AU CK FJ PF GU KI MH FM NR NC NZ NU NF MP PW PG PN WS SB TK TO TV VU WF CX CC",
    "Antarctica": "AQ",
}
CONTINENT = {cc: name for name, codes in _CONTINENTS.items() for cc in codes.split()}

CREDITS = {
    "nominatim": ("Place search: OpenStreetMap Nominatim (or Photon by Komoot), OpenStreetMap data", "ODbL 1.0"),
}


class MapMakerError(Exception):
    """A problem a non-expert can act on: `str(err)` says what happened, `err.advice` what to do."""

    def __init__(self, message, advice=""):
        super().__init__(message)
        self.advice = advice


# ------------------------------------------------------------------------------------------------ small helpers


def say(text):
    print(text, flush=True)


def size_km(size):
    """'small' | 'medium' | 'large' | a number of km (also as text) -> km. Raises MapMakerError when out of range."""
    if isinstance(size, str) and size.strip().lower() in SIZES:
        return SIZES[size.strip().lower()]
    try:
        km = float(str(size).lower().replace("km", "").strip())
    except ValueError:
        raise MapMakerError(f"I do not understand the size {size!r}.",
                            "Use small (about 1 km), medium (1.4 km), large (2 km) or a number of km such as 1.2.")
    if not 0.3 <= km <= MAX_KM:
        raise MapMakerError(f"A {km:g} km square is {'too small' if km < 0.3 else 'too large'}.",
                            f"Choose between 0.3 and {MAX_KM:g} km; most studies need 1 to 2 km.")
    return km


def size_name(km):
    return next((k for k, v in SIZES.items() if abs(v - km) < 1e-6), f"{km:g} km")


def bbox(lat, lon, km):
    """[south, west, north, east] of a km x km square around (lat, lon), as coupledcarla.cities does."""
    half = km / 2.0
    dlat = half / 111.32
    dlon = half / (111.32 * math.cos(math.radians(lat)))
    return [round(lat - dlat, 5), round(lon - dlon, 5), round(lat + dlat, 5), round(lon + dlon, 5)]


def estimate_minutes(km, roads=None):
    """A rough first-build time: downloads and scenery grow with the area (measured on lin4080: 1.4 km boxes took
    3-6 minutes, most of it waiting for the open data services)."""
    minutes = 1.5 + 2.5 * (km / 1.4) ** 2
    if roads:
        minutes += roads / 4000.0
    return max(1, round(minutes))


def parse_coords(text):
    """'51.44, 5.48' / '51.44 5.48' / '51.44N 5.48E' / '-33.9;18.4' -> (lat, lon), else None."""
    m = re.fullmatch(r"\s*(-?\d{1,2}(?:\.\d+)?)\s*°?\s*([NS])?\s*[,; ]\s*(-?\d{1,3}(?:\.\d+)?)\s*°?\s*([EWO])?\s*",
                     str(text or ""), re.I)
    if not m:
        return None
    lat, lon = float(m.group(1)), float(m.group(3))
    if (m.group(2) or "").upper() == "S":
        lat = -abs(lat)
    if (m.group(4) or "").upper() == "W":
        lon = -abs(lon)
    if not (-85 <= lat <= 85 and -180 <= lon <= 180):
        return None
    return lat, lon


def slugify(text, fallback="place"):
    """'Plaza Mayor, Salamanca' -> 'plaza_mayor_salamanca'; text without Latin letters gives the fallback."""
    t = unicodedata.normalize("NFKD", str(text or ""))
    t = "".join(c for c in t if not unicodedata.combining(c))
    t = t.replace("ß", "ss").replace("ø", "o").replace("Ø", "o").replace("æ", "ae").replace("ł", "l").replace("đ", "d")
    t = re.sub(r"[^a-z0-9]+", "_", t.encode("ascii", "ignore").decode().lower()).strip("_")
    t = re.sub(r"_+", "_", t)[:40].strip("_")
    return t if t and not t[0].isdigit() else (f"{fallback}_{t}" if t else fallback)


def parse_population(value):
    """OSM population tags: '9,586,195', '1 234 567', '12000;12500', '~5000' -> int, else None."""
    if value is None:
        return None
    m = re.search(r"\d[\d,.   ']*", str(value))
    if not m:
        return None
    digits = re.sub(r"\D", "", m.group(0).split(";")[0])
    return int(digits) if digits and int(digits) > 0 else None


def town_class(population=None, place_type=None):
    """('large' | 'medium' | 'small', how it was decided)."""
    if population:
        return next(c for floor, c in TOWN_CLASSES if population >= floor), f"~{population:,} people, from OpenStreetMap"
    if place_type in PLACE_TYPE_CLASS:
        return PLACE_TYPE_CLASS[place_type], f"OpenStreetMap calls it a {place_type}"
    return "medium", "size unknown, a medium city assumed"


def driving_side(country_code):
    return "left" if (country_code or "").upper() in LEFT_HAND else "right"


def traffic_for(town, km):
    """Traffic demand as experiments/cities.yaml gives the cities of this class, scaled from its 1.4 km box to this
    box by its side (vehicles enter across the box's edge)."""
    try:
        cat = yaml.safe_load((ROOT / "experiments" / "cities.yaml").read_text())
        base = dict(cat["defaults"]["traffic"])
        base.update(cat["size_traffic"].get(town, {}))
        ref = float(cat.get("size_km", 1.4))
    except (OSError, KeyError, TypeError, yaml.YAMLError):
        base = {"cars_per_hour": {"large": 3000, "small": 900}.get(town, 2000), "bikes_per_hour": 300,
                "buses_per_hour": {"large": 20, "small": 4}.get(town, 10)}
        ref = 1.4
    f = km / ref
    return {"cars_per_hour": int(round(base["cars_per_hour"] * f / 50.0) * 50),
            "bikes_per_hour": int(round(base["bikes_per_hour"] * f / 50.0) * 50),
            "buses_per_hour": max(1, int(round(base["buses_per_hour"] * f)))}


# ------------------------------------------------------------------------------------------------ geocoding

_lock = threading.Lock()
_last = [0.0]


def _load_cache():
    try:
        return json.loads(CACHE.read_text())
    except (OSError, ValueError):
        return {}


def _save_cache(cache):
    try:
        CACHE.parent.mkdir(parents=True, exist_ok=True)
        tmp = CACHE.with_suffix(".tmp")
        tmp.write_text(json.dumps(cache, ensure_ascii=False))
        tmp.replace(CACHE)
    except OSError:
        pass


def _fetch_json(url, params, timeout=20):
    """GET a JSON answer with coupled-carla's User-Agent, at most one request a second (Nominatim's usage policy).
    Only the place text or coordinates are sent: nothing about the user."""
    import requests
    from .realworld import UA
    with _lock:
        wait = 1.1 - (time.time() - _last[0])
        if wait > 0:
            time.sleep(wait)
        try:
            r = requests.get(url, params=params, headers=dict(UA, **{"Accept-Language": "en"}), timeout=timeout)
        finally:
            _last[0] = time.time()
    r.raise_for_status()
    return r.json()


def _cached(key, fetch):
    cache = _load_cache()
    if key in cache:
        return cache[key]
    data = fetch()
    cache = _load_cache()           # another thread may have written meanwhile
    cache[key] = data
    _save_cache(cache)
    return data


def _place_from_nominatim(r):
    a = r.get("address") or {}
    nd = r.get("namedetails") or {}
    ex = r.get("extratags") or {}
    settlement = next((a[k] for k in ("city", "town", "village", "municipality", "hamlet") if a.get(k)), None)
    name = nd.get("name:en") or r.get("name") or settlement or (r.get("display_name") or "").split(",")[0]
    kind = r.get("addresstype") or r.get("type")
    pop = parse_population(ex.get("population")) if r.get("category") in ("place", "boundary") else None
    label = name if not settlement or settlement == name or settlement in name else f"{name}, {settlement}"
    return {"name": label, "full": r.get("display_name", label), "lat": float(r["lat"]), "lon": float(r["lon"]),
            "country": a.get("country"), "country_code": (a.get("country_code") or "").upper() or None,
            "settlement": settlement, "kind": kind, "population": pop, "source": "OpenStreetMap Nominatim",
            "area": a.get("neighbourhood") or a.get("quarter") or a.get("suburb"),
            "boundary": int(r["osm_id"]) if r.get("category") == "boundary" and r.get("osm_type") == "relation" and
            str(r.get("osm_id", "")).isdigit() else None,
            "areas": [a[k] for k in ("city_district", "state_district", "county", "state") if a.get(k)]}


def _place_from_photon(f):
    p = f.get("properties") or {}
    lon, lat = f["geometry"]["coordinates"][:2]
    settlement = p.get("city") or p.get("town") or p.get("village")
    name = p.get("name") or p.get("street") or settlement or "this place"
    label = name if not settlement or settlement == name else f"{name}, {settlement}"
    full = ", ".join(x for x in (p.get("name"), p.get("street"), settlement, p.get("state"), p.get("country")) if x)
    kind = p.get("osm_value") if p.get("osm_key") == "place" else p.get("type")
    return {"name": label, "full": full or label, "lat": float(lat), "lon": float(lon), "country": p.get("country"),
            "country_code": (p.get("countrycode") or "").upper() or None, "settlement": settlement, "kind": kind,
            "areas": [p[k] for k in ("district", "county", "state") if p.get(k)],
            "population": None, "source": "Photon (Komoot), OpenStreetMap data"}


REGION_TYPES = {"country", "state", "region", "province", "state_district", "county", "district", "municipality",
                "archipelago", "island", "continent"}


def rank_places(res):
    """Nominatim's answers with towns, streets and squares before regions of the same name: "Chernihiv" is both a
    city and its oblast, and the oblast came first (its centre lies in fields: "No roads found here"). Stable."""
    def region(r):
        t = r.get("addresstype") or r.get("type")
        return 1 if t in REGION_TYPES or (r.get("category") == "boundary" and int(r.get("place_rank") or 30) < 14) \
            else 0
    return sorted(res, key=region)


def search(text, limit=5):
    """Places matching free text, best first: [{name, full, lat, lon, country, country_code, settlement, kind,
    population, source}]. Coordinates ('51.44, 5.48') are looked up in reverse. Raises MapMakerError when nothing
    is found or no service answers."""
    coords = parse_coords(text)
    if coords:
        return [reverse(*coords)]
    q = " ".join(str(text or "").split())
    if not q:
        raise MapMakerError("Type a place first.", "A town, a square, an address, or coordinates like 51.44, 5.48.")
    errors = []
    try:
        res = _cached(f"search:{q.lower()}:{limit}", lambda: _fetch_json(
            f"{NOMINATIM}/search", {"q": q, "format": "jsonv2", "limit": limit, "addressdetails": 1, "extratags": 1,
                                    "namedetails": 1, "accept-language": "en"}))
        if res:
            return [_place_from_nominatim(r) for r in rank_places(res)]
    except Exception as err:  # noqa: BLE001 - try the fallback
        errors.append(f"Nominatim: {err}")
    try:
        res = _cached(f"photon:{q.lower()}:{limit}", lambda: _fetch_json(
            f"{PHOTON}/api/", {"q": q, "limit": limit, "lang": "en"}))
        if res and res.get("features"):
            return [_place_from_photon(f) for f in res["features"]]
    except Exception as err:  # noqa: BLE001
        errors.append(f"Photon: {err}")
    if len(errors) == 2:
        raise MapMakerError("The place search did not answer.",
                            "Check the internet connection and try again, or type coordinates such as 51.44, 5.48 "
                            f"(details: {'; '.join(errors)}).")
    raise MapMakerError(f"I could not find \"{q}\".",
                        "Check the spelling, add the town or country (\"Markt, Eindhoven\"), or click on the map.")


def reverse(lat, lon):
    """The place at coordinates (country, town). Without an answer: the bare coordinates, country unknown."""
    key = f"reverse:{lat:.4f},{lon:.4f}"
    try:
        r = _cached(key, lambda: _fetch_json(f"{NOMINATIM}/reverse", {
            "lat": f"{lat:.6f}", "lon": f"{lon:.6f}", "format": "jsonv2", "zoom": 17, "addressdetails": 1,
            "extratags": 1, "namedetails": 1, "accept-language": "en"}))
        if r and "lat" in r and not r.get("error"):
            p = _place_from_nominatim(r)
            p["lat"], p["lon"] = lat, lon          # the square stays where it was asked for
            p["clicked"] = r.get("category") not in ("place", "boundary")   # named after its area, not a street
            return p
    except Exception:  # noqa: BLE001 - try the fallback
        pass
    try:
        r = _cached(f"photon-reverse:{lat:.4f},{lon:.4f}", lambda: _fetch_json(
            f"{PHOTON}/reverse", {"lat": f"{lat:.6f}", "lon": f"{lon:.6f}", "lang": "en"}))
        if r and r.get("features"):
            p = _place_from_photon(r["features"][0])
            p["lat"], p["lon"] = lat, lon
            return p
    except Exception:  # noqa: BLE001
        pass
    return {"name": f"{lat:.4f}, {lon:.4f}", "full": f"{lat:.5f}, {lon:.5f}", "lat": lat, "lon": lon,
            "country": None, "country_code": None, "settlement": None, "kind": None, "population": None,
            "source": "coordinates"}


def _km(lat1, lon1, lat2, lon2):
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return 6371.0 * math.hypot(x, math.radians(lat2 - lat1))


NEAR_KM = 40.0           # a town or area counts for the place only when its centre is this close


def town_centre(place):
    """(lat, lon) of a town whose search result is its boundary: Nominatim then gives the polygon's centroid, which
    can lie in the mountains or a lake (Hallstatt's lies 4 km from the village). The boundary's label or admin_centre
    node, else a place node of that name within 20 km, from Overpass; None when there is none or no answer."""
    from .mapbuild import OVERPASS
    rid, lat, lon = place.get("boundary"), place["lat"], place["lon"]
    if not rid:
        return None
    name = str(place.get("settlement") or place["name"].split(",")[0]).replace("\\", "").replace('"', "")
    q = (f'[out:json][timeout:20];rel({int(rid)})->.r;(node(r.r:"label");node(r.r:"admin_centre");'
         f'node["place"]["name"="{name}"](around:20000,{lat:.5f},{lon:.5f}););out body;')
    for url in OVERPASS[:2]:
        try:
            res = _cached(f"centre:{rid}", lambda: _fetch_json(url, {"data": q}, timeout=12))
        except Exception:  # noqa: BLE001 - the next mirror
            continue
        nodes = [el for el in (res or {}).get("elements", []) if "lat" in el and (el.get("tags") or {}).get("place")]
        if not nodes:
            return None
        best = min(nodes, key=lambda el: ((el.get("tags") or {}).get("name") != name,
                                          _km(lat, lon, el["lat"], el["lon"])))
        return float(best["lat"]), float(best["lon"])
    return None


TOWN_KM = 4.0            # the nearest town or village node stands in for a place whose address names none


def nearby_places(lat, lon):
    """From OSM's place nodes via Overpass: (the largest population of the cities whose reach covers the point,
    (name, population) of the nearest town, village or suburb within TOWN_KM or None), or None when Overpass did not
    answer. A city reaches 3 km x sqrt(population / 100,000), at most NEAR_KM: Tokyo's 14 M reach 36 km, so Shibuya
    (a city of its own in OSM, 0.23 M) counts as Tokyo; Cork's 0.2 M reach 4 km, so Kinsale 22 km away stays a town."""
    from .mapbuild import OVERPASS
    around = f"{lat:.5f},{lon:.5f}"
    q = (f'[out:json][timeout:20];(node["place"="city"]["population"](around:{NEAR_KM * 1000:.0f},{around});'
         f'node["place"~"^(town|village|suburb|hamlet)$"]["population"](around:{TOWN_KM * 1000:.0f},{around}););'
         f'out body qt;')
    for url in OVERPASS[:2]:
        try:
            res = _cached(f"places:{lat:.3f},{lon:.3f}", lambda: _fetch_json(url, {"data": q}, timeout=12))
        except Exception:  # noqa: BLE001 - the next mirror
            continue
        best, nearest = 0, None
        for el in (res or {}).get("elements", []):
            tags = el.get("tags") or {}
            pop = parse_population(tags.get("population"))
            try:
                d = _km(lat, lon, float(el["lat"]), float(el["lon"]))
            except (KeyError, TypeError, ValueError):
                continue
            if not pop:
                continue
            if tags.get("place", "city") == "city" and d <= min(NEAR_KM, 3.0 * math.sqrt(pop / 1e5)):
                best = max(best, pop)
            if d <= TOWN_KM and tags.get("name") and (nearest is None or d < nearest[2]):
                nearest = (tags.get("name:en") or tags["name"], pop, d)
        return best, (nearest[:2] if nearest else None)
    return None


def settlement_population(place):
    """Population of the town or city the place lies in (OSM's population tags), else None: the larger of its own
    town's (Nominatim; where the address names no town, the nearest town or village node, whose name then fills in
    place["settlement"]) and that of a big city whose reach covers it (nearby_places: Tokyo's wards are cities of
    their own in OSM). Without an Overpass answer, the larger named areas around the place (district, state) are
    looked up as settlements instead and count when their centre lies within NEAR_KM (OSM's "New Delhi" is the NDMC
    district, 0.25 M people; the city-state Delhi has many millions)."""
    if place.get("population"):
        return place["population"]
    got = nearby_places(place["lat"], place["lon"])
    near, town = got if got else (None, None)
    if town and not place.get("settlement"):
        place["settlement"] = town[0]
    else:
        town = None                       # the address's own town is looked up by name below
    names = []
    for n in [place.get("settlement")] + (list(place.get("areas") or []) if near is None else []):
        if n and n not in names:
            names.append(n)
    best = None
    for name in names:
        try:
            res = _cached(f"pop:{name.lower()}|{place.get('country_code') or ''}", lambda: _fetch_json(
                f"{NOMINATIM}/search", {"q": f"{name}, {place.get('country') or ''}".strip(", "), "format": "jsonv2",
                                        "limit": 3, "extratags": 1, "featureType": "settlement",
                                        "accept-language": "en"}))
        except Exception:  # noqa: BLE001 - the class falls back to the place type
            continue
        for r in res or []:
            pop = parse_population((r.get("extratags") or {}).get("population"))
            try:
                close = _km(place["lat"], place["lon"], float(r["lat"]), float(r["lon"])) <= NEAR_KM
            except (KeyError, TypeError, ValueError):
                close = False
            if pop and close:
                best = max(best or 0, pop)
        if best and best >= TOWN_CLASSES[0][0]:
            break
    if not best and town:
        best = town[1]
    return max(best or 0, near or 0) or None


# ------------------------------------------------------------------------------------------------ the plan


def plan(place=None, lat=None, lon=None, size="medium", name=None, pedestrians=False, found=None, twin=False,
         realistic=False):
    """Everything about a map before it is built: the place, the guesses and the experiment it will write.
    `found`: a place dict already looked up (from search or reverse). twin: a CARLA 1.0 digital twin instead
    (coupledcarla.twinmaker; experiments/places/<name>_twin.yaml). realistic: the twin plus
    experiments/places/<name>_real.yaml with Google's photorealistic tiles over it and the live data streams."""
    twin = twin or realistic
    km = size_km(size)
    if found is None:
        if lat is not None and lon is not None:
            found = reverse(float(lat), float(lon))
        elif place:
            found = search(place, limit=1)[0]
        else:
            raise MapMakerError("Which place?", "Give a place name, an address or coordinates.")
    if found.get("boundary"):            # a town found as its boundary: its centre, not the polygon's centroid
        centre = town_centre(found)
        found = dict(found, boundary=None, **({"lat": centre[0], "lon": centre[1]} if centre else {}))
    found = dict(found)
    lat, lon = float(found["lat"]), float(found["lon"])
    pop = settlement_population(found)
    town, why = town_class(pop, found.get("kind"))
    cc = found.get("country_code")
    side = driving_side(cc)
    if found.get("clicked"):             # a clicked point: "Centrum, Eindhoven" rather than a street's name
        area, sett = found.get("area"), found.get("settlement")
        found["name"] = (f"{area}, {sett}" if area and sett and area != sett else sett or area or found["name"])
    label = name or found["name"]
    point = (f"point_{abs(lat):.3f}{'n' if lat >= 0 else 's'}_{abs(lon):.3f}{'e' if lon >= 0 else 'w'}"
             .replace(".", "_"))
    base = point if (not name and parse_coords(found["name"])) else slugify(
        label, fallback=slugify(found.get("settlement") or "", point) if found.get("settlement") else point)
    slug = unique_slug(base, bbox(lat, lon, km))
    warnings = []
    if km > SLOW_KM:
        warnings.append(f"This area is very large ({km:g} km across): building takes ~{estimate_minutes(km)} minutes "
                        "and CARLA gets slow. A smaller square is usually enough.")
    if cc is None:
        warnings.append("I could not tell the country: right-hand traffic is assumed. Change it in the file if "
                        "needed.")
    return {"slug": slug, "label": label, "place": found, "lat": lat, "lon": lon, "km": km, "size": size_name(km),
            "bbox": bbox(lat, lon, km), "country": found.get("country"), "country_code": cc,
            "continent": CONTINENT.get(cc or "", "unknown"), "drive": side, "town": town, "town_why": why,
            "population": pop, "traffic": traffic_for(town, km), "pedestrians": bool(pedestrians),
            "minutes": estimate_minutes(km) + (twin_minutes(km) if twin else 0), "warnings": warnings,
            "twin": bool(twin), "realistic": bool(realistic),
            "experiment": str((PLACES / (f"{slug}_twin.yaml" if twin else f"{slug}.yaml")).relative_to(ROOT))}


def twin_minutes(km):
    """A twin's extra time: the national data, the real buildings and the bake grow with the area (1 km: ~35 min)."""
    from .twinmaker import TWIN_MINUTES
    return int(round(TWIN_MINUTES * max(1.0, km) ** 2))


def unique_slug(slug, box):
    """slug, or slug_2, slug_3, ... when that name is taken by another built map (of another box), a city or an
    experiment. The experiment file of a map that was never built (a failed attempt) does not take its name."""
    def free(s):
        for f in (ROOT / "experiments" / f"{s}.yaml", ROOT / "experiments" / "cities" / f"{s}.yaml"):
            if f.exists():
                return False
        built = (ROOT / "maps" / s / "meta.json").exists()
        for meta in (ROOT / "maps" / s / "meta.json", PLACES / f"{s}.yaml"):
            if meta.exists() and (built or meta.suffix == ".json"):   # a file of a map never built is reused
                try:
                    data = json.loads(meta.read_text()) if meta.suffix == ".json" else \
                        (yaml.safe_load(meta.read_text()) or {}).get("map", {})
                except (OSError, ValueError, yaml.YAMLError):
                    return False
                old = data.get("bbox")
                if not old or any(abs(float(a) - float(b)) > 1e-6 for a, b in zip(old, box)):
                    return False
        return True
    n, cand = 1, slug
    while not free(cand):
        n += 1
        cand = f"{slug}_{n}"
    return cand


def experiment(p):
    """The experiment dict for a plan, in the shape coupledcarla.cities writes."""
    exp = {
        "name": p["slug"],
        "city": {"name": p["label"], "country": p["country"] or "unknown", "continent": p["continent"],
                 "size": p["town"], "population": p["population"] or 0, "lat": round(p["lat"], 5),
                 "lon": round(p["lon"], 5), "drive": p["drive"], "country_code": p["country_code"]},
        "map": {"bbox": p["bbox"], "lefthand": p["drive"] == "left"},
        "traffic": dict(p["traffic"]),
        "participants": [{"role": "driver", "vehicle": "vehicle.lincoln.mkz_2020", "input": "keyboard",
                          "view": "driver"}],      # no spawn: a random spawn point of the map
        "trials": [{"duration_s": 300}],
    }
    if p["pedestrians"]:
        exp["traffic"]["walkers"] = {"count": "auto"}   # a crowd that walks and crosses like the country's people
    return exp


def _one_line(text):
    return " ".join(str(text or "").split())


def experiment_text(p, query=None):
    p = dict(p, label=_one_line(p["label"]), country=_one_line(p["country"]) or None)   # header lines stay comments
    query = _one_line(query) or None
    pop = f"~{p['population']:,} people" if p["population"] else p["town_why"]
    t = p["traffic"]
    head = [
        f"# {p['label']}{', ' + p['country'] if p['country'] else ''}: a {p['km']:g} km square ({p['size']}) around "
        f"{p['lat']:.5f}, {p['lon']:.5f}.",
        f"# Made by coupledcarla.mapmaker on {time.strftime('%Y-%m-%d')}"
        + (f" from \"{query}\"" if query else "") + f" (place found with {p['place']['source']}).",
        "# Guessed for you; change them here if you know better, then rebuild with --force:",
        f"#   driving side: {p['drive']}-hand traffic ({p['country'] or 'country unknown'})",
        f"#   traffic: {TOWN_WORDS[p['town']]} ({pop}): {t['cars_per_hour']} cars, {t['bikes_per_hour']} bikes and "
        f"{t['buses_per_hour']} buses an hour enter the box",
        f"# Build the map:  python -m coupledcarla.mapmaker --build {p['experiment']}",
        f"# Drive it:       python -m coupledcarla.run {p['experiment']}",
        "# A pedestrian at a station of their own: see experiments/eindhoven_crowd.yaml (participants, kind: "
        "pedestrian).",
    ]
    return "\n".join(head) + "\n" + yaml.safe_dump(experiment(p), sort_keys=False, allow_unicode=True)


def write_experiment(p, query=None):
    path = ROOT / p["experiment"]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(experiment_text(p, query), encoding="utf-8")
    return path


# ------------------------------------------------------------------------------------------------ building

STEPS = [
    ("place", "Finding the place"),
    ("streets", "Downloading the streets from OpenStreetMap"),
    ("terrain", "Measuring the hills and valleys"),
    ("roads", "Turning the streets into a drivable road network"),
    ("buildings", "Placing the buildings at their real heights"),
    ("trees", "Planting the trees"),
    ("furniture", "Adding street lamps, benches and signs"),
    ("ground", "Laying out parks, water and pavements"),
    ("parked", "Parking cars along the streets"),
    ("buses", "Finding the bus lines and timetables"),
    ("picture", "Drawing the preview picture"),
]
TITLES = dict(STEPS)


def _n(x):
    return f"{int(x):,}"


# (step, regex on a mapbuild log line without its "[mapbuild 12.3s] " prefix, "done" | "note", message)
RULES = [
    ("streets", r"OSM from \S+: ([\d.]+) MB", "done", None),          # message: road pieces counted in map.osm
    ("streets", r"OSM: reusing", "done", lambda m: "done (downloaded before)"),
    (None, r"Overpass \S+ failed|a partial answer|Overpass buildings \S+ failed", "note",
     lambda m: "OpenStreetMap's server is busy: trying a mirror…"),
    ("terrain", r"elevation \((\w+)\): (-?[\d.]+) to (-?[\d.]+) m", "note",
     lambda m: f"{float(m.group(3)) - float(m.group(2)):.0f} m from the lowest to the highest point"),
    ("terrain", r"elevation on (\d+) road nodes", "done", lambda m: f"done, heights on {_n(m.group(1))} road points"),
    ("roads", r"urban speeds", "note", lambda m: "speed limits where OpenStreetMap has none"),
    ("roads", r"pruning: (\d+) car edges -> (\d+) kept", "note",
     lambda m: f"{_n(m.group(2))} road pieces kept, dead ends trimmed"),
    ("roads", r"OpenDRIVE: (\d+) lane links, (\d+) waypoints, (\d+) lane ends, (\d+) traffic-light", "done",
     lambda m: f"done, {_n(m.group(1))} lane connections, {_n(m.group(4))} traffic lights"),
    ("roads", r"carla module not installed", "done", lambda m: "done (not checked: the carla package is missing)"),
    ("buildings", r"(Overture \S+|3DBAG|BD TOPO|NYC Open Data|PLATEAU): ([1-9]\d*) buildings", "note",
     lambda m: f"{_n(m.group(2))} buildings from {m.group(1).split()[0]}"),
    ("buildings", r"Overture failed|Overture skipped", "note",
     lambda m: "Overture Maps not available: OpenStreetMap's buildings instead"),
    ("buildings", r"scenery: (\d+) building meshes for (\d+) footprints", "done",
     lambda m: f"done, {_n(m.group(2))} buildings"),
    ("trees", r"px from \d+ of \d+ tiles", "note", lambda m: "sizing the trees from a canopy height map"),
    ("trees", r"scenery: (\d+) trees", "done", lambda m: f"done, {_n(m.group(1))} trees"),
    ("furniture", r"scenery: (\d+) pieces of street furniture, (\d+) lamps", "note",
     lambda m: f"{_n(m.group(2))} lamps, {_n(m.group(1))} benches, bins and other street furniture"),
    ("furniture", r"scenery: (\d+) road signs", "done", lambda m: f"done, {_n(m.group(1))} road signs"),
    ("ground", r"scenery: (\d+) ground tiles", "done", lambda m: f"done, {_n(m.group(1))} ground tiles"),
    ("parked", r"parked: (\d+) kerb stalls", "done", lambda m: f"done, {_n(m.group(1))} parking places at the kerb"),
    ("parked", r"parked: failed", "done", lambda m: "skipped (no parking data)"),
    ("buses", r"(\d+) OSM bus routes in the box; (\d+) matched onto the net \((\d+) line numbers\)", "note",
     lambda m: f"{_n(m.group(3))} bus lines"),
    ("buses", r"GTFS \S+ .*: (\d+) of (\d+) lines with a timetable", "note",
     lambda m: f"{m.group(1)} of {m.group(2)} lines with a real timetable"),
    ("buses", r"public transport failed", "note", lambda m: "no bus lines: buses drive random routes"),
    ("buses", r"^done: ", "done", None),
]
_RULES = [(s, re.compile(rx), kind, msg) for s, rx, kind, msg in RULES]
_ORDER = [s for s, _ in STEPS]
_PREFIX = re.compile(r"^\[[\w-]+(?:\s+[\d.]+s)?\]\s*")    # "[mapbuild  12.3s] ", "[transit 4.0s] ", "[canopy] "


def count_roads(osm_file):
    """OSM ways with a highway tag in an extract (cheap byte count)."""
    try:
        return Path(osm_file).read_bytes().count(b'k="highway"')
    except OSError:
        return 0


class Progress:
    """Turns mapbuild's log lines into plain-language steps. emit(event) gets {step, title, state: running | done |
    skipped | failed, text, at} for every change."""

    def __init__(self, emit, map_dir=None):
        self.emit, self.map_dir = emit, map_dir
        self.state = {s: "pending" for s in _ORDER}
        self.state["place"] = "done"            # make_map reports the place search itself
        self.text = {s: "" for s in _ORDER}
        self.current = None
        self.bus = {"lines": 0, "timetable": 0}
        self.t0 = time.time()

    def set(self, step, state, text=""):
        if state == "running" and self.current and self.current != step and self.state[self.current] == "running":
            self.set(self.current, "done", self.text[self.current] or "done")
        if state in ("running", "done") and step in _ORDER:   # every earlier step without news is done or skipped
            for s in _ORDER[:_ORDER.index(step)]:
                if self.state[s] == "pending":
                    self.set(s, "skipped", "not needed here")
        self.state[step], self.text[step] = state, text
        if state == "running":
            self.current = step
        self.emit({"step": step, "title": TITLES.get(step, step), "state": state, "text": text,
                   "at": round(time.time() - self.t0, 1)})

    def feed(self, line):
        msg = _PREFIX.sub("", line.strip())
        for step, rx, kind, fmt in _RULES:
            m = rx.search(msg)
            if not m:
                continue
            if step is None:                     # a note about whatever step is running
                step = self.current or "streets"
                if self.state[step] != "running":
                    return None
            if fmt is None and step == "streets":
                n = count_roads(self.map_dir / "map.osm") if self.map_dir else 0
                text = f"done, {n:,} streets and paths" if n else "done"
            elif fmt is None:
                text = "done"
            else:
                text = fmt(m)
            if step == "buses" and m.re.pattern.startswith("(\\d+) OSM bus routes"):
                self.bus["lines"] = int(m.group(3))
            elif step == "buses" and m.re.pattern.startswith("GTFS"):
                self.bus["timetable"] = max(self.bus["timetable"], int(m.group(1)))
                text = f"{self.bus['lines']} bus lines, {self.bus['timetable']} with a real timetable"
            if kind == "done" and step == "buses":
                text = (f"done, {self.bus['lines']} bus lines ({self.bus['timetable']} with a real timetable)"
                        if self.bus["lines"] else "done, no bus lines mapped here (buses drive random routes)")
            elif kind == "done" and step == "furniture" and self.text[step]:
                text = f"done, {self.text[step]}, {text[6:]}"
            if kind == "done":
                self.set(step, "done", text)
                nxt = _ORDER.index(step) + 1
                if nxt < len(_ORDER) and step != "buses":
                    self.set(_ORDER[nxt], "running", "")
            elif self.state[step] != "done":     # a late note about a finished step is only logged
                self.set(step, "running", text)
            return step
        return None


def explain_failure(log_text, returncode=1):
    """(message, advice) in plain words for a failed build, from its log."""
    t = log_text or ""
    if "no Overpass server answered" in t:
        if re.search(r"NameResolution|ConnectionError|Max retries|Temporary failure in name", t):
            return ("I cannot reach the internet.",
                    "Building a map downloads open data. Check the connection (Wi-Fi, VPN, proxy) and try again.")
        return ("OpenStreetMap's servers did not answer (all mirrors were busy).",
                "This happens at busy times. Wait a few minutes and press Build again; what was already "
                "downloaded is kept.")
    if re.search(r"No module named '?(\w+)", t):
        mod = re.search(r"No module named '?(\w+)", t).group(1)
        return (f"A part of the software is missing (the Python package \"{mod}\").",
                "Run the one-time setup again (docs/make_a_map.md, \"Set up once\").")
    if re.search(r"netconvert.*(not found|No such file)|FileNotFoundError.*netconvert", t):
        return ("SUMO (the traffic simulator) is not set up.",
                "Run the one-time setup again (docs/make_a_map.md, \"Set up once\"); it installs SUMO.")
    if re.search(r"No nodes loaded|No edges loaded|core of 0 junctions|-> 0 kept|"
                 r"max\(\) arg is an empty sequence|empty network|No valid edges", t):
        return ("There are no usable roads in this square.",
                "Is it in the sea, a desert or a park? Move the square onto a town, or make it larger.")
    if returncode in (-9, 137) or "MemoryError" in t or "Killed" in t:
        return ("The computer ran out of memory while building.",
                "Choose a smaller square (small or medium) and close other programs.")
    return ("Something went wrong while building the map.",
            "Try again once; if it fails again, send the build log (see below) to whoever set this up.")


def check_roads(box):
    """Number of roads in the box from Overpass (a quick count before the download), or None when no server
    answered (the build then finds out itself)."""
    import requests
    from .mapbuild import OVERPASS
    from .realworld import UA
    s, w, n, e = box
    q = (f'[out:json][timeout:25];way["highway"~"^(motorway|trunk|primary|secondary|tertiary|unclassified|'
         f'residential|living_street)(_link)?$"]({s},{w},{n},{e});out count;')
    for url in OVERPASS[:3]:
        try:
            r = requests.post(url, data={"data": q}, headers=UA, timeout=40)
            r.raise_for_status()
            tags = (r.json().get("elements") or [{}])[0].get("tags", {})
            return int(tags.get("ways", tags.get("total", 0)))
        except Exception:  # noqa: BLE001 - try the next mirror
            continue
    return None


def sumo_env():
    """The environment for mapbuild: SUMO_HOME, PATH and PROJ_LIB from the eclipse-sumo package when unset."""
    env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
    if not env.get("SUMO_HOME"):
        try:
            import sumo
            env["SUMO_HOME"] = sumo.SUMO_HOME
        except ImportError:
            return env
    home = env["SUMO_HOME"]
    env["PATH"] = os.pathsep.join([str(Path(home) / "bin"), env.get("PATH", "")])
    env.setdefault("PROJ_LIB", str(Path(home) / "data" / "proj"))
    return env


def build(exp_path, emit=None, force=False, check=True):
    """Build the map of an experiment file with plain-language progress. Returns the summary dict. Raises
    MapMakerError with advice when it fails; the raw log is in maps/<name>/build.log."""
    from . import config
    emit = emit or printer()
    exp_path = Path(exp_path)
    cfg = config.load(exp_path)
    mdir = config.map_dir(cfg)
    mdir.mkdir(parents=True, exist_ok=True)
    prog = Progress(emit, mdir)
    already = (mdir / "meta.json").exists() and not force
    if check and not already and not (mdir / "map.osm").exists() and cfg["map"].get("bbox"):
        prog.set("streets", "running", "checking that there are roads here")
        n = check_roads(cfg["map"]["bbox"])
        if n == 0:
            prog.set("streets", "failed", "no roads here")
            if exp_path.resolve().parent == PLACES.resolve():      # nothing to keep of a square without roads
                exp_path.unlink(missing_ok=True)
                shutil.rmtree(mdir, ignore_errors=True)
            raise MapMakerError("No roads found here: is this in the sea or in a park?",
                                "Try a place name in a town, or move the square onto streets.")
        if n:
            prog.set("streets", "running", f"{n:,} streets found")
    else:
        prog.set("streets", "running", "")
    log_path = mdir / "build.log"
    cmd = [sys.executable, "-u", "-m", "coupledcarla.mapbuild", str(exp_path)] + (["--force"] if force else [])
    with open(log_path, "a", encoding="utf-8") as logf:
        logf.write(f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} {' '.join(cmd)}\n")
        logf.flush()
        try:
            proc = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                    encoding="utf-8", errors="replace", env=sumo_env(), bufsize=1)
        except OSError as err:
            raise MapMakerError("The map builder could not start.", f"Run the one-time setup again ({err}).")
        tail = []
        for line in proc.stdout:
            logf.write(line)
            logf.flush()
            tail = (tail + [line])[-400:]
            if "already built" in line:
                prog.set("streets", "done", "built before (use --force to build it again)")
            prog.feed(line)
        proc.wait()
    if proc.returncode != 0 or not (mdir / "meta.json").exists():
        msg, advice = explain_failure("".join(tail), proc.returncode)
        cur = prog.current or "streets"
        prog.set(cur, "failed", msg)
        raise MapMakerError(msg, f"{advice} The full build log is {log_path}.")
    for s in _ORDER[:-1]:
        if prog.state[s] in ("pending", "running"):
            prog.set(s, "done" if prog.state[s] == "running" else "skipped",
                     prog.text[s] or ("done" if prog.state[s] == "running" else "not available here"))
    prog.set("picture", "running", "")
    summ = summary(mdir, cfg)
    try:
        title = cfg.get("city", {}).get("name") or cfg["name"]
        sub = (f"{summ['roads_km']:.1f} km of road, {summ['buildings']:,} buildings, {summ['trees']:,} trees; "
               f"{summ['drive']}-hand traffic; © OpenStreetMap contributors")
        from . import preview
        preview.picture(mdir, mdir / "preview.png", title=title, subtitle=sub, bbox=json.loads(
            (mdir / "meta.json").read_text()).get("bbox"))
        summ["preview"] = str(mdir / "preview.png")
        prog.set("picture", "done", "done")
    except Exception as err:  # noqa: BLE001 - the map is fine without its picture
        prog.set("picture", "skipped", f"no picture ({type(err).__name__}: {err}; pip install matplotlib)")
    (mdir / "summary.json").write_text(json.dumps(summ, indent=1, ensure_ascii=False), encoding="utf-8")
    (mdir / "summary.txt").write_text(summary_text(summ), encoding="utf-8")
    return summ


def summary(mdir, cfg=None):
    """What a built map contains, in numbers, with its data sources and their licences."""
    mdir = Path(mdir)
    meta = json.loads((mdir / "meta.json").read_text())
    roads_km = None
    try:
        import sumolib
        net = sumolib.net.readNet(str(mdir / "geo.net.xml"), withInternal=False, withPedestrianConnections=False)
        roads_km = sum(e.getLength() for e in net.getEdges() if e.allows("passenger")) / 1000.0
    except Exception:  # noqa: BLE001 - sumolib missing or the net unreadable
        pass
    heights = []
    if (mdir / "buildings.json").exists():
        heights = sorted(b.get("h", 0) for b in json.loads((mdir / "buildings.json").read_text()))
    sc = meta.get("scenery") or {}
    tr = meta.get("transit") or {}
    el = meta.get("elevation") or {}
    city = (cfg or {}).get("city") or {}
    sources = dict(meta.get("sources") or {})
    sources.setdefault("osm", {"what": "OpenStreetMap contributors via the Overpass API", "licence": "ODbL 1.0"})
    sources["nominatim"] = {"what": CREDITS["nominatim"][0], "licence": CREDITS["nominatim"][1]}
    if (cfg or {}).get("simulation", {}).get("weather") == "real":
        sources.setdefault("open-meteo", {"what": "Open-Meteo.com weather API", "licence": "CC BY 4.0"})
    return {
        "name": meta.get("name"), "label": city.get("name") or meta.get("name"), "country": city.get("country"),
        "country_code": city.get("country_code"),
        "bbox": meta.get("bbox"), "built": meta.get("built"),
        "drive": "left" if meta.get("lefthand") else "right",
        "roads_km": round(roads_km, 1) if roads_km is not None else 0.0,
        "lane_links": (meta.get("xodr") or {}).get("lane_links"),
        "traffic_lights": (meta.get("xodr") or {}).get("tl_signals"),
        "buildings": int(meta.get("buildings") or len(heights)),
        "building_height_m": [round(heights[0], 1), round(heights[len(heights) // 2], 1), round(heights[-1], 1)]
        if heights else None,
        "trees": int(sc.get("tree", 0)),
        "lamps": int(sc.get("lamp", 0)), "furniture": int(sc.get("furniture", 0)), "signs": int(sc.get("sign", 0)),
        "parking_places": int(sc.get("parked_kerb_stalls", 0)) + int(sc.get("parked_lot_stalls", 0)),
        "bus_lines": int(tr.get("refs") or 0), "bus_lines_with_timetable": int(tr.get("with_timetable") or 0),
        "ground_m": [el.get("min_m"), el.get("max_m")] if el else None,
        "relief_m": round(el["max_m"] - el["min_m"]) if el else None,
        "terrain_source": el.get("source"),
        "traffic": (cfg or {}).get("traffic", {}) and {k: (cfg or {})["traffic"].get(k) for k in
                                                       ("cars_per_hour", "bikes_per_hour", "buses_per_hour")},
        "sources": sources,
        "warnings": ([f"Only {roads_km:.1f} km of road here connect into a drivable network (dead ends are removed, "
                      "so a car can always drive on). A larger square or a spot with more streets gives more."]
                     if roads_km is not None and roads_km < 2.0 else []),
        "map_dir": str(mdir),
        "experiment": (cfg or {}).get("_path"),
        "twin": twin_summary(cfg, mdir),
    }


def twin_summary(cfg, mdir):
    """The twin lines of a map built from a baked level (map.carla_level), else []."""
    if not ((cfg or {}).get("map") or {}).get("carla_level"):
        return []
    try:
        from . import twinmaker
        return twinmaker.summary_lines(cfg, mdir)
    except Exception:  # noqa: BLE001 - the summary is fine without them
        return []


def summary_text(s):
    lines = [f"{s['label']}{', ' + s['country'] if s.get('country') and s['country'] != 'unknown' else ''}",
             f"  Roads: {s['roads_km']:.1f} km ({s['drive']}-hand traffic, {s.get('traffic_lights') or 0} traffic "
             f"lights)"]
    if s.get("building_height_m"):
        lo, mid, hi = s["building_height_m"]
        lines.append(f"  Buildings: {s['buildings']:,} (typically {mid:.0f} m tall, the tallest {hi:.0f} m)")
    else:
        lines.append(f"  Buildings: {s['buildings']:,}")
    lines.append(f"  Trees: {s['trees']:,}; street lamps: {s['lamps']:,}; benches, bins and other furniture: "
                 f"{s['furniture']:,}; road signs: {s['signs']:,}")
    if s.get("bus_lines"):
        lines.append(f"  Bus lines: {s['bus_lines']} ({s['bus_lines_with_timetable']} with a real timetable)")
    if s.get("relief_m") is not None:
        lines.append(f"  Ground: {s['ground_m'][0]:.0f} to {s['ground_m'][1]:.0f} m above sea level "
                     f"({s['relief_m']} m of hills)")
    lines += list(s.get("twin") or [])
    for w in s.get("warnings") or []:
        lines.append(f"  Note: {w}")
    lines.append("  Made from (free, open data; please credit them when you publish pictures or results):")
    for k, v in sorted(s["sources"].items()):
        lines.append(f"    - {v['what']} ({v['licence']})")
    lines.append(f"  Files: {s['map_dir']}")
    return "\n".join(lines) + "\n"


def printer(stream=None):
    """An emit() that prints the steps in plain words, one line per step."""
    stream = stream or sys.stdout
    shown = {}

    def emit(ev):
        key, state, text = ev["step"], ev["state"], ev["text"]
        if state == "running":
            if key not in shown:
                print(f"{ev['title']}…" + (f" {text}" if text else ""), file=stream, flush=True)
                shown[key] = text
            elif text and text != shown[key]:
                print(f"    {text}", file=stream, flush=True)
                shown[key] = text
        elif state == "done":
            if key not in shown:
                print(f"{ev['title']}… {text}", file=stream, flush=True)
            else:
                print(f"    {text}", file=stream, flush=True)
            shown[key] = "done"
        elif state == "failed":
            print(f"{ev['title']}… FAILED: {text}", file=stream, flush=True)
    return emit


# ------------------------------------------------------------------------------------------------ the one call


def make_map(place=None, lat=None, lon=None, size="medium", name=None, pedestrians=False, build_it=True,
             force=False, emit=None, twin=False, realistic=False):
    """Make a drivable map of a place. Returns the experiment file (experiments/places/<name>.yaml); the map is built
    into maps/<name>/ with preview.png, summary.json and summary.txt next to it.

    place: free text ("Markt, Eindhoven", "Shibuya crossing", an address, "51.44, 5.48"), or lat/lon.
    size: "small" (about 1 km), "medium" (1.4 km), "large" (2 km) or a number of km.
    name: a readable name for the map (default: the place's name).
    pedestrians: a crowd of pedestrians on the pavements that behaves like the country's people.
    build_it: False only writes the experiment file (not with twin).
    twin: a baked CARLA 1.0 digital twin instead (needs the toolchain of coupledcarla.twinmaker); returns
    experiments/places/<name>_twin.yaml.
    realistic: the twin, and experiments/places/<name>_real.yaml with Google's Photorealistic 3D Tiles over it (your
    own key on the server) and every live data stream on: the sun and weather of now, traffic fitted to open counts,
    the real buses where they are now, cyclists from counters, a calibrated crowd."""
    twin = twin or realistic
    emit = emit or printer()
    t0 = time.time()
    emit({"step": "place", "title": TITLES["place"], "state": "running", "text": "", "at": 0.0})
    p = plan(place, lat, lon, size, name, pedestrians, twin=twin, realistic=realistic)
    emit({"step": "place", "title": TITLES["place"], "state": "done",
          "text": f"{p['place']['full']} ({p['drive']}-hand traffic, {TOWN_WORDS[p['town']]})",
          "at": round(time.time() - t0, 1)})
    for w in p["warnings"]:
        emit({"step": "place", "title": TITLES["place"], "state": "note", "text": w, "at": 0.0})
    if twin:
        return make_twin_map(p, emit, force=force)
    path = write_experiment(p, query=place if place else f"{lat}, {lon}")
    if build_it:
        build(path, emit, force=force)
    return path


def make_twin_map(p, emit, force=False):
    """The twin of a plan (coupledcarla.twinmaker), then its SUMO net from the twin's OpenDRIVE. Returns the
    experiment file. Raises MapMakerError with advice."""
    from . import twinmaker
    try:
        path = twinmaker.make_twin(p, emit, force=force)
    except twinmaker.TwinError as err:
        raise MapMakerError(str(err), err.advice)
    build(path, emit, force=force, check=False)
    return path


# ------------------------------------------------------------------------------------------------ driving


CARLA_EXES = ("CarlaUE4.sh", "CarlaUE4.exe", "CarlaUnreal.sh", "CarlaUnreal.exe")


def find_carla():
    """The CARLA folder: CARLA_ROOT, else a CARLA_0.9.16 folder beside this project, in the home folder, in Downloads
    or at C:\\CARLA_0.9.16 (where people unpack it). None when there is none."""
    if os.environ.get("CARLA_ROOT"):
        return os.environ["CARLA_ROOT"]
    home = Path.home()
    for d in (ROOT.parent / "CARLA_0.9.16", home / "CARLA_0.9.16", home / "carla" / "CARLA_0.9.16",
              home / "Downloads" / "CARLA_0.9.16", Path("C:/CARLA_0.9.16")):
        if any((d / n).exists() for n in CARLA_EXES):
            return str(d)
    return None


def check_setup():
    """Missing packages the map maker needs, as a list (empty: all there)."""
    import importlib.util
    return [m for m in ("requests", "sumolib", "numpy", "networkx", "pyproj")
            if importlib.util.find_spec(m) is None]


def carla_status():
    """Whether this computer can drive a map: {ok, why, how}. CARLA 0.9.16 runs on Windows and Linux with an
    NVIDIA (or recent AMD) GPU; the carla Python package must match it."""
    import importlib.util
    if sys.platform == "darwin":
        return {"ok": False, "why": "CARLA does not run on a Mac.",
                "how": "Copy this folder (with maps/ and experiments/places/) to a Windows or Linux PC with a gaming "
                       "graphics card and CARLA 0.9.16, then run the command below there."}
    root = find_carla()
    if not root:
        return {"ok": False, "why": "CARLA is not set up on this computer.",
                "how": "Download CARLA 0.9.16 from https://github.com/carla-simulator/carla/releases/tag/0.9.16 and "
                       "unpack it as a folder CARLA_0.9.16 next to this project's folder (or set CARLA_ROOT to it), "
                       "then press Drive it again or run the command below."}
    exe = [n for n in CARLA_EXES if (Path(root) / n).exists()]
    if not exe:
        return {"ok": False, "why": f"No CARLA program in CARLA_ROOT ({root}).",
                "how": "Point CARLA_ROOT at the unpacked CARLA 0.9.16 folder (it holds CarlaUE4.sh or CarlaUE4.exe)."}
    if importlib.util.find_spec("carla") is None:
        return {"ok": False, "why": "The carla Python package is missing.",
                "how": "Run the one-time setup again (pip install carla==0.9.16)."}
    return {"ok": True, "why": f"CARLA found ({exe[0]})", "how": "", "root": root}


def drive_command(exp_path, headless=False):
    rel = Path(exp_path).resolve()
    try:
        rel = rel.relative_to(ROOT)
    except ValueError:
        pass
    cmd = [sys.executable, "-m", "coupledcarla.run", str(rel)]
    if headless:
        cmd += ["--input", "autopilot", "--headless", "--view", "chase", "--duration", "40", "--frames-every", "10"]
    return cmd


def shown_command(cmd):
    """A command as the user would type it: the project's .venv Python by its relative path, else "python"."""
    exe = Path(cmd[0])
    try:
        first = str(exe.relative_to(ROOT)) if exe.is_absolute() else cmd[0]
    except ValueError:
        first = "python"
    return " ".join([first] + [f'"{c}"' if " " in c else c for c in cmd[1:]])


def drive(exp_path, wait=True):
    """Start a drive of the map (keyboard: W/S/A/D or the arrows, Esc ends). Returns the process, or None with the
    reason printed when CARLA is not set up."""
    st = carla_status()
    if not st["ok"]:
        say(f"Cannot drive here: {st['why']}\n{st['how']}\n    {shown_command(drive_command(exp_path))}")
        return None
    cmd = drive_command(exp_path)
    log = ROOT / "logs" / f"drive_{Path(exp_path).stem}.log"
    log.parent.mkdir(exist_ok=True)
    say("Starting CARLA and the traffic; a window opens in about a minute. Drive with W/S/A/D or the arrow keys, "
        f"Esc to stop. (Log: {log})")
    env = dict(sumo_env(), CARLA_ROOT=st["root"])
    with open(log, "w") as logf:
        proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=logf, stderr=subprocess.STDOUT)
    if wait:
        proc.wait()
    return proc


def built_maps():
    """Maps made here and built, newest first: [{name, label, country, built, experiment, preview, summary}]."""
    out = []
    for exp in sorted(PLACES.glob("*.yaml")) if PLACES.exists() else []:
        try:
            cfg = yaml.safe_load(exp.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError):
            continue
        name = (cfg.get("map") or {}).get("name") or cfg.get("name") or exp.stem
        mdir = ROOT / "maps" / name
        if not (mdir / "meta.json").exists():          # written but never built (or a failed build)
            continue
        city = cfg.get("city") or {}
        summ = None
        if (mdir / "summary.json").exists():
            try:
                summ = json.loads((mdir / "summary.json").read_text(encoding="utf-8"))
            except ValueError:
                pass
        out.append({"name": name, "label": city.get("name") or name, "country": city.get("country"),
                    "drive": city.get("drive"), "built": (summ or {}).get("built"),
                    "ready": (mdir / "meta.json").exists(), "experiment": str(exp.relative_to(ROOT)),
                    "preview": (mdir / "preview.png").exists(), "summary": summ,
                    "mtime": (mdir / "meta.json").stat().st_mtime if (mdir / "meta.json").exists() else exp.stat().st_mtime})
    return sorted(out, key=lambda m: -m["mtime"])


def open_file(path):
    path = str(path)
    try:
        if sys.platform == "darwin":
            subprocess.Popen(["open", path])
        elif os.name == "nt":
            os.startfile(path)  # noqa: S606 - opening the user's own picture
        elif shutil.which("xdg-open"):
            subprocess.Popen(["xdg-open", path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            import webbrowser
            webbrowser.open(Path(path).as_uri())
    except OSError as err:
        say(f"(could not open {path}: {err})")


# ------------------------------------------------------------------------------------------------ command line


def _ask(prompt, default=""):
    try:
        ans = input(prompt).strip()
    except EOFError:
        return default
    return ans or default


def interactive():
    say("Make a map of any place in the world.\n")
    while True:
        q = _ask("Which place? (a name, an address, or coordinates like 51.44, 5.48): ")
        if not q:
            return None
        try:
            found = search(q, limit=5)
        except MapMakerError as err:
            say(f"  {err} {err.advice}")
            continue
        for i, f in enumerate(found, 1):
            say(f"  {i}. {f['full']}")
        pick = _ask("Which one? [1]: ", "1")
        if pick.isdigit() and 1 <= int(pick) <= len(found):
            break
    size = _ask("How big? small (1 km), medium (1.4 km) or large (2 km) [medium]: ", "medium")
    return {"found": found[int(pick) - 1], "size": size, "query": q}


def main(argv=None):
    ap = argparse.ArgumentParser(description="Make a drivable map of any place in the world.",
                                 epilog="Examples: \"Markt, Eindhoven\" | \"51.4416, 5.4697\" --size large | --web")
    ap.add_argument("place", nargs="*", help="a place name, an address or coordinates (lat, lon)")
    ap.add_argument("--size", default="medium", help="small (~1 km), medium (1.4 km), large (2 km) or km")
    ap.add_argument("--name", help="a name for the map (default: the place's name)")
    ap.add_argument("--pedestrians", action="store_true", help="add a crowd of pedestrians on the pavements")
    ap.add_argument("--drive", action="store_true", help="drive the map when it is built (needs CARLA)")
    ap.add_argument("--open", action="store_true", help="show the preview picture when it is built")
    ap.add_argument("--force", action="store_true", help="build again even if built before")
    ap.add_argument("--no-build", action="store_true", help="only write the experiment file")
    ap.add_argument("--build", metavar="EXPERIMENT", help="build an experiment file written before")
    ap.add_argument("--twin", action="store_true",
                    help="make a baked CARLA 1.0 digital twin instead (needs the toolchain: docs/make_a_map.md)")
    ap.add_argument("--realistic", action="store_true",
                    help="the digital twin plus Google's photorealistic tiles over it and live data (your own key)")
    ap.add_argument("--web", action="store_true", help="open the map maker page in the browser")
    ap.add_argument("--port", type=int, default=8777, help="port of the page (--web), on 127.0.0.1 only")
    ap.add_argument("--no-browser", action="store_true", help="with --web: do not open a browser")
    args = ap.parse_args(argv)
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    missing = check_setup()
    if missing:
        say(f"The map maker needs some Python packages that are not installed ({', '.join(missing)}).\n"
            "Do the one-time setup in docs/make_a_map.md (\"Set up once\"), then start it again.")
        return 3
    if args.web:
        from . import mapmaker_web
        return mapmaker_web.serve(args.port, open_browser=not args.no_browser)
    try:
        if args.build:
            exp = Path(args.build)
            build(exp, force=args.force)
        else:
            query = " ".join(args.place).strip()
            found = None
            if not query:
                if not sys.stdin.isatty():
                    ap.error("give a place, e.g. python -m coupledcarla.mapmaker \"Markt, Eindhoven\"")
                got = interactive()
                if not got:
                    return 1
                found, args.size, query = got["found"], got["size"], got["query"]
            t0 = time.time()
            emit = printer()
            emit({"step": "place", "title": TITLES["place"], "state": "running", "text": "", "at": 0.0})
            args.twin = args.twin or args.realistic
            if args.twin:
                from . import twinmaker
                st = twinmaker.status()
                if not st["ok"]:
                    raise MapMakerError(f"Cannot make a digital twin here: {st['why']}", st["how"])
                if not st["bake"]:
                    say(f"Note: {st['why']}. {st['how']}")
            p = plan(query, size=args.size, name=args.name, pedestrians=args.pedestrians, found=found, twin=args.twin,
                     realistic=args.realistic)
            emit({"step": "place", "title": TITLES["place"], "state": "done",
                  "text": f"{p['place']['full']}", "at": 0.0})
            say(f"    {p['drive']}-hand traffic ({p['country'] or 'country unknown'}); {TOWN_WORDS[p['town']]} "
                f"({p['town_why']}): {p['traffic']['cars_per_hour']} cars an hour; a {p['km']:g} km square")
            for w in p["warnings"]:
                say(f"    Note: {w}")
            if args.twin:
                say(f"A digital twin takes about {p['minutes']} minutes (most of it the national data and the "
                    "bake).")
                exp = make_twin_map(p, emit, force=args.force)
                say(f"    experiment file: {exp.relative_to(ROOT)}")
                if args.realistic:
                    say(f"    as real as it gets: {(PLACES / (p['slug'] + '_real.yaml')).relative_to(ROOT)} "
                        "(Google's photorealistic tiles with your key on the server, live data)")
            else:
                exp = write_experiment(p, query)
                say(f"    experiment file: {exp.relative_to(ROOT)}")
                if args.no_build:
                    return 0
                say(f"Building takes about {p['minutes']} minutes the first time.")
                build(exp, emit, force=args.force)
            say(f"\nReady in {(time.time() - t0) / 60:.1f} minutes.")
        from . import config
        cfg = config.load(exp)
        mdir = config.map_dir(cfg)
        if (mdir / "summary.txt").exists():
            say("\n" + (mdir / "summary.txt").read_text(encoding="utf-8"))
        if (mdir / "preview.png").exists():
            say(f"Preview picture: {mdir / 'preview.png'}")
            if args.open:
                open_file(mdir / "preview.png")
        say(f"Drive it: {shown_command(drive_command(exp))}")
        if args.drive:
            drive(exp)
        return 0
    except MapMakerError as err:
        say(f"\n{err}\n{err.advice}")
        return 2
    except KeyboardInterrupt:
        say("\nStopped.")
        return 130


if __name__ == "__main__":
    sys.exit(main())
