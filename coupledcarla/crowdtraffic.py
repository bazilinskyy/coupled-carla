"""The traffic of the place: SUMO's vehicle mix and density from the CROWD dashcam dataset, the models from
data/place_vehicles.json.

CROWD (City Road Observations With Dashcams; data/crowd_traffic.json, built by tools/crowd_traffic.py) counts the
unique cars, trucks, buses, motorcycles, bicycles and persons that YOLO tracks in each minute of dashcam footage of
over a thousand localities, by day and by night. Each trial's traffic (traffic.prepare) takes from it:

- the mix: the car stream (cars_per_hour: every motor vehicle but the buses) becomes cars, vans and lorries (CROWD's
  "truck" class, split by `van_share`), motorcycles and scooters, in CROWD's shares of the place, plus a few
  ambulances, police cars and fire engines (`service_share`, not in CROWD: COCO has no such class);
- the cyclists: bikes_per_hour "auto" = the car stream x CROWD's bicycles per motor vehicle of the place (the ratio of
  tracks is taken as the ratio of flows); open bike counters (cycling.py) still win where they reach;
- the random buses of a map without OSM bus lines: buses_per_hour "auto" = the car stream x buses per motor vehicle;
- the density: cars_per_hour "auto" (counts.py: lane-km by road class and the hour) x (the place's motor vehicles per
  minute / the median CROWD locality's) ** density_power, clipped to density_range.

Rates are shrunk towards the next level up (empirical Bayes: a locality's tracks plus `prior_min` minutes at its
country's rate, the country towards its continent, the continent towards the world), so Salamanca's single hour of
footage leans on Spain's. The night rates are used when the trial's hour is dark (CROWD's day/night label; a level
without an hour of night footage takes its overall rates times the world's night/overall ratio per class).

Limits: a dashcam count is a proxy for flow. Parked cars and bikes are tracked too, a slow dashcam sees more of
everything, and what the footage drove (main roads, highways) is not what a twin's streets carry; so the mix is
taken as the visible mix, the density only within density_range, and the hourly profile comes from counts.py.
"""
import json
import math
import random
from functools import lru_cache
from pathlib import Path

DATA = Path(__file__).resolve().parent / "data"
MOTOR = ("cars", "trucks", "buses", "motorcycles")
CLASSES = ("persons", "cars", "bicycles", "motorcycles", "buses", "trucks")

DEFAULTS = {
    "enabled": True,          # false: the experiment's car_types and rates only (and bikes_per_hour "auto" -> 1200)
    "prior_min": 120.0,       # minutes of footage the next level's rates count as
    "density": True,          # scale cars_per_hour "auto" by the place's density
    "density_power": 0.5,     # ... (place / median locality) ** this
    "density_range": [0.7, 1.4],
    "mix": True,              # motorcycles, vans, lorries and service vehicles in the car stream
    "van_share": 0.7,         # of CROWD's trucks (COCO "truck": vans, pickups, box vans, lorries): vans
    "service_share": 0.004,   # ambulances, police cars, fire engines in the car stream
    "bike_flow": 1.0,         # bikes per hour = car stream x CROWD bicycles per motor vehicle x this
    "night": "auto",          # "auto": night rates from 20:00 to 06:00 local (simulation.time); true / false
}


def settings(tc):
    return {**DEFAULTS, **((tc.get("traffic") or {}).get("crowd") or {})}


@lru_cache(maxsize=None)
def table():
    return json.loads((DATA / "crowd_traffic.json").read_text(encoding="utf-8"))


@lru_cache(maxsize=None)
def vehicles_table():
    return json.loads((DATA / "place_vehicles.json").read_text(encoding="utf-8"))


def _km(lat1, lon1, lat2, lon2):
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return 6371.0 * math.hypot(x, math.radians(lat2 - lat1))


def nearest_locality(lat, lon, iso3=None, max_km=25.0, t=None):
    """The nearest CROWD locality with footage within max_km (of the country iso3 when given), and its distance."""
    t = t or table()
    best, bd = None, max_km
    for loc in t["localities"]:
        if iso3 and loc["iso3"] != iso3:
            continue
        if abs(loc["lat"] - lat) > 0.5 or abs(loc["lon"] - lon) > 0.8:
            continue
        d = _km(lat, lon, loc["lat"], loc["lon"])
        if d < bd:
            best, bd = loc, d
    return best, bd


def levels(iso3, lat, lon, t=None):
    """[(label, entry)] from the finest to the world: the nearest locality (25 km), its country, continent, world."""
    t = t or table()
    out = []
    loc, km = nearest_locality(lat, lon, iso3, t=t)
    if loc:
        out.append((f"locality {loc['name']} ({km:.0f} km)", loc))
    country = t["countries"].get(iso3) if iso3 else None
    if country:
        out.append((f"country {country['name']}", country))
        cont = t["continents"].get(country.get("continent"))
        if cont:
            out.append((f"continent {country['continent']}", cont))
    out.append(("world", t["world"]))
    return out


def _part(entry, night, world):
    """(minutes, per_min) of an entry for day or night (None: all footage)."""
    if night is None:
        return entry["minutes"], entry["per_min"]
    key = "night" if night else "day"
    if entry.get(key):
        return entry[key]["minutes"], entry[key]["per_min"]
    wp, wa = (world.get(key) or world)["per_min"], world["per_min"]
    return entry["minutes"] * 0.5, {k: entry["per_min"][k] * (wp[k] / wa[k] if wa[k] else 1.0) for k in CLASSES}


def rates(lv, night=None, prior_min=120.0):
    """Tracks per minute of each class at the finest level, shrunk towards the coarser ones. Returns (rates,
    [(label, hours)] of the levels that had footage)."""
    world = lv[-1][1]
    _, r = _part(world, night, world)
    r = dict(r)
    used = [("world", round(world["minutes"] / 60, 1))]
    for label, e in reversed(lv[:-1]):
        m, pm = _part(e, night, world)
        if m <= 0:
            continue
        r = {k: (pm[k] * m + prior_min * r[k]) / (m + prior_min) for k in CLASSES}
        used.append((label, round(e["minutes"] / 60, 1)))
    return r, list(reversed(used))


def motor(r):
    return sum(r[k] for k in MOTOR)


def is_night(when, setting="auto"):
    if setting not in ("auto", None):
        return bool(setting)
    return when is not None and (when.hour >= 20 or when.hour < 6)


def plan_for(iso3, lat, lon, when=None, s=None):
    """What a place's traffic is: shares, ratios, density factor and where they came from (no CARLA, no SUMO)."""
    s = {**DEFAULTS, **(s or {})}
    t = table()
    night = is_night(when, s["night"])
    lv = levels(iso3, lat, lon, t)
    r, used = rates(lv, night, float(s["prior_min"]))
    med = t["median_locality"]["night" if night else "day"]
    mv = motor(r) or 1e-9
    lo, hi = s["density_range"]
    dens = min(hi, max(lo, (mv / motor(med)) ** float(s["density_power"]))) if motor(med) else 1.0
    stream = r["cars"] + r["trucks"] + r["motorcycles"]      # the car stream: every motor vehicle but buses
    van = float(s["van_share"])
    return {
        "iso3": iso3, "night": night, "levels": used,
        "rates": {k: round(v, 3) for k, v in r.items()},
        # shares of the car stream (cars_per_hour)
        "stream": {"cars": r["cars"] / stream, "vans": van * r["trucks"] / stream,
                   "lorries": (1 - van) * r["trucks"] / stream, "motorcycles": r["motorcycles"] / stream},
        "bikes_per_motor": r["bicycles"] / mv,
        "buses_per_stream": r["buses"] / stream,
        "persons_per_min": r["persons"],
        "density": round(dens, 3),
        "density_raw": round(mv / motor(med), 3) if motor(med) else None,
        # shares of every vehicle seen (the table in the README)
        "seen": {k: r[k] / (mv + r["bicycles"]) for k in ("cars", "trucks", "buses", "motorcycles", "bicycles")},
    }


def plan(cfg, bbox, when=None, cache=None):
    """plan_for() of the trial's map: its country as crowdpeds.locate finds it (city.country, nearest CROWD place,
    Nominatim)."""
    from . import crowdpeds
    iso3, how = crowdpeds.locate(cfg, bbox, cache)
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    p = plan_for(iso3, lat, lon, when, settings(cfg))
    p["located"] = how
    return p


# --- the models -----------------------------------------------------------------------------------------------------

def region_of(iso3):
    """The place_vehicles region name of a country ("world" when none lists it)."""
    for name, reg in vehicles_table()["regions"].items():
        if iso3 and iso3 in reg.get("countries", ()):
            return name
    return "world"


def place_vehicles(iso3, lat=None, lon=None):
    """The models of a place: the world entry, overridden by its region's keys, then its country's, then those of a
    listed place (a box: Manhattan's yellow cabs) that holds lat, lon."""
    vt = vehicles_table()
    out = {k: v for k, v in vt["regions"]["world"].items() if k != "countries"}
    reg = region_of(iso3)
    out.update({k: v for k, v in vt["regions"][reg].items() if k != "countries"})
    out.update(vt.get("countries", {}).get(iso3 or "", {}))
    out["region"] = reg
    if lat is not None and lon is not None:
        for pl in vt.get("places", []):
            s, w, n, e = pl["bbox"]
            if s <= lat <= n and w <= lon <= e:
                out.update({k: v for k, v in pl.items() if k not in ("name", "bbox", "source")})
                out["region"] = f"{reg}, {pl['name']}"
                break
    return out


def for_server(bp, major):
    """A 0.9.16 blueprint name as the server of CARLA `major` names it (compat.ALIASES' first that drives)."""
    if major < 1:
        return bp
    from . import compat, drivers
    sizes = drivers.sizes("1.0")
    if bp in sizes and sizes[bp].get("drives", True):
        return bp
    for c in compat.ALIASES.get(bp, []):
        if c in sizes and sizes[c].get("drives", True):
            return c
    return bp


def mix_types(p, pv, car_types, major=0, s=None):
    """{blueprint: (weight, vClass)} of the car stream: the cars (car_types, register or regional fleet) at the cars'
    share less taxis, taxis, motorcycles, vans, lorries and service vehicles at theirs."""
    s = {**DEFAULTS, **(s or {})}
    st = p["stream"]
    svc = float(s["service_share"])
    taxi = pv.get("taxi") or {}
    taxi_share = float(taxi.get("share", 0.0)) * st["cars"]
    out = {}

    def add(table_, share, vclass):
        tot = sum(table_.values()) or 1.0
        for bp, w in table_.items():
            bp = for_server(bp, major)
            if vclass in SUFFIX and not _is_taxi(bp):     # a taxi Kia is its own type (the bridge strips "#...")
                bp += SUFFIX[vclass]
            old = out.get(bp, (0.0, vclass))[0]
            out[bp] = (old + share * w / tot, vclass)
    if taxi.get("replace_register"):               # the street's taxis, not the register's few (Manhattan)
        car_types = {bp: w for bp, w in car_types.items() if not _is_taxi(bp)} or car_types
    elif any(_is_taxi(bp) for bp in car_types):    # a register with its taxis (NY DMV's yellow cabs) keeps them
        taxi_share = 0.0
    add(car_types, max(0.0, st["cars"] - taxi_share) * (1 - svc), "passenger")
    for bp in list(out):
        if _is_taxi(bp):
            out[bp] = (out[bp][0], "taxi")
    if taxi_share > 0:
        add(taxi.get("blueprints") or {"vehicle.ford.crown": 1}, taxi_share * (1 - svc), "taxi")
    add(pv["motorcycles"], st["motorcycles"] * (1 - svc), "motorcycle")
    add(pv["vans"], st["vans"] * (1 - svc), "delivery")
    add(pv["lorries"], st["lorries"] * (1 - svc), "truck")
    if svc > 0:
        add(pv["service"], svc, "emergency")
    return {bp: (round(100 * w, 3), c) for bp, (w, c) in out.items() if w > 0}


SUFFIX = {"taxi": "#taxi"}        # a model that is also a private car gets its own SUMO type as a taxi


def _is_taxi(bp):
    return "taxi" in bp or "ford.crown" in bp


def colors(pv, major=0):
    """{blueprint: "r,g,b"} for the vehicles whose colour is their livery (taxis, service vehicles)."""
    out = {}
    taxi = pv.get("taxi") or {}
    if taxi.get("color"):
        for bp in (taxi.get("blueprints") or {}):
            bp = for_server(bp, major)
            out[bp if _is_taxi(bp) else bp + SUFFIX["taxi"]] = ",".join(str(int(c)) for c in taxi["color"])
    for bp, c in (pv.get("service_colors") or {}).items():
        out[for_server(bp, major)] = ",".join(str(int(v)) for v in c)
    return out


def assign(rou_path, weights, net=None, seed=0, livery=None):
    """Gives every vehicle of type "cars" in a SUMO route file its own type drawn from weights ({blueprint: (weight,
    vClass)}), seeded; a vehicle whose route a lorry, bus or service vehicle may not drive (net: sumolib net) draws
    from the passenger types instead. Vehicles drawn already (randomTrips --validate: duarouter samples the
    distribution and routes each for its class) keep their type. Each colour becomes the livery's (taxi, ambulance)
    where there is one. Returns {vClass: vehicles}."""
    import xml.etree.ElementTree as ET
    rng = random.Random(seed + 7)
    tree = ET.parse(rou_path)
    root = tree.getroot()
    routes = {r.get("id"): r.get("edges", "").split() for r in root.iter("route") if r.get("id")}
    bps = list(weights)
    ws = [weights[b][0] for b in bps]
    light = [b for b in bps if weights[b][1] in ("passenger", "taxi", "motorcycle", "delivery")]
    lw = [weights[b][0] for b in light]
    allowed = {}

    def ok(edges, vclass):
        if net is None or vclass in ("passenger", "taxi", "motorcycle", "delivery"):
            return True
        for e in edges:
            key = (e, vclass)
            if key not in allowed:
                try:
                    allowed[key] = net.getEdge(e).allows(vclass)
                except KeyError:
                    allowed[key] = True
            if not allowed[key]:
                return False
        return True
    counts = {}
    for v in root.iter("vehicle"):
        if v.get("type") != "cars":     # drawn already (duarouter samples the distribution when it validates)
            bp = v.get("type")
            if bp in weights:
                counts[weights[bp][1]] = counts.get(weights[bp][1], 0) + 1
                if livery and bp in livery:
                    v.set("color", livery[bp])
            continue
        bp = rng.choices(bps, ws)[0]
        vclass = weights[bp][1]
        if vclass not in ("passenger", "taxi", "motorcycle", "delivery"):
            r = v.find("route")
            edges = r.get("edges", "").split() if r is not None else routes.get(v.get("route"), [])
            if not ok(edges, vclass) and light:
                bp = rng.choices(light, lw)[0]
                vclass = weights[bp][1]
        v.set("type", bp)
        if livery and bp in livery:
            v.set("color", livery[bp])
        counts[vclass] = counts.get(vclass, 0) + 1
    tree.write(rou_path, encoding="utf-8", xml_declaration=True)
    return counts


def describe(p):
    """One log line of a plan."""
    seen = p["seen"]
    return (f"{' < '.join(f'{lab} ({h:g} h)' for lab, h in p['levels'][:2])}"
            f"{' at night' if p['night'] else ''}: seen cars {100 * seen['cars']:.0f}%, trucks "
            f"{100 * seen['trucks']:.0f}%, buses {100 * seen['buses']:.1f}%, motorcycles {100 * seen['motorcycles']:.1f}%, "
            f"bicycles {100 * seen['bicycles']:.1f}%; density x{p['density']:.2f} (raw {p['density_raw']}), "
            f"{p['bikes_per_motor']:.3f} bikes per motor vehicle")
