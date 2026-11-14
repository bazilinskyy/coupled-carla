"""Shared bikes and scooters (GBFS): parked where they really are, and cyclists starting at the docks.

Feeds. MobilityData's catalogue of GBFS systems (systems.csv on GitHub, keyless) names each system's city; the
systems whose city is the map's (or whose area is its whole country) are asked for their stations
(station_information + station_status: where the docks are and how many bikes each holds) and their free-floating
vehicles (free_bike_status in GBFS 2, vehicle_status in 3), with vehicle_types for the kind of each (bicycle,
scooter, moped; car-sharing cars are left out). Only what lies in the map box is kept. Two big docked systems publish
the same in their own keyless format and are read too: TfL's BikePoint (London, Santander Cycles) and Taipei's
YouBike 2.0. Systems whose data are more than STALE_DAYS old (an operator that left the city) are dropped. The
snapshot is cached in maps/<name>/real/gbfs.json; offline or a failing feed: the cached snapshot or none, with one
log line.

Scenery (`map.scenery.shared`). Each dock gets its available bikes (up to `max_per_station`) side by side,
square to the street, and each free-floating vehicle stands at its reported position, parallel to the street;
a spot on a lane or inside a building moves up to 3 m away from the road, else the vehicle is left out. At map
build time the snapshot of that moment is used; a trial with `simulation.time: now` takes a fresh snapshot (older
than LIVE_MAX_AGE_S) and puts today's bikes in place of the built ones. CARLA has no static mesh of a shared bike
or a kick scooter: bikes are CARLA's road bike (SM_RoadBike, the static mesh of the vehicle.bh.crossbike family;
CARLA 1.0's town props have an e-bike, SM_Ebike, used once tools/mesh_catalog.py has measured it), mopeds and
seated scooters the Vespa (SM_Vespa), and kick scooters the Vespa at 0.6 scale, the closest there is. Their sizes
were read from the cooked meshes' vertex buffers of CARLA 0.9.16 (the same method gives SM_BikeParking and
SM_MailBox exactly as the catalogue measured them on a server); CARLA 1.0 has the same meshes at the same paths.

Cyclists (`traffic.cycling.dock_weight`). Bike edges within DOCK_RADIUS_M of a dock draw (1 + dock_weight) times
as many trip starts and ends in cycling.py's demand: shared-bike trips begin and end at docks.
"""
import csv
import hashlib
import io
import json
import math
import re
import time
from datetime import datetime, timezone
from pathlib import Path

import requests

from . import realworld

SYSTEMS = "https://raw.githubusercontent.com/MobilityData/gbfs/master/systems.csv"
CACHE = Path.home() / ".cache" / "coupled-carla" / "gbfs"
MAX_AGE_DAYS = 7              # a map's snapshot is fetched again after this long (map builds, cyclists)
LIVE_MAX_AGE_S = 600          # a trial at time "now" fetches again when the snapshot is older than this
STALE_DAYS = 2                # feeds whose last update is older than this are dropped
MARGIN_DEG = 0.001            # docks and vehicles this far outside the box still count (about 100 m)
DOCK_RADIUS_M = 30.0          # bike edges this close to a dock draw its trips (the street it stands on)
SPACING_M = 0.8               # bikes side by side in a dock

SOURCES = {
    "systems": ("MobilityData GBFS systems catalogue (github.com/MobilityData/gbfs, systems.csv)", "CC BY 3.0"),
    "tfl": ("Transport for London, BikePoint API (Santander Cycles docks)",
            "TfL open data terms (OGL v2 based), powered by TfL Open Data"),
    "youbike": ("Taipei City, YouBike 2.0 station status (data.taipei)",
                "Open Government Data License, version 1.0 (Taiwan)"),
}

# form factor (GBFS vehicle_types) -> [(mesh, scale)], first one the catalogue has; None: not placed
MESHES = {
    "bicycle": [("SM_Ebike", 1.0), ("SM_RoadBike", 1.0)],
    "cargo_bicycle": [("SM_RoadBike", 1.1)],
    "moped": [("SM_Vespa", 1.0)],
    "scooter_seated": [("SM_Vespa", 1.0)],
    "scooter_standing": [("SM_Vespa", 0.6)],
    "scooter": [("SM_Vespa", 0.6)],
    "car": None, "other": None,
}
# sizes of the meshes the catalogues lack (until tools/mesh_catalog.py measures them on a server): half extents
# and centre offset in metres, from the LOD0 vertex buffers of CARLA 0.9.16's cooked meshes
EXTRA_MESHES = {
    "SM_RoadBike": ("/Game/Carla/Static/Bicycle/Roadbike/SM_RoadBike.SM_RoadBike",
                    {"cat": "furniture", "ext": [0.82, 0.19, 0.51], "off": [0.0, 0.0, 0.53]}),
    "SM_Vespa": ("/Game/Carla/Static/Motorcycle/Vespa/SM_Vespa.SM_Vespa",
                 {"cat": "furniture", "ext": [0.9, 0.43, 0.62], "off": [0.01, 0.0, 0.62]}),
}


def log(msg, t0=[time.time()]):
    print(f"[gbfs       {time.time() - t0[0]:6.1f}s] {msg}", flush=True)


def _get(url, timeout=60):
    r = requests.get(url, headers=realworld.UA, timeout=timeout)
    r.raise_for_status()
    return r


def _json(url, timeout=60):
    return _get(url, timeout).json()


def _in_box(lat, lon, bbox, margin=MARGIN_DEG):
    try:
        lat, lon = float(lat), float(lon)
    except (TypeError, ValueError):
        return False
    s, w, n, e = bbox
    return s - margin <= lat <= n + margin and w - margin <= lon <= e + margin


# ---------------------------------------------------------------- the catalogue

def systems_csv():
    path = CACHE / "systems.csv"
    if not path.exists() or time.time() - path.stat().st_mtime > 7 * 86400:
        CACHE.mkdir(parents=True, exist_ok=True)
        path.write_bytes(_get(SYSTEMS).content)
    return list(csv.DictReader(io.StringIO(path.read_text(encoding="utf-8"))))


def _tokens(text):
    import unicodedata
    s = unicodedata.normalize("NFKD", str(text or "")).encode("ascii", "ignore").decode().lower()
    return [t for t in re.split(r"[^a-z]+", s) if len(t) > 2]


def place_names(cfg):
    """Lower-case names the map's place goes by: the city's name (without what is in brackets), the experiment
    name's first word (eindhoven_centre -> eindhoven)."""
    names = set()
    city = (cfg.get("city") or {}).get("name") or ""
    base = re.sub(r"\(.*?\)", "", city).split(",")[0].strip()
    if base:
        names.add(" ".join(_tokens(base)))
    for n in (cfg.get("name"), (cfg.get("map") or {}).get("name")):
        if n:
            names.add(re.split(r"[_\W]+", str(n).lower())[0])
    return {n for n in names if n}


def candidates(rows, names, country=None):
    """systems.csv rows whose location's city is one of the names (or whose location is the whole country), that
    need no key."""
    out, seen = [], set()
    for r in rows:
        url = (r.get("Auto-Discovery URL") or "").strip()
        if (r.get("Authentication Type") or "").strip() or not url.startswith("http") or url in seen:
            continue
        loc = " ".join(_tokens((r.get("Location") or "").split(",")[0]))
        if loc in names or (country and loc == " ".join(_tokens(country))):
            out.append(r)
            seen.add(url)     # one feed listed under the city and the country (docomo: Tokyo and Japan)
    return out


# ---------------------------------------------------------------- GBFS feeds

def feed_urls(discovery):
    """gbfs.json -> {feed name: url}, GBFS 2 (feeds per language, English first) or 3."""
    data = discovery.get("data") or {}
    feeds = data.get("feeds")
    if feeds is None and data:
        lang = "en" if "en" in data else sorted(data)[0]
        feeds = (data.get(lang) or {}).get("feeds") or []
    return {f.get("name"): f.get("url") for f in feeds or [] if f.get("name") and f.get("url")}


def updated(doc):
    """A feed's last_updated as a UTC datetime (POSIX seconds in GBFS 2, ISO 8601 in 3), or None."""
    v = doc.get("last_updated")
    try:
        if isinstance(v, (int, float)):
            return datetime.fromtimestamp(v, timezone.utc)
        if isinstance(v, str):
            return datetime.fromisoformat(v.replace("Z", "+00:00"))
    except (ValueError, OSError):
        pass
    return None


def _name(v):
    if isinstance(v, list):     # GBFS 3: [{text, language}]
        return (v[0] or {}).get("text") if v else None
    return v


def parse_system(feeds, bbox, now=None):
    """The docks and free-floating vehicles of one system in the box, from its parsed feeds
    ({name: document}): ({stations, vehicles, licence}, last update or None)."""
    now = now or datetime.now(timezone.utc)
    types = {}
    for t in ((feeds.get("vehicle_types") or {}).get("data") or {}).get("vehicle_types", []):
        types[t.get("vehicle_type_id")] = t.get("form_factor") or "other"
    info = (feeds.get("system_information") or {}).get("data") or {}
    licence = info.get("license_id") or info.get("license_url") or "the operator's GBFS terms (no licence given)"
    if types and all(MESHES.get(ff) is None for ff in types.values()):      # car sharing
        return {"stations": [], "vehicles": [], "licence": licence}, None
    status = {}
    for s in ((feeds.get("station_status") or {}).get("data") or {}).get("stations", []):
        status[s.get("station_id")] = s
    stations = []
    for s in ((feeds.get("station_information") or {}).get("data") or {}).get("stations", []):
        lat, lon = s.get("lat"), s.get("lon")
        if not _in_box(lat, lon, bbox):
            continue
        st = status.get(s.get("station_id"), {})
        if st.get("is_installed") in (0, False) or st.get("is_renting") in (0, False):
            continue
        n = st.get("num_vehicles_available", st.get("num_bikes_available"))
        by_type = {}
        for vt in st.get("vehicle_types_available") or []:
            ff = types.get(vt.get("vehicle_type_id"), "bicycle")
            by_type[ff] = by_type.get(ff, 0) + int(vt.get("count") or 0)
        if not by_type and n:
            by_type = {"bicycle": int(n)}
        if by_type and all(MESHES.get(ff) is None for ff in by_type):
            continue                 # a car-sharing station
        stations.append({"lat": round(float(lat), 6), "lon": round(float(lon), 6), "name": _name(s.get("name")),
                         "capacity": s.get("capacity"), "available": int(n or 0), "types": by_type})
    vehicles = []
    doc = feeds.get("vehicle_status") or feeds.get("free_bike_status") or {}
    data = doc.get("data") or {}
    for v in data.get("vehicles", data.get("bikes", [])) or []:
        if v.get("station_id") or v.get("is_reserved") in (1, True) or v.get("is_disabled") in (1, True):
            continue
        lat, lon = v.get("lat"), v.get("lon")
        if not _in_box(lat, lon, bbox):
            continue
        ff = types.get(v.get("vehicle_type_id"), "bicycle" if not types else "other")
        if MESHES.get(ff) is None:
            continue
        vehicles.append({"lat": round(float(lat), 6), "lon": round(float(lon), 6), "form_factor": ff})
    stamps = [updated(d) for d in (feeds.get("station_status"), feeds.get("vehicle_status"),
                                   feeds.get("free_bike_status")) if d]
    stamps = [t for t in stamps if t]
    return {"stations": stations, "vehicles": vehicles, "licence": licence}, (max(stamps) if stamps else None)


def fetch_system(row, bbox, now=None):
    urls = feed_urls(_json(row["Auto-Discovery URL"]))
    feeds = {}
    for name in ("system_information", "vehicle_types", "station_information", "station_status", "vehicle_status",
                 "free_bike_status"):
        if name in urls:
            try:
                feeds[name] = _json(urls[name], 120)
            except Exception:  # noqa: BLE001 - an optional feed missing: the others still count
                if name in ("station_information", "station_status", "vehicle_status", "free_bike_status"):
                    raise
    return parse_system(feeds, bbox, now)


def parse_tfl(places, bbox):
    out = []
    for p in places:
        lat, lon = p.get("lat"), p.get("lon")
        if not _in_box(lat, lon, bbox):
            continue
        props = {a.get("key"): a.get("value") for a in p.get("additionalProperties") or []}
        if props.get("Installed") == "false" or props.get("Locked") == "true":
            continue
        bikes, docks_n = int(props.get("NbBikes", 0) or 0), int(props.get("NbDocks", 0) or 0)
        st = {"lat": round(lat, 6), "lon": round(lon, 6), "name": p.get("commonName"), "capacity": docks_n,
              "available": bikes, "types": {"bicycle": bikes}}
        if bikes + int(props.get("NbEmptyDocks", 0) or 0) == 0 and docks_n > 0:
            # no bikes and no free docks: the occupancy is unknown (all 792 docks said so on 13-14 November 2026):
            # half full, the typical fill
            st.update(available=docks_n // 2, types={"bicycle": docks_n // 2}, estimated=True)
        out.append(st)
    return out


def parse_youbike(rows, bbox):
    out = []
    for r in rows:
        try:
            lat, lon = float(r["latitude"]), float(r["longitude"])
        except (KeyError, TypeError, ValueError):
            continue
        if str(r.get("act", "1")) != "1" or not _in_box(lat, lon, bbox):
            continue
        n = int(r.get("available_rent_bikes") or 0)
        out.append({"lat": round(lat, 6), "lon": round(lon, 6), "name": r.get("snaen") or r.get("sna"),
                    "capacity": int(r.get("Quantity") or 0), "available": n, "types": {"bicycle": n}})
    return out


def _london(bbox):
    s, w, n, e = bbox
    return 51.28 <= (s + n) / 2 <= 51.70 and -0.52 <= (w + e) / 2 <= 0.34


def _taipei(bbox):
    s, w, n, e = bbox
    return 24.96 <= (s + n) / 2 <= 25.21 and 121.45 <= (w + e) / 2 <= 121.67


OWN_FORMATS = [   # (source, covers(bbox), name, fetch -> stations)
    ("tfl", _london, "Santander Cycles", lambda b: parse_tfl(_json("https://api.tfl.gov.uk/BikePoint", 120), b)),
    ("youbike", _taipei, "YouBike 2.0", lambda b: parse_youbike(
        _json("https://tcgbusfs.blob.core.windows.net/dotapp/youbike/v2/youbike_immediate.json", 120), b)),
]


def fetch(cfg, bbox, log=log, now=None):
    """All systems with docks or vehicles in the box -> snapshot dict (see snapshot)."""
    now = now or datetime.now(timezone.utc)
    country = (cfg.get("city") or {}).get("country") or ("Netherlands" if realworld.in_netherlands(bbox) else None)
    rows = candidates(systems_csv(), place_names(cfg), country)
    systems, notes, seen = [], [], set()
    for r in rows:
        try:
            res, stamp = fetch_system(r, bbox, now)
        except Exception as err:  # noqa: BLE001 - one broken feed: the others still count
            notes.append(f"{r['Name']}: {type(err).__name__}")
            continue
        if stamp is not None and (now - stamp).total_seconds() > STALE_DAYS * 86400:
            notes.append(f"{r['Name']}: stale since {stamp.date()}")
            continue
        dup = [st for st in res["stations"] if (round(st["lat"], 5), round(st["lon"], 5)) in seen]
        res["stations"] = [st for st in res["stations"] if st not in dup]
        seen.update((round(st["lat"], 5), round(st["lon"], 5)) for st in res["stations"])
        if dup and not res["stations"] and not res["vehicles"]:
            notes.append(f"{r['Name']}: the same docks as a system before it")
            continue
        if res["stations"] or res["vehicles"]:
            systems.append({"id": r.get("System ID"), "name": r["Name"], "url": r["Auto-Discovery URL"],
                            "licence": res["licence"], "stations": res["stations"], "vehicles": res["vehicles"]})
        else:
            notes.append(f"{r['Name']}: nothing in the box")
    for src, covers, name, get in OWN_FORMATS:
        if covers(bbox):
            try:
                st = get(bbox)
            except Exception as err:  # noqa: BLE001
                notes.append(f"{name}: {type(err).__name__}")
                continue
            if st:
                systems.append({"id": src, "name": name, "url": SOURCES[src][0], "licence": SOURCES[src][1],
                                "stations": st, "vehicles": []})
    return {"fetched": datetime.now().isoformat(timespec="seconds"), "candidates": [r["Name"] for r in rows],
            "notes": notes, "systems": systems}


def snapshot(cfg, map_dir, bbox, max_age_s=MAX_AGE_DAYS * 86400, log=log):
    """The map's shared bikes and scooters: {fetched, candidates, notes, systems: [{id, name, url, licence, stations:
    [{lat, lon, name, capacity, available, types}], vehicles: [{lat, lon, form_factor}]}]}, cached in
    real/gbfs.json; fetched again when older than max_age_s. Offline: the cached snapshot, or None."""
    path = Path(map_dir) / "real" / "gbfs.json"
    old = json.loads(path.read_text()) if path.exists() else None
    if old:
        age = (datetime.now() - datetime.fromisoformat(old.get("fetched", "2000-01-01"))).total_seconds()
        if age < max_age_s:
            return old
    try:
        snap = fetch(cfg, bbox, log)
    except Exception as err:  # noqa: BLE001 - offline: what we have
        log(f"shared bikes: GBFS not reached ({type(err).__name__}: {err})"
            + ("; the cached snapshot is used" if old else ""))
        return old
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(snap, indent=1, ensure_ascii=False))
    st = sum(len(s["stations"]) for s in snap["systems"])
    bikes = sum(x["available"] for s in snap["systems"] for x in s["stations"])
    veh = sum(len(s["vehicles"]) for s in snap["systems"])
    log(f"shared bikes: {', '.join(s['name'] for s in snap['systems']) or 'no system'} in the box: {st} docks with "
        f"{bikes} bikes, {veh} free-floating vehicles" + (f" ({'; '.join(snap['notes'])})" if snap["notes"] else ""))
    return snap


# ---------------------------------------------------------------- scenery

def ensure_meshes(cat):
    """Adds the measured sizes of EXTRA_MESHES to a scenery.Catalog that lacks them."""
    for name, (path, m) in EXTRA_MESHES.items():
        if name not in cat.by_name:
            cat.by_name[name] = (path, m)


def mesh_for(cat, form_factor):
    for mesh, scale in MESHES.get(form_factor) or []:
        if mesh in cat:
            return mesh, scale
    return None


def _jitter(*parts):
    """A repeatable number in [-1, 1) from the parts (positions), so a rebuild puts everything back the same."""
    h = hashlib.md5("|".join(f"{p:.6f}" if isinstance(p, float) else str(p) for p in parts).encode()).digest()
    return h[0] / 128.0 - 1.0


def layout(snap, cat, to_xy, road_at, free, max_per_station=10):
    """Placements [(form factor, mesh, x, y, yaw, scale)] for a snapshot. to_xy(lat, lon) -> CARLA (x, y);
    road_at(x, y) -> (yaw of the nearest lane, (x, y) of its nearest point) or None; free(x, y) -> True when
    nothing (lane, building) is in the way. Docked bikes stand side by side square to the street; free-floating
    vehicles parallel to it; a blocked spot moves up to 3 m away from the road, else it is skipped."""
    out = []

    def spot(x, y, away):
        for k in (0.0, 1.0, 2.0, 3.0):
            px, py = x + away[0] * k, y + away[1] * k
            if free(px, py):
                return px, py
        return None

    def frame(x, y):
        r = road_at(x, y)
        if r is None:
            return 0.0, (1.0, 0.0), (0.0, 1.0)
        yaw, (rx, ry) = r
        u = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
        dx, dy = x - rx, y - ry
        d = math.hypot(dx, dy)
        away = (dx / d, dy / d) if d > 0.05 else (-u[1], u[0])
        return yaw, u, away

    for system in snap.get("systems", []):
        for st in system.get("stations", []):
            n = min(int(st.get("available") or 0), max_per_station)
            if n <= 0:
                continue
            x, y = to_xy(st["lat"], st["lon"])
            yaw, u, away = frame(x, y)
            for i, ff in enumerate(_dock_kinds(st, n)):
                m = mesh_for(cat, ff)
                if m is None:
                    continue
                off = (i - (n - 1) / 2.0) * SPACING_M
                p = spot(x + u[0] * off, y + u[1] * off, away)
                if p:
                    out.append((ff, m[0], p[0], p[1], yaw + 90.0 + 4.0 * _jitter(x, y, i), m[1]))
        for v in system.get("vehicles", []):
            m = mesh_for(cat, v.get("form_factor"))
            if m is None:
                continue
            x, y = to_xy(v["lat"], v["lon"])
            yaw, u, away = frame(x, y)
            p = spot(x, y, away)
            if p:
                flip = 180.0 if _jitter(v["lat"], v["lon"], "dir") > 0 else 0.0
                out.append((v["form_factor"], m[0], p[0], p[1], yaw + flip + 15.0 * _jitter(v["lat"], v["lon"]),
                            m[1]))
    return out


def _dock_kinds(st, n):
    """The form factors of the first n vehicles in a dock, the most common first."""
    kinds = []
    for ff, c in sorted((st.get("types") or {"bicycle": n}).items(), key=lambda kv: -kv[1]):
        kinds += [ff] * c
    return (kinds + ["bicycle"] * n)[:n]


def wanted(snap, max_per_station=10):
    """How many vehicles layout() tries to place (docked bikes up to the cap, free-floating ones of placeable kinds)."""
    n = 0
    for s in (snap or {}).get("systems", []):
        for st in s.get("stations", []):
            k = min(int(st.get("available") or 0), max_per_station)
            n += sum(1 for ff in _dock_kinds(st, k) if MESHES.get(ff)) if k > 0 else 0
        n += sum(1 for v in s.get("vehicles", []) if MESHES.get(v.get("form_factor")))
    return n


def _carla_helpers(cmap, buildings=None):
    """road_at and free for layout() from a carla.Map and building footprints (shapely polygons)."""
    import carla
    lt = carla.LaneType
    blocking = lt.Driving | lt.Parking | lt.Biking | lt.Bidirectional     # pavements are where bikes are parked
    tree = None
    if buildings:
        from shapely.strtree import STRtree
        tree = STRtree(buildings)

    def road_at(x, y):
        wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True)
        if wp is None:
            return None
        loc = wp.transform.location
        return wp.transform.rotation.yaw, (loc.x, loc.y)

    def free(x, y):
        if cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=False, lane_type=blocking) is not None:
            return False
        if tree is not None:
            from shapely.geometry import Point
            p = Point(x, y)
            if any(buildings[i].contains(p) for i in tree.query(p)):
                return False
        return True
    return road_at, free


def _footprints(mdir):
    path = Path(mdir) / "buildings.json"
    if not path.exists():
        return []
    from shapely.geometry import Polygon
    out = []
    for b in json.loads(path.read_text()):
        try:
            poly = Polygon(b["p"])
            out.append(poly if poly.is_valid else poly.buffer(0))
        except (ValueError, KeyError):
            continue
    return out


def add_to_scenery(pl, cfg, mdir, geo, cmap, log=log):
    """scenery.build's hook: the snapshot's shared bikes and scooters as placements of kind "shared"."""
    sc = cfg["map"].get("scenery") or {}
    if not sc.get("shared", True):
        return 0
    snap = snapshot(cfg, mdir, cfg["map"]["bbox"], log=log)
    if not snap or not snap.get("systems"):
        return 0
    ensure_meshes(pl.cat)
    road_at, free = _carla_helpers(cmap, _footprints(mdir))
    items = layout(snap, pl.cat, geo.to_carla, road_at, free, int(sc.get("shared_per_station", 10)))
    kinds = {}
    for ff, mesh, x, y, yaw, scale in items:
        pl.add("shared", mesh, x, y, yaw, scale)
        kinds[ff] = kinds.get(ff, 0) + 1
    want = wanted(snap, int(sc.get("shared_per_station", 10)))
    pl.counts["shared_skipped"] = want - len(items)
    log(f"scenery: {len(items)} shared bikes and scooters ({', '.join(f'{k} {v}' for k, v in sorted(kinds.items()))})"
        f" from {', '.join(s['name'] for s in snap['systems'])}, snapshot {snap['fetched']}; {want - len(items)} of "
        f"{want} left out (on a lane or in a building, with no free spot within 3 m)")
    return len(items)


def refresh_scenery(scenery, cfg, mdir, geo, cmap, log=log):
    """At run time, for a trial at time "now": a fresh snapshot replaces the shared bikes and scooters the map was
    built with (scenery.Scenery before its first update). Returns the number placed, or None (nothing changed)."""
    sim = cfg.get("simulation") or {}
    sc = cfg["map"].get("scenery") or {}
    if str(sim.get("time")).strip() != "now" or not sc.get("shared", True):
        return None
    if "shared" not in set((sim.get("scenery") or {}).get("kinds") or ["shared"]):
        return None
    try:
        from . import scenery as scen
        bbox = json.loads((Path(mdir) / "meta.json").read_text())["bbox"]
        snap = snapshot(cfg, mdir, bbox, LIVE_MAX_AGE_S, log)
        if not snap:
            return None
        built = json.loads((Path(mdir) / "scenery.json").read_text()).get("catalog", "0.9.16")
        cat = scen.Catalog(built)
        ensure_meshes(cat)
        meta = json.loads((Path(mdir) / "meta.json").read_text())
        elev = meta.get("elevation")
        dem = realworld.Dem.load(Path(mdir) / "real" / "dem.npz") if elev else None
        base = elev["base_m"] if elev else 0.0
        road_at, free = _carla_helpers(cmap, _footprints(mdir))
        items = layout(snap, cat, geo.to_carla, road_at, free, int(sc.get("shared_per_station", 10)))
        for key in list(scenery.cells):
            scenery.cells[key] = [it for it in scenery.cells[key] if it[6] != "shared"]
        index = {p: i for i, p in enumerate(scenery.meshes)}
        for ff, mesh, x, y, yaw, scale in items:
            path = cat.path(mesh)
            if path not in index:
                index[path] = len(scenery.meshes)
                scenery.meshes.append(path)
            if dem is not None:
                lat, lon = geo.to_latlon(x, y)
                z = dem.sample(lat, lon) - base
            else:
                import carla
                wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True)
                z = wp.transform.location.z if wp is not None else 0.0
            px, py, pz, pyaw = cat.place(mesh, x, y, z, yaw, scale)
            scenery.cells[(int(px // scenery.cell), int(py // scenery.cell))].append(
                [index[path], round(px, 2), round(py, 2), round(pz, 2), round(pyaw % 360, 1), round(scale, 3),
                 "shared"])
        log(f"scenery: {len(items)} shared bikes and scooters where they are now ({snap['fetched']})")
        return len(items)
    except Exception as err:  # noqa: BLE001 - the trial keeps the built snapshot
        log(f"shared bikes: no fresh snapshot ({type(err).__name__}: {err}); the map's own are used")
        return None


# ---------------------------------------------------------------- cyclists

def docks(snap):
    """[(lat, lon, capacity)] of the snapshot's bike docks (stations of bicycles, or of nothing yet but with docks;
    not the virtual parking areas of scooter systems)."""
    out = []
    for s in (snap or {}).get("systems", []):
        for st in s.get("stations", []):
            types = st.get("types") or {}
            if any(t in types for t in ("bicycle", "cargo_bicycle")) or (not types and (st.get("capacity") or 0) > 0):
                out.append((st["lat"], st["lon"], st.get("capacity") or 0))
    return out


def dock_edges(net, snap, radius=DOCK_RADIUS_M):
    """Bike edges within radius of a dock -> number of docks near each."""
    out = {}
    for lat, lon, _ in docks(snap):
        x, y = net.convertLonLat2XY(lon, lat)
        for e, _d in net.getNeighboringEdges(x, y, radius):
            if e.allows("bicycle"):
                out[e.getID()] = out.get(e.getID(), 0) + 1
    return out


def weight_docks(srcs, dsts, near, weight):
    """Trip start and end weights (cycling.od_weights, changed in place) times (1 + weight) on edges near a dock."""
    n = 0
    for w in list(srcs) + list(dsts):
        for e in near:
            if e in w:
                w[e] *= 1.0 + weight
                n += 1
    return n


def cycling_hook(cfg, net, map_dir, srcs, dsts, log=log):
    """cycling.py's hook: weights the trip ends near shared-bike docks. Returns the dock edges (or {})."""
    cc = (cfg["traffic"].get("cycling") or {})
    weight = float(cc.get("dock_weight", 2.0) or 0.0)
    if weight <= 0:
        return {}
    try:
        bbox = json.loads((Path(map_dir) / "meta.json").read_text())["bbox"]
        snap = snapshot(cfg, map_dir, bbox, log=log)
        near = dock_edges(net, snap) if snap else {}
        if near:
            weight_docks(srcs, dsts, near, weight)
            log(f"cyclists: {len(docks(snap))} shared-bike docks draw {1 + weight:g} x the trips on {len(near)} bike "
                f"edges within {DOCK_RADIUS_M:.0f} m")
        return near
    except Exception as err:  # noqa: BLE001 - a refinement: the bike demand stays as it was
        log(f"cyclists: shared-bike docks not used ({type(err).__name__}: {err})")
        return {}
