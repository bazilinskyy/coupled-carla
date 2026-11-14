"""Parked cars along the kerbs and on car parks, as real streets have them.

Where cars park (map build time, `build()`, called by scenery.build; writes parked.json):

- Kerbs. Every road of the OpenDRIVE has a kerb on each side: the outer edge of its outermost lane (driving, cycle or
  parking lane; a side whose outer lane is a bus lane gets none). A side has parked cars where
  1. OSM tags the street's side: `parking:left/right/both` = lane | street_side | on_kerb | half_on_kerb | yes |
     separate (the 2022 scheme) or `parking:lane:<side>` = parallel | diagonal | perpendicular | marked (the old
     one), with `...:orientation`; `no`, `no_parking`, `no_stopping` (also as `:restriction` or `parking:condition`)
     mean none;
  2. else a parking area lies at that kerb: a BGT wegdeel "parkeervlak" in the Netherlands, or an OSM
     amenity=parking with parking=street_side/lane/layby; its depth across the road gives the orientation (under
     3.2 m parallel, under 4.3 m diagonal, else perpendicular);
  3. else a default by road class and region (DEFAULT_SIDE): the share of street sides with kerb parking, drawn per
     road side. Residential streets in Europe, North America and Oceania usually have parallel parking on one or
     both sides (0.8), unclassified streets 0.5, tertiary 0.4, secondary 0.15; primary roads and above none unless
     tagged; none on sides with two or more lanes in one direction; none in Japan (the Garage Act requires an
     off-street space for every car and forbids overnight kerb parking). In the Netherlands, where the BGT maps the
     parking strips, an untagged side without one keeps 0.3 of the default (parking on the carriageway itself).
  Never within 5 m of an intersection (Dutch RVV art. 24 and most European codes), 5 m of a pedestrian crossing,
  10 m of traffic signals or 12 m of a bus stop; never on a lane CARLA drives, rides or walks on (checked against
  the CARLA map, as all scenery), clear of trees, lamps, furniture, signs and buildings.
- Car parks: OSM amenity=parking areas (parking=surface or untagged; not multi-storey, underground or rooftop) and
  BGT parking surfaces at least 4.6 m wide away from the kerbs are filled with rows of perpendicular stalls
  (2.5 x 5 m, 6 m aisles).

How many (trial time, `scenery()`): every stall carries a random number u; it is occupied when u is below the
occupancy of its zone at the trial's local hour and day type (simulation.time; 08:00 on a weekday when unset), so the
same stalls fill first in every trial. The zone is the use of the buildings within 60 m (residential, centre or
mixed). Occupancy = PEAK[zone] x PRESENCE[zone][day][hour]: the share of a zone's peak parking demand present at that
hour, after CROW's presence percentages (aanwezigheidspercentages, CROW publication 381 "Toekomstbestendig
parkeren"): residents are all home at night and about half of them in the working day; shops and offices fill the
centre in the day and empty it at night. An assumption, not a measurement: `simulation.parked.occupancy` overrides it.

Cars: CARLA's static car meshes (data/parked_cars.json, measured per CARLA version), spawned as static.prop.mesh
scenery in cells around the participants like the rest (scenery.Scenery). A static mesh has the paint it was made
with, so the body type of each car is drawn from the register's fleet (fleet.py) and, within a body type, the mesh
whose paint matches the register's colour shares is preferred. Vehicle actors with physics off could take any colour
but cost the server per actor and tick like SUMO traffic.
"""
import json
import math
import zlib
from collections import defaultdict
from pathlib import Path

DATA = Path(__file__).parent / "data"
VERSION = 1

# static car meshes: name -> (body type as fleet.BODY_BLUEPRINTS, paint colour as fleet.COLORS, blueprints it stands
# for). CARLA 0.9.16's "parked" meshes (Town10's parked layer) and the whole-car meshes of the older models.
CARS_09 = {
    "SM_NissanPatrol2021_parked": ("suv", "maroon", ["vehicle.nissan.patrol_2021"]),
    "SM_TeslaM3_parked": ("sedan", "black", ["vehicle.tesla.model3"]),
    "SM_Mini2021_parked": ("hatchback", "red", ["vehicle.mini.cooper_s_2021"]),
    "SM_VolkswagenT2_2021_Parked": ("van", "blue", ["vehicle.volkswagen.t2_2021", "vehicle.volkswagen.t2"]),
    "SM_LincolnParked": ("sedan", "black", ["vehicle.lincoln.mkz_2020"]),
    "SM_MercedesCCC_Parked": ("coupe", "red", ["vehicle.mercedes.coupe_2020"]),
    "SM_FordCrown_parked": ("taxi", "taxi_yellow", ["vehicle.ford.crown"]),
    "SM_ChargerParked": ("us_sedan", "red", ["vehicle.dodge.charger_2020"]),
    "SM_AudiA2": ("hatchback", "blue", ["vehicle.audi.a2"]),
    "SM_VolkswagenBeetle": ("hatchback", "brown", []),
    "SM_BMWGrandTourer": ("mpv", "blue", ["vehicle.bmw.grandtourer"]),
    "SM_ChevroletImpala": ("us_sedan", "maroon", ["vehicle.chevrolet.impala"]),
    "SM_Citroen_C3": ("hatchback", "green", ["vehicle.citroen.c3"]),
    "SM_Cybertruck": ("pickup", "grey", ["vehicle.tesla.cybertruck"]),
    "SM_JeepWranglerRubicon": ("suv", "white", ["vehicle.jeep.wrangler_rubicon"]),
    "SM_SeatLeon": ("hatchback", "gold", ["vehicle.seat.leon"]),
    "SM_LincolnMkz2017_prop": ("sedan", "grey", ["vehicle.lincoln.mkz_2017"]),
    "SM_MercedesBenzCoupeC": ("coupe", "beige", ["vehicle.mercedes.coupe"]),
    "SM_MiniCooperS": ("hatchback", "silver", ["vehicle.mini.cooper_s"]),
    "SM_Mustang_prop": ("coupe", "red", ["vehicle.ford.mustang"]),
    "SM_NissanMicra": ("hatchback", "blue", ["vehicle.nissan.micra"]),
    "SM_NissanPatrolST": ("suv", "beige", ["vehicle.nissan.patrol"]),
    "SM_Tesla": ("sedan", "grey", ["vehicle.tesla.model3"]),
}
# CARLA 1.0 has a Parked/ folder per model (from its content listing; colours not seen yet). They are used once
# data/parked_cars.json has their sizes for "1.0" (measure them on a 1.0 server as tools/mesh_catalog.py does).
CARS_10 = {
    "SM_AudiA2": ("hatchback", None, ["vehicle.audi.a2"]),
    "SM_BMWGranTourer_parked": ("mpv", None, ["vehicle.bmw.grandtourer"]),
    "SM_ChevroletImpala_Parked": ("us_sedan", None, ["vehicle.chevrolet.impala"]),
    "SM_FordCrown02_Parked": ("taxi", None, ["vehicle.ford.crown"]),
    "SM_Mustang_Parked": ("coupe", None, ["vehicle.ford.mustang"]),
    "SM_DodgeCharger_Parked": ("us_sedan", None, ["vehicle.dodge.charger_2020"]),
    "SM_JeepWranglerRubicon_Parked": ("suv", None, ["vehicle.jeep.wrangler_rubicon"]),
    "SM_AudiTT_Parked": ("coupe", None, ["vehicle.audi.tt"]),
    "SM_CitroenC3_Parked": ("hatchback", None, ["vehicle.citroen.c3"]),
    "SM_SeatLeon_Parked": ("hatchback", None, ["vehicle.seat.leon"]),
    "SM_TeslaM3_parked": ("sedan", None, ["vehicle.tesla.model3"]),
    "SM_NissanPatrol_Parked": ("suv", None, ["vehicle.nissan.patrol_2021"]),
    "SM_MiniCooper_Parked": ("hatchback", None, ["vehicle.mini.cooper_s_2021"]),
    "SM_ToyotaPrius_Parked": ("hatchback", None, ["vehicle.toyota.prius"]),
    "SM_Cybertruck_Parked": ("pickup", None, ["vehicle.tesla.cybertruck"]),
    "SM_MercedesCCC_Parked": ("coupe", None, ["vehicle.mercedes.coupe_2020"]),
    "SM_EtronParked": ("suv", None, ["vehicle.audi.etron"]),
    "SM_LincolnMKZ_Parked": ("sedan", None, ["vehicle.lincoln.mkz_2020"]),
    "SM_VolkswagenT2_Parked": ("van", None, ["vehicle.volkswagen.t2_2021"]),
    "SM_MercedesSprinter_Parked": ("van", None, ["vehicle.mercedes.sprinter"]),
}
# a body type without a mesh of its own borrows the next one
MAX_SHARE = 0.15            # no car model takes more than this share of the parked cars
BODY_FALLBACK = {"estate": "mpv", "mpv": "hatchback", "car": "hatchback", "us_sedan": "sedan", "taxi": "sedan",
                 "pickup": "suv", "van": "mpv", "coupe": "sedan", "suv": "mpv", "sedan": "hatchback"}

# --- where -----------------------------------------------------------------------------------------------------

NONE_VALUES = {"no", "none", "no_parking", "no_stopping", "fire_lane", "no_standing", "prohibited"}
POSITIONS = {"lane", "street_side", "on_kerb", "half_on_kerb", "shoulder", "yes", "separate", "marked", "on_street",
             "lay_by", "layby", "painted_area_only", "parallel", "diagonal", "perpendicular"}
ORIENTATIONS = {"parallel", "diagonal", "perpendicular"}


def side_tags(tags, side):
    """OSM street-parking tags of one side ("left"/"right") of a way -> None (untagged), False (no parking) or
    {"orientation": parallel|diagonal|perpendicular, "position": ...}. Both the 2022 scheme (parking:<side>=lane,
    parking:<side>:orientation) and the older parking:lane:<side>=parallel are read; <side> also "both"."""
    def get(*keys):
        for k in keys:
            for s in (side, "both"):
                v = tags.get(k.format(s))
                if v:
                    return v.split(";")[0].strip().lower()
        return None
    restriction = get("parking:{}:restriction", "parking:condition:{}", "parking:lane:{}:condition")
    new = get("parking:{}")
    old = get("parking:lane:{}")
    value = new or old
    if value is None:
        return None
    if value in NONE_VALUES or restriction in NONE_VALUES:
        return False
    if value not in POSITIONS:
        return None
    orient = get("parking:{}:orientation") or (old if old in ORIENTATIONS else None) or \
        (value if value in ORIENTATIONS else None) or "parallel"
    position = get("parking:lane:{}:" + (old or "")) if old in ORIENTATIONS else value
    return {"orientation": orient if orient in ORIENTATIONS else "parallel", "position": position or "lane"}


# share of street sides with kerb parking where nothing is tagged, by region and OSM highway class (see the
# module docstring; primary and above: none unless tagged)
DEFAULT_SIDE = {
    "europe": {"residential": 0.8, "living_street": 0.5, "unclassified": 0.5, "tertiary": 0.4, "secondary": 0.15},
    "americas_oceania": {"residential": 0.8, "living_street": 0.5, "unclassified": 0.5, "tertiary": 0.4,
                         "secondary": 0.15},
    "japan": {},
    "other": {"residential": 0.6, "living_street": 0.4, "unclassified": 0.4, "tertiary": 0.3, "secondary": 0.1},
}
BGT_WITHOUT_STRIP = 0.3     # NL: an untagged side without a BGT parking strip keeps this share of the default


def region(bbox):
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    if 24 <= lat <= 46 and 123 <= lon <= 146:
        return "japan"
    if 35 <= lat <= 72 and -25 <= lon <= 45:
        return "europe"
    if (lon < -50 and lat > 15) or (lat < -10 and lon > 110):
        return "americas_oceania"      # USA, Canada, Australia, New Zealand (and the Pacific)
    return "other"


def default_share(highway, reg, lanes_one_way, bgt_covered=False, bgt_strip=False):
    p = DEFAULT_SIDE.get(reg, DEFAULT_SIDE["other"]).get(highway or "", 0.0)
    if lanes_one_way >= 2:
        p = 0.0
    if bgt_covered and not bgt_strip:
        p *= BGT_WITHOUT_STRIP
    return p


# stall geometry (m): along-kerb pitch per orientation and the gap between the lane edge and the car
PITCH = {"parallel": 6.0, "diagonal": 3.0, "perpendicular": 2.6}
DIAGONAL_DEG = 60.0
GAP = 0.25
STALL_W, STALL_L, AISLE = 2.5, 5.0, 6.0
BUFFER = {"intersection": 5.0, "crossing": 5.0, "signals": 10.0, "bus_stop": 12.0}
ZONE_R = 60.0
RES_CLS = {"house", "detached", "semidetached_house", "terrace", "residential", "apartments", "bungalow",
           "dormitory", "farm", "cabin"}
CENTRE_CLS = {"commercial", "retail", "office", "supermarket", "hotel", "civic", "public", "government",
              "train_station", "transportation", "university", "school", "hospital", "church", "cathedral"}

# --- how many ---------------------------------------------------------------------------------------------------

ZONES = ["residential", "centre", "mixed"]
PEAK = {"residential": 0.92, "centre": 0.90, "mixed": 0.85}
_RES_WD = [1.0] * 7 + [0.85, 0.65] + [0.55] * 8 + [0.7, 0.8, 0.9, 0.9, 0.9, 0.95, 0.95]
_RES_WE = [1.0] * 8 + [0.9, 0.8] + [0.7] * 7 + [0.75, 0.8, 0.85, 0.9, 0.9, 0.95, 0.95]
_CEN_WD = [0.25] * 7 + [0.35, 0.55, 0.75] + [0.95] * 3 + [1.0] * 3 + [0.95, 0.85, 0.7, 0.6, 0.6, 0.5, 0.4, 0.3]
_CEN_SA = [0.25] * 8 + [0.4, 0.6] + [0.9] * 2 + [1.0] * 5 + [0.85, 0.7, 0.6, 0.6, 0.5, 0.4, 0.3]
_CEN_SU = [0.25] * 10 + [0.4, 0.5] + [0.6] * 5 + [0.55, 0.5, 0.45, 0.4, 0.35, 0.3, 0.3]
PRESENCE = {
    "residential": {"weekday": _RES_WD, "saturday": _RES_WE, "sunday": _RES_WE},
    "centre": {"weekday": _CEN_WD, "saturday": _CEN_SA, "sunday": _CEN_SU},
}
PRESENCE["mixed"] = {d: [(a + b) / 2 for a, b in zip(PRESENCE["residential"][d], PRESENCE["centre"][d])]
                     for d in ("weekday", "saturday", "sunday")}


def occupancy(zone, day, hour):
    """Share of a zone's stalls taken at that local hour of that day type (weekday, saturday, sunday)."""
    return PEAK[zone] * PRESENCE[zone][day][int(hour) % 24]


def _u(*parts):
    return (zlib.crc32("|".join(str(p) for p in parts).encode()) % 100000) / 100000.0


# --- the car meshes --------------------------------------------------------------------------------------------

def car_meshes(cat):
    """{mesh name: (body, colour, blueprints)} of the cars this catalogue's CARLA version has, with their sizes added
    to the catalogue (data/parked_cars.json). Empty when that version's cars are not measured."""
    from .scenery import major
    table = CARS_09 if major(cat.version) == 0 else CARS_10
    try:
        sizes = json.loads((DATA / "parked_cars.json").read_text()).get(cat.version, {})
    except (OSError, ValueError):
        sizes = {}
    out = {}
    for path, m in sizes.items():
        name = path.rsplit(".", 1)[-1]
        if name in table and m["ext"][0] > 0:
            cat.by_name.setdefault(name, (path, m))
            out[name] = table[name]
    return out


class Chooser:
    """Draws a car mesh for a stall. The body types come from the fleet's body shares (the register's, else
    traffic.car_types mapped to body types); within a body type the meshes are weighted by how common their paint is
    in the register. A body type CARLA has fewer than three meshes for lends the rest of its share to the next body
    type (BODY_FALLBACK), so no one model fills a street: CARLA has one MPV, and the Dutch register's 22 % MPVs would
    otherwise all be the same blue BMW."""

    def __init__(self, cars, reg=None, car_types=None):
        from . import fleet
        self.cars = cars
        pools = defaultdict(list)
        for name, (body, _, bps) in cars.items():
            pools[body].append(name)
            for b, bb in fleet.BODY_BLUEPRINTS.items():
                if b != body and set(bps) & set(bb) and name not in pools[b]:
                    pools[b].append(name)
        share = {}
        if reg and reg.get("body"):
            share = {b: float(n) for b, n in reg["body"].items() if n}
        else:
            bp_body = {}
            for body, bps in fleet.BODY_BLUEPRINTS.items():
                for bp in bps:
                    bp_body.setdefault(bp, body)
            own = {bp: body for body, _, bps in cars.values() for bp in bps}
            for bp, w in (car_types or {}).items():
                body = own.get(bp) or bp_body.get(bp)
                if body:
                    share[body] = share.get(body, 0.0) + float(w)
        colors = (reg or {}).get("colors") or fleet.WORLD_COLORS
        tot = sum(colors.values()) or 1.0
        col = {k: v / tot for k, v in colors.items()}
        col.setdefault("maroon", col.get("red", 0.0) * 0.3)
        col.setdefault("gold", col.get("beige", 0.0) + col.get("yellow", 0.0))
        col.setdefault("silver", col.get("grey", 0.0) * 0.5)
        taxis = bool(reg and reg.get("body", {}).get("taxi"))

        def paint(m):
            c = cars[m][1]
            if c == "taxi_yellow":
                return 0.02 if taxis else 0.0
            return 0.05 if c is None else col.get(c, 0.0) + 0.02

        self.weights = defaultdict(float)

        def spread(body, w, depth=0):
            pool = pools.get(body, [])
            nxt = BODY_FALLBACK.get(body)
            own = min(1.0, len(pool) / 3.0) if nxt and depth < 6 else (1.0 if pool else 0.0)
            ps = [paint(m) for m in pool]
            if pool and sum(ps) <= 0:      # only yellow cabs, and the register has no cabs: the next body type
                if nxt and depth < 6:
                    spread(nxt, w, depth + 1)
                    return
                ps = [1.0] * len(pool)
            for m, p in zip(pool, ps):
                self.weights[m] += w * own * p / sum(ps)
            if own < 1.0 and nxt and depth < 6:
                spread(nxt, w * (1.0 - own), depth + 1)

        for body, w in (share or {"hatchback": 1.0}).items():
            spread(body, w)
        if not self.weights:
            self.weights.update({m: 1.0 for m in cars})
        for _ in range(8 if len(self.weights) * MAX_SHARE >= 1.2 else 0):
            # and no model above MAX_SHARE of all parked cars (its excess shared by the others)
            tot = sum(self.weights.values())
            over = {m: w - MAX_SHARE * tot for m, w in self.weights.items() if w > MAX_SHARE * tot * 1.001}
            rest = {m: w for m, w in self.weights.items() if m not in over and w > 0}
            if not over or not rest:
                break
            extra, rtot = sum(over.values()), sum(rest.values())
            for m in over:
                self.weights[m] -= over[m]
            for m, w in rest.items():
                self.weights[m] += extra * w / rtot
        self.names = sorted(self.weights)

    def __call__(self, u1, u2=None):
        return _pick(self.names, [self.weights[m] for m in self.names], u1)


def _pick(items, weights, u):
    tot = sum(weights)
    if tot <= 0:
        return items[int(u * len(items)) % len(items)]
    acc = 0.0
    for it, w in zip(items, weights):
        acc += w / tot
        if u < acc:
            return it
    return items[-1]


# --- map build ---------------------------------------------------------------------------------------------------

def _osm_ways(mdir, geo, road_types):
    """Highway ways of the map's OSM extract in CARLA coordinates: [(tags, [(x, y), ...])]."""
    import xml.etree.ElementTree as ET
    f = mdir / "map.osm"
    if not f.exists():
        return [], {}
    root = ET.parse(f).getroot()
    nodes, points = {}, {}
    for n in root.iter("node"):
        nodes[n.get("id")] = (float(n.get("lat")), float(n.get("lon")))
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        if tags.get("highway") in ("crossing", "traffic_signals") or tags.get("crossing") or \
                tags.get("highway") == "bus_stop":
            kind = "bus_stop" if tags.get("highway") == "bus_stop" else \
                "signals" if tags.get("highway") == "traffic_signals" else "crossing"
            points.setdefault(kind, []).append(nodes[n.get("id")])
    ways = []
    for w in root.iter("way"):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        if tags.get("highway") not in road_types:
            continue
        pts = [geo.to_carla(*nodes[r.get("ref")]) for r in w.findall("nd") if r.get("ref") in nodes]
        if len(pts) >= 2:
            ways.append((tags, pts))
    pts_xy = {k: [geo.to_carla(lat, lon) for lat, lon in v] for k, v in points.items()}
    return ways, pts_xy


def osm_parking(bbox, path, log=print):
    """OSM amenity=parking areas in the box (Overpass JSON with geometry and tags, cached); {} when it fails."""
    from . import realworld as rw
    if path.exists():
        return json.loads(path.read_text())
    s, w, n, e = bbox
    q = (f'[out:json][timeout:120];(way["amenity"="parking"]({s},{w},{n},{e});'
         f'relation["amenity"="parking"]({s},{w},{n},{e}););out geom tags;')
    try:
        data = rw.overpass_json(q)
    except Exception as err:  # noqa: BLE001 - offline: kerbs from tags, BGT and defaults only
        log(f"parked: OSM parking areas not fetched ({err})")
        return {}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, separators=(",", ":")))
    return data


def _areas(osm, bgt_data, geo):
    """Parking areas in CARLA coordinates: [(polygon, kind, source)], kind "street" (beside a road) or "lot"."""
    from shapely.geometry import Polygon
    out = []
    for el in osm.get("elements", []):
        t = el.get("tags", {})
        if el["type"] == "relation":
            rings = [m.get("geometry") for m in el.get("members", []) if m.get("role") == "outer" and m.get("geometry")]
        else:
            rings = [el.get("geometry")] if el.get("geometry") else []
        p = (t.get("parking") or "surface").lower()
        if p in ("multi-storey", "underground", "rooftop", "garage_boxes", "carports", "garages") or \
                t.get("location") in ("underground", "rooftop") or t.get("building") or \
                str(t.get("layer", "0")).strip().startswith("-") or (t.get("level") or "0").strip() not in ("0", ""):
            continue
        kind = "street" if p in ("street_side", "lane", "layby", "on_kerb", "half_on_kerb") else "lot"
        for g in rings:
            if not g or len(g) < 4:
                continue
            try:
                poly = Polygon([geo.to_carla(q["lat"], q["lon"]) for q in g])
            except (ValueError, KeyError, TypeError):
                continue
            poly = poly if poly.is_valid else poly.buffer(0)
            if not poly.is_empty and poly.area > 8:
                out.append((poly, kind, "osm"))
    if bgt_data:
        from .bgt import _xy_many
        for s in bgt_data.get("surfaces", []):
            if s.get("c") != "wegdeel" or s.get("f") != "parkeervlak":
                continue
            rings = [_xy_many(geo, r) for r in s["r"] if len(r) >= 4]
            if not rings:
                continue
            poly = Polygon(rings[0], rings[1:])
            poly = poly if poly.is_valid else poly.buffer(0)
            if not poly.is_empty and poly.area > 8:
                out.append((poly, "any", "bgt"))
    return out


def _depth(poly, cx, cy, nx, ny, reach=8.0):
    """Length of the line across the road (normal n through (cx, cy)) inside the polygon."""
    from shapely.geometry import LineString
    line = LineString([(cx - nx * reach, cy - ny * reach), (cx + nx * reach, cy + ny * reach)])
    return poly.intersection(line).length


def _rect(cx, cy, length, width, yaw_deg):
    from shapely.geometry import Polygon
    r = math.radians(yaw_deg)
    ux, uy, vx, vy = math.cos(r), math.sin(r), -math.sin(r), math.cos(r)
    hl, hw = length / 2, width / 2
    return Polygon([(cx + a * hl * ux + b * hw * vx, cy + a * hl * uy + b * hw * vy)
                    for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))])


class _Grid:
    """Points (with a radius) and polygons to keep clear of, looked up by a box."""

    def __init__(self, cell=20.0):
        self.cell, self.cells = cell, defaultdict(list)

    def add(self, geom):
        x0, y0, x1, y1 = geom.bounds
        for i in range(int(x0 // self.cell), int(x1 // self.cell) + 1):
            for j in range(int(y0 // self.cell), int(y1 // self.cell) + 1):
                self.cells[(i, j)].append(geom)

    def hits(self, geom):
        x0, y0, x1, y1 = geom.bounds
        seen = set()
        for i in range(int(x0 // self.cell), int(x1 // self.cell) + 1):
            for j in range(int(y0 // self.cell), int(y1 // self.cell) + 1):
                for g in self.cells.get((i, j), ()):
                    if id(g) not in seen:
                        seen.add(id(g))
                        if g.intersects(geom):
                            return True
        return False


OBSTACLE_R = {"tree": 0.6, "lamp": 0.35, "sign": 0.3, "furniture": 0.4, "bush": 0.8}


def build(mdir, cfg, geo, cat, cmap, ground, placer, blds, log=print):
    """parked.json: every stall (kerb and car park) with its car, zone and draw; returns counts for meta.json."""
    import carla
    from shapely.geometry import LineString, Point
    from . import fleet
    from . import realworld as rw
    mdir = Path(mdir)
    mc = cfg["map"]
    sc = mc.get("scenery") or {}
    (mdir / "parked.json").unlink(missing_ok=True)      # never leave one for another catalogue behind
    if mc.get("carla_level"):
        log("parked: not on a baked level yet (its own ground heights; tools/snap_scenery.py snaps scenery.json only)")
        return {}
    cars = car_meshes(cat)
    if not cars:
        log(f"parked: no measured car meshes for CARLA {cat.version} (data/parked_cars.json); none placed")
        return {}
    bbox = mc["bbox"]
    reg_name = region(bbox)
    tc = cfg.get("traffic") or {}
    reg = fleet.registry(bbox) if tc.get("fleet", "auto") == "auto" else None
    choose = Chooser(cars, reg, tc.get("car_types"))
    LT = carla.LaneType
    blocking = LT.Driving | LT.Biking | LT.Bidirectional | LT.Sidewalk | LT.Restricted
    passable = {LT.Driving, LT.Biking, LT.Parking, LT.Bidirectional, LT.Restricted}

    def on_lane(x, y, z):
        return cmap.get_waypoint(carla.Location(x=x, y=y, z=z), project_to_road=False, lane_type=blocking) is not None

    # what to keep clear of: scenery objects, buildings, and the stalls already placed
    names = {i: m for m, i in placer.mesh_index.items()}
    clear = _Grid()
    for it in placer.items:
        r = OBSTACLE_R.get(it[6])
        if r is not None:
            cx, cy = it[8] if len(it) > 8 else it[1:3]
            if it[6] == "furniture":
                L, W, _ = cat.size(names[it[0]])
                r = max(0.3, min(1.5, 0.5 * max(L, W) * it[5]))
            clear.add(Point(cx, cy).buffer(r, 6))
    for b in blds:
        clear.add(b["poly"])
    cars_grid = _Grid()
    ways, pts = _osm_ways(mdir, geo, set(mc.get("road_types") or []) | {"living_street"})
    wseg = _Grid(30.0)
    for wi, (tags, line) in enumerate(ways):
        for a, b in zip(line, line[1:]):
            seg = _Seg(LineString([a, b]), wi, (b[0] - a[0], b[1] - a[1]))
            wseg.add(seg)
    no_park = _Grid()
    for kind, r in BUFFER.items():
        if kind == "intersection":
            continue
        for x, y in pts.get(kind, ()):
            no_park.add(Point(x, y).buffer(r, 8))
    bgt_data = None
    if sc.get("bgt", "auto") not in (False, "off", "false") and rw.in_netherlands(bbox):
        f = mdir / "real" / "bgt.json"
        bgt_data = json.loads(f.read_text()) if f.exists() else None
    areas = _areas(osm_parking(bbox, mdir / "real" / "osm_parking.json", log), bgt_data, geo)
    street_areas, area_src = _Grid(), {}
    for poly, kind, src in areas:
        if kind in ("street", "any"):
            street_areas.add(poly)
            area_src[id(poly)] = src

    # zones by the buildings around
    bgrid = defaultdict(list)
    for b in blds:
        c = b["poly"].centroid
        k = "res" if b["cls"] in RES_CLS else "cen" if b["cls"] in CENTRE_CLS else None
        if k:
            bgrid[(int(c.x // ZONE_R), int(c.y // ZONE_R))].append((c.x, c.y, k, b["poly"].area))

    def zone(x, y, highway):
        res = cen = 0.0
        gx, gy = int(x // ZONE_R), int(y // ZONE_R)
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                for bx, by, k, a in bgrid.get((gx + i, gy + j), ()):
                    if (bx - x) ** 2 + (by - y) ** 2 < ZONE_R ** 2:
                        if k == "res":
                            res += a
                        else:
                            cen += a
        if res + cen > 200:
            if res >= 0.65 * (res + cen):
                return 0
            if cen >= 0.5 * (res + cen):
                return 1
            return 2
        return 0 if highway in ("residential", "living_street") else 2

    def osm_side(x, y, yaw_deg):
        """(tags, side, highway) of the OSM way along this point (its side of the way), or (None, None, None)."""
        p = Point(x, y).buffer(14.0)
        best = None
        ux, uy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        x0, y0, x1, y1 = p.bounds
        cand = set()
        for i in range(int(x0 // wseg.cell), int(x1 // wseg.cell) + 1):
            for j in range(int(y0 // wseg.cell), int(y1 // wseg.cell) + 1):
                cand.update(wseg.cells.get((i, j), ()))
        for seg in cand:
            d = seg.line.distance(Point(x, y))
            if d > 14.0:
                continue
            dx, dy = seg.dir
            n = math.hypot(dx, dy) or 1.0
            if abs(dx / n * ux + dy / n * uy) < 0.8:        # not along the road (over 37 degrees)
                continue
            if best is None or d < best[0]:
                best = (d, seg)
        if best is None:
            return None, None, None
        seg = best[1]
        tags = ways[seg.way][0]
        (ax, ay), (bx, by) = seg.line.coords[0], seg.line.coords[1]
        # CARLA's y points south (left-handed): left of the way's direction is a negative cross product
        cross = (bx - ax) * (y - ay) - (by - ay) * (x - ax)
        return tags, ("left" if cross < 0 else "right"), tags.get("highway")

    junction_kind = {}

    def is_intersection(j):
        if j.id not in junction_kind:
            bins = set()
            for a, _ in j.get_waypoints(LT.Driving):
                bins.add(int(((a.transform.rotation.yaw % 360) + 22.5) // 45) % 8)
            junction_kind[j.id] = len(bins) >= 3
        return junction_kind[j.id]

    def near_intersection(wp, reach):
        for f in (wp.next, wp.previous):
            try:
                for w in f(reach):
                    if w.is_junction and is_intersection(w.get_junction()):
                        return True
            except RuntimeError:
                pass
        return False

    # kerb lines: per road section and side, the outermost lane at each s
    wps = cmap.generate_waypoints(1.0)
    sections = defaultdict(list)
    for w in wps:
        if not w.is_junction and w.lane_type == LT.Driving:     # roads for cars (not cycle paths)
            sections[(w.road_id, w.section_id)].append(w)
    counts = defaultdict(int)
    slots = []
    mesh_index = {}
    src_names = ["osm_tag", "bgt", "osm_area", "default", "lot_osm", "lot_bgt"]

    def lane_at(road, lid, s):
        try:
            return cmap.get_waypoint_xodr(road, lid, s)
        except RuntimeError:
            return None

    def place(mesh, cx, cy, z, yaw, zi, u, src):
        x, y, zz, yaw = cat.place(mesh, cx, cy, z, yaw, 1.0)
        mi = mesh_index.setdefault(mesh, len(mesh_index))
        slots.append([mi, round(x, 2), round(y, 2), round(zz, 2), round(yaw % 360, 1), zi, round(u, 4),
                      src_names.index(src)])
        counts[src] += 1

    for (road, sec), lst in sections.items():
        ids = sorted({w.lane_id for w in lst})
        if not ids:
            continue
        smin, smax = min(w.s for w in lst), max(w.s for w in lst)
        length = smax - smin
        if length < 6.0:
            continue
        n_drive = len(ids)
        for side in ("right", "left"):
            smid = (smin + smax) / 2
            inner = ids[0] if side == "right" else ids[-1]     # the driving lane nearest this side's edge
            step = -1 if side == "right" else 1
            # the outermost lane on this side: driving, cycle, parking or bus lane
            outer, lid = lane_at(road, inner, smid), inner
            if outer is None:
                continue
            while True:
                nxt = lid + step if lid + step != 0 else lid + 2 * step
                w = lane_at(road, nxt, smid)
                if w is None or w.lane_type not in passable or w.lane_width < 0.5:
                    break
                outer, lid = w, nxt
            if outer.lane_type == LT.Restricted:
                counts["side_bus_lane"] += 1
                continue
            on_parking_lane = outer.lane_type == LT.Parking
            ux, uy = _ref_dir(road, lid, smid, smin, smax, lane_at)
            nx, ny = (-uy, ux) if side == "right" else (uy, -ux)
            tf = outer.transform
            ex = tf.location.x + nx * outer.lane_width / 2
            ey = tf.location.y + ny * outer.lane_width / 2
            # another road's lane just beyond (the other half of a two-way street: netconvert writes each
            # direction as its own road) means this edge is not a kerb
            if not on_parking_lane and on_lane(ex + nx * 0.7, ey + ny * 0.7, tf.location.z):
                counts["side_beside_road"] += 1
                continue
            tags, wside, highway = osm_side(ex, ey, math.degrees(math.atan2(uy, ux)))
            if tags is not None and (tags.get("junction") in ("roundabout", "circular")
                                     or tags.get("highway", "").endswith("_link")):
                counts["side_roundabout_or_link"] += 1
                continue
            if side == "left" and not on_parking_lane and (tags is None or tags.get("oneway") not in
                                                           ("yes", "true", "1", "-1")):
                counts["side_median"] += 1      # the inner edge of a carriageway that is not a one-way street
                continue
            tagged = side_tags(tags, wside) if tags else None
            if tagged is False:
                counts["side_tagged_no"] += 1
                continue
            position = "lane"
            if tagged:
                src, orient, position = "osm_tag", tagged["orientation"], tagged["position"]
            else:
                src = orient = None
                probe = Point(ex + nx * 2.0, ey + ny * 2.0).buffer(3.0)
                hit = [g for g in _cands(street_areas, probe) if g.intersects(probe)]
                if hit:
                    poly = max(hit, key=lambda g: g.intersection(probe).area)
                    d = _depth(poly, ex + nx * 2.0, ey + ny * 2.0, nx, ny)
                    orient = "parallel" if d < 3.2 else "diagonal" if d < 4.3 else "perpendicular"
                    src = "bgt" if area_src[id(poly)] == "bgt" else "osm_area"
                elif on_parking_lane:
                    src, orient = "osm_tag", "parallel"
                elif _u(road, sec, side, "side") < default_share(highway, reg_name, n_drive, bgt_data is not None):
                    src, orient = "default", "parallel"
            if src is None:
                counts["side_none"] += 1
                continue
            counts["side_" + src] += 1
            # the kerb's direction of travel: that of the nearest driving lane on this side
            drive_lid = lid
            while drive_lid != inner and (lane_at(road, drive_lid, smid) is None or
                                          lane_at(road, drive_lid, smid).lane_type != LT.Driving):
                drive_lid = drive_lid - step if drive_lid - step != 0 else drive_lid - 2 * step
            pitch = PITCH[orient]
            s, k = smin + 1.0, 0
            while s + pitch <= smax - 1.0:
                sc_ = s + pitch / 2
                s += pitch
                k += 1
                w = lane_at(road, lid, sc_)
                if w is None or w.lane_type != outer.lane_type:
                    continue
                u = _u(road, sec, side, k)
                mesh = choose(_u(road, sec, side, k, "body"), _u(road, sec, side, k, "mesh"))
                L, W, H = cat.size(mesh)
                ux, uy = _ref_dir(road, lid, sc_, smin, smax, lane_at)
                nx, ny = (-uy, ux) if side == "right" else (uy, -ux)
                tf = w.transform
                dw = lane_at(road, drive_lid, sc_) or w
                dr = math.radians(dw.transform.rotation.yaw)
                dx, dy = math.cos(dr), math.sin(dr)
                jitter = _u(road, sec, side, k, "j") - 0.5
                if orient == "parallel":
                    depth, yaw, along = W, dw.transform.rotation.yaw + 2.0 * jitter, 0.4 * jitter
                elif orient == "perpendicular":      # mostly nose in, as people drive in forwards
                    depth, along = L, 0.15 * jitter
                    nose_in = _u(road, sec, side, k, "nose") < 0.8
                    yaw = math.degrees(math.atan2(ny, nx)) + (0.0 if nose_in else 180.0) + 3.0 * jitter
                else:                                 # angled forwards, into the kerb
                    a = math.radians(DIAGONAL_DEG)
                    depth, along = L * math.sin(a) + W * math.cos(a), 0.1 * jitter
                    yaw = math.degrees(math.atan2(math.cos(a) * dy + math.sin(a) * ny,
                                                  math.cos(a) * dx + math.sin(a) * nx)) + 2.0 * jitter
                if on_parking_lane:
                    cx, cy = tf.location.x + ux * along, tf.location.y + uy * along
                else:
                    lat = GAP + {"on_kerb": 1.0, "half_on_kerb": 0.5}.get(position, 0.0) + depth / 2
                    cx = tf.location.x + nx * (w.lane_width / 2 + lat) + ux * along
                    cy = tf.location.y + ny * (w.lane_width / 2 + lat) + uy * along
                z = tf.location.z
                if near_intersection(w, pitch / 2 + BUFFER["intersection"]):
                    counts["skip_intersection"] += 1
                    continue
                fp = _rect(cx, cy, L + 0.1, W + 0.1, yaw)
                if no_park.hits(fp):
                    counts["skip_crossing_stop_signals"] += 1
                    continue
                if any(on_lane(px, py, z) for px, py in list(fp.exterior.coords)[:4] + _edge_mids(fp) + [(cx, cy)]):
                    counts["skip_lane"] += 1
                    continue
                if clear.hits(fp) or cars_grid.hits(fp):
                    counts["skip_obstacle"] += 1
                    continue
                cars_grid.add(_rect(cx, cy, L + 0.4, W + 0.3, yaw))
                place(mesh, cx, cy, z, yaw, zone(cx, cy, highway), u, src)
    n_kerb = len(slots)

    # car parks: rows of perpendicular stalls in OSM parking lots and wide BGT parking surfaces
    for ai, (poly, kind, src) in enumerate(areas):
        if kind == "street":
            continue
        from .scenery import _rect_of
        cx0, cy0, L0, W0, yaw0 = _rect_of(poly)
        if W0 < 4.6 or poly.area < 30:
            continue
        inner = poly.buffer(-0.2)
        if inner.is_empty:
            continue
        r = math.radians(yaw0)
        ux, uy, vx, vy = math.cos(r), math.sin(r), -math.sin(r), math.cos(r)
        rows, v, facing = [], -W0 / 2 + STALL_L / 2, 1     # facing +1: the aisle on the +v side
        while v + STALL_L / 2 <= W0 / 2 + 0.01:
            rows.append((v, facing))
            v, facing = (v + STALL_L + AISLE, -1) if facing == 1 else (v + STALL_L, 1)
        if len(rows) == 1:      # one row (a strip along a street): the aisle is the road on its nearer side
            wp = cmap.get_waypoint(carla.Location(x=cx0, y=cy0), project_to_road=True, lane_type=LT.Driving)
            if wp is not None:
                d = (wp.transform.location.x - cx0) * vx + (wp.transform.location.y - cy0) * vy
                rows = [(rows[0][0], 1 if d > 0 else -1)]
        for ri, (v, facing) in enumerate(rows):
            a = -L0 / 2 + STALL_W / 2
            k = 0
            while a <= L0 / 2 - STALL_W / 2:
                k += 1
                sx, sy = cx0 + ux * a + vx * v, cy0 + uy * a + vy * v
                a += STALL_W
                stall = _rect(sx, sy, STALL_L, STALL_W, yaw0 + 90.0)
                if not inner.contains(stall):
                    continue
                key = ("lot", round(cx0), round(cy0), ri, k)
                mesh = choose(_u(*key, "body"), _u(*key, "mesh"))
                L, W, H = cat.size(mesh)
                nose_in = _u(*key, "nose") < 0.8
                yaw = yaw0 - 90.0 * facing + (0.0 if nose_in else 180.0) + 3.0 * (_u(*key, "j") - 0.5)
                z = ground(sx, sy) - 0.05
                fp = _rect(sx, sy, L + 0.1, W + 0.1, yaw)
                if any(on_lane(px, py, z) for px, py in list(fp.exterior.coords)[:4] + [(sx, sy)]):
                    counts["skip_lane"] += 1
                    continue
                if clear.hits(fp) or cars_grid.hits(fp):
                    counts["skip_obstacle"] += 1
                    continue
                cars_grid.add(_rect(sx, sy, L + 0.3, W + 0.3, yaw))
                place(mesh, sx, sy, z, yaw, zone(sx, sy, None), _u(*key), "lot_" + src)
    counts["kerb_stalls"], counts["lot_stalls"] = n_kerb, len(slots) - n_kerb
    meshes = sorted(mesh_index, key=mesh_index.get)
    out = {"version": VERSION, "catalog": cat.version, "cell_m": 100.0, "meshes": [cat.path(m) for m in meshes],
           "zones": ZONES, "sources": src_names, "region": reg_name, "fleet": (reg or {}).get("source"),
           "slots": slots, "counts": dict(counts)}
    (mdir / "parked.json").write_text(json.dumps(out, separators=(",", ":")))
    zc = defaultdict(int)
    for sl in slots:
        zc[ZONES[sl[5]]] += 1
    log(f"parked: {n_kerb} kerb stalls ({', '.join(f'{k[5:]} {v}' for k, v in sorted(counts.items()) if k.startswith('side_'))} "
        f"road sides), {len(slots) - n_kerb} in car parks ({len(areas)} parking areas); zones "
        f"{', '.join(f'{k} {v}' for k, v in sorted(zc.items()))}; skipped {counts['skip_lane']} on lanes, "
        f"{counts['skip_obstacle']} at objects, {counts['skip_intersection']} at intersections, "
        f"{counts['skip_crossing_stop_signals']} at crossings/stops/signals -> {mdir / 'parked.json'}")
    return {"parked_" + k: v for k, v in counts.items() if not k.startswith("skip")}


class _Seg:
    """A segment of an OSM way, with the way's index and direction (shapely geometries take no attributes)."""

    def __init__(self, line, way, d):
        self.line, self.way, self.dir, self.bounds = line, way, d, line.bounds

    def intersects(self, geom):
        return self.line.intersects(geom)


def _cands(grid, geom):
    x0, y0, x1, y1 = geom.bounds
    out = set()
    for i in range(int(x0 // grid.cell), int(x1 // grid.cell) + 1):
        for j in range(int(y0 // grid.cell), int(y1 // grid.cell) + 1):
            out.update(grid.cells.get((i, j), ()))
    return out


def _edge_mids(poly):
    c = list(poly.exterior.coords)[:4]
    return [((c[i][0] + c[(i + 1) % 4][0]) / 2, (c[i][1] + c[(i + 1) % 4][1]) / 2) for i in range(4)]


def _ref_dir(road, lid, s, smin, smax, lane_at):
    """Unit vector along the road's reference line (increasing s) at s, from two points of the lane."""
    a, b = max(smin, s - 1.0), min(smax, s + 1.0)
    wa, wb = lane_at(road, lid, a), lane_at(road, lid, b)
    if wa is None or wb is None or b - a < 0.2:
        w = wa or wb
        r = math.radians(w.transform.rotation.yaw)
        return math.cos(r), math.sin(r)
    dx = wb.transform.location.x - wa.transform.location.x
    dy = wb.transform.location.y - wa.transform.location.y
    n = math.hypot(dx, dy) or 1.0
    return dx / n, dy / n


# --- trial time ----------------------------------------------------------------------------------------------------

def trial_data(mdir, tc, log=print):
    """scenery.json-like data of the stalls taken at the trial's local time ({meshes, items, cell_m}), or None."""
    pc = (tc.get("simulation") or {}).get("parked") or {}
    f = Path(mdir) / "parked.json"
    if not pc.get("enabled", True) or not ((tc.get("map") or {}).get("scenery") or {}).get("parked", True) \
            or not f.exists():
        return None
    data = json.loads(f.read_text())
    from datetime import datetime
    from . import transit
    bb = tc["map"]["bbox"]
    text = str((tc.get("simulation") or {}).get("time") or "").strip()
    when = None
    for fmt in ("%Y-%m-%d %H:%M", "%Y-%m-%dT%H:%M"):     # a full date and time needs no clock of the place
        try:
            when = datetime.strptime(text, fmt)
        except ValueError:
            pass
    if when is None:
        try:
            when = transit.trial_time(text or None, ((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2), Path(mdir) / "real")
        except Exception:  # noqa: BLE001 - offline: 08:00 on a weekday
            when = transit.trial_time(None)
    day, hour = transit.day_type(when), when.hour
    fixed = pc.get("occupancy", "auto")
    occ = {z: (float(fixed) if fixed not in (None, "auto") else occupancy(z, day, hour)) for z in ZONES}
    items = [[s[0], s[1], s[2], s[3], s[4], 1.0, "parked"] for s in data["slots"] if s[6] < occ[ZONES[s[5]]]]
    n = len(data["slots"])
    log(f"parked: {len(items)} of {n} stalls taken at {when:%a %H:%M} ({day}; occupancy "
        f"{', '.join(f'{z} {occ[z]:.2f}' for z in ZONES)})")
    return {"meshes": data["meshes"], "items": items, "cell_m": data.get("cell_m", 100.0), "catalog": data["catalog"]}


def scenery(client, world, mdir, tc, log=print, on_spawn=None):
    """A scenery.Scenery streaming the parked cars of this trial around the participants, or None."""
    from .scenery import Scenery
    data = trial_data(mdir, tc, log)
    if not data or not data["items"]:
        return None
    pc = (tc.get("simulation") or {}).get("parked") or {}
    return Scenery(client, world, mdir, {"radius_m": float(pc.get("radius_m", 250.0)), "kinds": ["parked"]}, log,
                   on_spawn=on_spawn, data=data)
