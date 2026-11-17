"""Street life that makes a street read as Dutch (CARLA 1.0, the CoupledCarla content): clinker roads, grime and
litter, parked bikes everywhere, advertising columns, shop signs.

What made an "authentically Amsterdam" street read as Dutch: brick clinker road paving on residential streets,
woonerven and canal streets; dirt in the gutters, litter and stains; bicycles parked everywhere (against facades,
railings and lamp posts, not only in racks, with Swapfiets' blue front tyre on about 1 in 8 of them); advertising
columns and shop signs. This module places those, from open data only, as static.prop.mesh scenery (meshes of
tools/street_props.py in /Game/CoupledCarla/Street; a server without them spawns nothing or CARLA's own prop):

- clinker roads: the BGT's carriageways of "open verharding" (Dutch roads of bricks or concrete pavers) and OSM
  roads tagged surface=paving_stones/bricks/clinker (in the Netherlands) get red-brown clay clinkers in herringbone;
  surface=sett/cobblestone/unhewn_cobblestone (anywhere) grey granite setts. One strip across the carriageway (2,
  3, 4, 6, 8 or 12 m meshes at about scale 1, so the stones keep their 200 x 100 mm) every 1.9 widths along the
  road, 9 mm above it.
- grime and litter: a dirt band in the gutter along the kerb (more on shopping streets), litter (cigarette butts,
  paper, cans) and chewing-gum stains near bins, benches, cafes, snack bars and stations, puddles in the gutters
  shown only when the trial's road is wet (Scenery: weather wetness or rain).
- parked bikes: Dutch city bikes (step-through and diamond frames in five colours; BIKE_SWAPFIETS of them
  Swapfiets with the blue front tyre: 12 %) in the bike stands (occupancy by context), in rows against facades
  near shops, cafes and stations (BIKES_AT), a few against lamp posts; leaning on their kickstands.
- advertising columns where OSM maps advertising=column (three poster sets; any country).
- shop signs: a fascia board with the trade's Dutch word (BAKKERIJ, CAFÉ, APOTHEEK, ...: generic words, no brand)
  on the facade nearest each OSM shop, cafe or restaurant, at 3.2 m; on baked twins only by default (their
  buildings are the real ones; CARLA's building meshes on OSM maps do not reach the footprint's walls).
All placements are repeatable (hashes of positions), and fail soft without data.
"""
import json
import math
import zlib
from pathlib import Path
from .compat import left_lane, right_lane

BIKE_COLOURS = ("Black", "Navy", "Grey", "Green", "Red")
BIKE_COLOUR_SHARE = (0.46, 0.18, 0.14, 0.12, 0.10)     # black dominates Dutch city bikes
BIKE_SWAPFIETS = 0.12      # Swapfiets' share of bikes in Dutch cities (about 1 in 8)
BIKE_MEN = 0.4             # diamond ("opafiets") frames; the rest step-through
BIKES_AT = {               # (cluster chance per facade stretch, bikes per cluster min, max)
    "station": (0.95, 5, 14), "shop": (0.6, 2, 7), "cafe": (0.6, 2, 6), "home": (0.12, 1, 3)}
STAND_OCCUPANCY = {"station": 0.95, "shop": 0.75, "home": 0.5}
LAMP_BIKE = 0.05           # share of lamp posts near shops with a bike leaning against them
CLINKER_OSM = {"paving_stones", "bricks", "brick", "clinker", "paved:bricks"}
SETT_OSM = {"sett", "cobblestone", "unhewn_cobblestone", "cobblestone:flattened"}
SURFACE_WIDTHS = (2, 3, 4, 6, 8, 12)
SIGN_OF = {   # (OSM key, value) -> sign key (tools/street_props.SIGN_WORDS)
    ("shop", "bakery"): "bakkerij", ("amenity", "cafe"): "cafe", ("amenity", "bar"): "cafe",
    ("amenity", "pub"): "cafe", ("amenity", "restaurant"): "restaurant", ("shop", "hairdresser"): "kapper",
    ("amenity", "pharmacy"): "apotheek", ("shop", "chemist"): "apotheek", ("shop", "clothes"): "mode",
    ("shop", "fashion"): "mode", ("shop", "boutique"): "mode", ("shop", "bicycle"): "fietsen",
    ("shop", "florist"): "bloemen", ("shop", "supermarket"): "supermarkt", ("shop", "convenience"): "supermarkt",
    ("shop", "butcher"): "slagerij", ("shop", "optician"): "optiek", ("amenity", "fast_food"): "snackbar",
    ("shop", "books"): "boeken", ("shop", "shoes"): "schoenen", ("amenity", "ice_cream"): "lunchroom",
    ("shop", "deli"): "lunchroom"}


def _rng(*parts):
    return zlib.crc32("|".join(str(p) for p in parts).encode()) / 2 ** 32


def _on(sc, key, default="auto"):
    return sc.get(key, default) not in (False, "off", "false", None)


# --- data ------------------------------------------------------------------------------------------------------------

def osm_streetlife(bbox, path):
    """OSM shops, cafes and stations (nodes and building centres) and roads with a stone surface, cached."""
    if path.exists():
        return json.loads(path.read_text())
    from . import realworld as rw
    s, w, n, e = bbox
    b = f"({s},{w},{n},{e})"
    data = rw.overpass_json(
        f'[out:json][timeout:120];('
        f'nwr["shop"]{b};nwr["amenity"~"^(cafe|restaurant|bar|pub|fast_food|pharmacy|ice_cream)$"]{b};'
        f'nwr["railway"~"^(station|halt)$"]{b};nwr["public_transport"="station"]{b};'
        f'way["highway"]["surface"~"^(paving_stones|bricks|brick|clinker|sett|cobblestone|unhewn_cobblestone|'
        f'cobblestone:flattened)$"]{b};);out center geom tags;')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, separators=(",", ":")))
    return data


def pois(data, geo):
    """[(x, y, kind, tags)] with kind "station", "cafe" (cafes, bars, restaurants, snack bars) or "shop"."""
    out = []
    for el in data.get("elements", []):
        t = el.get("tags", {})
        if "lat" in el:
            lat, lon = el["lat"], el["lon"]
        elif el.get("center"):
            lat, lon = el["center"]["lat"], el["center"]["lon"]
        else:
            continue
        if t.get("railway") in ("station", "halt") or t.get("public_transport") == "station":
            kind = "station"
        elif t.get("amenity") in ("cafe", "restaurant", "bar", "pub", "fast_food", "ice_cream"):
            kind = "cafe"
        elif t.get("shop") or t.get("amenity") == "pharmacy":
            kind = "shop"
        else:
            continue
        x, y = geo.to_carla(lat, lon)
        out.append((x, y, kind, t))
    return out


class Near:
    """Points by kind on a grid, for 'is there a station within r m' questions."""

    def __init__(self, pts, cell=50.0):
        self.cell, self.grid = cell, {}
        for x, y, kind, *_ in pts:
            self.grid.setdefault((int(x // cell), int(y // cell)), []).append((x, y, kind))

    def kinds(self, x, y, r):
        out = set()
        k = int(math.ceil(r / self.cell))
        cx, cy = int(x // self.cell), int(y // self.cell)
        for i in range(-k, k + 1):
            for j in range(-k, k + 1):
                for px, py, kind in self.grid.get((cx + i, cy + j), ()):
                    if math.hypot(px - x, py - y) <= r:
                        out.add(kind)
        return out

    def context(self, x, y):
        """"station" within 150 m, else "cafe" or "shop" within 30 m, else "home"."""
        if "station" in self.kinds(x, y, 150.0):
            return "station"
        k = self.kinds(x, y, 30.0)
        return "cafe" if "cafe" in k else "shop" if "shop" in k else "home"


class Taken:
    """Positions already used by props (centre, radius), so bikes and litter do not stand in a bench or a tree."""

    def __init__(self, pl, cell=2.0):
        self.cell, self.grid = cell, {}
        skip = {"ground", "water", "surface", "wear", "marking", "leaves", "building", "landmark", "roof"}
        for it in pl.items:
            if it[6] not in skip:
                self.add(it[1], it[2], 0.5 if it[6] != "tree" else 0.8)

    def add(self, x, y, r):
        self.grid.setdefault((int(x // self.cell), int(y // self.cell)), []).append((x, y, r))

    def free(self, x, y, r):
        cx, cy = int(x // self.cell), int(y // self.cell)
        return not any(math.hypot(px - x, py - y) < pr + r for i in (-1, 0, 1) for j in (-1, 0, 1)
                       for px, py, pr in self.grid.get((cx + i, cy + j), ()))


# --- clinker and sett roads -----------------------------------------------------------------------------------------

def strip_mesh(kind, width, cat):
    """(mesh, scale) of the clinker or sett strip whose width is nearest `width` (scale within 0.8-1.25), or None."""
    pre = "SM_CC_NL_Clinker_W" if kind == "clinker" else "SM_CC_Any_Sett_W"
    have = [w for w in SURFACE_WIDTHS if f"{pre}{w}" in cat]
    if not have:
        return None
    w = min(have, key=lambda v: abs(math.log(max(0.5, width) / v)))
    return f"{pre}{w}", round(max(0.8, min(1.25, width / w)), 3)


def stone_roads(pl, cat, cmap, carla, geo, xodr, bgt_data, ways, region):
    """Clinker/sett strips on the driving and parking lanes of roads paved with bricks or setts: the BGT's
    carriageways of open paving (NL) and OSM ways with such a surface tag. Returns {kind: strips}."""
    from shapely.geometry import LineString, Point, Polygon
    from shapely.strtree import STRtree
    from .streetlook import along, xodr_lanes
    shapes, kinds = [], []
    if region == "NL" and bgt_data:
        for srf in bgt_data.get("surfaces", []):
            f = srf.get("f") or ""
            if srf.get("c") == "wegdeel" and (f.startswith("rijbaan") or f in ("woonerf", "erf", "parkeervlak")) \
                    and srf.get("v") == "open verharding":
                for ring in srf.get("r", [])[:1]:
                    try:
                        poly = Polygon([geo.to_carla(lat, lon) for lon, lat in ring]).buffer(0.5)
                    except Exception:  # noqa: BLE001
                        continue
                    if not poly.is_empty:
                        shapes.append(poly)
                        kinds.append("clinker")
    for w in ways:
        surf = (w.get("tags") or {}).get("surface", "")
        kind = "sett" if surf in SETT_OSM else "clinker" if (surf in CLINKER_OSM and region == "NL") else None
        if kind is None or len(w.get("geometry") or []) < 2:
            continue
        line = LineString([geo.to_carla(g["lat"], g["lon"]) for g in w["geometry"]])
        shapes.append(line.buffer(4.0))
        kinds.append(kind)
    if not shapes:
        return {}
    tree = STRtree(shapes)
    # one strip across the whole carriageway (all its driving, parking and bidirectional lanes) per step along the
    # road's first right lane (else its first left one): a quarter of the actors of one strip per lane
    allowed = (carla.LaneType.Driving, carla.LaneType.Parking, carla.LaneType.Bidirectional)
    lanes = xodr_lanes(xodr, ("driving", "parking", "bidirectional"))
    firsts = {}
    for rid, lid, s0, s1 in lanes:
        if abs(lid) == 1:
            k = (rid, s0)
            if k not in firsts or lid == -1:
                firsts[k] = (rid, lid, s0, s1)
    out = {}

    def width_of(wp):
        tf = wp.transform
        r = tf.get_right_vector()
        o = tf.location
        lo, hi = [], []
        for start, step_ in ((wp, "get_right_lane"), (wp, "get_left_lane")):
            w, seen = start, 0
            while w is not None and seen < 8 and w.lane_type in allowed:
                c, half = w.transform.location, w.lane_width / 2
                d = (c.x - o.x) * r.x + (c.y - o.y) * r.y
                lo.append(d - half)
                hi.append(d + half)
                w = getattr(w, step_)()
                seen += 1
        return (min(lo), max(hi)) if lo else (-wp.lane_width / 2, wp.lane_width / 2)
    for wp in along(cmap, list(firsts.values()), lambda w: 1.9 * max(1.5, sum(map(abs, width_of(w))))):
        loc = wp.transform.location
        a, b = width_of(wp)
        W = b - a
        r = wp.transform.get_right_vector()
        cx, cy = loc.x + r.x * (a + b) / 2, loc.y + r.y * (a + b) / 2
        p = Point(cx, cy)
        hit = [kinds[i] for i in tree.query(p) if shapes[i].contains(p)]
        if not hit:
            continue
        kind = "sett" if "sett" in hit else "clinker"
        got = strip_mesh(kind, W, cat)
        if got is None:
            continue
        pl.add("surface", got[0], cx, cy, wp.transform.rotation.yaw, got[1], z=loc.z)
        out[kind] = out.get(kind, 0) + 1
    return out


# --- grime, litter, puddles -------------------------------------------------------------------------------------

def grime_and_litter(pl, cat, cmap, carla, near, taken, lefthand=False, step=6.0):
    """Gutter grime (and puddles) along the kerb of the outermost driving lanes, litter and stains beside them on
    busier streets and around bins, benches, cafes and stations. Returns {kind: count}."""
    have = {k: m for k, m in (("grime", "SM_CC_Any_Grime_A"), ("puddle", "SM_CC_Any_Puddle_A"),
                              ("litter", "SM_CC_Any_Litter_A"), ("stains", "SM_CC_Any_Stains_A")) if m in cat}
    out = {}
    if not have:
        return out

    def put(kind, x, y, yaw, z=None, s=1.0, what="wear"):
        if kind in have:
            pl.add(what, have[kind], x, y, yaw, s, z=z)
            out[kind] = out.get(kind, 0) + 1
    for wp in cmap.generate_waypoints(step):
        if wp.lane_type != carla.LaneType.Driving or wp.is_junction:
            continue
        side = left_lane(wp) if lefthand else right_lane(wp)
        if side is not None and side.lane_type == carla.LaneType.Driving and side.lane_id * wp.lane_id > 0:
            continue        # not the kerb lane
        tf, loc = wp.transform, wp.transform.location
        r = tf.get_right_vector()
        k = -1.0 if lefthand else 1.0
        ctx = near.context(loc.x, loc.y)
        key = (wp.road_id, wp.lane_id, round(wp.s))
        busy = ctx in ("shop", "cafe", "station")
        if _rng(key, "grime") < (0.8 if busy else 0.45):
            off = k * (wp.lane_width / 2 - 0.3)
            # the band's dark edge (mesh -y in Unreal) towards the kerb
            put("grime", loc.x + r.x * off, loc.y + r.y * off, tf.rotation.yaw + (180.0 if not lefthand else 0.0),
                z=loc.z)
        if _rng(key, "puddle") < 0.12:
            off = k * (wp.lane_width / 2 - 0.55)
            put("puddle", loc.x + r.x * off, loc.y + r.y * off, _rng(key, "py") * 360, z=loc.z,
                s=0.7 + 0.6 * _rng(key, "ps"), what="puddle")
        if busy and _rng(key, "litter") < 0.35:        # on the pavement beyond the kerb
            off = k * (wp.lane_width / 2 + 1.0 + 1.5 * _rng(key, "lo"))
            x, y = loc.x + r.x * off, loc.y + r.y * off
            if not pl.on_lane(x, y, loc.z):
                put("litter" if _rng(key, "lk") < 0.6 else "stains", x, y, _rng(key, "ly") * 360)
    # around bins, benches and bus stops, and at cafes and stations
    for it in list(pl.items):
        name = pl_name(pl, it[0])
        if it[6] != "furniture" or not any(s in name for s in ("Bin", "TrashCan", "trashCan", "Bench", "BusStop")):
            continue
        for i in range(1 + int(_rng(it[1], it[2]) < 0.4)):
            a = _rng(it[1], it[2], i) * 2 * math.pi
            d = 0.8 + 1.2 * _rng(it[2], it[1], i)
            x, y = it[1] + math.cos(a) * d, it[2] + math.sin(a) * d
            if not pl.on_lane(x, y, it[3]):
                put("litter" if i == 0 else "stains", x, y, a * 57.3)
    for x0, y0, kind in near.points(("cafe", "station")):
        for i in range(2 if kind == "cafe" else 6):
            a = _rng(x0, y0, i) * 2 * math.pi
            d = 1.5 + (2.0 if kind == "cafe" else 12.0) * _rng(y0, x0, i)
            x, y = x0 + math.cos(a) * d, y0 + math.sin(a) * d
            if not pl.on_lane(x, y, 0.0):
                put("stains" if _rng(x, y) < 0.5 else "litter", x, y, a * 57.3)
    return out


def pl_name(pl, idx):
    if not hasattr(pl, "_names") or len(pl._names) != len(pl.mesh_index):
        pl._names = {i: m for m, i in pl.mesh_index.items()}
    return pl._names.get(idx, "")


# --- parked bikes ---------------------------------------------------------------------------------------------------

def bike_mesh(cat, *key):
    """A Dutch city bike for a spot (repeatable): colour by BIKE_COLOUR_SHARE, BIKE_SWAPFIETS Swapfiets, BIKE_MEN
    diamond frames. None when the catalogue lacks them."""
    men = _rng(*key, "frame") < BIKE_MEN
    pre = "SM_CC_NL_BikeMen_" if men else "SM_CC_NL_Bike_"
    if _rng(*key, "swap") < BIKE_SWAPFIETS:
        name = pre + "Swapfiets"
    else:
        u, acc, col = _rng(*key, "colour"), 0.0, BIKE_COLOURS[0]
        for c, share in zip(BIKE_COLOURS, BIKE_COLOUR_SHARE):
            acc += share
            if u < acc:
                col = c
                break
        name = pre + col
    return name if name in cat else None


def bike_fits(pl, taken, free, x, y, yaw, z):
    """A parked bike (1.9 x 0.7 m) fits here: off the lanes and buildings, clear of other props."""
    r = math.radians(yaw)
    ux, uy = math.cos(r), math.sin(r)
    pts = [(x + ux * t, y + uy * t) for t in (-0.75, 0.0, 0.75)]
    return all(free(px, py, 0.2) and taken.free(px, py, 0.35) for px, py in pts)


def place_bike(pl, cat, taken, x, y, yaw, z, key, out, where):
    mesh = bike_mesh(cat, *key)
    if mesh is None:
        return False
    pl.add("furniture", mesh, x, y, yaw, 1.0, z=z)
    r = math.radians(yaw)
    for t in (-0.6, 0.0, 0.6):
        taken.add(x + math.cos(r) * t, y + math.sin(r) * t, 0.3)
    out[where] = out.get(where, 0) + 1
    if "Swapfiets" in mesh:
        out["swapfiets"] = out.get("swapfiets", 0) + 1
    return True


def parked_bikes(pl, cat, blds, near, taken, free, max_bikes=4000):
    """Bikes in the bike stands (by the stands' context: stations nearly full), in rows against facades near shops,
    cafes and stations (and a few at homes), and against some lamp posts near shops. Returns {where: count}."""
    out = {}
    if bike_mesh(cat, 0) is None:
        return out
    total = 0
    # 1. in the stands: on both sides of each hoop of our nietjes (the row runs along the stand's x)
    for it in list(pl.items):
        name = pl_name(pl, it[0])
        if not name.startswith("SM_CC_NL_Nietje_x"):
            continue
        n = int(name.rsplit("x", 1)[1])
        occ = STAND_OCCUPANCY.get("shop" if near.context(it[1], it[2]) == "cafe" else near.context(it[1], it[2]), 0.5)
        r = math.radians(it[4])
        ux, uy = math.cos(r), math.sin(r)
        for k in range(n):
            off = (k - (n - 1) / 2) * 0.9
            for side in (-0.2, 0.2):
                if _rng(it[1], it[2], k, side) >= occ or total >= max_bikes:
                    continue
                x, y = it[1] + ux * (off + side), it[2] + uy * (off + side)
                total += place_bike(pl, cat, taken, x, y, it[4] + 90.0, it[3], (round(x, 1), round(y, 1)), out,
                                    "stands")
    # 2. against facades: walk each building edge that faces a pavement
    for b in blds:
        if total >= max_bikes:
            break
        poly = b["poly"]
        ring = list(poly.exterior.coords)
        if poly.exterior.is_ccw is False:
            ring = ring[::-1]
        for (ax, ay), (bx, by) in zip(ring, ring[1:]):
            L = math.hypot(bx - ax, by - ay)
            if L < 3.0:
                continue
            ex, ey = (bx - ax) / L, (by - ay) / L
            nx, ny = ey, -ex           # outward normal of a counter-clockwise ring
            mx, my = (ax + bx) / 2 + nx * 0.5, (ay + by) / 2 + ny * 0.5
            ctx = near.context(mx, my)
            chance, lo, hi = BIKES_AT[ctx]
            key = (round(mx), round(my))
            if _rng(key, "cluster") >= chance:
                continue
            k = lo + int(_rng(key, "n") * (hi - lo + 1))
            start = _rng(key, "start") * max(0.0, L - k * 0.7)
            yaw = math.degrees(math.atan2(-ny, -nx)) + 90.0       # the wall on the bike's left (its kickstand side)
            for i in range(k):
                t = start + 0.35 + i * 0.7
                if t > L - 0.3 or total >= max_bikes:
                    break
                x, y = ax + ex * t + nx * 0.45, ay + ey * t + ny * 0.45
                z = pl.ground(x, y)
                if not bike_fits(pl, taken, free, x, y, yaw, z):
                    continue
                total += place_bike(pl, cat, taken, x, y, yaw + (_rng(key, i, "a") - 0.5) * 8, z,
                                    (round(x, 1), round(y, 1)), out, "facades")
    # 3. against lamp posts near shops and cafes
    for it in list(pl.items):
        if it[6] != "lamp" or total >= max_bikes or near.context(it[1], it[2]) not in ("shop", "cafe"):
            continue
        if _rng(it[1], it[2], "lampbike") >= LAMP_BIKE * 4:
            continue
        yaw = _rng(it[2], it[1]) * 360
        r = math.radians(yaw + 90)
        x, y = it[1] + math.cos(r) * 0.45, it[2] + math.sin(r) * 0.45
        if bike_fits(pl, taken, free, x, y, yaw, it[3]):
            total += place_bike(pl, cat, taken, x, y, yaw, it[3], (round(x, 1), round(y, 1)), out, "lamp posts")
    return out


# --- shop signs -----------------------------------------------------------------------------------------------------

def shop_signs(pl, cat, blds, pts, height=3.2, reach=10.0):
    """A fascia sign on the facade nearest each shop / cafe POI (within reach m), facing out, at `height` above the
    ground; one per building edge. Returns the count."""
    from shapely.geometry import Point
    from shapely.strtree import STRtree
    if not blds or not any(f"SM_CC_NL_ShopSign_{k}" in cat for k in ("cafe", "winkel")):
        return 0
    polys = [b["poly"] for b in blds]
    tree = STRtree(polys)
    used, n = set(), 0
    for x, y, kind, tags in pts:
        if kind == "station":
            continue
        key = next((k for (tk, tv), k in SIGN_OF.items() if tags.get(tk) == tv), "winkel")
        mesh = f"SM_CC_NL_ShopSign_{key}"
        if mesh not in cat:
            continue
        p = Point(x, y)
        cand = [i for i in tree.query(p.buffer(reach))]
        if not cand:
            continue
        i = min(cand, key=lambda j: polys[j].exterior.distance(p))
        poly = polys[i]
        if poly.exterior.distance(p) > reach:
            continue
        ring = list(poly.exterior.coords)
        if poly.exterior.is_ccw is False:
            ring = ring[::-1]
        best = min(range(len(ring) - 1), key=lambda k: _seg_dist(ring[k], ring[k + 1], (x, y)))
        if (i, best) in used:
            continue
        (ax, ay), (bx, by) = ring[best], ring[best + 1]
        L = math.hypot(bx - ax, by - ay)
        if L < 3.2:
            continue
        used.add((i, best))
        ex, ey = (bx - ax) / L, (by - ay) / L
        nx, ny = ey, -ex
        t = max(1.6, min(L - 1.6, (x - ax) * ex + (y - ay) * ey))
        sx, sy = ax + ex * t + nx * 0.02, ay + ey * t + ny * 0.02
        yaw = math.degrees(math.atan2(ny, nx)) + 90.0       # the face (mesh Unreal -y) out of the wall
        pl.add("furniture", mesh, sx, sy, yaw, 1.0, z=pl.ground(sx, sy) + height - 0.3)
        n += 1
    return n


def _seg_dist(a, b, p):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy or 1e-9
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


# --- the hook ---------------------------------------------------------------------------------------------------

def add_to_scenery(pl, cat, cmap, carla, geo, mc, sc, mdir, blds, region, bgt_data, free, log=print):
    """streetlook.add_to_scenery calls this: stone roads, grime and litter, parked bikes, shop signs."""
    said = []
    rdir = Path(mdir) / "real"
    try:
        data = osm_streetlife(mc["bbox"], rdir / "osm_streetlife.json")
    except Exception as err:  # noqa: BLE001 - without POIs: no shop context, no signs, no OSM stone roads
        log(f"scenery: OSM shops and paved roads not read ({type(err).__name__}: {err})")
        data = {"elements": []}
    pts = pois(data, geo)
    near = Near(pts)
    near.points = lambda kinds: [(x, y, k) for x, y, k, _ in pts if k in kinds]
    ways = [e for e in data.get("elements", []) if e.get("type") == "way" and (e.get("tags") or {}).get("surface")]
    xodr = Path(mdir) / "map.xodr"
    if _on(sc, "stone_roads") and xodr.exists():
        got = stone_roads(pl, cat, cmap, carla, geo, xodr, bgt_data, ways, region)
        for k, v in got.items():
            pl.counts[f"surface_{k}"] = v
        if got:
            said.append("stone roads " + ", ".join(f"{k} {v} strips" for k, v in sorted(got.items())))
    taken = Taken(pl)
    if _on(sc, "grime"):
        got = grime_and_litter(pl, cat, cmap, carla, near, taken, bool(mc.get("lefthand")))
        for k, v in got.items():
            pl.counts[f"grime_{k}"] = v
        if got:
            said.append("grime " + ", ".join(f"{k} {v}" for k, v in sorted(got.items())))
    bikes = sc.get("parked_bikes", "auto")
    if bikes is True or (bikes == "auto" and region == "NL"):
        got = parked_bikes(pl, cat, blds, near, taken, free)
        for k, v in got.items():
            pl.counts[f"bikes_{k.replace(' ', '_')}"] = v
        if got:
            said.append(f"{sum(v for k, v in got.items() if k != 'swapfiets')} parked bikes ("
                        + ", ".join(f"{k} {v}" for k, v in sorted(got.items())) + ")")
    signs = sc.get("shop_signs", "auto")
    if region == "NL" and (signs is True or (signs == "auto" and mc.get("carla_level"))):
        n = shop_signs(pl, cat, blds, pts)
        pl.counts["shop_signs"] = n
        said.append(f"{n} shop signs on the facades of {len(pts)} shops, cafes and stations")
    if said:
        log("scenery: street life: " + "; ".join(said))
