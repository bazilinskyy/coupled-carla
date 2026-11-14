"""The Dutch BGT (Basisregistratie Grootschalige Topografie) for maps in the Netherlands: what lies beside and between
the roads, and the street furniture the municipalities register.

PDOK serves the BGT as an OGC API Features service (CC0, no key). Two parts are used:

- Surfaces, the BGT's gap-free partition of the ground: wegdeel (roads, pavements, cycle paths, parking),
  ondersteunendwegdeel (traffic islands, verges), onbegroeidterreindeel (yards, squares, sand),
  begroeidterreindeel (green) and waterdeel, with their function (rijbaan, voetpad, fietspad, parkeervlak, ...)
  and physical appearance (gesloten verharding = asphalt or concrete, open verharding = bricks or tiles, half
  verhard, groenvoorziening, grasland, bos, ...). scenery.py picks each ground tile's mesh from them.
- Plus objects (IMGeo, registered by most larger municipalities): paal (lamp posts, bollards), straatmeubilair
  (benches, bicycle racks, parking meters, play equipment), bak (litter bins, waste containers), kast (street
  cabinets), bord (boards) and vegetatieobject (trees). They become OSM-like nodes that replace the OSM objects
  of the same kind nearby.

Only current objects (eind_registratie IS NULL) at ground level (relatieve_hoogteligging 0) are kept. The result is
cached in maps/<name>/real/bgt.json; polygons are simplified to 0.2 m.
"""
import json
import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import date

import requests

API = "https://api.pdok.nl/kadaster/bgt/ogc/v1/collections/{}/items"
RD = "http://www.opengis.net/def/crs/EPSG/0/28992"
SURFACES = ["wegdeel", "ondersteunendwegdeel", "onbegroeidterreindeel", "begroeidterreindeel", "waterdeel"]
OBJECTS = ["paal", "straatmeubilair", "bak", "kast", "bord", "vegetatieobject_punt"]
SIMPLIFY_M = 0.2

# --- surfaces --------------------------------------------------------------------------------------------------

# wegdeel functions CARLA draws itself (its road meshes from the OpenDRIVE): not ground
ROAD = {"OV-baan", "overweg", "woonerf", "verbindingsweg", "calamiteitendoorsteek", "baan voor vliegverkeer"}
OPEN = {"open verharding", "erf"}                 # bricks, concrete pavers, tiles (klinkers); yards
CLOSED = {"gesloten verharding"}                  # asphalt, concrete
UNPAVED = {"half verhard", "onverhard", "zand"}   # gravel, shell, earth, sand
GREEN = {"groenvoorziening"}                      # planted green on a road or verge; all begroeidterreindeel
WOOD = {"loofbos": "wood", "naaldbos": "wood", "gemengd bos": "wood", "houtwal": "scrub", "struiken": "scrub"}


def surface_kind(coll, functie, look):
    """A BGT surface -> "green", "open" (bricks, tiles), "closed" (asphalt, concrete), "unpaved", "water",
    "road" (under CARLA's road meshes) or None (unknown)."""
    if coll == "waterdeel":
        return "water"
    if coll == "begroeidterreindeel":
        return "green"
    f, v = (functie or "").strip(), (look or "").strip()
    if f.startswith("rijbaan") or f in ROAD:
        return "road"
    if f == "spoorbaan":
        return "closed"       # grey ballast: CARLA's grey concrete is nearer than the earth of "unpaved"
    if v in GREEN or v.startswith("grasland"):
        return "green"
    if v in CLOSED:
        return "closed"
    if v in OPEN:
        return "open"
    if v in UNPAVED:
        return "unpaved"
    return None


# --- plus objects ------------------------------------------------------------------------------------------------

# (collection, plus_type) -> OSM-like tags (scenery.FURNITURE picks the mesh); None: every plus_type
PLUS = {
    ("paal", "lichtmast"): {"highway": "street_lamp"},
    ("paal", "afsluitpaal"): {"barrier": "bollard"},
    ("paal", "poller"): {"barrier": "bollard"},
    ("straatmeubilair", "bank"): {"amenity": "bench"},
    ("straatmeubilair", "fietsenrek"): {"amenity": "bicycle_parking"},
    ("straatmeubilair", "brievenbus"): {"amenity": "post_box"},
    ("straatmeubilair", "betaalautomaat"): {"man_made": "street_cabinet", "vending": "parking_tickets"},
    ("straatmeubilair", "speelvoorziening"): {"playground": "swing"},     # or slide, by id
    ("bak", "afvalbak"): {"amenity": "waste_basket"},
    ("bak", "afval apart plaats"): {"amenity": "recycling"},
    ("kast", None): {"man_made": "street_cabinet"},
    ("bord", "reclamebord"): {"advertising": "board"},
    ("bord", "informatiebord"): {"tourism": "information", "information": "board"},
    ("vegetatieobject_punt", "boom"): {"natural": "tree"},
}
# left out: lamp posts' traffic-light, sign and push-button poles (drawn from SUMO and OSM), flag poles,
# art (kunstobject), signposts (no matching mesh), and putten: drains and the Dutch fire hydrants are underground
KINDS = [("highway", "street_lamp"), ("natural", "tree"), ("barrier", "bollard"), ("amenity", "bench"),
         ("amenity", "bicycle_parking"), ("amenity", "post_box"), ("amenity", "waste_basket"),
         ("amenity", "recycling"), ("man_made", "street_cabinet"), ("advertising", "board"),
         ("information", "board"), ("playground", "swing"), ("playground", "slide")]
REPLACE_M = {"highway=street_lamp": 5.0, "amenity=bicycle_parking": 8.0}    # else 3 m
SPACING_M = {"amenity=bicycle_parking": 4.0, "natural=tree": 1.5}            # else 0.8 m (BGT duplicates)


def tags_of(coll, plus_type):
    t = PLUS.get((coll, plus_type)) or PLUS.get((coll, None))
    return dict(t) if t else None


def kind_of(tags):
    """The kind ("amenity=bench") an OSM or BGT node is merged by, or None."""
    for k, v in KINDS:
        if tags.get(k) == v:
            return f"{k}={v}"
    return None


# --- fetching ----------------------------------------------------------------------------------------------------

def log_default(msg):
    print(f"[bgt] {msg}", flush=True)


def _pages(coll, bbox_lonlat, ua, stop=None, timeout=120, tries=3):
    """All current features of a collection in a lon/lat box, in RD New (EPSG:28992), following the next links.
    Gives up when another collection failed (stop is set), so an offline build falls back at once."""
    w, s, e, n = bbox_lonlat
    url, params, out = API.format(coll), {"f": "json", "bbox": f"{w},{s},{e},{n}", "limit": 1000, "crs": RD,
                                          "filter": "eind_registratie IS NULL", "filter-lang": "cql2-text"}, []
    while url:
        for attempt in range(tries):
            if stop is not None and stop.is_set():
                raise RuntimeError("BGT: another request failed")
            try:
                r = requests.get(url, params=params, headers=ua, timeout=timeout)
                r.raise_for_status()
                page = r.json()
                break
            except (requests.RequestException, ValueError) as err:
                if attempt == tries - 1 or isinstance(err, requests.ConnectionError):
                    if stop is not None:
                        stop.set()
                    raise RuntimeError(f"BGT {coll}: {err}") from err
                time.sleep(5 * (attempt + 1))
        out += page.get("features", [])
        url = next((l["href"] for l in page.get("links", []) if l.get("rel") == "next"), None)
        params = None
    return out


def _quarters(bbox):
    """[s, w, n, e] -> four lon/lat boxes (w, s, e, n), fetched in parallel (big collections page slowly)."""
    s, w, n, e = bbox
    ms, mw = (s + n) / 2, (w + e) / 2
    return [(w, s, mw, ms), (mw, s, e, ms), (w, ms, mw, n), (mw, ms, e, n)]


def fetch(bbox, path, ua, log=log_default, workers=6):
    """{surfaces: [{c, f, v, r: [ring, hole...]}], objects: [{c, y, lon, lat}], counts} for a box in the
    Netherlands, cached in path; None elsewhere. Raises when the service fails (the caller falls back)."""
    if path.exists():
        return json.loads(path.read_text())
    from pyproj import Transformer
    from shapely.geometry import shape
    t0 = time.time()
    tasks = [(c, q) for c in SURFACES for q in _quarters(bbox)] + \
        [(c, (bbox[1], bbox[0], bbox[3], bbox[2])) for c in OBJECTS]
    stop = threading.Event()
    with ThreadPoolExecutor(workers) as ex:
        results = list(ex.map(lambda t: (t[0], _pages(t[0], t[1], ua, stop)), tasks))
    to_ll = Transformer.from_crs("EPSG:28992", "EPSG:4326", always_xy=True)
    seen, surfaces, objects, counts = set(), [], [], {}
    for coll, feats in results:
        for f in feats:
            p, g = f.get("properties") or {}, f.get("geometry")
            key = p.get("lokaal_id") or f.get("id")
            if g is None or key in seen:
                continue
            seen.add(key)
            if (p.get("relatieve_hoogteligging") or 0) != 0:
                continue        # bridges, viaducts, tunnels: not the ground
            counts[coll] = counts.get(coll, 0) + 1
            if coll in OBJECTS:
                if g["type"] != "Point":
                    continue
                lon, lat = to_ll.transform(*g["coordinates"][:2])
                objects.append({"c": coll, "y": p.get("plus_type") or p.get("type"),
                                "lon": round(lon, 7), "lat": round(lat, 7)})
                continue
            geom = shape(g)
            if not geom.is_valid:
                geom = geom.buffer(0)
            geom = geom.simplify(SIMPLIFY_M, preserve_topology=True)
            for poly in getattr(geom, "geoms", [geom]):
                if poly.geom_type != "Polygon" or poly.is_empty:
                    continue
                rings = []
                for ring in [poly.exterior] + list(poly.interiors):
                    xs, ys = ring.coords.xy
                    lon, lat = to_ll.transform(list(xs), list(ys))
                    rings.append([[round(a, 7), round(b, 7)] for a, b in zip(lon, lat)])
                surfaces.append({"c": coll, "f": p.get("functie"), "v": p.get("fysiek_voorkomen"), "r": rings})
    data = {"version": 1, "fetched": date.today().isoformat(), "bbox": list(bbox), "counts": counts,
            "surfaces": surfaces, "objects": objects}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, separators=(",", ":")))
    log(f"BGT: {len(surfaces)} surfaces, {len(objects)} plus objects at ground level in {time.time() - t0:.0f} s "
        f"({', '.join(f'{k} {v}' for k, v in sorted(counts.items()))})")
    return data


# --- using it in scenery.py ------------------------------------------------------------------------------------

def _xy_many(geo, pts):
    """[(lon, lat)] -> [(x, y)] in CARLA coordinates (vectorised through the net's projection when there is one)."""
    net = getattr(geo, "net", None)
    if net is not None and hasattr(net, "getGeoProj"):
        lon, lat = zip(*pts)
        xs, ys = net.getGeoProj()(list(lon), list(lat))
        xo, yo = net.getLocationOffset()
        return [(x + xo, -(y + yo)) for x, y in zip(xs, ys)]
    return [geo.to_carla(lat, lon) for lon, lat in pts]


class Surfaces:
    """BGT surfaces in CARLA coordinates: which kind of ground covers most of a square."""

    def __init__(self, data, geo):
        from shapely.geometry import Polygon
        from shapely.strtree import STRtree
        self.polys, self.kinds, self.wood = [], [], []
        for s in data.get("surfaces", []):
            kind = surface_kind(s["c"], s.get("f"), s.get("v"))
            if kind is None:
                continue
            rings = [_xy_many(geo, r) for r in s["r"] if len(r) >= 4]
            if not rings:
                continue
            poly = Polygon(rings[0], rings[1:])
            if not poly.is_valid:
                poly = poly.buffer(0)
            if poly.is_empty:
                continue
            self.polys.append(poly)
            self.kinds.append(kind)
            if s["c"] == "begroeidterreindeel" and s.get("v") in WOOD:
                self.wood.append((poly, WOOD[s["v"]]))
        self.tree = STRtree(self.polys) if self.polys else None

    def __len__(self):
        return len(self.polys)

    def shares(self, x, y, size):
        """{kind: area} of the BGT surfaces inside the square of this size centred on (x, y)."""
        from shapely.geometry import box
        out = {}
        if self.tree is None:
            return out
        sq = box(x - size / 2, y - size / 2, x + size / 2, y + size / 2)
        for i in self.tree.query(sq):
            a = self.polys[i].intersection(sq).area
            if a > 0:
                out[self.kinds[i]] = out.get(self.kinds[i], 0.0) + a
        return out

    def classify(self, x, y, size, min_cover=0.25):
        """(kind, share) of the ground kind covering most of the square beside the roads, or (None, 0) where the
        BGT covers less than min_cover of it (outside the fetched box) or only road."""
        sh = self.shares(x, y, size)
        if sum(sh.values()) < min_cover * size * size:
            return None, 0.0
        ground = {k: a for k, a in sh.items() if k != "road"}
        total = sum(ground.values())
        if total <= 0.02 * size * size:
            return None, 0.0
        kind = max(ground, key=ground.get)
        return kind, ground[kind] / total


def merge(els, data, geo, skip_lamps=False, skip_trees=False):
    """OSM (and municipal) nodes plus the BGT's plus objects: a BGT object replaces the OSM nodes of its kind within
    a few metres (REPLACE_M); BGT duplicates closer than SPACING_M are skipped. Lamps and trees are skipped when a
    municipal register already gives them. Returns (elements, stats)."""
    from shapely.geometry import Point
    from shapely.strtree import STRtree
    objs = data.get("objects", [])
    xy = _xy_many(geo, [(o["lon"], o["lat"]) for o in objs]) if objs else []
    by_kind, grids, new = {}, {}, []
    stats = {"bgt_objects": len(objs), "added": {}, "replaced": {}, "skipped_register": 0, "too_close": 0}
    for i, (o, (x, y)) in enumerate(zip(objs, xy)):
        tags = tags_of(o["c"], o.get("y"))
        if tags is None:
            continue
        if tags.get("playground"):
            tags["playground"] = ("swing", "slide")[i % 2]
        k = kind_of(tags)
        if (skip_lamps and k == "highway=street_lamp") or (skip_trees and k == "natural=tree"):
            stats["skipped_register"] += 1
            continue
        gap = SPACING_M.get(k, 0.8)
        grid = grids.setdefault(k, {})
        gx, gy = int(x // gap), int(y // gap)
        if any((px - x) ** 2 + (py - y) ** 2 < gap * gap
               for i_ in (-1, 0, 1) for j_ in (-1, 0, 1) for px, py in grid.get((gx + i_, gy + j_), ())):
            stats["too_close"] += 1
            continue
        grid.setdefault((gx, gy), []).append((x, y))
        by_kind.setdefault(k, []).append((x, y))
        tags["source"] = f"bgt-{o['c']}"
        new.append({"type": "node", "id": -4_000_000 - i, "lat": o["lat"], "lon": o["lon"], "tags": tags})
        stats["added"][k] = stats["added"].get(k, 0) + 1
    trees = {k: (STRtree([Point(p) for p in pts]), pts) for k, pts in by_kind.items() if pts}
    nodes = [el for el in els if el.get("type") == "node"]
    nxy = _xy_many(geo, [(el["lon"], el["lat"]) for el in nodes]) if nodes else []
    pos = {id(el): p for el, p in zip(nodes, nxy)}
    out = []
    for el in els:
        k = kind_of(el.get("tags", {})) if el.get("type") == "node" else None
        if k in trees and not str(el.get("tags", {}).get("source", "")).startswith("bgt"):
            tree, pts = trees[k]
            r = REPLACE_M.get(k, 3.0)
            p = Point(pos[id(el)])
            if any(math.dist(pts[j], (p.x, p.y)) < r for j in tree.query(p.buffer(r))):
                stats["replaced"][k] = stats["replaced"].get(k, 0) + 1
                continue
        out.append(el)
    return out + new, stats
