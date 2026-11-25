"""Where each placed object and baked layer comes from: measured, register, inferred or generated.

- measured: geometry or appearance measured by an instrument (3DBAG, PLATEAU, Catastro, BD TOPO and NYC
  buildings, AHN terrain, aerial-photo ground).
- register: an object recorded at its own position in a public register or map (the city's tree and lamp
  registers, NDW signs, BGT surfaces, OSM-mapped trees, lamps, benches and signs, shared-bike feeds).
- inferred: position or form derived by rules from mapped or measured data (OSM footprints with heights from levels
  or ML, lamps every 32 m along kerbs, trees from the canopy-height map, tree rows every 8 m, shop signs at OSM
  shops, facade era from the building year, ground kind from land cover).
- generated: invented detail with no evidence at its position (trees and bushes filling a park, parked cars and
  bikes, road wear and leaves, roof clutter, interiors).

scenery.json carries one (tag, source) per placement in "prov" ({"keys": [[tag, source], ...], "items": [k, ...]},
aligned with "items"); placements without one get the default of their kind. A trial writes provenance.json:
counts per tag and kind, and the share of "generated" within 50 m of the participants at the start, so analyses can
leave generated detail out. Twins (baked CARLA 1.0 levels) list their layers in twin_layers().
"""
from collections import Counter, defaultdict

TAGS = ("measured", "register", "inferred", "generated")
# national building sources with measured footprints and heights (lidar or photogrammetry); others are inferred
MEASURED_BUILDINGS = {"3dbag", "plateau", "catastro", "bdtopo", "nyc"}
DEFAULT = {
    "building": ("inferred", "osm"), "landmark": ("inferred", "osm"),
    "tree": ("register", "osm"), "bush": ("generated", "green_fill"),
    "lamp": ("register", "osm"), "furniture": ("register", "osm"), "sign": ("register", "osm"),
    "ground": ("inferred", "landcover"), "water": ("register", "osm"),
    "shared": ("register", "gbfs"), "parked": ("generated", "parking"),
    "marking": ("inferred", "lanes"), "surface": ("inferred", "streetlook"), "wear": ("generated", "streetlook"),
    "leaves": ("generated", "season"), "roof": ("generated", "streetlook"), "puddle": ("generated", "streetlife"),
    "facade": ("inferred", "screens"),
}


def default(kind):
    return DEFAULT.get(kind, ("generated", kind or "unknown"))


def building(src):
    """(tag, source) of a building from its footprint source (scenery.building_records' "src")."""
    s = (src or "osm").lower()
    return ("measured", s) if s in MEASURED_BUILDINGS else ("inferred", s)


def element(el, kind=None):
    """(tag, source) of an object placed at an OSM / register / BGT element's own position."""
    src = el.get("_src") or (el.get("tags") or {}).get("source") or "osm"
    return ("register", str(src))


def encode(items, prov_of):
    """scenery.json's "prov" for the placements: prov_of maps id(item) -> (tag, source)."""
    keys, index, out = [], {}, []
    for it in items:
        tag = prov_of.get(id(it)) if it is not None else None
        if tag is None:
            tag = default(it[6] if it is not None and len(it) > 6 else None)
        tag = tuple(tag)
        if tag not in index:
            index[tag] = len(keys)
            keys.append(list(tag))
        out.append(index[tag])
    return {"keys": keys, "items": out}


def of_items(data):
    """[(tag, source)] per item of a scenery.json (or parked) dict, defaults where it has no "prov"."""
    items = data.get("items") or []
    pv = data.get("prov")
    if pv and len(pv.get("items", [])) == len(items):
        keys = [tuple(k) for k in pv["keys"]]
        return [keys[k] for k in pv["items"]]
    return [default(it[6] if it is not None and len(it) > 6 else None) for it in items]


def summary(data, near=None, radius=50.0):
    """Counts per tag, per kind and tag, per source; with near (a list of (x, y)), also the counts within radius."""
    items = data.get("items") or []
    tags = of_items(data)
    by_tag, by_kind, by_src = Counter(), defaultdict(Counter), Counter()
    near_tag = Counter()
    for it, (tag, src) in zip(items, tags):
        if it is None:
            continue
        kind = it[6] if len(it) > 6 else "?"
        by_tag[tag] += 1
        by_kind[kind][tag] += 1
        by_src[f"{tag}:{src}"] += 1
        if near and any((it[1] - x) ** 2 + (it[2] - y) ** 2 <= radius * radius for x, y in near):
            near_tag[tag] += 1
    out = {"total": sum(by_tag.values()), "by_tag": dict(by_tag),
           "by_kind": {k: dict(v) for k, v in sorted(by_kind.items())}, "by_source": dict(by_src.most_common())}
    if near is not None:
        n = sum(near_tag.values())
        out["near"] = {"radius_m": radius, "by_tag": dict(near_tag),
                       "generated_share": round(near_tag["generated"] / n, 3) if n else 0.0}
    return out


# baked CARLA 1.0 twins (carla-digitaltwins TwinModel): their layers by country, as the twin tool builds them
_TWIN = {
    "NL": {"terrain": ("measured", "AHN 0.5 m DTM"), "buildings": ("measured", "3DBAG LoD 2.2"),
           "ground photo": ("measured", "PDOK aerial photo 8 cm"), "roads": ("register", "OSM"),
           "lanes and markings": ("inferred", "lane graph from OSM tags"),
           "facades": ("inferred", "facade era rules from BAG construction year"),
           "building use, lit windows": ("register", "BAG use")},
    "ES": {"terrain": ("measured", "IGN MDT05"), "buildings": ("measured", "Catastro"),
           "ground photo": ("measured", "IGN PNOA 25 cm"), "roads": ("register", "OSM"),
           "lanes and markings": ("inferred", "lane graph from OSM tags"),
           "facades": ("inferred", "materials from the Catastro year and use")},
    "JP": {"terrain": ("measured", "GSI 5 m DEM"), "buildings": ("measured", "PLATEAU LoD 2"),
           "facades": ("measured", "PLATEAU photo textures (oblique aerial photos)"),
           "ground photo": ("measured", "GSI seamless photo"), "roads": ("register", "OSM"),
           "lanes and markings": ("inferred", "lane graph from OSM tags")},
}


def twin_layers(country):
    """{layer: [tag, source]} of a baked twin in that country (ISO 3166-1 alpha-2), {} when not known."""
    return {k: list(v) for k, v in _TWIN.get((country or "").upper(), {}).items()}


_BOXES = {"NL": (50.75, 3.2, 53.7, 7.3), "ES": (35.9, -9.4, 43.9, 4.4), "JP": (24.0, 122.0, 46.0, 146.0)}


def country_at(lat, lon):
    """The twin tool's country (NL, ES, JP) whose rough box holds the point, else None."""
    for cc, (s, w, n, e) in _BOXES.items():
        if s <= lat <= n and w <= lon <= e:
            return cc
    return None
