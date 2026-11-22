"""Paved lanes on generated maps: each car lane of a road whose OSM way is paved with bricks or setts gets an
OpenDRIVE ``<material surface="clinker|pavers|sett"/>``.

CARLA 1.0 with the road-surface change (carla cc fork: MeshFactory names a lane's mesh section "road:<surface>" and
OpenDriveGenerator gives it /Game/CoupledCarla/Paving/MI_Surface_<surface>, -carla-road-surface-materials) then
builds those lanes in their own paving material: the road surface itself, as smooth as the asphalt, where overlay
strips (streetlife.stone_roads) showed their edges and planes. Such a server names those road actors
RoadSurface_<surface> (world.get_names_of_all_objects), and the scenery then leaves its overlays out
(server_paints_surfaces). Other servers ignore the materials and keep the overlays.

Which road is paved: every road is sampled along its lanes; a sample takes the surface of the highest-class OSM car
road whose carriageway (half width + 0.5 m) holds it, else of the nearest within 12 m; the road takes the majority.
So an asphalt main road keeps its surface through a junction and a brick side street starts at its edge, as in the
street (carla-digitaltwins tools/road_paving.py does the same for baked twins).

    python -m coupledcarla.road_surfaces maps/<map>          # annotate maps/<map>/map.xodr in place
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

# car roads by class (higher wins in a junction) and default half width (m) when OSM has no lanes/width
CAR_ROADS = {"motorway": (9, 7.0), "trunk": (8, 6.0), "primary": (7, 5.0), "secondary": (6, 4.5),
             "tertiary": (5, 3.8), "unclassified": (4, 3.2), "residential": (4, 3.2), "busway": (4, 3.2),
             "living_street": (3, 3.0), "service": (2, 2.8), "pedestrian": (1, 3.0), "road": (3, 3.0),
             "track": (0, 2.0)}
for _k in ("motorway", "trunk", "primary", "secondary", "tertiary"):
    CAR_ROADS[_k + "_link"] = (CAR_ROADS[_k][0], CAR_ROADS[_k][1] * 0.7)
BRICKS = {"paving_stones", "bricks", "brick", "clinker", "paved:bricks"}
SETT = {"sett", "cobblestone", "unhewn_cobblestone", "cobblestone:flattened"}
LANE_TYPES = ("driving", "parking", "bidirectional")
FRICTION = {"clinker": 0.75, "pavers": 0.75, "sett": 0.65}


def _num(v, default=0.0):
    try:
        return float(str(v).split()[0].replace(",", "."))
    except (TypeError, ValueError):
        return default


def style_of(tags: dict, region: str):
    """clinker (Dutch red klinkers), pavers (grey concrete blocks), sett, or None, from a way's OSM tags."""
    s = (tags.get("surface") or "").strip()
    if s in SETT:
        return "sett"
    if s in BRICKS:
        colour = (tags.get("surface:colour") or tags.get("colour") or "").lower()
        if colour in ("grey", "gray", "black", "white"):
            return "pavers"
        if colour in ("red", "brown") or region == "NL" or s in ("bricks", "brick", "clinker"):
            return "clinker"
        return "pavers"
    return None


def osm_roads(map_osm: Path, geo, region: str) -> list:
    """[{line, style, rank, half, layer}] of the car roads in an OSM XML extract (CARLA x, y)."""
    from shapely.geometry import LineString
    root = ET.parse(str(map_osm)).getroot()
    nodes = {n.get("id"): (float(n.get("lat")), float(n.get("lon"))) for n in root.iter("node")}
    out = []
    for w in root.iter("way"):
        tags = {t.get("k"): t.get("v") for t in w.iter("tag")}
        hw = tags.get("highway")
        if hw not in CAR_ROADS or tags.get("area") == "yes":
            continue
        pts = [geo.to_carla(*nodes[nd.get("ref")]) for nd in w.iter("nd") if nd.get("ref") in nodes]
        if len(pts) < 2:
            continue
        rank, dh = CAR_ROADS[hw]
        width = _num(tags.get("width"))
        lanes = _num(tags.get("lanes"))
        half = width / 2 if width > 0 else (max(dh, lanes * 1.6) if lanes else dh)
        out.append({"line": LineString(pts), "style": style_of(tags, region), "rank": rank, "half": half,
                    "layer": int(_num(tags.get("layer")))})
    return out


class Classifier:
    def __init__(self, roads, reach=12.0):
        from shapely.strtree import STRtree
        self.roads, self.reach = roads, reach
        self.tree = STRtree([r["line"] for r in roads]) if roads else None

    def __call__(self, x, y):
        """The paving style at (x, y) (None: asphalt or no road)."""
        if self.tree is None:
            return None
        from shapely.geometry import Point
        p = Point(x, y)
        cand = [self.roads[i] for i in self.tree.query(p.buffer(self.reach))]
        if not cand:
            return None
        d = [(r["line"].distance(p), r) for r in cand]
        inside = [(r["rank"], -dd, r) for dd, r in d if dd <= r["half"] + 0.5]
        if inside:
            return max(inside, key=lambda z: (z[0], z[1]))[2]["style"]
        dd, r = min(d, key=lambda z: z[0])
        return r["style"] if dd <= self.reach else None


def road_styles(xodr: Path, cmap, classify, step=5.0) -> dict:
    """{road id: style} of the roads whose car lanes are mostly paved."""
    from .streetlook import xodr_lanes
    votes = {}
    seen = set()
    for rid, lid, s0, s1 in xodr_lanes(xodr, LANE_TYPES):
        if (rid, s0) in seen:       # one lane per road section is enough: the road takes one surface
            continue
        seen.add((rid, s0))
        n = max(1, int((s1 - s0) / step))
        for k in range(n):
            s = s0 + (k + 0.5) * (s1 - s0) / n
            try:
                wp = cmap.get_waypoint_xodr(rid, lid, s)
            except RuntimeError:
                wp = None
            if wp is None:
                continue
            loc = wp.transform.location
            v = votes.setdefault(rid, {})
            st = classify(loc.x, loc.y)
            v[st] = v.get(st, 0) + 1
    out = {}
    for rid, v in votes.items():
        best = max(v.items(), key=lambda kv: kv[1])
        if best[0] is not None and best[1] * 2 > sum(v.values()):
            out[rid] = best[0]
    return out


def write_materials(xodr: Path, styles: dict, out: Path | None = None) -> int:
    """Set <material surface=.../> on the car lanes of these roads (and remove it from all others); returns lanes."""
    tree = ET.parse(str(xodr))
    n = 0
    for road in tree.getroot().iter("road"):
        st = styles.get(int(road.get("id")))
        for lane in road.iter("lane"):
            for m in [m for m in lane.findall("material")]:
                lane.remove(m)
            if st is None or lane.get("type") not in LANE_TYPES:
                continue
            mat = ET.Element("material", {"sOffset": "0", "surface": st, "friction": str(FRICTION[st]),
                                          "roughness": "0.01"})
            kids = list(lane)
            pos = next((i for i, c in enumerate(kids) if c.tag in ("visibility", "speed", "access", "height",
                                                                   "rule")), len(kids))
            lane.insert(pos, mat)
            n += 1
    tree.write(str(out or xodr), encoding="UTF-8", xml_declaration=True)
    return n


def annotate(mdir, region=None, log=print) -> dict:
    """Paved lanes in maps/<map>/map.xodr from its map.osm. Returns {style: roads}."""
    import carla
    from .geo import Geo
    mdir = Path(mdir)
    xodr, osm = mdir / "map.xodr", mdir / "map.osm"
    if not xodr.exists() or not osm.exists():
        log(f"road surfaces: {mdir.name} has no map.xodr or map.osm")
        return {}
    geo = Geo(mdir / "geo.net.xml")
    cmap = carla.Map(mdir.name, xodr.read_text(encoding="utf-8"))
    if region is None:      # the country at the map's middle (paving_stones are red klinkers in NL only)
        from .provenance import country_at
        tops = cmap.get_topology()
        mx = sum(a.transform.location.x for a, _ in tops) / max(1, len(tops))
        my = sum(a.transform.location.y for a, _ in tops) / max(1, len(tops))
        region = country_at(*geo.to_latlon(mx, my)) or ""
    roads = osm_roads(osm, geo, region)
    styles = road_styles(xodr, cmap, Classifier(roads))
    lanes = write_materials(xodr, styles)
    got = {}
    for st in styles.values():
        got[st] = got.get(st, 0) + 1
    log(f"road surfaces: {lanes} lanes on {len(styles)} roads paved ({got})")
    return got


def server_paints_surfaces(world) -> bool:
    """The server built paved lanes in their own material (RoadSurface_* road actors of a generated map)."""
    try:
        return any(n.startswith("RoadSurface_") for n in world.get_names_of_all_objects())
    except Exception:  # noqa: BLE001 - an older server without the call
        return False


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("map_dir")
    ap.add_argument("--region")
    a = ap.parse_args(argv)
    annotate(a.map_dir, a.region)


if __name__ == "__main__":
    main()
