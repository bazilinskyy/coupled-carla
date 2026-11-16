"""Street furniture that looks like the place: regional prop sets, road wear and rooftop clutter (CARLA 1.0).

CARLA's own props are generic (US and Spanish lamp posts, benches, bins). On CARLA 1.0 with the CoupledCarla content
cooked into the server (/Game/CoupledCarla/Street, made by tools/street_props.py and tools/ue_street_props.py from
real dimensions; data/street_props.json lists them), scenery.py places a region's own instead:

- the Netherlands (map in NLD): lamp posts by the street-light register's mast type and height where the city has
  one (Eindhoven: type_drager_mast "AL 3,5 CONISCH" -> a 4 m post-top lamp, "AL 6 NEDAL" -> a 6 m mast with a top
  LED luminaire, "ST 9 STANDAARD" with an arm -> a 9 m steel mast with a 1.5 m arm, "DUBBEL" -> a double-arm mast,
  "GY 3,5" cast iron -> a lantern; lights that are not on a pole of their own are left out by nightlights.mount),
  else by height and distance to the road; at night with lamp lights, a lamp spawns as CARLA's lit lamp in its
  place (scenery.Scenery: our meshes carry no light); the "Model
  Eindhoven" bench in the register's colour (zitbanken0), round or square bins in its colour (afvalbakken0),
  steel bollards (the red-brown "Amsterdammertje" in Amsterdam), rows of bike stands ("nietjes") by capacity,
  shark-teeth give-way lines at the register's B6 signs and OSM's give-way points, red asphalt on cycle lanes;
- the UK and Japan: their lamp columns and (UK) litter bins; elsewhere (and the US, which CARLA's own props are
  modelled on) CARLA's props as before;
- anywhere: road wear (cracks, repair patches, potholes where OSM maps a bad smoothness, oil spots in parking
  lanes and car parks, autumn leaves under deciduous street trees in autumn) and rooftop clutter (air-conditioning
  units, stair bulkheads, vents) inside flat roofs of buildings with measured heights (3DBAG, PLATEAU, ...).

Every mesh of the set has a stock CARLA fallback (scenery.json "fallbacks", as the Dutch sign faces have), so a
server without the content spawns CARLA's prop in its place; markings, decals and roof clutter have none and are
simply missing there. CARLA 0.9.16 cannot load new meshes without a source build (lin4080 has none): its catalogue
does not list them and it keeps CARLA's props. Ideas for road wear, rooftop clutter and furniture by country
standard come from BeamNG.Earth (github.com/TecnaGamer/BeamNG.Earth, MIT, TecnaGamer); no code was ported.
"""
import json
import math
import re
import zlib
from functools import lru_cache
from pathlib import Path

DATA = Path(__file__).resolve().parent / "data" / "street_props.json"
REGION_OF = {"NLD": "NL", "GBR": "UK", "JPN": "JP"}
AMSTERDAM = (52.27, 4.72, 52.43, 5.08)       # s, w, n, e: the municipality, where the Amsterdammertje stands
LAMP_MESH = {"post_top": "SM_CC_NL_LampPostTop4", "top_led": "SM_CC_NL_LampTopLed6", "arm9": "SM_CC_NL_LampArm9",
             "arm12": "SM_CC_NL_LampArm12", "double": "SM_CC_NL_LampDouble10",
             "heritage": "SM_CC_NL_LampHeritage4"}
LAMP_H = {"post_top": 4.0, "top_led": 6.0, "arm9": 9.0, "arm12": 12.0, "double": 10.0, "heritage": 3.6}
SCALE_LIMITS = (0.75, 1.35)         # a mesh stands in for masts this much lower or higher (the luminaire scales too)


@lru_cache(maxsize=1)
def props():
    """{mesh name: spec} of data/street_props.json ({} without it)."""
    try:
        return json.loads(DATA.read_text())["meshes"]
    except (FileNotFoundError, KeyError, ValueError):
        return {}


def region(iso3, bbox=None):
    """The prop set for a country (ISO 3166 alpha-3): "NL", "UK", "JP" or None (CARLA's props)."""
    return REGION_OF.get((iso3 or "").upper())


def in_amsterdam(bbox):
    if not bbox:
        return False
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    s, w, n, e = AMSTERDAM
    return s <= lat <= n and w <= lon <= e


def fallbacks(names, cat):
    """{our mesh path: stock CARLA mesh path} for our meshes among `names` that have a stock stand-in."""
    out = {}
    for n in names:
        spec = props().get(n)
        if spec and spec.get("fallback") and n in cat and spec["fallback"] in cat:
            out[cat.path(n)] = cat.path(spec["fallback"])
    return out


def _num(text):
    m = re.search(r"\d+(?:[.,]\d+)?", str(text or ""))
    return float(m.group(0).replace(",", ".")) if m else None


def parse_mast(mast, arm=None):
    """The register's mast and arm types -> (material, height m or None, arm kind, arm length m).
    material: AL aluminium, ST steel, GY cast iron, BB built-in (wall, ceiling, pavement: no mast), TRAP a
    floodlight mast with a ladder; arm kind: None, "single", "double", "top" (a short spigot), "wall"."""
    s = str(mast or "").upper().strip()
    mat = None
    if "TRAP" in s:
        mat = "TRAP"
    else:
        mm = re.match(r"(AL|ST|GY|BB|HO|BE|GI|PO)\b", s)
        mat = mm.group(1) if mm else None
    h = None
    mh = re.match(r"[A-Z]+\s+(\d+(?:[.,]\d+)?)", s)
    if mh and mat not in ("BB", None):
        h = float(mh.group(1).replace(",", "."))
        if h > 30:
            h = None
    a = str(arm or "").upper()
    kind, length = None, 0.0
    if "WAND" in a:
        kind = "wall"
    elif "DUB" in a:
        kind, length = "double", _num(a.split("DUB", 1)[1]) or 1.5
    elif "ENKEL" in a or re.search(r"\bUE\d", a):
        kind, length = "single", _num(a.split("ENKEL", 1)[-1]) or 1.0
        if length > 5:          # a size code (100 X 162) rather than the arm's length
            length = 1.5 if "1,5" in a or "1.5" in a else 1.0
    elif "OPZET" in a or "AU." in a or "ZETBEUGEL" in a or re.search(r"0[,.][1-4]", a):
        kind, length = "top", 0.2
    return mat, h, kind, length


class Chooser:
    """The regional mesh for a lamp or a piece of furniture, given a catalogue (only meshes it lists are used)."""

    def __init__(self, cat, iso3=None, bbox=None, enabled=True):
        self.cat = cat
        self.region = region(iso3, bbox) if enabled else None
        self.amsterdam = in_amsterdam(bbox)
        self.counts = {}

    def _have(self, name):
        return name is not None and name in self.cat

    def _count(self, key):
        self.counts[key] = self.counts.get(key, 0) + 1

    def street_lamp(self, tags, near_road):
        """(mesh, scale, has_arm) for a street lamp on a pole of its own (scenery.py leaves the others out by
        nightlights.mount), or None to keep CARLA's choice. Register tags: support:register (type_drager_mast),
        lamp_arm (type_drager_uithouder), height (the light point's)."""
        if self.region is None:
            return None
        mat, h, kind, length = parse_mast(tags.get("support:register"), tags.get("lamp_arm"))
        if h is None:
            h = _num(tags.get("height")) if tags.get("height") else None
        if self.region in ("UK", "JP"):
            if not near_road:
                return None
            self._count("lamp_column" if self.region == "UK" else "lamp_gooseneck")
            mesh = "SM_CC_UK_LampColumn8" if self.region == "UK" else "SM_CC_JP_LampGooseneck8"
            return self._scaled(mesh, 8.0, h, True)
        if mat == "TRAP":
            style = "arm12"
            h = h or 15.0
        elif mat == "GY":
            style = "heritage"
        elif kind == "double":
            style = "double"
        elif h is None:
            style = "top_led" if near_road else "post_top"
        elif h <= 4.5:
            style = "top_led" if kind == "single" and length >= 1.0 else "post_top"
        elif h <= 7.5:
            style = "arm9" if kind == "single" and length >= 1.0 and h >= 7.0 else "top_led"
        elif h <= 10.5:
            style = "arm9"
        else:
            style = "arm12"
        got = self._scaled(LAMP_MESH[style], LAMP_H[style], h, style not in ("post_top", "heritage"))
        if got:
            self._count("lamp_" + style)
        return got

    def _scaled(self, mesh, mesh_h, h, arm):
        if not self._have(mesh):
            return None
        s = 1.0 if not h else max(SCALE_LIMITS[0], min(SCALE_LIMITS[1], h / mesh_h))
        return mesh, round(s, 3), arm

    def furniture(self, key, value, tags, uid=0):
        """The regional mesh for an OSM-style (key, value) feature, or None (CARLA's)."""
        if self.region is None:
            return None
        kv = (key, value)
        name = None
        model = str(tags.get("ehv:model") or "").lower()
        colour = str(tags.get("colour") or "").lower()
        if self.region == "NL":
            if kv == ("amenity", "bench"):
                if "picknick" in model:
                    return None
                light = "licht" in colour or colour in ("light_brown", "tan", "beige")
                name = "SM_CC_NL_BenchEindhoven_" + ("LightBrown" if light else "DarkBrown")
            elif kv == ("amenity", "waste_basket"):
                if "constructo" in model or "beton" in model:
                    name = "SM_CC_NL_BinSquare_Grey"
                else:
                    grey = "grijs" in colour or "grey" in colour or "gray" in colour
                    if not colour and not model:        # OSM bins without data: the register's mix (62 % green)
                        grey = zlib.crc32(str(uid).encode()) % 100 < 38
                    name = "SM_CC_NL_BinRound_" + ("Grey" if grey else "Green")
            elif kv == ("barrier", "bollard"):
                name = "SM_CC_NL_Amsterdammertje" if self.amsterdam else "SM_CC_NL_Bollard"
            elif kv == ("amenity", "bicycle_parking"):
                cap = _num(tags.get("capacity")) or 6
                name = "SM_CC_NL_Nietje_x1" if cap <= 2 else "SM_CC_NL_Nietje_x3" if cap <= 8 else \
                    "SM_CC_NL_Nietje_x6"
        elif self.region == "UK":
            if kv == ("amenity", "waste_basket"):
                name = "SM_CC_UK_BinCast"
        if not self._have(name):
            return None
        self._count(name)
        return name


# --- ground: Dutch pavers -----------------------------------------------------------------------------------------

def ground_mesh(kind, size, cat, region):
    """Our paving square for a ground square of `size` m (40, 20 or 10: scale 1, so the pavers keep their size) of
    kind "open" (BGT open verharding: bricks, pavers, tiles) in the Netherlands, or None (CARLA's: on 1.0 its
    sidewalk cube stretched over the whole square, which reads as smooth concrete)."""
    if region != "NL" or kind != "open":
        return None
    name = f"SM_CC_NL_Paving_{int(round(size))}"
    return name if name in cat else None


# --- signs: one design, many countries -----------------------------------------------------------------------------

# Countries whose give-way sign is not the plain red-bordered inverted triangle (text in it, or none at all) and
# whose speed limits are not the red roundel with the number (rectangular SPEED LIMIT plates)
GIVE_WAY_OTHER = {"USA", "PRI", "GUM", "VIR", "ASM", "MNP", "PHL", "LBR", "CAN", "GBR", "IRL", "AUS", "NZL", "JPN",
                  "TWN", "CHN", "HKG", "MAC", "KOR", "IND", "MYS", "SGP"}
SPEED_RECT = {"USA", "PRI", "GUM", "VIR", "ASM", "MNP", "LBR", "CAN"}


def osm_face(kind, value, iso3, cat):
    """A Dutch RVV face (data/nl_sign_faces.json, CARLA 1.0) that is also this country's design: the Vienna
    Convention's B6 give-way triangle (UA 2.1, DE 205, FR AB3, ...) and the A1 speed roundel (UA 3.29, DE 274,
    UK and Japan's red ring too), or None. CARLA's own plates are US designs (placement.sign_fits)."""
    if not iso3:
        return None
    from . import ndwsigns
    faces = ndwsigns.faces()
    key = None
    if kind == "give_way" and iso3 not in GIVE_WAY_OTHER:
        key = "B6"
    elif kind == "speed" and value and iso3 not in SPEED_RECT:
        key = f"A1-{int(value)}"
    f = faces.get(key) if key else None
    return f["mesh"] if f and f["mesh"] in cat else None


# --- markings, road surface, wear, roofs ------------------------------------------------------------------------

def _lane_end(wp, step=1.0, max_m=60.0):
    """The last waypoint of wp's lane before the junction it leads into (None if none within max_m)."""
    last, d = wp, 0.0
    while d < max_m:
        nxt = last.next(step)
        if not nxt:
            return None
        if nxt[0].is_junction:
            return last
        last, d = nxt[0], d + step
    return None


def shark_teeth(pl, cat, cmap, carla, points, lefthand=False, log=print):
    """Give-way triangles across every lane of the approach at each point (x, y) (an NDW B6 sign, OSM's
    highway=give_way): at the lane's end before the junction, the tips towards the driver. Returns how many rows."""
    from . import placement
    names = {n: f"SM_CC_NL_SharkTeeth_{n}" for n in (3, 4, 5)}
    names = {n: m for n, m in names.items() if m in cat}
    if not names:
        return 0
    done, rows = set(), 0
    for x, y in points:
        wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None or math.hypot(wp.transform.location.x - x, wp.transform.location.y - y) > 25:
            continue
        wp = placement.approach(wp)
        end = _lane_end(wp)
        if end is None:
            continue
        lanes, lane = [], end
        while lane is not None and lane.lane_type == carla.LaneType.Driving and lane.lane_id * end.lane_id > 0:
            lanes.append(lane)      # this direction's lanes, from the kerb lane inwards
            lane = lane.get_left_lane() if not lefthand else lane.get_right_lane()
        for ln in lanes:
            key = (ln.road_id, ln.section_id, ln.lane_id)
            if key in done:
                continue
            done.add(key)
            n = min(names, key=lambda k: abs((k * 0.75 - 0.25) - (ln.lane_width - 0.3)))
            tf = ln.transform
            fw = tf.get_forward_vector()
            cx, cy = tf.location.x - fw.x * 0.4, tf.location.y - fw.y * 0.4
            pl.add("marking", names[n], cx, cy, tf.rotation.yaw + 180.0, 1.0, z=tf.location.z)
            rows += 1
    return rows


def xodr_lanes(xodr, types=("biking",)):
    """[(road id, lane id, s start, s end)] of the lanes of these OpenDRIVE types (generate_waypoints covers driving
    lanes only, and a separate cycle path is a road without one)."""
    import xml.etree.ElementTree as ET
    out = []
    root = ET.parse(str(xodr)).getroot()
    for road in root.iter("road"):
        rid, length = int(road.get("id")), float(road.get("length"))
        secs = road.findall("./lanes/laneSection")
        for k, sec in enumerate(secs):
            s0 = float(sec.get("s"))
            s1 = float(secs[k + 1].get("s")) if k + 1 < len(secs) else length
            for lane in sec.iter("lane"):
                if lane.get("type") in types and int(lane.get("id")) != 0:
                    out.append((rid, int(lane.get("id")), s0, s1))
    return out


def along(cmap, lanes, step):
    """Waypoints every `step` metres along xodr_lanes' lanes (step may be a function of the lane width)."""
    for rid, lid, s0, s1 in lanes:
        s = s0 + 0.05
        while s < s1 - 0.05:
            try:
                wp = cmap.get_waypoint_xodr(rid, lid, s)
            except RuntimeError:
                wp = None
            if wp is None:
                break
            yield wp
            s += step(wp) if callable(step) else step


def red_cycle_lanes(pl, cat, cmap, xodr, skip=()):
    """Red asphalt strips along the map's cycle lanes (OpenDRIVE lane type biking), as Dutch fietspaden are: one
    strip of the lane's width and six widths long every 5.7 widths (a little overlap on curves)."""
    name = "SM_CC_NL_RedAsphalt"
    if name not in cat:
        return 0
    n = 0
    tree = None
    if skip:
        from shapely.geometry import Point
        from shapely.strtree import STRtree
        skip = [p.buffer(2.5) for p in skip]      # OSM's line of the same path lies a metre or two off the BGT's
        tree = STRtree(skip)
    for wp in along(cmap, xodr_lanes(xodr, ("biking",)), lambda w: 5.7 * max(0.8, w.lane_width)):
        tf = wp.transform
        if tree is not None:
            p = Point(tf.location.x, tf.location.y)
            if any(skip[i].contains(p) for i in tree.query(p)):
                continue        # a BGT cycle path already covered
        pl.add("surface", name, tf.location.x, tf.location.y, tf.rotation.yaw, max(0.8, wp.lane_width),
               z=tf.location.z)
        n += 1
    return n


def red_cycle_paths(pl, cat, cmap, carla, geo, bgt_data, seg_m=12.0):
    """Red asphalt over the BGT's cycle paths (wegdeel fietspad, gesloten verharding: Dutch cycle paths are red
    asphalt), on OSM maps and twins alike: each polygon cut into pieces about seg_m long along its main direction,
    each piece covered by strips of its width (the 1 x 6 m strip scaled to the width, one per 5.7 widths) at the
    surface's height (the lane there, else the ground). Returns (strips, the cycle path polygons)."""
    from shapely.geometry import Polygon, box
    from shapely import affinity
    name = "SM_CC_NL_RedAsphalt"
    if name not in cat or not bgt_data:
        return 0, []
    n, polys = 0, []
    any_lane = carla.LaneType.Any

    def height(x, y):
        wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=False, lane_type=any_lane)
        return wp.transform.location.z if wp is not None else pl.ground(x, y)
    for srf in bgt_data.get("surfaces", []):
        if srf.get("c") != "wegdeel" or srf.get("f") != "fietspad" or srf.get("v") != "gesloten verharding":
            continue
        for ring in srf.get("r", [])[:1]:
            try:
                poly = Polygon([geo.to_carla(lat, lon) for lon, lat in ring]).buffer(0)
            except Exception:  # noqa: BLE001
                continue
            if poly.is_empty or poly.area < 4:
                continue
            polys.append(poly)
            rect = list(poly.minimum_rotated_rectangle.exterior.coords)
            e = max(((rect[i], rect[i + 1]) for i in range(4)), key=lambda p: math.dist(*p))
            ang = math.degrees(math.atan2(e[1][1] - e[0][1], e[1][0] - e[0][0]))
            c = poly.centroid
            flat_ = affinity.rotate(poly, -ang, origin=c)        # main direction along x
            minx, miny, maxx, maxy = flat_.bounds
            x0 = minx
            while x0 < maxx - 0.3:
                piece = flat_.intersection(box(x0, miny - 1, min(maxx, x0 + seg_m), maxy + 1))
                x0 += seg_m
                if piece.is_empty or piece.area < 1.0:
                    continue
                for part in getattr(piece, "geoms", [piece]):
                    if part.area < 1.0:
                        continue
                    px0, py0, px1, py1 = part.bounds
                    w = max(0.8, min(5.0, part.area / max(0.5, px1 - px0)))
                    step = 5.7 * w
                    k = max(1, math.ceil((px1 - px0) / step))
                    for i in range(k):
                        cx_ = px0 + (i + 0.5) * (px1 - px0) / k
                        mid = part.intersection(box(cx_ - 0.05, py0 - 1, cx_ + 0.05, py1 + 1))
                        cy_ = mid.centroid.y if not mid.is_empty else (py0 + py1) / 2
                        p = affinity.rotate(Point_(cx_, cy_), ang, origin=c)
                        pl.add("surface", name, p.x, p.y, ang, w, z=height(p.x, p.y))
                        n += 1
    return n, polys


def Point_(x, y):
    from shapely.geometry import Point
    return Point(x, y)


def _rng(*parts):
    return zlib.crc32("|".join(str(p) for p in parts).encode()) / 2 ** 32


WEAR = {    # mesh -> (per lane metre at "normal" wear, kind); the smoothness factor multiplies the potholes
    "SM_CC_Any_Crack_A": (1 / 45.0, "crack"),
    "SM_CC_Any_Patch_A": (1 / 90.0, "patch"),
    "SM_CC_Any_Pothole_A": (1 / 600.0, "pothole"),
}
SMOOTHNESS = {"excellent": 0.2, "good": 0.6, "intermediate": 1.5, "bad": 4.0, "very_bad": 8.0, "horrible": 12.0,
              "very_horrible": 16.0, "impassable": 20.0}


def road_wear(pl, cat, cmap, carla, level=1.0, rough=None, step=6.0):
    """Cracks, repair patches and potholes on the driving lanes, deterministic per spot. rough: {road id: factor}
    from OSM smoothness (bad roads get more of everything, potholes most). Returns {kind: count}."""
    have = {m: v for m, v in WEAR.items() if m in cat}
    out = {}
    if not have or level <= 0:
        return out
    for wp in cmap.generate_waypoints(step):
        if wp.lane_type != carla.LaneType.Driving or wp.is_junction:
            continue
        f = (rough or {}).get(wp.road_id, 1.0)
        for mesh, (rate, kind) in have.items():
            p = rate * step * level * (f if kind != "pothole" else f * f)
            u = _rng(wp.road_id, wp.lane_id, round(wp.s, 1), kind)
            if u >= p:
                continue
            tf = wp.transform
            r = tf.get_right_vector()
            lat = (_rng(wp.road_id, wp.s, kind, "lat") - 0.5) * max(0.0, wp.lane_width - 1.6)
            if kind == "pothole":      # in the wheel tracks
                lat = (0.9 if _rng(wp.s, "side") < 0.5 else -0.9)
            x, y = tf.location.x + r.x * lat, tf.location.y + r.y * lat
            yaw = tf.rotation.yaw + (0.0 if kind == "patch" else _rng(wp.s, kind, "yaw") * 360.0)
            pl.add("wear", mesh, x, y, yaw, 0.8 + 0.4 * _rng(wp.s, kind, "s"), z=tf.location.z)
            out[kind] = out.get(kind, 0) + 1
    return out


def oil_spots(pl, cat, cmap, xodr, parking_areas=(), step=5.5):
    """Oil stains in parking lanes (one in seven stalls) and on car parks (polygons in CARLA metres)."""
    name = "SM_CC_Any_Oil_A"
    if name not in cat:
        return 0
    n = 0
    for wp in along(cmap, xodr_lanes(xodr, ("parking",)), step):
        if _rng(wp.road_id, wp.lane_id, round(wp.s)) > 0.15:
            continue
        tf = wp.transform
        pl.add("wear", name, tf.location.x, tf.location.y, tf.rotation.yaw, 1.0, z=tf.location.z)
        n += 1
    for poly in parking_areas:
        minx, miny, maxx, maxy = poly.bounds
        from shapely.geometry import Point
        gx = minx + 1.5
        while gx < maxx:
            gy = miny + 1.5
            while gy < maxy:
                if _rng(round(gx), round(gy), "oil") < 0.06 and poly.contains(Point(gx, gy)):
                    pl.add("wear", name, gx, gy, _rng(gx, gy) * 360.0, 0.8 + 0.4 * _rng(gy, gx))
                    n += 1
                gy += 2.6
            gx += 5.0
    return n


def leaf_litter(pl, cat, trees, roads_near, step=1.0):
    """Leaf litter around deciduous street trees near a road or pavement (trees: [(x, y, crown radius)]); placed
    only for trials in autumn (scenery.json kind "leaves": the run time drops them otherwise)."""
    name = "SM_CC_Any_Leaves_A"
    if name not in cat:
        return 0
    n = 0
    for x, y, r in trees:
        if not roads_near(x, y):
            continue
        k = 1 + int(r > 3.5)
        for i in range(k):
            a = _rng(x, y, i) * 2 * math.pi
            d = r * 0.6 * _rng(y, x, i)
            pl.add("leaves", name, x + math.cos(a) * d, y + math.sin(a) * d, _rng(x, i) * 360.0,
                   0.8 + 0.6 * _rng(i, x, y))
            n += 1
    return n


ROOF = {"ac": "SM_CC_Any_RoofAC", "ac_large": "SM_CC_Any_RoofACLarge", "bulkhead": "SM_CC_Any_RoofBulkhead",
        "vent": "SM_CC_Any_RoofVent"}
FLAT_ROOFS = {None, "flat", "unknown"}
EVERGREEN = ("Pine", "Cypress", "Palm", "Conifer", "Fir")
HOUSE_CLASSES = {"house", "detached", "semidetached_house", "terrace", "residential_house", "bungalow", "farm"}


def roof_clutter(pl, cat, buildings, min_area=60.0, margin=1.2, ground=None):
    """Rooftop clutter on flat roofs of buildings with measured heights (records of scenery.building_records with a
    national source): AC units (one per 120 m2, large condensers on roofs over 600 m2), a stair bulkhead on roofs
    over 250 m2 of buildings of 3 storeys and more, vents; inside the roof outline less a margin, at the roof's
    height (ground(x, y) at the roof's middle plus the building's height; without it the record's z). The big
    pieces are placed first. Returns {kind: count}."""
    from shapely.geometry import Point
    have = {k: m for k, m in ROOF.items() if m in cat}
    out = {}
    if not have:
        return out
    for b in buildings:
        poly, h = b["poly"], b["h"]
        if b.get("src") in (None, "osm", "overture") or b.get("roof") not in FLAT_ROOFS or h < 5.0:
            continue
        if (b.get("cls") or "") in HOUSE_CLASSES or poly.area < min_area:
            continue
        inner = poly.buffer(-margin)
        if inner.is_empty:
            continue
        c = poly.representative_point()
        z = (ground(c.x, c.y) if ground else b["z"]) + h
        minx, miny, maxx, maxy = inner.bounds
        key = (round(poly.centroid.x), round(poly.centroid.y))
        # the roof's main direction: the longest edge of its minimum rotated rectangle
        rect = list(poly.minimum_rotated_rectangle.exterior.coords)
        e = max(((rect[i], rect[i + 1]) for i in range(4)), key=lambda p: math.dist(*p))
        yaw = math.degrees(math.atan2(e[1][1] - e[0][1], e[1][0] - e[0][0]))
        want = []
        if "bulkhead" in have and poly.area > 250 and h >= 9.0:
            want.append("bulkhead")
        if "ac_large" in have and poly.area > 600:
            want += ["ac_large"] * (1 + int(poly.area > 2000))
        n_ac = max(1 if poly.area > 80 else 0, round(poly.area / 120.0 * (0.5 + _rng(key, "ac"))))
        want += ["ac"] * min(n_ac, 12)
        want += ["vent"] * min(4, int(poly.area / 150.0 * _rng(key, "vent")))
        placed = []
        for i, kind in enumerate(want):
            if kind not in have:
                continue
            L, W, _ = cat.size(have[kind])
            rad = math.hypot(L, W) / 2
            for t in range(12):
                px = minx + (maxx - minx) * _rng(key, kind, i, t, "x")
                py = miny + (maxy - miny) * _rng(key, kind, i, t, "y")
                if not inner.buffer(-rad * 0.7).contains(Point(px, py)):
                    continue
                if any(math.hypot(px - qx, py - qy) < rad + qr + 0.5 for qx, qy, qr in placed):
                    continue
                placed.append((px, py, rad))
                pl.add("roof", have[kind], px, py, yaw + (90.0 if _rng(key, i) < 0.5 else 0.0), 1.0, z=z)
                out[kind] = out.get(kind, 0) + 1
                break
    return out


def osm_wear(bbox, path):
    """OSM roads with a smoothness tag and surface car parks in the box (Overpass JSON with geometry), cached."""
    if path.exists():
        return json.loads(path.read_text())
    from . import realworld as rw
    s, w, n, e = bbox
    b = f"({s},{w},{n},{e})"
    data = rw.overpass_json(f'[out:json][timeout:120];(way["highway"]["smoothness"]{b};'
                            f'way["amenity"="parking"]["parking"!~"underground|multi-storey|rooftop"]{b};);'
                            f'out geom tags;')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, separators=(",", ":")))
    return data


def smoothness_by_road(cmap, carla, geo, ways):
    """{OpenDRIVE road id: factor} from OSM ways with a smoothness tag (Overpass JSON with geometry): each way's
    points vote for the road under them."""
    out = {}
    for w in ways:
        f = SMOOTHNESS.get((w.get("tags") or {}).get("smoothness"))
        if f is None:
            continue
        for g in (w.get("geometry") or [])[::2]:
            x, y = geo.to_carla(g["lat"], g["lon"])
            wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True)
            if wp is not None and math.hypot(wp.transform.location.x - x, wp.transform.location.y - y) < 6:
                out[wp.road_id] = max(out.get(wp.road_id, 0.0), f)
    return out


# --- the scenery hook ---------------------------------------------------------------------------------------------

def _on(sc, key):
    return sc.get(key, "auto") not in (False, "off", "false", None)


def add_to_scenery(pl, cat, cmap, carla, geo, mc, sc, mdir, els, blds, look, log=print, ndw=None, bgt_data=None):
    """scenery.build's hook: markings and red cycle lanes (the Dutch set), road wear, oil spots, leaf litter and
    rooftop clutter (any region), each where the catalogue has the meshes (CARLA 1.0) and its option is on."""
    from shapely.geometry import Polygon
    xodr = Path(mdir) / "map.xodr"
    lefthand = bool(mc.get("lefthand"))
    said = []
    if look.region == "NL" and _on(sc, "shark_teeth"):
        pts = [geo.to_carla(e["lat"], e["lon"]) for e in els if e.get("type") == "node"
               and (e.get("tags", {}).get("highway") == "give_way"
                    or re.search(r"(^|[:;])B6\b", e.get("tags", {}).get("traffic_sign", "")))]
        pts += [geo.to_carla(s_["lat"], s_["lon"]) for s_ in (ndw or [])
                if str(s_.get("rvv_code") or s_.get("code") or "").upper() == "B6"]
        n = shark_teeth(pl, cat, cmap, carla, pts, lefthand, log)
        pl.counts["marking_shark_teeth"] = n
        said.append(f"{n} shark-teeth rows at {len(pts)} give-way points")
    if look.region == "NL" and _on(sc, "red_cycle"):
        m, polys = red_cycle_paths(pl, cat, cmap, carla, geo, bgt_data)
        n = red_cycle_lanes(pl, cat, cmap, xodr, polys) if xodr.exists() else 0
        pl.counts["surface_red_cycle"] = n + m
        said.append(f"red asphalt: {m} strips on {len(polys)} BGT cycle paths, {n} on the map's other cycle lanes")
    wear = sc.get("road_wear", "auto")
    if _on(sc, "road_wear"):
        level = 1.0 if wear in ("auto", True) else float(wear)
        rough = None
        extra = {}
        try:
            extra = osm_wear(mc["bbox"], Path(mdir) / "real" / "osm_streetlook.json")
            rough = smoothness_by_road(cmap, carla, geo, [e for e in extra.get("elements", [])
                                                          if e.get("tags", {}).get("smoothness")])
        except Exception as err:  # noqa: BLE001 - no smoothness: the same wear everywhere
            log(f"scenery: OSM smoothness and car parks not read ({type(err).__name__}: {err})")
        got = road_wear(pl, cat, cmap, carla, level, rough)
        areas = []
        for e in list(els) + extra.get("elements", []):
            t = e.get("tags", {})
            if t.get("amenity") == "parking" and t.get("parking", "surface") == "surface" and e.get("geometry") \
                    and len(e["geometry"]) >= 4:
                try:
                    poly = Polygon([geo.to_carla(g["lat"], g["lon"]) for g in e["geometry"]])
                    if poly.is_valid and 30 < poly.area < 20000:
                        areas.append(poly)
                except Exception:  # noqa: BLE001
                    pass
        if xodr.exists():
            got["oil"] = oil_spots(pl, cat, cmap, xodr, areas)
        for k, v in got.items():
            pl.counts[f"wear_{k}"] = v
        if got:
            said.append("road wear " + ", ".join(f"{k} {v}" for k, v in sorted(got.items()))
                        + (f" ({len(rough)} roads with OSM smoothness)" if rough else ""))
    if _on(sc, "leaf_litter") and "SM_CC_Any_Leaves_A" in cat:
        names = {i: m for m, i in pl.mesh_index.items()}
        trees = [(base[0], base[1], 2.0 + 2.0 * min(2.0, it[5])) for it, base in zip(pl.items, pl.bases)
                 if it[6] == "tree" and not any(k in names.get(it[0], "") for k in EVERGREEN)]

        def near(x, y):
            return any(pl.on_lane(x + dx, y + dy, 0.0) for dx, dy in ((2.5, 0), (-2.5, 0), (0, 2.5), (0, -2.5)))
        n = leaf_litter(pl, cat, trees[:4000], near)
        pl.counts["leaves"] = n
        said.append(f"{n} leaf litter patches under {len(trees)} deciduous trees (shown in autumn)")
    roofs = sc.get("roof_clutter", "auto")
    if blds and (roofs is True or (roofs == "auto" and mc.get("carla_level"))):
        # only where the buildings are the measured ones (a baked twin): CARLA's building meshes stretched over a
        # footprint have their own roofs, which AC units at the record's height would float over or sink into
        got = roof_clutter(pl, cat, blds, ground=pl.ground)
        for k, v in got.items():
            pl.counts[f"roof_{k}"] = v
        if got:
            said.append("rooftop " + ", ".join(f"{k} {v}" for k, v in sorted(got.items())))
    if said:
        log("scenery: " + "; ".join(said))


# --- night: CARLA's lit lamp in place of ours ----------------------------------------------------------------------

def night_stand_in(pl, cat, item, stock, x, y, yaw):
    """Record CARLA's own lamp `stock` (the one scenery would have placed, which carries a light on CARLA 1.0) as
    the night stand-in of our lamp `item`, placed at (x, y) as it would have been; relative to the item so that
    placement.settle's height changes carry over. Returns the stand-in as an item-like list (its position for
    nightlights' lamp specs), or None when the catalogue lacks the stock lamp."""
    if stock not in cat:
        return None
    sx, sy, sz, syaw = cat.place(stock, x, y, _item_ground(pl, item), yaw, 1.0)
    idx = pl.mesh_index.setdefault(stock, len(pl.mesh_index))
    if not hasattr(pl, "night_lamps"):
        pl.night_lamps = []
    pl.night_lamps.append((item, [idx, round(sx - item[1], 2), round(sy - item[2], 2), round(sz - item[3], 2),
                                  round(syaw % 360, 1)]))
    return [idx, round(sx, 2), round(sy, 2), round(sz, 2), round(syaw % 360, 1), 1.0, "lamp"]


def _item_ground(pl, item):
    """The ground height the item stands on (its base), else the placer's ground under it."""
    for it, base in zip(reversed(pl.items), reversed(pl.bases)):
        if it is item and base is not None:
            return base[2]
    return pl.ground(item[1], item[2])


def night_rows(pl):
    """scenery.json "lamp_night": [item index, stock mesh index, dx, dy, dz, yaw] per regional lamp."""
    pairs = getattr(pl, "night_lamps", None) or []
    if not pairs:
        return []
    index = {id(it): k for k, it in enumerate(pl.items)}
    return [[index[id(it)]] + alt for it, alt in pairs if id(it) in index]
