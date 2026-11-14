"""Scenery for OSM maps from real-world data: buildings, trees, street furniture and ground.

CARLA builds only the roads of an OpenDRIVE map. At map build time `build()` turns the real-world data of
realworld.py into placements of CARLA's own meshes (scenery.json); at run time `Scenery` spawns them as
static.prop.mesh actors around the participants and removes them behind, in 100 m cells, as the SUMO traffic is.

- Buildings: every footprint (3DBAG, Overture or OSM) is split into rectangles, and each rectangle gets the
  CARLA building whose proportions, height and kind (house, apartment block, tower, industrial) fit it best,
  turned to the footprint and scaled to it, standing on the terrain. The shapes and facades are CARLA's; the
  positions, footprints and heights are the real ones.
- Trees: OSM street trees with their genus, leaf type and height pick a matching mesh and size; tree rows get
  a tree every 8 m, and parks, gardens and woods are planted at a density for their kind.
- Road signs: OSM stop, give-way and speed-limit signs on a post at the edge of the lane they govern, facing
  its traffic.
- Street furniture: OSM lamps, benches, bins, bus shelters, bollards, post boxes, bicycle racks, hydrants,
  fountains and ad columns at their mapped positions; where OSM maps few lamps, lamps line the roads.
- Ground: 40 m tiles (20 m on hilly maps), grass in green areas, water on rivers and lakes, paving elsewhere, tilted to the terrain.
  In the Netherlands the BGT (bgt.py) says what covers the ground beside the roads: grass, bricks or tiles,
  asphalt or concrete, gravel or sand; mixed tiles are split in four. Its street furniture replaces OSM's nearby.

Nothing is placed on a lane CARLA can drive, ride or walk on.
"""
import json
import math
import random
import zlib
from collections import defaultdict
from pathlib import Path

import numpy as np

DATA = Path(__file__).parent / "data"
CELL = 100.0          # streaming cell size (m)
TILE_M = 40.0         # ground tile size (m); 20 m on maps with more than 40 m of relief

# whole CARLA buildings by kind (file names in Content/Carla/Static/Building)
BUILDINGS = {
    "house": ["SM_House01", "SM_House02", "SM_House03", "SM_House04", "SM_House05", "SM_House06", "SM_House10",
              "SM_House11", "SM_House12_1", "SM_House_13", "SM_TerracedHouse_1", "SM_TerracedHouse02_1",
              "SM_TerracedHouse02_2", "SM_Mansion01", "SM_Mansion02", "SM_Mansion03", "SM_Mansion04",
              "SM_Apartment01_1", "SM_Apartment02_1"],
    "apartments": ["SM_Apartment04_v01_Merged", "SM_Apartment04_v02_Merged", "SM_Apartment04_v03_Merged",
                   "SM_Apartment04_v04_Merged", "SM_Apartment04_v05_Merged", "SM_Apartment04_v06_Merged",
                   "SM_Apartment04_v07_Merged", "SM_Apartment04_v08_Merged", "SM_Apartment03_v01_merged",
                   "SM_Apartment03_v02_merged", "SM_Block12_v02_merged", "SM_Block12_v03_merged",
                   "SM_Block12_v04_merged", "SM_Block13NY_v01_Merged", "SM_Block13NY_v02_Merged",
                   "SM_Block13NY_v03_Merged", "SM_Block13NY_v04_Merged", "SM_Block13NY_v05_Merged",
                   "SM_Block01_fixed", "SM_Block07_1_fixed", "SM_Block07_2_fixed", "SM_Block08", "SM_Bloc04",
                   "SM_Block04_fixed", "SM_Block05"],
    "commercial": ["SM_Hotel_v01_merged", "SM_Hotel_v02_merged", "SM_Hotel_v03_merged", "SM_Mall02", "SM_Mall03",
                   "SM_Block13NY_v04_Merged", "SM_Block12_v03_merged", "SM_Apartment04_v07_Merged",
                   "SM_Skyscraper_05_v03_merged", "SM_Skyscraper_04_v02_Merged", "SM_Skyscraper_04_v01_Merged"],
    "tower": ["SM_Skycraper01", "SM_Skycraper02", "SM_Skycraper03", "SM_Skycraper05", "SM_Skycraper06",
              "SM_Skycraper04_1", "SM_Skycraper04_2", "SM_Skyscraper_01_v01_Merged", "SM_Skyscraper_01_v02_Merged",
              "SM_Skyscraper_02_v01_Merged", "SM_Skyscraper_02_v02_Merged", "SM_Skyscraper_02_v03_Merged",
              "SM_Skyscraper_02_v04_Merged", "SM_Skyscraper_03_v01_Merged", "SM_Skyscraper_v01_Merged",
              "SM_Skyscraper_v02_Merged", "SM_Skyscraper_v03_Merged", "SM_Skyscraper_v04_Merged",
              "SM_Skyscraper_05_v01_merged", "SM_Skyscraper_05_v02_merged", "SM_Skyscraper_05_v05_merged",
              "SM_SkyScraperNY_1_merged", "SM_SkyscraperNY_02_v01_merged", "SM_SkyscraperNY_02_v02_merged",
              "SM_SkyscraperNY_02_v03_merged", "SM_SkyscraperPoly_merged"],
    "industrial": ["SM_Factory_v01_Merged", "SM_Factory_v02_Merged", "SM_Factory_v03_Merged",
                   "SM_Parking_v01_merged", "SM_ParkingRamp_v01_merged", "SM_Garage01"],
    "church": ["SM_Church01"],
}
# left out after a contact sheet of all of them: untextured boxes and open shells / single facades
BROKEN = {"SM_Block04_fixed", "SM_Block07_1_fixed", "SM_Block07_2_fixed", "SM_Block08",
          "SM_SkyscraperNY_02_v01_merged", "SM_SkyscraperNY_02_v02_merged", "SM_SkyscraperNY_02_v03_merged",
          "SM_Skyscraper_04_v01_Merged", "SM_Skyscraper_04_v02_Merged", "SM_Skyscraper_05_v01_merged",
          "SM_Skyscraper_05_v02_merged", "SM_Skyscraper_05_v03_merged", "SM_Skyscraper_05_v05_merged",
          "SM_Skyscraper_v02_Merged", "SM_Skyscraper_v03_Merged", "SM_Skyscraper_v04_Merged",
          "SM_ParkingRamp_v01_merged"}
# Overture/OSM building class -> kinds to choose from (first preferred)
KIND_OF = {
    "house": ["house"], "detached": ["house"], "semidetached_house": ["house"], "terrace": ["house", "apartments"],
    "bungalow": ["house"], "farm": ["house"], "cabin": ["house"], "residential": ["apartments", "house"],
    "apartments": ["apartments"], "dormitory": ["apartments"], "hotel": ["commercial", "apartments"],
    "commercial": ["commercial", "apartments"], "retail": ["commercial", "apartments"],
    "office": ["commercial", "apartments", "tower"], "supermarket": ["commercial", "industrial"],
    "industrial": ["industrial"], "warehouse": ["industrial"], "manufacture": ["industrial"],
    "garage": ["industrial"], "garages": ["industrial"], "parking": ["industrial"], "service": ["industrial"],
    "church": ["church", "apartments"], "cathedral": ["church", "apartments"], "chapel": ["church"],
    "school": ["apartments", "commercial"], "university": ["apartments", "commercial"],
    "hospital": ["apartments", "commercial"], "civic": ["commercial", "apartments"],
    "public": ["commercial", "apartments"], "government": ["commercial", "apartments"],
    "train_station": ["commercial"], "transportation": ["commercial", "industrial"],
}
DEFAULT_HEIGHT = {"house": 7.5, "apartments": 13.0, "commercial": 13.0, "industrial": 8.0, "church": 18.0,
                  "tower": 60.0, None: 10.0}

TREES = {   # family -> meshes, roughly small to large
    "maple": ["SM_Maple_S_v4", "SM_Maple_S_v6", "SM_Maple_M_v1", "SM_Maple_M_v4", "SM_Maple_M_v5", "SM_Maple_M_v6",
              "SM_Maple_L_v1", "SM_Maple_L_v4", "SM_Maple_L_v5", "SM_Maple_L_v6"],
    "oak": ["SM_Oak_S_v1", "SM_Oak_S_v4", "SM_Oak_S_v6", "SM_Oak_M_v1", "SM_Oak_M_v4", "SM_Oak_M_v6", "SM_Oak_L_v1",
            "SM_Oak_L_v4", "SM_Oak_L_v6"],
    "pine": ["SM_Pine_S_v4", "SM_Pine_M_v4", "SM_Pine_M_v5", "SM_Pine_L_v4", "SM_Pine_L_v5", "SM_TreePine_1",
             "SM_TreePine_2"],
    "ash": ["SM_Ash_01", "SM_Ash_02"],
    "beech": ["SM_Beech"],
    "walnut": ["SM_Wallnut_01"],
    "cypress": ["SM_Cypress_Small", "SM_Cypress"],
    "palm": ["SM_FanPalm_01", "SM_FanPalm_02", "SM_FanPalm_03", "SM_CoconutPalm_01", "SM_CoconutPalm_02"],
    "ornamental": ["SM_Acer_Red_01", "SM_Acer_02", "SM_Japanese_Maple_01", "SM_Japanese_Maple_03"],
    "sassafras": ["SM_Sassafras_01", "SM_Sassafras_02", "SM_Sassafras_03"],
    "tropical": ["SM_Aporosa"],
}
GENUS = {
    "acer": "maple", "platanus": "maple", "aesculus": "maple", "liquidambar": "maple", "liriodendron": "maple",
    "quercus": "oak", "ulmus": "oak", "zelkova": "oak", "castanea": "oak", "celtis": "oak",
    "pinus": "pine", "picea": "pine", "abies": "pine", "larix": "pine", "taxus": "pine", "pseudotsuga": "pine",
    "cedrus": "pine", "metasequoia": "pine", "sequoia": "pine", "araucaria": "pine",
    "fraxinus": "ash", "gleditsia": "ash", "robinia": "ash", "sophora": "ash", "styphnolobium": "ash",
    "eucalyptus": "ash", "ailanthus": "ash", "sorbus": "ash",
    "fagus": "beech", "carpinus": "beech", "tilia": "beech", "alnus": "beech", "corylus": "beech",
    "juglans": "walnut", "pterocarya": "walnut",
    "cupressus": "cypress", "thuja": "cypress", "chamaecyparis": "cypress", "juniperus": "cypress",
    "phoenix": "palm", "washingtonia": "palm", "cocos": "palm", "trachycarpus": "palm", "roystonea": "palm",
    "archontophoenix": "palm", "syagrus": "palm", "livistona": "palm", "butia": "palm", "chamaerops": "palm",
    "prunus": "ornamental", "malus": "ornamental", "pyrus": "ornamental", "magnolia": "ornamental",
    "cercis": "ornamental", "lagerstroemia": "ornamental", "crataegus": "ornamental", "amelanchier": "ornamental",
    "betula": "sassafras", "populus": "sassafras", "ginkgo": "sassafras", "salix": "sassafras",
    "ficus": "tropical", "jacaranda": "tropical", "tabebuia": "tropical", "handroanthus": "tropical",
    "delonix": "tropical", "tipuana": "tropical", "mangifera": "tropical", "terminalia": "tropical",
    "spathodea": "tropical", "cassia": "tropical", "pongamia": "tropical", "melia": "tropical",
}
BUSHES = ["SM_Bush_M_v1", "SM_Bush_M_v3", "SM_Bush_L_v1", "SM_Bush_L_v3", "SM_oldBush_01", "SM_longLeavesBush_01"]

FURNITURE = {   # (OSM key, value) -> meshes
    ("amenity", "bench"): ["SM_StreetBench", "SM_Bench01"],
    ("leisure", "picnic_table"): ["SM_Bench02"],
    ("amenity", "waste_basket"): ["SM_TrashCan1", "SM_TrashCan2", "SM_TrashCan04", "SM_trashCanV3"],
    ("amenity", "waste_disposal"): ["SM_Dumpster_Opt"],
    ("amenity", "recycling"): ["SM_Container_Opt", "SM_Dumpster_Opt"],
    ("amenity", "post_box"): ["SM_MailBox"],
    ("amenity", "bicycle_parking"): ["SM_BikeParking"],
    ("amenity", "fountain"): ["SM_fountainSmall", "SM_Fountain_02"],
    ("amenity", "drinking_water"): ["SM_StreetFountain"],
    ("amenity", "vending_machine"): ["SM_CarlaCola"],
    ("amenity", "atm"): ["SM_Atm"],
    ("amenity", "telephone"): ["SM_LightBox1"],
    ("amenity", "shelter"): ["SM_BusStop"],
    ("barrier", "bollard"): ["SM_Bollard01"],
    ("barrier", "planter"): ["SM_PlantpotM2", "SM_PlantPot06"],
    ("emergency", "fire_hydrant"): ["SM_FireHdrant"],
    ("advertising", "column"): ["SM_Billboard1"],
    ("advertising", "billboard"): ["SM_StreetAD01"],
    ("advertising", "board"): ["SM_StreetAD02", "SM_StreetAD03"],
    ("advertising", "poster_box"): ["SM_LightBox2"],
    ("man_made", "street_cabinet"): ["SM_StreetCounter"],
    ("information", "board"): ["SM_MapTable"],
    ("playground", "swing"): ["SM_Swing"],
    ("playground", "slide"): ["SM_Slide"],
}
LAMP, PARK_LAMP = "SM_Streetlight3_group3", "SM_parkLightV2"     # arm towards +x; a park lamp
POST = "SM_RoadSigns01"          # a 2.3 m sign post; the sign plates have their face towards +y
SPEEDS = [30, 40, 50, 60, 70, 80, 90, 100, 110, 120]
GRASS, PAVING, WATER = "SM_Square_60", "SideWalkCube", "SM_TileLake"
CONCRETE, DIRT = "SM_Square_61", "SM_Square_131"     # M_Concrete_04; M_Ground01 (grass worn to earth)
# ground kind (OSM fallback or bgt.surface_kind) -> mesh; bricks and tiles get CARLA's sidewalk paving (no
# brick ground mesh in the 0.9.16 catalogue)
GROUND_MESH = {"green": GRASS, "open": PAVING, "closed": CONCRETE, "unpaved": DIRT, "water": WATER}
SPLIT_SHARE = 0.6     # a BGT tile whose main kind covers less than this of its ground is split in four
GREEN_LANDUSE = {"grass", "forest", "meadow", "village_green", "recreation_ground", "cemetery", "allotments",
                 "farmland", "orchard", "flowerbed"}
GREEN_LEISURE = {"park", "garden", "pitch", "playground", "dog_park", "common"}
GREEN_NATURAL = {"wood", "scrub", "grassland", "heath"}
PLANT_SPACING = {"wood": 7.0, "forest": 7.0, "orchard": 6.0, "park": 16.0, "garden": 14.0, "cemetery": 14.0,
                 "common": 20.0, "village_green": 22.0, "recreation_ground": 30.0, "scrub": 9.0}


# --- the mesh catalogue --------------------------------------------------------------------------------------

class Catalog:
    """Mesh sizes measured by tools/mesh_catalog.py on a CARLA version, looked up by file name."""

    def __init__(self, version="0.9.16"):
        d = json.loads((DATA / f"meshes_{version}.json").read_text())
        self.version = d["carla"]
        self.by_name = {}
        for path, m in d["meshes"].items():
            self.by_name.setdefault(path.rsplit(".", 1)[-1], (path, m))

    def __contains__(self, name):
        return name in self.by_name

    def optimised(self, name):
        """CARLA's optimised (_Opt, fewer triangles, LODs) version of a mesh if it has one, else the mesh.
        The full Town buildings halve the real-time factor (919 of them: 0.70 -> 0.39 in Stellenbosch)."""
        if not hasattr(self, "_opt"):
            self._opt = {key_of(n): n for n in self.by_name if n.lower().endswith("opt")}
        return self._opt.get(key_of(name), name)

    def path(self, name):
        return self.by_name[name][0]

    def size(self, name):
        """(length x, width y, height z) at scale 1."""
        e = self.by_name[name][1]["ext"]
        return 2 * e[0], 2 * e[1], 2 * e[2]

    def place(self, name, cx, cy, z, yaw, scale, sink=0.0, pitch=0.0, roll=0.0, anchor="bottom"):
        """Actor transform (x, y, z, yaw) that puts the bottom (or top) centre of the mesh's bounding box at
        (cx, cy, z) for a rotation of (pitch, yaw, roll) in degrees (Unreal's rotation order)."""
        m = self.by_name[name][1]
        ox, oy, oz = (scale * v for v in m["off"])
        ez = scale * m["ext"][2]
        lx, ly, lz = ox, oy, (oz - ez if anchor == "bottom" else oz + ez)
        X, Y, Z = _axes(pitch, yaw, roll)
        wx = lx * X[0] + ly * Y[0] + lz * Z[0]
        wy = lx * X[1] + ly * Y[1] + lz * Z[1]
        wz = lx * X[2] + ly * Y[2] + lz * Z[2]
        return cx - wx, cy - wy, z - wz - sink, yaw


def _axes(pitch, yaw, roll):
    """The local x, y, z axes in world coordinates for an Unreal FRotator (degrees)."""
    p, y, r = (math.radians(v) for v in (pitch, yaw, roll))
    sp, cp, sy, cy, sr, cr = math.sin(p), math.cos(p), math.sin(y), math.cos(y), math.sin(r), math.cos(r)
    return ((cp * cy, cp * sy, sp),
            (sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp),
            (-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp))


def key_of(name):
    return name.lower().replace("_merged", "").replace("_opt", "").replace("opt", "")


def _h(*parts):
    return zlib.crc32("|".join(str(p) for p in parts).encode())


# --- geometry helpers ----------------------------------------------------------------------------------------

def _rect_of(poly):
    """Minimum rotated rectangle of a polygon -> (cx, cy, length, width, yaw of the length axis in degrees)."""
    r = poly.minimum_rotated_rectangle
    xs, ys = r.exterior.coords.xy
    p = list(zip(xs, ys))[:4]
    e1 = (p[1][0] - p[0][0], p[1][1] - p[0][1])
    e2 = (p[2][0] - p[1][0], p[2][1] - p[1][1])
    l1, l2 = math.hypot(*e1), math.hypot(*e2)
    if l1 >= l2:
        length, width, yaw = l1, l2, math.degrees(math.atan2(e1[1], e1[0]))
    else:
        length, width, yaw = l2, l1, math.degrees(math.atan2(e2[1], e2[0]))
    c = r.centroid
    return c.x, c.y, length, width, yaw


def split_footprint(poly, depth=0, max_width=60.0):
    """Rectangles (cx, cy, l, w, yaw) covering a footprint: halve the bounding rectangle until it fits well."""
    from shapely.affinity import rotate
    from shapely.geometry import box
    if poly.is_empty or poly.area < 1.0:
        return []
    cx, cy, l, w, yaw = _rect_of(poly)
    fill = poly.area / max(l * w, 1e-6)
    if depth < 4 and poly.area > 150 and (fill < 0.75 or w > max_width or l > 2.5 * max_width):
        # cut across the long axis through the centre
        half = box(cx - l, cy - w * 2, cx, cy + w * 2)
        other = box(cx, cy - w * 2, cx + l, cy + w * 2)
        out = []
        for b in (half, other):
            piece = poly.intersection(rotate(b, yaw, origin=(cx, cy)))
            for g in getattr(piece, "geoms", [piece]):
                if g.geom_type == "Polygon" and g.area > 8:
                    out += split_footprint(g, depth + 1, max_width)
        return out
    return [(cx, cy, l, w, yaw)]


ROOF_OF = {"house": "pitched", "church": "pitched"}     # the other CARLA building kinds have flat roofs
PITCHED = {"gabled", "hipped", "half-hipped", "gambrel", "mansard", "saltbox", "pyramidal", "skillion", "round",
           "dome", "onion", "cone", "side_hipped"}


def _roof(value):
    """A roof description (3DBAG b3_dak_type, OSM roof:shape) -> "pitched", "flat" or None."""
    v = (value or "").lower()
    if v in ("slanted",) or v in PITCHED:
        return "pitched"
    if "horizontal" in v or v == "flat":
        return "flat"
    return None


def fit_building(cat, rect, height, kinds, key, roof=None):
    """Best (mesh, n, scale, yaw_extra) for a rectangle: the mesh repeated n times along its length. A known roof
    shape (pitched or flat, from 3DBAG or OSM) steers the choice between pitched (house, church) and flat kinds."""
    cx, cy, l, w, yaw = rect
    best = []
    kinds = kinds or ["apartments", "house", "commercial"]
    if height > 45 and "tower" not in kinds:
        kinds = ["tower"] + kinds
    pool = [(k, cat.optimised(m)) for k in BUILDINGS for m in BUILDINGS[k] if m in cat and m not in BROKEN]
    for kind, mesh in pool:
        L, W, H = cat.size(mesh)
        turn = 0.0
        if W > L:
            L, W, turn = W, L, 90.0
        if kind == "church" and "church" not in kinds:
            continue        # a church only where the data says church (else big pitched roofs become churches)
        penalty = 0.0 if kind in kinds[:1] else 0.35 if kind in kinds else 1.0
        if roof and ROOF_OF.get(kind, "flat") != roof:
            penalty += 0.3
        for n in range(1, 9):
            pl = l / n
            if pl < 0.4 * w and n > 1:
                break
            s = min(pl / L, w / W)
            s = max(0.6, min(1.5, s))
            if s * L > pl * 1.2 or s * W > w * 1.2:
                continue
            cover = (s * L * s * W) / (pl * w)
            score = (1 - cover) + 0.9 * abs(math.log(s * H / max(height, 2.5))) + 0.25 * abs(math.log(s)) \
                + penalty + 0.04 * (n - 1)
            best.append((score, mesh, n, s, turn))
    if not best:
        return None
    best.sort()
    near = [b for b in best[:6] if b[0] <= best[0][0] + 0.06]
    return near[_h(key) % len(near)][1:]


def _tree_family(tags, lat):
    genus = (tags.get("genus") or tags.get("species") or tags.get("taxon") or "").split(" ")[0].lower()
    fam = GENUS.get(genus)
    if fam:
        return fam
    if tags.get("leaf_type") == "needleleaved":
        return "pine"
    if abs(lat) < 24:
        return ["palm", "tropical", "maple", "oak"]
    if abs(lat) < 35:
        return ["oak", "maple", "ash", "palm", "cypress"]
    return ["maple", "oak", "ash", "beech", "sassafras"]


def _tree(cat, fam, height, key):
    """(mesh, scale) of a family nearest to the height (m), or a usual street-tree size when not given."""
    if isinstance(fam, list):
        fam = fam[_h(key, "fam") % len(fam)]
    meshes = [m for m in TREES[fam] if m in cat] or [m for m in TREES["maple"] if m in cat]
    target = height or (7.0 + (_h(key, "h") % 70) / 10.0)       # 7 to 14 m
    mesh = min(meshes, key=lambda m: abs(math.log(cat.size(m)[2] / target)) + (_h(key, m) % 10) / 100.0)
    return mesh, max(0.5, min(1.6, target / cat.size(mesh)[2]))


def _number(text):
    import re
    m = re.match(r"\s*([0-9]+(?:\.[0-9]+)?)", str(text or ""))
    return float(m.group(1)) if m else None


# --- build ---------------------------------------------------------------------------------------------------

class Placer:
    def __init__(self, cat, cmap, ground, carla):
        self.cat, self.cmap, self.ground, self.carla = cat, cmap, ground, carla
        self.items, self.counts = [], defaultdict(int)
        self.mesh_index = {}
        lt = carla.LaneType
        self.blocking = lt.Driving | lt.Parking | lt.Biking | lt.Bidirectional | lt.Sidewalk

    def on_lane(self, x, y, z=0.0):
        wp = self.cmap.get_waypoint(self.carla.Location(x=x, y=y, z=z), project_to_road=False,
                                    lane_type=self.blocking)
        return wp is not None

    def footprint_clear(self, cx, cy, l, w, yaw, z):
        r = math.radians(yaw)
        ux, uy, vx, vy = math.cos(r), math.sin(r), -math.sin(r), math.cos(r)
        for a in (-0.5, 0.0, 0.5):
            for b in (-0.5, 0.0, 0.5):
                if self.on_lane(cx + a * l * ux + b * w * vx, cy + a * l * uy + b * w * vy, z):
                    return False
        return True

    def add(self, kind, mesh, cx, cy, yaw, scale, sink=0.0, tilt=None, z=None, anchor="bottom", centre=False):
        g = self.ground(cx, cy) if z is None else z
        x, y, zz, yaw = self.cat.place(mesh, cx, cy, g, yaw, scale, sink, *(tilt or (0.0, 0.0)), anchor=anchor)
        idx = self.mesh_index.setdefault(mesh, len(self.mesh_index))
        item = [idx, round(x, 2), round(y, 2), round(zz, 2), round(yaw % 360, 1), round(scale, 3), kind]
        if tilt or centre:
            item.append([round(tilt[0], 2), round(tilt[1], 2)] if tilt else [0.0, 0.0])
        if centre:      # where the mesh is (Town03 ground meshes have their pivot 100-400 m away): its cell
            item.append([round(cx, 1), round(cy, 1)])
        self.items.append(item)
        self.counts[kind] += 1


def tile_cover(size, T):
    """[((dx, dy), scale)]: copies of a flat mesh of size (L, W) at yaw 0, uniformly scaled, that cover a T x T
    square: its long side spans the square, copies side by side along the short one (overlapping a little)."""
    L, W = size[0], size[1]
    s = T / max(L, W)
    a = min(L, W) * s
    n = max(1, math.ceil(T / a - 1e-6))
    offs = [0.0] if n == 1 else [-T / 2 + a / 2 + i * (T - a) / (n - 1) for i in range(n)]
    return [(((o, 0.0) if L < W else (0.0, o)), s) for o in offs]


def _bgt(mc, sc, rdir, log):
    """The BGT for a map in the Netherlands (bgt.py), or None: off, elsewhere, or the service failed."""
    from . import bgt
    from . import realworld as rw
    want = sc.get("bgt", "auto")
    if want in (False, "off", "false", None) or not rw.in_netherlands(mc["bbox"]):
        return None
    try:
        return bgt.fetch(mc["bbox"], rdir / "bgt.json", rw.UA, log)
    except Exception as err:  # noqa: BLE001 - PDOK down or offline: OSM ground and furniture as before
        log(f"scenery: BGT failed ({err}); OSM ground and furniture only")
        return None


def _ring_xy(geo, ring):
    pts = []
    for lon, lat in ring:
        x, y = geo.net.convertLonLat2XY(lon, lat)
        pts.append((x, -y))
    return pts


def building_records(mc, rdir, geo, dem, base, size_class=None, log=print):
    """Footprints in CARLA coordinates with height, kind and source: national data (3DBAG, BD TOPO, NYC)
    > Overture > OSM."""
    from shapely.geometry import Polygon
    from . import realworld as rw
    bbox = mc["bbox"]
    src_used = []
    national, nat_src = [], None
    if mc.get("national_buildings", True):
        for name, fetch in (("3dbag", rw.threedbag_buildings), ("bdtopo", rw.bdtopo_buildings),
                            ("nyc", rw.nyc_buildings)):
            try:
                national = fetch(bbox, rdir / f"{name}.json")
            except Exception as err:  # noqa: BLE001 - a national service down: Overture/OSM heights instead
                log(f"{name} failed ({err})")
                national = []
            if national:
                nat_src = name
                break
    ovt = rw.overture_buildings(bbox, rdir / "overture_buildings.json") if mc.get("overture", True) else None
    recs = []
    if national:
        src_used.append(nat_src)
        for b in national:
            recs.append({"ring": b["ring"], "h": b["height"], "cls": b.get("cls"), "src": nat_src,
                         "roof": _roof(b.get("roof"))})
        if ovt:     # kinds from Overture (OSM tags) by centroid, where the national data has none
            from shapely.strtree import STRtree
            polys = [Polygon(o["ring"]) for o in ovt]
            tree = STRtree(polys)
            for r in recs:
                if r["cls"]:
                    continue
                c = Polygon(r["ring"]).centroid
                for i in tree.query(c):
                    if polys[i].contains(c):
                        r["cls"] = ovt[i]["cls"]
                        break
            src_used.append("overture")
    elif ovt:
        src_used.append("overture")
        for o in ovt:
            h = o["height"] or ((o["floors"] * 3.2 + 1.5) if o["floors"] else None)
            recs.append({"ring": o["ring"], "h": h, "cls": o["cls"], "src": (o["src"] or "overture")})
    else:
        src_used.append("osm")
        data = rw.osm_buildings(bbox, rdir / "osm_buildings.json")
        for el in data.get("elements", []):
            geom, tags = el.get("geometry"), el.get("tags", {})
            if not geom or len(geom) < 3:
                continue
            h = _number(tags.get("height"))
            if h is None and _number(tags.get("building:levels")) is not None:
                h = 3.2 * _number(tags.get("building:levels")) + 1.5
            recs.append({"ring": [[g["lon"], g["lat"]] for g in geom], "h": h, "cls": tags.get("building"),
                         "src": "osm", "roof": _roof(tags.get("roof:shape"))})
    out = []
    for r in recs:
        pts = _ring_xy(geo, r["ring"])
        if len(pts) < 3:
            continue
        poly = Polygon(pts)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty or poly.area < 12:
            continue
        kinds = KIND_OF.get(r["cls"] or "", None)
        h = r["h"]
        if h is None or h <= 1.0:
            k = kinds[0] if kinds else None
            h = DEFAULT_HEIGHT[k]
            if k is None:    # unknown use: by footprint size
                h = 4.0 if poly.area < 50 else 7.5 if poly.area < 160 else 11.0
            if size_class == "large" and h >= 10:
                h *= 1.5
        lat = sum(p[1] for p in r["ring"]) / len(r["ring"])
        lon = sum(p[0] for p in r["ring"]) / len(r["ring"])
        zs = [dem.sample(p[1], p[0]) - base for p in r["ring"]] if dem is not None else [0.0]
        out.append({"poly": poly, "h": round(float(h), 1), "cls": r["cls"], "kinds": kinds, "src": r["src"],
                    "z": round(float(min(zs)), 2), "lat": lat, "lon": lon, "roof": r.get("roof")})
    return out, src_used


def build(mdir, cfg, geo, dem=None, base=0.0, cmap=None, log=print):
    """Write scenery.json (placements) and buildings.json (footprints for the viewer) for a built map."""
    import carla
    from shapely.geometry import LineString, Point, Polygon, box
    from shapely.ops import unary_union
    from shapely.strtree import STRtree
    from . import realworld as rw

    mc = cfg["map"]
    sc = mc.get("scenery") or {}
    rdir = mdir / "real"
    cat = Catalog(sc.get("catalog", "0.9.16"))
    if cmap is None:
        cmap = carla.Map(mc["name"], (mdir / "map.xodr").read_text(encoding="utf-8"))
    lat0 = (mc["bbox"][0] + mc["bbox"][2]) / 2

    if dem is not None:
        def ground(x, y):
            lat, lon = geo.to_latlon(x, y)
            return dem.sample(lat, lon) - base
    else:
        def ground(x, y):
            return 0.0

    pl = Placer(cat, cmap, ground, carla)
    rng = random.Random(42)
    sources = {"osm"}

    # buildings
    blds, bsrc = building_records(mc, rdir, geo, dem, base, (cfg.get("city") or {}).get("size"), log)
    sources.update(bsrc)
    (mdir / "buildings.json").write_text(json.dumps(
        [{"h": b["h"], "z": b["z"], "k": b["cls"], "s": b["src"],
          "p": [[round(x, 2), round(y, 2)] for x, y in b["poly"].exterior.coords]} for b in blds],
        separators=(",", ":")))
    occupied = [b["poly"] for b in blds]
    if sc.get("buildings", True):
        for bi, b in enumerate(blds):
            for ri, rect in enumerate(split_footprint(b["poly"])):
                if rect[2] < 3.0 or rect[3] < 2.5:
                    continue
                fit = fit_building(cat, rect, b["h"], b["kinds"], (bi, ri), b.get("roof"))
                if fit is None:
                    continue
                mesh, n, s, turn = fit
                cx, cy, l, w, yaw = rect
                r = math.radians(yaw)
                L, W, H = cat.size(mesh)
                if turn:
                    L, W = W, L
                for k in range(n):
                    off = (k + 0.5) * l / n - l / 2
                    px, py = cx + off * math.cos(r), cy + off * math.sin(r)
                    ss = s
                    for _ in range(3):
                        if pl.footprint_clear(px, py, ss * L, ss * W, yaw, b["z"]):
                            break
                        ss *= 0.85
                    else:
                        pl.counts["building_dropped"] += 1
                        continue
                    # tall buildings stay loaded all trial (the skyline); others stream with their cell
                    pl.add("landmark" if b["h"] > 40 else "building", mesh, px, py, yaw + turn, ss, sink=0.3,
                           z=b["z"])
        log(f"scenery: {pl.counts['building'] + pl.counts['landmark']} building meshes for {len(blds)} footprints ({'+'.join(bsrc)})")

    # OSM features
    osm = rw.osm_scenery(mc["bbox"], rdir / "osm_scenery.json")
    els = osm.get("elements", [])
    # the city's own tree and street-light registers (fuller than OSM where they exist): they replace OSM's
    # trees within 3 m of one of theirs, and all OSM and guessed lamps
    muni = rw.municipal_scenery(mc["bbox"], rdir / "municipal.json") if sc.get("municipal", True) else {}
    m_els = muni.get("elements", [])
    m_trees = [e for e in m_els if e["tags"].get("natural") == "tree"]
    m_lamps = any(e["tags"].get("highway") == "street_lamp" for e in m_els)
    if m_trees:
        tpts = [Point(geo.to_carla(e["lat"], e["lon"])) for e in m_trees]
        ttree = STRtree(tpts)

        def dup(el):
            p = Point(geo.to_carla(el["lat"], el["lon"]))
            return any(tpts[i].distance(p) < 3.0 for i in ttree.query(p.buffer(3.0)))
        els = [el for el in els if not (el["type"] == "node" and el.get("tags", {}).get("natural") == "tree"
                                        and dup(el))]
    if m_lamps:
        els = [el for el in els if el.get("tags", {}).get("highway") != "street_lamp"]
    els = els + m_els
    for src in muni.get("sources", []):
        sources.add(src)
    # the Dutch BGT (bgt.py): what covers the ground beside the roads, and the municipalities' street furniture
    surf, bgt_data = None, _bgt(mc, sc, rdir, log)
    if bgt_data:
        from . import bgt
        n_osm = len(els)
        els, st = bgt.merge(els, bgt_data, geo, skip_lamps=m_lamps, skip_trees=bool(m_trees))
        surf = bgt.Surfaces(bgt_data, geo)
        sources.add("bgt")
        pl.counts["bgt_surfaces"], pl.counts["bgt_objects"] = len(surf), st["bgt_objects"]
        pl.counts["bgt_added"], pl.counts["bgt_replaced"] = sum(st["added"].values()), sum(st["replaced"].values())
        log(f"scenery: BGT {len(surf)} ground surfaces; {st['bgt_objects']} plus objects -> "
            f"{pl.counts['bgt_added']} added ({', '.join(f'{k} {v}' for k, v in sorted(st['added'].items()))}), "
            f"replacing {pl.counts['bgt_replaced']} of {n_osm} OSM/register features "
            f"({', '.join(f'{k} {v}' for k, v in sorted(st['replaced'].items()))}); "
            f"{st['skipped_register']} left to the city's register")
    green, water, tunnels = [], [], []
    for el in els:
        t = el.get("tags", {})
        geom = el.get("geometry")
        if el["type"] == "relation":
            parts = [m.get("geometry") for m in el.get("members", []) if m.get("role") == "outer" and m.get("geometry")]
        elif geom:
            parts = [geom]
        else:
            continue
        for g in parts:
            if len(g) < 4 or g[0] != g[-1] and el["type"] != "relation":
                continue
            try:
                poly = Polygon(_ring_xy(geo, [[p["lon"], p["lat"]] for p in g]))
            except ValueError:
                continue
            if not poly.is_valid:
                poly = poly.buffer(0)
            if poly.is_empty:
                continue
            nat, lu, le = t.get("natural"), t.get("landuse"), t.get("leisure")
            if nat == "water" or lu in ("basin", "reservoir") or t.get("waterway") == "riverbank":
                water.append(poly)
            elif nat in GREEN_NATURAL or lu in GREEN_LANDUSE or le in GREEN_LEISURE:
                green.append((poly, nat or lu or le))
    if surf is not None and surf.wood:     # BGT woods and shrubs that OSM does not map: planted like OSM's
        osm_green = unary_union([g for g, _ in green]) if green else None
        for poly, kind in surf.wood:
            extra = poly.difference(osm_green) if osm_green is not None else poly
            green += [(g, kind) for g in getattr(extra, "geoms", [extra]) if g.geom_type == "Polygon"]
    water_u = unary_union(water) if water else None
    green_u = unary_union([g for g, _ in green]) if green else None
    occ_tree = STRtree(occupied) if occupied else None

    def inside_building(x, y):
        if occ_tree is None:
            return False
        p = Point(x, y)
        return any(occupied[i].contains(p) for i in occ_tree.query(p))

    def free(x, y, r=0.6):
        if pl.on_lane(x, y, ground(x, y)) or inside_building(x, y):
            return False
        if water_u is not None and water_u.contains(Point(x, y)):
            return False
        return all(not pl.on_lane(x + dx, y + dy) for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r)))

    tree_pts = []
    if sc.get("trees", True):
        for el in els:
            t = el.get("tags", {})
            if el["type"] == "node" and t.get("natural") == "tree":
                x, y = geo.to_carla(el["lat"], el["lon"])
                if not free(x, y, 0.4):
                    pl.counts["tree_dropped"] += 1
                    continue
                h = _number(t.get("height"))
                mesh, s = _tree(cat, _tree_family(t, el["lat"]), h, el["id"])
                pl.add("tree", mesh, x, y, _h(el["id"]) % 360, s, sink=0.1)
                tree_pts.append((x, y))
                pl.counts["tree_osm"] += 1
            elif el["type"] == "way" and t.get("natural") == "tree_row" and el.get("geometry"):
                line = LineString(_ring_xy(geo, [[p["lon"], p["lat"]] for p in el["geometry"]]))
                fam = _tree_family(t, lat0)
                for i, d in enumerate(np.arange(2.0, line.length, 8.0)):
                    p = line.interpolate(d)
                    if not free(p.x, p.y, 0.4):
                        continue
                    mesh, s = _tree(cat, fam, _number(t.get("height")), (el["id"], 0))   # one size per row
                    pl.add("tree", mesh, p.x, p.y, _h(el["id"], i) % 360, s, sink=0.1)
                    tree_pts.append((p.x, p.y))
                    pl.counts["tree_row"] += 1
        # parks, gardens and woods
        pts_tree = STRtree([Point(p) for p in tree_pts]) if tree_pts else None
        planted = 0
        for poly, kind in green:
            spacing = PLANT_SPACING.get(kind)
            if not spacing or poly.area < 200:
                continue
            x0, y0, x1, y1 = poly.bounds
            fam = _tree_family({}, lat0)
            for gx in np.arange(x0 + spacing / 2, x1, spacing):
                for gy in np.arange(y0 + spacing / 2, y1, spacing):
                    x = gx + rng.uniform(-0.35, 0.35) * spacing
                    y = gy + rng.uniform(-0.35, 0.35) * spacing
                    p = Point(x, y)
                    if not poly.contains(p) or not free(x, y, 1.0):
                        continue
                    if pts_tree is not None and any(Point(tree_pts[i]).distance(p) < 4.0
                                                   for i in pts_tree.query(p.buffer(4.0))):
                        continue
                    if kind == "scrub" or (kind in ("park", "garden") and rng.random() < 0.25):
                        mesh = BUSHES[_h(x, y) % len(BUSHES)]
                        if mesh in cat:
                            pl.add("bush", mesh, x, y, rng.uniform(0, 360), rng.uniform(0.7, 1.2))
                        continue
                    mesh, s = _tree(cat, fam, None, (round(x), round(y)))
                    pl.add("tree", mesh, x, y, rng.uniform(0, 360), s, sink=0.1)
                    planted += 1
        pl.counts["tree_planted"] = planted
        log(f"scenery: {pl.counts['tree']} trees ({pl.counts['tree_osm']} street trees (OSM and registers), "
            f"{pl.counts['tree_row']} in rows, {planted} planted in parks and woods), {pl.counts['bush']} bushes")

    # street furniture and lamps
    if sc.get("furniture", True):
        def road_yaw(x, y):
            wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True)
            if wp is None:
                return 0.0, None
            loc = wp.transform.location
            return wp.transform.rotation.yaw, math.degrees(math.atan2(loc.y - y, loc.x - x))

        lamps = 0
        for el in els:
            if el["type"] != "node":
                continue
            t = el.get("tags", {})
            x, y = geo.to_carla(el["lat"], el["lon"])
            if t.get("highway") == "street_lamp":
                if not free(x, y, 0.3):
                    continue
                _, towards = road_yaw(x, y)
                wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True)
                far = wp is None or wp.transform.location.distance(carla.Location(x=x, y=y)) > 9
                far = far or (_number(t.get("height")) or 99) <= 4.5      # a low mast (register): a park lamp
                pl.add("lamp", PARK_LAMP if far else LAMP, x, y, 0.0 if far or towards is None else towards, 1.0)
                lamps += 1
                continue
            meshes = None
            if t.get("highway") == "bus_stop" or (t.get("public_transport") == "platform" and t.get("bus") == "yes"):
                meshes = ["SM_BusStop"] if t.get("shelter", "yes" if t.get("highway") != "bus_stop" else "no") \
                    in ("yes", "true") else None
            for (k, v), ms in FURNITURE.items():
                if t.get(k) == v:
                    meshes = ms
            if not meshes:
                continue
            meshes = [m for m in meshes if m in cat]
            if not meshes or not free(x, y, 0.2):
                continue
            ryaw, towards = road_yaw(x, y)
            yaw = ryaw if towards is None else towards + 90.0     # backs to the buildings, parallel to the road
            pl.add("furniture", meshes[_h(el["id"]) % len(meshes)], x, y, yaw, 1.0)
        # lamps along the roads where OSM maps few
        lane_km = sum(1 for w in cmap.generate_waypoints(10.0) if w.lane_type == carla.LaneType.Driving) / 100.0
        if sc.get("guess_lamps", True) and lamps < 8 * lane_km and not m_lamps:
            spacing = 32.0
            done = 0
            seen = set()
            for wp in cmap.generate_waypoints(spacing):
                if wp.lane_type != carla.LaneType.Driving or wp.is_junction:
                    continue
                # the outermost lane on this side of the road
                edge = wp
                for _ in range(8):
                    nxt = edge.get_right_lane()
                    if nxt is None or nxt.lane_type in (carla.LaneType.NONE,):
                        break
                    if nxt.lane_id * edge.lane_id < 0:
                        break
                    edge = nxt
                if edge.lane_id != wp.lane_id and edge.lane_type == carla.LaneType.Driving:
                    continue          # an inner lane: its outer lane gets the lamp
                key = (edge.road_id, edge.section_id, edge.lane_id, round(edge.s / spacing))
                if key in seen:
                    continue
                seen.add(key)
                tf = edge.transform
                right = tf.get_right_vector()
                off = edge.lane_width / 2 + 1.0
                x, y = tf.location.x + right.x * off, tf.location.y + right.y * off
                if not free(x, y, 0.3):
                    continue
                pl.add("lamp", LAMP, x, y, tf.rotation.yaw + 90.0 + 180.0, 1.0)     # arm over the road
                done += 1
            log(f"scenery: OSM maps {lamps} street lamps; {done} more along the roads")
        elif m_lamps:
            log(f"scenery: {lamps} street lamps from the city's register")
        log(f"scenery: {pl.counts['furniture']} pieces of street furniture, {pl.counts['lamp']} lamps")

    # road signs
    if sc.get("signs", True) and POST in cat:
        import re
        lefthand = bool(mc.get("lefthand"))
        speed_re = re.compile(r"(?:maxspeed|A1|274|B14|C43|R2-1|C14|R5)\D{0,3}(\d{2,3})", re.I)
        for el in els:
            if el["type"] != "node":
                continue
            t = el.get("tags", {})
            sign = None
            ts = t.get("traffic_sign", "")
            # NL B7 / DE 206 / FR AB4 / US R1-1: stop; NL B6 / DE 205 / FR AB3 / US R1-2: give way
            if t.get("highway") == "stop" or re.search(r"stop|R1-1|:B7\b|:206\b|AB4", ts, re.I):
                sign = "SM_stopSign"
            elif t.get("highway") == "give_way" or re.search(r"give_way|R1-2|:B6\b|:205\b|AB3", ts, re.I):
                sign = "SM_yield_"
            else:
                m = speed_re.search(ts) or (re.match(r"(\d{2,3})", t.get("maxspeed", "")) if ts else None)
                if m:
                    v = int(m.group(1))
                    sign = f"SM_SpeedSign{min(SPEEDS, key=lambda s_: abs(s_ - v))}"
            if sign is None or sign not in cat:
                continue
            x, y = geo.to_carla(el["lat"], el["lon"])
            wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp is None or wp.transform.location.distance(carla.Location(x=x, y=y, z=wp.transform.location.z)) > 25:
                continue
            if t.get("direction") == "backward" or t.get("traffic_sign:direction") == "backward":
                other = wp.get_left_lane()
                while other is not None and other.lane_id * wp.lane_id > 0:
                    other = other.get_left_lane()
                if other is not None and other.lane_type == carla.LaneType.Driving:
                    wp = other
            edge = wp
            for _ in range(8):     # the outermost lane on the traffic side
                nxt = edge.get_left_lane() if lefthand else edge.get_right_lane()
                if nxt is None or nxt.lane_id * edge.lane_id < 0 or nxt.lane_type != carla.LaneType.Driving:
                    break
                edge = nxt
            tf = edge.transform
            side = tf.get_right_vector()
            k = (-1 if lefthand else 1) * (edge.lane_width / 2 + 0.8)
            px, py = tf.location.x + side.x * k, tf.location.y + side.y * k
            if pl.on_lane(px, py, tf.location.z):
                continue
            face = tf.rotation.yaw + 90.0     # plate face (+y) towards the oncoming traffic (checked in a render)
            gz = tf.location.z
            pl.add("sign", POST, px, py, face, 1.0, z=gz)
            L, W, H = cat.size(sign)
            pl.add("sign", sign, px, py, face, 1.0, z=gz + cat.size(POST)[2] - H - 0.05)
        log(f"scenery: {pl.counts['sign'] // 2} road signs from OSM")

    # ground tiles: tilted to the terrain, and always below the road surface (which follows the terrain
    # model only at the road nodes) and below the pavements
    if sc.get("ground", True):
        pts = [w.transform.location for w in cmap.generate_waypoints(40.0)]
        xs, ys = [p.x for p in pts], [p.y for p in pts]
        TILE = TILE_M
        if dem is not None:     # hilly maps: 20 m tiles, so the terraces between tiles stay low
            heights = [p.z for p in pts]
            if max(heights) - min(heights) > 40:
                TILE = TILE_M / 2
        road = defaultdict(list)
        for wp in cmap.generate_waypoints(2.0):
            tf = wp.transform
            r = tf.get_right_vector()
            for k in (-0.5, 0.0, 0.5):
                x = tf.location.x + r.x * wp.lane_width * k
                y = tf.location.y + r.y * wp.lane_width * k
                road[(int(x // TILE), int(y // TILE))].append((x, y, tf.location.z))
        pad = 160.0
        gx0, gy0 = math.floor((min(xs) - pad) / TILE) * TILE, math.floor((min(ys) - pad) / TILE) * TILE
        gx1, gy1 = max(xs) + pad, max(ys) + pad
        tiles = lowered = 0
        bgt_tiles, changed, split, kinds, moves = 0, 0, 0, defaultdict(int), defaultdict(int)

        def lay(kind, x, y, T):
            """One square of ground: tilted to the terrain, under the road surface, covered by its mesh."""
            nonlocal lowered
            z = ground(x, y) - 0.05
            zx = (ground(x + T / 2, y) - ground(x - T / 2, y)) / T if dem is not None else 0.0
            zy = (ground(x, y + T / 2) - ground(x, y - T / 2)) / T if dem is not None else 0.0
            key = (int(x // TILE), int(y // TILE))
            near = [q for i in (-1, 0, 1) for j in (-1, 0, 1) for q in road.get((key[0] + i, key[1] + j), [])
                    if abs(q[0] - x) <= T / 2 + 1 and abs(q[1] - y) <= T / 2 + 1]
            if near:
                gap = min(q[2] - (z + zx * (q[0] - x) + zy * (q[1] - y)) for q in near)
                if gap < 0.15:
                    z += gap - 0.15
                    lowered += 1
            tilt = (math.degrees(math.atan(zx)), -math.degrees(math.atan(zy)))
            mesh = GROUND_MESH.get(kind, PAVING)
            if mesh not in cat:
                mesh, kind = PAVING, "open"
            dz = {"water": -0.5, "open": -0.02}.get(kind, 0.0)
            for i, ((dx, dy), s) in enumerate(tile_cover(cat.size(mesh), T)):
                pl.add("water" if kind == "water" else "ground", mesh, x + dx, y + dy, 0.0, s,
                       z=z + zx * dx + zy * dy + dz - 0.01 * i, tilt=tilt, anchor="top", centre=True)
            kinds[kind] += 1

        for x in np.arange(gx0 + TILE / 2, gx1, TILE):
            for y in np.arange(gy0 + TILE / 2, gy1, TILE):
                p = Point(x, y)
                if water_u is not None and water_u.contains(p):
                    osm_kind = "water"
                elif green_u is not None and green_u.contains(p):
                    osm_kind = "green"
                else:
                    osm_kind = "open"
                parts = [(x, y, TILE, osm_kind)]
                if surf is not None and osm_kind != "water":
                    kind, share = surf.classify(x, y, TILE)
                    if kind is not None:
                        bgt_tiles += 1
                        parts = [(x, y, TILE, kind)]
                        if share < SPLIT_SHARE and kind != "water":
                            q = TILE / 4
                            parts = [(x + a, y + b, TILE / 2, surf.classify(x + a, y + b, TILE / 2)[0] or kind)
                                     for a in (-q, q) for b in (-q, q)]
                            split += 1
                        changed += any(k != osm_kind for *_, k in parts)
                        for *_, k in parts:
                            if k != osm_kind:
                                moves[f"{osm_kind}->{k}"] += 1
                for px, py, T, kind in parts:
                    lay(kind, px, py, T)
                tiles += 1
        if surf is not None:
            log(f"scenery: BGT chose {bgt_tiles} of {tiles} tiles ({changed} changed from OSM, {split} split in four)"
                f": {', '.join(f'{k} {v}' for k, v in sorted(kinds.items()))}; squares changed "
                f"{', '.join(f'{k} {v}' for k, v in sorted(moves.items(), key=lambda kv: -kv[1]))}")
            pl.counts["ground_bgt"], pl.counts["ground_changed"], pl.counts["ground_split"] = bgt_tiles, changed, split
        log(f"scenery: {tiles} ground tiles of {TILE:.0f} m ({lowered} lowered under the road surface)")

    meshes = sorted(pl.mesh_index, key=pl.mesh_index.get)
    out = {"version": 1, "catalog": cat.version, "cell_m": CELL, "meshes": [cat.path(m) for m in meshes],
           "items": pl.items, "counts": dict(pl.counts), "sources": sorted(sources)}
    (mdir / "scenery.json").write_text(json.dumps(out, separators=(",", ":")))
    log(f"scenery: {len(pl.items)} placements -> {mdir / 'scenery.json'}")
    return out


# --- run time ------------------------------------------------------------------------------------------------

class Scenery:
    """Spawns scenery.json's placements in cells around focus points and removes them behind."""

    def __init__(self, client, world, mdir, sc, log=print, on_spawn=None):
        self.client, self.world, self.log, self.on_spawn = client, world, log, on_spawn
        self.sc = sc
        data = json.loads((Path(mdir) / "scenery.json").read_text())
        self.meshes = data["meshes"]
        kinds = set(sc.get("kinds") or ["building", "tree", "bush", "lamp", "furniture", "sign", "ground", "water"])
        self.radius = float(sc.get("radius_m", 400.0))
        self.cell = float(data.get("cell_m", CELL))
        self.cells = defaultdict(list)
        self.always = []
        for it in data["items"]:
            if it[6] == "landmark" and "building" in kinds:
                self.always.append(it)
            elif it[6] not in kinds:
                continue
            else:
                cx, cy = it[8] if len(it) > 8 else it[1:3]      # the mesh's centre where its pivot is far off
                self.cells[(int(cx // self.cell), int(cy // self.cell))].append(it)
        self.loaded = {}
        self.bp = world.get_blueprint_library().find("static.prop.mesh")
        self.n = 0
        if self.always:
            self._spawn(("always",), self.always)

    def _spawn(self, key, items):
        import carla
        cmds = []
        for it in items:
            self.bp.set_attribute("mesh_path", self.meshes[it[0]])
            self.bp.set_attribute("scale", str(it[5]))
            pitch, roll = (it[7] if len(it) > 7 else (0.0, 0.0))
            tf = carla.Transform(carla.Location(x=it[1], y=it[2], z=it[3]),
                                 carla.Rotation(pitch=pitch, yaw=it[4], roll=roll))
            cmds.append(carla.command.SpawnActor(self.bp, tf))
        ids = [r.actor_id for r in self.client.apply_batch_sync(cmds, False) if not r.error]
        self.loaded[key] = ids
        self.n += len(ids)
        if self.on_spawn:
            self.on_spawn(ids)

    def update(self, points):
        """Load the cells within radius_m of any point, unload those beyond radius + one cell."""
        import carla
        if self.radius <= 0:
            want = set(self.cells)
        else:
            want, keep = set(), set()
            r = self.radius
            for p in points:
                cx, cy = p.x // self.cell, p.y // self.cell
                k = int(r // self.cell) + 1
                for i in range(int(cx) - k, int(cx) + k + 1):
                    for j in range(int(cy) - k, int(cy) + k + 1):
                        dx = max(0.0, abs((i + 0.5) * self.cell - p.x) - self.cell / 2)
                        dy = max(0.0, abs((j + 0.5) * self.cell - p.y) - self.cell / 2)
                        d = math.hypot(dx, dy)
                        if d <= r:
                            want.add((i, j))
                        if d <= r + self.cell:
                            keep.add((i, j))
            for key in [k for k in self.loaded if k != ("always",) and k not in keep]:
                ids = self.loaded.pop(key)
                self.client.apply_batch([carla.command.DestroyActor(i) for i in ids])
                self.n -= len(ids)
        new = [k for k in want if k in self.cells and k not in self.loaded]
        for key in new:
            self._spawn(key, self.cells[key])
        return len(new)

    def destroy(self):
        import carla
        ids = [i for v in self.loaded.values() for i in v]
        if ids:
            self.client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], False)
        self.loaded, self.n = {}, 0
