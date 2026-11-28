"""Placement rules shared by the scenery, the signals and tools/placement_audit.py.

- carla_xodr(): the OpenDRIVE given to CARLA without its traffic lights. CARLA 0.9.16 builds a traffic-light actor
  for every dynamic <signal> of a world generated from OpenDRIVE: its North American heads, hung at the signal's
  zOffset (netconvert writes 5 m) with a mast arm and pole for the first head of a group only, so the others float.
  Those actors cannot be linked to SUMO and stay red (signals.py); SUMO's lights are drawn by signals.py instead.
  The SUMO net and the map.xodr file keep their signals (netconvert_carla needs them).
- JunctionAreas: the paved area of a generated map's junctions (the union of its connecting lanes with the gaps
  between them filled), which CARLA paves as one surface. A point there is on the road for every car turning
  through the junction, even where it is on no lane: the islands that OSM's give-way signs and register signs
  stand on in the real street do not exist in a generated map.
- approach(): the lane a junction lane comes from (a sign, a signal or a lamp belongs at its kerb, not in the
  junction).
- classify(): the audit's verdict on one placed object (floating, sunk, tilted).
"""
import math
import re

FLOAT_M = 0.15       # the bottom of an object this far over the ground under it: floating
SINK_M = 0.30        # this far under it: sunk
TILT_DEG = 8.0       # an upright object (pole, sign, lamp, car) leaning further than this: tilted


WATER_USERDATA = "coupledcarla.water"


def water_userdata(water):
    """The scenery's water bodies (scenery.json "water": [{level, rings}], CARLA metres) as an OpenDRIVE <userData>
    element: one line per ring, "<body> <level> x,y x,y ...". CARLA 1.0 with unreal/patches/
    OpenDriveGenerator_WaterTerrain.patch lowers its generated terrain under them (even-odd over a body's rings, so
    islands stay land); other readers ignore it."""
    lines = []
    for b, w in enumerate(water or []):
        for ring in w.get("rings") or []:
            if len(ring) >= 3:
                lines.append(f"{b} {float(w['level']):.3f} " + " ".join(f"{x:.1f},{y:.1f}" for x, y in ring))
    if not lines:
        return ""
    return f'<userData code="{WATER_USERDATA}">\n' + "\n".join(lines) + "\n</userData>\n"


def with_water(text, water):
    """The OpenDRIVE text with the water bodies' <userData> before </OpenDRIVE> (unchanged without water)."""
    ud = water_userdata(water)
    if not ud or "</OpenDRIVE>" not in text:
        return text
    i = text.rindex("</OpenDRIVE>")
    return text[:i] + ud + text[i:]


def carla_xodr(text, keep_lights=False, water=None):
    """(OpenDRIVE text for CARLA, number of signals removed). Removes the dynamic signals (traffic lights; static
    signs stay), the <controller>s that group them (a controller naming a missing signal crashes the 0.9.16 server
    while it loads the map) and the junctions' references to those controllers. keep_lights: unchanged (CARLA runs
    the lights: simulation.tls_manager carla). water: the scenery's water bodies, added as <userData>."""
    text = with_water(text, water)
    if keep_lights:
        return text, 0
    gone = set()

    def drop(m):
        if re.search(r'\bdynamic="yes"', m.group(1)):
            gone.add(m.group(2))
            return ""
        return m.group(0)
    # <signal .../> or <signal ...> ... </signal>, with the line's indentation
    pat = re.compile(r'[ \t]*(<signal\b(?=[^>]*\bid="([^"]*)")[^>]*?(?:/>|>.*?</signal>))[ \t]*\r?\n?', re.S)
    text = pat.sub(drop, text)
    text = re.sub(r'[ \t]*<signalReference\b[^>]*\bid="([^"]*)"[^>]*?(?:/>|>.*?</signalReference>)[ \t]*\r?\n?',
                  lambda m: "" if m.group(1) in gone else m.group(0), text, flags=re.S)
    text = re.sub(r'[ \t]*<controller\b[^>]*?(?:/>|>.*?</controller>)[ \t]*\r?\n?', "", text, flags=re.S)
    text = re.sub(r'[ \t]*<signals>\s*</signals>[ \t]*\r?\n?', "", text)
    return text, len(gone)


def fill_holes(geom):
    """A shapely (Multi)Polygon with its interior rings filled."""
    from shapely.geometry import MultiPolygon, Polygon
    if geom.is_empty:
        return geom
    parts = getattr(geom, "geoms", [geom])
    filled = [Polygon(p.exterior) for p in parts if p.geom_type == "Polygon"]
    return filled[0] if len(filled) == 1 else MultiPolygon(filled)


def junction_area(strips, pad=0.3):
    """The paved area of one junction from its lanes: strips [[(x, y, half width), ...] per lane, in order]. Each
    lane is buffered to its width (+ pad), the union's holes (islands between turning lanes) are filled."""
    from shapely.geometry import LineString, Point
    from shapely.ops import unary_union
    parts = []
    for pts in strips:
        if not pts:
            continue
        w = max(p[2] for p in pts) + pad
        if len(pts) == 1:
            parts.append(Point(pts[0][0], pts[0][1]).buffer(w))
        else:
            parts.append(LineString([(p[0], p[1]) for p in pts]).buffer(w, cap_style=2))
    if not parts:
        return None
    return fill_holes(unary_union(parts).buffer(0.5).buffer(-0.5))     # closes slivers between adjacent lanes


class JunctionAreas:
    """Point-in-junction tests for a carla.Map generated from OpenDRIVE (not a baked level: its junctions have real
    islands and kerbs). areas: {junction id: shapely geometry}."""

    def __init__(self, areas):
        from shapely.strtree import STRtree
        self.areas = {k: v for k, v in areas.items() if v is not None and not v.is_empty}
        self._ids = list(self.areas)
        self._geoms = [self.areas[k] for k in self._ids]
        self._tree = STRtree(self._geoms) if self._geoms else None

    @classmethod
    def of_map(cls, cmap, carla, step=1.0, log=None):
        types = (carla.LaneType.Driving, carla.LaneType.Biking, carla.LaneType.Bidirectional)
        strips, seen = {}, set()
        for wp in cmap.generate_waypoints(4.0):     # every junction has a lane waypoint at least every 4 m
            if not wp.is_junction or wp.junction_id in seen:
                continue
            seen.add(wp.junction_id)
            j = wp.get_junction()
            if j is None:
                continue
            for a, _b in (pair for t in types for pair in j.get_waypoints(t)):     # one lane type per call
                pts = [a] + [w for w in a.next_until_lane_end(step) if w.is_junction]
                strips.setdefault(j.id, []).append(
                    [(w.transform.location.x, w.transform.location.y, w.lane_width / 2) for w in pts])
        areas = {k: junction_area(v) for k, v in strips.items()}
        if log:
            log(f"junction areas: {len(areas)} junctions, {sum(a.area for a in areas.values() if a):.0f} m2")
        return cls(areas)

    def which(self, x, y):
        """The id of the junction whose paved area holds (x, y), or None."""
        if self._tree is None:
            return None
        from shapely.geometry import Point
        p = Point(x, y)
        for i in self._tree.query(p):
            if self._geoms[i].contains(p):
                return self._ids[i]
        return None

    def contains(self, x, y):
        return self.which(x, y) is not None


class GroundSurface:
    """The top of the ground tiles scenery.py lays (planes: square centre x, y, side T, top z at the centre, slopes
    zx, zy). The tiles follow the terrain model but are lowered wherever a road crosses them (0.15 m under the road,
    over a whole 40 m tile), so an object placed at the terrain model's height floated over them (eindhoven_centre:
    the median lamp, tree and bench 0.3 m up, parked cars 0.2 m). Objects off the lanes stand on this instead."""
    CELL = 10.0

    def __init__(self, planes=()):
        self.planes, self.grid = [], {}
        for p in planes:
            self.add(*p)

    def add(self, x, y, T, z, zx=0.0, zy=0.0):
        k, c = len(self.planes), self.CELL
        self.planes.append((x, y, T, z, zx, zy))
        for i in range(int((x - T / 2) // c), int((x + T / 2) // c) + 1):
            for j in range(int((y - T / 2) // c), int((y + T / 2) // c) + 1):
                self.grid.setdefault((i, j), []).append(k)

    def z(self, x, y):
        """The ground's top at (x, y) (the higher tile on an edge between two), or None off the tiles."""
        best = None
        for k in self.grid.get((int(x // self.CELL), int(y // self.CELL)), ()):
            px, py, T, z, zx, zy = self.planes[k]
            if abs(x - px) <= T / 2 + 1e-6 and abs(y - py) <= T / 2 + 1e-6:
                v = z + zx * (x - px) + zy * (y - py)
                best = v if best is None else max(best, v)
        return best

    def to_list(self):
        return [[round(x, 2), round(y, 2), round(T, 2), round(z, 3), round(zx, 5), round(zy, 5)]
                for x, y, T, z, zx, zy in self.planes]

    def __len__(self):
        return len(self.planes)


def ground_surface(scenery_json):
    """The GroundSurface of a built scenery.json (its ground_planes), or None (no file, no tiles, older build)."""
    import json
    from pathlib import Path
    try:
        planes = json.loads(Path(scenery_json).read_text()).get("ground_planes")
    except (OSError, ValueError):
        return None
    return GroundSurface(planes) if planes else None


def paved(carla):
    """The lane types CARLA 0.9.16 builds a road surface for in a world from OpenDRIVE (a Restricted lane, a
    pedestrian street in the OSM maps, has none: the ground tiles show there), as one get_waypoint mask."""
    lt = carla.LaneType
    return lt.Driving | lt.Biking | lt.Sidewalk | lt.Parking | lt.Bidirectional


SETTLE_KINDS = {"tree", "bush", "lamp", "furniture", "sign", "shared", "building", "landmark"}
BUILDING_DROP_MAX = 3.0     # a building goes at most this far down to its lowest corner's ground


def settle(items, bases, surface, on_lane, kinds=SETTLE_KINDS, tol=0.02):
    """Stands placed items on the ground tiles: items [scenery item, z at index 3, kind at 6], bases [(x, y, z the
    item was placed on)] aligned with them; on_lane(x, y, z): the spot is on one of CARLA's lanes (its surface is
    the road's: left alone). A sign's plates carry their post's base, so they move with it. A building's base is
    (x, y, z, length, width, yaw), its footprint: it only goes down, to the lowest ground under its corners (its
    foundation sinks into the slope; no corner hangs over a lowered tile). Returns (moved, sum of the shifts)."""
    moved, total = 0, 0.0
    cache = {}
    for it, base in zip(items, bases):
        if base is None or it[6] not in kinds:
            continue
        if len(base) == 6:
            bx, by, bz, length, width, yaw = base
            r = math.radians(yaw)
            ux, uy, vx, vy = math.cos(r), math.sin(r), -math.sin(r), math.cos(r)
            zs = [surface.z(bx + a * length / 2 * ux + b * width / 2 * vx, by + a * length / 2 * uy + b * width / 2 * vy)
                  for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1), (0, 0))]
            zs = [z for z in zs if z is not None]
            d = max(-BUILDING_DROP_MAX, min(zs) - bz) if zs and min(zs) < bz - tol else None
        elif len(base) != 3:
            continue
        else:
            bx, by, bz = base
            key = (round(bx, 2), round(by, 2), round(bz, 3))
            if key not in cache:
                s = surface.z(bx, by)
                cache[key] = None if s is None or on_lane(bx, by, bz) else s - bz
            d = cache[key]
        if d is None or abs(d) <= tol:
            continue
        it[3] = round(it[3] + d, 2)
        moved += 1
        total += abs(d)
    return moved, total


class LanePoints:
    """Lane centre points (x, y, half width) every 2 m, to test whole footprints against the lanes: a building's
    nine sample points (corners, edge middles, centre) let a lane through between them, and buildings stood over
    streets (central Eindhoven: five over a driving lane, a camera on the lane looked into a hotel's underside)."""

    def __init__(self, points):
        from shapely.geometry import Point
        from shapely.strtree import STRtree
        self.points = [(x, y, hw) for x, y, hw in points]
        self.geoms = [Point(x, y) for x, y, _ in self.points]
        self.tree = STRtree(self.geoms) if self.geoms else None
        self.reach = max((hw for _, _, hw in self.points), default=0.0)

    @classmethod
    def of_map(cls, cmap, types, step=2.0):
        return cls([(w.transform.location.x, w.transform.location.y, w.lane_width / 2)
                    for w in cmap.generate_waypoints(step) if w.lane_type in types])

    def hits(self, poly, margin=0.1):
        """True when a lane reaches into the polygon by more than `margin` metres."""
        if self.tree is None:
            return False
        for i in self.tree.query(poly.buffer(self.reach)):
            x, y, hw = self.points[int(i)]
            if poly.distance(self.geoms[int(i)]) < hw - margin:
                return True
        return False


def rect(cx, cy, length, width, yaw_deg):
    """A shapely rectangle of length (along yaw) x width centred on (cx, cy)."""
    from shapely.geometry import Polygon
    r = math.radians(yaw_deg)
    ux, uy, vx, vy = math.cos(r), math.sin(r), -math.sin(r), math.cos(r)
    return Polygon([(cx + a * length / 2 * ux + b * width / 2 * vx, cy + a * length / 2 * uy + b * width / 2 * vy)
                    for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])


def approach(wp, step=2.0, reach=60.0):
    """The waypoint of the lane leading into the junction wp is in (the approach), or wp itself outside junctions
    or when no lane leads in within `reach` metres."""
    if wp is None or not wp.is_junction:
        return wp
    w, d = wp, 0.0
    while w.is_junction and d < reach:
        prev = w.previous(step)
        if not prev:
            return wp
        w, d = prev[0], d + step
    return w if not w.is_junction else wp


# CARLA's sign plates (0.9.16 and their 1.0 copies) are North American designs (localsigns.py): the speed plate is
# the white rectangle "SPEED LIMIT 30" (0.61 x 0.76 m, MUTCD R2-1), the give-way plate has YIELD written on it, the
# no-entry plate DO NOT ENTER, which only these countries use (elsewhere the red roundel, a plain red-bordered
# triangle, a red disc with a white bar: Vienna Convention); the stop plate is the octagon nearly everywhere, but
# Japan's is a red inverted triangle (止まれ)
from .localsigns import NO_OCTAGON, YIELD_TEXT  # noqa: E402,F401 - kept importable from here


def sign_fits(kind, iso3):
    """True when CARLA's plate for a sign kind ("stop", "give_way", "speed", "no_entry") is the design of the
    country iso3 (None: unknown). A wrong country's sign is left out rather than shown: a US YIELD or SPEED LIMIT
    plate in Eindhoven."""
    from .localsigns import stock_fits
    return stock_fits(kind, iso3)


def classify(gap, tilt=0.0, upright=True):
    """[problems] of an object whose bounding-box bottom is `gap` m above the ground under it (negative: below)
    and that leans `tilt` degrees: "floating", "sunk", "tilted"."""
    out = []
    if gap > FLOAT_M:
        out.append("floating")
    elif gap < -SINK_M:
        out.append("sunk")
    if upright and abs(tilt) > TILT_DEG:
        out.append("tilted")
    return out


def tilt_of(pitch, roll):
    """How far (degrees) a body with this pitch and roll leans from upright."""
    p, r = math.radians(pitch), math.radians(roll)
    cz = math.cos(p) * math.cos(r)       # the local z axis's vertical component
    return math.degrees(math.acos(max(-1.0, min(1.0, cz))))


def facing_error(face_yaw, traffic_yaw):
    """Degrees between a sign or signal face's normal (its yaw) and the direction it should face: towards the
    oncoming traffic, i.e. against the traffic's travel. 0 is right, 180 backwards."""
    want = (traffic_yaw + 180.0) % 360.0
    return abs((face_yaw - want + 180.0) % 360.0 - 180.0)


# --- the audit's bookkeeping (tools/placement_audit.py; pure, so it is tested without a server) --------------------

# objects of one kind closer than this (m, between their centres) are the same object twice; a sign's second post
# stands on the first (same x, y, higher z), so pairs more than 1 m apart in height are not duplicates
DUPLICATE_M = {"lamp": 1.5, "signal": 0.6, "sign_post": 0.25, "tree": 0.7, "furniture": 0.4, "shelter": 2.5,
               "parked": 1.2, "shared": 0.3, "carla_light": 0.5}


def near_pairs(points, r):
    """[(i, j, distance)] of the points [(x, y[, z])] closer than r in x, y (and within 1 m in z when given)."""
    cell = {}
    for i, p in enumerate(points):
        cell.setdefault((int(p[0] // r), int(p[1] // r)), []).append(i)
    out = []
    for (cx, cy), idx in cell.items():
        cand = [j for dx in (-1, 0, 1) for dy in (-1, 0, 1) for j in cell.get((cx + dx, cy + dy), [])]
        for i in idx:
            for j in cand:
                if j <= i:
                    continue
                a, b = points[i], points[j]
                d = math.hypot(a[0] - b[0], a[1] - b[1])
                if d < r and (len(a) < 3 or len(b) < 3 or abs(a[2] - b[2]) <= 1.0):
                    out.append((i, j, d))
    return out


def duplicates(records):
    """Marks records that duplicate another of the same audit kind (DUPLICATE_M): adds "duplicate" to the later
    one's problems with detail["duplicate_of"] and the distance. Returns the number marked."""
    by = {}
    for k, r in enumerate(records):
        if r["kind"] in DUPLICATE_M:
            by.setdefault(r["kind"], []).append(k)
    n = 0
    for kind, idx in by.items():
        pts = [(records[k]["x"], records[k]["y"], records[k].get("z", 0.0)) for k in idx]
        for i, j, d in near_pairs(pts, DUPLICATE_M[kind]):
            if d < 0.02 and abs(pts[i][2] - pts[j][2]) > 0.05:
                continue        # a taller post: a second piece standing on the first (ndwsigns.place)
            r = records[idx[j]]
            if "duplicate" not in r["problems"]:
                r["problems"].append("duplicate")
                r.setdefault("detail", {})["duplicate_of"] = [records[idx[i]].get("mesh"), round(d, 2)]
                n += 1
    return n


def overlaps(records, kinds_a, kinds_b, min_area=0.3, label="overlap"):
    """Marks records of kinds_a whose footprint (detail "foot": [(x, y)] corners) overlaps one of kinds_b by more
    than min_area m2 (a parked car in another, a car in a tree trunk's box). Returns the number marked."""
    from shapely.geometry import Polygon
    from shapely.strtree import STRtree
    a = [k for k, r in enumerate(records) if r["kind"] in kinds_a and r.get("foot")]
    b = [k for k, r in enumerate(records) if r["kind"] in kinds_b and r.get("foot")]
    if not a or not b:
        return 0
    # a pole's "base_foot" (a lamp's box is mostly its arm, high over the cars)
    polys = {k: Polygon(records[k].get("base_foot") or records[k]["foot"]).buffer(0) for k in set(a) | set(b)}
    tree = STRtree([polys[k] for k in b])
    n = 0
    for k in a:
        for i in tree.query(polys[k]):
            j = b[int(i)]
            if j == k or (records[j]["kind"] == records[k]["kind"] and j > k):
                continue        # a pair of one kind is marked once, on the later record
            area = polys[k].intersection(polys[j]).area
            if area > min_area:
                r = records[k]
                if label not in r["problems"]:
                    r["problems"].append(label)
                    r.setdefault("detail", {})[label] = [records[j]["kind"], records[j].get("mesh"), round(area, 2)]
                    n += 1
                break
    return n


def summary(records):
    """{kind: {"n": count, problem: count, ...}} and {problem: count} over all records."""
    per, tot = {}, {}
    for r in records:
        d = per.setdefault(r["kind"], {"n": 0})
        d["n"] += 1
        for p in r["problems"]:
            d[p] = d.get(p, 0) + 1
            tot[p] = tot.get(p, 0) + 1
    return per, tot


def worst(records, problem, n=5):
    """The n records with this problem that show it most (largest gap, overlap, facing error, ...)."""
    def size(r):
        det = r.get("detail") or {}
        if problem in ("floating", "sunk"):
            return abs(r.get("gap") or 0.0)
        if problem == "tilted":
            return det.get("tilt", 0.0)
        if problem in ("backwards", "facing"):
            return det.get("facing_error", 0.0)
        v = det.get(problem)
        return v[-1] if isinstance(v, list) and v and isinstance(v[-1], (int, float)) else 0.0
    return sorted((r for r in records if problem in r["problems"]), key=size, reverse=True)[:n]
