"""Big screens and vertical shop signs on a twin's facades (realism: what defines a place like the Shibuya crossing).

Three kinds of item, all of kind "facade" (tools/snap_scenery.py moves them with the ground of their area, never onto
it), generic content only (tools/street_props.py: colour, shapes and common words, no brand, logo or real ad):

- mapped screens: OSM advertising=screen on the facade nearest them (within FACADE_REACH m); width from the tags,
  else SCREEN_W. OSM maps few (2026-11-25: none in the Shibuya, TU/e, Eindhoven, Salamanca and NYC twins).
- screens at places known for them (data/screen_places.json: the Shibuya scramble crossing, Times Square): on the
  tall walls around the place's centre that face it and look over open space (VIEW of 25 m and more), one a
  building, at most the place's number, tall and wide walls first, sized from the facade (6-18 m wide, 16:9; 9:16
  on a narrow tall facade), in the upper part of the building. (Finding such places from the data alone did not
  work: Shibuya's densest shops are in Center Gai, its densest OpenDRIVE crosswalks 190 m east of the crossing.)
- vertical shop signs (tategaki), Japanese twins: one per named shop, cafe, bar or restaurant, on the nearest facade
  (within SIGN_REACH m) at the floor OSM gives (level), sticking out of the wall, a generic word for its kind
  (SIGNV_OF), at most MAX_SIGNS.

Only on a twin with measured buildings (the facades must be the level's: map.carla_level), unless
map.scenery.screens is true; false turns it off. Needs the round-3 street props (SM_CC_Any_Screen_*,
SM_CC_JP_SignV_*) in the server's content: tools/ue_street_props.py imports them.
"""
from __future__ import annotations

import json
import math
import zlib
from pathlib import Path

KIND = "facade"
SCREEN_KEYS = "ABCD"
SCREEN_W = 8.0                   # m: a mapped screen without a width
BASE_W, BASE_TALL_W = 4.0, 2.25  # the meshes' own widths (tools/street_props.screen)
FACADE_REACH = 15.0              # m from a mapped screen to the facade it hangs on
HOT_MIN_H = 18.0                 # m: walls lower than this get no big screen
VIEW_MAX = 120.0                 # m: the longest open view a facade is measured for
SIGN_REACH, SIGN_MIN_H, MAX_SIGNS, SIGNS_PER_EDGE = 12.0, 7.0, 800, 8
FLOOR_M, SIGN_H = 3.4, 3.6
SIGNV_OF = [   # (tag, value or None for any) -> the sign's word (tools/street_props.SIGNV_WORDS)
    (("cuisine", "ramen"), "ramen"), (("cuisine", "noodle"), "ramen"), (("cuisine", "sushi"), "sushi"),
    (("amenity", "bar"), "izakaya"), (("amenity", "pub"), "izakaya"), (("amenity", "karaoke_box"), "karaoke"),
    (("amenity", "cafe"), "cafe"), (("amenity", "restaurant"), "shokudo"), (("amenity", "fast_food"), "shokudo"),
    (("amenity", "pharmacy"), "kusuri"), (("shop", "chemist"), "kusuri"), (("shop", "hairdresser"), "biyo"),
    (("shop", "beauty"), "biyo"), (("shop", "books"), "shoten"), (("shop", "clothes"), "fuku"),
    (("shop", "shoes"), "fuku"), (("shop", "fashion"), "fuku"), (("shop", "boutique"), "fuku"),
]


def sign_key(tags):
    """The vertical sign's word for a POI's tags."""
    for (k, v), key in SIGNV_OF:
        val = str(tags.get(k) or "")
        if v in val.split(";"):
            return key
    return "mise"


def floor_of(tags):
    """The floor a POI is on (OSM level / addr:floor), 0 = the street; None for a basement (its sign is at the
    street anyway: 0)."""
    for k in ("level", "addr:floor"):
        raw = str(tags.get(k) or "").split(";")[0].strip()
        try:
            f = int(float(raw))
        except ValueError:
            continue
        return max(f, 0)
    return 0


def _ring(poly):
    ring = list(poly.exterior.coords)
    if poly.exterior.is_ccw is False:
        ring = ring[::-1]
    return ring


def _edges(poly):
    """[(a, b, length, (ex, ey), (nx, ny))] of a footprint, n the outward normal (as streetlife.shop_signs)."""
    ring = _ring(poly)
    out = []
    for (ax, ay), (bx, by) in zip(ring, ring[1:]):
        L = math.hypot(bx - ax, by - ay)
        if L < 1e-6:
            continue
        ex, ey = (bx - ax) / L, (by - ay) / L
        out.append(((ax, ay), (bx, by), L, (ex, ey), (ey, -ex)))
    return out


def _seg_dist(a, b, p):
    (ax, ay), (bx, by), (px, py) = a, b, p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy or 1e-9
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


def _out(cat, mesh, scale):
    """How far out of the wall the mesh's bounding-box centre is (its face is Unreal -y; Catalog.place puts that
    centre at the point given): the item hangs on the wall instead of half inside it."""
    try:
        return abs(float(cat.by_name[mesh][1]["off"][1])) * scale + 0.02
    except (AttributeError, KeyError, IndexError, TypeError):
        return 0.05


def _yaw(n):
    return math.degrees(math.atan2(n[1], n[0])) + 90.0     # the face (mesh Unreal -y) out of the wall


def _nearest_facade(blds, tree, x, y, reach, min_h=0.0, open_m=0.0):
    """(building, edge) nearest (x, y) within reach, or None. open_m > 0: only an edge whose outside, open_m m out
    from its point nearest (x, y), is in no other building: a street facade, not a party wall between two buildings
    of a block (a sign there would hang inside the neighbour: 2026-11-28, 49 of Shibuya's 800 vertical signs and a
    third of Eindhoven's fascias)."""
    from shapely.geometry import Point
    p = Point(x, y)
    best = None
    for i in tree.query(p.buffer(reach)):
        b = blds[int(i)]
        if b.get("h", 0) < min_h:
            continue
        for e in _edges(b["poly"]):
            d = _seg_dist(e[0], e[1], (x, y))
            if d <= reach and (best is None or d < best[0]) and (open_m <= 0 or _open(blds, tree, int(i), e, x, y,
                                                                                     open_m)):
                best = (d, int(i), e)
    return None if best is None else (best[1], best[2])


def inside_any(blds, tree, x, y):
    from shapely.geometry import Point
    q = Point(x, y)
    return any(blds[int(j)]["poly"].contains(q) for j in tree.query(q))


def _open(blds, tree, i, e, x, y, out_m):
    """The point out_m m outside edge e (of building i) at its nearest point to (x, y) is in no building."""
    from shapely.geometry import Point
    (ax, ay), _b, L, (ex, ey), nn = e
    t = max(0.0, min(L, (x - ax) * ex + (y - ay) * ey))
    q = Point(ax + ex * t + nn[0] * out_m, ay + ey * t + nn[1] * out_m)
    return not any(blds[int(j)]["poly"].contains(q) for j in tree.query(q))


def _tree(blds):
    from shapely.strtree import STRtree
    return STRtree([b["poly"] for b in blds])


def place_screen(pl, cat, b, edge, along, width, key, prov, z_centre=None, loc=None):
    """One screen on edge of building b, its middle `along` m from the edge's start; returns the item or None.
    loc: the signage locale (signage.locale) whose screens show local-language content (SM_CC_JP_Screen_A, ...),
    when the server has them; else the generic ones."""
    (ax, ay), _b, L, (ex, ey), n = edge
    h = float(b.get("h") or 0.0)
    tall = width < 6.0 and h > 30.0
    kind = "ScreenTall" if tall else "Screen"
    mesh = f"SM_CC_{loc}_{kind}_{key}" if loc and f"SM_CC_{loc}_{kind}_{key}" in cat else f"SM_CC_Any_{kind}_{key}"
    if mesh not in cat:
        return None
    base = BASE_TALL_W if tall else BASE_W
    width = min(width, max(L - 1.0, 1.0))
    scale = width / base
    hs = width * (16 / 9 if tall else 9 / 16)
    t = max(width / 2 + 0.5, min(L - width / 2 - 0.5, along))
    o = _out(cat, mesh, scale)
    x, y = ax + ex * t + n[0] * o, ay + ey * t + n[1] * o
    g = pl.ground(x, y)
    if z_centre is None:
        lo, hi = 6.0 + hs / 2, h - 1.5 - hs / 2
        if hi < lo:
            return None
        z_centre = max(lo, min(hi, 0.55 * h))
    b["has_screen"] = True        # (signage.py puts no billboard on this building)
    return pl.add(KIND, mesh, x, y, _yaw(n), scale, z=g + z_centre - hs / 2, prov=prov)


def mapped_screens(pl, cat, blds, tree, elements, geo, loc=None):
    """OSM screens and lit wall billboards on their facades. Returns the count."""
    n = 0
    for el in elements:
        t = el.get("tags") or {}
        if t.get("advertising") != "screen":      # (billboards are scenery.py's furniture already)
            continue
        c = el if "lat" in el else el.get("center") or {}
        if "lat" not in c:
            continue
        x, y = geo.to_carla(c["lat"], c["lon"])
        got = _nearest_facade(blds, tree, x, y, FACADE_REACH)
        if got is None:
            continue
        i, edge = got
        try:
            w = float(str(t.get("width", "")).split()[0])
        except (ValueError, IndexError):
            w = SCREEN_W
        along = ((x - edge[0][0]) * edge[3][0] + (y - edge[0][1]) * edge[3][1])
        key = SCREEN_KEYS[zlib.crc32(str(el.get("id")).encode()) % len(SCREEN_KEYS)]
        n += place_screen(pl, cat, blds[i], edge, along, w, key, ("register", "osm"), loc=loc) is not None
    return n


PLACES = Path(__file__).resolve().parent / "data" / "screen_places.json"


def places(path=PLACES):
    try:
        return json.loads(Path(path).read_text()).get("places", [])
    except (OSError, ValueError):
        return []


def place_screens(pl, cat, blds, tree, cx, cy, r, most, view_min=25.0, loc=None):
    """Screens on the tall walls around (cx, cy) that face it (within r m, VIEW: an open view of view_min m and
    more), one a building, tall and wide walls with a wide view first. Returns the count."""
    from shapely.geometry import Point
    best = {}
    for i in tree.query(Point(cx, cy).buffer(r)):
        b = blds[int(i)]
        h = float(b.get("h") or 0.0)
        if h < HOT_MIN_H:
            continue
        for e in _edges(b["poly"]):
            (ax, ay), (bx, by), L, ed, nn = e
            if L < 8.0:
                continue
            mx, my = (ax + bx) / 2, (ay + by) / 2
            vx, vy = cx - mx, cy - my
            d = math.hypot(vx, vy)
            if d > r or (nn[0] * vx + nn[1] * vy) / max(d, 1e-6) < 0.5:
                continue                        # not facing the place
            view = view_m(blds, tree, int(i), mx, my, nn)
            if view < view_min:
                continue
            score = min(h, 60.0) * min(L, 40.0) * min(view, 100.0) / max(d, 20.0)
            if score > best.get(int(i), (0.0,))[0]:
                best[int(i)] = (score, e, h, L)
    placed, count = [], 0
    for i, (score, e, h, L) in sorted(best.items(), key=lambda kv: -kv[1][0]):
        if count >= most:
            break
        (ax, ay), _b, _L, ed, nn = e
        mx, my = ax + ed[0] * L / 2, ay + ed[1] * L / 2
        if any(math.hypot(mx - px, my - py) < 20.0 for px, py in placed):
            continue
        tall = L < 14.0 and h > 30.0
        w = min(5.9, max(4.0, 0.6 * L)) if tall else max(6.0, min(18.0, 0.55 * L))
        if place_screen(pl, cat, blds[i], e, L / 2, w, SCREEN_KEYS[count % len(SCREEN_KEYS)], ("inferred", "screens"),
                        loc=loc):
            placed.append((mx, my))
            count += 1
    return count


def view_m(blds, tree, i, mx, my, nn):
    """Open view in metres from a facade's midpoint along its outward normal to the next building (VIEW_MAX if none)."""
    from shapely.geometry import LineString, Point
    ray = LineString([(mx + nn[0] * 0.5, my + nn[1] * 0.5), (mx + nn[0] * VIEW_MAX, my + nn[1] * VIEW_MAX)])
    best = VIEW_MAX
    for j in tree.query(ray):
        if int(j) == i:
            continue
        hit = blds[int(j)]["poly"].intersection(ray)
        if not hit.is_empty:
            best = min(best, Point(mx, my).distance(hit))
    return best


def vertical_signs(pl, cat, blds, tree, pts, used=None):
    """A vertical sign per named shop / cafe / bar / restaurant on its facade. Returns the count. used: edge key ->
    [positions along it] of blade signs already there (shared with signage.tenant_signs)."""
    used, n = ({} if used is None else used), 0
    for x, y, kind, tags in pts:
        if n >= MAX_SIGNS:
            break
        if kind not in ("shop", "cafe") or not tags.get("name"):
            continue
        mesh = f"SM_CC_JP_SignV_{sign_key(tags)}"
        if mesh not in cat:
            continue
        got = _nearest_facade(blds, tree, x, y, SIGN_REACH, SIGN_MIN_H, open_m=1.0)
        if got is None:
            continue
        i, edge = got
        (ax, ay), _b, L, (ex, ey), nn = edge
        if L < 2.0:
            continue
        t = max(0.5, min(L - 0.5, (x - ax) * ex + (y - ay) * ey))
        key = (i, round(ax, 1), round(ay, 1))
        taken = used.setdefault(key, [])
        if len(taken) >= SIGNS_PER_EDGE:
            continue
        for _ in range(SIGNS_PER_EDGE):          # step along the facade past the signs already there
            if all(abs(t - u) >= 1.2 for u in taken):
                break
            t = t + 1.2 if t + 1.2 <= L - 0.5 else max(0.5, t - 2.4)
        if any(abs(t - u) < 1.2 for u in taken):
            continue
        h = float(blds[i].get("h") or 0.0)
        centre = 3.0 + SIGN_H / 2 + FLOOR_M * floor_of(tags)
        centre = min(centre, h - 0.8 - SIGN_H / 2)
        if centre < 2.6 + SIGN_H / 2:
            continue
        o = _out(cat, mesh, 1.0)
        sx, sy = ax + ex * t + nn[0] * o, ay + ey * t + nn[1] * o
        if inside_any(blds, tree, sx, sy):
            continue
        pl.add(KIND, mesh, sx, sy, _yaw(nn), 1.0, z=pl.ground(sx, sy) + centre - SIGN_H / 2,
               prov=("inferred", "screens"))
        taken.append(t)
        n += 1
    return n


def osm_screens(mdir):
    """The mapped screens and billboards from the map's cached OSM scenery (real/osm_scenery.json)."""
    p = Path(mdir) / "real" / "osm_scenery.json"
    try:
        data = json.loads(p.read_text())
    except (OSError, ValueError):
        return []
    return [e for e in data.get("elements", []) if (e.get("tags") or {}).get("advertising") == "screen"]


def add_to_scenery(pl, cat, blds, pts, region, mc, sc, mdir, geo, log=print, loc=None, used=None):
    """streetlife.add_to_scenery calls this. Returns the log phrases. loc: the signage locale (local-language screen
    content); used: the blade signs' spacing table, shared with signage.py."""
    on = sc.get("screens", "auto")
    if on is False or not blds or not (on is True or mc.get("carla_level")):
        return []
    tree = _tree(blds)
    said = []
    n = mapped_screens(pl, cat, blds, tree, osm_screens(mdir), geo, loc)
    if n:
        pl.counts["screens_mapped"] = n
        said.append(f"{n} mapped screens")
    s_, w_, n_, e_ = mc.get("bbox") or (0, 0, 0, 0)
    for p in places():
        if not (s_ <= p["lat"] <= n_ and w_ <= p["lon"] <= e_):
            continue
        cx, cy = geo.to_carla(p["lat"], p["lon"])
        n = place_screens(pl, cat, blds, tree, cx, cy, float(p.get("r", 90)), int(p.get("screens", 6)), loc=loc)
        if n:
            pl.counts["screens_place"] = pl.counts.get("screens_place", 0) + n
            said.append(f"{n} screens around {p['name']}")
    if region == "JP":
        n = vertical_signs(pl, cat, blds, tree, pts, used)
        if n:
            pl.counts["signs_vertical"] = n
            said.append(f"{n} vertical shop signs")
    return said
