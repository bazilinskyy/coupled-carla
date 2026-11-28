"""Generated shop signage and advertising on a twin's streets, in the place's own language and style (CARLA 1.0).

Where things go is real; what they show is generated. Positions come from open data: every OSM shop, cafe, bar and
restaurant (streetlife.pois) gets its fascia on the facade nearest it, at its floor (OSM level); OSM advertising
features keep their mapped positions (streetlook.Chooser.furniture: columns, poster cases, billboards); billboards and
screens go on the tall walls of dense commercial blocks and around the places known for them
(data/screen_places.json). Everything they show is made by tools/signage_art.py: invented shop names, brands,
products, films and slogans, procedural layouts, open fonts (Noto CJK JP, DejaVu, STIX). No real advertisement,
brand, logo or trademark is used, also not from openly licensed photos: brand imagery in a public repository and in
published videos is a legal risk, so real photos served only as style references (README "Generated signage").

Per locale (LOCALE_OF, from the country):
- JP (Shibuya): a lit signboard (kanban) over every shopfront, at its floor for upstairs shops (a signage band on
  the lower floors); a multi-tenant vertical sign on the corner of blocks with several shops or upstairs ones;
  cloth banners (nobori) in front of restaurants and bars, lit stand signs for upstairs shops; wall billboards and
  rooftop billboards on dense blocks; screens with Japanese copy (screens.py). The single-word vertical signs stay
  screens.vertical_signs'.
- NL, ES, DE (German-speaking Switzerland), FI (Finnish over Swedish), EN (US): a fascia board on every shop with
  its trade and an invented name; an A-board at some cafes and restaurants; EN also billboards on Times Square's
  tall walls. European old towns stay restrained: no wall or roof billboards.
Only on twins (map.carla_level: the facades are the level's measured walls), unless map.scenery.signage is true;
false turns it off. Needs the round-4 street props (tools/street_props.py + tools/ue_street_props.py); a server
without them places nothing of this (and NL keeps the older generic ShopSign boards).
"""
from __future__ import annotations

import math
import zlib

from . import screens

LOCALE_OF = {"JPN": "JP", "NLD": "NL", "BEL": "NL", "ESP": "ES", "CHE": "DE", "DEU": "DE", "AUT": "DE", "LIE": "DE",
             "FIN": "FI", "USA": "EN", "GBR": "EN", "IRL": "EN", "CAN": "EN", "AUS": "EN", "NZL": "EN"}
KIND_OF = [   # (OSM key, value) -> trade (tools/signage_art FASCIA / KANBAN keys); the first match wins
    (("cuisine", "ramen"), "ramen"), (("cuisine", "noodle"), "ramen"), (("cuisine", "sushi"), "sushi"),
    (("amenity", "karaoke_box"), "karaoke"), (("amenity", "bar"), "bar"), (("amenity", "pub"), "bar"),
    (("amenity", "cafe"), "cafe"), (("amenity", "ice_cream"), "cafe"), (("amenity", "restaurant"), "restaurant"),
    (("amenity", "fast_food"), "fastfood"), (("amenity", "pharmacy"), "pharmacy"), (("shop", "chemist"), "pharmacy"),
    (("shop", "hairdresser"), "hair"), (("shop", "beauty"), "hair"), (("shop", "clothes"), "fashion"),
    (("shop", "shoes"), "fashion"), (("shop", "fashion"), "fashion"), (("shop", "boutique"), "fashion"),
    (("shop", "jewelry"), "fashion"), (("shop", "books"), "books"), (("shop", "florist"), "florist"),
    (("shop", "supermarket"), "supermarket"), (("shop", "convenience"), "supermarket"),
    (("shop", "greengrocer"), "supermarket"), (("shop", "deli"), "supermarket"), (("shop", "bakery"), "bakery"),
    (("shop", "pastry"), "bakery"), (("shop", "confectionery"), "bakery"),
]
JP_KIND = {"bar": "izakaya", "fastfood": "restaurant", "bakery": "cafe", "florist": "shop"}   # trades a kanban lacks
EU_KIND = {"ramen": "restaurant", "sushi": "restaurant", "karaoke": "bar"}
EATS = {"ramen", "sushi", "izakaya", "restaurant", "cafe", "bar", "fastfood"}
FASCIA_REACH, FASCIA_MIN_H, FASCIA_GAP = 10.0, 3.5, 0.4
FASCIA_BOTTOM = {"JP": 2.6, "EN": 2.8}          # m above the pavement; others 2.9 (Dutch / European shopfronts)
TENANT_BOTTOM, TENANT_H, TENANT_MIN_H = 3.8, 7.2, 12.0
NOBORI_SHARE, STAND_SHARE, ABOARD_SHARE = 0.65, 0.3, 0.4
BILLBOARD = {"JP": {"min_h": 15.0, "dense": 8, "most": 120}, "EN": {"min_h": 20.0, "dense": 6, "most": 60}}
ROOF = {"min_h": 8.0, "max_h": 40.0, "dense": 6, "most": 60, "spacing": 30.0}
MAX_ITEMS = 6000


def locale(iso3):
    """The signage set of a country (ISO 3166 alpha-3): JP, NL, ES, DE, FI, EN, or None."""
    return LOCALE_OF.get((iso3 or "").upper())


def trade(tags):
    for (k, v), kind in KIND_OF:
        if v in str(tags.get(k) or "").split(";"):
            return kind
    return "shop"


def _u(*parts):
    """A repeatable number in [0, 1) from the parts."""
    return (zlib.crc32("|".join(str(p) for p in parts).encode()) % 100000) / 100000.0


def variants(cat, loc, family, key=None, most=12):
    """The catalogue's meshes SM_CC_<loc>_<family>[_<key>]_<n>, n = 1..most."""
    base = f"SM_CC_{loc}_{family}" + (f"_{key}" if key else "")
    return [f"{base}_{n}" for n in range(1, most + 1) if f"{base}_{n}" in cat]


def pick(names, *seed):
    return names[zlib.crc32("|".join(str(s) for s in seed).encode()) % len(names)] if names else None


def _ext(cat, mesh, axis):
    try:
        return float(cat.by_name[mesh][1]["ext"][axis])
    except (AttributeError, KeyError, IndexError, TypeError):
        return {0: 1.6, 1: 0.05, 2: 0.4}[axis]


class Density:
    """Shops and cafes per circle, on a grid."""

    def __init__(self, pts, cell=40.0):
        self.cell, self.grid = cell, {}
        for x, y, kind, _ in pts:
            if kind in ("shop", "cafe"):
                self.grid.setdefault((int(x // cell), int(y // cell)), []).append((x, y))

    def count(self, x, y, r):
        c, n = self.cell, 0
        for i in range(int((x - r) // c), int((x + r) // c) + 1):
            for j in range(int((y - r) // c), int((y + r) // c) + 1):
                n += sum(1 for px, py in self.grid.get((i, j), ()) if math.hypot(px - x, py - y) <= r)
        return n


def _edge_key(i, edge):
    return (i, round(edge[0][0], 1), round(edge[0][1], 1))      # as screens.vertical_signs keys an edge


def _free_t(taken, t, half, L, gap=FASCIA_GAP):
    """A position along an edge near t for an item half m wide that overlaps none of taken [(t, half)], or None."""
    lo, hi = half + 0.2, L - half - 0.2
    if hi < lo:
        return None
    for d in [0.0] + [s * k * 0.6 for k in range(1, 20) for s in (1, -1)]:
        c = min(hi, max(lo, t + d))
        if all(abs(c - u) >= half + h2 + gap for u, h2 in taken):
            return c
    return None


def fascias(pl, cat, blds, tree, pts, loc):
    """A fascia (JP: a lit kanban) on the facade nearest each shop, cafe, bar or restaurant, at street level (JP:
    at the shop's floor). Returns [(building, edge, t, half width, trade, floor)] of the placed ones."""
    fam = "Kanban" if loc == "JP" else "Fascia"
    remap = JP_KIND if loc == "JP" else EU_KIND
    if not variants(cat, loc, fam, "shop"):
        return []
    used, out = {}, []
    for x, y, kind, tags in pts:
        if kind not in ("shop", "cafe") or len(out) >= MAX_ITEMS:
            continue
        tr = trade(tags)
        tr = remap.get(tr, tr)
        names = variants(cat, loc, fam, tr) or variants(cat, loc, fam, "shop")
        mesh = pick(names, x, y, tags.get("name"))
        got = screens._nearest_facade(blds, tree, x, y, FASCIA_REACH, FASCIA_MIN_H, open_m=1.0)
        if got is None:
            continue
        i, edge = got
        (ax, ay), _b, L, (ex, ey), nn = edge
        half = _ext(cat, mesh, 0)
        floor = screens.floor_of(tags) if loc == "JP" else 0
        taken = used.setdefault((_edge_key(i, edge), floor), [])
        t = _free_t(taken, (x - ax) * ex + (y - ay) * ey, half, L)
        if t is None:
            continue
        h = float(blds[i].get("h") or 0.0)
        bottom = FASCIA_BOTTOM.get(loc, 2.9) + screens.FLOOR_M * floor
        if bottom + 2 * _ext(cat, mesh, 2) > h - 0.3:
            continue
        o = screens._out(cat, mesh, 1.0)
        sx, sy = ax + ex * t + nn[0] * o, ay + ey * t + nn[1] * o
        if screens.inside_any(blds, tree, sx, sy):
            continue
        pl.add(screens.KIND, mesh, sx, sy, screens._yaw(nn), 1.0, z=pl.ground(sx, sy) + bottom,
               prov=("inferred", "signage"))
        taken.append((t, half))
        out.append((i, edge, t, half, tr, floor))
    return out


def _street_item(pl, cat, mesh, edge, t, out_m, blds=None, tree=None):
    """A free-standing pavement item (banner, stand sign, A-board) out_m m out from the wall at t along the edge,
    turned across the pavement (its faces along the street); none where that spot is inside a building."""
    (ax, ay), _b, L, (ex, ey), nn = edge
    if not (0.3 <= t <= L - 0.3):
        return None
    sx, sy = ax + ex * t + nn[0] * out_m, ay + ey * t + nn[1] * out_m
    if tree is not None:
        from shapely.geometry import Point
        q = Point(sx, sy).buffer(0.35)
        if any(blds[int(j)]["poly"].intersects(q) for j in tree.query(q)):
            return None
    return pl.add("furniture", mesh, sx, sy, screens._yaw(nn) + 90.0, 1.0, prov=("inferred", "signage"))


def pavement_signs(pl, cat, placed, loc, blds=None, tree=None):
    """JP: nobori banners at restaurants and bars, lit stand signs for upstairs shops (and some cafes); elsewhere an
    A-board at some cafes and restaurants. Returns {kind: count}."""
    got = {}
    nobori, stands = variants(cat, loc, "Nobori"), variants(cat, loc, "Stand")
    boards = variants(cat, loc, "ABoard")
    for i, edge, t, half, tr, floor in placed:
        seed = (i, round(t, 1), tr)
        side = 1 if _u("side", *seed) < 0.5 else -1
        if loc == "JP":
            if nobori and floor == 0 and tr in EATS and _u("nobori", *seed) < NOBORI_SHARE:
                for k in range(1 + (_u("two", *seed) < 0.5)):
                    if _street_item(pl, cat, pick(nobori, "n", k, *seed), edge, t + side * (half + 0.35 + 0.6 * k),
                                    0.55, blds, tree):
                        got["nobori"] = got.get("nobori", 0) + 1
            if stands and (floor >= 1 or (tr in EATS and _u("stand", *seed) < STAND_SHARE)):
                if _street_item(pl, cat, pick(stands, "s", *seed), edge, t - side * (half + 0.4), 0.6, blds, tree):
                    got["stand"] = got.get("stand", 0) + 1
        elif boards and floor == 0 and tr in EATS | {"bakery"} and _u("aboard", *seed) < ABOARD_SHARE:
            if _street_item(pl, cat, pick(boards, "a", *seed), edge, t + side * (half + 0.5), 0.8, blds, tree):
                got["aboard"] = got.get("aboard", 0) + 1
    return got


def tenant_signs(pl, cat, blds, tree, pts, used=None):
    """JP: a multi-tenant vertical sign on blocks of TENANT_MIN_H m and more that have several shops or an upstairs
    one, near the edge's end on the wall facing most of them, above the shopfronts. `used` (edge key -> [t]) is
    screens.vertical_signs' spacing table, shared. Returns the count."""
    names = variants(cat, "JP", "Tenant")
    if not names:
        return 0
    per = {}
    for x, y, kind, tags in pts:
        if kind not in ("shop", "cafe"):
            continue
        got = screens._nearest_facade(blds, tree, x, y, screens.SIGN_REACH, TENANT_MIN_H, open_m=1.0)
        if got is not None:
            per.setdefault(got[0], []).append((got[1], screens.floor_of(tags)))
    used = {} if used is None else used
    n = 0
    for i, lst in sorted(per.items()):
        if len(lst) < 2 and not any(f >= 1 for _, f in lst):
            continue
        h = float(blds[i].get("h") or 0.0)
        edges = {}
        for e, _f in lst:
            edges.setdefault(_edge_key(i, e), [e, 0])[1] += 1
        key, (edge, _c) = max(edges.items(), key=lambda kv: (kv[1][1], kv[1][0][2]))
        (ax, ay), _b, L, (ex, ey), nn = edge
        if L < 3.0:
            continue
        t = 0.6 if _u("tenant", i) < 0.5 else L - 0.6
        if not screens._open(blds, tree, i, edge, ax + ex * t, ay + ey * t, 1.0):
            t = L - t           # the other end (the first one meets a neighbour's wall)
            if not screens._open(blds, tree, i, edge, ax + ex * t, ay + ey * t, 1.0):
                continue
        taken = used.setdefault(key, [])
        if any(abs(t - u) < 1.2 for u in taken):
            continue
        scale = min(1.3, (h - 0.3 - TENANT_BOTTOM) / TENANT_H, max(0.7, (h - 4.6) / TENANT_H))
        if scale < 0.6:
            continue
        mesh = pick(names, i)
        o = screens._out(cat, mesh, scale)
        sx, sy = ax + ex * t + nn[0] * o, ay + ey * t + nn[1] * o
        if screens.inside_any(blds, tree, sx, sy):
            continue
        pl.add(screens.KIND, mesh, sx, sy, screens._yaw(nn), scale, z=pl.ground(sx, sy) + TENANT_BOTTOM,
               prov=("inferred", "signage"))
        taken.append(t)
        n += 1
    return n


def _wall_candidates(blds, tree, dens, min_h, dense, view_min=15.0, centre=None, r=None):
    """[(score, building, edge, view)]: walls of tall buildings on dense streets (or around a centre) with an open
    view, best first."""
    from shapely.geometry import Point
    idx = range(len(blds)) if centre is None else [int(i) for i in tree.query(Point(*centre).buffer(r))]
    out = []
    for i in idx:
        b = blds[i]
        h = float(b.get("h") or 0.0)
        if h < min_h or b.get("has_screen"):
            continue
        for e in screens._edges(b["poly"]):
            (ax, ay), (bx, by), L, ed, nn = e
            if L < 10.0:
                continue
            mx, my = (ax + bx) / 2, (ay + by) / 2
            if centre is not None:
                vx, vy = centre[0] - mx, centre[1] - my
                d = math.hypot(vx, vy)
                if d > r or (nn[0] * vx + nn[1] * vy) / max(d, 1e-6) < 0.3:
                    continue
            elif dens.count(mx, my, 40.0) < dense:
                continue
            view = screens.view_m(blds, tree, i, mx, my, nn)
            if view < view_min:
                continue
            out.append((min(h, 60.0) * min(L, 40.0) * min(view, 80.0), i, e, view))
    return sorted(out, key=lambda c: -c[0])


def wall_billboards(pl, cat, blds, tree, cands, loc, most, placed=None):
    """Billboards (16:9, or 9:16 on narrow tall walls) high on the candidate walls, one a building, 25 m apart.
    Returns the count."""
    wide, tall = variants(cat, loc, "Billboard"), variants(cat, loc, "BillboardTall")
    if not wide:
        return 0
    placed = [] if placed is None else placed
    done, n = set(), 0
    for score, i, e, view in cands:
        if n >= most:
            break
        if i in done:
            continue
        (ax, ay), _b, L, ed, nn = e
        mx, my = ax + ed[0] * L / 2, ay + ed[1] * L / 2
        if any(math.hypot(mx - px, my - py) < 25.0 for px, py in placed):
            continue
        h = float(blds[i].get("h") or 0.0)
        narrow = L < 14.0 and h > 30.0 and tall
        mesh = pick(tall if narrow else wide, i, loc)
        w0, h0 = 2 * _ext(cat, mesh, 0), 2 * _ext(cat, mesh, 2)
        width = max(3.5, min(7.0, 0.5 * L)) if narrow else max(6.0, min(14.0, 0.5 * L))
        scale = width / w0
        hs = h0 * scale
        top = h - 1.0
        bottom = min(top - hs, max(8.0, 0.72 * h - hs / 2))
        if bottom < 6.0:
            continue
        o = screens._out(cat, mesh, scale)
        sx, sy = mx + nn[0] * o, my + nn[1] * o
        if screens.inside_any(blds, tree, sx, sy):
            continue
        pl.add(screens.KIND, mesh, sx, sy, screens._yaw(nn), scale, z=pl.ground(sx, sy) + bottom,
               prov=("inferred", "signage"))
        done.add(i)
        placed.append((mx, my))
        n += 1
    return n


def roof_billboards(pl, cat, blds, tree, dens, loc="JP"):
    """Rooftop billboards on mid-rise blocks of dense streets, at the edge facing the widest view, set back on the
    roof. Returns the count."""
    names = variants(cat, loc, "RoofBillboard")
    if not names:
        return 0
    placed, n = [], 0
    cands = []
    for i, b in enumerate(blds):
        h = float(b.get("h") or 0.0)
        if not (ROOF["min_h"] <= h <= ROOF["max_h"]) or b["poly"].area < 80.0 or b.get("has_screen"):
            continue
        best = None
        for e in screens._edges(b["poly"]):
            (ax, ay), (bx, by), L, ed, nn = e
            if L < 10.0:
                continue
            mx, my = (ax + bx) / 2, (ay + by) / 2
            if dens.count(mx, my, 40.0) < ROOF["dense"]:
                continue
            view = screens.view_m(blds, tree, i, mx, my, nn)
            if view >= 20.0 and (best is None or view * L > best[0]):
                best = (view * L, e)
        if best is not None and _u("roof", i) < 0.35:
            cands.append((best[0], i, best[1]))
    for score, i, e in sorted(cands, key=lambda c: -c[0]):
        if n >= ROOF["most"]:
            break
        (ax, ay), _b, L, ed, nn = e
        mx, my = ax + ed[0] * L / 2, ay + ed[1] * L / 2
        if any(math.hypot(mx - px, my - py) < ROOF["spacing"] for px, py in placed):
            continue
        mesh = pick(names, i)
        scale = min(1.0, (L - 1.0) / (2 * _ext(cat, mesh, 0)))
        front = 0.4 + 0.05
        # the bounding box's centre: the poster's plane `front` m inside the wall, the frame behind it
        depth = 2 * _ext(cat, mesh, 1) * scale
        c = front + depth / 2
        sx, sy = mx - nn[0] * c, my - nn[1] * c
        from shapely.geometry import Point
        if not blds[i]["poly"].buffer(0.05).contains(Point(mx - nn[0] * (front + depth), my - nn[1] * (front + depth))):
            continue        # the frame would stand off the roof's back
        h = float(blds[i].get("h") or 0.0)
        pl.add(screens.KIND, mesh, sx, sy, screens._yaw(nn), scale, z=pl.ground(mx + nn[0], my + nn[1]) + h,
               prov=("inferred", "signage"))
        placed.append((mx, my))
        n += 1
    return n


def add_to_scenery(pl, cat, blds, pts, iso3, mc, sc, geo=None, used=None, log=print):
    """streetlife.add_to_scenery calls this after the screens (whose walls get no billboard; `used` is their
    vertical signs' spacing table). Returns (log phrases, whether fascias were placed: NL's older generic shop
    signs are then left out)."""
    on = sc.get("signage", "auto")
    loc = locale(iso3)
    if on is False or loc is None or not blds or not (on is True or mc.get("carla_level")):
        return [], False
    tree = screens._tree(blds)
    dens = Density(pts)
    said = []
    placed = fascias(pl, cat, blds, tree, pts, loc)
    if placed:
        pl.counts["signage_fascia"] = len(placed)
        said.append(f"{len(placed)} {'lit signboards' if loc == 'JP' else 'shop fascias'} ({loc}, generated)")
    for k, v in sorted(pavement_signs(pl, cat, placed, loc, blds, tree).items()):
        pl.counts[f"signage_{k}"] = v
        said.append(f"{v} {k}")
    if loc == "JP":
        n = tenant_signs(pl, cat, blds, tree, pts, used)
        if n:
            pl.counts["signage_tenant"] = n
            said.append(f"{n} multi-tenant signs")
    if loc in BILLBOARD:
        cfg = BILLBOARD[loc]
        s_, w_, n_, e_ = mc.get("bbox") or (0, 0, 0, 0)
        spots, n = [], 0
        for p in screens.places():      # billboards around the screen places first (Times Square, Shibuya)
            if geo is None or not p.get("billboards") or not (s_ <= p["lat"] <= n_ and w_ <= p["lon"] <= e_):
                continue
            cx, cy = geo.to_carla(p["lat"], p["lon"])
            cands = _wall_candidates(blds, tree, dens, cfg["min_h"] * 0.75, 0, 15.0, (cx, cy), float(p.get("r", 90)) * 1.6)
            n += wall_billboards(pl, cat, blds, tree, cands, loc, int(p["billboards"]), spots)
        n += wall_billboards(pl, cat, blds, tree, _wall_candidates(blds, tree, dens, cfg["min_h"], cfg["dense"]), loc,
                             cfg["most"] - n, spots)
        if n:
            pl.counts["signage_billboard"] = n
            said.append(f"{n} wall billboards")
    if loc == "JP":
        n = roof_billboards(pl, cat, blds, tree, dens)
        if n:
            pl.counts["signage_roof_billboard"] = n
            said.append(f"{n} rooftop billboards")
    return said, bool(placed)
