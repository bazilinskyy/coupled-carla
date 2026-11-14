"""Road signs from the Dutch national traffic sign register (NDW Verkeersborden), for maps in the Netherlands.

NDW (Nationaal Dataportaal Wegverkeer) publishes every official traffic sign the road authorities have registered
(about 20,000 in Eindhoven alone) with its RVV code (A1, B6, C2, E4, ...), position, the compass direction of the
traffic it is read by ("bearing", most of them rounded to N/E/S/W), whether it stands beside or above the road and
the number on it (the speed of an A1). Keyless, CC0 1.0, a GeoJSON API that takes a lon/lat box.

scenery.py places each sign on a post at its registered position (pushed off any lane it falls on, as the OSM
roads are not exactly where the real ones are), facing the traffic it governs: the direction of the nearest lane
that agrees with the register's bearing, else the bearing itself. Signs of one post (several plates at one spot
for the same traffic) are stacked, the lowest plate's bottom at 2.2 m (the Dutch rule over pavements and cycle
paths; a second post in line with the first makes the post that tall). A sign gets CARLA's own mesh where CARLA
has the same sign (stop, give way, the speed limits it has, no entry), else on CARLA 1.0 a blank plate of the
sign's shape (round, triangle, square, ...), else none. OSM's stop, give-way and speed signs within DEDUP_M of a
register sign of the same kind are left out.

speed_check() compares the SUMO net's speeds with the Wegkenmerkendatabase's maximum speeds per NWB road section
(Rijkswaterstaat, CC0, an OGC API that takes a box): `python -m coupledcarla.ndwsigns maps/<name>`.
"""
import json
import math
import re
import time
from collections import Counter, defaultdict
from datetime import date

import requests

API = "https://data.ndw.nu/api/rest/static-road-data/traffic-signs/v4/current-state"
WKD = ("https://geo.rijkswaterstaat.nl/services/ogc/gdr/maximum_snelheden_wegen/ogc/features/v1/collections/"
       "maximum_snelheden_per_wegvak/items")
DEDUP_M = 8.0          # an OSM stop/give-way/speed sign this close to a register sign of the same kind is dropped
POST_M = 0.7           # register signs this close together, for the same traffic, share one post
CLEAR_M = 2.2          # the lowest plate's bottom above the ground (RVV/CROW: 2.2 m over pavements, cycle paths)
MAX_PLATES = 3         # plates stacked on one post (more is rare and would need a taller post)
NEAR_M = 40.0          # signs with no lane of theirs in the map (a street the map leaves out, a pedestrian street)
                       # are placed only this close to a driving lane: seen from the roads; farther off, left out

# the outline of a Dutch sign by RVV code (series, number), for a blank plate where CARLA lacks the sign
_ROUND_C = set(range(1, 23)) - {3, 4, 5}


def parse(code):
    """'G07' -> ('G', 7, ''), 'G12a' -> ('G', 12, 'a'), 'A1-50' -> ('A', 1, ''), 'onbekend' -> None."""
    m = re.match(r"\s*(OB|[A-L])\s*0*(\d+)([a-z]?)", str(code or ""))
    return (m.group(1), int(m.group(2)), m.group(3)) if m else None


def shape_of(code):
    p = parse(code)
    if p is None:
        return None
    s, n, suf = p
    if s == "A":
        return "round" if n in (1, 2) else "square"
    if s == "B":
        return {1: "diamond", 2: "diamond", 6: "inv_triangle", 7: "octagon"}.get(n, "triangle")
    if s == "C":
        return "round" if n in _ROUND_C else "rect_h"
    if s == "D":
        return "round"
    if s == "E":
        return "round" if n <= 3 else "square"
    if s == "F":
        return {6: "square", 10: "octagon"}.get(n, "round" if n <= 9 else "rect_v")
    if s == "G":
        return "round" if 7 <= n <= 12 else "rect_v"
    if s == "J":
        return "triangle"
    if s == "L":
        return "square"
    if s == "OB":
        return "small"
    return "rect_h"     # H (built-up area), K (direction)


def kind_of(sign):
    """(kind, value) of a register sign: ('stop', None), ('give_way', None), ('speed', 50), ('no_entry', None),
    ('other', None); None for an unknown code ("onbekend")."""
    p = parse(sign.get("c"))
    if p is None:
        return None
    s, n, suf = p
    if (s, n) == ("B", 7):
        return "stop", None
    if (s, n) == ("B", 6):
        return "give_way", None
    if (s, n) == ("C", 2):
        return "no_entry", None
    if (s, n) == ("A", 1) and not suf:
        m = re.search(r"(\d{2,3})", str(sign.get("k") or "")) or re.search(r"A0*1\s*-\s*0*(\d{2,3})", sign["c"])
        return "speed", (int(m.group(1)) if m else None)
    return "other", None


# CARLA meshes: the same sign where CARLA has it (the speed, stop and give-way plates come from scenery.py's mesh
# set), and blank plates of each outline (CARLA 1.0's TrafficSign/Signs, grey faces); 0.9.16 has no blank plates
NO_ENTRY = {0: None, 1: "SM_DoNoEnter01"}
GENERIC = {0: {}, 1: {"round": "SM_CircleShape", "triangle": "SM_DangerSignShape",
                      "inv_triangle": "SM_InvertedTriangleShape", "diamond": "SM_RomboidShape",
                      "octagon": "SM_OctogonalShape", "square": "SM_SquareShape",
                      "rect_h": "SM_SlightRectangleShape", "rect_v": "SM_VerticalRectangle",
                      "small": "SM_SmallRectangle"}}
WIDTH = {"stop": 0.7, "give_way": 0.9, "speed": 0.6, "no_entry": 0.6,   # plate widths (m), Dutch normal sizes
         "round": 0.6, "triangle": 0.9, "inv_triangle": 0.9, "diamond": 0.85, "octagon": 0.7, "square": 0.6,
         "rect_h": 0.8, "rect_v": 0.6, "small": 0.6}
# stacking order on a post, most important on top (as the Dutch guidelines do): priority, speed, closures,
# directions, the rest
ORDER = {"B": 0, "A": 1, "C": 2, "F": 3, "D": 4, "G": 5, "E": 6, "H": 7, "J": 8, "L": 9, "K": 10, "OB": 11}


def _major(cat):
    from .scenery import major
    return major(cat.version)


def choose(sign, cat):
    """(mesh, plate width, tier) for a register sign on this catalogue: tier "real" (CARLA has the sign), "generic"
    (a blank plate of its shape) or "skipped" (mesh None; also for unknown codes)."""
    k = kind_of(sign)
    if k is None:
        return None, None, "skipped"
    kind, value = k
    ms, major = cat.m, _major(cat)
    real = None
    if kind == "stop":
        real = ms.STOP
    elif kind == "give_way":
        real = ms.GIVE_WAY
    elif kind == "speed" and value in ms.SPEEDS:
        real = ms.SPEED.format(value)       # only the posted speed: 20 or 15 get a blank round plate
    elif kind == "no_entry":
        real = NO_ENTRY.get(major)
    if real and real in cat:
        return real, WIDTH[kind], "real"
    shape = shape_of(sign["c"])
    mesh = GENERIC.get(major, {}).get(shape)
    if mesh and mesh in cat:
        return mesh, WIDTH[shape], "generic"
    return None, None, "skipped"


def coverage(signs, cat):
    """{"real": n, "generic": n, "skipped": n, "by_code": {code: {tier: n}}} for a catalogue."""
    out, by = Counter(), defaultdict(Counter)
    for s in signs:
        tier = choose(s, cat)[2]
        out[tier] += 1
        p = parse(s["c"])
        by[f"{p[0]}{p[1]}{p[2]}" if p else "unknown"][tier] += 1
    return {**{t: out[t] for t in ("real", "generic", "skipped")},
            "by_code": {c: dict(v) for c, v in sorted(by.items(), key=lambda kv: -sum(kv[1].values()))}}


# --- fetching ----------------------------------------------------------------------------------------------------

def fetch(bbox, path, ua, log=print, timeout=90):
    """{version, fetched, bbox, signs: [{id, c, lat, lon, b, p, k, z}]} for a box [s, w, n, e] in the Netherlands,
    cached in path. c: RVV code, b: bearing of the traffic it is read by (deg, 0 = north), p: placement (L beside,
    B above the road), k: the number or letters on it (the speed of an A1), z: zone code (ZB begin, ZE end).
    Raises when the service fails (the caller falls back to OSM's signs)."""
    if path.exists():
        return json.loads(path.read_text())
    s, w, n, e = bbox
    t0 = time.time()
    r = requests.get(API, params={"bbox": f"{w},{s},{e},{n}"}, headers=ua, timeout=timeout)
    r.raise_for_status()
    feats = r.json().get("features", [])
    signs = []
    for f in feats:
        p, g = f.get("properties") or {}, f.get("geometry") or {}
        if g.get("type") != "Point" or p.get("status", "PLACED") != "PLACED":
            continue
        lon, lat = g["coordinates"][:2]
        if not (s <= lat <= n and w <= lon <= e):
            continue
        signs.append({"id": f.get("id"), "c": p.get("rvvCode"), "lat": round(lat, 7), "lon": round(lon, 7),
                      "b": p.get("bearing"), "p": p.get("placement"), "k": p.get("blackCode"),
                      "z": p.get("zoneCode")})
    data = {"version": 1, "fetched": date.today().isoformat(), "bbox": list(bbox), "signs": signs}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, separators=(",", ":")))
    log(f"NDW: {len(signs)} traffic signs in the box in {time.time() - t0:.1f} s")
    return data


# --- placing them (scenery.py) -----------------------------------------------------------------------------------

def posts(signs, xy):
    """Group signs into posts: same bearing, within POST_M of the post's first sign. -> [[(sign, (x, y))]]"""
    cells, out = defaultdict(list), []
    for s in signs:
        x, y = xy(s)
        key = (round(x / POST_M), round(y / POST_M))
        best = None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for gi in cells.get((key[0] + dx, key[1] + dy), ()):
                    g = out[gi]
                    gx, gy = g[0][1]
                    if g[0][0].get("b") == s.get("b") and math.hypot(gx - x, gy - y) < POST_M:
                        best = gi
        if best is None:
            cells[key].append(len(out))
            out.append([(s, (x, y))])
        else:
            out[best].append((s, (x, y)))
    return out


class Roads:
    """The CARLA map's driving lanes around a sign: which one its traffic uses, and the roadside next to it."""

    def __init__(self, cmap, carla, lefthand=False):
        self.cmap, self.carla, self.lefthand = cmap, carla, lefthand

    def _wp(self, x, y):
        c = self.carla
        return self.cmap.get_waypoint(c.Location(x=x, y=y), project_to_road=True, lane_type=c.LaneType.Driving)

    def distance(self, x, y):
        """Metres to the nearest driving lane's centre (inf: none)."""
        wp = self._wp(x, y)
        if wp is None:
            return math.inf
        loc = wp.transform.location
        return math.hypot(loc.x - x, loc.y - y)

    def governed(self, x, y, yaw, reach=20.0):
        """(traffic yaw, road z, roadside (x, y) or None) for a sign at (x, y) read by traffic heading `yaw` (CARLA
        degrees): the lane on the traffic's side of the sign whose direction is within 60 degrees of it; else the
        other direction's lanes of a road running against it, or a one-way lane running against it, as a no-entry
        sign faces; else the yaw itself. None: no road in reach. (SUMO's OpenDRIVE has each direction of a street
        as a road of its own, so the probes look for a lane of the traffic's direction first.)"""
        c, side = self.carla, (1 if self.lefthand else -1)
        r = math.radians(yaw + side * 90.0)      # from the sign towards its traffic's lane (left of it in NL)
        seen = against = None
        for d in (2.5, 5.0, 0.0, 8.0):
            wp = self._wp(x + d * math.cos(r), y + d * math.sin(r))
            if wp is None:
                continue
            loc = wp.transform.location
            if math.hypot(loc.x - x, loc.y - y) > reach:
                continue
            seen = seen or wp
            diff = abs((wp.transform.rotation.yaw - yaw + 180.0) % 360.0 - 180.0)
            if diff <= 60.0:
                return wp.transform.rotation.yaw, loc.z, self._roadside(wp, False)
            if diff >= 120.0 and against is None:
                against = wp
        if against is not None:
            wp = against
            other = wp.get_left_lane()      # the lanes of the other direction, on the same road
            while other is not None and other.lane_id * wp.lane_id > 0:
                other = other.get_left_lane()
            if other is not None and other.lane_type == c.LaneType.Driving:
                return other.transform.rotation.yaw, other.transform.location.z, self._roadside(other, False)
            return (wp.transform.rotation.yaw + 180.0) % 360.0, wp.transform.location.z, self._roadside(wp, True)
        if seen is None:
            return None
        return yaw, seen.transform.location.z, None

    def _roadside(self, wp, against):
        """0.8 m beyond the outermost driving lane on the traffic's right (left on left-hand maps); `against`: the
        traffic runs against wp's direction (so its right is wp's left)."""
        c = self.carla
        right = (not self.lefthand) != against      # walk to wp's right-hand lanes?
        edge = wp
        for _ in range(8):
            nxt = edge.get_right_lane() if right else edge.get_left_lane()
            if nxt is None or nxt.lane_id * edge.lane_id < 0 or nxt.lane_type != c.LaneType.Driving:
                break
            edge = nxt
        tf = edge.transform
        v = tf.get_right_vector()
        k = (1 if right else -1) * (edge.lane_width / 2 + 0.8)
        return tf.location.x + v.x * k, tf.location.y + v.y * k


def _free_spot(on_lane, x, y, z, away, step=0.5, reach=5.0, clear=0.4):
    """(x, y) itself when it is on no lane and `clear` m from one towards the road, else the first such point
    walking `away` (yaw) from the road, or None. The clearance keeps the plates (up to 0.45 m to either side of
    the post) from hanging far over a lane."""
    ux, uy = math.cos(math.radians(away)), math.sin(math.radians(away))
    for i in range(int(reach / step) + 1):
        px, py = x + i * step * ux, y + i * step * uy
        if not on_lane(px, py, z) and not on_lane(px - clear * ux, py - clear * uy, z):
            return px, py
    return None


def place(pl, cat, signs, xy, roads, log=print):
    """Place the register's signs (posts and plates) with the scenery Placer. Returns [(x, y, kind)] of the stop,
    give-way and speed signs placed (scenery.py drops OSM's duplicates of them) and fills pl.counts:
    sign_ndw (plates), sign_ndw_real, sign_ndw_generic, sign_ndw_posts, sign_ndw_skipped (no mesh or unknown
    code), sign_ndw_unplaced (no free spot, or no bearing and no road), sign_ndw_offroad (plates placed with no
    lane of theirs: pedestrian streets, cycle paths, streets the map leaves out), sign_ndw_far (left out: more
    than NEAR_M from any driving lane)."""
    from .geo import heading_to_yaw
    ms = cat.m
    post_h = cat.size(ms.POST)[2]
    placed = []
    for group in posts(signs, xy):
        plates = []
        for s, _ in group:
            mesh, width, tier = choose(s, cat)
            if mesh is None:
                pl.counts["sign_ndw_skipped"] += 1
                continue
            plates.append((ORDER.get(parse(s["c"])[0], 99), s, mesh, width, tier))
        if not plates:
            continue
        plates.sort(key=lambda t: t[0])
        extra = plates[MAX_PLATES:]
        plates = plates[:MAX_PLATES]
        pl.counts["sign_ndw_skipped"] += len(extra)
        s0, (x, y) = group[0]
        heading = heading_to_yaw(s0["b"]) if s0.get("b") is not None else None
        gov = roads.governed(x, y, heading) if heading is not None else None
        if gov is None:     # no driving lane near (a pedestrian street, a cycle path): as the register says
            if heading is None:
                pl.counts["sign_ndw_unplaced"] += len(plates)
                continue
            if roads.distance(x, y) > NEAR_M:
                pl.counts["sign_ndw_far"] += len(plates)
                continue
            gov = (heading, pl.ground(x, y), None)
            pl.counts["sign_ndw_offroad"] += len(plates)
        tyaw, z, roadside = gov
        away = tyaw + (-90.0 if roads.lefthand else 90.0)      # from the road towards the traffic's kerb side
        spot = None if s0.get("p") == "B" else _free_spot(pl.on_lane, x, y, z, away)
        if spot is None and roadside is not None:       # above the road (gantry) or on a lane: beside the road
            spot = _free_spot(pl.on_lane, roadside[0], roadside[1], z, away)
        if spot is None:
            pl.counts["sign_ndw_unplaced"] += len(plates)
            continue
        px, py = spot
        face = tyaw + 90.0      # plate face (+y) towards the oncoming traffic, as scenery.py turns OSM's signs
        sizes = []
        for _, s, mesh, width, tier in plates:
            L, W, H = cat.size(mesh)
            k = width / max(L, 0.05)
            sizes.append((k, H * k))
        stack = sum(h for _, h in sizes) + 0.05 * (len(sizes) - 1)
        height = min(max(CLEAR_M + stack + 0.05, post_h), 2 * post_h)
        pl.add("sign", ms.POST, px, py, face, 1.0, z=z)
        if height > post_h:     # a taller post: a second one in line with the first, reaching up to `height`
            pl.add("sign", ms.POST, px, py, face, 1.0, z=z + height - post_h)
        top = z + height - 0.05
        r = math.radians(face + 90.0)       # plates hang 5 cm in front of the post
        fx, fy = px + 0.05 * math.cos(r), py + 0.05 * math.sin(r)
        for (_, s, mesh, width, tier), (k, h) in zip(plates, sizes):
            pl.add("sign", mesh, fx, fy, face + ms.SIGN_YAW, k, z=top - h)
            top -= h + 0.05
            pl.counts["sign_ndw"] += 1
            pl.counts[f"sign_ndw_{tier}"] += 1
            kind = kind_of(s)[0]
            if kind in ("stop", "give_way", "speed"):
                placed.append((px, py, kind))
        pl.counts["sign_ndw_posts"] += 1
    return placed


def osm_kind(mesh, ms):
    """The kind (stop, give_way, speed) of an OSM sign scenery.py chose `mesh` for."""
    if mesh == ms.STOP:
        return "stop"
    if mesh == ms.GIVE_WAY:
        return "give_way"
    return "speed"


def near_same(placed, x, y, kind, r=DEDUP_M):
    return any(k == kind and math.hypot(px - x, py - y) < r for px, py, k in placed)


# --- speed limits per road section (Wegkenmerkendatabase) --------------------------------------------------------

def wkd_speeds(bbox, path, ua, log=print, timeout=90):
    """[{id, v, d, g}] for the NWB road sections in a box: v the maximum speed (km/h; None: not applicable,
    e.g. cycle paths, or unknown), d the direction it holds in (B both, H with, T against the geometry), g the
    line [[lon, lat], ...]. Cached in path."""
    if path.exists():
        return json.loads(path.read_text())
    s, w, n, e = bbox
    url, params, out = WKD, {"f": "json", "bbox": f"{w},{s},{e},{n}", "limit": 1000}, []
    while url:
        r = requests.get(url, params=params, headers=ua, timeout=timeout)
        r.raise_for_status()
        page = r.json()
        for f in page.get("features", []):
            p, g = f.get("properties") or {}, f.get("geometry") or {}
            if g.get("type") != "LineString":
                continue
            v = str(p.get("maxshd") or "")
            out.append({"id": p.get("wvk_id"), "v": int(v) if v.isdigit() else None, "d": p.get("kenm_richt"),
                        "g": [[round(a, 7), round(b, 7)] for a, b in g["coordinates"]]})
        url = next((l["href"] for l in page.get("links", []) if l.get("rel") == "next"), None)
        params = None
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, separators=(",", ":")))
    log(f"WKD: {len(out)} road section speeds")
    return out


def _bearing(ax, ay, bx, by):
    return math.degrees(math.atan2(by - ay, bx - ax))


def speed_check(net, segs, reach=6.0):
    """Compare a SUMO net's edge speeds with the WKD speeds: each edge open to cars is matched, at its middle, to
    the nearest road section within `reach` m that runs along it (within 30 degrees) and holds in its direction.
    -> {"edges", "matched", "agree", "disagree", "pairs": Counter((net km/h, wkd km/h))}"""
    from shapely.geometry import LineString, Point
    from shapely.strtree import STRtree
    lines, info = [], []
    for sg in segs:
        if sg["v"] is None or len(sg["g"]) < 2:
            continue
        pts = [net.convertLonLat2XY(lon, lat) for lon, lat in sg["g"]]
        lines.append(LineString(pts))
        info.append(sg)
    tree = STRtree(lines) if lines else None
    res = {"edges": 0, "matched": 0, "agree": 0, "disagree": 0, "pairs": Counter()}
    for edge in net.getEdges():
        if edge.getFunction() == "internal" or not edge.allows("passenger"):
            continue
        shape = edge.getShape()
        if len(shape) < 2:
            continue
        res["edges"] += 1
        el = LineString(shape)
        mid = el.interpolate(0.5, normalized=True)
        a, b = el.interpolate(max(el.length / 2 - 2, 0)), el.interpolate(min(el.length / 2 + 2, el.length))
        ebr = _bearing(a.x, a.y, b.x, b.y)
        best = None
        for i in (tree.query(mid.buffer(reach)) if tree is not None else ()):
            ln = lines[i]
            dist = ln.distance(mid)
            if dist > reach:
                continue
            t = ln.project(mid)
            p, q = ln.interpolate(max(t - 2, 0)), ln.interpolate(min(t + 2, ln.length))
            diff = (ebr - _bearing(p.x, p.y, q.x, q.y) + 180) % 360 - 180
            d = info[i]["d"]
            ok = abs(diff) <= 30 and d in ("B", "H", None) or abs(diff) >= 150 and d in ("B", "T", None)
            if ok and (best is None or dist < best[0]):
                best = (dist, info[i]["v"])
        if best is None:
            continue
        res["matched"] += 1
        mine = round(edge.getSpeed() * 3.6)
        if abs(mine - best[1]) <= 2:
            res["agree"] += 1
        else:
            res["disagree"] += 1
            res["pairs"][(mine, best[1])] += 1
    return res


def main(argv=None):
    """python -m coupledcarla.ndwsigns maps/<name>: the register's signs in the map, their CARLA coverage for both
    catalogues, and the SUMO net's speeds against the WKD's."""
    import argparse
    from pathlib import Path
    from . import realworld as rw
    from .scenery import Catalog
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument("mapdir")
    ap.add_argument("--no-speeds", action="store_true")
    args = ap.parse_args(argv)
    mdir = Path(args.mapdir)
    meta = json.loads((mdir / "meta.json").read_text())
    data = fetch(meta["bbox"], mdir / "real" / "ndw_signs.json", rw.UA)
    signs = data["signs"]
    print(f"{len(signs)} register signs in {mdir.name}")
    for v in ("0.9.16", "1.0"):
        cov = coverage(signs, Catalog(v))
        print(f"CARLA {v}: {cov['real']} real, {cov['generic']} generic, {cov['skipped']} skipped")
        print("  " + ", ".join(f"{c} {sum(t.values())} ({'/'.join(f'{k[0]}{n}' for k, n in sorted(t.items()))})"
                               for c, t in cov["by_code"].items()))
    if not args.no_speeds:
        import sumolib
        segs = wkd_speeds(meta["bbox"], mdir / "real" / "wkd_speeds.json", rw.UA)
        net = sumolib.net.readNet(str(mdir / "map.net.xml"), withInternal=False)
        res = speed_check(net, segs)
        print(f"speeds: {res['edges']} car edges, {res['matched']} matched to a WKD section, {res['agree']} agree, "
              f"{res['disagree']} disagree")
        print("  (net km/h, WKD km/h): " + ", ".join(f"{a}->{b} {n}" for (a, b), n in res["pairs"].most_common(12)))


if __name__ == "__main__":
    main()
