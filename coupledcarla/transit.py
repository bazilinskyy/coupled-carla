"""Public transport from real data: the bus lines of OpenStreetMap on the SUMO net, at their GTFS frequencies.

Map build (`build`): every OSM route relation of a bus, trolleybus, minibus or share taxi in the box is matched
onto map.net.xml (the edge under each point of its track that runs its way; gaps closed by short bus paths),
and its stops (OSM stop positions, else platforms) become SUMO bus stops on that route. Where the city publishes a GTFS timetable
(found in the Mobility Database catalogue by the box), the number of trips of each line that enter the box is
counted per hour, for a weekday, a Saturday and a Sunday. All of it goes to transit.json in the map folder.

Trial (`routes`): each line runs at its timetable's frequency for the trial's local hour (simulation.time, else
08:00 on a weekday), else at the OSM `interval` tag, else at a default; buses are already spread along the line at
t = 0, as they would be, and stop at every stop. Trams and trains are left out: CARLA has no tram, and the maps
have no rails.
"""
import csv
import io
import json
import math
import random
import re
import time
import unicodedata
import zipfile
from datetime import date, datetime, timedelta
from pathlib import Path

import requests

from . import realworld

BUS_ROUTES = {"bus", "trolleybus", "minibus", "share_taxi", "coach"}
GTFS_BUS_TYPES = {3, 11} | set(range(700, 800)) | {200}     # bus, trolleybus, the extended bus types, coach
CATALOG = "https://files.mobilitydatabase.org/feeds_v2.csv"
# national feeds the catalogue files under the wrong place (OVapi's gtfs-nl is listed as country AT, no box)
NATIONAL = {"NL": ["mdb-1077"]}
MAX_FEED_BYTES = 1_500_000_000
MAX_FEEDS = 8
V_LINE = 6.0            # a city bus's mean speed along its line, stops included (m/s), to spread buses at t = 0


def log(msg, t0=[time.time()]):
    print(f"[transit    {time.time() - t0[0]:6.1f}s] {msg}", flush=True)


# --- OpenStreetMap lines ---------------------------------------------------------------------------------------

def osm_routes(bbox, path):
    """Route relations of road public transport in the box, with member geometry clipped to the box."""
    if path.exists():
        return json.loads(path.read_text())
    s, w, n, e = bbox
    query = f"""[out:json][timeout:180];
relation["type"="route"]["route"~"^({'|'.join(sorted(BUS_ROUTES))})$"]({s},{w},{n},{e});
out geom({s},{w},{n},{e});"""
    data = realworld.overpass_json(query, timeout=200)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))
    return data


def _chain(ways):
    """Member ways (lists of (lat, lon)) in route order -> continuous pieces, each way turned to follow the last.

    A route leaves and re-enters the box, or OSM has a gap: a piece ends wherever the next way does not start (or
    end) within 30 m of where the piece is. The first way of a piece is turned to meet the way after it."""
    ways = [list(w) for w in ways]
    pieces, cur = [], []
    for i, pts in enumerate(ways):
        if len(pts) < 2:
            if cur:
                pieces.append(cur)
            cur = []
            continue
        if cur:
            d0, d1 = _m(cur[-1], pts[0]), _m(cur[-1], pts[-1])
            if min(d0, d1) <= 30:
                cur.extend(pts[1:] if d0 <= d1 else pts[::-1][1:])
                continue
            pieces.append(cur)
        nxt = ways[i + 1] if i + 1 < len(ways) and len(ways[i + 1]) >= 2 else None
        if nxt and min(_m(pts[0], nxt[0]), _m(pts[0], nxt[-1])) < min(_m(pts[-1], nxt[0]), _m(pts[-1], nxt[-1])):
            pts = pts[::-1]
        cur = list(pts)
    if cur:
        pieces.append(cur)
    return pieces


def _m(a, b):
    """Metres between two (lat, lon)."""
    k = math.cos(math.radians((a[0] + b[0]) / 2))
    return 111_320 * math.hypot(a[0] - b[0], (a[1] - b[1]) * k)


def _length(pts):
    return sum(_m(a, b) for a, b in zip(pts, pts[1:]))


def parse_relations(data):
    """Overpass JSON -> lines: tags, track pieces (lists of (lat, lon)) and stops in route order."""
    lines = []
    for rel in data.get("elements", []):
        if rel.get("type") != "relation":
            continue
        tags = rel.get("tags", {})
        ways, stops, platforms = [], [], []
        for mem in rel.get("members", []):
            role = mem.get("role", "")
            geom = mem.get("geometry")
            if mem["type"] == "way" and role in ("", "forward", "backward", "hail_and_ride"):
                run = []          # points outside the box come back as null: the way leaves the box there
                for p in geom or []:
                    if p is None:
                        if run:
                            ways += [run, []]
                        run = []
                    else:
                        run.append((p["lat"], p["lon"]))
                ways.append(run)
            elif role.startswith("stop") and mem["type"] == "node" and "lat" in mem:
                stops.append((mem["lat"], mem["lon"]))
            elif role.startswith("platform"):
                if mem["type"] == "node" and "lat" in mem:
                    platforms.append((mem["lat"], mem["lon"]))
                elif geom:
                    pts = [(p["lat"], p["lon"]) for p in geom if p]
                    if pts:
                        platforms.append((sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)))
        pieces = [p for p in _chain(ways) if _length(p) >= 150]
        if not pieces:
            continue
        lines.append({"osm_id": rel["id"], "ref": tags.get("ref", ""), "name": tags.get("name", ""),
                      "from": tags.get("from", ""), "to": tags.get("to", ""), "route": tags.get("route", "bus"),
                      "operator": tags.get("operator", ""), "colour": tags.get("colour", ""),
                      "interval_min": _interval(tags.get("interval", "")),
                      "pieces": pieces, "stops": stops or platforms})
    return lines


def _interval(v):
    """OSM `interval` ("10", "00:10", "PT10M" ...) -> minutes, or None."""
    v = (v or "").strip().split(";")[0]
    try:
        if ":" in v:
            parts = [int(x) for x in v.split(":")]
            return parts[0] * 60 + parts[1] + (parts[2] / 60 if len(parts) > 2 else 0)
        if v.upper().startswith("PT"):
            v = v.upper()[2:]
            h = float(v.split("H")[0]) if "H" in v else 0
            mins = float(v.split("H")[-1].rstrip("M") or 0) if "M" in v else 0
            return h * 60 + mins or None
        return float(v) or None
    except ValueError:
        return None


# --- onto the SUMO net -----------------------------------------------------------------------------------------

def match(lines, net):
    """Each line piece -> route edges on the net and its stops (lane, position); the rest is dropped."""
    out, stops = [], {}
    for line in lines:
        for k, piece in enumerate(line["pieces"]):
            trace = _densify([net.convertLonLat2XY(lon, lat) for lat, lon in piece], 10.0)
            for edges in _connected(_nearest_edges(trace, net), net):
                if sum(e.getLength() for e in edges) < MIN_PART_M:
                    continue
                ids = [e.getID() for e in edges]
                on = []
                for lat, lon in line["stops"]:
                    st = _stop_on(net, edges, *net.convertLonLat2XY(lon, lat))
                    if st:
                        on.append(st)
                on.sort(key=lambda s: (s[0], s[2]))
                seq = []
                for idx, lane, pos, lane_len in on:
                    if seq and seq[-1][0] == idx and abs(seq[-1][2] - pos) < 30:
                        continue        # stop position and platform of the same stop
                    key = f"{lane}@{int(pos // 25)}"
                    if key not in stops:
                        a = max(0.0, min(pos - 10, lane_len - 20))
                        stops[key] = {"id": f"stop{len(stops)}", "lane": lane, "start": round(a, 1),
                                      "end": round(min(lane_len, a + 20), 1), "lines": []}
                    if line["ref"] and line["ref"] not in stops[key]["lines"]:
                        stops[key]["lines"].append(line["ref"])
                    seq.append((idx, lane, pos, stops[key]["id"]))
                out.append({k2: line[k2] for k2 in ("osm_id", "ref", "name", "from", "to", "route", "operator",
                                                   "colour", "interval_min")} |
                           {"piece": k, "edges": ids, "length_m": round(sum(e.getLength() for e in edges)),
                            "track_m": round(_length(piece)), "stops": [s[3] for s in seq]})
    return out, list(stops.values())


MIN_PART_M = 300        # shorter matched stretches of a line are left out (a bus would appear and vanish at once)


def _densify(pts, step):
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(d // step))
        out += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1)]
    return out


def _heading(shape, x, y):
    """Direction (unit vector) of the shape's segment nearest to (x, y), and the distance to it."""
    best = None
    for (ax, ay), (bx, by) in zip(shape, shape[1:]):
        dx, dy = bx - ax, by - ay
        ll = dx * dx + dy * dy
        if ll == 0:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / ll))
        d = math.hypot(ax + t * dx - x, ay + t * dy - y)
        if best is None or d < best[0]:
            n = math.sqrt(ll)
            best = (d, dx / n, dy / n)
    return best


def _nearest_edges(trace, net, radius=12.0, max_angle=45.0):
    """The edge under each trace point that runs the trace's way (the net was built from the same OSM ways, so
    the track lies on its edges, a lane's width away); consecutive repeats removed."""
    cos_max = math.cos(math.radians(max_angle))
    seq = []
    for i, (x, y) in enumerate(trace):
        a, b = trace[max(0, i - 1)], trace[min(len(trace) - 1, i + 1)]
        hx, hy = b[0] - a[0], b[1] - a[1]
        hn = math.hypot(hx, hy)
        if hn == 0:
            continue
        hx, hy = hx / hn, hy / hn
        best = None
        for e, _ in net.getNeighboringEdges(x, y, radius):
            if not any(ln.allows("bus") for ln in e.getLanes()):
                continue
            h = _heading(e.getShape(), x, y)
            if h is None:
                continue
            d, ex, ey = h
            cos = ex * hx + ey * hy
            if cos < cos_max:
                continue
            score = d + 10 * (1 - cos)
            if best is None or score < best[0]:
                best = (score, e)
        if best and (not seq or seq[-1] != best[1]):
            seq.append(best[1])
    # A-B-A flicker where two edges overlap (junctions): drop the middle one
    out = []
    for e in seq:
        if len(out) >= 2 and out[-2] == e:
            out.pop()
            continue
        out.append(e)
    return out


def _connected(seq, net):
    """Close gaps between consecutive edges with the shortest bus path (no long detours); cut where that fails.
    Returns the connected parts."""
    if not seq:
        return []
    parts, cur = [], [seq[0]]
    for e in seq[1:]:
        prev = cur[-1]
        if e == prev:
            continue
        if e in prev.getOutgoing():
            cur.append(e)
            continue
        path, cost = net.getShortestPath(prev, e, vClass="bus")
        (ax, ay), (bx, by) = prev.getShape()[-1], e.getShape()[0]
        if path and len(path) <= 8 and cost - prev.getLength() - e.getLength() <= 2 * math.hypot(bx - ax, by - ay) + 60:
            cur.extend(path[1:])
        else:
            parts.append(cur)
            cur = [e]
    parts.append(cur)
    return parts


def _stop_on(net, edges, x, y, radius=35.0):
    """(edge index, lane id, position, lane length) of the route lane nearest to (x, y) that buses may use."""
    best = None
    index = {e.getID(): i for i, e in enumerate(edges)}
    for e, _ in net.getNeighboringEdges(x, y, radius):
        i = index.get(e.getID())
        if i is None:
            continue
        lanes = [ln for ln in e.getLanes() if ln.allows("bus")]
        if not lanes:
            continue
        lane = lanes[0]     # the outermost lane a bus may use: where it stops
        pos, dist = lane.getClosestLanePosAndDist((x, y))
        if best is None or dist < best[0]:
            best = (dist, i, lane.getID(), pos, lane.getLength())
    if best is None or best[3] < 0:
        return None
    return best[1:]


# --- GTFS frequencies ------------------------------------------------------------------------------------------

def feeds_for(bbox, cache):
    """GTFS feeds of the Mobility Database whose bounding box holds the box's centre, smallest first."""
    path = cache / "mobilitydatabase_feeds_v2.csv"
    if not path.exists() or time.time() - path.stat().st_mtime > 7 * 86400:
        cache.mkdir(parents=True, exist_ok=True)
        r = requests.get(CATALOG, headers=realworld.UA, timeout=120)
        r.raise_for_status()
        path.write_bytes(r.content)
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    found = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("data_type") != "gtfs" or row.get("status") not in ("active", ""):
                continue
            if row.get("redirect.id") or not row.get("urls.latest"):
                continue
            try:
                s, n = float(row["location.bounding_box.minimum_latitude"]), float(row["location.bounding_box.maximum_latitude"])
                w, e = float(row["location.bounding_box.minimum_longitude"]), float(row["location.bounding_box.maximum_longitude"])
            except (TypeError, ValueError):
                continue
            # some catalogue boxes are far too big (Sydney's feed "covers" Taipei): smallest first, and a feed only
            # counts when it has stops inside the map's box (gtfs_frequencies)
            if s <= lat <= n and w <= lon <= e:
                found.append(((n - s) * (e - w), row))
    found.sort(key=lambda x: x[0])
    rows = [r for _, r in found]
    if realworld.in_netherlands(bbox):
        with open(path, newline="", encoding="utf-8") as f:
            extra = [r for r in csv.DictReader(f) if r.get("id") in NATIONAL["NL"]]
        rows = extra + [r for r in rows if r["id"] not in NATIONAL["NL"]]
    return rows


def _fetch_feed(row, cache):
    path = cache / f"{row['id']}.zip"
    if path.exists() and time.time() - path.stat().st_mtime < 30 * 86400:
        return path
    url = row["urls.latest"]
    head = requests.head(url, headers=realworld.UA, timeout=60, allow_redirects=True)
    size = int(head.headers.get("content-length") or 0)
    if size > MAX_FEED_BYTES:
        log(f"GTFS {row['id']}: {size / 1e9:.1f} GB, skipped")
        return None
    with requests.get(url, headers=realworld.UA, timeout=600, stream=True) as r:
        r.raise_for_status()
        tmp = path.with_suffix(".part")
        with open(tmp, "wb") as f:
            for chunk in r.iter_content(1 << 20):
                f.write(chunk)
    tmp.rename(path)
    return path


def _rows(z, name):
    """Rows of a GTFS table as dicts (BOM and stray spaces in the header removed); none if the table is missing."""
    try:
        f = z.open(name)
    except KeyError:
        return
    reader = csv.reader(io.TextIOWrapper(f, encoding="utf-8-sig", newline=""))
    head = [h.strip() for h in next(reader, [])]
    for row in reader:
        yield dict(zip(head, row))


def _secs(t):
    try:
        h, m, s = (int(x) for x in t.strip().split(":"))
        return h * 3600 + m * 60 + s
    except ValueError:
        return None


def _service_dates(z):
    """service_id -> function(date) -> active, from calendar.txt and calendar_dates.txt; and the feed's span."""
    days = ["monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "sunday"]
    cal, plus, minus = {}, {}, {}
    lo, hi = None, None
    for r in _rows(z, "calendar.txt"):
        a, b = _d(r.get("start_date")), _d(r.get("end_date"))
        if a and b:
            cal[r["service_id"]] = (a, b, [r.get(d, "0") == "1" for d in days])
            lo, hi = min(lo or a, a), max(hi or b, b)
    for r in _rows(z, "calendar_dates.txt"):
        d = _d(r.get("date"))
        if not d:
            continue
        (plus if r.get("exception_type") == "1" else minus).setdefault(r["service_id"], set()).add(d)
        lo, hi = min(lo or d, d), max(hi or d, d)

    def active(sid, d):
        if d in minus.get(sid, ()):
            return False
        if d in plus.get(sid, ()):
            return True
        c = cal.get(sid)
        return bool(c and c[0] <= d <= c[1] and c[2][d.weekday()])
    return active, lo, hi


def _d(s):
    try:
        return datetime.strptime(s.strip(), "%Y%m%d").date()
    except (AttributeError, ValueError):
        return None


def _days_in(lo, hi):
    """A Tuesday, a Saturday and a Sunday inside the feed's span, as near today as possible."""
    today = date.today()
    start = min(max(today, lo or today), hi or today)
    out = {}
    for name, wd in (("weekday", 1), ("saturday", 5), ("sunday", 6)):
        d = start + timedelta(days=(wd - start.weekday()) % 7)
        if hi and d > hi:
            d -= timedelta(days=7)
        out[name] = d
    return out


def gtfs_frequencies(bbox, cache, refs):
    """For each line ref seen in OSM: trips per hour per direction that enter the box, for each day type.

    Returns ({ref: {"weekday": [24], "saturday": [24], "sunday": [24]}}, [feed ids used])."""
    s, w, n, e = bbox
    keys = {}
    for ref in refs:
        keys[_key(ref)] = ref
    for ref in refs:
        keys.setdefault(_base(ref), ref)
    result, used = {}, []
    for row in feeds_for(bbox, cache)[:MAX_FEEDS]:
        try:
            path = _fetch_feed(row, cache)
        except requests.RequestException as err:
            log(f"GTFS {row['id']}: download failed ({err})")
            continue
        if not path:
            continue
        try:
            z = zipfile.ZipFile(path)
        except zipfile.BadZipFile:
            log(f"GTFS {row['id']}: not a zip")
            continue
        inside = {r["stop_id"] for r in _rows(z, "stops.txt")
                  if _in(r.get("stop_lat"), r.get("stop_lon"), s, w, n, e)}
        if not inside:
            continue
        routes = {}
        for r in _rows(z, "routes.txt"):
            try:
                rtype = int(r.get("route_type") or 3)
            except ValueError:
                rtype = 3
            name = (r.get("route_short_name") or "").strip() or (r.get("route_long_name") or "").strip()
            ref = keys.get(_key(name)) or keys.get(_base(name))
            if rtype in GTFS_BUS_TYPES and ref:
                routes[r["route_id"]] = ref
        if not routes:
            continue
        trips = {}
        for r in _rows(z, "trips.txt"):
            if r.get("route_id") in routes:
                trips[r["trip_id"]] = (routes[r["route_id"]], r.get("service_id"), r.get("direction_id") or "0")
        first = {}      # trip -> earliest time at a stop inside the box
        for r in _rows(z, "stop_times.txt"):
            t = r.get("trip_id")
            if t in trips and r.get("stop_id") in inside:
                sec = _secs(r.get("departure_time") or r.get("arrival_time") or "")
                if sec is not None and (t not in first or sec < first[t]):
                    first[t] = sec
        if not first:
            continue
        active, lo, hi = _service_dates(z)
        days = _days_in(lo, hi)
        for ref in {v[0] for v in trips.values()}:
            if ref in result:
                continue        # a smaller (more local) feed already had it
            per = {}
            for day, d in days.items():
                counts, dirs = [0] * 24, set()
                for t, sec in first.items():
                    r_, sid, dr = trips[t]
                    if r_ == ref and active(sid, d):
                        counts[(sec // 3600) % 24] += 1
                        dirs.add(dr)
                per[day] = [round(c / max(1, len(dirs)), 2) for c in counts]
            if any(sum(v) for v in per.values()):
                result[ref] = per
        used.append({"id": row["id"], "provider": row.get("provider", ""), "license": row.get("urls.license", ""),
                     "days": {k: v.isoformat() for k, v in days.items()}})
        log(f"GTFS {row['id']} ({row.get('provider', '')}): {len(result)} of {len(refs)} lines with a timetable")
        if len(result) == len(refs):
            break
    return result, used


def _key(name):
    """A line name for matching OSM refs with GTFS names: full-width characters folded (Tokyo writes 都０１),
    spaces dropped, upper case."""
    return re.sub(r"\s+", "", unicodedata.normalize("NFKC", name or "")).upper()


def _base(name):
    """The name without a bracketed part: 都01(T01) -> 都01."""
    return re.sub(r"\(.*?\)", "", _key(name))


def _in(lat, lon, s, w, n, e):
    try:
        return s <= float(lat) <= n and w <= float(lon) <= e
    except (TypeError, ValueError):
        return False


# --- map build -------------------------------------------------------------------------------------------------

def build(out, bbox, use_gtfs=True, cache=None):
    """Lines, stops and frequencies for a built map -> out/transit.json; returns a summary."""
    import sumolib
    net = sumolib.net.readNet(str(out / "map.net.xml"), withInternal=False, withPedestrianConnections=False)
    lines = parse_relations(osm_routes(bbox, out / "real" / "osm_transit.json"))
    matched, stops = match(lines, net)
    refs = sorted({ln["ref"] for ln in matched if ln["ref"]})
    log(f"{len(lines)} OSM bus routes in the box; {len(matched)} matched onto the net ({len(refs)} line numbers), "
        f"{len(stops)} stops")
    freq, feeds = {}, []
    if use_gtfs and refs:
        try:
            freq, feeds = gtfs_frequencies(bbox, Path(cache or Path.home() / ".cache" / "coupled-carla" / "gtfs"),
                                           set(refs))
        except Exception as err:  # noqa: BLE001 - the lines still run at OSM or default intervals
            log(f"GTFS failed ({err}); lines run at OSM or default intervals")
    data = {"lines": matched, "stops": stops, "gtfs": freq, "feeds": feeds}
    (out / "transit.json").write_text(json.dumps(data, indent=1))
    return {"lines": len(matched), "refs": len(refs), "stops": len(stops), "with_timetable": len(freq),
            "feeds": [f["id"] for f in feeds]}


# --- trial -----------------------------------------------------------------------------------------------------

def headway_s(line, gtfs, day, hour, default_min):
    """Seconds between buses of a line in that hour; None when its timetable runs none then."""
    per = gtfs.get(line["ref"]) if line["ref"] else None
    if per:
        n = per[day][hour]
        return 3600.0 / n if n > 0 else None
    if line.get("interval_min"):
        return line["interval_min"] * 60.0
    return default_min * 60.0


def day_type(when):
    return "saturday" if when.weekday() == 5 else "sunday" if when.weekday() == 6 else "weekday"


def trial_time(text):
    """simulation.time -> the local datetime whose timetable is used; unset or "now": 08:00 on a weekday."""
    now = datetime.now()
    if text not in (None, "now", ""):
        text = str(text).strip()
        for fmt in ("%Y-%m-%d %H:%M", "%Y-%m-%dT%H:%M"):
            try:
                return datetime.strptime(text, fmt)
            except ValueError:
                pass
        try:
            hh, mm = (int(v) for v in text.split(":")[:2])
            return now.replace(hour=hh, minute=mm, second=0, microsecond=0)
        except ValueError:
            pass
        if text == "now":
            return now
    d = now.replace(hour=8, minute=0, second=0, microsecond=0)
    while d.weekday() >= 5:
        d += timedelta(days=1)
    return d


def routes(map_dir, work, end_s, vtype, vtype_xml, when, pc, seed, net):
    """Writes stops.add.xml and pt.rou.xml for the trial; returns (route file, additional file, summary) or None."""
    path = Path(map_dir) / "transit.json"
    if not path.exists():
        return None
    data = json.loads(path.read_text())
    if not data["lines"]:
        return None
    day, hour = day_type(when), when.hour
    rng = random.Random(seed)
    edges_len = {}
    add = ["<additional>"]
    for st in data["stops"]:
        lines = ",".join(st["lines"])
        lines_attr = f' lines="{_x(lines)}"' if lines else ""      # SUMO refuses an empty one
        add.append(f'  <busStop id="{st["id"]}" lane="{st["lane"]}" startPos="{st["start"]}" endPos="{st["end"]}"'
                   f'{lines_attr} friendlyPos="true"/>')
    add.append("</additional>")
    rou = ["<routes>", "  " + vtype_xml]
    vehicles, running = [], 0
    dwell = pc.get("dwell_s", 20)
    for li, line in enumerate(data["lines"]):
        h = headway_s(line, data["gtfs"], day, hour, pc.get("default_headway_min", 10))
        if not h:
            continue
        running += 1
        for e in line["edges"]:
            if e not in edges_len:
                edges_len[e] = net.getEdge(e).getLength()
        cum, acc = [], 0.0
        for e in line["edges"]:
            cum.append(acc)
            acc += edges_len[e]
        total = acc
        stop_edge = {st["id"]: st["lane"].rsplit("_", 1)[0] for st in data["stops"]}
        phase = rng.uniform(0, h)
        k = 0
        while True:                     # buses already on the line at t = 0, spread by the headway
            back = (k * h + (h - phase)) * V_LINE
            if back >= total - 60:
                break
            i = max(j for j, c in enumerate(cum) if c <= total - back - 1e-6) if total - back > 0 else 0
            start = total - back
            pos = max(0.0, min(start - cum[i], edges_len[line["edges"][i]] - 1))
            edges = line["edges"][i:]
            st_ids = [s for s in line["stops"] if stop_edge.get(s) in edges and not
                      (stop_edge[s] == edges[0] and _stop_pos(data, s) < pos + 5)]
            vehicles.append((0.0, f"pt{li}_{k}", edges, st_ids, pos, line))
            k += 1
        t = phase
        n = 0
        while t < end_s:                 # and the next ones entering at the start of the line
            vehicles.append((t, f"pt{li}_d{n}", line["edges"], line["stops"], 0.0, line))
            t += h
            n += 1
    vehicles.sort(key=lambda v: v[0])
    for depart, vid, edges, st_ids, pos, line in vehicles:
        attrs = f'id="{vid}" type="{vtype}" depart="{depart:.1f}" departLane="best" departPos="{pos:.1f}"'
        line_attr = _x(line["ref"] or line["name"])
        rou.append(f'  <vehicle {attrs} line="{line_attr}">')
        rou.append(f'    <route edges="{" ".join(edges)}"/>')
        for s in st_ids:
            rou.append(f'    <stop busStop="{s}" duration="{dwell}"/>')
        rou.append("  </vehicle>")
    rou.append("</routes>")
    if not vehicles:
        return None
    (work / "stops.add.xml").write_text("\n".join(add) + "\n")
    (work / "pt.rou.xml").write_text("\n".join(rou) + "\n")
    return "pt.rou.xml", "stops.add.xml", {"lines_running": running, "lines": len(data["lines"]),
                                           "buses": len(vehicles), "day": day, "hour": hour}


def _stop_pos(data, sid, _cache={}):
    key = id(data)
    if key not in _cache:
        _cache.clear()
        _cache[key] = {s["id"]: s["start"] for s in data["stops"]}
    return _cache[key].get(sid, 0.0)


def _x(s):
    return (s or "").replace("&", "&amp;").replace('"', "&quot;").replace("<", "&lt;")
