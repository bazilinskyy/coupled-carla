"""Stop lines before crosswalks: vehicles wait before a crossing at a junction, not on it.

SUMO stops a vehicle that must wait at a junction (a red light, a give-way, a turn across oncoming traffic: the nets
have no internal junctions, mapbuild.NO_INTERNAL_JUNCTIONS) with its front at the end of its lane. The nets have no
sidewalks or crossings (config.py), so a lane ends where the junction begins, and the crosswalk painted across the
approach lies on the last metres of that lane: on the Eindhoven twin clip (2026-11-28) the autopilot car stopped at
a red light on the zebra (the user: "car stops on the zebra crossing at the lights"), and the signal heads stood at
the lane's end beside the zebra. In the real street the stop line, and the signal head, are before the crossing.

Now every approach lane where a car waiting at its end would stand on a crosswalk (its front or its CAR_M body)
gets a SUMO stop line MARGIN_M before that crosswalk (and before the next one back, if the car would then stand on
that): the lane's <stopOffset> (all vehicle classes). An approach is followed back over plain lane-to-lane joins
(netconvert splits OpenDRIVE roads into SUMO edges as short as 0.2 m in front of a signal); SUMO takes an offset
longer than the lane and its vehicles then wait on the lane before. Where the car would not fit between the
previous junction and the crosswalk, it keeps waiting at the lane's end. SUMO's vehicles (cars, buses, cyclists,
the participant's SUMO autopilot, sumodriver.py) wait at the stop line, and signals.py puts the heads there and
subtracts it from next_light()'s distance (the CARLA autopilot, ai.py's scripted cars and myodriver stop for it).
It applies at every junction kind, signalised or not, on every map:

- crosswalks: a map's OpenDRIVE crosswalk objects (carla-digitaltwins twins, CARLA towns; their outline or their
  length x width box), and on maps from OSM (no crosswalk objects) every OSM crossing node (highway=crossing, or a
  crossing=* tag that is not "no") as a CROSSING_W_M wide strip across its road (CROSSING_ACROSS_M each way);
- crossings(map_dir) caches them in <map dir>/crossings.json (in OpenDRIVE x, y; CARLA = (x, -y));
- prepare() (traffic.prepare's hook) writes the trial's net with the stop lines (work/stops.net.xml), so maps need no
  rebuild; mapbuild runs it on new maps too (map.net.xml then has them);
- check() lists the stop positions inside a crosswalk, before and after: tools/stopline_check.py runs it over maps.

Mid-block zebras (no junction at the lane's end) are left to yielding.py: SUMO vehicles stop STOP_GAP_M before the
walker on it.
"""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

MARGIN_M = 1.0           # the stop line this far before the crosswalk's near edge
CAR_M = 4.5              # a waiting car's length behind its front: its body must be clear of the crosswalk too
ALONG_X = 2.2            # a lane inside a crosswalk for more than this times its width runs along it (lane_spans)
UPSTREAM_M = 30.0        # how far back an approach is followed over plain lane-to-lane joins (short SUMO edges)
MAX_OFFSET_M = 15.0      # never a stop line further back than this (a cycle crossing and a zebra: 6-9 m)
CROSSING_W_M = 3.0       # an OSM crossing node: a strip this wide along its road (Dutch zebras 2.5-4 m)
CROSSING_ACROSS_M = 9.0  # ... reaching this far each way across it (the road's lanes, cycle tracks beside it)
VERSION = 1              # crossings.json format
ROADS = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "living_street",
         "service", "road", "busway", "cycleway", "motorway_link", "trunk_link", "primary_link", "secondary_link",
         "tertiary_link"}


# --- crosswalks ----------------------------------------------------------------------------------------------------

def xodr_crosswalks(xodr):
    """[[(x, y), ...]] the outline of every crosswalk object of an OpenDRIVE (path or text), in its x, y: the
    object's cornerLocal or cornerRoad outline, else its length (local u) x width (local v) box, at (s, t) from the
    road's reference line turned by its hdg."""
    from .odroutline import Road
    text = xodr if str(xodr).lstrip().startswith("<") else Path(xodr).read_text(encoding="utf-8")
    root = ET.fromstring(text.encode("utf-8"))
    out = []
    for road in root.iter("road"):
        objs = [o for o in road.iter("object") if (o.get("type") or "").lower() == "crosswalk"]
        if not objs or road.find("planView/geometry") is None:
            continue
        rd = Road(road)

        def st_xy(s, t):
            x, y, h = rd.ref(min(max(s, 0.0), rd.length))
            return x - t * math.sin(h), y + t * math.cos(h), h
        for o in objs:
            s, t = float(o.get("s", 0)), float(o.get("t", 0))
            px, py, h = st_xy(s, t)
            a = h + float(o.get("hdg", 0) or 0)
            ux, uy, vx, vy = math.cos(a), math.sin(a), -math.sin(a), math.cos(a)
            pts = []
            for c in o.iter("cornerLocal"):
                u, v = float(c.get("u", 0)), float(c.get("v", 0))
                pts.append((px + u * ux + v * vx, py + u * uy + v * vy))
            if len(pts) < 3:
                pts = [st_xy(float(c.get("s", 0)), float(c.get("t", 0)))[:2] for c in o.iter("cornerRoad")]
            if len(pts) < 3:
                hl, hw = float(o.get("length", 0) or 0) / 2, float(o.get("width", 0) or 0) / 2
                if hl <= 0 or hw <= 0:
                    hl = hw = 2.0
                pts = [(px + u * ux + v * vx, py + u * uy + v * vy)
                       for u, v in ((-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw))]
            out.append([(round(x, 3), round(y, 3)) for x, y in pts])
    return out


def osm_crossings(osm_path, xy):
    """[[(x, y) x 4]] a strip across its road at every OSM crossing node of map.osm: CROSSING_W_M along the road (the
    direction of the road way through the node), CROSSING_ACROSS_M each way across it. xy(lat, lon) -> OpenDRIVE
    x, y (geo.net.xml's projection: signalplan._xy_of)."""
    nodes, crossing, ways = {}, set(), []
    cur, tags, refs = None, {}, []
    for ev, el in ET.iterparse(str(osm_path), events=("start", "end")):
        if ev == "start":
            if el.tag in ("node", "way"):
                cur, tags, refs = el, {}, []
            continue
        if el.tag == "tag" and cur is not None:
            tags[el.get("k")] = el.get("v")
        elif el.tag == "nd" and cur is not None:
            refs.append(el.get("ref"))
        elif el.tag == "node":
            nodes[el.get("id")] = (float(el.get("lat")), float(el.get("lon")))
            if (tags.get("highway") == "crossing" and tags.get("crossing") != "no") or \
                    tags.get("crossing") not in (None, "no"):
                crossing.add(el.get("id"))
            cur = None
            el.clear()
        elif el.tag == "way":
            if tags.get("highway") in ROADS and any(r in crossing for r in refs):
                ways.append(refs)
            cur = None
            el.clear()
    out, done = [], set()
    for refs in ways:
        for i, r in enumerate(refs):
            if r not in crossing or r in done or r not in nodes:
                continue
            a = refs[i - 1] if i > 0 else r
            b = refs[i + 1] if i + 1 < len(refs) else r
            if a == b or a not in nodes or b not in nodes:
                continue
            (x0, y0), (x1, y1), (cx, cy) = xy(*nodes[a]), xy(*nodes[b]), xy(*nodes[r])
            d = math.hypot(x1 - x0, y1 - y0)
            if d < 1e-6:
                continue
            ax, ay = (x1 - x0) / d, (y1 - y0) / d
            nx, ny = -ay, ax
            w, c = CROSSING_W_M / 2, CROSSING_ACROSS_M
            out.append([(round(cx + sa * w * ax + sn * c * nx, 3), round(cy + sa * w * ay + sn * c * ny, 3))
                        for sa, sn in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
            done.add(r)
    return out


def _source_key(paths):
    return [[p.name, p.stat().st_size, int(p.stat().st_mtime)] for p in paths if p.exists()]


def crossings(map_dir, cache=True, log=print):
    """The crosswalks of a built map in OpenDRIVE x, y: map.xodr's crosswalk objects, else (a map from OSM) its OSM
    crossing nodes. Cached in <map dir>/crossings.json while map.xodr and map.osm are unchanged."""
    mdir = Path(map_dir)
    xodr, osm, geo = mdir / "map.xodr", mdir / "map.osm", mdir / "geo.net.xml"
    key = _source_key([xodr, osm])
    path = mdir / "crossings.json"
    if cache and path.exists():
        try:
            data = json.loads(path.read_text())
            if data.get("version") == VERSION and data.get("key") == key:
                return [[tuple(p) for p in poly] for poly in data["polygons"]]
        except (OSError, ValueError, KeyError):
            pass
    polys, source = [], "none"
    if xodr.exists():
        polys, source = xodr_crosswalks(xodr), "opendrive"
    if not polys and osm.exists() and geo.exists():
        from .signalplan import _xy_of
        polys, source = osm_crossings(osm, _xy_of(geo)), "osm"
    if cache:
        try:
            path.write_text(json.dumps({"version": VERSION, "key": key, "source": source, "polygons": polys},
                                       separators=(",", ":")))
        except OSError as err:
            log(f"stop lines: crossings not cached ({err})")
    return polys


# --- the net -------------------------------------------------------------------------------------------------------

def net_lanes(net_file):
    """({lane id: {"length", "shape": [(x, y)], "edge", "from", "to", "to_type", "tls": bool, "preds", "succs"}},
    {junction id: (normal edges in, normal edges out)}, {junction id: type}) of a SUMO net's normal lanes (no
    internal, crossing or walking-area edges); preds / succs: the lanes with a connection into / out of the lane."""
    junctions, lanes, tls_lanes, conns = {}, {}, set(), []
    edge = None
    for ev, el in ET.iterparse(str(net_file), events=("start", "end")):
        if ev == "start":
            if el.tag == "edge":
                edge = el if not el.get("function") else None
            continue
        if el.tag == "lane" and edge is not None:
            shape = [tuple(float(v) for v in p.split(",")[:2]) for p in el.get("shape", "").split()]
            if len(shape) >= 2:
                lanes[el.get("id")] = {"length": float(el.get("length")), "shape": shape, "edge": edge.get("id"),
                                       "from": edge.get("from"), "to": edge.get("to"), "preds": [], "succs": []}
        elif el.tag == "edge":
            edge = None
        elif el.tag == "junction":
            junctions[el.get("id")] = el.get("type")
            el.clear()
        elif el.tag == "connection":
            a, b = f"{el.get('from')}_{el.get('fromLane')}", f"{el.get('to')}_{el.get('toLane')}"
            conns.append((a, b))
            if el.get("tl"):
                tls_lanes.add(a)
    degree = {}
    for e in {(ln["edge"], ln["from"], ln["to"]) for ln in lanes.values()}:
        degree.setdefault(e[2], [set(), set()])[0].add(e[0])
        degree.setdefault(e[1], [set(), set()])[1].add(e[0])
    for a, b in conns:
        if a in lanes and b in lanes:
            lanes[b]["preds"].append(a)
            lanes[a]["succs"].append(b)
    for lid, ln in lanes.items():
        ln["to_type"] = junctions.get(ln["to"])
        ln["tls"] = lid in tls_lanes
    nodes = {j: (len(i), len(o)) for j, (i, o) in degree.items()}
    return lanes, nodes, junctions


def net_shift(map_dir, net_file=None):
    """(dx, dy): SUMO net x, y = OpenDRIVE x, y + (dx, dy), the bridge's offset (map.net.xml's netOffset minus
    geo.net.xml's; run.py)."""
    mdir = Path(map_dir)
    mo = _location(net_file or mdir / "map.net.xml")
    go = _location(mdir / "geo.net.xml") if (mdir / "geo.net.xml").exists() else (0.0, 0.0)
    return mo[0] - go[0], mo[1] - go[1]


def _location(net_file):
    for _, el in ET.iterparse(str(net_file), events=("end",)):
        if el.tag == "location":
            return tuple(float(v) for v in el.get("netOffset", "0,0").split(","))
    return 0.0, 0.0


def lane_spans(lanes, polygons, shift=(0.0, 0.0)):
    """{lane id: [(s0, s1)]}: where each lane's centre line runs through a crosswalk, in the lane's own length
    (SUMO's lane positions). A lane that runs along a crosswalk (inside it for more than ALONG_X its width: a side
    road's lane at a junction's corner along an OSM crossing's strip) does not cross it. polygons in OpenDRIVE x, y;
    shift: net_shift()."""
    from shapely.geometry import LineString, Point, Polygon
    from shapely.strtree import STRtree
    geoms, widths = [], []
    for p in polygons:
        g = Polygon(p)
        g = g if g.is_valid else g.buffer(0)
        if not g.is_empty and g.area > 0.1:
            geoms.append(g)
            r = g.minimum_rotated_rectangle
            c = list(r.exterior.coords) if hasattr(r, "exterior") else []
            sides = [math.dist(c[i], c[i + 1]) for i in range(min(2, len(c) - 1))]
            widths.append(min(sides) if sides else 4.0)
    if not geoms:
        return {}
    tree = STRtree(geoms)
    out = {}
    for lid, ln in lanes.items():
        line = LineString([(x - shift[0], y - shift[1]) for x, y in ln["shape"]])
        if line.length <= 0:
            continue
        scale = ln["length"] / line.length
        spans = set()
        for g in tree.query(line):
            k = int(g) if not hasattr(g, "geom_type") else geoms.index(g)  # shapely 2: indices; 1.8: geometries
            inter = line.intersection(geoms[k])
            for part in getattr(inter, "geoms", [inter]):
                if part.is_empty or part.length > ALONG_X * widths[k]:
                    continue
                ss = [line.project(Point(c)) for c in part.coords]
                spans.add((round(min(ss) * scale, 3), round(max(ss) * scale, 3)))
        if spans:
            out[lid] = sorted(spans)
    return out


def approaches(lanes, nodes, spans, upstream=UPSTREAM_M):
    """{lane id: {"length", "spans", "start", "tls", "inner", "no_in"}} of every lane that ends at a junction (not a
    dead end, nor a plain join where it only runs on into one lane) with a crosswalk on it or on the lanes that lead
    into it, followed back up to `upstream` m (netconvert
    splits OpenDRIVE roads into SUMO edges as short as 0.2 m in front of a signal, and a twin's junction may be two
    nodes 0.2 m apart with the crosswalk across both). It is not followed past a signal, unless the lane is too
    short for a car to wait in after it (inner: one junction drawn as two signalised nodes; a car that passed the
    first light on green then waits before it, not over the crosswalk between them). spans in the lane's own
    positions (negative: on the lanes before it); start: the nearest signal back where the following stopped, None
    when there is none within `upstream` m (a lane no other lane leads into does not count: nothing comes from
    there); no_in: no lane leads into it (only a car that starts on it waits there). nodes: unused (net_lanes'
    node degrees)."""
    out = {}
    short = CAR_M + 2 * MARGIN_M
    for lid, ln in lanes.items():
        if ln.get("to_type") in (None, "dead_end") or _plain_join(lanes, lid):
            continue
        sp, ends, seen, inner = list(spans.get(lid, ())), [], {lid}, False
        todo = [(lid, 0.0)]
        while todo:
            cur, base = todo.pop()
            preds = lanes[cur]["preds"]
            for p in preds:
                prev = lanes[p]
                through = prev.get("tls") and ln["length"] - base < short     # no room for a car after it
                if p in seen:
                    continue
                if prev.get("tls") and not through:
                    ends.append(base)
                    continue
                inner = inner or bool(through)
                seen.add(p)
                pb = base - prev["length"]
                sp += [(a + pb, b + pb) for a, b in spans.get(p, ())]
                if -pb < upstream:
                    todo.append((p, pb))
        if sp:
            out[lid] = {"length": ln["length"], "spans": sorted(set(sp)), "start": max(ends) if ends else None,
                        "tls": ln.get("tls", False), "inner": inner, "no_in": not ln["preds"]}
    return out


def _plain_join(lanes, lid):
    """Whether a lane only runs on into one lane that only it leads into, with no signal: nobody waits at its end."""
    ln = lanes[lid]
    succs = ln.get("succs")
    if ln.get("tls") or succs is None or len(succs) != 1:
        return False
    return len(lanes[succs[0]]["preds"]) == 1


def blocking(stop, spans, car=CAR_M):
    """The crosswalk spans a vehicle waiting with its front at `stop` covers with its front or body."""
    return [(a, b) for a, b in spans if a < stop - 0.05 and b > stop - car + 0.05]


def stop_offset(ap, margin=MARGIN_M, car=CAR_M, max_offset=MAX_OFFSET_M):
    """The stop line (m before the lane's end) of an approach (approaches()): 0 when a car waiting at the lane's end
    is clear of every crosswalk; else MARGIN_M before the crosswalk it would cover, and again before the next one
    back while the car would then cover that, up to the first place where it covers none. Where that place is past
    the signal before (or the net's edge) or more than max_offset back, the last stop line on the way whose car has
    its front out of every crosswalk (its rear may then reach over the crosswalk behind: the crosswalk the waiting
    car must keep clear is the one of the junction it waits at); 0 when there is none."""
    L, start, spans = ap["length"], ap["start"], ap["spans"]
    stop, best = L, None
    for _ in range(50):
        hit = blocking(stop, spans, car)
        if not hit:
            best = stop
            break
        stop = min(a for a, _ in hit) - margin
        if L - stop > max_offset or (start is not None and stop < start + margin):
            break
        if not any(a < stop <= b for a, b in spans):        # its front out of every crosswalk
            best = stop
    if best is None or L - best <= 0.0:
        return 0.0
    return round(L - best, 2)


def offsets(aps):
    """{lane id: stop offset (m)} of the approaches whose waiting car would stand on a crosswalk."""
    out = {}
    for lid, ap in aps.items():
        off = stop_offset(ap)
        if off > 0:
            out[lid] = off
    return out


def point_back(shapes, back):
    """(x, y, ux, uy): the point `back` m before the end of shapes[0] (a lane's polyline) and the direction of
    travel there, going on along shapes[1], shapes[2], ... (each the lane before the one in front of it, ending
    where that begins) and straight on past the last (signals.py: a head at the stop line)."""
    rem, last = max(0.0, back), None
    for shape in shapes:
        pts = [tuple(p[:2]) for p in shape]
        for (x0, y0), (x1, y1) in zip(reversed(pts[:-1]), reversed(pts[1:])):
            d = math.hypot(x1 - x0, y1 - y0)
            if d <= 1e-9:
                continue
            ux, uy = (x1 - x0) / d, (y1 - y0) / d
            if rem <= d:
                return x1 - ux * rem, y1 - uy * rem, ux, uy
            rem -= d
            last = (x0, y0, ux, uy)
    if last is None:
        x, y = shapes[0][-1][:2]
        return x, y, 1.0, 0.0
    x0, y0, ux, uy = last
    return x0 - ux * rem, y0 - uy * rem, ux, uy


def write_net(src, dst, stop_offsets):
    """A copy of net `src` at `dst` with a <stopOffset> (all vehicle classes) in each lane of stop_offsets (a lane's
    own stopOffset, if it has one, is kept when larger). SUMO takes an offset longer than the lane: its vehicles
    then wait on the lane before it (SUMO 1.28, tested)."""
    tree = ET.parse(str(src))
    n = 0
    for edge in tree.getroot().iter("edge"):
        if edge.get("function"):
            continue
        for lane in edge.findall("lane"):
            off = stop_offsets.get(lane.get("id"))
            if not off:
                continue
            old = lane.find("stopOffset")
            if old is not None:
                if float(old.get("value", 0)) >= off:
                    continue
                lane.remove(old)
            el = ET.Element("stopOffset", {"value": f"{off:.2f}", "vClasses": "all"})
            el.tail = lane.text
            lane.insert(0, el)
            n += 1
    tree.write(str(dst), encoding="UTF-8", xml_declaration=True)
    return n


def read_offsets(net_file):
    """{lane id: stop offset (m)} of a net's lanes that have a <stopOffset> (signals.py: next_light, head spots)."""
    out, lane = {}, None
    if not net_file or not Path(net_file).exists():
        return out
    for ev, el in ET.iterparse(str(net_file), events=("start", "end")):
        if ev == "start":
            if el.tag == "lane":
                lane = el.get("id")
            continue
        if el.tag == "stopOffset" and lane is not None:
            out[lane] = max(out.get(lane, 0.0), float(el.get("value", 0)))
        elif el.tag == "lane":
            lane = None
        elif el.tag in ("edge", "junction", "connection"):
            el.clear()
    return out


def plan(map_dir, net_file=None, log=print, cache=True):
    """(lanes, approaches, crosswalk polygons) of a built map's net (default map.net.xml)."""
    mdir = Path(map_dir)
    net_file = Path(net_file or mdir / "map.net.xml")
    polys = crossings(mdir, cache=cache, log=log)
    lanes, nodes, _ = net_lanes(net_file)
    aps = approaches(lanes, nodes, lane_spans(lanes, polys, net_shift(mdir, net_file))) if polys else {}
    return lanes, aps, polys


def prepare(map_dir, net_file, work, log=print):
    """traffic.prepare's hook: the trial's net with the stop lines before crosswalks (work/stops.net.xml), or None
    (no waiting car would stand on a crosswalk, or the net already has the stop lines)."""
    if not crossings(map_dir, log=log):
        return None
    lanes, aps, polys = plan(map_dir, net_file, log)
    offs = offsets(aps)
    have = read_offsets(net_file)
    offs = {k: v for k, v in offs.items() if have.get(k, 0.0) < v}
    if not offs:
        return None
    out = Path(work) / "stops.net.xml"
    n = write_net(net_file, out, offs)
    log(f"stop lines: {n} approach lanes stop before a crosswalk ({len(polys)} crosswalks; "
        f"{sum(1 for k in offs if lanes[k]['tls'])} at signals, median "
        f"{sorted(offs.values())[len(offs) // 2]:.1f} m before the lane's end)")
    return out


# --- the check -----------------------------------------------------------------------------------------------------

def violations(aps, stop_offsets=None, car=CAR_M):
    """[{"lane", "length", "stop", "tls", "front_in", "body_on", "inner"}] of the approaches where a vehicle waiting at
    the stop line (lane end minus its stop offset) has its front inside a crosswalk (front_in) or its body (car m
    behind the front) on one (body_on). inner, no_in: approaches()."""
    out = []
    stop_offsets = stop_offsets or {}
    for lid, ap in sorted(aps.items()):
        stop = ap["length"] - stop_offsets.get(lid, 0.0)
        hit = blocking(stop, ap["spans"], car)
        if hit:
            out.append({"lane": lid, "length": round(ap["length"], 2), "stop": round(stop, 2), "tls": ap["tls"],
                        "front_in": any(a < stop - 0.05 <= b for a, b in hit), "body_on": True,
                        "inner": ap.get("inner", False), "no_in": ap.get("no_in", False)})
    return out


def has_crosswalks(xodr):
    """Whether an OpenDRIVE file has crosswalk objects (a quick text search)."""
    with open(xodr, encoding="utf-8", errors="replace") as f:
        return any('type="crosswalk"' in line for line in f)


def check(map_dir, net_file=None, log=print, cache=True):
    """Stop positions on crosswalks on a built map, before and after the stop lines: {"map", "crosswalks", "source",
    "approach_lanes" (lanes ending at a junction with a crosswalk before it), "before": [violations], "after":
    [violations], "offsets"}."""
    mdir = Path(map_dir)
    net_file = Path(net_file or mdir / "map.net.xml")
    lanes, aps, polys = plan(mdir, net_file, log, cache)
    have = read_offsets(net_file)
    offs = offsets(aps)
    after = {k: max(have.get(k, 0.0), offs.get(k, 0.0)) for k in set(have) | set(offs)}
    source = "opendrive" if (mdir / "map.xodr").exists() and has_crosswalks(mdir / "map.xodr") else \
        ("osm" if polys else None)
    return {"map": mdir.name, "crosswalks": len(polys), "source": source, "approach_lanes": len(aps),
            "signal_lanes": sum(1 for a in aps.values() if a["tls"]),
            "before": violations(aps, have), "after": violations(aps, after), "offsets": offs}
