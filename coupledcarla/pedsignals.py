"""Walkers cross at signals with their light, and scramble crossings get their all-pedestrian phase.

SUMO's net has no pedestrian crossings here (no sidewalks, see README), so its signal programmes have no pedestrian
phases and the crowd (crowdpeds.py) crossed at junctions on gap acceptance alone, also where the cars had green.
Now:

- PedSignals: a crowd walker about to cross near a signalised junction may start only when every signalised vehicle
  stream that crosses its path there shows red: the SUMO links whose stop line lies within NEAR_M of the crossing
  and whose lane runs across the crossing line. Gap acceptance still applies on top (turning vehicles), and SUMO
  vehicles still yield to walkers on the road (yielding.py). A crossing with no such stream is unsignalised.
  red_remaining() also says how long those streams stay red (the programme's phases ahead): scramble.py's walkers
  start only while the walk signal shows, run when it runs out and wait through the flashing.
- crossing_heads: where the pedestrian signal heads of those crossings stand (both ends, on the kerb, facing across);
  signals.py spawns and lights them by the same rule (walk while every crossing stream is red);
- Scramble crossings (scramble.py's model, README "Busy and scramble crossings"): junctions with a scramble
  crossing in OSM (`crossing:scramble=yes`, Shibuya's; a twin without map.osm asks Overpass once and caches
  real/osm_crossings.json) get an all-pedestrian phase: every vehicle stream of every light of the junction red for
  lead + walk + flashing + clearance seconds, in step across the junction's lights (one cycle, offset 0).
- Junction controllers where SUMO's lights cannot give pedestrians a phase: the SUMO net made from the OpenDRIVE has
  one light per OpenDRIVE signal, all green together (42 s green, 3 s amber, 3 s red), so a crossing at a
  signalised junction outside the Netherlands never saw its cars stopped for more than 3 s, and at Shibuya only 5 of
  the junction's links had a light at all. With traffic.walkers.crossings.busy_lights (auto: when the trial runs
  SUMO's lights) the signalised junctions get signalplan.py's controllers (one light per junction, stages from the
  conflicts; static, cycle from the stages), and a scramble junction's nodes a controller even where OSM put no
  signal on them.
"""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

NEAR_M = 20.0          # stop lines this close to a crossing control it
AMBER_S = 3           # amber for streams still green when the all-pedestrian phase starts
SCRAMBLE_REACH_M = 60  # a scramble tag this close to a traffic light's junction belongs to it
SCRAMBLE_CLUSTER_M = 60.0   # scramble-tagged OSM points this close to each other are one junction
SCRAMBLE_NODE_M = 32.0      # SUMO nodes this close to a scramble junction's centre are part of it
MIN_PED_RED_S = 10.0   # a light whose links are never red together this long gives walkers no time to cross
LOOKAHEAD_S = 300.0    # red_remaining looks this far ahead in a programme


def _seg_dist(px, py, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    u = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2)) if L2 > 0 else 0.0
    return math.hypot(px - ax - u * dx, py - ay - u * dy)


def crossing_links(links, a, b, near=NEAR_M):
    """The (tls, index) of links that control a crossing a-b (CARLA x, y): stop line within `near` m of the crossing,
    lane direction across it (more than 45 deg from the crossing line). links: [(tls, index, (x, y) stop point,
    (dx, dy) lane direction)] in CARLA coordinates."""
    ux, uy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(ux, uy) or 1.0
    ux, uy = ux / L, uy / L
    out = []
    for tls, idx, (x, y), (dx, dy) in links:
        if _seg_dist(x, y, a, b) > near:
            continue
        n = math.hypot(dx, dy) or 1.0
        if abs((dx * ux + dy * uy) / n) > math.cos(math.radians(45.0)):
            continue                                  # runs along the crossing, not across it
        out.append((tls, idx))
    return out


def all_red(states, controlling):
    """Whether every controlling link shows red in states {tls: state string}."""
    return all(states.get(tls, "")[idx:idx + 1] in ("r", "R") for tls, idx in controlling)


def net_links(net, offset):
    """[(tls, index, (x, y), (dx, dy))] of every signalised link of a sumolib net, in CARLA coordinates (the bridge's
    offset: CARLA = (x - off_x, off_y - y))."""
    out = []
    for tls in net.getTrafficLights():
        for in_lane, _out_lane, idx in tls.getConnections():
            shape = in_lane.getShape()
            if len(shape) < 2:
                continue
            (x0, y0), (x1, y1) = shape[-2], shape[-1]
            out.append((tls.getID(), idx, (x1 - offset[0], offset[1] - y1), (x1 - x0, -(y1 - y0))))
    return out


def red_ahead(phases, i, left, idxs, horizon=LOOKAHEAD_S):
    """Seconds the links idxs stay red from now: phases [(duration, state, ...)] of the running programme, i the
    current phase, left the seconds left in it; 0 when one of them is not red now."""
    if not phases or not 0 <= i < len(phases):
        return 0.0
    red = (lambda st: all(st[k:k + 1] in ("r", "R") for k in idxs))
    if not red(phases[i][1]):
        return 0.0
    r, j = max(0.0, float(left)), i
    for _ in range(len(phases)):
        j = (j + 1) % len(phases)
        if j == i or not red(phases[j][1]):
            return min(r, horizon) if j != i else horizon
        r += float(phases[j][0])
        if r >= horizon:
            return horizon
    return min(r, horizon)


HEAD_BEYOND_M = 0.6     # a pedestrian head's pole this far beyond the road's edge at the crossing's end
HEAD_REACH_M = 25.0     # a crossing's end is looked for this far from its middle
HEAD_APART_M = 1.5      # two heads facing the same way closer than this are one


def crossing_heads(polygons, links, on_road, near=NEAR_M, step=0.25):
    """Where the pedestrian heads of the signalised crossings stand: [(x, y, travel yaw (deg), [(tls, index)])] in
    CARLA coordinates, two per crossing, one at each end on the kerb (HEAD_BEYOND_M past where the crossing's axis
    leaves the road for a strip at least that wide: a narrow median is passed over), each facing across the crossing towards the people waiting at the other end, as the far-side
    heads of Dutch, British and Japanese crossings do (travel yaw: the direction the head's lenses face away from,
    signals.regional_points' convention). A crossing (a polygon [(x, y)] in CARLA coordinates: the map's crosswalks,
    stoplines.crossings with y flipped) is signalised when a signalised link (net_links) controls it (crossing_links);
    its axis runs across the nearest such link's lane. on_road(x, y): a driving or cycling lane of the map is there."""
    out = []
    for poly in polygons:
        if len(poly) < 3:
            continue
        cx = sum(p[0] for p in poly) / len(poly)
        cy = sum(p[1] for p in poly) / len(poly)
        cand = sorted(((math.hypot(x - cx, y - cy), (tls, i, (x, y), d)) for tls, i, (x, y), d in links
                       if math.hypot(x - cx, y - cy) <= near), key=lambda t: t[0])
        if not cand:
            continue
        dx, dy = cand[0][1][3]
        L = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / L, dx / L                       # the crossing's axis: across the nearest stream's lane
        a = (cx - nx * HEAD_REACH_M, cy - ny * HEAD_REACH_M)
        b = (cx + nx * HEAD_REACH_M, cy + ny * HEAD_REACH_M)
        ctl = crossing_links([c[1] for c in cand], a, b, near)
        if not ctl:
            continue
        def off(t, sgn):
            return not on_road(cx + sgn * nx * t, cy + sgn * ny * t)
        if off(0.0, 1.0):                             # the crossing's middle is not on a road: not a road crossing
            continue
        for sgn in (1.0, -1.0):
            t, edge = step, None
            while t <= HEAD_REACH_M:                  # the first strip off the road wide enough for the pole
                if off(t, sgn) and off(t + HEAD_BEYOND_M / 2, sgn) and off(t + HEAD_BEYOND_M, sgn):
                    edge = t
                    break
                t += step
            if edge is None:                          # never leaves the road
                continue
            hx, hy = cx + sgn * nx * (edge + HEAD_BEYOND_M), cy + sgn * ny * (edge + HEAD_BEYOND_M)
            yaw = math.degrees(math.atan2(sgn * ny, sgn * nx))      # lenses face back across (-sgn * axis)
            if any(math.hypot(hx - x, hy - y) < HEAD_APART_M and abs((yaw - w + 180) % 360 - 180) < 45
                   for x, y, w, _ in out):
                continue
            out.append((hx, hy, yaw, sorted(set(ctl))))
    return out


class PedSignals:
    """crowdpeds.CrowdSim asks may_cross(a, b): True (the streams crossing it are red), False (one is not) or None
    (no signal controls it); scramble.py asks red_remaining(controlling links) as well."""

    def __init__(self, links, state_of, logic_of=None):
        self.links, self.state_of = links, state_of     # state_of(tls) -> SUMO state string
        # logic_of(tls) -> (phases [(duration, state, name)], current phase index, seconds left in it)
        self.logic_of = logic_of
        self._cache = {}

    @classmethod
    def from_sumo(cls, net, offset):
        import traci
        progs = {}

        def logic_of(tls):
            pid = traci.trafficlight.getProgram(tls)
            if (tls, pid) not in progs:
                ph = []
                for lg in traci.trafficlight.getAllProgramLogics(tls):
                    if lg.programID == pid:
                        ph = [(float(p.duration), p.state, getattr(p, "name", "") or "") for p in lg.phases]
                progs[(tls, pid)] = ph
            left = traci.trafficlight.getNextSwitch(tls) - traci.simulation.getTime()
            return progs[(tls, pid)], traci.trafficlight.getPhase(tls), left
        return cls(net_links(net, offset), traci.trafficlight.getRedYellowGreenState, logic_of)

    def controlling(self, a, b):
        key = (round(a[0], 1), round(a[1], 1), round(b[0], 1), round(b[1], 1))
        ctl = self._cache.get(key)
        if ctl is None:
            ctl = self._cache[key] = crossing_links(self.links, a, b)
        return ctl

    def may_cross(self, a, b):
        ctl = self.controlling(a, b)
        if not ctl:
            return None
        try:
            states = {tls: self.state_of(tls) for tls in {t for t, _ in ctl}}
        except Exception:  # noqa: BLE001 - SUMO gone at the end of a trial: let gap acceptance decide
            return None
        return all_red(states, ctl)

    def red_remaining(self, ctl):
        """(red now, seconds they stay red, name of the phase now) for controlling links [(tls, index)]: every one red
        now, and how long until the first of them turns amber or green (the programme's phases ahead)."""
        by = {}
        for tls, k in ctl:
            by.setdefault(tls, []).append(k)
        best, name = LOOKAHEAD_S, ""
        try:
            for tls, idxs in by.items():
                if not all(self.state_of(tls)[k:k + 1] in ("r", "R") for k in idxs):
                    return False, 0.0, ""
                if self.logic_of is None:
                    best = min(best, 0.0)
                    continue
                phases, i, left = self.logic_of(tls)
                if 0 <= i < len(phases) and len(phases[i]) > 2 and phases[i][2]:
                    name = phases[i][2]
                best = min(best, red_ahead(phases, i, left, idxs))
        except Exception:  # noqa: BLE001 - SUMO gone at the end of a trial
            return False, 0.0, ""
        return True, best, name

    def tls_links(self, tls_ids):
        """[(tls, index)] of every link of these lights."""
        ids = set(tls_ids)
        return sorted({(t, k) for t, k, _p, _d in self.links if t in ids})


def scramble_points(osm_path):
    """(lat, lon) of OSM nodes tagged crossing:scramble=yes, and of the nodes of ways tagged so."""
    pts, nodes, ways = [], {}, []
    try:
        for _, el in ET.iterparse(str(osm_path)):
            if el.tag == "node":
                nodes[el.get("id")] = (float(el.get("lat")), float(el.get("lon")))
                if any(t.get("k") == "crossing:scramble" and t.get("v") == "yes" for t in el.iter("tag")):
                    pts.append(nodes[el.get("id")])
            elif el.tag == "way":
                if any(t.get("k") == "crossing:scramble" and t.get("v") == "yes" for t in el.iter("tag")):
                    ways.append([nd.get("ref") for nd in el.iter("nd")])
                el.clear()
    except (OSError, ET.ParseError):
        return []
    for refs in ways:
        pts += [nodes[r] for r in refs if r in nodes]
    return pts


def scramble_tls(net, xy_points, reach=SCRAMBLE_REACH_M):
    """Ids of the traffic lights whose junction is the nearest within `reach` m of any of the points (net x, y)."""
    centres = {}
    for tls in net.getTrafficLights():
        xs, ys = [], []
        for in_lane, _o, _i in tls.getConnections():
            x, y = in_lane.getShape()[-1]
            xs.append(x)
            ys.append(y)
        if xs:
            centres[tls.getID()] = (sum(xs) / len(xs), sum(ys) / len(ys))
    out = set()
    for px, py in xy_points:
        best = min(centres.items(), key=lambda kv: math.hypot(kv[1][0] - px, kv[1][1] - py), default=None)
        if best and math.hypot(best[1][0] - px, best[1][1] - py) <= reach:
            out.add(best[0])
    return sorted(out)


def tls_programs(net_file):
    """{tls id: {programID: [(duration, state)]}} of a SUMO net, read as XML. (sumolib's readNet(withPrograms=True)
    failed on our nets: a <param> outside a tlLogic hit its parser before any programme, AttributeError
    '_currentProgram', so scramble crossings were silently off.)"""
    out = {}
    for _, el in ET.iterparse(str(net_file), events=("end",)):
        if el.tag == "tlLogic":
            out.setdefault(el.get("id"), {})[el.get("programID", "0")] = [
                (float(p.get("duration")), p.get("state")) for p in el.findall("phase")]
            el.clear()
        elif el.tag in ("edge", "junction", "connection"):
            el.clear()
    return out


def _scramble_seconds():
    from . import scramble
    t = scramble.DEFAULTS
    return t["lead_s"] + t["walk_s"] + t["flash_s"] + t["clear_s"]


def scramble_programs(programs, tls_ids, path, seconds=None, cycle=None):
    """Writes an additional file with each programme of the given traffic lights plus an all-red phase at its end
    (SUMO runs the programme loaded last); returns the number of programmes written. programs: tls_programs().
    cycle: the greens are scaled so that every programme lasts exactly this long (lights of one junction then stay
    in step: their all-pedestrian phases at the same time)."""
    seconds = _scramble_seconds() if seconds is None else seconds
    rows, n = ["<additional>"], 0
    for tid in tls_ids:
        for pid, phases in (programs.get(tid) or {}).items():
            if not phases:
                continue
            last = phases[-1][1]
            extra = (AMBER_S if any(c in "Gg" for c in last) else 0.0) + seconds
            if cycle:
                phases = scale_greens(phases, cycle - extra)
            size = len(phases[0][1])
            rows.append(f'  <tlLogic id="{tid}" type="static" programID="{pid}_scramble" offset="0">')
            for d, st in phases:
                rows.append(f'    <phase duration="{d:g}" state="{st}"/>')
            if any(c in "Gg" for c in last):       # streams still green: amber before the all-red phase
                amber = "".join("y" if c in "Gg" else c for c in last)
                rows.append(f'    <phase duration="{AMBER_S:g}" state="{amber}"/>')
            rows.append(f'    <phase duration="{seconds:g}" state="{"r" * size}" name="scramble"/>')
            rows.append("  </tlLogic>")
            n += 1
    rows.append("</additional>")
    Path(path).write_text("\n".join(rows) + "\n")
    return n


def scale_greens(phases, total):
    """[(duration, state)] with the green phases (some link green, none amber) scaled so that all of them last
    `total` s (ambers and all-reds kept); each green at least 5 s."""
    def is_green(st):
        return any(c in "Gg" for c in st) and not any(c in "yY" for c in st)
    fixed = sum(d for d, st in phases if not is_green(st))
    green = sum(d for d, st in phases if is_green(st))
    if green <= 0:
        return list(phases)
    f = max(0.1, (total - fixed) / green)
    out = [(max(5.0, round(d * f, 1)) if is_green(st) else d, st) for d, st in phases]
    k = max(range(len(out)), key=lambda i: out[i][0] if is_green(out[i][1]) else -1)
    rest = total - sum(d for d, _ in out)                 # rounding: the longest green takes the remainder
    out[k] = (max(5.0, round(out[k][0] + rest, 1)), out[k][1])
    return out


# --- where the scramble crossings are -------------------------------------------------------------------------------

OVERPASS_CROSSINGS = """[out:json][timeout:90];
(node["highway"="crossing"]({s},{w},{n},{e});
 way["footway"="crossing"]({s},{w},{n},{e});
 nwr["crossing:scramble"]({s},{w},{n},{e}););
out tags geom;"""


def osm_crossings_json(path):
    """{"scramble": [(lat, lon)], "signals": [(lat, lon)]} from a cached Overpass answer (real/osm_crossings.json)."""
    data = json.loads(Path(path).read_text())
    out = {"scramble": [], "signals": []}
    for el in data.get("elements", []):
        t = el.get("tags") or {}
        if el.get("type") == "node":
            pts = [(el["lat"], el["lon"])] if "lat" in el else []
        else:
            pts = [(g["lat"], g["lon"]) for g in el.get("geometry") or [] if g]
        if t.get("crossing:scramble") == "yes":
            out["scramble"] += pts
        if t.get("crossing") == "traffic_signals" and el.get("type") == "node":
            out["signals"] += pts
    return out


def osm_crossings(map_dir, bbox=None, fetch=True, log=print):
    """The map's scramble-tagged points and signalised crossings (lat, lon): from map.osm, else (a twin's map has no
    map.osm) real/osm_crossings.json, fetched from Overpass once when fetch."""
    osm = Path(map_dir) / "map.osm"
    if osm.exists():
        from .crowdpeds import osm_crossings as xings
        return {"scramble": scramble_points(osm), "signals": [(la, lo) for la, lo, sig in xings(osm) if sig]}
    cache = Path(map_dir) / "real" / "osm_crossings.json"
    if not cache.exists():
        if not fetch or not bbox:
            return {"scramble": [], "signals": []}
        try:
            from .realworld import overpass_json
            s, w, n, e = bbox
            data = overpass_json(OVERPASS_CROSSINGS.format(s=s, w=w, n=n, e=e))
        except Exception as err:  # noqa: BLE001 - offline: no scramble crossings, not cached (tried again next time)
            log(f"scramble crossings: no OSM crossings ({type(err).__name__}: {err})")
            return {"scramble": [], "signals": []}
        cache.parent.mkdir(parents=True, exist_ok=True)
        cache.write_text(json.dumps(data))
    try:
        return osm_crossings_json(cache)
    except (OSError, ValueError, KeyError) as err:
        log(f"scramble crossings: {cache} unreadable ({err})")
        return {"scramble": [], "signals": []}


def clusters(points, reach=SCRAMBLE_CLUSTER_M):
    """Single-linkage groups of points (x, y) within reach of each other: [(cx, cy, n)], largest first."""
    pts = list(points)
    parent = list(range(len(pts)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            if math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]) <= reach:
                parent[find(i)] = find(j)
    groups = {}
    for i, p in enumerate(pts):
        groups.setdefault(find(i), []).append(p)
    out = [(sum(p[0] for p in g) / len(g), sum(p[1] for p in g) / len(g), len(g)) for g in groups.values()]
    return sorted(out, key=lambda c: -c[2])


def scramble_centres(map_dir, bbox, to_xy, fetch=True, log=print):
    """[(x, y, lat, lon, n)]: the centres of the map's scramble junctions (OSM points clustered), x, y = to_xy(lat,
    lon)."""
    pts = osm_crossings(map_dir, bbox, fetch, log)["scramble"]
    if not pts:
        return []
    out = []
    for la, lo, n in clusters(pts, reach=SCRAMBLE_CLUSTER_M / 111000.0):      # in degrees: about 60 m
        x, y = to_xy(la, lo)
        out.append((x, y, la, lo, n))
    return out


# --- junction controllers and the scramble phase ---------------------------------------------------------------------

def uncoordinated(net):
    """Whether the net's lights are SUMO's per-signal ones that never give a crossing time: a signalised node with
    links of more than one light, or a light none of whose links is ever red for MIN_PED_RED_S (junctions.Net)."""
    by_node = {}
    for c in net.conns:
        if c.get("tl") and not c["from"].startswith(":"):
            e = net.edges.get(c["from"]) or {}
            by_node.setdefault(e.get("to"), set()).add(c["tl"])
    if any(len(t) > 1 for t in by_node.values()):
        return True
    for tid, states in net.tls.items():
        if not states:
            continue
        durs = net.tls_durations.get(tid) if hasattr(net, "tls_durations") else None
        if not durs:
            return True            # unknown durations: treat as SUMO's 42/3/3
        reds = [d for d, st in zip(durs, states) if st and all(ch in "rRs" for ch in st)]
        if max(reds, default=0.0) < MIN_PED_RED_S and not any(
                any(ch in "rR" for ch in st) and any(ch in "Gg" for ch in st) for st in states):
            return True
    return False


def scramble_programme(prog, timing, cycle):
    """A signalplan.design() controller's programme as a static one with an all-pedestrian phase at its end: greens
    scaled so that the programme lasts `cycle` s (the junction's controllers in step), then amber for streams still
    green, then all red for lead + walk + flashing + clearance (phase name "scramble")."""
    phases = [(float(p["duration"]), p["state"]) for p in prog["phases"]]
    sec = timing["lead_s"] + timing["walk_s"] + timing["flash_s"] + timing["clear_s"]
    last = phases[-1][1]
    extra = (AMBER_S if any(c in "Gg" for c in last) else 0.0) + sec
    phases = scale_greens(phases, cycle - extra)
    out = [{"duration": d, "state": st, "name": "stage"} for d, st in phases]
    if any(c in "Gg" for c in last):
        out.append({"duration": float(AMBER_S), "state": "".join("y" if c in "Gg" else c for c in last),
                    "name": "amber"})
    out.append({"duration": float(sec), "state": "r" * len(last), "name": "scramble"})
    return {**prog, "type": "static", "offset": 0.0, "cycle": round(sum(p["duration"] for p in out), 1),
            "phases": [{**p, "minDur": None, "maxDur": None} for p in out]}


def site_cycle(progs, timing):
    """One cycle (s) for a scramble junction's controllers: timing's cycle_s, longer when a controller's fixed
    phases and half its greens do not fit."""
    sec = timing["lead_s"] + timing["walk_s"] + timing["flash_s"] + timing["clear_s"] + AMBER_S
    need = [sum(p["duration"] * (0.5 if any(c in "Gg" for c in p["state"]) and not any(c in "yY" for c in p["state"])
                                 else 1.0) for p in pr["phases"]) + sec for pr in progs]
    return max([float(timing["cycle_s"])] + need)


def controller_programmes(map_dir, net_file, centres_xy, timing, busy=True, log=print):
    """(programmes, scramble ids, summary): signalplan.py's controllers of the net's signalised junctions (busy, or
    only those of scramble junctions), the scramble junctions' nodes (within SCRAMBLE_NODE_M of a centre, net x, y)
    each in one controller with the all-pedestrian phase, in step."""
    from . import junctions as J
    from . import signalplan as SP
    net = J.read_net(net_file)
    info = SP.node_info(net)
    locs, owner = [], {}
    for k, (x, y) in enumerate(centres_xy):           # a scramble junction is one listed junction
        locs.append({"id": f"scramble{k}", "name": f"scramble crossing {k}", "xy": (x, y)})
        for n, d in info.items():
            if math.hypot(d["xy"][0] - x, d["xy"][1] - y) <= SCRAMBLE_NODE_M and \
                    len(d["in_edges"] | d["out_edges"]) >= 3 and d["in_edges"]:
                owner.setdefault(n, f"scramble{k}")
    groups = {}
    for n, sid in owner.items():
        groups.setdefault(sid, []).append(n)
    progs, scramble_ids = [], []
    for sid, nodes in sorted(groups.items()):
        p = SP.design(net, sorted(nodes), sid, log=log)
        if p is not None:
            progs.append(p)
            scramble_ids.append(sid)
    n_scr = len(progs)
    if busy:      # every other signalised junction: one controller each (signalplan's grouping and design)
        for c in SP.controllers(net, [], None, log):
            nodes = [n for n in c["nodes"] if n not in owner]     # a scramble node stays in its own controller
            p = SP.design(net, nodes, c["id"], log=log) if nodes else None
            if p is not None:
                progs.append(p)
    if scramble_ids:
        cyc = site_cycle(progs[:n_scr], timing)
        progs[:n_scr] = [scramble_programme(p, timing, cyc) for p in progs[:n_scr]]
    for p in progs[n_scr:]:          # static: walkers read the phases ahead (actuated greens would not be known)
        p["type"] = "static"
    summary = {"scramble_controllers": n_scr, "scramble_nodes": sum(len(p["nodes"]) for p in progs[:n_scr]),
               "busy_controllers": len(progs) - n_scr, "old_lights": len(net.tls)}
    return progs, scramble_ids, summary


def _key(*parts):
    import hashlib
    return hashlib.sha1(json.dumps(parts, sort_keys=True, default=str).encode()).hexdigest()[:12]


def prepare(map_dir, net_file, work, log=print, cfg=None):
    """traffic.prepare's hook: (net file to run instead or None, additional file or None).
    - SUMO's per-signal lights (net_file is map.net.xml and uncoordinated()): a net with signalplan's controllers
      (traffic.walkers.crossings.busy_lights) and the scramble junctions' controllers with their all-pedestrian
      phase, cached in map_dir/signals/scramble_<key>.net.xml;
    - lights already one per junction (signalplan's dutch/real nets, OSM maps): the scramble junctions' programmes
      plus the all-pedestrian phase, in step (scramble.add.xml in work)."""
    from . import scramble
    cfg = cfg or {}
    sc = scramble.settings(cfg)
    if not sc["enabled"]:
        return None, None
    map_dir, work = Path(map_dir), Path(work)
    bbox = None
    try:
        bbox = json.loads((map_dir / "meta.json").read_text())["bbox"]
    except (OSError, ValueError, KeyError):
        pass
    import sumolib
    net = sumolib.net.readNet(str(net_file))
    to_xy = (lambda la, lo: net.convertLonLat2XY(lo, la))
    centres = scramble_centres(map_dir, bbox, to_xy, fetch=sc["fetch"], log=log) if sc["scramble"] else []
    from . import junctions as J
    plain = Path(net_file).resolve() == (map_dir / "map.net.xml").resolve()
    jn = J.read_net(net_file)
    durs = {}
    for _, el in ET.iterparse(str(net_file), events=("end",)):
        if el.tag == "tlLogic":
            durs.setdefault(el.get("id"), [float(ph.get("duration")) for ph in el.findall("phase")])
            el.clear()
    jn.tls_durations = durs
    busy = sc["busy_lights"]
    if busy == "auto":
        busy = plain and uncoordinated(jn)
    if plain and (busy or centres) and uncoordinated(jn):
        key = _key(SCRAMBLE_VERSION, str(net_file), Path(net_file).stat().st_size, int(Path(net_file).stat().st_mtime),
                   [(round(x, 1), round(y, 1)) for x, y, *_ in centres], bool(busy),
                   {k: sc[k] for k in ("lead_s", "walk_s", "flash_s", "clear_s", "cycle_s")})
        out = map_dir / "signals" / f"scramble_{key}.net.xml"
        if not out.exists():
            from . import signalplan as SP
            import os
            progs, ids, summ = controller_programmes(map_dir, net_file, [(x, y) for x, y, *_ in centres], sc,
                                                     busy=bool(busy), log=log)
            if not progs:
                return None, None
            if not busy:    # only the scramble junctions changed: the other lights stay SUMO's
                log("scramble crossings: busy_lights off, other junctions keep SUMO's lights")
                progs = progs + _keep_sumo_lights(jn, {n for p in progs for n in p["nodes"]})
            tmp = out.with_name(f".{out.name}.{os.getpid()}")
            SP.write_net(net_file, tmp, progs)
            os.replace(tmp, out)
            (out.with_suffix(".json")).write_text(json.dumps({"summary": summ, "scramble": ids,
                                                              "centres": centres}, indent=1, default=str))
        summ = json.loads(out.with_suffix(".json").read_text())["summary"]
        log(f"crossings: {summ['busy_controllers']} junction controllers in place of {summ['old_lights']} per-signal "
            f"lights" + (f"; {len(centres)} scramble junction(s) ({summ['scramble_nodes']} nodes): all-pedestrian "
                         f"phase {sc['walk_s']:g} s walk + {sc['flash_s']:g} s flashing (+ {sc['lead_s']:g} s lead, "
                         f"{sc['clear_s']:g} s clearance), cycle {sc['cycle_s']:g} s" if centres else ""))
        return out, None
    if not centres:
        return None, None
    tls = scramble_tls(net, [(x, y) for x, y, *_ in centres])
    if not tls:
        log(f"scramble crossings: {len(centres)} junction(s) in OSM, no traffic light near them")
        return None, None
    progs = tls_programs(net_file)
    cyc = max([float(sc["cycle_s"])] + [sum(d for d, _ in ph) * 0.6 + _scramble_seconds() + AMBER_S
                                        for t in tls for ph in (progs.get(t) or {}).values()])
    n = scramble_programs(progs, tls, work / "scramble.add.xml", cycle=cyc)
    log(f"scramble crossings: {len(centres)} junction(s) -> all-pedestrian phase of {_scramble_seconds():g} s at "
        f"{len(tls)} traffic lights ({n} programmes, cycle {cyc:.0f} s, in step)")
    return None, "scramble.add.xml"


SCRAMBLE_VERSION = 1


def _keep_sumo_lights(net, skip_nodes):
    """SUMO's lights of the nodes not in skip_nodes as write_net programmes (busy_lights off)."""
    by_tl = {}
    for c in net.conns:
        if c.get("tl") and not c["from"].startswith(":"):
            node = (net.edges.get(c["from"]) or {}).get("to")
            if node in skip_nodes:
                continue
            by_tl.setdefault(c["tl"], []).append((int(c["linkIndex"]), c, node))
    out = []
    for tid, lks in by_tl.items():
        lks.sort(key=lambda x: x[0])
        states = net.tls.get(tid) or []
        durs = net.tls_durations.get(tid) or [42.0] * len(states)
        out.append({"id": tid, "nodes": sorted({n for _, _, n in lks}), "type": "static", "offset": 0.0,
                    "links": [{"k": k, "from": f"{c['from']}_{c['fromLane']}", "to": f"{c['to']}_{c['toLane']}"}
                              for k, c, _ in lks],
                    "phases": [{"duration": d, "state": st} for d, st in zip(durs, states)]})
    return out
