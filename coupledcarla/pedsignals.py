"""Walkers cross at signals with their light, and scramble crossings get their all-pedestrian phase.

SUMO's net has no pedestrian crossings here (no sidewalks, see README), so its signal programmes have no pedestrian
phases and the crowd (crowdpeds.py) crossed at junctions on gap acceptance alone, also where the cars had green.
Now:

- PedSignals: a crowd walker about to cross near a signalised junction may start only when every signalised vehicle
  stream that crosses its path there shows red: the SUMO links whose stop line lies within NEAR_M of the crossing
  and whose lane runs across the crossing line. Gap acceptance still applies on top (turning vehicles), and SUMO
  vehicles still yield to walkers on the road (yielding.py). A crossing with no such stream is unsignalised.
- scramble_programs: junctions with a scramble crossing in OSM (`crossing:scramble=yes`, Shibuya's) get an extra
  phase in which every vehicle stream is red (SCRAMBLE_S long), appended to each programme of the SUMO traffic
  light nearest to the tagged crossings; walkers then cross while it lasts, in any direction.
"""
import math
import types
import xml.etree.ElementTree as ET
from pathlib import Path

NEAR_M = 20.0          # stop lines this close to a crossing control it
SCRAMBLE_S = 30        # the all-pedestrian phase (s); no open timing for Shibuya, 30 s is a typical scramble phase
AMBER_S = 3           # amber for streams still green when the all-pedestrian phase starts
SCRAMBLE_REACH_M = 60  # a scramble tag this close to a traffic light's junction belongs to it


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


class PedSignals:
    """crowdpeds.CrowdSim asks may_cross(a, b): True (the streams crossing it are red), False (one is not) or None
    (no signal controls it)."""

    def __init__(self, links, state_of):
        self.links, self.state_of = links, state_of     # state_of(tls) -> SUMO state string
        self._cache = {}

    @classmethod
    def from_sumo(cls, net, offset):
        import traci
        return cls(net_links(net, offset), traci.trafficlight.getRedYellowGreenState)

    def may_cross(self, a, b):
        key = (round(a[0], 1), round(a[1], 1), round(b[0], 1), round(b[1], 1))
        ctl = self._cache.get(key)
        if ctl is None:
            ctl = self._cache[key] = crossing_links(self.links, a, b)
        if not ctl:
            return None
        try:
            states = {tls: self.state_of(tls) for tls in {t for t, _ in ctl}}
        except Exception:  # noqa: BLE001 - SUMO gone at the end of a trial: let gap acceptance decide
            return None
        return all_red(states, ctl)


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


def scramble_programs(programs, tls_ids, path, seconds=SCRAMBLE_S):
    """Writes an additional file with each programme of the given traffic lights plus an all-red phase at its end
    (SUMO runs the programme loaded last); returns the number of programmes written. programs: tls_programs()."""
    rows, n = ["<additional>"], 0
    for tid in tls_ids:
        for pid, phases in (programs.get(tid) or {}).items():
            if not phases:
                continue
            phases = [types.SimpleNamespace(duration=d, state=st) for d, st in phases]
            size = len(phases[0].state)
            rows.append(f'  <tlLogic id="{tid}" type="static" programID="{pid}_scramble" offset="0">')
            for ph in phases:
                rows.append(f'    <phase duration="{ph.duration:g}" state="{ph.state}"/>')
            last = phases[-1].state
            if any(c in "Gg" for c in last):       # streams still green: amber before the all-red phase
                amber = "".join("y" if c in "Gg" else c for c in last)
                rows.append(f'    <phase duration="{AMBER_S:g}" state="{amber}"/>')
            rows.append(f'    <phase duration="{seconds:g}" state="{"r" * size}"/>')
            rows.append("  </tlLogic>")
            n += 1
    rows.append("</additional>")
    Path(path).write_text("\n".join(rows) + "\n")
    return n


def prepare(map_dir, net_file, work, log=print):
    """traffic.prepare's hook: the scramble programmes of the map (scramble.add.xml in work), or None."""
    pts = scramble_points(Path(map_dir) / "map.osm")
    if not pts:
        return None
    import sumolib
    net = sumolib.net.readNet(str(net_file))          # the lights' links; their programmes: tls_programs
    xy = [net.convertLonLat2XY(lon, lat) for lat, lon in pts]
    tls = scramble_tls(net, xy)
    if not tls:
        log(f"scramble crossings: {len(pts)} OSM points, no traffic light near them")
        return None
    n = scramble_programs(tls_programs(net_file), tls, Path(work) / "scramble.add.xml")
    log(f"scramble crossings: {len(pts)} OSM points -> all-pedestrian phase of {SCRAMBLE_S} s at {len(tls)} traffic "
        f"lights ({n} programmes)")
    return "scramble.add.xml"
