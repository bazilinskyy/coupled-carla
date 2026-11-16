"""Traffic lights that run like the real ones (traffic.signals): a programme per junction instead of netconvert's.

The SUMO net made from the OpenDRIVE (mapbuild.sumo_net, netconvert_carla.py) has a traffic light per OpenDRIVE
signal: on eindhoven_centre 210 one-link lights, each 42 s green, 3 s amber, 3 s red, all in step. A junction's
streams were therefore green together (left turns, cyclists and the crossing road at once, sorted out by right of
way) and red together, and links the OSM signal did not reach had no light at all. Here a junction is one
controller, as on the street:

- controllers(): the signalised SUMO nodes, matched to the city's own list of signalised junctions where one is
  known (Eindhoven: the 180 locations of data.eindhoven.nl `locaties-verkeerlichten`, joined to the VRI numbers of
  the Nota Verkeerslichten 2024 and to the iVRIs on the UDAP map); nodes of one junction (dual carriageways) are
  one controller, and a listed junction OSM has no signal for gets one;
- design(): Dutch signal groups (per approach: through and right together, a protected left from its own lane,
  cyclists their own), stages of groups that may be green together (conflicts from SUMO's own request matrix and
  the link shapes; a turning stream may share its green with one it yields to, as a 'g', except a left turn from
  its own lane), amber from CROW's 2022 handbook, all-red clearance from the paths' distances to the conflict
  points (rounded up to 0.5 s as Eindhoven does), cycle and maximum cycle from the Nota (under 90 s inside the
  Ring, at most 120 s on it), vehicle-actuated (SUMO's "actuated": the green ends when the detectors see a 3 s gap,
  between a minimum and a maximum) as nearly every Dutch signal is;
- write_net(): a copy of map.net.xml with these controllers (connections' tl and linkIndex, the junction type and
  the tlLogic); lanes, edges and their geometry stay byte for byte as they were (netconvert is not run again: it
  re-shaped 110 lanes and dropped 3);
- prepare(): traffic.prepare's hook. traffic.signals: "sumo" (netconvert's, as before), "dutch", "real" (replay of
  recorded phase data where signal_data has it, signaldata.py; Dutch elsewhere) or "auto" (real where data is
  configured, else Dutch in the Netherlands, else SUMO's).

Data sources and what access to the real data needs: README, Traffic lights, and docs/signals.md.
"""
import csv
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from . import junctions as J

VERSION = 11                       # bump when the design changes: the cached nets are rebuilt
DATA = Path(__file__).parent / "data" / "signals"

# --- Dutch timings ----------------------------------------------------------------------------------------------
# CROW, Handboek verkeerslichtenregelingen 2022, table "duur van de geelfase": straight-on amber by the approach's
# speed limit; turning streams 3.0 s; cyclists with mopeds 3.0 s (cyclists only 2.0 s); buses on their own lane 2.0 s
# only with selective detection, so 3.0 s here.
AMBER_STRAIGHT = ((50, 3.5), (60, 4.0), (70, 4.5), (80, 5.0))
AMBER_TURN = 3.0
AMBER_BIKE = 3.0
# all-red (clearance, "ontruimingstijd"): the last vehicle leaving at EXIT m/s must clear the conflict point (plus its
# length) before the first one of the next stage, starting at ENTRY m/s, reaches it; rounded up to 0.5 s (Nota 2024)
EXIT = {"car": 10.0, "turn": 7.0, "bike": 5.0, "bus": 8.0}
ENTRY = {"car": 6.0, "bike": 4.0, "bus": 5.0}
LENGTH = {"car": 6.0, "bike": 2.0, "bus": 12.0}
ALL_RED_MAX = 6.0
# greens (s): CROW's fixed green ("vastgroen") of 3-4 s plus the guaranteed green; Dutch minimum greens are 4-6 s
MIN_GREEN = {"car": 6.0, "bike": 5.0, "bus": 5.0}
# cycle (s): Nota Verkeerslichten 2024 (Eindhoven): design cycle under 90 s for junctions inside the Ring, at most
# 120 s on the Ring and elsewhere; the base cycle of a junction grows with its stages
CYCLE_BY_STAGES = {1: 50.0, 2: 50.0, 3: 65.0, 4: 80.0}
CYCLE_MAX_INNER, CYCLE_MAX_MAIN = 90.0, 120.0
MAIN_ROAD_SPEED = 13.0            # m/s (about 50 km/h and more) with 2+ lanes: a main road (the Ring, radials)
MAX_GREEN_FACTOR = 1.8            # actuated: the maximum green is this times the planned green (within the cap)
FEEDERS = True                    # a stream into the middle of a multi-node junction keeps its exit's conflicts
PED_STAGE = 12.0                  # a mid-block crossing's red for the road (s): 9 m at 1.2 m/s (Regeling) + start
# actuated: SUMO's defaults (max-gap 3.0 s, detector-gap 2.0 s, passing-time 1.9 s); no <param> is written: the
# bridge (sumo_simulation.SumoTLLogic) reads every tlLogic param as a CARLA landmark link and fails on others

# --- matching -----------------------------------------------------------------------------------------------------
MATCH_M = 45.0                    # a listed junction this close to a signalised node controls it
JOIN_M = 30.0                     # signalised nodes this close to each other (and to no listed junction) are one
NEW_M = 30.0                      # a listed junction with no signalised node: unsignalised nodes this close get one
TURNS = {"l": "l", "L": "l", "t": "l", "r": "r", "R": "r", "s": "s"}
EHV_LOCATIONS = "https://data.eindhoven.nl/api/explore/v2.1/catalog/datasets/locaties-verkeerlichten/exports/json"
EHV_AREA = (51.38, 5.38, 51.51, 5.56)       # the municipality, roughly (s, w, n, e)
SOURCES = {
    "eindhoven_vri": ("Gemeente Eindhoven, locaties verkeerslichten (data.eindhoven.nl), with the VRI numbers of "
                      "the Nota Verkeerslichten 2024", "Public Domain"),
    "udap_ivri": ("iVRI list of the UDAP map (map.udap.nl, NDW / Ministerie van IenW, run by Monotch)",
                  "no licence stated; used only to mark which junctions publish SPaT"),
    "crow": ("CROW, Handboek verkeerslichtenregelingen 2022 (amber times); Gemeente Eindhoven, Nota "
             "Verkeerslichten 2024 (cycle times, maximum waits, clearance rounding)", "cited values"),
}


def _log(msg):
    print(f"[signals] {msg}", flush=True)


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def amber(mode, turn, speed_ms):
    """CROW 2022 amber (s) for a stream: mode car|bike|bus, turn l|s|r, its approach speed limit (m/s)."""
    if mode == "bike":
        return AMBER_BIKE
    if turn != "s" or mode == "bus":
        return AMBER_TURN if mode != "bus" else 3.0
    kmh = speed_ms * 3.6
    for limit, a in AMBER_STRAIGHT:
        if kmh <= limit + 2.0:
            return a
    return AMBER_STRAIGHT[-1][1]


def round_half_up(x):
    """Up to the next 0.5 s (Eindhoven rounds clearance times so)."""
    return math.ceil(x * 2.0 - 1e-9) / 2.0


# --- the city's list of signalised junctions ----------------------------------------------------------------------

def _bundled_vri():
    rows = []
    try:
        with open(DATA / "eindhoven_vri.csv", newline="") as f:
            for r in csv.DictReader(f):
                rows.append({"vri": r["vri"], "name": r["name"], "lat": float(r["lat"]) if r["lat"] else None,
                             "lon": float(r["lon"]) if r["lon"] else None})
    except OSError:
        pass
    return rows


def _ivri():
    out = []
    try:
        with open(DATA / "ivri_eindhoven_region.csv", newline="") as f:
            for r in csv.DictReader(f):
                out.append({"name": r["name"], "tlc_id": r["tlc_id"], "lat": float(r["lat"]),
                            "lon": float(r["lon"]), "authority": r["road_authority"]})
    except OSError:
        pass
    return out


def _norm(name):
    return "".join(c for c in (name or "").lower() if c.isalnum())


def locations(bbox, real_dir=None, fetch=True, log=_log):
    """The city's signalised junctions in or near bbox (s, w, n, e): [{id, name, lat, lon, vri, ivri, source}].
    Eindhoven: the open data list (fetched once, cached in real/signal_locations.json; the bundled copy when the
    portal does not answer) joined to the Nota's VRI numbers by name and to the iVRIs (UDAP) by distance. Elsewhere
    an empty list (the OSM signals of the net are used as they are)."""
    s, w, n, e = bbox
    if not (s < EHV_AREA[2] and n > EHV_AREA[0] and w < EHV_AREA[3] and e > EHV_AREA[1]):
        return []
    cache = Path(real_dir) / "signal_locations.json" if real_dir else None
    rows = None
    if cache is not None and cache.exists():
        rows = json.loads(cache.read_text())
    elif fetch:
        try:
            import requests
            from .realworld import UA
            got = requests.get(EHV_LOCATIONS, params={"select": "objectid,locatie,geo_point_2d"}, headers=UA,
                               timeout=60).json()
            rows = [{"id": f"ehv{r['objectid']}", "name": (r.get("locatie") or "").strip(),
                     "lat": r["geo_point_2d"]["lat"], "lon": r["geo_point_2d"]["lon"]}
                    for r in got if (r.get("geo_point_2d") or {}).get("lat") is not None]
            if cache is not None and rows:
                cache.parent.mkdir(parents=True, exist_ok=True)
                cache.write_text(json.dumps(rows, indent=0))
        except Exception as err:  # noqa: BLE001 - offline or the portal is down: the bundled list
            log(f"Eindhoven's signal locations not fetched ({type(err).__name__}: {err}); the bundled list is used")
            rows = None
    vri = _bundled_vri()
    if not rows:
        rows = [{"id": f"vri{r['vri']}", "name": r["name"], "lat": r["lat"], "lon": r["lon"]}
                for r in vri if r["lat"] is not None]
    by_name = {_norm(r["name"]): r["vri"] for r in vri}
    by_pos = [(r["lat"], r["lon"], r["vri"]) for r in vri if r["lat"] is not None]
    ivri = _ivri()
    out = []
    pad = 0.002
    for r in rows:
        if not (s - pad <= r["lat"] <= n + pad and w - pad <= r["lon"] <= e + pad):
            continue
        key = _norm(r["name"])
        num = by_name.get(key) or next((v for k, v in by_name.items() if key and k.startswith(key)), None)
        if num is None:
            near = [v for la, lo, v in by_pos if _geo_m((la, lo), (r["lat"], r["lon"])) < 5.0]
            num = near[0] if near else None
        iv = [i for i in ivri if i["authority"] == "Eindhoven" and _geo_m((i["lat"], i["lon"]),
                                                                          (r["lat"], r["lon"])) < 80.0]
        out.append({**r, "vri": num, "ivri": iv[0]["name"] if iv else None,
                    "tlc_id": iv[0]["tlc_id"] if iv else None, "source": "eindhoven_vri"})
    return out


def _geo_m(a, b):
    """Metres between two (lat, lon) (equirectangular, fine over a city)."""
    k = math.cos(math.radians((a[0] + b[0]) / 2))
    return math.hypot((a[0] - b[0]) * 111_320.0, (a[1] - b[1]) * 111_320.0 * k)


# --- the net's signalised junctions, grouped into controllers -----------------------------------------------------

def _mode(lane):
    allow = (lane or {}).get("allow") or ""
    kinds = set(allow.split())
    if kinds and kinds <= {"bicycle", "pedestrian"} and "bicycle" in kinds:
        return "bike"
    if kinds and "passenger" not in kinds and kinds & {"bus", "coach", "tram"}:
        return "bus"
    if kinds == {"pedestrian"}:
        return "ped"
    return "car"


def node_info(net):
    """{node: {"xy", "signalised", "in_edges" (normal incoming edges), "out_edges", "links" (normal->internal)}}."""
    info = {}
    for jid, j in net.junctions.items():
        if j["type"] == "internal":
            continue
        info[jid] = {"xy": (j["x"], j["y"]), "signalised": False, "in_edges": set(), "out_edges": set()}
    for c in net.conns:
        if c["from"].startswith(":"):
            continue
        e = net.edges.get(c["from"])
        if not e or e["to"] not in info:
            continue
        nd = info[e["to"]]
        nd["in_edges"].add(c["from"])
        nd["out_edges"].add(c["to"])
        if c.get("tl"):
            nd["signalised"] = True
    return info


def controllers(net, locs, xy_of, log=_log):
    """Group the net's signalised nodes into controllers: [{id, nodes, name, vri, ivri, location, source}].
    locs: locations(); xy_of(lat, lon) -> net (x, y)."""
    info = node_info(net)
    sig = {n for n, d in info.items() if d["signalised"]}
    loc_xy = [(loc, xy_of(loc["lat"], loc["lon"])) for loc in locs]
    owner = {}
    for n in sig:                                         # each signalised node: its nearest listed junction
        best = min(((_dist(info[n]["xy"], xy), loc["id"]) for loc, xy in loc_xy), default=None)
        if best and best[0] <= MATCH_M:
            owner[n] = best[1]
    added = []
    motor = {}                                            # edges with a car or bus lane
    for lane in net.lanes.values():
        if not lane["internal"] and _mode(lane) in ("car", "bus"):
            motor[lane["edge"]] = True
    for loc, xy in loc_xy:                                # a listed junction without a signalised node near
        if any(owner.get(n) == loc["id"] for n in sig):
            continue
        if any(_dist(info[n]["xy"], xy) <= MATCH_M for n in sig):
            continue                                      # its signals belong to a neighbour (one controller)
        cands = [n for n, d in info.items() if n not in sig and _dist(d["xy"], xy) <= NEW_M
                 and (sum(motor.get(e, False) for e in d["in_edges"]) >= 2 or      # a road junction, or a
                      (any(motor.get(e) for e in d["in_edges"]) and len(d["in_edges"]) >= 3))   # bus street's
                 and len(d["in_edges"] | d["out_edges"]) >= 3]
        for n in cands:
            owner[n] = loc["id"]
            added.append(n)
    # the rest: signalised nodes near each other are one junction
    rest = sorted(n for n in sig if n not in owner)
    parent = {n: n for n in rest}

    def find(n):
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n
    for i, a in enumerate(rest):
        for b in rest[i + 1:]:
            if _dist(info[a]["xy"], info[b]["xy"]) <= JOIN_M:
                parent[find(a)] = find(b)
    groups = {}
    for n, lid in owner.items():
        groups.setdefault(("loc", lid), []).append(n)
    for n in rest:
        groups.setdefault(("osm", find(n)), []).append(n)
    by_id = {loc["id"]: loc for loc in locs}
    out = []
    for (kind, key), nodes in sorted(groups.items(), key=lambda kv: (kv[0][0], str(kv[0][1]))):
        nodes = sorted(nodes, key=lambda x: (len(x), x))
        if kind == "loc":
            loc = by_id[key]
            cid = f"vri{loc['vri']}" if loc.get("vri") else key
            out.append({"id": cid, "nodes": nodes, "name": loc["name"], "vri": loc.get("vri"),
                        "ivri": loc.get("ivri"), "tlc_id": loc.get("tlc_id"), "location": key,
                        "source": "added" if all(n in added for n in nodes) else "matched"})
        else:
            out.append({"id": f"tls{nodes[0]}", "nodes": nodes, "name": None, "vri": None, "ivri": None,
                        "tlc_id": None, "location": None, "source": "osm"})
    seen = set()
    for c in out:                                         # ids are unique
        while c["id"] in seen:
            c["id"] += "b"
        seen.add(c["id"])
    return out


# --- a controller's signal groups, stages and programme -----------------------------------------------------------

def _bearing(shape):
    """Compass bearing (deg, 0 = north) of travel at the end of a lane shape (SUMO x east, y north)."""
    (x0, y0), (x1, y1) = shape[-2], shape[-1]
    return math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0


def controller_links(net, nodes):
    """The links of a controller in the order of their new link indices: [{k, node, req, from, to, edge, dir,
    turn, mode, speed, shape, length, bearing, internal_from}]. internal_from: the approach comes from another node
    of the same controller (the median of a dual carriageway)."""
    out = []
    nodeset = set(nodes)
    for node in nodes:
        for lk in J.links(net, node):
            if not lk["from"] or lk["from"].startswith(":"):
                continue
            lane = net.lanes.get(lk["from"]) or {}
            mode = _mode(lane)
            if mode == "ped":
                continue
            edge = lane.get("edge") or lk["from"].rsplit("_", 1)[0]
            shp = lane.get("shape") or [(0, 0), (0, 1)]
            out.append({"k": len(out), "node": node, "req": lk["index"], "from": lk["from"], "to": lk["to"],
                        "edge": edge, "dir": lk["dir"] or "s", "turn": TURNS.get(lk["dir"] or "s", "s"),
                        "mode": mode, "speed": lane.get("speed", 13.89), "shape": lk["shape"],
                        "length": lk["length"], "bearing": _bearing(shp) if len(shp) >= 2 else 0.0,
                        "internal_from": (net.edges.get(edge) or {}).get("from") in nodeset})
    return out


def signal_groups(links):
    """Dutch signal groups: per approach edge and mode, the links of lanes that turn left only (a protected left,
    its own group) and the rest (through, right, and lefts from a shared lane). Bikes: one group per approach.
    [{name, links: [k], mode, kind ("l" | "x"), lanes, speed, bearing, internal}]."""
    by_lane = {}
    for lk in links:
        by_lane.setdefault(lk["from"], []).append(lk)
    groups = {}
    for lane, lks in by_lane.items():
        mode = lks[0]["mode"]
        kind = "l" if mode == "car" and all(lk["turn"] == "l" for lk in lks) else "x"
        key = (lks[0]["edge"], mode, kind)
        g = groups.setdefault(key, {"edge": key[0], "mode": mode, "kind": kind, "links": [], "lanes": 0,
                                    "speed": 0.0, "bearing": lks[0]["bearing"],
                                    "internal": lks[0]["internal_from"]})
        g["links"] += [lk["k"] for lk in lks]
        g["lanes"] += 1
        g["speed"] = max(g["speed"], max(lk["speed"] for lk in lks))
    out = sorted(groups.values(), key=lambda g: (g["edge"], g["mode"], g["kind"]))
    for i, g in enumerate(out):
        g["name"] = f"{'fc' if g['mode'] == 'car' else g['mode']}{i + 1:02d}"
        g["links"].sort()
    return out


def split_groups(groups, links, lc):
    """A group whose lanes' paths cross (the net's lanes of one approach that swap sides inside the junction, where
    neither yields) is split into groups whose lanes do not: each lane joins the first part it conflicts with
    nowhere. Groups are renamed in order."""
    out = []
    for g in groups:
        lanes = {}
        for k in g["links"]:
            lanes.setdefault(links[k]["from"], []).append(k)
        parts = []
        for lane, ks in sorted(lanes.items()):
            for part in parts:
                if not any(lc.get((min(a, b), max(a, b))) == "full" for a in ks for b in part):
                    part.extend(ks)
                    break
            else:
                parts.append(list(ks))
        for part in parts:
            out.append({**g, "links": sorted(part), "lanes": len({links[k]["from"] for k in part})})
    for i, g in enumerate(out):
        g["name"] = f"{'fc' if g['mode'] == 'car' else g['mode']}{i + 1:02d}"
    return out


def make_stages(full, weights, tries=300, seed=1, edges=None):
    """Stages (lists of group indices) for a conflict matrix full[i][j]: greedy colouring of the groups in many
    orders (heaviest first, then shuffled with a fixed seed), each stage then filled with every group that conflicts
    with none of it; the fewest stages win, then the most green weight per stage, then the most approaches per
    stage (edges: each group's approach; opposite through streams together rather than one arm at a time)."""
    import random
    n = len(full)
    rng = random.Random(seed)
    base = sorted(range(n), key=lambda i: (-weights[i], -sum(full[i]), i))
    orders = [base, sorted(range(n), key=lambda i: (-sum(full[i]), -weights[i], i))]
    for _ in range(tries):
        o = list(base)
        rng.shuffle(o)
        orders.append(o)
    best, best_score = None, None
    for order in orders:
        stages = []
        for i in order:
            for st in stages:
                if not any(full[i][j] for j in st):
                    st.append(i)
                    break
            else:
                stages.append([i])
        for st in stages:
            for i in base:
                if i not in st and not any(full[i][j] for j in st):
                    st.append(i)
        uniq = []
        for st in stages:
            if set(st) not in [set(u) for u in uniq]:
                uniq.append(sorted(st))
        score = (len(uniq), -round(sum(weights[i] for st in uniq for i in st) / len(uniq), 6),
                 -sum(len({edges[i] for i in st}) for st in uniq) if edges else 0)
        if best_score is None or score < best_score:
            best, best_score = uniq, score
    return best


def _turn_dirs(lks):
    return {lk["turn"] for lk in lks}


def link_conflicts(net, links):
    """{(a, b): kind} for links a < b of the same node that cross or merge: "full" (they may not be green together)
    or "a" / "b" (that one may be green with the other as a 'g', yielding to it: SUMO's request matrix says it
    yields, and it is a turning stream, not a left from its own lane)."""
    out = {}
    by_node = {}
    for lk in links:
        by_node.setdefault(lk["node"], []).append(lk)
    for node, lks in by_node.items():
        reqs = net.junctions[node]["requests"]
        for i, a in enumerate(lks):
            for b in lks[i + 1:]:
                if a["from"] == b["from"]:
                    continue                                    # one lane's links diverge
                ra, rb = reqs.get(a["req"], (set(), set(), False)), reqs.get(b["req"], (set(), set(), False))
                foes = b["req"] in ra[1] or a["req"] in rb[1]
                if not (foes or a["to"] == b["to"] or J.crossing(a["shape"], b["shape"])):
                    continue
                kind = "full"
                if b["req"] in ra[0] and _may_yield(a, b):
                    kind = "a"
                elif a["req"] in rb[0] and _may_yield(b, a):
                    kind = "b"
                out[(a["k"], b["k"])] = kind
    return out


def _may_yield(a, b):
    """May stream a be green while it must give way to stream b (a 'deelconflict')? Turning streams yield (right turns
    to straight-on cyclists and pedestrians' parallel streams, lefts from a shared lane to oncoming traffic), cyclists
    yield to motor traffic when they turn; a left turn from a lane of its own is protected, straight streams never
    share a green with a crossing one."""
    if a["turn"] == "s":
        return False
    if a["mode"] == "car" and a.get("own_left"):
        return False
    return True


def design(net, nodes, cid="tls", inner=True, log=_log):
    """The Dutch programme of one controller. Returns {id, links, groups, stages, phases, cycle, max_cycle, type}:
    phases [{duration, state, minDur, maxDur, name}] for SUMO (state: one character per link index)."""
    links = controller_links(net, nodes)
    if not links:
        return None
    groups = signal_groups(links)
    gof = {}
    for gi, g in enumerate(groups):
        for k in g["links"]:
            gof[k] = gi
            links[k]["own_left"] = g["kind"] == "l"
    lc = link_conflicts(net, links)
    groups = split_groups(groups, links, lc)
    gof = {k: gi for gi, g in enumerate(groups) for k in g["links"]}
    n = len(groups)
    full = [[False] * n for _ in range(n)]
    for (a, b), kind in lc.items():
        ga, gb = gof[a], gof[b]
        if ga != gb and kind == "full":
            full[ga][gb] = full[gb][ga] = True
    # a junction of several nodes (dual carriageways): a stream that drives into the middle must be able to leave it,
    # so a group may not share a stage with what conflicts with the inner group it feeds (vehicles stood inside the
    # junction, blocking it: 8 teleports in 900 s at Kennedylaan/Fellenoord)
    feeds = []
    for gb, b in enumerate(groups if FEEDERS else []):
        if not b["internal"]:
            continue
        for ga, a in enumerate(groups):
            if ga != gb and any(links[k]["to"].rsplit("_", 1)[0] == b["edge"] for k in a["links"]):
                feeds.append((ga, gb))
    base = [row[:] for row in full]
    for ga, gb in feeds:
        for x in range(n):
            if x != ga and base[gb][x]:
                full[ga][x] = full[x][ga] = True

    # stages: groups that may be green together (make_stages), each filled with every group that conflicts with none
    # of it (a group may be green in several stages); ordered below for the least lost time
    def weight(g):
        return g["lanes"] * (2.0 if g["mode"] == "car" else 1.0) * (1.3 if g["speed"] >= MAIN_ROAD_SPEED else 1.0) \
            * (0.5 if g["internal"] else 1.0)
    stages = make_stages(full, [weight(g) for g in groups], edges=[g["edge"] for g in groups])
    ped_stage = len(stages) == 1          # a crossing on a straight road: a red for the road (pedestrians cross)
    if ped_stage:
        stages.append([])

    def state_in(stage):
        """Link states while a stage is green: 'G', or 'g' where the link yields to another green link."""
        green = {k for gi in stage for k in groups[gi]["links"]}
        st = ["r"] * len(links)
        for k in green:
            st[k] = "G"
        for (a, b), kind in lc.items():
            if a in green and b in green:
                if kind == "a":
                    st[a] = "g"
                elif kind == "b":
                    st[b] = "g"
        return st

    def g_amber(gi):
        g = groups[gi]
        turns = _turn_dirs([links[k] for k in g["links"]])
        return max(amber(g["mode"], t, g["speed"]) for t in turns)

    def clearance(ending, starting):
        """All-red (s) between the groups that end and those that start: the worst conflicting link pair."""
        worst = 0.0
        if not starting:
            return 0.0
        start_links = {k for gi in starting for k in groups[gi]["links"]}
        for gi in ending:
            for a in groups[gi]["links"]:
                for b in start_links:
                    key = (min(a, b), max(a, b))
                    if key not in lc:
                        continue
                    la, lb = links[a], links[b]
                    cp = J.crossing(la["shape"], lb["shape"], margin=0.0)
                    da, db = (cp if cp else (la["length"], lb["length"]))
                    v_out = EXIT["bike"] if la["mode"] == "bike" else EXIT["bus"] if la["mode"] == "bus" else \
                        EXIT["turn"] if la["turn"] != "s" else EXIT["car"]
                    t = (da + LENGTH[la["mode"]]) / v_out - db / ENTRY.get(lb["mode"], 6.0)
                    worst = max(worst, t)
        if worst <= 0.0:
            return 0.0
        return min(ALL_RED_MAX, round_half_up(worst))

    def trans_cost(a, b):
        ending, starting = set(a) - set(b), set(b) - set(a)
        return max((g_amber(gi) for gi in ending), default=0.0) + clearance(ending, starting)
    if 2 < len(stages) <= 6:              # the order with the least lost time, the heaviest stage first
        import itertools
        first = max(range(len(stages)), key=lambda i: sum(weight(groups[g]) for g in stages[i]))
        rest = [i for i in range(len(stages)) if i != first]
        best = min(itertools.permutations(rest), key=lambda perm: sum(
            trans_cost(stages[a], stages[b]) for a, b in zip((first,) + perm, perm + (first,))))
        stages = [stages[i] for i in (first,) + best]

    # transitions, lost time
    k_st = len(stages)
    trans = []
    for i in range(k_st):
        a, b = set(stages[i]), set(stages[(i + 1) % k_st])
        ending, starting, cont = a - b, b - a, a & b
        ambers = sorted({g_amber(gi) for gi in ending})
        trans.append({"ending": ending, "starting": starting, "cont": cont, "ambers": ambers,
                      "red": clearance(ending, starting)})
    lost = sum((t["ambers"][-1] if t["ambers"] else 0.0) + t["red"] for t in trans)
    main = any(groups[gi]["speed"] >= MAIN_ROAD_SPEED and groups[gi]["lanes"] >= 2 and groups[gi]["mode"] == "car"
               for gi in range(n))
    cap = CYCLE_MAX_MAIN if (main or not inner) else CYCLE_MAX_INNER
    cycle = min(cap, CYCLE_BY_STAGES.get(k_st, 95.0) + (15.0 if main else 0.0))
    # splits: each stage's green in proportion to its heaviest group that is green in no other stage
    weights = []
    for i, st in enumerate(stages):
        only = [gi for gi in st if sum(gi in s for s in stages) == 1] or st
        weights.append(max((weight(groups[gi]) for gi in only), default=1.0))
    if ped_stage:
        weights[-1] = 0.0
    mins = []
    for i, st in enumerate(stages):
        mins.append(max((MIN_GREEN[groups[gi]["mode"]] for gi in st), default=PED_STAGE) if st else PED_STAGE)
    avail = max(cycle - lost - (PED_STAGE if ped_stage else 0.0), sum(m for m, w in zip(mins, weights) if w))
    tot = sum(weights) or 1.0
    greens = [max(m, math.floor(avail * w / tot)) if w else m for m, w in zip(mins, weights)]
    cycle = sum(greens) + lost
    max_greens = list(greens)
    spare = max(0.0, cap - cycle)
    for i, w in enumerate(weights):
        if w:
            max_greens[i] = math.floor(min(greens[i] * MAX_GREEN_FACTOR, greens[i] + spare * w / tot))

    phases = []
    for i, st in enumerate(stages):
        s_green = state_in(st)
        actuated = bool(st) and max_greens[i] > greens[i]
        phases.append({"duration": float(greens[i]), "state": "".join(s_green),
                       "minDur": float(mins[i]) if actuated else None,
                       "maxDur": float(max_greens[i]) if actuated else None,
                       "name": "+".join(groups[gi]["name"] for gi in sorted(st)) or "pedestrians"})
        t = trans[i]
        ending_links = {k for gi in t["ending"] for k in groups[gi]["links"]}
        prev = 0.0
        for a in t["ambers"]:             # amber, in steps where the ending groups' ambers differ
            still = {k for gi in t["ending"] if g_amber(gi) >= a for k in groups[gi]["links"]}
            stt = []
            for k, c in enumerate(s_green):
                if k in still:
                    stt.append("y")
                elif k in ending_links:
                    stt.append("r")
                else:
                    stt.append(c)
            phases.append({"duration": round(a - prev, 1), "state": "".join(stt), "minDur": None, "maxDur": None,
                           "name": "amber"})
            prev = a
        if t["red"] > 0:
            stt = ["r" if (k in ending_links or c in "yY") else c for k, c in enumerate(s_green)]
            phases.append({"duration": t["red"], "state": "".join(stt), "minDur": None, "maxDur": None,
                           "name": "all-red"})
    phases = [p for p in phases if p["duration"] > 0]
    yields = sorted([a, b] if kind == "a" else [b, a] for (a, b), kind in lc.items() if kind != "full")
    return {"id": cid, "nodes": list(nodes), "links": links, "groups": groups, "yields": yields,
            "stages": [[groups[gi]["name"] for gi in st] for st in stages], "phases": phases,
            "cycle": round(sum(p["duration"] for p in phases), 1),
            "max_cycle": round(sum((p["maxDur"] or p["duration"]) for p in phases), 1),
            "type": "actuated" if any(p["maxDur"] for p in phases) else "static", "offset": 0.0}


def max_red(prog):
    """The longest time (s, at maximum greens) a group waits for its green: Eindhoven's maximum waits are 80 s for
    cyclists and pedestrians inside the Ring and 110 s for everything else (Nota 2024)."""
    out = {}
    ph = prog["phases"]
    durs = [p["maxDur"] or p["duration"] for p in ph]
    for g in prog["groups"]:
        k = g["links"][0]
        red = [p["state"][k] in "rR" for p in ph]
        if all(red):
            out[g["name"]] = float("inf")
            continue
        best, run = 0.0, 0.0
        for i in range(2 * len(ph)):
            if red[i % len(ph)]:
                run += durs[i % len(ph)]
                best = max(best, run)
            else:
                run = 0.0
        out[g["name"]] = best
    return out


# --- writing the net ---------------------------------------------------------------------------------------------

def tl_logic_xml(prog, program_id="dutch"):
    """A <tlLogic> element for a programme (design() or signaldata.program_from_log())."""
    el = ET.Element("tlLogic", {"id": prog["id"], "type": prog.get("type", "static"), "programID": program_id,
                                "offset": f"{prog.get('offset', 0.0):g}"})
    for p in prog["phases"]:
        at = {"duration": f"{p['duration']:g}", "state": p["state"]}
        if prog.get("type") == "actuated" and p.get("maxDur"):
            at["minDur"] = f"{p['minDur']:g}"
            at["maxDur"] = f"{p['maxDur']:g}"
        if p.get("name"):
            at["name"] = p["name"]
        ET.SubElement(el, "phase", at)
    return el


def write_net(src, dst, programs):
    """map.net.xml -> dst with the given controllers' programmes in place of every traffic light it had: the
    connections of each controller's links get its tl and linkIndex, its nodes become traffic_light junctions, all
    other tl attributes and tlLogics go. Nothing else changes."""
    tree = ET.parse(str(src))
    root = tree.getroot()
    for el in root.findall("tlLogic"):
        root.remove(el)
    key = {}
    for prog in programs:
        for lk in prog["links"]:
            f, fl = lk["from"].rsplit("_", 1)
            t, tl = lk["to"].rsplit("_", 1)
            key[(f, fl, t, tl)] = (prog["id"], lk["k"])
    nodes = {nd: prog["id"] for prog in programs for nd in prog["nodes"]}
    first_junction = None
    for i, el in enumerate(list(root)):
        if el.tag == "connection":
            el.attrib.pop("tl", None)
            el.attrib.pop("linkIndex", None)
            hit = key.get((el.get("from"), el.get("fromLane"), el.get("to"), el.get("toLane")))
            if hit and not el.get("from", "").startswith(":"):
                el.set("tl", hit[0])
                el.set("linkIndex", str(hit[1]))
        elif el.tag == "junction":
            if first_junction is None:
                first_junction = i
            if el.get("id") in nodes and el.get("type") != "internal":
                el.set("type", "traffic_light")
            elif el.get("type") == "traffic_light":
                el.set("type", "priority")
    at = first_junction if first_junction is not None else len(root)
    for prog in reversed(programs):
        root.insert(at, tl_logic_xml(prog))
    ET.indent(tree, space="    ")
    Path(dst).parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(dst), encoding="UTF-8", xml_declaration=True)


# --- the hook ------------------------------------------------------------------------------------------------------

def mode_for(cfg, bbox, data_ok):
    """The traffic.signals mode a trial runs: "sumo", "dutch" or "real"."""
    want = str((cfg.get("traffic") or {}).get("signals", "auto")).lower()
    if want in ("sumo", "dutch", "real"):
        return want
    if data_ok:
        return "real"
    from .speeds import country_of
    try:
        nl = country_of(cfg, bbox) == "NL"
    except Exception:  # noqa: BLE001
        nl = False
    return "dutch" if nl else "sumo"


def _xy_of(net_path):
    """(lat, lon) -> the net's x, y (its <location>: UTM projection and offset), without sumolib."""
    loc = None
    for _, el in ET.iterparse(str(net_path), events=("end",)):
        if el.tag == "location":
            loc = dict(el.attrib)
            break
    if loc is None:
        raise ValueError("no <location> in the net")
    ox, oy = (float(v) for v in loc["netOffset"].split(","))
    proj = loc.get("projParameter", "")
    import pyproj
    tr = pyproj.Transformer.from_proj(pyproj.Proj("+proj=longlat +datum=WGS84"), pyproj.Proj(proj),
                                      always_xy=True) if proj and proj != "!" else None

    def xy(lat, lon):
        x, y = tr.transform(lon, lat)
        return x + ox, y + oy
    return xy


def _xy_of_sumolib(net_path):
    import sumolib
    net = sumolib.net.readNet(str(net_path), withInternal=False, withPedestrianConnections=False)

    def xy(lat, lon):
        return net.convertLonLat2XY(lon, lat)
    return xy


def plan(map_dir, bbox, inner=True, fetch=True, log=_log, xy_of=None):
    """Controllers and Dutch programmes of a map: (programmes, summary). Cached in map_dir/signals/plan.json."""
    map_dir = Path(map_dir)
    net_path = map_dir / "map.net.xml"
    net = J.read_net(net_path)
    locs = locations(bbox, map_dir / "real", fetch=fetch, log=log)
    if xy_of is None and locs:
        try:
            xy_of = _xy_of(net_path)
        except Exception:  # noqa: BLE001 - no pyproj: sumolib's projection
            xy_of = _xy_of_sumolib(net_path)
    ctl = controllers(net, locs, xy_of, log)
    progs, skipped = [], 0
    for c in ctl:
        p = design(net, c["nodes"], c["id"], inner=inner, log=log)
        if p is None:
            skipped += 1
            continue
        p.update({k: c[k] for k in ("name", "vri", "ivri", "tlc_id", "location", "source")})
        progs.append(p)
    inside = [loc for loc in locs if bbox[0] <= loc["lat"] <= bbox[2] and bbox[1] <= loc["lon"] <= bbox[3]]
    matched = {p["location"] for p in progs if p["location"]}
    summary = {"controllers": len(progs), "matched": sum(p["source"] == "matched" for p in progs),
               "added": sum(p["source"] == "added" for p in progs), "osm_only": sum(p["source"] == "osm" for p in progs),
               "listed_in_box": len(inside), "listed_matched": sum(loc["id"] in matched for loc in inside),
               "listed_unmatched": sorted(loc["name"] for loc in inside if loc["id"] not in matched),
               "ivri": sorted(p["ivri"] for p in progs if p["ivri"]),
               "links": sum(len(p["links"]) for p in progs), "skipped": skipped,
               "old_lights": len(net.tls)}
    return progs, summary


def tls_states_additional(net_file, work):
    """[an additional file that makes SUMO write every light's state changes to tls_states.xml] (SUMO's own record,
    to check the drawn heads and analyse a trial), or [] when the net has no lights."""
    ids = []
    for _, el in ET.iterparse(str(net_file), events=("end",)):
        if el.tag == "tlLogic":
            ids.append(el.get("id"))
        elif el.tag == "junction":
            break
    if not ids:
        return []
    Path(work).mkdir(parents=True, exist_ok=True)
    root = ET.Element("additional")
    for tid in sorted(set(ids)):          # one event per light (source takes one id), all into one file
        ET.SubElement(root, "timedEvent", {"type": "SaveTLSSwitchStates", "source": tid, "dest": "tls_states.xml"})
    ET.ElementTree(root).write(str(Path(work) / "tls_states.add.xml"), encoding="UTF-8", xml_declaration=True)
    return ["tls_states.add.xml"]


def _key(map_dir, cfg_part):
    st = (Path(map_dir) / "map.net.xml").stat()
    raw = json.dumps([VERSION, FEEDERS, st.st_size, int(st.st_mtime), cfg_part], sort_keys=True)
    return hashlib.sha1(raw.encode()).hexdigest()[:12]


def _public(prog):
    """A programme without its link geometry (plan.json)."""
    return {**{k: v for k, v in prog.items() if k != "links"},
            "links": [{k: v for k, v in lk.items() if k not in ("shape",)} for lk in prog["links"]]}


def prepare(cfg, map_dir, work, when=None, log=_log):
    """traffic.prepare's hook: (net file for the trial or None for map.net.xml, [additional files], info dict).
    "sumo": nothing changes. "dutch"/"real": map_dir/signals/<key>.net.xml (cached) with the controllers' Dutch
    programmes; "real" then replaces the programmes of controllers the phase data covers (signaldata.py)."""
    from . import signaldata
    map_dir, work = Path(map_dir), Path(work)
    bbox = json.loads((map_dir / "meta.json").read_text())["bbox"]
    sd = dict((cfg.get("traffic") or {}).get("signal_data") or {})
    data = signaldata.Source(sd, cfg.get("_root"), log=log)
    mode = mode_for(cfg, bbox, data.available())
    info = {"mode": mode}
    if mode == "sumo":
        log("traffic lights: SUMO's (netconvert's programmes)")
        return None, tls_states_additional(map_dir / "map.net.xml", work), info
    inner = sd.get("inner_ring", "auto")
    key = _key(map_dir, {"inner": inner})
    out_dir = map_dir / "signals"
    net_out, plan_out = out_dir / f"{key}.net.xml", out_dir / f"{key}.plan.json"
    if net_out.exists() and plan_out.exists():
        saved = json.loads(plan_out.read_text())
        progs, summary = saved["programs"], saved["summary"]
        net = None
    else:
        progs, summary = plan(map_dir, bbox, inner=inner is not False, log=log)
        if not progs:
            log("traffic lights: no signalised junction found; SUMO's programmes")
            return None, [], {"mode": "sumo"}
        write_net(map_dir / "map.net.xml", net_out, progs)
        out_dir.mkdir(parents=True, exist_ok=True)
        plan_out.write_text(json.dumps({"summary": summary, "programs": [_public(p) for p in progs]}, indent=0))
        import time as _time
        for old in out_dir.glob("*.net.xml"):          # older designs of this map (a week old: trials may run them)
            if old.name != net_out.name and _time.time() - old.stat().st_mtime > 7 * 86400:
                old.unlink()
                (out_dir / old.name.replace(".net.xml", ".plan.json")).unlink(missing_ok=True)
        net = True
    info.update(summary)
    info["net"] = str(net_out)
    log(f"traffic lights (Dutch): {summary['controllers']} junctions ({summary['matched']} matched to the city's "
        f"list, {summary['added']} added from it, {summary['osm_only']} from OSM only; {summary['listed_matched']} of "
        f"{summary['listed_in_box']} listed junctions in the box), {summary['links']} links, replacing "
        f"{summary['old_lights']} netconvert lights" + ("" if net else " (cached)"))
    additional = tls_states_additional(net_out, work)
    if mode == "real":
        add, rinfo = data.programs(progs, work, when, log)
        info["real"] = rinfo
        if add:
            additional.append(add)
    info["driver"] = data.driver_spec(when)
    (work / "signals.json").parent.mkdir(parents=True, exist_ok=True)
    (work / "signals.json").write_text(json.dumps(info, indent=1, default=str))
    return net_out, additional, info
