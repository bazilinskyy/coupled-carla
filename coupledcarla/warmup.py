"""Streets already busy at the trial's start: the SUMO demand as if it had been running for `traffic.warmup_s`.

Without it SUMO starts empty: its trips enter from t = 0 and take minutes to reach the middle of the map, so the
first minute of every trial and demo clip had almost no traffic (the Eindhoven drone clip, filmed at 25-45 s, showed
none: 11 cars and 6 bikes on the whole twin at 30 s). Clips held the driver in CARLA for 20-100 s to fill the
streets, at CARLA's speed.

prefill() rewrites the route files made for [0, end + warmup] so that they describe [-warmup, end]: every departure
is moved warmup seconds earlier; a vehicle that would have left before t = 0 is put where it would be at t = 0 (the
route edge it would have reached driving at `speed_share` of each edge's limit, the bikes at most at bike_mps,
departEdge, at a random free place on it, at once) or left out when it would have arrived already. Buses on their
lines (pt*, with stops) keep their timetable. Pure (no SUMO run): the net is read with sumolib.
"""
import xml.etree.ElementTree as ET

SPEED_SHARE = 0.6     # mean speed / limit in town (signals, junctions, queues)
BIKE_MPS = 4.5        # a cyclist's mean speed with stops (16 km/h moving)
KEEP_CLASSES_OUT = ("bus",)


def travel_times(edges, net, bike=False, speed_share=SPEED_SHARE):
    """Cumulative time (s) at the end of each route edge."""
    out, t = [], 0.0
    for e in edges:
        try:
            edge = net.getEdge(e)
            length, vmax = edge.getLength(), edge.getSpeed()
        except KeyError:
            length, vmax = 50.0, 13.9
        v = min(vmax * speed_share, BIKE_MPS) if bike else vmax * speed_share
        t += length / max(v, 1.0)
        out.append(t)
    return out


def _fmt(t):
    return f"{t:.2f}"


def prefill(path, net, warmup_s, bike_types=(), seed=0):
    """Shifts the route file at `path` by warmup_s and places the vehicles that departed before t = 0. Returns
    (vehicles placed at t = 0, vehicles left out as arrived, vehicles shifted)."""
    import random
    rng = random.Random(seed)
    tree = ET.parse(path)
    root = tree.getroot()
    routes = {r.get("id"): r for r in root.findall("route") if r.get("id")}
    vclass = {}
    for vt in root.iter("vType"):
        vclass[vt.get("id")] = vt.get("vClass", "passenger")
    placed = dropped = shifted = 0
    kept = []
    for el in list(root):
        if el.tag not in ("vehicle", "trip"):
            kept.append(el)
            continue
        vid = el.get("id", "")
        try:
            dep = float(el.get("depart"))
        except (TypeError, ValueError):
            kept.append(el)
            continue
        cls = vclass.get(el.get("type"), "passenger")
        if vid.startswith("pt") or el.find("stop") is not None or cls in KEEP_CLASSES_OUT:
            kept.append(el)             # a bus on its line: its timetable stays
            continue
        t0 = dep - warmup_s
        if t0 >= 0:
            el.set("depart", _fmt(t0))
            kept.append(el)
            shifted += 1
            continue
        if el.tag == "trip":            # no route to place it on: it departs at once from its start
            el.set("depart", "0.00")
            kept.append(el)
            placed += 1
            continue
        r = el.find("route")
        edges = (r.get("edges") if r is not None else (routes.get(el.get("route")) or {}).get("edges", "")) or ""
        edges = edges.split()
        if not edges:
            dropped += 1
            continue
        bike = cls == "bicycle" or el.get("type") in bike_types or str(el.get("type", "")).startswith("bike")
        times = travel_times(edges, net, bike)
        age = -t0
        idx = next((i for i, t in enumerate(times) if t > age), None)
        if idx is None:
            dropped += 1                # it would have arrived before t = 0
            continue
        el.set("depart", _fmt(rng.uniform(0.0, 1.0)))
        if idx > 0:
            el.set("departEdge", str(idx))
        el.set("departPos", "random_free")
        el.set("departSpeed", "max")
        el.set("departLane", "best")
        kept.append(el)
        placed += 1
    # SUMO wants the vehicles sorted by departure; definitions (vTypes, routes) first
    defs = [e for e in kept if e.tag not in ("vehicle", "trip")]
    vehs = sorted((e for e in kept if e.tag in ("vehicle", "trip")), key=lambda e: float(e.get("depart", 0)))
    for e in list(root):
        root.remove(e)
    for e in defs + vehs:
        root.append(e)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return placed, dropped, shifted


def warmup_for(tc, duration_s):
    """traffic.warmup_s: a number, or "auto" (600 s, the time a trip takes to cross a city-centre map), 0: off."""
    w = (tc.get("traffic") or {}).get("warmup_s", "auto")
    if w in (None, False, "off"):
        return 0
    return 600 if w == "auto" else max(0, int(float(w)))
