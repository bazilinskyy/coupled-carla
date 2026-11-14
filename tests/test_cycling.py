"""Cyclists along the cycle network at counted volumes (cycling.py): OSM parsing, counter profiles and fetchers,
calibration, trip-end weights and the bike graph (no SUMO, no network)."""
import json
import math
import xml.etree.ElementTree as ET
from datetime import date, datetime

from coupledcarla import cycling as C


# --- a tiny stand-in for a sumolib net: straight edges between nodes, coordinates in metres -------------------

class _Lane:
    def __init__(self, ok):
        self.ok = ok

    def allows(self, vclass):
        return self.ok


class _Conn:
    def __init__(self, a, b):
        self.a, self.b = a, b

    def getFromLane(self):
        return _Lane(self.a.bike)

    def getToLane(self):
        return _Lane(self.b.bike)


class _Node:
    def __init__(self, xy):
        self.xy = xy
        self.inc = []

    def getCoord(self):
        return self.xy

    def getIncoming(self):
        return self.inc


class _Edge:
    def __init__(self, eid, a, b, bike=True, speed=13.9):
        self.id, self.a, self.b, self.bike, self.speed = eid, a, b, bike, speed
        self.out, self.inc = {}, []

    def getID(self):
        return self.id

    def getShape(self):
        return [self.a.xy, self.b.xy]

    def getLength(self):
        return math.dist(self.a.xy, self.b.xy)

    def getSpeed(self):
        return self.speed

    def allows(self, vclass):
        return self.bike

    def getOutgoing(self):
        return self.out

    def getIncoming(self):
        return self.inc

    def getFromNode(self):
        return self.a

    def getToNode(self):
        return self.b


class _Net:
    """Edges (id, (x0, y0), (x1, y1)); each edge connects to every edge leaving its end node."""

    def __init__(self, edges, bike_only=None):
        nodes, self.edges = {}, {}
        for eid, a, b, *rest in edges:
            na, nb = nodes.setdefault(a, _Node(a)), nodes.setdefault(b, _Node(b))
            self.edges[eid] = _Edge(eid, na, nb, bike=rest[0] if rest else True)
        for e in self.edges.values():
            e.b.inc.append(e)
            for f in self.edges.values():
                if f.a is e.b and f is not e:
                    e.out[f] = [_Conn(e, f)]
                    f.inc.append(e)

    def getEdges(self):
        return list(self.edges.values())

    def getEdge(self, eid):
        return self.edges[eid]

    def convertLonLat2XY(self, lon, lat):
        return lon, lat

    def getNeighboringEdges(self, x, y, r):
        out = []
        for e in self.edges.values():
            (ax, ay), (bx, by) = e.getShape()
            dx, dy = bx - ax, by - ay
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
            d = math.hypot(ax + t * dx - x, ay + t * dy - y)
            if d <= r:
                out.append((e, d))
        return out


def _grid():
    """A two-way street a-b-c along y = 0 (100 m edges), a one-way side street up from b, a parallel cycle track
    along y = 8, and an isolated two-way cycle path far away."""
    return _Net([("ab", (0, 0), (100, 0)), ("ba", (100, 0), (0, 0)), ("bc", (100, 0), (200, 0)),
                 ("cb", (200, 0), (100, 0)), ("bd", (100, 0), (100, 100)),
                 ("t1", (0, 8), (200, 8)), ("t2", (200, 8), (0, 8)),
                 ("iso1", (500, 500), (600, 500)), ("iso2", (600, 500), (500, 500))])


# --- OSM ----------------------------------------------------------------------------------------------------------

def test_infra_kind():
    assert C.infra_kind({"highway": "cycleway"}) == "cycleway"
    assert C.infra_kind({"highway": "path", "bicycle": "designated"}) == "cycleway"
    assert C.infra_kind({"highway": "primary", "cycleway:right": "track"}) == "track"
    assert C.infra_kind({"highway": "residential", "cycleway:both": "lane"}) == "lane"
    assert C.infra_kind({"highway": "residential", "cyclestreet": "yes"}) == "cyclestreet"
    assert C.infra_kind({"highway": "residential", "cycleway": "no"}) is None


def test_parse_routes_and_infra_with_clipped_geometry():
    geom = [{"lat": 51.0, "lon": 5.0}, {"lat": 51.0, "lon": 5.002}, None, {"lat": 51.0, "lon": 5.006},
            {"lat": 51.0, "lon": 5.009}]
    data = {"elements": [
        {"type": "relation", "id": 1, "tags": {"route": "bicycle", "network": "rcn", "network:type": "node_network",
                                               "ref": "12-34"},
         "members": [{"type": "way", "role": "", "geometry": geom},
                     {"type": "way", "role": "alternative", "geometry": geom},
                     {"type": "node", "role": "", "lat": 51.0, "lon": 5.0}]},
        {"type": "way", "id": 2, "tags": {"highway": "cycleway"}, "geometry": geom},
        {"type": "way", "id": 3, "tags": {"highway": "residential"}, "geometry": geom}]}
    routes, infra = C.parse(data)
    (r,) = routes
    assert r["network"] == "rcn" and r["node_network"] and r["ref"] == "12-34"
    assert len(r["pieces"]) == 2                   # split where the way leaves the box; the alternative is ignored
    (w,) = infra
    assert w["kind"] == "cycleway" and len(w["pieces"]) == 2


# --- counters -----------------------------------------------------------------------------------------------------

def test_profile_by_day_type_and_broken_days():
    mon, tue, sat, dead = date(2026, 11, 9), date(2026, 11, 10), date(2026, 11, 14), date(2026, 11, 11)
    hourly = {(mon, 8): 100, (tue, 8): 200, (mon, 9): 50, (sat, 8): 30, (dead, 8): 0, (dead, 9): 0}
    prof, days = C.profile(hourly)
    assert prof["weekday"][8] == 150 and prof["weekday"][9] == 50 and prof["weekday"][7] is None
    assert prof["saturday"][8] == 30 and prof["sunday"][8] is None
    assert days == {"weekday": 2, "saturday": 1, "sunday": 0}      # the all-zero day is left out
    assert C.typical({"profile": prof}, "weekday", 8) == 150


def test_near_box():
    box = [51.0, 5.0, 51.01, 5.01]
    assert C._near(box, 51.005, 5.005)
    assert C._near(box, 51.012, 5.005)                # ~220 m north
    assert not C._near(box, 51.02, 5.005)


def test_nyc_picks_one_flow_per_direction(monkeypatch):
    sensors = [{"id": "7", "name": "8th Ave", "lat": "51.005", "lon": "5.005", "travelmodes": "bike",
                "lastdata": "2026-11-13T23:45:00.000"},
               {"id": "8", "name": "far", "lat": "52", "lon": "6", "travelmodes": "bike",
                "lastdata": "2026-11-13T23:45:00.000"}]
    rows = [{"sensor_id": "7", "flowid": "a", "direction": "in", "d": "2026-11-10T00:00:00.000", "h": "8", "n": "40"},
            {"sensor_id": "7", "flowid": "b", "direction": "in", "d": "2026-11-10T00:00:00.000", "h": "8", "n": "0"},
            {"sensor_id": "7", "flowid": "c", "direction": "out", "d": "2026-11-10T00:00:00.000", "h": "8", "n": "60"},
            {"sensor_id": "7", "flowid": "d", "direction": "out", "d": "2026-11-10T00:00:00.000", "h": "8",
             "n": "55"}]

    class R:
        def __init__(self, data):
            self.data = data

        def json(self):
            return self.data
    calls = []

    def get(url, params=None, **k):
        calls.append(params)
        return R(sensors if "6up2" in url else rows)
    monkeypatch.setattr(C, "_get", get)
    (site,) = C._nyc([51.0, 5.0, 51.01, 5.01], datetime(2026, 11, 14))
    assert site["profile"]["weekday"][8] == 100          # in: a (40); out: c (60), not c + d
    assert "'7'" in calls[1]["$where"] and "'8'" not in calls[1]["$where"]


def test_flanders_reads_bike_rows_of_near_sites(monkeypatch):
    sites = "300025593,300025593,5.005,51.005,Leuven_2,AWV,,,Leuven,15,2019\n1,1,3.0,50.8,far,AWV,,,X,15,2019\n"
    data = ("300025593,IN,FIETSERS,2026-11-10 08:00:00.0,2026-11-10 08:15:00.0,10\n"
            "300025593,OUT,FIETSERS,2026-11-10 08:15:00.0,2026-11-10 08:30:00.0,15\n"
            "300025593,IN,VOETGANGERS,2026-11-10 08:00:00.0,2026-11-10 08:15:00.0,99\n"
            "1,IN,FIETSERS,2026-11-10 08:00:00.0,2026-11-10 08:15:00.0,99\n").encode()

    class R:
        text = sites

        def iter_lines(self):
            return iter(data.splitlines())

    def get(url, *a, **k):
        if "data-2026-10" in url:
            raise C.requests.HTTPError("404")
        return R()
    monkeypatch.setattr(C, "_get", get)
    (site,) = C._flanders([51.0, 5.0, 51.01, 5.01], datetime(2026, 11, 14))
    assert site["name"] == "Leuven_2" and site["profile"]["weekday"][8] == 25


def test_counters_cached_and_fail_soft(tmp_path, monkeypatch):
    def broken(bbox, now):
        raise OSError("offline")
    site = {"source": "x", "id": "1", "name": "n", "lat": 51.0, "lon": 5.0, "kind": "section",
            "profile": {}, "days": {}}
    monkeypatch.setattr(C, "FETCHERS", [("lyon", (50, 4, 52, 6), lambda bbox, now: [site])])
    monkeypatch.setitem(C.SOURCES, "lyon", ("what", "licence"))
    assert C.counters([51, 5, 51.01, 5.01], tmp_path) == [site]
    assert json.loads((tmp_path / "bike_counters.json").read_text())["sources"][0]["licence"] == "licence"
    monkeypatch.setattr(C, "FETCHERS", [("lyon", (50, 4, 52, 6), broken)])
    assert C.counters([51, 5, 51.01, 5.01], tmp_path) == [site]          # from the cache
    (tmp_path / "bike_counters.json").unlink()
    assert C.counters([51, 5, 51.01, 5.01], tmp_path) == []              # offline: nothing, no crash, no cache
    assert not (tmp_path / "bike_counters.json").exists()
    assert C.counters([10, 10, 10.01, 10.01], tmp_path) == []            # no source covers the box


def test_counter_edges():
    net = _grid()
    # on the street: both directions of a-b, and the parallel cycle track 8 m away; not the side street
    assert C.counter_edges(net, {"lat": 3, "lon": 50}) == ["ab", "ba", "t1", "t2"]
    assert C.counter_edges(net, {"lat": 60, "lon": 101}) == ["bd"]
    # 40 m beyond the end of the side street (the net stops short of it): the street that runs on toward it
    assert C.counter_edges(net, {"lat": 140, "lon": 100}) == ["bd"]
    assert C.counter_edges(net, {"lat": 140, "lon": 300}) == []           # nothing points at it
    assert C.counter_edges(net, {"lat": 2, "lon": 102, "kind": "junction"}) == ["ab", "cb"]


# --- trial demand -------------------------------------------------------------------------------------------------

def test_bike_component_starts_and_ends():
    net = _Net([("in", (-100, 0), (0, 0)), ("ab", (0, 0), (100, 0)), ("ba", (100, 0), (0, 0)),
                ("out", (100, 0), (200, 0)), ("car", (0, 0), (0, -100), False),
                ("iso", (500, 500), (600, 500))])
    comp, starts, ends = C.bike_component(net)
    assert comp == {"ab", "ba"}
    assert "in" in starts and "out" not in starts and "out" in ends and "in" not in ends
    assert "iso" not in starts | ends


def test_od_weights_and_inflow():
    net = _Net([("in", (-100, 0), (0, 0)), ("ab", (0, 0), (100, 0)), ("ba", (100, 0), (0, 0)),
                ("out", (100, 0), (200, 0))])
    comp, starts, ends = C.bike_component(net)
    src, dst = C.od_weights(net, ["ab"], [], 10, (starts, ends))
    assert src == {"in": 3.0, "ab": 10.0, "ba": 1.0}            # fringe x3, cycle route x10, "out" only leaves
    assert dst == {"ab": 10.0, "ba": 1.0, "out": 3.0}
    rate = C.set_inflow([src], {"in": 100.0}, 300.0)
    assert rate == 300 and abs(src["in"] / sum(src.values()) - 100 / 300) < 1e-9
    assert C.set_inflow([dict(src)], {"in": 400.0}, 300.0) == 800        # the counted edge takes at most half


def test_costs_cut_cycle_routes(tmp_path):
    C.write_costs(_grid(), tmp_path / "c.xml", ["ab"], ["t1"], 0.5, 0.7, 360)
    tt = {e.get("id"): float(e.get("traveltime")) for e in ET.parse(tmp_path / "c.xml").getroot().iter("edge")}
    free = 100 / C.BIKE_SPEED
    assert abs(tt["ab"] - free) < 0.01                          # cycle route at free flow ...
    assert abs(tt["ba"] - 2 * free) < 0.01                      # ... a street twice that (0.5) ...
    assert abs(tt["t1"] - 2 * 0.7 * 200 / C.BIKE_SPEED) < 0.01  # ... a cycle track 1.4 times


def test_calibrate_median_and_share():
    routes = [("v%d" % i, ["ab", "bc"]) for i in range(20)] + [("w%d" % i, ["ba"]) for i in range(80)]
    rate, per = C.calibrate(routes, 100, [(["ab"], 50.0), (["ba", "zz"], 400.0), (["zz"], 9.0)])
    assert per == [250.0, 500.0, None]                          # 20% and 80% pass; nobody passes "zz"
    assert rate == 375.0
    assert C.calibrate(routes, 200, [(["ab"], 50.0)])[0] == 1000.0   # half the trips failed to route
    assert C.calibrate([], 100, [(["ab"], 50.0)]) == (None, [])
    net = _grid()
    assert abs(C.km_share(routes, net, ["ab", "bc"]) - 0.2 * 200 / (0.2 * 200 + 0.8 * 100)) < 1e-9


def test_merge_routes_keeps_types_once(tmp_path):
    a, b = tmp_path / "a.rou.xml", tmp_path / "b.rou.xml"
    a.write_text('<routes><vType id="bike1" vClass="bicycle"/><vehicle id="x1" depart="5.00"><route edges="e"/>'
                 '</vehicle></routes>')
    b.write_text('<routes><vType id="bike1" vClass="bicycle"/><vehicle id="y1" depart="2.00"><route edges="f"/>'
                 '</vehicle></routes>')
    root = ET.parse(C.merge_routes([a, b], tmp_path / "m.rou.xml")).getroot()
    assert [el.tag for el in root] == ["vType", "vehicle", "vehicle"]
    assert [el.get("id") for el in root.iter("vehicle")] == ["y1", "x1"]
    assert C.routes_of([tmp_path / "m.rou.xml"]) == [("y1", ["f"]), ("x1", ["e"])]


def test_demand_off_or_failing_falls_back(tmp_path):
    cfg = {"traffic": {"bikes_per_hour": 300, "seed": 1, "cycling": {"routes": False, "counts": False}},
           "simulation": {}}
    assert C.demand(cfg, tmp_path, tmp_path, tmp_path / "t.xml", 360) is None
    cfg["traffic"]["cycling"] = {"routes": "auto", "counts": "auto"}
    cfg["traffic"]["bikes_per_hour"] = 0
    assert C.demand(cfg, tmp_path, tmp_path, tmp_path / "t.xml", 360) is None
    cfg["traffic"]["bikes_per_hour"] = 300
    lines = []
    assert C.demand(cfg, tmp_path / "nomap", tmp_path, tmp_path / "t.xml", 360, log=lines.append) is None
    assert len(lines) == 1 and "random bike trips" in lines[0]
