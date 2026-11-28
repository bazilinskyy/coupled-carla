"""The bike window (coupledcarla/bikewindow.py): the place's share of cyclists around the drivers. No SUMO, no CARLA:
a stand-in TraCI and actors."""
import math

from coupledcarla import bikewindow as bw
from coupledcarla.crowdwindow import RouteIndex


def test_settings_and_target():
    tc = {"traffic": {"_bikes_per_motor": 0.226, "bikes_per_hour": 900}, "participants": [{"role": "driver"}]}
    s = bw.settings(tc)
    assert s["enabled"] and s["ratio"] == 0.226
    assert not bw.settings({**tc, "participants": [{"role": "p", "kind": "pedestrian"}]})["enabled"]
    assert not bw.settings({"traffic": {"bikes_per_hour": 900}, "participants": [{"role": "driver"}]})["enabled"]
    off = {"traffic": {**tc["traffic"], "cycling": {"window": {"enabled": False}}}, "participants": tc["participants"]}
    assert not bw.settings(off)["enabled"]
    fixed = {"traffic": {"bikes_per_hour": 900, "cycling": {"window": {"ratio": 0.5}}}, "participants": [{}]}
    assert bw.settings(fixed)["ratio"] == 0.5 and bw.settings(fixed)["enabled"]
    assert bw.target(0.226, 20) == 5 and bw.target(0.226, 2) == 0 and bw.target(0.226, 3) == 1
    assert bw.target(None, 20) == 0


def _anchor_on_route():
    ri = RouteIndex([(0.0, 0.0), (400.0, 0.0)])
    return {"x": 0.0, "y": 0.0, "yaw": 0.0, "ri": ri, "s": 0.0}


def test_zones_along_a_route_and_around_a_car():
    cfg = dict(bw.DEFAULTS)
    an = _anchor_on_route()
    assert bw.in_zone(100.0, 20.0, an, cfg) and not bw.in_zone(100.0, 60.0, an, cfg)     # band 40 m
    assert bw.in_zone(-20.0, 0.0, an, cfg) and not bw.in_zone(-60.0, 0.0, an, cfg)       # 30 m behind
    assert not bw.in_zone(300.0, 0.0, an, cfg)                                            # beyond 250 m ahead
    assert bw.in_zone(150.0, 0.0, an, cfg, spawn=True) and not bw.in_zone(40.0, 0.0, an, cfg, spawn=True)
    free = {"x": 0.0, "y": 0.0, "yaw": 90.0, "ri": None}
    assert bw.in_zone(0.0, -120.0, free, cfg) and not bw.in_zone(0.0, -120.0, free, cfg, spawn=True)  # behind
    assert bw.in_zone(0.0, 120.0, free, cfg, spawn=True) and not bw.in_zone(0.0, 200.0, free, cfg)


def test_candidates_keep_away_from_viewers():
    cfg = dict(bw.DEFAULTS)
    lanes = [("a_0", "a", 0, 50.0, 100.0, 10.0, True), ("b_0", "b", 0, 50.0, 200.0, -10.0, False),
             ("c_0", "c", 0, 50.0, 30.0, 0.0, True), ("d_0", "d", 0, 50.0, 100.0, 90.0, True)]
    an = _anchor_on_route()
    assert bw.candidates(lanes, an, cfg) == [0, 1]
    assert bw.candidates(lanes, an, cfg, viewers=[(110.0, 0.0)]) == [1]      # a film camera stands there


class Actor:
    def __init__(self, x, y, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw

    def get_transform(self):
        class L:
            pass
        tf, loc, rot = L(), L(), L()
        loc.x, loc.y, loc.z = self.x, self.y, 0.0
        rot.yaw = self.yaw
        tf.location, tf.rotation = loc, rot
        return tf


class FakeTraci:
    def __init__(self):
        self.vehicles, self.routes = {}, {}
        tr = self

        class Vehicle:
            @staticmethod
            def getIDList():
                return list(tr.vehicles)

            @staticmethod
            def add(vid, rid, typeID, departLane, departPos, departSpeed):
                tr.vehicles[vid] = {"route": tr.routes[rid], "type": typeID, "lane": departLane, "pos": departPos}

            @staticmethod
            def setColor(vid, c):
                tr.vehicles[vid]["color"] = c

            @staticmethod
            def getPosition(vid):
                return tr.vehicles[vid].get("xy", (100.0, 0.0))

            @staticmethod
            def remove(vid):
                tr.vehicles.pop(vid)

        class Route:
            @staticmethod
            def add(rid, edges):
                tr.routes[rid] = edges

        class Sim:
            @staticmethod
            def findRoute(a, b, t):
                class S:
                    edges = [a, "x", b]
                return S()
        self.vehicle, self.route, self.simulation = Vehicle, Route, Sim


def test_window_adds_bikes_to_the_place_share_and_takes_them_off():
    lanes = [(f"e{i}_0", f"e{i}", 0, 60.0, 80.0 + 20.0 * i, 15.0, i % 2 == 0) for i in range(8)] + \
            [("far_0", "far", 0, 60.0, 900.0, 0.0, True)]
    car = Actor(0.0, 0.0)
    poses = {i: (60.0 + 10 * i, 0.0) for i in range(20)}             # 20 motor vehicles ahead on the route
    ids = {i: f"car{i}" for i in range(20)}
    tr = FakeTraci()
    cfg = bw.settings({"traffic": {"_bikes_per_motor": 0.226, "bikes_per_hour": 900}, "participants": [{}]})
    w = bw.BikeWindow([{"actor": car, "route": lambda: [(0.0, 0.0), (400.0, 0.0)]}], lanes, cfg,
                      {"vehicle.gazelle.omafiets": 4, "vehicle.bh.crossbike": 1}, lambda: [], poses, lambda: ids,
                      log=lambda m: None, seed=3, traci=tr)
    w.step(0.0)
    assert len(tr.vehicles) == 4                                      # at most 4 a check
    w.step(1.0)                                                       # within check_s: nothing
    assert len(tr.vehicles) == 4
    w.step(2.0)
    assert len(tr.vehicles) == 5                                      # 0.226 x 20 -> 5 (none shown in CARLA yet)
    v = next(iter(tr.vehicles.values()))
    assert v["route"][0].startswith("e") and v["route"][-1] == "far" and v["type"].startswith("vehicle.")
    assert len(v["color"]) == 4
    # once shown in CARLA near the car, they count: no more are added
    for k, vid in enumerate(list(tr.vehicles)):
        poses[100 + k] = (120.0, 5.0)
        ids[100 + k] = vid
    w.step(4.0)
    assert len(tr.vehicles) == 5 and w.summary()["added"] == 5
    car.x = 2000.0                                                    # the car drives off: its bikes far behind go
    w.step(6.0)
    assert w.dropped == 5 and not any(k.startswith("bikewin") for k in tr.vehicles)
    assert math.isclose(w.summary()["ratio"], 0.226)


def test_a_failing_window_stops_quietly():
    msgs = []

    class Bad:
        def get_transform(self):
            raise ValueError("boom")
    w = bw.BikeWindow([{"actor": Bad(), "route": None}], [], dict(bw.DEFAULTS, ratio=0.2), {}, lambda: [], {},
                      lambda: {}, log=msgs.append, traci=FakeTraci())
    w.step(0.0)
    w.step(5.0)
    assert w.broken and len(msgs) == 1 and w.summary()["broken"]
