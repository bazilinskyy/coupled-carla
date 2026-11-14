"""CROWD-calibrated pedestrians: the table, the map's place and fallbacks, kerbs, the walker plan, gap acceptance
(no network, no SUMO binary, no CARLA)."""
import json
import math
import random

import pytest

from coupledcarla import crowdpeds as cp

NL = [51.436, 5.47, 51.45, 5.493]          # Eindhoven
CAIRO = [30.03811, 31.22844, 30.05069, 31.24296]
OCEAN = [-40.0, -30.0, -39.99, -29.99]     # South Atlantic: no CROWD locality near


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    def refuse(*a, **k):
        raise OSError("no network in tests")
    import requests
    monkeypatch.setattr(requests, "get", refuse)
    monkeypatch.setattr(requests, "post", refuse)


def test_table_has_sources_and_world_values():
    t = cp.table()
    assert "CC BY 4.0" in t["source"]["licence"] and t["source"]["crowd_version"]
    w = t["world"]
    assert 5 < w["persons_per_min"] < 40 and 1.1 < w["speed_mps"] < 1.6
    assert 0 < w["crossing_share_city_median"] < 0.1 and 0 < w["jaywalk_share"] < 0.2
    # the measured countries of crowd-jaywalking
    for iso3 in ("JPN", "BGD", "VNM", "FRA", "NLD"):
        assert t["countries"][iso3]["jaywalk_n"] >= 50
    assert t["countries"]["BGD"]["jaywalk_share"] > 0.5 > t["countries"]["JPN"]["jaywalk_share"]


def test_country_codes():
    assert cp.country_code("USA") == "USA" and cp.country_code("United States") == "USA"
    assert cp.country_code("Egypt") == "EGY" and cp.country_code("NL") == "NLD" and cp.country_code("jpn") == "JPN"
    assert cp.country_code("Atlantis") is None and cp.country_code(None) is None


def test_locate_from_experiment_nearest_locality_and_offline(tmp_path):
    assert cp.locate({"city": {"country": "Egypt"}}, CAIRO)[0] == "EGY"
    iso3, how = cp.locate({}, NL)
    assert iso3 == "NLD" and "nearest CROWD locality" in how
    iso3, how = cp.locate({}, OCEAN, tmp_path / "c.json")        # Nominatim refused: unknown
    assert iso3 is None and not (tmp_path / "c.json").exists()
    (tmp_path / "c.json").write_text(json.dumps({"iso2": "AR"}))  # a cached answer is used
    assert cp.locate({}, OCEAN, tmp_path / "c.json") == ("ARG", "OSM Nominatim")


def test_behaviour_levels_and_fallbacks():
    b = cp.behaviour({}, {}, NL)
    assert b["country"] == "NLD" and b["persons_per_min_from"] == "locality Eindhoven"
    assert b["speed_from"] == "locality Eindhoven" and 1.2 < b["speed_median"] < 1.5 and b["speed_sd"] > 0
    assert b["jaywalk_from"].startswith("measured")
    eg = cp.behaviour({}, {"city": {"country": "Egypt"}}, CAIRO)
    assert eg["jaywalk_from"].startswith("modelled") and 0 < eg["jaywalk_share"] < 1
    assert eg["speed_from"].startswith("continent")          # no Egyptian city in crowd-city's table
    none = cp.behaviour({}, {}, OCEAN)
    assert none["country"] is None and none["speed_from"] == "world" and none["persons_per_min_from"] == "world"
    assert none["density_factor"] == 1.0 and none["crossing_factor"] > 0
    # overrides
    o = cp.behaviour({"jaywalk_share": 0.3, "speed_mps": 1.1}, {}, NL)
    assert o["jaywalk_share"] == 0.3 and o["speed_median"] == 1.1


def test_night_lowers_density_where_crowd_saw_it():
    day, night = cp.behaviour({}, {}, NL), cp.behaviour({}, {}, NL, night=True)
    assert night["night_factor"] < 1 and night["walkers_per_km"] < day["walkers_per_km"]


def _street(n=81, width=10.0, crossings=(40,), step=2.5):
    a = [(i * step, 0.0) for i in range(n)]
    b = [(i * step, width) for i in range(n)]
    return cp.Segment("s", a, b, step, crossings, (True, True))


PARAMS = {"crossings_per_km": 5.0, "jaywalk_share": 0.3, "speed_median": 1.3, "speed_sd": 0.2}


def test_plan_crosses_where_and_as_often_as_asked():
    seg = _street()
    att = cp.attraction([seg], [])
    walkers = cp.plan([seg], att, PARAMS, 300, 600, random.Random(2))
    st = cp.plan_stats([seg], walkers, 600)
    assert st["walkers"] == 300 and st["crossings"] > 100
    assert 0.15 < st["midblock_share"] < 0.45
    assert st["junction"] == 0                                 # the street has a marked crossing
    for w in walkers:
        for leg in w["legs"]:
            if leg[0] == "cross" and leg[4] == "crosswalk":
                assert leg[3] == 40
            if leg[0] == "cross" and leg[4] == "midblock":
                assert abs(leg[3] - 40) > 4 and 4 <= leg[3] <= 76
    # walking legs join up: each starts where the previous one ended
    for w in walkers[:30]:
        side, i = w["side"], w["i"]
        for leg in w["legs"]:
            assert leg[2] == side and leg[3] == i
            if leg[0] == "walk":
                i = leg[4]
            else:
                side = 1 - side
    # no crossings asked: none planned
    none = cp.plan([seg], att, {**PARAMS, "crossings_per_km": 0.0}, 50, 600, random.Random(1))
    assert cp.plan_stats([seg], none, 600)["crossings"] == 0


def test_plan_without_marked_crossing_uses_junctions():
    seg = _street(crossings=())
    walkers = cp.plan([seg], cp.attraction([seg], []), {**PARAMS, "jaywalk_share": 0.0}, 100, 600, random.Random(3))
    st = cp.plan_stats([seg], walkers, 600)
    assert st["crosswalk"] == 0 and st["midblock"] == 0 and st["junction"] > 0


def test_calibration_reaches_the_target_rate():
    seg = _street(n=200)
    att = cp.attraction([seg], [])
    walkers, rate, jay = cp.plan_calibrated([seg], att, {**PARAMS, "crossings_per_km": 3.0}, 300, 900)
    st = cp.plan_stats([seg], walkers, 900)
    assert abs(st["crossings_per_km"] - 3.0) < 0.5 and abs(st["midblock_share"] - 0.3) < 0.1 and jay >= 0.3


def test_places_and_counts_draw_walkers():
    seg = _street()
    att = cp.attraction([seg], [(10.0, -5.0, 10.0)])          # a station near the west end of side 0
    walkers = cp.plan([seg], att, PARAMS, 400, 60, random.Random(4))
    west = sum(1 for w in walkers if w["side"] == 0 and w["i"] < 15)
    assert west > 2 * 400 * 15 / 162                          # far more than an even spread
    att2 = cp.attraction([seg], [], counts_xy=[(0.0, 0.0, 1000), (200.0, 0.0, 100), (100.0, 300.0, 400)])
    assert att2[0][0][0] > att2[0][0][80]                     # the busy count site weighs more


def test_gap_acceptance():
    a, b = (0.0, 0.0), (0.0, 10.0)                            # crossing northwards, road along x
    t_c = cp.critical_gap(10.0, 1.25)
    assert abs(t_c - 10.0) < 1e-9 and cp.critical_gap(10.0, 1.25, 60) == pytest.approx(6.0)
    near = [(-40.0, 3.0, 12.0, 0.0)]                          # 40 m away at 12 m/s: 3.3 s
    far = [(-200.0, 3.0, 12.0, 0.0)]                          # 16.7 s
    away = [(-10.0, 3.0, -12.0, 0.0)]
    standing = [(0.5, 5.0, 0.0, 0.0)]
    assert not cp.gap_ok(a, b, near, t_c) and cp.gap_ok(a, b, far, t_c) and cp.gap_ok(a, b, away, t_c)
    assert not cp.gap_ok(a, b, standing, t_c)
    # at a marked crossing a car that can still stop does not hold the walker back; one that cannot does
    slow = [(-40.0, 3.0, 8.0, 0.0)]                           # 5 s away, stops in 8 + 10.7 + 3 m
    assert not cp.gap_ok(a, b, slow, t_c) and cp.gap_ok(a, b, slow, t_c, marked=True)
    assert not cp.gap_ok(a, b, [(-12.0, 3.0, 8.0, 0.0)], t_c, marked=True)


class _Lane:
    def __init__(self, shape, width=3.2, allow=("passenger",)):
        self._s, self._w, self._a = shape, width, allow

    def getShape(self):
        return self._s

    def getWidth(self):
        return self._w

    def allows(self, c):
        return c in self._a


class _Node:
    def __init__(self, deg):
        self._d = deg

    def getIncoming(self):
        return [0] * self._d

    def getOutgoing(self):
        return [0] * self._d


class _Edge:
    def __init__(self, eid, lanes, typ="highway.residential"):
        self._id, self._l, self._t = eid, lanes, typ

    def getID(self):
        return self._id

    def getType(self):
        return self._t

    def getLanes(self):
        return self._l

    def getLength(self):
        s = self._l[0].getShape()
        return math.hypot(s[-1][0] - s[0][0], s[-1][1] - s[0][1])

    def getFromNode(self):
        return _Node(3)

    def getToNode(self):
        return _Node(1)

    def getName(self):
        return "Teststraat"


class _Net:
    def __init__(self, edges):
        self._e = edges

    def getEdges(self):
        return self._e


def test_kerbs_of_a_two_way_street_and_a_one_way_street():
    # right-hand two-way street along x: eastbound lane at y = -1.6, westbound at y = +1.6
    east = _Edge("7", [_Lane([(0, -1.6), (100, -1.6)])])
    west = _Edge("-7", [_Lane([(100, 1.6), (0, 1.6)])])
    cyc = _Edge("9", [_Lane([(0, 20), (100, 20)], allow=("bicycle",))], "highway.cycleway")
    one = _Edge("8", [_Lane([(0, 50), (100, 50)]), _Lane([(0, 53.2), (100, 53.2)])])
    segs = cp.kerb_segments(_Net([east, west, cyc, one]))
    assert len(segs) == 2
    two = next(s for s in segs if s.id == "7")
    ys = sorted([two.sides[0][10][1], two.sides[1][10][1]])
    assert ys == pytest.approx([-1.6 - 1.6 - 1.5, 1.6 + 1.6 + 1.5])
    assert two.width(10) == pytest.approx(9.4) and two.junction == (True, False)
    o = next(s for s in segs if s.id == "8")
    assert sorted([o.sides[0][5][1], o.sides[1][5][1]]) == pytest.approx([50 - 3.1, 53.2 + 3.1])
    # an OSM crossing on the street is snapped onto it, one off the street is not
    assert cp.snap_crossings(segs, [(50.0, 0.5), (50.0, 30.0)]) == 1 and two.crossings == [20]
    # crop to 30 m around (50, 0)
    c = cp.crop(segs, [(50.0, 0.0)], 30.0)
    assert len(c) == 1 and c[0].length <= 60 and c[0].crossings and c[0].junction == (False, False)


def test_osm_crossings(tmp_path):
    f = tmp_path / "map.osm"
    f.write_text('<osm><node id="1" lat="51.1" lon="5.1"><tag k="highway" v="crossing"/>'
                 '<tag k="crossing" v="traffic_signals"/></node><node id="2" lat="51.2" lon="5.2">'
                 '<tag k="highway" v="crossing"/><tag k="crossing" v="no"/></node>'
                 '<node id="3" lat="51.3" lon="5.3"><tag k="railway" v="crossing"/></node>'
                 '<way id="4"><nd ref="1"/><tag k="highway" v="residential"/></way></osm>')
    assert cp.osm_crossings(f) == [(51.1, 5.1, True)]
    assert cp.osm_crossings(tmp_path / "missing.osm") == []


def test_nyc_counts_take_the_latest_survey(monkeypatch):
    class R:
        def raise_for_status(self):
            pass

        def json(self):
            return [{"the_geom": {"coordinates": [-73.98, 40.75]}, "from_stree": "Broadway", "to_street": "W 42",
                     "may_07_am": "N", "sept_19_md": "100", "oct24_am": "2,000", "oct24_pm": "1000",
                     "may25_md": "3,000", "may_22_p_m": "5"}]
    import requests
    monkeypatch.setattr(requests, "get", lambda *a, **k: R())
    c = cp._nyc_counts([40.748, -73.99, 40.76, -73.97])
    assert c == [{"lat": 40.75, "lon": -73.98, "count": 3000.0, "year": 2025, "name": "Broadway W 42"}]


def test_ped_counts_fail_soft_and_skip_elsewhere(tmp_path):
    nyc = cp.ped_counts([40.748, -73.99, 40.76, -73.97], tmp_path / "a.json")
    assert nyc["counts"] == [] and not (tmp_path / "a.json").exists()     # refused: not cached, retried later
    ehv = cp.ped_counts(NL, tmp_path / "b.json")
    assert ehv == {"source": None, "counts": []} and (tmp_path / "b.json").exists()


class _V:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


def _fake_carla():
    import types
    m = types.ModuleType("carla")
    m.Location = lambda x=0.0, y=0.0, z=0.0: _V(x, y, z)
    m.Vector3D = lambda x=0.0, y=0.0, z=0.0: _V(x, y, z)
    m.Transform = lambda loc=None, rot=None: types.SimpleNamespace(location=loc)
    m.WalkerControl = lambda direction=None, speed=0.0: types.SimpleNamespace(direction=direction or _V(), speed=speed)
    m.LaneType = types.SimpleNamespace(Any=0)
    return m


class _Actor:
    _ids = iter(range(100, 1000))

    def __init__(self, loc, vel=None, type_id="walker.pedestrian.0015"):
        self.id, self.loc, self.vel, self.type_id, self.ctl = next(self._ids), loc, vel or _V(), type_id, None

    def get_location(self):
        return self.loc

    def get_velocity(self):
        return self.vel

    def apply_control(self, ctl):
        self.ctl = ctl


class _BP:
    id = "walker.pedestrian.0015"

    def has_attribute(self, k):
        return True

    def set_attribute(self, k, v):
        pass


class _World:
    def __init__(self):
        import types
        self.vehicles, self.walkers = [], []
        wp = types.SimpleNamespace(transform=types.SimpleNamespace(location=_V(z=0.2)))
        self._map = types.SimpleNamespace(get_waypoint=lambda *a, **k: wp)

    def get_map(self):
        return self._map

    def get_blueprint_library(self):
        import types
        return types.SimpleNamespace(filter=lambda pat: [_BP()])

    def try_spawn_actor(self, bp, tf):
        a = _Actor(_V(tf.location.x, tf.location.y, tf.location.z))
        self.walkers.append(a)
        return a

    def get_actors(self):
        import types
        return types.SimpleNamespace(filter=lambda pat: list(self.vehicles))


def test_crowdsim_waits_for_a_gap_then_crosses_and_counts_as_on_the_road(monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, "carla", _fake_carla())
    seg = _street(n=9, width=8.0, crossings=(4,))
    plan = [{"seg": 0, "side": 0, "i": 2, "speed": 1.6,
             "legs": [("walk", 0, 0, 2, 4), ("cross", 0, 0, 4, "crosswalk"), ("walk", 0, 1, 4, 8)]}]
    world = _World()
    # a car on the road (CARLA y = -geo y: the road runs along x at y between 0 and -8) coming fast from the west
    car = _Actor(_V(-30.0, -4.0), _V(14.0, 0.0), "vehicle.tesla.model3")
    world.vehicles.append(car)
    sim = cp.CrowdSim(world, None, [seg], plan, {}, log=lambda m: None)
    (w,) = sim.walkers
    assert (w.loc.x, w.loc.y, w.loc.z) == (5.0, 0.0, 1.2)

    def tick(t, dt=0.1):
        sim.step(t)
        if w.ctl is not None and w.ctl.speed > 0:
            w.loc = _V(w.loc.x + w.ctl.direction.x * w.ctl.speed * dt, w.loc.y + w.ctl.direction.y * w.ctl.speed * dt)

    t = 0.0
    for _ in range(40):                     # walks 5 m to the crossing, then waits at the kerb
        tick(t)
        t += 0.1
    assert abs(w.loc.x - 10.0) < 1.0 and w.ctl.speed == 0.0 and sim.on_road() == [] and not sim.events
    world.vehicles.clear()                  # the car has passed
    tick(t)
    assert sim.on_road() == [w] and sim.events[0][0] == "crowd_crosses"
    for _ in range(80):
        t += 0.1
        tick(t)
    assert sim.on_road() == [] and w.loc.y < -7.0 and w.loc.x > 12.0     # across, walking on along the far kerb


def test_replay_waits_for_traffic_and_reports_the_gap():
    seg = _street(n=9, width=8.0, crossings=(4,))
    plan = [{"seg": 0, "side": 0, "i": 4, "speed": 1.6, "legs": [("cross", 0, 0, 4, "midblock")]}]
    # CARLA y = -geo y: the crossing runs from (10, 0) to (10, -8); a car at 13 m/s on y = -4 passes x = 10 at 3 s
    frames = [(t / 2, [(-29.0 + 13.0 * t / 2, -4.0, 13.0, 0.0)] if t / 2 < 3.0 else []) for t in range(0, 40)]
    (r,) = cp.replay([seg], plan, frames, 30.0)
    assert not r["censored"] and r["kind"] == "midblock" and 2.5 <= r["wait"] <= 3.5 and r["gap"] is None
    (r,) = cp.replay([seg], plan, [(0.0, [(-60.0, -4.0, 0.5, 0.0)])], 30.0)       # a crawling car far off
    assert r["wait"] == 0.0 and r["gap"] > 60
