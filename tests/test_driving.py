"""Driving realism: SUMO driver parameters (drivers.py), the measures (drivecheck.py), the physics follow of SUMO
vehicles in CARLA (bridge/animate.py), SUMO vehicles braking for walkers (yielding.py) and the SUMO driver's route
helpers (sumodriver.py). No SUMO binary, no CARLA."""
import math
import sys
import types
import xml.etree.ElementTree as ET

import pytest

from coupledcarla import drivecheck, drivers


# -- drivers.py ------------------------------------------------------------------------------------------------------

def test_vtypes_sizes_and_models():
    from coupledcarla import config, traffic
    tc = dict(config.DEFAULTS["traffic"])
    root = ET.fromstring(traffic._vtypes(tc))
    types_ = {t.get("id"): t for t in root.iter("vType")}
    a2 = types_["vehicle.audi.a2"]
    assert a2.get("carFollowModel") == "EIDM" and a2.get("decel") == "3.4" and a2.get("impatience") == "0"
    assert float(a2.get("length")) == pytest.approx(3.71, abs=0.01)       # the CARLA body, not SUMO's 5 m
    bike = types_["vehicle.gazelle.omafiets"]
    assert bike.get("carFollowModel") == "EIDM"           # not Krauss: no random dawdling 50 times a second
    assert float(bike.get("length")) == pytest.approx(1.84, abs=0.01)
    bus = types_[tc["bus_type"]]
    assert float(bus.get("width")) <= 2.55 and bus.get("decel") == "2.0"


def test_size_attrs_unknown_blueprint():
    assert drivers.size_attrs("vehicle.nope.none", "passenger") == ""


# -- drivecheck.py -------------------------------------------------------------------------------------------------

def test_pct_and_summary():
    assert drivecheck.pct([1, 2, 3, 4], 50) == 2.5
    assert drivecheck.pct([], 50) is None
    s = drivecheck.summary([1.0, 2.0, 3.0])
    assert s["n"] == 3 and s["mean"] == 2.0 and s["max"] == 3.0


def test_fcd_stats_gap_jerk_free_speed_junction():
    st = drivecheck.FcdStats({"e_0": 13.89, ":j_0_0": 10.0}, lambda t: "bike" if "bike" in t else "car")
    for i in range(10):
        t = i * 0.1
        st.add(t, {"id": "a", "type": "car", "speed": "10", "lane": "e_0", "acceleration": str(0.1 * i),
                   "leaderID": "b", "leaderGap": "15"})
        st.add(t, {"id": "f", "type": "car", "speed": "13.89", "lane": "e_0", "acceleration": "0"})
    for i in range(70):        # 7 s standing inside a junction
        st.add(1.0 + i * 0.1, {"id": "j", "type": "car", "speed": "0", "lane": ":j_0_0"})
    st.add(8.0, {"id": "j", "type": "car", "speed": "3", "lane": "e_0"})
    r = st.result()
    assert r["car"]["time_gap_s"]["p50"] == 1.5                       # 15 m at 10 m/s
    assert r["car"]["abs_jerk"]["max"] == pytest.approx(1.0, abs=1e-6)  # 0.1 m/s^2 per 0.1 s
    assert r["car"]["free_speed_over_limit"]["p50"] == pytest.approx(1.0, abs=1e-3)
    assert r["junction_stops_over_5s"] == 1


def test_sumo_statistics_and_collisions(tmp_path):
    (tmp_path / "s.xml").write_text(
        '<statistics><vehicles loaded="10" inserted="9" running="3" waiting="0"/>'
        '<teleports total="2" jam="1" yield="1" wrongLane="0"/>'
        '<safety collisions="3" emergencyStops="0" emergencyBraking="4"/></statistics>')
    s = drivecheck.sumo_statistics(tmp_path / "s.xml")
    assert s["teleports"] == 2 and s["teleports_yield"] == 1 and s["collisions"] == 3 and s["vehicles_inserted"] == 9
    (tmp_path / "c.xml").write_text('<collisions><collision time="1.0" type="junction" collider="a" victim="b"/>'
                                    '<collision time="2.0" type="frontal" collider="c" victim="d"/></collisions>')
    c = drivecheck.collisions(tmp_path / "c.xml")
    assert c["n"] == 2 and c["by_type"] == {"junction": 1, "frontal": 1}


def test_link_state_on_entry():
    links = [("out_0", True, True, False, ":j_0_0", "G", "s", 10.0), ("out2_0", False, False, True, ":j_1_0", "r",
                                                                         "l", 12.0)]
    assert drivecheck.link_state_on_entry(links, ":j_1_0") == "r" and drivecheck.is_red("r")
    assert drivecheck.link_state_on_entry(links, ":x") is None
    assert "M" not in drivecheck.TLS_STATES and "r" in drivecheck.TLS_STATES


def test_obb_overlap_and_point():
    a = (0.0, 0.0, 0.0, 2.0, 1.0)
    assert drivecheck.obb_overlap(a, (3.5, 0.0, 0.0, 2.0, 1.0))
    assert not drivecheck.obb_overlap(a, (4.5, 0.0, 0.0, 2.0, 1.0))
    assert not drivecheck.obb_overlap(a, (3.9, 0.0, 0.0, 2.0, 1.0), shrink=0.1)
    assert drivecheck.point_in_obb(1.9, 0.9, a) and not drivecheck.point_in_obb(0.0, 1.5, a)
    turned = (0.0, 0.0, 90.0, 2.0, 1.0)            # long along y
    assert drivecheck.point_in_obb(0.0, 1.9, turned) and not drivecheck.point_in_obb(1.9, 0.0, turned)


def test_frame_conflicts_and_episodes():
    veh = {"1": (0.0, 0.0, 0.0, 2.0, 1.0), "2": (3.0, 0.0, 0.0, 2.0, 1.0), "3": (50.0, 0.0, 0.0, 2.0, 1.0)}
    pairs, hits = drivecheck.frame_conflicts(veh, {"w": (50.5, 0.2)})
    assert pairs == {("1", "2")} and hits == {("3", "w")}
    assert drivecheck.episodes([(0, "a"), (5, "a"), (100, "a"), (5, "b")], gap_frames=25) == 3


def test_series_jerk_constant_accel():
    times = [i * 0.02 for i in range(200)]
    vel = [(1.0 * t, 0.0) for t in times]          # 1 m/s^2
    acc, jerk = drivecheck.series_jerk(times, vel)
    assert drivecheck.pct(acc, 50) == pytest.approx(1.0, abs=1e-6) and max(jerk) < 1e-6


# -- bridge/animate.py -----------------------------------------------------------------------------------------------

def test_follow_on_track_and_teleport():
    from coupledcarla.bridge import animate
    f = animate.follow((10.0, 0.0, 0.0), (10.0, 0.0, 0.0), 5.0, 0.0, 2.7)
    assert not f["teleport"] and f["velocity"][0] == pytest.approx(5.0) and f["velocity"][1] == pytest.approx(0.0)
    assert f["throttle"] > 0 and f["steer"] == 0.0 and f["brake"] == 0.0
    f = animate.follow((10.0, 0.3, 0.0), (10.0, 0.0, 0.0), 5.0, 0.0, 2.7)     # 0.3 m to the right (+y)
    assert f["velocity"][1] == pytest.approx(1.0) and not f["teleport"]
    f = animate.follow((10.0, 5.0, 0.0), (10.0, 0.0, 0.0), 5.0, 0.0, 2.7)
    assert f["teleport"] and math.hypot(f["velocity"][0] - 5.0, f["velocity"][1]) <= animate.MAX_CORRECTION + 1e-9
    f = animate.follow((0.0, 0.0, 40.0), (0.0, 0.0, 0.0), 5.0, 0.0, 2.7)
    assert f["teleport"]
    f = animate.follow((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, 0.0, 2.7)
    assert f["throttle"] == 0.0 and f["brake"] == 1.0


def test_follow_steers_into_turns():
    from coupledcarla.bridge import animate
    # 5 m/s on a 10 m radius: yaw rate 0.5 rad/s, wheelbase 2.7 m -> about 15 deg of front wheel
    f = animate.follow((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 5.0, math.degrees(0.5), 2.7)
    assert f["steer"] == pytest.approx(math.degrees(math.atan(0.27)) / animate.MAX_STEER_DEG, rel=1e-3)
    assert animate.yaw_rate(359.0, 1.0, 0.02) == pytest.approx(100.0)
    vx, vy = animate.velocity(2.0, 90.0)
    assert vx == pytest.approx(0.0, abs=1e-9) and vy == pytest.approx(2.0)


# -- yielding.py -----------------------------------------------------------------------------------------------------

@pytest.fixture()
def yielding(monkeypatch):
    calls = {"speed": {}, "mode": {}}
    fake = types.SimpleNamespace(
        vehicle=types.SimpleNamespace(setSpeed=lambda s, v: calls["speed"].__setitem__(s, v),
                                      slowDown=lambda s, v, d: calls["speed"].__setitem__(s, v),
                                      setSpeedMode=lambda s, m: calls["mode"].__setitem__(s, m)),
        TraCIException=RuntimeError)
    monkeypatch.setitem(sys.modules, "traci", fake)
    sys.modules.pop("coupledcarla.yielding", None)
    import coupledcarla.yielding as Y
    Y.calls = calls
    yield Y
    sys.modules.pop("coupledcarla.yielding", None)


def test_first_conflict_ahead_and_beside(yielding):
    pose = (0.0, 0.0, 0.0, 10.0, 2.2, 0.9)          # 10 m/s along +x
    assert yielding.first_conflict(pose, 0.0, [(20.0, 0.0, 0.0, 0.0)]) == pytest.approx(18.0, abs=1.0)
    assert yielding.first_conflict(pose, 0.0, [(20.0, 4.0, 0.0, 0.0)]) is None      # on the kerb, standing
    # stepping out from the kerb at 1.4 m/s towards the lane: there when the car arrives
    assert yielding.first_conflict(pose, 0.0, [(25.0, 4.0, 0.0, -1.4)]) is not None
    assert yielding.first_conflict(pose, 0.0, [(-10.0, 0.0, 0.0, 0.0)]) is None     # behind


def test_braking_levels(yielding):
    a, em = yielding.braking(10.0, 50.0)
    assert (a, em) == (0.0, False)                                # far: no need yet
    a, em = yielding.braking(10.0, 25.0)
    assert 2.0 <= a <= 3.4 and not em                              # 100 / (2 * 22.5) = 2.2
    a, em = yielding.braking(14.0, 12.0)
    assert em and a == pytest.approx(min(8.0, 196 / 19.0), rel=1e-6)
    assert yielding.braking(14.0, 2.0) == (yielding.EMERGENCY, True)


class _Loc(types.SimpleNamespace):
    pass


def _walker(x, y, vx=0.0, vy=0.0):
    return types.SimpleNamespace(get_location=lambda: _Loc(x=x, y=y), get_velocity=lambda: _Loc(x=vx, y=vy))


def test_sumo_yield_brakes_holds_and_releases(yielding):
    sync = types.SimpleNamespace(sumo2carla_ids={"veh7": 70}, poses={70: (0.0, 0.0, 0.0, 12.0, 2.2, 0.9)})
    y = yielding.SumoYield(every=1)
    y.step(sync, None, [_walker(20.0, 0.0)])
    assert yielding.calls["speed"]["veh7"] < 12.0 and "veh7" in y.held
    assert yielding.calls["mode"].get("veh7") == 27                 # 144 / (2 * 15.3) = 4.7 > 3.4: emergency
    assert any(n == "sumo_emergency_stop" for n, _ in y.events)
    sync.poses = {70: (0.0, 0.0, 0.0, 0.0, 2.2, 0.9)}                # stopped; walker still there
    y.step(sync, None, [_walker(20.0, 0.0)])
    assert yielding.calls["speed"]["veh7"] == 0.0
    for _ in range(int(yielding.CLEAR_S / 0.02) + 2):              # walker gone: released after CLEAR_S
        y.step(sync, None, [_walker(20.0, 30.0)])
    assert yielding.calls["speed"]["veh7"] == -1 and yielding.calls["mode"]["veh7"] == 31 and not y.held


def test_sumo_yield_releases_cars_that_leave_carla(yielding):
    sync = types.SimpleNamespace(sumo2carla_ids={"veh7": 70}, poses={70: (0.0, 0.0, 0.0, 5.0, 2.2, 0.9)})
    y = yielding.SumoYield(every=1)
    y.step(sync, None, [_walker(8.0, 0.3)])
    assert "veh7" in y.held
    sync.sumo2carla_ids, sync.poses = {}, {}
    y.step(sync, None, [_walker(8.0, 0.3)])
    assert yielding.calls["speed"]["veh7"] == -1 and not y.held


# -- sumodriver.py ---------------------------------------------------------------------------------------------------

class _Edge:
    def __init__(self, eid, length=100.0, to=(0.0, 0.0)):
        self.eid, self.length, self.to = eid, length, to

    def getID(self):
        return self.eid

    def getLength(self):
        return self.length

    def getFunction(self):
        return ""

    def allows(self, vclass):
        return True

    def getToNode(self):
        return types.SimpleNamespace(getCoord=lambda: self.to)


class _Lane:
    def __init__(self, edge, shape, allow=True):
        self.edge, self.shape, self.allow = edge, shape, allow

    def getShape(self):
        return self.shape

    def getEdge(self):
        return self.edge

    def allows(self, vclass):
        return self.allow

    def getClosestLanePosAndDist(self, p):
        (x0, y0), (x1, y1) = self.shape[0], self.shape[-1]
        L = math.hypot(x1 - x0, y1 - y0)
        u = max(0.0, min(L, ((p[0] - x0) * (x1 - x0) + (p[1] - y0) * (y1 - y0)) / L))
        return u, 0.0


def test_lane_heading_and_pick_lane(monkeypatch):
    sd = pytest.importorskip("coupledcarla.sumodriver")
    east = _Lane(_Edge("e"), [(0.0, 0.0), (100.0, 0.0)])
    west = _Lane(_Edge("-e"), [(100.0, 3.0), (0.0, 3.0)])
    north = _Lane(_Edge("n"), [(0.0, 0.0), (0.0, 100.0)])     # SUMO north = CARLA yaw -90
    assert sd.lane_heading(east, 10.0) == pytest.approx(0.0)
    assert abs(sd.lane_heading(west, 10.0)) == pytest.approx(180.0)
    assert sd.lane_heading(north, 10.0) == pytest.approx(-90.0)
    net = types.SimpleNamespace(getNeighboringLanes=lambda x, y, r: [(west, 1.0), (east, 2.0), (north, 5.0)])
    lane, pos = sd.pick_lane(net, 50.0, 1.0, yaw=0.0)
    assert lane is east and pos == pytest.approx(50.0)
    assert sd.pick_lane(net, 50.0, 1.0, yaw=180.0)[0] is west
    assert sd.pick_lane(net, 50.0, 1.0)[0] is west               # no heading: the nearest
    assert sd.sumo_xy((10.0, 20.0), 1.0, 2.0) == (11.0, 18.0)


def test_plan_route_and_random_destination():
    sd = pytest.importorskip("coupledcarla.sumodriver")
    import random
    a, b, c = _Edge("a", to=(0, 0)), _Edge("b", to=(800, 0)), _Edge("c", to=(900, 0))
    paths = {("a", "b"): [a, b], ("b", "c"): [b, c], ("a", "c"): [a, b, c]}
    net = types.SimpleNamespace(getShortestPath=lambda f, t, vClass=None: (paths.get((f.getID(), t.getID())), 1.0),
                                getEdges=lambda: [a, b, c])
    assert sd.plan_route(net, a, [b, c]) == ["a", "b", "c"]
    assert sd.plan_route(net, b, [a]) is None
    assert sd.random_destination(net, a, random.Random(1)) == ["a", "b", "c"]


# -- pedsignals.py ---------------------------------------------------------------------------------------------------

def test_crossing_links_and_all_red():
    from coupledcarla import pedsignals as P
    # a crossing along y at x = 0 (from (0, -5) to (0, 5)); traffic along +x stops at (-3, 2); traffic along y
    # (parallel to the crossing) stops at (2, -8); a far stop line at (60, 0)
    links = [("J", 0, (-3.0, 2.0), (1.0, 0.0)), ("J", 1, (2.0, -8.0), (0.0, 1.0)), ("K", 0, (60.0, 0.0), (1.0, 0.0))]
    ctl = P.crossing_links(links, (0.0, -5.0), (0.0, 5.0))
    assert ctl == [("J", 0)]
    assert P.all_red({"J": "rG"}, ctl) and not P.all_red({"J": "Gr"}, ctl) and not P.all_red({"J": "yr"}, ctl)
    sig = P.PedSignals(links, lambda tls: {"J": "rG"}[tls])
    assert sig.may_cross((0.0, -5.0), (0.0, 5.0)) is True
    assert sig.may_cross((100.0, -5.0), (100.0, 5.0)) is None          # no signal near


def test_scramble_programs(tmp_path):
    from coupledcarla import pedsignals as P
    phase = lambda d, s: types.SimpleNamespace(duration=d, state=s)   # noqa: E731
    prog = types.SimpleNamespace(getPhases=lambda: [phase(30, "GGrr"), phase(3, "yyrr"), phase(30, "rrGg")])
    tls = types.SimpleNamespace(getPrograms=lambda: {"0": prog})
    net = types.SimpleNamespace(getTLS=lambda tid: tls)
    assert P.scramble_programs(net, ["J"], tmp_path / "s.add.xml", seconds=25) == 1
    root = ET.parse(tmp_path / "s.add.xml").getroot()
    states = [(p.get("duration"), p.get("state")) for p in root.iter("phase")]
    assert states[-1] == ("25", "rrrr") and root.find("tlLogic").get("id") == "J"
    assert states[-2] == ("3", "rryy") and len(states) == 5            # amber first for the streams still green


def test_scramble_points(tmp_path):
    from coupledcarla import pedsignals as P
    (tmp_path / "map.osm").write_text(
        '<osm><node id="1" lat="35.0" lon="139.0"><tag k="crossing:scramble" v="yes"/></node>'
        '<node id="2" lat="35.1" lon="139.1"/><node id="3" lat="35.2" lon="139.2"/>'
        '<way id="9"><nd ref="2"/><nd ref="3"/><tag k="crossing:scramble" v="yes"/></way></osm>')
    assert sorted(P.scramble_points(tmp_path / "map.osm")) == [(35.0, 139.0), (35.1, 139.1), (35.2, 139.2)]
    assert P.scramble_points(tmp_path / "missing.osm") == []


def test_crowd_walker_blocked_by_standing_car():
    from coupledcarla import crowdpeds
    poses = {1: (10.0, 0.0, 0.0, 0.0, 2.3, 0.9)}       # a car along +x centred at x = 10
    assert crowdpeds.blocked(12.4, 0.0, poses)           # just ahead of its bumper (within the margin)
    assert not crowdpeds.blocked(13.0, 0.0, poses) and not crowdpeds.blocked(10.0, 1.5, poses)
    assert not crowdpeds.blocked(0.0, 0.0, None)
