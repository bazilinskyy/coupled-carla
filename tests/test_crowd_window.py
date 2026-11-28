"""The crowd's rolling window around drivers (coupledcarla/crowdwindow.py): no CARLA, SUMO or network."""
import math
import random

from coupledcarla import config, crowdpeds, crowdwindow
from coupledcarla.crowdpeds import Segment

CFG = {k: float(v) for k, v in crowdwindow.DEFAULTS.items() if k != "enabled"}
PARAMS = {"crossings_per_km": 2.0, "jaywalk_share": 0.05, "speed_median": 1.35, "speed_sd": 0.15}


def street(x0, x1, y=0.0, half=5.0, step=2.5, sid="s"):
    """A straight street along x (CARLA y = y); kerbs at CARLA y +- half (geo.net y is CARLA's -y)."""
    n = int(round((x1 - x0) / step)) + 1
    xs = [x0 + i * step for i in range(n)]
    return Segment(sid, [(x, -(y - half)) for x in xs], [(x, -(y + half)) for x in xs], step)


def cross_street(x, y0, y1, half=5.0, step=2.5, sid="c"):
    """A straight street along y at CARLA x."""
    n = int(round((y1 - y0) / step)) + 1
    ys = [y0 + i * step for i in range(n)]
    return Segment(sid, [(x - half, -y) for y in ys], [(x + half, -y) for y in ys], step)


def ones(segments):
    return [[[1.0] * s.n, [1.0] * s.n] for s in segments]


# --- geometry -------------------------------------------------------------------------------------------------------

def test_route_index_points_every_5_m_and_finds_the_car_on_it():
    ri = crowdwindow.RouteIndex([(0, 0), (100, 0), (100, 100)])
    assert ri.length == 200.0
    assert all(abs((b[2] - a[2]) - 5.0) < 1e-9 for a, b in zip(ri.pts, ri.pts[1:]))
    assert abs(ri.locate(42, 3) - 40.0) <= 2.5
    assert abs(ri.locate(103, 60) - 160.0) <= 5.0       # within a point of it
    assert ri.inside(50, 20, 0, 100, 25) and not ri.inside(50, 20, 60, 200, 25)
    assert not ri.inside(50, 40, 0, 200, 25)


def test_locate_takes_the_first_pass_of_a_route_that_comes_back():
    # out along y = 0 and back along y = 8: the car at the start is at the start, not on the way back
    ri = crowdwindow.RouteIndex([(0, 0), (200, 0), (200, 8), (0, 8)])
    assert ri.locate(10, 1) < 20.0


def test_initial_mask_is_the_route_ahead_the_car_and_the_people_on_foot():
    main = street(0, 1000)                  # the driver's street
    side_near = cross_street(520, 10, 300, sid="near")     # crosses the route 420 m ahead: beyond the window
    side_far = cross_street(900, 10, 300, sid="far")
    segs = [main, side_near, side_far]
    ri = crowdwindow.RouteIndex([(100, 0), (1000, 0)])
    anchor = {"x": 100.0, "y": 0.0, "yaw": 0.0, "ri": ri, "s": 0.0}
    mask = crowdwindow.initial_mask(segs, [anchor], [(900.0, 250.0)], CFG, 120.0)
    on = [main.sides[0][i][0] for i in range(main.n) if mask[0][0][i]]
    assert min(on) <= 52.5 and max(on) >= 397.5 and max(on) <= 402.5      # 50 m behind .. 300 m ahead
    assert not any(mask[1][0]) and not any(mask[1][1])      # 400 m ahead is beyond the window
    # the pedestrian at (900, 250) keeps its own 150 m around it
    ys = [-side_far.sides[0][i][1] for i in range(side_far.n) if mask[2][0][i]]
    assert ys and min(ys) >= 100.0 - 2.5 and max(ys) == 300.0


def test_spawn_zone_is_the_window_s_far_end_and_never_near_a_participant():
    segs = [street(0, 1000)]
    smp = crowdwindow.Samples(segs)
    ri = crowdwindow.RouteIndex([(0, 0), (1000, 0)])
    anchor = {"x": 500.0, "y": 0.0, "yaw": 0.0, "ri": ri, "s": 500.0}
    xs = [smp.items[j][3] for j in crowdwindow.zone(smp, anchor, CFG, 120.0, spawn=True)]
    assert xs and min(xs) >= 700.0 - 2.5 and max(xs) <= 800.0 + 2.5       # within a route point (5 m)
    # without a route: the far edge of the radius, ahead of the car
    ring = {"x": 500.0, "y": 0.0, "yaw": 180.0, "ri": None}
    xs = [smp.items[j][3] for j in crowdwindow.zone(smp, ring, CFG, 120.0, spawn=True)]
    assert xs and all(380.0 - 1e-6 <= x <= 440.0 + 1e-6 for x in xs)


def test_in_window_keeps_a_margin_and_drops_walkers_far_behind():
    ri = crowdwindow.RouteIndex([(0, 0), (1000, 0)])
    an = {"x": 500.0, "y": 0.0, "yaw": 0.0, "ri": ri, "s": 500.0}
    assert crowdwindow.in_window(540, 5, an, CFG, 120.0)      # just passed (within behind_m)
    assert crowdwindow.in_window(810, 5, an, CFG, 120.0)      # the far end, within the slack
    assert crowdwindow.in_window(600, 60, an, CFG, 120.0)     # off the route, within band + slack
    assert not crowdwindow.in_window(300, 5, an, CFG, 120.0)  # 200 m behind
    assert not crowdwindow.in_window(600, 80, an, CFG, 120.0)


def test_unseen_behind_or_beyond_45_m():
    v = [(0.0, 0.0, 0.0)]                   # looking along +x
    assert not crowdwindow.unseen(30, 0, v)
    assert crowdwindow.unseen(-30, 0, v)    # behind
    assert crowdwindow.unseen(50, 0, v)


# --- planning -------------------------------------------------------------------------------------------------------

def test_plan_from_is_plan_with_the_start_given():
    segs = [street(0, 300), street(0, 200, y=50, sid="b")]
    att = ones(segs)
    a = crowdpeds.plan(segs, att, PARAMS, 6, 120, random.Random(7))
    rng = random.Random(7)
    b = []
    for _ in range(6):         # plan's own draws for the start, then plan_from's
        acc = sum(sum(sd) for sides in att for sd in sides)
        rng.random()
        w = a[len(b)]
        rng.choices(range(segs[w["seg"]].n), weights=att[w["seg"]][w["side"]])
        b.append(crowdpeds.plan_from(segs, PARAMS, w["seg"], w["side"], w["i"], 120, rng))
    assert acc > 0 and a == b


def test_masked_att_and_its_kerb_length():
    segs = [street(0, 100)]
    mask = [[[i < 10 for i in range(segs[0].n)], [False] * segs[0].n]]
    att, km = crowdpeds.masked(segs, ones(segs), mask)
    assert abs(km - 10 * 2.5 / 1000) < 1e-12
    assert sum(att[0][0]) == 10 and sum(att[0][1]) == 0
    walkers = crowdpeds.plan(segs, att, PARAMS, 20, 60, random.Random(1))
    assert all(w["side"] == 0 and w["i"] < 10 for w in walkers)


def test_route_reach_cap():
    assert crowdpeds.route_reach(3600) == crowdpeds.ROUTE_AHEAD_MAX_M
    assert crowdpeds.route_reach(3600, 6000) == 6000.0 and crowdpeds.route_reach(60, 6000) == 13.9 * 60


def test_crop_with_a_grid_keeps_the_same_stretches():
    segs = [street(0, 500), cross_street(250, 10, 400)]
    centres = [(0.0, 0.0), (250.0, -380.0)]           # geo.net x, y
    got = crowdpeds.crop(segs, centres, 120.0)
    assert [s.id for s in got] == ["s", "c"]
    assert max(x for x, _ in got[0].sides[0]) <= 120.0 and got[1].n < segs[1].n


# --- settings -------------------------------------------------------------------------------------------------------

def test_settings_auto_on_for_drivers_only():
    traffic = config.DEFAULTS["traffic"]
    assert crowdwindow.settings({"traffic": traffic, "participants": [{"role": "d"}]})["enabled"]
    assert crowdwindow.settings({"traffic": traffic, "participants": [{"role": "a", "kind": "av_passenger"}]})["enabled"]
    assert not crowdwindow.settings({"traffic": traffic, "participants": [{"role": "p", "kind": "pedestrian"}]})[
        "enabled"]
    off = {**traffic, "walkers": {**traffic["walkers"], "behaviour": "off"}}
    assert not crowdwindow.settings({"traffic": off, "participants": [{"role": "d"}]})["enabled"]
    no = {**traffic, "walkers": {**traffic["walkers"], "window": {"enabled": False}}}
    assert not crowdwindow.settings({"traffic": no, "participants": [{"role": "d"}]})["enabled"]


def test_config_and_module_defaults_agree():
    cfg = config.DEFAULTS["traffic"]["walkers"]["window"]
    assert set(cfg) == set(crowdwindow.DEFAULTS)
    assert all(cfg[k] == crowdwindow.DEFAULTS[k] for k in cfg)


# --- the window at work, with stand-ins for CARLA ----------------------------------------------------------------

class _Loc:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _Rot:
    def __init__(self, yaw):
        self.yaw = yaw


class _Tf:
    def __init__(self, x, y, yaw):
        self.location, self.rotation = _Loc(x, y), _Rot(yaw)


class _Actor:
    def __init__(self, x, y, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw

    def get_transform(self):
        return _Tf(self.x, self.y, self.yaw)


class _Sim:
    """CrowdSim's surface that the window uses: state, add, remove, hidden."""

    def __init__(self, segments):
        self.segments, self.state, self.hidden = segments, [], None
        self.removed = []

    def xy(self, seg, side, i):
        x, y = self.segments[seg].sides[side][i]
        return x, -y

    def add(self, w):
        pos = w.get("xy") or self.xy(w["seg"], w["side"], w["i"])
        self.state.append({"alive": True, "pos": pos, "follow": w.get("follow"), "tag": w.get("tag", "crowd"),
                           "crossing": False, "plan": w})
        return len(self.state) - 1

    def remove(self, k, reason="", occluder=None, force=False):
        self.state[k]["alive"] = False
        self.removed.append((k, reason))
        return True


def _window(sim, segs, driver, keep=(), density=50.0, route=((0, 0), (1000, 0))):
    anchors = [{"actor": driver, "route": (lambda: list(route)) if route else None}]
    return crowdwindow.CrowdWindow(sim, segs, ones(segs), PARAMS, anchors, list(keep), dict(CFG), 120.0, density,
                                   600.0, log=lambda m: None)


def test_a_walker_left_behind_reappears_at_the_window_s_far_end():
    segs = [street(0, 1000)]
    sim = _Sim(segs)
    sim.add({"xy": (300.0, 5.0), "tag": "crowd"})            # 200 m behind the car
    sim.add({"xy": (520.0, 5.0), "tag": "crowd"})            # beside it
    sim.add({"xy": (100.0, 5.0), "tag": "crowd"})            # behind, but near a pedestrian participant
    sim.add({"xy": (250.0, 5.0), "tag": "living_city"})      # the living city's own: left alone
    win = _window(sim, segs, _Actor(500, 0), keep=[_Actor(100, 10)])
    win.step(10.0)
    assert sim.removed == [(0, "recycled")] and win.recycled == 1
    new = sim.state[-1]
    assert new["alive"] and 700.0 - 1e-6 <= new["pos"][0] <= 800.0 + 1e-6
    assert new["plan"]["legs"] and new["plan"]["speed"] > 0
    assert sim.state[1]["alive"] and sim.state[2]["alive"] and sim.state[3]["alive"]
    win.step(10.5)                                            # within check_s: nothing happens
    assert len(sim.state) == 5


def test_a_group_moves_together_and_keeps_its_looks():
    segs = [street(0, 1000)]
    sim = _Sim(segs)
    sim.add({"xy": (200.0, 5.0), "group": 2, "speed_factor": 0.9})
    sim.add({"xy": (199.0, 5.0), "follow": (0, 0.7, 0.3), "bp": "walker.pedestrian.0050", "child": True})
    win = _window(sim, segs, _Actor(500, 0))
    win.step(5.0)
    assert sorted(sim.removed) == [(0, "recycled"), (1, "recycled")] and win.recycled == 2
    lead, child = sim.state[2], sim.state[3]
    assert lead["plan"]["group"] == 2 and lead["plan"]["speed_factor"] == 0.9 and lead["follow"] is None
    assert child["follow"] == (2, 0.7, 0.3) and child["plan"]["bp"] == "walker.pedestrian.0050"
    assert 690.0 <= child["pos"][0] <= 810.0


def test_nothing_moves_while_seen_or_when_the_window_is_full():
    segs = [street(0, 1000)]
    sim = _Sim(segs)
    sim.add({"xy": (300.0, 5.0)})
    sim.hidden = lambda x, y, occ: False                      # a film camera sees everything
    win = _window(sim, segs, _Actor(500, 0))
    win.step(1.0)
    assert not sim.removed
    sim.hidden = None
    full = _window(sim, segs, _Actor(500, 0), density=0.001)   # the window needs no walkers: kept in reserve
    full.step(1.0)
    assert not sim.removed
    win.last = None
    win.step(2.0)
    assert sim.removed == [(0, "recycled")]


def test_without_a_route_the_crowd_follows_within_the_radius():
    segs = [street(0, 1000)]
    sim = _Sim(segs)
    sim.add({"xy": (200.0, 5.0)})                             # 300 m behind a car driving +x with no route
    win = _window(sim, segs, _Actor(500, 0), route=None)
    win.step(1.0)
    assert win.recycled == 1
    assert 560.0 - 1e-6 <= sim.state[-1]["pos"][0] <= 620.0 + 1e-6    # radius - spawn_band .. radius, ahead


def test_density_holds_along_a_drive():
    # a car drives 1.5 km at 10 m/s past 40 walkers: the window keeps them about it
    segs = [street(0, 2000), cross_street(1000, 10, 200)]
    sim = _Sim(segs)
    rng = random.Random(3)
    for _ in range(40):
        sim.add({"xy": (rng.uniform(50, 400), 5.0)})
    car = _Actor(100, 0)
    win = _window(sim, segs, car, density=40 / (0.75 * 2 * 350 / 1000), route=((0, 0), (2000, 0)))
    for k in range(150):
        car.x = 100 + 10 * k
        win.step(float(k))
    alive = [s for s in sim.state if s["alive"]]
    assert len(alive) == 40
    near = [s for s in alive if car.x - 65 <= s["pos"][0] <= car.x + 315]
    assert len(near) >= 30 and win.recycled >= 40


def test_sumo_driver_route_follows_the_live_route():
    class E:
        def __init__(self, shape):
            self.shape = shape

        def getShape(self):
            return self.shape

    class N:
        def getEdge(self, e):
            return {"a": E([(0, 0), (100, 0)]), "b": E([(100, 0), (100, 100)])}[e]

    # before SUMO drives the car: the planned route from its start position
    assert crowdpeds.driver_route_xy(N(), (0.0, 0.0), ["a", "b"], 10.0, None, 1000)[0] == (10.0, 0.0)
    xy = crowdpeds.driver_route_xy(N(), (0.0, 0.0), ["a", "b"], 10.0, (("a", "b"), 1), 1000)   # on edge b now
    assert xy[0] == (100.0, 0.0) and xy[-1] == (100.0, -100.0)
    assert math.isclose(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(xy, xy[1:])), 100.0)
    assert crowdpeds.driver_route_xy(None, (0, 0), ["a"], 0, None, 100) == []
    assert crowdpeds.driver_route_xy(N(), (0, 0), [], 0, None, 100) == []


def test_kerbs_are_added_when_the_route_runs_past_the_prepared_ones():
    # prepared: only the first 400 m; the route then goes on for 1.6 km (a SUMO driver's next random destination)
    near, far, aside = street(0, 400, sid="near"), street(400, 2000, sid="far"), street(0, 2000, y=500, sid="aside")
    segs = [near]
    sim = _Sim(segs)
    sim.add({"xy": (50.0, 5.0)})
    car = _Actor(300, 0)
    anchors = [{"actor": car, "route": lambda: [(0, 0), (2000, 0)]}]
    extend = {"segments": [near, far, aside], "pois": [], "counts": []}
    att = ones(segs)
    win = crowdwindow.CrowdWindow(sim, segs, att, PARAMS, anchors, [], dict(CFG), 120.0, 20.0, 600.0,
                                  log=lambda m: None, extend=extend)
    win.step(1.0)
    assert [s.id for s in segs] == ["near", "far"] and win.added == 1     # not the street 500 m aside
    assert len(att) == 2 and len(att[1][0]) == far.n                       # weighted like the rest
    new = sim.state[-1]
    assert win.recycled == 1 and new["plan"]["seg"] == 1 and 500.0 <= new["pos"][0] <= 605.0
    win.last = None
    win.step(2.0)                                                         # the same route: nothing added again
    assert win.added == 1 and len(segs) == 2


def test_spawn_zone_ends_where_the_route_ends():
    segs = [street(0, 1000)]
    smp = crowdwindow.Samples(segs)
    ri = crowdwindow.RouteIndex([(0, 0), (700, 0)])          # 200 m left of a 300 m window
    anchor = {"x": 500.0, "y": 0.0, "yaw": 0.0, "ri": ri, "s": 500.0}
    xs = [smp.items[j][3] for j in crowdwindow.zone(smp, anchor, CFG, 120.0, spawn=True)]
    assert xs and min(xs) >= 600.0 - 2.5 and max(xs) <= 700.0 + CFG["band_m"]   # (+ the street past its end)
