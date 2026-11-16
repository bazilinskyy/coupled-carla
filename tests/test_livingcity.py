"""The living city's plan (livingcity.py), reactions' rules (reactions.py) and the crowd's new legs: no CARLA, no
SUMO."""
import math
import random
import sys
import types

import pytest

from coupledcarla import config, crowdpeds as cp, livingcity as lc, reactions as rx
from tests.test_crowdpeds import PARAMS, _Actor, _V, _World, _fake_carla, _street


def _lc(**over):
    s = lc.settings(config._merge(config.DEFAULTS, {"living_city": over}))
    return s


def _plan(n=200, seed=3):
    seg = _street()
    att = cp.attraction([seg], [])
    return [seg], att, cp.plan([seg], att, PARAMS, n, 300, random.Random(seed))


# -- settings ----------------------------------------------------------------------------------------------------

def test_auto_is_on_for_city_drives_and_off_for_controlled_studies():
    base = config._merge(config.DEFAULTS, {"name": "city"})
    assert lc.settings(base)["enabled"] is True
    assert lc.settings({**base, "agents": [{"id": "av1"}]})["enabled"] is False
    assert lc.settings({**base, "design": {"conditions": ["a", "b"]}})["enabled"] is False
    forced = {**base, "agents": [{"id": "av1"}], "living_city": {"enabled": True}}
    assert lc.settings(forced)["enabled"] is True
    assert lc.settings({**base, "living_city": {"enabled": False}})["enabled"] is False


def test_crowd_default_turns_the_crowd_on_only_when_none_is_set():
    tc = config._merge(config.DEFAULTS, {"name": "city"})
    assert lc.apply_crowd_default(tc, lc.settings(tc), log=lambda m: None)
    assert tc["traffic"]["walkers"]["count"] == "auto"
    tc = config._merge(config.DEFAULTS, {"name": "city", "traffic": {"walkers": {"count": 12}}})
    assert not lc.apply_crowd_default(tc, lc.settings(tc), log=lambda m: None)
    assert tc["traffic"]["walkers"]["count"] == 12


def test_every_rate_has_a_source_and_a_kind():
    for table, sources in ((lc.RATES, lc.SOURCES), (rx.RATES, rx.SOURCES)):
        for name, (value, unit, src, kind, note) in table.items():
            assert src in sources and kind in ("measured", "derived", "assumption") and note and unit, name
    assert "| `phone_share`" in lc.rates_table()


def test_rate_overrides():
    s = _lc(rates={"phone_share": 0.5, "honk_p12": 0.2})
    assert lc.rate(s, "phone_share") == 0.5 and rx.rate(s, "honk_p12") == 0.2
    assert lc.rate(s, "child_share") == lc.RATES["child_share"][0]


# -- groups, children, phones ---------------------------------------------------------------------------------------

def test_group_sizes_follow_moussaid():
    rng = random.Random(1)
    sizes = [k for _ in range(200) for k in lc.group_sizes(100, 1.0, rng)]
    people = sum(sizes)
    in_groups = sum(k for k in sizes if k > 1) / people
    assert 0.48 < in_groups < 0.60                         # 54 % at low density
    rng = random.Random(1)
    sizes = [k for _ in range(200) for k in lc.group_sizes(100, 2.0, rng)]
    assert 0.65 < sum(k for k in sizes if k > 1) / sum(sizes) < 0.77      # 71 % in a busy street
    assert lc.group_speed_factor(1) == 1.0 and lc.group_speed_factor(4) < lc.group_speed_factor(2) < 1.0
    assert abs(lc.group_speed_factor(4) - (1.26 - 0.16) / 1.22) < 1e-9


def test_dress_groups_followers_walk_with_their_leader_and_are_seeded():
    s = _lc()
    _, _, walkers = _plan()
    counts = lc.dress_groups(walkers, s, 1.0, child_bps=["walker.pedestrian.0009"], wheel_bps=["walker.x"])
    assert counts["groups"] > 10 and counts["children"] > 0 and counts["phones"] > 0
    for w in walkers:
        if "follow" in w:
            lead = walkers[w["follow"][0]]
            assert w["legs"] == lead["legs"] and w["speed_factor"] == lead["speed_factor"] < 1.0
    kids = [w for w in walkers if w.get("child")]
    assert all(w["bp"] == "walker.pedestrian.0009" and "follow" in w for w in kids)
    phones = [w for w in walkers if w.get("phone")]
    assert all(w["speed_factor"] == lc.RATES["phone_speed"][0] and "follow" not in w for w in phones)
    _, _, again = _plan()
    lc.dress_groups(again, s, 1.0, child_bps=["walker.pedestrian.0009"], wheel_bps=["walker.x"])
    assert [w.get("follow") for w in again] == [w.get("follow") for w in walkers]
    # turning phones off changes nothing else
    _, _, nophone = _plan()
    lc.dress_groups(nophone, _lc(activities={"phones": False}), 1.0, child_bps=["walker.pedestrian.0009"],
                    wheel_bps=["walker.x"])
    assert [w.get("follow") for w in nophone] == [w.get("follow") for w in walkers]
    assert not any(w.get("phone") for w in nophone)


def test_phone_share_is_near_thompson():
    s = _lc(activities={"groups": False})
    seg = _street()
    att = cp.attraction([seg], [])
    walkers = cp.plan([seg], att, PARAMS, 4000, 60, random.Random(5))
    c = lc.dress_groups(walkers, s, 1.0)
    assert 0.06 < c["phones"] / 4000 < 0.09


# -- shops and doors -------------------------------------------------------------------------------------------------

def test_shop_stops_face_the_shop():
    s = _lc(rates={"shop_stop_share": 1.0})
    segs, att, walkers = _plan(30)
    shop = (50.0, -12.0)                     # geo: beside side 0 (y = 0), away from the road (side 1 at y = 10)
    active = lc.shop_samples(segs, [shop])
    assert (0, 0) in active and (0, 1) not in active
    n = lc.add_shop_stops(walkers, segs, active, [shop], s)
    assert n > 0
    stops = [leg for w in walkers for leg in w["legs"] if leg[0] == "stop"]
    for _, (x, y), secs, face in stops:
        assert 5.0 <= secs <= 40.0
        want = math.degrees(math.atan2(-(shop[1] - -y), shop[0] - x))
        assert abs(((face - want + 180) % 360) - 180) < 1.0


def test_doors_are_on_the_building_side_and_walkers_go_in():
    segs, att, walkers = _plan(200)
    # a building 6 m beyond side 0 (geo y = -6 .. -20 is CARLA y 6 .. 20), one beyond side 1 too far away
    bld = [{"p": [[0.0, 6.0], [200.0, 6.0], [200.0, 20.0], [0.0, 20.0], [0.0, 6.0]]},
           {"p": [[0.0, -40.0], [200.0, -40.0], [200.0, -60.0], [0.0, -60.0], [0.0, -40.0]]}]
    doors = lc.doors_for(segs, lc.building_edges(bld))
    assert doors and all(side == 0 for (_, side, _) in doors)
    on_wall = [y for _, y in doors.values() if abs(y - 5.5) < 1e-6]
    assert len(on_wall) >= len(doors) - 2       # (the building's corner: its end wall)
    n = lc.add_doors_in(walkers, doors, 600, _lc())
    p = 1 - math.exp(-10 / 15)
    assert n > 0.5 * p * 200 * 0.3
    ins = [w for w in walkers if w.get("door_in")]
    assert all(w["legs"][-1] == ("vanish", "door") and w["legs"][-2][0] == "to" for w in ins)


def test_walk_from_starts_where_asked_and_door_out_times():
    segs, att, _ = _plan(1)
    w = lc.walk_from(segs, att, PARAMS, 0, 1, 20, 1.4, 120, random.Random(1))
    assert (w["seg"], w["side"], w["i"]) == (0, 1, 20) and w["speed"] == 1.4 and w["legs"]
    times = lc.door_out_times(60, 300, _lc())
    assert all(0 < t < 300 for t in times) and 10 <= len(times) <= 35          # 60 / 15 min x 5 min = 20


# -- kerbside numbers --------------------------------------------------------------------------------------------------

def test_kerbside_numbers():
    mu, sg = lc.lognormal_from_quartiles(12.3, 7.8, 21.1)
    assert abs(math.exp(mu) - 12.3) < 1e-9 and abs(math.exp(mu - 0.6745 * sg) - 7.8) < 0.4
    assert 0.5 < lc.deliveries_present(1.0, 0.0, _lc()) < 1.5          # 3 per km-h x ~16 min
    assert lc.deliveries_present(1.0, 1.0, _lc()) < lc.deliveries_present(1.0, 0.0, _lc())
    s = _lc()
    assert abs(lc.stop_waiting(10, s) - 20 / 60 * 5) < 1e-9             # half the headway below 11 min
    assert abs(lc.stop_waiting(30, s) - 20 / 60 * 5) < 1e-9             # capped above it
    assert lc.bus_dwell_s(4, 2, s) == 5 + 4 * 3.5
    assert abs(lc.departures_per_h({"centre": 10, "residential": 10}, s) - (5 + 1)) < 1e-9
    rng = random.Random(3)
    rest = [lc.remaining_dwell_s(rng, s) for _ in range(2000)]
    assert 3 * 60 < sorted(rest)[1000] < 15 * 60
    assert sum(lc.poisson(2.0, random.Random(i)) for i in range(2000)) / 2000 == pytest.approx(2.0, abs=0.15)


def test_bench_seats_face_the_side_asked():
    seats = lc.bench_seats([(10.0, 5.0, 0.0, 0.0, "SM_Bench02")], toward=lambda x, y, a, b: b)
    assert len(seats) == 2 and {round(s[0], 2) for s in seats} == {9.55, 10.45} and seats[0][3] == 270.0
    seats = lc.bench_seats([(10.0, 5.0, 0.0, 30.0, "SM_Bench01")], toward=lambda x, y, a, b: b)
    assert seats[0][3] == 120.0          # a bench with a backrest: its seat side (measured), not the road's
    assert lc.bench_share(_lc(), wet=True) == pytest.approx(0.03)


# -- reactions -----------------------------------------------------------------------------------------------------

def test_reaction_rules():
    lam = rx.honk_hazard(0.7, 2.0)
    assert 1 - math.exp(-lam * 10.0) == pytest.approx(0.7)
    assert rx.look_probability(0.9) == 0.9
    p = rx.look_probability(0.9, phone=True, odds_ratio=4.0)
    assert p == pytest.approx(2.25 / 3.25)
    t, d = rx.ttc(0, 0, 10, 0, 20, 0, 0, 0, 2.0)
    assert t == pytest.approx(1.8) and d == 20
    assert rx.ttc(0, 0, -10, 0, 20, 0, 0, 0, 2.0)[0] is None
    assert rx.ttc(0, 0, 10, 0, 20, 3.5, -10, 0, 2.0)[0] is None       # oncoming in the next lane: passes clear
    assert rx.ttc(0, 0, 10, 0, 20, 1.0, -10, 0, 2.0)[0] < 1.0
    t, d = rx.closest(0, 0, 0, 0, 20, 3.0, -10, 0)        # a car passing a kerb walker 3 m away in 2 s
    assert t == pytest.approx(2.0) and d == pytest.approx(3.0)
    assert rx.closest(0, 0, 0, 0, 20, 3.0, 10, 0)[0] is None
    assert rx.behind(0, 0, 0, -8, 0.5, 5) == 8 and rx.behind(0, 0, 0, 8, 0, 0) is None
    assert rx.behind(0, 0, 0, -8, 0, 180) is None


# -- the crowd's new legs ------------------------------------------------------------------------------------------

def _sim(monkeypatch, plan):
    fake = _fake_carla()
    fake.Rotation = lambda pitch=0.0, yaw=0.0, roll=0.0: types.SimpleNamespace(yaw=yaw)
    monkeypatch.setitem(sys.modules, "carla", fake)
    world = _World()
    for a in world.walkers:
        a.destroy = lambda: None
    sim = cp.CrowdSim(world, None, [_street(n=9, width=8.0, crossings=(4,))], plan, {}, log=lambda m: None)
    for a in sim.actors:
        a.destroy = lambda: None
    return sim, world


def _tick(sim, t, dt=0.1):
    sim.step(t)
    for a in sim.walkers:
        if a.ctl is not None and a.ctl.speed > 0.06:
            a.loc = _V(a.loc.x + a.ctl.direction.x * a.ctl.speed * dt, a.loc.y + a.ctl.direction.y * a.ctl.speed * dt)


def test_crowd_stops_waits_and_vanishes(monkeypatch):
    plan = [{"xy": (0.0, 2.0), "speed": 1.5, "legs": [("to", [(3.0, 2.0)]), ("stop", None, 2.0, 90.0),
                                                       ("wait", "bus"), ("to", [(3.0, 5.0)]), ("vanish", "bus")]}]
    sim, _ = _sim(monkeypatch, plan)
    stood, gone = [], []
    sim.on_stand = lambda k, a, st, kind: stood.append(kind)
    sim.on_vanish = lambda k, a, why: gone.append(why)
    t = 0.0
    for _ in range(60):
        _tick(sim, t)
        t += 0.1
    (w,) = sim.walkers
    assert abs(w.loc.x - 3.0) < 0.9 and stood == ["stop", "wait"] and sim.state[0]["standing"] == "wait"
    sim.release(0)
    for _ in range(60):
        _tick(sim, t)
        t += 0.1
    assert sim.walkers == [] and gone == ["bus"] and sim.on_road() == []


def test_followers_walk_beside_their_leader(monkeypatch):
    plan = [{"xy": (0.0, 0.0), "speed": 1.2, "legs": [("to", [(20.0, 0.0)])]},
            {"xy": (0.0, 0.75), "speed": 1.2, "follow": (0, 0.75, 0.0), "legs": [("to", [(20.0, 0.0)])]}]
    sim, _ = _sim(monkeypatch, plan)
    t = 0.0
    for _ in range(80):
        _tick(sim, t)
        t += 0.1
    lead, mate = sim.walkers
    assert lead.loc.x > 8.0 and abs(mate.loc.x - lead.loc.x) < 0.8 and abs(mate.loc.y - 0.75) < 0.5
    sim.remove(0, "door")
    _tick(sim, t)
    assert sim.walkers == []                  # the group goes in together


def test_redirect_and_hurry(monkeypatch):
    plan = [{"xy": (0.0, 0.0), "speed": 1.0, "legs": [("to", [(50.0, 0.0)])]}]
    sim, _ = _sim(monkeypatch, plan)
    sim.state[0]["hurry"], sim.state[0]["hurry_until"] = 1.44, 10.0
    _tick(sim, 0.0)
    assert sim.walkers[0].ctl.speed == pytest.approx(1.44)
    sim.redirect(0, [("to", [(0.0, 10.0)])])
    _tick(sim, 20.0)
    assert sim.walkers[0].ctl.direction.y > 0.9 and sim.walkers[0].ctl.speed == pytest.approx(1.0)


def test_scenery_take_at_removes_the_item_for_good(monkeypatch):
    from coupledcarla import scenery
    fake = types.ModuleType("carla")
    fake.command = types.SimpleNamespace(DestroyActor=lambda i: ("destroy", i))
    monkeypatch.setitem(sys.modules, "carla", fake)
    sc = scenery.Scenery.__new__(scenery.Scenery)
    it = [0, 10.0, 20.0, 0.0, 90.0, 1.0, "parked"]
    sent = []
    sc.cell, sc.cells, sc.actor_of, sc.loaded, sc.n = 100.0, {(0, 0): [it]}, {id(it): 77}, {(0, 0): [77]}, 1
    sc.client = types.SimpleNamespace(apply_batch=lambda cmds: sent.extend(cmds))
    assert sc.take_at(10.2, 19.8) and sent == [("destroy", 77)] and sc.cells[(0, 0)] == [] and sc.n == 0
    assert not sc.take_at(10.2, 19.8)


def test_unused_imports_are_fine():
    assert _Actor is not None


def test_honk_draws_give_the_measured_share_within_12_s():
    s = _lc()
    lam = rx.honk_hazard(rx.rate(s, "honk_p12"), rx.rate(s, "honk_grace_s"))
    honked = 0
    for ep in range(600):              # checks every 0.1 s, one draw each (as Reactions does)
        t = 2.0
        while t < 12.0:
            if lc.rng_for(s, "horn", f"role{ep}", round(t, 2)).random() < lam * 0.1:
                honked += 1
                break
            t += 0.1
    assert 0.62 < honked / 600 < 0.78


def test_sound_events_reach_the_soundscape():
    from coupledcarla import soundrec, stationux
    got = []
    ha = soundrec.HostAudio.__new__(soundrec.HostAudio)
    ha.scape = types.SimpleNamespace(event=lambda *a: got.append(a))
    ha.sound_event({"name": "horn", "actor_id": 7, "kind": "double"})
    ha.sound_event({"name": "door", "actor_id": 7})                # not a sound the soundscape makes of others
    assert got == [("horn", 7, "double")]
    assert "horn" in stationux.SOUND_EVENTS and "bell" in stationux.SOUND_EVENTS


def test_sumo_car_crossing_a_participants_path_is_found():
    # the participant heads east at 9 m/s from (0, 0); a SUMO car heads north (CARLA -y) at 7 m/s, 20 m to its east
    got = rx.crossing(0, 0, 9.0, 0, 20.0, 14.0, 0, -7.0)
    assert got is not None
    tp, ts, b = got
    assert tp == pytest.approx(20 / 9) and b == pytest.approx(14.0) and ts == pytest.approx(2.0)
    assert rx.crossing(0, 0, 9.0, 0, -20.0, 3.5, 9.0, 0) is None          # the next lane, same way: SUMO's job
    assert rx.crossing(0, 0, 9.0, 0, 20.0, -14.0, 0, -7.0) is None        # driving away from the crossing


def test_segment_box_occlusion():
    bus = (10.0, 0.0, 0.0, 6.0, 1.3)                       # a bus at x 4..16, y -1.3..1.3
    assert lc._segment_hits_box(10.0, -20.0, 10.0, 2.0, bus)          # seen through the bus: hidden
    assert not lc._segment_hits_box(10.0, -20.0, 20.0, 2.0, bus)      # past its end: seen
    assert not lc._segment_hits_box(10.0, 20.0, 10.0, 2.0, bus)       # from the walker's side: seen


def test_walkers_leave_only_unseen(monkeypatch):
    plan = [{"xy": (0.0, 0.0), "speed": 1.5, "legs": [("to", [(2.0, 0.0)]), ("vanish", "door")]}]
    sim, _ = _sim(monkeypatch, plan)
    seen = {"on": True}
    sim.hidden = lambda x, y, occ=None: not seen["on"]
    t = 0.0
    for _ in range(40):
        _tick(sim, t)
        t += 0.1
    assert len(sim.walkers) == 1 and sim.state[0]["leaving"] == ("door", None)    # at the door, waiting
    seen["on"] = False
    _tick(sim, t)
    assert sim.walkers == []


def test_autopilot_slows_before_a_turn(monkeypatch):
    if "carla" not in sys.modules:
        try:
            import carla  # noqa: F401
        except ImportError:
            monkeypatch.setitem(sys.modules, "carla", _fake_carla())
    from coupledcarla import inputs
    ap = inputs.Autopilot.__new__(inputs.Autopilot)
    ap.target_speed = 40.0

    def wp(x, y, yaw):
        return (types.SimpleNamespace(transform=types.SimpleNamespace(location=_V(x, y), rotation=types.SimpleNamespace(yaw=yaw))), None)

    straight = [wp(2.0 * i, 0.0, 0.0) for i in range(1, 40)]
    # 20 m straight, then a 90 deg left turn of 10 m radius (CARLA yaw falls to the left)
    turn = [wp(2.0 * i, 0.0, 0.0) for i in range(1, 11)] + \
        [wp(20 + 10 * math.sin(a), -10 + 10 * math.cos(a), -math.degrees(a)) for a in
         [k * math.pi / 2 / 8 for k in range(1, 9)]]
    def at(x, y, yaw=0.0):
        return types.SimpleNamespace(get_transform=lambda: types.SimpleNamespace(
            location=_V(x, y), rotation=types.SimpleNamespace(yaw=yaw)))

    car = at(0.0, 0.0)
    ap.agent = types.SimpleNamespace(get_local_planner=lambda: types.SimpleNamespace(_waypoints_queue=straight))
    assert ap._curve_speed(car) == 40.0
    # 10 m before the turn: braking at 2.5 m/s^2 to 5 m/s at its start allows sqrt(25 + 50) m/s = 31 km/h
    car10 = at(10.0, 0.0)
    ap.agent = types.SimpleNamespace(get_local_planner=lambda: types.SimpleNamespace(_waypoints_queue=turn[5:]))
    assert 27.0 <= ap._curve_speed(car10) <= 34.0
    # at the turn itself: sqrt(2.5 m/s^2 x 10 m) = 5 m/s = 18 km/h
    ap.agent = types.SimpleNamespace(get_local_planner=lambda: types.SimpleNamespace(_waypoints_queue=turn[10:]))
    assert ap._curve_speed(at(20.0, 0.0)) < 23.0
    # cutting the corner: its plan only has the road after the turn, 8 m away at 60 deg off its heading
    ap.agent = types.SimpleNamespace(get_local_planner=lambda: types.SimpleNamespace(_waypoints_queue=[
        wp(20 + 4.0, -6.93, -90.0), wp(20 + 4.0, -8.93, -90.0), wp(20 + 4.0, -10.93, -90.0)]))
    assert ap._curve_speed(at(20.0, 0.0)) < 13.0


def test_autopilot_steering_is_limited_by_speed(monkeypatch):
    if "carla" not in sys.modules:
        try:
            import carla  # noqa: F401
        except ImportError:
            monkeypatch.setitem(sys.modules, "carla", _fake_carla())
    from coupledcarla import inputs
    ap = inputs.Autopilot.__new__(inputs.Autopilot)
    ap._geom = (2.9, 70.0)
    assert ap._steer_limit(None, 0.5) == 1.0
    lim = ap._steer_limit(None, 3.0)               # 11 km/h: radius >= 9 / 3.5 = 2.6 m, about 48 deg of 70
    r = 2.9 / math.tan(math.radians(lim * 70.0))
    assert 3.0 ** 2 / r == pytest.approx(3.5, rel=0.01)
    assert ap._steer_limit(None, 11.0) < 0.1
