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
    seats = lc.bench_seats([(10.0, 5.0, 0.0, 0.0, "SM_Bench01")], toward=lambda x, y, a, b: b)
    assert len(seats) == 2 and {round(s[0], 2) for s in seats} == {9.55, 10.45} and seats[0][3] == 270.0
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
