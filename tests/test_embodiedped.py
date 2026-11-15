"""embodiedped: crowd crossing decisions from what each walker sees. A fake embodied-human (exact, quick eyes) tests
the gaze, belief and decision logic without the package; the real one, when installed, tests that the walkers'
decisions depend on a car's speed the way people's do (no network, no SUMO, no CARLA)."""
import math
import sys
import types

import pytest

from coupledcarla import crowdpeds as cp
from coupledcarla import embodiedped as ep

A, B = (10.0, 0.0), (10.0, -8.0)        # an 8 m crossing; cars drive along y = -2 (one way) or -6 (the other)


# --- a fake embodied-human: saccades of 50 ms latency and 30 ms, noise-free vision within 90 deg ----------------------

class _Eyes:
    def __init__(self, **p):
        self.rng = None
        self.reset()

    def reset(self, rng=None, gaze=(0.0, 0.0)):
        self.gaze, self._pending, self._land, self.onset = tuple(gaze), None, None, 0.0
        self.saccades = []

    def command(self, target, t):
        self._pending = (t + 0.05, tuple(target))

    @property
    def busy(self):
        return self._pending is not None or self._land is not None

    def step(self, t):
        if self._land is not None and t >= self._land[0]:
            self.gaze, self.onset, self._land = self._land[1], self._land[0], None
        if self._land is None and self._pending is not None and t >= self._pending[0]:
            self._land, self._pending = (self._pending[0] + 0.03, self._pending[1]), None
            self.saccades.append(t)
        return types.SimpleNamespace(gaze=self.gaze, in_saccade=self._land is not None, fixation_onset=self.onset)


class _Objects:
    def __init__(self, **p):
        pass

    def reset(self, rng=None):
        pass

    def observe(self, items, gaze, suppressed=False):
        if suppressed:
            return {}
        out = {}
        for k, it in items.items():
            e = math.hypot(it.position[0] - gaze[0], it.position[1] - gaze[1])
            if e <= 90.0:
                out[k] = types.SimpleNamespace(value=it.value, sd=it.foveal_sd * (1 + e / 2.0), eccentricity=e)
        return out


class _Response:
    def __init__(self, **p):
        pass

    def reset(self, rng=None):
        pass

    def visual_latency(self, e):
        return 0.1

    def respond(self, signals, load=()):
        return types.SimpleNamespace(time=signals[0].onset + 0.3)


def fake_eh():
    choice = types.SimpleNamespace
    cfg = types.SimpleNamespace(eyes=choice(name="e", params={}), objects=choice(name="o", params={}),
                                response=choice(name="r", params={}))

    class Population:
        def __init__(self, variations=()):
            pass

        def sample(self, rng):
            return cfg
    made = {"eyes": _Eyes, "objects": _Objects, "response": _Response}
    return types.SimpleNamespace(create=lambda cat, name, **p: made[cat](**p), Population=Population,
                                 BASELINE_VARIATIONS=(),
                                 Item=lambda position, value, foveal_sd: types.SimpleNamespace(
                                     position=position, value=value, foveal_sd=foveal_sd),
                                 Signal=lambda modality, onset, latency: types.SimpleNamespace(
                                     modality=modality, onset=onset, latency=latency))


def model(**over):
    return ep.EmbodiedCrossing({**ep.DEFAULTS, "model": "embodied", **over}, eh=fake_eh())


def run_until_go(m, cars, t_c=7.0, t_end=20.0, dt=0.08, key=1, marked=False):
    """cars: [(id, x0, y, vx)] moving along x; returns the time the walker steps out (None: still waiting)."""
    t = 0.0
    while t <= t_end:
        veh = [(x0 + vx * t, y, vx, 0.0) for _, x0, y, vx in cars]
        if m.decide(key, t, A, B, t_c, veh, [c[0] for c in cars], marked):
            return t
        t += dt
    return None


def test_options_and_make_fail_soft(monkeypatch):
    assert ep.options({})["model"] == "off" and ep.options({"perception": "embodied"})["model"] == "embodied"
    assert ep.make({"traffic": {"walkers": {}}}) is None
    said = []
    monkeypatch.setitem(sys.modules, "embodied_human", None)         # not installed: import fails
    assert ep.make({"traffic": {"walkers": {"perception": {"model": "embodied"}}}}, log=said.append) is None
    assert "needs embodied-human" in said[0]
    assert ep.make({"traffic": {"walkers": {"perception": "telepathy"}}}, log=said.append) is None
    m = ep.make({"traffic": {"walkers": {"perception": "embodied"}}}, eh=fake_eh())
    assert isinstance(m, ep.EmbodiedCrossing)


def test_approach_geometry():
    on, d, closing, az = ep.approach(A, B, -30.0, -2.0, 14.0, 0.0)    # from the walker's left (x < 10)
    assert not on and d == pytest.approx(40.0) and closing == pytest.approx(14.0) and az < -45
    *_, az_r = ep.approach(A, B, 50.0, -6.0, -14.0, 0.0)                 # from the right
    assert az_r > 45
    assert ep.approach(A, B, -30.0, -2.0, -14.0, 0.0) is None          # driving away
    assert ep.approach(A, B, 10.0, -4.0, 0.0, 0.0)[0] is True          # standing on the crossing


def test_empty_road_looks_both_ways_then_reacts():
    m = model()
    t = run_until_go(m, [])
    w = m.walkers[1]
    assert t is not None and m.last["looks"] >= 2 and set(w.seen_side) == {-1, 1}
    assert t >= 0.3 and m.counts["crossed"] == 1 and m.last["true_gap"] is None


def test_waits_for_a_close_car_then_goes_after_it():
    m = model()
    t = run_until_go(m, [(7, -40.0, -2.0, 13.0)])       # reaches the crossing at ~3.8 s, gap < 7 s: wait
    assert t is not None and t > 3.5
    assert any(n == "crowd_notices" and " saw 7 " in d for n, d in m.events)


def test_a_far_fast_car_whose_approach_is_not_seen_is_judged_by_the_prior_speed():
    # 60 km/h from 130 m: it looms below the threshold until ~100 m, so its speed is unseen and the walker assumes
    # 30 km/h; it steps out with a true gap under t_c, which the crowd's gap check (true speed) never does
    m = model()
    t = run_until_go(m, [(3, 10.0 - 130.0, -2.0, 16.7)])
    assert t is not None and t < 2.5
    assert m.last["true_gap"] < 7.0 and m.last["believed_gap"] >= 7.0
    veh = [(10.0 - 130.0 + 16.7 * t, -2.0, 16.7, 0.0)]
    assert not cp.gap_ok(A, B, veh, 7.0)
    # the same car seen clearly (a lower looming threshold): the walker waits for it
    assert run_until_go(model(looming_threshold=1e-6), [(3, 10.0 - 130.0, -2.0, 16.7)], t_end=2.5) is None


def test_marked_crossing_counts_cars_that_can_stop():
    car = [(5, 10.0 - 60.0, -2.0, 10.0)]             # 6 s away: under t_c, but 60 m is ample to stop from 36 km/h
    assert run_until_go(model(), car, t_end=3.0) is None
    assert run_until_go(model(), car, t_end=3.0, marked=True) is not None


def test_gaze_log_and_replay(tmp_path):
    m = model()
    m.open_log(tmp_path)
    seg = _street()
    plan = [{"seg": 0, "side": 0, "i": 4, "speed": 1.6, "legs": [("cross", 0, 0, 4, "midblock")]}]
    frames = [(k * 0.1, [(-29.0 + 13.0 * k * 0.1, -4.0, 13.0, 0.0)] if k * 0.1 < 3.0 else [], [11] if k * 0.1 < 3.0
               else []) for k in range(300)]
    (r,) = cp.replay([seg], plan, frames, 30.0, dt=0.1, decider=m)
    m.close()
    assert not r["censored"] and r["wait"] >= 2.5
    rows = (tmp_path / "crowd_gaze.csv").read_text().splitlines()
    assert rows[0].startswith("timestamp;walker;gaze_deg") and len(rows) > 20 and rows[-1].endswith(";go")


def _street(n=9, width=8.0, crossings=(4,), step=2.5):
    """A straight street along x (geo.net: y up; CARLA y = -geo y), as in test_crowdpeds."""
    return cp.Segment("s", [(i * step, 0.0) for i in range(n)], [(i * step, width) for i in range(n)], step,
                      crossings, (True, True))


def test_crowdsim_uses_the_decider(monkeypatch):
    sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
    from test_crowdpeds import _Actor, _V, _World, _fake_carla
    monkeypatch.setitem(sys.modules, "carla", _fake_carla())
    seg = _street()
    plan = [{"seg": 0, "side": 0, "i": 4, "speed": 1.6, "legs": [("cross", 0, 0, 4, "crosswalk")]}]
    world = _World()
    sim = cp.CrowdSim(world, None, [seg], plan, {}, log=lambda m: None)
    sim.decider = model()
    world.vehicles.append(_Actor(_V(-200.0, -4.0), _V(1.0, 0.0), "vehicle.tesla.model3"))   # far and slow
    t = 0.0
    while not sim.on_road() and t < 5.0:
        sim.step(t)
        t += 0.08
    assert sim.on_road() and t > 0.3                  # looked both ways and reacted first, unlike gap_ok's instant go
    name, detail = sim.events[-1]
    assert name == "crowd_crosses" and "believed gap" in detail and "looks" in detail


def test_real_embodied_human_speed_effect():
    """With embodied-human's own eyes and vision: a car at 60 km/h is accepted at gaps under t_c far more often than
    one at 30 km/h, both first seen 9 s away: people accept shorter time gaps from faster cars (Lobjois & Cavallo
    2007); the crowd's gap check never accepts a gap under t_c."""
    pytest.importorskip("embodied_human")
    under = {}
    for kmh in (30, 60):
        v, under[kmh] = kmh / 3.6, 0
        for key in range(20):
            m = ep.EmbodiedCrossing({**ep.DEFAULTS, "model": "embodied", "seed": key})
            t = run_until_go(m, [(1, 10.0 - 9.0 * v, -2.0, v)], t_end=9.0, key=key)
            under[kmh] += t is not None and (9.0 - t) < 7.0
    assert under[60] >= under[30] + 6, under          # 14 against 5 of 20 (seeds 0-19)
