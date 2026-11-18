"""City-sized maps: SUMO cut to the participants' area (sumocrop.py) and the pipelined bridge (bridge/pipeline.py):
SUMO every few ticks with interpolated cars, in a thread with the same results, asked only about what is near."""
import math
import re
import threading
import types
from pathlib import Path

import pytest

from test_bridge_speed import mods  # noqa: F401 - the fixture (stand-ins for carla/traci where missing)

from coupledcarla import config, sumocrop
from coupledcarla.bridge import pipeline

ROOT = Path(__file__).resolve().parent.parent
EHV_CITY = [51.395, 5.380, 51.465, 5.509]


# -- pipeline helpers ---------------------------------------------------------------------------------------------

def test_ratio_for():
    assert pipeline.ratio_for(None, 0.02) == 1
    assert pipeline.ratio_for(0.02, 0.02) == 1
    assert pipeline.ratio_for(0.1, 0.02) == 5
    with pytest.raises(ValueError):
        pipeline.ratio_for(0.03, 0.02)


def test_angle_lerp_takes_the_short_way_round():
    assert pipeline.angle_lerp(350.0, 10.0, 0.5) % 360.0 == pytest.approx(0.0)
    assert pipeline.angle_lerp(10.0, 350.0, 0.25) == pytest.approx(5.0)
    assert pipeline.angle_lerp(0.0, 90.0, 1.0) == pytest.approx(90.0)


def _actor(x, y=0.0, yaw=90.0, speed=10.0):
    loc = types.SimpleNamespace(x=x, y=y, z=0.0)
    rot = types.SimpleNamespace(pitch=0.0, yaw=yaw, roll=0.0)
    Actor = __import__("collections").namedtuple("A", "transform speed signals")
    return Actor(types.SimpleNamespace(location=loc, rotation=rot), speed, 8)


def _make(x, y, z, pitch, yaw):
    return types.SimpleNamespace(location=types.SimpleNamespace(x=x, y=y, z=z),
                                 rotation=types.SimpleNamespace(pitch=pitch, yaw=yaw, roll=0.0))


def test_lerp_actor():
    a, b = _actor(0.0, speed=10.0), _actor(10.0, yaw=100.0, speed=12.0)
    m = pipeline.lerp_actor(a, b, 0.4, _make)
    assert m.transform.location.x == pytest.approx(4.0) and m.transform.rotation.yaw == pytest.approx(94.0)
    assert m.speed == pytest.approx(10.8) and m.signals == 8
    assert pipeline.lerp_actor(None, b, 0.4, _make) is b
    assert pipeline.lerp_actor(a, b, 1.0, _make) is b
    assert pipeline.lerp_actor(a, _actor(100.0), 0.4, _make).transform.location.x == 100.0    # a teleport


def test_near_ids_and_accumulator():
    assert pipeline.near_ids({"a": (0, 0), "b": (100, 0), "c": (40, 30)}, [(0, 0)], 50.0) == {"a", "c"}
    acc = pipeline.Accumulator()
    acc.add({1, 2}, set())
    acc.add({3}, {2})
    acc.add(set(), {9})
    assert acc.take({1, 3}) == ({1, 3}, {2, 9})
    assert acc.take(set()) == (set(), set())


# -- the SUMO area ------------------------------------------------------------------------------------------------

def test_crop_radius_auto_only_for_city_sized_maps():
    assert sumocrop.box_km2(EHV_CITY) == pytest.approx(69.8, abs=0.5)
    assert sumocrop.radius_for("auto", EHV_CITY) == 1500.0
    assert sumocrop.radius_for("auto", [51.43, 5.46, 51.45, 5.49]) == 0.0     # EhvCentre-sized
    assert sumocrop.radius_for(0, EHV_CITY) == 0.0
    assert sumocrop.radius_for(None, EHV_CITY) == 0.0
    assert sumocrop.radius_for(800, [51.43, 5.46, 51.45, 5.49]) == 800.0


def test_only_the_city_experiments_are_cropped_by_default():
    """Defaults unchanged for the other maps: no experiment but the city twins has a box of AUTO_KM2 or more."""
    big = []
    for f in sorted((ROOT / "experiments").glob("*.yaml")):
        if f.stem.startswith("lin_"):      # a machine's own untracked copies
            continue
        m = re.search(r"^\s*bbox:\s*\[([-\d.,\s]+)\]", f.read_text(), re.M)
        if m and sumocrop.radius_for("auto", [float(v) for v in m.group(1).split(",")]):
            big.append(f.stem)
    assert big and all(n.startswith("eindhoven_city") for n in big)


def test_centres_from_participants_routes_and_agents():
    tc = {"participants": [{"spawn": {"lat": 51.44, "lon": 5.47}, "route": [[51.45, 5.48], [51.46, 5.49]]},
                           {"spawn": {"x": 3, "y": 4}, "destination": {"lat": 51.41, "lon": 5.41}}],
          "agents": [{"path": [[51.43, 5.46], [51.431, 5.461]]}]}
    assert sumocrop.centres(tc) == [(51.44, 5.47), (51.45, 5.48), (51.41, 5.41), (51.43, 5.46)]


def test_area_polygon_covers_each_disc():
    poly = sumocrop.area_polygon([(0.0, 0.0), (3000.0, 0.0)], 1500.0)

    def inside(px, py):     # convex, counter-clockwise
        return all((b[0] - a[0]) * (py - a[1]) - (b[1] - a[1]) * (px - a[0]) >= -1e-6
                   for a, b in zip(poly, poly[1:] + poly[:1]))
    assert inside(0, 1490) and inside(1500, 1400) and inside(4490, 0) and inside(-1490, 0)
    assert not inside(0, 1600) and not inside(4600, 0)
    arg = sumocrop.boundary_arg(poly).split(",")
    assert len(arg) == 2 * (len(poly) + 1) and arg[:2] == arg[-2:]


class _Net:
    def __init__(self, lanes, tls):
        self._lanes, self._tls = lanes, tls

    def getEdges(self):
        edges = {}
        for lane in self._lanes:
            edges.setdefault(lane.rsplit("_", 1)[0], []).append(lane)
        return [types.SimpleNamespace(getID=lambda e=e: e,
                                      getLanes=lambda ls=ls: [types.SimpleNamespace(getID=lambda i=i: i) for i in ls])
                for e, ls in edges.items()]

    def getTrafficLights(self):
        return [types.SimpleNamespace(getID=lambda t=t: t,
                                      getConnections=lambda n=n: [(None, None, k) for k in range(n)])
                for t, n in self._tls.items()]


def test_filter_additional_keeps_what_the_area_has(tmp_path):
    src = tmp_path / "add.xml"
    src.write_text("""<additional>
  <variableSpeedSign id="v1" lanes="a_0 a_1 gone_0"><step time="0" speed="8.3"/></variableSpeedSign>
  <variableSpeedSign id="v2" lanes="gone_0"><step time="0" speed="8.3"/></variableSpeedSign>
  <busStop id="s1" lane="a_0" startPos="1" endPos="20"/>
  <busStop id="s2" lane="gone_1" startPos="1" endPos="20"/>
  <tlLogic id="J1" type="static" programID="0_scramble" offset="0"><phase duration="5" state="GGrr"/></tlLogic>
  <tlLogic id="J2" type="static" programID="0_scramble" offset="0"><phase duration="5" state="GGrr"/></tlLogic>
  <tlLogic id="J9" type="static" programID="0_scramble" offset="0"><phase duration="5" state="Gr"/></tlLogic>
</additional>""")
    out = tmp_path / "out.xml"
    kept, dropped = sumocrop.filter_additional(src, _Net(["a_0", "a_1", "b_0"], {"J1": 4, "J2": 3}), out)
    assert (kept, dropped) == (3, 4)
    text = out.read_text()
    assert 'lanes="a_0 a_1"' in text and 'id="s1"' in text and 'id="J1"' in text
    assert all(f'id="{i}"' not in text for i in ("v2", "s2", "J2", "J9"))


# -- config ---------------------------------------------------------------------------------------------------------

def test_defaults_keep_the_plain_bridge():
    sim, tr = config.DEFAULTS["simulation"], config.DEFAULTS["traffic"]
    assert sim["sumo_step"] is None and sim["sumo_thread"] is False and sim["sumo_near"] is False
    assert tr["crop_radius_m"] == "auto" and sim["scenery"]["radius_m"] == 400.0


@pytest.mark.parametrize("name", ["eindhoven_city_twin", "eindhoven_city_real", "eindhoven_city_quick"])
def test_city_experiments_turn_the_pipeline_on(name):
    cfg = config.load(ROOT / "experiments" / f"{name}.yaml")
    sim = cfg["simulation"]
    assert sim["sumo_step"] == 0.1 and sim["sumo_thread"] is True and sim["sumo_near"] is True
    assert sim["scenery"]["radius_m"] == 200 and sim["scenery"]["kinds"]      # merged with the defaults
    assert cfg["traffic"]["crop_radius_m"] == 1500
    assert pipeline.ratio_for(sim["sumo_step"], sim["step"]) == 5


# -- the pipelined bridge -------------------------------------------------------------------------------------------

class _StepSumo:
    """SUMO stand-in: each vehicle moves vx m per step along x; departures by step number."""

    def __init__(self, mods, depart, vx=10.0):
        self.mods, self.depart, self.vx = mods, depart, vx
        self.pos, self.subs, self.steps = {}, {}, 0
        self.spawned_actors, self.destroyed_actors = set(), set()
        self.frozen, self._stepper, self.focus_calls = None, None, []
        self.traffic_light_ids, self.net, self._focus = set(), None, None
        self.thread_names = []

    def tick(self):
        self.thread_names.append(threading.current_thread().name)
        self.steps += 1
        for i, (x, y) in self.pos.items():
            self.pos[i] = (x + self.vx, y)
        new = self.depart.get(self.steps, {})
        self.pos.update(new)
        self.spawned_actors, self.destroyed_actors = set(new), set()

    def start_step(self):
        self._stepper = threading.Thread(target=self.tick, name="sumo-step")
        self._stepper.start()

    def wait_step(self):
        if self._stepper is not None:
            self._stepper.join()
            self._stepper = None

    def subscribe(self, i):
        self.subs[i] = "full"

    def subscribe_pose(self, i):
        self.subs[i] = "full" if self.subs.get(i) == "full" else "pose"

    def unsubscribe(self, i):
        self.subs.pop(i, None)

    def _tf(self, i):
        c = self.mods.carla
        return c.Transform(c.Location(self.pos[i][0], self.pos[i][1], 0.0), c.Rotation(0.0, 90.0, 0.0))

    def get_pose(self, i):
        return self._tf(i), self.mods.carla.Vector3D(2.0, 0.0, 0.0)

    def read_actor(self, i):
        if self.subs.get(i) != "full":
            raise KeyError(i)
        return self.mods.ss.SumoActor("car", None, self._tf(i), 0, self.mods.carla.Vector3D(2.0, 1.0, 0.8),
                                      (1, 2, 3), self.vx / 0.1)

    def get_actor(self, i):
        return self.frozen.get(i) if self.frozen is not None else self.read_actor(i)

    def set_focus(self, points, radius):
        self.focus_calls.append(list(points))
        self._focus = (points, radius)

    def near_poses(self):
        if self._focus is None:
            return {}
        pts, r = self._focus
        return {i: p for i, p in self.pos.items() if any(math.hypot(p[0] - x, p[1] - y) <= r for x, y in pts)}


class _RecCarla:
    def __init__(self):
        self.next_id, self.placed, self.log = 100, {}, []
        self.spawned_actors, self.destroyed_actors, self._active_actors = set(), set(), set()
        self.traffic_light_ids, self._physics, self.step_length = set(), set(), 0.02

    def spawn_actor(self, bp, tf):
        self.next_id += 1
        return self.next_id

    def destroy_actor(self, i):
        self.placed.pop(i, None)

    def current_lights(self, i):
        return 0

    def place_vehicle(self, i, tf, lights=None):
        self.placed[i] = round(tf.location.x, 6)

    def follow_vehicle(self, i, tf, speed, yaw_rate, length, lights=None):
        self.place_vehicle(i, tf)

    def tick(self):
        self.log.append(dict(self.placed))


def _bridge(mods, monkeypatch, sumo, ratio, threaded=False, near=False, radius=100.0):
    from test_bridge_speed import _sync
    c = mods.carla
    monkeypatch.setattr(mods.sync.BridgeHelper, "offset", (0.0, 0.0))
    monkeypatch.setattr(mods.sync.BridgeHelper, "get_carla_blueprint", staticmethod(lambda a, color=False: "bp"))
    carla_sim = _RecCarla()
    s = _sync(mods, sumo, carla_sim, c.Location(0.0, 0.0, 0.0))
    s.radius = radius
    s.configure_pipeline(ratio, threaded, near)
    return s, carla_sim


def test_sumo_steps_every_fifth_tick_and_cars_glide_between(mods, monkeypatch):
    sumo = _StepSumo(mods, {1: {"a": (0.0, 0.0)}})
    s, carla_sim = _bridge(mods, monkeypatch, sumo, 5)
    for _ in range(20):
        s.tick()
    assert sumo.steps == 1 + 4      # the first step at once, then one at each of the 4 boundaries
    xs = [frame[next(iter(frame))] for frame in carla_sim.log if frame]
    steps = [round(b - a, 6) for a, b in zip(xs, xs[1:])]
    assert set(steps[5:]) == {2.0}            # 10 m per SUMO step = 2 m per tick, no jumps
    assert s.sumo_stepped is False      # the 20th tick had no SUMO step


def test_threaded_sumo_gives_the_same_placements(mods, monkeypatch):
    runs = []
    for threaded in (False, True):
        sumo = _StepSumo(mods, {1: {"a": (0.0, 0.0)}, 3: {"b": (30.0, 0.0)}, 6: {"c": (500.0, 0.0)}})
        s, carla_sim = _bridge(mods, monkeypatch, sumo, 5, threaded=threaded)
        for _ in range(60):
            s.tick()
        s.sumo.wait_step()
        runs.append((carla_sim.log, sumo.steps, "sumo-step" in sumo.thread_names))
    assert runs[0][0] == runs[1][0] and runs[0][1] == runs[1][1]
    assert runs[0][2] is False and runs[1][2] is True


def test_hooks_run_at_each_sumo_step_before_it_starts(mods, monkeypatch):
    sumo = _StepSumo(mods, {})
    s, _ = _bridge(mods, monkeypatch, sumo, 5, threaded=True)
    seen = []
    s.hooks.append(lambda: seen.append((sumo.steps, sumo._stepper is None)))
    for _ in range(15):
        s.tick()
    s.sumo.wait_step()
    assert seen == [(1, True), (2, True), (3, True)]


def test_near_asks_only_about_vehicles_around_the_participant(mods, monkeypatch):
    sumo = _StepSumo(mods, {1: {"near": (50.0, 0.0), "far": (2000.0, 0.0)}}, vx=0.0)
    s, carla_sim = _bridge(mods, monkeypatch, sumo, 5, near=True)
    for _ in range(15):
        s.tick()
    assert set(s.sumo2carla_ids) == {"near"}
    assert "far" not in sumo.subs                      # never subscribed: no per-vehicle subscription in the city
    assert sumo.focus_calls[0] == [(0.0, 0.0)]
    sumo.pos["near"] = (400.0, 0.0)                   # drives off: out of CARLA at the next step
    for _ in range(10):
        s.tick()
    assert s.sumo2carla_ids == {} and "near" not in sumo.subs


def test_plain_tick_unchanged_without_the_switches(mods, monkeypatch):
    sumo = _StepSumo(mods, {})
    s, _ = _bridge(mods, monkeypatch, sumo, 1)
    assert s.pipelined is False
