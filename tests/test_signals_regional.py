"""signals.py regional heads (CARLA 1.0 street content): mesh choice, lens positions, fallback (no SUMO, no server)."""
import math
import types

import pytest

signals = pytest.importorskip("coupledcarla.signals")


def test_regional_style_from_street_props():
    nl_car = signals.regional_style("NL", "car")
    assert nl_car["pole"].endswith("SM_CC_NL_SignalPole") and nl_car["head"].endswith("SM_CC_NL_SignalCar200")
    assert len(nl_car["lenses"]) == 3 and nl_car["lenses"][0][2] > nl_car["lenses"][2][2]      # red on top
    assert nl_car["lens_d"] == pytest.approx(0.20) and nl_car["faces"] == [None, None, None]
    assert nl_car["alt"]["head"].endswith("SM_CC_NL_SignalCar") and nl_car["alt"]["lens_d"] == pytest.approx(0.30)
    assert nl_car["alt"]["lenses"][0][2] == pytest.approx(0.36)          # the older head's own lens positions
    nl_bike = signals.regional_style("NL", "bike")
    assert nl_bike["head"].endswith("SM_CC_NL_SignalBike") and nl_bike["mode"] == "bike"
    assert nl_bike["faces"] == ["bike"] * 3 and nl_bike["alt"] is None
    nl_ped = signals.regional_style("NL", "ped")
    assert nl_ped["faces"] == ["ped_stand", "ped_walk"] and len(nl_ped["lenses"]) == 2
    uk = signals.regional_style("UK", "bike")          # no UK cyclist head: the UK car head, on the UK pole
    assert uk["head"].endswith("SM_CC_UK_SignalCar200") and uk["mode"] == "car" and uk["lens_d"] == 0.2
    assert uk["pole"].endswith("SM_CC_UK_SignalPole")  # CARLA 1.0 has no stand-alone signal pole of its own
    jp = signals.regional_style("JP", "car")
    assert jp["pole"].endswith("SM_CC_JP_SignalPole") and jp["lens_d"] == pytest.approx(0.30)
    assert abs(jp["lenses"][0][0]) > 0.3 and jp["lenses"][0][2] == 0.0                  # horizontal
    assert signals.regional_style(None, "car") is None and signals.regional_style("US", "car") is None


def test_regional_points_face_the_traffic():
    st = signals.regional_style("NL", "car")
    p = signals.regional_points(10.0, 20.0, 1.0, 0.0, st)         # traffic travels along +x
    assert p["yaw"] == pytest.approx(90.0)
    assert all(x < 10.0 for x, _, _ in p["lamps"])                 # lenses towards the oncoming cars (-x)
    assert all(y == pytest.approx(20.0) for _, y, _ in p["lamps"])
    zs = [z for _, _, z in p["lamps"]]
    assert zs[0] > zs[1] > zs[2] and zs[1] == pytest.approx(1.0 + signals.REGION_HEAD_Z["car"])
    assert p["pole"] == (10.0, 20.0, 1.0) and signals.regional_points(0, 0, 1.0, 0.0, st, ground=0.8)["pole"][2] == 0.8
    lens_y = st["lenses"][0][1] + signals.LENS_FRONT
    assert 10.0 - p["lamps"][0][0] == pytest.approx(lens_y)
    q = signals.regional_points(0.0, 0.0, 0.0, 90.0, st)           # along +y: towards -y
    assert all(y < 0 for _, y, _ in q["lamps"]) and all(abs(x) < 1e-9 for x, _, _ in q["lamps"])
    jp = signals.regional_points(0.0, 0.0, 0.0, 90.0, signals.regional_style("JP", "car"))
    assert len({round(x, 3) for x, _, _ in jp["lamps"]}) == 3                       # side by side
    assert math.hypot(*jp["lamps"][1][:2]) == pytest.approx(signals.regional_style("JP", "car")["lenses"][1][1]
                                                            + signals.LENS_FRONT)


def test_lens_of():
    assert [signals.lens_of(r, 3) for r in (0, 1, 2, None)] == [0, 1, 2, None]
    assert [signals.lens_of(r, 2) for r in (0, 1, 2)] == [0, None, 1]          # pedestrian head: no amber


def test_lane_mode():
    lane = lambda allowed: types.SimpleNamespace(allows=lambda c: c in allowed)    # noqa: E731
    assert signals.lane_mode(lane({"bicycle"})) == "bike"
    assert signals.lane_mode(lane({"bicycle", "passenger"})) == "car"
    assert signals.lane_mode(lane({"bus", "bicycle"})) == "car"
    assert signals.lane_mode(object()) == "car"


class _BP:
    def __init__(self):
        self.attrs = {}

    def set_attribute(self, k, v):
        self.attrs[k] = v


class _World:
    """try_spawn_actor gives an actor for the meshes it has, None for the others."""

    def __init__(self, has):
        self.has, self.spawned = has, []

    def get_blueprint_library(self):
        bp = _BP()
        self.bp = bp
        return types.SimpleNamespace(find=lambda name: bp)

    def try_spawn_actor(self, bp, tf):
        mesh = bp.attrs["mesh_path"]
        if not any(mesh.endswith(h) for h in self.has):
            return None
        a = types.SimpleNamespace(mesh=mesh, tf=tf, destroy=lambda: None)
        self.spawned.append(a)
        return a


def _sig(region, modes):
    s = signals.Signals.__new__(signals.Signals)
    loc = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    s.heads = [("tl", [0], loc, 0.0) for _ in modes]
    s.head_links = [[("tl", [0])] for _ in modes]
    s.head_modes = list(modes)
    s.region, s.actors, s.surface = region, [], None
    s._on_road_any = lambda loc: True
    return s


@pytest.fixture
def carla_stub(monkeypatch):
    class Loc:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z
    monkeypatch.setattr(signals.carla, "Location", Loc, raising=False)
    monkeypatch.setattr(signals.carla, "Rotation", lambda yaw=0.0: types.SimpleNamespace(yaw=yaw), raising=False)
    monkeypatch.setattr(signals.carla, "Transform", lambda loc, rot: (loc, rot), raising=False)


def test_spawn_regional_heads_on_a_server_with_the_content(carla_stub):
    w = _World({"SM_CC_NL_SignalPole", "SM_CC_NL_SignalCar200", "SM_CC_NL_SignalBike"})
    s = _sig("NL", ["car", "bike"])
    assert s._spawn_heads(w, log=lambda m: None)
    meshes = [a.mesh.rsplit(".", 1)[-1] for a in w.spawned]
    assert meshes == ["SM_CC_NL_SignalPole", "SM_CC_NL_SignalCar200", "SM_CC_NL_SignalPole", "SM_CC_NL_SignalBike"]
    assert [st["mode"] for st in s.styles] == ["car", "bike"]


def test_older_content_spawns_the_head_it_replaces(carla_stub):
    # a package built before the 200 mm car heads: the 300 mm SignalCar with its own lens positions, for every car
    # head (the new mesh is tried once), also when a cyclist head (same name in both) spawned first
    w = _World({"SM_CC_NL_SignalPole", "SM_CC_NL_SignalCar", "SM_CC_NL_SignalBike"})
    s = _sig("NL", ["bike", "car", "car"])
    msgs = []
    assert s._spawn_heads(w, log=msgs.append)
    heads = [a.mesh.rsplit(".", 1)[-1] for a in w.spawned if "Pole" not in a.mesh]
    assert heads == ["SM_CC_NL_SignalBike", "SM_CC_NL_SignalCar", "SM_CC_NL_SignalCar"]
    assert [st["head"].rsplit(".", 1)[-1] for st in s.styles] == ["SM_CC_NL_SignalBike", "SM_CC_NL_SignalCar",
                                                                   "SM_CC_NL_SignalCar"]
    assert s.styles[1]["lens_d"] == pytest.approx(0.30) and s.styles[1]["lenses"][0][2] == pytest.approx(0.36)
    assert sum("older" in m for m in msgs) == 1


def test_no_content_keeps_carlas_heads(carla_stub):
    w = _World({"SM_TrafficLight__VPole_Main", "SM_TrafficLight_holster"})      # CARLA 0.9.16 or 1.0 without it
    s = _sig("NL", ["car", "bike"])
    msgs = []
    assert s._spawn_heads(w, log=msgs.append)
    assert s.styles == [None, None] and len(w.spawned) == 4
    assert any("no NL signal meshes" in m for m in msgs)
    s = _sig(None, ["car"])
    assert s._spawn_heads(w, log=msgs.append) and s.styles == [None]
