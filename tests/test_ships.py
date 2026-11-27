"""AI ships in a trial (ships.py) against a fake CARLA world: the frames, the shapes look, routes and their ends,
the water level, the logs and the config checks. No CARLA, no network."""
import csv
import math
import sys
import types
from types import SimpleNamespace as NS

import pytest

from coupledcarla import shipmodel as sm

LAT0, LON0 = 51.98, 4.12           # metres per degree below: about Hoek van Holland
M_LAT, M_LON = 111_000.0, 68_700.0


def fake_carla():
    m = types.ModuleType("carla")

    class Vec:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = float(x), float(y), float(z)

    class Rotation:
        def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
            self.pitch, self.yaw, self.roll = pitch, yaw, roll

    class Transform:
        def __init__(self, location=None, rotation=None):
            self.location, self.rotation = location or Vec(), rotation or Rotation()

    m.Location = m.Vector3D = Vec
    m.Rotation, m.Transform = Rotation, Transform
    m.command = NS(ApplyTransform=lambda i, tf: ("tf", i, tf), DestroyActor=lambda i: ("destroy", i))
    return m


@pytest.fixture(scope="module")
def carla_mod():
    real = sys.modules.get("carla")
    try:
        import carla  # noqa: F401
        yield sys.modules["carla"]
        return
    except ImportError:
        pass
    sys.modules["carla"] = fake_carla()
    yield sys.modules["carla"]
    if real is None:
        sys.modules.pop("carla", None)


class Actor:
    def __init__(self, i, bp, tf):
        self.id, self.bp, self.tf = i, bp, tf
        self.physics = self.collisions = True
        has_mesh = bp.attrs.get("mesh_path", "").startswith("/Engine/")      # only basic shapes are cooked here
        self.bounding_box = NS(extent=NS(x=0.5 if has_mesh else 0.0, y=0.5 if has_mesh else 0.0, z=0.5))

    def set_simulate_physics(self, on):
        self.physics = on

    def set_collisions(self, on):
        self.collisions = on

    def destroy(self):
        pass


class Blueprint:
    def __init__(self):
        self.attrs = {}

    def set_attribute(self, k, v):
        self.attrs[k] = v

    def has_attribute(self, k):
        return k in ("role_name", "mesh_path", "scale", "mass")


class FakeWorld:
    """Water at z 1.5 (a Water hit) over ground at z -14; static.prop.mesh with basic shapes only."""

    def __init__(self, carla, water=True):
        self.c, self.water, self.spawned, self.next_id = carla, water, [], 100

    def get_blueprint_library(self):
        def find(name):
            if name != "static.prop.mesh":
                raise IndexError(name)
            return Blueprint()
        return NS(find=find, filter=lambda pat: [])

    def _spawn(self, bp, tf):
        self.next_id += 1
        a = Actor(self.next_id, NS(attrs=dict(bp.attrs)), tf)
        self.spawned.append(a)
        return a

    spawn_actor = try_spawn_actor = _spawn

    def cast_ray(self, a, b):
        hits = [NS(location=self.c.Location(a.x, a.y, -14.0), label="CityObjectLabel.Ground")]
        if self.water:
            hits.insert(0, NS(location=self.c.Location(a.x, a.y, 1.5), label="CityObjectLabel.Water"))
        return hits

    def get_weather(self):
        return NS(wind_intensity=0.0)


class FakeLogger:
    def __init__(self):
        self.events, self.rows = [], 0

    def event(self, t, frame, name, aid="", detail=""):
        self.events.append((t, name, detail))

    def drone_rows(self, t, frame, records):
        self.rows += len(records)


GEO = NS(to_carla=lambda lat, lon: ((lon - LON0) * M_LON, -(lat - LAT0) * M_LAT),
         to_latlon=lambda x, y: (LAT0 - y / M_LAT, LON0 + x / M_LON))


def north_east(n, e):
    """[lat, lon] of a point n m north and e m east of the origin."""
    return [LAT0 + n / M_LAT, LON0 + e / M_LON]


def make(carla_mod, tmp_path, fleet, visual="shapes", env=None, world=None, water_z="auto"):
    from coupledcarla import ships
    tc = {"ships": {"fleet": fleet, "visual": visual, "water_z": water_z,
                    "environment": env or {"wind_mps": 0.0}, "log_every_s": 0.5},
          "simulation": {"step": 0.1}}
    world = world or FakeWorld(carla_mod)
    sw = ships.ShipWorld(world, None, GEO, tc, tmp_path, tmp_path, log=lambda m: None)
    sent = []
    sw.queue = sent.append
    sw.start()
    return sw, world, sent


def run(sw, seconds, dt=0.1, lg=None):
    lg = lg or FakeLogger()
    for k in range(int(round(seconds / dt))):
        sw.step(k * dt, dt)
        sw.log_frame(lg, k * dt, k)
    return lg


def test_frames_round_trip():
    from coupledcarla import ships
    assert ships.earth_from_carla(10.0, -20.0) == (20.0, 10.0)        # CARLA y south: -20 is 20 m north
    pts = ships.route_points(GEO, [north_east(500.0, 0.0), north_east(0.0, 300.0)])
    assert pts[0] == pytest.approx((500.0, 0.0), abs=1e-6) and pts[1] == pytest.approx((0.0, 300.0), abs=1e-6)


def test_shape_parts_sit_on_the_waterline():
    from coupledcarla import ships
    h = sm.KVLCC2
    parts = ships.shape_parts(h)
    hull = [p for p in parts if p[1] == h.beam]
    assert len(hull) == math.ceil(h.lpp / h.beam)
    assert all(p[4] + h.beam / 2 == pytest.approx(h.freeboard) for p in hull)      # deck at the freeboard (9.2 m)
    ends = [p[2] for p in hull]
    assert min(ends) - h.beam / 2 == pytest.approx(-h.lpp / 2) and max(ends) + h.beam / 2 == pytest.approx(h.lpp / 2)
    house = parts[len(hull)]
    assert house[2] < -h.lpp / 4 and house[4] > h.freeboard                         # deckhouse aft, above the deck


def test_a_ship_sails_its_route_and_stops(carla_mod, tmp_path):
    route = [north_east(0.0, 0.0), north_east(2000.0, 0.0), north_east(2000.0, 2000.0)]
    sw, world, sent = make(carla_mod, tmp_path, [{"id": "tanker", "hull": "kvlcc2", "route": route, "speed_kn": 12}])
    lg = run(sw, 1200.0)                       # a VLCC swings about 1 km past a right-angle corner
    s = sw.ships[0]
    assert s.state == "stopping" and any(e[1] == "ship_arrived" for e in lg.events)
    assert s.sailed_m > 3000.0
    assert s.ship.n_cmd == 0.0 and s.ship.speed / sm.KNOT < 11.0      # engine stopped: she coasts and slows
    # the shapes look: every part moved each tick at the waterline found by the ray (1.5 m)
    assert s.visual.mode == "shapes" and len(s.visual.actors) == len(sw.ships[0].visual.parts)
    assert all(not a.physics and not a.collisions for a in s.visual.actors)
    assert s.z == pytest.approx(1.5)
    tfs = [c for c in sent if c[0] == "tf"]
    assert len(tfs) == 12000 * len(s.visual.actors)
    # ships.csv: every 0.5 s, the columns of LOG_COLUMNS
    from coupledcarla import ships
    rows = list(csv.reader(open(tmp_path / "ships.csv"), delimiter=";"))
    assert rows[0] == ships.LOG_COLUMNS and len(rows) - 1 == pytest.approx(1200 / 0.5, abs=2)
    first = dict(zip(rows[0], rows[1]))
    assert float(first["speed_kn"]) == pytest.approx(12.0, abs=0.1) and float(first["heading_deg"]) == pytest.approx(0, abs=0.5)
    assert lg.rows == 12000                      # a frames.csv row per tick


def test_start_s_waits_and_remove_destroys(carla_mod, tmp_path):
    route = [north_east(0.0, 0.0), north_east(800.0, 0.0)]
    sw, world, _ = make(carla_mod, tmp_path, [{"id": "late", "route": route, "start_s": 5.0, "at_end": "remove",
                                               "speed_kn": 14}])
    run(sw, 4.0)
    assert sw.ships[0].state == "waiting" and not world.spawned[1:]       # [0]: the look probe
    run(sw, 400.0)
    assert sw.ships[0].state == "removed" and sw.ships[0].visual.actors == []
    assert sw.summary()["arrived"] == ["late"]


def test_loop_goes_back_to_the_start(carla_mod, tmp_path):
    route = [north_east(0.0, 0.0), north_east(1000.0, 0.0)]
    sw, _, _ = make(carla_mod, tmp_path, [{"id": "ferry", "route": route, "at_end": "loop", "speed_kn": 15}])
    lg = run(sw, 300.0)
    arrivals = [e for e in lg.events if e[1] == "ship_arrived"]
    assert len(arrivals) >= 1 and sw.ships[0].state == "sailing"
    assert sw.ships[0].ship.x < 1000.0


def test_advance_s_starts_the_ship_under_way(carla_mod, tmp_path):
    route = [north_east(0.0, 0.0), north_east(5000.0, 0.0)]
    sw, _, _ = make(carla_mod, tmp_path, [{"id": "a", "route": route, "advance_s": 300, "speed_kn": 10}])
    assert sw.ships[0].ship.x == pytest.approx(300 * 10 * sm.KNOT, rel=0.02)
    assert sw.ships[0].ship.t == 0.0


def test_environment_from_the_config_and_the_current_moves_the_track(carla_mod, tmp_path):
    route = [north_east(0.0, 0.0), north_east(3000.0, 0.0)]
    env = {"wind_mps": 12.0, "wind_from_deg": 90, "current_mps": 1.0, "current_to_deg": 90, "wave_hs": 2.0,
           "wave_tp": 7.0}
    sw, _, _ = make(carla_mod, tmp_path, [{"id": "a", "route": route, "speed_kn": 10}], env=env)
    e = sw.env
    assert (e.wind_mps, e.wind_from_deg, e.current_mps, e.wave_hs, e.wave_from_deg) == (12.0, 90.0, 1.0, 2.0, 90.0)
    run(sw, 60.0)
    sh = sw.ships[0].ship
    assert sh.wave_force[1] < 0                          # waves from starboard push to port
    assert math.degrees(sh.course) > 0.5                 # the current sets her east of her heading


def test_water_level_fixed_one_per_map_or_from_the_ground(carla_mod, tmp_path):
    route = [north_east(0.0, 0.0), north_east(500.0, 0.0)]
    sw, _, _ = make(carla_mod, tmp_path, [{"route": route}], water_z=-0.4)
    assert sw.ships[0].z == -0.4
    sw, _, _ = make(carla_mod, tmp_path, [{"route": route}], world=FakeWorld(carla_mod, water=False))
    assert sw.ships[0].z == pytest.approx(-14.0)          # no Water label: the highest hit

    class Quay(FakeWorld):          # one route point lands on a quay 3 m up: the median keeps the water's 1.5 m
        def cast_ray(self, a, b):
            if a.x > 900:
                return [NS(location=self.c.Location(a.x, a.y, 3.0), label="CityObjectLabel.Water")]
            return super().cast_ray(a, b)
    r2 = [north_east(0.0, 0.0), north_east(0.0, 400.0), north_east(0.0, 1000.0)]
    sw, _, _ = make(carla_mod, tmp_path, [{"id": "a", "route": route}, {"id": "b", "route": r2}],
                    world=Quay(carla_mod))
    assert [s.z for s in sw.ships] == [1.5, 1.5]


def test_no_look_and_unknown_hull(carla_mod, tmp_path):
    route = [north_east(0.0, 0.0), north_east(500.0, 0.0)]
    sw, world, sent = make(carla_mod, tmp_path, [{"route": route, "hull": "titanic"}, {"id": "ok", "route": route}],
                           visual="none")
    assert [s.name for s in sw.ships] == ["ok"] and not world.spawned
    run(sw, 1.0)
    assert sent == [] and sw.ships[0].visual.mode == "none"


def test_check_config():
    from coupledcarla import ships
    errs, pts = [], []
    tc = {"ships": {"visual": "boats", "environment": {"wind": 3},
                    "fleet": [{"hull": "titanic", "route": [[52, 4]], "at_end": "sink", "speed": 3},
                              {"route": [[52.0, 4.1], [52.01, 4.1]]}]}}
    ships.check_config(tc, "trial", lambda w, m: errs.append((w, m)), lambda w, m: None,
                       lambda w, p, kind: pts.append((w, kind)))
    text = " | ".join(f"{w}: {m}" for w, m in errs)
    for bit in ("visual", "environment", "unknown key 'speed'", "titanic", "two [lat, lon]", "sink"):
        assert bit in text
    assert pts == [("trial ships fleet[1] route", "any")]
    errs.clear()
    ships.check_config({"ships": {}}, "trial", lambda w, m: errs.append(w), None, None)
    assert errs == []


# -- water bodies: the scenery's levels, the OpenDRIVE userData ---------------------------------------------------

SQUARE = [[-500.0, -500.0], [1500.0, -500.0], [1500.0, 1500.0], [-500.0, 1500.0], [-500.0, -500.0]]
ISLAND = [[100.0, -50.0], [300.0, -50.0], [300.0, 50.0], [100.0, 50.0], [100.0, -50.0]]


def test_point_in_ring_and_islands():
    from coupledcarla import ships
    assert ships.point_in_ring(0.0, 0.0, SQUARE) and not ships.point_in_ring(2000.0, 0.0, SQUARE)
    assert ships.point_in_ring(200.0, 0.0, ISLAND)


def test_ships_take_the_sceneries_water_level(carla_mod, tmp_path):
    import json
    (tmp_path / "scenery.json").write_text(json.dumps({"water": [{"level": 1.1, "rings": [SQUARE, ISLAND]},
                                                                 {"level": 4.6, "rings": [[[5000, 5000], [5100, 5000],
                                                                                           [5100, 5100], [5000, 5000]]]}]}))
    route = [north_east(0.0, 0.0), north_east(500.0, 0.0)]       # CARLA (0, 0) and (0, -500): in the square
    sw, _, _ = make(carla_mod, tmp_path, [{"route": route}])
    assert sw.ships[0].z == pytest.approx(1.1)                      # not the ray's 1.5
    sw2, _, _ = make(carla_mod, tmp_path, [{"route": route}], water_z=0.2)
    assert sw2.ships[0].z == 0.2                                    # set by hand wins
    assert sw.scenery_level([(200.0, 0.0)]) is None                 # on the island: no water there


def test_water_userdata_in_the_opendrive():
    from coupledcarla import placement
    water = [{"level": 1.1, "rings": [SQUARE, ISLAND]}]
    xodr = '<?xml version="1.0"?>\n<OpenDRIVE>\n<header/>\n</OpenDRIVE>\n'
    out, n = placement.carla_xodr(xodr, keep_lights=True, water=water)
    assert n == 0 and out.index('<userData code="coupledcarla.water">') < out.index("</OpenDRIVE>")
    lines = out.split('<userData code="coupledcarla.water">')[1].split("</userData>")[0].strip().splitlines()
    assert len(lines) == 2 and lines[0].startswith("0 1.100 -500.0,-500.0 1500.0,-500.0")
    assert placement.carla_xodr(xodr, keep_lights=True)[0] == xodr          # no water: unchanged
    assert placement.with_water(xodr, [{"level": 1.0, "rings": [[[0, 0], [1, 1]]]}]) == xodr    # too few points
