"""Drones beyond the flight model: pilot input mapping, the planner (obstacles, missions, separation), the frame
conversions, the noise model, the config checks, the frame log, and a DroneWorld flying against a fake CARLA world
(crash into a wall, geofence, a delivery over a building). No CARLA, no network."""
import importlib
import json
import math
import sys
import types
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from coupledcarla import droneinput as di
from coupledcarla import dronemodel as dm
from coupledcarla import droneplan as plan


# -- a stand-in for the carla module (CI has none) ----------------------------------------------------------------

def fake_carla():
    m = types.ModuleType("carla")

    class Vec:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = float(x), float(y), float(z)

        def distance(self, o):
            return math.dist((self.x, self.y, self.z), (o.x, o.y, o.z))

    class Rotation:
        def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
            self.pitch, self.yaw, self.roll = pitch, yaw, roll

    class Transform:
        def __init__(self, location=None, rotation=None):
            self.location, self.rotation = location or Vec(), rotation or Rotation()

    class Color:
        def __init__(self, r=0, g=0, b=0, a=255):
            self.r, self.g, self.b = r, g, b

    class LS(int):
        pass
    for k, v in dict(NONE=0, Position=1, LowBeam=2, HighBeam=4, Brake=8, RightBlinker=16, LeftBlinker=32).items():
        setattr(LS, k, v)
    m.Location = m.Vector3D = Vec
    m.Rotation, m.Transform, m.Color, m.VehicleLightState = Rotation, Transform, Color, LS
    m.LaneType = NS(Any=-1, Driving=1)
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


# -- frames, noise -----------------------------------------------------------------------------------------------

def test_rotation_to_carla(carla_mod):
    from coupledcarla import drones
    r = dm.q_matrix(dm.q_from_yaw(np.array([math.pi / 2])))[0]          # facing north
    pitch, yaw, roll = drones.rotation_deg(r)
    assert yaw == pytest.approx(-90.0) and abs(pitch) < 1e-9 and abs(roll) < 1e-9   # CARLA: north is -y
    # nose up 10 deg about the body y (left) axis: x forward tilts up
    a = math.radians(-10)
    ry = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    assert drones.rotation_deg(ry)[0] == pytest.approx(10.0)
    # right side down: rotation about body x by -10 deg (left side up)
    b = math.radians(10)
    rx = np.array([[1, 0, 0], [0, math.cos(b), -math.sin(b)], [0, math.sin(b), math.cos(b)]])
    assert drones.rotation_deg(rx)[2] == pytest.approx(10.0)       # UE: positive roll, right side down
    assert drones.to_carla((1, 2, 3)) == (1.0, -2.0, 3.0)
    assert drones.yaw_from_heading(90) == pytest.approx(0.0)       # east


def test_noise_levels_and_wind_profile(carla_mod):
    from coupledcarla import drones
    # consumer drone (LWA 81 dB) hovering: 59 dB(A) at 5 m, 6 dB less per doubling of distance
    assert drones.sound_level(81.0, 1.0, 5.0) == pytest.approx(81 - 20 * math.log10(5) - 8)
    assert drones.sound_level(81.0, 1.0, 10.0) == pytest.approx(drones.sound_level(81.0, 1.0, 5.0) - 6.02, abs=0.01)
    assert drones.sound_level(81.0, 2.0, 5.0) > drones.sound_level(81.0, 1.0, 5.0)   # more thrust, louder
    assert 60 <= drones.sound_level(81.0, 1.0, 3.0) <= 80     # published small quadrotors: 60-80 dB(A) at a few m
    assert drones.wind_at(5.0, 10.0) == pytest.approx(5.0)
    assert drones.wind_at(5.0, 60.0) > 5.0 > drones.wind_at(5.0, 3.0)


# -- pilot input ---------------------------------------------------------------------------------------------------

def test_stick_shaping():
    assert di.shape(0.03) == 0.0
    assert di.shape(1.0) == pytest.approx(1.0) and di.shape(-1.0) == pytest.approx(-1.0)
    assert 0 < di.shape(0.5) < 0.5, "expo: softer near the centre"


def test_keyboard_and_joystick_sticks():
    k = NS(**{f"K_{n}": i for i, n in enumerate("UP SPACE DOWN LSHIFT RIGHT e LEFT q w s d a i k c m h ESCAPE".split())})
    held = {k.K_w, k.K_UP, k.K_LEFT}
    s = di.keyboard_sticks(lambda key: key in held, [k.K_c, k.K_h], k)
    assert (s.climb, s.forward, s.yaw, s.lateral) == (1.0, 1.0, -1.0, 0.0)
    assert s.events == ["view", "home"]
    # an Xbox pad: left stick pushed up is axis 1 = -1 (climb), right stick right is axis 2 = +1 (lateral)
    axes = {0: 0.0, 1: -1.0, 2: 1.0, 3: 0.0}
    j = di.joystick_sticks(lambda i: axes[i], 6, di.GAMEPAD_AXES)
    assert j.climb == pytest.approx(1.0) and j.lateral == pytest.approx(1.0) and j.forward == 0.0
    rc = di.joystick_sticks(lambda i: [0.0, 0.5, 0.0, -1.0][i], 4, di.RC_AXES, expo=0.0)   # AETR
    assert rc.forward > 0 and rc.yaw == pytest.approx(-1.0) and rc.climb == 0.0


def test_pilot_command_modes():
    af = dm.CONSUMER
    hold = di.Hold()
    assert di.pilot_command(di.Sticks(), (0, 0, 0), (0, 0, 0), 0.0, True, af, hold=hold)["mode"] == "off"
    c = di.pilot_command(di.Sticks(climb=1.0), (0, 0, 0), (0, 0, 0), 0.0, True, af, hold=hold)
    assert c["mode"] == "velocity" and c["vel"][2] == pytest.approx(af.max_climb)
    # flying north (yaw 90 deg), forward full: velocity north at max speed; right turn: negative yaw rate
    c = di.pilot_command(di.Sticks(forward=1.0, yaw=0.5), (0, 0, 10), (0, 0, 0), math.pi / 2, False, af, hold=hold)
    assert c["vel"][1] == pytest.approx(af.max_speed) and abs(c["vel"][0]) < 1e-9 and c["yaw_rate"] < 0
    # sticks released while moving east at 10 m/s: hold 6 m ahead (braking), and keep that point
    c = di.pilot_command(di.Sticks(), (0, 0, 10), (10, 0, 0), 0.0, False, af, hold=hold)
    assert c["mode"] == "position" and c["pos"][0] == pytest.approx(6.0)
    c2 = di.pilot_command(di.Sticks(), (3, 0, 10), (5, 0, 0), 0.0, False, af, hold=hold)
    assert c2["pos"] == c["pos"]
    att = di.pilot_command(di.Sticks(forward=1.0, lateral=-0.5), (0, 0, 10), (0, 0, 0), 0.0, False, af, "attitude")
    assert att["mode"] == "attitude" and att["pitch"] == pytest.approx(af.max_tilt) and att["roll"] < 0


def test_landing_protection_slows_the_descent_near_the_ground():
    af = dm.CONSUMER
    st = di.Sticks(climb=-1.0)
    high = di.pilot_command(st, (0, 0, 30), (0, 0, 0), 0.0, False, af, agl=30.0)
    low = di.pilot_command(st, (0, 0, 2), (0, 0, 0), 0.0, False, af, agl=2.0)
    att = di.pilot_command(st, (0, 0, 2), (0, 0, 0), 0.0, False, af, "attitude", agl=2.0)
    assert high["vel"][2] == pytest.approx(-af.max_descent)
    assert low["vel"][2] == pytest.approx(-1.6)
    assert att["climb"] == pytest.approx(-af.max_descent), "no protection in attitude mode"


def test_bot_stops_after_a_crash():
    bot = di.BotPilot([(0, 0, 10), (100, 0, 10)], speed=5.0)
    assert bot.sticks(1.0, (0, 0, 10), (0, 0, 0), 0.0).forward > 0
    bot.crashed = True
    assert bot.sticks(2.0, (0, 0, 0), (0, 0, 0), 0.0).as_list() == [0, 0, 0, 0]
    bot.after_crash = "continue"
    assert bot.sticks(3.0, (0, 0, 10), (0, 0, 0), 0.0).forward > 0


def test_bot_pilot_flies_its_path():
    """The bot's sticks through pilot_command fly the flight model to its waypoints."""
    q = dm.Quadrotor(dm.CONSUMER, (0, 0, 0), 0.0)
    bot = di.BotPilot([(0, 0, 15), (60, 40, 20)], speed=8.0, start_s=0.5)
    hold = di.Hold()
    for k in range(1500):
        t = 0.02 * k
        st = bot.sticks(t, q.pos, q.vel, q.yaw)
        q.command(**di.pilot_command(st, q.pos, q.vel, q.yaw, q.on_ground, dm.CONSUMER, hold=hold))
        q.step(0.02)
        if bot.done:
            break
    assert bot.done, q.pos
    assert np.linalg.norm(q.pos - (60, 40, 20)) < 3.0


# -- planner -------------------------------------------------------------------------------------------------------

def square(cx, cy, half, top, k="house"):
    """A building record (CARLA frame: y south) centred on model (cx, cy)."""
    y = -cy
    return {"p": [[cx - half, y - half], [cx + half, y - half], [cx + half, y + half], [cx - half, y + half],
                  [cx - half, y - half]], "top": top, "k": k}


def test_obstacles_heightfield_and_walls():
    obs = plan.Obstacles([square(50, 0, 10, 25.0), square(-40, 30, 5, 9.0)], trees=[(0.0, -60.0, 14.0, 3.0)])
    assert obs.top_at(50, 0) == pytest.approx(25.0)
    assert obs.top_at(50, 30) == -np.inf
    assert obs.top_at(0, 60) == pytest.approx(14.0), "a tree (CARLA y -60 is north 60)"
    assert obs.top_at(63, 0, radius=4.0) == pytest.approx(25.0), "within 4 m of the wall"
    assert obs.corridor_top((0, 0), (100, 0), 5.0) == pytest.approx(25.0)
    assert obs.corridor_top((0, -50), (100, -50), 5.0) == -np.inf
    assert not obs.is_open(50, 0) and obs.is_open(0, 0)
    w = obs.walls(np.array([70.0, 0.0, 20.0]), 30.0, above_z=15.0)
    assert len(w) == 1
    (sp, sv) = w[0]
    assert sp[0] == pytest.approx(60.0) and sv[0] == pytest.approx(1.0), "shill on the east wall, pointing out"
    assert obs.walls(np.array([70.0, 0.0, 30.0]), 30.0, above_z=28.0) == []


def test_addresses_have_open_drop_points():
    rng = plan.make_rng(3)
    obs = plan.Obstacles([square(x, y, 6, 8.0) for x in (0, 40, 80) for y in (0, 40)] +
                         [square(200, 200, 20, 30.0, "office")])
    picks = plan.pick_addresses(obs, 4, rng, (40, 20), 100)
    assert len(picks) == 4
    for bi, (x, y) in picks:
        assert obs.polys[bi][2] == "house"
        assert obs.is_open(x, y, 3.0)
        c = obs.polys[bi][0].mean(axis=0)
        assert math.hypot(x - c[0], y - c[1]) < 25


def fly(fleet, missions, seconds, dt=0.02, every=None):
    for k in range(int(seconds / dt)):
        for ms in missions:
            ms.step(k * dt, dt)
        fleet.step(dt)
        if every:
            every(k * dt)


def test_delivery_cruises_over_the_roofs_and_comes_back():
    obs = plan.Obstacles([square(60, 0, 12, 25.0)])
    f = dm.Fleet()
    i = f.add(dm.INDUSTRIAL, (0, 0, 0), 0.0)
    drop = (120.0, 0.0, 0.0)
    cruise = obs.corridor_top((0, 0), drop, 10.0) + 20.0
    assert cruise == pytest.approx(45.0)
    ms = plan.Delivery("d1", f, i, (0.0, 0.0, 0.0), [drop], [cruise], None, start_s=1.0, dwell_s=3.0, cruise=12.0,
                       loop=False)
    lowest = []

    def watch(t):
        p = f.pos[i]
        if abs(p[0] - 60) < 14 and abs(p[1]) < 14:
            lowest.append(p[2])
    fly(f, [ms], 75.0, every=watch)
    assert ms.delivered == 1
    assert min(lowest) > 25.0 + 15.0, "over the building at cruise height"
    names = [e[1] for e in ms.events]
    assert any("delivering" in n for n in names) and any("returning" in n for n in names)
    assert np.linalg.norm(f.pos[i][:2]) < 2.0 and f.on_ground[i], "back on its pad"


def test_winch_delivery_hovers_at_the_drop():
    f = dm.Fleet()
    i = f.add(dm.INDUSTRIAL, (0, 0, 0), 0.0)
    ms = plan.Delivery("w", f, i, (0.0, 0.0, 0.0), [(40.0, 0.0, 0.0)], [25.0], None, dwell_s=4.0, drop="winch",
                       winch_m=7.0, loop=False)
    heights = []
    fly(f, [ms], 30.0, every=lambda t: heights.append((ms.phase, f.pos[i][2])))
    assert ms.delivered == 1
    drop_h = [z for ph, z in heights if ph == "dropping"]
    assert drop_h and abs(np.median(drop_h) - 7.0) < 1.0 and not f.on_ground[i]


def test_patrol_loops_forever():
    f = dm.Fleet()
    i = f.add(dm.CONSUMER, (0, 0, 0), 0.0)
    loop = [(30 * math.cos(a), 30 * math.sin(a), 40.0) for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
    ms = plan.Patrol("p", f, i, loop, speed=8.0)
    fly(f, [ms], 80.0)
    assert ms.route.index >= 1 and not ms.route.done
    assert abs(f.pos[i][2] - 40.0) < 2.0 and 20 < np.hypot(*f.pos[i][:2]) < 40


def test_swarm_flocks_without_collisions():
    f = dm.Fleet()
    ids = [f.add(dm.CONSUMER, (3.0 * (k % 3), 3.0 * (k // 3), 0.0), 0.0) for k in range(6)]
    route = [(150, 0), (150, 150), (0, 150), (0, 0)]
    ms = plan.Swarm("s", f, ids, route, 30.0, "coupled_sim", None, stagger_s=0.3)
    closest = []

    def watch(t):
        if t > 15:
            p = f.pos[ids]
            d = np.linalg.norm(p[:, None] - p[None], axis=2) + np.eye(len(ids)) * 1e9
            closest.append(d.min())
    fly(f, [ms], 60.0, every=watch)
    assert all(ph == "flock" for ph in ms.member_phase)
    assert min(closest) > 3.0, "nobody collides"
    assert np.all(np.abs(f.pos[ids][:, 2] - 30.0) < 3.0)
    c = f.pos[ids].mean(axis=0)
    assert c[0] > 60, "migrating towards its first point"


def test_separation_and_obstacle_floor():
    f = dm.Fleet()
    a = f.add(dm.CONSUMER, (0, 0, 20), 0.0, on_ground=False)
    b = f.add(dm.CONSUMER, (4, 0, 20), 0.0, on_ground=False)
    n = plan.separation(f, [a, b], radius=10.0)
    assert n[0][0] < 0 < n[1][0]
    obs = plan.Obstacles([square(10, 0, 5, 30.0)])
    f.vel[a] = (5, 0, 0)
    floor = plan.avoid_obstacles(f, [a], obs)
    assert floor[a] == pytest.approx(34.0)


# -- config, validator, logs -------------------------------------------------------------------------------------

EXP = {
    "name": "dronecheck",
    "map": {"bbox": [51.43, 5.46, 51.45, 5.49]},
    "participants": [{"role": "pilot", "kind": "drone", "input": "gamepad", "view": "fpv", "airframe": "consumer",
                      "spawn": {"lat": 51.44, "lon": 5.47, "heading": 90}, "geofence": {"max_altitude_m": 100},
                      "bot": {"path": [[51.441, 5.471, 30]], "speed": 6}}],
    "drones": {"missions": [{"kind": "delivery", "id": "parcels", "count": 2, "drop": "winch"},
                            {"kind": "swarm", "id": "flock", "count": 8, "flocking": "vasarhelyi2018"},
                            {"kind": "patrol", "route": [[51.44, 5.47], [51.445, 5.475]]}],
               "cameras": [{"name": "street", "lat": 51.44, "lon": 5.47, "look": "flock_0"}]},
    "trials": [{"duration_s": 60}, {"duration_s": 60, "drones": {"missions": []}}],
}


def _check(raw, tmp_path):
    from coupledcarla import config, validate
    p = tmp_path / "e.yaml"
    import yaml
    p.write_text(yaml.safe_dump(raw))
    cfg = config.load(p)
    rep, _ = validate.check(raw, cfg, roads=False, participants=[])
    return rep


def test_validator_accepts_drones(tmp_path, carla_mod):
    rep = _check(EXP, tmp_path)
    assert rep.errors == [], rep.items


def test_validator_flags_drone_mistakes(tmp_path, carla_mod):
    import copy
    bad = copy.deepcopy(EXP)
    bad["participants"][0].update(view="cockpit", input="joystick", airframe="jumbo", geofence={"max_altitude_m": 300})
    bad["drones"]["missions"].append({"kind": "zeppelin", "colour": "red"})
    bad["drones"]["visual"] = "fancy"
    msgs = " | ".join(m for _, _, m in _check(bad, tmp_path).items)
    for frag in ("view 'cockpit'", "input 'joystick'", "airframe 'jumbo'", "kind 'zeppelin'", "unknown key 'colour'",
                 "drones.visual" if False else "'fancy'", "above the EU open category"):
        assert frag in msgs, (frag, msgs)


def test_trial_override_switches_drones_off(tmp_path):
    from coupledcarla import config
    import yaml
    p = tmp_path / "e.yaml"
    p.write_text(yaml.safe_dump(EXP))
    cfg = config.load(p)
    assert len(config.trial_config(cfg, cfg["trials"][0])["drones"]["missions"]) == 3
    t1 = config.trial_config(cfg, cfg["trials"][1])["drones"]
    assert t1["missions"] == [] and t1["visual"] == "auto"


def test_frames_rows_for_drones(tmp_path, carla_mod):
    logger_mod = importlib.import_module("coupledcarla.logger")
    geo = NS(to_latlon=lambda x, y: (51.44 - y / 111000.0, 5.47 + x / 70000.0))
    lg = logger_mod.TrialLogger(tmp_path, geo, {"radius_m": 100.0, "udp": None}, 0.02)
    c = carla_mod
    tf = c.Transform(c.Location(1.0, -2.0, 30.0), c.Rotation(pitch=5.0, yaw=-90.0, roll=1.0))
    lg.drone_rows(1.5, 75, [("pilot", "drone", "drone.consumer", 7, tf, c.Vector3D(3.0, 0.0, 1.0),
                             c.Vector3D(0.0, 0.0, 0.0))])
    lg.close()
    rows = (tmp_path / "frames.csv").read_text().splitlines()
    head, row = rows[0].split(";"), rows[1].split(";")
    r = dict(zip(head, row))
    assert r["kind"] == "drone" and r["role"] == "pilot" and r["pos_z"] == "30.000" and r["rot_yaw"] == "-90.00"
    assert float(r["speed_kmh"]) == pytest.approx(3.6 * math.sqrt(10), abs=0.01) and r["throttle"] == ""


# -- sound -----------------------------------------------------------------------------------------------------------

def test_soundscape_hears_a_drone(carla_mod):
    from coupledcarla import audio
    from coupledcarla.soundscape import Soundscape, render_offline
    sc = Soundscape({"enabled": True, "reference_db": 94.0}, "pedestrian", log=lambda m: None)
    hover_bpf = 2 * dm.CONSUMER.hover_w / (2 * math.pi)
    d = [[11, 5.0, 0.0, 1.7, 0.0, 0.0, 0.0, hover_bpf, 1.0, 81.0, 2],
         [12, 900.0, 0.0, 30.0, 0.0, 0.0, 0.0, 110.0, 1.0, 96.6, 2]]        # too far: not heard
    sc.update(0.0, (0.0, 0.0, 1.7, 0.0), [], drones=d)
    assert ("drone", 11) in sc.info and ("drone", 12) not in sc.info
    # Lw - 20 lg r - 11 + 3 (spherical spreading, the ground's reflection, ISO 9613-2), less a little air absorption
    assert sc.info[("drone", 11)]["db"] == pytest.approx(81 - 20 * math.log10(5) - 8, abs=0.2)
    v = sc.mixer.get(("drone", 11))
    assert v.base == pytest.approx(hover_bpf) and abs(v.paths[0].az) < 0.1, "straight ahead of a listener facing east"
    from coupledcarla.soundrec import level_dba
    x = render_offline(sc, 1.0)
    assert level_dba(x) == pytest.approx(sc.info[("drone", 11)]["db"], abs=1.5)   # the level at the ears
    sc.update(0.1, (0.0, 0.0, 1.7, 0.0), [], drones=[])
    assert sc.mixer.get(("drone", 11)) is None


def test_rotor_voice_has_its_blade_passing_tone():
    from coupledcarla import audio
    v = audio.Rotor(freq=200.0, blades=2, broadband=0.0)
    x = v.render(audio.RATE)
    spec = np.abs(np.fft.rfft(x))
    f = np.fft.rfftfreq(len(x), 1.0 / audio.RATE)
    assert abs(f[np.argmax(spec)] - 200.0) < 4.0
    assert audio.rms(x) == pytest.approx(audio.FS_RMS, rel=0.15)


# -- a DroneWorld against a fake CARLA world ----------------------------------------------------------------------

class FakeWorld:
    """Flat ground at z 0, a wall x in [30, 31] (CARLA), up to 40 m, for cast_ray; no blueprints (visual none)."""

    def __init__(self, carla):
        self.c = carla
        self.debug = None

    def get_map(self):
        c = self.c
        return NS(get_waypoint=lambda loc, **k: NS(transform=c.Transform(c.Location(loc.x, loc.y, 0.0))))

    def get_blueprint_library(self):
        def find(name):
            raise IndexError(name)
        return NS(find=find, filter=lambda pat: [])

    def ground_projection(self, loc, dist):
        return NS(location=self.c.Location(loc.x, loc.y, 0.0), label="Ground")

    def cast_ray(self, a, b):
        if (a.x - 30.0) * (b.x - 30.0) <= 0 and a.x != b.x and min(a.z, b.z) < 40:
            t = (30.0 - a.x) / (b.x - a.x)
            return [NS(location=self.c.Location(30.0, a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)),
                       label="CityObjectLabel.Buildings")]
        return []

    def get_weather(self):
        return NS(wind_intensity=0.0)


class FakeLogger:
    def __init__(self):
        self.events, self.rows = [], 0

    def event(self, t, frame, name, aid="", detail=""):
        self.events.append((t, name, detail))

    def drone_rows(self, t, frame, records):
        self.rows += len(records)


def make_world(carla_mod, tmp_path, participants, missions=(), buildings=()):
    from coupledcarla import drones
    (tmp_path / "buildings.json").write_text(json.dumps(buildings))
    geo = NS(to_carla=lambda lat, lon: ((lon - 5.47) * 70000.0, -(lat - 51.44) * 111000.0),
             to_latlon=lambda x, y: (51.44 - y / 111000.0, 5.47 + x / 70000.0))
    tc = {"drones": {"missions": list(missions), "visual": "none", "wind_mps": 0.0}, "map": {"bbox": None},
          "simulation": {"step": 0.02}, "logging": {}}
    dw = drones.DroneWorld(FakeWorld(carla_mod), None, geo, tc, tmp_path, tmp_path, log=lambda m: None)
    parts = []
    for p in participants:
        part = NS(cfg=p, role=p["role"], station=False, kind="drone")
        dw.add_pilot(part)
        parts.append(part)
    dw.queue = lambda cmd: None
    dw.start()
    return dw, parts


def run_world(dw, seconds, lg):
    for k in range(int(seconds / 0.02)):
        t = k * 0.02
        dw.step(t, 0.02, (), None)
        dw.log_frame(lg, t, k)


def test_pilot_crashes_into_a_wall_and_falls(carla_mod, tmp_path):
    p = {"role": "pilot", "input": "bot", "spawn": {"lat": 51.44, "lon": 5.47, "heading": 90},
         "bot": {"path": [[51.44, 5.47, 15], [51.44, 5.47 + 60 / 70000.0, 15]], "speed": 8, "start_s": 0.5}}
    dw, (part,) = make_world(carla_mod, tmp_path, [p])
    lg = FakeLogger()
    xs = []
    for k in range(int(25.0 / 0.02)):
        dw.step(k * 0.02, 0.02, (), None)
        dw.log_frame(lg, k * 0.02, k)
        xs.append(dw.fleet.pos[part.drone.idx][0])
    names = [e[1] for e in lg.events]
    crash = [e for e in lg.events if e[1] == "drone_crash"]
    assert crash and "Buildings" in crash[0][2], lg.events
    assert "drone_crash_down" in names, "it fell and is ready again"
    assert max(xs) < 30.0, "never through the wall"
    assert part.drone.crashes >= 1
    assert lg.rows == int(25.0 / 0.02)
    dw.close()
    rows = (tmp_path / "drones.csv").read_text().splitlines()
    assert rows[0].startswith("timestamp;frame;drone") and len(rows) == 1 + int(25.0 / 0.02)


def test_geofence_caps_the_height(carla_mod, tmp_path):
    p = {"role": "pilot", "input": "bot", "spawn": {"lat": 51.44, "lon": 5.47}, "geofence": {"max_altitude_m": 30},
         "bot": {"sticks": [[0.0, 1.0, 0.0, 0.0, 0.0]]}}      # full climb, for ever
    dw, (part,) = make_world(carla_mod, tmp_path, [p])
    lg = FakeLogger()
    run_world(dw, 20.0, lg)
    z = dw.fleet.pos[part.drone.idx][2]
    assert 27.0 < z < 31.5, z
    assert any(e[1] == "drone_geofence" for e in lg.events)
    dw.close()


def test_station_messages_and_broadcast(carla_mod, tmp_path):
    """A station's sticks and camera reach its pilot; the HUD and the drones' sound sources go out every 5th tick."""
    p = {"role": "pilot", "spawn": {"lat": 51.44, "lon": 5.47}}
    dw, (part,) = make_world(carla_mod, tmp_path, [p])
    part.station = True
    pil = part.drone
    assert dw.on_message({"type": "drone_camera", "role": "pilot", "camera_id": 42}) and pil.camera_id == 42
    assert dw.on_message({"type": "drone_input", "role": "pilot", "s": [1, 0, 0, 0], "g": 0.5, "e": ["view"]})
    assert pil.sticks.climb == 1.0 and pil.view == "los"
    assert not dw.on_message({"type": "gaze", "role": "pilot"})
    for k in range(100):
        dw.step(k * 0.02, 0.02, (), None)
    assert dw.fleet.pos[pil.idx][2] > 3.0, "the station's climb stick lifted it"
    sent = []
    ctl = NS(send=lambda role, msg: sent.append((role, msg)), broadcast=lambda msg: sent.append(("*", msg)))
    dw.broadcast(ctl, 5)
    hud = [m for r, m in sent if m["type"] == "drone_hud"][0]
    assert hud["lines"][0].startswith("ALT") and len(hud["ears"]) == 3
    d = [m for r, m in sent if m["type"] == "drones"][0]["d"]
    assert len(d) == 1 and d[0][7] > 100 and d[0][9] == dm.CONSUMER.sound_power_dba
    sent.clear()
    dw.broadcast(ctl, 6)
    assert sent == []
    dw.close()


def test_world_flies_a_delivery_over_a_building(carla_mod, tmp_path):
    bld = [{"h": 22.0, "z": 0.0, "k": "office", "p": [[40, -10], [60, -10], [60, 10], [40, 10], [40, -10]]}]
    m = {"kind": "delivery", "id": "parcel", "depot": {"lat": 51.44, "lon": 5.47},
         "addresses": [[51.44, 5.47 + 100 / 70000.0]], "loop": False, "dwell_s": 2.0}
    dw, _ = make_world(carla_mod, tmp_path, [], [m], bld)
    ms = dw.missions[0]
    assert ms.cruise_z[0] >= 22.0 + 20.0
    lg = FakeLogger()
    run_world(dw, 60.0, lg)
    assert ms.delivered == 1, lg.events
    assert any(e[1] == "drone_delivery" for e in lg.events)
    s = dw.summary()
    assert s["deliveries"] == 1 and s["ai"] == 1 and s["ms_per_tick"] > 0
    dw.close()
