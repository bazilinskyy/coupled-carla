"""coupledcarla.soundrec: the host's audio recording in simulation time (alignment with frames, file format,
audio.json, summary), and realistic total levels (kerb of a busy street, inside a car). Fake CARLA objects only."""
import json
import math
import types
import wave

import numpy as np
import pytest

from coupledcarla import audio
from coupledcarla import soundscape as ss
from coupledcarla import soundrec


class Loc:
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z


class Actor:
    def __init__(self, i, tid, x, y, yaw, speed, wheels=4):
        self.id, self.type_id = i, tid
        self.attributes = {"number_of_wheels": str(wheels), "role_name": ""}
        self.x, self.y, self.yaw, self.speed = x, y, yaw, speed

    def step(self, dt):
        self.x += self.speed * math.cos(math.radians(self.yaw)) * dt
        self.y += self.speed * math.sin(math.radians(self.yaw)) * dt

    def get_transform(self):
        return types.SimpleNamespace(location=Loc(self.x, self.y, 0.0), rotation=types.SimpleNamespace(yaw=self.yaw))

    def get_velocity(self):
        return Loc(self.speed * math.cos(math.radians(self.yaw)), self.speed * math.sin(math.radians(self.yaw)))


class Snap:
    def __init__(self, actors, frame, elapsed):
        self.actors = {a.id: a for a in actors}
        self.frame = frame
        self.timestamp = types.SimpleNamespace(elapsed_seconds=elapsed)

    def __iter__(self):
        return iter(self.actors.values())

    def find(self, i):
        return self.actors.get(i)


def _world(actors):
    return types.SimpleNamespace(get_actors=lambda ids: [a for a in actors if a.id in ids])


def test_recording_is_in_simulation_time_and_aligned(tmp_path):
    own = Actor(10, "vehicle.lincoln.mkz_2020", 0.0, 0.0, 0.0, 0.0)          # waiting, engine idling
    other = Actor(11, "vehicle.audi.a2", 120.0, 3.5, 180.0, 12.0)
    actors = [own, other]
    (tmp_path / "map").mkdir()
    (tmp_path / "map" / "meta.json").write_text(json.dumps({"bbox": [51.43, 5.47, 51.45, 5.49],
                                                            "speed_defaults": {"country": "NL"}}))
    (tmp_path / "map" / "buildings.json").write_text(json.dumps(
        [{"p": [[-50, 10], [50, 10], [50, 30], [-50, 30]], "h": 12, "z": 0}]))
    tc = {"sound": {"recordings": False, "environment": False}, "_root": str(tmp_path)}
    part = types.SimpleNamespace(cfg={}, role="driver", kind="driver", actor=own, ctl=None)
    wx = {"local_time": "2026-11-13 08:00", "sun_altitude": 5.0, "night": False,
          "open_meteo": {"temperature_2m": 8.0, "relative_humidity_2m": 90, "rain": 0.0, "wind_speed_10m": 18.0}}
    step = 0.05
    ha = soundrec.HostAudio(tc, tmp_path, part, tmp_path / "map", wx, step, _world(actors), log=lambda s: None)
    assert ha.env.temp == 8.0 and ha.env.country == "NL" and ha.scape.buildings.n == 1
    frame0, elapsed0 = 1234, 56.7
    import time
    for k in range(60):                       # 3 s of simulation; the "wall" time per tick varies (slow ticks)
        t = k * step
        if k == 20:
            ha.scape.play("beeps", 85.0, name="marker")      # an in-car cue at t = 1.0 s
        if k % 7 == 0:
            time.sleep(0.01)
        for a in actors:
            a.step(step)
        ha.tick(t, Snap(actors, frame0 + k, elapsed0 + t), lights={10: 0, 11: 0})
    summary = {}
    rec = ha.close(summary)
    assert (tmp_path / "audio.wav").exists()
    with wave.open(str(tmp_path / "audio.wav")) as w:
        assert (w.getnchannels(), w.getsampwidth(), w.getframerate()) == (2, 2, 44100)
        n = w.getnframes()
        x = np.frombuffer(w.readframes(n), "<i2").reshape(-1, 2).astype(float) / 32768
    assert n == round((59 * step + step) * 44100) == rec["samples"]       # exactly [0, t_last + step)
    meta = json.loads((tmp_path / "audio.json").read_text())
    assert meta["start_frame"] == frame0 and meta["start_sim_s"] == 0.0 and meta["step_s"] == step
    assert meta["start_elapsed_s"] == pytest.approx(elapsed0)
    assert summary["audio_start_sim_s"] == 0.0 and summary["audio"]["start_frame"] == frame0
    # the cue started at t = 1.0 s, frame frame0 + 20: at sample 44100 by the frame rule
    f = frame0 + 20
    t_f = (f - meta["start_frame"]) * meta["step_s"]
    s = int(round(t_f * 44100))
    before = np.abs(x[s - 2000:s - 100]).max()
    after = np.abs(x[s:s + 2000]).max()
    onset = s + int(np.argmax(np.abs(x[s - 200:s + 2000, 0]) > 2.5 * before)) - 200
    assert after > 2.5 * before and abs(onset - s) < 120
    assert meta["ms_per_tick"] > 0 and meta["peak"] <= 1.0
    assert not (tmp_path / "audio_credits.txt").exists()      # nothing recorded was used (recordings off)


def _car(i, x, y, kmh=50.0, yaw=0.0, kind="car", electric=False):
    v = kmh / 3.6
    return ss.Body(i, "vehicle.audi.a2", kind, x, y, 0.0, v * math.cos(math.radians(yaw)),
                   v * math.sin(math.radians(yaw)), 0.0, yaw, electric, False)


def test_busy_street_at_the_kerb_and_inside_a_car():
    # a busy two-lane street (about 1,300 vehicles an hour at 50 km/h, one bus), the listener on the kerb 2 m from
    # the near lane: 65-75 dB(A) (L_Aeq), as measured beside busy urban streets
    rng = np.random.default_rng(3)
    cars = [_car(20 + i, rng.uniform(-150, 150), 2.0 if i % 2 == 0 else 5.5, 50.0, 0.0 if i % 2 == 0 else 180.0,
                 kind="bus" if i == 3 else "car") for i in range(8)]
    sc = ss.Soundscape({"enabled": True, "recordings": False}, kind="pedestrian", log=lambda s: None,
                       env={"local_time": "2026-05-12 17:30"})
    blocks = []
    for k in range(240):
        t = 0.05 * k
        moved = [c._replace(x=((c.x + c.vx * t + 150.0) % 300.0) - 150.0) for c in cars]
        sc.update(t, (0.0, 0.0, 1.6, 0.0), moved, listener_h=1.6)
        blocks.append(sc.mixer.render(2205))
    x = np.concatenate(blocks)
    leq = soundrec.level_dba(x)
    assert 65.0 <= leq <= 77.0          # the bus 2 m away dominates (recorded engines keep its CNOSSOS level)
    assert sc.mixer.clipped == 0
    # inside a car at 50 km/h: about 60 dB(A)
    own = _car(1, 0.0, 0.0, 50.0)
    sc2 = ss.Soundscape({"enabled": True, "environment": False}, kind="driver", log=lambda s: None)
    for k in range(60):
        sc2.update(0.05 * k, (0.0, 0.0, 1.2, 0.0), [own], own=own)
    inside = soundrec.level_dba(ss.render_offline(sc2, 2.0))
    assert 56.0 <= inside <= 65.0
    assert sc2.voices_info()["own"]["total_dba"] == pytest.approx(inside, abs=1.0)


def test_level_helpers():
    tone = np.stack([audio.tone(1000, 1.0) * math.sqrt(0.5)] * 2, axis=1)    # the calibration tone, centre
    assert soundrec.level_dba(tone, 94.0) == pytest.approx(94.0, abs=0.1)
    t, lv = soundrec.dba_timeline(tone, 94.0)
    assert len(t) == 8 and np.allclose(lv, 94.0, atol=0.2)
    assert soundrec.a_weighting_db([1000.0])[0] == pytest.approx(0.0, abs=0.05)
    assert soundrec.a_weighting_db([100.0])[0] == pytest.approx(-19.1, abs=0.1)


def test_pedestrian_signals_walk_with_the_crossing_traffic(monkeypatch):
    import sys
    states = {"a": "r", "b": "G", "c": "r"}
    fake = types.SimpleNamespace(trafficlight=types.SimpleNamespace(
        getControlledLinks=lambda tls: [[("lane_" + tls, "out", "via")]],
        getRedYellowGreenState=lambda tls: states[tls]))
    monkeypatch.setitem(sys.modules, "traci", fake)
    shapes = {"lane_a": [(0.0, -20.0), (0.0, 0.0)], "lane_b": [(-20.0, 5.0), (0.0, 5.0)],      # a north, b east,
              "lane_c": [(-6.0, 20.0), (-6.0, 8.0)]}                                         # c south (opposite a)
    net = types.SimpleNamespace(getLane=lambda lid: types.SimpleNamespace(getShape=lambda: shapes[lid]))
    signals = types.SimpleNamespace(heads=[("a", [0], Loc(3.0, 0.0)), ("b", [0], Loc(0.0, -8.0)),
                                           ("c", [0], Loc(-9.0, -10.0))])
    ps = soundrec.PedSignals(signals, net)
    near = ps.near(5.0, 0.0, 0.0)
    assert near[0][:2] == (3.0, 0.0) and near[0][3] is True      # lane a and the opposite lane c red: walk
    states.update(a="G", b="r")
    assert ps.near(5.0, 0.0, 0.1)[0][3] is True                  # cached for 0.25 s
    assert ps.near(5.0, 0.0, 1.0)[0][3] is False                 # lane a green: wait
    states.update(a="r", c="G")
    assert ps.near(5.0, 0.0, 2.0)[0][3] is False                 # the opposite direction still has green: wait


def test_replay_from_frames_csv(tmp_path, monkeypatch):
    """A trial's soundscape rendered again from its logs, in the same alignment."""
    from coupledcarla import config
    trial = tmp_path / "trial"
    trial.mkdir()
    (trial / "config.yaml").write_text("name: x\nmap: {name: nomap}\nsimulation: {step: 0.05}\n"
                                       "sound: {recordings: false, environment: false}\n"
                                       "participants: [{role: walker1, kind: pedestrian}]\n")
    head = ("timestamp;frame;actor_id;kind;role;type_id;pos_x;pos_y;pos_z;rot_pitch;rot_yaw;rot_roll;lat;lon;vel_x;"
            "vel_y;vel_z;speed_kmh;acc_x;acc_y;acc_z;blinkers;front_lights;stop_lights;throttle;steer;brake;reverse;"
            "hand_brake;next_tl;next_tl_dist;next_tl_state;ehmi")
    rows = [head]
    for k in range(40):
        t = 0.05 * k
        rows.append(f"{t:.3f};{100 + k};5;participant;walker1;walker.pedestrian.0001;0;0;1;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;;;;;;;;;")
        rows.append(f"{t:.3f};{100 + k};9;car;;vehicle.audi.a2;{-20 + 13.9 * t:.2f};6;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;;;;;;;;;")
    (trial / "frames.csv").write_text("\n".join(rows) + "\n")
    seen = []
    out = soundrec.replay(trial, log=lambda s: None, probe=lambda t, sc: seen.append(sc.info.get(9, {}).get("kmh")))
    x, rate = soundrec.read_wav(out)
    assert len(x) == round((39 * 0.05 + 0.05) * 44100)
    assert seen[-1] == pytest.approx(50.0, abs=3.0)                 # speed taken from the positions
    assert soundrec.level_dba(x[-20000:], 100.0) > 55.0
    assert config.DEFAULTS["sound"]["record_reference_db"] == 100.0


def test_sound_env_follows_the_picture_weather():
    """A preset weather with simulation.time keeps Open-Meteo's record in weather.json (for the sun): the sound's rain,
    wind and wet road follow the preset the participant sees, temperature and humidity stay Open-Meteo's."""
    from types import SimpleNamespace
    from coupledcarla.soundenv import Env
    om = {"temperature_2m": 6.2, "relative_humidity_2m": 95, "rain": 1.4, "precipitation": 1.4,
          "wind_speed_10m": 13.0, "precipitation_3h": 8.5}
    dry = SimpleNamespace(precipitation=0.0, wind_intensity=10.0, wetness=0.0, precipitation_deposits=0.0,
                          sun_altitude_angle=16.4)
    preset = Env.from_weather({"weather": "CloudyNoon", "open_meteo": om, "sun_altitude": 16.4, "night": False},
                              {}, dry)
    assert preset.rain == 0.0 and preset.wet == 0.0 and abs(preset.wind - 10 / 2 / 3.6) < 1e-9
    assert preset.temp == 6.2 and preset.rh == 95.0
    real = Env.from_weather({"weather": "real", "open_meteo": om, "sun_altitude": 16.4, "night": False}, {}, dry)
    assert real.rain == 1.4 and real.wet == 1.0 and abs(real.wind - 13 / 3.6) < 1e-9
