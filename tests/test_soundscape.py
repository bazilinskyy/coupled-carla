"""coupledcarla.audio and coupledcarla.soundscape: synthesis, calibrated levels, the mixer, the vehicle sound model,
AVAS, auditory eHMI cues, the own car, calibration, and recording without an audio device. No pygame, no CARLA."""
import math
import time
import types
import wave

import numpy as np
import pytest

from coupledcarla import audio
from coupledcarla import soundscape as ss


def level(x):
    """dB of a mono or stereo-channel signal against the full-scale sine, at the default reference 94 dB."""
    return audio.gain_db(audio.rms(x) / audio.FS_RMS)


def test_gain_and_buffers_are_calibrated():
    assert audio.db_gain(94) == pytest.approx(1.0)
    assert audio.db_gain(74) == pytest.approx(0.1)
    assert audio.db_gain(200) == 1.0                       # clamped
    assert audio.db_gain(None) == 0.0
    assert audio.gain_db(0.1) == pytest.approx(74.0)
    assert audio.rms(audio.tone(1000, 1.0)) == pytest.approx(audio.FS_RMS, rel=1e-3)
    for buf in (audio.pink_noise(2.0), audio.brown_noise(2.0), audio.band_noise(80, 3000, 2.0),
                audio.city_ambience(4.0), audio.rural_ambience(4.0)):
        assert audio.rms(buf) == pytest.approx(audio.FS_RMS, rel=1e-3)
        assert abs(buf[0] - buf[-1]) < 0.5                  # loops without a big jump
    for name in ("chime", "beeps", "ding", "tick", "takeover"):
        assert len(audio.cue_sound(name)) > 100
    with pytest.raises(ValueError):
        audio.cue_sound("trumpet")


def test_mixer_pans_with_constant_power_and_filters():
    m = audio.Mixer()
    m.add("a", audio.Loop(audio.tone(1000, 1.0), gain=0.1, pan=0.0))
    m.render(2048)                                        # the first block sets the ramps
    x = m.render(8192)
    assert level(x[:, 0]) == pytest.approx(74.0 - 3.01, abs=0.1)     # centre: -3 dB in each ear
    m.get("a").pan = 1.0
    m.render(2048)
    x = m.render(8192)
    assert audio.rms(x[:, 0]) < 1e-4 and level(x[:, 1]) == pytest.approx(74.0, abs=0.1)
    # a low-pass at 500 Hz takes most of white noise's power away
    rng = np.random.default_rng(1)
    white = rng.uniform(-1, 1, 44100).astype(np.float32)
    m2 = audio.Mixer()
    m2.add("w", audio.Loop(white, gain=0.5, cutoff=500.0))
    m2.render(2048)
    assert audio.rms(m2.render(8192)[:, 0]) < 0.3 * audio.rms(white) * 0.5
    # a one-shot is removed at its end
    m3 = audio.Mixer()
    m3.add("c", audio.OneShot(audio.cue_sound("tick"), gain=0.5))
    m3.render(4096)
    assert m3.keys() == []


def test_engine_and_avas_voices_are_continuous_and_calibrated():
    e = audio.Engine(freq=40.0)
    x = np.concatenate([e.render(2048) for _ in range(20)])
    assert audio.rms(x) == pytest.approx(audio.FS_RMS, rel=0.05)
    e.freq = 80.0                                        # a frequency change keeps the phase: no jump
    y = e.render(2048)
    assert abs(float(y[0]) - float(x[-1])) < 0.1
    a = audio.Avas()
    z = np.concatenate([a.render(2048) for _ in range(20)])
    assert audio.rms(z) == pytest.approx(audio.FS_RMS, rel=0.05)


def test_wav_round_trip_and_speech_fallback(tmp_path, monkeypatch):
    st = np.stack([audio.tone(440, 0.5) * 0.5] * 2, axis=1)
    audio.write_wav(tmp_path / "t.wav", st)
    back = audio.load_sound(tmp_path / "t.wav")
    assert len(back) == pytest.approx(len(st), abs=2)
    assert audio.rms(back) == pytest.approx(audio.FS_RMS, rel=0.05)      # scaled like a cue
    monkeypatch.setattr(audio, "speech_engines", lambda: [])
    assert audio.speak_to_wav("hello", cache=tmp_path) is None
    logs = []
    sc = ss.Soundscape({"auditory_ehmi": {"enabled": True, "cues": [{"ehmi": "go", "say": "go ahead",
                                                                      "sound": "ding"}]}}, log=logs.append)
    assert any("no speech synthesiser" in m for m in logs)
    assert sc.cues[0]["_buf"] is sc.sounds["ding"]


def test_output_records_without_a_device(tmp_path):
    m = audio.Mixer()
    m.add("t", audio.Loop(audio.tone(1000, 1.0), gain=0.5))
    out = audio.Output(m, device=False, record=tmp_path / "rec.wav", log=lambda s: None)
    out.start()
    time.sleep(0.4)
    out.stop()
    with wave.open(str(tmp_path / "rec.wav")) as w:
        assert w.getnchannels() == 2 and w.getnframes() >= audio.BLOCK * 3


def _car(i=1, x=7.5, y=0.0, kmh=50.0, yaw=90.0, tid="vehicle.audi.a2", electric=False, automated=False, ehmi=None,
         kind="car", lights=0):
    v = kmh / 3.6
    vx, vy = v * math.cos(math.radians(yaw)), v * math.sin(math.radians(yaw))
    return ss.Body(i, tid, kind, x, y, 0.0, vx, vy, 0.0, yaw, electric, automated, lights, ehmi)


def test_level_model():
    assert ss.tyre_db_at_7_5(50) == 68.0
    assert ss.tyre_db_at_7_5(100) == pytest.approx(68 + 30 * math.log10(2))
    assert ss.tyre_db_at_7_5(2) is None
    assert ss.at_distance(68.0, 15.0) == pytest.approx(68 - 6.02, abs=0.01)
    assert ss.engine_state(1.0)[0] == 0
    assert ss.engine_state(30.0)[0] == 2
    assert 1100 <= ss.engine_state(30.0)[1] <= 3000
    assert ss.classify("vehicle.gazelle.omafiets", 2) == "bike"
    assert ss.classify("vehicle.yamaha.yzf", 2) == "motorbike"
    assert ss.classify("vehicle.mitsubishi.fusorosa") == "bus"
    assert ss.classify("vehicle.mercedes.sprinter") == "van"
    assert ss.classify("walker.pedestrian.0001") == "walker"
    assert ss.is_electric("auto", "vehicle.tesla.model3", 1)
    assert not ss.is_electric("auto", "vehicle.audi.a2", 1)
    assert ss.is_electric("auto", "vehicle.audi.a2", 1, automated=True)
    assert ss.is_electric(True, "vehicle.audi.a2", 1) and not ss.is_electric(False, "vehicle.tesla.model3", 1)
    share = sum(ss.is_electric(0.3, "vehicle.audi.a2", i) for i in range(2000)) / 2000
    assert 0.25 < share < 0.35
    assert ss.is_electric(["vehicle.audi.*"], "vehicle.audi.a2", 1)


def test_vehicle_levels_at_the_listener_match_the_model():
    from coupledcarla import acoustics as ac
    cfg = {"enabled": True, "environment": False, "reverb": False}
    sc = ss.Soundscape(cfg, log=lambda s: None)
    car = _car(electric=True)                               # tyres only
    sc.update(0.0, (0, 0, 1.2, 0), [car], listener_h=1.2)
    sc.update(0.05, (0, 0, 1.2, 0), [car], listener_h=1.2)
    info = sc.voices_info()[1]
    roll, _ = ac.vehicle_power("1", 50.0, temp_c=15.0)      # the default air temperature: +0.4 dB
    d = math.dist((7.5, 0, 0.3), (0, 0, 1.2))
    want = ac.a_weighted(roll - 20 * math.log10(d) - 11 - ac.air_absorption(15, 70) * d
                         - ac.ground_attenuation(0.05, 1.2, 7.5))
    assert info["tyre_db"] == pytest.approx(want, abs=0.01) and info["engine_db"] is None
    assert 70.0 < info["tyre_db"] < 74.0                    # CNOSSOS-EU: a car at 50 km/h, 7.5 m: ~72-73 dB(A)
    x = ss.render_offline(sc, 2.0)
    both = audio.gain_db(math.sqrt(audio.rms(x[:, 0]) ** 2 + audio.rms(x[:, 1]) ** 2) / audio.FS_RMS)
    from coupledcarla.soundrec import level_dba
    assert level_dba(x) == pytest.approx(info["tyre_db"], abs=0.7)     # the rendered sound has the model's level
    assert both > level_dba(x)                               # unweighted: more (the rolling noise's low end)
    # the combustion car adds its engine
    sc2 = ss.Soundscape(cfg, log=lambda s: None)
    for k in range(20):
        sc2.update(0.05 * k, (0, 0, 1.2, 0), [_car()], listener_h=1.2)
    i2 = sc2.voices_info()[1]
    assert i2["engine_db"] is not None and 55.0 < i2["engine_db"] < i2["tyre_db"]
    assert 1200 < i2["rpm"] < 2600


def test_panning_doppler_and_nearest_vehicles():
    sc = ss.Soundscape({"enabled": True, "max_vehicles": 2}, log=lambda s: None)
    right = _car(1, x=0.0, y=10.0, yaw=0.0)                 # CARLA: +y is to the right of a listener facing +x
    sc.update(0.0, (0, 0, 0, 0), [right])
    assert sc.voices_info()[1]["pan"] == pytest.approx(1.0)
    v = sc.mixer.get(("veh", 1, "tyre"))
    approaching = _car(2, x=40.0, y=0.0, yaw=180.0, kmh=72)
    leaving = _car(3, x=40.0, y=0.5, yaw=0.0, kmh=72)
    far = _car(4, x=100.0, y=0.0)
    sc.update(0.1, (0, 0, 0, 0), [approaching, leaving, far, right])
    assert set(sc.voices_info()) == {1, 2}                  # the nearest two
    assert sc.mixer.get(("veh", 3, "tyre")) is None
    sc3 = ss.Soundscape({"enabled": True}, log=lambda s: None)
    sc3.update(0.0, (0, 0, 0, 0), [approaching, leaving])
    assert sc3._doppler(approaching, 40, 0, 40, 0, 0) > 1.0 > sc3._doppler(leaving, 40, 0.5, 40, 0, 0)
    assert v is not None


def test_avas_up_to_20_kmh():
    cfg = {"enabled": True, "avas": {"enabled": True, "db": 56.0}, "environment": False}
    at75 = 56.0 - 20 * math.log10(7.5 / 2) + 0.2           # the ground's +3 dB near both, a little air absorption
    for kmh, want in ((10, at75), (17.5, None), (25, "none"), (0.0, "none")):
        sc = ss.Soundscape(cfg, log=lambda s: None)
        car = _car(kmh=kmh, tid="vehicle.tesla.model3", electric=True)
        sc.update(0.0, (0, 0, 0, 0), [car], listener_h=0.3)
        a = sc.voices_info()[1]["avas_db"]
        if want == "none":
            assert a is None
        elif want is None:
            assert a < at75 - 5                              # fading out
        else:
            assert a == pytest.approx(want, abs=0.6)
    sc = ss.Soundscape(cfg, log=lambda s: None)
    sc.update(0.0, (0, 0, 0, 0), [_car(kmh=10)])             # a combustion car has none
    assert sc.voices_info()[1]["avas_db"] is None


def test_auditory_ehmi_cues_on_state_changes_and_motion():
    cues = []
    cfg = {"enabled": True, "auditory_ehmi": {"enabled": True, "cues": [
        {"ehmi": "yielding", "sound": "chime"}, {"ehmi": "go", "sound": "ding", "repeat_s": 1.0},
        {"on": "stopped", "sound": "beeps"}, {"on": "driving_off", "sound": "beeps"},
        {"on": "pedestrian_ahead", "value": 12, "sound": "tick"}]}}
    sc = ss.Soundscape(cfg, log=lambda s: None, on_cue=lambda *a: cues.append(a))
    t = 0.0
    av = lambda kmh, ehmi, x=10.0: _car(5, x=x, kmh=kmh, automated=True, ehmi=ehmi, yaw=180.0)
    sc.update(t, (0, 0, 0, 0), [av(30, "driving")])          # baseline: no cue
    assert cues == []
    t += 0.05
    sc.update(t, (0, 0, 0, 0), [av(30, "yielding")])
    assert [c[1] for c in cues] == ["chime"]
    for _ in range(40):                                      # comes to a stop
        t += 0.05
        sc.update(t, (0, 0, 0, 0), [av(0, "yielding")])
    assert "beeps" in [c[1] for c in cues]
    n = len(cues)
    for _ in range(50):                                      # "go" with a repeat every second
        t += 0.05
        sc.update(t, (0, 0, 0, 0), [av(0, "go")])
    assert sum(c[1] == "ding" for c in cues[n:]) == 3       # at once, after 1 s and 2 s
    for _ in range(30):
        t += 0.05
        sc.update(t, (0, 0, 0, 0), [av(25, "driving")])
    assert sum("driving_off" in c[2] for c in cues) == 1
    walker = ss.Body(9, "walker.pedestrian.0001", "walker", 2.0, 0.3, 0.0, 0, 0, 0, 0)
    t += 0.05
    sc.update(t, (0, 0, 0, 0), [av(25, "driving"), walker])  # the walker is 8 m ahead of the AV
    assert cues[-1][1] == "tick"
    # a plain (not automated) car gets no motion cues by default
    sc2 = ss.Soundscape(cfg, log=lambda s: None, on_cue=lambda *a: cues.append(a))
    n = len(cues)
    for k in range(40):
        sc2.update(k * 0.05, (0, 0, 0, 0), [_car(6, kmh=30 if k < 2 else 0)])
    assert len(cues) == n


def test_own_car_inside_and_blinker_ticks():
    cues = []
    sc = ss.Soundscape({"enabled": True, "environment": False}, kind="driver", log=lambda s: None,
                       on_cue=lambda *a: cues.append(a))
    own = _car(1, x=0, kmh=100.0, yaw=0.0, lights=ss.LEFT_BLINKER)
    for k in range(31):
        sc.update(k * 0.05, (0, 0, 1.2, 0), [own], own=own)
    info = sc.voices_info()["own"]
    assert info["wind_db"] == pytest.approx(60.0)
    assert 64.0 < info["total_dba"] < 72.0                    # inside a mid-size car at 100 km/h
    assert 1 not in sc.voices_info()                          # not heard as an outside car
    ticks = [c for c in cues if c[1] == "blinker_tick"]
    assert len(ticks) == 5                                    # 1.5 s at three clicks a second (0, 1/3, ...)


def test_calibration_tone_mutes_the_rest(monkeypatch):
    sc = ss.Soundscape({"enabled": True, "ambience": {"db": 60}}, log=lambda s: None)
    sc.update(0.0, (0, 0, 0, 0), [_car()])
    sc.toggle_calibration()
    sc.update(0.05, (0, 0, 0, 0), [_car()])
    ss.render_offline(sc, 0.1)
    x = ss.render_offline(sc, 1.0)
    # the full-scale tone, panned to the centre: each ear at -3 dB of full scale
    assert level(x[:, 0]) == pytest.approx(94.0 - 3.01, abs=0.2)
    sc.toggle_calibration()
    assert sc.mixer.get("calibration") is None and sc.mixer.get("ambience").gain > 0


class _Loc:
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Snap:
    """A world snapshot: iterable ActorSnapshots, find(id)."""

    def __init__(self, poses):
        self.items = {i: types.SimpleNamespace(
            id=i, get_transform=lambda p=p: types.SimpleNamespace(location=_Loc(p[0], p[1]),
                                                                    rotation=types.SimpleNamespace(yaw=p[2])),
            get_velocity=lambda p=p: _Loc(p[3], 0.0)) for i, p in poses.items()}

    def __iter__(self):
        return iter(self.items.values())

    def find(self, i):
        return self.items.get(i)


def test_world_bodies_fetches_types_once():
    calls = []

    class World:
        def get_actors(self, ids):
            calls.append(list(ids))
            tids = {1: "vehicle.tesla.model3", 2: "walker.pedestrian.0003", 3: "static.prop.mesh"}
            return [types.SimpleNamespace(id=i, type_id=tids[i], attributes={"number_of_wheels": "4"}) for i in ids]

    wb = ss.WorldBodies(World(), electric="auto", radius=100.0, every=10)
    snap = _Snap({1: (10, 0, 0, 5.0), 2: (5, 5, 0, 0.0), 3: (20, 0, 0, 0.0), 4: (900, 0, 0, 0.0)})
    bodies = wb(snap, (0, 0))
    assert sorted(b.id for b in bodies) == [1, 2]
    assert calls == [[1, 2, 3]]                              # the far actor is not looked up
    car = [b for b in bodies if b.id == 1][0]
    assert car.electric and car.kind == "car" and car.vx == 5.0
    for _ in range(12):
        wb(snap, (0, 0))
    assert len(calls) == 1                                   # all known now
    del snap.items[1]
    assert [b.id for b in wb(snap, (0, 0))] == [2]           # gone from the world


def test_speed_from_positions_when_carla_reports_none():
    """SUMO's vehicles are moved by transform: CARLA reports no velocity; the soundscape takes it from positions."""
    sc = ss.Soundscape({"enabled": True, "environment": False}, log=lambda s: None)
    for k in range(40):
        t = 0.05 * k
        car = ss.Body(7, "vehicle.audi.a2", "car", -20.0 + 13.9 * t, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, True, False)
        sc.update(t, (0, 0, 1.6, 0), [car])
    assert sc.voices_info()[7]["kmh"] == pytest.approx(50.0, abs=2.0)
    assert sc.voices_info()[7]["tyre_db"] is not None
