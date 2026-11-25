"""coupledcarla.soundsamples: the engine from recorded loops (pair and cross-fade, pitch, level calibration against
the synthesised voice, fallback), recorded footsteps, an electric own car's sound and cluster. Loops are written as
WAV in a temporary folder (no decoder needed); the shipped OGG files are checked only where ffmpeg or pygame can
decode them."""
import importlib.util
import json
import math
import shutil
import wave
from types import SimpleNamespace

import numpy as np
import pytest

from coupledcarla import acoustics as ac
from coupledcarla import audio, soundsamples, soundscape, synth
from coupledcarla.soundrec import level_dba

REF = 94.0
RATE = audio.RATE


def band_levels(x, ref=REF):
    spec = np.abs(np.fft.rfft(x)) ** 2 * 2.0 / len(x) ** 2
    f = np.fft.rfftfreq(len(x), 1.0 / RATE)
    return np.array([ref + 10 * math.log10(spec[(f >= ac.EDGES[b]) & (f < ac.EDGES[b + 1])].sum() / audio.FS_RMS ** 2
                                           + 1e-30) for b in range(ac.NB)])


def render(v, seconds=2.0, warm=2):
    m = audio.Mixer()
    m.limiter = None
    m.reverb = None
    m.add("v", v)
    for _ in range(warm):
        m.render(2048)
    return m.render(int(seconds * RATE))


def write_wav(path, x):
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes((np.clip(x, -1, 1) * 32767).astype("<i2").tobytes())


def engine_loop(f0, seconds=2.0, seed=0):
    """A fake engine recording: a whole number of cycles of harmonics of f0 (falling with order) and a little noise,
    so it loops seamlessly."""
    n = int(round(round(seconds * f0) / f0 * RATE))
    t = np.arange(n) / RATE
    rng = np.random.default_rng(seed)
    x = sum(np.sin(2 * np.pi * k * f0 * t + rng.uniform(0, 6.28)) / k for k in range(1, 40) if k * f0 < 15000)
    x = x + 0.05 * rng.standard_normal(n)
    return 0.25 * x / np.max(np.abs(x))


@pytest.fixture
def fake_sounds(tmp_path):
    """A sounds.json with a petrol set of three WAV loops (30, 60, 100 Hz) and two footsteps."""
    (tmp_path / "engines").mkdir()
    (tmp_path / "footsteps").mkdir()
    loops = []
    for k, f0 in enumerate((30.0, 60.0, 100.0)):
        fn = f"engines/petrol_{k}.wav"
        write_wav(tmp_path / fn, engine_loop(f0, seed=k))
        loops.append({"file": fn, "firing_hz": f0})
    slot = int(0.5 * RATE)
    steps = np.zeros(2 * slot)
    rng = np.random.default_rng(3)
    for k in range(2):
        n = int(0.15 * RATE)
        steps[k * slot:k * slot + n] = rng.standard_normal(n) * np.exp(-np.arange(n) / RATE / 0.02) * 0.3
    write_wav(tmp_path / "footsteps" / "shoes.wav", steps)
    man = {"engines": {"petrol": {"source_engine": "test", "loops": loops}},
           "footsteps": {"shoes": {"file": "footsteps/shoes.wav", "slot_s": 0.5, "steps": 2}}}
    p = tmp_path / "sounds.json"
    p.write_text(json.dumps(man))
    soundsamples._SETS.clear()
    soundsamples._STEPS.clear()
    yield p
    soundsamples._SETS.clear()
    soundsamples._STEPS.clear()


def test_pair_and_equal_power_crossfade(fake_sounds):
    es = soundsamples.load_set("petrol", path=fake_sounds)
    assert list(es.firing) == [30.0, 60.0, 100.0]
    assert es.pair(20.0) == (0, 0, 0.0) and es.pair(30.0) == (0, 0, 0.0)
    assert es.pair(150.0) == (2, 2, 0.0)
    lo, hi, a = es.pair(math.sqrt(30.0 * 60.0))          # half way in log frequency
    assert (lo, hi) == (0, 1) and a == pytest.approx(0.5)
    v = soundsamples.SampleEngineVoice(es, 4, kinds=("direct",))
    v.set_rpm(math.sqrt(30.0 * 60.0) * 30.0, 0.3)        # 4 cylinders: firing = rpm / 30
    assert v.base == pytest.approx(math.sqrt(1800.0))
    assert v.gains[0] ** 2 + v.gains[1] ** 2 == pytest.approx(1.0)       # equal power
    assert v.gains[0] == pytest.approx(v.gains[1])
    # a loop keeps its slot (index parity): moving up to the next pair swaps only the silent loop
    v.set_rpm(60.0 * 30.0, 0.3)
    assert v.slots[1] == 1 and v.gains[1] == pytest.approx(1.0) and v.gains[0] == pytest.approx(0.0)
    v.set_rpm(80.0 * 30.0, 0.3)
    assert v.slots[1] == 1 and v.slots[0] == 2


def test_pitch_follows_the_rpm(fake_sounds):
    es = soundsamples.load_set("petrol", path=fake_sounds)
    for f in (36.0, 75.0, 120.0):                          # between loops, and above the last one
        v = soundsamples.SampleEngineVoice(es, 4, kinds=("direct",))
        v.set_rpm(f * 30.0, 0.3)
        v.paths[0].w = v.weights(np.full(ac.NB, 70.0), REF)
        x = render(v, 2.0)[:, 0]
        spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
        fq = np.fft.rfftfreq(len(x), 1.0 / RATE)
        m = (fq > 0.7 * f) & (fq < 1.4 * f)
        assert fq[m][np.argmax(spec[m])] == pytest.approx(f, rel=0.01)


def test_doppler_scales_the_pitch(fake_sounds):
    es = soundsamples.load_set("petrol", path=fake_sounds)
    v = soundsamples.SampleEngineVoice(es, 4, kinds=("direct",))
    v.set_rpm(60.0 * 30.0, 0.3)
    v.rate = 1.1
    v.paths[0].w = v.weights(np.full(ac.NB, 70.0), REF)
    x = render(v, 2.0)[:, 0]
    spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    fq = np.fft.rfftfreq(len(x), 1.0 / RATE)
    m = (fq > 50) & (fq < 80)
    assert fq[m][np.argmax(spec[m])] == pytest.approx(66.0, rel=0.01)


@pytest.mark.parametrize("rpm", [900.0, 1500.0, 2400.0, 3300.0])
def test_level_matches_the_band_levels_and_the_synthesised_engine(fake_sounds, rpm):
    want = np.array([75.0, 72, 70, 67, 65, 62, 58, 52])
    es = soundsamples.load_set("petrol", path=fake_sounds)
    new = soundsamples.SampleEngineVoice(es, 4, seed=1, kinds=("direct",))
    old = synth.EngineVoice(4, False, seed=1, kinds=("direct",))
    got = {}
    for name, v in (("new", new), ("old", old)):
        v.set_rpm(rpm, 0.3)
        v.paths[0].w = v.weights(want, REF)
        x = render(v, 3.0)
        got[name] = (level_dba(x), band_levels(x[:, 0] * math.sqrt(2)))
    # the same A-weighted level as the band levels ask for, and as the synthesised voice (within ~1 dB)
    assert got["new"][0] == pytest.approx(ac.a_weighted(want), abs=1.0)
    assert got["new"][0] == pytest.approx(got["old"][0], abs=1.2)
    # band by band near the CNOSSOS levels (a loop played up to half an octave off its pitch moves its bands; the
    # synthesised voice put up to ~12 dB too much into the 63 Hz band)
    d = got["new"][1] - want
    assert np.all(np.abs(d[1:7]) < 5.0), d


def test_recorded_engine_is_cheaper_than_the_synthesised_one(fake_sounds):
    import time
    es = soundsamples.load_set("petrol", path=fake_sounds)
    voices = {"new": soundsamples.SampleEngineVoice(es, 4, kinds=("direct",)),
              "old": synth.EngineVoice(4, False, kinds=("direct",))}
    for v in voices.values():
        v.set_rpm(1700.0, 0.3)
        v.basis(2048)
    # the two in turn, in short rounds, each judged by its fastest round: one voice timed after the other failed
    # now and then on a loaded machine (a burst of load during one of the two runs)
    costs = {name: float("inf") for name in voices}
    for _ in range(7):
        for name, v in voices.items():
            t = time.perf_counter()
            for _ in range(10):
                v.basis(2048)
            costs[name] = min(costs[name], time.perf_counter() - t)
    assert costs["new"] < 1.5 * costs["old"]


def test_fallback_to_the_synthesised_engine(tmp_path, monkeypatch):
    soundsamples._SETS.clear()
    monkeypatch.setattr(soundsamples, "MANIFEST", tmp_path / "missing.json")
    said = []
    v = soundsamples.engine_voice("car", 4, False, log=said.append)
    assert isinstance(v, synth.EngineVoice)
    assert len(said) == 1 and "synthesised engine" in said[0]
    soundsamples.engine_voice("car", 4, False, log=said.append)
    assert len(said) == 1                                  # said once
    assert isinstance(soundsamples.engine_voice("car", 4, False, mode="synth"), synth.EngineVoice)
    # a file named in the manifest but missing
    (tmp_path / "s.json").write_text(json.dumps({"engines": {"petrol": {"loops": [{"file": "nope.ogg",
                                                                                   "firing_hz": 30}]}}}))
    assert soundsamples.load_set("petrol", log=said.append, path=tmp_path / "s.json") is None
    soundsamples._SETS.clear()


def test_engine_voice_picks_the_set_by_kind(fake_sounds, monkeypatch):
    monkeypatch.setattr(soundsamples, "MANIFEST", fake_sounds)
    soundsamples._SETS.clear()
    assert isinstance(soundsamples.engine_voice("car", 4, False), soundsamples.SampleEngineVoice)
    # no diesel set in the fake manifest: the synthesised diesel
    assert isinstance(soundsamples.engine_voice("bus", 6, True, log=lambda m: None), synth.EngineVoice)


def _decoder():
    if shutil.which("ffmpeg"):
        return True
    return importlib.util.find_spec("pygame") is not None


@pytest.mark.skipif(not _decoder(), reason="neither ffmpeg nor pygame to decode the shipped OGG loops")
def test_shipped_loops_load_and_cover_the_engine_speeds():
    soundsamples._SETS.clear()
    man = soundsamples.manifest()
    for name, cyl, idle, top in (("petrol", 4, 800, 4000), ("diesel", 4, 750, 3500), ("motorbike", 2, 1200, 6000)):
        es = soundsamples.load_set(name)
        assert es is not None and len(es) == len(man["engines"][name]["loops"]) >= 3
        # the loops span idle to the upper revs within a pitch shift of 1.5 either way
        assert es.firing[0] <= 1.5 * cyl / 2 * idle / 60 and es.firing[-1] * 1.5 >= cyl / 2 * top / 60
        assert np.all(np.diff(es.lf) < math.log(2.5))      # no gap over 2.5x between neighbours
    for surf in ("shoes", "trainers", "grass"):
        steps = soundsamples.footstep_buffers(surf)
        assert steps and len(steps) >= 6 and all(0.1 * RATE < len(s) <= 0.5 * RATE for s in steps)
    soundsamples._SETS.clear()


def test_recorded_footsteps_and_their_variation(fake_sounds):
    steps = soundsamples.footstep_buffers("shoes", path=fake_sounds)
    assert len(steps) == 2
    assert all(audio.rms(s[:4410]) > 0.3 for s in steps)          # normalised like a cue
    said = []
    assert soundsamples.footstep_buffers("gravel", log=said.append, path=fake_sounds) is None and said
    v = synth.EventVoice(steps, seed=1, rate_sd=0.05, kinds=("direct",))
    v.period = 0.5
    lengths = set()
    for _ in range(40):
        v.basis(2048)
        lengths.update(ev[0].shape[1] for ev in v.active)
    assert len(lengths) > 3                                # each step resampled a little differently


def _drive(sc, kmh_of_t, seconds, type_id, electric, throttle=0.3):
    out = []
    x = 0.0
    for k in range(int(seconds / 0.05)):
        t = k * 0.05
        v = kmh_of_t(t) / 3.6
        x += v * 0.05
        own = soundscape.Body(1, type_id, "car", x, 0.0, 0.0, v, 0.0, 0.0, 0.0, electric)
        sc.update(t, (x, -0.35, 1.2, 0.0), [own], own=own, own_control=SimpleNamespace(throttle=throttle, brake=0))
        sc.mixer.render(int(0.05 * RATE))
        out.append(dict(sc.info["own"]))
    return out


def _scape(**cfg):
    return soundscape.Soundscape({"enabled": True, "environment": False, "buildings": False, "recordings": False,
                                  "reverb": False, **cfg}, kind="driver", log=lambda m: None)


def test_electric_own_car_has_no_engine_and_a_whine_rising_with_speed():
    sc = _scape()
    info = _drive(sc, lambda t: 10.0 + 40.0 * t / 4.0, 4.0, "vehicle.tesla.model3", True)
    assert all(i["engine_db"] is None for i in info)
    eng = sc.mixer.get(("own", "engine"))
    assert eng is None or not eng.audible()
    motor = [i["motor_db"] for i in info[5:]]
    assert all(m is not None for m in motor) and motor[-1] > motor[0] + 5.0         # +20 lg v from 12 to 50 km/h
    v = sc.mixer.get(("own", "motor"))
    assert isinstance(v, soundsamples.MotorVoice)
    assert v.base == pytest.approx(info[-1]["kmh"] * 72.0 / 60.0, rel=0.02)          # motor shaft frequency
    # the AVAS inside: on at low speed, faint, off above 20 km/h
    assert info[2]["avas_db"] is not None and info[2]["avas_db"] < 40.0
    assert info[-1]["avas_db"] is None
    # the whine sits under the tyres and wind at 50 km/h but within ~25 dB
    assert info[-1]["total_dba"] - 25.0 < motor[-1] < info[-1]["total_dba"]


def test_whine_louder_with_torque():
    assert soundsamples.motor_dba(50, 1.0) > soundsamples.motor_dba(50, 0.0) + 6.0
    assert soundsamples.motor_dba(2.0, 0.5) is None
    sc = _scape(ev_motor_db=0)
    info = _drive(sc, lambda t: 30.0, 1.0, "vehicle.tesla.model3", True)
    assert info[-1]["motor_db"] is None


def test_combustion_own_car_uses_the_recorded_engine(fake_sounds, monkeypatch):
    monkeypatch.setattr(soundsamples, "MANIFEST", fake_sounds)
    soundsamples._SETS.clear()
    sc = _scape()
    info = _drive(sc, lambda t: 0.0, 1.0, "vehicle.lincoln.mkz_2020", False, throttle=0.0)
    assert isinstance(sc.mixer.get(("own", "engine")), soundsamples.SampleEngineVoice)
    assert info[-1]["engine_db"] is not None and info[-1]["motor_db"] is None
    soundsamples._SETS.clear()


def test_part_load_quietens_a_cruising_engine():
    from types import SimpleNamespace as NS
    b = soundscape.Body(1, "vehicle.audi.a2", "car", 0, 0, 0, 50 / 3.6, 0, 0, 0)
    quiet = _scape()._source_power(b, NS(kmh=50.0, accel=0.0), 0.05)[1]
    full = _scape(engine_part_load_db=0)._source_power(b, NS(kmh=50.0, accel=0.0), 0.05)[1]
    assert np.allclose(full - quiet, 6.0 * (0.4 - 0.07) / 0.4, atol=0.5)
    # pulling away (load over 0.4) and idling are untouched
    acc = _scape()._source_power(b, NS(kmh=20.0, accel=2.0), 0.05)[1]
    acc0 = _scape(engine_part_load_db=0)._source_power(b, NS(kmh=20.0, accel=2.0), 0.05)[1]
    assert np.allclose(acc, acc0)


def test_own_footsteps_at_a_walkers_level(fake_sounds, monkeypatch):
    monkeypatch.setattr(soundsamples, "MANIFEST", fake_sounds)
    lv = {}
    for fw in ("shoes", "trainers"):
        soundsamples._STEPS.clear()
        sc = soundscape.Soundscape({"enabled": True, "environment": False, "buildings": False, "recordings": False,
                                    "footsteps": fw}, kind="pedestrian", log=lambda m: None)
        for k in range(20):
            sc.update(k * 0.05, (1.4 * k * 0.05, 0.0, 1.6, 0.0), [])
            sc.mixer.render(2205)
        lv[fw] = sc.own_steps["dba"]
        assert sc.steps_recorded == (fw == "shoes")        # no trainers in the fake set: synthesised
    assert 55.0 < lv["shoes"] < 65.0
    assert lv["trainers"] == pytest.approx(lv["shoes"] - 9.0)


def test_cluster_of_an_electric_car_shows_power_not_rpm():
    from coupledcarla.incar import InCar
    ic = InCar({"cluster": True}, "driver", (1280, 720), log=lambda m: None)
    ctl = SimpleNamespace(throttle=0.5, brake=0.0, steer=0.0, reverse=False)
    ic.update(0.0, 30.0, control=ctl)
    assert ic.values["rpm"] > 0 and ic.values["gear"].startswith("D")
    ic.set_electric(True)
    assert "electric" in ic.displays["cluster"]["design"]["name"].lower() or \
        any("{power}" in str(e.get("value", "")) for st in ic.displays["cluster"]["design"]["states"]
            for e in st["elements"])
    ic.update(0.1, 30.0, control=ctl)
    assert ic.values["rpm"] == 0 and ic.values["gear"] == "D" and ic.values["power"] == 50
    ic.update(0.2, 30.0, control=SimpleNamespace(throttle=0.0, brake=0.5, steer=0.0, reverse=False))
    assert ic.values["power"] < 0                          # regeneration
