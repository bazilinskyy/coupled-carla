"""The soundscape's sound events (horns, bells, a shout, hazard lights), wipers, rain on an umbrella and in a parked
car, and the ambient bed of places (soundplaces.py). Clips are fake WAV files in a temporary folder (no decoder)."""
import json
import math
import wave
from types import SimpleNamespace

import numpy as np
import pytest

from coupledcarla import acoustics as ac
from coupledcarla import audio, soundplaces, soundsamples, soundscape
from coupledcarla.soundrec import level_dba

RATE = audio.RATE


def _wav(path, x):
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes((np.clip(x, -1, 1) * 32767).astype("<i2").tobytes())


def _tone(f, s, seed=0):
    t = np.arange(int(s * RATE)) / RATE
    rng = np.random.default_rng(seed)
    return 0.3 * (np.sin(2 * np.pi * f * t) + 0.3 * np.sin(2 * np.pi * 2.1 * f * t)) + 0.01 * rng.standard_normal(len(t))


@pytest.fixture
def clips(tmp_path, monkeypatch):
    man = {"clips": {}}
    slots = {"horn_car": (0.9, [400, 450, 500]), "horn_bus": (3.0, [300]), "horn_truck": (0.7, [250]),
             "bicycle_bell": (1.8, [3000, 3300]), "shout": (1.0, [600]), "blinker": (0.3, [2000, 1800]),
             "wiper": (0.9, [700, 650]), "church_bell": (2.4, [440])}
    for name, (slot, fs) in slots.items():
        n = int(slot * RATE)
        buf = np.zeros(n * len(fs))
        for k, f in enumerate(fs):
            d = 0.6 * slot
            x = _tone(f, d, k) * np.minimum(1.0, np.linspace(0, 30, int(d * RATE)))
            buf[k * n:k * n + len(x)] = x
        _wav(tmp_path / f"events/{name}.wav", buf)
        man["clips"][name] = {"file": f"events/{name}.wav", "kind": "slots", "slot_s": slot, "items": len(fs)}
    for name in ("rain_in_car", "walla", "cafe", "construction", "fountain", "crickets", "cicadas", "leaves"):
        rng = np.random.default_rng(len(name))
        _wav(tmp_path / f"ambience/{name}.wav", 0.1 * rng.standard_normal(3 * RATE))
        man["clips"][name] = {"file": f"ambience/{name}.wav", "kind": "loop"}
    p = tmp_path / "sounds.json"
    p.write_text(json.dumps(man))
    monkeypatch.setattr(soundsamples, "MANIFEST", p)
    soundsamples._CLIPS.clear()
    soundsamples._SETS.clear()
    soundsamples._STEPS.clear()
    yield p
    soundsamples._CLIPS.clear()


def _scape(kind="pedestrian", env=None, places=None, **cfg):
    return soundscape.Soundscape({"enabled": True, "environment": False, "buildings": False, "recordings": False,
                                  "reverb": False, "vehicles": True, **cfg}, kind=kind, log=lambda m: None, env=env,
                                 places=places)


def _car(i, x, y, kmh=0.0, kind="car"):
    return soundscape.Body(i, "vehicle.audi.a2", kind, x, y, 0.0, kmh / 3.6, 0.0, 0.0, 0.0)


def test_horn_events_short_long_double_and_per_vehicle_type(clips):
    sc = _scape()
    cues = []
    sc.on_cue = lambda *a: cues.append(a)
    car, bus = _car(5, 10.0, 3.0), _car(6, 20.0, 3.0, kind="bus")
    sc.update(0.0, (0.0, 0.0, 1.6, 0.0), [car, bus])
    lens = {}
    for k in ("short", "long", "double"):
        assert sc.event("horn", 5, kind=k)
        v = sc.mixer.get(max((key for key in sc.mixer.keys() if key[0] == "event"), key=lambda q: q[3]))
        lens[k] = v.n / RATE
    assert lens["short"] == pytest.approx(0.25, abs=0.02) and lens["long"] == pytest.approx(1.2, abs=0.02)
    assert lens["double"] == pytest.approx(0.18 * 2 + 0.12, abs=0.02)
    assert ("sound_event", "horn", "short 5") in cues
    # the same car always has the same horn; a bus has its own
    a = soundsamples.horn_buffer(sc.horns["car"][0], 0.3)
    assert len(sc.horns["car"]) == 3 and len(sc.horns["bus"]) == 1
    assert sc.event("horn", 6, kind="long") and len(a) == int(0.3 * RATE)
    # an actor never seen: nothing, said once
    assert not sc.event("horn", 999)
    assert not sc.event("nonsense", 5)


def test_horn_level_near_the_regulation_range(clips):
    sc = _scape(reflections=0)
    car = _car(5, 7.0, 0.0)
    for k in range(3):
        sc.update(0.05 * k, (0.0, 0.0, 1.2, 0.0), [car], listener_h=1.2)
    sc.mixer.remove(("veh", 5, "engine"))
    sc.mixer.remove(("veh", 5, "tyre"))
    sc.event("horn", 5, kind="long")
    x = sc.mixer.render(int(1.2 * RATE))
    loud = max(level_dba(x[i:i + 4410], sc.ref) for i in range(0, len(x) - 4410, 2205)) + 3.0   # centre pan
    # 95 dB(A) at 7 m, its loudest 100 ms (UN R28 asks 87-112 dB(A) at 7 m of a car's horn); ground and air +-
    assert 92.0 < loud < 99.0


def test_bell_shout_and_the_own_horn_in_the_cabin(clips):
    sc = _scape()
    bike = soundscape.Body(7, "vehicle.gazelle.omafiets", "bike", 5.0, 2.0, 0.0, 4.0, 0.0, 0.0, 0.0)
    walker = soundscape.Body(8, "walker.pedestrian.0001", "walker", 3.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    sc.update(0.0, (0.0, 0.0, 1.6, 0.0), [bike, walker])
    assert sc.event("bell", 7) and sc.event("bell", 7, kind="double") and sc.event("shout", 8)
    assert sc.used_credits() is not None
    drv = _scape("driver")
    own = _car(1, 0.0, 0.0)
    drv.update(0.0, (0.0, 0.0, 1.2, 0.0), [own], own=own)
    assert drv.event("horn", None, kind="short")            # the participant's own horn, through the cabin
    v = drv.mixer.get(next(k for k in drv.mixer.keys() if k[0] == "event"))
    assert len(v.paths) == 1


def test_hazard_event_ticks_the_relay(clips):
    sc = _scape("driver")
    own = _car(1, 0.0, 0.0)
    ticks = []
    sc.on_cue = lambda kind, name, detail="": ticks.append(name) if name == "blinker_tick" else None
    sc.event("hazard", kind="on")
    for k in range(40):                                   # 2 s
        sc.update(0.05 * k, (0.0, 0.0, 1.2, 0.0), [own], own=own)
    assert 5 <= ticks.count("blinker_tick") <= 7          # every 1/3 s
    sc.event("hazard", kind="off")
    n = ticks.count("blinker_tick")
    for k in range(40, 60):
        sc.update(0.05 * k, (0.0, 0.0, 1.2, 0.0), [own], own=own)
    assert ticks.count("blinker_tick") == n
    assert sc.relay and len(sc.relay) == 2                # the recorded relay's on and off clicks


@pytest.mark.parametrize("rain,per_min", [(0.0, 0), (0.5, 12), (2.0, 45), (8.0, 65)])
def test_wipers_follow_the_rain(clips, rain, per_min):
    sc = _scape("driver", env={"rain": rain})
    own = _car(1, 0.0, 0.0, 30.0)
    for k in range(10):
        sc.update(0.05 * k, (0.0, 0.0, 1.2, 0.0), [own], own=own)
    v = sc.mixer.get(("own", "wiper"))
    if not per_min:
        assert v is None
    else:
        assert v.period == pytest.approx(30.0 / per_min)
        assert sc.info["own"]["wiper_per_min"] == per_min
    off = _scape("driver", env={"rain": 2.0}, wipers="off")
    off.update(0.0, (0.0, 0.0, 1.2, 0.0), [own], own=own)
    assert off.mixer.get(("own", "wiper")) is None


def test_rain_on_an_umbrella_and_in_a_parked_car(clips):
    sc = _scape(env={"rain": 3.0}, umbrella=True, environment=True)
    for k in range(20):
        sc.update(0.05 * k, (0.0, 0.0, 1.6, 0.0), [])
        sc.mixer.render(2205)
    v = sc.mixer.get(("own", "umbrella"))
    assert v is not None and v.period == pytest.approx(1.0 / 198.0) and v.events > 50
    dry = _scape(env={"rain": 0.0}, umbrella=True)
    dry.update(0.0, (0.0, 0.0, 1.6, 0.0), [])
    assert dry.mixer.get(("own", "umbrella")) is None
    car = _scape("driver", env={"rain": 3.0}, environment=True)
    own = _car(1, 0.0, 0.0)
    for k in range(10):
        car.update(0.05 * k, (0.0, 0.0, 1.2, 0.0), [own], own=own)
    rc = car.ambience.rain_car
    assert rc is not None and np.any(rc.paths[0].w > 0)
    assert "rain_in_car" in car.ambience.used


def test_wet_road_hiss_is_louder(clips):
    from types import SimpleNamespace as NS
    b = _car(1, 0.0, 0.0, 50.0)
    dry = _scape(env={"rain": 0.0})._source_power(b, NS(kmh=50.0, accel=0.0), 0.05)[0]
    wet = _scape(env={"rain": 2.0})._source_power(b, NS(kmh=50.0, accel=0.0), 0.05)[0]
    d = wet - dry
    assert d[:4].max() < 1.0 and d[6] > 5.0 and d[7] > 8.0    # IMAGINE: the wet hiss above 2 kHz


def _places():
    return {"cafe": [[12.0, 0.0]], "construction": [[60.0, 40.0]], "fountain": [[300.0, 0.0]],
            "church": [[200.0, 100.0]], "green": [[0.0, 30.0, 40.0]], "tunnel": [[-100.0, -1.0, -60.0, -1.0]]}


def test_places_bed_by_place_and_hour(clips):
    walkers = [soundscape.Body(100 + i, "walker.pedestrian.0001", "walker", 3.0 + i, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0)
               for i in range(4)]
    noon = _scape(env={"local_time": "2026-06-17 12:30", "temperature": 20.0}, places=_places())
    for k in range(5):
        noon.update(0.05 * k, (0.0, 0.0, 1.6, 0.0), walkers)
    info = noon.places.info
    assert "cafe" in info and 40.0 < info["cafe"] < 60.0          # the terrace 12 m away
    assert "construction" in info                                  # a weekday at noon
    assert "fountain" not in info                                  # 300 m: out of reach
    assert info["walla"] == pytest.approx(44.0 + 10 * math.log10(4))
    assert "crickets" not in info and "cicadas" not in info
    night = _scape(env={"local_time": "2026-07-04 23:30", "temperature": 19.0}, places=_places())   # a Saturday
    night.update(0.0, (0.0, 0.0, 1.6, 0.0), [])
    assert "crickets" in night.places.info and "cafe" not in night.places.info
    assert "construction" not in night.places.info
    hot = _scape(env={"local_time": "2026-07-01 14:00", "temperature": 31.0, "lat": 38.0}, places=_places(),
                 trees=0.8)
    hot.update(0.0, (0.0, 0.0, 1.6, 0.0), [])
    assert "cicadas" in hot.places.info


def test_church_bells_strike_the_hour(clips):
    sc = _scape(env={"local_time": "2026-06-17 14:59"}, places=_places())
    cues = []
    sc.on_cue = lambda *a: cues.append(a)
    t = 0.0
    while t < 75.0:                                  # 14:59:00 -> 15:00:15
        sc.update(t, (150.0, 80.0, 1.6, 0.0), [])
        sc.mixer.render(int(0.1 * RATE))
        t += 0.1
    assert any(c[1] == "church_bells" and c[2].startswith("3 strikes") for c in cues)


def test_tunnel_lengthens_the_tail(clips):
    sc = _scape(places=_places())
    sc.update(0.0, (-80.0, 0.0, 1.6, 0.0), [])
    assert sc.places.in_tunnel
    sc.update(0.1, (0.0, 0.0, 1.6, 0.0), [])
    assert not sc.places.in_tunnel


def test_places_from_a_map_folder(tmp_path):
    (tmp_path / "real").mkdir()
    (tmp_path / "meta.json").write_text(json.dumps({"bbox": [51.0, 5.0, 51.01, 5.01]}))
    (tmp_path / "real" / "overture_places.json").write_text(json.dumps([[51.005, 5.005, 1, "food_and_drink"],
                                                                         [51.006, 5.005, 1, "shopping"],
                                                                         [51.007, 5.005, 1, "education"]]))
    (tmp_path / "real" / "osm_scenery.json").write_text(json.dumps({"elements": [
        {"type": "node", "lat": 51.004, "lon": 5.004, "tags": {"amenity": "fountain"}},
        {"type": "way", "tags": {"leisure": "park"}, "geometry": [{"lat": 51.001, "lon": 5.001},
                                                                  {"lat": 51.001, "lon": 5.002},
                                                                  {"lat": 51.002, "lon": 5.002}]}]}))
    (tmp_path / "real" / "osm_sound.json").write_text(json.dumps({"elements": [
        {"type": "way", "center": {"lat": 51.003, "lon": 5.003}, "tags": {"amenity": "place_of_worship"}},
        {"type": "way", "center": {"lat": 51.002, "lon": 5.003}, "tags": {"landuse": "construction"}}]}))
    (tmp_path / "map.osm").write_text("""<osm><node id="1" lat="51.0" lon="5.0"/><node id="2" lat="51.0" lon="5.001"/>
<way id="9"><nd ref="1"/><nd ref="2"/><tag k="highway" v="tertiary"/><tag k="tunnel" v="yes"/></way></osm>""")
    pl = soundplaces.places(tmp_path, log=lambda m: None, fetch=False,
                            to_xy=lambda lat, lon: ((lon - 5.0) * 70000.0, -(lat - 51.0) * 111000.0))
    assert len(pl["cafe"]) == 1 and len(pl["shop"]) == 1 and len(pl["fountain"]) == 1 and len(pl["green"]) == 1
    assert len(pl["church"]) == 1 and len(pl["construction"]) == 1 and len(pl["tunnel"]) == 1
    assert (tmp_path / "sound_places.json").exists()
    assert soundplaces.places(tmp_path, log=lambda m: None, fetch=False, to_xy=None) == pl     # the cache


def test_event_names_documented():
    assert set(soundscape.Soundscape.EVENTS) == {"horn", "bell", "shout", "hazard", "indicator"}
    assert ac.NB == 8
    assert SimpleNamespace is not None
