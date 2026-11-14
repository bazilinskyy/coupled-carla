"""coupledcarla.stationux and the configuration: all off by default (existing experiments unchanged), the host's
welcome for each station of the demo experiment, and a whole station trial with a fake CARLA world: sound
recorded headless, in-car messages and a take-over request, haptic cues, the cue log. No CARLA, no pygame."""
import csv
import json
import math
import types
import wave
from pathlib import Path

import pytest

from coupledcarla import config, stationux

ROOT = Path(__file__).resolve().parent.parent
STATES = ["off", "driving", "yielding", "go"]


def test_defaults_are_off_and_trials_can_override(tmp_path):
    d = config.DEFAULTS
    assert d["sound"]["enabled"] is False and d["haptics"]["enabled"] is False
    assert not any(d["displays"][k] for k in ("cluster", "hud", "centre", "messages", "messages_file"))
    p = tmp_path / "e.yaml"
    p.write_text("name: e\ntrials:\n  - {duration_s: 5}\n  - {duration_s: 5, sound: {enabled: true}, "
                 "displays: {hud: hud_speed}}\n")
    cfg = config.load(p)
    assert config.trial_config(cfg, cfg["trials"][0])["sound"]["enabled"] is False
    t1 = config.trial_config(cfg, cfg["trials"][1])
    assert t1["sound"]["enabled"] is True and t1["sound"]["reference_db"] == 94.0 and t1["displays"]["hud"]


def test_existing_experiments_ask_for_nothing():
    for name in ("eindhoven_crossing", "eindhoven_crowd", "eindhoven_centre"):
        cfg = config.load(ROOT / "experiments" / f"{name}.yaml")
        tc = config.trial_config(cfg, cfg["trials"][0])
        for p in tc["participants"]:
            w = stationux.welcome_extras(tc, p, p.get("kind"), ROOT / "experiments", default_states=STATES)
            assert not stationux.wanted(w), (name, p["role"])
            assert "ehmi_designs" not in w and "ehmi_states" not in w


def test_demo_experiment_welcomes():
    cfg = config.load(ROOT / "experiments" / "eindhoven_station_ux.yaml")
    tc = config.trial_config(cfg, cfg["trials"][0])
    w = {p["role"]: stationux.welcome_extras(tc, p, p.get("kind"), ROOT / "experiments", default_states=STATES)
         for p in tc["participants"]}
    for role, x in w.items():
        json.loads(json.dumps(x))                            # goes over the control channel
        assert stationux.wanted(x), role
        assert x["sound"]["enabled"] and x["sound"]["auditory_ehmi"]["cues"]
        assert set(x["ehmi_designs"]) == {"av1", "car2"}
    drv = w["driver"]["displays"]
    assert drv["cluster"]["design"]["name"].startswith("Instrument cluster")
    assert [m["text"] for m in drv["messages"]][-1] == "TAKE OVER" and drv["messages_file"] is None
    assert w["av_passenger"]["displays"]["centre"]["design"]["name"].startswith("Centre screen")
    assert w["av_passenger"]["ehmi_states"] == ["yielding", "go", "driving", "wait"]
    assert w["pedestrian"]["displays"]["cluster"] is None
    assert w["driver"]["sim_time"] == "14:30"


# -- a fake CARLA world for one station ------------------------------------------------------------------------

class V:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

    def distance(self, o):
        return math.dist((self.x, self.y, self.z), (o.x, o.y, o.z))


class Actor:
    def __init__(self, i, tid, x, y, yaw, speed, lights=0):
        self.id, self.type_id = i, tid
        self.attributes = {"number_of_wheels": "4", "role_name": ""}
        self.x, self.y, self.yaw, self.speed, self.lights = x, y, yaw, speed, lights
        self.bounding_box = types.SimpleNamespace(extent=V(2.3, 0.9, 0.75))
        self.ctl = types.SimpleNamespace(throttle=0.3, brake=0.0, steer=0.0, reverse=False)

    def step(self, dt):
        self.x += self.speed * math.cos(math.radians(self.yaw)) * dt
        self.y += self.speed * math.sin(math.radians(self.yaw)) * dt

    def get_transform(self):
        return types.SimpleNamespace(location=V(self.x, self.y, 0.0),
                                     rotation=types.SimpleNamespace(yaw=self.yaw, pitch=0.0, roll=0.0))

    def get_location(self):
        return V(self.x, self.y, 0.0)

    def get_velocity(self):
        return V(self.speed * math.cos(math.radians(self.yaw)), self.speed * math.sin(math.radians(self.yaw)))

    def get_control(self):
        return self.ctl

    def get_light_state(self):
        return self.lights


class Snapshot:
    def __init__(self, actors):
        self.actors = {a.id: a for a in actors}

    def __iter__(self):
        return iter(self.actors.values())

    def find(self, i):
        return self.actors.get(i)


def test_station_trial_with_a_fake_world(tmp_path):
    own = Actor(10, "vehicle.lincoln.mkz_2020", 0.0, 0.0, 0.0, 10.0, lights=32)
    av = Actor(11, "vehicle.tesla.model3", 30.0, 3.5, 180.0, 4.0)
    car = Actor(12, "vehicle.audi.a2", -40.0, 3.5, 0.0, 12.0)
    actors = [own, av, car]
    cmap = types.SimpleNamespace(to_opendrive=lambda: "<OpenDRIVE/>", get_waypoint=lambda loc: None)
    world = types.SimpleNamespace(get_map=lambda: cmap,
                                  get_actors=lambda ids: [a for a in actors if a.id in ids],
                                  get_actor=lambda i: next((a for a in actors if a.id == i), None))
    cfg = config.load(ROOT / "experiments" / "eindhoven_station_ux.yaml")
    tc = config.trial_config(cfg, cfg["trials"][0])
    part = [p for p in tc["participants"] if p["role"] == "driver"][0]
    welcome = {"kind": "driver", **stationux.welcome_extras(tc, part, "driver", ROOT / "experiments",
                                                            default_states=STATES)}
    welcome["sound"]["auditory_ehmi"]["cues"] = [{"ehmi": "yielding", "sound": "chime"}]   # no speech here
    welcome["displays"]["messages"][-1]["at"] = 2.0                                     # the take-over at 2 s
    logs = []
    ux = stationux.StationUX(welcome, "driver", tmp_path / "driver_test", (1280, 720), headless=True,
                             log=logs.append)
    ux.start(world, own, own, 0, agents={"av1": 11, "car2": 12})
    assert ux.scape is not None and ux.incar is not None and ux.haptics is not None
    ux.on_message({"type": "ehmi_state", "all": {"11": "driving", "12": "driving"}})
    t = 0.0
    for k in range(80):
        if k == 40:
            ux.on_message({"type": "ehmi_state", "actor_id": 11, "state": "yielding"})
        for a in actors:
            a.step(0.05)
        ux.tick(Snapshot(actors), t)
        t += 0.05
    info = ux.scape.voices_info()
    assert info[11]["electric"] and info[11]["engine_db"] is None and info[11]["avas_db"] is not None
    assert info[12]["engine_db"] is not None and "own" in info
    ux.stop()
    rows = list(csv.reader(open(tmp_path / "driver_test_cues.csv"), delimiter=";"))
    kinds = [(r[3], r[4]) for r in rows[1:]]
    assert ("sound_cue", "chime") in kinds                     # the AV's eHMI changed to yielding
    assert ("takeover_request", "TAKE OVER") in kinds and ("haptic", "takeover") in kinds
    assert ("sound_cue", "takeover") in kinds and ("message", "Follow the blue car") in kinds
    assert any(k == ("sound_cue", "blinker_tick") for k in kinds)
    with wave.open(str(tmp_path / "driver_test_trial0.wav")) as wv:
        assert wv.getnframes() > 0
    assert any("sound: on" in m for m in logs)
