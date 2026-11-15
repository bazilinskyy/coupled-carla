"""tools/test_carla10.py: reading a trial's log and summary into the table (no CARLA, no SUMO)."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def _tool():
    spec = importlib.util.spec_from_file_location("test_carla10_tool", ROOT / "tools" / "test_carla10.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


LOG = """[    0.1s] trial 0 -> /x/logs/t10_twin/20261115_trial0
[    3.3s] connected to CARLA 1.0-38b72cc52 at 127.0.0.1:2080
[   13.6s] world ready: 400 spawn points
 Retrying in 1 seconds
[   17.1s] traffic lights: 144 signal heads from SUMO (head) drawn near participants
[   18.0s] scenery: 2824 meshes within 400 m of the participants (0.3 s); 61 lamps with light within 200 m
[   18.1s] crowd: 24 of 24 walkers spawned
[transit       0.1s] live positions failed (No module named 'google'); buses spread by the timetable
"""


def test_trial_numbers_from_log_summary_and_events():
    t = _tool()
    lines, errors = t.parse_log(LOG)
    assert len(errors) == 1 and "live positions failed" in errors[0]       # rpclib's retry notice is not an error
    rec = {"lines": lines, "summary": {"sim_s": 60.02, "rtf": 1.15, "ticks": 3002, "collisions": 0,
                                       "sumo_vehicles_spawned": 2, "parked_cars": 739,
                                       "ms_per_tick": {"carla_tick": 9.9, "carla_to_sumo": 0.9, "sumo_step": 2.0}},
           "events": [{"event": "spawn", "detail": "bus vehicle.fuso.mitsubishi sumo:bus0"},
                      {"event": "spawn", "detail": "car vehicle.kia.niro sumo:car0"},
                      {"event": "reset_to_lane", "detail": ""}], "rc": 0, "server_died": None, "errors": errors}
    n = t.trial_numbers(rec)
    assert n["load_s"] == 10.3 and n["spawn_points"] == 400 and n["signals"] == 144 and n["scenery"] == 2824
    assert n["lamps_lit"] == 61 and n["crowd"] == 24 and n["buses"] == 1 and n["reset"] == 1
    assert n["ms_tick"] == 12.8 and n["sumo_vehicles"] == 2
    assert t.verdict(rec, 60) == "ok" and t.verdict(rec, 90) == "FAILED"
    assert t.verdict(dict(rec, server_died="watchdog: RSS 23 GB"), 60) == "FAILED"
    row = {"test": "twin", "result": "ok", "numbers": t.fmt(n, ["rtf", "signals"]), "notes": ""}
    assert "| ok | RTF 1.15, signals 144 |" in t.table([row])


def test_derived_experiment_attaches_to_the_server(tmp_path):
    t = _tool()

    class A:
        host, port, duration, render_checks = "127.0.0.1", 2080, 45.0, False

    class C:
        a = A()

    exp = t.derive("experiments/eindhoven_drones.yaml", "drones", C())
    assert exp["server"]["port"] == 2080 and exp["server"]["launch"] is False
    assert exp["simulation"]["profile"] is True and exp["logging"]["frames_every_s"] == 0.0
    assert exp["map"]["opendrive"].endswith("twins/EhvCentre.xodr") and Path(exp["map"]["opendrive"]).is_absolute()
    assert exp["trials"] == [{"duration_s": 45.0}] and exp["name"] == "t10_drones"
