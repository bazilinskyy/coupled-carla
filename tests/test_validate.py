"""coupledcarla.validate: unknown keys, eHMI states, conditions, spawns and durations, offline."""
from pathlib import Path

import pytest
import yaml

from coupledcarla import config, validate

ROOT = Path(__file__).resolve().parent.parent
BBOX = [51.4360, 5.4700, 51.4500, 5.4930]


def run(tmp_path, doc, roads=False, **kw):
    p = tmp_path / "exp.yaml"
    p.write_text(yaml.safe_dump(doc, sort_keys=False))
    raw, cfg = validate.load(p)
    rep, plans = validate.check(raw, cfg, roads=roads, **kw)
    return rep, plans


def msgs(rep, level=None):
    return [f"{w}: {m}" for lv, w, m in rep.items if level in (None, lv)]


def base(**over):
    doc = {"name": "unit_test_exp", "map": {"name": "unit_test_missing_map", "bbox": BBOX},
           "ehmi": {"states": {"yielding": {"text": "STOP", "color": [0, 0, 255]}, "go": {"text": "GO"}}},
           "participants": [{"role": "pedestrian", "kind": "pedestrian", "station": True,
                             "spawn": {"lat": 51.44287, "lon": 5.48897, "heading": 165},
                             "response": {"button": "I would cross now", "slider": True}},
                            {"role": "av_passenger", "kind": "av_passenger", "agent": "av1",
                             "bot": {"ehmi": [[13, "yielding"], [16, "go"]]}}],
           "agents": [{"id": "av1", "route": [[51.44309, 5.49081], [51.44246, 5.48640]], "ehmi": "manual",
                       "triggers": [{"at_m": 30, "speed_kmh": 20}]}],
           "trials": [{"duration_s": 45}]}
    doc.update(over)
    return doc


def test_a_good_file_has_no_errors(tmp_path):
    rep, plans = run(tmp_path, base())
    assert not rep.errors, msgs(rep)
    assert any("not built yet" in m for m in msgs(rep, "warning"))
    assert list(plans) == [1] and len(plans[1]) == 1


def test_typos_undefined_ehmi_and_bad_roles(tmp_path):
    doc = base()
    doc["traffic"] = {"cars_per_hour": 100, "walkers": {"cuont": 3}}
    doc["simulaton"] = {"step": 0.05}
    doc["participants"][0]["respose"] = True
    doc["participants"][1]["bot"]["ehmi"].append([20, "driving"])         # not in ehmi.states
    doc["participants"].append({"role": "pedestrian", "kind": "cyclist", "input": "joystick"})
    doc["agents"][0]["triggers"].append({"at_m": 50, "ehmi": "wait", "speeed_kmh": 10})
    doc["agents"].append({"id": "car2", "route": [[51.4425, 5.4867], [51.4431, 5.4913]], "ehmi": "auto"})
    doc["participants"][1]["agent"] = "av2"
    rep, _ = run(tmp_path, doc)
    errs = "\n".join(msgs(rep, "error"))
    assert "traffic.walkers: unknown key 'cuont' (did you mean 'count'?)" in errs
    assert "top level: unknown key 'simulaton' (did you mean 'simulation'?)" in errs
    assert "unknown key 'respose' (did you mean 'response'?)" in errs
    assert "eHMI state 'driving' is not defined" in errs
    assert "eHMI state 'wait' is not defined" in errs
    assert "unknown key 'speeed_kmh'" in errs
    assert "ehmi auto shows 'driving', which ehmi.states does not define" in errs
    assert "roles are not unique" in errs
    assert "kind 'cyclist'" in errs and "input 'joystick'" in errs
    assert "agent 'av2' is not one of the agents (av1, car2)" in errs


def test_spawns_outside_overlapping_and_far_from_roads(tmp_path):
    doc = base()
    doc["participants"].append({"role": "driver", "kind": "driver",
                                "spawn": {"lat": 51.44310, "lon": 5.49083, "heading": 255}})   # 2 m from av1
    doc["agents"].append({"id": "walker1", "kind": "walker", "path": [[51.5, 5.48]]})          # outside the box
    doc["pois"] = [{"name": "crossing", "lat": 51.4428, "lon": 5.4889, "radus": 5}]

    class FakeRoads:                       # 30 m from a driving lane east of 5.49, else on one
        def distance(self, lat, lon, kind="car"):
            return 30.0 if lon > 5.490 else 1.0
    rep, _ = run(tmp_path, doc, roads=FakeRoads())
    errs = "\n".join(msgs(rep, "error"))
    warns = "\n".join(msgs(rep, "warning"))
    assert "starts 1.8 m from av1" in errs
    assert "51.500000, 5.480000 is outside the map's box" in errs
    assert "pois[0]: unknown key 'radus'" in errs
    assert "is 30 m from the nearest driving lane (moved onto it)" in warns


def test_conditions_design_and_durations(tmp_path):
    doc = base(trials=[{"duration_s": -5}, {"duration_s": 10, "condition": "fog"},
                       {"duration_s": 30, "patch": {"agents": {"av9": {"ehmi": "auto"}}}}],
               coupling={"visual_sync_at_s": [2, 40]})
    doc["participants"][1]["bot"]["ehmi"] = [[13, "yielding"], [50, "go"]]
    doc["conditions"] = {"dusk": {"simulation": {"wether": "ClearSunset"}}}
    rep, plans = run(tmp_path, doc)
    errs = "\n".join(msgs(rep, "error"))
    assert "condition 'fog' is not defined in conditions" in errs
    assert "conditions.dusk.simulation: unknown key 'wether'" in errs
    assert plans == {}                     # no plan without its conditions
    doc["trials"][1]["condition"] = "dusk"
    del doc["conditions"]["dusk"]["simulation"]["wether"]
    rep, plans = run(tmp_path, doc)
    errs = "\n".join(msgs(rep, "error"))
    warns = "\n".join(msgs(rep, "warning"))
    assert "duration_s -5 must be a positive number" in errs
    assert "agent 'av9' does not exist" in errs
    assert "visual sync at 40 s, after the trial's end (30 s)" in warns
    assert "at 50 s, after the trial's end (45 s)" not in warns and "at 50 s, after the trial's end (10 s)" in warns


def test_design_errors_and_plan_per_participant(tmp_path):
    doc = base(conditions={"a": {}, "b": {}, "c": {}},
               design={"factors": {"x": ["a", "b", "c"]}, "order": "latin", "block_by": "y"})
    rep, _ = run(tmp_path, doc)
    errs = "\n".join(msgs(rep, "error"))
    assert "design.block_by: 'y' is not one of the factors" in errs
    assert any("design.factors generates the trials" in m for m in msgs(rep, "warning"))
    del doc["design"]["block_by"]
    rep, plans = run(tmp_path, doc)
    assert not rep.errors, msgs(rep)
    assert sorted(plans) == list(range(1, 7))            # 3 blocks: 6 rows of the balanced Latin square
    assert [t["_condition"] for t in plans[1]] == ["a", "b", "c"]
    assert [t["_condition"] for t in plans[4]] == ["c", "b", "a"]
    doc["design"]["order"] = "sideways"
    rep, _ = run(tmp_path, doc)
    assert any("design.order" in m for m in msgs(rep, "error"))


def test_automation_and_response_settings(tmp_path):
    doc = base()
    doc["participants"].append({"role": "driver", "kind": "driver", "station": True,
                                "automation": {"route": [[51.4431, 5.4913]], "speeed_kmh": 30,
                                               "takeover": {"at_s": 60, "budget": 7, "no_reaction": "swerve"}}})
    doc["participants"][0]["response"] = "button,dial"
    rep, _ = run(tmp_path, doc)
    errs = "\n".join(msgs(rep, "error"))
    warns = "\n".join(msgs(rep, "warning"))
    assert "automation route: needs a list of at least 2 [lat, lon] points" in errs
    assert "automation: unknown key 'speeed_kmh'" in errs
    assert "automation takeover: unknown key 'budget' (did you mean 'budget_s'?)" in errs
    assert "no_reaction must be brake or keep" in errs
    assert "request at 60 s, after the trial's end (45 s)" in warns
    assert "unknown dial" in errs


@pytest.mark.parametrize("path", sorted((ROOT / "experiments").glob("*.yaml")) +
                         sorted((ROOT / "experiments" / "cities").glob("*.yaml")))
def test_the_shipped_experiments_pass(path):
    if path.name == "cities.yaml":
        pytest.skip("the city catalogue, not an experiment")
    raw, cfg = validate.load(path)
    rep, _ = validate.check(raw, cfg, roads=False, participants=[])
    errs = [m for m in msgs(rep, "error") if "map.opendrive" not in m]     # the twins' OpenDRIVE is not in git
    assert not errs


def test_main_prints_the_plan(tmp_path, capsys):
    p = tmp_path / "exp.yaml"
    p.write_text(yaml.safe_dump(base(conditions={"a": {}, "b": {}},
                                     design={"factors": {"x": ["a", "b"]}, "trial": {"duration_s": 60}}),
                                sort_keys=False))
    assert validate.main([str(p), "--no-roads", "--participants", "1-2"]) == 0
    out = capsys.readouterr().out
    assert "0 errors" in out and "participant 2  (2.0 min of trials)" in out
    assert out.index("block b", out.index("participant 2")) < out.index("block a", out.index("participant 2"))
    assert config.DEFAULTS["design"]["order"] == "latin"
