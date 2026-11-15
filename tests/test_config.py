"""coupledcarla.config: defaults, merging and per-trial overrides."""
import yaml

from coupledcarla import config


def test_merge_is_deep_but_replaces_type_mixes():
    base = {"a": {"b": 1, "c": 2}, "traffic": {"car_types": {"x": 1, "y": 1}}}
    out = config._merge(base, {"a": {"c": 3}, "traffic": {"car_types": {"z": 5}}})
    assert out == {"a": {"b": 1, "c": 3}, "traffic": {"car_types": {"z": 5}}}
    assert base["a"]["c"] == 2                         # the base is not changed


def test_load_and_trial(tmp_path):
    p = tmp_path / "exp.yaml"
    p.write_text(yaml.safe_dump({
        "name": "demo",
        "traffic": {"cars_per_hour": 100},
        "trials": [{"duration_s": 10}, {"duration_s": 20, "traffic": {"cars_per_hour": 900},
                                        "eye_contact": {"angle_deg": 15}, "name": "ignored"}]}))
    cfg = config.load(p)
    assert cfg["map"]["name"] == "demo"
    assert cfg["logging"]["analyse"] is True
    assert cfg["traffic"]["walkers"]["count"] == 0
    t1 = config.trial_config(cfg, cfg["trials"][1])
    assert t1["traffic"]["cars_per_hour"] == 900
    assert t1["eye_contact"] == {"angle_deg": 15}
    assert t1["name"] == "demo"                        # only the listed sections can be overridden
    assert cfg["traffic"]["cars_per_hour"] == 100


def test_scenery_false_means_off(tmp_path):
    from coupledcarla import config
    f = tmp_path / "e.yaml"
    f.write_text("name: e\nmap: {bbox: [51.43, 5.47, 51.45, 5.49], scenery: false}\nsimulation: {scenery: false}\n")
    cfg = config.load(f)
    assert cfg["map"]["scenery"]["enabled"] is False and cfg["map"]["scenery"]["trees"] is True
    assert cfg["simulation"]["scenery"]["kinds"] == []
    t = config.trial_config(cfg, {"duration_s": 10, "simulation": {"scenery": False}})
    assert t["simulation"]["scenery"]["kinds"] == []
