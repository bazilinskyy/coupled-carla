"""The digital-twin experiments (baked CARLA 1.0 levels driven on their own OpenDRIVE): config only, no files."""
from pathlib import Path

import pytest

from coupledcarla import config

EXP = Path(__file__).resolve().parent.parent / "experiments"


@pytest.mark.parametrize("name,level", [("eindhoven_twin", "EhvCentre"), ("nyc_twin", "NycMidtown")])
def test_twin_experiments_name_their_level_and_opendrive(name, level):
    cfg = config.load(EXP / f"{name}.yaml")
    m = cfg["map"]
    assert m["carla_level"] == level
    assert m["opendrive"] == f"twins/{level}.xodr"
    assert config.map_dir(cfg).name == name


def test_nyc_twin_is_the_midtown_city_box():
    twin = config.load(EXP / "nyc_twin.yaml")
    city = config.load(EXP / "cities" / "new_york_midtown.yaml")
    assert twin["map"]["bbox"] == city["map"]["bbox"]
    s, w, n, e = twin["map"]["bbox"]
    for p in twin["pois"] + [twin["participants"][0]["spawn"]]:
        assert s <= p["lat"] <= n and w <= p["lon"] <= e, p


def test_city_twin_names_its_level_and_keeps_every_spawn_in_the_box():
    cfg = config.load(EXP / "eindhoven_city_twin.yaml")
    m = cfg["map"]
    assert m["carla_level"] == "EhvCity" and m["opendrive"] == "twins/EhvCity.xodr"
    assert config.map_dir(cfg).name == "eindhoven_city_twin"
    s, w, n, e = m["bbox"]
    spawns = [cfg["participants"][0]["spawn"]]
    for t in cfg["trials"]:
        tc = config.trial_config(cfg, t)
        spawns.append(tc["participants"][0]["spawn"])
    assert len({(p["lat"], p["lon"]) for p in spawns}) == 3   # station, TU/e campus, Veldhoven
    for p in spawns + cfg["pois"]:
        assert s <= p["lat"] <= n and w <= p["lon"] <= e, p


def test_city_real_is_the_city_twin_under_google_tiles_with_live_data():
    real = config.load(EXP / "eindhoven_city_real.yaml")
    twin = config.load(EXP / "eindhoven_city_twin.yaml")
    m = real["map"]
    assert m["name"] == "eindhoven_city_twin"
    assert (m["opendrive"], m["carla_level"], m["bbox"]) == (twin["map"]["opendrive"], twin["map"]["carla_level"],
                                                             twin["map"]["bbox"])
    assert m["photoreal"]["provider"] == "google" and m["photoreal"]["carve"] == "carriageway"
    assert m["scenery"]["trees"] is False and m["scenery"]["buildings"] is False
    assert real["simulation"]["time"] == "now" and real["simulation"]["weather"] == "real"
    assert real["traffic"]["cars_per_hour"] == "auto"
    assert [t.get("patch") for t in real["trials"]] == [t.get("patch") for t in twin["trials"]]


def test_city_quick_is_the_city_twin_as_a_stand_in():
    quick = config.load(EXP / "eindhoven_city_quick.yaml")
    twin = config.load(EXP / "eindhoven_city_twin.yaml")
    assert quick["map"]["name"] == "eindhoven_city_twin" and not quick["map"].get("carla_level")
    assert quick["map"]["opendrive"] == twin["map"]["opendrive"] and quick["map"]["standin"]["radius_m"] == 800
