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
