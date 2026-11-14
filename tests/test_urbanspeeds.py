"""Urban default limits in the map's type file (urbanspeeds.py): no SUMO binary, no network."""
import json
import xml.etree.ElementTree as ET

import pytest

from coupledcarla import speeds as S
from coupledcarla import urbanspeeds as U

BASE = """<?xml version="1.0" encoding="UTF-8"?>
<types xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="x.xsd">
    <!-- a comment -->
    <type id="highway.motorway"       numLanes="2" speed="39.44" priority="14" oneway="true"  disallow="pedestrian"/>
    <type id="highway.motorway_link"  numLanes="1" speed="22.22" priority="9"  oneway="true"/>
    <type id="highway.trunk"          numLanes="2" speed="27.78" priority="13" oneway="false"/>
    <type id="highway.primary"        numLanes="2" speed="27.78" priority="12" oneway="false" disallow="rail"/>
    <type id="highway.secondary_link" numLanes="1" speed="22.22" priority="6"  oneway="false"/>
    <type id="highway.tertiary"       numLanes="1" speed="22.22" priority="10" oneway="false"/>
    <type id="highway.residential"    numLanes="1" speed="13.89" priority="3"  oneway="false"/>
    <type id="highway.living_street"  numLanes="1" speed="2.78"  priority="2"  oneway="false"/>
    <type id="highway.service"        numLanes="1" speed="5.56"  priority="1"  oneway="false" allow="delivery"/>
    <type id="highway.busway"         numLanes="1" speed="27.78" priority="1"  oneway="true"  allow="bus"/>
    <type id="railway.tram"           numLanes="1" speed="27.78" priority="1"  oneway="true"/>
</types>
"""


def _types(path):
    return {t.get("id"): t.attrib for t in ET.parse(path).getroot().iter("type")}


@pytest.fixture
def base(tmp_path):
    p = tmp_path / "osmNetconvert.typ.xml"
    p.write_text(BASE)
    return p


def test_us_urban_speeds_lower_only_the_city_road_types(tmp_path, base):
    kmh = U.write_typemap(tmp_path / "urban.typ.xml", "US", base)
    assert kmh == {"busway": 40, "primary": 40, "residential": 40, "secondary_link": 40, "tertiary": 40, "trunk": 40}
    t = _types(tmp_path / "urban.typ.xml")
    assert t["highway.primary"]["speed"] == "11.17"
    assert t["highway.motorway"]["speed"] == "39.44" and t["highway.motorway_link"]["speed"] == "22.22"
    assert t["highway.living_street"]["speed"] == "2.78" and t["highway.service"]["speed"] == "5.56"
    assert t["railway.tram"]["speed"] == "27.78"
    # everything else as in SUMO's file
    assert t["highway.primary"]["priority"] == "12" and t["highway.primary"]["disallow"] == "rail"
    assert t["highway.primary"]["numLanes"] == "2" and t["highway.service"]["allow"] == "delivery"


def test_country_tables(tmp_path, base):
    # Japan: 60 on general roads, 30 on residential/unclassified streets (no centre line)
    assert U.write_typemap(tmp_path / "jp.typ.xml", "JP", base) == {
        "busway": 60, "primary": 60, "residential": 30, "secondary_link": 60, "tertiary": 60, "trunk": 60}
    # the Netherlands: 50, and residential streets keep netconvert's 50 (nothing to lower)
    assert U.write_typemap(tmp_path / "nl.typ.xml", "NL", base) == {
        "busway": 50, "primary": 50, "secondary_link": 50, "tertiary": 50, "trunk": 50}
    # South Africa's 60 never raises a residential street's 50
    assert "residential" not in U.write_typemap(tmp_path / "za.typ.xml", "ZA", base)
    # a country without an entry: 50
    assert U.write_typemap(tmp_path / "la.typ.xml", None, base)["primary"] == 50


def test_urban_limit_table():
    assert S.urban_limit("JP", "residential") == 30 and S.urban_limit("JP", "primary") == 60
    assert S.urban_limit("TW", "unclassified") == 30 and S.urban_limit("TW", "secondary_link") == 50
    assert S.urban_limit("EG", "primary") == 60 and S.urban_limit("RW", "trunk") == 40


def test_netconvert_args(tmp_path, base, monkeypatch):
    logs = []
    cfg = {"city": {"country": "Taiwan"}, "map": {"bbox": [25.04, 121.51, 25.05, 121.52]}}
    monkeypatch.setattr(U, "base_typemap", lambda: base)
    args, meta = U.netconvert_args(cfg, tmp_path, logs.append)
    assert args == ["--type-files", str(tmp_path / "urban.typ.xml")]
    assert meta["country"] == "TW" and meta["file"] == "urban.typ.xml" and meta["kmh"]["primary"] == 50
    assert meta["kmh"]["residential"] == 30
    json.dumps(meta)
    assert "urban default of TW" in logs[-1]
    # switched off
    assert U.netconvert_args(dict(cfg, map=dict(cfg["map"], urban_speeds=False)), tmp_path, logs.append) == ([], None)
    # SUMO's type file missing: netconvert's defaults, one log line
    monkeypatch.setattr(U, "base_typemap", lambda: None)
    assert U.netconvert_args(cfg, tmp_path, logs.append) == ([], None)
    assert "not found" in logs[-1]

    def broken(*a, **k):
        raise ValueError("bad file")
    monkeypatch.setattr(U, "write_typemap", broken)
    assert U.netconvert_args(cfg, tmp_path, logs.append) == ([], None)
    assert "not used" in logs[-1]


def test_base_typemap_from_sumo_home(tmp_path, monkeypatch):
    d = tmp_path / "data" / "typemap"
    d.mkdir(parents=True)
    (d / "osmNetconvert.typ.xml").write_text(BASE)
    monkeypatch.setenv("SUMO_HOME", str(tmp_path))
    assert U.base_typemap() == d / "osmNetconvert.typ.xml"


def test_plan_leaves_maps_built_with_urban_speeds_alone(tmp_path, monkeypatch):
    (tmp_path / "geo.net.xml").write_text("<net/>")
    (tmp_path / "map.net.xml").write_text("<net/>")
    (tmp_path / "meta.json").write_text(json.dumps({"bbox": [0, 0, 0.01, 0.01],
                                                    "speed_defaults": {"country": "US", "file": "urban.typ.xml"}}))

    def no_signs(*a, **k):
        raise AssertionError("a map built with urban speeds needs no variable speed signs")
    monkeypatch.setattr(S, "implicit_limits", no_signs)
    monkeypatch.setattr(S, "load", lambda *a, **k: None)
    logs = []
    cfg = {"traffic": {"speed_factor": "auto", "speed_limits": "auto"}}
    assert S.plan(cfg, tmp_path, tmp_path / "w", [0, 0, 0.01, 0.01], log=logs.append) == ("normc(1,0.1,0.8,1.2)", None)
    assert any("the map's own" in m for m in logs)
    assert not (tmp_path / "w" / "vss.add.xml").exists()
