"""CARLA towns (towns.py): names, defaults under the experiment's own settings, the OpenDRIVE source, environment
objects as buildings and trees, CARLA x/y points, and the hooks in mapbuild, validate and make_videos (no CARLA,
no SUMO)."""
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml

from coupledcarla import config, mapbuild, towns, validate

ROOT = Path(__file__).resolve().parent.parent


def load(tmp_path, doc):
    p = tmp_path / "exp.yaml"
    p.write_text(yaml.safe_dump(doc, sort_keys=False))
    return config.load(p)


def test_names():
    assert towns.is_town("Town10HD_Opt") and towns.is_town("Town01") and towns.is_town("Mine_01")
    assert not towns.is_town("EhvCentre") and not towns.is_town("") and not towns.is_town(None)
    assert towns.base_name("Town10HD_Opt") == "Town10HD" and towns.base_name("Town03") == "Town03"
    assert towns.info("Town03_Opt")[1] > towns.info("Town07")[1]          # busier than the village
    assert towns.info("TownX")[1] > 0                                      # an unknown town: a default


def test_no_town_no_overrides():
    assert towns.overrides({"map": {"bbox": [0, 0, 1, 1]}}) == {}
    with pytest.raises(ValueError, match="not a CARLA town"):
        towns.overrides({"map": {"carla_town": "Eindhoven"}})


def test_town_defaults_and_the_file_wins(tmp_path):
    cfg = load(tmp_path, {"name": "t", "map": {"carla_town": "Town10HD_Opt"}})
    m = cfg["map"]
    assert m["name"] == "Town10HD_Opt" and m["carla_level"] == "Town10HD_Opt"
    assert config.map_dir(cfg).name == "Town10HD_Opt"
    assert not m["elevation"] and not m["transit"] and not m["scenery"]["buildings"] and not m["scenery"]["trees"]
    assert m["scenery"]["parked"] and m["scenery"]["park_shoulders_m"] == [2.0, 3.0]
    assert cfg["simulation"]["traffic_lights"]["style"] == "none"
    tr = cfg["traffic"]
    assert tr["cars_per_hour"] == towns.info("Town10HD")[1] and tr["counts"] is False
    assert tr["cycling"]["routes"] is False and tr["walkers"]["places"] is False and tr["bus_min_distance_m"] == 300
    assert tr["walkers"]["ai"]["count"] == 0 and tr["walkers"]["behaviour"] == "auto"     # the rest stays default
    assert cfg["city"]["country"] == "USA"
    # the experiment's own settings win over the town's defaults
    cfg = load(tmp_path, {"name": "t", "map": {"carla_town": "Town03", "name": "mine", "scenery": {"parked": False}},
                          "traffic": {"cars_per_hour": 99, "walkers": {"places": "auto"}},
                          "simulation": {"traffic_lights": {"style": "head"}}, "city": {"country": "NLD"}})
    assert cfg["map"]["name"] == "mine" and cfg["map"]["carla_level"] == "Town03"
    assert cfg["map"]["scenery"]["parked"] is False and cfg["map"]["scenery"]["trees"] is False
    assert cfg["traffic"]["cars_per_hour"] == 99 and cfg["traffic"]["walkers"]["places"] == "auto"
    assert cfg["simulation"]["traffic_lights"]["style"] == "head" and cfg["city"]["country"] == "NLD"
    # other maps are untouched
    cfg = load(tmp_path, {"name": "plain", "map": {"bbox": [51.4, 5.4, 51.5, 5.5]}})
    assert cfg["map"]["carla_town"] is None and cfg["map"]["elevation"] is True and cfg["map"]["name"] == "plain"
    assert cfg["simulation"]["traffic_lights"]["style"] == "head"


def test_place():
    cfg = {"map": {"bbox": [0.0, 0.0, 2.0, 4.0], "place": None}}
    assert config.place(cfg) == (1.0, 2.0)
    cfg["map"]["place"] = [49.0, 8.4]
    assert config.place(cfg) == (49.0, 8.4)


def test_local_opendrive(tmp_path, monkeypatch):
    assert towns.local_opendrive("Town03", roots=[str(tmp_path)]) is None
    d = tmp_path / "Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive"        # a CARLA 1.0 source tree
    d.mkdir(parents=True)
    (d / "Town03_Opt.xodr").write_text("<OpenDRIVE/>")
    assert towns.local_opendrive("Town03_Opt", roots=[str(tmp_path)]) == d / "Town03_Opt.xodr"
    d0 = tmp_path / "CarlaUE4/Content/Carla/Maps/OpenDrive"                 # 0.9.x first
    d0.mkdir(parents=True)
    (d0 / "Town03_Opt.xodr").write_text("<OpenDRIVE/>")
    monkeypatch.setenv("CARLA_ROOT", str(tmp_path))
    assert towns.local_opendrive("Town03_Opt") == d0 / "Town03_Opt.xodr"
    out = tmp_path / "maps" / "Town03_Opt"
    cfg = {"map": {"carla_town": "Town03_Opt"}, "server": {"host": "127.0.0.1", "port": 1, "launch": False}}
    f = towns.opendrive_file(cfg, out, log=lambda m: None)
    assert f == out / "town.xodr" and f.read_text() == "<OpenDRIVE/>"
    (d0 / "Town03_Opt.xodr").write_text("<changed/>")
    assert towns.opendrive_file(cfg, out).read_text() == "<OpenDRIVE/>"    # cached


def test_no_opendrive_and_no_server(tmp_path, monkeypatch):
    monkeypatch.delenv("CARLA_ROOT", raising=False)
    monkeypatch.setattr(towns, "_port_open", lambda h, p: False)
    pytest.importorskip("carla")
    cfg = {"map": {"carla_town": "Town05_Opt"}, "server": {"host": "127.0.0.1", "port": 1, "launch": True}}
    with pytest.raises(RuntimeError, match="no OpenDRIVE for Town05_Opt"):
        towns.opendrive_file(cfg, tmp_path / "m")


def test_georeference():
    text = ('<header><geoReference><![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 '
            '+units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs ]]></geoReference></header>')
    assert towns.georeference(text).startswith("+proj=tmerc +lat_0=0")
    assert towns.geo_origin(text) == (0.0, 0.0)
    assert towns.geo_origin("<geoReference>+lat_0=4.9e+1 +lon_0=8.0e+0</geoReference>") == (49.0, 8.0)
    assert towns.georeference("<header/>") is None and towns.geo_origin("<header/>") is None


def _obj(x, y, z, ex, ey, ez, yaw=0.0):
    v = lambda a, b, c: NS(x=a, y=b, z=c)       # noqa: E731
    return NS(bounding_box=NS(location=v(x, y, z), extent=v(ex, ey, ez), rotation=NS(yaw=yaw)))


def test_footprints_and_trees():
    objs = [_obj(10, 20, 5, 4, 3, 5), _obj(0, 0, 0.5, 2, 2, 0.5),          # a 10 m house; a 1 m box: dropped
            _obj(50, 50, 30, 10, 10, 30, yaw=90.0), _obj(0, 0, 5, 200, 200, 5),     # a tower; a merged block
            _obj(5, 5, 5, 3, 3, 5)]                                         # on the road
    recs = towns.footprints(objs, on_road=lambda x, y: (x, y) == (5, 5))
    assert len(recs) == 2
    house, tower = recs
    assert house["h"] == 10.0 and house["z"] == 0.0 and house["k"] == "house" and house["s"] == "carla"
    xs, ys = [p[0] for p in house["p"]], [p[1] for p in house["p"]]
    assert (min(xs), max(xs), min(ys), max(ys)) == (6.0, 14.0, 17.0, 23.0)
    assert tower["k"] == "commercial" and tower["h"] == 60.0
    trees = towns.tree_tops([_obj(1, 2, 4, 2, 2, 4), _obj(0, 0, 0.5, 1, 1, 0.5), _obj(3, 3, 6, 20, 20, 6)])
    assert trees == [[1, 2, 8, 2], [3, 3, 12, 8.0]]                        # the bush is not a tree; radius capped


def test_resolve_xy():
    class Geo:
        def to_latlon(self, x, y):
            return y / 1000.0, x / 1000.0
    tc = {"participants": [{"role": "d", "spawn": {"x": 10.0, "y": -20.0, "heading": 90}},
                           {"role": "p", "spawn": {"lat": 1.0, "lon": 2.0}, "bot": {"path_xy": [[1, 2], [3, 4]]}},
                           {"role": "pilot", "bot": {"path_xy": [[1, 2, 30]]}}],
          "agents": [{"id": "av", "route_xy": [[0, 0], [100, 0]], "triggers": [{"at_xy": [50, 0], "speed_kmh": 0}]}],
          "pois": [{"name": "c", "x": 1, "y": 2}], "film": {"camera": {"kind": "tripod", "at": {"x": 5, "y": 6}}},
          "map": {"x": 1, "y": 2}}
    assert towns.resolve_xy(tc, Geo()) == 7
    assert tc["participants"][0]["spawn"] == {"heading": 90, "lat": -0.02, "lon": 0.01}
    assert tc["participants"][1]["spawn"] == {"lat": 1.0, "lon": 2.0}
    assert tc["participants"][1]["bot"]["path"] == [[0.002, 0.001], [0.004, 0.003]]
    assert tc["participants"][2]["bot"]["path"] == [[0.002, 0.001, 30]]       # a drone's height kept
    assert tc["agents"][0]["route"] == [[0.0, 0.0], [0.0, 0.1]] and "route_xy" not in tc["agents"][0]
    assert tc["agents"][0]["triggers"][0]["at"] == [0.0, 0.05]
    assert tc["pois"][0] == {"name": "c", "lat": 0.002, "lon": 0.001}
    assert tc["film"]["camera"]["at"] == {"lat": 0.006, "lon": 0.005}
    assert tc["map"] == {"x": 1, "y": 2}                                    # only the point sections


def test_mapbuild_takes_the_towns_opendrive(tmp_path, monkeypatch):
    cfg = load(tmp_path, {"name": "t", "map": {"carla_town": "Town02_Opt"}})
    cfg["_root"] = str(tmp_path)
    seen = {}
    monkeypatch.setattr(towns, "opendrive_file", lambda c, out, log=None: tmp_path / "town.xodr")
    monkeypatch.setattr(mapbuild, "build_from_opendrive", lambda c, out: seen.setdefault("map", c["map"]))
    mapbuild.build(cfg)
    assert seen["map"]["opendrive"] == str(tmp_path / "town.xodr") and seen["map"]["carla_level"] == "Town02_Opt"


def test_sumo_net_guesses_no_lights_for_a_town(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(mapbuild.subprocess, "check_call", lambda cmd: calls.append(cmd))
    (tmp_path / "geo.net.xml").write_text('<net version="1.20">')
    mapbuild.sumo_net(tmp_path, guess_tls=False)
    mapbuild.sumo_net(tmp_path)
    assert "--guess-tls" not in calls[0] and "--guess-tls" in calls[1]


def test_build_scenery_hook(tmp_path, monkeypatch):
    cfg = load(tmp_path, {"name": "t", "map": {"carla_town": "Town10HD_Opt"}})
    seen = {}
    monkeypatch.setattr(towns, "build_scenery", lambda c, out, meta, cmap, major, log: seen.setdefault("cfg", c))
    mapbuild.build_scenery(cfg, tmp_path, {"bbox": [0, 0, 1, 1]})
    assert seen["cfg"]["map"]["bbox"] == [0, 0, 1, 1]


def test_validate_an_unbuilt_town(tmp_path):
    doc = {"name": "t", "map": {"carla_town": "Town10HD_Opt", "name": "unit_test_missing_town"},
           "participants": [{"role": "driver", "input": "autopilot"}], "trials": [{"duration_s": 40}]}
    p = tmp_path / "exp.yaml"
    p.write_text(yaml.safe_dump(doc))
    raw, cfg = validate.load(p)
    rep, _ = validate.check(raw, cfg, roads=False)
    errors = [m for lv, w, m in rep.items if lv == "error"]
    assert not errors, errors
    assert any("Town10HD_Opt's OpenDRIVE" in m for lv, w, m in rep.items)


def test_town_experiments_and_clips_load():
    for f in sorted((ROOT / "experiments" / "towns").glob("*.yaml")):
        cfg = config.load(f)
        assert towns.is_town(cfg["map"]["carla_town"]), f
        assert cfg["map"]["name"] == cfg["map"]["carla_town"]
    spec = importlib.util.spec_from_file_location("make_videos_towns", ROOT / "tools" / "make_videos.py")
    mv = importlib.util.module_from_spec(spec)
    sys.modules["make_videos_towns"] = mv
    spec.loader.exec_module(mv)
    clips = [c for c in mv.load_clips() if c["name"].startswith("town")]
    assert 3 <= len(clips) <= 5
    for c in clips:
        exp = mv.derive_experiment(c, {"host": "127.0.0.1", "port": 2800, "launch": True})
        assert exp["map"].get("name") in (None, exp["map"]["carla_town"])     # the town's map folder, not a copy
        assert json.dumps(exp)


def test_missing_level_message():
    client = NS(get_available_maps=lambda: ["/Game/Carla/Maps/Town01", "/Game/Carla/Maps/Town10HD_Opt"])
    msg = towns.missing_level(client, "Town06_Opt", RuntimeError("Map 'Town06_Opt' not found"))
    assert "Town01, Town10HD_Opt" in msg and "AdditionalMaps" in msg
    assert "only the _Opt towns" in towns.missing_level(client, "Town03", "not found")


@pytest.mark.parametrize("level,stalls,level_cars,want,ours,unloaded", [
    ("Town10HD_Opt", True, 29, "auto", True, True),      # ours, and the layer of the level's cars unloaded
    ("Town10HD", True, 29, "auto", False, False),        # a full level's cars cannot be unloaded: keep them
    ("Town10HD", True, 0, "auto", True, False),
    ("Town01_Opt", False, 9, "auto", False, False),      # no stalls (0.3 m shoulders): the level's cars
    ("Town03_Opt", True, 43, True, False, False),        # town_layer true: the level's cars only
    ("Town03", True, 43, False, True, False),            # false: ours
])
def test_own_parked(tmp_path, monkeypatch, level, stalls, level_cars, want, ours, unloaded):
    monkeypatch.setitem(sys.modules, "carla", NS(MapLayer=NS(ParkedVehicles=16)))
    (tmp_path / "parked.json").write_text(json.dumps({"slots": [[0] * 8] if stalls else []}))
    (tmp_path / "town_objects.json").write_text(json.dumps({"parked_layer": level_cars}))
    calls = []
    world = NS(unload_map_layer=lambda layer: calls.append(layer))
    tc = {"map": {"carla_level": level}, "simulation": {"parked": {"town_layer": want}}}
    assert towns.own_parked(world, tc, tmp_path, log=lambda m: None) is ours
    assert bool(calls) is unloaded


def test_snap_kerbs(monkeypatch):
    class Loc:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z
    monkeypatch.setitem(sys.modules, "carla", NS(Location=Loc, LaneType=NS(Sidewalk=4)))

    class Map:      # a pavement along CARLA y = 10 (geo y = -10) for x < 50, none beyond
        def get_waypoint(self, loc, project_to_road=True, lane_type=None):
            return NS(transform=NS(location=Loc(loc.x, 10.0))) if loc.x < 50 else None
    seg = NS(sides=[[(0.0, -7.0), (60.0, -7.0)], [(0.0, -30.0), (10.0, -5.0)]])
    assert towns.snap_kerbs(Map(), [seg], log=lambda m: None) == 2
    assert seg.sides[0] == [(0.0, -10.0), (60.0, -7.0)]          # moved 3 m; nothing beyond x 50
    assert seg.sides[1] == [(0.0, -30.0), (10.0, -10.0)]         # 20 m away: stays; 5 m: moved
