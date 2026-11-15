"""Regression tests for bugs in the data and map-building code (no network, no SUMO binary, no CARLA)."""
import json
from datetime import datetime, timedelta, timezone

import pytest

from coupledcarla import realworld as rw


def test_sources_keep_nyc_buildings_and_name_trees_and_gbfs():
    # "nyc" was in SOURCES twice: the tree census replaced the building footprints, so a New York map's meta.json
    # credited its building heights to the tree census; GBFS (shared bikes) had no entry and was dropped
    assert "Building Footprints" in rw.SOURCES["nyc"][0]
    assert "Tree Census" in rw.SOURCES["nyc_trees"][0]
    assert "gbfs" in rw.SOURCES
    assert [m[0] for m in rw.MUNICIPAL] == ["eindhoven", "nyc_trees"]


def test_municipal_cache_with_the_old_nyc_name(tmp_path):
    f = tmp_path / "municipal.json"
    f.write_text(json.dumps({"elements": [], "sources": ["nyc"]}))
    assert rw.municipal_scenery([40.75, -73.99, 40.76, -73.97], f)["sources"] == ["nyc_trees"]


def test_weather_now_east_of_utc_after_local_midnight(monkeypatch):
    # Tokyo at 20:30 UTC is 05:30 the next day: the hour must be found, not the last hour of the UTC day
    class Clock(datetime):
        @classmethod
        def now(cls, tz=None):
            t = datetime(2026, 11, 14, 20, 30, tzinfo=timezone.utc)
            return t if tz else t.replace(tzinfo=None)

    asked = {}

    class Answer:
        def __init__(self, params):
            d0 = datetime.fromisoformat(params["start_date"])
            d1 = datetime.fromisoformat(params["end_date"])
            hours = int((d1 - d0).total_seconds() // 3600) + 24
            times = [(d0 + timedelta(hours=h)).strftime("%Y-%m-%dT%H:%M") for h in range(hours)]
            self.data = {"utc_offset_seconds": 9 * 3600, "timezone": "Asia/Tokyo",
                         "hourly": {"time": times, "precipitation": [0.0] * hours,
                                    "temperature_2m": list(range(hours))}}

        def raise_for_status(self):
            pass

        def json(self):
            return self.data

    def get(url, params=None, **kw):
        asked.update(params)
        return Answer(params)

    monkeypatch.setattr(rw, "datetime", Clock)
    monkeypatch.setattr(rw.requests, "get", get)
    rec, local, off = rw.weather_at(35.66, 139.70, "now")
    assert local == datetime(2026, 11, 15, 5, 30)
    assert rec["time"] == "2026-11-15T05:00"


def test_parking_median_side_follows_the_traffic_side():
    # left-hand maps skipped every kerb of a two-way street as a "median" (London 926 kerb stalls -> 1479 fixed)
    from coupledcarla import parking
    assert parking.median_side(False) == "left"
    assert parking.median_side(True) == "right"


def _map_cfg(tmp_path, **map_over):
    from coupledcarla import config
    cfg = config._merge(config.DEFAULTS, {"name": "m", "map": map_over})
    cfg["map"]["name"] = "m"
    cfg["_root"] = str(tmp_path)
    out = tmp_path / "maps" / "m"
    (out / "real").mkdir(parents=True)
    return cfg, out


def test_changed_bbox_drops_the_old_box_caches(tmp_path):
    # map.osm and real/*.json are cached by name: after a change of map.bbox, --force built the old box's roads
    # (Noumea moved 1.1 km north: geo.net.xml still covered the old box) under the new box's meta.json
    from coupledcarla import mapbuild
    cfg, out = _map_cfg(tmp_path)
    old, new = [-22.28209, 166.4512, -22.26951, 166.4648], [-22.27209, 166.4512, -22.25951, 166.4648]
    (out / "meta.json").write_text(json.dumps({"bbox": old}))
    (out / "map.osm").write_text("<osm/>")
    (out / "real" / "dem.npz").write_text("x")
    mapbuild.forget_other_box(out, old, log=lambda m: None)
    assert (out / "map.osm").exists() and (out / "real" / "dem.npz").exists()      # the same box: kept
    mapbuild.forget_other_box(out, new, log=lambda m: None)
    assert not (out / "map.osm").exists() and not (out / "real" / "dem.npz").exists()
    assert json.loads((out / "real" / "bbox.json").read_text())["bbox"] == new


def test_transit_rebuild_of_an_opendrive_map_uses_its_built_box(tmp_path, monkeypatch):
    # map.opendrive maps (the Eindhoven twin) have no map.bbox: --transit passed None to transit.build
    from coupledcarla import mapbuild, transit
    cfg, out = _map_cfg(tmp_path, opendrive="twin.xodr", carla_level="EhvCentre")
    box = [51.436, 5.47, 51.45, 5.493]
    (out / "meta.json").write_text(json.dumps({"bbox": box}))
    got = {}
    monkeypatch.setattr(mapbuild, "sumo_net", lambda out: None)
    monkeypatch.setattr(transit, "build", lambda out, bbox: got.setdefault("bbox", bbox) and
                        {"lines": 0, "refs": 0, "stops": 0, "with_timetable": 0, "feeds": []})
    mapbuild.build(cfg, transit_only=True)
    assert got["bbox"] == box


def test_overture_down_falls_back_to_osm_buildings(tmp_path, monkeypatch):
    # an S3/network error in the DuckDB query raised out of scenery.build: no scenery at all (Noumea with the query
    # failing: the traceback; fixed: 2452 OSM footprints)
    import sys
    import types

    class Con:
        def execute(self, q):
            if "read_parquet" in q:
                raise OSError("IO Error: Could not establish connection")
            return self
    monkeypatch.setitem(sys.modules, "duckdb", types.SimpleNamespace(connect=lambda: Con()))
    f = tmp_path / "overture_buildings.json"
    assert rw.overture_buildings([-22.28, 166.45, -22.27, 166.46], f) is None
    assert not f.exists()          # not cached: the next build tries again


def test_city_table_names_plateau_heights():
    # Tokyo's heights come from PLATEAU, but the smoke-test table said "overture"
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
    import test_cities
    assert test_cities.height_source({"sources": {"canopy": 1, "osm": 1, "overture": 1, "plateau": 1}}) == "plateau"
    assert test_cities.height_source({"sources": {"osm": 1}}) == "osm"


def test_osm_extract_sent_with_the_project_user_agent(tmp_path, monkeypatch):
    from coupledcarla import mapbuild

    class R:
        content = b"<osm><way id='1'/></osm>"

        def raise_for_status(self):
            pass
    sent = []
    monkeypatch.setattr(mapbuild.requests, "post", lambda url, **kw: sent.append(kw["headers"]) or R())
    mapbuild.fetch_osm([0, 0, 1, 1], tmp_path / "map.osm")
    assert sent == [rw.UA]


def test_lane_queries_skip_types_the_map_lacks(tmp_path):
    # carla.Map.get_waypoint for a lane type the map has none of costs ~0.5 ms (100 x a hit): parking asked for
    # Bidirectional and Sidewalk lanes OSM maps lack (Eindhoven scenery 56 s -> 9 s), the placer for bike lanes on
    # maps without them (Cairo 17.8 s -> 5.4 s), with identical scenery.json and parked.json
    from types import SimpleNamespace
    from coupledcarla import scenery
    x = tmp_path / "map.xodr"
    x.write_text('<OpenDRIVE><road><lanes><laneSection><center><lane id="0" type="none" level="false"/></center>'
                 '<right><lane id="-1" type="driving" level="false"/></right></laneSection></lanes></road></OpenDRIVE>')
    assert scenery.lane_types(x) == {"none", "driving"}
    assert scenery.lane_types(tmp_path / "missing.xodr") is None
    carla = SimpleNamespace(LaneType=SimpleNamespace(Driving=1, Parking=2, Bidirectional=4, Sidewalk=8, Biking=16))
    assert scenery.Placer(None, None, None, carla, lanes={"driving"}).masks == (15,)
    assert scenery.Placer(None, None, None, carla, lanes={"driving", "biking"}).masks == (15, 16)
    assert scenery.Placer(None, None, None, carla).masks == (15, 16)
