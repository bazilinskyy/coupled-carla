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


def test_expired_caches_are_used_when_the_refresh_fails(tmp_path, monkeypatch):
    # fleet/counts/speeds caches older than their age limit were fetched again and, offline, the error threw the
    # old copy away: Eindhoven's trial fell back from the RDW fleet to the configured one
    import os
    import time
    import requests
    from coupledcarla import counts, fleet, speeds, transit

    def down(*a, **k):
        raise requests.ConnectionError("offline")
    old = time.time() - 60 * 86400
    reg = {"source": "RDW", "body": {"hatchback": 3, "estate": 2}, "colors": {"grey": 1}}
    for mod in (fleet, counts, speeds):
        monkeypatch.setattr(mod, "CACHE", tmp_path / mod.__name__)
        (tmp_path / mod.__name__).mkdir()
        f = tmp_path / mod.__name__ / "x.json"
        f.write_text(json.dumps(reg))
        os.utime(f, (old, old))
        assert mod._cached("x", down) == reg
        with pytest.raises(requests.ConnectionError):
            mod._cached("never_fetched", down)
    monkeypatch.setattr(fleet, "_rdw", down)
    (tmp_path / fleet.__name__ / "rdw_nl.json").write_text(json.dumps(reg))
    os.utime(tmp_path / fleet.__name__ / "rdw_nl.json", (old, old))
    assert fleet.registry([51.436, 5.47, 51.45, 5.493]) == reg
    # the GTFS catalogue: a week old and offline
    csv = tmp_path / "mobilitydatabase_feeds_v2.csv"
    csv.write_text("id,data_type,status,urls.latest,location.bounding_box.minimum_latitude,"
                   "location.bounding_box.maximum_latitude,location.bounding_box.minimum_longitude,"
                   "location.bounding_box.maximum_longitude\nmdb-1,gtfs,active,http://x/1.zip,45,46,4,5\n")
    os.utime(csv, (old, old))
    monkeypatch.setattr(transit.requests, "get", down)
    assert [r["id"] for r in transit.feeds_for([45.75, 4.82, 45.76, 4.84], tmp_path)] == ["mdb-1"]


def test_map_counts_older_than_their_age_are_kept_offline(tmp_path, monkeypatch):
    # real/counts.json and real/bike_counters.json past their age: a failed refetch dropped them (random demand)
    import os
    import time
    from coupledcarla import counts, cycling

    def down(*a, **k):
        raise OSError("offline")
    real = tmp_path / "real"
    real.mkdir()
    (real / "counts.json").write_text(json.dumps({"source": "nyc_dot", "fetched": "2026-01-01T00:00:00",
                                                  "counters": [{"id": 1}]}))
    monkeypatch.setattr(counts, "FETCHERS", [("nyc_dot", lambda b: True, down)])
    assert counts.load(tmp_path, [40.75, -73.99, 40.76, -73.97])["counters"] == [{"id": 1}]
    (real / "bike_counters.json").write_text(json.dumps({"sites": [{"id": "a"}]}))
    old = time.time() - 400 * 86400
    os.utime(real / "bike_counters.json", (old, old))
    monkeypatch.setattr(cycling, "FETCHERS", [("nyc", (40.0, -75.0, 41.0, -73.0), down)])
    assert cycling.counters([40.75, -73.99, 40.76, -73.97], real) == [{"id": "a"}]


def test_guessed_lamps_stand_at_the_kerb_on_both_traffic_sides():
    # a road along +x with two lanes right of its reference line (y = 0): lane -1 at y = 1.6, lane -2 at y = 4.8
    # (CARLA's y points right of +x). Right-hand traffic: the kerb is beyond lane -2; left-hand: beyond the
    # reference line, left of lane -1 (London's guessed lamps once stood along the centre line instead)
    from types import SimpleNamespace as NS
    from coupledcarla import scenery
    carla = NS(LaneType=NS(Driving=1, NONE=0))
    lanes = {}

    def lane(lid, y):
        tf = NS(location=NS(x=10.0, y=y), rotation=NS(yaw=0.0), get_right_vector=lambda: NS(x=0.0, y=1.0))
        return NS(lane_id=lid, lane_type=1, lane_width=3.2, transform=tf, road_id=1, section_id=0, s=10.0,
                  get_right_lane=lambda: lanes.get(lid - 1), get_left_lane=lambda: lanes.get(lid + 1))
    lanes.update({-1: lane(-1, 1.6), -2: lane(-2, 4.8)})
    assert scenery.kerb_lamp(lanes[-1], False, carla) is None
    edge, x, y, yaw = scenery.kerb_lamp(lanes[-2], False, carla)
    assert edge.lane_id == -2 and y == pytest.approx(7.4) and yaw % 360 == pytest.approx(270.0)   # arm to -y
    assert scenery.kerb_lamp(lanes[-2], True, carla) is None
    edge, x, y, yaw = scenery.kerb_lamp(lanes[-1], True, carla)
    assert edge.lane_id == -1 and y == pytest.approx(-1.0) and yaw % 360 == pytest.approx(90.0)    # arm to +y


def test_overpass_partial_answers_are_not_cached(tmp_path, monkeypatch):
    # Overpass answers a query that runs out of time or memory with HTTP 200, the elements found so far and a
    # "runtime error" remark: that was cached as the box's OSM extract or scenery data for good
    from coupledcarla import mapbuild

    class R:
        def __init__(self, body):
            self.content = body

        def raise_for_status(self):
            pass

        def json(self):
            return json.loads(self.content)
    answers = []
    monkeypatch.setattr(rw.time, "sleep", lambda s: None)
    monkeypatch.setattr(mapbuild.time, "sleep", lambda s: None)
    monkeypatch.setattr(rw.requests, "post", lambda url, **kw: answers.pop(0))
    answers += [R(b'{"elements": [{"type": "node"}], "remark": "runtime error: Query timed out in \\"query\\" at '
                  b'line 1 after 101 seconds."}'), R(b'{"elements": [{"type": "node"}, {"type": "way"}]}')]
    assert len(rw.overpass_json("q")["elements"]) == 2          # the second mirror's full answer
    answers += [R(b'<?xml version="1.0"?><osm><way id="1"/><remark> runtime error: Query run out of memory '
                  b'using about 2048 MB of RAM. </remark></osm>'), R(b'<osm><way id="1"/><way id="2"/></osm>')]
    mapbuild.fetch_osm([0, 0, 1, 1], tmp_path / "map.osm")
    assert b'id="2"' in (tmp_path / "map.osm").read_bytes()


def test_a_register_that_is_down_is_not_cached_as_none(tmp_path, monkeypatch):
    # Eindhoven's tree and lamp register failing once wrote municipal.json without them: every later build had
    # OSM's trees and guessed lamps only
    def down(bbox):
        raise OSError("503")
    monkeypatch.setattr(rw, "MUNICIPAL", [("eindhoven", (51.38, 5.38, 51.50, 5.57), down, "registers")])
    f = tmp_path / "municipal.json"
    assert rw.municipal_scenery([51.436, 5.47, 51.45, 5.493], f) == {"elements": [], "sources": []}
    assert not f.exists()
    monkeypatch.setattr(rw, "MUNICIPAL", [("eindhoven", (51.38, 5.38, 51.50, 5.57), lambda b: [], "registers")])
    rw.municipal_scenery([51.436, 5.47, 51.45, 5.493], f)
    assert f.exists()
