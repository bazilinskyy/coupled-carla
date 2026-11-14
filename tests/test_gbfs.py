"""Shared bikes and scooters (GBFS): the catalogue, feed parsing (GBFS 2 and 3, TfL, YouBike), the layout of
docked and free-floating vehicles, the cached snapshot and the dock weights for cyclists (no network, no CARLA)."""
import json
import math
from datetime import datetime, timezone

import pytest

from coupledcarla import gbfs as G
from coupledcarla import scenery

BOX = [40.7486, -73.9923, 40.7612, -73.9757]


def test_place_names_and_candidates():
    assert G.place_names({"name": "new_york_midtown", "city": {"name": "New York (Midtown)"}}) == {"new york", "new"}
    assert G.place_names({"name": "eindhoven_centre"}) == {"eindhoven"}
    assert "portland" in G.place_names({"name": "portland_or", "city": {"name": "Portland, Oregon"}})
    rows = [
        {"Name": "Citi Bike", "Location": "New York, NY", "Auto-Discovery URL": "https://a/gbfs.json",
         "Authentication Type": ""},
        {"Name": "Bird Portland", "Location": "Portland, OR", "Auto-Discovery URL": "https://b/gbfs.json",
         "Authentication Type": ""},
        {"Name": "Keyed", "Location": "New York, NY", "Auto-Discovery URL": "https://c/gbfs.json",
         "Authentication Type": "api_key"},
        {"Name": "HELLO CYCLING", "Location": "Japan", "Auto-Discovery URL": "https://d/gbfs.json",
         "Authentication Type": ""},
    ]
    assert [r["Name"] for r in G.candidates(rows, {"new york"})] == ["Citi Bike"]
    assert [r["Name"] for r in G.candidates(rows, {"tokyo"}, "Japan")] == ["HELLO CYCLING"]
    twice = rows + [dict(rows[0], Location="United States")]
    assert [r["Name"] for r in G.candidates(twice, {"new york"}, "United States")] == ["Citi Bike"]


def test_feed_urls_gbfs2_and_3():
    v2 = {"data": {"fr": {"feeds": [{"name": "station_status", "url": "fr"}]},
                   "en": {"feeds": [{"name": "station_status", "url": "en"}]}}}
    v3 = {"version": "3.0", "data": {"feeds": [{"name": "vehicle_status", "url": "u"}]}}
    assert G.feed_urls(v2) == {"station_status": "en"}
    assert G.feed_urls(v3) == {"vehicle_status": "u"}
    assert G.feed_urls({}) == {}
    assert G.updated({"last_updated": 1794684779}).year == 2026
    assert G.updated({"last_updated": "2026-11-14T19:01:13Z"}) == datetime(2026, 11, 14, 19, 1, 13,
                                                                            tzinfo=timezone.utc)
    assert G.updated({}) is None


def _feeds_v2():
    return {
        "system_information": {"data": {"license_url": "https://example.org/data-licence"}},
        "vehicle_types": {"data": {"vehicle_types": [{"vehicle_type_id": "1", "form_factor": "bicycle"},
                                                     {"vehicle_type_id": "2", "form_factor": "scooter_standing"},
                                                     {"vehicle_type_id": "3", "form_factor": "car"}]}},
        "station_information": {"data": {"stations": [
            {"station_id": "a", "name": "W 41 St & 8 Ave", "lat": 40.756, "lon": -73.99, "capacity": 30},
            {"station_id": "b", "name": "far away", "lat": 40.70, "lon": -73.90, "capacity": 20},
            {"station_id": "c", "name": "closed", "lat": 40.755, "lon": -73.98, "capacity": 20}]}},
        "station_status": {"last_updated": 1794684779, "data": {"stations": [
            {"station_id": "a", "num_bikes_available": 7, "is_installed": 1, "is_renting": 1,
             "vehicle_types_available": [{"vehicle_type_id": "1", "count": 7}]},
            {"station_id": "c", "num_bikes_available": 3, "is_installed": 1, "is_renting": 0}]}},
        "free_bike_status": {"last_updated": 1794684779, "data": {"bikes": [
            {"bike_id": "x", "lat": 40.752, "lon": -73.985, "vehicle_type_id": "2", "is_reserved": 0, "is_disabled": 0},
            {"bike_id": "y", "lat": 40.753, "lon": -73.986, "vehicle_type_id": "1", "is_reserved": 1, "is_disabled": 0},
            {"bike_id": "z", "lat": 40.754, "lon": -73.987, "vehicle_type_id": "1", "station_id": "a"},
            {"bike_id": "w", "lat": 40.754, "lon": -73.987, "vehicle_type_id": "3"},
            {"bike_id": "v", "lat": 40.80, "lon": -73.95, "vehicle_type_id": "1"}]}},
    }


def test_parse_system_gbfs2():
    res, stamp = G.parse_system(_feeds_v2(), BOX)
    assert [s["name"] for s in res["stations"]] == ["W 41 St & 8 Ave"]
    st = res["stations"][0]
    assert st["available"] == 7 and st["capacity"] == 30 and st["types"] == {"bicycle": 7}
    # reserved, docked, car-sharing and far vehicles left out
    assert res["vehicles"] == [{"lat": 40.752, "lon": -73.985, "form_factor": "scooter_standing"}]
    assert res["licence"] == "https://example.org/data-licence"
    assert stamp.year == 2026


def test_parse_system_gbfs3_without_vehicle_types():
    feeds = {"station_information": {"data": {"stations": [
                {"station_id": "1", "name": [{"text": "Bellecour", "language": "fr"}], "lat": 45.757, "lon": 4.832,
                 "capacity": 40}]}},
             "station_status": {"last_updated": "2026-11-14T19:00:00Z", "data": {"stations": [
                {"station_id": "1", "num_vehicles_available": 12, "is_installed": True, "is_renting": True}]}},
             "vehicle_status": {"data": {"vehicles": [{"vehicle_id": "q", "lat": 45.756, "lon": 4.831}]}}}
    res, stamp = G.parse_system(feeds, [45.7515, 4.8230, 45.7641, 4.8410])
    assert res["stations"][0]["name"] == "Bellecour" and res["stations"][0]["types"] == {"bicycle": 12}
    assert res["vehicles"] == [{"lat": 45.756, "lon": 4.831, "form_factor": "bicycle"}]   # no types: bicycles
    assert "no licence" in res["licence"]


def test_own_formats():
    tfl = [{"commonName": "Strand", "lat": 51.51, "lon": -0.12, "additionalProperties": [
                {"key": "NbBikes", "value": "9"}, {"key": "NbEBikes", "value": "2"}, {"key": "NbStandardBikes", "value": "7"},
                {"key": "NbDocks", "value": "20"}, {"key": "Installed", "value": "true"}, {"key": "Locked", "value": "false"}]},
           {"commonName": "Locked", "lat": 51.51, "lon": -0.12, "additionalProperties": [
                {"key": "Locked", "value": "true"}]}]
    st = G.parse_tfl(tfl, [51.5017, -0.1382, 51.5143, -0.118])
    assert st == [{"lat": 51.51, "lon": -0.12, "name": "Strand", "capacity": 20, "available": 9,
                   "types": {"bicycle": 9}}]
    unknown = [{"commonName": "Frozen", "lat": 51.51, "lon": -0.12, "additionalProperties": [
        {"key": "NbBikes", "value": "0"}, {"key": "NbEmptyDocks", "value": "0"}, {"key": "NbDocks", "value": "19"}]}]
    st = G.parse_tfl(unknown, [51.5017, -0.1382, 51.5143, -0.118])
    assert st[0]["available"] == 9 and st[0]["estimated"]           # occupancy unknown: half full
    yb = [{"sna": "YouBike2.0_台北車站", "snaen": "Taipei Main Station", "latitude": 25.0478, "longitude": 121.517,
           "Quantity": 40, "available_rent_bikes": 11, "act": "1"},
          {"sna": "off", "latitude": 25.0478, "longitude": 121.517, "act": "0", "Quantity": 5}]
    st = G.parse_youbike(yb, [25.0415, 121.5100, 25.0541, 121.5239])
    assert len(st) == 1 and st[0]["available"] == 11 and st[0]["name"] == "Taipei Main Station"


def test_meshes_and_catalogue():
    cat = scenery.Catalog("0.9.16")
    assert "SM_RoadBike" not in cat
    G.ensure_meshes(cat)
    assert G.mesh_for(cat, "bicycle") == ("SM_RoadBike", 1.0)          # no SM_Ebike on 0.9.16
    assert G.mesh_for(cat, "scooter_standing") == ("SM_Vespa", 0.6)
    assert G.mesh_for(cat, "moped") == ("SM_Vespa", 1.0)
    assert G.mesh_for(cat, "car") is None
    L, W, H = cat.size("SM_RoadBike")
    assert 1.5 < L < 1.8 and 0.3 < W < 0.5 and 0.9 < H < 1.1          # a bicycle
    x, y, z, yaw = cat.place("SM_RoadBike", 10.0, 5.0, 2.0, 0.0, 1.0)
    assert z == pytest.approx(2.0 - 0.02, abs=1e-6)                   # wheels on the ground


def _street_layout(snap, free=lambda x, y: abs(y) > 4.0, max_per_station=10):
    """A street along x (lanes within 4 m of y = 0); lat/lon map to metres around (0, 0)."""
    cat = scenery.Catalog("0.9.16")
    G.ensure_meshes(cat)

    def to_xy(lat, lon):
        return lon * 1e5, lat * 1e5

    def road_at(x, y):
        return 0.0, (x, max(-3.0, min(3.0, y)))
    return G.layout(snap, cat, to_xy, road_at, free, max_per_station)


def test_layout_docks_side_by_side_square_to_the_street():
    snap = {"systems": [{"stations": [{"lat": 6e-5, "lon": 0.0, "available": 4, "types": {"bicycle": 4}}],
                         "vehicles": []}]}
    items = _street_layout(snap)
    assert len(items) == 4 and all(i[1] == "SM_RoadBike" for i in items)
    xs = sorted(i[2] for i in items)
    assert [round(b - a, 2) for a, b in zip(xs, xs[1:])] == [G.SPACING_M] * 3
    assert all(abs(i[3] - 6.0) < 1e-6 for i in items)
    assert all(abs(((i[4] - 90.0) + 180) % 360 - 180) <= 4.0 for i in items)   # square to the street
    capped = _street_layout({"systems": [{"stations": [{"lat": 6e-5, "lon": 0.0, "available": 25}]}]},
                            max_per_station=10)
    assert len(capped) == 10


def test_layout_moves_vehicles_off_the_lanes():
    snap = {"systems": [{"stations": [], "vehicles": [
        {"lat": 3e-5, "lon": 0.0, "form_factor": "scooter_standing"},     # on the lane: moved 1-3 m away
        {"lat": 8e-5, "lon": 5e-5, "form_factor": "bicycle"},
        {"lat": 1e-5, "lon": 0.0, "form_factor": "car"}]}]}
    items = _street_layout(snap)
    assert len(items) == 2
    sc = next(i for i in items if i[0] == "scooter_standing")
    assert 4.0 < sc[3] <= 6.0 and sc[5] == 0.6
    yaw = sc[4] % 180.0
    assert min(yaw, 180 - yaw) <= 15.0                                 # parallel to the street
    blocked = _street_layout(snap, free=lambda x, y: False)
    assert blocked == []
    # the same snapshot gives the same placements (a scenery rebuild changes nothing)
    assert _street_layout(snap) == items


def test_snapshot_cache_and_offline(tmp_path, monkeypatch):
    calls = []

    def fetch(cfg, bbox, log=None, now=None):
        calls.append(1)
        return {"fetched": datetime.now().isoformat(timespec="seconds"), "candidates": ["Citi Bike"], "notes": [],
                "systems": [{"id": "citi", "name": "Citi Bike", "url": "u", "licence": "l",
                             "stations": [{"lat": 40.755, "lon": -73.985, "available": 3, "capacity": 20,
                                           "types": {"bicycle": 3}}], "vehicles": []}]}
    monkeypatch.setattr(G, "fetch", fetch)
    s = G.snapshot({}, tmp_path, BOX, log=lambda m: None)
    assert s["systems"][0]["name"] == "Citi Bike" and (tmp_path / "real" / "gbfs.json").exists()
    G.snapshot({}, tmp_path, BOX, log=lambda m: None)
    assert len(calls) == 1

    def offline(*a, **k):
        raise OSError("offline")
    monkeypatch.setattr(G, "fetch", offline)
    assert G.snapshot({}, tmp_path, BOX, max_age_s=0, log=lambda m: None)["systems"][0]["name"] == "Citi Bike"
    assert G.snapshot({}, tmp_path / "x", BOX, log=lambda m: None) is None


def test_fetch_drops_stale_systems(monkeypatch):
    rows = [{"Name": "Old Mopeds", "Location": "Eindhoven", "Auto-Discovery URL": "https://o/gbfs.json",
             "Authentication Type": "", "System ID": "old"},
            {"Name": "Fresh Bikes", "Location": "Eindhoven", "Auto-Discovery URL": "https://f/gbfs.json",
             "Authentication Type": "", "System ID": "fresh"}]
    monkeypatch.setattr(G, "systems_csv", lambda: rows)
    now = datetime(2026, 11, 14, 20, tzinfo=timezone.utc)

    def fetch_system(row, bbox, now=None):
        st = [{"lat": 51.44, "lon": 5.48, "available": 2, "capacity": 5, "types": {"bicycle": 2}}]
        stamp = datetime(2025, 5, 21, tzinfo=timezone.utc) if row["System ID"] == "old" else now
        return {"stations": st, "vehicles": [], "licence": "CC0"}, stamp
    monkeypatch.setattr(G, "fetch_system", fetch_system)
    snap = G.fetch({"name": "eindhoven_centre"}, [51.436, 5.47, 51.45, 5.493], now=now)
    assert [s["name"] for s in snap["systems"]] == ["Fresh Bikes"]
    assert any("stale" in n for n in snap["notes"])


class _E:
    def __init__(self, eid, bike=True):
        self.eid, self.bike = eid, bike

    def getID(self):
        return self.eid

    def allows(self, vclass):
        return self.bike


class _Net:
    """Edges at x = 0, 100, 200 ... m along a line; the dock at (lon, lat) = (x, 0)."""

    def __init__(self, n=5):
        self.edges = [_E(f"e{i}", bike=(i != 1)) for i in range(n)]

    def convertLonLat2XY(self, lon, lat):
        return lon, lat

    def getNeighboringEdges(self, x, y, r):
        return [(e, abs(i * 100.0 - x)) for i, e in enumerate(self.edges) if abs(i * 100.0 - x) <= r]


def test_docks_are_bike_stations_only():
    snap = {"systems": [{"stations": [
        {"lat": 1.0, "lon": 1.0, "capacity": 20, "types": {"bicycle": 3}},
        {"lat": 2.0, "lon": 2.0, "capacity": 0, "types": {}},                # a scooter system's parking area
        {"lat": 3.0, "lon": 3.0, "capacity": 15, "types": {}},               # an empty dock
        {"lat": 4.0, "lon": 4.0, "capacity": 0, "types": {"scooter": 2}}]}]}
    assert G.docks(snap) == [(1.0, 1.0, 20), (3.0, 3.0, 15)]


def test_car_sharing_systems_are_dropped():
    feeds = _feeds_v2()
    feeds["vehicle_types"]["data"]["vehicle_types"] = [{"vehicle_type_id": "3", "form_factor": "car"}]
    res, _ = G.parse_system(feeds, BOX)
    assert res["stations"] == [] and res["vehicles"] == []


def test_fetch_drops_a_system_listed_twice(monkeypatch):
    rows = [{"Name": "docomo", "Location": "Tokyo", "Auto-Discovery URL": "https://a/gbfs.json",
             "Authentication Type": "", "System ID": "a"},
            {"Name": "docomo", "Location": "Japan", "Auto-Discovery URL": "https://b/gbfs.json",
             "Authentication Type": "", "System ID": "b"}]
    monkeypatch.setattr(G, "systems_csv", lambda: rows)
    st = [{"lat": 35.658, "lon": 139.70, "available": 2, "capacity": 5, "types": {"bicycle": 2}}]
    monkeypatch.setattr(G, "fetch_system", lambda row, bbox, now=None: ({"stations": [dict(x) for x in st],
                                                                         "vehicles": [], "licence": "CC-BY-4.0"}, None))
    snap = G.fetch({"name": "tokyo_shibuya", "city": {"name": "Tokyo (Shibuya)", "country": "Japan"}},
                   [35.6517, 139.6939, 35.6643, 139.7093])
    assert len(snap["systems"]) == 1 and "same docks" in snap["notes"][0]


def test_car_sharing_stations_are_dropped():
    feeds = _feeds_v2()
    feeds["station_status"]["data"]["stations"][0]["vehicle_types_available"] = [{"vehicle_type_id": "3", "count": 4}]
    res, _ = G.parse_system(feeds, BOX)
    assert res["stations"] == []


def test_dock_weights():
    snap = {"systems": [{"stations": [{"lat": 0.0, "lon": 110.0, "capacity": 20},
                                      {"lat": 0.0, "lon": 290.0, "capacity": 10}]}]}
    near = G.dock_edges(_Net(), snap)
    assert near == {"e3": 1}                    # e1 is near the first dock but takes no bikes; e2 is 90 m away
    near = G.dock_edges(_Net(), snap, radius=100.0)
    assert near == {"e2": 2, "e3": 1}           # e2 lies 90 m from both docks
    src, dst = {"e0": 1.0, "e2": 3.0, "e3": 1.0}, {"e2": 1.0}
    G.weight_docks([src], [dst], near, 2.0)
    assert src == {"e0": 1.0, "e2": 9.0, "e3": 3.0} and dst == {"e2": 3.0}


def test_cycling_hook_fails_soft(tmp_path):
    cfg = {"traffic": {"cycling": {"dock_weight": 2.0}}}
    assert G.cycling_hook(cfg, _Net(), tmp_path, [{}], [{}], log=lambda m: None) == {}     # no meta.json
    cfg["traffic"]["cycling"]["dock_weight"] = 0
    assert G.cycling_hook(cfg, _Net(), tmp_path, [{}], [{}], log=lambda m: None) == {}
