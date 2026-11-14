"""Parked cars: OSM parking tags, the default by road class, occupancy by hour, the car chooser, parking areas and the
trial-time filter (no CARLA, no network)."""
import json
from types import SimpleNamespace

from coupledcarla import parking


def test_side_tags_2022_scheme():
    t = {"parking:right": "lane", "parking:right:orientation": "perpendicular", "parking:left": "no"}
    assert parking.side_tags(t, "right") == {"orientation": "perpendicular", "position": "lane"}
    assert parking.side_tags(t, "left") is False
    assert parking.side_tags({"parking:both": "half_on_kerb"}, "left") == \
        {"orientation": "parallel", "position": "half_on_kerb"}
    assert parking.side_tags({"parking:both": "street_side", "parking:both:orientation": "diagonal"}, "right") == \
        {"orientation": "diagonal", "position": "street_side"}
    assert parking.side_tags({"parking:both": "lane", "parking:both:restriction": "no_stopping"}, "right") is False
    assert parking.side_tags({"highway": "residential"}, "right") is None
    assert parking.side_tags({"parking:right": "separate"}, "right")["orientation"] == "parallel"


def test_side_tags_old_scheme():
    t = {"parking:lane:both": "parallel", "parking:lane:both:parallel": "on_street"}
    assert parking.side_tags(t, "left") == {"orientation": "parallel", "position": "on_street"}
    assert parking.side_tags({"parking:lane:right": "no_parking"}, "right") is False
    assert parking.side_tags({"parking:lane:left": "perpendicular"}, "left")["orientation"] == "perpendicular"
    assert parking.side_tags({"parking:lane:both": "parallel", "parking:condition:both": "no_stopping"},
                             "right") is False
    assert parking.side_tags({"parking:lane:right": "parallel"}, "left") is None     # only the right side tagged


def test_default_share_is_conservative():
    assert parking.region([51.43, 5.47, 51.45, 5.49]) == "europe"
    assert parking.region([35.65, 139.69, 35.67, 139.71]) == "japan"
    assert parking.region([40.75, -74.0, 40.76, -73.98]) == "americas_oceania"
    assert parking.region([-36.86, 174.76, -36.84, 174.78]) == "americas_oceania"
    assert parking.region([30.04, 31.23, 30.05, 31.24]) == "other"
    assert parking.default_share("residential", "europe", 1) == 0.8
    assert parking.default_share("primary", "europe", 1) == 0.0          # never untagged on primary roads
    assert parking.default_share("trunk", "other", 1) == 0.0
    assert parking.default_share("residential", "europe", 2) == 0.0      # two lanes in one direction
    assert parking.default_share("residential", "japan", 1) == 0.0       # the Garage Act
    assert parking.default_share("residential", "europe", 1, bgt_covered=True) == \
        0.8 * parking.BGT_WITHOUT_STRIP


def test_occupancy_by_zone_and_hour():
    for day in ("weekday", "saturday", "sunday"):
        for z in parking.ZONES:
            assert len(parking.PRESENCE[z][day]) == 24
            assert all(0.0 <= parking.occupancy(z, day, h) <= 1.0 for h in range(24))
    # residential streets are full at night and half empty in the working day; centres the other way round
    assert parking.occupancy("residential", "weekday", 3) > parking.occupancy("residential", "weekday", 11) + 0.3
    assert parking.occupancy("centre", "weekday", 14) > parking.occupancy("centre", "weekday", 3) + 0.5
    assert parking.occupancy("residential", "sunday", 11) > parking.occupancy("residential", "weekday", 11)


def _cars():
    return {"H1": ("hatchback", "grey", ["vehicle.audi.a2"]), "H2": ("hatchback", "red", ["vehicle.citroen.c3"]),
            "S1": ("suv", "white", []), "T1": ("taxi", "taxi_yellow", ["vehicle.ford.crown"])}


def test_chooser_follows_register_bodies_and_colours():
    reg = {"body": {"hatchback": 80, "suv": 20}, "colors": {"grey": 90, "red": 10}}
    ch = parking.Chooser(_cars(), reg)
    picks = [ch(((i * 7919) % 1000) / 1000.0) for i in range(1000)]
    # one SUV mesh: it keeps a third of the SUV share, the rest goes on to the next body types (here hatchbacks)
    assert abs(picks.count("S1") / 1000 - 0.2 / 3) < 0.02
    assert picks.count("T1") == 0                               # no taxis in a register without them
    assert picks.count("H1") > 4 * picks.count("H2")            # grey cars are common in this register
    # no register: traffic.car_types blueprints mapped to body types
    ch = parking.Chooser(_cars(), None, {"vehicle.audi.a2": 1, "vehicle.nissan.patrol_2021": 1})
    picks = [ch(i / 100.0) for i in range(100)]
    assert set(picks) <= {"H1", "H2", "S1"} and "S1" in picks
    # traffic.car_types with a Ford Crown but no register with cabs: no yellow cab parks
    ch = parking.Chooser(_cars(), None, {"vehicle.ford.crown": 1})
    assert "T1" not in {ch(i / 50.0) for i in range(50)}
    # a body type without a mesh falls back (estate -> mpv -> hatchback)
    ch = parking.Chooser(_cars(), {"body": {"estate": 1}, "colors": {"grey": 1}})
    assert {ch(i / 10.0) for i in range(10)} <= {"H1", "H2"}


def test_no_model_dominates():
    cars = {f"M{i}": ("hatchback", "grey", []) for i in range(8)}
    cars["BIG"] = ("mpv", "blue", [])
    ch = parking.Chooser(cars, {"body": {"mpv": 90, "hatchback": 10}, "colors": {"blue": 1, "grey": 1}})
    tot = sum(ch.weights.values())
    assert max(ch.weights.values()) <= parking.MAX_SHARE * tot * 1.01


def test_car_table_matches_the_measured_meshes():
    data = json.loads((parking.DATA / "parked_cars.json").read_text())
    names = {p.rsplit(".", 1)[-1] for p in data["0.9.16"]}
    assert names <= set(parking.CARS_09) and len(names) >= 15
    for path, m in data["0.9.16"].items():
        assert all(e > 0 for e in m["ext"]) and 1.6 < 2 * m["ext"][0] < 6.5 and 1.4 < 2 * m["ext"][1] < 2.6, path
    cat = SimpleNamespace(version="0.9.16", by_name={})
    cars = parking.car_meshes(cat)
    assert set(cars) == names and set(cat.by_name) == names


class FakeGeo:
    """lat/lon are metres here: CARLA x = lon, y = lat."""

    def to_carla(self, lat, lon):
        return lon, lat


def test_areas_and_depth():
    osm = {"elements": [
        {"type": "way", "tags": {"amenity": "parking", "parking": "street_side"},
         "geometry": [{"lat": 0, "lon": 0}, {"lat": 0, "lon": 30}, {"lat": 2.2, "lon": 30}, {"lat": 2.2, "lon": 0},
                      {"lat": 0, "lon": 0}]},
        {"type": "way", "tags": {"amenity": "parking"},       # a surface car park
         "geometry": [{"lat": 10, "lon": 0}, {"lat": 10, "lon": 40}, {"lat": 30, "lon": 40}, {"lat": 30, "lon": 0},
                      {"lat": 10, "lon": 0}]},
        {"type": "way", "tags": {"amenity": "parking", "parking": "multi-storey"},
         "geometry": [{"lat": 50, "lon": 0}, {"lat": 50, "lon": 40}, {"lat": 90, "lon": 40}, {"lat": 50, "lon": 0}]},
    ]}
    areas = parking._areas(osm, None, FakeGeo())
    assert [(k, s) for _, k, s in areas] == [("street", "osm"), ("lot", "osm")]
    strip = areas[0][0]
    assert abs(parking._depth(strip, 15.0, 1.0, 0.0, 1.0) - 2.2) < 1e-6     # parallel parking strip
    assert abs(parking._depth(areas[1][0], 20.0, 20.0, 0.0, 1.0) - 16.0) < 1e-6


def test_trial_data_filters_by_occupancy(tmp_path):
    slots = [[0, float(i), 0.0, 0.0, 90.0, 0, i / 100.0, 3] for i in range(100)] + \
            [[0, float(i), 50.0, 0.0, 90.0, 1, i / 100.0, 3] for i in range(100)]
    (tmp_path / "parked.json").write_text(json.dumps({
        "version": 1, "catalog": "0.9.16", "cell_m": 100.0, "meshes": ["/Game/Car.Car"], "zones": parking.ZONES,
        "slots": slots}))
    tc = {"map": {"bbox": [51.43, 5.47, 51.45, 5.49]}, "simulation": {"time": "2026-11-16 03:00"}}
    night = parking.trial_data(tmp_path, tc, log=lambda m: None)
    res = sum(1 for it in night["items"] if it[2] == 0.0)
    cen = sum(1 for it in night["items"] if it[2] == 50.0)
    assert res == round(100 * parking.occupancy("residential", "weekday", 3)) or \
        abs(res - 100 * parking.occupancy("residential", "weekday", 3)) <= 1
    assert cen < res and all(it[6] == "parked" and it[5] == 1.0 for it in night["items"])
    tc["simulation"]["time"] = "2026-11-16 14:00"
    day = parking.trial_data(tmp_path, tc, log=lambda m: None)
    assert sum(1 for it in day["items"] if it[2] == 50.0) > cen
    # the stalls taken at a lower occupancy are a subset of those at a higher one (same u)
    assert {tuple(it[1:3]) for it in night["items"] if it[2] == 50.0} <= \
        {tuple(it[1:3]) for it in day["items"] if it[2] == 50.0}
    tc["simulation"]["parked"] = {"occupancy": 0.5}
    assert len(parking.trial_data(tmp_path, tc, log=lambda m: None)["items"]) == 100
    tc["simulation"]["parked"] = {"enabled": False}
    assert parking.trial_data(tmp_path, tc, log=lambda m: None) is None
    assert parking.trial_data(tmp_path / "nowhere", {"map": tc["map"]}, log=lambda m: None) is None
