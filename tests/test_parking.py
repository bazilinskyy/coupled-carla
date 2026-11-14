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
    tc["map"]["scenery"] = {"parked": False}
    assert parking.trial_data(tmp_path, tc, log=lambda m: None) is None
    tc["map"]["scenery"] = {}
    tc["simulation"]["parked"] = {"enabled": False}
    assert parking.trial_data(tmp_path, tc, log=lambda m: None) is None
    assert parking.trial_data(tmp_path / "nowhere", {"map": tc["map"]}, log=lambda m: None) is None


def test_carla10_cars_measured_with_paint_and_glass():
    from coupledcarla import fleet
    data = json.loads((parking.DATA / "parked_cars.json").read_text())["1.0"]
    names = {p.rsplit(".", 1)[-1] for p in data}
    assert names <= set(parking.CARS_10) and len(names) >= 15
    for path, m in data.items():
        assert all(0 < e < 100 for e in m["ext"]) and 1.6 < 2 * m["ext"][0] < 6.5 and 1.4 < 2 * m["ext"][1] < 2.6
        assert m.get("paint") is None or m["paint"] in fleet.COLORS, path
        if "glass" in m:      # the window glass lies inside the body's box
            g, b = m["glass_box"], m
            assert all(g["off"][k] - g["ext"][k] >= b["off"][k] - b["ext"][k] - 0.1 for k in range(3)), path
    cat = SimpleNamespace(version="1.0", by_name={})
    cars = parking.car_meshes(cat)
    assert set(cars) == names
    assert cars["SM_LincolnMKZ_Parked"][1] == "black" and cars["SM_DodgeCharger_Parked"][1] == "red"
    assert cat.glass_of["SM_LincolnMKZ_Parked"] == "SM_LincolnMKZ_Glass_Parked"
    assert "SM_LincolnMKZ_Glass_Parked" in cat.by_name and "SM_AudiA2" not in cat.glass_of
    # the 1.0 chooser follows the register's colours with the measured paints
    ch = parking.Chooser(cars, {"body": {"hatchback": 1}, "colors": {"red": 9, "blue": 1}})
    reds = sum(ch.weights[m] for m in ch.weights if cars[m][1] == "red")
    assert reds > 0.5 * sum(ch.weights.values())


def test_paint_names_and_glass_pairs():
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location(
        "parked_catalog", Path(__file__).resolve().parent.parent / "tools" / "parked_catalog.py")
    pc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pc)
    assert [pc.paint_name(c) for c in ((0, 0, 0), (128, 128, 128), (188, 188, 188), (250, 250, 250))] == \
        ["black", "grey", "silver", "white"]
    assert [pc.paint_name(c) for c in ((182, 0, 0), (113, 30, 30), (0, 131, 239), (43, 101, 112), (71, 41, 0),
                                       (232, 192, 0), (159, 127, 0), (30, 90, 45))] == \
        ["red", "maroon", "blue", "blue", "brown", "yellow", "gold", "green"]
    assert pc.srgb((0.5052, 0.5052, 0.5052)) == (188, 188, 188)
    assert pc.glass_key("SM_LincolnMKZ_Glass_Parked") == pc.glass_key("SM_LincolnMKZ_Parked")
    assert pc.glass_key("SM_TeslaM3_parked_glass") == pc.glass_key("SM_TeslaM3_parked")
    assert pc.glass_key("SM_DodgeChargerCop_Parked_Glass") != pc.glass_key("SM_DodgeCharger_Parked")
    assert pc.glass_key("SM_MiniCooper_Old_Parked") != pc.glass_key("SM_MiniCooper_Glass_Parked")
    body = {"off": [0, 0, 0.8], "ext": [2.4, 1.1, 0.8]}
    assert pc.inside({"off": [0, 0, 1.0], "ext": [1.5, 0.9, 0.4]}, body)
    assert not pc.inside({"off": [0, 0, 2.0], "ext": [1.5, 0.9, 0.4]}, body)


def test_trial_data_takes_glass_with_its_car(tmp_path):
    slots = [[0, 1.0, 0.0, 0.0, 90.0, 0, 0.01, 6], [1, 1.0, 0.0, 0.0, 90.0, 0, 0.01, 6, 1],
             [0, 9.0, 0.0, 0.0, 90.0, 0, 0.99, 6], [1, 9.0, 0.0, 0.0, 90.0, 0, 0.99, 6, 1]]
    (tmp_path / "parked.json").write_text(json.dumps({
        "version": 1, "catalog": "1.0", "cell_m": 100.0, "meshes": ["/Game/Car.Car", "/Game/Glass.Glass"],
        "zones": parking.ZONES, "slots": slots}))
    logs = []
    tc = {"map": {"bbox": [51.43, 5.47, 51.45, 5.49], "carla_level": "EhvCentre"},
          "simulation": {"time": "2026-11-16 03:00"}}
    d = parking.trial_data(tmp_path, tc, log=logs.append)
    assert [(it[0], it[1]) for it in d["items"]] == [(0, 1.0), (1, 1.0)]     # the car and its glass, not the other
    assert "1 of 2 stalls" in logs[0] and "snap_scenery" in logs[1]           # glass slots are not stalls


class _Loc:
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Tf:
    def __init__(self, x, y, fwd, right):
        self.location, self._f, self._r = _Loc(x, y), fwd, right

    def get_forward_vector(self):
        return _Loc(*self._f)

    def get_right_vector(self):
        return _Loc(*self._r)


class _StraightRoad:
    """A road along +x from the origin, lanes 3 m wide: lane -1 right of it (+y in CARLA), lane 1 left (-y)."""

    def get_waypoint_xodr(self, road, lane, s):
        if lane == -1:
            return SimpleNamespace(transform=_Tf(s, 1.5, (1, 0), (0, 1)), lane_width=3.0)
        if lane == 1:
            return SimpleNamespace(transform=_Tf(s, -1.5, (-1, 0), (0, -1)), lane_width=3.0)
        return None


def test_xodr_crosswalks_and_signals(tmp_path):
    f = tmp_path / "map.xodr"
    f.write_text('''<OpenDRIVE><road id="7" length="50"><objects>
      <object id="c1" s="10" t="-2" hdg="1.570796327" type="crosswalk" width="4" length="7"><outline>
        <cornerLocal u="-3.5" v="-2"/><cornerLocal u="3.5" v="-2"/><cornerLocal u="3.5" v="2"/>
        <cornerLocal u="-3.5" v="2"/></outline></object></objects>
      <signals><signal id="s1" s="20" t="1" dynamic="yes" type="1000001"/>
        <signal id="s2" s="30" t="0" dynamic="no" type="205"/>
        <signal id="s3" s="40" t="-4" dynamic="yes" type="1000002"/></signals></road></OpenDRIVE>''')
    pts = parking._xodr_points(f, _StraightRoad())
    cw = pts["crossing"][0]
    # OpenDRIVE t > 0 is left of the road (-y in CARLA): t = -2 lies at y = +2; the crosswalk runs across the road
    assert abs(cw.centroid.x - 10) < 1e-6 and abs(cw.centroid.y - 2) < 1e-6
    x0, y0, x1, y1 = cw.bounds
    assert abs((x1 - x0) - 4) < 1e-6 and abs((y1 - y0) - 7) < 1e-6
    assert pts["signals"] == [(20.0, -1.0)]
    assert len(pts["crossing"]) == 2 and abs(pts["crossing"][1].centroid.y - 4) < 1e-6   # the pedestrian light
