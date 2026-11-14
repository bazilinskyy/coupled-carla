"""Traffic counts: hourly profile, street names, counter-to-edge mapping, routeSampler's count file and the
fetchers' parsing (no network, no SUMO)."""
import io
import json
import math
import xml.etree.ElementTree as ET
import zipfile

import pytest

from coupledcarla import counts as C


def test_profile_shares_and_day_factors():
    for day in C.DAYS:
        assert abs(sum(C.PROFILE[day]) - 1.0) < 1e-9 and len(C.PROFILE[day]) == 24
    week = 5 * C.DAY_FACTOR["weekday"] + C.DAY_FACTOR["saturday"] + C.DAY_FACTOR["sunday"]
    assert abs(week - 7.0) < 1e-9
    # weekday peaks in the morning and afternoon rush hours, the weekend around midday
    wd = C.PROFILE["weekday"]
    assert wd[8] > wd[11] and wd[17] > wd[13] and wd[3] < 0.01
    assert max(range(24), key=lambda h: C.PROFILE["sunday"][h]) in (12, 13)


def test_hourly_flow_from_daily_and_hourly_counts():
    aadt = {"daily": 10000, "daily_kind": "aadt"}
    assert C.hourly_flow(aadt, "weekday", 8) == pytest.approx(10000 * 1.05 * C.PROFILE["weekday"][8])
    workday = {"daily": 10500, "daily_kind": "weekday", "light_share": 0.9}
    assert C.hourly_flow(workday, "weekday", 8) == pytest.approx(0.9 * 10500 * C.PROFILE["weekday"][8])
    hourly = {"hourly": {"weekday": [None] * 8 + [1200.0] + [None] * 15}}
    assert C.hourly_flow(hourly, "weekday", 8) == 1200.0
    # a Sunday from weekday counts: carried over with the profile
    sun = C.hourly_flow(hourly, "sunday", 8)
    assert sun == pytest.approx(1200.0 * C.day_weight("sunday", 8) / C.day_weight("weekday", 8)) and sun < 600
    assert C.hourly_flow(hourly, "weekday", 3) is None
    assert C.hourly_flow({}, "weekday", 8) is None


def test_street_names():
    assert C.name_score("5 AVENUE", "5th Avenue") == 1.0
    assert C.name_score("WEST 35 STREET", "West 35th Street") == 1.0
    assert C.name_score("W 39 ST", "West 39th Street") == 1.0
    assert C.name_score("W 39 ST", "West 40th Street") == 0.0
    assert C.name_score("Gailleton", "Quai du Docteur Gailleton") == 1.0
    assert C.name_score("18 de Julio", "Avenida 18 de Julio") == 1.0
    assert C.name_score("Pt Wilson", "Pont Wilson") == 1.0
    assert C.name_score("Rio Branco", "Río Branco") == 1.0
    assert C.name_score("Street", "Avenue") == 0.0 and C.name_score(None, "A") == 0.0


def _edge(eid, pts, name=None, lanes=2, ref=None, cls="secondary"):
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    return {"id": eid, "shape": pts, "name": name, "ref": ref, "lanes": lanes, "length": length, "cls": cls}


@pytest.fixture
def grid_net():
    """Main Street (two-way, along x), Elm Street (one-way west, 30 m north), Oak Avenue (two-way, along y)."""
    return [
        _edge("main_e", [(0, -2), (200, -2)], "Main Street", ref="A4"),
        _edge("main_w", [(200, 2), (0, 2)], "Main Street", ref="A4"),
        _edge("elm_w", [(200, 30), (0, 30)], "Elm Street", lanes=1, cls="minor"),
        _edge("oak_n", [(102, -100), (102, 100)], "Oak Avenue", ref="A400"),
        _edge("oak_s", [(98, 100), (98, -100)], "Oak Avenue", ref="A400"),
    ]


def _xy(lon, lat):      # the tests use metres directly as "lon, lat"
    return lon, lat


def test_match_directed_undirected_and_cross_streets(grid_net):
    cs = [
        {"id": "east", "lon": 50, "lat": 0, "road": "Main St", "heading": 90.0},
        {"id": "west", "lon": 50, "lat": 0, "road": "Main St", "heading": 270.0},
        {"id": "two_way", "lon": 150, "lat": 1, "road": "MAIN STREET"},
        {"id": "one_way", "lon": 40, "lat": 29, "road": "Elm St"},
        {"id": "to_oak", "lon": 40, "lat": 0, "road": "Main Street", "from": "Birch", "to": "Oak Avenue"},
        {"id": "named", "lon": 60, "lat": 19, "road": "Main Street", "heading": 270.0},   # nearer to Elm
        {"id": "far", "lon": 50, "lat": 500, "road": "Main Street"},
        {"id": "north", "lon": 101, "lat": 60, "ref": "A400", "heading": 0.0},
        {"id": "wrong_ref", "lon": 109, "lat": 60, "ref": "A4", "heading": 0.0},   # 7 m off an A400
    ]
    got = {c["id"]: parts for c, parts, _ in C.match(cs, grid_net, _xy)}
    assert got["east"] == [("main_e", 1.0)] and got["west"] == [("main_w", 1.0)]
    assert sorted(got["two_way"]) == [("main_e", 0.5), ("main_w", 0.5)]
    assert got["one_way"] == [("elm_w", 1.0)]
    assert got["to_oak"] == [("main_e", 1.0)]
    assert got["named"] == [("main_w", 1.0)]
    assert got["north"] == [("oak_n", 1.0)]
    assert "far" not in got and "wrong_ref" not in got


def test_edge_flows_latest_year_and_split(grid_net):
    old = {"id": "a", "lon": 50, "lat": 0, "heading": 90.0, "daily": 1000, "daily_kind": "aadt", "year": 2015}
    new = dict(old, id="b", daily=2000, year=2024)
    both = {"id": "c", "lon": 150, "lat": 0, "road": "Main", "daily": 4000, "daily_kind": "aadt", "year": 2024}
    m = C.match([old, new, both], grid_net, _xy)
    f = C.edge_flows(m, "weekday", 8)
    w = C.day_weight("weekday", 8)
    assert f["main_e"] == pytest.approx((2000 * w + 2000 * w) / 2)     # the 2024 count and half the two-way one
    assert f["main_w"] == pytest.approx(2000 * w)


def test_write_edgedata(tmp_path):
    p = tmp_path / "counts.xml"
    C.write_edgedata(p, {"-12": 1200.0, "a&b": 90.0}, 0, 360)
    root = ET.parse(p).getroot()
    iv = root.find("interval")
    assert iv.get("begin") == "0" and iv.get("end") == "360"
    got = {e.get("id"): int(e.get("entered")) for e in iv.iter("edge")}
    assert got == {"-12": 120, "a&b": 9}


def test_geh():
    assert C.geh(100, 100) == 0.0 and C.geh(0, 0) == 0.0
    assert C.geh(150, 100) == pytest.approx(math.sqrt(20))


def test_label_from_osm_edges(grid_net):
    edges = [dict(e, name=None, ref=None, cls="minor") for e in grid_net]
    osm = [{"id": "w1", "shape": [(0, 0), (200, 0)], "name": "Main Street", "ref": "A4", "cls": "primary"},
           {"id": "w2", "shape": [(200, 0), (0, 0)], "name": "Main Street", "ref": "A4", "cls": "primary"},
           {"id": "w3", "shape": [(100, -100), (100, 100)], "name": "Oak Avenue", "ref": None, "cls": "tertiary"}]
    C.label(edges, osm)
    by = {e["id"]: e for e in edges}
    assert by["main_e"]["name"] == "Main Street" and by["main_e"]["cls"] == "primary"
    assert by["oak_n"]["name"] == "Oak Avenue" and by["oak_n"]["cls"] == "tertiary"
    assert by["oak_s"]["name"] is None                                                   # no OSM edge runs south
    assert by["elm_w"]["name"] is None                                                   # 30 m off: not labelled


def test_road_class():
    assert C.road_class("highway.trunk_link") == "primary" and C.road_class("highway.secondary") == "secondary"
    assert C.road_class("highway.residential") == "minor" and C.road_class(None) == "minor"


def test_auto_cars_per_hour_scales_with_lane_km(grid_net):
    bbox = [51.0, 5.0, 51.0126, 5.02]               # about 1.4 km square
    km = C.lane_km(grid_net)
    assert km["secondary"] == pytest.approx((200 * 2 * 2 + 200 * 2 * 2) / 1000)
    a = C.auto_cars_per_hour(grid_net, bbox, "weekday", 8)
    b = C.auto_cars_per_hour(grid_net + [dict(e, id=e["id"] + "x") for e in grid_net], bbox, "weekday", 8)
    assert b == pytest.approx(2 * a) and C.auto_cars_per_hour(grid_net, bbox, "weekday", 3) < a / 5
    assert C.auto_cars_per_hour(grid_net * 400, bbox, "weekday", 8) == C.AUTO_MAX_PER_HOUR


def test_load_caches_and_falls_back(tmp_path, monkeypatch):
    bbox = [10.0, 10.0, 10.01, 10.01]
    calls = []

    def fetch(b):
        calls.append(b)
        return [{"id": "x", "lat": 10.005, "lon": 10.005, "daily": 5000, "daily_kind": "aadt"}]
    monkeypatch.setattr(C, "FETCHERS", [("dft", lambda b: True, fetch)])
    d = C.load(tmp_path, bbox)
    assert d["source"] == "dft" and d["counters"][0]["id"] == "x" and (tmp_path / "real" / "counts.json").exists()
    assert C.load(tmp_path, bbox)["counters"] and len(calls) == 1          # cached

    def broken(b):
        raise OSError("offline")
    monkeypatch.setattr(C, "FETCHERS", [("dft", lambda b: True, broken)])
    assert C.load(tmp_path / "other", bbox) is None                        # fails soft
    monkeypatch.setattr(C, "FETCHERS", [("dft", lambda b: False, broken)])
    assert C.load(tmp_path / "none", bbox) is None                          # no source covers the map


class _Resp:
    def __init__(self, data=None, content=b""):
        self.data, self.content = data, content

    def json(self):
        return self.data


def test_lyon_parser_merges_lanes_and_skips_turns_and_bikes(monkeypatch):
    def feat(nom, lon, lat, kind="Comptage tous véhicules", mjo=12000):
        return {"geometry": {"type": "Point", "coordinates": [lon, lat]},
                "properties": {"nom": nom, "typepostemesure": kind, "moyennejoursouvrable": mjo,
                               "debithorairemax": 900, "horairedebitmax": "08h00", "anneereference": 2024}}
    data = {"features": [feat("160_Gailleton_Ainay_Conde", 4.8328, 45.7518),
                         feat("160_Gailleton_Ainay_Conde", 4.8330, 45.7520),
                         feat("216_Pt Wilson_Augagneur_Courmont Tà G", 4.84, 45.76),
                         feat("185_Pt Universite_Bernard_Gailleton TG", 4.835, 45.752),
                         feat("657_Velo_Belleco_OE", 4.83, 45.758, kind="Comptage vélo")]}
    monkeypatch.setattr(C, "_get", lambda *a, **k: _Resp(data))
    (c,) = C._lyon([45.75, 4.82, 45.77, 4.85])
    assert c["road"] == "Gailleton" and c["from"] == "Ainay" and c["to"] == "Conde"
    assert c["daily"] == 12000 and c["daily_kind"] == "weekday" and c["lon"] == pytest.approx(4.8329)


def test_montevideo_parser_sums_lanes_into_whole_hours(monkeypatch):
    rows = ["cod_detector,id_carril,fecha,hora,dsc_avenida,dsc_int_anterior,dsc_int_siguiente,latitud,longitud,"
            "volumen_hora"]
    for det, lane in (("2001601", 1), ("2001602", 2)):
        for m in range(0, 60, 5):
            rows.append(f"{det},{lane},2026-11-03,08:{m:02d}:00,Libertador,Mercedes,Colonia,-34.9044,-56.1937,10")
        rows.append(f"{det},{lane},2026-11-03,09:00:00,Libertador,Mercedes,Colonia,-34.9044,-56.1937,10")  # partial
    rows.append("9999901,1,2026-11-03,08:00:00,Burgues,A,B,-34.85,-56.18,99")                        # outside
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("Conteo_Noviembre_2026.csv", "\n".join(rows) + "\n")
    meta = {"result": {"resources": [{"url": "https://ckan-data.montevideo.gub.uy/x/conteo_noviembre_2026.zip",
                                      "created": "2026-11-10T00:00:00"}]}}
    monkeypatch.setattr(C, "_get", lambda url, *a, **k: _Resp(meta) if "package_show" in url else
                        _Resp(content=buf.getvalue()))
    (c,) = C._montevideo([-34.9127, -56.2068, -34.9001, -56.1912])
    assert c["road"] == "Libertador" and c["from"] == "Mercedes" and c["to"] == "Colonia" and c["lanes"] == 2
    assert c["hourly"]["weekday"][8] == 240.0 and c["hourly"]["weekday"][9] is None     # 2 lanes x 12 x 10


def test_vtrans_prefers_actual_counts(monkeypatch):
    def f(oid, year, desc, aadt):
        return {"attributes": {"OBJECTID": oid, "LOCAL_ID": "D077", "AADT": aadt, "AADT_YEAR": year,
                               "DESCRIPTION": desc, "ON_ROAD": "N Union St", "LATITUDE": 44.48, "LONGITUDE": -73.21}}
    feats = [f(1, 2025, "Grown using Growth Factors", 3000), f(2, 2020, "Actual (seasonally adjusted)", 2771)]
    monkeypatch.setattr(C, "_arcgis", lambda url, bbox: feats)
    (c,) = C._vtrans([44.47, -73.22, 44.49, -73.20])
    assert c["daily"] == 2771 and c["year"] == 2020 and not c["estimated"]


def test_copy_vtypes(tmp_path):
    cand = tmp_path / "cand.rou.xml"
    cand.write_text('<routes>\n    <vType id="vehicle.audi.a2" vClass="passenger"/>\n'
                    '    <vehicle id="cand0" type="vehicle.audi.a2" depart="0"><route edges="a"/></vehicle>\n'
                    '    <vType id="vehicle.seat.leon" vClass="passenger"/>\n</routes>\n')
    rou = tmp_path / "car.rou.xml"
    rou.write_text('<routes xmlns:xsi="x">\n    <vehicle id="cand0" type="vehicle.audi.a2" depart="3">'
                   '<route edges="a"/></vehicle>\n</routes>\n')
    C._copy_vtypes(cand, rou)
    root = ET.parse(rou).getroot()
    assert [t.get("id") for t in root.iter("vType")] == ["vehicle.audi.a2", "vehicle.seat.leon"]
    assert list(root)[0].tag == "vType"
